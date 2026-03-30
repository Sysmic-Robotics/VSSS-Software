use crate::motion::RobotCommand;
use crate::protos::grSim_Commands::{GrSim_Commands, GrSim_Robot_Command};
use crate::protos::grSim_Packet::GrSim_Packet;
use crate::protos::ssl_simulation_robot_control::{
    RobotControl, RobotCommand as SSLRobotCommand, RobotMoveCommand, MoveGlobalVelocity
};
use crate::protos::fira_command::{Command as FiraCommand, Commands as FiraCommands};
use crate::protos::fira_packet::Packet as FiraPacket;
use protobuf::Message;
use std::sync::atomic::{AtomicU64, Ordering};

fn sanitize_and_clamp(value: f64, min: f64, max: f64) -> f64 {
    if !value.is_finite() {
        return 0.0;
    }
    value.clamp(min, max)
}

/// Serializa un RobotCommand a formato Protobuf de grSim
pub fn serialize_robot_command(cmd: &RobotCommand) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
    let mut robot_cmd = GrSim_Robot_Command::new();
    
    robot_cmd.set_id(cmd.id as u32);
    
    // Conversión de coordenadas globales a locales del robot
    let vx = cmd.motion.vx as f32;
    let vy = cmd.motion.vy as f32;
    let theta = cmd.motion.orientation as f32;
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();
    let veltangent = vx * cos_theta + vy * sin_theta;
    let velnormal = -vx * sin_theta + vy * cos_theta;
    
    robot_cmd.set_veltangent(veltangent);
    robot_cmd.set_velnormal(velnormal);
    robot_cmd.set_velangular(cmd.motion.omega as f32);
    
    // Comandos de kicker
    robot_cmd.set_kickspeedx(if cmd.kicker.kick_x { 3.0 } else { 0.0 });
    robot_cmd.set_kickspeedz(if cmd.kicker.kick_z { 3.0 } else { 0.0 });
    robot_cmd.set_spinner(cmd.kicker.dribbler > 0.0);
    
    // Campos requeridos adicionales
    robot_cmd.set_wheelsspeed(false); // No usar control de ruedas individuales
    
    let mut grsim_cmd = GrSim_Commands::new();
    // Usar timestamp real (grSim puede requerir un timestamp válido)
    use std::time::{SystemTime, UNIX_EPOCH};
    let timestamp = SystemTime::now().duration_since(UNIX_EPOCH)
        .unwrap_or_default().as_secs_f64();
    grsim_cmd.set_timestamp(timestamp);
    grsim_cmd.set_isteamyellow(cmd.team == 1); // true si es amarillo, false si es azul
    grsim_cmd.robot_commands.push(robot_cmd);
    
    let mut buffer = Vec::new();
    grsim_cmd.write_to_vec(&mut buffer)?;
    
    Ok(buffer)
}

/// Serializa múltiples comandos en un solo mensaje
/// Nota: Todos los comandos deben ser del mismo equipo
pub fn serialize_commands(commands: &[RobotCommand]) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
    if commands.is_empty() {
        return Err("No hay comandos para serializar".into());
    }
    
    // Todos los comandos deben ser del mismo equipo
    let team = commands[0].team;
    let is_yellow = team == 1;
    
    let mut grsim_cmd = GrSim_Commands::new();
    // Usar timestamp real (grSim puede requerir un timestamp válido)
    use std::time::{SystemTime, UNIX_EPOCH};
    let timestamp = SystemTime::now().duration_since(UNIX_EPOCH)
        .unwrap_or_default().as_secs_f64();
    grsim_cmd.set_timestamp(timestamp);
    grsim_cmd.set_isteamyellow(is_yellow); // true si es amarillo, false si es azul
    
    for cmd in commands {
        // Verificar que todos los comandos sean del mismo equipo
        if cmd.team != team {
            return Err(format!("Todos los comandos deben ser del mismo equipo. Encontrado equipo {} y {}", team, cmd.team).into());
        }
        
        let mut robot_cmd = GrSim_Robot_Command::new();
        robot_cmd.set_id(cmd.id as u32);
        
        // IMPORTANTE: grSim usa velocidades LOCALES (relativas a la orientación del robot)
        // veltangent = velocidad hacia adelante (en dirección del robot)
        // velnormal = velocidad lateral (perpendicular al robot, positivo = izquierda)
        // Necesitamos convertir de velocidades globales (vx, vy) a locales usando la orientación
        
        let vx = cmd.motion.vx as f32;
        let vy = cmd.motion.vy as f32;
        let theta = cmd.motion.orientation as f32;
        
        // Conversión de coordenadas globales a locales del robot
        // veltangent = vx*cos(theta) + vy*sin(theta)  (hacia adelante)
        // velnormal = -vx*sin(theta) + vy*cos(theta)  (lateral, positivo = izquierda)
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        let mut veltangent = vx * cos_theta + vy * sin_theta;
        let mut velnormal = -vx * sin_theta + vy * cos_theta;
        
        // Limitar velocidades a rangos válidos para grSim (típicamente -2.0 a 2.0 m/s)
        let max_vel = 2.0f32;
        veltangent = veltangent.max(-max_vel).min(max_vel);
        velnormal = velnormal.max(-max_vel).min(max_vel);
        
        robot_cmd.set_veltangent(veltangent);
        robot_cmd.set_velnormal(velnormal);
        
        // Limitar velocidad angular (típicamente -10 a 10 rad/s para grSim)
        let omega_limited = (cmd.motion.omega as f32).max(-10.0).min(10.0);
        robot_cmd.set_velangular(omega_limited);
        
        robot_cmd.set_kickspeedx(if cmd.kicker.kick_x { 3.0 } else { 0.0 });
        robot_cmd.set_kickspeedz(if cmd.kicker.kick_z { 3.0 } else { 0.0 });
        robot_cmd.set_spinner(cmd.kicker.dribbler > 0.0);
        robot_cmd.set_wheelsspeed(false); // No usar control de ruedas individuales
        
        grsim_cmd.robot_commands.push(robot_cmd);
    }
    
    // Verificar que el mensaje está inicializado correctamente
    if !grsim_cmd.has_timestamp() || !grsim_cmd.has_isteamyellow() {
        return Err("Mensaje grSim incompleto: faltan campos requeridos".into());
    }
    
    if grsim_cmd.robot_commands.is_empty() {
        return Err("No hay comandos de robot en el mensaje".into());
    }
    
    // Verificar que todos los comandos de robot tienen campos requeridos
    for (i, robot_cmd) in grsim_cmd.robot_commands.iter().enumerate() {
        if !robot_cmd.has_id() || !robot_cmd.has_veltangent() || !robot_cmd.has_velnormal() || 
           !robot_cmd.has_velangular() || !robot_cmd.has_kickspeedx() || !robot_cmd.has_kickspeedz() ||
           !robot_cmd.has_spinner() || !robot_cmd.has_wheelsspeed() {
            return Err(format!("Robot command {} está incompleto: faltan campos requeridos", i).into());
        }
    }
    
    // Verificar que el mensaje está completamente inicializado antes de serializar
    // Esto es crítico para protobuf - todos los campos required deben estar presentes
    if !grsim_cmd.has_timestamp() {
        return Err("Falta timestamp en grSim_Commands".into());
    }
    if !grsim_cmd.has_isteamyellow() {
        return Err("Falta isteamyellow en grSim_Commands".into());
    }
    
    // Verificar cada comando de robot
    for (i, robot_cmd) in grsim_cmd.robot_commands.iter().enumerate() {
        if !robot_cmd.has_id() {
            return Err(format!("Robot {}: falta id", i).into());
        }
        if !robot_cmd.has_veltangent() {
            return Err(format!("Robot {}: falta veltangent", i).into());
        }
        if !robot_cmd.has_velnormal() {
            return Err(format!("Robot {}: falta velnormal", i).into());
        }
        if !robot_cmd.has_velangular() {
            return Err(format!("Robot {}: falta velangular", i).into());
        }
        if !robot_cmd.has_kickspeedx() {
            return Err(format!("Robot {}: falta kickspeedx", i).into());
        }
        if !robot_cmd.has_kickspeedz() {
            return Err(format!("Robot {}: falta kickspeedz", i).into());
        }
        if !robot_cmd.has_spinner() {
            return Err(format!("Robot {}: falta spinner", i).into());
        }
        if !robot_cmd.has_wheelsspeed() {
            return Err(format!("Robot {}: falta wheelsspeed", i).into());
        }
    }
    
    // IMPORTANTE: grSim espera un grSim_Packet que contiene grSim_Commands
    // No enviar directamente grSim_Commands
    let mut packet = GrSim_Packet::new();
    packet.commands = protobuf::MessageField::some(grsim_cmd);
    
    let mut buffer = Vec::new();
    
    match packet.write_to_vec(&mut buffer) {
        Ok(_) => Ok(buffer),
        Err(e) => Err(format!("Error serializando packet grSim: {}", e).into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[test]
    fn test_serialize_robot_command() {
        let cmd = RobotCommand {
            id: 0,
            team: 0,
            motion: MotionCommand {
                id: 0,
                team: 0,
                vx: 1.0,
                vy: 0.5,
                omega: 0.1,
                orientation: 0.0,
            },
            kicker: KickerCommand {
                id: 0,
                team: 0,
                kick_x: true,
                kick_z: false,
                dribbler: 5.0,
            },
        };
        
        let buffer = serialize_robot_command(&cmd)
            .expect("serialización de un comando grSim debe ser exitosa");
        assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
    }
    
    #[test]
    fn test_serialize_commands() {
        let commands = vec![
            RobotCommand {
                id: 0,
                team: 0,
                motion: MotionCommand {
                    id: 0,
                    team: 0,
                    vx: 1.0,
                    vy: 0.0,
                    omega: 0.0,
                    orientation: 0.0,
                },
                kicker: KickerCommand {
                    id: 0,
                    team: 0,
                    kick_x: false,
                    kick_z: false,
                    dribbler: 0.0,
                },
            },
        ];
        
        let buffer = serialize_commands(&commands)
            .expect("serialización de comandos grSim debe ser exitosa");
        assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
    }
}

/// Serializa comandos a formato SSL-Simulation (FIRASim)
/// 
/// Convierte comandos internos RobotCommand a formato RobotControl de SSL-Simulation.
/// Usa MoveGlobalVelocity para velocidades globales directamente (sin conversión a locales).
/// 
/// # Argumentos
/// * `commands` - Vector de comandos de robots (pueden ser de diferentes equipos)
/// 
/// # Retorna
/// Buffer serializado con mensaje RobotControl listo para enviar a FIRASim
pub fn serialize_to_firasim(commands: &[RobotCommand]) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
    if commands.is_empty() {
        return Err("No hay comandos para serializar".into());
    }
    
    // Agrupar comandos por equipo - FIRASim podría necesitar comandos separados por equipo
    // O podríamos necesitar enviar todos juntos pero con IDs que incluyan información del equipo
    let mut robot_control = RobotControl::new();
    
    for cmd in commands {
        let mut ssl_cmd = SSLRobotCommand::new();
        
        // Establecer ID del robot
        // IMPORTANTE: En SSL-Simulation/FIRASim, el ID del robot es solo el número (0-2 para cada equipo)
        // El equipo NO se especifica en RobotCommand, parece que FIRASim identifica robots por ID solamente
        // o usa el lado del campo para determinar el equipo
        // 
        // Posible problema: FIRASim podría usar IDs globales donde:
        // - Azul: IDs 0-2
        // - Amarillo: IDs 3-5 (o viceversa)
        // Por ahora usamos el ID directo (0-2) y asumimos que FIRASim sabe qué equipo es
        ssl_cmd.set_id(cmd.id as u32);
        
        eprintln!("[serialize_to_firasim] Procesando comando: robot_id={}, team={} (nota: team no se envía en RobotCommand)", cmd.id, cmd.team);
        
        // Crear comando de movimiento con velocidades globales
        let mut move_cmd = RobotMoveCommand::new();
        let mut global_vel = MoveGlobalVelocity::new();
        
        // FIRASim usa velocidades globales directamente (no necesita conversión)
        // Limitar velocidades a rangos válidos (típicamente -2.0 a 2.0 m/s para VSS)
        let vx = sanitize_and_clamp(cmd.motion.vx, -MOTION_VEL_MAX, MOTION_VEL_MAX) as f32;
        let vy = sanitize_and_clamp(cmd.motion.vy, -MOTION_VEL_MAX, MOTION_VEL_MAX) as f32;
        let omega = sanitize_and_clamp(cmd.motion.omega, -MOTION_OMEGA_MAX, MOTION_OMEGA_MAX) as f32; // rad/s
        
        global_vel.set_x(vx);
        global_vel.set_y(vy);
        global_vel.set_angular(omega);
        
        move_cmd.set_global_velocity(global_vel);
        ssl_cmd.move_command = protobuf::MessageField::some(move_cmd);
        
        // En VSS no hay kick ni dribbler, así que NO establecemos estos campos
        // Dejarlos sin establecer (opcionales) para que FIRASim use valores por defecto
        // Solo establecer si realmente se necesita kick/dribbler
        if cmd.kicker.kick_x || cmd.kicker.kick_z || cmd.kicker.dribbler > 0.0 {
            // Solo establecer si hay algún comando de kicker/dribbler
            let kick_speed = if cmd.kicker.kick_x || cmd.kicker.kick_z {
                3.0f32 // m/s
            } else {
                0.0f32
            };
            let kick_angle = if cmd.kicker.kick_z {
                45.0f32
            } else {
                0.0f32
            };
            let dribbler_rpm = if cmd.kicker.dribbler > 0.0 {
                (cmd.kicker.dribbler * 1000.0) as f32
            } else {
                0.0f32
            };
            
            ssl_cmd.set_kick_speed(kick_speed);
            ssl_cmd.set_kick_angle(kick_angle);
            ssl_cmd.set_dribbler_speed(dribbler_rpm);
        }
        // Si no hay comandos de kicker/dribbler, dejar los campos sin establecer (opcionales)
        
        robot_control.robot_commands.push(ssl_cmd);
    }
    
    // Verificar que el mensaje está inicializado correctamente
    if robot_control.robot_commands.is_empty() {
        return Err("No hay comandos de robot en el mensaje RobotControl".into());
    }
    
    // Verificar que todos los comandos están inicializados
    for (i, ssl_cmd) in robot_control.robot_commands.iter().enumerate() {
        if !ssl_cmd.has_id() {
            return Err(format!("Robot command {}: falta id", i).into());
        }
        // Verificar que move_command está presente usando as_ref()
        if ssl_cmd.move_command.as_ref().is_none() {
            return Err(format!("Robot command {}: falta move_command", i).into());
        }
        // Verificar que move_command tiene global_velocity
        if let Some(ref move_cmd) = ssl_cmd.move_command.as_ref() {
            if !move_cmd.has_global_velocity() {
                return Err(format!("Robot command {}: falta global_velocity en move_command", i).into());
            }
        }
    }
    
    // Debug: mostrar estructura del mensaje antes de serializar (solo para primeros comandos)
    if robot_control.robot_commands.len() <= 3 {
        eprintln!("[serialize_to_firasim] RobotControl tiene {} comandos", robot_control.robot_commands.len());
        for (i, cmd) in robot_control.robot_commands.iter().enumerate() {
            eprintln!("[serialize_to_firasim] Comando {}: id={}, has_move_command={}, has_kick_speed={}, has_kick_angle={}, has_dribbler_speed={}", 
                     i, cmd.id(), cmd.move_command.as_ref().is_some(), cmd.has_kick_speed(), cmd.has_kick_angle(), cmd.has_dribbler_speed());
            if let Some(ref move_cmd) = cmd.move_command.as_ref() {
                if move_cmd.has_global_velocity() {
                    let gv = move_cmd.global_velocity();
                    eprintln!("[serialize_to_firasim]   global_velocity: x={:.3}, y={:.3}, angular={:.3}", gv.x(), gv.y(), gv.angular());
                }
            }
        }
    }
    
    // Serializar a buffer
    let mut buffer = Vec::new();
    match robot_control.write_to_vec(&mut buffer) {
        Ok(_) => {
            if buffer.len() <= 100 {
                eprintln!("[serialize_to_firasim] Mensaje serializado: {} bytes", buffer.len());
                // Mostrar primeros bytes en hex para debugging solo si el mensaje es pequeño
                eprintln!("[serialize_to_firasim] Primeros {} bytes (hex): {:02x?}", buffer.len(), &buffer[..]);
            }
            Ok(buffer)
        },
        Err(e) => Err(format!("Error serializando RobotControl para FIRASim: {}", e).into())
    }
}

/// Constantes del robot VSS para conversión v,omega → wheel_left/wheel_right (rad/s).
/// Ver RESPUESTAS_PUERTO_20011.md: L = separación entre ruedas, r = radio de rueda.
const WHEEL_BASE_L: f64 = 0.05;   // m, típico VSS ~0.05–0.06
const WHEEL_RADIUS_R: f64 = 0.02; // m, depende del modelo en FIRASim
/// Calculado para no clipear a MAX_LINEAR_SPEED=1.2 m/s:
///   wheel_needed = 1.2/0.02 + 3.0*0.05/(2*0.02) = 60 + 3.75 = 63.75 → 70 con margen.
/// El valor anterior (20.0) limitaba la velocidad efectiva a 0.4 m/s (3× por debajo).
const WHEEL_RAD_S_MAX: f64 = 70.0;
/// Límites de entrada esperados para comandos cinemáticos.
const MOTION_VEL_MAX: f64 = 2.0;   // m/s
const MOTION_OMEGA_MAX: f64 = 10.0; // rad/s

/// Serializa comandos al protocolo FIRA (VSSS/FIRASim) para el puerto 20011.
/// Convierte velocidades globales (vx, vy, omega) a wheel_left/wheel_right en rad/s.
/// Fórmula diferencial: omega_left = v/r - omega*L/(2*r), omega_right = v/r + omega*L/(2*r).
pub fn serialize_to_fira_actuator(commands: &[RobotCommand]) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
    if commands.is_empty() {
        return Err("No hay comandos para serializar (FIRA)".into());
    }

    static CLIPPED_WHEEL_COUNT: AtomicU64 = AtomicU64::new(0);
    static NON_FINITE_INPUT_COUNT: AtomicU64 = AtomicU64::new(0);

    let mut fira_commands = FiraCommands::new();
    for cmd in commands {
        let raw_theta = cmd.motion.orientation;
        let raw_vx = cmd.motion.vx;
        let raw_vy = cmd.motion.vy;
        let raw_omega = cmd.motion.omega;

        if !raw_theta.is_finite() || !raw_vx.is_finite() || !raw_vy.is_finite() || !raw_omega.is_finite() {
            let n = NON_FINITE_INPUT_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            if n <= 5 || n % 120 == 0 {
                eprintln!(
                    "[serialize_to_fira_actuator] entrada no finita (#{}) robot={} theta={} vx={} vy={} omega={}",
                    n, cmd.id, raw_theta, raw_vx, raw_vy, raw_omega
                );
            }
        }

        let theta = if raw_theta.is_finite() { raw_theta } else { 0.0 };
        let vx = sanitize_and_clamp(raw_vx, -MOTION_VEL_MAX, MOTION_VEL_MAX);
        let vy = sanitize_and_clamp(raw_vy, -MOTION_VEL_MAX, MOTION_VEL_MAX);
        let omega = sanitize_and_clamp(raw_omega, -MOTION_OMEGA_MAX, MOTION_OMEGA_MAX);

        // v = velocidad lineal en la dirección del robot (m/s)
        let v = vx * theta.cos() + vy * theta.sin();
        // omega_left/right en rad/s (drive diferencial: L = wheel base, r = wheel radius)
        let inv_r = 1.0 / WHEEL_RADIUS_R;
        let half_l_over_r = WHEEL_BASE_L / (2.0 * WHEEL_RADIUS_R);
        let mut wheel_left = v * inv_r - omega * half_l_over_r;
        let mut wheel_right = v * inv_r + omega * half_l_over_r;
        let unclamped_left = wheel_left;
        let unclamped_right = wheel_right;
        wheel_left = wheel_left.clamp(-WHEEL_RAD_S_MAX, WHEEL_RAD_S_MAX);
        wheel_right = wheel_right.clamp(-WHEEL_RAD_S_MAX, WHEEL_RAD_S_MAX);
        if (unclamped_left - wheel_left).abs() > f64::EPSILON || (unclamped_right - wheel_right).abs() > f64::EPSILON {
            let n = CLIPPED_WHEEL_COUNT.fetch_add(1, Ordering::Relaxed) + 1;
            if n <= 5 || n % 120 == 0 {
                eprintln!(
                    "[serialize_to_fira_actuator] saturación ruedas (#{}) robot={} left={:.3}->{:.3} right={:.3}->{:.3}",
                    n, cmd.id, unclamped_left, wheel_left, unclamped_right, wheel_right
                );
            }
        }

        let mut fira_cmd = FiraCommand::new();
        fira_cmd.id = cmd.id as u32;
        fira_cmd.yellowteam = cmd.team == 1;
        fira_cmd.wheel_left = wheel_left;
        fira_cmd.wheel_right = wheel_right;
        fira_commands.robot_commands.push(fira_cmd);
    }

    let mut packet = FiraPacket::new();
    packet.cmd = protobuf::MessageField::some(fira_commands);

    let mut buffer = Vec::new();
    packet.write_to_vec(&mut buffer)?;
    Ok(buffer)
}

#[cfg(test)]
mod firasim_tests {
    use super::*;
    use crate::motion::{MotionCommand, KickerCommand};
    
    #[test]
    fn test_serialize_to_firasim() {
        let commands = vec![
            RobotCommand {
                id: 0,
                team: 0,
                motion: MotionCommand {
                    id: 0,
                    team: 0,
                    vx: 1.0,
                    vy: 0.0,
                    omega: 0.0,
                    orientation: 0.0,
                },
                kicker: KickerCommand {
                    id: 0,
                    team: 0,
                    kick_x: false,
                    kick_z: false,
                    dribbler: 0.0,
                },
            },
        ];
        
        let buffer = serialize_to_firasim(&commands)
            .expect("serialización FIRASim debe ser exitosa");
        assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
    }
    
    #[test]
    fn test_serialize_to_firasim_with_kick() {
        let commands = vec![
            RobotCommand {
                id: 1,
                team: 0,
                motion: MotionCommand {
                    id: 1,
                    team: 0,
                    vx: 0.5,
                    vy: 0.5,
                    omega: 0.1,
                    orientation: 0.0,
                },
                kicker: KickerCommand {
                    id: 1,
                    team: 0,
                    kick_x: true,
                    kick_z: false,
                    dribbler: 5.0,
                },
            },
        ];
        
        let buffer = serialize_to_firasim(&commands)
            .expect("serialización FIRASim con kick debe ser exitosa");
        assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
    }
}
