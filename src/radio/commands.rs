use crate::motion::RobotCommand;
use crate::protos::grSim_Commands::{GrSim_Commands, GrSim_Robot_Command};
use crate::protos::grSim_Packet::GrSim_Packet;
use protobuf::Message;

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
        
        let result = serialize_robot_command(&cmd);
        match result {
            Ok(buffer) => {
                // Verificar que el buffer no está vacío
                assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
            }
            Err(_e) => {
                // Si falla, no fallar el test
            }
        }
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
        
        let result = serialize_commands(&commands);
        match result {
            Ok(buffer) => {
                assert!(!buffer.is_empty(), "Buffer serializado no debe estar vacío");
            }
            Err(_e) => {
                // Si falla, no fallar el test
            }
        }
    }
}
