use glam::Vec2;

/// Univector Field (UVF / Campos Potenciais Univetoriais).
///
/// Calcula el ángulo de heading deseado en cualquier punto del campo combinando:
///   - Atracción hacia el target (campo atractivo, vector unitario)
///   - Deflexión tangencial alrededor de obstáculos (evita mínimos locales del APF clásico)
///
/// La deflexión es tangencial, no radial: el robot rodea los obstáculos en lugar de
/// frenar contra ellos. Este es el estándar en VSS/VSSS brasileño (ITAndroids, Rinobot, PEQUI-MEC).
///
/// # Uso
/// ```text
/// let θ = uvf.compute(robot_pos, target, &obstacles);
/// // θ → ángulo en [-π, π] hacia donde debe apuntar el robot
/// ```
pub struct UniVectorField {
    /// Radio de influencia de cada obstáculo (m). Fuera de este radio: sin deflexión.
    pub influence_radius: f32,
    /// Ganancia repulsiva. Escala la fuerza tangencial. 1.5 = deflexión moderada.
    pub k_rep: f32,
}

impl UniVectorField {
    /// Crea un UVF con parámetros calibrados para VSS 3v3 (campo 1.5m × 1.3m).
    pub fn new() -> Self {
        Self {
            influence_radius: 0.20, // 2.5× diámetro de robot (~8cm); deflexión suave a 20cm
            k_rep: 1.5,
        }
    }

    /// Calcula el ángulo de heading deseado [rad] para ir desde `robot_pos` hacia `target`,
    /// deflectando alrededor de `obstacles`.
    ///
    /// # Algoritmo
    /// 1. Vector atractivo: dirección unitaria hacia target.
    /// 2. Por cada obstáculo dentro de `influence_radius`:
    ///    - Calcular peso `w = ((r0 - d) / r0)²` — cae a 0 en el borde de influencia.
    ///    - Deflexión tangencial: rotar el vector radial ±90° eligiendo el lado que
    ///      más se alinea con el target (avance > retroceso).
    /// 3. Sumar vectores en espacio cartesiano (evita discontinuidades angulares).
    /// 4. `atan2` del vector resultante = ángulo deseado.
    pub fn compute(&self, robot_pos: Vec2, target: Vec2, obstacles: &[Vec2]) -> f32 {
        let to_target = target - robot_pos;
        let dist_target = to_target.length();

        // En el target mismo: mantener heading actual (0 como fallback)
        if dist_target < f32::EPSILON {
            return 0.0;
        }

        // Campo atractivo: vector unitario hacia target
        let f_att = to_target / dist_target;
        let mut f_total = f_att;

        for obs in obstacles {
            let to_robot = robot_pos - *obs;
            let d = to_robot.length();

            if d < f32::EPSILON || d >= self.influence_radius {
                continue;
            }

            // Peso cuadrático: fuerte cerca del obstáculo, suave en el borde
            let w = {
                let t = (self.influence_radius - d) / self.influence_radius;
                t * t
            };

            // Vector radial (aléjate del obstáculo)
            let radial = to_robot / d;

            // Las dos opciones tangenciales (±90° del radial)
            let tang_cw = Vec2::new(radial.y, -radial.x);
            let tang_ccw = Vec2::new(-radial.y, radial.x);

            // Elegir la tangencial que más avanza hacia el goal
            let tang = if f_att.dot(tang_cw) >= f_att.dot(tang_ccw) {
                tang_cw
            } else {
                tang_ccw
            };

            // Acumular deflexión tangencial ponderada
            f_total += tang * (w * self.k_rep);
        }

        f32::atan2(f_total.y, f_total.x)
    }

    /// Error de heading entre el ángulo UVF y la orientación actual del robot.
    /// Devuelve el error en [-π, π].
    pub fn heading_error(uvf_angle: f32, robot_orientation: f64) -> f64 {
        let err = uvf_angle as f64 - robot_orientation;
        // Normalizar a [-π, π]
        let pi = std::f64::consts::PI;
        let two_pi = 2.0 * pi;
        let mut e = err % two_pi;
        if e > pi {
            e -= two_pi;
        } else if e < -pi {
            e += two_pi;
        }
        e
    }
}

impl Default for UniVectorField {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uvf_direct_path_no_obstacles() {
        let uvf = UniVectorField::new();
        let angle = uvf.compute(Vec2::ZERO, Vec2::new(1.0, 0.0), &[]);
        assert!(
            (angle - 0.0).abs() < 0.01,
            "sin obstáculos debe apuntar directo al target"
        );
    }

    #[test]
    fn test_uvf_deflects_around_obstacle() {
        let uvf = UniVectorField::new();
        // Obstáculo a 0.10m del robot (bien dentro del radio de influencia 0.20m)
        // Robot en (0,0), target en (1,0), obstáculo en (0.10, 0.0)
        let angle = uvf.compute(Vec2::ZERO, Vec2::new(1.0, 0.0), &[Vec2::new(0.10, 0.0)]);
        // El ángulo debe desviarse de 0 (el UVF deflecta tangencialmente)
        assert!(
            angle.abs() > 0.05,
            "debe deflectar alrededor del obstáculo, angle={:.3}",
            angle
        );
    }

    #[test]
    fn test_uvf_no_deflection_outside_influence() {
        let uvf = UniVectorField::new();
        // Obstáculo muy lejos: sin deflexión
        let angle = uvf.compute(Vec2::ZERO, Vec2::new(1.0, 0.0), &[Vec2::new(5.0, 5.0)]);
        assert!(
            (angle - 0.0).abs() < 0.01,
            "obstáculo fuera de radio no debe deflectar"
        );
    }

    #[test]
    fn test_uvf_heading_error_normalization() {
        // Error cruzando ±π debe normalizarse correctamente
        let err =
            UniVectorField::heading_error(-std::f32::consts::PI + 0.1, std::f64::consts::PI - 0.1);
        assert!(
            err.abs() < std::f64::consts::PI,
            "error debe estar en [-π, π], got {:.3}",
            err
        );
    }
}
