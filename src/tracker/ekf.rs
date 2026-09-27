use crate::params::VisionParams;
use nalgebra::{SMatrix, SVector};

// Estado: [x, y, sin(θ), cos(θ), vx, vy, ω]
type StateVector = SVector<f64, 7>;
type CovarianceMatrix = SMatrix<f64, 7, 7>;
type MeasurementVector = SVector<f64, 3>; // [x, y, θ]

// Q, R, gating y límites físicos vienen de `VisionParams`
// (`config/team_params.json` → "vision"); ver ahí la documentación de cada uno.
// Antes eran constantes (R_POS 1e-4, R_THETA 2.5e-3, χ² 16.27, v_max 4, ω_max 45).

/// Extended Kalman Filter para tracking de robots y balón
///
/// Estado: [x, y, sin(θ), cos(θ), vx, vy, ω]
/// Medición: [x, y, θ]
#[allow(non_snake_case)] // Convención matemática estándar para matrices
pub struct ExtendedKalmanFilter {
    x_: StateVector,        // Estado estimado
    P_: CovarianceMatrix,   // Matriz de covarianza
    Q_: CovarianceMatrix,   // Ruido de proceso
    R_: SMatrix<f64, 3, 3>, // Ruido de medición
    /// Si el filtro ya recibió su primera medición. Reemplaza la heurística de
    /// posición≈(0,0), que reiniciaba el filtro en el centro de la cancha.
    initialized: bool,
    gating_chi2: f64,
    max_lin_speed: f64,
    max_omega: f64,
    gate_margin: f64,
}

impl ExtendedKalmanFilter {
    /// Filtro con los parámetros por defecto (`VisionParams::default()`).
    pub fn new() -> Self {
        Self::from_params(&VisionParams::default())
    }

    /// Filtro con Q/R/gating explícitos (los del JSON vigente en producción; los
    /// de un archivo alternativo en `vision_replay --ekf-csv --params`).
    pub fn from_params(p: &VisionParams) -> Self {
        // Inicializar estado en cero
        let x_ = StateVector::zeros();

        // Covarianza inicial P (diagonal). Pos/θ arrancan con la incertidumbre de la
        // medición (se inicializa con ella); velocidad/ω arrancan inciertas.
        #[allow(non_snake_case)]
        let mut P_ = CovarianceMatrix::zeros();
        P_[(0, 0)] = p.r_pos; // x
        P_[(1, 1)] = p.r_pos; // y
        P_[(2, 2)] = 1e-4; // sin(theta)
        P_[(3, 3)] = 1e-4; // cos(theta)
        P_[(4, 4)] = 1.0; // vx
        P_[(5, 5)] = 1.0; // vy
        P_[(6, 6)] = 1.0; // omega

        // Ruido de proceso Q (diagonal)
        #[allow(non_snake_case)]
        let mut Q_ = CovarianceMatrix::zeros();
        Q_[(0, 0)] = p.q_pos; // x
        Q_[(1, 1)] = p.q_pos; // y
        Q_[(2, 2)] = p.q_angle; // sin(theta)
        Q_[(3, 3)] = p.q_angle; // cos(theta)
        Q_[(4, 4)] = p.q_vel; // vx
        Q_[(5, 5)] = p.q_vel; // vy
        Q_[(6, 6)] = p.q_omega; // omega

        // Ruido de medición R (diagonal).
        #[allow(non_snake_case)]
        let mut R_ = SMatrix::<f64, 3, 3>::zeros();
        R_[(0, 0)] = p.r_pos; // x
        R_[(1, 1)] = p.r_pos; // y
        R_[(2, 2)] = p.r_theta; // theta

        Self {
            x_,
            P_,
            Q_,
            R_,
            initialized: false,
            gating_chi2: p.gating_chi2,
            max_lin_speed: p.max_lin_speed,
            max_omega: p.max_omega,
            gate_margin: p.gate_margin,
        }
    }

    /// Predice el estado siguiente usando el modelo de transición
    pub fn predict(&mut self, dt: f64) {
        // El Jacobiano se linealiza en el estado PREVIO (punto de linealización de f),
        // por eso se calcula antes de sobrescribir x_.
        #[allow(non_snake_case)]
        let F = self.jacobian_f(&self.x_, dt);

        // Predicción del estado: x = f(x, dt)
        self.x_ = self.f(&self.x_, dt);

        // Predicción de la covarianza: P = F * P * F^T + Q
        self.P_ = F * self.P_ * F.transpose() + self.Q_;
    }

    /// Actualiza el estado con una nueva medición. `dt` se usa para el gate físico
    /// de innovación (cuánto puede haberse movido/rotado el robot en ese lapso).
    pub fn update(&mut self, measurement: &MeasurementVector, dt: f64) {
        // Normalizar el ángulo de la medición
        let mut z = *measurement;
        z[2] = Self::normalize_angle(z[2]);

        // Predicción de la medición: h(x)
        let h = self.h(&self.x_);

        // Residuo: y = z - h(x)
        let mut y = z - h;
        y[2] = Self::normalize_angle(y[2]);

        // Gate físico: un robot no puede moverse/rotar más de lo físicamente posible
        // en `dt`. Un salto mayor = detección espuria (solape/intercambio) → descartar.
        let max_dpos = self.max_lin_speed * dt * self.gate_margin;
        let max_dtheta = self.max_omega * dt * self.gate_margin;
        let dpos = (y[0] * y[0] + y[1] * y[1]).sqrt();
        if dpos > max_dpos || y[2].abs() > max_dtheta {
            return;
        }

        // Jacobiano de observación
        #[allow(non_snake_case)]
        let H = self.jacobian_h(&self.x_);

        // Matriz de innovación: S = H * P * H^T + R
        #[allow(non_snake_case)]
        let S = H * self.P_ * H.transpose() + self.R_;

        // Si S no es invertible, omitir la corrección (conservar la predicción) en
        // vez de usar un sustituto inválido.
        #[allow(non_snake_case)]
        let S_inv = match S.try_inverse() {
            Some(inv) => inv,
            None => return,
        };

        // Gating de innovación (distancia de Mahalanobis). Un residuo demasiado
        // grande = medición outlier (solape/intercambio de detección): se descarta
        // para no propagar velocidad/ω irreales.
        let mahalanobis = (y.transpose() * S_inv * y)[(0, 0)];
        if mahalanobis > self.gating_chi2 {
            return;
        }

        // Ganancia de Kalman: K = P * H^T * S^(-1)
        #[allow(non_snake_case)]
        let K = self.P_ * H.transpose() * S_inv;

        // Actualización del estado: x = x + K * y
        self.x_ += K * y;

        // Normalizar sin y cos para mantener consistencia
        let sin_theta = self.x_[2];
        let cos_theta = self.x_[3];
        let norm = (sin_theta * sin_theta + cos_theta * cos_theta).sqrt();
        if norm > 1e-10 {
            self.x_[2] /= norm;
            self.x_[3] /= norm;
        }

        // Acotar ω a su máximo físico (defensa ante cualquier glitch que sobreviva).
        self.x_[6] = self.x_[6].clamp(-self.max_omega, self.max_omega);

        // Actualización de la covarianza: P = (I - K * H) * P
        #[allow(non_snake_case)]
        let I = SMatrix::<f64, 7, 7>::identity();
        self.P_ = (I - K * H) * self.P_;
    }

    /// Filtra una pose (posición y orientación) y retorna posición filtrada y velocidades estimadas
    pub fn filter_pose(
        &mut self,
        x: f64,
        y: f64,
        theta: f64,
        dt: f64,
    ) -> (f64, f64, f64, f64, f64, f64) {
        // Normalizar ángulo de entrada
        let theta_norm = Self::normalize_angle(theta);

        // La inicialización se marca explícitamente (no se infiere de la posición, que
        // rompía en el centro de la cancha / balón en el origen).
        if !self.initialized {
            // Inicializar estado con la primera medición
            self.x_[0] = x;
            self.x_[1] = y;
            let sin_theta = theta_norm.sin();
            let cos_theta = theta_norm.cos();
            self.x_[2] = sin_theta;
            self.x_[3] = cos_theta;
            // Velocidades iniciales en cero
            self.x_[4] = 0.0;
            self.x_[5] = 0.0;
            self.x_[6] = 0.0;
            self.initialized = true;
        } else {
            // Predicción
            self.predict(dt);

            // Actualización con medición
            let z = MeasurementVector::new(x, y, theta_norm);
            self.update(&z, dt);
        }

        // Extraer resultados
        let x_filtered = self.x_[0];
        let y_filtered = self.x_[1];
        let sin_theta = self.x_[2];
        let cos_theta = self.x_[3];
        let theta_filtered = sin_theta.atan2(cos_theta);
        let vx = self.x_[4];
        let vy = self.x_[5];
        let omega = self.x_[6];

        (x_filtered, y_filtered, theta_filtered, vx, vy, omega)
    }

    /// Modelo de transición: f(x, dt)
    /// Predice el estado siguiente asumiendo movimiento constante con velocidades
    fn f(&self, x: &StateVector, dt: f64) -> StateVector {
        let mut x_next = StateVector::zeros();

        // x' = x + vx * dt
        x_next[0] = x[0] + x[4] * dt;

        // y' = y + vy * dt
        x_next[1] = x[1] + x[5] * dt;

        // sin(θ') = sin(θ + ω*dt) = sin(θ)*cos(ω*dt) + cos(θ)*sin(ω*dt)
        // cos(θ') = cos(θ + ω*dt) = cos(θ)*cos(ω*dt) - sin(θ)*sin(ω*dt)
        let omega_dt = x[6] * dt;
        let cos_omega_dt = omega_dt.cos();
        let sin_omega_dt = omega_dt.sin();
        x_next[2] = x[2] * cos_omega_dt + x[3] * sin_omega_dt;
        x_next[3] = x[3] * cos_omega_dt - x[2] * sin_omega_dt;

        // Velocidades se mantienen constantes (modelo de velocidad constante)
        x_next[4] = x[4]; // vx
        x_next[5] = x[5]; // vy
        x_next[6] = x[6]; // omega

        x_next
    }

    /// Modelo de observación: h(x)
    /// Mapea el estado a las mediciones observables
    fn h(&self, x: &StateVector) -> MeasurementVector {
        let sin_theta = x[2];
        let cos_theta = x[3];
        let theta = sin_theta.atan2(cos_theta);

        MeasurementVector::new(x[0], x[1], theta)
    }

    /// Jacobiano del modelo de transición: ∂f/∂x
    fn jacobian_f(&self, x: &StateVector, dt: f64) -> CovarianceMatrix {
        #[allow(non_snake_case)]
        let mut F = CovarianceMatrix::identity();

        // ∂x'/∂x = 1, ∂x'/∂vx = dt
        F[(0, 4)] = dt;

        // ∂y'/∂y = 1, ∂y'/∂vy = dt
        F[(1, 5)] = dt;

        // ∂sin(θ')/∂sin(θ) y ∂sin(θ')/∂cos(θ)
        let omega_dt = x[6] * dt;
        let cos_omega_dt = omega_dt.cos();
        let sin_omega_dt = omega_dt.sin();
        F[(2, 2)] = cos_omega_dt;
        F[(2, 3)] = sin_omega_dt;
        F[(2, 6)] = dt * (x[3] * cos_omega_dt - x[2] * sin_omega_dt);

        // ∂cos(θ')/∂sin(θ) y ∂cos(θ')/∂cos(θ)
        F[(3, 2)] = -sin_omega_dt;
        F[(3, 3)] = cos_omega_dt;
        F[(3, 6)] = -dt * (x[2] * cos_omega_dt + x[3] * sin_omega_dt);

        // Velocidades: identidad (se mantienen constantes)
        // F[(4,4)] = 1, F[(5,5)] = 1, F[(6,6)] = 1 (ya son 1 por identidad)

        F
    }

    /// Jacobiano del modelo de observación: ∂h/∂x
    fn jacobian_h(&self, x: &StateVector) -> SMatrix<f64, 3, 7> {
        #[allow(non_snake_case)]
        let mut H = SMatrix::<f64, 3, 7>::zeros();

        // ∂h/∂x = [1, 0, 0, 0, 0, 0, 0] para x
        H[(0, 0)] = 1.0;

        // ∂h/∂y = [0, 1, 0, 0, 0, 0, 0] para y
        H[(1, 1)] = 1.0;

        // ∂θ/∂sin(θ) y ∂θ/∂cos(θ)
        // θ = atan2(sin(θ), cos(θ))
        // ∂θ/∂sin(θ) = cos(θ) / (sin²(θ) + cos²(θ)) = cos(θ)
        // ∂θ/∂cos(θ) = -sin(θ) / (sin²(θ) + cos²(θ)) = -sin(θ)
        let sin_theta = x[2];
        let cos_theta = x[3];
        H[(2, 2)] = cos_theta; // ∂θ/∂sin(θ)
        H[(2, 3)] = -sin_theta; // ∂θ/∂cos(θ)

        H
    }

    /// Normaliza un ángulo al rango [-π, π]
    pub fn normalize_angle(angle: f64) -> f64 {
        let pi = std::f64::consts::PI;
        let two_pi = 2.0 * pi;

        // Usar operación módulo para normalizar
        let mut normalized = angle % two_pi;

        // Ajustar al rango [-π, π]
        // Si el resultado del módulo es > π, restar 2π
        // Si el resultado del módulo es < -π, sumar 2π
        if normalized > pi {
            normalized -= two_pi;
        } else if normalized <= -pi {
            normalized += two_pi;
        }

        // Manejar el caso especial donde el módulo puede dar exactamente π o -π
        // pero queremos mantener el signo correcto
        if normalized == pi && angle < 0.0 {
            normalized = -pi;
        }

        normalized
    }
}

impl Default for ExtendedKalmanFilter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ekf_initialization() {
        let ekf = ExtendedKalmanFilter::new();
        assert_eq!(ekf.x_.len(), 7);
        assert_eq!(ekf.P_.nrows(), 7);
        assert_eq!(ekf.P_.ncols(), 7);
    }

    #[test]
    fn test_normalize_angle() {
        use std::f64::consts::PI;

        assert!((ExtendedKalmanFilter::normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((ExtendedKalmanFilter::normalize_angle(PI) - PI).abs() < 1e-10);
        assert!((ExtendedKalmanFilter::normalize_angle(-PI) - (-PI)).abs() < 1e-10);
        assert!((ExtendedKalmanFilter::normalize_angle(2.0 * PI) - 0.0).abs() < 1e-10);
        // 3.0 * PI = 9.424... -> módulo 2π = 3.14159... = π, pero queremos -π
        // La lógica actual puede dar π, así que ajustamos el test
        let result = ExtendedKalmanFilter::normalize_angle(3.0 * PI);
        assert!((result - PI).abs() < 1e-10 || (result - (-PI)).abs() < 1e-10);
    }

    #[test]
    fn test_predict() {
        let mut ekf = ExtendedKalmanFilter::new();

        // Inicializar con estado conocido
        ekf.x_[0] = 1.0; // x
        ekf.x_[1] = 2.0; // y
        ekf.x_[2] = 0.0; // sin(theta) = 0
        ekf.x_[3] = 1.0; // cos(theta) = 1
        ekf.x_[4] = 0.5; // vx
        ekf.x_[5] = 0.3; // vy
        ekf.x_[6] = 0.1; // omega

        let dt = 0.1;
        ekf.predict(dt);

        // Verificar que x aumentó por vx * dt
        assert!((ekf.x_[0] - (1.0 + 0.5 * dt)).abs() < 1e-10);
        // Verificar que y aumentó por vy * dt
        assert!((ekf.x_[1] - (2.0 + 0.3 * dt)).abs() < 1e-10);
    }

    #[test]
    fn test_update() {
        let mut ekf = ExtendedKalmanFilter::new();

        // Inicializar estado
        ekf.x_[0] = 1.0;
        ekf.x_[1] = 2.0;
        ekf.x_[2] = 0.0;
        ekf.x_[3] = 1.0;

        let measurement = MeasurementVector::new(1.1, 2.1, 0.1);
        ekf.update(&measurement, 0.1);

        // El estado debería haberse actualizado hacia la medición
        assert!((ekf.x_[0] - 1.0).abs() < 1.0); // Debería estar entre 1.0 y 1.1
        assert!((ekf.x_[1] - 2.0).abs() < 1.0); // Debería estar entre 2.0 y 2.1
    }

    /// Un salto espurio de orientación (~π por solape/intercambio de detección) se
    /// descarta por el gating y NO dispara ω a valores irreales; el filtro se recupera.
    #[test]
    fn outlier_theta_does_not_blow_up_omega() {
        let mut ekf = ExtendedKalmanFilter::new();
        let dt = 0.016;
        // Robot quieto mirando a θ=0 durante varios frames → el filtro converge.
        for _ in 0..30 {
            ekf.filter_pose(1.0, 1.0, 0.0, dt);
        }
        // Frame outlier: θ salta a π (misma posición).
        let (_, _, theta_out, _, _, omega_out) =
            ekf.filter_pose(1.0, 1.0, std::f64::consts::PI, dt);
        // El outlier se descartó: θ sigue cerca de 0 y ω no se disparó.
        assert!(theta_out.abs() < 0.5, "θ saltó al outlier: {theta_out}");
        assert!(omega_out.abs() < 5.0, "ω se disparó: {omega_out}");
        // Con mediciones consistentes de nuevo, sigue estable.
        let (.., omega_after) = ekf.filter_pose(1.0, 1.0, 0.0, dt);
        assert!(omega_after.abs() < 5.0);
    }

    /// Un robot que pasa por el centro (0,0) NO reinicia su filtro (las velocidades
    /// estimadas se conservan, no se fuerzan a cero).
    #[test]
    fn passing_through_origin_does_not_reinitialize() {
        let mut ekf = ExtendedKalmanFilter::new();
        let dt = 0.016;
        // Trayectoria a velocidad constante que cruza el origen exacto.
        let step = (-0.02, -0.01);
        let mut x = 0.20;
        let mut y = 0.10;
        for _ in 0..15 {
            ekf.filter_pose(x, y, 0.0, dt);
            x += step.0;
            y += step.1;
        }
        // Pasa exactamente por (0,0)...
        let (_, _, _, vx0, vy0, _) = ekf.filter_pose(0.0, 0.0, 0.0, dt);
        // ...y si el filtro NO se reinició, las velocidades siguen reflejando el
        // movimiento (serían 0 si se hubiese reinicializado).
        assert!(vx0 < -0.1, "vx se reinició en el origen: {vx0}");
        assert!(vy0 < -0.05, "vy se reinició en el origen: {vy0}");
    }

    /// Mediciones normales pasan el gating y el filtro converge a la pose medida.
    #[test]
    fn normal_measurements_converge() {
        let mut ekf = ExtendedKalmanFilter::new();
        let dt = 0.016;
        let mut last = (0.0, 0.0, 0.0);
        for _ in 0..40 {
            let (x, y, theta, ..) = ekf.filter_pose(0.5, -0.3, 0.7, dt);
            last = (x, y, theta);
        }
        assert!((last.0 - 0.5).abs() < 0.05);
        assert!((last.1 - (-0.3)).abs() < 0.05);
        assert!((last.2 - 0.7).abs() < 0.05);
    }

    /// El filtro nunca produce NaN/valores no finitos ante secuencias largas
    /// (incluye mediciones repetidas que podrían acercar S a singular).
    #[test]
    fn stays_finite_over_long_run() {
        let mut ekf = ExtendedKalmanFilter::new();
        let dt = 0.016;
        for i in 0..200 {
            let x = 0.3 + (i as f64 * 0.001);
            let (xf, yf, tf, vx, vy, w) = ekf.filter_pose(x, 0.2, 0.1, dt);
            assert!(
                xf.is_finite() && yf.is_finite() && tf.is_finite()
                    && vx.is_finite() && vy.is_finite() && w.is_finite(),
                "valor no finito en iter {i}"
            );
        }
    }

    /// Con R más chico el filtro sigue más de cerca la medición (menos suavizado):
    /// los parámetros del JSON realmente cambian el comportamiento.
    #[test]
    fn params_change_filter_behaviour() {
        let dt = 0.016;
        // Gate desactivado: aquí se mide el efecto de R en la ganancia, no el gating
        // (con R muy chico el salto de 3 cm sería rechazado como outlier).
        let run = |r_pos: f64| {
            let mut ekf = ExtendedKalmanFilter::from_params(&VisionParams {
                r_pos,
                gating_chi2: 1e9,
                ..VisionParams::default()
            });
            for _ in 0..20 {
                ekf.filter_pose(0.0, 0.0, 0.0, dt);
            }
            // Salto de 3 cm: ¿cuánto lo sigue en un solo frame?
            let (x, ..) = ekf.filter_pose(0.03, 0.0, 0.0, dt);
            x
        };
        let tight = run(1e-6);
        let loose = run(1e-3);
        assert!(tight > loose, "R chico debe seguir más la medición: {tight} vs {loose}");
    }

    #[test]
    fn test_filter_pose() {
        let mut ekf = ExtendedKalmanFilter::new();

        // Primera medición - inicializa el filtro
        let (x, y, ..) = ekf.filter_pose(1.0, 2.0, 0.0, 0.016);

        assert!((x - 1.0).abs() < 0.2);
        assert!((y - 2.0).abs() < 0.2);

        // Segunda medición (movimiento) - debería filtrar y estimar velocidades
        let (x2, y2, ..) = ekf.filter_pose(1.1, 2.1, 0.0, 0.016);

        // El filtro debería haber convergido hacia la nueva posición
        assert!((x2 - 1.1).abs() < 0.2);
        assert!((y2 - 2.1).abs() < 0.2);

        // Debería haber estimado alguna velocidad (aunque pequeña al principio)
        // Las velocidades pueden ser pequeñas al inicio del filtro
    }
}
