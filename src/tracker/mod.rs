mod ekf;

pub use ekf::ExtendedKalmanFilter;

use std::collections::HashMap;

/// Tracker que mantiene un filtro Kalman por cada robot/balón
/// 
/// Identifica cada objeto por (team, id):
/// - Robots: team = 0 (azul) o 1 (amarillo), id = robot_id
/// - Balón: team = -1, id = -1
pub struct Tracker {
    filters: HashMap<(i32, i32), ExtendedKalmanFilter>,
}

impl Tracker {
    /// Crea un nuevo Tracker vacío
    pub fn new() -> Self {
        Self {
            filters: HashMap::new(),
        }
    }
    
    /// Filtra una pose y retorna posición filtrada y velocidades estimadas
    /// 
    /// # Arguments
    /// * `team` - Equipo del robot (0 = azul, 1 = amarillo, -1 = balón)
    /// * `id` - ID del robot o -1 para balón
    /// * `x` - Posición X en metros
    /// * `y` - Posición Y en metros
    /// * `theta` - Orientación en radianes
    /// * `dt` - Delta de tiempo en segundos
    /// 
    /// # Returns
    /// Tupla (x, y, theta, vx, vy, omega) con posiciones filtradas y velocidades estimadas
    pub fn track(&mut self, team: i32, id: i32, x: f64, y: f64, 
                 theta: f64, dt: f64) -> (f64, f64, f64, f64, f64, f64) {
        let key = (team, id);
        
        // Obtener o crear filtro para este (team, id)
        let filter = self.filters.entry(key)
            .or_insert_with(|| Self::create_initial_filter());
        
        // Filtrar la pose y retornar resultado
        filter.filter_pose(x, y, theta, dt)
    }
    
    /// Crea un filtro Kalman inicializado
    fn create_initial_filter() -> ExtendedKalmanFilter {
        ExtendedKalmanFilter::new()
    }
    
    /// Limpia filtros que no se han usado recientemente (opcional, para limpieza de memoria)
    pub fn cleanup_old_filters(&mut self) {
        // Por ahora, mantener todos los filtros
        // En el futuro, se podría implementar limpieza de filtros inactivos
    }
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tracker_new() {
        let tracker = Tracker::new();
        assert_eq!(tracker.filters.len(), 0);
    }
    
    #[test]
    fn test_tracker_track_robot() {
        let mut tracker = Tracker::new();
        
        // Primera llamada para un robot - debería crear el filtro
        let (x1, y1, theta1, vx1, vy1, omega1) = tracker.track(0, 1, 1.0, 2.0, 0.0, 0.016);
        
        assert!((x1 - 1.0).abs() < 0.2);
        assert!((y1 - 2.0).abs() < 0.2);
        assert_eq!(tracker.filters.len(), 1);
        
        // Segunda llamada - debería usar el mismo filtro
        let (x2, y2, theta2, vx2, vy2, omega2) = tracker.track(0, 1, 1.1, 2.1, 0.0, 0.016);
        
        assert!((x2 - 1.1).abs() < 0.2);
        assert!((y2 - 2.1).abs() < 0.2);
        assert_eq!(tracker.filters.len(), 1); // Mismo filtro
        
        // Debería haber estimado alguna velocidad (puede ser pequeña al inicio)
        // El filtro necesita varias iteraciones para estimar velocidades correctamente
    }
    
    #[test]
    fn test_tracker_multiple_robots() {
        let mut tracker = Tracker::new();
        
        // Robot azul 1
        tracker.track(0, 1, 1.0, 2.0, 0.0, 0.016);
        
        // Robot amarillo 1
        tracker.track(1, 1, 3.0, 4.0, 0.0, 0.016);
        
        // Balón
        tracker.track(-1, -1, 0.0, 0.0, 0.0, 0.016);
        
        assert_eq!(tracker.filters.len(), 3);
    }
    
    #[test]
    fn test_tracker_ball() {
        let mut tracker = Tracker::new();
        
        // Tracking del balón
        let (x, y, theta, vx, vy, omega) = tracker.track(-1, -1, 0.5, 0.3, 0.0, 0.016);
        
        assert!((x - 0.5).abs() < 0.1);
        assert!((y - 0.3).abs() < 0.1);
    }
}

