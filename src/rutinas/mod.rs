//! Rutinas: circuitos, coreografías y jugadas con pelota que se lanzan desde la GUI o por
//! consola, como pruebas repetibles en simulación y en el robot real (change `rutinas`,
//! fase 1).
//!
//! - `formato`: los archivos de `config/rutinas/` y su carga;
//! - `validacion`: las reglas de un archivo válido;
//! - `asignacion`: robots a puestos;
//! - `decisor`: `RoutineDecider`, un decisor más del lazo común;
//! - `vista` y `resumen`: lo que ven la GUI, el CSV y el registro de la corrida;
//! - `conmutador`: el decisor de `main` con GUI (coach o rutina, con las órdenes de la GUI).
//!
//! Una rutina solo elige skill y objetivo por robot: no hay otro camino de control.

pub mod asignacion;
pub mod conmutador;
pub mod decisor;
pub mod formato;
pub mod resumen;
pub mod validacion;
pub mod vista;

#[cfg(test)]
mod pruebas_lazo;

pub use asignacion::{Eleccion, asignar};
pub use conmutador::{DecisorConRutinas, OrdenRutina};
pub use decisor::{Entorno, OpcionesCorrida, OrdenCorrida, RoutineDecider, velocidad_valida};
pub use formato::{Entrada, Grupo, Repeticiones, Rutina, buscar, cargar_carpeta, carpeta};
pub use resumen::{Resultado, ResumenCorrida};
pub use validacion::ContextoValidacion;
pub use vista::{Fase, RutinaVista};

/// Textos de rutinas para los tests.
#[cfg(test)]
pub(crate) mod prueba {
    /// Rutina de un robot (puesto A, inicio en (−0.3, −0.25) mirando a +x, 1 repetición)
    /// con los pasos dados (el contenido de la lista `pasos`).
    pub fn un_puesto(pasos: &str) -> String {
        format!(
            r#"{{
              "nombre": "Prueba", "grupo": "circuito", "descripcion": "Para los tests.",
              "robots": {{ "min": 1, "max": 1 }},
              "repeticiones": 1,
              "puestos": [
                {{ "nombre": "A", "inicio": {{ "x": -0.3, "y": -0.25, "orientacion_deg": 0 }}, "pasos": [ {pasos} ] }}
              ]
            }}"#
        )
    }

    /// Rutina de dos robots con los inicios dados y un `ir` 0.3 m hacia arriba cada uno.
    pub fn dos_puestos(a: (f32, f32), b: (f32, f32)) -> String {
        format!(
            r#"{{
              "nombre": "Dos", "grupo": "coreografia", "descripcion": "Para los tests.",
              "robots": {{ "min": 2, "max": 2 }},
              "puestos": [
                {{ "nombre": "A", "inicio": {{ "x": {ax}, "y": {ay}, "orientacion_deg": 90 }},
                   "pasos": [ {{ "paso": "ir", "x": {ax}, "y": {ay2} }} ] }},
                {{ "nombre": "B", "inicio": {{ "x": {bx}, "y": {by}, "orientacion_deg": 90 }},
                   "pasos": [ {{ "paso": "ir", "x": {bx}, "y": {by2} }} ] }}
              ]
            }}"#,
            ax = a.0,
            ay = a.1,
            ay2 = a.1 + 0.3,
            bx = b.0,
            by = b.1,
            by2 = b.1 + 0.3,
        )
    }

    /// Pases entre dos robots: A en (−0.40, 0) y B en (0.40, 0), marca en (−0.28, 0). A le
    /// pasa a B, vuelve y recibe; B recibe, le pasa a A y vuelve.
    pub fn pases() -> String {
        r#"{
          "nombre": "Pases", "grupo": "con_pelota", "descripcion": "Para los tests.",
          "robots": { "min": 2, "max": 2 },
          "pelota": { "marca": { "x": -0.28, "y": 0.0 } },
          "puestos": [
            { "nombre": "A", "inicio": { "x": -0.40, "y": 0.0, "orientacion_deg": 0 },
              "pasos": [ { "paso": "empujar", "a_puesto": "B" }, { "paso": "ir", "x": -0.40, "y": 0.0 }, { "paso": "recibir", "x": -0.40, "y": 0.0 } ] },
            { "nombre": "B", "inicio": { "x": 0.40, "y": 0.0, "orientacion_deg": 180 },
              "pasos": [ { "paso": "recibir", "x": 0.40, "y": 0.0 }, { "paso": "empujar", "a_puesto": "A" }, { "paso": "ir", "x": 0.40, "y": 0.0 } ] }
          ]
        }"#
        .to_string()
    }

    /// La rutina de `texto`, que debe ser válida.
    pub fn rutina(texto: &str) -> std::sync::Arc<super::Rutina> {
        let e = super::formato::cargar_texto(
            texto,
            "prueba",
            std::path::PathBuf::from("prueba.json"),
            &super::ContextoValidacion::por_defecto(1.0),
        );
        assert!(e.razones.is_empty(), "{:?}", e.razones);
        e.rutina.expect("leída")
    }
}
