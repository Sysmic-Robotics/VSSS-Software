//! Asignación de robots a puestos (D8 del design). Función pura: la usan el decisor al
//! preparar y la GUI para la vista previa, con los mismos robots.
//!
//! - **Automática:** descarta primero los robots sin comandos (patrones desactivados, sin
//!   robot o repetidos, `Identidad::sin_comandos`). Con el resto,
//!   con k = min(`max`, robots vistos) y k ≥ `min`, elige los k robots y su
//!   orden sobre los primeros k puestos que minimizan la suma de distancias al inicio. Un
//!   puesto `arquero` lleva al `keeper_id`. Por fuerza bruta: con 5 robots y 3 puestos son
//!   60 combinaciones. Un empate se resuelve por la lista de ids más baja.
//! - **Manual:** un robot por puesto, en orden; deben verse y ser distintos.

use super::formato::{Puesto, Rutina};
use glam::Vec2;

/// Cómo se eligen los robots de la corrida.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Eleccion {
    Automatica,
    /// Un robot por puesto, en el orden de los puestos.
    Manual(Vec<i32>),
}

/// "se ven 0 robots propios", "se ve 1 robot propio", "se ven 2 robots propios".
fn se_ven(n: usize) -> String {
    if n == 1 { "se ve 1 robot propio".to_string() } else { format!("se ven {n} robots propios") }
}

/// Robot por puesto (los primeros k puestos de la rutina), o la razón por la que no se
/// puede asignar. `vistos` son los robots propios que la visión ve, con su posición;
/// `sin_comandos`, los patrones a los que no les llegan comandos (la automática no los
/// elige; la manual no cambia).
pub fn asignar(
    rutina: &Rutina,
    vistos: &[(i32, Vec2)],
    keeper_id: i32,
    eleccion: &Eleccion,
    sin_comandos: &[i32],
) -> Result<Vec<i32>, String> {
    let mut vistos = vistos.to_vec();
    vistos.sort_by_key(|(id, _)| *id);
    match eleccion {
        Eleccion::Automatica => automatica(rutina, &vistos, keeper_id, sin_comandos),
        Eleccion::Manual(ids) => manual(rutina, &vistos, keeper_id, ids),
    }
}

/// "1 no recibe comandos", "2 no reciben comandos".
fn no_reciben(n: usize) -> String {
    if n == 1 { "1 no recibe comandos".to_string() } else { format!("{n} no reciben comandos") }
}

fn automatica(
    rutina: &Rutina,
    todos: &[(i32, Vec2)],
    keeper_id: i32,
    sin_comandos: &[i32],
) -> Result<Vec<i32>, String> {
    let vistos: Vec<(i32, Vec2)> = todos.iter().copied().filter(|(id, _)| !sin_comandos.contains(id)).collect();
    let descartados = todos.len() - vistos.len();
    let k = rutina.robots_max.min(vistos.len()).min(rutina.puestos.len());
    if k < rutina.robots_min {
        let cuantos = if descartados > 0 {
            format!("{} y {} (pestaña Robots)", se_ven(todos.len()), no_reciben(descartados))
        } else {
            se_ven(todos.len())
        };
        return Err(format!("{cuantos}; la rutina pide {}", rutina.texto_robots()));
    }
    let puestos = &rutina.puestos[..k];
    if let Some(p) = puestos.iter().find(|p| p.arquero)
        && !vistos.iter().any(|(id, _)| *id == keeper_id)
    {
        let por_que = if todos.iter().any(|(id, _)| *id == keeper_id) {
            "no recibe comandos (pestaña Robots)"
        } else {
            "no se ve"
        };
        return Err(format!(
            "el puesto {} es de arquero y el arquero (robot {keeper_id}) {por_que}",
            p.nombre
        ));
    }
    let mut mejor: Option<(f32, Vec<i32>)> = None;
    let mut usados = vec![false; vistos.len()];
    let mut actual = Vec::with_capacity(k);
    recorrer(puestos, &vistos, keeper_id, &mut usados, &mut actual, 0.0, &mut mejor);
    mejor
        .map(|(_, ids)| ids)
        .ok_or_else(|| "no hay una asignación posible".to_string())
}

/// Prueba cada robot libre en el puesto `actual.len()`, acumulando la distancia.
fn recorrer(
    puestos: &[Puesto],
    vistos: &[(i32, Vec2)],
    keeper_id: i32,
    usados: &mut [bool],
    actual: &mut Vec<i32>,
    costo: f32,
    mejor: &mut Option<(f32, Vec<i32>)>,
) {
    let k = actual.len();
    if k == puestos.len() {
        let gana = match mejor {
            None => true,
            Some((c, ids)) => costo < *c || (costo == *c && *actual < *ids),
        };
        if gana {
            *mejor = Some((costo, actual.clone()));
        }
        return;
    }
    for (i, &(id, pos)) in vistos.iter().enumerate() {
        if usados[i] || (puestos[k].arquero && id != keeper_id) {
            continue;
        }
        usados[i] = true;
        actual.push(id);
        let d = (pos - puestos[k].inicio.pos).length();
        recorrer(puestos, vistos, keeper_id, usados, actual, costo + d, mejor);
        actual.pop();
        usados[i] = false;
    }
}

fn manual(rutina: &Rutina, vistos: &[(i32, Vec2)], keeper_id: i32, ids: &[i32]) -> Result<Vec<i32>, String> {
    let k = ids.len();
    if k < rutina.robots_min || k > rutina.robots_max.min(rutina.puestos.len()) {
        return Err(format!(
            "la elección manual tiene {k} {}; la rutina pide {}",
            if k == 1 { "robot" } else { "robots" },
            rutina.texto_robots()
        ));
    }
    for (i, id) in ids.iter().enumerate() {
        if ids[..i].contains(id) {
            return Err(format!("el robot {id} está repetido"));
        }
        if !vistos.iter().any(|(v, _)| v == id) {
            return Err(format!("el robot {id} no se ve"));
        }
    }
    if let Some(p) = rutina.puestos[..k]
        .iter()
        .zip(ids)
        .find(|(p, id)| p.arquero && **id != keeper_id)
        .map(|(p, _)| p)
    {
        return Err(format!(
            "el puesto {} es de arquero: va el robot {keeper_id} (coach.keeper_id)",
            p.nombre
        ));
    }
    Ok(ids.to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rutinas::formato::{Accion, Grupo, Paso, Pose, Repeticiones};

    fn puesto(nombre: &str, x: f32, y: f32, arquero: bool) -> Puesto {
        Puesto {
            nombre: nombre.to_string(),
            inicio: Pose { pos: Vec2::new(x, y), orientacion: 0.0 },
            arquero,
            pasos: vec![Paso {
                accion: Accion::Ir { punto: Vec2::new(x, y + 0.2), orientacion: None },
                tolerancia_m: None,
                tolerancia_deg: None,
                tiempo_max_s: None,
            }],
        }
    }

    fn rutina(min: usize, puestos: Vec<Puesto>) -> Rutina {
        Rutina {
            id: "prueba".to_string(),
            nombre: "Prueba".to_string(),
            grupo: Grupo::Coreografia,
            descripcion: String::new(),
            robots_min: min,
            robots_max: puestos.len(),
            pelota: None,
            repeticiones: Repeticiones::Veces(1),
            puestos,
        }
    }

    #[test]
    fn the_closest_robot_takes_the_post() {
        // El triángulo: un puesto con inicio en (−0.30, −0.25).
        let r = rutina(1, vec![puesto("A", -0.30, -0.25, false)]);
        let vistos = [(0, Vec2::new(0.5, 0.4)), (1, Vec2::new(-0.2, -0.2)), (2, Vec2::new(0.0, 0.5))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[]), Ok(vec![1]));
    }

    #[test]
    fn minimum_total_distance_not_greedy() {
        // Goloso por puesto daría A←0 (0.2) y B←1 (0.5): 0.7. El mínimo es A←1, B←0: 0.3.
        let r = rutina(2, vec![puesto("A", 0.0, 0.0, false), puesto("B", 0.3, 0.0, false)]);
        let vistos = [(0, Vec2::new(0.2, 0.0)), (1, Vec2::new(-0.2, 0.0))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[]), Ok(vec![1, 0]));
    }

    #[test]
    fn uses_as_many_robots_as_it_can() {
        let r = rutina(2, vec![puesto("A", -0.3, 0.0, false), puesto("B", 0.0, 0.0, false), puesto("C", 0.3, 0.0, false)]);
        let dos = [(0, Vec2::new(-0.3, 0.1)), (1, Vec2::new(0.3, 0.1))];
        // Con dos robots se usan los dos primeros puestos.
        assert_eq!(asignar(&r, &dos, 2, &Eleccion::Automatica, &[]), Ok(vec![0, 1]));
        let uno = [(0, Vec2::ZERO)];
        assert_eq!(
            asignar(&r, &uno, 2, &Eleccion::Automatica, &[]),
            Err("se ve 1 robot propio; la rutina pide 2–3 robots".to_string())
        );
        let r1 = rutina(1, vec![puesto("A", -0.3, 0.0, false)]);
        assert_eq!(
            asignar(&r1, &[], 2, &Eleccion::Automatica, &[]),
            Err("se ven 0 robots propios; la rutina pide 1 robot".to_string())
        );
    }

    #[test]
    fn keeper_post_takes_the_keeper() {
        let r = rutina(2, vec![puesto("A", -0.5, 0.0, true), puesto("B", 0.0, 0.0, false)]);
        // El 0 está más cerca del inicio de A, pero A es de arquero: va el 2.
        let vistos = [(0, Vec2::new(-0.5, 0.0)), (2, Vec2::new(0.0, 0.0))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[]), Ok(vec![2, 0]));
        let sin_arquero = [(0, Vec2::ZERO), (1, Vec2::ZERO)];
        let err = asignar(&r, &sin_arquero, 2, &Eleccion::Automatica, &[]).unwrap_err();
        assert!(err.contains("arquero (robot 2) no se ve"), "{err}");
    }

    #[test]
    fn ties_go_to_the_lowest_ids() {
        let r = rutina(1, vec![puesto("A", 0.0, 0.0, false)]);
        let vistos = [(2, Vec2::new(0.1, 0.0)), (1, Vec2::new(-0.1, 0.0))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[]), Ok(vec![1]));
    }

    #[test]
    fn automatica_no_elige_sin_comandos() {
        // El escenario "El más cercano" con el robot 1 sin comandos: va el siguiente (el 2).
        let r = rutina(1, vec![puesto("A", -0.30, -0.25, false)]);
        let vistos = [(0, Vec2::new(0.5, 0.4)), (1, Vec2::new(-0.2, -0.2)), (2, Vec2::new(0.0, 0.5))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[1]), Ok(vec![2]));
    }

    #[test]
    fn automatica_no_alcanzan_dice_cuantos_no_reciben() {
        let r = rutina(3, vec![puesto("A", -0.3, 0.0, false), puesto("B", 0.0, 0.0, false), puesto("C", 0.3, 0.0, false)]);
        let vistos = [(0, Vec2::ZERO), (1, Vec2::ZERO), (2, Vec2::ZERO)];
        assert_eq!(
            asignar(&r, &vistos, 2, &Eleccion::Automatica, &[0]),
            Err("se ven 3 robots propios y 1 no recibe comandos (pestaña Robots); la rutina pide 3 robots".to_string())
        );
        assert_eq!(
            asignar(&r, &vistos, 2, &Eleccion::Automatica, &[0, 2]),
            Err("se ven 3 robots propios y 2 no reciben comandos (pestaña Robots); la rutina pide 3 robots".to_string())
        );
    }

    #[test]
    fn automatica_arquero_sin_comandos() {
        let r = rutina(2, vec![puesto("A", -0.5, 0.0, true), puesto("B", 0.0, 0.0, false)]);
        let vistos = [(0, Vec2::new(-0.5, 0.0)), (1, Vec2::ZERO), (2, Vec2::ZERO)];
        let err = asignar(&r, &vistos, 2, &Eleccion::Automatica, &[2]).unwrap_err();
        assert_eq!(err, "el puesto A es de arquero y el arquero (robot 2) no recibe comandos (pestaña Robots)");
    }

    #[test]
    fn sin_lista_igual_que_antes() {
        let r = rutina(2, vec![puesto("A", 0.0, 0.0, false), puesto("B", 0.3, 0.0, false)]);
        let vistos = [(0, Vec2::new(0.2, 0.0)), (1, Vec2::new(-0.2, 0.0)), (2, Vec2::new(0.6, 0.0))];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[]), Ok(vec![1, 0]));
        // Un patrón fuera de los vistos en la lista no cambia nada.
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Automatica, &[3]), Ok(vec![1, 0]));
    }

    #[test]
    fn manual_no_cambia_con_la_lista() {
        let r = rutina(1, vec![puesto("A", -0.5, 0.0, false)]);
        let vistos = [(0, Vec2::ZERO), (1, Vec2::ZERO)];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Manual(vec![1]), &[1]), Ok(vec![1]));
    }

    #[test]
    fn manual_choice_is_checked() {
        let r = rutina(1, vec![puesto("A", -0.5, 0.0, true), puesto("B", 0.0, 0.0, false)]);
        let vistos = [(0, Vec2::ZERO), (1, Vec2::ZERO), (2, Vec2::ZERO)];
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Manual(vec![2, 1]), &[]), Ok(vec![2, 1]));
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Manual(vec![2, 2]), &[]), Err("el robot 2 está repetido".to_string()));
        assert_eq!(asignar(&r, &vistos, 2, &Eleccion::Manual(vec![2, 4]), &[]), Err("el robot 4 no se ve".to_string()));
        let err = asignar(&r, &vistos, 2, &Eleccion::Manual(vec![1]), &[]).unwrap_err();
        assert!(err.contains("es de arquero: va el robot 2"), "{err}");
        let err = asignar(&r, &vistos, 2, &Eleccion::Manual(vec![2, 1, 0]), &[]).unwrap_err();
        assert!(err.contains("la elección manual tiene 3 robots"), "{err}");
    }
}
