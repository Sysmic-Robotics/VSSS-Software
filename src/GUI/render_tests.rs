//! La cancha dibujada con tiny-skia, el renderer por CPU de iced (el respaldo cuando no
//! hay GPU), y el cálculo de daño que hace su compositor en cada cuadro. Ahí estaba el
//! pánico: un rectángulo de daño con NaN rompe el `sort` de
//! `iced_graphics::damage::group`, y con la GUI se cae el proceso entero.

use super::charts::{LineChart, MuestraSenal, SenalChart};
use super::field::{Fieltro, FieldCanvas, OrientacionVista};
use super::run_info::run_info_de_prueba;
use super::status_line::{Tasas, frases_estado};
use super::tag::MarcasRobot;
use super::{robot_rows, theme};
use crate::snapshot::{BallView, LoopSnapshot, RobotView};
use glam::Vec2;
use iced::widget::canvas::{self, Cache};
use iced::{Point, Rectangle, Size, Theme, Vector, mouse};
use iced_tiny_skia::Layer as Capa;
use iced_tiny_skia::core::Renderer as _;
use iced_tiny_skia::graphics::damage;
use iced_tiny_skia::graphics::geometry::Renderer as _;
use std::collections::VecDeque;

/// Ventana y lugar de la cancha en ella (a la derecha del panel de robots).
const VENTANA: Size = Size::new(1440.0, 900.0);
const CANCHA: Rectangle = Rectangle {
    x: 300.0,
    y: 40.0,
    width: 840.0,
    height: 690.0,
};

fn robot(team: i32, id: i32, x: f32, y: f32, orientation: f64) -> RobotView {
    RobotView {
        team,
        id,
        position: Vec2::new(x, y),
        orientation,
        velocity: Vec2::new(0.3, -0.1),
        angular_velocity: 1.0,
        active: true,
        age_s: 0.02,
    }
}

/// Fotos seguidas: la segunda mueve los robots y trae NaN e infinitos en las poses,
/// la velocidad y la pelota; la tercera vuelve a la normalidad.
fn fotos() -> Vec<(LoopSnapshot, Option<Vec2>)> {
    let sana = |dx: f32| LoopSnapshot {
        robots: vec![
            robot(0, 0, -0.5 + dx, 0.1, 0.0),
            robot(0, 1, 0.1 + dx, -0.2, 1.0),
            robot(0, 2, 0.3, 0.3 - dx, -2.0),
            robot(1, 0, 0.4 - dx, 0.0, 3.0),
        ],
        ball: Some(BallView {
            position: Vec2::new(dx, 0.05),
            velocity: Vec2::ZERO,
        }),
        ..LoopSnapshot::default()
    };
    let mut rara = sana(0.05);
    rara.robots[1].position = Vec2::new(f32::NAN, 0.2);
    rara.robots[2].orientation = f64::INFINITY;
    rara.robots[2].velocity = Vec2::new(f32::NAN, f32::INFINITY);
    rara.robots[3].position = Vec2::new(f32::INFINITY, f32::NEG_INFINITY);
    rara.robots.push(RobotView {
        age_s: f32::NAN,
        angular_velocity: f64::NAN,
        ..robot(1, 1, 0.2, 0.2, f64::NAN)
    });
    rara.ball = Some(BallView {
        position: Vec2::new(f32::NAN, f32::INFINITY),
        velocity: Vec2::splat(f32::NAN),
    });
    vec![
        (sana(0.0), Some(Vec2::new(0.2, 0.1))),
        (rara, Some(Vec2::new(f32::INFINITY, f32::NAN))),
        (sana(0.1), Some(Vec2::new(0.2, 0.1))),
    ]
}

/// Dibuja la cancha como el widget (trasladada a su lugar) y devuelve las capas.
fn cuadro(
    renderer: &mut iced::Renderer,
    base: &Cache,
    snap: &LoopSnapshot,
    skill_target: Option<Vec2>,
) -> Vec<Capa> {
    let cancha = FieldCanvas {
        snap,
        base,
        fieltro: Fieltro::Verde,
        orientacion: OrientacionVista::Normal,
        marcas: MarcasRobot::Anexo1,
        own_team: 0,
        selected: Some((0, 1)),
        skill_target,
        radio_slots: None,
    };
    renderer.clear();
    let geometrias = canvas::Program::<super::Message>::draw(
        &cancha,
        &(),
        renderer,
        &Theme::Light,
        Rectangle::with_size(CANCHA.size()),
        mouse::Cursor::Unavailable,
    );
    renderer.with_translation(Vector::new(CANCHA.x, CANCHA.y), |r| {
        for g in geometrias {
            r.draw_geometry(g);
        }
    });
    let iced::Renderer::Secondary(tiny_skia) = renderer else {
        unreachable!("el test arma un renderer tiny-skia")
    };
    tiny_skia.layers().to_vec()
}

fn renderer_tiny_skia() -> iced::Renderer {
    iced::Renderer::Secondary(iced_tiny_skia::Renderer::new(
        super::fonts::TEXTO,
        iced::Pixels(14.0),
    ))
}

#[test]
fn tiny_skia_damage_stays_finite_with_moving_labels_and_non_finite_data() {
    let mut renderer = renderer_tiny_skia();
    let base = Cache::new();
    let mut anteriores: Option<Vec<Capa>> = None;
    for (i, (snap, target)) in fotos().iter().enumerate() {
        let capas = cuadro(&mut renderer, &base, snap, *target);
        if let Some(prev) = &anteriores {
            // Lo mismo que hace el compositor de tiny-skia antes de presentar.
            let danio = damage::diff(prev, &capas, |c| vec![c.bounds], Capa::damage);
            assert!(!danio.is_empty(), "cuadro {i}: los robots se movieron");
            for r in &danio {
                assert!(
                    !r.center().distance(Point::ORIGIN).is_nan(),
                    "cuadro {i}: rectángulo de daño con NaN {r:?}"
                );
            }
            damage::group(danio, Rectangle::with_size(VENTANA));
        }
        anteriores = Some(capas);
    }
}

/// Los gráficos con muestras NaN e infinitas: en un build de debug, lyon entra en
/// pánico si le llega un punto no finito.
#[test]
fn charts_skip_non_finite_samples() {
    let renderer = renderer_tiny_skia();
    let senal: VecDeque<MuestraSenal> = [
        (0.0, Some(0.2), Some(0.1)),
        (0.1, Some(f32::NAN), Some(f32::INFINITY)),
        (0.2, Some(f32::NEG_INFINITY), None),
        (0.3, Some(0.3), Some(f32::NAN)),
        (0.4, Some(0.25), Some(0.2)),
    ]
    .into_iter()
    .map(|(t, pedida, medida)| MuestraSenal { t, pedida, medida })
    .collect();
    let error: VecDeque<(f64, f32, f32)> = [
        (0.0, 0.1, 0.2),
        (0.1, f32::NAN, f32::INFINITY),
        (0.2, f32::NEG_INFINITY, 0.3),
        (0.3, 0.2, f32::NAN),
    ]
    .into_iter()
    .collect();
    let bounds = Rectangle::with_size(Size::new(400.0, 160.0));
    for tope in [1.5, 0.0, f32::NAN, f32::INFINITY] {
        let cache = Cache::new();
        let chart = SenalChart {
            serie: &senal,
            tope,
            ventana_s: 5.0,
            cache: &cache,
        };
        canvas::Program::<super::Message>::draw(
            &chart,
            &(),
            &renderer,
            &Theme::Light,
            bounds,
            mouse::Cursor::Unavailable,
        );
    }
    let cache = Cache::new();
    let chart = LineChart {
        history: &error,
        ventana_s: 5.0,
        color_a: theme::AZUL,
        color_b: theme::TINTA,
        cache: &cache,
    };
    canvas::Program::<super::Message>::draw(
        &chart,
        &(),
        &renderer,
        &Theme::Light,
        bounds,
        mouse::Cursor::Unavailable,
    );
}

/// Los textos de las filas y de la barra con la foto de NaN e infinitos: sin pánico y
/// sin "NaN" ni "inf" a la vista.
#[test]
fn rows_and_status_bar_text_have_no_nan_with_non_finite_data() {
    let (snap, _) = &fotos()[1];
    let run = run_info_de_prueba();
    let tasas = Tasas {
        cuadros_s: Some(f32::NAN),
        perdidas: Some(f32::INFINITY),
        envios_s: Some(f32::NEG_INFINITY),
        velocidad_sim: Some(f32::NAN),
        sin_cuadros_s: Some(f32::INFINITY),
    };
    let mut textos: Vec<String> = Vec::new();
    for id in 0..3 {
        let fila = robot_rows::datos_fila(id, &run, snap, None);
        textos.extend([fila.titulo, fila.al_lado]);
        textos.extend(fila.skill.into_iter().chain(fila.pedido).chain(fila.medido));
        textos.extend(fila.avisos);
    }
    textos.push(robot_rows::linea_rivales(snap, 0, 3));
    textos.extend(frases_estado(&run, snap, &tasas, None).into_iter().map(|f| f.texto));
    // Así escribe Rust un f32 no finito: "NaN", "inf" y "-inf".
    for t in &textos {
        let no_finito = t
            .split(|c: char| !c.is_alphanumeric())
            .any(|w| w == "NaN" || w == "inf");
        assert!(!no_finito, "texto con un no finito: {t}");
    }
    // El robot 2 tiene la velocidad en NaN: su medido se omite.
    assert!(robot_rows::datos_fila(2, &run, snap, None).medido.is_none());
}
