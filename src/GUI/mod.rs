mod field;
mod vision_status;

use glam::Vec2;
use iced::futures::SinkExt;
use iced::stream;
use iced::widget::canvas::Cache;
use iced::{
    Element, Length, Subscription, Task, Theme,
    widget::{Canvas, button, column, container, row, text},
};
use std::collections::HashMap;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use tokio::sync::mpsc;

use field::FieldCanvas;
// StatusUpdate vive en vision.rs para que la lib no dependa de gui.
pub use rustengine::vision::StatusUpdate;

/// `true` imprime cada actualización de robot en stderr (muy ruidoso). Dejar en `false` para auditar con `[FieldAudit]` en `main`.
const GUI_LOG_EVERY_ROBOT_UPDATE: bool = false;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TabView {
    Vision,
    Robots,
}

#[derive(Debug, Clone)]
pub enum Message {
    StatusUpdate(StatusUpdate),
    ChangeIp(String),
    ChangePort(String),
    Connect,
    ToggleTracker(bool), // Nuevo mensaje para toggle del tracker
    Tick,
    TabSelected(TabView),
}

#[derive(Debug, Clone)]
pub struct Robot {
    #[allow(dead_code)] // ID may be used for labeling robots in the future
    pub id: u32,
    pub team: u32,
    pub position: Vec2,
    pub orientation: f32,
}

#[derive(Debug, Clone)]
pub struct Ball {
    pub position: Vec2,
}

pub struct VisionGui {
    vision_ip: String,
    vision_port: String,
    connected: bool,
    packet_count: u64,
    packet_frequency: f64,
    last_ball_count: usize,
    last_robot_count: usize,
    robots: HashMap<(u32, u32), Robot>,
    ball: Option<Ball>,
    field_cache: Cache,
    chart_cache: Cache,
    config_tx: Option<mpsc::Sender<ConfigUpdate>>,
    status_rx: Arc<Mutex<Option<mpsc::Receiver<StatusUpdate>>>>,
    active_tab: TabView,
    packet_history: VecDeque<(f64, u64)>, // (time_seconds, packets_in_that_second)
    start_time: Instant,
    last_second: u64,
    current_second_count: u64,
    tracker_enabled: bool, // Estado del filtro Kalman
}

#[derive(Debug, Clone)]
pub enum ConfigUpdate {
    ChangeIpPort(String, u16),
    ToggleTracker(bool), // true = habilitado, false = deshabilitado
}

impl VisionGui {
    fn new(
        ip: String,
        port: u16,
        config_tx: mpsc::Sender<ConfigUpdate>,
        status_rx: mpsc::Receiver<StatusUpdate>,
    ) -> (Self, Task<Message>) {
        (
            VisionGui {
                vision_ip: ip,
                vision_port: port.to_string(),
                connected: false,
                packet_count: 0,
                packet_frequency: 0.0,
                last_ball_count: 0,
                last_robot_count: 0,
                robots: HashMap::new(),
                ball: None,
                field_cache: Cache::default(),
                chart_cache: Cache::default(),
                config_tx: Some(config_tx),
                status_rx: Arc::new(Mutex::new(Some(status_rx))),
                active_tab: TabView::Vision,
                packet_history: VecDeque::new(),
                start_time: Instant::now(),
                last_second: 0,
                current_second_count: 0,
                tracker_enabled: true, // Habilitado por defecto
            },
            Task::none(),
        )
    }

    fn title(&self) -> String {
        String::from("RustEngine - Vision System")
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::StatusUpdate(update) => {
                match update {
                    StatusUpdate::Connected(ip, port) => {
                        self.vision_ip = ip;
                        self.vision_port = port.to_string();
                        self.connected = true;
                    }
                    StatusUpdate::PacketReceived => {
                        self.packet_count += 1;
                        let elapsed = self.start_time.elapsed().as_secs_f64();
                        let current_second = elapsed.floor() as u64;

                        // If we've moved to a new second, record the previous second's count
                        if current_second > self.last_second {
                            if self.current_second_count > 0 {
                                self.packet_history.push_back((
                                    self.last_second as f64,
                                    self.current_second_count,
                                ));
                            }
                            self.last_second = current_second;
                            self.current_second_count = 1;

                            // Keep only last 60 seconds of data
                            while let Some(&(time, _)) = self.packet_history.front() {
                                if current_second as f64 - time > 60.0 {
                                    self.packet_history.pop_front();
                                } else {
                                    break;
                                }
                            }

                            // Clear chart cache to trigger redraw
                            self.chart_cache.clear();
                        } else {
                            // Same second, just increment the counter
                            self.current_second_count += 1;
                        }
                    }
                    StatusUpdate::BallDetected(count) => {
                        self.last_ball_count = count;
                    }
                    StatusUpdate::RobotsDetected(count) => {
                        self.last_robot_count = count;
                    }
                    StatusUpdate::RobotPosition(id, team, position, orientation) => {
                        if GUI_LOG_EVERY_ROBOT_UPDATE {
                            eprintln!(
                                "[GUI] Recibida posición de robot: ID={}, team={}, pos=({:.2}, {:.2}) mm, orientación={:.2} rad",
                                id, team, position.x, position.y, orientation
                            );
                        }
                        self.robots.insert(
                            (team, id),
                            Robot {
                                id,
                                team,
                                position,
                                orientation,
                            },
                        );
                        self.field_cache.clear();
                        if GUI_LOG_EVERY_ROBOT_UPDATE {
                            eprintln!("[GUI] Total robots en mapa: {}", self.robots.len());
                        }
                    }
                    StatusUpdate::BallPosition(position) => {
                        self.ball = Some(Ball { position });
                        self.field_cache.clear();
                    }
                }
            }
            Message::ChangeIp(ip) => {
                self.vision_ip = ip;
            }
            Message::ChangePort(port) => {
                self.vision_port = port;
            }
            Message::Connect => {
                if let Ok(port) = self.vision_port.parse::<u16>()
                    && let Some(tx) = &self.config_tx
                {
                    let _ = tx.try_send(ConfigUpdate::ChangeIpPort(self.vision_ip.clone(), port));
                }
            }
            Message::Tick => {
                // Update chart even when no packets arrive
                let elapsed = self.start_time.elapsed().as_secs_f64();
                let current_second = elapsed.floor() as u64;

                // If we've moved to a new second, record the previous second's count
                if current_second > self.last_second {
                    // Record the count for the previous second (could be 0)
                    self.packet_history
                        .push_back((self.last_second as f64, self.current_second_count));

                    // Fill in any missing seconds with 0 packets
                    for sec in (self.last_second + 1)..current_second {
                        self.packet_history.push_back((sec as f64, 0));
                    }

                    self.last_second = current_second;
                    self.current_second_count = 0;

                    // Keep only last 60 seconds of data
                    while let Some(&(time, _)) = self.packet_history.front() {
                        if current_second as f64 - time > 60.0 {
                            self.packet_history.pop_front();
                        } else {
                            break;
                        }
                    }

                    // Clear chart cache to trigger redraw
                    self.chart_cache.clear();
                }
            }
            Message::TabSelected(tab) => {
                self.active_tab = tab;
            }
            Message::ToggleTracker(enabled) => {
                self.tracker_enabled = enabled;
                // Enviar comando al módulo Vision
                if let Some(tx) = &self.config_tx {
                    let _ = tx.try_send(ConfigUpdate::ToggleTracker(enabled));
                }
            }
        }
        Task::none()
    }

    fn subscription(&self) -> Subscription<Message> {
        let rx = self.status_rx.clone();

        let status_subscription = Subscription::run_with_id(
            "status_updates",
            stream::channel(100, move |mut output| async move {
                let receiver = {
                    let mut rx_lock = rx.lock().unwrap();
                    rx_lock.take()
                };

                if let Some(mut rx) = receiver {
                    while let Some(update) = rx.recv().await {
                        let _ = output.send(Message::StatusUpdate(update)).await;
                    }
                }

                loop {
                    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
                }
            }),
        );

        let tick_subscription =
            iced::time::every(std::time::Duration::from_millis(500)).map(|_| Message::Tick);

        Subscription::batch([status_subscription, tick_subscription])
    }

    fn theme(&self) -> Theme {
        Theme::Dark
    }

    fn view(&self) -> Element<'_, Message> {
        // Tab buttons
        let vision_button = button(text("Vision").size(14))
            .padding([8, 16])
            .style(if self.active_tab == TabView::Vision {
                button::primary
            } else {
                button::secondary
            })
            .on_press(Message::TabSelected(TabView::Vision));

        let robots_button = button(text("Robots").size(14))
            .padding([8, 16])
            .style(if self.active_tab == TabView::Robots {
                button::primary
            } else {
                button::secondary
            })
            .on_press(Message::TabSelected(TabView::Robots));

        let tabs = row![vision_button, robots_button]
            .spacing(5)
            .padding([8, 16]);

        // Content based on active tab
        let content = match self.active_tab {
            TabView::Vision => vision_status::view(
                self.connected,
                &self.vision_ip,
                &self.vision_port,
                self.packet_count,
                self.packet_frequency,
                self.last_ball_count,
                self.last_robot_count,
                &self.packet_history,
                &self.chart_cache,
                self.tracker_enabled,
            ),
            TabView::Robots => {
                let field = Canvas::new(FieldCanvas {
                    robots: &self.robots,
                    ball: &self.ball,
                    cache: &self.field_cache,
                })
                .width(Length::Fill)
                .height(Length::Fill);

                container(field)
                    .width(Length::Fill)
                    .height(Length::Fill)
                    .into()
            }
        };

        let main_content = column![tabs, content]
            .width(Length::Fill)
            .height(Length::Fill);

        container(main_content)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

pub fn run_gui(
    ip: String,
    port: u16,
    config_tx: mpsc::Sender<ConfigUpdate>,
    status_rx: mpsc::Receiver<StatusUpdate>,
) -> iced::Result {
    iced::application(VisionGui::title, VisionGui::update, VisionGui::view)
        .subscription(VisionGui::subscription)
        .theme(VisionGui::theme)
        .run_with(move || VisionGui::new(ip, port, config_tx, status_rx))
}
