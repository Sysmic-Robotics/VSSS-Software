use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, KeyEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Paragraph, Sparkline},
    Frame, Terminal,
};
use std::io;
use tokio::sync::mpsc;
use std::time::{Duration, Instant};
use crate::field::FieldView;
use glam::Vec2;

#[derive(Debug, Clone)]
pub enum StatusUpdate {
    Connected(String, u16),
    PacketReceived,
    BallDetected(usize),
    RobotsDetected(usize),
    RobotPosition(u32, u32, Vec2, f32), // id, team, position, orientation
    BallPosition(Vec2),
}

#[derive(Debug, Clone)]
pub enum ConfigUpdate {
    ChangeIpPort(String, u16),
}

#[derive(Debug, Clone, PartialEq)]
enum InputMode {
    Normal,
    EditingIp,
    EditingPort,
}

#[derive(Debug, Clone, PartialEq)]
enum Tab {
    Status,
    Field,
}

pub struct AppState {
    pub vision_ip: String,
    pub vision_port: u16,
    pub connected: bool,
    pub packet_frequency: f64,
    pub packets_in_window: u64,
    pub last_frequency_update: Instant,
    pub last_ball_count: usize,
    pub last_robot_count: usize,
    pub input_mode: InputMode,
    pub input_buffer: String,
    pub packet_history: Vec<u64>,  // Packets per second for each time window
    pub history_start: Instant,
    pub max_history_size: usize,
    pub current_tab: Tab,
    pub field_view: FieldView,
}

impl AppState {
    pub fn new(ip: String, port: u16) -> Self {
        Self {
            vision_ip: ip,
            vision_port: port,
            connected: false,
            packet_frequency: 0.0,
            packets_in_window: 0,
            last_frequency_update: Instant::now(),
            last_ball_count: 0,
            last_robot_count: 0,
            input_mode: InputMode::Normal,
            input_buffer: String::new(),
            packet_history: vec![0; 60],  // 60 seconds of history
            history_start: Instant::now(),
            max_history_size: 60,
            current_tab: Tab::Status,
            field_view: FieldView::new(),
        }
    }

    pub fn update(&mut self, status: StatusUpdate) {
        match status {
            StatusUpdate::Connected(ip, port) => {
                self.vision_ip = ip;
                self.vision_port = port;
                self.connected = true;
            }
            StatusUpdate::PacketReceived => {
                self.packets_in_window += 1;
            }
            StatusUpdate::BallDetected(count) => {
                self.last_ball_count = count;
            }
            StatusUpdate::RobotsDetected(count) => {
                self.last_robot_count = count;
            }
            StatusUpdate::RobotPosition(id, team, position, orientation) => {
                self.field_view.update_robot(id, team, position, orientation);
            }
            StatusUpdate::BallPosition(position) => {
                self.field_view.update_ball(position);
            }
        }
    }

    pub fn update_frequency(&mut self) {
        let elapsed = self.last_frequency_update.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            self.packet_frequency = self.packets_in_window as f64 / elapsed;
            
            // Add to history
            self.packet_history.remove(0);
            self.packet_history.push(self.packets_in_window);
            
            self.packets_in_window = 0;
            self.last_frequency_update = Instant::now();
        }
    }
}

fn render_status_tab(f: &mut Frame, app_state: &AppState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .margin(2)
        .constraints([
            Constraint::Length(3),
            Constraint::Length(3),
            Constraint::Length(3),
            Constraint::Length(3),
            Constraint::Length(8),  // Chart section
            Constraint::Length(3),
            Constraint::Min(0),
        ])
        .split(f.area());

    // Title
    let title = Paragraph::new("RustEngine - Vision System Monitor (Press Tab to switch)")
        .style(Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))
        .block(Block::default().borders(Borders::ALL));
    f.render_widget(title, chunks[0]);

            // Connection Status
            let connection_status = if app_state.connected {
                Span::styled("CONNECTED", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD))
            } else {
                Span::styled("DISCONNECTED", Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))
            };
            
            let connection_info = Paragraph::new(Line::from(vec![
                Span::raw("Vision Status: "),
                connection_status,
            ]))
            .block(Block::default().borders(Borders::ALL).title("Connection"));
            f.render_widget(connection_info, chunks[1]);

            // IP and Port
            let _ip_style = if app_state.input_mode == InputMode::EditingIp {
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
            } else {
                Style::default()
            };
            let _port_style = if app_state.input_mode == InputMode::EditingPort {
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)
            } else {
                Style::default()
            };

            let network_text = match app_state.input_mode {
                InputMode::EditingIp => format!("IP: {} [EDITING]  |  Port: {}", app_state.input_buffer, app_state.vision_port),
                InputMode::EditingPort => format!("IP: {}  |  Port: {} [EDITING]", app_state.vision_ip, app_state.input_buffer),
                InputMode::Normal => format!("IP: {}  |  Port: {}", app_state.vision_ip, app_state.vision_port),
            };

            let network_info = Paragraph::new(network_text)
                .style(if app_state.input_mode != InputMode::Normal { Style::default().fg(Color::Yellow) } else { Style::default() })
                .block(Block::default().borders(Borders::ALL).title("Network (Press 'i' for IP, 'p' for Port)"));
            f.render_widget(network_info, chunks[2]);

            // Packets Received
            let packets_color = if app_state.packet_frequency > 0.0 {
                Color::Green
            } else {
                Color::Yellow
            };
            
            let packets_info = Paragraph::new(Line::from(vec![
                Span::raw("Packet Frequency: "),
                Span::styled(
                    format!("{:.1} Hz", app_state.packet_frequency),
                    Style::default().fg(packets_color).add_modifier(Modifier::BOLD)
                ),
            ]))
            .block(Block::default().borders(Borders::ALL).title("Data"));
            f.render_widget(packets_info, chunks[3]);

            // Packet History Chart
            let sparkline = Sparkline::default()
                .block(Block::default().borders(Borders::ALL).title("Packets/s over last 60s"))
                .data(&app_state.packet_history)
                .style(Style::default().fg(Color::Cyan))
                .max(app_state.packet_history.iter().max().copied().unwrap_or(1));
            f.render_widget(sparkline, chunks[4]);

            // Detection Info
            let detection_info = Paragraph::new(format!(
                "Balls: {}  |  Robots: {}",
                app_state.last_ball_count, app_state.last_robot_count
            ))
            .block(Block::default().borders(Borders::ALL).title("Detection"));
            f.render_widget(detection_info, chunks[5]);

            // Help
            let help_text = match app_state.input_mode {
                InputMode::Normal => "Press 'i' to edit IP, 'p' to edit Port, 'q' to quit",
                InputMode::EditingIp | InputMode::EditingPort => "Type value, press Enter to confirm, Esc to cancel",
            };
            let help = Paragraph::new(help_text)
                .style(Style::default().fg(Color::Gray))
                .block(Block::default().borders(Borders::ALL).title("Help"));
            f.render_widget(help, chunks[6]);
}

pub async fn run_ui(mut rx: mpsc::Receiver<StatusUpdate>, config_tx: mpsc::Sender<ConfigUpdate>, ip: String, port: u16) -> io::Result<()> {
    // Setup terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app_state = AppState::new(ip, port);
    let mut should_quit = false;

    while !should_quit {
        // Draw UI
        terminal.draw(|f| {
            match app_state.current_tab {
                Tab::Status => render_status_tab(f, &app_state),
                Tab::Field => {
                    app_state.field_view.render(f, f.area());
                }
            }
        })?;

        // Handle input and status updates with timeout
        if event::poll(Duration::from_millis(100))? {
            if let Event::Key(key) = event::read()? {
                // Only process key press events, ignore key release and repeat
                if key.kind != KeyEventKind::Press {
                    // Check for status updates and continue
                    while let Ok(status) = rx.try_recv() {
                        app_state.update(status);
                    }
                    app_state.update_frequency();
                    continue;
                }
                
                match app_state.input_mode {
                    InputMode::Normal => {
                        match key.code {
                            KeyCode::Char('q') => should_quit = true,
                            KeyCode::Tab => {
                                app_state.current_tab = match app_state.current_tab {
                                    Tab::Status => Tab::Field,
                                    Tab::Field => Tab::Status,
                                };
                            }
                            KeyCode::Char('i') => {
                                app_state.input_mode = InputMode::EditingIp;
                                app_state.input_buffer = app_state.vision_ip.clone();
                            }
                            KeyCode::Char('p') => {
                                app_state.input_mode = InputMode::EditingPort;
                                app_state.input_buffer = app_state.vision_port.to_string();
                            }
                            _ => {}
                        }
                    }
                    InputMode::EditingIp => {
                        match key.code {
                            KeyCode::Enter => {
                                let new_ip = app_state.input_buffer.clone();
                                let new_port = app_state.vision_port;
                                app_state.vision_ip = new_ip.clone();
                                app_state.connected = false;
                                let _ = config_tx.send(ConfigUpdate::ChangeIpPort(new_ip, new_port)).await;
                                app_state.input_mode = InputMode::Normal;
                                app_state.input_buffer.clear();
                            }
                            KeyCode::Esc => {
                                app_state.input_mode = InputMode::Normal;
                                app_state.input_buffer.clear();
                            }
                            KeyCode::Char(c) => {
                                app_state.input_buffer.push(c);
                            }
                            KeyCode::Backspace => {
                                app_state.input_buffer.pop();
                            }
                            _ => {}
                        }
                    }
                    InputMode::EditingPort => {
                        match key.code {
                            KeyCode::Enter => {
                                if let Ok(new_port) = app_state.input_buffer.parse::<u16>() {
                                    let new_ip = app_state.vision_ip.clone();
                                    app_state.vision_port = new_port;
                                    app_state.connected = false;
                                    let _ = config_tx.send(ConfigUpdate::ChangeIpPort(new_ip, new_port)).await;
                                }
                                app_state.input_mode = InputMode::Normal;
                                app_state.input_buffer.clear();
                            }
                            KeyCode::Esc => {
                                app_state.input_mode = InputMode::Normal;
                                app_state.input_buffer.clear();
                            }
                            KeyCode::Char(c) if c.is_ascii_digit() => {
                                app_state.input_buffer.push(c);
                            }
                            KeyCode::Backspace => {
                                app_state.input_buffer.pop();
                            }
                            _ => {}
                        }
                    }
                }
            }
        }

        // Check for status updates (non-blocking)
        while let Ok(status) = rx.try_recv() {
            app_state.update(status);
        }

        // Update frequency calculation
        app_state.update_frequency();
    }

    // Restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    Ok(())
}
