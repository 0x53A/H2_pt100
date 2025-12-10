use anyhow::{Context, Result};
use chrono::Utc;
use clap::Parser;
use common::UartOutput;
use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, List, ListItem, ListState, Paragraph},
    Frame, Terminal,
};
use std::{
    fs::{File, OpenOptions},
    io::{self, stdout, BufRead, BufReader, Write},
    path::PathBuf,
    time::{Duration, Instant},
};

const COMMON_BAUD_RATES: &[u32] = &[9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port path (e.g., /dev/ttyACM0)
    #[arg(short, long)]
    port: Option<String>,

    /// Baud rate
    #[arg(short, long)]
    baud: Option<u32>,

    /// Log file directory (defaults to current directory)
    #[arg(short, long)]
    log_dir: Option<PathBuf>,
}

#[derive(Clone, Copy, PartialEq)]
enum SelectionMode {
    Port,
    Baud,
}

#[derive(Clone, Copy, PartialEq)]
enum AppState {
    SelectingPort,
    SelectingBaud,
    Connected,
    Disconnected,
}

struct App {
    state: AppState,
    port: Option<String>,
    baud: Option<u32>,
    log_dir: PathBuf,
    log_file: Option<File>,
    last_output: Option<UartOutput>,
    last_received: Option<Instant>,
    available_ports: Vec<String>,
    list_state: ListState,
    error_counter: u32,
    status_message: String,
    last_flush: Instant,
}

impl App {
    fn new(args: Args) -> Self {
        let log_dir = args.log_dir.unwrap_or_else(|| std::env::current_dir().unwrap_or_default());
        
        let initial_state = if args.port.is_none() {
            AppState::SelectingPort
        } else if args.baud.is_none() {
            AppState::SelectingBaud
        } else {
            AppState::Disconnected
        };

        let mut app = App {
            state: initial_state,
            port: args.port,
            baud: args.baud,
            log_dir,
            log_file: None,
            last_output: None,
            last_received: None,
            available_ports: Vec::new(),
            list_state: ListState::default(),
            error_counter: 0,
            status_message: String::new(),
            last_flush: Instant::now(),
        };
        
        if app.state == AppState::SelectingPort {
            app.refresh_ports();
        }
        
        app
    }

    fn refresh_ports(&mut self) {
        self.available_ports = serialport::available_ports()
            .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default();
        
        if !self.available_ports.is_empty() && self.list_state.selected().is_none() {
            self.list_state.select(Some(0));
        }
    }

    fn select_next(&mut self) {
        let len = match self.state {
            AppState::SelectingPort => self.available_ports.len(),
            AppState::SelectingBaud => COMMON_BAUD_RATES.len(),
            _ => return,
        };
        
        if len == 0 {
            return;
        }
        
        let i = match self.list_state.selected() {
            Some(i) => (i + 1) % len,
            None => 0,
        };
        self.list_state.select(Some(i));
    }

    fn select_previous(&mut self) {
        let len = match self.state {
            AppState::SelectingPort => self.available_ports.len(),
            AppState::SelectingBaud => COMMON_BAUD_RATES.len(),
            _ => return,
        };
        
        if len == 0 {
            return;
        }
        
        let i = match self.list_state.selected() {
            Some(i) => {
                if i == 0 {
                    len - 1
                } else {
                    i - 1
                }
            }
            None => 0,
        };
        self.list_state.select(Some(i));
    }

    fn confirm_selection(&mut self) {
        match self.state {
            AppState::SelectingPort => {
                if let Some(i) = self.list_state.selected() {
                    if let Some(port) = self.available_ports.get(i) {
                        self.port = Some(port.clone());
                        if self.baud.is_none() {
                            self.state = AppState::SelectingBaud;
                            self.list_state.select(Some(4)); // Default to 115200
                        } else {
                            self.state = AppState::Disconnected;
                        }
                    }
                }
            }
            AppState::SelectingBaud => {
                if let Some(i) = self.list_state.selected() {
                    if let Some(&baud) = COMMON_BAUD_RATES.get(i) {
                        self.baud = Some(baud);
                        self.state = AppState::Disconnected;
                    }
                }
            }
            _ => {}
        }
    }

    fn open_log_file(&mut self) -> Result<()> {
        let date = Utc::now().format("%Y-%m-%d_%H-%M-%S");
        let filename = format!("{}-temperatures.tsv", date);
        let path = self.log_dir.join(&filename);
        
        let file_exists = path.exists();
        
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
            .with_context(|| format!("Failed to open log file: {:?}", path))?;
        
        self.log_file = Some(file);
        
        // Write header if file is new
        if !file_exists {
            self.write_tsv_header()?;
        }
        
        self.status_message = format!("Logging to: {:?}", path);
        Ok(())
    }

    fn write_tsv_header(&mut self) -> Result<()> {
        if let Some(ref mut file) = self.log_file {
            writeln!(
                file,
                "timestamp\tboot_id\t\
                sampling_resolution_bits\tsampling_sample_time_cycles\tsampling_oversampling\tsampling_n_measurements\tsampling_amplification\t\
                pt100_1_series_resistor_ohms\tpt100_2_series_resistor_ohms\t\
                raw_pt100_1_p20\traw_pt100_1_median\traw_pt100_1_p80\t\
                raw_pt100_2_p20\traw_pt100_2_median\traw_pt100_2_p80\t\
                pt100_1_r_pt\tpt100_1_temperature\t\
                pt100_2_r_pt\tpt100_2_temperature"
            )?;
        }
        Ok(())
    }

    fn write_tsv_row(&mut self, output: &UartOutput) -> Result<()> {
        if let Some(ref mut file) = self.log_file {
            let timestamp = Utc::now().format("%Y-%m-%dT%H:%M:%S%.3fZ");
            writeln!(
                file,
                "{}\t{}\t\
                {}\t{}\t{}\t{}\t{}\t\
                {}\t{}\t\
                {}\t{}\t{}\t\
                {}\t{}\t{}\t\
                {}\t{}\t\
                {}\t{}",
                timestamp,
                output.boot_id,
                output.config.sampling.resolution_bits,
                output.config.sampling.sample_time_cycles,
                output.config.sampling.oversampling,
                output.config.sampling.n_measurements,
                output.config.sampling.amplification,
                output.config.pt100_1_series_resistor_ohms,
                output.config.pt100_2_series_resistor_ohms,
                output.raw.pt100_1.p20,
                output.raw.pt100_1.median,
                output.raw.pt100_1.p80,
                output.raw.pt100_2.p20,
                output.raw.pt100_2.median,
                output.raw.pt100_2.p80,
                output.calculated.pt100_1.r_pt,
                output.calculated.pt100_1.temperature,
                output.calculated.pt100_2.r_pt,
                output.calculated.pt100_2.temperature,
            )?;
        }
        Ok(())
    }

    fn flush_if_needed(&mut self) {
        if self.last_flush.elapsed() >= Duration::from_secs(1) {
            self.flush();
        }
    }

    fn flush(&mut self) {
        if let Some(ref mut file) = self.log_file {
            let _ = file.flush();
        }
        self.last_flush = Instant::now();
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut app = App::new(args);

    // Setup terminal
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout());
    let mut terminal = Terminal::new(backend)?;

    let result = run_app(&mut terminal, &mut app);

    // Restore terminal
    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    result
}

fn run_app<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>, app: &mut App) -> Result<()> {
    let mut serial_reader: Option<BufReader<Box<dyn serialport::SerialPort>>> = None;
    let mut line_buffer = String::new();

    loop {
        terminal.draw(|f| ui(f, app))?;

        // Handle connection attempts in Disconnected state
        if app.state == AppState::Disconnected && serial_reader.is_none() {
            if let (Some(port), Some(baud)) = (&app.port, app.baud) {
                match serialport::new(port, baud)
                    .timeout(Duration::from_millis(100))
                    .open()
                {
                    Ok(port) => {
                        serial_reader = Some(BufReader::with_capacity(20_000, port));
                        app.state = AppState::Connected;
                        app.status_message = format!("Connected to {} @ {}", app.port.as_ref().unwrap(), baud);
                        
                        if app.log_file.is_none() {
                            if let Err(e) = app.open_log_file() {
                                app.status_message = format!("Error opening log: {}", e);
                            }
                        }
                    }
                    Err(e) => {
                        app.status_message = format!("Connection failed: {} - retrying...", e);
                        app.refresh_ports();
                    }
                }
            }
        }

        // Read from serial if connected
        if app.state == AppState::Connected {
            if let Some(ref mut reader) = serial_reader {
                line_buffer.clear();
                match reader.read_line(&mut line_buffer) {
                    Ok(0) => {
                        // EOF - disconnected
                        app.state = AppState::Disconnected;
                        app.status_message = "Disconnected (EOF)".to_string();
                        serial_reader = None;
                        app.flush();
                    }
                    Ok(_) => {
                        let line = line_buffer.trim();
                        if !line.is_empty() {
                            match serde_json::from_str::<UartOutput>(line) {
                                Ok(output) => {
                                    if let Err(e) = app.write_tsv_row(&output) {
                                        app.error_counter += 1;
                                        app.status_message = format!("[{}] Write error: {}", app.error_counter, e);
                                    }
                                    app.last_output = Some(output);
                                    app.last_received = Some(Instant::now());
                                }
                                Err(e) => {
                                    app.error_counter += 1;
                                    app.status_message = format!("[{}] Parse error: {} ({})", app.error_counter, e, line[..20].to_string());
                                }
                            }
                        }
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                        // Normal timeout, continue
                    }
                    Err(e) => {
                        app.state = AppState::Disconnected;
                        app.error_counter += 1;
                        app.status_message = format!("[{}] Read error: {}", app.error_counter, e);
                        serial_reader = None;
                        app.flush();
                    }
                }
            }
        }

        app.flush_if_needed();

        // Handle keyboard events (with timeout to not block serial reading)
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key) = event::read()? {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => {
                            app.flush();
                            return Ok(());
                        }
                        KeyCode::Up | KeyCode::Char('k') => app.select_previous(),
                        KeyCode::Down | KeyCode::Char('j') => app.select_next(),
                        KeyCode::Enter => app.confirm_selection(),
                        KeyCode::Char('r') => {
                            if app.state == AppState::SelectingPort {
                                app.refresh_ports();
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
}

fn ui(f: &mut Frame, app: &App) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // Status bar
            Constraint::Min(10),   // Main content
            Constraint::Length(3), // Help bar
        ])
        .split(f.area());

    // Status bar
    let status_text = match app.state {
        AppState::SelectingPort => "Select Serial Port".to_string(),
        AppState::SelectingBaud => "Select Baud Rate".to_string(),
        AppState::Connected => format!(
            "Connected: {} @ {}",
            app.port.as_ref().unwrap_or(&"?".to_string()),
            app.baud.unwrap_or(0)
        ),
        AppState::Disconnected => format!(
            "Disconnected: {} @ {} - Waiting for reconnect...",
            app.port.as_ref().unwrap_or(&"?".to_string()),
            app.baud.unwrap_or(0)
        ),
    };
    
    let status_style = match app.state {
        AppState::Connected => Style::default().fg(Color::Green),
        AppState::Disconnected => Style::default().fg(Color::Yellow),
        _ => Style::default().fg(Color::Cyan),
    };
    
    let status = Paragraph::new(status_text)
        .style(status_style)
        .block(Block::default().borders(Borders::ALL).title("Status"));
    f.render_widget(status, chunks[0]);

    // Main content area
    match app.state {
        AppState::SelectingPort => {
            render_port_selection(f, app, chunks[1]);
        }
        AppState::SelectingBaud => {
            render_baud_selection(f, app, chunks[1]);
        }
        AppState::Connected | AppState::Disconnected => {
            render_temperature_display(f, app, chunks[1]);
        }
    }

    // Help bar
    let help_text = match app.state {
        AppState::SelectingPort => "↑/↓: Navigate | Enter: Select | r: Refresh | q: Quit",
        AppState::SelectingBaud => "↑/↓: Navigate | Enter: Select | q: Quit",
        _ => "q: Quit",
    };
    let help = Paragraph::new(help_text)
        .style(Style::default().fg(Color::DarkGray))
        .block(Block::default().borders(Borders::ALL).title("Help"));
    f.render_widget(help, chunks[2]);
}

fn render_port_selection(f: &mut Frame, app: &App, area: Rect) {
    let items: Vec<ListItem> = app
        .available_ports
        .iter()
        .map(|p| ListItem::new(p.as_str()))
        .collect();

    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title("Available Ports"))
        .highlight_style(
            Style::default()
                .bg(Color::Blue)
                .add_modifier(Modifier::BOLD),
        )
        .highlight_symbol(">> ");

    let mut state = app.list_state.clone();
    f.render_stateful_widget(list, area, &mut state);
}

fn render_baud_selection(f: &mut Frame, app: &App, area: Rect) {
    let items: Vec<ListItem> = COMMON_BAUD_RATES
        .iter()
        .map(|&b| ListItem::new(format!("{} baud", b)))
        .collect();

    let list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title("Select Baud Rate"))
        .highlight_style(
            Style::default()
                .bg(Color::Blue)
                .add_modifier(Modifier::BOLD),
        )
        .highlight_symbol(">> ");

    let mut state = app.list_state.clone();
    f.render_stateful_widget(list, area, &mut state);
}

fn render_temperature_display(f: &mut Frame, app: &App, area: Rect) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(5),  // Temperatures
            Constraint::Length(3),  // Time since last
            Constraint::Length(6),  // Raw data
            Constraint::Min(3),     // Status message
        ])
        .split(area);

    // Temperature display
    let temp_text = if let Some(ref output) = app.last_output {
        vec![
            Line::from(vec![
                Span::styled("PT100 #1: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!("{:.2}°C", output.calculated.pt100_1.temperature),
                    Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
                ),
                Span::raw(format!("  (R = {:.2}Ω)", output.calculated.pt100_1.r_pt)),
            ]),
            Line::from(vec![
                Span::styled("PT100 #2: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!("{:.2}°C", output.calculated.pt100_2.temperature),
                    Style::default().fg(Color::White).add_modifier(Modifier::BOLD),
                ),
                Span::raw(format!("  (R = {:.2}Ω)", output.calculated.pt100_2.r_pt)),
            ]),
            Line::from(vec![
                Span::styled("Boot ID: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{}", output.boot_id)),
            ]),
        ]
    } else {
        vec![Line::from("Waiting for data...")]
    };

    let temps = Paragraph::new(temp_text)
        .block(Block::default().borders(Borders::ALL).title("Temperatures"));
    f.render_widget(temps, chunks[0]);

    // Time since last received
    let time_text = if let Some(last) = app.last_received {
        let elapsed = last.elapsed();
        let secs = elapsed.as_secs();
        if secs < 60 {
            format!("{} seconds ago", secs)
        } else {
            format!("{}:{:02} ago", secs / 60, secs % 60)
        }
    } else {
        "Never".to_string()
    };
    
    let time_style = if let Some(last) = app.last_received {
        if last.elapsed() > Duration::from_secs(5) {
            Style::default().fg(Color::Yellow)
        } else {
            Style::default().fg(Color::Green)
        }
    } else {
        Style::default().fg(Color::DarkGray)
    };

    let time_widget = Paragraph::new(time_text)
        .style(time_style)
        .block(Block::default().borders(Borders::ALL).title("Last Received"));
    f.render_widget(time_widget, chunks[1]);

    // Raw data display
    let raw_text = if let Some(ref output) = app.last_output {
        vec![
            Line::from(vec![
                Span::styled("Raw PT100 #1: ", Style::default().fg(Color::Cyan)),
                Span::raw(format!(
                    "p20={}, median={}, p80={}",
                    output.raw.pt100_1.p20, output.raw.pt100_1.median, output.raw.pt100_1.p80
                )),
            ]),
            Line::from(vec![
                Span::styled("Raw PT100 #2: ", Style::default().fg(Color::Cyan)),
                Span::raw(format!(
                    "p20={}, median={}, p80={}",
                    output.raw.pt100_2.p20, output.raw.pt100_2.median, output.raw.pt100_2.p80
                )),
            ]),
            Line::from(vec![
                Span::styled("Sampling: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!(
                    "{}bit, {}cyc, {}x oversample, {}meas, {}x amp",
                    output.config.sampling.resolution_bits,
                    output.config.sampling.sample_time_cycles,
                    output.config.sampling.oversampling,
                    output.config.sampling.n_measurements,
                    output.config.sampling.amplification
                )),
            ]),
            Line::from(vec![
                Span::styled("Series R: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!(
                    "PT100#1={}Ω, PT100#2={}Ω",
                    output.config.pt100_1_series_resistor_ohms,
                    output.config.pt100_2_series_resistor_ohms
                )),
            ]),
        ]
    } else {
        vec![Line::from("No data yet")]
    };

    let raw_widget = Paragraph::new(raw_text)
        .style(Style::default().fg(Color::White))
        .block(Block::default().borders(Borders::ALL).title("Raw Data"));
    f.render_widget(raw_widget, chunks[2]);

    // Status message
    let status = Paragraph::new(app.status_message.as_str())
        .style(Style::default().fg(Color::DarkGray))
        .block(Block::default().borders(Borders::ALL).title("Log"));
    f.render_widget(status, chunks[3]);
}

