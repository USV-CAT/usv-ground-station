//! The start dialog: the simulator, or a real boat over MAVLink.

use egui::{Context, RichText, Ui};

use crate::config::LinkSettings;
use crate::link::flows::Bearer;
use crate::link::mavlink::{self, Endpoint, PathSpec};
use crate::link::Choice;
use crate::ui::theme;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    UdpListen,
    Udp,
    Tcp,
    Serial,
}

impl Kind {
    const ALL: [Kind; 4] = [Kind::UdpListen, Kind::Udp, Kind::Tcp, Kind::Serial];

    fn label(self) -> &'static str {
        match self {
            Kind::UdpListen => "UDP listen",
            Kind::Udp => "UDP to",
            Kind::Tcp => "TCP",
            Kind::Serial => "Serial radio",
        }
    }

    fn hint(self) -> &'static str {
        match self {
            Kind::UdpListen => "The boat (or a Wi-Fi telemetry bridge) sends to this Deck. ArduPilot's usual port is 14550.",
            Kind::Udp => "Send to the boat at an address and port.",
            Kind::Tcp => "A TCP connection — ArduPilot SITL listens on 127.0.0.1:5760.",
            Kind::Serial => "A SiK or RFD telemetry radio on USB. SiK radios run at 57600 baud.",
        }
    }
}

const BAUDS: [u32; 5] = [57600, 115200, 230400, 460800, 921600];

pub struct ConnectDialog {
    /// The boat tab, not the simulator.
    pub boat: bool,
    /// The paths to the boat; the form below edits `paths[selected]`.
    pub paths: Vec<PathSpec>,
    pub selected: usize,
    pub bearer: Bearer,
    pub kind: Kind,
    pub port: String,
    pub address: String,
    pub device: String,
    pub baud: u32,
    pub serial_ports: Vec<String>,
    pub dont_ask: bool,
    pub error: Option<String>,
    /// Whether there is a boat to go back to (Cancel).
    pub can_cancel: bool,
    recent: Vec<String>,
}

pub enum Answer {
    Connect(Choice, bool),
    Cancel,
}

impl ConnectDialog {
    pub fn new(s: &LinkSettings, can_cancel: bool) -> ConnectDialog {
        let mut d = ConnectDialog {
            boat: s.kind == "mavlink",
            paths: Vec::new(),
            selected: 0,
            bearer: Bearer::Ip,
            kind: Kind::UdpListen,
            port: "14550".into(),
            address: "192.168.4.1:14550".into(),
            device: String::new(),
            baud: 57600,
            serial_ports: mavlink::serial_ports(),
            dont_ask: !s.ask_at_start,
            error: None,
            can_cancel,
            recent: s.recent.clone(),
        };
        if d.device_default() {
            d.device = d.serial_ports.first().cloned().unwrap_or_else(|| "/dev/ttyUSB0".into());
        }
        d.load(&s.mavlink_url);
        d
    }

    fn device_default(&self) -> bool {
        self.device.is_empty()
    }

    /// Fill the dialog from a connection string (one path or several).
    pub fn load(&mut self, url: &str) {
        self.paths = mavlink::parse_paths(url).unwrap_or_default();
        if self.paths.is_empty() {
            self.paths.push(self.form_path());
        }
        self.select(0);
    }

    fn select(&mut self, i: usize) {
        self.selected = i.min(self.paths.len() - 1);
        let p = self.paths[self.selected].clone();
        self.bearer = p.bearer;
        self.load_endpoint(p.endpoint);
    }

    fn load_endpoint(&mut self, endpoint: Endpoint) {
        match endpoint {
            Endpoint::UdpListen(p) => {
                self.kind = Kind::UdpListen;
                self.port = p.to_string();
            }
            Endpoint::Udp(a) => {
                self.kind = Kind::Udp;
                self.address = a;
            }
            Endpoint::Tcp(a) => {
                self.kind = Kind::Tcp;
                self.address = a;
            }
            Endpoint::Serial(d, b) => {
                self.kind = Kind::Serial;
                self.device = d;
                self.baud = b;
            }
        }
    }

    /// The path the form stands for.
    fn form_path(&self) -> PathSpec {
        let url = match self.kind {
            Kind::UdpListen => format!("udp-listen:{}", self.port.trim()),
            Kind::Udp => format!("udp:{}", self.address.trim()),
            Kind::Tcp => format!("tcp:{}", self.address.trim()),
            Kind::Serial => format!("serial:{}:{}", self.device.trim(), self.baud),
        };
        // A form not yet valid (a half-typed port) keeps a placeholder.
        let endpoint = Endpoint::parse(&url).unwrap_or(Endpoint::UdpListen(0));
        PathSpec { bearer: self.bearer, endpoint }
    }

    /// The connection string for every path.
    pub fn url(&self) -> String {
        let mut paths = self.paths.clone();
        if let Some(p) = paths.get_mut(self.selected) {
            *p = self.form_path();
        }
        paths.iter().map(PathSpec::to_url).collect::<Vec<_>>().join(", ")
    }

    pub fn show(&mut self, ctx: &Context) -> Option<Answer> {
        let mut answer = None;
        egui::Modal::new(egui::Id::new("connect")).show(ctx, |ui| {
            ui.set_width(540.0);
            ui.set_max_width(540.0);
            ui.label(RichText::new("CONNECT").font(theme::bold(13.0)).color(theme::ACCENT));
            ui.label(RichText::new("Which boat?").font(theme::bold(22.0)).color(theme::TEXT));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if card(ui, "🖥  Simulator", "Practise on a simulated catamaran in the Gulf of Riga: wind, waves, current and batteries.", !self.boat) {
                    self.boat = false;
                }
                if card(ui, "🛥  Boat", "A real boat: ArduPilot Rover over MAVLink, by radio, Wi-Fi or cable.", self.boat) {
                    self.boat = true;
                }
            });
            ui.add_space(10.0);
            if self.boat {
                self.boat_form(ui);
            }
            if let Some(e) = &self.error {
                ui.add_space(6.0);
                ui.label(RichText::new(format!("Could not connect: {e}")).color(theme::BAD));
            }
            ui.add_space(8.0);
            ui.checkbox(&mut self.dont_ask, "Connect like this at start, without asking");
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if self.can_cancel && ui.add_sized([120.0, 40.0], egui::Button::new("Cancel")).clicked() {
                    answer = Some(Answer::Cancel);
                }
                let label = if self.boat { "Connect to the boat" } else { "Start the simulator" };
                let go = egui::Button::new(RichText::new(label).font(theme::bold(16.0)).color(theme::INK)).fill(theme::GOOD);
                if ui.add_sized([240.0, 40.0], go).clicked() {
                    let choice = if self.boat { Choice::Mavlink(self.url()) } else { Choice::Simulator };
                    answer = Some(Answer::Connect(choice, self.dont_ask));
                }
            });
            if !self.boat && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                answer = Some(Answer::Connect(Choice::Simulator, self.dont_ask));
            }
        });
        answer
    }

    fn boat_form(&mut self, ui: &mut Ui) {
        // The paths: one, or several at once (LoRa and Wi-Fi and 5G).
        let current = self.form_path();
        self.paths[self.selected] = current;
        ui.label(RichText::new("PATHS TO THE BOAT").font(theme::bold(12.0)).color(theme::TEXT_DIM));
        let mut select = None;
        let mut remove = None;
        for (i, p) in self.paths.iter().enumerate() {
            ui.horizontal(|ui| {
                let text = format!("{}  {} · {}", p.bearer.icon(), p.bearer.label(), p.endpoint.describe());
                if ui.selectable_label(i == self.selected, RichText::new(text).font(theme::bold(14.0))).clicked() {
                    select = Some(i);
                }
                if self.paths.len() > 1 && ui.small_button("✕").on_hover_text("Remove this path").clicked() {
                    remove = Some(i);
                }
            });
        }
        if ui.button("+ Add a path").on_hover_text("Reach the boat several ways at once: LoRa, Wi-Fi, 5G …").clicked() {
            let port = 14550 + self.paths.len() as u16;
            self.paths.push(PathSpec { bearer: Bearer::WiFi, endpoint: Endpoint::UdpListen(port) });
            select = Some(self.paths.len() - 1);
        }
        if let Some(i) = remove {
            self.paths.remove(i);
            select = Some(self.selected.min(self.paths.len() - 1).saturating_sub(usize::from(i < self.selected)));
        }
        if let Some(i) = select {
            self.select(i);
        }
        ui.add_space(6.0);
        ui.label(RichText::new("CARRIED BY").font(theme::bold(12.0)).color(theme::TEXT_DIM));
        ui.horizontal_wrapped(|ui| {
            for b in Bearer::ALL {
                if ui.selectable_label(self.bearer == b, format!("{} {}", b.icon(), b.label())).clicked() {
                    self.bearer = b;
                }
            }
        });
        ui.horizontal(|ui| {
            for k in Kind::ALL {
                if ui.selectable_label(self.kind == k, RichText::new(k.label()).font(theme::bold(14.0))).clicked() {
                    self.kind = k;
                    // A serial port is most likely a radio.
                    if k == Kind::Serial && matches!(self.bearer, Bearer::Ip | Bearer::WiFi) {
                        self.bearer = Bearer::Radio;
                    }
                    // …and a network connection is no serial radio.
                    if k != Kind::Serial && self.bearer == Bearer::Radio {
                        self.bearer = Bearer::Ip;
                    }
                }
            }
        });

        ui.label(RichText::new(self.kind.hint()).color(theme::TEXT_DIM));
        ui.add_space(4.0);
        egui::Grid::new("connect-form").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
            match self.kind {
                Kind::UdpListen => {
                    ui.label("Port");
                    ui.add(egui::TextEdit::singleline(&mut self.port).desired_width(100.0));
                }
                Kind::Udp | Kind::Tcp => {
                    ui.label("Address");
                    ui.add(egui::TextEdit::singleline(&mut self.address).hint_text("host:port").desired_width(240.0));
                }
                Kind::Serial => {
                    ui.label("Device");
                    ui.horizontal(|ui| {
                        egui::ComboBox::from_id_salt("serial-device")
                            .width(260.0)
                            .selected_text(self.device.clone())
                            .show_ui(ui, |ui| {
                                if self.serial_ports.is_empty() {
                                    ui.label("No serial devices found — plug the radio in");
                                }
                                for p in self.serial_ports.clone() {
                                    ui.selectable_value(&mut self.device, p.clone(), p);
                                }
                            });
                        if ui.button("⟳").on_hover_text("Look for devices again").clicked() {
                            self.serial_ports = mavlink::serial_ports();
                        }
                    });
                    ui.end_row();
                    ui.label("Baud");
                    egui::ComboBox::from_id_salt("serial-baud").selected_text(self.baud.to_string()).show_ui(ui, |ui| {
                        for b in BAUDS {
                            ui.selectable_value(&mut self.baud, b, b.to_string());
                        }
                    });
                }
            }
            ui.end_row();
        });
        if !self.recent.is_empty() {
            ui.add_space(6.0);
            ui.label(RichText::new("RECENT").font(theme::bold(12.0)).color(theme::TEXT_DIM));
            ui.horizontal_wrapped(|ui| {
                for url in self.recent.clone() {
                    let text = mavlink::describe_paths(&url);
                    if ui.button(text).clicked() {
                        self.load(&url);
                    }
                }
            });
        }
        ui.add_space(4.0);
        ui.add(egui::Label::new(
            RichText::new("The station drives the boat in MANUAL by RC override and runs the autopilot here. If the link drops, the boat's own failsafe stops it.")
                .small()
                .color(theme::TEXT_DIM),
        ).wrap());
    }
}

/// A big choice button.
fn card(ui: &mut Ui, title: &str, text: &str, selected: bool) -> bool {
    let stroke = if selected { egui::Stroke::new(2.0, theme::ACCENT) } else { egui::Stroke::new(1.0, theme::BORDER) };
    let response = egui::Frame::new()
        .fill(if selected { theme::SURFACE_RAISED } else { theme::SURFACE })
        .stroke(stroke)
        .corner_radius(8.0)
        .inner_margin(10)
        .show(ui, |ui| {
            ui.set_width(232.0);
            ui.set_height(78.0);
            ui.vertical(|ui| {
                ui.label(RichText::new(title).font(theme::bold(17.0)).color(if selected { theme::ACCENT } else { theme::TEXT }));
                ui.add(egui::Label::new(RichText::new(text).color(theme::TEXT_DIM)).wrap());
            });
        })
        .response;
    let response = ui.interact(response.rect, ui.id().with(title), egui::Sense::click());
    response.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, true, selected, title.split_whitespace().last().unwrap_or(title)));
    response.clicked()
}
