//! Command line.
//!
//! ```text
//! usvgs                 the ground station (asks: simulator or boat)
//! usvgs --sim           straight onto the simulator
//! usvgs --connect udp-listen:14550   straight to a boat over MAVLink
//! usvgs --windowed      not full screen, even on a small display
//! usvgs pads            list gamepads and show live stick values (test the Deck's controls)
//! usvgs chart ...       build an offline depth chart from EMODnet
//! usvgs osm ...         build an offline street map from an OpenStreetMap extract
//! ```

use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};

use usvgs::config;
use usvgs::input::gamepad;
use usvgs::tools::{bathy_build, osm_build};

#[derive(Parser)]
#[command(name = "usvgs", version, about = "Portable USV ground station")]
struct Cli {
    /// Never go full screen.
    #[arg(long)]
    windowed: bool,
    /// Always go full screen.
    #[arg(long, conflicts_with = "windowed")]
    fullscreen: bool,
    /// Start from default settings.
    #[arg(long)]
    reset_settings: bool,
    /// The view to start in: a chart view, or the power-flow diagram.
    #[arg(long, value_parser = ["north", "heading", "chase", "power"])]
    view: Option<String>,
    /// Start on the simulator, without the start dialog.
    #[arg(long)]
    sim: bool,
    /// Connect to a real boat over MAVLink, without the start dialog:
    /// udp-listen:14550, udp:HOST:PORT, tcp:HOST:PORT or serial:/dev/ttyUSB0:57600.
    #[arg(long, conflicts_with = "sim", value_name = "CONNECTION")]
    connect: Option<String>,
    /// A mission file to open at start.
    #[arg(long)]
    mission: Option<PathBuf>,
    #[arg(short, long)]
    verbose: bool,
    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// List gamepads and show their live state.
    Pads {
        #[arg(long, default_value_t = 30.0)]
        seconds: f64,
    },
    /// Build an offline depth chart (EMODnet Bathymetry → .npz).
    Chart {
        /// west,south,east,north in degrees: fetch this from EMODnet.
        #[arg(long, conflicts_with = "tif", allow_hyphen_values = true)]
        bbox: Option<String>,
        /// An EMODnet GeoTIFF already downloaded.
        #[arg(long)]
        tif: Option<PathBuf>,
        #[arg(long)]
        out: PathBuf,
    },
    /// Fetch the voice packs (EdgeTX sounds, GPL-2.0) for the spoken alerts.
    Sounds {
        /// Languages to fetch: en, ru.
        #[arg(long, num_args = 1.., default_values_t = ["en".to_string(), "ru".to_string()])]
        lang: Vec<String>,
        /// The EdgeTX voice to take English from: en, en_us-sara, en_gb-libby, …
        #[arg(long, default_value = "en")]
        english_voice: String,
    },
    /// Play one spoken alert: shallow, aground, link-lost, battery-critical, armed, …
    Say {
        alert: String,
        #[arg(long, default_value = "en")]
        lang: String,
    },
    /// Build an offline street map from an OpenStreetMap .osm.pbf extract.
    Osm {
        #[arg(long)]
        pbf: PathBuf,
        /// Output prefix: writes <out>.idx.json and <out>.bin.
        #[arg(long, default_value = "maps/latvia_map")]
        out: PathBuf,
        #[arg(long, default_value_t = 0.05)]
        tile: f64,
        /// Include buildings (more memory, a bigger map).
        #[arg(long)]
        buildings: bool,
    },
}

fn pads(seconds: f64) -> i32 {
    let found = gamepad::find_evdev_pads();
    if found.is_empty() {
        println!("no evdev gamepads readable (check /dev/input permissions)");
    } else {
        println!("evdev gamepads:");
        for (path, name) in &found {
            println!("  {path}  {name}");
        }
    }
    let Some(mut backend) = gamepad::open_backend("auto") else {
        println!("no input backend could start");
        return 1;
    };
    println!("reading with the {} backend for {seconds:.0} s — move the sticks, press buttons", backend.name);
    let end = Instant::now() + Duration::from_secs_f64(seconds);
    while Instant::now() < end {
        let s = backend.state();
        print!(
            "\r{} L({:+.2},{:+.2}) R({:+.2},{:+.2}) LT {:.2} RT {:.2}  {:<30}",
            if s.connected { '●' } else { '○' },
            s.lx,
            s.ly,
            s.rx,
            s.ry,
            s.lt,
            s.rt,
            s.held_names()
        );
        let _ = std::io::stdout().flush();
        std::thread::sleep(Duration::from_millis(50));
    }
    backend.stop();
    println!();
    0
}

fn main() {
    let cli = Cli::parse();
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(if cli.verbose { "debug" } else { "info,egui=error" }))
        .format_timestamp_secs()
        .init();

    let code = match cli.command {
        Some(Command::Pads { seconds }) => pads(seconds),
        Some(Command::Chart { bbox, tif, out }) => {
            let tif = match (bbox, tif) {
                (Some(bbox), _) => {
                    let parts: Vec<f64> = bbox.split(',').filter_map(|v| v.trim().parse().ok()).collect();
                    if parts.len() != 4 {
                        eprintln!("--bbox needs four numbers: west,south,east,north");
                        std::process::exit(2);
                    }
                    let tif = out.with_extension("tif");
                    if let Err(e) = bathy_build::fetch((parts[0], parts[1], parts[2], parts[3]), &tif, bathy_build::NATIVE_RES) {
                        eprintln!("!! {e}");
                        std::process::exit(1);
                    }
                    tif
                }
                (None, Some(tif)) => tif,
                (None, None) => {
                    eprintln!("give --bbox or --tif");
                    std::process::exit(2);
                }
            };
            match bathy_build::build(&tif, &out) {
                Ok(_) => 0,
                Err(e) => {
                    eprintln!("!! {e}");
                    1
                }
            }
        }
        Some(Command::Sounds { lang, english_voice }) => {
            let dir = usvgs::sound::sounds_dir();
            let mut code = 0;
            for l in lang {
                let voice = if l == "en" { english_voice.clone() } else { l.clone() };
                eprintln!("[*] {l} (EdgeTX voice {voice}) → {}", dir.join(&l).display());
                if let Err(e) = usvgs::sound::fetch(&dir, &l, &voice) {
                    eprintln!("!! {e}");
                    code = 1;
                }
            }
            code
        }
        Some(Command::Say { alert, lang }) => match usvgs::sound::Alert::parse(&alert) {
            None => {
                eprintln!("unknown alert {alert:?}");
                2
            }
            Some(a) => {
                let voice = usvgs::sound::Voice::new(1.0);
                voice.say(a, &lang, usvgs::config::SoundSettings::default().rule(a.key()).tone);
                std::thread::sleep(Duration::from_secs_f64(a.rough_length_s() + 1.0));
                if voice.available() { 0 } else { 1 }
            }
        },
        Some(Command::Osm { pbf, out, tile, buildings }) => {
            if !pbf.exists() {
                eprintln!("!! pbf not found: {}", pbf.display());
                std::process::exit(1);
            }
            match osm_build::build(&pbf, &out, tile, buildings) {
                Ok(()) => 0,
                Err(e) => {
                    eprintln!("!! {e}");
                    1
                }
            }
        }
        None => {
            let settings = if cli.reset_settings { config::Settings::default() } else { config::load(None) };
            let fullscreen = if cli.fullscreen {
                Some(true)
            } else if cli.windowed {
                Some(false)
            } else {
                None
            };
            let view = cli.view.as_deref().map(|v| match v {
                "heading" => usvgs::ui::chart::ViewMode::Heading,
                "chase" => usvgs::ui::chart::ViewMode::Chase,
                _ => usvgs::ui::chart::ViewMode::North,
            });
            let power = cli.view.as_deref() == Some("power");
            let start = match (cli.sim, cli.connect) {
                (true, _) => Some(usvgs::link::Choice::Simulator),
                (_, Some(url)) => {
                    if let Err(e) = usvgs::link::mavlink::Endpoint::parse(&url) {
                        eprintln!("--connect: {e}");
                        std::process::exit(2);
                    }
                    Some(usvgs::link::Choice::Mavlink(url))
                }
                _ => None,
            };
            match usvgs::ui::run(settings, fullscreen, view, power, cli.mission.clone(), start) {
                Ok(()) => 0,
                Err(e) => {
                    eprintln!("The ground station could not open its window: {e}");
                    1
                }
            }
        }
    };
    std::process::exit(code);
}
