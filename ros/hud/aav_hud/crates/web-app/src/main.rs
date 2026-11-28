//! `aav_web`: the Orin's web page, **read only** (docs/designs/2026-11-27_web-screens.md).
//!
//! One page, three tabs, for any browser on the car's network:
//!
//! | Tab | Shows |
//! |---|---|
//! | Car | each sensor and actuator: OK, STALE or NEVER, its rate, its numbers |
//! | Hops | what ROS asked (ego, the mux), what CORTEX wrote, what the ESP32 got: 60 s graphs, each hop's verdict, the counts that prove the wire |
//! | ROS | every topic against `docs/interfaces.yaml` for a profile: LIVE, MISSING, WRONG TYPE, QUIET, NOT IN CONTRACT |
//!
//! Node `web` (topic names as `hw_panel`'s, from `--config` or its parameters):
//!
//! | Direction | What |
//! |---|---|
//! | sub | `hw_panel`'s subscriptions (aav_hud README): `/vehicle/status`, the actuators, the sensors, `/system/source`, `/safety/estop`, `/control/cmd_vel`, `/ego/actuator_cmd`, `/control/car/actuator_cmd`, `/control/car/link_frame`, `/vehicle/esp32/uplink_health`... |
//! | reads | the graph once a second: topics, types, publishers |
//! | reads | `--esp32 ws://HOST:PORT/ws`: the ESP32's status lines, straight from the board |
//! | pub | **nothing** (only rclrs's own `/rosout` and `/parameter_events`) |
//! | serves | HTTP on `--bind`:`--port` (127.0.0.1:8780): `/` the page, `/api/snapshot` the JSON. GET only |
//!
//! The logic is in `contract` and `snapshot` (no ROS); the news is panel-feed's.

// Without ROS only the tests use the snapshot: the program itself needs the node.
#![cfg_attr(not(feature = "ros"), allow(dead_code))]

mod contract;
mod snapshot;

use std::{
    net::SocketAddr,
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, RwLock,
    },
};

use anyhow::{bail, Context as _, Result};

/// The page: plain HTML and JS, no library, nothing from the internet (the car is offline).
const PAGE: &str = include_str!("../web/index.html");
/// The contract built in: `--contract` reads another.
const CONTRACT: &str = include_str!("../../../../../../docs/interfaces.yaml");

/// The command line.
#[derive(Debug, Clone)]
struct Options {
    bind: String,
    port: u16,
    profile: String,
    contract: Option<PathBuf>,
    config: Option<PathBuf>,
    esp32: Option<String>,
    /// `--expected PROFILE`: print the topics the profile needs live, and stop.
    expected: bool,
}

impl Options {
    fn parse(args: &[String]) -> Result<Self> {
        let mut o = Self {
            bind: "127.0.0.1".into(),
            port: 8780,
            profile: "car".into(),
            contract: None,
            config: None,
            esp32: None,
            expected: false,
        };
        let mut it = args.iter().skip(1);
        while let Some(a) = it.next() {
            if a == "--ros-args" {
                break;
            }
            let mut value = || it.next().with_context(|| format!("{a} needs a value"));
            match a.as_str() {
                "--bind" => o.bind = value()?.clone(),
                "--port" => o.port = value()?.parse().context("--port")?,
                "--profile" => o.profile = value()?.clone(),
                "--contract" => o.contract = Some(value()?.into()),
                "--config" => o.config = Some(value()?.into()),
                "--expected" => {
                    o.profile = value()?.clone();
                    o.expected = true;
                }
                "--esp32" => o.esp32 = Some(value()?.clone()).filter(|u| !u.is_empty()),
                "-h" | "--help" => {
                    println!("{}", HELP);
                    std::process::exit(0);
                }
                other => bail!("unknown option {other}\n{HELP}"),
            }
        }
        if !contract::PROFILES.contains(&o.profile.as_str()) {
            bail!("--profile {}: one of {:?}", o.profile, contract::PROFILES);
        }
        Ok(o)
    }
}

const HELP: &str = "aav_web [--bind 127.0.0.1] [--port 8780] [--profile car|virtual|sim|...] \
[--contract docs/interfaces.yaml] [--config hw_panel.yaml] [--esp32 ws://HOST:PORT/ws] [--ros-args ...]
Read only: serves the page and /api/snapshot; publishes nothing.
aav_web --expected PROFILE: the topics the profile needs live, one a line (as check_graph.py).";

/// The latest snapshot, as JSON, for every request.
type Latest = Arc<RwLock<Arc<String>>>;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let opts = Options::parse(&args)?;
    let contract = match &opts.contract {
        Some(path) => contract::Contract::from_yaml(
            &std::fs::read_to_string(path).with_context(|| format!("{}", path.display()))?,
        )?,
        None => contract::Contract::from_yaml(CONTRACT)?,
    };
    if opts.expected {
        contract
            .must_be_live(&opts.profile)
            .for_each(|t| println!("{t}"));
        return Ok(());
    }
    let latest: Latest = Arc::new(RwLock::new(Arc::new("{}".to_owned())));
    let stop = Arc::new(AtomicBool::new(false));
    let feed = feed::start(
        args,
        &opts,
        contract,
        Arc::clone(&latest),
        Arc::clone(&stop),
    )?;
    let addr: SocketAddr = format!("{}:{}", opts.bind, opts.port)
        .parse()
        .context("--bind and --port")?;
    let served = serve(addr, latest, Arc::clone(&stop));
    stop.store(true, Ordering::SeqCst);
    let _ = feed.join();
    served
}

/// The web server, until `stop`.
fn serve(addr: SocketAddr, latest: Latest, stop: Arc<AtomicBool>) -> Result<()> {
    use axum::{
        http::{header, StatusCode},
        response::{Html, IntoResponse},
        routing::get,
        Router,
    };
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    runtime.block_on(async move {
        // GET only: there is no route that takes anything.
        let app = Router::new()
            .route("/", get(|| async { Html(PAGE) }))
            .route(
                "/api/snapshot",
                get(move || {
                    let latest = Arc::clone(&latest);
                    async move {
                        let json = latest
                            .read()
                            .map(|j| Arc::clone(&j))
                            .unwrap_or_else(|p| Arc::clone(&p.into_inner()));
                        (
                            [
                                (header::CONTENT_TYPE, "application/json"),
                                (header::CACHE_CONTROL, "no-store"),
                            ],
                            json.as_str().to_owned(),
                        )
                    }
                }),
            )
            .fallback(|| async { (StatusCode::NOT_FOUND, "not here").into_response() });
        let listener = tokio::net::TcpListener::bind(addr)
            .await
            .with_context(|| format!("listening on {addr}"))?;
        println!("aav_web: http://{addr}/ (read only)");
        axum::serve(listener, app)
            .with_graceful_shutdown(async move {
                while !stop.load(Ordering::SeqCst) {
                    tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                }
            })
            .await
            .context("serving")
    })
}

#[cfg(feature = "ros")]
mod feed {
    //! The news, on a thread of its own: ROS (listen-only) and the ESP32's lines into the
    //! panel's state every 5 ms, then a snapshot five times a second.

    use std::{
        sync::{
            atomic::{AtomicBool, Ordering},
            Arc,
        },
        thread::{self, JoinHandle},
        time::{Duration, Instant},
    };

    use anyhow::{Context as _, Result};
    use panel_core::{config::PanelConfig, state::PanelState};
    use panel_feed::{esp32::Esp32Reader, ros::RosSource, source::DataSource as _};

    use crate::{
        contract::{Contract, Seen},
        snapshot::{self, Ros},
        Latest, Options,
    };

    /// How often the news is taken: an update's time is when it was taken, so this is the
    /// delays' resolution.
    const DRAIN: Duration = Duration::from_millis(5);
    /// How often the page's snapshot is made.
    const EVERY: Duration = Duration::from_millis(200);

    pub fn start(
        args: Vec<String>,
        opts: &Options,
        contract: Contract,
        latest: Latest,
        stop: Arc<AtomicBool>,
    ) -> Result<JoinHandle<()>> {
        let mut config = match &opts.config {
            Some(path) => PanelConfig::from_yaml(
                &std::fs::read_to_string(path).with_context(|| format!("{}", path.display()))?,
            )?,
            None => PanelConfig::default(),
        };
        if let Some(url) = &opts.esp32 {
            config.esp32_url = url.clone();
        }
        let (mut ros, config) = RosSource::listen(args, config, "web")?;
        let mut esp32 = match config.esp32().context("esp32_url")? {
            Some(url) => {
                println!("aav_web: reading the ESP32's status lines from {url} (read only)");
                Some(Esp32Reader::start(url)?)
            }
            None => None,
        };
        let profile = opts.profile.clone();
        let mut state = PanelState::new(config.stale_after());
        state
            .graphs
            .set_frame_steering_sign(config.frame_steering_sign);
        Ok(thread::Builder::new().name("feed".into()).spawn(move || {
            let mut next_snapshot = Instant::now();
            while !stop.load(Ordering::SeqCst) && !ros.has_stopped() {
                let now = Instant::now();
                for update in ros.drain(now) {
                    state.apply(update, now);
                }
                if let Some(e) = esp32.as_mut() {
                    for update in e.drain() {
                        state.apply(update, now);
                    }
                }
                if now >= next_snapshot {
                    next_snapshot = now + EVERY;
                    let seen: Option<Vec<Seen>> = ros.topics().map(|t| {
                        t.into_iter()
                            .map(|t| Seen {
                                name: t.name,
                                types: t.types,
                                publishers: t.publishers,
                            })
                            .collect()
                    });
                    let ros_tab = Ros::judge(&contract, &profile, seen.as_deref());
                    let snap = snapshot::build(&state, ros_tab, now);
                    if let Ok(json) = serde_json::to_string(&snap) {
                        if let Ok(mut l) = latest.write() {
                            *l = Arc::new(json);
                        }
                    }
                }
                thread::sleep(DRAIN);
            }
            // Ctrl-C (the node's handler) or the node failed: the server stops too.
            stop.store(true, Ordering::SeqCst);
        })?)
    }
}

#[cfg(not(feature = "ros"))]
mod feed {
    use std::{
        sync::{atomic::AtomicBool, Arc},
        thread::JoinHandle,
    };

    use anyhow::{bail, Result};

    use crate::{contract::Contract, Latest, Options};

    pub fn start(
        _args: Vec<String>,
        _opts: &Options,
        _contract: Contract,
        _latest: Latest,
        _stop: Arc<AtomicBool>,
    ) -> Result<JoinHandle<()>> {
        bail!("aav_web was built without ROS (--no-default-features): only its tests run")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(a: &[&str]) -> Vec<String> {
        std::iter::once("aav_web")
            .chain(a.iter().copied())
            .map(str::to_owned)
            .collect()
    }

    #[test]
    fn the_defaults_listen_on_this_computer_only() {
        let o = Options::parse(&args(&[])).expect("ok");
        assert_eq!(
            (o.bind.as_str(), o.port, o.profile.as_str()),
            ("127.0.0.1", 8780, "car")
        );
        let o = Options::parse(&args(&[
            "--profile",
            "virtual",
            "--esp32",
            "",
            "--ros-args",
            "-r",
            "x",
        ]))
        .expect("ok");
        assert_eq!(o.profile, "virtual");
        assert!(o.esp32.is_none());
        assert!(Options::parse(&args(&["--profile", "moon"])).is_err());
        assert!(Options::parse(&args(&["--port"])).is_err());
    }

    #[test]
    fn the_page_asks_only_for_the_snapshot() {
        // Read only: the page never sends anything but GETs of its own two paths.
        assert!(PAGE.contains("/api/snapshot"));
        for verb in ["POST", "DELETE", "method:", "WebSocket", "XMLHttpRequest"] {
            assert!(!PAGE.contains(verb), "the page has {verb}");
        }
        assert!(
            !PAGE.contains("http://") && !PAGE.contains("https://"),
            "the page loads from the internet"
        );
    }

    #[test]
    fn the_built_in_contract_reads() {
        assert!(
            contract::Contract::from_yaml(CONTRACT)
                .expect("reads")
                .len()
                > 50
        );
    }
}
