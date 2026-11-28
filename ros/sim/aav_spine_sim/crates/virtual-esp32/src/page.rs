//! The virtual ESP32's page, as plain request in, reply out (no sockets here): the
//! board's own status page and its routes, as `esp32_page.py` serves them, and the
//! bench's Controls, Measure and Tests (`bench.html`).
//!
//! | Path | Gives |
//! |---|---|
//! | `GET /` | the board's page (`boards/esp32-umod/page.html`), with a link to `/bench` |
//! | `GET /status.json`, `/health`, `/config` | the latest `status`, `health`, `config` line |
//! | `GET /scan` | the four devices the stand-in bus has |
//! | `GET /ws` | the status lines, as a WebSocket ([`Reply::Stream`]) |
//! | `GET /bench` | Controls, Measure, Tests |
//! | `GET /bench.json` | the bench now: the mode, the button held, the steering calibration, the test |
//! | `GET /trace.json` | the latest recording, and its step responses |
//! | `POST /press?button=<name>` | a button's heartbeat (`bench-core`'s names: `creep`, `left`, `right`, `centre`, `brake_on`, `brake_off`, `stop`, `limp`) |
//! | `POST /leave` | the page let go: out of the test mode |
//! | `POST /test/start?name=<test>`, `/test/heartbeat`, `/test/stop` | a hardware test (`steering_steps`, `brake_travel`, `throttle_spin_up`), held like a button |
//!
//! **The writes**: only from this computer (the `Host`, and the `Origin` if sent, must be
//! `127.0.0.1` or `localhost` on this port, so another web site can't post here), and
//! only on the simulator's clock ([`Page::writes`]). Otherwise `403`.

use std::fmt::Write as _;

use bench_core::controls::Button;
use bench_core::hardware::{BenchTest, Failure, Progress, Verdict};
use bench_core::step::{Response, Why};
use legacy_link::{CalState, Refusal};

use crate::{Actuator, Phase, TestRefused, VirtualEsp32};

/// The board's own page.
pub const BOARD_PAGE: &str =
    include_str!("../../../../../../firmware/spine/boards/esp32-umod/page.html");
/// The bench's page: Controls, Measure, Tests.
pub const BENCH_PAGE: &str = include_str!("../bench.html");
/// The link the board's page gets, to the bench.
const NAV: &str = "<p style=\"margin:0 0 8px;font:14px system-ui,sans-serif\">The virtual \
                   car's ESP32: <a href=\"/bench\">Controls, Measure, Tests</a></p>";

/// The HTTP methods the page uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    /// Read.
    Get,
    /// A write.
    Post,
    /// Anything else.
    Other,
}

/// One request's head.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Request {
    /// Its method.
    pub method: Method,
    /// Its path, without the query.
    pub path: String,
    /// The query's pairs, as sent (no percent decoding: the names are plain).
    pub query: Vec<(String, String)>,
    /// The headers, names in lower case.
    pub headers: Vec<(String, String)>,
}

impl Request {
    /// Reads a request's head (up to the blank line). None if it isn't one.
    #[must_use]
    pub fn parse(head: &str) -> Option<Request> {
        let mut lines = head.split("\r\n");
        let mut first = lines.next()?.split(' ');
        let method = match first.next()? {
            "GET" => Method::Get,
            "POST" => Method::Post,
            _ => Method::Other,
        };
        let target = first.next()?;
        let (path, query) = target.split_once('?').unwrap_or((target, ""));
        let query = query
            .split('&')
            .filter(|p| !p.is_empty())
            .map(|p| {
                let (k, v) = p.split_once('=').unwrap_or((p, ""));
                (k.to_string(), v.to_string())
            })
            .collect();
        let headers = lines
            .filter_map(|l| l.split_once(':'))
            .map(|(k, v)| (k.trim().to_ascii_lowercase(), v.trim().to_string()))
            .collect();
        Some(Request {
            method,
            path: path.to_string(),
            query,
            headers,
        })
    }

    /// A header's value.
    #[must_use]
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }

    /// A query value.
    #[must_use]
    pub fn arg(&self, name: &str) -> Option<&str> {
        self.query
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }
}

/// What to answer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Reply {
    /// A body: the status line's code and words, its content type, the bytes.
    Body {
        /// `200 OK`, `404 Not Found`...
        status: &'static str,
        /// Its `Content-Type`.
        kind: &'static str,
        /// The body.
        body: Vec<u8>,
    },
    /// Upgrade to the status lines' WebSocket, with this accept key.
    Stream(String),
}

impl Reply {
    fn json(body: String) -> Self {
        Reply::Body {
            status: "200 OK",
            kind: "application/json",
            body: body.into_bytes(),
        }
    }

    fn text(status: &'static str, text: &str) -> Self {
        Reply::Body {
            status,
            kind: "text/plain",
            body: format!("{text}\n").into_bytes(),
        }
    }

    /// The whole HTTP reply for a body (`Connection: close`); none for a stream.
    #[must_use]
    pub fn http(&self) -> Option<Vec<u8>> {
        let Reply::Body { status, kind, body } = self else {
            return None;
        };
        let mut out = format!(
            "HTTP/1.1 {status}\r\nContent-Type: {kind}\r\nContent-Length: {}\r\n\
             Cache-Control: no-store\r\nConnection: close\r\n\r\n",
            body.len()
        )
        .into_bytes();
        out.extend_from_slice(body);
        Some(out)
    }
}

/// The latest line of each kind the routes give, kept from the ticks' lines.
#[derive(Debug, Clone, Default)]
pub struct Latest {
    status: Option<String>,
    health: Option<String>,
    config: Option<String>,
}

impl Latest {
    /// Keep what a tick's lines hold.
    pub fn keep(&mut self, lines: &[String]) {
        for line in lines {
            let slot = if line.starts_with("{\"t\":\"status\"") {
                &mut self.status
            } else if line.starts_with("{\"t\":\"health\"") {
                &mut self.health
            } else if line.starts_with("{\"t\":\"config\"") {
                &mut self.config
            } else {
                continue;
            };
            *slot = Some(line.clone());
        }
    }
}

/// The page's settings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Page {
    /// The port it serves on (127.0.0.1 only).
    pub port: u16,
    /// Writes allowed: true only on the simulator's clock.
    pub writes: bool,
}

impl Page {
    /// The reply to `req`, at `now_ms` on the board's clock.
    pub fn route(
        &self,
        req: &Request,
        esp: &mut VirtualEsp32,
        latest: &Latest,
        now_ms: u32,
    ) -> Reply {
        match (req.method, req.path.as_str()) {
            (Method::Get, "/ws") => match req.header("sec-websocket-key") {
                Some(key)
                    if req
                        .header("upgrade")
                        .is_some_and(|u| u.eq_ignore_ascii_case("websocket")) =>
                {
                    Reply::Stream(crate::ws::accept_key(key))
                }
                _ => Reply::text("400 Bad Request", "a WebSocket only"),
            },
            (Method::Get, "/") => Reply::Body {
                status: "200 OK",
                kind: "text/html",
                body: BOARD_PAGE
                    .replacen(
                        "<div class=\"viz-root\">",
                        &format!("<div class=\"viz-root\">{NAV}"),
                        1,
                    )
                    .into_bytes(),
            },
            (Method::Get, "/bench") => Reply::Body {
                status: "200 OK",
                kind: "text/html",
                body: BENCH_PAGE.as_bytes().to_vec(),
            },
            (Method::Get, "/status.json") => {
                Reply::json(latest.status.clone().unwrap_or_else(|| "{}".into()))
            }
            (Method::Get, "/health") => {
                Reply::json(latest.health.clone().unwrap_or_else(|| "{}".into()))
            }
            (Method::Get, "/config") => {
                Reply::json(latest.config.clone().unwrap_or_else(|| "{}".into()))
            }
            (Method::Get, "/scan") => {
                Reply::json("{\"scan\":\"done\",\"found\":[4,5,12,32]}".into())
            }
            (Method::Get, "/bench.json") => Reply::json(bench_json(esp, now_ms, self.writes)),
            (Method::Get, "/trace.json") => Reply::json(trace_json(esp)),
            (Method::Post, path) => self.write(req, path, esp, now_ms),
            (Method::Get, _) => Reply::text("404 Not Found", "not found"),
            (Method::Other, _) => Reply::text("405 Method Not Allowed", "GET or POST"),
        }
    }

    fn write(&self, req: &Request, path: &str, esp: &mut VirtualEsp32, now_ms: u32) -> Reply {
        if !self.writes {
            return Reply::text("403 Forbidden", "the controls work only in the simulator");
        }
        if !self.local(req) {
            return Reply::text("403 Forbidden", "the controls work only from this computer");
        }
        match path {
            "/press" => match req.arg("button").and_then(Button::from_name) {
                Some(button) => {
                    esp.press(button, now_ms);
                    Reply::json("{\"ok\":true}".into())
                }
                None => Reply::text(
                    "400 Bad Request",
                    "button: creep, left, right, centre, brake_on, brake_off, stop or limp",
                ),
            },
            "/leave" => {
                esp.leave();
                Reply::json("{\"ok\":true}".into())
            }
            "/test/start" => {
                let test = req
                    .arg("name")
                    .and_then(|n| BenchTest::ALL.into_iter().find(|t| t.name() == n));
                match test.map(|t| esp.start_test(t, now_ms)) {
                    Some(Ok(())) => Reply::json("{\"ok\":true}".into()),
                    Some(Err(TestRefused::NotOnStands)) => {
                        Reply::json("{\"ok\":false,\"why\":\"not on stands\"}".into())
                    }
                    Some(Err(TestRefused::Busy)) => {
                        Reply::json("{\"ok\":false,\"why\":\"a test is running\"}".into())
                    }
                    None => Reply::text(
                        "400 Bad Request",
                        "name: steering_steps, brake_travel or throttle_spin_up",
                    ),
                }
            }
            "/test/heartbeat" => {
                esp.test_heartbeat(now_ms);
                Reply::json("{\"ok\":true}".into())
            }
            "/test/stop" => {
                esp.stop_test();
                Reply::json("{\"ok\":true}".into())
            }
            _ => Reply::text("404 Not Found", "not found"),
        }
    }

    /// The request comes from a page on this computer, on this port.
    fn local(&self, req: &Request) -> bool {
        let ours = |host: &str| {
            [
                format!("127.0.0.1:{}", self.port),
                format!("localhost:{}", self.port),
            ]
            .iter()
            .any(|h| h == host)
        };
        let host_ok = req.header("host").is_some_and(ours);
        let origin_ok = req
            .header("origin")
            .is_none_or(|o| o.strip_prefix("http://").is_some_and(ours));
        host_ok && origin_ok
    }
}

/// A number for JSON: `null` if not finite.
fn num(x: f32) -> String {
    if x.is_finite() {
        format!("{x}")
    } else {
        "null".into()
    }
}

fn cal_name(state: CalState) -> String {
    match state {
        CalState::Off => "off".into(),
        CalState::Limp => "limp".into(),
        CalState::Jog => "jog".into(),
        CalState::Hold => "hold".into(),
        CalState::Return => "return".into(),
        CalState::Refused(why) => format!(
            "refused: {}",
            match why {
                Refusal::Moving => "moving",
                Refusal::NotBraked => "not braked",
                Refusal::NoFeedback => "no feedback",
                Refusal::Estop => "e-stop",
                Refusal::OtherDriver => "the RC or a test drives",
            }
        ),
    }
}

fn verdict_json(v: Verdict) -> String {
    let why = |f: Failure| match f {
        Failure::Timing(Why::NeverMoved) => "never moved".to_string(),
        Failure::Timing(Why::NeverArrived) => "never got 90 % of the way".to_string(),
        Failure::Timing(Why::AlreadyThere) => "already there".to_string(),
        Failure::Off(off) => format!("ended {off:.3} off"),
        Failure::NeverTurned(top) => format!("the wheels never turned (top {top:.3} m/s)"),
        Failure::NeverStopped(end) => format!("the wheels never stopped ({end:.3} m/s)"),
    };
    match v {
        Verdict::Pass => "{\"result\":\"pass\"}".into(),
        Verdict::Fail { step, why: w } => {
            format!(
                "{{\"result\":\"fail\",\"step\":{step},\"why\":\"{}\"}}",
                why(w)
            )
        }
        Verdict::NotMeasured(w) => format!("{{\"result\":\"not measured\",\"why\":\"{w}\"}}"),
    }
}

/// The bench now, for the page.
#[must_use]
pub fn bench_json(esp: &VirtualEsp32, now_ms: u32, writes: bool) -> String {
    let mut s = String::from("{");
    let _ = write!(s, "\"now_ms\":{now_ms},\"writes\":{writes},");
    let _ = write!(
        s,
        "\"held\":{},",
        esp.held(now_ms)
            .map_or("null".into(), |b| format!("\"{}\"", b.name()))
    );
    let _ = write!(
        s,
        "\"refused\":{},",
        if esp.refused().is_some() {
            "\"not on stands\""
        } else {
            "null"
        }
    );
    let _ = write!(s, "\"on_stands\":{},", esp.measured.on_stands);
    if let Some((out, drive, cal)) = esp.last() {
        let _ = write!(
            s,
            "\"mode\":\"{}\",\"armed\":{},\"faults\":{},\"steering_target\":{},\"throttle\":{},\"brake\":{},\
             \"drive\":{{\"throttle\":{},\"brake\":{},\"reverse\":{},\"steering\":{},\"limp\":{}}},\"calibration\":\"{}\",",
            crate::report::mode(out.mode).name(),
            out.armed,
            out.faults,
            num(out.steering_target),
            num(out.throttle),
            num(out.brake),
            num(drive.throttle),
            num(drive.brake),
            drive.reverse,
            num(drive.steering),
            drive.limp,
            cal_name(cal)
        );
    }
    let m = esp.measured;
    let _ = write!(
        s,
        "\"measured\":{{\"speed\":{},\"steering\":{},\"brake\":{}}},",
        num(m.speed),
        num(m.steering),
        num(m.brake)
    );
    let running = esp.running().map_or("null".into(), |(t, p)| {
        let phase = match p {
            Phase::Entering | Phase::Starting => "entering",
            Phase::Running => "running",
            Phase::Leaving => "leaving",
        };
        format!("{{\"name\":\"{}\",\"phase\":\"{phase}\"}}", t.name())
    });
    let _ = write!(s, "\"test\":{running},");
    let outcome = esp.outcome().map_or("null".into(), |o| {
        let progress = match o.progress {
            Progress::Running => "running",
            Progress::Finished => "finished",
            Progress::Stopped => "stopped",
        };
        format!(
            "{{\"name\":\"{}\",\"progress\":\"{progress}\",\"verdict\":{}}}",
            o.test.name(),
            verdict_json(o.verdict)
        )
    });
    let _ = write!(s, "\"outcome\":{outcome}}}");
    s
}

fn responses_json(rs: &[Response]) -> String {
    let ms = |r: Result<u32, Why>| r.map_or("null".into(), |v| v.to_string());
    let items: Vec<String> = rs
        .iter()
        .map(|r| {
            format!(
                "{{\"at_ms\":{},\"from\":{},\"to\":{},\"before\":{},\"delay_ms\":{},\"rise_ms\":{},\"end\":{},\"off\":{}}}",
                r.at_ms,
                num(r.step.asked_from),
                num(r.step.asked_to),
                num(r.before),
                ms(r.delay_ms),
                ms(r.rise_ms),
                num(r.end),
                num(r.off)
            )
        })
        .collect();
    format!("[{}]", items.join(","))
}

/// The latest recording: its samples (columns), and each actuator's step responses.
#[must_use]
pub fn trace_json(esp: &VirtualEsp32) -> String {
    let samples = esp.trace();
    let col = |f: fn(&bench_core::trace::Sample) -> String| {
        samples.iter().map(f).collect::<Vec<_>>().join(",")
    };
    format!(
        "{{\"test\":{},\"t_ms\":[{}],\"steer_asked\":[{}],\"steer\":[{}],\"brake_asked\":[{}],\"brake\":[{}],\
         \"throttle\":[{}],\"speed\":[{}],\"steering_steps\":{},\"brake_steps\":{}}}",
        esp.outcome()
            .map_or("null".into(), |o| format!("\"{}\"", o.test.name())),
        col(|s| s.at_ms.to_string()),
        col(|s| num(s.steer_asked)),
        col(|s| num(s.steer)),
        col(|s| num(s.brake_asked)),
        col(|s| num(s.brake)),
        col(|s| num(s.throttle)),
        col(|s| num(s.speed)),
        responses_json(&esp.step_responses(Actuator::Steering)),
        responses_json(&esp.step_responses(Actuator::Brake)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Settings;

    fn req(head: &str) -> Request {
        Request::parse(head).expect("a request")
    }

    fn body(r: &Reply) -> String {
        match r {
            Reply::Body { body, .. } => String::from_utf8_lossy(body).into_owned(),
            Reply::Stream(_) => String::new(),
        }
    }

    fn status(r: &Reply) -> &'static str {
        match r {
            Reply::Body { status, .. } => status,
            Reply::Stream(_) => "101",
        }
    }

    const PAGE: Page = Page {
        port: 8766,
        writes: true,
    };

    #[test]
    fn a_request_head_parses() {
        let r = req("POST /press?button=creep HTTP/1.1\r\nHost: 127.0.0.1:8766\r\nOrigin: http://127.0.0.1:8766\r\n");
        assert_eq!(r.method, Method::Post);
        assert_eq!(r.path, "/press");
        assert_eq!(r.arg("button"), Some("creep"));
        assert_eq!(r.header("origin"), Some("http://127.0.0.1:8766"));
    }

    #[test]
    fn the_board_page_and_the_stream() {
        let mut esp = VirtualEsp32::new(Settings::default(), 1);
        let latest = Latest::default();
        let page = PAGE.route(&req("GET / HTTP/1.1\r\nHost: x\r\n"), &mut esp, &latest, 0);
        assert!(body(&page).contains("/bench"));
        assert!(body(&page).contains("new WebSocket"));
        let ws = PAGE.route(
            &req("GET /ws HTTP/1.1\r\nUpgrade: websocket\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"),
            &mut esp,
            &latest,
            0,
        );
        assert_eq!(ws, Reply::Stream("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=".into()));
    }

    #[test]
    fn the_latest_lines_are_served() {
        let mut esp = VirtualEsp32::new(Settings::default(), 1);
        let mut latest = Latest::default();
        for t in 0..60 {
            let out = esp.tick(t * 10, crate::Measured::default());
            latest.keep(&out.lines);
        }
        let r = PAGE.route(&req("GET /health HTTP/1.1\r\n"), &mut esp, &latest, 600);
        assert!(body(&r).starts_with("{\"t\":\"health\""), "{}", body(&r));
        let r = PAGE.route(
            &req("GET /status.json HTTP/1.1\r\n"),
            &mut esp,
            &latest,
            600,
        );
        assert!(body(&r).starts_with("{\"t\":\"status\""));
    }

    #[test]
    fn a_press_from_this_computer_reaches_the_bench() {
        let mut esp = VirtualEsp32::new(Settings::default(), 1);
        let latest = Latest::default();
        let r = PAGE.route(
            &req("POST /press?button=left HTTP/1.1\r\nHost: localhost:8766\r\n"),
            &mut esp,
            &latest,
            100,
        );
        assert_eq!(status(&r), "200 OK");
        assert_eq!(esp.held(100), Some(Button::Left));
        let json = bench_json(&esp, 100, true);
        assert!(json.contains("\"held\":\"left\""), "{json}");
    }

    #[test]
    fn writes_from_elsewhere_or_off_the_simulator_are_refused() {
        let mut esp = VirtualEsp32::new(Settings::default(), 1);
        let latest = Latest::default();
        let other_site = req("POST /press?button=creep HTTP/1.1\r\nHost: 127.0.0.1:8766\r\nOrigin: http://example.com\r\n");
        assert_eq!(
            status(&PAGE.route(&other_site, &mut esp, &latest, 0)),
            "403 Forbidden"
        );
        let other_host = req("POST /press?button=creep HTTP/1.1\r\nHost: 10.0.0.5:8766\r\n");
        assert_eq!(
            status(&PAGE.route(&other_host, &mut esp, &latest, 0)),
            "403 Forbidden"
        );
        let real_clock = Page {
            writes: false,
            ..PAGE
        };
        let ours = req("POST /press?button=creep HTTP/1.1\r\nHost: 127.0.0.1:8766\r\n");
        assert_eq!(
            status(&real_clock.route(&ours, &mut esp, &latest, 0)),
            "403 Forbidden"
        );
        assert_eq!(esp.held(0), None);
    }

    #[test]
    fn a_test_starts_and_its_trace_reads_back() {
        let mut esp = VirtualEsp32::new(Settings::default(), 1);
        let latest = Latest::default();
        let start = req("POST /test/start?name=brake_travel HTTP/1.1\r\nHost: 127.0.0.1:8766\r\n");
        assert!(body(&PAGE.route(&start, &mut esp, &latest, 0)).contains("true"));
        let spin =
            req("POST /test/start?name=throttle_spin_up HTTP/1.1\r\nHost: 127.0.0.1:8766\r\n");
        assert!(body(&PAGE.route(&spin, &mut esp, &latest, 0)).contains("running"));
        for t in 0..30 {
            esp.test_heartbeat(t * 10);
            let _ = esp.tick(t * 10, crate::Measured::default());
        }
        let json = trace_json(&esp);
        assert!(json.starts_with("{\"test\":"), "{json}");
        assert!(json.contains("\"brake_asked\":["));
    }
}
