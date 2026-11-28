# Web screens: the ESP32's page and the Orin's page

Ben, 27 Nov: "on the esp32 we should have the very simple vehicle controls and the go slack
and measurement stuff and then the rest should be a stream of data to something else. Then
on the orin we will have the website that contains the graph of what is being sent to the
cortex and the esp32."

| | |
|---|---|
| State | **Steps 1 and 2 built** (27 Nov, on the virtual car): `aav_web`, aav_hud's `web-app` ([README](../../ros/hud/aav_hud/README.md#node-web-aav_web-the-orins-web-page-read-only)). Steps 3-6 not started |
| Replaces | nothing yet: the raylib hardware page and test bench **stay** beside it (Ben: "Maybe keep both for now") |
| Not changed | the in-car HUD stays raylib on the Pi (CLAUDE.md, Screens) |
| Open | W1-W5 at the bottom |

## 1. The picture

```text
                       ┌──────────────── browser (laptop, car screen) ───────────────┐
                       │   ESP32 page: controls, LIMP,      Orin page: graphs,       │
                       │   measurements, hardware tests     car status, ROS live     │
                       └──────────▲──────────────────────────────▲───────────────────┘
                                  │ HTTP + WebSocket             │ HTTP + WebSocket
   ROS ─► ego ─► /ego/actuator_cmd ─► CORTEX bridge ─UART─► ESP32 │
                 │                         │                 │   │
                 └────── the Orin page reads all three ──────┴───┘
                    what ROS asked · what CORTEX sent · what the ESP32 did
```

| Computer | Its page does | Its page never does |
|---|---|---|
| **ESP32** | simple controls, LIMP, measurements, the hardware tests; streams its lines | plan, drive a route, keep history |
| **Orin** | graphs that compare the three hops; the car's status; ROS live or not | send a command (read only) |

Both are HTML5 and JS (plain files, uPlot for graphs), so the same page runs in any browser.

## 2. The ESP32's page

Today's `wifi-status` page (read only) gets three new tabs.

| Tab | What | Writes? |
|---|---|---|
| **Status** | today's page: steering, brake, throttle graphs; health; I2C scan | no |
| **Controls** | creep forward, creep back, turn left, turn right, centre, STOP, **LIMP** (steering motor off, the counts still read) | **yes** |
| **Measure** | the board records a short trace in RAM (10 ms a sample), then serves it: steering step response, brake travel time, frame-to-pin delay, tick gaps, free-wheel spin-up (TODO 285) | starts a recording |
| **Tests** | the hardware side of the ladder: each test is a fixed sequence of moves + a recording + a pass/fail rule | yes |

### The rules for the writes

| Rule | Why |
|---|---|
| A separate feature, `wifi-control`, **off by default**, never in the car-day build | as `wifi-update` today |
| Hold to move: the browser sends a heartbeat every 50 ms; none for 100 ms: STOP | the same 100 ms rule as the Pi link |
| Only when **stopped and on stands** (a mode the RC switch selects, or no Pi link) | a page can't start the car mid-drive |
| Speeds capped low (creep 0.3 m/s); steering clamped 400-3500 counts at 300 counts/s | the calibration page's limits, reused |
| RC and the e-stop always win | the spine's own order, unchanged |
| The deciding is in a `no_std` crate, `bench-core` (tested on a laptop); the board only moves pins | "keep the maths out" |
| **On the real car: the hardware lead decides** whether to flash `wifi-control` | it is their board |

### The stream

Everything else goes out as a stream, as today: the JSON status lines over `/ws`. The ESP32
keeps no history beyond a measurement's buffer (a few seconds; it has 520 KB of RAM).

## 3. The Orin's page

A Rust program (axum), read only. It reads three things and lines them up in time:

| Hop | From |
|---|---|
| What ROS asked | `/ego/actuator_cmd` |
| What CORTEX sent | the bridge's frames (its counters, and the uplink it gets back) |
| What the ESP32 did | the ESP32's `/ws` stream, **straight from the board** over WiFi |

| Tab | Shows |
|---|---|
| **Hops** | the three graphs on one time axis: steering, throttle, brake; the delay between hops; dropped frames |
| **Car** | which sensors are live, the localizer's health, the source (REAL / SIM / BENCH), the e-stop, the gate |
| **ROS** | each topic in `docs/interfaces.yaml`: expected rate, the rate it has now, last seen. Red when it falls under |
| **Tests** | the software rungs of the ladder (later: W4) |

"Monitor ROS in a meaningful way": compare what runs against the contract, as `pixi run
check` already does. A topic that's missing or slow means something; a raw list of topics
doesn't.

## 4. The virtual car: the same pages

Everything is a drop-in, so the virtual car serves the same two pages.

| Real car | Virtual car |
|---|---|
| ESP32 page on the board | the fake ESP32 (`esp32_page.py`, `127.0.0.1:8766`) serves the same HTML, with Controls, Measure and Tests **on** |
| Orin page on the Orin | the same Rust program on the Mac ("the fake Orin"), reading the fake ESP32 and Ravensim's ROS |

On the virtual car, the full hardware test suite can run from the ESP32's page: speed up,
turn left, turn right, LIMP, each test's trace. Ravensim's car moves.

So the tests behave the same in both, the fake ESP32 runs **`bench-core` itself** (the same
Rust), not a Python copy (W2).

## 5. Steps

| # | Step | Done when |
|---|---|---|
| 1 | The Orin page, read only: **Car** and **ROS** tabs, on the virtual car | on the Mac, a stopped sensor turns red within 2 s. ✅ 27 Nov: built; ROS 65 of 65 live, 0 problems (the 2 s test not run) |
| 2 | The **Hops** tab: the ESP32's stream + `/ego/actuator_cmd` + CORTEX's frames | a turn in Ravensim shows on all three graphs, with the delay. 🟡 27 Nov: the graphs and verdicts built (panel-core's); the delay between hops not yet shown; the mux hop NO DATA in Ravensim (TODO 286a) |
| 3 | `bench-core`: the measurements (the trace buffer, step response) and the tests' sequences; laptop tests | `cargo test` |
| 4 | The fake ESP32 runs `bench-core`; its page gets Controls, Measure, Tests | from the page: creep, turn, LIMP, a step response, in Ravensim |
| 5 | The real ESP32: `wifi-control`, bench first, no car | the hardware lead says yes (W1) |
| 6 | The Orin page's **Tests** tab (the software rungs) | W4 |

The raylib hardware page and test bench stay until the HTML ones have been used for a while (W3).

## 6. Open questions

| # | Question | My pick |
|---|---|---|
| W1 | Writes on the real ESP32's page (Controls, Tests): flash `wifi-control` on the car? | the hardware lead's call; bench only until they say |
| W2 | The fake ESP32 runs `bench-core`: a small Rust program beside Ravensim, or Python calling it (PyO3)? | a small Rust program (Rust first) |
| W3 | When do the raylib hardware page and test bench go? | after a car day on the HTML ones |
| W4 | The ladder's software rungs: on the Orin page, or a laptop page? | the Orin page, run only on SIM (the ladder already never moves the real car) |
| W5 | Does the Orin page reach the ESP32 over WiFi on the car (the ESP32 joins the car's WiFi)? | yes; if WiFi delays the tick, CORTEX's uplink only |
