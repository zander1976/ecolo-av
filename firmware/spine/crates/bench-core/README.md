# bench-core

## Overview

The ESP32 page's bench ([design](../../../../docs/designs/2026-11-27_web-screens.md), step 3):
what its **Controls**, **Measure** and **Tests** tabs do, as plain `no_std` Rust with no
hardware and no web server. Tested on a laptop.

```text
 browser: button held (50 ms) ──► Controls ──► spine-core TestRequest  (the spine's test mode)
                                          └──► legacy-link CalRequest  (LIMP: SteerCal's rules)
 each tick: Sample ──► Recorder<N> ──► judge(test, samples) ──► Pass / Fail / NotMeasured
                                   └──► step::respond: delay, 10-90 % rise, end
```

**Nothing here decides what is safe.** Every button becomes one of the spine's own
requests; the spine and the link refuse what they refuse, RC and the e-stop win. The
page adds only two rules of its own: **hold to move** (100 ms without a heartbeat: `Hold`)
and **the throttle only on stands**.

## Interface

| Item | Does |
|---|---|
| `controls::Controls` | `press(button, now, max_steering)` each heartbeat; `tick(now, Where)` → `Command { test, cal }`; `leave()` sends one `Exit` |
| `controls::Button` | `Creep` (on stands only), `Left`, `Right` (0.05 rad a press), `Centre`, `BrakeOn`, `BrakeOff`, `Stop`, `Limp`; `name()` / `from_name()` as the browser sends them |
| `controls::Where` | `OnStands` (a person said so) or `OnTheGround` |
| `trace::Recorder<N>` | `start(now, for_ms)`, `push(now, Sample)` each tick, `samples()`; full or time up: `Done` |
| `trace::Sample` | steering asked and measured (rad), brake asked and measured (0..1), throttle, speed |
| `step::steps`, `step::respond` | `tools/step_response.py`'s rules, in ms: the steps, and each one's delay, rise and end (`Err(Why)` where the samples can't say) |
| `hardware::BenchTest` | `SteeringSteps`, `BrakeTravel`, `ThrottleSpinUp` (on stands): their moves and lengths |
| `hardware::Run` | `start(test, now, Where)`, `heartbeat(now)` while RUN is held, `tick(now)` → the move's `TestRequest`, `Hold` once done or let go |
| `hardware::judge` | the verdict from the recording: `Pass`, `Fail { step, why }`, `NotMeasured(why)` |

| Test | Moves | Passes when |
|---|---|---|
| steering steps | centre, +0.2, -0.2, centre rad (1.5 s each) | each step 90 % of the way, ending within 0.03 rad |
| brake travel | off, on, off (1.5 s each) | each step 90 % of the way, ending within 0.1 |
| throttle spin-up | creep 0.08 for 3 s, then hold 2 s; on stands | the wheels pass 0.1 m/s and end under 0.05 m/s |

## Invariants

- Only the spine's own requests go out: `TestRequest` (throttle at most
  `TEST_MAX_THROTTLE`, forward only) and `CalRequest::Limp`. No target of its own.
- A button or a test run stops (`Hold`) 100 ms after its last heartbeat.
- `Creep` and the throttle test need `Where::OnStands`; otherwise `Hold` and `Refused::NotOnStands`.
- No allocation. `judge` copies up to 640 samples onto the stack (about 7.5 KB).

## Side Effects

None: it decides what to ask. A board program (`wifi-control`, off by default, the
hardware lead's call: W1) or the virtual car's ESP32 carries the requests out.

## Verification

```bash
cd firmware/spine && cargo test -p bench-core
```

Not yet used by any board program or by the virtual car (step 4).
