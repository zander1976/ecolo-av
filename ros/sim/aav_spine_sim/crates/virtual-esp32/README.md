# virtual-esp32

## Overview

The virtual car's ESP32 ([design](../../../../../docs/designs/2026-11-27_web-screens.md),
step 4): the **Rust firmware's own logic** on a laptop, so the ESP32's page gets Controls,
Measure and Tests from `bench-core`, and Ravensim's car moves. No ROS: the node around it
is `spine-sim`'s `virtual_esp32`.

```text
 feed(bytes) ─► LegacyLink ─┐                 ┌─► Drive: throttle, brake, reverse, steering
 press(button) ─► Controls ─┼─► SpineBoard ───┤   (what the stand-in devices were told)
 start_test ─► Run ─────────┘    (spine-core) ├─► the uplink's bytes, the status lines
                                              └─► Recorder ─► judge, step responses
```

| Part | From |
|---|---|
| the board: clock, no e-stop, a wheel counter turned by the measured speed | `parts::LaptopBoard` (here) |
| the devices: a healthy Jrk, DAC and expander | `spine-board`'s `StandInBus` |
| the bridge's serial line, both ways | `parts::ByteQueue`, `ByteSink` (here), into `LegacyLink` |
| the spine, the test mode, LIMP's rules | `spine-board`'s `SpineBoard`, `spine-core`, `legacy-link`'s `SteerCal` |
| the page's buttons, tests, recording, verdicts | `bench-core` |
| the status lines | `status-lines`; their cadence ported from the board program's `report.rs` |

## Interface

| Item | Does |
|---|---|
| `VirtualEsp32::new(Settings, boot_id)` | boots it: the devices checked, `init()` |
| `Settings { frame_version, wheel_speed }` | `FrameVersion::V1` (today's car) or `V2` (2, 3, 4); the spine measures the speed (`true`, default) or not, as today's board |
| `feed(bytes)` | the bridge's bytes, any split |
| `press(button, now)`, `leave()` | the page's heartbeat for a held button (`bench-core`'s `Button`); the page let go |
| `start_test(test, now)`, `test_heartbeat(now)`, `stop_test()` | a hardware test (`BenchTest`), held like a button; `Err(TestRefused)` off stands or busy |
| `tick(now, Measured) -> TickOut` | one 10 ms tick: `outputs` (the spine's), `drive` (the devices'), `uplink` bytes, `lines` (JSON status lines) |
| `Measured { speed, steering, brake, on_stands }` | what the simulator measured |
| `held`, `refused`, `running`, `outcome`, `last`, `trace`, `step_responses(Actuator)`, `max_steering` | for the page |

## Invariants

- **Nothing here decides what is safe.** Each button is the spine's own request
  (`SpineBoard::request_test`, `LegacyLink::request_calibration`); the spine and the link
  refuse what they refuse. The page's two rules are `bench-core`'s: hold to move (100 ms),
  the throttle only on stands.
- The firmware crates are used unchanged, from `firmware/spine/crates/`.
- Differs from today's ESP32 on purpose: the spine measures the speed (the test mode's
  throttle needs one), no RC receiver is fitted, and the devices always answer.
- While limp, `drive.steering` is where the wheels are (the motor is off).

## Side Effects

None: it decides and formats. The node moves Ravensim's car.

## Verification

```sh
cd ros/sim/aav_spine_sim && cargo test -p virtual-esp32
```

`colcon test --packages-select aav_spine_sim` runs it too (CMake test `virtual_esp32`).
