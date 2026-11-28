# TODO

The current work list, grouped by topic. One line a row; the details live in the linked
pages. Tick items off as they land; each tick = a commit.

- Ben's messages, word for word: `docs/history/messages-*.md`.
- What is finished: [todo-done](docs/history/todo-done.md). The whole list as it was before
  this reorganisation (19 Nov), word for word:
  [there](docs/history/todo-done.md#the-whole-list-as-it-was-on-19-nov-before-the-reorganisation).

Status: ⬜ not started · 🔧 started · 🟡 built, a challenger or still being scored ·
👀 watching · ✅ done (then moved to todo-done).

## What is next

Ben, 19 Nov: "porting all the code and writing all the features left and then running the
tests too. liek th epixi test that is. Then we can start doing like algo testing like
diffused and stuff. start getting an idea of how the platform will work with everything
done". So: **build first** (each batch with `tools/test.sh` and `pixi run test`, no sim
scoring), **then score** the algorithms together. One building helper at a time; tests in
the background; each merged with a real merge (Ben's history stays).

### Stage 1: build everything left

| Batch | What | Rows |
|---|---|---|
| 1 | ✅ ONNX with Core ML (merged 19 Nov) | 243 |
| 2 | ✅ Lanes into the costmap (merged 19 Nov) | 264 |
| 3 | ✅ Before the car (merged 19 Nov) | 261, 258, 255 |
| 4 | ✅ Maps offline (merged 19 Nov) | 259, 260, 250 |
| 5 | ✅ Labels and the twin (merged 19 Nov) | 252, 253 |
| 6 | ✅ Driving (merged 19 Nov) | 174, 152, 219 |
| 7 | ✅ The last Python nodes to Rust (merged 19 Nov) | Rust first |
| 8 | ✅ The challengers built (merged 19 Nov) | 212, 214-217 |
| 9 | 🧪 **Clean-up** (20 Nov): the last of Nav2 (232), Ravensim's old names (241), the leftovers checked (242); the Python twins of gps_heading, localization_health, behaviour, things and aav_safety's five nodes removed (`save/python-*` tags); `docs/codemap.md`. 22 Nov, batch 1 (Ben: "lets start removing the python fall back after being tested enough times"; tag `save/python-fallbacks-1`): the Python ego + look_ahead, io_mux, lidar_scan, lane_finder, stop_sign_watcher, point_driver + its client and arm_when_open nodes removed; their maths, launch files and configs stay. Left: the Python `legacy_uart_bridge`, `rc_driver`, `spine_bridge` (hardware-facing, kept), `camera_stream` and voxels; and 176 | 232, 241, 242, 176 |
| test | ✅ **One full `pixi run test`** on everything: 2,185 tests, 0 failures (20 Nov); `main` pushed | |

### Stage 2: score the algorithms together

The league (`measure controllers`, `measure onnx`, `tools/lane_score.py`) on the lot, the campus and
the road world, 3 runs each: diffusion, Hybrid-A* and the smooth lattice again (with ours),
the lanes in the costmap (`lanes:=camera`) against the old followers' numbers (`measure controllers` on a lane road, then `tools/lane_score.py` on its results: `measure lane` went with the followers), NDT leading on the campus,
AMCL, the challengers of batch 8. Scores are tracked, not gates; what clearly won't work is
stripped (tagged `save/` first, its scores kept).

✅ Run 20 Nov, overnight: [the scores](docs/league-results-2026-11-20.md). `mppi_lag_1k` is
ours' steering now; diffusion from Rust plans every time (the Python worker on Core ML
barely); `path:=lane` keeps right 100 %. `sim-check --moving` passes since the fix (189). Left:
AMCL on the campus (no 2D map), the Orin's times.

### Waits for the car

The car is being upgraded (18 Nov): phases 0, A-D of [the testing plan](docs/testing-plan.md)
(246-250, 173).

## 1. The testing plan

[The testing plan](docs/testing-plan.md) (`docs/html/aav-testing-plan.pdf` once `pixi run docs` has run, for the team):
the testers' strategy, session by session (rewritten 19 Nov). Ben, 18 Nov: "My prof wanted a
full automated testing system"; 19 Nov: "Add offline processing to the list of stuff we can
and will do".

| # | Task | Status |
|---|---|---|
| 254 | **Phase 0, after the install**: [one page of checks](docs/after-install.md) for each laptop, each car computer, then the car together, before phase A | ✅ written 19 Nov (`aav-after-install.pdf`); ⬜ used on the car |
| 246 | **Phase A, on stands**: `pixi run car-test --stands`: every sensor, steering, throttle on free wheels, faults on purpose (the Orin lost, a sensor stopped, the e-stop) | ✅ built, passes in Ravensim; ⬜ the car |
| 247 | **The debug screen**: who is driving, what each asks, what ego sends, the gate and why (key D on the map) | ✅ built; ⬜ seen on a screen; ⬜ the car |
| 248 | **Phase B, the hallway**: speed-up, coast, brake, steering; IMU blended with the wheels; each run over the model, with error bars | ✅ built; ⬜ the car. Open: 0.05 m/s wheel noise trips the runaway rule (a person decides) |
| 249 | **Phase C, recording**: `pixi run record-drive`, every raw sensor at full rate | ✅ built; ⬜ the car. **87 GB/h**: a big SSD (tell the hardware lead) |
| 250 | **Phase D, maps from drives**: `pixi run map-drive` (KISS-SLAM): NDT's map, a 2D map, 64 m chunks; drives merged | ✅ built, 2.3 cm off the truth in Ravensim; ✅ 3+ drives and the 2D maps merged (`--merge`, `map-area`, 19 Nov); ⬜ the car, loop closures on a real place |
| 251 | **Phase E, the map editor**: [`pixi run map-edit <drive>/map`](docs/map-editor.md): layers (OpenStreetMap, our points, the 2D map, chunks, labels, the path), align, box-delete, label, undo, approve into a new folder | ✅ built 19 Nov; NDT on an edited sim map 3.6-4.9 cm; ⬜ seen on a screen; ⬜ a real drive |
| 252 | **Phase F, labelling**: open-vocabulary models offline, camera boxes onto lidar points, fixed in the editor, our small model retrained after each drive | 🟡 built 19 Nov (batch 5, [Labelling](docs/labelling.md)): `label-drive` (OWLv2, Apache-2.0, on mps) → `labels.yaml` (`source: camera`, `seen_in`) → `map-edit` → `label-train` (a point segmenter, ONNX, Core ML 0.6 ms); Ravensim campus lots P4, P5: labels 73 % right, 50-77 % found. ⬜ a real drive (camera-lidar calibration, TODO 156) |
| 253 | **Phase G, the campus twin** in Ravensim from our maps; passes when a replayed drive scores within NDT's error of the real one | 🟡 built 19 Nov (batch 5): `pixi run twin-world <map>`, `sim world:=<twin>.sdf` starts (lot P5's twin). ⬜ the pass test (stage 2) |
| 255 | **The boot self-test that refuses to arm** (Ben, 19 Nov: "make sure we have control over everything"): at start-up every sensor at rate, every actuator answering (the spine's self-test, 0b), every lock alive, the clocks; READY / NOT READY and why on the HUD; `spine_bridge` refuses `/spine/arm` until READY. Moves nothing | 🟡 built 19 Nov; untested on the car: `aav_self_test_rs` next to the mux, `/safety/self_test/status`; `spine_bridge require_ready` (the car's) refuses to arm until READY. On the HUD (20 Nov): a small green READY, or a red NOT READY banner with why (hud-core `self_test`; unit-tested, untested on a real screen); with `actuators:=uart` nothing arms, so READY is for the person (card 1); 0b still weak |
| 256 | **NDT in the hallway, a third measure of distance**: fixed boxes and cones along the walls, the hallway mapped first (`record-drive` + `map-drive`), NDT's pose recorded by `car-test --hallway` beside the wheels and IMU; trusted across more than along (a plain corridor lets it slide) | ⬜ needs 261 |
| 257 | **The transfer functions with and without a person aboard**: `car-test --hallway --aboard <kg>` notes the load; a fit per load into `vehicle.yaml`; the adaptive brake seeded from both | ⬜ |
| 258 | **The adaptive brake on the car**: today it runs only in the sim's `sim_adapter` (`adaptive_brake:=true`); on the car nothing adapts. Put it on the car's brake path (`spine_bridge` or `spine-core`), never above `safe_max`, the locks never through it | 🟡 built 19 Nov; untested on the car: `legacy_uart_bridge` after its pi loop (`car adaptive_brake:=true`, off by default; needs `speed_loop.mode: pi`), the Rust crate `adaptive-brake` with the Python's vectors; not in `spine-core` yet (`actuators:=can` refuses it). docs/stopping.md, on the car |
| 259 | **Occupancy across passes**: 3+ drives of an area, each ray traced; a cell hit in some passes and free in others (a parked car, a person) removed; never-seen-free kept; then camera labels (252) for cars parked every time. Learn from Removert, ERASOR, DUFOMap (licences to check) | ✅ built 19 Nov (`tools/passes.py`; ideas only: Removert CC BY-NC-SA, ERASOR GPL-3.0, DUFOMap BSD-3); proved on 3 Ravensim drives, a parked box in one removed, the walls kept; ⬜ real drives; ⬜ camera labels (252) |
| 260 | **The offline pipeline as one command**: a campus area from its drives: `map-drive` each, merge, occupancy, chunks, one report | ✅ built 19 Nov: `pixi run map-area <area> <drives...>` into `maps/<area>/`, opens in `map-edit`; ⬜ a campus area |
| 261 | **NDT on the car**: `car.launch.py` has no `ndt:=` / `ndt_map:=` (only the sim launch has); needed for the hallway (256) and `pose_source:=ndt` (210) | 🟡 built 19 Nov; untested on the car: `ndt:=true ndt_map:=` (the Orin's `ndt` part, off by default); `pose_source:=ndt` only with both, wired as the sim |
| 266 | **Three step tests still missing** ([Proving it](docs/proving-it.md)): hold a speed (how far it wanders), a steering step on the ground (yaw rate: delay, overshoot, settling), a lane change (sway); each with `--sim`, the same report both ways | ⬜ |
| 267 | **One step-test harness for every path follower**: a sideways step and a speed step, the same for each `tracker:=`, one table (today only `mpc_lag_rs`'s own crate test has the sideways step) | ⬜ |
| 268 | `sim-check --ghost` listens for the local costmap on its old name (`/local_costmap/costmap`; ours is `/drivers/nav/costmap/local`), so the leg reports FAIL for every follower | ✅ 22 Nov: it reads the same name as the other legs (`LOCAL_COSTMAP`); run: PASS with `--tracker mpc_lag_rs` (0.29 m before it, the costmap 100 of 100). ⬜ with the default `mppi_lag` it still FAILs, twice of two, for another reason: the gap reads `nan` (the car stands turned aside, so the ghost is not straight ahead in its scan) |
| 269 | **A run log**: every test run's commit, command, settings and result in one place, so a number can be traced | ⬜ |
| 270 | **The car's scores beside the simulator's** for the same route: the last step of the ladder against the first | ⬜ after the car drives |
| 271 | **The test ladder with saved progress** (Ben, 22 Nov: "spin up the testing and then start working through the rungs saving our progress as we go"): `pixi run ladder` ([Proving it](docs/proving-it.md)); then a window for it, `pixi run ladder-window` (its own program in `aav_dev_window`, asking the tool; it starts simulator runs only) | ✅ the tool, 22 Nov; ✅ the window, 22 Nov: its tests pass (`ladder-core` 38, the program's 4); opened on a Mac with `--pretend` and on the real tool with `AAV_LADDER_FILE` in `/tmp`, seen by its own screenshot. ⬜ untested: a press of Run, Stop, Record or Copy by a person (the tests press them against a stand-in tool); Linux |
| 272 | **The car on jacks in Ravensim** (Ben, 22 Nov: "put the car on jacks and take it off jacks type thing"): a switch that lifts the simulated car, so the wheels spin and steer and the car stays put, as on stands; `car-test --stands --sim` then tests what the real stands test does (today the simulated car crawls in arcs) | ✅ 22 Nov: `pixi run jacks up --sim`, `down`, `status`, and `sim on_stands:=true` ([how](docs/actuator-tests.md#the-simulated-car-on-stands-pixi-run-jacks)); lowering refused while the wheels spin; `car-test --stands --sim` runs on jacks and lowers at the end: 12 PASS, 2 SKIPPED, Ben's MacBook. Its first runs found two stands checks that a still car's noise failed (the GPS line, the IMU watch): fixed. ⬜ left: the free-wheel mass (20 kg) and drag are guesses; the odometry reads the rear wheels (`speed_wheels:=front` for today's car: TODO 49b); the car is not raised; the camera row 274; the hardware panel and debug screen do not show it; **untested on the car** |
| 273 | **Every rung of the ladder in the simulator, with the screens open** (Ben, 22 Nov: "do all the testing in simulation ... so I can figure out if we are testing everything"): Ravensim, the hardware panel, the debug screen and the ladder window side by side; the lot and route rungs get a one-command simulator twin (today: by hand) | 🟡 22 Nov. **Built and run in Ravensim** (Ben's MacBook): `pixi run dry-run` (Ravensim, its picture, the ladder window; `--panel`, `--debug`; Ctrl-C left nothing running); `car-test --sim --attach` tests the simulator that is open (stands 12 PASS 2 SKIPPED, hallway 12 of 12 passes, both through `ladder run`); the window shows the last session as a table with a graph and numbers, and a Run for each test; `car-test --lot --sim` and `measure controllers --tours lot` are the lot's and the route's twins; `pixi run clear <latch> --sim`; [what can be dry-run](docs/proving-it.md#what-can-be-dry-run-from-the-screen). ⬜ **Not done:** nobody has clicked the window's buttons beside a simulator (the commands were typed), and the final window (with the Show switch) has no screenshot (the screen had locked); the lot's twin fails today (TODO 275) and does not judge the 10 % / 0.2 m rule; the route's twin has no picture and is not scored by `score_drive.py`; nobody turns the simulated car round (340 m, then restart); cards 1, 3, 7 and 8 have no rung; "on stands" on the hardware panel and the debug screen |
| 274 | **Ravensim's cameras have no noise**: on jacks, with the wheels turning, every picture is the same, so the sensor watchdog says `camera FROZEN` (a real camera's noise never repeats). `car-test --stands --sim` step 5d then passes on that word, not on the lost camera. Give `aav_sim_cameras` a little pixel noise, or have 5d ask for `STALE` | ⬜ found 22 Nov |
| 275 | **The stage-4 tests stop themselves above 1 m/s in Ravensim**: `speed-up` (3 m/s), `brake` (2 m/s) and `throttle-map` (1.5 m/s) end on the runaway rule, "the speed rose 2.86 -> 3.04 m/s while the command fell (3.00 -> 0.00): a stuck throttle?". So the lot rung cannot pass at 2 to 3 m/s. The rule is a safety rule: unchanged | ✅ in Ravensim, 23 Nov. **Done 23 Nov (Ben chose: hold the top speed longer):** `speed-up`, `brake` and `throttle-map` now hold their top speed 2.5 s (`procedures.PLATEAU`) before the command falls; the runaway rule and the speed loop are unchanged. `car-test --lot --sim --no-manual` with the `default` loop: 6 of 6 PASS. Each run needs up to 7.5 m more room at 3 m/s. ⬜ the car |
| 276 | **Finish the port from RViz: our own top-down navigation view** (Ben, 23 Nov: "lets finish the port so we don't rely on something that pops up in a weird way like rviz does ... run ravensim on one screen and then a hardware screen on another and a third screen to set the tests") | 🟡 23 Nov. **Built:** the map panel's new layers (trail, local costmap, ghost, keep-out circle, stop zone's box and state, body, scale and grid; `nav-core`, 21 tests); `pixi run nav-view`; RViz opt-in everywhere (`rviz:=true`, `--rviz`); `dry-run`'s three screens; the research entries in the ladder window ([RViz and ours](docs/hud.md#rviz-and-ours)). **Run on Ben's MacBook:** `barrier-watch barrier --speed 3` with our view: its own picture shows the ghost, the circle, the trail and the plan; no `rviz2` process. ⬜ **Untested:** the stop zone's box and the local costmap seen live (only with pretend data: the drive had ended in the picture); `--side-by-side`'s two pictures (the drives ran, no RViz; the screen had locked, so the pictures are black); `pixi run demo` launched with the new nav screen; `dry-run`'s hardware panel and `--nav` (the dry run came up with no RViz and Ctrl-C left nothing running, but the screen had locked, so the raylib panel could not open its window); a press of a research Run; Linux. Left: `pixi run view` and `radar_rviz:=true` still use RViz (asked for by name); delete the `.rviz` files once the view has been used for a while |
| 277 | **Wrapping up** (Ben, 23 Nov: "We have done way more than we should so maybe build a testing suite and test it on the car but also in a virtual env. Lets do the virtual first"): [the plan](docs/wrap-up.md): the ladder rebuilt round ten rungs named for what the car can do, each with a graph and a report; one software in two worlds and the twin; fewer commands, four screens; the final report from the rungs | ✅ the plan, 23 Nov; 🟡 step 1: the ladder is the ten rungs and **rung 1 is done** (23 Nov: its answer, graphs and saved report, in the simulator; [Proving it](docs/proving-it.md#rung-1-on-stands-accelerate-brake-turn)); ⬜ rungs 2 and 3, once Ben has read rung 1; ⬜ a whole run from rung 1's own Run button (pressed once, 24 Nov, over another run: two runs at once are refused now, and a run is recorded under its own folder) |
| 278 | **Ravensim as the car, and eight screens become four** (Ben, 25 Nov: "I want to be able to do a full sweet of tests on a virtual car using the full ros messaging system"; [the plan](docs/wrap-up.md#4-the-screens-eight-become-four)) | 🟡 **step 1 done, 25 Nov:** `sim esp32:=true` runs the boot self-test (READY) and a battery stand-in saying SIM; `pixi run check virtual`: 0 failed on Ben's MacBook (road world); `drive-to-point --sim --ahead 10` through the car's UART bridge: stopped 0.06 m from B. Found: the battery stand-in said BENCH and the output mux latched STOP (fixed: `source:=sim`); two rc_pad topics were marked live in every sim (only with `rc_pad:=true`). ⬜ the Rust ESP32's uplink in the stand-in (steering raw, uplink health: blank on today's C++ car too); ✅ **step 2, 25 Nov:** the hardware page in the map program (F2 or its button; `map --demo --hardware-page`), one implementation with `hw_panel` (panel-app's `HardwarePage`), its node `hw_panel` in the map's process, every rule kept; tested in `--demo` only (pictures in docs/hud.md), not yet on the car's 1920 x 1080 screen or with ROS live. 🟡 **step 3, 25 Nov:** the test bench is the hardware page's LADDER tab (`hw_panel --page ladder`, `map --hardware-tab ladder`): the ten rungs, Run in the simulator and Stop, the verdict, the graphs with their numbers, the faults, the jacks, from `ladder-core` (its rules kept; no car test); tested with pretend data only (pictures in docs/hud.md); left: rung 1 run from it (the step's "done when"), the checks, one test at a time, Record and Open the report (still `ladder-window`'s). 🟡 **step 4, 25 Nov:** the world view is the developer window with the debug screen's pages (SENSORS, WHERE AM I, WHAT IT SEES, PLAN; `dev-window --page where`), each localizer's car in 3D and on the map (key W) and AMCL's particles (key A), from `debug-core`'s own state (a path dependency it already had); listens only, no new publisher; tested with pretend data (pictures in `aav_dev_window`'s README) and a short ROS check on a domain of its own (IMU, GPS rows live); left: the raw lidar's kept cubes and NDT's match as ellipses (not ported, [why](ros/hud/aav_dev_window/README.md#the-world-view-the-debug-screens-pages)); `debug-hud` and the `.rviz` layouts kept until step 5. 🟡 **step 5, 26 Nov:** the four commands first in README's Every command, the old names marked "old name: use ..." (none deleted). `pixi run sim` = `sim.launch.py setup:=car` (a named set-up, `aav_description` `setups.py`: world:=road esp32:=true; with no set-up the old defaults stay, so every test starts what it did); `pixi run screens` (`tools/screens.py`: the world view, the HUD, the map window, for the car on `/system/source` or `--sim`/`--car`; the real car only watched: no HUD, the map without its hardware page); `pixi run ladder all --sim` (every built rung, one report; `--car` refused). Left: the old names removed once the new ones have been used for a while. ✅ **The dash page is watch-only on the car** (Ben, 26 Nov, option A: "calibration stays laptop-only"): the map's HARDWARE page on REAL, or with nothing announced, has no drive, calibration, RUN or jacks buttons, E-STOP only (`panel-core` `controls`, tested for every host and source; [docs/hud.md](docs/hud.md#who-may-do-what-where)); `hw_panel` and the map on SIM or BENCH unchanged; seen only in tests (no window: the screen was locked). ⬜ step 6 |
| 284 | **The corridor: two lines, not one** (Ben, 26 Nov: "maybe instead of a line we use two lines and the car just needs to stay between them"; [design](docs/designs/2026-11-26_corridor.md), step 2): `mpc_lag_rs` clips e_y's target to the room to each lane line (less half the car and 0.2 m), point by point, from the lane map; the tube where no lane is known | 🟡 **built 26 Nov**, a challenger, off by default (`mpc_lag_corridor:=true`; [README](ros/drivers/aav_mpc_lag/README.md#the-corridor-ben-26-nov-a-challenger-off-by-default)); its tests pass. **League 26 Nov** (University Drive, `path:=lane`, `--reversing`, 2 runs each, Ben's MacBook): the corridor 5/6 legs, sway 0.175, the long straight 0.137; the tube 6/6, sway 0.171, the straight 0.107; both 100 % in the lane, 0 % over the centre line, 0 % past the kerb. The lost leg: the car stopped 1 m off the lane's centre at the last goal (the goal check is 1 m; the room shrinks over the last 5 m only), and the next plan found the car didn't fit at its start (as the 0.8 m tube's lost leg on 21 Nov). Not yet better than the tube: stays off by default. ⬜ the room shrinking sooner before a goal; the camera's lines where there is no map; step 3, against MPPI's lanes |
| 285 | **Rung 1 stops itself on stands in Ravensim** (screens day, 26 Nov, `pixi run ladder run 1`, twice): the preflight asks 0.3 m/s; the brake lets go over ~0.6 s while legacy_uart_bridge's PI speed loop (kp 2.0) builds throttle to 0.26; then the free wheels spin 0 to 0.51 m/s in 0.25 s and the 0.46 m/s limit stops the run. On stands the push comes from `jacks.py`'s `FREE_WHEEL_MASS` (20 kg: a guess, never measured). The limit is a safety rule: unchanged. Also: the report says "speed loop: none: open loop" for the legacy bridge (`reports.py` shaping), out of date since its PI loop | ⬜ found 26 Nov. **For Ben:** measure the free wheels' spin-up on the real stands (jacks.py says how); on stands, hold the loop's integral until the wheels turn, or a gentler loop |
| 286 | **What the screens showed on the virtual car** (26 Nov, `pixi run sim` + `pixi run screens --sim`, all four live; `check virtual` 0 failed): (a) GRAPHS' **mux** hop says NO DATA: `/control/car/actuator_cmd` is `not_virtual`, so the chain can't be seen whole in the sim; (b) the HUD shows `/perception/stop_sign` NEVER in red: allowed on the virtual car, but it reads as a fault; (c) the world view's map draws `maps/ndt/test_lot.pcd` (`dev_window.sh`'s fixed `ndt_map`) on the road world | ✅ (a) fixed 28 Nov: hw_panel (and the web page) read the output mux's simulator command too (`sim_command_topic`); on the virtual car every hop it can show is OK (the ESP32's: NOT TESTED, version 1 frames). Two more false verdicts found there and fixed: the bridge's steering sign (-1) read as DIFFERENT (now undone, `frame_steering_sign`, a test keeps it equal to legacy.yaml), and the PI speed loop's throttle read as DIFFERENT (now: on the same way, or braking). ✅ (b) and (c) fixed 26 Nov (TODO 288): the virtual car runs the stop-sign watcher (live, `not_virtual` gone); `dev_window.sh --live` asks the simulator its world and draws that world's NDT map (none on the road world) |
| 287 | **`pixi run sim` sometimes starts slowly, and `sim_cameras` sometimes ignores Ctrl-C** (Ben, 26 Nov: "pixi run sim takes forever to start now"): timed twice on Ben's Mac. 1st start: the load at 27 with about a third of the CPU busy (no swap, no throttling), the sim's clock pausing, the self-test back to the start 9 times: READY after ~240 s. 2nd start, a minute later: READY in ~6 s. Not the ESP32 stand-in (its link is up in ~2 s). After the slow run, Ctrl-C left `sim_cameras` running over a minute (stopped with SIGTERM); after the fast one, all gone in 5 s. A cold disk cache (45 Python programs) is the guess, not proven | ⬜ found 26 Nov. Next: time a cold start (after a reboot); if it repeats, start the Python nodes in fewer processes. ✅ 28 Nov: `sim_cameras` stops on Ctrl-C (Bevy's handler only flagged it for an update that never came; before: still running 20 s after SIGINT, now: at once) |
| 288 | **The virtual car, a watch-only Ravensim, the tests window, one map** (Ben, 26 Nov: "Maybe we need a ravensim that is just the monitoring everything and displays the results. THats it. Next we have a pixi run virtualcar and this creates all the sensors and actutors and boots the esp32 and loads all the orin stuff too like all the pi and the preception and planning and all that stuff. Basically everything the car would be providing that it isn't and then we have the testing screen and the hardware screen. why is the testing screen now a command prompt instead of a screen. Also, the sim has a display that shows a road with a stop sign and its clearly identified as an object. the debugging screen sees it too. The behaviour drives through it like nothing. And the nav map shows something completely different. Maybe we can create a consinstent map between all 3.") | ✅ **built 26 Nov** (branch `virtualcar`): `pixi run virtualcar` = `setup:=car`, now the campus, the ESP32, the Rust detector and the watcher (`sim` its old name); `pixi run ravensim` only watches (the world view on the running world's maps; refuses `name:=value`); `pixi run ladder` alone opens the tests window (`ladder table` the text); `screens --sim` opens the HUD and the map (the world view is ravensim's). `pixi run campus`: a stop sign at each OSM stop (85), its line set back before the road met, the lines in `lanes.yaml` `stops`; the behaviour planner reads them (`map_stops`, heading-matched). **Seen 26 Nov, campus, P4 to University Drive south through its all-way stop:** detector off: APPROACH stop_line, stood 2.4 s with the bumper 0.04-0.10 m past the line, GO; detector on: WAIT sign ~4 s about 6 m before the line, and its line then served (one stop, not two); `check virtual` 0 failed. **Left:** (1) the detector also fires on a sign's grey back: on the way out of P4 the car stood ~4 s at the sign at (57.8, -41.5), meant for the cross road; (2) a sign seen far or flickering holds APPROACH at the 0.3 m/s floor for up to ~55 s (left_m 1-9 m jumping): the watcher's range from the box height; (3) one start of four had the global EKF stuck at (0, 0) and still said READY (the planner then: "the car doesn't fit at the start"); (4) the window screenshots came out black (the Mac's screen was locked): look at both screens on campus by eye. ⬜ next: ignore a sign whose box says its back (or trust the map where it has stops), the boot self-test checking the global pose against the GPS |
| 289 | **Web screens: the ESP32's page and the Orin's page** (Ben, 27 Nov: "on the esp32 we should have the very simple vehicle controls and the go slack and measurement stuff and then the rest should be a stream of data"; [design](docs/designs/2026-11-27_web-screens.md)): the ESP32 page gets Controls (creep, turn, LIMP), Measure and the hardware tests (`bench-core`, `wifi-control` off by default); the Orin page (axum, read only) graphs ROS → CORTEX → ESP32 and shows the car's status and ROS against the contract; the virtual car serves both; the raylib screens kept for now | 🟡 **steps 1-2 built, 27 Nov** (`aav_web`, aav_hud's `web-app`, `pixi run virtualcar` serves it on 127.0.0.1:8780): CAR all OK, ROS 65 of 65 live, 0 problems; HOPS ego OK driving. It found: the virtual ESP32's `rx` was made up (fixed: the last frame it got) and version 1 frames were paired by check byte alone (39 of 100 "differed"; fixed in panel-core: NOT TESTED). ✅ **step 3, 27 Nov:** `bench-core` (no_std, 20 tests): held-to-move buttons into the spine's own test mode and LIMP, a trace, step responses, three hardware tests. ✅ the delays between hops, 28 Nov (ego to mux by stamp, CORTEX to ESP32 by counter: version 2 frames; to 5 ms). ⬜ the page never seen in a browser (no browser here: its script run on a real snapshot in JavaScriptCore); step 4 needs a Rust virtual ESP32 (today's is the C++ in Python, no test mode); steps 5-6 |
| 290 | **The radar driver survives a short frame** (old 111, 4 Nov weak links; dropped in a reorganisation, found again by the 28 Nov docs review): `TI.py` `read()` died on a frame it couldn't parse, and `pc_publisher.py` reused the finished reader 20 times, then shut down | ✅ 28 Nov: a bad frame is skipped (`None`), every mode skips it, the reader's end stops the node with its own message; `test/test_ti_read.py` (fails without the fix). The Rust port (265) should keep the rule |
| 291 | **Remove Stanley, LQR and pure pursuit** (Ben, 26 Nov: "aren't used really at all"; the rest stays) | ✅ 28 Nov: the 26 Nov helper's removal, finished and merged: `aav_track` keeps `mpc`, `mpc_full`, `mpc_lag`; the debug HUD's key 1 is MPC; scores kept in `docs/trials.md`; tag `save/trackers-stanley-lqr-pp-2026-11-26` |
| 292 | **Generated docs out of git; CI builds them** (Ben, 26 Nov: "anything generated can go away") | ✅ 28 Nov: branch `cleanup` (d602ce24) merged; CLAUDE.md, CONTRIBUTING and conventions say commit the Markdown; `.github/workflows/docs.yml` builds the pages on main (last committed copy: tag `save/docs-html-2026-11-26`) |
| 292b | **CI faster: the build cache's file times, and the car kit only when it changes** (26 Nov helper, unfinished) | ⬜ on Ben's fork branch `cleanup-wip` (b978323a): `tools/ci_mtimes.py` (+ test), `ci_docs_only.py`'s `car_kit=`, `colcon_test.sh`'s `AAV_CI_QUICK`; not yet wired into `.github/workflows/test.yml`, and only CI can prove it |
| 293 | **Ben's messages word for word for 8-12, 15, 17 and 20-26 Nov** (CLAUDE.md, "Where decisions live"; TODO 271-288 quote days with no file) | ✅ 28 Nov: `docs/history/messages-2026-11-08.md`, `-11`, `-15`, `-17`, `-20`, `-21`, `-22`, `-23`, `-25`, `-26`, from the session records; his own research, logins, addresses and pasted logs left out. No file for 9, 10 and 12 Nov (no record exists) or 24 Nov (one passing remark, no work) |
| 294 | **`decisions.md` and `status.md` brought up to date** (23-27 Nov decisions: the top speed held, the dash watch-only, the trackers, the Orin's web page read only, the web screens; status still says 18 Nov) | ✅ 28 Nov: decisions.md's "Decided from 22 to 27 November" (12 rows); status.md's "Since 20 November" (8 rows) |
| 295 | **The spine's test mode can't turn the wheels**: `TEST_MAX_THROTTLE` is 0.15 of the DAC (about 105 of 700) and the wheels start at 475 forward, 525 back (#52, Arsh, 27 Nov); bench-core's CREEP (0.08) the same. The test mode's throttle should start from `start_throttle`, as the open loop does, capped a little above it | ⬜ a team choice in spine-core (found reviewing #52, 28 Nov) |
| 296 | **CARLA once, on the lab's NVIDIA PC** (Ben, 15 Oct "carla 5"; 1 Nov "plan for that as part of car day") | ⬜ never run against a server (`docs/carla.md`); needs the lab PC's CARLA version |
| 297 | **Back up every car computer before any reflash, and find `best.pt`** (Ben, 10 and 15 Oct; 14 Nov review) | ⬜ no audit records a backup; `best.pt` only "probably" on the camera Pi |
| 298 | **Cut the long pages down** (EGM review, old 109; Ben's short pages): `designs/2026-11-05_one-window.md` 541 lines, `behaviour-and-speed.md` 545, `car-day.md` 1,141 | ⬜ |
| 299 | **What each stop cuts** (6 Nov car audit: the red button, the wireless e-stop, the spine's stop: throttle, brake, steering, power) | ⬜ the hardware lead's to answer |

## 2. Maps and position

| # | Task | Status |
|---|---|---|
| 210 | **NDT leads, the GPS checks** (`pose_source:=ndt`): a failing GPS no longer moves or stops the car | 🟡 merged 19 Nov, a challenger: the lot scored (every leg with the GPS stopped, jumped or drifting); ✅ the campus scored 20 Nov (every leg; 92 % of clouds gave a pose, heading off 0.1° at worst, 1.7 ms a match: [scores](docs/league-results-2026-11-20.md#position-and-the-checks)); the car refuses it until 261 and a real map |
| 231 | **AMCL in Rust** (`aav_amcl_rs`, the default): NDT's recovery | 🟡 ties stock on the lot; left: the campus, more runs |
| 121 | **NDT's score floor on the campus** (1.5 there, 2.3 in the lot): how high does a wrong pose score? | ⬜ |
| 175 | **Three levels, fixed chunks**: routes, paths and moves on 64 m chunks | 🔧 20 Nov, scored on the campus (3 runs each): `global_window:=192` ❌ 5 of 9 legs against 9 of 9. "Along the road" failed every run: the planner searches the whole 192 m window, runs out of time (error 207; 87 % CPU, 600 MB) and the car never moves. Kept off. Left: a route level that hands the planner one chunk's stretch at a time, then score it again |
| 191 | **Stops reset the drift**: wheels that read nothing mean the car stands | 🔧 left: the parked yaw creep, before and after (the car) |
| 94 | **Drive-to-point stops 0.61 m past B** (limit 0.5 m) | 🔧 left: more runs in Ravensim |

## 3. Driving

| # | Task | Status |
|---|---|---|
| 152 | **Our planner faster**: its checks on every core (rayon), then `mppi_lag` with 1000 samples; timed on the Orin | 🔧 built 19 Nov; scoring in stage 2: the planner 18-21 % faster, the same paths (rayon measured 7-20 times slower, dropped: `docs/own-planner.md`); `steer:=mppi_lag_1k`, 0.86 against 1.56 ms a tick; **stage 2: no column worse than `mppi_lag_calm`, half the work: `stack:=ours`' steering since 20 Nov** ([20 Nov](docs/league-results-2026-11-20.md)). Left: the Orin's times, then `stack:=car` |
| 174 | **Reverse everywhere**, through cusps; a narrow street; reversing in `aav_track` | 🔧 built 19 Nov; scoring in stage 2: `aav_track`'s trackers reverse (`gears.py`); a blocked narrow street backs out (planner scene, navigator test, `sim-check --blocked-street`); ✅ `--blocked-street` passes in stage 2 ([20 Nov](docs/league-results-2026-11-20.md)) |
| 189 | **Speed hunting**: one person crossing flipped behaviour 4 times; a 1 s hold built | ✅ 20 Nov: `sim-check --moving` failed twice (the walker's feet were kept as a kerb for ever, and the car stood before it); fixed in the voxels' step memory (`kerbs_only`), then ✅ passes: WAIT(moving) > GO > DRIVE > the goal |
| 163 | **The sim car vs its command**: `speed_loop_model:=` built; the backing legs go 1 m off | 🔧 |
| 169 | **Followers on the road ahead's line** (the lane centre, `/world/road_ahead`) | 🔧 `mppi_lag` done; left: `mpc_lag`, the campus live |
| 219 | **Kerbs at speed**: found 4.7 m out (too late at 3 m/s), 0.10 m never; behaviour slows for them | 🔧 built 19 Nov; scoring in stage 2: both behaviour twins slow for a kerb on the road ahead; a kerb under every beam is kept while the car stands before it. 0.10 m still never seen |
| 206 | **The pedal**: the stick as accelerator, brake and reverse in ego | 🔧 left: a joystick drive in the sim, tune `pedal_let_off`; `ai` as a challenger |
| 171 | **No switch leaves a gap**: respawn, age checks, the nodes' own tests | 🔧 |
| 159 | **The safety case**: one page per hazard | 🔧 left: a throttle stuck wide open; the spine in the loop |
| 207 | **Stuck steering and speed noticed** (the watchdog) | ✅ in the sim; ⬜ the car (needs a measured angle, Q2) |
| 264 | **Lanes are perception, the costmap prices them, navigation drives** (Ben, 19 Nov: "put lanes in the cost map and then behaviour keep to the right side. then the motion planner can just cross the line when its go no other choice", then "drop there code all together ... curb, high cost, yllow line med cost, lright side low cost, car in the way high cost, person in the way high cost"): the lane finder tells yellow from white and the road's edge; the lanes layer (148, `lanes.rs`) is drawn from the camera's lines where there is no lane map: kerb and off-road high, the yellow line and the oncoming lane medium, the right lane's centre low, every lane cost below a wall's; cars and people stay walls (the obstacle layer). Scored on the same roads against the followers' numbers (262: PID 3.9 cm, pure pursuit 2.4 cm on Library Road). Then **the lane-follower drivers are removed** (the 2025-26 PID, its Rust port, pure pursuit, Stanley, the `lane_follower*` switches), tagged `save/` first; the lane finder and `LaneGeometry` stay | ✅ built 19 Nov: both lane finders say each line's paint (`left_paint`, `right_paint`); `aav_costmap` `camera_lanes.rs` draws the lines with the lane map's own costs (`lanes::across`), `lanes:=camera` or `both` (the map wins where it has a road); the car's lane is the one it is in, a yellow right line puts it in the oncoming lane ([docs](docs/own-costmap.md#the-lanes-from-the-camera-19-nov-todo-264)). The followers removed (tag `save/lane-followers`, [scores](docs/history/lane-followers.md)). ✅ scored 20 Nov on University Drive: the lattice with `lanes:=camera` 59 % in the lane (48 % without), the lane map 100 % ([scores](docs/league-results-2026-11-20.md#lanes-university-drive-toolslane_scorepy)). Left: the camera's lanes where they miss lines |

## 4. Challengers

Each runs beside the champion until it wins on the same routes (Ben, 5 Nov). The ones
scored with Nav2's follower (177, 178) are scored again with ours.

Stage 1, batch 8 (19 Nov): 212, 214, 215, 216 and 217 built and unit-tested, each off by
default behind its league option ([switches](docs/switches.md#stage-1-batch-8-challengers-built-19-nov-scored-in-stage-2));
stage 2 scores them in Ravensim.

| # | Challenger | Status |
|---|---|---|
| 178 | **Hybrid-A\*** in our planner (`planner_search:=hybrid`) | 🟡 stage 2: 15/15 legs; the calmest on the box (sway 0.044 vs 0.218), a tie on the lot; one run changed gear 6 times ([20 Nov](docs/league-results-2026-11-20.md)) |
| 177 | **Steering-rate-aware lattice moves** | 🟡 stage 2: a tie on the lot, worse on the box; one run cut inside the turning radius (0.50 m) ([20 Nov](docs/league-results-2026-11-20.md)) |
| 213 | **`mpc_lag_rs` that reverses and doesn't swing** (Autoware's MPC idea); was 132 | 🟡 **reverses since 20 Nov** (Ben: "fix mpc to have reverse"): cut at cusps, mirrored, stands at each cusp ([README](ros/drivers/aav_mpc_lag/README.md#forward-and-back-todo-213)); lot tour `--reversing`, 1 run: 4/4 legs, 7.8 m backwards, sway 0.054. League 21 Nov (3 runs, [results](docs/league-results-2026-11-20.md#mpc-against-mppi-21-nov-todo-213)): 12/12 lot, 9/9 University, 1.6 ms a tick, sway 2x MPPI's. 🔧 the sway: the tube (Ben 21 Nov), tuned: **0.3 m the default**, University Drive's sway 0.251 to 0.160 (MPPI 0.132), every leg; the lot unchanged. ⬜ `r_rate`, the costmap in its cost |
| 212 | **Smooth MPPI** (sample the steering's change): `steer:=mppi_lag_smooth` (`mppi_lag_sampling:=smooth`) | 🟡 stage 2 against `mppi_lag_calm`: the calmest (sway 0.018 vs 0.025) but further off the path on the box (0.22 vs 0.14 m) ([20 Nov](docs/league-results-2026-11-20.md)) |
| 214 | **Passing with a jerk limit** (Apollo, Autoware): `path:=lane_jerk` (`lane_pass_shape:=jerk`) | 🟡 stage 2 against `path:=lane`: calmer on the box (0.108 vs 0.148), further off the path (0.22 vs 0.11 m); the tours have no pass ([20 Nov](docs/league-results-2026-11-20.md)) |
| 215 | **The ground ring by ring** (Patchwork++), for kerbs: `kerbs:=patchwork` (`ground:=patchwork`) | 🟡 stage 2 against `kerbs:=on`: better on most columns, more left-right changes on the lot (12.5 vs 10.1); real kerbs are its test ([20 Nov](docs/league-results-2026-11-20.md)) |
| 216 | **Kinematic-ICP**: lidar odometry that knows the car can't slide sideways: `pose:=kinematic_ekf` (`odom_lidar:=kinematic_icp`, `aav_kinematic_icp`) | 🟡 stage 2: no gain in the sim, worse off the path on the lot (no wheel slip there); kept for the car ([20 Nov](docs/league-results-2026-11-20.md)) |
| 217 | **A tracker with a motion model per kind** (car, person): `behaviour:=on_per_kind` (`things_tracker:=per_kind`), both twins | 🟡 stage 2 against `behaviour:=on`: better on the lot, worse on the box (sway 0.241 vs 0.204); nobody walks on these tours ([20 Nov](docs/league-results-2026-11-20.md)) |
| 243 | **Our models from Rust** (`ort`): the detector (8 ms) and the diffusion planner's model (`diffusion_impl:=rust`, Core ML takes all 2,178 nodes, 9.7 ms a plan against the CPU's 87) from Rust on a Mac, both checked against PyTorch; `pixi run measure onnx` (answers, coverage, ms, FP16); `onnx-session` shared; TensorRT ready, untested ([ONNX](docs/onnx.md)). Left: the Orin (JetPack 7.2); diffusion scored on the lot 20 Nov (`diffusion_rs` plans every time, [scores](docs/league-results-2026-11-20.md)); on the campus still to do | 🟡 |
| 262b | **A learned lane finder**: Ultra-Fast-Lane-Detection v2 (MIT), trained by us on Ravensim's pictures (labelled from the lane map) and the car's (252), through ONNX (243), behind `/perception/lane/geometry`. Published weights are out: CULane is non-commercial, BDD100K, TuSimple and CurveLanes unclear ([survey](docs/perception.md#a-learned-lane-finder-the-survey-19-nov)) | 🟡 built 19 Nov (`aav_lane_finder_learned`, `lane_finder_impl:=learned`): 7,472 Ravensim pictures; held out, the car's lines 1-4 cm median at 5-15 m; Core ML 2.4 ms, 1.2 ms FP16 on the Neural Engine; ⬜ driven and scored against the classic (stage 2); ⬜ the car's pictures (252); **merged 19 Nov**: 7,472 Ravensim pictures, ResNet-18 on mps, the car's lines 1.2-3.6 cm median, Core ML 2.4 ms, FP16 1.2 ms on the Neural Engine, `lane_finder_impl:=learned`. Its colour head (95 % right) wired into `left_paint`/`right_paint` (19 Nov, batch 5). **Left:** neighbour lines often wrong (3 m at the 90th %); driving scores (stage 2) |

## 5. Rust

| # | Task | Status |
|---|---|---|
| 176 | **More Rust-like**: every crate against the Rust API Guidelines | 🔧 left: a few spine-core items (a reviewed safety change). Done 20 Nov: **one shared `CloudView`**, the crate `aav_description/cloud-view`: `voxels` (and through it `lidar-scan`, `kinematic-icp`), `ndt-core`, `radar-clustering` (and `debug-core`, the dev window) and `things-core` read clouds through it, four copies now one, every vector the same. Not on it: three nodes that read x, y inline (`costmap-node`, `behaviour-node`, `stop-zone-node`: a safety lock, so a reviewed change) |
| 195 | **rayon**: chunks and costmaps done; the planner measured, no gain (152, 19 Nov); the things (20 Nov) | ✅ done 20 Nov (`things-core` `examples/timing.rs`, Ben's M4 Max, idle): the saved map's background on many cores: the campus `/map` (6400 x 6400) grown in **7.4 ms, not 112 ms** (growing row by row did a third of that on one core, rayon the rest); a chunk crossing's 3 x 3 read in **1.0 ms, not 2.5 ms** (worst 3.0, not 11.9). Answers the same (all vectors, a new cell-by-cell test). A cloud stays on one core: 0.15 ms for 700 cubes, grouping most of it; rayon there measured slower (0.16 to 0.20 ms; 0.005 to 0.054 ms on a small cloud), not kept |
| 265 | **Batch 7: the last on-car Python nodes in Rust** (Ben, 16 Nov: "lets remove python were possible") | 🔧 done (19 Nov), the Rust the default, the Python's answers to the bit: `geofence`, `stop_hold`, `goal_leash` (`aav_safety_rs`), `gps_heading` (`aav_gps_heading_rs`). Left: `lane_pose` (a trial, off by default: needs a TF buffer at the picture's stamp and the lane map's `roads_at` in Rust; first move `aav_behaviour_rs`'s `lanes` and `tf` into a shared crate), `street_router` (on demand, not on the safety path: HTTP to Valhalla or OSRM, the campus graph from OSM XML, 1,100 lines; a poor fit for now), the battery and radar drivers |
| 197 | **Rerun** for viewing a drive: `pixi run -e rerun view-bag` | 🔧 Ben looks, keep or drop |

## 6. Clean-up

| # | Task | Status |
|---|---|---|
| 242 | **Untried after Gazebo**: `stream_check.py`'s moved sign, `sim_frames.py`, `pixi run demo` with Ravensim | 🧪 20 Nov: checked at the import and argument level (Ravensim serves `spawn_entity`, `delete_entity`, `set_entity_state`; every launch switch they pass exists); `stream_check.py`'s words say Ravensim. Not run against a live sim |
| 23 | **The nightly sim job**: drop `continue-on-error` after a few green nights (the 240 fix) | 👀 |
| 114 | **The developer window** on a slower laptop | ⬜ |

## 7. Needs the car or the hardware lead

The steps: [car day](docs/car-day.md), [know the car](docs/know-the-car.md),
[car setup](docs/car-setup.md).

| # | Task |
|---|---|
| 173 | **Ours on the car**: the seven blockers (158), then driven, recorded and scored (157) |
| 158 | **The seven blockers**: the firmware timeout, the brake follows a wish, wheel speed seen, clocks checked... |
| — | **The upgrade** (under way 18 Nov): the Pis to Ubuntu 24.04, the Orin to JetPack 7.2, after a restore is proven |
| 49b, 101 | A speed sensor on the driven wheels; then the brake follows a wish (feed-forward + PI) |
| 46, 49 | The steering and brake position sensors (from the Jrks over I2C?) |
| 22o | Ackermann: both front wheels' real angles (Q2) |
| 14, 60, 61 | Weigh the car; the wheel diameter (roll 10 m); the tracks |
| 67, 79 | The steering actuator's ends and centre; then the brake and throttle |
| 52b, 27 | Refit the car model from the actuator tests; the `smooth` speed loop on the car |
| 104 | The car identifies itself (delays, lags, braking, error bars): phase B |
| 37c, 25, 156 | The cameras on the Orin and the Pi; calibrate them and the lidar (Q22) |
| 218 | Kerbs on the car: record by a real 0.15 m kerb, then `kerbs:=true` |
| 64, 100 | The first lot visit: the datum, the clocks, walk the fence, the site file |
| 120 | The campus map against a real drive (now through 250-253) |
| 68, 74 | The car-ready checklist; the first moves (a throttle cap, the joystick's numbers) |
| 75, 76 | The localization Pi's machine file; Python's speed on the Orin |
| 92, 126 | The people named (e-stop person, safety driver); the hardware debug plan |
| 71 | The hardware panel's actuator pages (wait for the two-way ESP32 link) |
| 0b | The spine's state machine follow-up (Arsh) |
| — | Flash the Rust Picos (wheel, sentinel); a spare Pico 2 and a 3.3 V CAN module |
| — | The Mac as the operator station: measure the car's WiFi |
| — | Tune `ekf_global` for the real GPS receiver (N7, Q18) |
| — | A read-only audit of the car's network; the photo checklist (`docs/car-photos.md`) |
| 65, 125 | The 2024-25 / 2025-26 teams' recordings: find them; rename their topics |

## 8. The team decides

| Question | Today |
|---|---|
| Reversing on long drives: cost more, or stay short? (122) | ours reverses where the plan says |
| The brake after boot, the steering on an e-stop | one constant each in `spine-core` |
| The RC's right stick to take and give the car, the left stick as one pedal, the failsafe on CH6 and CH2 (Arsh and Ben, 21 Nov) | built in the firmware, not yet flashed; nine choices made while building it for Arsh and Ben to check ([design](docs/designs/2026-11-22_rc-right-stick.md)) |
| The runaway rule trips on 0.05 m/s wheel noise (248) | unchanged: a safety lock |
| The indoor geofence with `stands:=true` (hallway tests) | the hallway tests use their own distance fence |
| `ROS_DOMAIN_ID` for the car and every laptop | 0-101 safe; never above 232 |
| The spine's arming rules: boot latched, arm on a 0 → 1 edge | Proposed |
| The GPS driver | `ublox_gps`, Tentative |
| N3: `respawn` on device nodes? | none today |
| Rotate the leaked secrets (Q34); answer Q1-Q34 | open |

## 9. Ideas kept, not picked

| Idea | When |
|---|---|
| 279 **Ravensim's views for the job** (Ben, 26 Nov: "multiple views ... the nav screen when we are doing algo testing, the debug screen like ndt when we want to look at that"): named view set-ups, `view:=drive` (chase camera + the HUD), `view:=algo` (the world view's map and PLAN page beside the scene), `view:=where` (NDT's cubes, WHERE AM I), `view:=sensors`; one key cycles them; each test names the view it wants | after step 5's `pixi run screens` has been used |
| 280 **Grass that looks like grass** (Ben, 26 Nov): blades on the green parts (GPU instancing, dense near the camera, the noise grain far off), a little wind; the cameras see it too, so the lane finder and detector learn real edges | after the screens |
| 281 **Better trees**: today's are cartoony; kinds from OpenStreetMap's tags (leaf or needle), a few shapes each, their size from NDT's points where we have drives (`twin-world`) | with 280 |
| 282 **Weather and light for perception tests** (Ottawa): time of day and the sun's shadows; night with headlights; rain (a wet road's shine, drops on the lens); snow on the grass and the road's edges. Each one a switch, so a rung can ask for it | after the learned lane finder is scored |
| 283 **A lived-in campus**: parked cars in the lots, people on the paths and cyclists (moving, for behaviour's tests), faded paint, cracks and manhole covers, signs from OpenStreetMap, windows on buildings by their storeys | a nice-looking campus, down the road (Ben, 26 Nov) |
| 108 Shadow mode: the drivers score themselves while a person drives | after real recordings |
| 96 A model builds our own algorithms from our recordings | after the car's data |
| 78 The talking model: explains, never drives | design first |
| Burn (Tracel, MIT/Apache: pure-Rust inference, ONNX import) as a challenger to `ort` (ONNX Runtime), for all-Rust builds; candle and tract noted (Ben, 19 Nov) | once the Orin's TensorRT numbers exist |
| Learned scan models for AMCL (IR-MCL's idea), Transformer MCL, Scan Context | after real maps (231's scan model is swappable) |
| zenoh (`rmw_zenoh`), if Fast DDS gives trouble on the car's WiFi (205) | if needed |
| kiddo nearest neighbours (200), once ICP or grouping comes to Rust | if needed |
| Learned state-space models (Mamba-style) of what the linear model misses | after hours of car logs |
| micro-ROS again | once the hardware is known |
| numpy above 2.3.5 in the `ml` environment | when coremltools passes 9.0 |
| "Two drive paths": the bridge refuses a second mux or heartbeat | later |
| A stop sign seen stays on the map | needs two rules first |
| The little robot's eyes on the HUD | later |

## For the 2027-28 team

Learned models: each needs training data and GPUs we don't have this year (Ben, 28 Oct):
learned BEV (BEVFusion, BEVFormer); one network for perception, prediction and planning
(UniAD); world models (GAIA-1) and vision-language driving (EMMA); predicting others' paths
(MTR, Wayformer); state-space models (Mamba).
