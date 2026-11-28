//! The ESP32 page's bench (docs/designs/2026-11-27_web-screens.md, step 3): what its
//! Controls, Measure and Tests tabs do, with no hardware and no web server.
//!
//! | Module | Does |
//! |---|---|
//! | [`controls`] | the page's buttons, **held to move**: a button stays pressed only while its heartbeat comes (100 ms); each becomes one of the spine's own requests, never a target of its own |
//! | [`trace`] | a short recording in RAM, one [`trace::Sample`] a tick, then read out |
//! | [`step`] | how a measured value followed a step in what was asked: the delay, the 10 to 90 % rise, where it ended (`tools/step_response.py`, ported) |
//! | [`hardware`] | the hardware tests: fixed moves through [`controls`]' requests, a recording, and a pass rule |
//!
//! **Nothing here decides what is safe.** A button becomes a [`spine_core::TestRequest`]
//! (the spine's hardware test mode: throttle at most [`spine_core::TEST_MAX_THROTTLE`],
//! forward only, out above [`spine_core::TEST_MAX_SPEED`]) or a
//! [`legacy_link::CalRequest`] (limp, with `SteerCal`'s rules: standing still, braked, no
//! e-stop, no RC). The spine and the link refuse what they refuse; RC and the e-stop win.
//! This crate only adds the page's own two rules: hold to move, and the throttle only on
//! stands ([`controls::Where`]).
//!
//! Off by default on the car: a board program serves it only with `wifi-control`, a build
//! the hardware lead decides on (W1). The virtual car runs it first.

#![no_std]

pub mod controls;
pub mod hardware;
pub mod step;
pub mod trace;
