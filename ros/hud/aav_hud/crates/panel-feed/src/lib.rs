//! The hardware panel's news, with no window: where it comes from and where the buttons
//! go. `hw_panel` and the map's HARDWARE page (panel-app) use it with their buttons; the
//! Orin's web page (web-app) uses it listen-only, with no publisher at all.
//!
//! | Module | Does |
//! |---|---|
//! | [`source`] | the [`source::DataSource`] trait, and a source with the ESP32's lines beside it |
//! | [`ros`] | the ROS node: `hw_panel`'s ([`ros::RosSource::start`]) or a listener ([`ros::RosSource::listen`]) |
//! | [`esp32`] | the ESP32's JSON status lines over its WebSocket, read only |

pub mod esp32;
#[cfg(feature = "ros")]
pub mod ros;
pub mod source;
