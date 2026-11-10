//! Host-testable firmware policies and transport-neutral control services.
//!
//! ESP-IDF task and peripheral adapters stay in the binary. This library owns
//! block scheduling, command delivery and readouts without depending on a HAL.

pub mod audio;
pub mod control;
pub mod ui_parameters;
