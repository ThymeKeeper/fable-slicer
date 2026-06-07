//! Profile / configuration system.
//!
//! Planned for M2: a three-tier inheritance model
//!   printer  (bed, kinematics, nozzle, start/end g-code)
//!     -> filament (temps, flow, cooling, retraction)
//!       -> process  (layer height, walls, infill, speeds, supports)
//!         -> user overrides + per-object / per-region modifiers
//!
//! Profiles are *data, not code* (TOML via serde), versioned with migrations, so
//! printers can be added/shared without recompiling. See docs/ARCHITECTURE.md.
//!
//! Empty placeholder for now.

/// Crate version string, so the stub exports something concrete.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
