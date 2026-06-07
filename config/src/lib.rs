//! Settings for the slicer.
//!
//! [`Settings`] is the *resolved*, flat configuration the engine and g-code
//! emitter consume. The [`profile`] module builds one of these from tiered
//! printer / filament / process profiles (with inheritance), loaded from TOML.
//! `Settings::default()` is the in-code fallback used by tests and for any field
//! a profile leaves unset.

use std::f64::consts::PI;

mod profile;
pub use profile::{FilamentProfile, PrinterProfile, ProcessProfile, Profiles};

/// Default start g-code (generic, heats + homes directly). `{placeholders}` are
/// substituted by the emitter. Used when a printer profile sets no `start_gcode`.
pub const GENERIC_START_GCODE: &str = "\
M140 S{bed_temp}
M104 S{nozzle_temp}
M190 S{bed_temp}
M109 S{nozzle_temp}
G28";

/// Default end g-code (cool down, lift, disable steppers).
pub const GENERIC_END_GCODE: &str = "\
M104 S0
M140 S0
M107
G91
G1 Z5 F600
G90
M84";

/// Where the start/end seam of each closed wall loop is placed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SeamMode {
    /// Rear-most point of each loop — seams align into a vertical column.
    #[default]
    Nearest,
    /// Sharpest corner of each loop — tucks the seam into a corner.
    Sharpest,
    /// Scattered per layer.
    Random,
}

impl SeamMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "nearest" | "rear" | "aligned" => Some(Self::Nearest),
            "sharpest" | "sharp" | "corner" => Some(Self::Sharpest),
            "random" => Some(Self::Random),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Nearest => "nearest",
            Self::Sharpest => "sharpest",
            Self::Random => "random",
        }
    }
}

/// Fully-resolved settings the pipeline runs on.
#[derive(Clone, Debug)]
pub struct Settings {
    // --- machine ---
    pub nozzle_diameter_mm: f64,
    pub filament_diameter_mm: f64,
    pub bed_size_x_mm: f64,
    pub bed_size_y_mm: f64,

    // --- process ---
    pub layer_height_mm: f64,
    /// Thickness of the first layer (often thicker for bed adhesion).
    pub first_layer_height_mm: f64,
    pub line_width_mm: f64,
    pub wall_count: usize,
    pub top_layers: usize,
    pub bottom_layers: usize,
    /// Sparse infill density, 0.0..=1.0 (0 disables sparse infill).
    pub infill_density: f64,
    /// Number of skirt loops around the first layer (0 disables).
    pub skirt_loops: usize,
    /// Gap between the skirt and the model (mm).
    pub skirt_gap_mm: f64,
    /// Where to place the wall seam.
    pub seam_mode: SeamMode,

    // --- retraction ---
    pub retract_len_mm: f64,
    pub retract_speed_mm_s: f64,

    // --- temperatures (°C) ---
    pub nozzle_temp_c: u32,
    pub bed_temp_c: u32,

    // --- speeds (mm/s) ---
    pub print_speed_mm_s: f64,
    pub travel_speed_mm_s: f64,
    pub first_layer_speed_mm_s: f64,

    // --- g-code templates (with {placeholders}) ---
    pub start_gcode: String,
    pub end_gcode: String,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nozzle_diameter_mm: 0.4,
            filament_diameter_mm: 1.75,
            bed_size_x_mm: 220.0,
            bed_size_y_mm: 220.0,
            layer_height_mm: 0.2,
            first_layer_height_mm: 0.2,
            line_width_mm: 0.45,
            wall_count: 2,
            top_layers: 4,
            bottom_layers: 4,
            infill_density: 0.15,
            skirt_loops: 2,
            skirt_gap_mm: 3.0,
            seam_mode: SeamMode::default(),
            retract_len_mm: 0.8,
            retract_speed_mm_s: 35.0,
            nozzle_temp_c: 200,
            bed_temp_c: 60,
            print_speed_mm_s: 50.0,
            travel_speed_mm_s: 120.0,
            first_layer_speed_mm_s: 20.0,
            start_gcode: GENERIC_START_GCODE.to_string(),
            end_gcode: GENERIC_END_GCODE.to_string(),
        }
    }
}

impl Settings {
    /// Cross-sectional area of the filament (mm²), used for extrusion math.
    pub fn filament_area_mm2(&self) -> f64 {
        let r = self.filament_diameter_mm / 2.0;
        PI * r * r
    }
}
