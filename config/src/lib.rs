//! Settings for the slicer.
//!
//! For M1 this is a single flat struct with sensible generic-PLA defaults. The
//! M2 work replaces it with the real tiered profile system (printer / filament /
//! process inheritance, TOML via serde). Keeping it here means `engine` and
//! `gcode` already depend on the right crate.

use std::f64::consts::PI;

/// All knobs the M1 pipeline needs.
#[derive(Clone, Debug)]
pub struct Settings {
    // --- machine ---
    pub nozzle_diameter_mm: f64,
    pub filament_diameter_mm: f64,

    // --- process ---
    pub layer_height_mm: f64,
    pub line_width_mm: f64,
    pub wall_count: usize,
    /// Sparse infill density, 0.0..=1.0 (0 disables infill).
    pub infill_density: f64,

    // --- temperatures (°C) ---
    pub nozzle_temp_c: u32,
    pub bed_temp_c: u32,

    // --- speeds (mm/s) ---
    pub print_speed_mm_s: f64,
    pub travel_speed_mm_s: f64,
    pub first_layer_speed_mm_s: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nozzle_diameter_mm: 0.4,
            filament_diameter_mm: 1.75,
            layer_height_mm: 0.2,
            line_width_mm: 0.45,
            wall_count: 2,
            infill_density: 0.15,
            nozzle_temp_c: 200,
            bed_temp_c: 60,
            print_speed_mm_s: 50.0,
            travel_speed_mm_s: 120.0,
            first_layer_speed_mm_s: 20.0,
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
