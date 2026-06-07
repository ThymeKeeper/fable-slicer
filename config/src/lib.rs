//! Settings for the slicer.
//!
//! For now this is a single flat struct with sensible generic-PLA defaults plus a
//! couple of printer presets. The full tiered profile system (printer / filament
//! / process inheritance, TOML via serde) is still upcoming; this already lets
//! `engine` and `gcode` depend on the right crate.

use std::f64::consts::PI;

/// All knobs the pipeline currently needs.
#[derive(Clone, Debug)]
pub struct Settings {
    // --- machine ---
    pub nozzle_diameter_mm: f64,
    pub filament_diameter_mm: f64,
    pub bed_size_x_mm: f64,
    pub bed_size_y_mm: f64,

    // --- process ---
    pub layer_height_mm: f64,
    pub line_width_mm: f64,
    pub wall_count: usize,
    /// Number of fully-solid layers at the top and bottom of the part.
    pub top_layers: usize,
    pub bottom_layers: usize,
    /// Sparse infill density, 0.0..=1.0 (0 disables sparse infill).
    pub infill_density: f64,

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
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nozzle_diameter_mm: 0.4,
            filament_diameter_mm: 1.75,
            bed_size_x_mm: 220.0,
            bed_size_y_mm: 220.0,
            layer_height_mm: 0.2,
            line_width_mm: 0.45,
            wall_count: 2,
            top_layers: 4,
            bottom_layers: 4,
            infill_density: 0.15,
            retract_len_mm: 0.8, // direct-drive default
            retract_speed_mm_s: 35.0,
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

    /// Voron 2.4 preset (Klipper, CoreXY, direct drive).
    ///
    /// NOTE: build size assumed 350mm — the 2.4 also ships as 250/300. Override
    /// with `--bed-x/--bed-y` if yours differs.
    pub fn voron_24() -> Self {
        Self {
            bed_size_x_mm: 350.0,
            bed_size_y_mm: 350.0,
            ..Self::default()
        }
    }

    /// Sovol Zero preset (Klipper, CoreXY, high-speed).
    ///
    /// NOTE: bed size is a placeholder pending confirmation (post-cutoff release).
    /// Override with `--bed-x/--bed-y`.
    pub fn sovol_zero() -> Self {
        Self {
            bed_size_x_mm: 160.0,
            bed_size_y_mm: 160.0,
            ..Self::default()
        }
    }
}
