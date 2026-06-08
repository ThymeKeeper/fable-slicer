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

/// Infill pattern for a region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum InfillPattern {
    /// Parallel lines (rectilinear), alternating direction per layer.
    #[default]
    Lines,
    /// Two perpendicular sets of lines.
    Grid,
    /// Loops following the region boundary inward.
    Concentric,
}

impl InfillPattern {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "lines" | "line" | "rectilinear" => Some(Self::Lines),
            "grid" => Some(Self::Grid),
            "concentric" => Some(Self::Concentric),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Lines => "lines",
            Self::Grid => "grid",
            Self::Concentric => "concentric",
        }
    }
}

/// How overhangs are handled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SupportMode {
    /// No support; overhangs print into air.
    #[default]
    None,
    /// Normal support structure under overhangs (sparse fill, removable).
    Grid,
    /// Support-free: fill flat overhangs with self-supporting concentric arcs
    /// (the "arc overhang" technique); steeper overhangs still get grid support.
    Arc,
}

impl SupportMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "none" | "off" => Some(Self::None),
            "grid" | "normal" | "on" => Some(Self::Grid),
            "arc" | "arcs" | "arc-overhang" => Some(Self::Arc),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Grid => "grid",
            Self::Arc => "arc",
        }
    }
}

/// Fully-resolved settings the pipeline runs on.
#[derive(Clone, Debug)]
pub struct Settings {
    // --- machine ---
    pub nozzle_diameter_mm: f64,
    pub filament_diameter_mm: f64,
    /// Filament density (g/cm³), for the weight estimate.
    pub filament_density_g_cm3: f64,
    pub bed_size_x_mm: f64,
    pub bed_size_y_mm: f64,
    /// Max build height (mm).
    pub bed_size_z_mm: f64,
    /// Acceleration (mm/s²) used for the time estimate.
    pub acceleration_mm_s2: f64,
    /// Junction speed limit (mm/s) used for the time estimate.
    pub jerk_mm_s: f64,

    // --- process ---
    pub layer_height_mm: f64,
    /// Thickness of the first layer (often thicker for bed adhesion).
    pub first_layer_height_mm: f64,
    pub line_width_mm: f64,
    /// Merge contour points whose deviation is below this (mm) before planning —
    /// removes sub-resolution mesh-facet noise. 0 disables.
    pub max_resolution_mm: f64,
    pub wall_count: usize,
    pub top_layers: usize,
    pub bottom_layers: usize,
    /// Sparse infill density, 0.0..=1.0 (0 disables sparse infill).
    pub infill_density: f64,
    /// Pattern for sparse (interior) infill.
    pub sparse_pattern: InfillPattern,
    /// Pattern for solid (top/bottom) interior infill.
    pub solid_pattern: InfillPattern,
    /// Number of skirt loops around the first layer (0 disables).
    pub skirt_loops: usize,
    /// Gap between the skirt and the model (mm).
    pub skirt_gap_mm: f64,
    /// Number of brim loops extending outward from the part (0 disables).
    pub brim_loops: usize,
    /// Where to place the wall seam.
    pub seam_mode: SeamMode,

    // --- supports ---
    /// How overhanging regions are handled.
    pub support_mode: SupportMode,
    /// Max printable overhang measured from vertical (deg); steeper needs support.
    /// 45° ⇒ a region must sit within one layer-height of the layer below.
    pub support_overhang_angle_deg: f64,
    /// Support infill density, 0.0..=1.0.
    pub support_density: f64,
    /// Horizontal gap kept between support and the model (mm).
    pub support_xy_clearance_mm: f64,
    /// Empty layers between a support top and the overhang it holds (removability).
    pub support_z_gap_layers: usize,
    /// Dense interface layers at the support top (smoother overhang underside).
    pub support_interface_layers: usize,
    /// In arc mode, a bridge (supported ≥2 sides) narrower than this (mm) is filled
    /// with straight bridge lines across the gap; wider ones use arcs.
    pub max_bridge_span_mm: f64,
    /// Max arc-overhang radius (mm); a fan that reaches it re-seeds from its
    /// frontier so arcs stay anchored on recently-printed material (McCulloch).
    pub max_arc_radius_mm: f64,

    // --- retraction ---
    pub retract_len_mm: f64,
    pub retract_speed_mm_s: f64,
    /// Z lift on travels that can't be combed (cross a void). 0 disables.
    pub z_hop_mm: f64,

    // --- temperatures (°C) ---
    pub nozzle_temp_c: u32,
    pub bed_temp_c: u32,

    // --- speeds (mm/s) ---
    pub print_speed_mm_s: f64,
    pub travel_speed_mm_s: f64,
    pub first_layer_speed_mm_s: f64,
    /// Speed (mm/s) for bridges and arc overhangs — slow so each bead solidifies.
    pub bridge_speed_mm_s: f64,
    /// Minimum time per layer (s); thin layers are slowed to allow cooling.
    pub min_layer_time_s: f64,
    /// Floor speed (mm/s) when slowing for min-layer-time.
    pub min_print_speed_mm_s: f64,

    // --- g-code templates (with {placeholders}) ---
    pub start_gcode: String,
    pub end_gcode: String,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nozzle_diameter_mm: 0.4,
            filament_diameter_mm: 1.75,
            filament_density_g_cm3: 1.24,
            bed_size_x_mm: 220.0,
            bed_size_y_mm: 220.0,
            bed_size_z_mm: 250.0,
            acceleration_mm_s2: 3000.0,
            jerk_mm_s: 10.0,
            layer_height_mm: 0.2,
            first_layer_height_mm: 0.2,
            line_width_mm: 0.45,
            max_resolution_mm: 0.05,
            wall_count: 2,
            top_layers: 4,
            bottom_layers: 4,
            infill_density: 0.15,
            sparse_pattern: InfillPattern::default(),
            solid_pattern: InfillPattern::default(),
            skirt_loops: 2,
            skirt_gap_mm: 3.0,
            brim_loops: 0,
            seam_mode: SeamMode::default(),
            support_mode: SupportMode::default(),
            support_overhang_angle_deg: 45.0,
            support_density: 0.12,
            support_xy_clearance_mm: 0.4,
            support_z_gap_layers: 1,
            support_interface_layers: 2,
            max_bridge_span_mm: 6.0,
            max_arc_radius_mm: 40.0,
            retract_len_mm: 0.8,
            retract_speed_mm_s: 35.0,
            z_hop_mm: 0.0,
            nozzle_temp_c: 200,
            bed_temp_c: 60,
            print_speed_mm_s: 50.0,
            travel_speed_mm_s: 120.0,
            first_layer_speed_mm_s: 20.0,
            bridge_speed_mm_s: 15.0,
            min_layer_time_s: 8.0,
            min_print_speed_mm_s: 10.0,
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
