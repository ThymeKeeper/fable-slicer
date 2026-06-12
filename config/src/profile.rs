//! Tiered profile system: printer / filament / process, each with single-parent
//! inheritance (`inherits = "name"`).
//!
//! Every field is optional; resolving a profile walks its `inherits` chain
//! (child overrides parent), and [`Profiles::resolve`] combines the three tiers
//! into a flat [`Settings`], falling back to `Settings::default()` for anything
//! still unset. Built-in profiles are embedded; extra ones load from a directory.

use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{
    InfillPattern, SeamMode, Settings, SupportMode, WallMode, GENERIC_END_GCODE,
    GENERIC_START_GCODE,
};

/// Printer (machine) tier: bed, extruder, and start/end g-code.
#[derive(Debug, Clone, Default, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct PrinterProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inherits: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bed_size_x_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bed_size_y_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bed_size_z_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nozzle_diameter_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub travel_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub print_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_layer_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acceleration: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outer_wall_accel: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_layer_accel: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jerk: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_len_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_hop_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wipe_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub host_url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub api_key: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heat_rate_c_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cool_rate_c_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heat_rate_fan_c_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cool_rate_fan_c_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aux_fan: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exhaust_fan: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_gcode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_gcode: Option<String>,
}

/// Filament (material) tier: diameter, temperatures, flow, and cooling.
#[derive(Debug, Clone, Default, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct FilamentProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inherits: Option<String>,
    /// Material class off the box ("pla", "petg", "abs", "tpu", "other") —
    /// drives every default below until a calibration value pins one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub material: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament_diameter_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density_g_cm3: Option<f64>,
    /// The packaging temperature range printed on the spool.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nozzle_temp_min_c: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nozzle_temp_max_c: Option<u32>,
    /// Cold ↔ hot preference (−1..+1) inside the packaging range.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temp_bias: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bed_temp_c: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extrusion_multiplier: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_volumetric_speed_mm3_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pressure_advance: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fan_speed: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_fan_speed: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_flow_derate_per_c: Option<f64>,
    /// Allowable heat-load ceiling (mW/mm², per island) — a material range
    /// bound heat control works within, not a tuning target. Auto: 15.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_heat_mw_mm2: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fan_off_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aux_fan_speed: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exhaust_fan_speed: Option<f64>,
}

/// Process (print) tier: quality/geometry knobs.
#[derive(Debug, Clone, Default, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct ProcessProfile {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inherits: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub layer_height_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_layer_height_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_resolution_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_fitting: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_tolerance_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_count: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_mode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottom_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub half_height_outer_walls: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub brick_layers: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub brick_flow: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub infill_density: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sparse_infill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid_infill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skirt_loops: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skirt_gap_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub brim_loops: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seam: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_overhang_angle_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_density: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_xy_clearance_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_z_gap_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_interface_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_bridge_span_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_arc_radius_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_seam_overlap_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub infill_overlap: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monotonic_solid: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gap_fill: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuzzy_skin: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuzzy_skin_thickness_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuzzy_skin_point_dist_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ironing: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elephant_foot_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub xy_compensation_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spiral_vase: Option<bool>,
    /// Finish ↔ speed preference (−1..+1) — the one speed control. Scales
    /// the derived speeds between 60% and 100% of the machine's rating.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_quality: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heat_control: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smooth_extra_time_pct: Option<f64>,
}

/// One inheritable tier: knows its parent and how to layer over a base.
/// Public so front-ends can merge a fresh diff over an existing user profile
/// when overwriting it (diff wins, stored fields fill the gaps).
pub trait Tier: Clone {
    fn parent(&self) -> Option<&str>;
    /// Combine `self` (child) over `base` (resolved parent); child wins.
    fn over(self, base: Self) -> Self;
}

/// `$child.or($base)` for each listed field — child values win.
macro_rules! merge_fields {
    ($child:expr, $base:expr, $($f:ident),+ $(,)?) => {
        Self { inherits: None, $($f: $child.$f.or($base.$f)),+ }
    };
}

impl Tier for PrinterProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, bed_size_x_mm, bed_size_y_mm, bed_size_z_mm, nozzle_diameter_mm,
            travel_speed_mm_s, print_speed_mm_s, first_layer_speed_mm_s, acceleration,
            outer_wall_accel, first_layer_accel, jerk,
            retract_len_mm, retract_speed_mm_s, z_hop_mm, wipe_mm, host_url, api_key,
            heat_rate_c_s, cool_rate_c_s, heat_rate_fan_c_s, cool_rate_fan_c_s,
            aux_fan, exhaust_fan, start_gcode, end_gcode)
    }
}

impl Tier for FilamentProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, material, filament_diameter_mm, density_g_cm3, temp_bias,
            nozzle_temp_min_c, nozzle_temp_max_c, bed_temp_c,
            extrusion_multiplier, max_volumetric_speed_mm3_s, max_flow_derate_per_c,
            max_heat_mw_mm2, pressure_advance,
            fan_speed, bridge_fan_speed, fan_off_layers, aux_fan_speed, exhaust_fan_speed)
    }
}

impl Tier for ProcessProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, layer_height_mm, first_layer_height_mm,
            max_resolution_mm, arc_fitting, arc_tolerance_mm, wall_count, wall_mode, top_layers, bottom_layers,
            half_height_outer_walls, brick_layers, brick_flow,
            infill_density, sparse_infill, solid_infill,
            skirt_loops, skirt_gap_mm, brim_loops, seam, support, support_overhang_angle_deg,
            support_density, support_xy_clearance_mm, support_z_gap_layers, support_interface_layers,
            max_bridge_span_mm, max_arc_radius_mm, arc_seam_overlap_mm,
            infill_overlap, monotonic_solid, gap_fill,
            fuzzy_skin, fuzzy_skin_thickness_mm, fuzzy_skin_point_dist_mm,
            ironing,
            elephant_foot_mm, xy_compensation_mm, spiral_vase,
            speed_quality, heat_control, smooth_extra_time_pct)
    }
}

/// `Some(current)` where it differs from the baseline, else `None`.
macro_rules! diff_field {
    ($cur:expr, $base:expr) => {
        if $cur != $base {
            Some($cur)
        } else {
            None
        }
    };
}

impl PrinterProfile {
    /// The printer-tier fields where `cur` differs from `base`. Print, travel,
    /// and first-layer speed live here: the printer tier wins those in
    /// `resolve`, so a process-tier copy would be dead on machines that set them.
    pub fn diff(cur: &Settings, base: &Settings) -> Self {
        Self {
            inherits: None,
            bed_size_x_mm: diff_field!(cur.bed_size_x_mm, base.bed_size_x_mm),
            bed_size_y_mm: diff_field!(cur.bed_size_y_mm, base.bed_size_y_mm),
            bed_size_z_mm: diff_field!(cur.bed_size_z_mm, base.bed_size_z_mm),
            nozzle_diameter_mm: diff_field!(cur.nozzle_diameter_mm, base.nozzle_diameter_mm),
            travel_speed_mm_s: diff_field!(cur.travel_speed_mm_s, base.travel_speed_mm_s),
            print_speed_mm_s: diff_field!(cur.machine_speed_mm_s, base.machine_speed_mm_s),
            first_layer_speed_mm_s: diff_field!(cur.first_layer_speed_mm_s, base.first_layer_speed_mm_s),
            acceleration: diff_field!(cur.acceleration_mm_s2, base.acceleration_mm_s2),
            outer_wall_accel: diff_field!(cur.outer_wall_accel_mm_s2, base.outer_wall_accel_mm_s2),
            first_layer_accel: diff_field!(cur.first_layer_accel_mm_s2, base.first_layer_accel_mm_s2),
            jerk: diff_field!(cur.jerk_mm_s, base.jerk_mm_s),
            retract_len_mm: diff_field!(cur.retract_len_mm, base.retract_len_mm),
            retract_speed_mm_s: diff_field!(cur.retract_speed_mm_s, base.retract_speed_mm_s),
            z_hop_mm: diff_field!(cur.z_hop_mm, base.z_hop_mm),
            wipe_mm: diff_field!(cur.wipe_mm, base.wipe_mm),
            host_url: diff_field!(cur.host_url.clone(), base.host_url),
            api_key: diff_field!(cur.api_key.clone(), base.api_key),
            heat_rate_c_s: diff_field!(cur.heat_rate_c_s, base.heat_rate_c_s),
            cool_rate_c_s: diff_field!(cur.cool_rate_c_s, base.cool_rate_c_s),
            heat_rate_fan_c_s: diff_field!(cur.heat_rate_fan_c_s, base.heat_rate_fan_c_s),
            cool_rate_fan_c_s: diff_field!(cur.cool_rate_fan_c_s, base.cool_rate_fan_c_s),
            aux_fan: diff_field!(cur.has_aux_fan, base.has_aux_fan),
            exhaust_fan: diff_field!(cur.has_exhaust_fan, base.has_exhaust_fan),
            start_gcode: diff_field!(cur.start_gcode.clone(), base.start_gcode),
            end_gcode: diff_field!(cur.end_gcode.clone(), base.end_gcode),
        }
    }

    /// True if no field is set (nothing worth saving).
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

impl FilamentProfile {
    /// The filament-tier fields where `cur` differs from `base`.
    pub fn diff(cur: &Settings, base: &Settings) -> Self {
        Self {
            inherits: None,
            filament_diameter_mm: diff_field!(cur.filament_diameter_mm, base.filament_diameter_mm),
            density_g_cm3: diff_field!(cur.filament_density_g_cm3, base.filament_density_g_cm3),
            material: diff_field!(cur.material.label().to_string(), base.material.label().to_string()),
            temp_bias: diff_field!(cur.temp_bias, base.temp_bias),
            nozzle_temp_min_c: diff_field!(cur.nozzle_temp_min_c, base.nozzle_temp_min_c),
            nozzle_temp_max_c: diff_field!(cur.nozzle_temp_max_c, base.nozzle_temp_max_c),
            bed_temp_c: diff_field!(cur.bed_temp_c, base.bed_temp_c),
            extrusion_multiplier: diff_field!(cur.extrusion_multiplier, base.extrusion_multiplier),
            max_volumetric_speed_mm3_s: diff_field!(cur.max_volumetric_speed_mm3_s, base.max_volumetric_speed_mm3_s),
            max_flow_derate_per_c: diff_field!(cur.max_flow_derate_per_c, base.max_flow_derate_per_c),
            max_heat_mw_mm2: diff_field!(cur.max_heat_mw_mm2, base.max_heat_mw_mm2),
            pressure_advance: diff_field!(cur.pressure_advance, base.pressure_advance),
            fan_speed: diff_field!(cur.fan_speed, base.fan_speed),
            bridge_fan_speed: diff_field!(cur.bridge_fan_speed, base.bridge_fan_speed),
            fan_off_layers: diff_field!(cur.fan_off_layers, base.fan_off_layers),
            aux_fan_speed: diff_field!(cur.aux_fan_speed, base.aux_fan_speed),
            exhaust_fan_speed: diff_field!(cur.exhaust_fan_speed, base.exhaust_fan_speed),
        }
    }

    /// True if no field is set (nothing worth saving).
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

impl ProcessProfile {
    /// The process-tier fields where `cur` differs from `base`.
    pub fn diff(cur: &Settings, base: &Settings) -> Self {
        Self {
            inherits: None,
            layer_height_mm: diff_field!(cur.layer_height_mm, base.layer_height_mm),
            first_layer_height_mm: diff_field!(cur.first_layer_height_mm, base.first_layer_height_mm),
            max_resolution_mm: diff_field!(cur.max_resolution_mm, base.max_resolution_mm),
            arc_fitting: diff_field!(cur.arc_fitting, base.arc_fitting),
            arc_tolerance_mm: diff_field!(cur.arc_tolerance_mm, base.arc_tolerance_mm),
            wall_count: diff_field!(cur.wall_count, base.wall_count),
            wall_mode: diff_field!(cur.wall_mode, base.wall_mode).map(|m| m.label().to_string()),
            top_layers: diff_field!(cur.top_layers, base.top_layers),
            bottom_layers: diff_field!(cur.bottom_layers, base.bottom_layers),
            half_height_outer_walls: diff_field!(cur.half_height_outer_walls, base.half_height_outer_walls),
            brick_layers: diff_field!(cur.brick_layers, base.brick_layers),
            brick_flow: diff_field!(cur.brick_flow, base.brick_flow),
            infill_density: diff_field!(cur.infill_density, base.infill_density),
            sparse_infill: diff_field!(cur.sparse_pattern, base.sparse_pattern).map(|p| p.label().to_string()),
            solid_infill: diff_field!(cur.solid_pattern, base.solid_pattern).map(|p| p.label().to_string()),
            skirt_loops: diff_field!(cur.skirt_loops, base.skirt_loops),
            skirt_gap_mm: diff_field!(cur.skirt_gap_mm, base.skirt_gap_mm),
            brim_loops: diff_field!(cur.brim_loops, base.brim_loops),
            seam: diff_field!(cur.seam_mode, base.seam_mode).map(|m| m.label().to_string()),
            support: diff_field!(cur.support_mode, base.support_mode).map(|m| m.label().to_string()),
            support_overhang_angle_deg: diff_field!(cur.support_overhang_angle_deg, base.support_overhang_angle_deg),
            support_density: diff_field!(cur.support_density, base.support_density),
            support_xy_clearance_mm: diff_field!(cur.support_xy_clearance_mm, base.support_xy_clearance_mm),
            support_z_gap_layers: diff_field!(cur.support_z_gap_layers, base.support_z_gap_layers),
            support_interface_layers: diff_field!(cur.support_interface_layers, base.support_interface_layers),
            max_bridge_span_mm: diff_field!(cur.max_bridge_span_mm, base.max_bridge_span_mm),
            max_arc_radius_mm: diff_field!(cur.max_arc_radius_mm, base.max_arc_radius_mm),
            arc_seam_overlap_mm: diff_field!(cur.arc_seam_overlap_mm, base.arc_seam_overlap_mm),
            // print/first-layer speed are printer-tier (see PrinterProfile::diff).
            infill_overlap: diff_field!(cur.infill_overlap, base.infill_overlap),
            monotonic_solid: diff_field!(cur.monotonic_solid, base.monotonic_solid),
            gap_fill: diff_field!(cur.gap_fill, base.gap_fill),
            fuzzy_skin: diff_field!(cur.fuzzy_skin, base.fuzzy_skin),
            fuzzy_skin_thickness_mm: diff_field!(cur.fuzzy_skin_thickness_mm, base.fuzzy_skin_thickness_mm),
            fuzzy_skin_point_dist_mm: diff_field!(cur.fuzzy_skin_point_dist_mm, base.fuzzy_skin_point_dist_mm),
            ironing: diff_field!(cur.ironing, base.ironing),
            elephant_foot_mm: diff_field!(cur.elephant_foot_mm, base.elephant_foot_mm),
            xy_compensation_mm: diff_field!(cur.xy_compensation_mm, base.xy_compensation_mm),
            spiral_vase: diff_field!(cur.spiral_vase, base.spiral_vase),
            speed_quality: diff_field!(cur.speed_quality, base.speed_quality),
            heat_control: diff_field!(cur.heat_control, base.heat_control),
            smooth_extra_time_pct: diff_field!(cur.smooth_extra_time_pct, base.smooth_extra_time_pct),
        }
    }

    /// True if no field is set (nothing worth saving).
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// Per-tier "modified" flags: does `cur` differ from `base` in any
/// (printer, filament, process) field? Drives the GUI's dirty indicators.
pub fn tier_dirty(cur: &Settings, base: &Settings) -> [bool; 3] {
    [
        !PrinterProfile::diff(cur, base).is_empty(),
        !FilamentProfile::diff(cur, base).is_empty(),
        !ProcessProfile::diff(cur, base).is_empty(),
    ]
}

/// Which profile tier a name belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TierKind {
    Printer,
    Filament,
    Process,
}

impl TierKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Printer => "printer",
            Self::Filament => "filament",
            Self::Process => "process",
        }
    }
}

/// A registry of named profiles for each tier. Built-ins are embedded and
/// read-only; user profiles load from (and save to) a config directory.
#[derive(Default)]
pub struct Profiles {
    printers: HashMap<String, PrinterProfile>,
    filaments: HashMap<String, FilamentProfile>,
    processes: HashMap<String, ProcessProfile>,
    /// Names that came from built-ins (protected from overwrite/delete).
    builtin: HashSet<(&'static str, String)>,
    /// Where user profiles live (set by `load_user_profiles`); saves go here.
    user_dir: Option<std::path::PathBuf>,
}

impl Profiles {
    /// The profiles embedded in the binary.
    pub fn builtin() -> Self {
        fn parse<T: for<'de> Deserialize<'de>>(name: &str, text: &str) -> T {
            toml::from_str(text).unwrap_or_else(|e| panic!("built-in profile {name}: {e}"))
        }
        let mut p = Profiles::default();
        p.printers.insert("generic".into(), parse("printer/generic", include_str!("../profiles/printer/generic.toml")));
        p.printers.insert("voron24".into(), parse("printer/voron24", include_str!("../profiles/printer/voron24.toml")));
        p.printers.insert("sovol-zero".into(), parse("printer/sovol_zero", include_str!("../profiles/printer/sovol_zero.toml")));
        p.filaments.insert("pla".into(), parse("filament/pla", include_str!("../profiles/filament/pla.toml")));
        p.filaments.insert("petg".into(), parse("filament/petg", include_str!("../profiles/filament/petg.toml")));
        p.filaments.insert("pla-hf".into(), parse("filament/pla_hf", include_str!("../profiles/filament/pla_hf.toml")));
        p.processes.insert("standard".into(), parse("process/standard", include_str!("../profiles/process/standard.toml")));
        p.processes.insert("fine".into(), parse("process/fine", include_str!("../profiles/process/fine.toml")));
        p.processes.insert("draft".into(), parse("process/draft", include_str!("../profiles/process/draft.toml")));
        for name in p.printers.keys() {
            p.builtin.insert(("printer", name.clone()));
        }
        for name in p.filaments.keys() {
            p.builtin.insert(("filament", name.clone()));
        }
        for name in p.processes.keys() {
            p.builtin.insert(("process", name.clone()));
        }
        p
    }

    /// Load extra profiles from `<dir>/{printer,filament,process}/*.toml`,
    /// overriding built-ins with the same file stem (explicit power feature —
    /// the auto-loaded user dir does *not* shadow; see `load_user_profiles`).
    pub fn load_dir(&mut self, dir: &Path) -> Result<(), String> {
        load_tier(&mut self.printers, &dir.join("printer"), None)?;
        load_tier(&mut self.filaments, &dir.join("filament"), None)?;
        load_tier(&mut self.processes, &dir.join("process"), None)?;
        Ok(())
    }

    /// The platform's per-user profile directory (`<config>/slicer/profiles`).
    pub fn default_user_dir() -> Option<std::path::PathBuf> {
        let base = if cfg!(target_os = "windows") {
            std::env::var_os("APPDATA").map(std::path::PathBuf::from)
        } else if cfg!(target_os = "macos") {
            std::env::var_os("HOME")
                .map(|h| std::path::PathBuf::from(h).join("Library/Application Support"))
        } else {
            std::env::var_os("XDG_CONFIG_HOME")
                .map(std::path::PathBuf::from)
                .or_else(|| std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config")))
        };
        base.map(|b| b.join("slicer").join("profiles"))
    }

    /// Load user profiles from `dir` (or the platform default) and remember it
    /// as the save destination. Missing directories are fine (nothing saved yet).
    ///
    /// Unlike `load_dir`, files whose stem collides with a built-in profile are
    /// **skipped** (returned for the caller to warn about) — built-ins stay
    /// read-only; base a user profile on one via `inherits` instead.
    pub fn load_user_profiles(&mut self, dir: Option<std::path::PathBuf>) -> Result<Vec<String>, String> {
        let Some(dir) = dir.or_else(Self::default_user_dir) else {
            return Err("no user config directory available".into());
        };
        let mut skipped = Vec::new();
        skipped.extend(load_tier(&mut self.printers, &dir.join("printer"), Some((&self.builtin, "printer")))?);
        skipped.extend(load_tier(&mut self.filaments, &dir.join("filament"), Some((&self.builtin, "filament")))?);
        skipped.extend(load_tier(&mut self.processes, &dir.join("process"), Some((&self.builtin, "process")))?);
        self.user_dir = Some(dir);
        Ok(skipped)
    }

    /// Where user profiles are saved, if a user dir has been established.
    pub fn user_dir(&self) -> Option<&Path> {
        self.user_dir.as_deref()
    }

    /// True if `name` is one of the embedded (read-only) profiles.
    pub fn is_builtin(&self, kind: TierKind, name: &str) -> bool {
        self.builtin.contains(&(kind.label(), name.to_string()))
    }

    /// True if `name` exists and is editable (loaded from / saved to the user dir).
    pub fn is_user(&self, kind: TierKind, name: &str) -> bool {
        let exists = match kind {
            TierKind::Printer => self.printers.contains_key(name),
            TierKind::Filament => self.filaments.contains_key(name),
            TierKind::Process => self.processes.contains_key(name),
        };
        exists && !self.is_builtin(kind, name)
    }

    /// The fully-merged (inherits-resolved) profile of one tier — lets the GUI
    /// see which fields the profile chain actually pins vs. leaves on auto.
    pub fn merged_printer(&self, name: &str) -> Result<PrinterProfile, String> {
        resolve_tier(&self.printers, name, "printer")
    }
    pub fn merged_filament(&self, name: &str) -> Result<FilamentProfile, String> {
        resolve_tier(&self.filaments, name, "filament")
    }
    pub fn merged_process(&self, name: &str) -> Result<ProcessProfile, String> {
        resolve_tier(&self.processes, name, "process")
    }

    pub fn get_printer(&self, name: &str) -> Option<&PrinterProfile> {
        self.printers.get(name)
    }
    pub fn get_filament(&self, name: &str) -> Option<&FilamentProfile> {
        self.filaments.get(name)
    }
    pub fn get_process(&self, name: &str) -> Option<&ProcessProfile> {
        self.processes.get(name)
    }

    /// Validate a profile name for saving: filesystem-safe and not a built-in.
    fn check_save_name(&self, kind: TierKind, name: &str) -> Result<(), String> {
        if name.is_empty()
            || !name.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.'))
        {
            return Err("name must be non-empty and use only letters, digits, '-', '_', '.'".into());
        }
        if self.is_builtin(kind, name) {
            return Err(format!("'{name}' is a built-in {} profile — pick another name", kind.label()));
        }
        Ok(())
    }

    fn save_toml(&self, kind: TierKind, name: &str, text: String) -> Result<(), String> {
        let dir = self
            .user_dir
            .as_ref()
            .ok_or("no user profile directory (call load_user_profiles first)")?
            .join(kind.label());
        fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let path = dir.join(format!("{name}.toml"));
        fs::write(&path, text).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Save (or overwrite) a user printer profile and register it.
    pub fn save_user_printer(&mut self, name: &str, p: PrinterProfile) -> Result<(), String> {
        self.check_save_name(TierKind::Printer, name)?;
        let text = toml::to_string_pretty(&p).map_err(|e| e.to_string())?;
        self.save_toml(TierKind::Printer, name, text)?;
        self.printers.insert(name.to_string(), p);
        Ok(())
    }

    /// Save (or overwrite) a user filament profile and register it.
    pub fn save_user_filament(&mut self, name: &str, p: FilamentProfile) -> Result<(), String> {
        self.check_save_name(TierKind::Filament, name)?;
        let text = toml::to_string_pretty(&p).map_err(|e| e.to_string())?;
        self.save_toml(TierKind::Filament, name, text)?;
        self.filaments.insert(name.to_string(), p);
        Ok(())
    }

    /// Save (or overwrite) a user process profile and register it.
    pub fn save_user_process(&mut self, name: &str, p: ProcessProfile) -> Result<(), String> {
        self.check_save_name(TierKind::Process, name)?;
        let text = toml::to_string_pretty(&p).map_err(|e| e.to_string())?;
        self.save_toml(TierKind::Process, name, text)?;
        self.processes.insert(name.to_string(), p);
        Ok(())
    }

    /// Delete a user profile (file + registry entry). Built-ins are refused.
    pub fn delete_user(&mut self, kind: TierKind, name: &str) -> Result<(), String> {
        if !self.is_user(kind, name) {
            return Err(format!("'{name}' is not a user {} profile", kind.label()));
        }
        if let Some(dir) = &self.user_dir {
            let path = dir.join(kind.label()).join(format!("{name}.toml"));
            if path.exists() {
                fs::remove_file(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            }
        }
        match kind {
            TierKind::Printer => self.printers.remove(name).map(|_| ()),
            TierKind::Filament => self.filaments.remove(name).map(|_| ()),
            TierKind::Process => self.processes.remove(name).map(|_| ()),
        };
        Ok(())
    }

    pub fn printer_names(&self) -> Vec<&str> {
        sorted_names(&self.printers)
    }
    pub fn filament_names(&self) -> Vec<&str> {
        sorted_names(&self.filaments)
    }
    pub fn process_names(&self) -> Vec<&str> {
        sorted_names(&self.processes)
    }

    /// Resolve the three named profiles into flat [`Settings`].
    pub fn resolve(&self, printer: &str, filament: &str, process: &str) -> Result<Settings, String> {
        let pr = resolve_tier(&self.printers, printer, "printer")?;
        let fl = resolve_tier(&self.filaments, filament, "filament")?;
        let pc = resolve_tier(&self.processes, process, "process")?;
        let d = Settings::default();
        // The material class off the box drives every filament default a
        // calibration entry doesn't pin.
        let material = fl.material.as_deref().and_then(crate::Material::parse).unwrap_or(d.material);
        // Packaging range + cold↔hot bias → the operating temperatures.
        let (class_min, class_max) = material.packaging_temp_c();
        let temp_min = fl.nozzle_temp_min_c.unwrap_or(class_min);
        let temp_max = fl.nozzle_temp_max_c.unwrap_or(class_max);
        let bias = fl.temp_bias.unwrap_or(d.temp_bias);
        let nozzle_temp = crate::derived_nozzle_temp_c(temp_min, temp_max, bias);
        // The machine's rating × the finish↔speed dial → the nominal speed.
        let machine_v = pr.print_speed_mm_s.unwrap_or(d.machine_speed_mm_s);
        let quality = pc.speed_quality.unwrap_or(d.speed_quality);
        let print_v = crate::derived_print_speed_mm_s(machine_v, quality);
        let nozzle = pr.nozzle_diameter_mm.unwrap_or(d.nozzle_diameter_mm);
        // The flow triangle: speed × bead area (line width × layer height) must
        // fit the filament's melt ceiling, so derived speeds balance against it.
        let line_w = crate::derived_line_width_mm(nozzle);
        let layer_h = pc.layer_height_mm.unwrap_or(d.layer_height_mm);
        let max_flow = fl.max_volumetric_speed_mm3_s.unwrap_or_else(|| material.max_flow_mm3_s());
        let flow_cap = crate::flow_speed_cap_mm_s(max_flow, line_w, layer_h);
        Ok(Settings {
            nozzle_diameter_mm: nozzle,
            filament_diameter_mm: fl.filament_diameter_mm.unwrap_or(d.filament_diameter_mm),
            filament_density_g_cm3: fl.density_g_cm3.unwrap_or_else(|| material.density_g_cm3()),
            bed_size_x_mm: pr.bed_size_x_mm.unwrap_or(d.bed_size_x_mm),
            bed_size_y_mm: pr.bed_size_y_mm.unwrap_or(d.bed_size_y_mm),
            bed_size_z_mm: pr.bed_size_z_mm.unwrap_or(d.bed_size_z_mm),
            acceleration_mm_s2: pr.acceleration.unwrap_or(d.acceleration_mm_s2),
            outer_wall_accel_mm_s2: pr.outer_wall_accel.unwrap_or_else(|| {
                crate::derived_outer_wall_accel_mm_s2(pr.acceleration.unwrap_or(d.acceleration_mm_s2))
            }),
            first_layer_accel_mm_s2: pr.first_layer_accel.unwrap_or_else(|| {
                crate::derived_first_layer_accel_mm_s2(pr.acceleration.unwrap_or(d.acceleration_mm_s2))
            }),
            jerk_mm_s: pr.jerk.unwrap_or(d.jerk_mm_s),
            layer_height_mm: layer_h,
            first_layer_height_mm: pc.first_layer_height_mm.unwrap_or(d.first_layer_height_mm),
            line_width_mm: line_w,
            max_resolution_mm: pc.max_resolution_mm.unwrap_or(d.max_resolution_mm),
            arc_fitting: pc.arc_fitting.unwrap_or(d.arc_fitting),
            arc_tolerance_mm: pc.arc_tolerance_mm.unwrap_or(d.arc_tolerance_mm),
            wall_count: pc.wall_count.unwrap_or(d.wall_count),
            wall_mode: pc.wall_mode.as_deref().and_then(WallMode::parse).unwrap_or(d.wall_mode),
            top_layers: pc.top_layers.unwrap_or(d.top_layers),
            bottom_layers: pc.bottom_layers.unwrap_or(d.bottom_layers),
            half_height_outer_walls: pc
                .half_height_outer_walls
                .unwrap_or(d.half_height_outer_walls),
            brick_layers: pc.brick_layers.unwrap_or(d.brick_layers),
            brick_flow: pc.brick_flow.unwrap_or(d.brick_flow),
            infill_density: pc.infill_density.unwrap_or(d.infill_density),
            sparse_pattern: pc.sparse_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.sparse_pattern),
            solid_pattern: pc.solid_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.solid_pattern),
            infill_overlap: pc.infill_overlap.unwrap_or(d.infill_overlap),
            monotonic_solid: pc.monotonic_solid.unwrap_or(d.monotonic_solid),
            gap_fill: pc.gap_fill.unwrap_or(d.gap_fill),
            fuzzy_skin: pc.fuzzy_skin.unwrap_or(d.fuzzy_skin),
            fuzzy_skin_thickness_mm: pc.fuzzy_skin_thickness_mm.unwrap_or(d.fuzzy_skin_thickness_mm),
            fuzzy_skin_point_dist_mm: pc.fuzzy_skin_point_dist_mm.unwrap_or(d.fuzzy_skin_point_dist_mm),
            ironing: pc.ironing.unwrap_or(d.ironing),
            ironing_flow: d.ironing_flow,
            ironing_spacing_mm: d.ironing_spacing_mm,
            ironing_speed_mm_s: d.ironing_speed_mm_s,
            elephant_foot_mm: pc.elephant_foot_mm.unwrap_or(d.elephant_foot_mm),
            xy_compensation_mm: pc.xy_compensation_mm.unwrap_or(d.xy_compensation_mm),
            spiral_vase: pc.spiral_vase.unwrap_or(d.spiral_vase),
            skirt_loops: pc.skirt_loops.unwrap_or(d.skirt_loops),
            skirt_gap_mm: pc.skirt_gap_mm.unwrap_or(d.skirt_gap_mm),
            brim_loops: pc.brim_loops.unwrap_or(d.brim_loops),
            seam_mode: pc.seam.as_deref().and_then(SeamMode::parse).unwrap_or(d.seam_mode),
            auto_center_on_bed: d.auto_center_on_bed, // runtime flag, not profile-driven
            support_mode: pc.support.as_deref().and_then(SupportMode::parse).unwrap_or(d.support_mode),
            support_overhang_angle_deg: pc
                .support_overhang_angle_deg
                .unwrap_or(d.support_overhang_angle_deg),
            support_density: pc.support_density.unwrap_or(d.support_density),
            support_xy_clearance_mm: pc.support_xy_clearance_mm.unwrap_or(d.support_xy_clearance_mm),
            support_z_gap_layers: pc.support_z_gap_layers.unwrap_or(d.support_z_gap_layers),
            support_interface_layers: pc.support_interface_layers.unwrap_or(d.support_interface_layers),
            max_bridge_span_mm: pc.max_bridge_span_mm.unwrap_or(d.max_bridge_span_mm),
            max_arc_radius_mm: pc.max_arc_radius_mm.unwrap_or(d.max_arc_radius_mm),
            arc_seam_overlap_mm: pc.arc_seam_overlap_mm.unwrap_or(d.arc_seam_overlap_mm),
            retract_len_mm: pr.retract_len_mm.unwrap_or(d.retract_len_mm),
            retract_speed_mm_s: pr.retract_speed_mm_s.unwrap_or(d.retract_speed_mm_s),
            z_hop_mm: pr.z_hop_mm.unwrap_or(d.z_hop_mm),
            wipe_mm: pr.wipe_mm.unwrap_or(d.wipe_mm),
            host_url: pr.host_url.unwrap_or(d.host_url),
            api_key: pr.api_key.unwrap_or(d.api_key),
            material,
            nozzle_temp_c: nozzle_temp,
            first_layer_nozzle_temp_c: crate::derived_first_layer_temp_c(temp_min, temp_max, bias, material),
            nozzle_temp_min_c: temp_min,
            nozzle_temp_max_c: temp_max,
            temp_bias: bias,
            bed_temp_c: fl.bed_temp_c.unwrap_or_else(|| material.bed_temp_c()),
            heat_rate_c_s: pr.heat_rate_c_s.unwrap_or(d.heat_rate_c_s),
            cool_rate_c_s: pr.cool_rate_c_s.unwrap_or(d.cool_rate_c_s),
            // Auto: un-measured fan-on rates follow the fan-off ones.
            heat_rate_fan_c_s: pr
                .heat_rate_fan_c_s
                .unwrap_or_else(|| pr.heat_rate_c_s.unwrap_or(d.heat_rate_c_s)),
            cool_rate_fan_c_s: pr
                .cool_rate_fan_c_s
                .unwrap_or_else(|| pr.cool_rate_c_s.unwrap_or(d.cool_rate_c_s)),
            machine_speed_mm_s: machine_v,
            speed_quality: quality,
            print_speed_mm_s: print_v,
            travel_speed_mm_s: pr.travel_speed_mm_s.unwrap_or(d.travel_speed_mm_s),
            first_layer_speed_mm_s: pr.first_layer_speed_mm_s.unwrap_or(d.first_layer_speed_mm_s),
            // Every feature speed derives: nominal × its quality ratio, under
            // the filament's flow ceiling. Heat control governs from there.
            external_perimeter_speed_mm_s: crate::derived_external_perimeter_speed_mm_s(print_v, flow_cap),
            solid_speed_mm_s: crate::derived_solid_speed_mm_s(print_v, flow_cap),
            support_speed_mm_s: crate::derived_support_speed_mm_s(print_v, flow_cap),
            gap_fill_speed_mm_s: crate::derived_gap_fill_speed_mm_s(print_v, flow_cap),
            bridge_speed_mm_s: d.bridge_speed_mm_s,
            overhang_speed_mm_s: crate::derived_overhang_speed_mm_s(d.bridge_speed_mm_s),
            min_layer_time_s: d.min_layer_time_s,
            min_print_speed_mm_s: d.min_print_speed_mm_s,
            max_volumetric_speed_mm3_s: max_flow,
            max_flow_derate_per_c: fl.max_flow_derate_per_c.unwrap_or_else(|| material.max_flow_derate_per_c()),
            extrusion_multiplier: fl.extrusion_multiplier.unwrap_or(d.extrusion_multiplier),
            bridge_flow: d.bridge_flow,
            heat_control: pc.heat_control.unwrap_or(d.heat_control),
            // Material ranges live with the filament; the class supplies
            // them until calibration pins one.
            max_heat_mw_mm2: fl.max_heat_mw_mm2.unwrap_or_else(|| material.max_heat_mw_mm2()),
            smooth_extra_time_pct: pc.smooth_extra_time_pct.unwrap_or(d.smooth_extra_time_pct),
            pressure_advance: fl.pressure_advance.unwrap_or(d.pressure_advance),
            fan_speed: fl.fan_speed.unwrap_or_else(|| material.fan().0),
            bridge_fan_speed: fl.bridge_fan_speed.unwrap_or_else(|| material.fan().1),
            fan_off_layers: fl.fan_off_layers.unwrap_or_else(|| material.fan().2),
            has_aux_fan: pr.aux_fan.unwrap_or(d.has_aux_fan),
            has_exhaust_fan: pr.exhaust_fan.unwrap_or(d.has_exhaust_fan),
            aux_fan_speed: fl.aux_fan_speed.unwrap_or_else(|| material.aux_exhaust().0),
            exhaust_fan_speed: fl.exhaust_fan_speed.unwrap_or_else(|| material.aux_exhaust().1),
            start_gcode: pr.start_gcode.unwrap_or_else(|| GENERIC_START_GCODE.to_string()),
            end_gcode: pr.end_gcode.unwrap_or_else(|| GENERIC_END_GCODE.to_string()),
        })
    }
}

fn sorted_names<T>(map: &HashMap<String, T>) -> Vec<&str> {
    let mut v: Vec<&str> = map.keys().map(String::as_str).collect();
    v.sort_unstable();
    v
}

/// Load one tier's directory into `map`. With `skip_builtin = Some((set, label))`,
/// files whose stem matches a built-in name are skipped and returned as
/// `"label/stem"` so the caller can warn.
fn load_tier<T: for<'de> Deserialize<'de>>(
    map: &mut HashMap<String, T>,
    dir: &Path,
    skip_builtin: Option<(&HashSet<(&'static str, String)>, &'static str)>,
) -> Result<Vec<String>, String> {
    let mut skipped = Vec::new();
    if !dir.is_dir() {
        return Ok(skipped);
    }
    for entry in fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
        let path = entry.map_err(|e| e.to_string())?.path();
        if path.extension().and_then(|s| s.to_str()) != Some("toml") {
            continue;
        }
        let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
        if let Some((builtin, label)) = skip_builtin {
            if builtin.contains(&(label, stem.clone())) {
                skipped.push(format!("{label}/{stem}"));
                continue;
            }
        }
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let profile: T = toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        map.insert(stem, profile);
    }
    Ok(skipped)
}

/// Resolve a profile's `inherits` chain into a single merged profile.
fn resolve_tier<T: Tier>(map: &HashMap<String, T>, name: &str, kind: &str) -> Result<T, String> {
    fn inner<T: Tier>(map: &HashMap<String, T>, name: &str, kind: &str, seen: &mut HashSet<String>) -> Result<T, String> {
        if !seen.insert(name.to_string()) {
            return Err(format!("{kind} profile inheritance cycle at '{name}'"));
        }
        let profile = map
            .get(name)
            .cloned()
            .ok_or_else(|| format!("unknown {kind} profile '{name}'"))?;
        match profile.parent() {
            Some(parent) => {
                let base = inner(map, &parent.to_string(), kind, seen)?;
                Ok(profile.over(base))
            }
            None => Ok(profile),
        }
    }
    inner(map, name, kind, &mut HashSet::new())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtins_parse_and_resolve() {
        let p = Profiles::builtin();
        // voron24 inherits generic: gets generic's nozzle dia, its own bed + macro.
        let s = p.resolve("voron24", "pla", "standard").unwrap();
        assert_eq!(s.bed_size_x_mm, 350.0);
        assert_eq!(s.bed_size_z_mm, 340.0); // build height
        assert_eq!(s.nozzle_diameter_mm, 0.4); // inherited from generic
        assert_eq!(s.nozzle_temp_c, 210); // from pla (Orca-matched bulk temp)
        assert_eq!(s.layer_height_mm, 0.2); // from standard
        assert!(s.start_gcode.contains("PRINT_START"));
        // No declared aux/exhaust hardware: M106 P-forms must stay locked out.
        assert!(!s.has_aux_fan && !s.has_exhaust_fan);
        // Per-feature acceleration: voron24 pins a gentle outer wall under a
        // fast interior; the first layer auto-derives to the adhesion cap.
        assert_eq!(s.acceleration_mm_s2, 10000.0);
        assert_eq!(s.outer_wall_accel_mm_s2, 3000.0);
        assert_eq!(s.first_layer_accel_mm_s2, 1000.0);
    }

    #[test]
    fn every_builtin_combination_resolves() {
        let p = Profiles::builtin();
        for printer in p.printer_names() {
            for filament in p.filament_names() {
                for process in p.process_names() {
                    p.resolve(printer, filament, process)
                        .unwrap_or_else(|e| panic!("{printer}/{filament}/{process}: {e}"));
                }
            }
        }
    }

    #[test]
    fn heat_control_keys_route_to_their_tiers() {
        // The switch + budget live on the process tier; the ranges are
        // material properties on the filament tier (auto when unset).
        let pc: ProcessProfile =
            toml::from_str("heat_control = true\nsmooth_extra_time_pct = 25.0\n").unwrap();
        assert_eq!(pc.heat_control, Some(true));
        assert_eq!(pc.smooth_extra_time_pct, Some(25.0));
        let fl: FilamentProfile =
            toml::from_str("material = \"petg\"\nmax_heat_mw_mm2 = 12.5\n").unwrap();
        assert_eq!(fl.max_heat_mw_mm2, Some(12.5));
        assert_eq!(fl.material.as_deref(), Some("petg"));
        // An unset ceiling resolves to the material class's value.
        let p = Profiles::builtin();
        let s = p.resolve("voron24", "pla", "standard").unwrap();
        assert_eq!(s.max_heat_mw_mm2, crate::Material::Pla.max_heat_mw_mm2());
        assert!(!s.heat_control, "off by default");
    }

    #[test]
    fn sovol_zero_matches_orca_speed_profile() {
        // The Sovol Zero + basic-pla numbers are matched to OrcaSlicer's
        // high-speed profile (measured from its g-code) — pin them so a
        // profile edit can't silently regress the pairing.
        let p = Profiles::builtin();
        let s = p.resolve("sovol-zero", "pla", "standard").unwrap();
        // Acceleration deliberately runs under the 40000 rating — full rate
        // hammers the frame for ~7% of a Benchy (see the profile comment).
        assert_eq!(s.acceleration_mm_s2, 15000.0);
        assert_eq!(s.outer_wall_accel_mm_s2, 6000.0); // also paces top/bottom skins
        assert_eq!(s.first_layer_accel_mm_s2, 1000.0); // auto = Orca's initial layer
        assert_eq!(s.machine_speed_mm_s, 400.0); // Orca inner wall = the rating
        assert_eq!(s.print_speed_mm_s, 320.0); // derived: 80% of rated at dial 0
        assert_eq!(s.first_layer_speed_mm_s, 55.0); // Orca initial layer
        assert_eq!(s.travel_speed_mm_s, 1000.0); // Orca travel
        assert_eq!(s.jerk_mm_s, 5.0); // Orca square-corner velocity
        assert_eq!(s.max_volumetric_speed_mm3_s, 21.0); // Orca generic-PLA melt ceiling
        // Stock firmware macros are bare START_PRINT/END_PRINT (not Voron-style
        // PRINT_START) and they do no heating — the g-code must heat explicitly,
        // to the first-layer temp (the emitter drops to the bulk temp at layer 2).
        assert!(s.start_gcode.contains("START_PRINT"));
        assert!(s.start_gcode.contains("M190 S{bed_temp}"));
        assert!(s.start_gcode.contains("M109 S{first_layer_nozzle_temp}"));
        assert!(s.end_gcode.contains("END_PRINT"));
        // Temps + pressure advance pinned to the same Orca pairing (its PLA
        // profile on this machine): hot 230 first layer for adhesion, 210 bulk.
        assert_eq!(s.first_layer_nozzle_temp_c, 230);
        assert_eq!(s.nozzle_temp_c, 210);
        assert_eq!(s.bed_temp_c, 65);
        assert_eq!(s.pressure_advance, 0.032);
        // Fan hardware flags + the Orca PLA duties for the side/exhaust fans.
        assert!(s.has_aux_fan && s.has_exhaust_fan);
        assert_eq!(s.aux_fan_speed, 0.75);
        assert_eq!(s.exhaust_fan_speed, 0.8);
        // The stock hotend is high-flow: pla-hf raises only the ceiling.
        let hf = p.resolve("sovol-zero", "pla-hf", "standard").unwrap();
        assert_eq!(hf.max_volumetric_speed_mm3_s, 30.0);
    }

    #[test]
    fn auto_speeds_balance_to_flow_ceiling() {
        let p = Profiles::builtin();
        let s = p.resolve("sovol-zero", "pla", "standard").unwrap();
        // Nominal = 80% of the 400 rating = 320. 21 mm³/s through a
        // 0.45 × 0.2 bead ≈ 258 mm/s. Support (90% of 320 = 288) would
        // overshoot — the triangle binds it...
        let cap = crate::flow_speed_cap_mm_s(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm);
        assert!((cap - 258.0).abs() < 1.0);
        assert_eq!(s.print_speed_mm_s, 320.0);
        assert_eq!(s.support_speed_mm_s, cap);
        // ...while solid's 80% (256) and outer wall's 50% (160) fit beneath.
        assert_eq!(s.solid_speed_mm_s, 256.0);
        assert_eq!(s.external_perimeter_speed_mm_s, 160.0);
        // A high-flow filament lifts the ceiling clear of every ratio.
        let hf = p.resolve("sovol-zero", "pla-hf", "standard").unwrap();
        assert_eq!(hf.solid_speed_mm_s, 256.0);
    }

    #[test]
    fn temperatures_derive_from_packaging() {
        // petg's packaging card says 230–250: the operating point lands at
        // the center, and the first layer adds the class bump clipped by the
        // packaging max.
        let p = Profiles::builtin();
        let s = p.resolve("generic", "petg", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 240);
        assert_eq!(s.first_layer_nozzle_temp_c, 250); // +10 PETG bump, fits
        // pla-hf biases warm: 190–230 at +0.25 → 215, first layer clipped 230.
        let s = p.resolve("generic", "pla-hf", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 215);
        assert_eq!(s.first_layer_nozzle_temp_c, 230);
    }

    #[test]
    fn process_inheritance_overrides() {
        let p = Profiles::builtin();
        let s = p.resolve("generic", "pla", "fine").unwrap();
        assert_eq!(s.layer_height_mm, 0.12); // fine overrides
        assert_eq!(s.line_width_mm, 0.45); // inherited from standard
        assert_eq!(s.top_layers, 6); // fine overrides
    }

    #[test]
    fn unknown_profile_errors() {
        let p = Profiles::builtin();
        assert!(p.resolve("nope", "pla", "standard").is_err());
    }

    #[test]
    fn line_width_derives_from_nozzle_when_unset() {
        let dir = std::env::temp_dir().join(format!("slicer_profiles_auto_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let mut p = Profiles::builtin();
        p.load_user_profiles(Some(dir.clone())).unwrap();
        // A 0.6 mm nozzle printer with no pinned line width -> auto 0.675.
        let pr = PrinterProfile {
            inherits: Some("generic".into()),
            nozzle_diameter_mm: Some(0.6),
            ..Default::default()
        };
        p.save_user_printer("fat-nozzle", pr).unwrap();
        let s = p.resolve("fat-nozzle", "pla", "standard").unwrap();
        assert!((s.line_width_mm - 0.675).abs() < 1e-9, "auto lw {}", s.line_width_mm);
        // Line width is pure stadium math now — every process derives it.
        let s = p.resolve("fat-nozzle", "pla", "draft").unwrap();
        assert!((s.line_width_mm - 0.675).abs() < 1e-9);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn diff_routes_fields_to_their_tiers() {
        let base = Settings::default();
        let mut cur = base.clone();
        cur.wall_count = 5; // process
        cur.temp_bias = 0.5; // filament
        cur.machine_speed_mm_s = 120.0; // printer (datasheet)
        cur.bed_size_x_mm = 300.0; // printer

        let pc = ProcessProfile::diff(&cur, &base);
        assert_eq!(pc.wall_count, Some(5));
        assert!(pc.layer_height_mm.is_none(), "untouched fields stay unset");

        let fl = FilamentProfile::diff(&cur, &base);
        assert_eq!(fl.temp_bias, Some(0.5));
        assert!(fl.bed_temp_c.is_none());

        let pr = PrinterProfile::diff(&cur, &base);
        assert_eq!(pr.print_speed_mm_s, Some(120.0));
        assert_eq!(pr.bed_size_x_mm, Some(300.0));

        assert_eq!(tier_dirty(&cur, &base), [true, true, true]);
        assert_eq!(tier_dirty(&base, &base), [false, false, false]);
    }

    #[test]
    fn save_load_roundtrip_in_user_dir() {
        let dir = std::env::temp_dir().join(format!("slicer_profiles_test_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);

        let mut p = Profiles::builtin();
        p.load_user_profiles(Some(dir.clone())).unwrap();

        // Save a filament diff inheriting petg with a hotter nozzle.
        // bias 0.5 over petg's 230-250 packaging range derives exactly 245.
        let fl = FilamentProfile { inherits: Some("petg".into()), temp_bias: Some(0.5), ..Default::default() };
        p.save_user_filament("my-petg", fl).unwrap();
        assert!(p.is_user(TierKind::Filament, "my-petg"));
        assert!(!p.is_builtin(TierKind::Filament, "my-petg"));

        // The saved file is a minimal diff (only inherits + the changed field).
        let text = fs::read_to_string(dir.join("filament/my-petg.toml")).unwrap();
        assert!(text.contains("inherits = \"petg\""), "saved: {text}");
        assert!(text.contains("temp_bias = 0.5"));
        assert!(!text.contains("bed_temp_c"), "unchanged fields must not be written");

        // It resolves over its parent, and a fresh registry loads it from disk.
        let s = p.resolve("voron24", "my-petg", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 245);
        let petg_bed = p.resolve("voron24", "petg", "standard").unwrap().bed_temp_c;
        assert_eq!(s.bed_temp_c, petg_bed, "inherited field follows the parent");

        let mut fresh = Profiles::builtin();
        fresh.load_user_profiles(Some(dir.clone())).unwrap();
        assert!(fresh.is_user(TierKind::Filament, "my-petg"));
        assert_eq!(fresh.resolve("generic", "my-petg", "standard").unwrap().nozzle_temp_c, 245);

        // Delete removes the file and the registry entry.
        fresh.delete_user(TierKind::Filament, "my-petg").unwrap();
        assert!(!dir.join("filament/my-petg.toml").exists());
        assert!(fresh.resolve("generic", "my-petg", "standard").is_err());

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn user_dir_cannot_shadow_builtins() {
        let dir = std::env::temp_dir().join(format!("slicer_profiles_shadow_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(dir.join("process")).unwrap();
        fs::write(dir.join("process/standard.toml"), "wall_count = 99\n").unwrap();
        fs::write(dir.join("process/mine.toml"), "inherits = \"standard\"\nwall_count = 5\n").unwrap();

        let mut p = Profiles::builtin();
        let skipped = p.load_user_profiles(Some(dir.clone())).unwrap();
        assert_eq!(skipped, vec!["process/standard".to_string()]);
        // The built-in survives untouched; the legit user profile loads.
        assert_ne!(p.resolve("generic", "pla", "standard").unwrap().wall_count, 99);
        assert_eq!(p.resolve("generic", "pla", "mine").unwrap().wall_count, 5);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn builtins_are_protected() {
        let dir = std::env::temp_dir().join(format!("slicer_profiles_prot_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let mut p = Profiles::builtin();
        p.load_user_profiles(Some(dir.clone())).unwrap();

        assert!(p.save_user_process("standard", ProcessProfile::default()).is_err(), "can't shadow a built-in");
        assert!(p.delete_user(TierKind::Process, "standard").is_err(), "can't delete a built-in");
        assert!(p.save_user_process("../evil", ProcessProfile::default()).is_err(), "path chars rejected");
        assert!(p.save_user_process("", ProcessProfile::default()).is_err(), "empty name rejected");

        let _ = fs::remove_dir_all(&dir);
    }
}
