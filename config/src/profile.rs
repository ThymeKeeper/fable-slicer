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

use crate::{InfillPattern, SeamMode, Settings, SupportMode, GENERIC_END_GCODE, GENERIC_START_GCODE};

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
    pub jerk: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_len_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_hop_mm: Option<f64>,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament_diameter_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density_g_cm3: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nozzle_temp_c: Option<u32>,
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
    pub fan_off_layers: Option<usize>,
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
    pub line_width_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_resolution_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_fitting: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_tolerance_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_count: Option<usize>,
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
    pub print_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub first_layer_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_layer_time_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_print_speed_mm_s: Option<f64>,
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
    pub ironing_flow: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ironing_spacing_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ironing_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elephant_foot_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub xy_compensation_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spiral_vase: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub external_perimeter_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gap_fill_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_flow: Option<f64>,
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
            travel_speed_mm_s, print_speed_mm_s, first_layer_speed_mm_s, acceleration, jerk,
            retract_len_mm, retract_speed_mm_s, z_hop_mm, start_gcode, end_gcode)
    }
}

impl Tier for FilamentProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, filament_diameter_mm, density_g_cm3, nozzle_temp_c, bed_temp_c,
            extrusion_multiplier, max_volumetric_speed_mm3_s, pressure_advance, fan_speed, bridge_fan_speed, fan_off_layers)
    }
}

impl Tier for ProcessProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, layer_height_mm, first_layer_height_mm, line_width_mm,
            max_resolution_mm, arc_fitting, arc_tolerance_mm, wall_count, top_layers, bottom_layers,
            half_height_outer_walls, brick_layers, brick_flow,
            infill_density, sparse_infill, solid_infill,
            skirt_loops, skirt_gap_mm, brim_loops, seam, support, support_overhang_angle_deg,
            support_density, support_xy_clearance_mm, support_z_gap_layers, support_interface_layers,
            max_bridge_span_mm, max_arc_radius_mm, arc_seam_overlap_mm, print_speed_mm_s, first_layer_speed_mm_s,
            bridge_speed_mm_s, min_layer_time_s, min_print_speed_mm_s,
            infill_overlap, monotonic_solid, gap_fill,
            fuzzy_skin, fuzzy_skin_thickness_mm, fuzzy_skin_point_dist_mm,
            ironing, ironing_flow, ironing_spacing_mm, ironing_speed_mm_s,
            elephant_foot_mm, xy_compensation_mm, spiral_vase,
            external_perimeter_speed_mm_s, solid_speed_mm_s, support_speed_mm_s,
            gap_fill_speed_mm_s, bridge_flow)
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
            print_speed_mm_s: diff_field!(cur.print_speed_mm_s, base.print_speed_mm_s),
            first_layer_speed_mm_s: diff_field!(cur.first_layer_speed_mm_s, base.first_layer_speed_mm_s),
            acceleration: diff_field!(cur.acceleration_mm_s2, base.acceleration_mm_s2),
            jerk: diff_field!(cur.jerk_mm_s, base.jerk_mm_s),
            retract_len_mm: diff_field!(cur.retract_len_mm, base.retract_len_mm),
            retract_speed_mm_s: diff_field!(cur.retract_speed_mm_s, base.retract_speed_mm_s),
            z_hop_mm: diff_field!(cur.z_hop_mm, base.z_hop_mm),
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
            nozzle_temp_c: diff_field!(cur.nozzle_temp_c, base.nozzle_temp_c),
            bed_temp_c: diff_field!(cur.bed_temp_c, base.bed_temp_c),
            extrusion_multiplier: diff_field!(cur.extrusion_multiplier, base.extrusion_multiplier),
            max_volumetric_speed_mm3_s: diff_field!(cur.max_volumetric_speed_mm3_s, base.max_volumetric_speed_mm3_s),
            pressure_advance: diff_field!(cur.pressure_advance, base.pressure_advance),
            fan_speed: diff_field!(cur.fan_speed, base.fan_speed),
            bridge_fan_speed: diff_field!(cur.bridge_fan_speed, base.bridge_fan_speed),
            fan_off_layers: diff_field!(cur.fan_off_layers, base.fan_off_layers),
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
            line_width_mm: diff_field!(cur.line_width_mm, base.line_width_mm),
            max_resolution_mm: diff_field!(cur.max_resolution_mm, base.max_resolution_mm),
            arc_fitting: diff_field!(cur.arc_fitting, base.arc_fitting),
            arc_tolerance_mm: diff_field!(cur.arc_tolerance_mm, base.arc_tolerance_mm),
            wall_count: diff_field!(cur.wall_count, base.wall_count),
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
            print_speed_mm_s: None,
            first_layer_speed_mm_s: None,
            bridge_speed_mm_s: diff_field!(cur.bridge_speed_mm_s, base.bridge_speed_mm_s),
            min_layer_time_s: diff_field!(cur.min_layer_time_s, base.min_layer_time_s),
            min_print_speed_mm_s: diff_field!(cur.min_print_speed_mm_s, base.min_print_speed_mm_s),
            infill_overlap: diff_field!(cur.infill_overlap, base.infill_overlap),
            monotonic_solid: diff_field!(cur.monotonic_solid, base.monotonic_solid),
            gap_fill: diff_field!(cur.gap_fill, base.gap_fill),
            fuzzy_skin: diff_field!(cur.fuzzy_skin, base.fuzzy_skin),
            fuzzy_skin_thickness_mm: diff_field!(cur.fuzzy_skin_thickness_mm, base.fuzzy_skin_thickness_mm),
            fuzzy_skin_point_dist_mm: diff_field!(cur.fuzzy_skin_point_dist_mm, base.fuzzy_skin_point_dist_mm),
            ironing: diff_field!(cur.ironing, base.ironing),
            ironing_flow: diff_field!(cur.ironing_flow, base.ironing_flow),
            ironing_spacing_mm: diff_field!(cur.ironing_spacing_mm, base.ironing_spacing_mm),
            ironing_speed_mm_s: diff_field!(cur.ironing_speed_mm_s, base.ironing_speed_mm_s),
            elephant_foot_mm: diff_field!(cur.elephant_foot_mm, base.elephant_foot_mm),
            xy_compensation_mm: diff_field!(cur.xy_compensation_mm, base.xy_compensation_mm),
            spiral_vase: diff_field!(cur.spiral_vase, base.spiral_vase),
            external_perimeter_speed_mm_s: diff_field!(cur.external_perimeter_speed_mm_s, base.external_perimeter_speed_mm_s),
            solid_speed_mm_s: diff_field!(cur.solid_speed_mm_s, base.solid_speed_mm_s),
            support_speed_mm_s: diff_field!(cur.support_speed_mm_s, base.support_speed_mm_s),
            gap_fill_speed_mm_s: diff_field!(cur.gap_fill_speed_mm_s, base.gap_fill_speed_mm_s),
            bridge_flow: diff_field!(cur.bridge_flow, base.bridge_flow),
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
        // Printer speed (machine default) takes precedence over the process value.
        let print_v = pr.print_speed_mm_s.or(pc.print_speed_mm_s).unwrap_or(d.print_speed_mm_s);
        let nozzle = pr.nozzle_diameter_mm.unwrap_or(d.nozzle_diameter_mm);
        Ok(Settings {
            nozzle_diameter_mm: nozzle,
            filament_diameter_mm: fl.filament_diameter_mm.unwrap_or(d.filament_diameter_mm),
            filament_density_g_cm3: fl.density_g_cm3.unwrap_or(d.filament_density_g_cm3),
            bed_size_x_mm: pr.bed_size_x_mm.unwrap_or(d.bed_size_x_mm),
            bed_size_y_mm: pr.bed_size_y_mm.unwrap_or(d.bed_size_y_mm),
            bed_size_z_mm: pr.bed_size_z_mm.unwrap_or(d.bed_size_z_mm),
            acceleration_mm_s2: pr.acceleration.unwrap_or(d.acceleration_mm_s2),
            jerk_mm_s: pr.jerk.unwrap_or(d.jerk_mm_s),
            layer_height_mm: pc.layer_height_mm.unwrap_or(d.layer_height_mm),
            first_layer_height_mm: pc.first_layer_height_mm.unwrap_or(d.first_layer_height_mm),
            line_width_mm: pc.line_width_mm.unwrap_or_else(|| crate::derived_line_width_mm(nozzle)),
            max_resolution_mm: pc.max_resolution_mm.unwrap_or(d.max_resolution_mm),
            arc_fitting: pc.arc_fitting.unwrap_or(d.arc_fitting),
            arc_tolerance_mm: pc.arc_tolerance_mm.unwrap_or(d.arc_tolerance_mm),
            wall_count: pc.wall_count.unwrap_or(d.wall_count),
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
            ironing_flow: pc.ironing_flow.unwrap_or(d.ironing_flow),
            ironing_spacing_mm: pc.ironing_spacing_mm.unwrap_or(d.ironing_spacing_mm),
            ironing_speed_mm_s: pc.ironing_speed_mm_s.unwrap_or(d.ironing_speed_mm_s),
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
            nozzle_temp_c: fl.nozzle_temp_c.unwrap_or(d.nozzle_temp_c),
            bed_temp_c: fl.bed_temp_c.unwrap_or(d.bed_temp_c),
            print_speed_mm_s: print_v,
            travel_speed_mm_s: pr.travel_speed_mm_s.unwrap_or(d.travel_speed_mm_s),
            first_layer_speed_mm_s: pr.first_layer_speed_mm_s.or(pc.first_layer_speed_mm_s).unwrap_or(d.first_layer_speed_mm_s),
            // Per-feature speeds scale with the machine's print speed when the
            // profile doesn't pin them (a Voron's outer wall shouldn't crawl at
            // an Ender's pace just because the default table says 25).
            external_perimeter_speed_mm_s: pc
                .external_perimeter_speed_mm_s
                .unwrap_or_else(|| crate::derived_external_perimeter_speed_mm_s(print_v)),
            solid_speed_mm_s: pc.solid_speed_mm_s.unwrap_or_else(|| crate::derived_solid_speed_mm_s(print_v)),
            support_speed_mm_s: pc
                .support_speed_mm_s
                .unwrap_or_else(|| crate::derived_support_speed_mm_s(print_v)),
            gap_fill_speed_mm_s: pc
                .gap_fill_speed_mm_s
                .unwrap_or_else(|| crate::derived_gap_fill_speed_mm_s(print_v)),
            bridge_speed_mm_s: pc.bridge_speed_mm_s.unwrap_or(d.bridge_speed_mm_s),
            min_layer_time_s: pc.min_layer_time_s.unwrap_or(d.min_layer_time_s),
            min_print_speed_mm_s: pc.min_print_speed_mm_s.unwrap_or(d.min_print_speed_mm_s),
            max_volumetric_speed_mm3_s: fl
                .max_volumetric_speed_mm3_s
                .unwrap_or(d.max_volumetric_speed_mm3_s),
            extrusion_multiplier: fl.extrusion_multiplier.unwrap_or(d.extrusion_multiplier),
            bridge_flow: pc.bridge_flow.unwrap_or(d.bridge_flow),
            pressure_advance: fl.pressure_advance.unwrap_or(d.pressure_advance),
            fan_speed: fl.fan_speed.unwrap_or(d.fan_speed),
            bridge_fan_speed: fl.bridge_fan_speed.unwrap_or(d.bridge_fan_speed),
            fan_off_layers: fl.fan_off_layers.unwrap_or(d.fan_off_layers),
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
        assert_eq!(s.nozzle_temp_c, 200); // from pla
        assert_eq!(s.layer_height_mm, 0.2); // from standard
        assert!(s.start_gcode.contains("PRINT_START"));
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
        // `draft` pins 0.5 explicitly - pin wins over auto.
        let s = p.resolve("fat-nozzle", "pla", "draft").unwrap();
        assert!((s.line_width_mm - 0.5).abs() < 1e-9);
        // Provenance is visible to the GUI: standard leaves it unset, draft pins.
        assert!(p.merged_process("standard").unwrap().line_width_mm.is_none());
        assert!(p.merged_process("draft").unwrap().line_width_mm.is_some());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn diff_routes_fields_to_their_tiers() {
        let base = Settings::default();
        let mut cur = base.clone();
        cur.wall_count = 5; // process
        cur.nozzle_temp_c = 245; // filament
        cur.print_speed_mm_s = 120.0; // printer (precedence)
        cur.bed_size_x_mm = 300.0; // printer

        let pc = ProcessProfile::diff(&cur, &base);
        assert_eq!(pc.wall_count, Some(5));
        assert_eq!(pc.print_speed_mm_s, None, "print speed must not land in process");
        assert!(pc.layer_height_mm.is_none(), "untouched fields stay unset");

        let fl = FilamentProfile::diff(&cur, &base);
        assert_eq!(fl.nozzle_temp_c, Some(245));
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
        let fl = FilamentProfile { inherits: Some("petg".into()), nozzle_temp_c: Some(245), ..Default::default() };
        p.save_user_filament("my-petg", fl).unwrap();
        assert!(p.is_user(TierKind::Filament, "my-petg"));
        assert!(!p.is_builtin(TierKind::Filament, "my-petg"));

        // The saved file is a minimal diff (only inherits + the changed field).
        let text = fs::read_to_string(dir.join("filament/my-petg.toml")).unwrap();
        assert!(text.contains("inherits = \"petg\""), "saved: {text}");
        assert!(text.contains("nozzle_temp_c = 245"));
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
