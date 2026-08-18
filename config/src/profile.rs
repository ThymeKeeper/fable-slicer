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
    InfillPattern, SeamMode, Settings, SupportMode, GENERIC_END_GCODE,
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
    pub min_cruise_ratio: Option<f64>,
    /// Emit curves as G2/G3 arcs — a firmware capability (needs Klipper
    /// `[gcode_arcs]`, Marlin `ARC_SUPPORT`, etc.), so it lives on the printer.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_fitting: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub arc_tolerance_mm: Option<f64>,
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
    pub aux_fan: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exhaust_fan: Option<bool>,
    /// Klipper `temperature_sensor` name of the chamber thermistor
    /// ("" / unset = the machine has none).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chamber_sensor: Option<String>,
    /// Machine kind: "toolchanger" (default) or "mmu" (single-nozzle Happy
    /// Hare / ERCF / AMS). Unset = toolchanger.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub machine_kind: Option<String>,
    /// Number of tools (StealthChanger etc.); 1 / unset = single-tool.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_count: Option<u32>,
    /// Template emitted at each tool change; `{tool}` = the target tool number
    /// (MMU swaps also get `{from_tool}` / `{to_temp}` / `{purge_mm3}` / `{purge_mm}`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolchange_gcode: Option<String>,
    /// Estimated seconds per tool change (time estimate / M73).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub toolchange_seconds: Option<f64>,
    /// MMU only: static filament volume (mm³) purged per swap.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub purge_volume_mm3: Option<f64>,
    /// Docked longer than this (estimated seconds) and a tool drops to its
    /// filament's standby temperature, reheating ahead of its next pickup.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standby_after_s: Option<f64>,
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
    /// Display color "#RRGGBB" for preview/part tinting.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament_diameter_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density_g_cm3: Option<f64>,
    /// Operating nozzle °C from the spool.
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
    /// Filament pulled back on travels (mm). A material property — PETG oozes
    /// more than PLA — so it lives here, not on the printer. Retraction speed,
    /// z-hop, and wipe stay machine-tier.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_len_mm: Option<f64>,
    /// Filament added (mm) to the de-retract at each restart; negative de-primes
    /// to absorb the unretract's seam blob. 0 = symmetric.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retract_restart_extra_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fan_speed: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_fan_speed: Option<f64>,
    /// Ceiling of the short-layer cooling ramp — the most fan this spool
    /// tolerates on plain walls (warp-prone materials cap it well under the
    /// bridge duty). Auto: `bridge_fan_speed`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fan_max: Option<f64>,
    /// Flow multiplier for bridge strands (and arc overhangs). Auto: 1.5.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_flow: Option<f64>,
    /// Print speed (mm/s) for bridge strands. Auto: 10.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_flow_derate_per_c: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fan_off_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aux_fan_speed: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exhaust_fan_speed: Option<f64>,
    /// Chamber pre-soak target (°C, 0 = off) on machines with a chamber
    /// sensor. Auto: the material class's value (ABS/ASA 50, others 0).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub chamber_temp_c: Option<u32>,
    /// Nozzle setpoint (°C) while this tool sits docked on a toolchanger —
    /// hot enough to restart quickly, cool enough not to ooze and cook.
    /// Auto: operating temperature − 50, floored at 110.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standby_temp_c: Option<u32>,
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
    /// Bead width. Unset = derived from the nozzle (× 1.125); set to override.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub line_width_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wall_count: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outer_wall_first: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottom_layers: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub infill_density: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sparse_infill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_infill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottom_infill: Option<String>,
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
    /// How far (mm) an enclosed-ceiling bridge lands onto the supported rim.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bridge_foothold_mm: Option<f64>,
    /// Tallest dither repeat (mm) a blend may have and still read as one
    /// color — the blend picker only offers mixes whose layer cycle fits.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub blend_band_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub infill_overlap: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monotonic_solid: Option<bool>,
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
            outer_wall_accel, first_layer_accel, jerk, min_cruise_ratio, arc_fitting, arc_tolerance_mm,
            retract_speed_mm_s, z_hop_mm, wipe_mm, host_url, api_key,
            aux_fan, exhaust_fan, chamber_sensor,
            machine_kind, tool_count, toolchange_gcode, toolchange_seconds, purge_volume_mm3,
            standby_after_s, start_gcode, end_gcode)
    }
}

impl Tier for FilamentProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, material, color, filament_diameter_mm, density_g_cm3,
            nozzle_temp_c, bed_temp_c,
            extrusion_multiplier, max_volumetric_speed_mm3_s, max_flow_derate_per_c,
            pressure_advance, retract_len_mm, retract_restart_extra_mm,
            fan_speed, bridge_fan_speed, fan_max, bridge_flow, bridge_speed_mm_s,
            fan_off_layers, aux_fan_speed, exhaust_fan_speed,
            chamber_temp_c, standby_temp_c)
    }
}

impl Tier for ProcessProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, layer_height_mm, first_layer_height_mm, line_width_mm,
            wall_count, outer_wall_first, top_layers, bottom_layers,
            infill_density, sparse_infill, top_infill, bottom_infill, solid_infill,
            skirt_loops, skirt_gap_mm, brim_loops, seam, support, support_overhang_angle_deg,
            support_density, support_xy_clearance_mm, support_z_gap_layers, support_interface_layers,
            max_bridge_span_mm, bridge_foothold_mm, blend_band_mm,
            infill_overlap, monotonic_solid,
            fuzzy_skin, fuzzy_skin_thickness_mm, fuzzy_skin_point_dist_mm,
            ironing,
            elephant_foot_mm, xy_compensation_mm, spiral_vase,
            speed_quality)
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
            min_cruise_ratio: diff_field!(cur.min_cruise_ratio, base.min_cruise_ratio),
            arc_fitting: diff_field!(cur.arc_fitting, base.arc_fitting),
            arc_tolerance_mm: diff_field!(cur.arc_tolerance_mm, base.arc_tolerance_mm),
            retract_speed_mm_s: diff_field!(cur.retract_speed_mm_s, base.retract_speed_mm_s),
            z_hop_mm: diff_field!(cur.z_hop_mm, base.z_hop_mm),
            wipe_mm: diff_field!(cur.wipe_mm, base.wipe_mm),
            host_url: diff_field!(cur.host_url.clone(), base.host_url),
            api_key: diff_field!(cur.api_key.clone(), base.api_key),
            aux_fan: diff_field!(cur.has_aux_fan, base.has_aux_fan),
            exhaust_fan: diff_field!(cur.has_exhaust_fan, base.has_exhaust_fan),
            chamber_sensor: diff_field!(cur.chamber_sensor.clone(), base.chamber_sensor),
            machine_kind: diff_field!(
                cur.machine_kind.label().to_string(),
                base.machine_kind.label().to_string()
            ),
            tool_count: diff_field!(cur.tool_count as u32, base.tool_count as u32),
            toolchange_gcode: diff_field!(cur.toolchange_gcode.clone(), base.toolchange_gcode),
            toolchange_seconds: diff_field!(cur.toolchange_seconds, base.toolchange_seconds),
            purge_volume_mm3: diff_field!(cur.purge_volume_mm3, base.purge_volume_mm3),
            standby_after_s: diff_field!(cur.standby_after_s, base.standby_after_s),
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
            color: diff_field!(cur.filament_color_rgb, base.filament_color_rgb).map(crate::hex_color),
            nozzle_temp_c: diff_field!(cur.nozzle_temp_c, base.nozzle_temp_c),
            bed_temp_c: diff_field!(cur.bed_temp_c, base.bed_temp_c),
            extrusion_multiplier: diff_field!(cur.extrusion_multiplier, base.extrusion_multiplier),
            max_volumetric_speed_mm3_s: diff_field!(cur.max_volumetric_speed_mm3_s, base.max_volumetric_speed_mm3_s),
            max_flow_derate_per_c: diff_field!(cur.max_flow_derate_per_c, base.max_flow_derate_per_c),
            pressure_advance: diff_field!(cur.pressure_advance, base.pressure_advance),
            retract_len_mm: diff_field!(cur.retract_len_mm, base.retract_len_mm),
            retract_restart_extra_mm: diff_field!(cur.retract_restart_extra_mm, base.retract_restart_extra_mm),
            fan_speed: diff_field!(cur.fan_speed, base.fan_speed),
            bridge_fan_speed: diff_field!(cur.bridge_fan_speed, base.bridge_fan_speed),
            fan_max: diff_field!(cur.fan_max, base.fan_max),
            bridge_flow: diff_field!(cur.bridge_flow, base.bridge_flow),
            bridge_speed_mm_s: diff_field!(cur.bridge_speed_mm_s, base.bridge_speed_mm_s),
            fan_off_layers: diff_field!(cur.fan_off_layers, base.fan_off_layers),
            aux_fan_speed: diff_field!(cur.aux_fan_speed, base.aux_fan_speed),
            exhaust_fan_speed: diff_field!(cur.exhaust_fan_speed, base.exhaust_fan_speed),
            chamber_temp_c: diff_field!(cur.chamber_temp_c, base.chamber_temp_c),
            standby_temp_c: diff_field!(cur.standby_temp_c, base.standby_temp_c),
        }
    }

    /// The filament-tier fields where one tool slot's resolved view differs
    /// from its baseline — the per-slot mirror of [`Self::diff`], for
    /// toolchanger tabs that edit `Settings::tools[i]` directly. Two fields
    /// are excluded by design: color (the loaded spool's override is slot
    /// data, never auto-promoted into the profile) and the first-layer
    /// temperature (always derived from the operating temp + material).
    pub fn diff_tool(cur: &crate::ToolSettings, base: &crate::ToolSettings) -> Self {
        Self {
            inherits: None,
            filament_diameter_mm: diff_field!(cur.filament_diameter_mm, base.filament_diameter_mm),
            density_g_cm3: diff_field!(cur.filament_density_g_cm3, base.filament_density_g_cm3),
            material: diff_field!(cur.material, base.material).map(|m| m.label().to_string()),
            color: None,
            nozzle_temp_c: diff_field!(cur.nozzle_temp_c, base.nozzle_temp_c),
            bed_temp_c: diff_field!(cur.bed_temp_c, base.bed_temp_c),
            extrusion_multiplier: diff_field!(cur.extrusion_multiplier, base.extrusion_multiplier),
            max_volumetric_speed_mm3_s: diff_field!(cur.max_volumetric_speed_mm3_s, base.max_volumetric_speed_mm3_s),
            max_flow_derate_per_c: diff_field!(cur.max_flow_derate_per_c, base.max_flow_derate_per_c),
            pressure_advance: diff_field!(cur.pressure_advance, base.pressure_advance),
            retract_len_mm: diff_field!(cur.retract_len_mm, base.retract_len_mm),
            retract_restart_extra_mm: diff_field!(cur.retract_restart_extra_mm, base.retract_restart_extra_mm),
            fan_speed: diff_field!(cur.fan_speed, base.fan_speed),
            bridge_fan_speed: diff_field!(cur.bridge_fan_speed, base.bridge_fan_speed),
            fan_max: diff_field!(cur.fan_max, base.fan_max),
            bridge_flow: diff_field!(cur.bridge_flow, base.bridge_flow),
            bridge_speed_mm_s: diff_field!(cur.bridge_speed_mm_s, base.bridge_speed_mm_s),
            fan_off_layers: diff_field!(cur.fan_off_layers, base.fan_off_layers),
            aux_fan_speed: diff_field!(cur.aux_fan_speed, base.aux_fan_speed),
            exhaust_fan_speed: diff_field!(cur.exhaust_fan_speed, base.exhaust_fan_speed),
            chamber_temp_c: diff_field!(cur.chamber_temp_c, base.chamber_temp_c),
            standby_temp_c: diff_field!(cur.standby_temp_c, base.standby_temp_c),
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
            wall_count: diff_field!(cur.wall_count, base.wall_count),
            outer_wall_first: diff_field!(cur.outer_wall_first, base.outer_wall_first),
            top_layers: diff_field!(cur.top_layers, base.top_layers),
            bottom_layers: diff_field!(cur.bottom_layers, base.bottom_layers),
            infill_density: diff_field!(cur.infill_density, base.infill_density),
            sparse_infill: diff_field!(cur.sparse_pattern, base.sparse_pattern).map(|p| p.label().to_string()),
            top_infill: diff_field!(cur.top_pattern, base.top_pattern).map(|p| p.label().to_string()),
            bottom_infill: diff_field!(cur.bottom_pattern, base.bottom_pattern).map(|p| p.label().to_string()),
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
            blend_band_mm: diff_field!(cur.blend_band_mm, base.blend_band_mm),
            bridge_foothold_mm: diff_field!(cur.bridge_foothold_mm, base.bridge_foothold_mm),
            // print/first-layer speed are printer-tier (see PrinterProfile::diff).
            infill_overlap: diff_field!(cur.infill_overlap, base.infill_overlap),
            monotonic_solid: diff_field!(cur.monotonic_solid, base.monotonic_solid),
            fuzzy_skin: diff_field!(cur.fuzzy_skin, base.fuzzy_skin),
            fuzzy_skin_thickness_mm: diff_field!(cur.fuzzy_skin_thickness_mm, base.fuzzy_skin_thickness_mm),
            fuzzy_skin_point_dist_mm: diff_field!(cur.fuzzy_skin_point_dist_mm, base.fuzzy_skin_point_dist_mm),
            ironing: diff_field!(cur.ironing, base.ironing),
            elephant_foot_mm: diff_field!(cur.elephant_foot_mm, base.elephant_foot_mm),
            xy_compensation_mm: diff_field!(cur.xy_compensation_mm, base.xy_compensation_mm),
            spiral_vase: diff_field!(cur.spiral_vase, base.spiral_vase),
            speed_quality: diff_field!(cur.speed_quality, base.speed_quality),
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
        p.filaments.insert("asa".into(), parse("filament/asa", include_str!("../profiles/filament/asa.toml")));
        p.processes.insert("standard".into(), parse("process/standard", include_str!("../profiles/process/standard.toml")));
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

    /// The per-user profile directory: `profiles/` inside the app's dotfile
    /// folder (see [`crate::config_dir`]), which also carries `state.toml`.
    pub fn default_user_dir() -> Option<std::path::PathBuf> {
        crate::config_dir().map(|d| d.join("profiles"))
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

    /// Resolve the three named profiles into flat [`Settings`]. Exactly
    /// [`Self::resolve_tools`] with a single filament.
    pub fn resolve(&self, printer: &str, filament: &str, process: &str) -> Result<Settings, String> {
        self.resolve_tools(printer, &[filament], process)
    }

    /// Resolve one printer + process with a filament per tool slot (a
    /// toolchanger loadout; `filaments` must be non-empty). Flat filament
    /// fields mirror `filaments[0]` — the initial tool's view — except the
    /// shared-hardware aggregates: bed and chamber targets take the **max**
    /// over tools (the hottest requirement wins the shared heater), and the
    /// derived feature speeds clamp to the **minimum** flow ceiling across
    /// tools (conservative: any tool may print any feature). `tools[i]`
    /// carries each filament's full per-tool view.
    pub fn resolve_tools(&self, printer: &str, filaments: &[&str], process: &str) -> Result<Settings, String> {
        let pr = resolve_tier(&self.printers, printer, "printer")?;
        let pc = resolve_tier(&self.processes, process, "process")?;
        if filaments.is_empty() {
            return Err("resolve_tools needs at least one filament".into());
        }
        let d = Settings::default();
        let tools = filaments
            .iter()
            .map(|name| Ok(tool_settings(name, &resolve_tier(&self.filaments, name, "filament")?, &d)))
            .collect::<Result<Vec<_>, String>>()?;
        let t0 = tools[0].clone();
        // The machine's rating × the finish↔speed dial → the nominal speed.
        let machine_v = pr.print_speed_mm_s.unwrap_or(d.machine_speed_mm_s);
        let quality = pc.speed_quality.unwrap_or(d.speed_quality);
        let print_v = crate::derived_print_speed_mm_s(machine_v, quality);
        let nozzle = pr.nozzle_diameter_mm.unwrap_or(d.nozzle_diameter_mm);
        // The flow triangle: speed × bead area (line width × layer height) must
        // fit the filament's melt ceiling, so derived speeds balance against
        // it. The slowest ceiling across loaded tools binds — any tool may
        // print any feature.
        let line_w = pc.line_width_mm.unwrap_or_else(|| crate::derived_line_width_mm(nozzle));
        let layer_h = pc.layer_height_mm.unwrap_or(d.layer_height_mm);
        let flow_cap = tools
            .iter()
            .map(|t| crate::flow_speed_cap_mm_s(t.max_volumetric_speed_mm3_s, line_w, layer_h))
            .fold(f64::INFINITY, f64::min);
        // The bed and chamber are shared across tools: the hottest wish wins.
        let bed_temp = tools.iter().map(|t| t.bed_temp_c).max().unwrap_or(d.bed_temp_c);
        let chamber_temp = tools.iter().map(|t| t.chamber_temp_c).max().unwrap_or(d.chamber_temp_c);
        Ok(Settings {
            nozzle_diameter_mm: nozzle,
            filament_diameter_mm: t0.filament_diameter_mm,
            filament_density_g_cm3: t0.filament_density_g_cm3,
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
            min_cruise_ratio: pr.min_cruise_ratio.unwrap_or(d.min_cruise_ratio),
            layer_height_mm: layer_h,
            first_layer_height_mm: pc.first_layer_height_mm.unwrap_or(d.first_layer_height_mm),
            line_width_mm: line_w,
            arc_fitting: pr.arc_fitting.unwrap_or(d.arc_fitting),
            arc_tolerance_mm: pr.arc_tolerance_mm.unwrap_or(d.arc_tolerance_mm),
            wall_count: pc.wall_count.unwrap_or(d.wall_count),
            outer_wall_first: pc.outer_wall_first.unwrap_or(d.outer_wall_first),
            top_layers: pc.top_layers.unwrap_or(d.top_layers),
            bottom_layers: pc.bottom_layers.unwrap_or(d.bottom_layers),
            infill_density: pc.infill_density.unwrap_or(d.infill_density),
            sparse_pattern: pc.sparse_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.sparse_pattern),
            top_pattern: pc.top_infill.as_deref().and_then(InfillPattern::parse)
                .or_else(|| pc.solid_infill.as_deref().and_then(InfillPattern::parse))
                .unwrap_or(d.top_pattern),
            bottom_pattern: pc.bottom_infill.as_deref().and_then(InfillPattern::parse)
                .or_else(|| pc.solid_infill.as_deref().and_then(InfillPattern::parse))
                .unwrap_or(d.bottom_pattern),
            solid_pattern: pc.solid_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.solid_pattern),
            infill_overlap: pc.infill_overlap.unwrap_or(d.infill_overlap),
            monotonic_solid: pc.monotonic_solid.unwrap_or(d.monotonic_solid),
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
            blend_band_mm: pc.blend_band_mm.unwrap_or(d.blend_band_mm),
            bridge_foothold_mm: pc.bridge_foothold_mm.unwrap_or(d.bridge_foothold_mm),
            // Retraction distance now rides the filament tier — mirror tool 0's,
            // like every other per-tool filament field. Speed/z-hop/wipe stay
            // machine-tier (`pr`).
            retract_len_mm: t0.retract_len_mm,
            retract_restart_extra_mm: t0.retract_restart_extra_mm,
            retract_speed_mm_s: pr.retract_speed_mm_s.unwrap_or(d.retract_speed_mm_s),
            z_hop_mm: pr.z_hop_mm.unwrap_or(d.z_hop_mm),
            wipe_mm: pr.wipe_mm.unwrap_or(d.wipe_mm),
            host_url: pr.host_url.unwrap_or(d.host_url),
            api_key: pr.api_key.unwrap_or(d.api_key),
            material: t0.material,
            nozzle_temp_c: t0.nozzle_temp_c,
            first_layer_nozzle_temp_c: t0.first_layer_nozzle_temp_c,
            bed_temp_c: bed_temp,
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
            bridge_speed_mm_s: t0.bridge_speed_mm_s,
            overhang_speed_mm_s: crate::derived_overhang_speed_mm_s(t0.bridge_speed_mm_s),
            min_layer_time_s: d.min_layer_time_s,
            min_print_speed_mm_s: d.min_print_speed_mm_s,
            max_volumetric_speed_mm3_s: t0.max_volumetric_speed_mm3_s,
            max_flow_derate_per_c: t0.max_flow_derate_per_c,
            extrusion_multiplier: t0.extrusion_multiplier,
            bridge_flow: t0.bridge_flow,
            pressure_advance: t0.pressure_advance,
            fan_speed: t0.fan_speed,
            bridge_fan_speed: t0.bridge_fan_speed,
            fan_max: t0.fan_max,
            fan_off_layers: t0.fan_off_layers,
            has_aux_fan: pr.aux_fan.unwrap_or(d.has_aux_fan),
            has_exhaust_fan: pr.exhaust_fan.unwrap_or(d.has_exhaust_fan),
            aux_fan_speed: t0.aux_fan_speed,
            exhaust_fan_speed: t0.exhaust_fan_speed,
            printer_name: printer.to_string(),
            chamber_sensor: pr.chamber_sensor.unwrap_or_else(|| d.chamber_sensor.clone()),
            chamber_temp_c: chamber_temp,
            machine_kind: pr
                .machine_kind
                .as_deref()
                .and_then(crate::MachineKind::parse)
                .unwrap_or(d.machine_kind),
            // A declared tool_count of 0 is meaningless — clamp to single-tool.
            tool_count: pr.tool_count.map(|c| (c as usize).max(1)).unwrap_or(d.tool_count),
            toolchange_gcode: pr.toolchange_gcode.unwrap_or_else(|| d.toolchange_gcode.clone()),
            toolchange_seconds: pr.toolchange_seconds.unwrap_or(d.toolchange_seconds),
            purge_volume_mm3: pr.purge_volume_mm3.unwrap_or(d.purge_volume_mm3),
            standby_temp_c: t0.standby_temp_c,
            standby_after_s: pr.standby_after_s.unwrap_or(d.standby_after_s),
            filament_color_rgb: t0.color_rgb,
            tools,
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

/// One tool slot's resolved view of a merged filament profile — the same
/// fallback chain the flat fields use: the profile's value, else the material
/// class off the box, else the code default.
fn tool_settings(name: &str, fl: &FilamentProfile, d: &Settings) -> crate::ToolSettings {
    let material = fl.material.as_deref().and_then(crate::Material::parse).unwrap_or(d.material);
    // The operating nozzle temperature: the spool's value, else the class.
    let nozzle_temp = fl.nozzle_temp_c.unwrap_or_else(|| material.nozzle_temp_c());
    let bridge_fan = fl.bridge_fan_speed.unwrap_or_else(|| material.fan().1);
    crate::ToolSettings {
        filament_name: name.to_string(),
        color_rgb: fl
            .color
            .as_deref()
            .and_then(crate::parse_hex_color)
            .unwrap_or(crate::NEUTRAL_FILAMENT_RGB),
        material,
        filament_diameter_mm: fl.filament_diameter_mm.unwrap_or(d.filament_diameter_mm),
        filament_density_g_cm3: fl.density_g_cm3.unwrap_or_else(|| material.density_g_cm3()),
        nozzle_temp_c: nozzle_temp,
        first_layer_nozzle_temp_c: crate::derived_first_layer_temp_c(nozzle_temp, material),
        bed_temp_c: fl.bed_temp_c.unwrap_or_else(|| material.bed_temp_c()),
        max_volumetric_speed_mm3_s: fl.max_volumetric_speed_mm3_s.unwrap_or_else(|| material.max_flow_mm3_s()),
        max_flow_derate_per_c: fl.max_flow_derate_per_c.unwrap_or_else(|| material.max_flow_derate_per_c()),
        extrusion_multiplier: fl.extrusion_multiplier.unwrap_or(d.extrusion_multiplier),
        pressure_advance: fl.pressure_advance.unwrap_or(d.pressure_advance),
        retract_len_mm: fl.retract_len_mm.unwrap_or(d.retract_len_mm),
        retract_restart_extra_mm: fl.retract_restart_extra_mm.unwrap_or(d.retract_restart_extra_mm),
        bridge_flow: fl.bridge_flow.unwrap_or(d.bridge_flow),
        bridge_speed_mm_s: fl.bridge_speed_mm_s.unwrap_or(d.bridge_speed_mm_s),
        fan_speed: fl.fan_speed.unwrap_or_else(|| material.fan().0),
        bridge_fan_speed: bridge_fan,
        // The ladder ceiling: the card's own cap, else the bridge duty —
        // the pre-fan_max behavior, so profiles without the field resolve
        // byte-identically.
        fan_max: fl.fan_max.unwrap_or(bridge_fan),
        fan_off_layers: fl.fan_off_layers.unwrap_or_else(|| material.fan().2),
        aux_fan_speed: fl.aux_fan_speed.unwrap_or_else(|| material.aux_exhaust().0),
        exhaust_fan_speed: fl.exhaust_fan_speed.unwrap_or_else(|| material.aux_exhaust().1),
        chamber_temp_c: fl.chamber_temp_c.unwrap_or_else(|| material.chamber_temp_c()),
        standby_temp_c: fl
            .standby_temp_c
            .unwrap_or_else(|| crate::derived_standby_temp_c(nozzle_temp)),
    }
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
        assert_eq!(s.nozzle_temp_c, 207); // from pla (user's calibrated bulk temp)
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
    fn removed_heat_keys_still_load() {
        // Heat control was removed; old saved profiles may still carry its keys
        // (heat_control / smooth_extra_time_pct on the process tier,
        // max_heat_mw_mm2 on the filament tier). They must be silently ignored,
        // not error the load — the rest of the profile still resolves.
        let pc: ProcessProfile =
            toml::from_str("heat_control = true\nsmooth_extra_time_pct = 25.0\nwall_count = 3\n").unwrap();
        assert_eq!(pc.wall_count, Some(3));
        let fl: FilamentProfile =
            toml::from_str("material = \"petg\"\nmax_heat_mw_mm2 = 12.5\n").unwrap();
        assert_eq!(fl.material.as_deref(), Some("petg"));
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
        assert_eq!(s.max_volumetric_speed_mm3_s, 15.0); // user's calibrated ceiling (was Orca's 21)
        // Stock firmware macros are bare START_PRINT/END_PRINT (not Voron-style
        // PRINT_START) and they do no heating — the g-code must heat explicitly,
        // to the first-layer temp (the emitter drops to the bulk temp at layer 2).
        assert!(s.start_gcode.contains("START_PRINT"));
        assert!(s.start_gcode.contains("M190 S{bed_temp}"));
        assert!(s.start_gcode.contains("M109 S{first_layer_nozzle_temp}"));
        assert!(s.end_gcode.contains("END_PRINT"));
        // Temps + pressure advance: the user's calibrated card (promoted from
        // pla-custom 2026-08-16): 207 bulk, +20 PLA bump = 227 first layer.
        assert_eq!(s.first_layer_nozzle_temp_c, 227);
        assert_eq!(s.nozzle_temp_c, 207);
        assert_eq!(s.bed_temp_c, 65);
        assert_eq!(s.pressure_advance, 0.044);
        // Fan hardware flags + the Orca PLA duties for the side/exhaust fans.
        assert!(s.has_aux_fan && s.has_exhaust_fan);
        assert_eq!(s.aux_fan_speed, 0.75);
        assert_eq!(s.exhaust_fan_speed, 0.8);
        // A high-flow card raises only the ceiling (synthesized fixture —
        // the shipped pla-hf card was removed with the other unused cards).
        let mut p = p;
        p.filaments.insert("hf".into(), hf_fixture());
        let hf = p.resolve("sovol-zero", "hf", "standard").unwrap();
        assert_eq!(hf.max_volumetric_speed_mm3_s, 30.0);
    }

    /// High-flow PLA fixture for ceiling/derivation tests: inherits the pla
    /// card, 215 °C operating, 30 mm³/s melt ceiling (the deleted pla-hf).
    fn hf_fixture() -> FilamentProfile {
        FilamentProfile {
            inherits: Some("pla".into()),
            nozzle_temp_c: Some(215),
            max_volumetric_speed_mm3_s: Some(30.0),
            ..Default::default()
        }
    }

    #[test]
    fn asa_rides_the_abs_class_with_chamber_presoak() {
        let p = Profiles::builtin();
        let s = p.resolve("sovol-zero", "asa", "standard").unwrap();
        // The card's operating temp; density is ASA's own (the class default
        // would be ABS's 1.04).
        assert_eq!(s.nozzle_temp_c, 260);
        assert_eq!(s.bed_temp_c, 100);
        assert_eq!(s.filament_density_g_cm3, 1.07);
        // Class-derived cooling: low base duty (moving air cracks ABS/ASA)
        // with the aux/exhaust fans fully off — the sealed still chamber is
        // part of what makes ABS/ASA prints hold together.
        assert_eq!(s.fan_speed, 0.15);
        assert_eq!(s.aux_fan_speed, 0.0);
        // The chamber pre-soak pairing: the printer declares the sensor (its
        // Klipper name, verified live on the machine); the card pins 40 °C
        // (measured comfortable on the Zero; the class default is 50).
        assert_eq!(s.chamber_sensor, "chamber_temp");
        assert_eq!(s.chamber_temp_c, 40);
        // PLA on the same machine must never soak — a warm chamber means
        // heat creep and sag.
        let pla = p.resolve("sovol-zero", "pla", "standard").unwrap();
        assert_eq!(pla.chamber_temp_c, 0);
        // And the generic printer declares no sensor: the ASA soak wish still
        // rides through (50 C). The emitter gates on the temp (not the sensor),
        // so the slice carries a TEMPERATURE_WAIT that aborts on a sensorless
        // machine rather than printing ASA cold — the pre-send Moonraker check
        // turns that into a legible message.
        let generic = p.resolve("generic", "asa", "standard").unwrap();
        assert!(generic.chamber_sensor.is_empty());
        assert_eq!(generic.chamber_temp_c, 40); // the wish survives; a sensorless machine errors at print time
        // The Voron spec wires [temperature_sensor chamber].
        let voron = p.resolve("voron24", "asa", "standard").unwrap();
        assert_eq!(voron.chamber_sensor, "chamber");
    }

    #[test]
    fn bambu_petg_matches_orca_reference() {
        // The card mirrors OrcaSlicer's "Bambu PETG Basic @System" — the
        // profile a Voron actually receives for this spool (Orca's Voron
        // vendor ships no filament profiles of its own) — pin the pairing so
        // a profile edit can't silently drift from the reference.
        let p = Profiles::builtin();
        let s = p.resolve("sovol-zero", "petg", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 255);
        // First layer flat at 255, matching Orca's emitted behavior — the
        // old +10 class bump (265) made PETG grab the nozzle and shred.
        assert_eq!(s.first_layer_nozzle_temp_c, 255);
        // 80 like the generic petg card (Bambu textured-plate value) — the
        // Orca card's 70 is the smooth-PEI number and lifted a corner here.
        assert_eq!(s.bed_temp_c, 80);
        assert_eq!(s.extrusion_multiplier, 0.90); // print-quality measured; Orca's flow_ratio is 0.95
        assert_eq!(s.pressure_advance, 0.05); // local; the Orca card sets none
        assert_eq!(s.filament_density_g_cm3, 1.25);
        // Orca's card rates 13 (Bambu hotend); walls sustained at 13 shred
        // PETG on the Sovol — deliberate deviation (see the card).
        assert_eq!(s.max_volumetric_speed_mm3_s, 9.0);
        // Orca's card ramps 10→40 by layer time (90 on overhangs); the base
        // deliberately deviates to a flat 40 — at 10% a dense wall field on
        // long layers heat-soaks PETG into ragged walls (see the card).
        assert_eq!(s.fan_speed, 0.4);
        assert_eq!(s.fan_max, 0.4);
        assert_eq!(s.bridge_fan_speed, 0.9);
        assert_eq!(s.fan_off_layers, 3); // class = Orca's first-3-layers-off
        assert_eq!(s.retract_len_mm, 0.6); // filament-tier here; Orca defers to printer
        assert_eq!(s.chamber_temp_c, 0); // PETG never pre-soaks
    }

    #[test]
    fn auto_speeds_balance_to_flow_ceiling() {
        let p = Profiles::builtin();
        let s = p.resolve("sovol-zero", "pla", "standard").unwrap();
        // Nominal = 80% of the 400 rating = 320. 15 mm³/s through a
        // 0.45 × 0.2 bead ≈ 184 mm/s. Support (90% of 320 = 288) and
        // solid (80% = 256) both overshoot — the ceiling binds them...
        let cap = crate::flow_speed_cap_mm_s(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm);
        assert!((cap - 184.3).abs() < 1.0);
        assert_eq!(s.print_speed_mm_s, 320.0);
        assert_eq!(s.support_speed_mm_s, cap);
        assert_eq!(s.solid_speed_mm_s, cap);
        // ...while outer wall's 50% (160) fits beneath.
        assert_eq!(s.external_perimeter_speed_mm_s, 160.0);
        // A high-flow filament lifts the ceiling clear of every ratio.
        let mut p = p;
        p.filaments.insert("hf".into(), hf_fixture());
        let hf = p.resolve("sovol-zero", "hf", "standard").unwrap();
        assert_eq!(hf.solid_speed_mm_s, 256.0);
    }

    #[test]
    fn nozzle_and_first_layer_temps() {
        // The card sets the operating nozzle temp directly; the first layer
        // adds the material class's adhesion bump on top.
        let p = Profiles::builtin();
        let s = p.resolve("generic", "petg", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 255);
        assert_eq!(s.first_layer_nozzle_temp_c, 255); // PETG: no bump (shreds hot)
        // A PLA-class card at 215 °C: +20 PLA bump on the first layer.
        let mut p = p;
        p.filaments.insert("hf".into(), hf_fixture());
        let s = p.resolve("generic", "hf", "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 215);
        assert_eq!(s.first_layer_nozzle_temp_c, 235);
    }

    #[test]
    fn process_inheritance_overrides() {
        // A process inheriting from the built-in `standard`, overriding a couple of
        // fields — exercises the process inherits chain (standard is the only
        // built-in process now, so synthesize the child here).
        let mut p = Profiles::builtin();
        p.processes.insert(
            "myfine".into(),
            ProcessProfile {
                inherits: Some("standard".into()),
                layer_height_mm: Some(0.12),
                top_layers: Some(6),
                ..Default::default()
            },
        );
        let s = p.resolve("generic", "pla", "myfine").unwrap();
        assert_eq!(s.layer_height_mm, 0.12); // child overrides
        assert_eq!(s.top_layers, 6); // child overrides
        assert_eq!(s.line_width_mm, 0.45); // derived from the 0.4 nozzle, not set by either
    }

    #[test]
    fn unknown_profile_errors() {
        let p = Profiles::builtin();
        assert!(p.resolve("nope", "pla", "standard").is_err());
    }

    #[test]
    fn line_width_auto_from_nozzle_or_overridden() {
        let dir = std::env::temp_dir().join(format!("slicer_profiles_auto_{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        let mut p = Profiles::builtin();
        p.load_user_profiles(Some(dir.clone())).unwrap();
        // A 0.6 mm nozzle printer with no pinned line width -> auto 0.675 (× 1.125).
        let pr = PrinterProfile {
            inherits: Some("generic".into()),
            nozzle_diameter_mm: Some(0.6),
            ..Default::default()
        };
        p.save_user_printer("fat-nozzle", pr).unwrap();
        let s = p.resolve("fat-nozzle", "pla", "standard").unwrap();
        assert!((s.line_width_mm - 0.675).abs() < 1e-9, "auto lw {}", s.line_width_mm);

        // A process that pins line width overrides the nozzle-derived value.
        let proc = ProcessProfile {
            inherits: Some("standard".into()),
            line_width_mm: Some(0.5),
            ..Default::default()
        };
        p.save_user_process("wide-bead", proc).unwrap();
        let s = p.resolve("fat-nozzle", "pla", "wide-bead").unwrap();
        assert!((s.line_width_mm - 0.5).abs() < 1e-9, "pinned lw {}", s.line_width_mm);
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn diff_routes_fields_to_their_tiers() {
        let base = Settings::default();
        let mut cur = base.clone();
        cur.wall_count = 5; // process
        cur.nozzle_temp_c = 245; // filament
        cur.machine_speed_mm_s = 120.0; // printer (datasheet)
        cur.bed_size_x_mm = 300.0; // printer

        let pc = ProcessProfile::diff(&cur, &base);
        assert_eq!(pc.wall_count, Some(5));
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

        // Save a filament diff inheriting petg with a hotter nozzle (245 °C,
        // over petg's 240).
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
    fn single_filament_resolve_tools_matches_resolve() {
        // The multi-tool path with one filament IS the classic resolve —
        // field for field, tools included.
        let p = Profiles::builtin();
        for (pr, fl) in [("sovol-zero", "pla"), ("voron24", "asa"), ("generic", "petg")] {
            let a = p.resolve(pr, fl, "standard").unwrap();
            let b = p.resolve_tools(pr, &[fl], "standard").unwrap();
            assert_eq!(a, b, "{pr}/{fl}");
            assert_eq!(a.tools.len(), 1);
            assert_eq!(a.tools[0].filament_name, fl);
        }
        assert!(p.resolve_tools("generic", &[], "standard").is_err(), "no empty loadout");
        assert!(p.resolve_tools("generic", &["pla", "nope"], "standard").is_err());
    }

    #[test]
    fn resolve_tools_populates_per_tool_settings() {
        let mut p = Profiles::builtin();
        p.printers.insert(
            "changer".into(),
            PrinterProfile {
                inherits: Some("generic".into()),
                tool_count: Some(4),
                toolchange_gcode: Some("TOOL_PICKUP T={tool}".into()),
                toolchange_seconds: Some(6.5),
                ..Default::default()
            },
        );
        p.filaments.insert(
            "red-pla".into(),
            FilamentProfile { inherits: Some("pla".into()), color: Some("#FF0000".into()), ..Default::default() },
        );
        p.filaments.insert(
            "mystery".into(),
            FilamentProfile { inherits: Some("petg".into()), color: Some("banana".into()), ..Default::default() },
        );
        let s = p.resolve_tools("changer", &["red-pla", "mystery"], "standard").unwrap();
        // The printer's toolchanger datasheet.
        assert_eq!(s.tool_count, 4);
        assert_eq!(s.toolchange_gcode, "TOOL_PICKUP T={tool}");
        assert_eq!(s.toolchange_seconds, 6.5);
        // Per-tool temps ride each slot's own filament.
        assert_eq!(s.tools.len(), 2);
        assert_eq!(s.tools[0].nozzle_temp_c, 207); // pla
        assert_eq!(s.tools[1].nozzle_temp_c, 255); // petg
        // Retraction distance rides each slot's own filament now.
        assert_eq!(s.tools[0].retract_len_mm, 0.5); // pla
        assert_eq!(s.tools[1].retract_len_mm, 0.6); // petg
        assert_eq!(s.retract_len_mm, s.tools[0].retract_len_mm); // flat mirrors tool 0
        assert_eq!(s.tools[1].first_layer_nozzle_temp_c, 255); // PETG: no bump (shreds hot)
        // Colors: parsed hex, garbage → the neutral fallback.
        assert_eq!(s.tools[0].color_rgb, [1.0, 0.0, 0.0]);
        assert_eq!(s.tools[1].color_rgb, crate::NEUTRAL_FILAMENT_RGB);
        // The flat view is tool 0's.
        assert_eq!(s.nozzle_temp_c, 207);
        assert_eq!(s.filament_color_rgb, [1.0, 0.0, 0.0]);
        // tool() clamps past the loadout instead of panicking.
        assert_eq!(s.tool(9).filament_name, "mystery");
        // An ordinary printer stays single-tool with the T{tool} default.
        let single = p.resolve("generic", "pla", "standard").unwrap();
        assert_eq!(single.tool_count, 1);
        assert_eq!(single.toolchange_gcode, "T{tool}");
        assert_eq!(single.toolchange_seconds, 10.0);
    }

    #[test]
    fn shared_bed_and_chamber_take_the_hottest_tool() {
        // Bed and chamber are shared hardware: the hottest tool's wish wins,
        // whatever slot it sits in; the flat filament view stays tool 0's.
        let p = Profiles::builtin();
        let s = p.resolve_tools("sovol-zero", &["pla", "asa"], "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 207); // flat = pla (tool 0)
        assert_eq!(s.bed_temp_c, 100); // asa's bed
        assert_eq!(s.chamber_temp_c, 40); // asa's soak (card-pinned)
        let s = p.resolve_tools("sovol-zero", &["asa", "pla"], "standard").unwrap();
        assert_eq!(s.nozzle_temp_c, 260); // flat = asa (tool 0)
        assert_eq!(s.bed_temp_c, 100);
        assert_eq!(s.chamber_temp_c, 40);
    }

    #[test]
    fn derived_speeds_clamp_to_the_slowest_tool() {
        // Any tool may print any feature, so the derived feature speeds work
        // under the minimum flow ceiling across the loadout — loading a
        // slow-melt filament in another slot drags them all down, even though
        // the flat filament fields stay tool 0's.
        let mut p = Profiles::builtin();
        p.filaments.insert(
            "slow-melt".into(),
            FilamentProfile {
                inherits: Some("pla".into()),
                max_volumetric_speed_mm3_s: Some(5.0),
                ..Default::default()
            },
        );
        let alone = p.resolve("sovol-zero", "pla", "standard").unwrap();
        let s = p.resolve_tools("sovol-zero", &["pla", "slow-melt"], "standard").unwrap();
        let cap = crate::flow_speed_cap_mm_s(5.0, s.line_width_mm, s.layer_height_mm);
        assert_eq!(s.max_volumetric_speed_mm3_s, 15.0); // flat = tool 0 (pla)
        assert_eq!(s.support_speed_mm_s, cap);
        assert_eq!(s.solid_speed_mm_s, cap);
        assert_eq!(s.external_perimeter_speed_mm_s, cap);
        assert!(s.support_speed_mm_s < alone.support_speed_mm_s);
    }

    #[test]
    fn toolchanger_fields_roundtrip_toml() {
        let pr = PrinterProfile {
            tool_count: Some(6),
            toolchange_gcode: Some("T{tool}\nM400".into()),
            toolchange_seconds: Some(8.0),
            ..Default::default()
        };
        let text = toml::to_string_pretty(&pr).unwrap();
        assert_eq!(toml::from_str::<PrinterProfile>(&text).unwrap(), pr);
        let fl = FilamentProfile { color: Some("#DDDDDD".into()), ..Default::default() };
        let text = toml::to_string_pretty(&fl).unwrap();
        assert_eq!(toml::from_str::<FilamentProfile>(&text).unwrap(), fl);
        // The plain profile-file form parses too.
        let pr: PrinterProfile = toml::from_str("tool_count = 4\ntoolchange_seconds = 2.5\n").unwrap();
        assert_eq!(pr.tool_count, Some(4));
        assert_eq!(pr.toolchange_seconds, Some(2.5));
    }

    #[test]
    fn toolchanger_and_color_fields_diff_to_their_tiers() {
        let base = Settings::default();
        let mut cur = base.clone();
        cur.tool_count = 4;
        cur.toolchange_gcode = "TOOL_PICKUP T={tool}".into();
        cur.toolchange_seconds = 3.0;
        cur.filament_color_rgb = [1.0, 0.0, 0.0];
        let pr = PrinterProfile::diff(&cur, &base);
        assert_eq!(pr.tool_count, Some(4));
        assert_eq!(pr.toolchange_gcode.as_deref(), Some("TOOL_PICKUP T={tool}"));
        assert_eq!(pr.toolchange_seconds, Some(3.0));
        // The color round-trips as the hex string a profile carries.
        let fl = FilamentProfile::diff(&cur, &base);
        assert_eq!(fl.color.as_deref(), Some("#FF0000"));
        assert!(FilamentProfile::diff(&base, &base).color.is_none());
    }

    #[test]
    fn diff_tool_tracks_per_slot_edits() {
        let base = Settings::default().flat_tool("pla".into());
        // Identical views: nothing worth saving.
        assert!(FilamentProfile::diff_tool(&base, &base).is_empty());
        // Edited temp / PA / fan land in the diff; untouched fields stay unset.
        let mut cur = base.clone();
        cur.nozzle_temp_c = 245;
        cur.pressure_advance = 0.05;
        cur.fan_speed = 0.35;
        let d = FilamentProfile::diff_tool(&cur, &base);
        assert_eq!(d.nozzle_temp_c, Some(245));
        assert_eq!(d.pressure_advance, Some(0.05));
        assert_eq!(d.fan_speed, Some(0.35));
        assert!(d.bed_temp_c.is_none(), "untouched fields stay unset");
        assert!(!d.is_empty());
        // Material compares as the label string, like `diff` does.
        let mut petg = base.clone();
        petg.material = crate::Material::Petg;
        assert_eq!(FilamentProfile::diff_tool(&petg, &base).material.as_deref(), Some("PETG"));
    }

    #[test]
    fn diff_tool_never_promotes_slot_data() {
        let base = Settings::default().flat_tool("pla".into());
        // A spool-color override alone is slot data, not profile data.
        let mut painted = base.clone();
        painted.color_rgb = [1.0, 0.0, 0.0];
        assert!(FilamentProfile::diff_tool(&painted, &base).is_empty());
        // The first-layer temperature is always derived — never diffed.
        let mut fl = base.clone();
        fl.first_layer_nozzle_temp_c += 15;
        assert!(FilamentProfile::diff_tool(&fl, &base).is_empty());
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
