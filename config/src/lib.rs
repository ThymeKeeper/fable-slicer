//! Settings for the slicer.
//!
//! [`Settings`] is the *resolved*, flat configuration the engine and g-code
//! emitter consume. The [`profile`] module builds one of these from tiered
//! printer / filament / process profiles (with inheritance), loaded from TOML.
//! `Settings::default()` is the in-code fallback used by tests and for any field
//! a profile leaves unset.

use std::f64::consts::PI;

mod profile;
pub use profile::{tier_dirty, FilamentProfile, PrinterProfile, ProcessProfile, Profiles, Tier, TierKind};
mod state;
pub use state::{config_dir, AppState};

/// Default start g-code (generic, heats + homes directly). `{placeholders}` are
/// substituted by the emitter. Used when a printer profile sets no `start_gcode`.
pub const GENERIC_START_GCODE: &str = "\
M140 S{bed_temp}
M104 S{first_layer_nozzle_temp}
M190 S{bed_temp}
M109 S{first_layer_nozzle_temp}
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

/// Material class from the spool's packaging — the data that drives every
/// filament-side default. The user types in what the box says (material,
/// temperature range, bed, diameter) and everything else derives from the
/// class until a calibration value pins it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Material {
    #[default]
    Pla,
    Petg,
    Abs,
    Tpu,
    /// Unknown material: conservative generic defaults.
    Other,
}

impl Material {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "pla" => Some(Self::Pla),
            "petg" | "pet" => Some(Self::Petg),
            "abs" | "asa" => Some(Self::Abs),
            "tpu" | "flex" => Some(Self::Tpu),
            "other" | "generic" => Some(Self::Other),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Pla => "PLA",
            Self::Petg => "PETG",
            Self::Abs => "ABS/ASA",
            Self::Tpu => "TPU",
            Self::Other => "other",
        }
    }
    /// Density (g/cm³) for the weight estimate.
    pub fn density_g_cm3(self) -> f64 {
        match self {
            Self::Pla => 1.24,
            Self::Petg => 1.27,
            Self::Abs => 1.04,
            Self::Tpu => 1.21,
            Self::Other => 1.24,
        }
    }
    /// Packaging temperature range fallback when the box wasn't entered.
    pub fn packaging_temp_c(self) -> (u32, u32) {
        match self {
            Self::Pla => (190, 220),
            Self::Petg => (230, 260),
            Self::Abs => (240, 270),
            Self::Tpu => (210, 240),
            Self::Other => (190, 220),
        }
    }
    pub fn bed_temp_c(self) -> u32 {
        match self {
            Self::Pla => 60,
            Self::Petg => 75,
            Self::Abs => 95,
            Self::Tpu => 40,
            Self::Other => 60,
        }
    }
    /// First-layer bump over the operating temperature (adhesion), clipped
    /// by the packaging max.
    pub fn first_layer_bump_c(self) -> u32 {
        match self {
            Self::Pla => 20,
            Self::Petg => 10,
            Self::Abs => 10,
            Self::Tpu => 5,
            Self::Other => 10,
        }
    }
    /// Part-fan duty (1.0 = 100%) and the layers to keep it off.
    pub fn fan(self) -> (f64, f64, usize) {
        // (fan, bridge fan, fan-off layers)
        match self {
            Self::Pla => (1.0, 1.0, 1),
            Self::Petg => (0.5, 0.8, 3),
            Self::Abs => (0.15, 0.5, 3),
            Self::Tpu => (0.7, 1.0, 1),
            Self::Other => (1.0, 1.0, 1),
        }
    }
    /// Melt ceiling (mm³/s) for a standard modern hotend — deliberately
    /// conservative; a measured value belongs in calibration.
    pub fn max_flow_mm3_s(self) -> f64 {
        match self {
            Self::Pla => 12.0,
            Self::Petg => 10.0,
            Self::Abs => 12.0,
            Self::Tpu => 4.0,
            Self::Other => 10.0,
        }
    }
    /// Flow-ceiling derate per °C below the operating temperature.
    pub fn max_flow_derate_per_c(self) -> f64 {
        match self {
            Self::Tpu => 0.15,
            _ => 0.3,
        }
    }
    /// Allowable heat-load ceiling (mW/mm², per island) for heat control.
    pub fn max_heat_mw_mm2(self) -> f64 {
        match self {
            Self::Pla => 15.0,
            Self::Petg => 13.0,
            Self::Abs => 20.0,
            Self::Tpu => 10.0,
            Self::Other => 15.0,
        }
    }
    /// Aux-fan and chamber-exhaust duties (machines that declare them).
    pub fn aux_exhaust(self) -> (f64, f64) {
        match self {
            Self::Pla => (0.75, 0.8),
            Self::Petg => (0.4, 0.4),
            Self::Abs => (0.1, 0.1),
            Self::Tpu => (0.3, 0.5),
            Self::Other => (0.5, 0.5),
        }
    }
    /// Chamber pre-soak target (°C; 0 = none) for machines that declare a
    /// chamber thermistor. ABS/ASA wants a warm chamber before the first
    /// layer (warping/splitting); PLA must NOT soak (heat creep, sag).
    pub fn chamber_temp_c(self) -> u32 {
        match self {
            Self::Abs => 50,
            _ => 0,
        }
    }
}

/// Where the start/end seam of each closed wall loop is placed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SeamMode {
    /// Rear-most point of each loop — seams align into a vertical column.
    #[default]
    Nearest,
    /// Sharpest real corner of each loop (concave preferred — the seam tucks
    /// into the notch), held in one column across layers; smooth loops with
    /// no corner worth chasing fall back to an aligned column instead of
    /// scattering over noise.
    Sharpest,
    /// Scattered per layer.
    Random,
    /// Each outer loop starts at the vertex nearest the previous layer's seam
    /// (seeded at the rear), so the seam follows one continuous line even
    /// where the rear-most vertex jumps between competing features.
    Aligned,
}

impl SeamMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "nearest" | "rear" => Some(Self::Nearest),
            "sharpest" | "sharp" | "corner" => Some(Self::Sharpest),
            "random" => Some(Self::Random),
            "aligned" => Some(Self::Aligned),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Nearest => "nearest",
            Self::Sharpest => "sharpest",
            Self::Random => "random",
            Self::Aligned => "aligned",
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
    /// Three sets of lines at 60° to each other.
    Triangles,
    /// Loops following the region boundary inward.
    Concentric,
    /// The gyroid minimal surface's level set — strong in every direction,
    /// self-crossing-free per layer, and printable without retractions.
    Gyroid,
}

impl InfillPattern {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "lines" | "line" | "rectilinear" => Some(Self::Lines),
            "grid" => Some(Self::Grid),
            "triangles" | "triangle" => Some(Self::Triangles),
            "concentric" => Some(Self::Concentric),
            "gyroid" => Some(Self::Gyroid),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Lines => "lines",
            Self::Grid => "grid",
            Self::Triangles => "triangles",
            Self::Concentric => "concentric",
            Self::Gyroid => "gyroid",
        }
    }
}

/// How wall toolpaths are generated.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WallMode {
    /// Variable-width beading (Arachne-class): inner walls stretch, squeeze,
    /// split and merge with the local feature thickness; thin features become
    /// single tapered beads. The outer wall stays a fixed-width exact loop.
    /// Validated on Benchy preview review 2026-06-09 (contiguity, junctions,
    /// centerline tracking); grid-based with an exact-skeleton upgrade path.
    #[default]
    Arachne,
    /// Fixed-width concentric offsets everywhere (gaps go to gap fill).
    Classic,
}

impl WallMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "arachne" | "variable" | "adaptive" => Some(Self::Arachne),
            "classic" | "fixed" | "concentric" => Some(Self::Classic),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Arachne => "arachne",
            Self::Classic => "classic",
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
    /// Acceleration (mm/s²) for everything not listed below — inner walls,
    /// infill, solid fill, support, travel. Emitted as M204 and used by the
    /// time estimate. (Klipper clamps to printer.cfg `max_accel`.)
    pub acceleration_mm_s2: f64,
    /// Acceleration (mm/s²) for the outermost wall — lower hides ringing on
    /// the visible surface. Auto-derives as half the main acceleration.
    pub outer_wall_accel_mm_s2: f64,
    /// Acceleration (mm/s²) for the whole first layer — gentle for adhesion.
    /// Auto-derives as min(1000, main acceleration).
    pub first_layer_accel_mm_s2: f64,
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
    /// Fit circular arcs to curved toolpath runs and emit G2/G3 (smaller g-code,
    /// smoother motion). Needs firmware arc support (Klipper `[gcode_arcs]`).
    pub arc_fitting: bool,
    /// Max deviation (mm) a point may have from a fitted arc to be folded into it.
    pub arc_tolerance_mm: f64,
    pub wall_count: usize,
    /// Wall generation: variable-width (arachne) or fixed concentric (classic).
    pub wall_mode: WallMode,
    pub top_layers: usize,
    pub bottom_layers: usize,
    /// Print the outer wall as two half-height passes per layer, each sliced at
    /// its own plane — halves the visible Z staircase on sloped surfaces while
    /// the interior keeps the full layer height. Mutually exclusive with brick
    /// layering (their Z choreographies collide).
    pub half_height_outer_walls: bool,
    /// Brick layering: stagger odd-indexed perimeters by half a layer height so
    /// adjacent wall rings interlock (the outer wall stays put). The lifted
    /// beads' extra flow is DERIVED — see [`brick_flow_factor`].
    pub brick_layers: bool,
    /// Sparse infill density, 0.0..=1.0 (0 disables sparse infill).
    pub infill_density: f64,
    /// Pattern for sparse (interior) infill.
    pub sparse_pattern: InfillPattern,
    /// Pattern for solid (top/bottom) interior infill.
    pub solid_pattern: InfillPattern,
    /// How far infill lines push into the innermost wall bead, as a fraction of
    /// the line width (0..~0.5). A little overlap bonds infill to the walls.
    pub infill_overlap: f64,
    /// Print solid-fill lines in monotonic order (strict sweep across each
    /// region) so top surfaces get an even sheen without overlap ridges.
    pub monotonic_solid: bool,
    /// Fill gaps too thin for normal infill (between/inside walls) with single
    /// width-matched strokes.
    pub gap_fill: bool,
    /// Jitter external perimeters for a rough "fuzzy" surface texture.
    pub fuzzy_skin: bool,
    /// Total jitter band (mm) for fuzzy skin, centered on the wall line.
    pub fuzzy_skin_thickness_mm: f64,
    /// Approximate spacing (mm) between fuzzy-skin jitter points.
    pub fuzzy_skin_point_dist_mm: f64,
    /// Iron top surfaces: re-traverse them with a hot nozzle and a trickle of
    /// flow to melt ridges flat.
    pub ironing: bool,
    /// Ironing extrusion as a fraction of a normal line's flow at that spacing.
    pub ironing_flow: f64,
    /// Spacing (mm) between ironing passes.
    pub ironing_spacing_mm: f64,
    /// Ironing speed (mm/s).
    pub ironing_speed_mm_s: f64,
    /// Shrink the first layer's outline inward by this much (mm) to counter
    /// first-layer squish ("elephant foot"). 0 disables.
    pub elephant_foot_mm: f64,
    /// Grow (+) or shrink (−) every layer's outline by this much (mm) to dial in
    /// dimensional accuracy. 0 disables.
    pub xy_compensation_mm: f64,
    /// Spiral-vase mode: one continuously rising outer wall, no infill or top
    /// shells above the solid bottom. Forces 1 wall / 0% infill / no supports.
    pub spiral_vase: bool,
    /// Number of skirt loops around the first layer (0 disables).
    pub skirt_loops: usize,
    /// Gap between the skirt and the model (mm).
    pub skirt_gap_mm: f64,
    /// Number of brim loops extending outward from the part (0 disables).
    pub brim_loops: usize,
    /// Where to place the wall seam.
    pub seam_mode: SeamMode,
    /// Auto-center the model on the bed before slicing. The GUI positions objects
    /// explicitly (multi-object layout) so it turns this off; the CLI keeps it on.
    pub auto_center_on_bed: bool,

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
    /// How far (mm) arc-overhang fans overlap where they meet (a little helps them
    /// mesh; too much over-extrudes the seam). Per fan, so the join is ~2× this.
    pub arc_seam_overlap_mm: f64,

    // --- retraction ---
    pub retract_len_mm: f64,
    pub retract_speed_mm_s: f64,
    /// Z lift on travels that can't be combed (cross a void). 0 disables.
    pub z_hop_mm: f64,
    /// After retracting, wipe the nozzle back along the just-printed path by
    /// this much before travelling — the ooze smears over existing plastic
    /// instead of blobbing the seam. 0 disables.
    pub wipe_mm: f64,

    // --- printer connection (Moonraker/Klipper API server) ---
    /// Printer address — `voron24.local`, an IP, or a full URL; empty = not
    /// configured. Plain HTTP is assumed when no scheme is given (LAN norm).
    pub host_url: String,
    /// Moonraker API key, only when its `[authorization]` requires one.
    pub api_key: String,

    // --- temperatures (°C) ---
    /// The material class off the spool's box — drives every filament-side
    /// default until calibration pins one.
    pub material: Material,
    /// Operating nozzle °C — DERIVED: the packaging range's center, shifted
    /// by `temp_bias`. Never a slider; heat control owns it from there.
    pub nozzle_temp_c: u32,
    /// First-layer nozzle °C — DERIVED: operating + the class's adhesion
    /// bump, clipped by the packaging max.
    pub first_layer_nozzle_temp_c: u32,
    /// The packaging range printed on the box: heat control's full authority
    /// — schedules never leave [min, max].
    pub nozzle_temp_min_c: u32,
    pub nozzle_temp_max_c: u32,
    /// Cold ↔ hot preference (−1..+1): where in the packaging range the
    /// operating point sits. Cold = matte/dimensional, hot = glossy, strong
    /// bonding, more flow headroom.
    pub temp_bias: f64,
    pub bed_temp_c: u32,
    /// Hotend heating rate near printing temps (°C/s) — sets how early
    /// the temp schedule must issue warming M104s. Conservatively low until
    /// measured (a Moonraker calibration routine will fill it in).
    pub heat_rate_c_s: f64,
    /// Passive cooling rate near printing temps (°C/s) — far slower than
    /// heating; sets the long lead times for cooling into a zone.
    pub cool_rate_c_s: f64,
    /// Heating rate with the part fan at 100% (°C/s) — fan spillover steals
    /// heater power, so this is ≤ `heat_rate_c_s`. Auto: follows the fan-off
    /// rate until measured. The temp scheduler interpolates between the
    /// off/on pairs by the filament's fan duty.
    pub heat_rate_fan_c_s: f64,
    /// Cooling rate with the part fan at 100% (°C/s) — the realistic in-print
    /// case, faster than passive. Auto: follows the fan-off rate until measured.
    pub cool_rate_fan_c_s: f64,

    // --- speeds (mm/s) ---
    /// The machine's rated print speed (printer datasheet) — the hard cap
    /// the derived speeds work under. Lower it to slow the whole machine.
    pub machine_speed_mm_s: f64,
    /// Finish ↔ speed preference (−1..+1): the one intent dial. Scales the
    /// derived nominal speed between 60% and 100% of the machine rating.
    pub speed_quality: f64,
    /// Nominal print speed — DERIVED: machine rating × the quality factor,
    /// then each feature derives from it under the filament's flow ceiling,
    /// and heat control governs the result. Never a slider.
    pub print_speed_mm_s: f64,
    pub travel_speed_mm_s: f64,
    pub first_layer_speed_mm_s: f64,
    /// Speed (mm/s) for the outermost wall — slow for surface quality.
    pub external_perimeter_speed_mm_s: f64,
    /// Speed (mm/s) for solid top/bottom fill.
    pub solid_speed_mm_s: f64,
    /// Speed (mm/s) for support structure.
    pub support_speed_mm_s: f64,
    /// Speed (mm/s) for gap-fill strokes — slow, they sit in tight corners.
    pub gap_fill_speed_mm_s: f64,
    /// Speed (mm/s) for straight bridges (spans anchored on both sides).
    /// Arc overhangs derive ~30% of this, clamped to 5–15 mm/s — each arc
    /// cantilevers off the previous ring, far more delicate than a bridge.
    pub bridge_speed_mm_s: f64,
    /// Speed (mm/s) for wall stretches that overhang the layer below by more
    /// than half a bead — slow so the unsupported side cools in place.
    /// Auto-derives from the bridge speed (same physics: printing onto air).
    pub overhang_speed_mm_s: f64,
    /// Minimum time per layer (s); thin layers are slowed to allow cooling.
    pub min_layer_time_s: f64,
    /// Floor speed (mm/s) when slowing for min-layer-time.
    pub min_print_speed_mm_s: f64,

    // --- flow ---
    /// Hard ceiling on volumetric flow (mm³/s) — the filament's melt rate
    /// through the hotend. Per-feature speeds are clamped so
    /// `width × height × speed × flow` never exceeds it (loudly: the g-code
    /// header, CLI, and GUI all report what got clamped). ≤ 0 disables.
    pub max_volumetric_speed_mm3_s: f64,
    /// How much of that ceiling is lost per °C below `nozzle_temp_c`
    /// (mm³/s/°C): when heat control cools a zone, its flow cap — and
    /// therefore its clamped speeds — derate by this. Never raised on warmer
    /// layers (the profile cap is the calibrated number).
    pub max_flow_derate_per_c: f64,
    /// Global extrusion multiplier (filament-specific flow tuning). 1.0 = nominal.
    pub extrusion_multiplier: f64,
    /// Flow multiplier for bridges and arc overhangs (slight under-extrusion can
    /// tighten sagging strands). 1.0 = nominal.
    pub bridge_flow: f64,
    /// Klipper pressure advance, emitted as SET_PRESSURE_ADVANCE after the start
    /// g-code when > 0. 0 leaves the printer's configured value untouched.
    pub pressure_advance: f64,

    // --- cooling ---
    /// Part-cooling fan duty for normal printing, 0.0..=1.0.
    pub fan_speed: f64,
    /// Fan duty while printing bridges / arc overhangs (usually maxed).
    pub bridge_fan_speed: f64,
    /// Keep the fan off for this many initial layers (adhesion).
    pub fan_off_layers: usize,
    /// The machine has an auxiliary part-cooling fan addressed as `M106 P2`
    /// (Sovol Zero / Bambu-style side fan). Gates all P2 emission: vanilla
    /// Klipper/Marlin would read the P-form as the primary fan.
    pub has_aux_fan: bool,
    /// Aux-fan duty 0.0..=1.0, flat once past `fan_off_layers`. 0 = off.
    pub aux_fan_speed: f64,
    /// The machine has a chamber-exhaust fan addressed as `M106 P3`.
    pub has_exhaust_fan: bool,
    /// Exhaust duty 0.0..=1.0 for the whole print — vents chamber heat
    /// (PLA wants it high, ABS low or zero). 0 = off.
    pub exhaust_fan_speed: f64,
    /// The machine's chamber thermistor, by its Klipper `temperature_sensor`
    /// name (e.g. "chamber_temp" on the Sovol Zero, "chamber" on a Voron).
    /// Empty = no sensor; gates all chamber pre-soak emission.
    pub chamber_sensor: String,
    /// Chamber pre-soak (°C, 0 = off): after the start g-code — bed already
    /// hot, radiating into the chamber — emit a `TEMPERATURE_WAIT` on the
    /// chamber sensor before printing. Auto: the material class's value
    /// (ABS/ASA soak at 50, everything else 0).
    pub chamber_temp_c: u32,
    /// Heat control, the automatic: keep every island's heat load inside the
    /// filament's allowable ranges and smooth layer-to-layer transitions —
    /// the banding/shrinkage killer — spending at most
    /// `smooth_extra_time_pct` extra print time. One gradient-limited target
    /// curve is derived per print (the gentlest the budget affords; the
    /// achieved %/layer is reported) and both levers serve it with no
    /// per-lever knobs: the nozzle-temperature schedule warms cold dips and
    /// cools hot ranges inside the filament window (free in print time;
    /// ramps lead per the printer's measured rates, plain async M104s, no
    /// live feedback; cooled layers derate the flow ceiling via
    /// `max_flow_derate_per_c`), and per-island slowing plus park-and-wait
    /// dwells supply what temperature can't reach — never below
    /// `min_print_speed_mm_s` without saying so. ON BY DEFAULT — it is part
    /// of the derived surface, not an opt-in extra; a profile may still set
    /// `heat_control = false` to print raw derived speeds and temperatures.
    pub heat_control: bool,
    /// The filament's allowable heat-load ceiling (mW/mm², per island) — a
    /// material range bound, not a tuning target. Auto: the material class's
    /// value; a calibration entry pins it. Heat control's temperature
    /// authority is the packaging range itself.
    pub max_heat_mw_mm2: f64,
    /// Heat control's time budget: extra print time it may spend, as a % of
    /// the un-smoothed estimate. The transition-gradient limit is bisected
    /// to the gentlest that fits; 0 still does everything that's free
    /// (warming cold dips, capping at the ceiling).
    pub smooth_extra_time_pct: f64,

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
            outer_wall_accel_mm_s2: derived_outer_wall_accel_mm_s2(3000.0),
            first_layer_accel_mm_s2: derived_first_layer_accel_mm_s2(3000.0),
            jerk_mm_s: 10.0,
            layer_height_mm: 0.2,
            first_layer_height_mm: 0.2,
            line_width_mm: 0.45,
            max_resolution_mm: 0.05,
            arc_fitting: false,
            arc_tolerance_mm: 0.05,
            wall_count: 2,
            wall_mode: WallMode::default(),
            half_height_outer_walls: false,
            brick_layers: false,
            top_layers: 4,
            bottom_layers: 4,
            infill_density: 0.15,
            sparse_pattern: InfillPattern::default(),
            solid_pattern: InfillPattern::default(),
            infill_overlap: 0.25,
            monotonic_solid: true,
            gap_fill: true,
            fuzzy_skin: false,
            fuzzy_skin_thickness_mm: 0.3,
            fuzzy_skin_point_dist_mm: 0.8,
            ironing: false,
            ironing_flow: 0.15,
            ironing_spacing_mm: 0.15,
            ironing_speed_mm_s: 30.0,
            elephant_foot_mm: 0.0,
            xy_compensation_mm: 0.0,
            spiral_vase: false,
            skirt_loops: 2,
            skirt_gap_mm: 3.0,
            brim_loops: 0,
            seam_mode: SeamMode::default(),
            auto_center_on_bed: true,
            support_mode: SupportMode::default(),
            support_overhang_angle_deg: 45.0,
            support_density: 0.12,
            support_xy_clearance_mm: 0.4,
            support_z_gap_layers: 1,
            support_interface_layers: 2,
            max_bridge_span_mm: 6.0,
            max_arc_radius_mm: 40.0,
            arc_seam_overlap_mm: 0.1,
            retract_len_mm: 0.8,
            retract_speed_mm_s: 35.0,
            z_hop_mm: 0.0,
            wipe_mm: 2.0,
            host_url: String::new(),
            api_key: String::new(),
            material: Material::Pla,
            nozzle_temp_c: derived_nozzle_temp_c(185, 215, 0.0),
            first_layer_nozzle_temp_c: derived_first_layer_temp_c(185, 215, 0.0, Material::Pla),
            nozzle_temp_min_c: 185,
            nozzle_temp_max_c: 215,
            temp_bias: 0.0,
            heat_rate_c_s: 2.0,
            cool_rate_c_s: 0.7,
            heat_rate_fan_c_s: 2.0,
            cool_rate_fan_c_s: 0.7,
            bed_temp_c: 60,
            machine_speed_mm_s: 60.0,
            speed_quality: 0.0,
            print_speed_mm_s: derived_print_speed_mm_s(60.0, 0.0),
            travel_speed_mm_s: 120.0,
            first_layer_speed_mm_s: 20.0,
            external_perimeter_speed_mm_s: 25.0,
            solid_speed_mm_s: 40.0,
            support_speed_mm_s: 45.0,
            gap_fill_speed_mm_s: 20.0,
            bridge_speed_mm_s: 50.0,
            overhang_speed_mm_s: derived_overhang_speed_mm_s(50.0),
            min_layer_time_s: 8.0,
            min_print_speed_mm_s: 10.0,
            max_volumetric_speed_mm3_s: 15.0,
            max_flow_derate_per_c: 0.3,
            extrusion_multiplier: 1.0,
            bridge_flow: 1.0,
            pressure_advance: 0.0,
            fan_speed: 1.0,
            bridge_fan_speed: 1.0,
            fan_off_layers: 1,
            has_aux_fan: false,
            aux_fan_speed: 0.0,
            has_exhaust_fan: false,
            exhaust_fan_speed: 0.0,
            chamber_sensor: String::new(),
            chamber_temp_c: Material::Pla.chamber_temp_c(),
            heat_control: true,
            // Calibrated on the Benchy: lone towers / chimneys / arch pillars
            // run 20+ mW/mm², cabin-class thin walls ~13, hulls < 10.
            max_heat_mw_mm2: Material::Pla.max_heat_mw_mm2(),
            smooth_extra_time_pct: 10.0,
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

// --- auto-derived defaults ---------------------------------------------------
// One source of truth for the "auto" values: `Profiles::resolve` uses these
// when a profile leaves the field unset, and the GUI recomputes them live for
// unpinned fields (so dragging the master visibly moves its dependents).

/// Auto line width: 112.5% of the nozzle bore — wide enough to squeeze a solid
/// bead, narrow enough to hold detail (0.4 mm nozzle → 0.45 mm).
/// The operating nozzle temperature: the packaging range's center shifted by
/// the cold↔hot bias (−1 = the box's minimum, +1 = its maximum).
pub fn derived_nozzle_temp_c(min_c: u32, max_c: u32, bias: f64) -> u32 {
    let (lo, hi) = (min_c.min(max_c) as f64, max_c.max(min_c) as f64);
    (lo + (hi - lo) * (0.5 + 0.5 * bias.clamp(-1.0, 1.0))).round() as u32
}

/// First-layer temperature: operating + the material's adhesion bump,
/// clipped by the packaging max.
pub fn derived_first_layer_temp_c(min_c: u32, max_c: u32, bias: f64, material: Material) -> u32 {
    (derived_nozzle_temp_c(min_c, max_c, bias) + material.first_layer_bump_c())
        .min(max_c.max(min_c))
}

/// Nominal print speed: the machine's rated speed × the finish↔speed factor
/// (−1 → 60%, 0 → 80%, +1 → 100%).
pub fn derived_print_speed_mm_s(machine_speed_mm_s: f64, speed_quality: f64) -> f64 {
    machine_speed_mm_s * (0.8 + 0.2 * speed_quality.clamp(-1.0, 1.0))
}

pub fn derived_line_width_mm(nozzle_diameter_mm: f64) -> f64 {
    nozzle_diameter_mm * 1.125
}

/// The flow triangle's speed bound: the fastest feed (mm/s) at which a
/// `line_width × layer_height` bead still fits under the filament's melt
/// ceiling (mm³/s). Auto speeds balance against this, so the slice-time
/// volumetric clamp never has to quietly slow a derived value — it only
/// fires for pinned or master-driven speeds. Unlimited when the ceiling is 0.
pub fn flow_speed_cap_mm_s(max_flow_mm3_s: f64, line_width_mm: f64, layer_height_mm: f64) -> f64 {
    if max_flow_mm3_s <= 0.0 {
        return f64::INFINITY;
    }
    max_flow_mm3_s / bead_area_mm2(line_width_mm, layer_height_mm)
}

/// Auto outer-wall speed: half the machine's print speed, for surface
/// quality; never past the flow cap.
pub fn derived_external_perimeter_speed_mm_s(print_speed_mm_s: f64, flow_cap_mm_s: f64) -> f64 {
    (print_speed_mm_s * 0.5).min(flow_cap_mm_s)
}

/// Auto solid-fill speed: 80% of print speed, never past the flow cap.
pub fn derived_solid_speed_mm_s(print_speed_mm_s: f64, flow_cap_mm_s: f64) -> f64 {
    (print_speed_mm_s * 0.8).min(flow_cap_mm_s)
}

/// Auto support speed: 90% of print speed (surface quality doesn't matter),
/// never past the flow cap.
pub fn derived_support_speed_mm_s(print_speed_mm_s: f64, flow_cap_mm_s: f64) -> f64 {
    (print_speed_mm_s * 0.9).min(flow_cap_mm_s)
}

/// Auto gap-fill speed: 40% of print speed, capped — gap strokes live in tight
/// corners where the head is always turning; never past the flow cap.
pub fn derived_gap_fill_speed_mm_s(print_speed_mm_s: f64, flow_cap_mm_s: f64) -> f64 {
    (print_speed_mm_s * 0.4).min(40.0).min(flow_cap_mm_s)
}

/// Auto overhang-wall speed: same as bridges — both lay beads onto air.
pub fn derived_overhang_speed_mm_s(bridge_speed_mm_s: f64) -> f64 {
    bridge_speed_mm_s
}

/// Auto outer-wall acceleration: half the main acceleration — gentle direction
/// changes on the visible surface hide ringing.
pub fn derived_outer_wall_accel_mm_s2(acceleration_mm_s2: f64) -> f64 {
    (acceleration_mm_s2 * 0.5).max(500.0)
}

/// Auto first-layer acceleration: capped at 1000 mm/s² so the squished first
/// beads aren't sheared off the bed.
pub fn derived_first_layer_accel_mm_s2(acceleration_mm_s2: f64) -> f64 {
    acceleration_mm_s2.min(1000.0)
}

/// Cross-section area (mm²) of a deposited bead: a **stadium** — a flat core
/// with semicircular caps on the smaller dimension (a circle when w == h).
/// This is the physical bead shape; the rectangle model it replaces over-fed
/// by the cap-corner area (~9.5% at 0.45 × 0.2).
pub fn bead_area_mm2(width_mm: f64, height_mm: f64) -> f64 {
    let a = width_mm.min(height_mm);
    let b = width_mm.max(height_mm);
    a * (b - a) + PI * a * a / 4.0
}

/// Centerline distance (mm) at which adjacent beads fuse into a watertight
/// surface: the rounded shoulders overlap exactly enough to fill the cusps
/// between them. Area-exact by construction (`area / spacing / height = 1`),
/// which also makes `spacing / density` preserve density semantics for sparse
/// fills. For the usual w ≥ h this is `w − h·(1 − π/4)`.
pub fn bead_spacing_mm(width_mm: f64, height_mm: f64) -> f64 {
    bead_area_mm2(width_mm, height_mm) / height_mm.max(1.0e-9)
}

/// Flow multiplier for a brick-lifted bead — derived from the stadium model,
/// no knob. Aligned columns are spaced ([`bead_spacing_mm`]) so the lens
/// overlap of facing cap circles exactly feeds the cusps and the wall tiles
/// watertight. Lifting a column half a layer splits its flank contact: the
/// bead now meets two neighbours diagonally, and the two diagonal lenses
/// (cap-centre distance √(d² + (h/2)²), d = πh/4 the facing distance at
/// design spacing) sum to less than the aligned lens — that shortfall, on
/// both flanks, is real unfilled void the lifted bead must carry as extra
/// material. At 0.45 × 0.2 this derives 1.057 — right where hand-tuning had
/// settled (1.05).
pub fn brick_flow_factor(line_width_mm: f64, layer_height_mm: f64) -> f64 {
    let (w, h) = (line_width_mm, layer_height_mm);
    if w <= 0.0 || h <= 0.0 {
        return 1.0;
    }
    let r = h / 2.0;
    // Overlap area of two radius-r circles with centres `c` apart.
    let lens = |c: f64| -> f64 {
        if c >= 2.0 * r {
            return 0.0;
        }
        2.0 * r * r * (c / (2.0 * r)).acos() - (c / 2.0) * (4.0 * r * r - c * c).sqrt()
    };
    let d = std::f64::consts::FRAC_PI_4 * h;
    let aligned = lens(d);
    let staggered = 2.0 * lens((d * d + r * r).sqrt());
    let deficit_both_flanks = 2.0 * (aligned - staggered).max(0.0);
    1.0 + deficit_both_flanks / bead_area_mm2(w, h).max(1.0e-9)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brick_flow_derives_from_the_bead_geometry() {
        // 0.45 × 0.2: the diagonal-lens shortfall derives ≈ 1.057 — right on
        // top of the value hand-tuning had settled at (1.05).
        let f = brick_flow_factor(0.45, 0.2);
        assert!((f - 1.057).abs() < 0.005, "derived {f}");
        // Taller beads have rounder flanks and lose more diagonal contact —
        // the factor must grow with height and shrink with width.
        assert!(brick_flow_factor(0.45, 0.28) > f);
        assert!(brick_flow_factor(0.6, 0.2) < f);
        // Degenerate inputs stay sane.
        assert_eq!(brick_flow_factor(0.0, 0.2), 1.0);
        assert!(brick_flow_factor(0.2, 0.2) >= 1.0);
    }

    #[test]
    fn stadium_bead_math() {
        // 0.45 × 0.2 bead: A = 0.2·0.25 + π·0.04/4 = 0.0814 mm²,
        // spacing = 0.45 − 0.2·(1 − π/4) ≈ 0.4071 mm.
        let a = bead_area_mm2(0.45, 0.2);
        assert!((a - 0.081_416).abs() < 1.0e-5, "area {a}");
        let sp = bead_spacing_mm(0.45, 0.2);
        assert!((sp - 0.407_08).abs() < 1.0e-4, "spacing {sp}");
        // Square bead degenerates to a circle; spacing stays positive.
        let c = bead_area_mm2(0.2, 0.2);
        assert!((c - PI * 0.01).abs() < 1.0e-9, "circle {c}");
        // Solid surfaces are exactly dense: area / (spacing × height) = 1.
        assert!((a / (sp * 0.2) - 1.0).abs() < 1.0e-12);
        // Narrower-than-tall (gap-fill strokes) stays positive and sane.
        assert!(bead_area_mm2(0.12, 0.2) > 0.0);
    }
}
