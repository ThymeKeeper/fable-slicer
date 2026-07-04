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
pub use state::{config_dir, AppState, BlendState, Loadout};

/// Default start g-code. `{placeholders}` are substituted by the emitter; used
/// when a printer profile sets no `start_gcode`. The order mirrors the presoak
/// strategy — bed → home → chamber soak (`{chamber_soak}`, empty unless the
/// filament wants one) → nozzle — so the nozzle reaches temp last and never
/// idles hot over the bed (during homing, or while the chamber catches up).
pub const GENERIC_START_GCODE: &str = "\
M140 S{bed_temp}
M190 S{bed_temp}
G28
{chamber_soak}
M104 S{first_layer_nozzle_temp}
M109 S{first_layer_nozzle_temp}";

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
    /// Operating nozzle °C fallback when a profile doesn't set one.
    pub fn nozzle_temp_c(self) -> u32 {
        match self {
            Self::Pla => 205,
            Self::Petg => 245,
            Self::Abs => 255,
            Self::Tpu => 225,
            Self::Other => 205,
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

/// How a multi-tool machine changes filament — described by its mechanism, not
/// any product name. Two things actually vary and drive the g-code: whether
/// there is one shared heater or a heater per tool (temperature handling), and
/// whether the filaments share a melt zone that must be purged (waste).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MachineKind {
    /// Independent hotends, one per tool — each its own nozzle, filament, and
    /// live heater. A change docks one head and picks up another; nothing is
    /// purged (every color stays in its own tip).
    #[default]
    IndependentHotends,
    /// One nozzle fed by many filaments through a selector. A change unloads the
    /// old filament, loads the new, and PURGES the old color out through the
    /// shared melt zone. One heater, ramping between materials.
    SharedNozzle,
    /// Separate nozzles (one per filament, so nothing to purge) that take turns
    /// in ONE shared heater. A change indexes the next nozzle into the heater
    /// and waits for it to reach temperature. One heater, no flush.
    SharedHeater,
}

impl MachineKind {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "independent-hotends" | "independent" | "toolchanger" | "changer" => {
                Some(Self::IndependentHotends)
            }
            "shared-nozzle" | "shared_nozzle" | "mmu" => Some(Self::SharedNozzle),
            "shared-heater" | "shared_heater" | "indexed-nozzles" | "indexed" => {
                Some(Self::SharedHeater)
            }
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::IndependentHotends => "independent-hotends",
            Self::SharedNozzle => "shared-nozzle",
            Self::SharedHeater => "shared-heater",
        }
    }
    /// User-facing dropdown label (mechanism, no brands).
    pub fn display(self) -> &'static str {
        match self {
            Self::IndependentHotends => "Independent hotends, dock & swap",
            Self::SharedNozzle => "Shared nozzle, flush per swap",
            Self::SharedHeater => "Separate nozzles + shared heater, no flush",
        }
    }
}

/// Infill pattern for a region.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum InfillPattern {
    /// Parallel lines (rectilinear), alternating direction per layer for
    /// cross-hatching.
    #[default]
    Lines,
    /// Parallel lines at one fixed direction every layer — they stack into
    /// continuous walls (strong and fast along Z, weak across the lines).
    AlignedLines,
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
            "aligned" | "aligned-lines" | "aligned_lines" | "aligned lines" => {
                Some(Self::AlignedLines)
            }
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
            Self::AlignedLines => "aligned lines",
            Self::Grid => "grid",
            Self::Triangles => "triangles",
            Self::Concentric => "concentric",
            Self::Gyroid => "gyroid",
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
}

impl SupportMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "none" | "off" => Some(Self::None),
            "grid" | "normal" | "on" => Some(Self::Grid),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Grid => "grid",
        }
    }
}

/// One tool slot's resolved filament settings on a toolchanger — a mirror of
/// [`Settings`]' filament-tier fields. `Settings::tools` always holds at least
/// one; tool 0 mirrors the flat fields.
#[derive(Clone, Debug, PartialEq)]
pub struct ToolSettings {
    /// The filament profile loaded in this slot.
    pub filament_name: String,
    /// Display color for preview/part tinting.
    pub color_rgb: [f32; 3],
    pub material: Material,
    pub filament_diameter_mm: f64,
    pub filament_density_g_cm3: f64,
    pub nozzle_temp_c: u32,
    pub first_layer_nozzle_temp_c: u32,
    pub bed_temp_c: u32,
    pub max_volumetric_speed_mm3_s: f64,
    pub max_flow_derate_per_c: f64,
    pub extrusion_multiplier: f64,
    pub pressure_advance: f64,
    pub bridge_flow: f64,
    pub bridge_speed_mm_s: f64,
    pub fan_speed: f64,
    pub bridge_fan_speed: f64,
    pub fan_off_layers: usize,
    pub aux_fan_speed: f64,
    pub exhaust_fan_speed: f64,
    pub chamber_temp_c: u32,
    /// Nozzle setpoint while this tool sits docked long enough to matter —
    /// hot enough to restart quickly, cool enough not to ooze and cook.
    pub standby_temp_c: u32,
}

/// Fully-resolved settings the pipeline runs on.
#[derive(Clone, Debug, PartialEq)]
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
    /// Minimum cruise ratio (Klipper accel-to-decel smoothing): the fraction of
    /// each move that must cruise rather than accelerate-then-decelerate. 0 = off
    /// (fastest, spikier); higher = smoother/quieter, slower on short moves.
    /// Emitted as `SET_VELOCITY_LIMIT ACCEL_TO_DECEL = accel × (1 − ratio)`.
    pub min_cruise_ratio: f64,

    // --- process ---
    pub layer_height_mm: f64,
    /// Thickness of the first layer (often thicker for bed adhesion).
    pub first_layer_height_mm: f64,
    pub line_width_mm: f64,
    /// Fit circular arcs to curved toolpath runs and emit G2/G3 (smaller g-code,
    /// smoother motion). Needs firmware arc support (Klipper `[gcode_arcs]`).
    pub arc_fitting: bool,
    /// Max deviation (mm) a point may have from a fitted arc to be folded into it.
    pub arc_tolerance_mm: f64,
    pub wall_count: usize,
    /// Print each island's outer wall FIRST (before its inner walls) for crisper
    /// overhang edges, instead of last (inner walls first) — the default, which
    /// backs the outer wall against solid for the best flat-surface finish.
    pub outer_wall_first: bool,
    pub top_layers: usize,
    pub bottom_layers: usize,
    /// Sparse infill density, 0.0..=1.0 (0 disables sparse infill).
    pub infill_density: f64,
    /// Pattern for sparse (interior) infill.
    pub sparse_pattern: InfillPattern,
    /// Pattern for the top skin (the visible top surface) layers.
    pub top_pattern: InfillPattern,
    /// Pattern for the bottom skin (the visible bottom surface) layers.
    pub bottom_pattern: InfillPattern,
    /// Pattern for buried solid infill (between the sparse infill and the skins).
    pub solid_pattern: InfillPattern,
    /// How far infill lines push into the innermost wall bead, as a fraction of
    /// the line width (0..~0.5). A little overlap bonds infill to the walls.
    pub infill_overlap: f64,
    /// Print solid-fill lines in monotonic order (strict sweep across each
    /// region) so top surfaces get an even sheen without overlap ridges.
    pub monotonic_solid: bool,
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
    /// A bridge (supported on ≥2 sides) up to this span (mm) is filled with straight
    /// anchored bridge lines across the gap. Wider gaps fall back to the ordered
    /// bottom shell.
    pub max_bridge_span_mm: f64,
    /// Tallest dither repeat (mm) a blend may have and still fuse into one
    /// perceived color at viewing distance. Bounds the blend picker's gamut:
    /// weights quantize to whole layers of a `blend_band_mm / layer_height`
    /// cycle, so extreme ratios (one layer in ten = a visible stripe every
    /// 2 mm) are simply not offered.
    pub blend_band_mm: f64,

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
    /// Operating nozzle °C — set directly, held fixed for the whole print.
    pub nozzle_temp_c: u32,
    /// First-layer nozzle °C — DERIVED: operating + the material class's
    /// adhesion bump, clipped to the hotend ceiling.
    pub first_layer_nozzle_temp_c: u32,
    pub bed_temp_c: u32,

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
    /// Speed (mm/s) for straight bridges (spans anchored on both sides).
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
    /// (mm³/s/°C): a cooler nozzle melts slower, so any layer whose planned
    /// temperature dips below base derates its flow cap — and therefore its
    /// clamped speeds — by this. Never raised on warmer layers (the profile
    /// cap is the calibrated number).
    pub max_flow_derate_per_c: f64,
    /// Global extrusion multiplier (filament-specific flow tuning). 1.0 = nominal.
    pub extrusion_multiplier: f64,
    /// Flow multiplier for bridges and arc overhangs. <1.0 tightens a strand that
    /// cools and sets (round-bead, anti-sag); >1.0 gives a fatter strand with the
    /// body to span and grip when cooling is poor (enclosed chamber) and a lean
    /// strand would curl into vines. 1.0 = nominal.
    pub bridge_flow: f64,
    /// How far (mm) a bridge sheet over an enclosed ceiling lands onto the
    /// supported rim — the perimeter-free foothold its ends rest on. Bigger =
    /// more solid under the bridge ends (sturdier anchor) but the inner
    /// perimeters start further from the hollow. 0 = no foothold band.
    pub bridge_foothold_mm: f64,
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
    /// (Sovol Zero / Bambu-style side fan). Off by default — declare it per
    /// printer (the GUI checkbox / `aux_fan = true`). Gates all P2 emission:
    /// vanilla Klipper/Marlin read the P-form as the *primary* fan and there's
    /// no non-breaking raw-g-code guard, so the slicer only emits it once the
    /// hardware is confirmed.
    pub has_aux_fan: bool,
    /// Aux-fan duty 0.0..=1.0, flat once past `fan_off_layers`. 0 = off.
    pub aux_fan_speed: f64,
    /// The machine has a chamber-exhaust fan addressed as `M106 P3`. Off by
    /// default; declare per printer like the aux fan.
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

    // --- tools (toolchanger / MMU) ---
    /// Physical toolchanger vs single-nozzle MMU (Happy Hare / ERCF / AMS).
    pub machine_kind: MachineKind,
    /// Number of tool slots on the machine (1 = ordinary single-tool printer).
    pub tool_count: usize,
    /// Template emitted at each tool change. Placeholders: `{tool}` (target),
    /// `{from_tool}` (previous), `{to_temp}` (target nozzle °C for this layer),
    /// `{purge_mm3}` / `{purge_mm}` (MMU static purge as volume / filament
    /// length). A toolchanger uses `T{tool}`; an MMU a swap macro like
    /// `MMU_CHANGE_TOOL TOOL={tool}`.
    pub toolchange_gcode: String,
    /// Estimated seconds per tool change (time estimate / M73). On an MMU the
    /// purge time (`purge_volume_mm3` / max flow) is added on top per swap.
    pub toolchange_seconds: f64,
    /// MMU only: filament volume (mm³) purged at every swap — one static figure
    /// handed to the swap macro (`{purge_mm3}` / `{purge_mm}`) and counted as
    /// waste in the estimates. The firmware decides WHERE it goes (bucket /
    /// blobifier); the slicer never lays a wipe tower.
    pub purge_volume_mm3: f64,
    /// Tool-0 / flat-view standby temperature (docked-tool setpoint).
    pub standby_temp_c: u32,
    /// Docked longer than this (estimated seconds) and a tool drops to its
    /// standby temperature, reheating a layer ahead of its next pickup.
    /// Short docks — blend dithering alternates tools every layer — stay at
    /// print temperature; thermal cycling would cost more than it saves.
    pub standby_after_s: f64,
    /// Tool-0 / flat-view filament display color.
    pub filament_color_rgb: [f32; 3],
    /// Per-slot filament settings — never empty (a single-filament resolve
    /// carries one entry, mirroring the flat fields).
    pub tools: Vec<ToolSettings>,

    // --- g-code templates (with {placeholders}) ---
    pub start_gcode: String,
    pub end_gcode: String,
}

impl Default for Settings {
    fn default() -> Self {
        let mut s = Self {
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
            min_cruise_ratio: 0.5,
            layer_height_mm: 0.2,
            first_layer_height_mm: 0.2,
            line_width_mm: 0.45,
            arc_fitting: true,
            arc_tolerance_mm: 0.05,
            wall_count: 2,
            outer_wall_first: false,
            top_layers: 4,
            bottom_layers: 4,
            infill_density: 0.15,
            sparse_pattern: InfillPattern::default(),
            top_pattern: InfillPattern::default(),
            bottom_pattern: InfillPattern::default(),
            solid_pattern: InfillPattern::default(),
            infill_overlap: 0.25,
            monotonic_solid: true,
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
            max_bridge_span_mm: 18.0,
            // 0.8 mm: 4 layers at 0.2 — greys fuse comfortably past arm's
            // length; tighten for saturated colors, at a sparser palette.
            blend_band_mm: 0.8,
            retract_len_mm: 0.8,
            retract_speed_mm_s: 35.0,
            z_hop_mm: 0.0,
            wipe_mm: 2.0,
            host_url: String::new(),
            api_key: String::new(),
            material: Material::Pla,
            nozzle_temp_c: 210,
            first_layer_nozzle_temp_c: derived_first_layer_temp_c(210, Material::Pla),
            bed_temp_c: 60,
            machine_speed_mm_s: 60.0,
            speed_quality: 0.0,
            print_speed_mm_s: derived_print_speed_mm_s(60.0, 0.0),
            travel_speed_mm_s: 120.0,
            first_layer_speed_mm_s: 20.0,
            external_perimeter_speed_mm_s: 25.0,
            solid_speed_mm_s: 40.0,
            support_speed_mm_s: 45.0,
            bridge_speed_mm_s: 10.0,
            overhang_speed_mm_s: derived_overhang_speed_mm_s(50.0),
            min_layer_time_s: 8.0,
            min_print_speed_mm_s: 10.0,
            max_volumetric_speed_mm3_s: 15.0,
            max_flow_derate_per_c: 0.3,
            extrusion_multiplier: 1.0,
            bridge_flow: 1.5,
            bridge_foothold_mm: 0.9,
            pressure_advance: 0.0,
            fan_speed: 1.0,
            bridge_fan_speed: 1.0,
            fan_off_layers: 1,
            has_aux_fan: false, // off until the printer declares it (GUI checkbox / aux_fan = true)
            aux_fan_speed: 0.0,
            has_exhaust_fan: false,
            exhaust_fan_speed: 0.0,
            chamber_sensor: String::new(),
            chamber_temp_c: Material::Pla.chamber_temp_c(),
            machine_kind: MachineKind::default(),
            tool_count: 1,
            toolchange_gcode: "T{tool}".to_string(),
            toolchange_seconds: 10.0,
            purge_volume_mm3: 80.0,
            standby_temp_c: derived_standby_temp_c(210),
            standby_after_s: 120.0,
            filament_color_rgb: NEUTRAL_FILAMENT_RGB,
            tools: Vec::new(),
            start_gcode: GENERIC_START_GCODE.to_string(),
            end_gcode: GENERIC_END_GCODE.to_string(),
        };
        // `tools` is never empty: tool 0 mirrors the flat defaults.
        s.tools = vec![s.flat_tool(String::new())];
        s
    }
}

impl Settings {
    /// Cross-sectional area of the filament (mm²), used for extrusion math.
    pub fn filament_area_mm2(&self) -> f64 {
        let r = self.filament_diameter_mm / 2.0;
        PI * r * r
    }

    /// One shared heater: the temperature ramps between materials at each swap
    /// (with a blocking wait) and idle tools can't be held at standby. True for
    /// both single-heater machines (shared nozzle and shared heater), false for
    /// independent per-tool hotends.
    pub fn single_heater(&self) -> bool {
        matches!(self.machine_kind, MachineKind::SharedNozzle | MachineKind::SharedHeater)
    }

    /// The filaments share a melt zone, so a swap must purge the old color out —
    /// only the shared-nozzle machine. Separate nozzles (independent hotends or
    /// shared heater) keep each color in its own tip and never flush.
    pub fn purges(&self) -> bool {
        self.machine_kind == MachineKind::SharedNozzle
    }

    /// The tool at slot `i`, clamped to the last loaded slot (`tools` is
    /// never empty — a single-filament resolve carries one entry).
    pub fn tool(&self, i: usize) -> &ToolSettings {
        &self.tools[i.min(self.tools.len() - 1)]
    }

    /// The per-filament view of the flat fields — what tool 0 mirrors.
    pub fn flat_tool(&self, filament_name: String) -> ToolSettings {
        ToolSettings {
            filament_name,
            color_rgb: self.filament_color_rgb,
            material: self.material,
            filament_diameter_mm: self.filament_diameter_mm,
            filament_density_g_cm3: self.filament_density_g_cm3,
            nozzle_temp_c: self.nozzle_temp_c,
            first_layer_nozzle_temp_c: self.first_layer_nozzle_temp_c,
            bed_temp_c: self.bed_temp_c,
            max_volumetric_speed_mm3_s: self.max_volumetric_speed_mm3_s,
            max_flow_derate_per_c: self.max_flow_derate_per_c,
            extrusion_multiplier: self.extrusion_multiplier,
            pressure_advance: self.pressure_advance,
            bridge_flow: self.bridge_flow,
            bridge_speed_mm_s: self.bridge_speed_mm_s,
            fan_speed: self.fan_speed,
            bridge_fan_speed: self.bridge_fan_speed,
            fan_off_layers: self.fan_off_layers,
            aux_fan_speed: self.aux_fan_speed,
            exhaust_fan_speed: self.exhaust_fan_speed,
            chamber_temp_c: self.chamber_temp_c,
            standby_temp_c: self.standby_temp_c,
        }
    }
}

// --- auto-derived defaults ---------------------------------------------------
// One source of truth for the "auto" values: `Profiles::resolve` uses these
// when a profile leaves the field unset, and the GUI recomputes them live for
// unpinned fields (so dragging the master visibly moves its dependents).

/// The nozzle-temperature slider bounds (°C); the upper bound also caps the
/// derived first-layer temperature.
pub const NOZZLE_TEMP_MIN_C: u32 = 150;
pub const NOZZLE_TEMP_MAX_C: u32 = 320;

/// Standby temperature for a docked tool: 50 °C under the operating
/// temperature — no meaningful ooze or heat damage, a few seconds to recover —
/// floored where reheating stops being quick.
pub fn derived_standby_temp_c(nozzle_temp_c: u32) -> u32 {
    nozzle_temp_c.saturating_sub(50).max(110)
}

/// First-layer temperature: the operating temperature + the material's
/// adhesion bump, clipped to the hotend ceiling.
pub fn derived_first_layer_temp_c(nozzle_temp_c: u32, material: Material) -> u32 {
    (nozzle_temp_c + material.first_layer_bump_c()).min(NOZZLE_TEMP_MAX_C)
}

/// Filament display color when a profile sets none — a neutral grey.
pub const NEUTRAL_FILAMENT_RGB: [f32; 3] = [0.66, 0.66, 0.66];

/// Parse a display color: "#RRGGBB", bare "RRGGBB", or short "#RGB".
/// Anything else is `None` — callers fall back to the neutral default.
pub fn parse_hex_color(s: &str) -> Option<[f32; 3]> {
    let digits: Vec<f32> = s
        .trim()
        .trim_start_matches('#')
        .chars()
        .map(|c| c.to_digit(16).map(|v| v as f32))
        .collect::<Option<_>>()?;
    match digits[..] {
        [r1, r0, g1, g0, b1, b0] => Some([
            (r1 * 16.0 + r0) / 255.0,
            (g1 * 16.0 + g0) / 255.0,
            (b1 * 16.0 + b0) / 255.0,
        ]),
        [r, g, b] => Some([r * 17.0 / 255.0, g * 17.0 / 255.0, b * 17.0 / 255.0]),
        _ => None,
    }
}

/// Format a color back to the profile/TOML "#RRGGBB" form.
pub fn hex_color(rgb: [f32; 3]) -> String {
    let ch = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u8;
    format!("#{:02X}{:02X}{:02X}", ch(rgb[0]), ch(rgb[1]), ch(rgb[2]))
}

/// What interleaved filaments read as at viewing distance: the weighted
/// average in LINEAR light (spatial/partitive mixing is additive in light,
/// not in gamma-encoded sRGB — an sRGB average would render every blend too
/// dark). Inputs and output are sRGB; weights are relative shares.
pub fn mix_colors_linear(entries: &[([f32; 3], f32)]) -> [f32; 3] {
    let to_lin = |c: f32| {
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };
    let to_srgb = |c: f32| {
        if c <= 0.003_130_8 {
            c * 12.92
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        }
    };
    let total: f32 = entries.iter().map(|&(_, w)| w.max(0.0)).sum();
    if total <= 0.0 {
        return NEUTRAL_FILAMENT_RGB;
    }
    let mut lin = [0.0f32; 3];
    for &(rgb, w) in entries {
        let w = w.max(0.0) / total;
        for (l, &c) in lin.iter_mut().zip(&rgb) {
            *l += to_lin(c) * w;
        }
    }
    [to_srgb(lin[0]), to_srgb(lin[1]), to_srgb(lin[2])]
}

/// The mix of `palette` colors that lands nearest `target` — the inverse of
/// [`mix_colors_linear`]: least squares in linear light over the simplex
/// (weights ≥ 0 summing to 1), by projected gradient descent (a handful of
/// colors, a few hundred tiny steps — exact enough for a picker). A target
/// outside the achievable gamut snaps to its closest mixable color.
pub fn blend_weights_for_color(target: [f32; 3], palette: &[[f32; 3]]) -> Vec<f32> {
    let to_lin = |c: f32| {
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };
    let n = palette.len();
    if n == 0 {
        return Vec::new();
    }
    let t: Vec<f32> = target.iter().map(|&c| to_lin(c)).collect();
    let p: Vec<[f32; 3]> = palette
        .iter()
        .map(|c| [to_lin(c[0]), to_lin(c[1]), to_lin(c[2])])
        .collect();
    let mut w = vec![1.0f32 / n as f32; n];
    for _ in 0..500 {
        // Residual of the current mix, then one gradient step per weight.
        let mut mix = [0.0f32; 3];
        for (wi, pi) in w.iter().zip(&p) {
            for (m, &c) in mix.iter_mut().zip(pi) {
                *m += wi * c;
            }
        }
        let r = [mix[0] - t[0], mix[1] - t[1], mix[2] - t[2]];
        for (wi, pi) in w.iter_mut().zip(&p) {
            *wi -= 0.5 * (r[0] * pi[0] + r[1] * pi[1] + r[2] * pi[2]);
        }
        project_onto_simplex(&mut w);
    }
    w
}

/// Quantize blend fractions to whole layers of a `cycle`-layer dither
/// (largest-remainder apportionment; the counts sum to `cycle`). Rational
/// weights n/cycle make the engine's error diffusion exactly periodic in
/// `cycle` layers — the credits net to zero each cycle — so the pattern's
/// repeat height is bounded by cycle × layer height: every offered mix
/// actually fuses within the blend band. The price is a discrete palette:
/// a 90/10 ask at cycle 4 lands on pure (4:0), not a 2 mm stripe.
pub fn quantize_blend_fractions(fractions: &[f32], cycle: usize) -> Vec<u32> {
    let n = fractions.len();
    let cycle = cycle.max(1);
    if n == 0 {
        return Vec::new();
    }
    let total: f32 = fractions.iter().map(|&f| f.max(0.0)).sum();
    if total <= 0.0 {
        let mut out = vec![0u32; n];
        out[0] = cycle as u32;
        return out;
    }
    let ideal: Vec<f32> =
        fractions.iter().map(|&f| f.max(0.0) / total * cycle as f32).collect();
    let mut out: Vec<u32> = ideal.iter().map(|&x| x.floor() as u32).collect();
    let mut left = cycle as u32 - out.iter().sum::<u32>();
    // Hand the remaining layers to the largest remainders (ties → lowest
    // slot, matching the dither's own tie-break).
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        let (ra, rb) = (ideal[a] - ideal[a].floor(), ideal[b] - ideal[b].floor());
        rb.partial_cmp(&ra).unwrap_or(std::cmp::Ordering::Equal).then(a.cmp(&b))
    });
    let mut oi = 0usize;
    while left > 0 {
        out[order[oi % n]] += 1;
        left -= 1;
        oi += 1;
    }
    out
}

/// OkLCh — perceptual lightness, chroma, hue° — of an sRGB color. Blends are
/// ordered in this space rather than by luma/HSV: luma is red-blind (it
/// weights green 0.72, red 0.21, so it barely "sees" a red's saturation) and
/// HSV hue is unstable near neutral and seams at pure red. Oklab lightness is
/// even, and its hue only seams out in the purples, away from the primaries.
fn oklab_lch(rgb: [f32; 3]) -> [f32; 3] {
    let to_lin = |c: f32| {
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };
    let (r, g, b) = (to_lin(rgb[0]), to_lin(rgb[1]), to_lin(rgb[2]));
    let l = 0.412_221_47 * r + 0.536_332_54 * g + 0.051_445_995 * b;
    let m = 0.211_903_5 * r + 0.680_699_5 * g + 0.107_396_96 * b;
    let s = 0.088_302_46 * r + 0.281_718_85 * g + 0.629_978_7 * b;
    let (l_, m_, s_) = (l.cbrt(), m.cbrt(), s.cbrt());
    let big_l = 0.210_454_26 * l_ + 0.793_617_8 * m_ - 0.004_072_047 * s_;
    let a = 1.977_998_5 * l_ - 2.428_592_2 * m_ + 0.450_593_7 * s_;
    let bb = 0.025_904_037 * l_ + 0.782_771_77 * m_ - 0.808_675_77 * s_;
    let chroma = (a * a + bb * bb).sqrt();
    let hue = bb.atan2(a).to_degrees().rem_euclid(360.0);
    [big_l, chroma, hue]
}

// Below this Oklab chroma a mix reads as grey and joins the neutral ramp.
const BLEND_NEUTRAL_CHROMA: f32 = 0.02;
// Hue-family width (degrees): wide enough that one spool's tints stay in a
// single family (and never seam at pure red), narrow enough to keep visibly
// distinct hues apart. Splitting hue finer than this would fracture a red
// run, not order it — the ordering within a family is what does that work.
const BLEND_HUE_STEP: f32 = 45.0;
// Lightness band (Oklab L): within a hue family the chips are grouped into
// bands this tall and sorted by chroma inside each. That makes chroma — the
// axis a pure lightness sort drops on the floor — an ordered dimension, so a
// family reads as a little tint-chart (dark→light down, dull→vivid across).
const BLEND_L_BAND: f32 = 0.06;

/// Every printable mix of `palette` at a `cycle`-layer dither — the finite
/// lattice the blend band affords: all whole-layer compositions of the
/// cycle, deduplicated by resulting color (slots sharing a spool color
/// collapse; the simplest recipe wins), sorted into a natural ramp — greys
/// first (dark→light), then each hue family as a lightness×chroma tint-chart
/// (see [`oklab_lch`]). `None` when
/// the lattice exceeds `cap` compositions — too rich to enumerate usefully,
/// the caller falls back to pick-and-snap.
pub fn blend_lattice(
    palette: &[[f32; 3]],
    cycle: usize,
    cap: usize,
) -> Option<Vec<(Vec<u32>, [f32; 3])>> {
    let k = palette.len();
    if k == 0 {
        return Some(Vec::new());
    }
    // C(cycle + k − 1, k − 1), bailing early — the sequential product stays
    // integral, so no factorial blowup.
    let mut count: u128 = 1;
    for i in 1..k {
        count = count * (cycle + i) as u128 / i as u128;
        if count > cap as u128 {
            return None;
        }
    }
    fn fill(
        palette: &[[f32; 3]],
        comp: &mut Vec<u32>,
        slot: usize,
        left: u32,
        out: &mut Vec<(Vec<u32>, [f32; 3])>,
    ) {
        if slot + 1 == comp.len() {
            comp[slot] = left;
            let entries: Vec<([f32; 3], f32)> =
                palette.iter().copied().zip(comp.iter().map(|&n| n as f32)).collect();
            out.push((comp.clone(), mix_colors_linear(&entries)));
            return;
        }
        for n in 0..=left {
            comp[slot] = n;
            fill(palette, comp, slot + 1, left - n, out);
        }
    }
    let mut all = Vec::new();
    fill(palette, &mut vec![0u32; k], 0, cycle as u32, &mut all);
    // Dedupe by displayed color; among lookalikes keep the fewest spools.
    let mut best: std::collections::HashMap<[u8; 3], usize> = std::collections::HashMap::new();
    let key = |c: [f32; 3]| {
        [
            (c[0] * 255.0).round() as u8,
            (c[1] * 255.0).round() as u8,
            (c[2] * 255.0).round() as u8,
        ]
    };
    for (i, (comp, rgb)) in all.iter().enumerate() {
        let parts = comp.iter().filter(|&&n| n > 0).count();
        match best.entry(key(*rgb)) {
            std::collections::hash_map::Entry::Vacant(e) => {
                e.insert(i);
            }
            std::collections::hash_map::Entry::Occupied(mut e) => {
                let cur = all[*e.get()].0.iter().filter(|&&n| n > 0).count();
                if parts < cur {
                    e.insert(i);
                }
            }
        }
    }
    let mut keep: Vec<usize> = best.into_values().collect();
    // Order the palette so a colored run reads as naturally as a grey one:
    // the neutral ramp first (greys, dark→light), then each hue family laid
    // out as a little tint-chart — lightness bands top to bottom, chroma
    // (dull→vivid) across each band. All in Oklab: sorting reds by luma is
    // what made them look shuffled — luma is nearly blind to red saturation,
    // and it never ordered chroma at all — while the greys, being purely a
    // lightness ramp, happened to sort fine.
    let sort_key = |c: &[f32; 3]| -> (u8, u8, i64, i64, i64) {
        let [l, chroma, hue] = oklab_lch(*c);
        if chroma < BLEND_NEUTRAL_CHROMA {
            // A grey: one clean lightness ramp — hue and chroma don't apply.
            return (0, 0, (l * 1e6) as i64, 0, 0);
        }
        let family = (hue / BLEND_HUE_STEP) as u8;
        let band = (l / BLEND_L_BAND) as i64;
        (1, family, band, (chroma * 1e6) as i64, (l * 1e6) as i64)
    };
    keep.sort_by(|&a, &b| {
        sort_key(&all[a].1).cmp(&sort_key(&all[b].1)).then(all[a].0.cmp(&all[b].0))
    });
    Some(keep.into_iter().map(|i| all[i].clone()).collect())
}

/// A blend's dither repeat, in layers: the weights' sum after dividing out
/// their common factor (error diffusion is exactly periodic in that many
/// layers). Multiply by the layer height for the repeat's physical height —
/// the thing that must stay inside the blend band to read as one color.
/// Pure tools and empty blends trivially repeat every layer.
pub fn blend_repeat_layers(weights: &[(u32, f32)]) -> u32 {
    let ints: Vec<u64> =
        weights.iter().filter(|&&(_, w)| w >= 0.5).map(|&(_, w)| w.round() as u64).collect();
    if ints.len() < 2 {
        return 1;
    }
    let g = ints.iter().fold(0u64, |acc, &b| gcd(acc, b)).max(1);
    (ints.iter().sum::<u64>() / g) as u32
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// Euclidean projection onto the probability simplex (Duchi et al.): shift
/// everything down by the threshold that makes the clamped sum exactly 1.
/// (Plain clamp-and-renormalize is NOT this — rescaling can cancel a gradient
/// step and stall a solve short of the boundary.)
fn project_onto_simplex(w: &mut [f32]) {
    let mut u = w.to_vec();
    u.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
    let mut cumsum = 0.0f32;
    let mut theta = 0.0f32;
    for (k, &uk) in u.iter().enumerate() {
        cumsum += uk;
        let t = (cumsum - 1.0) / (k as f32 + 1.0);
        if uk - t > 0.0 {
            theta = t;
        }
    }
    for wi in w.iter_mut() {
        *wi = (*wi - theta).max(0.0);
    }
}

/// Nominal print speed: the machine's rated speed × the finish↔speed factor
/// (−1 → 60%, 0 → 80%, +1 → 100%).
pub fn derived_print_speed_mm_s(machine_speed_mm_s: f64, speed_quality: f64) -> f64 {
    machine_speed_mm_s * (0.8 + 0.2 * speed_quality.clamp(-1.0, 1.0))
}

/// Auto line width: 112.5% of the nozzle bore — wide enough to squeeze a solid
/// bead, narrow enough to hold detail (0.4 mm nozzle → 0.45 mm).
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

/// Contour-cleanup threshold (mm), no knob. After slicing, contour points whose
/// deviation falls under this are dropped — they're mesh-facet tessellation noise
/// below the printer's mechanical step. 0.01 mm matches Orca/Prusa: it preserves
/// genuine curve detail (arc fitting renders it as smooth G2/G3) rather than the
/// bead-scale decimation we used to do. It's a path-representation precision, not a
/// bead property, so it doesn't scale with line width.
pub fn contour_resolution_mm() -> f64 {
    0.01
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contour_resolution_is_a_fine_fixed_floor() {
        // A fixed 0.01 mm path-precision (Orca-style), not bead-derived: fine
        // enough to keep curve detail for arc fitting, coarse enough to drop
        // mesh-facet tessellation noise.
        assert!((contour_resolution_mm() - 0.01).abs() < 1e-12);
    }

    #[test]
    fn hex_colors_parse_tolerantly() {
        assert_eq!(parse_hex_color("#FF0000"), Some([1.0, 0.0, 0.0]));
        assert_eq!(parse_hex_color("00ff00"), Some([0.0, 1.0, 0.0])); // bare, lowercase
        assert_eq!(parse_hex_color(" #00F "), Some([0.0, 0.0, 1.0])); // short, padded
        assert_eq!(parse_hex_color("#abc"), parse_hex_color("#AABBCC"));
        assert_eq!(parse_hex_color("not-a-color"), None);
        assert_eq!(parse_hex_color(""), None);
        assert_eq!(parse_hex_color("#12345"), None); // wrong length
        // Formatting round-trips exactly for byte-derived channels.
        let c = parse_hex_color("#8A2BE2").unwrap();
        assert_eq!(hex_color(c), "#8A2BE2");
    }

    #[test]
    fn default_settings_carry_a_mirroring_tool_zero() {
        let s = Settings::default();
        assert_eq!(s.tools.len(), 1);
        assert_eq!(s.tools[0], s.flat_tool(String::new()));
        assert_eq!(s.tool_count, 1);
        assert_eq!(s.toolchange_gcode, "T{tool}");
        assert_eq!(s.toolchange_seconds, 10.0);
        // tool() clamps past the last loaded slot instead of panicking.
        assert_eq!(s.tool(7), s.tool(0));
    }

    #[test]
    fn blend_lattice_enumerates_the_printable_mixes() {
        let w = [0.95, 0.95, 0.95];
        let b = [0.10, 0.10, 0.10];
        let g = [0.50, 0.50, 0.50];
        // Two spools, 4-layer cycle: the five ramp steps, dark to light.
        let lat = blend_lattice(&[w, b], 4, 200).unwrap();
        assert_eq!(lat.len(), 5);
        assert_eq!(lat.first().unwrap().0, vec![0, 4], "darkest first");
        assert_eq!(lat.last().unwrap().0, vec![4, 0], "lightest last");
        // Three spools: C(6,2) = 15 compositions.
        assert_eq!(blend_lattice(&[w, g, b], 4, 200).unwrap().len(), 15);
        // Two slots loaded with the SAME spool color collapse to one ramp
        // point each — five compositions, one color.
        assert_eq!(blend_lattice(&[w, w], 4, 200).unwrap().len(), 1);
        // Eight spools at cycle 4 is 330 compositions: over a 168 cap → None;
        // under a 400 cap they enumerate, then coincident mixes collapse (a
        // grey ramp has MANY compositions landing on the same grey — that's
        // the point of the dedupe), every survivor a whole 4-layer recipe.
        let eight: Vec<[f32; 3]> = (0..8).map(|i| [i as f32 / 7.0; 3]).collect();
        assert!(blend_lattice(&eight, 4, 168).is_none());
        let lat = blend_lattice(&eight, 4, 400).unwrap();
        assert!(lat.len() > 100 && lat.len() < 330, "deduped: {}", lat.len());
        assert!(lat.iter().all(|(c, _)| c.iter().sum::<u32>() == 4));
        // Palette order: with red + white + black loaded, the neutral
        // white↔black ramp leads (Oklab lightness, dark→light), the reds
        // follow as one hue family, and greys never interleave with them.
        let lat = blend_lattice(&[[0.9, 0.1, 0.1], w, b], 4, 200).unwrap();
        let is_neutral = |c: &[f32; 3]| oklab_lch(*c)[1] < BLEND_NEUTRAL_CHROMA;
        let first_chromatic = lat.iter().position(|(_, c)| !is_neutral(c)).unwrap();
        assert!(
            lat[first_chromatic..].iter().all(|(_, c)| !is_neutral(c)),
            "achromatic ramp first, then the chromatic run — no interleaving"
        );
        for pair in lat[..first_chromatic].windows(2) {
            assert!(
                oklab_lch(pair[0].1)[0] <= oklab_lch(pair[1].1)[0],
                "neutral run is dark to light in Oklab lightness"
            );
        }
        // The reds are one hue family, ordered into lightness bands (chroma
        // sorted within each): the band index never decreases across the run.
        let band = |c: &[f32; 3]| (oklab_lch(*c)[0] / BLEND_L_BAND) as i64;
        for pair in lat[first_chromatic..].windows(2) {
            assert!(
                band(&pair[0].1) <= band(&pair[1].1),
                "reds are banded by lightness, dark to light"
            );
        }
    }

    #[test]
    fn blend_repeat_period_is_sum_over_gcd() {
        // 3:1 repeats every 4 layers; 50:50 every 2; 72:18:10 every 50.
        assert_eq!(blend_repeat_layers(&[(0, 3.0), (2, 1.0)]), 4);
        assert_eq!(blend_repeat_layers(&[(0, 50.0), (1, 50.0)]), 2);
        assert_eq!(blend_repeat_layers(&[(0, 72.0), (1, 18.0), (2, 10.0)]), 50);
        // Pure tools and empty blends trivially repeat every layer.
        assert_eq!(blend_repeat_layers(&[(1, 4.0)]), 1);
        assert_eq!(blend_repeat_layers(&[]), 1);
    }

    #[test]
    fn blend_fractions_quantize_to_whole_layers() {
        // 75/25 at a 4-layer cycle: 3 layers + 1 layer.
        assert_eq!(quantize_blend_fractions(&[0.75, 0.25], 4), vec![3, 1]);
        // A 90/10 ask can't fit a 4-layer cycle — it lands on pure.
        assert_eq!(quantize_blend_fractions(&[0.9, 0.1], 4), vec![4, 0]);
        // …but an 8-layer cycle can hold one layer in eight.
        assert_eq!(quantize_blend_fractions(&[0.9, 0.1], 8), vec![7, 1]);
        // Equal thirds at cycle 4: the spare layer goes to the lowest slot.
        assert_eq!(quantize_blend_fractions(&[1.0, 1.0, 1.0], 4), vec![2, 1, 1]);
        // Counts always sum to the cycle.
        for cycle in 1..=12 {
            let q = quantize_blend_fractions(&[0.61, 0.29, 0.10], cycle);
            assert_eq!(q.iter().sum::<u32>(), cycle as u32, "cycle {cycle}: {q:?}");
        }
        // Degenerate input still fills the cycle.
        assert_eq!(quantize_blend_fractions(&[0.0, 0.0], 4), vec![4, 0]);
    }

    #[test]
    fn blend_weights_invert_the_mix() {
        let white = [0.95, 0.95, 0.95];
        let black = [0.10, 0.10, 0.10];
        let grey = [0.50, 0.50, 0.50];
        // A palette color is itself: full weight on the matching slot.
        let w = blend_weights_for_color(white, &[white, black]);
        assert!(w[0] > 0.99, "white is pure white: {w:?}");
        // The mix of any weights round-trips back to those weights.
        let mixed = mix_colors_linear(&[(white, 0.3), (black, 0.7)]);
        let w = blend_weights_for_color(mixed, &[white, black]);
        assert!((w[0] - 0.3).abs() < 0.01 && (w[1] - 0.7).abs() < 0.01, "{w:?}");
        // A target outside the gamut (saturated red over greys) still lands on
        // a valid simplex point.
        let w = blend_weights_for_color([1.0, 0.0, 0.0], &[white, grey, black]);
        let sum: f32 = w.iter().sum();
        assert!((sum - 1.0).abs() < 1e-4 && w.iter().all(|&x| x >= 0.0), "{w:?}");
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
