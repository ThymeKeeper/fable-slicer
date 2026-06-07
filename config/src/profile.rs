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

use serde::Deserialize;

use crate::{InfillPattern, SeamMode, Settings, SupportMode, GENERIC_END_GCODE, GENERIC_START_GCODE};

/// Printer (machine) tier: bed, extruder, and start/end g-code.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct PrinterProfile {
    pub inherits: Option<String>,
    pub bed_size_x_mm: Option<f64>,
    pub bed_size_y_mm: Option<f64>,
    pub nozzle_diameter_mm: Option<f64>,
    pub travel_speed_mm_s: Option<f64>,
    pub print_speed_mm_s: Option<f64>,
    pub first_layer_speed_mm_s: Option<f64>,
    pub acceleration: Option<f64>,
    pub jerk: Option<f64>,
    pub retract_len_mm: Option<f64>,
    pub retract_speed_mm_s: Option<f64>,
    pub z_hop_mm: Option<f64>,
    pub start_gcode: Option<String>,
    pub end_gcode: Option<String>,
}

/// Filament (material) tier: diameter and temperatures.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct FilamentProfile {
    pub inherits: Option<String>,
    pub filament_diameter_mm: Option<f64>,
    pub density_g_cm3: Option<f64>,
    pub nozzle_temp_c: Option<u32>,
    pub bed_temp_c: Option<u32>,
}

/// Process (print) tier: quality/geometry knobs.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct ProcessProfile {
    pub inherits: Option<String>,
    pub layer_height_mm: Option<f64>,
    pub first_layer_height_mm: Option<f64>,
    pub line_width_mm: Option<f64>,
    pub max_resolution_mm: Option<f64>,
    pub wall_count: Option<usize>,
    pub top_layers: Option<usize>,
    pub bottom_layers: Option<usize>,
    pub infill_density: Option<f64>,
    pub sparse_infill: Option<String>,
    pub solid_infill: Option<String>,
    pub skirt_loops: Option<usize>,
    pub skirt_gap_mm: Option<f64>,
    pub brim_loops: Option<usize>,
    pub seam: Option<String>,
    pub support: Option<String>,
    pub support_overhang_angle_deg: Option<f64>,
    pub support_density: Option<f64>,
    pub support_xy_clearance_mm: Option<f64>,
    pub print_speed_mm_s: Option<f64>,
    pub first_layer_speed_mm_s: Option<f64>,
    pub min_layer_time_s: Option<f64>,
    pub min_print_speed_mm_s: Option<f64>,
}

/// One inheritable tier: knows its parent and how to layer over a base.
trait Tier: Clone {
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
        merge_fields!(self, base, bed_size_x_mm, bed_size_y_mm, nozzle_diameter_mm,
            travel_speed_mm_s, print_speed_mm_s, first_layer_speed_mm_s, acceleration, jerk,
            retract_len_mm, retract_speed_mm_s, z_hop_mm, start_gcode, end_gcode)
    }
}

impl Tier for FilamentProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, filament_diameter_mm, density_g_cm3, nozzle_temp_c, bed_temp_c)
    }
}

impl Tier for ProcessProfile {
    fn parent(&self) -> Option<&str> {
        self.inherits.as_deref()
    }
    fn over(self, base: Self) -> Self {
        merge_fields!(self, base, layer_height_mm, first_layer_height_mm, line_width_mm,
            max_resolution_mm, wall_count, top_layers, bottom_layers, infill_density, sparse_infill, solid_infill,
            skirt_loops, skirt_gap_mm, brim_loops, seam, support, support_overhang_angle_deg,
            support_density, support_xy_clearance_mm, print_speed_mm_s, first_layer_speed_mm_s,
            min_layer_time_s, min_print_speed_mm_s)
    }
}

/// A registry of named profiles for each tier.
#[derive(Default)]
pub struct Profiles {
    printers: HashMap<String, PrinterProfile>,
    filaments: HashMap<String, FilamentProfile>,
    processes: HashMap<String, ProcessProfile>,
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
        p.processes.insert("standard".into(), parse("process/standard", include_str!("../profiles/process/standard.toml")));
        p.processes.insert("fine".into(), parse("process/fine", include_str!("../profiles/process/fine.toml")));
        p.processes.insert("draft".into(), parse("process/draft", include_str!("../profiles/process/draft.toml")));
        p
    }

    /// Load extra profiles from `<dir>/{printer,filament,process}/*.toml`,
    /// overriding built-ins with the same file stem.
    pub fn load_dir(&mut self, dir: &Path) -> Result<(), String> {
        load_tier(&mut self.printers, &dir.join("printer"))?;
        load_tier(&mut self.filaments, &dir.join("filament"))?;
        load_tier(&mut self.processes, &dir.join("process"))?;
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
        Ok(Settings {
            nozzle_diameter_mm: pr.nozzle_diameter_mm.unwrap_or(d.nozzle_diameter_mm),
            filament_diameter_mm: fl.filament_diameter_mm.unwrap_or(d.filament_diameter_mm),
            filament_density_g_cm3: fl.density_g_cm3.unwrap_or(d.filament_density_g_cm3),
            bed_size_x_mm: pr.bed_size_x_mm.unwrap_or(d.bed_size_x_mm),
            bed_size_y_mm: pr.bed_size_y_mm.unwrap_or(d.bed_size_y_mm),
            acceleration_mm_s2: pr.acceleration.unwrap_or(d.acceleration_mm_s2),
            jerk_mm_s: pr.jerk.unwrap_or(d.jerk_mm_s),
            layer_height_mm: pc.layer_height_mm.unwrap_or(d.layer_height_mm),
            first_layer_height_mm: pc.first_layer_height_mm.unwrap_or(d.first_layer_height_mm),
            line_width_mm: pc.line_width_mm.unwrap_or(d.line_width_mm),
            max_resolution_mm: pc.max_resolution_mm.unwrap_or(d.max_resolution_mm),
            wall_count: pc.wall_count.unwrap_or(d.wall_count),
            top_layers: pc.top_layers.unwrap_or(d.top_layers),
            bottom_layers: pc.bottom_layers.unwrap_or(d.bottom_layers),
            infill_density: pc.infill_density.unwrap_or(d.infill_density),
            sparse_pattern: pc.sparse_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.sparse_pattern),
            solid_pattern: pc.solid_infill.as_deref().and_then(InfillPattern::parse).unwrap_or(d.solid_pattern),
            skirt_loops: pc.skirt_loops.unwrap_or(d.skirt_loops),
            skirt_gap_mm: pc.skirt_gap_mm.unwrap_or(d.skirt_gap_mm),
            brim_loops: pc.brim_loops.unwrap_or(d.brim_loops),
            seam_mode: pc.seam.as_deref().and_then(SeamMode::parse).unwrap_or(d.seam_mode),
            support_mode: pc.support.as_deref().and_then(SupportMode::parse).unwrap_or(d.support_mode),
            support_overhang_angle_deg: pc
                .support_overhang_angle_deg
                .unwrap_or(d.support_overhang_angle_deg),
            support_density: pc.support_density.unwrap_or(d.support_density),
            support_xy_clearance_mm: pc.support_xy_clearance_mm.unwrap_or(d.support_xy_clearance_mm),
            retract_len_mm: pr.retract_len_mm.unwrap_or(d.retract_len_mm),
            retract_speed_mm_s: pr.retract_speed_mm_s.unwrap_or(d.retract_speed_mm_s),
            z_hop_mm: pr.z_hop_mm.unwrap_or(d.z_hop_mm),
            nozzle_temp_c: fl.nozzle_temp_c.unwrap_or(d.nozzle_temp_c),
            bed_temp_c: fl.bed_temp_c.unwrap_or(d.bed_temp_c),
            // Printer speed (machine default) takes precedence over the process value.
            print_speed_mm_s: pr.print_speed_mm_s.or(pc.print_speed_mm_s).unwrap_or(d.print_speed_mm_s),
            travel_speed_mm_s: pr.travel_speed_mm_s.unwrap_or(d.travel_speed_mm_s),
            first_layer_speed_mm_s: pr.first_layer_speed_mm_s.or(pc.first_layer_speed_mm_s).unwrap_or(d.first_layer_speed_mm_s),
            min_layer_time_s: pc.min_layer_time_s.unwrap_or(d.min_layer_time_s),
            min_print_speed_mm_s: pc.min_print_speed_mm_s.unwrap_or(d.min_print_speed_mm_s),
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

fn load_tier<T: for<'de> Deserialize<'de>>(map: &mut HashMap<String, T>, dir: &Path) -> Result<(), String> {
    if !dir.is_dir() {
        return Ok(());
    }
    for entry in fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
        let path = entry.map_err(|e| e.to_string())?.path();
        if path.extension().and_then(|s| s.to_str()) != Some("toml") {
            continue;
        }
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let profile: T = toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
        map.insert(stem, profile);
    }
    Ok(())
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
}
