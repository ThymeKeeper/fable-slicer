//! Turn planned toolpaths into G-code.
//!
//! Extrusion math: the deposited bead is a **stadium** (flat core, semicircular
//! shoulders — see `config::bead_area_mm2`), so a move of length `L` advances
//! the filament by `E = L * bead_area / filament_cross_section`. Adjacent beads
//! are *placed* at the stadium spacing by the planner, so the shoulders overlap
//! and surfaces come out watertight without over-extrusion.
//!
//! Output targets Klipper: relative extrusion (`M83`) and a retraction before
//! every travel between separate extrusions. The start/end g-code comes from the
//! printer profile (see `config`); `{placeholders}` are substituted here.
//!
//! On a toolchanger (`tool_count > 1`) each path carries a tool id and the plan
//! groups paths by tool (serpentine across layers). The toolchange template is
//! emitted at group boundaries; the dock macro invalidates feed/Z/PA/fan state,
//! which is re-established with the new tool's filament settings.

use config::{Settings, ToolSettings};
use gcode::GcodeBuilder;
use geo2d::{Point, Polygons};
use rayon::prelude::*;

use crate::plan::{LayerPlan, PathKind, SegAttr, ToolPath, Travel};

/// Per-path filament view. On a toolchanger the slot table is authoritative; on
/// a single-tool machine the flat fields are — `tools[0]` mirrors them only at
/// resolve time, and callers may have adjusted the flat fields since.
struct Tools<'a> {
    s: &'a Settings,
    flat: ToolSettings,
}

impl Tools<'_> {
    fn new(s: &Settings) -> Tools<'_> {
        Tools { s, flat: s.flat_tool(String::new()) }
    }

    fn get(&self, tool: u32) -> &ToolSettings {
        if self.s.tool_count > 1 {
            self.s.tool(tool as usize)
        } else {
            &self.flat
        }
    }
}

/// Cross-sectional area (mm²) of a tool's filament — the per-slot mirror of
/// `Settings::filament_area_mm2`.
fn tool_area_mm2(tool: &ToolSettings) -> f64 {
    let r = tool.filament_diameter_mm / 2.0;
    std::f64::consts::PI * r * r
}

/// A path counts (for tool bookkeeping, volumes, timing) only if it can print.
fn printable(p: &ToolPath) -> bool {
    p.points.len() >= 2
}

/// Distinct tools over all printable paths, in first-use order. Public for
/// the front-ends' pre-send checks (does the connected machine have these?).
pub fn used_tools(layers: &[LayerPlan]) -> Vec<u32> {
    let mut out = Vec::new();
    for layer in layers {
        for p in layer.paths.iter().filter(|p| printable(p)) {
            if !out.contains(&p.tool) {
                out.push(p.tool);
            }
        }
    }
    out
}

/// The tool of the first printable path — what the print opens with.
fn initial_tool(layers: &[LayerPlan]) -> u32 {
    layers.iter().flat_map(|l| &l.paths).find(|p| printable(p)).map_or(0, |p| p.tool)
}

/// Distinct tools printing the first non-empty layer (they want their
/// first-layer temperature from the start; the rest idle at print temp).
fn first_layer_tools(layers: &[LayerPlan]) -> Vec<u32> {
    let mut out = Vec::new();
    if let Some(l) = layers.iter().find(|l| l.paths.iter().any(printable)) {
        for p in l.paths.iter().filter(|p| printable(p)) {
            if !out.contains(&p.tool) {
                out.push(p.tool);
            }
        }
    }
    out
}

/// Tool in hand entering each layer (the previous layer's last printable
/// path's tool; None before anything has printed). The estimators charge a
/// toolchange against this exactly where `to_gcode` emits one.
fn entry_tools(layers: &[LayerPlan]) -> Vec<Option<u32>> {
    let mut out = Vec::with_capacity(layers.len());
    let mut last: Option<u32> = None;
    for layer in layers {
        out.push(last);
        if let Some(p) = layer.paths.iter().rev().find(|p| printable(p)) {
            last = Some(p.tool);
        }
    }
    out
}

/// One tool selection: the marker comment + the profile's toolchange template
/// (may be multi-line), placeholders substituted. `{tool}` / `{from_tool}` are
/// the target / previous tool; `{to_temp}` the target nozzle °C; `{purge_mm3}` /
/// `{purge_mm}` the MMU static purge as volume / incoming-filament length. A
/// toolchanger template uses only `{tool}`; the extras are harmless there.
fn emit_toolchange(g: &mut GcodeBuilder, s: &Settings, from: u32, to: u32, purge_mm3: f64, to_temp: u32) {
    g.comment(&format!("toolchange T{to}"));
    let area = tool_area_mm2(s.tool(to as usize));
    let purge_mm = if area > 0.0 { purge_mm3 / area } else { 0.0 };
    // `{from_tool}` before `{tool}` so the shorter key can't clip the longer.
    let text = s
        .toolchange_gcode
        .replace("{from_tool}", &from.to_string())
        .replace("{to_temp}", &to_temp.to_string())
        .replace("{purge_mm3}", &format!("{purge_mm3:.1}"))
        .replace("{purge_mm}", &format!("{purge_mm:.2}"))
        .replace("{tool}", &to.to_string());
    for line in text.lines() {
        if !line.trim().is_empty() {
            g.raw(line.trim_end());
        }
    }
}

/// Emit complete G-code for a planned model.
pub fn to_gcode(layers: &[LayerPlan], s: &Settings) -> String {
    let mut g = GcodeBuilder::new();
    let tools = Tools::new(s);
    // A toolchanger print. tool_count is authoritative: it alone gates every
    // T-form line, so a single-tool profile emits exactly the classic bytes.
    let multi = s.tool_count > 1;
    // A shared-heater machine (one nozzle multiplexing filament, or separate
    // nozzles indexing into one heater): no per-tool T-form temps and no
    // docked-tool standby — the one heater ramps between materials at each swap.
    // `single_heater` is only ever true alongside `multi` in practice.
    let single_heater = s.single_heater();
    // Filaments share a melt zone → each swap purges the old color out (the
    // shared-nozzle machine only; separate nozzles keep their own color).
    let purges = s.purges();
    let used = used_tools(layers);
    let init_tool = initial_tool(layers);
    let layer0_tools = first_layer_tools(layers);
    let init = tools.get(init_tool);
    let travel_f = s.travel_speed_mm_s * 60.0;
    // The wipe backtrack shears residual ooze off against the printed bead —
    // that takes contact TIME. At travel speed the 2mm pass lasts ~3ms and
    // wipes nothing; pace it like the wall it retraces (the reference prints
    // wipe at roughly their wall speed).
    let wipe_f = s.external_perimeter_speed_mm_s.max(1.0) * 60.0;
    let retract_f = s.retract_speed_mm_s * 60.0;
    // Hopped travels lift by this much.
    let hop_mm = s.z_hop_mm;

    // Cumulative time after each layer — drives M73 progress/remaining.
    let mut cum_secs: Vec<f64> = Vec::with_capacity(layers.len());
    {
        let travel_v = s.travel_speed_mm_s.max(1.0);
        let mut acc = 0.0;
        let mut prev_z = 0.0;
        for (layer, entry) in layers.iter().zip(entry_tools(layers)) {
            acc += (layer.print_z_mm - prev_z).abs() / travel_v;
            prev_z = layer.print_z_mm;
            acc += layer_print_seconds(layer, s, &tools, layer.speed_scale, entry);
            cum_secs.push(acc);
        }
    }
    let total_secs = cum_secs.last().copied().unwrap_or(0.0);
    let (fil_mm, fil_g) = estimate_filament(layers, s);

    // Standby plan for docked tools: a tool released for longer than
    // standby_after_s (layer-level estimate) drops to its filament's standby
    // temperature at the swap, reheats a layer ahead of its pickup, and the
    // pickup swap confirms with a blocking wait (normally instant — the lead
    // layer absorbs the reheat; the wait is unmodeled in the estimates).
    // Short docks — blend dithering alternates tools every layer — stay at
    // print temperature: thermal cycling would cost more than it saves.
    let mut drop_at_swap: std::collections::HashMap<usize, u32> = std::collections::HashMap::new();
    let mut confirm_at_swap: std::collections::HashMap<usize, u32> =
        std::collections::HashMap::new();
    let mut reheat_at_layer: Vec<Vec<u32>> = vec![Vec::new(); layers.len()];
    if multi && !single_heater && s.standby_after_s > 0.0 {
        // The swap sequence exactly as the emission loop below will see it.
        let mut events: Vec<(usize, u32, u32)> = Vec::new(); // (layer, from, to)
        let mut t = init_tool;
        for (li, layer) in layers.iter().enumerate() {
            for p in layer.paths.iter().filter(|p| printable(p)) {
                if p.tool != t {
                    events.push((li, t, p.tool));
                    t = p.tool;
                }
            }
        }
        for (i, &(li, from, _)) in events.iter().enumerate() {
            let pickup =
                events[i + 1..].iter().position(|&(_, _, to)| to == from).map(|k| i + 1 + k);
            let docked = match pickup {
                // Whole layers strictly between release and pickup.
                Some(j) => cum_secs[events[j].0.saturating_sub(1)] - cum_secs[li],
                // Released for good: it would otherwise sit at print
                // temperature until the end g-code shuts it off.
                None => total_secs - cum_secs[li],
            };
            if docked <= s.standby_after_s {
                continue;
            }
            drop_at_swap.insert(i, from);
            if let Some(j) = pickup {
                let lj = events[j].0;
                confirm_at_swap.insert(j, from);
                // A layer ahead of the pickup — never before the release.
                let rl = lj.saturating_sub(1).clamp(li + 1, lj);
                reheat_at_layer[rl].push(from);
            }
        }
    }
    let mut swap_seq = 0usize;
    let mut in_standby: std::collections::HashSet<u32> = std::collections::HashSet::new();

    // Identity line, in the exact shape print hosts sniff for:
    //     "; generated by <name> <version> on YYYY-MM-DD at HH:MM:SS"
    // Moonraker matches that pattern generically (its "unknown PrusaSlicer
    // fork" path, added 2025-10-05), so writing it earns a full metadata
    // card under our own name with nothing installed on the host side —
    // whereas anything else needs the host to know "Fable Slicer" by name.
    // The timestamp is the one non-reproducible line in the file (as it is
    // for every slicer); diff with `-I 'generated by'` when comparing runs.
    // UTC, since resolving a local zone would mean a tz database.
    g.comment(&format!(
        "generated by Fable Slicer {} on {}",
        env!("CARGO_PKG_VERSION"),
        utc_stamp(),
    ));
    g.comment(&format!("estimated printing time = {}", format_duration(total_secs)));
    g.comment(&format!("filament used [mm] = {fil_mm:.1}"));
    g.comment(&format!("filament used [g] = {fil_g:.1}"));
    if multi {
        for (n, mm, grams) in estimate_filament_per_tool(layers, s) {
            g.comment(&format!("filament used [T{n}] = {mm:.0} mm ({grams:.1} g)"));
        }
    }
    g.comment(&format!("total layers = {}", layers.len()));
    if s.max_volumetric_speed_mm3_s > 0.0 {
        g.comment(&format!("flow limit = {:.1} mm3/s", s.max_volumetric_speed_mm3_s));
        for (kind, nominal, clamped) in audit_flow_clamps(layers, s) {
            g.comment(&format!(
                "flow-limited: {} {:.0} -> {:.0} mm/s",
                kind_label(kind),
                nominal,
                clamped
            ));
        }
    }
    g.comment(&format!(
        "layer_h={} line_w={} walls={} top/bot={}/{} infill={} ({}) bed={}x{}",
        s.layer_height_mm,
        s.line_width_mm,
        s.wall_count,
        s.top_layers,
        s.bottom_layers,
        s.infill_density,
        s.sparse_pattern.label(),
        s.bed_size_x_mm,
        s.bed_size_y_mm
    ));
    emit_host_metadata(&mut g, layers.len(), fil_g, s);

    // Machine state we depend on, then the printer's start sequence (heating /
    // homing / PRINT_START macro).
    g.raw("G21"); // millimeters
    g.raw("G90"); // absolute XYZ
    g.raw("M83"); // relative extrusion (Klipper-recommended)
    // Chamber pre-soak. A start template positions it precisely with the
    // {chamber_soak} placeholder — the Sovol template puts it after the bed
    // soak but *before* the nozzle reaches temp, so the hot nozzle never idles
    // over the bed oozing a blob while the chamber catches up. Templates that
    // don't place it get the block appended after the start sequence (still
    // correct, just not ooze-optimal). Either way the soak is filament-gated
    // (chamber_temp_c) and emitted as a plain TEMPERATURE_WAIT — see
    // chamber_soak_block.
    let positioned_soak = s.start_gcode.contains("{chamber_soak}");
    emit_template(&mut g, &s.start_gcode, s, init, init_tool);
    if !positioned_soak {
        for line in chamber_soak_block(s).lines() {
            g.raw(line);
        }
    }
    if multi {
        // Explicit initial selection: load the first filament (idempotent under
        // Klipper toolchanger AND MMU macros). The start g-code already heated
        // the nozzle to the initial tool's first-layer temp.
        emit_toolchange(&mut g, s, init_tool, init_tool, 0.0, init.first_layer_nozzle_temp_c);
        // Independent hotends: preheat every OTHER used tool's idle setpoint so
        // the first swap doesn't wait on heat (M104 T{n} is klipper's per-tool
        // form; a tool printing layer 0 wants its adhesion temp). A shared heater
        // has no T{n} to preheat, so this is skipped entirely.
        if !single_heater {
            for &n in &used {
                if n == init_tool {
                    continue;
                }
                let tn = tools.get(n);
                let temp = if layer0_tools.contains(&n) {
                    tn.first_layer_nozzle_temp_c
                } else {
                    tn.nozzle_temp_c
                };
                g.raw(&format!("M104 T{n} S{temp}"));
            }
        }
    }
    // Motion limits, set after PRINT_START so our values win. M204 S is understood
    // by Klipper and Marlin; SQUARE_CORNER_VELOCITY is Klipper's "jerk" equivalent.
    // Acceleration then follows the feature being printed (M204 per change).
    let mut cur_accel = s.first_layer_accel_mm_s2;
    // Pressure advance in force (eased on unsupported beads, see `pa_for`); starts
    // at the initial tool's value the startup `SET_PRESSURE_ADVANCE` establishes.
    let mut cur_pa = init.pressure_advance;
    g.raw(&format!("M204 S{cur_accel:.0}"));
    g.raw(&format!("SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY={:.1}", s.jerk_mm_s));
    if let Some(c) = atd_cmd(cur_accel, s) {
        g.raw(&c);
    }
    if init.pressure_advance > 0.0 {
        g.raw(&format!("SET_PRESSURE_ADVANCE ADVANCE={:.4}", init.pressure_advance));
    }
    // Every print assumes a neutral extrude factor. Nothing in a normal print
    // sets M221, but the flow calibration tower sweeps it and only restores on
    // a COMPLETED print — a canceled tower would otherwise poison every later
    // print by up to ±15% until a firmware restart.
    g.raw("M221 S100");
    g.fan(0);
    let mut cur_fan = 0u32;
    let mut cur_tool = init_tool;
    // Single-nozzle (MMU) heater state: the one nozzle's current setpoint,
    // seeded at the initial tool's first-layer temp (the start g-code set it).
    // Unused on a toolchanger / single-tool print.
    let mut cur_set_temp = init.first_layer_nozzle_temp_c;
    // The nozzle temp a tool wants on a given layer: its first-layer (adhesion)
    // temp on layer 0, its print temp above. Drives the MMU heater ramp.
    let temp_for = |tool: u32, layer_idx: usize| -> u32 {
        let t = tools.get(tool);
        if layer_idx == 0 {
            t.first_layer_nozzle_temp_c
        } else {
            t.nozzle_temp_c
        }
    };

    let fan_duty = |frac: f64| (frac.clamp(0.0, 1.0) * 255.0).round() as u32;
    // Extra fans, gated on the printer declaring them — vanilla Klipper/Marlin
    // reads the M106 P-form as the primary fan. The aux part fan (M106 P2)
    // follows the part fan's first-layers gate in the loop below; the chamber
    // exhaust (M106 P3) runs flat for the whole print. Both shut off at the end.
    let mut cur_aux_fan = 0u32;
    if s.has_aux_fan {
        g.raw("M106 P2 S0");
    }
    if s.has_exhaust_fan && s.exhaust_fan_speed > 0.0 {
        g.raw(&format!("M106 P3 S{}", fan_duty(s.exhaust_fan_speed)));
    }
    let first_spiral = s.bottom_layers; // vase mode: first continuously-rising layer
    // Tail of the most recently printed path, for wiping the nozzle back over
    // it when the next travel retracts. Carried across layers: a layer-change
    // wipe runs one layer height above the bead, which still releases the
    // pressure while moving instead of parked on the seam.
    let mut wipe_tail: Option<Vec<Point>> = None;

    for layer in layers {
        g.comment(&format!("LAYER {} z={:.3}", layer.index, layer.print_z_mm));
        // Time-based progress for the printer's display.
        let done = if layer.index == 0 { 0.0 } else { cum_secs[layer.index - 1] };
        if total_secs > 0.0 {
            g.raw(&format!(
                "M73 P{:.0} R{:.0}",
                (done / total_secs * 100.0).min(100.0),
                ((total_secs - done) / 60.0).ceil()
            ));
        }
        // Temperature: a per-layer setpoint override wins; otherwise the
        // one-shot drop from first-layer temp to printing temp. M104 either
        // way — set without waiting. On a toolchanger every tool that printed
        // layer 0 drops: bare M104 steers the tool in hand, the T-form the docked.
        if let Some(t) = layer.temp_command_c {
            g.raw(&format!("M104 S{t:.0}"));
            cur_set_temp = t.round() as u32;
        } else if layer.index == 1 {
            if single_heater {
                // One heater: drop the loaded tool from its first-layer adhesion
                // temp to its print temp (a swap ramps the incoming tool itself).
                let target = temp_for(cur_tool, layer.index);
                if target != cur_set_temp {
                    g.raw(&format!("M104 S{target}"));
                    cur_set_temp = target;
                }
            } else if multi {
                for &n in &used {
                    let tn = tools.get(n);
                    // A tool already dropped to standby keeps its standby
                    // setpoint — its scheduled reheat targets print temp.
                    if !layer0_tools.contains(&n)
                        || tn.first_layer_nozzle_temp_c == tn.nozzle_temp_c
                        || in_standby.contains(&n)
                    {
                        continue;
                    }
                    if n == cur_tool {
                        g.raw(&format!("M104 S{}", tn.nozzle_temp_c));
                    } else {
                        g.raw(&format!("M104 T{n} S{}", tn.nozzle_temp_c));
                    }
                }
            } else if s.first_layer_nozzle_temp_c != s.nozzle_temp_c {
                g.raw(&format!("M104 S{}", s.nozzle_temp_c));
            }
        }
        // Scheduled reheats: docked tools whose pickup is a layer away come
        // back to print temperature now, so the swap doesn't wait on heat.
        for &n in &reheat_at_layer[layer.index] {
            g.raw(&format!("M104 T{n} S{}", tools.get(n).nozzle_temp_c));
            in_standby.remove(&n);
        }
        // Part cooling: off for the first `fan_off_layers`, then the normal duty
        // climbed toward `fan_max` on short layers (fan_boost) — both from the
        // tool in hand; bridges may override per path below. The ladder's
        // ceiling is deliberately NOT the bridge duty: a spool can want 90%
        // on airborne beads while tolerating far less draft on plain walls.
        let active = tools.get(cur_tool);
        let mut normal_fan = if layer.index < active.fan_off_layers {
            0
        } else {
            let duty = active.fan_speed
                + (active.fan_max - active.fan_speed).max(0.0) * layer.fan_boost;
            fan_duty(duty)
        };
        if normal_fan != cur_fan {
            g.fan(normal_fan);
            cur_fan = normal_fan;
        }
        if s.has_aux_fan {
            let aux = if layer.index < active.fan_off_layers {
                0
            } else {
                fan_duty(active.aux_fan_speed)
            };
            if aux != cur_aux_fan {
                g.raw(&format!("M106 P2 S{aux}"));
                cur_aux_fan = aux;
            }
        }

        // Spiral vase: above the solid bottom, the single wall loop climbs
        // continuously from the previous layer's top to this one's.
        if s.spiral_vase && layer.index >= first_spiral {
            if let Some(path) = spiral_loop(layer) {
                let t = tools.get(path.tool);
                let feed = feed_for(path, layer.index, layer.height_mm, layer_flow_cap_mm3_s(layer, t), t, s)
                    * layer.speed_scale;
                let coeff = config::bead_area_mm2(path.width_mm, layer.height_mm * path.height_scale)
                    / tool_area_mm2(t)
                    * flow_factor(path, t, layer.index == 0);
                g.raw(&format!(";TYPE:{}", type_label(path.kind)));
                let accel = accel_for(path.kind, layer.index, s);
                if (accel - cur_accel).abs() > 0.5 {
                    g.raw(&format!("M204 S{accel:.0}"));
                    if let Some(c) = atd_cmd(accel, s) {
                        g.raw(&c);
                    }
                    cur_accel = accel;
                }
                emit_spiral_layer(&mut g, layer, path, coeff, feed, travel_f, retract_f, s);
                continue;
            }
        }

        // LAYER CHANGE — retract BEFORE lifting to the new layer. The nozzle is
        // parked on the last bead of the previous layer still under melt
        // pressure, and that last bead is normally the outer wall's seam: a Z
        // move with a charged nozzle bleeds pressure straight down onto it, and
        // because the seam sits at the same XY every layer the ooze stacks into
        // a blob column — a seam defect the closing trim can't touch, because
        // the trim has already ended the bead by the time this happens.
        // This is the same retract the first path's travel was going to do; it
        // just has to happen on this side of the lift, so the loop below skips
        // it (`hoisted_retract`) and only unretracts on arrival.
        let first_printable = layer
            .paths
            .iter()
            .enumerate()
            .find(|(_, p)| p.points.len() >= 2)
            .map(|(i, p)| (i, p.tool));
        let mut hoisted_retract: Option<usize> = None;
        if let Some((i0, tool0)) = first_printable {
            let rlen = tools.get(cur_tool).retract_len_mm;
            // Not on a tool boundary: the toolchange block runs its own
            // pre-dock retract with the outgoing tool's distance.
            let toolchange = multi && tool0 != cur_tool;
            // Every layer change retracts, not only those whose first travel
            // was going to. Island-aware ordering can start a layer right
            // where the last one ended, and then the lift happened with a
            // charged nozzle — the exact bleed this block exists to prevent.
            // The reference slicer retracts on every layer change for the
            // same reason (retract_when_changing_layer).
            if rlen > 0.0 && !toolchange && layer.index > 0 {
                g.retract(rlen, retract_f);
                if s.wipe_mm > 0.0 {
                    if let Some(tail) = &wipe_tail {
                        for p in tail {
                            g.travel(p.x_mm(), p.y_mm(), wipe_f);
                        }
                    }
                }
                hoisted_retract = Some(i0);
            }
        }
        g.move_z(layer.print_z_mm, travel_f);
        let mut cur_z = layer.print_z_mm;

        let island_pace = island_pace_factors(layer);
        let mut cur_type: Option<&'static str> = None;
        for (i, path) in layer.paths.iter().enumerate() {
            if path.points.len() < 2 {
                continue;
            }
            let tr = &layer.travels[i];
            let mut retract_done = hoisted_retract == Some(i);
            let mut force_z = false;
            // Tool boundary: swap heads before the lead-in travel. The planned
            // travel already carries retract+hop (plan_travels is tool-aware);
            // its retract/wipe portion runs HERE, with the old tool, so the dock
            // move doesn't drag ooze — the replay below skips it and keeps only
            // the approach + unretract.
            if multi && path.tool != cur_tool {
                // The pre-dock retract clears the OLD tool's melt, so it uses the
                // outgoing tool's (cur_tool, not yet swapped) retraction distance.
                let out_retract = tools.get(cur_tool).retract_len_mm;
                if tr.retract && out_retract > 0.0 {
                    g.retract(out_retract, retract_f);
                    if s.wipe_mm > 0.0 {
                        if let Some(tail) = &wipe_tail {
                            for p in tail {
                                g.travel(p.x_mm(), p.y_mm(), wipe_f);
                            }
                        }
                    }
                    retract_done = true;
                }
                wipe_tail = None;
                let from = cur_tool;
                let to = path.tool;
                if single_heater {
                    // One shared heater: ramp to the incoming filament's temp for
                    // this layer — start it BEFORE the swap so it heats through
                    // the change, WAIT after so the purge (if any) and print are
                    // at temp. The macro does the physical swap; only a shared
                    // NOZZLE flushes, so the purge volume is zero on a shared
                    // heater with separate nozzles (its tip already holds its
                    // color, waiting only on the reheat).
                    let target = temp_for(to, layer.index);
                    let ramp = target != cur_set_temp;
                    if ramp {
                        g.raw(&format!("M104 S{target}"));
                    }
                    let purge = if purges { s.purge_volume_mm3 } else { 0.0 };
                    emit_toolchange(&mut g, s, from, to, purge, target);
                    if ramp {
                        g.raw(&format!("M109 S{target}"));
                        cur_set_temp = target;
                    }
                } else {
                    // Independent hotends: confirm the pre-reheated incoming tool
                    // is up to temp (a pickup out of standby) before docking.
                    if let Some(&n) = confirm_at_swap.get(&swap_seq) {
                        g.raw(&format!("M109 T{n} S{}", tools.get(n).nozzle_temp_c));
                    }
                    emit_toolchange(&mut g, s, from, to, 0.0, tools.get(to).nozzle_temp_c);
                }
                cur_tool = path.tool;
                // The macro moved axes under its own F words and parked Z at the
                // dock: forget the feed cache, re-issue the layer Z, and force
                // ;TYPE/M204/PA/fan back out with the new tool's values.
                g.invalidate_feed();
                force_z = true;
                cur_type = None;
                cur_accel = -1.0;
                cur_pa = -1.0;
                cur_fan = u32::MAX;
                let tn = tools.get(cur_tool);
                normal_fan = if layer.index < tn.fan_off_layers {
                    0
                } else {
                    let duty = tn.fan_speed
                        + (tn.fan_max - tn.fan_speed).max(0.0) * layer.fan_boost;
                    fan_duty(duty)
                };
                if s.has_aux_fan {
                    let aux = if layer.index < tn.fan_off_layers {
                        0
                    } else {
                        fan_duty(tn.aux_fan_speed)
                    };
                    if aux != cur_aux_fan {
                        g.raw(&format!("M106 P2 S{aux}"));
                        cur_aux_fan = aux;
                    }
                }
                // A long dock ahead for the tool just released: park it at its
                // filament's standby temperature. (MMU has one heater and an
                // empty drop map, so this never fires there.)
                if let Some(&n) = drop_at_swap.get(&swap_seq) {
                    debug_assert_eq!(n, from);
                    g.raw(&format!("M104 T{n} S{}", tools.get(n).standby_temp_c));
                    in_standby.insert(n);
                }
                swap_seq += 1;
            }
            let t = tools.get(path.tool);
            let area = tool_area_mm2(t);
            let n_pts = path.points.len();
            let n_segs = if path.closed { n_pts } else { n_pts - 1 };
            // Per-segment attribute lookup: the override when present (an overhang or
            // bridge stretch inside a continuous bead), else the whole-path kind.
            let seg = |k: usize| -> SegAttr {
                match &path.segs {
                    Some(sa) if !sa.is_empty() => sa[k.min(sa.len() - 1)],
                    _ => SegAttr { kind: path.kind, overhang: path.overhang, flow: 1.0 },
                }
            };
            // Feature label, acceleration + ATD, pressure advance, and fan for the
            // FIRST segment — set before the lead-in travel so the whole block moves as
            // one (a segmented bead re-tunes label/accel/fan at each run boundary below,
            // with no travel between runs; PA is per-path only — see `pa_for_kind`).
            // Cooling grades by how unsupported the bead is: bridges/bottom skins take
            // the bridge fan, an overhang wall grades toward it, everything else the
            // normal duty.
            let a0 = seg(0);
            set_seg_attrs(
                &mut g, a0.kind, a0.overhang, layer, normal_fan, t, s, &mut cur_type, &mut cur_accel,
                &mut cur_pa, &mut cur_fan,
                // PA is deliberately NOT set here: a SET_PRESSURE_ADVANCE
                // flushes Klipper's queue, and at this point the nozzle is
                // still parked unretracted on the path just printed — the
                // flush dwell prints a blob on the finished wall. It's issued
                // at the travel's destination instead (below), where any
                // dwell mark lands on the point the new bead immediately
                // overprints.
                None,
            );

            let z = layer.print_z_mm;
            // The first layer's speed is an absolute off the filament card —
            // one pace for every feature, chosen for adhesion. Thermal pacing
            // on top of it would quietly override the number the user set.
            // Inside a small ISLAND the island's factor is the ceiling for
            // every path — min() so a short bead keeps its own harder pace.
            let small = if layer.index == 0 {
                1.0
            } else {
                small_loop_factor(path).min(island_pace[i])
            };
            let feed =
                feed_for(path, layer.index, layer.height_mm, layer_flow_cap_mm3_s(layer, t), t, s)
                    * layer.speed_scale
                    * small;
            let coeff = config::bead_area_mm2(path.width_mm, layer.height_mm * path.height_scale) / area
                * flow_factor(path, t, layer.index == 0);
            let start = path.points[0];

            // Scarf (taper) seam: a closed outer-wall loop above the first layer
            // opens its closing leg a hair short of its start instead of butting
            // them at a point — no start/stop blob to stack into a column. The
            // whole loop then prints at one feed (its slowest segment, so an
            // overhang stretch is never outrun), which also sidesteps the slew
            // limiter. Needs room for the loop plus its overlap; too-short loops
            // and variable-width beads fall through to the normal branches.
            // Pressure-continuous entry for a joined path: no retract, no
            // travel, no unretract — one short junction bead from the donor's
            // exit (the path just emitted; `join_walls` seated its end here) to
            // this loop's seam. The E-stream never stops, so the seam never
            // sees the restart flow transient. Flow is cut to
            // [`JUNCTION_FLOW`]: the hop crosses in a few milliseconds, far
            // inside Klipper's pressure-advance smoothing window, so the
            // reduced flow barely moves the modeled pressure — while keeping
            // the crossing from overstuffing the wall gap it lands in. Each
            // branch below emits the junction itself at ITS OWN entry feed
            // (the slew-limited first piece for a segmented wall), so the
            // junction never introduces a commanded feed step at the seam —
            // the donor-side step is absorbed at square-corner velocity by
            // the two ~90° junction corners.
            let junction: Option<f64> = if path.joined {
                cur_z = z;
                let jd = dist_mm(path_end(&layer.paths[i - 1]), start);
                (jd > 1.0e-6).then_some(jd * coeff * JUNCTION_FLOW)
            } else {
                // The travel (combed route, or a retracted/z-hopped hop over a
                // void) was planned in `plan_travels` — replay it, at this
                // path's Z. Retraction distance is filament-tier: use the tool
                // in hand's.
                let rlen = tools.get(cur_tool).retract_len_mm;
                // The de-retract restores `rlen` plus the restart-extra: negative
                // de-primes (absorbing the unretract's seam blob), positive
                // compensates travel ooze. Never negative net.
                let restart_mm = (rlen + tools.get(cur_tool).retract_restart_extra_mm).max(0.0);
                if !retract_done && tr.retract && rlen > 0.0 {
                    g.retract(rlen, retract_f);
                    // Wipe back along the printed bead: the pressure-release ooze
                    // smears onto plastic instead of blobbing the seam.
                    if s.wipe_mm > 0.0 {
                        if let Some(tail) = &wipe_tail {
                            for p in tail {
                                g.travel(p.x_mm(), p.y_mm(), wipe_f);
                            }
                        }
                    }
                }
                if tr.hop && hop_mm > 0.0 && !tr.points.is_empty() {
                    // Slant the lift into the first travel leg: the nozzle
                    // leaves the just-finished seam already moving instead of
                    // dwelling over it at zero XY velocity while Z climbs
                    // (that dwell radiates heat straight onto the seam). The
                    // drop at the destination stays a plain Z move — that
                    // point is overprinted by the bead about to start.
                    let first = tr.points[0];
                    g.travel_z(first.x_mm(), first.y_mm(), z + hop_mm, travel_f);
                    for pt in &tr.points[1..] {
                        g.travel(pt.x_mm(), pt.y_mm(), travel_f);
                    }
                    g.move_z(z, travel_f);
                } else {
                    if tr.hop && hop_mm > 0.0 {
                        g.move_z(z + hop_mm, travel_f);
                    } else if force_z || (z - cur_z).abs() > 1.0e-9 {
                        g.move_z(z, travel_f);
                    }
                    for pt in &tr.points {
                        g.travel(pt.x_mm(), pt.y_mm(), travel_f);
                    }
                    if tr.hop && hop_mm > 0.0 {
                        g.move_z(z, travel_f);
                    }
                }
                cur_z = z;
                // PA from the PATH-level kind: base for a mixed wall (its
                // overhang stretches must not re-issue PA mid-bead), eased for
                // a uniform overhang loop or bridge. Issued at the travel's
                // destination so the queue-flush stall parks over the point
                // the new bead overprints, not on the finished wall. (A
                // JOINED path never reaches here — its PA is untouched, the
                // donor's base value is already correct since joins only pair
                // base-PA kinds.)
                set_seg_attrs(
                    &mut g, a0.kind, a0.overhang, layer, normal_fan, t, s, &mut cur_type,
                    &mut cur_accel, &mut cur_pa, &mut cur_fan,
                    Some((path.kind, path.overhang)),
                );
                if tr.retract && rlen > 0.0 {
                    g.unretract(restart_mm, retract_f);
                }
                None
            };
            let emit_junction = |g: &mut GcodeBuilder, entry_feed: f64| {
                if let Some(je) = junction {
                    // The marker makes the pressure-join visible to humans,
                    // tests, and g-code viewers; Klipper ignores it.
                    g.raw(";JOIN");
                    g.extrude(start.x_mm(), start.y_mm(), je, entry_feed);
                }
            };

            // Concave-corner overlap trim: a per-segment E-scale (≤1) that
            // removes the double-counted bead overlap at concave wall vertices
            // — exact swept-area physics, always on. Every polyline-E branch
            // consults it (per-segment, plain, the chord fallbacks inside the
            // arc branch); a fitted G2/G3 extrudes from
            // true arc length and needs no correction. Rings only — closed, or
            // opened a hair by the butt-seam trim (still ≥99% of a ring, same
            // winding); a genuinely open arc (a paint split) has no winding to
            // tell buried from exposed overlap. Variable-width beads keep
            // their own E model.
            let nearly_ring = !path.closed
                && n_pts >= 3
                && dist_mm(path.points[0], path.points[n_pts - 1]) <= path.width_mm;
            let ov_scale: Vec<f64> = if (path.closed || nearly_ring)
                && path.widths.is_none()
                && matches!(
                    path.kind,
                    PathKind::ExternalPerimeter | PathKind::Perimeter | PathKind::OverhangWall
                ) {
                concave_overlap_seg_scale(&path.points, path.closed, path.width_mm)
            } else {
                Vec::new()
            };
            let ov = |k: usize| ov_scale.get(k).copied().unwrap_or(1.0);

            if let Some(ws) = &path.widths {
                // Variable-width bead (a gap-fill stroke, or a pinch-narrowed
                // ring): E per segment from the local width, so the bead tapers
                // continuously. Arc fitting assumes a constant width, so it's
                // skipped here.
                //
                // The path-level feed was flow-capped against the MEAN width;
                // a segment wider than the mean would exceed the melt ceiling
                // at that feed (the extruder skips, then the pressure spike
                // blobs downstream) — re-clamp per segment from its own width.
                //
                // A widths ring may ALSO carry `segs` (a pinch-kept stretch
                // bridging a lintel is stamped OverhangWall): each segment
                // takes its attr's feed — the overhang slowdown — and attr
                // boundaries re-tune label/accel/fan exactly like the plain
                // segmented branch below. These rings are resampled at
                // PINCH_STEP_MM, so slew-limiting per segment is as fine as
                // the piece planner; the sweep only runs when attrs vary
                // (a plain tapered bead emits byte-identically).
                let h = layer.height_mm * path.height_scale;
                let flow_cap = layer_flow_cap_mm3_s(layer, t);
                let width_feed = |w: f64, base: f64, ffk: f64| -> f64 {
                    if flow_cap > 0.0 {
                        let mm3_per_mm = config::bead_area_mm2(w, h) * ffk;
                        base.min(60.0 * flow_cap / mm3_per_mm.max(1.0e-9))
                    } else {
                        base
                    }
                };
                let has_segs = path.segs.as_ref().is_some_and(|sa| !sa.is_empty());
                let seg_w = |k: usize| (ws[k] + ws[(k + 1) % n_pts]) * 0.5;
                let seg_ff =
                    |k: usize| flow_factor_kind(seg(k).kind, path.flow, t, layer.index == 0) * seg(k).flow as f64;
                let mut feeds: Vec<f64> = (0..n_segs)
                    .map(|k| {
                        let base = if has_segs {
                            let a = seg(k);
                            feed_for_seg(a.kind, a.overhang, path, layer.index, layer.height_mm, flow_cap, t, s)
                                * layer.speed_scale
                                * small
                        } else {
                            feed
                        };
                        width_feed(seg_w(k), base, seg_ff(k))
                    })
                    .collect();
                if has_segs {
                    let len = |k: usize| dist_mm(path.points[k], path.points[(k + 1) % n_pts]);
                    let sweeps = if path.closed { 2 * n_segs } else { n_segs };
                    for i in 1..sweeps {
                        let (p, k) = ((i - 1) % n_segs, i % n_segs);
                        feeds[k] = feeds[k].min(feeds[p] + FEED_SLEW_PER_MM * len(k));
                    }
                    for i in (0..sweeps.saturating_sub(1)).rev() {
                        let (k, nx) = (i % n_segs, (i + 1) % n_segs);
                        feeds[k] = feeds[k].min(feeds[nx] + FEED_SLEW_PER_MM * len(k));
                    }
                }
                let mut run = seg(0);
                let mut prev = start;
                for k in 0..n_segs {
                    if has_segs {
                        let a = seg(k);
                        if a != run {
                            set_seg_attrs(
                                &mut g, a.kind, a.overhang, layer, normal_fan, t, s, &mut cur_type,
                                &mut cur_accel, &mut cur_pa, &mut cur_fan,
                                // Mid-bead: never touch PA (planner flush = blob).
                                None,
                            );
                            run = a;
                        }
                    }
                    let c = config::bead_area_mm2(seg_w(k), h) / area * seg_ff(k);
                    let p = path.points[(k + 1) % n_pts];
                    g.extrude(p.x_mm(), p.y_mm(), dist_mm(prev, p) * c, feeds[k]);
                    prev = p;
                }
            } else if path.segs.is_some() {
                // Per-segment attributes: sub-block the bead into runs of equal
                // (kind, overhang), retuning feed/flow — and via set_seg_attrs the
                // accel/fan (never PA) — at each run boundary, without lifting the nozzle.
                // No arc fitting (feed varies within the path). The first run's
                // attributes were already set before the lead-in travel above.
                let flow_cap = layer_flow_cap_mm3_s(layer, t);
                let bead = config::bead_area_mm2(path.width_mm, layer.height_mm * path.height_scale);
                // Per-segment target feeds, then SLEW-LIMITED along the bead:
                // a band boundary used to step the commanded feed by the full
                // wall-to-overhang gap in one junction (150 mm/s on a fast
                // profile), and pressure advance turns a step that size into a
                // visible transient — the zit ring a few cm from the seam on
                // sloped shells (the seam seeds at the rear, where shells
                // slope). The limiter plans over ~[`FEED_PIECE_MM`] pieces so
                // the ramp is continuous along the metal (limiting whole
                // segments would let a long one dump its change at a single
                // junction), and only ever LOWERS the fast side into the
                // transition — slow zones keep their full slowdown. Equal-feed
                // pieces merge back at emission, so away from transitions the
                // output is the plain segment.
                let seg_feed: Vec<f64> = (0..n_segs)
                    .map(|k| {
                        let a = seg(k);
                        feed_for_seg(a.kind, a.overhang, path, layer.index, layer.height_mm, flow_cap, t, s)
                            * layer.speed_scale
                            * small
                    })
                    .collect();
                // (piece end x/y mm, piece len mm, parent segment)
                let mut pieces: Vec<(f64, f64, f64, usize)> = Vec::with_capacity(n_segs * 2);
                for k in 0..n_segs {
                    let a = path.points[k];
                    let b = path.points[(k + 1) % n_pts];
                    let len = dist_mm(a, b);
                    let np = ((len / FEED_PIECE_MM).ceil() as usize).clamp(1, FEED_PIECES_MAX);
                    let (ax, ay) = (a.x_mm(), a.y_mm());
                    let (dx, dy) = (b.x_mm() - ax, b.y_mm() - ay);
                    for i in 1..=np {
                        let f = i as f64 / np as f64;
                        pieces.push((ax + dx * f, ay + dy * f, len / np as f64, k));
                    }
                }
                let n_pc = pieces.len();
                let mut pf: Vec<f64> = pieces.iter().map(|p| seg_feed[p.3]).collect();
                // Two wrap-around sweeps each way settle the cyclic constraint.
                let sweeps = if path.closed { 2 * n_pc } else { n_pc };
                for i in 1..sweeps {
                    let (p, k) = ((i - 1) % n_pc, i % n_pc);
                    pf[k] = pf[k].min(pf[p] + FEED_SLEW_PER_MM * pieces[k].2);
                }
                for i in (0..sweeps.saturating_sub(1)).rev() {
                    let (k, nx) = (i % n_pc, (i + 1) % n_pc);
                    pf[k] = pf[k].min(pf[nx] + FEED_SLEW_PER_MM * pieces[k].2);
                }
                let mut run = seg(0);
                let mut rcoeff =
                    bead / area * flow_factor_kind(run.kind, path.flow, t, layer.index == 0) * run.flow as f64;
                // A joined entry lands at the slew-limited first piece's feed —
                // no commanded step at the seam.
                emit_junction(&mut g, pf[0]);
                // Emit, merging consecutive pieces that share feed + segment
                // (collinear by construction).
                let mut i = 0usize;
                while i < n_pc {
                    let k = pieces[i].3;
                    let a = seg(k);
                    if a != run {
                        set_seg_attrs(
                            &mut g, a.kind, a.overhang, layer, normal_fan, t, s, &mut cur_type,
                            &mut cur_accel, &mut cur_pa, &mut cur_fan,
                            // Mid-bead: never touch PA (planner flush = blob).
                            None,
                        );
                        rcoeff =
                            bead / area * flow_factor_kind(a.kind, path.flow, t, layer.index == 0) * a.flow as f64;
                        run = a;
                    }
                    let f = pf[i];
                    let mut len = pieces[i].2;
                    let mut end = i;
                    while end + 1 < n_pc
                        && pieces[end + 1].3 == k
                        && (pf[end + 1] - f).abs() < 0.5
                    {
                        end += 1;
                        len += pieces[end].2;
                    }
                    let (ex, ey, ..) = pieces[end];
                    g.extrude(ex, ey, len * rcoeff * ov(k), f);
                    i = end + 1;
                }
            } else if s.arc_fitting {
                // A fitted G2/G3 already extrudes from true arc length (see
                // emit_arcs), which equals the exact swept-annulus area — no
                // vertex double-counting there. Chords that carry a concave
                // overlap trim stay G1 (arcs are forbidden from spanning them)
                // and apply their scale.
                emit_junction(&mut g, feed);
                emit_arcs(&mut g, &path.points, path.closed, coeff, feed, s.arc_tolerance_mm, &ov_scale);
            } else {
                emit_junction(&mut g, feed);
                let mut prev = start;
                for k in 0..n_pts - 1 {
                    let p = path.points[k + 1];
                    g.extrude(p.x_mm(), p.y_mm(), dist_mm(prev, p) * coeff * ov(k), feed);
                    prev = p;
                }
                if path.closed {
                    g.extrude(start.x_mm(), start.y_mm(), dist_mm(prev, start) * coeff * ov(n_pts - 1), feed);
                }
            }
            if path.joined {
                // Exit relocation — the PA-flush cut. The loop just finished at
                // full pressure a trim short of its seam; stopping HERE parks
                // the flush ooze on the seam column (and an unretracted glide
                // exit drools there — sporadic seam zits), while a backward
                // wipe would drag ooze across the fresh closure (sporadic
                // near-seam pockmarks). Instead dive, without extruding, back
                // across the junction chord to the donor's exit: the same
                // in-material hop `join_walls` verified, reversed. The stop,
                // its pressure-advance flush, and any following retract all
                // happen buried in the wall gap, never on the visible wall.
                let exit = path_end(&layer.paths[i - 1]);
                g.travel(exit.x_mm(), exit.y_mm(), feed);
            }
            // A joined loop dove into the gap — a wipe would climb back out
            // onto the wall.
            wipe_tail = if path.joined { None } else { compute_wipe_tail(path, s.wipe_mm) };
        }
    }

    let end_retract = tools.get(cur_tool).retract_len_mm;
    if end_retract > 0.0 {
        g.retract(end_retract, retract_f);
    }
    if total_secs > 0.0 {
        g.raw("M73 P100 R0");
    }
    // Leave no P-addressed fan running: end g-code macros handle the primary
    // fan (M107) but don't know about these.
    if cur_aux_fan != 0 {
        g.raw("M106 P2 S0");
    }
    if s.has_exhaust_fan && s.exhaust_fan_speed > 0.0 {
        g.raw("M106 P3 S0");
    }
    if multi && !single_heater {
        // Independent hotends: leave no idle heater on — the end template only
        // knows the active tool. A shared heater is a single one the end
        // template already turns off, so there is nothing per-tool to shut down.
        for &n in &used {
            g.raw(&format!("M104 T{n} S0"));
        }
    }
    emit_template(&mut g, &s.end_gcode, s, init, init_tool);

    g.finish()
}

/// The single closed external-perimeter loop of a vase layer, if the layer is
/// spiralable (exactly one printable path of that shape).
fn spiral_loop(layer: &LayerPlan) -> Option<&ToolPath> {
    let mut it = layer.paths.iter().filter(|p| p.points.len() >= 2);
    match (it.next(), it.next()) {
        (Some(p), None) if p.closed && p.kind == PathKind::ExternalPerimeter => Some(p),
        _ => None,
    }
}

/// Print one vase layer: walk the loop once, ramping Z linearly with the
/// distance traveled so the wall rises in one continuous helix (no seam, no
/// layer-change retraction).
#[allow(clippy::too_many_arguments)]
fn emit_spiral_layer(
    g: &mut GcodeBuilder,
    layer: &LayerPlan,
    path: &ToolPath,
    coeff: f64,
    feed: f64,
    travel_f: f64,
    retract_f: f64,
    s: &Settings,
) {
    let z_top = layer.print_z_mm;
    let z_bot = z_top - layer.height_mm;
    // Reach the loop start (real travel only on the first spiral layer — after
    // that the nozzle is already at the loop's start/end point).
    if let Some(tr) = layer.travels.first() {
        if !tr.points.is_empty() {
            if tr.retract && s.retract_len_mm > 0.0 {
                g.retract(s.retract_len_mm, retract_f);
            }
            for pt in &tr.points {
                g.travel(pt.x_mm(), pt.y_mm(), travel_f);
            }
            if tr.retract && s.retract_len_mm > 0.0 {
                g.unretract((s.retract_len_mm + s.retract_restart_extra_mm).max(0.0), retract_f);
            }
        }
    }
    let pts = &path.points;
    let n = pts.len();
    let total: f64 = (0..n).map(|k| dist_mm(pts[k], pts[(k + 1) % n])).sum();
    if total < 1.0e-6 {
        return;
    }
    let mut cum = 0.0;
    for k in 1..=n {
        let a = pts[k - 1];
        let b = pts[k % n];
        let d = dist_mm(a, b);
        if d < 1.0e-9 {
            continue;
        }
        cum += d;
        let z = z_bot + (z_top - z_bot) * (cum / total);
        g.extrude_z(b.x_mm(), b.y_mm(), z, d * coeff, feed);
    }
}

/// Substitute `{placeholders}` in a template and emit it line by line.
/// Nozzle temperatures resolve from the initial tool's slot — the head the
/// start macro heats first (toolchanger START_PRINT macros take INITIAL_TOOL=).
fn emit_template(
    g: &mut GcodeBuilder,
    template: &str,
    s: &Settings,
    init: &ToolSettings,
    init_tool: u32,
) {
    let text = substitute(template, s, init, init_tool);
    for line in text.lines() {
        if !line.trim().is_empty() {
            g.raw(line.trim_end());
        }
    }
}

/// The chamber pre-soak block for the current settings, or empty when the
/// filament wants no soak (`chamber_temp_c == 0`, e.g. PLA — which then needs
/// no chamber sensor at all, so the slice references none).
///
/// A plain `TEMPERATURE_WAIT` on the named sensor *is* the requirement: Klipper
/// waits when the sensor exists and aborts the print ("Unknown sensor …") when
/// it doesn't — exactly the fail-on-missing we want, with no printer-side macro
/// to install. The friendly version of that failure is raised *before* sending,
/// by pinging Moonraker for the sensor (see
/// `printhost::Client::ensure_chamber_sensor`); this g-code line is the backstop
/// for a file sliced and run by hand. A profile that names no sensor at all
/// waits on an empty name and still aborts — the pre-send check turns that into
/// a clear message.
fn chamber_soak_block(s: &Settings) -> String {
    if s.chamber_temp_c == 0 {
        return String::new();
    }
    format!(
        "; chamber pre-soak: hold until the bed drives the chamber to {temp} C\n\
         TEMPERATURE_WAIT SENSOR=\"temperature_sensor {sensor}\" MINIMUM={temp}",
        sensor = s.chamber_sensor.trim(),
        temp = s.chamber_temp_c
    )
}

fn substitute(template: &str, s: &Settings, init: &ToolSettings, init_tool: u32) -> String {
    template
        .replace("{nozzle_temp}", &init.nozzle_temp_c.to_string())
        .replace("{first_layer_nozzle_temp}", &init.first_layer_nozzle_temp_c.to_string())
        .replace("{initial_tool}", &init_tool.to_string())
        .replace("{tool_count}", &s.tool_count.to_string())
        .replace("{bed_temp}", &s.bed_temp_c.to_string())
        .replace("{chamber_temp}", &s.chamber_temp_c.to_string())
        .replace("{chamber_soak}", &chamber_soak_block(s))
        .replace("{bed_x}", &format!("{:.3}", s.bed_size_x_mm))
        .replace("{bed_y}", &format!("{:.3}", s.bed_size_y_mm))
        .replace("{bed_z}", &format!("{:.3}", s.bed_size_z_mm))
        .replace("{layer_height}", &format!("{:.3}", s.layer_height_mm))
        .replace("{first_layer_height}", &format!("{:.3}", s.first_layer_height_mm))
        .replace("{nozzle_diameter}", &format!("{:.3}", s.nozzle_diameter_mm))
}

/// The configured speed (mm/s) for a feature, before any limits. Bridge pace is
/// filament physics, so it comes from the path's tool slot.
fn nominal_speed_mm_s(
    kind: PathKind,
    overhang: f32,
    layer_index: usize,
    tool: &ToolSettings,
    s: &Settings,
) -> f64 {
    if layer_index == 0 {
        // Slow everywhere on layer 0, and off the CARD: how gently a material
        // must be laid onto bare plate is a property of the material, the
        // same way its bridge speed is.
        return tool.first_layer_speed_mm_s;
    }
    match kind {
        PathKind::ExternalPerimeter => s.external_perimeter_speed_mm_s,
        PathKind::Solid => s.solid_speed_mm_s,
        // Visible skins share the outer wall's pace: finish over time.
        PathKind::TopSkin | PathKind::BottomSkin => s.external_perimeter_speed_mm_s,
        // A gap bead is a thin, wiggly precision stroke — the finish pace, not
        // the interior sprint.
        PathKind::GapFill => s.external_perimeter_speed_mm_s,
        PathKind::Ironing => s.ironing_speed_mm_s,
        PathKind::Support => s.support_speed_mm_s,
        // Bridges print into air anchored on both ends.
        PathKind::Bridge => tool.bridge_speed_mm_s,
        // Internal bridge: the first buried solid layer over low-density sparse,
        // spanning mostly air between the infill lines — print it at bridge speed.
        PathKind::InternalBridge => tool.bridge_speed_mm_s,
        // Wall stretches past the layer below print at overhang tiers matched
        // to the reference profile: a fired degree means at least half the
        // bead is on air — 30 mm/s there (the reference's 50-75% tier), the
        // floor once three quarters hang (its 75-100% tier). The old
        // continuous ramp let those beads coast at 78-113 mm/s — the melt
        // has to freeze onto whatever support it has before the nozzle drags
        // it, and that takes a hard slowdown, not a graded one.
        PathKind::OverhangWall => {
            let ceiling = s.external_perimeter_speed_mm_s;
            let floor = s.overhang_speed_mm_s;
            let d = (overhang as f64).clamp(0.0, 1.0);
            let tier = if d <= 0.5 { 30.0 } else { floor };
            tier.clamp(floor, ceiling)
        }
        // Skirt is layer-0 only (handled above); listed for exhaustiveness.
        PathKind::Skirt | PathKind::Perimeter | PathKind::Infill => s.print_speed_mm_s,
    }
}

/// Extrusion-flow factor for a path beyond its w×h geometry: per-path flow
/// (ironing), per-kind flow (bridges), and the tool's filament multiplier.
fn flow_factor(path: &ToolPath, tool: &ToolSettings, first_layer: bool) -> f64 {
    flow_factor_kind(path.kind, path.flow, tool, first_layer)
}

/// [`flow_factor`] for one segment kind (so a bridge stretch inside an otherwise
/// solid bead still gets bridge flow). `path_flow` is the bead's own multiplier.
fn flow_factor_kind(kind: PathKind, path_flow: f64, tool: &ToolSettings, first_layer: bool) -> f64 {
    // Bridges and internal bridges (solid over open sparse) both span air — the
    // stretched strand wants the reduced bridge flow so it doesn't droop.
    let kind_flow = if matches!(kind, PathKind::Bridge | PathKind::InternalBridge) {
        tool.bridge_flow
    } else {
        1.0
    };
    // The first layer's own multiplier rides on top of everything else: it is
    // about what the PLATE takes (texture to fill, squash to force), not about
    // the feature being printed. Folded in here rather than at the call sites
    // so the melt-rate clamp in `feed_for_seg` sees it too — a fattened first
    // layer slows itself instead of skipping the extruder.
    let first = if first_layer { tool.first_layer_flow } else { 1.0 };
    path_flow * kind_flow * first * tool.extrusion_multiplier
}

/// The nozzle temperature for a layer: a per-layer planned override when the
/// plan carries one, else the first-layer adhesion temp (layer 0) or the
/// profile printing temperature.
fn layer_nozzle_c(layer: &LayerPlan, tool: &ToolSettings) -> f64 {
    layer.planned_temp_c.unwrap_or(if layer.index == 0 {
        tool.first_layer_nozzle_temp_c as f64
    } else {
        tool.nozzle_temp_c as f64
    })
}

/// The melt ceiling in force on a layer (mm³/s; 0 = unlimited): the profile
/// cap at the profile temperature, derated linearly when a layer's planned
/// temperature runs below base. Never raised on warmer layers — the profile
/// cap is the calibrated number.
fn layer_flow_cap_mm3_s(layer: &LayerPlan, tool: &ToolSettings) -> f64 {
    let cap = tool.max_volumetric_speed_mm3_s;
    if cap <= 0.0 {
        return 0.0;
    }
    let below = tool.nozzle_temp_c as f64 - layer_nozzle_c(layer, tool);
    if below > 0.0 {
        (cap - tool.max_flow_derate_per_c.max(0.0) * below).max(1.0)
    } else {
        cap
    }
}

/// Circumference below which a closed wall loop slows: the loop finishes
/// before its previous pass has solidified, so the bead lands on remelt and
/// the feature ovalizes and rings. Matches the reference profile's
/// small-perimeter rule (50% speed on small loops).
const SMALL_LOOP_MM: f64 = 40.0;

/// Pace factor (≤1) for a small wall feature, fading linearly from full speed
/// at [`SMALL_LOOP_MM`] down to half. Applied at emission (and to the segment
/// feeds), not inside `feed_for` — the flow-clamp audit reads `feed_for` and
/// must not report thermal pacing as a flow limit.
///
/// Measured on PATH LENGTH, not on a closed circumference. Every wall loop is
/// opened a hair short of its own start by `apply_seam_gap`, so keying this on
/// `path.closed` meant it fired for nothing — a fact hidden for as long as
/// outer walls were exempt from that trim because they scarfed. It is the
/// SIZE of a feature that says whether it can shed heat between passes;
/// whether its loop happens to be closed says nothing.
///
/// Gap fill counts too. A gap stroke lives in the sliver between two walls of
/// the same small feature, laid seconds after them into plastic that has not
/// set — and it was taking the full external-perimeter speed while the loop
/// around it was paced, which is how a 0.9 mm free-standing rib ends up
/// stirred rather than printed.
fn small_loop_factor(path: &ToolPath) -> f64 {
    if !matches!(
        path.kind,
        PathKind::ExternalPerimeter
            | PathKind::Perimeter
            | PathKind::OverhangWall
            | PathKind::GapFill
    ) {
        return 1.0;
    }
    let n = path.points.len();
    if n < 2 {
        return 1.0;
    }
    let segs = if path.closed { n } else { n - 1 };
    let mut circ = 0.0;
    for k in 0..segs {
        circ += dist_mm(path.points[k], path.points[(k + 1) % n]);
        if circ >= SMALL_LOOP_MM {
            return 1.0;
        }
    }
    (circ / SMALL_LOOP_MM).clamp(0.5, 1.0)
}

/// Per-path ISLAND pace factors for one layer. The island — a connected
/// region, one outer contour of the layer outline — is the thermal unit:
/// nothing inside a small island can shed heat any better than its outer
/// wall can, so everything printed in it paces together. Keying only on each
/// PATH's length missed exactly the biggest deposit: `spiralize_shells`
/// chains a small island's inner rings and concentric fill into one stroke
/// whose summed length clears [`SMALL_LOOP_MM`] — the outer wall slowed while
/// most of the island's volume went down at full speed.
///
/// A path's island factor is `clamp(outer perimeter / SMALL_ISLAND_MM, 0.5, 1)`
/// for the innermost outer contour containing its start — the island
/// threshold sits above the per-path one because an island deposits its
/// whole volume back-to-back with no travel elsewhere to shed heat in, so
/// the slowdown starts earlier and bites harder for the same size; emission
/// takes
/// `min(own-length factor, island factor)`, so membership only ever slows a
/// path further — a hair bead keeps its own harder pace, and behavior in
/// islands with no small outer contour is untouched (the early return).
///
/// Out of scope by design: bridges keep their calibrated strand speed
/// (slower bridging sags MORE, not less), the skirt rings the whole plate,
/// and ironing's trickle is a finish parameter.
///
/// The island fade starts at this perimeter, wider than [`SMALL_LOOP_MM`]:
/// a 29 mm island paces to 0.58 here where the per-path rule alone gave its
/// outer wall 0.72.
const SMALL_ISLAND_MM: f64 = 50.0;

fn island_pace_factors(layer: &LayerPlan) -> Vec<f64> {
    let cs = &layer.outline.contours;
    // Outer contours (even containment depth), with perimeter and area.
    let mut outers: Vec<(usize, f64, f64)> = Vec::new();
    for (ci, c) in cs.iter().enumerate() {
        if c.points.len() < 3 {
            continue;
        }
        let depth = cs
            .iter()
            .enumerate()
            .filter(|(cj, o)| *cj != ci && o.contains(c.points[0]))
            .count();
        if depth % 2 != 0 {
            continue; // a hole, not an island
        }
        let m = c.points.len();
        let mut perim = 0.0;
        let mut area2 = 0.0;
        for k in 0..m {
            let (a, b) = (c.points[k], c.points[(k + 1) % m]);
            perim += dist_mm(a, b);
            area2 += a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm();
        }
        outers.push((ci, perim, area2.abs() * 0.5));
    }
    if !outers.iter().any(|&(_, p, _)| p < SMALL_ISLAND_MM) {
        return vec![1.0; layer.paths.len()]; // no small island (the common case)
    }
    layer
        .paths
        .iter()
        .map(|path| {
            if matches!(
                path.kind,
                PathKind::Skirt | PathKind::Bridge | PathKind::InternalBridge | PathKind::Ironing
            ) || path.points.is_empty()
            {
                return 1.0;
            }
            let q = path.points[0];
            outers
                .iter()
                .filter(|&&(ci, _, _)| cs[ci].contains(q))
                .min_by(|a, b| a.2.total_cmp(&b.2))
                .map_or(1.0, |&(_, perim, _)| (perim / SMALL_ISLAND_MM).clamp(0.5, 1.0))
        })
        .collect()
}

/// Feed rate (mm/min) for a path: the per-feature speed, clamped so the
/// volumetric flow `width × height × speed × flow` never exceeds the layer's
/// melt-rate ceiling (`flow_cap_mm3_s`, 0 = unlimited — pass
/// `layer_flow_cap_mm3_s` so any sub-base temperature derate applies). One
/// function feeds the g-code, the time estimate, and the min-layer-time pass,
/// so they always agree.
fn feed_for(
    path: &ToolPath,
    layer_index: usize,
    layer_height_mm: f64,
    flow_cap_mm3_s: f64,
    tool: &ToolSettings,
    s: &Settings,
) -> f64 {
    feed_for_seg(
        path.kind, path.overhang, path, layer_index, layer_height_mm, flow_cap_mm3_s, tool, s,
    )
}

/// [`feed_for`] for one segment's `(kind, overhang)`, keeping the bead's own width /
/// height / flow — so a bridge or overhang stretch inside a continuous bead gets its
/// own speed and flow-clamp without the bead being split.
#[allow(clippy::too_many_arguments)]
fn feed_for_seg(
    kind: PathKind,
    overhang: f32,
    path: &ToolPath,
    layer_index: usize,
    layer_height_mm: f64,
    flow_cap_mm3_s: f64,
    tool: &ToolSettings,
    s: &Settings,
) -> f64 {
    let mut v = nominal_speed_mm_s(kind, overhang, layer_index, tool, s);
    if flow_cap_mm3_s > 0.0 {
        let mm3_per_mm = config::bead_area_mm2(path.width_mm, layer_height_mm * path.height_scale)
            * flow_factor_kind(kind, path.flow, tool, layer_index == 0);
        if mm3_per_mm > 1.0e-9 {
            v = v.min(flow_cap_mm3_s / mm3_per_mm);
        }
    }
    v * 60.0
}

/// Where the flow ceiling bites: per feature kind (skipping the first layer),
/// the nominal speed and the worst (slowest) clamped speed, for kinds where
/// the clamp actually engaged. Drives the loud reporting in the g-code
/// header, the CLI summary, and the GUI status line.
pub fn audit_flow_clamps(layers: &[LayerPlan], s: &Settings) -> Vec<(PathKind, f64, f64)> {
    use std::collections::BTreeMap;
    let tools = Tools::new(s);
    let mut worst: BTreeMap<&'static str, (PathKind, f64, f64)> = BTreeMap::new();
    for layer in layers {
        if layer.index == 0 {
            continue; // first layer is slowed anyway
        }
        for path in &layer.paths {
            if path.points.len() < 2 {
                continue;
            }
            let t = tools.get(path.tool);
            let nominal = nominal_speed_mm_s(path.kind, path.overhang, layer.index, t, s);
            let clamped =
                feed_for(path, layer.index, layer.height_mm, layer_flow_cap_mm3_s(layer, t), t, s)
                    / 60.0;
            if clamped < nominal - 1.0e-6 {
                worst
                    .entry(kind_label(path.kind))
                    .and_modify(|e| e.2 = e.2.min(clamped))
                    .or_insert((path.kind, nominal, clamped));
            }
        }
    }
    worst.into_values().collect()
}

/// Human-readable feature name for a path kind — used in g-code header
/// comments, the CLI flow report, and the GUI's flow-limit table.
pub fn kind_label(kind: PathKind) -> &'static str {
    match kind {
        PathKind::Skirt => "skirt",
        PathKind::ExternalPerimeter => "outer wall",
        PathKind::Perimeter => "inner wall",
        PathKind::OverhangWall => "overhang wall",
        PathKind::Solid => "solid",
        PathKind::TopSkin => "top surface",
        PathKind::BottomSkin => "bottom surface",
        PathKind::Infill => "infill",
        PathKind::GapFill => "gap fill",
        PathKind::Ironing => "ironing",
        PathKind::Support => "support",
        PathKind::Bridge => "bridge",
        PathKind::InternalBridge => "internal bridge",
    }
}

/// `;TYPE:` annotation per feature, in the names g-code viewers (Mainsail,
/// Fluidd, OrcaSlicer's own) already colour-code.
fn type_label(kind: PathKind) -> &'static str {
    match kind {
        PathKind::Skirt => "Skirt",
        PathKind::ExternalPerimeter => "Outer wall",
        PathKind::Perimeter => "Inner wall",
        PathKind::OverhangWall => "Overhang wall",
        PathKind::Solid => "Solid infill",
        PathKind::TopSkin => "Top surface",
        PathKind::BottomSkin => "Bottom surface",
        PathKind::Infill => "Sparse infill",
        PathKind::GapFill => "Gap infill",
        PathKind::Ironing => "Ironing",
        PathKind::Support => "Support",
        PathKind::Bridge => "Bridge",
        PathKind::InternalBridge => "Internal Bridge",
    }
}

/// Acceleration (mm/s²) for a feature: gentle on the first layer (adhesion)
/// and the visible outer wall (ringing); everything else at the main limit.
/// Used for both the emitted M204s and the time estimate.
fn accel_for(kind: PathKind, layer_index: usize, s: &Settings) -> f64 {
    if layer_index == 0 {
        return s.first_layer_accel_mm_s2;
    }
    match kind {
        // Visible surfaces: the outer wall for ringing, the skins because
        // turnaround transients at every fill-line end print ripple into
        // exactly the surfaces people look at.
        PathKind::ExternalPerimeter
        | PathKind::OverhangWall
        | PathKind::TopSkin
        | PathKind::BottomSkin => s.outer_wall_accel_mm_s2,
        _ => s.acceleration_mm_s2,
    }
}

/// `SET_VELOCITY_LIMIT ACCEL_TO_DECEL=…` for the accel-to-decel smoothing at this
/// acceleration, or `None` when smoothing is off. Tracks the per-feature accel so
/// the cruise ratio stays constant as the acceleration changes feature to feature.
fn atd_cmd(accel: f64, s: &Settings) -> Option<String> {
    (s.min_cruise_ratio > 1.0e-3).then(|| {
        let atd = accel * (1.0 - s.min_cruise_ratio.clamp(0.0, 0.95));
        format!("SET_VELOCITY_LIMIT ACCEL_TO_DECEL={atd:.0}")
    })
}

/// The fraction of profile pressure advance a fully-airborne bead keeps. A
/// bridge or a wall hanging fully past its support carries little nozzle
/// pressure, so full PA over-corrects at the bead ends; halving it softens the
/// flow blip where a fast supported wall steps down to a slow overhang/bridge.
const OVERHANG_PA_FLOOR: f64 = 0.5;


/// Cap on how fast the commanded feed may change along a segmented bead
/// (mm/min of feed per mm of path). Pressure advance swings filament in
/// proportion to a speed step — a 150 mm/s wall→overhang cliff at one
/// junction moves ~0.25 mm of filament in ~25 ms and prints a zit — so band
/// crossings ramp at ≤25 mm/s per mm instead: the same 150 mm/s change
/// spread over 6 mm of bead.
const FEED_SLEW_PER_MM: f64 = 25.0 * 60.0;
/// Granularity the slew limiter plans at (mm). Long segments are cut into
/// pieces this size before limiting, so a ramp really happens ALONG the
/// metal — limiting whole segments lets a long one absorb a big change on
/// paper and still dump it at a single junction. Equal-feed pieces merge
/// back at emission (they are collinear), so far from any transition the
/// g-code is unchanged. Max per-junction step ≈ slew × piece = 10 mm/s.
const FEED_PIECE_MM: f64 = 0.4;
/// Piece-count ceiling per segment (a pathological metres-long segment must
/// not explode the plan; its junction budget grows instead, rarely).
const FEED_PIECES_MAX: usize = 256;

/// Pressure advance (mm of filament per mm/s of flow) for a feature: the profile
/// value for supported moves, eased toward [`OVERHANG_PA_FLOOR`]×profile as a
/// bead loses support — a uniformly overhanging wall loop graduates with how far
/// it hangs, true bridges take the floor. Returns 0 when PA is disabled.
///
/// Applied per PATH, never per segment: `SET_PRESSURE_ADVANCE` flushes Klipper's
/// lookahead, so changing it mid-bead parks the nozzle on the wall and prints
/// the ooze as a blob (a StealthBurner cowl collected 470 across its sloped
/// surface — the band boundaries of a smooth slope wander layer to layer, so the
/// blobs read as random). A mixed wall therefore keeps base PA through its
/// overhang stretches (which still slow down and take graded cooling); only
/// paths that ARE wholly overhang/bridge — entered via a travel, where the
/// planner was stopping anyway — carry an eased value. InternalBridge stays at
/// base for the same reason (cut per-bead into the surrounding solid, it would
/// toggle the setting along a single onion); its droop is already handled by the
/// reduced bridge flow.
fn pa_for_kind(kind: PathKind, overhang: f32, tool: &ToolSettings) -> f64 {
    let base = tool.pressure_advance;
    if base <= 0.0 {
        return 0.0;
    }
    match kind {
        PathKind::Bridge => base * OVERHANG_PA_FLOOR,
        PathKind::OverhangWall => base * (1.0 - (1.0 - OVERHANG_PA_FLOOR) * overhang as f64),
        _ => base,
    }
}

/// Emit the state changes (feature label, acceleration + accel-to-decel, pressure
/// advance, fan) for a run of `(kind, overhang)`, each only when it actually changes
/// from the tracked current value. Shared by a path's first segment (set before the
/// lead-in travel) and every attribute-run boundary inside a segmented bead — so a
/// continuous loop retunes its speed/cooling mid-path without lifting the nozzle.
///
/// `pa_src` is the (kind, overhang) that PA eases from, and it is `None` at every
/// mid-bead run boundary: unlike F/M204/M106, `SET_PRESSURE_ADVANCE` flushes
/// Klipper's lookahead — a dead stop with the nozzle parked ON the wall, whose
/// ooze prints as a blob (a StealthBurner main body collected 470 of them across
/// its sloped cowl). So PA is set once per path, from the PATH-level kind (base
/// value for a mixed wall; eased for a uniform overhang loop or a bridge, whose
/// path start follows a travel where the planner was stopping anyway), and the
/// overhang stretches keep their slowdown, label, and graded cooling instead.
#[allow(clippy::too_many_arguments)]
fn set_seg_attrs(
    g: &mut GcodeBuilder,
    kind: PathKind,
    overhang: f32,
    layer: &LayerPlan,
    normal_fan: u32,
    tool: &ToolSettings,
    s: &Settings,
    cur_type: &mut Option<&'static str>,
    cur_accel: &mut f64,
    cur_pa: &mut f64,
    cur_fan: &mut u32,
    pa_src: Option<(PathKind, f32)>,
) {
    let t = type_label(kind);
    if *cur_type != Some(t) {
        g.raw(&format!(";TYPE:{t}"));
        *cur_type = Some(t);
    }
    let accel = accel_for(kind, layer.index, s);
    if (accel - *cur_accel).abs() > 0.5 {
        g.raw(&format!("M204 S{accel:.0}"));
        if let Some(c) = atd_cmd(accel, s) {
            g.raw(&c);
        }
        *cur_accel = accel;
    }
    if let Some((pk, po)) = pa_src {
        if tool.pressure_advance > 0.0 {
            let pa = pa_for_kind(pk, po, tool);
            if (pa - *cur_pa).abs() > 1.0e-4 {
                g.raw(&format!("SET_PRESSURE_ADVANCE ADVANCE={pa:.4}"));
                *cur_pa = pa;
            }
        }
    }
    let want_fan = if layer.index < tool.fan_off_layers {
        normal_fan
    } else {
        // A bead laid mostly onto AIR gets the full cooling ceiling outright
        // — the melt must freeze the moment it lands or it sags. A bead that
        // merely leans past its support does NOT: blasting 80% on every
        // gently-sloped outer wall from layer 3 up chilled an ASA part into
        // lifting off the bed (the reference print's bursts are confined to
        // its truly airborne ceiling rings, and brief). Mild overhang keeps
        // the tier speeds and the layer-time ladder duty via the max() below.
        let frac = match kind {
            PathKind::Bridge | PathKind::BottomSkin => tool.bridge_fan_speed,
            PathKind::OverhangWall if overhang as f64 > 0.5 => tool.bridge_fan_speed,
            _ => tool.fan_speed,
        };
        ((frac.clamp(0.0, 1.0) * 255.0).round() as u32).max(normal_fan)
    };
    if want_fan != *cur_fan {
        g.fan(want_fan);
        *cur_fan = want_fan;
    }
}

fn dist_mm(a: Point, b: Point) -> f64 {
    let dx = a.x_mm() - b.x_mm();
    let dy = a.y_mm() - b.y_mm();
    (dx * dx + dy * dy).sqrt()
}

/// The vertices to wipe back over after finishing `path`: walking backwards
/// from its end (the seam, for closed loops) over at most `wipe_mm` — capped
/// at half the path so short strokes don't fully retrace.
fn compute_wipe_tail(path: &ToolPath, wipe_mm: f64) -> Option<Vec<Point>> {
    let pts = &path.points;
    let n = pts.len();
    if wipe_mm <= 0.0 || n < 2 {
        return None;
    }
    let mut total: f64 = pts.windows(2).map(|w| dist_mm(w[0], w[1])).sum();
    if path.closed {
        total += dist_mm(pts[n - 1], pts[0]);
    }
    let budget = wipe_mm.min(total * 0.5);
    // Predecessors of the end point: a closed loop ends back at pts[0], an
    // open path at pts[n-1].
    let mut cur = if path.closed { pts[0] } else { pts[n - 1] };
    let idxs: Box<dyn Iterator<Item = usize>> =
        if path.closed { Box::new((1..n).rev()) } else { Box::new((0..n - 1).rev()) };
    let mut tail = Vec::new();
    let mut acc = 0.0;
    for j in idxs {
        let p = pts[j];
        let d = dist_mm(cur, p);
        if acc + d >= budget {
            let t = ((budget - acc) / d.max(1.0e-9)).clamp(0.0, 1.0);
            tail.push(Point::from_mm(
                cur.x_mm() + (p.x_mm() - cur.x_mm()) * t,
                cur.y_mm() + (p.y_mm() - cur.y_mm()) * t,
            ));
            break;
        }
        tail.push(p);
        acc += d;
        cur = p;
    }
    (!tail.is_empty()).then_some(tail)
}

/// Need at least this many points to bother emitting an arc (avoids fitting corners).
const ARC_MIN_PTS: usize = 4;
/// Above this radius (mm) a run is treated as straight, not an arc.
const ARC_MAX_R: f64 = 1000.0;

/// Emit a path's extrusion, replacing runs of points that lie on a circular arc
/// with a single G2/G3 and leaving the rest as G1 segments. `ov` is the
/// per-source-segment concave overlap scale (chord `k` ↔ segment `k`; for a
/// closed loop the appended closing chord maps to the closing segment). It
/// applies ONLY to the G1 chord fallbacks: a fitted arc's arc-length E is
/// already exact (no vertex double-count exists on it), so the trim is needed
/// exactly where fitting fails — sharp corners and coarse runs. The hug test
/// keeps genuinely sharp reflex corners out of arcs, so an arc can swallow at
/// most a few-degree vertex's worth of trim (negligible).
fn emit_arcs(
    g: &mut GcodeBuilder,
    points: &[Point],
    closed: bool,
    coeff: f64,
    feed: f64,
    tol: f64,
    ov: &[f64],
) {
    let mut pts: Vec<(f64, f64)> = points.iter().map(|p| (p.x_mm(), p.y_mm())).collect();
    if closed {
        pts.push(pts[0]); // walk the closing segment too
    }
    let n = pts.len();
    let sc = |k: usize| ov.get(k).copied().unwrap_or(1.0);
    let mut i = 0;
    while i + 1 < n {
        match fit_arc(&pts, i, tol) {
            Some((j, cx, cy, cw)) => {
                let len = arc_span_len(&pts[i..=j], cx, cy);
                g.arc(cw, pts[j].0, pts[j].1, cx - pts[i].0, cy - pts[i].1, len * coeff, feed);
                i = j;
            }
            None => {
                let (x, y) = pts[i + 1];
                g.extrude(x, y, pdist(pts[i], pts[i + 1]) * coeff * sc(i), feed);
                i += 1;
            }
        }
    }
}

/// Longest run starting at `i` that lies on one circular arc within `tol`.
/// Returns (end index, center x, center y, clockwise) or None for a straight run.
fn fit_arc(pts: &[(f64, f64)], i: usize, tol: f64) -> Option<(usize, f64, f64, bool)> {
    let n = pts.len();
    if i + 2 >= n {
        return None;
    }
    let (cx, cy) = circumcenter(pts[i], pts[i + 1], pts[i + 2])?;
    let r = pdist(pts[i], (cx, cy));
    if !(1.0e-3..=ARC_MAX_R).contains(&r) {
        return None;
    }
    let cw = turn_cw(pts[i], pts[i + 1], pts[i + 2]);
    // A segment "hugs" the arc when its chord deviates from the circle by ≤ tol
    // (sagitta = r − distance(chord midpoint, center)). This rejects polygons whose
    // vertices happen to be concyclic (e.g. a square's 4 corners) — only a densely
    // sampled curve passes.
    let hugs = |a: (f64, f64), b: (f64, f64)| {
        let m = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
        r - pdist(m, (cx, cy)) <= tol
    };
    if !hugs(pts[i], pts[i + 1]) || !hugs(pts[i + 1], pts[i + 2]) {
        return None;
    }
    // The sweep about the center must be monotone (and under a full turn):
    // a path that doubles back along the same circle stays within `tol` and
    // keeps a consistent polyline turn, but the firmware draws only the
    // start→end angular gap — the bead's full length would be squeezed into
    // a short arc as an over-extruded blob (seen on seam-adjacent jitters).
    let sweep = |a: (f64, f64), b: (f64, f64)| -> f64 {
        let v0 = (a.0 - cx, a.1 - cy);
        let v1 = (b.0 - cx, b.1 - cy);
        (v0.0 * v1.1 - v0.1 * v1.0).atan2(v0.0 * v1.0 + v0.1 * v1.1)
    };
    let dir = if cw { -1.0 } else { 1.0 };
    let s0 = sweep(pts[i], pts[i + 1]) * dir;
    let s1 = sweep(pts[i + 1], pts[i + 2]) * dir;
    if s0 <= 0.0 || s1 <= 0.0 {
        return None;
    }
    let mut total = s0 + s1;
    let mut j = i + 2;
    while j + 1 < n {
        let p = pts[j + 1];
        if (pdist(p, (cx, cy)) - r).abs() > tol || turn_cw(pts[j - 1], pts[j], p) != cw || !hugs(pts[j], p) {
            break;
        }
        let ds = sweep(pts[j], p) * dir;
        if ds <= 0.0 || total + ds >= std::f64::consts::TAU - 0.05 {
            break;
        }
        total += ds;
        j += 1;
    }
    if j + 1 - i < ARC_MIN_PTS {
        return None;
    }
    // Refit on the run's endpoints + midpoint so I/J land accurately.
    let (cx, cy) = circumcenter(pts[i], pts[(i + j) / 2], pts[j]).unwrap_or((cx, cy));
    Some((j, cx, cy, cw))
}

fn pdist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// Clockwise turn a→b→c in printer XY (G2 direction).
fn turn_cw(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0) < 0.0
}

/// Center of the circle through three points (None if collinear).
fn circumcenter(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<(f64, f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1.0e-9 {
        return None;
    }
    let (a2, b2, c2) = (a.0 * a.0 + a.1 * a.1, b.0 * b.0 + b.1 * b.1, c.0 * c.0 + c.1 * c.1);
    let ux = (a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d;
    let uy = (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d;
    Some((ux, uy))
}

/// Arc length along a run of points about center (cx, cy): radius × total swept angle.
fn arc_span_len(run: &[(f64, f64)], cx: f64, cy: f64) -> f64 {
    let r = pdist(run[0], (cx, cy));
    let mut ang = 0.0;
    for w in run.windows(2) {
        let v0 = (w[0].0 - cx, w[0].1 - cy);
        let v1 = (w[1].0 - cx, w[1].1 - cy);
        ang += (v0.0 * v1.1 - v0.1 * v1.0).atan2(v0.0 * v1.0 + v0.1 * v1.1).abs();
    }
    r * ang
}

/// Estimate print time (seconds) via a trapezoidal motion simulation:
/// acceleration-limited moves with a jerk-based junction speed limit and
/// look-ahead. Mirrors the move sequence `to_gcode` emits; heating/homing in the
/// start g-code is not counted.
pub fn estimate_seconds(layers: &[LayerPlan], s: &Settings) -> f64 {
    let tools = Tools::new(s);
    let travel_v = s.travel_speed_mm_s.max(1.0);
    let mut total = 0.0;
    let mut prev_z = 0.0;
    for (layer, entry) in layers.iter().zip(entry_tools(layers)) {
        total += (layer.print_z_mm - prev_z).abs() / travel_v;
        prev_z = layer.print_z_mm;
        total += layer_print_seconds(layer, s, &tools, layer.speed_scale, entry);
    }
    total
}

/// Extrusion + intra-layer travel time for one layer, at the given speed scale,
/// using the planned travels. Acceleration follows the per-feature M204s the
/// emitter writes, so the estimate tracks what the printer is actually told.
/// Toolchanges cost a flat `toolchange_seconds` each, charged to the layer they
/// happen in — including a lead swap when the layer opens on a different tool
/// than `entry_tool` (the one left in hand by the layer before).
fn layer_print_seconds(
    layer: &LayerPlan,
    s: &Settings,
    tools: &Tools,
    scale: f64,
    entry_tool: Option<u32>,
) -> f64 {
    let travel_v = s.travel_speed_mm_s.max(1.0);
    let retract_t = if s.retract_len_mm > 0.0 {
        2.0 * s.retract_len_mm / s.retract_speed_mm_s.max(1.0)
    } else {
        0.0
    };
    let hop_t = if s.z_hop_mm > 0.0 { 2.0 * s.z_hop_mm / travel_v } else { 0.0 };
    let mut t = 0.0;
    let mut last_pos: Option<Point> = None;
    let mut prev_path_len = 0.0_f64;
    for (i, path) in layer.paths.iter().enumerate() {
        if path.points.len() < 2 {
            continue;
        }
        let accel = accel_for(path.kind, layer.index, s).max(1.0);
        if let Some(tr) = layer.travels.get(i) {
            // Travel length: from the previous position through the route points.
            let mut len = 0.0;
            let mut prev = last_pos;
            for &pt in &tr.points {
                if let Some(p) = prev {
                    len += dist_mm(p, pt);
                }
                prev = Some(pt);
            }
            t += trapezoid_time(len, 0.0, 0.0, travel_v, accel, s.min_cruise_ratio);
            if tr.retract {
                t += retract_t;
                // The wipe back over the previous path before travelling.
                if s.wipe_mm > 0.0 && prev_path_len > 0.0 {
                    t += s.wipe_mm.min(prev_path_len * 0.5) / travel_v;
                }
            }
            if tr.hop {
                t += hop_t;
            }
        }
        t += path_extrusion_seconds(layer, i, s, tools, scale);
        last_pos = Some(path_exit(path, layer.index, s));
        prev_path_len = path.points.windows(2).map(|w| dist_mm(w[0], w[1])).sum();
    }
    if s.tool_count > 1 {
        // Per swap: the fixed change cost plus, on a shared-nozzle machine, the
        // time to extrude the static purge at the machine's max volumetric flow.
        let purge_secs = if s.purges() && s.max_volumetric_speed_mm3_s > 0.0 {
            s.purge_volume_mm3 / s.max_volumetric_speed_mm3_s
        } else {
            0.0
        };
        let per_swap = s.toolchange_seconds + purge_secs;
        if per_swap > 0.0 {
            let mut in_hand = entry_tool;
            for path in layer.paths.iter().filter(|p| printable(p)) {
                if in_hand.is_some_and(|n| n != path.tool) {
                    t += per_swap;
                }
                in_hand = Some(path.tool);
            }
        }
    }
    t
}

/// Plan the lead-in travel for every path: comb a route that stays inside the
/// part when one exists, else retract and z-hop straight over the gap. Stored on
/// each layer so g-code and the GUI preview agree.
///
/// The cross-layer chain state (where the nozzle is when a layer starts) only
/// depends on path endpoints, so it is derived sequentially first — then each
/// layer's travels (including its comb graph, the expensive part) are planned in
/// parallel.
pub(crate) fn plan_travels(plans: &mut [LayerPlan], s: &Settings) {
    // Entry state per layer: nozzle position (and tool in hand) after the
    // previous layer.
    let mut entries: Vec<(Option<Point>, Option<u32>)> = Vec::with_capacity(plans.len());
    let mut last_pos: Option<Point> = None;
    let mut last_tool: Option<u32> = None;
    for plan in plans.iter() {
        entries.push((last_pos, last_tool));
        for path in &plan.paths {
            if path.points.len() >= 2 {
                last_pos = Some(path_exit(path, plan.index, s));
                last_tool = Some(path.tool);
            }
        }
    }

    plans
        .par_iter_mut()
        .zip(entries)
        .for_each(|(plan, (entry_pos, entry_tool))| {
            plan_layer_travels(plan, entry_pos, entry_tool, s)
        });
}

/// Plan one layer's travels, starting from the given entry position.
fn plan_layer_travels(
    plan: &mut LayerPlan,
    entry_pos: Option<Point>,
    entry_tool: Option<u32>,
    s: &Settings,
) {
    let mut last_pos = entry_pos;
    let mut last_tool = entry_tool;
    // The kind the nozzle last printed — the travel policy needs to know whether
    // it is leaving a buried bead or a visible surface. `None` at layer entry
    // (the previous layer's exit) is treated as visible: the conservative side.
    let mut last_kind: Option<PathKind> = None;
    let mut travels: Vec<Travel> = Vec::with_capacity(plan.paths.len());
    let mut comb: Option<CombGraph> = None;
    // EVERY travel lifts. The nozzle never crosses the layer at bead height,
    // so nothing it passes over can be raked, smeared, or picked up — whatever
    // the travel's length or what it starts and ends on. The old rule lifted
    // only when the travel also retracted and touched a visible surface, which
    // on a dense wall field left a few tens of mm per layer skimming the
    // print at bead height for no benefit worth the exception. A pressure-join
    // is not a travel and still never lifts.
    let hop = s.z_hop_mm > 0.0;
    for path in &plan.paths {
        if path.points.len() < 2 {
            travels.push(Travel::default());
            continue;
        }
        let start = path.points[0];
        // A toolchange precedes this path: the head re-enters from the dock, so
        // the chained position is meaningless — one retracted, hopped, direct
        // move to the start (combing from the stale point would be fiction).
        let changes_tool = last_tool.is_some_and(|t| t != path.tool);
        let travel = match last_pos {
            // A pressure-joined wall consumes no travel at all: emission
            // extrudes a junction bead straight out of the previous path
            // (`join_walls` guaranteed same tool + adjacency). Empty points
            // also zero the estimator's travel/retract/wipe terms for it.
            _ if path.joined && !changes_tool => {
                Travel { points: Vec::new(), retract: false, hop: false }
            }
            _ if changes_tool => Travel {
                points: vec![start],
                retract: s.retract_len_mm > 0.0,
                hop,
            },
            None => Travel { points: vec![start], retract: false, hop },
            // The one exception to the lift: the next bead starts within a
            // bead's reach of where the last one ended, on a clear line. There
            // is nothing to clear at that range — the only thing under the
            // nozzle is the bead it just laid — and lifting would cycle Z and
            // hand the seam another chance to ooze for no gain. A short move
            // whose line is BLOCKED still lifts: 0.8 mm is plenty to step over
            // a wall's edge or a hole's rim.
            Some(prev)
                if dist_mm(prev, start) < MIN_TRAVEL_MM
                    && !travel_blocked(&plan.outline, prev, start) =>
            {
                Travel { points: vec![start], retract: false, hop: false }
            }
            // Endpoints in different islands (or off the part — a skirt/brim
            // lead-in): no in-material route can exist, so hop the gap
            // retracted WITHOUT consulting the comb graph. Asking it anyway
            // was a bug: on flat faces the edges of separate islands are
            // collinear, the router admitted diagonals along those shared
            // lines, and hole-split layers got naked travels dragged across
            // the outer face (scarred bands beside every hole).
            Some(prev)
                if island_of(&plan.outline, prev) != island_of(&plan.outline, start) =>
            {
                Travel { points: vec![start], retract: s.retract_len_mm > 0.0, hop }
            }
            Some(prev) if !travel_blocked(&plan.outline, prev, start) => {
                // A clear in-material straight: how it travels depends on WHOSE
                // material it glides over. Between two BURIED beads (internal
                // fill nobody will ever see) an unretracted glide is free — cap
                // it only at the melt-pressure drool threshold, no hop. But a
                // travel that starts or ends on anything VISIBLE — a skin, a
                // wall, gap fill — drags a pressurized tip at bead height
                // across a surface that never gets covered again: drool
                // trails and shear scars (the step-surface glides the audit
                // reproduced). Those retract past a short floor AND hop clear
                // of the bead crowns — the reference slicer's known-good
                // output on this printer retracts + lifts on every one. Vase
                // mode is exempt: its whole point is one continuous
                // unretracted extrusion.
                let d = dist_mm(prev, start);
                let surface = !(last_kind.is_some_and(buried_kind) && buried_kind(path.kind));
                let retract = !s.spiral_vase
                    && s.retract_len_mm > 0.0
                    && (d > COMB_RETRACT_MM || (surface && d > SURFACE_RETRACT_MM));
                Travel { points: vec![start], retract, hop }
            }
            Some(prev) => {
                let graph = comb.get_or_insert_with(|| CombGraph::build(&plan.outline));
                match graph.route(&plan.outline, prev, start) {
                    Some(route) => {
                        // Combing exists to glide over material unretracted,
                        // but the same surface rule as the straight glide
                        // applies: buried-to-buried combs freely (retract only
                        // past the drool threshold, no hop); a route touching
                        // visible surfaces retracts past the short floor and
                        // hops — the route still steers over the part, the
                        // lift keeps the tip off the finished crowns.
                        let mut len = 0.0;
                        let mut at = prev;
                        for &p in &route {
                            len += dist_mm(at, p);
                            at = p;
                        }
                        let surface =
                            !(last_kind.is_some_and(buried_kind) && buried_kind(path.kind));
                        let retract = s.retract_len_mm > 0.0
                            && (len > COMB_RETRACT_MM
                                || (!s.spiral_vase && surface && len > SURFACE_RETRACT_MM));
                        Travel { points: route, retract, hop }
                    }
                    None => Travel { points: vec![start], retract: s.retract_len_mm > 0.0, hop },
                }
            }
        };
        last_pos = Some(path_exit(path, plan.index, s));
        last_kind = Some(path.kind);
        last_tool = Some(path.tool);
        travels.push(travel);
    }
    plan.travels = travels;
}

/// Layer time below which the part fan starts climbing from the filament's
/// base duty toward its ceiling. A layer shorter than this hasn't solidified
/// when the next bead lands, so airflow has to carry the heat away instead
/// of time doing it.
const FAN_COOL_LAYER_TIME_S: f64 = 35.0;

/// The cooling clock: NAIVE commanded time — path length over commanded
/// feed plus travels at travel speed plus retract strokes, with no junction
/// or acceleration modeling. Deliberately not `layer_print_seconds`: the
/// 35s window is calibrated against this clock (reconstructed from the
/// reference print's emitted fan to ~2 duty-points per layer across all
/// 159 layers), and the junction-aware time reads up to ~2x longer on
/// micro-segmented layers — which silently halves the boost and leaves
/// short layers semi-molten (the shrink band above a shelf).
fn layer_naive_seconds(plan: &LayerPlan, s: &Settings, tools: &Tools) -> f64 {
    let mut t = 0.0;
    let island_pace = island_pace_factors(plan);
    for (i, path) in plan.paths.iter().enumerate() {
        if let Some(tr) = plan.travels.get(i) {
            let mut d = 0.0;
            for w in tr.points.windows(2) {
                d += dist_mm(w[0], w[1]);
            }
            t += d / s.travel_speed_mm_s.max(1.0);
            if tr.retract {
                let tool = tools.get(path.tool);
                t += 2.0 * tool.retract_len_mm / s.retract_speed_mm_s.max(1.0);
            }
        }
        if path.points.len() < 2 {
            continue;
        }
        let tool = tools.get(path.tool);
        // Mirrors emission, first layer included (see `small_loop_factor`),
        // so the estimate and the g-code cannot disagree about pacing.
        let small = if plan.index == 0 {
            1.0
        } else {
            small_loop_factor(path).min(island_pace[i])
        };
        let feed =
            feed_for(path, plan.index, plan.height_mm, layer_flow_cap_mm3_s(plan, tool), tool, s)
                * small
                / 60.0;
        if feed > 1.0e-6 {
            let mut len = 0.0;
            for w in path.points.windows(2) {
                len += dist_mm(w[0], w[1]);
            }
            if path.closed {
                len += dist_mm(*path.points.last().unwrap(), path.points[0]);
            }
            t += len / feed;
        }
    }
    t
}

/// Per-layer cooling: slow layers that print faster than `min_layer_time_s`
/// (down to a floor speed), and ramp the fan on layers shorter than the
/// cooling window so they solidify before the next layer lands.
pub(crate) fn apply_min_layer_time(plans: &mut [LayerPlan], s: &Settings) {
    let tools = Tools::new(s);
    let floor = s.min_print_speed_mm_s / s.print_speed_mm_s.max(1.0);
    // The toolchange dwell counts as cooling time here (it's inside
    // layer_print_seconds): a layer that spends the swap docked needs that much
    // less slowing.
    let entries = entry_tools(plans);
    plans.par_iter_mut().zip(entries).for_each(|(plan, entry)| {
        // Fan rides the naive clock (see `layer_naive_seconds`); the
        // min-layer-time slowdown keeps the junction-aware real seconds —
        // its floor is about how long the machine actually dwells.
        let tn = layer_naive_seconds(plan, s, &tools);
        if tn > 0.0 {
            plan.fan_boost = (1.0 - tn / FAN_COOL_LAYER_TIME_S).clamp(0.0, 1.0);
        }
        let t = layer_print_seconds(plan, s, &tools, 1.0, entry);
        if s.min_layer_time_s > 0.0 && t > 0.0 && t < s.min_layer_time_s {
            plan.speed_scale = (t / s.min_layer_time_s).clamp(floor, 1.0);
        }
    });
}

/// Time for an extrusion polyline, with junction-speed look-ahead. Both ends stop
/// (a retraction brackets every path).
fn polyline_time(pts: &[Point], closed: bool, v_seg: &[f64], accel: f64, jerk: f64, min_cruise_ratio: f64) -> f64 {
    let n_pts = pts.len();
    let count = if closed { n_pts } else { n_pts.saturating_sub(1) };
    let mut dist: Vec<f64> = Vec::with_capacity(count);
    let mut dir: Vec<(f64, f64)> = Vec::with_capacity(count);
    // Per-segment nominal (cruise) speed — one entry per surviving segment, so a
    // bead that slows over an overhang or bridge stretch is timed at the right pace.
    let mut vseg: Vec<f64> = Vec::with_capacity(count);
    for k in 0..count {
        let p0 = pts[k];
        let p1 = pts[(k + 1) % n_pts];
        let (dx, dy) = (p1.x_mm() - p0.x_mm(), p1.y_mm() - p0.y_mm());
        let d = (dx * dx + dy * dy).sqrt();
        if d < 1.0e-6 {
            continue;
        }
        dist.push(d);
        dir.push((dx / d, dy / d));
        vseg.push(v_seg[k.min(v_seg.len().saturating_sub(1))]);
    }
    let n = dist.len();
    if n == 0 {
        return 0.0;
    }

    // Entry speed at each segment: start with the junction limit (full stop at
    // the first), where a sharper corner allows a lower speed.
    let mut entry = vseg.clone();
    entry[0] = 0.0;
    for i in 1..n {
        let cos = (dir[i - 1].0 * dir[i].0 + dir[i - 1].1 * dir[i].1).clamp(-1.0, 1.0);
        let sin_half = ((1.0 - cos) * 0.5).max(0.0).sqrt();
        let vj = if sin_half < 1.0e-6 { vseg[i] } else { jerk / (2.0 * sin_half) };
        entry[i] = vj.min(vseg[i]);
    }
    // Reverse: cap so we can decelerate to the next entry over the move.
    for i in (0..n).rev() {
        let exit = if i + 1 < n { entry[i + 1] } else { 0.0 };
        entry[i] = entry[i].min((exit * exit + 2.0 * accel * dist[i]).sqrt());
    }
    // Forward: cap so it's reachable by accelerating from the previous entry.
    for i in 1..n {
        entry[i] = entry[i].min((entry[i - 1] * entry[i - 1] + 2.0 * accel * dist[i - 1]).sqrt());
    }

    let mut t = 0.0;
    for i in 0..n {
        let exit = if i + 1 < n { entry[i + 1] } else { 0.0 };
        t += trapezoid_time(dist[i], entry[i], exit, vseg[i], accel, min_cruise_ratio);
    }
    t
}

/// Time for one move of `dist` mm from `v_entry` to `v_exit`, cruising up to
/// `v_cruise`, at acceleration `accel`.
fn trapezoid_time(dist: f64, v_entry: f64, v_exit: f64, v_cruise: f64, accel: f64, min_cruise_ratio: f64) -> f64 {
    if dist <= 0.0 {
        return 0.0;
    }
    // Accel-to-decel smoothing: cap the peak as if the spike above the endpoints
    // climbed at accel·(1−ratio), so the move cruises for `ratio` of its length
    // instead of sprinting to v_cruise and braking back down. ratio 0 ⇒ cap is the
    // natural peak, i.e. no change.
    let atd = accel * (1.0 - min_cruise_ratio.clamp(0.0, 0.95));
    let cap = (atd * dist + 0.5 * (v_entry * v_entry + v_exit * v_exit)).max(0.0).sqrt();
    let vc = v_cruise.min(cap).max(v_entry).max(v_exit);
    let d_acc = ((vc * vc - v_entry * v_entry) / (2.0 * accel)).max(0.0);
    let d_dec = ((vc * vc - v_exit * v_exit) / (2.0 * accel)).max(0.0);
    if d_acc + d_dec <= dist {
        let d_cruise = dist - d_acc - d_dec;
        (vc - v_entry) / accel + d_cruise / vc.max(1.0e-6) + (vc - v_exit) / accel
    } else {
        let v_peak = (((2.0 * accel * dist + v_entry * v_entry + v_exit * v_exit) * 0.5).max(0.0)).sqrt();
        let v_peak = v_peak.max(v_entry).max(v_exit).max(1.0e-6);
        (v_peak - v_entry) / accel + (v_peak - v_exit) / accel
    }
}

/// The print's facts in the field names print hosts already parse — the
/// PrusaSlicer/Orca comment convention, which Moonraker's metadata scanner
/// (and every viewer modelled on it) reads to build a file card: layer
/// count, heights, nozzle, filament identity and the first-layer targets.
///
/// Moonraker only RUNS that parser on files whose header names a slicer from
/// its hardcoded alias list, which we are not on (it falls back to a scanner
/// that recovers little more than object height from the Z moves). We emit
/// the fields under our own name regardless — they cost a dozen comment
/// lines, they are correct for any other tool reading this convention, and
/// they light up the moment a host learns the name. See
/// docs/moonraker-metadata.md for the one-line host patch.
///
/// Multi-tool values follow the same convention as the single-tool ones:
/// per-extruder entries joined with ';', in slot order.
/// "YYYY-MM-DD at HH:MM:SS" (UTC) for the identity line — the timestamp
/// shape host scanners expect. Calendar math is Howard Hinnant's
/// civil-from-days; a clock is all std gives us, and a date crate would be
/// a dependency for one line of header.
fn utc_stamp() -> String {
    utc_stamp_at(
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0),
    )
}

/// The pure half of [`utc_stamp`]: seconds since the epoch → the stamp.
fn utc_stamp_at(secs: i64) -> String {
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Shift the epoch to 0000-03-01 so leap days land at the end of the cycle.
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097; // day of era, [0, 146_096]
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365; // [0, 399]
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // March-based month, [0, 11]
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02} at {:02}:{:02}:{:02}",
        rem / 3_600,
        (rem % 3_600) / 60,
        rem % 60
    )
}

fn emit_host_metadata(g: &mut GcodeBuilder, layer_count: usize, fil_g: f64, s: &Settings) {
    let per_tool = |f: &dyn Fn(&config::ToolSettings) -> String| -> String {
        s.tools.iter().map(|t| f(t)).collect::<Vec<_>>().join(";")
    };
    g.comment(&format!("total layers count = {layer_count}"));
    g.comment(&format!("total filament used [g] = {fil_g:.2}"));
    g.comment(&format!("layer_height = {}", s.layer_height_mm));
    g.comment(&format!("first_layer_height = {}", s.first_layer_height_mm));
    g.comment(&format!("nozzle_diameter = {}", s.nozzle_diameter_mm));
    // The class label carries the ABS/ASA family as one token; hosts expect a
    // single type word, and the spool's own name rides filament_settings_id.
    g.comment(&format!(
        "filament_type = {}",
        per_tool(&|t| t.material.label().split('/').next().unwrap_or("other").to_string())
    ));
    g.comment(&format!(
        "filament_settings_id = {}",
        per_tool(&|t| format!("\"{}\"", t.filament_name))
    ));
    g.comment(&format!(
        "first_layer_temperature = {}",
        per_tool(&|t| t.first_layer_nozzle_temp_c.to_string())
    ));
    g.comment(&format!("first_layer_bed_temperature = {}", s.bed_temp_c));
    g.comment(&format!("chamber_temperature = {}", s.chamber_temp_c));
    if !s.printer_name.is_empty() {
        g.comment(&format!("printer_settings_id = \"{}\"", s.printer_name));
    }
}

/// Format a duration like "1h 23m" / "12m 30s" / "45s".
pub fn format_duration(seconds: f64) -> String {
    let total = seconds.max(0.0).round() as u64;
    let (h, m, sec) = (total / 3600, (total % 3600) / 60, total % 60);
    if h > 0 {
        format!("{h}h {m:02}m")
    } else if m > 0 {
        format!("{m}m {sec:02}s")
    } else {
        format!("{sec}s")
    }
}

/// Deposited bead volume (mm³) for one path — honors per-path flow (ironing),
/// bridge flow, and the extrusion multiplier. Shared by the
/// filament estimate and the heat stats so the preview maps can't disagree
/// with the totals.
fn path_volume_mm3(path: &ToolPath, layer_height_mm: f64, tool: &ToolSettings, first_layer: bool) -> f64 {
    let bead = config::bead_area_mm2(path.width_mm, layer_height_mm * path.height_scale);
    let n = path.points.len();
    if n < 2 {
        return 0.0;
    }
    // Per-segment when the attrs channel is present (a bridge stretch or a
    // derated turnaround chord inside a continuous bead), else one flat factor.
    if let Some(sa) = path.segs.as_ref().filter(|sa| !sa.is_empty()) {
        let count = if path.closed { n } else { n - 1 };
        let mut v = 0.0;
        for k in 0..count {
            let a = sa[k.min(sa.len() - 1)];
            v += dist_mm(path.points[k], path.points[(k + 1) % n])
                * bead
                * flow_factor_kind(a.kind, path.flow, tool, first_layer)
                * a.flow as f64;
        }
        return v;
    }
    let mut len = 0.0;
    for w in path.points.windows(2) {
        len += dist_mm(w[0], w[1]);
    }
    if path.closed {
        len += dist_mm(path.points[n - 1], path.points[0]);
    }
    len * bead * flow_factor(path, tool, first_layer)
}

/// Extrusion-only seconds for one path at a total speed multiplier — the
/// trapezoid simulation the layer estimate uses, exposed so heat
/// control budgets with the same numbers.
fn path_extrusion_seconds(
    layer: &LayerPlan,
    i: usize,
    s: &Settings,
    tools: &Tools,
    total_scale: f64,
) -> f64 {
    let path = &layer.paths[i];
    if path.points.len() < 2 {
        return 0.0;
    }
    let tool = tools.get(path.tool);
    let accel = accel_for(path.kind, layer.index, s).max(1.0);
    let cap = layer_flow_cap_mm3_s(layer, tool);
    // Per-segment cruise speed: an overhang / bridge stretch inside the bead is timed
    // at its own (slower) pace, else the whole bead runs at the path speed.
    let n_pts = path.points.len();
    let count = if path.closed { n_pts } else { n_pts - 1 };
    let v_seg: Vec<f64> = (0..count)
        .map(|k| {
            let (kind, oh) = match &path.segs {
                Some(sa) if !sa.is_empty() => {
                    let a = sa[k.min(sa.len() - 1)];
                    (a.kind, a.overhang)
                }
                _ => (path.kind, path.overhang),
            };
            feed_for_seg(kind, oh, path, layer.index, layer.height_mm, cap, tool, s) / 60.0
                * total_scale
        })
        .collect();
    polyline_time(&path.points, path.closed, &v_seg, accel, s.jerk_mm_s.max(0.1), s.min_cruise_ratio)
}

/// Estimate filament used: `(length_mm, grams)` over all tools. Honors per-path
/// flow (ironing), bridge flow, and each tool's extrusion multiplier /
/// filament diameter / density.
pub fn estimate_filament(layers: &[LayerPlan], s: &Settings) -> (f64, f64) {
    estimate_filament_per_tool(layers, s)
        .iter()
        .fold((0.0, 0.0), |acc, (_, mm, grams)| (acc.0 + mm, acc.1 + grams))
}

/// Filament per used tool: `(tool, length_mm, grams)`, ascending tool number.
/// Sums to [`estimate_filament`]; drives the per-slot header comments and any
/// "will slot N run out" arithmetic.
pub fn estimate_filament_per_tool(layers: &[LayerPlan], s: &Settings) -> Vec<(u32, f64, f64)> {
    use std::collections::BTreeMap;
    let tools = Tools::new(s);
    let mut volume: BTreeMap<u32, f64> = BTreeMap::new();
    for layer in layers {
        // Per-layer subtotals, added layer by layer (keeps the aggregate's
        // summation order — and its bytes — on a single-tool print).
        let mut layer_vol: BTreeMap<u32, f64> = BTreeMap::new();
        for path in layer.paths.iter().filter(|p| printable(p)) {
            *layer_vol.entry(path.tool).or_insert(0.0) +=
                path_volume_mm3(path, layer.height_mm, tools.get(path.tool), layer.index == 0);
        }
        for (n, v) in layer_vol {
            *volume.entry(n).or_insert(0.0) += v;
        }
    }
    // Shared nozzle: every swap wastes a static purge of the INCOMING filament.
    // Attribute it to that tool so per-slot grams and runout math see the true
    // draw. (Separate-nozzle machines never flush, so this is skipped.)
    if s.purges() && s.purge_volume_mm3 > 0.0 {
        let mut in_hand: Option<u32> = Some(initial_tool(layers));
        for layer in layers {
            for path in layer.paths.iter().filter(|p| printable(p)) {
                if in_hand.is_some_and(|n| n != path.tool) {
                    *volume.entry(path.tool).or_insert(0.0) += s.purge_volume_mm3;
                }
                in_hand = Some(path.tool);
            }
        }
    }
    volume
        .into_iter()
        .map(|(n, v)| {
            let tool = tools.get(n);
            (n, v / tool_area_mm2(tool), v / 1000.0 * tool.filament_density_g_cm3)
        })
        .collect()
}

/// Per-layer numbers behind the preview layer-time map.
pub struct LayerStats {
    /// Seconds printing this layer (its Z move + extrusion + travels).
    pub secs: f64,
}

/// Time per layer. Mirrors [`estimate_seconds`]'s per-layer terms, so the
/// preview layer-time map tracks what the totals and M73 progress report.
pub fn per_layer_stats(layers: &[LayerPlan], s: &Settings) -> Vec<LayerStats> {
    let tools = Tools::new(s);
    let travel_v = s.travel_speed_mm_s.max(1.0);
    let mut prev_z = 0.0;
    let mut out = Vec::with_capacity(layers.len());
    for (layer, entry) in layers.iter().zip(entry_tools(layers)) {
        let mut secs = (layer.print_z_mm - prev_z).abs() / travel_v;
        prev_z = layer.print_z_mm;
        secs += layer_print_seconds(layer, s, &tools, layer.speed_scale, entry);
        out.push(LayerStats { secs });
    }
    out
}

const MIN_TRAVEL_MM: f64 = 0.8;

/// A combed travel longer than this retracts anyway: gliding unretracted over
/// material is combing's point, but a long enough glide drools the melt
/// pressure out of the nozzle and the next bead starts starved.
const COMB_RETRACT_MM: f64 = 30.0;

/// Retract floor for a travel that starts or ends on a VISIBLE surface (see
/// [`buried_kind`]). Buried-to-buried travels keep the generous
/// [`COMB_RETRACT_MM`] glide; anything that could scar a finished face
/// retracts almost immediately instead of dragging a pressurized tip across
/// it. Above [`MIN_TRAVEL_MM`], so sub-bead repositioning still glides.
const SURFACE_RETRACT_MM: f64 = 1.5;

/// Is this bead BURIED — swallowed by later layers, so a nozzle gliding over
/// it leaves nothing anyone will see? Only sparse infill and buried solid
/// qualify. Everything else is visible on the finished part (walls and skins
/// obviously; gap fill and bridges sit in or under visible faces; ironing runs
/// over the top surface by definition), or is laid on bare bed (skirt/brim),
/// where dragging melt is just as ugly.
fn buried_kind(kind: PathKind) -> bool {
    matches!(kind, PathKind::Infill | PathKind::Solid)
}

fn path_end(p: &ToolPath) -> Point {
    if p.closed {
        p.points[0]
    } else {
        p.points[p.points.len() - 1]
    }
}

/// The fixed butt-seam trim, as a fraction of a line width: the closing bead
/// stops this short of the seam start so its end-cap doesn't pile onto it.
/// Fixed, not a knob — the end-cap it cancels is bead geometry, invariant
/// across prints (the old tunable version was a knife edge only because it was
/// also fighting the restart underprime, which the pressure-join removes).
const SEAM_TRIM_LW: f64 = 0.1;

/// Junction-bead flow factor for a pressure-joined wall entry. The crossing is
/// ~one bead spacing at wall speed — a few milliseconds, far inside Klipper's
/// pressure-advance smoothing window — so cutting commanded flow here barely
/// moves the modeled pressure; it just keeps the crossing from overstuffing
/// the (already area-exact) wall gap it lands in. What matters is that the
/// E-stream never stops. 0.30 balances the two failure modes: higher overfills
/// the shoulder against the seam bead, lower starves the smoothed E-stream
/// enough to pockmark the wall just past the seam (seen at 0.15 on the Sovol).
const JUNCTION_FLOW: f64 = 0.30;

/// Seam anti-zit: open each closed wall loop a hair short of its own start so
/// the closing bead stops before the seam point instead of butting onto it and
/// piling a zit there. JOINED loops take the trim too: entered at pressure the closing bead
/// arrives at FULL flow, so butting it onto the (also full) entry bead piles
/// exactly the end-cap this trim cancels — the first Sovol reprint showed it
/// as minor seam zits. The concave overlap trim survives the opening (the
/// nearly-closed-ring gate in `to_gcode`). Runs after seam placement and
/// BEFORE `plan_travels`, so travel and wipe chain from the trimmed end
/// ([`path_end`] returns the real last point once the loop is opened).
/// Variable-width beads (gap fill) have no visible seam and are skipped, and
/// spiral vase must keep its loops closed for the spiral emitter.
pub(crate) fn apply_seam_gap(plans: &mut [LayerPlan], s: &Settings) {
    if s.spiral_vase {
        return;
    }
    let gap = SEAM_TRIM_LW * s.line_width_mm;
    for plan in plans.iter_mut() {
        for path in plan.paths.iter_mut() {
            if path.closed
                && path.widths.is_none()
                && matches!(
                    path.kind,
                    PathKind::ExternalPerimeter | PathKind::Perimeter | PathKind::OverhangWall
                )
            {
                trim_seam_gap(path, gap);
            }
        }
    }
}

/// Open a closed loop `gap` mm short of its start so the closing segment isn't
/// re-emitted onto the seam start (no start/close overlap — the anti-zit trim).
/// A per-segment loop keeps its `segs` (the clamped lookup in `to_gcode` covers
/// the reshaped last leg).
fn trim_seam_gap(path: &mut ToolPath, gap: f64) {
    let n = path.points.len();
    if n < 3 {
        return;
    }
    let start = path.points[0];
    if gap > 0.0 {
        // Trim short: stop `gap` before the start on the closing leg.
        let last = path.points[n - 1];
        let seg = dist_mm(last, start);
        if seg <= 1.0e-6 {
            return;
        }
        if gap >= seg {
            // The gap would eat the whole closing leg — just drop the close.
            path.closed = false;
            return;
        }
        let f = (seg - gap) / seg; // fraction from `last` toward `start`
        let end = Point::from_mm(
            last.x_mm() + (start.x_mm() - last.x_mm()) * f,
            last.y_mm() + (start.y_mm() - last.y_mm()) * f,
        );
        path.points.push(end);
        path.closed = false;
    }
}

/// Per-segment extrusion scale (≤1) that removes the double-counted bead overlap
/// at CONCAVE (reflex) vertices of a closed wall loop — the excess that blobs on
/// the outer surface of a concave curve. Convex vertices stay at 1.0 (their
/// overlap is buried, and trimming there would open a visible gap). At a vertex
/// of deflection θ the overlap is `(w/4)·tan(θ/2)` of E-length — exact
/// swept-area physics (on a curve of radius R the reduction is `w/(8R)` per
/// unit length, exactly the curvature-driven over-extrusion, which is why a
/// fitted arc's arc-length E needs no correction), so it applies always, with
/// no strength dial. The trim splits half/half across the entering and leaving
/// segments: a short exit segment isn't starved to zero, and the flow step PA
/// prints at the corner is halved. Returns a vec of length `n_segs`.
fn concave_overlap_seg_scale(pts: &[Point], closed: bool, width: f64) -> Vec<f64> {
    let n = pts.len();
    let n_segs = if closed { n } else { n.saturating_sub(1) };
    let mut scale = vec![1.0f64; n_segs];
    if n < 3 {
        return scale;
    }
    // Winding (shoelace): positive twice-area = CCW. The sum wraps — for an
    // OPEN path that adds a virtual closing edge, which is exactly right for
    // the callers' nearly-closed rings (a butt-seam-trimmed wall loop): the
    // winding is the ring's. Open paths trim INTERIOR vertices only — the two
    // seam ends are bead tips, not corners.
    let mut area2 = 0.0;
    for k in 0..n {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        area2 += a.x_mm() * b.y_mm() - b.x_mm() * a.y_mm();
    }
    let ccw = area2 > 0.0;
    let (v0, v1) = if closed { (0, n) } else { (1, n - 1) };
    for i in v0..v1 {
        let prev = pts[(i + n - 1) % n];
        let cur = pts[i];
        let next = pts[(i + 1) % n];
        let (ax, ay) = (cur.x_mm() - prev.x_mm(), cur.y_mm() - prev.y_mm());
        let (bx, by) = (next.x_mm() - cur.x_mm(), next.y_mm() - cur.y_mm());
        let la = (ax * ax + ay * ay).sqrt();
        let lb = (bx * bx + by * by).sqrt();
        if la < 1.0e-9 || lb < 1.0e-9 {
            continue;
        }
        let cross = ax * by - ay * bx; // >0 = left turn
        if cross == 0.0 {
            continue; // straight — no overlap
        }
        // Concave = the turn opposes the loop's winding (a reflex vertex).
        if (cross > 0.0) == ccw {
            continue; // convex — leave it alone
        }
        // tan(θ/2) = |sinθ| / (1 + cosθ), clamped near the 180° blowup.
        let c = (ax * bx + ay * by) / (la * lb); // cos θ
        let s = cross.abs() / (la * lb); // |sin θ|
        let half_tan = if 1.0 + c > 1.0e-6 { (s / (1.0 + c)).min(4.0) } else { 4.0 };
        let trim = (width * 0.25) * half_tan; // E-length removed at the vertex
        let leave = i % n_segs; // the segment leaving vertex i, length lb
        let enter = (i + n_segs - 1) % n_segs; // the segment entering it, length la
        scale[leave] = (scale[leave] - 0.5 * trim / lb).max(0.0);
        scale[enter] = (scale[enter] - 0.5 * trim / la).max(0.0);
    }
    scale
}


/// Nozzle position after a path's emission: the plain loop start (closed) or
/// the last point. Travel planning and time estimates chain from here.
fn path_exit(p: &ToolPath, _layer_index: usize, _s: &Settings) -> Point {
    path_end(p)
}

/// Which island a point sits in: the innermost CCW contour containing it.
/// `None` = outside every outer (skirt/brim territory). Two points with
/// different islands can never be joined by an in-material route.
pub(crate) fn island_of(outline: &Polygons, p: Point) -> Option<usize> {
    let mut best: Option<(usize, f64)> = None;
    for (k, c) in outline.contours.iter().enumerate() {
        if c.is_ccw() && c.contains(p) {
            let a = c.area_mm2();
            if best.map_or(true, |(_, ba)| a < ba) {
                best = Some((k, a));
            }
        }
    }
    best.map(|(k, _)| k)
}

/// Whether the straight hop `p → q` is blocked by the outline: it properly
/// crosses a contour edge, passes exactly through a contour vertex, lands an
/// endpoint on an edge's interior, or overlaps an edge collinearly for any
/// positive length. Exact integer arithmetic; a shared endpoint alone (a hop
/// starting or ending AT a vertex) does not block. The degenerate cases are
/// load-bearing: on any flat face the edges of different islands (and hole
/// chords) are collinear, which is exactly how void-bridging hops used to
/// read as clear under a proper-crossings-only test.
pub(crate) fn travel_blocked(outline: &Polygons, p: Point, q: Point) -> bool {
    for c in &outline.contours {
        let n = c.points.len();
        for i in 0..n {
            let a = c.points[i];
            let b = c.points[(i + 1) % n];
            if a == b {
                continue;
            }
            let o1 = orient(p, q, a);
            let o2 = orient(p, q, b);
            let o3 = orient(a, b, p);
            let o4 = orient(a, b, q);
            if o1 != 0
                && o2 != 0
                && o3 != 0
                && o4 != 0
                && (o1 > 0) != (o2 > 0)
                && (o3 > 0) != (o4 > 0)
            {
                return true; // proper crossing
            }
            // Degenerate contacts: a contour vertex strictly inside the hop
            // (pass-through, or the start of a collinear run along the edge),
            // or a hop endpoint strictly inside an edge (tangent landing).
            if (o1 == 0 && strictly_between(p, q, a)) || (o2 == 0 && strictly_between(p, q, b)) {
                return true;
            }
            if (o3 == 0 && strictly_between(a, b, p)) || (o4 == 0 && strictly_between(a, b, q)) {
                return true;
            }
        }
    }
    false
}

/// For `c` collinear with the segment `a–b`: strictly interior to it?
fn strictly_between(a: Point, b: Point, c: Point) -> bool {
    c != a
        && c != b
        && c.x >= a.x.min(b.x)
        && c.x <= a.x.max(b.x)
        && c.y >= a.y.min(b.y)
        && c.y <= a.y.max(b.y)
}

/// Proper segment intersection (touching/collinear treated as no crossing).
/// Travel decisions use the stricter [`travel_blocked`]; only the
/// `audit_combing` hole diagnostic keys on this.
fn segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool {
    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0 && (o1 > 0) != (o2 > 0) && (o3 > 0) != (o4 > 0)
}

fn orient(p: Point, q: Point, r: Point) -> i128 {
    ((q.x - p.x) as i128) * ((r.y - p.y) as i128) - ((q.y - p.y) as i128) * ((r.x - p.x) as i128)
}

/// Even-odd containment in a polygon-with-holes.
fn in_region(outline: &Polygons, p: Point) -> bool {
    let mut inside = false;
    for c in &outline.contours {
        if c.contains(p) {
            inside = !inside;
        }
    }
    inside
}

/// A segment is a valid in-region travel if its midpoint is inside the solid
/// region and it touches no contour except at its own endpoints.
fn visible(outline: &Polygons, p: Point, q: Point) -> bool {
    if p == q {
        return true;
    }
    let mid = Point::new((p.x + q.x) / 2, (p.y + q.y) / 2);
    in_region(outline, mid) && !travel_blocked(outline, p, q)
}

/// Above this many outline vertices, skip the O(n²) diagonal precompute and
/// route along the boundary only (still inside the part, never through a hole).
const COMB_VERT_CAP: usize = 600;
/// Hard ceiling for routing at all (beyond this, fall back to retract).
const ROUTE_CAP: usize = 3000;

/// Per-layer visibility graph over the outline vertices, used to route combing
/// travels that would otherwise cross a wall.
struct CombGraph {
    verts: Vec<Point>,
    adj: Vec<Vec<(usize, f64)>>,
}

impl CombGraph {
    fn build(outline: &Polygons) -> Self {
        let mut verts: Vec<Point> = Vec::new();
        let mut edges: Vec<(usize, usize)> = Vec::new();
        for c in &outline.contours {
            let m = c.points.len();
            if m < 2 {
                continue;
            }
            let base = verts.len();
            verts.extend_from_slice(&c.points);
            for k in 0..m {
                edges.push((base + k, base + (k + 1) % m));
            }
        }
        let n = verts.len();
        let mut adj = vec![Vec::new(); n];
        // Boundary edges are always traversable (route can follow the wall).
        for &(i, j) in &edges {
            let w = dist_mm(verts[i], verts[j]);
            adj[i].push((j, w));
            adj[j].push((i, w));
        }
        // Visibility diagonals (skip the O(n²) pass on very complex layers).
        if n <= COMB_VERT_CAP {
            for i in 0..n {
                for j in (i + 1)..n {
                    if visible(outline, verts[i], verts[j]) {
                        let w = dist_mm(verts[i], verts[j]);
                        adj[i].push((j, w));
                        adj[j].push((i, w));
                    }
                }
            }
        }
        Self { verts, adj }
    }

    /// Shortest in-region route from `a` to `b`, as the points after `a`
    /// (intermediates + `b`), or None if unreachable.
    fn route(&self, outline: &Polygons, a: Point, b: Point) -> Option<Vec<Point>> {
        let n = self.verts.len();
        if n == 0 || n > ROUTE_CAP {
            return None;
        }
        let (ai, bi) = (n, n + 1);
        let total = n + 2;
        let mut adj = self.adj.clone();
        adj.push(Vec::new());
        adj.push(Vec::new());
        for (idx, p) in [(ai, a), (bi, b)] {
            for k in 0..n {
                if visible(outline, p, self.verts[k]) {
                    let w = dist_mm(p, self.verts[k]);
                    adj[idx].push((k, w));
                    adj[k].push((idx, w));
                }
            }
        }
        let mut dist = vec![f64::INFINITY; total];
        let mut prev = vec![usize::MAX; total];
        let mut done = vec![false; total];
        dist[ai] = 0.0;
        loop {
            let mut u = usize::MAX;
            let mut best = f64::INFINITY;
            for (k, &dk) in dist.iter().enumerate() {
                if !done[k] && dk < best {
                    best = dk;
                    u = k;
                }
            }
            if u == usize::MAX || u == bi {
                break;
            }
            done[u] = true;
            for &(v, w) in &adj[u] {
                if dist[u] + w < dist[v] {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                }
            }
        }
        if !dist[bi].is_finite() {
            return None;
        }
        let point = |idx: usize| {
            if idx == ai {
                a
            } else if idx == bi {
                b
            } else {
                self.verts[idx]
            }
        };
        let mut route = Vec::new();
        let mut cur = bi;
        while cur != usize::MAX {
            route.push(point(cur));
            cur = prev[cur];
        }
        route.reverse();
        route.remove(0); // drop `a` (already there)
        Some(route)
    }
}

/// Diagnostic over the planned travels: `(crossing_travels, combed,
/// fell_back_to_straight, straights_cutting_a_hole)`. A combed travel has a
/// multi-point route; a fallback is a single retracted/z-hopped hop.
pub fn audit_combing(layers: &[LayerPlan]) -> (usize, usize, usize, usize) {
    let (mut crossing, mut combed, mut fallback, mut fallback_hole) = (0, 0, 0, 0);
    for layer in layers {
        let mut last_pos: Option<Point> = None;
        for (i, path) in layer.paths.iter().enumerate() {
            if path.points.len() < 2 {
                continue;
            }
            let start = path.points[0];
            if let (Some(prev), Some(tr)) = (last_pos, layer.travels.get(i)) {
                if tr.points.len() > 1 {
                    crossing += 1;
                    combed += 1;
                } else if tr.retract {
                    crossing += 1;
                    fallback += 1;
                    if crosses_hole(&layer.outline, prev, start) {
                        fallback_hole += 1;
                    }
                }
            }
            last_pos = Some(path_end(path));
        }
    }
    (crossing, combed, fallback, fallback_hole)
}

fn crosses_hole(outline: &Polygons, a: Point, b: Point) -> bool {
    for c in &outline.contours {
        if c.is_ccw() {
            continue; // only holes (CW contours)
        }
        let n = c.points.len();
        for i in 0..n {
            if segments_intersect(a, b, c.points[i], c.points[(i + 1) % n]) {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::{
        apply_seam_gap, concave_overlap_seg_scale, dist_mm, fit_arc, island_of, path_end,
        plan_travels, travel_blocked, utc_stamp_at, CombGraph, ARC_MIN_PTS,
        COMB_RETRACT_MM, FEED_SLEW_PER_MM, SEAM_TRIM_LW,
    };
    use crate::{
        estimate_filament, estimate_filament_per_tool, estimate_seconds, generate, generate_parts,
        per_layer_stats, to_gcode, LayerPlan, PathKind, ToolPath, Travel,
    };
    use config::Settings;
    use geo2d::{Contour, Point, Polygons};

    #[test]
    fn comb_route_never_bridges_islands() {
        // Cross-section of a standing plate at hole height: two rectangles
        // whose top and bottom edges lie on shared collinear lines. Diagonals
        // along those lines used to pass the visibility test (collinear overlap
        // and vertex pass-throughs read as "no crossing"), so the router
        // returned "combed" routes straight across the void and island hops
        // went out unretracted, dragging ooze along the printed face.
        let rect = |x0: f64, x1: f64| {
            Contour::new(vec![
                Point::from_mm(x0, 0.0),
                Point::from_mm(x1, 0.0),
                Point::from_mm(x1, 3.0),
                Point::from_mm(x0, 3.0),
            ])
        };
        let mut outline = Polygons::new();
        outline.push(rect(0.0, 14.0));
        outline.push(rect(26.0, 40.0));
        let a = Point::from_mm(7.0, 1.5);
        let b = Point::from_mm(33.0, 1.5);
        assert_ne!(island_of(&outline, a), island_of(&outline, b), "distinct islands");
        let graph = CombGraph::build(&outline);
        assert!(
            graph.route(&outline, a, b).is_none(),
            "no comb route may bridge disjoint islands"
        );
        // The face line itself is blocked, not readable as clear...
        assert!(travel_blocked(&outline, Point::from_mm(7.0, 0.0), Point::from_mm(33.0, 0.0)));
        // ...while a clear in-island hop stays unblocked (no spurious retracts).
        assert!(!travel_blocked(&outline, a, Point::from_mm(2.0, 1.5)));
    }

    #[test]
    fn concave_island_still_combs_around_the_void() {
        // One L-shaped island: the tip-to-tip hop cuts the concave void, so it
        // must comb around the inside corner — the strict degenerate-contact
        // rules must not break legitimate routing.
        let pts = [(0.0, 0.0), (40.0, 0.0), (40.0, 8.0), (8.0, 8.0), (8.0, 40.0), (0.0, 40.0)];
        let mut outline = Polygons::new();
        outline.push(Contour::new(pts.iter().map(|&(x, y)| Point::from_mm(x, y)).collect()));
        let a = Point::from_mm(4.0, 36.0);
        let b = Point::from_mm(36.0, 4.0);
        assert_eq!(island_of(&outline, a), island_of(&outline, b), "one island");
        assert!(travel_blocked(&outline, a, b), "the direct hop cuts the void");
        let graph = CombGraph::build(&outline);
        let route = graph.route(&outline, a, b).expect("same island → a comb route exists");
        assert_eq!(route.last(), Some(&b), "route ends at the target");
        let mut len = 0.0;
        let mut at = a;
        for &p in &route {
            len += dist_mm(at, p);
            at = p;
        }
        assert!(len > dist_mm(a, b), "the route detours around the corner, not through the void");
    }

    fn solid_outline(w: f64, h: f64) -> Polygons {
        let mut o = Polygons::new();
        o.push(Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(w, 0.0),
            Point::from_mm(w, h),
            Point::from_mm(0.0, h),
        ]));
        o
    }

    fn bead(kind: PathKind, closed: bool, pts: Vec<Point>) -> ToolPath {
        ToolPath {
            kind,
            closed,
            width_mm: 0.45,
            points: pts,
            flow: 1.0,
            group: None,
            height_scale: 1.0,
            widths: None,
            overhang: 0.0,
            segs: None,
            tool: 0,
            joined: false,
        }
    }

    fn one_layer(paths: Vec<ToolPath>, outline: Polygons) -> Vec<LayerPlan> {
        vec![LayerPlan {
            index: 0,
            print_z_mm: 0.2,
            height_mm: 0.2,
            paths,
            travels: Vec::new(),
            outline,
            speed_scale: 1.0,
            fan_boost: 0.0,
            planned_temp_c: None,
            temp_command_c: None,
        }]
    }

    #[test]
    fn layer_change_retracts_before_lifting_off_the_seam() {
        // The seam blob column. A layer ends on the outer wall's seam with the
        // nozzle charged; if Z rises before the retract, melt pressure bleeds
        // straight down onto that seam, and since the seam sits at the same XY
        // every layer the ooze stacks into a visible column. The trim can't
        // help — it has finished feathering by then. So on every layer change
        // that retracts at all, the retract must come FIRST.
        let mut s = Settings::default();
        s.wall_count = 3;
        s.top_layers = 2;
        s.bottom_layers = 2;
        s.skirt_loops = 0;
        s.z_hop_mm = 0.4;
        let g = to_gcode(&generate(&mesh::Mesh::cube(10.0), &s), &s);
        let mut checked = 0;
        for n in 2..12 {
            let chunk = layer_chunk(&g, n);
            let retract = chunk.lines().position(|l| l.starts_with("G1 E-"));
            let lift = chunk
                .lines()
                .position(|l| l.starts_with("G1 Z") && !l.contains(" X") && !l.contains(" E"));
            if let (Some(r), Some(z)) = (retract, lift) {
                assert!(
                    r < z,
                    "layer {n}: Z lifted at line {z} before the retract at line {r} — \
                     the nozzle bleeds pressure onto the seam it just closed"
                );
                checked += 1;
            }
        }
        assert!(checked >= 5, "expected several layer changes to exercise this, got {checked}");
    }

    #[test]
    fn travels_touching_a_surface_retract_while_buried_ones_glide() {
        // The D5 retraction policy (the HOP is unconditional — see below). Same geometry, same 10 mm hop, one big solid outline
        // so every travel is a CLEAR in-material straight — only the KINDS
        // differ. Buried→buried (sparse infill) glides unretracted: nobody
        // will ever see that bead. Anything touching a visible surface
        // retracts AND hops, because a pressurized tip dragged at bead height
        // across a finished face leaves a drool trail that never gets covered
        // (the step-surface scars: 11.9/21.5/25.5 mm unretracted glides).
        let mut s = Settings::default();
        s.z_hop_mm = 0.4; // the hop half of the policy is profile-gated
        assert!(s.retract_len_mm > 0.0 && !s.spiral_vase);
        let hop = |from: PathKind, to: PathKind| -> Travel {
            let mut l = one_layer(
                vec![
                    bead(from, false, vec![Point::from_mm(5.0, 5.0), Point::from_mm(6.0, 5.0)]),
                    bead(to, false, vec![Point::from_mm(16.0, 5.0), Point::from_mm(17.0, 5.0)]),
                ],
                solid_outline(100.0, 100.0),
            );
            plan_travels(&mut l, &s);
            l[0].travels[1].clone()
        };
        assert!(dist_mm(Point::from_mm(6.0, 5.0), Point::from_mm(16.0, 5.0)) < COMB_RETRACT_MM);
        let buried = hop(PathKind::Infill, PathKind::Infill);
        assert!(!buried.retract, "buried→buried glides: no retract under the drool cap");
        // ...but it still LIFTS. Retraction is graded by what the travel can
        // spoil; the hop is not, since 2026-08-18: every travel clears the
        // layer, so nothing is ever crossed at bead height.
        assert!(buried.hop, "every travel hops, buried or not");
        let buried_solid = hop(PathKind::Solid, PathKind::Infill);
        assert!(!buried_solid.retract, "buried solid is buried too");
        for (from, to, what) in [
            (PathKind::TopSkin, PathKind::TopSkin, "skin→skin"),
            (PathKind::Infill, PathKind::TopSkin, "landing on skin"),
            (PathKind::TopSkin, PathKind::Infill, "leaving skin"),
            (PathKind::ExternalPerimeter, PathKind::Infill, "leaving an outer wall"),
            (PathKind::Infill, PathKind::GapFill, "landing on gap fill"),
        ] {
            let t = hop(from, to);
            assert!(t.retract, "{what} must retract");
            assert!(t.hop, "{what} must hop clear of the bead crowns");
        }
        // Sub-bead repositioning still glides even on a surface: below
        // MIN_TRAVEL_MM nothing is planned at all.
        let mut tiny = one_layer(
            vec![
                bead(PathKind::TopSkin, false, vec![Point::from_mm(5.0, 5.0), Point::from_mm(6.0, 5.0)]),
                bead(PathKind::TopSkin, false, vec![Point::from_mm(6.4, 5.0), Point::from_mm(7.0, 5.0)]),
            ],
            solid_outline(100.0, 100.0),
        );
        plan_travels(&mut tiny, &s);
        assert!(!tiny[0].travels[1].retract, "a sub-bead reposition still glides");
        // ...and it is the ONE travel that also skips the lift: the next bead
        // starts 0.4 mm away on a clear line, so there is nothing to clear.
        assert!(!tiny[0].travels[1].hop, "a bead-adjacent reposition needs no lift");
    }

    #[test]
    fn long_clear_straight_glide_retracts_but_short_one_glides() {
        // Two beads inside one big solid rectangle: no islands, no wall between
        // them, so every hop is a CLEAR in-material straight (not combed). Past
        // COMB_RETRACT_MM the glide must retract (the melt-pressure drool guard,
        // same as a combed route); a short one still glides unretracted.
        let s = Settings::default();
        assert!(s.retract_len_mm > 0.0 && !s.spiral_vase);
        let near = Point::from_mm(6.0, 5.0);
        // Far hop ~118 mm → retract.
        let mut far = one_layer(
            vec![
                bead(PathKind::Solid, false, vec![Point::from_mm(5.0, 5.0), near]),
                bead(PathKind::Solid, false, vec![Point::from_mm(90.0, 90.0), Point::from_mm(91.0, 90.0)]),
            ],
            solid_outline(100.0, 100.0),
        );
        assert!(dist_mm(near, Point::from_mm(90.0, 90.0)) > COMB_RETRACT_MM);
        plan_travels(&mut far, &s);
        assert!(far[0].travels[1].retract, "a >30mm clear glide must retract");
        assert!(!far[0].travels[1].hop, "it stays over the part — no hop");
        // Short hop ~2 mm → glide.
        let mut short = one_layer(
            vec![
                bead(PathKind::Solid, false, vec![Point::from_mm(5.0, 5.0), near]),
                bead(PathKind::Solid, false, vec![Point::from_mm(8.0, 5.0), Point::from_mm(9.0, 5.0)]),
            ],
            solid_outline(100.0, 100.0),
        );
        plan_travels(&mut short, &s);
        assert!(!short[0].travels[1].retract, "a short clear glide must not retract");
    }

    #[test]
    fn seam_trim_opens_butt_loops_but_spares_joined_and_vase() {
        // The fixed butt-seam trim: an UNJOINED closed wall loop is opened and
        // its last point stops `SEAM_TRIM_LW × line_width` short of the seam
        // start, so the closing bead never butts onto (and zits) the start. A
        // JOINED loop is entered at pressure and must stay fully closed (its
        // closure is deterministic, and the concave trim needs the loop
        // closed); a vase print must keep its loop closed for the spiral.
        let square = || {
            bead(
                PathKind::ExternalPerimeter,
                true,
                vec![
                    Point::from_mm(0.0, 0.0),
                    Point::from_mm(10.0, 0.0),
                    Point::from_mm(10.0, 10.0),
                    Point::from_mm(0.0, 10.0),
                ],
            )
        };
        let s = Settings::default();
        let gap = SEAM_TRIM_LW * s.line_width_mm;
        let mut on = one_layer(vec![square()], solid_outline(20.0, 20.0));
        apply_seam_gap(&mut on, &s);
        let p = &on[0].paths[0];
        assert!(!p.closed, "trim → loop is opened so the close isn't re-emitted");
        let end = *p.points.last().unwrap();
        let start = p.points[0];
        // The end sits one trim before the start, on the closing leg (from (0,10)).
        assert!((dist_mm(end, start) - gap).abs() < 1.0e-6, "ends exactly one trim short of the seam");
        assert!(end.x_mm().abs() < 1.0e-9, "still on the x=0 closing leg");
        // Joined: the SAME trim — the closing bead arrives at full pressure,
        // so butting it onto the (also full) entry bead would pile exactly
        // the end-cap the trim cancels.
        let mut j = square();
        j.joined = true;
        let mut joined = one_layer(vec![j], solid_outline(20.0, 20.0));
        apply_seam_gap(&mut joined, &s);
        let jp = &joined[0].paths[0];
        assert!(!jp.closed, "a joined loop is trimmed like any butt loop");
        assert!(
            (dist_mm(*jp.points.last().unwrap(), jp.points[0]) - gap).abs() < 1.0e-6,
            "joined closure stops one trim short of the seam"
        );
        // Vase: untouched (an opened loop would kill the spiral emitter).
        let mut vase_s = Settings::default();
        vase_s.spiral_vase = true;
        let mut vase = one_layer(vec![square()], solid_outline(20.0, 20.0));
        apply_seam_gap(&mut vase, &vase_s);
        assert!(vase[0].paths[0].closed, "vase loops stay closed for the spiral");
    }

    #[test]
    fn restart_extra_deprimes_the_unretract() {
        // Two separated boxes → island-to-island hops that retract. A negative
        // restart-extra makes the de-retract restore LESS than was pulled,
        // absorbing the unretract's seam blob: retract 0.5, re-prime 0.4.
        let mut tris = Vec::new();
        cuboid(&mut tris, 0.0, 0.0, 0.0, 8.0, 8.0, 4.0);
        cuboid(&mut tris, 20.0, 0.0, 0.0, 28.0, 8.0, 4.0);
        let mesh = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.retract_len_mm = 0.5;
        s.retract_restart_extra_mm = -0.1;
        let g = to_gcode(&generate(&mesh, &s), &s);
        assert!(g.contains("G1 E-0.50000"), "retracts the full 0.5mm");
        assert!(g.contains("G1 E0.40000"), "de-retracts 0.4mm (0.5 − 0.1 de-prime)");
        assert!(!g.contains("G1 E0.50000"), "no symmetric 0.5mm re-prime remains");
    }

    #[test]
    fn concave_overlap_scale_trims_only_reflex_vertices() {
        // A CCW L-shape: (8,8) is the single reflex (concave) corner; every
        // other vertex is convex. The corner's trim splits half/half across
        // the segment entering it and the segment leaving it; convex segments
        // keep full flow.
        let pts: Vec<Point> = [(0., 0.), (40., 0.), (40., 8.), (8., 8.), (8., 40.), (0., 40.)]
            .iter()
            .map(|&(x, y)| Point::from_mm(x, y))
            .collect();
        let scale = concave_overlap_seg_scale(&pts, true, 0.45);
        assert_eq!(scale.len(), 6);
        // Entering segment (index 2: (40,8)->(8,8)) and leaving segment
        // (index 3: (8,8)->(8,40)) each carry half the trim.
        assert!(scale[2] < 1.0, "reflex vertex's entering segment is trimmed: {}", scale[2]);
        assert!(scale[3] < 1.0, "reflex vertex's leaving segment is trimmed: {}", scale[3]);
        for (k, s) in scale.iter().enumerate() {
            if k != 2 && k != 3 {
                assert_eq!(*s, 1.0, "convex segment {k} is left at full flow");
            }
        }
        // The 90° corner trims (w/4)·tan(45°) = 0.1125 mm of E-length total,
        // half over each 32 mm adjacent segment.
        let expected = 1.0 - 0.5 * (0.45 * 0.25 * 1.0) / 32.0;
        assert!((scale[2] - expected).abs() < 1.0e-9, "entering half: (w/8)·tan(45°)/len");
        assert!((scale[3] - expected).abs() < 1.0e-9, "leaving half: (w/8)·tan(45°)/len");
        // A butt-seam-trimmed loop (opened a hair short of its start) keeps
        // its trim: the virtual closure recovers the ring winding, and the
        // reflex corner is an interior vertex. The seam-trim pass must never
        // silently disable the concave compensation.
        let mut open_pts = pts.clone();
        open_pts.push(Point::from_mm(0.0, 0.045)); // 0.1×lw short of the (0,0) start
        let os = concave_overlap_seg_scale(&open_pts, false, 0.45);
        assert_eq!(os.len(), 6);
        assert!(os[2] < 1.0 && os[3] < 1.0, "trimmed-open ring keeps its reflex trim");
        assert!((os[2] - expected).abs() < 1.0e-9 && (os[3] - expected).abs() < 1.0e-9);
        assert_eq!(os[0], 1.0, "seam-end segments untouched");
        assert_eq!(os[5], 1.0, "seam-end segments untouched");
    }

    fn cuboid(tris: &mut Vec<[[f64; 3]; 3]>, x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64) {
        let v = [
            [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
            [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
        ];
        for [a, b, c, d] in [
            [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
            [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
        ] {
            tris.push([v[a], v[b], v[c]]);
            tris.push([v[a], v[c], v[d]]);
        }
    }

    #[test]
    fn island_hops_at_hole_heights_retract_and_hop() {
        // A standing 60×3×30 plate with two through-windows (x 14–26 and
        // 34–46, z 10–20), built from five clean overlapping boxes: at window
        // heights every layer is three islands. This is the geometry that came
        // off the printer with scarred bands beside every hole — island hops
        // were emitted as naked "combed" travels dragged along the outer face.
        // Every island-to-island travel must be a retracted, z-hopped straight
        // hop, and the retractions must reach the g-code.
        let mut tris = Vec::new();
        cuboid(&mut tris, 0.0, 0.0, 0.0, 14.0, 3.0, 30.0);
        cuboid(&mut tris, 26.0, 0.0, 0.0, 34.0, 3.0, 30.0);
        cuboid(&mut tris, 46.0, 0.0, 0.0, 60.0, 3.0, 30.0);
        cuboid(&mut tris, 0.0, 0.0, 0.0, 60.0, 3.0, 10.0);
        cuboid(&mut tris, 0.0, 0.0, 20.0, 60.0, 3.0, 30.0);
        let mesh = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.z_hop_mm = 0.4;
        assert!(s.retract_len_mm > 0.0, "test needs retraction enabled");
        let layers = generate(&mesh, &s);
        let mut hops = 0usize;
        for layer in &layers {
            let mut last: Option<Point> = None;
            for (i, path) in layer.paths.iter().enumerate() {
                if path.points.len() < 2 {
                    continue;
                }
                let start = path.points[0];
                if let (Some(prev), Some(tr)) = (last, layer.travels.get(i)) {
                    if island_of(&layer.outline, prev) != island_of(&layer.outline, start) {
                        hops += 1;
                        assert!(
                            tr.retract && tr.hop && tr.points.len() == 1,
                            "island hop must be a retracted+hopped straight move at z={:.2} \
                             (retract={} hop={} pts={})",
                            layer.print_z_mm,
                            tr.retract,
                            tr.hop,
                            tr.points.len()
                        );
                    }
                }
                last = Some(path_end(path));
            }
        }
        assert!(hops >= 40, "the window band must produce many island hops, saw {hops}");
        let gcode = to_gcode(&layers, &s);
        let retracts = gcode.lines().filter(|l| l.trim_start().starts_with("G1 E-")).count();
        assert!(retracts >= hops, "retractions must reach the g-code: {retracts} for {hops} hops");
    }

    #[test]
    fn arc_fit_curves_yes_corners_no() {
        // A densely-sampled circle fits one arc...
        let circle: Vec<(f64, f64)> = (0..64)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 64.0;
                (10.0 * a.cos(), 10.0 * a.sin())
            })
            .collect();
        let fit = fit_arc(&circle, 0, 0.05).expect("dense circle is an arc");
        assert!(fit.0 + 1 >= ARC_MIN_PTS, "arc spans several points");
        assert!(fit.1.hypot(fit.2) < 0.2, "center near origin: ({}, {})", fit.1, fit.2);
        // ...but a square's (concyclic) corners must NOT be rounded into an arc.
        let square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)];
        assert!(fit_arc(&square, 0, 0.05).is_none(), "square is straight, not an arc");
    }

    #[test]
    fn arc_fit_rejects_tangential_jitter() {
        // Points that double back along the same circle stay within tolerance
        // and keep a consistent polyline turn, but the swept angle is not
        // monotone: emitting an arc would squeeze the whole back-and-forth
        // bead into the short start→end angular gap (an over-extruded blob —
        // found as 5 moves commanding up to 84 mm³/s on a real Benchy export).
        let r = 6.0;
        let angs = [0.0f64, 0.06, 0.12, 0.06, 0.01, 0.08, 0.14];
        let wiggle: Vec<(f64, f64)> = angs.iter().map(|&a| (r * a.cos(), r * a.sin())).collect();
        match fit_arc(&wiggle, 0, 0.05) {
            None => {}
            Some((j, cx, cy, _)) => {
                // If a prefix fits, its emitted length must equal the polyline
                // length over the same points (monotone sweep — no folding).
                let run = &wiggle[0..=j];
                let poly: f64 = run.windows(2).map(|w| (w[0].0 - w[1].0).hypot(w[0].1 - w[1].1)).sum();
                let arc = super::arc_span_len(run, cx, cy);
                assert!((arc - poly).abs() < 0.05, "arc len {arc:.3} vs polyline {poly:.3}");
            }
        }
        // A full circle plus overlap must not wrap past 360°.
        let lap: Vec<(f64, f64)> = (0..80)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 64.0; // 1.25 laps
                (10.0 * a.cos(), 10.0 * a.sin())
            })
            .collect();
        let (j, cx, cy, _) = fit_arc(&lap, 0, 0.05).expect("the first lap is a fine arc");
        let run = &lap[0..=j];
        let poly: f64 = run.windows(2).map(|w| (w[0].0 - w[1].0).hypot(w[0].1 - w[1].1)).sum();
        let arc = super::arc_span_len(run, cx, cy);
        assert!((arc - poly).abs() < 0.1, "wrapped arc: len {arc:.3} vs polyline {poly:.3}");
    }

    #[test]
    fn gcode_is_klipper_relative_with_retraction() {
        let m = mesh::Mesh::cube(20.0);
        let s = Settings::default();
        let g = to_gcode(&generate(&m, &s), &s);

        assert!(g.contains("M83"), "relative extrusion mode");
        // Acceleration follows the feature: gentle first layer, gentle outer
        // wall (auto = accel/2), the main limit for everything else.
        assert!(g.contains("M204 S1000"), "first-layer acceleration");
        assert!(g.contains("M204 S1500"), "outer-wall acceleration");
        assert!(g.contains("M204 S3000"), "main acceleration");
        // Feature annotations for g-code viewers and analysis tools.
        for t in [";TYPE:Outer wall", ";TYPE:Inner wall", ";TYPE:Solid infill", ";TYPE:Skirt"] {
            assert!(g.contains(t), "missing {t}");
        }
        assert!(g.contains("SQUARE_CORNER_VELOCITY=10.0"), "emits Klipper square corner velocity");
        assert!(g.contains("G28"), "homes (generic start template)");
        assert!(g.contains("M104 S210"), "nozzle temp substituted");
        assert!(g.contains("M140 S60"), "bed temp substituted");
        assert!(g.contains("M84"), "disables steppers (generic end template)");
        assert!(
            g.lines().any(|l| l.starts_with("G1 X") && l.contains(" E")),
            "has extruding moves"
        );
        assert!(g.lines().any(|l| l.starts_with("G1 E-")), "has retraction moves");
    }

    #[test]
    fn retract_wipes_back_over_the_printed_bead() {
        let m = mesh::Mesh::cube(20.0);
        let s = Settings::default(); // wipe_mm = 2.0
        let plans = generate(&m, &s);
        let g = to_gcode(&plans, &s);
        // Find a retract that is followed by moves before the unretract — the
        // wipe — and check the first wipe target lies on the printed cube
        // perimeter (x or y at the wall coordinate), not out in the open.
        let lines: Vec<&str> = g.lines().collect();
        let mut found = false;
        for w in lines.windows(3) {
            if w[0].starts_with("G1 E-") && w[1].starts_with("G0 X") && !w[1].contains('Z') {
                found = true;
                break;
            }
        }
        assert!(found, "no wipe move directly after a retraction");
        // And with wipe disabled those moves disappear: the file shrinks.
        let mut s2 = s.clone();
        s2.wipe_mm = 0.0;
        let g2 = to_gcode(&generate(&m, &s2), &s2);
        assert!(
            g.lines().count() > g2.lines().count(),
            "wipe must add moves ({} vs {})",
            g.lines().count(),
            g2.lines().count()
        );
    }

    #[test]
    fn placeholders_are_substituted() {
        let mut s = Settings::default();
        s.start_gcode = "PRINT_START EXTRUDER={nozzle_temp} BED={bed_temp}".into();
        s.nozzle_temp_c = 215;
        s.bed_temp_c = 65;
        let g = to_gcode(&generate(&mesh::Mesh::cube(10.0), &s), &s);
        assert!(g.contains("PRINT_START EXTRUDER=215 BED=65"), "macro substituted");
        assert!(!g.contains("{nozzle_temp}"), "no leftover placeholders");
    }

    #[test]
    fn chamber_presoak_emits_temperature_wait() {
        let m = mesh::Mesh::cube(10.0);
        // Filament wants a soak + the printer declares its sensor: a plain
        // TEMPERATURE_WAIT on the sensor lands before the first layer. No macro
        // — Klipper aborts natively if the sensor is missing (the friendly
        // version of that error is the pre-send Moonraker ping).
        let mut s = Settings::default();
        s.chamber_sensor = "chamber_temp".into();
        s.chamber_temp_c = 50;
        let g = to_gcode(&generate(&m, &s), &s);
        let wait = "TEMPERATURE_WAIT SENSOR=\"temperature_sensor chamber_temp\" MINIMUM=50";
        let at = g.find(wait).expect("pre-soak emitted");
        assert!(at < g.find("; LAYER 0 ").unwrap(), "soak precedes the first layer");
        // No soak wanted (the PLA-class 0): nothing emitted, no sensor referenced.
        s.chamber_temp_c = 0;
        assert!(!to_gcode(&generate(&m, &s), &s).contains("TEMPERATURE_WAIT"));
        // Soak wanted but the profile names no sensor: the wait is still emitted
        // (empty name) so the print aborts at the soak rather than running cold —
        // the pre-send check is what makes that legible.
        s.chamber_temp_c = 50;
        s.chamber_sensor = String::new();
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(g.contains("TEMPERATURE_WAIT SENSOR=\"temperature_sensor \" MINIMUM=50"));
    }

    #[test]
    fn chamber_soak_placeholder_precedes_nozzle_heat() {
        // A template that positions {chamber_soak} gets the soak inline, before
        // its own nozzle heat — the anti-ooze ordering — and not also appended.
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.chamber_sensor = "chamber_temp".into();
        s.chamber_temp_c = 50;
        s.start_gcode =
            "M190 S{bed_temp}\nG28\n{chamber_soak}\nM104 S{first_layer_nozzle_temp}\nM109 S{first_layer_nozzle_temp}"
                .into();
        let g = to_gcode(&generate(&m, &s), &s);
        let wait = "TEMPERATURE_WAIT SENSOR=\"temperature_sensor chamber_temp\" MINIMUM=50";
        let soak = g.find(wait).expect("soak emitted in place");
        let nozzle = g
            .find(&format!("M109 S{}", s.first_layer_nozzle_temp_c))
            .expect("nozzle heat emitted");
        assert!(soak < nozzle, "soak waits before the nozzle reaches temp");
        assert_eq!(g.matches(wait).count(), 1, "placeholder consumes it; no double append");
    }

    #[test]
    fn first_layer_temp_drops_after_first_layer() {
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.first_layer_nozzle_temp_c = 230;
        s.nozzle_temp_c = 210;
        let g = to_gcode(&generate(&m, &s), &s);
        let l0 = g.find("; LAYER 0 ").unwrap();
        let l1 = g.find("; LAYER 1 ").unwrap();
        assert!(g[..l0].contains("M104 S230"), "start g-code heats to the first-layer temp");
        assert!(!g[l0..l1].contains("M104"), "no temp change during the first layer");
        assert!(g[l1..].contains("M104 S210"), "drops to the printing temp at layer 2");

        // Equal temps (the auto default): no temp change at all between the
        // first layer and the end g-code's cooldown.
        s.first_layer_nozzle_temp_c = 210;
        let g = to_gcode(&generate(&m, &s), &s);
        let l0 = g.find("; LAYER 0 ").unwrap();
        let cooldown = g.rfind("M104 S0").unwrap();
        assert!(!g[l0..cooldown].contains("M104"), "no mid-print temp change");
    }

    #[test]
    fn aux_and_exhaust_fans_gated_and_emitted() {
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.aux_fan_speed = 0.75;
        s.exhaust_fan_speed = 0.8;
        // Off by default (no declared hardware): the P-forms must never appear —
        // vanilla Klipper/Marlin would read them as the primary fan.
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(!g.contains("M106 P"), "no P-fans until the hardware is declared");

        s.has_aux_fan = true;
        s.has_exhaust_fan = true;
        let g = to_gcode(&generate(&m, &s), &s);
        let l0 = g.find("; LAYER 0 ").unwrap();
        let l1 = g.find("; LAYER 1 ").unwrap();
        assert!(g[..l0].contains("M106 P3 S204"), "exhaust on from the start");
        assert!(!g[l0..l1].contains("M106 P2 S191"), "aux respects fan-off layers");
        assert!(g[l1..].contains("M106 P2 S191"), "aux at duty past the first layer");
        assert!(g.rfind("M106 P2 S0").unwrap() > l1, "aux shut off at the end");
        assert!(g.rfind("M106 P3 S0").unwrap() > l1, "exhaust shut off at the end");
    }

    #[test]
    fn per_layer_stats_match_totals() {
        let m = mesh::Mesh::cube(20.0);
        let s = Settings::default();
        let layers = generate(&m, &s);
        let stats = per_layer_stats(&layers, &s);
        assert_eq!(stats.len(), layers.len());
        // Per-layer seconds sum to the total estimate (identical terms).
        let total: f64 = stats.iter().map(|st| st.secs).sum();
        let est = estimate_seconds(&layers, &s);
        assert!((total - est).abs() < est * 1e-9, "{total} vs {est}");
    }

    #[test]
    fn shell_solids_split_into_skins() {
        // A 15 mm cube (big enough that island pacing stays out of the feeds —
        // this test is about skin CLASSIFICATION and skin pace, not thermal
        // pacing): layer 0 is all bottom skin (the bed face), the roof layer
        // all top skin, and the shell layers between stay buried Solid.
        let m = mesh::Mesh::cube(15.0);
        let mut s = Settings::default();
        s.min_layer_time_s = 0.0; // keep nominal feeds visible in the g-code
        let layers = generate(&m, &s);
        let n = layers.len();
        let count = |l: &LayerPlan, k: PathKind| l.paths.iter().filter(|p| p.kind == k).count();
        assert!(count(&layers[0], PathKind::BottomSkin) > 0, "bed face is bottom skin");
        assert_eq!(count(&layers[0], PathKind::Solid), 0, "layer 0 has no buried solid");
        assert!(count(&layers[n - 1], PathKind::TopSkin) > 0, "roof is top skin");
        assert_eq!(count(&layers[n - 1], PathKind::Solid), 0, "roof has no buried solid");
        assert!(count(&layers[n - 2], PathKind::Solid) > 0, "covered shell stays Solid");
        assert_eq!(count(&layers[n - 2], PathKind::TopSkin), 0, "covered shell is not skin");
        let mid = n / 2;
        let shell = count(&layers[mid], PathKind::Solid)
            + count(&layers[mid], PathKind::TopSkin)
            + count(&layers[mid], PathKind::BottomSkin);
        assert_eq!(shell, 0, "mid layers have no shell solid");

        // Skins are labelled for viewers and paced like the outer wall.
        let g = to_gcode(&layers, &s);
        assert!(g.contains(";TYPE:Bottom surface"), "bottom skin TYPE emitted");
        let lines: Vec<&str> = g.lines().collect();
        let ti = lines.iter().position(|l| *l == ";TYPE:Top surface").expect("top skin TYPE");
        let f_of = |from: usize| -> f64 {
            let mut f = 0.0;
            for l in &lines[..from] {
                if let Some(p) = l.find('F') {
                    if l.starts_with('G') {
                        f = l[p + 1..].split_whitespace().next().unwrap().parse().unwrap_or(f);
                    }
                }
            }
            for l in &lines[from..] {
                if l.starts_with(';') {
                    break;
                }
                if let Some(p) = l.find('F') {
                    f = l[p + 1..].split_whitespace().next().unwrap().parse().unwrap_or(f);
                }
                // The first real MOTION sets the pace. A stationary retract or
                // unretract (E-only, at retract speed) is not the skin's feed.
                if l.starts_with("G1 ") && (l.contains(" X") || l.contains(" Y")) {
                    return f;
                }
            }
            f
        };
        let skin_f = f_of(ti + 1);
        assert!(
            (skin_f - s.external_perimeter_speed_mm_s * 60.0).abs() < 1.0,
            "top skin feeds at the outer-wall pace: F{skin_f}"
        );
    }


    #[test]
    fn time_estimate_is_sane() {
        let m = mesh::Mesh::cube(20.0);
        let s = Settings::default();
        let secs = estimate_seconds(&generate(&m, &s), &s);
        // A 20mm cube at these settings: minutes, not seconds or days.
        assert!(secs > 60.0 && secs < 86_400.0, "got {secs}s");
    }

    #[test]
    fn volumetric_clamp_slows_and_reports() {
        let m = mesh::Mesh::cube(20.0);
        let mut s = Settings::default();
        s.print_speed_mm_s = 300.0; // 300 × 0.45 × 0.2 = 27 mm³/s, over any PLA
        s.solid_speed_mm_s = 240.0;
        // Stadium bead 0.45 × 0.2 = 0.0814 mm² → cap = 9 / 0.0814 ≈ 110.5 mm/s.
        s.max_volumetric_speed_mm3_s = 9.0;
        let layers = generate(&m, &s);

        let cap = 9.0 / config::bead_area_mm2(0.45, 0.2);
        let clamps = crate::emit::audit_flow_clamps(&layers, &s);
        assert!(!clamps.is_empty(), "clamp should engage");
        for (_, nominal, clamped) in &clamps {
            assert!(clamped < nominal, "{clamped} should be below {nominal}");
            assert!((*clamped - cap).abs() < 1.0, "cap ≈ {cap:.1} mm/s, got {clamped}");
        }

        // The g-code announces it and actually uses the capped feed,
        // never the asked-for F18000.
        let g = to_gcode(&layers, &s);
        assert!(g.contains("; flow limit = 9.0 mm3/s"));
        assert!(
            g.contains(&format!("; flow-limited: infill 300 -> {cap:.0} mm/s")),
            "header reports the clamp"
        );
        assert!(!g.contains("F18000"), "unclamped feed must not appear");

        // And the time estimate reflects the slower reality. (The margin is
        // modest on a small cube: at 3000 mm/s² the head rarely reaches the
        // nominal 300 mm/s before a corner anyway.)
        let slow = estimate_seconds(&layers, &s);
        s.max_volumetric_speed_mm3_s = 0.0; // unlimited
        let layers_fast = generate(&m, &s);
        let fast = estimate_seconds(&layers_fast, &s);
        assert!(slow > fast * 1.02, "clamped print must take longer ({slow:.0}s vs {fast:.0}s)");
        assert!(crate::emit::audit_flow_clamps(&layers_fast, &s).is_empty(), "0 = unlimited");
    }

    #[test]
    fn default_speeds_are_not_clamped() {
        // The stock profile combos must not trigger the limiter (15 mm³/s vs
        // 50 mm/s × 0.45 × 0.2 = 4.5 mm³/s) — no behavior change by default.
        let m = mesh::Mesh::cube(10.0);
        let s = Settings::default();
        assert!(crate::emit::audit_flow_clamps(&generate(&m, &s), &s).is_empty());
    }

    #[test]
    fn gcode_has_progress_and_metadata() {
        let m = mesh::Mesh::cube(10.0);
        let s = Settings::default();
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(g.contains("; estimated printing time = "), "metadata header");
        assert!(g.contains("; filament used [g] = "), "filament metadata");
        assert!(g.lines().any(|l| l.starts_with("M73 P0 ")), "progress at start");
        assert!(g.contains("M73 P100 R0"), "progress at end");
    }

    #[test]
    fn host_metadata_uses_the_field_names_hosts_parse() {
        // These spellings are an INTERFACE, not prose: Moonraker (and the
        // viewers modelled on its scanner) match them literally, so a tidy-up
        // rename silently empties the file card in Mainsail/Fluidd. Pinned
        // against the parser's own regexes — note "total layers count" and
        // "total filament used [g]", which differ from the human-readable
        // lines above them in the header.
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.chamber_temp_c = 40;
        let plans = generate(&m, &s);
        let n = plans.len();
        let g = to_gcode(&plans, &s);
        for field in [
            "; total layers count = ",
            "; total filament used [g] = ",
            "; filament used [mm] = ",
            "; estimated printing time = ",
            "; layer_height = ",
            "; first_layer_height = ",
            "; nozzle_diameter = ",
            "; filament_type = ",
            "; filament_settings_id = ",
            "; first_layer_temperature = ",
            "; first_layer_bed_temperature = ",
            "; chamber_temperature = ",
        ] {
            assert!(g.contains(field), "host metadata field missing: {field:?}");
        }
        // Values, not just labels: the layer count is the real one, and the
        // temperatures are the first-layer targets (not the bulk ones).
        assert!(g.contains(&format!("; total layers count = {n}")));
        assert!(g.contains(&format!("; first_layer_temperature = {}", s.first_layer_nozzle_temp_c)));
        assert!(g.contains(&format!("; first_layer_bed_temperature = {}", s.bed_temp_c)));
        assert!(g.contains("; chamber_temperature = 40"));
        // The metadata sits in the header, before any motion.
        let meta = g.find("; total layers count = ").unwrap();
        assert!(meta < g.find("; LAYER 0 ").unwrap(), "metadata precedes the print");
    }

    #[test]
    fn header_identity_matches_the_host_scanner_shape() {
        // Print hosts sniff the identity line with (Moonraker's generic
        // "unknown PrusaSlicer fork" path — the reason this exact shape is
        // worth a timestamp):
        //     r"; generated by (.+?) on \d{4}-\d{2}-\d{2} at \d{2}:\d{2}:\d{2}"
        // and then split the captured text on its last space into name +
        // version. Anything else drops the whole card back to a Z-move
        // sniffer, so pin the shape here.
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.printer_name = "sovol-zero".into();
        let g = to_gcode(&generate(&m, &s), &s);
        let line = g.lines().find(|l| l.starts_with("; generated by ")).expect("identity line");
        let captured = line
            .strip_prefix("; generated by ")
            .and_then(|r| r.split_once(" on "))
            .expect("' on ' clause");
        let (name, version) = captured.0.rsplit_once(' ').expect("name then version");
        assert_eq!(name, "Fable Slicer");
        assert!(version.starts_with(|c: char| c.is_ascii_digit()), "version: {version:?}");
        // "YYYY-MM-DD at HH:MM:SS", digits where the scanner wants digits.
        let (date, time) = captured.1.split_once(" at ").expect("date at time");
        let dash: Vec<&str> = date.split('-').collect();
        assert_eq!(dash.iter().map(|p| p.len()).collect::<Vec<_>>(), vec![4, 2, 2], "{date:?}");
        let colon: Vec<&str> = time.split(':').collect();
        assert_eq!(colon.iter().map(|p| p.len()).collect::<Vec<_>>(), vec![2, 2, 2], "{time:?}");
        assert!(date.chars().all(|c| c.is_ascii_digit() || c == '-'));
        assert!(time.chars().all(|c| c.is_ascii_digit() || c == ':'));
        // Everything BUT that one line is reproducible: same job, same bytes.
        let g2 = to_gcode(&generate(&m, &s), &s);
        let strip = |t: &str| {
            t.lines().filter(|l| !l.starts_with("; generated by ")).collect::<Vec<_>>().join("\n")
        };
        assert_eq!(strip(&g), strip(&g2), "only the timestamp may vary between runs");
        // The machine rides its own standard field, not the identity line.
        assert!(g.contains("; printer_settings_id = \"sovol-zero\""));
    }

    #[test]
    fn utc_stamp_converts_known_instants() {
        // Pin the calendar math against instants with known answers — a leap
        // day, a century-leap year boundary, and the epoch itself.
        for (secs, want) in [
            (0_i64, "1970-01-01 at 00:00:00"),
            (951_782_400, "2000-02-29 at 00:00:00"),   // leap day, /400 rule
            (1_709_164_800, "2024-02-29 at 00:00:00"), // leap day, /4 rule
            (1_755_561_599, "2025-08-18 at 23:59:59"),
        ] {
            let d = std::time::UNIX_EPOCH + std::time::Duration::from_secs(secs as u64);
            let got = utc_stamp_at(
                d.duration_since(std::time::UNIX_EPOCH).unwrap().as_secs() as i64
            );
            assert_eq!(got, want, "at {secs}");
        }
    }

    #[test]
    fn host_metadata_lists_every_tool() {
        // Per-extruder fields follow the same ';'-joined convention the
        // hosts already expect from multi-material files.
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        assert!(g.contains("; filament_settings_id = \"a\";\"b\""), "both spools named");
        assert!(
            g.contains(&format!(
                "; first_layer_temperature = {};{}",
                s.tools[0].first_layer_nozzle_temp_c, s.tools[1].first_layer_nozzle_temp_c
            )),
            "per-tool first-layer temps"
        );
    }

    #[test]
    fn fan_and_pressure_advance_emitted() {
        let m = mesh::Mesh::cube(10.0);
        let mut s = Settings::default();
        s.fan_speed = 0.8; // 204/255
        // Ceiling == base: the short-layer cooling ramp has no headroom, so
        // the emitted duty is exactly the base (this test pins base + off-layers).
        s.bridge_fan_speed = 0.8;
        s.fan_max = 0.8;
        s.fan_off_layers = 2;
        s.pressure_advance = 0.045;
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(g.contains("SET_PRESSURE_ADVANCE ADVANCE=0.0450"), "PA emitted");
        assert!(g.contains("M106 S204"), "fan at 80% duty");
        // Fan must stay off until layer 2: no M106 between LAYER 0 and LAYER 2.
        let l0 = g.find("; LAYER 0 ").unwrap();
        let l2 = g.find("; LAYER 2 ").unwrap();
        assert!(!g[l0..l2].contains("M106"), "fan off for the first two layers");
        assert!(g[l2..].contains("M106 S204"), "fan on from layer 2");
    }

    // --- toolchanger ----------------------------------------------------------

    fn push_box(tris: &mut Vec<[[f64; 3]; 3]>, lo: [f64; 3], hi: [f64; 3]) {
        let v = [
            [lo[0], lo[1], lo[2]],
            [hi[0], lo[1], lo[2]],
            [hi[0], hi[1], lo[2]],
            [lo[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]],
            [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], hi[2]],
            [lo[0], hi[1], hi[2]],
        ];
        for t in [
            [0, 2, 1], [0, 3, 2],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [3, 6, 2], [3, 7, 6],
            [0, 7, 3], [0, 4, 7],
            [1, 2, 6], [1, 6, 5],
        ] {
            tris.push([v[t[0]], v[t[1]], v[t[2]]]);
        }
    }

    /// Two 10 mm cubes 20 mm apart on tools 0 and 1; `raise_b_mm` lifts the
    /// tool-1 cube off the bed (so tool 1 doesn't print on layer 0).
    fn two_tool_plans(s: &Settings, raise_b_mm: f64) -> Vec<LayerPlan> {
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, raise_b_mm], [40.0, 10.0, 10.0 + raise_b_mm]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        generate_parts(&[(&a, 0), (&b, 1)], s)
    }

    /// A two-slot machine: tool 0 mirrors the flat fields, tool 1 runs hotter
    /// with its own PA, filament diameter, and fan duty.
    fn two_tool_settings() -> Settings {
        let mut s = Settings::default();
        s.skirt_loops = 0;
        s.min_layer_time_s = 0.0; // keep speed_scale at 1 (independent of swap time)
        s.nozzle_temp_c = 210;
        s.first_layer_nozzle_temp_c = 220;
        s.pressure_advance = 0.04;
        s.tool_count = 2;
        let t0 = s.flat_tool("a".into());
        let mut t1 = s.flat_tool("b".into());
        t1.nozzle_temp_c = 250;
        t1.first_layer_nozzle_temp_c = 255;
        t1.pressure_advance = 0.08;
        t1.filament_diameter_mm = 2.85;
        t1.fan_speed = 0.5;
        t1.bridge_fan_speed = 0.5;
        t1.fan_max = 0.5;
        s.tools = vec![t0, t1];
        s
    }

    /// The tool of the first printable path of a layer (its lead group).
    fn lead_tool(plan: &LayerPlan) -> u32 {
        plan.paths.iter().find(|p| p.points.len() >= 2).expect("printable path").tool
    }

    #[test]
    fn toolchanges_once_per_serpentine_layer() {
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        let chunks: Vec<&str> = g.split("; LAYER ").collect();
        assert_eq!(chunks.len(), plans.len() + 1);
        // The preamble selects the initial tool once; each serpentine layer
        // swaps exactly once, at its group boundary.
        assert_eq!(chunks[0].matches("; toolchange T").count(), 1);
        for (k, c) in chunks[1..].iter().enumerate() {
            assert_eq!(c.matches("toolchange T").count(), 1, "layer {k}: one swap");
        }
        // The default template is the bare Klipper selection macro.
        assert!(g.lines().any(|l| l == "T1"), "T1 macro line emitted");
        // Every used tool is shut off before the end g-code.
        assert!(g.contains("M104 T0 S0") && g.contains("M104 T1 S0"));
    }

    #[test]
    fn retraction_distance_is_per_tool_filament() {
        // Retraction distance rides the filament tier: each slot retracts by
        // ITS OWN value, not one shared machine setting.
        let mut s = two_tool_settings();
        s.retract_len_mm = 0.5; // flat mirror = tool 0
        s.tools[0].retract_len_mm = 0.5;
        s.tools[1].retract_len_mm = 1.2;
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        // Tool 0's pre-dock retract (old tool) pulls back 0.5; tool 1's
        // post-swap re-prime (new tool) pushes 1.2 — distinct distances prove
        // the emitter reads the tool in hand, not a single flat value.
        assert!(g.contains("G1 E-0.50000"), "tool 0 retracts its own 0.5mm");
        assert!(g.contains("G1 E1.20000"), "tool 1 re-primes its own 1.2mm");
    }

    #[test]
    fn preamble_selects_initial_tool_and_preheats_the_rest() {
        let mut s = two_tool_settings();
        s.start_gcode =
            "PRINT_START INITIAL_TOOL={initial_tool} TOOLS={tool_count} EXTRUDER={first_layer_nozzle_temp}"
                .into();
        let plans = two_tool_plans(&s, 0.0);
        let init = lead_tool(&plans[0]);
        let other = 1 - init;
        let g = to_gcode(&plans, &s);
        let pre = &g[..g.find("; LAYER 0 ").unwrap()];
        // Placeholders resolve from the initial tool's slot.
        let want = format!(
            "PRINT_START INITIAL_TOOL={init} TOOLS=2 EXTRUDER={}",
            s.tool(init as usize).first_layer_nozzle_temp_c
        );
        assert!(pre.contains(&want), "start template substituted:\n{pre}");
        // Selection follows the start template; the docked tool preheats to its
        // own first-layer temp (it prints on layer 0).
        let sel = pre.find(&format!("; toolchange T{init}")).expect("initial selection");
        assert!(sel > pre.find("PRINT_START").unwrap());
        let preheat =
            format!("M104 T{other} S{}", s.tool(other as usize).first_layer_nozzle_temp_c);
        assert!(pre.contains(&preheat), "docked preheat:\n{pre}");

        // A part that starts above layer 0 idles at its printing temp instead.
        let plans = two_tool_plans(&s, 2.0);
        assert_eq!(lead_tool(&plans[0]), 0, "only the bed cube prints layer 0");
        let g = to_gcode(&plans, &s);
        let pre = &g[..g.find("; LAYER 0 ").unwrap()];
        assert!(pre.contains("M104 T1 S250"), "idle tool preheats to print temp:\n{pre}");
    }

    #[test]
    fn layer_one_drops_first_layer_temps_per_tool() {
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        let chunk = &g[g.find("; LAYER 1 ").unwrap()..g.find("; LAYER 2 ").unwrap()];
        // Both tools printed layer 0 and both run hotter there: the tool in
        // hand drops via bare M104, the docked one via the T-form.
        let lead = lead_tool(&plans[1]);
        let docked = 1 - lead;
        let active_drop = format!("M104 S{}", s.tool(lead as usize).nozzle_temp_c);
        let docked_drop = format!("M104 T{docked} S{}", s.tool(docked as usize).nozzle_temp_c);
        assert!(chunk.contains(&active_drop), "active drop missing:\n{chunk}");
        assert!(chunk.contains(&docked_drop), "docked drop missing:\n{chunk}");
    }

    #[test]
    fn toolchange_reestablishes_z_pa_fan_and_feed() {
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        // The swap inside layer 1 — the part fan is live there.
        let chunk = &g[g.find("; LAYER 1 ").unwrap()..g.find("; LAYER 2 ").unwrap()];
        let at = chunk.find("; toolchange T").unwrap();
        let after = &chunk[at..];
        let new_tool: u32 =
            after["; toolchange T".len()..].lines().next().unwrap().trim().parse().unwrap();
        // Up to the first extruding move: Z re-issued, accel/PA/fan forced back
        // out at the new tool's values.
        let block = &after[..after.find("G1 X").unwrap()];
        assert!(block.lines().any(|l| l.starts_with("G1 Z")), "Z re-issued:\n{block}");
        assert!(block.contains("M204 S"), "accel re-asserted:\n{block}");
        let want_pa = format!(
            "SET_PRESSURE_ADVANCE ADVANCE={:.4}",
            s.tool(new_tool as usize).pressure_advance
        );
        assert!(block.contains(&want_pa), "new tool's PA:\n{block}");
        let want_fan =
            format!("M106 S{}", (s.tool(new_tool as usize).fan_speed * 255.0).round() as u32);
        assert!(block.contains(&want_fan), "new tool's fan duty:\n{block}");
        // The macro moves under its own F words: the first motion after it must
        // re-assert F even though the requested feed never changed.
        let first_motion = after
            .lines()
            .find(|l| l.starts_with("G0 ") || l.starts_with("G1 "))
            .expect("motion after the macro");
        assert!(first_motion.contains(" F"), "feed re-asserted: {first_motion}");
    }

    #[test]
    fn toolchange_seconds_add_per_swap() {
        let mut s = two_tool_settings();
        s.toolchange_seconds = 0.0;
        let plans = two_tool_plans(&s, 0.0);
        let base = estimate_seconds(&plans, &s);
        s.toolchange_seconds = 12.0;
        let with = estimate_seconds(&plans, &s);
        // Emitted swaps (the per-layer boundaries, not the initial selection)
        // are exactly what the estimate charges for.
        let g = to_gcode(&plans, &s);
        let swaps = g.matches("; toolchange T").count() - 1;
        assert_eq!(swaps, plans.len(), "serpentine: one swap per layer");
        assert!(
            (with - base - 12.0 * swaps as f64).abs() < 1.0e-6,
            "{with} vs {base} + 12 x {swaps}"
        );
        // The per-layer map shares the same term.
        let sum: f64 = per_layer_stats(&plans, &s).iter().map(|st| st.secs).sum();
        assert!((sum - with).abs() < with * 1.0e-9);
    }

    #[test]
    fn per_tool_filament_sums_to_aggregate() {
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let per = estimate_filament_per_tool(&plans, &s);
        assert_eq!(per.len(), 2);
        assert_eq!((per[0].0, per[1].0), (0, 1), "ascending tool order");
        let (mm, grams) = estimate_filament(&plans, &s);
        let (mm_sum, g_sum) = per.iter().fold((0.0, 0.0), |a, p| (a.0 + p.1, a.1 + p.2));
        assert!((mm_sum - mm).abs() < 1.0e-9 * mm.max(1.0));
        assert!((g_sum - grams).abs() < 1.0e-9 * grams.max(1.0));
        // Equal geometry through a fatter filament: fewer mm by the area ratio.
        let ratio = (2.85f64 / 1.75).powi(2);
        assert!(
            (per[0].1 / per[1].1 - ratio).abs() < 0.02,
            "mm ratio {} vs {ratio}",
            per[0].1 / per[1].1
        );
        // And the header reports each slot.
        let g = to_gcode(&plans, &s);
        assert!(g.contains("; filament used [T0] = "));
        assert!(g.contains("; filament used [T1] = "));
    }

    #[test]
    fn multiline_toolchange_template_emits_verbatim() {
        let mut s = two_tool_settings();
        s.toolchange_gcode =
            "SAVE_GCODE_STATE NAME=tc\nT{tool}\nRESTORE_GCODE_STATE NAME=tc".into();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        let at = g.find("; toolchange T1").unwrap();
        let mut lines = g[at..].lines().skip(1);
        assert_eq!(lines.next(), Some("SAVE_GCODE_STATE NAME=tc"));
        assert_eq!(lines.next(), Some("T1"));
        assert_eq!(lines.next(), Some("RESTORE_GCODE_STATE NAME=tc"));
    }

    #[test]
    fn single_tool_output_has_no_tool_vocabulary() {
        let m = mesh::Mesh::cube(10.0);
        let s = Settings::default();
        let plans = generate(&m, &s);
        let g = to_gcode(&plans, &s);
        assert!(!g.contains("toolchange"), "no selection on a single-tool machine");
        assert!(!g.contains("M104 T"), "no T-form setpoints");
        assert!(!g.lines().any(|l| l == "T0"), "no bare T macro");
        assert!(!g.contains("filament used [T"), "no per-tool header lines");
        // The per-tool estimate degenerates to the aggregate.
        let per = estimate_filament_per_tool(&plans, &s);
        let (mm, grams) = estimate_filament(&plans, &s);
        assert_eq!(per.len(), 1);
        assert!((per[0].1 - mm).abs() < 1.0e-12 && (per[0].2 - grams).abs() < 1.0e-12);
    }

    #[test]
    fn overhang_stretches_never_reissue_pa_mid_bead() {
        // A wall that rolls into a slope used to re-issue SET_PRESSURE_ADVANCE
        // at every overhang-band boundary INSIDE the printing loop. Klipper
        // flushes its lookahead on that command — a dead stop with the nozzle
        // parked on the visible wall, whose ooze prints as a blob (a
        // StealthBurner main body collected 470 of them across its sloped
        // cowl). The stretch keeps its slowdown and label; PA holds the
        // path-start value until the next travel.
        //
        // A box with one face sheared 55° past vertical: every wall loop is
        // MIXED (three supported faces + one overhang stretch), the shape
        // that used to thrash PA at the run boundaries.
        let sh = 8.0 * 55f64.to_radians().tan();
        let v = [
            [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 10.0, 0.0], [0.0, 10.0, 0.0],
            [0.0, sh, 8.0], [20.0, sh, 8.0], [20.0, 10.0 + sh, 8.0], [0.0, 10.0 + sh, 8.0],
        ];
        let idx: [[usize; 3]; 12] = [
            [0, 2, 1], [0, 3, 2], // bottom
            [4, 5, 6], [4, 6, 7], // top
            [0, 1, 5], [0, 5, 4], // front (stays vertical-ish)
            [2, 7, 6], [2, 3, 7], // back (the 55° slope)
            [0, 4, 7], [0, 7, 3], // left
            [1, 6, 5], [1, 2, 6], // right
        ];
        let tris: Vec<[[f64; 3]; 3]> = idx.iter().map(|t| [v[t[0]], v[t[1]], v[t[2]]]).collect();
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.pressure_advance = 0.04;
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(g.contains(";TYPE:Overhang wall"), "the slope is still classified and slowed");
        let lines: Vec<&str> = g.lines().collect();
        let extrude = |l: &str| {
            l.starts_with("G1 ") && l.contains(" E") && !l.contains(" E-")
                && (l.contains(" X") || l.contains(" Y"))
        };
        let motion = |l: &str| {
            l.starts_with("G0 ") || l.starts_with("G1 ") || l.starts_with("G2 ") || l.starts_with("G3 ")
        };
        let mut prev = "";
        for (i, l) in lines.iter().enumerate() {
            if l.starts_with("SET_PRESSURE_ADVANCE") {
                let nxt = lines[i + 1..].iter().find(|m| motion(m)).copied().unwrap_or("");
                assert!(
                    !(extrude(prev) && extrude(nxt)),
                    "PA re-issued mid-bead at line {i}: a planner flush parks the nozzle on the wall"
                );
            }
            if motion(l) {
                prev = l;
            }
        }
    }

    /// The g-code of one layer (from its "; LAYER N " marker to the next).
    fn layer_chunk(g: &str, n: usize) -> &str {
        let start = g.find(&format!("; LAYER {n} ")).unwrap();
        let rest = &g[start..];
        let end = rest[1..].find("; LAYER ").map(|i| i + 1).unwrap_or(rest.len());
        &rest[..end]
    }

    fn single_wall_cube() -> Settings {
        let mut s = Settings::default();
        s.wall_count = 1;
        s.top_layers = 0;
        s.bottom_layers = 0;
        s.infill_density = 0.0;
        s
    }

    /// Lines that actually lay a bead: a move with filament behind it. A pure
    /// E move is a retract or its unretract — fixed amounts that no flow
    /// multiplier touches, and counting them dilutes every ratio measured here.
    fn extruding_lines(g: &str, n: usize) -> impl Iterator<Item = &str> {
        layer_chunk(g, n).lines().filter(|l| {
            (l.starts_with("G1 ") || l.starts_with("G2 ") || l.starts_with("G3 "))
                && l.contains(" E")
                && !l.contains("E-")
                && (l.contains(" X") || l.contains(" Y"))
        })
    }

    /// Filament extruded (mm) into beads inside one layer's chunk of g-code.
    fn layer_e_mm(g: &str, n: usize) -> f64 {
        extruding_lines(g, n)
            .filter_map(|l| l.split_whitespace().find_map(|w| w.strip_prefix('E')))
            .filter_map(|v| v.parse::<f64>().ok())
            .sum()
    }

    /// The slowest feed (mm/min) any bead on that layer is laid at.
    fn slowest_bead_feed(g: &str, n: usize) -> f64 {
        let mut cur = f64::NAN;
        let mut worst = f64::MAX;
        for l in extruding_lines(g, n) {
            if let Some(f) = l.split_whitespace().find_map(|w| w.strip_prefix('F')) {
                cur = f.parse().unwrap_or(cur);
            }
            if cur.is_finite() {
                worst = worst.min(cur);
            }
        }
        worst
    }

    #[test]
    fn the_first_layer_flow_scales_the_first_layer_and_nothing_else() {
        // The volume twin of the first-layer speed: extra plastic for what a
        // textured plate swallows. It must land on layer 0 alone — a global
        // multiplier is what `flow ×` already is.
        let s = single_wall_cube();
        let plans = generate(&mesh::Mesh::cube(10.0), &s);
        let base = to_gcode(&plans, &s);

        let mut fat = s.clone();
        fat.first_layer_flow = 1.2;
        let g = to_gcode(&generate(&mesh::Mesh::cube(10.0), &fat), &fat);

        let (e0, e0f) = (layer_e_mm(&base, 0), layer_e_mm(&g, 0));
        let (e1, e1f) = (layer_e_mm(&base, 1), layer_e_mm(&g, 1));
        assert!(e0 > 0.0 && e1 > 0.0, "no extrusion to measure ({e0}, {e1})");
        assert!(
            (e0f / e0 - 1.2).abs() < 0.01,
            "first layer scaled by {:.3}, expected 1.2",
            e0f / e0
        );
        assert!(
            (e1f / e1 - 1.0).abs() < 0.001,
            "the second layer moved too, by {:.3}",
            e1f / e1
        );
    }

    #[test]
    fn a_fattened_first_layer_slows_itself_instead_of_skipping() {
        // The multiplier goes through `flow_factor_kind`, so the melt-rate
        // clamp in `feed_for_seg` sees it: asking for 40% more plastic at a
        // ceiling that already binds has to come out of the feed, or the
        // extruder just skips and blobs downstream.
        let mut s = single_wall_cube();
        // Low enough that the ceiling — not the first-layer speed — is what
        // sets the feed in BOTH runs; otherwise the baseline is speed-limited
        // and the ratio measures nothing.
        s.max_volumetric_speed_mm3_s = 1.0;
        let slow = to_gcode(&generate(&mesh::Mesh::cube(10.0), &s), &s);

        let mut fat = s.clone();
        fat.first_layer_flow = 1.4;
        let g = to_gcode(&generate(&mesh::Mesh::cube(10.0), &fat), &fat);

        let (a, b) = (slowest_bead_feed(&slow, 0), slowest_bead_feed(&g, 0));
        assert!(
            (a / b - 1.4).abs() < 0.05,
            "feed went {a:.0} -> {b:.0} (x{:.3}); 40% more plastic should cost 40% of the feed",
            a / b
        );
    }

    #[test]
    fn the_first_layer_speed_comes_off_the_filament_card() {
        // It moved out of the motion profile: the card decides how gently a
        // material is laid onto bare plate, the way it already decides its
        // bridge speed. Layer 0 takes it whole, every feature alike.
        let mut s = single_wall_cube();
        s.first_layer_speed_mm_s = 17.0;
        let g = to_gcode(&generate(&mesh::Mesh::cube(10.0), &s), &s);
        let feeds: Vec<f64> = extruding_lines(&g, 0)
            .filter_map(|l| l.split_whitespace().find_map(|w| w.strip_prefix('F')))
            .filter_map(|v| v.parse::<f64>().ok())
            .collect();
        assert!(!feeds.is_empty(), "no extruding feeds on the first layer");
        for f in feeds {
            assert!((f - 17.0 * 60.0).abs() < 1.0, "first-layer feed {f} is not 17 mm/s");
        }
    }

    #[test]
    fn joined_outer_wall_flows_from_its_inner_wall_without_a_stop() {
        // A default two-wall cube: the outer wall's donor (its inner wall /
        // spiralized shell stroke) ends next to the outer seam and the outer
        // wall is entered AT PRESSURE — between the donor's last extrude and
        // the outer wall's first there must be NO travel, NO retract, and NO
        // unretract; just attribute lines and the junction extrude. And a
        // joined wall prints flat: no extruding-Z ramp in its body.
        let mut s = Settings::default();
        s.top_layers = 0;
        s.bottom_layers = 0;
        s.infill_density = 0.0;
        let plans = generate(&mesh::Mesh::cube(20.0), &s);
        let joined = plans[40].paths.iter().filter(|p| p.joined).count();
        assert!(joined >= 1, "the cube's outer wall joins its inner wall on layer 40");
        let g = to_gcode(&plans, &s);
        let chunk = layer_chunk(&g, 40);
        let wall = chunk.find(";TYPE:Outer wall").expect("outer wall in layer 40");
        // Walk BACK from the marker to the donor's last extrude, and FORWARD to
        // the wall's first: nothing between them may lift, travel, or retract.
        let before = &chunk[..wall];
        let after = &chunk[wall..];
        let donor_tail = &before[before.rfind(" E").map(|i| before[..i].rfind('\n').unwrap_or(0)).unwrap_or(0)..];
        let first_ex = after
            .lines()
            .position(|l| l.starts_with("G1 ") && l.contains(" E") && !l.contains(" E-"))
            .expect("outer wall extrudes");
        let between: Vec<&str> = donor_tail
            .lines()
            .skip(1)
            .chain(after.lines().take(first_ex))
            .collect();
        for l in &between {
            assert!(
                !l.starts_with("G0 ") && !l.contains("G1 E") && !l.starts_with("G1 Z"),
                "no travel/retract/unretract/Z between donor and joined wall, found: {l}"
            );
        }
        // The loop prints at one height throughout — nothing extrudes while
        // moving Z anywhere in the file. (The taper seam that used to do that
        // is gone: it ran the nozzle in contact with the layer below.)
        let wall_body = &after[..after[1..].find(";TYPE:").map(|i| i + 1).unwrap_or(after.len())];
        assert!(
            !wall_body.lines().any(|l| l.starts_with("G1 ") && l.contains(" Z") && l.contains(" E")),
            "an extruding move changed Z inside a wall loop"
        );
    }

    #[test]
    fn overhang_speed_uses_reference_tiers() {
        // Tier table matched to the reference profile: 50 / 30 / floor mm/s
        // as the bead goes from a quarter unsupported to fully airborne. The
        // old graded ramp let moderately-unsupported beads coast at 78-113
        // mm/s — the melt has to freeze onto whatever support it has before
        // the nozzle drags it, and that takes a hard slowdown.
        let mut s = Settings::default();
        s.external_perimeter_speed_mm_s = 160.0;
        s.overhang_speed_mm_s = 20.0;
        let tool = s.flat_tool("x".into());
        {
            let at =
                |deg: f32| super::nominal_speed_mm_s(PathKind::OverhangWall, deg, 1, &tool, &s);
            assert_eq!(at(0.25), 30.0, "half-the-bead-on-air tier");
            assert_eq!(at(0.5), 30.0, "half-to-three-quarters tier");
            assert_eq!(at(0.75), 20.0, "airborne floor");
            assert_eq!(at(1.0), 20.0, "airborne floor");
            assert!(at(0.25) >= at(0.5) && at(0.5) >= at(1.0), "monotone");
        }
        // A slow outer wall caps the tiers; a slower floor still wins below.
        s.external_perimeter_speed_mm_s = 25.0;
        let tool = s.flat_tool("x".into());
        let at = |deg: f32| super::nominal_speed_mm_s(PathKind::OverhangWall, deg, 1, &tool, &s);
        assert_eq!(at(0.25), 25.0, "tier clamped to the wall ceiling");
        assert_eq!(at(1.0), 20.0, "floor unaffected by the ceiling clamp");
    }

    #[test]
    fn overhang_slowdown_ramps_instead_of_stepping() {
        // A sheared cylinder: a smooth curved shell rolling into a 75° slope,
        // so every wall loop crosses the full overhang grade mid-arc (75°
        // reaches degree 1.0 — a 55° shear only grazes 0.1 and never
        // exercises the floor) — the StealthBurner-cowl shape that used to
        // step the commanded feed by the full wall→overhang gap at one
        // junction and print a zit ring near the seam. Every junction's feed
        // change must now respect the piece budget, while the slow zone
        // still reaches its full slowdown.
        let (r, h, n) = (8.0, 8.0, 64);
        let shear = h * 75f64.to_radians().tan();
        let ring = |z: f64| -> Vec<[f64; 3]> {
            (0..n)
                .map(|i| {
                    let a = std::f64::consts::TAU * i as f64 / n as f64;
                    [r * a.cos(), r * a.sin() + shear * z / h, z]
                })
                .collect()
        };
        let (bot, top) = (ring(0.0), ring(h));
        let mut tris = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            tris.push([bot[i], bot[j], top[j]]);
            tris.push([bot[i], top[j], top[i]]);
        }
        let c0 = [0.0, 0.0, 0.0];
        let c1 = [0.0, shear, h];
        for i in 0..n {
            let j = (i + 1) % n;
            tris.push([c0, bot[j], bot[i]]);
            tris.push([c1, top[i], top[j]]);
        }
        let m = mesh::Mesh::from_triangle_soup(&tris);
        let mut s = Settings::default();
        s.pressure_advance = 0.04;
        // A FAST profile, or the test is vacuous: at the default 25 mm/s the
        // band grading's own steps never exceed the budget even with the
        // limiter deleted. At 150 → 10 mm/s the unlimited emitter steps
        // 140 mm/s at one junction — reverting the limiter must trip this.
        s.external_perimeter_speed_mm_s = 150.0;
        s.overhang_speed_mm_s = 10.0;
        s.max_volumetric_speed_mm3_s = 30.0;
        // No layer-time throttle: the tiny test cylinder would otherwise
        // print at a per-layer speed_scale and mask the real feed spread.
        s.min_layer_time_s = 0.0;
        let g = to_gcode(&generate(&m, &s), &s);
        assert!(g.contains(";TYPE:Overhang wall"), "the slope grades");
        // Walk wall extrusions: track position + feed; every mid-bead feed
        // change must fit the slew budget for that segment's length.
        let num = |l: &str, k: &str| {
            l.split(k).nth(1).and_then(|v| v.split(' ').next().unwrap_or("").parse::<f64>().ok())
        };
        let (mut x, mut y, mut wall, mut last_ex) = (0.0f64, 0.0f64, false, false);
        let (mut curf, mut fmin, mut fmax, mut steps) = (None::<f64>, f64::MAX, 0.0f64, 0);
        for l in g.lines() {
            if l == ";JOIN" {
                // A pressure-joined entry: the junction crosses two ~90°
                // corners that clamp to square-corner velocity, so the feed
                // change from the donor into the junction is not a mid-bead
                // step. The junction itself is emitted at the wall's
                // slew-limited first-piece feed, so everything AFTER it stays
                // measured.
                curf = None;
                continue;
            }
            if let Some(t) = l.strip_prefix(";TYPE:") {
                wall = matches!(t.trim(), "Outer wall" | "Overhang wall");
                continue;
            }
            let (nx, ny) = (num(l, " X"), num(l, " Y"));
            let motion = l.starts_with("G0 ") || l.starts_with("G1 ");
            let ex = l.starts_with("G1 ")
                && l.contains(" E")
                && !l.contains(" E-")
                && (nx.is_some() || ny.is_some());
            if ex && wall {
                if let Some(f) = num(l, " F") {
                    if last_ex {
                        if let Some(c) = curf {
                            // The slew plan works at FEED_PIECE_MM granularity,
                            // so no junction may step more than one piece's
                            // budget (≈10 mm/s) — never the old 150 mm/s cliff.
                            let budget = FEED_SLEW_PER_MM * super::FEED_PIECE_MM + 3.0;
                            assert!(
                                (f - c).abs() <= budget,
                                "feed stepped {c}→{f} (budget {budget:.0})"
                            );
                            steps += 1;
                        }
                    }
                    curf = Some(f);
                }
                if let Some(f) = curf {
                    fmin = fmin.min(f);
                    fmax = fmax.max(f);
                }
            }
            if motion && (nx.is_some() || ny.is_some()) {
                x = nx.unwrap_or(x);
                y = ny.unwrap_or(y);
            }
            last_ex = ex;
        }
        assert!(steps > 20, "the ramp is graded across many junctions, got {steps}");
        // Non-vacuity: the fast profile really cruises, so an unlimited
        // emitter would step ~140 mm/s at one junction and trip the budget.
        assert!(
            fmax >= s.external_perimeter_speed_mm_s * 60.0 * 0.9,
            "wall never reached cruise: max F{fmax:.0}"
        );
        // The deep-overhang floor is still reached — the min-only limiter
        // never robs the slow zone of its slowdown.
        assert!(
            fmin < s.overhang_speed_mm_s * 60.0 + 1.0,
            "slow zone floor lost: min F{fmin:.0}"
        );
    }

    #[test]
    fn spiral_vase_ramps_z_continuously() {
        let m = mesh::Mesh::cube(20.0);
        let mut s = Settings::default();
        s.spiral_vase = true;
        s.bottom_layers = 2;
        s.skirt_loops = 0;
        let layers = generate(&m, &s);
        let g = to_gcode(&layers, &s);
        // Above the bottom, extruding moves carry Z (G1 X.. Y.. Z.. E..).
        let spiral_moves = g
            .lines()
            .filter(|l| l.starts_with("G1 X") && l.contains(" Z") && l.contains(" E"))
            .count();
        assert!(spiral_moves > 100, "vase should extrude with rising Z, got {spiral_moves}");
        // Z never decreases over the spiral moves.
        let mut last_z = 0.0;
        for line in g.lines().filter(|l| l.starts_with("G1 X") && l.contains(" Z")) {
            let z: f64 = line
                .split_whitespace()
                .find_map(|t| t.strip_prefix('Z').and_then(|v| v.parse().ok()))
                .unwrap();
            assert!(z >= last_z - 1e-6, "vase Z must never drop: {z} after {last_z}");
            last_z = z;
        }
        // No retractions once the spiral starts (continuous extrusion) — only
        // the single end-of-print park retract is allowed.
        let spiral_start = g.find("; LAYER 3 ").unwrap();
        let tail = &g[spiral_start..g.find("M73 P100").unwrap()];
        assert!(tail.matches("G1 E-").count() <= 1, "no retractions inside the spiral");
    }

    #[test]
    fn tool_done_early_drops_to_standby_for_the_rest_of_the_print() {
        // Tool 1 finishes near the bottom of a tall tool-0 print: at its final
        // release it parks at standby (250 − 50 = 200) instead of cooking at
        // print temperature for the remaining hour; never reheated after.
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 20.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, 0.0], [40.0, 10.0, 4.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = two_tool_settings();
        s.standby_after_s = 5.0;
        s.tools[1].standby_temp_c = 200; // what resolve derives for a 250° spool
        let plans = generate_parts(&[(&a, 0), (&b, 1)], &s);
        let g = to_gcode(&plans, &s);
        let park = g.find("M104 T1 S200").expect("tool 1 parks at standby");
        assert!(!g[park..].contains("M104 T1 S25"), "never reheated after its last use");
        assert!(!g.contains("M109 T"), "no pickup, no blocking wait");
        // Tool 0 works to the end: it never parks (its standby is 160).
        assert!(!g.contains("M104 T0 S160"), "tool 0 has no standby drop");
    }

    #[test]
    fn long_dock_reheats_a_layer_ahead_and_confirms_at_pickup() {
        // Tool 1 prints the bottom band and a top band of the same part with a
        // long tool-0-only stretch between: it drops at the release, gets its
        // M104 reheat one layer before the pickup, and the pickup swap opens
        // with a blocking M109 confirm.
        let mut ta = Vec::new();
        push_box(&mut ta, [0.0, 0.0, 0.0], [10.0, 10.0, 20.0]);
        let a = mesh::Mesh::from_triangle_soup(&ta);
        let mut tb = Vec::new();
        push_box(&mut tb, [30.0, 0.0, 0.0], [40.0, 10.0, 2.0]);
        push_box(&mut tb, [30.0, 0.0, 16.0], [40.0, 10.0, 20.0]);
        let b = mesh::Mesh::from_triangle_soup(&tb);
        let mut s = two_tool_settings();
        s.standby_after_s = 5.0;
        s.tools[1].standby_temp_c = 200;
        let plans = generate_parts(&[(&a, 0), (&b, 1)], &s);
        // The pickup layer: tool 1's first appearance above the gap.
        let pickup = plans
            .iter()
            .find(|p| p.print_z_mm > 10.0 && p.paths.iter().any(|q| q.tool == 1 && q.points.len() >= 2))
            .map(|p| p.index)
            .expect("tool 1 returns for the top band");
        let g = to_gcode(&plans, &s);
        let park = g.find("M104 T1 S200").expect("tool 1 parks during the gap");
        let reheat = g[park..].find("M104 T1 S250").map(|i| park + i).expect("reheat scheduled");
        let confirm = g[reheat..].find("M109 T1 S250").map(|i| reheat + i).expect("pickup confirms");
        // The reheat lands in the layer BEFORE the pickup; the confirm sits at
        // the pickup swap itself, after that layer starts.
        let lead_mark = format!("; LAYER {} ", pickup - 1);
        let pickup_mark = format!("; LAYER {pickup} ");
        let lead_at = g.find(&lead_mark).unwrap();
        let pickup_at = g.find(&pickup_mark).unwrap();
        assert!(lead_at < reheat && reheat < pickup_at, "reheat rides the lead layer");
        assert!(confirm > pickup_at, "confirm at the swap, not earlier");
        assert!(g[confirm..].find("; toolchange T1").is_some(), "confirm precedes the swap");
    }

    #[test]
    fn quick_alternation_never_thermally_cycles() {
        // The serpentine two-cube print swaps every layer — docks last seconds.
        // With the default threshold nothing drops, nothing waits: dithered
        // blends must not thermal-cycle the hotends.
        let s = two_tool_settings();
        let plans = two_tool_plans(&s, 0.0);
        let g = to_gcode(&plans, &s);
        assert!(!g.contains("M104 T1 S160"), "no standby drops on quick alternation");
        assert!(!g.contains("M104 T0 S160"), "tool 0 likewise");
        assert!(!g.contains("M109 T"), "no blocking waits");
    }


    /// A small feature is paced whether or not its loop is closed, and the gap
    /// stroke inside it is paced with it.
    ///
    /// Both halves were broken and both are load-bearing. `apply_seam_gap`
    /// opens every wall loop, so keying the pace on `path.closed` fired for
    /// nothing — hidden only while outer walls were exempt from that trim
    /// because they scarfed. And gap fill took the full external-perimeter
    /// speed regardless: on a 0.9 mm free-standing rib that meant a 126 mm/s
    /// stroke down the middle of two beads laid moments earlier.
    #[test]
    fn a_small_feature_is_paced_and_so_is_the_gap_inside_it() {
        let mut s = single_wall_cube();
        s.wall_count = 2;
        // A 6 mm cube: 24 mm around, comfortably inside SMALL_LOOP_MM.
        let plans = generate(&mesh::Mesh::cube(6.0), &s);
        let nominal = s.external_perimeter_speed_mm_s;
        let g = to_gcode(&plans, &s);

        // Layer 2+ (the first layer is exempt — its speed is an absolute).
        let chunk = layer_chunk(&g, 3);
        let feeds: Vec<f64> = chunk
            .lines()
            .filter(|l| l.starts_with("G1 ") && l.contains(" X") && l.contains(" E") && !l.contains("E-"))
            .filter_map(|l| l.split(" F").nth(1).and_then(|f| f.split(' ').next().unwrap_or("").parse::<f64>().ok()))
            .collect();
        assert!(!feeds.is_empty(), "no feeds on the wall");
        let fastest = feeds.iter().cloned().fold(0.0f64, f64::max) / 60.0;
        assert!(
            fastest < nominal * 0.95,
            "a {nominal:.0} mm/s nominal on a 24 mm loop should be paced, got {fastest:.1} mm/s"
        );

        // The pace is a property of the path, not of its closure: a loop that
        // apply_seam_gap has opened is still a small loop.
        let mut open_loop = plans[3].paths[0].clone();
        assert!(!open_loop.closed, "apply_seam_gap should have opened this loop");
        let paced = super::small_loop_factor(&open_loop);
        assert!(paced < 1.0, "an opened small loop must still pace, got {paced}");
        open_loop.closed = true;
        assert!(
            (super::small_loop_factor(&open_loop) - paced).abs() < 0.05,
            "closing the loop should barely change its pace"
        );

        // And a short gap stroke paces like the walls it sits between.
        let mut gap = plans[3].paths[0].clone();
        gap.kind = PathKind::GapFill;
        gap.closed = false;
        assert!(
            super::small_loop_factor(&gap) < 1.0,
            "a short gap stroke must be paced, not run at wall speed"
        );
    }

    /// Everything inside a small ISLAND paces at the island's factor — the
    /// island is the thermal unit. A spiralized interior stroke whose summed
    /// length clears SMALL_LOOP_MM used to escape the per-path rule and dump
    /// most of a small island's volume at full speed while its outer wall was
    /// being paced.
    #[test]
    fn a_small_island_paces_everything_inside_it() {
        // Two islands on one layer: a 12x2 fin (perimeter 28 mm — small) and
        // a 100x100 slab (perimeter 400 mm — not).
        let mut outline = Polygons::new();
        outline.push(Contour::new(vec![
            Point::from_mm(0.0, 0.0),
            Point::from_mm(12.0, 0.0),
            Point::from_mm(12.0, 2.0),
            Point::from_mm(0.0, 2.0),
        ]));
        outline.push(Contour::new(vec![
            Point::from_mm(20.0, 0.0),
            Point::from_mm(120.0, 0.0),
            Point::from_mm(120.0, 100.0),
            Point::from_mm(20.0, 100.0),
        ]));
        // A long serpentine Perimeter stroke inside the fin (60 mm — its own
        // length clears the threshold), a bridge inside the fin, and a wall
        // inside the slab.
        let mut serp = Vec::new();
        for k in 0..6 {
            let y = 0.6 + 0.16 * (k as f64);
            serp.push(Point::from_mm(if k % 2 == 0 { 1.0 } else { 11.0 }, y));
            serp.push(Point::from_mm(if k % 2 == 0 { 11.0 } else { 1.0 }, y));
        }
        let paths = vec![
            bead(PathKind::Perimeter, false, serp),
            bead(
                PathKind::Bridge,
                false,
                vec![Point::from_mm(2.0, 1.0), Point::from_mm(10.0, 1.0)],
            ),
            bead(
                PathKind::Perimeter,
                false,
                vec![Point::from_mm(30.0, 50.0), Point::from_mm(110.0, 50.0)],
            ),
        ];
        let plan = &one_layer(paths, outline)[0];
        let f = super::island_pace_factors(plan);
        let fin = 28.0 / super::SMALL_ISLAND_MM;
        assert!(
            (f[0] - fin).abs() < 1e-9,
            "the fin's interior stroke must take the island factor {fin:.2}, got {:.2}",
            f[0]
        );
        assert!(
            super::small_loop_factor(&plan.paths[0]) >= 1.0,
            "precondition: the stroke's own length must NOT pace it (the escape)"
        );
        assert_eq!(f[1], 1.0, "a bridge keeps its calibrated strand speed");
        assert_eq!(f[2], 1.0, "the big island is not paced");
    }

    /// A pillar standing inside another island's hole is its own island: the
    /// containment parity must hand it the pillar's factor, not the ring's.
    #[test]
    fn island_pace_resolves_nesting_through_holes() {
        let mut outline = Polygons::new();
        // A 60x60 ring with a 40x40 hole, and an 8x8 pillar inside the hole.
        for (lo, hi) in [(0.0, 60.0), (10.0, 50.0), (26.0, 34.0)] {
            outline.push(Contour::new(vec![
                Point::from_mm(lo, lo),
                Point::from_mm(hi, lo),
                Point::from_mm(hi, hi),
                Point::from_mm(lo, hi),
            ]));
        }
        let paths = vec![bead(
            PathKind::Perimeter,
            false,
            vec![Point::from_mm(28.0, 30.0), Point::from_mm(32.0, 30.0)],
        )];
        let plan = &one_layer(paths, outline)[0];
        let f = super::island_pace_factors(plan);
        let pillar = 32.0 / super::SMALL_ISLAND_MM;
        assert!(
            (f[0] - pillar).abs() < 1e-9,
            "the pillar wall must take the pillar's factor {pillar:.2}, got {:.2}",
            f[0]
        );
    }
}
