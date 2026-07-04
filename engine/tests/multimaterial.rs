//! End-to-end multi-material: the three-part figurine fixture (body/cap/emblem
//! on extruders 1/2/3) loads with its config names and hints, slices per part
//! on one grid, and emits toolchanger g-code with per-tool state.

use config::Settings;

fn fixture() -> Vec<(String, u32, mesh::Mesh)> {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../fixtures/figurine.3mf");
    let items = mesh::load_3mf(path).expect("fixture loads");
    assert_eq!(items.len(), 1, "one build item");
    let item = items.into_iter().next().unwrap();
    assert_eq!(item.name, "figurine");
    item.parts
        .into_iter()
        .map(|p| {
            let tool = p.extruder.expect("config assigns every part") - 1;
            (p.name, tool, p.mesh)
        })
        .collect()
}

fn three_tool_settings() -> Settings {
    let mut s = Settings::default();
    s.skirt_loops = 0;
    s.tool_count = 3;
    s.toolchange_seconds = 8.0;
    s.tools = (0..3)
        .map(|i| {
            let mut t = s.flat_tool(format!("tone-{i}"));
            // Distinct temps so per-tool emission is observable.
            t.nozzle_temp_c = 200 + i * 10;
            t.first_layer_nozzle_temp_c = 205 + i * 10;
            t
        })
        .collect();
    s
}

#[test]
fn figurine_slices_and_emits_toolchanger_gcode() {
    let parts = fixture();
    let names: Vec<&str> = parts.iter().map(|(n, _, _)| n.as_str()).collect();
    assert_eq!(names, ["body", "cap", "emblem"], "config names in document order");
    assert_eq!(
        parts.iter().map(|&(_, t, _)| t).collect::<Vec<_>>(),
        [0, 1, 2],
        "extruder hints map to tools"
    );

    let s = three_tool_settings();
    let refs: Vec<(&mesh::Mesh, u32)> = parts.iter().map(|(_, t, m)| (m, *t)).collect();
    let plans = engine::generate_parts(&refs, &s);
    assert!(!plans.is_empty());

    // Every part put material somewhere, and each layer is tool-grouped
    // (no tool appears twice with another tool in between).
    let mut seen = [false; 3];
    for plan in &plans {
        let mut order: Vec<u32> = Vec::new();
        for p in &plan.paths {
            seen[p.tool as usize] = true;
            if order.last() != Some(&p.tool) {
                order.push(p.tool);
            }
        }
        let mut dedup = order.clone();
        dedup.sort_unstable();
        dedup.dedup();
        assert_eq!(order.len(), dedup.len(), "layer {}: tool revisited: {order:?}", plan.index);
    }
    assert_eq!(seen, [true; 3], "all three tools print");

    // The cap prints only above the body (z > 8), the emblem only low.
    for plan in &plans {
        let has = |tool: u32| plan.paths.iter().any(|p| p.tool == tool);
        if plan.print_z_mm < 7.8 {
            assert!(!has(1), "cap material below its z range at layer {}", plan.index);
        }
        if plan.print_z_mm > 8.5 {
            assert!(!has(2), "emblem material above its z range at layer {}", plan.index);
        }
    }

    // Per-tool filament splits are all real and sum to the aggregate.
    let per_tool = engine::estimate_filament_per_tool(&plans, &s);
    assert_eq!(per_tool.len(), 3);
    assert!(per_tool.iter().all(|&(_, mm, g)| mm > 0.0 && g > 0.0));
    let (total_mm, total_g) = engine::estimate_filament(&plans, &s);
    let sum_mm: f64 = per_tool.iter().map(|&(_, mm, _)| mm).sum();
    let sum_g: f64 = per_tool.iter().map(|&(_, _, g)| g).sum();
    assert!((sum_mm - total_mm).abs() < 1e-6 && (sum_g - total_g).abs() < 1e-6);

    let gcode = engine::to_gcode(&plans, &s);
    // Toolchanges for every tool, preheats for the docked ones, per-tool
    // usage in the header, and every tool shut off at the end.
    for n in 0..3 {
        assert!(gcode.contains(&format!("; toolchange T{n}")), "T{n} never selected");
        assert!(gcode.contains(&format!("filament used [T{n}]")), "T{n} missing from header");
        assert!(gcode.contains(&format!("M104 T{n} S0")), "T{n} not shut off");
    }
    assert!(
        gcode.contains("M104 T1 S") && gcode.contains("M104 T2 S"),
        "docked tools get temperature setpoints"
    );

    // The toolchange time term shows up in the estimate.
    let mut free = s.clone();
    free.toolchange_seconds = 0.0;
    let with = engine::estimate_seconds(&plans, &s);
    let without = engine::estimate_seconds(&plans, &free);
    let changes = gcode.matches("; toolchange T").count();
    assert!(changes > 2, "a stacked three-tool print changes tools more than twice");
    let expect = (changes - 1) as f64 * s.toolchange_seconds; // the initial selection is free
    assert!(
        (with - without - expect).abs() < 1e-6,
        "estimate charges {} s for {changes} selections, expected {expect}",
        with - without
    );
}

#[test]
fn blended_cap_dithers_between_white_and_black() {
    // The figurine's cap painted with a 50/50 blend of tools 0 and 2: its
    // layers alternate between the two tools, the split comes out even, and
    // the g-code carries the alternation as ordinary toolchanges.
    let parts = fixture();
    let s = three_tool_settings();
    let painted: Vec<(&mesh::Mesh, engine::PartPaint)> = parts
        .iter()
        .map(|(name, tool, m)| {
            let paint = if name == "cap" {
                engine::PartPaint::Blend(vec![(0, 1.0), (2, 1.0)])
            } else {
                engine::PartPaint::Tool(*tool)
            };
            (m, paint)
        })
        .collect();
    let plans = engine::generate_painted(&painted, &s);

    // Cap layers (z > 8.5, above the body) — one tool per layer, alternating.
    let mut cap_tools = Vec::new();
    for plan in &plans {
        if plan.print_z_mm > 8.5 && !plan.paths.is_empty() {
            let tools: Vec<u32> = plan.paths.iter().map(|p| p.tool).collect();
            assert!(tools.windows(2).all(|w| w[0] == w[1]), "layer {} mixes tools", plan.index);
            cap_tools.push(tools[0]);
        }
    }
    assert!(cap_tools.len() >= 20, "the cap spans ~30 layers");
    for w in cap_tools.windows(2) {
        assert_ne!(w[0], w[1], "a 50/50 blend alternates every layer: {cap_tools:?}");
    }
    let zeros = cap_tools.iter().filter(|&&t| t == 0).count();
    let twos = cap_tools.iter().filter(|&&t| t == 2).count();
    assert!(zeros.abs_diff(twos) <= 1, "even split, got {zeros} vs {twos}");
    assert!(cap_tools.contains(&0) && cap_tools.contains(&2));

    let gcode = engine::to_gcode(&plans, &s);
    // Alternating cap layers each cost one toolchange — far more selections
    // than the uniform paint job's 22.
    let changes = gcode.matches("; toolchange T").count();
    assert!(changes >= cap_tools.len(), "dithered cap must swap about once per cap layer");
}

/// The same three-part figurine on a shared-nozzle machine: one heater ramping
/// between materials, a swap macro carrying the static purge, and the purge
/// counted as waste — no per-tool T-form temperatures anywhere.
#[test]
fn shared_nozzle_emits_single_heater_swaps_with_static_purge() {
    let parts = fixture();
    let mut s = three_tool_settings();
    s.machine_kind = config::MachineKind::SharedNozzle;
    s.purge_volume_mm3 = 100.0;
    s.toolchange_gcode =
        "MMU_CHANGE_TOOL TOOL={tool} FROM={from_tool} TEMP={to_temp} PURGE={purge_mm3}".to_string();

    let refs: Vec<(&mesh::Mesh, u32)> = parts.iter().map(|(_, t, m)| (m, *t)).collect();
    let plans = engine::generate_parts(&refs, &s);
    let gcode = engine::to_gcode(&plans, &s);

    // ONE heater: not a single per-tool T-form temperature command anywhere
    // (those address hotends this machine doesn't have).
    assert!(!gcode.contains("M104 T"), "shared nozzle must not set per-tool heaters (M104 T…)");
    assert!(!gcode.contains("M109 T"), "shared nozzle must not wait on per-tool heaters (M109 T…)");

    // Swaps run the MMU macro with every placeholder substituted, and ramp the
    // shared nozzle with a blocking wait to the incoming filament's temp.
    assert!(gcode.contains("MMU_CHANGE_TOOL TOOL=1"), "swap selects the incoming tool");
    assert!(gcode.contains("FROM=0"), "{{from_tool}} substituted");
    assert!(gcode.contains("PURGE=100.0"), "static purge volume substituted into the macro");
    assert!(gcode.contains("M109 S"), "the one nozzle waits on its ramp at a swap");
    // Tool 1 prints at 210 °C (200 + 1·10); an above-first-layer swap ramps there.
    assert!(gcode.contains("M109 S210"), "ramps + waits to tool 1's print temp");

    // The static purge is counted as waste: the MMU draws more filament than the
    // identical job on a toolchanger, which purges nothing.
    let swaps = gcode.matches("; toolchange T").count().saturating_sub(1); // initial select is free
    assert!(swaps > 0, "the print swaps tools");
    let (mmu_mm, mmu_g) = engine::estimate_filament(&plans, &s);
    let mut tc = s.clone();
    tc.machine_kind = config::MachineKind::IndependentHotends;
    let (tc_mm, _) = engine::estimate_filament(&plans, &tc);
    assert!(mmu_mm > tc_mm, "purge waste ({mmu_mm} mm) must exceed the no-purge job ({tc_mm} mm)");
    assert!(mmu_g > 0.0);

    // Purge lands on the INCOMING tools (1 and 2), so their per-slot totals grow.
    let per_tool = engine::estimate_filament_per_tool(&plans, &s);
    let tc_per = engine::estimate_filament_per_tool(&plans, &tc);
    for &(n, mm, _) in &per_tool {
        if n == 0 {
            continue;
        }
        let base = tc_per.iter().find(|&&(m, ..)| m == n).map(|&(_, mm, _)| mm).unwrap_or(0.0);
        assert!(mm > base, "incoming tool {n} carries its purge waste ({mm} > {base})");
    }
}

/// Separate nozzles sharing ONE heater (indexed nozzles): the single-heater temp
/// ramp of the shared-nozzle machine, but NO purge — each tip keeps its own
/// color, so a swap only waits on the reheat and wastes nothing.
#[test]
fn shared_heater_ramps_temp_but_never_purges() {
    let parts = fixture();
    let mut s = three_tool_settings();
    s.machine_kind = config::MachineKind::SharedHeater;
    s.purge_volume_mm3 = 100.0; // set, but must be ignored (separate nozzles)
    s.toolchange_gcode = "INDEX TOOL={tool} TEMP={to_temp} PURGE={purge_mm3}".to_string();

    let refs: Vec<(&mesh::Mesh, u32)> = parts.iter().map(|(_, t, m)| (m, *t)).collect();
    let plans = engine::generate_parts(&refs, &s);
    let gcode = engine::to_gcode(&plans, &s);

    // Single heater, exactly like the shared nozzle: no per-tool T-form temps,
    // and a blocking ramp+wait at each swap.
    assert!(!gcode.contains("M104 T"), "shared heater is one heater — no M104 T…");
    assert!(!gcode.contains("M109 T"), "shared heater is one heater — no M109 T…");
    assert!(gcode.contains("M109 S210"), "indexed nozzle waits on the reheat to tool 1's temp");

    // But NOTHING is purged: the macro's purge placeholder resolves to 0, and the
    // filament total matches the same job on independent hotends (no waste).
    assert!(gcode.contains("PURGE=0.0"), "no flush on separate nozzles ({{purge_mm3}} = 0)");
    assert!(!gcode.contains("PURGE=100"), "the static purge must be ignored here");
    let (sh_mm, _) = engine::estimate_filament(&plans, &s);
    let mut ih = s.clone();
    ih.machine_kind = config::MachineKind::IndependentHotends;
    let (ih_mm, _) = engine::estimate_filament(&plans, &ih);
    assert!((sh_mm - ih_mm).abs() < 1e-6, "shared heater wastes nothing: {sh_mm} vs {ih_mm}");
}
