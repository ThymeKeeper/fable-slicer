//! Headless offscreen render: slice a model and render one preview layer to a
//! PNG through the *same* `Scene`/`render_to` path the GUI uses — no window, no
//! egui. This is the reliable visual oracle for wall-generation changes: a fix
//! is a single command, pixel-faithful to the GUI, with none of the xdotool
//! fragility of driving the live app.
//!
//! A `.3mf` slices multi-material: every item's parts in document order, each
//! on the tool its extruder hint names (1-based hint − 1; 0 when absent),
//! overridable per part via `TOOLS=0,1,2,…`. A TOOLS entry may also be a
//! blend literal `w:t+w:t…` (`50:0+50:2` = a 50/50 layer dither of tools 0
//! and 2 — the GUI's pseudo colors); `COLOR=filament` colors the beads by
//! tool instead of by feature.

use crate::{build_instances, render::Scene};
use eframe::wgpu;
use glam::{Mat4, Vec3};

pub struct Args {
    /// Model to slice — `.stl`, or `.3mf` (parts print per-tool; see above).
    pub model: std::path::PathBuf,
    pub out: std::path::PathBuf,
    pub layer: usize,
    pub walls: usize,
    pub width: u32,
    pub height: u32,
    /// Camera distance multiplier (smaller = closer).
    pub zoom: f32,
    /// Camera pitch: the -Y tilt of the view direction (0 = straight down).
    pub pitch: f32,
    /// Target offset from the model centre, in mm (frames a corner like the bow).
    pub tx: f32,
    pub ty: f32,
}

pub fn run(a: &Args) -> Result<(), String> {
    // Headless GPU (GL fallback in software is fine; no surface needed).
    let instance = wgpu::Instance::default();
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
        .map_err(|e| format!("no GPU adapter: {e:?}"))?;
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("offscreen"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::default(),
        ..Default::default()
    }))
    .map_err(|e| format!("no GPU device: {e:?}"))?;

    // Slice.
    let mut settings = config::Settings::default();
    settings.wall_count = a.walls;
    // Debug env overrides so the offscreen render can reproduce GUI experiments.
    if let Some(v) = std::env::var("INFILL_DENSITY").ok().and_then(|s| s.parse::<f64>().ok()) {
        settings.infill_density = v;
    }
    if let Some(p) = std::env::var("INFILL_PATTERN").ok().and_then(|s| config::InfillPattern::parse(&s)) {
        settings.sparse_pattern = p;
        settings.solid_pattern = p;
        settings.top_pattern = p;
        settings.bottom_pattern = p;
    }
    if let Some(n) = std::env::var("TOP").ok().and_then(|s| s.parse().ok()) {
        settings.top_layers = n;
    }
    if let Some(n) = std::env::var("BOTTOM").ok().and_then(|s| s.parse().ok()) {
        settings.bottom_layers = n;
    }
    if std::env::var("SUPPORT").is_ok() {
        settings.support_mode = config::SupportMode::Grid;
    }
    let is_3mf = a.model.extension().map(|e| e.eq_ignore_ascii_case("3mf")).unwrap_or(false);
    let layers = if is_3mf {
        // Every item's parts in document order; paint = extruder hint − 1
        // (0 when absent), each overridable in order via TOOLS entries —
        // plain slot indices or blend literals (see the module doc).
        let items = mesh::load_3mf(&a.model).map_err(|e| format!("load {}: {e}", a.model.display()))?;
        let mut parts: Vec<(mesh::Mesh, engine::PartPaint)> = items
            .into_iter()
            .flat_map(|it| it.parts)
            .map(|p| {
                let tool = p.extruder.map(|e| e.saturating_sub(1)).unwrap_or(0);
                (p.mesh, engine::PartPaint::Tool(tool))
            })
            .collect();
        if parts.is_empty() {
            return Err("no printable parts in the 3MF".into());
        }
        if let Ok(list) = std::env::var("TOOLS") {
            for (part, tok) in parts.iter_mut().zip(list.split(',')) {
                part.1 = parse_paint(tok.trim())?;
            }
        }
        let max_tool = |p: &engine::PartPaint| match p {
            engine::PartPaint::Tool(t) => *t,
            engine::PartPaint::Blend(w) => w.iter().map(|&(t, _)| t).max().unwrap_or(0),
            engine::PartPaint::Surface { face_tool, .. } => face_tool.iter().copied().max().unwrap_or(0),
        };
        settings.tool_count = parts.iter().map(|(_, p)| max_tool(p)).max().unwrap_or(0) as usize + 1;
        // Deterministic slot colors (no profiles load here): three greys the
        // eye orders instantly, then a hue spread for bigger changers.
        settings.tools = (0..settings.tool_count)
            .map(|i| {
                let mut t = settings.flat_tool(format!("tool-{i}"));
                t.color_rgb = oracle_tool_color(i);
                t
            })
            .collect();
        let refs: Vec<(&mesh::Mesh, engine::PartPaint)> =
            parts.iter().map(|(m, p)| (m, p.clone())).collect();
        engine::generate_painted(&refs, &settings)
    } else {
        let mesh =
            mesh::Mesh::load_stl(&a.model).map_err(|e| format!("load {}: {e}", a.model.display()))?;
        engine::generate(&mesh, &settings)
    };
    if layers.is_empty() {
        return Err("slice produced no layers".into());
    }
    let layer = a.layer.clamp(1, layers.len());

    // Same bead geometry the GUI builds. COLOR=filament swaps the feature
    // palette for each path's tool color (the slot table above).
    let accent = (190.0, 0.25, 0.55);
    let path_colors: Option<Vec<Vec<[f32; 3]>>> = match std::env::var("COLOR").as_deref() {
        Ok("filament") => Some(
            layers
                .iter()
                .map(|l| {
                    l.paths
                        .iter()
                        .map(|p| {
                            crate::render::visible_against_backdrop(
                                settings.tool(p.tool as usize).color_rgb,
                            )
                        })
                        .collect()
                })
                .collect(),
        ),
        _ => None,
    };
    let (inst, ends, joints, joint_ends) =
        build_instances(&layers, 0.0, path_colors.as_deref(), accent, 0.0, None);
    let count = ends.get(layer - 1).copied().unwrap_or(0);
    let joint_count = joint_ends.get(layer - 1).copied().unwrap_or(0);
    eprintln!("offscreen: instances beads={} joints={} (through layer {layer})", count, joint_count);

    // Scene (4× MSAA — within the WebGPU baseline, so no device feature needed).
    let format = wgpu::TextureFormat::Rgba8Unorm;
    let mut scene = Scene::new_core(&device, format, 4);
    scene.resize_core(&device, a.width, a.height);
    scene.set_toolpaths(&device, &queue, &inst);
    let joint_count = if std::env::var("NO_JOINTS").is_ok() { 0 } else { joint_count };
    if joint_count > 0 {
        scene.set_joints(&device, &queue, &joints);
    }

    // Camera: frame the geometry visible through this layer, from a high front
    // angle (like the GUI's default orbit) so bead-surface detail reads.
    let (mut center, radius) = bounds(&layers, layer);
    center.x += a.tx;
    center.y += a.ty;
    let aspect = a.width as f32 / a.height as f32;
    let dir = Vec3::new(0.0, a.pitch, 1.0).normalize();
    let dist = radius / (22.5_f32.to_radians().tan()) * a.zoom;
    let eye = center + dir * dist;
    let proj = Mat4::perspective_rh(45_f32.to_radians(), aspect.max(0.01), 1.0, 20_000.0);
    let view = Mat4::look_at_rh(eye, center, Vec3::Z);
    let view_proj = proj * view;

    // Show every fill category (bits 0-9), hide travels (bit 4). CAT_MASK (decimal)
    // overrides for diagnostics — e.g. CAT_MASK=2 = walls only (bit 1).
    let mask = std::env::var("CAT_MASK").ok().and_then(|v| v.parse::<u32>().ok()).unwrap_or(0x3FFu32 & !(1 << 4));
    // DIM (0.0–1.0) fades layers below the current one — set DIM=0 to isolate just
    // the top layer's beads (everything below renders black).
    let dim = std::env::var("DIM").ok().and_then(|v| v.parse::<f32>().ok()).unwrap_or(1.0);
    // The oracle bakes tool colors into each bead's rgb (COLOR=filament
    // path_colors), so it defaults to the baked-color path — faithful to the
    // GUI. CMODE=1 forces the Filament palette path (recolor extrusion from
    // tool_palette by tool id) — a diagnostic to confirm it matches the baked
    // path when the palette equals the baked colors.
    let mut tool_palette = [[0.0f32; 4]; crate::render::TOOL_PALETTE_LEN];
    for (i, slot) in tool_palette
        .iter_mut()
        .enumerate()
        .take(settings.tool_count.min(crate::render::TOOL_PALETTE_LEN))
    {
        let c = crate::render::visible_against_backdrop(settings.tool(i).color_rgb);
        *slot = [c[0], c[1], c[2], 1.0];
    }
    let color_mode = std::env::var("CMODE").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    let preview = crate::render::Preview {
        // The oracle renders a slice, never a live machine.
        marker: None,
        count,
        joint_count,
        current_layer: layer as f32,
        dim,
        mask,
        color_mode,
        tool_palette,
    };
    let (key, fill) = crate::render::camera_lights(eye, center);
    scene.render_to(&device, &queue, view_proj, eye, false, false, Some(preview), key, fill, [0.0; 4]);

    let (w, h, rgba) = scene.read_rgba(&device, &queue);
    write_png(&a.out, w, h, &rgba).map_err(|e| format!("write png: {e}"))?;
    eprintln!(
        "offscreen: layer {layer}/{} walls={}  {w}x{h} -> {}",
        layers.len(),
        a.walls,
        a.out.display()
    );
    // Multi-material: the per-slot filament split (the toolchanger oracle).
    if settings.tool_count > 1 {
        for (t, mm, g) in engine::estimate_filament_per_tool(&layers, &settings) {
            eprintln!("offscreen: T{t} {:.2} m ({g:.1} g)", mm / 1000.0);
        }
    }
    Ok(())
}

/// One TOOLS= entry: a plain slot index, or a blend literal `w:t+w:t…`
/// (weights are relative shares — `50:0+50:2` and `1:0+1:2` are the same mix).
fn parse_paint(tok: &str) -> Result<engine::PartPaint, String> {
    if !tok.contains(':') {
        return tok.parse::<u32>().map(engine::PartPaint::Tool).map_err(|_| format!("bad TOOLS entry {tok:?}"));
    }
    let mut weights = Vec::new();
    for term in tok.split('+') {
        let (w, t) = term.split_once(':').ok_or_else(|| format!("bad TOOLS blend term {term:?}"))?;
        weights.push((
            t.trim().parse::<u32>().map_err(|_| format!("bad blend tool {t:?}"))?,
            w.trim().parse::<f64>().map_err(|_| format!("bad blend weight {w:?}"))?,
        ));
    }
    Ok(engine::PartPaint::Blend(weights))
}

/// Slot colors for the headless oracle, where no profiles load: white → grey →
/// dark grey read instantly in a monochrome PNG; slots past 2 spread hues.
fn oracle_tool_color(i: usize) -> [f32; 3] {
    match i {
        0 => [0.82, 0.82, 0.82],
        1 => [0.55, 0.55, 0.55],
        2 => [0.28, 0.28, 0.28],
        n => crate::hsl_to_rgb(((n - 3) as f32 * 67.0) % 360.0, 0.55, 0.55),
    }
}

/// XY centre + bounding radius of all toolpath points through `up_to` layers.
fn bounds(layers: &[engine::LayerPlan], up_to: usize) -> (Vec3, f32) {
    let (mut xmn, mut ymn, mut xmx, mut ymx) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    let mut ztop = 0.0;
    for layer in layers.iter().take(up_to) {
        ztop = layer.print_z_mm;
        for path in &layer.paths {
            for p in &path.points {
                xmn = xmn.min(p.x_mm());
                xmx = xmx.max(p.x_mm());
                ymn = ymn.min(p.y_mm());
                ymx = ymx.max(p.y_mm());
            }
        }
    }
    if xmn > xmx {
        return (Vec3::new(0.0, 0.0, 0.0), 50.0);
    }
    let center = Vec3::new(((xmn + xmx) / 2.0) as f32, ((ymn + ymx) / 2.0) as f32, ztop as f32);
    let radius = (((xmx - xmn).max(ymx - ymn)) / 2.0).max(1.0) as f32;
    (center, radius)
}

// ---------------------------------------------------------------------------
// Minimal PNG encoder (RGBA8, stored/uncompressed deflate — no extra crate).
// ---------------------------------------------------------------------------

fn write_png(path: &std::path::Path, w: u32, h: u32, rgba: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let mut raw = Vec::with_capacity((w * h * 4 + h) as usize);
    let stride = (w * 4) as usize;
    for y in 0..h as usize {
        raw.push(0); // filter: none
        raw.extend_from_slice(&rgba[y * stride..(y + 1) * stride]);
    }
    let mut png = Vec::new();
    png.extend_from_slice(&[0x89, b'P', b'N', b'G', b'\r', b'\n', 0x1a, b'\n']);
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&w.to_be_bytes());
    ihdr.extend_from_slice(&h.to_be_bytes());
    ihdr.extend_from_slice(&[8, 6, 0, 0, 0]); // 8-bit, RGBA, deflate, no filter/interlace
    chunk(&mut png, b"IHDR", &ihdr);
    chunk(&mut png, b"IDAT", &zlib_stored(&raw));
    chunk(&mut png, b"IEND", &[]);
    std::fs::File::create(path)?.write_all(&png)
}

fn chunk(out: &mut Vec<u8>, tag: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(tag);
    out.extend_from_slice(data);
    let mut crc = crc32(tag);
    crc = crc32_update(crc, data);
    out.extend_from_slice(&(crc ^ 0xFFFF_FFFF).to_be_bytes());
}

/// zlib stream wrapping uncompressed (BTYPE=00) deflate blocks.
fn zlib_stored(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0x78, 0x01];
    let mut i = 0;
    while i < data.len() {
        let n = (data.len() - i).min(0xFFFF);
        let last = i + n >= data.len();
        out.push(if last { 1 } else { 0 });
        out.extend_from_slice(&(n as u16).to_le_bytes());
        out.extend_from_slice(&(!(n as u16)).to_le_bytes());
        out.extend_from_slice(&data[i..i + n]);
        i += n;
    }
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &x in data {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

fn crc32(tag: &[u8]) -> u32 {
    crc32_update(0xFFFF_FFFF, tag)
}

fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
        }
    }
    crc
}
