//! Headless offscreen render: slice an STL and render one preview layer to a
//! PNG through the *same* `Scene`/`render_to` path the GUI uses — no window, no
//! egui. This is the reliable visual oracle for wall-generation changes: a fix
//! is a single command, pixel-faithful to the GUI, with none of the xdotool
//! fragility of driving the live app.

use crate::{build_instances, render::Scene};
use eframe::wgpu;
use glam::{Mat4, Vec3};

pub struct Args {
    pub stl: std::path::PathBuf,
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
    let mesh = mesh::Mesh::load_stl(&a.stl).map_err(|e| format!("load {}: {e}", a.stl.display()))?;
    let mut settings = config::Settings::default();
    settings.wall_count = a.walls;
    // Debug env overrides so the offscreen render can reproduce GUI experiments.
    if let Some(v) = std::env::var("INFILL_DENSITY").ok().and_then(|s| s.parse::<f64>().ok()) {
        settings.infill_density = v;
    }
    if let Some(p) = std::env::var("INFILL_PATTERN").ok().and_then(|s| config::InfillPattern::parse(&s)) {
        settings.sparse_pattern = p;
        settings.solid_pattern = p;
    }
    if let Some(n) = std::env::var("TOP").ok().and_then(|s| s.parse().ok()) {
        settings.top_layers = n;
    }
    if let Some(n) = std::env::var("BOTTOM").ok().and_then(|s| s.parse().ok()) {
        settings.bottom_layers = n;
    }
    if std::env::var("NO_GAP").is_ok() {
        settings.gap_fill = false;
    }
    if std::env::var("SUPPORT").is_ok() {
        settings.support_mode = config::SupportMode::Grid;
    }
    let layers = engine::generate(&mesh, &settings);
    if layers.is_empty() {
        return Err("slice produced no layers".into());
    }
    let layer = a.layer.clamp(1, layers.len());

    // Same bead geometry the GUI builds.
    let accent = (190.0, 0.25, 0.55);
    let (inst, ends, joints, joint_ends) = build_instances(&layers, 0.0, None, accent, 0.0);
    let count = ends.get(layer - 1).copied().unwrap_or(0);
    let joint_count = joint_ends.get(layer - 1).copied().unwrap_or(0);

    // Scene (4× MSAA — within the WebGPU baseline, so no device feature needed).
    let format = wgpu::TextureFormat::Rgba8Unorm;
    let mut scene = Scene::new_core(&device, format, 4);
    scene.resize_core(&device, a.width, a.height);
    scene.set_toolpaths(&device, &inst);
    let joint_count = if std::env::var("NO_JOINTS").is_ok() { 0 } else { joint_count };
    if joint_count > 0 {
        scene.set_joints(&device, &joints);
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

    // Show every fill category, hide travels (bit 4). CAT_MASK (decimal) overrides
    // for diagnostics — e.g. CAT_MASK=2 = walls only (bit 1).
    let mask = std::env::var("CAT_MASK").ok().and_then(|v| v.parse::<u32>().ok()).unwrap_or(0x1FFu32 & !(1 << 4));
    let preview = crate::render::Preview {
        count,
        joint_count,
        current_layer: layer as f32,
        dim: 1.0,
        mask,
    };
    scene.render_to(&device, &queue, view_proj, false, Some(preview), [0.0; 3], [0.0; 3], [0.0; 4]);

    let (w, h, rgba) = scene.read_rgba(&device, &queue);
    write_png(&a.out, w, h, &rgba).map_err(|e| format!("write png: {e}"))?;
    eprintln!(
        "offscreen: layer {layer}/{} walls={}  {w}x{h} -> {}",
        layers.len(),
        a.walls,
        a.out.display()
    );
    Ok(())
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
