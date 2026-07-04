//! wgpu renderer for the 3D viewport.
//!
//! Scene (bed grid + model, or sliced toolpaths) is drawn into an offscreen
//! color+depth texture, handed to egui as a native texture. Our own pass gives a
//! depth buffer for correct 3D occlusion.
//!
//! Toolpaths are drawn as real **beads**: one unit box is instanced per extrusion
//! segment and oriented/scaled to the segment's direction, length, line width and
//! layer height in the vertex shader. Per-instance layer index + category drive
//! the layer slider, per-category visibility, and dimming of lower layers — all
//! in-shader, so scrubbing/toggling never rebuilds the buffer.

use std::borrow::Cow;

use bytemuck::{Pod, Zeroable};
use eframe::egui::TextureId;
use eframe::egui_wgpu::RenderState;
use eframe::wgpu;
use eframe::wgpu::util::DeviceExt;
use std::sync::atomic::{AtomicU32, Ordering};

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// The viewport clear color, in the sRGB values the pipeline displays.
/// Preview colors are contrast-checked against it (`visible_against_backdrop`),
/// so it lives here beside the clear that uses it.
pub const BACKDROP_RGB: [f32; 3] = [0.058, 0.048, 0.038];

/// A displayed color the shade of the backdrop would vanish into it — a
/// near-black spool on the near-black stage renders as a hole. Hold every
/// preview color to a minimum luminance gap from the backdrop, nudged toward
/// the pole the backdrop is far from (white, on this stage) just far enough
/// to clear the floor. Hue survives the nudge; colors already clear of the
/// floor pass through untouched. Display only — never the color that reaches
/// profiles, blends, or g-code.
pub fn visible_against_backdrop(c: [f32; 3]) -> [f32; 3] {
    // 0.18 rather than a bare-minimum gap: the bead shader multiplies
    // colors down to 0.40x on faces the light misses, and the layer
    // slider dims lower layers further — the floor must survive both.
    const FLOOR: f32 = 0.18;
    let luma = |c: [f32; 3]| 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
    let bl = luma(BACKDROP_RGB);
    let cl = luma(c);
    if (cl - bl).abs() >= FLOOR {
        return c;
    }
    let (pole, target) = if bl < 0.5 { (1.0, bl + FLOOR) } else { (0.0, bl - FLOOR) };
    let t = ((target - cl) / (pole - cl)).clamp(0.0, 1.0);
    [c[0] + t * (pole - c[0]), c[1] + t * (pole - c[1]), c[2] + t * (pole - c[2])]
}

const SHADER: &str = r#"
struct U {
    mvp: mat4x4<f32>,
    light: vec4<f32>,
    // x = current (top visible) layer, y = dim factor, z = category bitmask, w = unused
    ctrl: vec4<f32>,
    // Accent-derived model tints (rgb; w unused). The base (unselected) tint
    // now rides each mesh vertex (per-part colors); mesh_unsel stays in the
    // block only to keep the uniform layout, unread.
    mesh_unsel: vec4<f32>,
    mesh_sel: vec4<f32>,
    label_color: vec4<f32>,
    // xyz = world-space camera eye (stars ride an infinite sphere around it).
    cam_eye: vec4<f32>,
    // x,y = viewport pixels (round, aspect-correct star billboards).
    viewport: vec4<f32>,
};
@group(0) @binding(0) var<uniform> u: U;

// --- mesh (shaded) ---
struct MeshOut { @builtin(position) clip: vec4<f32>, @location(0) normal: vec3<f32>, @location(1) rgb: vec3<f32>, @location(2) @interpolate(flat) sel: f32, @location(3) @interpolate(flat) invalid: f32 };
@vertex fn vs_mesh(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>, @location(2) rgb: vec3<f32>, @location(3) sel: f32, @location(4) invalid: f32) -> MeshOut {
    var o: MeshOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.normal = n;
    o.rgb = rgb;
    o.sel = sel;
    o.invalid = invalid;
    return o;
}
@fragment fn fs_mesh(i: MeshOut) -> @location(0) vec4<f32> {
    let l = normalize(u.light.xyz);
    let d = max(dot(normalize(i.normal), l), 0.0);
    // Per-vertex base tint: the part's filament color on a toolchanger, or
    // the accent sunk into porcelain on a single tool (main.rs bakes either
    // into the buffer). Selected mixes toward the accent proper exactly as
    // before. An invalid object (outside the build volume, or overlapping
    // another) overrides both with terracotta (the theme's error color) —
    // and when that invalid object is also the selection it gets a brighter
    // coral, so you can tell which of two colliding parts is selected — so
    // the warning reads over any filament color. It can't print until fixed.
    var base = mix(i.rgb, u.mesh_sel.rgb, i.sel);
    let warn = mix(vec3<f32>(0.862, 0.420, 0.320), vec3<f32>(0.980, 0.670, 0.520), i.sel);
    base = mix(base, warn, i.invalid);
    return vec4<f32>(base * (0.35 + 0.65 * d), 1.0);
}

// --- plain lines (bed grid) ---
struct LineOut { @builtin(position) clip: vec4<f32>, @location(0) color: vec3<f32> };
@vertex fn vs_line(@location(0) p: vec3<f32>, @location(1) c: vec3<f32>) -> LineOut {
    var o: LineOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.color = c;
    return o;
}
@fragment fn fs_line(i: LineOut) -> @location(0) vec4<f32> {
    return vec4<f32>(i.color, 1.0);
}

// --- selection spotlight: a translucent gradient pool laid on z=0 that traces
// the selected object's footprint. Straight (non-premultiplied) RGBA per vertex;
// alpha fades from the silhouette outward. Depth is ignored (it paints over the
// grid) but it is drawn before the opaque mesh, so the object overdraws its core. ---
struct GlowOut { @builtin(position) clip: vec4<f32>, @location(0) rgba: vec4<f32> };
@vertex fn vs_glow(@location(0) p: vec3<f32>, @location(1) rgba: vec4<f32>) -> GlowOut {
    var o: GlowOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.rgba = rgba;
    return o;
}
@fragment fn fs_glow(i: GlowOut) -> @location(0) vec4<f32> {
    return i.rgba;
}

// --- toolpath beads (instanced boxes) ---
// base box vertex: lpos in (x:[0,1], y/z:[-0.5,0.5]); instance places/scales it.
struct BeadOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) @interpolate(flat) layer: f32,
    @location(3) @interpolate(flat) cat: f32,
};
@vertex fn vs_bead(
    @location(0) lpos: vec3<f32>,
    @location(1) lnorm: vec3<f32>,
    @location(2) p0: vec3<f32>,
    @location(3) dir_len: vec3<f32>,
    @location(4) dims: vec2<f32>,
    @location(5) color: vec3<f32>,
    @location(6) lc: vec2<f32>,
) -> BeadOut {
    let xaxis = vec3<f32>(dir_len.x, dir_len.y, 0.0); // along the segment (unit)
    let zaxis = vec3<f32>(0.0, 0.0, 1.0);
    let yaxis = cross(zaxis, xaxis);                  // across, in the bed plane
    let local = xaxis * (lpos.x * dir_len.z) + yaxis * (lpos.y * dims.x) + zaxis * (lpos.z * dims.y);
    var o: BeadOut;
    o.clip = u.mvp * vec4<f32>(p0 + local, 1.0);
    // Correct the normal for the non-uniform (width, height) scaling of the
    // cross-section (inverse scale), then rotate into the segment frame.
    let n_local = normalize(vec3<f32>(lnorm.x, lnorm.y / dims.x, lnorm.z / dims.y));
    o.normal = xaxis * n_local.x + yaxis * n_local.y + zaxis * n_local.z;
    o.color = color;
    o.layer = lc.x;
    o.cat = lc.y;
    return o;
}
@fragment fn fs_bead(i: BeadOut) -> @location(0) vec4<f32> {
    let mask = u32(u.ctrl.z + 0.5);
    let cat = u32(i.cat + 0.5);
    if ((mask & (1u << cat)) == 0u) { discard; }
    let l = normalize(u.light.xyz);
    let d = max(dot(normalize(i.normal), l), 0.0);
    var shade = 0.40 + 0.60 * d;
    if (i.layer < u.ctrl.x - 0.5) { shade = shade * u.ctrl.y; } // dim lower layers
    return vec4<f32>(i.color * shade, 1.0);
}

// --- joint blobs (instanced; round path ends and fill corners) ---
@vertex fn vs_joint(
    @location(0) lpos: vec3<f32>,
    @location(1) lnorm: vec3<f32>,
    @location(2) p0: vec3<f32>,
    @location(3) dims: vec2<f32>,
    @location(4) color: vec3<f32>,
    @location(5) lc: vec2<f32>,
) -> BeadOut {
    let r = vec3<f32>(dims.x * 0.5, dims.x * 0.5, dims.y * 0.5);
    var o: BeadOut;
    o.clip = u.mvp * vec4<f32>(p0 + lpos * r, 1.0);
    o.normal = normalize(lnorm / r);
    o.color = color;
    o.layer = lc.x;
    o.cat = lc.y;
    return o;
}

// --- bed label ("Front"): textured glyph quads laid on z=0, depth-tested so
// the model occludes them per-pixel; the atlas is egui's font coverage (R8). ---
@group(1) @binding(0) var lbl_tex: texture_2d<f32>;
@group(1) @binding(1) var lbl_samp: sampler;
struct LabelOut { @builtin(position) clip: vec4<f32>, @location(0) uv: vec2<f32> };
@vertex fn vs_label(@location(0) p: vec3<f32>, @location(1) uv: vec2<f32>) -> LabelOut {
    var o: LabelOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.uv = uv;
    return o;
}
@fragment fn fs_label(i: LabelOut) -> @location(0) vec4<f32> {
    let cov = textureSample(lbl_tex, lbl_samp, i.uv).r;
    return vec4<f32>(u.label_color.rgb, u.label_color.a * cov);
}

// --- night sky: real bright-star directions billboarded onto an infinite
// celestial sphere centered on the eye, so orbiting the model sweeps across the
// actual sky. Instance = (dir, apparent magnitude); base quad = corner in
// [-1,1]^2. Drawn first with depth OFF (the backdrop), alpha over the ink. ---
struct StarOut { @builtin(position) clip: vec4<f32>, @location(0) offset: vec2<f32>, @location(1) bright: f32 };
@vertex fn vs_star(@location(0) corner: vec2<f32>, @location(1) dir: vec3<f32>, @location(2) mag: f32) -> StarOut {
    // Any point on the ray from the eye projects to the same screen direction,
    // so distance is arbitrary; 1000 keeps it inside the [near, far] frustum.
    let world = u.cam_eye.xyz + dir * 1000.0;
    var clip = u.mvp * vec4<f32>(world, 1.0);
    // Apparent magnitude → brightness (lower mag = brighter; Sirius ≈ -1.5).
    let bright = clamp(1.28 - mag * 0.15, 0.08, 1.10);
    // Radius in pixels: brighter stars a touch larger, a floor so faint ones
    // still register. Offset the corner in clip space (÷w cancels perspective).
    let px = mix(0.95, 3.3, clamp(bright, 0.0, 1.0));
    clip.x = clip.x + corner.x * (px * 2.0 / u.viewport.x) * clip.w;
    clip.y = clip.y + corner.y * (px * 2.0 / u.viewport.y) * clip.w;
    var o: StarOut;
    o.clip = clip;
    o.offset = corner;
    o.bright = bright;
    return o;
}
@fragment fn fs_star(i: StarOut) -> @location(0) vec4<f32> {
    let r = length(i.offset);
    if (r > 1.0) { discard; }
    // Round falloff (fuller than a squared one, so the core reads); a faintly
    // warm-white star. Bright stars clamp to a solid white core.
    let a = clamp((1.0 - r * r) * i.bright, 0.0, 1.0);
    return vec4<f32>(vec3<f32>(0.99, 0.98, 0.94) * a, a);
}
"#;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    pos: [f32; 3],
    normal: [f32; 3],
}

/// Mesh vertex with its base tint and state flags: `rgb` = the part's color
/// (filament on a toolchanger, accent porcelain otherwise); `sel` 1 = selected
/// highlight; `invalid` 1 = can't be printed (outside the build volume or
/// overlapping another object) — drawn with the warning tint.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MeshVertex {
    pos: [f32; 3],
    normal: [f32; 3],
    rgb: [f32; 3],
    sel: f32,
    invalid: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 3],
}

/// A selection-spotlight vertex: a point on the bed plane and straight RGBA.
/// Alpha carries the pool's falloff — opaque-ish at the selected object's
/// footprint, fading to 0 at the outer rim.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GlowVertex {
    pos: [f32; 3],
    rgba: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],
    light: [f32; 4],
    ctrl: [f32; 4],
    mesh_unsel: [f32; 4],
    mesh_sel: [f32; 4],
    label_color: [f32; 4],
    cam_eye: [f32; 4],
    viewport: [f32; 4],
}

/// How to draw the toolpaths this frame.
pub struct Preview {
    /// Number of bead (segment) instances to draw, through the current layer.
    pub count: u32,
    /// Number of joint-blob instances to draw, through the current layer.
    pub joint_count: u32,
    /// Current (top visible) layer, 1-based.
    pub current_layer: f32,
    /// Brightness multiplier for layers below the current one (1.0 = no dim).
    pub dim: f32,
    /// Category visibility bitmask (bit per category id).
    pub mask: u32,
}

pub struct Scene {
    format: wgpu::TextureFormat,
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bead_pipeline: wgpu::RenderPipeline,
    uniform_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    size: (u32, u32),
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    /// The single-sample resolve texture (kept for headless read-back).
    resolve_tex: wgpu::Texture,
    /// egui texture handle — `None` in the headless (offscreen) path.
    tex_id: Option<TextureId>,
    mesh_vbuf: Option<wgpu::Buffer>,
    mesh_count: u32,
    line_vbuf: Option<wgpu::Buffer>,
    line_count: u32,
    glow_pipeline: wgpu::RenderPipeline,
    glow_vbuf: Option<wgpu::Buffer>,
    glow_count: u32,
    // Night-sky backdrop: a fixed catalog of star billboards (base quad +
    // per-star instance = direction + magnitude), drawn first with depth off.
    star_pipeline: wgpu::RenderPipeline,
    star_quad_vbuf: wgpu::Buffer,
    star_inst_vbuf: wgpu::Buffer,
    star_count: u32,
    /// Opaque backdrop-colored bed fill (glow pipeline), so stars don't show
    /// through the grid. Built by `set_beds`.
    bed_fill_vbuf: Option<wgpu::Buffer>,
    bed_fill_count: u32,
    box_vbuf: wgpu::Buffer,
    box_count: u32,
    inst_vbuf: Option<wgpu::Buffer>,
    inst_count: u32,
    joint_pipeline: wgpu::RenderPipeline,
    blob_vbuf: wgpu::Buffer,
    blob_count: u32,
    joint_vbuf: Option<wgpu::Buffer>,
    joint_count: u32,
    label_pipeline: wgpu::RenderPipeline,
    label_bgl: wgpu::BindGroupLayout,
    label_sampler: wgpu::Sampler,
    label_bind_group: Option<wgpu::BindGroup>,
    label_vbuf: Option<wgpu::Buffer>,
    label_count: u32,
}

impl Scene {
    pub fn new(rs: &RenderState) -> Self {
        let samples = pick_samples(rs, rs.target_format);
        let mut scene = Self::new_core(&rs.device, rs.target_format, samples);
        scene.tex_id = Some(rs.renderer.write().register_native_texture(
            &rs.device,
            &scene.resolve_view,
            wgpu::FilterMode::Linear,
        ));
        scene
    }

    /// GPU-only construction (no egui texture registration) — shared by the
    /// interactive GUI (`new`) and the headless offscreen renderer.
    pub fn new_core(device: &wgpu::Device, format: wgpu::TextureFormat, samples: u32) -> Self {
        SAMPLES.store(samples, Ordering::Relaxed);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(SHADER)),
        });

        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scene_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let uniform_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scene_uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scene_bg"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buf.as_entire_binding(),
            }],
        });

        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene_layout"),
            bind_group_layouts: &[Some(&bgl)],
            immediate_size: 0,
        });

        let mesh_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_mesh", "fs_mesh",
            &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<MeshVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32, 4 => Float32],
            }],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
        );
        let line_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_line", "fs_line",
            &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<LineVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
            }],
            wgpu::PrimitiveTopology::LineList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
        );
        let bead_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_bead", "fs_bead",
            &[
                wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                },
                wgpu::VertexBufferLayout {
                    array_stride: (13 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x3, 3 => Float32x3, 4 => Float32x2, 5 => Float32x3, 6 => Float32x2],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
        );
        let joint_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_joint", "fs_bead",
            &[
                wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                },
                wgpu::VertexBufferLayout {
                    array_stride: (10 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x3, 3 => Float32x2, 4 => Float32x3, 5 => Float32x2],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
        );

        // Bed-label pass: group(1) = its R8 font-atlas texture + sampler; alpha
        // blended and depth-tested (no depth write — it's a flat decal on z=0).
        let label_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("label_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let label_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("label_sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let label_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("label_layout"),
            bind_group_layouts: &[Some(&bgl), Some(&label_bgl)],
            immediate_size: 0,
        });
        let label_pipeline = make_pipeline(
            device, &label_layout, &shader, format, "vs_label", "fs_label",
            &[wgpu::VertexBufferLayout {
                // LabelVertex = [pos.xyz, uv.xy]
                array_stride: (5 * 4) as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x2],
            }],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState {
                color: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                alpha: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
            },
            false,
            wgpu::CompareFunction::Less,
        );

        // Selection spotlight: alpha-blended like the label, but keyed off the
        // group(0) uniforms only (no texture), with depth reads OFF so the pool
        // paints over the bed grid. Drawn before the opaque mesh, which
        // overdraws the object's own footprint.
        let glow_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_glow", "fs_glow",
            &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<GlowVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
            }],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState {
                color: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                alpha: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
            },
            false,
            wgpu::CompareFunction::Always,
        );

        // Night-sky stars: instanced billboards, alpha-over like the glow, depth
        // off (the backdrop). Base quad at @location 0; per-star instance =
        // direction (loc 1) + apparent magnitude (loc 2).
        let star_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_star", "fs_star",
            &[
                wgpu::VertexBufferLayout {
                    array_stride: (2 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x2],
                },
                wgpu::VertexBufferLayout {
                    array_stride: (4 * 4) as u64, // dir.xyz + magnitude
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![1 => Float32x3, 2 => Float32],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState {
                color: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::SrcAlpha,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
                alpha: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
                    operation: wgpu::BlendOperation::Add,
                },
            },
            false,
            wgpu::CompareFunction::Always,
        );
        let star_quad: [[f32; 2]; 6] =
            [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let star_quad_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("star_quad"),
            contents: bytemuck::cast_slice(&star_quad[..]),
            usage: wgpu::BufferUsages::VERTEX,
        });
        // Real Yale Bright Star Catalogue: [dir.x, dir.y, dir.z, magnitude] f32
        // per star (unit directions on the celestial sphere), packed offline.
        const STAR_DATA: &[u8] = include_bytes!("stars.bin");
        let star_inst_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("star_inst"),
            contents: STAR_DATA,
            usage: wgpu::BufferUsages::VERTEX,
        });
        let star_count = (STAR_DATA.len() / (4 * 4)) as u32;

        let box_verts = bead_vertices();
        let box_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bead_base"),
            contents: bytemuck::cast_slice(&box_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let blob_verts = blob_vertices();
        let blob_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("joint_base"),
            contents: bytemuck::cast_slice(&blob_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let (color_view, depth_view, resolve_view, resolve_tex) = make_targets(device, format, 1, 1);

        Self {
            format,
            mesh_pipeline,
            line_pipeline,
            bead_pipeline,
            uniform_buf,
            bind_group,
            size: (1, 1),
            color_view,
            depth_view,
            resolve_view,
            resolve_tex,
            tex_id: None,
            mesh_vbuf: None,
            mesh_count: 0,
            line_vbuf: None,
            line_count: 0,
            glow_pipeline,
            glow_vbuf: None,
            glow_count: 0,
            star_pipeline,
            star_quad_vbuf,
            star_inst_vbuf,
            star_count,
            bed_fill_vbuf: None,
            bed_fill_count: 0,
            box_vbuf,
            box_count: box_verts.len() as u32,
            inst_vbuf: None,
            inst_count: 0,
            joint_pipeline,
            blob_vbuf,
            blob_count: blob_verts.len() as u32,
            joint_vbuf: None,
            joint_count: 0,
            label_pipeline,
            label_bgl,
            label_sampler,
            label_bind_group: None,
            label_vbuf: None,
            label_count: 0,
        }
    }

    pub fn texture_id(&self) -> TextureId {
        self.tex_id.expect("texture_id() on a headless Scene")
    }

    pub fn resize(&mut self, rs: &RenderState, w: u32, h: u32) {
        if self.resize_core(&rs.device, w, h) {
            if let Some(id) = self.tex_id {
                rs.renderer.write().update_egui_texture_from_wgpu_texture(
                    &rs.device,
                    &self.resolve_view,
                    wgpu::FilterMode::Linear,
                    id,
                );
            }
        }
    }

    /// Resize the GPU render targets; returns true if they changed.
    pub fn resize_core(&mut self, device: &wgpu::Device, w: u32, h: u32) -> bool {
        let (w, h) = (w.max(1), h.max(1));
        if self.size == (w, h) {
            return false;
        }
        let (color_view, depth_view, resolve_view, resolve_tex) = make_targets(device, self.format, w, h);
        self.color_view = color_view;
        self.depth_view = depth_view;
        self.resolve_view = resolve_view;
        self.resolve_tex = resolve_tex;
        self.size = (w, h);
        true
    }

    pub fn clear_mesh(&mut self) {
        self.mesh_vbuf = None;
        self.mesh_count = 0;
    }

    /// Upload all scene parts (each: mesh, placement, base tint, selected?,
    /// invalid?) as one buffer, baking the transform into bed coordinates and
    /// the tint/flags into each vertex. Returns the combined bounding box
    /// (min, max), or None if empty.
    pub fn set_mesh(
        &mut self,
        device: &wgpu::Device,
        objects: &[(&mesh::Mesh, mesh::Transform, [f32; 3], bool, bool)],
    ) -> Option<([f32; 3], [f32; 3])> {
        let mut verts: Vec<MeshVertex> = Vec::new();
        let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
        for (mesh, t, rgb, selected, invalid) in objects {
            let sel = if *selected { 1.0 } else { 0.0 };
            let invalid = if *invalid { 1.0 } else { 0.0 };
            for i in 0..mesh.triangles.len() {
                let tri = mesh.triangle(i);
                let f3 = |v: [f64; 3]| [v[0] as f32, v[1] as f32, v[2] as f32];
                let p: [[f32; 3]; 3] = [f3(t.apply(tri[0])), f3(t.apply(tri[1])), f3(t.apply(tri[2]))];
                let n = flat_normal(p[0], p[1], p[2]);
                let c = *rgb;
                for pos in p {
                    for k in 0..3 {
                        lo[k] = lo[k].min(pos[k]);
                        hi[k] = hi[k].max(pos[k]);
                    }
                    verts.push(MeshVertex { pos, normal: n, rgb: c, sel, invalid });
                }
            }
        }
        if verts.is_empty() {
            self.clear_mesh();
            return None;
        }
        self.mesh_count = verts.len() as u32;
        self.mesh_vbuf = make_vbuf(device, "mesh_vbuf", bytemuck::cast_slice(&verts));
        Some((lo, hi))
    }

    /// Build the bed grids: `n` beds in a row along +X, `gap` apart. The
    /// active bed gets the cream border and full-strength grid; the others
    /// recede into the ink.
    pub fn set_beds(
        &mut self,
        device: &wgpu::Device,
        bed_x: f32,
        bed_y: f32,
        n: usize,
        gap: f32,
        active: usize,
    ) {
        let step = 20.0_f32;
        let mut v = Vec::new();
        // A backdrop-colored fill under each bed (z=0), drawn between the stars
        // and the grid so the sky doesn't show THROUGH the plate — stars sit
        // only in the surrounding sky, not in the grid cells.
        let mut fill: Vec<GlowVertex> = Vec::new();
        let bg = [BACKDROP_RGB[0], BACKDROP_RGB[1], BACKDROP_RGB[2], 1.0];
        for k in 0..n.max(1) {
            let ox = k as f32 * (bed_x + gap);
            let q = [[ox, 0.0], [ox + bed_x, 0.0], [ox + bed_x, bed_y], [ox, bed_y]];
            for &idx in &[0usize, 1, 2, 0, 2, 3] {
                fill.push(GlowVertex { pos: [q[idx][0], q[idx][1], 0.0], rgba: bg });
            }
            let (grid, border) = if k == active {
                ([0.28, 0.25, 0.20], [0.64, 0.60, 0.51]) // warm ink + cream
            } else {
                ([0.14, 0.125, 0.10], [0.34, 0.31, 0.26]) // receded
            };
            // INTERIOR grid lines only — the border below owns the four edges.
            // Starting at 0 (or landing on bed_x/bed_y when they're a multiple of
            // step) would double a grid line onto the border, so those edges read
            // thicker than the others.
            let mut x = step;
            while x < bed_x - 0.01 {
                v.push(LineVertex { pos: [ox + x, 0.0, 0.0], color: grid });
                v.push(LineVertex { pos: [ox + x, bed_y, 0.0], color: grid });
                x += step;
            }
            let mut y = step;
            while y < bed_y - 0.01 {
                v.push(LineVertex { pos: [ox, y, 0.0], color: grid });
                v.push(LineVertex { pos: [ox + bed_x, y, 0.0], color: grid });
                y += step;
            }
            let corners = [[ox, 0.0], [ox + bed_x, 0.0], [ox + bed_x, bed_y], [ox, bed_y]];
            for c in 0..4 {
                let a = corners[c];
                let b = corners[(c + 1) % 4];
                v.push(LineVertex { pos: [a[0], a[1], 0.0], color: border });
                v.push(LineVertex { pos: [b[0], b[1], 0.0], color: border });
            }
        }
        self.line_count = v.len() as u32;
        self.line_vbuf = make_vbuf(device, "bed_vbuf", bytemuck::cast_slice(&v));
        self.bed_fill_count = fill.len() as u32;
        self.bed_fill_vbuf = make_vbuf(device, "bed_fill", bytemuck::cast_slice(&fill));
    }

    /// Upload the selection spotlight: a triangle soup on the bed plane where
    /// each vertex is `[x, y, z, r, g, b, a]`. An empty slice clears it.
    pub fn set_spotlight(&mut self, device: &wgpu::Device, verts: &[[f32; 7]]) {
        if verts.is_empty() {
            self.glow_vbuf = None;
            self.glow_count = 0;
            return;
        }
        let gv: Vec<GlowVertex> = verts
            .iter()
            .map(|v| GlowVertex { pos: [v[0], v[1], v[2]], rgba: [v[3], v[4], v[5], v[6]] })
            .collect();
        self.glow_count = gv.len() as u32;
        self.glow_vbuf = make_vbuf(device, "glow_vbuf", bytemuck::cast_slice(&gv));
    }

    /// Upload bead instances: `[p0.xyz, dir.xy, len, width, height, r, g, b, layer, cat]`.
    pub fn set_toolpaths(&mut self, device: &wgpu::Device, instances: &[[f32; 13]]) {
        self.inst_count = instances.len() as u32;
        self.inst_vbuf = make_vbuf(device, "bead_instances", bytemuck::cast_slice(instances));
    }

    /// Upload joint-blob instances: `[p0.xyz, width, height, r, g, b, layer, cat]`.
    pub fn set_joints(&mut self, device: &wgpu::Device, joints: &[[f32; 10]]) {
        self.joint_count = joints.len() as u32;
        self.joint_vbuf = make_vbuf(device, "joint_instances", bytemuck::cast_slice(joints));
    }

    /// Upload the font-atlas coverage (R8) the bed label samples, and (re)build
    /// its bind group. Called once — the "Front" glyphs never change.
    pub fn set_label_atlas(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        w: u32,
        h: u32,
        coverage: &[u8],
    ) {
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("label_atlas"),
            size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            coverage,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(w), rows_per_image: Some(h) },
            wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        );
        let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        self.label_bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("label_bg"),
            layout: &self.label_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&self.label_sampler) },
            ],
        }));
    }

    /// Upload the label's world-space glyph triangles (rebuilt when the bed changes).
    pub fn set_label_geom(&mut self, device: &wgpu::Device, verts: &[[f32; 5]]) {
        self.label_count = verts.len() as u32;
        self.label_vbuf = make_vbuf(device, "label", bytemuck::cast_slice(verts));
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &self,
        rs: &RenderState,
        view_proj: glam::Mat4,
        cam_eye: glam::Vec3,
        show_stars: bool,
        show_mesh: bool,
        preview: Option<Preview>,
        mesh_unsel: [f32; 3],
        mesh_sel: [f32; 3],
        label_color: [f32; 4],
    ) {
        self.render_to(
            &rs.device, &rs.queue, view_proj, cam_eye, show_stars, show_mesh, preview, mesh_unsel,
            mesh_sel, label_color,
        );
    }

    /// Device/queue render path — used directly by the headless offscreen
    /// renderer; the GUI's `render` wraps it with the egui `RenderState`.
    #[allow(clippy::too_many_arguments)]
    pub fn render_to(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        view_proj: glam::Mat4,
        cam_eye: glam::Vec3,
        show_stars: bool,
        show_mesh: bool,
        preview: Option<Preview>,
        mesh_unsel: [f32; 3],
        mesh_sel: [f32; 3],
        label_color: [f32; 4],
    ) {
        let ctrl = match &preview {
            Some(p) => [p.current_layer, p.dim, p.mask as f32, 0.0],
            None => [0.0, 1.0, 0.0, 0.0],
        };
        let uniforms = Uniforms {
            mvp: view_proj.to_cols_array_2d(),
            light: [0.4, 0.5, 0.85, 0.0],
            ctrl,
            mesh_unsel: [mesh_unsel[0], mesh_unsel[1], mesh_unsel[2], 0.0],
            mesh_sel: [mesh_sel[0], mesh_sel[1], mesh_sel[2], 0.0],
            label_color,
            cam_eye: [cam_eye.x, cam_eye.y, cam_eye.z, 0.0],
            viewport: [self.size.0 as f32, self.size.1 as f32, 0.0, 0.0],
        };
        queue.write_buffer(&self.uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("scene_encoder") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("scene_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.color_view,
                    depth_slice: None,
                    resolve_target: Some(&self.resolve_view),
                    ops: wgpu::Operations {
                        // The viewport stage: ink a step deeper than the
                        // panels, so the chrome floats on it.
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: BACKDROP_RGB[0] as f64,
                            g: BACKDROP_RGB[1] as f64,
                            b: BACKDROP_RGB[2] as f64,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.bind_group, &[]);

            // Night sky FIRST — the backdrop, drawn with depth off so the bed
            // grid and everything else compose over it.
            if show_stars && self.star_count > 0 {
                pass.set_pipeline(&self.star_pipeline);
                pass.set_vertex_buffer(0, self.star_quad_vbuf.slice(..));
                pass.set_vertex_buffer(1, self.star_inst_vbuf.slice(..));
                pass.draw(0..6, 0..self.star_count);
                // Then paint the plate's footprint back to backdrop (opaque, over
                // the stars via the glow pipeline) so the sky shows only AROUND
                // the plate — never through the grid cells.
                if let Some(buf) = &self.bed_fill_vbuf {
                    if self.bed_fill_count > 0 {
                        pass.set_pipeline(&self.glow_pipeline);
                        pass.set_vertex_buffer(0, buf.slice(..));
                        pass.draw(0..self.bed_fill_count, 0..1);
                    }
                }
            }

            if let Some(buf) = &self.line_vbuf {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.line_count, 0..1);
            }

            // The selection spotlight: right after the grid so the opaque model
            // (drawn below) overdraws its core and the glow spills out around
            // the footprint. Model view only.
            if show_mesh && self.glow_count > 0 {
                if let Some(buf) = &self.glow_vbuf {
                    pass.set_pipeline(&self.glow_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.glow_count, 0..1);
                }
            }

            if let Some(p) = &preview {
                let n = p.count.min(self.inst_count);
                if n > 0 {
                    if let Some(inst) = &self.inst_vbuf {
                        pass.set_pipeline(&self.bead_pipeline);
                        pass.set_vertex_buffer(0, self.box_vbuf.slice(..));
                        pass.set_vertex_buffer(1, inst.slice(..));
                        pass.draw(0..self.box_count, 0..n);
                    }
                }
                let jn = p.joint_count.min(self.joint_count);
                if jn > 0 {
                    if let Some(jinst) = &self.joint_vbuf {
                        pass.set_pipeline(&self.joint_pipeline);
                        pass.set_vertex_buffer(0, self.blob_vbuf.slice(..));
                        pass.set_vertex_buffer(1, jinst.slice(..));
                        pass.draw(0..self.blob_count, 0..jn);
                    }
                }
            }

            if show_mesh {
                if let Some(buf) = &self.mesh_vbuf {
                    pass.set_pipeline(&self.mesh_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.mesh_count, 0..1);
                }
            }

            // The bed label LAST, so it depth-tests against the model/beads and
            // is hidden per-pixel where they stand in front of it.
            if self.label_count > 0 {
                if let (Some(bg), Some(vbuf)) = (&self.label_bind_group, &self.label_vbuf) {
                    pass.set_pipeline(&self.label_pipeline);
                    pass.set_bind_group(0, &self.bind_group, &[]);
                    pass.set_bind_group(1, bg, &[]);
                    pass.set_vertex_buffer(0, vbuf.slice(..));
                    pass.draw(0..self.label_count, 0..1);
                }
            }
        }
        queue.submit(std::iter::once(encoder.finish()));
    }

    /// Read the resolved color target back to tightly-packed RGBA8 (headless).
    pub fn read_rgba(&self, device: &wgpu::Device, queue: &wgpu::Queue) -> (u32, u32, Vec<u8>) {
        let (w, h) = self.size;
        // Copy rows are padded to 256 bytes per wgpu's COPY_BYTES_PER_ROW_ALIGNMENT.
        let unpadded = w * 4;
        let padded = unpadded.div_ceil(256) * 256;
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: (padded * h) as u64,
            usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut enc = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &self.resolve_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buf,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(padded),
                    rows_per_image: Some(h),
                },
            },
            wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        );
        queue.submit(std::iter::once(enc.finish()));
        let slice = buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::PollType::wait_indefinitely()).ok();
        let data = slice.get_mapped_range();
        let mut out = Vec::with_capacity((unpadded * h) as usize);
        for row in 0..h {
            let start = (row * padded) as usize;
            out.extend_from_slice(&data[start..start + unpadded as usize]);
        }
        let swizzle = matches!(self.format, wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb);
        if swizzle {
            for px in out.chunks_mut(4) {
                px.swap(0, 2);
            }
        }
        (w, h, out)
    }
}

/// Unit bead: an open tube along +X (`x` in [0,1]) with a rounded cross-section
/// (unit circle radius 0.5 in the Y-Z plane). The instance scales the
/// cross-section to (line width, layer height). Ends are left open; a joint blob
/// at every vertex rounds the ends and fills corners between segments.
fn bead_vertices() -> Vec<Vertex> {
    const N: usize = 8;
    let ring: Vec<[f32; 2]> = (0..N)
        .map(|k| {
            let t = std::f32::consts::TAU * (k as f32) / (N as f32);
            [0.5 * t.cos(), 0.5 * t.sin()]
        })
        .collect();

    let mut v = Vec::with_capacity(6 * N);
    for k in 0..N {
        let k1 = (k + 1) % N;
        let (y0, z0) = (ring[k][0], ring[k][1]);
        let (y1, z1) = (ring[k1][0], ring[k1][1]);
        let n0 = [0.0, y0 * 2.0, z0 * 2.0]; // (cos, sin) — unit radial
        let n1 = [0.0, y1 * 2.0, z1 * 2.0];
        let a = Vertex { pos: [0.0, y0, z0], normal: n0 };
        let b = Vertex { pos: [0.0, y1, z1], normal: n1 };
        let c = Vertex { pos: [1.0, y1, z1], normal: n1 };
        let d = Vertex { pos: [1.0, y0, z0], normal: n0 };
        v.extend_from_slice(&[a, b, c, a, c, d]);
    }
    v
}

/// Unit joint blob: an octagonal bipyramid (unit equator, poles at z = ±1).
/// The instance scales it to (width/2, width/2, height/2) and places it at a
/// path vertex, rounding ends and filling corners. Vertex positions are unit
/// vectors, so they double as normals.
fn blob_vertices() -> Vec<Vertex> {
    const S: usize = 8;
    let eq: Vec<[f32; 3]> = (0..S)
        .map(|k| {
            let t = std::f32::consts::TAU * (k as f32) / (S as f32);
            [t.cos(), t.sin(), 0.0]
        })
        .collect();
    let top = [0.0, 0.0, 1.0];
    let bot = [0.0, 0.0, -1.0];
    let mut v = Vec::with_capacity(6 * S);
    for k in 0..S {
        let k1 = (k + 1) % S;
        v.push(Vertex { pos: top, normal: top });
        v.push(Vertex { pos: eq[k], normal: eq[k] });
        v.push(Vertex { pos: eq[k1], normal: eq[k1] });
        v.push(Vertex { pos: bot, normal: bot });
        v.push(Vertex { pos: eq[k1], normal: eq[k1] });
        v.push(Vertex { pos: eq[k], normal: eq[k] });
    }
    v
}

fn make_vbuf(device: &wgpu::Device, label: &str, data: &[u8]) -> Option<wgpu::Buffer> {
    if data.is_empty() {
        return None;
    }
    Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: data,
        usage: wgpu::BufferUsages::VERTEX,
    }))
}

fn flat_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// MSAA sample count for the offscreen scene — smooths the dense thin beads,
/// which otherwise alias into a moiré / screen-door pattern. Resolved once at
/// startup (`pick_samples`) to the most the device supports, capped at 8×:
/// counts above the WebGPU-guaranteed 4× aren't universal (software backends
/// cap at 4), so requesting 8× unconditionally panics on those.
static SAMPLES: AtomicU32 = AtomicU32::new(4);

/// Highest MSAA count in {8, 4, 2, 1} the device supports for both the color
/// and depth attachments.
fn pick_samples(rs: &RenderState, color: wgpu::TextureFormat) -> u32 {
    // A device only accepts sample counts above the WebGPU-guaranteed 4× when it
    // was created with TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES (requested in
    // main.rs). The adapter's format flags describe the *hardware*, not what this
    // device will allow — so cap at 4× unless the device actually has the
    // feature, otherwise an 8× target panics even on a GPU that can do it.
    let cap = if rs
        .device
        .features()
        .contains(wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES)
    {
        8
    } else {
        4
    };
    let cf = rs.adapter.get_texture_format_features(color).flags;
    let df = rs.adapter.get_texture_format_features(DEPTH_FORMAT).flags;
    [8, 4, 2, 1]
        .into_iter()
        .filter(|&s| s <= cap)
        .find(|&s| cf.sample_count_supported(s) && df.sample_count_supported(s))
        .unwrap_or(1)
}

fn make_targets(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    w: u32,
    h: u32,
) -> (wgpu::TextureView, wgpu::TextureView, wgpu::TextureView, wgpu::Texture) {
    let size = wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 };
    // Multisampled color + depth are the render targets; the pass resolves into
    // `resolve` (single-sample), which is the texture egui samples.
    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("scene_color_msaa"),
        size,
        mip_level_count: 1,
        sample_count: SAMPLES.load(Ordering::Relaxed),
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("scene_depth"),
        size,
        mip_level_count: 1,
        sample_count: SAMPLES.load(Ordering::Relaxed),
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let resolve = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("scene_resolve"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let resolve_view = resolve.create_view(&wgpu::TextureViewDescriptor::default());
    (
        color.create_view(&wgpu::TextureViewDescriptor::default()),
        depth.create_view(&wgpu::TextureViewDescriptor::default()),
        resolve_view,
        resolve,
    )
}

fn make_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    format: wgpu::TextureFormat,
    vs: &str,
    fs: &str,
    buffers: &[wgpu::VertexBufferLayout],
    topology: wgpu::PrimitiveTopology,
    blend: wgpu::BlendState,
    depth_write: bool,
    depth_compare: wgpu::CompareFunction,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("scene_pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs),
            buffers,
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fs),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(blend),
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        }),
        primitive: wgpu::PrimitiveState {
            topology,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None,
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(depth_write),
            depth_compare: Some(depth_compare),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: SAMPLES.load(Ordering::Relaxed),
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview_mask: None,
        cache: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn near_backdrop_colors_lift_to_the_luma_floor() {
        let luma = |c: [f32; 3]| 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
        let bl = luma(BACKDROP_RGB);
        // The backdrop's own color, pure black, and a #1A1A1A spool all
        // clear the floor after the nudge.
        for c in [BACKDROP_RGB, [0.0; 3], [0.102; 3]] {
            let v = visible_against_backdrop(c);
            assert!(
                (luma(v) - bl).abs() >= 0.179,
                "{c:?} -> {v:?} still hides against the backdrop"
            );
        }
        // A dark red stays red-dominant — the nudge moves shade, not hue.
        let v = visible_against_backdrop([0.13, 0.02, 0.02]);
        assert!(v[0] > v[1] && v[0] > v[2], "hue must survive: {v:?}");
        assert!((luma(v) - bl).abs() >= 0.179);
        // Colors already clear of the floor pass through byte-identical.
        for c in [[1.0, 1.0, 1.0], [0.82, 0.82, 0.82], [0.28, 0.28, 0.28], [0.9, 0.1, 0.1]] {
            assert_eq!(visible_against_backdrop(c), c);
        }
    }
}
