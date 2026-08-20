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
    // xyz = KEY light direction (camera-relative, over the shoulder).
    light: vec4<f32>,
    // x = current (top visible) layer, y = dim factor, z = category bitmask,
    // w = bead color mode (0 = use the baked per-instance rgb; 1 = Filament:
    // recolor extrusion beads from tool_palette[tool], leaving travel/seam on
    // their baked color). Lets a spool-color change be a uniform write instead
    // of a full instance rebuild.
    ctrl: vec4<f32>,
    // xyz = FILL light direction (camera-relative, from the opposite side).
    // (Was an accent tint slot; the base tint now rides each mesh vertex.)
    mesh_unsel: vec4<f32>,
    mesh_sel: vec4<f32>, // unused
    label_color: vec4<f32>,
    // xyz = world-space camera eye (stars ride an infinite sphere around it).
    cam_eye: vec4<f32>,
    // x,y = viewport pixels (round, aspect-correct star billboards).
    viewport: vec4<f32>,
    // Per-tool spool colors (rgb) indexed by a bead's tool id, used only when
    // ctrl.w == 1. Fixed 16 slots (the machine tool-count ceiling).
    tool_palette: array<vec4<f32>, 16>,
};
@group(0) @binding(0) var<uniform> u: U;

// --- mesh (shaded) ---
// Per-part placement + tint, bound with a dynamic offset per draw call. The
// mesh geometry is uploaded ONCE in object-local space; this is the only thing
// a drag or a color change rewrites (a ~96-byte record), so neither re-walks or
// re-uploads a single vertex. `flags.x` = invalid (build-volume / overlap).
struct PartData {
    model: mat4x4<f32>,
    rgb: vec4<f32>,
    flags: vec4<f32>,
};
@group(1) @binding(0) var<uniform> part: PartData;

struct MeshOut { @builtin(position) clip: vec4<f32>, @location(0) normal: vec3<f32>, @location(1) paint: vec4<f32> };
@vertex fn vs_mesh(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>, @location(2) paint: vec4<f32>) -> MeshOut {
    var o: MeshOut;
    let world = part.model * vec4<f32>(p, 1.0);
    o.clip = u.mvp * world;
    // The local flat normal rotated into world space. `model` is rotation ×
    // uniform-scale + translation, so its 3×3 preserves the normal's direction
    // (scale cancels on normalize) — this reproduces the old CPU flat_normal(of
    // the transformed triangle) exactly, without storing world normals.
    o.normal = (part.model * vec4<f32>(n, 0.0)).xyz;
    // Per-vertex paint tint (rgb + coverage in w), flat across a painted
    // triangle. Zero w = unpainted → the part's base tint shows through.
    o.paint = paint;
    return o;
}
@fragment fn fs_mesh(i: MeshOut) -> @location(0) vec4<f32> {
    // Camera-relative two-light rig (key over the shoulder + a softer fill from
    // the opposite side, both riding the camera — see the App). So whatever you
    // orbit to is lit with form, and the shadow side stays readable instead of
    // sinking to flat ambient. `u.light` = key dir, `u.mesh_unsel` = fill dir.
    let n = normalize(i.normal);
    let kd = max(dot(n, normalize(u.light.xyz)), 0.0);
    let fd = max(dot(n, normalize(u.mesh_unsel.xyz)), 0.0);
    let shade = 0.20 + 0.66 * kd + 0.30 * fd;
    // Base tint: the part's filament color on a toolchanger, or the accent sunk
    // into porcelain on a single tool. An invalid object (outside the build
    // volume, or overlapping another) overrides it with terracotta (the theme's
    // error color) — the warning reads over any filament color; it can't print
    // until fixed. (Selection is shown by the bed spotlight, not a mesh tint.)
    var base = part.rgb.xyz;
    let warn = vec3<f32>(0.862, 0.420, 0.320);
    base = mix(base, warn, part.flags.x);
    // Painted regions tint over the base (and over the warning) so the brush is
    // visible while editing; the tint is still lit by the same rig.
    base = mix(base, i.paint.xyz, i.paint.w);
    return vec4<f32>(base * shade, 1.0);
}

// --- the nozzle: a lit little hotend parked where a mirrored print has
// reached. Its own entry rather than the mesh one because it carries per-
// vertex color (brass, anodized block, aluminium fins) and needs no model
// matrix — the CPU parks it, since it moves every frame and is ~300
// triangles. Lit by the same camera-relative rig as the parts, so it sits in
// the same world rather than floating over it. ---
struct NozzleOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) @interpolate(flat) metal: f32,
};
@vertex fn vs_nozzle(
    @location(0) p: vec3<f32>,
    @location(1) n: vec3<f32>,
    @location(2) c: vec3<f32>,
    @location(3) m: f32,
) -> NozzleOut {
    var o: NozzleOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.normal = n;
    o.color = c;
    o.metal = m;
    return o;
}
@fragment fn fs_nozzle(i: NozzleOut) -> @location(0) vec4<f32> {
    let n = normalize(i.normal);
    let l = normalize(u.light.xyz);
    let kd = max(dot(n, l), 0.0);
    let fd = max(dot(n, normalize(u.mesh_unsel.xyz)), 0.0);
    // Metal is mostly CONTRAST: little ambient, a steep diffuse, so it goes
    // dark where it turns away and bright where it faces the key. Silicone
    // keeps the softer, flatter response.
    let amb = mix(0.28, 0.11, i.metal);
    let dg = mix(0.62, 0.70, i.metal);
    let fg = mix(0.26, 0.30, i.metal);
    var base = i.color * (amb + dg * kd + fg * fd);
    // Metal IS its highlight. Lit by diffuse alone, brass reads as yellow
    // paint — the hue is right and the material is missing. The rig is
    // camera-relative, so the key light stands in for the view direction and
    // dot(n, l) serves as a half-vector: a tight lobe for the hot spot, a
    // broad one for the sheen along the turned flanks. Tinted toward the
    // base colour, because a metal's highlight carries its own colour rather
    // than the light's.
    // Tinted hard toward the metal's own colour: a whiter highlight turns
    // the flat faces — the flange top, the ring above the sock — to cream,
    // which reads as painted rather than turned.
    let tint = mix(vec3<f32>(1.0, 0.97, 0.92), i.color, 0.75);
    base = base + tint * (pow(kd, 16.0) * 0.30 + pow(kd, 5.0) * 0.10) * i.metal;
    return vec4<f32>(base, 1.0);
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

// --- flat z=0 quads, two consumers: the bed's backdrop fill (opaque, plain
// alpha-over via fs_glow) and the selection spotlight (fs_glow_max + MAX
// blending). The spotlight is a splat soup — one gradient quad per silhouette
// edge plus corner fans — whose overlaps MUST NOT stack: with premultiplied
// output and max blending, overlapping splats resolve to the brightest
// (nearest-edge) value, so the pool reads as the outline's distance field
// instead of a pile of translucent wedges. Depth is ignored (it paints over
// the grid) but it is drawn before the opaque mesh, so the object overdraws
// its core. ---
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
@fragment fn fs_glow_max(i: GlowOut) -> @location(0) vec4<f32> {
    return vec4<f32>(i.rgba.rgb * i.rgba.a, i.rgba.a);
}

// --- toolpath beads (instanced boxes) ---
// base box vertex: lpos in (x:[0,1], y/z:[-0.5,0.5]); instance places/scales it.
struct BeadOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) @interpolate(flat) layer: f32,
    @location(3) @interpolate(flat) cat: f32,
    @location(4) @interpolate(flat) tool: f32,
};
@vertex fn vs_bead(
    @location(0) lpos: vec3<f32>,
    @location(1) lnorm: vec3<f32>,
    @location(2) p0: vec3<f32>,
    @location(3) dir_len: vec3<f32>,
    @location(4) dims: vec3<f32>,   // (width at x=0 end, width at x=1 end, height)
    @location(5) color: vec3<f32>,
    @location(6) lc: vec2<f32>,
    @location(7) tool: f32,
    @location(8) tang: vec2<f32>,
) -> BeadOut {
    let xaxis = vec3<f32>(dir_len.x, dir_len.y, 0.0); // along the segment (unit)
    let zaxis = vec3<f32>(0.0, 0.0, 1.0);
    // Place each END's cross-section ring in its vertex's MITER frame (the
    // mean direction of the two segments meeting there, from the instance)
    // at ITS OWN end width. Both segments at a shared vertex then generate
    // the identical world hexagon — same frame, same width — so consecutive
    // tubes weld into one continuous mitered surface even mid-taper, where a
    // single per-instance width left a step at every joint of a width-
    // modulated bead. A tapering segment renders as the cone between its
    // end rings, which is what the deposited bead does.
    let ang = select(tang.x, tang.y, lpos.x > 0.5);
    let wend = select(dims.x, dims.y, lpos.x > 0.5);
    let taxis = vec3<f32>(cos(ang), sin(ang), 0.0);
    let yaxis = cross(zaxis, taxis);                  // across, in the bed plane
    let local = xaxis * (lpos.x * dir_len.z) + yaxis * (lpos.y * wend) + zaxis * (lpos.z * dims.z);
    var o: BeadOut;
    o.clip = u.mvp * vec4<f32>(p0 + local, 1.0);
    // Correct the normal for the non-uniform (width, height) scaling of the
    // cross-section (inverse scale), then rotate into the end frame (so
    // shading is continuous across the welded joint too).
    let n_local = normalize(vec3<f32>(lnorm.x, lnorm.y / wend, lnorm.z / dims.z));
    o.normal = taxis * n_local.x + yaxis * n_local.y + zaxis * n_local.z;
    o.color = color;
    o.layer = lc.x;
    o.cat = lc.y;
    o.tool = tool;
    return o;
}
// The same bead as a single view-facing quad — two triangles instead of the
// tube's twelve. A bead is one to three pixels wide at working zoom, so the
// facets were never visible; what the eye reads is the ribbon's silhouette
// and its round shading, and both survive. The silhouette is exact (the quad
// spans the cross-section's support along the view-perpendicular axis) and
// the roundness is synthesised in the normal, the standard impostor. Only
// the depth is a simplification: the quad is flat where the tube bulged,
// which shows only where beads interpenetrate at extreme zoom.
@vertex fn vs_bead_flat(
    @location(0) lpos: vec3<f32>,
    @location(1) lnorm: vec3<f32>,
    @location(2) p0: vec3<f32>,
    @location(3) dir_len: vec3<f32>,
    @location(4) dims: vec3<f32>,   // (width at x=0 end, width at x=1 end, height)
    @location(5) color: vec3<f32>,
    @location(6) lc: vec2<f32>,
    @location(7) tool: f32,
    @location(8) tang: vec2<f32>,
) -> BeadOut {
    let xaxis = vec3<f32>(dir_len.x, dir_len.y, 0.0); // along the segment (unit)
    let zaxis = vec3<f32>(0.0, 0.0, 1.0);
    let along = xaxis * (lpos.x * dir_len.z);
    // Frame each END on its vertex's MITER tangent (the mean of the two
    // segments meeting there) and view from the vertex itself, not the
    // segment midpoint. Both segments at a shared vertex then receive the
    // same angle and the same eye ray, so they compute the IDENTICAL
    // cross-section edge — the quads weld, where per-segment frames left
    // view-dependent slit gaps between them at close zoom.
    let ang = select(tang.x, tang.y, lpos.x > 0.5);
    let taxis = vec3<f32>(cos(ang), sin(ang), 0.0);
    let yaxis = cross(zaxis, taxis);                  // across, in the bed plane
    let pend = p0 + along;
    var view = u.cam_eye.xyz - pend;
    let vlen = length(view);
    view = select(zaxis, view / vlen, vlen > 1.0e-6);
    // Walk the cross-section, not the screen. The half of the ellipse facing
    // the camera spans a quarter turn either side of the view direction, so
    // the quad's edges land on the silhouette and every point between them
    // carries the angle it would have on the tube — which is what keeps a
    // bead lit from above when seen from above. (Rolling the normal toward
    // the CAMERA instead, the obvious impostor, lights every bead as if the
    // key light sat on the lens: the whole print goes flat and dark.)
    let th_v = atan2(dot(view, zaxis), dot(view, yaxis));
    let th = th_v + lpos.y * 3.14159265;   // lpos.y in [-0.5, 0.5] = +/- a quarter turn
    let c = cos(th);
    let s = sin(th);
    // Each end's OWN width (the taper weld, same as the tube): both quads at
    // a shared vertex then agree on the silhouette edge mid-taper too.
    let a = select(dims.x, dims.y, lpos.x > 0.5) * 0.5;
    let b = dims.z * 0.5;
    var o: BeadOut;
    o.clip = u.mvp * vec4<f32>(pend + yaxis * (a * c) + zaxis * (b * s), 1.0);
    // The ellipse's own normal at that angle (gradient of x²/a² + z²/b²).
    let n_local = normalize(vec2<f32>(c / a, s / b));
    o.normal = yaxis * n_local.x + zaxis * n_local.y;
    o.color = color;
    o.layer = lc.x;
    o.cat = lc.y;
    o.tool = tool;
    return o;
}
@fragment fn fs_bead(i: BeadOut) -> @location(0) vec4<f32> {
    let mask = u32(u.ctrl.z + 0.5);
    let cat = u32(i.cat + 0.5);
    if ((mask & (1u << cat)) == 0u) { discard; }
    // Filament mode (ctrl.w == 1) recolors extrusion from the tool palette;
    // travels (cat 4) and seams (cat 5) keep their baked accent color.
    var col = i.color;
    if (u.ctrl.w > 0.5 && cat != 4u && cat != 5u) {
        col = u.tool_palette[u32(i.tool + 0.5)].rgb;
    }
    // Camera-relative key + fill (u.light / u.mesh_unsel), so bead surfaces read
    // from any orbit angle — a bit more ambient than the mesh so thin beads at a
    // distance stay visible.
    let n = normalize(i.normal);
    let kd = max(dot(n, normalize(u.light.xyz)), 0.0);
    let fd = max(dot(n, normalize(u.mesh_unsel.xyz)), 0.0);
    var shade = 0.30 + 0.58 * kd + 0.26 * fd;
    if (i.layer < u.ctrl.x - 0.5) { shade = shade * u.ctrl.y; } // dim lower layers
    return vec4<f32>(col * shade, 1.0);
}

// --- joint instances: seam markers (blob mesh) and bead end caps (dome
// mesh). One shader serves both draws; dims.z is a bed-plane rotation so a
// cap's dome points OUT of its tube along the end tangent (markers pass 0,
// where the rotation is the identity for the symmetric blob). The dome's
// unit ring matches the tube's unit ring, and the same (width, height)
// scaling applies, so a cap welds onto the tube's end hexagon exactly. ---
@vertex fn vs_joint(
    @location(0) lpos: vec3<f32>,
    @location(1) lnorm: vec3<f32>,
    @location(2) p0: vec3<f32>,
    @location(3) dims: vec3<f32>,   // (width, height, out-angle)
    @location(4) color: vec3<f32>,
    @location(5) lc: vec2<f32>,
    @location(6) tool: f32,
) -> BeadOut {
    let r = vec3<f32>(dims.x * 0.5, dims.x * 0.5, dims.y * 0.5);
    let taxis = vec3<f32>(cos(dims.z), sin(dims.z), 0.0);
    let zaxis = vec3<f32>(0.0, 0.0, 1.0);
    let yaxis = cross(zaxis, taxis);
    let l = lpos * r;
    var o: BeadOut;
    o.clip = u.mvp * vec4<f32>(p0 + taxis * l.x + yaxis * l.y + zaxis * l.z, 1.0);
    let n = normalize(lnorm / r);
    o.normal = taxis * n.x + yaxis * n.y + zaxis * n.z;
    o.color = color;
    o.layer = lc.x;
    o.cat = lc.y;
    o.tool = tool;
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
    /// Per-tool spool colors (rgb in xyz), indexed by a bead's tool id; read
    /// only when `ctrl.w == 1` (Filament mode). 16 = the tool-count ceiling.
    tool_palette: [[f32; 4]; 16],
}

/// Number of tool-palette slots (the machine tool-count ceiling); must match the
/// `array<vec4<f32>, 16>` in the shader.
pub const TOOL_PALETTE_LEN: usize = 16;

/// Per-part placement + tint, one per drawn part, addressed by a dynamic uniform
/// offset. Geometry stays static in object-local space; a drag rewrites only
/// `model`, a recolor only `rgb`/`flags` — 96 bytes, no vertex touched.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct PartData {
    /// Column-major model matrix (object-local → bed/world), same convention as
    /// `Uniforms::mvp`.
    model: [[f32; 4]; 4],
    /// xyz = base tint (filament/accent); w unused.
    rgb: [f32; 4],
    /// x = invalid (1 → warning tint); yzw unused.
    flags: [f32; 4],
}

/// Dynamic-uniform stride for `PartData` — must be a multiple of the device's
/// `min_uniform_buffer_offset_alignment` (256 on desktop backends). `PartData`
/// is 96 bytes, so it pads out to one 256-byte slot per part.
const PART_STRIDE: u64 = 256;

/// How to draw the toolpaths this frame.
pub struct Preview {
    /// Number of bead (segment) instances to draw, through the current layer.
    pub count: u32,
    /// Number of joint-blob instances to draw, through the current layer.
    pub joint_count: u32,
    pub cap_count: u32,
    /// Current (top visible) layer, 1-based.
    pub current_layer: f32,
    /// Brightness multiplier for layers below the current one (1.0 = no dim).
    pub dim: f32,
    /// Category visibility bitmask (bit per category id).
    pub mask: u32,
    /// Bead color mode: 0 = draw each bead's baked rgb (Feature / LayerTime, and
    /// the headless oracle); 1 = Filament: recolor extrusion from `tool_palette`
    /// by tool id so a spool-color change is a uniform write, not a rebuild.
    pub color_mode: u32,
    /// Per-tool spool colors (rgb; index by tool id). Only read when
    /// `color_mode == 1`. Extra slots are zero.
    pub tool_palette: [[f32; 4]; TOOL_PALETTE_LEN],
    /// Draw the nozzle body (uploaded separately by `set_nozzle`) — on for a
    /// mirrored print, off for a slice preview.
    pub nozzle: bool,
    /// Draw beads as view-facing quads rather than tubes: a sixth of the
    /// vertices, for the view that redraws every frame.
    pub flat_beads: bool,
}

pub struct Scene {
    format: wgpu::TextureFormat,
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    nozzle_pipeline: wgpu::RenderPipeline,
    bead_pipeline: wgpu::RenderPipeline,
    bead_flat_pipeline: wgpu::RenderPipeline,
    uniform_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    /// The render-target size — halved while the camera is moving (dynamic
    /// resolution), so it can differ from the on-screen size below.
    size: (u32, u32),
    /// The full on-screen size, fed to the star-billboard shader so the backdrop
    /// stars keep a constant apparent size even when `size` is scaled down.
    /// Tracks `size` by default (the headless oracle never scales), so the
    /// offscreen render is unaffected.
    display_size: (u32, u32),
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    resolve_view: wgpu::TextureView,
    /// The single-sample resolve texture (kept for headless read-back).
    resolve_tex: wgpu::Texture,
    /// egui texture handle — `None` in the headless (offscreen) path.
    tex_id: Option<TextureId>,
    /// Concatenated object-LOCAL mesh geometry (pos+normal), uploaded once per
    /// structure change (import/delete). Placement + tint live in `part_ubuf`,
    /// so a drag/recolor never touches this.
    mesh_geo: GrowBuf,
    /// Per-vertex paint tint (rgb + coverage), parallel to `mesh_geo`. Kept the
    /// same length as `mesh_geo` (zeroed by `upload_mesh_geometry`, overwritten
    /// by `upload_mesh_paint`).
    mesh_paint: GrowBuf,
    /// Per-part draw ranges `(vertex_offset, vertex_count)` into `mesh_geo`.
    mesh_parts: Vec<(u32, u32)>,
    /// Dynamic uniform buffer of `PartData`, one 256-byte slot per part.
    part_ubuf: Option<wgpu::Buffer>,
    part_ubuf_cap: u64,
    part_bgl: wgpu::BindGroupLayout,
    part_bind_group: Option<wgpu::BindGroup>,
    line_vbuf: GrowBuf,
    line_count: u32,
    glow_pipeline: wgpu::RenderPipeline,
    /// The selection spotlight's own pipeline: premultiplied + MAX blending,
    /// so the splat soup's overlaps take the nearest-edge value instead of
    /// stacking into bright wedges.
    spot_pipeline: wgpu::RenderPipeline,
    glow_vbuf: GrowBuf,
    glow_count: u32,
    // Night-sky backdrop: a fixed catalog of star billboards (base quad +
    // per-star instance = direction + magnitude), drawn first with depth off.
    star_pipeline: wgpu::RenderPipeline,
    star_quad_vbuf: wgpu::Buffer,
    star_inst_vbuf: wgpu::Buffer,
    star_count: u32,
    /// Opaque backdrop-colored bed fill (glow pipeline), so stars don't show
    /// through the grid. Built by `set_beds`.
    bed_fill_vbuf: GrowBuf,
    bed_fill_count: u32,
    box_vbuf: wgpu::Buffer,
    box_count: u32,
    /// The flat bead's two triangles (see `vs_bead_flat`).
    quad_vbuf: wgpu::Buffer,
    quad_count: u32,
    inst_vbuf: GrowBuf,
    inst_count: u32,
    joint_pipeline: wgpu::RenderPipeline,
    blob_vbuf: wgpu::Buffer,
    blob_count: u32,
    joint_vbuf: GrowBuf,
    joint_count: u32,
    cap_vbuf: wgpu::Buffer,
    cap_mesh_count: u32,
    cap_inst: GrowBuf,
    cap_count: u32,
    nozzle_vbuf: GrowBuf,
    nozzle_count: u32,
    label_pipeline: wgpu::RenderPipeline,
    label_bgl: wgpu::BindGroupLayout,
    label_sampler: wgpu::Sampler,
    label_bind_group: Option<wgpu::BindGroup>,
    label_vbuf: GrowBuf,
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

        // Per-part placement + tint (mesh pipeline, group 1), addressed by a
        // dynamic offset so one draw per part carries its own `PartData`.
        let part_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("part_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<PartData>() as u64),
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

        // The mesh pipeline also binds the per-part uniform (group 1).
        let mesh_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mesh_layout"),
            bind_group_layouts: &[Some(&bgl), Some(&part_bgl)],
            immediate_size: 0,
        });

        let mesh_pipeline = make_pipeline(
            device, &mesh_layout, &shader, format, "vs_mesh", "fs_mesh",
            &[
                wgpu::VertexBufferLayout {
                    // Object-local geometry: pos + local flat normal.
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                },
                wgpu::VertexBufferLayout {
                    // Parallel per-vertex paint tint (rgb + coverage), same order
                    // and count as the geometry buffer. A recolor rewrites only
                    // this; geometry stays put.
                    array_stride: std::mem::size_of::<[f32; 4]>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x4],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
            Some(wgpu::Face::Back),
        );
        let nozzle_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_nozzle", "fs_nozzle",
            &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<[f32; 10]>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![
                    0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32
                ],
            }],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
            // Two-sided: the bands are open tubes, so a fin's underside must
            // draw when the camera is below it.
            None,
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
            None,
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
                    array_stride: (17 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x3, 3 => Float32x3, 4 => Float32x3, 5 => Float32x3, 6 => Float32x2, 7 => Float32, 8 => Float32x2],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
            Some(wgpu::Face::Back),
        );
        let bead_flat_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_bead_flat", "fs_bead",
            &[
                wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                },
                wgpu::VertexBufferLayout {
                    array_stride: (17 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x3, 3 => Float32x3, 4 => Float32x3, 5 => Float32x3, 6 => Float32x2, 7 => Float32, 8 => Float32x2],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
            // No culling: the quad turns to face the camera, so which way it
            // winds depends on where you are standing.
            None,
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
                    array_stride: (12 * 4) as u64,
                    step_mode: wgpu::VertexStepMode::Instance,
                    attributes: &wgpu::vertex_attr_array![2 => Float32x3, 3 => Float32x3, 4 => Float32x3, 5 => Float32x2, 6 => Float32],
                },
            ],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState::REPLACE,
            true,
            wgpu::CompareFunction::Less,
            Some(wgpu::Face::Back),
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
            None,
        );

        // Flat z=0 quads (bed backdrop fill): alpha-blended like the label, but
        // keyed off the group(0) uniforms only (no texture), with depth reads
        // OFF so it paints over the stars.
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
            None,
        );

        // Selection spotlight: same flat quads, but premultiplied + MAX
        // compositing — overlapping splats take the brighter (nearest-edge)
        // fragment instead of stacking, so the gradient pool reads as one
        // distance field however much its edge quads and corner fans overlap.
        // (Max also means the grid stays visible under the pool rather than
        // being dimmed — a lighting cue, not a coat of paint.)
        let spot_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_glow", "fs_glow_max",
            &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<GlowVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
            }],
            wgpu::PrimitiveTopology::TriangleList,
            wgpu::BlendState {
                color: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::One,
                    operation: wgpu::BlendOperation::Max,
                },
                alpha: wgpu::BlendComponent {
                    src_factor: wgpu::BlendFactor::One,
                    dst_factor: wgpu::BlendFactor::One,
                    operation: wgpu::BlendOperation::Max,
                },
            },
            false,
            wgpu::CompareFunction::Always,
            None,
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
            None,
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
        let quad_verts = bead_quad_vertices();
        let quad_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("bead_quad_base"),
            contents: bytemuck::cast_slice(&quad_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let blob_verts = blob_vertices();
        let blob_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("joint_base"),
            contents: bytemuck::cast_slice(&blob_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let cap_verts = cap_vertices();
        let cap_vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cap_base"),
            contents: bytemuck::cast_slice(&cap_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let (color_view, depth_view, resolve_view, resolve_tex) = make_targets(device, format, 1, 1);

        Self {
            format,
            mesh_pipeline,
            line_pipeline,
            nozzle_pipeline,
            bead_pipeline,
            bead_flat_pipeline,
            uniform_buf,
            bind_group,
            size: (1, 1),
            display_size: (1, 1),
            color_view,
            depth_view,
            resolve_view,
            resolve_tex,
            tex_id: None,
            mesh_geo: GrowBuf::default(),
            mesh_paint: GrowBuf::default(),
            mesh_parts: Vec::new(),
            part_ubuf: None,
            part_ubuf_cap: 0,
            part_bgl,
            part_bind_group: None,
            line_vbuf: GrowBuf::default(),
            line_count: 0,
            glow_pipeline,
            spot_pipeline,
            glow_vbuf: GrowBuf::default(),
            glow_count: 0,
            star_pipeline,
            star_quad_vbuf,
            star_inst_vbuf,
            star_count,
            bed_fill_vbuf: GrowBuf::default(),
            bed_fill_count: 0,
            box_vbuf,
            box_count: box_verts.len() as u32,
            quad_vbuf,
            quad_count: quad_verts.len() as u32,
            inst_vbuf: GrowBuf::default(),
            inst_count: 0,
            joint_pipeline,
            blob_vbuf,
            blob_count: blob_verts.len() as u32,
            joint_vbuf: GrowBuf::default(),
            joint_count: 0,
            cap_vbuf,
            cap_mesh_count: cap_verts.len() as u32,
            cap_inst: GrowBuf::default(),
            cap_count: 0,
            nozzle_vbuf: GrowBuf::default(),
            nozzle_count: 0,
            label_pipeline,
            label_bgl,
            label_sampler,
            label_bind_group: None,
            label_vbuf: GrowBuf::default(),
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
        // Default the on-screen size to the target size; the GUI overrides it via
        // `set_display_size` when it renders at a reduced (dynamic) resolution.
        self.display_size = (w, h);
        true
    }

    /// Set the full on-screen size (for the star-billboard shader) independently
    /// of the render-target `size`, so dynamic-resolution rendering doesn't
    /// balloon the backdrop stars.
    pub fn set_display_size(&mut self, w: u32, h: u32) {
        self.display_size = (w.max(1), h.max(1));
    }

    /// How many parts the current geometry buffer holds — the caller uploads
    /// fresh geometry whenever its part list length no longer matches this.
    pub fn mesh_part_count(&self) -> usize {
        self.mesh_parts.len()
    }

    /// Upload each part's mesh in OBJECT-LOCAL space (pos + local flat normal),
    /// concatenated into one buffer with a `(offset, count)` draw range per part.
    /// Called only when the part SET changes (import/delete) — a drag or recolor
    /// leaves this untouched and rewrites just the per-part uniforms. `meshes`
    /// must be in the same order as the `parts` handed to `upload_mesh_parts`.
    pub fn upload_mesh_geometry(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, meshes: &[&mesh::Mesh]) {
        let mut verts: Vec<Vertex> = Vec::new();
        self.mesh_parts.clear();
        for mesh in meshes {
            let start = verts.len() as u32;
            for i in 0..mesh.triangles.len() {
                let tri = mesh.triangle(i);
                let f3 = |v: [f64; 3]| [v[0] as f32, v[1] as f32, v[2] as f32];
                let p: [[f32; 3]; 3] = [f3(tri[0]), f3(tri[1]), f3(tri[2])];
                // Flat normal in LOCAL space; the shader rotates it by the part
                // matrix, reproducing the old transformed flat_normal exactly.
                let n = flat_normal(p[0], p[1], p[2]);
                for pos in p {
                    verts.push(Vertex { pos, normal: n });
                }
            }
            self.mesh_parts.push((start, verts.len() as u32 - start));
        }
        self.mesh_geo.write(device, queue, "mesh_geo", bytemuck::cast_slice(&verts));
        // Keep the paint buffer valid and the same length (all-unpainted) so the
        // draw can always bind it; painting overwrites it via `upload_mesh_paint`.
        let zero = vec![[0.0f32; 4]; verts.len()];
        self.mesh_paint.write(device, queue, "mesh_paint", bytemuck::cast_slice(&zero));
    }

    /// Overwrite the per-vertex paint tint (rgb + coverage in w), one entry per
    /// vertex in the SAME order/count as `upload_mesh_geometry` produced (3 per
    /// triangle, parts concatenated). Painting rewrites only this buffer.
    pub fn upload_mesh_paint(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, paint: &[[f32; 4]]) {
        self.mesh_paint.write(device, queue, "mesh_paint", bytemuck::cast_slice(paint));
    }

    /// Rewrite the per-part placement + tint uniforms — the ONLY thing a drag
    /// (new `model`) or a recolor (new `rgb`/`invalid`) touches. `parts` must
    /// match the `meshes` order/count from `upload_mesh_geometry`.
    pub fn upload_mesh_parts(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        parts: &[([[f32; 4]; 4], [f32; 3], bool)],
    ) {
        let needed = (parts.len() as u64).max(1) * PART_STRIDE;
        if self.part_ubuf.is_none() || needed > self.part_ubuf_cap {
            let cap = needed.max(self.part_ubuf_cap * 2).max(PART_STRIDE * 16);
            let buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("part_uniforms"),
                size: cap,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.part_bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("part_bg"),
                layout: &self.part_bgl,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                        buffer: &buf,
                        offset: 0,
                        size: wgpu::BufferSize::new(std::mem::size_of::<PartData>() as u64),
                    }),
                }],
            }));
            self.part_ubuf = Some(buf);
            self.part_ubuf_cap = cap;
        }
        // One 256-byte slot per part (the rest of each slot is padding).
        let mut bytes = vec![0u8; parts.len() * PART_STRIDE as usize];
        for (i, (model, rgb, invalid)) in parts.iter().enumerate() {
            let pd = PartData {
                model: *model,
                rgb: [rgb[0], rgb[1], rgb[2], 0.0],
                flags: [if *invalid { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0],
            };
            let off = i * PART_STRIDE as usize;
            bytes[off..off + std::mem::size_of::<PartData>()].copy_from_slice(bytemuck::bytes_of(&pd));
        }
        if let Some(buf) = &self.part_ubuf {
            if !bytes.is_empty() {
                queue.write_buffer(buf, 0, &bytes);
            }
        }
    }

    /// Build the bed grids: `n` beds in a row along +X, `gap` apart. The
    /// active bed gets the cream border and full-strength grid; the others
    /// recede into the ink.
    pub fn set_beds(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
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
        self.line_vbuf.write(device, queue, "bed_vbuf", bytemuck::cast_slice(&v));
        self.bed_fill_count = fill.len() as u32;
        self.bed_fill_vbuf.write(device, queue, "bed_fill", bytemuck::cast_slice(&fill));
    }

    /// Upload the selection spotlight: a triangle soup on the bed plane where
    /// each vertex is `[x, y, z, r, g, b, a]`. An empty slice clears it.
    pub fn set_spotlight(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, verts: &[[f32; 7]]) {
        if verts.is_empty() {
            self.glow_count = 0;
            return;
        }
        let gv: Vec<GlowVertex> = verts
            .iter()
            .map(|v| GlowVertex { pos: [v[0], v[1], v[2]], rgba: [v[3], v[4], v[5], v[6]] })
            .collect();
        self.glow_count = gv.len() as u32;
        self.glow_vbuf.write(device, queue, "glow_vbuf", bytemuck::cast_slice(&gv));
    }

    /// Upload bead instances: `[p0.xyz, dir.xy, len, width, height, r, g, b, layer, cat, tool]`.
    pub fn set_toolpaths(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, instances: &[[f32; 17]]) {
        self.inst_count = instances.len() as u32;
        self.inst_vbuf.write(device, queue, "bead_instances", bytemuck::cast_slice(instances));
    }

    /// Upload the nozzle body, in world coordinates: `[pos.xyz, normal.xyz,
    /// rgb]` per vertex. Rewritten whenever it moves — a few hundred vertices,
    /// which is cheaper than threading a model matrix through the uniform.
    /// An empty slice hides it.
    pub fn set_nozzle(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, verts: &[[f32; 10]]) {
        self.nozzle_count = verts.len() as u32;
        if !verts.is_empty() {
            self.nozzle_vbuf.write(device, queue, "nozzle_body", bytemuck::cast_slice(verts));
        }
    }

    /// Upload joint-blob instances: `[p0.xyz, width, height, r, g, b, layer, cat, tool]`.
    pub fn set_joints(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, joints: &[[f32; 12]]) {
        self.joint_count = joints.len() as u32;
        self.joint_vbuf.write(device, queue, "joint_instances", bytemuck::cast_slice(joints));
    }

    /// Bead end caps: same instance layout as joints, drawn with the dome mesh.
    pub fn set_caps(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, caps: &[[f32; 12]]) {
        self.cap_count = caps.len() as u32;
        self.cap_inst.write(device, queue, "cap_instances", bytemuck::cast_slice(caps));
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
    pub fn set_label_geom(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, verts: &[[f32; 5]]) {
        self.label_count = verts.len() as u32;
        self.label_vbuf.write(device, queue, "label", bytemuck::cast_slice(verts));
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
        key: [f32; 3],
        fill: [f32; 3],
        label_color: [f32; 4],
    ) {
        self.render_to(
            &rs.device, &rs.queue, view_proj, cam_eye, show_stars, show_mesh, preview, key, fill,
            label_color,
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
        key: [f32; 3],
        fill: [f32; 3],
        label_color: [f32; 4],
    ) {
        let ctrl = match &preview {
            Some(p) => [p.current_layer, p.dim, p.mask as f32, p.color_mode as f32],
            None => [0.0, 1.0, 0.0, 0.0],
        };
        let tool_palette = match &preview {
            Some(p) => p.tool_palette,
            None => [[0.0; 4]; TOOL_PALETTE_LEN],
        };
        let uniforms = Uniforms {
            mvp: view_proj.to_cols_array_2d(),
            light: [key[0], key[1], key[2], 0.0],
            ctrl,
            mesh_unsel: [fill[0], fill[1], fill[2], 0.0],
            mesh_sel: [0.0; 4],
            label_color,
            cam_eye: [cam_eye.x, cam_eye.y, cam_eye.z, 0.0],
            // Full on-screen size (not the possibly-scaled render target), so the
            // star billboards keep a constant apparent size at reduced resolution.
            viewport: [self.display_size.0 as f32, self.display_size.1 as f32, 0.0, 0.0],
            tool_palette,
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
                if let Some(buf) = self.bed_fill_vbuf.as_ref() {
                    if self.bed_fill_count > 0 {
                        pass.set_pipeline(&self.glow_pipeline);
                        pass.set_vertex_buffer(0, buf.slice(..));
                        pass.draw(0..self.bed_fill_count, 0..1);
                    }
                }
            }

            if let Some(buf) = self.line_vbuf.as_ref() {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.line_count, 0..1);
            }

            // The selection spotlight: right after the grid so the opaque model
            // (drawn below) overdraws its core and the glow spills out around
            // the footprint. Model view only.
            if show_mesh && self.glow_count > 0 {
                if let Some(buf) = self.glow_vbuf.as_ref() {
                    pass.set_pipeline(&self.spot_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.glow_count, 0..1);
                }
            }

            if let Some(p) = &preview {
                let n = p.count.min(self.inst_count);
                if n > 0 {
                    if let Some(inst) = self.inst_vbuf.as_ref() {
                        let (pipe, base, count) = if p.flat_beads {
                            (&self.bead_flat_pipeline, &self.quad_vbuf, self.quad_count)
                        } else {
                            (&self.bead_pipeline, &self.box_vbuf, self.box_count)
                        };
                        pass.set_pipeline(pipe);
                        pass.set_vertex_buffer(0, base.slice(..));
                        pass.set_vertex_buffer(1, inst.slice(..));
                        pass.draw(0..count, 0..n);
                    }
                }
                if p.nozzle && self.nozzle_count > 0 {
                    if let Some(nbuf) = self.nozzle_vbuf.as_ref() {
                        pass.set_pipeline(&self.nozzle_pipeline);
                        pass.set_vertex_buffer(0, nbuf.slice(..));
                        pass.draw(0..self.nozzle_count, 0..1);
                    }
                }
                let jn = p.joint_count.min(self.joint_count);
                if jn > 0 {
                    if let Some(jinst) = self.joint_vbuf.as_ref() {
                        pass.set_pipeline(&self.joint_pipeline);
                        pass.set_vertex_buffer(0, self.blob_vbuf.slice(..));
                        pass.set_vertex_buffer(1, jinst.slice(..));
                        pass.draw(0..self.blob_count, 0..jn);
                    }
                }
                let cn = p.cap_count.min(self.cap_count);
                if cn > 0 {
                    if let Some(cinst) = self.cap_inst.as_ref() {
                        pass.set_pipeline(&self.joint_pipeline);
                        pass.set_vertex_buffer(0, self.cap_vbuf.slice(..));
                        pass.set_vertex_buffer(1, cinst.slice(..));
                        pass.draw(0..self.cap_mesh_count, 0..cn);
                    }
                }
            }

            if show_mesh {
                // One draw per part: static local geometry (bound once) + the
                // part's placement/tint via a dynamic uniform offset.
                if let (Some(geo), Some(paint), Some(pbg)) =
                    (self.mesh_geo.as_ref(), self.mesh_paint.as_ref(), &self.part_bind_group)
                {
                    if !self.mesh_parts.is_empty() {
                        pass.set_pipeline(&self.mesh_pipeline);
                        pass.set_vertex_buffer(0, geo.slice(..));
                        pass.set_vertex_buffer(1, paint.slice(..));
                        for (i, &(off, count)) in self.mesh_parts.iter().enumerate() {
                            if count == 0 {
                                continue;
                            }
                            pass.set_bind_group(1, pbg, &[(i as u64 * PART_STRIDE) as u32]);
                            pass.draw(off..off + count, 0..1);
                        }
                    }
                }
            }

            // The bed label LAST, so it depth-tests against the model/beads and
            // is hidden per-pixel where they stand in front of it.
            if self.label_count > 0 {
                if let (Some(bg), Some(vbuf)) = (&self.label_bind_group, self.label_vbuf.as_ref()) {
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
/// Two triangles spanning the segment: x along it in [0,1], y across it in
/// [-0.5, 0.5]. `vs_bead_flat` turns this to face the camera and sizes it to
/// the cross-section's silhouette, so six vertices stand in for the tube's
/// thirty-six.
fn bead_quad_vertices() -> Vec<Vertex> {
    let n = [0.0, 0.0, 1.0]; // unused: the shader synthesises the normal
    let v = |x: f32, y: f32| Vertex { pos: [x, y, 0.0], normal: n };
    // THREE columns across, not two. With only the silhouette edges, their
    // normals point in opposite directions and interpolate through zero —
    // every bead goes dark down its middle. The extra column carries the
    // crown of the cross-section, where the normal faces the camera-side
    // top, and the two halves interpolate over a quarter turn each. Still
    // twelve vertices against the tube's thirty-six.
    vec![
        v(0.0, -0.5), v(0.0, 0.0), v(1.0, 0.0),
        v(0.0, -0.5), v(1.0, 0.0), v(1.0, -0.5),
        v(0.0, 0.0), v(0.0, 0.5), v(1.0, 0.5),
        v(0.0, 0.0), v(1.0, 0.5), v(1.0, 0.0),
    ]
}

/// (unit circle radius 0.5 in the Y-Z plane). The instance scales the
/// cross-section to (line width, layer height). Ends are left open; a joint blob
/// at every vertex rounds the ends and fills corners between segments.
fn bead_vertices() -> Vec<Vertex> {
    // A hexagonal tube: at bead scale (a fraction of a mm) the extra facets of
    // an octagon aren't visible, and dropping 8→6 sides is 25% fewer verts per
    // bead across the whole preview.
    const N: usize = 6;
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
/// Unit bead end cap: a half-dome closing the tube's hexagonal cross-section
/// (ring at x=0 matching `bead_vertices`' ring phase, dome bulging to a pole
/// at x=+0.5). Instanced through `vs_joint` with the end's own (width,
/// height) and the OUTWARD tangent angle, its base hexagon lands exactly on
/// the tube's welded end ring — the cap continues the bead's surface instead
/// of a ball intersecting it.
fn cap_vertices() -> Vec<Vertex> {
    const N: usize = 6;
    // vs_joint scales by (w/2, w/2, h/2): build on the unit sphere so
    // positions double as normals (same convention as the blob).
    let ring = |phi: f32| -> Vec<[f32; 3]> {
        (0..N)
            .map(|k| {
                let t = std::f32::consts::TAU * (k as f32) / (N as f32);
                [phi.sin(), phi.cos() * t.cos(), phi.cos() * t.sin()]
            })
            .collect()
    };
    let r0 = ring(0.0);
    let r1 = ring(std::f32::consts::FRAC_PI_4);
    let pole = [1.0, 0.0, 0.0];
    let v = |p: [f32; 3]| Vertex { pos: p, normal: p };
    let mut out = Vec::with_capacity(9 * N);
    for k in 0..N {
        let k1 = (k + 1) % N;
        // base ring -> mid ring (quad), mid ring -> pole (tri)
        out.extend_from_slice(&[v(r0[k]), v(r0[k1]), v(r1[k1]), v(r0[k]), v(r1[k1]), v(r1[k])]);
        out.extend_from_slice(&[v(r1[k]), v(r1[k1]), v(pole)]);
    }
    out
}

fn blob_vertices() -> Vec<Vertex> {
    // A tiny corner/end filler — a pentagonal bipyramid is plenty round at this
    // scale (and joints are now culled to only sharp corners anyway).
    const S: usize = 5;
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

/// A persistent GPU vertex buffer that grows on demand. It reuses its
/// allocation via `queue.write_buffer` whenever the new upload fits the current
/// capacity, and only reallocates (with 1.5× headroom) when it must grow — so a
/// recolor, a drag, or a re-slice that produces the same-or-smaller byte count
/// never frees+reallocs a fresh buffer the way the old `create_buffer_init` path
/// did. Modeled on the already-correct `uniform_buf` (COPY_DST + write_buffer).
#[derive(Default)]
struct GrowBuf {
    buf: Option<wgpu::Buffer>,
    cap: u64,
}

impl GrowBuf {
    /// Upload `bytes` into the retained buffer, growing the allocation only when
    /// it no longer fits. Empty uploads keep the allocation untouched (callers
    /// zero their own count so nothing is drawn). Requires `bytes.len()` to be a
    /// multiple of 4 (all callers pass `f32` arrays).
    fn write(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, label: &str, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        let need = bytes.len() as u64;
        if let Some(buf) = &self.buf {
            if need <= self.cap {
                queue.write_buffer(buf, 0, bytes);
                return;
            }
        }
        // Grow past the immediate need so a steadily-growing buffer doesn't
        // realloc every step; round to COPY_BUFFER_ALIGNMENT (4).
        let cap = (need.max(self.cap * 3 / 2) + 3) & !3;
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: cap,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&buf, 0, bytes);
        self.buf = Some(buf);
        self.cap = cap;
    }

    fn as_ref(&self) -> Option<&wgpu::Buffer> {
        self.buf.as_ref()
    }
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
/// startup (`pick_samples`) to the most the device supports, capped at 4× (the
/// WebGPU-guaranteed baseline — universal, and cheap enough for the dense bead
/// preview).
static SAMPLES: AtomicU32 = AtomicU32::new(4);

/// Highest MSAA count in {4, 2, 1} the device supports for both the color and
/// depth attachments.
///
/// Capped at 4× (the WebGPU-guaranteed baseline, and what the offscreen oracle
/// already runs at). On a dense, high-overdraw bead preview, 8× doubled the
/// depth + coverage + resolve bandwidth of every orbit/scrub redraw for
/// near-invisible quality gain over 4× on opaque beads — so we no longer ask
/// for it. This also drops the need for TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES
/// to avoid an 8×-target panic.
fn pick_samples(_rs: &RenderState, color: wgpu::TextureFormat) -> u32 {
    let cf = _rs.adapter.get_texture_format_features(color).flags;
    let df = _rs.adapter.get_texture_format_features(DEPTH_FORMAT).flags;
    [4, 2, 1]
        .into_iter()
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

/// The camera-relative KEY + FILL light directions for the mesh/bead shaders.
/// The key rides over the camera's upper-right shoulder and the fill comes from
/// the lower-left, both biased toward the viewer — so whatever you orbit to is
/// lit with form, and the shadow side stays readable instead of sinking to flat
/// ambient. Returns unit vectors (never zero) for the fixed camera too (offscreen).
pub fn camera_lights(eye: glam::Vec3, target: glam::Vec3) -> ([f32; 3], [f32; 3]) {
    let fwd = (target - eye).normalize_or_zero(); // into the screen
    let mut right = fwd.cross(glam::Vec3::Z);
    if right.length_squared() < 1.0e-6 {
        right = glam::Vec3::X; // looking straight down — pick any horizontal
    }
    let right = right.normalize();
    let up = right.cross(fwd).normalize();
    let key = (-fwd * 0.55 + up * 0.62 + right * 0.5).normalize_or_zero();
    let fill = (-fwd * 0.8 - up * 0.25 - right * 0.55).normalize_or_zero();
    (key.to_array(), fill.to_array())
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
    // Back-face culling for the closed solids (beads/joints/mesh) whose inner
    // faces are never visible; `None` for lines and the single-sided flat quads
    // (spotlight/label/bed-fill/stars) that must draw from both sides.
    cull: Option<wgpu::Face>,
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
            cull_mode: cull,
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
