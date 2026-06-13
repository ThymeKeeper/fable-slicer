//! `slicer-gui` — desktop GUI: import STL/3MF models as a multi-object scene,
//! lay them out on the bed, choose profiles, slice, preview toolpaths in 3D,
//! and export g-code.

mod camera;
mod render;

use camera::Camera;
use eframe::egui;
use render::Scene;

use config::{FilamentProfile, PrinterProfile, ProcessProfile, Profiles, Settings, Tier, TierKind};
use engine::generate;
use std::sync::Arc;

/// The "ink & cream" palette: the wordmark's warm paper colors inverted into
/// a dark mode. Surfaces are warm near-blacks (ink), text is the icon's cream,
/// and the wordmark gradient's blush is the one accent. Everything chrome
/// derives from these — preview/heat colors stay semantic and are not here.
mod palette {
    use eframe::egui::Color32;

    /// Deepest surface: the viewport stage, text-entry wells, code blocks.
    pub const INK_DEEP: Color32 = Color32::from_rgb(17, 14, 11);
    /// Panel / window surface.
    pub const INK: Color32 = Color32::from_rgb(26, 22, 17);
    /// Raised widgets (buttons, checkboxes, slider rails).
    pub const INK_RAISED: Color32 = Color32::from_rgb(39, 34, 27);
    /// Hovered widgets.
    pub const INK_HOVER: Color32 = Color32::from_rgb(52, 45, 36);
    /// Pressed widgets.
    pub const INK_ACTIVE: Color32 = Color32::from_rgb(63, 55, 44);

    /// Headline cream — the icon tile / wordmark "F".
    pub const CREAM: Color32 = Color32::from_rgb(242, 236, 222);
    /// Body text.
    pub const CREAM_DIM: Color32 = Color32::from_rgb(189, 181, 163);
    /// Weak / hint text.
    pub const CREAM_FAINT: Color32 = Color32::from_rgb(142, 134, 120);

    /// The wordmark gradient's far end — selection strokes, links, highlights.
    pub const BLUSH: Color32 = Color32::from_rgb(230, 212, 226);
    /// Blush sunk into ink — selection fills, slider trailing fill.
    pub const PLUM: Color32 = Color32::from_rgb(84, 64, 78);

    /// Warm status colors (terracotta / amber, not alarm red / traffic yellow).
    pub const ERROR: Color32 = Color32::from_rgb(224, 118, 92);
    pub const WARN: Color32 = Color32::from_rgb(214, 164, 92);

    /// Hairline rule: cream at low alpha (premultiplied).
    pub const HAIRLINE: Color32 = Color32::from_rgba_premultiplied(25, 24, 23, 26);
    /// Slightly louder hairline for hovered outlines.
    pub const HAIRLINE_LOUD: Color32 = Color32::from_rgba_premultiplied(57, 55, 52, 60);
}

/// Apply the ink & cream theme: warm dark visuals, square-ish corners,
/// hairline strokes, and a slightly airier vertical rhythm than egui's
/// default. Called once at startup.
fn apply_ink_cream(ctx: &eframe::egui::Context) {
    use palette::*;
    let mut style = (*ctx.global_style()).clone();
    let v = &mut style.visuals;

    v.panel_fill = INK;
    v.window_fill = INK;
    v.window_stroke = egui::Stroke::new(1.0, HAIRLINE_LOUD);
    v.extreme_bg_color = INK_DEEP;
    v.code_bg_color = INK_DEEP;
    v.faint_bg_color = INK_RAISED;
    v.selection.bg_fill = PLUM;
    v.selection.stroke = egui::Stroke::new(1.0, BLUSH);
    v.hyperlink_color = BLUSH;
    v.error_fg_color = ERROR;
    v.warn_fg_color = WARN;
    // Sliders show their filled portion — the value reads at a glance.
    v.slider_trailing_fill = true;

    let r = egui::CornerRadius::same(3);
    let w = &mut v.widgets;
    w.noninteractive.bg_fill = INK;
    w.noninteractive.weak_bg_fill = INK;
    w.noninteractive.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);
    w.noninteractive.fg_stroke = egui::Stroke::new(1.0, CREAM_DIM);
    w.noninteractive.corner_radius = r;
    w.inactive.bg_fill = INK_RAISED;
    w.inactive.weak_bg_fill = INK_RAISED;
    w.inactive.bg_stroke = egui::Stroke::NONE;
    w.inactive.fg_stroke = egui::Stroke::new(1.0, CREAM_DIM);
    w.inactive.corner_radius = r;
    w.hovered.bg_fill = INK_HOVER;
    w.hovered.weak_bg_fill = INK_HOVER;
    w.hovered.bg_stroke = egui::Stroke::new(1.0, HAIRLINE_LOUD);
    w.hovered.fg_stroke = egui::Stroke::new(1.5, CREAM);
    w.hovered.corner_radius = r;
    w.active.bg_fill = INK_ACTIVE;
    w.active.weak_bg_fill = INK_ACTIVE;
    w.active.bg_stroke = egui::Stroke::new(1.0, BLUSH);
    w.active.fg_stroke = egui::Stroke::new(1.5, CREAM);
    w.active.corner_radius = r;
    w.open.bg_fill = INK_RAISED;
    w.open.weak_bg_fill = INK_RAISED;
    w.open.bg_stroke = egui::Stroke::new(1.0, HAIRLINE);
    w.open.fg_stroke = egui::Stroke::new(1.0, CREAM);
    w.open.corner_radius = r;

    style.spacing.item_spacing = egui::vec2(8.0, 5.0);
    style.spacing.button_padding = egui::vec2(8.0, 3.0);
    ctx.set_global_style(style);
}

/// State of the save / delete profile dialog.
struct ProfileDialog {
    kind: TierKind,
    name: String,
    delete: bool,
}

/// A printer-host action requested from the controls panel; executed after
/// the panel closure returns (it borrows the settings).
enum HostOp {
    Test,
    Send { start: bool },
    Pause,
    Resume,
    Cancel,
    Status,
    /// One-shot thermal profiling: measure the hotend's heat/cool rates and
    /// save them into the printer profile. Manual only — never part of slicing.
    CalibrateThermal,
}

/// What a finished host operation reports back to the UI thread.
enum HostReply {
    /// One-line outcome for the status line (test / pause / resume / cancel).
    Message(String),
    /// A Send / Send & print finished; success reveals the live-print overlay.
    SendDone { ok: bool, msg: String },
    /// A quiet interval poll feeding the live-print overlay — never touches
    /// the status line.
    Status(Result<printhost::PrintStatus, String>),
    /// A mid-operation progress line (thermal calibration streams these);
    /// updates the status line but keeps the operation running.
    Progress(String),
    /// Thermal calibration finished; on success the rates go to the printer
    /// profile via `persist_thermal_rates`.
    Rates(Result<printhost::ThermalRates, String>),
}

/// What the preview colors encode.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ColorBy {
    /// Feature type (walls, infill, …) — the classic view.
    Feature,
    /// Per-layer print time: red = quick layers, little cooling before the
    /// nozzle returns.
    LayerTime,
    /// Per-layer heat load: deposited energy ÷ (time × footprint). Red = lots
    /// of hot plastic delivered quickly to a small area.
    Heat,
    /// Planned nozzle temperature per layer — heat control's temp schedule
    /// made visible. One flat color = nothing scheduled.
    Temp,
}

/// Which derivable settings the user has pinned to manual values. Unpinned
/// fields recompute live from their master setting every frame (camera-style
/// "priority mode": auto until touched, visible either way).
#[derive(Default, Clone, Copy)]
struct Pins {
    outer_wall_accel: bool,
    first_layer_accel: bool,
    max_heat: bool,
}

/// A slider with an auto/pin badge: while unpinned it shows a weak "auto" tag
/// and tracks `derived` (the caller recomputes it each frame); dragging pins
/// it, and the ⟲ button returns it to auto.
fn auto_slider(
    ui: &mut egui::Ui,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
    label: &str,
    pinned: &mut bool,
    derived: f64,
    hover: &str,
) {
    ui.horizontal(|ui| {
        // Tooltip lives on the label only: popping help text while hovering
        // or dragging the slider itself would cover the value.
        let r = ui.add(egui::Slider::new(value, range));
        if r.changed() {
            *pinned = true;
        }
        ui.label(label).on_hover_text(hover);
        if *pinned {
            if ui
                .small_button("⟲")
                .on_hover_text(format!(
                    "Pinned manually. Click to return to auto ({derived:.2}) and follow the master setting again."
                ))
                .clicked()
            {
                *pinned = false;
                *value = derived;
            }
        } else {
            ui.label(egui::RichText::new("auto").small().weak())
                .on_hover_text("Following its master setting — drag the slider to pin a manual value.");
        }
    });
}

/// The flow-triangle ceiling spelled out with live numbers, for tooltips —
/// names every participant so the relationship is learnable from any corner:
/// "max flow 21.0 mm³/s ÷ bead (line width 0.45 × layer height 0.20 mm) = ~258 mm/s".
fn flow_ceiling_text(s: &config::Settings) -> String {
    let cap = config::flow_speed_cap_mm_s(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm);
    if cap.is_finite() {
        format!(
            "max flow {:.1} mm³/s ÷ bead (line width {:.2} × layer height {:.2} mm) = ~{:.0} mm/s",
            s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm, cap
        )
    } else {
        "max flow 0 = unlimited".into()
    }
}


/// A labelled slider whose hover help triggers on the label — and only the
/// label: tooltips over the slider itself would cover the value while
/// adjusting. (egui's built-in `.text()` label can't carry a tooltip at all —
/// it sits outside the slider's response.)
fn hslider(
    ui: &mut egui::Ui,
    enabled: bool,
    slider: egui::Slider<'_>,
    label: &str,
    hover: impl Into<egui::WidgetText>,
) -> egui::Response {
    ui.horizontal(|ui| {
        let r = ui.add_enabled(enabled, slider);
        ui.add_enabled(enabled, egui::Label::new(label)).on_hover_text(hover);
        r
    })
    .inner
}

/// `hslider` plus a lockout explanation shown while the row is disabled.
fn hslider_lockout(
    ui: &mut egui::Ui,
    enabled: bool,
    slider: egui::Slider<'_>,
    label: &str,
    hover: &str,
    disabled_hover: &str,
) -> egui::Response {
    ui.horizontal(|ui| {
        let r = ui.add_enabled(enabled, slider);
        ui.add_enabled(enabled, egui::Label::new(label))
            .on_hover_text(hover)
            .on_disabled_hover_text(disabled_hover);
        r
    })
    .inner
}

/// Fixed bounds of the heat-load color scale (mW/mm², log) — constant so any
/// two slices are visually comparable regardless of data or settings.
const HEAT_SCALE_LO_MW: f64 = 1.0;
const HEAT_SCALE_HI_MW: f64 = 40.0;

/// The default accent (brass): ONE hue drives every 3D-view color — the
/// model tint, the feature palette, and the heat ramps are all derived from
/// it (see `color_for` / `heat_ramp` / `mesh_tints`). The user picks any
/// color via the expandable picker in the panel; persisted in the state
/// dotfile as "#RRGGBB".
const DEFAULT_ACCENT: egui::Color32 = egui::Color32::from_rgb(216, 168, 82);

fn accent_to_hex(c: egui::Color32) -> String {
    format!("#{:02X}{:02X}{:02X}", c.r(), c.g(), c.b())
}

fn accent_from_hex(s: &str) -> Option<egui::Color32> {
    let s = s.trim().trim_start_matches('#');
    if s.len() != 6 {
        return None;
    }
    let v = u32::from_str_radix(s, 16).ok()?;
    Some(egui::Color32::from_rgb((v >> 16) as u8, (v >> 8) as u8, v as u8))
}

/// The accent as (hue°, saturation, lightness), with a saturation floor so
/// muted swatches still yield distinguishable derived palettes.
fn accent_hsl(c: egui::Color32) -> (f32, f32, f32) {
    let (r, g, b) = (c.r() as f32 / 255.0, c.g() as f32 / 255.0, c.b() as f32 / 255.0);
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let l = (max + min) / 2.0;
    let d = max - min;
    if d < 1e-6 {
        return (0.0, 0.35, l);
    }
    let s = d / (1.0 - (2.0 * l - 1.0).abs());
    let h = if max == r {
        60.0 * ((g - b) / d).rem_euclid(6.0)
    } else if max == g {
        60.0 * ((b - r) / d + 2.0)
    } else {
        60.0 * ((r - g) / d + 4.0)
    };
    (h, s.max(0.35), l)
}

fn hsl_to_rgb(h: f32, s: f32, l: f32) -> [f32; 3] {
    let h = h.rem_euclid(360.0) / 60.0;
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s.clamp(0.0, 1.0);
    let x = c * (1.0 - (h % 2.0 - 1.0).abs());
    let (r, g, b) = match h as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = l - c / 2.0;
    [r + m, g + m, b + m]
}

/// Model-mesh tints derived from the accent: unselected = the accent sunk
/// into porcelain, selected = the accent proper.
fn mesh_tints(accent: egui::Color32) -> ([f32; 3], [f32; 3]) {
    let (h, s, _) = accent_hsl(accent);
    (hsl_to_rgb(h, s * 0.22, 0.72), hsl_to_rgb(h, s * 0.85, 0.60))
}

/// Cool → hot ramp for the preview heat maps (u in 0..=1), riffed off the
/// accent: its hue glowing up from a dark cool-drifted shade, through the
/// accent itself, to a bright — but still saturated — top end (capped at
/// L 0.76 with the saturation held up, so the hot end reads as the accent
/// at full glow, never as white). Lightness is monotonic — dark = cool,
/// bright = hot — so the ramp stays ordered whichever hue drives it.
fn heat_ramp(u: f32, accent: (f32, f32, f32)) -> [f32; 3] {
    let (h, s, _) = accent;
    let u = u.clamp(0.0, 1.0);
    let hh = h - 20.0 + 30.0 * u;
    let ll = 0.24 + 0.52 * u;
    let arc = 1.0 - (u - 0.6).abs() * 0.9;
    let ss = (s * (0.50 + 0.70 * arc)).clamp(0.05, 0.95);
    hsl_to_rgb(hh, ss, ll)
}

/// Accent color per profile tier — used on the selector rows and on every
/// settings-section header, so it's visible at a glance which profile a
/// setting is saved to.
fn tier_color(kind: TierKind) -> egui::Color32 {
    // Dusty hues that keep their identities (blue/ochre/sage) but sit inside
    // the ink & cream world instead of shouting over it. Saturated just
    // enough to tell apart at dot size.
    match kind {
        TierKind::Printer => egui::Color32::from_rgb(124, 165, 215), // dusty steel
        TierKind::Filament => egui::Color32::from_rgb(228, 158, 72), // ochre
        TierKind::Process => egui::Color32::from_rgb(148, 192, 116), // sage
    }
}

/// Editorial section title: a small tier-colored dot, then the title as
/// tracked small caps in cream — print-like, with the tier as a quiet mark
/// instead of a colored headline.
fn section_title(title: &str, kind: TierKind) -> egui::text::LayoutJob {
    let mut job = egui::text::LayoutJob::default();
    // U+2022 bullet — present in egui's default fonts (U+25CF "●" is not,
    // and renders as a missing-glyph box).
    job.append(
        "•  ",
        0.0,
        egui::TextFormat {
            font_id: egui::FontId::proportional(18.0),
            color: tier_color(kind),
            valign: egui::Align::Center,
            ..Default::default()
        },
    );
    job.append(
        &title.to_uppercase(),
        0.0,
        egui::TextFormat {
            font_id: egui::FontId::proportional(11.0),
            color: palette::CREAM_DIM,
            extra_letter_spacing: 1.1,
            valign: egui::Align::Center,
            ..Default::default()
        },
    );
    job
}

/// A collapsible settings section owned by one profile tier: the header
/// carries the tier's dot and explains the mapping on hover.
fn tier_section(
    ui: &mut egui::Ui,
    title: &str,
    kind: TierKind,
    default_open: bool,
    add: impl FnOnce(&mut egui::Ui),
) {
    let header = egui::CollapsingHeader::new(section_title(title, kind))
        .default_open(default_open)
        .show(ui, add);
    header.header_response.on_hover_text(format!(
        "These settings are saved to the {} profile (dot-matched in the selector above).",
        kind.label()
    ));
    ui.add_space(2.0);
}

/// One object placed on the bed: shared mesh geometry plus an editable placement
/// (Euler rotation, uniform scale, and a bed-plane position for the footprint
/// center). The object always rests on z=0 — its baked transform drops it there.
struct SceneObject {
    name: String,
    mesh: Arc<mesh::Mesh>,
    /// Euler rotation in degrees, applied X then Y then Z.
    rot_deg: [f64; 3],
    scale: f64,
    /// Bed XY of the rotated/scaled footprint's center.
    pos: [f64; 2],
}

impl SceneObject {
    fn new(name: String, mesh: mesh::Mesh) -> Self {
        Self { name, mesh: Arc::new(mesh), rot_deg: [0.0; 3], scale: 1.0, pos: [0.0, 0.0] }
    }

    /// Footprint of the rotated+scaled mesh (no placement): (minx,miny,maxx,maxy,minz).
    fn footprint(&self) -> (f64, f64, f64, f64, f64) {
        let lin = mesh::Transform { rotation: euler_matrix(self.rot_deg), scale: self.scale, ..Default::default() };
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN, f64::MAX);
        for &v in &self.mesh.vertices {
            let p = lin.apply_linear(v);
            b.0 = b.0.min(p[0]);
            b.1 = b.1.min(p[1]);
            b.2 = b.2.max(p[0]);
            b.3 = b.3.max(p[1]);
            b.4 = b.4.min(p[2]);
        }
        b
    }

    /// Bake the placement into an affine transform: footprint centered on `pos`,
    /// bottom dropped to z=0.
    fn transform(&self) -> mesh::Transform {
        let (minx, miny, maxx, maxy, minz) = self.footprint();
        mesh::Transform {
            rotation: euler_matrix(self.rot_deg),
            scale: self.scale,
            translation: [self.pos[0] - (minx + maxx) / 2.0, self.pos[1] - (miny + maxy) / 2.0, -minz],
        }
    }
}

/// Rotation matrix for Euler angles (degrees), applied X then Y then Z (R = Rz·Ry·Rx).
fn euler_matrix(deg: [f64; 3]) -> [[f64; 3]; 3] {
    let (sx, cx) = deg[0].to_radians().sin_cos();
    let (sy, cy) = deg[1].to_radians().sin_cos();
    let (sz, cz) = deg[2].to_radians().sin_cos();
    [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]
}

fn main() -> eframe::Result<()> {
    // The window/taskbar icon: a Playfair "F" sliced into offset layers on a
    // cream tile (generated from the wordmark font; raw RGBA so no image
    // decoder is needed). Wayland ignores per-window icons by design — there
    // it comes from a .desktop file instead, when we ship one.
    let icon = egui::IconData {
        rgba: include_bytes!("../assets/icon.rgba").to_vec(),
        width: 128,
        height: 128,
    };
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([1024.0, 640.0])
            .with_icon(icon),
        ..Default::default()
    };
    eframe::run_native("Fable Slicer", options, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}

/// What the last slice produced — rendered, together with the one-line
/// `status` (imports, exports, printer ops), in the dismissable messages
/// pane floated over the viewport.
struct SliceSummary {
    layers: usize,
    toolpaths: usize,
    secs: f64,
    filament_m: f64,
    grams: f64,
    /// The filament's heat-load ceiling in force at slice time (mW/mm²).
    heat_target_mw: f64,
}

struct App {
    profiles: Profiles,
    printer: String,
    filament: String,
    process: String,
    /// Program state as last written to the dotfile folder — compared each
    /// frame, so every path that changes a selection persists it.
    saved_state: config::AppState,
    /// Where the STL import dialog last picked a file.
    last_model_dir: Option<std::path::PathBuf>,
    /// Where the g-code export dialog last saved.
    last_export_dir: Option<std::path::PathBuf>,
    settings: Settings,
    /// Settings as resolved from the selected profiles — panel edits are
    /// compared against this for the per-tier "modified" indicators.
    baseline: Settings,
    /// Open save/delete-profile dialog, if any.
    profile_dialog: Option<ProfileDialog>,
    /// Thermal calibration runs heaters and motion — the button arms this
    /// confirmation dialog instead of firing directly.
    confirm_calibration: bool,
    /// A profile switch requested while settings carry unsaved (*) edits —
    /// held here until the user confirms discarding them.
    pending_switch: Option<(String, String, String)>,
    /// Auto/pinned state of the derivable settings.
    pins: Pins,
    objects: Vec<SceneObject>,
    selected: Option<usize>,
    scene: Scene,
    camera: Camera,
    status: String,
    sliced: Option<Vec<engine::LayerPlan>>,
    /// Readable result block for the last slice; cleared with `sliced`.
    slice_summary: Option<SliceSummary>,
    /// Per-layer time/heat numbers behind the preview color modes.
    layer_stats: Vec<engine::LayerStats>,
    /// Per-island split of the heat numbers (parallel to `layer_stats`).
    layer_islands: Vec<engine::LayerIslands>,
    /// What the preview colors encode.
    color_by: ColorBy,
    /// The 3D view's accent: model tint, feature palette, and heat ramps are
    /// all derived from this one hue. Persisted in the state dotfile.
    accent: egui::Color32,
    /// Set while the accent picker is changing; the preview instance buffers
    /// re-bake once the pointer releases (not every drag frame).
    accent_rebake: bool,
    /// In-flight printer-host operation: its reply arrives here from the
    /// worker thread (one op at a time; buttons disable).
    host_rx: Option<std::sync::mpsc::Receiver<HostReply>>,
    /// True once a file has been sent this session — reveals the live-print
    /// card on the viewport. The card's ✖ clears it (which also stops the
    /// quiet status polls) until the next send.
    sent_to_printer: bool,
    /// Latest polled printer state (None until the first poll lands).
    printer_status: Option<Result<printhost::PrintStatus, String>>,
    /// When the last quiet status poll started (None = poll next frame).
    last_status_poll: Option<std::time::Instant>,
    /// Viewport rect of the live-print overlay (blocks camera input under it).
    print_overlay_rect: Option<egui::Rect>,
    /// Cumulative bead-instance count after each layer (for the layer slider).
    layer_ends: Vec<u32>,
    /// Cumulative joint-blob count after each layer.
    joint_layer_ends: Vec<u32>,
    /// false = show the model mesh; true = show the sliced toolpaths.
    view_preview: bool,
    /// Highest layer shown in preview (1-based).
    preview_layer: usize,
    show_walls: bool,
    show_solid: bool,
    show_infill: bool,
    show_skirt: bool,
    show_support: bool,
    show_travel: bool,
    show_seams: bool,
    show_gap_fill: bool,
    show_ironing: bool,
    needs_rebuild: bool,
    /// Re-frame the camera on the next rebuild (set on scene changes, not selection).
    refit_camera: bool,
    /// Move the orbit pivot to the bed center next frame — set on printer
    /// profile switches and bed-size edits. Runs after any refit, so the
    /// pivot wins; zoom and view angle stay put.
    recenter_camera: bool,
    /// Bed XY (mm) the scene and camera last saw — a change (bed sliders,
    /// printer switch, profile delete fallback) is detected by comparison
    /// each frame, wherever it came from.
    last_bed: (f64, f64),
    /// Object being dragged in the viewport (None = orbiting the camera).
    drag_obj: Option<usize>,
    /// Offset (bed XY) between the dragged object's pos and the cursor at grab time.
    drag_grab: [f64; 2],
    /// Screen rect of the transform overlay (so viewport input ignores clicks on it).
    overlay_rect: Option<egui::Rect>,
    /// Screen rect of the messages pane (same input-blocking purpose).
    msgs_overlay_rect: Option<egui::Rect>,
    /// Set when the messages pane is dismissed: the (status, slice generation)
    /// it was showing. The pane stays hidden while both still match — any new
    /// message or a fresh slice brings it back.
    msgs_dismissed: Option<(String, u64)>,
    /// Bumped on every slice so a re-slice re-shows a dismissed messages pane
    /// even when the visible text happens to be identical.
    slice_gen: u64,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // The wordmark's serif (Playfair Display, OFL — license alongside the
        // asset). Registered as its own family so nothing else picks it up.
        let mut fonts = egui::FontDefinitions::default();
        fonts.font_data.insert(
            "playfair".into(),
            egui::FontData::from_static(include_bytes!("../assets/PlayfairDisplay.ttf")).into(),
        );
        fonts
            .families
            .insert(egui::FontFamily::Name("wordmark".into()), vec!["playfair".into()]);
        cc.egui_ctx.set_fonts(fonts);
        apply_ink_cream(&cc.egui_ctx);

        let rs = cc
            .wgpu_render_state
            .as_ref()
            .expect("a wgpu render state (run with the wgpu backend)");
        let scene = Scene::new(rs);
        let mut profiles = Profiles::builtin();
        let mut status = "Open an STL to begin.".to_string();
        match profiles.load_user_profiles(None) {
            Ok(skipped) if !skipped.is_empty() => {
                status = format!(
                    "Skipped user profiles shadowing built-ins: {} (base on them with 'inherits' instead)",
                    skipped.join(", ")
                );
            }
            Err(e) => status = format!("User profiles: {e}"),
            _ => {}
        }
        // Restore the last session's selections from the dotfile state where
        // they still exist — a deleted profile falls back tier by tier, and a
        // restored triple that no longer resolves falls back entirely.
        let state = config::AppState::load();
        let accent = state.accent.as_deref().and_then(accent_from_hex).unwrap_or(DEFAULT_ACCENT);
        let pick = |saved: &str, names: Vec<&str>, default: &str| {
            if !saved.is_empty() && names.contains(&saved) {
                saved.to_string()
            } else {
                default.to_string()
            }
        };
        let (mut printer, mut filament, mut process) = (
            pick(&state.printer, profiles.printer_names(), "voron24"),
            pick(&state.filament, profiles.filament_names(), "pla"),
            pick(&state.process, profiles.process_names(), "standard"),
        );
        let mut settings = match profiles.resolve(&printer, &filament, &process) {
            Ok(s) => s,
            Err(_) => {
                (printer, filament, process) =
                    ("voron24".to_string(), "pla".to_string(), "standard".to_string());
                profiles.resolve(&printer, &filament, &process).unwrap_or_default()
            }
        };
        settings.auto_center_on_bed = false; // objects are placed explicitly on the bed
        let baseline = settings.clone();
        let pins = match (
            profiles.merged_process(&process),
            profiles.merged_printer(&printer),
            profiles.merged_filament(&filament),
        ) {
            (Ok(_pc), Ok(pr), Ok(fl)) => Pins {
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
                max_heat: fl.max_heat_mw_mm2.is_some(),
            },
            _ => Pins::default(),
        };
        let last_bed = (settings.bed_size_x_mm, settings.bed_size_y_mm);
        Self {
            profiles,
            printer,
            filament,
            process,
            // Seed with the file's content as-loaded: if the fallbacks above
            // corrected anything, the first persist pass rewrites the file.
            last_model_dir: state.last_model_dir.clone(),
            last_export_dir: state.last_export_dir.clone(),
            saved_state: state,
            accent,
            accent_rebake: false,
            settings,
            baseline,
            profile_dialog: None,
            confirm_calibration: false,
            pending_switch: None,
            pins,
            objects: Vec::new(),
            selected: None,
            scene,
            camera: Camera::new(),
            status,
            sliced: None,
            slice_summary: None,
            layer_stats: Vec::new(),
            layer_islands: Vec::new(),
            color_by: ColorBy::Feature,
            host_rx: None,
            sent_to_printer: false,
            printer_status: None,
            last_status_poll: None,
            print_overlay_rect: None,
            layer_ends: Vec::new(),
            joint_layer_ends: Vec::new(),
            view_preview: false,
            preview_layer: 1,
            show_walls: true,
            show_solid: true,
            show_infill: true,
            show_skirt: true,
            show_support: true,
            show_travel: false,
            show_seams: false,
            show_gap_fill: true,
            show_ironing: true,
            needs_rebuild: true,
            refit_camera: true,
            recenter_camera: false,
            last_bed,
            drag_obj: None,
            drag_grab: [0.0, 0.0],
            overlay_rect: None,
            msgs_overlay_rect: None,
            msgs_dismissed: None,
            slice_gen: 0,
        }
    }

    /// Index of the object whose surface the ray (world origin/dir) first hits.
    fn pick(&self, o: glam::Vec3, d: glam::Vec3) -> Option<usize> {
        let mut best: Option<(f32, usize)> = None;
        for (i, obj) in self.objects.iter().enumerate() {
            let t = obj.transform();
            let v = |p: [f64; 3]| {
                let q = t.apply(p);
                glam::Vec3::new(q[0] as f32, q[1] as f32, q[2] as f32)
            };
            for k in 0..obj.mesh.triangles.len() {
                let tri = obj.mesh.triangle(k);
                if let Some(dist) = ray_triangle(o, d, v(tri[0]), v(tri[1]), v(tri[2])) {
                    if best.map_or(true, |(bd, _)| dist < bd) {
                        best = Some((dist, i));
                    }
                }
            }
        }
        best.map(|(_, i)| i)
    }

    fn category_mask(&self) -> u32 {
        let mut m = 0u32;
        if self.show_skirt {
            m |= 1;
        }
        if self.show_walls {
            m |= 1 << 1;
        }
        if self.show_solid {
            m |= 1 << 2;
        }
        if self.show_infill {
            m |= 1 << 3;
        }
        if self.show_travel {
            m |= 1 << 4;
        }
        if self.show_seams {
            m |= 1 << 5;
        }
        if self.show_support {
            m |= 1 << 6;
        }
        if self.show_gap_fill {
            m |= 1 << 7;
        }
        if self.show_ironing {
            m |= 1 << 8;
        }
        m
    }

    fn reresolve(&mut self) {
        if let Ok(s) = self.profiles.resolve(&self.printer, &self.filament, &self.process) {
            self.settings = s;
            self.settings.auto_center_on_bed = false;
            self.baseline = self.settings.clone();
            self.sliced = None;
            self.slice_summary = None;
            self.view_preview = false;
            self.needs_rebuild = true;
            self.refit_camera = true;
        }
        self.refresh_pins();
    }

    /// Pin state comes from the selected profiles: a field the profile chain
    /// sets explicitly is pinned; one it leaves unset follows auto.
    fn refresh_pins(&mut self) {
        if let (Ok(_pc), Ok(pr), Ok(fl)) = (
            self.profiles.merged_process(&self.process),
            self.profiles.merged_printer(&self.printer),
            self.profiles.merged_filament(&self.filament),
        ) {
            self.pins = Pins {
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
                max_heat: fl.max_heat_mw_mm2.is_some(),
            };
        }
    }

    /// Recompute every unpinned derivable setting from its master, so dragging
    /// print speed (or changing the nozzle) visibly moves its dependents.
    fn apply_auto(&mut self) {
        let s = &mut self.settings;
        // The data-driven chain: bead from the nozzle, temperatures from the
        // packaging range + bias, nominal speed from the machine rating ×
        // the finish↔speed dial, features from the nominal under the melt
        // ceiling. No speed or temperature is a slider anywhere.
        s.line_width_mm = config::derived_line_width_mm(s.nozzle_diameter_mm);
        s.nozzle_temp_c =
            config::derived_nozzle_temp_c(s.nozzle_temp_min_c, s.nozzle_temp_max_c, s.temp_bias);
        s.first_layer_nozzle_temp_c = config::derived_first_layer_temp_c(
            s.nozzle_temp_min_c,
            s.nozzle_temp_max_c,
            s.temp_bias,
            s.material,
        );
        if !self.pins.max_heat {
            s.max_heat_mw_mm2 = s.material.max_heat_mw_mm2();
        }
        s.print_speed_mm_s = config::derived_print_speed_mm_s(s.machine_speed_mm_s, s.speed_quality);
        let cap = config::flow_speed_cap_mm_s(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm);
        s.external_perimeter_speed_mm_s =
            config::derived_external_perimeter_speed_mm_s(s.print_speed_mm_s, cap);
        s.solid_speed_mm_s = config::derived_solid_speed_mm_s(s.print_speed_mm_s, cap);
        s.support_speed_mm_s = config::derived_support_speed_mm_s(s.print_speed_mm_s, cap);
        s.gap_fill_speed_mm_s = config::derived_gap_fill_speed_mm_s(s.print_speed_mm_s, cap);
        s.overhang_speed_mm_s = config::derived_overhang_speed_mm_s(s.bridge_speed_mm_s);
        if !self.pins.outer_wall_accel {
            s.outer_wall_accel_mm_s2 = config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2);
        }
        if !self.pins.first_layer_accel {
            s.first_layer_accel_mm_s2 = config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2);
        }
    }

    /// Strip unpinned auto fields from a process diff: auto values are derived,
    /// not chosen, so they're never saved (and never count as dirty).
    /// Strip unpinned auto fields from a filament diff.
    fn mask_auto_filament(&self, fl: &mut FilamentProfile) {
        if !self.pins.max_heat {
            fl.max_heat_mw_mm2 = None;
        }
    }

    /// Printer-tier counterpart of `mask_auto`.
    fn mask_auto_printer(&self, pr: &mut PrinterProfile) {
        if !self.pins.outer_wall_accel {
            pr.outer_wall_accel = None;
        }
        if !self.pins.first_layer_accel {
            pr.first_layer_accel = None;
        }
    }

    /// Per-tier dirty flags vs. the baseline, ignoring unpinned auto fields.
    fn tier_dirty_masked(&self) -> [bool; 3] {
        let mut pr = PrinterProfile::diff(&self.settings, &self.baseline);
        self.mask_auto_printer(&mut pr);
        let mut fl = FilamentProfile::diff(&self.settings, &self.baseline);
        self.mask_auto_filament(&mut fl);
        let pc = ProcessProfile::diff(&self.settings, &self.baseline);
        [!pr.is_empty(), !fl.is_empty(), !pc.is_empty()]
    }

    /// Re-resolve only the dirty baseline (after a save) — keeps the user's
    /// current panel edits in other tiers intact.
    fn refresh_baseline(&mut self) {
        if let Ok(mut b) = self.profiles.resolve(&self.printer, &self.filament, &self.process) {
            b.auto_center_on_bed = false;
            self.baseline = b;
        }
    }

    /// Save the current settings' diff as a user profile named `name` in `kind`.
    ///
    /// New name: the profile inherits the currently selected one and stores only
    /// the changed fields. Same name (overwriting a user profile): the new diff
    /// is merged over the stored fields and the original parent is kept.
    fn save_profile(&mut self, kind: TierKind, name: &str) -> Result<(), String> {
        match kind {
            TierKind::Printer => {
                let mut diff = PrinterProfile::diff(&self.settings, &self.baseline);
                self.mask_auto_printer(&mut diff);
                if name == self.printer && self.profiles.is_user(kind, name) {
                    let existing = self.profiles.get_printer(name).cloned().unwrap_or_default();
                    let parent = existing.parent().map(str::to_string);
                    diff = diff.over(existing);
                    diff.inherits = parent;
                } else {
                    diff.inherits = Some(self.printer.clone());
                }
                self.profiles.save_user_printer(name, diff)?;
                self.printer = name.to_string();
            }
            TierKind::Filament => {
                let mut diff = FilamentProfile::diff(&self.settings, &self.baseline);
                self.mask_auto_filament(&mut diff);
                if name == self.filament && self.profiles.is_user(kind, name) {
                    let existing = self.profiles.get_filament(name).cloned().unwrap_or_default();
                    let parent = existing.parent().map(str::to_string);
                    diff = diff.over(existing);
                    diff.inherits = parent;
                } else {
                    diff.inherits = Some(self.filament.clone());
                }
                self.profiles.save_user_filament(name, diff)?;
                self.filament = name.to_string();
            }
            TierKind::Process => {
                let mut diff = ProcessProfile::diff(&self.settings, &self.baseline);
                if name == self.process && self.profiles.is_user(kind, name) {
                    let existing = self.profiles.get_process(name).cloned().unwrap_or_default();
                    let parent = existing.parent().map(str::to_string);
                    diff = diff.over(existing);
                    diff.inherits = parent;
                } else {
                    diff.inherits = Some(self.process.clone());
                }
                self.profiles.save_user_process(name, diff)?;
                self.process = name.to_string();
            }
        }
        self.refresh_baseline();
        Ok(())
    }

    /// Measured thermal rates land in the live settings *and* the printer
    /// profile on disk — that is the whole point of profiling once: every
    /// future slice reads them. A selected user profile is updated in place;
    /// a built-in (read-only) gets/extends its `<name>-custom` copy and the
    /// selection moves there. Returns the status-line message.
    fn persist_thermal_rates(&mut self, r: printhost::ThermalRates) -> String {
        self.settings.heat_rate_c_s = r.heat_rate_c_s;
        self.settings.cool_rate_c_s = r.cool_rate_c_s;
        self.settings.heat_rate_fan_c_s = r.heat_rate_fan_c_s;
        self.settings.cool_rate_fan_c_s = r.cool_rate_fan_c_s;
        let sel = self.printer.clone();
        let name = if self.profiles.is_user(TierKind::Printer, &sel) {
            sel.clone()
        } else {
            format!("{sel}-custom")
        };
        let mut pr = self.profiles.get_printer(&name).cloned().unwrap_or_default();
        if pr.inherits.is_none() && name != sel {
            pr.inherits = Some(sel.clone());
        }
        pr.heat_rate_c_s = Some(r.heat_rate_c_s);
        pr.cool_rate_c_s = Some(r.cool_rate_c_s);
        pr.heat_rate_fan_c_s = Some(r.heat_rate_fan_c_s);
        pr.cool_rate_fan_c_s = Some(r.cool_rate_fan_c_s);
        let summary = format!(
            "heating ≈{:.1}/{:.1} °C/s, cooling ≈{:.1}/{:.1} °C/s (fan off/on)",
            r.heat_rate_c_s, r.heat_rate_fan_c_s, r.cool_rate_c_s, r.cool_rate_fan_c_s
        );
        match self.profiles.save_user_printer(&name, pr) {
            Ok(()) => {
                self.printer = name.clone();
                self.refresh_baseline();
                format!("Thermal response: {summary} — saved to printer profile '{name}'.")
            }
            Err(e) => format!("Measured {summary} (set for this session), but saving the profile failed: {e}"),
        }
    }

    /// The temperatures and pose thermal calibration will use — shared by the
    /// confirmation dialog (which describes them) and the runner, so the
    /// prompt can never drift from what actually happens.
    fn calibration_plan(&self) -> (f64, f64, (f64, f64, f64)) {
        let base = self.settings.nozzle_temp_c as f64;
        let step = (self.settings.nozzle_temp_max_c as f64 - base).min(20.0).max(10.0);
        let park = (
            self.settings.bed_size_x_mm / 2.0,
            self.settings.bed_size_y_mm / 2.0,
            (self.settings.bed_size_z_mm * 0.5).clamp(20.0, 50.0),
        );
        (base, base + step, park)
    }

    /// Kick off the one-shot thermal profiling on a worker thread: steps the
    /// idle hotend around the printing temperature over Moonraker and fits
    /// the heat/cool rates. Progress streams into the status line.
    fn spawn_thermal_calibration(&mut self, ctx: &egui::Context) {
        let host = self.settings.host_url.trim().to_string();
        if host.is_empty() {
            self.status = "No printer host configured (Connection section).".into();
            return;
        }
        let client = printhost::Client::new(&host, &self.settings.api_key);
        let (base, top, park) = self.calibration_plan();
        let step = top - base;
        // A pinned, repeatable pose: bed center, high enough to clear a part
        // left on the plate; bed controlled at the filament's temperature.
        let setup = printhost::CalibrationSetup {
            park_xyz: park,
            bed_c: self.settings.bed_temp_c as f64,
        };
        let (tx, rx) = std::sync::mpsc::channel();
        self.host_rx = Some(rx);
        self.status = "Thermal calibration: contacting printer…".into();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let mut progress = |m: String| {
                let _ = tx.send(HostReply::Progress(m));
                ctx.request_repaint();
            };
            let res = printhost::measure_thermal_rates(&client, base, step, &setup, &mut progress);
            let _ = tx.send(HostReply::Rates(res));
            ctx.request_repaint();
        });
    }

    /// Delete a user profile; the selection falls back to a built-in default.
    fn delete_profile(&mut self, kind: TierKind, name: &str) -> Result<(), String> {
        self.profiles.delete_user(kind, name)?;
        let sel = match kind {
            TierKind::Printer => &mut self.printer,
            TierKind::Filament => &mut self.filament,
            TierKind::Process => &mut self.process,
        };
        if sel == name {
            *sel = match kind {
                TierKind::Printer => "generic".to_string(),
                TierKind::Filament => "pla".to_string(),
                TierKind::Process => "standard".to_string(),
            };
            self.refresh_baseline();
        }
        Ok(())
    }

    /// Load an STL and add it to the scene as a new object.
    fn import_model(&mut self, path: std::path::PathBuf) {
        let file = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "object".into());
        let is_3mf = path
            .extension()
            .map(|e| e.eq_ignore_ascii_case("3mf"))
            .unwrap_or(false);
        if is_3mf {
            // A 3MF build can carry several objects — each becomes its own
            // scene object (named from the file, or its own name), and the
            // grid arrange in after_scene_change lays the plate out.
            match mesh::load_3mf(&path) {
                Ok(items) if items.is_empty() => {
                    self.status = format!("{file}: no printable objects in the build");
                }
                Ok(items) => {
                    let n = items.len();
                    let tris: usize = items.iter().map(|it| it.mesh.triangles.len()).sum();
                    for (k, it) in items.into_iter().enumerate() {
                        let name = if !it.name.is_empty() {
                            it.name
                        } else if n == 1 {
                            file.clone()
                        } else {
                            format!("{file} #{}", k + 1)
                        };
                        let mut obj = SceneObject::new(name, it.mesh);
                        // Keep the file's plate layout: pos = the baked
                        // footprint center reproduces the build placement
                        // (SceneObject::transform recenters the footprint
                        // on pos).
                        let (minx, miny, maxx, maxy, _) = obj.footprint();
                        obj.pos = [(minx + maxx) / 2.0, (miny + maxy) / 2.0];
                        self.objects.push(obj);
                    }
                    self.selected = Some(self.objects.len() - 1);
                    self.status = if n == 1 {
                        format!("Imported {file} ({tris} triangles)")
                    } else {
                        format!("Imported {file}: {n} objects ({tris} triangles)")
                    };
                    // The build's own layout wins while it fits our bed
                    // (it was arranged for *some* plate); re-grid only when
                    // something hangs off.
                    let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
                    let off_bed = self.objects.iter().any(|o| {
                        let (minx, miny, maxx, maxy, _) = o.footprint();
                        let c = o.pos;
                        let (w, h) = ((maxx - minx) / 2.0, (maxy - miny) / 2.0);
                        c[0] - w < 0.0 || c[1] - h < 0.0 || c[0] + w > bx || c[1] + h > by
                    });
                    if off_bed {
                        self.after_scene_change();
                    } else {
                        self.sliced = None;
                        self.slice_summary = None;
                        self.view_preview = false;
                        self.needs_rebuild = true;
                        self.refit_camera = true;
                    }
                }
                Err(e) => self.status = format!("Load failed: {e}"),
            }
            return;
        }
        match mesh::Mesh::load_stl(&path) {
            Ok(m) => {
                self.status = format!("Imported {file} ({} triangles)", m.triangles.len());
                self.objects.push(SceneObject::new(file, m));
                self.selected = Some(self.objects.len() - 1);
                self.after_scene_change();
            }
            Err(e) => self.status = format!("Load failed: {e}"),
        }
    }

    fn duplicate_selected(&mut self) {
        let Some(i) = self.selected else { return };
        let src = &self.objects[i];
        let copy = SceneObject {
            name: format!("{} copy", src.name),
            mesh: Arc::clone(&src.mesh),
            rot_deg: src.rot_deg,
            scale: src.scale,
            pos: src.pos,
        };
        self.objects.push(copy);
        self.selected = Some(self.objects.len() - 1);
        self.after_scene_change();
    }

    fn delete_selected(&mut self) {
        let Some(i) = self.selected else { return };
        self.objects.remove(i);
        self.selected = if self.objects.is_empty() {
            None
        } else {
            Some(i.min(self.objects.len() - 1))
        };
        self.after_scene_change();
    }

    /// Invalidate slice/preview and re-layout after the object set changed.
    fn after_scene_change(&mut self) {
        self.arrange();
        self.sliced = None;
        self.slice_summary = None;
        self.view_preview = false;
        self.needs_rebuild = true;
        self.refit_camera = true;
    }

    /// Lay all objects out in a grid centered on the bed (each footprint centered
    /// in its cell). Objects always sit on z=0 via their baked transform.
    fn arrange(&mut self) {
        let n = self.objects.len();
        if n == 0 {
            return;
        }
        let foot: Vec<(f64, f64, f64, f64, f64)> = self.objects.iter().map(SceneObject::footprint).collect();
        let cell_w = foot.iter().map(|f| f.2 - f.0).fold(0.0, f64::max) + 5.0;
        let cell_h = foot.iter().map(|f| f.3 - f.1).fold(0.0, f64::max) + 5.0;
        let cols = (n as f64).sqrt().ceil() as usize;
        let rows = n.div_ceil(cols);
        let x0 = self.settings.bed_size_x_mm / 2.0 - cols as f64 * cell_w / 2.0;
        let y0 = self.settings.bed_size_y_mm / 2.0 - rows as f64 * cell_h / 2.0;
        for (i, obj) in self.objects.iter_mut().enumerate() {
            obj.pos = [
                x0 + (i % cols) as f64 * cell_w + cell_w / 2.0,
                y0 + (i / cols) as f64 * cell_h + cell_h / 2.0,
            ];
        }
    }

    /// Bake every object's placement into one mesh, in bed coordinates.
    fn combined_mesh(&self) -> Option<mesh::Mesh> {
        if self.objects.is_empty() {
            return None;
        }
        let mut tris: Vec<[[f64; 3]; 3]> = Vec::new();
        for obj in &self.objects {
            let t = obj.transform();
            for i in 0..obj.mesh.triangles.len() {
                let tri = obj.mesh.triangle(i);
                tris.push([t.apply(tri[0]), t.apply(tri[1]), t.apply(tri[2])]);
            }
        }
        Some(mesh::Mesh::from_triangle_soup(&tris))
    }

    fn slice(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let Some(m) = self.combined_mesh() else {
            return;
        };
        let layers = generate(&m, &self.settings);
        let n = layers.len();
        let paths: usize = layers.iter().map(|l| l.paths.len()).sum();
        let secs = engine::estimate_seconds(&layers, &self.settings);
        let (fil_mm, grams) = engine::estimate_filament(&layers, &self.settings);
        self.slice_summary = Some(SliceSummary {
            layers: n,
            toolpaths: paths,
            secs,
            filament_m: fil_mm / 1000.0,
            grams,
            heat_target_mw: engine::effective_heat_target(&layers, &self.settings) * 1e3,
        });
        self.status.clear();
        self.slice_gen += 1;
        self.layer_stats = engine::per_layer_stats(&layers, &self.settings);
        self.layer_islands = engine::per_layer_islands(&layers, &self.settings);
        self.sliced = Some(layers);
        self.set_preview_instances(rs);
        self.preview_layer = n.max(1);
        self.view_preview = true;
    }

    /// (Re)build the preview bead instances from the sliced layers, colored
    /// per the active mode. Called after slicing and when the mode changes.
    fn set_preview_instances(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let Some(layers) = self.sliced.as_ref() else { return };
        // Match the emitter's brick-aware hop height so preview travels line up.
        let hop = if self.settings.brick_layers {
            self.settings.z_hop_mm.max(self.settings.layer_height_mm + 0.25)
        } else {
            self.settings.z_hop_mm
        };
        let layer_colors = self.layer_color_table();
        let (verts, ends, joints, joint_ends) =
            build_instances(layers, hop as f32, layer_colors.as_deref(), accent_hsl(self.accent));
        self.scene.set_toolpaths(&rs.device, &verts);
        self.scene.set_joints(&rs.device, &joints);
        self.layer_ends = ends;
        self.joint_layer_ends = joint_ends;
    }

    /// The active metric mapped to ramp colors, per path — or None in feature
    /// mode (`build_instances` then colors by path kind). Layer time is one
    /// color per layer broadcast to its paths; heat load is scored per island,
    /// so a skinny chimney can't hide inside a big layer's average.
    fn layer_color_table(&self) -> Option<Vec<Vec<[f32; 3]>>> {
        if self.color_by == ColorBy::Feature || self.layer_stats.is_empty() {
            return None;
        }
        let layers = self.sliced.as_ref()?;
        let acc = accent_hsl(self.accent);
        match self.color_by {
            ColorBy::LayerTime => {
                let logs: Vec<f64> = self.layer_stats.iter().map(|st| st.secs.max(1e-12).ln()).collect();
                let lo = logs.iter().cloned().fold(f64::INFINITY, f64::min);
                let hi = logs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let span = (hi - lo).max(1e-12);
                Some(
                    layers
                        .iter()
                        .zip(&logs)
                        .map(|(layer, &l)| {
                            // Inverted: short layers = least cooling = the hot end.
                            let u = 1.0 - ((l - lo) / span) as f32;
                            vec![heat_ramp(u, acc); layer.paths.len()]
                        })
                        .collect(),
                )
            }
            ColorBy::Temp => {
                // The temp schedule itself: a diverging map centered on the
                // printing temperature — cooler zones toward blue, warmer
                // toward red. (The first layer's adhesion temperature is its
                // own mechanism and stays out of this view.)
                let base = self.settings.nozzle_temp_c as f64;
                let span = layers
                    .iter()
                    .filter_map(|l| l.planned_temp_c)
                    .map(|t| (t - base).abs())
                    .fold(2.0_f64, f64::max);
                Some(
                    layers
                        .iter()
                        .map(|layer| {
                            let t = layer.planned_temp_c.unwrap_or(base);
                            let u = 0.5 + ((t - base) / (2.0 * span)) as f32;
                            vec![heat_ramp(u, acc); layer.paths.len()]
                        })
                        .collect(),
                )
            }
            ColorBy::Heat => {
                if self.layer_islands.len() != layers.len() {
                    return None;
                }
                // FIXED absolute log scale: colors mean the same thing in
                // every slice regardless of settings, so before/after slider
                // comparisons are honest. (Both the relative and the
                // target-anchored ramps re-stretched with the data/settings —
                // two screenshots of different configs were incomparable,
                // which misled real tuning.)
                let lo = HEAT_SCALE_LO_MW.ln();
                let span = HEAT_SCALE_HI_MW.ln() - lo;
                Some(
                    self.layer_islands
                        .iter()
                        .enumerate()
                        .map(|(li, l)| {
                            l.path_island
                                .iter()
                                .map(|&k| {
                                    let q = (self.island_heat(li, &l.islands[k]) * 1e3).max(1e-6).ln();
                                    heat_ramp((((q - lo) / span).clamp(0.0, 1.0)) as f32, acc)
                                })
                                .collect()
                        })
                        .collect(),
                )
            }
            ColorBy::Feature => unreachable!(),
        }
    }

    /// Heat-load metric for one island (W/mm²): its deposited energy over the
    /// whole layer's time (an island keeps cooling while the nozzle is
    /// elsewhere) and its own footprint.
    fn island_heat(&self, li: usize, isl: &engine::IslandStats) -> f64 {
        isl.joules / (self.layer_stats[li].secs.max(1e-9) * isl.footprint_mm2.max(1e-9))
    }

    /// The one-line status plus the last slice's summary — the body of the
    /// dismissable messages pane floated over the viewport.
    fn slice_messages(&self, ui: &mut egui::Ui) {
        if !self.status.is_empty() {
            ui.label(&self.status);
        }
        if let Some(sum) = &self.slice_summary {
            ui.label(format!("Sliced: {} layers, {} toolpaths", sum.layers, sum.toolpaths))
                .on_hover_text("Toolpaths = individual extrusion paths (walls, infill, …) across all layers.");
            ui.label(format!(
                "~{} · {:.2} m / {:.0} g filament",
                engine::format_duration(sum.secs),
                sum.filament_m,
                sum.grams
            ))
            .on_hover_text("Estimated print time and filament length / weight.");
        }
    }

    fn export(&mut self) {
        let Some(layers) = self.sliced.as_ref() else { return };
        let mut dialog = rfd::FileDialog::new()
            .add_filter("g-code", &["gcode"])
            .set_file_name("out.gcode");
        if let Some(dir) = &self.last_export_dir {
            dialog = dialog.set_directory(dir);
        }
        let Some(path) = dialog.save_file() else {
            return;
        };
        self.last_export_dir = path.parent().map(|d| d.to_path_buf());
        let gcode = engine::to_gcode(layers, &self.settings);
        self.status = match std::fs::write(&path, gcode) {
            Ok(()) => format!("Wrote {}", path.display()),
            Err(e) => format!("Write failed: {e}"),
        };
    }

    /// Write the program state to the dotfile folder when it changed —
    /// convenience memory only, so a failed save never blocks anything.
    fn persist_state(&mut self) {
        let cur = config::AppState {
            printer: self.printer.clone(),
            filament: self.filament.clone(),
            process: self.process.clone(),
            last_model_dir: self.last_model_dir.clone(),
            last_export_dir: self.last_export_dir.clone(),
            accent: Some(accent_to_hex(self.accent)),
        };
        if cur != self.saved_state {
            if let Err(e) = cur.save() {
                eprintln!("warning: program state not saved: {e}");
            }
            self.saved_state = cur;
        }
    }

    /// Upload filename: the first object's name with a .gcode extension.
    fn upload_filename(&self) -> String {
        let base = self
            .objects
            .first()
            .map(|o| {
                o.name
                    .trim_end_matches(".stl")
                    .trim_end_matches(".STL")
                    .trim_end_matches(".3mf")
                    .trim_end_matches(".3MF")
                    .to_string()
            })
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "print".into());
        format!("{base}.gcode")
    }

    /// Run a printer-host operation on a worker thread; its reply lands in
    /// `host_rx`. One at a time — callers disable while busy. `quiet` skips
    /// the "Contacting printer…" status (interval polls would spam it).
    fn spawn_host_op(
        &mut self,
        ctx: &egui::Context,
        quiet: bool,
        op: impl FnOnce(&printhost::Client) -> HostReply + Send + 'static,
    ) {
        let host = self.settings.host_url.trim().to_string();
        if host.is_empty() {
            self.status = "No printer host configured (Connection section).".into();
            return;
        }
        let client = printhost::Client::new(&host, &self.settings.api_key);
        let (tx, rx) = std::sync::mpsc::channel();
        self.host_rx = Some(rx);
        if !quiet {
            self.status = "Contacting printer…".into();
        }
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let _ = tx.send(op(&client));
            ctx.request_repaint();
        });
    }

    fn rebuild_scene(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let bx = self.settings.bed_size_x_mm as f32;
        let by = self.settings.bed_size_y_mm as f32;
        self.scene.set_bed(&rs.device, bx, by);

        // The selected object is flagged so the renderer highlights it.
        let objs: Vec<(&mesh::Mesh, mesh::Transform, bool)> = self
            .objects
            .iter()
            .enumerate()
            .map(|(i, o)| (o.mesh.as_ref(), o.transform(), self.selected == Some(i)))
            .collect();
        let bounds = self.scene.set_mesh(&rs.device, &objs);
        // Only re-frame on scene changes (import/duplicate/delete/arrange/profile),
        // not when the user merely selects an object.
        if self.refit_camera {
            match bounds {
                Some((lo, hi)) => {
                    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(hi[2] - lo[2]);
                    self.camera.frame(
                        glam::Vec3::new((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0),
                        span * 0.5 + 1.0,
                    );
                }
                None => self.camera.frame(glam::Vec3::new(bx / 2.0, by / 2.0, 0.0), bx.max(by) * 0.5),
            }
            self.refit_camera = false;
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let rs = frame.wgpu_render_state().expect("wgpu render state").clone();

        // A bed-size change — slider edit, printer switch, profile delete
        // fallback — refreshes the bed mesh and re-pivots the view on the new
        // plate, whatever path it arrived by.
        let bed = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
        if bed != self.last_bed {
            self.last_bed = bed;
            self.needs_rebuild = true;
            self.recenter_camera = true;
            // Objects placed for the old plate may not fit the new one —
            // re-grid them onto it. Only when something actually hangs off:
            // manual placements that still fit are left alone. (Without this,
            // a 350→152 mm switch leaves the model far outside the plate and
            // the freshly recentered pivot FEELS broken — you orbit an empty
            // bed while the part sweeps in the distance.)
            let off_bed = self.objects.iter().any(|o| {
                let (minx, miny, maxx, maxy, _) = o.footprint();
                minx < 0.0 || miny < 0.0 || maxx > bed.0 || maxy > bed.1
            });
            if off_bed {
                self.arrange();
                self.sliced = None;
                self.slice_summary = None;
                self.view_preview = false;
                self.refit_camera = true;
            }
        }
        if self.needs_rebuild {
            self.rebuild_scene(&rs);
            self.needs_rebuild = false;
        }
        // After any refit so the plate-center pivot wins; distance and angles
        // are untouched (it's a pivot move, not a re-frame).
        if self.recenter_camera {
            self.recenter_camera = false;
            self.camera.target =
                glam::Vec3::new((bed.0 / 2.0) as f32, (bed.1 / 2.0) as f32, 0.0);
        }
        // Unpinned auto settings track their masters every frame, before
        // anything (incl. the Slice button) reads them.
        self.apply_auto();

        // 320 wide fits the longest slider row (90 slider + value + 19-char
        // A printer-host operation reports back; quiet status polls feed the
        // live-print overlay, thermal calibration streams Progress lines (the
        // channel stays open), everything else lands in the status line.
        let mut rates: Option<Result<printhost::ThermalRates, String>> = None;
        let mut op_done = false;
        if let Some(rx) = &self.host_rx {
            while let Ok(reply) = rx.try_recv() {
                match reply {
                    HostReply::Progress(msg) => self.status = msg,
                    HostReply::Message(msg) => {
                        self.status = msg;
                        // Pause/resume/cancel just changed the printer's state:
                        // refresh the overlay promptly.
                        self.last_status_poll = None;
                        op_done = true;
                    }
                    HostReply::SendDone { ok, msg } => {
                        self.status = msg;
                        if ok {
                            self.sent_to_printer = true;
                            self.last_status_poll = None;
                        }
                        op_done = true;
                    }
                    HostReply::Status(st) => {
                        self.printer_status = Some(st);
                        op_done = true;
                    }
                    HostReply::Rates(r) => {
                        rates = Some(r);
                        op_done = true;
                    }
                }
            }
        }
        if op_done {
            self.host_rx = None;
        }
        match rates {
            Some(Ok(r)) => self.status = self.persist_thermal_rates(r),
            Some(Err(e)) => self.status = format!("Thermal calibration failed: {e}"),
            None => {}
        }
        // Host actions requested from inside the panel closure (which borrows
        // settings) run after it returns.
        let mut host_op: Option<HostOp> = None;
        let host_busy = self.host_rx.is_some();
        let host_set = !self.settings.host_url.trim().is_empty();
        // The live-print overlay keeps itself fresh with quiet polls — brisk
        // while printing, relaxed once idle/finished. No manual status button.
        if self.sent_to_printer && host_set {
            let interval = match &self.printer_status {
                Some(Ok(st)) if st.state == "printing" || st.state == "paused" => 2.0,
                None => 2.0, // first reading after a send
                _ => 10.0,
            };
            if !host_busy && self.last_status_poll.map_or(true, |t| t.elapsed().as_secs_f64() >= interval) {
                host_op = Some(HostOp::Status);
            }
            // egui only repaints on input; keep frames coming for the timer.
            ui.ctx().request_repaint_after(std::time::Duration::from_secs(1));
        }

        // label + auto badge ≈ 287). Content wider than the panel doesn't just
        // clip: egui reserves the overflowed width, pushing the central panel
        // right and leaving an unpainted band between the two (egui #4475) —
        // if a future row overflows, that band is the symptom to look for.
        egui::Panel::left("controls")
            .resizable(false)
            .exact_size(320.0)
            .frame(
                egui::Frame::new()
                    .fill(palette::INK)
                    .inner_margin(egui::Margin { left: 12, right: 12, top: 10, bottom: 6 }),
            )
            .show_inside(ui, |ui| {
            ui.spacing_mut().slider_width = 90.0;
            // The wordmark, after the Fable model's own branding: a classic
            // high-contrast serif in near-monochrome ink — warm paper cream
            // with only a whisper of blush across "Fable" — paired with a
            // small tracked sans "Slicer", serif-name / sans-subtitle.
            // Painted as two galleys so "Slicer" can sit a precise few pixels
            // above the serif row's descent-heavy bottom — LayoutJob's valign
            // stops (bottom / center) bracket the right spot but miss it.
            const SLICER_RAISE_PX: f32 = 3.5;
            let serif = egui::FontFamily::Name("wordmark".into());
            let wordmark_px = 30.0;
            let ink = |t: f32| {
                let lerp = |a: f32, b: f32| (a + (b - a) * t) as u8;
                egui::Color32::from_rgb(lerp(242.0, 230.0), lerp(236.0, 212.0), lerp(222.0, 226.0))
            };
            let wordmark_fmt = |color: egui::Color32| egui::TextFormat {
                font_id: egui::FontId::new(wordmark_px, serif.clone()),
                color,
                ..Default::default()
            };
            // Three galleys: the full "Fable" (sizing + where "able" lands,
            // so any cross-glyph kerning is preserved), plus a lone "F" and
            // the tail "able" — the F is painted sliced, like the icon.
            let mut fable_job = egui::text::LayoutJob::default();
            let mut f_job = egui::text::LayoutJob::default();
            let mut able_job = egui::text::LayoutJob::default();
            let fable: Vec<char> = "Fable".chars().collect();
            for (i, ch) in fable.iter().enumerate() {
                let t = i as f32 / (fable.len() - 1) as f32;
                fable_job.append(&ch.to_string(), 0.0, wordmark_fmt(ink(t)));
                if i == 0 {
                    f_job.append(&ch.to_string(), 0.0, wordmark_fmt(ink(t)));
                } else {
                    able_job.append(&ch.to_string(), 0.0, wordmark_fmt(ink(t)));
                }
            }
            let mut slicer_job = egui::text::LayoutJob::default();
            slicer_job.append(
                "Slicer",
                0.0,
                egui::TextFormat {
                    font_id: egui::FontId::proportional(20.0),
                    color: palette::CREAM_FAINT,
                    extra_letter_spacing: 1.4,
                    ..Default::default()
                },
            );
            let fable_galley = ui.ctx().fonts_mut(|f| f.layout_job(fable_job));
            let f_galley = ui.ctx().fonts_mut(|f| f.layout_job(f_job));
            let able_galley = ui.ctx().fonts_mut(|f| f.layout_job(able_job));
            let slicer_galley = ui.ctx().fonts_mut(|f| f.layout_job(slicer_job));
            let gap = 9.0;
            let size = egui::vec2(
                fable_galley.size().x + gap + slicer_galley.size().x,
                fable_galley.size().y,
            );
            let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
            let slicer_pos = egui::pos2(
                rect.min.x + fable_galley.size().x + gap,
                rect.max.y - slicer_galley.size().y - SLICER_RAISE_PX,
            );
            // The F gets the icon's treatment (it IS the icon): the glyph cut
            // into three horizontal slices with a hairline of background at
            // each cut, middle slice nudged right. Geometry measured off the
            // icon, as fractions of cap height below the cap top: slice 1
            // ends at 0.307, slice 2 spans 0.360–0.727 shifted +0.083·cap,
            // slice 3 starts at 0.779. Cap metrics are Playfair's "F" ink:
            // cap height 0.711 × size, cap top 0.375 × size below galley top.
            let cap = 0.711 * wordmark_px;
            let cap_top = rect.min.y + 0.375 * wordmark_px;
            let f_width = f_galley.size().x;
            let slices: [(f32, f32, f32); 3] = [
                (f32::NEG_INFINITY, 0.307, 0.0),
                (0.360, 0.727, 0.083),
                (0.779, f32::INFINITY, 0.0),
            ];
            // Snapped to the pixel grid so the cuts are hard lines, not
            // antialiased smears — at 30 px the gaps are only ~1 px.
            let ppp = ui.ctx().pixels_per_point();
            let snap = |v: f32| (v * ppp).round() / ppp;
            for (top, bot, dx) in slices {
                let band = egui::Rect::from_min_max(
                    egui::pos2(rect.min.x - 2.0, snap((cap_top + top * cap).max(rect.min.y))),
                    egui::pos2(rect.min.x + f_width + 4.0, snap((cap_top + bot * cap).min(rect.max.y))),
                );
                ui.painter().with_clip_rect(band).galley(
                    rect.min + egui::vec2(snap(dx * cap), 0.0),
                    f_galley.clone(),
                    egui::Color32::WHITE,
                );
            }
            // "able" lands exactly where the one-galley layout put it.
            let able_pos = egui::pos2(
                rect.min.x + fable_galley.size().x - able_galley.size().x,
                rect.min.y,
            );
            ui.painter().galley(able_pos, able_galley, egui::Color32::WHITE);
            ui.painter().galley(slicer_pos, slicer_galley, egui::Color32::WHITE);
            ui.add_space(8.0);
            // Object actions as one even row — same grid the Slice/Export
            // rows use, so the panel's top reads as aligned blocks.
            let third = (ui.available_width() - 2.0 * ui.spacing().item_spacing.x) / 3.0;
            ui.horizontal(|ui| {
                if ui
                    .add(egui::Button::new("Import…").min_size(egui::vec2(third, 26.0)))
                    .on_hover_text("Load an STL or 3MF file and add it to the bed (a 3MF build's objects each arrive separately).")
                    .clicked()
                {
                    let mut dialog =
                        rfd::FileDialog::new().add_filter("models (STL, 3MF)", &["stl", "3mf"]);
                    if let Some(dir) = &self.last_model_dir {
                        dialog = dialog.set_directory(dir);
                    }
                    if let Some(path) = dialog.pick_file() {
                        self.last_model_dir = path.parent().map(|d| d.to_path_buf());
                        self.import_model(path);
                    }
                }
                if ui
                    .add_enabled(
                        self.selected.is_some(),
                        egui::Button::new("Duplicate").min_size(egui::vec2(third, 26.0)),
                    )
                    .on_hover_text("Add a copy of the selected object (shares geometry; re-arranged on the bed).")
                    .clicked()
                {
                    self.duplicate_selected();
                }
                if ui
                    .add_enabled(
                        self.selected.is_some(),
                        egui::Button::new("Delete").min_size(egui::vec2(third, 26.0)),
                    )
                    .on_hover_text("Remove the selected object from the bed.")
                    .clicked()
                {
                    self.delete_selected();
                }
            });
            if self.objects.is_empty() {
                ui.weak("Import an STL to begin.");
            } else {
                for i in 0..self.objects.len() {
                    let sel = self.selected == Some(i);
                    let name = self.objects[i].name.clone();
                    if ui
                        .selectable_label(sel, name)
                        .on_hover_text("Click to select. Drag it in the 3D view to move it; rotate/scale via the on-screen panel.")
                        .clicked()
                    {
                        self.selected = Some(i);
                        self.needs_rebuild = true; // refresh highlight (camera stays put)
                    }
                }
            }

            ui.separator();

            let printers: Vec<String> = self.profiles.printer_names().iter().map(|s| s.to_string()).collect();
            let filaments: Vec<String> = self.profiles.filament_names().iter().map(|s| s.to_string()).collect();
            let processes: Vec<String> = self.profiles.process_names().iter().map(|s| s.to_string()).collect();
            let dirty = self.tier_dirty_masked();
            let mut changed = false;
            let mut open_dialog: Option<ProfileDialog> = None;
            let prev_sel = (self.printer.clone(), self.filament.clone(), self.process.clone());
            {
                let rows: [(TierKind, &mut String, &[String], bool, &str); 3] = [
                    (TierKind::Printer, &mut self.printer, &printers, dirty[0],
                        "Machine profile — bed size, nozzle, motion limits, and start/end g-code."),
                    (TierKind::Filament, &mut self.filament, &filaments, dirty[1],
                        "Material profile — hotend/bed temperatures, diameter, density, flow, cooling."),
                    (TierKind::Process, &mut self.process, &processes, dirty[2],
                        "Print-quality profile (layer height, walls, speeds, supports…). Edits below override it until you switch or save."),
                ];
                for (kind, sel, names, is_dirty, hover) in rows {
                    ui.horizontal(|ui| {
                        let title = match kind {
                            TierKind::Printer => "Printer",
                            TierKind::Filament => "Filament",
                            TierKind::Process => "Process",
                        };
                        // Fixed-width label column (tier dot + name) so the
                        // three combos align into one clean column. The dot is
                        // the same mark the section headers carry.
                        ui.scope(|ui| {
                            ui.set_width(78.0);
                            ui.spacing_mut().item_spacing.x = 5.0;
                            // Painted dot (the "●" glyph is missing from the
                            // default fonts and renders as a box).
                            let (dot, _) =
                                ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                            ui.painter().circle_filled(dot.center(), 5.0, tier_color(kind));
                            let label = if is_dirty {
                                egui::RichText::new(format!("{title} *")).color(palette::CREAM)
                            } else {
                                egui::RichText::new(title).color(palette::CREAM_DIM)
                            };
                            ui.label(label).on_hover_text(hover);
                        });
                        let is_user = self.profiles.is_user(kind, sel);
                        let r = egui::ComboBox::from_id_salt(kind.label())
                            .width(136.0)
                            .selected_text(sel.clone())
                            .show_ui(ui, |ui| {
                                for opt in names {
                                    if ui.selectable_value(sel, opt.clone(), opt).changed() {
                                        changed = true;
                                    }
                                }
                            });
                        r.response.on_hover_text(hover);
                        if ui
                            .small_button("💾")
                            .on_hover_text(if is_dirty {
                                "Save the * changes as a user profile (only changed fields are written)."
                            } else {
                                "Save a copy as a user profile."
                            })
                            .clicked()
                        {
                            let name = if is_user { sel.clone() } else { format!("{sel}-custom") };
                            open_dialog = Some(ProfileDialog { kind, name, delete: false });
                        }
                        if is_user
                            && ui
                                .small_button("🗑")
                                .on_hover_text("Delete this user profile from disk.")
                                .clicked()
                        {
                            open_dialog = Some(ProfileDialog { kind, name: sel.clone(), delete: true });
                        }
                    });
                }
            }
            if let Some(d) = open_dialog {
                self.profile_dialog = Some(d);
            }
            if changed {
                if dirty.iter().any(|&d| d) {
                    // Switching re-resolves settings from disk and would
                    // silently wipe the unsaved (*) edits — park the switch
                    // behind a confirmation instead.
                    self.pending_switch =
                        Some((self.printer.clone(), self.filament.clone(), self.process.clone()));
                    self.printer = prev_sel.0.clone();
                    self.filament = prev_sel.1.clone();
                    self.process = prev_sel.2.clone();
                } else {
                    // A new printer means a new plate — re-pivot even when its
                    // dimensions happen to match (the bed-size check at the
                    // top of `ui` only catches actual changes).
                    if self.printer != prev_sel.0 {
                        self.recenter_camera = true;
                    }
                    self.reresolve();
                }
            }
            ui.separator();

            // Slice / export / send — the panel's primary actions, sized to be
            // unmissable. Live-print controls float over the viewport instead
            // (they appear after a successful send).
            let half = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
            let big = egui::vec2(half, 32.0);
            ui.horizontal(|ui| {
                // Slice is the hero action: printed in reverse — cream plate,
                // ink text — the one inverted block in the panel.
                let can_slice = !self.objects.is_empty();
                let mut label = egui::RichText::new("Slice").size(15.0).strong();
                if can_slice {
                    label = label.color(palette::INK);
                }
                let mut slice_btn = egui::Button::new(label).min_size(big);
                if can_slice {
                    slice_btn = slice_btn.fill(palette::CREAM);
                }
                if ui
                    .add_enabled(can_slice, slice_btn)
                    .on_hover_text("Slice all objects on the bed into toolpaths using the current settings.")
                    .on_disabled_hover_text("Import a model first.")
                    .clicked()
                {
                    self.slice(&rs);
                }
                let export_btn = egui::Button::new(egui::RichText::new("Export…").size(15.0)).min_size(big);
                if ui
                    .add_enabled(self.sliced.is_some(), export_btn)
                    .on_hover_text("Save the sliced toolpaths to a .gcode file.")
                    .on_disabled_hover_text("Slice first.")
                    .clicked()
                {
                    self.export();
                }
            });
            ui.horizontal(|ui| {
                let can_send = self.sliced.is_some() && host_set && !host_busy;
                let send_btn = egui::Button::new(egui::RichText::new("Send").size(15.0)).min_size(big);
                if ui
                    .add_enabled(can_send, send_btn)
                    .on_hover_text("Upload the g-code to the printer's storage (host under Connection).")
                    .on_disabled_hover_text("Needs a sliced model and a printer host (Connection section).")
                    .clicked()
                {
                    host_op = Some(HostOp::Send { start: false });
                }
                let print_btn = egui::Button::new(egui::RichText::new("▶ Send & print").size(15.0)).min_size(big);
                if ui
                    .add_enabled(can_send, print_btn)
                    .on_hover_text("Upload the g-code and start printing it immediately.")
                    .on_disabled_hover_text("Needs a sliced model and a printer host (Connection section).")
                    .clicked()
                {
                    host_op = Some(HostOp::Send { start: true });
                }
            });
            ui.separator();

            // Prominent Model / Preview toggle (Preview enabled once sliced).
            let n_layers = self.sliced.as_ref().map(|l| l.len()).unwrap_or(0);
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                // Split the row exactly: two buttons plus the spacing between
                // them must not exceed the row (a 2 pt overflow here widens
                // the whole panel — see the note at Panel::left above).
                let bw = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
                if ui
                    .add_sized([bw, 28.0], egui::Button::selectable(!self.view_preview, "Model"))
                    .on_hover_text("Show the 3D model(s) on the bed.")
                    .clicked()
                {
                    self.view_preview = false;
                }
                let prev = ui
                    .add_enabled_ui(n_layers > 0, |ui| {
                        ui.add_sized([bw, 28.0], egui::Button::selectable(self.view_preview, "Preview"))
                            .on_hover_text("Show the sliced toolpaths.")
                    })
                    .inner;
                if prev.clicked() {
                    self.view_preview = true;
                }
            });
            // The accent picker: one hue drives the whole 3D view (model
            // tint, feature palette, heat ramps). The mesh tints ride shader
            // uniforms and follow the picker live; the baked preview colors
            // re-derive when the mouse releases — re-baking every instance
            // buffer per drag frame would stutter on big slices.
            ui.horizontal(|ui| {
                ui.label("accent").on_hover_text(
                    "The 3D view's color. The model tint, the feature palette, and the \
                     heat-map ramps are all derived from this one hue — pick whatever \
                     reads best to you. Remembered across sessions.",
                );
                let mut rgb = [self.accent.r(), self.accent.g(), self.accent.b()];
                if ui.color_edit_button_srgb(&mut rgb).changed() {
                    self.accent = egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
                    self.accent_rebake = true;
                }
                if self.accent != DEFAULT_ACCENT
                    && ui
                        .small_button("⟲")
                        .on_hover_text("Back to the default brass.")
                        .clicked()
                {
                    self.accent = DEFAULT_ACCENT;
                    self.accent_rebake = true;
                }
            });
            if self.accent_rebake && !ui.ctx().input(|i| i.pointer.any_down()) {
                self.accent_rebake = false;
                self.set_preview_instances(&rs);
            }
            if self.view_preview && n_layers > 0 {
                hslider(ui, true, egui::Slider::new(&mut self.preview_layer, 1..=n_layers), "layer",
                    "Highest layer shown; lower layers are dimmed.");
                ui.label(format!("showing layers 1–{}/{}", self.preview_layer, n_layers));
                ui.add_space(2.0);
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(&mut self.show_walls, "walls").on_hover_text("Show wall (perimeter) toolpaths.");
                    ui.checkbox(&mut self.show_solid, "solid").on_hover_text("Show solid top/bottom fill.");
                    ui.checkbox(&mut self.show_infill, "infill").on_hover_text("Show sparse interior infill.");
                    ui.checkbox(&mut self.show_gap_fill, "gap fill").on_hover_text("Show thin gap-fill strokes between walls.");
                    ui.checkbox(&mut self.show_ironing, "ironing").on_hover_text("Show the top-surface ironing pass.");
                    ui.checkbox(&mut self.show_skirt, "skirt").on_hover_text("Show skirt and brim.");
                    ui.checkbox(&mut self.show_support, "support").on_hover_text("Show support, bridge, and arc-overhang toolpaths.");
                    ui.checkbox(&mut self.show_travel, "travel").on_hover_text("Show non-printing travel moves.");
                    ui.checkbox(&mut self.show_seams, "seams").on_hover_text("Highlight where each wall loop starts (the seam).");
                });
                ui.horizontal(|ui| {
                    ui.label("color").on_hover_text(
                        "What the preview colors encode. Feature type is the classic view. The two heat \
                         maps spot overheating: layer time shows where the nozzle returns quickly (little \
                         cooling time), and heat load also weighs how much hot plastic is deposited and \
                         how much footprint it has to cool through — scored per island, i.e. per \
                         disconnected region of each layer. Nozzle °C shows the planned temp \
                         schedule directly. All views reflect the last slice — re-slice after \
                         changing toggles.",
                    );
                    let before = self.color_by;
                    egui::ComboBox::from_id_salt("preview_color_by")
                        .selected_text(match self.color_by {
                            ColorBy::Feature => "feature type",
                            ColorBy::LayerTime => "layer time",
                            ColorBy::Heat => "heat load",
                            ColorBy::Temp => "nozzle °C",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.color_by, ColorBy::Feature, "feature type");
                            ui.selectable_value(&mut self.color_by, ColorBy::LayerTime, "layer time");
                            ui.selectable_value(&mut self.color_by, ColorBy::Heat, "heat load");
                            ui.selectable_value(&mut self.color_by, ColorBy::Temp, "nozzle °C");
                        });
                    if self.color_by != before {
                        self.set_preview_instances(&rs);
                    }
                });
                if self.color_by != ColorBy::Feature && !self.layer_stats.is_empty() {
                    let mut vals: Vec<f64> = Vec::new();
                    match self.color_by {
                        ColorBy::LayerTime => vals.extend(self.layer_stats.iter().map(|st| st.secs)),
                        ColorBy::Temp => {} // the temp legend computes its own ends
                        _ => {
                            for (li, l) in self.layer_islands.iter().enumerate() {
                                for isl in &l.islands {
                                    if isl.joules > 0.0 {
                                        vals.push(self.island_heat(li, isl));
                                    }
                                }
                            }
                        }
                    }
                    let lo = vals.iter().cloned().fold(f64::INFINITY, f64::min);
                    let hi = vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    // Legend ends carry the real numbers: left = dark (cool/safe), right = bright (hot).
                    let anchored = self.color_by == ColorBy::Heat
                        && self.settings.heat_control
                        && self.settings.max_heat_mw_mm2 > 0.0;
                    let (left, right, expl) = match self.color_by {
                        ColorBy::LayerTime => (
                            format!("{hi:.1}s"),
                            format!("{lo:.1}s"),
                            "Per-layer print time, log scale. Brightest = the quickest layers: the plastic \
                             below gets the least time to cool before the nozzle returns. The min-layer \
                             slowdown under Feature speeds is the usual fix."
                                .to_string(),
                        ),
                        ColorBy::Temp => {
                            let base = self.settings.nozzle_temp_c as f64;
                            let span = self.sliced.as_ref().map_or(2.0, |layers| {
                                layers
                                    .iter()
                                    .filter_map(|l| l.planned_temp_c)
                                    .map(|t| (t - base).abs())
                                    .fold(2.0_f64, f64::max)
                            });
                            (
                                format!("{:.0} °C", base - span),
                                format!("{:.0} °C", base + span),
                                "Heat control's temp schedule: the planned nozzle temperature of every \
                                 layer, centered on the printing temperature — dark = scheduled cooler \
                                 (hot zones), bright = scheduled warmer (cold bands). One flat color means \
                                 nothing is scheduled (heat control off, or nothing to do). The first \
                                 layer's adhesion temperature is separate and not shown."
                                    .to_string(),
                            )
                        }
                        _ => {
                            let target_note = if anchored {
                                let target_mw = self
                                    .slice_summary
                                    .as_ref()
                                    .map(|s| s.heat_target_mw)
                                    .unwrap_or(self.settings.max_heat_mw_mm2);
                                format!(" The tick marks the ceiling ({target_mw:.1}).")
                            } else {
                                String::new()
                            };
                            (
                                format!("{HEAT_SCALE_LO_MW:.0}"),
                                format!("{HEAT_SCALE_HI_MW:.0} mW/mm²"),
                                format!(
                                    "Heat load: how fast hot plastic lands on a region, per mm² of \
                                     it — the heat the new layer deposits there, divided by the \
                                     layer's print time and the region's area. Too high and the \
                                     plastic below can't cool before more heat arrives (sagging, \
                                     gloss bands). Too low and the layers contract as they print, \
                                     pulling the band inward — the dimple that appears where the \
                                     cross-section suddenly grows and layers turn slow and cold — \
                                     and fuse weakly. \
                                     Fixed log scale, {HEAT_SCALE_LO_MW:.0}–{HEAT_SCALE_HI_MW:.0} mW/mm².{target_note}"
                                ),
                            )
                        }
                    };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(left).small()).on_hover_text(expl.as_str());
                        let (rect, _) = ui.allocate_exact_size(egui::vec2(80.0, 10.0), egui::Sense::hover());
                        if ui.is_rect_visible(rect) {
                            let n = 24;
                            let acc = accent_hsl(self.accent);
                            for i in 0..n {
                                let c = heat_ramp(i as f32 / (n - 1) as f32, acc);
                                let x0 = rect.min.x + rect.width() * i as f32 / n as f32;
                                let x1 = rect.min.x + rect.width() * (i + 1) as f32 / n as f32;
                                ui.painter().rect_filled(
                                    egui::Rect::from_min_max(egui::pos2(x0, rect.min.y), egui::pos2(x1, rect.max.y)),
                                    0.0,
                                    egui::Color32::from_rgb(
                                        (c[0] * 255.0) as u8,
                                        (c[1] * 255.0) as u8,
                                        (c[2] * 255.0) as u8,
                                    ),
                                );
                            }
                            if anchored {
                                // The heat target at its true position on the fixed
                                // scale — the computed level in even mode.
                                let target_mw = self
                                    .slice_summary
                                    .as_ref()
                                    .map(|s| s.heat_target_mw)
                                    .unwrap_or(self.settings.max_heat_mw_mm2);
                                let lo = HEAT_SCALE_LO_MW.ln();
                                let fr = ((target_mw.max(1e-6).ln() - lo)
                                    / (HEAT_SCALE_HI_MW.ln() - lo))
                                    .clamp(0.0, 1.0) as f32;
                                let x = rect.min.x + rect.width() * fr;
                                // Cream core in an ink casing: visible on
                                // both the dark and bright halves, whatever
                                // hue the ramp runs in.
                                let seg = [egui::pos2(x, rect.min.y - 1.0), egui::pos2(x, rect.max.y + 1.0)];
                                ui.painter().line_segment(seg, egui::Stroke::new(3.5, palette::INK));
                                ui.painter().line_segment(seg, egui::Stroke::new(1.5, palette::CREAM));
                            }
                        }
                        ui.label(egui::RichText::new(right).small()).on_hover_text(expl.as_str());
                    });
                }
            }
            ui.separator();

            // Settings, grouped into collapsible categories (Orca-style) and scrolled.
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                let s = &mut self.settings;
                let pins = &mut self.pins;
                tier_section(ui, "Quality", TierKind::Process, true, |ui| {
                    let lh_hint = format!(
                        "Height of each printed layer. Smaller = finer detail but slower.\n\n\
                         One corner of the flow triangle: every mm/s of print speed extrudes a bead of \
                         line width × layer height, and the hotend can only melt `max flow` mm³ per second. \
                         The speed ceiling is therefore {}. \
                         Thicker layers lower that ceiling — unpinned feature speeds follow it live.",
                        flow_ceiling_text(s)
                    );
                    hslider(ui, true, egui::Slider::new(&mut s.layer_height_mm, 0.05..=0.4), "layer mm",
                        lh_hint);
                    hslider(ui, true, egui::Slider::new(&mut s.first_layer_height_mm, 0.1..=0.4), "first layer mm",
                        "Thickness of the first layer — often thicker for bed adhesion.");
                    seam_combo(ui, &mut s.seam_mode)
                        .on_hover_text("Where each wall loop starts: nearest point, sharpest corner, or random.");
                    ui.checkbox(&mut s.arc_fitting, "arc fitting (G2/G3)")
                        .on_hover_text("Emit curved toolpaths as G2/G3 arcs — smaller g-code, smoother motion. Needs firmware arc support (Klipper [gcode_arcs]).");
                    hslider(ui, s.arc_fitting, egui::Slider::new(&mut s.arc_tolerance_mm, 0.005..=0.2), "arc tol mm",
                        "Max deviation a point may have from a fitted arc to be folded into it.");
                    hslider(ui, true, egui::Slider::new(&mut s.elephant_foot_mm, 0.0..=0.5), "elephant foot mm",
                        "Shrink the first layer's outline inward to counter first-layer squish. 0 = off.");
                    hslider(ui, true, egui::Slider::new(&mut s.xy_compensation_mm, -0.5..=0.5), "XY comp mm",
                        "Grow (+) or shrink (−) every layer's outline for dimensional accuracy. 0 = off.");
                    let vase = s.spiral_vase;
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.ironing, "ironing"))
                        .on_hover_text("Re-traverse top surfaces with a hot nozzle and a trickle of flow to melt them smooth.")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.fuzzy_skin, "fuzzy skin"))
                        .on_hover_text("Jitter the outer wall into a rough, textured surface (hides layer lines).")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    hslider(ui, s.fuzzy_skin && !vase, egui::Slider::new(&mut s.fuzzy_skin_thickness_mm, 0.05..=1.0), "fuzzy thickness mm",
                        "Total jitter band, centered on the wall line.");
                    hslider(ui, s.fuzzy_skin && !vase, egui::Slider::new(&mut s.fuzzy_skin_point_dist_mm, 0.2..=2.0), "fuzzy point dist mm",
                        "Spacing between jittered points — smaller is noisier.");
                    ui.checkbox(&mut s.spiral_vase, "spiral vase")
                        .on_hover_text("One continuously rising outer wall above a solid bottom — no infill, no seams. Forces 1 wall / 0% infill / no supports (those controls gray out).");
                });
                tier_section(ui, "Walls & top/bottom", TierKind::Process, false, |ui| {
                    let vase = s.spiral_vase;
                    ui.add_enabled_ui(!vase && !s.brick_layers, |ui| {
                        egui::ComboBox::from_id_salt("wall_mode")
                            .selected_text(s.wall_mode.label())
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut s.wall_mode, config::WallMode::Arachne, "arachne");
                                ui.selectable_value(&mut s.wall_mode, config::WallMode::Classic, "classic");
                            })
                            .response
                            .on_hover_text("How gaps are handled is binary: arachne walls vary their width with the local thickness, absorbing every gap into the beads (thin cores become tapered walls). Classic uses fixed offsets and patches the leftovers with gap fill.")
                            .on_disabled_hover_text("Brick layering and spiral vase need classic uniform rings.");
                        ui.label("wall mode");
                    });
                    hslider_lockout(ui, !vase, egui::Slider::new(&mut s.wall_count, 1..=6), "walls",
                        "Number of perimeter loops (shell wall thickness).",
                        "Spiral vase forces a single wall.");
                    hslider_lockout(ui, !vase, egui::Slider::new(&mut s.top_layers, 0..=10), "top layers",
                        "Number of solid layers on top surfaces.",
                        "Spiral vase prints no top shells.");
                    hslider(ui, true, egui::Slider::new(&mut s.bottom_layers, 0..=10), "bottom layers",
                        "Number of solid layers on bottom surfaces.");
                    ui.add_enabled(!vase && s.wall_mode == config::WallMode::Classic, egui::Checkbox::new(&mut s.gap_fill, "gap fill"))
                        .on_hover_text("Classic mode: fill gaps too thin for walls/infill with single width-matched strokes.")
                        .on_disabled_hover_text("Arachne absorbs gaps into the walls themselves — gap fill only applies to classic mode (and is off in spiral vase).");
                    ui.checkbox(&mut s.monotonic_solid, "monotonic top/bottom")
                        .on_hover_text("Print solid-fill lines in one strict sweep per surface for an even sheen.");
                    ui.add_enabled(!vase && !s.brick_layers, egui::Checkbox::new(&mut s.half_height_outer_walls, "half-height outer wall"))
                        .on_hover_text("Print the outer wall as two half-height passes, each sliced at its own plane — halves the visible Z staircase on slopes while the interior keeps full layer height. Costs roughly the outer-wall print time again.")
                        .on_disabled_hover_text("Unavailable in spiral vase mode or with brick layers (their Z choreographies collide).");
                    ui.add_enabled(!vase && !s.half_height_outer_walls, egui::Checkbox::new(&mut s.brick_layers, "brick layers"))
                        .on_hover_text("Stagger odd perimeters by half a layer height so wall rings interlock like bricks (the outer wall stays put). The lifted beads' extra flow is derived from the bead geometry. Best with 3+ walls.")
                        .on_disabled_hover_text("Unavailable in spiral vase mode or with half-height outer walls.");
                });
                tier_section(ui, "Infill", TierKind::Process, false, |ui| {
                    let vase = s.spiral_vase;
                    hslider_lockout(ui, !vase, egui::Slider::new(&mut s.infill_density, 0.0..=1.0), "density",
                        "Sparse interior fill density (0 = hollow, 1 = solid).",
                        "Spiral vase prints no infill.");
                    ui.add_enabled_ui(s.infill_density > 0.0 && !vase, |ui| {
                        pattern_combo(ui, "sparse fill", &mut s.sparse_pattern)
                            .on_hover_text("Pattern for the sparse interior infill.");
                    });
                    pattern_combo(ui, "solid fill", &mut s.solid_pattern)
                        .on_hover_text("Pattern for the solid top/bottom layers.");
                    hslider(ui, true, egui::Slider::new(&mut s.infill_overlap, 0.0..=0.5), "wall overlap",
                        "How far infill pushes into the innermost wall (fraction of a line width) so they bond.");
                });
                tier_section(ui, "Heat control", TierKind::Process, false, |ui| {
                    ui.checkbox(&mut s.heat_control, "heat control")
                        .on_hover_text(
                            "The automatic: keeps every island's heat load inside the filament's \
                             allowable ranges and smooths layer-to-layer transitions — the \
                             banding/shrinkage killer. One gradient-limited heat curve is derived \
                             per print, the gentlest the time budget below affords, and both \
                             levers serve it: the nozzle-temperature schedule warms cold dips and \
                             cools hot ranges inside the filament's packaging range (free in \
                             print time, and never past the point where the flow derate would \
                             cost more than the cooling saves), per-island slowing and \
                             park-and-wait dwells supply what temperature can't reach. The \
                             schedule only ever FADES — a few °C per millimetre of height — so \
                             temperature itself never draws a line on the surface. Off restores \
                             the derived speeds and temperatures exactly.",
                        );
                    hslider(ui, s.heat_control, egui::Slider::new(&mut s.smooth_extra_time_pct, 0.0..=50.0), "extra time %",
                        "Heat control's budget: the most extra print time smoothing may spend, as a % of the un-smoothed estimate. The transition gradient is bisected to the gentlest that fits and reported after slicing; 0 still does everything free (warming cold dips, capping at the filament's ceiling).");
                    ui.weak("speeds and temperatures are chosen by the system: the machine \
                             rating under the filament's melt ceiling, then heat control \
                             governs the result");
                });
                tier_section(ui, "Support", TierKind::Process, true, |ui| {
                    let vase = s.spiral_vase;
                    ui.add_enabled_ui(!vase, |ui| {
                        support_combo(ui, &mut s.support_mode)
                            .on_hover_text("Overhang handling: none, grid supports, or self-supporting arcs.")
                            .on_disabled_hover_text("Forced off in spiral vase mode.");
                    });
                    let has_support = s.support_mode != config::SupportMode::None && !vase;
                    let arc = s.support_mode == config::SupportMode::Arc && !vase;
                    hslider(ui, has_support, egui::Slider::new(&mut s.support_overhang_angle_deg, 0.0..=80.0), "overhang °",
                        "Steepest overhang (from vertical) printable without support. 45° ≈ one layer-width.");
                    hslider(ui, has_support, egui::Slider::new(&mut s.support_density, 0.0..=1.0), "density",
                        "Infill density of grid supports.");
                    hslider(ui, has_support, egui::Slider::new(&mut s.support_xy_clearance_mm, 0.0..=2.0), "xy gap mm",
                        "Horizontal gap between support and the model (for easy removal).");
                    hslider(ui, has_support, egui::Slider::new(&mut s.support_z_gap_layers, 0..=5), "z-gap layers",
                        "Empty layers between a support top and the part it holds up.");
                    hslider(ui, has_support, egui::Slider::new(&mut s.support_interface_layers, 0..=5), "interface",
                        "Dense solid layers at the support top for a smoother overhang underside.");
                    hslider(ui, arc, egui::Slider::new(&mut s.max_bridge_span_mm, 0.0..=30.0), "bridge span mm",
                        "Arc mode: gaps narrower than this bridge with straight lines; wider use arcs.");
                    hslider(ui, arc, egui::Slider::new(&mut s.max_arc_radius_mm, 5.0..=100.0), "arc radius mm",
                        "Arc mode: max arc-overhang radius before a fan re-seeds.");
                    hslider(ui, arc, egui::Slider::new(&mut s.arc_seam_overlap_mm, 0.0..=0.6), "arc seam overlap mm",
                        "Arc mode: how far fans overlap where they meet (per fan). A little helps them mesh; too much over-extrudes the seam. 0 = butt.");
                });
                tier_section(ui, "Bed adhesion", TierKind::Process, false, |ui| {
                    hslider(ui, true, egui::Slider::new(&mut s.skirt_loops, 0..=5), "skirt loops",
                        "Loops printed around the first layer to prime the nozzle. 0 = off.");
                    hslider(ui, s.skirt_loops > 0, egui::Slider::new(&mut s.skirt_gap_mm, 0.0..=10.0), "skirt gap mm",
                        "Distance from the skirt to the model.");
                    hslider(ui, true, egui::Slider::new(&mut s.brim_loops, 0..=20), "brim loops",
                        "Loops attached around the first layer for adhesion. 0 = off.");
                });
                tier_section(ui, "Filament", TierKind::Filament, false, |ui| {
                    // The packaging card: what the box says. The material
                    // class itself is profile data — switching filament
                    // profiles changes it — and supplies every derived value
                    // here until a calibration entry pins it.
                    hslider(ui, true, egui::Slider::new(&mut s.nozzle_temp_min_c, 150..=300), "nozzle min °C",
                        "The low end of the temperature range printed on the spool. Heat control's schedules never go below it.");
                    hslider(ui, true, egui::Slider::new(&mut s.nozzle_temp_max_c, 150..=320), "nozzle max °C",
                        "The high end of the range on the spool. Heat control's schedules never go above it.");
                    hslider(ui, true, egui::Slider::new(&mut s.bed_temp_c, 0..=120), "bed °C",
                        "Bed temperature from the packaging.");
                    hslider_lockout(ui, !s.chamber_sensor.trim().is_empty(), egui::Slider::new(&mut s.chamber_temp_c, 0..=70), "chamber soak °C",
                        "Hold after the start g-code until the chamber reaches this (the heated \
                         bed does the soaking — TEMPERATURE_WAIT on the printer's chamber sensor). \
                         0 = off. Auto: the material class's value — ABS/ASA soak at 50 against \
                         warping and layer splits; PLA must stay 0 (a hot chamber means heat \
                         creep and sag).",
                        "Needs a chamber sensor declared under Machine & motion.");
                    hslider(ui, true, egui::Slider::new(&mut s.filament_diameter_mm, 1.0..=3.0), "filament Ø mm",
                        "Filament diameter (1.75 or 2.85). Drives the extrusion math.");
                    ui.weak(format!(
                        "operating point: {} °C (first layer {} °C) — chosen by the system",
                        s.nozzle_temp_c, s.first_layer_nozzle_temp_c
                    ));
                    egui::CollapsingHeader::new("calibration")
                        .id_salt("filament_calibration")
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.weak("class defaults until you measure better numbers; saved with the filament");
                            hslider(ui, true, egui::Slider::new(&mut s.filament_density_g_cm3, 0.8..=2.0), "density g/cm³",
                                "Filament density — used for the weight estimate. Auto: the material class's value.");
                            hslider(ui, true, egui::Slider::new(&mut s.extrusion_multiplier, 0.8..=1.2), "flow ×",
                                "Global extrusion multiplier — the per-spool flow calibration escape.");
                            let mf_hint = format!(
                                "The filament's measured melt-rate ceiling (mm³/s). The class default is deliberately conservative; a flow-test value belongs here. Right now: {}.",
                                flow_ceiling_text(s)
                            );
                            hslider(ui, true, egui::Slider::new(&mut s.max_volumetric_speed_mm3_s, 0.0..=80.0), "max flow mm³/s",
                                mf_hint);
                            hslider(ui, true, egui::Slider::new(&mut s.max_flow_derate_per_c, 0.0..=1.0), "flow derate /°C",
                                "How much of the melt ceiling is lost per °C below the operating temperature. Heat control weighs this when cooling: once a zone's flow-bound paths would lose more time to the derate than the cooling saves in slowdowns, the schedule holds at that crossing instead of the cold rail — steep derates mean gentler cooling.");
                            auto_slider(ui, &mut s.max_heat_mw_mm2, 2.0..=40.0, "max heat mW/mm²", &mut pins.max_heat, s.material.max_heat_mw_mm2(),
                                "The material's allowable heat-load ceiling, per island — heat control never plans above it. Read natural values off the heat-load preview legend. Auto: the class's value.");
                            hslider(ui, true, egui::Slider::new(&mut s.pressure_advance, 0.0..=0.2), "pressure advance",
                                "Klipper pressure advance, emitted as SET_PRESSURE_ADVANCE. 0 = leave the printer's value.");
                            hslider(ui, true, egui::Slider::new(&mut s.fan_speed, 0.0..=1.0), "fan",
                                "Part-cooling fan duty while printing. Auto: the class's policy.");
                            hslider(ui, true, egui::Slider::new(&mut s.bridge_fan_speed, 0.0..=1.0), "bridge fan",
                                "Fan duty on bridges and arc overhangs.");
                            hslider(ui, true, egui::Slider::new(&mut s.fan_off_layers, 0..=5), "fan off layers",
                                "Keep the fan off for this many first layers (bed adhesion).");
                            hslider(ui, s.has_aux_fan, egui::Slider::new(&mut s.aux_fan_speed, 0.0..=1.0), "aux fan",
                                "Auxiliary part-cooling duty (M106 P2). Needs the aux fan declared under Machine & motion.");
                            hslider(ui, s.has_exhaust_fan, egui::Slider::new(&mut s.exhaust_fan_speed, 0.0..=1.0), "exhaust fan",
                                "Chamber-exhaust duty (M106 P3), whole print. Needs the exhaust fan declared under Machine & motion.");
                        });
                });
                tier_section(ui, "Retraction", TierKind::Printer, false, |ui| {
                    hslider(ui, true, egui::Slider::new(&mut s.retract_len_mm, 0.0..=10.0), "length mm",
                        "Filament pulled back on travels to prevent oozing/stringing.");
                    hslider(ui, true, egui::Slider::new(&mut s.retract_speed_mm_s, 5.0..=100.0), "speed mm/s",
                        "How fast filament is retracted and recovered.");
                    hslider(ui, true, egui::Slider::new(&mut s.z_hop_mm, 0.0..=2.0), "z-hop mm",
                        "Lift the nozzle on travels that cross a gap/void. 0 = off.");
                    hslider(ui, true, egui::Slider::new(&mut s.wipe_mm, 0.0..=5.0), "wipe mm",
                        "After retracting, drag the nozzle back along the printed bead by this much before travelling — ooze smears onto plastic instead of blobbing the seam. 0 = off.");
                });
                tier_section(ui, "Machine & motion", TierKind::Printer, false, |ui| {
                    hslider(ui, true, egui::Slider::new(&mut s.bed_size_x_mm, 50.0..=500.0), "bed X mm",
                        "Bed width (X).");
                    hslider(ui, true, egui::Slider::new(&mut s.bed_size_y_mm, 50.0..=500.0), "bed Y mm",
                        "Bed depth (Y).");
                    hslider(ui, true, egui::Slider::new(&mut s.bed_size_z_mm, 50.0..=600.0), "bed Z mm",
                        "Maximum build height (Z).");
                    hslider(ui, true, egui::Slider::new(&mut s.nozzle_diameter_mm, 0.1..=1.2), "nozzle mm",
                        "Nozzle diameter.");
                    hslider(ui, true, egui::Slider::new(&mut s.machine_speed_mm_s, 10.0..=700.0), "rated mm/s",
                        "The machine's rated print speed — a datasheet number, the hard cap every derived speed works under. Lower it to slow the whole machine.");
                    hslider(ui, true, egui::Slider::new(&mut s.first_layer_speed_mm_s, 5.0..=100.0), "1st layer mm/s",
                        "Speed for the first layer — slower improves bed adhesion.");
                    hslider(ui, true, egui::Slider::new(&mut s.travel_speed_mm_s, 20.0..=600.0), "travel mm/s",
                        "Speed for non-printing moves between features.");
                    hslider(ui, true, egui::Slider::new(&mut s.acceleration_mm_s2, 100.0..=20000.0), "accel mm/s²",
                        "Acceleration for inner walls, infill, solid, support, and travel — emitted as M204 per feature. Klipper clamps to printer.cfg max_accel. Higher = faster but more ringing.");
                    auto_slider(ui, &mut s.outer_wall_accel_mm_s2, 100.0..=20000.0, "outer accel",
                        &mut pins.outer_wall_accel, config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2),
                        "Acceleration for the visible outermost wall — lower hides ringing. Auto = 50% of accel.");
                    auto_slider(ui, &mut s.first_layer_accel_mm_s2, 100.0..=20000.0, "1st layer accel",
                        &mut pins.first_layer_accel, config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2),
                        "Acceleration for the whole first layer — gentle for bed adhesion. Auto = min(1000, accel).");
                    hslider(ui, true, egui::Slider::new(&mut s.heat_rate_c_s, 0.2..=10.0), "heat °C/s",
                        "How fast the hotend heats near printing temperatures. Heat control's temp schedule starts warming ramps this much ahead so the nozzle arrives on temperature. Conservative default — a Moonraker calibration routine will measure the real value.");
                    hslider(ui, true, egui::Slider::new(&mut s.cool_rate_c_s, 0.1..=5.0), "cool °C/s",
                        "Passive cooling rate near printing temperatures — far slower than heating; this sets the long lead times for cooling into a zone. Conservative default until measured.");
                    hslider(ui, true, egui::Slider::new(&mut s.heat_rate_fan_c_s, 0.2..=10.0), "heat °C/s (fan)",
                        "Heating rate with the part fan at 100% — spillover steals heater power. The temp schedule interpolates between the fan-off and fan-on pairs by the filament's fan duty. Follows the fan-off rate until measured.");
                    hslider(ui, true, egui::Slider::new(&mut s.cool_rate_fan_c_s, 0.1..=10.0), "cool °C/s (fan)",
                        "Cooling rate with the part fan at 100% — the realistic in-print case, faster than passive. Follows the fan-off rate until measured.");
                    if ui
                        .add_enabled(host_set && !host_busy, egui::Button::new("⟲ measure thermal response"))
                        .on_hover_text(
                            "Profile the hotend once: homes if needed, parks over bed center at a \
                             clearance height with the bed at the filament's temperature (a pinned, \
                             repeatable environment — bed proximity changes the answer), then steps \
                             the nozzle target around the printing temperature — fan off and at \
                             100% — and fits all four heating/cooling rates. Saves them into the \
                             printer profile. Takes a few minutes; clear the bed first. Never runs \
                             on its own: the temp schedule just reads the saved rates at slice time.",
                        )
                        .on_disabled_hover_text(
                            "Needs a printer host (Connection section) and no other printer operation in flight.",
                        )
                        .clicked()
                    {
                        self.confirm_calibration = true;
                    }
                    hslider(ui, true, egui::Slider::new(&mut s.jerk_mm_s, 1.0..=50.0), "jerk mm/s",
                        "Klipper square-corner-velocity — how briskly direction changes are taken.");
                    ui.checkbox(&mut s.has_aux_fan, "aux part fan (M106 P2)")
                        .on_hover_text("The machine has an auxiliary side part-cooling fan (Sovol Zero, Bambu-style). Unlocks the filament's aux-fan duty; off = M106 P2 is never emitted (vanilla firmware reads it as the primary fan).");
                    ui.checkbox(&mut s.has_exhaust_fan, "exhaust fan (M106 P3)")
                        .on_hover_text("The machine has a chamber-exhaust fan. Unlocks the filament's exhaust duty; off = M106 P3 is never emitted.");
                    ui.horizontal(|ui| {
                        ui.add(egui::TextEdit::singleline(&mut s.chamber_sensor).desired_width(120.0).hint_text("none"));
                        ui.label("chamber sensor").on_hover_text(
                            "The chamber thermistor's Klipper temperature_sensor name — \
                             \"chamber_temp\" on the Sovol Zero, \"chamber\" on a spec Voron \
                             (check Fluidd/Mainsail or `SENSORS` in the console). Empty = the \
                             machine has none; the filament's chamber soak stays locked out.",
                        );
                    });
                });
                tier_section(ui, "Connection", TierKind::Printer, false, |ui| {
                    ui.label("printer host").on_hover_text(
                        "The printer's Moonraker address — e.g. voron24.local or 192.168.1.50. \
                         Plain HTTP is assumed without a scheme. Empty = no connection.",
                    );
                    ui.add(egui::TextEdit::singleline(&mut s.host_url).hint_text("192.168.1.50 or printer.local"));
                    ui.label("API key").on_hover_text(
                        "Only needed when Moonraker's [authorization] section requires one.",
                    );
                    ui.add(egui::TextEdit::singleline(&mut s.api_key).password(true));
                    let testable = !s.host_url.trim().is_empty() && !host_busy;
                    if ui
                        .add_enabled(testable, egui::Button::new("Test connection"))
                        .on_hover_text("Query /server/info and report the Klipper state.")
                        .clicked()
                    {
                        host_op = Some(HostOp::Test);
                    }
                });
                tier_section(ui, "Custom g-code", TierKind::Printer, false, |ui| {
                    ui.label("Start g-code").on_hover_text(
                        "Emitted before the print. Placeholders: {nozzle_temp} {first_layer_nozzle_temp} {bed_temp} {bed_x} {bed_y} {bed_z} {layer_height} {first_layer_height} {nozzle_diameter}.",
                    );
                    ui.add(
                        egui::TextEdit::multiline(&mut s.start_gcode)
                            .code_editor()
                            .desired_rows(4)
                            .desired_width(f32::INFINITY),
                    );
                    ui.label("End g-code").on_hover_text("Emitted after the print (cooldown, park, motors off).");
                    ui.add(
                        egui::TextEdit::multiline(&mut s.end_gcode)
                            .code_editor()
                            .desired_rows(4)
                            .desired_width(f32::INFINITY),
                    );
                });
                ui.add_space(6.0);
                ui.weak("drag: orbit · right-drag: pan · scroll: zoom");
                });
        });

        // Execute any printer-host action requested above, on a worker thread.
        // Frameless: the viewport texture runs edge-to-edge against the panel
        // separator instead of sitting in an 8 pt dark mat.
        egui::CentralPanel::default().frame(egui::Frame::NONE).show_inside(ui, |ui| {
            let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let aspect = rect.width() / rect.height().max(1.0);
            let vp = self.camera.view_proj(aspect);

            // Objects are only editable in Model view; Preview is read-only.
            let edit = !self.view_preview;
            // Ignore viewport input when the cursor is over a floating overlay.
            let pointer = ui.ctx().pointer_interact_pos();
            let over = |r: Option<egui::Rect>| matches!((r, pointer), (Some(r), Some(p)) if r.contains(p));
            let blocked = over(self.overlay_rect)
                || over(self.print_overlay_rect)
                || over(self.msgs_overlay_rect);

            // Left-press on an object grabs it for dragging; on empty space, orbits.
            if edit && !blocked && response.drag_started_by(egui::PointerButton::Primary) {
                self.drag_obj = None;
                if let Some(p) = response.interact_pointer_pos() {
                    let (o, d) = pointer_ray(vp, rect, p);
                    if let Some(i) = self.pick(o, d) {
                        self.selected = Some(i);
                        self.drag_obj = Some(i);
                        if let Some(xy) = ray_plane_z0(o, d) {
                            let pos = self.objects[i].pos;
                            self.drag_grab = [pos[0] - xy.x as f64, pos[1] - xy.y as f64];
                        }
                        self.needs_rebuild = true;
                    }
                }
            }
            if response.dragged_by(egui::PointerButton::Primary) {
                match self.drag_obj {
                    Some(i) => {
                        if let Some(p) = response.interact_pointer_pos() {
                            let (o, d) = pointer_ray(vp, rect, p);
                            if let Some(xy) = ray_plane_z0(o, d) {
                                self.objects[i].pos =
                                    [xy.x as f64 + self.drag_grab[0], xy.y as f64 + self.drag_grab[1]];
                                self.needs_rebuild = true;
                                self.sliced = None;
                                self.slice_summary = None;
                                self.view_preview = false;
                            }
                        }
                    }
                    None => {
                        if !blocked {
                            let d = response.drag_delta();
                            self.camera.orbit(d.x, d.y);
                        }
                    }
                }
            }
            if response.drag_stopped_by(egui::PointerButton::Primary) {
                self.drag_obj = None;
            }
            // A plain click selects the object under the cursor, or deselects on empty.
            if edit && !blocked && response.clicked() {
                if let Some(p) = response.interact_pointer_pos() {
                    let (o, d) = pointer_ray(vp, rect, p);
                    self.selected = self.pick(o, d);
                    self.needs_rebuild = true;
                }
            }
            if response.dragged_by(egui::PointerButton::Secondary) {
                let d = response.drag_delta();
                self.camera.pan(d.x, d.y);
            }
            if response.hovered() && !blocked {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(scroll);
                }
            }

            let ppp = ui.ctx().pixels_per_point();
            let w = (rect.width() * ppp).round().max(1.0) as u32;
            let h = (rect.height() * ppp).round().max(1.0) as u32;
            self.scene.resize(&rs, w, h);
            let show_mesh = !(self.view_preview && self.sliced.is_some());
            let preview = if self.view_preview && self.sliced.is_some() {
                let n = self.layer_ends.len();
                let idx = self.preview_layer.saturating_sub(1);
                let count = self.layer_ends.get(idx).copied().unwrap_or(0);
                let joint_count = self.joint_layer_ends.get(idx).copied().unwrap_or(0);
                // Dim lower layers only when the slider is below the top.
                let dim = if self.preview_layer >= n { 1.0 } else { 0.15 };
                Some(render::Preview {
                    count,
                    joint_count,
                    current_layer: self.preview_layer as f32,
                    dim,
                    mask: self.category_mask(),
                })
            } else {
                None
            };
            let (mesh_unsel, mesh_sel) = mesh_tints(self.accent);
            self.scene.render(&rs, vp, show_mesh, preview, mesh_unsel, mesh_sel);

            ui.painter().image(
                self.scene.texture_id(),
                rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );

            // Floating translucent transform panel — only while an object is selected
            // and we're in Model view (Preview is read-only).
            if let (Some(i), false) = (self.selected, self.view_preview) {
                let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
                let mut changed = false;
                let area = egui::Area::new(egui::Id::new("transform_overlay"))
                    .order(egui::Order::Foreground)
                    .fixed_pos(rect.min + egui::vec2(10.0, 10.0))
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style())
                            .fill(egui::Color32::from_rgba_unmultiplied(26, 22, 17, 220))
                            .show(ui, |ui| {
                                ui.set_max_width(210.0);
                                let obj = &mut self.objects[i];
                                ui.label(egui::RichText::new(obj.name.as_str()).strong());
                                ui.horizontal(|ui| {
                                    ui.label("move");
                                    changed |= ui.add(egui::DragValue::new(&mut obj.pos[0]).speed(0.5).prefix("X ")).changed();
                                    changed |= ui.add(egui::DragValue::new(&mut obj.pos[1]).speed(0.5).prefix("Y ")).changed();
                                });
                                ui.horizontal(|ui| {
                                    ui.label("rot°");
                                    changed |= ui.add(egui::DragValue::new(&mut obj.rot_deg[0]).speed(1.0).prefix("X ")).changed();
                                    changed |= ui.add(egui::DragValue::new(&mut obj.rot_deg[1]).speed(1.0).prefix("Y ")).changed();
                                    changed |= ui.add(egui::DragValue::new(&mut obj.rot_deg[2]).speed(1.0).prefix("Z ")).changed();
                                });
                                changed |= ui.add(egui::Slider::new(&mut obj.scale, 0.1..=5.0).text("scale")).changed();
                                ui.horizontal(|ui| {
                                    if ui.button("Center").clicked() {
                                        obj.pos = [bx / 2.0, by / 2.0];
                                        changed = true;
                                    }
                                    if ui.button("Reset rot").clicked() {
                                        obj.rot_deg = [0.0; 3];
                                        changed = true;
                                    }
                                });
                            });
                    });
                self.overlay_rect = Some(area.response.rect);
                if changed {
                    self.needs_rebuild = true;
                    self.sliced = None;
                    self.slice_summary = None;
                    self.view_preview = false;
                }
            } else {
                self.overlay_rect = None;
            }

            // Live-print card: translucent, top-right of the viewport, shown
            // once a file has been sent. The state refreshes itself on a timer
            // (quiet polls), so there's no manual status button. Its ✖ hides
            // the card (and stops the polls) until the next send.
            if self.sent_to_printer && host_set {
                let state = self
                    .printer_status
                    .as_ref()
                    .and_then(|r| r.as_ref().ok())
                    .map(|st| st.state.as_str())
                    .unwrap_or("");
                let mut hide_card = false;
                let area = egui::Area::new(egui::Id::new("print_overlay"))
                    .order(egui::Order::Foreground)
                    .fixed_pos(egui::pos2(rect.right() - 240.0, rect.top() + 10.0))
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style())
                            .fill(egui::Color32::from_rgba_unmultiplied(26, 22, 17, 196))
                            .show(ui, |ui| {
                                ui.set_width(220.0);
                                // Header: title left (fixed width, truncating)
                                // with the dismiss ✖ to its right. No fill
                                // layouts in here: an Area hands its content
                                // LAST frame's rect as the available space, so
                                // anything that centers or justifies against
                                // it re-measures bigger every repaint and the
                                // card grows ~1 Hz with the status polls.
                                ui.horizontal(|ui| {
                                    let title = match &self.printer_status {
                                        Some(Ok(st)) if !st.filename.is_empty() => st.filename.as_str(),
                                        Some(Ok(_)) => "(no file)",
                                        _ => "Printer",
                                    };
                                    ui.scope(|ui| {
                                        ui.set_width(194.0);
                                        ui.add(egui::Label::new(egui::RichText::new(title).strong()).truncate());
                                    });
                                    if ui
                                        .small_button("✖")
                                        .on_hover_text("Hide this card. Sending to the printer again brings it back.")
                                        .clicked()
                                    {
                                        hide_card = true;
                                    }
                                });
                                match &self.printer_status {
                                    None => {
                                        ui.weak("checking…");
                                    }
                                    Some(Err(e)) => {
                                        ui.colored_label(ui.visuals().error_fg_color, e);
                                    }
                                    Some(Ok(st)) => {
                                        if st.state == "printing" || st.state == "paused" {
                                            ui.add(egui::ProgressBar::new(st.progress as f32).show_percentage());
                                        }
                                        ui.weak(&st.state);
                                    }
                                }
                                ui.horizontal(|ui| {
                                    let live = !host_busy;
                                    if ui
                                        .add_enabled(live && state == "printing", egui::Button::new("⏸"))
                                        .on_hover_text("Pause the running print.")
                                        .clicked()
                                    {
                                        host_op = Some(HostOp::Pause);
                                    }
                                    if ui
                                        .add_enabled(live && state == "paused", egui::Button::new("▶"))
                                        .on_hover_text("Resume the paused print.")
                                        .clicked()
                                    {
                                        host_op = Some(HostOp::Resume);
                                    }
                                    if ui
                                        .add_enabled(
                                            live && (state == "printing" || state == "paused"),
                                            egui::Button::new("✖"),
                                        )
                                        .on_hover_text("Cancel the running print.")
                                        .clicked()
                                    {
                                        host_op = Some(HostOp::Cancel);
                                    }
                                });
                            });
                    });
                if hide_card {
                    // Dismissed: drop the card and the polling behind it; the
                    // next send re-arms both.
                    self.sent_to_printer = false;
                    self.printer_status = None;
                    self.print_overlay_rect = None;
                } else {
                    self.print_overlay_rect = Some(area.response.rect);
                }
            } else {
                self.print_overlay_rect = None;
            }

            // Messages pane: the one-line status plus the last slice's
            // summary, translucent, bottom-left of the viewport. ✖ hides it;
            // any new message or a fresh slice brings it back.
            let show_msgs = (!self.status.is_empty() || self.slice_summary.is_some())
                && self
                    .msgs_dismissed
                    .as_ref()
                    .map_or(true, |(s, g)| *s != self.status || *g != self.slice_gen);
            if show_msgs {
                let mut dismiss = false;
                let area = egui::Area::new(egui::Id::new("messages_overlay"))
                    .order(egui::Order::Foreground)
                    .pivot(egui::Align2::LEFT_BOTTOM)
                    .fixed_pos(rect.left_bottom() + egui::vec2(10.0, -10.0))
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style())
                            .fill(egui::Color32::from_rgba_unmultiplied(26, 22, 17, 196))
                            .show(ui, |ui| {
                                ui.horizontal_top(|ui| {
                                    // The content inherits this row's
                                    // left-to-right layout unless re-rooted in
                                    // a vertical column — labels would render
                                    // over each other on one line.
                                    //
                                    // No ScrollArea (or anything else sized by
                                    // available height) in here: a bottom-
                                    // pivoted Area hands its content last
                                    // frame's height as the available space,
                                    // so height-adaptive content locks the
                                    // pane at its collapsed size instead of
                                    // growing when a section expands. Natural
                                    // sizing measures true height and the
                                    // pivot re-anchors; if it ever outgrows
                                    // the window, the Area clamps to the top
                                    // and the collapsibles default closed.
                                    ui.vertical(|ui| {
                                        ui.set_width(290.0);
                                        self.slice_messages(ui);
                                    });
                                    if ui
                                        .small_button("✖")
                                        .on_hover_text(
                                            "Hide these messages. The next slice or status message brings them back.",
                                        )
                                        .clicked()
                                    {
                                        dismiss = true;
                                    }
                                });
                            });
                    });
                self.msgs_overlay_rect = Some(area.response.rect);
                if dismiss {
                    self.msgs_dismissed = Some((self.status.clone(), self.slice_gen));
                }
            } else {
                self.msgs_overlay_rect = None;
            }
        });

        // A profile switch while edits are unsaved: confirm the discard.
        if let Some((p, f, pr)) = self.pending_switch.clone() {
            let mut act = false;
            let mut open = true;
            egui::Window::new("Discard unsaved changes?")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, -40.0])
                .show(ui.ctx(), |ui| {
                    ui.label(
                        "Switching profiles re-reads settings from disk — the edits marked \
                         with * would be lost.",
                    );
                    ui.label("Save them first (💾 next to the tier), or discard and switch.");
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("Discard & switch").clicked() {
                            act = true;
                            open = false;
                        }
                        if ui.button("Cancel").clicked() {
                            open = false;
                        }
                    });
                });
            if act {
                if p != self.printer {
                    self.recenter_camera = true;
                }
                self.printer = p;
                self.filament = f;
                self.process = pr;
                self.reresolve();
            }
            if !open {
                self.pending_switch = None;
            }
        }

        // Thermal calibration runs heaters and motion — describe exactly what
        // is about to happen and ask before doing any of it.
        if self.confirm_calibration {
            let (base, top, park) = self.calibration_plan();
            let bed = self.settings.bed_temp_c;
            let mut open = true;
            egui::Window::new("Measure thermal response?")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, -40.0])
                .show(ui.ctx(), |ui| {
                    ui.label("This operates the printer for a few minutes:");
                    if bed > 0 {
                        ui.label(format!(
                            "• heats the bed to {bed} °C and the nozzle between {base:.0} and {top:.0} °C"
                        ));
                    } else {
                        ui.label(format!("• heats the nozzle between {base:.0} and {top:.0} °C"));
                    }
                    ui.label(format!(
                        "• homes if needed, then parks over bed center at Z {:.0} mm",
                        park.2
                    ));
                    ui.label("• runs the part fan at 100% for half the run");
                    ui.label("The nozzle may ooze a little if filament is loaded.");
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new("Clear the bed and make sure the printer is idle.").strong(),
                    );
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        if ui.button("Start calibration").clicked() {
                            host_op = Some(HostOp::CalibrateThermal);
                            open = false;
                        }
                        if ui.button("Cancel").clicked() {
                            open = false;
                        }
                    });
                });
            self.confirm_calibration = open;
        }

        // Dispatch host actions after both panels have run — the left panel
        // and the live-print overlay both request them via `host_op`.
        if let Some(op) = host_op {
            let ctx = ui.ctx().clone();
            match op {
                HostOp::Test => self.spawn_host_op(&ctx, false, |c| {
                    HostReply::Message(match c.server_info() {
                        Ok(state) => format!("Printer reachable — Klipper is {state}."),
                        Err(e) => format!("Connection failed: {e}"),
                    })
                }),
                HostOp::Send { start } => {
                    if let Some(layers) = self.sliced.as_ref() {
                        let gcode = engine::to_gcode(layers, &self.settings);
                        let filename = self.upload_filename();
                        self.spawn_host_op(&ctx, false, move |c| {
                            match c.upload(&filename, gcode.as_bytes(), start) {
                                Ok(()) if start => HostReply::SendDone { ok: true, msg: format!("Printing {filename}.") },
                                Ok(()) => HostReply::SendDone { ok: true, msg: format!("Uploaded {filename}.") },
                                Err(e) => HostReply::SendDone { ok: false, msg: format!("Upload failed: {e}") },
                            }
                        });
                    }
                }
                HostOp::Pause => self.spawn_host_op(&ctx, false, |c| {
                    HostReply::Message(match c.pause() {
                        Ok(()) => "Print paused.".into(),
                        Err(e) => format!("Pause failed: {e}"),
                    })
                }),
                HostOp::Resume => self.spawn_host_op(&ctx, false, |c| {
                    HostReply::Message(match c.resume() {
                        Ok(()) => "Print resumed.".into(),
                        Err(e) => format!("Resume failed: {e}"),
                    })
                }),
                HostOp::Cancel => self.spawn_host_op(&ctx, false, |c| {
                    HostReply::Message(match c.cancel() {
                        Ok(()) => "Print cancelled.".into(),
                        Err(e) => format!("Cancel failed: {e}"),
                    })
                }),
                HostOp::Status => {
                    self.last_status_poll = Some(std::time::Instant::now());
                    self.spawn_host_op(&ctx, true, |c| HostReply::Status(c.print_status()));
                }
                HostOp::CalibrateThermal => self.spawn_thermal_calibration(&ctx),
            }
        }

        // Save / delete profile dialog (floats over the viewport).
        if let Some(mut dlg) = self.profile_dialog.take() {
            let mut keep = true;
            let mut act = false;
            let title = if dlg.delete { "Delete profile" } else { "Save profile" };
            egui::Window::new(title)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, -40.0])
                .show(ui.ctx(), |ui| {
                    let tier = egui::RichText::new(dlg.kind.label())
                        .color(tier_color(dlg.kind))
                        .strong();
                    if dlg.delete {
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Delete the");
                            ui.label(tier);
                            ui.label(format!("profile '{}' from disk?", dlg.name));
                        });
                    } else {
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Save as a");
                            ui.label(tier);
                            ui.label(format!(
                                "profile (inherits '{}', stores only changed fields):",
                                match dlg.kind {
                                    TierKind::Printer => &self.printer,
                                    TierKind::Filament => &self.filament,
                                    TierKind::Process => &self.process,
                                }
                            ));
                        });
                        ui.text_edit_singleline(&mut dlg.name);
                    }
                    ui.horizontal(|ui| {
                        let verb = if dlg.delete { "Delete" } else { "Save" };
                        if ui.button(verb).clicked() {
                            act = true;
                        }
                        if ui.button("Cancel").clicked() {
                            keep = false;
                        }
                    });
                });
            if act {
                let result = if dlg.delete {
                    self.delete_profile(dlg.kind, &dlg.name)
                } else {
                    self.save_profile(dlg.kind, &dlg.name)
                };
                match result {
                    Ok(()) => {
                        let verb = if dlg.delete { "Deleted" } else { "Saved" };
                        let dir = self
                            .profiles
                            .user_dir()
                            .map(|d| format!(" ({})", d.display()))
                            .unwrap_or_default();
                        self.status = format!("{verb} {} profile '{}'{dir}", dlg.kind.label(), dlg.name);
                        keep = false;
                    }
                    Err(e) => {
                        self.status = format!("Profile error: {e}");
                        keep = !dlg.delete; // keep the save dialog open to fix the name
                    }
                }
            }
            if keep {
                self.profile_dialog = Some(dlg);
            }
        }

        // Whatever changed the selections this frame — combo switch, save
        // dialog, calibration move, delete fallback — lands in the dotfile
        // state, so the next launch starts where this one left off.
        self.persist_state();
    }
}

fn pattern_combo(ui: &mut egui::Ui, label: &str, current: &mut config::InfillPattern) -> egui::Response {
    use config::InfillPattern::*;
    egui::ComboBox::from_label(label)
        .selected_text(current.label())
        .show_ui(ui, |ui| {
            ui.selectable_value(current, Lines, "lines");
            ui.selectable_value(current, Grid, "grid");
            ui.selectable_value(current, Triangles, "triangles");
            ui.selectable_value(current, Concentric, "concentric");
            ui.selectable_value(current, Gyroid, "gyroid");
        })
        .response
}

/// World-space ray (origin, normalized dir) through a screen position in `rect`.
fn pointer_ray(vp: glam::Mat4, rect: egui::Rect, pos: egui::Pos2) -> (glam::Vec3, glam::Vec3) {
    let ndc_x = 2.0 * (pos.x - rect.left()) / rect.width().max(1.0) - 1.0;
    let ndc_y = 1.0 - 2.0 * (pos.y - rect.top()) / rect.height().max(1.0);
    let inv = vp.inverse();
    let near = inv.project_point3(glam::Vec3::new(ndc_x, ndc_y, 0.0));
    let far = inv.project_point3(glam::Vec3::new(ndc_x, ndc_y, 1.0));
    (near, (far - near).normalize_or_zero())
}

/// Where a ray meets the bed plane z=0 (None if parallel or behind the origin).
fn ray_plane_z0(o: glam::Vec3, d: glam::Vec3) -> Option<glam::Vec2> {
    if d.z.abs() < 1e-6 {
        return None;
    }
    let t = -o.z / d.z;
    (t >= 0.0).then(|| (o + d * t).truncate())
}

/// Möller–Trumbore ray/triangle hit distance (either face), if the ray hits it.
fn ray_triangle(o: glam::Vec3, d: glam::Vec3, a: glam::Vec3, b: glam::Vec3, c: glam::Vec3) -> Option<f32> {
    let (e1, e2) = (b - a, c - a);
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-7 {
        return None;
    }
    let inv = 1.0 / det;
    let tv = o - a;
    let u = tv.dot(p) * inv;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = tv.cross(e1);
    let v = d.dot(q) * inv;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > 1e-4).then_some(t)
}

fn seam_combo(ui: &mut egui::Ui, current: &mut config::SeamMode) -> egui::Response {
    use config::SeamMode::*;
    egui::ComboBox::from_label("seam")
        .selected_text(current.label())
        .show_ui(ui, |ui| {
            ui.selectable_value(current, Nearest, "nearest");
            ui.selectable_value(current, Aligned, "aligned");
            ui.selectable_value(current, Sharpest, "sharpest");
            ui.selectable_value(current, Random, "random");
        })
        .response
}

fn support_combo(ui: &mut egui::Ui, current: &mut config::SupportMode) -> egui::Response {
    use config::SupportMode::*;
    egui::ComboBox::from_label("support")
        .selected_text(current.label())
        .show_ui(ui, |ui| {
            ui.selectable_value(current, None, "none");
            ui.selectable_value(current, Grid, "grid");
            ui.selectable_value(current, Arc, "arc");
        })
        .response
}

/// Profile picker: a combo with a stable id (the colored label text changes
/// with the dirty `*`, so it can't double as the widget id).
/// Flatten sliced layers into line-segment vertices (`[x,y,z,r,g,b]`, consecutive
/// pairs = segments) plus a cumulative per-layer vertex count for the layer slider.
// Category ids — must match the bit positions in `App::category_mask`.
const CAT_SKIRT: f32 = 0.0;
const CAT_WALLS: f32 = 1.0;
const CAT_SOLID: f32 = 2.0;
const CAT_INFILL: f32 = 3.0;
const CAT_TRAVEL: f32 = 4.0;
const CAT_SEAM: f32 = 5.0;
const CAT_SUPPORT: f32 = 6.0;
const CAT_GAPFILL: f32 = 7.0;
const CAT_IRONING: f32 = 8.0;

/// Flatten sliced layers into bead instances (one per extrusion/travel segment)
/// plus joint blobs (one per extrusion vertex, to round ends and fill corners),
/// each with a cumulative per-layer count for the layer slider.
/// Bead:  `[p0.xyz, dir.xy, len, width, height, r, g, b, layer, category]`.
/// Joint: `[p.xyz, width, height, r, g, b, layer, category]`.
type Instances = (Vec<[f32; 13]>, Vec<u32>, Vec<[f32; 10]>, Vec<u32>);
fn build_instances(
    layers: &[engine::LayerPlan],
    z_hop_mm: f32,
    path_colors: Option<&[Vec<[f32; 3]>]>,
    accent: (f32, f32, f32),
) -> Instances {
    let mut inst: Vec<[f32; 13]> = Vec::new();
    let mut ends: Vec<u32> = Vec::with_capacity(layers.len());
    let mut joints: Vec<[f32; 10]> = Vec::new();
    let mut joint_ends: Vec<u32> = Vec::with_capacity(layers.len());
    let (ah, as_, _) = accent;
    // Travels whisper on the complement (hairline, usually toggled off);
    // seams scream on it (debug dots must pop against the accent shell).
    let travel_color = hsl_to_rgb(ah + 180.0, as_ * 0.30, 0.62);
    let seam_color = hsl_to_rgb(ah + 180.0, (as_ * 0.90).clamp(0.0, 0.9), 0.55);
    let travel_dim = 0.08_f32;
    let mut prev_end: Option<geo2d::Point> = None;

    for (li, layer) in layers.iter().enumerate() {
        let layer_id = (li + 1) as f32; // 1-based, matches preview_layer
        let z_top = layer.print_z_mm as f32;
        let h = layer.height_mm as f32;

        for (pi, path) in layer.paths.iter().enumerate() {
            if path.points.len() < 2 {
                continue;
            }
            // Render the planned travel: the combed route (around holes/walls),
            // raised when it z-hops over a void.
            if let (Some(pe), Some(tr)) = (prev_end, layer.travels.get(pi)) {
                let zc = if tr.hop { z_top + z_hop_mm } else { z_top } - travel_dim * 0.5;
                let mut from = pe;
                for &pt in &tr.points {
                    push_inst(&mut inst, from, pt, zc, travel_dim, travel_dim, travel_color, layer_id, CAT_TRAVEL);
                    from = pt;
                }
            }
            // Heat-map modes override the feature palette per path (per island).
            let c = path_colors.map_or_else(|| color_for(path.kind, accent), |t| t[li][pi]);
            let cat = category_of(path.kind);
            // Brick-layered perimeters render half a layer up (z_offset) and a touch
            // fatter (flow > 1), so the staggered, over-packed walls are visible.
            // Trickle-flow paths (ironing) render as a thin film at the layer top
            // instead: full width, height scaled by flow.
            let base_h = h * path.height_scale as f32; // half-height outer walls
            let (w, bh) = if path.flow >= 1.0 {
                ((path.width_mm * path.flow) as f32, base_h)
            } else {
                (path.width_mm as f32, (base_h * path.flow as f32).max(0.04))
            };
            let zc = z_top - bh * 0.5 + path.z_offset_mm as f32;
            // Variable-width (arachne) beads render their true per-segment width.
            let seg_w = |k: usize| -> f32 {
                match &path.widths {
                    Some(ws) => (0.5 * (ws[k] + ws[(k + 1) % ws.len()])) as f32,
                    None => w,
                }
            };
            let n_pts = path.points.len();
            for k in 0..n_pts - 1 {
                push_inst(&mut inst, path.points[k], path.points[k + 1], zc, seg_w(k), bh, c, layer_id, cat);
            }
            if path.closed {
                push_inst(&mut inst, path.points[n_pts - 1], path.points[0], zc, seg_w(n_pts - 1), bh, c, layer_id, cat);
            }
            // Joint blob at every vertex (extrusion paths only — travels stay bare).
            for (k, p) in path.points.iter().enumerate() {
                let jw = match &path.widths {
                    Some(ws) => ws[k] as f32,
                    None => w,
                };
                joints.push([
                    p.x_mm() as f32, p.y_mm() as f32, zc,
                    jw, bh,
                    c[0], c[1], c[2],
                    layer_id, cat,
                ]);
            }
            // Highlight the external-perimeter seam (loop start) with a larger
            // complement-colored marker, toggleable via the "seams" category.
            // Only closed
            // loops have a seam — the open pieces of an overhang-split wall
            // start mid-loop wherever the split fell, and marking those reads
            // as scatter that no seam strategy could fix.
            if path.kind == engine::PathKind::ExternalPerimeter && path.closed {
                let s = path.points[0];
                joints.push([
                    s.x_mm() as f32, s.y_mm() as f32, zc,
                    w * 2.5, h * 2.5,
                    seam_color[0], seam_color[1], seam_color[2],
                    layer_id, CAT_SEAM,
                ]);
            }
            prev_end = Some(if path.closed {
                path.points[0]
            } else {
                path.points[path.points.len() - 1]
            });
        }
        ends.push(inst.len() as u32);
        joint_ends.push(joints.len() as u32);
    }
    (inst, ends, joints, joint_ends)
}

#[allow(clippy::too_many_arguments)]
fn push_inst(
    v: &mut Vec<[f32; 13]>,
    a: geo2d::Point,
    b: geo2d::Point,
    z_center: f32,
    width: f32,
    height: f32,
    color: [f32; 3],
    layer: f32,
    cat: f32,
) {
    let (ax, ay) = (a.x_mm() as f32, a.y_mm() as f32);
    let (bx, by) = (b.x_mm() as f32, b.y_mm() as f32);
    let (dx, dy) = (bx - ax, by - ay);
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1.0e-4 {
        return;
    }
    v.push([
        ax, ay, z_center,
        dx / len, dy / len, len,
        width, height,
        color[0], color[1], color[2],
        layer, cat,
    ]);
}

fn category_of(kind: engine::PathKind) -> f32 {
    use engine::PathKind::*;
    match kind {
        Skirt => CAT_SKIRT,
        ExternalPerimeter | Perimeter | OverhangWall => CAT_WALLS,
        Solid | TopSkin | BottomSkin => CAT_SOLID,
        Infill => CAT_INFILL,
        GapFill => CAT_GAPFILL,
        Ironing => CAT_IRONING,
        Support | Bridge | InternalBridge | ArcOverhang => CAT_SUPPORT,
    }
}

fn color_for(kind: engine::PathKind, accent: (f32, f32, f32)) -> [f32; 3] {
    use engine::PathKind::*;
    // The categorical palette, derived from the one accent hue. Structure:
    // the printed shell reads as paper (near-cream with a whisper of the
    // accent), solid surfaces are the accent family ordered by lightness
    // (bright crown → core → dark underside), the analogous neighbors ±40°
    // carry infill and gap fill, and auxiliary material (support/bridge
    // family) sits on the complement — unmistakably "other" whatever hue
    // drives the scheme. Feature view stays flat blocks, so it never
    // masquerades as a heat map (which is gradients).
    let (h, s, _) = accent;
    let col = |dh: f32, sm: f32, l: f32| hsl_to_rgb(h + dh, (s * sm).clamp(0.0, 0.95), l);
    match kind {
        Skirt => col(0.0, 0.08, 0.42),         // near-neutral — peripheral
        ExternalPerimeter => col(0.0, 0.18, 0.86), // paper shell
        Perimeter => col(0.0, 0.30, 0.56),     // the wall family's shadow step
        OverhangWall => col(0.0, 1.0, 0.42),   // deepest + fully saturated: walls over air
        Solid => col(0.0, 0.80, 0.52),         // the accent's core
        TopSkin => col(0.0, 0.90, 0.68),       // the crown — the accent at its brightest
        BottomSkin => col(0.0, 0.70, 0.36),    // dark underside
        Infill => col(40.0, 0.45, 0.54),       // analogous step one way — recedes
        GapFill => col(-40.0, 0.55, 0.50),     // analogous step the other way
        Ironing => col(0.0, 0.30, 0.78),       // pale sheen over the top skin
        Support => col(180.0, 0.35, 0.48),     // complement, muted — auxiliary material
        Bridge => col(180.0, 0.55, 0.58),      // complement, brighter — spans over air
        InternalBridge => col(180.0, 0.55, 0.40), // complement, deep — spans over infill
        ArcOverhang => col(150.0, 0.50, 0.55), // just off the complement — arc fans
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_plane_and_triangle_math() {
        let down = glam::Vec3::new(0.0, 0.0, -1.0);
        // Straight down at (5,5) meets z=0 at (5,5).
        let xy = ray_plane_z0(glam::Vec3::new(5.0, 5.0, 10.0), down).unwrap();
        assert!((xy.x - 5.0).abs() < 1e-4 && (xy.y - 5.0).abs() < 1e-4);
        // Triangle (0,0)-(10,0)-(0,10) on z=0: hit inside at (2,2), miss at (20,20).
        let (a, b, c) =
            (glam::Vec3::ZERO, glam::Vec3::new(10.0, 0.0, 0.0), glam::Vec3::new(0.0, 10.0, 0.0));
        let t = ray_triangle(glam::Vec3::new(2.0, 2.0, 5.0), down, a, b, c).unwrap();
        assert!((t - 5.0).abs() < 1e-4, "t={t}");
        assert!(ray_triangle(glam::Vec3::new(20.0, 20.0, 5.0), down, a, b, c).is_none());
    }

    #[test]
    fn euler_identity_and_z_rotation() {
        assert_eq!(euler_matrix([0.0; 3]), mesh::Transform::IDENTITY.rotation);
        // 90° about Z maps +X to +Y.
        let r = euler_matrix([0.0, 0.0, 90.0]);
        let t = mesh::Transform { rotation: r, ..Default::default() };
        let p = t.apply_linear([1.0, 0.0, 0.0]);
        assert!((p[0]).abs() < 1e-9 && (p[1] - 1.0).abs() < 1e-9, "{p:?}");
    }
}
