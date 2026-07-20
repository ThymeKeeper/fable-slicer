//! `slicer-gui` — desktop GUI: import STL/3MF models as a multi-object scene,
//! lay them out on the bed, choose profiles, slice, preview toolpaths in 3D,
//! and export g-code.

mod camera;
mod offscreen;
mod paint;
mod render;

use camera::Camera;
use eframe::egui;
use render::Scene;

use config::{FilamentProfile, PrinterProfile, ProcessProfile, Profiles, Settings, Tier, TierKind};
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
    /// A toolchanger tab's save/delete: the slot whose per-tool diff (and
    /// inherits parent) the save uses. None = the classic tier-row flow.
    slot: Option<usize>,
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
}

/// What the preview colors encode.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ColorBy {
    /// Feature type (walls, infill, …) — the classic view.
    Feature,
    /// Per-layer print time: red = quick layers, little cooling before the
    /// nozzle returns.
    LayerTime,
    /// Each path in its printing tool's filament color (toolchangers only).
    Filament,
}

/// Which derivable settings the user has pinned to manual values. Unpinned
/// fields recompute live from their master setting every frame (camera-style
/// "priority mode": auto until touched, visible either way).
#[derive(Default, Clone, Copy)]
struct Pins {
    line_width: bool,
    outer_wall_accel: bool,
    first_layer_accel: bool,
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
    profile_pinned: bool,
    baseline: f64,
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
        // ⟲ = revert to what the PROFILE says — its pinned value when it
        // pins this field, auto otherwise. Clean rows (auto following, or
        // sitting exactly on the profile's pin) show the weak "auto" tag or
        // nothing, same as every other row's revert.
        let clean = if profile_pinned { *pinned && *value == baseline } else { !*pinned };
        if !clean {
            let hover = if profile_pinned {
                format!("Edited — click to revert to the profile's {baseline:.2}.")
            } else {
                format!(
                    "Pinned manually. Click to return to auto ({derived:.2}) and follow the master setting again."
                )
            };
            if ui.small_button("⟲").on_hover_text(hover).clicked() {
                *pinned = profile_pinned;
                *value = if profile_pinned { baseline } else { derived };
            }
        } else if !*pinned {
            ui.label(egui::RichText::new("auto").small().weak())
                .on_hover_text("Following its master setting — drag the slider to pin a manual value.");
        }
    });
}

/// The flow-triangle ceiling spelled out with live numbers, for tooltips —
/// names every participant so the relationship is learnable from any corner:
/// "max flow 21.0 mm³/s ÷ bead (line width 0.45 × layer height 0.20 mm) = ~258 mm/s".
fn flow_ceiling_text(s: &config::Settings) -> String {
    flow_ceiling_parts_text(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm)
}

/// `flow_ceiling_text` from the raw triangle numbers — the toolchanger tabs
/// speak it for the active slot's own melt ceiling.
fn flow_ceiling_parts_text(max_flow_mm3_s: f64, line_width_mm: f64, layer_height_mm: f64) -> String {
    let cap = config::flow_speed_cap_mm_s(max_flow_mm3_s, line_width_mm, layer_height_mm);
    if cap.is_finite() {
        format!(
            "max flow {:.1} mm³/s ÷ bead (line width {:.2} × layer height {:.2} mm) = ~{:.0} mm/s",
            max_flow_mm3_s, line_width_mm, layer_height_mm, cap
        )
    } else {
        "max flow 0 = unlimited".into()
    }
}


/// One editable row plus its per-row revert: draws the row, then — when the
/// value has strayed from the resolved profile chain — a trailing ⟲ that
/// restores the profile's value. The tier rows' * says a tier carries edits;
/// this says WHICH row, and undoes just that one. Rows whose edits need side
/// effects (tool count) diff the value around the call, so a revert triggers
/// them the same as a drag.
fn revert_row<T: PartialEq + Clone>(
    ui: &mut egui::Ui,
    value: &mut T,
    baseline: &T,
    row: impl FnOnce(&mut egui::Ui, &mut T),
) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 5.0;
        row(ui, value);
        if *value != *baseline
            && ui
                .small_button("⟲")
                .on_hover_text("Edited — click to revert to the profile's value.")
                .clicked()
        {
            *value = baseline.clone();
        }
    });
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

/// &mut views of every field the Filament card and Cooling section edit.
/// One set of widget rows serves both surfaces: the flat fields on a
/// single-tool machine, or the active tab's slot of `settings.tools` on a
/// toolchanger — `config::ToolSettings` mirrors the flat filament tier
/// field-for-field, so both constructors are total.
struct FilamentFields<'a> {
    nozzle_temp_c: &'a mut u32,
    standby_temp_c: &'a mut u32,
    bed_temp_c: &'a mut u32,
    chamber_temp_c: &'a mut u32,
    filament_diameter_mm: &'a mut f64,
    extrusion_multiplier: &'a mut f64,
    max_volumetric_speed_mm3_s: &'a mut f64,
    pressure_advance: &'a mut f64,
    bridge_flow: &'a mut f64,
    bridge_speed_mm_s: &'a mut f64,
    fan_speed: &'a mut f64,
    bridge_fan_speed: &'a mut f64,
    fan_off_layers: &'a mut usize,
    aux_fan_speed: &'a mut f64,
    exhaust_fan_speed: &'a mut f64,
}

impl<'a> FilamentFields<'a> {
    /// The single-tool surface: the flat fields, exactly as before.
    fn flat(s: &'a mut Settings) -> Self {
        Self {
            nozzle_temp_c: &mut s.nozzle_temp_c,
            standby_temp_c: &mut s.standby_temp_c,
            bed_temp_c: &mut s.bed_temp_c,
            chamber_temp_c: &mut s.chamber_temp_c,
            filament_diameter_mm: &mut s.filament_diameter_mm,
            extrusion_multiplier: &mut s.extrusion_multiplier,
            max_volumetric_speed_mm3_s: &mut s.max_volumetric_speed_mm3_s,
            pressure_advance: &mut s.pressure_advance,
            bridge_flow: &mut s.bridge_flow,
            bridge_speed_mm_s: &mut s.bridge_speed_mm_s,
            fan_speed: &mut s.fan_speed,
            bridge_fan_speed: &mut s.bridge_fan_speed,
            fan_off_layers: &mut s.fan_off_layers,
            aux_fan_speed: &mut s.aux_fan_speed,
            exhaust_fan_speed: &mut s.exhaust_fan_speed,
        }
    }

    /// One toolchanger slot's surface (the active tab).
    fn tool(t: &'a mut config::ToolSettings) -> Self {
        Self {
            nozzle_temp_c: &mut t.nozzle_temp_c,
            standby_temp_c: &mut t.standby_temp_c,
            bed_temp_c: &mut t.bed_temp_c,
            chamber_temp_c: &mut t.chamber_temp_c,
            filament_diameter_mm: &mut t.filament_diameter_mm,
            extrusion_multiplier: &mut t.extrusion_multiplier,
            max_volumetric_speed_mm3_s: &mut t.max_volumetric_speed_mm3_s,
            pressure_advance: &mut t.pressure_advance,
            bridge_flow: &mut t.bridge_flow,
            bridge_speed_mm_s: &mut t.bridge_speed_mm_s,
            fan_speed: &mut t.fan_speed,
            bridge_fan_speed: &mut t.bridge_fan_speed,
            fan_off_layers: &mut t.fan_off_layers,
            aux_fan_speed: &mut t.aux_fan_speed,
            exhaust_fan_speed: &mut t.exhaust_fan_speed,
        }
    }
}

/// The profile-resolved values behind [`FilamentFields`], by value — each
/// card row compares against its baseline twin and offers the per-row ⟲.
struct FilamentBaseline {
    nozzle_temp_c: u32,
    standby_temp_c: u32,
    bed_temp_c: u32,
    chamber_temp_c: u32,
    filament_diameter_mm: f64,
    extrusion_multiplier: f64,
    max_volumetric_speed_mm3_s: f64,
    pressure_advance: f64,
    bridge_flow: f64,
    bridge_speed_mm_s: f64,
    fan_speed: f64,
    bridge_fan_speed: f64,
    fan_off_layers: usize,
    aux_fan_speed: f64,
    exhaust_fan_speed: f64,
}

impl FilamentBaseline {
    fn flat(s: &Settings) -> Self {
        Self {
            nozzle_temp_c: s.nozzle_temp_c,
            standby_temp_c: s.standby_temp_c,
            bed_temp_c: s.bed_temp_c,
            chamber_temp_c: s.chamber_temp_c,
            filament_diameter_mm: s.filament_diameter_mm,
            extrusion_multiplier: s.extrusion_multiplier,
            max_volumetric_speed_mm3_s: s.max_volumetric_speed_mm3_s,
            pressure_advance: s.pressure_advance,
            bridge_flow: s.bridge_flow,
            bridge_speed_mm_s: s.bridge_speed_mm_s,
            fan_speed: s.fan_speed,
            bridge_fan_speed: s.bridge_fan_speed,
            fan_off_layers: s.fan_off_layers,
            aux_fan_speed: s.aux_fan_speed,
            exhaust_fan_speed: s.exhaust_fan_speed,
        }
    }

    fn tool(t: &config::ToolSettings) -> Self {
        Self {
            nozzle_temp_c: t.nozzle_temp_c,
            standby_temp_c: t.standby_temp_c,
            bed_temp_c: t.bed_temp_c,
            chamber_temp_c: t.chamber_temp_c,
            filament_diameter_mm: t.filament_diameter_mm,
            extrusion_multiplier: t.extrusion_multiplier,
            max_volumetric_speed_mm3_s: t.max_volumetric_speed_mm3_s,
            pressure_advance: t.pressure_advance,
            bridge_flow: t.bridge_flow,
            bridge_speed_mm_s: t.bridge_speed_mm_s,
            fan_speed: t.fan_speed,
            bridge_fan_speed: t.bridge_fan_speed,
            fan_off_layers: t.fan_off_layers,
            aux_fan_speed: t.aux_fan_speed,
            exhaust_fan_speed: t.exhaust_fan_speed,
        }
    }
}

/// The App-side state the guided flow-calibration row drives: arming the
/// test print, the measured wall, and the status line it reports to.
struct FlowCalUi<'a> {
    host_ready: bool,
    start: &'a mut bool,
    measured_mm: &'a mut f64,
    status: &'a mut String,
}

/// The Filament packaging-card rows, written once for both surfaces (see
/// [`FilamentFields`]). `show_standby` = toolchanger (docked tools carry a
/// standby setpoint); line width and layer height are read-only context for
/// the flow hints and the calibration math.
fn filament_card_rows(
    ui: &mut egui::Ui,
    f: FilamentFields<'_>,
    base: &FilamentBaseline,
    show_standby: bool,
    line_width_mm: f64,
    layer_height_mm: f64,
    cal: FlowCalUi<'_>,
) {
    revert_row(ui, f.nozzle_temp_c, &base.nozzle_temp_c, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, config::NOZZLE_TEMP_MIN_C..=config::NOZZLE_TEMP_MAX_C), "nozzle °C",
            "Operating nozzle temperature from the spool. The first layer adds the material's adhesion bump on top.");
    });
    if show_standby {
        revert_row(ui, f.standby_temp_c, &base.standby_temp_c, |ui, v| {
            hslider(ui, true, egui::Slider::new(v, 80..=config::NOZZLE_TEMP_MAX_C), "standby °C",
                "Setpoint while this tool sits docked longer than the machine's \
                 standby threshold — hot enough to restart in seconds, cool enough \
                 not to ooze and cook. Auto: operating temperature − 50.");
        });
    }
    revert_row(ui, f.bed_temp_c, &base.bed_temp_c, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0..=120), "bed °C",
            "Bed temperature from the packaging.");
    });
    revert_row(ui, f.chamber_temp_c, &base.chamber_temp_c, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0..=70), "chamber soak °C",
            "Hold during the start g-code — after the bed soak, before the nozzle \
             finishes heating — until the chamber reaches this (the heated bed does \
             the soaking, via TEMPERATURE_WAIT on the chamber sensor). Needs a chamber \
             sensor declared under Machine & motion; Send pings the printer for it \
             first and won't start a soak it can't honor. 0 = off. Auto: the material \
             class's value — ABS/ASA soak at 50 against warping and layer splits; PLA \
             must stay 0 (a hot chamber means heat creep and sag).");
    });
    revert_row(ui, f.filament_diameter_mm, &base.filament_diameter_mm, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 1.0..=3.0), "filament Ø mm",
            "Filament diameter (1.75 or 2.85). Drives the extrusion math.");
    });
    // Measured calibration — the slicer is blind to the true output, so
    // these are pinned from a test, not derived (default 1.0 / conservative;
    // nudge after a flow test or a pressure-advance tower). Density,
    // flow-derate and the heat ceiling are material physics —
    // class-derived, not knobs.
    let flow = f.extrusion_multiplier;
    revert_row(ui, &mut *flow, &base.extrusion_multiplier, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.8..=1.2), "flow ×",
            "Per-spool flow calibration — scales every extrusion. 1.0 = trust the geometry; pin a measured value after a single-wall flow test.");
    });
    // Guided flow calibration: print a single-wall cube at the current
    // settings, caliper a wall, enter it → pin flow × (the active tab's
    // multiplier on a toolchanger).
    ui.horizontal(|ui| {
        if ui
            .add_enabled(cal.host_ready, egui::Button::new("⟲ print flow test"))
            .on_hover_text("Print a single-wall 20 mm cube at your current settings, then caliper a side wall's thickness — the ~line-width dimension, on a flat face mid-height, away from the seam (not the height or the 20 mm width) — and enter it below to pin flow ×. Clear the bed first.")
            .on_disabled_hover_text("Needs a printer host (Connection section) and no other printer operation in flight.")
            .clicked()
        {
            *cal.start = true;
        }
        ui.add(
            egui::DragValue::new(cal.measured_mm)
                .speed(0.01)
                .range(0.0..=2.0)
                .fixed_decimals(2)
                .suffix(" mm"),
        )
        .on_hover_text(format!("Wall *thickness* off the cube — a flat side, mid-height, away from the seam (not the height or the 20 mm width). Should read ≈{line_width_mm:.2} mm (your line width); thicker = over-extruding."));
        if ui
            .button("apply")
            .on_hover_text("Pin flow × = current × line width ÷ measured wall thickness.")
            .clicked()
            && *cal.measured_mm > 0.0
        {
            let before = *flow;
            *flow = engine::flow_from_wall(before, line_width_mm, *cal.measured_mm);
            *cal.status = format!(
                "flow × {before:.3} → {:.3} (wall {:.2} mm vs {:.2} target)",
                *flow, *cal.measured_mm, line_width_mm
            );
        }
    });
    let mf_hint = format!(
        "The filament's measured melt-rate ceiling (mm³/s). The class default is deliberately conservative; a flow-test value belongs here. Right now: {}.",
        flow_ceiling_parts_text(*f.max_volumetric_speed_mm3_s, line_width_mm, layer_height_mm)
    );
    revert_row(ui, f.max_volumetric_speed_mm3_s, &base.max_volumetric_speed_mm3_s, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.0..=80.0), "max flow mm³/s",
            mf_hint);
    });
    revert_row(ui, f.pressure_advance, &base.pressure_advance, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.0..=0.2), "pressure advance",
            "Klipper pressure advance, emitted as SET_PRESSURE_ADVANCE. 0 = leave the printer's value.");
    });
    revert_row(ui, f.bridge_flow, &base.bridge_flow, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.3..=2.0), "bridge flow",
            "Extrusion multiplier for bridge strands and arc overhangs. <1 thins them so they pull taut over air; >1 fattens them to grip when cooling is poor.");
    });
    revert_row(ui, f.bridge_speed_mm_s, &base.bridge_speed_mm_s, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 5.0..=100.0), "bridge speed mm/s",
            "Print speed for bridge strands. Slow lets each strand cool and set before the next is laid.");
    });
}

/// The Cooling section rows — the same one-source treatment (see
/// [`FilamentFields`]); aux/exhaust knobs appear only where the printer
/// declares the hardware.
fn cooling_rows(
    ui: &mut egui::Ui,
    f: FilamentFields<'_>,
    base: &FilamentBaseline,
    has_aux_fan: bool,
    has_exhaust_fan: bool,
) {
    revert_row(ui, f.fan_speed, &base.fan_speed, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.0..=1.0), "fan",
            "Part-cooling fan duty while printing. Auto: the class's policy.");
    });
    revert_row(ui, f.bridge_fan_speed, &base.bridge_fan_speed, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0.0..=1.0), "bridge fan",
            "Fan duty on bridges and arc overhangs.");
    });
    revert_row(ui, f.fan_off_layers, &base.fan_off_layers, |ui, v| {
        hslider(ui, true, egui::Slider::new(v, 0..=5), "fan off layers",
            "Keep the fan off for this many first layers (bed adhesion).");
    });
    // Aux/exhaust duties appear only when the printer profile declares the
    // hardware — no fan, no knob, no M106 emitted.
    if has_aux_fan {
        revert_row(ui, f.aux_fan_speed, &base.aux_fan_speed, |ui, v| {
            hslider(ui, true, egui::Slider::new(v, 0.0..=1.0), "aux fan",
                "Auxiliary part-cooling duty (M106 P2).");
        });
    }
    if has_exhaust_fan {
        revert_row(ui, f.exhaust_fan_speed, &base.exhaust_fan_speed, |ui, v| {
            hslider(ui, true, egui::Slider::new(v, 0.0..=1.0), "exhaust fan",
                "Chamber-exhaust duty (M106 P3), whole print.");
        });
    }
}

/// The blend's weights that name a slot this machine actually has, with
/// zero/negative shares dropped. Empty = the blend references only missing
/// tools (shrunk `tool_count`) and reads as neutral/tool 0. A free function
/// (with an [`App::valid_weights`] wrapper) so the Filament card — which
/// holds `&mut settings` while it renders — can call it too.
fn valid_weights_for(tool_count: usize, blend: &config::BlendState) -> Vec<(u32, f32)> {
    blend
        .weights
        .iter()
        .filter(|&&(t, w)| (t as usize) < tool_count && w > 0.0)
        .copied()
        .collect()
}

/// The blend's dither repeat (mm) at the given layer height, when it
/// overflows the blend band — the loud signal that the layers grew (or the
/// band shrank) under an existing mix. None = fuses fine. Free for the same
/// reason as [`valid_weights_for`].
fn blend_banding_for(
    tool_count: usize,
    layer_height_mm: f64,
    blend_band_mm: f64,
    blend: &config::BlendState,
) -> Option<f64> {
    let valid = valid_weights_for(tool_count, blend);
    let period = config::blend_repeat_layers(&valid) as f64;
    let h = period * layer_height_mm;
    (h > blend_band_mm + 1e-6).then_some(h)
}

/// The tool-tab row atop the Filament card: one spool-colored dot per slot —
/// ring = the active tab, small amber dot = unsaved (*) edits on that slot.
/// Returns the active tab, clamped to the loaded slots.
fn tool_tab_row(
    ui: &mut egui::Ui,
    active: &mut usize,
    tools: &[config::ToolSettings],
    dirty: &[bool],
) -> usize {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 4.0;
        for (i, t) in tools.iter().enumerate() {
            let (rect, resp) =
                ui.allocate_exact_size(egui::vec2(22.0, 20.0), egui::Sense::click());
            let c = rect.center();
            ui.painter().circle_filled(c, 5.5, rgb32(t.color_rgb));
            if *active == i {
                ui.painter().circle_stroke(c, 8.0, egui::Stroke::new(1.5, palette::BLUSH));
            } else if resp.hovered() {
                ui.painter().circle_stroke(c, 8.0, egui::Stroke::new(1.0, palette::HAIRLINE_LOUD));
            }
            let is_dirty = dirty.get(i).copied().unwrap_or(false);
            if is_dirty {
                // The tier rows' * mark at dot scale: unsaved edits here.
                ui.painter().circle_filled(c + egui::vec2(7.5, -7.0), 2.5, palette::WARN);
            }
            let hover = format!(
                "T{i} — {}{}",
                t.filament_name,
                if is_dirty { " (unsaved * edits)" } else { "" }
            );
            if resp.on_hover_text(hover).clicked() {
                *active = i;
            }
        }
    });
    *active = (*active).min(tools.len().saturating_sub(1));
    *active
}

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

/// 2D convex hull (Andrew's monotone chain) of points in mm, returned in CCW
/// order. Fewer than three distinct points passes them through unchanged.
fn convex_hull_2d(mut pts: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    use std::cmp::Ordering::Equal;
    pts.sort_by(|a, b| {
        a[0].partial_cmp(&b[0]).unwrap_or(Equal).then(a[1].partial_cmp(&b[1]).unwrap_or(Equal))
    });
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    // Cross product of OA × OB: > 0 is a CCW (left) turn.
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut lower: Vec<[f64; 2]> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<[f64; 2]> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    // Drop each chain's last point (it's the other chain's first); concatenate.
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
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

/// What paints a part: a plain tool slot, or a pseudo color from the blend
/// palette (an index into `App::blends`), realized at slice time by layer
/// dithering. Blend indices can dangle after a delete — every consumer reads
/// them through `App::paint_display_rgb` / `App::paint_engine`, which fall
/// back to neutral / tool 0.
#[derive(Clone, Copy, PartialEq)]
enum PartColor {
    Tool(u32),
    Blend(usize),
}

/// One leaf mesh of a scene object: on a toolchanger each part keeps its own
/// paint (tool slot or blend); single-tool machines print everything on
/// tool 0. Parts share the object's one placement — they move as a body.
struct ScenePart {
    name: String,
    mesh: Arc<mesh::Mesh>,
    /// The mesh the viewer draws: `mesh` minus ghost faces (generator fins,
    /// buried walls) — computed once at import, NEVER sliced (see
    /// `Mesh::display_mesh`).
    display: Arc<mesh::Mesh>,
    /// The tool or blend that prints this part. Clamped to the machine's
    /// tool count at bake time.
    paint: PartColor,
    /// Surface paint: a tool per `display` triangle (`None` = unpainted → the
    /// part's base `paint` shows). Empty until first brushed. Indexed by
    /// `display` triangle (what's shown + picked); the smart-fill brush writes
    /// it. (Stage A: preview/selection only — not yet fed to the slicer.)
    paint_tri: Vec<Option<u32>>,
}

/// One object placed on the bed: its parts (shared mesh geometry) plus an
/// editable placement (Euler rotation, uniform scale, and a bed-plane position
/// for the footprint center). The object always rests on z=0 — its baked
/// transform drops it there.
struct SceneObject {
    name: String,
    parts: Vec<ScenePart>,
    /// Euler rotation in degrees, applied X then Y then Z.
    rot_deg: [f64; 3],
    scale: f64,
    /// Bed XY of the rotated/scaled footprint's center.
    pos: [f64; 2],
    /// Memoized rotated/scaled bounds — `footprint()`/`height()`/`transform()`
    /// all read it. It depends ONLY on `rot_deg`+`scale`, so a move (pos) or a
    /// recolor never invalidates it; the full per-vertex walk these once did on
    /// every rebuild (drag/select/color) now runs only when rotation or scale
    /// actually changes. Interior-mutable so the `&self` accessors can fill it
    /// lazily (GUI is single-threaded).
    bounds_cache: std::cell::Cell<Option<BoundsCache>>,
}

/// Signature of the inputs that determine slice GEOMETRY (not paint): the
/// active-bed parts' source meshes + their baked placements, and the full
/// settings. Two signatures match iff a re-slice would produce byte-identical
/// geometry, so the only thing that could have changed is the parts' paint —
/// making [`engine::restamp_paint`] valid. Meshes are compared by `Arc` pointer
/// (and held alive here, so an address can't be reused for a different mesh).
struct GeomSig {
    parts: Vec<(std::sync::Arc<mesh::Mesh>, mesh::Transform)>,
    settings: config::Settings,
}

impl GeomSig {
    fn matches(&self, other: &GeomSig) -> bool {
        self.settings == other.settings
            && self.parts.len() == other.parts.len()
            && self
                .parts
                .iter()
                .zip(&other.parts)
                .all(|(a, b)| std::sync::Arc::ptr_eq(&a.0, &b.0) && a.1 == b.1)
    }
}

/// Cached rotated/scaled extents: `(minx, miny, maxx, maxy, minz, maxz)` under
/// `(rot_deg, scale)`.
#[derive(Clone, Copy)]
struct BoundsCache {
    rot_deg: [f64; 3],
    scale: f64,
    b: (f64, f64, f64, f64, f64, f64),
}

impl SceneObject {
    fn new(name: String, mesh: mesh::Mesh) -> Self {
        Self {
            name,
            parts: vec![ScenePart {
                name: String::new(),
                display: Arc::new(mesh.display_mesh()),
                mesh: Arc::new(mesh),
                paint: PartColor::Tool(0),
                paint_tri: Vec::new(),
            }],
            rot_deg: [0.0; 3],
            scale: 1.0,
            pos: [0.0, 0.0],
            bounds_cache: std::cell::Cell::new(None),
        }
    }

    /// Rotated/scaled extents `(minx,miny,maxx,maxy,minz,maxz)`, memoized on
    /// `(rot_deg, scale)`. The one place that walks the parts' vertices.
    fn bounds6(&self) -> (f64, f64, f64, f64, f64, f64) {
        if let Some(c) = self.bounds_cache.get() {
            if c.rot_deg == self.rot_deg && c.scale == self.scale {
                return c.b;
            }
        }
        let lin = mesh::Transform { rotation: euler_matrix(self.rot_deg), scale: self.scale, ..Default::default() };
        let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN, f64::MAX, f64::MIN);
        for &v in self.parts.iter().flat_map(|p| &p.mesh.vertices) {
            let p = lin.apply_linear(v);
            b.0 = b.0.min(p[0]);
            b.1 = b.1.min(p[1]);
            b.2 = b.2.max(p[0]);
            b.3 = b.3.max(p[1]);
            b.4 = b.4.min(p[2]);
            b.5 = b.5.max(p[2]);
        }
        self.bounds_cache.set(Some(BoundsCache { rot_deg: self.rot_deg, scale: self.scale, b }));
        b
    }

    /// Footprint of the rotated+scaled parts (no placement): (minx,miny,maxx,maxy,minz).
    fn footprint(&self) -> (f64, f64, f64, f64, f64) {
        let b = self.bounds6();
        (b.0, b.1, b.2, b.3, b.4)
    }

    /// Printed height (mm): the z-span of the rotated/scaled parts. The object
    /// rests on z=0 (its transform drops it there), so it occupies [0, h].
    fn height(&self) -> f64 {
        let b = self.bounds6();
        (b.5 - b.4).max(0.0)
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

/// An in-flight camera glide: when bed focus changes, the orbit target
/// travels to the new bed instead of teleporting — the motion is what tells
/// you where you went. Distance and orientation are left exactly as the user
/// had them (a pure translation) unless the current zoom is so far off that
/// the target bed wouldn't be legible, in which case `dist_to` eases it back
/// just into the comfortable band (see `FLY_ZOOM_MIN`/`FLY_ZOOM_MAX`).
struct CameraGlide {
    from: glam::Vec3,
    to: glam::Vec3,
    dist_from: f32,
    dist_to: f32,
    started: std::time::Instant,
}

/// Glide duration (ease-out cubic — brisk start, soft landing).
const GLIDE_SECS: f32 = 0.4;

/// Bed-focus flies keep the current zoom (and always the orientation) as long
/// as the distance frames the target bed reasonably — within these multiples
/// of the ideal one-bed framing distance. Outside the band (zoomed into a
/// feature, or pulled back to see every bed) the fly eases distance only to
/// the nearest edge: the least change that restores a usable view.
const FLY_ZOOM_MIN: f32 = 0.5;
const FLY_ZOOM_MAX: f32 = 2.5;

/// Gap between adjacent print beds in the world layout (mm). Beds line up
/// along +X; an object's world position decides which bed it belongs to, so
/// dragging a part across the gap moves it between beds.
const BED_GAP_MM: f64 = 25.0;

/// Smart-fill connectivity gate: the flood won't cross an edge whose smoothed
/// face normals differ by more than this (a crease). The GLOBAL drift budget
/// (`brush_drift_deg`) is the live anti-bleed knob on top of it.
const PAINT_THETA_LOCAL_DEG: f64 = 35.0;

/// Cell size (mm) of the spatial index over sliced beads for the bead brush.
const BEAD_GRID_CELL_MM: f32 = 2.0;

/// In paint mode the preview beads are subdivided to this max length so the
/// bead brush paints only the piece under it, not a whole (possibly long) path
/// segment. Off outside paint mode (avoids bloating the normal preview).
const PAINT_BEAD_MAX_MM: f32 = 0.5;

/// World X origin of bed `k`.
fn bed_origin_x(k: usize, bed_x: f64) -> f64 {
    k as f64 * (bed_x + BED_GAP_MM)
}

/// Which bed a world X belongs to: the nearest bed center, never negative.
fn bed_of_pos(x: f64, bed_x: f64) -> usize {
    ((x - bed_x / 2.0) / (bed_x + BED_GAP_MM)).round().max(0.0) as usize
}

/// Do two world XY axis-aligned boxes `[minx, miny, maxx, maxy]` overlap by
/// more than a hair? (A small epsilon so exactly-touching footprints — what
/// the auto-arrange leaves — don't read as collisions.)
fn aabb_overlap(a: [f64; 4], b: [f64; 4]) -> bool {
    const EPS: f64 = 0.05;
    a[0] < b[2] - EPS && a[2] > b[0] + EPS && a[1] < b[3] - EPS && a[3] > b[1] + EPS
}

/// Shelf-pack footprint rectangles onto beds: tallest-first fills shelves
/// left-to-right, `margin` apart, and each bed's finished layout is centered
/// on its plate. Each part takes its own footprint — no uniform worst-case
/// cells — so mixed sizes pack tight. Overflow flows onto the next bed; a
/// part bigger than the plate gets a bed to itself (centered, hanging over).
/// Returns `(bed, center_x, center_y)` per input rect, bed-local, input
/// order preserved.
fn shelf_pack(sizes: &[(f64, f64)], bx: f64, by: f64, margin: f64) -> Vec<(usize, f64, f64)> {
    let mut order: Vec<usize> = (0..sizes.len()).collect();
    order.sort_by(|&a, &b| sizes[b].1.total_cmp(&sizes[a].1)); // tallest first
    let mut out = vec![(0usize, 0.0, 0.0); sizes.len()];

    let mut bed = 0usize;
    // Corner-anchored placements on the current bed: (input idx, x, y).
    let mut placed: Vec<(usize, f64, f64)> = Vec::new();
    let (mut cur_x, mut cur_y, mut shelf_h) = (0.0f64, 0.0f64, 0.0f64);
    let (mut used_w, mut used_h) = (0.0f64, 0.0f64);

    // Emit the current bed's placements, centered on the plate (a negative
    // offset = centered overflow for oversized parts).
    macro_rules! flush_bed {
        () => {
            let (ox, oy) = ((bx - used_w) / 2.0, (by - used_h) / 2.0);
            for &(i, x, y) in &placed {
                let (w, h) = sizes[i];
                out[i] = (bed, ox + x + w / 2.0, oy + y + h / 2.0);
            }
            placed.clear();
        };
    }

    for &i in &order {
        let (w, h) = sizes[i];
        if cur_x > 0.0 && cur_x + w > bx {
            // Next shelf.
            cur_y += shelf_h + margin;
            cur_x = 0.0;
            shelf_h = 0.0;
        }
        if cur_x == 0.0 && cur_y > 0.0 && cur_y + h > by {
            // Bed full — flush and start the next one.
            flush_bed!();
            bed += 1;
            cur_y = 0.0;
            used_w = 0.0;
            used_h = 0.0;
        }
        placed.push((i, cur_x, cur_y));
        used_w = used_w.max(cur_x + w);
        used_h = used_h.max(cur_y + h);
        cur_x += w + margin;
        shelf_h = shelf_h.max(h);
    }
    flush_bed!();
    out
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

/// A `mesh::Transform` (`p ↦ R·(s·p) + T`, row-major R) as a COLUMN-MAJOR 4×4
/// for the GPU — same convention as `Uniforms::mvp`. The shader does
/// `mvp * model * vec4(p,1)`, reproducing the old CPU `Transform::apply` bake.
fn transform_to_mat4(t: &mesh::Transform) -> [[f32; 4]; 4] {
    let r = &t.rotation;
    let s = t.scale as f32;
    let (r00, r01, r02) = (r[0][0] as f32, r[0][1] as f32, r[0][2] as f32);
    let (r10, r11, r12) = (r[1][0] as f32, r[1][1] as f32, r[1][2] as f32);
    let (r20, r21, r22) = (r[2][0] as f32, r[2][1] as f32, r[2][2] as f32);
    let tr = &t.translation;
    // Columns: the linear part is R·s (column j = (R[0][j], R[1][j], R[2][j])·s).
    [
        [r00 * s, r10 * s, r20 * s, 0.0],
        [r01 * s, r11 * s, r21 * s, 0.0],
        [r02 * s, r12 * s, r22 * s, 0.0],
        [tr[0] as f32, tr[1] as f32, tr[2] as f32, 1.0],
    ]
}

fn main() -> eframe::Result<()> {
    // Headless one-shot render (no window): `--render-layer N [--walls W]
    // [--out p.png] [--size WxH] model.stl|model.3mf` (a 3MF slices per part
    // on the tools its extruder hints name — see offscreen.rs env knobs).
    let argv: Vec<String> = std::env::args().collect();
    if argv.iter().any(|a| a == "--render-layer") {
        if let Err(e) = run_offscreen(&argv) {
            eprintln!("offscreen render failed: {e}");
            std::process::exit(1);
        }
        return Ok(());
    }

    // The window/taskbar icon: a Playfair "F" sliced into offset layers on a
    // cream tile (generated from the wordmark font; raw RGBA so no image
    // decoder is needed). Wayland ignores per-window icons by design — there
    // it comes from a .desktop file instead, when we ship one.
    let icon = egui::IconData {
        rgba: include_bytes!("../assets/icon.rgba").to_vec(),
        width: 128,
        height: 128,
    };
    // Enable the adapter's full format features on the device where the adapter
    // has them, so MSAA can exceed the WebGPU-baseline 4× (8× on this GPU).
    // Without it the device rejects an 8× target and panics. Wrap eframe's
    // default device descriptor so its limits and other features are preserved.
    let mut wgpu_options = eframe::egui_wgpu::WgpuConfiguration::default();
    if let eframe::egui_wgpu::WgpuSetup::CreateNew(create) = &mut wgpu_options.wgpu_setup {
        let base = std::sync::Arc::clone(&create.device_descriptor);
        create.device_descriptor = std::sync::Arc::new(move |adapter: &eframe::wgpu::Adapter| {
            let mut dd = base(adapter);
            let f = eframe::wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES;
            if adapter.features().contains(f) {
                dd.required_features |= f;
            }
            dd
        });
    }
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        wgpu_options,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1600.0, 1000.0])
            .with_min_inner_size([1024.0, 640.0])
            .with_icon(icon),
        ..Default::default()
    };
    // An optional model path on the command line opens on startup (shell use,
    // file associations, and scripted GUI checks).
    let open_path = argv
        .iter()
        .skip(1)
        .find(|a| !a.starts_with('-'))
        .map(std::path::PathBuf::from);
    eframe::run_native(
        "Fable Slicer",
        options,
        Box::new(move |cc| {
            let mut app = App::new(cc);
            if let Some(p) = open_path {
                app.import_model(p);
            }
            Ok(Box::new(app))
        }),
    )
}

/// Parse the headless-render CLI and run it (see the `--render-layer` branch).
fn run_offscreen(argv: &[String]) -> Result<(), String> {
    let mut a = offscreen::Args {
        model: std::path::PathBuf::new(),
        out: std::path::PathBuf::from("/tmp/offscreen.png"),
        layer: 1,
        walls: 99,
        width: 1400,
        height: 1000,
        zoom: 1.15,
        pitch: -0.45,
        tx: 0.0,
        ty: 0.0,
    };
    let next = |i: usize| -> Result<&String, String> {
        argv.get(i + 1).ok_or_else(|| format!("missing value after {}", argv[i]))
    };
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--render-layer" => {
                a.layer = next(i)?.parse().map_err(|_| "bad --render-layer")?;
                i += 2;
            }
            "--walls" => {
                a.walls = next(i)?.parse().map_err(|_| "bad --walls")?;
                i += 2;
            }
            "--out" => {
                a.out = std::path::PathBuf::from(next(i)?);
                i += 2;
            }
            "--size" => {
                let (w, h) = next(i)?.split_once('x').ok_or("--size wants WxH")?;
                a.width = w.parse().map_err(|_| "bad --size width")?;
                a.height = h.parse().map_err(|_| "bad --size height")?;
                i += 2;
            }
            "--zoom" => {
                a.zoom = next(i)?.parse().map_err(|_| "bad --zoom")?;
                i += 2;
            }
            "--pitch" => {
                a.pitch = next(i)?.parse().map_err(|_| "bad --pitch")?;
                i += 2;
            }
            "--target" => {
                let (x, y) = next(i)?.split_once(',').ok_or("--target wants X,Y")?;
                a.tx = x.parse().map_err(|_| "bad --target x")?;
                a.ty = y.parse().map_err(|_| "bad --target y")?;
                i += 2;
            }
            s if !s.starts_with("--") => {
                a.model = std::path::PathBuf::from(s);
                i += 1;
            }
            _ => i += 1,
        }
    }
    if a.model.as_os_str().is_empty() {
        return Err("no model path given".into());
    }
    offscreen::run(&a)
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
    /// Per used tool: (tool, length_mm, grams). Rendered only on toolchangers.
    per_tool: Vec<(u32, f64, f64)>,
    /// Tool switches across consecutive printable paths over the whole print
    /// (layer boundaries included) — exactly where the g-code swaps tools.
    toolchanges: usize,
}

/// One in-flight background slice. The heavy geometry pass (`plan_geometry` —
/// seconds of wall offsets on a 99-wall model) runs off the UI thread so the
/// app never freezes; `ui()` polls `rx` each frame and commits the result. Only
/// paint-only re-stamps (fast) stay on the UI thread; a fresh geometry slice
/// takes this path.
struct SliceJob {
    rx: std::sync::mpsc::Receiver<SliceOutput>,
    /// The geometry signature the parts had at spawn — cached with the result
    /// so a later paint-only change can re-stamp instead of re-slicing.
    sig: GeomSig,
    /// A re-slice keeps the viewed layer; a first slice jumps to the top.
    resliced: bool,
    /// Bed world offset baked into the preview instances at slice time.
    origin_x: f64,
    /// Live wall-pass progress, ticked by the worker; the Slice button reads
    /// `fraction()` each frame to fill as a progress bar.
    progress: std::sync::Arc<engine::SliceProgress>,
}

/// The product of a slice — computed on the UI thread (paint-only re-stamp) or
/// a worker (full re-slice), ready to commit and upload to the GPU.
struct SliceOutput {
    layers: Vec<engine::LayerPlan>,
    /// `Some` when the geometry was freshly planned (cache it); `None` on a
    /// paint-only re-stamp of already-cached geometry.
    geo: Option<engine::GeometryPlan>,
    summary: SliceSummary,
    layer_stats: Vec<engine::LayerStats>,
    verts: Vec<[f32; 14]>,
    ends: Vec<u32>,
    joints: Vec<[f32; 11]>,
    joint_ends: Vec<u32>,
}

/// A freehand paint stroke on the SLICED beads: a shallow disc on the surface at
/// A freehand paint stroke on the sliced beads. `tris` is the CONNECTED
/// front-surface patch flooded from where the brush clicked (world space); a bead
/// is painted only if it hugs the outer shell of that patch (`engine::dab_covers`)
/// — surface connectivity that keeps the brush off disconnected/back-side/interior
/// beads. `center` + `radius` bound it for cheap culling. Assigns a tool (`None`
/// clears → the bead reverts to base). World space so it re-applies across
/// re-slices; composes in order (later wins).
#[derive(Clone)]
struct BeadDab {
    center: glam::Vec3,
    radius: f32,
    tool: Option<u32>,
    tris: Vec<[[f64; 3]; 3]>,
}

/// Uniform spatial hash over bead midpoints so "which beads are in this sphere"
/// is O(beads near the brush), not O(all beads) — the difference between a
/// smooth stroke and a stutter on an 800k-bead slice. Rebuilt whenever the
/// instance set changes.
#[derive(Default)]
struct BeadGrid {
    cell: f32,
    map: std::collections::HashMap<(i32, i32, i32), Vec<u32>>,
}

/// A bead instance's midpoint (start + ½·dir·len) in world/render coords.
fn bead_mid(b: &[f32; 14]) -> glam::Vec3 {
    glam::Vec3::new(b[0] + b[3] * b[5] * 0.5, b[1] + b[4] * b[5] * 0.5, b[2])
}

impl BeadGrid {
    fn build(inst: &[[f32; 14]], cell: f32) -> Self {
        let inv = 1.0 / cell.max(1e-3);
        let mut map: std::collections::HashMap<(i32, i32, i32), Vec<u32>> =
            std::collections::HashMap::new();
        for (i, b) in inst.iter().enumerate() {
            // Only extrusion carries a printing tool — skip travels/seams.
            if b[12] >= CAT_TRAVEL {
                continue;
            }
            let m = bead_mid(b);
            let key = ((m.x * inv).floor() as i32, (m.y * inv).floor() as i32, (m.z * inv).floor() as i32);
            map.entry(key).or_default().push(i as u32);
        }
        Self { cell, map }
    }

    /// Indices of beads whose midpoint is within `radius` of `center`.
    fn query(&self, inst: &[[f32; 14]], center: glam::Vec3, radius: f32) -> Vec<u32> {
        let inv = 1.0 / self.cell.max(1e-3);
        let r2 = radius * radius;
        let lo = (center - glam::Vec3::splat(radius)) * inv;
        let hi = (center + glam::Vec3::splat(radius)) * inv;
        let mut out = Vec::new();
        for gx in lo.x.floor() as i32..=hi.x.floor() as i32 {
            for gy in lo.y.floor() as i32..=hi.y.floor() as i32 {
                for gz in lo.z.floor() as i32..=hi.z.floor() as i32 {
                    if let Some(v) = self.map.get(&(gx, gy, gz)) {
                        for &i in v {
                            if (bead_mid(&inst[i as usize]) - center).length_squared() <= r2 {
                                out.push(i);
                            }
                        }
                    }
                }
            }
        }
        out
    }
}

/// Cached world bounds of one scene object, refreshed by `rebuild_scene` so
/// the bounds/collision checks (run for tinting, the transform card, and the
/// Send gate) don't re-walk mesh vertices every frame.
#[derive(Clone, Copy)]
struct ObjBounds {
    /// World XY axis-aligned bounds: `[minx, miny, maxx, maxy]`.
    aabb: [f64; 4],
    /// Printed height (mm); the object rests on z=0, occupying `[0, height]`.
    height: f64,
    /// Bed index (by world X), so collisions only pair same-bed objects.
    bed: usize,
}

struct App {
    profiles: Profiles,
    printer: String,
    filament: String,
    /// Filament profile loaded in each tool slot (toolchangers). Slot 0 IS
    /// `filament` — the tier row and the Filament card's slot row edit the same selection;
    /// length always tracks `settings.tool_count` (see `sync_tool_slots`).
    tools: Vec<String>,
    /// Loaded-spool color per slot: an override of the slot filament's
    /// profile color, remembered with the filament it was picked for — a slot
    /// whose filament changes drops it (new spool, new color story).
    tool_colors: Vec<Option<(String, [f32; 3])>>,
    /// Per-slot hex-entry buffers ("#RRGGBB" typed off a spool label);
    /// refreshed from the effective color whenever the field isn't focused.
    tool_hex: Vec<String>,
    /// The blend palette: pseudo colors dithered from the tool slots,
    /// assignable to parts alongside the plain tools.
    blends: Vec<config::BlendState>,
    process: String,
    /// Every OTHER machine's loadout (spools, colors, blends, process),
    /// keyed by printer profile name — the active printer's lives in the
    /// fields above and is stashed here on switch (see
    /// `switch_printer_loadout`). Machines don't share spools.
    loadouts: std::collections::BTreeMap<String, config::Loadout>,
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
    /// Flow calibration: the Filament-panel button arms this; the dispatch
    /// generates the single-wall test cube and sends it to the printer.
    start_flow_cal: bool,
    /// Measured wall thickness (mm) entered after the flow-cal print; "apply"
    /// turns it into the filament's `extrusion_multiplier`.
    flow_cal_mm: f64,
    /// A profile switch requested while settings carry unsaved (*) edits —
    /// held here until the user confirms discarding them.
    pending_switch: Option<(String, String, String)>,
    /// A tool slot's filament switch requested while THAT slot carries
    /// unsaved (*) edits — same confirm pattern, scoped to one slot.
    pending_slot: Option<(usize, String)>,
    /// The slot the Filament/Cooling cards edit on a toolchanger (the tab
    /// row on the Filament card); clamped whenever the tool count changes.
    active_tool_tab: usize,
    /// Auto/pinned state of the derivable settings.
    pins: Pins,
    objects: Vec<SceneObject>,
    selected: Option<usize>,
    /// The active bed: slicing operates on its objects, the camera pivots on
    /// it, and the viewport draws it highlighted.
    active_bed: usize,
    /// How many beds exist — explicit state, not derived from occupancy.
    /// Grows when objects land beyond the current set (import/duplicate/drag)
    /// or via the bed card's `+`; shrinks only when the user removes a bed
    /// with `−`. Empty beds persist across navigation.
    bed_count: usize,
    /// Per-object cached world bounds (parallel to `objects`), rebuilt by
    /// `rebuild_scene`. Drives the build-volume and collision checks.
    obj_bounds: Vec<ObjBounds>,
    /// World X origin of the bed that was active when `sliced` was produced —
    /// the preview renders there even if the active bed changes afterward.
    sliced_origin_x: f64,
    /// In-flight camera travel to a newly focused bed (None = settled).
    camera_glide: Option<CameraGlide>,
    scene: Scene,
    camera: Camera,
    status: String,
    sliced: Option<Vec<engine::LayerPlan>>,
    /// Paint-independent slice geometry from the last full slice, plus the
    /// signature of the inputs that made it. A re-slice whose signature still
    /// matches (only the parts' paint changed — a tool reassignment or a blend
    /// edit) re-stamps this instead of re-slicing the geometry.
    geom_cache: Option<(GeomSig, engine::GeometryPlan)>,
    /// An in-flight background slice, if any. Set when a geometry change kicks
    /// off `plan_geometry` on a worker thread; polled + committed in `ui()`.
    /// The Slice button disables while this is `Some`.
    slice_job: Option<SliceJob>,
    /// Readable result block for the last slice; cleared with `sliced`.
    slice_summary: Option<SliceSummary>,
    /// Per-layer time/heat numbers behind the preview color modes.
    layer_stats: Vec<engine::LayerStats>,
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
    show_surface: bool,
    show_infill: bool,
    show_skirt: bool,
    show_support: bool,
    show_travel: bool,
    show_seams: bool,
    show_ironing: bool,
    /// Night-sky star field on the viewport backdrop (Model + Preview).
    show_stars: bool,
    /// Per-resource scene-invalidation flags. Split out of a single
    /// `needs_rebuild` bool so a cheap-intent action only redoes the resource it
    /// actually touched: a selection or bed-highlight refreshes the spotlight
    /// (and beds) without re-walking + re-uploading the whole mesh vertex
    /// buffer, and a color change stays off the beds/spotlight/bounds work.
    /// The mesh is geometry (local, uploaded once) + a per-part uniform (model
    /// matrix + tint). `mesh_struct_dirty` = the part SET changed (import/delete)
    /// → re-upload geometry; `mesh_xform_dirty` = an object moved/rotated/scaled
    /// → rewrite only the model matrices (+refresh bounds); `mesh_color_dirty` =
    /// only a tint changed → rewrite only the colors. A drag or recolor is now a
    /// ~96-byte-per-part uniform write, never a vertex re-walk.
    beds_dirty: bool,
    spotlight_dirty: bool,
    mesh_struct_dirty: bool,
    mesh_xform_dirty: bool,
    mesh_color_dirty: bool,
    /// The per-vertex paint tint buffer needs re-upload (a brush stroke changed
    /// `paint_tri`). Distinct from `mesh_color_dirty` (per-part base tint).
    mesh_paint_dirty: bool,
    // --- surface paint brush (Stage A) ---
    /// Paint mode: viewport clicks flood-fill surface regions instead of
    /// selecting/orbiting.
    paint_mode: bool,
    /// Tool the brush lays down; `brush_erase` clears instead.
    brush_tool: u32,
    brush_erase: bool,
    /// Live smart-fill tolerances: global normal drift from the seed (degrees)
    /// and geodesic reach (mm).
    brush_drift_deg: f32,
    brush_radius_mm: f32,
    /// Cached flood topology for the part under the brush (`(obj, part, topo)`),
    /// rebuilt when the target part changes.
    paint_topo: Option<(usize, usize, paint::PaintTopology)>,
    /// A freehand brush stroke is underway (drag started on the model in paint
    /// mode) — drag events paint instead of orbiting.
    painting: bool,
    /// Freehand brush radius (world mm) for the drag-stroke add/erase that
    /// corrects what the flood over/under-grabbed.
    brush_dab_mm: f32,
    // --- bead painting (paint the sliced preview directly, bead resolution) ---
    /// Paint strokes on the sliced beads, in world space — resolution-
    /// independent, re-applied after every (re)slice and mode change.
    bead_dabs: Vec<BeadDab>,
    /// Working copy of the bead instances (== what's on the GPU); dabs recolor it.
    bead_inst: Vec<[f32; 14]>,
    /// The pristine (mesh-paint) instances, so an erase reverts a bead and the
    /// dabs can be re-applied onto a fresh slice.
    bead_pristine: Vec<[f32; 14]>,
    /// Spatial index over `bead_inst` midpoints for brush queries.
    bead_grid: BeadGrid,
    /// Bumped whenever the scene's GPU buffers change (mesh, beads, beds), so the
    /// render-skip below can tell a content change from a static frame.
    content_version: u64,
    /// The scene inputs at the last actual render; the scene re-renders only when
    /// these change (see `RenderSig`).
    last_render_sig: Option<RenderSig>,
    /// The "Front" bed label laid out once: flat triangle verts (local galley-px
    /// position, normalized atlas uv) plus the galley size (tw, th). Mapped to the
    /// bed plane and uploaded as depth-tested scene geometry.
    front_label: Option<(Vec<([f32; 2], [f32; 2])>, f32, f32)>,
    /// Bed key (x, y, active) the label geometry was last built for.
    last_label_bed: Option<(f64, f64, usize)>,
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
    /// Screen rect of the floating bed card (same input-blocking purpose).
    bed_overlay_rect: Option<egui::Rect>,
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
        let picked_printer = pick(&state.printer, profiles.printer_names(), "voron24");
        // The machine's own loadout: spools, colors, blends, and the process
        // last used on it. The flat fields only mirror the machine active at
        // save time, which a deleted-profile fallback may no longer match.
        let loadout = state.loadouts.get(&picked_printer).cloned().unwrap_or_else(|| {
            config::Loadout {
                tools: state.tool_filaments(),
                tool_colors: state.tool_colors.clone(),
                blends: state.blends.clone(),
                process: state.process.clone(),
            }
        });
        let (mut printer, mut filament, mut process) = (
            picked_printer,
            pick(
                loadout.tools.first().map_or(state.filament.as_str(), String::as_str),
                profiles.filament_names(),
                "pla",
            ),
            pick(&loadout.process, profiles.process_names(), "standard"),
        );
        // Tool slots restore per-slot (a vanished profile falls back alone).
        let mut tools: Vec<String> = loadout
            .tools
            .iter()
            .map(|t| pick(t, profiles.filament_names(), "pla"))
            .collect();
        let mut settings = match profiles.resolve(&printer, &filament, &process) {
            Ok(s) => s,
            Err(_) => {
                (printer, filament, process) =
                    ("voron24".to_string(), "pla".to_string(), "standard".to_string());
                profiles.resolve(&printer, &filament, &process).unwrap_or_default()
            }
        };
        // The slot list tracks the machine's tool count (slot 0 IS the
        // filament tier); a toolchanger re-resolves through every slot so
        // cross-tool facts (bed temp, flow ceiling, per-slot colors) are real.
        tools.truncate(settings.tool_count.max(1));
        while tools.len() < settings.tool_count.max(1) {
            tools.push(filament.clone());
        }
        tools[0] = filament.clone();
        if settings.tool_count > 1 {
            let names: Vec<&str> = tools.iter().map(String::as_str).collect();
            if let Ok(s) = profiles.resolve_tools(&printer, &names, &process) {
                settings = s;
            }
        }
        settings.auto_center_on_bed = false; // objects are placed explicitly on the bed
        let baseline = settings.clone();
        let pins = match (
            profiles.merged_process(&process),
            profiles.merged_printer(&printer),
            profiles.merged_filament(&filament),
        ) {
            (Ok(pc), Ok(pr), Ok(_)) => Pins {
                line_width: pc.line_width_mm.is_some(),
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
            },
            _ => Pins::default(),
        };
        let last_bed = (settings.bed_size_x_mm, settings.bed_size_y_mm);
        // Loaded-spool colors ride with the slot list they were saved beside;
        // apply_tool_colors below lays them over the resolved tool table.
        let tool_colors: Vec<Option<(String, [f32; 3])>> = tools
            .iter()
            .enumerate()
            .map(|(i, name)| {
                loadout
                    .tool_colors
                    .get(i)
                    .and_then(|h| config::parse_hex_color(h))
                    .map(|c| (name.clone(), c))
            })
            .collect();
        let multi_tool = settings.tool_count > 1;
        let mut app = Self {
            profiles,
            printer,
            filament,
            tools,
            tool_colors,
            tool_hex: Vec::new(),
            blends: loadout.blends.clone(),
            process,
            loadouts: state.loadouts.clone(),
            // Seed with the file's content as-loaded: if the fallbacks above
            // corrected anything, the first persist pass rewrites the file.
            last_model_dir: state.last_model_dir.clone(),
            last_export_dir: state.last_export_dir.clone(),
            saved_state: state,
            accent,
            accent_rebake: false,
            active_bed: 0,
            bed_count: 1,
            obj_bounds: Vec::new(),
            sliced_origin_x: 0.0,
            camera_glide: None,
            settings,
            baseline,
            profile_dialog: None,
            start_flow_cal: false,
            flow_cal_mm: 0.0,
            pending_switch: None,
            pending_slot: None,
            active_tool_tab: 0,
            pins,
            objects: Vec::new(),
            selected: None,
            scene,
            camera: Camera::new(),
            status,
            sliced: None,
            geom_cache: None,
            slice_job: None,
            slice_summary: None,
            layer_stats: Vec::new(),
            // Filament colors are the point of a toolchanger preview; the
            // mode doesn't exist on single-tool machines (see
            // `sync_tool_slots`), where features carry the information.
            color_by: if multi_tool { ColorBy::Filament } else { ColorBy::Feature },
            host_rx: None,
            sent_to_printer: false,
            printer_status: None,
            last_status_poll: None,
            print_overlay_rect: None,
            bed_overlay_rect: None,
            layer_ends: Vec::new(),
            joint_layer_ends: Vec::new(),
            view_preview: false,
            preview_layer: 1,
            show_walls: true,
            show_solid: true,
            show_surface: true,
            show_infill: true,
            show_skirt: true,
            show_support: true,
            show_travel: false,
            show_seams: false,
            show_stars: true,
            show_ironing: true,
            beds_dirty: true,
            spotlight_dirty: true,
            mesh_struct_dirty: true,
            mesh_xform_dirty: false,
            mesh_color_dirty: false,
            mesh_paint_dirty: false,
            paint_mode: false,
            brush_tool: 1,
            brush_erase: false,
            brush_drift_deg: 60.0,
            brush_radius_mm: 25.0,
            paint_topo: None,
            painting: false,
            brush_dab_mm: 6.0,
            bead_dabs: Vec::new(),
            bead_inst: Vec::new(),
            bead_pristine: Vec::new(),
            bead_grid: BeadGrid::default(),
            content_version: 0,
            last_render_sig: None,
            front_label: None,
            last_label_bed: None,
            refit_camera: true,
            recenter_camera: false,
            last_bed,
            drag_obj: None,
            drag_grab: [0.0, 0.0],
            overlay_rect: None,
            msgs_overlay_rect: None,
            msgs_dismissed: None,
            slice_gen: 0,
        };
        app.apply_tool_colors();
        app
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
            for part in &obj.parts {
                for k in 0..part.mesh.triangles.len() {
                    let tri = part.mesh.triangle(k);
                    if let Some(dist) = ray_triangle(o, d, v(tri[0]), v(tri[1]), v(tri[2])) {
                        if best.map_or(true, |(bd, _)| dist < bd) {
                            best = Some((dist, i));
                        }
                    }
                }
            }
        }
        best.map(|(_, i)| i)
    }

    /// Nearest FRONT-facing `display` triangle the ray hits: `(obj, part, tri,
    /// world hit)`. Front-face only so a click paints the visible surface, never
    /// punches through to a hidden back wall. Rays/triangles are in world space.
    fn pick_paint_face(
        &self,
        o: glam::Vec3,
        d: glam::Vec3,
    ) -> Option<(usize, usize, usize, glam::Vec3)> {
        let mut best: Option<(f32, usize, usize, usize)> = None;
        for (i, obj) in self.objects.iter().enumerate() {
            let t = obj.transform();
            let v = |p: [f64; 3]| {
                let q = t.apply(p);
                glam::Vec3::new(q[0] as f32, q[1] as f32, q[2] as f32)
            };
            for (pi, part) in obj.parts.iter().enumerate() {
                for k in 0..part.display.triangles.len() {
                    let tri = part.display.triangle(k);
                    let (a, b, c) = (v(tri[0]), v(tri[1]), v(tri[2]));
                    // Front-face: geometric normal points toward the ray origin.
                    if (b - a).cross(c - a).dot(o - a) <= 0.0 {
                        continue;
                    }
                    if let Some(dist) = ray_triangle(o, d, a, b, c) {
                        if best.map_or(true, |(bd, ..)| dist < bd) {
                            best = Some((dist, i, pi, k));
                        }
                    }
                }
            }
        }
        best.map(|(dist, i, pi, k)| (i, pi, k, o + d * dist))
    }

    /// Camera eye in part `oi`'s LOCAL mesh frame. Topology (normals, centroids)
    /// lives in local space, so the front-face mask needs the eye there:
    /// world = R·(s·local) + T  ⇒  local = Rᵀ·(world − T)/s.
    fn eye_in_local(&self, oi: usize) -> glam::Vec3 {
        let t = self.objects[oi].transform();
        let e = self.camera.eye();
        let dw = [
            e.x as f64 - t.translation[0],
            e.y as f64 - t.translation[1],
            e.z as f64 - t.translation[2],
        ];
        let mut loc = [0.0f64; 3];
        for j in 0..3 {
            for i in 0..3 {
                loc[j] += t.rotation[i][j] * dw[i];
            }
            loc[j] /= t.scale;
        }
        glam::Vec3::new(loc[0] as f32, loc[1] as f32, loc[2] as f32)
    }

    /// Build (or reuse) the cached flood topology for part `(oi, pi)` — the one
    /// expensive step; strokes/re-clicks on the same part reuse it.
    fn ensure_topo(&mut self, oi: usize, pi: usize) {
        if self.paint_topo.as_ref().map_or(true, |(a, b, _)| *a != oi || *b != pi) {
            let topo = paint::PaintTopology::build(&self.objects[oi].parts[pi].display);
            self.paint_topo = Some((oi, pi, topo));
        }
    }

    /// Stamp a flooded face set into `paint_tri` (the brush tool, or clear when
    /// erasing). `drift_max` drops faces past that normal drift from the seed
    /// (the smart-fill gate); `None` takes every face (the freehand brush).
    fn apply_region(&mut self, oi: usize, pi: usize, region: &[paint::FloodFace], drift_max: Option<f32>) {
        let val = if self.brush_erase { None } else { Some(self.brush_tool) };
        let part = &mut self.objects[oi].parts[pi];
        let ntri = part.display.triangles.len();
        if part.paint_tri.len() != ntri {
            part.paint_tri = vec![None; ntri];
        }
        let mut changed = false;
        for f in region {
            if drift_max.is_some_and(|dm| f.drift > dm) {
                continue;
            }
            if part.paint_tri[f.face as usize] != val {
                part.paint_tri[f.face as usize] = val;
                changed = true;
            }
        }
        if changed {
            self.mesh_paint_dirty = true;
        }
        // The mesh tint above is just the live model-view feedback; the SLICE
        // truth is a sub-bead dab of the same flooded patch (applied by
        // `dab_covers` when sliced/exported — bead-resolution, not per-triangle).
        self.record_dab(oi, pi, region, drift_max);
    }

    /// Record a sub-bead paint dab from a flooded region — the drift-gated faces
    /// in world space, bounded by a sphere for cheap culling — so a Model-view
    /// stroke resolves to bead-resolution paint (`engine::dab_covers`) at slice
    /// time, matching the mesh tint shown live. Empty region ⇒ no dab.
    fn record_dab(&mut self, oi: usize, pi: usize, region: &[paint::FloodFace], drift_max: Option<f32>) {
        let t = self.objects[oi].transform();
        let disp = &self.objects[oi].parts[pi].display;
        let tris: Vec<[[f64; 3]; 3]> = region
            .iter()
            .filter(|f| drift_max.map_or(true, |dm| f.drift <= dm))
            .map(|f| {
                let tr = disp.triangle(f.face as usize);
                [t.apply(tr[0]), t.apply(tr[1]), t.apply(tr[2])]
            })
            .collect();
        if tris.is_empty() {
            return;
        }
        // Bounding sphere (centroid + max radius) of the patch — used only to
        // cull far beads; works for a small freehand dab or a big smart-fill.
        let mut c = [0.0f64; 3];
        for tr in &tris {
            for v in tr {
                c[0] += v[0];
                c[1] += v[1];
                c[2] += v[2];
            }
        }
        let inv = 1.0 / (tris.len() * 3) as f64;
        c = [c[0] * inv, c[1] * inv, c[2] * inv];
        let mut r2 = 0.0f64;
        for tr in &tris {
            for v in tr {
                let d = [v[0] - c[0], v[1] - c[1], v[2] - c[2]];
                r2 = r2.max(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            }
        }
        let dab = BeadDab {
            center: glam::Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32),
            radius: r2.sqrt() as f32,
            tool: if self.brush_erase { None } else { Some(self.brush_tool) },
            tris,
        };
        // If already sliced, stamp this dab onto the beads NOW (incremental — no
        // GPU upload; that lands on stroke end). Otherwise it applies at the next
        // slice. Keeps a live stroke cheap: no full replay per frame.
        if self.sliced.is_some() && !self.bead_pristine.is_empty() {
            self.recolor_beads(&dab);
        }
        self.bead_dabs.push(dab);
    }

    /// Smart-fill the surface region under a paint-mode click: flood from the
    /// front face bounded by the live drift + reach, and stamp the brush tool.
    fn paint_click(&mut self, o: glam::Vec3, d: glam::Vec3) {
        let Some((oi, pi, tri, _)) = self.pick_paint_face(o, d) else {
            return;
        };
        self.ensure_topo(oi, pi);
        let s = (self.objects[oi].transform().scale as f32).max(1e-6);
        let params = paint::FloodParams {
            theta_local: (PAINT_THETA_LOCAL_DEG as f32).to_radians(),
            max_dist: self.brush_radius_mm / s,
            // "Paint what you can see": don't wrap onto faces turned away from
            // the camera (the dress interior, the far side).
            front_faces_only: true,
            eye_local: self.eye_in_local(oi),
        };
        let region = self.paint_topo.as_ref().unwrap().2.flood(tri, &params);
        self.apply_region(oi, pi, &region, Some(self.brush_drift_deg.to_radians()));
    }

    /// The brush's on-surface RADIUS in mm. The `brush_dab_mm` slider value is
    /// the spot DIAMETER — what the user reads as the brush size, so a "6" paints
    /// a ~6 mm-wide spot — hence the radius the paint/cursor tests use is half it.
    fn dab_radius(&self) -> f32 {
        self.brush_dab_mm * 0.5
    }

    /// Freehand brush: flood the CONNECTED front-facing surface within
    /// `dab_radius()` GEODESIC (along-the-surface) distance of the hit — a disc
    /// that hugs the surface, NOT a Euclidean sphere, so it never jumps onto a
    /// disconnected surface that merely happens to be nearby in space. No crease
    /// gate (a manual brush paints across corners); front-face keeps it off the
    /// hidden side. Called per drag frame for a stroke — the correction for what
    /// the flood over/under-grabbed.
    fn paint_dab(&mut self, o: glam::Vec3, d: glam::Vec3) {
        let Some((oi, pi, tri, _)) = self.pick_paint_face(o, d) else {
            return;
        };
        self.ensure_topo(oi, pi);
        let s = (self.objects[oi].transform().scale as f32).max(1e-6);
        let params = paint::FloodParams {
            theta_local: std::f32::consts::PI, // cross any edge — it's a manual brush
            max_dist: self.dab_radius() / s,
            front_faces_only: true,
            eye_local: self.eye_in_local(oi),
        };
        let region = self.paint_topo.as_ref().unwrap().2.flood(tri, &params);
        self.apply_region(oi, pi, &region, None);
    }

    /// Screen-space boundary of the region the freehand brush would paint at the
    /// cursor — the SAME geodesic disc the brush floods, traced as its boundary
    /// edges and projected to screen, so the cursor drapes over the real surface
    /// instead of being a flat tangent circle. `None` if the cursor isn't over a
    /// paintable front face.
    fn paint_cursor_outline(
        &mut self,
        vp: glam::Mat4,
        rect: egui::Rect,
        pointer: egui::Pos2,
    ) -> Option<Vec<[egui::Pos2; 2]>> {
        let (o, d) = pointer_ray(vp, rect, pointer);
        let (oi, pi, tri, _) = self.pick_paint_face(o, d)?;
        self.ensure_topo(oi, pi);
        let s = (self.objects[oi].transform().scale as f32).max(1e-6);
        // The brush floods the CONNECTED front-surface patch within `dab_radius`
        // of the click and paints the beads hugging it — so the cursor is that
        // same flooded patch's boundary, draped over the real surface. Identical
        // for the mesh brush (Model) and bead brush (Preview): both paint this
        // patch.
        let params = paint::FloodParams {
            theta_local: std::f32::consts::PI,
            max_dist: self.dab_radius() / s,
            front_faces_only: true,
            eye_local: self.eye_in_local(oi),
        };
        let region = self.paint_topo.as_ref().unwrap().2.flood(tri, &params);
        if region.is_empty() {
            return None;
        }
        let t = self.objects[oi].transform();
        let disp = &self.objects[oi].parts[pi].display;
        let face_set: std::collections::HashSet<u32> = region.iter().map(|f| f.face).collect();
        // Boundary edge = shared by exactly ONE face in the kept set (the outer
        // rim of the disc); interior edges are shared by two.
        let mut edge_count: std::collections::HashMap<(u32, u32), u32> =
            std::collections::HashMap::new();
        for &f in &face_set {
            let t3 = disp.triangles[f as usize];
            for (a, b) in [(t3[0], t3[1]), (t3[1], t3[2]), (t3[2], t3[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }
        let project = |vi: u32| -> Option<egui::Pos2> {
            let p = disp.vertices[vi as usize];
            let q = t.apply(p);
            let clip = vp * glam::Vec3::new(q[0] as f32, q[1] as f32, q[2] as f32).extend(1.0);
            if clip.w <= 1e-4 {
                return None;
            }
            let sx = rect.min.x + (clip.x / clip.w * 0.5 + 0.5) * rect.width();
            let sy = rect.min.y + (1.0 - (clip.y / clip.w * 0.5 + 0.5)) * rect.height();
            Some(egui::pos2(sx, sy))
        };
        let mut segs = Vec::new();
        for (&(a, b), &count) in &edge_count {
            if count == 1 {
                if let (Some(sa), Some(sb)) = (project(a), project(b)) {
                    segs.push([sa, sb]);
                }
            }
        }
        Some(segs)
    }

    /// Clear all surface paint on every part.
    fn clear_paint(&mut self) {
        for o in &mut self.objects {
            for p in &mut o.parts {
                p.paint_tri.clear();
            }
        }
        self.mesh_paint_dirty = true;
    }

    /// Per-vertex paint tint for the whole scene, in the SAME object→part→
    /// display-triangle order (3 verts/triangle) that `upload_mesh_geometry`
    /// lays down — so it lines up with the geometry buffer. Painted triangles
    /// carry their tool's display color at full coverage; unpainted are zero.
    fn build_mesh_paint(&self) -> Vec<[f32; 4]> {
        let mut out = Vec::new();
        for o in &self.objects {
            for part in &o.parts {
                for k in 0..part.display.triangles.len() {
                    let entry = match part.paint_tri.get(k).copied().flatten() {
                        Some(t) => {
                            let c = render::visible_against_backdrop(
                                self.settings.tool(t as usize).color_rgb,
                            );
                            [c[0], c[1], c[2], 1.0]
                        }
                        None => [0.0; 4],
                    };
                    out.push(entry);
                    out.push(entry);
                    out.push(entry);
                }
            }
        }
        out
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
        if self.show_ironing {
            m |= 1 << 8;
        }
        if self.show_surface {
            m |= 1 << 9;
        }
        m
    }

    fn reresolve(&mut self) {
        // The flat resolve reads the machine's tool count (a printer-tier
        // fact), the slot list syncs to it, then a toolchanger re-resolves
        // through every loaded slot so the cross-tool facts (shared bed temp,
        // the binding flow ceiling, per-slot colors) are real.
        if let Ok(flat) = self.profiles.resolve(&self.printer, &self.filament, &self.process) {
            self.sync_tool_slots(flat.tool_count);
            self.settings = if flat.tool_count > 1 {
                let names: Vec<&str> = self.tools.iter().map(String::as_str).collect();
                self.profiles.resolve_tools(&self.printer, &names, &self.process).unwrap_or(flat)
            } else {
                flat
            };
            self.settings.auto_center_on_bed = false;
            self.apply_tool_colors();
            self.baseline = self.settings.clone();
            self.sliced = None;
            self.slice_summary = None;
            self.view_preview = false;
            self.mark_scene_dirty();
            self.refit_camera = true;
        }
        self.refresh_pins();
    }

    /// Machines don't share spools: leaving a printer stashes its loadout
    /// (per-slot filaments, spool colors, blend palette, process) under the
    /// outgoing name and arriving at one restores its own. A printer seen
    /// for the first time inherits the outgoing loadout as its seed — also
    /// the pre-loadout behavior. Call with the OUTGOING printer's name after
    /// `self.printer` has moved, before `reresolve`.
    fn switch_printer_loadout(&mut self, old: &str) {
        if old == self.printer {
            return;
        }
        self.loadouts.insert(old.to_string(), self.current_loadout());
        if let Some(lo) = self.loadouts.get(&self.printer).cloned() {
            self.apply_loadout(&lo);
        }
    }

    /// Bring a stored loadout onto the live fields: slots (a vanished
    /// filament profile falls back to the tier selection), spool colors,
    /// blends, and — where it still resolves — the machine's process.
    /// `reresolve` afterwards syncs the slot count and the tool table.
    fn apply_loadout(&mut self, lo: &config::Loadout) {
        if !lo.tools.is_empty() {
            self.tools = lo
                .tools
                .iter()
                .map(|t| {
                    if self.profiles.filament_names().contains(&t.as_str()) {
                        t.clone()
                    } else {
                        self.filament.clone()
                    }
                })
                .collect();
            self.filament = self.tools[0].clone();
        }
        self.tool_colors = (0..self.tools.len())
            .map(|i| {
                lo.tool_colors
                    .get(i)
                    .and_then(|h| config::parse_hex_color(h))
                    .map(|c| (self.tools[i].clone(), c))
            })
            .collect();
        self.blends = lo.blends.clone();
        if !lo.process.is_empty() && self.profiles.process_names().contains(&lo.process.as_str()) {
            self.process = lo.process.clone();
        }
    }

    /// Keep the slot list in step with the machine: `count` entries, slot 0
    /// mirroring the filament tier, new slots opening with the same spool.
    fn sync_tool_slots(&mut self, count: usize) {
        let n = count.max(1);
        let was_single = self.tools.len() <= 1;
        self.tools.truncate(n);
        while self.tools.len() < n {
            self.tools.push(self.filament.clone());
        }
        self.tools[0] = self.filament.clone();
        self.active_tool_tab = self.active_tool_tab.min(n - 1);
        if n == 1 && self.color_by == ColorBy::Filament {
            self.color_by = ColorBy::Feature; // the mode only exists on a toolchanger
        } else if n > 1 && was_single && self.color_by == ColorBy::Feature {
            // Arriving at a toolchanger defaults the preview to filament
            // colors — the mode that shows what the machine is for. An
            // explicit LayerTime pick survives the trip.
            self.color_by = ColorBy::Filament;
        }
    }

    /// Re-resolve just the per-slot `ToolSettings` (colors, temps, names)
    /// after the slot list or the tool count changed — unlike `reresolve`,
    /// panel edits outside the tool table are kept.
    fn resync_tools(&mut self) {
        self.sync_tool_slots(self.settings.tool_count);
        // A slot naming a vanished profile (deleted user filament) falls back
        // to the tier selection instead of erroring the whole resolve.
        for t in &mut self.tools[1..] {
            if !self.profiles.filament_names().contains(&t.as_str()) {
                *t = self.filament.clone();
            }
        }
        let names: Vec<&str> = self.tools.iter().map(String::as_str).collect();
        let resolved = match self.profiles.resolve_tools(&self.printer, &names, &self.process) {
            Ok(s) => s.tools,
            Err(_) => self.tools.iter().map(|n| self.settings.flat_tool(n.clone())).collect(),
        };
        if self.settings.tool_count > 1 {
            // The tabs edit `settings.tools` directly, so a resync must not
            // wipe unsaved (*) edits on slots it didn't move: a slot whose
            // fresh resolve matches its baseline keeps the edited view, one
            // whose profile changed (slot switch, save, delete fallback)
            // snaps to the fresh resolve. The baseline follows the SAME
            // resolve, so a bare slot switch never reads as dirty.
            let mut tools = resolved.clone();
            for (i, t) in tools.iter_mut().enumerate() {
                let unmoved = self.baseline.tools.get(i).is_some_and(|b| {
                    b.filament_name == t.filament_name
                        && FilamentProfile::diff_tool(b, t).is_empty()
                });
                if unmoved {
                    if let Some(edited) = self.settings.tools.get(i) {
                        *t = edited.clone();
                    }
                }
            }
            self.settings.tools = tools;
            self.baseline.tools = resolved;
        } else {
            self.settings.tools = resolved;
            // Slot 0 mirrors the flat filament fields — unsaved edits included.
            let t0 = self.settings.flat_tool(self.filament.clone());
            self.settings.tools[0] = t0;
        }
        self.apply_tool_colors();
    }

    /// Pin state comes from the selected profiles: a field the profile chain
    /// sets explicitly is pinned; one it leaves unset follows auto.
    fn refresh_pins(&mut self) {
        self.pins = self.profile_pins();
    }

    /// Which auto-capable fields the PROFILE chain pins (the live `pins`
    /// start here and diverge as the user drags) — the auto sliders' revert
    /// target.
    fn profile_pins(&self) -> Pins {
        if let (Ok(pc), Ok(pr), Ok(_)) = (
            self.profiles.merged_process(&self.process),
            self.profiles.merged_printer(&self.printer),
            self.profiles.merged_filament(&self.filament),
        ) {
            Pins {
                line_width: pc.line_width_mm.is_some(),
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
            }
        } else {
            self.pins
        }
    }

    /// Recompute every unpinned derivable setting from its master, so dragging
    /// print speed (or changing the nozzle) visibly moves its dependents.
    fn apply_auto(&mut self) {
        let s = &mut self.settings;
        // The data-driven chain: bead from the nozzle, first-layer temp from
        // the operating temperature + the material's adhesion bump, nominal
        // speed from the machine rating × the finish↔speed dial, features from
        // the nominal under the melt ceiling.
        if !self.pins.line_width {
            s.line_width_mm = config::derived_line_width_mm(s.nozzle_diameter_mm);
        }
        s.first_layer_nozzle_temp_c =
            config::derived_first_layer_temp_c(s.nozzle_temp_c, s.material);
        // Per-tool the same derivation: the tabs edit each slot's operating
        // temperature, and its first-layer temp follows (never a slider).
        for t in &mut s.tools {
            t.first_layer_nozzle_temp_c =
                config::derived_first_layer_temp_c(t.nozzle_temp_c, t.material);
        }
        s.print_speed_mm_s = config::derived_print_speed_mm_s(s.machine_speed_mm_s, s.speed_quality);
        let cap = config::flow_speed_cap_mm_s(s.max_volumetric_speed_mm3_s, s.line_width_mm, s.layer_height_mm);
        s.external_perimeter_speed_mm_s =
            config::derived_external_perimeter_speed_mm_s(s.print_speed_mm_s, cap);
        s.solid_speed_mm_s = config::derived_solid_speed_mm_s(s.print_speed_mm_s, cap);
        s.support_speed_mm_s = config::derived_support_speed_mm_s(s.print_speed_mm_s, cap);
        s.overhang_speed_mm_s = config::derived_overhang_speed_mm_s(s.bridge_speed_mm_s);
        if !self.pins.outer_wall_accel {
            s.outer_wall_accel_mm_s2 = config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2);
        }
        if !self.pins.first_layer_accel {
            s.first_layer_accel_mm_s2 = config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2);
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

    /// Does slot `i` carry unsaved per-tool edits? (Toolchanger tabs edit
    /// `settings.tools[i]`; the diff excludes spool color and the derived
    /// first-layer temperature by construction.)
    fn tool_dirty(&self, i: usize) -> bool {
        match (self.settings.tools.get(i), self.baseline.tools.get(i)) {
            (Some(cur), Some(base)) => !FilamentProfile::diff_tool(cur, base).is_empty(),
            _ => false,
        }
    }

    /// Per-tier dirty flags vs. the baseline, ignoring unpinned auto fields.
    /// On a toolchanger the filament entry is any slot's per-tool dirt — the
    /// flat fields are only the tool-0 mirror there, not an edit surface.
    fn tier_dirty_masked(&self) -> [bool; 3] {
        let mut pr = PrinterProfile::diff(&self.settings, &self.baseline);
        self.mask_auto_printer(&mut pr);
        let fl_dirty = if self.settings.tool_count > 1 {
            (0..self.settings.tools.len()).any(|i| self.tool_dirty(i))
        } else {
            !FilamentProfile::diff(&self.settings, &self.baseline).is_empty()
        };
        let pc = ProcessProfile::diff(&self.settings, &self.baseline);
        [!pr.is_empty(), fl_dirty, !pc.is_empty()]
    }

    /// Re-resolve only the dirty baseline (after a save) — keeps the user's
    /// current panel edits in other tiers intact.
    fn refresh_baseline(&mut self) {
        // Freshen the slot table first (a filament save may have renamed the
        // tier selection — slot 0 must follow).
        self.resync_tools();
        let names: Vec<String> = self.tools.clone();
        let refs: Vec<&str> = names.iter().map(String::as_str).collect();
        let b = if refs.len() > 1 {
            self.profiles.resolve_tools(&self.printer, &refs, &self.process)
        } else {
            self.profiles.resolve(&self.printer, &self.filament, &self.process)
        };
        if let Ok(mut b) = b {
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
            if kind == TierKind::Printer {
                // The machine is gone, its loadout with it; the fallback
                // printer brings in its own.
                self.loadouts.remove(name);
                if let Some(lo) = self.loadouts.get(&self.printer).cloned() {
                    self.apply_loadout(&lo);
                }
                self.reresolve();
            }
        } else if kind == TierKind::Filament {
            // A non-selected filament may still be loaded in a tool slot —
            // those fall back to the tier selection.
            self.resync_tools();
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
                    let tris: usize =
                        items.iter().flat_map(|it| &it.parts).map(|p| p.mesh.triangles.len()).sum();
                    // Imports land on a fresh bed; existing beds are never
                    // disturbed.
                    let dest = self.first_empty_bed();
                    let first_new = self.objects.len();
                    let tool_cap = self.settings.tool_count.saturating_sub(1) as u32;
                    for (k, it) in items.into_iter().enumerate() {
                        // Bambu/Orca name a merged multi-part object after its
                        // FIRST part (object 5 = "Beak", parts Beak.stl/Body.stl/
                        // …), which reads as a mislabel in the outliner. Treat
                        // that echo as anonymous and fall back to the file name.
                        let named_after_first_part = it.parts.len() > 1
                            && it
                                .parts
                                .first()
                                .is_some_and(|p| it.name.as_str() == mesh_name_stem(&p.name));
                        let name = if !it.name.is_empty() && !named_after_first_part {
                            it.name
                        } else if n == 1 {
                            file.clone()
                        } else {
                            format!("{file} #{}", k + 1)
                        };
                        // Parts keep their identity — and their extruder hint
                        // (1-based) becomes the tool slot, clamped to what
                        // this machine has.
                        let parts: Vec<ScenePart> = it
                            .parts
                            .into_iter()
                            .enumerate()
                            .map(|(j, p)| ScenePart {
                                name: if p.name.is_empty() { format!("part {}", j + 1) } else { p.name },
                                paint: PartColor::Tool(
                                    p.extruder.map(|e| e.saturating_sub(1)).unwrap_or(0).min(tool_cap),
                                ),
                                paint_tri: Vec::new(),
                                display: Arc::new(p.mesh.display_mesh()),
                                mesh: Arc::new(p.mesh),
                            })
                            .collect();
                        let mut obj = SceneObject {
                            name,
                            parts,
                            rot_deg: [0.0; 3],
                            scale: 1.0,
                            pos: [0.0, 0.0],
                            bounds_cache: std::cell::Cell::new(None),
                        };
                        // pos = the baked footprint center reproduces the
                        // file's build placement (SceneObject::transform
                        // recenters the footprint on pos).
                        let (minx, miny, maxx, maxy, _) = obj.footprint();
                        obj.pos = [(minx + maxx) / 2.0, (miny + maxy) / 2.0];
                        self.objects.push(obj);
                    }
                    // The build's own layout wins while it fits one bed:
                    // shift the whole group onto the destination. Otherwise
                    // (multi-plate projects span virtual plates far wider
                    // than any bed) grid the newcomers across fresh beds.
                    let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
                    let fits = self.objects[first_new..].iter().all(|o| {
                        let (minx, miny, maxx, maxy, _) = o.footprint();
                        let (w, h) = ((maxx - minx) / 2.0, (maxy - miny) / 2.0);
                        let c = o.pos;
                        c[0] - w >= 0.0 && c[1] - h >= 0.0 && c[0] + w <= bx && c[1] + h <= by
                    });
                    if fits {
                        let ox = bed_origin_x(dest, bx);
                        for o in &mut self.objects[first_new..] {
                            o.pos[0] += ox;
                        }
                    } else {
                        let idx: Vec<usize> = (first_new..self.objects.len()).collect();
                        self.arrange_from(dest, &idx);
                    }
                    self.selected = Some(self.objects.len() - 1);
                    self.status = if n == 1 {
                        format!("Imported {file} ({tris} triangles)")
                    } else {
                        format!("Imported {file}: {n} objects ({tris} triangles)")
                    };
                    self.set_active_bed(dest);
                    // Glide-frame the destination bed (even when it was
                    // already active) — no instant refit pop.
                    self.recenter_camera = true;
                    self.scene_dirty();
                }
                Err(e) => self.status = format!("Load failed: {e}"),
            }
            return;
        }
        match mesh::Mesh::load_stl(&path) {
            Ok(m) => {
                self.status = format!("Imported {file} ({} triangles)", m.triangles.len());
                let mut obj = SceneObject::new(file, m);
                let dest = self.first_empty_bed();
                obj.pos = self.bed_center(dest);
                self.objects.push(obj);
                self.selected = Some(self.objects.len() - 1);
                self.set_active_bed(dest);
                self.recenter_camera = true;
                self.scene_dirty();
            }
            Err(e) => self.status = format!("Load failed: {e}"),
        }
    }

    fn duplicate_selected(&mut self) {
        let Some(i) = self.selected else { return };
        let src = &self.objects[i];
        let (minx, _, maxx, _, _) = src.footprint();
        let w = maxx - minx;
        // Beside the source if its bed has room to the right; otherwise the
        // copy gets a fresh bed. Nothing else moves.
        let bx = self.settings.bed_size_x_mm;
        let bed_right = bed_origin_x(self.bed_of(src), bx) + bx;
        let mut pos = [src.pos[0] + w + 5.0, src.pos[1]];
        if pos[0] + w / 2.0 > bed_right {
            pos = self.bed_center(self.first_empty_bed());
        }
        let copy = SceneObject {
            name: format!("{} copy", src.name),
            parts: src
                .parts
                .iter()
                .map(|p| ScenePart {
                    name: p.name.clone(),
                    mesh: Arc::clone(&p.mesh),
                    display: Arc::clone(&p.display),
                    paint: p.paint,
                    paint_tri: Vec::new(),
                })
                .collect(),
            rot_deg: src.rot_deg,
            scale: src.scale,
            pos,
            bounds_cache: std::cell::Cell::new(None),
        };
        self.objects.push(copy);
        self.selected = Some(self.objects.len() - 1);
        self.scene_dirty();
    }

    /// Split every part of the selected object into its connected components —
    /// separate solids in one mesh (the goose's two feet in "Legs.stl") become
    /// their own parts, each independently colorable. Single-body parts are
    /// left as-is; a part's paint carries to all its bodies.
    fn split_selected_parts(&mut self) {
        let Some(i) = self.selected else { return };
        let mut new_parts: Vec<ScenePart> = Vec::new();
        let mut gained = 0usize;
        for part in std::mem::take(&mut self.objects[i].parts) {
            let bodies = part.mesh.split_connected();
            if bodies.len() <= 1 {
                new_parts.push(part);
                continue;
            }
            gained += bodies.len() - 1;
            let stem = mesh_name_stem(&part.name);
            let base = if stem.is_empty() { "part".to_string() } else { stem.to_string() };
            for (k, body) in bodies.into_iter().enumerate() {
                new_parts.push(ScenePart {
                    name: format!("{base} {}", k + 1),
                    display: Arc::new(body.display_mesh()),
                    mesh: Arc::new(body),
                    paint: part.paint,
                    paint_tri: Vec::new(),
                });
            }
        }
        let count = new_parts.len();
        self.objects[i].parts = new_parts;
        if gained > 0 {
            self.scene_dirty();
            self.status = format!("Split ‘{}’ into {count} parts", self.objects[i].name);
        } else {
            self.status = "Nothing to split — every part is already a single body".into();
        }
    }

    fn delete_selected(&mut self) {
        let Some(i) = self.selected else { return };
        self.objects.remove(i);
        self.selected = if self.objects.is_empty() {
            None
        } else {
            Some(i.min(self.objects.len() - 1))
        };
        // No re-arrange and no bed removal: survivors keep their places, and
        // a bed left empty by the delete persists (only `−` removes a bed).
        self.scene_dirty();
    }

    /// Which bed an object sits on (by its world position).
    fn bed_of(&self, obj: &SceneObject) -> usize {
        bed_of_pos(obj.pos[0], self.settings.bed_size_x_mm)
    }

    /// How many beds exist (explicit — see `bed_count`).
    fn n_beds(&self) -> usize {
        self.bed_count
    }

    /// Grow the bed count so every object has a bed under it — never shrinks
    /// (empty beds persist until explicitly removed). Run whenever objects
    /// move; cheap, so `scene_dirty` calls it unconditionally.
    fn grow_beds_to_fit(&mut self) {
        let need = self.objects.iter().map(|o| self.bed_of(o) + 1).max().unwrap_or(0);
        self.bed_count = self.bed_count.max(need).max(1);
    }

    /// The lowest bed with nothing on it (where imports land) — an existing
    /// empty bed if there is one, else a brand-new index past the last bed.
    fn first_empty_bed(&self) -> usize {
        let mut k = 0;
        while self.objects.iter().any(|o| self.bed_of(o) == k) {
            k += 1;
        }
        k
    }

    /// Make bed `k` active: the camera pivots onto it, the highlight moves,
    /// and slicing targets it.
    fn set_active_bed(&mut self, k: usize) {
        if self.active_bed != k {
            self.active_bed = k;
            self.recenter_camera = true;
            // Bed highlight: only the bed grid's active index and the selection
            // spotlight change — the mesh is untouched.
            self.beds_dirty = true;
            self.spotlight_dirty = true;
        }
    }

    /// World center of bed `k`.
    fn bed_center(&self, k: usize) -> [f64; 2] {
        let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
        [bed_origin_x(k, bx) + bx / 2.0, by / 2.0]
    }

    /// True if the object's footprint hangs past its own bed's XY edges.
    fn off_its_bed(&self, obj: &SceneObject) -> bool {
        let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
        let ox = bed_origin_x(self.bed_of(obj), bx);
        let (minx, miny, maxx, maxy, _) = obj.footprint();
        let (w, h) = ((maxx - minx) / 2.0, (maxy - miny) / 2.0);
        obj.pos[0] - w < ox || obj.pos[0] + w > ox + bx || obj.pos[1] - h < 0.0 || obj.pos[1] + h > by
    }

    /// Why object `i` can't be printed — out of its bed's build volume (the
    /// offending axes) and/or overlapping another object on the same bed.
    /// None = printable. Reads the cached bounds (`rebuild_scene` keeps them
    /// fresh), so it's cheap to call per frame.
    fn obj_problem(&self, i: usize) -> Option<String> {
        let Some(b) = self.obj_bounds.get(i) else { return None };
        const EPS: f64 = 1.0e-3;
        let (bx, by, bz) = (
            self.settings.bed_size_x_mm,
            self.settings.bed_size_y_mm,
            self.settings.bed_size_z_mm,
        );
        let ox = bed_origin_x(b.bed, bx);
        let mut axes = Vec::new();
        if b.aabb[0] < ox - EPS || b.aabb[2] > ox + bx + EPS {
            axes.push("X");
        }
        if b.aabb[1] < -EPS || b.aabb[3] > by + EPS {
            axes.push("Y");
        }
        if b.height > bz + EPS {
            axes.push("Z");
        }
        let mut parts = Vec::new();
        if !axes.is_empty() {
            parts.push(format!("outside build volume ({})", axes.join("/")));
        }
        // Footprint (AABB) overlap with any other object sharing this bed.
        let collides = self
            .obj_bounds
            .iter()
            .enumerate()
            .any(|(j, bj)| j != i && bj.bed == b.bed && aabb_overlap(b.aabb, bj.aabb));
        if collides {
            parts.push("overlapping another object".to_string());
        }
        (!parts.is_empty()).then(|| parts.join("; "))
    }

    /// Names of active-bed objects that can't be printed (off the bed or
    /// overlapping). Empty = the bed is printable.
    fn active_bed_violations(&self) -> Vec<String> {
        (0..self.objects.len())
            .filter(|&i| self.bed_of(&self.objects[i]) == self.active_bed && self.obj_problem(i).is_some())
            .map(|i| self.objects[i].name.clone())
            .collect()
    }

    /// Invalidate slice/preview and refresh the scene after objects changed.
    fn scene_dirty(&mut self) {
        self.grow_beds_to_fit();
        self.sliced = None;
        self.slice_summary = None;
        self.view_preview = false;
        self.mark_scene_dirty();
    }

    /// Full scene rebuild next frame: mesh geometry (part set may have changed),
    /// beds, and the selection spotlight (import / delete / arrange / profile /
    /// bed). Re-uploads geometry — use `mark_geom_dirty` for a mere move.
    fn mark_scene_dirty(&mut self) {
        self.mesh_struct_dirty = true;
        self.beds_dirty = true;
        self.spotlight_dirty = true;
    }

    /// An object moved/rotated/scaled: rewrite only the per-part model matrices
    /// (+refresh the bounds cache) and move the selection spotlight to follow.
    /// The mesh geometry buffer and beds are untouched.
    fn mark_geom_dirty(&mut self) {
        self.mesh_xform_dirty = true;
        self.spotlight_dirty = true;
    }

    /// Only per-part tints changed (spool color / blend / accent): rewrite just
    /// the per-part color uniforms. Geometry, matrices, beds, spotlight, and the
    /// bounds cache all stay put.
    fn mark_mesh_color_dirty(&mut self) {
        self.mesh_color_dirty = true;
    }

    /// True while any scene resource is awaiting an upload.
    fn scene_dirty_any(&self) -> bool {
        self.beds_dirty
            || self.spotlight_dirty
            || self.mesh_struct_dirty
            || self.mesh_xform_dirty
            || self.mesh_color_dirty
            || self.mesh_paint_dirty
    }

    /// Re-layout every object, flowing across beds (shelf packing — see
    /// `shelf_pack`).
    fn arrange(&mut self) {
        let all: Vec<usize> = (0..self.objects.len()).collect();
        self.arrange_from(0, &all);
    }

    /// Shelf-pack just `idx`, starting at `first_bed` and flowing onto
    /// subsequent beds as they fill. Objects always sit on z=0 via their
    /// baked transform.
    fn arrange_from(&mut self, first_bed: usize, idx: &[usize]) {
        if idx.is_empty() {
            return;
        }
        let sizes: Vec<(f64, f64)> = idx
            .iter()
            .map(|&i| {
                let f = self.objects[i].footprint();
                (f.2 - f.0, f.3 - f.1)
            })
            .collect();
        let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
        for (j, (bed, cx, cy)) in shelf_pack(&sizes, bx, by, 5.0).into_iter().enumerate() {
            self.objects[idx[j]].pos = [bed_origin_x(first_bed + bed, bx) + cx, cy];
        }
    }

    /// The blend's weights that name a slot this machine actually has, with
    /// zero/negative shares dropped. Empty = the blend references only
    /// missing tools (shrunk `tool_count`) and reads as neutral/tool 0.
    fn valid_weights(&self, blend: &config::BlendState) -> Vec<(u32, f32)> {
        valid_weights_for(self.settings.tool_count, blend)
    }

    /// The blend's dither repeat (mm) at the CURRENT layer height, when it
    /// overflows the blend band — the loud signal that the layers grew (or
    /// the band shrank) under an existing mix. Weights never rewrite
    /// themselves; the consequence surfaces here. None = fuses fine.
    fn blend_banding_mm(&self, blend: &config::BlendState) -> Option<f64> {
        blend_banding_for(
            self.settings.tool_count,
            self.settings.layer_height_mm,
            self.settings.blend_band_mm,
            blend,
        )
    }

    /// What a paint looks like: the tool slot's spool color, or the blend's
    /// linear-light mix over its valid weights (`mix_colors_linear` — what
    /// the interleaved layers read as at viewing distance). A blend with no
    /// valid weights, or a dangling index, reads the neutral grey.
    fn paint_display_rgb(&self, paint: PartColor) -> [f32; 3] {
        match paint {
            PartColor::Tool(t) => self.settings.tool(t as usize).color_rgb,
            PartColor::Blend(b) => match self.blends.get(b) {
                Some(blend) => {
                    let entries: Vec<([f32; 3], f32)> = self
                        .valid_weights(blend)
                        .into_iter()
                        .map(|(t, w)| (self.settings.tool(t as usize).color_rgb, w))
                        .collect();
                    if entries.is_empty() {
                        config::NEUTRAL_FILAMENT_RGB
                    } else {
                        config::mix_colors_linear(&entries)
                    }
                }
                None => config::NEUTRAL_FILAMENT_RGB,
            },
        }
    }

    /// What a paint prints as: the tool clamped to the machine's slots, or
    /// the blend's valid weights for the engine's layer dither. A blend with
    /// nothing valid degrades to tool 0.
    fn paint_engine(&self, paint: PartColor) -> engine::PartPaint {
        let cap = self.settings.tool_count.saturating_sub(1) as u32;
        match paint {
            PartColor::Tool(t) => engine::PartPaint::Tool(t.min(cap)),
            PartColor::Blend(b) => {
                let weights: Vec<(u32, f64)> = self
                    .blends
                    .get(b)
                    .map(|blend| {
                        self.valid_weights(blend).into_iter().map(|(t, w)| (t, w as f64)).collect()
                    })
                    .unwrap_or_default();
                if weights.is_empty() {
                    engine::PartPaint::Tool(0)
                } else {
                    engine::PartPaint::Blend(weights)
                }
            }
        }
    }

    /// The geometry signature of the active bed's parts — the same parts, in the
    /// same order and with the same empty-skip, that `baked_parts` produces, but
    /// carrying the source-mesh `Arc` + baked transform instead of the placed
    /// geometry (so it's cheap and the paint is excluded). Paired with the full
    /// settings, it captures everything a slice's GEOMETRY depends on.
    fn geom_signature(&self) -> GeomSig {
        let ox = bed_origin_x(self.active_bed, self.settings.bed_size_x_mm);
        let mut parts = Vec::new();
        for obj in self.objects.iter().filter(|o| self.bed_of(o) == self.active_bed) {
            let mut t = obj.transform();
            t.translation[0] -= ox;
            for part in &obj.parts {
                if !part.mesh.triangles.is_empty() {
                    parts.push((std::sync::Arc::clone(&part.mesh), t));
                }
            }
        }
        GeomSig { parts, settings: self.settings.clone() }
    }

    /// Bake the ACTIVE bed's parts into placed meshes, each paired with its
    /// paint, in that bed's local coordinates (the engine plans in [0, bed]
    /// space). Tools and blend weights clamp to the machine's slot count
    /// here (see `paint_engine`).
    fn baked_parts(&self) -> Vec<(mesh::Mesh, engine::PartPaint)> {
        let ox = bed_origin_x(self.active_bed, self.settings.bed_size_x_mm);
        let mut out: Vec<(mesh::Mesh, engine::PartPaint)> = Vec::new();
        for obj in self.objects.iter().filter(|o| self.bed_of(o) == self.active_bed) {
            let mut t = obj.transform();
            t.translation[0] -= ox;
            for part in &obj.parts {
                // `part.mesh` is already welded/indexed (STL import + 3MF load
                // both go through `from_triangle_soup`), and an affine placement
                // is injective — so baking is just a per-vertex map that reuses
                // the existing triangle indices. Re-welding through a HashMap
                // (the old soup + `from_triangle_soup` round-trip) recomputed
                // topology that can't have changed; `transformed()` is O(V) with
                // no hashing and yields byte-identical geometry.
                if !part.mesh.triangles.is_empty() {
                    // Surface paint is no longer baked into the slice — the
                    // freehand/smart-fill brush records sub-bead DABS
                    // (`bead_dabs`) that `apply_bead_dabs` stamps onto the sliced
                    // beads at bead resolution (see `apply_slice_output`). The
                    // slice itself carries only the part's base tool / blend.
                    let paint = self.paint_engine(part.paint);
                    out.push((part.mesh.transformed(&t), paint));
                }
            }
        }
        out
    }

    /// Slot 0 IS the flat filament view. Single-tool, the flat fields are
    /// the edit surface — copy them into the tool table before anything
    /// consumes `settings.tools`. On a toolchanger the arrow REVERSES: the
    /// per-tool tabs edit `settings.tools`, the flat fields are only the
    /// estimate/summary mirror, and copying flat over tools[0] here would
    /// erase T0's tab edits. Bed and chamber stay the shared-hardware
    /// aggregates (hottest tool wins), exactly as `resolve_tools` builds
    /// them — the emitter reads the flat values for both.
    fn refresh_tool0(&mut self) {
        let s = &mut self.settings;
        if s.tool_count > 1 {
            let Some(t0) = s.tools.first().cloned() else { return };
            let bed = s.tools.iter().map(|t| t.bed_temp_c).max().unwrap_or(t0.bed_temp_c);
            let chamber =
                s.tools.iter().map(|t| t.chamber_temp_c).max().unwrap_or(t0.chamber_temp_c);
            s.filament_color_rgb = t0.color_rgb;
            s.material = t0.material;
            s.filament_diameter_mm = t0.filament_diameter_mm;
            s.filament_density_g_cm3 = t0.filament_density_g_cm3;
            s.nozzle_temp_c = t0.nozzle_temp_c;
            s.first_layer_nozzle_temp_c = t0.first_layer_nozzle_temp_c;
            s.max_volumetric_speed_mm3_s = t0.max_volumetric_speed_mm3_s;
            s.max_flow_derate_per_c = t0.max_flow_derate_per_c;
            s.extrusion_multiplier = t0.extrusion_multiplier;
            s.pressure_advance = t0.pressure_advance;
            s.bridge_flow = t0.bridge_flow;
            s.bridge_speed_mm_s = t0.bridge_speed_mm_s;
            s.fan_speed = t0.fan_speed;
            s.bridge_fan_speed = t0.bridge_fan_speed;
            s.fan_off_layers = t0.fan_off_layers;
            s.aux_fan_speed = t0.aux_fan_speed;
            s.exhaust_fan_speed = t0.exhaust_fan_speed;
            s.standby_temp_c = t0.standby_temp_c;
            s.bed_temp_c = bed;
            s.chamber_temp_c = chamber;
        } else {
            let t0 = s.flat_tool(self.filament.clone());
            if let Some(slot) = s.tools.first_mut() {
                *slot = t0;
            }
        }
    }

    /// Load a different filament into slot `i` (any dirty-slot confirm has
    /// already happened). Slot 0 also moves the tier selection — the
    /// `filament ≡ tools[0]` invariant feeds resolve and persistence.
    fn apply_slot_switch(&mut self, i: usize, name: String) {
        if i == 0 {
            self.filament = name;
        } else if let Some(t) = self.tools.get_mut(i) {
            *t = name;
        }
        if self.settings.tool_count == 1 {
            // One tool: the FLAT fields are the engine's source, so a slot-0
            // switch is the old Filament tier switch — re-resolve the whole
            // chain (which also drops the stale plan).
            self.reresolve();
            return;
        }
        self.resync_tools();
        // The plan carries per-path tools and temps — stale.
        // (scene_dirty minus its camera/bounds side: the rebuild refreshes
        // tints and bumps content_version.)
        self.sliced = None;
        self.slice_summary = None;
        self.view_preview = false;
        self.mark_mesh_color_dirty();
    }

    /// Save slot `slot`'s per-tool diff as the user filament profile `name` —
    /// the tab counterpart of `save_profile`, same overwrite/inherit
    /// semantics against the slot's own profile instead of the tier row's.
    fn save_tool_profile(&mut self, slot: usize, name: &str) -> Result<(), String> {
        let mut diff = match (self.settings.tools.get(slot), self.baseline.tools.get(slot)) {
            (Some(cur), Some(base)) => FilamentProfile::diff_tool(cur, base),
            _ => return Err(format!("no tool slot {slot}")),
        };
        let slot_profile =
            self.tools.get(slot).cloned().unwrap_or_else(|| self.filament.clone());
        if name == slot_profile && self.profiles.is_user(TierKind::Filament, name) {
            let existing = self.profiles.get_filament(name).cloned().unwrap_or_default();
            let parent = existing.parent().map(str::to_string);
            diff = diff.over(existing);
            diff.inherits = parent;
        } else {
            diff.inherits = Some(slot_profile);
        }
        self.profiles.save_user_filament(name, diff)?;
        if let Some(t) = self.tools.get_mut(slot) {
            *t = name.to_string();
        }
        if slot == 0 {
            self.filament = name.to_string();
        }
        self.refresh_baseline();
        Ok(())
    }

    /// Lay the loaded-spool colors over the resolved tool table. An override
    /// remembered for a different filament than the slot now holds is
    /// dropped — the profile's own color takes over. Slot 0 also writes the
    /// flat color, so `refresh_tool0`'s snapshot carries it.
    fn apply_tool_colors(&mut self) {
        self.tool_colors.resize(self.settings.tool_count.max(1), None);
        for i in 0..self.settings.tools.len() {
            let slot_name =
                if i == 0 { &self.filament } else { self.tools.get(i).unwrap_or(&self.filament) };
            match &self.tool_colors[i] {
                Some((name, c)) if name == slot_name => {
                    self.settings.tools[i].color_rgb = *c;
                    if i == 0 {
                        self.settings.filament_color_rgb = *c;
                    }
                }
                Some(_) => self.tool_colors[i] = None,
                None => {}
            }
        }
    }

    fn slice(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        if self.slice_job.is_some() {
            return; // a background slice is already running
        }
        self.refresh_tool0();
        let parts = self.baked_parts();
        if parts.is_empty() {
            return;
        }
        // A re-slice keeps the layer the user was viewing (clamped in
        // `apply_slice_output`); only the first slice of a fresh model jumps to
        // the top.
        let resliced = self.sliced.is_some();
        // The preview belongs to the bed that was active at slice time —
        // instances bake its world offset, so it stays put if the user switches
        // beds afterward.
        let origin_x = bed_origin_x(self.active_bed, self.settings.bed_size_x_mm);
        let color_by = self.color_by;
        let accent = accent_hsl(self.accent);
        let z_hop = self.settings.z_hop_mm as f32;
        let sig = self.geom_signature();
        // If nothing but the parts' paint changed since the last full slice (a
        // tool reassignment or a blend edit — same meshes, same settings),
        // re-stamp the cached geometry: that's ~100 ms, so do it inline and land
        // the result this frame. `restamp_paint` is proven byte-identical to a
        // full slice (engine test `restamp_matches_full_slice`).
        if self.geom_cache.as_ref().is_some_and(|(cs, _)| cs.matches(&sig)) {
            let refs: Vec<(&mesh::Mesh, engine::PartPaint)> =
                parts.iter().map(|(m, paint)| (m, paint.clone())).collect();
            let layers = engine::restamp_paint(&self.geom_cache.as_ref().unwrap().1, &refs);
            let out =
                finish_slice(layers, None, &self.settings, color_by, accent, origin_x as f32, z_hop);
            self.apply_slice_output(out, sig, rs, resliced, origin_x);
        } else {
            // A geometry change: the heavy `plan_geometry` (seconds at high wall
            // counts) runs on a worker so the UI thread never blocks. `ui()`
            // polls the channel and commits + uploads when it lands.
            let settings = self.settings.clone();
            let (tx, rx) = std::sync::mpsc::channel();
            let progress = std::sync::Arc::new(engine::SliceProgress::new());
            let worker_progress = std::sync::Arc::clone(&progress);
            std::thread::spawn(move || {
                let refs: Vec<(&mesh::Mesh, engine::PartPaint)> =
                    parts.iter().map(|(m, paint)| (m, paint.clone())).collect();
                let geo = engine::plan_geometry_tracked(&refs, &settings, &worker_progress);
                let layers = engine::restamp_paint(&geo, &refs);
                let out = finish_slice(
                    layers,
                    Some(geo),
                    &settings,
                    color_by,
                    accent,
                    origin_x as f32,
                    z_hop,
                );
                let _ = tx.send(out);
            });
            self.slice_job = Some(SliceJob { rx, sig, resliced, origin_x, progress });
            self.status.clear();
        }
    }

    /// Commit a finished slice on the UI thread: publish the layers + summary,
    /// cache freshly-planned geometry, and upload the preview instances to the
    /// GPU (which must happen on the main thread that owns the wgpu queue).
    fn apply_slice_output(
        &mut self,
        out: SliceOutput,
        sig: GeomSig,
        rs: &eframe::egui_wgpu::RenderState,
        resliced: bool,
        origin_x: f64,
    ) {
        let n = out.layers.len();
        if let Some(geo) = out.geo {
            self.geom_cache = Some((sig, geo));
        }
        self.slice_summary = Some(out.summary);
        self.status.clear();
        self.slice_gen += 1;
        self.layer_stats = out.layer_stats;
        self.sliced = Some(out.layers);
        self.sliced_origin_x = origin_x;
        self.commit_bead_instances(out.verts, rs);
        self.scene.set_joints(&rs.device, &rs.queue, &out.joints);
        self.content_version += 1;
        self.layer_ends = out.ends;
        self.joint_layer_ends = out.joint_ends;
        self.preview_layer = if resliced {
            self.preview_layer.clamp(1, n.max(1))
        } else {
            n.max(1)
        };
        self.view_preview = true;
        // The worker built the beads UNsubdivided + unpainted (it can't see paint
        // state). If there's surface paint (dabs) — or we're actively painting —
        // rebuild the beads subdivided and stamp the dabs, and refresh the readout
        // so the preview + summary reflect the paint immediately after a (re)slice.
        if self.paint_mode || !self.bead_dabs.is_empty() {
            self.set_preview_instances(rs);
        }
        if !self.bead_dabs.is_empty() {
            self.refresh_bead_summary();
        }
    }

    /// (Re)build the preview bead instances from the sliced layers, colored
    /// per the active mode. Called after slicing and when the mode changes.
    fn set_preview_instances(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let Some(layers) = self.sliced.as_ref() else { return };
        let hop = self.settings.z_hop_mm;
        let layer_colors = self.layer_color_table();
        let (verts, ends, joints, joint_ends) = build_instances(
            layers,
            hop as f32,
            layer_colors.as_deref(),
            accent_hsl(self.accent),
            self.sliced_origin_x as f32,
            // Subdivide beads when there's surface paint (or we're painting), so a
            // dab boundary can fall mid-bead (sub-bead resolution).
            if self.paint_mode || !self.bead_dabs.is_empty() {
                Some(PAINT_BEAD_MAX_MM)
            } else {
                None
            },
        );
        self.commit_bead_instances(verts, rs);
        self.scene.set_joints(&rs.device, &rs.queue, &joints);
        self.content_version += 1;
        self.layer_ends = ends;
        self.joint_layer_ends = joint_ends;
    }

    /// Take a fresh bead instance buffer (from `build_instances` — pristine
    /// mesh-paint colors), keep it as the pristine base, build the spatial
    /// index, re-apply the accumulated bead dabs on top, and upload to the GPU.
    fn commit_bead_instances(
        &mut self,
        verts: Vec<[f32; 14]>,
        rs: &eframe::egui_wgpu::RenderState,
    ) {
        self.bead_grid = BeadGrid::build(&verts, BEAD_GRID_CELL_MM);
        self.bead_pristine = verts;
        self.replay_bead_dabs();
        self.scene.set_toolpaths(&rs.device, &rs.queue, &self.bead_inst);
    }

    /// Rebuild the working bead buffer from pristine and re-apply every dab in
    /// order. `mem::take` lets each `recolor_beads` borrow `&mut self` without
    /// fighting the dab list it's iterating.
    fn replay_bead_dabs(&mut self) {
        self.bead_inst = self.bead_pristine.clone();
        let dabs = std::mem::take(&mut self.bead_dabs);
        for dab in &dabs {
            self.recolor_beads(dab);
        }
        self.bead_dabs = dabs;
    }

    /// Recolor the beads a dab covers in the working buffer: paint → the dab's
    /// tool + display color; erase → revert to the pristine bead. A bead is
    /// covered only if it hugs the dab's flooded surface patch (connected, front,
    /// outer shell) — never a disconnected/back-side/interior bead nearby.
    fn recolor_beads(&mut self, dab: &BeadDab) {
        // Bounding-sphere query for candidates (padded for the patch's reach past
        // its center), then the exact patch-hugging test.
        let reach = dab.radius + 1.0;
        let beads = self.bead_grid.query(&self.bead_inst, dab.center, reach);
        for i in beads {
            let i = i as usize;
            let m = bead_mid(&self.bead_inst[i]);
            if !engine::dab_covers(&dab.tris, [m.x as f64, m.y as f64, m.z as f64]) {
                continue;
            }
            match dab.tool {
                Some(t) => {
                    let c = render::visible_against_backdrop(self.settings.tool(t as usize).color_rgb);
                    self.bead_inst[i][8] = c[0];
                    self.bead_inst[i][9] = c[1];
                    self.bead_inst[i][10] = c[2];
                    self.bead_inst[i][13] = t as f32;
                }
                None => self.bead_inst[i] = self.bead_pristine[i],
            }
        }
    }

    /// End of a Model-view paint stroke: the dabs were already stamped onto the
    /// beads incrementally in `record_dab`, so just upload the fresh bead buffer
    /// and refresh the readout, so a switch to Preview + the export reflect it.
    /// No-op when unsliced — the dabs apply at the next slice.
    fn commit_paint_stroke(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        if self.sliced.is_some() && !self.bead_pristine.is_empty() {
            self.scene.set_toolpaths(&rs.device, &rs.queue, &self.bead_inst);
            self.content_version += 1;
            self.refresh_bead_summary();
        }
    }

    /// The bead dabs in the sliced LayerPlan frame (the render frame minus the
    /// bed origin that `build_instances` bakes into the bead x), ready for the
    /// engine's `apply_bead_dabs`.
    fn engine_dabs(&self) -> Vec<engine::BeadDab> {
        let ox = self.sliced_origin_x;
        self.bead_dabs
            .iter()
            .map(|d| engine::BeadDab {
                center: [d.center.x as f64 - ox, d.center.y as f64, d.center.z as f64],
                radius: d.radius as f64,
                tool: d.tool,
                tris: d
                    .tris
                    .iter()
                    .map(|t| [[t[0][0] - ox, t[0][1], t[0][2]], [t[1][0] - ox, t[1][1], t[1][2]], [t[2][0] - ox, t[2][1], t[2][2]]])
                    .collect(),
            })
            .collect()
    }

    /// Recompute the paint-affected summary fields (toolchanges, per-tool
    /// filament, path count) from the bead-dab-applied layers — so the readout
    /// matches what will export. Called on stroke end, not per frame.
    fn refresh_bead_summary(&mut self) {
        let dabs = self.engine_dabs();
        let Some(base) = self.sliced.as_ref() else { return };
        let mut layers = base.clone();
        engine::apply_bead_dabs(&mut layers, &dabs, &self.settings);
        let mut toolchanges = 0usize;
        let mut last: Option<u32> = None;
        for path in layers.iter().flat_map(|l| &l.paths).filter(|p| p.points.len() >= 2) {
            if last.is_some_and(|t| t != path.tool) {
                toolchanges += 1;
            }
            last = Some(path.tool);
        }
        let per_tool = engine::estimate_filament_per_tool(&layers, &self.settings);
        let toolpaths = layers.iter().map(|l| l.paths.len()).sum();
        if let Some(s) = &mut self.slice_summary {
            s.toolchanges = toolchanges;
            s.per_tool = per_tool;
            s.toolpaths = toolpaths;
        }
    }

    /// Rebuild the working bead buffer from pristine + the current dab list and
    /// re-upload — after clearing/undoing dabs (reuses the cached pristine+grid).
    fn reapply_bead_dabs(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        if self.bead_pristine.is_empty() {
            return;
        }
        self.replay_bead_dabs();
        self.scene.set_toolpaths(&rs.device, &rs.queue, &self.bead_inst);
        self.content_version += 1;
    }

    /// The active metric mapped to per-path colors — or None in feature mode
    /// (`build_instances` then colors by path kind). Layer time is one ramp
    /// color per layer broadcast to its paths; filament is each path's
    /// printing tool wearing its slot's color.
    fn layer_color_table(&self) -> Option<Vec<Vec<[f32; 3]>>> {
        let layers = self.sliced.as_ref()?;
        build_color_table(
            layers,
            self.color_by,
            &self.settings,
            accent_hsl(self.accent),
            &self.layer_stats,
        )
    }

    /// The per-tool spool colors as a fixed 16-slot palette for the bead
    /// shader's Filament mode, indexed by a bead's tool id. Matches exactly what
    /// `layer_color_table`'s Filament branch bakes (`visible_against_backdrop`
    /// of the slot color), so driving color from this uniform instead of the
    /// baked instance rgb is visually identical — and lets a spool-color change
    /// be a per-frame uniform write with no instance rebuild.
    fn tool_palette(&self) -> [[f32; 4]; render::TOOL_PALETTE_LEN] {
        let mut pal = [[0.0f32; 4]; render::TOOL_PALETTE_LEN];
        let n = self.settings.tool_count.min(render::TOOL_PALETTE_LEN);
        for (i, slot) in pal.iter_mut().enumerate().take(n) {
            let c = render::visible_against_backdrop(self.settings.tool(i).color_rgb);
            *slot = [c[0], c[1], c[2], 1.0];
        }
        pal
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
            // Toolchanger breakdown: each used slot's share, and how many
            // swaps the print costs.
            if self.settings.tool_count > 1 && !sum.per_tool.is_empty() {
                for &(t, mm, g) in &sum.per_tool {
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 5.0;
                        let (dot, _) =
                            ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                        ui.painter().circle_filled(
                            dot.center(),
                            5.0,
                            rgb32(self.settings.tool(t as usize).color_rgb),
                        );
                        ui.label(format!("T{t} {:.1} m ({g:.0} g)", mm / 1000.0)).on_hover_text(
                            format!(
                                "Filament this print draws from slot {t} ({}).",
                                self.settings.tool(t as usize).filament_name
                            ),
                        );
                    });
                }
                let swaps_label = if self.settings.single_heater() { "swaps" } else { "toolchanges" };
                ui.label(format!("{} {swaps_label}", sum.toolchanges)).on_hover_text(
                    "Tool swaps over the whole print — each costs the per-change time \
                     under Machine & motion.",
                );
                // Shared nozzle: call out the purge waste (already in the totals).
                if self.settings.purges() && self.settings.purge_volume_mm3 > 0.0 {
                    let purge_g = sum.toolchanges as f64 * self.settings.purge_volume_mm3 / 1000.0
                        * self.settings.filament_density_g_cm3;
                    ui.colored_label(
                        egui::Color32::from_rgb(0xC8, 0x8A, 0x4B),
                        format!("≈ {purge_g:.0} g purged"),
                    )
                    .on_hover_text(format!(
                        "Filament flushed at swaps ({} × {:.0} mm³) to clear the old color — \
                         already counted in the totals above. The firmware purges it to a \
                         bucket / dump; no wipe tower is printed.",
                        sum.toolchanges, self.settings.purge_volume_mm3,
                    ));
                }
                // Painted blends whose repeat outgrew the band (a layer-height
                // change after picking): the print will stripe — say so here,
                // where the person deciding to Send is looking.
                let mut warned: Vec<usize> = Vec::new();
                for part in self.objects.iter().flat_map(|o| &o.parts) {
                    if let PartColor::Blend(b) = part.paint {
                        if !warned.contains(&b) && b < self.blends.len() {
                            if let Some(mm) = self.blend_banding_mm(&self.blends[b]) {
                                ui.colored_label(
                                    egui::Color32::from_rgb(0xC8, 0x8A, 0x4B),
                                    format!(
                                        "\"{}\" repeats every {mm:.1} mm — bands will show \
                                         (blend band {:.1} mm)",
                                        elide(&self.blends[b].name, 18),
                                        self.settings.blend_band_mm,
                                    ),
                                );
                                warned.push(b);
                            }
                        }
                    }
                }
            }
        }
    }

    fn export(&mut self) {
        self.refresh_tool0();
        let Some(mut layers) = self.sliced.clone() else { return };
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
        // Bake the bead-paint strokes into the toolpaths (no-op when unpainted).
        engine::apply_bead_dabs(&mut layers, &self.engine_dabs(), &self.settings);
        let gcode = engine::to_gcode(&layers, &self.settings);
        self.status = match std::fs::write(&path, gcode) {
            Ok(()) => format!("Wrote {}", path.display()),
            Err(e) => format!("Write failed: {e}"),
        };
    }

    /// The active machine's loadout as the live fields describe it: which
    /// filament each slot holds, the spool-color overrides (a color entry
    /// only survives while it still names its slot's loaded filament), the
    /// blend palette, and the process in use.
    fn current_loadout(&self) -> config::Loadout {
        config::Loadout {
            tools: self.tools.clone(),
            tool_colors: (0..self.tools.len())
                .map(|i| {
                    let slot = if i == 0 { &self.filament } else { &self.tools[i] };
                    match self.tool_colors.get(i) {
                        Some(Some((name, c))) if name == slot => config::hex_color(*c),
                        _ => String::new(),
                    }
                })
                .collect(),
            blends: self.blends.clone(),
            process: self.process.clone(),
        }
    }

    /// Write the program state to the dotfile folder when it changed —
    /// convenience memory only, so a failed save never blocks anything.
    /// The flat fields stay the active machine's mirror (the legacy read
    /// path); the per-printer truth is the loadout map.
    fn persist_state(&mut self) {
        let lo = self.current_loadout();
        let mut loadouts = self.loadouts.clone();
        loadouts.insert(self.printer.clone(), lo.clone());
        let cur = config::AppState {
            printer: self.printer.clone(),
            filament: self.filament.clone(),
            process: lo.process,
            tools: lo.tools,
            tool_colors: lo.tool_colors,
            blends: lo.blends,
            loadouts,
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

    /// Upload filename: the active bed's first object name with a .gcode
    /// extension (uploads carry the last slice, which is per-bed).
    fn upload_filename(&self) -> String {
        let base = self
            .objects
            .iter()
            .find(|o| self.bed_of(o) == self.active_bed)
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

    /// Geometry for the selection spotlight: a translucent gradient band on the
    /// bed (z=0) that hugs the selected object's footprint — brightest at the
    /// silhouette, fading out into the bed. Returned as `[x,y,z,r,g,b,a]` verts
    /// (a triangle soup). Empty when nothing is selected. Uses the convex hull
    /// of the object's world-projected vertices: a clean single loop, cheap
    /// enough to rebuild on every drag/rotate frame (same order as the mesh
    /// upload that already walks these vertices).
    fn selection_spotlight(&self) -> Vec<[f32; 7]> {
        let Some(i) = self.selected else { return Vec::new() };
        let Some(obj) = self.objects.get(i) else { return Vec::new() };
        let t = obj.transform();
        let mut pts: Vec<[f64; 2]> = Vec::new();
        for v in obj.parts.iter().flat_map(|p| &p.mesh.vertices) {
            let w = t.apply(*v);
            pts.push([w[0], w[1]]);
        }
        let hull = convex_hull_2d(pts);
        let n = hull.len();
        if n < 3 {
            return Vec::new();
        }

        // Outward vertex normals: the average of the two adjacent edges'
        // outward normals. In a CCW loop an edge a→b's outward normal is
        // (dy, -dx). Averaging gives a miter direction with no corner spikes.
        let edge_n = |a: [f64; 2], b: [f64; 2]| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = (d[0] * d[0] + d[1] * d[1]).sqrt().max(1e-9);
            [d[1] / l, -d[0] / l]
        };
        let mut nrm = vec![[0.0f64; 2]; n];
        for k in 0..n {
            let n0 = edge_n(hull[(k + n - 1) % n], hull[k]);
            let n1 = edge_n(hull[k], hull[(k + 1) % n]);
            let s = [n0[0] + n1[0], n0[1] + n1[1]];
            let l = (s[0] * s[0] + s[1] * s[1]).sqrt();
            nrm[k] = if l < 1e-6 { n1 } else { [s[0] / l, s[1] / l] };
        }

        // Pool width scales with the footprint, clamped to a sane range.
        let (mut mnx, mut mny, mut mxx, mut mxy) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in &hull {
            mnx = mnx.min(p[0]);
            mny = mny.min(p[1]);
            mxx = mxx.max(p[0]);
            mxy = mxy.max(p[1]);
        }
        let diag = ((mxx - mnx).powi(2) + (mxy - mny).powi(2)).sqrt();
        let width = (diag * 0.18).clamp(12.0, 45.0);

        // Warm accent glow (the same "accent proper" the old highlight used).
        let (_, col) = mesh_tints(self.accent);
        // Concentric rings offset outward from the silhouette; alpha → 0.
        let rings: [(f64, f32); 3] = [(0.0, 0.55), (width * 0.4, 0.20), (width, 0.0)];
        let ring_at = |r: f64| -> Vec<[f64; 2]> {
            (0..n).map(|k| [hull[k][0] + nrm[k][0] * r, hull[k][1] + nrm[k][1] * r]).collect()
        };
        let ring_pts: Vec<Vec<[f64; 2]>> = rings.iter().map(|&(r, _)| ring_at(r)).collect();

        let vert = |p: [f64; 2], a: f32| -> [f32; 7] {
            [p[0] as f32, p[1] as f32, 0.0, col[0], col[1], col[2], a]
        };
        let mut out: Vec<[f32; 7]> = Vec::new();
        for b in 0..rings.len() - 1 {
            let (inner, ai) = (&ring_pts[b], rings[b].1);
            let (outer, ao) = (&ring_pts[b + 1], rings[b + 1].1);
            for k in 0..n {
                let k2 = (k + 1) % n;
                let i0 = vert(inner[k], ai);
                let i1 = vert(inner[k2], ai);
                let o0 = vert(outer[k], ao);
                let o1 = vert(outer[k2], ao);
                out.extend_from_slice(&[i0, i1, o1, i0, o1, o0]);
            }
        }
        out
    }

    fn rebuild_scene(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let mut changed = false;
        let bx = self.settings.bed_size_x_mm as f32;
        let by = self.settings.bed_size_y_mm as f32;

        if self.beds_dirty {
            self.scene.set_beds(&rs.device, &rs.queue, bx, by, self.n_beds(), BED_GAP_MM as f32, self.active_bed);
            self.beds_dirty = false;
            changed = true;
        }

        if self.spotlight_dirty {
            // Selection cue: a warm spotlight pool tracing the selected object's
            // footprint on the bed (in place of a color tint on the model).
            let spotlight = self.selection_spotlight();
            self.scene.set_spotlight(&rs.device, &rs.queue, &spotlight);
            self.spotlight_dirty = false;
            changed = true;
        }

        // The mesh (and its bounds) only matter when the model view is showing
        // them. In Preview the model is hidden, so a color/geometry change is
        // DEFERRED — the flags stay set and the expensive vertex re-bake waits
        // until the user returns to model view. This is what makes a spool-color
        // change while judging the sliced preview free: the beads recolor from
        // the tool palette (a uniform, tracked by RenderSig) and the hidden mesh
        // isn't touched at all.
        let show_mesh = !(self.view_preview && self.sliced.is_some());
        if (self.mesh_struct_dirty
            || self.mesh_xform_dirty
            || self.mesh_color_dirty
            || self.mesh_paint_dirty)
            && show_mesh
        {
            // A structure or placement change refreshes the world-bounds cache
            // (positions feed both the invalid check and the model matrices); a
            // color-only change reuses it.
            if self.mesh_struct_dirty || self.mesh_xform_dirty {
                let bx_cache = self.settings.bed_size_x_mm;
                self.obj_bounds = self
                    .objects
                    .iter()
                    .map(|o| {
                        let (minx, miny, maxx, maxy, _) = o.footprint();
                        let (hw, hh) = ((maxx - minx) / 2.0, (maxy - miny) / 2.0);
                        ObjBounds {
                            aabb: [o.pos[0] - hw, o.pos[1] - hh, o.pos[0] + hw, o.pos[1] + hh],
                            height: o.height(),
                            bed: bed_of_pos(o.pos[0], bx_cache),
                        }
                    })
                    .collect();
            }
            // Build the per-part lists (same object→part order for geometry and
            // uniforms). Unprintable objects (off the bed or overlapping, any
            // bed) get the warning tint via the `invalid` flag; on a toolchanger
            // each part wears its paint's display color, single-tool keeps the
            // accent porcelain. Selection is shown by the bed spotlight, not a
            // mesh tint.
            let blocked: Vec<bool> = (0..self.objects.len()).map(|i| self.obj_problem(i).is_some()).collect();
            let multi = self.settings.tool_count > 1;
            let (unsel_tint, _) = mesh_tints(self.accent);
            let mut meshes: Vec<&mesh::Mesh> = Vec::new();
            let mut part_data: Vec<([[f32; 4]; 4], [f32; 3], bool)> = Vec::new();
            for (i, o) in self.objects.iter().enumerate() {
                let model = transform_to_mat4(&o.transform());
                for part in &o.parts {
                    let rgb = if multi {
                        render::visible_against_backdrop(self.paint_display_rgb(part.paint))
                    } else {
                        unsel_tint
                    };
                    meshes.push(part.display.as_ref());
                    part_data.push((model, rgb, blocked[i]));
                }
            }
            // Re-upload geometry only when the part SET changed. The count guard
            // is a safety net: a missed struct flag would otherwise desync the
            // draw ranges from the uniforms.
            if self.mesh_struct_dirty || part_data.len() != self.scene.mesh_part_count() {
                self.scene.upload_mesh_geometry(&rs.device, &rs.queue, &meshes);
            }
            // Placement + tint: the only thing a drag or recolor rewrites.
            self.scene.upload_mesh_parts(&rs.device, &rs.queue, &part_data);

            // Per-vertex surface paint. A geometry re-upload zeroed the paint
            // buffer, so restore it whenever the struct changed, or when a brush
            // stroke marked it dirty.
            if self.mesh_struct_dirty || self.mesh_paint_dirty {
                let paint = self.build_mesh_paint();
                self.scene.upload_mesh_paint(&rs.device, &rs.queue, &paint);
            }

            // Only re-frame on scene changes (import/duplicate/delete/arrange/
            // profile), not when the user merely selects an object. Objects rest
            // on z=0, so the world box is the footprints' XY span × [0, height].
            if self.refit_camera {
                if self.obj_bounds.is_empty() {
                    let c = self.bed_center(self.active_bed);
                    self.camera.frame(glam::Vec3::new(c[0] as f32, c[1] as f32, 0.0), bx.max(by) * 0.5);
                } else {
                    let (mut lo, mut hi) = ([f32::MAX; 2], [f32::MIN; 2]);
                    let mut top = 0.0f32;
                    for b in &self.obj_bounds {
                        lo[0] = lo[0].min(b.aabb[0] as f32);
                        lo[1] = lo[1].min(b.aabb[1] as f32);
                        hi[0] = hi[0].max(b.aabb[2] as f32);
                        hi[1] = hi[1].max(b.aabb[3] as f32);
                        top = top.max(b.height as f32);
                    }
                    let span = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(top);
                    self.camera.frame(
                        glam::Vec3::new((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, top / 2.0),
                        span * 0.5 + 1.0,
                    );
                }
                self.refit_camera = false;
            }
            self.mesh_struct_dirty = false;
            self.mesh_xform_dirty = false;
            self.mesh_color_dirty = false;
            self.mesh_paint_dirty = false;
            changed = true;
        }

        if changed {
            self.content_version += 1;
        }
    }
}

/// A snapshot of everything the 3D scene render depends on. The scene is
/// re-rendered only when this changes — egui repaints on every input, and a
/// large slice shouldn't redraw all its beads just because the pointer moved.
#[derive(PartialEq)]
struct RenderSig {
    vp: glam::Mat4,
    show_mesh: bool,
    show_stars: bool,
    /// (count, joint_count, current_layer bits, dim bits, mask), or None in model mode.
    preview: Option<(u32, u32, u32, u32, u32)>,
    accent: egui::Color32,
    size: (u32, u32),
    content: u64,
    /// Bead color mode + a hash of the tool palette, so a Filament-mode
    /// spool-color change re-renders (it updates only a uniform, bumping no
    /// content_version — the mesh re-bake it would otherwise trigger is deferred
    /// while the model is hidden).
    preview_color: (u32, u64),
}

/// A cheap order-sensitive hash of the tool palette's bit patterns — only used
/// to detect a spool-color change for the render gate.
fn palette_hash(pal: &[[f32; 4]; render::TOOL_PALETTE_LEN]) -> u64 {
    let mut h = 0xcbf29ce484222325u64; // FNV-1a offset basis
    for slot in pal {
        for &c in slot {
            h = (h ^ c.to_bits() as u64).wrapping_mul(0x100000001b3);
        }
    }
    h
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let rs = frame.wgpu_render_state().expect("wgpu render state").clone();

        // A background slice finished? Commit + upload it. While one is in
        // flight, keep repainting so we notice the frame it lands (the worker
        // can't wake egui itself). A worker that died mid-slice (a panic in the
        // engine) just drops the job — the app stays alive, unlike the old
        // synchronous slice.
        if let Some(job) = self.slice_job.take() {
            match job.rx.try_recv() {
                Ok(out) => {
                    let (sig, resliced, origin_x) = (job.sig, job.resliced, job.origin_x);
                    self.apply_slice_output(out, sig, &rs, resliced, origin_x);
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    self.slice_job = Some(job);
                    ui.ctx().request_repaint();
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.status = "Slice failed — see the console for details.".into();
                }
            }
        }

        // A bed-size change — slider edit, printer switch, profile delete
        // fallback — refreshes the bed mesh and re-pivots the view on the new
        // plate, whatever path it arrived by.
        let bed = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
        if bed != self.last_bed {
            let old_bx = self.last_bed.0;
            self.last_bed = bed;
            self.mark_scene_dirty();
            self.recenter_camera = true;
            // The bed pitch changed: re-pin every object to the same bed
            // index + relative offset under the new layout, so membership
            // survives resizes.
            for o in &mut self.objects {
                let k = bed_of_pos(o.pos[0], old_bx);
                let local = o.pos[0] - bed_origin_x(k, old_bx);
                o.pos[0] = bed_origin_x(k, bed.0) + local;
            }
            // Objects placed for the old plate may not fit the new one —
            // re-grid only when something actually hangs off: manual
            // placements that still fit are left alone. (Without this, a
            // 350→152 mm switch leaves the model far outside the plate and
            // the freshly recentered pivot FEELS broken — you orbit an empty
            // bed while the part sweeps in the distance.)
            if self.objects.iter().any(|o| self.off_its_bed(o)) {
                self.arrange();
                self.sliced = None;
                self.slice_summary = None;
                self.view_preview = false;
                self.refit_camera = true;
            }
        }
        if self.scene_dirty_any() {
            self.rebuild_scene(&rs);
        }
        // After any refit so the plate-center pivot wins; distance and angles
        // are untouched (it's a pivot move, not a re-frame).
        if self.recenter_camera {
            self.recenter_camera = false;
            let c = self.bed_center(self.active_bed);
            let to = glam::Vec3::new(c[0] as f32, c[1] as f32, 0.0);
            // Pure translation by default: keep the current zoom (orientation
            // is never touched). Only when the distance is outside the band
            // that frames a bed legibly do we ease it — to the nearest edge,
            // the least change. `ideal` is the distance Camera::frame picks
            // for one bed.
            let ideal = ((bed.0.max(bed.1) as f32) * 1.25).max(20.0);
            let dist_to = self.camera.distance.clamp(ideal * FLY_ZOOM_MIN, ideal * FLY_ZOOM_MAX);
            // Travel, don't teleport (trivial moves snap — no point
            // animating a recenter onto itself).
            if (to - self.camera.target).length() > 1.0
                || (dist_to - self.camera.distance).abs() > 1.0
            {
                self.camera_glide = Some(CameraGlide {
                    from: self.camera.target,
                    to,
                    dist_from: self.camera.distance,
                    dist_to,
                    started: std::time::Instant::now(),
                });
            } else {
                self.camera.target = to;
            }
        }
        if let Some(g) = &self.camera_glide {
            let t = (g.started.elapsed().as_secs_f32() / GLIDE_SECS).min(1.0);
            let p = 1.0 - (1.0 - t).powi(3); // ease-out cubic
            self.camera.target = g.from.lerp(g.to, p);
            self.camera.distance = g.dist_from + (g.dist_to - g.dist_from) * p;
            if t >= 1.0 {
                self.camera_glide = None;
            } else {
                // egui repaints on input only — keep frames coming mid-glide.
                ui.ctx().request_repaint();
            }
        }
        // Unpinned auto settings track their masters every frame, before
        // anything (incl. the Slice button) reads them.
        self.apply_auto();

        // 320 wide fits the longest slider row (90 slider + value + 19-char
        // A printer-host operation reports back; quiet status polls feed the
        // live-print overlay, everything else lands in the status line.
        let mut op_done = false;
        if let Some(rx) = &self.host_rx {
            while let Ok(reply) = rx.try_recv() {
                match reply {
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
                }
            }
        }
        if op_done {
            self.host_rx = None;
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

        // Multi-tool edits requested inside the panel closure (which borrows
        // settings mutably); handled after it returns.
        let mut tool_count_changed = false;
        let mut filament_color_changed = false;
        // Parts on the active bed — the engine ignores spiral vase for
        // multi-part plates, and the vase toggle mirrors that lockout.
        let active_bed_parts: usize = self
            .objects
            .iter()
            .filter(|o| self.bed_of(o) == self.active_bed)
            .map(|o| o.parts.len())
            .sum();

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
            // Hard cap at the panel's inner width: an overflowing row would
            // not just clip — egui reserves the overflow, pushing the central
            // panel right and opening an unpainted band between the two
            // (egui #4475). With the cap, overflow clips and the seam holds.
            ui.set_max_width(296.0);
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
            // Import is the panel's one object action — selection, duplicate
            // and delete live on the viewport's floating cards, and the bed
            // controls float over the 3D view.
            if ui
                .add(egui::Button::new("Import…").min_size(egui::vec2(ui.available_width(), 26.0)))
                .on_hover_text(
                    "Load an STL or 3MF onto a fresh bed (a 3MF build's objects each \
                     arrive separately, keeping their plate layout when it fits).",
                )
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

            ui.separator();

            let printers: Vec<String> = self.profiles.printer_names().iter().map(|s| s.to_string()).collect();
            let processes: Vec<String> = self.profiles.process_names().iter().map(|s| s.to_string()).collect();
            let dirty = self.tier_dirty_masked();
            let mut changed = false;
            let mut open_dialog: Option<ProfileDialog> = None;
            let prev_sel = (self.printer.clone(), self.filament.clone(), self.process.clone());
            {
                // No Filament tier row at ANY tool count: the filament is
                // "slot 0 with privileges" and every slot — the single tool
                // included — picks and edits its filament on the Filament
                // card's slot row, so the panel reads the same on one tool
                // as on eight.
                let rows: [(TierKind, &mut String, &[String], bool, &str); 2] = [
                    (TierKind::Printer, &mut self.printer, &printers, dirty[0],
                        "Machine profile — bed size, nozzle, motion limits, and start/end g-code."),
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
                            open_dialog = Some(ProfileDialog { kind, name, delete: false, slot: None });
                        }
                        if is_user
                            && ui
                                .small_button("🗑")
                                .on_hover_text("Delete this user profile from disk.")
                                .clicked()
                        {
                            open_dialog =
                                Some(ProfileDialog { kind, name: sel.clone(), delete: true, slot: None });
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
                    // top of `ui` only catches actual changes) — and its own
                    // loadout: spools stay with their machine.
                    if self.printer != prev_sel.0 {
                        self.recenter_camera = true;
                        self.switch_printer_loadout(&prev_sel.0);
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
                // ink text — the one inverted block in the panel. It slices
                // the ACTIVE bed.
                if let Some(job) = self.slice_job.as_ref() {
                    // While a background slice runs, the Slice button doubles as
                    // a progress bar: a cream fill sweeps left→right as the wall
                    // pass advances, "Slicing… NN%" in ink over it. Held just shy
                    // of full (there's a brief stamp/upload tail after the wall
                    // pass) so it never sticks at 100% — it flips to the finished
                    // button the frame the result lands.
                    let frac = job.progress.fraction();
                    let shown = frac.min(0.99);
                    let (rect, _) = ui.allocate_exact_size(big, egui::Sense::hover());
                    let radius = egui::CornerRadius::same(3);
                    let painter = ui.painter();
                    painter.rect_filled(rect, radius, palette::CREAM_DIM);
                    let mut fill = rect;
                    fill.set_width(rect.width() * shown);
                    painter.rect_filled(fill, radius, palette::CREAM);
                    painter.text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        format!("Slicing… {:.0}%", (frac * 100.0).min(99.0)),
                        egui::FontId::proportional(15.0),
                        palette::INK,
                    );
                    ui.ctx().request_repaint();
                } else {
                    let can_slice = self.objects.iter().any(|o| self.bed_of(o) == self.active_bed);
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
                        .on_hover_text("Slice the active bed's objects into toolpaths using the current settings.")
                        .on_disabled_hover_text("Nothing on the active bed — import a model, or step beds with ◀ ▶.")
                        .clicked()
                    {
                        self.slice(&rs);
                    }
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
            // An object off the active bed's build volume or overlapping
            // another can't be printed — block both send buttons and name it.
            let violations = self.active_bed_violations();
            let bed_printable = violations.is_empty();
            let send_disabled_hover = if bed_printable {
                "Needs a sliced model and a printer host (Connection section).".to_string()
            } else {
                format!(
                    "Not printable: {} (off the bed or overlapping another object) — fix before printing.",
                    violations.join(", ")
                )
            };
            ui.horizontal(|ui| {
                let can_send = self.sliced.is_some() && host_set && !host_busy && bed_printable;
                let send_btn = egui::Button::new(egui::RichText::new("Send").size(15.0)).min_size(big);
                if ui
                    .add_enabled(can_send, send_btn)
                    .on_hover_text("Upload the g-code to the printer's storage (host under Connection).")
                    .on_disabled_hover_text(send_disabled_hover.as_str())
                    .clicked()
                {
                    host_op = Some(HostOp::Send { start: false });
                }
                let print_btn = egui::Button::new(egui::RichText::new("▶ Send & print").size(15.0)).min_size(big);
                if ui
                    .add_enabled(can_send, print_btn)
                    .on_hover_text("Upload the g-code and start printing it immediately.")
                    .on_disabled_hover_text(send_disabled_hover.as_str())
                    .clicked()
                {
                    host_op = Some(HostOp::Send { start: true });
                }
            });
            if !bed_printable {
                ui.label(
                    egui::RichText::new(format!(
                        "{} — not printable (off bed or overlapping)",
                        violations.join(", ")
                    ))
                    .color(palette::ERROR),
                )
                .on_hover_text(
                    "Each highlighted object hangs past the bed (in X/Y, or taller than the \
                     build height) or overlaps another object. Move or rescale it — it's \
                     tinted in the view.",
                );
            }
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
                ui.add_space(8.0);
                ui.checkbox(&mut self.show_stars, "stars").on_hover_text(
                    "Plot the real night sky — the ~9,000 naked-eye stars of the Yale \
                     Bright Star Catalogue — on the viewport backdrop. It's a celestial \
                     sphere, so it rotates as you orbit. Off for a plain background.",
                );
            });
            if self.accent_rebake && !ui.ctx().input(|i| i.pointer.any_down()) {
                self.accent_rebake = false;
                self.set_preview_instances(&rs);
                // The model tint bakes into the mesh vertices now (per-part
                // colors) — re-upload them on the same release.
                self.mark_mesh_color_dirty();
                ui.ctx().request_repaint();
            }

            // Surface paint brush: paint tool-color regions on the MODEL (mesh);
            // the strokes are recorded as sub-bead dabs that resolve at bead
            // resolution when sliced (Preview shows the result, read-only). Only
            // meaningful on a multi-tool machine, in Model view.
            if self.settings.tool_count > 1 && !self.view_preview {
                ui.horizontal(|ui| {
                    let tip = "Paint tool-color regions on the model. Click a spot to smart-fill \
                         (crease-bounded); drag to freehand brush; drag empty space orbits. \
                         Boundaries resolve at bead resolution when you slice.";
                    if ui.add(egui::Button::selectable(self.paint_mode, "🖌 paint")).on_hover_text(tip).clicked() {
                        self.paint_mode = !self.paint_mode;
                        // Entering/leaving paint mode toggles bead subdivision, so
                        // rebuild the sliced beads (kept ready for a Preview switch).
                        if self.sliced.is_some() {
                            self.set_preview_instances(&rs);
                        }
                    }
                    if self.paint_mode {
                        for t in 0..self.settings.tool_count as u32 {
                            if ui
                                .selectable_label(!self.brush_erase && self.brush_tool == t, format!("T{t}"))
                                .clicked()
                            {
                                self.brush_tool = t;
                                self.brush_erase = false;
                            }
                        }
                        if ui.selectable_label(self.brush_erase, "erase").clicked() {
                            self.brush_erase = true;
                        }
                        if ui.button("clear").on_hover_text("Remove all paint.").clicked() {
                            self.clear_paint();
                            self.bead_dabs.clear();
                            self.reapply_bead_dabs(&rs);
                        }
                    }
                });
                if self.paint_mode {
                    // drift/reach tune the smart-fill CLICK; brush = freehand drag width.
                    ui.horizontal(|ui| {
                        ui.add(egui::Slider::new(&mut self.brush_drift_deg, 5.0..=120.0).text("drift°"))
                            .on_hover_text(
                                "How far a smart-fill may drift from the clicked surface's angle \
                                 before stopping. Lower = tighter; higher = grabs more curve.",
                            );
                        ui.add(egui::Slider::new(&mut self.brush_radius_mm, 2.0..=120.0).text("reach"))
                            .on_hover_text("Max distance a smart-fill spreads from the click (mm).");
                    });
                    ui.horizontal(|ui| {
                        ui.add(egui::Slider::new(&mut self.brush_dab_mm, 1.0..=30.0).text("brush"))
                            .on_hover_text("Brush size — the WIDTH (mm) of the painted spot, for the freehand drag.");
                    });
                }
            }
            if self.view_preview && n_layers > 0 {
                // The layer slider itself lives on the right edge of the 3D pane
                // (vertical); here in the panel are just the feature toggles.
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(&mut self.show_walls, "walls").on_hover_text("Show wall (perimeter) toolpaths.");
                    ui.checkbox(&mut self.show_solid, "solid").on_hover_text("Show buried solid infill (solid layers covered above and below).");
                    ui.checkbox(&mut self.show_surface, "surface").on_hover_text("Show top and bottom surface skins (the visible faces).");
                    ui.checkbox(&mut self.show_infill, "infill").on_hover_text("Show sparse interior infill.");
                    ui.checkbox(&mut self.show_ironing, "ironing").on_hover_text("Show the top-surface ironing pass.");
                    ui.checkbox(&mut self.show_skirt, "skirt").on_hover_text("Show skirt and brim.");
                    ui.checkbox(&mut self.show_support, "support").on_hover_text("Show support, bridge, and arc-overhang toolpaths.");
                    ui.checkbox(&mut self.show_travel, "travel").on_hover_text("Show non-printing travel moves.");
                    ui.checkbox(&mut self.show_seams, "seams").on_hover_text("Highlight where each wall loop starts (the seam).");
                });
                ui.horizontal(|ui| {
                    ui.label("color").on_hover_text(
                        "What the preview colors encode. Feature type is the classic view; layer time \
                         shows where the nozzle returns quickly (little cooling time), scored per layer; \
                         filament (multi-tool) paints each path in its tool's spool color. \
                         All reflect the last slice — re-slice after changing toggles.",
                    );
                    let before = self.color_by;
                    egui::ComboBox::from_id_salt("preview_color_by")
                        .selected_text(match self.color_by {
                            ColorBy::Feature => "feature type",
                            ColorBy::LayerTime => "layer time",
                            ColorBy::Filament => "filament",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.color_by, ColorBy::Feature, "feature type");
                            ui.selectable_value(&mut self.color_by, ColorBy::LayerTime, "layer time");
                            ui.add_enabled_ui(self.settings.tool_count > 1, |ui| {
                                ui.selectable_value(&mut self.color_by, ColorBy::Filament, "filament")
                                    .on_disabled_hover_text(
                                        "Filament coloring needs a multi-tool machine (2+ tools).",
                                    );
                            });
                        });
                    if self.color_by != before {
                        self.set_preview_instances(&rs);
                    }
                });
                if self.color_by == ColorBy::LayerTime && !self.layer_stats.is_empty() {
                    let vals: Vec<f64> = self.layer_stats.iter().map(|st| st.secs).collect();
                    let lo = vals.iter().cloned().fold(f64::INFINITY, f64::min);
                    let hi = vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    // Legend ends carry the real numbers: left = bright (quick), right = dark (slow).
                    let left = format!("{hi:.1}s");
                    let right = format!("{lo:.1}s");
                    let expl = "Per-layer print time, log scale. Brightest = the quickest layers: the \
                                plastic below gets the least time to cool before the nozzle returns. The \
                                min-layer slowdown under Feature speeds is the usual fix.";
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(left).small()).on_hover_text(expl);
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
                        }
                        ui.label(egui::RichText::new(right).small()).on_hover_text(expl);
                    });
                }
            }
            ui.separator();

            // Settings, grouped into collapsible categories (Orca-style) and scrolled.
            // Per-slot dirty flags for the toolchanger tabs (computed before
            // the sections borrow settings mutably) + the dialog they open.
            // Per-slot dirty for the tab dots; single-tool edits live on
            // the FLAT fields (tools[0] is only their mirror), so the one
            // dot reads the flat filament diff instead.
            let tool_dirty_flags: Vec<bool> = if self.settings.tool_count > 1 {
                (0..self.settings.tools.len()).map(|i| self.tool_dirty(i)).collect()
            } else {
                vec![!FilamentProfile::diff(&self.settings, &self.baseline).is_empty()]
            };
            let mut tool_dialog: Option<ProfileDialog> = None;
            // Slot actions picked inside the settings closures (which hold
            // `s = &mut self.settings`) — applied after the scroll region,
            // where self is whole again.
            let mut slot_pick: Option<(usize, String)> = None;
            let mut color_pick: Option<(usize, [f32; 3])> = None;
            // What the PROFILE pins on the auto-capable fields — the auto
            // sliders' revert target (live `pins` diverge as the user drags).
            let profile_pins = self.profile_pins();
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
                    revert_row(ui, &mut s.layer_height_mm, &self.baseline.layer_height_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.05..=0.4), "layer mm",
                            lh_hint);
                    });
                    revert_row(ui, &mut s.first_layer_height_mm, &self.baseline.first_layer_height_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.1..=0.4), "first layer mm",
                            "Thickness of the first layer — often thicker for bed adhesion.");
                    });
                    auto_slider(ui, &mut s.line_width_mm, 0.1..=1.5, "line width mm",
                        &mut pins.line_width, config::derived_line_width_mm(s.nozzle_diameter_mm),
                        profile_pins.line_width, self.baseline.line_width_mm,
                        "Bead (extrusion) width. Auto = nozzle × 1.125 (0.45 for a 0.4 nozzle); override to tune wall strength / detail. ⟲ returns to auto.");
                    revert_row(ui, &mut s.seam_mode, &self.baseline.seam_mode, |ui, v| {
                        seam_combo(ui, v)
                            .on_hover_text("Where each wall loop starts: nearest point, sharpest corner, or random.");
                    });
                    revert_row(ui, &mut s.elephant_foot_mm, &self.baseline.elephant_foot_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.0..=0.5), "elephant foot mm",
                            "Shrink the first layer's outline inward to counter first-layer squish. 0 = off.");
                    });
                    if s.tool_count > 1 {
                        revert_row(ui, &mut s.blend_band_mm, &self.baseline.blend_band_mm, |ui, v| {
                            hslider(ui, true, egui::Slider::new(v, 0.2..=3.0), "blend band mm",
                                "Tallest dither repeat a blend may have and still read as one color \
                                 at viewing distance. Sets the blend picker's palette: mixes quantize \
                                 to whole layers of a band ÷ layer-height cycle, so a ratio that \
                                 would stripe visibly (one layer in ten = a 2 mm band) simply isn't \
                                 offered. Tighten it for saturated colors; close greys fuse at \
                                 longer repeats.");
                        });
                    }
                    revert_row(ui, &mut s.xy_compensation_mm, &self.baseline.xy_compensation_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, -0.5..=0.5), "XY comp mm",
                            "Grow (+) or shrink (−) every layer's outline for dimensional accuracy. 0 = off.");
                    });
                    let vase = s.spiral_vase;
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.ironing, "ironing"))
                        .on_hover_text("Re-traverse top surfaces with a hot nozzle and a trickle of flow to melt them smooth.")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.fuzzy_skin, "fuzzy skin"))
                        .on_hover_text("Jitter the outer wall into a rough, textured surface (hides layer lines).")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    revert_row(ui, &mut s.fuzzy_skin_thickness_mm, &self.baseline.fuzzy_skin_thickness_mm, |ui, v| {
                        hslider(ui, s.fuzzy_skin && !vase, egui::Slider::new(v, 0.05..=1.0), "fuzzy thickness mm",
                            "Total jitter band, centered on the wall line.");
                    });
                    revert_row(ui, &mut s.fuzzy_skin_point_dist_mm, &self.baseline.fuzzy_skin_point_dist_mm, |ui, v| {
                        hslider(ui, s.fuzzy_skin && !vase, egui::Slider::new(v, 0.2..=2.0), "fuzzy point dist mm",
                            "Spacing between jittered points — smaller is noisier.");
                    });
                    // The engine ignores vase for multi-part plates — mirror it.
                    ui.add_enabled(active_bed_parts <= 1, egui::Checkbox::new(&mut s.spiral_vase, "spiral vase"))
                        .on_hover_text("One continuously rising outer wall above a solid bottom — no infill, no seams. Forces 1 wall / 0% infill / no supports (those controls gray out).")
                        .on_disabled_hover_text("The active bed holds multiple parts — spiral vase is one continuous wall, so the engine ignores it on multi-part plates.");
                });
                tier_section(ui, "Walls & top/bottom", TierKind::Process, false, |ui| {
                    let vase = s.spiral_vase;
                    revert_row(ui, &mut s.wall_count, &self.baseline.wall_count, |ui, v| {
                        hslider_lockout(ui, !vase, egui::Slider::new(v, 0..=99), "walls",
                            "Number of perimeter loops (shell wall thickness). 0 = infill only, no perimeters.",
                            "Spiral vase forces a single wall.");
                    });
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.outer_wall_first, "outer wall first"))
                        .on_hover_text("Print each island's outer wall before its inner walls — crisper overhang edges. Off (default): inner walls first, outer wall last, for the best flat-surface finish.")
                        .on_disabled_hover_text("Spiral vase prints a single wall.");
                    revert_row(ui, &mut s.top_layers, &self.baseline.top_layers, |ui, v| {
                        hslider_lockout(ui, !vase, egui::Slider::new(v, 0..=10), "top layers",
                            "Number of solid layers on top surfaces.",
                            "Spiral vase prints no top shells.");
                    });
                    revert_row(ui, &mut s.bottom_layers, &self.baseline.bottom_layers, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0..=10), "bottom layers",
                            "Number of solid layers on bottom surfaces.");
                    });
                    revert_row(ui, &mut s.monotonic_solid, &self.baseline.monotonic_solid, |ui, v| {
                        ui.checkbox(v, "monotonic top/bottom")
                            .on_hover_text("Print solid-fill lines in one strict sweep per surface for an even sheen.");
                    });
                });
                tier_section(ui, "Infill", TierKind::Process, false, |ui| {
                    let vase = s.spiral_vase;
                    revert_row(ui, &mut s.infill_density, &self.baseline.infill_density, |ui, v| {
                        hslider_lockout(ui, !vase, egui::Slider::new(v, 0.0..=1.0), "density",
                            "Sparse interior fill density (0 = hollow, 1 = solid).",
                            "Spiral vase prints no infill.");
                    });
                    ui.add_enabled_ui(s.infill_density > 0.0 && !vase, |ui| {
                        revert_row(ui, &mut s.sparse_pattern, &self.baseline.sparse_pattern, |ui, v| {
                            pattern_combo(ui, "sparse fill", v)
                                .on_hover_text("Pattern for the sparse interior infill.");
                        });
                    });
                    revert_row(ui, &mut s.top_pattern, &self.baseline.top_pattern, |ui, v| {
                        pattern_combo(ui, "top", v)
                            .on_hover_text("Pattern for the top skin (the visible top surface) layers.");
                    });
                    revert_row(ui, &mut s.bottom_pattern, &self.baseline.bottom_pattern, |ui, v| {
                        pattern_combo(ui, "bottom", v)
                            .on_hover_text("Pattern for the bottom skin (the visible bottom surface) layers.");
                    });
                    revert_row(ui, &mut s.solid_pattern, &self.baseline.solid_pattern, |ui, v| {
                        pattern_combo(ui, "solid fill", v)
                            .on_hover_text("Pattern for buried solid fill, between the sparse infill and the skins.");
                    });
                    revert_row(ui, &mut s.infill_overlap, &self.baseline.infill_overlap, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.0..=0.5), "wall overlap",
                            "How far infill pushes into the innermost wall (fraction of a line width) so they bond.");
                    });
                });
                tier_section(ui, "Support", TierKind::Process, true, |ui| {
                    let vase = s.spiral_vase;
                    ui.add_enabled_ui(!vase, |ui| {
                        revert_row(ui, &mut s.support_mode, &self.baseline.support_mode, |ui, v| {
                            support_combo(ui, v)
                                .on_hover_text("Overhang handling: none, grid supports, or self-supporting arcs.")
                                .on_disabled_hover_text("Forced off in spiral vase mode.");
                        });
                    });
                    let has_support = s.support_mode != config::SupportMode::None && !vase;
                    revert_row(ui, &mut s.support_overhang_angle_deg, &self.baseline.support_overhang_angle_deg, |ui, v| {
                        hslider(ui, has_support, egui::Slider::new(v, 0.0..=80.0), "overhang °",
                            "Steepest overhang (from vertical) printable without support. 45° ≈ one layer-width.");
                    });
                    revert_row(ui, &mut s.support_density, &self.baseline.support_density, |ui, v| {
                        hslider(ui, has_support, egui::Slider::new(v, 0.0..=1.0), "density",
                            "Infill density of grid supports.");
                    });
                    revert_row(ui, &mut s.support_xy_clearance_mm, &self.baseline.support_xy_clearance_mm, |ui, v| {
                        hslider(ui, has_support, egui::Slider::new(v, 0.0..=2.0), "xy gap mm",
                            "Horizontal gap between support and the model (for easy removal).");
                    });
                    revert_row(ui, &mut s.support_z_gap_layers, &self.baseline.support_z_gap_layers, |ui, v| {
                        hslider(ui, has_support, egui::Slider::new(v, 0..=5), "z-gap layers",
                            "Empty layers between a support top and the part it holds up.");
                    });
                    revert_row(ui, &mut s.support_interface_layers, &self.baseline.support_interface_layers, |ui, v| {
                        hslider(ui, has_support, egui::Slider::new(v, 0..=5), "interface",
                            "Dense solid layers at the support top for a smoother overhang underside.");
                    });
                    revert_row(ui, &mut s.max_bridge_span_mm, &self.baseline.max_bridge_span_mm, |ui, v| {
                        hslider(ui, !vase, egui::Slider::new(v, 0.0..=30.0), "bridge span mm",
                            "Widest gap (supported on \u{2265}2 sides) filled with straight anchored bridge lines; wider gaps fall back to the bottom shell.");
                    });
                    revert_row(ui, &mut s.bridge_foothold_mm, &self.baseline.bridge_foothold_mm, |ui, v| {
                        hslider(ui, !vase, egui::Slider::new(v, 0.0..=3.0), "bridge foothold mm",
                            "How far an enclosed-ceiling bridge sheet lands onto the supported rim. Bigger = more solid under the sheet's ends, but inner perimeters start further from the hollow. 0 = no foothold band. Applies in every support mode.");
                    });
                });
                tier_section(ui, "Bed adhesion", TierKind::Process, false, |ui| {
                    revert_row(ui, &mut s.skirt_loops, &self.baseline.skirt_loops, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0..=5), "skirt loops",
                            "Loops printed around the first layer to prime the nozzle. 0 = off.");
                    });
                    revert_row(ui, &mut s.skirt_gap_mm, &self.baseline.skirt_gap_mm, |ui, v| {
                        hslider(ui, s.skirt_loops > 0, egui::Slider::new(v, 0.0..=10.0), "skirt gap mm",
                            "Distance from the skirt to the model.");
                    });
                    revert_row(ui, &mut s.brim_loops, &self.baseline.brim_loops, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0..=20), "brim loops",
                            "Loops attached around the first layer for adhesion. 0 = off.");
                    });
                });
                tier_section(ui, "Filament", TierKind::Filament, false, |ui| {
                    // The packaging card: what the box says. The material
                    // class itself is profile data — switching filament
                    // profiles changes it — and supplies every derived value
                    // here until a calibration entry pins it.
                    // Spool color rides the slot row below (multi-tool)
                    // or the accent flow (single) — the loaded spool, not the
                    // profile, owns the color.
                    let (line_w, layer_h) = (s.line_width_mm, s.layer_height_mm);
                    let cal = FlowCalUi {
                        host_ready: host_set && !host_busy,
                        start: &mut self.start_flow_cal,
                        measured_mm: &mut self.flow_cal_mm,
                        status: &mut self.status,
                    };
                    // The blend palette lives at the top of the Filament
                    // card — blends are filament-tier facts (mixes of the
                    // spools below). Hidden on a single tool: there is
                    // nothing to mix.
                    if s.tool_count > 1 {
                    // The blend palette: named pseudo colors mixed from the tool
                    // slots below, realized at slice time by alternating whole layers
                    // (engine PartPaint::Blend). Parts pick them in the same
                    // dropdowns as plain tools. Editors expand INLINE in the
                    // panel flow — never a floating Area (bottom-pivoted Areas
                    // feed back their own rect; see the messages pane).
                    let tool_count = s.tool_count;
                    let slot_colors: Vec<[f32; 3]> =
                        (0..tool_count).map(|t| s.tool(t).color_rgb).collect();
                    // The dither cycle the blend band affords: mixes quantize to
                    // whole layers of it, so every offered color actually fuses.
                    let layer_h = s.layer_height_mm;
                    let band_mm = s.blend_band_mm;
                    let dither_cycle = ((band_mm / layer_h.max(0.01)).floor() as usize).max(1);
                    ui.add_space(2.0);
                    ui.horizontal(|ui| {
                        ui.label("blends").on_hover_text(
                            "Pseudo colors mixed from the tool slots — e.g. 3 parts white + \
                             1 part black reads as 25% grey. Printed by alternating whole \
                             layers between the tools in that ratio; paint parts with them \
                             in the object list.",
                        );
                        let add = ui
                            .small_button("+")
                            .on_hover_text(
                                "Add a blend (an even mix of the first and last slot to start).",
                            );
                        if add.clicked() {
                            self.blends.push(config::BlendState {
                                name: format!("blend {}", self.blends.len() + 1),
                                weights: vec![(0, 1.0), (tool_count.saturating_sub(1) as u32, 1.0)],
                                // A fresh blend opens on every spool; narrow it
                                // with the spool toggles in the editor.
                                tools: Vec::new(),
                            });
                        }
                    });
                    let mut delete_blend: Option<usize> = None;
                    let mut weights_edited: Option<usize> = None;
                    // Out-of-band repeats, precomputed (the loop holds the
                    // blends mutably; banding needs &self).
                    let banding: Vec<Option<f64>> =
                        self.blends
                        .iter()
                        .map(|b| blend_banding_for(tool_count, layer_h, band_mm, b))
                        .collect();
                    for (k, blend) in self.blends.iter_mut().enumerate() {
                        let edit_resp = ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 5.0;
                            // Mixed swatch — recomputed every frame, so weight
                            // drags recolor it live. No valid weights = neutral.
                            let entries: Vec<([f32; 3], f32)> = blend
                                .weights
                                .iter()
                                .filter(|&&(t, w)| (t as usize) < tool_count && w > 0.0)
                                .map(|&(t, w)| (slot_colors[t as usize], w))
                                .collect();
                            let rgb = if entries.is_empty() {
                                config::NEUTRAL_FILAMENT_RGB
                            } else {
                                config::mix_colors_linear(&entries)
                            };
                            let (dot, dot_resp) =
                                ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                            ui.painter().circle_filled(dot.center(), 5.0, rgb32(rgb));
                            // Out of the blend band at the current layer height:
                            // an amber ring, visible without opening the editor.
                            if banding[k].is_some() {
                                ui.painter().circle_stroke(
                                    dot.center(),
                                    7.0,
                                    egui::Stroke::new(1.5, egui::Color32::from_rgb(0xC8, 0x8A, 0x4B)),
                                );
                            }
                            if entries.is_empty() {
                                dot_resp.on_hover_text("references missing tools");
                            } else if let Some(mm) = banding[k] {
                                dot_resp.on_hover_text(format!(
                                    "Repeats every {mm:.1} mm at this layer height — past the \
                                     blend band, so the layers will read as stripes. Open the \
                                     editor and re-pick (or \"fit band\"), or use finer layers."
                                ));
                            }
                            ui.scope(|ui| {
                                ui.set_width(150.0);
                                ui.add(egui::Label::new(elide(&blend.name, 22)).truncate());
                            });
                            // ✏ opens the editor popup; ✖ deletes. (✎ and ✕ are
                            // missing from the default fonts — they'd render as
                            // boxes, like "●" does.)
                            let edit_btn = ui
                                .small_button("✏")
                                .on_hover_text("Edit this blend — its color, spools, and name.");
                            if ui
                                .small_button("✖")
                                .on_hover_text(
                                    "Delete this blend — parts painted with it fall back \
                                     to its heaviest tool.",
                                )
                                .clicked()
                            {
                                delete_blend = Some(k);
                            }
                            edit_btn
                        })
                        .inner;
                        // The editor is a breakout popup off the ✏ button: name,
                        // the spool selector, and the printable-mix chip palette.
                        // Clicks inside (spool toggles, chips) keep it open — only
                        // a click outside dismisses it.
                        egui::Popup::menu(&edit_resp)
                            .close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)
                            .show(|ui| {
                            ui.set_max_width(300.0);
                            ui.add(
                                egui::TextEdit::singleline(&mut blend.name)
                                    .desired_width(180.0)
                                    .hint_text("name"),
                            );
                            // The blend's own sub-palette: which spools it draws
                            // from. The mix surface follows THIS count — narrow
                            // an eight-tool machine to two spools and you get the
                            // ramp back, three the triangle.
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 4.0;
                                for t in 0..tool_count as u32 {
                                    let inside =
                                        blend.tools.is_empty() || blend.tools.contains(&t);
                                    let (rect, resp) = ui.allocate_exact_size(
                                        egui::vec2(16.0, 16.0),
                                        egui::Sense::click(),
                                    );
                                    ui.painter().circle_filled(
                                        rect.center(),
                                        if inside { 5.0 } else { 3.5 },
                                        rgb32(slot_colors[t as usize]),
                                    );
                                    if inside {
                                        ui.painter().circle_stroke(
                                            rect.center(),
                                            7.0,
                                            egui::Stroke::new(
                                                1.5,
                                                ui.visuals().strong_text_color(),
                                            ),
                                        );
                                    }
                                    let resp = resp.on_hover_text(format!(
                                        "T{t} — click to {} this blend's palette.",
                                        if inside { "drop it from" } else { "add it to" }
                                    ));
                                    if resp.clicked() {
                                        let mut set: Vec<u32> = if blend.tools.is_empty() {
                                            (0..tool_count as u32).collect()
                                        } else {
                                            blend.tools.clone()
                                        };
                                        if inside {
                                            // Never below one spool; dropping one
                                            // takes its share of the mix along.
                                            if set.len() > 1 {
                                                set.retain(|&x| x != t);
                                                blend.weights.retain(|&(bt, _)| bt != t);
                                                weights_edited = Some(k);
                                            }
                                        } else {
                                            set.push(t);
                                            set.sort_unstable();
                                        }
                                        blend.tools = set;
                                    }
                                }
                                ui.weak("spools");
                            });
                            // The chosen spools, resolved (legacy empty = all).
                            let participants: Vec<u32> = if blend.tools.is_empty() {
                                (0..tool_count as u32).collect()
                            } else {
                                blend
                                    .tools
                                    .iter()
                                    .copied()
                                    .filter(|&t| (t as usize) < tool_count)
                                    .collect()
                            };
                            let sub_colors: Vec<[f32; 3]> =
                                participants.iter().map(|&t| slot_colors[t as usize]).collect();
                            // Subset fractions → stored weights on real slot ids.
                            let apply = |fracs: &[f32]| -> Vec<(u32, f32)> {
                                participants
                                    .iter()
                                    .zip(config::quantize_blend_fractions(fracs, dither_cycle))
                                    .filter(|&(_, n)| n > 0)
                                    .map(|(&t, n)| (t, n as f32))
                                    .collect()
                            };
                            // The palette: every color the chosen spools can
                            // dither to within the band, one chip per whole-layer
                            // recipe — click a chip to set this blend's color.
                            let lat = config::blend_lattice(&sub_colors, dither_cycle, 2000);
                            if let Some(lat) = &lat {
                                if let Some(w) =
                                    lattice_chips(ui, lat, &participants, &blend.weights)
                                {
                                    blend.weights = w;
                                    weights_edited = Some(k);
                                }
                            } else {
                                // Too many mixes to lay out: pick-and-snap —
                                // choose any color, land on the nearest mix.
                                let cur = {
                                    let entries: Vec<([f32; 3], f32)> = blend
                                        .weights
                                        .iter()
                                        .filter(|&&(t, w)| (t as usize) < tool_count && w > 0.0)
                                        .map(|&(t, w)| (slot_colors[t as usize], w))
                                        .collect();
                                    if entries.is_empty() {
                                        config::NEUTRAL_FILAMENT_RGB
                                    } else {
                                        config::mix_colors_linear(&entries)
                                    }
                                };
                                let mut rgb8 = [
                                    (cur[0] * 255.0).round() as u8,
                                    (cur[1] * 255.0).round() as u8,
                                    (cur[2] * 255.0).round() as u8,
                                ];
                                if ui
                                    .color_edit_button_srgb(&mut rgb8)
                                    .on_hover_text(
                                        "Too many printable mixes to lay out as swatches — \
                                         pick any color; it snaps to the nearest mix this \
                                         blend's spools can dither to.",
                                    )
                                    .changed()
                                {
                                    let target = [
                                        rgb8[0] as f32 / 255.0,
                                        rgb8[1] as f32 / 255.0,
                                        rgb8[2] as f32 / 255.0,
                                    ];
                                    blend.weights = apply(&config::blend_weights_for_color(
                                        target,
                                        &sub_colors,
                                    ));
                                    weights_edited = Some(k);
                                }
                            }
                        });
                    }
                    if let Some(k) = delete_blend {
                        // Parts painted with it fall back to its heaviest valid
                        // tool; parts on later blends keep their blend (the index
                        // shifts down with the removal).
                        let fallback = valid_weights_for(tool_count, &self.blends[k])
                            .into_iter()
                            .max_by(|a, b| a.1.total_cmp(&b.1))
                            .map(|(t, _)| t)
                            .unwrap_or(0);
                        self.blends.remove(k);
                        let mut repainted = false;
                        for part in self.objects.iter_mut().flat_map(|o| &mut o.parts) {
                            match part.paint {
                                PartColor::Blend(b) if b == k => {
                                    part.paint = PartColor::Tool(fallback);
                                    repainted = true;
                                }
                                PartColor::Blend(b) if b > k => part.paint = PartColor::Blend(b - 1),
                                _ => {}
                            }
                        }
                        // Same invalidation as a tool reassignment — only when a
                        // part actually changed paint (index shifts are identity).
                        if repainted {
                            self.sliced = None;
                            self.slice_summary = None;
                            self.view_preview = false;
                        }
                        self.mesh_color_dirty = true; // swatch rows changed; tints may have
                    }
                    if let Some(k) = weights_edited {
                        // The swatch is already live (recomputed above per frame);
                        // a part wearing this blend needs its model tint refreshed
                        // (rebuild_scene bumps content_version — RenderSig won't
                        // catch a tint change on its own) and its plan is stale.
                        if self
                            .objects
                            .iter()
                            .flat_map(|o| &o.parts)
                            .any(|p| p.paint == PartColor::Blend(k))
                        {
                            self.sliced = None;
                            self.slice_summary = None;
                            self.view_preview = false;
                            self.mesh_color_dirty = true;
                        }
                        ui.ctx().request_repaint();
                    }
                    ui.separator();
                        ui.add_space(4.0);
                    }
                    // The per-slot edit surface — identical shape at every
                    // tool count (a single-tool machine is a one-slot
                    // toolchanger here): tab dots, the slot row, the card
                    // rows (temps/flow, then the cooling fans). Data still
                    // flows the old way underneath: on a
                    // toolchanger every row binds the active slot only (the
                    // flat fields are just the tool-0 mirror, see
                    // `refresh_tool0`, so a save can never write one profile
                    // over another), while a single tool edits the FLAT
                    // fields and saves through the flat tier path.
                    {
                        let tab =
                            tool_tab_row(ui, &mut self.active_tool_tab, &s.tools, &tool_dirty_flags);
                        let name = self.tools.get(tab).cloned().unwrap_or_default();
                        let is_user = self.profiles.is_user(TierKind::Filament, &name);
                        let is_dirty = tool_dirty_flags.get(tab).copied().unwrap_or(false);
                        self.tool_hex.resize(s.tool_count, String::new());
                        // The slot's whole loadout in one row — spool color,
                        // loaded filament profile, the label's hex code — so
                        // every per-slot act lives on this card (this row
                        // replaced the tool strip that sat at the top of the
                        // panel). The tab dots above carry T-number and the
                        // unsaved-* mark.
                        ui.horizontal(|ui| {
                            let c = s.tool(tab).color_rgb;
                            let mut rgb8 = [
                                (c[0] * 255.0).round() as u8,
                                (c[1] * 255.0).round() as u8,
                                (c[2] * 255.0).round() as u8,
                            ];
                            if ui
                                .color_edit_button_srgb(&mut rgb8)
                                .on_hover_text(
                                    "The color of the spool loaded in this slot — tints its \
                                     parts, blends, and the filament preview. Overrides the \
                                     filament profile's color; changing the slot's filament \
                                     returns to the profile's own.",
                                )
                                .changed()
                            {
                                color_pick = Some((
                                    tab,
                                    [
                                        rgb8[0] as f32 / 255.0,
                                        rgb8[1] as f32 / 255.0,
                                        rgb8[2] as f32 / 255.0,
                                    ],
                                ));
                            }
                            let mut sel = name.clone();
                            egui::ComboBox::from_id_salt(("tool_slot", tab))
                                .width(104.0)
                                .selected_text(elide(&sel, 12))
                                .show_ui(ui, |ui| {
                                    for opt in self.profiles.filament_names() {
                                        let opt = opt.to_string();
                                        if ui
                                            .selectable_value(&mut sel, opt.clone(), &opt)
                                            .changed()
                                        {
                                            slot_pick = Some((tab, sel.clone()));
                                        }
                                    }
                                })
                                .response
                                .on_hover_text(
                                    "The filament loaded in this tool slot — the rows below \
                                     edit it.",
                                );
                            // The spool label's code, typed straight in:
                            // applies on enter/blur; the field snaps back to
                            // the current color whenever it isn't being edited.
                            let cur_hex = config::hex_color(s.tool(tab).color_rgb);
                            let field = ui.add(
                                egui::TextEdit::singleline(&mut self.tool_hex[tab])
                                    .desired_width(56.0)
                                    .hint_text("#RRGGBB"),
                            );
                            if field.lost_focus() {
                                if let Some(c) = config::parse_hex_color(&self.tool_hex[tab]) {
                                    color_pick = Some((tab, c));
                                }
                            }
                            if !field.has_focus() {
                                self.tool_hex[tab] = cur_hex;
                            }
                            field.on_hover_text(
                                "Type the spool's color code from its label (#RRGGBB or \
                                 #RGB) and press enter.",
                            );
                            if ui
                                .small_button("💾")
                                .on_hover_text(if is_dirty {
                                    "Save this tab's * changes as a user profile (only changed fields are written)."
                                } else {
                                    "Save a copy as a user profile."
                                })
                                .clicked()
                            {
                                let dlg = if is_user { name.clone() } else { format!("{name}-custom") };
                                tool_dialog = Some(ProfileDialog {
                                    kind: TierKind::Filament,
                                    name: dlg,
                                    delete: false,
                                    slot: (s.tool_count > 1).then_some(tab),
                                });
                            }
                            if is_user
                                && ui
                                    .small_button("🗑")
                                    .on_hover_text("Delete this user profile from disk.")
                                    .clicked()
                            {
                                tool_dialog = Some(ProfileDialog {
                                    kind: TierKind::Filament,
                                    name: name.clone(),
                                    delete: true,
                                    slot: Some(tab),
                                });
                            }
                        });
                        let fb = if s.tool_count > 1 {
                            self.baseline
                                .tools
                                .get(tab)
                                .map(FilamentBaseline::tool)
                                .unwrap_or_else(|| FilamentBaseline::flat(&self.baseline))
                        } else {
                            FilamentBaseline::flat(&self.baseline)
                        };
                        // Cooling rides the filament card now (no separate
                        // section): part-fan duties are filament-tier facts,
                        // per material/spool, bound to the same active tab.
                        let (aux, exhaust) = (s.has_aux_fan, s.has_exhaust_fan);
                        // Docked-tool standby only applies to independent hotends;
                        // a shared heater ramps its one heater, so hide the row.
                        let show_standby = !s.single_heater();
                        if s.tool_count > 1 {
                            filament_card_rows(ui, FilamentFields::tool(&mut s.tools[tab]), &fb, show_standby, line_w, layer_h, cal);
                            ui.separator();
                            cooling_rows(ui, FilamentFields::tool(&mut s.tools[tab]), &fb, aux, exhaust);
                        } else {
                            filament_card_rows(ui, FilamentFields::flat(s), &fb, false, line_w, layer_h, cal);
                            ui.separator();
                            cooling_rows(ui, FilamentFields::flat(s), &fb, aux, exhaust);
                        }
                    }
                });
                tier_section(ui, "Retraction", TierKind::Printer, false, |ui| {
                    revert_row(ui, &mut s.retract_len_mm, &self.baseline.retract_len_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.0..=10.0), "length mm",
                            "Filament pulled back on travels to prevent oozing/stringing.");
                    });
                    revert_row(ui, &mut s.retract_speed_mm_s, &self.baseline.retract_speed_mm_s, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 5.0..=100.0), "speed mm/s",
                            "How fast filament is retracted and recovered.");
                    });
                    revert_row(ui, &mut s.z_hop_mm, &self.baseline.z_hop_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.0..=2.0), "z-hop mm",
                            "Lift the nozzle on travels that cross a gap/void. 0 = off.");
                    });
                    revert_row(ui, &mut s.wipe_mm, &self.baseline.wipe_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.0..=5.0), "wipe mm",
                            "After retracting, drag the nozzle back along the printed bead by this much before travelling — ooze smears onto plastic instead of blobbing the seam. 0 = off.");
                    });
                });
                tier_section(ui, "Machine & motion", TierKind::Printer, false, |ui| {
                    revert_row(ui, &mut s.bed_size_x_mm, &self.baseline.bed_size_x_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 50.0..=500.0), "bed X mm",
                            "Bed width (X).");
                    });
                    revert_row(ui, &mut s.bed_size_y_mm, &self.baseline.bed_size_y_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 50.0..=500.0), "bed Y mm",
                            "Bed depth (Y).");
                    });
                    revert_row(ui, &mut s.bed_size_z_mm, &self.baseline.bed_size_z_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 50.0..=600.0), "bed Z mm",
                            "Maximum build height (Z).");
                    });
                    revert_row(ui, &mut s.nozzle_diameter_mm, &self.baseline.nozzle_diameter_mm, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 0.1..=1.2), "nozzle mm",
                            "Nozzle diameter.");
                    });
                    // Toolchanger datasheet facts: how many physical tool
                    // slots, and what one swap costs the estimate. One value
                    // per row — three DragValue+label pairs on one line is
                    // ~380 pt, and any row wider than the panel reserves the
                    // overflow, pushing the viewport right and opening an
                    // unpainted band (egui #4475; see the panel comment).
                    let before = s.tool_count;
                    // Machine type: one dropdown = the machine's identity. "Single
                    // nozzle" is simply one tool; the three multi-tool modes
                    // describe how 2+ tools swap (heater model + whether a swap
                    // purges). The slot count and swap facts appear below only when
                    // it's a multi-tool machine.
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 5.0;
                        machine_type_combo(ui, &mut s.tool_count, &mut s.machine_kind).on_hover_text(
                            "What kind of machine this is:\n\
                             • Single nozzle — one hotend, one filament. The ordinary case.\n\
                             • Independent hotends — a head per tool, docked and swapped. Each \
                             keeps its own filament (nothing purged) and its own live temperature.\n\
                             • Shared nozzle — one hotend fed by a selector. A swap unloads, loads, \
                             and FLUSHES the old color through the shared melt zone; the single \
                             heater ramps between materials.\n\
                             • Separate nozzles + shared heater — a nozzle per filament (so no \
                             flush) taking turns in ONE heater. A swap indexes the next nozzle in \
                             and waits for it to reach temperature.\n\
                             Set the swap macro under Custom g-code — placeholders {tool} / \
                             {from_tool} / {to_temp} / {purge_mm3} / {purge_mm}.",
                        );
                        // Revert the whole machine type (count + swap mode) at once.
                        let cur_single = s.tool_count <= 1;
                        let base_single = self.baseline.tool_count <= 1;
                        let differs = cur_single != base_single
                            || (!cur_single && s.machine_kind != self.baseline.machine_kind);
                        if differs
                            && ui
                                .small_button("⟲")
                                .on_hover_text("Edited — click to revert to the profile's value.")
                                .clicked()
                        {
                            s.tool_count = self.baseline.tool_count;
                            s.machine_kind = self.baseline.machine_kind;
                        }
                    });
                    if s.tool_count > 1 {
                        revert_row(ui, &mut s.tool_count, &self.baseline.tool_count, |ui, v| {
                            ui.add(egui::DragValue::new(v).range(2..=8));
                            ui.label("tools").on_hover_text(
                                "Filament slots on the machine. Each loads its filament on the \
                                 Filament card's tabs; parts pick their tool in the object list.",
                            );
                        });
                        revert_row(ui, &mut s.toolchange_seconds, &self.baseline.toolchange_seconds, |ui, v| {
                            ui.add(
                                egui::DragValue::new(v).speed(0.5).range(0.0..=300.0).suffix(" s"),
                            );
                            ui.label("s/change").on_hover_text(
                                "Seconds one tool swap takes — feeds the print-time estimate. \
                                 A shared nozzle adds the purge time on top; a shared heater's \
                                 swap should include its nozzle reheat here (one heater can't \
                                 preheat the next nozzle while printing).",
                            );
                        });
                        match s.machine_kind {
                            config::MachineKind::SharedNozzle => {
                                // Shared melt zone: a static purge per swap, handed
                                // to the macro and counted as waste. No standby row
                                // (one heater).
                                revert_row(ui, &mut s.purge_volume_mm3, &self.baseline.purge_volume_mm3, |ui, v| {
                                    ui.add(
                                        egui::DragValue::new(v).speed(1.0).range(0.0..=1000.0).suffix(" mm³"),
                                    );
                                    ui.label("purge/swap").on_hover_text(
                                        "Filament flushed at every swap to clear the old color — \
                                         handed to the swap macro as {purge_mm3} / {purge_mm} and \
                                         counted as waste in the estimate. The firmware decides \
                                         where it goes (a purge bucket / dump); the slicer lays \
                                         no wipe tower.",
                                    );
                                });
                            }
                            config::MachineKind::IndependentHotends => {
                                revert_row(ui, &mut s.standby_after_s, &self.baseline.standby_after_s, |ui, v| {
                                    ui.add(
                                        egui::DragValue::new(v).speed(5.0).range(0.0..=3600.0).suffix(" s"),
                                    );
                                    ui.label("standby after").on_hover_text(
                                        "Docked longer than this and a tool drops to its filament's \
                                         standby temperature, reheating a layer ahead of its next \
                                         pickup. Short docks (blend dithering) stay at print \
                                         temperature. 0 disables standby entirely.",
                                    );
                                });
                            }
                            // Separate nozzles + shared heater: no flush (each tip
                            // holds its own color) and no standby (idle nozzles
                            // cool) — the reheat rides s/change. Nothing to add.
                            config::MachineKind::SharedHeater => {}
                        }
                    }
                    if s.tool_count != before {
                        tool_count_changed = true;
                    }
                    revert_row(ui, &mut s.machine_speed_mm_s, &self.baseline.machine_speed_mm_s, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 10.0..=700.0), "rated mm/s",
                            "The machine's rated print speed — a datasheet number, the hard cap every derived speed works under. Lower it to slow the whole machine.");
                    });
                    revert_row(ui, &mut s.first_layer_speed_mm_s, &self.baseline.first_layer_speed_mm_s, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 5.0..=100.0), "1st layer mm/s",
                            "Speed for the first layer — slower improves bed adhesion.");
                    });
                    revert_row(ui, &mut s.travel_speed_mm_s, &self.baseline.travel_speed_mm_s, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 20.0..=600.0), "travel mm/s",
                            "Speed for non-printing moves between features.");
                    });
                    revert_row(ui, &mut s.acceleration_mm_s2, &self.baseline.acceleration_mm_s2, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 100.0..=20000.0), "accel mm/s²",
                            "Acceleration for inner walls, infill, solid, support, and travel — emitted as M204 per feature. Klipper clamps to printer.cfg max_accel. Higher = faster but more ringing.");
                    });
                    auto_slider(ui, &mut s.outer_wall_accel_mm_s2, 100.0..=20000.0, "outer accel",
                        &mut pins.outer_wall_accel, config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2),
                        profile_pins.outer_wall_accel, self.baseline.outer_wall_accel_mm_s2,
                        "Acceleration for the visible outermost wall — lower hides ringing. Auto = 50% of accel.");
                    auto_slider(ui, &mut s.first_layer_accel_mm_s2, 100.0..=20000.0, "1st layer accel",
                        &mut pins.first_layer_accel, config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2),
                        profile_pins.first_layer_accel, self.baseline.first_layer_accel_mm_s2,
                        "Acceleration for the whole first layer — gentle for bed adhesion. Auto = min(1000, accel).");
                    revert_row(ui, &mut s.jerk_mm_s, &self.baseline.jerk_mm_s, |ui, v| {
                        hslider(ui, true, egui::Slider::new(v, 1.0..=50.0), "jerk mm/s",
                            "Klipper square-corner-velocity — how briskly direction changes are taken.");
                    });
                    revert_row(ui, &mut s.min_cruise_ratio, &self.baseline.min_cruise_ratio, |ui, v| {
                        hslider(ui, true,
                            egui::Slider::new(v, 0.0..=0.95)
                                .custom_formatter(|v, _| format!("{:.0}%", v * 100.0))
                                .custom_parser(|t| t.trim().trim_end_matches('%').parse::<f64>().ok().map(|v| v / 100.0)),
                            "cruise smoothing",
                            "Cruise smoothing (Klipper accel-to-decel): forces each move to spend at least this fraction cruising instead of sprinting up to speed and braking back down. 0% = fastest/sharpest; higher = smoother and quieter on short moves (infill, fine detail, arcs), a touch slower. Emitted as ACCEL_TO_DECEL = accel × (1 − this). Separate from jerk, which only sets cornering speed.");
                    });
                    revert_row(ui, &mut s.arc_fitting, &self.baseline.arc_fitting, |ui, v| {
                        ui.checkbox(v, "arc fitting (G2/G3)")
                            .on_hover_text("Emit curved toolpaths as G2/G3 arcs — smaller g-code, smoother motion. A firmware capability: needs arc support enabled (Klipper [gcode_arcs]; Marlin ARC_SUPPORT). Turn off for firmware that doesn't recognize G2/G3.");
                    });
                    revert_row(ui, &mut s.arc_tolerance_mm, &self.baseline.arc_tolerance_mm, |ui, v| {
                        hslider(ui, s.arc_fitting, egui::Slider::new(v, 0.005..=0.2), "arc tol mm",
                            "Max deviation a point may have from a fitted arc to be folded into it.");
                    });
                    // Hardware the printer either has or doesn't. Declared here
                    // rather than via a printer.cfg macro so a downloaded slicer
                    // is self-contained — no macros to install. Off by default:
                    // M106 P2/P3 have no non-breaking guard (vanilla firmware
                    // reads them as the primary fan), so they emit only once the
                    // hardware is confirmed.
                    revert_row(ui, &mut s.has_aux_fan, &self.baseline.has_aux_fan, |ui, v| {
                        ui.checkbox(v, "aux part fan (M106 P2)")
                            .on_hover_text("Tick if the machine has an auxiliary side part-cooling fan (Sovol Zero / Bambu-style). Off by default and gated — vanilla Klipper/Marlin read M106 P2 as the *primary* fan, so the slicer emits it only once you confirm the hardware. Its duty lives in the Cooling section.");
                    });
                    revert_row(ui, &mut s.has_exhaust_fan, &self.baseline.has_exhaust_fan, |ui, v| {
                        ui.checkbox(v, "exhaust fan (M106 P3)")
                            .on_hover_text("Tick if the machine has a chamber-exhaust fan (M106 P3). Off by default for the same reason as the aux fan; its duty lives in the Cooling section.");
                    });
                    revert_row(ui, &mut s.chamber_sensor, &self.baseline.chamber_sensor, |ui, v| {
                        ui.add(egui::TextEdit::singleline(v).desired_width(120.0).hint_text("none"));
                        ui.label("chamber sensor").on_hover_text(
                            "The chamber thermistor's Klipper temperature_sensor name — \
                             \"chamber_temp\" on the Sovol Zero, \"chamber\" on a spec Voron \
                             (check Fluidd/Mainsail or `SENSORS`). Empty = none; a filament that \
                             wants a soak then fails the pre-send check (and would abort at the \
                             printer).",
                        );
                    });
                });
                tier_section(ui, "Connection", TierKind::Printer, false, |ui| {
                    revert_row(ui, &mut s.host_url, &self.baseline.host_url, |ui, _| {
                        ui.label("printer host").on_hover_text(
                            "The printer's Moonraker address — e.g. voron24.local or 192.168.1.50. \
                             Plain HTTP is assumed without a scheme. Empty = no connection.",
                        );
                    });
                    ui.add(egui::TextEdit::singleline(&mut s.host_url).hint_text("192.168.1.50 or printer.local"));
                    revert_row(ui, &mut s.api_key, &self.baseline.api_key, |ui, _| {
                        ui.label("API key").on_hover_text(
                            "Only needed when Moonraker's [authorization] section requires one.",
                        );
                    });
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
                    revert_row(ui, &mut s.start_gcode, &self.baseline.start_gcode, |ui, _| {
                        ui.label("Start g-code").on_hover_text(
                            "Emitted before the print. Placeholders: {nozzle_temp} {first_layer_nozzle_temp} {bed_temp} {bed_x} {bed_y} {bed_z} {layer_height} {first_layer_height} {nozzle_diameter}.",
                        );
                    });
                    ui.add(
                        egui::TextEdit::multiline(&mut s.start_gcode)
                            .code_editor()
                            .desired_rows(4)
                            .desired_width(f32::INFINITY),
                    );
                    revert_row(ui, &mut s.end_gcode, &self.baseline.end_gcode, |ui, _| {
                        ui.label("End g-code").on_hover_text("Emitted after the print (cooldown, park, motors off).");
                    });
                    ui.add(
                        egui::TextEdit::multiline(&mut s.end_gcode)
                            .code_editor()
                            .desired_rows(4)
                            .desired_width(f32::INFINITY),
                    );
                    if s.tool_count > 1 {
                        revert_row(ui, &mut s.toolchange_gcode, &self.baseline.toolchange_gcode, |ui, _| {
                            ui.label("Swap g-code").on_hover_text(
                                "Emitted at every tool change. Placeholders: {tool} (incoming \
                                 slot), {from_tool} (previous), {to_temp} (target nozzle °C), \
                                 {purge_mm3} / {purge_mm} (shared-nozzle purge as volume / \
                                 filament length). The default \"T{tool}\" is a bare tool select \
                                 (which Klipper macros can remap to a full swap).",
                            );
                        });
                        ui.add(
                            egui::TextEdit::multiline(&mut s.toolchange_gcode)
                                .code_editor()
                                .desired_rows(2)
                                .desired_width(f32::INFINITY),
                        );
                    }
                });
                ui.add_space(6.0);
                ui.weak("drag: orbit · right-drag: pan · scroll: zoom");
                });
            if let Some(d) = tool_dialog {
                self.profile_dialog = Some(d);
            }
            if let Some((i, c)) = color_pick {
                let slot_name =
                    if i == 0 { self.filament.clone() } else { self.tools[i].clone() };
                self.tool_colors.resize(self.settings.tool_count.max(1), None);
                self.tool_colors[i] = Some((slot_name, c));
                if let Some(t) = self.settings.tools.get_mut(i) {
                    t.color_rgb = c;
                }
                if i == 0 {
                    self.settings.filament_color_rgb = c;
                }
                filament_color_changed = true;
            }
            if let Some((i, name)) = slot_pick {
                let cur = if i == 0 { &self.filament } else { &self.tools[i] };
                if name != *cur {
                    // Unsaved (*) edits on the slot: switching re-reads it
                    // from disk, so park the pick behind the per-slot
                    // confirm. Single-tool edits live on the FLAT fields
                    // (tools[0] is only their mirror), so ask the flat diff.
                    let dirty = if self.settings.tool_count > 1 {
                        self.tool_dirty(i)
                    } else {
                        !FilamentProfile::diff(&self.settings, &self.baseline).is_empty()
                    };
                    if dirty {
                        self.pending_slot = Some((i, name));
                    } else {
                        self.apply_slot_switch(i, name);
                    }
                }
            }
        });

        // A tool-count edit resizes the slot list and re-resolves the tool
        // table; the plan is stale (tools were clamped to the old count).
        if tool_count_changed {
            self.resync_tools();
            self.sliced = None;
            self.slice_summary = None;
            self.view_preview = false;
            // single↔multi flips the model tint (accent vs per-part spool
            // colors) and reclamps part paints — a color change, not geometry.
            self.mark_mesh_color_dirty();
            ui.ctx().request_repaint();
        }
        // A spool-color edit re-tints tool-0 parts; single-tool models keep the
        // accent tint, nothing to do. The Filament preview recolors itself from
        // the tool palette in-shader (no instance rebuild) — the mesh re-tint
        // below bumps content_version, so the fresh palette lands on the beads.
        if filament_color_changed && self.settings.tool_count > 1 {
            self.mark_mesh_color_dirty();
            ui.ctx().request_repaint();
        }

        // Execute any printer-host action requested above, on a worker thread.
        // Frameless: the viewport texture runs edge-to-edge against the panel
        // separator instead of sitting in an 8 pt dark mat.
        egui::CentralPanel::default().frame(egui::Frame::NONE).show_inside(ui, |ui| {
            let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let aspect = rect.width() / rect.height().max(1.0);
            let vp = self.camera.view_proj(aspect);

            // The camera is in motion this frame if a glide is running or the
            // user orbits/pans/zooms below. While it moves, the scene renders at
            // reduced resolution (dynamic resolution) — you can't see fine detail
            // while spinning — and snaps back to full res the frame it settles.
            let mut camera_moving = self.camera_glide.is_some();

            // Objects are only editable in Model view; Preview is read-only.
            // Painting also happens in Model view now (it records sub-bead dabs
            // that resolve at slice time); Preview just shows the result.
            let edit = !self.view_preview;
            // Ignore viewport input when the cursor is over a floating overlay.
            let pointer = ui.ctx().pointer_interact_pos();
            let over = |r: Option<egui::Rect>| matches!((r, pointer), (Some(r), Some(p)) if r.contains(p));
            let blocked = over(self.overlay_rect)
                || over(self.bed_overlay_rect)
                || over(self.print_overlay_rect)
                || over(self.msgs_overlay_rect);

            // Left-press on an object grabs it for dragging; on empty space, orbits.
            // In paint mode nothing is grabbed — a drag still orbits (so you can
            // rotate to the back), and a click paints (handled below).
            // In paint mode, a drag that starts on the model is a brush stroke
            // (painted below); starting on empty space still orbits.
            if edit && !blocked && self.paint_mode && response.drag_started_by(egui::PointerButton::Primary) {
                self.painting = response
                    .interact_pointer_pos()
                    .map(|p| {
                        let (o, d) = pointer_ray(vp, rect, p);
                        self.pick_paint_face(o, d).is_some()
                    })
                    .unwrap_or(false);
            }
            if edit && !blocked && !self.paint_mode && response.drag_started_by(egui::PointerButton::Primary) {
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
                        // Grabbing an object selects it — only the spotlight moves.
                        self.spotlight_dirty = true;
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
                                // Dragging a part rightward past the last bed
                                // creates the bed under it (and it persists).
                                self.grow_beds_to_fit();
                                // The object's placement moved (and a bed may
                                // have been added) — re-bake geometry + beds.
                                self.mark_geom_dirty();
                                self.beds_dirty = true;
                                self.sliced = None;
                                self.slice_summary = None;
                                self.view_preview = false;
                            }
                        }
                    }
                    None => {
                        if self.paint_mode && self.painting {
                            // A brush stroke on the MODEL: flood the surface patch
                            // under the ray each frame → tint the mesh (live) +
                            // record a sub-bead dab (applied when sliced). Skips
                            // orbit; a drag off the model still orbits.
                            if let Some(p) = response.interact_pointer_pos() {
                                let (o, d) = pointer_ray(vp, rect, p);
                                self.paint_dab(o, d);
                            }
                        } else if !blocked {
                            let d = response.drag_delta();
                            self.camera.orbit(d.x, d.y);
                            camera_moving = true;
                        }
                    }
                }
            }
            if response.drag_stopped_by(egui::PointerButton::Primary) {
                self.drag_obj = None;
                // A finished paint stroke: push the new dabs onto the sliced beads
                // + refresh the readout (no-op if not yet sliced).
                if self.painting {
                    self.commit_paint_stroke(&rs);
                }
                self.painting = false;
            }
            // A plain click selects the object under the cursor (its bed
            // becomes active), or — on empty space — deselects and activates
            // the bed nearest the click.
            if edit && !blocked && response.clicked() {
                if let Some(p) = response.interact_pointer_pos() {
                    let (o, d) = pointer_ray(vp, rect, p);
                    if self.paint_mode {
                        // Smart-fill the surface region under the cursor, then
                        // push the dab onto the sliced beads + readout.
                        self.paint_click(o, d);
                        self.commit_paint_stroke(&rs);
                    } else {
                        self.selected = self.pick(o, d);
                        match self.selected {
                            Some(i) => {
                                let k = self.bed_of(&self.objects[i]);
                                self.set_active_bed(k);
                            }
                            None => {
                                if let Some(xy) = ray_plane_z0(o, d) {
                                    let k = bed_of_pos(xy.x as f64, self.settings.bed_size_x_mm)
                                        .min(self.n_beds() - 1);
                                    self.set_active_bed(k);
                                }
                            }
                        }
                        // A click only changes the selection (and maybe the active
                        // bed, handled by set_active_bed) — the mesh is untouched.
                        self.spotlight_dirty = true;
                    }
                }
            }
            if response.dragged_by(egui::PointerButton::Secondary) {
                // A manual pan takes the pivot back — cancel any glide so it
                // doesn't fight the hand on the camera.
                self.camera_glide = None;
                let d = response.drag_delta();
                self.camera.pan(d.x, d.y);
                camera_moving = true;
            }
            if response.hovered() && !blocked {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    // Manual zoom takes the camera back mid-glide.
                    self.camera_glide = None;
                    self.camera.zoom(scroll);
                    camera_moving = true;
                }
            }

            // Dynamic resolution: render at half size while the camera moves
            // (invisible while spinning — egui upsamples with Linear), full size
            // when it settles. resize() no-ops when the size is unchanged, so a
            // continuous drag reallocates the targets only at motion start/stop.
            let ppp = ui.ctx().pixels_per_point();
            let full_w = (rect.width() * ppp).round().max(1.0) as u32;
            let full_h = (rect.height() * ppp).round().max(1.0) as u32;
            let scale = if camera_moving { 0.5 } else { 1.0 };
            let w = ((full_w as f32 * scale).round() as u32).max(1);
            let h = ((full_h as f32 * scale).round() as u32).max(1);
            // Full dims drive the star-billboard sizing; the (maybe halved) w/h
            // drive the render target AND RenderSig.size, so the settle frame
            // (scale 1.0 → different size) forces a full-res re-render.
            self.scene.set_display_size(full_w, full_h);
            self.scene.resize(&rs, w, h);
            // Guarantee the frame after motion ends is drawn at full res (during
            // an active drag egui repaints anyway; this covers the smooth-scroll
            // tail). Self-terminating: the settle frame has camera_moving=false.
            if camera_moving {
                ui.ctx().request_repaint();
            }
            let show_mesh = !(self.view_preview && self.sliced.is_some());
            let preview = if self.view_preview && self.sliced.is_some() {
                let n = self.layer_ends.len();
                let idx = self.preview_layer.saturating_sub(1);
                let count = self.layer_ends.get(idx).copied().unwrap_or(0);
                let joint_count = self.joint_layer_ends.get(idx).copied().unwrap_or(0);
                // Dim lower layers only when the slider is below the top.
                let dim = if self.preview_layer >= n { 1.0 } else { 0.15 };
                // Filament mode recolors extrusion beads from the tool palette
                // in-shader, so a spool-color change is a uniform update, not an
                // instance rebuild. Other modes use each bead's baked rgb.
                let color_mode = if self.color_by == ColorBy::Filament { 1 } else { 0 };
                Some(render::Preview {
                    count,
                    joint_count,
                    current_layer: self.preview_layer as f32,
                    dim,
                    mask: self.category_mask(),
                    color_mode,
                    tool_palette: self.tool_palette(),
                })
            } else {
                None
            };
            // Re-render the 3D scene only when something it depends on changed —
            // camera, layer, mask, contents, accent, size. egui repaints on every
            // input (a mouse-move over the panel included); without this a big
            // slice redrew all its beads on each of those frames.
            let preview_sig = preview.as_ref().map(|p| {
                (p.count, p.joint_count, p.current_layer.to_bits(), p.dim.to_bits(), p.mask)
            });
            // Only the palette actually feeds a pixel (via ctrl.w) in Filament
            // mode, so hash it just then — a spool-color change re-renders the
            // beads without any content_version bump.
            let preview_color = match preview.as_ref() {
                Some(p) if p.color_mode == 1 => (1u32, palette_hash(&p.tool_palette)),
                _ => (0, 0),
            };
            // Re-derive the view-projection from the camera AFTER this frame's
            // orbit/pan/zoom. The `vp` above is the PRE-interaction one, kept for
            // unprojecting clicks against the frame the user is actually looking
            // at; but the render must use the POST-interaction camera so its view
            // matrix agrees with the `cam_eye` the star backdrop is centered on.
            // If they disagree by a frame of motion, `view * cam_eye != 0` and the
            // infinite-distance stars pick up parallax — negligible under orbit (a
            // tangential shift ÷1000) but a visible radial drift under zoom (the
            // eye delta is along the view axis), which read as lag/vertigo.
            let vp = self.camera.view_proj(aspect);
            let sig = RenderSig {
                vp,
                show_mesh,
                show_stars: self.show_stars,
                preview: preview_sig,
                accent: self.accent,
                size: (w, h),
                content: self.content_version,
                preview_color,
            };
            if self.last_render_sig.as_ref() != Some(&sig) {
                // Camera-relative key + fill so surface detail reads at any orbit.
                let (key, fill) = render::camera_lights(self.camera.eye(), self.camera.target);
                // The dim taupe the egui overlay used, in the scene's perceptual
                // space (mesh/grid shaders don't linearize either).
                let label_color = [104.0 / 255.0, 98.0 / 255.0, 86.0 / 255.0, 1.0];
                self.scene.render(
                    &rs, vp, self.camera.eye(), self.show_stars, show_mesh, preview, key, fill,
                    label_color,
                );
                self.last_render_sig = Some(sig);
            }

            ui.painter().image(
                self.scene.texture_id(),
                rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );

            // Live brush cursor: a ring on the surface at the pointer, sized to
            // the freehand brush — drawn AFTER the scene image so it sits on top.
            if edit && !blocked && self.paint_mode {
                if let Some(p) = ui.ctx().pointer_hover_pos() {
                    if rect.contains(p) {
                        // Keep repainting so the cursor tracks smoothly.
                        ui.ctx().request_repaint();
                        let col = if self.brush_erase {
                            egui::Color32::from_rgb(230, 120, 100)
                        } else {
                            let c = render::visible_against_backdrop(
                                self.settings.tool(self.brush_tool as usize).color_rgb,
                            );
                            egui::Color32::from_rgb(
                                (c[0] * 255.0) as u8,
                                (c[1] * 255.0) as u8,
                                (c[2] * 255.0) as u8,
                            )
                        };
                        let stroke = egui::Stroke::new(1.5, col);
                        // The brush footprint draped on the actual surface (a
                        // geodesic disc of the brush radius, traced as its
                        // boundary) — projected onto the model in both views, so
                        // it hugs the geometry instead of floating as a flat ring.
                        if let Some(segs) = self.paint_cursor_outline(vp, rect, p) {
                            for [a, b] in &segs {
                                ui.painter().line_segment([*a, *b], stroke);
                            }
                        }
                    }
                }
            }

            // Front-edge marker: which edge is the printer's front (the origin
            // edge, y=0) is otherwise ambiguous. The "Front" wordmark (Playfair,
            // sliced "F") is laid out once, then mapped onto the active bed's
            // front edge as depth-tested scene geometry (see render.rs) so the
            // model occludes it per-pixel instead of painting over it.
            {
                let bx = self.settings.bed_size_x_mm;
                let by = self.settings.bed_size_y_mm;
                if self.front_label.is_none() {
                    if let Some(local) = front_label_local(ui) {
                        // Snapshot egui's font atlas into an R8 coverage texture.
                        let img = ui.ctx().fonts(|f| f.image());
                        let (aw, ah) = (img.size[0] as u32, img.size[1] as u32);
                        let gray = img.pixels.iter().all(|p| p.a() == 255);
                        let coverage: Vec<u8> =
                            img.pixels.iter().map(|p| if gray { p.r() } else { p.a() }).collect();
                        self.scene.set_label_atlas(&rs.device, &rs.queue, aw, ah, &coverage);
                        self.front_label = Some(local);
                        self.last_label_bed = None;
                    }
                }
                let key = (bx, by, self.active_bed);
                if self.front_label.is_some() && self.last_label_bed != Some(key) {
                    let (local, tw, th) = {
                        let (l, tw, th) = self.front_label.as_ref().unwrap();
                        (l.clone(), *tw, *th)
                    };
                    // galley px → world on z=0, laid just outside the near (y=0)
                    // edge, sized to a fraction of the bed (matches the old overlay).
                    let ox = bed_origin_x(self.active_bed, bx) as f32;
                    let (bxf, byf) = (bx as f32, by as f32);
                    let h_world = (bxf.min(byf) * 0.05).clamp(8.0, 14.0);
                    let scale = h_world / th;
                    let x0 = ox + bxf * 0.5 - tw * scale * 0.5;
                    let pad = 0.01 * byf + 2.0;
                    let verts: Vec<[f32; 5]> = local
                        .iter()
                        .map(|&([gx, gy], [u, v])| [x0 + gx * scale, -pad - gy * scale, 0.0, u, v])
                        .collect();
                    self.scene.set_label_geom(&rs.device, &rs.queue, &verts);
                    self.last_label_bed = Some(key);
                    self.content_version += 1;
                    // Geometry uploaded after this frame's render — make sure the
                    // next frame (which re-renders on the content bump) happens.
                    ui.ctx().request_repaint();
                }
            }

            // Floating bed card, top-center: step/select the active bed, add
            // a fresh one, remove an empty one. Always visible — bed
            // switching matters in Preview too (it re-aims Slice).
            {
                let n = self.n_beds();
                let active_empty =
                    !self.objects.iter().any(|o| self.bed_of(o) == self.active_bed);
                let mut go: Option<usize> = None;
                let (mut add, mut remove) = (false, false);
                let area = egui::Area::new(egui::Id::new("bed_overlay"))
                    .order(egui::Order::Foreground)
                    .pivot(egui::Align2::CENTER_TOP)
                    .fixed_pos(egui::pos2(rect.center().x, rect.top() + 10.0))
                    .show(ui.ctx(), |ui| {
                        egui::Frame::popup(ui.style())
                            .fill(egui::Color32::from_rgba_unmultiplied(26, 22, 17, 196))
                            .show(ui, |ui| {
                                // Natural sizing only (Area stale-rect gotcha).
                                ui.horizontal(|ui| {
                                    if ui
                                        .add_enabled(self.active_bed > 0, egui::Button::new("◀"))
                                        .on_hover_text("Previous bed.")
                                        .clicked()
                                    {
                                        go = Some(self.active_bed - 1);
                                    }
                                    ui.label(format!("bed {} / {}", self.active_bed + 1, n));
                                    if ui
                                        .add_enabled(self.active_bed + 1 < n, egui::Button::new("▶"))
                                        .on_hover_text("Next bed.")
                                        .clicked()
                                    {
                                        go = Some(self.active_bed + 1);
                                    }
                                    ui.separator();
                                    if ui
                                        .button("+")
                                        .on_hover_text(
                                            "Add a fresh bed and make it active. Slicing targets \
                                             the active bed, the camera orbits it, and imports \
                                             land on the first empty one.",
                                        )
                                        .clicked()
                                    {
                                        add = true;
                                    }
                                    if ui
                                        .add_enabled(n > 1 && active_empty, egui::Button::new("−"))
                                        .on_hover_text(
                                            "Remove the active bed; beds to the right slide over.",
                                        )
                                        .on_disabled_hover_text(
                                            "Only an empty bed can be removed (and the last bed stays).",
                                        )
                                        .clicked()
                                    {
                                        remove = true;
                                    }
                                });
                            });
                    });
                self.bed_overlay_rect = Some(area.response.rect);
                if let Some(k) = go {
                    self.set_active_bed(k);
                }
                if add {
                    // A new empty bed at the end, made active. It persists
                    // (navigation never removes it) until `−`.
                    self.bed_count += 1;
                    self.set_active_bed(self.bed_count - 1);
                }
                if remove {
                    // Slide everything right of the removed bed one pitch
                    // left; bed indices follow position, so that's the whole
                    // operation. The preview's world offset may now be stale —
                    // invalidate.
                    let bx = self.settings.bed_size_x_mm;
                    let k = self.active_bed;
                    for o in &mut self.objects {
                        if bed_of_pos(o.pos[0], bx) > k {
                            o.pos[0] -= bx + BED_GAP_MM;
                        }
                    }
                    self.bed_count -= 1; // guarded: − only enabled when bed_count > 1
                    self.active_bed = k.min(self.bed_count - 1);
                    self.recenter_camera = true;
                    self.scene_dirty();
                }
            }

            // Vertical layer slider on the right edge of the viewport — drag to
            // set the highest layer shown (lower layers dim). Preview only.
            if self.view_preview && self.sliced.is_some() {
                let n = self.layer_ends.len();
                if n > 0 {
                    egui::Area::new(egui::Id::new("layer_slider"))
                        .order(egui::Order::Foreground)
                        .pivot(egui::Align2::RIGHT_CENTER)
                        .fixed_pos(egui::pos2(rect.right() - 12.0, rect.center().y))
                        .show(ui.ctx(), |ui| {
                            egui::Frame::popup(ui.style())
                                .fill(egui::Color32::from_rgba_unmultiplied(26, 22, 17, 196))
                                .show(ui, |ui| {
                                    ui.vertical_centered(|ui| {
                                        // Slider length keys off the viewport rect (an
                                        // external rect — no Area stale-rect feedback).
                                        ui.spacing_mut().slider_width =
                                            (rect.height() * 0.62).clamp(140.0, 640.0);
                                        ui.add(
                                            egui::Slider::new(&mut self.preview_layer, 1..=n)
                                                .vertical()
                                                .show_value(false),
                                        )
                                        .on_hover_text(
                                            "Highest layer shown; lower layers are dimmed.",
                                        );
                                        ui.label(format!("{} / {}", self.preview_layer, n));
                                    });
                                });
                        });
                }
            }

            // Floating translucent transform panel — only while an object is selected
            // and we're in Model view (Preview is read-only).
            if let (Some(i), false) = (self.selected, self.view_preview) {
                let (bx, by) = (self.settings.bed_size_x_mm, self.settings.bed_size_y_mm);
                let mut changed = false;
                let (mut dup, mut del, mut split) = (false, false, false);
                // Checked before the mutable borrow below (reads the cache).
                let problem = self.obj_problem(i);
                // Tool + blend tables snapshotted before the closure mutably
                // borrows the object (colors + names for the per-part pickers).
                let multi = self.settings.tool_count > 1;
                let tool_opts: Vec<([f32; 3], String)> = (0..self.settings.tool_count)
                    .map(|t| {
                        let ts = self.settings.tool(t);
                        (ts.color_rgb, ts.filament_name.clone())
                    })
                    .collect();
                let blend_opts: Vec<([f32; 3], String)> = (0..self.blends.len())
                    .map(|b| (self.paint_display_rgb(PartColor::Blend(b)), self.blends[b].name.clone()))
                    .collect();
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
                                if let Some(reason) = &problem {
                                    ui.label(
                                        egui::RichText::new(reason)
                                            .color(palette::ERROR)
                                            .strong(),
                                    );
                                }
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
                                ui.horizontal(|ui| {
                                    ui.label("scale");
                                    // DragValue, not a slider: 1%/px drag and
                                    // type-to-set give fine control (the slider
                                    // over 0.1..5 was ~5% per pixel). Matches
                                    // the move/rot fields above.
                                    changed |= ui
                                        .add(
                                            egui::DragValue::new(&mut obj.scale)
                                                .speed(0.01)
                                                .range(0.1..=5.0)
                                                .fixed_decimals(2),
                                        )
                                        .changed();
                                });
                                ui.horizontal(|ui| {
                                    if ui.button("Center").clicked() {
                                        // Center on the object's OWN bed.
                                        let ox = bed_origin_x(bed_of_pos(obj.pos[0], bx), bx);
                                        obj.pos = [ox + bx / 2.0, by / 2.0];
                                        changed = true;
                                    }
                                    if ui.button("Reset rot").clicked() {
                                        obj.rot_deg = [0.0; 3];
                                        changed = true;
                                    }
                                });
                                // Per-part paint assignment (toolchangers) —
                                // the outliner rows, compacted onto the card.
                                if multi {
                                    ui.separator();
                                    for (j, part) in obj.parts.iter_mut().enumerate() {
                                        ui.horizontal(|ui| {
                                            ui.spacing_mut().item_spacing.x = 5.0;
                                            let (dot, _) = ui.allocate_exact_size(
                                                egui::vec2(10.0, 10.0),
                                                egui::Sense::hover(),
                                            );
                                            let c = paint_rgb_from(&tool_opts, &blend_opts, part.paint);
                                            ui.painter().circle_filled(dot.center(), 5.0, rgb32(c));
                                            ui.scope(|ui| {
                                                ui.set_width(96.0);
                                                let name = if part.name.is_empty() {
                                                    format!("part {}", j + 1)
                                                } else {
                                                    part.name.clone()
                                                };
                                                ui.add(egui::Label::new(elide(&name, 14)).truncate());
                                            });
                                            if let Some(p) = paint_combo(
                                                ui,
                                                ("overlay_tool", i, j),
                                                part.paint,
                                                &tool_opts,
                                                &blend_opts,
                                            ) {
                                                part.paint = p;
                                                changed = true;
                                            }
                                        });
                                    }
                                    if ui
                                        .small_button("Split multi-body parts")
                                        .on_hover_text(
                                            "Break any part that is several separate solids (two \
                                             feet exported in one mesh, say) into one part per \
                                             solid, so each can take its own color.",
                                        )
                                        .clicked()
                                    {
                                        split = true;
                                    }
                                }
                                ui.separator();
                                ui.horizontal(|ui| {
                                    if ui
                                        .button("Duplicate")
                                        .on_hover_text(
                                            "Add a copy beside this object (a fresh bed when its bed is full).",
                                        )
                                        .clicked()
                                    {
                                        dup = true;
                                    }
                                    if ui
                                        .button("Delete")
                                        .on_hover_text("Remove this object from the bed.")
                                        .clicked()
                                    {
                                        del = true;
                                    }
                                });
                            });
                    });
                self.overlay_rect = Some(area.response.rect);
                if changed {
                    // Rotation / scale / position edit — geometry+placement moved.
                    self.mark_geom_dirty();
                    self.sliced = None;
                    self.slice_summary = None;
                    self.view_preview = false;
                }
                if dup {
                    self.duplicate_selected();
                }
                if split {
                    self.split_selected_parts();
                }
                if del {
                    self.delete_selected();
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
                let old = self.printer.clone();
                self.printer = p;
                self.filament = f;
                self.process = pr;
                self.switch_printer_loadout(&old);
                self.reresolve();
            }
            if !open {
                self.pending_switch = None;
            }
        }

        // A slot filament pick while that slot's tab carries unsaved (*)
        // edits: confirm the discard — the tier-switch pattern above, scoped
        // to the one slot (the others keep their edits).
        if let Some((slot, name)) = self.pending_slot.clone() {
            let mut act = false;
            let mut open = true;
            egui::Window::new("Discard this tool's unsaved changes?")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, -40.0])
                .show(ui.ctx(), |ui| {
                    ui.label(format!(
                        "Loading '{name}' re-reads slot T{slot} from disk — the tab's \
                         edits marked with * would be lost."
                    ));
                    ui.label("Save them first (💾 on the Filament card), or discard and switch.");
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
                self.apply_slot_switch(slot, name);
            }
            if !open {
                self.pending_slot = None;
            }
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
                    self.refresh_tool0();
                    if let Some(mut layers) = self.sliced.clone() {
                        engine::apply_bead_dabs(&mut layers, &self.engine_dabs(), &self.settings);
                        let gcode = engine::to_gcode(&layers, &self.settings);
                        let filename = self.upload_filename();
                        // A chamber soak waits on the printer's chamber sensor —
                        // confirm it's there before sending, so a missing/misnamed
                        // sensor fails with a clear message instead of aborting
                        // mid-startup on the machine.
                        let chamber_temp = self.settings.chamber_temp_c;
                        let chamber_sensor = self.settings.chamber_sensor.clone();
                        // Tools this job actually prints with, for the
                        // does-the-machine-have-them preflight below.
                        let used = engine::used_tools(&layers);
                        let needs_t_macros = self.settings.toolchange_gcode.contains("T{tool}");
                        self.spawn_host_op(&ctx, false, move |c| {
                            if chamber_temp > 0 {
                                if let Err(e) = c.ensure_chamber_sensor(&chamber_sensor, chamber_temp) {
                                    return HostReply::SendDone { ok: false, msg: e };
                                }
                            }
                            if let Err(e) = c.ensure_tools(&used, needs_t_macros) {
                                return HostReply::SendDone { ok: false, msg: e };
                            }
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
            }
        }

        // Flow calibration: the Filament-panel button armed `start_flow_cal`.
        // Generate the single-wall test from the current settings and send it;
        // the user calipers a wall and applies the result back in the panel.
        if std::mem::take(&mut self.start_flow_cal) {
            let gcode = engine::flow_test_gcode(&self.settings);
            let lw = self.settings.line_width_mm;
            let ctx = ui.ctx().clone();
            self.spawn_host_op(&ctx, false, move |c| {
                match c.upload("flow-cal.gcode", gcode.as_bytes(), true) {
                    Ok(()) => HostReply::SendDone {
                        ok: true,
                        msg: format!("Printing the flow cube — when it's done, caliper a side wall's THICKNESS (mid-height, away from the seam; should read ≈{lw:.2} mm) and enter it in the Filament panel."),
                    },
                    Err(e) => HostReply::SendDone { ok: false, msg: format!("flow-cal upload failed: {e}") },
                }
            });
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
                        // A tab save inherits the SLOT's profile; a tier-row
                        // save inherits the tier selection.
                        let parent = match dlg.slot {
                            Some(i) => self.tools.get(i).unwrap_or(&self.filament),
                            None => match dlg.kind {
                                TierKind::Printer => &self.printer,
                                TierKind::Filament => &self.filament,
                                TierKind::Process => &self.process,
                            },
                        };
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Save as a");
                            ui.label(tier);
                            ui.label(format!(
                                "profile (inherits '{parent}', stores only changed fields):"
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
                    // Slot deletes share the tier fallback: the slot (and any
                    // other naming the vanished profile) re-resolves through
                    // `delete_profile`'s existing paths.
                    self.delete_profile(dlg.kind, &dlg.name)
                } else if let Some(slot) = dlg.slot {
                    self.save_tool_profile(slot, &dlg.name)
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

/// A resolved tool color as an egui color (the GPU side stays f32).
fn rgb32(c: [f32; 3]) -> egui::Color32 {
    egui::Color32::from_rgb(
        (c[0] * 255.0).round() as u8,
        (c[1] * 255.0).round() as u8,
        (c[2] * 255.0).round() as u8,
    )
}

/// Elide to `n` chars with an ellipsis — free-text labels in the fixed-width
/// panel must never overflow it (egui #4475: overflow widens the whole panel).
fn elide(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        s.to_string()
    } else {
        let mut t: String = s.chars().take(n.saturating_sub(1)).collect();
        t.push('…');
        t
    }
}

/// A part name minus a trailing mesh-file extension (case-insensitive) — Bambu/
/// Orca store parts as e.g. "Beak.stl", so the stem is "Beak". Used to spot the
/// merged-object name that just echoes the first part.
fn mesh_name_stem(name: &str) -> &str {
    let lower = name.to_ascii_lowercase();
    for ext in [".stl", ".obj", ".3mf", ".step", ".stp", ".ply", ".gltf", ".glb"] {
        if lower.ends_with(ext) {
            return &name[..name.len() - ext.len()];
        }
    }
    name
}

/// Display color for a paint from the snapshotted option tables (used where
/// the object is mutably borrowed): tool rows clamp to the last slot, a
/// dangling blend index reads the neutral grey.
fn paint_rgb_from(
    tools: &[([f32; 3], String)],
    blends: &[([f32; 3], String)],
    paint: PartColor,
) -> [f32; 3] {
    match paint {
        PartColor::Tool(t) => tools[(t as usize).min(tools.len() - 1)].0,
        PartColor::Blend(b) => blends.get(b).map(|o| o.0).unwrap_or(config::NEUTRAL_FILAMENT_RGB),
    }
}

/// Part-paint picker: compact face ("T{n}", or the blend's name), popup rows
/// carrying every tool slot (color dot + filament name) then every blend
/// (mixed dot + name). Returns the newly chosen paint, if any.
fn paint_combo(
    ui: &mut egui::Ui,
    id: impl std::hash::Hash,
    current: PartColor,
    tools: &[([f32; 3], String)],
    blends: &[([f32; 3], String)],
) -> Option<PartColor> {
    let face = match current {
        PartColor::Tool(t) => format!("T{t}"),
        PartColor::Blend(b) => blends.get(b).map(|(_, n)| elide(n, 8)).unwrap_or_else(|| "?".into()),
    };
    let mut picked = None;
    let mut row = |ui: &mut egui::Ui, rgb: [f32; 3], selected: bool, text: String, paint: PartColor| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 5.0;
            let (dot, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
            ui.painter().circle_filled(dot.center(), 5.0, rgb32(rgb));
            if ui.selectable_label(selected, text).clicked() {
                picked = Some(paint);
            }
        });
    };
    egui::ComboBox::from_id_salt(id)
        .width(56.0)
        .selected_text(face)
        .show_ui(ui, |ui| {
            for (t, (rgb, name)) in tools.iter().enumerate() {
                let paint = PartColor::Tool(t as u32);
                row(ui, *rgb, current == paint, format!("T{t} {}", elide(name, 16)), paint);
            }
            for (b, (rgb, name)) in blends.iter().enumerate() {
                let paint = PartColor::Blend(b);
                row(ui, *rgb, current == paint, elide(name, 16), paint);
            }
        })
        .response
        .on_hover_text("The tool (filament slot) or blend that prints this part.");
    picked
}

fn pattern_combo(ui: &mut egui::Ui, label: &str, current: &mut config::InfillPattern) -> egui::Response {
    use config::InfillPattern::*;
    egui::ComboBox::from_label(label)
        .selected_text(current.label())
        .show_ui(ui, |ui| {
            ui.selectable_value(current, Lines, "lines");
            ui.selectable_value(current, AlignedLines, "aligned lines");
            ui.selectable_value(current, Grid, "grid");
            ui.selectable_value(current, Triangles, "triangles");
            ui.selectable_value(current, Concentric, "concentric");
            ui.selectable_value(current, Gyroid, "gyroid");
        })
        .response
}

/// The printable palette as clickable swatches — THE mix picker, at every
/// spool count (a continuous surface is only honest through three colors,
/// and the band makes the real palette finite anyway, so show it outright).
/// One chip per distinct mixable color, neutral ramp first then hue
/// families dark→light; a ring marks the blend's current recipe.
fn lattice_chips(
    ui: &mut egui::Ui,
    lattice: &[(Vec<u32>, [f32; 3])],
    slot_ids: &[u32],
    current: &[(u32, f32)],
) -> Option<Vec<(u32, f32)>> {
    // Compositions are indexed by the blend's sub-palette; `slot_ids` maps
    // them back to real tool slots. Ratios compare after dividing out common
    // factors, so a hand-dialed 50/50 still rings the 2:2 chip.
    let reduce = |v: Vec<u64>| -> Vec<u64> {
        let g = v.iter().fold(0u64, |a, &b| config_gcd(a, b)).max(1);
        v.into_iter().map(|x| x / g).collect()
    };
    let cur_key: Vec<u64> = {
        let mut v = vec![0u64; slot_ids.len()];
        for &(t, w) in current {
            if let Some(pos) = slot_ids.iter().position(|&s| s == t) {
                if w >= 0.5 {
                    v[pos] = w.round() as u64;
                }
            }
        }
        reduce(v)
    };
    let mut picked = None;
    let mut sel_rgb: Option<[f32; 3]> = None;
    let mut hov_rgb: Option<[f32; 3]> = None;
    ui.horizontal(|ui| {
        ui.add_space(16.0);
        // A fixed, roomy height (about the spool color picker's): the grid fills
        // it and renders from the top; a big palette scrolls in place.
        let grid_w = ui.available_width();
        ui.allocate_ui_with_layout(
            egui::vec2(grid_w, 300.0),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
        egui::ScrollArea::vertical()
            .id_salt("blend_chips")
            .auto_shrink([false, false])
            .show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing = egui::vec2(3.0, 3.0);
            for (comp, rgb) in lattice {
                let (rect, resp) =
                    ui.allocate_exact_size(egui::vec2(13.0, 13.0), egui::Sense::click());
                ui.painter().rect_filled(rect, 2.0, rgb32(*rgb));
                let is_current =
                    reduce(comp.iter().map(|&n| n as u64).collect()) == cur_key;
                if is_current {
                    sel_rgb = Some(*rgb);
                    ui.painter().rect_stroke(
                        rect.expand(1.5),
                        3.0,
                        egui::Stroke::new(1.5, egui::Color32::WHITE),
                        egui::StrokeKind::Outside,
                    );
                }
                let recipe: Vec<String> = comp
                    .iter()
                    .enumerate()
                    .filter(|&(_, &n)| n > 0)
                    .map(|(i, &n)| format!("{n}·T{}", slot_ids[i]))
                    .collect();
                let resp = resp.on_hover_text(format!("{} layers", recipe.join(" + ")));
                if resp.hovered() {
                    hov_rgb = Some(*rgb);
                }
                if resp.clicked() {
                    picked = Some(
                        comp.iter()
                            .enumerate()
                            .filter(|&(_, &n)| n > 0)
                            .map(|(i, &n)| (slot_ids[i], n as f32))
                            .collect(),
                    );
                }
            }
        });
            });
        });
    });
    // Hex readout: the hovered chip's exact printable color, falling back to
    // the selected one. The row is ALWAYS present at the same size — even when
    // nothing resolves — so hovering a chip only swaps the text; it never adds
    // the row and grows the popup, which egui would then reposition, shoving
    // the chips above it (the "chips move on hover" jitter).
    ui.horizontal(|ui| {
        ui.add_space(16.0);
        let (sw, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
        match hov_rgb.or(sel_rgb) {
            Some(rgb) => {
                ui.painter().rect_filled(sw, 2.0, rgb32(rgb));
                ui.monospace(config::hex_color(rgb));
            }
            // No color yet: reserve the identical footprint (7-char hex width
            // and line height) so the layout is byte-for-byte stable.
            None => {
                ui.monospace("       ");
            }
        }
    });
    picked
}

fn config_gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { config_gcd(b, a % b) }
}

/// World-space ray (origin, normalized dir) through a screen position in `rect`.
/// Project a world point to a screen position in `rect`, or None if it's
/// behind the camera. Inverse of `pointer_ray`'s NDC↔screen mapping.
/// Paint the word "Front" — the wordmark's Playfair with the sliced "F",
/// like the logo — lying flat on the bed plane (z=0) just outside the active
/// bed's near (y=0) edge, so it foreshortens with the grid. Real glyphs: lay
/// out a galley, then project each glyph's quad from the bed plane to the
/// screen; the F is split into three offset horizontal slices.
/// The "Front" wordmark laid out once: a flat list of triangle vertices, each
/// (local galley-px (x, y), normalized atlas uv), plus the galley size (tw, th).
/// The caller maps the local px onto the bed plane and uploads it as depth-tested
/// scene geometry, so the model — via the depth buffer — occludes it per-pixel.
fn front_label_local(ui: &mut egui::Ui) -> Option<(Vec<([f32; 2], [f32; 2])>, f32, f32)> {
    let wordmark = egui::FontFamily::Name("wordmark".into());
    // Coverage only; the tint is applied in the shader.
    let galley = ui.ctx().fonts_mut(|f| {
        f.layout_no_wrap("Front".to_owned(), egui::FontId::new(64.0, wordmark), egui::Color32::WHITE)
    });
    let row = galley.rows.first()?;
    let src = &row.visuals.mesh;
    let (tw, th) = (galley.size().x, galley.size().y);
    if src.vertices.len() < 4 || tw <= 0.0 || th <= 0.0 {
        return None;
    }
    // Galley uv is in atlas texels; normalize to [0,1] for a plain texture sample.
    let [aw, ah] = ui.ctx().fonts(|f| f.font_image_size());
    let (aw, ah) = (aw as f32, ah as f32);
    let nuv = |uv: egui::Pos2| [uv.x / aw, uv.y / ah];

    let mut out: Vec<([f32; 2], [f32; 2])> = Vec::new();
    // Two triangles (0,1,2),(0,2,3) for a corner+uv quad.
    let mut quad = |c: [[f32; 2]; 4], uv: [[f32; 2]; 4]| {
        for &i in &[0usize, 1, 2, 0, 2, 3] {
            out.push((c[i], uv[i]));
        }
    };

    // The F (first glyph, first 4 vertices): a px box sliced into three offset
    // bands — the logo's cut F: 0–0.307, 0.360–0.727 (shift +0.083), 0.779–1.0.
    let f = &src.vertices[0..4];
    let (mut px0, mut py0, mut px1, mut py1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    let (mut u0, mut v0, mut u1, mut v1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for vert in f {
        px0 = px0.min(vert.pos.x);
        py0 = py0.min(vert.pos.y);
        px1 = px1.max(vert.pos.x);
        py1 = py1.max(vert.pos.y);
        u0 = u0.min(vert.uv.x);
        v0 = v0.min(vert.uv.y);
        u1 = u1.max(vert.uv.x);
        v1 = v1.max(vert.uv.y);
    }
    let unit = py1 - py0;
    for (fa, fb, shift) in [(0.0_f32, 0.307_f32, 0.0_f32), (0.360, 0.727, 0.083), (0.779, 1.0, 0.0)] {
        let sx = shift * unit;
        let (ya, yb) = (py0 + unit * fa, py0 + unit * fb);
        let (va, vb) = (v0 + (v1 - v0) * fa, v0 + (v1 - v0) * fb);
        quad(
            [[px0 + sx, ya], [px1 + sx, ya], [px1 + sx, yb], [px0 + sx, yb]],
            [nuv(egui::pos2(u0, va)), nuv(egui::pos2(u1, va)), nuv(egui::pos2(u1, vb)), nuv(egui::pos2(u0, vb))],
        );
    }

    // The rest ("ront"): keep each glyph triangle (all vertices past the F's 4).
    for tri in src.indices.chunks_exact(3) {
        if tri.iter().all(|&i| i as usize >= 4) {
            for &i in tri {
                let vert = &src.vertices[i as usize];
                out.push(([vert.pos.x, vert.pos.y], nuv(vert.uv)));
            }
        }
    }

    Some((out, tw, th))
}

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

/// The machine-type picker: a single-nozzle printer plus the three multi-tool
/// swap mechanisms. "Single nozzle" is just `tool_count == 1`; picking a
/// multi-tool mode from one tool bumps the count to 2. Drives both fields so the
/// dropdown reads as the machine's identity.
fn machine_type_combo(
    ui: &mut egui::Ui,
    tool_count: &mut usize,
    machine_kind: &mut config::MachineKind,
) -> egui::Response {
    use config::MachineKind::*;
    #[derive(PartialEq, Clone, Copy)]
    enum T {
        Single,
        Ind,
        SharedNoz,
        SharedHeat,
    }
    let single_text = "Single nozzle (one filament)";
    let mut sel = if *tool_count <= 1 {
        T::Single
    } else {
        match *machine_kind {
            IndependentHotends => T::Ind,
            SharedNozzle => T::SharedNoz,
            SharedHeater => T::SharedHeat,
        }
    };
    let text = match sel {
        T::Single => single_text,
        T::Ind => IndependentHotends.display(),
        T::SharedNoz => SharedNozzle.display(),
        T::SharedHeat => SharedHeater.display(),
    };
    let resp = egui::ComboBox::from_id_salt("machine_type")
        .selected_text(text)
        .width(240.0)
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut sel, T::Single, single_text);
            ui.selectable_value(&mut sel, T::Ind, IndependentHotends.display());
            ui.selectable_value(&mut sel, T::SharedNoz, SharedNozzle.display());
            ui.selectable_value(&mut sel, T::SharedHeat, SharedHeater.display());
        })
        .response;
    match sel {
        T::Single => *tool_count = 1,
        T::Ind => {
            *machine_kind = IndependentHotends;
            *tool_count = (*tool_count).max(2);
        }
        T::SharedNoz => {
            *machine_kind = SharedNozzle;
            *tool_count = (*tool_count).max(2);
        }
        T::SharedHeat => {
            *machine_kind = SharedHeater;
            *tool_count = (*tool_count).max(2);
        }
    }
    resp
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
const CAT_IRONING: f32 = 8.0;
const CAT_SURFACE: f32 = 9.0;

/// Flatten sliced layers into bead instances (one per extrusion/travel segment)
/// plus joint blobs (one per extrusion vertex, to round ends and fill corners),
/// each with a cumulative per-layer count for the layer slider.
/// Bead:  `[p0.xyz, dir.xy, len, width, height, r, g, b, layer, category]`.
/// Joint: `[p.xyz, width, height, r, g, b, layer, category]`.
type Instances = (Vec<[f32; 14]>, Vec<u32>, Vec<[f32; 11]>, Vec<u32>);

/// The active metric mapped to per-path colors — or `None` in feature mode
/// (`build_instances` then colors by path kind). Filament = each path's tool
/// wearing its slot color; layer-time = one ramp color per layer. A free
/// function (no `&self`) so a worker thread can build it during a background
/// slice; `App::layer_color_table` delegates here.
fn build_color_table(
    layers: &[engine::LayerPlan],
    color_by: ColorBy,
    settings: &config::Settings,
    accent: (f32, f32, f32),
    layer_stats: &[engine::LayerStats],
) -> Option<Vec<Vec<[f32; 3]>>> {
    match color_by {
        ColorBy::Feature => None,
        ColorBy::Filament => Some(
            layers
                .iter()
                .map(|layer| {
                    layer
                        .paths
                        .iter()
                        .map(|p| {
                            render::visible_against_backdrop(settings.tool(p.tool as usize).color_rgb)
                        })
                        .collect()
                })
                .collect(),
        ),
        ColorBy::LayerTime => {
            if layer_stats.is_empty() {
                return None;
            }
            let logs: Vec<f64> = layer_stats.iter().map(|st| st.secs.max(1e-12).ln()).collect();
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
                        vec![heat_ramp(u, accent); layer.paths.len()]
                    })
                    .collect(),
            )
        }
    }
}

/// Turn a freshly sliced `layers` into a committable [`SliceOutput`]: the
/// estimate block, the layer-time stats, and the built preview instances. Runs
/// on the UI thread (paint-only re-stamp) or a worker (full re-slice) — no
/// `&self`, no GPU — so it can live entirely off the UI thread.
fn finish_slice(
    layers: Vec<engine::LayerPlan>,
    geo: Option<engine::GeometryPlan>,
    settings: &config::Settings,
    color_by: ColorBy,
    accent: (f32, f32, f32),
    origin_x: f32,
    z_hop: f32,
) -> SliceOutput {
    let n = layers.len();
    let paths: usize = layers.iter().map(|l| l.paths.len()).sum();
    // `per_layer_stats` already sums each layer's Z-move + extrusion + travel —
    // the exact terms `estimate_seconds` recomputes — so derive the total from
    // it instead of re-running the priciest per-path trapezoid math a second
    // time.
    let layer_stats = engine::per_layer_stats(&layers, settings);
    let secs: f64 = layer_stats.iter().map(|s| s.secs).sum();
    let (fil_mm, grams) = engine::estimate_filament(&layers, settings);
    let per_tool = engine::estimate_filament_per_tool(&layers, settings);
    // Tool switches: transitions between consecutive printable paths, across
    // layer boundaries too (where the g-code swaps tools).
    let mut toolchanges = 0usize;
    let mut last_tool: Option<u32> = None;
    for path in layers.iter().flat_map(|l| &l.paths).filter(|p| p.points.len() >= 2) {
        if last_tool.is_some_and(|t| t != path.tool) {
            toolchanges += 1;
        }
        last_tool = Some(path.tool);
    }
    let summary = SliceSummary {
        layers: n,
        toolpaths: paths,
        secs,
        filament_m: fil_mm / 1000.0,
        grams,
        per_tool,
        toolchanges,
    };
    let color_table = build_color_table(&layers, color_by, settings, accent, &layer_stats);
    let (verts, ends, joints, joint_ends) =
        build_instances(&layers, z_hop, color_table.as_deref(), accent, origin_x, None);
    SliceOutput { layers, geo, summary, layer_stats, verts, ends, joints, joint_ends }
}

/// Emit one bead segment `a→b`, subdivided into ≤ `max_seg` mm pieces when set
/// (paint mode) so a brush can target a sub-portion of a long straight run.
#[allow(clippy::too_many_arguments)]
fn push_bead(
    v: &mut Vec<[f32; 14]>,
    origin_x: f32,
    a: geo2d::Point,
    b: geo2d::Point,
    zc: f32,
    w: f32,
    h: f32,
    color: [f32; 3],
    layer: f32,
    cat: f32,
    tool: f32,
    max_seg: Option<f32>,
) {
    let Some(ms) = max_seg else {
        push_inst(v, origin_x, a, b, zc, w, h, color, layer, cat, tool);
        return;
    };
    let len = ((b.x_mm() - a.x_mm()).powi(2) + (b.y_mm() - a.y_mm()).powi(2)).sqrt();
    let nsub = (len / ms.max(0.05) as f64).ceil().max(1.0) as usize;
    if nsub <= 1 {
        push_inst(v, origin_x, a, b, zc, w, h, color, layer, cat, tool);
        return;
    }
    let mut prev = a;
    for si in 1..=nsub {
        let t = si as f64 / nsub as f64;
        let p = geo2d::Point::from_mm(
            a.x_mm() + (b.x_mm() - a.x_mm()) * t,
            a.y_mm() + (b.y_mm() - a.y_mm()) * t,
        );
        push_inst(v, origin_x, prev, p, zc, w, h, color, layer, cat, tool);
        prev = p;
    }
}

fn build_instances(
    layers: &[engine::LayerPlan],
    z_hop_mm: f32,
    path_colors: Option<&[Vec<[f32; 3]>]>,
    accent: (f32, f32, f32),
    origin_x: f32,
    max_seg: Option<f32>,
) -> Instances {
    let mut inst: Vec<[f32; 14]> = Vec::new();
    let mut ends: Vec<u32> = Vec::with_capacity(layers.len());
    let mut joints: Vec<[f32; 11]> = Vec::new();
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
                    // Travels keep their baked accent color in all modes (tool id
                    // unused for CAT_TRAVEL).
                    push_inst(&mut inst, origin_x, from, pt, zc, travel_dim, travel_dim, travel_color, layer_id, CAT_TRAVEL, 0.0);
                    from = pt;
                }
            }
            // Heat-map modes override the feature palette per path (per layer).
            let c = path_colors.map_or_else(|| color_for(path.kind, accent), |t| t[li][pi]);
            let cat = category_of(path.kind);
            // The printing tool — the palette index the Filament-mode shader path
            // uses to recolor this bead without a rebuild.
            let tool = path.tool as f32;
            // Trickle-flow paths (ironing) render as a thin film at the layer top:
            // full width, height scaled by flow.
            let base_h = h * path.height_scale as f32; // height_scale is 1.0 today
            let (w, bh) = if path.flow >= 1.0 {
                ((path.width_mm * path.flow) as f32, base_h)
            } else {
                (path.width_mm as f32, (base_h * path.flow as f32).max(0.04))
            };
            let zc = z_top - bh * 0.5;
            let n_pts = path.points.len();
            // Per-vertex width for a tapering bead (gap fill); else the uniform w.
            let vert_w = |i: usize| -> f32 {
                match &path.widths {
                    Some(ws) => {
                        let wm = ws[i.min(ws.len() - 1)];
                        (if path.flow >= 1.0 { wm * path.flow } else { wm }) as f32
                    }
                    None => w,
                }
            };
            // Per-segment colour + category: a segmented bead (an overhang wall kept
            // whole) shows each stretch in its own feature colour. Heat-map modes stay
            // per-path (path_colors set).
            let seg_cc = |k: usize| -> ([f32; 3], f32) {
                match &path.segs {
                    Some(sa) if path_colors.is_none() && !sa.is_empty() => {
                        let sk = sa[k.min(sa.len() - 1)].kind;
                        (color_for(sk, accent), category_of(sk))
                    }
                    _ => (c, cat),
                }
            };
            for k in 0..n_pts - 1 {
                let sw = (vert_w(k) + vert_w(k + 1)) * 0.5;
                let (sc, scat) = seg_cc(k);
                push_bead(&mut inst, origin_x, path.points[k], path.points[k + 1], zc, sw, bh, sc, layer_id, scat, tool, max_seg);
            }
            if path.closed {
                let (sc, scat) = seg_cc(n_pts - 1);
                push_bead(&mut inst, origin_x, path.points[n_pts - 1], path.points[0], zc, w, bh, sc, layer_id, scat, tool, max_seg);
            }
            // Joint blobs round path ends and fill the outer wedge at CORNERS
            // (extrusion paths only — travels stay bare). A shallow bend's two
            // tube-ends already abut — the gap is a sliver of the bead width, so
            // the blob there is fully hidden under the beads. Emit one only at an
            // open-path cap or a turn sharper than JOINT_MIN_TURN; this cuts the
            // joint count several-fold, the biggest per-frame cost of a dense
            // preview, with no visible change.
            let closed = path.closed;
            for (vi, p) in path.points.iter().enumerate() {
                let keep = if !closed && (vi == 0 || vi == n_pts - 1) {
                    true // open-path cap
                } else {
                    let prev = path.points[(vi + n_pts - 1) % n_pts];
                    let next = path.points[(vi + 1) % n_pts];
                    joint_needed(prev, *p, next)
                };
                if !keep {
                    continue;
                }
                let (sc, scat) = seg_cc(vi);
                joints.push([
                    p.x_mm() as f32 + origin_x, p.y_mm() as f32, zc,
                    vert_w(vi), bh,
                    sc[0], sc[1], sc[2],
                    layer_id, scat, tool,
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
                    s.x_mm() as f32 + origin_x, s.y_mm() as f32, zc,
                    w * 2.5, h * 2.5,
                    seam_color[0], seam_color[1], seam_color[2],
                    layer_id, CAT_SEAM, 0.0,
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

/// A path vertex needs a joint blob only where the tube turns enough that the
/// two segment-ends leave a visible outer wedge. Below `JOINT_MIN_TURN` the
/// ends abut (the gap is a sub-bead-width sliver hidden under the beads), so no
/// blob is needed. `a→b→c` are consecutive path points.
fn joint_needed(a: geo2d::Point, b: geo2d::Point, c: geo2d::Point) -> bool {
    /// Emit a joint above this turn angle. ~22° leaves worst-case gaps under
    /// ~0.1·bead-width — invisible — while culling the many shallow vertices of
    /// a curved wall.
    const JOINT_MIN_COS: f64 = 0.927; // cos(22°)
    let (ix, iy) = (b.x_mm() - a.x_mm(), b.y_mm() - a.y_mm());
    let (ox, oy) = (c.x_mm() - b.x_mm(), c.y_mm() - b.y_mm());
    let li = (ix * ix + iy * iy).sqrt();
    let lo = (ox * ox + oy * oy).sqrt();
    if li < 1.0e-9 || lo < 1.0e-9 {
        return true; // a degenerate/zero-length neighbor — keep the blob
    }
    (ix * ox + iy * oy) / (li * lo) < JOINT_MIN_COS
}

#[allow(clippy::too_many_arguments)]
fn push_inst(
    v: &mut Vec<[f32; 14]>,
    origin_x: f32,
    a: geo2d::Point,
    b: geo2d::Point,
    z_center: f32,
    width: f32,
    height: f32,
    color: [f32; 3],
    layer: f32,
    cat: f32,
    tool: f32,
) {
    let (ax, ay) = (a.x_mm() as f32 + origin_x, a.y_mm() as f32);
    let (bx, by) = (b.x_mm() as f32 + origin_x, b.y_mm() as f32);
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
        layer, cat, tool,
    ]);
}

fn category_of(kind: engine::PathKind) -> f32 {
    use engine::PathKind::*;
    match kind {
        Skirt => CAT_SKIRT,
        ExternalPerimeter | Perimeter | OverhangWall => CAT_WALLS,
        Solid => CAT_SOLID,
        TopSkin | BottomSkin => CAT_SURFACE,
        Infill | GapFill => CAT_INFILL,
        Ironing => CAT_IRONING,
        Support | Bridge | InternalBridge => CAT_SUPPORT,
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
        GapFill => col(-40.0, 0.45, 0.54),     // analogous step the other way — the seam strokes
        Ironing => col(0.0, 0.30, 0.78),       // pale sheen over the top skin
        Support => col(180.0, 0.35, 0.48),     // complement, muted — auxiliary material
        Bridge => col(180.0, 0.55, 0.58),      // complement, brighter — spans over air
        InternalBridge => col(180.0, 0.55, 0.40), // complement, deep — spans over infill
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blend_part_splits_filament_evenly() {
        // A 50/50 blend layer-dithers a part between its two tools — the
        // per-tool filament split lands near even (off by at most ~one
        // layer's worth: odd layer counts and the taller first layer).
        let m = mesh::Mesh::load_stl(std::path::Path::new("../fixtures/cube.stl")).unwrap();
        let mut s = config::Settings::default();
        s.tool_count = 2;
        s.tools = (0..2).map(|i| s.flat_tool(format!("tool-{i}"))).collect();
        let blend = engine::PartPaint::Blend(vec![(0, 1.0), (1, 1.0)]);
        let layers = engine::generate_painted(&[(&m, blend)], &s);
        let per = engine::estimate_filament_per_tool(&layers, &s);
        assert_eq!(per.len(), 2, "both tools print: {per:?}");
        let (a, b) = (per[0].1, per[1].1);
        let share = a / (a + b);
        assert!((share - 0.5).abs() < 0.05, "T0 share {share:.3} (T0 {a:.0} mm, T1 {b:.0} mm)");
    }

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
    fn shelf_packing_fills_a_bed_before_overflowing() {
        // The Gridfinity shape of the problem: 30 small gauges plus one long
        // holder. Uniform worst-case cells needed 8 beds; shelves fit it all
        // on one 350 mm plate.
        let mut sizes: Vec<(f64, f64)> = vec![(48.0, 40.0); 30];
        sizes.push((170.0, 45.0));
        let placed = shelf_pack(&sizes, 350.0, 350.0, 5.0);
        assert!(placed.iter().all(|&(bed, _, _)| bed == 0), "everything on one bed");
        // No two parts overlap (the margin keeps them separated).
        for a in 0..sizes.len() {
            for b in a + 1..sizes.len() {
                let ((wa, ha), (wb, hb)) = (sizes[a], sizes[b]);
                let ((_, ax, ay), (_, bx, by)) = (placed[a], placed[b]);
                let apart =
                    (ax - bx).abs() * 2.0 >= wa + wb || (ay - by).abs() * 2.0 >= ha + hb;
                assert!(apart, "parts {a} and {b} overlap");
            }
        }
        // A flood of 100 mm parts: 9 fit a 350 plate (3 x 3), the rest
        // overflow onto bed 1 — and every part stays inside its plate.
        let many: Vec<(f64, f64)> = vec![(100.0, 100.0); 12];
        let placed = shelf_pack(&many, 350.0, 350.0, 5.0);
        assert_eq!(placed.iter().filter(|p| p.0 == 0).count(), 9);
        assert_eq!(placed.iter().filter(|p| p.0 == 1).count(), 3);
        for (&(w, h), &(_, cx, cy)) in many.iter().zip(&placed) {
            assert!(cx - w / 2.0 >= -1e-9 && cx + w / 2.0 <= 350.0 + 1e-9, "x inside");
            assert!(cy - h / 2.0 >= -1e-9 && cy + h / 2.0 <= 350.0 + 1e-9, "y inside");
        }
    }

    #[test]
    fn bed_world_layout_roundtrips() {
        let bx = 152.4;
        for k in 0..5 {
            let o = bed_origin_x(k, bx);
            assert_eq!(bed_of_pos(o + bx / 2.0, bx), k, "center maps to bed {k}");
            assert_eq!(bed_of_pos(o + 1.0, bx), k, "left edge stays on bed {k}");
            assert_eq!(bed_of_pos(o + bx - 1.0, bx), k, "right edge stays on bed {k}");
        }
        assert_eq!(bed_of_pos(-50.0, bx), 0, "left of bed 0 clamps to 0");
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
