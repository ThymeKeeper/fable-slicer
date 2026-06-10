//! `slicer-gui` — desktop GUI: import STLs as a multi-object scene, lay them out
//! on the bed, choose profiles, slice, preview toolpaths in 3D, and export g-code.

mod camera;
mod render;

use camera::Camera;
use eframe::egui;
use render::Scene;

use config::{FilamentProfile, PrinterProfile, ProcessProfile, Profiles, Settings, Tier, TierKind};
use engine::generate;
use std::sync::Arc;

/// State of the save / delete profile dialog.
struct ProfileDialog {
    kind: TierKind,
    name: String,
    delete: bool,
}

/// Which derivable settings the user has pinned to manual values. Unpinned
/// fields recompute live from their master setting every frame (camera-style
/// "priority mode": auto until touched, visible either way).
#[derive(Default, Clone, Copy)]
struct Pins {
    line_width: bool,
    external_speed: bool,
    solid_speed: bool,
    support_speed: bool,
    gap_fill_speed: bool,
    overhang_speed: bool,
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
    hover: &str,
) {
    ui.horizontal(|ui| {
        let r = ui.add(egui::Slider::new(value, range).text(label));
        if r.changed() {
            *pinned = true;
        }
        r.on_hover_text(hover);
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

/// Accent color per profile tier — used on the selector rows and on every
/// settings-section header, so it's visible at a glance which profile a
/// setting is saved to.
fn tier_color(kind: TierKind) -> egui::Color32 {
    match kind {
        TierKind::Printer => egui::Color32::from_rgb(110, 170, 255), // blue
        TierKind::Filament => egui::Color32::from_rgb(255, 170, 90), // orange
        TierKind::Process => egui::Color32::from_rgb(140, 210, 120), // green
    }
}

/// A collapsible settings section owned by one profile tier: the header is
/// tinted with the tier's color and explains the mapping on hover.
fn tier_section(
    ui: &mut egui::Ui,
    title: &str,
    kind: TierKind,
    default_open: bool,
    add: impl FnOnce(&mut egui::Ui),
) {
    let header = egui::CollapsingHeader::new(
        egui::RichText::new(title).color(tier_color(kind)).strong(),
    )
    .default_open(default_open)
    .show(ui, add);
    header.header_response.on_hover_text(format!(
        "These settings are saved to the {} profile (color-matched in the selector above).",
        kind.label()
    ));
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
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native("slicer", options, Box::new(|cc| Ok(Box::new(App::new(cc)))))
}

struct App {
    profiles: Profiles,
    printer: String,
    filament: String,
    process: String,
    settings: Settings,
    /// Settings as resolved from the selected profiles — panel edits are
    /// compared against this for the per-tier "modified" indicators.
    baseline: Settings,
    /// Open save/delete-profile dialog, if any.
    profile_dialog: Option<ProfileDialog>,
    /// Auto/pinned state of the derivable settings.
    pins: Pins,
    objects: Vec<SceneObject>,
    selected: Option<usize>,
    scene: Scene,
    camera: Camera,
    status: String,
    sliced: Option<Vec<engine::LayerPlan>>,
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
    /// Object being dragged in the viewport (None = orbiting the camera).
    drag_obj: Option<usize>,
    /// Offset (bed XY) between the dragged object's pos and the cursor at grab time.
    drag_grab: [f64; 2],
    /// Screen rect of the transform overlay (so viewport input ignores clicks on it).
    overlay_rect: Option<egui::Rect>,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
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
        let (printer, filament, process) =
            ("voron24".to_string(), "pla".to_string(), "standard".to_string());
        let mut settings = profiles.resolve(&printer, &filament, &process).unwrap_or_default();
        settings.auto_center_on_bed = false; // objects are placed explicitly on the bed
        let baseline = settings.clone();
        let pins = match (profiles.merged_process(&process), profiles.merged_printer(&printer)) {
            (Ok(pc), Ok(pr)) => Pins {
                line_width: pc.line_width_mm.is_some(),
                external_speed: pc.external_perimeter_speed_mm_s.is_some(),
                solid_speed: pc.solid_speed_mm_s.is_some(),
                support_speed: pc.support_speed_mm_s.is_some(),
                gap_fill_speed: pc.gap_fill_speed_mm_s.is_some(),
                overhang_speed: pc.overhang_speed_mm_s.is_some(),
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
            },
            _ => Pins::default(),
        };
        Self {
            profiles,
            printer,
            filament,
            process,
            settings,
            baseline,
            profile_dialog: None,
            pins,
            objects: Vec::new(),
            selected: None,
            scene,
            camera: Camera::new(),
            status,
            sliced: None,
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
            drag_obj: None,
            drag_grab: [0.0, 0.0],
            overlay_rect: None,
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
            self.view_preview = false;
            self.needs_rebuild = true;
            self.refit_camera = true;
        }
        self.refresh_pins();
    }

    /// Pin state comes from the selected profiles: a field the profile chain
    /// sets explicitly is pinned; one it leaves unset follows auto.
    fn refresh_pins(&mut self) {
        if let (Ok(pc), Ok(pr)) =
            (self.profiles.merged_process(&self.process), self.profiles.merged_printer(&self.printer))
        {
            self.pins = Pins {
                line_width: pc.line_width_mm.is_some(),
                external_speed: pc.external_perimeter_speed_mm_s.is_some(),
                solid_speed: pc.solid_speed_mm_s.is_some(),
                support_speed: pc.support_speed_mm_s.is_some(),
                gap_fill_speed: pc.gap_fill_speed_mm_s.is_some(),
                overhang_speed: pc.overhang_speed_mm_s.is_some(),
                outer_wall_accel: pr.outer_wall_accel.is_some(),
                first_layer_accel: pr.first_layer_accel.is_some(),
            };
        }
    }

    /// Recompute every unpinned derivable setting from its master, so dragging
    /// print speed (or changing the nozzle) visibly moves its dependents.
    fn apply_auto(&mut self) {
        let s = &mut self.settings;
        if !self.pins.line_width {
            s.line_width_mm = config::derived_line_width_mm(s.nozzle_diameter_mm);
        }
        if !self.pins.external_speed {
            s.external_perimeter_speed_mm_s =
                config::derived_external_perimeter_speed_mm_s(s.print_speed_mm_s);
        }
        if !self.pins.solid_speed {
            s.solid_speed_mm_s = config::derived_solid_speed_mm_s(s.print_speed_mm_s);
        }
        if !self.pins.support_speed {
            s.support_speed_mm_s = config::derived_support_speed_mm_s(s.print_speed_mm_s);
        }
        if !self.pins.gap_fill_speed {
            s.gap_fill_speed_mm_s = config::derived_gap_fill_speed_mm_s(s.print_speed_mm_s);
        }
        if !self.pins.overhang_speed {
            s.overhang_speed_mm_s = config::derived_overhang_speed_mm_s(s.bridge_speed_mm_s);
        }
        if !self.pins.outer_wall_accel {
            s.outer_wall_accel_mm_s2 = config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2);
        }
        if !self.pins.first_layer_accel {
            s.first_layer_accel_mm_s2 = config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2);
        }
    }

    /// Strip unpinned auto fields from a process diff: auto values are derived,
    /// not chosen, so they're never saved (and never count as dirty).
    fn mask_auto(&self, pc: &mut ProcessProfile) {
        if !self.pins.line_width {
            pc.line_width_mm = None;
        }
        if !self.pins.external_speed {
            pc.external_perimeter_speed_mm_s = None;
        }
        if !self.pins.solid_speed {
            pc.solid_speed_mm_s = None;
        }
        if !self.pins.support_speed {
            pc.support_speed_mm_s = None;
        }
        if !self.pins.gap_fill_speed {
            pc.gap_fill_speed_mm_s = None;
        }
        if !self.pins.overhang_speed {
            pc.overhang_speed_mm_s = None;
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
        let fl = FilamentProfile::diff(&self.settings, &self.baseline);
        let mut pc = ProcessProfile::diff(&self.settings, &self.baseline);
        self.mask_auto(&mut pc);
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
                self.mask_auto(&mut diff);
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
        }
        Ok(())
    }

    /// Load an STL and add it to the scene as a new object.
    fn import_stl(&mut self, path: std::path::PathBuf) {
        match mesh::Mesh::load_stl(&path) {
            Ok(m) => {
                let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_else(|| "object".into());
                self.status = format!("Imported {name} ({} triangles)", m.triangles.len());
                self.objects.push(SceneObject::new(name, m));
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
        // Match the emitter's brick-aware hop height so preview travels line up.
        let hop = if self.settings.brick_layers {
            self.settings.z_hop_mm.max(self.settings.layer_height_mm + 0.25)
        } else {
            self.settings.z_hop_mm
        };
        let (verts, ends, joints, joint_ends) = build_instances(&layers, hop as f32);
        self.scene.set_toolpaths(&rs.device, &verts);
        self.scene.set_joints(&rs.device, &joints);
        let n = layers.len();
        let paths: usize = layers.iter().map(|l| l.paths.len()).sum();
        let secs = engine::estimate_seconds(&layers, &self.settings);
        let (fil_mm, grams) = engine::estimate_filament(&layers, &self.settings);
        self.status = format!(
            "Sliced {n} layers, {paths} toolpaths · ~{} · {:.2} m / {:.0} g",
            engine::format_duration(secs),
            fil_mm / 1000.0,
            grams
        );
        // Loud, not silent: say exactly which features the flow ceiling slowed.
        let clamps = engine::audit_flow_clamps(&layers, &self.settings);
        if !clamps.is_empty() {
            let list: Vec<String> = clamps
                .iter()
                .map(|(k, nom, cl)| format!("{k:?} {nom:.0}→{cl:.0}"))
                .collect();
            self.status += &format!(
                " · ⚠ flow-limited ({:.0} mm³/s): {} mm/s",
                self.settings.max_volumetric_speed_mm3_s,
                list.join(", ")
            );
        }
        self.layer_ends = ends;
        self.joint_layer_ends = joint_ends;
        self.preview_layer = n.max(1);
        self.view_preview = true;
        self.sliced = Some(layers);
    }

    fn export(&mut self) {
        let Some(layers) = self.sliced.as_ref() else { return };
        let Some(path) = rfd::FileDialog::new()
            .add_filter("g-code", &["gcode"])
            .set_file_name("out.gcode")
            .save_file()
        else {
            return;
        };
        let gcode = engine::to_gcode(layers, &self.settings);
        self.status = match std::fs::write(&path, gcode) {
            Ok(()) => format!("Wrote {}", path.display()),
            Err(e) => format!("Write failed: {e}"),
        };
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

        if self.needs_rebuild {
            self.rebuild_scene(&rs);
            self.needs_rebuild = false;
        }
        // Unpinned auto settings track their masters every frame, before
        // anything (incl. the Slice button) reads them.
        self.apply_auto();

        // 320 wide fits the longest slider row (90 slider + value + 19-char
        // label + auto badge ≈ 287). Content wider than the panel doesn't just
        // clip: egui reserves the overflowed width, pushing the central panel
        // right and leaving an unpainted band between the two (egui #4475) —
        // if a future row overflows, that band is the symptom to look for.
        egui::Panel::left("controls").resizable(false).exact_size(320.0).show_inside(ui, |ui| {
            ui.spacing_mut().slider_width = 90.0;
            ui.heading("slicer");
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .button("Import STL…")
                    .on_hover_text("Load an STL file and add it to the bed as a new object.")
                    .clicked()
                {
                    if let Some(path) = rfd::FileDialog::new().add_filter("STL", &["stl"]).pick_file() {
                        self.import_stl(path);
                    }
                }
                if ui
                    .add_enabled(self.selected.is_some(), egui::Button::new("Duplicate"))
                    .on_hover_text("Add a copy of the selected object (shares geometry; re-arranged on the bed).")
                    .clicked()
                {
                    self.duplicate_selected();
                }
                if ui
                    .add_enabled(self.selected.is_some(), egui::Button::new("Delete"))
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
                        let label = egui::RichText::new(format!(
                            "{title}{}",
                            if is_dirty { " *" } else { "" }
                        ))
                        .color(tier_color(kind))
                        .strong();
                        let is_user = self.profiles.is_user(kind, sel);
                        changed |= combo(ui, kind.label(), label, sel, names, hover);
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
                self.reresolve();
            }
            ui.separator();

            // Slice / export + status stay pinned above the scrollable settings.
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(!self.objects.is_empty(), egui::Button::new("Slice"))
                    .on_hover_text("Slice all objects on the bed into toolpaths using the current settings.")
                    .clicked()
                {
                    self.slice(&rs);
                }
                if ui
                    .add_enabled(self.sliced.is_some(), egui::Button::new("Export g-code…"))
                    .on_hover_text("Save the sliced toolpaths to a .gcode file.")
                    .clicked()
                {
                    self.export();
                }
            });
            ui.label(&self.status);
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
            if self.view_preview && n_layers > 0 {
                ui.add(egui::Slider::new(&mut self.preview_layer, 1..=n_layers).text("layer"))
                    .on_hover_text("Highest layer shown; lower layers are dimmed.");
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
            }
            ui.separator();

            // Settings, grouped into collapsible categories (Orca-style) and scrolled.
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                let s = &mut self.settings;
                let pins = &mut self.pins;
                tier_section(ui, "Quality", TierKind::Process, true, |ui| {
                    ui.add(egui::Slider::new(&mut s.layer_height_mm, 0.05..=0.4).text("layer mm"))
                        .on_hover_text("Height of each printed layer. Smaller = finer detail but slower.");
                    ui.add(egui::Slider::new(&mut s.first_layer_height_mm, 0.1..=0.4).text("first layer mm"))
                        .on_hover_text("Thickness of the first layer — often thicker for bed adhesion.");
                    let d_lw = config::derived_line_width_mm(s.nozzle_diameter_mm);
                    auto_slider(ui, &mut s.line_width_mm, 0.2..=1.0, "line width mm", &mut pins.line_width, d_lw,
                        "Width of one extruded line. Auto = 112.5% of the nozzle, so it follows nozzle changes.");
                    ui.add(egui::Slider::new(&mut s.max_resolution_mm, 0.0..=0.5).text("resolution mm"))
                        .on_hover_text("Merge contour points closer than this to drop mesh noise. 0 = off.");
                    seam_combo(ui, &mut s.seam_mode)
                        .on_hover_text("Where each wall loop starts: nearest point, sharpest corner, or random.");
                    ui.checkbox(&mut s.arc_fitting, "arc fitting (G2/G3)")
                        .on_hover_text("Emit curved toolpaths as G2/G3 arcs — smaller g-code, smoother motion. Needs firmware arc support (Klipper [gcode_arcs]).");
                    ui.add_enabled(s.arc_fitting, egui::Slider::new(&mut s.arc_tolerance_mm, 0.005..=0.2).text("arc tol mm"))
                        .on_hover_text("Max deviation a point may have from a fitted arc to be folded into it.");
                    ui.add(egui::Slider::new(&mut s.elephant_foot_mm, 0.0..=0.5).text("elephant foot mm"))
                        .on_hover_text("Shrink the first layer's outline inward to counter first-layer squish. 0 = off.");
                    ui.add(egui::Slider::new(&mut s.xy_compensation_mm, -0.5..=0.5).text("XY comp mm"))
                        .on_hover_text("Grow (+) or shrink (−) every layer's outline for dimensional accuracy. 0 = off.");
                    let vase = s.spiral_vase;
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.ironing, "ironing"))
                        .on_hover_text("Re-traverse top surfaces with a hot nozzle and a trickle of flow to melt them smooth.")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    ui.add_enabled(s.ironing && !vase, egui::Slider::new(&mut s.ironing_flow, 0.0..=0.5).text("ironing flow"))
                        .on_hover_text("Ironing extrusion as a fraction of a normal line at that spacing.");
                    ui.add_enabled(s.ironing && !vase, egui::Slider::new(&mut s.ironing_spacing_mm, 0.05..=0.5).text("ironing spacing mm"))
                        .on_hover_text("Distance between ironing passes — finer is smoother and slower.");
                    ui.add_enabled(s.ironing && !vase, egui::Slider::new(&mut s.ironing_speed_mm_s, 5.0..=100.0).text("ironing mm/s"))
                        .on_hover_text("Ironing pass speed.");
                    ui.add_enabled(!vase, egui::Checkbox::new(&mut s.fuzzy_skin, "fuzzy skin"))
                        .on_hover_text("Jitter the outer wall into a rough, textured surface (hides layer lines).")
                        .on_disabled_hover_text("Forced off in spiral vase mode.");
                    ui.add_enabled(s.fuzzy_skin && !vase, egui::Slider::new(&mut s.fuzzy_skin_thickness_mm, 0.05..=1.0).text("fuzzy thickness mm"))
                        .on_hover_text("Total jitter band, centered on the wall line.");
                    ui.add_enabled(s.fuzzy_skin && !vase, egui::Slider::new(&mut s.fuzzy_skin_point_dist_mm, 0.2..=2.0).text("fuzzy point dist mm"))
                        .on_hover_text("Spacing between jittered points — smaller is noisier.");
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
                    ui.add_enabled(!vase, egui::Slider::new(&mut s.wall_count, 1..=6).text("walls"))
                        .on_hover_text("Number of perimeter loops (shell wall thickness).")
                        .on_disabled_hover_text("Spiral vase forces a single wall.");
                    ui.add_enabled(!vase, egui::Slider::new(&mut s.top_layers, 0..=10).text("top layers"))
                        .on_hover_text("Number of solid layers on top surfaces.")
                        .on_disabled_hover_text("Spiral vase prints no top shells.");
                    ui.add(egui::Slider::new(&mut s.bottom_layers, 0..=10).text("bottom layers"))
                        .on_hover_text("Number of solid layers on bottom surfaces.");
                    ui.add_enabled(!vase && s.wall_mode == config::WallMode::Classic, egui::Checkbox::new(&mut s.gap_fill, "gap fill"))
                        .on_hover_text("Classic mode: fill gaps too thin for walls/infill with single width-matched strokes.")
                        .on_disabled_hover_text("Arachne absorbs gaps into the walls themselves — gap fill only applies to classic mode (and is off in spiral vase).");
                    ui.checkbox(&mut s.monotonic_solid, "monotonic top/bottom")
                        .on_hover_text("Print solid-fill lines in one strict sweep per surface for an even sheen.");
                    ui.add_enabled(!vase && !s.brick_layers, egui::Checkbox::new(&mut s.half_height_outer_walls, "half-height outer wall"))
                        .on_hover_text("Print the outer wall as two half-height passes, each sliced at its own plane — halves the visible Z staircase on slopes while the interior keeps full layer height. Costs roughly the outer-wall print time again.")
                        .on_disabled_hover_text("Unavailable in spiral vase mode or with brick layers (their Z choreographies collide).");
                    ui.add_enabled(!vase && !s.half_height_outer_walls, egui::Checkbox::new(&mut s.brick_layers, "brick layers"))
                        .on_hover_text("Stagger odd perimeters by half a layer height so wall rings interlock like bricks (the outer wall stays put). Best with 3+ walls.")
                        .on_disabled_hover_text("Unavailable in spiral vase mode or with half-height outer walls.");
                    ui.add_enabled(s.brick_layers && !vase, egui::Slider::new(&mut s.brick_flow, 1.0..=1.3).text("brick flow"))
                        .on_hover_text("Extra extrusion on the lifted brick perimeters to fill the diagonal gaps between staggered beads so they mesh solidly.");
                });
                tier_section(ui, "Infill", TierKind::Process, false, |ui| {
                    let vase = s.spiral_vase;
                    ui.add_enabled(!vase, egui::Slider::new(&mut s.infill_density, 0.0..=1.0).text("density"))
                        .on_hover_text("Sparse interior fill density (0 = hollow, 1 = solid).")
                        .on_disabled_hover_text("Spiral vase prints no infill.");
                    ui.add_enabled_ui(s.infill_density > 0.0 && !vase, |ui| {
                        pattern_combo(ui, "sparse fill", &mut s.sparse_pattern)
                            .on_hover_text("Pattern for the sparse interior infill.");
                    });
                    pattern_combo(ui, "solid fill", &mut s.solid_pattern)
                        .on_hover_text("Pattern for the solid top/bottom layers.");
                    ui.add(egui::Slider::new(&mut s.infill_overlap, 0.0..=0.5).text("wall overlap"))
                        .on_hover_text("How far infill pushes into the innermost wall (fraction of a line width) so they bond.");
                });
                tier_section(ui, "Feature speeds", TierKind::Process, false, |ui| {
                    let v = s.print_speed_mm_s;
                    auto_slider(ui, &mut s.external_perimeter_speed_mm_s, 5.0..=200.0, "outer wall mm/s",
                        &mut pins.external_speed, config::derived_external_perimeter_speed_mm_s(v),
                        "Speed for the visible outermost wall — slower is cleaner. Auto = 50% of print speed.");
                    auto_slider(ui, &mut s.solid_speed_mm_s, 5.0..=200.0, "solid mm/s",
                        &mut pins.solid_speed, config::derived_solid_speed_mm_s(v),
                        "Speed for solid top/bottom fill. Auto = 80% of print speed.");
                    auto_slider(ui, &mut s.support_speed_mm_s, 5.0..=200.0, "support mm/s",
                        &mut pins.support_speed, config::derived_support_speed_mm_s(v),
                        "Speed for support structure. Auto = 90% of print speed.");
                    auto_slider(ui, &mut s.gap_fill_speed_mm_s, 5.0..=100.0, "gap fill mm/s",
                        &mut pins.gap_fill_speed, config::derived_gap_fill_speed_mm_s(v),
                        "Speed for thin gap-fill strokes. Auto = 40% of print speed, capped at 40.");
                    ui.add(egui::Slider::new(&mut s.bridge_speed_mm_s, 5.0..=100.0).text("bridge mm/s"))
                        .on_hover_text("Speed for bridges and arc overhangs — slow so beads solidify in air.");
                    auto_slider(ui, &mut s.overhang_speed_mm_s, 5.0..=100.0, "overhang mm/s",
                        &mut pins.overhang_speed, config::derived_overhang_speed_mm_s(s.bridge_speed_mm_s),
                        "Speed for wall stretches hanging past the layer below (printed with bridge cooling). Auto = bridge speed — same physics, beads onto air.");
                    ui.add(egui::Slider::new(&mut s.bridge_flow, 0.7..=1.2).text("bridge flow ×"))
                        .on_hover_text("Flow multiplier on bridges; slight under-extrusion tightens sagging strands.");
                    ui.add(egui::Slider::new(&mut s.min_layer_time_s, 0.0..=30.0).text("min layer s"))
                        .on_hover_text("Cooling slowdown: layers faster than this are slowed so they can solidify.");
                    ui.add_enabled(s.min_layer_time_s > 0.0, egui::Slider::new(&mut s.min_print_speed_mm_s, 5.0..=50.0).text("min mm/s"))
                        .on_hover_text("Floor speed when slowing down to hit the minimum layer time.");
                    ui.weak("machine speed & accel live under Machine & motion");
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
                    ui.add_enabled(has_support, egui::Slider::new(&mut s.support_overhang_angle_deg, 0.0..=80.0).text("overhang °"))
                        .on_hover_text("Steepest overhang (from vertical) printable without support. 45° ≈ one layer-width.");
                    ui.add_enabled(has_support, egui::Slider::new(&mut s.support_density, 0.0..=1.0).text("density"))
                        .on_hover_text("Infill density of grid supports.");
                    ui.add_enabled(has_support, egui::Slider::new(&mut s.support_xy_clearance_mm, 0.0..=2.0).text("xy gap mm"))
                        .on_hover_text("Horizontal gap between support and the model (for easy removal).");
                    ui.add_enabled(has_support, egui::Slider::new(&mut s.support_z_gap_layers, 0..=5).text("z-gap layers"))
                        .on_hover_text("Empty layers between a support top and the part it holds up.");
                    ui.add_enabled(has_support, egui::Slider::new(&mut s.support_interface_layers, 0..=5).text("interface"))
                        .on_hover_text("Dense solid layers at the support top for a smoother overhang underside.");
                    ui.add_enabled(arc, egui::Slider::new(&mut s.max_bridge_span_mm, 0.0..=30.0).text("bridge span mm"))
                        .on_hover_text("Arc mode: gaps narrower than this bridge with straight lines; wider use arcs.");
                    ui.add_enabled(arc, egui::Slider::new(&mut s.max_arc_radius_mm, 5.0..=100.0).text("arc radius mm"))
                        .on_hover_text("Arc mode: max arc-overhang radius before a fan re-seeds.");
                    ui.add_enabled(arc, egui::Slider::new(&mut s.arc_seam_overlap_mm, 0.0..=0.6).text("arc seam overlap mm"))
                        .on_hover_text("Arc mode: how far fans overlap where they meet (per fan). A little helps them mesh; too much over-extrudes the seam. 0 = butt.");
                });
                tier_section(ui, "Bed adhesion", TierKind::Process, false, |ui| {
                    ui.add(egui::Slider::new(&mut s.skirt_loops, 0..=5).text("skirt loops"))
                        .on_hover_text("Loops printed around the first layer to prime the nozzle. 0 = off.");
                    ui.add_enabled(s.skirt_loops > 0, egui::Slider::new(&mut s.skirt_gap_mm, 0.0..=10.0).text("skirt gap mm"))
                        .on_hover_text("Distance from the skirt to the model.");
                    ui.add(egui::Slider::new(&mut s.brim_loops, 0..=20).text("brim loops"))
                        .on_hover_text("Loops attached around the first layer for adhesion. 0 = off.");
                });
                tier_section(ui, "Material & temperature", TierKind::Filament, false, |ui| {
                    ui.add(egui::Slider::new(&mut s.nozzle_temp_c, 150..=300).text("nozzle °C"))
                        .on_hover_text("Hotend temperature.");
                    ui.add(egui::Slider::new(&mut s.bed_temp_c, 0..=120).text("bed °C"))
                        .on_hover_text("Heated bed temperature.");
                    ui.add(egui::Slider::new(&mut s.filament_diameter_mm, 1.0..=3.0).text("filament Ø mm"))
                        .on_hover_text("Filament diameter (1.75 or 2.85). Drives the extrusion math.");
                    ui.add(egui::Slider::new(&mut s.filament_density_g_cm3, 0.8..=2.0).text("density g/cm³"))
                        .on_hover_text("Filament density — used for the weight estimate.");
                    ui.add(egui::Slider::new(&mut s.max_volumetric_speed_mm3_s, 0.0..=80.0).text("max flow mm³/s"))
                        .on_hover_text("The filament's melt-rate ceiling through the hotend. Speeds are clamped so width × height × speed never exceeds it — the status line reports anything that gets slowed. 0 = unlimited.");
                    ui.add(egui::Slider::new(&mut s.extrusion_multiplier, 0.8..=1.2).text("flow ×"))
                        .on_hover_text("Global extrusion multiplier — filament-specific flow tuning.");
                    ui.add(egui::Slider::new(&mut s.pressure_advance, 0.0..=0.2).text("pressure advance"))
                        .on_hover_text("Klipper pressure advance, emitted as SET_PRESSURE_ADVANCE. 0 = leave the printer's value.");
                });
                tier_section(ui, "Cooling", TierKind::Filament, false, |ui| {
                    ui.add(egui::Slider::new(&mut s.fan_speed, 0.0..=1.0).text("fan"))
                        .on_hover_text("Part-cooling fan duty while printing.");
                    ui.add(egui::Slider::new(&mut s.bridge_fan_speed, 0.0..=1.0).text("bridge fan"))
                        .on_hover_text("Fan duty on bridges and arc overhangs — usually maxed.");
                    ui.add(egui::Slider::new(&mut s.fan_off_layers, 0..=5).text("fan off layers"))
                        .on_hover_text("Keep the fan off for this many first layers (bed adhesion).");
                    ui.weak("min-layer-time slowdown lives under Feature speeds");
                });
                tier_section(ui, "Retraction", TierKind::Printer, false, |ui| {
                    ui.add(egui::Slider::new(&mut s.retract_len_mm, 0.0..=10.0).text("length mm"))
                        .on_hover_text("Filament pulled back on travels to prevent oozing/stringing.");
                    ui.add(egui::Slider::new(&mut s.retract_speed_mm_s, 5.0..=100.0).text("speed mm/s"))
                        .on_hover_text("How fast filament is retracted and recovered.");
                    ui.add(egui::Slider::new(&mut s.z_hop_mm, 0.0..=2.0).text("z-hop mm"))
                        .on_hover_text("Lift the nozzle on travels that cross a gap/void. 0 = off.");
                });
                tier_section(ui, "Machine & motion", TierKind::Printer, false, |ui| {
                    ui.add(egui::Slider::new(&mut s.bed_size_x_mm, 50.0..=500.0).text("bed X mm"))
                        .on_hover_text("Bed width (X).");
                    ui.add(egui::Slider::new(&mut s.bed_size_y_mm, 50.0..=500.0).text("bed Y mm"))
                        .on_hover_text("Bed depth (Y).");
                    ui.add(egui::Slider::new(&mut s.bed_size_z_mm, 50.0..=600.0).text("bed Z mm"))
                        .on_hover_text("Maximum build height (Z).");
                    ui.add(egui::Slider::new(&mut s.nozzle_diameter_mm, 0.1..=1.2).text("nozzle mm"))
                        .on_hover_text("Nozzle diameter.");
                    ui.add(egui::Slider::new(&mut s.print_speed_mm_s, 10.0..=400.0).text("print mm/s"))
                        .on_hover_text("The machine's nominal print speed (inner walls, sparse infill). Per-feature speeds derive from it when a profile leaves them unset. Lives here because the printer profile owns it.");
                    ui.add(egui::Slider::new(&mut s.first_layer_speed_mm_s, 5.0..=100.0).text("1st layer mm/s"))
                        .on_hover_text("Speed for the first layer — slower improves bed adhesion.");
                    ui.add(egui::Slider::new(&mut s.travel_speed_mm_s, 20.0..=600.0).text("travel mm/s"))
                        .on_hover_text("Speed for non-printing moves between features.");
                    ui.add(egui::Slider::new(&mut s.acceleration_mm_s2, 100.0..=20000.0).text("accel mm/s²"))
                        .on_hover_text("Acceleration for inner walls, infill, solid, support, and travel — emitted as M204 per feature. Klipper clamps to printer.cfg max_accel. Higher = faster but more ringing.");
                    auto_slider(ui, &mut s.outer_wall_accel_mm_s2, 100.0..=20000.0, "outer accel",
                        &mut pins.outer_wall_accel, config::derived_outer_wall_accel_mm_s2(s.acceleration_mm_s2),
                        "Acceleration for the visible outermost wall — lower hides ringing. Auto = 50% of accel.");
                    auto_slider(ui, &mut s.first_layer_accel_mm_s2, 100.0..=20000.0, "1st layer accel",
                        &mut pins.first_layer_accel, config::derived_first_layer_accel_mm_s2(s.acceleration_mm_s2),
                        "Acceleration for the whole first layer — gentle for bed adhesion. Auto = min(1000, accel).");
                    ui.add(egui::Slider::new(&mut s.jerk_mm_s, 1.0..=50.0).text("jerk mm/s"))
                        .on_hover_text("Klipper square-corner-velocity — how briskly direction changes are taken.");
                });
                tier_section(ui, "Custom g-code", TierKind::Printer, false, |ui| {
                    ui.label("Start g-code").on_hover_text(
                        "Emitted before the print. Placeholders: {nozzle_temp} {bed_temp} {bed_x} {bed_y} {bed_z} {layer_height} {first_layer_height} {nozzle_diameter}.",
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

        // Frameless: the viewport texture runs edge-to-edge against the panel
        // separator instead of sitting in an 8 pt dark mat.
        egui::CentralPanel::default().frame(egui::Frame::NONE).show_inside(ui, |ui| {
            let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let aspect = rect.width() / rect.height().max(1.0);
            let vp = self.camera.view_proj(aspect);

            // Objects are only editable in Model view; Preview is read-only.
            let edit = !self.view_preview;
            // Ignore viewport input when the cursor is over the transform overlay.
            let blocked = match (self.overlay_rect, ui.ctx().pointer_interact_pos()) {
                (Some(r), Some(p)) => r.contains(p),
                _ => false,
            };

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
            if response.hovered() {
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
            self.scene.render(&rs, vp, show_mesh, preview);

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
                            .fill(egui::Color32::from_rgba_unmultiplied(22, 24, 30, 205))
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
                    self.view_preview = false;
                }
            } else {
                self.overlay_rect = None;
            }
        });

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
fn combo(
    ui: &mut egui::Ui,
    id: &str,
    label: egui::RichText,
    current: &mut String,
    options: &[String],
    hover: &str,
) -> bool {
    let mut changed = false;
    let r = egui::ComboBox::from_id_salt(id)
        .selected_text(current.clone())
        .show_ui(ui, |ui| {
            for opt in options {
                if ui.selectable_value(current, opt.clone(), opt).changed() {
                    changed = true;
                }
            }
        });
    r.response.on_hover_text(hover);
    ui.label(label).on_hover_text(hover);
    changed
}

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
fn build_instances(layers: &[engine::LayerPlan], z_hop_mm: f32) -> Instances {
    let mut inst: Vec<[f32; 13]> = Vec::new();
    let mut ends: Vec<u32> = Vec::with_capacity(layers.len());
    let mut joints: Vec<[f32; 10]> = Vec::new();
    let mut joint_ends: Vec<u32> = Vec::with_capacity(layers.len());
    let travel_color = [0.45, 0.75, 0.85];
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
            let c = color_for(path.kind);
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
            // magenta marker, toggleable via the "seams" category.
            if path.kind == engine::PathKind::ExternalPerimeter {
                let s = path.points[0];
                joints.push([
                    s.x_mm() as f32, s.y_mm() as f32, zc,
                    w * 2.5, h * 2.5,
                    1.0, 0.2, 0.85,
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
        Solid => CAT_SOLID,
        Infill => CAT_INFILL,
        GapFill => CAT_GAPFILL,
        Ironing => CAT_IRONING,
        Support | Bridge | InternalBridge => CAT_SUPPORT,
    }
}

fn color_for(kind: engine::PathKind) -> [f32; 3] {
    use engine::PathKind::*;
    match kind {
        Skirt => [0.60, 0.60, 0.66],
        ExternalPerimeter => [0.92, 0.34, 0.22],
        Perimeter => [0.36, 0.80, 0.45],
        // Hot amber: slowed wall stretches hanging past the layer below.
        OverhangWall => [0.98, 0.62, 0.10],
        Solid => [0.94, 0.80, 0.24],
        Infill => [0.32, 0.62, 0.95],
        GapFill => [0.95, 0.45, 0.55],
        Ironing => [0.85, 0.85, 0.55],
        Support => [0.55, 0.40, 0.70],
        Bridge => [0.20, 0.85, 0.85],
        // Deeper teal: solid-over-sparse spans (anchored every infill cell).
        InternalBridge => [0.12, 0.55, 0.75],
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
