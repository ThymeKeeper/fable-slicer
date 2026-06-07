//! `slicer-gui` — desktop GUI: load an STL, choose profiles, slice, preview the
//! model in 3D, and export g-code. (3D toolpath preview is the next increment.)

mod camera;
mod render;

use camera::Camera;
use eframe::egui;
use render::Scene;

use config::{Profiles, Settings};
use engine::generate;

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
    mesh: Option<mesh::Mesh>,
    scene: Scene,
    camera: Camera,
    status: String,
    sliced: Option<Vec<engine::LayerPlan>>,
    /// Cumulative toolpath vertex count after each layer (for the layer slider).
    layer_ends: Vec<u32>,
    /// false = show the model mesh; true = show the sliced toolpaths.
    view_preview: bool,
    /// Highest layer shown in preview (1-based).
    preview_layer: usize,
    needs_rebuild: bool,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let rs = cc
            .wgpu_render_state
            .as_ref()
            .expect("a wgpu render state (run with the wgpu backend)");
        let scene = Scene::new(rs);
        let profiles = Profiles::builtin();
        let (printer, filament, process) =
            ("voron24".to_string(), "pla".to_string(), "standard".to_string());
        let settings = profiles.resolve(&printer, &filament, &process).unwrap_or_default();
        Self {
            profiles,
            printer,
            filament,
            process,
            settings,
            mesh: None,
            scene,
            camera: Camera::new(),
            status: "Open an STL to begin.".to_string(),
            sliced: None,
            layer_ends: Vec::new(),
            view_preview: false,
            preview_layer: 1,
            needs_rebuild: true,
        }
    }

    fn reresolve(&mut self) {
        if let Ok(s) = self.profiles.resolve(&self.printer, &self.filament, &self.process) {
            self.settings = s;
            self.sliced = None;
            self.view_preview = false;
            self.needs_rebuild = true;
        }
    }

    fn load_stl(&mut self, path: std::path::PathBuf) {
        match mesh::Mesh::load_stl(&path) {
            Ok(m) => {
                let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
                self.status = format!("Loaded {name} ({} triangles)", m.triangles.len());
                self.mesh = Some(m);
                self.sliced = None;
                self.view_preview = false;
                self.needs_rebuild = true;
            }
            Err(e) => self.status = format!("Load failed: {e}"),
        }
    }

    fn slice(&mut self, rs: &eframe::egui_wgpu::RenderState) {
        let Some(layers) = self.mesh.as_ref().map(|m| generate(m, &self.settings)) else {
            return;
        };
        let (verts, ends) = build_toolpaths(&layers);
        self.scene.set_toolpaths(&rs.device, &verts);
        let n = layers.len();
        let paths: usize = layers.iter().map(|l| l.paths.len()).sum();
        self.status = format!("Sliced {n} layers, {paths} toolpaths.");
        self.layer_ends = ends;
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

        if let Some(m) = self.mesh.as_ref() {
            let (minx, miny, maxx, maxy) = m.xy_bounds().unwrap_or((0.0, 0.0, 0.0, 0.0));
            let (zmin, zmax) = m.z_bounds().unwrap_or((0.0, 0.0));
            let offset = [
                bx / 2.0 - ((minx + maxx) / 2.0) as f32,
                by / 2.0 - ((miny + maxy) / 2.0) as f32,
                -(zmin as f32),
            ];
            self.scene.set_mesh(&rs.device, m, offset);
            let span = ((maxx - minx).max(maxy - miny).max(zmax - zmin)) as f32;
            self.camera.frame(
                glam::Vec3::new(bx / 2.0, by / 2.0, ((zmax - zmin) / 2.0) as f32),
                span * 0.5 + 1.0,
            );
        } else {
            self.scene.clear_mesh();
            self.camera.frame(glam::Vec3::new(bx / 2.0, by / 2.0, 0.0), bx.max(by) * 0.5);
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

        egui::Panel::left("controls").resizable(false).exact_size(250.0).show_inside(ui, |ui| {
            ui.heading("slicer");
            ui.add_space(4.0);
            if ui.button("Open STL…").clicked() {
                if let Some(path) = rfd::FileDialog::new().add_filter("STL", &["stl"]).pick_file() {
                    self.load_stl(path);
                }
            }
            ui.separator();

            let printers: Vec<String> = self.profiles.printer_names().iter().map(|s| s.to_string()).collect();
            let filaments: Vec<String> = self.profiles.filament_names().iter().map(|s| s.to_string()).collect();
            let processes: Vec<String> = self.profiles.process_names().iter().map(|s| s.to_string()).collect();
            let mut changed = false;
            changed |= combo(ui, "Printer", &mut self.printer, &printers);
            changed |= combo(ui, "Filament", &mut self.filament, &filaments);
            changed |= combo(ui, "Process", &mut self.process, &processes);
            if changed {
                self.reresolve();
            }
            ui.separator();

            ui.add(egui::Slider::new(&mut self.settings.layer_height_mm, 0.05..=0.4).text("layer mm"));
            ui.add(egui::Slider::new(&mut self.settings.first_layer_height_mm, 0.1..=0.4).text("first layer mm"));
            ui.add(egui::Slider::new(&mut self.settings.wall_count, 1..=6).text("walls"));
            ui.add(egui::Slider::new(&mut self.settings.infill_density, 0.0..=1.0).text("infill"));
            ui.add(egui::Slider::new(&mut self.settings.skirt_loops, 0..=5).text("skirt loops"));
            ui.separator();

            ui.horizontal(|ui| {
                if ui.add_enabled(self.mesh.is_some(), egui::Button::new("Slice")).clicked() {
                    self.slice(&rs);
                }
                if ui.add_enabled(self.sliced.is_some(), egui::Button::new("Export g-code…")).clicked() {
                    self.export();
                }
            });
            ui.separator();

            let n_layers = self.sliced.as_ref().map(|l| l.len()).unwrap_or(0);
            if n_layers > 0 {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.view_preview, false, "Model");
                    ui.selectable_value(&mut self.view_preview, true, "Preview");
                });
                if self.view_preview {
                    ui.add(egui::Slider::new(&mut self.preview_layer, 1..=n_layers).text("layer"));
                    ui.label(format!("showing layers 1–{}/{}", self.preview_layer, n_layers));
                }
                ui.separator();
            }

            ui.label(format!(
                "printer {} · bed {:.0}×{:.0} mm",
                self.printer, self.settings.bed_size_x_mm, self.settings.bed_size_y_mm
            ));
            ui.label(&self.status);
            ui.add_space(8.0);
            ui.weak("drag: orbit · right-drag: pan · scroll: zoom");
        });

        egui::CentralPanel::default().show_inside(ui, |ui| {
            let (rect, response) = ui.allocate_exact_size(ui.available_size(), egui::Sense::drag());

            if response.dragged_by(egui::PointerButton::Primary) {
                let d = response.drag_delta();
                self.camera.orbit(d.x, d.y);
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
            let aspect = rect.width() / rect.height().max(1.0);
            let show_mesh = !(self.view_preview && self.sliced.is_some());
            let toolpath_count = if self.view_preview {
                self.layer_ends.get(self.preview_layer.saturating_sub(1)).copied().unwrap_or(0)
            } else {
                0
            };
            self.scene.render(&rs, self.camera.view_proj(aspect), show_mesh, toolpath_count);

            ui.painter().image(
                self.scene.texture_id(),
                rect,
                egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                egui::Color32::WHITE,
            );
        });
    }
}

fn combo(ui: &mut egui::Ui, label: &str, current: &mut String, options: &[String]) -> bool {
    let mut changed = false;
    egui::ComboBox::from_label(label)
        .selected_text(current.clone())
        .show_ui(ui, |ui| {
            for opt in options {
                if ui.selectable_value(current, opt.clone(), opt).changed() {
                    changed = true;
                }
            }
        });
    changed
}

/// Flatten sliced layers into line-segment vertices (`[x,y,z,r,g,b]`, consecutive
/// pairs = segments) plus a cumulative per-layer vertex count for the layer slider.
fn build_toolpaths(layers: &[engine::LayerPlan]) -> (Vec<[f32; 6]>, Vec<u32>) {
    let mut verts: Vec<[f32; 6]> = Vec::new();
    let mut ends: Vec<u32> = Vec::with_capacity(layers.len());
    for layer in layers {
        let z = layer.print_z_mm as f32;
        for path in &layer.paths {
            if path.points.len() < 2 {
                continue;
            }
            let c = color_for(path.kind);
            for w in path.points.windows(2) {
                push_seg(&mut verts, w[0], w[1], z, c);
            }
            if path.closed {
                let last = path.points[path.points.len() - 1];
                push_seg(&mut verts, last, path.points[0], z, c);
            }
        }
        ends.push(verts.len() as u32);
    }
    (verts, ends)
}

fn push_seg(v: &mut Vec<[f32; 6]>, a: geo2d::Point, b: geo2d::Point, z: f32, c: [f32; 3]) {
    v.push([a.x_mm() as f32, a.y_mm() as f32, z, c[0], c[1], c[2]]);
    v.push([b.x_mm() as f32, b.y_mm() as f32, z, c[0], c[1], c[2]]);
}

fn color_for(kind: engine::PathKind) -> [f32; 3] {
    use engine::PathKind::*;
    match kind {
        Skirt => [0.60, 0.60, 0.66],
        ExternalPerimeter => [0.92, 0.34, 0.22],
        Perimeter => [0.36, 0.80, 0.45],
        Solid => [0.94, 0.80, 0.24],
        Infill => [0.32, 0.62, 0.95],
    }
}
