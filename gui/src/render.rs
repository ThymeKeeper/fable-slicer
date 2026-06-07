//! wgpu renderer for the 3D viewport.
//!
//! Renders the scene (bed grid + model) into an offscreen color+depth texture,
//! then hands that color texture to egui as a native texture to draw in the
//! central panel. Doing our own pass (rather than egui's callback) gives us a
//! depth buffer for correct 3D occlusion.

use std::borrow::Cow;

use bytemuck::{Pod, Zeroable};
use eframe::egui::TextureId;
use eframe::egui_wgpu::RenderState;
use eframe::wgpu;
use eframe::wgpu::util::DeviceExt;

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

const SHADER: &str = r#"
struct U { mvp: mat4x4<f32>, light: vec4<f32> };
@group(0) @binding(0) var<uniform> u: U;

struct MeshOut { @builtin(position) clip: vec4<f32>, @location(0) normal: vec3<f32> };
@vertex fn vs_mesh(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>) -> MeshOut {
    var o: MeshOut;
    o.clip = u.mvp * vec4<f32>(p, 1.0);
    o.normal = n;
    return o;
}
@fragment fn fs_mesh(i: MeshOut) -> @location(0) vec4<f32> {
    let l = normalize(u.light.xyz);
    let d = max(dot(normalize(i.normal), l), 0.0);
    let base = vec3<f32>(0.30, 0.55, 0.90);
    return vec4<f32>(base * (0.35 + 0.65 * d), 1.0);
}

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
"#;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MeshVertex {
    pos: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct LineVertex {
    pos: [f32; 3],
    color: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    mvp: [[f32; 4]; 4],
    light: [f32; 4],
}

pub struct Scene {
    format: wgpu::TextureFormat,
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    uniform_buf: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    size: (u32, u32),
    color_view: wgpu::TextureView,
    depth_view: wgpu::TextureView,
    tex_id: TextureId,
    mesh_vbuf: Option<wgpu::Buffer>,
    mesh_count: u32,
    line_vbuf: Option<wgpu::Buffer>,
    line_count: u32,
}

impl Scene {
    pub fn new(rs: &RenderState) -> Self {
        let device = &rs.device;
        let format = rs.target_format;

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
            std::mem::size_of::<MeshVertex>() as u64,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
            wgpu::PrimitiveTopology::TriangleList,
        );
        let line_pipeline = make_pipeline(
            device, &layout, &shader, format, "vs_line", "fs_line",
            std::mem::size_of::<LineVertex>() as u64,
            &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
            wgpu::PrimitiveTopology::LineList,
        );

        let (color_view, depth_view) = make_targets(device, format, 1, 1);
        let tex_id = rs.renderer.write().register_native_texture(
            device,
            &color_view,
            wgpu::FilterMode::Linear,
        );

        Self {
            format,
            mesh_pipeline,
            line_pipeline,
            uniform_buf,
            bind_group,
            size: (1, 1),
            color_view,
            depth_view,
            tex_id,
            mesh_vbuf: None,
            mesh_count: 0,
            line_vbuf: None,
            line_count: 0,
        }
    }

    pub fn texture_id(&self) -> TextureId {
        self.tex_id
    }

    pub fn resize(&mut self, rs: &RenderState, w: u32, h: u32) {
        let (w, h) = (w.max(1), h.max(1));
        if self.size == (w, h) {
            return;
        }
        let (color_view, depth_view) = make_targets(&rs.device, self.format, w, h);
        self.color_view = color_view;
        self.depth_view = depth_view;
        self.size = (w, h);
        rs.renderer.write().update_egui_texture_from_wgpu_texture(
            &rs.device,
            &self.color_view,
            wgpu::FilterMode::Linear,
            self.tex_id,
        );
    }

    pub fn clear_mesh(&mut self) {
        self.mesh_vbuf = None;
        self.mesh_count = 0;
    }

    /// Upload a mesh, translated by `offset` (used to center the model on the bed).
    pub fn set_mesh(&mut self, device: &wgpu::Device, mesh: &mesh::Mesh, offset: [f32; 3]) {
        let mut verts = Vec::with_capacity(mesh.triangles.len() * 3);
        for i in 0..mesh.triangles.len() {
            let tri = mesh.triangle(i);
            let p: [[f32; 3]; 3] = [
                [tri[0][0] as f32 + offset[0], tri[0][1] as f32 + offset[1], tri[0][2] as f32 + offset[2]],
                [tri[1][0] as f32 + offset[0], tri[1][1] as f32 + offset[1], tri[1][2] as f32 + offset[2]],
                [tri[2][0] as f32 + offset[0], tri[2][1] as f32 + offset[1], tri[2][2] as f32 + offset[2]],
            ];
            let n = flat_normal(p[0], p[1], p[2]);
            for pos in p {
                verts.push(MeshVertex { pos, normal: n });
            }
        }
        self.mesh_count = verts.len() as u32;
        self.mesh_vbuf = make_vbuf(device, "mesh_vbuf", bytemuck::cast_slice(&verts));
    }

    /// Build the bed grid (gray lines on z=0 plus a brighter border).
    pub fn set_bed(&mut self, device: &wgpu::Device, bed_x: f32, bed_y: f32) {
        let grid = [0.35, 0.35, 0.40];
        let border = [0.65, 0.65, 0.72];
        let step = 20.0_f32;
        let mut v = Vec::new();
        let mut x = 0.0;
        while x <= bed_x + 0.01 {
            v.push(LineVertex { pos: [x, 0.0, 0.0], color: grid });
            v.push(LineVertex { pos: [x, bed_y, 0.0], color: grid });
            x += step;
        }
        let mut y = 0.0;
        while y <= bed_y + 0.01 {
            v.push(LineVertex { pos: [0.0, y, 0.0], color: grid });
            v.push(LineVertex { pos: [bed_x, y, 0.0], color: grid });
            y += step;
        }
        // border
        let corners = [[0.0, 0.0], [bed_x, 0.0], [bed_x, bed_y], [0.0, bed_y]];
        for k in 0..4 {
            let a = corners[k];
            let b = corners[(k + 1) % 4];
            v.push(LineVertex { pos: [a[0], a[1], 0.0], color: border });
            v.push(LineVertex { pos: [b[0], b[1], 0.0], color: border });
        }
        self.line_count = v.len() as u32;
        self.line_vbuf = make_vbuf(device, "bed_vbuf", bytemuck::cast_slice(&v));
    }

    pub fn render(&self, rs: &RenderState, view_proj: glam::Mat4) {
        let uniforms = Uniforms {
            mvp: view_proj.to_cols_array_2d(),
            light: [0.4, 0.5, 0.85, 0.0],
        };
        rs.queue.write_buffer(&self.uniform_buf, 0, bytemuck::bytes_of(&uniforms));

        let mut encoder = rs
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("scene_encoder") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("scene_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.11, g: 0.12, b: 0.14, a: 1.0 }),
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
            if let Some(buf) = &self.line_vbuf {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.line_count, 0..1);
            }
            if let Some(buf) = &self.mesh_vbuf {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.mesh_count, 0..1);
            }
        }
        rs.queue.submit(std::iter::once(encoder.finish()));
    }
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

fn make_targets(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    w: u32,
    h: u32,
) -> (wgpu::TextureView, wgpu::TextureView) {
    let size = wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 };
    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("scene_color"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("scene_depth"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    (
        color.create_view(&wgpu::TextureViewDescriptor::default()),
        depth.create_view(&wgpu::TextureViewDescriptor::default()),
    )
}

#[allow(clippy::too_many_arguments)]
fn make_pipeline(
    device: &wgpu::Device,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    format: wgpu::TextureFormat,
    vs: &str,
    fs: &str,
    stride: u64,
    attrs: &[wgpu::VertexAttribute],
    topology: wgpu::PrimitiveTopology,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("scene_pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some(vs),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: stride,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: attrs,
            }],
            compilation_options: wgpu::PipelineCompilationOptions::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fs),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(wgpu::BlendState::REPLACE),
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
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        },
        multiview_mask: None,
        cache: None,
    })
}
