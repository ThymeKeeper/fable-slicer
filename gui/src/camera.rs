//! Orbit camera for the 3D viewport. Z-up (matches the printer: bed = XY plane).

use glam::{Mat4, Vec3};

pub struct Camera {
    pub target: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl Camera {
    pub fn new() -> Self {
        Self { target: Vec3::ZERO, yaw: 0.9, pitch: 0.6, distance: 300.0 }
    }

    pub fn eye(&self) -> Vec3 {
        let cp = self.pitch.cos();
        self.target
            + Vec3::new(
                self.distance * cp * self.yaw.cos(),
                self.distance * cp * self.yaw.sin(),
                self.distance * self.pitch.sin(),
            )
    }

    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let proj = Mat4::perspective_rh(45f32.to_radians(), aspect.max(0.01), 1.0, 20_000.0);
        let view = Mat4::look_at_rh(self.eye(), self.target, Vec3::Z);
        proj * view
    }

    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw -= dx * 0.01;
        self.pitch = (self.pitch + dy * 0.01).clamp(-1.45, 1.45);
    }

    /// `scroll` > 0 zooms in.
    pub fn zoom(&mut self, scroll: f32) {
        self.distance = (self.distance * 0.999_f32.powf(scroll)).clamp(5.0, 8000.0);
    }

    pub fn pan(&mut self, dx: f32, dy: f32) {
        let fwd = (self.target - self.eye()).normalize_or_zero();
        let right = fwd.cross(Vec3::Z).normalize_or_zero();
        let up = right.cross(fwd).normalize_or_zero();
        let s = self.distance * 0.0015;
        self.target += right * (-dx * s) + up * (dy * s);
    }

    pub fn frame(&mut self, center: Vec3, radius: f32) {
        self.target = center;
        self.distance = (radius * 2.5).max(20.0);
    }
}
