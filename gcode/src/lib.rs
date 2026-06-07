//! Low-level G-code emitter.
//!
//! This crate knows nothing about slicing — it just formats moves and tracks
//! machine state (absolute extruder position + the last feed rate, so `F` is
//! only emitted when it changes). `engine` drives it, computing the extrusion
//! amounts. Extrusion is absolute (`M82` / `G92 E0`), the most compatible
//! default for stock Marlin.

/// Accumulates G-code text while tracking extruder position and feed rate.
#[derive(Debug)]
pub struct GcodeBuilder {
    buf: String,
    e: f64,
    last_feed: Option<f64>,
}

impl GcodeBuilder {
    pub fn new() -> Self {
        Self { buf: String::new(), e: 0.0, last_feed: None }
    }

    pub fn comment(&mut self, text: &str) {
        self.buf.push_str("; ");
        self.buf.push_str(text);
        self.buf.push('\n');
    }

    /// Emit a raw line verbatim (no trailing newline needed).
    pub fn raw(&mut self, line: &str) {
        self.buf.push_str(line);
        self.buf.push('\n');
    }

    /// `G92 E0` — zero the extruder and our running total.
    pub fn reset_extruder(&mut self) {
        self.e = 0.0;
        self.raw("G92 E0");
    }

    /// Returns a ` F{n}` token only when the feed rate has changed.
    fn feed_token(&mut self, feed_mm_min: f64) -> String {
        if self.last_feed.map_or(true, |f| (f - feed_mm_min).abs() > 1.0e-6) {
            self.last_feed = Some(feed_mm_min);
            format!(" F{feed_mm_min:.0}")
        } else {
            String::new()
        }
    }

    /// Rapid travel move (no extrusion).
    pub fn travel(&mut self, x: f64, y: f64, feed_mm_min: f64) {
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G0 X{x:.3} Y{y:.3}{f}\n"));
    }

    /// Extruding move; `e_delta` is the filament length (mm) to add.
    pub fn extrude(&mut self, x: f64, y: f64, e_delta: f64, feed_mm_min: f64) {
        self.e += e_delta;
        let f = self.feed_token(feed_mm_min);
        let e = self.e;
        self.buf.push_str(&format!("G1 X{x:.3} Y{y:.3} E{e:.5}{f}\n"));
    }

    /// Change layer height (Z move only).
    pub fn move_z(&mut self, z: f64, feed_mm_min: f64) {
        let f = self.feed_token(feed_mm_min);
        self.buf.push_str(&format!("G1 Z{z:.3}{f}\n"));
    }

    pub fn set_bed_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M190" } else { "M140" };
        self.raw(&format!("{code} S{celsius}"));
    }

    pub fn set_nozzle_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M109" } else { "M104" };
        self.raw(&format!("{code} S{celsius}"));
    }

    /// Part-cooling fan, 0..=255 (0 turns it off).
    pub fn fan(&mut self, speed: u32) {
        if speed == 0 {
            self.raw("M107");
        } else {
            self.raw(&format!("M106 S{speed}"));
        }
    }

    pub fn finish(self) -> String {
        self.buf
    }
}

impl Default for GcodeBuilder {
    fn default() -> Self {
        Self::new()
    }
}
