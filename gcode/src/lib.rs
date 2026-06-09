//! Low-level G-code emitter.
//!
//! This crate knows nothing about slicing — it just formats moves and tracks
//! machine state (the running filament total + the last feed rate, so `F` is only
//! emitted when it changes). `engine` drives it, computing the extrusion amounts.
//!
//! Extrusion is **relative** (`M83`): each extruding move carries its own delta,
//! and retraction is a plain negative-E move. This is what Klipper recommends, and
//! it avoids unbounded absolute-E growth.

use std::fmt::Write;

/// Accumulates G-code text while tracking the filament total and feed rate.
#[derive(Debug)]
pub struct GcodeBuilder {
    buf: String,
    e_total: f64,
    last_feed: Option<f64>,
}

impl GcodeBuilder {
    pub fn new() -> Self {
        // Pre-size for a medium print; large models grow from here in few steps.
        Self { buf: String::with_capacity(1 << 20), e_total: 0.0, last_feed: None }
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

    /// Total filament consumed so far (mm), retraction included.
    pub fn filament_used_mm(&self) -> f64 {
        self.e_total
    }

    /// Appends a ` F{n}` token only when the feed rate has changed.
    fn push_feed(&mut self, feed_mm_min: f64) {
        if self.last_feed.map_or(true, |f| (f - feed_mm_min).abs() > 1.0e-6) {
            self.last_feed = Some(feed_mm_min);
            let _ = write!(self.buf, " F{feed_mm_min:.0}");
        }
    }

    /// Rapid travel move (no extrusion).
    pub fn travel(&mut self, x: f64, y: f64, feed_mm_min: f64) {
        let _ = write!(self.buf, "G0 X{x:.3} Y{y:.3}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Extruding move; `e_delta` is the filament length (mm) for this segment.
    pub fn extrude(&mut self, x: f64, y: f64, e_delta: f64, feed_mm_min: f64) {
        self.e_total += e_delta;
        let _ = write!(self.buf, "G1 X{x:.3} Y{y:.3} E{e_delta:.5}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Extruding move that also changes Z (spiral-vase ramps Z continuously).
    pub fn extrude_z(&mut self, x: f64, y: f64, z: f64, e_delta: f64, feed_mm_min: f64) {
        self.e_total += e_delta;
        let _ = write!(self.buf, "G1 X{x:.3} Y{y:.3} Z{z:.3} E{e_delta:.5}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Extruding circular arc — `cw` selects G2 (clockwise) vs G3; `i`/`j` are the
    /// arc center offset from the current position; `e_delta` is the filament for
    /// the arc length. Needs firmware arc support (Klipper `[gcode_arcs]`).
    pub fn arc(&mut self, cw: bool, x: f64, y: f64, i: f64, j: f64, e_delta: f64, feed_mm_min: f64) {
        self.e_total += e_delta;
        let code = if cw { "G2" } else { "G3" };
        let _ = write!(self.buf, "{code} X{x:.3} Y{y:.3} I{i:.3} J{j:.3} E{e_delta:.5}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Retract filament by `len` mm.
    pub fn retract(&mut self, len: f64, feed_mm_min: f64) {
        self.e_total -= len;
        let _ = write!(self.buf, "G1 E-{len:.5}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Undo a retraction.
    pub fn unretract(&mut self, len: f64, feed_mm_min: f64) {
        self.e_total += len;
        let _ = write!(self.buf, "G1 E{len:.5}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    /// Change layer height (Z move only).
    pub fn move_z(&mut self, z: f64, feed_mm_min: f64) {
        let _ = write!(self.buf, "G1 Z{z:.3}");
        self.push_feed(feed_mm_min);
        self.buf.push('\n');
    }

    pub fn set_bed_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M190" } else { "M140" };
        let _ = writeln!(self.buf, "{code} S{celsius}");
    }

    pub fn set_nozzle_temp(&mut self, celsius: u32, wait: bool) {
        let code = if wait { "M109" } else { "M104" };
        let _ = writeln!(self.buf, "{code} S{celsius}");
    }

    /// Part-cooling fan, 0..=255 (0 turns it off).
    pub fn fan(&mut self, speed: u32) {
        if speed == 0 {
            self.raw("M107");
        } else {
            let _ = writeln!(self.buf, "M106 S{speed}");
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
