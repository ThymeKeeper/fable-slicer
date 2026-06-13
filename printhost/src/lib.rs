//! Talk to a printer over Moonraker's HTTP API (the API server every Klipper
//! machine runs — Mainsail and Fluidd are clients of the same endpoints).
//!
//! Blocking by design: callers own their threading (the GUI runs these on a
//! worker thread; the CLI just blocks). Every method returns a plain
//! human-readable `Err(String)` — these surface directly in the status line.

use std::time::Duration;

/// One configured printer connection.
pub struct Client {
    base: String,
    api_key: Option<String>,
    agent: ureq::Agent,
}

/// A snapshot of the printer's print state.
#[derive(Debug, Clone)]
pub struct PrintStatus {
    /// Moonraker print state: standby / printing / paused / complete / error / cancelled.
    pub state: String,
    /// File being printed (empty in standby).
    pub filename: String,
    /// 0.0..=1.0 when printing.
    pub progress: f64,
}

impl Client {
    /// `host` is the printer address — `voron24.local`, `192.168.1.50`, or a
    /// full URL; a missing scheme means plain HTTP (the LAN norm). The API
    /// key is only needed when Moonraker's `[authorization]` requires one.
    pub fn new(host: &str, api_key: &str) -> Client {
        let mut base = host.trim().trim_end_matches('/').to_string();
        if !base.starts_with("http://") && !base.starts_with("https://") {
            base = format!("http://{base}");
        }
        Client {
            base,
            api_key: (!api_key.trim().is_empty()).then(|| api_key.trim().to_string()),
            agent: ureq::AgentBuilder::new()
                .timeout_connect(Duration::from_secs(4))
                // Moonraker's gcode/script endpoint blocks until the script
                // finishes — homing during thermal calibration can take a
                // while, and big uploads on slow networks too.
                .timeout(Duration::from_secs(120))
                .build(),
        }
    }

    fn request(&self, method: &str, path: &str) -> ureq::Request {
        let req = self.agent.request(method, &format!("{}{path}", self.base));
        match &self.api_key {
            Some(k) => req.set("X-Api-Key", k),
            None => req,
        }
    }

    fn call(&self, method: &str, path: &str) -> Result<serde_json::Value, String> {
        let resp = self.request(method, path).call().map_err(err_str)?;
        resp.into_json().map_err(|e| format!("bad response: {e}"))
    }

    /// Connectivity + Klipper readiness check.
    pub fn server_info(&self) -> Result<String, String> {
        let v = self.call("GET", "/server/info")?;
        let state = v["result"]["klippy_state"].as_str().unwrap_or("unknown");
        Ok(state.to_string())
    }

    /// Upload `gcode` as `filename` into the printer's g-code storage,
    /// optionally starting the print immediately.
    pub fn upload(&self, filename: &str, gcode: &[u8], start: bool) -> Result<(), String> {
        let boundary = "----slicer-boundary-7MA4YWxkTrZu0gW";
        let body = multipart_body(boundary, filename, gcode, start);
        self.request("POST", "/server/files/upload")
            .set("Content-Type", &format!("multipart/form-data; boundary={boundary}"))
            .send_bytes(&body)
            .map_err(err_str)?;
        Ok(())
    }

    /// Start printing an already-uploaded file.
    pub fn start_print(&self, filename: &str) -> Result<(), String> {
        let encoded = urlencode(filename);
        self.call("POST", &format!("/printer/print/start?filename={encoded}"))?;
        Ok(())
    }

    pub fn pause(&self) -> Result<(), String> {
        self.call("POST", "/printer/print/pause").map(|_| ())
    }

    pub fn resume(&self) -> Result<(), String> {
        self.call("POST", "/printer/print/resume").map(|_| ())
    }

    pub fn cancel(&self) -> Result<(), String> {
        self.call("POST", "/printer/print/cancel").map(|_| ())
    }

    /// Current print state / file / progress.
    pub fn print_status(&self) -> Result<PrintStatus, String> {
        let v = self.call("GET", "/printer/objects/query?print_stats&virtual_sdcard")?;
        let status = &v["result"]["status"];
        Ok(PrintStatus {
            state: status["print_stats"]["state"].as_str().unwrap_or("unknown").to_string(),
            filename: status["print_stats"]["filename"].as_str().unwrap_or("").to_string(),
            progress: status["virtual_sdcard"]["progress"].as_f64().unwrap_or(0.0),
        })
    }

    /// Current extruder temperature and target (°C).
    pub fn extruder_temp(&self) -> Result<(f64, f64), String> {
        let v = self.call("GET", "/printer/objects/query?extruder")?;
        let e = &v["result"]["status"]["extruder"];
        match e["temperature"].as_f64() {
            Some(t) => Ok((t, e["target"].as_f64().unwrap_or(0.0))),
            None => Err("no extruder temperature in the response".into()),
        }
    }

    /// Run one g-code command on the printer.
    pub fn run_gcode(&self, script: &str) -> Result<(), String> {
        self.call("POST", &format!("/printer/gcode/script?script={}", urlencode(script)))
            .map(|_| ())
    }

    /// Which axes are homed, as Klipper reports them ("", "xy", "xyz", …).
    pub fn homed_axes(&self) -> Result<String, String> {
        let v = self.call("GET", "/printer/objects/query?toolhead")?;
        Ok(v["result"]["status"]["toolhead"]["homed_axes"].as_str().unwrap_or("").to_string())
    }

    /// Bed temperature and target, or `None` when the printer has no heated bed.
    pub fn bed_temp(&self) -> Result<Option<(f64, f64)>, String> {
        let v = self.call("GET", "/printer/objects/query?heater_bed")?;
        let b = &v["result"]["status"]["heater_bed"];
        Ok(b["temperature"].as_f64().map(|t| (t, b["target"].as_f64().unwrap_or(0.0))))
    }

    /// Set the extruder target without waiting (M104).
    pub fn set_extruder_temp(&self, c: f64) -> Result<(), String> {
        self.run_gcode(&format!("M104 S{c:.0}"))
    }

    /// Every Klipper config object Moonraker exposes, by name — e.g.
    /// `"temperature_sensor chamber_temp"`, `"heater_bed"`, `"extruder"`.
    pub fn objects(&self) -> Result<Vec<String>, String> {
        let v = self.call("GET", "/printer/objects/list")?;
        let arr = v["result"]["objects"].as_array().ok_or("no objects list in the response")?;
        Ok(arr.iter().filter_map(|o| o.as_str().map(str::to_string)).collect())
    }

    /// Pre-flight for a chamber soak: a slice that soaks the chamber waits on
    /// `temperature_sensor <name>`, and Klipper aborts the print if that object
    /// isn't configured. Calling this before upload turns that late, cryptic
    /// abort into a clear message up front. `sensor` is the bare Klipper name
    /// (e.g. `"chamber_temp"`) from the printer profile; empty means the profile
    /// declares none. Only call it when the slice actually soaks (`soak_c > 0`).
    pub fn ensure_chamber_sensor(&self, sensor: &str, soak_c: u32) -> Result<(), String> {
        let sensor = sensor.trim();
        if sensor.is_empty() {
            return Err(format!(
                "This slice soaks the chamber to {soak_c} °C, but the printer profile names no \
                 chamber sensor — the print would abort at the soak. Set the filament's chamber \
                 soak to 0, or declare the sensor (Machine & motion → chamber sensor)."
            ));
        }
        let want = format!("temperature_sensor {sensor}");
        let objects = self.objects()?;
        if objects.iter().any(|o| o == &want) {
            return Ok(());
        }
        // List the chamber-ish objects the machine *does* expose, to make the
        // fix obvious (wrong name? wired as a temperature_fan?).
        let candidates: Vec<&str> = objects
            .iter()
            .filter(|o| {
                o.starts_with("temperature_sensor ")
                    || o.starts_with("temperature_fan ")
                    || o.starts_with("heater_generic ")
            })
            .map(String::as_str)
            .collect();
        Err(format!(
            "This slice soaks the chamber to {soak_c} °C and waits on [{want}], but the printer \
             has no such object — the print would abort at the soak. Sensors it does expose: {}. \
             Fix the printer profile's chamber sensor name, or set the filament's chamber soak to 0.",
            if candidates.is_empty() { "(none)".to_string() } else { candidates.join(", ") }
        ))
    }
}

/// The hotend's measured thermal response near printing temperature, with the
/// part fan off and at 100% — the fan's spillover both steals heater power
/// (slower heating) and strips the block (faster cooling), and it runs for
/// essentially every layer the temp schedule touches. Heating is driven by
/// the full cartridge; cooling is passive. (Heat removed by flowing filament
/// is not captured here; the scheduler's leads stay conservative because of it.)
#[derive(Debug, Clone, Copy)]
pub struct ThermalRates {
    pub heat_rate_c_s: f64,
    pub cool_rate_c_s: f64,
    pub heat_rate_fan_c_s: f64,
    pub cool_rate_fan_c_s: f64,
}

/// A controlled environment for the measurement. The rates persist into the
/// printer profile, so two runs must agree — and the hotend's response is
/// position-dependent (bed proximity slows passive cooling and reflects fan
/// spillover back onto the block). Pinning pose and bed temperature makes
/// the measurement repeatable and print-representative.
pub struct CalibrationSetup {
    /// Where to park for the measurement: bed-center XY, a clearance Z high
    /// enough to clear anything left on the plate.
    pub park_xyz: (f64, f64, f64),
    /// Bed target (°C) during the measurement; 0 leaves the bed alone.
    pub bed_c: f64,
}

/// Profile the hotend once: park in a consistent pose, then step the idle
/// printer's nozzle target `base → base+step → cool-off`, fan off and at
/// 100%, and fit the reported temperatures. Blocking and slow (minutes —
/// passive cooling dominates), so run it on a worker thread; `progress`
/// receives one-line updates for the status line. Refuses to run while a
/// print is active; heater, fan, and bed are shut off on every exit path.
pub fn measure_thermal_rates(
    client: &Client,
    base_c: f64,
    step_c: f64,
    setup: &CalibrationSetup,
    progress: &mut dyn FnMut(String),
) -> Result<ThermalRates, String> {
    let st = client.print_status()?;
    if st.state == "printing" || st.state == "paused" {
        return Err(format!("printer is {} — calibration needs it idle", st.state));
    }
    let res = measure_inner(client, base_c, step_c, setup, progress);
    // Never leave anything running, success or failure.
    let _ = client.set_extruder_temp(0.0);
    let _ = client.run_gcode("M107");
    if setup.bed_c > 0.0 {
        let _ = client.run_gcode("M140 S0");
    }
    if res.is_ok() {
        progress(
            "Thermal calibration done — heater, fans, and bed off. (The hotend's own \
             heatbreak fan keeps spinning until the nozzle cools below ~50 °C; that's \
             Klipper's heat-creep protection stopping itself, not a leftover.)"
                .into(),
        );
    }
    res
}

fn measure_inner(
    client: &Client,
    base_c: f64,
    step_c: f64,
    setup: &CalibrationSetup,
    progress: &mut dyn FnMut(String),
) -> Result<ThermalRates, String> {
    let top = base_c + step_c;
    let floor = base_c - 0.2 * step_c;
    // Fit windows: the middle of climbs (skips the PID's launch and approach
    // tails) and the early stretch of descents (the near-print-temp rate,
    // before the exponential flattens).
    let (climb_lo, climb_hi) = (base_c + 0.1 * step_c, base_c + 0.8 * step_c);
    let (desc_lo, desc_hi) = (top - 0.6 * step_c, top - 0.05 * step_c);

    // Pin the environment first: bed heating in parallel, a consistent pose.
    if setup.bed_c > 0.0 {
        client.run_gcode(&format!("M140 S{:.0}", setup.bed_c))?;
    }
    if !client.homed_axes()?.contains('z') {
        progress("Thermal calibration: homing first (clear the bed!)…".into());
        client.run_gcode("G28")?;
    }
    let (px, py, pz) = setup.park_xyz;
    progress("Thermal calibration: parking over bed center…".into());
    client.run_gcode("G90")?;
    client.run_gcode(&format!("G1 X{px:.1} Y{py:.1} Z{pz:.1} F3000"))?;

    progress(format!("Thermal calibration: settling at {base_c:.0} °C, fan off…"));
    client.run_gcode("M107")?; // a known fan state
    client.set_extruder_temp(base_c)?;
    sample_toward(client, base_c, 300.0, progress)?;
    if setup.bed_c > 0.0 {
        wait_bed_near(client, setup.bed_c, 360.0, progress)?;
    }
    std::thread::sleep(Duration::from_secs(8)); // let the PID flatten out

    progress(format!("Thermal calibration 1/4: heating to {top:.0} °C, fan off…"));
    client.set_extruder_temp(top)?;
    let trace = sample_toward(client, top, 240.0, progress)?;
    let heat_off = fit_slope(&trace, climb_lo, climb_hi)
        .ok_or("could not fit the fan-off heating slope")?;
    std::thread::sleep(Duration::from_secs(8));

    progress("Thermal calibration 2/4: cooling with the part fan at 100% — the in-print case…".into());
    client.run_gcode("M106 S255")?;
    client.set_extruder_temp((base_c - step_c).max(0.0))?; // heater idles while above target
    let trace = sample_until_below(client, floor, 900.0, progress)?;
    let cool_fan = fit_slope(&trace, desc_lo, desc_hi)
        .map(f64::abs)
        .ok_or("could not fit the fan-on cooling slope")?;

    progress(format!("Thermal calibration 3/4: heating back to {top:.0} °C with the fan on…"));
    client.set_extruder_temp(top)?;
    let trace = sample_toward(client, top, 300.0, progress)?;
    let heat_fan = fit_slope(&trace, climb_lo, climb_hi)
        .ok_or("could not fit the fan-on heating slope")?;
    std::thread::sleep(Duration::from_secs(8));

    progress("Thermal calibration 4/4: fan off, passive cooling — the slow half, hang tight…".into());
    client.run_gcode("M107")?;
    client.set_extruder_temp((base_c - step_c).max(0.0))?;
    let trace = sample_until_below(client, floor, 900.0, progress)?;
    let cool_off = fit_slope(&trace, desc_lo, desc_hi)
        .map(f64::abs)
        .ok_or("could not fit the fan-off cooling slope")?;

    Ok(ThermalRates {
        heat_rate_c_s: heat_off.abs().clamp(0.1, 20.0),
        cool_rate_c_s: cool_off.clamp(0.05, 10.0),
        heat_rate_fan_c_s: heat_fan.abs().clamp(0.05, 20.0),
        cool_rate_fan_c_s: cool_fan.clamp(0.05, 20.0),
    })
}

/// Wait until the bed sits within 3 °C of `target` (it heats in parallel with
/// the earlier phases, so this is usually already true). Printers without a
/// heated bed pass straight through.
fn wait_bed_near(
    client: &Client,
    target: f64,
    timeout_s: f64,
    progress: &mut dyn FnMut(String),
) -> Result<(), String> {
    let t0 = std::time::Instant::now();
    loop {
        let Some((temp, _)) = client.bed_temp()? else { return Ok(()) };
        if (temp - target).abs() <= 3.0 {
            return Ok(());
        }
        if t0.elapsed().as_secs_f64() > timeout_s {
            return Err(format!("bed never reached {target:.0} °C (at {temp:.1})"));
        }
        progress(format!("Thermal calibration: waiting for the bed, {temp:.1} °C → {target:.0} °C…"));
        std::thread::sleep(Duration::from_secs(3));
    }
}

/// Sample ~4 Hz until the reading is within 1.5 °C of `target`, returning the
/// `(seconds, °C)` trace. Errors after `timeout_s`.
fn sample_toward(
    client: &Client,
    target: f64,
    timeout_s: f64,
    progress: &mut dyn FnMut(String),
) -> Result<Vec<(f64, f64)>, String> {
    let t0 = std::time::Instant::now();
    let mut trace = Vec::new();
    loop {
        let (temp, _) = client.extruder_temp()?;
        let t = t0.elapsed().as_secs_f64();
        trace.push((t, temp));
        if (temp - target).abs() <= 1.5 {
            return Ok(trace);
        }
        if t > timeout_s {
            return Err(format!("timed out heading for {target:.0} °C (stuck at {temp:.1})"));
        }
        if trace.len() % 20 == 0 {
            progress(format!("Thermal calibration: {temp:.1} °C → {target:.0} °C…"));
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// Sample ~4 Hz until the reading falls to `threshold` or below.
fn sample_until_below(
    client: &Client,
    threshold: f64,
    timeout_s: f64,
    progress: &mut dyn FnMut(String),
) -> Result<Vec<(f64, f64)>, String> {
    let t0 = std::time::Instant::now();
    let mut trace = Vec::new();
    loop {
        let (temp, _) = client.extruder_temp()?;
        let t = t0.elapsed().as_secs_f64();
        trace.push((t, temp));
        if temp <= threshold {
            return Ok(trace);
        }
        if t > timeout_s {
            return Err(format!("timed out cooling to {threshold:.0} °C (still {temp:.1})"));
        }
        if trace.len() % 20 == 0 {
            progress(format!("Thermal calibration: cooling, {temp:.1} °C…"));
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// Least-squares slope (°C/s) over the samples whose temperature lies inside
/// `[lo, hi]`. None when the window holds too little signal to trust.
fn fit_slope(trace: &[(f64, f64)], lo: f64, hi: f64) -> Option<f64> {
    let pts: Vec<(f64, f64)> = trace.iter().copied().filter(|&(_, c)| c >= lo && c <= hi).collect();
    if pts.len() < 5 || pts.last()?.0 - pts.first()?.0 < 1.0 {
        return None;
    }
    let n = pts.len() as f64;
    let sx: f64 = pts.iter().map(|p| p.0).sum();
    let sy: f64 = pts.iter().map(|p| p.1).sum();
    let sxy: f64 = pts.iter().map(|p| p.0 * p.1).sum();
    let sxx: f64 = pts.iter().map(|p| p.0 * p.0).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-12 {
        return None;
    }
    Some((n * sxy - sx * sy) / denom)
}

/// Compact, status-line-friendly error text.
fn err_str(e: ureq::Error) -> String {
    match e {
        ureq::Error::Status(code, resp) => {
            let body = resp.into_string().unwrap_or_default();
            // Moonraker errors carry {"error": {"message": ...}}.
            let msg = serde_json::from_str::<serde_json::Value>(&body)
                .ok()
                .and_then(|v| v["error"]["message"].as_str().map(str::to_string))
                .unwrap_or(body);
            format!("HTTP {code}: {}", msg.chars().take(120).collect::<String>())
        }
        ureq::Error::Transport(t) => format!("{t}"),
    }
}

/// multipart/form-data body for Moonraker's upload endpoint: the file part
/// plus a `print` field when the print should start right away.
fn multipart_body(boundary: &str, filename: &str, gcode: &[u8], start: bool) -> Vec<u8> {
    let mut body = Vec::with_capacity(gcode.len() + 512);
    if start {
        body.extend_from_slice(
            format!("--{boundary}\r\nContent-Disposition: form-data; name=\"print\"\r\n\r\ntrue\r\n")
                .as_bytes(),
        );
    }
    body.extend_from_slice(
        format!(
            "--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"{filename}\"\r\n\
             Content-Type: text/x-gcode\r\n\r\n"
        )
        .as_bytes(),
    );
    body.extend_from_slice(gcode);
    body.extend_from_slice(format!("\r\n--{boundary}--\r\n").as_bytes());
    body
}

fn urlencode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(b as char),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpListener;

    /// One-shot HTTP server: accepts a single request, captures it fully,
    /// answers 200 with the given JSON.
    fn one_shot(response: &'static str) -> (String, std::thread::JoinHandle<String>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let handle = std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            let mut buf = Vec::new();
            let mut tmp = [0u8; 4096];
            // Read headers, then the declared body length.
            let header_end;
            loop {
                let n = sock.read(&mut tmp).unwrap();
                buf.extend_from_slice(&tmp[..n]);
                if let Some(pos) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
                    header_end = pos + 4;
                    break;
                }
            }
            let headers = String::from_utf8_lossy(&buf[..header_end]).to_string();
            let content_length: usize = headers
                .lines()
                .find_map(|l| l.to_ascii_lowercase().strip_prefix("content-length:").map(|v| v.trim().parse().unwrap()))
                .unwrap_or(0);
            while buf.len() < header_end + content_length {
                let n = sock.read(&mut tmp).unwrap();
                if n == 0 {
                    break;
                }
                buf.extend_from_slice(&tmp[..n]);
            }
            let reply = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{response}",
                response.len()
            );
            sock.write_all(reply.as_bytes()).unwrap();
            String::from_utf8_lossy(&buf).to_string()
        });
        (format!("127.0.0.1:{}", addr.port()), handle)
    }

    #[test]
    fn upload_builds_a_correct_multipart_request() {
        let (addr, server) = one_shot("{\"result\": \"ok\"}");
        let client = Client::new(&addr, "secret-key");
        client.upload("benchy.gcode", b"G28\nG1 X10\n", true).expect("upload ok");
        let req = server.join().unwrap();
        assert!(req.starts_with("POST /server/files/upload"), "got: {}", &req[..60]);
        assert!(req.contains("X-Api-Key: secret-key") || req.contains("x-api-key: secret-key"));
        assert!(req.contains("name=\"file\"; filename=\"benchy.gcode\""));
        assert!(req.contains("G28\nG1 X10\n"));
        assert!(req.contains("name=\"print\"") && req.contains("true"), "starts the print");
    }

    #[test]
    fn status_and_info_parse() {
        let (addr, server) = one_shot("{\"result\": {\"klippy_state\": \"ready\"}}");
        let client = Client::new(&format!("http://{addr}/"), "");
        assert_eq!(client.server_info().unwrap(), "ready");
        let req = server.join().unwrap();
        assert!(req.starts_with("GET /server/info"));
        assert!(!req.to_ascii_lowercase().contains("x-api-key"), "no key header when unset");

        let (addr, server) = one_shot(
            "{\"result\": {\"status\": {\"print_stats\": {\"state\": \"printing\", \"filename\": \"a.gcode\"}, \"virtual_sdcard\": {\"progress\": 0.42}}}}",
        );
        let client = Client::new(&addr, "");
        let st = client.print_status().unwrap();
        assert_eq!(st.state, "printing");
        assert_eq!(st.filename, "a.gcode");
        assert!((st.progress - 0.42).abs() < 1e-9);
        server.join().unwrap();
    }

    #[test]
    fn slope_fit_recovers_rates() {
        // A clean 3 °C/s climb sampled at 4 Hz.
        let up: Vec<(f64, f64)> =
            (0..28).map(|i| (i as f64 * 0.25, 200.0 + 3.0 * i as f64 * 0.25)).collect();
        let s = fit_slope(&up, 202.0, 216.0).unwrap();
        assert!((s - 3.0).abs() < 1e-9, "{s}");
        // Exponential decay: the early-window fit is the near-print-temp rate.
        let down: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.25;
                (t, 180.0 + 40.0 * (-t / 30.0).exp())
            })
            .collect();
        let s = fit_slope(&down, 196.0, 218.0).unwrap();
        assert!(s < 0.0 && s.abs() > 0.6 && s.abs() < 1.4, "{s}");
        // Too little signal → no fit, not a junk number.
        assert!(fit_slope(&up[..3], 0.0, 999.0).is_none());
    }

    #[test]
    fn extruder_temp_parses() {
        let (addr, server) = one_shot(
            "{\"result\": {\"status\": {\"extruder\": {\"temperature\": 209.6, \"target\": 210.0}}}}",
        );
        let client = Client::new(&addr, "");
        let (temp, target) = client.extruder_temp().unwrap();
        assert!((temp - 209.6).abs() < 1e-9 && (target - 210.0).abs() < 1e-9);
        let req = server.join().unwrap();
        assert!(req.starts_with("GET /printer/objects/query?extruder"));
    }

    #[test]
    fn set_temp_urlencodes_the_script() {
        let (addr, server) = one_shot("{\"result\": \"ok\"}");
        let client = Client::new(&addr, "");
        client.set_extruder_temp(195.4).unwrap();
        let req = server.join().unwrap();
        assert!(
            req.starts_with("POST /printer/gcode/script?script=M104%20S195"),
            "got: {}",
            &req[..70]
        );
    }

    #[test]
    fn start_print_urlencodes() {
        let (addr, server) = one_shot("{\"result\": \"ok\"}");
        let client = Client::new(&addr, "");
        client.start_print("my part v2.gcode").unwrap();
        let req = server.join().unwrap();
        assert!(req.starts_with("POST /printer/print/start?filename=my%20part%20v2.gcode"), "got: {}", &req[..80]);
    }

    #[test]
    fn chamber_sensor_preflight() {
        // Present → ok, and it queries the object-list endpoint.
        let (addr, server) =
            one_shot("{\"result\": {\"objects\": [\"extruder\", \"heater_bed\", \"temperature_sensor chamber_temp\"]}}");
        let client = Client::new(&addr, "");
        client.ensure_chamber_sensor("chamber_temp", 50).expect("sensor present");
        let req = server.join().unwrap();
        assert!(req.starts_with("GET /printer/objects/list"), "got: {}", &req[..40]);

        // Absent → error names the sensor we wanted and the ones that exist.
        let (addr, server) =
            one_shot("{\"result\": {\"objects\": [\"extruder\", \"temperature_sensor mcu_temp\"]}}");
        let client = Client::new(&addr, "");
        let err = client.ensure_chamber_sensor("chamber_temp", 50).unwrap_err();
        assert!(err.contains("temperature_sensor chamber_temp"), "names the target: {err}");
        assert!(err.contains("temperature_sensor mcu_temp"), "names what exists: {err}");
        server.join().unwrap();

        // No sensor named in the profile → clear error, returned before any network call.
        let client = Client::new("127.0.0.1:1", "");
        let err = client.ensure_chamber_sensor("  ", 50).unwrap_err();
        assert!(err.contains("names no chamber sensor"), "{err}");
    }
}
