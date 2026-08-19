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

/// A snapshot of the printer: everything the Machine view shows, in ONE
/// query. Deliberately one round trip — the view polls on a timer, and a
/// machine mid-print should never be answering four questions where one
/// does. Temperatures are `None` when the machine has no such heater.
#[derive(Debug, Clone)]
pub struct PrintStatus {
    /// Moonraker print state: standby / printing / paused / complete / error / cancelled.
    pub state: String,
    /// File being printed (empty in standby).
    pub filename: String,
    /// 0.0..=1.0 when printing.
    pub progress: f64,
    /// Nozzle temperature and target (°C).
    pub nozzle: Option<(f64, f64)>,
    /// Bed temperature and target (°C).
    pub bed: Option<(f64, f64)>,
    /// Part-cooling fan duty, 0.0..=1.0.
    pub fan: Option<f64>,
    /// Seconds of printing executed, pauses excluded — Klipper's own clock.
    /// The signal a local playhead disciplines against: unlike a position or
    /// a file offset it isn't running ahead of the motion queue.
    pub print_duration_s: f64,
    /// Byte offset the g-code reader has reached, for matching progress
    /// against a file we sliced ourselves.
    pub file_position: u64,
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
                // finishes — a START_PRINT that homes and heats can take a
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

    /// Current print state / file / progress / temperatures — one round trip.
    pub fn print_status(&self) -> Result<PrintStatus, String> {
        let v = self.call(
            "GET",
            "/printer/objects/query?print_stats&virtual_sdcard&extruder&heater_bed&fan",
        )?;
        let status = &v["result"]["status"];
        let pair = |o: &serde_json::Value| {
            o["temperature"].as_f64().map(|t| (t, o["target"].as_f64().unwrap_or(0.0)))
        };
        Ok(PrintStatus {
            state: status["print_stats"]["state"].as_str().unwrap_or("unknown").to_string(),
            filename: status["print_stats"]["filename"].as_str().unwrap_or("").to_string(),
            progress: status["virtual_sdcard"]["progress"].as_f64().unwrap_or(0.0),
            nozzle: pair(&status["extruder"]),
            bed: pair(&status["heater_bed"]),
            fan: status["fan"]["speed"].as_f64(),
            print_duration_s: status["print_stats"]["print_duration"].as_f64().unwrap_or(0.0),
            file_position: status["virtual_sdcard"]["file_position"].as_u64().unwrap_or(0),
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

    /// Pre-flight for a multi-tool job: every tool the slice uses must exist
    /// on the connected machine — its extruder for the per-tool temperatures,
    /// and, when the toolchange g-code invokes the standard `T{n}` macros,
    /// the macro itself. Catches "sliced for the toolchanger, sent to the
    /// single-tool printer" before upload instead of mid-print.
    /// `needs_t_macros` = the profile's toolchange template uses `T{n}`.
    pub fn ensure_tools(&self, used: &[u32], needs_t_macros: bool) -> Result<(), String> {
        let multi = used.iter().any(|&t| t > 0);
        if !multi {
            return Ok(()); // a tool-0-only job runs on any machine
        }
        let objects = self.objects()?;
        let have = |name: &str| objects.iter().any(|o| o == name);
        let mut missing: Vec<String> = Vec::new();
        for &t in used {
            let ext = if t == 0 { "extruder".to_string() } else { format!("extruder{t}") };
            if !have(&ext) {
                missing.push(format!("[{ext}]"));
            }
            if needs_t_macros && !have(&format!("gcode_macro T{t}")) {
                missing.push(format!("macro T{t}"));
            }
        }
        if missing.is_empty() {
            return Ok(());
        }
        let tools: Vec<String> = used.iter().map(|t| format!("T{t}")).collect();
        Err(format!(
            "This slice uses tools {} but the connected printer is missing {} — wrong printer, \
             or the toolchanger isn't configured yet. It reports {} tool(s). Select the right \
             printer profile (or host), or reassign the parts to the tools it has.",
            tools.join("/"),
            missing.join(", "),
            count_tools(&objects),
        ))
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

/// Compact, status-line-friendly error text.
/// Count tools from a Klipper object list: extruders (`extruder`,
/// `extruder1`, …) vs `gcode_macro T<n>` selection macros, larger wins.
fn count_tools(objects: &[String]) -> usize {
    let all_digits = |s: &str| !s.is_empty() && s.chars().all(|c| c.is_ascii_digit());
    let mut extruders = 0usize;
    let mut macros = 0usize;
    for o in objects {
        if o == "extruder" || o.strip_prefix("extruder").is_some_and(&all_digits) {
            extruders += 1;
        }
        if o.strip_prefix("gcode_macro T").is_some_and(&all_digits) {
            macros += 1;
        }
    }
    extruders.max(macros).max(1)
}

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

    #[test]
    fn tools_are_counted_from_the_object_list() {
        let objs = |names: &[&str]| names.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        // A single-tool machine (the Sovol): one extruder, no T macros.
        assert_eq!(count_tools(&objs(&["extruder", "heater_bed", "gcode_macro START_PRINT"])), 1);
        // A StealthChanger: three extruders AND three selection macros.
        assert_eq!(
            count_tools(&objs(&[
                "extruder", "extruder1", "extruder2",
                "gcode_macro T0", "gcode_macro T1", "gcode_macro T2",
                "gcode_macro TOOLCHANGER",
            ])),
            3
        );
        // Macros lead extruders (partially configured) — the larger wins;
        // lookalikes (T without digits, TIMELAPSE) don't count.
        assert_eq!(count_tools(&objs(&["extruder", "gcode_macro T0", "gcode_macro T1"])), 2);
        assert_eq!(count_tools(&objs(&["extruder", "gcode_macro TIMELAPSE_TAKE_FRAME"])), 1);
        // An empty/odd list still reports one hotend.
        assert_eq!(count_tools(&objs(&[])), 1);
    }

    #[test]
    fn multi_tool_jobs_are_checked_against_the_machine() {
        // A tool-0-only job never even calls the printer.
        let client = Client::new("127.0.0.1:1", "");
        assert!(client.ensure_tools(&[0], true).is_ok());

        // A T1 job against a single-tool machine: named missing pieces.
        let (host, server) = one_shot(
            r#"{"result":{"objects":["extruder","heater_bed","gcode_macro START_PRINT"]}}"#,
        );
        let client = Client::new(&host, "");
        let err = client.ensure_tools(&[0, 1], true).unwrap_err();
        assert!(err.contains("T0/T1"), "{err}");
        assert!(err.contains("[extruder1]") && err.contains("macro T1"), "{err}");
        assert!(err.contains("reports 1 tool"), "{err}");
        server.join().unwrap();

        // The same job against a real toolchanger passes.
        let (host, server) = one_shot(
            r#"{"result":{"objects":["extruder","extruder1","gcode_macro T0","gcode_macro T1"]}}"#,
        );
        let client = Client::new(&host, "");
        assert!(client.ensure_tools(&[0, 1], true).is_ok());
        server.join().unwrap();
    }
}
