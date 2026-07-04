//! 3MF import: the core spec's meshes, build items, transforms, and
//! components, plus the production extension's cross-part references
//! (`p:path` — how Orca/Bambu project files split objects into parts), plus
//! the human part names and extruder hints those slicers stash in
//! `Metadata/model_settings.config`. Geometry and those hints only: embedded
//! slicer settings, materials/colors, and the other extensions are
//! deliberately ignored — profiles are this slicer's own three-tier system,
//! not something to import from a foreign project file.
//!
//! Tolerant by design (the "load & repair" philosophy): unknown elements are
//! skipped, out-of-range triangle indices and degenerate triangles are
//! dropped, and a file with no `<build>` falls back to its printable objects.

use crate::{Mesh, Vec3};
use quick_xml::events::Event;
use quick_xml::Reader;
use std::collections::HashMap;
use std::io::{BufReader, Read, Seek};
use std::path::Path;

/// One leaf mesh of a build item: its name (may be empty), the extruder hint
/// from `Metadata/model_settings.config` when the file carries one (1-based,
/// as Bambu/Orca write it), and its mesh with every transform baked, in
/// millimeters.
pub struct ThreeMfPart {
    pub name: String,
    pub extruder: Option<u32>,
    pub mesh: Mesh,
}

/// One printable object from the 3MF build: its name (may be empty) and its
/// leaf parts, each with every transform baked, in millimeters.
pub struct ThreeMfItem {
    pub name: String,
    pub parts: Vec<ThreeMfPart>,
}

impl ThreeMfItem {
    /// All parts appended into one mesh — the single-material view.
    pub fn merged(&self) -> Mesh {
        let mut m = Mesh::default();
        for p in &self.parts {
            m.append(&p.mesh);
        }
        m
    }
}

pub fn load_3mf<P: AsRef<Path>>(path: P) -> Result<Vec<ThreeMfItem>, String> {
    let f = std::fs::File::open(path.as_ref()).map_err(|e| e.to_string())?;
    load_3mf_reader(BufReader::new(f))
}

pub fn load_3mf_reader<R: Read + Seek>(reader: R) -> Result<Vec<ThreeMfItem>, String> {
    let mut zip = zip::ZipArchive::new(reader).map_err(|e| format!("not a 3MF (zip): {e}"))?;
    let root = root_model_path(&mut zip)?;

    // Parse the root model part, then any parts referenced via p:path
    // (resolved lazily, each parsed once).
    let mut parts: HashMap<String, Part> = HashMap::new();
    parse_part_into(&mut zip, &root, &mut parts)?;

    // Referenced-part discovery loop: parsing a part can surface new paths.
    loop {
        let missing: Vec<String> = parts
            .values()
            .flat_map(|p| p.referenced_paths())
            .filter(|p| !parts.contains_key(p))
            .collect();
        if missing.is_empty() {
            break;
        }
        for p in missing {
            parse_part_into(&mut zip, &p, &mut parts)?;
        }
    }

    let config = read_model_settings(&mut zip);

    let root_part = &parts[&root];
    let mut out = Vec::new();
    // The build's items are the plate; a (spec-violating) file without a
    // build still yields its printable objects at identity.
    let items: Vec<(u64, Option<String>, Affine)> = if root_part.build.is_empty() {
        root_part.objects.keys().map(|&id| (id, None, Affine::IDENTITY)).collect()
    } else {
        root_part.build.clone()
    };
    for (id, path, t) in items {
        let part_path = path.unwrap_or_else(|| root.clone());
        let mut item = ThreeMfItem { name: String::new(), parts: Vec::new() };
        flatten(&parts, &part_path, id, t, "", &mut item, 0)?;
        if item.parts.is_empty() {
            continue;
        }
        // model_settings hints fill what the core XML left anonymous; config
        // parts pair with leaf parts by document order (Bambu writes one
        // <part> per component), a count mismatch leaves the tail untouched.
        if let Some(cfg) = config.get(&id) {
            if item.name.is_empty() {
                item.name.clone_from(&cfg.name);
            }
            for (lp, cp) in item.parts.iter_mut().zip(&cfg.parts) {
                if lp.name.is_empty() {
                    lp.name.clone_from(&cp.name);
                }
                lp.extruder = cp.extruder.or(cfg.extruder);
            }
        }
        out.push(item);
    }
    Ok(out)
}

/// The model part's zip path, from the OPC root relationships — falling back
/// to the conventional location, then to any `*.model` entry.
fn root_model_path<R: Read + Seek>(zip: &mut zip::ZipArchive<R>) -> Result<String, String> {
    if let Ok(mut rels) = zip.by_name("_rels/.rels") {
        let mut text = String::new();
        if rels.read_to_string(&mut text).is_ok() {
            let mut reader = Reader::from_str(&text);
            let mut buf = Vec::new();
            while let Ok(ev) = reader.read_event_into(&mut buf) {
                match ev {
                    Event::Empty(e) | Event::Start(e)
                        if local_name(e.name().as_ref()) == b"Relationship" =>
                    {
                        let (mut target, mut is_model) = (None, false);
                        for a in e.attributes().flatten() {
                            let key = local_name(a.key.as_ref()).to_vec();
                            let val = String::from_utf8_lossy(&a.value).into_owned();
                            match key.as_slice() {
                                b"Target" => target = Some(val),
                                b"Type" => is_model = val.ends_with("3dmodel"),
                                _ => {}
                            }
                        }
                        if is_model {
                            if let Some(t) = target {
                                return Ok(t.trim_start_matches('/').to_string());
                            }
                        }
                    }
                    Event::Eof => break,
                    _ => {}
                }
                buf.clear();
            }
        }
    }
    if zip.by_name("3D/3dmodel.model").is_ok() {
        return Ok("3D/3dmodel.model".into());
    }
    let fallback = (0..zip.len())
        .filter_map(|i| zip.by_index(i).ok().map(|f| f.name().to_string()))
        .find(|n| n.ends_with(".model"));
    fallback.ok_or_else(|| "no 3D model part in the archive".into())
}

/// 3MF affine transform, exactly as the spec writes it: 12 numbers
/// "m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32", row-vector convention
/// (p' = [p 1] · M; the last row is the translation).
#[derive(Clone, Copy, Debug)]
struct Affine([[f64; 3]; 4]);

impl Affine {
    const IDENTITY: Affine =
        Affine([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]]);

    fn parse(s: &str) -> Option<Affine> {
        let v: Vec<f64> = s.split_whitespace().filter_map(|t| t.parse().ok()).collect();
        if v.len() != 12 {
            return None;
        }
        Some(Affine([
            [v[0], v[1], v[2]],
            [v[3], v[4], v[5]],
            [v[6], v[7], v[8]],
            [v[9], v[10], v[11]],
        ]))
    }

    fn apply(&self, p: Vec3) -> Vec3 {
        let m = &self.0;
        [
            p[0] * m[0][0] + p[1] * m[1][0] + p[2] * m[2][0] + m[3][0],
            p[0] * m[0][1] + p[1] * m[1][1] + p[2] * m[2][1] + m[3][1],
            p[0] * m[0][2] + p[1] * m[1][2] + p[2] * m[2][2] + m[3][2],
        ]
    }

    /// `self` applied first, then `next` (row-vector composition self·next).
    fn then(&self, next: &Affine) -> Affine {
        let (a, b) = (&self.0, &next.0);
        let mut c = [[0.0; 3]; 4];
        for (i, row) in c.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        for j in 0..3 {
            c[3][j] += b[3][j];
        }
        Affine(c)
    }

    /// Whether this transform mirrors the geometry (negative determinant of its
    /// linear part). A mirrored placement — common for a left/right pair built
    /// from one mesh — flips triangle winding when baked in, turning the mesh
    /// inside-out; the caller must reverse the winding to keep it outward-facing.
    fn flips_orientation(&self) -> bool {
        let m = &self.0;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        det < 0.0
    }
}

/// A reference from a component to another object, possibly in another part.
#[derive(Clone)]
struct CompRef {
    id: u64,
    path: Option<String>,
    t: Affine,
}

enum ObjKind {
    Mesh { vertices: Vec<Vec3>, triangles: Vec<[u32; 3]> },
    Components(Vec<CompRef>),
}

struct Obj {
    name: String,
    kind: ObjKind,
}

/// One parsed `.model` part: its printable objects and build items, vertices
/// and translations already scaled to millimeters per the part's `unit`.
struct Part {
    objects: HashMap<u64, Obj>,
    /// (objectid, p:path, transform)
    build: Vec<(u64, Option<String>, Affine)>,
}

impl Part {
    fn referenced_paths(&self) -> Vec<String> {
        let mut v: Vec<String> = self.build.iter().filter_map(|(_, p, _)| p.clone()).collect();
        for o in self.objects.values() {
            if let ObjKind::Components(refs) = &o.kind {
                v.extend(refs.iter().filter_map(|r| r.path.clone()));
            }
        }
        v
    }
}

fn unit_to_mm(unit: &str) -> f64 {
    match unit {
        "micron" => 0.001,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        _ => 1.0, // millimeter (the default), or unknown
    }
}

fn local_name(qname: &[u8]) -> &[u8] {
    match qname.iter().rposition(|&b| b == b':') {
        Some(i) => &qname[i + 1..],
        None => qname,
    }
}

fn parse_part_into<R: Read + Seek>(
    zip: &mut zip::ZipArchive<R>,
    path: &str,
    parts: &mut HashMap<String, Part>,
) -> Result<(), String> {
    let mut file = zip.by_name(path).map_err(|e| format!("model part {path}: {e}"))?;
    let mut text = String::new();
    file.read_to_string(&mut text).map_err(|e| format!("model part {path}: {e}"))?;
    let part = parse_model_xml(&text).map_err(|e| format!("model part {path}: {e}"))?;
    parts.insert(path.to_string(), part);
    Ok(())
}

fn parse_model_xml(text: &str) -> Result<Part, String> {
    let mut reader = Reader::from_str(text);
    let mut buf = Vec::new();

    let mut scale = 1.0;
    let mut objects: HashMap<u64, Obj> = HashMap::new();
    let mut build: Vec<(u64, Option<String>, Affine)> = Vec::new();

    // Current-object accumulation state.
    let mut cur_id: Option<u64> = None;
    let mut cur_name = String::new();
    let mut cur_printable = true;
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut components: Vec<CompRef> = Vec::new();
    let mut saw_components = false;

    let attr_str = |a: &quick_xml::events::attributes::Attribute| -> String {
        String::from_utf8_lossy(&a.value).into_owned()
    };

    loop {
        let ev = reader.read_event_into(&mut buf).map_err(|e| format!("XML: {e}"))?;
        match &ev {
            Event::Start(e) | Event::Empty(e) => match local_name(e.name().as_ref()) {
                b"model" => {
                    for a in e.attributes().flatten() {
                        if local_name(a.key.as_ref()) == b"unit" {
                            scale = unit_to_mm(&attr_str(&a));
                        }
                    }
                }
                b"object" => {
                    cur_id = None;
                    cur_name.clear();
                    cur_printable = true;
                    vertices.clear();
                    triangles.clear();
                    components.clear();
                    saw_components = false;
                    for a in e.attributes().flatten() {
                        match local_name(a.key.as_ref()) {
                            b"id" => cur_id = attr_str(&a).parse().ok(),
                            b"name" => cur_name = attr_str(&a),
                            // Only type="model" (the default) is printable —
                            // support/surface/other objects are tooling.
                            b"type" => cur_printable = attr_str(&a) == "model",
                            _ => {}
                        }
                    }
                }
                b"vertex" => {
                    let (mut x, mut y, mut z) = (0.0, 0.0, 0.0);
                    for a in e.attributes().flatten() {
                        let v: f64 = attr_str(&a).parse().unwrap_or(0.0);
                        match local_name(a.key.as_ref()) {
                            b"x" => x = v,
                            b"y" => y = v,
                            b"z" => z = v,
                            _ => {}
                        }
                    }
                    vertices.push([x * scale, y * scale, z * scale]);
                }
                b"triangle" => {
                    let mut t = [u32::MAX; 3];
                    for a in e.attributes().flatten() {
                        let v: u32 = attr_str(&a).parse().unwrap_or(u32::MAX);
                        match local_name(a.key.as_ref()) {
                            b"v1" => t[0] = v,
                            b"v2" => t[1] = v,
                            b"v3" => t[2] = v,
                            _ => {}
                        }
                    }
                    // Drop out-of-range and degenerate triangles (tolerance).
                    let n = vertices.len() as u32;
                    if t.iter().all(|&i| i < n) && t[0] != t[1] && t[1] != t[2] && t[0] != t[2] {
                        triangles.push(t);
                    }
                }
                b"component" => {
                    saw_components = true;
                    let (mut id, mut cpath, mut t) = (None, None, Affine::IDENTITY);
                    for a in e.attributes().flatten() {
                        match local_name(a.key.as_ref()) {
                            b"objectid" => id = attr_str(&a).parse().ok(),
                            b"path" => {
                                cpath = Some(attr_str(&a).trim_start_matches('/').to_string())
                            }
                            b"transform" => {
                                t = Affine::parse(&attr_str(&a)).unwrap_or(Affine::IDENTITY);
                                // Translation is in this part's units.
                                for j in 0..3 {
                                    t.0[3][j] *= scale;
                                }
                            }
                            _ => {}
                        }
                    }
                    if let Some(id) = id {
                        components.push(CompRef { id, path: cpath, t });
                    }
                }
                b"item" => {
                    let (mut id, mut ipath, mut t) = (None, None, Affine::IDENTITY);
                    for a in e.attributes().flatten() {
                        match local_name(a.key.as_ref()) {
                            b"objectid" => id = attr_str(&a).parse().ok(),
                            b"path" => {
                                ipath = Some(attr_str(&a).trim_start_matches('/').to_string())
                            }
                            b"transform" => {
                                t = Affine::parse(&attr_str(&a)).unwrap_or(Affine::IDENTITY);
                                for j in 0..3 {
                                    t.0[3][j] *= scale;
                                }
                            }
                            _ => {}
                        }
                    }
                    if let Some(id) = id {
                        build.push((id, ipath, t));
                    }
                }
                _ => {}
            },
            Event::End(e) if local_name(e.name().as_ref()) == b"object" => {
                if let (Some(id), true) = (cur_id, cur_printable) {
                    let kind = if saw_components {
                        ObjKind::Components(std::mem::take(&mut components))
                    } else {
                        ObjKind::Mesh {
                            vertices: std::mem::take(&mut vertices),
                            triangles: std::mem::take(&mut triangles),
                        }
                    };
                    objects.insert(id, Obj { name: std::mem::take(&mut cur_name), kind });
                }
                cur_id = None;
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    Ok(Part { objects, build })
}

#[derive(Default)]
struct ConfigPart {
    name: String,
    extruder: Option<u32>,
}

/// Per-object hints from `Metadata/model_settings.config`: name, 1-based
/// extruder (the default for parts without their own), and one entry per
/// `<part>` in document order.
#[derive(Default)]
struct ConfigObject {
    name: String,
    extruder: Option<u32>,
    parts: Vec<ConfigPart>,
}

/// Read `Metadata/model_settings.config` (Bambu/Orca keep part names and
/// extruder assignments there, keyed by build object id — not in the core
/// XML). Tolerant by design: a missing entry or malformed XML yields
/// whatever parsed, never an error.
fn read_model_settings<R: Read + Seek>(zip: &mut zip::ZipArchive<R>) -> HashMap<u64, ConfigObject> {
    let mut out = HashMap::new();
    let mut text = String::new();
    match zip.by_name("Metadata/model_settings.config") {
        Ok(mut f) => {
            if f.read_to_string(&mut text).is_err() {
                return out;
            }
        }
        Err(_) => return out,
    }
    let mut reader = Reader::from_str(&text);
    let mut buf = Vec::new();
    let mut cur: Option<(u64, ConfigObject)> = None;
    let mut cur_part: Option<ConfigPart> = None;
    loop {
        let Ok(ev) = reader.read_event_into(&mut buf) else {
            break; // malformed from here on: keep what parsed
        };
        match &ev {
            Event::Start(e) | Event::Empty(e) => {
                let self_closing = matches!(&ev, Event::Empty(_));
                let attr = |key: &[u8]| -> Option<String> {
                    e.attributes().flatten().find_map(|a| {
                        (local_name(a.key.as_ref()) == key)
                            .then(|| String::from_utf8_lossy(&a.value).into_owned())
                    })
                };
                match local_name(e.name().as_ref()) {
                    b"object" => {
                        cur_part = None;
                        if let Some(id) = attr(b"id").and_then(|s| s.parse().ok()) {
                            if self_closing {
                                out.insert(id, ConfigObject::default());
                            } else {
                                cur = Some((id, ConfigObject::default()));
                            }
                        }
                    }
                    b"part" if cur.is_some() => {
                        // A bare <part/> still occupies its document-order slot.
                        if self_closing {
                            if let Some((_, o)) = cur.as_mut() {
                                o.parts.push(ConfigPart::default());
                            }
                        } else {
                            cur_part = Some(ConfigPart::default());
                        }
                    }
                    b"metadata" => {
                        let target = if let Some(p) = cur_part.as_mut() {
                            Some((&mut p.name, &mut p.extruder))
                        } else {
                            cur.as_mut().map(|(_, o)| (&mut o.name, &mut o.extruder))
                        };
                        if let (Some((name, extruder)), Some(key)) = (target, attr(b"key")) {
                            let value = attr(b"value").unwrap_or_default();
                            match key.as_str() {
                                "name" => *name = value,
                                "extruder" => *extruder = value.trim().parse().ok(),
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
            Event::End(e) => match local_name(e.name().as_ref()) {
                b"part" => {
                    if let (Some(p), Some((_, o))) = (cur_part.take(), cur.as_mut()) {
                        o.parts.push(p);
                    }
                }
                b"object" => {
                    cur_part = None;
                    if let Some((id, o)) = cur.take() {
                        out.insert(id, o);
                    }
                }
                _ => {}
            },
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }
    out
}

/// Recursively bake `(part_path, id)` under transform `t`, emitting one
/// part into `item` per leaf mesh object (empty meshes dropped). `ancestor`
/// is the nearest named object above on this path — the name fallback for
/// anonymous leaves; the item keeps the first non-empty name found top-down.
fn flatten(
    parts: &HashMap<String, Part>,
    part_path: &str,
    id: u64,
    t: Affine,
    ancestor: &str,
    item: &mut ThreeMfItem,
    depth: usize,
) -> Result<(), String> {
    if depth > 32 {
        return Err("component recursion too deep (cyclic references?)".into());
    }
    let part = parts
        .get(part_path)
        .ok_or_else(|| format!("missing model part {part_path}"))?;
    let Some(obj) = part.objects.get(&id) else {
        return Ok(()); // dangling reference: tolerate
    };
    if item.name.is_empty() {
        item.name.clone_from(&obj.name);
    }
    let named = if obj.name.is_empty() { ancestor } else { &obj.name };
    match &obj.kind {
        ObjKind::Mesh { vertices, triangles } => {
            if !triangles.is_empty() {
                // A mirrored placement flips winding when baked — reverse it
                // back so normals face outward (else the viewer shades it dark
                // and the winding-directed slicer fills it to nothing).
                let triangles: Vec<[u32; 3]> = if t.flips_orientation() {
                    triangles.iter().map(|&[a, b, c]| [a, c, b]).collect()
                } else {
                    triangles.clone()
                };
                let mesh = Mesh {
                    vertices: vertices.iter().map(|&v| t.apply(v)).collect(),
                    triangles,
                };
                item.parts.push(ThreeMfPart { name: named.to_string(), extruder: None, mesh });
            }
        }
        ObjKind::Components(refs) => {
            for r in refs {
                let child_path = r.path.as_deref().unwrap_or(part_path);
                flatten(parts, child_path, r.id, r.t.then(&t), named, item, depth + 1)?;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Cursor, Write};

    /// Zip up a root model (at the conventional path, with rels) plus any
    /// extra entries (`.model` parts, `Metadata/model_settings.config`, …).
    fn make_3mf(root_model: &str, extra: &[(&str, &str)]) -> Cursor<Vec<u8>> {
        let mut zip = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let opts = zip::write::SimpleFileOptions::default();
        zip.start_file("_rels/.rels", opts).unwrap();
        zip.write_all(
            br#"<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
            <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#,
        )
        .unwrap();
        zip.start_file("3D/3dmodel.model", opts).unwrap();
        zip.write_all(root_model.as_bytes()).unwrap();
        for (path, xml) in extra {
            zip.start_file(*path, opts).unwrap();
            zip.write_all(xml.as_bytes()).unwrap();
        }
        zip.finish().unwrap()
    }

    /// A 2-triangle square at z=0, vertices (0..10)².
    const SQUARE_MESH: &str = r#"<mesh><vertices>
        <vertex x="0" y="0" z="0"/><vertex x="10" y="0" z="0"/>
        <vertex x="10" y="10" z="0"/><vertex x="0" y="10" z="0"/>
        </vertices><triangles>
        <triangle v1="0" v2="1" v3="2"/><triangle v1="0" v2="2" v3="3"/>
        </triangles></mesh>"#;

    fn bbox(m: &Mesh) -> ([f64; 3], [f64; 3]) {
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for v in &m.vertices {
            for k in 0..3 {
                lo[k] = lo[k].min(v[k]);
                hi[k] = hi[k].max(v[k]);
            }
        }
        (lo, hi)
    }

    #[test]
    fn mirrored_build_item_flips_winding() {
        // A build item with a mirror transform (negative determinant — here a
        // flip across X) reverses the baked triangle winding, so a left/right
        // pair built from one mesh stays outward-facing. Without this the
        // mirrored copy shades dark in the viewer and slices to nothing.
        let mirror = format!(
            r#"<?xml version="1.0"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
            <resources><object id="1" name="square" type="model">{SQUARE_MESH}</object></resources>
            <build><item objectid="1" transform="-1 0 0 0 1 0 0 0 1 0 0 0"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&mirror, &[])).unwrap();
        assert_eq!(
            &items[0].parts[0].mesh.triangles,
            &vec![[0, 2, 1], [0, 3, 2]],
            "mirror reverses winding"
        );
        // A plain (non-mirror) placement leaves winding untouched.
        let plain = format!(
            r#"<?xml version="1.0"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
            <resources><object id="1" name="square" type="model">{SQUARE_MESH}</object></resources>
            <build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/></build></model>"#
        );
        let plain = load_3mf_reader(make_3mf(&plain, &[])).unwrap();
        assert_eq!(&plain[0].parts[0].mesh.triangles, &vec![[0, 1, 2], [0, 2, 3]]);
    }

    #[test]
    fn core_object_with_build_item() {
        let model = format!(
            r#"<?xml version="1.0"?><model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
            <resources><object id="1" name="square" type="model">{SQUARE_MESH}</object></resources>
            <build><item objectid="1" transform="1 0 0 0 1 0 0 0 1 5 0 0"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        assert_eq!(items.len(), 1);
        assert_eq!(items[0].name, "square");
        assert_eq!(items[0].parts.len(), 1);
        assert_eq!(items[0].parts[0].name, "square");
        assert_eq!(items[0].parts[0].extruder, None);
        assert_eq!(items[0].parts[0].mesh.vertices.len(), 4);
        assert_eq!(items[0].parts[0].mesh.triangles.len(), 2);
        let (lo, hi) = bbox(&items[0].parts[0].mesh);
        assert_eq!((lo[0], hi[0]), (5.0, 15.0), "item transform translates +5 in x");
    }

    #[test]
    fn units_scale_to_millimeters() {
        let model = format!(
            r#"<model unit="inch"><resources><object id="1">{SQUARE_MESH}</object></resources>
            <build><item objectid="1"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        let (_, hi) = bbox(&items[0].merged());
        assert_eq!(hi[0], 254.0, "10 in = 254 mm");
    }

    #[test]
    fn components_compose_transforms() {
        // Object 2 = two copies of object 1, one shifted +20 x; the build
        // item lifts the assembly +5 z.
        let model = format!(
            r#"<model><resources>
            <object id="1">{SQUARE_MESH}</object>
            <object id="2" name="pair"><components>
                <component objectid="1"/>
                <component objectid="1" transform="1 0 0 0 1 0 0 0 1 20 0 0"/>
            </components></object></resources>
            <build><item objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 5"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        assert_eq!(items.len(), 1);
        assert_eq!(items[0].name, "pair");
        // One part per leaf component, unnamed leaves inherit the ancestor.
        assert_eq!(items[0].parts.len(), 2);
        for p in &items[0].parts {
            assert_eq!(p.name, "pair");
            assert_eq!(p.mesh.triangles.len(), 2);
        }
        let (lo, hi) = bbox(&items[0].parts[0].mesh);
        assert_eq!((lo[0], hi[0]), (0.0, 10.0), "first copy stays at 0..10");
        let (lo, hi) = bbox(&items[0].parts[1].mesh);
        assert_eq!((lo[0], hi[0]), (20.0, 30.0), "second copy lands at 20..30");
        let merged = items[0].merged();
        assert_eq!(merged.triangles.len(), 4);
        let (lo, hi) = bbox(&merged);
        assert_eq!((lo[0], hi[0]), (0.0, 30.0));
        assert_eq!((lo[2], hi[2]), (5.0, 5.0), "assembly lifted to z=5");
    }

    #[test]
    fn production_extension_cross_part_reference() {
        // The Orca/Bambu layout: the root model only references an object
        // stored in a separate part.
        let part = format!(
            r#"<model><resources><object id="7" name="boat">{SQUARE_MESH}</object></resources><build/></model>"#
        );
        let root = r#"<model xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
            <resources><object id="1"><components>
            <component objectid="7" p:path="/3D/Objects/object_1.model"/>
            </components></object></resources>
            <build><item objectid="1"/></build></model>"#;
        let items =
            load_3mf_reader(make_3mf(root, &[("3D/Objects/object_1.model", &part)])).unwrap();
        assert_eq!(items.len(), 1);
        assert_eq!(items[0].name, "boat");
        assert_eq!(items[0].parts.len(), 1);
        assert_eq!(items[0].parts[0].name, "boat");
        assert_eq!(items[0].parts[0].mesh.triangles.len(), 2);
    }

    #[test]
    fn multiple_build_items_become_multiple_objects() {
        let model = format!(
            r#"<model><resources><object id="1" name="a">{SQUARE_MESH}</object></resources>
            <build><item objectid="1"/><item objectid="1" transform="1 0 0 0 1 0 0 0 1 50 0 0"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        assert_eq!(items.len(), 2);
        let (lo2, _) = bbox(&items[1].merged());
        assert_eq!(lo2[0], 50.0);
    }

    #[test]
    fn tolerant_of_junk_and_strict_about_non_3mf() {
        // Bad indices + degenerate triangles dropped; support-type objects
        // skipped; missing build falls back to printable objects.
        let model = format!(
            r#"<model><resources>
            <object id="1"><mesh><vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/></vertices>
            <triangles><triangle v1="0" v2="1" v3="2"/><triangle v1="0" v2="1" v3="9"/><triangle v1="1" v2="1" v3="2"/></triangles></mesh></object>
            <object id="2" type="support">{SQUARE_MESH}</object>
            </resources></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        assert_eq!(items.len(), 1, "support object skipped, mesh object kept");
        assert_eq!(items[0].parts.len(), 1);
        assert_eq!(items[0].parts[0].mesh.triangles.len(), 1, "junk triangles dropped");
        // Not a zip at all:
        assert!(load_3mf_reader(Cursor::new(b"solid nope".to_vec())).is_err());
    }

    #[test]
    fn part_names_leaf_over_ancestor() {
        // A named leaf keeps its own name; an anonymous leaf inherits the
        // nearest named ancestor on its path.
        let model = format!(
            r#"<model><resources>
            <object id="1" name="wheel">{SQUARE_MESH}</object>
            <object id="2">{SQUARE_MESH}</object>
            <object id="3" name="cart"><components>
                <component objectid="1"/>
                <component objectid="2" transform="1 0 0 0 1 0 0 0 1 20 0 0"/>
            </components></object></resources>
            <build><item objectid="3"/></build></model>"#
        );
        let items = load_3mf_reader(make_3mf(&model, &[])).unwrap();
        assert_eq!(items[0].name, "cart");
        assert_eq!(items[0].parts.len(), 2);
        assert_eq!(items[0].parts[0].name, "wheel");
        assert_eq!(items[0].parts[1].name, "cart");
    }

    /// Two-part assembly with anonymous core-XML objects — the Bambu shape.
    fn anonymous_pair_model() -> String {
        format!(
            r#"<model><resources>
            <object id="1">{SQUARE_MESH}</object>
            <object id="2">{SQUARE_MESH}</object>
            <object id="3"><components>
                <component objectid="1"/>
                <component objectid="2" transform="1 0 0 0 1 0 0 0 1 20 0 0"/>
            </components></object></resources>
            <build><item objectid="3"/></build></model>"#
        )
    }

    #[test]
    fn model_settings_names_and_extruders() {
        let config = r#"<?xml version="1.0" encoding="UTF-8"?>
            <config>
              <object id="3">
                <metadata key="name" value="Figurine"/>
                <metadata key="extruder" value="1"/>
                <part id="1" subtype="normal_part">
                  <metadata key="name" value="hair"/>
                  <metadata key="extruder" value="2"/>
                </part>
                <part id="2" subtype="normal_part">
                  <metadata key="name" value="body"/>
                </part>
              </object>
              <plate><metadata key="plater_id" value="1"/></plate>
            </config>"#;
        let items = load_3mf_reader(make_3mf(
            &anonymous_pair_model(),
            &[("Metadata/model_settings.config", config)],
        ))
        .unwrap();
        assert_eq!(items[0].name, "Figurine", "empty core name adopts the config's");
        assert_eq!(items[0].parts.len(), 2);
        assert_eq!(items[0].parts[0].name, "hair");
        assert_eq!(items[0].parts[0].extruder, Some(2));
        assert_eq!(items[0].parts[1].name, "body");
        assert_eq!(items[0].parts[1].extruder, Some(1), "object-level extruder inherited");
    }

    #[test]
    fn model_settings_mismatch_and_junk_tolerated() {
        // One config <part> for a two-part item: the overlapping prefix is
        // assigned, the tail is untouched.
        let config = r#"<config><object id="3">
            <metadata key="extruder" value="1"/>
            <part id="1"><metadata key="name" value="hair"/><metadata key="extruder" value="2"/></part>
            </object></config>"#;
        let items = load_3mf_reader(make_3mf(
            &anonymous_pair_model(),
            &[("Metadata/model_settings.config", config)],
        ))
        .unwrap();
        assert_eq!(items[0].parts[0].name, "hair");
        assert_eq!(items[0].parts[0].extruder, Some(2));
        assert_eq!(items[0].parts[1].name, "");
        assert_eq!(items[0].parts[1].extruder, None);

        // Garbage config: loaded as if absent.
        let items = load_3mf_reader(make_3mf(
            &anonymous_pair_model(),
            &[("Metadata/model_settings.config", "<<< not xml at all")],
        ))
        .unwrap();
        assert_eq!(items[0].parts.len(), 2);
        assert!(items[0].parts.iter().all(|p| p.extruder.is_none()));
    }
}
