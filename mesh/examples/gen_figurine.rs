//! Generate fixtures/figurine.3mf — a three-part, three-extruder test model in
//! the Bambu layout: anonymous mesh objects composed into one build item, with
//! the human names and extruder assignments in Metadata/model_settings.config.
//!
//!   body   20×20×8 at the origin           extruder 1
//!   cap    12×12×6 centered on top (z 8)   extruder 2  (rests on the body)
//!   emblem  4×4×4 on the front face,       extruder 3  (overlaps the body by
//!           1 mm in y — exercises overlap ownership)
//!
//! Usage: cargo run -p mesh --example gen_figurine [-- out.3mf]

use std::io::Write;

fn box_xml(lo: [f64; 3], hi: [f64; 3]) -> String {
    let v = [
        [lo[0], lo[1], lo[2]],
        [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]],
        [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]],
        [hi[0], lo[1], hi[2]],
        [hi[0], hi[1], hi[2]],
        [lo[0], hi[1], hi[2]],
    ];
    let mut s = String::from("<mesh><vertices>");
    for p in v {
        s += &format!(r#"<vertex x="{}" y="{}" z="{}"/>"#, p[0], p[1], p[2]);
    }
    s += "</vertices><triangles>";
    for t in [
        [0, 2, 1], [0, 3, 2], // bottom
        [4, 5, 6], [4, 6, 7], // top
        [0, 1, 5], [0, 5, 4], // front
        [3, 6, 2], [3, 7, 6], // back
        [0, 7, 3], [0, 4, 7], // left
        [1, 2, 6], [1, 6, 5], // right
    ] {
        s += &format!(r#"<triangle v1="{}" v2="{}" v3="{}"/>"#, t[0], t[1], t[2]);
    }
    s += "</triangles></mesh>";
    s
}

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "fixtures/figurine.3mf".into());

    let body = box_xml([0.0, 0.0, 0.0], [20.0, 20.0, 8.0]);
    let cap = box_xml([4.0, 4.0, 8.0], [16.0, 16.0, 14.0]);
    let emblem = box_xml([8.0, -3.0, 2.0], [12.0, 1.0, 6.0]);

    let model = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model">{body}</object>
  <object id="2" type="model">{cap}</object>
  <object id="3" type="model">{emblem}</object>
  <object id="4" type="model"><components>
   <component objectid="1"/>
   <component objectid="2"/>
   <component objectid="3"/>
  </components></object>
 </resources>
 <build><item objectid="4"/></build>
</model>"#
    );

    let settings = r#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="4">
    <metadata key="name" value="figurine"/>
    <metadata key="extruder" value="1"/>
    <part id="1" subtype="normal_part">
      <metadata key="name" value="body"/>
      <metadata key="extruder" value="1"/>
    </part>
    <part id="2" subtype="normal_part">
      <metadata key="name" value="cap"/>
      <metadata key="extruder" value="2"/>
    </part>
    <part id="3" subtype="normal_part">
      <metadata key="name" value="emblem"/>
      <metadata key="extruder" value="3"/>
    </part>
  </object>
</config>"#;

    let rels = r#"<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#;
    let types = r#"<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>"#;

    let f = std::fs::File::create(&out).expect("create output");
    let mut zip = zip::ZipWriter::new(f);
    let opts = zip::write::SimpleFileOptions::default();
    for (path, content) in [
        ("[Content_Types].xml", types),
        ("_rels/.rels", rels),
        ("3D/3dmodel.model", model.as_str()),
        ("Metadata/model_settings.config", settings),
    ] {
        zip.start_file(path, opts).unwrap();
        zip.write_all(content.as_bytes()).unwrap();
    }
    zip.finish().unwrap();
    println!("wrote {out}");
}
