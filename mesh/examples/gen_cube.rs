//! Generate the cube test fixture: `cargo run -p mesh --example gen_cube`.
//! Writes `fixtures/cube.stl` (20mm cube) relative to the workspace root.

fn main() -> std::io::Result<()> {
    let m = mesh::Mesh::cube(20.0);
    std::fs::create_dir_all("fixtures")?;
    m.write_stl_ascii("fixtures/cube.stl")?;
    println!("wrote fixtures/cube.stl ({} triangles)", m.triangles.len());
    Ok(())
}
