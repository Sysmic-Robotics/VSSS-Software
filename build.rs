// build.rs
use protobuf_codegen::Codegen;
use std::fs;
use std::path::PathBuf;

fn main() {
    let proto_dir = "src/protos";

    // 1. Collect all .proto files in the directory
    let proto_files: Vec<PathBuf> = fs::read_dir(proto_dir)
        .expect("Failed to read protos directory")
        .filter_map(|entry| {
            let entry = entry.ok()?;
            let path = entry.path();
            if path.extension()? == "proto" {
                Some(path)
            } else {
                None
            }
        })
        .collect();

    // 2. Tell Cargo to re-run this script if any .proto file changes
    for proto_file in &proto_files {
        println!("cargo:rerun-if-changed={}", proto_file.display());
    }

    // 3. Generate the Rust code
    Codegen::new()
        .pure() // Use pure Rust generator (no protoc binary needed on system)
        // .protoc() // OR use this if you have protoc installed and prefer it
        .out_dir(proto_dir) // Output the generated .rs files back into src/protos
        .inputs(&proto_files)
        .include(proto_dir) // Directory where .proto files import each other from
        .run_from_script();
}
