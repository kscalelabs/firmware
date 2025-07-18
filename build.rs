use std::env;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::process::Command;

fn main() {
    let output = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .unwrap();
    let git_hash = String::from_utf8(output.stdout).unwrap();
    let git_hash = git_hash.trim();

    let dest_path = Path::new(&env::var("OUT_DIR").unwrap()).join("git_hash.rs");
    let mut f = File::create(&dest_path).unwrap();

    writeln!(f, "pub const GIT_HASH: &str = \"{git_hash}\";").unwrap();
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/index");
    println!("cargo:rerun-if-changed=build.rs");
}
