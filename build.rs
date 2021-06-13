extern crate cc;

fn main() {
    cc::Build::new().file("libfoo/foo.c").compile("foo");
    println!("cargo:rerun-if-changed=libfoo/foo.c");
}