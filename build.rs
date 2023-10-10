extern crate bindgen;

use std::path::PathBuf;
use std::{env, str};

use bindgen::CargoCallbacks;

fn main() {
    println!("cargo:rustc-env=DEFMT_LOG=trace");

    // This is the directory where the `c` library is located.
    let libdir_path = PathBuf::from("/Users/pronvis/arduino/TMCStepper/src")
        // Canonicalize the path as `rustc-link-search` requires an absolute
        // path.
        .canonicalize()
        .expect("cannot canonicalize path");

    // This is the path to the `c` headers file.
    let headers_path = libdir_path.join("TMCStepper.h");
    let headers_path_str = headers_path.to_str().expect("Path is not a valid string");

    // This is the path to the intermediate object file for our library.
    let obj_path = libdir_path.join("TMCStepper.o");
    // This is the path to the static library file.
    let lib_path = libdir_path.join("TMCStepper.a");

    // Tell cargo to look for shared libraries in the specified directory
    println!("cargo:rustc-link-search={}", libdir_path.to_str().unwrap());

    // Tell cargo to tell rustc to link our `hello` library. Cargo will
    // automatically know it must look for a `libhello.a` file.
    println!("cargo:rustc-link-lib=TMCStepper");

    // Tell cargo to invalidate the built crate whenever the header changes.
    println!("cargo:rerun-if-changed={}", headers_path_str);

    // Run `clang` to compile the `hello.c` file into a `hello.o` object file.
    // Unwrap if it is not possible to spawn the process.
    let compile_result = std::process::Command::new("clang")
        .arg("-c")
        .arg("-o")
        .arg(&obj_path)
        .arg("--gcc-install-dir=/usr/local/Cellar/gcc/13.2.0/bin/gcc-13")
        // .arg("--gcc-toolchain=/usr/local/Cellar/gcc/13.2.0/bin/gcc-13")
        // .arg("--gcc-toolchain=/usr/bin/gcc")
        // .arg("/usr/bin/gcc")
        // .arg("")
        // .arg("-print-libgcc-file-name")
        .arg("-D")
        .arg("ARDUINO=101") // TMC library contains check `#if defined(ARDUINO) && ARDUINO >= 100`
        .arg("-I")
        .arg(libdir_path.clone())
        .arg("-I")
        .arg("/Users/pronvis/arduino/Arduino_Core_STM32/cores/arduino/")
        .arg(libdir_path.join("source/TMCStepper.cpp"))
        .output()
        .expect("could not spawn `clang`");
    // .status
    // .success()

    if !compile_result.status.success() {
        println!(
            "compile error: {}",
            str::from_utf8(compile_result.stderr.as_slice()).unwrap()
        );
        // Panic if the command was not successful.
        panic!("could not compile object file");
    } else {
        println!(
            "compile success: {}",
            str::from_utf8(compile_result.stdout.as_slice()).unwrap()
        );
    }

    // Run `ar` to generate the `libhello.a` file from the `hello.o` file.
    // Unwrap if it is not possible to spawn the process.
    if !std::process::Command::new("ar")
        .arg("rcs")
        .arg(lib_path)
        .arg(obj_path)
        .output()
        .expect("could not spawn `ar`")
        .status
        .success()
    {
        // Panic if the command was not successful.
        panic!("could not emit library file");
    }

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header(headers_path_str)
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(CargoCallbacks))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");
}
