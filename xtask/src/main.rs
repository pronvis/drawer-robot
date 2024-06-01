//! Custom commands for cargo.
//!
//! See [cargo-xtask](https://github.com/matklad/cargo-xtask) for details.

use clap::{Parser, Subcommand};
use colored::*;
use eyre::{bail, eyre, Context, ContextCompat};
use std::{
    env,
    ffi::OsString,
    path::{Path, PathBuf},
    process::{Command, ExitCode},
};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// `cd` into specified or default package and run `cargo run`
    #[clap(visible_alias = "r")]
    Run {
        /// Name of the package
        ///
        /// If not provided a default specified in XTASK_RUN_DEFAULT env variable will be used.
        #[arg(short, long)]
        package: Option<OsString>,
        /// Name of the bin
        #[arg(short, long)]
        bin: Option<OsString>,
        /// Arguments for `cargo embed`
        args: Vec<String>,
    },
    /// `cd` into specified or default package and run `cargo size`
    #[clap(visible_alias = "s")]
    Size {
        /// Name of the package
        ///
        /// If not provided a default specified in XTASK_RUN_DEFAULT env variable will be used.
        #[arg(short, long)]
        package: Option<OsString>,
        /// Name of the bin
        #[arg(short, long)]
        bin: Option<OsString>,
        /// Arguments for `cargo embed`
        args: Vec<String>,
    },
    /// `cd` into specified or default package and run `cargo readobj`
    #[clap(visible_alias = "ro")]
    Readobj {
        /// Name of the package
        ///
        /// If not provided a default specified in XTASK_RUN_DEFAULT env variable will be used.
        #[arg(short, long)]
        package: Option<OsString>,
        /// Name of the bin
        #[arg(short, long)]
        bin: Option<OsString>,
        /// Arguments for `cargo embed`
        args: Vec<String>,
    },
    /// `cd` into specified or default package and run `cargo objdump`
    #[clap(visible_alias = "od")]
    Objdump {
        /// Name of the package
        ///
        /// If not provided a default specified in XTASK_RUN_DEFAULT env variable will be used.
        #[arg(short, long)]
        package: Option<OsString>,
        /// Name of the bin
        #[arg(short, long)]
        bin: Option<OsString>,
        /// Arguments for `cargo embed`
        args: Vec<String>,
    },
    /// `cd` into each package and run `cargo build`
    #[clap(visible_alias = "b")]
    Build {
        /// Name of the package
        #[arg(short, long)]
        package: Option<OsString>,
        /// Arguments for `cargo build`
        args: Vec<String>,
    },
    /// `cd` into each package and run `cargo test`
    ///
    /// Packages can be excluded with XTASK_TEST_EXCLUDE environment variable.
    #[clap(visible_alias = "t")]
    Test {
        /// Name of the package
        #[arg(short, long)]
        package: Option<OsString>,
        /// Arguments for `cargo test`
        args: Vec<String>,
    },
}

fn start() -> eyre::Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Run { package, bin, args } => run(package, bin, args)?,
        Cmd::Size { package, bin, args } => size(package, bin, args)?,
        Cmd::Readobj { package, bin, args } => read_obj(package, bin, args)?,
        Cmd::Objdump { package, bin, args } => obj_dump(package, bin, args)?,
        Cmd::Build { package, args } => build(package, args)?,
        Cmd::Test { package, args } => test(package, args)?,
    };
    Ok(())
}

fn run(package: Option<OsString>, bin: Option<OsString>, mut args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let default = match env::var("XTASK_RUN_DEFAULT") {
        Ok(v) => Some(v.into()),
        Err(env::VarError::NotPresent) => None,
        Err(e) => bail!(e),
    };

    let package = package.or(default).ok_or(eyre!(
        "No package to run.\
            Either pass the name to the crate with `-p <package>` option, \
            or define XTASK_RUN_DEFAULT=<package> env variable."
    ))?;

    let mut release_args = vec!["--release".to_string()];
    release_args.append(&mut args);
    args = release_args;

    if let Some(bin) = bin {
        let mut bin = vec!["--bin".to_string(), bin.into_string().unwrap()];
        bin.append(&mut args);
        args = bin;
    }

    let path = members
        .iter()
        .find(|path| path.file_name().unwrap() == package)
        .wrap_err("Thre is no such package")?;

    Command::new("cargo").arg("run").args(&args).current_dir(path).status()?;
    Ok(())
}

fn size(package: Option<OsString>, bin: Option<OsString>, mut args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let default = match env::var("XTASK_RUN_DEFAULT") {
        Ok(v) => Some(v.into()),
        Err(env::VarError::NotPresent) => None,
        Err(e) => bail!(e),
    };

    let package = package.or(default).ok_or(eyre!(
        "No package to size.\
            Either pass the name to the crate with `-p <package>` option, \
            or define XTASK_RUN_DEFAULT=<package> env variable."
    ))?;

    let mut release_args = vec!["--release".to_string()];
    release_args.append(&mut args);
    args = release_args;

    if let Some(bin) = bin {
        let mut bin = vec!["--bin".to_string(), bin.into_string().unwrap()];
        bin.append(&mut args);
        args = bin;
    }

    if !args.contains(&"--".to_string()) {
        args.push("--".to_string());
    }
    args.push("-A".to_string());

    let path = members
        .iter()
        .find(|path| path.file_name().unwrap() == package)
        .wrap_err("Thre is no such package")?;

    Command::new("cargo").arg("size").args(&args).current_dir(path).status()?;
    Ok(())
}

fn read_obj(package: Option<OsString>, bin: Option<OsString>, mut args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let default = match env::var("XTASK_RUN_DEFAULT") {
        Ok(v) => Some(v.into()),
        Err(env::VarError::NotPresent) => None,
        Err(e) => bail!(e),
    };

    let package = package.or(default).ok_or(eyre!(
        "No package to size.\
            Either pass the name to the crate with `-p <package>` option, \
            or define XTASK_RUN_DEFAULT=<package> env variable."
    ))?;

    let mut release_args = vec!["--release".to_string()];
    release_args.append(&mut args);
    args = release_args;

    if let Some(bin) = bin {
        let mut bin = vec!["--bin".to_string(), bin.into_string().unwrap()];
        bin.append(&mut args);
        args = bin;
    }

    if !args.contains(&"--".to_string()) {
        args.push("--".to_string());
    }
    args.push("--file-headers".to_string());

    let path = members
        .iter()
        .find(|path| path.file_name().unwrap() == package)
        .wrap_err("Thre is no such package")?;

    Command::new("cargo").arg("readobj").args(&args).current_dir(path).status()?;
    Ok(())
}

fn obj_dump(package: Option<OsString>, bin: Option<OsString>, mut args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let default = match env::var("XTASK_RUN_DEFAULT") {
        Ok(v) => Some(v.into()),
        Err(env::VarError::NotPresent) => None,
        Err(e) => bail!(e),
    };

    let mut release_args = vec!["--release".to_string()];
    release_args.append(&mut args);
    args = release_args;

    let package = package.or(default).ok_or(eyre!(
        "No package to size.\
            Either pass the name to the crate with `-p <package>` option, \
            or define XTASK_RUN_DEFAULT=<package> env variable."
    ))?;

    if let Some(bin) = bin {
        let mut bin = vec!["--bin".to_string(), bin.into_string().unwrap()];
        bin.append(&mut args);
        args = bin;
    }

    if !args.contains(&"--".to_string()) {
        args.push("--".to_string());
    }
    args.push("--disassemble".to_string());
    args.push("--no-show-raw-insn".to_string());
    args.push("--print-imm-hex".to_string());

    let path = members
        .iter()
        .find(|path| path.file_name().unwrap() == package)
        .wrap_err("Thre is no such package")?;

    Command::new("cargo").arg("objdump").args(&args).current_dir(path).status()?;
    Ok(())
}

fn build(package: Option<OsString>, mut args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let mut release_args = vec!["--release".to_string()];
    release_args.append(&mut args);
    args = release_args;

    let build = |member: &PathBuf| {
        let msg = format!("   xtask: Running `{}` in `{}`", format_cmd("cargo build", &args), member.display());
        println!("{}", msg.blue().bold());

        let status = Command::new("cargo").arg("build").args(&args).current_dir(member).status()?;
        if !status.success() {
            bail!("`cargo build` failed for {}", member.display());
        }
        Ok(())
    };

    if let Some(package) = package {
        let member = members
            .iter()
            .find(|path| path.file_name().unwrap() == package)
            .wrap_err("Thre is no such package")?;
        build(member)?;
    } else {
        for member in members.iter().filter(|m| m != &Path::new("xtask")) {
            build(member)?;
        }
    }
    Ok(())
}

fn test(package: Option<OsString>, args: Vec<String>) -> eyre::Result<()> {
    let members = workspace_members()?;

    let test = |member: &PathBuf| {
        let msg = format!("   xtask: Running `{}` in `{}`", format_cmd("cargo test", &args), member.display());
        println!("{}", msg.blue().bold());
        let status = Command::new("cargo").arg("test").args(&args).current_dir(member).status()?;
        if !status.success() {
            bail!("`cargo test` failed for {}", member.display());
        }
        Ok(())
    };

    if let Some(package) = package {
        let member = members
            .iter()
            .find(|path| path.file_name().unwrap() == package)
            .wrap_err("Thre is no such package")?;
        test(member)?;
    } else {
        let excluded = match env::var("XTASK_TEST_EXCLUDE") {
            Ok(v) => Some(v),
            Err(env::VarError::NotPresent) => None,
            Err(e) => bail!(e),
        };
        let excluded: Vec<OsString> = excluded
            .as_ref()
            .map(|s| s.split(',').map(|s| s.into()).collect())
            .unwrap_or_default();
        let members = members
            .iter()
            .filter(|m| !excluded.contains(&m.file_name().unwrap().to_os_string()));

        for member in members {
            test(member)?;
        }
    }
    Ok(())
}

fn workspace_members() -> eyre::Result<Vec<PathBuf>> {
    let dir = env::var("CARGO_WORKSPACE_DIR").wrap_err("`CARGO_WORKSPACE_DIR` env variable is missing")?;
    let dir = Path::new(&dir);
    let path = dir.join("Cargo.toml");
    let manifest = cargo_toml::Manifest::from_path(&path).wrap_err_with(|| format!("Failed to read top-level Cargo.toml: {path:?}"))?;
    let members = manifest
        .workspace
        .ok_or_else(|| eyre!("No `workspace` field in Cargo.toml"))?
        .members
        .iter()
        .map(|s| s.into())
        .collect();
    Ok(members)
}

fn format_cmd(cmd: &str, args: &[String]) -> String {
    let mut out = String::new();
    out += cmd;
    for arg in args {
        out += " ";
        out += arg;
    }
    out
}

fn main() -> ExitCode {
    if let Err(e) = start() {
        eprintln!("{}{} {e}", "error".red().bold(), ":".bold());
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
