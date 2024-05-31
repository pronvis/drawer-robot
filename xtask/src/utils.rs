use std::{path::PathBuf, process::Command, str};

use anyhow::anyhow;
use colored::Colorize;

/// Execute the [`Command`]. If success return `stdout`, if failure print to `stderr`
pub fn run_capturing_stdout(cmd: &mut Command) -> anyhow::Result<String> {
    let output = cmd.output()?;
    match output.status.success() {
        true => Ok(str::from_utf8(&output.stdout)?.to_string()),
        false => {
            eprintln!("{}", str::from_utf8(&output.stderr)?.dimmed());
            Err(anyhow!(""))
        }
    }
}

pub fn run_command(program: &str, args: &[&str], cwd: Option<&str>, envs: &[(&str, &str)]) -> anyhow::Result<()> {
    let mut cmd = Command::new(program);
    cmd.args(args).envs(envs.iter().copied());

    let cwd = if let Some(path) = cwd {
        cmd.current_dir(path);
        format!("{path}$ ")
    } else {
        "".to_string()
    };

    let cmdline = format!("{cwd}{program} {}", args.join(" "));
    println!("🏃 {cmdline}");

    cmd.status()
        .map_err(|e| anyhow!("could not run '{}': {}", cmdline, e))
        .and_then(|exit_status| match exit_status.success() {
            true => Ok(()),
            false => Err(anyhow!("'{}' did not finish successfully: {}", cmdline, exit_status)),
        })
}
