Based on https://github.com/knurling-rs/app-template

## Run
To execute binary file: `cargo xtask run -p {project_name} -b {bin_file_name}`, for example: `cargo xtask run -p experiments -b stepper_configurator`.
Or just `cargo xtask run -b stepper_configurator`, cause `experiments` is the default project for `run` task: `XTASK_RUN_DEFAULT = "experiments"`
To execute main file: `cargo xtask run -p {project_name}`.
To set logging level change env vat `DEFMT_LOG` at `.cargo/config.toml`

### To stop microcontoller
Erase program: `probe-rs erase --chip STM32F103RE`

## TMC2209

Good Vref calculator: https://wiki.fysetc.com/TMC2208/#motor-current-setting
For my steppers (1.7A for phase) Vref should be `1.6970562748477143`,
but I set it to 1.9, cause my friend told me that active cooling required above Vref=1.4

## PS3 controller bluetooth connection

To connect PS3 controller to HC-05 you need to set master address. To do that:
- install SixAxisPairTool (for mac get it from [here](https://github.com/user-none/sixaxispairer))
- connect PS3 to PC
- set master address

## Debugging

1. run `openopcd` in one terminal
2. run `arm-none-eabi-gdb -x openocd.gdb -q {path_to_bin_file}` or change `runner` at `crate/{project}/.cargo/config.toml`
For example: `arm-none-eabi-gdb -x openocd.gdb -q ~/rust/rust_build_artifacts/thumbv7m-none-eabi/release/minimal_x`
