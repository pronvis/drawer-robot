Based on https://github.com/knurling-rs/app-template

## Run
To execute binary file: `cargo xtask run -p {project_name} -b {bin_file_name}`, for example: `cargo xtask run -p experiments -b stepper_configurator`.
Or just `cargo xtask run -b stepper_configurator`, cause `experiments` is the default project for `run` task: `XTASK_RUN_DEFAULT = "experiments"`
To execute main file: `cargo xtask run -p {project_name}`.
To set logging level change env vat `DEFMT_LOG` at `.cargo/config.toml`

### To stop microcontoller
Erase program: `probe-rs erase --chip STM32F103RE`

## Size
To get info about sections size on microcontoller: `cargo xtask size -p {project_name} -b {bin_file_name}`, for example: `cargo xtask size -p experiments -b stepper_configurator`.

## ReadObj
To get info about binary object: `cargo xtask readobj -p {project_name} -b {bin_file_name}`, for example: `cargo xtask readobj -p experiments -b stepper_configurator`.

## Attach
To attach to running microcontroller, example: `probe-rs attach $CARGO_TARGET_DIR/thumbv7m-none-eabi/release/companion --chip STM32F103C8 --probe 0483:3748:C296C294070132124647524B4E  --log-format "{L} {s}"`

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

## Manual Flashing

1. Build ELF file (`cargo build`)
2. `arm-none-eabi-objcopy -O binary $CARGO_TARGET_DIR/thumbv7m-none-eabi/release/{binary_name} {path_to_output}.bin`
3. `st-flash write {path_to_output}.bin 0x08000000`

## HC-05

Use arduino device to configure HC-05. Scetch and connection image is in folder `./hc05_arduino_configurator/`.
With pressed button `reset` connect HC-05 to arduino - diod whould blink once per 2 seconds.

Commands can be found in `./documentation/HC-05_AT_Command_Set.pdf`

One device should be Master. Another one slave.

I set speed at 115200: `AT+UART=115200,0,0`

For slave: 
- `AT+NAME=slave`
- `AT+UART=115200,0,0`
- `AT+CMODE=1`
- `AT+ROLE=0`
- get addr, we will need it for master: `AT+ADDR?`

For master: 
- `AT+NAME=master`
- `AT+UART=115200,0,0`
- `AT+CMODE=0`
- `AT+BIND={slave_addr}` // replace `:` with `,` in addr string
- `AT+ROLE=1`
