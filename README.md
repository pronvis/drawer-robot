Based on https://github.com/knurling-rs/app-template

## Run
To execute binary file: `cargo rrb {bin file name}`, for example: `cargo rrb several_motors`.
To execute main file: `cargo rmain`.
To set logging level change `./build.rs`

For more information take a look at aliases at `.cargo/config.toml`

## TMC2209

Good Vref calculator: https://wiki.fysetc.com/TMC2208/#motor-current-setting
For my steppers (1.7A for phase) Vref should be `1.6970562748477143`,
but I set it to 1.9, cause my friend told me that active cooling required above Vref=1.4

