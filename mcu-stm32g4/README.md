Chip: stm32g431kb

Based on example https://github.com/embassy-rs/embassy/tree/main/examples/stm32g4

## Pin Configuration

On the Nucleo Board, CN3:[7,8] (PA5/PA6) are used for I2C, CN3:[5,10] (PA2/PA3) are used for UART. This leaves PA0, PA1, PA4, PA7 as analog-capable pins.

## Setup

Install Rust using [rustup](https://rustup.rs/).

```sh
# once
cargo install probe-rs-tools

# compile, flash and run, monitor output
cargo run --release

# monitor the output of a running mcu without resetting it
probe-rs attach ./target/thumbv7em-none-eabi/release/pt100_stm32g4 --chip STM32G431KB
```

---


Board:
NUCLEO-G431KB
https://www.st.com/en/evaluation-tools/nucleo-g431kb.html
