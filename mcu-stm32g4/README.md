Chip: stm32g431kb

Based on example https://github.com/embassy-rs/embassy/tree/main/examples/stm32g4

## Pin Configuration

### ADC

On the Nucleo Board, CN3:[7,8] (PA5/PA6) are used for I2C, CN3:[5,10] (PA2/PA3) are used for UART. This leaves PA0, PA1, PA4, PA7 as analog-capable pins.

### OpAmp

To use the OpAmp in buffer mode, we need to connect OPAMPx_VINP to the signal we want to measure.

Potential Pins:

| Pin | Func |
| -- | -- |
| PA1 | OPAMP1_VINP, OPAMP3_VINP |
| ~PA3~ | _(PA3 is used for UART)_ |
| PA7 | OPAMP1_VINP, OPAMP2_VINP |
| PB0 | OPAMP2_VINP, OPAMP3_VINP |
| ~PB13~ | OPAMP3_VINP |
| ~PB14~ | OPAMP2_VINP |
| ~PD14~ | OPAMP2_VINP |

### Usage

There is an approximately 3.2 kΩ series resistor between V_3.3 and the PT100. The PT100 are connected via PA7 and PB0 to OpAmp#2, which amplifies the signal by x16 and feeds it into ADC#2.

Data is output through the ST-Link via UART at PA2.

The firmware calculates temperature based on the series resistor values, but that is extremely inaccurate and should be ignored. Instead, the receiver should calculate the temperature based on the raw ADC values and a two-point calculation, which needs to be done for each board seperately.


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
