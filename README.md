This project contains the firmware for the STM32 Nucelo board that's used to measure electrolyte in- and outflow temperatures using PT100 probes at the big ely test station.

The firmware is in ./mcu-stm32g4.

In ./app, there's a small TUI that reads received data from UART.

In ./analysis there's the worksheet that was used for 2-point calibration.

The ROS Node which receives the data ignores the temperatures calculated internally by the firmware (which are inaccurate) and recalibrates based on the worksheet:

```
=== Calibration Coefficients ===
PT100 #1: T = 0.008517 * ADC + (-270.186)
PT100 #2: T = 0.008559 * ADC + (-269.970)
```