# pico-spl06-001

A library to use Goertek's SPL06-001 Pressure & Temperature Sensor
with RP2040 boards (such as Raspberry Pi Pico)
using Raspberry Pi Pico C/C++ SDK.

This library currently support only Background mode for continuous measurements.

## Getting Started

1. Clone this repo
1. Change config to your setting in example
1. Build with your computer
1. Write generated uf2 file to your board
1. Check serial monitor!
1. Integrate into your project!
   Don't forget to ```target_link_libraries(<target> pico-spl06-001)```in your CMakeLists.txt!

## Quick Steps

1. Set config. "rate" means measure per second.
   ```spl06_config_t config = {<address>, <i2c0 or i2c1>, <mode>, {<pressure rate>, <pressure oversampling>}, {<temperature rate>, <temperature oversampling>}};```
1. Preapare a variable for coefficients, pressure and temperature.
   ```spl06_coef_t coef;```
   ```float prs, temp;```
1. Initialize sensor.
   ```spl06_init();```
1. Read pressure and temperature.
   ```spl06_read_press_cal(&config, &coef, &prs);```
   ```spl06_read_temp_cal(&config, &coef, &temp);```

For details, refer [pico-spl06-001.h](./src/pico-spl06-001.h)(include quick reference) and [Datasheet](https://datasheet.lcsc.com/lcsc/2101201914_Goertek-SPL06-001_C2684428.pdf)

## ToDo

- Support Command Mode

## License

BSD 3-Clause License
