This is a firmware for USB to I2S converter.

* use AT32UC3A3 microcontroller
* use two crystal clocks as 44.1kHz and 48kHz master clock
* USB Audio class 2.0 compatible
* Based on [sdr-widget](https://github.com/borgestrand/sdr-widget.git).

Schematic and Board are [here](https://github.com/tarori/Circuit.USB-DDC.git).

To build
* Use WSL or Cygwin
* Install [dfu-programmer](https://dfu-programmer.github.io/) and AVR32 Toolchain 2.x (download from [microchip site](https://www.microchip.com/mplab/avr-support/avr-and-sam-downloads-archive)).
* `make write` to compile and write to board
