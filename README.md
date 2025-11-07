# nRF52840-BLE-Button

A BLE keyboard demo using the affordable Tenstar Robot nrf52840 devboard (aka nice!nano).
Builds with PlatformIO, using the Adafruit nRF52 and Bluefruit libraries. 
This code is based upon the [BLE keyboard example](https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/blehid_keyboard/blehid_keyboard.ino)

## Requirements

* install VSCode/PlatformIO and build a project for the Adafruit Feather nRF52840 Express (or a similar Adafruit nRF board, to install the nRF platform packages) 
* add the files for the nice nano board as described here: [Nicenano-NRF52-Supermini-PlatformIO-Support](https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support)

## Usage

* build and upload the demo code
* pair the BLE device (nrf52840)
* connect up to 5 pushbuttons from GPIO pins 017, 020, 022, 024 and 025 to GND
* when pressed, the buttons shall trigger keys ' ', '\n', '1', '2' and '3'

### Note regarding low power / battery operation

The Tenstar board might need modifications for improving the current consumption in sleep mode, see this [Arduino forum post](https://forum.arduino.cc/t/nrf52840-development-board-with-adafruit-nrf52-core/1290505/6)
 
 
 