# nRF52840-BLE-Button

A BLE keyboard demo using the affordable Tenstar Robot nrf52840 devboard (aka nice!nano).
Builds with PlatformIO, using the Adafruit nRF52 and Bluefruit libraries. 
This code is based upon the [BLE keyboard example](https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/blehid_keyboard/blehid_keyboard.ino)

## Requirements

* install VSCode/PlatformIO and build a project for the Adafruit Feather nRF52840 Express (or a similar Adafruit nRF board, to install the nRF platform packages) 
* add the files for the nice nano board as described here: [Nicenano-NRF52-Supermini-PlatformIO-Support](https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support)

## Usage

* build and upload the demo code (note that the Serial USB CDC interface must be enabled in the code in order to use the auto-upload via DFU in PlatformIO)
* pair the BLE device (nrf52840)
* connect up to 5 pushbuttons from GPIO pins 017, 020, 022, 024 and 100 to GND
* when pressed, the buttons shall trigger keys (default settings are: ' ', '\n', '1', '2' and '3')

### Note regarding low power / battery operation
During operation, the current consumption is about 2mA @ 3,3V (wich can reduced to 1mA if the activity LED is turned off). Currently, the nRF enters hibernation / deep sleep after an adjustable time of user inactivity (constant `SLEEP_TIMEOUT_MS`) and is reset when a button is pressed.
BLE connection to a paired host is usually re-established in 1-3 seconds. During sleep mode, the current consumption is about 38uA @ 3V / 50uA @3,7V. A vast amount of this caused by the charger IC and might be reduced by 
by supplying power directly to the VCC pin on the backside of the PCB, see these [posts in the Arduino forum](https://forum.arduino.cc/t/nrf52840-development-board-with-adafruit-nrf52-core/1290505/6)
 
 
 