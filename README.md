# Bleeny: an nRF52840-based low cost BLE Button

The Bleeny Button is an assistive switch for creating a key press or other HID activities using Bluetooth Low Energy (BLE). It can be used to control SmartPhones, Tablets, Computers or AAC Software like our free and open source [Asterics AAC](https://aac.asterics.eu). The Bleeny Button utilizes the affordable Tenstar Robot devboard (aka nice!nano) with the Nordic nrf52840 microcontroller. A good summary of the features of this SoC/board is provided at the [Zephyr documentation page](https://docs.zephyrproject.org/latest/boards/others/promicro_nrf52840/doc/index.html). The source code builds with PlatformIO, using the Adafruit nRF52 and Bluefruit libraries. The code is based upon the [BLE keyboard example](https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/blehid_keyboard/blehid_keyboard.ino)

## Requirements

* install VSCode/PlatformIO and build a project for the Adafruit Feather nRF52840 Express (or a similar Adafruit nRF board, to install the nRF platform packages) 
* add the files for the nice nano board as described here: [Nicenano-NRF52-Supermini-PlatformIO-Support](https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support)

## Usage

* build and upload the demo code (note that the Serial USB CDC interface must be enabled in the code in order to use the auto-upload via DFU in PlatformIO)
* pair the BLE device (e.g. Bleeny-214D, the unique number is composed from the mac address of your device) 
* attach a pushbuttons to GPIO pin 017 and GND (optionally, more buttons can be connected to pins 020, 022, 024 and 100) 
* when pressed, the buttons shall trigger keys (default settings are: `SPACE` , `ENTER`, `1`, `2` and `3`)

### Notes regarding low power operation / current consumption

In paired active mode, current consumption is about 1.2mA @ 3,3V (which can reduced to ~1mA if the activity LED is not used).  
After an adjustable time of user inactivity (constant `SLEEP_TIMEOUT_MS`), the nRF enters hibernation / deep sleep where current consumption drops to 
~1,4uA @ 3,7V. (For this, the LDO for providing external VCC must be disabled, else around 40uA are consumed). This could be further improved by supplying power directly to the VDD pin on the backside of the PCB, see [this post in the Arduino forum](https://forum.arduino.cc/t/nrf52840-development-board-with-adafruit-nrf52-core/1290505/6)
A wakeup can be triggered by pressing any of the configured buttons. This will cause a system reset. BLE connection to a paired host is usually re-established in 1-3 seconds. 

## Suggested hardware setup
In the `hw` folder, STL files for a 3D-printed enclosure are provided for housing the microcontroller PCB, one [tactile pushbutton (size 12 x 12 mm, height 10mm)](https://de.aliexpress.com/item/1005008563113806.html) and a [170mA LiPo battery](https://www.pollin.de/p/lithium-polymer-akku-hwe601525-3-7-v-170-mah-5-stueck-273699). Using this setup, a BLE switch can be made for less than 10 € material cost!

![parts and size comparison](img/Bleeny_parts1.jpg)
![parts mounted in base](img/Bleeny_parts2.jpg)
![assembled Bleeny Button](img/BleenyButton.jpg)
 
 # Acknowledgement
This work has been accomplished at the UAS Technikum Wien in course of the R&D-project [InDiKo](https://www.technikum-wien.at/en/research-projects/indiko/) (MA23 project 38-09), which is supported by the [City of Vienna](https://www.wien.gv.at/kontakte/ma23/index.html).
