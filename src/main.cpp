// Tenstar nrf52840 (aka nice!nano) BLE keyboard button demo
//
// this demo is based upon:
// https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support
//
// Adafruit nrf52 Arduino library and BLE keyboard example:
// https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Peripheral/blehid_keyboard/blehid_keyboard.ino
//
// see also:
// https://forum.arduino.cc/t/nrf52840-development-board-with-adafruit-nrf52-core/1290505/6


#include <Arduino.h>
#include <bluefruit.h>

// Note: P0.15 is connected to the built-in red led
#define LED PIN_015 
#define LED_RED LED

// ASCII-key action for each button
#define NUM_BUTTONS 5
uint8_t button_map[NUM_BUTTONS] = {PIN_017, PIN_020, PIN_022, PIN_024, PIN_025};
uint8_t key_map[NUM_BUTTONS] = {' ', '\n', '1', '2', '3'};
uint8_t buttonStates = 0;

// Multi-key state (up to 6 simultaneous keys + modifiers)
static uint8_t active_keys[6] = {0};
static uint8_t modifiers = 0; // e.g. KEYBOARD_MODIFIER_LEFT_SHIFT

// Define modifier bit for left shift (USB HID standard)
#ifndef KEYBOARD_MODIFIER_LEFT_SHIFT
#define KEYBOARD_MODIFIER_LEFT_SHIFT 0x02
#endif

BLEDis bledis;
BLEHidAdafruit blehid;


// Convert ASCII to HID keycode & modifier (subset)
uint8_t asciiToKeycode(uint8_t ch, uint8_t &outMod) {
  outMod = 0;
  if (ch >= 'a' && ch <= 'z') return HID_KEY_A + (ch - 'a');
  if (ch >= 'A' && ch <= 'Z') { outMod = KEYBOARD_MODIFIER_LEFT_SHIFT; return HID_KEY_A + (ch - 'A'); }
  if (ch >= '1' && ch <= '9') return HID_KEY_1 + (ch - '1');
  if (ch == '0') return HID_KEY_0;
  if (ch == ' ') return HID_KEY_SPACE;
  if (ch == '\n' || ch == '\r') return HID_KEY_ENTER;
  return 0; // unsupported
}

void rebuildAndSendReport() {
  blehid.keyboardReport(modifiers, active_keys);
}

bool addActive(uint8_t keycode) {
  if (!keycode) return false;
  for (uint8_t i=0;i<6;i++) if (active_keys[i] == keycode) return false; // already
  for (uint8_t i=0;i<6;i++) if (active_keys[i] == 0) { active_keys[i] = keycode; return true; }
  return false; // full
}

bool removeActive(uint8_t keycode) {
  bool removed = false;
  for (uint8_t i=0;i<6;i++) if (active_keys[i] == keycode) { active_keys[i] = 0; removed = true; }
  return removed;
}

void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}


void setup() 
{
  Serial.begin(115200);  // note: the USB CDC serial port is not only useful for debugging
                         // but also for resetting the nRF52 when uploading code via the bootloader

  // while ( !Serial ) delay(10);   // wait until Serial is connected 
  // Serial.println("Tenstar nRF52840 BLE Keyboard Demo ready!");

  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {
    pinMode( button_map[i], INPUT_PULLUP );
  }

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // Configure and Start Device Information Service
  bledis.setManufacturer("Tenstar");
  bledis.setModel("nrf52840 devboard");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blehid.begin();

  // Set callback for set LED from central
  // blehid.setKeyboardLedCallback(set_keyboard_led);

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /* Bluefruit.Periph.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();
  pinMode(LED, OUTPUT); //Set the LED to output mode.
}

void loop() 
{
  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {
    bool pressed = ( digitalRead( button_map[i] ) == LOW );
    if ( pressed && !(buttonStates & (1 << i)) ) {
      // button just pressed
      buttonStates |= (1 << i);
      uint8_t mod=0; uint8_t kc = asciiToKeycode(key_map[i], mod);
      if (kc) {
        modifiers |= mod; // add modifier bits
        addActive(kc);
        rebuildAndSendReport();
        digitalToggle(LED);
      }
    } else if ( !pressed && (buttonStates & (1 << i)) ) {
      // button just released
      buttonStates &= ~(1 << i);
      uint8_t mod=0; uint8_t kc = asciiToKeycode(key_map[i], mod);
      if (kc) {
        removeActive(kc);
        // naive modifier cleanup: clear shift if no uppercase keys remain
        if (mod && mod == KEYBOARD_MODIFIER_LEFT_SHIFT) modifiers &= ~KEYBOARD_MODIFIER_LEFT_SHIFT;
        rebuildAndSendReport();
        digitalToggle(LED);
      }
    }
  }
  delay (10);
}

