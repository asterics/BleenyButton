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

#define EXT_LOW_PIN PIN_013

#define ENABLE_ACTIVITY_LED 1
#define ENABLE_DEBUG_OUTPUT 0  // set to 1 to enable serial debug output; note that in this case serial must be connected to start operation

// sleep configuration
#define SLEEP_TIMEOUT_MS 180000  // Sleep after 180 seconds of inactivity
unsigned long lastActivityTime = 0;
bool sleepMode = false;

// define ASCII-key action for each button
#define NUM_BUTTONS 5
uint8_t button_map[NUM_BUTTONS] = {PIN_017, PIN_020, PIN_022, PIN_024, PIN_100};
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


void enterSleepMode() {
  if (ENABLE_DEBUG_OUTPUT) Serial.println("Entering sleep mode...");
  sleepMode = true;
  
  // Configure all button pins as wakeup sources with pullup
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    // Set pin as wake source (low level trigger)
    NRF_GPIO->PIN_CNF[button_map[i]] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos) |
                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                       (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
  }
  
  // Turn off LED to save power
  if (ENABLE_ACTIVITY_LED) digitalWrite(LED, LOW); 
  
  // Put nRF52 into low power mode
  sd_power_system_off();
  // This line won't be reached as system_off() causes a reset
}


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


bool addActiveKey(uint8_t keycode) {
  if (!keycode) return false;
  for (uint8_t i=0;i<6;i++) if (active_keys[i] == keycode) return false; // already
  for (uint8_t i=0;i<6;i++) if (active_keys[i] == 0) { active_keys[i] = keycode; return true; }
  return false; // full
}

bool removeActiveKey(uint8_t keycode) {
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
  if (!Bluefruit.Advertising.start(0)) {          // 0 = Don't stop advertising after n seconds
    if (ENABLE_DEBUG_OUTPUT) Serial.println("ERROR: Failed to start advertising!");
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      char name_buffer[64];
      if (Bluefruit.getName(name_buffer, sizeof(name_buffer)) > 0) {
        Serial.print("Advertising as: ");
        Serial.println(name_buffer);
      }    
    }
  }
}


void setup() 
{
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.begin(115200);  // note: the USB CDC serial port is not only useful for debugging
                         // but also for resetting the nRF52 when uploading code via the bootloader

    while ( !Serial ) delay(10);   // wait until Serial is connected 
    Serial.println("Tenstar nRF52840 BLE Keyboard Demo ready!");
  }

  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {
    pinMode( button_map[i], INPUT_PULLUP );
  }

  if (ENABLE_ACTIVITY_LED) {
    pinMode(LED, OUTPUT);  //Set the LED to output mode.
  }

  pinMode(EXT_LOW_PIN, OUTPUT); 
  digitalWrite(EXT_LOW_PIN, LOW); // turn off external LDO to save power 
 
  if (!Bluefruit.begin()) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("ERROR: Failed to initialize Bluefruit!");
    while(1);
  }

  Bluefruit.setTxPower(4);    
  // Bluefruit.Periph.clearBonds();  // Clear all bonding data  Check bluefruit.h for supported values

  // Get MAC address and create unique advertising name
  uint8_t mac[6];
  Bluefruit.getAddr(mac);
  char name_buffer[64]={0};
  sprintf (name_buffer, "Bleeny-%02X%02X", mac[4], mac[5]);
  Bluefruit.setName(name_buffer);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Asterics Foundation");
  bledis.setModel("BleenyButton");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blehid.begin();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /* Bluefruit.Periph.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();

  lastActivityTime = millis();  // initialize activity timer
}

void loop() 
{
  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {

    bool pressed = ( digitalRead( button_map[i] ) == LOW );
    
    if ( pressed && !(buttonStates & (1 << i)) ) {
      // button just pressed
      lastActivityTime = millis();
      buttonStates |= (1 << i);
      uint8_t mod=0; uint8_t kc = asciiToKeycode(key_map[i], mod);
      if (kc) {
        modifiers |= mod; // add modifier bits
        addActiveKey(kc);
        blehid.keyboardReport(modifiers, active_keys);
        // if (ENABLE_ACTIVITY_LED ) digitalToggle(LED);
        if (ENABLE_DEBUG_OUTPUT) Serial.println("Button pressed");
      }
    } else if ( !pressed && (buttonStates & (1 << i)) ) {
      // button just released
      lastActivityTime = millis();
      buttonStates &= ~(1 << i);
      uint8_t mod=0; uint8_t kc = asciiToKeycode(key_map[i], mod);
      if (kc) {
        removeActiveKey(kc);
        // naive modifier cleanup: clear shift if no uppercase keys remain
        if (mod && mod == KEYBOARD_MODIFIER_LEFT_SHIFT) modifiers &= ~KEYBOARD_MODIFIER_LEFT_SHIFT;
        blehid.keyboardReport(modifiers, active_keys);
        //if (ENABLE_ACTIVITY_LED ) digitalToggle(LED);
        if (ENABLE_DEBUG_OUTPUT) Serial.println("Button released");
      }
    }
  }

  //  __WFI(); // wait for interrupt - safe power until a button is pressed/released - did not work as expected here

  // Check for sleep timeout
  if (millis() - lastActivityTime  > SLEEP_TIMEOUT_MS) enterSleepMode();
  
  if (ENABLE_ACTIVITY_LED) {
    static int ledCount=0;
    ledCount++;
    if (ledCount==50) digitalWrite(LED, HIGH);
    else if (ledCount==55) digitalWrite(LED, LOW);
    else if (ledCount>60) ledCount=0;
  }

  delay(20);  // main loop polling @50Hz
}

