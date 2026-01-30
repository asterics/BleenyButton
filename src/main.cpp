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
#include "Adafruit_LittleFS.h"
#include "gpio_helper.h"
#include "InternalFileSystem.h"

#define EXT_LOW_PIN 13

#define ENABLE_ACTIVITY_LED 1
#define ENABLE_DEBUG_OUTPUT 1  // set to 1 to enable serial debug output; 
#define STARTUP_WAIT_SERIAL 0  // wait for Serial to be connected on startup. Note: this applies after wakeups too!

// sleep configuration
uint sleep_timeout_ms = 180000;  // Sleep after 180 seconds of inactivity
unsigned long lastActivityTime = 0;
bool sleepMode = false;

// define ASCII-key action for each button
#define NUM_BUTTONS 4
uint8_t button_map[NUM_BUTTONS] = {BUTTON1, BUTTON2, BUTTON3, BUTTON4};
//default key map (index in key_codes)
uint8_t key_map[NUM_BUTTONS] = {0, 1, 2, 3};
//overwrite the key_map (if set to >= 0), (index in key_codes)
int8_t key_code_map[NUM_BUTTONS] = {-1};
uint8_t buttonStates = 0;

//this firmware supports following keycodes in the settings (printHelp(), parseCommands() -> enums of keys)
//Space, Enter, 1, 2, Tab, F1, F2, F13, F14
#define SELECTABLE_KEYS 9
uint8_t key_codes[SELECTABLE_KEYS] = {HID_KEY_SPACE, HID_KEY_ENTER, HID_KEY_1, HID_KEY_2, HID_KEY_TAB, HID_KEY_F1, HID_KEY_F2, HID_KEY_F13, HID_KEY_F14};

// Multi-key state (up to 6 simultaneous keys + modifiers)
static uint8_t active_keys[6] = {0};
static uint8_t modifiers = 0; // e.g. KEYBOARD_MODIFIER_LEFT_SHIFT

BLEDis bledis;
BLEHidAdafruit blehid;
//settings handling and parser at the end of the file
void loadSettings();
bool storeSettings();
using namespace Adafruit_LittleFS_Namespace;
File file(InternalFS);
void parseCommand(char *buf);
void printHelp();
#define MAX_PARAM_LEN 32

/******* output to 3.5mm jackplug ******/
//use output functions ('c' command)
#define OUTPUT_ACTIVE

#ifdef OUTPUT_ACTIVE
  //latching 1 coil relay on P0.02 (D18) & P0.29 (D20)
  uint8_t pin_out[2] = {OUT1,OUT2};
  //different modes for the output
  int mode = 0; 
  // how many modes are used
  #define MODE_MAX 3
  //pin for the mode switch button
  uint8_t pin_mode = PIN_MODE; 
  //output mode: 0: click the output on each action; 1: toggle the output on each action
  int output_mode = 0;
  //f-prototypes to control the output
  uint tremor_timeout_ms = 1000;
  uint pause_timeout_s = 5;
  void triggerOutput();
  void output(bool on);
  void handleOutput(bool pressed, bool released);
#endif

void enterSleepMode() {
  //TODO: check if we are charging -> no sleep mode!


  if (ENABLE_DEBUG_OUTPUT) { Serial.println("Entering sleep mode..."); delay(50); } /*delay in debug is necessary to still print out via USB.*/
  sleepMode = true;
  
  // Configure all button pins as wakeup sources with pullup
  uint32_t ulPin;

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    //ignore unused pins
    if(button_map[i] > NUMBER_OF_PINS) continue;

    //map pins to correct port & pin
    ulPin = button_map[i];
    NRF_GPIO_Type * port = nrf_gpio_pin_port_decode(&ulPin);

    // Set pin as wake source (low level trigger)
    port->PIN_CNF[ulPin] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos) |
                                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                       (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                                       (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                       (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
  }
  
  // Turn off LED to save power
  if (ENABLE_ACTIVITY_LED) dWrite(LED_R, !LED_ON);
  #ifdef LED_G 
    dWrite(LED_G,!LED_ON); 
  #endif
  #ifdef LED_B
    dWrite(LED_B,!LED_ON);
  #endif
  
  // Put nRF52 into low power mode
  sd_power_system_off();
  // This line won't be reached as system_off() causes a reset
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
    if(STARTUP_WAIT_SERIAL) {
      while ( !Serial ) delay(10);   // wait until Serial is connected 
    }
    Serial.println("BleenyButton by AsTeRICS Foundation / Assistronik ready");
  }

  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {
    dMode( button_map[i], INPUT_PULLUP );
  }

  if (ENABLE_ACTIVITY_LED) {
    dMode(LED_R, OUTPUT);  //Set the LED to output mode.
  }

  dMode(EXT_LOW_PIN, OUTPUT); 
  //OUTPUT_H0H1 //high drive sink & source
  dWrite(EXT_LOW_PIN, LOW); // turn off external LDO to save power

  //if output is active, activate GPIOs in high drive mode
  #ifdef OUTPUT_ACTIVE
  dMode(pin_out[0], OUTPUT_H0H1);
  dMode(pin_out[1], OUTPUT_H0H1);
  dMode(pin_mode, INPUT_PULLUP);
  #endif

  loadSettings();

 
  if (!Bluefruit.begin()) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("ERROR: Failed to initialize Bluefruit!");
    while(1);
  }

  Bluefruit.setTxPower(4);    

  // Get MAC address and create unique advertising name
  uint8_t mac[6];
  Bluefruit.getAddr(mac);
  char name_buffer[64]={0};
  sprintf (name_buffer, "Bleeny-%02X%02X", mac[4], mac[5]);
  Bluefruit.setName(name_buffer);
  //not used, we read out the connected device in printHelp()
  //Bluefruit.Periph.setConnectCallback(connect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("AsTeRICS Foundation / Assistronik");
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
  static char serialbuffer[MAX_PARAM_LEN];
  static int offset = 0;

  for ( uint8_t i=0; i<NUM_BUTTONS; i++ ) {

    //if input pins are set to 0xFF (or -1 or whatever; >number of pins) -> ignore
    if(button_map[i] > NUMBER_OF_PINS) continue;

    bool pressed = ( dRead( button_map[i] ) == LOW );
    
    if ( pressed && !(buttonStates & (1 << i)) ) {
      // button just pressed
      lastActivityTime = millis();
      buttonStates |= (1 << i);
      #ifdef OUTPUT_ACTIVE
        if(i == 0) handleOutput(true,false);
      #endif
      addActiveKey(key_codes[key_map[i]]);
      blehid.keyboardReport(modifiers, active_keys);
      // if (ENABLE_ACTIVITY_LED ) dToggle(LED_R);
      if (ENABLE_DEBUG_OUTPUT) Serial.println("Button pressed");
    } else if ( !pressed && (buttonStates & (1 << i)) ) {
      // button just released
      lastActivityTime = millis();
      buttonStates &= ~(1 << i);
      #ifdef OUTPUT_ACTIVE
        if(i == 0) handleOutput(false,true);
      #endif
      removeActiveKey(key_codes[key_map[i]]);
      blehid.keyboardReport(modifiers, active_keys);
      //if (ENABLE_ACTIVITY_LED ) dToggle(LED_R);
      if (ENABLE_DEBUG_OUTPUT) Serial.println("Button released");
    }
  }

  #ifdef OUTPUT_ACTIVE
    //even if not pressed or released, handle the output (for possible auto-releasing of output)
    handleOutput(false,false);
  #endif

  //read one line from serial
  while(Serial.available()) {
    serialbuffer[offset] = Serial.read();
    if(serialbuffer[offset] == '\n') {
      parseCommand(serialbuffer);
      offset = 0;
      memclr(serialbuffer,sizeof(serialbuffer));
    } else {
      offset++;
      if(offset == MAX_PARAM_LEN) {
        Serial.println("Too long");
        offset = 0;
      }
    }
  }

  //if enabled, check the mode switch button for the output
  #ifdef OUTPUT_ACTIVE
  if(dRead(pin_mode) == false) {
    delay(10);
    mode ++;
    if(mode == MODE_MAX) mode = 0;

    if(ENABLE_DEBUG_OUTPUT) { Serial.print("Mode: "); Serial.println(mode+1); }

    while(dRead(pin_mode) == false);
  }
  #endif

  // Check for sleep timeout
  if (millis() - lastActivityTime  > sleep_timeout_ms) enterSleepMode();
  
  if (ENABLE_ACTIVITY_LED) {
    static int ledCount=0;
    ledCount++;
    if (ledCount==100) dWrite(LED_R, LED_ON);
    else if (ledCount==105) dWrite(LED_R, !LED_ON);
    else if (ledCount>110) ledCount=0;
  }

  delay(20);  // main loop polling @50Hz
}


bool storeSettings() {
  InternalFS.remove("settings.txt");
  file.open("settings.txt", FILE_O_WRITE);
  //activity timeout
  char buffer[MAX_PARAM_LEN] = {0};
  snprintf(buffer,MAX_PARAM_LEN,"i:%d\n",sleep_timeout_ms);
  file.write(buffer);

  //save keycodes
  //Note: currently only 2 buttons are saved, although it might be possible to
  // configure all buttons, but in the UI only 2 are visible (for better overview)
  for(int i = 0; i<2; i++) {
    snprintf(buffer,MAX_PARAM_LEN,"%d:%d\n",i,key_map[i]);
    file.write(buffer);
  }
  

  #ifdef OUTPUT_ACTIVE
    snprintf(buffer,MAX_PARAM_LEN,"t:%d\n",tremor_timeout_ms);
    file.write(buffer);
    snprintf(buffer,MAX_PARAM_LEN,"o:%d\n",output_mode);
    file.write(buffer);
    snprintf(buffer,MAX_PARAM_LEN,"p:%d\n",pause_timeout_s);
    file.write(buffer);    
    snprintf(buffer,MAX_PARAM_LEN,"m:%d\n",mode+1);
    file.write(buffer);
  #endif

  file.close();
  return true;
}

void loadSettings() {
  char buffer[MAX_PARAM_LEN] = {0};
  uint totalRead = 0;
  bool readSuccess = false;
  InternalFS.begin();
  file.open("settings.txt", FILE_O_READ);
  if(file) {
    do {
      //repeat until one line is found  (\n); always reserve the last \0.
      for(int i = 0; i<(MAX_PARAM_LEN-1); i++) {
        if(file.read(buffer+i,1)) {
          readSuccess = true;
          totalRead++;
        } else {
          readSuccess = false;
          break;
        }

        if(buffer[i] == '\n') {
          break;
        }
      }
      if(ENABLE_DEBUG_OUTPUT) { Serial.println("Found setting: "); Serial.print(buffer);}
      //send command to parser
      parseCommand(buffer);
    } while(readSuccess || totalRead < file.size());
    file.close();
  } else {
    if(ENABLE_DEBUG_OUTPUT) Serial.println("Settings not found");
  }
}

void parseCommand(char *buf) {
  uint newValue = 0;

  //very very simple, check first character.
  //should be sufficient for this device
  switch(buf[0]) {
    //?: help with supported commands
    default:
      Serial.println("Unknown command, use:");
    case '?':
      printHelp();
    break;
    //i: set the inactivity time to poweroff the uC
    case 'i':
      Serial.print("Prev: "); Serial.println(sleep_timeout_ms);
      newValue = String(buf+2).toInt();
      if(newValue >= 30000 && newValue <= 600000) sleep_timeout_ms = newValue;
      Serial.print("New: "); Serial.println(sleep_timeout_ms);
    break;

    //handle button<->key code assignment
    case '1':
    case '2':
      newValue = buf[0] - '1'; //get button index
      Serial.print("Prev: "); Serial.println(key_map[newValue]);
      key_map[newValue] = String(buf+2).toInt();
      Serial.print("New: "); Serial.println(key_map[newValue]);
    break;
    
    #ifdef OUTPUT_ACTIVE
      //t: tremor timeout (mode 1)
      case 't':
        Serial.print("Prev: "); Serial.println(tremor_timeout_ms);
        newValue = String(buf+2).toInt();
        if(newValue >= 300 && newValue <= 5000) tremor_timeout_ms = newValue;
        Serial.print("New: "); Serial.println(tremor_timeout_ms);
      break;    
      //p: auto-pause (mode 3)
      case 'p':
        Serial.print("Prev: "); Serial.println(pause_timeout_s);
        newValue = String(buf+2).toInt();
        if(newValue >= 2 && newValue <= 600) pause_timeout_s = newValue;
        Serial.print("New: "); Serial.println(pause_timeout_s);
      break;        
      //o: output mode type (click or toggle)
      case 'o':
        Serial.print("Prev: "); Serial.println(output_mode);
        newValue = String(buf+2).toInt();
        if(newValue >= 0 && newValue <= 1) output_mode = newValue;
        Serial.print("New: "); Serial.println(output_mode);
      break;      
      //m: mode
      case 'm':
        Serial.print("Prev: "); Serial.println(mode+1);
        newValue = String(buf+2).toInt();
        if(newValue >= 1 && newValue <= 3) mode = newValue-1;
        Serial.print("New: "); Serial.println(mode+1);
      break;
      case 'c':
        triggerOutput();
        Serial.println("OK");
      break;
    #endif

    case 's':
      if(storeSettings()) Serial.println("OK");
      else Serial.println("NOK");
    break;

    case 'r':
      //reset the BLE pairings
      Bluefruit.Periph.clearBonds();
      Serial.println("OK");
    break;
  }
}

#ifdef OUTPUT_ACTIVE
void handleOutput(bool pressed, bool released) {
  static int lastMode = 0xFF;
  static unsigned long lastAction = 0;

  //reset actions when switching mode
  if(mode != lastMode) {
    lastAction = 0;
    lastMode = mode;
  }

  switch(mode) {
    //anti-tremor: click & lock action for <tremor_timeout_ms>
    case 0:
      //1.) click when pressed & store last press
      if(pressed && lastAction == 0) {
        lastAction = millis();
        triggerOutput();
      }
      //2.) no action until tremor_timeout_ms passed
      if(lastAction != 0 && (millis() - lastAction > tremor_timeout_ms)) {
        lastAction = 0;
      }
      //3.) reset timer if multiple presses
      if(pressed && lastAction != 0) {
        lastAction = millis();
      }
    break;

    //on each edge, click output once
    case 1:
      if(pressed || released) {
        triggerOutput();
        #if ENABLE_DEBUG_OUTPUT
          Serial.println("Mode 2: click");
        #endif
      }
    break;

    //on press: click, lock any action for <timeout> seconds, click. Then wait for click again
    case 2:
      //1.) click when pressed & store last press
      if(pressed && lastAction == 0) {
        lastAction = millis();
        triggerOutput();
      }
      //2.) no action until pause_timeout_s passed, then click
      if(lastAction != 0 && (millis() - lastAction > (pause_timeout_s*1000))) {
        lastAction = 0;
        triggerOutput();
      }
      //3.) reset timeout on multiple presses
      if(pressed && lastAction != 0) {
        lastAction = millis();
      }

    break;

    default: break;
  }
}

void printHelp() {
  // id string
  #ifdef OUTPUT_ACTIVE
    Serial.print("Bleeny with Output - "); 
  #else
    Serial.print("Bleeny - "); 
  #endif

  char central_name[32] = { 0 };
  Bluefruit.Connection(Bluefruit.connHandle())->getPeerName(central_name, sizeof(central_name));
  
  Serial.println(__DATE__);
  Serial.println("s:<none>:Store new settings on the device");
  Serial.print("i:<int>:Inactivity time [ms]:30000-600000:"); Serial.println(sleep_timeout_ms);
  Serial.print("d:<info>:Connected device::"); Serial.println(central_name);
  Serial.println("r:<none>:Reset paired devices");
  Serial.print("1:<enum>:Key 1:Space,Enter,1,2,Tab,F1,F2,F13,F14:"); Serial.println(key_map[0]);
  Serial.print("2:<enum>:Key 2:Space,Enter,1,2,Tab,F1,F2,F13,F14:"); Serial.println(key_map[1]);

  #ifdef OUTPUT_ACTIVE
  Serial.println("c:<none>:Trigger the output");
  Serial.println("o:<enum>:Output mode:click,toggle");
  Serial.print("t:<int>:Mode 1 - Tremor Timeout [ms]:300-5000:"); Serial.println(tremor_timeout_ms);
  Serial.print("p:<int>:Mode 3 - Auto-Pause Timeout [s]:2-600:"); Serial.println(pause_timeout_s);
  Serial.print("m:<int>:Startup Mode:1-3:"); Serial.println(mode+1);
  #endif
  Serial.println("?:<none>:Print out supported commands and build date");
  //examples for more commands (+types)
  //Serial.println("b:<bool>:Enable Bluetooth");
  //Serial.println("b:<info>:Connected device");
  //Serial.println("m:<enum>:Operating mode:auto,manual,test");
  //Serial.println("n:<string>:Device name:1-32");
  //Serial.println("f:<float>:Temperature offset:-10.0-10.0");
}

void triggerOutput() {
  static int current = -1;
  if(current == -1) { 
    output(false); 
    current = 0;
  }

  if(output_mode == 0) {
    output(true);
    delay(100);
    output(false);
  } else {
    if(current == 0) {
      output(true);
      current = 1;
    } else {
      output(false);
      current = 0;
    }
  }
}

void output(bool on) {
  dWrite(pin_out[0], on);
  dWrite(pin_out[1], !on);

  //2ms settle time
  delay(2);

  dWrite(pin_out[0], false);
  dWrite(pin_out[1], false);
}
#endif
