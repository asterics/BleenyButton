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
#if STARTUP_WAIT_SERIAL == 1
  #warning "NO RELEASE WITH WAIT FOR SERIAL!"
#endif

// sleep configuration
uint sleep_timeout_ms = 180000;  // Sleep after 180 seconds of inactivity
unsigned long lastActivityTime = 0;
bool sleepMode = false;

// define ASCII-key action for each button
#define NUM_BUTTONS 1
uint8_t button_map[NUM_BUTTONS] = {BUTTON1};
uint8_t buttonStates = 0;

//settings handling and parser at the end of the file
void loadSettings();
bool storeSettings();
using namespace Adafruit_LittleFS_Namespace;
File file(InternalFS);
void parseCommand(char *buf);
void printHelp();
#define MAX_PARAM_LEN 32

/******* output to Shelly Plug ******/
uint8_t peripheral_mac[6];
//different modes for the output
int mode = 0; 
// how many modes are used
#define MODE_MAX 3
//output mode: 0: click the output on each action; 1: toggle the output on each action
int output_mode = 1;
//f-prototypes to control the output
uint tremor_timeout_ms = 500;
uint pause_timeout_s = 5;
void triggerOutput(bool on, bool toggle);
void handleOutput(bool pressed, bool released);
void printMAC();

/******* Shelly BLE stuff */
//from: https://kb.shelly.cloud/knowledge-base/kbsa-communicating-with-shelly-devices-via-bluetoo
#define SHELLY_GATT_SERVICE_UUID "5f6d4f53-5f52-5043-5f53-56435f49445f"
#define SHELLY_RPC_CHAR_DATA_UUID  "5f6d4f53-5f52-5043-5f64-6174615f5f5f"
#define SHELLY_RPC_CHAR_RX_CTL_UUID "5f6d4f53-5f52-5043-5f72-785f63746c5f"
#define SHELLY_RPC_CHAR_TX_CTL_UUID "5f6d4f53-5f52-5043-5f74-785f63746c5f"
BLEClientService        shelly_service(SHELLY_GATT_SERVICE_UUID);
BLEClientCharacteristic shelly_rpc_data(SHELLY_RPC_CHAR_DATA_UUID);
BLEClientCharacteristic shelly_rpc_rx_ctl(SHELLY_RPC_CHAR_RX_CTL_UUID);
BLEClientCharacteristic shelly_rpc_tx_ctl(SHELLY_RPC_CHAR_TX_CTL_UUID);

const char * shelly_toggle = "{\"id\":1,\"method\":\"Switch.Toggle\",\"params\":{\"id\":0}}"; 
const char * shelly_on = "{\"id\":1,\"method\":\"Switch.Set\",\"params\":{\"id\":0,\"on\":true}}";
const char * shelly_off = "{\"id\":1,\"method\":\"Switch.Set\",\"params\":{\"id\":0,\"on\":false}}";
void shelly_send_rpc(const char * cmd); //user called function to send an RPC call to the shelly
void shelly_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len); //callback when receiving RPC answers from shelly (we will receive the size here)
void connect_callback(uint16_t conn_handle); //activate service & characteristics here
void scan_callback(ble_gap_evt_adv_report_t* report); //scanning -> store MAC address of Shelly here, if it is VERY close

void enterSleepMode() {
  //if charge state pin is enabled, check: when charging don't enter sleep mode
  #ifdef CHARGE_STATE
    if(dRead(CHARGE_STATE) == LOW) {
      if(ENABLE_DEBUG_OUTPUT) Serial.println("Charging, don't enter sleep mode");
      lastActivityTime = millis();
      return;
    }
  #endif
  //if we have the battery state on a voltage divider, disable it when entering sleep mode
  //note: don't set this to input when charging, worst case: 4.2V on GPIO pin...
  #ifdef VBATT_DIVIDER_LOW
    dMode(VBATT_DIVIDER_LOW,INPUT);
  #endif

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

void setup() 
{
  //if on XIAO: enable res-divider (might be dangerous: charging & disabled res-divider...)
  //https://wiki.seeedstudio.com/XIAO_BLE/#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging
  #ifdef VBATT_DIVIDER_LOW
    dMode(VBATT_DIVIDER_LOW,OUTPUT);
    dWrite(VBATT_DIVIDER_LOW,LOW);
  #endif
  #ifdef CHARGE_STATE
    dMode(CHARGE_STATE,INPUT_PULLUP);
  #endif

  Serial.begin(115200);  // note: the USB CDC serial port is not only useful for debugging
                         // but also for resetting the nRF52 when uploading code via the bootloader
  if (ENABLE_DEBUG_OUTPUT) {

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

  dMode(PIN_MODE,INPUT_PULLUP);

  dMode(EXT_LOW_PIN, OUTPUT); 
  //OUTPUT_H0H1 //high drive sink & source
  dWrite(EXT_LOW_PIN, LOW); // turn off external LDO to save power

  loadSettings();

 
  if (!Bluefruit.begin(0,1)) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("ERROR: Failed to initialize Bluefruit!");
    while(1);
  }

  Bluefruit.setName("BleenyButton - Shelly");
  // Initialize Shelly service
  shelly_service.begin();
  // and add characteristics
  shelly_rpc_data.begin(); //READ / WRITE
  shelly_rpc_rx_ctl.setNotifyCallback(shelly_notify_callback); //INDICATE / NOTIFY / READ
  shelly_rpc_rx_ctl.setIndicateCallback(shelly_notify_callback); //INDICATE / NOTIFY / READ
  shelly_rpc_rx_ctl.begin(); 
  shelly_rpc_tx_ctl.begin(); // WRITE

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept HRM service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds

  if(ENABLE_DEBUG_OUTPUT) Serial.println("Bluefruit Central setup finished");

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
      if(i == 0) handleOutput(true,false);
      if (ENABLE_DEBUG_OUTPUT) Serial.println("Button pressed");
    } else if ( !pressed && (buttonStates & (1 << i)) ) {
      // button just released
      lastActivityTime = millis();
      buttonStates &= ~(1 << i);
      if(i == 0) handleOutput(false,true);
      //if (ENABLE_ACTIVITY_LED ) dToggle(LED_R);
      if (ENABLE_DEBUG_OUTPUT) Serial.println("Button released");
    }
  }

  //even if not pressed or released, handle the output (for possible auto-releasing of output)
  handleOutput(false,false);

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

  //check the mode switch button for the output
  if(dRead(PIN_MODE) == false) {
    delay(10);
    mode ++;
    if(mode == MODE_MAX) mode = 0;

    if(ENABLE_DEBUG_OUTPUT) { Serial.print("Mode: "); Serial.println(mode+1); }

    while(dRead(PIN_MODE) == false);
  }

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


  memclr(buffer,MAX_PARAM_LEN); snprintf(buffer,MAX_PARAM_LEN,"t:%d\n",tremor_timeout_ms);
  file.write(buffer);
  memclr(buffer,MAX_PARAM_LEN); snprintf(buffer,MAX_PARAM_LEN,"o:%d\n",output_mode);
  file.write(buffer);
  memclr(buffer,MAX_PARAM_LEN); snprintf(buffer,MAX_PARAM_LEN,"p:%d\n",pause_timeout_s);
  file.write(buffer);    
  memclr(buffer,MAX_PARAM_LEN); snprintf(buffer,MAX_PARAM_LEN,"m:%d\n",mode+1);
  file.write(buffer);  
  memclr(buffer,MAX_PARAM_LEN); snprintf(buffer,MAX_PARAM_LEN,"r:%02X:%02X:%02X:%02X:%02X:%02X\n",peripheral_mac[0],peripheral_mac[1],peripheral_mac[2],peripheral_mac[3],peripheral_mac[4],peripheral_mac[5]);
  file.write(buffer);

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
          buffer[i] = 0;
          break;
        }
      }
      if(ENABLE_DEBUG_OUTPUT) { Serial.print("Found setting: "); Serial.println(buffer);}
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
      triggerOutput(false,false);
      Serial.println("OK");
    break;

    case 's':
      if(storeSettings()) Serial.println("OK");
      else Serial.println("NOK");
    break;

    case 'r':
      //reset the BLE pairings
      //TODO: set or clear here the MAC Adress
      Serial.print("Prev: "); printMAC();
      for(int i = 0; i<6; i++) {
        peripheral_mac[i] = strtol(buf+2+i*3,NULL,16);
      }
      Serial.print("New: "); printMAC();
    break;
  }
}

void printHelp() {
  // id string
  Serial.println("BleenyShelly");

  char shelly_name[32] = { 0 };
  Bluefruit.Connection(Bluefruit.connHandle())->getPeerName(shelly_name, sizeof(shelly_name));
  
  Serial.println(__DATE__);
  Serial.println("s:<none>:Store new settings on the device");
  Serial.print("i:<int>:Inactivity time [ms]:30000-600000:"); Serial.println(sleep_timeout_ms);
  Serial.print("d:<info>:Connected device::"); Serial.println(shelly_name);
  Serial.print("r:<string>:Set Shelly MAC:18:"); printMAC(); Serial.println("");
  Serial.println("c:<none>:Trigger the output");
  Serial.print("o:<enum>:Output mode:click,toggle:"); Serial.println(output_mode);
  Serial.print("t:<int>:Mode 1 - Tremor Timeout [ms]:300-5000:"); Serial.println(tremor_timeout_ms);
  Serial.print("p:<int>:Mode 3 - Auto-Pause Timeout [s]:2-600:"); Serial.println(pause_timeout_s);
  Serial.print("m:<int>:Startup Mode:1-3:"); Serial.println(mode+1);
  Serial.println("?:<none>:Print out supported commands and build date");
  //examples for more commands (+types)
  //Serial.println("b:<bool>:Enable Bluetooth");
  //Serial.println("b:<info>:Connected device");
  //Serial.println("m:<enum>:Operating mode:auto,manual,test");
  //Serial.println("n:<string>:Device name:1-32");
  //Serial.println("f:<float>:Temperature offset:-10.0-10.0");
}

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
        #if ENABLE_DEBUG_OUTPUT
          Serial.println("Mode 1: toggle");
        #endif
        triggerOutput(false,true);
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
        triggerOutput(pressed,false);
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
        triggerOutput(pressed,false);
        #if ENABLE_DEBUG_OUTPUT
          Serial.println("Mode 3: start");
        #endif
      }
      //2.) no action until pause_timeout_s passed, then click
      if(lastAction != 0 && (millis() - lastAction > (pause_timeout_s*1000))) {
        lastAction = 0;
        triggerOutput(false,false);
        #if ENABLE_DEBUG_OUTPUT
          Serial.println("Mode 3: stop");
        #endif
      }
      //3.) reset timeout on multiple presses
      if(pressed && lastAction != 0) {
        lastAction = millis();
      }

    break;

    default: break;
  }
}

void triggerOutput(bool on, bool toggle) {
  static int current = -1;
  if(current == -1) {
    shelly_send_rpc(shelly_off);
    current = 0;
  }

  if(output_mode == 0) {
    //mode 0 -> always toggle
    shelly_send_rpc(shelly_on);
    delay(500);
    shelly_send_rpc(shelly_off);
    return;
  }

  //mode 1: on/off
  if(output_mode == 1) {
    if(!toggle) {
      if(on && current == 0) {
        shelly_send_rpc(shelly_on);
        current = 1;
      } else if(!on && current == 1) {
        shelly_send_rpc(shelly_off);
        current = 0;
      }
    } else {
      shelly_send_rpc(shelly_toggle);
      if(current) current = 0;
    }
  }
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  if(ENABLE_DEBUG_OUTPUT) {
    Serial.println("Got scan callback");

    Serial.print("RSSI: "); Serial.println(report->rssi);
    Serial.print("Peer: "); 
    for(int i = 0; i<5; i++) { Serial.print(report->peer_addr.addr[i],HEX); Serial.print(":"); }
    Serial.println(report->peer_addr.addr[5],HEX);
  }

  //VERY close -> store MAC
  if(report->rssi > -35) {
    memcpy(peripheral_mac,report->peer_addr.addr,6);
    Serial.print("New pairing: "); printMAC(); Serial.println("");
    storeSettings();
  }

  //we need to connect always, don't know why
  Bluefruit.Central.connect(report);
}

void connect_callback(uint16_t conn_handle)
{
  if(ENABLE_DEBUG_OUTPUT)  {
    Serial.println("Connected");
    Serial.print("Discovering Shelly Service ... ");
  }

  //if MAC is correct, then connect
  ble_gap_addr_t peer = Bluefruit.Connection(conn_handle)->getPeerAddr();
  if(memcmp(peer.addr, peripheral_mac, 6) != 0) {
    Serial.print("Not matching paired MAC");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  // If Shelly service is not found, disconnect and return
  if ( !shelly_service.discover(conn_handle) )
  {
    if(ENABLE_DEBUG_OUTPUT) Serial.println("Found NONE");
    // disconnect
    Bluefruit.disconnect(conn_handle);
    return;
  }

  // Once HRM service is found, we continue to discover its characteristic
  if(ENABLE_DEBUG_OUTPUT) {
    Serial.println("Found it");
    Serial.print("Discovering RPC Data characteristic ... ");
  }

  if ( !shelly_rpc_data.discover() )
  {
    // necessary, not found -> disconnect
    if(ENABLE_DEBUG_OUTPUT) Serial.println("RPC data is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if(ENABLE_DEBUG_OUTPUT)  {
    Serial.println("Found it");
    Serial.print("Discovering RPC RX ctl characteristic ... ");
  }

  if ( !shelly_rpc_rx_ctl.discover() )
  {
    // necessary, not found -> disconnect
    if(ENABLE_DEBUG_OUTPUT) Serial.println("RPC RX CTL is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  if(ENABLE_DEBUG_OUTPUT)  {
    Serial.println("Found it");  
    Serial.println("Discovering RPC TX ctl characteristic ... ");
  }

  if ( !shelly_rpc_tx_ctl.discover() )
  {
    // necessary, not found -> disconnect
    if(ENABLE_DEBUG_OUTPUT) Serial.println("RPC TX CTL is mandatory but not found");
    Bluefruit.disconnect(conn_handle);
    return;
  }

  // Reaching here means we are ready to go, let's enable notification on RX CTL
  if ( shelly_rpc_rx_ctl.enableNotify() ) {
    if(ENABLE_DEBUG_OUTPUT)  Serial.println("Ready to receive RX CTL value");
  } else {
    if(ENABLE_DEBUG_OUTPUT)  Serial.println("Couldn't enable notify for RX CTL.");
  }
  // Reaching here means we are ready to go, let's enable notification on RX CTL
  if ( shelly_rpc_rx_ctl.enableIndicate() ) {
    if(ENABLE_DEBUG_OUTPUT)  Serial.println("Ready to receive RX CTL value indicate");
  } else {
    if(ENABLE_DEBUG_OUTPUT)  Serial.println("Couldn't enable indicate for RX CTL.");
  }
}

void printMAC() {
  for(int i = 0; i<6; i++) {
    if(peripheral_mac[i] < 0x10) Serial.print("0");
    Serial.print(peripheral_mac[i],HEX);
    if(i<5) Serial.print(":");
  }
}

void shelly_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  //currently we don't read back RPC answers, so simply print length and do nothing
  //if the data characteristics is not read, the length won't be updated
  //expect this callback firing once when nothing else is done.
  if(ENABLE_DEBUG_OUTPUT) {
    Serial.print("Got RX - Len: ");
    uint32_t rcv_len = 0;
    //we expect a 32bit length field
    if(len == 4) {
      rcv_len = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | (uint32_t)data[3];
    }
    Serial.println(rcv_len);
  }
}


void shelly_send_rpc(const char * cmd) {

    uint8_t length[4];
    uint32_t sent = 0;

    length[3] = strlen(cmd);
    sent = shelly_rpc_tx_ctl.write(length,4);

    if(ENABLE_DEBUG_OUTPUT) {
      Serial.print("Sending len, size sent: ");
      Serial.println(sent);
    }
  
    delay(10);

    sent = shelly_rpc_data.write(cmd, strlen(cmd));
    if(ENABLE_DEBUG_OUTPUT) {
      Serial.print("Sending rpc call, size sent: ");
      Serial.println(sent);
    }

    delay(10);
}

