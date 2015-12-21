/*
  Firmata is a generic protocol for communicating with microcontrollers
  from software on a host computer. It is intended to work with
  any host computer software package.

  To download a host software package, please clink on the following link
  to open the list of Firmata client libraries your default browser.

  https://github.com/firmata/arduino#firmata-client-libraries

  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2015 Jeff Hoefs.  All rights reserved.
  Copyright (C) 2015 Sergei Kotlyachkov. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated by Sergei Kotlyachkov, Dec 20, 2015.
*/

/*
  README

  To use this Firmata you will need to have one of the following
  boards or shields:

  - NodeMCU
*/
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>

// #define SERIAL_DEBUG

#include "utility/firmataDebug.h"
#include <Firmata.h>

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   10

#include <ESP8266WiFi.h>

#define SSID_SIZE 32
#define PASS_SIZE 64

char ssid[SSID_SIZE+1];
char password[PASS_SIZE+1];
uint32_t remote_ip;
uint16_t remote_port;
/*
 * buf   - char buffer. must have length of size+1
 * index - in EEPROM to read from
 * size  - how many chars to read from EEPROM. buf[size] will be cleared with null terminating char.
 */
void readROM(char *buf, int romIndex, int size) 
{
  for (int i = 0; i < size; ++i)
  {
    buf[i] = char(EEPROM.read(i+romIndex));
  }
  buf[size] = 0;
}

void writeROM(char *buf, int romIndex, int size) 
{
  for (int i = 0; i < size; ++i)
  { 
    EEPROM.write(i+romIndex, buf[i]);
  }
}

/*==============================================================================
   GLOBAL VARIABLES
  ============================================================================*/

/* network */

#include "utility/EthernetClientStream.h"

/* analog inputs */
int analogInputsToReport = 0;      // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 10; // how often to sample analog inputs (in ms)


boolean isResetting = false;

// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
*/
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(PIN_TO_ANALOG(analogPin)));
      }
    }
  }
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
   two bit-arrays that track Digital I/O and PWM status
*/
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == PIN_MODE_IGNORE)
    return;

  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(pin, mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  pinState[pin] = 0;

  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
          digitalWrite(PIN_TO_DIGITAL(pin), HIGH); // disable internal pull-ups
        } else { if (IS_PIN_PWM(pin)) {
          pinMode(PIN_TO_PWM(pin), INPUT);    // disable output driver
        }}
        pinConfig[pin] = ANALOG;
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); 
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case PWM:
        if (IS_PIN_PWM(pin)) {
          analogWrite(PIN_TO_PWM(pin), value);
        }
        pinState[pin] = value;
        break;
    }
  }
}

/*==============================================================================
   SYSEX-BASED commands
  ============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PWM);
          Firmata.write(8); // 8 = 8-bit resolution
        }
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      //Firmata.flush();
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write((byte)pinConfig[pin]);
          Firmata.write((byte)pinState[pin] & 0x7F);
          if (pinState[pin] & 0xFF80) Firmata.write((byte)(pinState[pin] >> 7) & 0x7F);
          if (pinState[pin] & 0xC000) Firmata.write((byte)(pinState[pin] >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
        //Firmata.flush();
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? pin : 127);
      }
      Firmata.write(END_SYSEX);
      //Firmata.flush();
      break;
  }
}

/*==============================================================================
   SETUP()
  ============================================================================*/

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;
  isResetting = false;
}

void printWifiStatus() {
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
  } else {
    Serial.println("WiFi not connected");
    Serial.println("Only serial interface will be available to setup AP settings");
  }
  Serial.flush();
}

WiFiClient client;
EthernetClientStream *stream = NULL;

void setup()
{
  // We start serial anyway, if we are debuging or not.
  Serial.begin(57600); 
  EEPROM.begin(128);
  readROM(ssid, 0, SSID_SIZE);
  readROM(password, SSID_SIZE, PASS_SIZE);
  EEPROM.get(SSID_SIZE+PASS_SIZE, remote_ip);
  EEPROM.get(SSID_SIZE+PASS_SIZE+sizeof(remote_ip), remote_port);
  EEPROM.end();
  stream = new EthernetClientStream(client, IPAddress(0, 0, 0, 0), IPAddress(remote_ip), NULL, remote_port);
  
  int attempts = 0;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(500);
    DEBUG_PRINT(".");
    attempts++;
  }
  printWifiStatus();
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  // start up Network Firmata:
  Firmata.begin(*stream);
  systemResetCallback();  // reset to default config
  DEBUG_PRINTLN("setup done.");
}

void executeSerialCommand(String command) {
  DEBUG_PRINT("Executing command:");
  DEBUG_PRINTLN(command);
  EEPROM.begin(128);
  if (command.indexOf("set ssid ") == 0) {
    String s = command.substring(9);
    s.toCharArray(ssid, SSID_SIZE);
    ssid[s.length()] = 0;   
    writeROM(ssid, 0, SSID_SIZE);
    Serial.print("ssid was set:");
    Serial.println(s);
  } else {
  if (command.indexOf("set pass ") == 0) {
    String s = command.substring(9);
    s.toCharArray(password, PASS_SIZE);
    password[s.length()] = 0;
    writeROM(password, SSID_SIZE, PASS_SIZE);
    Serial.println("password was set.");
  } else {
  if (command.indexOf("set remote ip ") == 0) {
    String s = command.substring(14);    
    char* ipAddr = (char*) &remote_ip;
    int offset = 0;
    for (int i = 0; i<4 ; i++) {
      int idx = s.indexOf('.', offset);
      if (idx > 0) {
        String num = s.substring(offset, idx);
        ipAddr[i] = num.toInt();
        offset = idx+1;
      } else {
        String num = s.substring(offset);
        ipAddr[i] = num.toInt();
        break;                
      }
    }
    EEPROM.put(SSID_SIZE+PASS_SIZE, remote_ip);
    Serial.println("remote_ip was set.");
  } else {
  if (command.indexOf("set remote port ") == 0) {
    String s = command.substring(16);
    remote_port = s.toInt();
    EEPROM.put(SSID_SIZE+PASS_SIZE+sizeof(remote_ip), remote_port);
    Serial.print("remote_port was set:");
    Serial.println(s);
  } else {
  if (command.indexOf("connect") == 0) {
    delete stream;
    stream = new EthernetClientStream(client, IPAddress(0, 0, 0, 0), IPAddress(remote_ip), NULL, remote_port);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to Wifi...");
  } else {
  if (command.indexOf("status") ==0 ) {
    printWifiStatus();
  }}}}}}
  EEPROM.commit();
  EEPROM.end();
  Serial.flush();
}

/*==============================================================================
   LOOP()
  ============================================================================*/
void loop()
{
  byte pin;
  if(Serial.available()) {
    executeSerialCommand(Serial.readStringUntil('\n'));
  }
  /* STREAMREAD - processing incoming messagse as soon as possible, while still
     checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        if (analogInputsToReport & (1 << pin)) {
          int val = analogRead(PIN_TO_ANALOG(pin));
          Firmata.sendAnalog(pin, val);
        }
      }
    }
  }
}
