// This sketch sets up the RTC long-time measurement experiment and then measures the RTCs
// each day.
// 
// The idea is to synchronize all RTCs at a certain date that is then stored in an EEPROM.
// The MCU wakes up shortly before midnight each day, synchronizes with DCF77 and stores the
// deviation for each RTC in the EEPROM together with 24 temperature measurements of the last 24 hours
//
// Configured as an UNO, but EEPROM should be preserved in order to protect re-initialization by accident
//
// Version 0.0.1 (8.3.2023)
//  - basic framework
//  - commands ?, V, T, X work, I and # are just there for show

#define VERSION "0.0.1"
#define BAUD 19200
//#define DEBUG

#define INTREF 1076 // special value for this MCU 

#define DCF_PIN 2
#define DCF_IRQ 0
#define DCF_VCC A5
#define TEMP_VCC 12
#define TEMP_PIN A1

#define DCF_TIMEOUT_MS 600000UL // if no DCF77 sync after 600 secs, we give up
#define INPUT_TIMEOUT_MS 60000UL // wait 1 minute for user inputs, then contine measuring 

#define MAGIC 0x6519454FUL // magic marker

#define ONEWIRE_CRC 0 // save some space
#include <TimeLib.h>
#include <DCF77.h>
#include <OneWire.h>
#include <I2C_24LC1025.h>
#include <EEPROM.h>
#include <Vcc.h>

#include <RTC_I2C.h>
#include <RTC_DS1307.h>
#include <RTC_DS1337.h>
#include <RTC_DS3231.h>
#include <RTC_MCP79410.h>
#include <RTC_PCF8523.h>
#include <RTC_PCF8563.h>
#include <RTC_RS5C372.h>
#include <RTC_RV3028.h>
#include <RTC_RV3028U.h>
#include <RTC_RV3032.h>
#include <RTC_RV8523.h>
#include <RTC_RV8803.h>
#include <RTC_SD2405.h>

#define MAXRTC 13

DS1307 rtc0;
DS1337 rtc1;
DS3231 rtc2; // SN version
DS3231 rtc3; // M  version
MCP79410 rtc4;
PCF8523 rtc5;
PCF8563 rtc6;
RS5C372 rtc7;
RV3028 rtc8;
RV3028U rtc9;
RV3032 rtc10;
RV8523 rtc11;
RV8803 rtc12;
SD2405 rtc13;

typedef struct {
  RTC *rtc;
  byte multiplexer; // which multiplexer
  byte mask; // mask for addressing local bus
  unsigned int offset; // offset value as a binary value
} RTCentry_t;

RTCentry_t rtcentry[MAXRTC] = {
  { &rtc0, 0, 0, 0 }
};

OneWire ds(TEMP_PIN);

bool synced = false;
unsigned long syncstart = 0;
unsigned long lastinput = 0;
unsigned long magic;

void setup(void) {
  Serial.begin(BAUD);
  Serial.println();
  Serial.println(F("RTCEval V" VERSION));
}

void loop(void) {
  char c = '\0';
  Serial.print(F("cmd: "));
  while (c <= ' ') {
    while (!Serial.available())
      if (millis() - lastinput > INPUT_TIMEOUT_MS) {
	process();
	// we never return here
      }
    c = Serial.read();
    if (c > ' ') break;
  }
  lastinput = millis();
  Serial.println(c);
  switch (toupper(c)) {
  case '?':
  case 'H':
    help();
    break;
  case 'I':
    EEPROM.get(0,magic);
    if (magic != MAGIC) {
      initialize(); // only possible if magic not set in EEPROM
      EEPROM.put(0,MAGIC);
    } else {
      Serial.println(F("Already initialized! Clear fist with '#'"));
    }
    break;
  case '#':
    delay(100);
    while (Serial.read() >= 0);
    Serial.print(F("Really restarting from scratch? (Y/N): "));
    delay(500);
    while (!Serial.available());
    c = Serial.read();
    Serial.println(c);
    if ('Y' == toupper(c)) {
      magic = 0xFFFFFFFF;
      EEPROM.put(0,magic);
      Serial.println(F("System has been un-initialized!"));
    } else Serial.println(F("Nothing done"));
    break;
  case 'V':
    Serial.print(F("Supply voltage: "));
    Serial.print(Vcc::measure(500,INTREF));
    Serial.println(F(" mV"));
    break;
  case 'T':
    Serial.print(F("Current temperature: "));
    Serial.println(temperature());
    break; 
  case 'X':
    process();
    // we do not return here
    break;
  default:
    Serial.println(F("Illegal command"));
    break;
  }
}

void help(void) {
  Serial.println(F("H,?  - Help\n\r"
		   "I    - Initialize system\n\r"
		   "#    - Prepare for reinitializing the system\n\r"
		   //		   "L    - show log so far\n\r"
		   //		   "D    - wait for sync with DCF\n\r"
		   //		   "C    - show current state of clocks\n\r"
		   "T    - show current temperature\n\r"
		   //		   "S    - show statistics so far\n\r"
		   "V    - system voltage\n\r"
		   "X    - exit and continue the experiment\n\r"));
}

// get temperature as an integer from a DS18S20
int temperature(void) {
  byte data[9];
  int16_t res;
  pinMode(TEMP_VCC, OUTPUT);
  digitalWrite(TEMP_VCC, HIGH); // power up the sensor
  delay(100); // wait after powering up 
  ds.reset();
  ds.skip();
  ds.write(0x44,1); // start conversion
  delay(1000); // at least 750 ms - give it some leeway
  ds.reset();
  ds.skip();
  ds.write(0xBE); // read scratchpad
  for (byte i = 0; i < 9; i++) data[i] = ds.read();
  res = ((data[1] << 8)|data[0]);
  digitalWrite(TEMP_VCC, LOW);
  return (res>>1);
}

void initialize(void) {
  Serial.println(F("Setting up the experiment ..."));
  delay(1000);
  Serial.println(F("...done"));
}

void process(void) {
  Serial.println(F("\n\rContinuing experiment ..."));
  while (1);
}

