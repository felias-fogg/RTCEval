// This sketch sets up the RTC long-time measurement experiment and then measures the RTCs
// each day.
// 
// The idea is to synchronize all RTCs at a certain date that is then stored in an EEPROM.
// The MCU wakes up shortly before midnight each day, synchronizes with DCF77 and stores the
// deviation for each RTC in the EEPROM together with 24 temperature measurements of the last 24 hours
//
// Configured as an UNO, but EEPROM should be preserved in order to protect re-initialization by accident
//
// Measurements for a day are stored as follows:
// time_t UTC time marker, when the record was started
// 24 temperature readings as signed one byte values. 
// time_t UTC time marker when clocks were compared (MAX_ULONG, if no DCF sync possible)
// 13 signed long time differences in msec to DCF time; time difference to last RTC (8803), when DCF not accessible
// = 84 bytes
// Since there are 128 kByte available, this means 1560 records (or more than 4 years).
// All bytes are initially 0.
// Every hour, the MCU is woken up in order to store a temperature reading. After midnight (UTC),
// the time comparison is made and stored.
// 
//
// Version 0.0.1 (8.3.2023)
//  - basic framework
//  - commands ?, V, T, X work, I and # are just there for show
//
// Version 0.1.0 (8.3.2023)
//  - command U (print UTC time), and D (wait for sync with DCF77) work now
//
// Version 0.2.0 (8.3.2023)
// - command P (check presence of all devices)

#define VERSION "0.2.0"
#define BAUD 19200
//#define DEBUG

#define INTREF 1076 // special value for this MCU 

#define DCF_PIN 2
#define DCF_IRQ 0
#define DCF_VCC 12
#define TEMP_VCC 10
#define TEMP_PIN A1
#define I2C_VCC 7
#define RGROUP1_VCC 5 // the four RTCs without battery backup on the chip
#define RGROUP2_VCC 6 // the remaining 9

// I2C devices
#define MP1ADDR 0x70 // first multiplexer
#define MP2ADDR 0x71 // second multiplexer
#define EEPROMADDR 0x50 // base address of EEPROM
#define EEPROMADDR2 0x54 // second address of EEPROM

#define DCF_TIMEOUT_MS 600000UL // if no DCF77 sync after 600 secs, we give up
#define DCF_WAIT_TIMEOUT_MS 10000UL // wait 10 seconds for at least some signal
#define INPUT_TIMEOUT_MS 60000UL // wait 1 minute for user inputs, then contine measuring
#define NO_TEMP -10000          // no legal temperatur, sensor not present

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

DS1307 rtc1;
DS1337 rtc2;
DS3231 rtc3; // SN version
DS3231 rtc4; // M  version
MCP79410 rtc5;
PCF8523 rtc6;
PCF8563 rtc7;
RS5C372 rtc8;
RV3028 rtc9;
RV3032 rtc10;
RV8523 rtc11;
RV8803 rtc12;
SD2405 rtc13;

typedef struct {
  RTC *rtc;
  byte multiplexer; // which multiplexer
  byte port; // multiplexer port
  unsigned int offset; // offset value as a binary value
} RTCentry_t;

RTCentry_t rtcentry[MAXRTC] = {
  { &rtc1,  MP2ADDR, 4, 0 }, // DS1307
  { &rtc2,  MP1ADDR, 1, 0 }, // DS1337
  { &rtc3,  MP2ADDR, 1, 0 }, // DS3231SN
  { &rtc4,  MP1ADDR, 6, 0 }, // DS3231M
  { &rtc5,  MP2ADDR, 0, 0 }, // MPC79140
  { &rtc6,  MP2ADDR, 3, 0 }, // PCF8523 
  { &rtc7,  MP2ADDR, 5, 0 }, // PCF8563
  { &rtc8,  MP1ADDR, 2, 0 }, // RS5C372
  { &rtc9,  MP2ADDR, 2, 0 }, // RV-3028
  { &rtc10, MP1ADDR, 3, 0 }, // RV-3032
  { &rtc11, MP1ADDR, 4, 0 }, // RV-8523
  { &rtc12, MP1ADDR, 5, 0 }, // RV-8803
  { &rtc13, MP1ADDR, 0, 0 }  // SD2405
};

OneWire ds(TEMP_PIN);

I2C_24LC1025 ee(EEPROMADDR);

DCF77 dcf = DCF77(DCF_PIN, DCF_IRQ, false);

bool synced = false;
unsigned long syncstart = 0;
unsigned long lastinput = 0;
unsigned long magic;

void setup(void) {
  Serial.begin(BAUD);
  Serial.println();
  Serial.println(F("RTCEval V" VERSION));
  pinMode(TEMP_VCC, OUTPUT);
  pinMode(DCF_VCC, OUTPUT);
  pinMode(I2C_VCC, OUTPUT);
  pinMode(RGROUP1_VCC, OUTPUT);
  pinMode(RGROUP2_VCC, OUTPUT);
  digitalWrite(I2C_VCC, HIGH);
  Wire.begin();
  digitalWrite(I2C_VCC, LOW);
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
  case 'D':
    if (waitForDCF77()) {
      Serial.println(F("Synced with DCF77"));
    } else {
      Serial.println(F("Sync with DCF77 impossible"));
    }
    lastinput = millis();
    break;
  case 'I':
    EEPROM.get(0,magic);
    if (magic != MAGIC) {
      initialize(); // only possible if magic not set in EEPROM
      EEPROM.put(0,MAGIC);
    } else {
      Serial.println(F("Already initialized! Clear first with '#'"));
    }
    break;
  case 'P': checkPresence();
    break;
  case 'T':
    Serial.print(F("Current temperature: "));
    Serial.println(temperature());
    break; 
  case 'U':
    printTime(now()); printDate(now()); Serial.println(F(" UTC"));
    break;
  case 'V':
    Serial.print(F("Supply voltage: "));
    Serial.print(Vcc::measure(500,INTREF));
    Serial.println(F(" mV"));
    break;
  case 'X':
    process();
    // we do not return here
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
  default:
    Serial.println(F("Illegal command"));
    break;
  }
}

void help(void) {
  Serial.println(F("H,?  - Help\n\r"
		   //		   "C    - show current state of clocks\n\r"
		   "D    - wait for sync with DCF\n\r"
		   "I    - Initialize system\n\r"
		   //		   "L    - show log so far\n\r"
		   "P    - test for presence of all devices\n\r" 
		   //		   "S    - show statistics so far\n\r"
		   "T    - show current temperature\n\r"
		   "U    - print UCT system time\n\r"
		   "V    - system voltage\n\r"
		   "X    - exit and continue the experiment\n\r"
		   "#    - Prepare for reinitializing the system\n\r"));
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
  for (byte i = 0; i < 9; i++) {
    data[i] = ds.read();
    //Serial.println(data[i], HEX);
  }
  if (data[4] != 0xFF || data[5] != 0xFF || data[7] != 0x10) return NO_TEMP; // no sensor present 
  res = ((data[1] << 8)|data[0]);
  digitalWrite(TEMP_VCC, LOW);
  return (res>>1);
}

void checkPresence(void) {
  digitalWrite(I2C_VCC, HIGH);
  digitalWrite(RGROUP1_VCC, HIGH);
  digitalWrite(RGROUP2_VCC, HIGH);
  digitalWrite(TEMP_VCC, HIGH);
  digitalWrite(DCF_VCC, HIGH);
  Wire.begin();
  Serial.println(F("Devices present"));
  Serial.print(F("DCF77 module:       ")); Serial.println(testDCF());
  Serial.print(F("Temperature sensor: ")); Serial.println(temperature() != NO_TEMP);
  for (byte i=0; i < 2; i++) {
    Serial.print(F("I2C multiplexer #")); Serial.print(i+1); Serial.print(F(": "));
    Serial.println(i2cPresent(MP1ADDR+i));
  }
  for (byte i=0; i<5; i=i+4) {
    Serial.print(F("I2C EEPROM #")); Serial.print((i/4)+1); Serial.print(F(":      "));
    Serial.println(i2cPresent(EEPROMADDR+i));
  }
  for (byte i=0; i<MAXRTC; i++) {
    Serial.print(F("RTC #")); printDigits(i+1,'\0'); Serial.print(F(":            "));
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    Serial.println(rtcentry[i].rtc->begin());
    i2cSwitchOff(rtcentry[i].multiplexer);
  }
  digitalWrite(I2C_VCC, LOW);
  digitalWrite(RGROUP1_VCC, LOW);
  digitalWrite(RGROUP2_VCC, LOW);
  digitalWrite(TEMP_VCC, LOW);
  digitalWrite(DCF_VCC, LOW);
}

bool i2cSwitchOn(byte mpaddr, byte port) {
  Wire.beginTransmission(mpaddr);
  Wire.write(port > 7 ? 0 : (1 << port));
  return (Wire.endTransmission() == 0);
}

bool i2cSwitchOff(byte mpaddr) {
  return i2cSwitchOn(mpaddr, 8);
}

bool i2cPresent(byte addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool testDCF(void) {
  unsigned long start = millis();
  bool level;

  pinMode(DCF_PIN, INPUT_PULLUP);
  level = digitalRead(DCF_PIN);
  while (millis() - start < DCF_WAIT_TIMEOUT_MS) {
    if (digitalRead(DCF_PIN) != level) return true;
  }
  return false;
}

void initialize(void) {
  unsigned long start, stop;
  Serial.println(F("Setting up the experiment ..."));
  Serial.println(F("Clearing EEPROM"));
  if (!ee.begin()) {
    Serial.println(F("EEPROM not found"));
    return;
  }
  start = millis();
  for (uint32_t i = 0; i < I2C_DEVICESIZE_24LC1025; i+=128)
  {
    if (i % 0x1000 == 0) Serial.print('.');
    ee.setBlock(i, 0x00, 128);
  }
  stop = millis();

  Serial.print("\n\rTime: \t");
  Serial.print((stop - start)/1000);
  Serial.println(F( " sec"));
  if (!waitForDCF77()) {
    Serial.println(F("Failure!"));
    return;
  }
  Serial.println(F("...done"));
}

void process(void) {
  Serial.println(F("\n\rContinuing experiment ..."));
  while (1);
}

bool waitForDCF77(void) {
  unsigned long start = millis(), lastdot=0;
  time_t utc;
  int dots = 0;
  pinMode(DCF_VCC, OUTPUT);
  digitalWrite(DCF_VCC, HIGH);
  pinMode(DCF_PIN, INPUT_PULLUP);

  dcf.Start();
  Serial.println(F("Waiting for DCF77 time ... "));
  do {
    utc = dcf.getUTCTime();
    if (utc != 0) {
      syncstart = millis();
      setTime(utc);
      Serial.println();
      return true;
    }
    if (millis() - lastdot >= 1000) {
      lastdot = millis();
      Serial.print('.');
      dots++;
      if (dots == 60) {
	dots = 0;
	Serial.println();
      }
    }
  } while (millis() - start <= DCF_TIMEOUT_MS);
  Serial.println();
  return false;
}

void printTime(time_t t) {
  printDigits(hour(t),':');
  printDigits(minute(t),':');
  printDigits(second(t),' ');
}

void printDate(time_t t) {
  printDigits(day(t),'.');
  printDigits(month(t),'.');
  Serial.print(year(t));
}

void printDigits(byte num, char sep) {
  if (num < 10) Serial.print('0');
  Serial.print(num);
  Serial.print(sep);
}
