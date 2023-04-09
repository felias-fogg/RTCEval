// This sketch sets up the RTC long-time measurement experiment and then measures the RTCs
// each day.
// 
// The idea is to synchronize all RTCs at a certain date that is then stored in an EEPROM togther with their initial deviations.
// after that every hour the temperature is taken and stored. Every 24 hours, the MCU synchronizes with DCF77 and starts a new
// record.
//
// Configured as an UNO, but EEPROM should be preserved in order to protect re-initialization by accident
//
// All bytes are initially 0.
// Measurements for a day are stored as follows.
// time_t UTC time marker, when the record was started (could be all bits set when no DCF77 connection)
// 14 deviation values (as signed long in msec), also initially in order to be able to normalize!
// 24 temperature readings as signed one byte values for the current hour and then each following hour
// temperatures. 0 degrees or lower are stored as temp-1, i.e., 0 means unused, or no valid temperature.
// = 84 bytes
// Since there are 128 kByte available, this means 1560 records (or more than 4 years).
// 
//
// Version 0.0.1 (8.3.2023)
//  - basic framework
//  - commands ?, V, T, X work, I and # are just there for show
//
// Version 0.1.0 (8.3.2023)
//  - added command U (print UTC time)
//  - added D (wait for sync with DCF77) work now
//
// Version 0.2.0 (8.3.2023)
// - added command P (check presence of all devices)
//
// Version 0.3.0 (9.3.2023)
// - command I (initialize) implemented,
// - added C (show clock values);
// - fixed: changed power-down devices to switching to INPUT mode (otherwise RV-8523 had problems)
//
// Version 0.3.1 (21.3.2023)
// - added: during setup, all global and local I2C busses are cleared
// - added: timeout for I2C bus
// - added: Wire.begin() for all functions that address the I2C bus
// - fixed: rtc->begin() added in timedrift function
// - added: contents of offset reg and ppms in displaying time drift
// - added: offset reg values for initialization
//
// Version 0.4.0 
// - added: storing deviation record
// - added: storing one temperature value
// - added: synchronize with reference RTC (8803) in setup
// - added: ATOMIC_BLOCK around time measurements
// - added: computation and display of uptime in setup
// - added: provision for IRQ by RTC, set alarm in initialize, activatated in processing()
// - added: switch off RTC alarm when de-initializing
// - added: check whether logging is necessary after (re-)start (based on IRQ flag)
// - added: rtc0 - another DS1307
//
// Version 0.5.0
// - added: E command - show (internal) EEPROM contents
// - added: M command - show memory block
// - fixed: bug in readLong
// - added: TESTING option - for activating E,R,M commands
// - added: R command - show RTC reg of reference RTC
// - fixed: uptime days/hours are computed with respect to initial Unix time stamp
// - changed: alarm minute is now set as the next minute after everything has been completed.
// - added: first temperature measurement is stored in initialize routine
// - changed: case updays==0 && uphours==0 ignored in checkLogging (since handled in initialize)
// - changed: initial timedrift is displayed without subtracting initial drift
//
// Version 0.5.1
// - changed: write '1' as a marker when DCF sync has not been achieved
// - changed: resync with reference RTC when no DCF sync


#define VERSION "0.5.1"
#define BAUD 115200UL
//#define DEBUG
#define TESTING

#define INTREF 1076 // special value for this MCU 

// pins
#define DCF_PIN 2
#define DCF_IRQ 0
#define RTC_IRQ_PIN 3
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

// timeout values
#define TERM_INPUT_TIMEOUT_MS 15000UL // wait this time for a terminal input, when a number is called for 
#define DCF_TIMEOUT_MS 360000UL // if no DCF77 sync after 360 secs, we give up
#define DCF_WAIT_TIMEOUT_MS 10000UL // wait 10 seconds for at least some signal
#define INPUT_TIMEOUT_MS 60000UL // wait 1 minute for user inputs, then contine measuring
#define NO_TEMP -10000          // no legal temperatur, sensor not present

// EEPROM addresses
#define MAGIC_ADDR 0 // address, where magic 4-byte value is stored
#define STARTTIME_ADDR 4 // address, where the start time (Unix format) is stored (before everything is set & stored)
#define STARTMINUTE_ADDR 8 // address for minute for experiment start (when we we are done with setting everything)
#define STARTHOUR_ADDR 10 // address for start hour (when we we are done with setting everything)

// I2C EEPROM addresses 
#define RECLEN 84 // length of one record
#define TMPSTART 60 // that is were the temperature values start

#define MAGIC 0x6519454FUL // magic marker

#define ONEWIRE_CRC 0 // save some space
#include <util/atomic.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <TimeLib.h>
#include <DCF77.h>
#include <OneWire.h>
#include <I2C_24LC1025.h>
#include <EEPROM.h>
#include <Vcc.h>
#include <I2Cbus.h>
#include <LowPower.h>

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

#define MAXRTC 14
#define REFRTC 12 // index of the reference clock (it's the RV-8803)

DS1307 rtc0; // Adafruit
DS1307 rtc1; // Elecrow
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
  { &rtc0,  MP1ADDR, 7, 0 }, // DS1307 (Adafruit)
  { &rtc1,  MP2ADDR, 4, 0 }, // DS1307
  { &rtc2,  MP1ADDR, 1, 0 }, // DS1337
  { &rtc3,  MP2ADDR, 1, 0x04 }, // DS3231SN (Adafruit)
  { &rtc4,  MP1ADDR, 6, 0xD5 }, // DS3231M
  { &rtc5,  MP2ADDR, 0, 0x9E }, // MPC79140
  { &rtc6,  MP2ADDR, 3, 0x9A }, // PCF8523 
  { &rtc7,  MP2ADDR, 5, 0 }, // PCF8563
  { &rtc8,  MP1ADDR, 2, 0x04 }, // RS5C372
  { &rtc9,  MP2ADDR, 2, 0x1FC }, // RV-3028
  { &rtc10, MP1ADDR, 3, 0x00 }, // RV-3032
  { &rtc11, MP1ADDR, 4, 0x81 }, // RV-8523 (note that a diode in the Vcc supply line is necessary)
  { &rtc12, MP1ADDR, 5, 0x03 }, // RV-8803
  { &rtc13, MP1ADDR, 0, 0x00 }  // SD2405
};

OneWire ds(TEMP_PIN);

I2C_24LC1025 ee(EEPROMADDR);

DCF77 dcf = DCF77(DCF_PIN, DCF_IRQ, false);

bool synced = false;
unsigned long syncstart = 0;
unsigned long lastinput = 0;
unsigned long magic;
int updays = -1, uphours = -1;
int startminute;
int starthour;
time_t starttime;


void setup(void) {
  time_t diff;
  time_t curr;
  Serial.begin(BAUD);
  Serial.println();
  Serial.println(F("RTCEval V" VERSION));
  // initialize everything
  allVccOn();
  Wire.setWireTimeout(10000);
  clearAllI2C(); // clear all local I2C busses
  Wire.begin();
  // read time from ref clock
  i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
  rtcentry[REFRTC].rtc->begin();
  curr = rtcentry[REFRTC].rtc->getTime(true); // return current time (blocked to start of second)
  setTime(curr); // set time synchronized with the reference RTC (will be used when no DCF77 sync is possible)
  i2cSwitchOff(rtcentry[REFRTC].multiplexer);
  printTimeDate(now());
  Serial.println(F(" UTC"));
  // switch on DCF77 module
  pinMode(DCF_VCC, OUTPUT);
  digitalWrite(DCF_VCC, HIGH);
  pinMode(DCF_PIN, INPUT_PULLUP);
  dcf.Start();
  // allow for RTC IRQ
  pinMode(RTC_IRQ_PIN, INPUT_PULLUP);
  EEPROM.get(STARTTIME_ADDR, starttime);
  EEPROM.get(STARTMINUTE_ADDR, startminute);
  EEPROM.get(STARTHOUR_ADDR, starthour);
  EEPROM.get(MAGIC_ADDR,magic);
  if (magic == MAGIC) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      diff = now()-starttime;
      uphours = (diff/SECS_PER_HOUR)%24;
      updays = (diff/SECS_PER_DAY);
    }
    Serial.print(F("Uptime: ")); Serial.print(updays); Serial.print(F(" days and "));
    Serial.print(uphours); Serial.println(F(" hours"));
  }
  checkLogging();
  lastinput = millis();
}

void loop(void) {
  char c = '\0';
  long addr;
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
  Serial.print(c);
  if (toupper(c) != 'M' && toupper(c) != 'R') Serial.println(); 
  switch (toupper(c)) {
  case '?':
  case 'H':
    help();
    break;
  case 'C':
    showClocks();
    lastinput = millis();
    break;
  case 'D':
    showTimeDrift(-1);
    lastinput = millis();
    break;
  case 'I':
    EEPROM.get(MAGIC_ADDR,magic);
    if (magic != MAGIC) {
      initialize(); // only possible if magic not set in EEPROM
    } else {
      Serial.println(F("Already initialized! Clear first with '#'"));
    }
    lastinput = millis();
    break;
  case 'P': checkPresence();
    lastinput = millis();
    break;
  case 'T':
    showTemperature(-1, -1);
    break; 
  case 'U':
    printTimeDate(now()); Serial.println(F(" UTC"));
    break;
  case 'V':
    Serial.print(F("Supply voltage: "));
    Serial.print(Vcc::measure(500,INTREF));
    Serial.println(F(" mV"));
    break;
  case 'W':
    if (waitForDCF77()) {
      Serial.println(F("Synced with DCF77"));
    } else {
      Serial.println(F("Sync with DCF77 impossible"));
    }
    lastinput = millis();
    break;
  case 'X':
    process();
    // we never return here
    break;
  case '#':
    uninitialize();
    break;
#ifdef TESTING
  case 'M': addr = parseInt();
    Serial.println();
    if (addr < 0) Serial.println(F("Illegal block number"));
    else displayMemBlock(addr);
    break;
  case 'E':
    displayEEPROM();
    break;
  case 'R':
    addr = parseHex();
    Serial.println();
    if (addr < 0) Serial.println(F("Illegal register number"));
    else displayRTCReg(addr);
    break;
#endif
  default:
    Serial.println(F("Illegal command"));
    break;
  }
}

void help(void) {
  Serial.println(F("H,?    - provide help text\n\r"
		   "C      - show current state of clocks\n\r"
		   "D      - measure time drift\n\r"
		   "I      - initialize system\n\r"
		   //		   "L    - show log so far\n\r"
		   "P      - test for presence of all devices\n\r"
		   //		   "S    - show statistics so far\n\r"
		   "T      - show current temperature\n\r"
		   "U      - print UCT system time\n\r"
		   "V      - system voltage\n\r"
		   "W      - wait for sync with DCF\n\r"
		   "X      - exit and continue the experiment\n\r"
		   "#      - Prepare for reinitializing the system\n\r"
#ifdef TESTING
		   "E      - show EEPROM params\n\r"
		   "M<num> - display external EEPROM mem block\n\r"
		   "R<num> - read RTC reg of reference RTC\n\r"
#endif
		   ));
}

// show temperature 
// if day and hour >= 0, store the value in the record
void showTemperature(int day, int hour) {
  int temp = temperature();
  Serial.print(F("Temperature: "));
  Serial.println(temp);
  if (day >= 0 && hour >= 0 && temp != NO_TEMP) {
    if (temp <= 0 && temp != INT8_MIN) temp--;
    ee.writeByte(day*RECLEN+TMPSTART+hour, (byte)(temp&0xFF));
  }
}

// return temperature value from DS18S20
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
  res = res>>1;
  digitalWrite(TEMP_VCC, LOW); // power down the sensor
  return res;
}

// show time of all clocks
void showClocks(void) {
  time_t t;
  bool valid;
  Wire.begin();
  Serial.print(F("System:  "));
  printTimeDate(now());
  Serial.println(F(" UTC"));
  for (byte i=0; i<MAXRTC; i++) {
    Serial.print(F("RTC #"));
    printDigits(i,'\0');
    Serial.print(F(": "));
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    rtcentry[i].rtc->begin();
    t = rtcentry[i].rtc->getTime();
    valid = rtcentry[i].rtc->isValid();
    i2cSwitchOff(rtcentry[i].multiplexer);
    printTimeDate(t);
    Serial.print(F(" UTC"));
    if (!valid) Serial.println(F(" ?"));
    else Serial.println();
  }
}


void checkPresence(void) {
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
    Serial.print(F("RTC #")); printDigits(i,'\0'); Serial.print(F(":            "));
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    Serial.println(rtcentry[i].rtc->begin());
    i2cSwitchOff(rtcentry[i].multiplexer);
  }
}


// compute deviation so far comparing it with the initial time
// store it in the appropriate record if day parameter >= 0
// store also current time, if day > 0
void showTimeDrift(int day) { 
  time_t absstart;
  long drift;

  Wire.begin();
  absstart = readTime_t(0);
  if (day < 0) {
    Serial.print(updays);
    Serial.println(F(" days since start"));
  } else if (day == 0) {
    Serial.println(F("Store initial drift"));
  } else {
    Serial.println(F("Log drift after "));
    Serial.print(day);
    Serial.println(F(" days"));
  }    
  for (byte i=0; i<MAXRTC; i++) {
    drift = timeDrift(i);
    if (day >=0) writeLong(day*RECLEN+(i+1)*4,drift);
    if (day != 0) drift = drift - readTime_t(4+(i*4)); // subtract initial drift (execept for initial)
    Serial.print(F("RTC #"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(drift);
    Serial.print(F("ms / "));
    Serial.print(((float)drift)*10e3/(now()-absstart),2);
    Serial.print(F("ppm (0x"));
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    Serial.print(rtcentry[i].rtc->getOffset(),HEX);
    i2cSwitchOff(rtcentry[i].multiplexer);
    Serial.println(')');
  }
}

// compute time drift for one RTC
// assume all supply lines are enabled
// assume that we have synchronized with DCF77 before 
long timeDrift(byte ix) {
  time_t stop, start;
  unsigned long startms, stopms, diff;

  // enable port
  i2cSwitchOn(rtcentry[ix].multiplexer, rtcentry[ix].port);
  // wait until we start a new second
  start = now();
  while (start == now());
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    start = now();
    startms = millis();
  }
  
  // now wait until the RTC starts a new second
  rtcentry[ix].rtc->begin(); // init
  stop = rtcentry[ix].rtc->getTime(true); // blocked call
  stopms = millis();

  // now compute the difference in ms
  diff = (stop - start)*1000-(stopms - startms);

  i2cSwitchOff(rtcentry[ix].multiplexer);
  return diff;
}

void allVccOn(void) {
  pinMode(I2C_VCC, OUTPUT);
  digitalWrite(I2C_VCC, HIGH);
  pinMode(RGROUP1_VCC, OUTPUT);
  digitalWrite(RGROUP1_VCC, HIGH);
  pinMode(RGROUP2_VCC, OUTPUT);
  digitalWrite(RGROUP2_VCC, HIGH);
  pinMode(TEMP_VCC, OUTPUT);
  digitalWrite(TEMP_VCC, HIGH);
  delay(100);
}

void allVccOff(void) {
  delay(100);
  digitalWrite(RGROUP1_VCC, LOW);
  digitalWrite(RGROUP2_VCC, LOW);
  digitalWrite(TEMP_VCC, LOW);
  digitalWrite(I2C_VCC, LOW);
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

  level = digitalRead(DCF_PIN);
  while (millis() - start < DCF_WAIT_TIMEOUT_MS) {
    if (digitalRead(DCF_PIN) != level) return true;
  }
  return false;
}

void clearAllI2C(void) {
  int i2cres;
  if (I2Cbus_clear(A4, A5) < 0) Serial.println(F("Cannot clear global bus"));
  for (byte i=0; i < MAXRTC; i++) {
    Wire.begin(); // enable 
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    i2cres = I2Cbus_clear(A4, A5);
    if (i2cres < 0 || i2cres == 2) {
      Serial.print(F("I2C bus for RTC#")); Serial.print(i);
      if (i2cres < 0) Serial.println(F(" stuck"));
      else Serial.println(F(" recovered"));
    }
    Wire.begin(); // enable again
    i2cSwitchOff(rtcentry[i].multiplexer);
  }
}

void initialize(void) {
  unsigned long start, stop, last, next;
  time_t starttime;
  byte data[128];
  bool good;
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
    ee.readBlock(i, data, 128);
    good = true;
    for (int j=0; j < 128; j++) if (data[j]) good = false;
    if (!good) ee.setBlock(i, 0x00, 128);
  }
  stop = millis();

  Serial.print("\n\rTime: \t");
  Serial.print((stop - start)/1000);
  Serial.println(F( " sec"));
  if (!waitForDCF77()) {
    Serial.println(F("Failure!"));
    return;
  }
  Serial.println(F("Writing first EEPROM record"));
  starttime = now();
  writeTime_t(0, starttime);
  EEPROM.put(STARTTIME_ADDR, starttime);
  Serial.println(F("Setting all RTCs"));
  for (byte i=0; i < MAXRTC; i++) {
    Serial.print(F("Setting RTC #")); Serial.println(i);
    i2cSwitchOn(rtcentry[i].multiplexer, rtcentry[i].port);
    rtcentry[i].rtc->begin(); 
    rtcentry[i].rtc->init();
    rtcentry[i].rtc->setOffset(rtcentry[i].offset,2);
    last = now();
    next = last;
    while (last == next) next = now();
    /* Serial.print("Diff before: ");
       Serial.println((millis()-syncstart)%1000); */
    rtcentry[i].rtc->setTime(next);
    /*    Serial.print("Diff after:  ");
	  Serial.println((millis()-syncstart)%1000); */
    i2cSwitchOff(rtcentry[i].multiplexer);
  }
  Serial.println(F("...done"));
  showTimeDrift(0);
  showTemperature(0,0);
  magic = MAGIC;
  EEPROM.put(MAGIC_ADDR, magic); // mark that we are initialized
  starthour = hour();
  startminute = minute()+1;
  if (startminute == 60) {
    startminute = 0;
    starthour = (starthour+1)%24;
  }
  EEPROM.put(STARTHOUR_ADDR, starthour);
  EEPROM.put(STARTMINUTE_ADDR, startminute);
  i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
  rtcentry[REFRTC].rtc->begin();
  rtcentry[REFRTC].rtc->clearAlarm(); // clear alarm (just to be sure)
  rtcentry[REFRTC].rtc->setAlarm(startminute); // set alarm 
  rtcentry[REFRTC].rtc->enableAlarm(); // enable alarm
  i2cSwitchOff(rtcentry[REFRTC].multiplexer);
  uphours = 0;
  updays = 0;
}

void uninitialize(void) {
  char c;
  delay(100);
  while (Serial.read() >= 0);
  Serial.print(F("Really restarting from scratch? (Y/N): "));
  delay(500);
  while (!Serial.available());
  c = Serial.read();
  Serial.println(c);
  if ('Y' == toupper(c)) {
    magic = 0xFFFFFFFF;
    EEPROM.put(MAGIC_ADDR, magic);
    i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
    rtcentry[REFRTC].rtc->begin();
    rtcentry[REFRTC].rtc->clearAlarm(); // clear alarm    
    rtcentry[REFRTC].rtc->disableAlarm(); // disable alarm
    i2cSwitchOff(rtcentry[REFRTC].multiplexer);
    Serial.println(F("System has been un-initialized!"));
  } else Serial.println(F("Nothing done"));
}

void process(void) {
  dcf.Stop();
  pinMode(DCF_PIN, INPUT);
  pinMode(DCF_VCC, INPUT);
  allVccOff();
  Wire.end();
  Serial.println(F("\n\rContinuing experiment ..."));
  ADCSRA = 0;
  delay(1000); // wait for finishing printout
  attachInterrupt(digitalPinToInterrupt(RTC_IRQ_PIN), rtcWakeup, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_ON);
  Serial.println(F("\nWoken up"));
  delay(1000); // wait until print to terminal ended
  wdt_enable(WDTO_15MS);
  while (1); // force reset
}

void rtcWakeup(void) { // do nothing, but simply return, which will force a reset
  detachInterrupt(digitalPinToInterrupt(RTC_IRQ_PIN));
}

bool waitForDCF77(void) {
  unsigned long start = millis(), lastdot=0;
  time_t utc;
  int dots = 0;
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
  Serial.println(F("DFC77 receiver timeout"));
  return false;
}

void printTimeDate(time_t t) {
  printTime(t);
  printDate(t);
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

void writeTime_t(unsigned long addr, time_t t) {
  byte data[4] = { (byte)(t & 0xFF), (byte)((t>>8) & 0xFF), (byte)((t>>16) & 0xFF), (byte)((t>>24) & 0xFF) };
  /*  Serial.println();
  Serial.println((byte)(t & 0xFF),HEX);
  Serial.println((byte)((t>>8) & 0xFF),HEX);
  Serial.println((byte)((t>>16) & 0xFF),HEX);
  Serial.println((byte)((t>>24) & 0xFF),HEX); */
  ee.writeBlock(addr, data, 4);
}

void writeLong(unsigned long addr, long val) {
  byte data[4] = {  (byte)(val & 0xFF),  (byte)((val>>8) & 0xFF),
		    (byte)((val>>16) & 0xFF),  (byte)((val>>24) & 0xFF) };
  /*  Serial.print(F("writeLong: ")); Serial.print(val); Serial.print('/'); Serial.println(val,HEX);
      Serial.print(F("bytes: ")); Serial.print(data[0], HEX); Serial.print(' '); Serial.print(data[1], HEX); Serial.print(' '); Serial.print(data[2], HEX); Serial.print(' '); Serial.println(data[3], HEX); */
  ee.writeBlock(addr, data, 4);
}

time_t readTime_t(unsigned long addr) {
  byte data[4];
  ee.readBlock(addr,data,4);
  /*  Serial.println();
  Serial.println(data[0],HEX);
  Serial.println(data[1],HEX);
  Serial.println(data[2],HEX);
  Serial.println(data[3],HEX); */
  return ((time_t)data[0] | (((time_t)data[1])<<8) | (((time_t)data[2])<<16) | (((time_t)data[3])<<24));
}

long readLong(unsigned long addr) {
  byte data[4];
  long val;
  ee.readBlock(addr,data,4);
  val = ((long)data[0] | (((long)data[1]<<8)&0xFF00L) |
	 (((long)data[2]<<16)&0xFF0000L) | (((long)data[3]<<24)&0xFF000000L));
  /*  Serial.print(F("readLong: ")); Serial.print(val); Serial.print('/'); Serial.println(val,HEX);
  Serial.print(F("bytes: ")); Serial.print(data[0], HEX); Serial.print(' '); Serial.print(data[1], HEX); Serial.print(' '); Serial.print(data[2], HEX); Serial.print(' '); Serial.println(data[3], HEX); */
  return val;
}

int8_t readInt8(unsigned long addr) {
  return (int8_t)(ee.readByte(addr));
}

    

// check whether we have something to log
// if so, do it!
void checkLogging(void) {
  bool alarm;
  bool dcf77on;
  if ((magic == MAGIC) && (uphours != 0 || updays != 0)) {
    // only if experiment started and not first hour
    i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
    rtcentry[REFRTC].rtc->begin();
    alarm = rtcentry[REFRTC].rtc->senseAlarm();
    i2cSwitchOff(rtcentry[REFRTC].multiplexer);
    if (!alarm) { // if no alarm has been rasied, then return
      return;
    }
    Serial.println(F("Logging ..."));
    if (uphours != 0) {
      showTemperature(updays, uphours);
    } else { // start a new day record
      dcf77on = waitForDCF77();
      if (dcf77on) 
	writeTime_t((unsigned long)updays*RECLEN,now());
      else {
	writeTime_t((unsigned long)updays*RECLEN,1); // if no DCF77 time, write 1 as a marker!
	i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
	rtcentry[REFRTC].rtc->begin();
	setTime(rtcentry[REFRTC].rtc->getTime(true));
	i2cSwitchOff(rtcentry[REFRTC].multiplexer);
      }
      showTimeDrift(updays);
      showTemperature(updays, uphours);
    }
  }
  i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
  rtcentry[REFRTC].rtc->begin();
  rtcentry[REFRTC].rtc->clearAlarm();
  i2cSwitchOff(rtcentry[REFRTC].multiplexer);
  process();
}

void displayEEPROM(void) {
  Serial.print(F("Start time: ")); printTimeDate(starttime); Serial.println();
  Serial.print(F("Start hour: ")); Serial.println(starthour);
  Serial.print(F("Start min:  ")); Serial.println(startminute);
}

void displayMemBlock(unsigned int rec) {
  time_t begin;
  long dev;
  int8_t temp;
  begin = readTime_t(rec*RECLEN);
  Serial.print(F("Begin: ")); printTimeDate(begin); Serial.print(F(" (")); Serial.print(begin); Serial.println(F(")"));
  for (byte i=0; i < MAXRTC; i++) {
    dev = readLong(rec*RECLEN+(i+1)*4);
    Serial.print(F("Dev #"));
    printDigits(i, '\0');
    Serial.print(F(": "));
    Serial.println(dev);
  }
  Serial.print(F("Temps: "));
  for (byte i=0; i < 24; i++) {
    temp = readInt8((rec*RECLEN)+TMPSTART+i);
    if (temp == 0) Serial.print(F("---"));
    else if (temp == -1) Serial.print(F("  0"));
    else if (temp < -1) {
      Serial.print('-');
      printDigits(temp+1, '\0');
    } else {
      Serial.print(' ');
      printDigits(temp, '\0');
    }
    Serial.print(' ');
  }
  Serial.println();
}

long parseInt(void) {
  int c = -1;
  unsigned long start= millis();
  long res = 0;
  bool sign = false, numstart = false;

  while (millis() - start < TERM_INPUT_TIMEOUT_MS) {
    c = Serial.read();
    if (c < 0) continue;
    Serial.print((char)c);
    start = millis();
    if (!numstart) {
      if (!isDigit(c) && c != '-' && c != '+') continue;
      else {
	numstart = true;
	if (c == '-' || c == '+') {
	  sign = ( c == '-' );
	  continue;
	}
      }
    }
    if (isdigit(c)) res = res*10 + c - '0';
    else return(sign ? -res : res);
  }
  return -1;
}
	  

void displayRTCReg(int reg) {
  byte val;
  i2cSwitchOn(rtcentry[REFRTC].multiplexer, rtcentry[REFRTC].port);
  rtcentry[REFRTC].rtc->begin();
  val = rtcentry[REFRTC].rtc->getRegister((byte)reg);
  i2cSwitchOff(rtcentry[REFRTC].multiplexer);
  Serial.print(F("RTC reg: 0x")); Serial.println(val,HEX);
}

int checkHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  else if (toupper(c) >= 'A' && toupper(c) <= 'F') return toupper(c) - 'A' + 10;
  else return -1;
}
  
int parseHex(void) {
  int c = -1;
  unsigned long start= millis();
  long res = 0;

  while (millis() - start < TERM_INPUT_TIMEOUT_MS) {
    c = Serial.read();
    if (c < 0) continue;
    Serial.print((char)c);
    start = millis();
    if (checkHex(c) < 0) return res;
    res = res*16+checkHex(c);
  }
  return -1;
}
	  
