/// popcEsp Arduino UNO / ESP8266 Controller / Artisan Logger with MQTT client 'popc'
// 
//  Sections (units) in this code, ordered alphabetically:
//  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
//  eprm  EEPROM setup and handling 
//  lcds  support for I2C 2x16 LCD display                             
//  mill  fast ~1mSec sequencer inc A/D, Off-On, modulo variable freq/pwmd
//  mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
//  offn  On-Off <= mains cycle/2 rate cycle PSW for SSR control (eg heater )
//  pidc  PID controller for PWM powered temperature control; a delta-time summing anti windup PID 
//  pwmd  8-bit PWM control via freq >= ~30Hz hardware pwm pins 
//  prof  Profile control; selects auto/manual temp setpt, manual pwm width, real/fake temp sensor
//  rots  Rotary 16way encoded ( 4pin + common) selector switch manager
//  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
//  twio  PCD8574 I2C 8b IO extender for ESP 4b In, 4b Out: Rot Sw, indl, SSR on/off drives
//  user  receive user's or Artisan commands via serial port, MQTT for setpoint, ramp, profiles 
//    
//  Command - Response supported for Artisan interface:
//                 ** 'CHA' cmd causes auto-switch into Artisan speak            
//  CHA(N)    #    Acknowlege channel setup command, no action, respond '#' **
//  FILT      #    Acknowlege filter parms  command, no action, respond '#'
//  IO3 nn         Set duty cycle to nn <= 100    
//  OT1 nn         (Set duty cycle #1 to nn <= 100), no action, respond '#'
//  OT2 nn         (Set duty cycle #2 to nn <= 100), no action, respond '#'
//  PID;OFF        Reset PID, set PWM <= 0, set ROC <= 0, set PID Runs <= 0 
//  PID;RESET      Reset PID, zero all internal PID computations, run status unchanged
//  PID;SV nnn     Set new target setpoint temp to nnn
//  PID;SYNC       Zero all internal PID computations, set target temp to sensed temp
//  PID;T;pp,ii,dd Set new PID prop, Integral, Differential gain values              
//  POPC           Exit Artisan Mode, PopC <=> Serial  
//  REA(D)         Send Artisan formatted response line
//  UNIT;C         Set units to Centigrade 
//  UNIT;F         Set units to Fahrenheit
//                 ** 'CHA' cmd causes auto-switch into Artisan speak            
// 
//  Command & LCD Indicators; Upcase: User Set; LowCase: Auto/Sensed/Readback value 
//  a/A     Set serial interface to Artisan talk
//  b/B
//  CHAN    (Auto from Artisan) Set Artisan speak
//  c/C     Set centigrade units; LCD display actual/target temp
//  d/D     toggle diagnostic verbose messages on serial 
//  e/E     Readback / Update From/To EEPROM PID parameters 
//  f/F     Set fahrenheit units; LCD display actual/target temp
//  g/G     Set gate TmpC /
//  h/H     Set hold temperature setpoint for ramp endpoint (soak temp )
//  Iff     Set PID I-Term time (Float Ti: lower value == higher gain)
//  Jff     Set PID D-Term gain (Float Td: lower value == lower  gain)
//  k/K     
//  l/L     Send Artisan CSV format on serial ( for capture and Artisan Import )  
//  m/M     Rsvd MQTT msg / Set bean Mass for virtual tcpl  
//  n/N     Rsvd NetSock  / PID oN
//  o       Rsvd          / PID Off
//  p/Pff   Readback / Set PID P-Term gain (Float Kp: lower value == lower  gain)
//  q/Q     
//  rnn/Rnn Set Temperature Ramp C/F Deg per min (Set before hold temp) 
//  snn/Snn Set immediate target setPoint C/F temperature  
//  t/T     Spare
//  u/U     Set PWM Frequency Hz (TBD)  
//  v/V     Readback Version, PID, EEPROM to serial / EPROM => PID Values
//  wnn/Wnn Set PWM Duty Cycle nn <= 100, disable PID
//  x/X
//  y/Y     Live Sync PID to current power setting / ESP restart
//  z/Z     Zero reset all timecounts / Reset PID 
//
//  Copyright (c) 2017 2018 Bitwise Technologies  popc@bitwisetech.com  
//
//  This program is free software; you can redistribute it and/or                  
//  modify it under the terms of the GNU General Public License as                 
//  published by the Free Software Foundation; either version 2 of the             
//  License, or (at your option) any later version.                                
//
//  This program is distributed in the hope that it will be useful, but            
//  WITHOUT ANY WARRANTY; without even the implied warranty of                     
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU              
//  General Public License for more details.                                       
//
//  You should have received a copy of the GNU General Public License              
//  along with this program; if not, write to the Free Software                    
// .Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA 
//
/// Compiling this sketch
//    for ESP8266   
//      Setup your Arduino IDE for ESP8266, see www.adafruit.com and install board support package
//      Get the ESP TickerScheduler package, copy TickerScheduler.* into the same folder as this sketch, not into library folder.
//      Set '1' compile-time switches, below:  PROC_ESP           ( set unused switches to '0')
//    for UNO
//      Set '1' compile-time switches, below:  PROC_UNO, ( set unused switches to '0')
//    for either processor 
//      Set '1' compile-time switches, below:  WITH_LCD, WITH_MAX31855, WITH_MAX6675, WITH_PCF8574 if you have that hardware
//      Your sketchbook/library must contain libraries:  LiquidCrystal, Adafruit_MAX31855_library (for UNO), MAX31855 (for ESP), PCF8574 as needed
//      For ESP: Adafruit_ESP8266 plus libraries for wifi as selected: esp-mqtt-arduino, WiFiManager, WebSockets  ( not all wifi options tested ! ) 
//               Copy from /.arduino15/packages/esp8266/hardware/esp8266/2.3.0/libraries/Ticker/Ticker.h
//
//  Code compiler switches: 1/0 Enab/Dsel UNO-ESP proc HW, Wifi options - Rebuild, Upload after changing these 
#define PROC_ESP      0                  // Compile for ESP8266
#define PROC_NMCU     1                  // Compile for ESP with NodeMCU pins
#define PROC_UNO      0                  // Compile for Arduino Uno
#define IFAC_ARTI     1                  // Start with Artisan interface on Serial
#define WITH_LCD      1                  // Hdwre has I2C 2x16 LCD display of either type
#define WITH_MAX31855 0                  // Hdwre has MAX31855 thermocouple + circuit
#define WITH_MAX6675  1                  // Hdwre has MAX6675  thermocouple + circuit
#define WITH_VIRTTCPL 0                  // No hdwre, simulate virtual thermocouple output
#define WITH_TCPL_2   1                  // Second Thermocouple MUST be same type ( libraies conflict ) 
#define WITH_PCF8574  0                  // Hdwre  as I2C I/O Extender      
#define WITH_OFFN     1                  // Use 250mSec cycle via mill Off-On SSR
#define WITH_PWMD     0                  // Use fast h/w PWM
#define WITH_WIFI     0                  // Compile for Wifi MQTT client (must have TickerScheduler.h .c in folder)
#define WIFI_MQTT     0                  // Compile for Wifi MQTT client
#define WIFI_SOKS     0                  // Compile for Wifi Web Sckt Srvr
#define WIFI_WMAN     0                  // Compile for Wifi Manager
//
#define SHOW_OPT1     0                  // Optional alternative display
//
#define AMBI_TMPC    19                  // Set 'Room Temperature' degC, returned on uninstalled sensor channels
#define IDLE_TMPC    48                  // Set 'Idle Temperature' degC, returned as temp with eg permanent heater 
#define INIT_SETP   IDLE_TMPC            // Set 'Maxi Temperature' degC  allowed temp for PID setpoint 
#define INIT_PWMD     0                  // Set 'Maxi Temperature' degC  allowed temp for PID setpoint 
#define MAXI_TMPC   269                  // Set 'Maxi Temperature' degC  allowed temp for PID setpoint 
///
// ESP Pin Assignments 
#if (PROC_ESP && !PROC_NMCU)
// A/D 1v 10b on ADC Pin
// Handle either LED polarity with: (1 & ~ONBD_LOWON) for light, (0 | LED_LOWN) for dark 
#define ONBD_OPIN   0                    // Pin0 ESP Red led low on )
#define ONBD_LOWON  1
//
// Off/On SSR driver GPIO 16 deepWake     
#define OFFN_OPIN  16
// PWM Drive SSR driver GPIO 2 BLed  
#define PWMD_OPIN   2                    // Pin2 ESP Blue led low off )
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ 128
// spi on ESP FOR tcpl
#define TCPL_MISO  12  // SPI Mstr In Slve Out 
#define TCPL_CLCK  14  // SPI SCk
#define TCPL_CSEL  15  // SPI ChipSel 10K Gnd 
#if WITH_TCPL_2
#define TCPL_CSL2  13  // SPI ChipSel 10K Gnd 
#endif
// PCF8574 ESP: ROTS Rotary 16way encoder switch;
// i2c on esp for TWIO pcf8574
#define TWIO_SDA    4     // I2C SDA
#define TWIO_SCL    5     // I2C SCL
// 
#define SCOP_OPIN   0    // debug flag uses 'MOSI' line  
//
#endif   // PROC_ESP Non PROC_NMCU
///
#if PROC_NMCU
// A/D 1v 10b on 
// Handle either LED polarity with: (1 & ~ONBD_LOWON) for light, (0 | LED_LOWN) for dark 
#define ONBD_OPIN   0
#define ONBD_LOWON  1
// Off/On SSR driver GPIO 16 deepWake     +
#define OFFN_OPIN  16
// PWM Drive SSR driver GPIO 2 BLed  
#define PWMD_OPIN   2
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ 128
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3   3                        // Pin Val 8 
#define ROTS_BIT2   2                        // Pin Val 4 
#define ROTS_BIT1   1                        // Pin Val 2 
#define ROTS_BIT0   0                        // Pin Val 1 
// Ensure for direct connected pins in Init(), Valu()
#define PINS_ROTS   1
// spi on ESP FOR tcpl
#define TCPL_MISO  12  // SPI Mstr In Slve Out 
#define TCPL_CLCK  14  // SPI SCk
#define TCPL_CSEL  15  // SPI ChipSel 10K Gnd 
#if WITH_TCPL_2
#define TCPL_CSL2  13  // SPI ChipSel 10K Gnd 
#endif
// PCF8574 ESP: ROTS Rotary 16way encoder switch;
// i2c on esp for TWIO pcf8574
#define TWIO_SDA    4     // I2C SDA
#define TWIO_SCL    5     // I2C SCL
// 
#define SCOP_OPIN  13    // debug flag uses 'MOSI' line  
// ensure ESP selections are active
#define PROC_ESP    1
//
#endif   // PROC_NMCU
///
// UNO pin assignments
#if PROC_UNO
// Off/On SSR driver 
#define OFFN_OPIN  16
// Onboard Led indicates duty cycle
#define ONBD_OPIN LED_BUILTIN
#define ONBD_LOWON 0         
// PWM Drive
// d9 needed by RFI scan  d6 would use tmr0 want d3 used by max13855
#define PWMD_OPIN  9                        // Pin D9
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ  123                      // UNO: timer counter range
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3  6                        // Pin Val 8 
#define ROTS_BIT2 10                        // Pin Val 4 
#define ROTS_BIT1 11                        // Pin Val 2 
#define ROTS_BIT0 12                        // Pin Val 1 
// Ensure for direct connected pins in Init(), Valu()
#define PINS_ROTS  1
// spi2 on UNO for alternative tcpl interface  (excludes twio)
#define SPI2_CLCK  3                        // Pin Clock
#define SPI2_MISO  5                        // (Pin D4 used for TCPL)
#define SPI2_CSEL  2                        // Pin CSel
#if WITH_TCPL_2
#define TCPL_CSL2 13                        // tbd is this pin OK ?
#endif
//#define RSVD_MOSI  5                        // Pin D5 Rsvd 
// spi on uno FOR tcpl
#define TCPL_CLCK  4                        // Pin Clock
#define TCPL_MISO  7                        // Pin Data
#define TCPL_CSEL  8                        // Pin CSel
// i2c on pcf8574 TWIO (excl spi2)
#define TWIO_SDA   5     // I2C SDA
#define TWIO_SCL   3     // I2C SCL
//
#define SCOP_OPIN 13     // debug flag nixes SPI2_CSEL
///
#endif   // PROC_UNO

// macros to toggle scope output pin specified above for logic analyser
#define scopHi digitalWrite( SCOP_OPIN, 1)
#define scopLo digitalWrite( SCOP_OPIN, 0)

// shorthand
#define SePrn Serial.print
#define SePln Serial.println
#define DbPrn(tName)  SePrn(F(\"# DbPrn: \"));SePln(tName);

///
//   WiFi preamble for ESP with Wifi Router ID, PW Info 
#if ( PROC_ESP && WITH_WIFI) 
#include <Adafruit_ESP8266.h>
//
#define UPWDSSID   "myRouterAddx"
#define UPWDPSWD   "myRouterAddx"
#define DWNDSSID   "myAPAddx"
#define DWNDPSWD   "myAPPswd"
#define DWNDADDX   10,1,1,1
#define DWNDPORT   80
#define MQTTADDX   "test.mosquitto.org"
#define MQTTPORT   1883
//
// upward wifi: replace with your own network router's SSID, Password
const char* upwdSsid = UPWDSSID;
const char* upwdPswd = UPWDPSWD;
//
// downward wifi: for Esp8266 as an Access Point replace with AP's SSID, Password
const char* dnwdSsid = DWNDSSID;
const char* dnwdPswd = DWNDPSWD;
//
// these incs via popcShed ingo MQTT with tickScheduler 
// 
#include <ESP8266WiFi.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>

//  Mqtt
#if WIFI_MQTT
#include <MQTT.h>
#define MQCL_ID "popc"
//  Mqtt
// create MQTT object with IP address, port of MQTT broker e.g.mosquitto application
// MQTT myMqtt(MQCL_ID, "test.mosquitto.org", 1883);
//
// 
MQTT popcMqtt(MQCL_ID, MQTTADDX, MQTTPORT);
#endif  // WIFI_MQTT

//Jn01 WifiManager 
#if WIFI_WMAN
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#endif  // WIFI_WMAN
//
#if WIFI_SOKS
//  Jn02 arduWWebSockets
//Jn03 wsokSrvr
#include <WebSocketsServer.h>
#include <Hash.h>
//  WebSockets
ESP8266WiFiMulti WiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(5981);
#define USE_SERIAL Serial
#endif // WIFI_SOKS
//
// from another copy ??
#include <dummy.h>
#endif  // ( PROC_ESP && WITH_WIFI)

/// system settings 
//  milliSecond poll values Primes to suppress beating 
#define ADC0_POLL_MILL     2UL           // A/D mill steps per cycle
#define LCDS_POLL_MSEC  1000UL           // mS lcd display poll
#define MILL_POLL_USEC  5000UL           // uS 200Hz  mill poll 
#define MILL_STEP_PSEC   200UL           // mill steps per sec 
#define PIDC_POLL_MSEC   101UL           // mS pid control poll
#define PROF_POLL_MSEC   997UL           // mS run control poll
#define PWMD_POLL_MSEC   103UL           // mS pwm driver  poll
#define ROTS_POLL_MSEC   503UL           // mS rotary sw   poll
#define TCPL_POLL_MSEC   253UL           // mS termocouple poll
#define USER_POLL_MSEC   101UL           // mS user cmd    poll
#define VTCP_POLL_MSEC   251UL           // mS virt tcpl   poll
#define POLL_SLOP_MSEC     1UL           // Avge loop time is 10mSec
//
#if 0
// milliSecond poll values
#define ADC0_POLL_MILL    2UL            // mill count for A/D 
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define MILL_POLL_USEC 5000UL            // uS 200Hz mill  poll
#define MILL_STEP_PSEC  200UL            // mill steps per sec 
#define PIDC_POLL_MSEC  100UL            // mS pid control poll
#define PROF_POLL_MSEC 1000UL            // mS run control poll
#define PWMD_POLL_MSEC  100UL            // mS pwm driver  poll
#define ROTS_POLL_MSEC  500UL            // mS rotary sw   poll
#define TCPL_POLL_MSEC  250UL            // mS termocouple poll
#define USER_POLL_MSEC  100UL            // mS user cmd    poll
#define VTCP_POLL_MSEC  250UL            // mS virt tcpl   poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec
#endif 
//
typedef unsigned int  uint;

// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif
//

/// library files and addx assignments for options
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>  // Comes with Arduino IDE

// LCD: I2C 2x16                                      
// UNO: Analog Pin A4 SDA Pin A5 SCL  I2C  ESP: GPIO Pin D4 SDA  GPIO Pin D5 SCL 
// set LCD address to 0x27 for a A0-A1-A2  display
//   args: (addr, en,rw,rs,d4,d5,d6,d7,bl,blpol)
#if WITH_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif  // any LCD type installed  

//  MAX31855 Thermocouple: Different libraries for ESP, UNO  
#if WITH_MAX31855
#if PROC_UNO
#include "Adafruit_MAX31855.h"
Adafruit_MAX31855 tcpl(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#if WITH_TCPL_2
Adafruit_MAX31855 tcp2(TCPL_CLCK, TCPL_CSL2, TCPL_MISO);                  // Second Tcpl shares clock, data
#endif  //  WITH_TCPL_2
#endif  //  WITH_UNO
//
#if PROC_ESP
#include "MAX31855.h"
MAX31855 tcpl(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#if WITH_TCPL_2
MAX31855 tcp2(TCPL_CLCK, TCPL_CSL2, TCPL_MISO);                  // Second Tcpl shares clock, data
#endif  //  WITH_TCPL_2
#endif  //  ESP
#endif  //  MAX31855

// MAX6675 Thermocouple: same libraries for ESP, UNO  
#if WITH_MAX6675
#include <max6675.h>
MAX6675 tcpl(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#if WITH_TCPL_2
MAX6675 tcp2(TCPL_CLCK, TCPL_CSL2, TCPL_MISO);                  // Second Tcpl shares clock, data
#endif  //  WITH_TCPL_2
#endif  //  MAX6675

// PCF8574 I2C I/O extender 
#if WITH_PCF8574
#define TWIO_IMSK  0x0F  // PCF port 3-0 pin  7-4 are inputs 
#define TWIO_OMSK  0xF0  // PCF port 7-4 pin 12-9 are outputs 
//
#define TWIO_ADDX  0x20  // A0,A1, A2 Gnd
#define ADC0_TWPO  4     // Curr Hi-lo v Adc0 Avg prt 4  pin  9
#define MILL_TWPO  5     // mill fast io          prt 5  pin 10
#define OFFN_TWPO  6     // offn                  prt 6  pin 11
#define SPAR       7     // spare                 prt 7  pin 12
#include "PCF8574.h"
void twioInit(void);
byte twioRead8(void);
void twioWritePin(byte, boolean);
void twioWrite8(byte);
PCF8574 twio( TWIO_ADDX );
int twioCurr;
#endif  // WITH_PCF8574

/// Declarations by unit

#if (PROC_ESP && !PROC_NMCU)
// A/D Chan 0 
int adc0Curr, adc0Prev, adc0Maxi, adc0Mini, adc0Avge;
boolean        adc0Bit0; 
#endif

// Artisan 40+ char serial pkt: Ambient, ChnA, ChnB, ChnC, ChnD, HTR, FAN, SV, ITNL ; 
// Fields when csv logging: TimeTotal TimeRamp ChnA(ET) ChnB(BT), ChnC, ChnD, PWM%, IO3, SV, AMB	Event
char artiResp[] = "012.4,678.0,234.6,890.2,345.7,901.3,567.9,123.5,789.1    ";        // 57chars [0]..[56] + null
// Artisan csv header format: these two lines must contain tab chars, not spaces. Can't figure how to 'Event' at EOL
const char csvlHdr1[] = "Date:	Unit:C	CHARGE:	TP:	DRYe:	FCs:	FCe:	SCs:	SCe:	DROP:	COOL:	Time:";
const char csvlHdr2[] = "Time1	Time2	ET	BT	Event	S3	S4	Htr	Fan	SetPt	Amb";

// billboard string for LCD + Serial  Lower cases: computed/measured Upper case: User/Setpoints 
char bbrdLin0[] = "w100% r-123 128c"; 
char bbrdLin1[] = "P0-0 S12.3m 228C";
// UpperCase imperatives 
#define holdChar  'H'                       // Prefix to decimal mins alternating with total time
#define manuChar  'M' 
#define rampChar  'R'
#define setpChar  'S'
#define fahrChar  'F'
#define celsChar  'C'
// lowerCase computed
#define orffChar  'o'
#define boilChar  'b'
#define chgdChar  'c'
#define dryeChar  'd'
#define finiChar  'f'
#define fc1sChar  'k'
#define muteChar  'm'
#define prehChar  'p'
#define tpntChar  't'
#define BOILTMPC  100
#define DRYETMPC  150
#define FC1STMPC  200
#define FINITMPC  230
char bbrdMode, phseChar;
char userScal   = 'C';

//
String     userCmdl("                                                 ");  // 49Chrs + null
// char array, below, is incompatible with MQQQTT topic handling 
//char   userCmdl[] = "                                                 "; // 49Chrs + null

//eprm
int eprmSize, eprmFree;
// Addresses at end of EEPROM for saved PID parameters, TALE = count
#define EADX_KP (eprmSize - 1 * (sizeof(float)))
#define EADX_TI (eprmSize - 2 * (sizeof(float)))
#define EADX_TD (eprmSize - 3 * (sizeof(float)))
#define EADX_BE (eprmSize - 4 * (sizeof(float)))
#define EADX_GA (eprmSize - 5 * (sizeof(float)))
#define EADX_KA (eprmSize - 6 * (sizeof(float)))
#define EADX_CT (eprmSize - 7 * (sizeof(float)))
#define EPRM_TALE 7

// Duty Cycle Step Size for OT(U/D) commands 
#define DUTY_STEP 5 // for OTn UP/DOWN commands

#if ( PROC_ESP && WITH_WIFI) 
// i-n-g-o MQTT 
// mqtt strings are declared for both ESP8266 and UNO 
char mqttVals[] =  "                ";                     // mqtt value 16sp 15ch max
// General Info topics 
const char c100Tops[]  = "/popc/cbck1000";
const char c200Tops[]  = "/popc/cbck2000";
const char c900Tops[]  = "/popc/cbck9000";
const char echoTops[]  = "/popc/echoCmdl";
const char inf0Tops[]  = "/popc/bbrdLin0";
const char inf1Tops[]  = "/popc/bbrdLin1";
const char userTops[]  = "/popc/userCmdl";
// Artisan interface UC names  'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
const char ArspTops[]  = "/popc/artiResp";
#endif

//pidc
//Ap15
// Fast response PID to match approx 30Hz PWM frequency 
//  Date   Kp   Ti   Td   Beta Gamma  Ka
//  My10  6.0  3.0  5.00  1.0  1.0  0.00  popc 
//  My19  8.0  2.0  2.00  1.0  1.0  0.00  popc 
//  My19  8.0  3.0  2.00  1.0  1.0  0.00  popc 
//  
//
float pidcKp      =  12.000;              // P-Term gain
float pidcKc      =  12.000;              // P-Term gain compensated for setpoint above ambient
float pidcTi      =   1.350;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd      =   0.800;              // D-Term Gain sec ( Td++ = Gain++)
//
float pidcBeta    =   1.000;              // P-term Refr vs YInp
float pidcGamma   =   1.000;              // D-term Refr vs YInp
float pidcKappa   =   0.000;              // Ambient comp Kp * ( 1 + Ka (sens - idle)/idle )
//
float pidcRn      =  AMBI_TMPC;           // Refr setpoint
float pidcYn      =  AMBI_TMPC;           // YInp input
float pidcEn                       = 0.0; // Refr - Inpu
float dUn, Edn, Epn, Epn1          = 0.0; // Calc error values
float Edfn2, Edfn1, Edfn, Un, Un1  = 0.0; // Calc error, output, prev output values
float Tf, Ts, TsDivTf              = 0.0; // Filter, Actual sample period, stash
float pidcPn, pidcIn, pidcDn       = 0.0; // per sample P-I-D-Err Terms
float pidcUn                       = 0.0; // PID controller Output
int   pidcPc, pidcIc, pidcDc       = 0.0; // cumulative P-I-D components for diagnostic readout 
// 
#define pidcUMax 255.000                  // Outp Max
#define pidcUMin   0.000                  // Outp Min
#define pidcAlpha  0.100                  // D-term Filter time
#define PIDC_NVDT  0                      // Sets Inverted D-Term, not classical Incremental PID 
///
//
const char versChrs[] = "18Au04-sync";
/// wip: stored profiles
// profiles
//   stored as profiles 1-9 with steps 0-9 in each 
struct profTplt {
  float profTarg;                         // Ramp endpoint or Setpoint if RMin == 0
  float profRMin;                         // Ramp time to TargTemp decimal minutes
  float profHMin;                         // Hold time at TargTemp decimal minutes 
}; 
// profTplt profLine = { AMBI_TMPC, 0, 99 };

//
#define EADX_PROF 0                       // EEPROM starting address of stored profiles
#define STEP_SIZE 3 * sizeof( float)
#define PROF_SIZE 9 * STEP_SIZE
//

#if 0
// prof, line and parm are based 1 - n 
profTplt profStep( int prof ) {
  profTplt profWork;
  int eadx = EADX_PROF + (prof - 1) * PROF_SIZE;
  EEPROM.get( eadx, profWork);
  return profWork; 
}
#endif

// pwmd vbls
byte pwmdPcnt                              = 0;                       // Percent duty cycle 
int pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp = 0;                       // Freq, Duty Cycle Target (255max) Output

// Run Cntl vbls determine On/Off/Auto/Manual/Attention et al Bit 0:Run 1:Atto 2:Manu 3:Attn 4:spare 5:Dbug 6:Arti 7:Info 
#define RCTL_RUNS 0x80
#define RCTL_AUTO 0x40
#define RCTL_MANU 0x20
#define RCTL_ATTN 0x10
#define RCTL_SPAR 0x08
#define RCTL_DIAG 0x04
#define RCTL_ARTI 0x02
#define RCTL_INFO 0x01

#if IFAC_ARTI
byte  bbrdRctl  = RCTL_ARTI;
#else
//
byte  bbrdRctl  = RCTL_INFO;                    // Non-CSV serial out
//byte  bbrdRctl  = 0x00;                        // Force CSV logging 
#endif
//
#if WITH_LCD
byte  lcdstRctl = RCTL_RUNS;
#else
byte  lcdstRctl = 0x00;
#endif
//
byte  pidcRctl  = (RCTL_RUNS | RCTL_AUTO );
byte  pwmdRctl  = (RCTL_RUNS | RCTL_AUTO);
//
#if WITH_OFFN
byte  offnRctl  = (RCTL_RUNS | RCTL_AUTO);
#else
byte  offnRctl  = RCTL_RUNS;
#endif
//
byte  profRctl  = RCTL_MANU;
byte  stepRctl  = 0x00;
byte  rampRctl  = 0x00;
byte  holdRctl  = 0x00;
byte  tcplRctl  = RCTL_RUNS;
byte  userRctl  = RCTL_RUNS;
//
#if WITH_WIFI
byte  wifiRctl  = RCTL_RUNS;
#else
byte  wifiRctl  = 0x00;
#endif
//
byte profNmbr, stepNmbr, profChar, stepChar;    // Numeric Profile No, Step No, Character values 

//  Rotary Switch
byte rotsCurr, rotsNewb;                        // Current value, newb test for change
boolean offnOutp;                               // off/on cycle counter, output 

//  time markers compared with uS mS and mill count 
#if (PROC_ESP && !PROC_NMCU)
unsigned long adc0Mark, adc0Poll                               = 0UL;
#endif
float         pidcPoll                                         = 0UL;
unsigned long lcdsMark, millStep, millMark,           pidcMark = 0UL;
unsigned long profMark, pwmdMark, rotsMark, tcplMark, vtcpMark = 0UL;
#if WITH_TCPL_2
unsigned long tcp2Mark = 0UL;
#endif
//

//
float  setpTmpC;                          // PID setpoint temp degC and previous value 
int    sns1Cdpm, sns2Cdpm           = 0;  // PID input, sens1, 2
int    userDuty, userDgpm = 0;            // userSet duty cycle; userSet C/F, meas C dg pm
int    userAOT1, userAOT2, userAIO3 = 0;  // Arti OT1Dty, Arti OT2Dty, Arti IO3Dty 
int    baseTmpC, rampCdpm, holdTmpC = 0;  // Ramp start temp, hold at  endpoint 
int    stepSecs, secsTgat, totlSecs = 0;  // Step elapsed, hold countdown, total run time
int    gateTmpC, userDegs = 0;            // User C/F temp, temporary array indexers, scratch vbls
float  sns1TmpC, sns2TmpC;                // Sensed temperature deg C for sensors 1, 2 
float  *chnATmpC, *chnBTmpC;              // Pointers to sensor tempC for first two Artisan response slots
#if PROC_ESP
float         sns3TmpC,  sns4TmpC;        // Sensed temperature deg C for sensors 3, 4 
float        *chnCTmpC, *chnDTmpC;        // Pointers to sensor tempC for 2nd   two Artisan response slots
#endif
float        *pidcSens;                        // Pointers to sensor input to PID

// Constants and variables for virtual tcpl, temperature / ROC tracking 
#define vChgSpht     0.2                       // Specific heat of bean mass 
#define vHtrWatt  1500.0                       // Heater power 
#define vTmpMaxC   260.0                       // Limit temp at max heater power
int   vChgGrms = 250;                          // Bean mass, may be altered while running  
int   vPrvMSec;                          
float vTmpDegC = IDLE_TMPC; 
float vTmpDegF = ((IDLE_TMPC + 40 ) * (9.0 / 5.0 )) - 40 ;

// History Dataset for moving average of sensor values for e.g. rate of change calculation 
#if PROC_ESP
int   dataSet1[] = { AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC  };
#define DATASET1_TALE 60              // How many elements in history 

// History Dataset for further process of sensor / moving average values for e.g. Savitzky-Golay curve fitting 
int   dataSet2[] = { AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC  };
#define DATASET2_TALE 60                 // How many elements in history 
#else
// UNO storage limits size of history datasets
int   dataSet1[] = { AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC  };
#define DATASET1_TALE 30              // How many elements in history 

int   dataSet2[] = { AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, \
                      AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC, AMBI_TMPC  };
#define DATASET2_TALE 12                 // How many elements in history 
#endif
//
#define FMA0_WDOW  3                      // Window numb slots for fast MA trend direction detection 
#define FMA1_WDOW  3
#define FMA2_WDOW  3
//
#define MAV0_WDOW  5                      // Window numb slots for moving average Latest values
#define MAV1_WDOW  5                      // Window numb slots for moving average Medium values
#define MAV2_WDOW  5                      // Window numb slots for moving average Oldest values
//
#define FMA4_WDOW  4                          // Window numb slots for PWM Mavs
#define FMA5_DIST 10                          // Window numb slots for PWM Mavs
#define PWM0_WDOW  3
#define PWM2_WDOW  3
//
#define DIRN_THLD  2                      // Threshold for phase direction detection 
#define SCAL_FACT 10                      // Mutiplier on history samples for improved resolution 
// Ref: https://docs.scipy.org/doc/scipy-0.15.1/reference/generated/scipy.signal.savgol_coeffs.html
//   Depending on desired window width uncomment one set of cefficients below 
#if 0
// 5 Wide 3rd order coefficients generated for Width=5 Order=3 Posn=4
float coefPwr0[] = {0.98571429,  0.05714286, -0.08571429,  0.05714286, -0.01428571};
//   (test) coefficients generated for Width=5 Order=3 Posn=0 
//float coefPwr0[] = {-0.01428571,  0.05714286, -0.08571429,  0.05714286,  0.98571429};
//   1st Differential coefficients generated for Width=5 Order=3 deriv=1 Posn=4 
float coefPwr1[] = { 1.48809524, -1.61904762, -0.57142857,  1.04761905, -0.3452381};
//   2nd Differential coefficients generated for Width=5 Order=3 deriv=1 Posn=4 
float coefPwr2[] = { 1.28571429, -2.14285714, -0.28571429,  1.85714286, -0.7142857};
#endif
//
int   set1Indx, set2Indx;
int   mav0Indx, mav1Indx, mav2Indx;                 // Indeces for Curr, Prev, Old long-Window MAVs for ROC 
int   histMav0, histMav1, histMav2;                 // Values  for Curr, Prev, Old long-Window MAVs for ROC 
int   fma0Indx, fma1Indx, fma2Indx;                 // Indeces for Curr, Prev, Old fast response ROC's for direction trends 
int   histFma0, histFma1, histFma2;                 // Values  for Curr, Prev, Old fast response ROC's for direction trends
float histRoc1, histRoc2, histFRoc;                 // Fast, Long PID RoC, fast RRoc
boolean chgeArmd, chgeSeen, tpntArmd, tpntSeen;     // Armed, Detection flags for events 
//
int   pwmdFma0, pwmdFma2, pwmdIdx0, pwmdIdx2;   // Values & Index for pwmd Mavs 
//float pwmdFRoc, beanRlHt;                     // Fast Mav of power input; inst ratio power chge / temp chge   
int beanRlHt;                                   // Fast Mav of power input; inst ratio power chge / temp chge   
#if PROC_ESP
// 11, 3, pos=0  coefficients generated for Width=11 Order=3 Posn=0
float coefPwr0[] = { 0.79020979,  0.33566434,  0.05594406, -0.08391608, -0.11888112, \
                    -0.08391608, -0.01398601,  0.05594406,  0.09090909,  0.05594406, \
                    -0.08391608 };
                   
float coefPwr1[] = { 0.55361305,  0.03962704, -0.22882673, -0.30730381, -0.25135975, \
                    -0.11655012,  0.04156954,  0.16744367,  0.20551671,  0.1002331 , \
                    -0.2039627 };
float coefPwr2[] = { 0.20979021, -0.02097902, -0.13053613, -0.14801865, -0.1025641 , \
                    -0.02331002,  0.06060606,  0.12004662,  0.12587413,  0.04895105,
                    -0.13986014 };
#define SAGO_WDOW 11                      // Window numb slots for S-G coefficients 
float histSago, hist1Dif, hist2Dif;                  // Savitzky-Golay MAV, 1st Dif, 2nd Dif
int   fma4Indx, fma5Indx;                            // ESP Fast MA
float fma4Scal, fma5Scal, fma45Roc ;                  // Fast, Long PID RoC, fast RRoc
#endif
//
///  Array indexing, moving averages
//  Initialise moving average history and calculated MA with supplied initial valu, Zero headIndex 
void initMavs(int initSet1, int initSet2) {
  byte i;
  // dataSet1 (raw sensor history) elements initialised
  for  (i=0; (i < DATASET1_TALE  ); i++) {
    dataSet1[i] = initSet1;
  }
  histMav2 = histMav1 = histMav0 = initSet1;                                             // Slow Moving Avge
  histFma0 = histFma1 = histFma2 = initSet1;
  histFRoc = histRoc1 = histRoc2 = 0;                                           // Short, Long rates of change
  //
  mav0Indx = wrapIndx( ( 0                                  ), DATASET1_TALE);  // Slow MA Index, current: at first element of smoothed history   
  mav1Indx = wrapIndx( (mav0Indx - (DATASET1_TALE / 2)      ), DATASET1_TALE);  // MidP MA Index, mid-age: at mid history plus window width
  mav2Indx = wrapIndx( (mav0Indx + 1 + MAV2_WDOW            ), DATASET1_TALE);  // Slow MA index, oldest:  at end history, curr plus window width
  //
  fma0Indx = wrapIndx( ( 0                                  ), DATASET1_TALE);  // Fast MA index                                                     // Fast MA Index, current
  fma1Indx = wrapIndx( (fma0Indx - FMA0_WDOW                ), DATASET1_TALE);  // Fast MA Index, mid-age
  fma2Indx = wrapIndx( (fma1Indx - FMA2_WDOW                ), DATASET1_TALE);  // Fast MA Index, oldest 
  //  
#if PROC_ESP  
  // dataSet2 (S-G smoothed sensor history) elements initialised 
  for  (i=0; (i < DATASET2_TALE  ); i++) {
    dataSet2[i] = initSet1;
  }
  fma4Scal = fma5Scal = initSet1;
  hist1Dif = hist2Dif = fma45Roc = 0;                                                      // Short, Long rates of change
  fma4Indx = wrapIndx( ( 0                                  ), DATASET1_TALE);  // Fast MA index                                                     // Fast MA Index, current
  fma5Indx = wrapIndx( (fma4Indx - FMA5_DIST                ), DATASET1_TALE);  // Fast MA Index, mid-age
#else
  pwmdIdx0 = wrapIndx( ( 0                                  ), DATASET2_TALE);  // Slow PWM MA Index, current
  pwmdIdx2 = wrapIndx( (pwmdIdx0 - (4 * PWM0_WDOW )         ), DATASET2_TALE);  // Delayed PWM MA Index  
  pwmdFma2  = pwmdFma0  = INIT_PWMD;
  //pwmdFRoc  = 0;                                        // PWM fast Mav, fast Roc
  // dataSet2 (S-G smoothed sensor history) elements initialised 
  for  (i=0; (i < DATASET2_TALE  ); i++) {
    dataSet2[i] = INIT_PWMD;
  }
#endif  
  set1Indx = set2Indx = 0; 
  chgeSeen = tpntSeen = 0; 
  chgeArmd = tpntArmd = 0; 
}

//
int wrapIndx ( int iIndx, int iTale) {
  // Returns Index number above/below tIndx with wrap around based on tTale: array length
  int respIndx = iIndx;
  while (respIndx <  0    ) { 
    respIndx += iTale;
  }  
  while (respIndx >= iTale) { 
    respIndx -= iTale; 
  }
  //SePrn(F("iIndx: ")); SePrn(iIndx); SePrn(F("  respIndx: ")); SePln(respIndx);   
  return(respIndx);
}
#if PROC_ESP
// Returns incremental change in moving average, supplied index of new latest value in history 
float indbMavs ( int *aHistAddx, int iHistTale, int iIndx, int iWindWide ) {
  float tempFltA;
  int tempIndx = wrapIndx ( (iIndx - iWindWide), iHistTale);
  tempFltA  = float(aHistAddx[iIndx]   );    // Add-In latest elem
  SePrn(F("# mavs Indx:")); SePrn(iIndx); SePrn(F(" Add:"));SePrn(tempFltA);
  tempFltA -= float(aHistAddx[tempIndx]);    // Del-Out prev oldest
  tempFltA /=      (iWindWide);              // Divide by weighting facor 
  SePrn(F(" Del:"));SePrn(aHistAddx[tempIndx]); SePrn(F(" Icr:"));SePln(tempFltA);
  return( tempFltA );
}
#endif

float incrMavs ( int *aHistAddx, int iHistTale, int iIndx, int iWindWide ) {
  float tempFltA;
  int tempIndx = wrapIndx ( (iIndx - iWindWide), iHistTale);
  tempFltA  = float(aHistAddx[iIndx]   );    // Add-In latest elem
  tempFltA -= float(aHistAddx[tempIndx]);    // Del-Out prev oldest
  tempFltA /=      (iWindWide);              // Divide by weighting facor 
  //Serial.print("#Mavs : ");  Serial.println(tempFltA);
  return( tempFltA );
}

#if PROC_ESP
// Returns Savitzky-Golay Value according to supplied Coefficients or differential coefficients
float saGoCalc (  int *aHistAddx, int iHistTale, int iIndx, int iWindWide, float *coefVals ) {
  float tempFltA = 0.000;
  int tempIndx, i;
  for (i = 0; (i < iWindWide ); i++) {
    tempIndx = wrapIndx ( (iIndx - iWindWide + i +1), iHistTale);  // Start at oldest, 'leftmost'
    tempFltA += aHistAddx[tempIndx] * coefVals[i];
  }  
  //Serial.print("# saGoCalc: ");  Serial.println(tempFltA);
  return(tempFltA);
}
#endif 

/// Conversion functions  
//   Temperature conversin 
float floatCtoF( float celsInp) {
  return (float( ((celsInp + 40.0) * 9.0 / 5.0 )   - 40.0 ));
}

float floatFtoC( float fahrInp) {
  return (float( ((fahrInp + 40.0) * 5.0 / 9.0 )   - 40.0 ));
}

//
int    intgFtoC( int   fahrInp) {
  return (int  ( ((fahrInp + 40) * 5 / 9 ) - 39.5 ));
}

//
byte nibl2Hex ( byte tNibl){
  if (tNibl > 15) return '?'; 
  if (tNibl > 9 ) {
    return ( 'A' + ( tNibl - 10));  
  } else {
    return ( '0' +   tNibl ); 
  }
}

/// Billboard LCDisplay and Artisan Serial Response Buffer 
void  respArti() {
  //// Artisan Iface : resp 'READ' = Amb,Ch1,2,3,4 :  Ambient, Setpoint ('Expected'), Sensed, CDpm, PW%
  //     Use Artisan "Config-Device-Extra TC4Ch3-4" and label CH3,4 as CDpm, PW%
  //   Clear entire buffer with spaces
  #if 0
  int iIndx;
  for  (iIndx=0; (iIndx < (sizeof(artiResp)-1)); iIndx++) {
    artiResp[iIndx] = ' ';
  }
  #endif
  if ( userScal == fahrChar) {                                
    dtostrf( floatCtoF(AMBI_TMPC),             5, 1, &artiResp[0]  );   // Posn0: Art's Ta Ch
    dtostrf( floatCtoF(*chnATmpC),             5, 1, &artiResp[6]  );   // P1: ( dflt: ET )
    dtostrf( floatCtoF(*chnBTmpC),             5, 1, &artiResp[12] );   // P2: ( dflt: BT )
#if PROC_ESP
    dtostrf( floatCtoF(*chnCTmpC),             5, 1, &artiResp[18] );   // P3:
    dtostrf( floatCtoF(*chnDTmpC),             5, 1, &artiResp[24] );   // P4:
#endif    
    dtostrf( floatCtoF( setpTmpC),             5, 1, &artiResp[42] );   // P7: SV
    dtostrf( floatCtoF(IDLE_TMPC),             5, 1, &artiResp[48] );   // P8: (Internal Temp)
  } else {
    dtostrf(            AMBI_TMPC,             5, 1, &artiResp[0]  );   // P0: Art's Ta Ch
    dtostrf(            *chnATmpC,             5, 1, &artiResp[6]  );   // P1: ( dflt: ET )
    dtostrf(            *chnBTmpC,             5, 1, &artiResp[12] );   // P2: ( dflt: BT )
#if PROC_ESP
    dtostrf(            *chnCTmpC,             5, 1, &artiResp[18] );   // P3:
    dtostrf(            *chnDTmpC,             5, 1, &artiResp[24] );   // P4:
    // ESP pseems to pad with nulls
    // artiResp[45] = artiResp[44] = artiResp[43] = artiResp[42] = ' ';
#endif    
    dtostrf(             setpTmpC,             5, 1, &artiResp[42] );   // P7: SV
    dtostrf(            IDLE_TMPC,             5, 1, &artiResp[48] );   // P8: (Internal Temp)
  } 
  // ESP pseems to pad with nulls
  //artResp[33] = artiResp[32] = artiResp[31] = artiResp[30] = ' ';
  dtostrf(               pwmdPcnt,             5, 1, &artiResp[30] );   // P5: (HTR) PWM % Duty Cycle
  dtostrf(               userAIO3,             5, 1, &artiResp[36] );   // P6: (FAN) PWM % Duty Cycle
  //
  // Fields below over write Slots 0, 3, 4 Defaults to record ROC's and Savitzky-Golay algorithms
  if ( userScal == fahrChar) {                                
    //dtostrf( (histSago * 9 / 5    +   0),  5, 1, &artiResp[ 0] );   // P0: Sa-Go MAvg temperature
#if (PROC_ESP  && !SHOW_OPT1)
    dtostrf( 10 * (histRoc1 * 9 / 5 +   0), 5, 1, &artiResp[18] );   // P3: Latest RoC
    dtostrf( 10 * (histRoc2 * 9 / 5 +   0), 5, 1, &artiResp[24] );   // P4: Oldest RoC
#else    
    dtostrf( (histRoc1 * 9 / 5      + 100), 5, 1, &artiResp[18] );   // P3: Latest RoC
    dtostrf( (histRoc2 * 9 / 5      + 100), 5, 1, &artiResp[24] );   // P4: Oldest RoC
#endif     
  } else {
#if (PROC_ESP  && !SHOW_OPT1)
    dtostrf( ( 1 *(histRoc1 )       +   0), 5, 0, &artiResp[18] );   // P3: 1/2 Min ROC from Simple MAvgs, scaled
    dtostrf( ( 1 *(histRoc2)        +   0), 5, 0, &artiResp[24] );   // P4: 1.0 Min ROC from Simple MAvgs, scaled
    //dtostrf( ( 1 *(histMav1 / SCAL_FACT) ), 5, 0, &artiResp[18] );   // P3:
    //dtostrf( ( 1 *(histMav2 / SCAL_FACT) ), 5, 0, &artiResp[24] );   // P4:
    //dtostrf( ( 1 *(fma45Roc)        +  40), 5, 0, &artiResp[18] );   // P3:
    //dtostrf( ( 1 *(fma45Roc)        +  50), 5, 0, &artiResp[24] );   // P4:
    dtostrf( ( 1 *(histFma0 / SCAL_FACT )), 5, 0, &artiResp[ 0] );   // P0: MAvg tmpC
    dtostrf( ( 1 *(histFma0 / SCAL_FACT )), 5, 0, &artiResp[48] );   // P8: Opt
    dtostrf( ( 1 *(fma45Roc)        +   0), 5, 0, &artiResp[36] );   // P6: FastMA ROC
#else    
    dtostrf( ( 1 * (histRoc1      ) +   0), 5, 1, &artiResp[18] );   // P3: 1/2 Min ROC from Simple MAvgs, scaled
    dtostrf( ( 1 * (histRoc2      ) +   0), 5, 1, &artiResp[24] );   // P4: 1.0 Min ROC from Simple MAvgs, scaled
    //dtostrf( (0.1*(pidcPc   )     + 100), 5, 1, &artiResp[18] );   // P3: pid Prop component                    
    //dtostrf( (0.1*(pidcIc   )     + 100), 5, 1, &artiResp[24] );   // P4: pid Intg component
    //dtostrf( ( 1 *(beanRlHt)        + 100), 5, 1, &artiResp[ 0] );   // P0: Sa-Go MAvg temperature
    //dtostrf( ( 1 *(beanRlHt)        + 100), 5, 1, &artiResp[48] );   // P8: Opt0-3
    //dtostrf( ( 1 *(histMav0 / SCAL_FACT )), 5, 1, &artiResp[ 0] );   // P0: MAvg tmpC
    dtostrf( ( 1 *(histFRoc )),             5, 1, &artiResp[48] );   // P8: Opt
    dtostrf(          userAIO3,             5, 1, &artiResp[36] );   // P6: (FAN) PWM % Duty Cycle
#endif     
  }  
  // Add commas, eof to response fields  
  artiResp[5]  = ',';
  artiResp[11] = ',';
  artiResp[17] = ',';
  artiResp[23] = ',';
  artiResp[29] = ',';
  artiResp[35] = ',';
  artiResp[41] = ',';
  artiResp[47] = ',';
  //artiResp[53] = '/r'; 
  //artiResp[54] = '/n';
  //artiResp[53] = artiResp[54] = artiResp[55] = artiResp[56] = ' ';
}

//
void fillBbrd() {
  // Billboard lines are filled for 2x16 LCD display and/or Serial 'Info' 
  //   strf ops append null chars, fill single chars later 
  // Billboard Line [0] 
  // LCD Display alternates on Even/Odd seconds
  if ( !(totlSecs % 2 )){
    // Even Seconds count
    bbrdLin0[0]   = phseChar;
    // Even Secs: show gate togo time in min.tenths
    dtostrf( (secsTgat     ),                       3, 0, &bbrdLin0[1]);
    bbrdLin0[4]  = ' ';
    //
    // Even Secs: show measured ROC  
    bbrdLin0[6]  = 'r';
    // Even Secs: show measured ROC  / Channel A sensor if two tcpls   
    if ( userScal == fahrChar) {
      dtostrf( int(    histRoc1 * 9.00 / 5.00), +4, 0, &bbrdLin0[7] );
    } else {
      dtostrf(         histRoc1,                +4, 0, &bbrdLin0[7] );
    }
    // Show thermocouple installed on ChanB = BT 
    bbrdLin0[11] = 'b';
    if ( userScal == fahrChar) {
      dtostrf( floatCtoF(*chnBTmpC),               3, 0, &bbrdLin0[12] );
    } else {
      dtostrf(           *chnBTmpC,                3, 0, &bbrdLin0[12] );
    } 
    bbrdLin0[15]  = userScal + 0x20;                 // Measured temp: lower case c/f
  }                                             //  End Even Seconds
  if (  (totlSecs % 2 )){
    // Odd Seconds count
    // Odd  seconds, Show PWM output 
    if ( pwmdRctl & RCTL_MANU) {
      bbrdLin0[0] = 'W';                           // Indicate manual PWM upper case 'W'
    } else {
      bbrdLin0[0] = 'w';                           // Indicate PID    PWM lower case 'w'
    }
    dtostrf( pwmdPcnt,                           3, 0, &bbrdLin0[1]);
    bbrdLin0[4]   = '%';
    // Odd Secs: show desred ROC
    bbrdLin0[6]  = 'R';
    // Odd secs:  show Target ROC, ChanB (ET) or Setpoint if not Tcpl2
    dtostrf( userDgpm,                          +4, 0, &bbrdLin0[7] );
#if WITH_TCPL_2
    // Second thermocouple installed: Show ChanA = ET 
    bbrdLin0[11] = 'a';
    if ( userScal == fahrChar) {
      dtostrf( floatCtoF(*chnATmpC),             3, 0, &bbrdLin0[12] );
    } else {
      dtostrf(           *chnATmpC,              3, 0, &bbrdLin0[12] );
    } 
#else
    bbrdLin0[11] = 's';
    if ( userScal == fahrChar) {
      dtostrf( floatCtoF(setpTmpC),              3, 0, &bbrdLin0[12] );
    } else {
      dtostrf(           setpTmpC,               3, 0, &bbrdLin0[12] );
    }
#endif
  }
  // Either Secs count, overwrite string nulls with spaces
  bbrdLin0[5]  = ' ';
  bbrdLin0[15]  = userScal + 0x20;                 // Measured temp: lower case c/f
  //
  // Billboard Line [1]
  // Profile Nmbr-Step 
  bbrdLin1[0]   = 'P';                              // P(rofile) Indicator 
  bbrdLin1[1]   = nibl2Hex( profNmbr);
  bbrdLin1[2]   = '-';
  bbrdLin1[3]   = nibl2Hex(stepNmbr);                 
  bbrdLin1[4]   = ' ';
  if ( !(totlSecs % 2 )) {
    // Even Secs: Preface time display with T(otal) time from startup
    bbrdLin1[5] = 'T';
    // Even Secs: show total time in min.tenths
    dtostrf( (totlSecs / 60),                     2, 0, &bbrdLin1[6]);
    dtostrf( ((totlSecs / 6) % 10),               1, 0, &bbrdLin1[9]);
    if ( userScal == fahrChar) {
      dtostrf( floatCtoF(holdTmpC),               3, 0, &bbrdLin1[12] );
    } else {
      dtostrf(           holdTmpC,               3, 0, &bbrdLin1[12] );
    }
    bbrdLin1[11]  = holdChar;                       //  Indicate Hold (Soak) Temp
  }                                            //  End Even Seconds
  if ( totlSecs % 2 ) {
    // Odd Secs: Preface time display with H(old), M(anu), R(amp), S(etp)
    bbrdLin1[5] = bbrdMode;
    // Odd Secs: show step time in min.tenths
    dtostrf( (stepSecs / 60),                     2, 0, &bbrdLin1[6]);
    dtostrf( ((stepSecs / 6) % 10),               1, 0, &bbrdLin1[9]);
    if ( userScal == fahrChar) {
      dtostrf( floatCtoF(setpTmpC),               3, 0, &bbrdLin1[12] );
  } else {
      dtostrf(           setpTmpC,               3, 0, &bbrdLin1[12] );
    } 
    bbrdLin1[11]  = setpChar;
  }  
  // Either Secs count, Always show ChanB-Sensor1 BT; overwrite string nulls with spaces
  bbrdLin1[8]   = '.';                             // Mins decimal pt, indecator, space
  bbrdLin1[10]  = 'm';                                
  bbrdLin1[15]  = userScal;
  //
  // fill info for serial output either Artisan protocol or console Info/csv logging 
  if (bbrdRctl & RCTL_ARTI ){
    // nothing is done here, for Artisan response usercmdl == 'REA' does respArti etc
    // 12Dc fill the bbrd for mqtt but dont send serial
    respArti();
  } else {
    // ! RCTL_ARTI so use artiresp space to create csv response  
    if ( bbrdRctl & RCTL_INFO ) {
      // ~ARTI &  INFO : Non-CSV tbd
    } else {
      // ~ARTI & ~INFO : csv logging for Artisan import Select 1s / Ns : TotMin:Sec StepMin:sec BT ET Event SV Duty
      dtostrf( (totlSecs / 60)                       ,02, -0, &artiResp[0]);      // CSV-1 Min:Tnths Total
      dtostrf( (totlSecs % 60)                       ,02, -0, &artiResp[3]);                         
      dtostrf( (stepSecs / 60)                       ,02, -0, &artiResp[6]);      // CSV-2 Min:Tnths Step
      dtostrf( (stepSecs % 60)                       ,02, -0, &artiResp[9]);
      if ( userScal == fahrChar) {
        dtostrf( floatCtoF(*chnATmpC)                 ,5, 1, &artiResp[12] );     // CSV-3
        dtostrf( floatCtoF(*chnBTmpC)                 ,5, 1, &artiResp[18] );     // CSV-4
        // ChnC,D replaced by ROC calculations 
        dtostrf( (histRoc1 * 9.0 / 5.0      )         ,5, 1, &artiResp[25] );     // CSV-5
        dtostrf( (histRoc2 * 9.0 / 5.0      )         ,5, 1, &artiResp[31] );     // CSV-6
        dtostrf( floatCtoF( setpTmpC)                 ,3, 0, &artiResp[46] );     // CSV-7
#if (!PROC_NMCU || SHOW_OPT1)
        dtostrf(( histFma0 / SCAL_FACT +0)            ,5, 1, &artiResp[41] );     // CSV-8 FMav
#else
        dtostrf(             userAIO3                 ,5, 1, &artiResp[41] );     // CSV-8 FAN
#endif
      } else {
        dtostrf(           *chnATmpC                  ,5, 1, &artiResp[12] );     // CSV-3 ET
        dtostrf(           *chnBTmpC                  ,5, 1, &artiResp[18] );     // CSV-4 BT
        // ChnC,D replaced by ROC calculations 
        dtostrf( (histRoc1             +  40)         ,5, 1, &artiResp[25] );     // CSV-5 Roc1
        dtostrf( (histRoc2             +  40)         ,5, 1, &artiResp[31] );     // CSV-6 Roc2
        dtostrf(             pwmdPcnt                 ,3, 0, &artiResp[37] );     // CSV-7 PWM
#if PROC_UNO 
        dtostrf(( histMav0 / SCAL_FACT +   0)         ,5, 1, &artiResp[41] );     // CSV-8 FMav
        dtostrf( ( 1  * beanRlHt       +   0 )        ,5, 1, &artiResp[51] );     // CSV-10 Pwr/dTmpC
#else
        dtostrf(             userAIO3                 ,5, 1, &artiResp[41] );     // CSV-8 FAN
        dtostrf( (fma45Roc / SCAL_FACT +   0 )        ,5, 1, &artiResp[51] );     // CSV-10 fma45Roc
#endif
        dtostrf(            setpTmpC                  ,3, 0, &artiResp[47] );     // CSV-9
      }
      // insert leading zero into timestamps 
      if (artiResp[0] == ' ') artiResp[0] = '0'; 
      if (artiResp[3] == ' ') artiResp[3] = '0'; 
      if (artiResp[6] == ' ') artiResp[6] = '0'; 
      if (artiResp[9] == ' ') artiResp[9] = '0'; 
      // fill fixed chars in response string, Artisan needs Tabs
      artiResp[2]  =  ':';
      artiResp[5]  = 0x09;
      artiResp[8]  =  ':';
      artiResp[11] = 0x09;
      artiResp[17] = 0x09;
      artiResp[23] = 0x09;
      artiResp[24] = 0x09;  // extra tab for 'event' field
      artiResp[30] = 0x09;
      artiResp[36] = 0x09;
      artiResp[40] = 0x09;
      artiResp[46] = 0x09;
      artiResp[50] = 0x09;
      //
      artiResp[56] =  ' ';
      //
      // Flag csv is ready for posting 
      bbrdRctl |= RCTL_ATTN;
    }
  }
}

///  EEPROM 
void eprmInit() {
#if PROC_ESP
  // ESP8266 has no .length function 
  eprmSize = 512;
  EEPROM.begin(eprmSize);
#else  
  eprmSize = EEPROM.length();
#endif  
  eprmFree = eprmSize - ( EPRM_TALE * sizeof(float));  // Reserve floats: Ka Ga Be Td Ti Kp XB YB CT
}

void eprmInfo() {
  float fromEprm;
  Serial.print(F("# EEPROM Size: ")); Serial.print(eprmSize);
  Serial.print(F(" Free: "));         Serial.println(eprmFree);
  EEPROM.get(EADX_KP, fromEprm);
  Serial.print(F("# EEPROM Kp:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_TI, fromEprm);
  Serial.print(F(" Ti:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_TD, fromEprm);
  Serial.print(F(" Td:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_BE, fromEprm);
  Serial.print(F(" Be:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_GA, fromEprm);
  Serial.print(F(" Ga:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_KA, fromEprm);
  Serial.print(F(" Ka:"));
  Serial.print(fromEprm);
  EEPROM.get(EADX_CT, fromEprm);
  Serial.print(F(" Tc:"));
  Serial.println(fromEprm);
}

#if WITH_LCD
/// LCD DISPLAY
void lcdsInit() {
  bbrdMode = setpChar;
  lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.home ();
  lcd.print(F("<== PopC-PID ==>"));
  lcd.setCursor ( 0, 1 );
  lcd.print(F("@bitwisetech.com"));
  delay ( 500 );                //  500mS startup delay
  lcd.clear();
  lcdsMark  =  millis() + LCDS_POLL_MSEC;
}

void lcdsLoop() {
  if (millis() < lcdsMark) {
    return;
  } else {  
    lcdsMark += LCDS_POLL_MSEC;
    //
    if (lcdstRctl == 0) {
      // Rctl == 0 Shutdown
      lcd.home ();
      lcd.print(F("lcdsLoop() Halt "));
      delay ( 500 );                //  500mS startup delay
    } else {  
      //
      lcd.setCursor(0, 0); // Posn char 0  line 0
      lcd.print(bbrdLin0);
      lcd.setCursor(0, 1); // Posn char 0  line 1
      lcd.print(bbrdLin1);
    }
  }
}
#endif 

/// Off-On SSR driver mill 
//
// Wrap writes to OFFN_PIN with TWIO Out 
void offnDrve ( byte tPin, byte tVal) {
#if WITH_PCF8574
  twioWritePin( OFFN_TWPO, ((tVal > 0) ? 1:0));
#endif
#if WITH_OFFN   
  digitalWrite( OFFN_OPIN, ((tVal > 0) ? 1:0));
#endif
}

void millInit() {
  millStep = 0;
#if ( PROC_ESP && !PROC_NMCU)
  adc0Curr = adc0Prev = adc0Maxi = adc0Mini = 0;
  adc0Poll = ADC0_POLL_MILL;
  adc0Mark = ADC0_POLL_MILL;
#endif
#if WITH_OFFN
#if WITH_PCF8574
  twioWritePin( OFFN_TWPO, 0);
#else
  digitalWrite( OFFN_OPIN, 0);
  pinMode( OFFN_OPIN, OUTPUT);
#endif
#endif
  millMark = micros() + MILL_POLL_USEC;
}

void millLoop() {
  int sensXMul;                         
  if ( micros() < millMark ) {
    return;
  } else {
    millStep += 1;
    millMark +=  MILL_POLL_USEC;
    // mill        
    //  Every step of mill: checck PWM %  for slow off/on output change
    //    Mill step * 400 is 2000mSec cycle (100% PWM), 1%PWM is 4 * Steps
    //    Writes to flicker onboard LED disabled, interfere if using as output    
    if ( ( pwmdPcnt * 2 )  > ( millStep % 200 ) ) {
      //use Outp as trig to avoid repeated i/o traffic, set on: offn mark time 
      if (offnOutp == 0) {
        offnOutp = 1;
        offnRctl |=  RCTL_ATTN;  // for virt tcpl
        //  offnDrve will do: digitalWrite( ONBD_OPIN, (1 & ~ONBD_LOWON));
        if ( offnRctl & RCTL_AUTO) {
          offnDrve ( OFFN_OPIN,   1);
        }
      }
    } else {
      if (offnOutp == 1) {
      //Output Off: use Outp as trig to avoid repeated i/o traffic, set off: offn space time 
        offnOutp = 0;
        offnRctl &= ~RCTL_ATTN;  // for virt tcpl
        //  offnDrve will do: digitalWrite( ONBD_OPIN, (0 | ONBD_LOWON));
        if ( offnRctl & RCTL_AUTO) {
          offnDrve ( OFFN_OPIN,   0);
        }
      }
    }
    // flicker tell tale LED in case of fast PWM
    if ( (offnRctl & RCTL_AUTO) && (millStep & 64)) {
      //
      digitalWrite( ONBD_OPIN, (0 | ONBD_LOWON));
    }
    // ESP's A/D converter is <= mill rate: controlled by ACDC_POLL_MILL  
#if (PROC_ESP && !PROC_NMCU)
    // ESP: A/D sense and signal maxima / minima
    if ( millStep >= adc0Mark ) {
      adc0Mark += adc0Poll;
      adc0Prev = adc0Curr;
      adc0Curr = analogRead(A0) / 102.4 ;
      adc0Maxi = (adc0Curr > adc0Maxi) ? (adc0Maxi / 2 + adc0Curr / 2) : adc0Maxi;
      adc0Mini = (adc0Curr < adc0Mini) ? (adc0Mini / 2 + adc0Curr / 2) : adc0Mini;
      adc0Avge = adc0Mini / 2  + adc0Maxi / 2 ;
      if ( adc0Maxi >    0 ) adc0Maxi -= 1 ;
      if ( adc0Mini < 1024 ) adc0Mini += 1 ;
#if WITH_PCF8574
      // save bit0 as flag for i2c traffic only on change   
      if (adc0Curr > adc0Prev) {
        if (!adc0Bit0) {
          adc0Bit0 = 1;
          twioWritePin( ADC0_TWPO, adc0Bit0);
        }
      } else {
        if (adc0Bit0) {
          adc0Bit0 = 0;
          twioWritePin( ADC0_TWPO, adc0Bit0);
        }
      }
#endif 
    } 
#endif //PROC_ESP !NMCU
    // Each mill step: 
    // Per second moving average calculation 
    if ( !( millStep % MILL_STEP_PSEC) ) {
      if (!isnan(*pidcSens)) {
        // New value: 
        sensXMul = int(*pidcSens * SCAL_FACT + 0.5) ;
        //        SePrn("# sensX:");SePln(sensXMul);      
      } else {  
        // Replicate
        //
        SePln("# ReplSnsr");
        sensXMul = dataSet1[set1Indx];
      }
      ///
      // dataSet1: wheel of sample * scale
      //
      //  Sensor history new Value: Incr index, insert new val and uodate moving averages
      /// DataSet1: Mavg Roc1, Roc2 
      set1Indx = wrapIndx(++set1Indx, DATASET1_TALE);
      dataSet1[set1Indx] = sensXMul;
      //
      // Current, mid-age, oldest Slow Moving averages on sensor samples 
      mav0Indx = wrapIndx( (++mav0Indx) , DATASET1_TALE);
      histMav0 += int(incrMavs ( dataSet1, DATASET1_TALE, mav0Indx, MAV0_WDOW) + 0 );
      //
      mav1Indx = wrapIndx( (++mav1Indx), DATASET1_TALE);
      histMav1 += int(incrMavs ( dataSet1, DATASET1_TALE, mav1Indx, MAV1_WDOW) + 0 );
      //
      mav2Indx = wrapIndx((++mav2Indx), DATASET1_TALE);
      histMav2 += int(incrMavs ( dataSet1, DATASET1_TALE, mav2Indx, MAV2_WDOW) + 0 );
      //
      // Current, older Fast Moving Averages on sensor samples 
      fma0Indx = wrapIndx(++fma0Indx, DATASET1_TALE);
      //   incrMavs MAvg calc returns change in cumulative moving average for latest values 
      histFma0 += int(incrMavs ( dataSet1, DATASET1_TALE, fma0Indx, FMA0_WDOW)  + 0   );
      //   No New value: Bump-Wrap index, Update calculated moving average
      fma1Indx = wrapIndx(++fma1Indx, DATASET1_TALE);
      //   incrMavs MAvg calc returns change in cumulative moving average for latest values 
      histFma1 += int(incrMavs ( dataSet1, DATASET1_TALE, fma1Indx, FMA1_WDOW)  + 0   );
      //   No New value: Bump-Wrap index, Update calculated moving average
      fma2Indx = wrapIndx(++fma2Indx, DATASET1_TALE);
      //   incrMavs MAvg calc returns change in cumulative moving average for latest values 
      histFma2 += int(incrMavs ( dataSet1, DATASET1_TALE, fma2Indx, FMA2_WDOW) + 0    );
      //
      // Scale rates of change by distance between samples 
      //histRoc1 = (histMav0 - histMav1) * ( (60.0 / SCAL_FACT) / ( DATASET1_TALE / 2  - ( MAV0_WDOW + MAV1_WDOW) / 2));
      histRoc1 = (histMav0 - histMav1) * ( (60.0 / SCAL_FACT) / ( DATASET1_TALE / 2                                ));
      //histRoc2 = (histMav0 - histMav2) * ( (60.0 / SCAL_FACT) / ( DATASET1_TALE - ( MAV0_WDOW + MAV2_WDOW) / 2));
      //
      histRoc2 = (histMav0 - histMav2) * ( (60.0 / SCAL_FACT) / ( DATASET1_TALE                            ));
      histFRoc = (histFma0 - histFma2) * ( (60.0 / SCAL_FACT) / ((FMA0_WDOW+FMA2_WDOW )/2  + FMA1_WDOW)); 
      //
#if PROC_ESP
      // ESP/NMCU: dataset2 is Savitzky-Golay Smooting using latest sample and previous over window width
      //histSago = saGoCalc( dataSet1, DATASET1_TALE, mav0Indx, SAGO_WDOW, coefPwr0);
      // Insert smoothed, scaled sample point into history dataSet2
      // New value: Bump-Wrap index, insert scaled value into history, remove scaling factor
      set2Indx = wrapIndx(++set2Indx, DATASET2_TALE);
      dataSet2[set2Indx] = int(histSago + 0.0 );
      // Remove resolution multiplier for external use 
      histSago = (histSago / SCAL_FACT);
      //  Calculate 1st, second differential
      hist1Dif = (saGoCalc( dataSet2, DATASET2_TALE, set2Indx, SAGO_WDOW, coefPwr1)/ SCAL_FACT);
      hist2Dif = (saGoCalc( dataSet2, DATASET2_TALE, set2Indx, SAGO_WDOW, coefPwr2)/ SCAL_FACT);
      // Scale rates of change by distance between samples 
      // tbd What's the right scaling for Sa-Go differentials ? 
      hist1Dif *= ( 60.0 / SAGO_WDOW);
      hist2Dif *= ( 60.0 / SAGO_WDOW);
      //
      fma4Indx = wrapIndx( ++fma4Indx  , DATASET1_TALE);
      fma5Indx = wrapIndx( ++fma5Indx  , DATASET1_TALE);
      // calc rates of change of dset2 samples: change in heater power 
      fma4Scal += int (incrMavs ( dataSet1, DATASET1_TALE, fma4Indx, FMA4_WDOW) + 0   );
      fma5Scal += int (incrMavs ( dataSet1, DATASET1_TALE, fma5Indx, FMA4_WDOW) + 0   );
      // Below: 
      fma45Roc = float(fma4Scal - fma5Scal) * ( (60.0 / SCAL_FACT) / (FMA5_DIST ));
#else       
      // UNO: Dataset2 History of PWM power 
      //Bump-Wrap index, insert scaled value into history
      set2Indx = wrapIndx(++set2Indx, DATASET2_TALE);
      dataSet2[set2Indx] = int(pwmdPcnt * SCAL_FACT);
      // bump dset2 indeces 
      pwmdIdx0 = wrapIndx( ++pwmdIdx0  , DATASET2_TALE);
      pwmdIdx2 = wrapIndx( ++pwmdIdx2  , DATASET2_TALE);
      // calc rates of change of dset2 samples: change in heater power 
      pwmdFma0 += int(incrMavs( dataSet2, DATASET2_TALE, pwmdIdx0, PWM0_WDOW) + 0   );
      pwmdFma2 += int(incrMavs( dataSet2, DATASET2_TALE, pwmdIdx2, PWM2_WDOW) + 0   );
      //       pwmdFRoc = float(pwmdFma0 - pwmdFma2) * ( (60.0 / SCAL_FACT) / ((FMA0_WDOW+FMA2_WDOW )/2  + FMA1_WDOW));
      // Below: dTempC / dPWM
      // beanRlHt = float( histFRoc / pwmdFRoc);
      // Below: MAv TmpC / MAv PWM 
      // 
      beanRlHt = ( pwmdFma0 / histMav0);        // Scale Magnified by arb amount
      //SePln(beanRlHt);  
#endif      
      // Key Event detection 
      //SePrn(F("# cA, tA, D1, D0, cS, tS:   "));
      //SePrn(chgeArmd); SePrn(F(" ")); SePrn(tpntArmd); SePrn(F(" "));  
      //SePrn(histFma1 - histFma2); SePrn(F(" ")); SePrn(histFma0 - histFma1); SePrn(F(" ")); SePrn(chgeSeen); 
      //SePrn(F(" ")); SePln(tpntSeen); 
      if ( chgeArmd) {
        if ( ((histFma0 - histFma1 + DIRN_THLD) < 0 ) && ((histFma1 - histFma2 + DIRN_THLD) < 0 )) { 
          chgeSeen = 1;
          chgeArmd = 0;
          phseChar = chgdChar;
          //pidcSync();
          //Serial.println(F("# CHARGE Seen"));
        }
      }    
      if ( tpntArmd) {
        if ( ((histFma1 - histFma2 + DIRN_THLD) < 0 ) && ((histFma0 - histFma1 - DIRN_THLD) > 0 )) { 
          tpntSeen = 1;
          tpntArmd = 0;
          phseChar = tpntChar;
          //pidcSync();
          //Serial.println(F("# TURNPT Seen"));
        }
      }    
      if ( histFma0 > BOILTMPC * SCAL_FACT) { phseChar = dryeChar; gateTmpC = DRYETMPC;}
      if ( histFma0 > DRYETMPC * SCAL_FACT) { phseChar = fc1sChar; gateTmpC = FC1STMPC;}
      if ( histFma0 > FC1STMPC * SCAL_FACT) { phseChar = finiChar; gateTmpC = FINITMPC;}
    }  // Enp per second moving average
  }  //  End Mill Step 
}  
    
///  PID Controller
//   pidc - implementation of PID controller
//   dUn =  Kp * [ (ep_n - ep_n-1)
//               + ((Ts/Ti) *  e_n)
//               + ((Td/Ts) * (edf_n - 2*edf_n-1 + edf_n-2) ) ]
//
//    Un = Un1 + dUn  Output = prev output + delta output
//
//   Inputs Yn: Measured Rn: Reference
//        Beta: Prop refr weighting Gamma: Diff Refr Weighting
//        Kp: Prop (Overall ?? ) Gain
//        Ti, Td, Ts Integrator, Deriv, Sample Times
//        Umin, Umax Limited output clamps
//
//   Output: Un
//
//        Ep: Proportional error with reference weighing
//        Ep = pidcBeta * Rn - Yn
//         E: Error = Rn - Yn
//        Ed: Unfiltered Derivative Error
//        Ed = pidcGamma * Rn -Yn
//       Edf: Deriv error with reference weighing and filtering
//       Edfn = Efn1 / ((Ts/Tf) + 1) + Edn * (Ts/Tf) / ((Ts/Tf) + 1)
//         where:
//         Tf : Filter time
//         Tf = alpha * Td , where alpha usually is set to 0.1
///


int anewFprm( int eadx, float *targ, char *titl) {
  float fromEprm;
  EEPROM.get (eadx, fromEprm ); 
  if ( !( fromEprm == *targ)){
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.print(titl); Serial.print(fromEprm);
    }
    *targ = fromEprm;
    return(1);
  } else {
    return(0);  
  }
}

// Read PID Gain parms from Eprom 
void pidcFprm() {
  byte anewTale = 0;
  if ( !( bbrdRctl & RCTL_ARTI )) {
    Serial.print(F("# New vals from EEPROM:"));
  }
  anewTale += anewFprm( EADX_KP, &pidcKp,   ("Kp: "));
  anewTale += anewFprm( EADX_TI, &pidcTi,   ("Ti: "));
  anewTale += anewFprm( EADX_TD, &pidcTd,   ("Td: "));
  anewTale += anewFprm( EADX_CT, &pidcPoll, ("Ct: "));
  anewTale += anewFprm( EADX_BE, &pidcBeta, ("Be: "));
  anewTale += anewFprm( EADX_KA, &pidcKappa,("Ka: "));
  //
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
    Serial.print(F("  "));Serial.print( anewTale); Serial.println(F(" new Vals"));
  }
}

// Serial logout of PID internal values 
void pidcInfo() {
  Serial.print(F("# PIDC   Kp:"));
  Serial.print(pidcKp);
  Serial.print(F(" Ti:"));
  Serial.print(pidcTi);
  Serial.print(F(" Td:"));
  Serial.print(pidcTd);
  Serial.print(F(" Be:"));
  Serial.print(pidcBeta);
  Serial.print(F(" Ga:"));
  Serial.print(pidcGamma);
  Serial.print(F(" Ka:"));
  Serial.print(pidcKappa);
  Serial.print(F(" Tc:"));
  Serial.println(pidcPoll);
}

void pidcRset() {
  // PID live reset: Zero P, I, D terms  Setpoint <= Ambient 
  // tbd: Should PID run after reset ? 
  Tf = pidcAlpha * pidcTd;
  pidcEn = Epn1   = Epn    = 0;
  Edfn2  = Edfn1  = Edfn   = 0;
  pidcPn = pidcIn = pidcDn = 0;
  pidcPc = pidcIc = pidcDc = 0;
  pidcUn = Un   = Un1      = 0;
  pidcRn = pidcYn = setpTmpC = userDegs = AMBI_TMPC;
  initMavs(AMBI_TMPC, INIT_PWMD);     // To clear dataSets and arm chge, tpnt detectors
  bbrdMode = manuChar;
  phseChar = orffChar;
#if WITH_OFFN
  offnDrve(OFFN_OPIN, 0);
#endif  
#if WITH_PWMD
  pwmdRctl &= ~RCTL_AUTO;
  pwmdRctl |=  RCTL_MANU;
#endif  
#if WITH_VIRTTCPL 
  //sns1TmpC =  AMBI_TMPC;
#if WITH_TCPL_2
  //sns2TmpC =  AMBI_TMPC;
#endif
#endif  
}

// Initialise at processor boot setup; pulls gains from Eprom and overwrites compiled values
void pidcInit() {
  pidcRset();
  // uncomm to get pidParms frm eprm 
#if PROC_NONE
  // pidcFprm();
#endif  
  // first time being enabled.
  pidcPoll = PIDC_POLL_MSEC;
  pidcMark =  millis() + pidcPoll;
  setpTmpC = int(AMBI_TMPC);
}

// PID runtime iteration 
void pidcLoop() {
  if ( millis() < pidcMark ) return; else {
    pidcMark += pidcPoll;  
    //
    if ( (pidcRctl & RCTL_RUNS)  == 0 ) {
      // Poll/Thermocouple == 0 Shutdown
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.println(F("# pidc stop"));
      }
      Un = pidcUn = pidcRn = pidcYn = 0.00;
    } else {
      //  Run PID iteration 
      Ts = (pidcPoll / 1000.0);                          // Sample Interval             (Sec)
      if (!isnan(setpTmpC)) pidcRn  = (float)setpTmpC;   // Update Reference            (Setpoint)
      if (!isnan(*pidcSens)) pidcYn = (float)*pidcSens;  // Update Measured Value       (Sensed temperature) 
      pidcEn  = pidcRn - pidcYn;                         // Error term:              =  (Setpoint - Measured ) 
      Epn = pidcBeta * pidcRn - pidcYn;                  // Proportional Error term: =  (Beta * Setpoint - Measured )  
      // D term 
      if ( pidcTd <= 0 ) {
        Edfn2 = Edfn1 = Edfn = 0;
      } else {
        Edn = pidcGamma * pidcRn - pidcYn;             // For D term Refence value is scaled by Gamma
        // Filter the derivate error:
        Tf = pidcAlpha * pidcTd;                       // Filter time is usually Sample time / 10 
        TsDivTf = Ts / Tf;                             // Filter smooths abrupt changes in error
        Edfn = (Edfn1 / (TsDivTf + 1.0)) +  (Edn * ((TsDivTf) / (TsDivTf + 1.0)));
      }
      // Accum Combine P, I, D terms 
      dUn = 0;
      // P term 
      // try temp compensated gain  pidcPn = pidcKp *  (Epn - Epn1);
      pidcPn = pidcKc *  (Epn - Epn1);                 // P term for this sample period ( Using compensated gain)
      pidcPc += int(pidcPn +0.5);                                // Pc cumulative used only for logging 
      // I term 
      if ( pidcTi > 0.0 ) {
        //Jn10 KtKp pidcIn = pidcKp * ((Ts / pidcTi) * pidcEn);
        pidcIn = ((Ts / pidcTi) * pidcEn);             // I term for sample period
      } else {
        pidcIn = 0;
      }  
      pidcIc += int(pidcIn + 0.5);                                // Ic cumulative I-term for logging 
      // D term computes change in rate                                      
      if ( pidcTd > 0.0 ) {                            // D term compares ( curr + oldest ) vs (2 * middle)  values  
        //Jn10 KtKp pidcDn = pidcKp * ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
#if PIDC_NVDT     
        pidcDn = ((pidcTd / Ts) * ((2 * Edfn1) - Edfn - Edfn2));
#else  
        pidcDn = ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
#endif       
      } else {
        pidcDn = 0;
      } 
      pidcDc += int(pidcDn);                                // Dc cumulative for logging 
      //dUn    = pidcPn + pidcIn + pidcDn;
      dUn    = pidcPn + pidcIn + pidcDn;               // dUn: Combined P+I+D deltas for sample period
      // Integrator anti-windup logic:
      if ( dUn > (pidcUMax - Un1) ) {
        dUn = pidcUMax - Un1;
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.println(F("# maxSatn"));
        }
      } else if ( dUn < (pidcUMin - Un1) ) {
        dUn = pidcUMin - Un1;
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.println(F("# minSatn"));
        }
      }
      // Update output if increment is valid number 
      if (!isnan(dUn)) {
        Un = Un1 + dUn;                               // Add PID delta to Output value
      }
      if ( pidcRctl & RCTL_AUTO) {  
        pidcUn = Un;                                    // Only if PID in Auto:  Copy into external Output Variable  
      } else { 
        pidcUn = 0;
      }
      // Updates indexed values;
      Un1   = Un;
      Epn1  = Epn;
      Edfn2 = Edfn1;
      Edfn1 = Edfn;
    }
  }  
}

void pidcStrt(){
  // PID ON | START 
  pidcRctl |=  (RCTL_RUNS | RCTL_AUTO);
#if WITH_PWMD
  pwmdRctl &= ~RCTL_MANU;
  pwmdRctl |=  (RCTL_RUNS | RCTL_AUTO);
#endif  
  bbrdMode  =  setpChar;
  phseChar  =  prehChar;
}

void pidcStop() {
  bbrdMode = manuChar;
  pidcRctl &= ~RCTL_AUTO;
  pidcRn = pidcYn = setpTmpC = userDegs = AMBI_TMPC;
#if WITH_PCF8574
  twioWritePin( OFFN_TWPO, 0);
#endif
#if WITH_OFFN
  offnDrve(OFFN_OPIN, 0);
#endif  
#if WITH_PWMD
  pwmdRctl &= ~RCTL_AUTO;
  pwmdRctl |=  RCTL_MANU;
#endif  
  userDuty = 0; 
  phseChar = muteChar;
}

void pidcSync() {
  // PID live reset: Clear P,I,D terms but not accumuated output
  //   PID is not stopped, use this function to e.g clear large error after charge event
  pidcPn = pidcIn = pidcDn = 0;
  pidcPc = pidcIc = pidcDc = 0;
  if (!isnan(*pidcSens)) {
    baseTmpC = pidcRn = pidcYn = userDegs = setpTmpC = *pidcSens;
    initMavs(baseTmpC, pwmdPcnt);     // To clear dataSets and arm chge, tpnt detectors
    //SePrn(F("#Sync tmpC:")); SePln(baseTmpC);
  } else {
    pidcRn = pidcYn = float ( IDLE_TMPC + ( pwmdTarg / 255.0 ) * ( MAXI_TMPC - IDLE_TMPC)); 
  }  
  pidcUn = Un  = float ( pwmdTarg);
  phseChar = prehChar;
}

/// Profile Control Sequence
//    None selected                     : skip to sequencer end
//      step not active                 : get next step
//      past last step                  : reset,  skip to sequencer end
//      ramp zero:                      : skip to hold temperature
//      ramp active:                    : skip to sequencer end
//      ramp (else ~zero && ~active)    : start active ramp, skip to sequencer end
//      ramp done                       : pr rt active ramp, skip to sequencer end
//      hold time zero                  : skip to sequencer hold
//      hTemp ctive:                    : skip to sequencer end
//      ramp (else ~zero && ~active)    : start active ramp, skip to sequencer end
//      
void profInit() {
  // simulation
  *pidcSens = holdTmpC = int(AMBI_TMPC);
  stepSecs = totlSecs = secsTgat  = 0;
  gateTmpC = BOILTMPC;
  profMark =  millis() + PROF_POLL_MSEC;
}

void profLoop() {
  if ( millis() < profMark) return; else { 
    profMark += PROF_POLL_MSEC;  
    // prevent lcd rollover; 6000 secs == 100 min 
    (secsTgat >    0) ? (secsTgat -= 1) : (secsTgat  = 0);
    (totlSecs > 5998) ? (totlSecs  = 0) : (totlSecs += 1);
    (stepSecs > 5998) ? (stepSecs  = 0) : (stepSecs += 1);
    // apply PID gain compensation if non-zero
     if ( (!isnan(*pidcSens)) && ( pidcKappa > 0 ) ) {
      pidcKc = pidcKp * ( 1 + pidcKappa * ( *pidcSens - IDLE_TMPC ) / IDLE_TMPC );
    } else {
      pidcKc = pidcKp;
    }
    // Handle stored profile 
    if (profNmbr <= 0) {
      // Stop auto profile and revert to manual control 
      profRctl = RCTL_MANU;
      stepRctl, rampRctl, holdRctl = 0x00;
    }
    else {
      // wip Active saved profile:  is active
      if ( rampRctl & RCTL_RUNS ){
        // Ramp to temp is active
        if ( rampRctl & RCTL_ATTN ){
        // Ramp to temp is active and complete
        }
      }
      //  
    }
    //
    if (rampCdpm == 0) {
      // ramp of zero implies full power to setpoint 
      bbrdMode = setpChar;
    } else {
      // Profile or User ramp to  setpoint temp and then hold
      // User Ramp forces max/min setpoint temp: after 'r' cmd use 's' cmd to set Hold temp 
      if (stepSecs > 10 )  {
        if (holdTmpC >= baseTmpC ) {
          // Rising  ramp finished when sensed is above hold Temp 
          if ((!isnan(*pidcSens)) && (*pidcSens >= holdTmpC)) bbrdMode = holdChar;
        } else {
          // Falling ramp finished when sensed is below hold Temp 
          if ((!isnan(*pidcSens)) && (*pidcSens <  holdTmpC)) bbrdMode = holdChar;
        }
      }
      if ( bbrdMode == holdChar ) {
        // do not really want to have s mode 
        rampCdpm = 0;
        setpTmpC = holdTmpC;
        // wiprampFini = 1;
      } else {
        bbrdMode = rampChar;
        setpTmpC = float(baseTmpC) \
                 + float (stepSecs) * float(rampCdpm) / 60.0;
      }          
    }
    if ( histRoc1 > 0) { 
      //          secsTgat = 60 * ( gateTmpC - *pidcSens ) / rampCdpm;
      //
      secsTgat = 60 * ( gateTmpC - *pidcSens ) / histRoc1;
    }  
    // update billboard
    fillBbrd();
    int tempIntA;
    //
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      if ( bbrdRctl & RCTL_INFO ) {
        // Send billboard 'Info' on serial 
        for ( tempIntA = 0; tempIntA < 16; tempIntA++ ) {
          Serial.write(bbrdLin0[tempIntA]);
        }
        Serial.write(" <=> ");
        for ( tempIntA = 0; tempIntA < 16; tempIntA++ ) {
          Serial.write(bbrdLin1[tempIntA]);
        }
        // For PID debug Un-comment and set #if 1 at pidcDbug() 
        //if ((bbrdRctl & RCTL_DIAG) == RCTL_DIAG) {  
          //pidcDbug();
        //}
        Serial.println(F(" "));
      } else {
        // If bbrd flagged send Artisan csv logging serial 
        if ( bbrdRctl & RCTL_ATTN ) {
          for ( tempIntA = 0; tempIntA < (sizeof(artiResp) -1); tempIntA++ ) {
            Serial.write(artiResp[tempIntA]);
          }
          Serial.write('\r');
          Serial.write('\n');
          bbrdRctl &= ~RCTL_ATTN;
        }
      }
    }  
  }
}

/// PWM Drive
// For Arduino Uno, Nano, Micro Magician, Mini Driver, Lilly Pad, any ATmega 8, 168, 328 board**
//---------------------------------------------- Set PWM frequency for D5 & D6 -----------------
//                                                               for PWM freq 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010     tmr 0 divisor:     8 for PWM freq  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011    *tmr 0 divisor:    64 for PWM freq   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100     tmr 0 divisor:   256 for PWM freq   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101     tmr 0 divisor:  1024 for PWM freq    61.04 Hz
//---------------------------------------------- Set PWM frequency for D9 & D10 ----------------
//TCCR1B = TCCR1B & B11111000 | B00000001     tmr 1 divisor:     1 for PWM freq 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010     tmr 1 divisor:     8 for PWM freq  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011    *tmr 1 divisor:    64 for PWM freq   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100     tmr 1 divisor:   256 for PWM freq   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101     tmr 1 divisor:  1024 for PWM freq    30.64 Hz
//------------------------------------------- Set PWM frequency for D3 & D11 -------------------
//TCCR2B = TCCR2B & B11111000 | B00000001     tmr 2 divisor:     1 for PWM freq 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010     tmr 2 divisor:     8 for PWM freq  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011     tmr 2 divisor:    32 for PWM freq   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100    *tmr 2 divisor:    64 for PWM freq   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101     tmr 2 divisor:   128 for PWM freq   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110     tmr 2 divisor:   256 for PWM freq   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111     tmr 2 divisor:  1024 for PWM freq    30.64 Hz
//
void pwmdExpo( byte dtwoExpo) {
// ESP Default Frequ is 1024. Writing new Freq seems to cause exception
#if PROC_ESP
#endif  // PROC_ESP
#if PROC_UNO
  switch (dtwoExpo) {
    case   0:
      // Pscl:    1 Freq: 31372.55Hz
      TCCR1B = TCCR1B & B11111000 | B00000001;
    break;
    case   3: 
      // Pscl:    8 Freq:  3921.16Hz
      TCCR1B = TCCR1B & B11111000 | B00000010;
    break;
    case   8: 
      // Pscl:  256 Freq:   122.55Hz
      TCCR1B = TCCR1B & B11111000 | B00000100;
    break;
    case   10: 
      // Pscl: 1024 Freq:    30.64Hz
      TCCR1B = TCCR1B & B11111000 | B00000101;
    break;
    default:
      // Pscl:   64 Freq:   490.20Hz
      TCCR1B = TCCR1B & B11111000 | B00000011;
    break;
  }
#endif  
}  

void pwmdSetF( double newFreq) {
#if PROC_ESP
  // ESP tbd 2018Ap10 ESP8266 throws reboot exception if analogWriteFreq() is used
  //analogWriteFreq(newFreq);
  pwmdFreq = PWMD_FREQ;
#else 
  if        ( newFreq < 43.32) {
    pwmdExpo( 10); 
    pwmdFreq =  31;
  } else if ( newFreq <  173.20 ) {
    pwmdExpo( 8); 
    pwmdFreq = 123;
  } else if ( newFreq <  693.14 ) {
    pwmdExpo( 6); 
    pwmdFreq = 490;
  } else if ( newFreq < 5544.30 ) {
    pwmdExpo( 3); 
    pwmdFreq = 3921;
  } else                         {
    pwmdExpo( 0); 
    pwmdFreq = 31372;
  } 
#endif 
}

void pwmdInit() {
  // PROC either define pins, exponent
  pwmdDuty = pwmdOutp = 0;
#if WITH_PWMD  
  pinMode( PWMD_OPIN, PWMD_MODE);
#if PROC_ESP
  #define PWMRANGE 255
  analogWriteRange(uint(PWMRANGE));
  //analogWriteFreq(uint(PWMD_FREQ));
#endif  
  pwmdSetF( PWMD_FREQ);			
#endif  
  pwmdMark =  millis() + PWMD_POLL_MSEC;
  pwmdRctl &= ~RCTL_ATTN;
}

void pwmdLoop() {
  if ( millis() < pwmdMark ) { 
    return;
  } else { 
    pwmdMark += PWMD_POLL_MSEC;  
    //
    if ( (pwmdRctl & RCTL_RUNS) == 0) {
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.println(F("# pwmdRctl==0"));
      }
      pwmdOutp = 0;
    } else {
      // last run control test has precedence 
      if ( pwmdRctl & RCTL_MANU) {
        pwmdTarg = int ( 255.0 * userDuty / 100.0 );
      }
      if ( pwmdRctl & RCTL_AUTO) {
        pwmdTarg = int (pidcUn + 0.5 );
      }
      pwmdOutp = pwmdTarg;
      pwmdPcnt = byte ((100.0 * pwmdOutp / 255) +0.5);
#if PROC_ESP
      if ( pwmdRctl & RCTL_ATTN ) {
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(F("# pwO")); Serial.println(int(pwmdOutp));
        }
        pwmdRctl &= ~RCTL_ATTN;
        //yield();
        //delay(10);
      }  
#if WITH_PWMD  
      analogWrite( PWMD_OPIN, int(pwmdOutp));
#endif      
#endif      
#if PROC_UNO
#if WITH_PWMD  
      analogWrite( PWMD_OPIN, pwmdOutp);
#endif      
#endif      
    }
  }
}

/// Rotary Enc Switch outputs binary 0000-1111 over 16 detent positions
//
int rotsValu() {
  byte resp, tVal;
  resp = 0;
#if PINS_ROTS
  if ( digitalRead(ROTS_BIT3) == LOW  ) resp  = 8; 
  if ( digitalRead(ROTS_BIT2) == LOW  ) resp += 4; 
  if ( digitalRead(ROTS_BIT1) == LOW  ) resp += 2; 
  if ( digitalRead(ROTS_BIT0) == LOW  ) resp += 1; 
#endif
#if WITH_PCF8574
  tVal = (~(twioRead8()) & TWIO_IMSK) ; 
  resp  = tVal & 0x01;
  if ( tVal & 0x02) resp += 2;
  if ( tVal & 0x04) resp += 4;
  if ( tVal & 0x08) resp += 8;
#endif
  return(resp);
}
    
void rotsInit() {
  // Set pins to weak pullup 
#if PINS_ROTS
  pinMode( ROTS_BIT3, INPUT_PULLUP);
  pinMode( ROTS_BIT2, INPUT_PULLUP);
  pinMode( ROTS_BIT1, INPUT_PULLUP);
  pinMode( ROTS_BIT0, INPUT_PULLUP);
#endif
  rotsMark =  millis() + ROTS_POLL_MSEC;
}
    
void rotsLoop() {
  if (millis() < rotsMark) return; else {
    rotsMark += ROTS_POLL_MSEC;
    // Only process consecutive new steady value 
    byte rotsTemp = rotsValu();
    if ( rotsTemp != rotsNewb) {
      // unsteady value: save as Newb but don't change Curr
      rotsNewb = rotsTemp;
    } else {
      // steady value for two polls
      if ( rotsCurr != rotsNewb) {
        // steady value changed from previous
        rotsCurr = rotsNewb;
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(F("# Rots:"));  Serial.println(rotsCurr);
        }
        // For each detent position insert into command command line: ClkWise:Rate CCW:Power settings 
        switch( rotsCurr) {
          case 0:
            stepNmbr = 0;
            userCmdl = "W0";
          break;
          case 1:
            stepNmbr = 1;
            userCmdl = "R3";
          break;
          case 2:
            stepNmbr = 2;
            userCmdl = "R6";
          break;
          case 3:
            stepNmbr = 3;
            userCmdl = "R10";
          break;
          case 4:
            stepNmbr = 4;
            userCmdl = "R12";
          break;
          case 5:
            stepNmbr = 5;
            userCmdl = "R15";
          break;
          case 6:
            stepNmbr = 6;
            userCmdl = "R20";
          break;
          case 7:
            stepNmbr = 7;
            userCmdl = "R30";
          break;
          case 8:
            stepNmbr = 8;
            userCmdl = "R0";
          break;
          case 9:
            stepNmbr = 9;
            userCmdl = "W100";
          break;
          case 10:
            stepNmbr = 10;
            userCmdl = "W95";
          break;
          case 11:
            stepNmbr = 11;
            userCmdl = "W90";
          break;
          case 12:
            stepNmbr = 12;
            userCmdl = "W85";
          break;
          case 13:
            stepNmbr = 13;
            userCmdl = "W80";
          break;
          case 14:
            stepNmbr = 14;
            userCmdl = "W70";
          break;
          case 15:
            stepNmbr = 15;
            userCmdl = "W60";
          break;
        }
        userRctl |= RCTL_ATTN; 
      }
    }
  }
}  
    
/// Channel - sensor allocation 

// return addx of sensor's output vble according to index number 
float *sensAddx ( int tSensNumb) {
  switch   (tSensNumb) {
#if PROC_ESP                                  // Save space on UNO
    case 4:
      return ( &sns4TmpC);
    break;  
    case 3:
      return ( &sns3TmpC);
    break;  
#endif
    case 2:
      return ( &sns2TmpC);
    break;  
    default :
      return ( &sns1TmpC);
    break;  
  }
}  

//
void chanSens( int tChanNumb,  char tSensChar ) {
  int sensNumb = 0;
  if ((tSensChar > '0') && (tSensChar < '5')) { 
    sensNumb = int(tSensChar - '0');
    switch  (tChanNumb) {
#if PROC_ESP
      case 4: 
        chnDTmpC = sensAddx( sensNumb);
      break;
      case 3: 
        chnCTmpC = sensAddx( sensNumb);
      break;
#endif
      case 2: 
        chnBTmpC = sensAddx( sensNumb);
      break;
      case 1: 
        chnATmpC = sensAddx( sensNumb);
      break;
    } 
  }   
}

/// Thermocouple
// 
void tcplInit() {
  // stabilize wait .. lcds banner is 1000mA anyway
  // delay(500);
#if WITH_MAX31855
#if PROC_ESP
  tcpl.setTCfactor(K_TC);
  tcpl.begin();
#if WITH_TCPL_2
  tcp2.setTCfactor(K_TC);
  tcp2.begin();
#endif // WITH_TCPL_2
#endif // PROC_ESP
#endif // WITH_MAX31855
// MAX6675 needs begin()
#if WITH_MAX6675
  //  begin(TCPL_CSEL) forces H/W SPI
  //  tcpl.begin(TCPL_CSEL);
  //  use, instead, below: begin(TCPL_CLCK, TCPL_CSEL, TCPL_MISO)  For S/W SPI 
  //  
  tcpl.begin(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#if WITH_TCPL_2
  //  begin(TCPL_CSEL) forces H/W SPI
  //  tcp2.begin(TCPL_CSL2);
  //  use, instead, below: begin(TCPL_CLCK, TCPL_CSEL, TCPL_MISO)  For S/W SPI 
  //
  tcp2.begin(TCPL_CLCK, TCPL_CSL2, TCPL_MISO);
#endif  
#endif  
// 
  tcplMark = millis() + TCPL_POLL_MSEC;
#if WITH_TCPL_2
  tcp2Mark = millis() + TCPL_POLL_MSEC;
#endif // WITH_TCPL_2
  vtcpMark = millis() + VTCP_POLL_MSEC;
  sns1TmpC = sns2TmpC = AMBI_TMPC;
#if PROC_ESP  
  sns3TmpC = sns4TmpC = AMBI_TMPC;
#endif  
}

// Thermocouple 
//   MAX6675 lib has low funtion, same as UNO-MAX31855 library; ESP-MAX31855 has more function  
#if ( WITH_MAX31855 || WITH_MAX6675) 
void tcplLoop() {
  double tcplTmpC;
  byte tResp = 0;            // preload response with non-error code
  if ( millis() < tcplMark) return; else {
    tcplMark += TCPL_POLL_MSEC;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      sns1TmpC = AMBI_TMPC;
    } else {
      // Read thermocouple 
#if (PROC_UNO || WITH_MAX6675)
      tcplTmpC = tcpl.readCelsius();
      // Serial.print("# tcpl-1-6675||UNO readCelsius:"); Serial.print(tcplTmpC);
#endif
#if ( PROC_ESP && WITH_MAX31855 ) 
      // read() gets both status and temp 
      tResp    = tcpl.read();
      //TMI 
#if 0 
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        // tcpl diags 
        Serial.print(F("# tcpl Stat:"));  Serial.print(tcpl.getStatus());
        Serial.print(F(" Fctr:"));        Serial.print(tcpl.getTCfactor());
        Serial.print(F(" Ofst:"));        Serial.print(tcpl.getOffset());
        Serial.print(F(" ITmp:"));        Serial.print(tcpl.getInternal());
        Serial.print(F(" TdegC:"));       Serial.println(tcpl.getTemperature());
      }
#endif
      // getTemperature returns internal variable from read()
      tcplTmpC = tcpl.getTemperature();
#endif
      // Error  condition ESP:tResp <> 0   (UNO | MAX6675): isNan Temperature  
      if (isnan(tcplTmpC)) {
        // Bad reading: use avges 
        sns1TmpC = float( histMav0 / SCAL_FACT ); 
#if (PROC_UNO && WITH_MAX31855)
        tResp = tcpl.readError();
#endif       
      }
      if ( tResp != 0 ) {
        // tResp, extended status available only for ESP-MAX31855 
#if WITH_LCD
        lcd.clear();
        lcd.home ();
        lcd.print(F("thermoCouple : "));
        lcd.setCursor ( 0, 1 );
#endif
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(F("# tcpl sense: "));
        }
        switch ( tResp ) {
          case 0:
#if WITH_LCD
            lcd.println("STATUS_OK      ");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println(F("OK"));
            }
          break;  
          case 1:
#if WITH_LCD
            lcd.println("Error - Open-Cct");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println(F("Open-Cct"));
            }
          break;  
          case 2:
#if WITH_LCD
            lcd.println("Error - Shrt-Gnd");
#endif
            if ( 0  & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("Shrt-Gnd"));
            }
          break;  
          case 4:
#if WITH_LCD
            lcd.println("Error - Shrt-Vcc");
#endif
            if (0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("Shrt-Vcc"));
            }
          break;  
          default:
#if WITH_LCD
            lcd.println("Error - NReadEtc");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("FailCase-NOREAD"));
            }
          break;  
        }
#if WITH_LCD
        delay (500);
        lcd.clear();
#endif
        // Bad reading: use avges 
        sns1TmpC = float( setpTmpC / 2.00 + dataSet1[0] / 4.00 + dataSet1[1] / 4.00 ); 
      }  // End tResp != 0 extended status 
      else {
        // Valid reading: update sensed temp
        sns1TmpC = float( tcplTmpC); 
        if (sns1TmpC >= MAXI_TMPC) {
          //pidcStop();
          Serial.println("# Max Temperature on Sensor 1: PID Stopped") ;
        }
      }
    }   // End tcplRctl != 0  
  }  //End tcpl poll time
}  // End tcplLoop 
#endif // (WITH_MAX31855 || WITH_MAX6675)

#if WITH_TCPL_2
// Thermocouple 2 
//   MAX6675 lib has low funtion, same as UNO-MAX31855 library; ESP-MAX31855 has more function  
#if ( WITH_MAX31855 || WITH_MAX6675) 
void tcp2Loop() {
  double tcp2TmpC;
  byte tResp = 0;            // preload response with non-error code
  if ( millis() < tcp2Mark) return; else {
    tcp2Mark += TCPL_POLL_MSEC;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      sns2TmpC = AMBI_TMPC;
    } else {
      // Read thermocouple 
#if (PROC_UNO || WITH_MAX6675)
      tcp2TmpC = tcp2.readCelsius();
      //  Serial.print("# tcpl-2-6675||UNO readCelsius:"); Serial.print(tcp2TmpC);
#endif
#if ( PROC_ESP && WITH_MAX31855 ) 
      // read() gets both status and temp 
      tResp    = tcp2.read();
      //TMI 
#if 0 
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        // tcpl diags 
        Serial.print(F("# tcp2 Stat:"));  Serial.print(tcpl.getStatus());
        Serial.print(F(" Fctr:"));        Serial.print(tcpl.getTCfactor());
        Serial.print(F(" Ofst:"));        Serial.print(tcpl.getOffset());
        Serial.print(F(" ITmp:"));        Serial.print(tcpl.getInternal());
        Serial.print(F(" TdegC:"));       Serial.println(tcpl.getTemperature());
      }
#endif
      // getTemperature returns internal variable from read()
      tcp2TmpC = tcp2.getTemperature();
#endif
      // Error  condition ESP:tResp <> 0   (UNO | MAX6675): isNan Temperature  
      if (isnan(tcp2TmpC)) {
        // Bad reading: don't update sensor temperature 
        //sns2TmpC = float( setpTmpC / 2.00 + dataSet1[0] / 4.00 + dataSet1[1] / 4.00 ); 
#if (PROC_UNO && WITH_MAX31855)
        tResp = tcp2.readError();
#endif       
      }
      if ( tResp != 0 ) {
        // tResp, extended status available only for ESP-MAX31855 
#if WITH_LCD
        lcd.clear();
        lcd.home ();
        lcd.print(F("thermoCouple : "));
        lcd.setCursor ( 0, 1 );
#endif
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(F("# tcp2 sense: "));
        }
        switch ( tResp ) {
          case 0:
#if WITH_LCD
            lcd.println("STATUS_OK      ");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println(F("OK"));
            }
          break;  
          case 1:
#if WITH_LCD
            lcd.println("Error - Open-Cct");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println(F("Open-Cct"));
            }
          break;  
          case 2:
#if WITH_LCD
            lcd.println("Error - Shrt-Gnd");
#endif
            if ( 0  & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("Shrt-Gnd"));
            }
          break;  
          case 4:
#if WITH_LCD
            lcd.println("Error - Shrt-Vcc");
#endif
            if (0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("Shrt-Vcc"));
            }
          break;  
          default:
#if WITH_LCD
            lcd.println("Error - NReadEtc");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println(F("FailCase-NOREAD"));
            }
          break;  
        }
#if WITH_LCD
        delay (500);
        lcd.clear();
#endif
        // Bad reading: don't update  
        //   sns2TmpC = float( setpTmpC / 2.00 + dataSet1[0] / 4.00 + dataSet1[1] / 4.00 ); 
      }  // End tResp != 0 extended status 
      else {
        // Valid reading: update sensed temp
        sns2TmpC = float( tcp2TmpC);
        if (sns2TmpC >= MAXI_TMPC) {
          pidcStop();
          Serial.println("# Max Temperature on Sensor 2: PID Stopped") ;
        }
      }
    }   // End tcplRctl != 0  
  }  //End tcp2 poll time
}  // End tcpl2oop 
#endif // (WITH_MAX31855 || WITH_MAX6675)
#endif

///
void virtTcplInit() {
  vPrvMSec = millis();
}
  
// virtual tcpl for debug 
void virtTcplLoop() {
  float vHtrLoss, vHtrCals;
  int   vMSecSmpl;
  vMSecSmpl = millis() - vPrvMSec;
  if (vMSecSmpl < VTCP_POLL_MSEC) {
	  return;
  } else {
    vPrvMSec  = millis();  
    vHtrLoss  = (float(vTmpDegC) - float(IDLE_TMPC)) / (vTmpMaxC - float(IDLE_TMPC)); 
    vHtrCals  = ( vMSecSmpl / 1000.0) * (vHtrWatt * (pwmdPcnt / 100.0  - vHtrLoss ) / 4.2);
    vTmpDegC += vHtrCals / ( vChgGrms * vChgSpht );
    sns1TmpC  = vTmpDegC + ( (random(2) / 1.0 ));             //  random variation added 
#if WITH_TCPL_2
    sns2TmpC  = sns1TmpC + 22.22 + ( (random(4) ));           //  fake second sensor for debug
#endif    
  }
#if 0  
  //  if ( !( millis() % 1000 )) {
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print(F("# vChgGrms: "));  Serial.print  (vChgGrms);
    Serial.print(F(" pwmdOutp: "));   Serial.print  (pwmdOutp);
    Serial.print(F(" vHtrLoss: "));   Serial.print  (vHtrLoss);
    Serial.print(F(" vHtrCals: "));   Serial.print  (vHtrCals);
    Serial.print(F(" vTmpDegC: "));   Serial.print  (vTmpDegC);
    Serial.print(F(" vTmpDegF:" ));   Serial.println(vTmpDegF);
  }
#endif  
}

//
#if WITH_PCF8574
// PCF8574 I2C IO 

void twioInit() {
  twio.begin();
  // init all outputs low
  twioWrite8(TWIO_IMSK );
}

byte twioRead8() {
  //  Datasheet says wrirte ones to precharge inputs but that resets ones out
  //  Write 'FF' to all bits ? 
  //twioWrite8( 0xFF );
  twioCurr = twio.read8();
  return twioCurr;
}

void twioWrite8( byte tByt ) {
  twio.write8( tByt);
}

void twioWritePin( byte tPin, boolean tBin ) {
  twio.write( tPin, tBin);
}
#endif // WITH_PCF8574

/// User Interface 
//
void userInit() {
  profNmbr = stepNmbr = 0;
  rampCdpm =  0;
  userScal = celsChar;
  baseTmpC = holdTmpC = userDegs = int(AMBI_TMPC);
  userDuty = 0;
  // Normal Artisan CHAN assignments CHNA,B,C,D:Sensor2, (Sensor1), (None), (None)
#if WITH_TCPL_2  
  chnATmpC = &sns2TmpC;
#else
  chnATmpC = &setpTmpC;
#endif  
  chnBTmpC = &sns1TmpC;
#if PROC_ESP
  chnCTmpC = &sns2TmpC;
  chnDTmpC = &sns1TmpC;
#endif
  pidcSens = &sns1TmpC;
  //
  //userRctl |= RCTL_ATTN;
}

void userLoop() {
  float fromEprm, tempFltA;
  int tempIntA, tempIntB;
  // test for when chars arriving on serial port, set ATTN
  //if ( Serial.available() && Serial.find('\n')  ) {
  if ( Serial.available() ) {
    // no wait for entire message  .. 115200bps 14 char ~ 1mSec there's a newline
    // delay(100);
    // read from buffer until first linefeed, remaining chars staay in hdwre serial buffer 
    userCmdl = Serial.readStringUntil('\n');
    //userCmdl[userCmdl.length()-1] = '\0';
    userRctl |= RCTL_ATTN;
  }
  if (userRctl & RCTL_ATTN) {
    if ( (!( bbrdRctl & RCTL_ARTI )) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print (F("# userCmdl: "));  Serial.println(userCmdl);
    }
#if WIFI_MQTT
    // echo back network originated cmds 
    wrapPubl( echoTops, userCmdl.c_str(), userCmdl.length()); 
#endif  
    if ( bbrdRctl & RCTL_ARTI ) {
      // Artisan Mode only cmds 
      //  CHA(N);S1S2S3S4 cmd: Assign inputs ID into message sequence Sx; Resp '#' ack
      if ((userCmdl[0] == 'C') && (userCmdl[1] == 'H') && (userCmdl[2] == 'A')) {
        chanSens( 1, userCmdl.charAt(5));
        chanSens( 2, userCmdl.charAt(6));
        chanSens( 3, userCmdl.charAt(7));
        chanSens( 4, userCmdl.charAt(8));
        // Send ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.println(F("# CHAN"));
        }
      }
      //  FILT;L1;L2;L3;L4 cmd: Assign digital filter lever Lx to Input n; Resp '#' ack
      if ((userCmdl[0] == 'F') && (userCmdl[1] == 'I') && (userCmdl[2] == 'L') && (userCmdl[3] == 'T')) {
        // Send ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.println(F("# FILT"));
        }
      }
      //  IO3 cmd
      if ((userCmdl[0] == 'I') && (userCmdl[1] == 'O') && (userCmdl[2] == '3')) {
        userAIO3 = (userCmdl.substring(4)).toInt();
        if (userAIO3 > 99) userAIO3 = 100;
        // Send ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.println(F("# IO3"));
        }
      }
      // OT(U/D)1/2 Commands
      if ((userCmdl[0] == 'O') && (userCmdl[1] == 'T')) {
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.print(F("# OT"));
        }
        if (userCmdl[2] == '1') {
          if ((userCmdl[4] == 'U')) {
              userAOT1 = userAOT1 + DUTY_STEP;
          } else if ((userCmdl[4] == 'D')) {
            userAOT1 = userAOT1 - DUTY_STEP;
          } else {
            userAOT1 = (userCmdl.substring(4)).toInt();
          }
          userAOT1 = (userAOT1 >  99) ?  100 : userAOT1;
          userAOT1 = (userAOT1 <   1) ?    0 : userAOT1;
          userDuty = userAOT1;
          pidcRctl &= ~RCTL_AUTO;
#if WITH_PWMD          
          pwmdRctl &= ~RCTL_AUTO;
          pwmdRctl |=  ( RCTL_MANU | RCTL_ATTN );
#endif          
          // manual pwm will apply in pwmd loop; unset manual ramp ctrl 
          rampCdpm = userDgpm = 0;
          // Send ack response
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("1"));
          }
        }
        if (userCmdl[2] == '2') {
          if ((userCmdl[4] == 'U')) {
              userAOT2 = userAOT2 + DUTY_STEP;
          } else if ((userCmdl[4] == 'D')) {
            userAOT2 = userAOT2 - DUTY_STEP;
          } else {
            userAOT2 = (userCmdl.substring(4)).toInt();
          }
          userAOT2 = (userAOT2 >  99) ?  100 : userAOT1;
          userAOT2 = (userAOT2 <   1) ?    0 : userAOT1;
          // Send ack response
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("2"));
          }
        }
      }
      //  PIDxxxx cmds
      if ((userCmdl[0] == 'P') && (userCmdl[1] == 'I') && (userCmdl[2] == 'D')) { 
          // Start ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.print(F("# PID "));
        }
        // PID;CHG;GRMS cmd: PID <= New Cycle Time mSec
        if ((userCmdl[4] == 'C') && (userCmdl[5] == 'H') && (userCmdl[6] == 'G')) {
          // cont ack response 
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.print(F("CHG: "));
          }
          // temp hold new bean mass to compare with old  
          tempIntB = (userCmdl.substring(8)).toInt();
          // Gets unstable with v low masses 
          if (tempIntB < 10 ) tempIntB = 10;
          // Beans added at ambient, reduce wkng temp towards ambient
          if (tempIntB > vChgGrms) {
            vTmpDegC = float(IDLE_TMPC) + (vChgGrms / ( vChgGrms + tempIntB )) * ( vTmpDegC - float(IDLE_TMPC));
          }
          vChgGrms = tempIntB;
        }  
        // PID;CT;mSec cmd: PID <= New Cycle Time mSec
        if ((userCmdl[4] == 'C') && (userCmdl[5] == 'T')) {
          //   cont ack response 
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.print(F("CT: "));
          }
          tempFltA  = (userCmdl.substring(7)).toFloat();
          if (!isnan(tempFltA)) {
            pidcPoll = tempFltA;
            if ( !( bbrdRctl & RCTL_ARTI ) ) {
              Serial.print(F("mSec poll: ")); Serial.print(pidcPoll);
            }
          }
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F(""));
          }
        }
        // PID;OFF cmd: PID <= Stop PID running, zero outputs 
        if ((userCmdl[4] == 'O') && (userCmdl[5] == 'F') && (userCmdl[6] == 'F')) {
          // Artisan <=> TC4 PID;OFF cmd: PWM, PID Run Ctrl Off,  PID reset internals
          pidcRset();                      // Switch off PID Rctl after this in case pidcRset() doesn't 
          pidcStop();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("OFF"));
          }
        }
        // PID;ON cmd: PID <= Set PID run control & auto
        if ((userCmdl[4] == 'O') && (userCmdl[5] == 'N')) {
          pidcStrt();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("ON"));
          }
        }
        // PID;PV;s    cmd: PID <= Select Process Variable ( Input) to Sensor Nbr 's' 
        if ((userCmdl[4] == 'P') && (userCmdl[5] == 'V')) {
          //   cont ack response 
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.print(F("PV Inp Snsr: "));
          }
          tempIntA = (userCmdl.substring(7,8)).toInt();
          if (( tempIntA > 0 ) && (tempIntA <5 )) {
            pidcSens = sensAddx( tempIntA);
            if ( !( bbrdRctl & RCTL_ARTI ) ) {
              Serial.println( tempIntA);
            }
          }
        }
        // PID;RESET
        if ((userCmdl[4] == 'R') && (userCmdl[5] == 'E') && (userCmdl[6] == 'S')) {
          // Artisan <=> TC4 PID;RESET cmd: PID reset internals Setpoint <= Ambient
          pidcRset();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("RESET"));
          }
        }
        // PID;START
        if (  ((userCmdl[4] == 'S') && (userCmdl[5] == 'T') && (userCmdl[6] == 'A')) \
            ||((userCmdl[4] == 'G') && (userCmdl[5] == 'O') )  ){
          pidcStrt();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("START"));
          }
        }
        // PID;STOP
        if ((userCmdl[4] == 'S') && (userCmdl[5] == 'T') && (userCmdl[6] == 'O')) {
          pidcStop();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("STOP"));
          }
        }
        // PID;SV
        if ((userCmdl[4] == 'S') && (userCmdl[5] == 'V')) {
          // set desired temperatre degC
          setpTmpC = (userCmdl.substring(7)).toFloat();
          userDegs = (userCmdl.substring(7)).toInt();
          if ( userScal == fahrChar) {
            setpTmpC = floatFtoC( setpTmpC); 
          }
          if (setpTmpC > MAXI_TMPC) setpTmpC = MAXI_TMPC;
          holdTmpC = setpTmpC;
          rampCdpm = userDgpm = 0;          // Setting target temp implies no ramp 
          stepSecs = 0;                     // User cmd: reset step timer 
          pidcRctl |=  (RCTL_RUNS | RCTL_AUTO);
          pwmdRctl &= ~RCTL_MANU;
          pwmdRctl |=  (RCTL_RUNS | RCTL_AUTO | RCTL_ATTN);
          // Send ack response 
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.print  (F("SV: "));  Serial.println(userDegs);
          }
        }
        // PID;SYNC  cmd: PID reset internals Setpoint <= pidcSens temperature
        if ((userCmdl[4] == 'S') && (userCmdl[5] == 'Y') && (userCmdl[6] == 'N')) { 
          pidcSync();
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("SYNC"));
          }
        }
        // PID;T;Kp;Ki;Kd tuning values
        if ((userCmdl[3] == ';') && (userCmdl[4] == 'T')) {
          tempIntA = userCmdl.indexOf( ';' , 6);              // find third ';'
          if (int(tempIntA) < userCmdl.length()) { 
            pidcKp = (userCmdl.substring(6, tempIntA)).toFloat();
          }
          tempIntB = userCmdl.indexOf( ';' , (tempIntA + 1));       // find fourth ';'
          if (tempIntB < userCmdl.length()) { 
            // Artisan's Ki converted to Ti by taking reciprocal 
            tempFltA = (userCmdl.substring((tempIntA + 1), tempIntB)).toFloat();
            if ( tempFltA <= 0.00 ) {
              pidcTi = 99999.99; 
            } else {
              pidcTi = ( 1.00 / tempFltA);
            }
          }
          tempFltA  = (userCmdl.substring(tempIntB +1)).toFloat();
          if (!isnan(tempFltA)) pidcTd = tempFltA;
          // Send ack response 
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.print(F("sets new Kp, Ti, Kd: "));
            Serial.print(   pidcKp); Serial.print(F(", "));
            Serial.print(   pidcTi); Serial.print(F(", "));
            Serial.println( pidcTd);
          }
        }
      }
      // POPC Cmd : Escape autoArti respond as popC
      if (  ((userCmdl[0] == 'P') && (userCmdl[1] == 'O') && (userCmdl[2] == 'P') && (userCmdl[3] == 'C')) \
         || ((userCmdl[0] == 'p') && (userCmdl[1] == 'o') && (userCmdl[2] == 'p') && (userCmdl[3] == 'c')) ) {
        bbrdRctl &= ~RCTL_ARTI; 
        Serial.println(F("# PopC Speak"));
      }
      // READ cmd:
      if ((userCmdl[0] == 'R') && (userCmdl[1] == 'E') && (userCmdl[2] == 'A')) {
        respArti();    // Fill response with latest
        for ( tempIntA = 0; tempIntA < sizeof(artiResp); tempIntA++ ) {
          Serial.print(artiResp[tempIntA]);
        }
        Serial.println('\r');
        // don't Send ack response 
        // Serial.println(F("#rea"));
      }
      // UNIT(S);F/C cmd:
      if ((userCmdl[0] == 'U') && (userCmdl[1] == 'N') && (userCmdl[2] == 'I') && (userCmdl[3] == 'T')) {
        if ( !( bbrdRctl & RCTL_ARTI ) ) {
          Serial.print(F("# UNITS: degs"));
        }
        //  'units' cmd, set user temperature scale 
        if (userCmdl[5] == 'C') {
          userScal = celsChar;
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("C"));
          }
        }
        if (userCmdl[5] == 'F') {
          userScal = fahrChar;
          if ( !( bbrdRctl & RCTL_ARTI ) ) {
            Serial.println(F("F"));
          }
        }
      }
    }
    // non-Artisan 'norm' speak cmds 
    // if (!( bbrdRctl & RCTL_ARTI )) {
    // inspect in either mode
    if (1){
      //  a/A Toggle Artisan format serial interface
      if ((userCmdl[0] == 'A') || (userCmdl[0] == 'a')) {
        // Toggle Artisan Interface
        if ( bbrdRctl & RCTL_ARTI ) {
          bbrdRctl &= ~RCTL_ARTI; 
          Serial.println(F("# Serial  <=> Console"));
        } else {
          bbrdRctl |=  RCTL_ARTI;
          Serial.println(F("# Serial  <=> Artisan"));
        }
      }
      // Artisan CHAN cmd For reset flip to Artisan mode   
      if ((userCmdl[0] == 'C') && (userCmdl[1] == 'H') && (userCmdl[2] == 'A') && (userCmdl[3] == 'N')) {
        bbrdRctl |=  RCTL_ARTI;
        //  'chan' cmd, respond '#'    
        Serial.println(F("#"));
      }
      //  c/C set Centigrade units 
      if (((userCmdl[0] == 'C') || (userCmdl[0] == 'c')) && (userCmdl[1] != 'H')) {
        userScal = celsChar;
      }
      // d/D Toggle Diagnostic Flag
      if ((userCmdl[0] == 'D') || (userCmdl[0] == 'd')) {
        if ( bbrdRctl & RCTL_DIAG ) {
          Serial.println(F("# Diagnostics Mode  is InActive"));
          bbrdRctl &= ~RCTL_DIAG; 
        } else {
          bbrdRctl |=  RCTL_DIAG; 
          Serial.println(F("# Diagnostics Mode  is   Active"));
        }
      }
      // e Readback EEPROM values 
      if (userCmdl[0] == 'e') {
        eprmInfo();
      }
      // E Write EEPROM values from PID current parameters 
      if (userCmdl[0] == 'E') {
        EEPROM.get(EADX_KP, fromEprm);
        if ( pidcKp    != fromEprm ) {
          EEPROM.put( EADX_KP, pidcKp);
        }
        EEPROM.get(EADX_TI, fromEprm);
        if ( pidcTi    != fromEprm ) {
          EEPROM.put( EADX_TI, pidcTi);
        }
        EEPROM.get(EADX_TD, fromEprm);
        if ( pidcTd    != fromEprm ) {
          EEPROM.put( EADX_TD, pidcTd);
        }
        EEPROM.get(EADX_CT, fromEprm);
        if ( pidcPoll    != int(fromEprm)) {
          EEPROM.put( EADX_CT, float(pidcPoll));
        }
        EEPROM.get(EADX_BE, fromEprm);
        if ( pidcBeta  != fromEprm ) {
          EEPROM.put( EADX_BE, pidcBeta);
        }
        EEPROM.get(EADX_GA, fromEprm);
        if ( pidcGamma != fromEprm ) {
          EEPROM.put( EADX_GA, pidcGamma);
        }
        EEPROM.get(EADX_KA, fromEprm);
        if ( pidcKappa != fromEprm ) {
          EEPROM.put( EADX_KA, pidcKappa);
        }
    #if PROC_ESP
        EEPROM.commit();
    #endif    
        eprmInfo();  
      }
      // f/F Set fahrenheit units 
      if (((userCmdl[0] == 'F') || (userCmdl[0] == 'f')) && (userCmdl[1] != 'I')) {
        userScal = fahrChar;
      }
      // g set gateTmpC
      if (userCmdl[0] == 'g') {
        userDegs = (userCmdl.substring(1)).toInt();
        if ( userScal == fahrChar) {
          userDegs = intgFtoC( userDegs); 
        }
        gateTmpC = userDegs;
        if (gateTmpC > MAXI_TMPC) gateTmpC = MAXI_TMPC;
      }
      if ((userCmdl[0] == 'H') || (userCmdl[0] == 'h')) {
        // set desired hold temperatre deg
        userDegs = (userCmdl.substring(1)).toInt();
        if ( userScal == fahrChar) {
          holdTmpC = intgFtoC( userDegs); 
        } else {
          holdTmpC = userDegs;
        }
        if (holdTmpC > MAXI_TMPC) holdTmpC = MAXI_TMPC;
        pwmdRctl &= ~RCTL_MANU;
        pwmdRctl |=  RCTL_AUTO;
      }
      // I  put pid Ti term 
      if (userCmdl[0] == 'I') {
        pidcTi = (userCmdl.substring(1)).toFloat();
        pidcInfo();
      }
      // J put pid Td term 
      if (userCmdl[0] == 'J') {
        pidcTd = (userCmdl.substring(1)).toFloat();
        pidcInfo();
      }
      if ((userCmdl[0] == 'L') || (userCmdl[0] == 'l')) {
        // Toggle logging, preface with banner lines when logging started 
        if ( bbrdRctl & RCTL_INFO ) {
          // Artisan csv Logging: send two header lines with tab chars
          // prefix with version 
          Serial.println(versChrs);
          eprmInfo();
          pidcInfo();
          Serial.println(csvlHdr1); 
          Serial.println(csvlHdr2); 
          // Switch On  Artisan csv Logging. TotalTime, StepTime must start at 0. 
          bbrdRctl &= ~RCTL_INFO; 
          stepSecs = totlSecs = 0;
        } else {
          bbrdRctl |= RCTL_INFO; 
        }
      }
      if (userCmdl[0] == 'M') {
        // temp hold new bean mass to compare with old  
        tempIntB = (userCmdl.substring(1)).toInt();
        // Gets unstable with v low masses 
        if (tempIntB < 10 ) tempIntB = 10;
        // Beans added at ambient, reduce wkng temp towards ambient
        if (tempIntB > vChgGrms) {
          vTmpDegC = float(IDLE_TMPC) + (vChgGrms / ( vChgGrms + tempIntB )) * ( vTmpDegC - float(IDLE_TMPC));
        }
        vChgGrms = tempIntB;
  #if 0
          Serial.print(F("# New vChgGrms: "));
          Serial.print(vChgGrms);
          Serial.print(F("  vTmpDegC: "));
          Serial.println(vTmpDegC);
  #endif
      }
      // m cmd TBD stored profile from memory 
      if (userCmdl[0] == 'm'){
        profNmbr = (userCmdl.substring(1)).toInt();
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        }
        stepSecs = 0;
      }
      // N  PID oN 
      if ((userCmdl[0] == 'N') && (userCmdl[1] != 'T')) {
        pidcRctl ^= RCTL_RUNS;
        pidcStrt();
      }
      // O  PID Off 
      if ((userCmdl[0] == 'O') ) {
        pidcRctl &= ~RCTL_RUNS;
        pidcStop();
      }
      // P  put pid Kp term 
      if (((userCmdl[0] == 'p') || (userCmdl[0] == 'P')) && \
           (userCmdl[1] != 'I') && (userCmdl[1] != 'i') && (userCmdl[1] != 'O') && (userCmdl[1] != 'o')) {
        pidcKp = (userCmdl.substring(1)).toFloat();
        pidcInfo();
      }
      // R/r Set RoC 
      if (((userCmdl[0] == 'R') || (userCmdl[0] == 'r')) && (userCmdl[1] != 'E')) {
        // Keep user entry for billboard; convert, set profile temp ramp rate degC/min
        userDgpm = (userCmdl.substring(1)).toInt();
        if ( userScal == fahrChar) {
          rampCdpm = int ( float(userDgpm) * 5.00 / 9.00);
        } else {
          rampCdpm = userDgpm;
        }
        if (!isnan(*pidcSens)) {   
          baseTmpC = int(*pidcSens);
        }
        // R cmd here must force hold temp, use H command later 
        if (userDgpm > 0) holdTmpC = MAXI_TMPC - 20;
        if (userDgpm < 0) holdTmpC = AMBI_TMPC;
        if (userDgpm == 0) {
          // Selected ROC 0: Hold current temp 
          bbrdMode = holdChar;
          if (!isnan(*pidcSens)) {   
            setpTmpC = int(*pidcSens);
          }
        } else {
          // On R0 don't reset step timer 
          stepSecs = 0;
        }
        // seting ramp unsets manual PWM width 
        pidcRctl |=  ( RCTL_AUTO | RCTL_RUNS);
        pwmdRctl &= ~RCTL_MANU;
        pwmdRctl |=  RCTL_AUTO;
        // Send ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(F("#r:")); Serial.println(userDgpm);
        }
      }
      if ((userCmdl[0] == 'S') || (userCmdl[0] == 's')) {
        // set desired setpoint / immed target temperatre deg
        userDegs = (userCmdl.substring(1)).toInt();
        if ( userScal == fahrChar) {
          setpTmpC = intgFtoC( userDegs); 
        } else {
          setpTmpC = userDegs;
        }
        if (setpTmpC > MAXI_TMPC) setpTmpC = MAXI_TMPC;
        holdTmpC = setpTmpC;
        rampCdpm = userDgpm = 0;          // Setting target temp implies no ramp 
        stepSecs = 0;                     // User command: reset step timer 
        pidcRctl |=  RCTL_RUNS;
        pidcRctl |=  RCTL_AUTO;
        pwmdRctl &= ~RCTL_MANU;
        pwmdRctl |=  RCTL_AUTO;
        bbrdMode  = setpChar;
      }
#if !PROC_UNO
      if (userCmdl[0] == 'U')  {
        // set new PWM frequency 
        pwmdFreq = (userCmdl.substring(1)).toInt();
        pwmdSetF( pwmdFreq);
      }
      if (userCmdl[0] == 'u') {
        if  (!( bbrdRctl & RCTL_ARTI )) {
          Serial.print(F("# pwmdFreq: ")); Serial.println(pwmdFreq);
        }
      }
#endif      
      if (userCmdl[0] == 'V') {
        // New PID Values from Eprom => PID 
        pidcFprm();
      }
      if (userCmdl[0] == 'v') {
        // Version string e if Artisan is setting Unit C/F 
        if  (!( bbrdRctl & RCTL_ARTI )) {
          Serial.println(versChrs);
          eprmInfo();
          pidcInfo();
        }
      }
      if ((userCmdl[0] == 'W') || (userCmdl[0] == 'w')) {
        // set new pwmD Width, run control flag to indicate manual override
        userDuty = (userCmdl.substring(1)).toInt();
        if (userDuty > 99) userDuty = 100;
        if ( !( bbrdRctl & RCTL_ARTI ) && (bbrdRctl & RCTL_DIAG) ) {  
          Serial.print(F("# Manu PWM%: "));
          Serial.println(userDuty);
        }
        //    Serial.print("# New PWM: ");
        //    Serial.println(userDuty);
        if ( userDuty == 0) {
          // Power off: Sense ambient ( fan htr pwr), temp setpt to meas ambient
          setpTmpC = AMBI_TMPC;
          pidcStop();
        } else {
          stepSecs = 0;                   // User command: reset step timer 
        }
        bbrdMode = manuChar;
        pidcRctl &= ~RCTL_AUTO;
        pwmdRctl &= ~RCTL_AUTO;
        pwmdRctl |=  ( RCTL_MANU | RCTL_ATTN );
        // manual pwm will apply in pwmd loop; unset manual ramp ctrl 
        rampCdpm = userDgpm = 0;
        // Send ack response 
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.println(F("#w"));
        }
      }
      // y re-sync PID to current conditions  
      if ((userCmdl[0] == 'y')) {
        // Live sync PID to present condition
        pidcSync(); 
        histFma0 = histFma1 = histFma2 = 0;
        chgeSeen = tpntSeen = 0; 
        chgeArmd = tpntArmd = 1; 
      }
      // Y  re-start ESP 
  #if PROC_ESP  
      // Y Restart ESP  
      if ((userCmdl[0] == 'Y') ) {
        Serial.print("# ESP restart in 3: ");
        delay(3000);
        ESP.restart(); 
      }
  #endif   
      //z  reset Time counts  
      if ((userCmdl[0] == 'z')) {
        // Zero 'Total Time' 
        totlSecs = 0;
        stepSecs = 0;
        phseChar = chgdChar;
      }
      // Z Reset PID
      if ((userCmdl[0] == 'Z')) {
        pidcRset();
      }
      if ((userCmdl[0] == '?')) {
        // For debug to see if Artisan is setting Unit C/F 
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.println(userScal);
        }
      }
      // end norm nonArti speak
    }  
    //  clean out cmdLine 
    //for ( tempIntA = 0; tempIntA < (userCmdl.length()); tempIntA++ ) {
    //  userCmdl[tempIntA] = ' ';
    //}
    // Drop Attn flag immediately so not masked during process
    userRctl &= ~RCTL_ATTN;
    //Serial.println(F("#z"));
  }
  // End single command parsing   
}

/// Wifi ESP|NMCU 
#if ( PROC_ESP && WITH_WIFI)
//
#include "TickerScheduler.h"
// scheduler tick callback flags use Attn bit for service request 
byte  cb10Rctl  = RCTL_RUNS;
byte  cb20Rctl  = RCTL_RUNS;
byte  cb90Rctl  = RCTL_RUNS;

TickerScheduler popcShed(3);
//

#if WIFI_MQTT
//share RW pubsub + RO
void dataCbck(String& topic, String& data);
void publCbck();
void discCbck();
void connCbck();
#endif //WIFI_MQTT

//
#if WIFI_SOKS  // wsokSrvr
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      if ( !( bbrdRctl & RCTL_ARTI ) ) {
        USE_SERIAL.printf("[%u] SockSrvr Disc!\n", num);
      }
    break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        USE_SERIAL.printf("# [%u] SockSrvr Connect from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
    	// send message to client
		  webSocket.sendTXT(num, "pSockSrvr acks Connected");
    }
    break;
    case WStype_TEXT:
      //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      if ( !( bbrdRctl & RCTL_ARTI ) ) {
        USE_SERIAL.printf("# popcSockSrvr [%u] get Text: %s\n", num, payload);
      }
      // send message to client
      // webSocket.sendTXT(num, "popcSockSrvr message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("popcSockSrvr message here");
    break;
    case WStype_BIN:
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      USE_SERIAL.printf("# [%u] popcSockSrvr get binary length: %u\n", num, length);
    }
    hexdump(payload, length);

    // send message to client
    // webSocket.sendBIN(num, payload, length);
    break;
  }
}
#endif //WIFI_SOKS

//  Init fn for all wifi Svces 
void wifsInit() {
  if (wifiRctl & RCTL_RUNS) {
    if ( !(bbrdRctl & RCTL_ARTI)) {
      Serial.println( F("# wifsInit MAC: "));
      Serial.print  ( WiFi.macAddress());
      Serial.print  ( F(" to upwdSSID: "));
      Serial.println(upwdSsid);
    } 
#if WIFI_WMAN
    WiFiManager popcWMan; //   Also in the setup function add
    //set custom ip for portal
    popcWMan.setAPStaticIPConfig(IPAddress( DWNDADDX ), IPAddress( DWNDADDX ), IPAddress(255,255,255,0));
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println(F("# WIFI_WMAN : init call popcMqtt.autoConnect"));
    }
    //first parameter is name of access point, second is the password
    popcWMan.autoConnect(dnwdSsid, dnwdPswd);


  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!popcWMan.autoConnect(dnwdSsid, dnwdPswd)) {
    if ( !(bbrdRctl & RCTL_ARTI)) {
      Serial.println(F("popcWMan autoConnect failed, ESP reset in 3"));
    }  
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  //if you get here you have connected to the WiFi
  if ( !(bbrdRctl & RCTL_ARTI)) {
    Serial.println(F("# WMan autoConnect OK"));
  }
#endif // WIFI_WMAN
#if WIFI_SOKS
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("# WIFI_SOKS WebS Init"));
    }
    //  WebSockets
    //Serial.setDebugOutput(true);
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      USE_SERIAL.setDebugOutput(true);
      USE_SERIAL.println();
      USE_SERIAL.println();
    }
    for(uint8_t t = 4; t > 0; t--) {
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
      USE_SERIAL.flush();
    }  
    delay(100);
    }
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println(F("# WMul : add upbd AP  (local IP: ) "));
      Serial.println(WiFi.localIP());
    }
    //WiFiMulti.addAP("SSID", "passpasspass");
    WiFiMulti.addAP(upwdSsid, upwdPswd);
    //
    while(WiFiMulti.run() != WL_CONNECTED) {
      if ( (bbrdRctl & RCTL_ARTI) == 0) {
        Serial.print(F("~"));
      }
      delay(250);
    }
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WMul connected as local IP: ");
      Serial.println(WiFi.localIP());
    }
    webSocket.begin();
    //    webSocket.setAuthorization("user", "Password"); // HTTP Basic Authorization
    webSocket.onEvent(webSocketEvent);
#else // No SOCKS
    //  Wifi Setup 
    while (WiFi.status() != WL_CONNECTED) {
      if ( (bbrdRctl & RCTL_ARTI) == 0) {
      Serial.println(F("# WiFi.begin() "));
      }
      WiFi.begin(upwdSsid, upwdPswd);
      if ( (bbrdRctl & RCTL_ARTI) == 0) {
        Serial.println(F("# WiFi.begin ! CONN"));
      }
      delay(5000);
    }
    if ( !( bbrdRctl & RCTL_ARTI ))  {
      Serial.println(F("# WiFi connected as local IP:"));
      Serial.println(WiFi.localIP());
    }
#endif // WIFIMAN
//
#if WIFI_MQTT
    //  MQTT Setup 
    //    setup callbacks
    popcMqtt.onConnected(connCbck);
    popcMqtt.onDisconnected(discCbck);
    popcMqtt.onPublished(publCbck);
    popcMqtt.onData(dataCbck);
    if ( !( bbrdRctl & RCTL_ARTI)) {
      Serial.println(F("# WIFI_MQTT : call popcMgtt.Connect()"));
    }
    popcMqtt.connect();
    delay(8000);
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.println(F("# setup() Mqtt.connect timeout. calling popcSubs()"));
    }
    //
    popcSubs();
#endif // WIFI_MQTT
  }
  //    TickerScheduler(uint size);
  //    boolean add(uint i, uint32_t period, tscallback_t f, boolean shouldFireNow = false);
  //      ts.add(0, 3000, sendData)
  int shedRcod;
  shedRcod = popcShed.add( 0, 1000, cbck1000);
  shedRcod = popcShed.add( 1, 2000, cbck2000);
  shedRcod = popcShed.add( 2, 9000, cbck9000);
}
//  End wifi services
//
/// MQTT Pub-Sub: Wifi publish/subscribe messaging uses callback functions 
//
void connCbck() {
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
    Serial.println(F("# MQTT srvr connCbck"));
  }
}

void discCbck() {
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
    Serial.println(F("# MQTT srvr disc"));
    delay(100);
  }
  #if WIFI_MQTT 
  popcMqtt.connect();
  delay(8000);
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println(F("# discCbck: Mqtt.connect timeOut"));
  }
  //Je18     
  popcSubs();
  #endif  
}

void publCbck() {
  //if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.println(F("# popc publCbck"));
  //}
}

void dataCbck(String& topic, String& data) {
  int   topiIndx;
  float topiValu;
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print(F("# dataCbck topic:"));  Serial.print(topic);
    Serial.print(F("   data:"));  Serial.println(data);
  }
  topiIndx = topic.indexOf("pidc/Kp");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcKp) {
      pidcKp = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print(F("# dataCbck-new Kp: "));  Serial.println(topiValu);
      }
    }
  }
  topiIndx = topic.indexOf("pidc/Td");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTd) {
      pidcTd = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print(F("# dataCbck-new Td: "));  Serial.println(topiValu);
      }
    }
  }
  topiIndx = topic.indexOf("pidc/Ti");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTi) {
      pidcTi = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print(F("# dataCbck-new Ti: "));  Serial.println(topiValu);
      }
    }
  }
  topiIndx = topic.indexOf("pidc/Beta");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcBeta) {
      pidcBeta = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print(F("# dataCbck-new Beta: "));  Serial.println(topiValu);
      }
    }
  }
  topiIndx = topic.indexOf("pidc/Gamma");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcGamma) {
      pidcGamma = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print(F("# dataCbck-new Gamma: "));  Serial.println(topiValu);
      }
    }
  }
  topiIndx = topic.indexOf("userCmdl");
  if (topiIndx >= 0){
    // copy data into user command line
    //
    userCmdl = String(data);
    //data.getBytes((byte[])userCmdl, userCmdl.length());
    //for ( tempIntA = 0; tempIntA < userCmdl.length(); tempIntA++ ) {
    //  userCmdl[tempIntA] = data.charAt(tempIntA) ;
    //  if (data.charAt(tempIntA) == '\0') {break;}
    //}
  }
  // //test 
  userRctl |= RCTL_ATTN; 
  //if ( 0 ) {
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print(F("# dataCbck-userCmdl - "));  Serial.println(userCmdl);
  }
}

//
void wrapPubl( const char * tTops , const char * tVals, int tInt ) {
  int rCode = 999;
#if PROC_ESP  
  rCode = 998;
#if WIFI_MQTT  
  rCode = 997;
  rCode = popcMqtt.publish( (const char * )tTops , (const char * )tVals, tInt );
#endif
#endif
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    //Serial.print  (F("# wrapPubl popcMqtt.publish topic : "));
    //Serial.println(tTops);
    //Serial.print  (F(" tVals : "));
    //Serial.print  (tVals);
    //Serial.print  (F(" RC : "));
    //Serial.println(rCode);
  //}
  //delay(100);
}

// Wrapper for common MQTT subscribe functions 
void wrapSubs( const char * tTops ) {
  int rCode = 999;
#if PROC_ESP  
  rCode = 998;
#if WIFI_MQTT  
  rCode = popcMqtt.subscribe( (const char * )tTops );
#endif
#endif
  //delay(100);
}

//  Subscribe to MQTT topics, using wrapper
void popcSubs() {
  int rCode = 0;
  wrapSubs( userTops );
}

/// ESP TickerScheduler callbacks and service functions 
// Every 1sec Callback and Service 
void cbck1000() {
  cb10Rctl |= RCTL_ATTN;
}

void cb10Svce() {
  int rCode = 0;
  //
  wrapPubl( (const char * )inf0Tops, (const char * )(bbrdLin0), sizeof(bbrdLin0) ); 
  wrapPubl( (const char * )inf1Tops, (const char * )(bbrdLin1), sizeof(bbrdLin1) ); 
  // Artisan interface                      'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
  wrapPubl( (const char * )ArspTops , (const char * )artiResp, sizeof(artiResp) ); 
  //  dtostrf( adc0Curr, 8, 3, mqttVals);
  //  
  dtostrf( millStep, 8, 3, mqttVals);
  wrapPubl( c100Tops, (const char *)(mqttVals), sizeof(mqttVals) ); 
  cb10Rctl &= ~RCTL_ATTN;
}

// Every 2sec Callback and Service 
void cbck2000() {
  cb20Rctl |=  RCTL_ATTN;
}

void cb20Svce() {
  int rCode = 0;
  // 2Sec sensor sample 
  dtostrf( *pidcSens, 12, 3, mqttVals);
  wrapPubl( c200Tops, (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  cb20Rctl &= ~RCTL_ATTN;
}

// Every 9sec Callback and Service 
void cbck9000() {
  cb90Rctl |= RCTL_ATTN;
}

void cb90Svce() {
  int rCode = 0;
  dtostrf( histRoc1, 12, 3, mqttVals);
  wrapPubl( c900Tops, (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  cb90Rctl &= ~RCTL_ATTN;
}
#endif  //  ( PROC_ESP && WITH_WIFI)
  
/// Arduino Setup 
//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.setTimeout(100);
  delay(100);
  eprmInit();
  pidcInit();
  pidcRset();
  pidcStop();
  pwmdInit();
  rotsInit();
  userInit();
  tcplInit();
  virtTcplInit();
  millInit();
  initMavs( AMBI_TMPC, INIT_PWMD);
  profInit();
#if WITH_PCF8574
  twioInit();
#endif
#if PROC_ESP
#if WITH_WIFI
  wifsInit();
#endif // WITH_WIFI
#else 
#endif // PROC_ESP
//
#if WITH_LCD
  lcdsInit();
#endif
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println(F("# popcEsp init() ends"));
  } 
}

/// Arduino Loop 
//
void loop() {
  // put your main code here, to run repeatedly:
  pidcLoop();
  userLoop();
  pwmdLoop();
  rotsLoop();
#if WITH_VIRTTCPL
  virtTcplLoop();
#endif 
#if ( WITH_MAX31855 || WITH_MAX6675)
  tcplLoop();
#if WITH_TCPL_2
  tcp2Loop();
#endif  // WITH_TCPL_2
#endif  // ( WITH_MAX31855 || WITH_MAX6675 )
  millLoop();
  profLoop();
#if WITH_LCD
  lcdsLoop();
#endif 
  userLoop();
  //
#if ( PROC_ESP && WITH_WIFI) 
  if (cb10Rctl & RCTL_ATTN) {
    cb10Svce();
  }
  if (cb20Rctl & RCTL_ATTN) {
    cb20Svce();
  }
  if (cb90Rctl & RCTL_ATTN) {
    cb90Svce();
  }
  popcShed.update();
#if WIFI_SOKS
  webSocket.loop(); 
#endif  // WIFI_SOKS
#endif  // PROC_ESP && WITH_WIFI
  // Why     delay(10);
}
