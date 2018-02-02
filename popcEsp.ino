/// roboPopc Arduino UNO / ESP8266 Controller / Artisan Logger with MQTT client 'popc'
// 
//  Sections (units) in this code, ordered alphabetically:
//  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
//  eprm  EEPROM setup and handling 
//  frnt  deprecated, sends data over Serial to Frontside / Process apps on PC for PID tuning   
//  lcds  support for I2C 2x16 LCD display                             
//  mill  fast ~1mSec sequencer inc A/D, Off-On, modulo variable freq/pwmd
//  mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
//  offn  On-Off <= mains cycle/2 rate cycle PSW for SSR control (eg heater
//  pidc  PID controller for PWM powered temperature control; a delta-time summing anti windup PID 
//  pwmd  8-bit PWM control via freq >= ~30Hz hardware pwm pins 
//  prof  Profile control; selects auto/manual temp setpt, manual pwm width, real/fake temp sensor
//  rots  Rotary 16way encoded ( 4pin + common) selector switch manager
//  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
//  twio  PCD8574 I2C 8b IO for (esp) rotary sw via 4bit in + 4b i/o eg indl, SSR on/off drives
//  user  receive user's or Artisan commands via serial port, MQTT for setpoint, ramp, profiles 
//    
//  Arduino H/W details:  
//
//  nano: Ser:0,1  ExtInt:2 PWM:9,10(490HzTmr1) 3,11(490HzTmr2) PWM:5,6 (980HzTmr0 + mS, delay)
//        SPI:10,11,12,13 I2C SDA:A4 SCL:A5 LED:13
//
//  pins:  lcdi: A4 A5; 
//      tcplSPI:        D3 D5    D10
//          pwm:              D9      
// 
//  Command - Response supported for Artisan interface:
//  CHA       #    Acknowlege command, no action
//  IO3 nn         Set duty cycle to nn <= 100    
//  OT1 nn         Set duty cycle to nn <= 100    
//  OT2 nn         Set duty cycle to nn <= 100    
//  REA            Send Artisan formatted response line
//  UNC            Set units to Centigrade 
//  UNF            Set units to Fahrenheit
//  PID SV nnn     Set new target setpoint temp to nnn
// 
// 
//  Command & LCD Indicators; Upcase: User Set; LowCase: Auto/Sensed/Readback value 
//  a/A     Toggle run time interface to Artisan 
//  Bff     Set PID Beta parameter (Float; Expert only ! )  
//  c/C     Set centigrade units; LCD display actual/target temp
//  d/D     toggle diagnostic verbose messages on serial 
//  e/E     Readback / Update EEPROM PID parameters 
//  f/F     Set fahrenheit units; LCD display actual/target temp
//  Gff     Set PID Gamma parameter (Float; Expert only ! )  
//  h/H     Set hold temperature setpoint fore ramp endpoint ( soak temp )
//  Iff     Set PID I-Term time (Float Ti: lower value == higher gain)
//  Jff     Set PID D-Term gain (Float Td: lower value == lower  gain)
//  Kff     Set PID Gain TComp  (Float Kappa: 0 == no Temp comp)
//  l/L     Send Artisan CSV format on serial ( for capture and Artisan Import )  
//  m/M     Rsvd MQTT msg  
//  n/N     Rsvd NetSock + 
//  o       
//  Pff     Readback / Set PID P-Term gain (Float Kp: lower value == lower  gain)
//  q/Q     Query Readback PIDC operating (not eeprom) parameters / Q tbd
//  rnn/Rnn Set Temperature Ramp C/F Deg per min (Set before hold temp) 
//  snn/Snn Set immediate target setPoint C/F temperature  
//  t/T     Spare
//  u/U     Set PWM Frequency Hz (TBD)  
//  v/V     Readback firmware version, PID, EEPROM parms to serial 
//  wnn/Wnn Set PWM Duty Cycle nn <= 100, disable PID
//  x/X     tbd rddbk  / wRite  x axis parms
//  y/Y     tbd
//  z/Z     Zero reset all timecounts
//
//  Copyright (c) 2017, 2018 Bitwise Technologies  popc@bitwisetech.com  
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
///
//  Code compiler switches: 1/0 Enab/Dsel UNO-ESP proc HW, Wifi options - Rebuild, Upload after changing these 
#define PROC_UNO      1                  // Compile for Arduino Uno
#define NEWP_UNO      1                  //   Uno new pins polly 
#define PROC_ESP      0                  // Compile for ESP8266
#define WITH_LCD      0                  // Hdwre has I2C 2x16 LCD display
#define WITH_MAX31855 0                  // Hdwre has thermocouple + circuit
#define WITH_PCF8574  0                  // Hdwre has I2C I/O Extender      
#define WITH_OFFN     0                  // Use ~250cy via mill Off-On SSR, not fast h/w PWM
#define WIFI_MQTT     0                  // Compile for Wifi MQTT client
#define WIFI_SOKS     0                  // Compile for Wifi Web Sckt Srvr
#define WIFI_WMAN     0                  // Compile for Wifi Manager
#define IFAC_ARTI     1                  // Start with Artisan interface on Serial
#define IFAC_FRNT     0                  // Obsolete Front/Process interface on Serial 
///
// UNO pin assignments
#if PROC_UNO
#if NEWP_UNO
// Off/On SSR driver 
#define OFFN_OPIN  2
// Onboard Led indicates duty cycle
#define ONBD_OPIN LED_BUILTIN
#define ONBD_LOWON  0         
// PWM Drive
// d9 needed by RFI scan  d6 would use tmr0 want d3 used by max13855
#define PWMD_OPIN  9                        // Pin D6
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ  31                       // UNO: timer counter range
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3  6                        // Pin Val 8 
#define ROTS_BIT2 10                        // Pin Val 4 
#define ROTS_BIT1 11                        // Pin Val 2 
#define ROTS_BIT0 12                        // Pin Val 1 
// spi2 on NEWP_ for alternative tcpl interface  (excludes twio)
#define SPI2_CLCK  3                        // Pin Clock
#define SPI2_MISO  5                        // ( Pin D4 used TCPL
#define SPI2_CSEL  2                        // Pin CSel
//#define RSVD_MOSI  5                        // Pin D5 Rsvd 
// spi on uno FOR tcpl
#define TCPL_CLCK  4                        // Pin Clock
#define TCPL_MISO  7                        // Pin Data
#define TCPL_CSEL  8                        // Pin CSel
// i2c on pcf8574 TWIO (excl spi2)
#define TWIO_SDA   5     // I2C SDA
#define TWIO_SCL   3     // I2C SCL
//
#define SCOP_OPIN  2     // debug flag nixes SPI2_CSEL
// macros to toggle scope output pin specified above for logic analyser
#define scopHi digitalWrite( SCOP_OPIN, 1)
#define scopLo digitalWrite( SCOP_OPIN, 0)
///
#else    // NOT NEWP_UNO
//
// Off/On SSR driver 
#define OFFN_OPIN  5
// Onboard Led indicates duty cycle
#define ONBD_OPIN LED_BUILTIN
#define ONBD_LOWON  0         
// PWM Drive
// d9 needed by RFI scan  d6 would use tmr0 want d3 used by max13855
#define PWMD_OPIN  9                        // Pin D9
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ  31                       // UNO: timer counter range
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3  6                        // Pin Val 8 
#define ROTS_BIT2 10                        // Pin Val 4 
#define ROTS_BIT1 11                        // Pin Val 2 
#define ROTS_BIT0 12                        // Pin Val 1 
// spi on uno FOR tcpl
#define TCPL_CLCK  4                        // Pin Clock
#define TCPL_MISO  7                        // Pin Data
#define TCPL_CSEL  8                        // Pin CSel
//
//
#define SCOP_OPIN   2    // debug flag is +OnBrdLed
#endif   // !NEWP_UNO
#endif   // PROC_UNO

///
// ESP Pin Assignments 
#if PROC_ESP
// A/D 1v 10b on 
// Off/On SSR driver GPIO 16 deepWake     
#define OFFN_OPIN 16
// Handle either LED polarity with: (1 & ~ONBD_LOWON) for light, (0 | LED_LOWN) for dark 
#define ONBD_OPIN  0
#define ONBD_LOWON  1
// PWM Drive SSR driver GPIO 2 BLed  
#define PWMD_OPIN  2
#define PWMD_MODE  OUTPUT
#define PWMD_FREQ  33
// spi on ESP FOR tcpl
#define TCPL_MISO 12  // SPI Mstr In Slve Out 
#define TCPL_CLCK 14  // SPI SCk
#define TCPL_CSEL 15  // SPI ChipSel 10K Gnd 
// PCF8574 ESP: ROTS Rotary 16way encoder switch;
// i2c on esp for TWIO pcf8574
#define TWIO_SDA   4     // I2C SDA
#define TWIO_SCL   5     // I2C SCL
// 
#define SCOP_OPIN  13    // debug flag uses 'MOSI' line  
//
#endif   // PROC_ESP

///
//   WiFi preamble for ESP with Wifi Router ID, PW Info 
#if PROC_ESP
#include <Adafruit_ESP8266.h>
//
// upward wifi: replace with your own network router's SSID, Password
//
const char* upwdSsid = "myRouterAddx";
//
const char* upwdPwrd = "myRouterPswd";
// downward wifi: for esp8266 access point replace with AP's SSID, Password
//
const char* dnwdSsid = "myAPsSSID";
//
const char* dnwdPwrd = "myAPsPswd";
//
// wifiManager.autoConnect("upwdSsid", "password");
//
// Beg paste from pubsShed 
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
// MQTT myMqtt(MQCL_ID, "test.mosquitto.org", 1883);
//
//
MQTT popcMqtt(MQCL_ID, "myPCwithPaho", 1883);
#endif  // WIFI_MQTT

//Jn01 WifiManager 
#if WIFI_WMAN
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#endif  // WIFI_WMANT
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
#endif  // PROC_ESP

/// system settings 
//  milliSecond poll values Primes to suppress beating 
#define ADC0_POLL_MILL    2UL            // mS lcd display poll
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define MILL_POLL_USEC 1000UL            // uS 1KHz mill   poll
#define PIDC_POLL_MSEC  101UL            // mS pid control poll
#define PROF_POLL_MSEC  997UL            // mS run control poll
#define PWMD_POLL_MSEC  103UL            // mS pwm driver  poll
#define ROTS_POLL_MSEC  503UL            // mS rotary sw   poll
#define TCPL_POLL_MSEC   97UL            // mS termocouple poll
#define VTCP_POLL_MSEC  251UL            // mS virt tcpl   poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec
//
#if 0
// milliSecond poll values
#define TCPL_POLL_MSEC  100UL            // mS termocouple poll
#define PWMD_POLL_MSEC  100UL            // mS pwm driver  poll
#define PIDC_POLL_MSEC  100UL            // mS pid control poll
#define VTCP_POLL_MSEC  250UL            // mS virt tcpl   poll
#define ROTS_POLL_MSEC  500UL            // mS rotary sw   poll
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define PROF_POLL_MSEC 1000UL            // mS run control poll
#define MILL_POLL_USEC 4000UL            // uS 240Hz mill  poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec
#endif 

//
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

// lcd                                     
// Pin A4 Pin A5 i2c
// set LCD address to 0x27 for a A0-A1-A2  display
//   args: (addr, en,rw,rs,d4,d5,d6,d7,bl,blpol)
#if WITH_LCD
#if PROC_UNO
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif 
#endif 

// 
#if WITH_MAX31855
#if PROC_UNO
#include "Adafruit_MAX31855.h"
Adafruit_MAX31855 tcpl(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#endif
#if PROC_ESP
#include "MAX31855.h"
MAX31855 tcpl(TCPL_CLCK, TCPL_CSEL, TCPL_MISO);
#endif  //  ESP
#endif  //  MAX31855

// TWIO pcf8574
#if WITH_PCF8574
#define TWIO_IMSK  0x0F  // PCF port 3-0 pin  7-4 are inputs 
#define TWIO_OMSK  0xF0  // PCF port 7-4 pin 12-9 are outputs 
//
#define TWIO_ADDX  0x20  // A0,A1, A2 Gnd
#define ADC0_TWPO  4     // Curr Hi-lo v Adc0 Avg prt 4  pin  9
#define MILL_TWPO  5     // mill fast io          prt 5  pin 10
#define OFFN_TWPO  6     // offn                  prt 6  pin 11
#define SPAR       7     // mill fast io          prt 7  pin 12
#include "PCF8574.h"
void twioInit(void);
byte twioRead8(void);
void twioWritePin(byte, byte);
void twioWrite8(byte);
PCF8574 twio( TWIO_ADDX );
int twioCurr;
#endif

/// Declarations by unit
//
float   ambiTmpC  = 28;                    //  28C  82F rm temp then W0 + fan htr temp
#define idleTmpC    50                     //  50C temp with W0 due to fan heater coil  
#define maxiTmpC    248                    // 248C 480F as maximum temp 

// A/D Chan 0 
long adc0Curr, adc0Prev, adc0Maxi, adc0Mini, adc0Avge, adc0Bit0; 

// Artisan 40+ char serial pkt: ambient, ch1, ch2, ch3, ch4 or Logging Tt Ts BT ET SV Duty
char artiResp[] = "023.0,128.8,138.8,000.0,000.0          ";  // 39 + null
// buffer for mqtt Artisan Program interface
char artiProg[] = "                        ";  // 23 + null
// Artisan csv header format: these two lines must contain tab chars, not spaces
const char csvlLin1[] = "Date:	Unit:C	CHARGE:	TP:	DRYe:	FCs:	FCe:	SCs:	SCe:	DROP:	COOL:	Time:";
const char csvlLin2[] = "Time1	Time2	BT	ET	Event	SV	DUTY";

// billboard string for LCD + Serial  Lower cases: computed/measured Upper case: User/Setpoints 
char bbrdLin0[] = "w100% r-123 128c"; 
char bbrdLin1[] = "P0-0 S12.3m 228C";
char bbrdHold   = 'H';                      // Prefix to decimal mins alternating with total time
char bbrdManu   = 'M';
char bbrdRamp   = 'R';
char bbrdSetp   = 'S';
char bbrdTmde;
char dbugLine[] = " <==>                                                                           ";
char centScal   = 'C';
char fahrScal   = 'F';
char userScal   = 'C';

//
String fromSeri;
String     userCmdl("                               ");  // 39Chrs + null
//String     userCmdl = "                               ";  // 39Chrs + null
char   userChrs[] = "023.0,128.8,138.8,000.0,000.0           ";  // 40sp 39 + nullch 

#if IFAC_FRNT
// FRNT
union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
frntComm;              // float array
byte Auto_Man = -1;
byte Direct_Reverse = -1;
unsigned long frntPoll; //this will help us know when to talk with processing
#endif

//eprm
float fromEprm;
int eprmSize, eprmFree;
// Addresses at end of EEPROM for saved PID parameters
#define EADX_KP (eprmSize - 1 * (sizeof(float)))
#define EADX_TI (eprmSize - 2 * (sizeof(float)))
#define EADX_TD (eprmSize - 3 * (sizeof(float)))
#define EADX_BE (eprmSize - 4 * (sizeof(float)))
#define EADX_GA (eprmSize - 5 * (sizeof(float)))
#define EADX_KA (eprmSize - 6 * (sizeof(float)))

// i-n-g-o MQTT 
// mqtt strings are declared for both ESP8266 and UNO 
char mqttVals[] =  "                ";                     // mqtt value 16sp 15ch max
// General Info topics 
const char adc0Tops[]  = "/popc/adc0Curr";
const char c900Tops[]  = "/popc/cbck9000";
const char echoTops[]  = "/popc/echoCmdl";
const char inf0Tops[]  = "/popc/bbrdLin0";
const char inf1Tops[]  = "/popc/bbrdLin1";
const char psecTops[]  = "/popc/stepSecs";
const char dutyTops[]  = "/popc/pwmdDuty";
const char freqTops[]  = "/popc/pwmdFreq";
const char ptmpTops[]  = "/popc/profDegs";
const char dgpmTops[]  = "/popc/sensDgpm";
const char userTops[]  = "/popc/userCmdl";
// Artisan interface UC names  'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
const char AmbiTops[]  = "/popc/arti/ATmp";
const char ArspTops[]  = "/popc/arti/Arsp";
const char ETmpTops[]  = "/popc/arti/ETmp";
const char BTmpTops[]  = "/popc/arti/BTmp";
const char PTmpTops[]  = "/popc/arti/PTmp";
const char PwmdTops[]  = "/popc/arti/Pwmd";
const char AOT1Tops[]  = "/popc/arti/AOT1";
const char AOT2Tops[]  = "/popc/arti/AOT2";
const char AIO3Tops[]  = "/popc/arti/AIO3";
// PID controller topics
const char RnTops[]    = "/popc/pidc/Rn";
const char YnTops[]    = "/popc/pidc/Yn";
const char EnTops[]    = "/popc/pidc/En";
const char UnTops[]    = "/popc/pidc/Un";
const char KpTops[]    = "/popc/pidc/Kp";
const char TdTops[]    = "/popc/pidc/Td";
const char TiTops[]    = "/popc/pidc/Ti";
const char BetaTops[]  = "/popc/pidc/Beta";
const char GammaTops[] = "/popc/pidc/Gamma";
const char KappaTops[] = "/popc/pidc/Kappa";

//pidc
//Ap15
// Fast response PID to match approx 30Hz PWM frequency 
//  Date  Kp      Ti      Td      Beta      Gamma 
// Jn04   2.400   8.000   0.025   1.000     1.000  Jn04-Migs-Furn BBSF     
// Jn04++ Kp applies only to Pn no more to Ti, Td 
//           comp: Rdce Ti, Incr Td by Kp       
//        2.400   3.333   0.060                    Jn04++ Theor sett
// 17Jn10 1.75 4.50 0.448 1.0 1.0 post Kt-Kp adj Kick up when ramp lowered
// 17Jn10 2.00 5.00 0.320 1.0 1.0 tune: was slow on 20-10-5                
// 17Jn14 2.25 4.25 0.250 1.0 1.0 Je14 Ethi need more Kp
// 17Jn17 2.50 4.00 0.100 1.0 1.0 Je17 Good ESP Virt                 
// 17Jn17 2.00 3.00 0.100 1.0 1.0 Je17 Migs-Furn slow osc          
// 17Jn17 2.40 3.33 0.060 1.0 1.0 Je17 attempt Jn04 clone          
// 17Je22 2.50 3.00 0.005 1.0 1.0 Je22
// 17Au04 4.00 2.00 0.005 1.0 1.0 Je22
// 17Au31 2.00 2.00 1.000 2.0 1.0 0.25 Kappa comp 
// 17Se02 2.00 2.00 0.250 2.0 1.0 0.25 Overshoots S80->s180 Reduce Td  
// 17Se02 2.50 2.50 0.250 2.0 1.0 0.20 Less area at OShoot? Inc Kp, Ti, Ka
// 17Se15 2.50 4.00 0.250 2.0 1.0 0.20 Attempt
//
float pidcKp    =   2.500;                // P-Term gain
float pidcKc    =   2.500;                // P-Term gain
float pidcTi    =   4.000;                // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd    =   0.250;                // D-Term Gain sec ( Td++ = Gain++)
//
float pidcBeta    =   2.000;              // P-term Refr vs YInp
float pidcGamma   =   1.000;              // D-term Refr vs YInp
float pidcAlpha   =   0.100;              // D-term Filter time
float pidcKappa   =   0.200;              // Ambient comp Kp * ( 1 + Ka (sens - idle)/idle )
//
float pidcRn      =  ambiTmpC;            // Refr setpoint
float pidcYn      =  ambiTmpC;            // YInp input
float pidcEn      =   0.000;              // YInp input
float pidcUMax    = 255.000;              // Outp Max
float pidcUMin    =   0.000;              // Outp Min
float dUn, Edn, Epn, Epn1          = 0.0; // Calc error values
float Edfn2, Edfn1, Edfn, Un, Un1  = 0.0; // Calc error, output, prev output values
float Tf, Ts, TsDivTf              = 0.0; // Filter, Actual sample period, stash
float pidcPn, pidcIn, pidcDn       = 0.0; // per sample P-I-D-Err Terms
float pidcPc, pidcIc, pidcDc       = 0.0; // cumulative P-I-D components 
float pidcUn = 0.0;                       // PID controller Output
// 
const char versChrs[] = "2018Fe02-Publ-Arti-Ifac";
/// wip: stored profiles
// profiles
//   stored as profiles 1-9 with steps 0-9 in each 
struct profTplt {
  float profTarg;                         // Ramp endpoint or Setpoint if RMin == 0
  float profRMin;                         // Ramp time to TargTemp decimal minutes
  float profHMin;                         // Hold time at TargTemp decimal minutes 
}; 
//
profTplt profLine = { idleTmpC, 0, 99 };
//
#define EADX_PROF 0                       // EEPROM starting address of stored profiles
#define STEP_SIZE 3 * sizeof( float)
#define PROF_SIZE 9 * STEP_SIZE
// prof, line and parm are based 1 - n 
profTplt profStep( int prof ) {
  profTplt profWork;
  int eadx = EADX_PROF + (prof - 1) * PROF_SIZE;
  EEPROM.get( eadx, profWork);
  return profWork; 
}

// pwmd vbls
int  pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp;                          // Freq, Duty Cycle Target (255max) Output
int  heatHist[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // mavg 16  for virtTcpl 
int  degCHist[] = { 0, 0, 0, 0, 0, 0, 0, 0};                          // mavg store for temp rate of change 
byte pwmdPcnt;                                                        // Percent duty cycle 

// Run Cntl vbls  Bit 0:Run 1:Atto 2:Manu 3:Attn 4:spare 5:Dbug 6:Arti 7:Info 
#define RCTL_RUNS 0x80
#define RCTL_AUTO 0x40
#define RCTL_MANU 0x20
#define RCTL_ATTN 0x10
#define RCTL_SPAR 0x08
#define RCTL_DIAG 0x04
#define RCTL_ARTI 0x02
#define RCTL_INFO 0x01

#if IFAC_ARTI
byte  bbrdRctl =  RCTL_ARTI;
#else
//byte  bbrdRctl =  RCTL_INFO | RCTL_DIAG; 
byte  bbrdRctl =  RCTL_INFO;
#endif
byte  frntRctl  = 0x00;
#if WITH_LCD
byte  lcdstRctl = RCTL_RUNS;
#else
byte  lcdstRctl = 0x00;
#endif
byte  pidcRctl  = (RCTL_RUNS | RCTL_AUTO );
byte  pwmdRctl  = (RCTL_RUNS | RCTL_AUTO);
#if WITH_OFFN
byte  offnRctl  = (RCTL_RUNS | RCTL_AUTO);
#else
byte  offnRctl  = RCTL_RUNS;
#endif
byte  profRctl  = RCTL_MANU;
byte  stepRctl  = 0x00;
byte  rampRctl  = 0x00;
byte  holdRctl  = 0x00;
byte  tcplRctl  = RCTL_RUNS;
byte  userRctl  = RCTL_RUNS;
byte  wifiRctl  = RCTL_RUNS;

// scheduler tick callback attn flags 
byte  cb10Rctl  = RCTL_RUNS;
byte  cb20Rctl  = RCTL_RUNS;
byte  cb90Rctl  = RCTL_RUNS;

byte profNmbr, stepNmbr, profChar, stepChar;    // Numeric Profile No, Step No, Character values 

//  Rotary Switch
byte rotsCurr, rotsNewb, offnOutp;   // Current value, newb test for change; off/on cycle counter, output 

//  time markers compared with uS mS and mill count 
unsigned long adc0Mark, adc0Poll                               = 0UL;
unsigned long lcdsMark, millStep, millMark, pidcElap, pidcMark = 0UL;
unsigned long profMark, pwmdMark, rotsMark, tcplMark, vtcpMark = 0UL;
//unsigned long lcdsMark, pidcMark, profMark = 0UL;
//unsigned long pwmdMark, rotsMark, tcplMark = 0UL;
//unsigned long vtcpMark, millMark, millStep = 0UL;
//

#if PROC_ESP
#include "TickerScheduler.h"
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
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch(type) {
    case WStype_DISCONNECTED:
      if ( !( bbrdRctl & RCTL_ARTI ) ) {
        USE_SERIAL.printf("[%u] popcSockSrvr Disconnected!\n", num);
      }  
    break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        USE_SERIAL.printf("[%u] popcSockSrvr Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }  
    	// send message to client
		  webSocket.sendTXT(num, "popcSockSrvr acks Connected");
    }
    break;
    case WStype_TEXT:
      //if ( !(  Rctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      if ( !( bbrdRctl & RCTL_ARTI ) ) {
        USE_SERIAL.printf("popcSockSrvr [%u] get Text: %s\n", num, payload);
      }  
      // send message to client
      // webSocket.sendTXT(num, "popcSockSrvr message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("popcSockSrvr message here");
    break;
    case WStype_BIN:
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      USE_SERIAL.printf("[%u] popcSockSrvr get binary length: %u\n", num, lenght);
    }  
    hexdump(payload, lenght);

    // send message to client
    // webSocket.sendBIN(num, payload, length);
    break;
  }
}
#endif //WIFI_SOKS
//
#else 
#endif  // PROC_ESP

//
float         targTmpC, sensTmpC, prevTmpC;   // target, sensor, previous temperature deg C
int           userDuty, userDgpm, sensCdpm;   // userSet duty cycle; userSet C/F, meas C dg pm
int           userAOT1, userAOT2, userAIO3;   // Arti OT1Dty, Arti OT2Dty, Arti IO3Dty 
int           baseTmpC, rampCdpm, holdTmpC;   // Ramp start temp, hold at  endpoint 
int           stepSecs, holdTogo, totlSecs;   // Step elapsed, hold countdown, total run time
int           userDegs, tempIndx;             // User C/F temp,  temporary array indexer

/// Common 
//    Convert
float floatCtoF( float celsInp) {
  return (float( ((celsInp + 40.0) * 9.0 / 5.0 )  -40.0 ));
}  

// Not needed: Degs per min conversions need not to have offset 
//float floatFtoC( float fahrInp) {
  //return (float( ((fahrInp + 40.0) * 5.0 / 9.0 )  -40.0 ));
//}  
  
// integer fns add 0.5 for rounding
//int    intgCtoF( int   celsInp) {
  //return (int  ( ((celsInp + 40.0) * 9.0 / 5.0 )  -39.5 ));
//} 
 
//
int    intgFtoC( int   fahrInp) {
  return (int  ( ((fahrInp + 40.0) * 5.0 / 9.0 )  -39.5 ));
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
  
/// Billboard LCDisplay and Serial info Lines 
void  bbrdArti() {
  //// Artisan Iface : resp 'READ' = Amb,Ch1,2,3,4 Amb,ET,BT,PT,PW
  // Je 15 to get SV/Duty into Config-Device-Extra TC4Ch3-4  send Amb ET, BT SV, D% 
  // Je15 Artisan Iface : resp 'READ' = Amb,Ch1,2,3,4 Ta,Te,Tb,Te,Du
  if ( userScal == fahrScal) {                                
    dtostrf( floatCtoF(ambiTmpC), 5, 1,  &artiResp[0]  );               // Art's Ta Ch
    //if (floatCtoF(ambiTmpC) < 100)      { artiResp[0]  = '0'; }
    //if (floatCtoF(ambiTmpC) <  10)      { artiResp[1]  = '0'; }
    //if (floatCtoF(ambiTmpC) <   1)      { artiResp[2]  = '0'; }
    dtostrf( floatCtoF(targTmpC), 5, 1,  &artiResp[6]  );               // ET
    //if (floatCtoF(targTmpC) < 100)   { artiResp[6]  = '0'; }
    //if (floatCtoF(targTmpC) <  10)   { artiResp[7]  = '0'; }
    //if (floatCtoF(targTmpC) <   1)   { artiResp[8]  = '0'; }
    dtostrf( floatCtoF(sensTmpC), 5, 1, &artiResp[12] );               // BT
    //if (floatCtoF(sensTmpC) < 100)   { artiResp[12]  = '0'; }
    //if (floatCtoF(sensTmpC) <  10)   { artiResp[13]  = '0'; }
    //if (floatCtoF(sensTmpC) <   1)   { artiResp[14]  = '0'; }
    dtostrf( int(sensCdpm * 9.00 / 5.00 + 50 ), 5, 1, &artiResp[18] ); // SV  
    //if (floatCtoF(sensCdpm) < 100)   { artiResp[18]  = '0'; }
    //if (floatCtoF(sensCdpm) <  10)   { artiResp[19]  = '0'; }
    //if (floatCtoF(sensCdpm) <   1)   { artiResp[20]  = '0'; }
    dtostrf( floatCtoF(targTmpC), 5, 1,  &artiProg[0]  );               // Art's Ta Ch
    dtostrf( floatCtoF(sensTmpC), 5, 1,  &artiProg[6]  );               // ET
    dtostrf( int(sensCdpm * 9.00 / 5.00 + 50 ), 5, 1, &artiProg[12] ); // SV  
  } else {
    dtostrf(           ambiTmpC,  5, 1, &artiResp[0]  );               // AT
    //if (ambiTmpC < 100)   { artiResp[0]  = '0'; }
    //if (ambiTmpC <  10)   { artiResp[1]  = '0'; }
    //if (ambiTmpC <   1)   { artiResp[2]  = '0'; }
    dtostrf(           targTmpC,  5, 1, &artiResp[6]  );               // ET
    //if (targTmpC < 100)   { artiResp[6]  = '0'; }
    //if (targTmpC <  10)   { artiResp[7]  = '0'; }
    //if (targTmpC <   1)   { artiResp[8]  = '0'; }
    dtostrf(           sensTmpC,  5, 1, &artiResp[12] );               // BT
    //if (sensTmpC < 100)   { artiResp[12]  = '0'; }
    //if (sensTmpC <  10)   { artiResp[13]  = '0'; }
    //if (sensTmpC <   1)   { artiResp[14]  = '0'; }
    dtostrf( int(sensCdpm + 50   ), 5, 1, &artiResp[18] );             // SV
    //if (sensCdpm < 100)   { artiResp[18]  = '0'; }
    //if (sensCdpm <  10)   { artiResp[19]  = '0'; }
    //if (sensCdpm <   1)   { artiResp[20]  = '0'; }
    dtostrf(           targTmpC,  5, 1, &artiProg[0]  );               // ET
    //if (targTmpC < 100)   { artiProg[0]  = '0'; }
    //if (targTmpC <  10)   { artiProg[1]  = '0'; }
    //if (targTmpC <   1)   { artiProg[2]  = '0'; }
    dtostrf(           sensTmpC,  5, 1, &artiProg[6] );                // BT
    //if (sensTmpC < 100)   { artiProg[6]  = '0'; }
    //if (sensTmpC <  10)   { artiProg[7]  = '0'; }
    //if (sensTmpC <   1)   { artiProg[8]  = '0'; }
    dtostrf( int(sensCdpm + 50   ), 5, 1, &artiProg[12] );             // SV
    //if (sensCdpm < 100)   { artiProg[12]  = '0'; }
    //if (sensCdpm <  10)   { artiProg[13]  = '0'; }
    //if (sensCdpm <   1)   { artiProg[14]  = '0'; }
  } 
  dtostrf(           pwmdPcnt,  5, 1, &artiResp[24] );                 // DU
    //if (pwmdPcnt < 100)   { artiResp[24]  = '0'; }
    //if (pwmdPcnt <  10)   { artiResp[25]  = '0'; }
    //if (pwmdPcnt <   1)   { artiResp[26]  = '0'; }
  dtostrf(           pwmdPcnt,  5, 1, &artiProg[18] );                 // DU
    //if (pwmdPcnt < 100)   { artiProg[18]  = '0'; }
    //if (pwmdPcnt <  10)   { artiProg[19]  = '0'; }
    //if (pwmdPcnt <   1)   { artiProg[20]  = '0'; }
  artiResp[5]  = ',';
  artiResp[11] = ',';
  artiResp[17] = ',';
  artiResp[23] = ',';
  artiResp[30] = '\0';
  artiProg[5]  = ',';
  artiProg[11] = ',';
  artiProg[17] = ',';
  artiProg[23] = '\0';
  artiProg[24] = '\n';
}  

//
void bbrdFill() {
  // billboard fill lcd sisplay 32 chars for lcd / mqtt / serial 
  // Billboard lines are filled for display on LCD and/or Serial with 'info' 
  // billboard line[0]; strf ops append null chars, fill single chars later 
  dtostrf( pwmdPcnt,                              3, 0, &bbrdLin0[1]);
  // Odd secs: show desred ROC; Even secs: show measured ROC 
  if ((stepSecs != 0) && ( totlSecs % 2 )){
    dtostrf( userDgpm,                           +4, 0, &bbrdLin0[7] );
  } else {
    if ( userScal == fahrScal) {
      dtostrf( int(    sensCdpm * 9.00 / 5.00),  +4, 0, &bbrdLin0[7] );
    } else {
      dtostrf(         sensCdpm,                 +4, 0, &bbrdLin0[7] );
    }  
  }
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(sensTmpC), 3, 0, &bbrdLin0[12] );
  } else {
    dtostrf(           sensTmpC,  3, 0, &bbrdLin0[12] );
  } 
  if ( pwmdRctl & RCTL_MANU) {
    bbrdLin0[0] = 'W';
  } else {
    bbrdLin0[0] = 'w';
  }  
  bbrdLin0[4]   = '%';
  bbrdLin0[5]  = ' ';
  // Odd secs: show desred ROC; Even secs: show measured ROC 
  if (( stepSecs != 0 ) && ( totlSecs % 2 )){
    bbrdLin0[6]  = 'R';
  } else {
    bbrdLin0[6]  = 'r';
  } 
  bbrdLin0[11] = ' ';
  bbrdLin0[15]  = userScal + 0x20;           // lowCase
  //
  // billboard line[1]; strf ops append null chars, fill single chars later 
  // At even seconds show total time, odd seconds show step time in min.minths   
  if ( totlSecs % 2 ) {
    dtostrf( (stepSecs / 60), 2, 0, &bbrdLin1[6]);
  } else {
    dtostrf( (totlSecs / 60), 2, 0, &bbrdLin1[6]);
  }  
  // alternate stepTime  + target Temp  with  totalTime + profile Temp
  if ( totlSecs % 2 ) {
    dtostrf( ((stepSecs / 6) % 10), 1, 0, &bbrdLin1[9]);
    if ( userScal == fahrScal) {
      dtostrf( floatCtoF(targTmpC),                 3, 0, &bbrdLin1[12] );
    } else {
      dtostrf(           targTmpC,                  3, 0, &bbrdLin1[12] );
    }  
    //bbrdLin1[15]  = userScal + 0x20;           // lowCase
    bbrdLin1[15]  = userScal;                  //  upCase
  } else {
    dtostrf( ((totlSecs / 6) % 10), 1, 0, &bbrdLin1[9]);
    if ( userScal == fahrScal) {
      dtostrf( floatCtoF(holdTmpC),                 3, 0, &bbrdLin1[12] );
    } else {
      dtostrf(           holdTmpC,                  3, 0, &bbrdLin1[12] );
    }  
    bbrdLin1[15]  = userScal;                  //  upCase
  }  
  // fill line[1] legend 
  bbrdLin1[0]   = 'P';
  bbrdLin1[1]   = nibl2Hex( profNmbr);
  bbrdLin1[2]   = '-';
  bbrdLin1[3]   = nibl2Hex(stepNmbr);
  bbrdLin1[4]   = ' ';
  if ( totlSecs % 2 ) {
    bbrdLin1[5] = bbrdTmde;
  } else {
    bbrdLin1[5] = 'P';
  }  
  bbrdLin1[8]   = '.';
  bbrdLin1[10]   = 'm';
  bbrdLin1[11]  = ' ';
  // fill info for serial output either Artisan protocol or console Info/csv logging 
  if (bbrdRctl & RCTL_ARTI ){
    // nothing is done here, for Artisan response usercmdl == 'REA' does bbrdArti etc
    // 12Dc fill the bbrd for mqtt but dont send serial 
    bbrdArti();
  } else {
    // ! RCTL_ARTI so use artiresp space to create csv response  
    if ( bbrdRctl & RCTL_INFO ) {
      // ~ARTI &  INFO : Non-CSV tbd
    } else {
      // ~ARTI & ~INFO : csv logging for Artisan import Select 1s / Ns : TotMin:Sec StepMin:sec BT ET Event SV Duty
      dtostrf( (totlSecs / 60), 02, -0, &artiResp[0]);
      dtostrf( (totlSecs % 60), 02, -0, &artiResp[3]);
      dtostrf( (stepSecs / 60), 02, -0, &artiResp[6]);
      dtostrf( (stepSecs % 60), 02, -0, &artiResp[9]);
      if ( userScal == fahrScal) {
        dtostrf( floatCtoF(sensTmpC),       5, 1, &artiResp[12] );
        dtostrf( floatCtoF(targTmpC),       5, 1, &artiResp[18] );
        dtostrf( floatCtoF(sensCdpm * 9.0/5.0  + 50),  5, 1, &artiResp[25] ); // Arti scale offset
      } else {
        dtostrf(           sensTmpC,        5, 1, &artiResp[12] );
        dtostrf(           targTmpC,        5, 1, &artiResp[18] );
        dtostrf(          (sensCdpm + 50),  5, 1, &artiResp[25] );
      }
      dtostrf(             pwmdPcnt,        3, 0, &artiResp[31] );
      // insert leading zero into timestamps 
      if (artiResp[0] == ' ') artiResp[0] = '0'; 
      if (artiResp[3] == ' ') artiResp[3] = '0'; 
      if (artiResp[6] == ' ') artiResp[6] = '0'; 
      if (artiResp[9] == ' ') artiResp[9] = '0'; 
      // fill fixed chars is response string, Artisan needs Tabs
      artiResp[2]  = ':';
      artiResp[5]  =  0x09;
      artiResp[8]  = ':';
      artiResp[11]  = 0x09;
      artiResp[17]  = 0x09;
      artiResp[23]  = 0x09;
      artiResp[24]  = 0x09;
      artiResp[30]  = 0x09;
      artiResp[34]  = ' ';   // overwrite <nul>
      artiResp[35]  = ' ';   // overwrite <nul>
      artiResp[36]  = ' ';   // overwrite <nul>
      artiResp[37]  = ' ';
      artiResp[38]  = ' ';
      artiResp[39]  = ' ';
      // Flag csv is ready for posting 
      bbrdRctl |= RCTL_ATTN;
    }
  }  
}

///  EEPROM 
void eprmInit() {
#if PROC_ESP
  // ESP8266 has no .length function 
  eprmSize = 1024;
#else  
  eprmSize = EEPROM.length();
#endif  
  eprmFree = eprmSize - ( 6 * sizeof(float));  // Reserve floats: Kappa Gamma Beta Td Ti Kp 
}

void eprmInfo() {
    Serial.print(F("EEPROM Size: "));
    Serial.print(eprmSize);
    Serial.print(F(" Free: "));
    Serial.println(eprmFree);
    EEPROM.get(EADX_KP, fromEprm);
    Serial.print(F("EEPROM Kp:"));
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
    Serial.println(fromEprm);
}

///  Front Side Serial Interface to PC 'Processing' graphing app
//
#if IFAC_FRNT
void frntRecv() {
  // read the bytes sent from Processing
  int index=0;
  while( (Serial.available()) && (index<26))
  {
    if(index==0) {
      Auto_Man = Serial.read();
    } else if (index==1) {
      Direct_Reverse = Serial.read();
    } else {
      frntComm.asBytes[index-2] = Serial.read();
      //Serial.print("\nIndx:");
      //Serial.g(index);
      //Serial.print(frntComm.asBytes[index]);
      //Serial.print("    ");
    }  
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if( (index==26)  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    targTmpC=float(frntComm.asFloat[0]);
    //sensTmpC=double(frntComm.asFloat[1]); // * the user has the ability to send the 
                                            //   value of "Input"  in most cases (as 
                                            //   in this one) this is not needed.
    if(Auto_Man==0)                         // * only change the output if we are in 
    {                                       //   manual mode.  otherwise we'll get an
      pidcUn=double(frntComm.asFloat[2]);      //   output blip, then the controller will 
    }                                       //   overwrite.
    
    double p, i, d;                         // * read in and set the controller tunings
    pidcKp = double(frntComm.asFloat[3]);
    pidcTi = double(frntComm.asFloat[4]);
    pidcTd = double(frntComm.asFloat[5]);
    //myPID.SetTunings(p, i, d);            //
    
    //if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    //else myPID.SetMode(AUTOMATIC);        //
    
    //if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT); // * set controller Direction
    //else myPID.SetControllerDirection(REVERSE);
  }
  Serial.flush();                           // * clear any random data from the serial buffer
}

void frntSend() {
  Serial.print(F("PID "));
  Serial.print(targTmpC);   
  Serial.print(F(" "));
  Serial.print(sensTmpC);   
  Serial.print(F(" "));
  Serial.print(pidcUn);   
  Serial.print(F(" "));
  Serial.print(pidcKp);   
  Serial.print(F(" "));
  Serial.print(pidcTd);   
  Serial.print(F(" "));
  Serial.print(pidcTd);   
  Serial.print(F(" "));
  //if(myPID.GetMode()==AUTOMATIC) Serial.print("Auto");
  if(Auto_Man==1) Serial.print(F("Auto"));
  else Serial.print(F("Manu"));  
  Serial.print(F(" "));
  //if(myPID.GetDirection()==DIRECT) Serial.println(F("Drct"));
  if(Direct_Reverse==1) Serial.println(F("Drct"));
  else Serial.println(F("Rvse"));
}

void frntLoop() {
  if(millis()>frntPoll)
  {
    frntRecv();
    frntSend();
    frntPoll += 500;
  }
}
#endif  // IFAC_FRNT


#if WITH_LCD
/// LCD DISPLAY
void lcdsInit() {
  bbrdTmde = bbrdSetp;
  lcd.begin(16, 2);
  lcd.home ();
  lcd.print(F("<== PopC-PID ==>"));
  lcd.setCursor ( 0, 1 );
  lcd.print(F("@bitwisetech.com"));
  delay ( 2000 );                //  1000mS startup delay
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
      delay ( 1000 );                //  1000mS startup delay
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

/// MQTT Pub-Sub 
//
void connCbck() {
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
    Serial.println("connCbck : connected to MQTT server");
  }
}

void discCbck() {
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
    Serial.println("discCbck disc from mqtt, pause ..");
    delay(1000);
  }  
  #if WIFI_MQTT 
  if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.println("discCbck : popcMqtt.connect pause  4 ..");
    }  
    popcMqtt.connect();
    delay(2000);
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("Wifi Mqtt.connect timed out. calling popcSubs()");
    }  
    //Je18     
    popcSubs();
  #endif  
}

void publCbck() {
  //if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.println("popc publCbck");
  //}  
}

void dataCbck(String& topic, String& data) {
  int   topiIndx;
  float topiValu;
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //if ( 0 ) {
    Serial.print("dataCbck topic:");
    Serial.print(topic);
    Serial.print("   data:");
    Serial.println(data);
  }  
  topiIndx = topic.indexOf("pidc/Kp");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcKp) {
      pidcKp = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("dataCbck-new Kp: ");
        Serial.println(topiValu);
      }  
    }  
  }
  topiIndx = topic.indexOf("pidc/Td");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTd) {
      pidcTd = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("dataCbck-new Td: ");
        Serial.println(topiValu);
      }  
    }  
  }
  topiIndx = topic.indexOf("pidc/Ti");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTi) {
      pidcTi = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("dataCbck-new Ti: ");
        Serial.println(topiValu);
      }  
    }  
  }  
  topiIndx = topic.indexOf("pidc/Beta");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcBeta) {
      pidcBeta = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("dataCbck-new Beta: ");
        Serial.println(topiValu);
      }  
    }  
  }
  topiIndx = topic.indexOf("pidc/Gamma");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcGamma) {
      pidcGamma = topiValu;
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("dataCbck-new Gamma: ");
        Serial.println(topiValu);
      }  
    }  
  }
  topiIndx = topic.indexOf("userCmdl");
  //char * charPntr = strncpy (userChrs, "userCmdl");
  if (topiIndx >= 0){
    // copy data into user command line
    //
    userCmdl = String(data);
    //strncpy(userChrs, data.c_str(), sizeof(userChrs));
    //data.getBytes((byte[])userCmdl, sizeof(userCmdl));
    //for ( tempIndx = 0; tempIndx < sizeof(userCmdl); tempIndx++ ) {
    //  userCmdl[tempIndx] = data.charAt(tempIndx) ;
    //  if (data.charAt(tempIndx) == '\0') {break;}
    //}  
  }  
  // //test 
  userRctl |= RCTL_ATTN; 
  //if ( 0 ) {
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print("dataCbck-userCmdl - ");
    Serial.println(userCmdl);
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
    //Serial.print  ("wrapPubl popcMqtt.publish topic : ");
    //Serial.println(tTops);
    //Serial.print  (" tVals : ");
    //Serial.print  (tVals);
    //Serial.print  (" RC : ");
    //Serial.println(rCode);
  //}
  //delay(100);
}    

//pupSub / ESP ticker callbacks and service 
void cbck1000() {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.println("cbck1000 sets cb10Rctl ATTN");
  //}  
  cb10Rctl |= RCTL_ATTN;
}

//
void cb10Svce() {
  int rCode = 0;
  //
  wrapPubl( (const char * )inf0Tops, (const char * )(bbrdLin0), sizeof(bbrdLin0) ); 
  wrapPubl( (const char * )inf1Tops, (const char * )(bbrdLin1), sizeof(bbrdLin1) ); 
  //
  dtostrf( adc0Curr, 8, 3, mqttVals);
  wrapPubl( (const char * )adc0Tops , (const char * )mqttVals, sizeof(mqttVals) ); 
  wrapPubl( (const char * )ArspTops , (const char * )artiProg, sizeof(artiProg) ); 
  //
  //if ( rCode) {
    //Serial.print("cbck1000 bad      RC: ");
    //Serial.println(rCode);
  //}
  //
  cb10Rctl &= ~RCTL_ATTN;
}

void cbck2000() {
  if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("cbck200 sets cb10Rctl ATTN");
  }  
  //
  cb20Rctl |= RCTL_ATTN;
}

void cb20Svce() {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    //Serial.println("cb20Svce");
  int rCode = 0;
  // Artisan interface                      'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(ambiTmpC), 5, 1, mqttVals);
  } else {
    dtostrf(           ambiTmpC , 5, 1, mqttVals);
	}		
  wrapPubl( (const char * )AmbiTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(targTmpC), 5, 1, mqttVals);
  } else {
    dtostrf(           targTmpC , 5, 1, mqttVals);
	}		
  wrapPubl( (const char * )ETmpTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(sensTmpC), 5, 1, mqttVals);
  } else {
    dtostrf(           sensTmpC , 5, 1, mqttVals);
	}		
  wrapPubl( (const char * )BTmpTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(holdTmpC), 5, 1, mqttVals);
  } else {
    dtostrf(           holdTmpC , 5, 1, mqttVals);
	}		
  wrapPubl( (const char * )ptmpTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  wrapPubl( (const char * )PTmpTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  if ( userScal == fahrScal) {
    dtostrf(  sensCdpm  * 9.0/5.0 , 5, 1, mqttVals);
  } else {
    dtostrf(             sensCdpm , 5, 1, mqttVals);
  }  
  wrapPubl( (const char * )dgpmTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  dtostrf(             pwmdPcnt , 5, 1, mqttVals);
  wrapPubl( (const char * )dutyTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  wrapPubl( (const char * )PwmdTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  dtostrf( stepSecs, 8, 3, mqttVals);
  wrapPubl( psecTops, (const char * )(mqttVals), sizeof(mqttVals) );
  //
  dtostrf( pidcRn, 8, 3, mqttVals);
  wrapPubl( (const char * )RnTops , (const char * )mqttVals, sizeof(mqttVals) ); 
  //
  dtostrf( pidcYn, 8, 3, mqttVals);
  wrapPubl( (const char * )YnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcEn, 8, 3, mqttVals);
  wrapPubl( (const char * )EnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcUn, 8, 3, mqttVals);
  wrapPubl( (const char * )UnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  cb20Rctl &= ~RCTL_ATTN;
}

void cbck9000() {
  if ( 0 ) {
  // if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("cbck9000 sets cb90Rctl ATTN");
  }  
  cb90Rctl |= RCTL_ATTN;
}

void cb90Svce() {
	//if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  	//Serial.println("cb90Svce");
  int rCode = 0;
  dtostrf( pidcElap, 12, 3, mqttVals);
  wrapPubl( c900Tops, (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcKp, 8, 3, mqttVals);
  wrapPubl( KpTops,   (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcTi, 8, 3, mqttVals);
  wrapPubl( TiTops,   (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcTd, 8, 3, mqttVals);
  wrapPubl( TdTops,   (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcBeta, 8, 3, mqttVals);
  wrapPubl( BetaTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcGamma, 8, 3, mqttVals);
  wrapPubl( GammaTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  dtostrf( pidcKappa, 8, 3, mqttVals);
  wrapPubl( KappaTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(holdTmpC), 8, 3, mqttVals);
  } else {
    dtostrf(           holdTmpC,  8, 3, mqttVals);
  }  
  wrapPubl( (const char * )ptmpTops, (const char *)(mqttVals), sizeof(mqttVals) ); 
  //
  //if ( rCode) {
    //Serial.print("cb90Svce bad cuml RC: ");
    //Serial.println(rCode);
  //}
  //if ( rCode) {
    //Serial.print("cbck9000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  cb90Rctl &= ~RCTL_ATTN;
}  
  
//
void wrapSubs( const char * tTops ) {
  int rCode = 999;
#if PROC_ESP  
  rCode = 998;
#if WIFI_MQTT  
  rCode = popcMqtt.subscribe( (const char * )tTops );
#endif
#endif
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print  ("wrapSubs popcMqtt.subscribe topic : ");
    Serial.println(tTops);
    Serial.print  (" RC : ");
    Serial.println(rCode);
  }
  //delay(100);
}    

//
void popcSubs() {
  int rCode = 0;
  wrapSubs( KpTops );
  wrapSubs( TiTops );
  wrapSubs( TdTops );
  wrapSubs( BetaTops );
  wrapSubs( GammaTops );
  wrapSubs( userTops );
}  

/// Off-On SSR driver mill 
//
// Wrap writes to OFFN_PIN with TWIO Out 
void offnDrve ( byte tPin, byte tVal) {
  digitalWrite( tPin, tVal);
#if WITH_PCF8574
  twioWritePin( OFFN_TWPO, tVal);
#endif  
}  

void millInit() {
  millStep = 0;
  adc0Curr = adc0Prev = adc0Maxi = adc0Mini = 0;
  pinMode( OFFN_OPIN, OUTPUT);
  pinMode( SCOP_OPIN, OUTPUT);
  scopLo;
  millMark = micros() + MILL_POLL_USEC;
  adc0Poll = ADC0_POLL_MILL;
  adc0Mark = ADC0_POLL_MILL;
}  

void millLoop() {
  // UNO nonExec: 8uS at 28uSec with 602/882uSec idle (pcf/not) exec: 400u/96Sec(pcf/not)
  // ESP 
  if ( micros() <= millMark ) {
    return;
  } else {
    millStep += 1;
    millMark +=  MILL_POLL_USEC;
    // mill        
    // max mill rate 5uSec per on-off
    if ( !(millStep % 1000) )  {
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print  ("adc0:");  Serial.print  (adc0Curr);
        Serial.print  (" Maxi:"); Serial.print  (adc0Maxi);
        Serial.print  (" Mini:"); Serial.print  (adc0Mini);
        Serial.print  (" Avge:"); Serial.println(adc0Avge);
      }
    }  
    if ( ( pwmdPcnt * 8 )  > ( millStep % 833 ) ) {
      //use Outp as trig to avoid repeated i/o traffic, set on: offn mark time 
      if (offnOutp == 0) {
        offnOutp = 1;
        offnRctl |=  RCTL_ATTN;  // for virt tcpl
        digitalWrite( ONBD_OPIN, (1 & ~ONBD_LOWON));
        if ( offnRctl & RCTL_AUTO) {
          offnDrve ( OFFN_OPIN,   1);
        }  
      }  
    } else {
      if (offnOutp == 1) {
      //use Outp as trig to avoid repeated i/o traffic, set off: offn space time 
        offnOutp = 0;
        offnRctl &= ~RCTL_ATTN;  // for virt tcpl
        digitalWrite( ONBD_OPIN, (0 | ONBD_LOWON));
        if ( offnRctl & RCTL_AUTO) {
          offnDrve ( OFFN_OPIN,   0);
        }
      }    
    }  
    // flicker tell tale LED in case of fast PWM
    if ( (offnRctl & RCTL_AUTO) && (millStep & 64)) {
      digitalWrite( ONBD_OPIN, (0 | ONBD_LOWON));
    }
    // AD
#if PROC_ESP
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
#endif //PROC_ESP
  }
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

// Retrieve updated parms from eprom 
void pidcFprm() {
  EEPROM.get( EADX_KP, fromEprm);
  if ( pidcKp != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.print(F("EEPROM new Kp:"));
      Serial.println(fromEprm);
    }
    pidcKp = fromEprm; 
  }  
  //
  EEPROM.get( EADX_TI, fromEprm);
  if ( pidcTi != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.print(F("EEPROM new Ti:"));
      Serial.println(fromEprm);
    }
    pidcTi =  fromEprm; 
  }  
  //
  EEPROM.get( EADX_TD, fromEprm);
  if ( pidcTd != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.print(F("EEPROM new Td:"));
      Serial.println(fromEprm);
    }
    pidcTd = fromEprm; 
  }  
  //
  EEPROM.get( EADX_BE, fromEprm);
  if ( pidcBeta != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.print(F("EEPROM new Be:"));
      Serial.println(fromEprm);
    }
    pidcBeta = fromEprm; 
  }  
  //
  EEPROM.get( EADX_GA, fromEprm);
  if ( pidcGamma != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("EEPROM new Ga:"));
      Serial.println(fromEprm);
    }
    pidcGamma = fromEprm; 
  }  
  //
  EEPROM.get( EADX_KA, fromEprm);
  if ( pidcKappa != fromEprm) {
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("EEPROM new Ka:"));
      Serial.println(fromEprm);
    }
    pidcKappa = fromEprm; 
  }  
}
void pidcInit() {
  float fromEprm;
  Tf = pidcAlpha * pidcTd;
  Epn1 = 0.0;
  Edfn2 = Edfn1 = Edfn = 0;
  // get 
  pidcFprm();
  // first time being enabled, seed with current property tree value
  Un1 = Un = 0;
  pidcMark =  millis() + PIDC_POLL_MSEC;
  targTmpC = int(ambiTmpC);
}

//
void pidcInfo() {
  Serial.print(F("PIDC   Kp:"));
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
  Serial.println(pidcKappa);
}  
//
void pidcLoop() {
  if ( millis() < pidcMark ) return; else {
    pidcMark += PIDC_POLL_MSEC;  
    //
    if ( (pidcRctl & RCTL_RUNS)  == 0 ) {
      // Poll/Thermocouple == 0 Shutdown
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.println(F("pidc stop"));
      }  
      Un = 0;
    } else {
      // 
      Ts = (PIDC_POLL_MSEC / 1000.0); // sample interval (Sec)
      // 
      pidcRn = (float)targTmpC;
      pidcYn = (float)sensTmpC;
      // P term 
      pidcEn  = pidcRn - pidcYn;
      Epn = pidcBeta * pidcRn - pidcYn;
      // D term 
      if ( pidcTd <= 0 ) {
        Edfn2 = Edfn1 = Edfn = 0;
      } else {
        Edn = pidcGamma * pidcRn - pidcYn;
        // Filter the derivate error:
        Tf = pidcAlpha * pidcTd;
        TsDivTf = Ts / Tf;
        Edfn = (Edfn1 / (TsDivTf + 1.0)) +  (Edn * ((TsDivTf) / (TsDivTf + 1.0)));
      }
      // Accum Combine P, I, D terms 
      dUn = 0;
      // P term 
      // try temp compensated gain  pidcPn = pidcKp *  (Epn - Epn1);
      pidcPn = pidcKc *  (Epn - Epn1);
      pidcPc += pidcPn;
      // I term 
      if ( pidcTi > 0.0 ) {
        //Jn10 KtKp pidcIn = pidcKp * ((Ts / pidcTi) * pidcEn);
        pidcIn = ((Ts / pidcTi) * pidcEn);
      } else {
        pidcIn = 0;
      }    
      pidcIc += pidcIn;
      // D term
      if ( pidcTd > 0.0 ) {
        //Jn10 KtKp pidcDn = pidcKp * ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
        pidcDn = ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
      } else {
        pidcDn = 0;
      } 
      pidcDc += pidcDn; 
      dUn    = pidcPn + pidcIn + pidcDn;
      // Integrator anti-windup logic:
      if ( dUn > (pidcUMax - Un1) ) {
        dUn = pidcUMax - Un1;
        if (pidcRctl & RCTL_DIAG ) {
          if ( !( bbrdRctl & RCTL_ARTI )  ) {
            Serial.println(F("maxSatn"));
          }  
        }  
      } else if ( dUn < (pidcUMin - Un1) ) {
        dUn = pidcUMin - Un1;
        if (pidcRctl & RCTL_DIAG ) {
          if ( !( bbrdRctl & RCTL_ARTI )  ) {
            Serial.println(F("minSatn"));
          }  
        }  
      }
      Un = Un1 + dUn;
      pidcUn = Un;
      // Updates indexed values;
      Un1   = Un;
      Epn1  = Epn;
      Edfn2 = Edfn1;
      Edfn1 = Edfn;
    }
  }    
}

void pidcDbug() {
  //
  dbugLine[6]   = 'r';
  dtostrf( (pidcRn  ), 6, 2, &dbugLine[7] );
  dbugLine[13]  = ' ';
  //
  dbugLine[14]  = 'y';
  dtostrf( (pidcYn  ), 6, 2, &dbugLine[15] );
  dbugLine[21]  = ' ';
  //
  dbugLine[22]  = 'e';
  dtostrf( (pidcEn  ), 6, 2, &dbugLine[23] );
  dbugLine[29]  = ' ';
  //
  dbugLine[30]  = 'o';
  dtostrf( (pidcUn  ), 6, 2, &dbugLine[31] );
  dbugLine[37]  = ' ';
  //
  dbugLine[38]  = 'K';
  dtostrf( (pidcKp  ), 6, 2, &dbugLine[39] );
  dbugLine[45]  = ' ';
  //
  dbugLine[46]  = 'S';
  dtostrf( (pidcTi  ), 6, 2, &dbugLine[47] );
  dbugLine[53]  = ' ';
  //
  dbugLine[54]  = 'V';
  dtostrf( (pidcTd  ), 6, 2, &dbugLine[55] );
  dbugLine[61]  = ' ';
  //
  #if 0
  dbugLine[62]  = 'P';
  dtostrf( (pidcPc  ), 6, 2, &dbugLine[63] );
  dbugLine[69]  = ' ';
  //
  dbugLine[70]  = 'I';
  dtostrf( (pidcIc  ), 6, 2, &dbugLine[71] );
  dbugLine[77]  = ' ';
  //
  dbugLine[78]  = 'D';
  dtostrf( (pidcDc  ), 6, 2, &dbugLine[79] );
  dbugLine[85]  = ' ';
  #endif
  //
  for ( tempIndx = 0; tempIndx < 64; tempIndx++ ) {
    if (( bbrdRctl & RCTL_ARTI ) == 0) {
      Serial.write(dbugLine[tempIndx]);
    }  
  } 
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
  prevTmpC = sensTmpC = holdTmpC = int(ambiTmpC);
  stepSecs = totlSecs = 0;
  profMark =  millis() + PROF_POLL_MSEC;
}

void profLoop() {
  if ( millis() < profMark) return; else { 
    profMark += PROF_POLL_MSEC;  
    // apply PID gain compensation if non-zero
    if ( pidcKappa > 0 ) {
      pidcKc = pidcKp * ( 1 + pidcKappa * ( sensTmpC - idleTmpC ) / idleTmpC );
    } else {
      pidcKc = pidcKp;
    }
    // prevent lcd rollover; 6000 secs == 100 min 
    (totlSecs > 5998) ? (totlSecs = 0):(totlSecs += 1);
    (stepSecs > 5998) ? (stepSecs = 0):(stepSecs += 1);
    // Handle stored profile 
    if (profNmbr <= 0) {
      // Stop auto profile and revert to manual control 
      profRctl = RCTL_MANU;
      stepRctl, rampRctl, holdRctl = 0x00;
    }
    else {
      // Active saved profile:  is active
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
      bbrdTmde = bbrdSetp;
    } else {
      // Profile or User ramp to  setpoint temp and then hold
      // User Ramp forces max/min setpoint temp: after 'r' cmd use 's' cmd to set Hold temp 
      if (stepSecs > 10 )  {
        if (holdTmpC >= baseTmpC ) {
          // Rising  ramp finished when sensed is above hold Temp 
          if (sensTmpC >= holdTmpC) bbrdTmde = bbrdHold;
        } else {
          // Falling ramp finished when sensed is below hold Temp 
          if (sensTmpC <  holdTmpC) bbrdTmde = bbrdHold;
        }
      }
      if ( bbrdTmde == bbrdHold ) {
        // do not really want to have s mode 
        rampCdpm = 0;
        targTmpC = holdTmpC;
        // wiprampFini = 1;
      } else {
        bbrdTmde = bbrdRamp;
        targTmpC = float(baseTmpC) \
                 + float (stepSecs) * float(rampCdpm) / 60.0;
      }            
    }
    // Run exp mavg 5 second apart temp change 
    // ROC degrees per min is 60 * avg per second change 
    if ( totlSecs % 2 ) {
      // time dist abot six seconds so ten samples each eand 
      sensCdpm = ( (   2 * int(sensTmpC) + 1 * degCHist[0] + 1 * degCHist[1] + 1 * degCHist[2] )\
                    -( 2 * degCHist[7]   + 1 * degCHist[6] + 1 * degCHist[5] + 1 * degCHist[4] ) ) ; 
      degCHist[7] = degCHist[6] ;
      degCHist[6] = degCHist[5] ;
      degCHist[5] = degCHist[4] ;
      degCHist[4] = degCHist[3] ;
      degCHist[3] = degCHist[2] ;
      degCHist[2] = degCHist[1] ;
      degCHist[1] = degCHist[0] ;
      degCHist[0] = int(sensTmpC);
      prevTmpC    = sensTmpC;
      //  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        //Serial.print("Curr:");
        //Serial.print(sensTmpC);
        //Serial.print(" Prev:");
        //Serial.print(prevTmpC);
        //Serial.print(" Cdpm:");
        //Serial.println(sensCdpm);
      //  }  
    }  
    // update billboard
    bbrdFill();
    //
    if (( bbrdRctl & RCTL_ARTI ) == 0) {
      if ( bbrdRctl & RCTL_INFO ) {
//
        // Send billboard 'Info' on serial 
        for ( tempIndx = 0; tempIndx < 16; tempIndx++ ) {
          Serial.write(bbrdLin0[tempIndx]);
        }  
        Serial.write(" <=> ");
        for ( tempIndx = 0; tempIndx < 16; tempIndx++ ) {
          Serial.write(bbrdLin1[tempIndx]);
        }
        //if ((bbrdRctl & RCTL_DIAG) == RCTL_DIAG) {  
          //pidcDbug();
        //}  
        Serial.println(" ");
        // Rotswitch 
        //Serial.print("Rots: ");
        //Serial.print(rotsValu());
        //Serial.print("    ");
        //Serial.print("tcplRctl: ");
        //Serial.print(tcplRctl);
        //Serial.print("    ");
        // Front End
      } else {
        // If bbrd flagged send Artisan csv logging serial 
        if ( bbrdRctl & RCTL_ATTN ) {
          for ( tempIndx = 0; tempIndx < sizeof(artiResp) - 1; tempIndx++ ) {
            Serial.write(artiResp[tempIndx]);
          }  
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
//TCCR0B = TCCR0B & B11111000 | B00000001     tmr 0 divisor:     1 for PWM freq 62500.00 Hz
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
// tbd exponent for ESP
#if PROC_ESP
  //analogWriteFrequency(32);
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
  //tbd   
  analogWriteFreq(newFreq);
  pwmdFreq = int(newFreq);
#else 
  pwmdFreq = PWMD_FREQ;
#endif 
}

void pwmdInit() {
  // PROC either define pins, exponent
  pwmdDuty = 0;
  pinMode( PWMD_OPIN, PWMD_MODE);
#if PROC_ESP
  analogWriteRange(255);
#endif  
  pwmdSetF( PWMD_FREQ);			
  pwmdMark =  millis() + PWMD_POLL_MSEC;
}

void pwmdLoop() {
  if ( millis() < pwmdMark ) { 
    return;
  } else { 
    pwmdMark += PWMD_POLL_MSEC;  
    //
    if ( (pwmdRctl & RCTL_RUNS) == 0) {
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("pwmdRctl==0");
      }  
      pwmdOutp = 0;
    } else {
      // last run control test has precedence 
      if ( pwmdRctl & RCTL_MANU) {
        pwmdTarg = byte( 255.0 * userDuty / 100.0 );
        //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          //Serial.print("pwmdTarg: ");
          //Serial.println(pwmdTarg);
        //}  
      }
      if ( pwmdRctl & RCTL_AUTO) {
        pwmdTarg = byte(pidcUn);
      }
      pwmdOutp = byte (pwmdTarg + 0.5);
      pwmdPcnt = byte ((100.0 * pwmdOutp / 255) +0.5);
      analogWrite( PWMD_OPIN, pwmdOutp);
    }  
  }  
}

/// Rotary 16Way Enc Switch 
//
int rotsValu() {
  byte resp, tVal;
  resp = 0;
#if PROC_UNO
  if ( digitalRead(ROTS_BIT3) == LOW  ) resp  = 8; 
  if ( digitalRead(ROTS_BIT2) == LOW  ) resp += 4; 
  if ( digitalRead(ROTS_BIT1) == LOW  ) resp += 2; 
  if ( digitalRead(ROTS_BIT0) == LOW  ) resp += 1; 
  //Serial.print("B3:");
  //Serial.print(digitalRead(ROTS_BIT3));
  //Serial.print(" B2:");
  //Serial.print(digitalRead(ROTS_BIT2));
  //Serial.print(" B1:");
  //Serial.print(digitalRead(ROTS_BIT1));
  //Serial.print(" B0:");
  //Serial.print(digitalRead(ROTS_BIT0));
#endif
#if WITH_PCF8574
  tVal = (~(twioRead8()) & TWIO_IMSK) ; 
  resp  = tVal & 0x01;
  if ( tVal & 0x02) resp += 2;
  if ( tVal & 0x04) resp += 4;
  if ( tVal & 0x08) resp += 8;
#endif
  //Serial.print("TWval:");
  //Serial.println(tVal);
  return(resp);
}
    
void rotsInit() {
  // Set pins to weak pullup 
#if PROC_UNO
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
          Serial.print("Rots:");
          Serial.print(rotsCurr);
        }  
        // manage rotary switch settings
        switch( rotsCurr) {
          case 0:
            stepNmbr = 0;
            userCmdl = "W0";
          break;
          case 1:
            stepNmbr = 1;
            userCmdl = "R5";
          break;
          case 2:
            stepNmbr = 2;
            userCmdl = "R10";
          break;
          case 3:
            stepNmbr = 3;
            userCmdl = "R15";
          break;
          case 4:
            stepNmbr = 4;
            userCmdl = "R20";
          break;
          case 5:
            stepNmbr = 5;
            userCmdl = "R30";
          break;
          case 6:
            stepNmbr = 6;
            userCmdl = "R45";
          break;
          case 7:
            stepNmbr = 7;
            userCmdl = "R0";
          break;
          case 8:
            stepNmbr = 8;
            userCmdl = "W100";
          break;
          case 9:
            stepNmbr = 9;
            userCmdl = "W95";
          break;
          case 10:
            stepNmbr = 10;
            userCmdl = "W88";
          break;
          case 11:
            stepNmbr = 11;
            userCmdl = "W80";
          break;
          case 12:
            stepNmbr = 12;
            userCmdl = "W70";
          break;
          case 13:
            stepNmbr = 13;
            userCmdl = "W60";
          break;
          case 14:
            stepNmbr = 14;
            userCmdl = "W50";
          break;
          case 15:
            stepNmbr = 15;
            userCmdl = "W40";
          break;
        }  
        userRctl |= RCTL_ATTN; 
      }
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
  Serial.println("tcpl.begin");
#endif
#endif
  tcplMark = millis() + TCPL_POLL_MSEC;
  vtcpMark = millis() + VTCP_POLL_MSEC;
  sensTmpC = ambiTmpC;
}

#if WITH_MAX31855
void tcplRealLoop() {
  double tcplTmpC;
  byte tResp = 0;
  if ( millis() < tcplMark) return; else {
    tcplMark += TCPL_POLL_MSEC;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      sensTmpC = ambiTmpC;
    } else {
      // Read thermocouple 
#if PROC_UNO
      tcplTmpC = tcpl.readCelsius();
#endif
#if PROC_ESP
      // read() gets both status and temp 
      tResp    = tcpl.read();
      if ( 0 ) {
      // block tcpl diags if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("tcpl Stat:");
        Serial.print(tcpl.getStatus());
        Serial.print(" Fctr:");
        Serial.print(tcpl.getTCfactor());
        Serial.print(" Ofst:");
        Serial.print(tcpl.getOffset());
        Serial.print(" ITmp:");
        Serial.print(tcpl.getInternal());
        Serial.print(" TdegC:");
        Serial.println(tcpl.getTemperature());
      }  
      // getTemperature returns internal variable from read()
      tcplTmpC = tcpl.getTemperature();
#endif
      // Error  condition ESP:tResp <> 0   UNO: isNan Temperature  
#if PROC_UNO
      if (isnan(tcplTmpC)) {
        tResp = tcpl.readError();
      }
#endif       
      if ( tResp != 0 ) {
#if WITH_LCD
        lcd.clear();
        lcd.home ();
        lcd.print(F("thermoCouple : "));
        lcd.setCursor ( 0, 1 );
#endif
       if ( 0 ) {
          // block tcpl diags if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print(" Sense: ");
        }  
        switch ( tResp ) {
          case 0:
#if WITH_LCD
            lcd.println("STATUS_OK      ");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println("OK");
            }
          break;  
          case 1:
#if WITH_LCD
            lcd.println("Error - Open-Cct");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
                Serial.println("Open-Cct");
            }
          break;  
          case 2:
#if WITH_LCD
            lcd.println("Error - Shrt-Gnd");
#endif
            if ( 0  & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("Shrt-Gnd");
            }
          break;  
          case 4:
#if WITH_LCD
            lcd.println("Error - Shrt-Vcc");
#endif
            if (0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("Shrt-Vcc");
            }
          break;  
          default:
#if WITH_LCD
            lcd.println("Error - NReadEtc");
#endif
            if ( 0 & !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("FailCase-NOREAD");
            }
          break;  
        }
#if WITH_LCD
        delay (500);
        lcd.clear();
#endif
      } else {
        // Only update sensed temp with a valid reading 
        //sensTmpC = float( tcplTmpC); 
      }  
      // get temp even if error 
      sensTmpC = float( tcplTmpC); 
    }  
  }  
}  
#endif // WITH_MAX31855

//
void virtTcplLoop() {
  int pwmdMavg;
  float heatInpu = 0; 
  if ( millis() < vtcpMark ) return; else {
    vtcpMark += VTCP_POLL_MSEC;
    // virt tcpl 
    if ( offnRctl & RCTL_AUTO) {
      if (offnRctl & RCTL_ATTN ) {
        heatInpu = 255;     
      } else {
        heatInpu = 0;
      }    
    } else {
      heatInpu = pwmdOutp;
    }
// Moving average stores heat input over time periods
    pwmdMavg = int( 0.01 * heatInpu     \
                 +  0.04 * heatHist[0]  \
                 +  0.08 * heatHist[1]  \
                 +  0.10 * heatHist[2]  \
                 +  0.10 * heatHist[3]  \
                 +  0.10 * heatHist[4]  \
                 +  0.10 * heatHist[5]  \
                 +  0.14 * heatHist[6]  \
                 +  0.18 * heatHist[7]  \
                 +  0.22 * heatHist[8]  \
                 +  0.22 * heatHist[9]  \
                 +  0.22 * heatHist[10] \
                 +  0.16 * heatHist[11] \
                 +  0.12 * heatHist[12] \
                 +  0.08 * heatHist[13] \
                 +  0.02 * heatHist[14] \
                 +  0.01 * heatHist[15] );
    sensTmpC = sensTmpC + float(pwmdMavg) / 255.0 \
                 -  (sensTmpC - ambiTmpC) / 100.0;
//                 
    heatHist[15] = heatHist[14]; 
    heatHist[14] = heatHist[13]; 
    heatHist[13] = heatHist[12]; 
    heatHist[12] = heatHist[11]; 
    heatHist[11] = heatHist[10]; 
    heatHist[10] = heatHist[9]; 
    heatHist[9]  = heatHist[8]; 
    heatHist[8]  = heatHist[7]; 
    heatHist[7]  = heatHist[6]; 
    heatHist[6]  = heatHist[5]; 
    heatHist[5]  = heatHist[4]; 
    heatHist[4]  = heatHist[3]; 
    heatHist[3]  = heatHist[2]; 
    heatHist[2]  = heatHist[1]; 
    heatHist[1]  = heatHist[0]; 
    heatHist[0]  = heatInpu;
  }
}


//
// PCF8574 I2C IO 
#if WITH_PCF8574
void twioInit() {
  twio.begin();
}

byte twioRead8() {
  twioWrite8( 0xFF & TWIO_IMSK );
  twioCurr = twio.read8();
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.print("twioRead8:");
  //  Serial.println(twioCurr);
  //}  
  return twioCurr;
}

void twioWrite8( byte tByt ) {
  twio.write8( tByt);
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.print("twioWrite8: ");
  //  Serial.println(tByt);
  //}  
}

void twioWritePin( byte tPin, byte tByt) {
  twio.write( tPin, tByt);
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  //  Serial.print("twioWritePin: ");
  //  Serial.println(tByt);
  //}  
}
#endif // WITH_PCF8574

/// User Interface 
//
void userInit() {
  profNmbr = stepNmbr = 0;
  rampCdpm =  0;
  userScal = centScal;
  baseTmpC = holdTmpC = userDegs = int(ambiTmpC);
  //userCmdl = String("W0");
  userDuty = 28;
  //userRctl |= RCTL_ATTN;
}

void userLoop() {
  // test for when chars arriving on serial port, set ATTN
  if (Serial.available()) {
    // wait for entire message  .. 115200cps 14 char ~ 1mSec
    delay(100);
    // read all the available characters
    // Dc14 
    userCmdl = Serial.readStringUntil('\n');
    //fromSeri = Serial.readStringUntil('\n');
    //for ( tempIndx = 0; tempIndx < sizeof(userCmdl); tempIndx++ ) {
    //  userCmdl[tempIndx] = fromSeri.charAt(tempIndx) ;
    //  if (fromSeri.charAt(tempIndx) == '\0') {break;}
    //}  
    //userCmdl[sizeof(userCmdl)] = '\0';
    //Serial.println("loop");
    //Serial.println(userCmdl);
    userRctl |= RCTL_ATTN;
  }
}    

void userSvce() {
  // called from loop() if (userRctl & RCTL_ATTN) via MQTT, rotsLoop or Serial
  if ( (!( bbrdRctl & RCTL_ARTI )) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print ("userSvce usderCmdl : ");
    Serial.println(userCmdl);
  }  
  // Drop Attn flag immediately so not masked during process
  userRctl &= ~RCTL_ATTN;
#if WIFI_MQTT
  // echo back network originated commands 
  wrapPubl( echoTops, userCmdl.c_str(), sizeof(userCmdl)); 
#endif  
  if ( bbrdRctl & RCTL_ARTI ) {
    // Artisan CHAN command 
    if ((userCmdl[0] == 'C') && (userCmdl[1] == 'H') && (userCmdl[2] == 'A')) {
      //  'chan' command, respond '#'
      Serial.println("#");
    }  
    if ((userCmdl[0] == 'I') && (userCmdl[1] == 'O') && (userCmdl[2] == '3')) {
      // Cmd : IO3 
      userAIO3 = (userCmdl.substring(4)).toInt();
      if (userAIO3 > 99) userAIO3 = 100;
      //
      dtostrf( userDuty, 8, 3, mqttVals);
      wrapPubl( (const char * )AIO3Tops , (const char * )mqttVals, sizeof(mqttVals) ); 
    }
    if ((userCmdl[0] == '0') && (userCmdl[1] == 'T') && (userCmdl[2] == '1')) {
      // Cmd : IO3 
      userAOT1 = (userCmdl.substring(4)).toInt();
      if (userAOT1 > 99) userAOT1 = 100;
      userDuty = userAOT1;
      //
      dtostrf( userAOT1, 8, 3, mqttVals);
      wrapPubl( (const char * )AOT1Tops , (const char * )mqttVals, sizeof(mqttVals) ); 
    }
    if ((userCmdl[0] == '0') && (userCmdl[1] == 'T') && (userCmdl[2] == '2')) {
      // Cmd : IO3 
      userAOT2 = (userCmdl.substring(4)).toInt();
      if (userAOT2 > 99) userAOT2 = 100;
            //
      dtostrf( userAOT2, 8, 3, mqttVals);
      wrapPubl( (const char * )AOT2Tops , (const char * )mqttVals, sizeof(mqttVals) ); 

    }
    if ((userCmdl[0] == 'R') && (userCmdl[1] == 'E') && (userCmdl[2] == 'A')) {
      //
      //bbrdArti();
      for ( tempIndx = 0; tempIndx < sizeof(artiResp) - 1; tempIndx++ ) {
        Serial.print(artiResp[tempIndx]);
      }  
      Serial.println("");
      //Serial.println(artiResp);
    }
    //  'unit' command, set user scale 
    if ((userCmdl[0] == 'U') && (userCmdl[1] == 'N') && (userCmdl[5] == 'C')) {
      userScal = centScal;
    }  
    if ((userCmdl[0] == 'U') && (userCmdl[1] == 'N') && (userCmdl[5] == 'F')) {
      userScal = fahrScal;
    }
    if ((userCmdl[0] == 'P') && (userCmdl[1] == 'I') && (userCmdl[2] == 'D')) {
      if ((userCmdl[4] == 'S') && (userCmdl[5] == 'V')) {
        // set desired temperatre degC
        userDegs = (userCmdl.substring(7)).toInt();
        if ( userScal == fahrScal) {
          targTmpC = intgFtoC( userDegs); 
        } else {
          targTmpC = userDegs;
        }
        if (targTmpC > maxiTmpC) targTmpC = maxiTmpC;
        holdTmpC = targTmpC;
        rampCdpm = userDgpm = 0;          // Setting target temp implies no ramp 
        stepSecs = 0;                     // User command: reset step timer 
        pwmdRctl &= ~RCTL_MANU;
        pwmdRctl |=  RCTL_AUTO;
      }  
    }
  }  
  //  a/A Toggle Artisan format serial interface
  if ((userCmdl[0] == 'A') || (userCmdl[0] == 'a')) {
    // Toggle Artisan Interface
    if ( bbrdRctl & RCTL_ARTI ) {
      bbrdRctl &= ~RCTL_ARTI; 
      Serial.println("# Serial  <=> Console");
    } else {
      bbrdRctl |=  RCTL_ARTI;
      Serial.println("# Serial  <=> Artisan");
    }
  }
  // B  put pidc Beta term 
  if (userCmdl[0] == 'B') {
    pidcBeta = (userCmdl.substring(1)).toFloat();
    pidcInfo();
  }
  //  c/C set Centigrade units 
  if (((userCmdl[0] == 'C') || (userCmdl[0] == 'c')) && (userCmdl[1] != 'H')) {
    userScal = centScal;
  }
  // d/D Toggle Diagnostic Flag
  if ((userCmdl[0] == 'D') || (userCmdl[0] == 'd')) {
    if ( bbrdRctl & RCTL_DIAG ) {
      Serial.println("# Diagnostics Mode  is InActive");
      bbrdRctl &= ~RCTL_DIAG; 
    } else {
      bbrdRctl |=  RCTL_DIAG; 
      Serial.println("# Diagnostics Mode  is   Active");
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
    eprmInfo();  
  }
  // f/F Set fahrenheit units 
  if (((userCmdl[0] == 'F') || (userCmdl[0] == 'f')) && (userCmdl[1] != 'I')) {
    userScal = fahrScal;
  }
  // G  put pid Gamma term 
  if (userCmdl[0] == 'G') {
    pidcGamma = (userCmdl.substring(1)).toFloat();
    pidcInfo();
  }
  if ((userCmdl[0] == 'H') || (userCmdl[0] == 'h')) {
    // set desired hold temperatre deg
    userDegs = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      holdTmpC = intgFtoC( userDegs); 
    } else {
      holdTmpC = userDegs;
    }
    if (holdTmpC > maxiTmpC) holdTmpC = maxiTmpC;
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
  // K put pid Kappa term 
  if (userCmdl[0] == 'K') {
    pidcKappa = (userCmdl.substring(1)).toFloat();
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
      Serial.println(csvlLin1); 
      Serial.println(csvlLin2); 
      // Switch On  Artisan csv Logging. TotalTime, StepTime must start at 0. 
      bbrdRctl &= ~RCTL_INFO; 
      stepSecs = totlSecs = 0;
    } else {
      bbrdRctl |= RCTL_INFO; 
    }  
  }
  if ((userCmdl[0] == 'm') || (userCmdl[0] == 'M')) {
    // TBD stored profile from memory 
    profNmbr = (userCmdl.substring(1)).toInt();
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      //Serial.println(F("userfSele"));
    }  
    stepSecs = 0;
  }
  // P  put pid Kp term 
  if ((userCmdl[0] == 'p') || (userCmdl[0] == 'P')) {
    pidcKp = (userCmdl.substring(1)).toFloat();
    pidcInfo();
  }
  // o Readback PID operating parameters
  if (userCmdl[0] == 'q') {
    pidcInfo();
  }  
  if (((userCmdl[0] == 'R') || (userCmdl[0] == 'r')) && (userCmdl[1] != 'E')) {
    // Keep user entry for billboard; convert, set profile temp ramp rate degC/min
    userDgpm = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      rampCdpm = int ( float(userDgpm) * 5.00 / 9.00);
    } else {
      rampCdpm = userDgpm;
    }  
    baseTmpC = int(sensTmpC);
    if (userDgpm > 0) holdTmpC = maxiTmpC;
    if (userDgpm < 0) holdTmpC = int(ambiTmpC);
    if (userDgpm == 0) {
      // Selected ROC 0: Hold current temp 
      bbrdTmde = bbrdHold;
      targTmpC = int(sensTmpC);
    } else {
      // On R0 don't reset step timer 
      stepSecs = 0;
    }
    // seting ramp unsets manual PWM width 
    pwmdRctl &= ~RCTL_MANU;
    pwmdRctl |=  RCTL_AUTO;
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("userCmdl r sets rampCdpm to: "));
      Serial.println(rampCdpm);
    }  
  }
  if ((userCmdl[0] == 'S') || (userCmdl[0] == 's')) {
    // set desired setpoint / immed target temperatre deg
    userDegs = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      targTmpC = intgFtoC( userDegs); 
    } else {
      targTmpC = userDegs;
    }
    if (targTmpC > maxiTmpC) targTmpC = maxiTmpC;
    rampCdpm = userDgpm = 0;          // Setting target temp implies no ramp 
    stepSecs = 0;                     // User command: reset step timer 
    pwmdRctl &= ~RCTL_MANU;
    pwmdRctl |=  RCTL_AUTO;
  }
  if ((userCmdl[0] == 'U')                        ) {
    // set new PWM frequency 
    pwmdFreq = (userCmdl.substring(1)).toInt();
    pwmdSetF( pwmdFreq);
  }
  if ((userCmdl[0] == 'U') || (userCmdl[0] == 'u')) {
    if  (!( bbrdRctl & RCTL_ARTI )) {
      Serial.print("pwmdFreq: ");
      Serial.println(pwmdFreq);
    }  
  }
  if ((userCmdl[0] == 'V') || (userCmdl[0] == 'v')) {
    // Version string e if Artisan is setting Unit C/F 
    if  (!( bbrdRctl & RCTL_ARTI )) {
      Serial.println(versChrs);
      eprmInfo();
      pidcInfo();
    }  
  }
  if ((userCmdl[0] == 'W') || (userCmdl[0] == 'w')) {
    // set new pwmD Width, run control flag to indicate manual override
    userDuty = (userCmdl.substring(1, 5)).toInt();
    if (userDuty > 99) userDuty = 100;
    if ((bbrdRctl & RCTL_DIAG) == RCTL_DIAG) {  
      Serial.print("Manu userDuty: ");
      Serial.println(userDuty);
    }  
    if ( userDuty == 0) {
      // Power off: Sense ambient ( fan htr pwr), temp setpt to meas ambient
      targTmpC = sensTmpC;
    } else {
      stepSecs = 0;                   // User command: reset step timer 
    }
    bbrdTmde = bbrdManu;
    pwmdRctl &= ~RCTL_AUTO;
    pwmdRctl |=  RCTL_MANU;
    // manual pwm will apply in pwmd loop; unset manual ramp ctrl 
    rampCdpm = 0;
  }
  if ((userCmdl[0] == 'Z') || (userCmdl[0] == 'z')) {
    // Zero 'Total Time' 
    totlSecs = 0;
    stepSecs = 0;                   // User command: reset step timer 
  }
  // For debug to see if Artisan is setting Unit C/F 
  if ((userCmdl[0] == '?')) {
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println(userScal);
    }  
  }
//  clean out cmdLine 
  for ( tempIndx = 0; tempIndx < sizeof(userCmdl); tempIndx++ ) {
    userCmdl[tempIndx] = ' ';
  }  
  //userCmdl = "";
}

/// Arduino Setup 
//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  eprmInit();
  rotsInit();
  userInit();
  tcplInit();
  pidcInit();
  pwmdInit();
  millInit();
  profInit();
#if WITH_PCF8574
  twioInit();
#endif
#if PROC_ESP
  if (wifiRctl & RCTL_RUNS) {
    if ( (bbrdRctl & RCTL_ARTI) == 0) {
      Serial.println();
      Serial.print(F("PROC_ESP : setup Init wifi to upward SSID:"));
      Serial.println(upwdSsid);
    } 
#if WIFI_WMAN
    //Jn01 
    //WiFiManager wifiManager; //   Also in the setup function add
    //set custom ip for portal
    ////wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
    //first parameter is name of access point, second is the password
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WIFI_WMAN : init call popcMqtt.autoConnect");
    }  
    wifiManager.autoConnect(dnwdSsid, dnwdPwrd);
#endif
#if WIFI_SOKS
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("WIFI_SOKS WebS Init"));
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
    delay(1000);
    }
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WMul : add upbd AP  (local IP: ) ");
      Serial.println(WiFi.localIP());
    }  
    //WiFiMulti.addAP("SSID", "passpasspass");
    WiFiMulti.addAP(upwdSsid, upwdPwrd);
    //
    while(WiFiMulti.run() != WL_CONNECTED) {
      if ( (bbrdRctl & RCTL_ARTI) == 0) {
        Serial.print("~");
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
#else
    //  Wifi Setup 
    WiFi.begin(upwdSsid, upwdPwrd);
    while (WiFi.status() != WL_CONNECTED) {
      if ( (bbrdRctl & RCTL_ARTI) == 0) {
        Serial.print("!");
      }  
      delay(1000);
    }  
    if ( !( bbrdRctl & RCTL_ARTI ))  {
      Serial.println("WiFi connected as local IP:");
      Serial.println(WiFi.localIP());
    }  
#endif
#if WIFI_MQTT
    //  MQTT Setup 
    //    setup callbacks
    popcMqtt.onConnected(connCbck);
    popcMqtt.onDisconnected(discCbck);
    popcMqtt.onPublished(publCbck);
    popcMqtt.onData(dataCbck);
    if ( !( bbrdRctl & RCTL_ARTI)) {
      Serial.println("WIFI_MQTT : call popcMgtt.Connect()");
    }  
    popcMqtt.connect();
    delay(18000);
    if ( !( bbrdRctl & RCTL_ARTI ) ) {
      Serial.println("Wifi Mqtt.connect timed out. calling popcSubs()");
    }  
    //
    popcSubs();
#endif
  }  
  //    TickerScheduler(uint size);
  //    boolean add(uint i, uint32_t period, tscallback_t f, boolean shouldFireNow = false);
  //      ts.add(0, 3000, sendData)
  int shedRcod;
  shedRcod = popcShed.add( 0, 1000, cbck1000);
  shedRcod = popcShed.add( 1, 2000, cbck2000);
  shedRcod = popcShed.add( 2, 9000, cbck9000);
#else 
#endif // PROC_ESP
//
#if WITH_LCD
  lcdsInit();
#endif
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("popcEsp init() ends");
  } 
}  

/// Arduino Loop 
//
void loop() {
  // put your main code here, to run repeatedly:
  millLoop();
  pidcLoop();
  pwmdLoop();
  profLoop();
  rotsLoop();
#if WITH_MAX31855
  tcplRealLoop();
#else  
  virtTcplLoop();
#endif
#if WITH_LCD
  lcdsLoop();
#endif 
  userLoop();
  //
  if (cb10Rctl & RCTL_ATTN) {
    cb10Svce();
  }  
  if (cb20Rctl & RCTL_ATTN) {
    cb20Svce();
  }  
  if (cb90Rctl & RCTL_ATTN) {
    cb90Svce();
  }  
  if (userRctl & RCTL_ATTN) {
    userSvce();
  }  
#if PROC_ESP
  popcShed.update();
#if WIFI_SOKS
  webSocket.loop(); 
#endif  
#endif  
  //frntLoop();
  // Why 
  delay(10);
}
