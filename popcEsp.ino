/// roboPopc Arduino UNO / ESP8266 Controller / Artisan Logger with MQTT client 'popc'
// 
//  Sections (units) in this code, ordered alphabetically:
//  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
//  eprm  EEPROM setup and handling 
//  frnt  deprecated, sends data over Serial to Frontside / Process apps on PC for PID tuning   
//  lcds  support for I2C 2x16 LCD display                             
//  mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
//  offn  Slow Off/On approx 3sec cycle PWN for SSR controlled heater                      
//  pidc  PID controller for PWM powered temperature control; a delta-time incremental PID 
//  pwmd  8-bit PWM control via hardware pwm pins 
//  prof  Profile control; selects auto/manual temp setpt, manual pwm width, real/fake temp sensor
//  rots  Rotary 16way encoded ( 4pin + common) selector switch manager
//  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
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
//  b/Bff   Readback / Set PID Beta parameter (Float; Expert only ! )  
//  c/C     Set centigrade units; LCD display actual/target temp
//  d/D     toggle diagnostic verbose messages on serial 
//  e/E     Readback / Update EEPROM PID parameters 
//  f/F     Set fahrenheit units; LCD display actual/target temp
//  g/Gff   Readback / Set PID Gamma parameter (Float; Expert only ! )  
//  h/H     Set hold temperature setpoint fore ramp endpoint ( soak temp )
//  i/Iff   Set PID I-Term time (Float Ti: lower value == higher gain)
//  j/Jff   Set PID D-Term gain (Float Td: lower value == lower  gain)
//  k/Kff   Set PID Gain TComp  (Float Kappa: 0 == no Temp comp)
//  l/L     Send Artisan CSV format on serial ( for capture and Artisan Import )  
//  o       Readback PIDC operating (not eeprom) parameters 
//  p/Pff   Readback / Set PID P-Term gain (Float Kp: lower value == lower  gain)
//  mn/MN   Reserved for TBD Profile handling 
//  rnn/Rnn Set Temperature Ramp C/F Deg per min (Set before hold temp) 
//  snn/Snn Set immediate target setPoint C/F temperature  
//  v/V     Readback firmware version, PID, EEPROM parms to serial 
//  wnn/Wnn Set PWM Duty Cycle nn <= 100, disable PID
//  y/Y     Set PWM Frequency Hz (TBD)  
//  z/Z     Zero reset all timecounts
//
//  Copyright (c) 2017 Bitwise Technologies  popc@bitwisetech.com  
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

//  Code section compiler switches - Rebuild and Upload after changing these 
#define PROC_ESP      0                  // Compile for ESP8266
#define WIFI_WMAN     0                  // Compile for Wifi Manager
#define WIFI_MQTT     0                  // Compile for Wifi MQTT client
#define WIFI_SOKS     0                  // Compile for Wifi Web Sckt Srvr
#define PROC_UNO      1                  // Compile for Arduino Uno
#define WITH_LCD      1                  // Hdwre has I2C 2x16 LCD display
#define WITH_MAX31855 1                  // Hdwre has thermocouple + circuit
#define IFAC_ARTI     0                  // Start with Artisan interface on Serial
#define WITH_OFFN     0                  // Use ~4sec Off-On SSR, not fast PWM
#define IFAC_FRNT     0                  // Obsolete Front/Process interface on Serial 
 
#if 0
// milliSecond poll values
#define TCPL_POLL_MSEC  100UL            // mS termocouple poll
#define PWMD_POLL_MSEC  100UL            // mS pwm driver  poll
#define PIDC_POLL_MSEC  100UL            // mS pid control poll
#define VTCP_POLL_MSEC  250UL            // mS virt tcpl   poll
#define ROTS_POLL_MSEC  500UL            // mS rotary sw   poll
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define PROF_POLL_MSEC 1000UL            // mS run control poll
#define OFFN_POLL_MSEC   25UL            // mS run control poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec 
#endif 

// milliSecond poll values Primes to suppress beating 
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define OFFN_POLL_MSEC   23UL            // mS run control poll
#define PIDC_POLL_MSEC  101UL            // mS pid control poll
#define PROF_POLL_MSEC  997UL            // mS run control poll
#define PWMD_POLL_MSEC  103UL            // mS pwm driver  poll
#define ROTS_POLL_MSEC  503UL            // mS rotary sw   poll
#define TCPL_POLL_MSEC   97UL            // mS termocouple poll
#define VTCP_POLL_MSEC  251UL            // mS virt tcpl   poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec 
//
#define LCDS_TOGO (LCDS_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define OFFN_TOGO (OFFN_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define PIDC_TOGO (PIDC_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define PROF_TOGO (PROF_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define PWMD_TOGO (PWMD_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define ROTS_TOGO (ROTS_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define TCPL_TOGO (TCPL_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare
#define VTCP_TOGO (VTCP_POLL_MSEC - POLL_SLOP_MSEC)  // poll delay for compare

// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif
//
#include <Arduino.h>
#include <EEPROM.h>
// 
#if PROC_ESP
#include <Adafruit_ESP8266.h>
// Beg paste from pubsShed 
// popcShed ingo MQTT with tick, scheduler 
// 
#include <WiFiClient.h>
#include <ESP8266WiFiAP.h>
#include <WiFiUdp.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiMulti.h>
//Jn01 WifiManager 
#if WIFI_WMAN
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#endif
#if WIFI_SOKS
//  Jn02 arduWWebSockets
//Jn03 wsokSrvr
#include <WebSocketsServer.h>
#include <Hash.h>
#endif
//  Mqtt
#if WIFI_MQTT
#include <MQTT.h>
#define MQCL_ID "pipc"
#endif
//
// wifi Replace with your own network's SSID, Password
//const char* upwdSsid = "cherib";
//const char* upwdPwrd = "sUmMeRsEaT";
//const char* upwdSsid = "inactive";
//const char* upwdPwrd = "pickledcrab1102190";
//const char* upwdSsid = "bitwComc";
//const char* upwdPwrd = "tWiStEdTeA";
//
const char* upwdSsid = "bitwComw";
//
const char* upwdPwrd = "manchester1102190tan";
//
// wifiManager.autoConnect("upwdSsid", "password");
//
const char* dnwdSsid = "espipc8101";
//
const char* dnwdPwrd = "summerseat";
//
#if WIFI_MQTT
//  Mqtt
// create MQTT object with IP address, port of MQTT broker e.g.mosquitto application
// MQTT myMqtt(MQCL_ID, "test.mosquitto.org", 1883);
//
MQTT popcMqtt(MQCL_ID, "172.20.224.111", 5983);
//MQTT popcMqtt(MQCL_ID, "172.20.224.117", 5983);
#endif
//
#if WIFI_SOKS
//  WebSockets
ESP8266WiFiMulti WiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(5981);
#define USE_SERIAL Serial

#endif // WIFI_SOKS

// from another copy ??
#include <dummy.h>
#endif

/// Declarations by unit

// 40+ char Used for Artisan: ambient, ch1, ch2, ch3, ch4 or Logging Tt Ts BT ET SV Duty on serial 
char artiResp[] = "023.0,128.8,138.8,000.0,000.0          ";  // 39 + null

// billboard  Legend Lower cases: computed/Measured Upper case: User/Setpoints 
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
float   ambiTmpC  = 28;                    //  28C  82F rm temp then W0 + fan htr temp
#define idleTmpC    50                     //  50C temp with W0 due to fan heater coil  
#define maxiTmpC    248                    // 248C 480F as maximum temp 

// Theese two lines must contain tab chars, not spaces
const char csvlLin1[] = "Date:	Unit:C	CHARGE:	TP:	DRYe:	FCs:	FCe:	SCs:	SCe:	DROP:	COOL:	Time:";
const char csvlLin2[] = "Time1	Time2	BT	ET	Event	SV	DUTY";

String     userCmdl("exactly thirty one chars length");  // Crashable ! 
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

#include <SPI.h>
#include <Wire.h>  // Comes with Arduino IDE

// i-n-g-o MQTT 
// mqtt strings are declared for both ESP8266 and UNO 
char mqttVals[] =  "                ";                     // mqtt value 16sp 15ch max
// General Info topics 
const char c900Tops[]  = "/pipc/cbck9000";
const char echoTops[]  = "/pipc/echoCmdl";
const char inf0Tops[]  = "/pipc/bbrdLin0";
const char inf1Tops[]  = "/pipc/bbrdLin1";
const char psecTops[]  = "/pipc/stepSecs";
const char dutyTops[]  = "/pipc/pwmdPcnt";
const char ptmpTops[]  = "/pipc/profDegs";
const char dgpmTops[]  = "/pipc/sensDgpm";
const char userTops[]  = "/pipc/userCmdl";
// Artisan interface UC names  'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
const char AmbiTops[]  = "/pipc/arti/ATmp";
const char ArspTops[]  = "/pipc/arti/Read";
const char ETmpTops[]  = "/pipc/arti/ETmp";
const char BTmpTops[]  = "/pipc/arti/BTmp";
const char PTmpTops[]  = "/pipc/arti/PTmp";
const char PwmdTops[]  = "/pipc/arti/Pwmd";
const char AOT1Tops[]  = "/pipc/arti/AOT1";
const char AOT2Tops[]  = "/pipc/arti/AOT2";
const char AIO3Tops[]  = "/pipc/arti/AIO3";
// PID controller topics
const char RnTops[]    = "/pipc/pidc/Rn";
const char YnTops[]    = "/pipc/pidc/Yn";
const char EnTops[]    = "/pipc/pidc/En";
const char UnTops[]    = "/pipc/pidc/Un";
const char PnTops[]    = "/pipc/pidc/Pn";
const char InTops[]    = "/pipc/pidc/In";
const char DnTops[]    = "/pipc/pidc/Dn";
const char KpTops[]    = "/pipc/pidc/Kp";
const char TdTops[]    = "/pipc/pidc/Td";
const char TiTops[]    = "/pipc/pidc/Ti";
const char BetaTops[]  = "/pipc/pidc/Beta";
const char GammaTops[] = "/pipc/pidc/Gamma";
const char TlapTops[]  = "/pipc/pidc/LoopMillis";

//eprm
float fromEprm;
int eprmSize, eprmFree;
#define EADX_KP (eprmSize - 1 * (sizeof(float)))
#define EADX_TI (eprmSize - 2 * (sizeof(float)))
#define EADX_TD (eprmSize - 3 * (sizeof(float)))
#define EADX_BE (eprmSize - 4 * (sizeof(float)))
#define EADX_GA (eprmSize - 5 * (sizeof(float)))
#define EADX_KA (eprmSize - 6 * (sizeof(float)))

//pidc
#if WITH_OFFN
// Slow response PID to match 4sec cycle of SSR Off-On 
float pidcKp      =   6.000;              // P-Term gain
float pidcKc      =   6.000;              // P-Term gain
float pidcTi      =   1.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd      =   0.200;              // D-Term Gain sec ( Td++ = Gain++)
#else
//Ap15
// Fast response PID to match approx 30Hz PWM frequency 
//  Date  Kp      Ti      Td      Beta      Gamma 
//  Ap15  4.000   2.000   0.010
//        8.000   8.000   0.002
//        6.000  10.000   0.002
//       10.000  12.000   0.002       
//  Ap22  3.000   8.000   4.000
//  My01  1.500   6.000   0.600   2.000     0.400
// My4-1  2.000   6.000   1.000   4.000     0.200
// My4-2  1.000   4.000   1.000   2.000     0.400
// My4-3  1.000   5.000   2.000   2.000     0.400
// My4-4  0.800   4.000   2.000   2.000     0.400
// My07   1.000   4.000   1.000   2.000     0.400 
// My09   1.000   6.000   0.100   1.000     1.000 
// My11   1.200   5.000   0.100   1.000     1.000  Did not download 
// My11   2.000   5.000   0.050   1.000     1.000  Still lags / oshoots
// My24   3.000   6.000   0.750   1.000     1.000  PWM oscillates              
// My25   2.000   5.000   0.050   1.000     1.000  Still lags / oshoots
// Jn01   2.250   4.000   0.100   1.000     1.000  Migs-Furn High osht then drop 
// Jn01   1.800   5.000   0.050   1.000     1.000  Braz-Furn BSF Still osc
// Jn01   2.000   4.000   0.025   1.000     1.000  Jn02-Vuid
// Jn01   3.600  10.000   0.025   1.000     1.000  not installed           
// Jn02   3.000   8.000   0.025   1.000     1.000  Jn03-Furn Osc grows     
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
//
float pidcKp    =   2.000;              // P-Term gain
float pidcKc    =   2.000;              // P-Term gain
float pidcTi    =   2.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd    =   1.000;              // D-Term Gain sec ( Td++ = Gain++)
#endif
//
float pidcBeta    =   2.000;              // P-term Refr vs YInp
float pidcGamma   =   1.000;              // D-term Refr vs YInp
float pidcAlpha   =   0.100;              // D-term Filter time
float pidcKappa   =   0.250;              // Ambient comp Kp * ( 1 + Ka (sens - idle)/idle )
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
const char versChrs[] = "2017Au31 ";

// pwmd vbls
int  pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp;                          // Freq, Duty Cycle Target (255max) Output
int  heatHist[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // mavg 16  for virtTcpl 
int  degCHist[] = { 0, 0, 0, 0, 0, 0, 0, 0};                                // mavg store for temp rate of change 
byte pwmdPcnt;                                                        // Percent duty cycle 

// Run Cntl vbls  Bit 0:Run 1:Ctrl 2:Auto 3:Info  Info: 4: 5:Virt 6:Dbug 7:bbrd 
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
byte  profRctl  = RCTL_RUNS;
byte  rotsRctl  = RCTL_RUNS;
byte  tcplRctl  = RCTL_RUNS;
byte  userRctl  = RCTL_RUNS;
byte  wifiRctl  = RCTL_RUNS;

// scheduler tick callback attn flags 
byte  cb10Rctl  = RCTL_RUNS;
byte  cb20Rctl  = RCTL_RUNS;
byte  cb90Rctl  = RCTL_RUNS;

byte profNmbr, profStep, profChar, stepChar;    // Numeric Profile No, Step No, Character values 

//  Rotary Switch
byte rotsCurr, rotsNewb, offnCntr, offnOutp;   // Current value, newb test for change; off/on cycle counter, output 

//  time
unsigned int  mSecOflo;
unsigned long currMSec, elapMSec, pidcElap = 0UL;
unsigned long lcdsPrev, pidcPrev, profPrev = 0UL;  // mSec poll loop timer counters
unsigned long pwmdPrev, rotsPrev, tcplPrev = 0UL;  // mSec poll loop timer counters
unsigned long vtcpPrev, offnPrev           = 0UL;  // mSec poll loop timer counters
//

#if PROC_ESP
#include "TickerScheduler.h"
TickerScheduler popcShed(3);
// End paste from pubsShed 

//share RW pubsub + RO
void dataCbck(String& topic, String& data);
void publCbck();
void discCbck();
void connCbck();

#if WIFI_SOKS  // wsokSrvr
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[%u] popcSockSrvr Disconnected!\n", num);
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
      //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      if ( !( bbrdRctl & RCTL_ARTI ) ) {
        USE_SERIAL.printf("popcSockSrvr [%u] get Text: %s\n", num, payload);
      }  
      // send message to client
      // webSocket.sendTXT(num, "popcSockSrvr message here");

      // send data to all connected clients
      // webSocket.broadcastTXT("popcSockSrvr message here");
    break;
    case WStype_BIN:
    USE_SERIAL.printf("[%u] popcSockSrvr get binary length: %u\n", num, lenght);
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

// Todo maybe these pin assignments are only good for UNO 
#if PROC_ESP
// Off/On SSR driver GPIO 16 deepWake     
#define OFFN_OPIN 16
// Handle either LED polarity with: (1 & ~LED_LOWON) for light, (0 | LED_LOWOFF) for dark 
#define LED_ONBRD  0
#define LED_LOWON  1
// PWM Drive PWM SSR driver GPIO 2 BLed  
#define PWMD_OPIN  2
#define PWMD_MODE  OUTPUT
// ESP: No Rotary 16way encoder switch;
// tcpl
#define TCPL_CL   12  // SPI SCk
#define TCPL_MI   14  // SPI Mstr In Slve Out 
#define TCPL_CS   15  // SPI ChipSel 10K Gnd 
#endif   // PROC_ESP

//
#if PROC_UNO
// Off/On SSR driver 
#define OFFN_OPIN  5
//
#define LED_ONBRD LED_BUILTIN
#define LED_LOWON  0
// PWM Drive
// d9 needed by RFI scan  d6 would use tmr0 want d3 used by max13855
#define PWMD_OPIN  9                        // Pin D9
#define PWMD_MODE  OUTPUT
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3  6                        // Pin D6  Val 8 
#define ROTS_BIT2 10                        // Pin D10 Val 4 
#define ROTS_BIT1 11                        // Pin D11 Val 2 
#define ROTS_BIT0 12                        // Pin D12 Val 1 
// tcpl
#define TCPL_CL    4                        // Pin D4 Clock
#define TCPL_MI    7                        // Pin D7 Data
#define TCPL_CS    8                        // Pin D8 CSel
#endif   // PROC_UNO
 
#if WITH_MAX31855
#if PROC_UNO
#include "Adafruit_MAX31855.h"
Adafruit_MAX31855 tcpl(TCPL_CL, TCPL_CS, TCPL_MI);
#endif
#if PROC_ESP
#include "MAX31855.h"
MAX31855 tcpl(TCPL_CL, TCPL_CS, TCPL_MI);
#endif  //  ESP
#endif  //  MAX31855

//
float         targTmpC, sensTmpC, prevTmpC;   // target, sensor, previous temperature deg C
int           userDuty, userDgpm, sensCdpm;   // userSet duty cycle; userSet C/F, meas C dg pm
int           userAOT1, userAOT2, userAIO3;   // Arti OT1Dty, Arti OT2Dty, Arti IO3Dty 
int           profTbeg;                       // profile start time
int           profTmpC, profCdpm, userDegs;   // profile final temp, rate, user C/F tempr 
int           stepSecs, totlSecs, tempIndx;   // profile step elap; total run time; temporary array indexer

/// Common 
//    Convert
float floatCtoF( float celsInp) {
  return (float( ((celsInp + 40.0) * 9.0 / 5.0 )  -40.0 ));
}  

//
float floatFtoC( float fahrInp) {
  return (float( ((fahrInp + 40.0) * 5.0 / 9.0 )  -40.0 ));
}  
  
// integer fns add 0.5 for rounding
int    intgCtoF( int   celsInp) {
  return (int  ( ((celsInp + 40.0) * 9.0 / 5.0 )  -39.5 ));
} 
 
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
  
//  Timing 
unsigned long mSecPast( unsigned long mSecLast) {
  // last millis() remains in global currMSec
  currMSec =  millis();
  // correct result with uns long even after oflo
  return (currMSec - mSecLast);
}

/// Billboard LCDisplay and Serial info Lines 
void  bbrdArti() {
  //// Artisan Iface : resp 'READ' = Amb,Ch1,2,3,4 Amb,ET,BT,PT,PW
  //Je 15 to get SV/Duty into Config-Device-Extra TC4Ch3-4  send Amb ET, BT SV, D% 
  // Je15 Artisan Iface : resp 'READ' = Amb,Ch1,2,3,4 Ta,Te,Tb,Te,Du
  if ( userScal == fahrScal) {                                
    dtostrf( floatCtoF(ambiTmpC), 5, 1, &artiResp[0]  );         // Art's Ta Chn
    dtostrf( floatCtoF(targTmpC), 5, 1, &artiResp[6]  );         //   Te 
    dtostrf( floatCtoF(sensTmpC), 5, 1, &artiResp[12] );         //   Tb
    //dtostrf( floatCtoF(targTmpC), 5, 1, &artiResp[18] );         //  Sv
    dtostrf( int(sensCdpm * 9.00 / 5.00 + 50 ), 5, 1, &artiResp[18] ); //   SV  
  } else {
    dtostrf(           ambiTmpC,  5, 1, &artiResp[0]  );         //  Ta
    dtostrf(           targTmpC,  5, 1, &artiResp[6]  );         //  Te
    dtostrf(           sensTmpC,  5, 1, &artiResp[12] );         //  Tb
    //dtostrf(           targTmpC,  5, 1, &artiResp[18] );         //  SV
    dtostrf( int(sensCdpm + 50   ), 5, 1, &artiResp[18] );         //  SV
  } 
  dtostrf(           pwmdPcnt,  5, 1, &artiResp[24] );             // Dy
  artiResp[5]  = ',';
  artiResp[11] = ',';
  artiResp[17] = ',';
  artiResp[23] = ',';
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
      dtostrf( floatCtoF(profTmpC),                 3, 0, &bbrdLin1[12] );
    } else {
      dtostrf(           profTmpC,                  3, 0, &bbrdLin1[12] );
    }  
    bbrdLin1[15]  = userScal;                  //  upCase
  }  
  // fill line[1] legend 
  bbrdLin1[0]   = 'P';
  bbrdLin1[1]   = nibl2Hex( profNmbr);
  bbrdLin1[2]   = '-';
  bbrdLin1[3]   = nibl2Hex(profStep);
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
  if ( bbrdRctl & RCTL_ARTI ) {
    bbrdArti();
  } else {
    // ~ARTI
    //if ( !( bbrdRctl & RCTL_INFO ) && !(totlSecs % 2)) {
    if ( bbrdRctl & RCTL_INFO ) {
      // ~ARTI &  INFO 
    } else {
      // ~ARTI & ~INFO : csv logging for Artisan import Select 1s / Ns : TotMin:Sec StepMin:sec BT ET Event SV Duty
      dtostrf( (totlSecs / 60), 02, -0, &artiResp[0]);
      dtostrf( (totlSecs % 60), 02, -0, &artiResp[3]);
      dtostrf( (stepSecs / 60), 02, -0, &artiResp[6]);
      dtostrf( (stepSecs % 60), 02, -0, &artiResp[9]);
      if ( userScal == fahrScal) {
        dtostrf( floatCtoF(sensTmpC),       5, 1, &artiResp[12] );
        dtostrf( floatCtoF(targTmpC),       5, 1, &artiResp[18] );
        //Je18 dtostrf( floatCtoF(profTmpC),5, 1, &artiResp[25] );
        dtostrf( floatCtoF(sensCdpm * 9.0/5.0  + 50),  5, 1, &artiResp[25] ); // Arti scale offset
      } else {
        dtostrf(           sensTmpC,        5, 1, &artiResp[12] );
        dtostrf(           targTmpC,        5, 1, &artiResp[18] );
        //Je18 instead dtostrf(           profTmpC,        5, 1, &artiResp[25] );
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
  eprmSize = EEPROM.length();
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
      //Serial.print(index);
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
  lcdsPrev  =  millis();
}

void lcdsLoop() {
  currMSec = millis();
  if ( LCDS_TOGO > (currMSec - lcdsPrev)) {
    return;
  } else {
    lcdsPrev = currMSec;
    //
    if (lcdstRctl == 0) {
      // Rctl == 0 Shutdown
      lcd.home ();
      lcd.print(F("<= lcdsLoop() =>"));
      lcd.setCursor ( 0, 1 );
      lcd.print(F(" Poll 0 Halted  "));
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
    delay(4000);
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WIFI_MQTT timeout : wait cbck cted & call popcMqtt.Subs");
    }  
    // Je18 popcSubs();
  #endif  
}

void publCbck() {
  if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("popc publCbck");
  }  
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
    //strncpy(userChrs, data, sizeof(userChrs));
    // //test 
    userRctl |= RCTL_ATTN; 
    //if ( 0 ) {
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print("dataCbck-userCmdl - ");
      Serial.println(userCmdl);
    }
  }  
}

//
void wrapPubl( const char * tTops , const char * tVals, int tInt ) {
  int rCode = 999;
#if PROC_ESP  
  rCode = 998;
#if WIFI_MQTT  
  rCode = popcMqtt.publish( (const char * )tTops , (const char * )tVals, tInt );
#endif
#endif
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  if ( 0 ) {
    Serial.print  ("wrapPubl popcMqtt.publish topic : ");
    Serial.println(tTops);
    Serial.print  (" tVals : ");
    Serial.print  (tVals);
    Serial.print  (" RC : ");
    Serial.println(rCode);
  }
  //delay(100);
}    

void cbck1000() {
  if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("cbck1000 sets cb10Rctl ATTN");
  }  
  cb10Rctl |= RCTL_ATTN;
}

//
void cb10Svce() {
  if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    //Serial.println("cb10Svce - popcMqtt.publish..");
  }  
  int rCode = 0;
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
  dtostrf( pidcPn, 8, 3, mqttVals);
  wrapPubl( (const char * )PnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
  //
  wrapPubl( (const char * )inf0Tops, (const char * )(bbrdLin0), sizeof(bbrdLin0) ); 
  wrapPubl( (const char * )inf1Tops, (const char * )(bbrdLin1), sizeof(bbrdLin1) ); 
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
    Serial.println("cbck2000 sets cb20Rctl ATTN");
  }  
  cb20Rctl |= RCTL_ATTN;
}

void cb20Svce() {
  if ( 0 ) {
  //if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("cb20Svce");
  }  
  int rCode = 0;
  // Artisan interface                                        'Read' Cmd; Send Ambient:Targ:Sens:Prof:Duty
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
    dtostrf( floatCtoF(profTmpC), 5, 1, mqttVals);
  } else {
    dtostrf(           profTmpC , 5, 1, mqttVals);
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
  wrapPubl( (const char * )ArspTops , (const char * )artiResp, sizeof(artiResp) ); 
  //
  dtostrf( stepSecs, 8, 3, mqttVals);
  wrapPubl( psecTops, (const char * )(mqttVals), sizeof(mqttVals) );
  //
  dtostrf( pidcElap, 9, 3, mqttVals);
  //dtostrf( (100*(Epn-Epn1)), 8, 3, mqttVals);
  wrapPubl( TlapTops, (const char * )(mqttVals), sizeof(mqttVals) );
  //
  dtostrf( pidcUn, 8, 3, mqttVals);
  wrapPubl( (const char * )UnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
    //
  if (0){
    dtostrf( pidcDn, 8, 3, mqttVals);
    wrapPubl( (const char * )DnTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
    //
    dtostrf( pidcIn, 8, 3, mqttVals);
    wrapPubl( (const char * )InTops, (const char * )(mqttVals), sizeof(mqttVals) ); 
    //
    dtostrf( stepSecs, 8, 3, mqttVals);
    wrapPubl( psecTops, (const char * )(mqttVals), sizeof(mqttVals) );
    //
  }  
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
  if ( 0 ) {
	//if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  	Serial.println("cb90Svce");
	}	
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
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(profTmpC), 8, 3, mqttVals);
  } else {
    dtostrf(           profTmpC,  8, 3, mqttVals);
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
  //wrapSubs( TlapTops );
  wrapSubs( userTops );
}  

/// Off-On SSR driver 
//
void offnInit() {
  offnPrev = millis();
  offnCntr = 0;
  pinMode( LED_ONBRD, OUTPUT);
  pinMode( OFFN_OPIN,   OUTPUT);
}  

void offnLoop() {
  currMSec = millis();
  if ( OFFN_TOGO > ( currMSec - offnPrev )) {
    return;
  } else {
    offnPrev = currMSec;
    //
    if (!( offnRctl & RCTL_RUNS )) {
      return; 
    } else {
      // RCTL_RUNS runs Off/On Loop for LED indicator, RCTL_AUTO drives output pin
      if (offnCntr >= 100 ) {
        offnCntr = 0;
      } else {
        offnCntr += 1;
      }  
      switch ( pwmdPcnt) {
        case 0:
          //Serial.println("offnlLoop Case 0 ");
          offnOutp = 0;
          offnRctl &= ~RCTL_ATTN;
          digitalWrite( LED_ONBRD, (0 | LED_LOWON));
          if ( offnRctl & RCTL_AUTO) digitalWrite( OFFN_OPIN,   0);
        break; 
        case 100: 
          //Serial.println("offnlLoop Case 100 ");
          offnOutp = 1;
          offnRctl |=  RCTL_ATTN;
          digitalWrite( LED_ONBRD, (1 & ~LED_LOWON));
          if ( offnRctl & RCTL_AUTO) digitalWrite( OFFN_OPIN,   1);
        break; 
        default: 
          if (offnCntr > pwmdPcnt ) {
            //Serial.print("offnlLoop Case dflt is On Cntr:");
            //Serial.print(offnCntr);
            //Serial.print(" pwmdPcnt:");
            //Serial.print(pwmdPcnt);
            //Serial.println(" ");
            //output is on, switch it off, set space time 
            offnOutp = 0;
            offnRctl &= ~RCTL_ATTN;
            digitalWrite( LED_ONBRD, (0 | LED_LOWON));
            if ( offnRctl & RCTL_AUTO) digitalWrite( OFFN_OPIN,   0);
          } else {
            //Serial.print("offnlLoop Case dflt is Off Cntr:");
            //Serial.print(offnCntr);
            //Serial.print(" pwmdPcnt:");
            //Serial.print(pwmdPcnt);
            //Serial.println(" ");
          //output is off, switch it on,  set space time 
            offnOutp = 1;
            offnRctl |=  RCTL_ATTN;
            digitalWrite( LED_ONBRD, (1 & ~LED_LOWON));
            if ( offnRctl & RCTL_AUTO) digitalWrite( OFFN_OPIN,   1);
        }
        break;  
      }
      // flicker tell tale LED in case of fast PWM
      if ( (!(offnRctl & RCTL_AUTO)) && !(offnCntr % 4)) {
        digitalWrite( LED_ONBRD, (0 | LED_LOWON));
      }
    }    
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
//
void pidcFrom() {
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
    pidcTi = fromEprm; 
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
  pidcFrom();
  // first time being enabled, seed with current property tree value
  Un1 = Un = 0;
  pidcPrev =  millis();
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
  currMSec = millis();
  pidcElap = currMSec - pidcPrev;
  if ( PIDC_TOGO > pidcElap ) {
    return;
  } else {
    // save this currMSec as time of service
    pidcPrev = currMSec;
    // Adjusting with elapMSec errors after init when 8secs elapse  time togo  
    //pidcTogo = 2 * PIDC_POLL_MSEC - pidcElap;
    //Serial.print("pidc ");
    //Serial.println( pidcElap);
    //
    if ( (pidcRctl & RCTL_RUNS)  == 0 ) {
      // Poll/Thermocouple == 0 Shutdown
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.println(F("pidc stop"));
      }  
      Un = 0;
    } else {
      // 
      //Ts = (PIDC_POLL_MSEC - (float)pidcTogo) / 1000000.0; // sample interval (Sec)
      Ts = (float)pidcElap / 1000.0;
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
  dbugLine[62]  = 'p';
  dtostrf( (pidcPc  ), 6, 2, &dbugLine[63] );
  dbugLine[69]  = ' ';
  //
  dbugLine[70]  = 'i';
  dtostrf( (pidcIc  ), 6, 2, &dbugLine[71] );
  dbugLine[77]  = ' ';
  //
  dbugLine[78]  = 'd';
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

/// Profile Control
void profInit() {
  // simulation
  prevTmpC = sensTmpC = profTmpC = int(ambiTmpC);
  stepSecs = totlSecs = 0;
  profPrev =  millis();
}

void profLoop() {
  currMSec = millis();
  if ( PROF_TOGO >  (currMSec - profPrev)) {
    return;
  } else {
    profPrev = currMSec;
    // apply PID gain compensation if non-zero
    if ( pidcKappa > 0 ) {
      pidcKc = pidcKp * ( 1 + pidcKappa * ( sensTmpC - idleTmpC ) / idleTmpC );
    } else {
      pidcKc = pidcKp;
    }
    // update billboard
    bbrdFill();
    // prevent lcd rollover; 6000 secs == 100 min 
    (totlSecs > 5998) ? (totlSecs = 0):(totlSecs += 1);
    (stepSecs > 5998) ? (stepSecs = 0):(stepSecs += 1);
    if (profNmbr > 0) {
      // ToDo saved profiles
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        //Serial.println("profNmbr");
      }  
    } else {
      // Default: No ramp, Do Not target setpoint, let userCmds do that  
      // targTmpC = float(profTmpC);
      // Ramp to setpt temp and hold, Unless Manual Ramp 
      if (profCdpm != 0)  {
        if (profTmpC >= profTbeg)  {
          // User Ramp forces max temp, use Setpt after ramp setting for stop temp 
          if ((sensTmpC >=  profTmpC) && (stepSecs > 10) ) {
            profCdpm = 0;
            //Je 18 stepSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     + float (stepSecs) * float(profCdpm) / 60.0;
          }  
        } else {
          // User Ramp forces min temp, use Setpt after ramp setting for stop temp 
          //if ((sensTmpC <= targTmpC) && (stepSecs > 10) && (userDgpm == 0)) {
          if ((sensTmpC <= profTmpC) && (stepSecs > 10)) {
            profCdpm = 0;
            //Je18 stepSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     + float (stepSecs) * float(profCdpm) / 60.0;
          }  
        }
      } 
    }
    if ((profCdpm == 0) && (userDgpm == 0)) {
      // user calls for ramp==0 ( not holdPt): target temp  unchanged
      //profCdpm = 0;
      // StepSecs shows time in either Setpoint of Hold after ramp 
      //stepSecs = 0;
      bbrdTmde = bbrdSetp;
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
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        //Serial.print("Curr:");
        //Serial.print(sensTmpC);
        //Serial.print(" Prev:");
        //Serial.print(prevTmpC);
        //Serial.print(" Cdpm:");
        //Serial.println(sensCdpm);
      }  
    }  
    if (( bbrdRctl & RCTL_ARTI ) == 0) {
      if ( bbrdRctl & RCTL_INFO ) {
        // Send billboard 'Info' on serial 
        for ( tempIndx = 0; tempIndx < 16; tempIndx++ ) {
          Serial.write(bbrdLin0[tempIndx]);
        }  
        Serial.write(" <=> ");
        for ( tempIndx = 0; tempIndx < 16; tempIndx++ ) {
          Serial.write(bbrdLin1[tempIndx]);
        }
        if ((bbrdRctl & RCTL_DIAG) == RCTL_DIAG) {  
          pidcDbug();
        }  
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
        // If new 3 Sec artiResp is flagged send Artisan csv logging serial 
        if ( bbrdRctl & RCTL_ATTN) {
          for ( tempIndx = 0; tempIndx < sizeof(artiResp) - 1; tempIndx++ ) {
            Serial.write(artiResp[tempIndx]);
          }  
          Serial.println(" ");
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
#else 
#endif 
  pwmdFreq = int(newFreq);
}

void pwmdInit() {
  // PROC either define pins, exponent
  pwmdDuty = 0;
  pinMode( PWMD_OPIN, PWMD_MODE);
#if PROC_ESP
  analogWriteRange(255);
  //
  analogWriteFreq(32);
#endif  
#if PROC_UNO  
  pwmdExpo(10);
#endif  
  pwmdPrev  =  millis();            // pwm drvr poll period mSec
}

void pwmdLoop() {
  currMSec = millis();
  if ( PWMD_TOGO > (currMSec - pwmdPrev)) {
    return;
  } else {
    pwmdPrev = currMSec;
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
        if (0) {
          Serial.print("pwmdTarg: ");
          Serial.println(pwmdTarg);
        }  
      }
      if ( pwmdRctl & RCTL_AUTO) {
        pwmdTarg = byte(pidcUn);
      }
      pwmdOutp = byte (pwmdTarg + 0.5);
      pwmdPcnt = byte ((100.0 * pwmdOutp / 255) +0.5);
      if ( !(offnRctl & RCTL_AUTO)) {
        // Off/On not RCTL_AUTO means use fast analog PWM
        analogWrite( PWMD_OPIN, pwmdOutp);
      } 
    }  
  }  
}

/// Rotary 16Way Enc Switch 
//
int rotsValu() {
  int resp;
  //Serial.print("B3:");
  //Serial.print(digitalRead(ROTS_BIT3));
  //Serial.print(" B2:");
  //Serial.print(digitalRead(ROTS_BIT2));
  //Serial.print(" B1:");
  //Serial.print(digitalRead(ROTS_BIT1));
  //Serial.print(" B0:");
  //Serial.println(digitalRead(ROTS_BIT0));
  resp = 0;
#if PROC_UNO
  if ( digitalRead(ROTS_BIT3) == LOW  ) resp  = 8; 
  if ( digitalRead(ROTS_BIT2) == LOW  ) resp += 4; 
  if ( digitalRead(ROTS_BIT1) == LOW  ) resp += 2; 
  if ( digitalRead(ROTS_BIT0) == LOW  ) resp += 1; 
#endif
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
  rotsPrev =  millis();
}
    
void rotsLoop() {
  currMSec = millis();
  if ( ROTS_TOGO > (currMSec - rotsPrev)) {
    return;
  } else {
    rotsPrev = currMSec;
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
            profStep = 0;
            userCmdl = "W0";
            strstr(userChrs, "W0");
          break;
          case 1:
            profStep = 1;
            userCmdl = "R10";
            strstr(userChrs, "");
          break;
          case 2:
            profStep = 2;
            userCmdl = "R12";
          break;
          case 3:
            profStep = 3;
            userCmdl = "R15";
          break;
          case 4:
            profStep = 4;
            userCmdl = "R20";
          break;
          case 5:
            profStep = 5;
            userCmdl = "R25";
          break;
          case 6:
            profStep = 6;
            userCmdl = "R30";
          break;
          case 7:
            profStep = 7;
            userCmdl = "R40";
          break;
          case 8:
            profStep = 8;
            userCmdl = "R0";
          break;
          case 9:
            profStep = 9;
            userCmdl = "W100";
          break;
          case 10:
            profStep = 10;
            userCmdl = "W95";
          break;
          case 11:
            profStep = 11;
            userCmdl = "W90";
          break;
          case 12:
            profStep = 12;
            userCmdl = "W75";
          break;
          case 13:
            profStep = 13;
            userCmdl = "W60";
          break;
          case 14:
            profStep = 14;
            userCmdl = "W50";
          break;
          case 15:
            profStep = 15;
            userCmdl = "W40";
          break;
        }  
        userRctl |= RCTL_ATTN; 
      }
    }
  }
}    
    
/// THERMOCOUPLE
// 
void tcplInit() {
  // stabilize wait .. lcds banner is 1000mA anyway
  // delay(500);
#if WITH_MAX31855
#if PROC_ESP
  tcpl.begin();
  Serial.println("tcpl.begin");
#endif
#endif
  tcplPrev = vtcpPrev = millis();
  sensTmpC = ambiTmpC;
}

void virtTcplLoop() {
  int pwmdMavg;
  float heatInpu = 0; 
  currMSec = millis();
  if (( VTCP_TOGO ) > (currMSec - vtcpPrev)) {
    return;
  } else {
    vtcpPrev = currMSec;
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
#if 0 
  // Ap10 R20 on popc: virtTcpl showed 236C at real FC 10.7 mins,  260 @ 13.4 mins 
  // Arti shows Avge 195C at W70 
    pwmdMavg = int( 0.2 * heatInpu    \
                 +  0.8 * heatHist[0] \
                 +  2.0 * heatHist[1] \
                 +  4.0 * heatHist[2] \
                 +  2.0 * heatHist[3] ) ; 
    sensTmpC = sensTmpC + float(pwmdMavg) / 255.0 \
                 -  (sensTmpC - ambiTmpC) / 32.0;
//  #endif Ap16 try to get bumpy temp response 
// Want 195/236 power with slower decay 
    pwmdMavg = int( 0.1 * heatInpu    \
                 +  0.2 * heatHist[0] \
                 +  1.0 * heatHist[1] \
                 +  1.6 * heatHist[2] \
                 +  1.2 * heatHist[3] ) ; 
    sensTmpC = sensTmpC + float(pwmdMavg) / 255.0 \
                 -  (sensTmpC - ambiTmpC) / 72.0;
//                 
    heatHist[3] = heatHist[2] / 4; 
    heatHist[2] = heatHist[1]; topsTi
    heatHist[1] = heatHist[0]; 
    heatHist[0] = heatInpu;
  }  
#endif
// Ap16 0.200 total
    pwmdMavg = int( 0.00 * heatInpu     \
                 +  0.00 * heatHist[0]  \
                 +  0.01 * heatHist[1]  \
                 +  0.01 * heatHist[2]  \
                 +  0.01 * heatHist[3]  \
                 +  0.02 * heatHist[4]  \
                 +  0.05 * heatHist[5]  \
                 +  0.10 * heatHist[6]  \
                 +  0.10 * heatHist[7]  \
                 +  0.20 * heatHist[8]  \
                 +  0.50 * heatHist[9]  \
                 +  0.50 * heatHist[10] \
                 +  0.20 * heatHist[11] \
                 +  0.20 * heatHist[12] \
                 +  0.10 * heatHist[13] \
                 +  0.05 * heatHist[14] \
                 +  0.02 * heatHist[15] );
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

#if WITH_MAX31855
void tcplRealLoop() {
  double tcplTmpC;
  currMSec = millis();
  if ( TCPL_TOGO > (currMSec - tcplPrev)) {
    return;
  } else {
    tcplPrev = currMSec;
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
      tcplTmpC = tcpl.read();
#endif
      if (isnan(tcplTmpC)) {
#if WITH_LCD
        lcd.clear();
        lcd.home ();
        lcd.print(F("thermoCouple : "));
        lcd.setCursor ( 0, 1 );
#endif
        if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
          Serial.print("tcplRealLoop Sts_");
        }  
#if PROC_UNO
        switch (tcpl.readError()) {
          case 0: {
#if WITH_LCD
            lcd.println("STATUS_OK      ");
#endif
            if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("OK");
            }
          }
          break;  
          case 1: {
#if WITH_LCD
            lcd.println("Error - Open-Cct");
#endif
            if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("Open-Cct");
            }
          }
          break;  
         break;  
          case 2: {
#if WITH_LCD
            lcd.println("Error - Shrt-Gnd");
#endif
            if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("");
            }
          }
          break;  
          case 4: {
#if WITH_LCD
            lcd.println("Error - Shrt-Vcc");
#endif
            if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("");
            }
          }
          break;  
          default:  {
#if WITH_LCD
            lcd.println("Error - NReadEtc");
#endif
            if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
              Serial.println("FailCase-NOREAD");
            }
          }
        }
#endif  // UNO error hdlr
#if WITH_LCD
        delay (500);
        lcd.clear();
#endif
      } else {
        // Only update sensed temp with a valid reading 
        sensTmpC = float( tcplTmpC); 
      }  
    }  
  }  
}  
#endif // WITH_MAX31855

/// User Interface 
//
void userInit() {
  profNmbr = profStep = 0;
  profCdpm =  0;
  userScal = centScal;
  profTbeg = profTmpC = userDegs = int(ambiTmpC);
  userCmdl = String("W0");
  userRctl |= RCTL_ATTN;
}

void userLoop() {
  // test for when chars arriving on serial port, set ATTN
  if (Serial.available()) {
    // wait for entire message  .. 115200cps 14 char ~ 1mSec
    delay(100);
    // read all the available characters
    userCmdl = Serial.readStringUntil('\n');
    //Serial.println("loop");
    //Serial.println(userCmdl);
    userRctl |= RCTL_ATTN;
  }
}    

void userSvce() {
  // called from loop() if (userRctl & RCTL_ATTN) via MQTT, rotsLoop or Serial
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.print ("userSvce usderCmdl : ");
    Serial.println(userCmdl);
  }  
#if WIFI_MQTT
  // echo back network originated commands 
  popcMqtt.publish( echoTops, userCmdl, sizeof(userCmdl)); 
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
      dtostrf( userDuty, 8, 3, mqttVals);
      wrapPubl( (const char * )AOT2Tops , (const char * )mqttVals, sizeof(mqttVals) ); 

    }
    if ((userCmdl[0] == 'R') && (userCmdl[1] == 'E') && (userCmdl[2] == 'A')) {
      bbrdArti();
      Serial.println(artiResp);
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
        profCdpm = 0;                                     // Setting target temp implies no ramp 
        //profTbeg = int(sensTmpC);
        //stepSecs = 0;
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
  // b/B  put/get pidc Beta term 
  if (userCmdl[0] == 'B') {
    pidcBeta = (userCmdl.substring(1)).toFloat();
    //EEPROM.put( EADX_BE, pidcBeta);
  }
  if (userCmdl[0] == 'b') {
    Serial.print(F("Be: "));
    Serial.println(pidcBeta);
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
  }
  // f/F Set fahrenheit units 
  if (((userCmdl[0] == 'F') || (userCmdl[0] == 'f')) && (userCmdl[1] != 'I')) {
    userScal = fahrScal;
  }
  // G/g  put/get pid Gamma term 
  if (userCmdl[0] == 'G') {
    pidcGamma = (userCmdl.substring(1)).toFloat();
    //EEPROM.put( EADX_GA, pidcGamma);
  }
  if (userCmdl[0] == 'g') {
    Serial.print(F("Gamma: "));
    Serial.println(pidcGamma);
  }
  if ((userCmdl[0] == 'H') || (userCmdl[0] == 'h')) {
    // set desired hold temperatre deg
    userDegs = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      profTmpC = intgFtoC( userDegs); 
    } else {
      profTmpC = userDegs;
    }
    if (profTmpC > maxiTmpC) profTmpC = maxiTmpC;
    // profTbeg = int(sensTmpC);
    // stepSecs = 0;
    pwmdRctl &= ~RCTL_MANU;
    pwmdRctl |=  RCTL_AUTO;
  }
  // I/i  put/get pid Ti term 
  if ((userCmdl[0] == 'i') || (userCmdl[0] == 'I')) {
    pidcTi = (userCmdl.substring(1)).toFloat();
  }
  // j/J put pid Td term 
  if ((userCmdl[0] == 'j') || (userCmdl[0] == 'J')) {
    pidcTd = (userCmdl.substring(1)).toFloat();
  }
  // k/K put pid Kappa term 
  if ((userCmdl[0] == 'k') || (userCmdl[0] == 'K')) {
    pidcKappa = (userCmdl.substring(1)).toFloat();
  }
  if ((userCmdl[0] == 'm') || (userCmdl[0] == 'M')) {
    // TBD stored profile from memory 
    profNmbr = (userCmdl.substring(1)).toInt();
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      //Serial.println(F("userfSele"));
    }  
    stepSecs = 0;
  }
  // p/P  put pid Kp term 
  if ((userCmdl[0] == 'p') || (userCmdl[0] == 'P')) {
    pidcKp = (userCmdl.substring(1)).toFloat();
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
  // o/O Readback / Set from EEPROM  PID operating parameters
  if (userCmdl[0] == 'o') {
    pidcInfo();
  }  
  if (userCmdl[0] == 'O') {
    pidcFrom();
  }  
  if (((userCmdl[0] == 'R') || (userCmdl[0] == 'r')) && (userCmdl[1] != 'E')) {
    // Keep user entry for billboard; convert, set profile temp ramp rate degC/min
    userDgpm = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      profCdpm = int ( float(userDgpm) * 5.00 / 9.00);
    } else {
      profCdpm = userDgpm;
    }  
    profTbeg = int(sensTmpC);
    if (userDgpm > 0) profTmpC = maxiTmpC;
    if (userDgpm < 0) profTmpC = int(ambiTmpC);
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
      Serial.print(F("userCmdl r sets profCdpm to: "));
      Serial.println(profCdpm);
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
    profCdpm = userDgpm = 0;          // Setting target temp implies no ramp 
    stepSecs = 0;                     // User command: reset step timer 
    pwmdRctl &= ~RCTL_MANU;
    pwmdRctl |=  RCTL_AUTO;
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
    userDuty = (userCmdl.substring(1)).toInt();
    if (userDuty > 99) userDuty = 100;
    if ((bbrdRctl & RCTL_DIAG) == RCTL_DIAG) {  
      Serial.print("Manu userDuty: ");
      Serial.println(userDuty);
    }  
    if ( userDuty == 0) {
      // Power off: Sense ambient ( fan htr pwr), temp setpt to meas ambient
      //Je18 do not set ambient to some high sensed val 
      //ambiTmpC = sensTmpC;
      // profTmpC = int(ambiTmpC);
      targTmpC = sensTmpC;
    } else {
      stepSecs = 0;                   // User command: reset step timer 
    }
    bbrdTmde = bbrdManu;
    pwmdRctl &= ~RCTL_AUTO;
    pwmdRctl |=  RCTL_MANU;
    // manual pwm will apply in pwmd loop; unset manual ramp ctrl 
    profCdpm = 0;
  }
  if ((userCmdl[0] == 'Y') || (userCmdl[0] == 'y')) {
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      // set new PWM frequency 
      //Serial.println("Pwm frequency control TBD");
    }  
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
  userRctl &= ~RCTL_ATTN;
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
  offnInit();
  profInit();
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
    ////wifiManager.setAPStaticIPConfig(IPAddress(172,20,224,120), IPAddress(172,20,224,120), IPAddress(255,255,255,0));
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
    USE_SERIAL.setDebugOutput(true);
    USE_SERIAL.println();
    USE_SERIAL.println();
    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
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
      delay(4000);
   }
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
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
    if ( !( bbrdRctl & RCTL_ARTI) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WIFI_MQTT : call popcMgtt.Connect()");
    }  
    popcMqtt.connect();
    delay(4000);
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.println("WIFI_MQTT timeout : wait cbck cted & call popcMqtt.Subs");
    }  
    popcSubs();
#endif
  }  

  //  Tick setup 
  //  Sked setup
  //    TickerScheduler(uint size);
  //    boolean add(uint i, uint32_t period, tscallback_t f, boolean shouldFireNow = false);
  //      ts.add(0, 3000, sendData)
  // 
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
    Serial.println("popcEsp init end");
  } 
}  

/// Arduino Loop 
//
void loop() {
  // put your main code here, to run repeatedly:
  if (cb10Rctl & RCTL_ATTN) {
    cb10Svce();
  }  
  if (cb20Rctl & RCTL_ATTN) {
    cb20Svce();
  }  
  if (cb90Rctl & RCTL_ATTN) {
    cb90Svce();
  }  
#if WITH_MAX31855
  tcplRealLoop();
#else  
  virtTcplLoop();
#endif
  pidcLoop();
  pwmdLoop();
  offnLoop();
  rotsLoop();
  //
  if (userRctl & RCTL_ATTN) {
    userSvce();
  }  
  userLoop();
  if (userRctl & RCTL_ATTN) {
    userSvce();
  }  
  profLoop();
#if PROC_ESP
  popcShed.update();
#if WIFI_SOKS
  webSocket.loop(); 
#endif  
#endif  
#if WITH_LCD
  lcdsLoop();
#endif 
  //frntLoop();
  // Why delay(10);
}
