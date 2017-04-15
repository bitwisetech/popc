/// popcEsp popc port to esp with MQTT, tickScheduler, is MQTT client popc
//  Ap12 Fast 30Hz PWM OK on Uno
//  Ap11 Artisan Interface and Logging as run time option
//  Ap09 Arti branch UNO+ESP both connect OK to Artisan, all Serial output switched 
//  Ap02 Bigger delay in virtTcpl, better everage in ROC est; Builds ESP, UNO  
//  Ap01 Rot sw complete; Off/On SSR Driver synced with pwm driver; display format
//  Mr31 Backport UNO with Lcds new format, Tcpl, start on RotSwitch ToDo: Hw pin alloc for ESP
//  Mr16 Centigrade, Fahrenheit supported
//  Mr15 cleanup loop timing, elapMSec was too big imm after init()
//  Mr14 fix dsbld pwmdInit(), timing by millis() not micros()
//  Mr12 publish P, I, D, E, terms, subscribe Beta, Gamma 
//  Mr11 Do 1, 2, 5 sec tick callbacks only 
//  Mr10 Remove 500mS delay in cbckDisc(),  const char strings for topics
//  Mr01 Combine User serial command input with subs callback for user commands  
//  Fe22 Add section descriptions, re-order     
//  Fe20 redo callback to 500mS, 1sec, 2sec, 5sec and tune pidc
//  Oc29-popcFEpid pid controller with lcd output, timed loops and simulated temp response
//       lcd display has profile values output on second line  
//       serial data format is compatible with 'Front End' graph display on PC 
// 
//  Sections (units) in this code, ordered alphabetically:
//  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
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
//  Legend / Mode Indicators; Upcase: User Set; LowCase: Auto/Sensed value; User entry: either case;
//  a  toggle run time interface to Artisan 
//  c  centigrade entry/display
//  d  toggle diagnostic verbose messages on serial 
//  f  fahrenhet entry / display, internals always are Centigrade 
//  i  info strings and 'billboard' to serial 
//  l  logging (artisan csv) to serial 
//  p  profile (tbd)
//  r  ramp C/F Degrees per minute, enter before target setpoint
//  s  setPoint C/F temp 
//  w  pwm width override, disssabled PID
//  y  tbd pwm freq Hz 
//  z  reset total timecount
//

//  Code section compiler switches - Rebuild and Upload after changing these 
#define PROC_ESP      0                  // Compile for ESP8266
#define PROC_UNO      1                  // Compile for Arduino Uno
#define IFAC_ARTI     0                  // Start with Artisan interface on Serial
#define IFAC_FRNT     0                  // Obsolete Front/Process interface on Serial 
#define WITH_LCD      1                  // Hdwre has I2C 2x16 LCD display
#define WITH_MAX31855 1                  // Hdwre has thermocouple + circuit
#define WITH_OFFN     0                  // Use ~4sec Off-On SSR, not fast PWM
 
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
#define TCPL_POLL_MSEC   97UL            // mS termocouple poll
#define PWMD_POLL_MSEC  103UL            // mS pwm driver  poll
#define PIDC_POLL_MSEC  101UL            // mS pid control poll
#define VTCP_POLL_MSEC  251UL            // mS virt tcpl   poll
#define ROTS_POLL_MSEC  503UL            // mS rotary sw   poll
#define LCDS_POLL_MSEC 1000UL            // mS lcd display poll
#define PROF_POLL_MSEC  997UL            // mS run control poll
#define OFFN_POLL_MSEC   23UL            // mS run control poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec 

//
#define ambiTmpC  28                     //  28C  82F as room temp 
#define maxiTmpC 248                     // 248C 480F as maximum temp 

// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif

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

// from another copy ??
#include <dummy.h>
#endif

const char versChrs[] = "V 2017Ap15 PID Tweaked";

/// Declarations by unit

// Used for Artisan: ambient, ch1, ch2, ch3, ch4 or Logging Tt Ts BT ET on serial 
char artiResp[] = "023.0,128.8,138.8,000.0,000.0  ";

// billboard  Legend Lower cases: computed/Measured Upper case: User/Setpoints 
char bbrdLin0[] = "w100% r-123 128c"; 
char bbrdLin1[] = "P0-0 S12.3m 228C";
char bbrdHold  = 'H';                      // Prefix to decimal mins alternating with total time
char bbrdRamp  = 'R';
char bbrdSetp  = 'S';
char bbrdTmde;
char *dbugLine = " <==>                                                                           ";
char centScal  = 'C';
char fahrScal  = 'F';
char userScal  = 'C';

// Theese two lines must contain tab chars, not spaces
const char csvlLin1[] = "Date:	Unit:C	CHARGE:	TP:	DRYe:	FCs:	FCe:	SCs:	SCe:	DROP:	COOL:	Time:";
const char csvlLin2[] = "Time1	Time2	BT	ET	Event";

String userCmdl("exactly thirty one chars length");  // Crashable ! 

// FRNT
union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
frntComm;              // float array
byte Auto_Man = -1;
byte Direct_Reverse = -1;
unsigned long frntPoll; //this will help us know when to talk with processing

// lcd                                     
// Pin A4 Pin A5 i2c
// set LCD address to 0x27 for a A0-A1-A2  display
//   args: (addr, en,rw,rs,d4,d5,d6,d7,bl,blpol)
#if WITH_LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif 

#include <SPI.h>
#include <Wire.h>  // Comes with Arduino IDE

// i-n-g-o MQTT 
// mqtt strings are declared for both ESP8266 and UNO 
char mqttVals[] =  "                ";                     // mqtt value string 15 char max
const char RnTops[]    = "/popc/pidc/Rn";
const char YnTops[]    = "/popc/pidc/Yn";
const char EnTops[]    = "/popc/pidc/En";
const char UnTops[]    = "/popc/pidc/Un";
const char PnTops[]    = "/popc/pidc/Pn";
const char InTops[]    = "/popc/pidc/In";
const char DnTops[]    = "/popc/pidc/Dn";
const char KpTops[]    = "/popc/pidc/Kp";
const char TdTops[]    = "/popc/pidc/Td";
const char TiTops[]    = "/popc/pidc/Ti";
const char BetaTops[]  = "/popc/pidc/Beta";
const char GammaTops[] = "/popc/pidc/Gamma";
const char userTops[]  = "/popc/userCmdl";
const char psecTops[]  = "/popc/stepSecs";
const char pcntTops[]  = "/popc/pwmd/perCent";
const char ptmpTops[]  = "/popc/profDegs";
const char c500Tops[]  = "/popc/cbck5000";
const char echoTops[]  = "/popc/echoCmdl";

#if PROC_ESP
#include <MQTT.h>
#define MQCL_ID "popc"

// create MQTT object with IP address, port of MQTT broker e.g.mosquitto application
// MQTT myMqtt(MQCL_ID, "MQTTServerName", MQTTServerPort);
//
MQTT popcMqtt(MQCL_ID, "test.mosquitto.org", 1883);
//
// wifi Replace with your own network's SSID, Password
//
const char* ssid     = "mySSID";
//
const char* password = "myNetPassword";

#endif

//pidc
#if WITH_OFFN
// Slow response PID to match 4sec cycle of SSR Off-On 
float pidcKp      =   6.000;              // P-Term gain
float pidcTi      =   4.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd      =   0.050;              // D-Term Gain sec ( Td++ = Gain++)
#else
//Ap15
// Fast response PID to match approx 30Hz PWM frequency 
//float pidcKp      =   8.000;              // P-Term gain
//float pidcTi      =   8.000;              // I-Term Gain sec ( Ti++ = Gain--)
//float pidcTd      =   0.002;              // D-Term Gain sec ( Td++ = Gain++)
//float pidcKp      =   6.000;              // P-Term gain
//float pidcTi      =  10.000;              // I-Term Gain sec ( Ti++ = Gain--)
//float pidcTd      =   0.002;              // D-Term Gain sec ( Td++ = Gain++)
float pidcKp      =  10.000;              // P-Term gain
float pidcTi      =  12.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcTd      =   0.002;              // D-Term Gain sec ( Td++ = Gain++)
#endif
//
float pidcRn      =  ambiTmpC;            // Refr setpoint
float pidcYn      =  ambiTmpC;            // YInp input
float pidcEn      =   0.000;              // YInp input
float pidcBeta    =   1.000;              // P-term Refr vs YInp
float pidcGamma   =   1.000;              // D-term Refr vs YInp
float pidcAlpha   =   0.100;              // D-term Filter time
float pidcUMax    = 255.000;              // Outp Max
float pidcUMin    =   0.000;              // Outp Min
float dUn, Edn, Epn, Epn1          = 0.0; // Calc error values
float Edfn2, Edfn1, Edfn, Un, Un1  = 0.0; // Calc error, output, prev output values
float Tf, Ts, TsDivTf              = 0.0; // Filter, Actual sample period, stash
float pidcPn, pidcIn, pidcDn       = 0.0; // per sample P-I-D-Err Terms
float pidcPc, pidcIc, pidcDc       = 0.0; // cumulative P-I-D components 
float pidcUn = 0.0;                       // PID controller Output

// pwmd vbls
int  pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp; // Freq, Duty Cycle Target (255max) Output
int  heatHist[] = { 0, 0, 0, 0};             // mavg store for virtTcpl 
int  cdpmHist[] = { 0, 0, 0, 0};             // mavg store for temp rate of change 
byte pwmdPcnt;                               // Percent duty cycle 

// Run Cntl vbls  Bit 0:Run 1:Ctrl 2:Auto 3:Info  Info: 4: 5:Virt 6:Dbug 7:bbrd 
#define RCTL_RUNS 0x80
#define RCTL_AUTO 0x40
#define RCTL_MANU 0x20
#define RCTL_ATTN 0x10
#define RCTL_SPAR 0x08
#define RCTL_DIAG 0x04
#define RCTL_ARTI 0x02
#define RCTL_INFO 0x01

#if PROC_ESP
byte  bbrdRctl  = RCTL_RUNS | RCTL_INFO | RCTL_DIAG;  // See wifi connection 
#elif IFAC_ARTI
byte  bbrdRctl  = RCTL_RUNS | RCTL_ARTI;
#else
byte  bbrdRctl  = RCTL_RUNS | RCTL_INFO;
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
byte  cb50Rctl  = RCTL_RUNS;

byte profNmbr, profStep, profChar, stepChar;    // Numeric Profile No, Step No, Character values 

//  Rotary Switch
byte rotsCurr, rotsNewb, offnCntr, offnOutp;   // Current value, newb test for change; off/on cycle counter, output 

//  time
unsigned int  mSecOflo;
unsigned long currMSec, elapMSec = 0UL;
unsigned long lcdsPrev, lcdsTogo, pidcPrev, pidcTogo, profPrev, profTogo = 0UL;  // mSec poll loop timer counters
unsigned long pwmdPrev, pwmdTogo, rotsPrev, rotsTogo, tcplPrev, tcplTogo = 0UL;  // mSec poll loop timer counters
unsigned long vtcpPrev, vtcpTogo, offnPrev, offnTogo                     = 0UL;  // mSec poll loop timer counters
//

#if PROC_ESP
//pubsub
void dataCbck(String& topic, String& data);
void publCbck();
void discCbck();
void connCbck();

#include "TickerScheduler.h"
TickerScheduler popcShed(3);

// End paste from pubsShed 
//
#else 
#endif 

// Todo maybe these pin assignments are only good for UNO 
#if PROC_ESP
// Off/On SSR driver 
#define OFFN_OPIN  5
// Handle either LED polarity with: (1 & ~LED_LOWON) for light, (0 | LED_LOWOFF) for dark 
#define LED_ONBRD  0
#define LED_LOWON  1
#endif
// PWM Drive
#define PWMD_OPIN 16                        // Pin D9
// Rotary 16way encoder switch; D13 is LED on UNO 
#define ROTS_BIT3  4                        // Pin D4  Val 8 
#define ROTS_BIT2 14                        // Pin D14 Val 4 
#define ROTS_BIT1 12                        // Pin D12 Val 2 
#define ROTS_BIT0 13                        // Pin D13 Val 1 

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
#define TCPL_DO    7                        // Pin D7 Data
#define TCPL_CS    8                        // Pin D8 CSel
#endif
 
#if WITH_MAX31855
#include "Adafruit_MAX31855.h"
Adafruit_MAX31855 tcpl(TCPL_CL, TCPL_CS, TCPL_DO);
//#include "MAX31855.h"
//MAX31855 tcpl(TCPL_CS);
#endif

//
float         targTmpC, sensTmpC, prevTmpC;   // target, sensor, previous temperature deg C
int           userDuty, userDgpm, sensCdpm;   // userSet duty cycle; userSet C/F, measured C deg per min
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
//
void bbrdFill() {
  // Billboard lines are filled for display on LCD and/or Serial with 'info' 
  // billboard line[0]; strf ops append null chars, fill single chars later 
  dtostrf( pwmdPcnt,                              3, 0, &bbrdLin0[1]);
  // Odd secs: show desred ROC; Even secs: show measured ROC 
  if ((profCdpm != 0) && ( totlSecs % 2 )){
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
  if ((profCdpm != 0) && ( totlSecs % 2 )){
    bbrdLin0[6]  = 'R';
  } else {
    bbrdLin0[6]  = 'r';
  } 
  bbrdLin0[11] = ' ';
  bbrdLin0[15]  = userScal + 0x20;
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
    bbrdLin1[15]  = userScal + 0x20;
  } else {
    dtostrf( ((totlSecs / 6) % 10), 1, 0, &bbrdLin1[9]);
    if ( userScal == fahrScal) {
      dtostrf( floatCtoF(profTmpC),                 3, 0, &bbrdLin1[12] );
    } else {
      dtostrf(           profTmpC,                  3, 0, &bbrdLin1[12] );
    }  
    bbrdLin1[15]  = userScal;
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
  // Select 1 sec  or N Sec CSV log entries
  if ( !( bbrdRctl & RCTL_INFO ) && !(totlSecs % 2)) {
  //if ( !( bbrdRctl & RCTL_INFO ) ) {
    // Artisan non -'Info' output is csv logging format: TotMin:Sec StepMin:sec BT ET Event
    dtostrf( (totlSecs / 60), 02, -0, &artiResp[0]);
    dtostrf( (totlSecs % 60), 02, -0, &artiResp[3]);
    dtostrf( (stepSecs / 60), 02, -0, &artiResp[6]);
    dtostrf( (stepSecs % 60), 02, -0, &artiResp[9]);
    if ( userScal == fahrScal) {
      dtostrf( floatCtoF(sensTmpC),       5, 1, &artiResp[12] );
      dtostrf( floatCtoF(targTmpC),       5, 1, &artiResp[19]  );
    } else {
      dtostrf(           sensTmpC,        5, 1, &artiResp[12]  );
      dtostrf(           targTmpC,        5, 1, &artiResp[19]  );
    }
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
    artiResp[17]  = 0x09;   // overwrite <nul>
    artiResp[18]  = ' ';
    artiResp[24]  = ' ';   // overwrite <nul>
    artiResp[25]  = ' ';
    artiResp[26]  = ' ';
    artiResp[27]  = ' ';
    artiResp[28]  = ' ';
    artiResp[29]  = ' ';
    artiResp[30]  = ' ';
    artiResp[31]  = ' ';
    // Flag csv is ready for posting 
    bbrdRctl |= RCTL_ATTN;
  } else {
    if ( bbrdRctl & RCTL_ARTI ) {
      // If Artisan Iface is active, prepare 'READ' temperature response
      if ( userScal == fahrScal) {
        dtostrf( floatCtoF(ambiTmpC),      5, 1, &artiResp[0]  );
        dtostrf( floatCtoF(targTmpC),      5, 1, &artiResp[6]  );
        dtostrf( floatCtoF(sensTmpC),      5, 1, &artiResp[12] );
        dtostrf( floatCtoF(ambiTmpC - 2 ), 5, 1, &artiResp[18] );
        dtostrf( floatCtoF(ambiTmpC - 5 ), 5, 1, &artiResp[24] );
      } else {
        dtostrf(           ambiTmpC,       5, 1, &artiResp[0]  );
        dtostrf(           targTmpC,       5, 1, &artiResp[6]  );
        dtostrf(           sensTmpC,       5, 1, &artiResp[12] );
        dtostrf(           ambiTmpC - 2,   5, 1, &artiResp[18] );
        dtostrf(           ambiTmpC - 5,   5, 1, &artiResp[24] );
      } 
      artiResp[5]  = ',';
      artiResp[11] = ',';
      artiResp[17] = ',';
      artiResp[23] = ',';
    }  
  }  
}

void bbrdLoop() {
  // billboard fill lcd sisplay 32 chars for lcd / mqtt / serial 
  bbrdFill();
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
  lcdsTogo  = LCDS_POLL_MSEC;     // lcd display    poll period mSec
  lcdsPrev  =  millis();
}

void lcdsLoop() {
  currMSec = millis();
  elapMSec = currMSec - lcdsPrev;
  if (( lcdsTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    lcdsPrev = millis();
    lcdsTogo = LCDS_POLL_MSEC;
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
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("connected to MQTT server");
  }
}

void discCbck() {
if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
  Serial.println("disconnected. try to reconnect...");
  //delay(500);
}  
#if PROC_ESP  
  popcMqtt.connect();
#endif  
}

void publCbck() {
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
    Serial.println("popc publCbck");
  }  
}

void dataCbck(String& topic, String& data) {
  int   topiIndx;
  float topiValu;
  if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
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
        Serial.print("new Kp: ");
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
        Serial.print("new Td: ");
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
        Serial.print("new Ti: ");
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
        Serial.print("new Beta: ");
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
        Serial.print("new Gamma: ");
        Serial.println(topiValu);
      }  
    }  
  }
  topiIndx = topic.indexOf("userCmdl");
  if (topiIndx >= 0){
    // copy data into user command line
    userCmdl = data;
    userRctl |= RCTL_ATTN; 
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print("User Command: ");
      Serial.println(userCmdl);
    }
  }  
}

void cbck1000() {
  cb10Rctl |= RCTL_ATTN;
}

void cb10Svce() {
  int rCode = 0;
  //
  dtostrf( pidcRn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )RnTops , (const char * )mqttVals, 15 ); 
#endif  
  //
  dtostrf( pidcYn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )YnTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcEn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )EnTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcUn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )UnTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcPn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )PnTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcDn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )DnTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcIn, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )InTops, (const char * )(mqttVals), 15 ); 
#endif  
  //
  dtostrf( stepSecs, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( psecTops, (const char * )(mqttVals), 15 );
#endif  
  //
  //if ( rCode) {
    //Serial.print("cbck1000 bad      RC: ");
    //Serial.println(rCode);
  //}
  //
  cb10Rctl &= ~RCTL_ATTN;
}

void cbck2000() {
  cb20Rctl |= RCTL_ATTN;
}

void cb20Svce() {
  int rCode = 0;
  //
  cb20Rctl &= ~RCTL_ATTN;
}

void cbck5000() {
  cb50Rctl |= RCTL_ATTN;
}

void cb50Svce() {
  int rCode = 0;
  dtostrf( millis(), 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )c500Tops, (const char *)(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcKp, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )KpTops,   (const char *)(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcTi, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )TiTops,   (const char *)(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pidcTd, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )TdTops,   (const char *)(mqttVals), 15 ); 
#endif  
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(profTmpC), 8, 3, mqttVals);
  } else {
    dtostrf(           profTmpC,  8, 3, mqttVals);
  }  
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )ptmpTops, (const char *)(mqttVals), 15 ); 
#endif  
  //
  dtostrf( pwmdPcnt, 8, 3, mqttVals);
#if PROC_ESP  
  rCode += popcMqtt.publish( (const char * )pcntTops, (const char *)(mqttVals), 15 ); 
#endif  
  //
  //if ( rCode) {
    //Serial.print("cbck2000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  //if ( rCode) {
    //Serial.print("cbck5000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  cb50Rctl &= ~RCTL_ATTN;
}  
  
void popcSubs() {
  int rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( KpTops );
#endif  
  //if ( rCode) {
    //Serial.print("bbrdSubs Kp bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( BetaTops );
#endif  
  //if ( rCode) {
    //Serial.print("bbrdSubs Beta bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( GammaTops );
#endif  
  //if ( rCode) {
    //Serial.print("bbrdSubs Gamma bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( TdTops );
#endif  
  //if ( rCode) {
    //Serial.print("bbrdSubs Td bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TdTops);
  //}
  //
  rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( TiTops );
#endif  
  //
  //if ( rCode) {
    //Serial.print("bbrdSubs Ti bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TiTops);
  //}
  //
  rCode = 0;
#if PROC_ESP  
  rCode += popcMqtt.subscribe( userTops );
#endif  
  //
  //if ( rCode) {
    //Serial.print("bbrdSubs Ti bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TiTops);
  //}
  //
}  

/// Off-On SSR driver 
//
void offnInit() {
  offnTogo = OFFN_POLL_MSEC;   //  poll period mSec
  offnPrev = millis();
  offnCntr = 0;
  pinMode( LED_ONBRD, OUTPUT);
  pinMode( OFFN_OPIN,   OUTPUT);
}  

void offnLoop() {
  currMSec = millis();
  elapMSec = currMSec - offnPrev;
  if (( offnTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    offnPrev = millis();
    offnTogo = OFFN_POLL_MSEC ;
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
void pidcInit() {
  Tf = pidcAlpha * pidcTd;
  Epn1 = 0.0;
  Edfn2 = Edfn1 = Edfn = 0;
  // first time being enabled, seed with current property tree value
  Un1 = Un = 0;
  pidcTogo = PIDC_POLL_MSEC;      // PID control poll period mSec
  pidcPrev =  millis();
  targTmpC = ambiTmpC;
}

//
void pidcLoop() {
  currMSec = millis();
  elapMSec = currMSec - pidcPrev;
  if (( pidcTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    // save this currMSec as time of service
    //pidcPrev = currMSec;
    // Adjusting with elapMSec errors after init when 8secs elapse  time togo  
    //pidcTogo = 2 * PIDC_POLL_MSEC - elapMSec;
    pidcPrev = millis();
    pidcTogo = PIDC_POLL_MSEC;
    //Serial.print("pidc ");
    //Serial.println( elapMSec);
    //
    if ( pidcRctl & RCTL_RUNS  == 0 ) {
      // Poll/Thermocouple == 0 Shutdown
      Serial.println('pidc stop');
      Un = 0;
    } else {
      // 
      //Ts = (PIDC_POLL_MSEC - (float)pidcTogo) / 1000000.0; // sample interval (Sec)
      Ts = (float)elapMSec / 1000.0;
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
      pidcPn = pidcKp *  (Epn - Epn1);
      pidcPc += pidcPn;
      // I term 
      if ( pidcTi > 0.0 ) {
        pidcIn = pidcKp * ((Ts / pidcTi) * pidcEn);
      } else {
        pidcIn = 0;
      }    
      pidcIc += pidcIn;
      // D term
      if ( pidcTd > 0.0 ) {
        pidcDn = pidcKp * ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
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
      // Updates indexed values;
      Un1   = Un;
      Epn1  = Epn;
      Edfn2 = Edfn1;
      Edfn1 = Edfn;
      pidcUn = Un;
    }
  }    
}

void pidcDbug() {
  //
  dbugLine[6]   = 'k';
  dtostrf( (pidcKp  ), 6, 1, &dbugLine[7] );
  dbugLine[13]  = ' ';
  //
  dbugLine[14]  = 'v';
  dtostrf( (pidcTd  ), 6, 1, &dbugLine[15] );
  dbugLine[21]  = ' ';
  //
  dbugLine[22]  = 's';
  dtostrf( (pidcTi  ), 6, 1, &dbugLine[23] );
  dbugLine[29]  = ' ';
  //
  dbugLine[30]  = 'e';
  dtostrf( (pidcEn  ), 6, 1, &dbugLine[31] );
  dbugLine[37]  = ' ';
  //
  dbugLine[38]  = 'p';
  dtostrf( (pidcPc  ), 6, 1, &dbugLine[39] );
  dbugLine[45]  = ' ';
  //
  dbugLine[46]  = 'i';
  dtostrf( (pidcIc  ), 6, 1, &dbugLine[47] );
  dbugLine[53]  = ' ';
  //
  dbugLine[54]  = 'd';
  dtostrf( (pidcDc  ), 6, 1, &dbugLine[55] );
  dbugLine[61]  = ' ';
  //
  dbugLine[62]  = 'o';
  dtostrf( (pidcUn), 6, 1, &dbugLine[63] );
  dbugLine[69]  = ' ';
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
  prevTmpC = sensTmpC = profTmpC = ambiTmpC;
  stepSecs = totlSecs = 0;
  profTogo = PROF_POLL_MSEC;   //  poll period mSec
  profPrev =  millis();
}

void profLoop() {
  currMSec = millis();
  elapMSec = currMSec - profPrev;
  if (( profTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    profPrev = millis();
    profTogo = PROF_POLL_MSEC ;
    // prevent lcd rollover; 6000 secs == 100 min 
    (totlSecs > 5998) ? (totlSecs = 0):(totlSecs += 1);
    (stepSecs > 5998) ? (stepSecs = 0):(stepSecs += 1);
    if (profNmbr > 0) {
      // ToDo saved profiles
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        //Serial.println("profNmbr");
      }  
    } else {
      // Default: No ramp, target setpoint temperature 
      targTmpC = float(profTmpC);
      // Ramp to setpt temp and hold, Unless Manual Ramp 
      if (profCdpm != 0)  {
        if (profTmpC >= profTbeg)  {
          // Only stop ramp if non manual 
          if ((sensTmpC >=  targTmpC) && (stepSecs > 10) && (userDgpm == 0)) {
            profCdpm = 0;
            stepSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     + float (stepSecs) * float(profCdpm) / 60.0;
          }  
        } else {
          // Only stop ramp if non manual 
          if ((sensTmpC <= targTmpC) && (stepSecs > 10) && (userDgpm == 0)) {
            profCdpm = 0;
            stepSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     - float (stepSecs) * float(profCdpm) / 60.0;
          }  
        }
      } 
      if ((profCdpm == 0) && (userDgpm == 0)) {
        // ramp==0: target temp  unchanged
        profCdpm = 0;
        // StepSecs shows time in either Setpoint of Hold after ramp 
        //stepSecs = 0;
        bbrdTmde = bbrdSetp;
      }
    }
    // Run exp mavg of temp rate of Rise 
    if (( totlSecs % 6) == 0) {
      cdpmHist[3] = cdpmHist[2] ;
      cdpmHist[2] = cdpmHist[1] ;
      cdpmHist[1] = cdpmHist[0] ;
      cdpmHist[0] = (int)(sensTmpC - prevTmpC)  ;
      prevTmpC    = sensTmpC;
      // ROC degrees per min is 60 * avg per secon change 
      sensCdpm =      ( 4 * cdpmHist[0]  +  3 * cdpmHist[1] \     
                      + 2 * cdpmHist[2]  +      cdpmHist[3] ); 
    }                  
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      //Serial.print("Curr:");
      //Serial.print(sensTmpC);
      //Serial.print(" Prev:");
      //Serial.print(prevTmpC);
      //Serial.print(" Cdpm:");
      //Serial.println(sensCdpm);
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
        if ((pidcRctl & RCTL_DIAG) == RCTL_DIAG) {  
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
          for ( tempIndx = 0; tempIndx < 31; tempIndx++ ) {
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
  //analogWriteFreq(newFreq);
#else 
#endif 
  pwmdFreq = int(newFreq);
}

void pwmdInit() {
  pwmdDuty = 0;
  #if PROC_ESP
  #else 
  pwmdExpo(10);
  pinMode( PWMD_OPIN, PWMD_MODE);
  #endif
  pwmdTogo  = PWMD_POLL_MSEC;       // pwm drvr poll period mSec
  pwmdPrev  =  millis();            // pwm drvr poll period mSec
}

void pwmdLoop() {
  currMSec = millis();
  elapMSec = currMSec - pwmdPrev;
  if (( pwmdTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    pwmdPrev = millis();
    pwmdTogo = PWMD_POLL_MSEC;
    //
    if (pwmdRctl == 0) {
      if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
        Serial.print("pwmdRctl==0");
      }  
      pwmdOutp = 0;
    } else {
      // last run control test has precedence 
      if ( pwmdRctl & RCTL_MANU) {
        pwmdTarg = byte( 255.0 * userDuty / 100.0 );
      }
      if ( pwmdRctl & RCTL_AUTO) {
        pwmdTarg = byte(pidcUn);
      }
      //Serial.print("pwmdTarg: ");
      //Serial.println(pwmdTarg);
      pwmdOutp = byte(pwmdTarg);
      pwmdPcnt = byte (100.0 * pwmdOutp / 255);
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
  if ( digitalRead(ROTS_BIT3) == LOW  ) resp  = 8; 
  if ( digitalRead(ROTS_BIT2) == LOW  ) resp += 4; 
  if ( digitalRead(ROTS_BIT1) == LOW  ) resp += 2; 
  if ( digitalRead(ROTS_BIT0) == LOW  ) resp += 1; 
  return(resp);
}
    
void rotsInit() {
  // Set pins to weak pullup 
  pinMode( ROTS_BIT3, INPUT_PULLUP);
  pinMode( ROTS_BIT2, INPUT_PULLUP);
  pinMode( ROTS_BIT1, INPUT_PULLUP);
  pinMode( ROTS_BIT0, INPUT_PULLUP);
  rotsTogo = ROTS_POLL_MSEC;   //  poll period mSec
  rotsPrev =  millis();
}
    
void rotsLoop() {
  currMSec = millis();
  elapMSec = currMSec - rotsPrev;
  if (( rotsTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    rotsPrev = millis();
    rotsTogo = ROTS_POLL_MSEC;
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
          break;
          case 1:
            profStep = 1;
            userCmdl = "R0";
          break;
          case 2:
            profStep = 2;
            userCmdl = "R5";
          break;
          case 3:
            profStep = 3;
            userCmdl = "R10";
          break;
          case 4:
            profStep = 4;
            userCmdl = "R15";
          break;
          case 5:
            profStep = 5;
            userCmdl = "R20";
          break;
          case 6:
            profStep = 6;
            userCmdl = "R25";
          break;
          case 7:
            profStep = 7;
            userCmdl = "R30";
          break;
          case 8:
            profStep = 8;
            userCmdl = "R35";
          break;
          case 9:
            profStep = 9;
            userCmdl = "R40";
          break;
          case 10:
            profStep = 10;
            userCmdl = "R45";
          break;
          case 11:
            profStep = 11;
            userCmdl = "R60";
          break;
          case 12:
            profStep = 12;
            userCmdl = "R75";
          break;
          case 13:
            profStep = 13;
            userCmdl = "R90";
          break;
          case 14:
            profStep = 14;
            userCmdl = "R120";
          break;
          case 15:
            profStep = 15;
            userCmdl = "W100";
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
  tcplTogo  = TCPL_POLL_MSEC;       // thermocouple norm poll period mSec
  // stabilize wait .. lcds banner is 1000mA anyway
  // delay(500);
  vtcpTogo = VTCP_POLL_MSEC;      // PID control poll period mSec
  tcplPrev = vtcpPrev = millis();
  sensTmpC = ambiTmpC;
}

void virtTcplLoop() {
  int pwmdMavg;
  float heatInpu = 0; 
  currMSec = millis();
  elapMSec = currMSec - vtcpPrev;
  if (( vtcpTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    vtcpPrev = millis();
    vtcpTogo = VTCP_POLL_MSEC;
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
#endif                  
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
    heatHist[2] = heatHist[1]; 
    heatHist[1] = heatHist[0]; 
    heatHist[0] = heatInpu;
  }  
}    

#if WITH_MAX31855
void tcplRealLoop() {
  double tcplTmpC;
  currMSec = millis();
  elapMSec = currMSec - tcplPrev;
  if (( tcplTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    tcplPrev = millis();
    tcplTogo = TCPL_POLL_MSEC;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      sensTmpC = ambiTmpC;
    } else {
      // Read thermocouple 
      tcplTmpC = tcpl.readCelsius();
      if (isnan(tcplTmpC)) {
        lcd.clear();
        lcd.home ();
        lcd.print(F("@tcplRealLoop()"));
        lcd.setCursor ( 0, 1 );
        lcd.print(F("Thermocouple Err"));
        delay (500);
        lcd.clear();
      } else {
        // Only update sensed temp with a valid reading 
        sensTmpC = float( tcplTmpC); 
      }  
    }  
  }  
}  
#endif

/// User Interface 
//
void userInit() {
  profNmbr = profStep = 0;
  profCdpm =  0;
  userScal = centScal;
  profTbeg = profTmpC = userDegs = ambiTmpC;
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
    Serial.println("Svce");
    Serial.println(userCmdl);
  }  
#if PROC_ESP  
  popcMqtt.publish( (const char * )echoTops, userCmdl); 
#endif  
  if ( bbrdRctl & RCTL_ARTI ) {
    if ((userCmdl[0] == 'C') && (userCmdl[1] == 'H') && (userCmdl[2] == 'A')) {
      //  'chan' command, respond '#'
      Serial.println("#");
    }  
    //  'unit' command, set user scale 
    if ((userCmdl[0] == 'U') && (userCmdl[1] == 'N') && (userCmdl[5] == 'C')) {
      userScal = centScal;
    }  
    if ((userCmdl[0] == 'U') && (userCmdl[1] == 'N') && (userCmdl[5] == 'F')) {
      userScal = fahrScal;
    }
    if ((userCmdl[0] == 'R') && (userCmdl[1] == 'E') && (userCmdl[2] == 'A')) {
      Serial.println(artiResp);
    }
    if ((userCmdl[0] == 'I') && (userCmdl[1] == 'O') && (userCmdl[2] == '3')) {
      userDuty = (userCmdl.substring(4)).toInt();
      if (userDuty > 99) userDuty = 100;
    }
    if ((userCmdl[0] == 'P') && (userCmdl[1] == 'I') && (userCmdl[2] == 'D')) {
      if ((userCmdl[4] == 'S') && (userCmdl[5] == 'V')) {
        // set desired temperatre degC
        userDegs = (userCmdl.substring(7)).toInt();
        if ( userScal == fahrScal) {
          profTmpC = intgFtoC( userDegs); 
        } else {
          profTmpC = userDegs;
        }
        if (profTmpC > maxiTmpC) profTmpC = maxiTmpC;
        profTbeg = int(sensTmpC);
        stepSecs = 0;
        pwmdRctl &= ~RCTL_MANU;
        pwmdRctl |=  RCTL_AUTO;
      }  
    }
  }  
  //    
  if ((userCmdl[0] == 'A') || (userCmdl[0] == 'a')) {
    // Toggle Artisan Interface
    if ( bbrdRctl & RCTL_ARTI ) {
      bbrdRctl &= ~RCTL_ARTI; 
      Serial.println("# Serial  Interface is Active");
    } else {
      bbrdRctl |=  RCTL_ARTI;
      Serial.println("# Artisan Interface is Active");
    }
  }
  if ((userCmdl[0] == 'D') || (userCmdl[0] == 'd')) {
    // Toggle Diagnostic Flag
    if ( bbrdRctl & RCTL_DIAG ) {
      Serial.println("# Diagnostics Mode  is InActive");
      bbrdRctl &= ~RCTL_DIAG; 
    } else {
      bbrdRctl |=  RCTL_DIAG; 
      Serial.println("# Diagnostics Mode  is   Active");
    }
  }
  if (((userCmdl[0] == 'C') || (userCmdl[0] == 'c')) && (userCmdl[1] != 'H')) {
    userScal = centScal;
  }
  if (((userCmdl[0] == 'F') || (userCmdl[0] == 'f')) && (userCmdl[1] != 'I')) {
    userScal = fahrScal;
  }
  if ((userCmdl[0] == 'I') || (userCmdl[0] == 'i')) {
    // Switch Off Artisan csv Logging, start popc 'Info' on Serial 
    bbrdRctl |= RCTL_INFO; 
  }
  if ((userCmdl[0] == 'L') || (userCmdl[0] == 'l')) {
    // Toggle logging 
    if ( bbrdRctl & RCTL_INFO ) {
      // Artisan csv Logging: send two header lines with tab chars
      Serial.println(csvlLin1); 
      Serial.println(csvlLin2); 
      // Switch On  Artisan csv Logging. TotalTime, StepTime must start at 0. 
      bbrdRctl &= ~RCTL_INFO; 
      stepSecs = totlSecs = 0;
    } else {
      bbrdRctl |= RCTL_INFO; 
    }  
  }
  if ((userCmdl[0] == 'P') || (userCmdl[0] == 'P')) {
    // TBD select stored profile
    profNmbr = (userCmdl.substring(1)).toInt();
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      //Serial.println(F("userfSele"));
    }  
    stepSecs = 0;
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
    stepSecs = 0;
    if (userDgpm > 0) profTmpC = maxiTmpC;
    if (userDgpm < 0) profTmpC = ambiTmpC;
    if (userDgpm == 0) {
      // Selected ROC 0: Hold current temp 
      bbrdTmde = bbrdHold;
      profTmpC = int(sensTmpC);
    }
    // seting ramp unsets manual PWM width 
    pwmdRctl &= (~RCTL_MANU);
    pwmdRctl |=  RCTL_AUTO;
    if ( !( bbrdRctl & RCTL_ARTI ) && ( bbrdRctl & RCTL_DIAG) ) {
      Serial.print(F("\nprofCdpm"));
      Serial.println(profCdpm);
    }  
  }
  if ((userCmdl[0] == 'S') || (userCmdl[0] == 's')) {
    // set desired temperatre degC
    userDegs = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      profTmpC = intgFtoC( userDegs); 
    } else {
      profTmpC = userDegs;
    }
    if (profTmpC > maxiTmpC) profTmpC = maxiTmpC;
    profTbeg = int(sensTmpC);
    stepSecs = 0;
    pwmdRctl &= ~RCTL_MANU;
    pwmdRctl |=  RCTL_AUTO;
  }
  if ((userCmdl[0] == 'V') || (userCmdl[0] == 'v')) {
    // Version string e if Artisan is setting Unit C/F 
    if  (!( bbrdRctl & RCTL_ARTI )) {
      Serial.println(versChrs);
    }  
  }
  if ((userCmdl[0] == 'W') || (userCmdl[0] == 'w')) {
    // set new pwmD Width, run control flag to indicate manual override
    userDuty = (userCmdl.substring(1)).toInt();
    if (userDuty > 99) userDuty = 100;
    if ( userDuty == 0) {
      // Power off: Set temp setpt to nominal
      profTmpC = ambiTmpC;
    }
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
  rotsInit();
  userInit();
  tcplInit();
  pidcInit();
  pwmdInit();
  offnInit();
  profInit();
#if PROC_ESP
  if (wifiRctl & RCTL_RUNS) {
    if ( bbrdRctl & RCTL_ARTI == 0) {
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);
    }  
    //  Wifi Setup 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    if ( bbrdRctl & RCTL_ARTI == 0) {
      Serial.print(".");
    }  
      delay(4000);
    }
    if ( bbrdRctl & RCTL_ARTI == 0) {
      Serial.println("");
      Serial.println("WiFi conn to IP: ");
      Serial.println(WiFi.localIP());
    }  
    //  MQTT Setup 
    //    setup callbacks
    popcMqtt.onConnected(connCbck);
    popcMqtt.onDisconnected(discCbck);
    popcMqtt.onPublished(publCbck);
    popcMqtt.onData(dataCbck);
    if ( bbrdRctl & RCTL_ARTI == 0) {
      Serial.println("Connect to mqtt...");
    }  
    popcMqtt.connect();
    delay(8000);
    popcSubs();
  }  

  //  Tick setup 
  
  //  Sked setup
  //    TickerScheduler(uint size);
  //    boolean add(uint i, uint32_t period, tscallback_t f, boolean shouldFireNow = false);
  //      ts.add(0, 3000, sendData)
  // 
  int shedRcod;
  shedRcod = popcShed.add( 1, 1000, cbck1000);
  shedRcod = popcShed.add( 2, 2000, cbck2000);
  shedRcod = popcShed.add( 3, 5000, cbck5000);
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
  if (cb50Rctl & RCTL_ATTN) {
    cb50Svce();
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
  userLoop();
  if (userRctl & RCTL_ATTN) {
    userSvce();
  }  
  bbrdLoop();
#if PROC_ESP
  popcShed.update();
#endif  
#if WITH_LCD
  lcdsLoop();
#endif  
  profLoop();
  //frntLoop();
  // Why delay(10);
  delay(10);
}
