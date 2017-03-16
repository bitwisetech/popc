/// popcEsp popc port to esp with MQTT, tickScheduler, is MQTT client popc
//  Mr15
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
///
//  Sections (units) in this code, ordered alphabetically:
//  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
//  frnt  deprecated, sends data over Serial to Frontside / Process apps on PC for PID tuning   
//  lcds  support for I2C 2x16 LCD display ( was working on Nano )     
//  mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
//  pidc  PID controller for PWM powered temperature control; a delta-time incremental PID 
//  popc  Publish, Subscribe, Callback, Scheduler functions for MQTT /Ticker Scheduler
//  pwmd  8-bit PWM control via hardware pwm pins 
//  prof  Profile control; selects auto/manual temp setpt, manual pwm width, real/fake temp sensor
//  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
//  user  receive user's commands via serial port, MQTT for setpoint, ramp, profile modes 
//    
//  Backport to Arduino Nano is untested  Arduino H/W details:  
///
//  nano: Ser:0,1  ExtInt:2 PWM:9,10(490HzTmr1) 3,11(490HzTmr2) PWM:5,6 (980HzTmr0 + mS, delay)
//        SPI:10,11,12,13 I2C SDA:A4 SCL:A5 LED:13
//
//  pins:  lcdi: A4 A5; 
//      tcplSPI:        D3 D5    D10
//          pwm:              D9      
// include libs i2c, lcd, tcpl
///
//  defns for user / actions 
//  c  cent 
//  f  fahr 
//  p  profile
//  r  ramp
//  s  setPt temp 
//  t  ?? time temp
//  w  pwm width
//  y  pwm freq Hz 
//
#define IN_ESP 1 
#define IN_UNO 0 
#define NO_LCD 1 
// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif
// 
#if IN_ESP
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
#endif

//ab from another copy ??
#include <dummy.h>

//// declarations by unit

//wip
//union bbrd32 {
//typedef union lcdsBbrd {
  //struct{
    //char[16] Hlf0;
    //char[16] Hlf1;
  //};  
  //char[32] Full;
//}
//bbrd32 newShow;

// billboard       Tcpl  Setp Power
char bbrdFull[] = "T110c S210c 100%P185 R99  St    ";
char bbrdLin0[] = "T110c S210c 100%";
char bbrdLin1[] = "P185 R99  St    ";
char bbrdHold = 'h';
char bbrdRamp = 'r';
char bbrdSetp = 's';
char bbrdTmde;
char *dbugLine = " <==>                                                                           ";
char centScal  = 'C';
char fahrScal  = 'F';
char userScal  = 'C';

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
#if IN_ESP
#else 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif 

#include <SPI.h>
#include <Wire.h>  // Comes with Arduino IDE

// i-n-g-o MQTT 
#include <MQTT.h>
#define MQCL_ID "popc"
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
const char psecTops[]  = "/popc/profSecs";
const char pcntTops[]  = "/popc/pwmd/perCent";
const char ptmpTops[]  = "/popc/profDegs";
const char c500Tops[]  = "/popc/cbck5000";

// create MQTT object with IP address, port of MQTT broker e.g.mosquitto application
//ab  MQTT myMqtt(MQCL_ID, "192.168.0.1", 1883);
//  nettles MQTT popcMqtt(MQCL_ID, "192.168.1.16", 5983);
//  abby
MQTT popcMqtt(MQCL_ID, "172.20.224.111", 5983);

//pidc
float pidcRn      =  20.000;              // Refr setpoint
float pidcYn      =  20.000;              // YInp input
float pidcEn      =   0.000;              // YInp input
float pidcKp      =   4.000;              // P-Term gain
float pidcTd      =   0.010;              // D-Term Gain sec ( Td++ = Gain++)
float pidcTi      =   2.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcBeta    =   1.000;              // P-term Refr vs YInp
float pidcGamma   =   1.000;              // D-term Refr vs YInp
float pidcAlpha   =   0.100;              // D-term Filter time
float pidcUMax    = 250.000;              // Outp Max
float pidcUMin    =   0.000;              // Outp Min
float dUn, Edn, Epn, Epn1          = 0.0; // Calc error values
float Edfn2, Edfn1, Edfn, Un, Un1  = 0.0; // Calc error, output, prev output values
float Tf, Ts, TsDivTf              = 0.0; // Filter, Actual sample period, stash
float pidcPn, pidcIn, pidcDn       = 0.0; // per sample P-I-D-Err Terms
float pidcPc, pidcIc, pidcDc       = 0.0; // cumulative P-I-D components 
float pidcUn = 0.0;                       // PID controller Output

// pwmd vbls
int  pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp; // Freq, Duty Cycle Target (250max) Output
int  pwmdHist[] = { 0, 0, 0, 0};
byte pwmdPcnt;                               // Percent duty cycle 

// millisecond poll values
#define TCPL_POLL_MSEC  100UL            // 250mS termocouple poll
#define PWMD_POLL_MSEC  100UL            // 250mS pwm driver  poll
#define PIDC_POLL_MSEC  100UL            // 250mS pid control poll
#define LCDS_POLL_MSEC 5000UL            // 2Sec  lcd display poll
#define PROF_POLL_MSEC 1000UL            // 1Sec  run control poll
#define POLL_SLOP_MSEC    5UL            // Avge loop time is 10mSec 

// Run Cntl vbls  Bit 0:Run 1:Ctrl 2:Auto 3:Info  Info: 4: 5:Virt 6:Dbug 7:bbrd 
#define RCTL_RUNS 0x80
#define RCTL_MSET 0x40
#define RCTL_AUTO 0x20
#define RCTL_ATTN 0x10
#define RCTL_VIRT 0x08
#define RCTL_DIAG 0x04
#define RCTL_BBRD 0x02
#define RCTL_INFO 0x01

byte  bbrdRctl  = RCTL_RUNS;
byte  frntRctl  = 0x00;
byte  lcdstRctl  = 0x00;
byte  pidcRctl  = (RCTL_RUNS | RCTL_AUTO );
byte  pwmdRctl  = (RCTL_RUNS | RCTL_AUTO);
byte  profRctl  = RCTL_RUNS;
byte  tcplRctl  = (RCTL_RUNS | RCTL_VIRT);
byte  userRctl  = RCTL_RUNS;
byte  wifiRctl  = RCTL_RUNS;
// scheduler tick callback attn flags 
byte  cb10Rctl  = RCTL_RUNS;
byte  cb20Rctl  = RCTL_RUNS;
byte  cb50Rctl  = RCTL_RUNS;

#if IN_ESP
//pubsub
void dataCbck(String& topic, String& data);
void publCbck();
void discCbck();
void connCbck();

#include "TickerScheduler.h"
TickerScheduler popcShed(3);

//  time
unsigned int  mSecOflo;
unsigned long currMSec, elapMSec = 0UL;
unsigned long lcdsPrev, pidcPrev, profPrev, pwmdPrev, tcplPrev = 0UL; // mSec time of prev exec 
unsigned long lcdsTogo, pidcTogo, profTogo, pwmdTogo, tcplTogo = 0UL; // mSec time to next exec
//
// wifi
//abconst char* ssid     = "ssid";
//ab const char* password = "ssid_password";
//
const char* ssid     = "inactive";
//
const char* password = "pickledcrab1102190";
//const char* ssid     = "bitwComW";
//const char* password = "manchester1102190tan";
//const char* ssid     = "nettles385";
//const char* password = "oklahoma";

// End paste from pubsShed 
//
#else 
#endif 

// PWM Drive
// d9 needed by RFI scan  d6 would use tmr0 want d3 used by max13855

#define PWMD_OPIN  9                        // Pin D9
#define PWMD_MODE  OUTPUT

// tcpl
#define TCPL_CL    4                        // Pin D10 Chip Select 
#define TCPL_DO    7                        // Pin D10 Chip Select 
#define TCPL_CS    8                        // Pin D10 Chip Select 

// what breaks : Ada delay.h, no need cos using ctor c.f non Ada 
//ab#include "Adafruit_MAX31855.h"
#include "MAX31855.h"
//MAX31855 tcpl(TCPL_CS);
MAX31855 tcpl(TCPL_CL, TCPL_CS, TCPL_DO);

//
float         targTmpC, sensTmpC;             // target, sensor temperature deg C
int           msetDuty;                        // manual oride duty cycle, user temp deg C 
int           profSele, profTbeg, profSecs ;  // profile selector, step start, elapsed time
int           profTmpC, profCdpm, userDegs;   // profile final temp, rate, user C/F tempr 
int           tempIndx;                       // temporary array indexer

/// Common 
///   Convert
float floatCtoF( float celsInp) {
  return (float( ((celsInp + 40.0) * 9.0 / 5.0 )  -40.0 ));
}  

float floatFtoC( float fahrInp) {
  return (float( ((fahrInp + 40.0) * 5.0 / 9.0 )  -40.0 ));
}  
  
int    intgCtoF( int   celsInp) {
  return (int  ( ((celsInp + 40.0) * 9.0 / 5.0 )  -40.0 ));
}  

int    intgFtoC( int   fahrInp) {
  return (int  ( ((fahrInp + 40.0) * 5.0 / 9.0 )  -40.0 ));
}  
  
//  Timing 
unsigned long mSecPast( unsigned long mSecLast) {
  // last millis() remains in global currMSec
  currMSec =  millis();
  // correct result with uns long even after oflo
  return (currMSec - mSecLast);
}

//  billboard
void bbrdFill() {
  
  // strf ops append null chars, fill single chars later 
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(sensTmpC), 3, 0, &bbrdLin0[0] );
  } else {
    dtostrf(           sensTmpC,  3, 0, &bbrdLin0[0] );
  }  
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(targTmpC), 3, 0, &bbrdLin0[7] );
  } else {
    dtostrf(           targTmpC,  3, 0, &bbrdLin0[7] );
  }  
  dtostrf( pwmdPcnt, 3, 0, &bbrdLin0[12]);
  //
  bbrdLin0[3]  = userScal;
  bbrdLin0[4]  = '-';
  bbrdLin0[5]  = '>';
  bbrdLin0[6]  = 'S';
  bbrdLin0[10] = ' ';
  bbrdLin0[11] = 'W';
  bbrdLin0[15] = '%';
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(profTmpC), 3, 0, &bbrdLin1[1] );
  } else {
    dtostrf(           profTmpC,  3, 0, &bbrdLin1[1] );
  }  
  if ( (profCdpm > 0 ) && (userScal == fahrScal) ) {
    dtostrf( floatCtoF(profCdpm), 3, 0, &bbrdLin1[6] );
  } else {
    dtostrf(           profCdpm,  3, 0, &bbrdLin1[6] );
  }  
  dtostrf( profSecs, 4, 0, &bbrdLin1[12]);
  //
  bbrdLin1[0]  = 'P';
  bbrdLin1[4]  = ' ';
  bbrdLin1[5]  = 'R';
  bbrdLin1[ 9] = ' ';
  bbrdLin1[10] = 'T';
  bbrdLin1[11] = bbrdTmde;
  //
  if (pidcRctl & (RCTL_INFO | RCTL_BBRD) ){
    bbrdLin1[0]  = 'p';
    dtostrf(   pidcPc, 3, 0, &bbrdLin1[1] );
    bbrdLin1[4]  = 'i';
    dtostrf(   pidcIc, 3, 0, &bbrdLin1[5] );
    bbrdLin1[8]  = 'd';
    dtostrf(   pidcDc, 3, 0, &bbrdLin1[9] );
    bbrdLin1[12] = 'o';
    dtostrf( pidcUn, 3, 0, &bbrdLin1[13] );
  }
}

void bbrdLoop() {
  // billboard fill lcd sisplay 32 chars for lcd / mqtt / serial 
  bbrdFill();
}

void bbrdPubl( byte tTyp) {
  if (pidcRctl & (RCTL_INFO & RCTL_BBRD) ){
    Serial.print("bbrdPubl tTyp: ");
    Serial.println(tTyp);
  }
}  

// 'Front' something ( Processing ?), serial ifac to PC graphing app c
void frntRecv() {
  // read the bytes sent from Processing
  int index=0;
  while(Serial.available()&&index<26)
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
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
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

#if NO_LCD
#else 
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
  elapMSec = currMSec - pidcPrev;
  if (( lcdsTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    lcdsPrev = millis();
    lcdsTogo = LCDS_POLL_MSEC;
    //
    if (lcdstRctl== 0) {
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
#endif // end non-ESP

///  PID Controller
///  pidc - implementation of PID controller
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
  targTmpC = 20;
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
        if (pidcRctl & (RCTL_INFO | RCTL_BBRD) ) {
          Serial.println(F("maxSatn"));
        }  
      } else if ( dUn < (pidcUMin - Un1) ) {
        dUn = pidcUMin - Un1;
        if (pidcRctl & (RCTL_INFO | RCTL_BBRD) ) {
          Serial.println(F("minSatn"));
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
  dbugLine[6]  = 'k';
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
    Serial.write(dbugLine[tempIndx]);
  } 
}

////
//
void connCbck() {
  Serial.println("connected to MQTT server");
}

void discCbck() {
  Serial.println("disconnected. try to reconnect...");
  //delay(500);
  popcMqtt.connect();
}

void publCbck() {
  //Serial.println("popc publCbck");
}

void dataCbck(String& topic, String& data) {
  int   topiIndx;
  float topiValu;
  Serial.print("dataCbck topic:");
  Serial.print(topic);
  Serial.print("   data:");
  Serial.println(data);
  topiIndx = topic.indexOf("pidc/Kp");
  Serial.println("cbck Kp");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcKp) {
      Serial.print("new Kp: ");
      Serial.println(topiValu);
      pidcKp = topiValu;
    }  
  }
  topiIndx = topic.indexOf("pidc/Td");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTd) {
      Serial.print("new Td: ");
      Serial.println(topiValu);
      pidcTd = topiValu;
    }  
  }
  topiIndx = topic.indexOf("pidc/Ti");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcTi) {
      Serial.print("new Ti: ");
      Serial.println(topiValu);
      pidcTi = topiValu;
    }  
  }  
  topiIndx = topic.indexOf("pidc/Beta");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcBeta) {
      Serial.print("new Beta: ");
      Serial.println(topiValu);
      pidcBeta = topiValu;
    }  
  }
  topiIndx = topic.indexOf("pidc/Gamma");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcGamma) {
      Serial.print("new Gamma: ");
      Serial.println(topiValu);
      pidcGamma = topiValu;
    }  
  }
  topiIndx = topic.indexOf("userCmdl");
  if (topiIndx >= 0){
    // copy data into user command line
    userCmdl = data;
    Serial.print("User Command: ");
    Serial.println(userCmdl);
    userRctl |= RCTL_ATTN; 
  }  
}

void cbck1000() {
  cb10Rctl |= RCTL_ATTN;
}

void cb10Svce() {
  int rCode = 0;
  //
  dtostrf( pidcRn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )RnTops , (const char * )mqttVals, 15 ); 
  //
  dtostrf( pidcYn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )YnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcEn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )EnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcUn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )UnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcPn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )PnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcDn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )DnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcIn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )InTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( profSecs, 8, 3, mqttVals);
  rCode += popcMqtt.publish( psecTops, (const char * )(mqttVals), 15 );
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
  //
  bbrdPubl(1);
  cb20Rctl &= !RCTL_ATTN;
}

void cbck5000() {
  cb50Rctl |= RCTL_ATTN;
}

void cb50Svce() {
  int rCode = 0;
  dtostrf( millis(), 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )c500Tops, (const char *)(mqttVals), 15 ); 
  //
  dtostrf( pidcKp, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )KpTops,   (const char *)(mqttVals), 15 ); 
  //
  dtostrf( pidcTi, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )TiTops,   (const char *)(mqttVals), 15 ); 
  //
  dtostrf( pidcTd, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )TdTops,   (const char *)(mqttVals), 15 ); 
  //
  if ( userScal == fahrScal) {
    dtostrf( floatCtoF(profTmpC), 8, 3, mqttVals);
  } else {
    dtostrf(           profTmpC,  8, 3, mqttVals);
  }  
  rCode += popcMqtt.publish( (const char * )ptmpTops, (const char *)(mqttVals), 15 ); 
  //
  dtostrf( pwmdPcnt, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )pcntTops, (const char *)(mqttVals), 15 ); 
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
  rCode += popcMqtt.subscribe( KpTops );
  //if ( rCode) {
    //Serial.print("bbrdSubs Kp bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
  rCode += popcMqtt.subscribe( BetaTops );
  //if ( rCode) {
    //Serial.print("bbrdSubs Beta bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
  rCode += popcMqtt.subscribe( GammaTops );
  //if ( rCode) {
    //Serial.print("bbrdSubs Gamma bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(KpTops);
  //}
  //
  rCode = 0;
  rCode += popcMqtt.subscribe( TdTops );
  //if ( rCode) {
    //Serial.print("bbrdSubs Td bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TdTops);
  //}
  //
  rCode = 0;
  rCode += popcMqtt.subscribe( TiTops );
  //
  //if ( rCode) {
    //Serial.print("bbrdSubs Ti bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TiTops);
  //}
  //
  rCode = 0;
  rCode += popcMqtt.subscribe( userTops );
  //
  //if ( rCode) {
    //Serial.print("bbrdSubs Ti bad cuml RC: ");
    //Serial.println(rCode);
    //Serial.println(TiTops);
  //}
  //
}  

void pwmdExpo( byte tensMaxe) {
#if IN_ESP
#else 
  /// PWM Drive
  // For Arduino Uno, Nano, Micro Magician, Mini Driver, Lilly Pad, any ATmega 8, 168, 328 board**
  //---------------------------------------------- Set PWM frequency for D5 & D6 -----------------
  //TCCR0B = TCCR0B & B11111000 | B00000001; // tmr 0 divisor:     1 for PWM freq 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010; // tmr 0 divisor:     8 for PWM freq  7812.50 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000011; //*tmr 0 divisor:    64 for PWM freq   976.56 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000100; // tmr 0 divisor:   256 for PWM freq   244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101; // tmr 0 divisor:  1024 for PWM freq    61.04 Hz
  //---------------------------------------------- Set PWM frequency for D9 & D10 ----------------
  //TCCR1B = TCCR1B & B11111000 | B00000001; // tmr 1 divisor:     1 for PWM freq 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010; // tmr 1 divisor:     8 for PWM freq  3921.16 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000011; //*tmr 1 divisor:    64 for PWM freq   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100; // tmr 1 divisor:   256 for PWM freq   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101; // tmr 1 divisor:  1024 for PWM freq    30.64 Hz
  //------------------------------------------- Set PWM frequency for D3 & D11 -------------------
  //TCCR2B = TCCR2B & B11111000 | B00000001; // tmr 2 divisor:     1 for PWM freq 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010; // tmr 2 divisor:     8 for PWM freq  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011; // tmr 2 divisor:    32 for PWM freq   980.39 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100; //*tmr 2 divisor:    64 for PWM freq   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101; // tmr 2 divisor:   128 for PWM freq   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110; // tmr 2 divisor:   256 for PWM freq   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111; // tmr 2 divisor:  1024 for PWM freq    30.64 Hz
  //
  switch (tensMaxe) {
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
      TCCR1B = TCCR1B & B11111000 | B00000101;
    break;
  }  
#endif  
}    

void pwmdSetF( double newFreq) {
  #if IN_ESP
  //analogWriteFreq(newFreq);
  #else 
  #endif 
  pwmdFreq = int(newFreq);
}

void pwmdInit() {
  pwmdDuty = 0;
  #if IN_ESP
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
      Serial.print("pwmdRctl==0");
      pwmdOutp = 0;
    } else {
      // last run control test has precedence 
      if ( pwmdRctl & RCTL_MSET) {
        pwmdTarg = byte( 250.0 * msetDuty / 100.0 );
      }
      if ( pwmdRctl & RCTL_AUTO) {
        pwmdTarg = byte(pidcUn);
      }
      //Serial.print("pwmdTarg: ");
      //Serial.println(pwmdTarg);
      pwmdOutp = byte(pwmdTarg);
      pwmdPcnt = byte (100.0 * pwmdOutp / 250);
      if ( 0 ) {
        analogWrite( PWMD_OPIN, pwmdOutp);
      } else {
      }
    }  
  }  
}

/// Profile Control
void profInit() {
  // simulation
  sensTmpC = 20.0;
  profTmpC = 20.0;
  profSecs = 0;
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
    // 
    profSecs += 1;
    if (profSele < 0) {
      // ToDo saved profiles
      //Serial.println("profSele");
    } else {
      targTmpC = float(profTmpC);
      if (profCdpm != 0) {
        if (profTmpC >= profTbeg) {
          if ((sensTmpC >=  targTmpC) && (profSecs > 10)) {
            profCdpm = 0;
            profSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     + float (profSecs) * float(profCdpm) / 60.0;
          }  
        } else {
          if ((sensTmpC <= targTmpC) && (profSecs > 10)) {
            profCdpm = 0;
            profSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     - float (profSecs) * float(profCdpm) / 60.0;
          }  
        }
      }  else {
        // ramp==0: target temp  unchanged
        profCdpm = 0;
        profSecs = 0;
        bbrdTmde = bbrdSetp;
        //targTmpC = float(targTmpC)
      }
    }
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
    Serial.print('\n');
    if (pidcRctl & (RCTL_INFO & RCTL_BBRD) ){
      Serial.print(F("profSecs: "));
      Serial.println(profSecs);
    } else {
      // Front End 
    } 
  }
}  
    
/// THERMOCOUPLE
void tcplInit() {
  tcplTogo  = TCPL_POLL_MSEC;       // thermocouple norm poll period mSec
  // stabilize wait .. lcds banner is 1000mA anyway
  // delay(500);
  tcplTogo = PIDC_POLL_MSEC;      // PID control poll period mSec
  tcplPrev =  millis();
  sensTmpC = 20;
}

void tcplVirtLoop() {
  int pwmdMavg; 
  currMSec = millis();
  elapMSec = currMSec - tcplPrev;
  if (( tcplTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    tcplPrev = millis();
    tcplTogo = TCPL_POLL_MSEC;
    // virt tcpl 
    pwmdMavg = int( 0.1 * pwmdOutp    \
                 +  0.2 * pwmdHist[0] \
                 +  0.6 * pwmdHist[1] \
                 +  0.8 * pwmdHist[2] \
                 +  0.4 * pwmdHist[3] ) ; 
    sensTmpC = sensTmpC + float(pwmdMavg) / 250.0 \
                 -  0.5 * (sensTmpC - 20.0) / 64.0;
    pwmdHist[3] = pwmdHist[2] / 4; 
    pwmdHist[2] = pwmdHist[1]; 
    pwmdHist[1] = pwmdHist[0]; 
    pwmdHist[0] = pwmdOutp;
  }  
}    

#if IN_UNO
void tcplRealLoop() {
  double tcplTmpC;
  currMSec = millis();
  elapMSec = currMSec - tcplPrev;
  if (( tcplTogo - POLL_SLOP_MSEC ) > elapMSec ) {
    return;
  } else {
    tcplPrev = millis();
    tcplTogo = TCPL_POLL_MSEC - elapMSec;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      sensTmpC = 20;
    } else {
      // Read thermocouple 
      tcplTmpC = tcpl.readCelsius();
      if (isnan(tcplTmpC)) {
        lcd.clear();
        lcd.home ();
        lcd.print(F("@tcplRealLoop()"));
        lcd.setCursor ( 0, 1 );
        lcd.print(F("Thermocouple Err"));
        delay ( 5000 );                //  1000mS startup delay
        lcd.clear();
      }
     sensTmpC = float( tcplTmpC); 
    }  
  }  
}  
#endif

// User Commands 
void userInit() {
  profSele = 0;
  profCdpm =  0;
  userScal = centScal;
  profTbeg = profTmpC = userDegs = 20;
}

void userLoop() {
  // test for when chars arriving on serial port, set ATTN
  if (Serial.available()) {
    // wait for entire message  .. 115200cps 14 char ~ 1mSec
    delay(1);
    // read all the available characters
    userCmdl = Serial.readStringUntil('\n');
    userRctl |= RCTL_ATTN;
  }
}    

void userSvce() {
  // called from loopp() if RCTL_ATTN via either serial or MQTT    
  if ((userCmdl[0] == 'C') || (userCmdl[0] == 'c')) {
    userScal = centScal;
  }
  if ((userCmdl[0] == 'F') || (userCmdl[0] == 'f')) {
    userScal = fahrScal;
  }
  if ((userCmdl[0] == 'P') || (userCmdl[0] == 'P')) {
    // select stored profile
    profSele = (userCmdl.substring(1)).toInt();
    //Serial.println(F("userfSele"));
    profSecs = 0;
  }
  if ((userCmdl[0] == 'R') || (userCmdl[0] == 'r')) {
    // set desired temp ramp rate degC/min
    profCdpm = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      profCdpm = intgFtoC( profCdpm);
    }
    if (profCdpm < 0)  profCdpm = 0;
    profTbeg = int(sensTmpC);
    profSecs = 0;
    if (profCdpm == 0) bbrdTmde = bbrdSetp;
    //Serial.print(F("\nprofCdpm"));
    //Serial.println(profCdpm);
  }
  if ((userCmdl[0] == 'S') || (userCmdl[0] == 's')) {
    // set desired temperatre degC
    userDegs = (userCmdl.substring(1)).toInt();
    if ( userScal == fahrScal) {
      profTmpC = intgFtoC( userDegs); 
    } else {
      profTmpC = userDegs;
    }
    if (profTmpC > 299) profTmpC = 299;
    profTbeg = int(sensTmpC);
    profSecs = 0;
    pwmdRctl &= ~RCTL_MSET;
    pwmdRctl |=  RCTL_AUTO;
  }
  if ((userCmdl[0] == 'W') || (userCmdl[0] == 'w')) {
    // set new pwmD Width, run control flag to indicate manual override
    msetDuty = (userCmdl.substring(1)).toInt();
    if (msetDuty > 99) msetDuty = 100;
    pwmdRctl &= ~RCTL_AUTO;
    pwmdRctl |=  RCTL_MSET;
    // manual pwm width will apply in pwmd loop 
  }
  if ((userCmdl[0] == 'Y') || (userCmdl[0] == 'y')) {
    // set desired temperatre degC
    Serial.println("Pwm frequency control TBD");
  }
  userRctl &= ~RCTL_ATTN;
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(57600);
  Serial.begin(115200);
  delay(100);
  userInit();
  tcplInit();
  pidcInit();
  pwmdInit();
  profInit();
#if IN_ESP
  if (wifiRctl & RCTL_RUNS) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    //  Wifi Setup 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(4000);
    }
    Serial.println("");
    Serial.println("WiFi conn to IP: ");
    Serial.println(WiFi.localIP());
    //  MQTT Setup 
    //    setup callbacks
    popcMqtt.onConnected(connCbck);
    popcMqtt.onDisconnected(discCbck);
    popcMqtt.onPublished(publCbck);
    popcMqtt.onData(dataCbck);
    Serial.println("Connect to mqtt...");
    popcMqtt.connect();
    delay(4000);
    popcSubs();
    bbrdPubl(3);
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
  lcdsInit();
#endif 
  Serial.println("popcEsp init end");
}

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
  tcplVirtLoop();
  pidcLoop();
  pwmdLoop();
  //
  userLoop();
  if (userRctl & RCTL_ATTN) {
    userSvce();
  }  
  bbrdLoop();
#if IN_ESP
  popcShed.update();
#else 
  lcdsLoop();
#endif  
  profLoop();
  //frntLoop();
  delay(10);
}
