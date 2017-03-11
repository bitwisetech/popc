/// popcEsp popc port to esp with MQTT, tickScheduler, is MQTT client popc
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
//  rctl  Run time control; selects auto/manual temp setpt, pwm width, real/fake temp sensor
//  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
//  user  receive user's commands via serial port ( and MQTT ?? ) for setpoint, ramp, profile modes 
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
// popcShed ingo MQTT withtick, scheduler 
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
char scalCent = 'C';
char scalFahr = 'F';

String userLin0("exactly thirty one chars length");  // Crashable ! 

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
#define TOPS_BASE "/popc"
char mqttVals[] =  "                ";                     // mqtt value string 15 char max
const char KpTops[]   = "/popc/pidc/Kp";
const char TdTops[]   = "/popc/pidc/Td";
const char TiTops[]   = "/popc/pidc/Ti";
const char RnTops[]   = "/popc/pidc/Rn";
const char YnTops[]   = "/popc/pidc/Yn";
const char UnTops[]   = "/popc/pidc/Un";
const char psecTops[] = "/popc/profSecs";
const char pcntTops[] = "/popc/pwmd/perCent";
const char ptmpTops[] = "/popc/profTmpC";
const char c500Tops[] = "/popc/cbck5000";

// create MQTT object with IP address, port of MQTT broker e.g.mosquitto application
//ab  MQTT myMqtt(MQCL_ID, "192.168.0.1", 1883);
//  nettles MQTT popcMqtt(MQCL_ID, "192.168.1.16", 5983);
//  abby
MQTT popcMqtt(MQCL_ID, "172.20.224.111", 5983);

//pidc
int   pidcVbse    = 0;                    // 1: bbrd 2: DbugLine + Publ 
float pidcRn      =  20.000;              // Refr setpoint
float pidcYn      =  20.000;              // YInp input
float pidcKp      =   2.000;              // P-Term gain
float pidcTd      =   0.010;              // D-Term Gain sec ( Td++ = Gain++)
float pidcTi      =   2.000;              // I-Term Gain sec ( Ti++ = Gain--)
float pidcBeta    =   1.000;              // P-term Refr vs YInp
float pidcGamm    =   0.100;              // D-term Refr vs YInp
float alpha       =   0.100;              // D-term Filter time
float uMax        = 250.000;              // Outp Max
float uMin        =   0.000;              // Outp Min
float dUn, Edn, Epn, Epn1            = 0; // Calc error values
float Edfn2, Edfn1, Edfn             = 0; // Calc error values
float Tf, Ts, TsDivTf                = 0; // Filter, Actual sample period, stash
float pidcPc, pidcIc, pidcDc, pidcEn = 0; // Calc delta OP, P-I-D Terms
float Un  = 0;                            // OP for current sample
float Un1 = 0;                            // Save OP from prev sample
float pidcUn = 0;                         // PID controller Output

// pwmd vbls
int  pwmdFreq, pwmdDuty, pwmdTarg, pwmdOutp;
int  pwmdHist[] = { 0, 0, 0, 0};
byte pwmdPcnt;

// 
#define TCPL_POLL_USEC  250000L            // 250mS termocouple poll
#define PWMD_POLL_USEC  250000L            // 250mS termocouple poll
#define PIDC_POLL_USEC  250000L            // 250mS pid control poll
#define LCDS_POLL_USEC 2000000L            // 2Sec  lcd display poll
#define RCTL_POLL_USEC 1000000L            // 1Sec  run control poll
#define OPEN_POLL_USEC    1000L            // Open window for jitter

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
byte  rctlRctl  = RCTL_RUNS;
byte  tcplRctl  = (RCTL_RUNS | RCTL_VIRT);
byte  userRctl  = RCTL_RUNS;
byte  wifiRctl  = RCTL_RUNS;
// scheduler tick callback attn flags 
byte  cb05Rctl  = RCTL_RUNS;
byte  cb10Rctl  = RCTL_RUNS;
byte  cb20Rctl  = RCTL_RUNS;
byte  cb30Rctl  = RCTL_RUNS;
byte  cb50Rctl  = RCTL_RUNS;


#if IN_ESP
//pubsub
void dataCbck(String& topic, String& data);
void publCbck();
void discCbck();
void connCbck();

#include "TickerScheduler.h"
TickerScheduler popcShed(6);

//  time
unsigned long uSecCurr;
unsigned int  uSecOflo;
unsigned long lcdsPrev, pidcPrev, rctlPrev, pwmdPrev, tcplPrev;
         long lcdsTogo, pidcTogo, rctlTogo, pwmdTogo, tcplTogo, tempLong;
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

double tcplTmpC;

//
float         userInpu, targTmpC, sensTmpC;
int           profSele, profTmpC, profCdpm, profTbeg, profSecs;
int           userCdpm, msetPwmd, userTmpC, tempIndx; 

/// Common 
///   Convert
float floatCtoF( float celsInp) {
  return (float( ((celsInp + 40) * 9L / 5L )  -40 ));
}  

float floatFtoC( float fahrInp) {
  return (float( ((fahrInp + 40) * 5L / 9L )  -40 ));
}  
  
///   Timing 
unsigned long uSecElap( unsigned long *callPrev, unsigned long *callElap) {
  unsigned long uSecCurr = micros();
  unsigned long uSecPrev = *callPrev;
  unsigned long uSecElap;
  unsigned int  uSecOflo;
  if ( uSecCurr > uSecPrev) {
    uSecElap = ( uSecCurr - uSecPrev);
    uSecOflo = 0;
  } else {
    uSecElap = (( 0xFFFFFFFF - uSecPrev) + uSecCurr  + 1);
    uSecOflo = 1;
  }
  *callPrev = uSecCurr;
  *callElap = uSecElap;
  return ( uSecOflo);
}

long uSecPast( unsigned long uSecLast) {
  uSecCurr = micros();
  if ( uSecCurr >= uSecLast) {
    uSecOflo = 0;
    return (uSecCurr - uSecLast);
  } else {
    uSecOflo = 1;
    return ( 0xFFFFFFFF - uSecLast) + uSecCurr  + 1;
  }
}

// billboard
void bbrdFill() {
  // strf ops append null chars, fill single chars later 
  dtostrf( sensTmpC, 3, 0, &bbrdLin0[0] );
  dtostrf( targTmpC, 3, 0, &bbrdLin0[7] );
  dtostrf( pwmdPcnt, 3, 0, &bbrdLin0[12]);
  //
  bbrdLin0[3]  = 'C';
  bbrdLin0[4]  = '-';
  bbrdLin0[5]  = '>';
  bbrdLin0[6]  = 'S';
  bbrdLin0[10] = ' ';
  //ldsLin0[10] = 'c';
  //bbrdLin0[11] = ' ';
  bbrdLin0[11] = 'W';
  bbrdLin0[15] = '%';
  //
  dtostrf( profTmpC, 3, 0, &bbrdLin1[1] );
  dtostrf( profCdpm, 3, 0, &bbrdLin1[6] );
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
  // maybe do not re-publish here 
  cbck0500();  
  cbck3000();  
  //
}  

void bbrdSubs( byte tTyp) {
  Serial.print("bbrdSubs tTyp: ");
  Serial.println(tTyp);
  boolean rCode = 0;
  rCode += popcMqtt.subscribe( KpTops );
  //if ( rCode) {
    //Serial.print("bbrdSubs Kp bad cuml RC: ");
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
    //tcplTmpC=double(frntComm.asFloat[1]); // * the user has the ability to send the 
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
  Serial.print(tcplTmpC);   
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
  lcdsTogo  = LCDS_POLL_USEC;     // lcd display    poll period uSec
  lcdsPrev  = micros();
}

void lcdsLoop() {
  tempLong = uSecPast( lcdsPrev);
  if (( lcdsTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    lcdsPrev = uSecCurr;
    // in templong: uSec since last service
    lcdsTogo = 2 * LCDS_POLL_USEC - tempLong;
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
//   dUn =  Kp * (ep_n - ep_n-1)
//            + ((Ts/Ti) *  e_n)
//            + ((Td/Ts) * (edf_n - 2*edf_n-1 + edf_n-2) )
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
//        Ed = pidcGamm * Rn -Yn
//       Edf: Deriv error with reference weighing and filtering
//       Edfn = Efn1 / ((Ts/Tf) + 1) + Edn * (Ts/Tf) / ((Ts/Tf) + 1)
//         where:
//         Tf : Filter time
//         Tf = alpha * Td , where alpha usually is set to 0.1
///

//
void pidcInit() {
  Tf = alpha * pidcTd;
  Epn1 = 0.0;
  Edfn2 = Edfn1 = Edfn = 0;
  // first time being enabled, seed with current property tree value
  Un1 = Un = 0;
  pidcTogo = PIDC_POLL_USEC;      // PID control poll period uSec
  pidcPrev = micros();
  targTmpC = 20;
}

//
void pidcLoop() {
  tempLong = uSecPast( pidcPrev);
  if (( pidcTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    pidcPrev = uSecCurr;
    // in templong: uSec since last service
    pidcTogo = 2 * PIDC_POLL_USEC - tempLong;
    //
    if ( pidcRctl & RCTL_RUNS  == 0 ) {
      // Poll/Thermocouple == 0 Shutdown
      Un = 0;
    } else {
      Ts = (PIDC_POLL_USEC - (float)pidcTogo) / 1000000.0; // sample interval (Sec)
      pidcTogo += PIDC_POLL_USEC;
      // 
      pidcRn = targTmpC;
      pidcYn = sensTmpC;
      // P term 
      pidcEn  = pidcRn - pidcYn;
      Epn = pidcBeta * pidcRn - pidcYn;
      // D term 
      if ( pidcTd <= 0 ) {
        Edfn2 = Edfn1 = Edfn = 0;
      } else {
        Edn = pidcGamm * pidcRn - pidcYn;
        // Filter the derivate error:
        Tf = alpha * pidcTd;
        TsDivTf = Ts / Tf;
        Edfn = Edfn1 / (TsDivTf + 1)
               + Edn * (TsDivTf) / (TsDivTf + 1);
      }
      // Accum Combine P, I, D terms 
      dUn = 0;
      pidcPc = pidcKp *  (Epn - Epn1);
      if ( pidcTi > 0.0 ) {
        pidcIc = pidcKp * ((Ts / pidcTi) * pidcEn);
      } else {
        pidcIc = 0;
      }    
      if ( pidcTd > 0.0 ) {
        pidcDc = pidcKp * ((pidcTd / Ts) * (Edfn - (2 * Edfn1) + Edfn2));
      } else {
        pidcDc = 0;
      }  
      dUn    = pidcPc + pidcIc + pidcDc;
      // Integrator anti-windup logic:
      if ( dUn > (uMax - Un1) ) {
        dUn = uMax - Un1;
        if (pidcRctl & (RCTL_INFO | RCTL_BBRD) ) {
          Serial.println(F("maxSatn"));
        }  
      } else if ( dUn < (uMin - Un1) ) {
        dUn = uMin - Un1;
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
  //Serial.print("dataCbck topic:data  ");
  //Serial.print(topic);
  //Serial.print(":");
  //Serial.println(data);
  topiIndx = topic.indexOf("pidc/Rn");
  if (topiIndx >= 0){
    topiValu = data.toFloat();
    if ( topiValu != pidcRn) {
      Serial.print("new Rn: ");
      Serial.println(topiValu);
      pidcRn = topiValu;
    }  
  }
  topiIndx = topic.indexOf("pidc/Kp");
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
}

void cbck0500() {
  cb05Rctl |= RCTL_ATTN;
}

void cb05Svce() {
  boolean rCode = 0;
  // publish write parms
  dtostrf( pidcRn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )RnTops , (const char * )mqttVals, 15 ); 
  //
  dtostrf( pidcYn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )YnTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcUn, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )UnTops, (const char * )(mqttVals), 15 ); 
  //
  //if ( rCode) {
    //Serial.print("cbck0500 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  //
  cb05Rctl &= ~RCTL_ATTN;
}

void cbck1000() {
  cb10Rctl |= RCTL_ATTN;
}

void cb10Svce() {
  boolean rCode = 0;
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
  boolean rCode = 0;
  dtostrf( pwmdPcnt, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )pcntTops, (const char *)(mqttVals), 15 ); 
  //
  //if ( rCode) {
    //Serial.print("cbck2000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  //
  bbrdPubl(1);
  cb20Rctl &= !RCTL_ATTN;
}

void cbck3000() {
  cb30Rctl |= RCTL_ATTN;
}

void cb30Svce() {
  boolean rCode = 0;
  dtostrf( pidcKp, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )KpTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcTi, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )TiTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( pidcTd, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )TdTops, (const char * )(mqttVals), 15 ); 
  //
  dtostrf( profTmpC, 8, 3, mqttVals);
  rCode += popcMqtt.publish( (const char * )ptmpTops, (const char * )(mqttVals), 15 ); 
  //
  //if ( rCode) {
    //Serial.print("cbck3000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  cb30Rctl &= ~RCTL_ATTN;
}

void cbck5000() {
  cb30Rctl |= RCTL_ATTN;
}

void cb50Svce() {
  String valueStr("alive");
  boolean rCode = popcMqtt.publish( c500Tops, valueStr);
  //
  //if ( rCode) {
    //Serial.print("cbck5000 bad cuml RC: ");
    //Serial.println(rCode);
  //}
  cb50Rctl &= ~RCTL_ATTN;
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
  pwmdTogo  = PWMD_POLL_USEC;       // pwm drvr poll period uSec
  pwmdPrev  = micros();             // pwm drvr poll period uSec
}

void pwmdLoop() {
  tempLong = uSecPast( pwmdPrev);
  if (( pwmdTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    //Serial.print("pwmdPoll");
    pwmdPrev = uSecCurr;
    // in templong: uSec Poll + late uSec
    pwmdTogo = 2 * PWMD_POLL_USEC - tempLong;
    //
    if (pwmdRctl == 0) {
      //Serial.print("pwmdRctl==0");
      pwmdOutp = 0;
    } else {
      //?? pwmdTogo -= uSecPast(pwmdPrev);
      //Serial.print("pwmdOPOLL");
      //?? if (pwmdTogo <= OPEN_POLL_USEC ) {
        //Serial.print("pwmd");
        pwmdPrev = uSecCurr;
        pwmdTogo += PWMD_POLL_USEC;
        // last run control test has precedence 
        if ( pwmdRctl & RCTL_MSET) {
          pwmdTarg = byte(msetPwmd);
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
      //??}
    }  
  }  
}

/// Run Control
void rctlInit() {
  // simulation
  tcplTmpC = 20.0;
  profTmpC = 20.0;
  profSecs = 0;
  rctlTogo = RCTL_POLL_USEC;   //  poll period uSec
  rctlPrev = micros();
}

void rctlLoop() {
  tempLong = uSecPast( rctlPrev);
  if (( rctlTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    rctlPrev = uSecCurr;
    // in templong: uSec since last service
    rctlTogo = 2 * RCTL_POLL_USEC - tempLong;
    // 
    profSecs += 1;
    if (profSele < 0) {
      // ToDo saved profiles
      //Serial.println("profSele");
    } else {
      targTmpC = float(profTmpC);
      if (profCdpm != 0) {
        if (profTmpC >= profTbeg) {
          if ((tcplTmpC >=  targTmpC) && (profSecs > 10)) {
            profCdpm = 0;
            profSecs = 0;
            bbrdTmde = bbrdHold;
          } else {
            bbrdTmde = bbrdRamp;
            targTmpC = float(profTbeg) \
                     + float (profSecs) * float(profCdpm) / 60.0;
          }  
        } else {
          if ((tcplTmpC <= targTmpC) && (profSecs > 10)) {
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
  tcplTogo  = TCPL_POLL_USEC;       // thermocouple norm poll period uSec
  // stabilize wait .. lcds banner is 1000mA anyway
  // delay(500);
  tcplTogo = PIDC_POLL_USEC;      // PID control poll period uSec
  tcplPrev = micros();
  tcplTmpC = sensTmpC = 20;
}

void tcplVirtLoop() {
  int pwmdMavg; 
  tempLong = uSecPast( tcplPrev);
  if (( tcplTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    tcplPrev = uSecCurr;
    // in templong: uSec since last service
    tcplTogo = 2 * TCPL_POLL_USEC - tempLong;
    // virt tcpl 
    pwmdMavg = int( 0.1 * pwmdOutp    \
                 +  0.2 * pwmdHist[0] \
                 +  1.0 * pwmdHist[1] \
                 +  4.0 * pwmdHist[2] \
                 +  4.0 * pwmdHist[3] ) ; 
    tcplTmpC = tcplTmpC + float(pwmdMavg) / 250.0 \
                 -  1 * (tcplTmpC - 20.0) / 40.0;
    sensTmpC = tcplTmpC;
    pwmdHist[3] = pwmdHist[2] / 4; 
    pwmdHist[2] = pwmdHist[1]; 
    pwmdHist[1] = pwmdHist[0]; 
    pwmdHist[0] = pwmdOutp;
  }  
}    

#if IN_UNO
void tcplRealLoop() {
  tempLong = uSecPast( tcplPrev);
  if (( tcplTogo - OPEN_POLL_USEC ) > tempLong ) {
    return;
  } else {
    tcplPrev = uSecCurr;
    // in templong: uSec Poll + late uSec
    tcplTogo = 2 * TCPL_POLL_USEC - tempLong;
    //
    if (tcplRctl== 0) {
      // Rctl == 0 Shutdown
      tcplTmpC = 19;
    } else {
      tcplTogo -= uSecPast(tcplPrev);
      if (tcplTogo <= OPEN_POLL_USEC ) {
        tcplPrev = uSecCurr;
        tcplTogo += TCPL_POLL_USEC;
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
      }  
    }  
  }  
}  
#endif

///
void userInit() {
  userInpu = 0.0;
  profSele = 0;
  profCdpm = userCdpm =  0;
  profTbeg = profTmpC = userTmpC = 20;
}

void userLoop() {
  // when characters arrive over the serial port...
  if (Serial.available()) {
    // wait for entire message  .. 9600cps 10 char ~ 1mSec
    delay(1);
    // read all the available characters
    userLin0 = Serial.readStringUntil('\n');
    if ((userLin0[0] == 'P') || (userLin0[0] == 'P')) {
      // select stored profile
      profSele = (userLin0.substring(1)).toInt();
      //Serial.println(F("userfSele"));
      profSecs = 0;
    }
    if ((userLin0[0] == 'R') || (userLin0[0] == 'r')) {
      // set desired temp ramp rate degC/min
      profCdpm = userCdpm = (userLin0.substring(1)).toInt();
      profTbeg = int(tcplTmpC);
      profSecs = 0;
      if (profCdpm == 0) bbrdTmde = bbrdSetp;
      //Serial.print(F("\nuserCdpm"));
      //Serial.println(profCdpm);
    }
    if ((userLin0[0] == 'S') || (userLin0[0] == 's')) {
      // set desired temperatre degC
      userTmpC = (userLin0.substring(1)).toInt();
      if (userTmpC > 250) userTmpC = 250;
      profTmpC = userTmpC;
      profTbeg = int(tcplTmpC);
      profSecs = 0;
      pwmdRctl &= ~RCTL_MSET;
      pwmdRctl |=  RCTL_AUTO;
    }
    if ((userLin0[0] == 'W') || (userLin0[0] == 'w')) {
      // set new pwmD Width
      msetPwmd = (userLin0.substring(1)).toInt();
      if (msetPwmd > 99) userTmpC = 100;
      msetPwmd = int ( 250 * msetPwmd / 100.0 );
      pwmdRctl &= ~RCTL_AUTO;
      pwmdRctl |=  RCTL_MSET;
      // manual pwm width will apply in pwmd loop 
    }
    if ((userLin0[0] == 'Y') || (userLin0[0] == 'y')) {
      // set desired temperatre degC
      userTmpC = (userLin0.substring(1)).toInt();
      if (userTmpC > 250) userTmpC = 250;
      profTmpC = userTmpC;
      profTbeg = int(tcplTmpC);
      profSecs = 0;
      //Serial.print(F("\nuserTmpC"));
      //Serial.println(profTmpC);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(57600);
  Serial.begin(115200);
  delay(100);
  userInit();
  tcplInit();
  pidcInit();
  //pwmdInit();
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
    delay(1000);
    bbrdPubl(3);
    bbrdSubs(1);
  }  

  //  Tick setup 
  
  //  Sked setup
  //    TickerScheduler(uint size);
  //    boolean add(uint i, uint32_t period, tscallback_t f, boolean shouldFireNow = false);
  //      ts.add(0, 3000, sendData)
  // 
  int shedRcod;
  if (! popcShed.add( 0,  500, cbck0500)) {
    Serial.print("popcShed add cbck0500 RC: ");
    Serial.println(shedRcod);
  }
  shedRcod = popcShed.add( 1, 1000, cbck1000);
  shedRcod = popcShed.add( 2, 2000, cbck2000);
  shedRcod = popcShed.add( 3, 3000, cbck3000);
  shedRcod = popcShed.add( 4, 5000, cbck5000);
  
#else 
  lcdsInit();
#endif 
  rctlInit();
  Serial.println("popcEsp init end");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (cb05Rctl & RCTL_ATTN) {
    cb05Svce();
  }  
  if (cb10Rctl & RCTL_ATTN) {
    cb10Svce();
  }  
  if (cb20Rctl & RCTL_ATTN) {
    cb20Svce();
  }  
  if (cb30Rctl & RCTL_ATTN) {
    cb30Svce();
  }  
  if (cb50Rctl & RCTL_ATTN) {
    cb50Svce();
  }  
  tcplVirtLoop();
  pidcLoop();
  pwmdLoop();
  //
  userLoop();
  bbrdLoop();
#if IN_ESP
  popcShed.update();
#else 
  lcdsLoop();
#endif  
  rctlLoop();
  //frntLoop();
  delay(10);
}
