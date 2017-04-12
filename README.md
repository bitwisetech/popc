RoboPopc is an application for coffe roast control and logging. The software supports Arduino
  and ESP8266 development boards with software PID controller and drivers for LCD display. 

    Without additional hardware either platform can connect to  'Artisan' roaster logging
  application, providing 'virtual thermocouple' readings of Bean Temperature to Artisan. 

    With additional hardware 2x16 I2C display board, MAX31855 thermocouple interface and 
  PWM drive for solid state relay are supported.  On the ESP8266 version wifi connection and
  MQTT messaging allows for control via web based applications, a serial interface is necessary
  only for code loading. 

    Control of the roasting process is by any of: Artisan Logger over serial interface, manual
  setting of temperature / time ramps and target setpoints over a serial interface or manually 
  adjusting temperature ramp or PWM percentage by way of a 16 position rotary encoder switch. 

    PWM control may be selected to be either by slow On-Off cycling, 3mSec granularity in 3 Sec
  cycles or, by using the processor's PWM at an approx 31Hz rate.
  
Installation is done using the Arduino IDE, for ESP8266 the supplementary steps for ESP8266 
  development must be done. This code is supplied as an Arduino sketchbook, for ESP8266 the 
  'Ticker Scheduler' code and headers must be copied into this sketch's folder. 
  There are compiler switches near the beginning of the code for processor type and for 
    various hardware options to be included. 
  
  Sections (units) in this code, ordered alphabetically:
  bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
  frnt  deprecated, sends data over Serial to Frontside / Process apps on PC for PID tuning   
  lcds  support for I2C 2x16 LCD display                             
  mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
  offn  Slow Off/On approx 3sec cycle PWN for SSR controlled heater                      
  pidc  PID controller for PWM powered temperature control; a delta-time incremental PID 
  pwmd  8-bit PWM control via hardware pwm pins 
  prof  Profile control; selects auto/manual temp setpt, manual pwm width, real/fake temp sensor
  rots  Rotary 16way encoded ( 4pin + common) selector switch manager
  tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
  user  receive user's or Artisan commands via serial port, MQTT for setpoint, ramp, profiles 
    
  Arduino H/W details:  

  pins:  lcdi: A4 A5; 
      tcplSPI:        D3 D5    D10
          pwm:              D9      
 
  Legend / Mode Indicators; Upcase: User Set; LowCase: Auto/Sensed value; User entry: either case;
  a  toggle run time interface to Artisan 
  c  centigrade entry/display
  d  toggle diagnostic verbose messages on serial 
  f  fahrenhet entry / display, internals always are Centigrade 
  i  info strings and 'billboard' to serial 
  l  logging (artisan csv) to serial 
  p  profile (tbd)
  r  ramp C/F Degrees per minute, enter before target setpoint
  s  setPoint C/F temp 
  w  pwm width override, disssabled PID
  y  tbd pwm freq Hz 
  z  reset total timecount


