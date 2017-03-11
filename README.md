   popcEsp A work in progress uses ESP8266 to drive PID controlled popcorn coffee roaster 


    popcEsp popc port to esp with MQTT, tickScheduler, is MQTT client popc
    Mr01 Combine User serial command input with subs callback for user commands  
    Fe22 Add section descriptions, re-order     
    Fe20 redo callback to 500mS, 1sec, 2sec, 5sec and tune pidc
    Oc29-popcFEpid pid controller with lcd output, timed loops and simulated temp response
         lcd display has profile values output on second line  
         serial data format is compatible with 'Front End' graph display on PC 
   
    Sections (units) in this code, ordered alphabetically:
    bbrd  'billboard' posts info to either / both of 2x16 LCD display / Serial Port
    frnt  deprecated, sends data over Serial to Frontside / Process apps on PC for PID tuning   
    lcds  support for I2C 2x16 LCD display ( was working on Nano )     
    mqtt  ref ingo MQTT message queuing publish / subscribe protocol for control via PC MQTT client  
    pidc  PID controller for PWM powered temperature control; a delta-time incremental PID 
    popc  Publish, Subscribe, Callback, Scheduler functions for MQTT /Ticker Scheduler
    pwmd  8-bit PWM control via hardware pwm pins 
    rctl  Run time control; selects auto/manual temp setpt, pwm width, real/fake temp sensor
    tcpl  MAX31855 SPI thermocouple temperature sensor or virtual temp readings for debug
    user  receive user's commands via serial port ( and MQTT ?? ) for setpoint, ramp, profile modes 
      
    Backport to Arduino Nano is untested  Arduino H/W details:  
   
    nano: Ser:0,1  ExtInt:2 PWM:9,10(490HzTmr1) 3,11(490HzTmr2) PWM:5,6 (980HzTmr0 + mS, delay)
          SPI:10,11,12,13 I2C SDA:A4 SCL:A5 LED:13
  
    pins:  lcdi: A4 A5; 
        tcplSPI:        D3 D5    D10
            pwm:              D9      
   include libs i2c, lcd, tcpl
   
    defns for user / actions 
    c  cent 
    f  fahr 
    p  profile
    r  ramp
    s  setPt temp 
    t  ?? time temp
    w  pwm width
    y  pwm freq Hz 
  
