// **** INCLUDE libraries *****
#include "LowPower.h"
#include <Stepper.h>
#include "RTClib.h"
#include <Wire.h>

//////////---------------------------------------------------------------------------/////////////////////////// 
//Pump and stepper motor setup
//----------------------------------------------------------------------------------///////////////////////////

#define STEPS 2038         // the number of steps in one revolution of your motor (28BYJ-48)
#define relay_switch 7     // switch for power to motor control boards
#define relay_power 12     // switch for power to motor control boards
const int pump_power = 13; // switch for power to motor control boards
Stepper stepper(STEPS, 8, 10, 9, 11); // pins connected to stepper motor control board
int In3 = 4;               // control pump on forwards
int In4 = 5;               // control pump on backwards
int EnB = 6;               // control pump speed

//******USER EDIT***************//

int purge = 12000; ////edit  purge time - input required in ms
int sample_pump = 26000; ////edit for sample volume - input required in ms

//******USER EDIT***************//

/////------------------------------------------------------------------------///////////
///setup temperature sensor parameters
#define THERMISTORPIN A0   // which analog pin for Temp measurement      
#define THERMISTORNOMINAL 10000   // resistance at 25 degrees C      
#define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5 // N for average
#define BCOEFFICIENT 3950  // The beta coefficient of the thermistor (usually 3000-4000)r
#define SERIESRESISTOR 10000    // the value of the 'other' resisto
 
int samples[NUMSAMPLES];


////------------------------------------------------------------------------/////////
////Time and sampling frequency set up 
///----------------------------------------------------------------------////////////

RTC_DS1307 rtc; 
DateTime now;
int count = 0;
//uint32_t start_time = 0;
//uint32_t delay_time = 60; ///delay start time from now in secs
uint32_t next_sample;
uint32_t now_u;

//******USER EDIT***************//

uint32_t frequency_r = 120; ///sampling frequency in seconds 30 min = 1800, 15 min = 900, 10 min =600
uint32_t frequency_f = 180; ///sampling frequency in seconds 30 min = 1800, 15 min = 900
int rise = 6;// number of samples to collect for rising limb i.e. higher frequency

//******USER EDIT***************//

//////////--------------------------------------------------------------------//////////
//Float switch setup

const int wakeUpPin = 2;   //initializing pin 2 as float switch input

////------------------------------------------------------------------------/////////
///Stepper home function setup

int homeButton  = 3; // limit switch wired into pin 3
byte hBval;


//////////--------------------------------------------------------------------//////////
//SETUP LOOP
/////////---------------------------------------------------------------//////////////

void setup()
{
   Serial.begin(9600); 
    Serial.println("Pump sampler program nano_V1.0 uploaded");
   rtc.begin();                                    // start clock
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // set clock to time laptop time
   if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }
  //Set status of pins
    pinMode(relay_switch, OUTPUT);
    pinMode(relay_power, OUTPUT);
    pinMode(pump_power, OUTPUT);
    pinMode(In3, OUTPUT);
    pinMode(In4, OUTPUT);
    pinMode(EnB, OUTPUT);
    pinMode(homeButton, INPUT);
    pinMode(pump_power, OUTPUT);
    pinMode(wakeUpPin, INPUT_PULLUP); 
     
    //Serial.println("setup");
    delay(1000);
    //switch to motors set to low
    digitalWrite(relay_power, LOW);
    digitalWrite(relay_switch, LOW);
    digitalWrite(pump_power, LOW);
    //Serial.println("low");
    delay(1000);
    digitalWrite(relay_power, HIGH);
    //Serial.println("power high");
    stepperHome();   // run the stepper back to the limit switch
    delay(300);
    //Serial.println("sleep");
    digitalWrite(relay_power, LOW);

  ///print sampling freq
  Serial.println("Sampling frequency rising limb (s):");
  Serial.println(frequency_r);
  Serial.println("Sampling frequency falling limb (s):");
  Serial.println(frequency_f);
  Serial.println();
  Serial.println();

  ///print csv headers
  Serial.print("TimeStamp, ");
  Serial.print("Temperature, ");
  Serial.print("Note, ");
  Serial.println("Sample event");
    
}

void loop() 
{
    // Enter power down state for 8 s with ADC and BOD module disabled

  
   if(count == 0)
  {
  WaitForFlow();
  }
  
    mainSampleCycle();
    delay(500);
    
}

//----------------------------------------------------------------------
//***************************************************
//*************SUB ROUTINES*************************
///-----------------------------------------------------------------------
void goForward()   //run pump forward for purge time interval
{
  // turn on motor A
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(200);
  analogWrite(EnB, 255);
  delay(purge);
  // now turn off motors
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW); 
}
///-------------------------
void goBackwards()   //run pump backwards for purge time interval
{
  // turn on motor A
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  delay(200);
  analogWrite(EnB, 255);
  delay(purge);
  // now turn off motors
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW); 
}
///------------------------------------------------------
void stepperHome(){ //this routine will run the motor back to the limit switch
  //Serial.println("stepperhome");
  delay(1000);
  hBval = digitalRead(homeButton);
  while (hBval == 1)
  {
    //backwards slowly till it hits the switch and stops 
    
    stepper.setSpeed(3); // 1 rpm
    stepper.step(-5); // do 2038 steps -- corresponds to one revolution in one minute
    //Serial.println("H");
    hBval = digitalRead(homeButton);
   
  }
  delay(300);
  //Serial.println("L");
  stepper.setSpeed(2); // 1 rpm
  stepper.step(50); // do 2038 steps -- corresponds to one revolution in one minute

}
//////------------------------------------------------------------
void MeasureTemp(){
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  
  Serial.print(steinhart);
  Serial.print(",");
}

///-------------------------------------------
void purgeTube()   //purge before each sample is collected
{

goBackwards();
goForward();
goBackwards();
goForward();
goBackwards();
  
}

///-------------------------------------------

void pumpSample()   //run pump for length of time required to collect 50 ml
{

  // turn on motor A
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(200);
  analogWrite(EnB, 255);
  delay(sample_pump);//value set above
  // now turn off motors
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW); 
  
}

///----------------------------------------------

void PrintTime(){
  DateTime now = rtc.now();
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println(",");
    
}

///----------------------------------------------------------------------------------------
void wakeUp()
{
    // Just a handler for the pin interrupt.
}



////----------------------------------------------------------------------------------------

void WaitForFlow()
{
    delay(500);
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, LOW);
   
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0); 
    delay(300);
    PrintTime();
    MeasureTemp();
    count = count + 1; 
    Serial.print("Flow detected");
    Serial.print(", ");
    Serial.println(count);
    DateTime now = rtc.now();
    now_u = now.unixtime();
    DateTime next_s = (now + frequency_r);
    next_sample = next_s.unixtime();
    digitalWrite(relay_power, HIGH);
    delay(100);
    digitalWrite(pump_power, HIGH);
    purgeTube(); 
    pumpSample(); 
    stepper.setSpeed(3); // 1 rpm
    stepper.step(145); // do 2038 steps -- corresponds to one revolution in one minute
    delay(2000);
    digitalWrite(relay_power, LOW);
    digitalWrite(pump_power, LOW);
    Serial.println(next_sample);
    Serial.println(now_u);
  }


///----------------------------------------------------------------------------------------
void mainSampleCycle(){
if((now_u == next_sample) && (count <= rise)){
    PrintTime();
    MeasureTemp();
    count=count+1;
    DateTime now = rtc.now();
    DateTime next_s = (now + frequency_f);
    next_sample = next_s.unixtime();
    Serial.print("Rising limb");
    Serial.print(", ");
    Serial.println(count);
    digitalWrite(relay_power, HIGH);
    delay(100);
    digitalWrite(pump_power, HIGH);
    Serial.println("powerHigh");
    purgeTube(); 
    pumpSample(); 
    stepper.setSpeed(3); // 1 rpm
    stepper.step(145); // do 2038 steps -- corresponds to one revolution in one minute
    delay(2000);
    digitalWrite(relay_power, LOW);
    digitalWrite(pump_power, LOW);
  }  
  else if((now_u == next_sample) && ( rise <= count && count <= 12)){
    PrintTime();
    MeasureTemp();
    count=count+1;
    DateTime now = rtc.now();
    DateTime next_s = (now + frequency_f);
    next_sample = next_s.unixtime();
    Serial.print("Falling limb");
    Serial.print(", ");
    Serial.println(count);
    digitalWrite(relay_power, HIGH);
    delay(100);
    digitalWrite(pump_power, HIGH);
    Serial.println("powerHigh");
    purgeTube(); 
    pumpSample(); 
    stepper.setSpeed(3); // 1 rpm
    stepper.step(145); // do 2038 steps -- corresponds to one revolution in one minute
    delay(2000);
    digitalWrite(relay_power, LOW);
    digitalWrite(pump_power, LOW);

}
 else if(count == 13){
    PrintTime();
    count=count+1;
    MeasureTemp();
    digitalWrite(relay_power, HIGH);
    digitalWrite(pump_power, HIGH);
    Serial.println("powerHigh");
    purgeTube(); 
    pumpSample(); 
    stepper.setSpeed(3); // 1 rpm
    stepper.step(145); // do 2038 steps -- corresponds to one revolution in one minute
    digitalWrite(relay_power, LOW);
    digitalWrite(pump_power, LOW);
    Serial.print("Prog end");
    Serial.print(", ");
    Serial.println(count);
    
}

}
