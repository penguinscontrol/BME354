/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM HotOutput 3
 ********************************************************/

#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <LiquidCrystal.h>

//Define button voltage ranges
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5
#define V1 20
#define V2 119
#define V3 276
#define V4 430
#define V5 660
#define VNONE 1003

//Initialize LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//Define input pins
int btnPin = 0;
int tmpPin = 5;

//Define HotOutput pins
int heatPin = 2;
int coolPin = 11;

//Define Variables we'll be connecting to
double Setpoint, Input, HotOutput, CoolOutput;

//Define Variables for AutoTuner
byte ATuneModeRemember=2;
double kp=12,ki=0.1,kd=4;
double HotOutputStart=200;
double aTuneStep=200, aTuneNoise=5, aTuneStartValue=250;
unsigned int aTuneLookBack=20;
boolean tuning = false;

//Specify the links and initial tuning parameters
PID heatPID(&Input, &HotOutput, &Setpoint,kp,ki,kd, DIRECT);
PID coolPID(&Input, &CoolOutput, &Setpoint,2,5,1, REVERSE);
PID_ATune aTune(&Input, &HotOutput);

//Reflow curve variables
int counter = 1;
double input_times[4] = {130, 250, 320, 420};
double input_temps[2]= {150, 220};
double use_times[5];
double use_temps[5];

//Expected time constant
double tau = 7500;

//What is room temperature in AD counts? last rm_temp cal available at EEPROM 0
double rm_temp = 20;

// Stage selector
int select = 0;

//Temporary Variables
double last_updated;
double cur_incr;

//Button presses
int lcd_key = 0; // what button is being pressed?
int adc_key_in = 0; // what voltage is being applied to button pin?
int cur_but = btnNONE;
int t_wait = 600; // how long until a pressed button registers again?

//Auto Tune?
boolean shouldtune;

//LCD out
char* Stages[]={"Ramp", "Soak", "Peak","Cooling"};

//Fake PWM vars
int WindowSize = 500;
unsigned long windowStartTime;
unsigned long now = 0;
/**************** FUNCTIONS *********************************/
void changeAutoTune()
{
 if(!tuning)
  {
    //Set the HotOutput to the desired starting frequency.
    HotOutput=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    aTune.SetControlType(1);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = heatPID.GetMode();
  else
    heatPID.SetMode(ATuneModeRemember);
}

void TuneGains()
{
  now = millis();
  if(tuning)
  {
    byte val = (aTune.Runtime());
    sendPlotData("OUT",HotOutput);
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      
      EEPROM.write(0, 10*kp);
      EEPROM.write(1, 10*ki);
      EEPROM.write(2, 10*kd);
      
      heatPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else {
    heatPID.Compute();
    select++;
  }
  
  fake_PWM(heatPin,HotOutput);
}

void get_use_points()
{
  use_temps[0] = rm_temp;
  use_times[0] = (double)millis();
  use_temps[1] = input_temps[0];
  use_times[1] = (double)millis()+input_times[0]*1000;
  use_temps[2] = input_temps[0];
  use_times[2] = (double)millis()+input_times[1]*1000;
  use_temps[3] = input_temps[1];
  use_times[3] = (double)millis()+input_times[2]*1000;
  use_temps[4] = rm_temp;
  use_times[4] = (double)millis()+input_times[3]*1000;
}

double calculate_goal_increment(int coun)
{
  double next_time = use_times[coun];
  double last_time = use_times[coun-1];
  double next_temp = use_temps[coun];
  double last_temp = use_temps[coun-1];
  double dti = next_time-last_time;
  double dte = next_temp-last_temp;
  return tau*dte/dti;
}

void fake_PWM(int pin,double in)
{
    if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(in < now - windowStartTime) {
    digitalWrite(pin,LOW);
  }
  else {
    digitalWrite(pin,HIGH);
  }
}
/************ MAIN *********/

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2); // start the LCD
  
  //initialize the variables we're linked to
  Input = read_temp();
  
  rm_temp = 20;
  Setpoint = rm_temp;
  
  //Output pins
   pinMode(heatPin, OUTPUT);
   pinMode(coolPin, OUTPUT);
  
  //turn the PID on
  heatPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);
}

void loop()
{
  switch(select) {
    case 0:
    {
      lcd.setCursor(0,0);
      lcd.print("Soak Temp?");
      print_temp(input_temps[0],0,1);
      input_temps[0] = increment_var(input_temps[0], 130, 170);
      break;
    }
     case 1:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Soak?");
      print_temp(input_times[0],0,1);
      input_times[0] = increment_var(input_times[0], (input_temps[0]/3), (input_temps[0]/1));
      if (select == 2) input_times[1] = input_times[0]+90;
      break;
    }
     case 2:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Ramp?");
      print_temp(input_times[1],0,1);
      input_times[1] = increment_var(input_times[1], input_times[0]+60, input_times[0]+120);
      break;
    }
     case 3:
    {
      lcd.setCursor(0,0);
      lcd.print("Peak Temp?");
      print_temp(input_temps[1],0,1);
      input_temps[1] = increment_var(input_temps[1], 200, 240);
      if (select == 4)  input_times[2] = input_times[1]+(input_temps[1]-input_temps[0])/2;
      break;
    }
     case 4:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Peak?");
      print_temp(input_times[2],0,1);
      input_times[2] = increment_var(input_times[2],input_times[1]+(input_temps[1]-input_temps[0])/3,input_times[1]+(input_temps[1]-input_temps[0])/1);
      if (select == 5) input_times[3] = input_times[2]+(input_temps[1]-rm_temp)/3;
      break;
    }
    case 5:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Cool?");
      print_temp(input_times[3],0,1);
      input_times[3] = increment_var(input_times[3],input_times[2]+(input_temps[1]-rm_temp)/4,input_times[2]+(input_temps[1]-rm_temp)/2);
      break;
    }
    case 6:
    {
      shouldi();
      if (select == 7 && !shouldtune){ 
        select++;
      }
      else if (select == 7 && shouldtune) {
        windowStartTime = millis();
        heatPID.SetOutputLimits(0, WindowSize);
          tuning=false;
          changeAutoTune();
          tuning=true;
      }
      break;
    }
    case 7:
    {
      lcd.print("Wait for it...");
      TuneGains();
      //delay(3000);
      //lcd.clear();
      //select++;
      break;
    }
    case 8:
    {
      if (shouldtune){
      lcd.print("DONE! Starting...");
      delay(2000);
      lcd.clear();
      }
      get_use_points();
      last_updated = millis();
      windowStartTime = millis();
      heatPID.SetOutputLimits(0, WindowSize);
      cur_incr = calculate_goal_increment(counter);
      Setpoint = rm_temp+cur_incr;
      Input = read_temp();
      select++;
      break;
    }
    case 9:
    {
      now = millis();
      Input = read_input(Input);
      heatPID.Compute();
      fake_PWM(heatPin,HotOutput);
      coolPID.Compute();
      analogWrite(coolPin,CoolOutput);
      
      plot_stuff();
      heating_print();
      
      if(now > last_updated+tau)
      {
        last_updated = (double)millis();
        Setpoint += cur_incr;
      }
      
      if(now > use_times[counter])
      {
        counter++;
        cur_incr = calculate_goal_increment(counter);
        last_updated = (double)millis();
        if (counter > 4) select++;
      }
      break;
    }
    case 10:
    {
      analogWrite(heatPin,0);
      analogWrite(coolPin,0);
      break;
    }
  }
  
  /*Serial.print("Room temp corresponds to: ");
  Serial.print(rm_temp);
  Serial.print("\n");
  delay(1000);*/
  
  /*TuneGains();*/
}
