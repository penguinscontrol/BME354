/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
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
int btnPin = A0;
int tmpPin = A5;

//Define output pins
int heatPin = 2;
int coolPin = 11;

//Define Variables we'll be connecting to
double Setpoint, Input, HotOutput, CoolOutput;

//Define Variables for AutoTuner
byte ATuneModeRemember=2;
double kp=10,ki=0.1,kd=5;
double outputStart=100;
double aTuneStep=100, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long serialTime;

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
double tau = 2000;

//What is room temperature in AD counts? last rm_temp cal available at EEPROM 0
double rm_temp = 20;
double theoretical_rm_temp = 22;

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
double temp = 100;

//Fake PWM vars
int WindowSize = 500;
unsigned long windowStartTime;
int print_out = 100;
unsigned long tofnow = 0;
unsigned long now = 0;
/**************** FUNCTIONS *********************************/
void autoTuneSetup()
{ //Set the output to the desired starting frequency.
  HotOutput = aTuneStartValue;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  aTune.SetControlType(1);
  AutoTuneHelper(true);
  tuning = true;
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
  autoTuneSetup();
  while (tuning)
  {
    now = millis();
    Input = read_input(Input);
    byte val = (aTune.Runtime());
      //Serial.println("tuning mode");
      //Serial.print("kp: ");Serial.print(heatPID.GetKp());Serial.print(" ");
      //Serial.print("ki: ");Serial.print(heatPID.GetKi());Serial.print(" ");
      //Serial.print("kd: ");Serial.print(heatPID.GetKd());Serial.println();
      plot_stuff();
    if (val!=0)
    {
      tuning = false;
    }
    fake_PWM(heatPin,HotOutput);
  }
  //out of loop? we're done, set the tuning parameters
  kp = aTune.GetKp();
  ki = aTune.GetKi();
  kd = aTune.GetKd();
  tau = aTune.GetPu();
  EEPROM.write(0, aTune.GetKp());
  EEPROM.write(1, aTune.GetKi());
  EEPROM.write(2, aTune.GetKd());
  heatPID.SetTunings(kp,ki,kd);
  AutoTuneHelper(false);
}

void find_rm_temp()
{
  rm_temp = read_temp();
  int addr = 0;
  EEPROM.write(addr, rm_temp/4);
}

double get_rm_temp()
{
  return (double)EEPROM.read(0)*4;
}

void get_use_points()
{
  use_temps[0] = get_rm_temp();
  use_times[0] = (double)millis();
  use_temps[1] = input_temps[0];
  use_times[1] = (double)millis()+input_times[0]*1000;
  use_temps[2] = input_temps[0];
  use_times[2] = (double)millis()+input_times[1]*1000;
  use_temps[3] = input_temps[1];
  use_times[3] = (double)millis()+input_times[2]*1000;
  use_temps[4] = get_rm_temp();
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

void message(double tempVal, double setpoint)
{
  lcd.setCursor(0,0); // set cursor to first column, first row
  lcd.print("Temp:");
  lcd.print(tempVal);
  lcd.setCursor(0,1); // set cursor to first column, second row
  lcd.print("Set Temp:");
  lcd.print(setpoint);
}

void sendPlotData(String seriesName, double data)
{
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}

void plot_stuff()
{
  sendPlotData("Temperature",Input);
  sendPlotData("Error",heatPID.GetError());
  sendPlotData("SetPoint",Setpoint);
  sendPlotData("Output",HotOutput);
}

void print_temp(int temp,int x, int y){
  lcd.setCursor(x,y);
    if (temp > 99)
    {
      lcd.print(temp);
    }
    else
    {
      lcd.print("0");
      lcd.print(temp);
    }
  }
  
void fake_PWM(int pin,double in)
{
    if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(in < now - windowStartTime) {
    print_out = 0;
    digitalWrite(pin,LOW);
  }
  else {
    digitalWrite(pin,HIGH);
    print_out = 100;
  }
}

void heating_print(){
  print_temp(Input,0,1);
  lcd.setCursor(3,0);
  lcd.print('/');
  //print_temp(input_temps[counter]);
  lcd.setCursor(0,1);
  //lcd.print(Stages[counter]);      
}
/************ MAIN *********/

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2); // start the LCD
  //find_rm_temp();
  
  //initialize the variables we're linked to
  Input = read_temp();
  Setpoint = rm_temp;
  rm_temp = get_rm_temp();
  
  //Output pins
   pinMode(heatPin, OUTPUT);
   pinMode(coolPin, OUTPUT);
   pinMode(tmpPin,INPUT);
   
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
      lcd.print("Getting room temp");
      rm_temp = round(read_input(rm_temp));
      print_temp(rm_temp,0,1);
      temp = increment_var(temp, 10, 100);
      sendPlotData("Temperature",rm_temp);
      if (select == 1) {
        Setpoint = 100;
        Input = round(read_temp());
        windowStartTime = millis();
        heatPID.SetOutputLimits(0, WindowSize);
        //select++;
      }
      break;
    }
    
     case 1:
    {
      Input = read_input(Input);
      now = millis();
      heatPID.Compute();
      
      fake_PWM(heatPin,HotOutput);
      
      temp = increment_var(temp, 10, 255);
      
      plot_stuff();
      heating_print();
      
      if (select == 2) {
        temp = 250;
        lcd.clear();
        windowStartTime = millis();
        tofnow = millis();
      }
      break;
    }
     case 2:
    {
      now = millis();
      lcd.setCursor(0,0);
      lcd.print("PWM TEST");
      lcd.setCursor(0,1);
      lcd.print(temp);
      temp = increment_var(temp, 0, 500);
      fake_PWM(heatPin,250);
      sendPlotData("State",print_out);
      if (millis()>tofnow+35000) select++;
      break;
    }
  }
  /*Serial.print("Room temp corresponds to: ");
  Serial.print(rm_temp);
  Serial.print("\n");
  delay(1000);*/
  
  /*TuneGains();*/
}
