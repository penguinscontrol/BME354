/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

//Define input pins
int btnPin = 0;
int tmpPin = 1;

//Define output pins
int heatPin = 3;
int coolPin = 5;

//Define Variables we'll be connecting to
double Setpoint, Input, HotOutput, CoolOutput;

//Define Variables for AutoTuner
byte ATuneModeRemember=2;
double kp=2,ki=0.5,kd=2;
double outputStart=50;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long serialTime;

//Specify the links and initial tuning parameters
PID heatPID(&Input, &HotOutput, &Setpoint,kp,ki,kd, DIRECT);
PID coolPID(&Input, &CoolOutput, &Setpoint,2,5,1, REVERSE);
PID_ATune aTune(&Input, &HotOutput);

//Reflow curve variables
int counter = 0;
double input_times[4];
double input_temps[2];
double use_times[5];
double use_temps[5];
//Expected time constant
double tau;
//What is room temperature in AD counts? last rm_temp cal available at EEPROM 0
int rm_temp = 20;
int theoretical_rm_temp = 20;

//Temporary Variables
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
    byte val = (aTune.Runtime());    
    if (val!=0)
    {
      tuning = false;
    }
    analogWrite(heatPin,HotOutput);
  }
  //out of loop? we're done, set the tuning parameters
  kp = aTune.GetKp();
  ki = aTune.GetKi();
  kd = aTune.GetKd();
  tau = aTune.GetPu();
  heatPID.SetTunings(kp,ki,kd);
  AutoTuneHelper(false);
}

void find_rm_temp()
{
  rm_temp = analogRead(tmpPin);
  int addr = 0;
  EEPROM.write(addr, rm_temp/4);
}

int get_rm_temp()
{
  return EEPROM.read(0)*4;
}

int read_temp()
{
  return analogRead(tmpPin)-rm_temp+theoretical_rm_temp;
}

void get_use_points()
{
  use_temps[0] = get_rm_temp();
  use_times[0] = millis();
  use_temps[1] = input_temps[0];
  use_times[1] = millis()+input_times[0]*1000;
  use_temps[2] = input_temps[0];
  use_times[2] = millis()+input_times[1]*1000;
  use_temps[3] = input_temps[1];
  use_times[3] = millis()+input_times[2]*1000;
  use_temps[4] = get_rm_temp();
  use_times[4] = millis()+input_times[3]*1000;
}

double calculate_goal_increment(int counter)
{
  double next_time = use_times[counter];
  double last_time = use_times[counter-1];
  double next_temp = use_temps[counter];
  double last_temp = use_temps[counter-1];
  double dti = next_time-last_time;
  double dte = next_temp-last_temp;
  return 1000*dte/dti;
}

void next_goal()
{
  ;
}
/************MAIN*********/

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  Input = analogRead(tmpPin);
  Setpoint = 100;
  //turn the PID on
  heatPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);
}

void loop()
{
  rm_temp = get_rm_temp();
  
  Serial.print("Room temp corresponds to: ");
  Serial.print(rm_temp);
  Serial.print("\n");
  delay(1000);
  
  /*TuneGains();
  Input = read_temp();
  heatPID.Compute();
  analogWrite(heatPin,HotOutput);
  coolPID.Compute();
  analogWrite(coolPin,CoolOutput);*/
}
