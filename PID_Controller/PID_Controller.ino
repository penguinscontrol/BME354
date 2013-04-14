/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

//Define input pins
int btnPin = 0;
int tmpPin = 1;

//Define output pins
int heatPin = 3;
int coolPin = 4;

//Define Variables we'll be connecting to
double Setpoint, Input, HotOutput, CoolOutput;

//Specify the links and initial tuning parameters
PID heatPID(&Input, &HotOutput, &Setpoint,2,5,1, DIRECT);
PID coolPID(&Input, &CoolOutput, &Setpoint,2,5,1, REVERSE);

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(tmpPin);
  Setpoint = 100;

  //turn the PID on
  heatPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(tmpPin);
  HeatPID.Compute();
  analogWrite(heatPin,HotOutput);
  CoolPID.Compute();
  analogWrite(coolPin,CoolOutput);
}


