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
int counter = 1;
double input_times[4] = {130, 250, 320, 420};
double input_temps[2]= {150, 220};
double use_times[5];
double use_temps[5];
//Expected time constant
int tau = 2000;
//What is room temperature in AD counts? last rm_temp cal available at EEPROM 0
int rm_temp = 20;
int theoretical_rm_temp = 20;
//
int select = 0;
//Temporary Variables
int last_updated;
double cur_incr;
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
    Input = read_temp();
    byte val = (aTune.Runtime());
      Serial.println("tuning mode");
      Serial.print("kp: ");Serial.print(heatPID.GetKp());Serial.print(" ");
      Serial.print("ki: ");Serial.print(heatPID.GetKi());Serial.print(" ");
      Serial.print("kd: ");Serial.print(heatPID.GetKd());Serial.println();
      Serial.print("Current Temp: ");
      Serial.print(Input);
      Serial.print("\n");
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

void message(int tempVal, int setpoint)
{
  /*lcd.setCursor(0,0); // set cursor to first column, first row
  lcd.print("Current Temp:");
  lcd.print(tempVal);
  lcd.setCursor(0,1); // set cursor to first column, second row
  lcd.print("Setpoint Temp:");
  lcd.print(setpoint);*/
  Serial.print("Current Temp: ");
  Serial.print(tempVal);
  Serial.print("\n");
  Serial.print("Setpoint Temp: ");
  Serial.print(setpoint);
  Serial.print("\n");
  Serial.print("Control signal: ");
  Serial.print(HotOutput);
  Serial.print("\n");
}
void sendPlotData(String seriesName, float data)
{
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}
/************ MAIN *********/

void setup()
{
  Serial.begin(9600);
  
  //initialize the variables we're linked to
  Input = analogRead(tmpPin);
  Setpoint = 100;
  
  //turn the PID on
  heatPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);
  rm_temp = get_rm_temp();
}

void loop()
{
  switch(select) {
    case 0:
    {
      TuneGains();
      get_use_points();
      last_updated = millis();
      cur_incr = calculate_goal_increment(counter);
      Setpoint = rm_temp+cur_incr;
      select = 1;
      break;
    }
    case 1:
    {
      Input = read_temp();
      
      heatPID.Compute();
      analogWrite(heatPin,HotOutput);
      coolPID.Compute();
      analogWrite(coolPin,CoolOutput);
      
      if(millis()>last_updated+tau)
      {
        last_updated += tau;
        Setpoint += cur_incr;
      }
      if(millis()>use_times[counter])
      {
        counter++;
        cur_incr = calculate_goal_increment(counter);
        if (counter > 4) select = 2;
      }
      message(Input, Setpoint);
      break;
    }
    case 2:
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
