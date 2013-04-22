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
int btnPin = 0;
int tmpPin = 5;

//Define output pins
int heatPin = 2;
int coolPin = 11;

//Define Variables we'll be connecting to
double Setpoint, Input, HotOutput, CoolOutput;

//Define Variables for AutoTuner
byte ATuneModeRemember=2;
double kp=6,ki=0.1,kd=0;
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
double input_temps[2]= {100, 150};
double use_times[5];
double use_temps[5];

//Expected time constant
double tau = 2000;

//What is room temperature in AD counts? last rm_temp cal available at EEPROM 0
double rm_temp = 20;
double theoretical_rm_temp = 20;

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
      plot_stuff();
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

double get_rm_temp()
{
  return (double)EEPROM.read(0)*4;
}

double read_temp()
{
  double ins[8];
  for (int c = 0; c <8; c++)
  {
    ins[c] = (double)analogRead(tmpPin);
  }
  return (ins[0]+ins[1]+ins[2]+ins[3]+ins[4]+ins[5]+ins[6]+ins[7])/8;
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
  /*lcd.setCursor(0,0); // set cursor to first column, first row
  lcd.print("Current Temp:");
  lcd.print(tempVal);
  lcd.setCursor(0,1); // set cursor to first column, second row
  lcd.print("Setpoint Temp:");
  lcd.print(setpoint);*/
  Serial.print("\n");
  Serial.print("Current Temp: ");
  Serial.print(tempVal);
  Serial.print("\n");
  Serial.print("Setpoint Temp: ");
  Serial.print(setpoint);
  Serial.print("\n");
  Serial.print("Control signal: ");
  Serial.print(HotOutput);
  Serial.print("\n ");
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

// Improved reading of button state
int debounce(int last)
{
  int current = analogRead(btnPin); // Current button state
  if (last != current) // If a change happened
  {
    delay(10); // wait for bounce to settle
    current = analogRead(btnPin);
  }
  return current;
}

void waitforrelease(int t){
  cur_but = read_LCD_buttons();
  int t_press = millis();
  while (cur_but != btnNONE){
    cur_but = read_LCD_buttons();
    int cur_t = millis();
    if(cur_t > (t_press + t)) {
      if (t_wait > 100) t_wait = t_wait - 100;
      break;
    }
  }
}

int read_LCD_buttons()
{ 
adc_key_in = debounce(adc_key_in); // read the value from the sensor
if (adc_key_in > VNONE) return btnNONE; // We make this the 1st option for speed reasons since
if (adc_key_in < V1) return btnRIGHT;
if (adc_key_in < V2) return btnUP;
if (adc_key_in < V3) return btnDOWN;
if (adc_key_in < V4) return btnLEFT;
if (adc_key_in < V5) return btnSELECT;
return btnNONE; // when all others fail, return this...
}

void print_temp(int temp){
  lcd.setCursor(0,1);
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
  
double increment_var(double out, double l_lim, double r_lim)
{
      lcd_key = read_LCD_buttons();
      //Serial.println(lcd_key);
      if (lcd_key == btnNONE) {
        t_wait = 600;
        return out;
      }
      waitforrelease(t_wait);
      switch (lcd_key){
        case btnUP:
        {
          if (out < r_lim) out++;
          break;
        }
        case btnDOWN:
        {
          if (out > l_lim) out--;
          break;
        }
        case btnRIGHT:
        {
          lcd.clear();
          select++;
          break;
        }
         case btnLEFT:
        {
          lcd.clear();
          select--;
          break;
        }
      }
      return out;
}
/************ MAIN *********/

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2); // start the LCD
  
  //initialize the variables we're linked to
  Input = read_temp();
  Setpoint = rm_temp;
  rm_temp = get_rm_temp();
  
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
      lcd.print("Soak Temp?:");
      print_temp(input_temps[0]);
      input_temps[0] = increment_var(input_temps[0], 130, 170);
      break;
    }
     case 1:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Soak?:");
      print_temp(input_temps[0]);
      input_times[0] = increment_var(input_times[0], input_temps[0]/3, input_temps[0]/1);
      if (select == 2) input_times[1] = input_times[0]+30;
      break;
    }
     case 2:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Ramp?:");
      print_temp(input_times[1]);
      input_times[1] = increment_var(input_times[1], input_times[0]+60, input_times[0]+120);
      break;
    }
     case 3:
    {
      lcd.setCursor(0,0);
      lcd.print("Peak Temp:");
      print_temp(input_temps[1]);
      input_temps[1] = increment_var(input_temps[1], 200, 240);
      if (select == 4)  input_times[2] = input_times[1]+(input_temps[1]-input_temps[0])/3;
      break;
    }
     case 4:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Peak?:");
      print_temp(input_times[2]);
      input_times[2] = increment_var(input_times[2],input_times[1]+(input_temps[1]-input_temps[0])/3,input_times[1]+(input_temps[1]-input_temps[0])/1);
      if (select == 5) input_times[3] = input_times[2]+(input_temps[1]-rm_temp)/4;
      break;
    }
    case 5:
    {
      lcd.setCursor(0,0);
      lcd.print("Time to Cool?:");
      print_temp(input_times[3]);
      input_times[3] = increment_var(input_times[3],input_times[2]+(input_temps[1]-rm_temp)/4,input_times[2]+(input_temps[1]-rm_temp)/2);
      break;
    }
    case 6:
    {
      //TuneGains();
      get_use_points();
      last_updated = millis();
      cur_incr = calculate_goal_increment(counter);
      Setpoint = rm_temp+cur_incr;
      select++;
      break;
    }
    case 7:
    {
      Input = read_temp();
      heatPID.Compute();
      analogWrite(heatPin,(int)HotOutput);
      coolPID.Compute();
      analogWrite(coolPin,CoolOutput);
      
      plot_stuff();
      //message(Input, Setpoint);
      
      if((double)millis() > last_updated+tau)
      {
        last_updated = (double)millis();
        Setpoint += cur_incr;
      }
      
      if((double)millis() > use_times[counter])
      {
        counter++;
        cur_incr = calculate_goal_increment(counter);
        if (counter > 4) select++;
      }
      break;
    }
    case 8:
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
