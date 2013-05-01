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
int select = 1;

//Temporary Variables
double last_updated;
double cur_incr;

//Button presses
int lcd_key = 0; // what button is being pressed?
int adc_key_in = 0; // what voltage is being applied to button pin?
int cur_but = btnNONE;
int t_wait = 600; // how long until a pressed button registers again?
double temp_input = 100;

//Fake PWM vars
int WindowSize = 500;
unsigned long windowStartTime;
int print_out = 100;
unsigned long tofnow = 0;
unsigned long tnow = 0;
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

double read_input(double in)
{
  //double out = in;
  //for (int x = 0; x< 1; x++)
  //{
  //  delay(5);
  //  double reader = (double)analogRead(tmpPin);
  //  delay(5);
  //  out = 8*out/10+2*reader/10;
  //}
  double vcc = (double)readVcc();
  double adcvalue = (double)analogRead(tmpPin);
  double out = (adcvalue/5115)*vcc;
  return out;
}

#define NUM_READS 100
double testread(int sensorpin){
   // read multiple values and sort them to take the mode
   int sortedValues[NUM_READS];
   for(int i=0;i<NUM_READS;i++){
     int value = analogRead(sensorpin);
     int j;
     if(value<sortedValues[0] || i==0){
        j=0; //insert at first position
     }
     else{
       for(j=1;j<i;j++){
          if(sortedValues[j-1]<=value && sortedValues[j]>=value){
            // j is insert position
            break;
          }
       }
     }
     for(int k=i;k>j;k--){
       // move all values higher than current reading up one position
       sortedValues[k]=sortedValues[k-1];
     }
     sortedValues[j]=value; //insert current reading
   }
   //return scaled mode of 10 values
   float returnval = 0;
   for(int i=NUM_READS/2-5;i<(NUM_READS/2+5);i++){
     returnval +=sortedValues[i];
   }
   returnval = returnval/10;
   double vcc = (double)readVcc();
   double out = ((double)returnval/5115)*vcc;
   return out;
}

double read_temp()
{
  double ins = 0;
  for (int c = 1; c <257; c++)
  {
    ins = 9*ins/10+(double)analogRead(tmpPin)/10;
  }
  return ins;
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
  /*Serial.print("\n");
  Serial.print("Current Temp: ");
  Serial.print(tempVal);
  Serial.print("\n");
  Serial.print("Setpoint Temp: ");
  Serial.print(setpoint);
  Serial.print("\n");
  Serial.print("Control signal: ");
  Serial.print(HotOutput);
  Serial.print("\n ");*/
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
      if (t_wait > 10) t_wait = t_wait - 10;
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

void fake_PWM(int pin,double in)
{
    if(tnow - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(in < tnow - windowStartTime) {
    print_out = 0;
    digitalWrite(pin,LOW);
  }
  else {
    digitalWrite(pin,HIGH);
    print_out = 100;
  }
  /*
  double value = map(in,0,255,100,400);
  double start = (double)millis();
  digitalWrite(pin,HIGH);
  while (millis()<(start+500))
  {
  if (millis() > start+value) digitalWrite(pin,LOW);
  }*/
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
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
      find_rm_temp();
      print_temp(rm_temp);
      temp_input = increment_var(temp_input, 10, 100);
      break;
    }
    
     case 1:
    {
      Input = round(read_input(Input));
      Serial.println( readVcc(), DEC );
      delay(5);
      //Input = (double)analogRead(tmpPin);
      //delay(500);
      lcd.setCursor(0,0);
      lcd.print(Input);
      plot_stuff();
      temp_input = increment_var(temp_input, 130, 170);
      if (select == 2) {
        Setpoint = 100;
        Input = round(read_temp());
        windowStartTime = millis();
        heatPID.SetOutputLimits(0, WindowSize);
      }
      break;
    }
    
     case 2:
    {
      Input = round(testread(tmpPin));
      tnow = millis();
      heatPID.Compute();
      
      fake_PWM(heatPin,HotOutput);
      
      temp_input = increment_var(temp_input, 10, 255);
      
      plot_stuff();
      message(Input,Setpoint);
      
      if (select == 3) {
        temp_input = 250;
        lcd.clear();
        windowStartTime = millis();
        tofnow = millis();
      }
      break;
    }
     case 3:
    {
      tnow = millis();
      lcd.setCursor(0,0);
      lcd.print("PWM TEST");
      lcd.setCursor(0,1);
      lcd.print(temp_input);
      temp_input = increment_var(temp_input, 0, 500);
      fake_PWM(heatPin,250);
      sendPlotData("State",print_out);
      if (millis()>tofnow+3000) select++;
      break;
    }
  }
  /*Serial.print("Room temp corresponds to: ");
  Serial.print(rm_temp);
  Serial.print("\n");
  delay(1000);*/
  
  /*TuneGains();*/
}
