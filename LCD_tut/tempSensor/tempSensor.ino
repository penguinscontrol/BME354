/* Temperature Controller Objective 1 and 2
   Group 06
   */
  

// include the library code:
#include <LiquidCrystal.h>
#define belowSP 0
#define aboveSP 1
#define overheat 2
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int tempPin = 1;    // select the input pin for the temperature sensor
int heaterPin = 13;   // select the output pin for the heater
int setpoint = 100;

int read_temp_val = 0;
int tempVal;

int read_temp(int setpoint)
{
  tempVal = analogRead(tempPin);
  if (tempVal <= setpoint)          return belowSP; 
  if (tempVal > setpoint && tempVal < 300)  return aboveSP;
  if (tempVal >= 300)               return overheat;
}
void temp_control(int setpoint)
{
    read_temp_val = read_temp(setpoint);
    switch (read_temp_val){ // is the temperature below or above the set point?
      case belowSP:  // case for when temperature is below the set point
      {
      digitalWrite(heaterPin, HIGH); // turn oven on
      break;
      }
      case aboveSP: // case for when temperature is above the set point and below 300
      {
      digitalWrite(heaterPin, LOW); // turn oven off
      break; 
      }
      case overheat: // case for when temperature is above 300 degrees
      {
      digitalWrite(heaterPin, LOW); // turn oven off
      lcd.print("Warning! Over 300 Celsius. Turn le oven off.");
      break; 
      }
    }
} 

void setup() {
  Serial.begin(9600);
  pinMode(heaterPin, OUTPUT);
  lcd.begin(16, 2); // start the library
}

void loop() {
temp_control(setpoint);
}
