/* Temperature Controller Objective 1 and 2
   Group 06
   */
  

// include the library code:
#include <LiquidCrystal.h>

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

int tempPin = A1;    // select the input pin for the temperature sensor
int heaterPin = D13;   // select the output pin for the heater
int setpoint = 100;
int belowSP = 0;
int aboveSP = 1;
int overheat = 2;

int read_temp_val = 0;
int tempVal;

int read_temp()
{
  tempVal = analogRead(tempPin);
  if (tempVal <= setpoint)          return belowSP; 
  if (tempVal > setpoint && < 300)  return aboveSP;
  if (tempVal >= 300)               return overheat;
}
  
void setup() {
  Serial.begin(9600);
  pinMode(heaterPin, OUTPUT);
  lcd.begin(16, 2); // start the library
}
  
void loop() {
  read_temp_val = read_temp{};
    switch (read_temp_val) // is the temperature below or above the set point?
    {
      case belowSP: { // case for when temperature is below the set point
      digitalWrite(heaterPin, HIGH); // turn oven on
      break;
      }
      case aboveSP: {// case for when temperature is above the set point and below 300
      digitalWrite(heaterPin, LOW); // turn oven off
      break; 
      }
      case overheat: {// case for when temperature is above 300 degrees
      digitalWrite(heaterPin, LOW); // turn oven off
      lcd.print("Warning! Over 300 Celsius. Turn le oven off.");
      }
    }
}
