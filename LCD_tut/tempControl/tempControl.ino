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
int buttonPin = A0;

int lcd_key = 0; // what button is being pressed?
int adc_key_in = 0; // what voltage is being applied to button pin?
int wait_len = 150; // wait to avoid repeat readinsg
boolean running = 1; //should we be pausing?
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
// Improved reading of button state
int debounce(int last)
{
  int current = analogRead(buttonPin); // Current button state
  if (last != current) // If a change happened
  {
    delay(10); // wait for bounce to settle
    current = analogRead(buttonPin); 
  }
  return current;
}
int increment(int num) // wrap around to keep minutes and seconds within 0-59
{
  num++;
  if (num > 300) num = 300;
  return num;
}
int decrement(int num) // opposite of decrement
{
  num--;
  if (num < 60) num = 60;
  return num;
}

int read_temp_val = 0;

int state = analogRead(tempPin);
int voltage = state * (5.0/1023) // convert temperature sensor readings to scaled voltage ( 0 to 5 V)
int tempVal = voltage * (1/.005) // convert scaled voltage to temperature (in degrees C)
int read_temp(int setpoint)
{
  if (tempVal <= setpoint)          return belowSP; 
  if (tempVal > setpoint && < 300)  return aboveSP;
  if (tempVal >= 300)               return overheat;
}
  
void setup() {
  Serial.begin(9600);
  pinMode(heaterPin, OUTPUT);
}
void tempControl(void) {
  read_temp_val = read_temp{};
    switch (read_temp_val) // is the temperature below or above the set point?
    {
      case belowSP: { // case for when temperature is below the set point
      digitalWrite(heaterPin, HIGH)// turn oven on
      break;
      }
      case aboveSP: {// case for when temperature is above the set point and below 300
      digitalWrite(heaterPin, LOW) // turn oven off
      break; 
      }
      case overheat: {// case for when temperature is above 300 degrees
      digitalWrite(heaterPin, LOW) // turn oven off
      lcd.print("Warning! Over 300 Celsius. Turn le oven off. ")
      }
    }
}

void 
void loop() {
  tempControl();
}
