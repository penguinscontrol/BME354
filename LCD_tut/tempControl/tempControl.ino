//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
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
/*******************************************************
This program will test the LCD panel and the buttons
Mark Bramwell, July 2010
Modified by Group 06, April 2013

USER MANUAL
The device begins with reading in minutes and seconds for the timer.
1. To set minutes, increment and decrement the value using UP and DOWN.
2. When done, press SELECT to go to seconds selection.
3. UP and DOWN to select seconds
4. Press SELECT again to start timer.
5. The RIGHT button toggles the timer on/off
6. When time is 00:00, the world commences to burn.
******************************************************/
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// define some values used by the panel and buttons
byte flame1[8] = {
        B00100,
        B01100,
        B01010,
        B01010,
        B10011,
        B11011,
        B11111,
        B01110
};

byte flame2[8] = {
        B00100,
        B00110,
        B01110,
        B01010,
        B11001,
        B11001,
        B11111,
        B01110
};
// Variables
int buttonPin = A0;
int tempPin = A1;
int setpoint = 90;
int tempVal = analogRead(tempPin);
int lcd_key = 0; // what button is being pressed?
int adc_key_in = 0; // what voltage is being applied to button pin?
int cur_but = btnNONE;
int wait_len = 150; // wait to avoid repeat readings

boolean running = 1; //should we be pausing?

int select = 0;
/*
0 = temperature selection;
1 = running the controller;
*/
// read the buttons
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

void waitforrelease(void){
  cur_but = read_LCD_buttons();
  int t_press = millis();
  while (cur_but != btnNONE){
    cur_but = read_LCD_buttons();
    int cur_t = millis();
    if(cur_t > (t_press + 100)) break;    
    //Serial.println("looping");
  }
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


void setup()
{
  lcd.begin(16, 2); // start the library
  Serial.begin(9600);
  lcd.createChar(1, flame1);
  lcd.createChar(2, flame2);
}


void message(int temp, int setpoint)
{
  lcd.setCursor(0,0); // set cursor to first column, first row
  lcd.print("Current T:");
  lcd.print(temp);
  lcd.setCursor(0,1); // set cursor to first column, second row
  lcd.print("Setpoint T:");
  lcd.print(setpoint);
}

void loop()
{
  switch(select)
  {
    case 0:
    {
      lcd.setCursor(0,0);
      lcd.print("Desired T:");
      print_temp(setpoint);
      lcd_key = read_LCD_buttons();
      //Serial.println(lcd_key);
      if (lcd_key == btnNONE) {
        break;
      }
      waitforrelease();
      switch (lcd_key){
        case btnUP:
        {
          if (setpoint < 250) setpoint++;
          break;
        }
        case btnDOWN:
        {
          if (setpoint > 25) setpoint--;
          break;
        }
        case btnSELECT:
        {
          select = 1;
          break;
        }
      }
      break;
    }
    case 1:
    {
      lcd.clear();
      select = 2;
      break;
      
    }
    case 2:
    {
      message(tempVal, setpoint);
      break;
    }
  }
}


