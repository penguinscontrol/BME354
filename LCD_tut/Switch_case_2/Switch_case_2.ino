//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
/*******************************************************
This program will test the LCD panel and the buttons
Mark Bramwell, July 2010
Modified by Group 06, April 2013
********************************************************/
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// define some values used by the panel and buttons

// Timer Variables
int minutes = 0;
int seconds = 0;
int select = 0;
/*select = 0 Min selection
  select = 1 Sec selection
  select = 2 Run*/

// Variables
int buttonPin = A0;

int lcd_key = 0;
int adc_key_in = 0;
int wait_len = 200; // wait to avoid repeat readinsg
boolean running = 1; //should we be pausing
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
int increment(int num) // wrap around to keep minutes and seconds within 0-59
{
  num++;
  if (num > 59) num = 0;
  return num;
}
int decrement(int num) // opposite of decrement
{
  num--;
  if (num < 0) num = 59;
  return num;
}
void printtime(void) // print the current minutes:seconds combo in the bottom left
{
  lcd.setCursor(0,1);
    if (minutes > 9) //if it takes 2 spaces to print
    {
      lcd.print(minutes);
      lcd.print(":");
    }
    else // otherwise pad with a 0
    {
      lcd.print("0");
      lcd.print(minutes);
      lcd.print(":");
    }
    if (seconds > 9) // same for seconds
    {
      lcd.print(seconds);
    }
    else
    {
      lcd.print("0");
      lcd.print(seconds);    
    }
}
void time_passes() // increment time while in RUN mode
{
  delay(1000);
  seconds = decrement(seconds);
  if (seconds == 59) minutes = decrement(minutes);
}
void setup()
{
lcd.begin(16, 2); // start the library
Serial.begin(9600);
}


void loop()
{
lcd_key = read_LCD_buttons(); // read the buttons
switch (select) // which stage are we at?
{
  case 0: // MIN selection mode
  { 
   lcd.setCursor(0,0);   
   lcd.print("Set Minutes");
   printtime();
   switch (lcd_key)
   {
     case btnUP: // increment minutes
     {
       delay(wait_len);
       minutes = increment(minutes);
       break;
     }
     case btnDOWN: //decrement minutes
     {
       delay(wait_len);
       minutes = decrement(minutes);
       break;
     }
     case btnSELECT: // go to next stage
     {
       delay(wait_len);
       select = 1;
       break;
     }
   }
   break;
  }
  case 1: // analogous to minute selection, but for seconds
  {
   lcd.setCursor(0,0);   
   lcd.print("Set Seconds");
   printtime();
   switch (lcd_key)
   {
     case btnUP:
     {
       delay(wait_len);
       seconds = increment(seconds);
       break;
     }
     case btnDOWN:
     {
       delay(wait_len);
       seconds = decrement(seconds);
       break;
     }
     case btnSELECT:
     {
       delay(wait_len);
       select = 2;
       lcd.clear(); 
       break;
     }
   }
    break;
  }
  case 2: // Running mode
  {
   lcd.setCursor(0,0);  
   lcd.print("GO!");
   if (running) {
   time_passes();
   printtime();
   }
   if (lcd_key == btnRIGHT) 
   {
     delay(wait_len);
     running = !running;
   }
   break;
  }
}
}


