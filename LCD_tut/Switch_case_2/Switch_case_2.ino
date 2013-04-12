//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
/*******************************************************
This program will test the LCD panel and the buttons
Mark Bramwell, July 2010
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
int wait_len = 250;
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
int increment(int num)
{
  num++;
  if (num > 60) num = 0;
  return num;
}
int decrement(int num)
{
  num--;
  if (num < 0) num = 60;
  return num;
}
void printtime(void)
{
  lcd.setCursor(0,1);
    if (minutes > 9)
    {
      lcd.print(minutes);
      lcd.print(":");
    }
    else
    {
      lcd.print("0");
      lcd.print(minutes);
      lcd.print(":");
    }
    if (seconds > 9)
    {
      lcd.print(seconds);
    }
    else
    {
      lcd.print("0");
      lcd.print(seconds);    
    }
}
void setup()
{
lcd.begin(16, 2); // start the library
Serial.begin(9600);
}


void loop()
{
lcd_key = read_LCD_buttons(); // read the buttons
Serial.println(select);
switch (select) 
{
  case 0:
  { 
   lcd.setCursor(0,0);   
   lcd.print("Set Minutes");
   printtime();
   switch (lcd_key)
   {
     case btnUP:
     {
       delay(wait_len);
       minutes = increment(minutes);
       break;
     }
     case btnDOWN:
     {
       minutes = decrement(minutes);
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
       seconds = decrement(seconds);
       break;
     }
     case btnSELECT:
     {
       select = 2;
       break;
     }
   }
    break;
  }
  case 2:
  {
    break;
  }
}
}


