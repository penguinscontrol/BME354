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
int minutes;
int seconds;
boolean select = true;

// Variables
int buttonPin = A0;

int lcd_key = 0;
int last_key = 0;
int adc_key_in = 0;

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
  if num > 60 num = 0;
  return num;
}
int decrement(int num)
{
  num--;
  if num < 0 num = 60;
  return num;
}
void setup()
{
lcd.begin(16, 2); // start the library
Serial.begin(9600);
// Set up the timer
minutes = 0;
timer = 0;
}


void loop()
{
lcd.print("Set Minutes");
lcd.setCursor(0,1);
lcd.blink();
while (select == true) {
  lcd_key = read_LCD_buttons(); // read the buttons
  
  switch (lcd_key) // depending on which button was pushed, we perform an action
  {
    case btnRIGHT:
    {
      //Instructions for what to do on RIGHT button press
      break;
    }
    case btnLEFT:
    {
      //Instructions for what to do on LEFT button press
      break;
    }
    case btnUP:
    {
      //Instructions for what to do on UP button press
      minutes = minutes + 1;
      lcd.setCursor(0,1);
      lcd.print(minutes);
      break;
    }
    case btnDOWN:
    {
      //Instructions for what to do on DOWN button press
      if (minutes > 0) {
        minutes = minutes-1;
        lcd.setCursor(0,1);
        lcd.print(minutes);
      }
      break;
    }
    case btnSELECT:
    {
      //Instructions for what to do on SELECT button press
      break;
    }
    case btnNONE:
    {
      //Instructions for what to do if no button is pressed.
      break;
    }
  }
}

lcd.clear();
lcd.print("Set Timer for Seconds: ");
lcd.setCursor(0,1);
lcd.blink();
lcd_key = read_LCD_buttons(); // read the buttons

switch (lcd_key) // depending on which button was pushed, we perform an action
{
  case btnRIGHT:
  {
    //Instructions for what to do on RIGHT button press
    break;
  }
  case btnLEFT:
  {
    //Instructions for what to do on LEFT button press
    break;
  }
  case btnUP:
  {
    //Instructions for what to do on UP button press
    if (seconds < 60) {
    seconds = seconds + 1;
    lcd.setCursor(0,1);
    lcd.print(seconds);
    }
  }
  case btnDOWN:
  {
    //Instructions for what to do on DOWN button press
    if (seconds > 0) {
      seconds = seconds - 1;
      lcd.setCursor(0,1);
      lcd.print(seconds);
    }
  }
  case btnSELECT:
  {
    //Instructions for what to do on SELECT button press
    break;
  }
  case btnNONE:
  {
    //Instructions for what to do if no button is pressed.
    // do nothing
  }
}

}


