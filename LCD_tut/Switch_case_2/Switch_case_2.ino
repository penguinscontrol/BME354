//Sample using LiquidCrystal library
#include <LiquidCrystal.h>
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
// Timer Variables
int minutes = 0;
int seconds = 0;
int select = 0;
/*select = 0 Min selection
  select = 1 Sec selection
  select = 2 Run*/

// Variables
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
  seconds = decrement(seconds);
  if (seconds == 59) minutes = decrement(minutes);
}
void setup()
{
lcd.begin(16, 2); // start the library
Serial.begin(9600);
    lcd.createChar(1, flame1);
    lcd.createChar(2, flame2);
}


void loop()
{
lcd_key = read_LCD_buttons(); // read the buttons
   if (lcd_key == btnRIGHT) //&& last_button != btnRIGHT) 
   {
     running = !running; // toggle stopwatch on or off
     delay(200); // Frank
     Serial.print("lcd_key is: ");
     Serial.println(lcd_key);
     Serial.print("Running is: ");
     Serial.println(running);
   }
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
   while (running) { // if ON, time passes

   int timer = millis();
   int check = millis();
   while(check < timer + 1000) {           
       Serial.print("Timer is: ");
       Serial.println(timer);
       Serial.print("Check is: ");
       Serial.println(check);
     check = millis();
     lcd_key = read_LCD_buttons(); // read the buttons
     if (lcd_key == btnRIGHT)
       {
       running = !running;
       delay(wait_len); // toggle stopwatch on or off
       break;
       }
   }
   time_passes();
   printtime();
   if (minutes == 0 && seconds == 0) {
     select = 3;
     break;
   }
   }
   break;
  }
  case 3: //VICTORY!
  {
    lcd.write(1); // FLAMES!~!!!
    lcd.write(2);
    delay(50);
  }
}
}


