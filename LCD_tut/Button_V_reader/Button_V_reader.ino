/* LCD Shield Pushbutton Voltage Reader
   Group 06

 The circuit:
 * LCD RS pin to digital pin 8
 * LCD Enable pin to digital pin 9
 * LCD D4 pin to digital pin 4
 * LCD D5 pin to digital pin 5
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 7
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

// include the library code:
#include <LiquidCrystal.h>
int buttonPin = A0;    // select the input pin for the pushbuttons
int cur_button = 1023;
int state = analogRead(buttonPin);
int last_state = 0;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  Serial.begin(9600); 
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

void loop() {
  lcd.setCursor(0,0); // move to top left corner
  cur_button = debounce(cur_button);
  if (cur_button < 1020) state = cur_button;
  if (last_state != state) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Last key was ");
  lcd.setCursor(0,1);
  lcd.print(state);
  }
  last_state = state;
}
