/*
Arduino Tutorials
Episode 2
Switch3 Program (debounced)
Written by: Jeremy Blum
Modified by: Group 06
*/

// Initialize variables
int switchPin = 8;
int ledPin = 13;
boolean lastButton = LOW;
boolean currentButton = LOW;
boolean ledOn = false;

void setup()
{
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

// Improved reading of button state
boolean debounce(boolean last)
{
  boolean current = digitalRead(switchPin); // Current button state
  if (last != current) // If a change happened
  {
    delay(5); // wait for bounce to settle
    current = digitalRead(switchPin); 
  }
  return current;
}

void loop()
{
  currentButton = debounce(lastButton);
  if (lastButton == LOW && currentButton == HIGH) // if a change happened
  {
    ledOn = !ledOn; // flip LED state
  }
  lastButton = currentButton; // remember last state
  
  analogWrite(ledPin, ledLevel); // update LED

}
