/*
Arduino Tutorials
Episode 2
Switch1 Program
Written by: Jeremy Blum
Modified by: Group 06
*/

int switchPin = 8;
int ledPin = 13;

void setup()
{
  pinMode(switchPin, INPUT); // Button on/off state
  pinMode(ledPin, OUTPUT); // LED state
}

void loop()
{
  if (digitalRead(switchPin) == HIGH) // If button pressed
  {
    digitalWrite(ledPin, HIGH); // Turn LED on
  }
  else
  {
    digitalWrite(ledPin, LOW); // If button not pressed, LED off
  }
}
