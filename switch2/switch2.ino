/*
Arduino Tutorials
Episode 2
Switch Program
Written by: Jeremy Blum
Modified by: Group 06
*/

int switchPin = 8;
int ledPin = 13;
boolean lastButton = LOW;
boolean ledOn = false;

void setup()
{
  pinMode(switchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  if (digitalRead(switchPin) == HIGH && lastButton == LOW) // If button went from Low to High
  {
    ledOn = !ledOn; // LED changes state
    lastButton = HIGH;
  }
  else
  {
    //lastButton = LOW;
    lastButton = digitalRead(switchPin); // update lastButton state
  }
  
  digitalWrite(ledPin, ledOn); // update LED state

}
