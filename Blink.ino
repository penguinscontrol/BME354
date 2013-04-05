/*
Tutorial 1: Blinking LED
*/

int ledPin = 13;

void setup()
{
  pinMode(ledPin, OUTPUT); // initialize pin as outputs
}

void loop()
{
  digitalWrite(ledPin, HIGH);  // LED on
  delay(1000);  // set period of blink
  digitalWrite(ledPin, LOW);  // LED off
  delay(1000);
}
