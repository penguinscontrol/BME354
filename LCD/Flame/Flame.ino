#include <LiquidCrystal.h>

LiquidCrystal lcd(8,9,4,5,6,7);

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

void setup() {
    lcd.createChar(1, flame1);
    lcd.createChar(2, flame2);
    lcd.begin(16, 2);
    lcd.write(1);
}

void loop() {
  lcd.setCursor(0,0);
  lcd.write(1);
  delay(200);
  lcd.setCursor(0,0);
  lcd.write(2);
  delay(200);
}
