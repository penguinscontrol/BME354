LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Timer Variables
int minutes;
int seconds;

// Set-up stage
bool selectmin = true;
bool selectsec = true;
int previous;
int after;

int buttonPin = A0;

int lcd_key = 0;
int adc_key_in = 0;
#define btnRIGHT 0
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

#define V1 20
#define V2 119
#define V3 276
#define V4 430
#define V5 660
#define VNONE 1003

int debounce(int last) {
  int current = analogRead(buttonPin); // Current button state
  if (last != current) // If a change happened
  {
    delay(10); // wait for bounce to settle
    current = analogRead(buttonPin); 
  }
  return current;
}

int read_LCD_buttons() {
	adc_key_in = debounce(adc_key_in);
	if (adc_key_in < V2) return btnUP;
 	if (adc_key_in < V3) return btnDOWN;
 	if (adc_key_in < V4) return btnLEFT;
 	if (adc_key_in < V5) return btnSELECT;
 	return btnNONE;
}

void setup() {
	lcd.begin(16, 2); 
	Serial.begin(9600);
}

void loop() {
	lcd.print("Minutes: ");
	lcd.setCursor(0,1);
	lcd.blink();
	while (selectmin == true) {
		lcd_key = read_LCD_buttons();
		switch (lcd_key) {
			case btnRIGHT: 
				break;
			case btnLEFT: 
				break;
			case btnUP: {
				minutes = minutes + 1;
				lcd.setCursor(0,1);
				lcd.print(minutes);
				break;
			}
			case btnDOWN: {
				if (minutes > 0) {
					minutes = minutes - 1;
					lcd.setCursor(0,1);
					lcd.print(minutes);
				}
				break;
			}
			case btnSELECT: {
				selectmin = false;
				break;
			}
			case btnNONE: 
				break;			
		}
		delay(200); 
	}
	lcd.clear(); 
	delay(500);
	lcd.print("Seconds: ");
	lcd.setCursor(0,1);
	lcd.blink();
	while (selectsec == true) {
		lcd_key = read_LCD_buttons();
		switch (lcd_key) {
			case btnRIGHT: 
				break;
			case btnLEFT: 
				break;
			case btnUP: {
				if (seconds == 59) {
					seconds = 0;
				} else {
				seconds = seconds + 1;
				}
				lcd.setCursor(0,1);
				lcd.print(seconds);
				break;
			}
			case btnDOWN: {
				if (seconds > 0) {
					seconds = seconds - 1;
				} else if (seconds == 0) {
					seconds = 59;
				}
				lcd.setCursor(0,1);
				lcd.print(seconds);
				}
				break;
			}
			case btnSELECT: {
				selectsec = false;
				break;
			}
			case btnNONE: 
				break;			
		}
	}
	lcd.clear();
	delay(500);
	lcd.print("Timer Countdown:");
	lcd.setCursor(0,1);
	while ((minutes > 0) && (seconds > 0)) {
		lcd.print(minutes);
		lcd.setCursor(2,1);
		lcd.print(":");
		lcd.setCursor(3,1);
		lcd.print(seconds);
		delay(1000);
		if ((minutes == 0) && (seconds == 0)) {
			lcd.print("TIME IS UP!");
		} else if (seconds != 0) {
			seconds = seconds - 1;
		} else {
			seconds = 59;
			minutes = minutes - 1;
		}
	}
}