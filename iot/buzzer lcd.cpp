#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SDA 14
#define SCL 15
#define COLUMNS 16
#define ROWS    2

const int buzzerPin = 13;

LiquidCrystal_I2C lcd(0x27, COLUMNS, ROWS);  

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  Wire.begin(SDA, SCL);
  lcd.init();   
  lcd.clear();           
  lcd.backlight();                      
  
  lcd.setCursor(0,0);
  lcd.print(" uji coba ");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print(" LCD I2C Module ");
  delay(8000);
}

void loop() {
  digitalWrite(buzzerPin, HIGH);


  }

