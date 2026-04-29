#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// 1. Adjust these until the robot does exactly 90 degrees
int testTicksL = 35; 
int testTicksR = 35; 

// Motor Pins
const int LWhFwdPin = 4;
const int LWhPWMPin = 5;
const int RWhFwdPin = 8;
const int RWhPWMPin = 6;

// Encoder Pins
volatile long cntrL = 0;
volatile long cntrR = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  lcd.init();
  lcd.backlight();
  
  pinMode(LWhFwdPin, OUTPUT);
  pinMode(RWhFwdPin, OUTPUT);
  
  // Interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(2), []{cntrL++;}, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), []{cntrR++;}, CHANGE);

  lcd.setCursor(0, 0);
  lcd.print("Turn Test Ready");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for IR..."); 
  
  // Using your existing IR setup logic if you want to trigger by remote
  // For now, it will wait 3 seconds and then start automatically
  delay(3000);
}

void loop() {
  // --- TEST LEFT TURN ---
  lcd.clear();
  lcd.print("Testing LEFT");
  lcd.setCursor(0, 1);
  lcd.print("Target: "); lcd.print(testTicksL);
  
  cntrR = 0; 
  while(cntrR < testTicksL) {
    digitalWrite(LWhFwdPin, LOW);
    digitalWrite(RWhFwdPin, HIGH);
    analogWrite(RWhPWMPin, 140); // Right motor moves for Left turn
  }
  stopMotors();
  
  lcd.setCursor(0,1);
  lcd.print("Done. Waiting...");
  delay(3000); // Pause to check the angle

  // --- TEST RIGHT TURN ---
  lcd.clear();
  lcd.print("Testing RIGHT");
  lcd.setCursor(0, 1);
  lcd.print("Target: "); lcd.print(testTicksR);
  
  cntrL = 0;
  while(cntrL < testTicksR) {
    digitalWrite(LWhFwdPin, HIGH);
    digitalWrite(RWhFwdPin, LOW);
    analogWrite(LWhPWMPin, 150); // Left motor moves for Right turn
  }
  stopMotors();
  
  lcd.setCursor(0,1);
  lcd.print("Done. Waiting...");
  delay(5000); // Long pause before repeating
}

void stopMotors() {
  analogWrite(LWhPWMPin, 0);
  analogWrite(RWhPWMPin, 0);
  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(RWhFwdPin, LOW);
}