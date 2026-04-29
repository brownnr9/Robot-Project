#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_VL53L0X.h"

// Pins
const int LWhFwdPin = 4;
const int LWhBwdPin = 7;
const int LWhPWMPin = 5;
const int RWhFwdPin = 8;
const int RWhBwdPin = 9;
const int RWhPWMPin = 6;
const int encoderPinL = 2;

// Objects
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Variables
volatile long cntrL = 0;
int startDist = 0;
int endDist = 0;
bool testFinished = false;

void setup() {
  pinMode(LWhFwdPin, OUTPUT);
  pinMode(LWhBwdPin, OUTPUT);
  pinMode(LWhPWMPin, OUTPUT);
  pinMode(RWhFwdPin, OUTPUT);
  pinMode(RWhBwdPin, OUTPUT);
  pinMode(RWhPWMPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(2), countTicks, CHANGE);

  lcd.init();
  lcd.backlight();
  
  if (!lox.begin()) {
    lcd.print("TOF Error!");
    while(1);
  }

  lcd.print("Place vs Wall");
  delay(3000); // Time to place robot facing a wall
  
  // Take initial measurement
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  startDist = measure.RangeMilliMeter;
  
  lcd.clear();
  lcd.print("Running Test...");
}

void loop() {
  if (cntrL < 60) {
    // Drive forward at 150 power
    digitalWrite(LWhFwdPin, HIGH);
    digitalWrite(LWhBwdPin, LOW);
    analogWrite(LWhPWMPin, 150);

    digitalWrite(RWhFwdPin, HIGH);
    digitalWrite(RWhBwdPin, LOW);
    analogWrite(RWhPWMPin, 150);
  } 
  else if (!testFinished) {
    // Stop immediately
    digitalWrite(LWhFwdPin, LOW);
    digitalWrite(RWhFwdPin, LOW);
    analogWrite(LWhPWMPin, 0);
    analogWrite(RWhPWMPin, 0);

    // Take final measurement
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    endDist = measure.RangeMilliMeter;
    
    int traveled = startDist - endDist;

    // Display Results
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Start: "); lcd.print(startDist);
    lcd.setCursor(0,1);
    lcd.print("End:   "); lcd.print(endDist);
    lcd.setCursor(0,2);
    lcd.print("Travel: "); lcd.print(traveled);
    lcd.print("mm");
    
    testFinished = true;
  }
}

void countTicks() {
  cntrL++;
}