/* 
  WEEK 1: RUN WHEELS FWD + BWD
  WEEK 2: IR RECIEVER CONTROLLED FINITE STATE MACHINE (REMOTE CONTROL)
  WEEK 3: WHEEL ENCODERS AND P-CONTROLL
  WEEK 4: DETECT OBSTACLES, DISPLAY DISTANCE ON LCD, TURN 90 DEGREES WHEN TOO CLOSE TO OBSTACLE
     */

/*  NOTE: DON'T USE PIN 3 OR 11 FOR PULSE MODULATION */
/*  NOTE DON'T USE PIN 0 OR 1 FOR PULSE MODULATION */

/*
Pin Map:
  0
  1
  2 - Left Wheen Encoder
  3 - Right Wheel Encoder
  4 - Right Wheel Forward
  5 - Left Wheel Backward
  6 - Left Wheel Power
  7 - Left Wheel Forward
  8 - IR Senser
  9 - Right Wheel Power
  10  - Right Wheel Backward
  11 
  12 
  13

  A0
  A1
  A2  
  A3  
  A4 - SDA FROM LCD AND IR DISTANCE SENSOR
  A5 - SCL FROM LCD AND IR DISTANCE SENSOR
*/

/* Variables for finite state machine*/
  int ir_code;
  int state = 1;
  /* 
  0 = STOP
  1 = FWD
  2 = BWD */
  float rangeInches = 0;

/*MOTOR SPEED PRE-SETS*/
 //const int RSPD1 = 170;        //Right Wheel PWM
 //const int LSPD1 = 180;        //Left Wheel PWM

 const int RSPD2 = 110;        //Right Wheel PWM
 const int LSPD2 = 130;        //Left Wheel PWM

 /*MOTOR PINS*/
 const int LWhFwdPin = 7;
 const int LWhBwdPin = 5;
 const int LWhPWMPin = 6;

 const int RWhFwdPin = 4;
 const int RWhBwdPin = 10;
 const int RWhPWMPin = 9; 


/* IR SET UP */
#define DECODE_NEC
#include <IRremote.hpp>
#define IR_Pin 8

/*  P-CONTROLLER SET UP  */
 int gain=8;
 int delCntr=0; 
 volatile long cntrL, cntrR;
 volatile long LIntTime, RIntTime;
 long stopTime;

/*  LCD SET UP  */
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

/*  TOF IR DISTANCE SENSOR  */
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() 
{
  Serial.begin(9600);

  /*MOTOR SET UP*/
      /* START WHEELS IN SETPUP SO THAT P-CONTROLLER HAS VALUES TO ADJUST WITH  */
  pinMode(LWhFwdPin,OUTPUT);
  pinMode(LWhBwdPin,OUTPUT);
  pinMode(LWhPWMPin,OUTPUT);
  pinMode(RWhFwdPin,OUTPUT);
  pinMode(RWhBwdPin,OUTPUT);
  pinMode(RWhPWMPin,OUTPUT);

  digitalWrite(LWhFwdPin,HIGH);
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(LWhPWMPin,LOW);
  
  digitalWrite(RWhFwdPin,HIGH);
  digitalWrite(RWhBwdPin,LOW);
  digitalWrite(RWhPWMPin,LOW);


  /* Initiate the IR sensor */
  pinMode(IR_Pin, INPUT);
  IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK); // Start the receiver

  /*  Initiate the DISTANCE SENSOR  */


  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    state = 0;
    while(1);
  }

  /* Initiate the LCD */
  lcd.init();
  lcd.backlight();

  /*  WHEEL ENCODER SET UP  */
  attachInterrupt(digitalPinToInterrupt(2), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightWhlCnt, CHANGE);

  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;

  analogWrite(LWhPWMPin,LSPD2);
  analogWrite(RWhPWMPin,RSPD2);

}


void loop() 
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      rangeInches = measure.RangeMilliMeter / 25.4;

      lcd.setCursor(2, 0); // Set the cursor on the third column and first row.
      lcd.print(rangeInches, 2); // Print the distance in inches
      lcd.print(" in   ");
  }
  

  /*  P- CONTROLL SYSTEM  */
  int RSPD = RSPD2;   // RIGHT AND LEFT SPEED VARIABLES CHANGE BASED ON P-CONTROLL AND CONSTANT SPEED VALUES
  int LSPD = LSPD2;   
  long tmpLcntr, tmpRcntr;    //snapshot of the cntrL and cntrR value
  noInterrupts();
  tmpLcntr = cntrL;
  tmpRcntr = cntrR;
  interrupts();
  
  delCntr = abs(tmpLcntr - tmpRcntr);   // The error is wheel speed

  /*  Calculate speed (P-Controller)  */
  if(tmpLcntr > tmpRcntr) // LEFT WHEEL IS SPINNING TOO FAST
  {
    RSPD = RSPD2; 
    LSPD = max(LSPD2 - int(gain *  delCntr + 0.5),0);
    /*  Adjusts using: LSPD - (gain * delCntr)
          int(... + 0.5) is to round to the nearest whole number
          max(... , 0) is to prevent the speed from going negative (max function returns highest value parameter) */
  }
  else if(tmpRcntr > tmpLcntr) // RIGHT WHEEL IS SPINNING TOO FAST
  {
    RSPD = max(RSPD2 - int(gain *  delCntr + 0.5),0); 
    LSPD = LSPD2;
  }



/* IR SENSOR CONTROL SYSTEM (FSM)*/
 
    if (IrReceiver.decode()) {
      ir_code = IrReceiver.decodedIRData.command;
      //Serial.println(ir_code);
      IrReceiver.resume(); // Enable receiving of the next value
    }
    else
    {
      ir_code = -1;
    }
    

    // Reset counters whenever a new move command is sent
    if(ir_code == 70 || ir_code == 21) 
    {
      cntrL = 0; 
      cntrR = 0;
    }

    switch (ir_code) 
    {
      case 70:    //CH
        state = 1;    //state -> FWD
        break;

      case 64:    //NEXT
        state = 0;    //state -> STOP
        break;

      case 21:    //VOL+
        state = 2;    //state -> BWD
        break;

    }
  

  /* CHANGE SIGNAL DEPENDING ON STATE*/
  switch (state)
  {
    case 0:   //STOP
      digitalWrite(LWhFwdPin,LOW);    //STOP BOTH WHEELS
      digitalWrite(RWhFwdPin,LOW);   
      digitalWrite(LWhBwdPin,LOW);    
      digitalWrite(RWhBwdPin,LOW); 
 
      break; 

    case 1:   //FWD

      //NOTE: 4 INCHES IS ~100MM BUT ROVER DOES NOT STOP QUICK ENEUGH
      if(measure.RangeStatus != 4 && measure.RangeMilliMeter <=200)
        {
          state = 3;
          digitalWrite(LWhFwdPin,LOW);    //STOP BOTH WHEELS
          digitalWrite(RWhFwdPin,LOW);   
          digitalWrite(LWhBwdPin,LOW);    
          digitalWrite(RWhBwdPin,LOW);   
          delay(250);

          cntrL = 0; 
          cntrR = 0;
          break;
        }

      digitalWrite(LWhBwdPin,LOW);    //backward signals -> low
      digitalWrite(RWhBwdPin,LOW);   

      digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
      analogWrite(LWhPWMPin,LSPD);
      digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
      analogWrite(RWhPWMPin,RSPD);
      break;

    case 2:   //BWD

      digitalWrite(LWhFwdPin,LOW);    //forward signals -> low
      digitalWrite(RWhFwdPin,LOW);   

      digitalWrite(LWhBwdPin,HIGH);    //run left wheel forward
      analogWrite(LWhPWMPin,LSPD);
      digitalWrite(RWhBwdPin,HIGH);   //run right wheel forward
      analogWrite(RWhPWMPin,RSPD);
      break;

    case 3: //RIGHT TURN, FWD FOR 1 SEC, AND STOP

      noInterrupts();
      tmpLcntr = cntrL;
      tmpRcntr = cntrR;
      interrupts();

      //NOTE A 90 DEGREE TURN SHOULD BE ~40 PULSES BUT ROVER DOSNT STOP QUICK ENEUGH
      if(tmpLcntr < tmpRcntr + 25)
      {

        digitalWrite(LWhBwdPin,LOW);    //RIGHT WHEEL GOES BACKWARDS
        digitalWrite(RWhBwdPin,HIGH);   

        digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
        analogWrite(LWhPWMPin,110);
        digitalWrite(RWhFwdPin,LOW);   
        analogWrite(RWhPWMPin,110);
      }
      else
      {

          digitalWrite(LWhFwdPin,LOW);    //STOP BOTH WHEELS
          digitalWrite(RWhFwdPin,LOW);   
          digitalWrite(LWhBwdPin,LOW);    
          digitalWrite(RWhBwdPin,LOW);   
          delay(250);

        /*  go forward for 1 second and stop  */

        digitalWrite(LWhBwdPin,LOW);    //backward signals -> low
        digitalWrite(RWhBwdPin,LOW);   

        digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
        analogWrite(LWhPWMPin,110);
        digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
        analogWrite(RWhPWMPin,110);

        delay(1000);  

        state = 0;
      }

  }


Serial.print("L_Cnt:"); Serial.print(tmpLcntr); 
Serial.print(" | R_Cnt:"); Serial.print(tmpRcntr);
Serial.print(" | L_PWM:"); Serial.print(LSPD);
Serial.print(" | R_PWM:"); Serial.println(RSPD);



  delay(50);
}



/*  INTERUPT FUNCTIONS  */
void leftWhlCnt()
{
  long intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    cntrL++;
  }
}

void rightWhlCnt()  
{
  long intTime = micros();
  if(intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    if(state==3)    //need to incroment backewards for turn in place
    {
      cntrR--;
    }
    else
    {
      cntrR++;
    }
  }
}

