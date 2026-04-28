/* 
  WEEK 1: RUN WHEELS FWD + BWD
  WEEK 2: IR RECIEVER CONTROLLED FINITE STATE MACHINE (REMOTE CONTROL)
  WEEK 3: WHEEL ENCODERS AND P-CONTROLL
  WEEK 4: DETECT OBSTACLES, DISPLAY DISTANCE ON LCD, TURN 90 DEGREES WHEN TOO CLOSE TO OBSTACLE
  WEEK 5: FREELY NAVIGATE. USES SERVO MOTOR TO LOOK AROUND AND DETERMINE NEXT DIRECTION
     */

/*  NOTE: DON'T USE PIN 3 OR 11 FOR PULSE MODULATION */
/*  NOTE DON'T USE PIN 0 OR 1 FOR PULSE MODULATION */

/*
Pin Map:
  0
  1
  2 - Left Wheen Encoder
  ~3  - Right Wheel Encoder
  4 - Left wheel forward
  ~5 - Left Wheel Power (ENA)
  ~6 - Right wheel power (ENB)
  7 - Left wheel backwar
  8 - Right wheel forward
  9 - Right wheel backward
  ~10  IR reciever  ;
  ~11 
  12 - trig (ultrasonic sensor)
  13 - echo (ultrasonic sensor)

  A0 - Servo motor (analog pin used as a digital pin)
  A1
  A2  
  A3  
  A4 - SDA FROM LCD AND IR DISTANCE SENSOR
  A5 - SCL FROM LCD AND IR DISTANCE SENSOR
*/


  int heading = 1;
  //left turn -> -1
  //right turn -> +1
  
/* Variables for finite state machine*/
  int ir_code;
  int state = 1;
  /* 
  0 = STOP
  1 = FWD
  2 = BWD */
  bool Lturn = false;
  bool Rturn = false;

/*MOTOR SPEED PRE-SETS*/
 //const int RSPD1 = 170;        //Right Wheel PWM
 //const int LSPD1 = 180;        //Left Wheel PWM

 const int RSPD2 = 110;        //Right Wheel PWM
 const int LSPD2 = 130;        //Left Wheel PWM

 /*MOTOR PINS*/
 const int LWhFwdPin = 4;
 const int LWhBwdPin = 7;
 const int LWhPWMPin = 5;

 const int RWhFwdPin = 8;
 const int RWhBwdPin = 9;
 const int RWhPWMPin = 6; 


/* IR SET UP */
#define DECODE_NEC
#include <IRremote.hpp>
#define IR_Pin 10

/*  P-CONTROLLER SET UP  */
 int gain=8;
 int delCntr=0; 
 volatile long cntrL, cntrR;
 volatile long LIntTime, RIntTime;
 long stopTime;

/*  CONTROLLER FOR WALL */
int targetDistance = 150; // Target distance from wall in mm
float wallGain = 0.25;      // How much the wall distance affects steering
int wallError = 0;       // Calculated correction

/*  LCD SET UP  */
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

/*  TOF IR DISTANCE SENSOR  */
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Ultrasonic distance sensor
const int pingPin = 12;
const int echoPin = 13;
//NOTE ADD ASSIGNMENT



/* SERVO MOTOR  */
/*
#include <Servo.h>
Servo myservo;  // create Servo object to control a servo
int pos = 90;    // variable to store the servo position
*/


// ===== ULTRASONIC =====
long measureUltra;
long ultra() {
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  long cm = duration * 0.0343 / 2;
  long mm = cm*10;

  //if (cm <= 0 || cm > 500) return 500;
  return mm; 
}



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

  /*  Initiate the IR DISTANCE SENSOR  */


  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    state = 0;
    while(1);
  }

  /*  Instantiate the ultrasonic distance sensor    */
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  /* Initiate the LCD */
  lcd.init();
  lcd.backlight();

  /*  SERVO MOTOR SET UP  */
  /*
  myservo.attach(A0);  // attaches the servo on pin 9 to the Servo object
  myservo.write(pos); // sets the servo position to forward
  */

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
  measureUltra = ultra();

  VL53L0X_RangingMeasurementData_t measureIR;
  lox.rangingTest(&measureIR, false); // pass in 'true' to get debug data printout!

  if (measureIR.RangeStatus != 4) {  // phase failures have incorrect data
  
      lcd.setCursor(2, 0); // Set the cursor on the third column and first row.
      lcd.print(measureIR.RangeMilliMeter); // Print the distance in inches
      lcd.print(" mm     ");
      lcd.setCursor(2,1);
      lcd.print(measureUltra);
      lcd.print(" mm    ");
  }
  

  /*  P- CONTROLL SYSTEM  */

  wallError = measureUltra - targetDistance;

  // This correction value determines how much to deviate from driving straight
  int wallCorrection = int(wallError * wallGain);

  int RSPD = RSPD2;   // RIGHT AND LEFT SPEED VARIABLES CHANGE BASED ON P-CONTROLL AND CONSTANT SPEED VALUES
  int LSPD = LSPD2;   
  long tmpLcntr, tmpRcntr;    //snapshot of the cntrL and cntrR value
  noInterrupts();
  tmpLcntr = cntrL;
  tmpRcntr = cntrR;
  interrupts();
  
  int encoderError = tmpLcntr - tmpRcntr; 
  int encoderAdjustment = encoderError * gain;

  /*  Calculate speed (P-Controller)  */
  LSPD = LSPD2 + wallCorrection - encoderAdjustment;
  RSPD = RSPD2 - wallCorrection + encoderAdjustment;


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

    switch (ir_code) 
    {
      case 64:    //NEXT
        state = 0;    //state -> STOP
        break;
    }
  

  /* CHANGE SIGNAL DEPENDING ON STATE*/
  switch (state)
  {
    case 0:   //STOP
      stop();
      break; 

    case 1:   //FWD

      //NOTE: 4 INCHES IS ~100MM BUT ROVER DOES NOT STOP QUICK ENEUGH

      // 2 cases are right corner and no more wall
      
      if(measureIR.RangeStatus != 4 && measureIR.RangeMilliMeter <=200 && measureUltra <= 200)
        {//corner right case -> needs to turn left
          state = 3;
          stop();
          delay(250);

          cntrL = 0; 
          cntrR = 0;
          break;
        }
      else if(measureUltra > 200)
      {
        state = 4;
          delay(1000);
          stop();
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

    case 3: //CORNER RIGHT PROTOCALL
    //LEFFT TURN
        Lturn = true;

        while(tmpRcntr < tmpLcntr + 30){
          noInterrupts();
          tmpLcntr = cntrL;
          tmpRcntr = cntrR;
          interrupts();

          //turn right in place
          digitalWrite(LWhFwdPin,LOW);    //left wheel bwd
          digitalWrite(LWhBwdPin,HIGH);    

          digitalWrite(RWhFwdPin,HIGH);    //right wheel fwd
          digitalWrite(RWhBwdPin,LOW);    

          analogWrite(RWhPWMPin,RSPD);
          analogWrite(LWhPWMPin,LSPD);
        }
        stop();
        Lturn = false;
        state = 1;
        cntrL = 0;
        cntrR = 0;

        heading = heading +1;
        break;


    case 4: //right wall ends
    if(heading == 0){
      state = 1;
      break;
    }
    else
    {   //RIGHT TURN
      Rturn = true;

        while(tmpLcntr < tmpRcntr + 30){
          noInterrupts();
          tmpLcntr = cntrL;
          tmpRcntr = cntrR;
          interrupts();

          //turn right in place
          digitalWrite(LWhFwdPin,HIGH);    //left wheel forward
          digitalWrite(LWhBwdPin,LOW);    

          digitalWrite(RWhFwdPin,LOW);    //right wheel backward
          digitalWrite(RWhBwdPin,HIGH);    

          analogWrite(RWhPWMPin,130);
          analogWrite(LWhPWMPin,130);
        }
        
        stop();
        state = 1;
        cntrL = 0;
        cntrR = 0;
        Rturn = false;

        heading = heading -1;
        break;
        }
        
  }
        

    


Serial.print("L_Cnt:"); Serial.print(tmpLcntr); 
Serial.print(" | R_Cnt:"); Serial.print(tmpRcntr);
Serial.print(" | L_PWM:"); Serial.print(LSPD);
Serial.print(" | R_PWM:"); Serial.print(RSPD);
Serial.print(" | Ult: "); Serial.println(measureUltra);



  delay(50);
}
  




/*  INTERUPT FUNCTIONS  */
void leftWhlCnt()
{
  long intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    if(Lturn){
      cntrL--;    //left wheel turns back when turning left
    }
    else
    {
      cntrL++;
    }
  }
}

void rightWhlCnt()  
{
  long intTime = micros();
  if(intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    if(Rturn)    //right wheel turns back when turning right
    {
      cntrR--;
    }
    else
    {
      cntrR++;
    }
  }
}

void stop()
{
      digitalWrite(LWhFwdPin,LOW);    //STOP BOTH WHEELS
      digitalWrite(RWhFwdPin,LOW);   
      digitalWrite(LWhBwdPin,LOW);    
      digitalWrite(RWhBwdPin,LOW); 
}

