/* 
  WEEK 1: RUN WHEELS FWD + BWD
  WEEK 2: IR RECIEVER CONTROLLED FINITE STATE MACHINE (REMOTE CONTROL)
  WEEK 3: WHEEL ENCODERS AND P-CONTROLL
     */

/*  NOTE: DON'T USE PIN 3 OR 11 FOR PULSE MODULATION */
/*  NOTE DON'T USE PIN 0 OR 1 FOR PULSE MODULATION */

/*
Pin Map:
  0
  1
  2 - 
  3 
  4 - Right Wheel Forward
  5 - Left Wheel Backward
  6 - Left Wheel Power
  7 - Left Wheel Forward
  8 
  9 - Right Wheel Power
  10  - Right Wheel Backward
  11 
  12  - Right Wheel Encoder
  13
*/

/* Variables for finite state machine*/
  int ir_code;
  int state = 1;
  /* 
  0 = STOP
  1 = FWD
  2 = BWD */

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

  /*  P- CONTROLL SYSTEM  */
  int RSPD = RSPD2;   // RIGHT AND LEFT SPEED VARIABLES CHANGE BASED ON P-CONTROLL AND CONSTANT SPEED VALUES
  int LSPD = LSPD2;   
  long tmpLcntr, tmpRcntr;    //snapshot of the cntrL and cntrR value
  tmpLcntr = cntrL;
  tmpRcntr = cntrR;
  
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

    Serial.println(ir_code);
    IrReceiver.resume(); // Enable receiving of the next value

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
    cntrR++;
  }
}