/* 
  WEEK 1: RUN WHEELS FWD + BWD
  WEEK 2: IR RECIEVER CONTROLLED FINITE STATE MACHINE (REMOTE CONTROL)
     */

/*  NOTE: DON'T USE PIN 3 OR 11 FOR PULSE MODULATION */
/*  NOTE DON'T USE PIN 0 */

/* Variables for finite state machine*/
  int ir_code;
  int state = 0;
  /* 
  0 = STOP
  1 = FWD
  2 = BWD */

/*MOTOR SPEED PRE-SETS*/
 const int RSPD1 = 170;        //Right Wheel PWM
 const int LSPD1 = 180;        //Left Wheel PWM

 const int RSPD2 = 90;        //Right Wheel PWM
 const int LSPD2 = 100;        //Left Wheel PWM

 /*MOTOR PINS*/
 const int LWhFwdPin = 7;
 const int LWhBwdPin = 5;
 const int LWhPWMPin = 6;

 const int RWhFwdPin = 4;
 const int RWhBwdPin = 2;
 const int RWhPWMPin = 9; 


/* IR SET UP */
#define DECODE_NEC
#include <IRremote.hpp>
#define IR_Pin 8


void setup() 
{
  Serial.begin(9600);

  /*MOTOR SET UP*/
  pinMode(LWhFwdPin,OUTPUT);
  pinMode(LWhBwdPin,OUTPUT);
  pinMode(LWhPWMPin,OUTPUT);
  pinMode(RWhFwdPin,OUTPUT);
  pinMode(RWhBwdPin,OUTPUT);
  pinMode(RWhPWMPin,OUTPUT);

  digitalWrite(LWhFwdPin,LOW);
  digitalWrite(LWhBwdPin,LOW);
  digitalWrite(LWhPWMPin,LOW);
  
  digitalWrite(RWhFwdPin,LOW);
  digitalWrite(RWhBwdPin,LOW);
  digitalWrite(RWhPWMPin,LOW);


  /* Initiate the IR sensor */
  pinMode(IR_Pin, INPUT);
  IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK); // Start the receiver



}


void loop() 
{

/* IR SENSOR CONTROL SYSTEM (FSM)*/
  if (IrReceiver.decode())
  {
    ir_code = IrReceiver.decodedIRData.command;
    Serial.println(ir_code);
    IrReceiver.resume(); // Enable receiving of the next value
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
      analogWrite(LWhPWMPin,LSPD2);
      digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
      analogWrite(RWhPWMPin,RSPD2);
      break;

    case 2:   //BWD
      digitalWrite(LWhFwdPin,LOW);    //forward signals -> low
      digitalWrite(RWhFwdPin,LOW);   

      digitalWrite(LWhBwdPin,HIGH);    //run left wheel forward
      analogWrite(LWhPWMPin,LSPD2);
      digitalWrite(RWhBwdPin,HIGH);   //run right wheel forward
      analogWrite(RWhPWMPin,RSPD2);
      break;

  }

  delay(100);
}

