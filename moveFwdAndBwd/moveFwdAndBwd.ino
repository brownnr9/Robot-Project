/* Run Wheels Forward and Backward
 */
 const int RSPD1 = 180;        //Right Wheel PWM
 const int LSPD1 = 180;        //Left Wheel PWM
 const int RSPD2 = 175;        //Right Wheel PWM
 const int LSPD2 = 175;        //Left Wheel PWM

 
 const int LWhFwdPin = 7;
 const int LWhBwdPin = 5;
 const int LWhPWMPin = 6;

 const int RWhFwdPin = 4;
 const int RWhBwdPin = 2;
 const int RWhPWMPin = 3; 


void setup() 
{
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
}


void loop() 
{
        //move robot forward for 1 second
  digitalWrite(LWhFwdPin,HIGH);    //run left wheel forward
  analogWrite(LWhPWMPin,LSPD1);
  digitalWrite(RWhFwdPin,HIGH);   //run right wheel forward
  analogWrite(RWhPWMPin,RSPD1);
  delay(1000);
  
  digitalWrite(LWhFwdPin,LOW);   //allow for change in direction
  digitalWrite(RWhFwdPin,LOW);   

          //move robot backward for 1 second
  digitalWrite(LWhBwdPin,HIGH);    //run left wheel forward
  analogWrite(LWhPWMPin,LSPD2);
  digitalWrite(RWhBwdPin,HIGH);   //run right wheel forward
  analogWrite(RWhPWMPin,RSPD2);
  delay(1000);

        //stop the robot
  while(true)
  {
    analogWrite(LWhPWMPin,0);       // stop left wheel
    analogWrite(RWhPWMPin,0);       // stop right wheel
  }
}

