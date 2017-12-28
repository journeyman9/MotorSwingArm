/*
 * Tutorial 1a: Motor Driver
 * 
 * Drives two motors forward.
 *
 * The circuit:
 *  - 6 pins connected to the L298 Motor Driver
 *
 * created 27 Sep 2013
 * by Blaise Jarrett
 *
 * This example code is in the public domain.
 *
 */

// include our motor code
#include "motor.h"
#include "Pot.h"
#include "Arduino.h"

// The Right Motors Enable Pin
// Labelled on the motor driver as ENA
// Be carful of PWM Timers
const int motorRENPin = 10;
// The Right Motors IN1 Pin
// Labelled on the motor driver as IN2
const int motorRIN2Pin = 9;
// The Right Motors IN2 Pin
// Labelled on the motor driver as IN1
const int motorRIN1Pin = 8;

// The Left Motors Enable Pin
// Labelled on the motor driver as ENB
// Be carful of PWM Timers
const int motorLENPin = 5;
// The Left Motors IN1 Pin
// Labelled on the motor driver as IN3
const int motorLIN2Pin = 7;
// The Left Motors IN2 Pin
// Labelled on the motor driver as IN4
const int motorLIN1Pin = 6;

// Create two Motor objects (instances of our Motor class)
// to drive each motor.
Motor rightMotor(motorRIN1Pin, motorRIN2Pin, motorRENPin);
Motor leftMotor(motorLIN1Pin, motorLIN2Pin, motorLENPin);

Pot feedback;

uint32_t posSet;
uint32_t pos;
int32_t u;
int32_t error;

uint32_t serialdata;
char serialCMD;
int inbyte;
int32_t P;
int32_t I;
int32_t D;
int32_t N;
int32_t W;
int32_t A;
int switchnum;

void Command();

void setup()
{
  //Switch Input
  DDRD |= 0x00;
  PORTD |= 0b00000100; // Secondary function to set pull up resistor
  Serial.begin(9600);
  A = 90; //Define Angle as from 0 deg to 180deg. Assume nominal 90deg for vertical stick.
 }

void loop()
{
  if (Serial.available())
 {
   Command();
 } 

  posSet = 4.74*A + 127;
  pos = feedback.readFeedback();
  
  /* A2D min - 44
   *  A2D max - 1005
   *  A2D middle - 544
   *  A2D 0 - 127
   *  A2D 180 - 980
   */

   // Software limit switches for position
   if ((pos >= 767) || (pos <= 340))
   {
    u = 0;
    P = I = D = N = W = 0;
    Serial.println("Fault detected, Motor Stopped");
    _delay_ms(500);
   }
  error = posSet - pos;
  u = P*error/100;

  //Serial.print("u0: ");
  //Serial.println(u);
 
  if (u > 255)
  {
    u = 255;
  }
    else if (u < -255)
    {
      u = -255; 
    }
  if (u >= 0)
  {
    rightMotor.forward(u);
  }
    else if (u < 0)
    {
      u = -1*u;
      rightMotor.backward(u);
    } 
/*
  // delay for loop
  Serial.print("u: ");
  Serial.println(u);
  Serial.print("error: ");
  Serial.println(error);
  Serial.println("------");
  _delay_ms(1000); 
*/
}

void Command()
{
  void getCommand();
  void getSerial();
  serialCMD = 0;
  switchnum = 0;
  getCommand();

  if (serialCMD == 'R' || serialCMD == 'r')
  {
    switchnum = 1;
  }
  else if (serialCMD == 'P' || serialCMD == 'p')
  {
    switchnum = 2;
  }
  else if (serialCMD == 'I' || serialCMD == 'i')
  {
    switchnum = 3;
  }
  else if (serialCMD == 'D' || serialCMD == 'd')
  {
    switchnum = 4;
  }
  else if (serialCMD == 'N' || serialCMD == 'n')
  {
    switchnum = 5;
  }
  else if (serialCMD == 'W' || serialCMD == 'w')
  {
    switchnum = 6;
  }
  else if (serialCMD == 'A' || serialCMD == 'a')
  {
    switchnum = 7;
  }
  else
  {
    switchnum = 8;
  }
  
switch(switchnum)
{
  case 1:
       {
     //Reset PID parameters
          if(serialdata != 'E')
          {
            P = I = D = N = W = 0;
            A = 90;
          }

         break;  
        }
   case 2:
        {
          //update Proportional variable
    
          getSerial();
          if(serialdata != 'E')
          {
            P = serialdata;
          }
          
          break; 
        }
   case 3:
        {
          //Update Integral variable
          getSerial();
           if(serialdata != 'E')
          {
            I = serialdata;
          }
          break;
         
        }
        
   case 4:
       {
         //Update Derivative variable
          getSerial();
          if(serialdata != 'E')
          {
            D = serialdata;
          }
          break;
          
       }
       
   case 5:
       {
       //Update Derivative filter constant
         getSerial();
          if(serialdata != 'E')
          {
            N = serialdata;
          }
         break; 
       }
   case 6:
       {
         //Update anti-Windup limit constant
         getSerial();
          if(serialdata != 'E')
          {
            W = serialdata;
          }
         break; 
        
       }
     case 7:
       {
       
          //Set target angle
         getSerial();       
          if(serialdata != 'E')
          {
            if (serialdata >= 0 && serialdata <= 180)
             {               
               A = serialdata;
             }
             else 
             {
               Serial.print("OUT OF RANGE");
             }
             
         break;
          }
       } 
     case 8:
          {
            Serial.println("Command ERROR!");
          break;
          }
}
       Serial.print("P: ");
       Serial.print(P); 
       Serial.print(" I: ");
       Serial.print(I);
       Serial.print(" D: ");
       Serial.print(D);
       Serial.print(" N: ");
       Serial.print(N);
       Serial.print(" W: ");
       Serial.print(W);
       Serial.print(" A: ");
       Serial.println(A);
}

long getCommand()
{
  serialCMD = 0;
  while (inbyte != '/')
  {
     inbyte = Serial.read();
     if (inbyte > 0 && inbyte != '/')
    {
     serialCMD = inbyte;  
    }
  }
  inbyte = 0;
  return serialCMD;
}

long getSerial()
{
  serialdata = 0;
  inbyte = 0;
  while (inbyte != '\r')
  {
    inbyte = Serial.read(); 
    
    if (inbyte >= 48 && inbyte <= 57 && inbyte != '\r')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
    else if (inbyte > 0 && inbyte < 48 && inbyte != '\r' || inbyte > 57  )
    {
        Serial.print("Num Error: ");
        Serial.println(inbyte);
        serialdata = 'E';
        break;
    }
  }

  return serialdata;
}
