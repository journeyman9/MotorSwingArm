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
int32_t u_p;
float u_i;
float u_d;
int32_t dTerm;
int32_t previousError;
int32_t previousPos;
int32_t filter;


unsigned long previousMillis;
unsigned long currentMillis;
unsigned long duration;
int loopTime = 1; // 1 sec

float serialdata;
char serialCMD;
int inbyte;
float P;
float I;
float D;
float N;
float W;
float A;
int switchnum;
int inChar;
String inString = "";

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
  currentMillis = millis();
  duration = currentMillis-previousMillis;
  if (duration >= loopTime)
  {
    if (Serial.available())
   {
     Command();
   } 
  
    posSet = 4.71*A + 127;
    pos = feedback.readFeedback();
    /* A2D min - 42
     *  A2D max - 1020
     *  A2D middle - 544
     *  A2D 0 - 127
     *  A2D 180 - 975
     */
  
     // Software limit switches for position
     if ((pos >= 900) || (pos <= 155)) // for 30 and 150, 767 340
     {
      u = 0;
      P = I = D = N = W = 0;
      Serial.println("Fault detected, Motor Stopped");
      _delay_ms(500);
     } 
     
    error = posSet - pos;
    
    // Proportional
    u_p = P*error;
    
    // Integral
    u_i += (error)*.021;
    u_i *= I;
    // Serial.print("u_i before: ");
    // Serial.println(u_i);
  
    // Derivative of error
    //dTerm = error - previousError;
    //previousError = error;
    // derivative on measurement, remember u_d is negative 
    //dTerm = pos - previousPos;
    //previousPos = pos;
    //u_d = D*dTerm/.021;

    // Filter derivative
    //u_d = (D*error - filter)*N;
    //filter += .021*u_d;
    //Serial.print("Filter term: ");
    //Serial.println(u_d);
    
    // command
    u = u_p + u_i + u_d;
    // Serial.print("u before: ");
    // Serial.println(u);
    
    if (u > 255)
    {
      // Saturate for anti-wind up
      u_i -= u - 255;
      //Saturate command
      u = 255;
      // Saturate for anti-wind up
      //u_i = u-P*error;
     
    }
      else if (u < -255)
      { 
        // Saturate for anti-wind up
        u_i += -255 - u;
        // Saturate command
        u = -255;
        // Saturate for anti-wind up
        //u_i = u+P*error; 
      }
    
    if (u >= 0)
    {
      rightMotor.forward(u);
    }
      else if (u < 0)
      {
        rightMotor.backward(-u);
      } 
      
    // Check loop duration
    previousMillis = currentMillis;
    //Serial.print("loop duration: ");
    //Serial.println(duration);
    /*
    Serial.print("u_i: ");
    Serial.println(u_i);
    Serial.print("u: ");
    Serial.println(u);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("u_d: ");
    Serial.println(u_d);
    Serial.println("------"); 
    _delay_ms(500);
    */
  }
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

float getSerial()
{
 serialdata = 0;
 inChar = 0;
while (inChar != '\r')
{
   inChar = Serial.read();
//Serial.print("inChar: ");
//Serial.println(inChar);
  if (inChar >= 0 && inChar != '\r' ) 
  { 
    inString += (char)inChar;
  }
   else if (inChar >=0) 
   {
     serialdata = inString.toFloat();
//Serial.print("inString: ");
//Serial.print(inString);
//Serial.print("serialdata: ");
//Serial.println(serialdata);
     inString = "";
    }
}
return serialdata;
}

