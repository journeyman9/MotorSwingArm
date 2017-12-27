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

int sensorVal = 0;
int val;
unsigned int velocitySet;

void setup()
{
Serial.begin(9600);
// Potentiometer
//internal 5V, Right adjust, A0
ADMUX = 0b01000000;
//enable, start, 128 pre-scale
ADCSRA = 0b11000111;

//Switch Input
DDRD |= 0x00;
PORTD |= 0b00000100; // Secondary function to set pull up resistor
}

void loop()
{ 
  //Read potentiometer
  ADCSRA |= (1<<ADSC); //Restart conversion
  while(ADCSRA&(1<<ADSC)); //ADC conversion takes 13-260 microseconds
  sensorVal = ADC;
  val = map(sensorVal, 0, 1023, 0, 255);
  //Serial.println(val);
  //_delay_ms(1000);

    if ((PIND&(1<<2)) > 0)
    {
      rightMotor.forward(val);
      //Serial.println(val);
      //_delay_ms(500);
    }

    else if ((PIND&(1<<2)) == 0)
    {
      rightMotor.backward(val);
      //Serial.println(val+1);
      //_delay_ms(500);
    }

    else
    {
      rightMotor.freeRun();
      //Serial.println(val+2);
      //_delay_ms(500);
    }
}
