#include "Pot.h"
#include "Arduino.h"

unsigned long sensorVal = 0;

Pot::Pot()
{
  Serial.begin(9600);
  // Potentiometer
  //internal 5V, Right adjust, A0
  ADMUX = 0b01000000;
  //enable, start, 128 pre-scale
  ADCSRA = 0b11000111;
}

int Pot::readFeedback()
{   
      //Read potentiometer
      ADCSRA |= (1<<ADSC); //Restart conversion
      while(ADCSRA&(1<<ADSC)); //ADC conversion takes 13-260 microseconds
      sensorVal = ADC;

      return sensorVal;
}
