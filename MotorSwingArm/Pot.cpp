#include "Pot.h"
#include "Arduino.h"

int sensorVal = 0;
int val;

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
      val = map(sensorVal, 0, 1023, 0, 255);
      //Serial.println(val);
      //_delay_ms(1000);
      return val;
}
