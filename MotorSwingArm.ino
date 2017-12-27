int main() {
    //Timer 1, Mode 14 with ICR1 - period OCR1A/B - duty
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
  //set non-inverting
  TCCR1A |= (1 << COM1B1);
  //set period
  ICR1 = 39999;
  //set output to PB2 or pin 10
  DDRB = (1 << 2);
  char input;
  Serial.begin(9600);
  Serial.flush();
  Serial.println("Press g for go and s for stop");
  Serial.println("Press g for go and s for stop");
  Serial.println("Press g for go and s for stop");
  
    while (1) {
      if (Serial.available()){
        input = Serial.read();
        Serial.print("I received: ");
        Serial.println(input);
      }
            
        if (input == 'g')
          {
            OCR1B = 1199;
            _delay_ms(500);
            OCR1B = 2999;
            _delay_ms(4000); 
            Serial.println("Instruction Received: go");
          }
          
          if (input == 's')
          {
            OCR1B = 0;
            _delay_ms(500);
            Serial.println("Instruction Received: stop");
          }
     /*
      //set duty cycle
      OCR1B = 1199; // move to 600 us or 90 degrees left
      _delay_ms(1000);
      OCR1B = 2099; // move to 1050 us or 45 degrees left
      _delay_ms(1000);
      OCR1B = 2999; // move to 1500 us perpendicular
      _delay_ms(1000);
      OCR1B = 3899; // move to 1950 us or 45 degrees right
      _delay_ms(1000);
      OCR1B = 4799; // move to 2400 us or 90 degrees right
      _delay_ms(1000);
      */
      
    }
return 0;
}

