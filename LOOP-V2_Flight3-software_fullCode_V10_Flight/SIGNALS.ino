/*
Functions related to EXTERNAL SIGNALS (LED && BUZZER)
*/

//function to make Close Encounters of the Third Kind melody for  startup
void buzzerSound(){
  uint8_t delaysound=255;
  tone(buzzer, 587);//D
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 659);//E
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 523);//C
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 261);//C down 
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound);  
  tone(buzzer, 784);//G
  delay(delaysound);
  noTone(buzzer); 
}

//blink LED function
void blinkLED(uint8_t LEDpin, uint16_t delayblink, uint8_t iteration) {
  for (uint8_t i=0 ; i < iteration ; i++){
    digitalWrite(LEDpin, HIGH);
    delay(delayblink);
    digitalWrite(LEDpin, LOW);
    delay(delayblink);
  }
}

//blink Buzzer function
void buzzerBlink(uint16_t freq, uint16_t delaybuzz, uint8_t iteration){
  for (uint8_t k=0 ; k<iteration ; k++){
    tone(buzzer, freq);
    delay(delaybuzz);
    noTone(buzzer);
    delay(delaybuzz);
  }
}
