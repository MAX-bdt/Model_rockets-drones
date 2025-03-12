/*
Useful functions for convenient and repetitive operations
*/

//function to read values from I2C sensor
int16_t getReadings(uint8_t sensor, uint8_t adress, uint8_t size_data){
  Wire.beginTransmission(sensor);
  Wire.write(adress);
  Wire.endTransmission();
  int16_t res = 0;
  Wire.requestFrom(sensor,(uint8_t)size_data); //gets data of a defined size but
  res = Wire.read();
  while(Wire.available()){
    res = res << 8 | Wire.read();//adds up the bits of data until the data size is reached
  }
  return res;
}

//function to search in sensor (specify its adress in 2nd argument) + adress
void writeValueInside(uint8_t sensorAdress ,uint8_t adressValue, uint8_t value){
  Wire.beginTransmission(sensorAdress);
  Wire.write(adressValue);
  Wire.write(value);
  Wire.endTransmission();
}

//function to verify if LIPO has enough charge for flight (charge>60%)
void LIPOpowerOK(){
  float voltage = (float)analogRead(LIPO)*10/1023;//CAUTION, the float here is mandatory as analogread usually returns an int
  //analogRead(returns a 10bit INT and MAX voltage for a GPIO is 5V so you get 5/1023 (1023=2^10-1 because signed int) to convert raw data to voltage
  //then I use two 10kOhm resistors for a 1/2 voltage divider so x2 raw voltage to get real voltage
  chargeBatt = (-59*voltage*voltage) + (1016*voltage) -4272;
  //polynomial function to get capacity from measured voltage for a 2S LIPO battery (see excel doc 2S LIPO)
  if (chargeBatt<MinChargeBatt){//check if engough charge left for flight 
    EXITstatus();
  }
}

/*NOT USED but helpful for other projects
//function to verify if the whole checklist has been successful 
void verifyChecklist(uint8_t numTests){
  uint8_t c=0;
  for(uint8_t i=0 ; i<numTests ; i++){
    if (verificationList[i]==1){
      c++;
    }
  }
  if(c!=numTests){//if one of the tests resulted in a failure >>> exit status
    EXITstatus();
  }
}
*/

//function to check if parachute should be deployed
void paraCheck(){
  //if max altitude reached or chrono timeout >>> parachute deployment
  if (millis()-liftOffTime >= paraDelay){ //realAltitudeBarometer >= paraAltitude || this patrt to check para deploy with altitude has been removed due to the unreliable behaviour of the BARO
    hatch.write(AngleBaseHatch+AngleOpenHatch);//open the hatch
    if (paraToken){
      paraToken=0;
      dataFile.flush();//flush at parachute deployment only !
    }
    //external signals descent phase
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_R, HIGH);
    //tone(buzzer, 1000);
    digitalWrite(buzzer, LOW);
  }
  //else normal in-flight LED & buzzer signals
  else{
    currentMillis = millis();
    if ((currentMillis - previousMillisFlight) >= flightInterval) {
      // save the last time you blinked the LED & buzzer
      previousMillisFlight = currentMillis;
      // if the LED & buzzer are off turn it on and vice-versa:
      if (blinkStateFlight == LOW) {
        blinkStateFlight = HIGH;
        } else {
          blinkStateFlight = LOW;
          }
      // set the LED & buzzer with the ledState of the variable:
      digitalWrite(LED_PIN_G, blinkStateFlight);
      digitalWrite(buzzer, blinkStateFlight);
      }
  }
}

//returns the rounded number as an int16_t
int16_t roundnumber(float numfloat, int16_t numint) {
  if (numfloat >= 0) {
    numint = (int16_t)(numfloat + 0.5); // Round up if decimal >= 0.5
  } 
  else {
    numint = (int16_t)(numfloat - 0.5); // Round down if decimal < -0.5
  }
  return numint;
}