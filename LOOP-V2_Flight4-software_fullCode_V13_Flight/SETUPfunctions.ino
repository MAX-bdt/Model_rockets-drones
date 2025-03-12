/*
Functions related to SETUP AND PROGRAM INITIALISATION
*/

void initAll(){
  //LED INITIALISATION
  pinMode(LED_PIN_G, OUTPUT);
  digitalWrite(LED_PIN_G, LOW);
  pinMode(LED_PIN_Y, OUTPUT);
  digitalWrite(LED_PIN_Y, LOW);
  pinMode(LED_PIN_R, OUTPUT);
  digitalWrite(LED_PIN_R, LOW);
  //BUZZER INITIALISATION
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  //SERVO INITIALISATION
  gimbalX.attach(pinServoX);
  gimbalZ.attach(pinServoZ);
  hatch.attach(pinServoHatch);
  //IMU reset
  writeValueInside(IMUadress ,0x6B, 0x00);//resets the sensor
  //BARO INITIALISATION
  //reset the sensor (BMP180)
  Wire.beginTransmission(BMP180adress);
  Wire.write(0xE0);  //reset register
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(50);
  callSensor4readings(BMP180adress, 0xAA);//first register of EEPROM location for utils constants
  Wire.requestFrom(BMP180adress, (uint8_t)22);
  ac1= Wire.read() << 8 | Wire.read();
  ac2= Wire.read() << 8 | Wire.read();
  ac3= Wire.read() << 8 | Wire.read();
  ac4= Wire.read() << 8 | Wire.read();
  ac5= Wire.read() << 8 | Wire.read();
  ac6= Wire.read() << 8 | Wire.read();
  b1= Wire.read() << 8 | Wire.read();
  b2= Wire.read() << 8 | Wire.read();
  mb= Wire.read() << 8 | Wire.read();
  mc= Wire.read() << 8 | Wire.read();
  md= Wire.read() << 8 | Wire.read();
  //SD INITIALISATION
  pinMode(pinSD, OUTPUT);
  SD.begin(pinSD);
  dataFile = SD.open("DATALOG.csv", FILE_WRITE);
  dataFile.close();//opens/closes file to save it in the card
  //JACK INITIALISATION
  pinMode(jack, INPUT);
}

void startup(){
  digitalWrite(LED_PIN_Y, HIGH);
  buzzerSound();
  gimbalX.write(AngleBaseX);
  delay(100);
  gimbalZ.write(AngleBaseZ);
  delay(100);
  hatch.write(AngleBaseHatch);
  delay(10);
  dataFile = SD.open("DATALOG.csv", O_WRITE);
}


void checklist(){
  digitalWrite(LED_PIN_Y, LOW);
  blinkLED(LED_PIN_R, 200, 5);
  blinkLED(LED_PIN_Y, 200, 5);
  blinkLED(LED_PIN_G, 200, 5);
  digitalWrite(LED_PIN_Y, HIGH);
  delay(2000);
  
  engineFullRangeTest();
    
  digitalWrite(LED_PIN_R, HIGH);
  jackConnected();
  digitalWrite(LED_PIN_R, LOW);
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);
  
  LIPOpowerOK();
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);
  
  IMUdataOK();
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);
  
  BAROdataOK();
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);
  
  SDwriteOK();
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);
   
  jackConnected();
  blinkLED(LED_PIN_G, 200, 5);
  delay(2000);

  buzzerBlink(1000, 200, 6);
  digitalWrite(LED_PIN_R, HIGH);
  delay(5000);//TIME TO GET AWAY FROM THE PAD DURING SENSORS INITIALISATION
}

void waitingState(){
  bool jackPlugged=1;
  while (jackPlugged !=0){
    currentMillis = millis();
    if (currentMillis - previousMillisWait >= waitInterval) {
    // save the last time you blinked the LED & buzzer
    previousMillisWait = currentMillis;
    // if the LED & buzzer are off turn it on and vice-versa:
    if (blinkState == LOW) {
      blinkState = HIGH;
    } else {
      blinkState = LOW;
    }
    // set the LED & buzzer with the ledState of the variable:
    digitalWrite(LED_PIN_G, blinkState);
    digitalWrite(buzzer, blinkState);
    
    if (digitalRead(jack)==0){//jack plugged
      jackPlugged=0;
      }
    }
  }
}
