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
  ac1=getReadings(BMP180adress, 0xAA, 2);
  ac2=getReadings(BMP180adress, 0xAC, 2);
  ac3=getReadings(BMP180adress, 0xAE, 2);
  ac4=getReadings(BMP180adress, 0xB0, 2);
  ac5=getReadings(BMP180adress, 0xB2, 2);
  ac6=getReadings(BMP180adress, 0xB4, 2);
  b1=getReadings(BMP180adress, 0xB6, 2);
  b2=getReadings(BMP180adress, 0xB8, 2);
  mb=getReadings(BMP180adress, 0xBA, 2);
  mc=getReadings(BMP180adress, 0xBC, 2);
  md=getReadings(BMP180adress, 0xBE, 2);
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
