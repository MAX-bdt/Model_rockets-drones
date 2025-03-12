/*
Functions for emergency scenarios or non-nominal operations
*/
//EXIT status if stg in the checklist goes wrong
void EXITstatus(){
  while(1){
    digitalWrite(LED_PIN_G,LOW);
    digitalWrite(LED_PIN_Y,LOW);
    digitalWrite(LED_PIN_R,HIGH);
    tone(buzzer,500);
  }  
}

void rebootFlag(){
  pinMode(pinSD, OUTPUT);
  SD.begin(pinSD);
  //if the systems reeboots during flight it would first have created this file in setup and thus skip the setup to get back to the main loop
  if (SD.exists("REBflag.txt")){
    setupSkip = true;
  }
  else{
    //creates the file to detect reboot
    flagFile = SD.open("REBflag.txt", FILE_WRITE);
    flagFile.print("REBOOT");
    flagFile.close();
  }
}

//function activated if a reboot of avionics takes place during flight
void emergencyProcedure(){
  //SERVO INITIALISATION
  gimbalX.attach(pinServoX);
  gimbalZ.attach(pinServoZ);
  hatch.attach(pinServoHatch);
  gimbalX.write(AngleBaseX);
  gimbalZ.write(AngleBaseZ);
  hatch.write(AngleBaseHatch+30);//open the hatch
  
  //LED INITIALISATION
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_Y, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  //BUZZER INITIALISATION
  pinMode(buzzer, OUTPUT);
  EXITstatus();
}
