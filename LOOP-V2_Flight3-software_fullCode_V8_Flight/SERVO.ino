/*
Functions related to TVC SERVOS
*/

//TVC full range mvt test function
void engineFullRangeTest(){
  uint8_t MinAngle=65;
  uint8_t MaxAngle=115;
  for (uint8_t j=0; j<10; j++){//10 full range loops
    gimbalX.write(MinAngle);
    gimbalZ.write(MaxAngle);
    delay(200);
    gimbalX.write(MaxAngle);
    gimbalZ.write(MaxAngle);
    delay(200);
    gimbalX.write(MaxAngle);
    gimbalZ.write(MinAngle);
    delay(200);
    gimbalX.write(MinAngle);
    gimbalZ.write(MinAngle);
    delay(200);
  }
  gimbalX.write(AngleBaseX);
  delay(100);
  gimbalZ.write(AngleBaseZ);
  delay(100);
}


void commandServos(){
  RawCommandAngleRoll = AngleBaseX-InputRoll; //calculate order to send to servoX
  RawCommandAnglePitch = AngleBaseZ-InputPitch; //calculate order to send to servoZ
  //with the right sign !
  CommandAngleRoll = roundnumber(RawCommandAngleRoll, CommandAngleRoll); //round to an INT order to send to servoX
  CommandAnglePitch = roundnumber(RawCommandAnglePitch, CommandAnglePitch); //round to an INT order to send to servoZ
  if (straightToken < straightLoops){
    gimbalX.write(AngleBaseX); //send order 0 pos to servoX
    gimbalZ.write(AngleBaseZ); //send order 0 pos to servoZ
  }
  else{
    gimbalX.write(CommandAngleRoll); //send order to servoX
    gimbalZ.write(CommandAnglePitch); //send order to servoZ
  }
  straightToken = straightToken+1;
}
