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
}


void commandServos(){
  gimbalX.write(AngleBaseX-InputRoll);
  gimbalZ.write(AngleBaseZ-InputPitch);
}
