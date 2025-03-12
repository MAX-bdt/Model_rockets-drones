/*
Functions related to SD

for more info on how to get quicker speed :
https://forum.arduino.cc/t/why-is-the-sd-library-slow/49791
*/

//function that verifies if SD card is correctly mounted and well functioning
void SDwriteOK(){
   if (!SD.begin(pinSD)) {
    EXITstatus();
   }
   if (!SD.exists("DATALOG.csv")){
    EXITstatus();
   }
}
//function to write initial states of data
void SDinitialStates(){
  //fist row -> legend
  dataFile.print("batteryCharge_(%),IMUmoyRateRoll_(deg/s),IMUmoyRatePitch_(deg/s),IMUmoyRateYaw_(deg/s),IMUmoyAccX_(m/s-2),IMUmoyAccY_(m/s-2),IMUmoyAccZ_(m/s-2),BAROpressure_(Pa),BAROaltitude_(m),IMUangleErrorRoll_(deg),IMUangleErrorPitch_(deg),BAROinitialAltitude_(m)\n");
  dataFile.flush();
  //second row -> values
  dataFile.print(chargeBatt);
  dataFile.print(",");
  dataFile.print(moyRateRoll);
  dataFile.print(",");
  dataFile.print(moyRatePitch);
  dataFile.print(",");
  dataFile.print(moyRateYaw);
  dataFile.print(",");
  dataFile.print(moyAccX);
  dataFile.print(",");
  dataFile.print(moyAccY);
  dataFile.print(",");
  dataFile.print(moyAccZ);
  dataFile.print(",");
  dataFile.print(pressure);
  dataFile.print(",");
  dataFile.print(altitudeBarometer);
  dataFile.print(",");
  dataFile.print(AngleErrorRoll);
  dataFile.print(",");
  dataFile.print(AngleErrorPitch);
  dataFile.print(",");
  dataFile.print(initialAltitude);
  dataFile.print("\n");
  dataFile.print("\n");
  dataFile.flush();
}

//function to write first row of data as a legend for data below in SD
void SDwriteLegend(){
  dataFile.print("Time_(ms),Altitude_(m),AngleRoll_(deg),AnglePitch_(deg),AngleRollCommand_(deg),AnglePitchCommand_(deg)\n");
  dataFile.flush();
}

//function to write data on the SD card (during flight)
void writeSD(){
  logtime = millis()-liftOffTime;
  dataFile.print(logtime);
  dataFile.print(",");
  dataFile.print(realAltitudeBarometer);
  dataFile.print(",");
  dataFile.print(AngleRoll);
  dataFile.print(",");
  dataFile.print(AnglePitch);
  dataFile.print(",");
  dataFile.print(InputRoll);
  dataFile.print(",");
  dataFile.print(InputPitch);
  dataFile.print("\n");
  //Flush counter ; flush each "flushloopsNb" = 100 loops (=each 1sec)
  flushcount = flushcount+1;
  if (flushcount >= flushloopsNb) {
    flushcount=0;
    dataFile.flush();
  }
}
