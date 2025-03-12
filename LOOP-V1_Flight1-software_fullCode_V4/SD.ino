/*
Functions related to SD
*/

//function that verifies if SD card is correctly mounted and well functioning
void SDwriteOK(){
   if (!SD.begin()) {
    EXITstatus();
   }
   if (!SD.exists("dataLog.csv")){
    EXITstatus();
   }
}
//function to write initial states of data
void SDinitialStates(){
  SD.begin();
  dataFile = SD.open("dataLog.csv", FILE_WRITE);
  //fist raw -> legend
  dataFile.print("batteryCharge_(%)");
  dataFile.print(",");
  dataFile.print("IMUmoyRateRoll_(deg/s)");
  dataFile.print(",");
  dataFile.print("IMUmoyRatePitch_(deg/s)");
  dataFile.print(",");
  dataFile.print("IMUmoyRateYaw_(deg/s)");
  dataFile.print(",");
  dataFile.print("IMUmoyAccX_(m/s-2)");
  dataFile.print(",");
  dataFile.print("IMUmoyAccY_(m/s-2)");
  dataFile.print(",");
  dataFile.print("IMUmoyAccZ_(m/s-2)");
  dataFile.print(",");
  dataFile.print("BAROpressure_(hPa)");
  dataFile.print(",");
  dataFile.print("BAROaltitude_(m)");
  dataFile.print(",");
  dataFile.print("IMUangleErrorRoll_(deg)");
  dataFile.print(",");
  dataFile.print("IMUangleErrorPitch_(deg)");
  dataFile.print(",");
  dataFile.print("BAROinitialAltitude_(m)");
  dataFile.print("\n");
  //second raw -> values
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
  dataFile.flush();
}

//function to write first raw of data as a legend for data below in SD
void SDwriteLegend(){
  dataFile = SD.open("dataLog.csv", FILE_WRITE);
  dataFile.print("Time_(ms)");
  dataFile.print(",");
  dataFile.print("Altitude_(m)");
  dataFile.print(",");
  dataFile.print("AngleRoll_(deg)");
  dataFile.print(",");
  dataFile.print("AnglePitch_(deg)");
  dataFile.print(",");
  dataFile.print("AngleRollCommand_(deg)");
  dataFile.print(",");
  dataFile.print("AnglePitchCommand_(deg)");  
  dataFile.print("\n");
  dataFile.flush();
}

//function to write data on the SD card (during flight)
void writeSD(){
  dataFile.print(millis()-liftOffTime);
  dataFile.print(",");
  dataFile.print(realAltitudeBarometer);
  dataFile.print(",");
  dataFile.print(AngleRoll);
  dataFile.print(",");
  dataFile.print(AnglePitch);
  dataFile.print(",");
  dataFile.print(AngleBaseX-InputRoll);
  dataFile.print(",");
  dataFile.print(AngleBaseZ-InputPitch);
  dataFile.print("\n");
  dataFile.flush();
}
