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

//function to write first raw of data as a legend for data below in SD
void SDwriteLegend(){
  dataFile = SD.open("dataLog.csv", FILE_WRITE);
  dataFile.print("time");
  dataFile.print(",");
  dataFile.print("data");
  dataFile.print("\n");
}

//function to write data on the SD card (during flight)
void writeSD(){
  dataFile.print(millis()-liftOffTime);
  dataFile.print(",");
  dataFile.print("test");
  dataFile.print("\n");
  dataFile.flush();
}
