/*
Functions related to BAROMETER (Altimeter)
*/

//function that verifies if BARO Data is correct
void BAROdataOK(){
  BAROsignals();
  if(pressure<700||altitudeBarometer==0){
    EXITstatus();
    }
}

void initialErrorBARO(){
  for( uint16_t i=0; i<initialNUM; i++){
    BAROsignals();
    initialAltitude=initialAltitude+altitudeBarometer;
  }
  initialAltitude=initialAltitude/initialNUM;
  blinkLED(LED_PIN_G, 200, 5);
}

  
void BAROsignals(){
  //NO TEMPERATURE READINGS FOR THIS FLIGHT
  //read uncompensated pressure values
  writeValueInside(BMP180adress, 0xF4, 0x34+(OSS<<8));
  delay(4);
  up = getReadings(BMP180adress, 0xF6, 3)>>(8-OSS);
  //convert pressure values from raw to physics units
  b6=b5-4000;
  x1= (b2*(b6*b6/(p2_12)))/(p2_11);
  x2=ac2*b6/(p2_11);
  x3=x1+x2;
  b3=(((ac1*4)+x3)<<OSS+2)/4;
  x1=ac3*b6/(p2_13);
  x2=(b1*(b6*b6/(p2_12))/(p2_16));
  x3=((x1+x2)+2)/4;
  b4=ac4*(uint32_t)(x3+32768)/(p2_15);
  b7=((uint32_t)up-b3)*(50000>>OSS);
  if (b7<0x80000000){pressure=(b7*2)/b4;}
  else{pressure=(b7/b4)*2;}
  x1=(pressure/(p2_8)*(pressure/(p2_8)));
  x1=(x1*3038)/(p2_16);
  x2=(-7357*pressure)/(p2_16);
  pressure=2*(((pressure+(x1+x2+3791)/(p2_4))*2)-18100);//multiplied by two to work fine !! /in Pa for more precision/ -18100 is the error calculated from measured atm with weather the day of the test
  //calculates altitude using pressure (physics formula standard atm)
  altitudeBarometer=44330*(1-pow(pressure/(1013.25*100), 0.19));
  realAltitudeBarometer=altitudeBarometer-initialAltitude;
}

//BAROsignals() function split in 3 to higher loop speed by performing less calculations on each loop :

void BAROsignals_0(){
  writeValueInside(BMP180adress, 0xF4, 0x34+(OSS<<8));
  b6=b5-4000;
  x1= (b2*(b6*b6/(p2_12)))/(p2_11);
  x2=ac2*b6/(p2_11);
  x3=x1+x2;
  b3=(((ac1*4)+x3)<<OSS+2)/4;
  x1=ac3*b6/(p2_13);
  x2=(b1*(b6*b6/(p2_12))/(p2_16));
  x3=((x1+x2)+2)/4;
  b4=ac4*(uint32_t)(x3+32768)/(p2_15);
}

void BAROsignals_1(){
  up = getReadings(BMP180adress, 0xF6, 3)>>(8-OSS);
  b7=((uint32_t)up-b3)*(50000>>OSS);
  if (b7<0x80000000){pressure=(b7*2)/b4;}
  else{pressure=(b7/b4)*2;}
  x1=(pressure/(p2_8)*(pressure/(p2_8)));
  x1=(x1*3038)/(p2_16);
  x2=(-7357*pressure)/(p2_16);
  //convert pressure values from raw to physics units
  pressure=2*(((pressure+(x1+x2+3791)/(p2_4))*2)-18100);//multiplied by two to work fine !! /in Pa for more precision/ -18100 is the error calculated from measured atm with weather the day of the test
}

void BAROsignals_2(){
  //calculates altitude using pressure (physics formula standard atm)
  inpow = pressure/(1013.25*100);
  altitudeBarometer=44330*(1-pow(inpow, 0.19));
  realAltitudeBarometer=altitudeBarometer-initialAltitude;
}

//function to split BARO MEASURES :
void BAROsplitMeas(){
  if (baroCounter == 0){
    BAROsignals_0();//measure altitude of the rocket calculation 0
  }
  if (baroCounter == 1){
    BAROsignals_1();//measure altitude of the rocket calculation 1
  }
  if (baroCounter == 2){
    BAROsignals_2();//measure altitude of the rocket calculation 2 (final value measurement)
  }
  baroCounter=baroCounter+1;
  if (baroCounter == 3){
    baroCounter=0;
  }
}