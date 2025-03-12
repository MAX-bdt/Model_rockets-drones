/*
Functions related to BAROMETER (Altimeter)
*/

//function that verifies if BARO Data is correct
void BAROdataOK(){
  BAROsignals();
  if(pressure<700){
    EXITstatus();
    }
}

void initialErrorBARO(){
  for( uint16_t i=0; i<initialNUM; i++){
    BAROsignals();
    determineAltitude();
    initialAltitude=initialAltitude+altitudeBarometer;
  }
  initialAltitude=initialAltitude/initialNUM;
  blinkLED(LED_PIN_G, 200, 5);
}

  
void BAROsignals(){
  //read uncompensated temperature values
  writeValueInside(BMP180adress,0xF4,0x2E);
  delay(5);
  callSensor4readings (BMP180adress, 0xF6);
  Wire.requestFrom(BMP180adress, (uint8_t)2);
  ut = Wire.read() << 8 | Wire.read();
  //read uncompensated pressure values
  writeValueInside(BMP180adress,0xF4,(0x34+(OSS<<6)));
  delay(5);
  callSensor4readings (BMP180adress, 0xF6);
  Wire.requestFrom(BMP180adress, (uint8_t)3);
  up = ((long)Wire.read() << 16 | (long)Wire.read() << 8 | (long)Wire.read()) >> (8-OSS);
  //convert teperature values from raw to physics units
  x1=(long)((ut - ac6)*ac5/p2_15);
  x2=mc*p2_11/(x1+md);
  b5=x1+x2;
  T=(b5+8)/p2_4;//in 0.1 deg
  temperature=(float)T/10;
  //convert pressure values from raw to physics units
  b6=b5-4000;
  x1=(b2*(b6*b6/p2_12))/p2_11;
  x2=ac2*b6/p2_11;
  x3=x1+x2;
  b3=(((ac1*4+x3)<<OSS)+2)/4;
  x1=ac3*b6/p2_13;
  x2=(b1*(b6*b6/p2_12)/p2_16);
  x3=((x1+x2)+2)/4;
  b4=ac4*(unsigned long)(x3+32768)/p2_15;
  b7=((unsigned long)up-b3)*(50000>>OSS);
  if (b7<0x80000000){
    p=(b7*2)/b4;
  }
  else{
    p=(b7/b4)*2;
  }
  x1=(p/p2_8)*(p/p2_8);
  x1=(x1*3038)/p2_16;
  x2=(-7357*p)/p2_16;
  p=p+(x1+x2+3791)/p2_4;
  pressure=p;//in Pa
}

//function to calculate altitude using pressure (physics formula standard atm linearised as the rocket won't fly that high...)
void determineAltitude(){
  //h-h0=h=-(R*T0)/(g*M) * ln(P/P0) (if h0=0) => DL1 : h= -(R*T0)/(g*M*P0) * P (linear relation)
  //-(R*T0)/(g*M*P0) = -0.08 (pour R=8.31N.m/mol.K ; T0=293K (20°C) ; g=9.81m/s^2 ; M=0.029kg/mol)
  altitudeBarometer=-0.08*pressure;  //RELATIVE altitude... in m
  //ODG : -1hPa every 8m = OK
}

//BAROsignals() function split in 3 to higher loop speed by performing less calculations on each loop (& enables to skip delays!):
void BAROsignals_0(){
  //read uncompensated temperature values
  writeValueInside(BMP180adress,0xF4,0x2E);
}

void BAROsignals_1(){
  callSensor4readings (BMP180adress, 0xF6);
  Wire.requestFrom(BMP180adress, (uint8_t)2);
  ut = Wire.read() << 8 | Wire.read();
  //read uncompensated pressure values
  writeValueInside(BMP180adress,0xF4,(0x34+(OSS<<6)));
}

void BAROsignals_2(){
  callSensor4readings (BMP180adress, 0xF6);
  Wire.requestFrom(BMP180adress, (uint8_t)3);
  up = ((long)Wire.read() << 16 | (long)Wire.read() << 8 | (long)Wire.read()) >> (8-OSS);
  //convert teperature values from raw to physics units
  x1=(long)((ut - ac6)*ac5/p2_15);
  x2=mc*p2_11/(x1+md);
  b5=x1+x2;
  T=(b5+8)/p2_4;//in 0.1 deg
  temperature=(float)T/10;
  //convert pressure values from raw to physics units
  b6=b5-4000;
  x1=(b2*(b6*b6/p2_12))/p2_11;
  x2=ac2*b6/p2_11;
  x3=x1+x2;
  b3=(((ac1*4+x3)<<OSS)+2)/4;
  x1=ac3*b6/p2_13;
  x2=(b1*(b6*b6/p2_12)/p2_16);
  x3=((x1+x2)+2)/4;
  b4=ac4*(unsigned long)(x3+32768)/p2_15;
  b7=((unsigned long)up-b3)*(50000>>OSS);
}

void BAROsignals_3(){
  if (b7<0x80000000){
    p=(b7*2)/b4;
  }
  else{
    p=(b7/b4)*2;
  }
  x1=(p/p2_8)*(p/p2_8);
  x1=(x1*3038)/p2_16;
  x2=(-7357*p)/p2_16;
  p=p+(x1+x2+3791)/p2_4;
  pressure=p;//in Pa

  determineAltitude();
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
  if (baroCounter == 3){
    BAROsignals_3();//measure altitude of the rocket calculation 2 (final value measurement)
  }
  baroCounter=baroCounter+1;
  if (baroCounter == 4){
    baroCounter=0;
  }
}