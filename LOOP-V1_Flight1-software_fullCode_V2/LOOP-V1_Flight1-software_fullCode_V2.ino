//GLOBAL CODE FOR LOOP_V1 FLIGHT_1 SOFTWARE
//made by Maximilien BOUDET - JULY 2024

/*------------WIRING-------------------
I2C devices :
  pin IMU MPU6050 :
  SDA-> A4 Arduino NANO
  SCL-> A5 Arduino NANO
  VCC-> 5V Arduino NANO
  GND-> GND Arduino NANO

  pin BAROMETER BMP180 :
  SDA-> A4 Arduino NANO
  SCL-> A5 Arduino NANO
  VCC-> 5V Arduino NANO
  GND-> GND Arduino NANO
      -----------
SERVOS :
  pin SERVO X & Y :
  pin S servo X-axis    -> D7 Arduino NANO
  pin S servo Z-axis    -> D8 Arduino NANO
  
  pin SERVO HATCH :
  pin S servo HATCH     -> D2 Arduino NANO
      -----------
SPI devices :
  pin MICRO-SD module :
  CS  -> D10 Arduino NANO
  SCK -> D13 Arduino NANO
  MOSI-> D11 Arduino NANO
  MISO-> D12 Arduino NANO
  VCC -> 5V Arduino NANO
  GND -> GND Arduino NANO
      -----------
EXTERNAL SIGNALS :
  pin LED :
  RED    -> A0
  YELLOW -> A1
  GREEN  -> A2

  pin BUZZER :
  +   -> D9 Arduino NANO
  GND -> GND Arduino NANO
      -----------
VITAL MEASURES :
  pin BATTERY CHARGE :
  +   -> A3 Arduino NANO
  GND -> GND Arduino NANO

  pin JACK CONNECTOR :
  +   -> D4 Arduino NANO
  GND -> GND Arduino NANO
        -----------
--------------------------------------------*/

/*------------SERVO RANGE-------------------
servo X-axis    -> 65deg - 90deg - 115deg (90deg +- 25deg)
servo Z-axis    -> 65deg - 90deg - 115deg (90deg +- 25deg)
servo HATCH     -> 
--------------------------------------------*/

/*------------SENSORS CONFIG-------------------
IMU MPU6050 sensors :
Gyro -> +/- 500deg/s
Accel -> +/- 8g
X-axis -> Roll
Z-axis -> Pitch
Control loop frequency -> 250Hz (0.004s)
------------------------------------------------*/

//Def libraries used
#include <Wire.h>
#include <Servo.h>
#include <SD.h>

//-----------------------------------------Def global variables----------------------------------------------
//-----FLIGHT PATH VARIABLES-----------
#define DesiredAngleRoll 0
#define DesiredAnglePitch 0 
#define paraAltitude 20 //max altitude for parachute deployment (in meters)
#define paraDelay 2100 //parachute deployment delay in seconds
//-----CLOCKS--------------------------
uint32_t LoopTimer=0; //parameter that contains the length of each control loop (period)
uint32_t liftOffTime=0;
//waiting state variables
uint32_t previousMillisWait=0;
uint32_t previousMillisFlight=0;
uint32_t currentMillis=0;
//EMERGENCY PROCEDURES
bool setupSkip=false;
//SERVO VARIABLES
Servo gimbalX;
Servo gimbalZ;
Servo hatch;
#define pinServoX 7//D7 pin
#define AngleBaseX 90
#define pinServoZ 8//D8 pin
#define AngleBaseZ 90
#define pinServoHatch 2//D2 pin
#define AngleBaseHatch 90
//CHECKLIST VARIABLES
bool verificationList[]={0,0,0,0,0,0,0,0};//8tests in the current checklist
//EXTERNAL SIGNALS VARIABLES(LED/Buzzer)
#define LED_PIN_G A2 //A2 pin
#define LED_PIN_Y A1 //A1 pin
#define LED_PIN_R A0 //A0 pin
#define buzzer 9//D9 pin
#define waitInterval 1000
#define flightInterval 150
bool blinkState = LOW;             // blinkState used to set the LED & buzzer in waiting loop
bool blinkStateFlight = LOW;       // blinkState used to set the LED & buzzer in flight loop
//JACK VARIABLES
#define jack 4//D4 pin
//SENSOR VARIABLES
//LIPO measure port variable
#define LIPO A3//A3 pin
//IMU variables
#define IMUadress 0x68 //IMU sensor I2C ADRESS
float RateRoll=0, RatePitch=0, RateYaw=0;
float RateCalibrationRoll=0, RateCalibrationPitch=0, RateCalibrationYaw=0;
int16_t RateCalibrationNumber=0;
float AccX=0, AccY=0, AccZ=0;
float AngleErrorRoll=0, AngleErrorPitch=0;
float AngleRoll=0;
float AnglePitch=0;
float CorrectAngleRoll=0;
float CorrectAnglePitch=0;
float CorrectAngleRollPos=0;
float CorrectAnglePitchPos=0;
//BARO variables
#define BMP180adress 0x77 //BARO sensor I2C ADRESS
int16_t ac1;
int16_t ac2;
int16_t ac3;
int16_t ac4;
int16_t ac5;
int16_t ac6;
int16_t b1;
int16_t b2;
int16_t mb;
int16_t mc;
int16_t md;
int32_t b5; 
//define variables for pressure and temperature calculation
int32_t x1,x2;
//define variables for pressure calculation
int32_t x3,b3,b6,p;
uint32_t b4,b7;
//define variables for pressure reading
uint32_t pressure;
float altitudeBarometer;
float initialAltitude=0;
float realAltitudeBarometer;
#define OSS 0  // OverSampling Setting = precision setting
/* bmp180 page 12 Datasheet
OSS=0 ultra Low Power Setting,  1 sample, 4.5 ms 3uA
OSS=1 Standard Power Setting,   2 samples, 7.5 ms 5uA
OSS=2 High Resolution,          4 samples, 13.5 ms 7uA<<<<<<<<<<< selected
OSS=3 Ultra High Resolution,    2 samples, 25.5 ms 12uA
*/
//SD VARIABLES
File dataFile;
File flagFile;
#define pinSD 10 //D10 >>> CS = Chip Select for SPI bus used by micro SD module
//KALMAN INITIAL VALUES 
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2; //initial angle roll /initial uncertainty in angle (set as a pointer*) 
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2; //initial angle pitch /initial uncertainty in angle
float Kalman1DOutput[]={0,0}; //initial output for the Kalman filter: contains prediction angle/uncertainty on the angle)
float Ts=0.02; // time frame in the Kalman function 
               //(ratio between resposivity and filtering noise: +Ts >>> +responsivity = +noise (on the initial code Ts was set to 0.004)
//KALMAN UNCERTAINTIES
float AccStdDev=3; //accelero standard deviation = 3 deg
float GyroStdDev=4; //gyro standard deviation rate = 4 deg/s
//INITILISATION TEST
#define initialNUM 1000 //number of values measured for initialisation of IMU & BARO
//---------------------PID VARIABLES---------------------------
float ErrorRoll, ErrorPitch;
float InputRoll, InputThrottle, InputPitch;
float PrevErrorRateRoll, PrevErrorRatePitch;
float PrevItermRateRoll, PrevItermRatePitch;
float PrevDtermRateRoll, PrevDtermRatePitch;
float PIDReturn[]={0, 0, 0, 0};

#define Tpid 0.004 //sets the delay in seconds for calculation of discret integrals in PID (here 0.004s>>>250Hz)
#define MAXtvc 25  //defines MAX (or MIN = -MAXtvc) for TVC servos range of mvt (>>> can be seen as saturation limit for a DC motor)
float Wcut= 2*3.142*10; // cutoff amplitude (or frequency if divided by 2*pi) of the lowpass filter in derivative calculation in PID function
//------------PID GAINS--- define them when PID tuning --------------
#define PRateRoll 1
#define IRateRoll 0.02  
#define DRateRoll 0.1 
#define PRatePitch 1
#define IRatePitch 0.02
#define DRatePitch 0.1
//RQ : those variables will need specific tuning to be exactly suited for the loop rocket

//ADAPT THE LOOP FREQUENCY HERE :
#define LoopMAXTime 4000 //loop time in µs (microseconds)(here 4000µs = 0.004s -> 250Hz loop)
//CAUTION : LOOPMAXTIME (in µs) MUST BE EQUAL TO Tpid (in s) !!!


//-----------------------------------------Call functions------------------------------------------------------
void initAll();
void startup();              
void checklist();            
void initialErrorIMU();      
void initialErrorBARO();       
void waitingState();
void IMUsignals();
void KALMANfilter();
void PID();
void commandServos();
void BAROsignals();
//-----------------------------------------Def SECONDARY functions---------------------------------------------
//EXIT status if stg in the checklist goes wrong
void EXITstatus(){
  while(1){
    digitalWrite(LED_PIN_G,LOW);
    digitalWrite(LED_PIN_Y,LOW);
    digitalWrite(LED_PIN_R,HIGH);
    tone(buzzer,500);
  }  
}

//function to read values from I2C sensor
int16_t getReadings(uint8_t sensor, uint8_t adress, uint8_t size_data){
  Wire.beginTransmission(sensor);
  Wire.write(adress);
  Wire.endTransmission();
  int16_t res = 0;
  Wire.requestFrom(sensor,(uint8_t)size_data); //gets data of a defined size but
  res = Wire.read();
  while(Wire.available()){
    res = res << 8 | Wire.read();//adds up the bits of data until the data size is reached
  }
  return res;
}

//function to search in sensor (specify its adress in 2nd argument) + adress
void writeValueInside(uint8_t sensorAdress ,uint8_t adressValue, uint8_t value){
  Wire.beginTransmission(sensorAdress);
  Wire.write(adressValue);
  Wire.write(value);
  Wire.endTransmission();
}

//function to make Close Encounters of the Third Kind melody for  startup
void buzzerSound(){
  uint8_t delaysound=255;
  tone(buzzer, 587);//D
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 659);//E
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 523);//C
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound); 
  tone(buzzer, 261);//C down 
  delay(delaysound);        
  noTone(buzzer);     
  delay(delaysound);  
  tone(buzzer, 784);//G
  delay(delaysound);
  noTone(buzzer); 
}

//blink LED function
void blinkLED(uint8_t LEDpin, uint16_t delayblink, uint8_t iteration) {
  for (uint8_t i=0 ; i < iteration ; i++){
    digitalWrite(LEDpin, HIGH);
    delay(delayblink);
    digitalWrite(LEDpin, LOW);
    delay(delayblink);
  }
}

//blink Buzzer function
void buzzerBlink(uint16_t freq, uint16_t delaybuzz, uint8_t iteration){
  for (uint8_t k=0 ; k<iteration ; k++){
    tone(buzzer, freq);
    delay(delaybuzz);
    noTone(buzzer);
    delay(delaybuzz);
  }
}

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

//function to verify jack connection
void jackConnected(){
  bool jackUnplugged =0;
  while (jackUnplugged !=1){
    if (digitalRead(jack)==1){//jack plugged
      jackUnplugged=1;
      }
  }
}

//function to verify if LIPO has enough charge for flight (charge>60%)
void LIPOpowerOK(){
  float voltage = (float)analogRead(LIPO)*10/1023;//CAUTION, the float here is mandatory as analogread usually returns an int
  //analogRead(returns a 10bit INT and MAX voltage for a GPIO is 5V so you get 5/1023 (1023=2^10-1 because signed int) to convert raw data to voltage
  //then I use two 10kOhm resistors for a 1/2 voltage divider so x2 raw voltage to get real voltage
  float chargeBatt = -59*voltage*voltage + 1016*voltage -4272;
  //polynomial function to get capacity from measured voltage for a 2S LIPO battery (see excel doc 2S LIPO)
  if (chargeBatt<60){//not engough charge left for flight (less than 60%)
    EXITstatus();
  }
}

//function that verifies if IMU Data is correct
void IMUdataOK(){
  float moyRateRoll=0;
  float moyRateYaw=0;
  float moyRatePitch=0;
  float moyAccX=0;
  float moyAccY=0;
  float moyAccZ=0;
  for (uint16_t i=0 ; i<initialNUM ; i++){
    IMUsignals();
    moyRateRoll=moyRateRoll+RateRoll;
    moyRateYaw=moyRateYaw+RateYaw;
    moyRatePitch=moyRatePitch+RatePitch;
    moyAccX=moyAccX+AccX;
    moyAccY=moyAccY+AccY;
    moyAccZ=moyAccZ+AccZ;
  }
  moyRateRoll=moyRateRoll/initialNUM;
  moyRateYaw=moyRateYaw/initialNUM;
  moyRatePitch=moyRatePitch/initialNUM;
  moyAccX=moyAccX/initialNUM;
  moyAccY=moyAccY/initialNUM;
  moyAccZ=moyAccZ/initialNUM;
  
  if(moyRateRoll==0||moyRateYaw==0||moyRatePitch==0||moyAccX==0||moyAccY==0||moyAccZ==0){//tests if one of the values isn't well recorded
    EXITstatus();
    }
}

//function that verifies if BARO Data is correct
void BAROdataOK(){
  BAROsignals();
  if(pressure<700||altitudeBarometer==0){
    EXITstatus();
    }
}

//function that verifies if SD card is correctly mounted and well functioning
void SDwriteOK(){
   if (!SD.begin()) {
    EXITstatus();
   }
   if (!SD.exists("dataLog.csv")){
    EXITstatus();
   }
}


//function to verify if the whole checklist has been successful 
void verifyChecklist(uint8_t numTests){
  uint8_t c=0;
  for(uint8_t i=0 ; i<numTests ; i++){
    if (verificationList[i]==1){
      c++;
    }
  }
  if(c!=numTests){//if one of the tests resulted in a failure >>> exit status
    EXITstatus();
  }
}

//-----------------------------------------Def PRINCIPAL functions---------------------------------------------
void initAll(){
  //library startup
  Wire.setClock(400000);
  Wire.begin();
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
  //IMU INITIALISATION
  writeValueInside(IMUadress ,0x6B, 0x00);//resets the sensor
  //gyro callibration loop
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    IMUsignals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
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
  dataFile = SD.open("dataLog.csv", FILE_WRITE);
  dataFile.close();
  //JACK INITIALISATION
  pinMode(jack, INPUT);
}

void rebootFlag(){
  //if the systems reeboots during flight it would first have created this file in setup and thus skip the setup to get back to the main loop
  if (SD.exists("REBflag.txt")){
    setupSkip = true;
  }
  else{
    //creates the file to detect reboot
    SD.begin(pinSD);
    flagFile = SD.open("REBflag.txt", FILE_WRITE);
    flagFile.print("REBOOT");
    flagFile.close();
  }
}


void startup(){
  digitalWrite(LED_PIN_Y, HIGH);
  buzzerSound();
  gimbalX.write(AngleBaseX);
  delay(100);
  gimbalZ.write(AngleBaseZ);
  delay(100);
  hatch.write(AngleBaseHatch);
  dataFile = SD.open("dataLog.csv", FILE_WRITE);//re-open main file
}

void checklist(){
  digitalWrite(LED_PIN_Y, LOW);
  blinkLED(LED_PIN_R, 200, 5);
  blinkLED(LED_PIN_Y, 200, 5);
  blinkLED(LED_PIN_G, 200, 5);
  digitalWrite(LED_PIN_Y, HIGH);
  verificationList[0]=1;
  delay(1000);
  
  engineFullRangeTest();
  verificationList[1]=1;
  
  digitalWrite(LED_PIN_R, HIGH);
  jackConnected();
  digitalWrite(LED_PIN_R, LOW);
  verificationList[2]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);
  
  LIPOpowerOK();
  verificationList[3]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);
  
  IMUdataOK();
  verificationList[4]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);
  
  BAROdataOK();
  verificationList[5]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);
  
  SDwriteOK();
  verificationList[6]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);
   
  jackConnected();
  verificationList[7]=1;
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);

  verifyChecklist(8);//verifies a checklist composed of 8 tests
  blinkLED(LED_PIN_G, 200, 5);
  delay(1000);

  buzzerBlink(1000, 200, 4);
  digitalWrite(LED_PIN_R, HIGH);
}

void initialErrorBARO(){
  for( uint16_t i=0; i<initialNUM; i++){
    BAROsignals();
    initialAltitude=initialAltitude+altitudeBarometer;
  }
  initialAltitude=initialAltitude/initialNUM;
  blinkLED(LED_PIN_G, 200, 5);
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

//KALMAN filter function
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) { //function inputs
  KalmanState=KalmanState+ Ts*KalmanInput; //predicts current state of the system
  KalmanUncertainty=KalmanUncertainty + Ts * Ts * 4 * 4; //calculate uncertainty of the prediction
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //calculate Kalman Gain from uncertainties on the prediction and measurements
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState); //updates the predicted state using kalman gain and measurements
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty; //updates the uncertainties on the predicted state
  Kalman1DOutput[0]=KalmanState; //OUTPUTS to get the function to work one more time (recursive function)
  Kalman1DOutput[1]=KalmanUncertainty;
  //the value Ts has an impact on the ratio between responsivity and noise cutting on the filter : Ts is defined with useful variables
  // + responsivity = - filtering (+noise)
  // - responsivity = + filtering (-noise)
}

//PID function that calculates PID equations for Roll and Pitch :
void pidEquation(float Error, float P , float I, float D, float PrevError, float PrevIterm, float PrevDterm) {
  float Pterm=P*Error;                                // Proportional corrector
  float Iterm=PrevIterm+I*(Error+PrevError)*Tpid/2;  // Integral corrector >>> Tpid is the measurement sample time called "Ts" in def PID sheet (initially set to 0.004) >>> modify this term accordingly
  if (Iterm > MAXtvc-2) Iterm=MAXtvc-2;                         // Define range for Integral corrector, it is the same range as the one for servos (MAXtvc-1=9deg)
  else if (Iterm <-MAXtvc-2) Iterm=MAXtvc-2;                   // >>> ANTI-WINDUP : if Iterm is above max >>> stop correction with Iterm
  float Dterm=((D*(Error-PrevError)/Tpid)+((1/Wcut)*PrevDterm)/Tpid)*(Tpid/(Tpid+(1/Wcut))); // Derivative corrector + low pass filter
  // Derivative corrector >>> Tpid is the measurement sample time called "Ts" in def PID sheet (initially set to 0.004) >>> modify this term accordingly
  
  float PIDOutput= Pterm+Iterm+Dterm;                 // Sum P,I,D to get PID corrector
  
  if (PIDOutput>MAXtvc) PIDOutput=MAXtvc;                   // High Limit in output >>> adapt to MAX range of TVC mount !!!
  else if (PIDOutput <-MAXtvc) PIDOutput=-MAXtvc;           // Low Limit in output >>> adapt to MIN range of TVC mount !!!
  
  PIDReturn[0]=PIDOutput;                             // PID output tuple 1st varible = sent for command in servo
  PIDReturn[1]=Error;                                 // PID output tuple 2nd varible = error used in derivative and integral passing the loop again
  PIDReturn[2]=Iterm;                                 // PID output tuple 3rd varible = Iterm used in integral passing the loop again
  PIDReturn[3]=Dterm;
}

/*//function to reset PID values if needed (NOT USED yet)
void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; 
  PrevItermRateRoll=0; PrevItermRatePitch=0;
  PrevDtermRateRoll=0; PrevDtermRatePitch=0; 
}
*/

//collect IMU DATA function
void IMUsignals() {
  writeValueInside(IMUadress ,0x1A, 0x05);//asks for filtered accelero data and NO temperature data asked
  writeValueInside(IMUadress ,0x1C, 0x10);//configures to +-8g
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);//asks ACCELERO measurements
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
                                                    //collects RAW ACCELERO DATA
  writeValueInside(IMUadress ,0x1B, 0x8);//configures to +-500deg/s     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);//asks GYRO measurements
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
                                                      //collects RAW GYRO DATA
  RateRoll=(float)GyroX/65.5;
  RateYaw=(float)GyroY/65.5;
  RatePitch=(float)GyroZ/65.5;
                                                      //converts RAW gyro data into Deg/s (using right conversion coeff>>>see datasheet)
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
                                                      //converts RAW accelero data into g (using right conversion coeff>>>see datasheet)
  AngleRoll=atan(AccZ/sqrt(AccY*AccY+AccX*AccX))*1/(3.142/180);
  AnglePitch=atan(-AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
                                                      //converts vector g deviation into angles using trigo
}

void KALMANfilter(){
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  CorrectAngleRoll=KalmanAngleRoll-AngleErrorRoll;
  CorrectAngleRollPos=-CorrectAngleRoll; //reverts the angle to set the angle with correct sign (base directe SII)
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  CorrectAnglePitch=KalmanAnglePitch-AngleErrorPitch;
  CorrectAnglePitchPos=-CorrectAnglePitch; //reverts the angle to set the angle with correct sign (base directe SII)
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
}

void PID(){
   //-----------------------PID LOOP--------------------------
  //errors fed into the PID corrector
  ErrorRoll=DesiredAngleRoll-CorrectAngleRollPos;
  ErrorPitch=DesiredAnglePitch-CorrectAnglePitchPos;

  //------------------------PID APPLIED TO ROLL----------------------------------
  pidEquation(ErrorRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0]; //ROLL OUTPUT for Servo ROLL
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
       PrevDtermRateRoll=PIDReturn[3];
  //------------------------------------------------------------------------------
  
  //------------------------PID APPLIED TO PITCH----------------------------------
  pidEquation(ErrorPitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, PrevDtermRatePitch);
       InputPitch=PIDReturn[0]; //PITCH OUTPUT for Servo PITCH
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
       PrevDtermRatePitch=PIDReturn[3];
  //-------------------------------------------------------------------------------
}

void commandServos(){
  gimbalX.write(AngleBaseX-InputRoll);
  gimbalZ.write(AngleBaseZ-InputPitch);
}
  
void BAROsignals(){
  //NO TEMPERATURE READINGS FOR THIS FLIGHT
  //read uncompensated pressure values
  writeValueInside(BMP180adress, 0xF4, 0x34+(OSS<<8));
  delay(5);
  long up = getReadings(BMP180adress, 0xF6, 3)>>(8-OSS);
  //convert pressure values from raw to physics units
  b6=b5-4000;
  x1= (b2*(b6*b6/pow(2,12)))/pow(2,11);
  x2=ac2*b6/pow(2,11);
  x3=x1+x2;
  b3=((ac1*4+x3)<<OSS+2)/4;
  x1=ac3*b6/pow(2,13);
  x2=(b1*(b6*b6/pow(2,12))/pow(2,16));
  x3=((x1+x2)+2)/pow(2,2);
  b4=ac4*(uint32_t)(x3+32768)/pow(2,15);
  b7=((uint32_t)up-b3)*(50000>>OSS);
  if (b7<0x80000000){pressure=(b7*2)/b4;}
  else{pressure=(b7/b4)*2;}
  x1=(pressure/pow(2,8)*(pressure/pow(2,8)));
  x1=(x1*3038)/pow(2,16);
  x2=(-7357*pressure)/pow(2,16);
  pressure=((pressure+(x1+x2+3791)/pow(2,4))*2)-18100;//multiplied by two to work fine !! /in Pa for more precision/ -18100 is the error calculated from measured atm with weather the day of the test
  //calculates altitude using pressure (physics formula standard atm)
  altitudeBarometer=44330*(1-pow(pressure/(1013.25*100), 1/5.255));
  realAltitudeBarometer=altitudeBarometer-initialAltitude;
}

void paraCheck(){
  //if max altitude reached or chrono timeout >>> parachute deployment
  if (realAltitudeBarometer >= paraAltitude || millis()-liftOffTime >= paraDelay){
    hatch.write(AngleBaseHatch+30);//open the hatch
    //external signals descent phase
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_R, HIGH);
    tone(buzzer, 1000);
  }
  //else normal in-flight LED & buzzer signals
  else{
    currentMillis = millis();
    if ((currentMillis - previousMillisFlight) >= flightInterval) {
      // save the last time you blinked the LED & buzzer
      previousMillisFlight = currentMillis;
      // if the LED & buzzer are off turn it on and vice-versa:
      if (blinkStateFlight == LOW) {
        blinkStateFlight = HIGH;
        } else {
          blinkStateFlight = LOW;
          }
      // set the LED & buzzer with the ledState of the variable:
      digitalWrite(LED_PIN_G, blinkStateFlight);
      digitalWrite(buzzer, blinkStateFlight);
      }
  }
}
  
void writeSD(){
  
}
//--------------------------------------------------------------------------------------------------------------
//Setup before flight
void setup() {
  Serial.begin(115200);
  initAll();              //Initialises every input/output
  delay(5);
  rebootFlag();
  if (setupSkip==false){//nominal function
    startup();              //First actions at startup
    checklist();            //Checklist function to verify each component
    initialErrorIMU();      //Initialises IMU measurements
    initialErrorBARO();     //Initialises BARO measurements
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, HIGH);
    delay(5000); 
    waitingState();         //Rocket ready >>> waiting for lift-off
    liftOffTime = millis();
    LoopTimer = micros();//timer that measures the time of each control loop >>> START
  }
  
  else{//skips setup to get to the main loop again (EMERGENCY PROCEDURE)
    waitingState();         //Rocket ready >>> waiting for lift-off
    liftOffTime = millis();
    LoopTimer = micros();//timer that measures the time of each control loop >>> START
  }
}
//---------------------------------------------------------------------------------------------------------------
//infinite Loop for flight
void loop() {
  IMUsignals();//function to acquire data
  KALMANfilter();
  PID();
  commandServos();
  BAROsignals();
  paraCheck(); //checks if parachute should be deployed and adapts external signals accordingly
  //writeSD();
  
  while (micros() - LoopTimer < LoopMAXTime); //wait until Max loop time
  LoopTimer=micros(); //timer that measures the time of each control loop >>> END
  
}
