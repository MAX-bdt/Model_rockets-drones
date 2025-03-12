//GLOBAL CODE FOR LOOP_V1 FLIGHT_1 SOFTWARE
//RUNS ON RESILIENCE V2 FLIGHT COMPUTER MICROCONTROLLER (Arduino NANO EVERY)
//made by Maximilien BOUDET - OCTOBER 2024

/*------------WIRING-------------------
I2C devices :
  pin IMU MPU6050 :
  (datasheet : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
  (register map : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
  SDA-> A4 Arduino NANO
  SCL-> A5 Arduino NANO
  VCC-> 5V Arduino NANO
  GND-> GND Arduino NANO

  pin BAROMETER BMP180 :
  (datasheet : https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)
  SDA-> A4 Arduino NANO
  SCL-> A5 Arduino NANO
  VCC-> 5V Arduino NANO
  GND-> GND Arduino NANO
      -----------
SERVOS :
(datasheet : https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf)
  pin SERVO X & Y :
  pin S servo X-axis    -> D7 Arduino NANO
  pin S servo Z-axis    -> D8 Arduino NANO
  
  pin SERVO HATCH :
  pin S servo HATCH     -> D2 Arduino NANO
      -----------
SPI devices :
  pin MICRO-SD module :
  (datasheet : https://components101.com/sites/default/files/component_datasheet/Micro-SD-Card-Module-Datasheet.pdf)
  (or : https://components101.com/modules/micro-sd-card-module-pinout-features-datasheet-alternatives)
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
#include <Wire.h>//(datasheet : https://www.arduino.cc/reference/en/language/functions/communication/wire/)
#include <Servo.h>//(datasheet : https://www.arduino.cc/reference/en/libraries/servo/)
#include <SD.h>//(datasheet : https://www.arduino.cc/reference/en/libraries/sd/)
//MORE INFO on SD.h : https://forum.arduino.cc/t/why-is-the-sd-library-slow/49791/8

//-----------------------------------------Def global variables----------------------------------------------
//-----FLIGHT PATH VARIABLES-----------
#define DesiredAngleRoll 0
#define DesiredAnglePitch 0 
#define paraAltitude 20 //max altitude for parachute deployment (in meters)
#define paraDelay 2100 //parachute deployment delay in seconds
//-----CLOCKS--------------------------
uint32_t LoopTimer=0; //parameter that contains the length of each control loop (period)
uint32_t liftOffTime=0;
uint32_t logtime=0;
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
float chargeBatt=0;
uint8_t MinChargeBatt=65; //minimal charge of battery for flight
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
#define rad_deg (1/(3.142/180)) //radian to degrees conversion factor
//variable for checklist test IMU
float moyRateRoll=0;
float moyRateYaw=0;
float moyRatePitch=0;
float moyAccX=0;
float moyAccY=0;
float moyAccZ=0;
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
uint8_t baroToken=1;
uint8_t baroCounter=0;
uint32_t pressure;
float altitudeBarometer;
float initialAltitude=0;
float realAltitudeBarometer;
//power of 2 necessary in raw to hPa pressure conversion
#define p2_4 (pow(2,4))
#define p2_8 (pow(2,8))
#define p2_11 (pow(2,11))
#define p2_12 (pow(2,12))
#define p2_13 (pow(2,13))
#define p2_15 (pow(2,15))
#define p2_16 (pow(2,16))
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
uint16_t flushcount = 0;
uint16_t flushloopsNb = 100;
//KALMAN INITIAL VALUES 
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2; //initial angle roll /initial uncertainty in angle (set as a pointer*) 
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2; //initial angle pitch /initial uncertainty in angle
float Kalman1DOutput[]={0,0}; //initial output for the Kalman filter: contains prediction angle/uncertainty on the angle)
#define Ts 0.02 // time frame in the Kalman function 
               //(ratio between resposivity and filtering noise: +Ts >>> +responsivity = +noise (on the initial code Ts was set to 0.004)
//KALMAN UNCERTAINTIES
float AccStdDev=3; //accelero standard deviation = 3 deg
float GyroStdDev=4; //gyro standard deviation rate = 4 deg/s
//INITILISATION TEST
#define initialNUM 1000 //number of values measured for initialisation of IMU & BARO
//---------------------PID VARIABLES---------------------------
float ErrorRoll, ErrorPitch;
float InputRoll, InputPitch;
float RawCommandAngleRoll, RawCommandAnglePitch;
int16_t CommandAngleRoll, CommandAnglePitch;
float PrevErrorRateRoll, PrevErrorRatePitch;
float PrevItermRateRoll, PrevItermRatePitch;
float PrevDtermRateRoll, PrevDtermRatePitch;
float PIDReturn[]={0, 0, 0, 0};

#define Tpid 0.01 //sets the delay in seconds for calculation of discret integrals in PID (here 0.01s>>>100Hz)
#define Tpid_2 (Tpid/2) //macro def to get some calculation quicker in main loop
#define MAXtvc 25  //defines MAX (or MIN = -MAXtvc) for TVC servos range of mvt (>>> can be seen as saturation limit for a DC motor)
#define MAXtvc_2 (MAXtvc-2) //macro def to get some calculation quicker in main loop
#define Wcut (2*3.142*10) // cutoff amplitude (or frequency if divided by 2*pi) of the lowpass filter in derivative calculation in PID function
#define one_Wcut (1/Wcut) //macro def to get some calculation quicker in main loop
//------------PID GAINS--- define them when PID tuning --------------
#define PRateRoll 0.8
#define IRateRoll 0.02  
#define DRateRoll 0.1 
#define PRatePitch 0.8
#define IRatePitch 0.02
#define DRatePitch 0.1
//RQ : Tuned with pre-Flight 1 simulation study

//ADAPT THE LOOP FREQUENCY HERE :
#define LoopMAXTime 10000 //loop time in µs (microseconds)(here 10000µs = 0.01s -> 100Hz loop)
//CAUTION : LOOPMAXTIME (in µs) MUST BE EQUAL TO Tpid (in s) !!!


//-----------------------------------------Call functions------------------------------------------------------
//file location mentionned on the right
//-----------------------------------------SECONDARY functions-------------------------------------------------
void Pulse_PIN_A6(uint16_t del);//TESTsoftware
void Pulse_PIN_A7(uint16_t del);//TESTsoftware
void EXITstatus();//EMERGENCYprog
int16_t getReadings(uint8_t sensor, uint8_t adress, uint8_t size_data);//UTILS       
void writeValueInside(uint8_t sensorAdress ,uint8_t adressValue, uint8_t value);//UTILS
void buzzerSound();//SIGNALS
void blinkLED(uint8_t LEDpin, uint16_t delayblink, uint8_t iteration);//SIGNALS
void buzzerBlink(uint16_t freq, uint16_t delaybuzz, uint8_t iteration);//SIGNALS
void engineFullRangeTest();//SERVO
void jackConnected();//JACK
void LIPOpowerOK();//UTILS
void IMUdataOK();//IMU
void BAROdataOK();//BARO
void SDwriteOK();//SD
//void verifyChecklist(uint8_t numTests);//UTILS
int16_t roundnumber(float numfloat, int16_t numint);//UTILS
//-----------------------------------------PRIMARY functions-----------------------------------------------
void initAll();//SETUPfunctions
void rebootFlag();//EMERGENCYprog
void startup();//SETUPfunctions              
void checklist();//SETUPfunctions            
void initialErrorIMU();//IMU      
void initialErrorBARO();//BARO
void SDinitialStates();//SD
void SDwriteLegend();//SD
void waitingState();//SETUPfunctions
void IMUsignals();//IMU
void KALMANfilter();//IMU
void PID();//IMU
void commandServos();//SERVO
void BAROsignals();//BARO
void writeSD();//SD
void paraCheck();//UTILS
void emergencyProcedure();//EMERGENCYprog
//----------------------------------------------------------------------------------------------------------
//Setup before flight
void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);//Fast mode (https://www.arduino.cc/reference/en/language/functions/communication/wire/setclock/)
  Wire.begin();
  rebootFlag();
  if (setupSkip==true){//skips setup in case of system reboot to get to the main loop again (EMERGENCY PROCEDURE)
    emergencyProcedure();
  }
  initAll();              //Initialises every input/output
  delay(10);
  startup();              //First actions at startup
  checklist();            //Checklist function to verify each component
  initialErrorIMU();      //Initialise IMU measurements
  initialErrorBARO();     //Initialise BARO measurements
  delay(100);
  SDinitialStates();      //Write initial measures into the SD card file
  SDwriteLegend();        //Write legend row into the SD card file
  delay(100);
  digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, HIGH);
  delay(5000); 
  waitingState();         //Rocket ready >>> waiting for lift-off (EXITS AT LIFT OFF)
  liftOffTime = millis();
  LoopTimer = micros();//timer that measures the time of each control loop >>> START
}
//---------------------------------------------------------------------------------------------------------------
//infinite Loop for flight
void loop() {

  IMUsignals();//function to acquire data from IMU
  KALMANfilter();//get accurate attitude
  PID();//calculates accurate response to attitude error
  commandServos();//send order to servos
  //measure initial altitude
  if (baroToken){
    baroToken=0;
    BAROsignals();//measure altitude of the rocket
  }
  //measure altitude every 4 loops (to reduce loop time)
  baroCounter=baroCounter+1;
  if (baroCounter >= 4){
    baroCounter=0;
    BAROsignals();//measure altitude of the rocket
  }
  paraCheck();//check if parachute should be deployed and adapts external signals accordingly
  writeSD();//log data into SD
  
  while (micros() - LoopTimer < LoopMAXTime); //wait until Max loop time
  LoopTimer=micros(); //timer that measures the time of each control loop >>> END
}
