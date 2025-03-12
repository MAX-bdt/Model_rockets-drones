//GLOBAL CODE FOR LOOP_V2 FLIGHT_4 SOFTWARE
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
servo Z-axis    -> 75deg - 100deg - 125deg (100deg +- 25deg)
servo HATCH     -> 60deg = closed / 90deg = opened (60deg + 30deg)
--------------------------------------------*/

/*------------SENSORS CONFIG-------------------
IMU MPU6050 sensors :
Gyro -> +/- 500deg/s
Accel -> +/- 8g
X-axis -> Roll
Z-axis -> Pitch
Control loop frequency -> f=80Hz (T=1/f=0.0125s)
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
#define paraAltitude 18 //max altitude for parachute deployment (in meters)
#define paraDelay 1600 //parachute deployment delay in seconds
//ADAPT THE LOOP FREQUENCY HERE :
#define LoopMAXTime 13000 //loop time in µs (microseconds)(here 13000µs = 13ms = 0.013s -> 76Hz loop)
//CAUTION : LOOPMAXTIME (in µs) MUST BE EQUAL TO Tpid & Ts (in s) !!!


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
#define AngleBaseZ 100
#define pinServoHatch 2//D2 pin
#define AngleBaseHatch 60
#define AngleOpenHatch 50 //angle to add to open the hatch
//EXTERNAL SIGNALS VARIABLES(LED/Buzzer)
#define LED_PIN_G A2 //A2 pin
#define LED_PIN_Y A1 //A1 pin
#define LED_PIN_R A0 //A0 pin
#define buzzer 9//D9 pin
#define waitInterval 1000 //blink delay waiting state
#define flightInterval 150 //blink delay Flight
bool blinkState = LOW; //blinkState used to set the LED & buzzer in waiting loop
bool blinkStateFlight = LOW; //blinkState used to set the LED & buzzer in flight loop
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
int16_t RateCalibrationNumber=0;
float AccX=0, AccY=0, AccZ=0;
float AccNorm=0;
float AngleErrorRoll=0;
float AngleErrorPitch=0;
float AngleRoll=0;
float AnglePitch=0;
float CorrectAngleRoll=0;
float CorrectAnglePitch=0;
float CorrectAngleRollPos=0;
float CorrectAnglePitchPos=0;
#define rad_deg (1/(3.142/180)) //radian to degrees conversion factor
uint32_t straightToken=0;
uint8_t straightLoops=8; //during the first 8 loops (~100ms) the engine gimbal will stay in (0,0) position to clear the launch tower
//variable for checklist test IMU && init
//GYRO
float moyRateRoll=0;
float moyRateYaw=0;
float moyRatePitch=0;
float moyGyroRollError=0;
float moyGyroPitchError=0;
float moyGyroYawError=0;
//ACCELERO
float moyAccX=0;
float moyAccY=0;
float moyAccZ=0;
float moyAccXerror=0;//distance from 0g value at vertical rest
float moyAccYerror=0;//distance from 1g value at vertical rest
float moyAccZerror=0;//distance from 0g value at vertical rest
//BARO variables
#define BMP180adress 0x77 //BARO sensor I2C ADRESS
short ac1=0;
short ac2=0;
short ac3=0;
unsigned short ac4=0;
unsigned short ac5=0;
unsigned short ac6=0;
short b1=0;
short b2=0;
short mb=0;
short mc=0;
short md=0;
//define variables for temperature and pressure readings
long ut=0;
long up=0;
long x1=0;
long x2=0;
long b5=0;
long T=0;
long b6=0;
long x3=0;
long b3=0;
unsigned long b4=0;
unsigned long b7=0;
long p=0;
float temperature=0;
long pressure=0;
int16_t altitudeBarometer=0;
float initialAltitude=0;
int16_t realAltitudeBarometer=0;
uint8_t baroCounter=0;
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
OSS=0 ultra Low Power Setting,  1 sample, 4.5 ms 3uA<<<<<<<<<<< selected
OSS=1 Standard Power Setting,   2 samples, 7.5 ms 5uA
OSS=2 High Resolution,          4 samples, 13.5 ms 7uA
OSS=3 Ultra High Resolution,    2 samples, 25.5 ms 12uA
*/
//SD VARIABLES
File dataFile;
File flagFile;
#define pinSD 10 //D10 >>> CS = Chip Select for SPI bus used by micro SD module
uint16_t flushcount=0;
uint16_t flushloopsNb=80;
uint8_t paraToken=1;
//KALMAN INITIAL VALUES 
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2; //initial angle roll /initial uncertainty in angle Roll 
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2; //initial angle pitch /initial uncertainty in angle Pitch
//float Kalman1DOutput[]={0,0}; //initial output for the Kalman filter: contains prediction angle/uncertainty on the angle)
//KALMAN UNCERTAINTIES
//Tuned  based on MPU6050 Datasheet and tests to see Kalman filter OUTPUT
#define GyroStdDev 0.04 //gyro standard deviation rate = 0.04 deg/s
#define AccStdDev 0.007 //accelero standard deviation = 0.007 deg
#define gyroErrorEstimate2 (GyroStdDev*GyroStdDev) //gyro standard deviation rate = 4 deg/s Squared
#define accErrorEstimate2 (AccStdDev*AccStdDev) //Accelero standard deviation rate = 3 deg Squared
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

//CAUTION : Ts=Tpid=Tloop !!!
#define Ts 0.013 //sets the sample time in seconds for calculation of discret integrals in Kalman filter (here 0.013s=>76Hz)
//(ratio between resposivity and filtering noise: +Ts >>> +responsivity = +noise (on the initial code Ts was set to 0.004)
#define Tpid 0.013 //sets the sample time in seconds for calculation of discret integrals in PID (here 0.013s=>76Hz)
#define Tpid_2 (Tpid/2) //macro def to get some calculation quicker in main loop
//TVAS GIMBAL BOUNDS :
uint8_t MAXgimbalX=15;
uint8_t MINgimbalX=15;
uint8_t MAXgimbalZ=15;
uint8_t MINgimbalZ=20;
#define Wcut (2*3.142*10) // cutoff amplitude (or frequency if divided by 2*pi) of the lowpass filter in derivative calculation in PID function
#define one_Wcut (1/Wcut) //macro def to get some calculation quicker in main loop
//------------PID GAINS--- define them when PID tuning --------------
#define PRateRoll 1.25
#define IRateRoll 0.01  
#define DRateRoll 0.3 
#define PRatePitch 1.25
#define IRatePitch 0.01
#define DRatePitch 0.3
//RQ : Tuned with pre-Flight 2 SIMULINK simulation study

//-----------------------------------------Call functions------------------------------------------------------
//file location mentionned on the right
//-----------------------------------------SECONDARY functions-------------------------------------------------
//void Pulse_PIN_A6(uint16_t del);//TESTsoftware
//void Pulse_PIN_A7(uint16_t del);//TESTsoftware
void EXITstatus();//EMERGENCYprog
void callSensor4readings (uint8_t sensor, uint8_t adress);//UTILS
void writeValueInside(uint8_t sensorAdress ,uint8_t adressValue, int16_t value);//UTILS      
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
void Gyro_Accelero_IMUinit();//IMU            
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
void determineAltitude();//BARO
void BAROsignals_0();//BARO
void BAROsignals_1();//BARO
void BAROsignals_2();//BARO
void BAROsignals_3();//BARO
void BAROsplitMeas();//BARO
void writeSD();//SD
void paraCheck();//UTILS
void emergencyProcedure();//EMERGENCYprog
//----------------------------------------------------------------------------------------------------------
//Setup before flight
void setup() {
  //Serial.begin(115200); //only needed for testing
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
  delay(100);
  Gyro_Accelero_IMUinit();//Initialise IMU sensors measurements (ROTATION RATES & ACCELERATION)
  initialErrorIMU();      //Initialise IMU measurements (ATTITUDE)
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
  LoopTimer = micros();   //timer that measures the time of each control loop >>> START
}
//---------------------------------------------------------------------------------------------------------------
//infinite Loop for flight
void loop() {
  IMUsignals();                               //function to acquire data from IMU
  KALMANfilter();                             //get accurate attitude
  PID();                                      //calculates accurate response to attitude error
  commandServos();                            //send order to servos
  BAROsplitMeas();                            //altitude measure split in 3 loops (to reduce overall loop time)
  paraCheck();                                //check if parachute should be deployed and adapts external signals accordingly
  writeSD();                                  //log data into SD
  while (micros() - LoopTimer < LoopMAXTime); //wait until Max loop time
  LoopTimer=micros();                         //timer that measures the time of each control loop >>> END of the loop
}

/*ADD IN THE LOOP TO MAKE TVAS DATA VERIFICATION (on serial plotter)
  Serial.print(CorrectAngleRollPos);
  Serial.print(",");
  Serial.print(CorrectAnglePitchPos);
  Serial.print(",");
  Serial.print(InputRoll);
  Serial.print(",");
  Serial.println(InputPitch);
*/
