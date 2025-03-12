/*
Functions related to IMU (Inertial Measurement Unit)
*/

//function that verifies if IMU Data is correct
void IMUdataOK(){
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

//fuction that initialise IMU measurements to get rid of vertical position error
void initialErrorIMU() {
    for (int i = 0; i < initialNUM; i++) {
        IMUsignals();
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

        AngleErrorRoll = AngleErrorRoll + KalmanAngleRoll;
        AngleErrorPitch = AngleErrorPitch + KalmanAnglePitch;
        //sum all the readings
    }
    AngleErrorRoll = AngleErrorRoll / initialNUM;
    AngleErrorPitch = AngleErrorPitch / initialNUM;
    blinkLED(LED_PIN_G, 200, 5);
}

//KALMAN filter function
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) { //function inputs
  KalmanState=KalmanState+ Ts*KalmanInput; //predicts current state of the system
  KalmanUncertainty=KalmanUncertainty + Ts * Ts * 16; //calculate uncertainty of the prediction
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 9); //calculate Kalman Gain from uncertainties on the prediction and measurements
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
  float Iterm=PrevIterm+I*(Error+PrevError)*Tpid_2;  // Integral corrector >>> Tpid is the measurement sample time called "Ts" in def PID sheet (initially set to 0.004) >>> modify this term accordingly
  if (Iterm > MAXtvc_2){
    Iterm=MAXtvc_2;// Define range for Integral corrector, it is the same range as the one for servos (MAXtvc-1=9deg)
  }//here MAXtvc_2 = MAXtvc-2 ; see macro def in main file
  else if (Iterm <-MAXtvc_2){
    Iterm=MAXtvc_2;// >>> ANTI-WINDUP : if Iterm is above max >>> stop correction with Iterm
  }
  float Dterm=((D*(Error-PrevError)/Tpid)+((one_Wcut)*PrevDterm)/Tpid)*(Tpid/(Tpid+(one_Wcut))); // Derivative corrector + low pass filter
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
  AngleRoll=atan(AccZ/sqrt(AccY*AccY+AccX*AccX))*rad_deg;
  AnglePitch=atan(-AccX/sqrt(AccY*AccY+AccZ*AccZ))*rad_deg;
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
