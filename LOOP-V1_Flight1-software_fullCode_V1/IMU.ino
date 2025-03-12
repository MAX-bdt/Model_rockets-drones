/*


*/

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
