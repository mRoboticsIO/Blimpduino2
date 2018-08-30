
void Dameon_Loop(void) { //Background stuff.

  static unsigned long _timer = 0;

  //if ((millis() - _timer) > 20) {

  //_timer=millis();//Reseting timer.

  //static long timer_dameon = 0;
  /**************************************************/
  if (MPU6050_newData()) //Checking MPU
  {
    timer_mpu = micros(); //Reacording when the latest data was captured to use it later at mode 1.
    MPU6050_read_3axis();
    mpu_newDataReady = 1;
    MPU_dt = (timer_mpu - timer_mpu_old) * 0.000001;
    MPU_yaw_angle = MPU6050_yawAngle(MPU_dt);
    timer_mpu_old = timer_mpu; //Timer reset

  }
  /**************************************************/
  if (distanceSensor.newDataReady() == true) //Checking LIDAR
  {
    timer_laser = millis(); //Recording the time.
    laser_height = distanceSensor.getDistance();
    laser_newDataReady = 1;
    distanceSensor.startMeasurement();
    height = height * 0.7 + laser_height * 0.3; //Low pass filter.

    height_dt = (timer_laser - timer_laser_old) * 0.000001;
    timer_laser_old = timer_laser;
  }
  /**************************************************/
  if (newMessage == 1) //Checking new UDP data from APP.
  {
    newMessage = 0;
    throttle = iCH1 / 5;
    //steering = iCH2/5;

    // Non linearity on steering... (-100 to 100)
    steering = -iCH2 / 500.0;
    if (steering > 0)
      steering = (steering * steering + 0.5 * steering) * 66;
    else
      steering = (-steering * steering + 0.5 * steering) * 66;

    //value=((value*.9)+(newValue*.1))
    //mode = 1;
    //target_angle = MPU_yaw_angle;

    target_height += iCH3;

    target_height = constrain(target_height, -5000, 5000);

    timer_newData = millis(); //Safety timer, if not data received after a while, motors will shut off as a precatuion.
  }

  //}
} //End of Daemon Loop.


/**************************************************

**************************************************/

void USB_Print_Loop(int refreshRate) {

  if ((millis() - timer_termPrint) > refreshRate) {

    BatteryValue = analogRead(A0) / 16; // / BATTERY_FACTOR;
    if (BatteryValue < 37)
      digitalWrite(A2, HIGH); // Battery warning
    else
      digitalWrite(A2, LOW);
    char auxS[25];
    char aux2[100];
    //sprintf(auxS, "$tA,%+04d", height/10);

    sprintf(auxS, "B:%02d A:%+04d T:%+04d H:%04d", int(BatteryValue), int(MPU_yaw_angle), int(target_angle), int(laser_height));
    Serial1.println(auxS);
    sprintf(aux2, "Batt:%02d Yaw:%+04d SetP:%+04d LIDAR:%04d M0:%+04d M1:%+04d M2:%+04d CH5:%+06f", int(BatteryValue), int(MPU_yaw_angle), int(target_angle), int(height), int(mRight_Value), int(mLeft_Value), int(mVertical_Value), height_dt);
    SerialUSB.write(aux2);
    SerialUSB.println(" ");

    timer_termPrint = millis(); //Reseting timer
  }

}

/**************************************************

**************************************************/

void manualControl(void) {

  if (steering >= 0)
  {
    mRight_Value = throttle - steering;
    mLeft_Value = throttle + steering - (0.5 * (float)steering); //By Jordi
  }
  else
  {
    mRight_Value = throttle - steering - (0.5 * (float)steering); //By Jordi
    mLeft_Value = throttle + steering;
  }

}

/**************************************************

**************************************************/

//Mode one assisted vectoring -left/right- control...

void yawStabilized(void) {
  if (mpu_newDataReady == 1) {
    mpu_newDataReady = 0;

    float kp = 1.5; //2.0;
    float kd = 2.5; //2.0 //2.5;
    float ki = 1.0;
    error_old = error;
    error = (target_angle - MPU_yaw_angle);
    control = kp * error + (kd * (error - error_old) / MPU_dt); //Proportional, Derivative control only.
    control = constrain(control, -100, 100); //default -127 127

    mRight_Value = (throttle / 2) - control;
    mLeft_Value = (throttle / 2) + control;

    target_angle = target_angle - steering / 50.0;

    // Yaw angle normalization
    if ((MPU_yaw_angle > 360) && (target_angle > 360)) {
      MPU6050_setYawAngle(MPU_yaw_angle - 360);
      target_angle -= 360;
    }
    if ((MPU_yaw_angle < -360) && (target_angle < -360)) {
      MPU6050_setYawAngle(MPU_yaw_angle + 360);
      target_angle += 360;
    }
  }
}

void altitudeHold(void) {

  if (laser_newDataReady == 1) {
    laser_newDataReady = 0;

    error_h_old = error_h;
    error_h = (target_height - height);
    kp_h = 0.3; //0.22; //0.15 //0.075; //0.2 //22
    kd_h = .001; //0.35 //0.3 //0.15; //0.10;  //0.15 <<<<<<----- I had to reduce it dramatically for this new version.
    control_h = 0 + (kp_h * error_h) + (kd_h * (error_h - error_h_old) / height_dt);
    //if (control_h > 0) {
    //  control_h = (control_h * 4);
    //}
    //SerialUSB.println((kd_h * (error_h - error_h_old) / height_dt));
    control_h = constrain(control_h, -100, 127);
    mVertical_Value = control_h;
  }
}

void altitudeManual(int p) {

  mVertical_Value = iCH3 * p;
}

