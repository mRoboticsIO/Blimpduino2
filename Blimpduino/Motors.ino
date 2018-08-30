void initilize_motors(void) {

  // Blimpduino pins
  // Motor0
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  // Motor1
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  // Motor2
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  // Motor3
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  //Beeping motors:
  for (int q = 0; q <= 2; q++)
  {
    for (int m = 0; m <= 3; m++)
    {
      m_set_direct(m, 10);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      delay(30);
      m_set_direct(m, 0);
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      delay(30);
    }
  }
}


void m_stopAll(void) { //Stopping all motors.
  m_set_direct(0, 0);
  m_set_direct(1, 0);
  m_set_direct(2, 0);
  m_set_direct(3, 0);
}

//DON NOT CHANGE!!! Values are matched to BLimpduino hardware.
void m_set_direct(int motor, int value) { //The same as m_set below but with DeadZone control.

  switch (motor) {
    case 0: //Motor 0
#if m0_rev == 0 //if m0_rev is set to 1 it will reverse the motors 
      output_motor_direct(value, 5, 6);
#else
      output_motor_direct(value, 6, 5);
#endif
      break;
    case 1:
#if m1_rev == 0
      output_motor_direct(value, 9, 10); //Motor 1
#else
      output_motor_direct(value, 10, 9); //Reversed
#endif
      break;
    case 2:
#if m2_rev == 0
      output_motor_direct(value, 7, 8); //Motor 2
#else
      output_motor_direct(value, 8, 7); //Reversed
#endif
      break;
    case 3:
#if m3_rev == 0
      output_motor_direct(value, 11, 12); //Motor 3
#else
      output_motor_direct(value, 12, 11); //Reversed
#endif
      break;
    case 5: //Will set all motor at the same time. WARNING Reverse not working yet. 
      output_motor_direct(value, 5, 6);
      output_motor_direct(value, 9, 10);
      output_motor_direct(value, 7, 8);
      output_motor_direct(value, 11, 12);
      break;
    default:

      break;
  }

}

void m_set(int motor, int value, int deadZone) { //Adds "deadZones" to m_set_direct.

  if (abs(value) > deadZone) {
    m_set_direct(motor, value);
  }
  else
  {
    m_set_direct(motor, 0);
  }
}

void output_motor_direct(int value, uint8_t pin1, uint8_t pin2) //Direct motor control. Use with caution. 
{
  uint8_t val;
  char srt_out[60];

  if (value != 0) {

    if (value > 0) {
      if (value > 125)
        val = 255;
      else
        val = (value * 2);
      //sprintf(srt_out, "Spd:%03d Forward Pin1:%02d Pin2:%02d", int(val), int(pin1), int(pin2));
      //SerialUSB.println(srt_out);

      analogWrite(pin1, 0);//digitalWrite(pin1, LOW); <---When using DigitalWrite it creates a bug in certain conditions.
      analogWrite(pin2, val); //"Fordward"
    }
    else {
      if (value < -125)
        val = 255;
      else
        val = (-value * 2);

      //sprintf(srt_out, "Spd:%03d Backward Pin1:%02d Pin2:%02d", int(val), int(pin1), int(pin2));
      //SerialUSB.println(srt_out);

      analogWrite(pin2, 0); //digitalWrite(pin2, LOW); <---When using DigitalWrite it creates a bug in certain conditions.
      analogWrite(pin1, val); //Reverse
    }
  }
  else //Stop everything if is zero
  {
    //sprintf(srt_out, "Spd:%03d Stop! Pin1:%02d Pin2:%02d", int(val), int(pin1), int(pin2));
    //SerialUSB.println(srt_out);

    //digitalWrite(pin1, LOW);
    //digitalWrite(pin2, LOW);
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}
