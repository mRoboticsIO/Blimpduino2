// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Network functions (ESP module)

int ESPwait(String stopstr, int timeout_secs)
{
  String response;
  bool found = false;
  char c;
  long timer_init;
  long timer;

  timer_init = millis();
  while (!found) {
    timer = millis();
    if (((timer - timer_init) / 1000) > timeout_secs) { // Timeout?
      SerialUSB.println("!Timeout!");
      return 0;  // timeout
    }
    if (Serial1.available()) {
      c = Serial1.read();
      SerialUSB.print(c);
      response += c;
      if (response.endsWith(stopstr)) {
        found = true;
        delay(10);
        Serial1.flush();
        SerialUSB.println();
      }
    } // end Serial1_available()
  } // end while (!found)
  return 1;
}

// getMacAddress from ESP wifi module
int ESPgetMac()
{
  char c1, c2;
  bool timeout = false;
  long timer_init;
  long timer;
  uint8_t state = 0;
  uint8_t index = 0;

  MAC = "";
  timer_init = millis();
  while (!timeout) {
    timer = millis();
    if (((timer - timer_init) / 1000) > 5) // Timeout?
      timeout = true;
    if (Serial1.available()) {
      c2 = c1;
      c1 = Serial1.read();
      SerialUSB.print(c1);
      switch (state) {
        case 0:
          if (c1 == ':')
            state = 1;
          break;
        case 1:
          if (c1 == '\r') {
            MAC.toUpperCase();
            state = 2;
          }
          else {
            if ((c1 != '"') && (c1 != ':'))
              MAC += c1;  // Uppercase
          }
          break;
        case 2:
          if ((c2 == 'O') && (c1 == 'K')) {
            SerialUSB.println();
            Serial1.flush();
            return 1;  // Ok
          }
          break;
      } // end switch
    } // Serial_available
  } // while (!timeout)
  SerialUSB.println("!Timeout!");
  Serial1.flush();
  return -1;  // timeout
}

int ESPsendCommand(char *command, String stopstr, int timeout_secs)
{
  Serial1.println(command);
  ESPwait(stopstr, timeout_secs);
  delay(250);
}

int32_t ExtractParamInt4b(uint8_t pos) {
  union {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

int16_t ExtractParamInt2b(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

// Messgase: 8 channels (16 bits)
void MsgRead()
{

  uint8_t i;
  // New bytes available to process?
  while (Serial1.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    int rec = Serial1.read();


    //SerialUSB.print(rec);
    MsgBuffer[MSGMAXLEN - 1] = (uint8_t)rec;

    //SerialUSB.print((char)rec);
    // Message JJAM: Manual control mode
    if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'B') && (char(MsgBuffer[3]) == 'M')) {
      //SerialUSB.println("->MSG: JJBM:");
      iCH1 = ExtractParamInt2b(4);
      iCH2 = ExtractParamInt2b(6);
      iCH3 = ExtractParamInt2b(8);
      iCH4 = ExtractParamInt2b(10);
      iCH5 = ExtractParamInt2b(12);
      iCH6 = ExtractParamInt2b(14);
      iCH7 = ExtractParamInt2b(16);
      iCH8 = ExtractParamInt2b(18);
      //mode = 0;
      newMessage = 1;
    }
    // Message JJAI: Automatic control mode (Inverse Kinematic)
    if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'B') && (char(MsgBuffer[3]) == 'A')) {
      //SerialUSB.println("->MSG: JJBA:");
      iCH1 = ExtractParamInt2b(4);
      iCH2 = ExtractParamInt2b(6);
      iCH3 = ExtractParamInt2b(8);
      iCH4 = ExtractParamInt2b(10);
      iCH5 = ExtractParamInt2b(12);
      iCH6 = ExtractParamInt2b(14);
      iCH7 = ExtractParamInt2b(16);
      iCH8 = ExtractParamInt2b(18);
      //mode = 1;
      newMessage = 1;
    }
  }
  /************************************************************/
}


