
void TTLcmd(void) {

  int loopClosed = 0;
  static byte power= 10; //power in power %

  do{
    if (SerialUSB.available() > 0)
    {

      char din = SerialUSB.read();

      switch (din) {

        case '1': power = 10; break; 
        case '2': power = 20; break; 
        case '3': power = 30; break; 
        case '4': power = 40; break; 
        case '5': power = 50; break; 
        case '6': power = 60; break; 
        case '7': power = 70; break;
        case '8': power = 80; break;  
        case '9': power = 90; break; 
        case '0': power = 100; break; 

        case 'w':
          m_set_direct(5, ((power*255)/100));
          SerialUSB.print("\nAll motors pulsed! Press z to terminate. ");
          SerialUSB.println(((power*255)/100));
          loopClosed = 1;
          break;

        case 's':
          m_set_direct(5, 0);
          SerialUSB.println("\nAll motors stopped! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'e':
          m_set_direct(mR, ((power*255)/100));
          SerialUSB.println("\nTest Right motor fwd! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'd':
          m_set_direct(mR, -((power*255)/100));
          SerialUSB.println("\nTest Right motor rev! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'q':
          m_set_direct(mL, ((power*255)/100));
          SerialUSB.println("\nTest left motor fwd! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'a':
          m_set_direct(mL, -((power*255)/100));
          SerialUSB.println("\nTest left motor rev! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'r':
          m_set_direct(mV, ((power*255)/100));
          SerialUSB.println("\nTest Vertical motor Push UP! Press z to terminate.\n");
          loopClosed = 1;
          break;

        case 'f':
          m_set_direct(mV, -((power*255)/100));
          SerialUSB.println("\nTest left motor Push Down! Press z to terminate.\n");
          loopClosed = 1;
          break;
          
        case 'h':
          SerialUSB.println("\nCommands:");
          SerialUSB.println("Press from 1 to 9 to set power from 10% to 90%. Press 0 for 100% power. Default is 20%");
          SerialUSB.println("w: Pulse all motors");
          SerialUSB.println("s: Stop all motors");
          SerialUSB.println("e: Test Right motor fwd! Press z to terminate.");
          SerialUSB.println("d: Test Right motor rev! Press z to terminate.");
          SerialUSB.println("q: Test Left motor fwd! Press z to terminate.");
          SerialUSB.println("a: Test Left motor rev! Press z to terminate.");
          SerialUSB.println("r: Test Vertical motor push up! Press z to terminate.");
          SerialUSB.println("f: Test Vertical motor push down! Press z to terminate.\n");
          loopClosed = 1;
          break;
        case 'z':
          loopClosed = 0;
          break;
        default:
          SerialUSB.println("\nInvalid command, for help press h \n");
          break;

      }
    }
    delay(1);
    }while (loopClosed == 1);
}

