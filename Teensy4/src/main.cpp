#include <Arduino.h>
#include "SensorFunctions.h"

/* --------------------------------------------------------------------------------------
  Main Controller Code to run on Teensy 4.0
  Author: Daniel Hoven Date: 3/15/2021 Project: Senior Capstone
  --------------------------------------------------------------------------------------*/
/*= == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = == == == == == == == =
SETUP 
*/

void setup()

{
  /* Open Serial Ports*/

  SERIAL_USB.begin(250000);
  LIDAR_SERIAL.begin(115200);
  RADIO_SERIAL.begin(RADIO_BAUDRATE);
  delay(100);

  /* put liDAR in std. output mode */

  set_liDAR();

  while (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SERIAL_USB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(200);
  }

  SERIAL_USB.print("IMU found\n");
  bno.setExtCrystalUse(true);

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      K[i][j] *= LQRmult;
    }
  }

  calibrateESCs();
  delay(500);
}

/*-------------------------------------------------------------------------------
   main
  ------------------------------------------------------------------------------*/
void loop(void)
{

  global.dt = micros() - global.tMicros;
  global.tMicros = micros();
  global.dt = global.dt / 1000000;

  get_IMU_sample(global.dt,global.iterations),
  get_Distance_sample(global.dt),
  ELQR_calc();

  if (global.VERT_SPEED == true)
  {
    vertSpeedHold();
  }
  receiveData();

  if (global.iterations % 5 == 0)
  {
    printData();
    //printTestData();
  }

  // Make sure Vehicle isn't dying
void CheckAttitudeLimits(); // Oh Sh*t method

  if ((global.TAKEOFF_FLAG) && (millis() > 2000))
  {
    IntegralTracker(global.dt);
    AltitudePID();
  }

  if (global.iterations < 1000)
  {
    for (int i = 1; i < 4; i++)
    {
      signal.Ucal[i] = signal.U[i];
    }
  }
  else
  {
    if (!global.STOP_FLAG)
    {
      commandESCs();
    }
    else
    {
      STOP();
    }
  }
  global.iterations++;
  delay(MAIN_DELAY);
}
