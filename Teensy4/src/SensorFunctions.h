#include "Config.h"
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>

/* ==========================================================================================
  Declare globals */

struct Signal
{
  float e1 = 0,
        e2 = 0,
        e3 = 0,
        e4 = 0,
        Ecal[4] = {0, 0, 0, 0};
  
  int throttle = 0;

  double U[4] = {IDLE_SPEED, 0, 0, 0},
         Ucal[4];

};

volatile float liDARold = 0,
               _lidar = 0;

volatile int liDARval = 0,
             strength = 0;

struct Euler
{
  double x = 0,
         y = 0,
         z = 0;

  double dx = 0,
         dy = 0,
         dz = 0;

  double Hist[3][5];
};

struct State
{
  double Full[6] = {0, 0, 0, 0, 0, 0},
         Integral[6],
         Old[6],
         Error[6];
};

struct Altitude
{
  double alt = 0,
         d_alt = 0,
         initAlt = 0;
};

struct Setpoint
{
  double R[6] = {0, 0, 0, 0, 0, 0};
  double Alt = 0;
  double Rcal[3] = {0, 0, 0};
  double verticalSpeed = 0;
};

struct Global
{
  long int iterations = 0;
  int tMicros = 0;
  double dt = 0;

  String STATE = "STARTUP";

  bool STOP_FLAG = false,
       TAKEOFF_FLAG = false,
       VERT_SPEED = true;
};

float Ncode;
u_int16_t counter = 0;

String STATE = "STARTUP";

char Dcode[3];

State X;           // Vehicle Full state struct
Euler euler;       // Euler angle struct
Setpoint setPoint; // setpoint struct
Altitude altitude; // altitude data
Signal signal;     // controller output data
Global global;

/* =========================================================================================
  Declare Sensor Connections on I2C bus
*/

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo esc1,
    esc2,
    esc3,
    esc4;

/*-------------------------------------------------------------------------
   Write external functions
  ---------------------------------------------------------------------------*/

void set_liDAR()
{
  LIDAR_SERIAL.write(0x42);
  LIDAR_SERIAL.write(0x57);
  LIDAR_SERIAL.write(0x02);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x01);
  LIDAR_SERIAL.write(0x06);
}

void calibrateESCs()
{

  esc1.attach(ESC1);
   esc2.attach(ESC2);
    esc3.attach(ESC3);
     esc4.attach(ESC4);
      delay(100);
     esc1.write(30);
    esc2.write(30);
   esc3.write(30);
  esc4.write(30);
  delay(1000);

  for (int i = 30; i > 20; i--)
  {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
  }

  for (int i = 20; i < 35; i++)
  {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    delay(150);
  }
  delay(2000);
  esc1.write(40);
  esc2.write(40);
  esc3.write(40);
  esc4.write(40);
  delay(2000);
}

FASTRUN void get_IMU_sample(double dt, double iterations)
{

  /* get quaternions */

  imu::Quaternion quat = bno.getQuat();

  volatile double q0 = quat.w();
  volatile double q1 = quat.x();
  volatile double q2 = quat.y();
  volatile double q3 = quat.z();

  //quaternion conversion

  euler.x = (-setPoint.Rcal[0]) - atan2(2.0 * (q3 * q2 + q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2));  //* (180/PI);
  euler.y = (-setPoint.Rcal[1]) + asin(2.0 * (q2 * q0 - q3 * q1));                                    // * (180/PI);
  euler.z = (-setPoint.Rcal[2]) - atan2(2.0 * (q3 * q0 + q1 * q2), -1.0 + 2.0 * (q0 * q0 + q1 * q1)); //* (180/PI);

  for (int i = 0; i < 6; i++)
  {
    X.Old[i] = X.Full[i];
  }

  X.Full[0] = euler.x;
  X.Full[1] = euler.y;
  X.Full[2] = euler.z;

  // Compute Derivatives using 5 point stencil
  double h = dt;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 1; j < 5; j++)
    {
      euler.Hist[i][j] = euler.Hist[i][j - 1];
    }
    euler.Hist[i][0] = X.Full[i];
    double temp;
    temp = (-1) * euler.Hist[i][0] + (8) * euler.Hist[i][1] + (-8) * euler.Hist[i][3] + (1) * euler.Hist[i][4];
    temp /= 12 * h;
    if (iterations > 5)
    {
      X.Full[i + 3] = DERIVATIVE_FILT * X.Old[i + 3];
      X.Full[i + 3] += (1 - DERIVATIVE_FILT) * temp;
    }
    else
    {
      X.Full[i + 3] = .1; // prime derivatives with nonzero values
    }
  }
}

void get_Distance_sample(double dt)
{

  if (LIDAR_SERIAL.available() >= 9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if ((0x59 == LIDAR_SERIAL.read()) && (0x59 == LIDAR_SERIAL.read())) // byte 1and byte 2
    {
      unsigned int t1 = LIDAR_SERIAL.read(); // byte 3 = Dist_L
      unsigned int t2 = LIDAR_SERIAL.read(); // byte 4 = Dist_H
      t2 <<= 8;
      t2 += t1;
      liDARval = t2;
      t1 = LIDAR_SERIAL.read(); // byte 5 = Strength_L
      t2 = LIDAR_SERIAL.read(); // byte 6 = Strength_H
      t2 <<= 8;
      t2 += t1;
      strength = t2;
      for (int i = 0; i < 3; i++)
        LIDAR_SERIAL.read(); // ignore remaining bytes
    }
  }
  _lidar = (1 - filtAlt) * liDARval + filtAlt * liDARold;
  liDARold = _lidar;
  double _alt = _lidar * cos(euler.x) * cos(euler.y);
  altitude.d_alt *= filtAlt;
  altitude.d_alt += (1 - filtAlt) * (_alt - altitude.alt) / dt;
  altitude.alt = _alt;
}

FASTRUN void DerivativeComp()
{
  for (int i = 0; i < 3; i++)
  {
    setPoint.R[i + 3] = D_COMP * (X.Full[i] - setPoint.R[i]);
  }
}

FASTRUN void ELQR_calc()
{

  for (int i = 1; i < 4; i++)
  {
    double iter = 0;
    for (int j = 0; j < 3; j++)
    {
      iter += K[i][j] * ((X.Full[j] - setPoint.R[j]) + LQR_E * (X.Integral[j]));
    }
    for (int j = 3; j < 6; j++)
    {
      iter += K[i][j] * ((X.Full[j] - setPoint.R[j]) + LQR_P * (X.Integral[j]));
    }
    if (abs(iter) < SLEW_LIMIT)
    {
      signal.U[i] = iter;
      signal.U[i] -= signal.Ucal[i];
    }
    else if (iter > 0)
    {
      signal.U[i] = SLEW_LIMIT;
    }
    else if (iter < 0)
    {
      signal.U[i] = -SLEW_LIMIT;
    }
  }
}

FASTRUN void IntegralTracker(double dt)
{

  for (int i = 0; i < 3; i++)
  {
    double iter = (dt / 2) * (X.Old[i] + X.Full[i]);
    if ((abs(iter) + abs(X.Integral[i])) < INTEGRATOR_CLAMP)
    {
      X.Integral[i] += iter;
      X.Integral[i + 3] = X.Full[i];
    }
    else if ((iter + X.Integral[i]) < -INTEGRATOR_CLAMP)
    {
      X.Integral[i] += 0.01;
      X.Integral[i + 3] = X.Full[i];
    }
    else if ((iter + X.Integral[i]) > INTEGRATOR_CLAMP)
    {
      X.Integral[i] -= 0.01;
      X.Integral[i + 3] = X.Full[i];
    }
  }
  for (int i = 0; i < 3; i++)
  {
    if (abs(X.Integral[i] - setPoint.R[i]) > INTEGRATOR_CLAMP)
    {
      if ((X.Integral[i] - setPoint.R[i]) > 0)
      {
        X.Integral[i] = INTEGRATOR_CLAMP + setPoint.R[i] - 0.01;
      }
      else if ((X.Integral[i] - setPoint.R[i]) < 0)
      {
        X.Integral[i] = -(INTEGRATOR_CLAMP + setPoint.R[i]) + 0.01;
      }
    }
  }
}

bool goingToCrash()
{
  return false;
}

bool upsideDown()
{
  bool check = false;
  for (int i = 0; i < 2; i++)
  {
    if (abs(X.Full[i]) > PI / 6)
    {
      check = true;
    }
  }
  return check;
}

void STOP()
{
  Serial.println("------ CRASH CONDITION DETECTED!! -------");
  signal.e1 = 0;
  signal.e2 = 0;
  signal.e3 = 0;
  signal.e4 = 0;
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
}
void CheckAttitudeLimits()
{
  if (upsideDown())
  {
    STOP();
    global.STOP_FLAG = true;
  }
  else if (goingToCrash())
  {
    STOP();
  }
  else
  {
  }
}

void vertSpeedHold()
{
  double ERR = setPoint.verticalSpeed - altitude.d_alt;
  signal.e1 += ERR * V_SPD;
  signal.e2 += ERR * V_SPD;
  signal.e3 += ERR * V_SPD;
  signal.e4 += ERR * V_SPD;
}

void AltitudePID()
{
  double AltitudeError = setPoint.Alt - altitude.alt;
  setPoint.verticalSpeed = kp * AltitudeError + kd * altitude.d_alt;
}

void commandESCs()
{
  // Motor Mixing Algorithm

  float _e1 = signal.U[0] - signal.U[1] + signal.U[2] + signal.U[3];
   float _e2 = signal.U[0] - signal.U[1] - signal.U[2] - signal.U[3];
    float _e3 = signal.U[0] + signal.U[1] - signal.U[2] + signal.U[3];
     float _e4 = signal.U[0] + signal.U[1] + signal.U[2] - signal.U[3];

     _e1 = map(_e1, 0, 180, 1000, 2000);
    _e2 = map(_e2, 0, 180, 1000, 2000);
   _e3 = map(_e3, 0, 180, 1000, 2000);
  _e4 = map(_e4, 0, 180, 1000, 2000);

  signal.e1 *= (SLEW_FILTER);
   signal.e2 *= (SLEW_FILTER);
    signal.e3 *= (SLEW_FILTER);
     signal.e4 *= (SLEW_FILTER);

     signal.e1 += (1 - SLEW_FILTER) * _e1;
    signal.e2 += (1 - SLEW_FILTER) * _e2;
   signal.e3 += (1 - SLEW_FILTER) * _e3;
  signal.e4 += (1 - SLEW_FILTER) * _e4;

  signal.e1 *= E1CENTER;
   signal.e2 *= E2CENTER;
    signal.e3 *= E3CENTER;
     signal.e4 *= E4CENTER;

     signal.e1 += signal.throttle;
   signal.e2 += signal.throttle;
  signal.e3 += signal.throttle;
signal.e4 += signal.throttle;

  if ((signal.Ecal[0] + signal.Ecal[1] + signal.Ecal[2] + signal.Ecal[3]) == 0)
  {

    float eAv = (signal.e1 + signal.e2 + signal.e3 + signal.e4) / 4;
    signal.Ecal[0] = eAv - signal.e1;
     signal.Ecal[1] = eAv - signal.e2;
      signal.Ecal[2] = eAv - signal.e3;
       signal.Ecal[3] = eAv - signal.e4;
  }

     signal.e1 += signal.Ecal[0];
    signal.e2 += signal.Ecal[1];
   signal.e3 += signal.Ecal[2];
  signal.e4 += signal.Ecal[3];

  if ((signal.e1 < MAXVAL) && (signal.e1 > MINVAL))
  {
    esc1.writeMicroseconds((int)signal.e1);
  }
  else if (signal.e1 < MINVAL)
  {
    signal.e1 = MINVAL + 1;
  }
  else if (signal.e1 > MAXVAL)
  {
    signal.e1 = MAXVAL - 1;
  }

  if ((signal.e2 < MAXVAL) && (signal.e2 > MINVAL))
  {
    esc2.writeMicroseconds((int)signal.e2);
  }
  else if (signal.e2 < MINVAL)
  {
    signal.e2 = MINVAL + 1;
  }
  else if (signal.e2 > MAXVAL)
  {
    signal.e2 = MAXVAL - 1;
  }
  if ((signal.e3 < MAXVAL) && (signal.e3 > MINVAL))
  {
    esc3.writeMicroseconds((int)signal.e3);
  }
  else if (signal.e3 < MINVAL)
  {
    signal.e3 = MINVAL + 1;
  }
  else if (signal.e3 > MAXVAL)
  {
    signal.e3 = MAXVAL - 1;
  }
  if ((signal.e4 < MAXVAL) && (signal.e4 > MINVAL))
  {
    esc4.writeMicroseconds((int)signal.e4);
  }
  else if (signal.e4 < MINVAL)
  {
    signal.e4 = MINVAL + 1;
  }
  else if (signal.e4 > MAXVAL)
  {
    signal.e4 = MAXVAL - 1;
  }
}

void printData()
{

  TELEMETRY1.print(millis());
   TELEMETRY1.print(",1:,"), TELEMETRY1.print((int)signal.e1);
    TELEMETRY1.print(",2:,"), TELEMETRY1.print((int)signal.e2);
     TELEMETRY1.print(",3:,"), TELEMETRY1.print((int)signal.e3);
      TELEMETRY1.print(",4:,"), TELEMETRY1.print((int)signal.e4);
       TELEMETRY1.print(",roll:,"), TELEMETRY1.print(X.Full[0], 3);
        TELEMETRY1.print(",pitch:,"), TELEMETRY1.print(X.Full[1], 3);
       TELEMETRY1.print(",yaw:,"), TELEMETRY1.print(X.Full[2], 3);
      TELEMETRY1.print(","), TELEMETRY1.print(X.Full[3], 3);
     TELEMETRY1.print(","), TELEMETRY1.print(X.Full[4], 3);
    TELEMETRY1.print(","), TELEMETRY1.print(X.Full[5], 3);
   TELEMETRY1.print(",Alt:,"), TELEMETRY1.print(altitude.alt);
  TELEMETRY1.print("throttle"), TELEMETRY1.print(signal.throttle);
  
  if (global.TAKEOFF_FLAG)
  {
    TELEMETRY1.print(",Integrators ON, ");
  }
  else
  {
    TELEMETRY1.print(",Integrators OFF, ");
  }
  if (global.VERT_SPEED)
  {
    TELEMETRY1.print(", VERT_SPD_HOLD = true");
  }
  else
  {
    TELEMETRY1.print(", VERT_SPD_HOLD = false");
  }
  TELEMETRY1.println();
}

void printTestData()
{
  TELEMETRY1.println(Ncode);
}

void receiveData()
{

  if (CMD_SERIAL.available() >= 5)
  {

    if ((CMD_SERIAL.read() == 0x20) && (CMD_SERIAL.read() == 0x20))
    {

      char temp = CMD_SERIAL.read();
      uint8_t t1 = CMD_SERIAL.read();

      Ncode = t1;

      if (temp == 'a') // read incoming pitch command
      {
        if ((Ncode >= 0) && (Ncode <= 255))
        {
          Ncode -= 128;
          Ncode *= (0.1);
          Ncode *= PI/180;
          setPoint.R[0] = Ncode;
          
        }
      }
      else if (temp == 'w') // read incoming roll command
      {
        if ((Ncode >= 0) && (Ncode <= 255))
        {
          Ncode -= 128;
          Ncode *= (0.1);
          Ncode *= PI/180;
          setPoint.R[1] = Ncode;
        }
      }
      else if (temp == 'q') // read incoming yaw command
      {
        Ncode -= 128;
          Ncode *= (0.1);
            Ncode *= PI/180;
              setPoint.R[2] = Ncode;
      }
      else if (temp == 'r') // read incoming throttle command
      {
        Ncode -= 128;
        int throttle = (Ncode)*THROTTLE_SCALER;
        signal.throttle = throttle;
      }
      else if (temp == 'V')
      {
        global.VERT_SPEED = !global.VERT_SPEED;
      }
      else if (temp == 'I')
      {
        global.TAKEOFF_FLAG = !(global.TAKEOFF_FLAG);
      }
      else if (temp == 'H')
      {
        Dcode[0] = 'H',
        Dcode[1] = 'O';
        setPoint.R[0] = setPoint.Rcal[0];
        setPoint.R[1] = setPoint.Rcal[1];
        global.STOP_FLAG = false;
      }
      else if (temp == 'Y')
      {
        global.STOP_FLAG = false;
      }
      else if (temp == 'K')
      {
        signal.U[0] = 0;
        global.STOP_FLAG = true;
      }
    }
  }
}