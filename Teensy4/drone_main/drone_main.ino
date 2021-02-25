/* --------------------------------------------------------------------------------------
    Main Controller Code to run on Teensy 4.0

    Author:  Daniel Hoven
    Date:    2/24/2021
    Project: Senior Capstone

    Requires: Adafruit Unified Sensor Lib.
              Adafruit BNO055
              avr/io header
              avr/interrupt header
              PWMServo Library (interchangeable with Servo.h stdlib)
  --------------------------------------------------------------------------------------*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <PWMServo.h>

/* ========================================================================================
    Define controller constants
*/

#define kp 2                        // Altitude PID proportional constant
#define ki 0//0.5                      // Altitude PID integral constant
#define kd 0.01                     // Altitude PID derivative constant
#define filtAlt 0.95                // Altitude Estimation Filter constant (larger = more filtration)
#define LQRmult 0.9                 // Scaling factor for control law, varies between 0.5-1
#define LQR_P 2                   // LQR_P constant (proportional scaling for LQR controller)
#define LQR_E 0                     // LQR_E constant (scale steady state error rejection)
#define filtPID 0.95                // Altitude PID Filter constant

/* =========================================================================================
    Define communication setup
*/

#ifndef RADIO_BAUDRATE
#define RADIO_BAUDRATE 57600      // Telemetry radio baudrates (use 57600)
#endif
#ifndef LIDAR_BAUDRATE
#define LIDAR_BAUDRATE 115200     // LiDAR sensor UART speed. default is 115200
#endif
#ifndef USB_BAUDRATE
#define USB_BAUDRATE 115200       // USB Serial port baudtate. (N/A for USB mode)
#endif
#ifndef SERIAL_USB
#define SERIAL_USB Serial         // Serial port for USB communication (always Serial)
#endif
#ifndef RADIO_SERIAL
#define RADIO_SERIAL Serial4      // Serial port for radio communication
#endif
#ifndef LIDAR_SERIAL
#define LIDAR_SERIAL Serial1      // Serial port for LIDAR communication (Serial1 on PCB)
#endif

/* =========================================================================================
    Define motor setup
*/

#define ESC1 6
#define ESC2 7
#define ESC3 5
#define ESC4 4

#define MAXVAL 90
#define MINVAL 40

/* Set the delay between iterations */
#define MAIN_DELAY 1

/* ========================================================================================
    Declare Library Objects
*/

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

PWMServo esc1,
         esc2,
         esc3,
         esc4;

IntervalTimer BNO055_getSampleTimer;
IntervalTimer PrintDataTimer;
IntervalTimer ReceiveDataTimer;


/* =========================================================================================
    Declare globals
*/

int cycletime = 0,
    pos = 0,
    t = 0,
    i = 0,
    E_old,
    tim_old = 0;

int e1 = 0,
    e2 = 0,
    e3 = 0,
    e4 = 0;

volatile double _q0 = 0,
                _q1 = 1,
                _q2 = 0,
                _q3 = 0;

volatile float liDARold = 0,
               _lidar = 0;

unsigned int checksum = 0,
             check2 = 0,
             check1,
             altSet = 20,
             Xrot = 360,
             Yrot = 360;

volatile int liDARval = 0,
             strength = 0;

double pitch,
       roll,
       yaw,
       d_roll,
       d_pitch,
       d_yaw,
       alt = 0,
       dt = 0,
       I;

double X_Full [6] = {0, 0, 0, 0, 0, 0}, // RAM1 arrays
                    R [6] = {0, 0, 0, 0, 0, 0},
                            U [4] = {50, 0, 0, 0},
                                    Rcal[3] = {0, 0, 0};


double X_int [6],                       // RAM2 arrays
       X_old [6];

double K [4][6] = {{                    // Derive with MATLAB
    0, 0, 0, 0, 0, 0,
  },
  {
    38.7298, 0, 0, 4.5102, 0, 0,
  },
  {
    0, 38.7298, 0, 0, 4.5104, 0,
  },
  {
    0, 0, 0, 0, 0, 0,
  },
};

char Dcode[3];
uint16_t Ncode;

String STATE = "STARTUP";

/*=================================================================================
   SETUP
*/

void set_liDAR() {

  LIDAR_SERIAL.write(0x42);
  LIDAR_SERIAL.write(0x57);
  LIDAR_SERIAL.write(0x02);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x00);
  LIDAR_SERIAL.write(0x01);
  LIDAR_SERIAL.write(0x06);
}

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

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      K[i][j] *= LQRmult;
    }
  }

  //  esc1.attach(ESC1);
  //  esc2.attach(ESC2);
  //  esc3.attach(ESC3);
  //  esc4.attach(ESC4);
  //  delay(100);
  //  esc1.write(30);
  //  esc2.write(30);
  //  esc3.write(30);
  //  esc4.write(30);
  //  delay(2000);
  //  for (int i = 30; i > 20; i--) {
  //    esc1.write(i);
  //    esc2.write(i);
  //    esc3.write(i);
  //    esc4.write(i);
  //    delay(50);
  //  }
  //  for (int i = 20; i < 35; i++) {
  //    esc1.write(i);
  //    esc2.write(i);
  //    esc3.write(i);
  //    esc4.write(i);
  //    delay(150);
  //  }
  //  delay(5000);
  //  esc1.write(50);
  //  esc2.write(50);
  //  esc3.write(50);
  //  esc4.write(50);
  //  delay(5000);



  /* Initialize timed functions */

  ReceiveDataTimer.begin(receiveData, 10000);
  BNO055_getSampleTimer.begin(get_IMU_sample, 10000);
  PrintDataTimer.begin(printData, 10000);

  PrintDataTimer.priority(3);
  ReceiveDataTimer.priority(2);
  BNO055_getSampleTimer.priority(1);

  delay(500);

  noInterrupts();
  float q0 = _q0;
  float q1 = _q1;
  float q2 = _q2;
  float q3 = _q3;
  interrupts();
  R[0] = -atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2));
  R[1] = asin(2.0 * (q2 * q0 - q3 * q1));
  R[2] = -atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));
  Rcal[0] = R[0];
  Rcal[1] = R[1];
  Rcal[2] = R[2];


}

/*-------------------------------------------------------------------------------
   main
  ------------------------------------------------------------------------------*/

void loop(void)
{
  noInterrupts();

  dt = micros() - t;
  t = micros();
  dt = dt / 1000000;
  
  get_Distance_sample();
  _lidar = (1 - filtAlt) * liDARval + filtAlt * liDARold;
  liDARold = _lidar;
  
  alt = _lidar * cos(roll) * cos(pitch);

  // cosine error removal (altitude)

  IntegralTracker();
  ELQR_calc();
  //AltitudePID();
  commandESCs();

  interrupts();

  delay(MAIN_DELAY);

}

/*-------------------------------------------------------------------------
   Write external functions
  ---------------------------------------------------------------------------*/

FASTRUN void get_IMU_sample() {
  /* get quaternions */

  imu::Quaternion quat = bno.getQuat();


  _q0 = quat.w();
  _q1 = quat.x();
  _q2 = quat.y();
  _q3 = quat.z();


  double q0 = _q0;
  double q1 = _q1;
  double q2 = _q2;
  double q3 = _q3;


  double roll_old = roll;
  double pitch_old = pitch;
  double yaw_old = yaw;

  //quaternion conversion
  roll  = (-Rcal[0]) - atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2)); // * (180/PI);
  pitch = (-Rcal[1]) + asin(2.0 * (q2 * q0 - q3 * q1));// * (180/PI);
  yaw   = (-Rcal[2]) - atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1)); // * (180/PI);

  d_roll = (roll - roll_old) / dt;
  d_pitch = (pitch - pitch_old) / dt;
  d_yaw = (yaw - yaw_old) / dt;

  for (int i = 0; i < 6; i++) {
    X_old[i] = X_Full[i];
  }

  X_Full[0] = roll;
  X_Full[1] = pitch;
  X_Full[2] = yaw;
  X_Full[3] = d_roll;
  X_Full[4] = d_pitch;
  X_Full[5] = d_yaw;

}

void get_Distance_sample() {

  if (LIDAR_SERIAL.available() >= 9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if ((0x59 == LIDAR_SERIAL.read()) && (0x59 == LIDAR_SERIAL.read())) // byte 1 and byte 2
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
      for (int i = 0; i < 3; i++)LIDAR_SERIAL.read(); // byte 7, 8 are ignored
    }
  }
}


FASTRUN void ELQR_calc() {

  for (int i = 1; i < 4; i++) {
    U[i] = 0;
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      U[i] += K[i][j] * ((X_Full[j] - R[j]) + LQR_E * (X_int[j]));
    }
    for (int j = 3; j < 6; j++) {
      U[i] += K[i][j] * ((X_Full[j] - R[j]) + LQR_P * (X_int[j]));
    }
  }
}

FASTRUN void IntegralTracker() {

  for (int i = 0; i < 3; i++) {
    X_int[i] += (dt / 2) * (X_old[i] + X_Full[i]);
    X_int[i + 3] = X_Full[i];

  }
}

FASTRUN void AltitudePID() {


  float E = altSet - alt;
  float P = E;
  if ((I < 30) && (I > (-30))) {
    I += E * dt;
  } else {
    if (I > 0)I = 29;
    if (I < 0)I = -29;
  }
  float D = (E - E_old) / dt;


  U[0] = filtPID * U[0] + (1 - filtPID) * (kp * P + ki * I + kd * D);
  E_old = E;

}

void commandESCs() {


  e1 = U[0] - U[1] + U[2] + U[3];
  e2 = U[0] - U[1] - U[2] - U[3];
  e3 = U[0] + U[1] - U[2] + U[3];
  e4 = U[0] + U[1] + U[2] - U[3];


  if ((e1 < MAXVAL) && (e1 > MINVAL)) {
    esc1.write(e1);

  } else if (e1 < 50) {
    e1 = MINVAL + 1;

  } else if (e1 > 50) {
    e1 = MAXVAL - 1;
  }

  if ((e2 < MAXVAL) && (e2 > MINVAL)) {
    esc2.write(e2);

  } else if (e2 < 50) {
    e2 = MINVAL + 1;

  } else if (e2 > 50) {
    e2 = MAXVAL - 1;
  }

  if ((e3 < MAXVAL) && (e3 > MINVAL)) {
    esc3.write(e3);

  } else if (e3 < 50) {
    e3 = MINVAL + 1;

  } else if (e3 > 50) {
    e3 = MAXVAL - 1;
  }

  if ((e4 < MAXVAL) && (e4 > MINVAL)) {
    esc1.write(e4);

  } else if (e4 < 50) {
    e4 = MINVAL + 1;

  } else if (e4 > 50) {
    e4 = MAXVAL - 1;
  }

}

void printData() {

  SERIAL_USB.print("1: "), SERIAL_USB.print(e1);
  SERIAL_USB.print(",   2: "), SERIAL_USB.print(e2);
  SERIAL_USB.print(",   3: "), SERIAL_USB.print(e3);
  SERIAL_USB.print(",   4: "), SERIAL_USB.println(e4);

  // Serial.println(dt, 4);

  //    SERIAL_USB.print(roll, 2), SERIAL_USB.print(',');
  //    SERIAL_USB.print(pitch, 2), SERIAL_USB.print(',');
  //    SERIAL_USB.print(yaw, 2), SERIAL_USB.println(',');

  //  SERIAL_USB.print("RawValue: "),SERIAL_USB.println(liDARval);
  //  SERIAL_USB.print("altitude: ,"), SERIAL_USB.print(alt), SERIAL_USB.print(" cm, U[0] = ");
  //
  //    SERIAL_USB.println(U[0]);
  //    SERIAL_USB.println(U[1]);
  //    SERIAL_USB.println(U[2]);
  //    SERIAL_USB.println(U[3]);

//  SERIAL_USB.println("X[Full] = ");
//  SERIAL_USB.println(X_Full[0], 4);
//  SERIAL_USB.println(X_Full[1], 4);
//  SERIAL_USB.println(X_Full[2], 4);
//  SERIAL_USB.println(X_Full[3], 8);
//  SERIAL_USB.println(X_Full[4], 8);
//  SERIAL_USB.println(X_Full[5], 8);

  //  SERIAL_USB.print("4 "), SERIAL_USB.println(e4);
  //  SERIAL_USB.print("R[0] : "), SERIAL_USB.println(R[0]);
  //  SERIAL_USB.print("R[1] : "), SERIAL_USB.println(R[1]);



}

void receiveData() {

  if (RADIO_SERIAL.available() >= 5) {

    if ((RADIO_SERIAL.read() == 0x20) && (RADIO_SERIAL.read() == 0x20))
    {
      char temp = RADIO_SERIAL.read();
      if (temp == 'a') {
        Dcode[0] = '-', Dcode[1] = 'x';
        R[0] += 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'w') {
        Dcode[0] = '+', Dcode[1] = 'y';
        R[1] += 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 's') {
        Dcode[0] = '-', Dcode[1] = 'y';
        R[1] -= 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'd') {
        Dcode[0] = '+', Dcode[1] = 'x';
        R[0] -= 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'H') {
        Dcode[0] = 'H', Dcode[1] = 'O';
        R[0] = Rcal[0];
        R[1] = Rcal[1];
        digitalWrite(13, HIGH);
      } else if (temp == 'r') {
        SERIAL_USB.print("+z: ");
        Dcode[0] = '+', Dcode[1] = 'z';
        U[0] += 1;
      } else if (temp == 'f') {
        SERIAL_USB.print("-z: ");
        Dcode[0] = '-', Dcode[1] = 'z';
        U[0] -= 1;
        digitalWrite(13, HIGH);
      } else if (temp == 'K') {
        altSet = 20;
        SERIAL_USB.println("Halted!!");

      } else {
        Dcode[0] = 'N', Dcode[1] = 'A';
      }
      uint16_t t1 = RADIO_SERIAL.read();
      uint16_t t2 = RADIO_SERIAL.read();
      t2 <<= 8;
      t1 += t2;
      if (!((Dcode[0] == 'N') || (Dcode[1] == 'A'))) {
        Ncode = t1;
      }

      digitalWrite(13, LOW);
    }
  } else {
    digitalWrite(13, LOW);
  }

}
