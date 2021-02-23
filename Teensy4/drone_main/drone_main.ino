#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <PWMServo.h>

/* Set controller constants */

#define kp 2
#define ki 0.5
#define kd 0.01
#define LQRmult 0.75
#define filtAlt 0.95
#define filtPID 0.95

/* Define motor attach pins */
#define ESC1 6
#define ESC2 7
#define ESC3 5
#define ESC4 4

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 0

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

PWMServo esc1,
         esc2,
         esc3,
         esc4;

IntervalTimer BNO055_getSampleTimer;
IntervalTimer USART1_printData;
IntervalTimer USART1_receiveData;




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
             altSet = 10,
             Xrot = 360,
             Yrot = 360;

volatile int liDARval = 0,
             strength = 0;

double pitch,
       roll,
       yaw,
       alt,
       dt = 0,
       I;

double X_Full [6] = {0, 0, 0, 0, 0, 0},
                    X_int [6],
                    X_old [6],
                    U [4] = {0, 0, 0, 0},
                    Rcal[3] = {0, 0, 0},
                    R [6] = {0, 0, 0, 0, 0, 0};

double K [4][6] = {{
    0, 0, 0, 0, 0, 0,
  },
  {
    38.7298, 0, 0, 4.5102, 0, 0,
  },
  {
    0, 38.7298, 0, 0, 4.5104, 0,
  },
  {
    0, 0, 14.1421, 0, 0, 14.1474,
  },
};

char Dcode[3];
uint16_t Ncode;

/*=================================================================================
   SETUP
*/

void set_liDAR() {

  Serial1.write(0x42);
  Serial1.write(0x57);
  Serial1.write(0x02);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0x01);
  Serial1.write(0x06);
}

void setup()
{

  Serial.begin(250000);
  Serial1.begin(115200);
  Serial4.begin(57600);
  delay(100);
  set_liDAR();

  while (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(200);
  }
  Serial.print("IMU found\n");
  bno.setExtCrystalUse(true);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      K[i][j] *= LQRmult;
    }
  }

  esc1.attach(ESC1);
  esc2.attach(ESC2);
  esc3.attach(ESC3);
  esc4.attach(ESC4);
  delay(100);
  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
  delay(2000);
  for (int i = 30; i > 20; i--) {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    delay(50);
  }
  for (int i = 20; i < 35; i++) {
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    delay(150);
  }
  delay(5000);
  esc1.write(50);
  esc2.write(50);
  esc3.write(50);
  esc4.write(50);
  delay(5000);



  /* Initialize timed functions */

  USART1_receiveData.begin(receiveData, 10000);
  BNO055_getSampleTimer.begin(get_IMU_sample, 10000);


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

  dt = micros() - t;
  t = micros();
  dt = dt / 1000000;

  noInterrupts();
  float q0 = _q0;
  float q1 = _q1;
  float q2 = _q2;
  float q3 = _q3;
  get_Distance_sample();
  _lidar = (1 - filtAlt) * liDARval + filtAlt * liDARold;
  liDARold = _lidar;
  alt = _lidar * cos(roll) * cos(pitch);
  interrupts();

  //setup derivatives
  float roll_old = roll;
  float pitch_old = pitch;
  float yaw_old = yaw;

  //quaternion conversion
  roll  = -atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2));// * (180/PI);
  pitch = asin(2.0 * (q2 * q0 - q3 * q1));// * (180/PI);
  yaw   = -atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));// * (180/PI);

  // compute derivatives
  float d_roll = (roll - roll_old) / dt;
  float d_pitch = (pitch - pitch_old) / dt;
  float d_yaw = (yaw - yaw_old) / dt;

  X_Full[0] = roll;
  X_Full[1] = pitch;
  X_Full[2] = yaw;
  X_Full[3] = d_roll;
  X_Full[4] = d_pitch;
  X_Full[5] = d_yaw;

  // cosine error removal (altitude)

  IntegralTracker();
  ELQR_calc();
  AltitudePID();
  commandESCs();
  printData();

  delay(BNO055_SAMPLERATE_DELAY_MS);

}

/*-------------------------------------------------------------------------
   Write external functions
  ---------------------------------------------------------------------------*/

void get_IMU_sample() {
  /* get quaternions */

  //Serial.print(1000000/dt),Serial.println(" Hz,");
  imu::Quaternion quat = bno.getQuat();


  _q0 = quat.w();
  _q1 = quat.x();
  _q2 = quat.y();
  _q3 = quat.z();

}

void get_Distance_sample() {

  if (Serial1.available() >= 9) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if ((0x59 == Serial1.read()) && (0x59 == Serial1.read())) // byte 1 and byte 2
    {
      unsigned int t1 = Serial1.read(); // byte 3 = Dist_L
      unsigned int t2 = Serial1.read(); // byte 4 = Dist_H
      t2 <<= 8;
      t2 += t1;
      liDARval = t2;
      t1 = Serial1.read(); // byte 5 = Strength_L
      t2 = Serial1.read(); // byte 6 = Strength_H
      t2 <<= 8;
      t2 += t1;
      strength = t2;
      for (int i = 0; i < 3; i++)Serial1.read(); // byte 7, 8 are ignored
    }
  }
}


FASTRUN void ELQR_calc() {

  for (int i = 1; i < 4; i++) {
    U[i] = 0;
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 6; j++) {
      U[i] += K[i][j] * (X_Full[j] - R[j]);
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
    if (I > 0)I =29;
    if (I < 0)I =-29;
  }
  float D = (E - E_old) / dt;


  U[0] = filtPID * U[0] + (1 - filtPID) * (kp * P + ki * I + kd * D);
  E_old = E;

}

void commandESCs() {

  e1 = U[0] - U[1] + U[2];
  e2 = U[0] - U[1] - U[2];
  e3 = U[0] + U[1] - U[2];
  e4 = U[0] + U[1] + U[2];


  if ((e1 < 90) && (e1 > 40)) {
    esc1.write(e1);
    Serial4.print("1 "), Serial4.println(e1);
  }
  if ((e2 < 90) && (e2 > 40)) {
    esc2.write(e2);
    Serial4.print("2 "), Serial4.println(e2);
  }
  if ((e4 < 90) && (e4 > 40)) {
    esc3.write(e3);
    Serial4.print("3 "), Serial4.println(e3);
  }
  if ((e3 < 90) && (e4 > 40)) {
    esc4.write(e4);
    Serial4.print("4 "), Serial4.println(e4);
  }

}

void printData() {

  //  Serial.print(',');
    Serial4.print(roll, 2), Serial4.print(',');
    Serial4.print(pitch, 2), Serial4.print(',');
    Serial4.print(yaw, 2), Serial4.println(',');
    Serial4.print(altSet), Serial4.println();
  //Serial.print("RawValue: "),Serial.println(liDARval);
  //  Serial.print("altitude: ,"), Serial.print(alt), Serial.print(" cm, U[0] = ");
//    Serial.println(U[0]);
//    Serial.println(U[1]);
//    Serial.println(U[2]);
//    Serial.println(U[3]);
  //  Serial.println("X[Full] = ");
  //  Serial.println(X_Full[0]);
  //  Serial.println(X_Full[1]);
  //  Serial.println(X_Full[2]);
  //  Serial.println(X_Full[3]);
  //  Serial.println(X_Full[4]);
  //  Serial.println(X_Full[5]);
  
  
  
  //Serial.print("4 "), Serial.println(e4);
//  Serial.print("R[0] : "), Serial.println(R[0]);
//  Serial.print("R[1] : "), Serial.println(R[1]);



}

void receiveData() {

  if (Serial4.available() >= 5) {

    if ((Serial4.read() == 0x20) && (Serial4.read() == 0x20))
    {
      char temp = Serial4.read();
      if (temp == 'a') {
        //Serial.print("-x: ");
        Dcode[0] = '-', Dcode[1] = 'x';
        R[0] += 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'w') {
        //Serial.print("+y: ");
        Dcode[0] = '+', Dcode[1] = 'y';
        R[1] += 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 's') {
        //Serial.print("-y: ");
        Dcode[0] = '-', Dcode[1] = 'y';
        R[1] -= 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'd') {
        //Serial.print("+x: ");
        Dcode[0] = '+', Dcode[1] = 'x';
        R[0] -= 0.01;
        digitalWrite(13, HIGH);
      } else if (temp == 'H') {
        //Serial.print("HOME: ");
        Dcode[0] = 'H', Dcode[1] = 'O';
        R[0] = Rcal[0];
        R[1] = Rcal[1];
        digitalWrite(13, HIGH);
      } else if (temp == 'r') {
        Serial.print("+z: ");
        Dcode[0] = '+', Dcode[1] = 'z';
        altSet += 1;
      } else if (temp == 'f') {
        Serial.print("-z: ");
        Dcode[0] = '-', Dcode[1] = 'z';
        altSet -= 1;
        digitalWrite(13, HIGH);
      } else if (temp == 'K') {
          altSet = 20;
          Serial.println("Halted!!");
          
      } else {
        //Serial.print("NULL");
        Dcode[0] = 'N', Dcode[1] = 'A';
      }
      uint16_t t1 = Serial4.read();
      uint16_t t2 = Serial4.read();
      t2 <<= 8;
      t1 += t2;
      if (!((Dcode[0] == 'N') || (Dcode[1] == 'A'))) {
        Ncode = t1;
        //Serial.print(Ncode);
      }

      digitalWrite(13, LOW);
      //Serial.println();
    }
  } else {
    digitalWrite(13, LOW);
  }

}
