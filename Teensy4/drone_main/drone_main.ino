#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <PWMServo.h>

/* Set controller constants */

#define kp 1
#define ki 0.1
#define kd 0.5
#define LQRmult 0.5

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 50

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

PWMServo esc1,
         esc2,
         esc3,
         esc4;

IntervalTimer BNO055_getSampleTimer;
IntervalTimer USART1_printData;
IntervalTimer USART1_receiveData;
IntervalTimer TFMini_getDistance;

int cycletime = 0,
    pos = 0,
    t = 0,
    i = 0,
    E_old,
    tim_old = 0;

volatile double _q0 = 0, 
                _q1 = 1, 
                _q2 = 0, 
                _q3 = 0;

unsigned int checksum = 0,
             check2 = 0,
             check1,
             altSet,
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
       U [4] = {50, 0, 0, 0},
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

volatile unsigned int overflow0=0;
volatile uint8_t temp[10];

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

void setup(void)
{

  //SerialUSB1.begin(115200);
  Serial1.begin(115200);
  Serial.begin(115200);
  set_liDAR();

  esc1.attach(4);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(7);
  delay(1000);

  esc1.write(30);
  esc2.write(30);
  esc3.write(30);
  esc4.write(30);
  delay(2000);
  for (int i=30;i>20;i--){
    esc1.write(i);
    esc2.write(i);
    esc3.write(i);
    esc4.write(i);
    delay(50);
  }
  for(int i=20;i<35;i++){
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

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  bno.setExtCrystalUse(true);

  /* Initialize timed functions */

  USART1_receiveData.begin(receive_data, 5000);
  BNO055_getSampleTimer.begin(get_IMU_sample,10000);
  TFMini_getDistance.begin(get_Distance_sample,5000);
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

  for(int i=0; i<4; i++){
    for(int j=0; i<6; i++){
      K[i][j] *= LQRmult;
      Serial.print(K[i][j]);    }
  }
  
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
  interrupts();

  //setup derivatives
  float roll_old = roll;
  float pitch_old = roll;
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

  alt = liDARval * cos(roll) * cos(pitch);

  IntegralTracker();
  ELQR_calc();
  //printData();
  commandESCs();
  

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
      for (int i = 0; i < 2; i++)Serial1.read(); // byte 7, 8 are ignored
      checksum = Serial1.read();
    }
  }
}


FASTRUN void ELQR_calc(){

  for(int i=1; i<4; i++){
    U[i] = 0;
  }
  
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 6; j++){
      U[i]+= K[i][j] * (X_Full[j] - R[j]);
    }
  }
}

FASTRUN void IntegralTracker(){
  
  for (int i = 0; i < 3; i++){
    X_int[i] += (dt/2)*(X_old[i]+X_Full[i]);
    X_int[i+3] = X_Full[i];

  }
}

FASTRUN void AltitudePID(){

  
  float E = R[5]-alt;
  float P = E;
  I += E*dt;
  float D = (E-E_old)/dt;
  
  
  U[0] = kp*P + ki*I + kd*D;
  E_old = E;
  
}

void commandESCs() {

  int e1 = U[0] + U[1] + U[3];
  int e2 = U[0] + U[2] - U[3];
  int e3 = U[0] - U[1] - U[3];
  int e4 = U[0] - U[2] + U[3];

  
  if((e1<100)&&(e1>50)){
    esc1.write(e1);
    Serial.println(e1);
  }
  if((e2<100)&&(e2>50)){
    esc2.write(e2);
    Serial.println(e2);
  }
  if((e4<100)&&(e4>50)){
    esc3.write(e4);
    Serial.println(e4);
  }
  if((e3<100)&&(e4>50)){
    esc4.write(e3);
    Serial.println(e3);
  }
 
}

void printData() {

  //Serial.print(',');
  Serial.print(roll, 2), Serial.print(',');
  Serial.print(pitch, 2), Serial.print(',');
  Serial.print(yaw, 2), Serial.print(',');
  Serial.print(" altitude: ,"), Serial.print(alt), Serial.println(", cm");

}

void receive_data() {

  while (Serial.available() > 6)
    { 
      check1 = Serial.read();
      check2 = Serial.read();
      
      if ((check1 == 0x59) && (check2 == 0x59))
      {
          unsigned int t1 = (Serial.read()); // byte 3 = AltSet_L
          unsigned int t2 = (Serial.read()); // byte 4 = AltSet_H
          t2 <<= 8;
          t2 += t1;
          altSet = t2/5;
          t1 = (Serial.read()); // byte 5 = Xrot_L
          t2 = (Serial.read()); // byte 6 = Yrot_H
          t2 <<= 8;
          t2 += t1;
          Xrot = t2;
          t1 = (Serial.read()); // byte 7 = Yrot_L
          t2 = (Serial.read()); // byte 8 = Yrot_H
          t2 <<= 8;
          t2 += t1;
          Yrot = t2;
          unsigned int checksum = Serial.read();
          if (checksum == (((uint8_t)altSet + (uint8_t)Xrot + (uint8_t)Yrot)&255)){
              //R[0] = ((Xrot-360)/4)*(PI/180);
              //R[1] = ((Yrot-360)/4)*(PI/180);
              //R[5] = altSet/4;
          }
      } else {
        Xrot = 360;
        Yrot = 360;
      }
  }

//  Serial.println(R[0]);
//  Serial.println(R[1]);
//  Serial.println(R[5]);
}
