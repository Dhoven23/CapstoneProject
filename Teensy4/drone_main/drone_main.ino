#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 10

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


Servo esc1,
      esc2,
      esc3,
      esc4;

IntervalTimer SwitchMode;
IntervalTimer BNO055_getSampleTimer;
IntervalTimer USART1_printData;
IntervalTimer USART1_receiveData;
IntervalTimer TFMini_getDistance;


int cycletime = 0, 
    pos = 0,
    t = 0, 
    i = 0;
    tim_old = 0;
    
volatile float _q0 = 0, 
               _q1 = 1, 
               _q2 = 0, 
               _q3 = 0;

volatile int liDARval = 0,
             strength = 0;
               
float pitch, 
      roll, 
      yaw,
      alt;

float Full_X [6] = {0, 0, 0, 0, 0, 0};
float K [4][6];
float U [4] = {0, 0, 0, 0};
float R [6] = {0, 0, 0, 0, 0, 0};

volatile unsigned int overflow0=0;
volatile char temp = '0';



/*=================================================================================
 * SETUP
 */

void set_liDAR(){
  
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
  
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(100);
  set_liDAR();
  

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

 
  bno.setExtCrystalUse(true);

  /* Initialize timed functions */

  USART1_receiveData.begin(receive_data, 50000);
  SwitchMode.begin(ModeSelect, 100000);
  
}

/*-------------------------------------------------------------------------------
   main
  ------------------------------------------------------------------------------*/

void loop(void)
{
 
 dt = micros() - t;
 t = micros();
 double dt = dt/1000000;
 
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
 float d_roll = (roll - roll_old)/dt;
 float d_pitch = (pitch - pitch_old)/dt;
 float d_yaw = (yaw - yaw_old)/dt;



 X_Full[0] = roll;
 X_Full[1] = pitch;
 X_Full[2] = yaw;
 X_Full[3] = d_roll;
 X_Full[4] = d_pitch;
 X_Full[5] = d_yaw;

 // cosine error removal (altitude)

 alt = liDARval * cos(roll) * cos(pitch);

 IntegralTracker();
 
 delay(BNO055_SAMPLERATE_DELAY_MS);
 
}

/*-------------------------------------------------------------------------
   Write external functions
  ---------------------------------------------------------------------------*/


void ModeSelect() {
  
  switch (temp) {
    
    case 'S': //silent
    
        USART1_printData.end();
      break;
      
    case 'R': //recovery

        Serial.println("Recovery mode entered... ");
        BNO055_getSampleTimer.end();
        USART1_printData.end();
        USART1_receiveData.end();
        SwitchMode.end();
        Serial.println("Jobs halted... ");
        Serial1.end();
        delay(100);
        Serial1.begin(115200);
        Serial.println("Serial communication restarted... ");
        set_liDAR();
        Serial.println("Lidar Reinitialized... ");
        
        if (!bno.begin())
        {
          /* There was a problem detecting the BNO055 ... check your connections */
          Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
          while (1);
        }
        bno.setExtCrystalUse(true);
        Serial.println("IMU rebooted... ");
        BNO055_getSampleTimer.begin(get_IMU_sample, 10000);
        USART1_printData.begin(print_data, 10000);
        USART1_receiveData.begin(receive_data, 50000);
        SwitchMode.begin(ModeSelect, 100000);
        Serial.println("Jobs restarted");
        temp = 'N';
      break;
      
    case 'N': //normal mode
    
        BNO055_getSampleTimer.begin(get_IMU_sample, 10000);
        USART1_printData.begin(print_data, 10000);
        USART1_receiveData.begin(receive_data, 50000);
      break;
      
    case 'Q': //quit
        BNO055_getSampleTimer.end();
        USART1_printData.end();
        USART1_receiveData.end();
      break;

    case 'P':
        Serial.println(",Ping...,");
        
    default:
        delay(10);
      break;
}
}


void get_IMU_sample() {
  /* get quaternions */

 //Serial.print(1000000/dt),Serial.println(" Hz,");
  imu::Quaternion quat = bno.getQuat();
  

  _q0 = quat.w();
  _q1 = quat.x();
  _q2 = quat.y();
  _q3 = quat.z();
  
}

void get_Distance_sample(){
      
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
        for (int i = 0; i < 3; i++)Serial1.read(); // byte 7, 8, 9 are ignored
      }
    } 
}


void ELQR_calc(){

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 6; j++){
      U[i]+= K[i][j] * (Full_X[j] - R[j]);
    }
  }
}

void IntegralTracker(){
  
  int tim = micros();
  float dt = 0.000001*(tim-tim_old);
  tim_old = tim;
  
  for (int i = 0; i < 3; i++){
    X_int[i] += (dt/2)*(X_old[i]+X_full[i]);
    X_int[i+3] = X_Full[i];
  }
}

void commandESCs(){

  int e1 = U[1] + 

  noInterrupts();
  esc1.write(U[0]*(180/100));
  esc2.write(U[1]*(180/100));
  esc2.write(U[2]*(180/100));
  esc2.write(U[3]*(180/100));
  interrupts();
}

void print_data(){
  
  Serial.print(',');
  Serial.print(roll,4), Serial.print(',');
  Serial.print(pitch,4),Serial.print(',');
  Serial.print(yaw,4),Serial.print(',');
  Serial.print(" altitude: ,"),Serial.print(alt,4),Serial.println(", cm");
  
}

void receive_data(){
  if (Serial.available() > 1){ 
    temp = Serial.read();
  }
}
