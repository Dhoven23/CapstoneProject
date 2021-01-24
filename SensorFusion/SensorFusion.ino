/* This code is free and open source. I only ask that if you use my code in an academic setting, reference it appropriately. 
*  I mean, an extra citation can't hurt can it?
* 
* Author: Daniel Hoven
* Univeristy: Grand Canyon University
* Email: Daniel.Hoven@gcu.edu
*/

/* This code is for sensor fusion of IMU data with barometric altitude 
 *  and accelerometer data for odometric calculation of position. 
 */

// This code relies on the Adafruit Unified Sensor Library

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPU6050.h>
#include <utility/imumaths.h>


// Adafruit Sensors Initialize
Adafruit_BMP085 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_MPU6050 mpu;


// Declare ext. vars
float acc_x = 0,
      acc_y = 0,
      acc_z = 0,
      vel_x = 0,
      vel_y = 0,
      vel_z = 0,
      pos_x = 0,
      pos_y = 0,
      pos_z = 0,
      alt = 0, 
      alt_old = 0, 
      off_set = 0,
      q0,
      q1,
      q2,
      q3,
      av_alt[10],
      C[4];

float gravity_quaternion[4] = {0, 0, 0, 10.1};

      
// Declare Processor dependent vars.       
volatile float _q0 = 0, 
               _q1 = 1, 
               _q2 = 0, 
               _q3 = 0;
  
void setup() {
  
  Serial.begin(115200);
  delay(100);

  // Initialize Barometer
  if (!bmp.begin()) {
	  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	  while (1) {
	    delay(10);
	    }
  }

  // Initialize Accelerometer
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  // Initialize IMU
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1){
      delay(10);
    }
  }

  //Set Accelerometer limits
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Get Baseline alititude
  delay(500);
  off_set = bmp.readAltitude(101500);
  delay(1000);
}

  
void loop() {
   
    ReadBarometricAltitude();
    get_IMU_sample();
    get_Accels();

    
    // math for gravity correction
    q0 = _q0;
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
    
    float gamma  = -atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2));// * (180/PI);
    float beta = asin(2.0 * (q2 * q0 - q3 * q1));// * (180/PI);
    float alpha   = -atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));// * (180/PI);

    float RotationMatrix[3][3];
    
          RotationMatrix[0][0] = cos(alpha)*cos(beta);
          RotationMatrix[1][0] = cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma);
          RotationMatrix[2][0] = cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma);
          RotationMatrix[0][1] = sin(alpha)*cos(beta);
          RotationMatrix[1][1] = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
          RotationMatrix[2][1] = sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma);
          RotationMatrix[0][2] = -sin(beta);
          RotationMatrix[1][2] = cos(beta)*sin(gamma);
          RotationMatrix[2][2] = cos(beta)*cos(gamma);

//          for(int i=0;i<3;i++){
//            Serial.print("| ");
//            for(int j=0;j<3;j++){
//              Serial.print(RotationMatrix[i][j]),Serial.print(' ');
//            }
//            Serial.println(" |");
//          }

    

    float gravity[3] = {0, 0, 1},
          rot_quat[4] = {q0, q1, q2, q3},
          rot_quat2[4] = {q0,-q1,-q2,-q3},
          rot_gravity[4],
          acc_quat[4] = {0,acc_x,acc_y,acc_z};
                      
          
      
    // Rotate Gravity
    HamiltonProduct(rot_quat, gravity_quaternion);
    HamiltonProduct(C,rot_quat2); 

    // Store output
    for(int i=0;i<4;i++){
      rot_gravity[i] = C[i];
    }

    // Rotate Acceleration
    HamiltonProduct(rot_quat, acc_quat);
    HamiltonProduct(C,rot_quat2);

    for(int i=0;i<4;i++){
      acc_quat[i] = C[i];
    }
    
    
    //acc_x-=rot_gravity[1]; // Remove Gravity (i)
    //acc_y-=rot_gravity[2]; // Remove Gravity (j)
    //acc_z-=rot_gravity[3]; // Remove Gravity (k)

    

    Serial.print(" gravity.x "),Serial.print(rot_gravity[1],2);
    Serial.print(" gravity.y "),Serial.print(rot_gravity[2],2);
    Serial.print(" gravity.z "),Serial.println(rot_gravity[3],2);
    Serial.print(" acc.x: "),Serial.print(acc_x);
    Serial.print(" acc.y: "),Serial.print(acc_y);
    Serial.print(" acc.z: "),Serial.println(acc_z);
    Serial.print(" roll: "),Serial.print(gamma);
    Serial.print(" pitch: "),Serial.print(beta);
    Serial.print(" yaw: "),Serial.println(alpha);
    
    
    //Serial.print(", Alt: "),Serial.print(alt*100),Serial.println(" cm");
    
    delay(10);
}


void get_IMU_sample() {
  /* get quaternions */
  imu::Quaternion quat = bno.getQuat();
  

  _q0 = quat.w();
  _q1 = quat.x();
  _q2 = quat.y();
  _q3 = quat.z();
  
}

void ReadBarometricAltitude(){
  
    alt = bmp.readAltitude(101500) - off_set;
    
    float num = 0;
    
    for(int i = 0;i<9;i++){ av_alt[i] = av_alt[i+1]; num+=av_alt[i];}
    
    av_alt[9] = alt;
    num+=alt;
    alt=num/50; 
    
    
}

void get_Accels(){

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
 
    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;
}

void HamiltonProduct(float A[4], float B[4]){

    float a1 = A[0], //input quaternion A
          a2 = A[1], // (i)
          a3 = A[2], // (j)
          a4 = A[3]; // (k)

    float b1 = B[0], //input quaternion B
          b2 = B[1], // (i)
          b3 = B[2], // (j)
          b4 = B[3]; // (k)
          
    
    float c1 = a1*b1 - a2*b2 - a3*b3 - a4*b4, // Output Quaternion (w)
          c2 = a1*b2 + a2*b1 + a3*b4 - a4*b3, // Output Quaternion (i)
          c3 = a1*b3 - a2*b4 + a3*b1 + a4*b2, // Output Quaternion (j)
          c4 = a1*b4 + a2*b3 - a3*b2 + a4*b1; // Output Quaternion (k)
          
    C[0] = c1;
    C[1] = c2;
    C[2] = c3;
    C[3] = c4;

}
