#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 50

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

// declare ext. vars. 
float ANG_X_STPT=0,ANG_Y_STPT=0,ANG_Z_STPT=0, A_STPT = 0;
float ERR_X_ANG, ERR_Y_ANG, ERR_Z_ANG, ERR_alt;
float yaw_offset_initial = 0;
float q0, q1, q2, q3;
float pitch, roll, yaw;
int i = 0, count = 0;
float THRUST = 80;
float E1, E2, E3, E4;
float TauX, TauY, TauZ;
float off_set, Alt;
unsigned long time, time_old;
float dt;


// stuff for untrasonic sensor
int trigPin=32;
int echoPin=31;
int pingTravelTime;
float pingTravelDistance;
float distanceToTarget;


// PID stuff 
float kP_1=3,kI_1=0,kD_1=0; // altitude PID consts. 

float kP_X=-1,kI_X=0, kD_X=0.5; // X rot. PID consts. 

float kP_Y=1,kI_Y=0, kD_Y=-0.5;  // Y rot. PID consts. 

float kP_Z=1,kI_Z=0, kD_Z=0;  // Z rot. PID consts. 


float ERR_X_ANG_old,ERR_Y_ANG_old,ERR_Z_ANG_old, ERR_alt_old, Int_alt=0, IntX=0,IntY=0,IntZ=0; 
float DiffX, DiffY;


void setup(void)
{
  Serial.begin(115200);

   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  esc1.attach(16,1000,2000);
  esc2.attach(17,1000,2000);
  esc3.attach(3,1000,2000);
  esc4.attach(2,1000,2000); 

  esc1.write(0);
 esc2.write(0);
 esc3.write(0);
 esc4.write(0);

 pinMode(trigPin, OUTPUT);
 pinMode(echoPin,INPUT);
 
  
  delay(1000);
  bno.setExtCrystalUse(true);
}

/*-------------------------------------------------------------------------------
 * main
 ------------------------------------------------------------------------------*/

void loop(void) {


time = millis();


// ================================================================//
if (time > 10000){
  A_STPT = 50;
}
else {
  A_STPT = 0;
}
// ================================================================= //



 ERR_alt = A_STPT - readaltitude_US(); 
 
 ERR_X_ANG, ERR_Y_ANG, ERR_Z_ANG = read_angle_errors();

     

/*-----------------------------------------------------------------------      
 * PID CONTROLLER
 -----------------------------------------------------------------------*/
DiffX = (ERR_X_ANG - ERR_X_ANG_old)/dt;
DiffY = (ERR_Y_ANG - ERR_Y_ANG_old)/dt;

IntX = IntX + ERR_X_ANG*dt; // Forward Euler Integrators. dt small enough, will update to trapezoidal later. 
IntY = IntY + ERR_Y_ANG*dt;


 
THRUST = kP_1 * ERR_alt;


      TauX = kP_X*ERR_X_ANG + kI_X * IntX + kD_X*DiffX;
      TauY = kP_Y*ERR_Y_ANG + kI_Y * IntY + kD_Y*DiffX;
      TauZ = 0;
  
// compute motor speeds

      E1 = THRUST- TauX - TauY - TauZ;  
      E2 = THRUST- TauX + TauY + TauZ;  
      E3 = THRUST+ TauX + TauY - TauZ;  
      E4 = THRUST+ TauX - TauY + TauZ;

 
// write motor speeds 
 esc1.write(E1);
 esc2.write(E2);
 esc3.write(E3);
 esc4.write(E4);
 
 
 
// print data 
 Serial.print(E1);
 Serial.print(",");
 Serial.print(E2);
 Serial.print(",");
 Serial.print(E3);
 Serial.print(",");
 Serial.println(E4);
 Serial.println();

 

     dt = millis() - time;
     time = millis();
     dt = .001*dt;

    
     
     
     delay(BNO055_SAMPLERATE_DELAY_MS);

     
     ERR_X_ANG_old = ERR_X_ANG;
     ERR_Y_ANG_old = ERR_Y_ANG;
     ERR_Z_ANG_old = ERR_Z_ANG;
    
}




/*-------------------------------------------------------------------------
 * Write external functions
 * 
 * 
 ---------------------------------------------------------------------------*/


float read_angle_errors() {
  /* get quaternions */

  imu::Quaternion quat=bno.getQuat();
 
    q0=quat.w();
    q1=quat.x();
    q2=quat.y();
    q3=quat.z();
 
      roll  = atan2(2.0 * (q3 * q2 + q0 * q1) , 1.0 - 2.0 * (q1 * q1 + q2 * q2));
      pitch = asin(2.0 * (q2 * q0 - q3 * q1));
      yaw   = atan2(2.0 * (q3 * q0 + q1 * q2) , - 1.0 + 2.0 * (q0 * q0 + q1 * q1));

      roll = roll*(-180/3.141592653597);
      pitch = pitch*(-180/3.141592653597);
      yaw = yaw*(180/3.141592653597);

// Serial.print(pitch);
// Serial.print(",");
// Serial.print(roll);
// Serial.print(",");
// Serial.println(yaw); 

 //Serial.println(readaltitude_US());

 if (i<25){
  ANG_X_STPT = pitch;
  ANG_Y_STPT = roll;
  ANG_Z_STPT = yaw;
 }
 else {
  ANG_X_STPT = ANG_X_STPT;
  ANG_Y_STPT = ANG_Y_STPT;
  ANG_Z_STPT = ANG_Z_STPT;
 }
 i++;

  
  /* get errors*/
  ERR_X_ANG = ANG_X_STPT - pitch;
  ERR_Y_ANG = ANG_Y_STPT - roll;
  ERR_Z_ANG = ANG_Z_STPT - yaw;

  return ERR_X_ANG, ERR_Y_ANG, ERR_Z_ANG;

}

float readaltitude_US() {

  digitalWrite(trigPin,LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  pingTravelTime=pulseIn(echoPin,HIGH);
  delay(25);
  pingTravelDistance=(pingTravelTime*765.*5280.*12)/(3600.*1000000);
  distanceToTarget=pingTravelDistance/2;

  if (count < 20){
    off_set = distanceToTarget; 
  }
  else {
    off_set = off_set;
  }
  count++;
  
Alt = distanceToTarget - off_set;
  return Alt;
}

 
 
