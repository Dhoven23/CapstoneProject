#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPU6050.h>

#define FILT 0.25

  sensors_event_t gravity, a, g, temp, linAccel;

Adafruit_MPU6050 mpu;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int t = 0;
double dt;
double VelX = 0,
       AccX = 0,
       VelY = 0,
       AccY = 0,
       PosX = 0,
       PosY = 0,
       AccAv= 0;
       
double acc_x_old = 0,
       acc_y_old = 0,
       VelXold = 0,
       VelYold = 0;
       
double CalX,
       CalY;
       
void setup() {
  Serial.begin(115200);
  while (!bno.begin())
  {
    Serial.print("bno not found");
    delay(1000);
  }
  while (!mpu.begin())
  {
    Serial.print("mpu nout found");
    delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(500);

  for (int i=0;i<10;i++){
     bno.getEvent(&linAccel,Adafruit_BNO055::VECTOR_LINEARACCEL);
     CalX += linAccel.acceleration.x;
     CalY += linAccel.acceleration.y;
     delay(50);
     Serial.println(CalX);
     Serial.println(CalY);
     Serial.println();
  }
CalX /=10;
CalY /=10;
Serial.println();
Serial.println(CalX);
Serial.println(CalY);
delay(1000);
}

void loop() {

  dt = micros() - t;
  dt /= 1000000;
  t = micros();


  bno.getEvent(&gravity, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&linAccel,Adafruit_BNO055::VECTOR_LINEARACCEL);
  mpu.getEvent(&a, &g, &temp);

  double acc_x = a.acceleration.x;
  double acc_y = a.acceleration.y;
  double acc_z = a.acceleration.z;

  double norm_acc = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  double gravity_x = gravity.acceleration.x;
  double gravity_y = gravity.acceleration.y;
  double gravity_z = gravity.acceleration.z;

  double norm_grav = sqrt((gravity_x*gravity_x)+(gravity_y*gravity_y)+(gravity_z*gravity_z));

  acc_x *= (norm_grav/norm_acc);
  acc_y *= (norm_grav/norm_acc);
  acc_z *= (norm_grav/norm_acc);
  
//  Serial.print("Acc.X: "),Serial.println(acc_x);
//  Serial.print("Acc.Y: "),Serial.println(acc_y);
//  Serial.print("Acc.Z: "),Serial.println(acc_z);
  
  acc_x += gravity_x;
  acc_y += gravity_y;
  acc_z -= gravity_z;
  
  Serial.println("::::::::::: NEW DATA FRAME :::::::::::::::");
  Serial.print("Gravity.X: "),Serial.println(gravity.acceleration.x);
  Serial.print("Gravity.Y: "),Serial.println(gravity.acceleration.y);
  Serial.print("Gravity.Z: "),Serial.println(gravity.acceleration.z);
  Serial.print("Norm (bno) = "),Serial.println(norm_grav);

  acc_x = (1-FILT)*linAccel.acceleration.x + FILT*acc_x_old;
  acc_y = (1-FILT)*linAccel.acceleration.y + FILT*acc_y_old;
  acc_z = linAccel.acceleration.z;
  acc_x -= CalX;
  acc_y -= CalY;

  Serial.print("Acc.X: "),Serial.println(acc_x);
  Serial.print("Acc.Y: "),Serial.println(acc_y);
  Serial.print("Acc.Z: "),Serial.println(acc_z);
  Serial.print("Norm (acc) = "),Serial.println(norm_acc);

  

  VelX += acc_x*dt;
  VelY += acc_y*dt;
  acc_x_old = acc_x;
  acc_y_old = acc_y;
  PosX += VelX*dt;
  PosY += VelY*dt;
  PosY += (dt/2)*(VelYold + VelY);
  VelXold = VelX;
  VelYold = VelY;

  Serial.print("Pos.X: "),Serial.println(PosX);
  Serial.print("Pos.Y: "),Serial.println(PosY);
  //Serial.print("Pos.Z: "),Serial.println(acc_z);

  delay(10);
  
}
