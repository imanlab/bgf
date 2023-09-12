#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
 
 
#define BNO055_SAMPLERATE_DELAY_MS (10)
 
Adafruit_BNO055 myIMU = Adafruit_BNO055();
 
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myIMU.begin();
delay(1000);
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
Serial.println("q_w:,q_x:,q_y:,q_z:,acc_x:,acc_y:,acc_z:,cal_acc:,cal_gyr:,cal_mg:,cal_sys:");
}
 
void loop() {
  // put your main code here, to run repeatedly:
uint8_t system, gyro, accel, mg = 0;
myIMU.getCalibration(&system, &gyro, &accel, &mg);
 
imu::Quaternion quat = myIMU.getQuat();
imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 
Serial.print(quat.w());
Serial.print(",");
Serial.print(quat.x());
Serial.print(",");
Serial.print(quat.y());
Serial.print(",");
Serial.print(quat.z());
Serial.print(",");
Serial.print(acc.x());
Serial.print(",");
Serial.print(acc.y());
Serial.print(",");
Serial.print(acc.z());
Serial.print(",");
Serial.print(accel);
Serial.print(",");
Serial.print(gyro);
Serial.print(",");
Serial.print(mg);
Serial.print(",");
Serial.println(system);
 
delay(BNO055_SAMPLERATE_DELAY_MS);
}
