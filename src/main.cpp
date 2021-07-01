#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);
int status;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ = 0;
float roll, pitch, yaw, rollF, pitchF;
float elapsedTime, currentTime, previousTime;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void loop() {
  // read the sensor
  IMU.readSensor();

  AccX = IMU.getAccelX_mss()/9.81;
  AccY = IMU.getAccelY_mss()/9.81;
  AccZ = IMU.getAccelZ_mss()/9.81;
  accAngleX = atan(AccY / sqrt(pow(AccX,2) + pow(AccZ,2))) * 180 / PI;
  accAngleY = atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ , 2))) * 180 / PI;


  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  GyroX = IMU.getGyroX_rads() * 180 / PI;
  GyroY = IMU.getGyroY_rads() * 180 / PI;
  GyroZ = IMU.getGyroZ_rads() * 180 / PI;

  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ * elapsedTime;

  // display the data
  /*Serial.print("Xa = ");
  Serial.print(AccX,6);
  Serial.print("\t");
  Serial.print("Ya = ");
  Serial.print(AccY,6);
  Serial.print("\t");
  Serial.print("Za = ");
  Serial.print(AccZ,6);
  Serial.print("\t");
  Serial.print(gyroAngleX,6);
  Serial.print("\t");
  Serial.print(gyroAngleY,6);
  Serial.print("\t");
  Serial.println(yaw,6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);*/

  

  // Complementary Filter
  roll = 0.5 * gyroAngleX + 0.5 * accAngleX;
  pitch = 0.5 * gyroAngleY + 0.5 * accAngleY;
 
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  //delay(50);
}