#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);
int status;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ = 0;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void calculate_IMU_errors();

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

  calculate_IMU_errors();
  delay(20);
}

void loop() {
  // read the sensor
  IMU.readSensor();

  AccX = IMU.getAccelX_mss()/9.81;
  AccY = IMU.getAccelY_mss()/9.81;
  AccZ = IMU.getAccelZ_mss()/9.81;
  accAngleX = (atan(AccY / sqrt(pow(AccX,2) + pow(AccZ,2))) * 180 / PI) - 1.16;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ , 2))) * 180 / PI) + 2.84;


  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime)/1000;
  GyroX = (IMU.getGyroX_rads() * 180 / PI) - 1.12;
  GyroY = (IMU.getGyroY_rads() * 180 / PI) + 2.72;
  GyroZ = (IMU.getGyroZ_rads() * 180 / PI) - 1.59;

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
  roll = 0.94 * gyroAngleX + 0.06 * accAngleX;
  pitch = 0.94 * gyroAngleY + 0.06 * accAngleY;
 
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  //Serial.print("/");
  //Serial.println(elapsedTime*1000);
  delay(50);
}

void calculate_IMU_errors(){
  while (c < 200){
    AccX = IMU.getAccelX_mss()/9.81;
    AccY = IMU.getAccelY_mss()/9.81;
    AccZ = IMU.getAccelZ_mss()/9.81;

    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  // Read Gyro Values 200 times. 
  while (c < 200){
    GyroX = IMU.getGyroX_rads() * 180 / PI;
    GyroY = IMU.getGyroY_rads() * 180 / PI;
    GyroZ = IMU.getGyroZ_rads() * 180 / PI;
    // Sum all readings
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print all the error values.
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}