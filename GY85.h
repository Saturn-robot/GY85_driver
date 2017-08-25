#ifndef GY85_H
#define GY85_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <ITG3200.h>

struct imuData
{
  float ax;  float ay;  float az;
  float gx;  float gy;  float gz;
  float mx;  float my;  float mz;
  //float roll;  float pitch;  float yaw;
  imuData()
  {
    ax = -999;  ax = -999;  az = -999;
    gx = -999;  gy = -999;  gz = -999;
    mx = -999;  mz = -999;  mz = -999;
    //roll = -999;  pitch = -999;  yaw = -999;
  }
};

class GY85
{
private:
  Adafruit_ADXL345_Unified accel;
  Adafruit_HMC5883_Unified mag;
  ITG3200 gyro;
  imuData data;
public:
  GY85(int accelID, int gyroID, int magID);
  void init();
  // Get imu data
  imuData readIMU();
  // Only get accel data
  void readAccel(float accels[]);
  // Only get gyro data
  void readGyro(float gyros[]);
  // Only get mag data
  void readMag(float mags[]);
};
#endif
