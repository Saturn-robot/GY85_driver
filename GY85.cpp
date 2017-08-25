#include "GY85.h"

GY85::GY85(int accelID, int gyroID, int magID)
{
  accel = Adafruit_ADXL345_Unified(accelID);
  mag = Adafruit_HMC5883_Unified(magID);
  gyro = ITG3200();
  data = imuData();
}

void GY85::init()
{
  accel.begin();
  mag.begin();
  gyro.begin();
  //delay(1000);
  gyro.init(ITG3200_ADDR_AD0_LOW);
  // Calibrate gyro zero
  gyro.zeroCalibrate(2500, 2);
}

imuData GY85::readIMU()
{
  sensors_event_t accelEvent;
  sensors_event_t magEvent;
  sensors_vec_t   orient;
  float gyroEvent[3];

  accel.getEvent(&accelEvent);
  data.ax = accelEvent.acceleration.x;
  data.ay = accelEvent.acceleration.y;
  data.az = accelEvent.acceleration.z;

  gyro.readGyro(&gyroEvent[0], &gyroEvent[1], &gyroEvent[2]);
  data.gx = gyroEvent[0];
  data.gy = gyroEvent[1];
  data.gz = gyroEvent[2];

  mag.getEvent(&magEvent);
  data.mx = magEvent.magnetic.x;
  data.my = magEvent.magnetic.y;
  data.mz = magEvent.magnetic.z;

  return data;
}

void GY85::readAccel(float accels[])
{
  sensors_event_t accelEvent;

  accel.getEvent(&accelEvent);

  accels[0] = accelEvent.acceleration.x;
  accels[1] = accelEvent.acceleration.y;
  accels[2] = accelEvent.acceleration.z;
}

void GY85::readGyro(float gyros[])
{
  gyro.readGyro(&gyros[0], &gyros[1], &gyros[2]);
}

void GY85::readMag(float mags[])
{
  sensors_event_t magEvent;

  mag.getEvent(&magEvent);

  mags[0] = magEvent.magnetic.x;
  mags[1] = magEvent.magnetic.y;
  mags[2] = magEvent.magnetic.z;
}
