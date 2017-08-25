#include <GY85.h>

GY85 gy85(1234, 1235, 1236);
imuData imu = imuData();

void setup()
{
  Serial.begin(9600);
  gy85.init();
}

void loop()
{
  imu = gy85.readIMU();

  Serial.println("-------Accels-------");
  Serial.print("X: ");  Serial.println(imu.ax);
  Serial.print("Y: ");  Serial.println(imu.ay);
  Serial.print("Z: ");  Serial.println(imu.az);

  Serial.println("-------Gyros-------");
  Serial.print("X: ");  Serial.println(imu.gx);
  Serial.print("Y: ");  Serial.println(imu.gy);
  Serial.print("Z: ");  Serial.println(imu.gz);

  Serial.println("-------Mags-------");
  Serial.print("X: ");  Serial.println(imu.mx);
  Serial.print("Y: ");  Serial.println(imu.my);
  Serial.print("Z: ");  Serial.println(imu.mz);
}
