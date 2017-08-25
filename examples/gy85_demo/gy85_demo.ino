#include <GY85.h>

GY85 gy85(1234, 1235, 1236);

void setup()
{
  Serial.begin(9600);
  gy85.init();
}

void loop()
{
  float accels[3];
  float gyros[3];
  float mags[3];

  gy85.readAccel(accels);
  gy85.readGyro(gyros);
  gy85.readMag(mags);

  Serial.println("-------Accels-------");
  Serial.print("X: ");  Serial.println(accels[0]);
  Serial.print("Y: ");  Serial.println(accels[1]);
  Serial.print("Z: ");  Serial.println(accels[2]);

  Serial.println("-------Gyros-------");
  Serial.print("X: ");  Serial.println(gyros[0]);
  Serial.print("Y: ");  Serial.println(gyros[1]);
  Serial.print("Z: ");  Serial.println(gyros[2]);

  Serial.println("-------Mags-------");
  Serial.print("X: ");  Serial.println(mags[0]);
  Serial.print("Y: ");  Serial.println(mags[1]);
  Serial.print("Z: ");  Serial.println(mags[2]);
}
