#include <Wire.h>
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>  // ITG3200 GyroGyroscope Library

ADXL345 acc; //variable adxl is an instance of the ADXL345 library
HMC5883L compass;
ITG3200 gyro = ITG3200();
const float GyroPercentage = 0.9;

void initIMU()
{
  int error = 0;
  acc.powerOn();
  compass = HMC5883L();
  error = compass.SetScale(1.3); // Set the scale to +/- 1.3 Ga of the compass
  if(error != 0) // If there is an error, print it out.
      Serial.println(compass.GetErrorText(error));

  // Serial.println("Setting measurement mode to continous");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
  Serial.println(compass.GetErrorText(error));
  delay(1000);
  gyro.init(ITG3200_ADDR_AD0_LOW);
}

float complementaryFilter(float gyro_angle, float accelorMag_angle)
{
  return GyroPercentage * gyro_angle + (1 - GyroPercentage) * accelorMag_angle;
}

imuData readIMU()
{
  imuData_s data;
  int raw_ax, raw_ay, raw_az;
  float raw_gx, raw_gy, raw_gz;
  float accel_roll = 0.0, accel_pitch = 0.0;
  float gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;

  unsigned long time = millis();

  // code fragment for Accelerometer angles (roll and pitch)
  acc.readAccel(&raw_ax, &raw_ay, &raw_az); //read the accelerometer values and store them in variables  x,y,z
  // get values in unit G
  float ax = raw_ax * 0.0039;
  float ay = raw_ay * 0.0039;
  float az = raw_az * 0.0039;

  // Code fragment for Magnetometer yaw
  MagnetometerRaw raw = compass.ReadRawAxis();
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  //int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float mag_yaw = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.0457;
  mag_yaw += declinationAngle;

  if(mag_yaw < 0)
    mag_yaw += 2*PI;
  if(mag_yaw > 2*PI)
    mag_yaw -= 2*PI;

  // Code fragment for Gyroscope (roll, pitch, yaw)
  gyro.readGyro(&raw_gx,&raw_gy,&raw_gz);

  // Get delta time
  unsigned long looptime = millis() - time;
  // get values of gyro
  float gx = (raw_gx) / 14.375;
  float gy = (raw_gy) / 14.375;
  float gz = (raw_gz) / 14.375;

  // compute accel angles of x, y
  accel_roll = atan(ay / sqrt(ax * ax + az * az));
  accel_pitch = atan(ax / sqrt(ay * ay + az * az));
  // compute gyro angles of x, y, z
  gyro_roll += (gx * looptime);
  gyro_pitch += (gy * looptime);
  gyro_yaw += (gz * looptime);

  // use complementary filter to get accurate angles
  float accurateRoll = complementaryFilter(gyro_roll, accel_roll);
  float accuratePitch = complementaryFilter(gyro_pitch, accel_pitch);
  float accurateYaw = complementaryFilter(gyro_yaw, mag_yaw);

  // assign corresponding vales of data
  data.ax = ax;   data.gx = gx;   data.mx = scaled.XAxis;
  data.ay = ay;   data.gy = gy;   data.my = scaled.YAxis;
  data.az = az;   data.gz = gz;   data.mz = scaled.ZAxis;
  data.roll = accurateRoll;
  data.pitch = accuratePitch;
  data.uh = accurateYaw;

  return data;
}
