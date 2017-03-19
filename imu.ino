#include <Wire.h>
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>

ADXL345 acc; //variable adxl is an instance of the ADXL345 library
HMC5883L compass;
ITG3200 gyro = ITG3200();

float  gx,gy,gz;
float  gx_rate, gy_rate, gz_rate;
int ix, iy, iz;
float anglegx=0.0, anglegy=0.0, anglegz=0.0;
int ax,ay,az;
int rawX, rawY, rawZ;
float X, Y, Z;
float rollrad, pitchrad;
float rolldeg, pitchdeg;
int error = 0;
float aoffsetX, aoffsetY, aoffsetZ;
float goffsetX, goffsetY, goffsetZ;
unsigned long time, looptime;

void initIMU()
{
  acc.powerOn();
  compass = HMC5883L();
  error = compass.SetScale(1.3); // Set the scale to +/- 1.3 Ga of the compass
  if(error != 0) // If there is an error, print it out.
      Serial.println(compass.GetErrorText(error));

  // Serial.println("Setting measurement mode to continous");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
  Serial.println(compass.GetErrorText(error));

  gyro.init(ITG3200_ADDR_AD0_LOW);
}

imuData readIMU()
{
  imuData_s data;
  time = millis();
  acc.readAccel(&ax, &ay, &az); //read the accelerometer values and store them in variables  x,y,z

  X = ax/256.00; // used for angle calculations
  Y = ay/256.00; // used for angle calculations
  Z = az/256.00; // used for angle calculations

  data.ax = X;
  data.ay = Y;
  data.az = Z;


  //GYRO
  rollrad = atan(Y/sqrt(X*X+Z*Z));  // calculated angle in radians
  pitchrad = atan(X/sqrt(Y*Y+Z*Z)); // calculated angle in radians
  rolldeg = 180*(atan(Y/sqrt(X*X+Z*Z)))/PI; // calculated angle in degrees
  pitchdeg = 180*(atan(X/sqrt(Y*Y+Z*Z)))/PI; // calculated angle in degrees

  // Code fragment for Magnetometer heading (yaw)
  MagnetometerRaw raw = compass.ReadRawAxis();
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI;

  // Code fragment for Gyroscope (roll, pitch, yaw)
  gyro.readGyro(&gx,&gy,&gz);
  looptime = millis() - time;
  gx_rate = (gx) / 14.375;
  gy_rate = (gy) / 14.375;
  gz_rate = (gz) / 14.375;

  data.gx = gx_rate;
  data.gy = gy_rate;
  data.gz = gz_rate;

  data.mx = scaled.XAxis;
  data.my = scaled.YAxis;
  data.mz = scaled.ZAxis;

  data.roll = rollrad;
  data.pitch = pitchrad;
  data.uh = heading;

  return data;
}
