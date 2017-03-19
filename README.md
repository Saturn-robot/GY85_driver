# 概述

GY85是imu的一种型号，这里我们用一个GY85驱动程序读取imu三个传感器的信息，用值返回。如下所示：

```
switch (cmd) {
		case READ_IMU:
			imu_data = readIMU();
			Serial.print(imu_data.ax);
			Serial.print(F(" "));
			Serial.print(imu_data.ay);
			Serial.print(F(" "));
      Serial.print(imu_data.az);
      Serial.print(F(" "));
      Serial.print(imu_data.gx);
      Serial.print(F(" "));
      Serial.print(imu_data.gy);
      Serial.print(F(" "));
      Serial.print(imu_data.gz);
      Serial.print(F(" "));
      Serial.print(imu_data.mx);
      Serial.print(F(" "));
      Serial.print(imu_data.my);
      Serial.print(F(" "));
      Serial.print(imu_data.mz);
      Serial.print(F(" "));
      Serial.print(imu_data.roll);
      Serial.print(F(" "));
      Serial.print(imu_data.pitch);
      Serial.print(F(" "));
      Serial.println(imu_data.uh);
		  break;
```
这段代码中使用了“imu_data = readIMU();”这条语句。readIMU（）的返回值是结构体：
```
typedef struct imuData_s
    {
      float ax = -999;
      float ay = -999;
      float az = -999;
      float gx = -999;
      float gy = -999;
      float gz = -999;
      float mx = -999;
      float my = -999;
      float mz = -999;
      float roll = -999;
      float pitch = -999;
      float uh = -999;
    } imuData;
```

结构体中有9个变量，这9个变量我们是从三个传感器中分别获得的。
在readIMU中我们定义了结构体变量 imuData_s data做为返回值。
通过给结构体中的变量赋予测量值来得到我们想要的数据，赋值过程如下（有部分代码省略）：

 ```
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
```      
最后再返回:

 ```
      return data;
 ```
