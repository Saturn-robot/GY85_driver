[![Project Status: WIP - Initial development is in progress, but there has not yet been a stable, usable release suitable for the public.](http://www.repostatus.org/badges/latest/wip.svg)](http://www.repostatus.org/#wip)

# 概述

本驱动使用GPL协议，使用本驱动请遵守相关[协议](https://github.com/Saturn-robot/GY85_driver/blob/master/LICENSE)。

这是一个针对惯性测量模组（IMU）的Arduino驱动代码，目前只支持GY85。惯性测量模组是一种用于测量和报告设备速度、方向和重力的电子设备，它能将加速度计和陀螺仪，甚至是磁场强度计等传感器的数据进行综合。GY85在国内是一个很常用的惯性测量模组，用途很广范，主要用于无人机、机器人等。我们主要将其用在轮式机器人上，用于提供机器人的速度和位姿等信息。

# GY85简介

## 组成

GY85包括三个微控制器，包括三轴加速度计、三轴陀螺仪、三轴电子罗盘，分别用于测量加速度、方向以及磁感应强度。其中X轴、Y轴是水平方向的，Z轴是垂直方向的。

![gy85-b](http://img.dxcdn.com/productimages/sku_148436_1_small.jpg)

note:图片来自<http://www.dx.com/>

## 关于roll、pitch、yaw

在使用GY85之前，我们需要学习一些基本知识。飞行器在飞行时有三个自由度，分别是roll、pitch、yaw，如果翻译成中文的话，我们可以分别叫做横滚、俯仰、航向。

![axes](https://upload.wikimedia.org/wikipedia/commons/thumb/5/54/Flight_dynamics_with_text.png/200px-Flight_dynamics_with_text.png)

note:图片来自维基百科:<https://en.wikipedia.org/wiki/Aircraft_principal_axes>。

pitch是围绕X轴旋转，也叫做俯仰角;yaw是围绕Y轴旋转，也叫偏航角;roll是围绕Z轴旋转，也叫翻滚角。

![roll](http://p1.bpimg.com/567571/d450d5fb4a092f72.gif)

roll旋转

![pitch](http://p1.bqimg.com/567571/a92926dde729cecf.gif)

pitch旋转

![yaw](http://p1.bpimg.com/567571/a1b95614beeb9669.gif)

yaw旋转

上述图片来自互联网，关于飞机旋转方式的进一步信息请查阅相关资料。

## 加速度计(ADXL345)

加速度计是一个用于测量固有加速度(Proper acceleration)，即物理加速度的设备，固有加速度不同于我们所熟知的坐标系加速度(速度的变化率)。例如加速度在静止时测得的重力加速度大约为g，即9.81m/s^2，而当其以自由落体运动时测得的加速度则约等于0。详情请参考维基百科:*[Accelerometer](http://en.wikipedia.org/wiki/Accelerometer)*。

GY85所使用的加速度计ADXL345是一个模拟设备，它可以测量三个方向的加速度（x、y、z），具有13位的分辨率，测量范围达±16g。数字输出数据为16位二进制补码格式,可通过SPI(3线或4线)或I2C数字接口访问。ADXL345为驱动中断提供两个输出引脚:INT1和INT2，支持SINGLE_TAP、DOUBLE_TAP等特性，因此可以采用中断的方式来采集数据。

**NOTE:** 关于ADXL345的详细参数和信息请参考其[数据手册](http://www.analog.com/media/cn/technical-documentation/data-sheets/ADXL345_cn.pdf)
尽管ADXL345具有很高的精度，但是我们不可能每次都计算出精确的数据并将其存到芯片中。最常用的方式是多次采样取平均值。

该芯片通常返回的是10位分辨率的数据，为了便于使用，我们需要做一些简单的处理，并将其单位转化为G。

				测量值(单位:G) = 采样值 × (测量范围/(2^分辨率))

公式中测量范围和分辨率取决于芯片的配置。ADXL345支持的测量范围有±2g/±4g/±8g/±16g。一般分辨率可以设置为10或者13位。采样值就是指芯片直接采集到的数据。如果ADXL345使用的是默认设置的话，那么分辨率为10位精度，测量范围为±2g(因此公式中应使用4)，因此根据上述公式可得：

				测量值(单位:G) = 采样值 × (4/2^10) = 采样值 × (1/256) = 采样值 × 0.0039

必须对每个方向的数据都做进行上面的处理，最终的测量值的范围在±1之间。

```
xg = valX * 0.0039;
yg = valY * 0.0039;
zg = valZ * 0.0039;
```

这些测量值可能会有一些噪声，我们可以通过一些滤波方法来改善数据，下面是低通滤波的一个例子，详情请参考[这里](http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/)。

```
fXg = xg * alpha + fXg * (1.0 - alpha);
fYg = yg * alpha + fYg * (1.0 - alpha);
fZg = zg * alpha + fZg * (1.0 - alpha);
```

计算完x、y、z方向的加速度值之后，我们可以用这些值进一步计算角度。我们知道在飞行器理论坐标轴中，关于x轴的旋转叫做翻滚，关于y轴旋转叫做俯仰角，关于z轴旋转叫做偏航，但是三轴加速度计无法计算偏航角，因为重力力矩在移动过程中不会改变。

```
Roll = atan2(yg, zg) * 180/PI;
Pitch = atan2(-xg, sqrt(yg*yg + zg*zg)) * 180/PI;
```

## 陀螺仪(ITG3200/ITG3205)

陀螺仪是一种基于角动量理论来测量或者维持方向的设备。GY85使用InvenSense公司的ITG3200或者ITG3205来测量方向，它可以感知三个方向的移动，传感器数据采用16位模数转换器，除此之外，它还集成了一个温度传感器。

我们需要将采样值除以敏感范围因子(Sensitivity Scale Factor)来得到最终以°/s为单位的值，敏感范围因子可以在[数据手册](https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf)中找到。

				测量值(单位:°/s) = 采样值 / 敏感范围因子

敏感范围因子是一个常量，大小为14.375 LSB pro °/s，因此

				测量值(单位:°/s) = 采样值 / 敏感范围因子

同样将上述公式应用于每个方向，最终结果应该在±2000°/s之间。

```
xds = valX / 14.375;
yds = valY / 14.375;
zds = valZ / 14.375;
```

如果我们将上述值乘以时间间隔的话，

				角度 += 测量值(单位:°/s) × 时间间隔

我们便可得到每个方向的角度：

```
Roll += valX * dtime * 0.001;
Pitch += valY * dtime * 0.001;
Yaw += valZ * dtime * -0.001;
```

我们直接从传感器获得的温度值将是十分巨大的一个值，我们可以使用一个常量将其转换为摄氏度。-13200代表35°，每280就是1°。

				温度(单位:°C) = 35 + (采样值 + 13200) / 280

## 电子罗盘(HMC5883L)

电子罗盘主要用于测量磁性材料，比如磁铁，的磁化强度，以及测量某一点的磁场强度和方向。

在GY85中使用的是Honeywell公司的HMC5883L，它是一个三轴的数码电子罗盘。该芯片被广泛用于数码指南针，用于检测与地磁北极的偏转角度。

将采集到的数据乘以数字分辨率即可得到以高斯为单位的数据：

```
data[0] *= 0.92;
data[1] *= 0.92;
data[2] *= 0.92;
```

**NOTE:** 关于分辨率等参数的详细信息，请参考其[数据手册](http://210.27.82.7/cache/7/03/cloudfront.net/6181d2fb87c03344fcb19a03acc68c69/HMC5883L-FDS.pdf)。
转化为弧度(rad)：

```
Yaw = atan2(Y, X);
```
由于存在磁偏角，所以当值为负时，需要进行校正：

```
if(Yaw < 0)
    Yaw += 2*PI;
```
当存在正磁偏角时，同样需要校正：

```
if(Yaw > 2*PI)
    Yaw -= 2*PI;
```
最后，将结果转化为角度(°):

```
Yaw in ° = Yaw * 180/PI;
```

# 使用本驱动

## 电路连接

下图是GY85与Arduino Mega 2560的电路连接图，图片若加载不出来，请直接查看circuit目录下的图片。

![wiring-mega](https://github.com/Saturn-robot/GY85_driver/blob/master/circuit/wiring-mega2560.jpg)

如果你使用的是其他版本的Arduino，请视情况更改。比如你使用的是Uno的话，只需将GY85的SDA和SCL分别改为A4和A5，即Arduino的I2C接口。不同版本的Arduino I2C接口请查阅Arduino官方网站:<https://www.arduino.cc/en/Reference/Wire>。

## 安装函数库

运行本驱动之前请将lib下的三个函数库放到你的Arduino安装路径下的library目录中。

## 获取数据

打开串口监视器，输入`i`命令即可获得相应的数据。


<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="知识共享许可协议" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />本作品采用<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">知识共享署名-非商业性使用-相同方式共享 4.0 国际许可协议</a>进行许可。
