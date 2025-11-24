# ESP32 Micropython Magnetomter Driver (GY-271 module with QMC5883P chip) #

### The Code: ### 

This code is a driver for the QMC5883P magnetometer chip, commonly found on the GY-271 module, designed to run on an ESP32 in micropython. The code configures the module to the settings described below, and provides methods that allow you to get raw magnetometer readings (in Gauss) out, or allow the module to act as a compass (either pitch/roll compensated, or not).

The compass_2d method uses a basic algorithm to determine the module's heading (taking the chip's negative x axis pointing north as 0 degrees), which assumes that the module is flat. The heading returned is rounded to the nearest degree. You can input the magnetic declination for your location to make the method output true heading, rather than magnetic heading.

The compass_3d method uses a more complex algorithm, and takes a quaternion orientation input (typically gathered from an IMU such as the MPU-6050 I am using). This allows the algorithm to compensate for the sensor's pitch and roll angles - which affect the magnetometer readings - and produce a much more accurate heading (rounded to the nearest degree again), no matter the sensor's orientation. This method could easily be adapted to take pitch and roll Euler angles as input, rather than a quaternion, using the algorithms laid out here [1] to replace the quaternion rotations of the magnetometer and heading vectors.

You can also get raw magnetometer data out if you want to do outher things with the data - use the getdata_raw method for this. The readings are returned in Gauss, with a 2G range.

Calibration is done by calling the calibrate() method. The algorithm here compensates for both hard and soft iron effects - see the references for a detailed guide to magnetometer calibration. This code gets you to rotate the sensor - you need to rotate it 360 degrees around one axis (x, y or z) and then 360 degrees around a different axis, to complete a calibration rotation. The code provides a user output to let you know what it's up to. The code may complete calibration before you've done the full 360 degree rotation - this is not an issue. Calibration does not have to be completed to use the sensor, but is suggested - particularly for compass applications.

In the directory embedded_c_module, you will find the .c, .h, and .cmake files required to compile this QMC58833P driver into your micropython firmware. This has the full functionality of the MicroPython driver. Both the compass_2d and compass_3d methods require the declination parameter, and the calibrate() method does not accept the calibrationrotations parameter (it defaults to 1; I thought that anything else seemed redundant). See below for details on how to compile this into your micropython board's firmware.

### Python Example Usage: ###

```python3
gy271 = Magnetometer(42, 24)

q = imu.getorientation() # For example

gy271.calibrate(calibrationrotations=1) # Optional parameter to tell the code to only complete one set of calibration rotations - default value is 2.

data = gy271.getdata_raw()
heading_2d = gy271.compass_2d(declination=1.5)
heading_3d = gy271.compass_3d(q, declination=1.5)
```

The two parameters that must be given to initialse the driver are the magnetometer's SCL and SDA pin numbers, in that order.

### Embedded C Module Example Usage: ###

```python3
import qmc5883p

scl_pin = 1
sda_pin = 2
# Port can be either 0 or 1, or -1 to automatically select. Default is -1
port = 0
declination = 5
quaternion = [1, 0, 0, 0]

magnetometer = qmc5883p.QMC5883P(scl_pin, sda_pin, i2c_port=port)

magnetometer.calibrate()
raw = magnetometer.getdata_raw()
heading = magnetometer.compass_2d(declination)
heading = magnetometer.compass_3d(quaternion, declination)
```
SCL/SDA pins are required, i2c_port is an optional parameter.

### Compiling the module into firmware: ###

To do this, you will need:
 - ESP-IDF cloned from github
 - Micropython cloned from github

1. Enter your esp-idf directory, and run ./install.sh (only needs to be run the first time)
2. Enter your esp-idf directory and run . ./export.sh (needs to be run every new terminal session)
3. Download the files from embedded_c_module and place them in a directory of your choosing
4. Enter your directory ~/micropython/ports/esp32 (can be replaced with whichever micropython board you are using)
5. Run the make command, specifying USER_C_MODULES=/path/to/QMC5883P_magnetometer/embedded_c_module/micropython.cmake (replace with your file path)

For me, with an ESP32-S3 that has octal SPIRAM, the full make command is:
```
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT USER_C_MODULES=/path/to/QMC5883P_magnetometer/embedded_c_module/micropython.cmake
```

### Module Configuration Settings: ###

The QMC5883P chip is configured to the following settings:
 - Normal power mode
 - 200Hz data output rate
 - 4x sensor reads per data output
 - No down sampling
 - 2 Gauss sensor range
 - Set and reset mode on

### References: ###

 - Datasheet: <https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-19%20QMC5883P%20Datasheet%20Rev.C(1).pdf>
 - Guide to calibration: <https://www.appelsiini.net/2018/calibrate-magnetometer/>
 
[1]: <http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf>