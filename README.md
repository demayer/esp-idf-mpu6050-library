# Mpu6050 Example

This library uses the native esp-idf i2c interface to read data from an mpu6050 imu.
It includes adafruit's AHRS Madgwick filter to combine the accelerometer and gyroscope data in order to calculate
roll, pitch and yaw angles.

## How to Use Example

This library was tested on an esp32. Hook your imu to the following pins and it should work.:)
CLK Pin: 23
SDA Pin: 22

## How to Embed Example in Other Project

Copy the folder components including the files mpu6050.hpp and mpu6050.cpp into your existing project just like here.
In order to use C++ code, make the following changes:

1) Rename file main.c to main.cpp
2) In \main\CMakeLists.txt change SRCS "main.c" to "main.cpp" as this library uses C++
3) If you were to program further cpp files in the components folder, register them in the \main\components\CMakeLists.txt. For example:
set(component_srcs
    "src/motor.cpp"
    "src/mpu6050.cpp"
)
4) Delete the build folder before executing a new build.