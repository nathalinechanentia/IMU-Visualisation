# IMU-Visualisation
Basic MPU-6050 Arduino sketch 

This sketch demonstrates the fundamental functionality of the MPU-6050, encompassing initialization, accelerometer and gyro calibration. The implementation utilizes the Mahony Orientation Filter as the quaternion filter.

MPU_Mahony.ino can print out quaternions, euler angles, gravity, yaw pitch roll angles, and linear and world acceleration. The quaternions can then be passed on to MPU6050_Mahony_Quat.pde to visualize the real-time orientation of the sensor/object.
