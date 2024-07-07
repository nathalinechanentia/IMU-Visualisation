# IMU-Visualisation
Basic MPU-6050 Arduino sketch 

This sketch demonstrates MPU-6050 basic functionality including initialization, accelerometer and gyro calibration. The quaternion filter used here is the Mahony Orientation Filter. 

MPU_Mahony.ino can print out quaternions, euler angles, gravity, yaw pitch roll angles, and linear and world acceleration. The quaternions can then be passed on to MPU6050_Mahony_Quat.pde to visualize the real-time orientation of the sensor/object.
