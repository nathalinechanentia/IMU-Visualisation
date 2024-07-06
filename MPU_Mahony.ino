// MPU-6050 Mahony IMU  
// Run the IMU_Zero file before uploading this file 

#include "Wire.h"
#include "helper_3dmath.h"

#define OUTPUT_QUATERNION
// #define OUTPUT_EULER
// #define OUTPUT_GRAVITY
// #define OUTPUT_YAWPITCHROLL
// #define OUTPUT_LINEARACCEL
// #define OUTPUT_WORLDACCEL

int MPU_addr = 0x68;
uint8_t devAddr;
void *wireObj = 0;
float euler_angles[3];
float gravity[3];
float ypr[3];
float aaReal[3];
VectorInt16 aaWorld;

// -499.5, -17.7, -82.0
float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = {0.0, -14, -12}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 200; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output

void setup() {

  Wire.begin();
  Serial.begin(115200);
  Serial.println("starting");

  // initialize sensor
  // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

// AHRS loop
void loop()
{
  static float deltat = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers

  //raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Axyz[0] = (float) ax;
  Axyz[1] = (float) ay;
  Axyz[2] = (float) az;
  
  // Serial.print(ax);
  // Serial.print(" ");
  // Serial.print(ay);
  // Serial.print(" ");
  // Serial.println(az);

  // Serial.print(Axyz[0] / 16384);
  // Serial.print(" ");
  // Serial.print(Axyz[1] / 16384);
  // Serial.print(" ");
  // Serial.println(Axyz[2] / 16384);

  Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
  Gxyz[1] = ((float) gy - G_off[1]) * gscale;
  Gxyz[2] = ((float) gz - G_off[2]) * gscale;

  // Serial.print(Gxyz[0]);
  // Serial.print(" ");
  // Serial.print(Gxyz[1]);
  // Serial.print(" ");
  // Serial.println(Gxyz[2]);

  now = micros();
  deltat = (now - last) * 1.0e-6; //seconds since last update
  last = now;

  #ifdef OUTPUT_QUATERNION
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    Serial.print("w");
    Serial.print(q[0]);
    Serial.print("w");
    Serial.print("a");
    Serial.print(q[1]);
    Serial.print("a");
    Serial.print("b");
    Serial.print(q[2]);
    Serial.print("b");
    Serial.print("c");
    Serial.print(q[3]);
    Serial.println("c");
  #endif 

  #ifdef OUTPUT_EULER
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    GetEuler(euler_angles, q);
    Serial.print(euler_angles[0] * 180/M_PI);
    Serial.print(" ");
    Serial.print(euler_angles[1] * 180/M_PI);
    Serial.print(" ");
    Serial.println(euler_angles[2] * 180/M_PI);
  #endif

  #ifdef OUTPUT_GRAVITY
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    GetGravity(gravity, q);
    Serial.print(gravity[0]);
    Serial.print(" ");
    Serial.print(gravity[1]);
    Serial.print(" ");
    Serial.println(gravity[2]);
  #endif 

  #ifdef OUTPUT_YAWPITCHROLL
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    GetGravity(gravity, q);
    GetYawPitchRoll(ypr, q, gravity);
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(" ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(" ");
    Serial.println(ypr[2] * 180/M_PI);
  #endif

  #ifdef OUTPUT_LINEARACCEL
    GetGravity(gravity, q);
    GetLinearAccel(aaReal, Axyz, gravity);
    Serial.print(aaReal[0] / 16384);
    Serial.print(" ");
    Serial.print(aaReal[1] / 16384);
    Serial.print(" ");
    Serial.println(aaReal[2] / 16384);
  #endif

  #ifdef OUTPUT_WORLDACCEL
    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);
    GetGravity(gravity, q);
    GetLinearAccel(aaReal, Axyz, gravity);
    GetLinearWorldAccel(aaWorld, aaReal, q);
    // Serial.print(aaWorld.x / 16384);
    // Serial.print(" ");
    // Serial.print(aaWorld.y / 16384);
    // Serial.print(" ");
    // Serial.println(aaWorld.z / 16384);
  #endif

}
//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  tmp = ax * ax + ay * ay + az * az;
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2]; 
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }
    else {     
      ix = 0.0f;
      iy = 0.0f; 
      iz = 0.0f;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;      // prevent integral windup
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, q cross gyro term
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

// q[0] = q -> w;
// q[1] = q -> x;
// q[2] = q -> y; 
// q[3] = q -> z;

void GetEuler(float *data, float q[4]) {
  data[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1); // psi
  data[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]); // theta
  data[2] = atan2(2*q[2]*q[3]- 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1); // phi
}

void GetGravity(float *data, float q[4]) {
  data[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  data[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
  data[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

void GetYawPitchRoll(float *data, float q[4], float *gravity) {
  // yaw (about z-axis)
  data[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] -1);
  // pitch (nose up/down, about Y axis)
  data[1] = atan2(gravity[0], sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
  // roll (tilt left/right, about x-axis) 
  data[2] = atan2(gravity[1], gravity[2]);
  if (gravity[2] < 0) {
    if (data[1] > 0) {
      data[1] = PI - data[1];
    }
    else { 
      data[1] = -PI - data[1];
    }
  }
}

void GetLinearAccel(float *data, float Axyz[3], float gravity[3]) {
  // get rid of the gravity component (+1g = +8192, sensitivity is 2g);
  data[0] = Axyz[0] - gravity[0] * 16384;
  data[1] = Axyz[1] - gravity[1] * 16384;
  data[2] = Axyz[2] - gravity[2] * 16384;
}

void GetLinearWorldAccel(VectorInt16 aaWorld, float *linearAccel, float q[4]) {
  VectorInt16 aaLinear(aaReal[0], aaReal[1], aaReal[2]);
  Quaternion quat(q[0], q[1], q[2], q[3]);    
  memcpy(&aaWorld, &aaLinear, sizeof(VectorInt16));
  aaWorld.rotate(&quat);
}