/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  // while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  // {
  //   delay(500);
  // }
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.CalibrateGyro();
}


void loop()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(ax, sqrt(ay*ay + az*az))*180.0)/M_PI;
  accRoll  = (atan2(ay, az)*180.0)/M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gy);
  kalRoll = kalmanX.update(accRoll, gx);

  //accelerometer-calculated-pitch;accelerometer-calculated-roll;
  //Serial.print(accPitch);
  // Serial.print(":");
  // Serial.print(accRoll);
  // Serial.print(":");
  // Serial.print(kalPitch);
  // Serial.print(":");
  // Serial.print(kalRoll);
  // Serial.print(":");
  // Serial.print(ax);
  // Serial.print(":");
  // Serial.print(ay);
  // Serial.print(":");
  Serial.print(az);
  // Serial.print(":");
  // Serial.print(gx);
  // Serial.print(":");
  // Serial.print(gy);
  // Serial.print(":");
  // Serial.print(gz);

  Serial.println();
}

