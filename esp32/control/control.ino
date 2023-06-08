#include <AccelStepper.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

AccelStepper s1(AccelStepper::DRIVER, 27, 26); //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, 25, 33); //STRL, DIRL

Adafruit_MPU6050 mpu;

#define DIRR 1 //set to +-1 to compensate for rotating direction
#define DIRL 1

double theta;
double alpha;

void setup(){
  Serial.begin(115200);  
  s1.setMaxSpeed(700);
  s2.setMaxSpeed(700);
  mpu.begin();
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop(){
//sensor data in rad/s
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  theta = -g.gyro.y; //tilt forward = positive
  alpha = g.gyro.x; //rotate right = positive
  Serial.println(theta);

//sets speed of motors, in rad/s
  setSpeed(2, 2);

//to simulate sampling time, motor doesn't work well for 0.1s for now
  delay(100);

}

void setSpeed(int r, int l){
  s1.setSpeed(r*DIRR*100/PI);
  s2.setSpeed(l*DIRL*100/PI);
  s1.runSpeed();
  s2.runSpeed();
}
