#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int STRR = 23;
const int DIRR = 19;
const int STRL = 18;
const int DIRL = 5;


Adafruit_MPU6050 mpu;



double theta;
double alpha;

void setup(){
  Serial.begin(115200);  
  ledcSetup(1, 100, 10);
  ledcAttachPin(STRR, 1);
  ledcWrite(1, 127);
  ledcSetup(2, 100, 10);
  ledcAttachPin(STRL, 2);
  ledcWrite(2, 127);
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
  if(r < 0){
    digitalWrite(DIRR, HIGH); //opposite direction as the left motor
  }
  if(r > 0){
    digitalWrite(DIRR, LOW);
  }
  if(l < 0){
    digitalWrite(DIRL, LOW);
  }
  if(l > 0){
    digitalWrite(DIRL, HIGH);
  }
  ledcChangeFrequency(1, r*100/PI, 10);
  ledcChangeFrequency(2, l*100/PI, 10);
}