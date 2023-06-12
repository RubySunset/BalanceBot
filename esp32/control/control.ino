#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>
const int STRR = 27;
const int DIRR = 26;
const int STRL = 25;
const int DIRL = 33;

const int SAMPLE_TIME = 1; //sampling time in ms

AccelStepper s1(AccelStepper::DRIVER, STRR, DIRR); //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, STRL, DIRL); //STRL, DIRL

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

double theta;
double integral_theta = 0;
double alpha;

bool sample_flag = false;
hw_timer_t *sample_time = NULL;

void IRAM_ATTR doSample(){
  sample_flag = true;
}

void setSpeed(int r, int l){
 s1.setSpeed(r*100/PI);
 s2.setSpeed(-l*100/PI);
 s1.runSpeed();
 s2.runSpeed();
}


void setup() {
  Serial.begin(115200);

  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setXAccelOffset(-896);
  mpu.setYAccelOffset(-838);
  mpu.setZAccelOffset(1139);
  mpu.setXGyroOffset(88);
  mpu.setYGyroOffset(-48);
  mpu.setZGyroOffset(19);
    
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  s1.setMaxSpeed(700);
  s2.setMaxSpeed(700);
  
  sample_time = timerBegin(0, 80, true);
  timerAttachInterrupt(sample_time, &doSample, true);
  timerAlarmWrite(sample_time, 1000*SAMPLE_TIME, true); //sampling time in microseconds
  timerAlarmEnable(sample_time);

}
void loop() {
  
  if(sample_flag == true){
    //Write sensor sampling and control code here
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    theta = -gy/131; //angles in degrees/s
    alpha = gx/131;
    integral_theta += SAMPLE_TIME/1000*theta;

    Serial.println(theta);
    sample_flag = false;
  }
  setSpeed(3, 4);
  
}
