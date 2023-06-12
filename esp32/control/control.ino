#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>
const int STRR = 27;
const int DIRR = 26;
const int STRL = 25;
const int DIRL = 33;

//const int SAMPLE_TIME = 1; //sampling time in ms

AccelStepper s1(AccelStepper::DRIVER, STRR, DIRR); //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, STRL, DIRL); //STRL, DIRL

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

double theta;
double integral_theta = 0;
double alpha;

//PID variables
double sum_e = 0, e_n_1 = 0; //sum of errors and previous error initialisation
double control_max = 10; //absolute maximum value of actuators
double speed_max = 5;

//PID Output calculation
double simple_PID_calc(double Ts, double set_point, double sensor_point, double Kp, double Ki, double Kd){
  //variable finding
  double e = set_point - sensor_point; //current error for proportional term [1]
  sum_e += e; //cumulative error for sum term [2]
  double differential_e = e - e_n_1; //delta error for differential term [3]
  
  //output setting
  double control_input = Kp*e + Ki*Ts*sum_e + Kd*(1/Ts)*differential_e; //control law
  if(abs(control_input) >= control_max) control_input = control_max * abs(control_input)/control_input; //in case motor limit reached
  
  //incrementing
  e_n_1 = e;
  
  //return
  return control_input;
}

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
  timerAlarmWrite(sample_time, 2000, true); //sampling time in microseconds
  timerAlarmEnable(sample_time);

}
double speed_left = 0;
  double speed_right = 0;
  double speed_delta = 0;
void loop() {
  
  
  if(sample_flag == true){
    //Write sensor sampling and control code here
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    theta = -gy/131; //angles in degrees/s
    alpha = gx/131;
    integral_theta += 0.002*theta;

    //Control Code
    double control_speed = simple_PID_calc(0.002, 0, integral_theta, 2.1, 0.05, 0.01);//relevant variables go here
    //speed_delta = control_speed*0.06;
    //if((speed_right += speed_delta) < speed_max){
      //speed_right += speed_delta;
      //speed_left += speed_delta;
    //}
    speed_right = control_speed;
    speed_left = control_speed;
    Serial.print(speed_left);
    Serial.println(integral_theta);
    sample_flag = false;
  }
  setSpeed(speed_right*8, speed_left*8);
//  setSpeed(0, 0);
  
}
