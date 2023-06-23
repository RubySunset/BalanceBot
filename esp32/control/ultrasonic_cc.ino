#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 40 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(33, 34, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(32, 36, MAX_DISTANCE),
  NewPing(17, 39, MAX_DISTANCE)
};

const int STRR = 23;
const int DIRR = 19;
const int STRL = 18;
const int DIRL = 5;
AccelStepper s1(AccelStepper::DRIVER, STRR, DIRR);  //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, STRL, DIRL);  //STRL, DIRL

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
double theta_dash;
double theta = 0;
double alpha_dash;
double alpha = 0;
double integral_az = 0;
float sensed_speed = 0;
float positionX = 0;
float positionY = 0;
float displacement = 0;

double speed_right, speed_left;

unsigned long prevTime = 0;

bool sample_flag = false;
bool sample_flag_2 = false;
hw_timer_t* sample_time = NULL;
hw_timer_t* sample_time_2 = NULL;

void IRAM_ATTR doSample() {
  sample_flag = true;
}

void IRAM_ATTR doSample2() {
  sample_flag_2 = true;
}

double theta_setpoint = 0;
double velocity_setpoint = 0;
double angle_setpoint = 0;

 char movement_mode = 'm'; 

void setSpeed(int r, int l) {
  s1.setSpeed(r * 100 / PI);  //r*100/PI is steps per second
  s2.setSpeed(-l * 100 / PI);
  s1.runSpeed();
  s2.runSpeed();
}

// Ultrasonic sensor variables - duration is in us, distance is in cm.
float dur_F, dist_F, dur_L, dist_L, dur_R, dist_R;

// For course-correction.
float prev_dist_L = 10000, prev_dist_R = 10000;
bool turning_done = false;
int ultrasound_sequence = 0;
void ultrasoundSample(){
  if(ultrasound_sequence == 0){
    dist_L = sonar[0].ping_cm();
    ultrasound_sequence = 1;
  }
  if(ultrasound_sequence == 1){
    dist_F = sonar[1].ping_cm();
    ultrasound_sequence = 2;
  }
  if(ultrasound_sequence == 2){
    dist_R = sonar[2].ping_cm();
    ultrasound_sequence = 0;
  }
}

///-----COMMUNICATIONS GLOBALS & FUNCTIONS | START-----///
const char* WIFI_SSID = "BalanceBot";
const char* WIFI_PASS = "Ajanthan";

const int BUFFER_SIZE = 28;
char buf[BUFFER_SIZE + 1];
char hexBuf[BUFFER_SIZE * 2 + 1];
char LDR_chars[2 + 1];

TaskHandle_t Task1;
TaskHandle_t Task2;



void setup(){
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setXAccelOffset(-599);
  mpu.setYAccelOffset(-1235);
  mpu.setZAccelOffset(1196);
  mpu.setXGyroOffset(94);
  mpu.setYGyroOffset(-46);
  mpu.setZGyroOffset(22);

  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
  s1.setMaxSpeed(700);
  s2.setMaxSpeed(700);

  sample_time = timerBegin(0, 80, true);
  timerAttachInterrupt(sample_time, &doSample, true);
  timerAlarmWrite(sample_time, 2000, true);  //sampling time in microseconds
  timerAlarmEnable(sample_time);

  sample_time_2 = timerBegin(1, 80, true);
  timerAttachInterrupt(sample_time_2, &doSample2, true);
  timerAlarmWrite(sample_time_2, 30000, true);  //sampling time in microseconds
  timerAlarmEnable(sample_time_2);

  xTaskCreatePinnedToCore(ControlLoop, "Task1", 8000, NULL, 2, &Task1, 1);
  xTaskCreatePinnedToCore(CommunicationsLoop, "Task2", 8000, NULL, 1, &Task2, 0);
 
}

double control_speed = 2;
void loop() {}

void ControlLoop(void* pvParameters) {
  while (true) {
    velocity_setpoint = -3;
//    angle_setpoint = 0;
    if(sample_flag == true){
      unsigned long currentTime = millis();
      float elapsed_time = currentTime - prevTime;
      prevTime = currentTime;

      //Sensor data
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //angle speeds in degrees/s
      theta_dash = -gx / 131;
      alpha_dash = -gz / 131;
      float elapsed_time_seconds = elapsed_time / 1000;
      theta += elapsed_time_seconds * theta_dash;
      alpha += elapsed_time_seconds * alpha_dash;


      
      sensed_speed = control_speed * 8 * 0.033;
      displacement =  sensed_speed * elapsed_time_seconds;
      positionX += displacement * sin(alpha * PI / 180);
      positionY += displacement * cos(alpha * PI / 180);

      if (movement_mode == 'm') {
        speed_right = velocity_setpoint;
        speed_left = velocity_setpoint;
        } 
      else if (movement_mode == 't') {
        double angle_diff = angle_setpoint - alpha;
        if(angle_diff > 1){
          speed_right = -0.5;
          speed_left = 0.5;
        }
        else if(angle_diff < -1){
          speed_right = 0.5;
          speed_left = -0.5;
        }
        else{
           movement_mode = 'm';
//          turning_done = true;
        }
        
        
      }
      else {
        speed_right = 0;
        speed_left = 0;
      }

      
      sample_flag = false;
    }

    setSpeed(speed_right * 8, speed_left * 8);
  }
}

///-----COMMUNICATIONS LOOP | START-----///


const unsigned long SEND_INTERVAL_MS = 200;
static unsigned long lastSendMillis = 1;
void CommunicationsLoop(void* pvParameters) {
  int ultrasound_sequence = 0;
  while (true) {
    if (sample_flag_2 == true) {
//       delay(20);

    if(ultrasound_sequence == 0){
      dist_L = sonar[0].ping_cm();
      Serial.println(dist_L);
      ultrasound_sequence = 1;
    }
    if(ultrasound_sequence == 1){
      dist_F = sonar[1].ping_cm();
      Serial.println(dist_F);
      ultrasound_sequence = 2;
    }
    if(ultrasound_sequence == 2){
      dist_R = sonar[2].ping_cm();
      Serial.println(dist_R);
      ultrasound_sequence = 0;
    }

      if (dist_F > 10 || dist_F == 0) {
        if (dist_L < 10 && dist_L != 0) {
            movement_mode = 't';
            angle_setpoint = alpha + 50/dist_L;
//            Serial.println("Turning right");
            turning_done = false;
        } 
        else if (dist_R < 10 && dist_R != 0) {
            movement_mode = 't';
            angle_setpoint = alpha - 50/dist_R;
//            Serial.println("Turning left");
            turning_done = false;
        }
      }
      else{
        if(dist_L > 30 || dist_L == 0){
          movement_mode = 't';
          angle_setpoint = alpha - 90;
          turning_done = false;
        }
        if(dist_R > 30 || dist_R == 0){
          movement_mode = 't';
          angle_setpoint = alpha + 90;
          turning_done = false;
        }
      } 
        
      

      sample_flag_2 = false;
    }
  }
}