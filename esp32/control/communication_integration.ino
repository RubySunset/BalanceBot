#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

///-----LIGHT SENSORS-----///
#define F1 27
#define F2 26
#define F3 25
#define B1 33
#define B2 14
#define B3 4
#define L 15
#define R 2
///-----LIGHT SENSORS-----///

///-----CONTROL----///
bool STABILITY_MODE = true; //IF STABILITY MODE IS ON, STABILISING FRAME ATTACHED TO ROBOT

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
double integral_alpha = 0;
double integral_az = 0;

//-----PID variables-----//
//Note that for multiple PID controllers, we use an array for sum_e and e_n_1
//index 0 is for the theta controller, index 1 is for x', and index 2 is for alpha
double sum_e[3] = {0,0,0}, e_n_1[3] = {0,0,0}; //sum of errors and previous error initialisation
double control_max = 10; //absolute maximum value of actuators
double speed_max = 5;
char movement_mode = 'm'; //current movement mode of robot; s = stationary, m = moving, t = turning

//PID Output calculation
double simple_PID_calc(double Ts, double set_point, double sensor_point, int PID_ID, double Kp, double Ki, double Kd){
  //variable finding
  double e = set_point - sensor_point; //current error for proportional term [1]
  sum_e[PID_ID] += e; //cumulative error for sum term [2]
  double differential_e = e - e_n_1[PID_ID]; //delta error for differential term [3]
  
  //output setting
  double control_input = Kp*e + Ki*Ts*sum_e[PID_ID] + Kd*(1/Ts)*differential_e; //control law
  if(abs(control_input) >= control_max) control_input = control_max * abs(control_input)/control_input; //in case motor limit reached
  
  //incrementing
  e_n_1[PID_ID] = e;
  
  //return
  return control_input;
}

bool sample_flag = false;
bool sample_flag_2 = false;
hw_timer_t *sample_time = NULL;
hw_timer_t *sample_time_2 = NULL;

void IRAM_ATTR doSample(){
  sample_flag = true;
}

void IRAM_ATTR doSample2(){
  sample_flag_2 = true;
}

void setSpeed(int r, int l){
 s1.setSpeed(r*100/PI);
 s2.setSpeed(-l*100/PI);
 s1.runSpeed();
 s2.runSpeed();
}
///-----CONTROL-----///

const char *WIFI_SSID = "Pixel 6";
const char *WIFI_PASS = "Ajanthan";

const int BUFFER_SIZE = 28;
char buf[BUFFER_SIZE + 1];
char hexBuf[BUFFER_SIZE * 2 + 1];

TaskHandle_t Task2;

WebSocketsClient webSocket;

// forward declarations
void handleMovement(const char* message);
void handleScan(const char* message);
void handleReceivedText(char* payload);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] Received text: %s\n", payload);
      handleReceivedText((char*)payload);
      break;
  }
}

void byteToHex(unsigned char byte, char *hex) {
  const char *hexChars = "0123456789ABCDEF";
  hex[0] = hexChars[(byte >> 4) & 0x0F];
  hex[1] = hexChars[byte & 0x0F];
}

void setup() {
  ///-----CONTROL-----//
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

  sample_time_2 = timerBegin(1, 80, true);
  timerAttachInterrupt(sample_time_2, &doSample2, true);
  timerAlarmWrite(sample_time_2, 20000, true); //sampling time in microseconds
  timerAlarmEnable(sample_time_2);
  ///-----CONTROL-----///

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // server address, port and URL
  webSocket.begin("192.168.87.115", 8000, "/ws/rover");

  // event handler
	webSocket.onEvent(webSocketEvent);

  //xTaskCreatePinnedToCore(Task1Code, "Task1", 10000, NULL, 0, &Task1,  1); ;
  xTaskCreatePinnedToCore(Task2Code, "Task2", 10000, NULL, 0, &Task2,  0); 
  // try every 5000 again if connection has failed
  // idk why but its duplicating connections
	// webSocket.setReconnectInterval(5000);
}

///-----CONTROL LOOP-----///
double speed_left = 0;
double speed_right = 0;
int cycle_number = 0; //temp variable

//These parameters (apart from theta_setpoint) will be controlled by commands received from higher levels
double theta_setpoint = 0;
double velocity_setpoint = 0;
double angle_setpoint = 0; //decide that clockwise is positive direction; so could be +20deg, -35deg, etc

void loop() {
  //Serial.println("In Task 1\n");

  if(sample_flag_2 == true){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    integral_az += 0.02*az/16384;
    //Serial.println(integral_az);
    sample_flag_2 = false;
  };

  if(sample_flag == true){
    //Write sensor sampling and control code here
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    theta = -gy/131; //angles in degrees/s
    alpha = gx/131;
    integral_theta += 0.002*theta;
    integral_alpha += 0.002*alpha;

    //-----CONTROL-----//
    
    //Movement control- outer loop changes theta setpoint slightly, but only when theta is decently small
    if(abs(integral_theta) > 0.5){
      velocity_setpoint = 0;
    }
    if(movement_mode == 'm'){
      //theta_setpoint = simple_PID_calc(0.02, velocity_setpoint, integral_az, 0.1, 1, 0, 0.01);
      //theta_setpoint = simple_PID_calc(0.02, velocity_setpoint, az, 1, 1, 0, 0); //this version is actually an acceleration setpoint
    }

    double control_speed;
    //Theta control- always necessary, present in both of other modes too
    if(STABILITY_MODE = false){ //balances if no stabilisers
      control_speed = simple_PID_calc(0.002, theta_setpoint, integral_theta, 0, 2.13, 0.053, 0.0102); //0, 2.13, 0.05, 0.01);
    }
    else{
      control_speed = velocity_setpoint;
    }
    //if(movement_mode == 'm') control_speed += 0.05;
    speed_right = control_speed;
    speed_left = control_speed;
//    Serial.println(speed_left);
//    Serial.println(integral_theta);
      
    //Turning control- slightly modify left and right wheel speeds, but only when theta is decently small
    if((movement_mode == 't') && (abs(integral_theta) < 0.5)){
      double wheel_speed_difference = simple_PID_calc(0.002, angle_setpoint, integral_alpha, 2, 0.01, 0, 0);
      speed_right -= wheel_speed_difference;
      speed_left += wheel_speed_difference; 
    }

    sample_flag = false;
  }

  setSpeed(speed_right*8, speed_left*8);
  cycle_number += 1;
}
///-----CONTROL LOOP-----///

void Task2Code(void* pvParameters) {
  while(true){
    //Serial.println("in task 2");
    webSocket.loop();
  }
  // if (Serial.available() >= BUFFER_SIZE) {
  //   // read incoming bytes
  //   int rlen = Serial.readBytes(buf, BUFFER_SIZE);

  //   // convert buf to hexadecimal string
  //   for (int i = 0; i < rlen; i++) {
  //     byteToHex(buf[i], &hexBuf[i * 2]);
  //   }
  //   hexBuf[rlen * 2] = '\0';  // null-terminate the hex string

  //   // send beacon every half second
  //   static unsigned long lastSendMillis = 0;
  //   if (millis() - lastSendMillis > 500) { 
  //     lastSendMillis = millis();

  //     // JSON doc fixed memory allocation on stack
  //     StaticJsonDocument<256> doc;
  //     doc["type"] = "Beacons";
  //     doc["time"] = millis();
      
  //     // store the hexadecimal string in the JSON document
  //     doc["content"] = hexBuf;

  //     char message[256];
  //     serializeJson(doc, message, sizeof(message));
  //     Serial.println(message);
  //     webSocket.sendTXT(message);
  //   }
  //}
}

void handleReceivedText(char* payload) {
  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, payload);

  if (error) {
    Serial.println(F("Failed to read JSON"));
  }
  else {

    // c style string more efficient apparently so why not
    const char* type = doc["type"];

    if (strcmp(type, "movement") == 0) {
      handleMovement(doc["command"], doc["value"]);
    } 
    else if (strcmp(type, "scan") == 0) {
      handleScan(doc["message"]);
    }
  }
}

void handleMovement(const char* message, double value) {
  // handle movement command here
  //Serial.printf("Handling movement command: %s\n", message);
  if(strcmp(message, "turn") == 0){
    movement_mode = 't';
    angle_setpoint = value;
  }
  if(strcmp(message, "stabiliser") == 0){
    STABILITY_MODE = !STABILITY_MODE;
  }
  if(strcmp(message, "move") == 0){
    movement_mode = 'm';
    velocity_setpoint = 1;
  }
  if(strcmp(message, "stop") == 0){
    movement_mode = 'm';
    velocity_setpoint = 0;
  }
  if(strcmp(message, "sense") == 0){
    Serial.println(" L: ");
    Serial.println(analogRead(L));
    Serial.println(" R: ");
    Serial.println(analogRead(R));
  }
  if(strcmp(message, "read angle") == 0){
    Serial.println("Currently facing:");
    Serial.println(integral_alpha);
  }
  if(strcmp(message, "read data") == 0){
    //Send back position, light sensors, angle
    Serial.println("Position not yet available"); //position
    Serial.println(L); //left LDR
    Serial.println(R); //right LDR
    Serial.println(integral_alpha); //angle
  }
  Serial.println(message);
}

void handleScan(const char* message) {
  // handle scan command here
  Serial.printf("Handling scan command: %s\n", message);
}