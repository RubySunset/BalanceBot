//NOW WORKING TO DETECT WHEN BEACONS IN CENTRE
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

///-----SENSOR PINS | START-----///
#define trigPin_L 33
#define trigPin_F 32
#define trigPin_R 17                
#define echoPin_L 34
#define echoPin_F 36
#define echoPin_R 39 

// Ultrasonic sensor variables - duration is in us, distance is in cm.
float dur_F, dist_F, dur_L, dist_L, dur_R, dist_R;
///-----SENSOR PINS | END-----///

///-----CONTROL GLOBALS & FUNCTIONS | START----///
bool STABILITY_MODE = true;  //IF STABILITY MODE IS TRUE, STABILISING FRAME ATTACHED TO ROBOT

const int STRR = 23;
const int DIRR = 19;
const int STRL = 18;
const int DIRL = 5;

//const int SAMPLE_TIME = 1; //sampling time in ms

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
double sensed_speed = 0;
double positionX = 0;
double positionY = 0;

double lightL = 0;  //should these be volatile?
double lightR = 0;
double lightF2 = 0;

//Note that for multiple PID controllers, we use an array for sum_e and e_n_1
//index 0 is for the theta_dash controller, index 1 is for x', and index 2 is for alpha_dash
double sum_e[3] = { 0, 0, 0 }, e_n_1[3] = { 0, 0, 0 };  //sum of errors and previous error initialisation
double control_max = 10;                                //absolute maximum value of actuators
double speed_max = 5;
char movement_mode = 'm';  //current movement mode of robot; s = stationary, m = moving, t = turning
bool finished_turning = true;

//PID Output calculation
double simple_PID_calc(double Ts, double set_point, double sensor_point, int PID_ID, double Kp, double Ki, double Kd) {
  //variable finding
  double e = set_point - sensor_point;        //current error for proportional term [1]
  sum_e[PID_ID] += e;                         //cumulative error for sum term [2]
  double differential_e = e - e_n_1[PID_ID];  //delta error for differential term [3]

  //output setting
  double control_input = Kp * e + Ki * Ts * sum_e[PID_ID] + Kd * (1 / Ts) * differential_e;                 //control law
  if (abs(control_input) >= control_max) control_input = control_max * abs(control_input) / control_input;  //in case motor limit reached

  //incrementing
  e_n_1[PID_ID] = e;

  //return
  return control_input;
}

bool sample_flag = false;
bool sample_flag_2 = false;
hw_timer_t* sample_time = NULL;
hw_timer_t* sample_time_2 = NULL;
double sample_interval = 0.002;

void IRAM_ATTR doSample() {
  sample_flag = true;
}

void IRAM_ATTR doSample2() {
  sample_flag_2 = true;
}

double control_speed = 0;
double speed_left = 0;
double speed_right = 0;
int cycle_number = 0;  //temp variable

//These parameters (apart from theta_setpoint) will be controlled by commands received from higher levels
double theta_setpoint = 0;
double velocity_setpoint = 0;
double angle_setpoint = 0;  //decide that clockwise is positive direction; so ranges from 0 to 360 degrees

void setSpeed(int r, int l) {
  s1.setSpeed(r * 100 / PI);  //r*100/PI is steps per second
  s2.setSpeed(-l * 100 / PI);
  s1.runSpeed();
  s2.runSpeed();
}

double distance_per_step = 0;
//called in setup; units are metres
double find_distance_per_step() {
  double wheel_radius = 0.033;  //wheel radius is 33mm
  double wheel_circumference = TWO_PI * wheel_radius;
  // This is unnecessary to name but I'm just doing it for clarity
  double steps_per_revolution = 200;
  // As one revolution is done, it will go distance of 1 circumference, divided by steps per revolution would give distance per step
  return wheel_circumference / steps_per_revolution;
}

double distance_moved(double wheel_speed) {
  // if speed is in m/s, conversion to steps:
  //double step_speed = wheel_speed / distance_per_step

  //converting wheel_speed to steps per second
  wheel_speed = wheel_speed * 800 / PI;

  // Unsure on last step, speed if in steps per second * distance per step will get distance travelled in 1 second??
  // Might work better if wheel_speed is actually steps for steps * distance per steps but I think this works
  //double distance = wheel_speed * distance_per_step;
  // if needed:    steps/second *   metres/step     *    seconds         = metres
  double distance = wheel_speed * distance_per_step * sample_interval;
  return distance;
}

//Complementary filter for alpha and theta
const double COMPL_ALPHA = 0.98;
const int filter_adjustment_interval = 500;
double compl_alpha = 0;
double calibrateMPU() {
  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    compl_alpha = 0.999 * (compl_alpha + gy * 0.002 / 131) + 0.001 * atan2(ax, az) * RAD_TO_DEG;
    delay(2);
  }
  return compl_alpha;
}
double compl_theta = 0;

char sendingMode = 's';  //s for sensors, b for beacons, n for nothing (defaults to nothing)
char scanningMode = 'n';
bool cc_active = false;
//carries out appropriate actions once turning done
//figure out what- send message to node? retrieve instructions from some stack?
void check_turning_finished(double alpha_error) {
  if (alpha_error < 0.7) {
    alpha = normaliseAngle(alpha);
    //angle_setpoint = alpha;
    finished_turning = true;
    movement_mode = 'm';
    velocity_setpoint = 0;
    if (scanningMode == 'd') {  //this is case when just finished scan
      sendingMode = 'b';        //send beacon angles
    }
    if (cc_active) {
      cc_active = false;
    }
    //reset alpha variables
    sum_e[2] = 0;
    e_n_1[2] = 0;
  }
}

int ultrasound_sequence = 0;
unsigned long prevTime = 0;
bool writingToLight = false;
void sense_light_levels() {
  if (ultrasound_sequence == 0) {
    digitalWrite(trigPin_F, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin_F, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_F, LOW);
    dur_F = pulseIn(echoPin_F, HIGH);
    dist_F = 0.017 * dur_F;
    ultrasound_sequence = 1;
  }

  if (ultrasound_sequence == 1) {
    digitalWrite(trigPin_L, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin_L, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_L, LOW);
    dur_L = pulseIn(echoPin_L, HIGH);
    dist_L = 0.017 * dur_L;
    ultrasound_sequence = 2;
  }

  if (ultrasound_sequence == 2) {
    digitalWrite(trigPin_R, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin_R, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_R, LOW);
    dur_R = pulseIn(echoPin_R, HIGH);
    dist_R = 0.017 * dur_R;
    ultrasound_sequence = 0;
  }

  lightF2 = dist_F;
  lightL = dist_L;
  lightR = dist_R;
}

//prints all values, for debugging
char printWhat = 's';  //n for nothing, l for location, s for sensors
void dataPrinter() {
  if (printWhat == 'l') {
    //Serial.println("theta : " + String(theta) + " | compl_theta : " + String(compl_theta) + " | alpha : " + String(alpha) + " | positionX : " + String(positionX) + " | positionY : " + String(positionY));
  } else if (printWhat == 's') {
    Serial.println("L = " + String(lightL) + " | R = " + String(lightR) + " | F2 = " + String(lightF2));
  }
}

//Junction scanning
//char scanningMode = 'n'; //n for not scanning | b for seeking first beacon | s for scanning | d for scan done //declared earlier, with sendingMode
//n as default; b set when startBeaconScan(); s set after first beacon detected in checkForBeacon(); d set in check_turning_finished
double redBeaconAngle = -1;
double yellowBeaconAngle = -1;
double blueBeaconAngle = -1;
double scan_array_angles[72];
int scan_array_left_light[72];
int scan_array_right_light[72];
double lastScanAngle = 0;
int angle_array_index = 0;
double normaliseAngle(double givenAngle) {
  bool angleUnnormalised = true;
  while (angleUnnormalised) {
    if ((0 <= givenAngle) && (givenAngle < 360)) {
      angleUnnormalised = false;
    } else if (givenAngle < 0) {
      givenAngle += 360;
    } else if (givenAngle >= 360) {
      givenAngle -= 360;
    }
  }
  return givenAngle;
}
void startBeaconScan() {
  sendingMode = 'n';
  scanningMode = 'b';
  angle_setpoint = alpha + 359;  //two spins just in case
}
//format RBY xmin_r xmax_r xmin_y xmax_y xmin_b xmax_b
double boundingBoxArray[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
void retrieveBoundingBoxes() {
  static const int BUFFER_SIZE = 28;  // 4 for RBY and 16 for the rest
  char buf[BUFFER_SIZE];
  //format: xmin_r xmax_r xmax_b xmin_b xmin_y xmax_y
  int beacon_array[6];

  if (Serial.available() >= BUFFER_SIZE) {
    // read incoming bytes

    int rlen = Serial.readBytes(buf, BUFFER_SIZE);

    // for (int i=0 ; i<rlen ; i++) {
    //   Serial.print(buf[i], DEC);
    //   Serial.print(" ");
    // };

    // ignore RBY bytes (4)
    int offset = 4;

    // Insert into array
    for (int col = 1; col < 4; col++) {
      unsigned int xmin = (int)buf[col * offset + 2] + (int)(buf[col * offset + 3] << 8);
      unsigned int xmax = (int)buf[col * offset + 6] + (int)(buf[col * offset + 7] << 8);

      beacon_array[2 * col - 2] = xmin;
      beacon_array[2 * col - 1] = xmax;
    }
  }
  //format: xmin_r[0] xmax_r[1] xmax_b[2] xmin_b[3] xmin_y[4] xmax_y[5] -> r_present y_present b_present xmin_r xmax_r xmin_y xmax_y xmin_b xmax_b
  boundingBoxArray[3] = beacon_array[0];                                                 //xmin_r
  boundingBoxArray[4] = beacon_array[1];                                                 //xmax_r
  boundingBoxArray[5] = beacon_array[4];                                                 //xmin_y
  boundingBoxArray[6] = beacon_array[5];                                                 //xmax_y
  boundingBoxArray[7] = beacon_array[3];                                                 //xmin_b
  boundingBoxArray[8] = beacon_array[2];                                                 //xmax_b
  boundingBoxArray[0] = (abs(boundingBoxArray[3] - boundingBoxArray[4]) > 300) ? 1 : 0;  //r_present
  boundingBoxArray[1] = (abs(boundingBoxArray[5] - boundingBoxArray[6]) > 300) ? 1 : 0;  //y_present
  boundingBoxArray[2] = (abs(boundingBoxArray[7] - boundingBoxArray[8]) > 300) ? 1 : 0;  //b_present
}
bool facingBeaconMiddle(double xmin, double xmax) {
  //Check if we're facing almost the middle of the bounding box
  double beaconCentre = (xmax + xmin) / 2;
  bool facingMiddle = false;
  double margin = 10;  //margin of pixels which define centre
  if ((beaconCentre > (320 - margin)) && (beaconCentre < (320 + margin))) {
    facingMiddle = true;
  }
  return facingMiddle;
}
void check_for_beacon() {
  //retrieve bounding box coordinates here, in format:
  //R_present B_present Y_present xmin_r xmax_r xmin_b xmax_b xmin_y xmax_y
  retrieveBoundingBoxes();  //saved to boundingBoxArray
  bool foundBeacon = false;
  if (boundingBoxArray[0] == 1) {  //RED BEACON PRESENT
    if (facingBeaconMiddle(boundingBoxArray[3], boundingBoxArray[4])) redBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  } else if (boundingBoxArray[1] == 1) {  //BLUE BEACON PRESENT
    if (facingBeaconMiddle(boundingBoxArray[5], boundingBoxArray[6])) blueBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  } else if (boundingBoxArray[2] == 1) {  //YELLOW BEACON PRESENT
    if (facingBeaconMiddle(boundingBoxArray[7], boundingBoxArray[8])) yellowBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  }
  if ((foundBeacon = true) && (scanningMode = 'b')) {  //case when just found first beacon
    alpha = normaliseAngle(alpha);
    lastScanAngle = alpha;
    angle_array_index = 0;
    angle_setpoint = alpha + 360;
    scanningMode = 's';
  }
  if (((abs(alpha - lastScanAngle)) > 5) && (scanningMode = 's')) {
    sense_light_levels();
    scan_array_angles[angle_array_index] = alpha;
    scan_array_left_light[angle_array_index] = lightL;
    scan_array_right_light[angle_array_index] = lightR;
    angle_array_index += 1;
    if (angle_array_index >= 72) angle_array_index -= 72;
    lastScanAngle = alpha;
  }
}
///-----CONTROL GLOBALS & FUNCTIONS | END-----///

bool useCommsFunctions = false;  //for debugging; turn wifi functionality on and off

///-----COMMUNICATIONS GLOBALS & FUNCTIONS | START-----///
const char* WIFI_SSID = "BalanceBot";
const char* WIFI_PASS = "Ajanthan";

const int BUFFER_SIZE = 28;
char buf[BUFFER_SIZE + 1];
char hexBuf[BUFFER_SIZE * 2 + 1];
char LDR_chars[2 + 1];

TaskHandle_t Task1;
TaskHandle_t Task2;

WebSocketsClient webSocket;

// forward declarations
void handleMovement(const char* message);
void handleScan(const char* message);
void handleReceivedText(char* payload);
void Task1Code(void* pvParameters);
void Task2Code(void* pvParameters);

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
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

void byteToHex(unsigned char byte, char* hex) {
  const char* hexChars = "0123456789ABCDEF";
  hex[0] = hexChars[(byte >> 4) & 0x0F];
  hex[1] = hexChars[byte & 0x0F];
}
///-----COMMUNICATIONS GLOBALS & FUNCTIONS | END-----///

///-----MAIN BODY | START-----///
void setup() {
  //-----CONTROL SETUP | START-----//
  Serial.begin(9600);

  // configure the trigger pin to output mode
  pinMode(trigPin_F, OUTPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(trigPin_R, OUTPUT);
  // configure the echo pins to input mode
  pinMode(echoPin_F, INPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(echoPin_R, INPUT);

  Wire.begin();
  mpu.initialize();
  //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setXAccelOffset(-599);
  mpu.setYAccelOffset(-1235);
  mpu.setZAccelOffset(1196);
  mpu.setXGyroOffset(94);
  mpu.setYGyroOffset(-46);
  mpu.setZGyroOffset(22);

  //alpha = calibrateMPU();

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
  timerAlarmWrite(sample_time_2, 50000, true);  //sampling time in microseconds
  timerAlarmEnable(sample_time_2);

  distance_per_step = find_distance_per_step();
  //-----CONTROL SETUP | END-----//

  //-----COMMUNICATIONS SETUP | START-----//
  if (useCommsFunctions == true) {
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

    //try every 5000 again if connection has failed
    //idk why but its duplicating connections
    //webSocket.setReconnectInterval(5000);
  }
  //-----COMMUNICATIONS SETUP | END-----//

  //-----RTOS SETUP | START-----//
  //During debugging, can swap around ControlLoop and CommunicationsCode to change priority
  xTaskCreatePinnedToCore(ControlLoop, "Task1", 8000, NULL, 2, &Task1, 1);
  if (useCommsFunctions == true) {
    xTaskCreatePinnedToCore(CommunicationsLoop, "Task2", 8000, NULL, 1, &Task2, 0);
  }
  //-----RTOS SETUP | END-----//

  //xTaskCreatePinnedToCore(Task2Code, "Task2", 10000, NULL, tskIDLE_PRIORITY, &Task2, 0);
}

//null loop
void loop() {}

int k = 0;
///-----CONTROL LOOP | START-----///
void ControlLoop(void* pvParameters) {
  while (true) {
    if (sample_flag == true) {
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

      //alpha complementary filter
      int accy = ay / 1638;
      int accz = az / 1638;
      // compl_alpha = COMPL_ALPHA * (compl_alpha + alpha_dash * 0.002) - (1 - COMPL_ALPHA) * (atan2(accy, accz) + PI) * RAD_TO_DEG;
      // if (k > filter_adjustment_interval) {
      //   alpha = compl_alpha;
      //   k = 0;
      // }
      //compl_alpha = 0.998 * (compl_theta - gx * 0.002 / 131) + 0.002 * atan(-ay / sqrt(pow(ax, 2.0) + pow(az, 2.0))) * RAD_TO_DEG;
      //tune the ALPHA value 0.995+, higher value means less vibrations, but also would take longer initially for it to find its balance
      compl_theta = 0.998 * (compl_theta - gx * 0.002 / 131) + 0.002 * atan(-ay / sqrt(pow(ax, 2.0) + pow(az, 2.0))) * RAD_TO_DEG;

      //velocity and position in m/s and m. displacement is since last sample
      double displacement = distance_moved(control_speed);
      sensed_speed = displacement / sample_interval;
      positionX += displacement * sin(alpha * PI / 180);
      positionY += displacement * cos(alpha * PI / 180);

      //comment out when not debugging
      //dataPrinter();
      //Serial.println(alpha);

      //Course-correction takes priority, comment out line below to disable
      // double course_correction_setpoint = course_correct();
      // if (cc_active) {
      //   movement_mode = 't';
      //   angle_setpoint = course_correction_setpoint;
      // }

      //When stabilising wheels and frame attached
      if (STABILITY_MODE == true) {
        if (movement_mode == 'm') {
          //control_speed = simple_PID_calc(sample_interval, velocity_setpoint, sensed_speed, 1, 1, 0.5, 0.01); //values to be tuned
          control_speed = velocity_setpoint;
          speed_right = control_speed;
          speed_left = control_speed;
        } else if (movement_mode == 't') {
          finished_turning = false;
          double wheel_speed_difference = simple_PID_calc(sample_interval, angle_setpoint, alpha, 2, 0.008, 0, 0);
          if ((scanningMode == 'b') | (scanningMode == 's')) check_for_beacon();
          check_turning_finished(abs(alpha - angle_setpoint));
          speed_right -= wheel_speed_difference;
          speed_left += wheel_speed_difference;
          control_speed = 0;
        } else {
          speed_right = 0;
          speed_left = 0;
        }
      }
      //No stabilising wheels and frame
      else {
        //Movement control- outer loop changes theta_dash setpoint slightly
        if (abs(theta) > 0.5) {  //but only when theta_dash is decently small
          velocity_setpoint = 0;
        }
        if (movement_mode == 'm') {
          theta_setpoint = simple_PID_calc(sample_interval, velocity_setpoint, sensed_speed, 1, 1, 0, 0);
        }

        //theta_dash control- always necessary, present in both of other modes too
        //tune the compl_theta-16 value between 16.5-15.5, depending on the setup, but I think cascaded outer loop would also solve the slight drift; more I value makes the system less responsive so this might be better lower
        control_speed = simple_PID_calc(sample_interval, theta_setpoint, compl_theta - 16, 0, 2.1, 0.04, 0.01);
        speed_right = control_speed;
        speed_left = control_speed;

        //Turning control- slightly modify left and right wheel speeds, but only when theta_dash is decently small
        if ((movement_mode == 't') && (abs(theta) < 0.5)) {
          finished_turning = false;
          double wheel_speed_difference = simple_PID_calc(sample_interval, angle_setpoint, alpha, 2, 0.01, 0, 0);
          check_turning_finished(abs(alpha - angle_setpoint));
          speed_right -= wheel_speed_difference;
          speed_left += wheel_speed_difference;
          control_speed = 0;
        }
      }
      sample_flag = false;
    }
    //Set speed and increment loop number
    //cycle_number += 1;
    setSpeed(speed_right * 8, speed_left * 8);
  }
}
///-----CONTROL LOOP | END-----///

///-----COMMUNICATIONS LOOP | START-----///
const unsigned long SEND_INTERVAL_MS = 200;
static unsigned long lastSendMillis = 1;
void CommunicationsLoop(void* pvParameters) {
  while (true) {
    vTaskDelay(100);
    //Serial.println("in task 2");
    webSocket.loop();

    if (sample_flag_2 == true) {
      if(scanningMode == 'n') sense_light_levels(); //only sense levels when not scanning; scanning mode has scan every 5 degrees
      sample_flag_2 = false;
    }
    // if (Serial.available() >= BUFFER_SIZE) {
    //   // read incoming bytes
    //   int rlen = Serial.readBytes(buf, BUFFER_SIZE);

    //   // convert buf to hexadecimal string
    //   for (int i = 0; i < rlen; i++) {
    //     byteToHex(buf[i], &hexBuf[i * 2]);
    //   }
    //   hexBuf[rlen * 2] = '\0';  // null-terminate the hex string

    // send information to server
    if (millis() - lastSendMillis >= SEND_INTERVAL_MS) {
      lastSendMillis = millis();

      // JSON doc fixed memory allocation on stack
      StaticJsonDocument<256> doc;
      doc["time"] = millis();

      if (sendingMode == 's') {  //not sending scan data, only sensors + position + alpha
        doc["type"] = "Sensors";

        // store LDR readings in JSON array
        JsonArray Sense_Array = doc.createNestedArray("Sense_Array");
        //POS- Dead reckoning position- X, Y
        Sense_Array.add(positionX);
        Sense_Array.add(positionY);
        //ANGLE- alpha
        Sense_Array.add(normaliseAngle(alpha));
        //LDR array- L R F1 F2 F3
        Sense_Array.add(lightL);
        Sense_Array.add(lightR);
        Sense_Array.add(lightF2);
      } else if (sendingMode == 'b') {
        doc["type"] = "Beacons";

        //Store beacon angles in JSON array
        JsonArray Beacon_Array = doc.createNestedArray("Beacon_Array");
        //Beacon angles
        Beacon_Array.add(redBeaconAngle);
        Beacon_Array.add(yellowBeaconAngle);
        Beacon_Array.add(blueBeaconAngle);

        JsonArray Sample_Angle_Array = doc.createNestedArray("Sample_Angle_Array");
        JsonArray Left_Scan_Array = doc.createNestedArray("Left_Scan_Array");
        JsonArray Right_Scan_Array = doc.createNestedArray("Right_Scan_Array");
        for (int i = 72; i = 0; i++) {
          Sample_Angle_Array.add(scan_array_angles[i]);
          Left_Scan_Array.add(scan_array_left_light[i]);
          Right_Scan_Array.add(scan_array_right_light[i]);
        }
        sendingMode = 's';  //stop sending once beacon doc sent
      } else if (sendingMode == 'n') {
        //send nothing but record LDR and angle arrays
      }

      char message[256];
      serializeJson(doc, message, sizeof(message));
      // Serial.println(message);
      webSocket.sendTXT(message);
    }
  }
}
///-----COMMUNICATIONS LOOP | END-----///

///-----COMMUNICATIONS FUNCTIONS | START-----///
void handleReceivedText(char* payload) {
  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, payload);

  if (error) {
    Serial.println(F("Failed to read JSON"));
  } else {

    // c style string more efficient apparently so why not
    const char* type = doc["type"];

    if (strcmp(type, "movement") == 0) {
      handleMovement(doc["command"], doc["value"]);
    } else if (strcmp(type, "scan") == 0) {
      handleScan(doc["message"]);
    }
  }
}

void handleMovement(const char* message, double value) {
  if (strcmp(message, "turn") == 0) {
    movement_mode = 't';
    angle_setpoint = normaliseAngle(alpha + value);
  }
  if (strcmp(message, "stabiliser") == 0) {
    STABILITY_MODE = !STABILITY_MODE;
  }
  if (strcmp(message, "move") == 0) {
    movement_mode = 'm';
    velocity_setpoint = -10;
  }
  if (strcmp(message, "stop") == 0) {
    movement_mode = 'm';
    velocity_setpoint = 0;
  }
  if (strcmp(message, "sense") == 0) {
    sense_light_levels();
  }
  if (strcmp(message, "scan") == 0) {
    startBeaconScan();
  }
  if (strcmp(message, "read data") == 0) {
    //Send back position, light sensors, angle
    // Serial.print("x = ");  //position
    // Serial.print(positionX);
    // Serial.print("y = ");
    // Serial.println(positionY);
    // Serial.println(lightL);  //left LDR
    // Serial.println(lightR);  //right LDR
    // Serial.println(alpha);   //angle
  }
  if (strcmp(message, "print values") == 0) {
    // Serial.println("Stability mode value");
    // Serial.println(STABILITY_MODE);
    // Serial.println("Velocity setpoint");
    // Serial.println(velocity_setpoint);
    // Serial.println("theta value");
    // Serial.println(theta);
  }
  if (strcmp(message, "print setpoints") == 0) {
    // Serial.println(velocity_setpoint);
    // Serial.println(angle_setpoint);
  }
  if (strcmp(message, "printWhat n") == 0) {  //nothing
    printWhat = 'n';
  }
  if (strcmp(message, "printWhat l") == 0) {  //location
    printWhat = 'l';
  }
  if (strcmp(message, "printWhat s") == 0) {  //sensors
    printWhat = 's';
  }
  if (strcmp(message, "beacon angles received") == 0) {
    sendingMode = 's';
  }
  if (strcmp(message, "change CC_light_threshold") == 0) {
    //CC_light_threshold = value;
  }
  //Serial.println(message);
}

//NB: NEED TO STORE VERTEX OF SCAN FOR COURSE CORRECTION
void handleScan(const char* message) {
  // handle scan command here
  //Serial.printf("Handling scan command: %s\n", message);

  //Record current vertex coordinate; using dead reckoning currently
  // cc_prev_vertex_x = positionX;
  // cc_prev_vertex_y = positionY;

  //check whether can see beacons
  startBeaconScan();
  //store current angle
  //turn to centre beacons
  //stop turning when beacon in frame (angle_setpoint = alpha)
  //send current angle
}
///-----COMMUNICATIONS FUNCTIONS | END-----///