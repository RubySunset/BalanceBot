#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

///-----LIGHT SENSOR PINS | START-----///
#define F1 32  //A4
#define F2 35  //A5
#define F3 34  //A3
// #define B1 33
// #define B2 14
// #define B3 4
#define L 33  //soldered connection
#define R 39  //soldered connection
///-----LIGHT SENSOR PINS | END-----///

///-----CONTROL GLOBALS & FUNCTIONS | START----///
bool STABILITY_MODE = false;  //IF STABILITY MODE IS TRUE, STABILISING FRAME ATTACHED TO ROBOT

/* CHANGED BC NOT ENOUGH PINS
const int STRR = 27;
const int DIRR = 26;
const int STRL = 25;
const int DIRL = 33;
*/
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

volatile int lightL = 1;  //should these be volatile?
int lightR = 1;
int lightF1 = 1;
int lightF2 = 1;
int lightF3 = 1;

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

//copied over from distance_for_control branch, first uploaded as .txt file there

// derivation = angular speed = change in angle(theta)/time
// 		arc length = radius * theta so theta = arc length/radius
//		angular speed = arc length/(radius * time)
//		velocity = distance(arc length)/time
//		so angular speed = velocity / radius

// NEW STUFF THIS MAY NOT WORK, I TRIED MY BEST - thanks will
// So Step Angle is given in datasheet as 1.8 degrees, this means that the step per revolution is 200 as 360/1.8
// This works because step angle is the angle at which the rotor moves when one input pulse is applied in degrees.
// One revolution (360 degrees) can only be completed by 200 steps with each step being 1.8 degrees.

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
    Serial.println("Finished turning");
    alpha = normaliseAngle(alpha);
    angle_setpoint = alpha;
    finished_turning = true;
    movement_mode = 'm';
    velocity_setpoint = 0;
    if (scanningMode == 'd') {  //this is case when just finished scan
      sendingMode = 'b';        //send beacon angles
    }
    if (cc_active) {
      cc_active = false;
      //velocity_setpoint = stored_velocity_setpoint;
    }
  }
}

bool writingToLight = false;
void sense_light_levels() {
  writingToLight = true;
  lightL = analogRead(L);
  lightR = analogRead(R);
  lightF1 = analogRead(F1);
  lightF2 = analogRead(F2);
  lightF3 = analogRead(F3);
  writingToLight = false;
  Serial.println("Task on core " + String(xPortGetCoreID()) + " read LDR values as: L = " + String(lightL) + " | R = " + String(lightR) + " | F1 = " + String(lightF1) + " | F2 = " + String(lightF2) + " | F3 = " + String(lightF3));
}

void analogSample() {
  const unsigned long MEASURE_INTERVAL = 100;
  static unsigned long lastMeasureMillis;

  if (millis() - lastMeasureMillis > MEASURE_INTERVAL) {
    lastMeasureMillis = millis();
    lightL = analogRead(L);
    lightR = analogRead(R);
    lightF1 = analogRead(F1);
    lightF2 = analogRead(F2);
    lightF3 = analogRead(F3);
  }
  //Serial.println("Task on core " + String(xPortGetCoreID()) + " read LDR values as: L = " + String(lightL) + " | R = " + String(lightR) + " | F1 = " + String(lightF1) + " | F2 = " + String(lightF2) + " | F3 = " + String(lightF3));
}

//prints all values, for debugging
char printWhat = 'l';  //n for nothing, l for location, s for sensors
void dataPrinter() {
  if (printWhat == 'l') {
    Serial.println("theta : " + String(theta) + " | compl_theta : " + String(compl_theta) + " | alpha : " + String(alpha) + " | positionX : " + String(positionX) + " | positionY : " + String(positionY));
  } else if (printWhat == 's') {
    Serial.println("L = " + String(lightL) + " | R = " + String(lightR) + " | F1 = " + String(lightF1) + " | F2 = " + String(lightF2) + " | F3 = " + String(lightF3));
  }
}

int detectionThreshold = 770;
bool avoiding_obstacle = false;
char previous_movement_mode = 'm';
//Decide whether to reroute; return direction of obstacle. This is a failsafe if the routing controller fails
char obstacleDetected() {
  if ((lightF1 > detectionThreshold) | (lightF2 > detectionThreshold) | (lightF3 > detectionThreshold)) {
    return 'f';
  }
  // else if((analogRead(B1) > detectionThreshold)|(analogRead(B2) > detectionThreshold)|(analogRead(B3) > detectionThreshold)){
  //     return 'b';
  // }
  else if (lightL > 470) {
    return 'l';
  } else if (lightR > 370) {
    return 'r';
  } else {
    return 'n';
  }
}

//-----Course correction controller-----//
double get_distance(double x1, double y1, double x2, double y2) {
  return (sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)));
}

//gains
double P_CC = 0.3;
double I_CC = 0.0001;
double D_CC = 0.5;
double MAX_OFFSET = 10;     //max course correction, degrees
double UNBOUND_DIST = 0.5;  //distance for which unbounded CC is appled
// Unbounded CC ignores the condition imposed by MAX_OFFSET. in testing, I found that the robot tended to move
// better if it was allowed to course correct itself by larger amounts shortly after leaving a vertex

//CC controller variables
//bool cc_active = false; //declared with check_turning_finished
double cc_sum = 0;
double cc_prev_balance = 0;
double cc_prev_left_dist = 0;
double cc_prev_right_dist = 0;
double cc_offset = 0;
double cc_prev_vertex_x = 0;
double cc_prev_vertex_y = 0;
double cc_prev_width = 0;
double cc_alpha = 0;
// Note that we need to keep track of the position of the previous vertex to determine whether we apply
// unbounded CC (the robot will need to store its position when receiving a junction scan command)

double CC_light_threshold = 450;
//Apply course correction. Returns the target angle, alpha
double course_correct(/*pos, alpha, LDR readings are already global variables*/) {
  double left_light = lightL;
  double right_light = lightR;
  //only apply course correction in a corridor
  if ((left_light > CC_light_threshold) && (right_light > CC_light_threshold)) {
    //use the inverse square law to estimate the distance to the walls
    //will require a lot of fine tuning to work
    double left_dist = 1 / sqrt(left_light);
    double right_dist = 1 / sqrt(right_light);

    double balance = left_dist - right_dist;  //measure of closeness to right wall compared to left wall
    double width = left_dist - right_dist;    //estimated width of corridor

    if (cc_active) {
      //Rates of change of distances to respective walls
      double left_diff = left_dist - cc_prev_left_dist;
      double right_diff = right_dist - cc_prev_right_dist;

      if ((abs(width - cc_prev_width) < 1.5) && (abs(abs(left_diff) - abs(right_diff)) < 1.5)) {
        //A link to the side is inferred if:
        //1. The width of the passage is increasing rapidly
        //2. One wall is receeding much more quickly than the other
        //This condition also needs tuning
        cc_sum += balance;
        double diff = balance - cc_prev_balance;
        double delta = P_CC * balance + I_CC * cc_sum + D_CC * diff;
        if (get_distance(cc_prev_vertex_x, cc_prev_vertex_y, positionX, positionY) <= UNBOUND_DIST) {
          //if we're close enough to the last vertex (aka likely junction), neglect to limit CC
          cc_alpha -= delta;
        } else {
          cc_offset -= delta;
          if ((abs(cc_offset) <= MAX_OFFSET)) {
            cc_alpha -= delta;  //if we're within our allowance, proceed as normal
          } else if ((abs(cc_offset) >= MAX_OFFSET + delta)) {
            cc_offset += delta;  //if we're completely out, revert the change to cc_offset
          } else {
            //we can reduce delta to match the offset limit
            if (delta > 0) {
              delta -= abs(cc_offset) - MAX_OFFSET;
            } else {
              delta += abs(cc_offset) - MAX_OFFSET;
            }
            cc_alpha -= delta;
          }
        }
      }
    }
    cc_active = true;
    cc_prev_balance = balance;
    cc_prev_left_dist = left_dist;
    cc_prev_right_dist = right_dist;
    cc_prev_width = width;
  } else {
    //Reset the variables once the controller is inative
    cc_active = false;
    cc_sum = 0;
    cc_offset = 0;
  }
  return cc_alpha;
}
// Course correction controller end

//Junction scanning
//char scanningMode = 'n'; //n for not scanning | b for seeking first beacon | s for scanning | d for scan done //declared earlier, with sendingMode
//n as default; b set when startBeaconScan(); s set after first beacon detected in checkForBeacon(); d set in check_turning_finished
double redBeaconAngle = -1;
double yellowBeaconAngle = -1;
double blueBeaconAngle = -1;
double scan_array_angles[72];
int scan_array_left_light [72];
int scan_array_right_light [72];
double lastScanAngle = 0;
int angle_array_index = 0;
double normaliseAngle(double givenAngle) {
  return givenAngle % 360;
}
void startBeaconScan() {
  sendingMode = 'n';
  scanningMode = 'b';
  angle_setpoint = alpha + 720;  //two spins just in case
}
//format RBY xmin_r xmax_r ymin_r ymax_ xmin xmax ymin ymax xmin xmax ymin ymax
double boundingBoxArray[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
void retrieveBoundingBoxes() {
  //AJ CODE HERE
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
  if(((abs(alpha-lastScanAngle)) > 5) && (scanningMode = 's')){
    scan_array_angles[angle_array_index] = alpha;
    scan_array_left_light[angle_array_index] = lightL;
    scan_array_right_light[angle_array_index] = lightR;
    angle_array_index += 1;
    if(angle_array_index >= 72) angle_array_index -= 72;
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
  Serial.begin(115200);

  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
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
  timerAlarmWrite(sample_time_2, 20000, true);  //sampling time in microseconds
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
    webSocket.begin("api.balancebot.site", 8000, "/ws/rover");

    // event handler
    webSocket.onEvent(webSocketEvent);

    //try every 5000 again if connection has failed
    //idk why but its duplicating connections
    webSocket.setReconnectInterval(5000);
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
      //Sensor data
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //angle speeds in degrees/s
      theta_dash = -gx / 131;
      alpha_dash = -gz / 131;
      theta += sample_interval * theta_dash;
      alpha += sample_interval * alpha_dash;

      //alpha complementary filter
      int accy = ay / 1638;
      int accz = az / 1638;
      compl_alpha = COMPL_ALPHA * (compl_alpha + alpha_dash * 0.002) - (1 - COMPL_ALPHA) * (atan2(accy, accz) + PI) * RAD_TO_DEG;
      if (k > filter_adjustment_interval) {
        alpha = compl_alpha;
        k = 0;
      }
      //tune the ALPHA value 0.995+, higher value means less vibrations, but also would take longer initially for it to find its balance
      compl_theta = 0.998 * (compl_theta - gx * 0.002 / 131) + 0.002 * atan(-ay / sqrt(pow(ax, 2.0) + pow(az, 2.0))) * RAD_TO_DEG;

      //velocity and position in m/s and m. displacement is since last sample
      double displacement = distance_moved(control_speed);
      sensed_speed = displacement / sample_interval;
      positionX += displacement * sin(alpha * PI / 180);
      positionY += displacement * cos(alpha * PI / 180);

      //sense_light_levels();
      analogSample();

      //comment out when not debugging
      //dataPrinter();
      //Serial.println(alpha);

      //Course-correction takes priority, comment out line below to disable
      double course_correction_setpoint = course_correct();
      if (cc_active) {
        movement_mode = 't';
        angle_setpoint = course_correction_setpoint;
      }

      //LDR-BASED WALL AVOIDANCE FAILSAFE
      //- Needs much more work, but should stop crashes
      //- Better to rely on wall detection controller EDIT: this is our only obstacle avoidance now. godspeed
      // char obstacle_direction = 'n';
      // obstacle_direction = 'n'; //obstacleDetected();
      // switch (obstacle_direction) {
      //   case 'f':
      //     velocity_setpoint = 0;
      //     if (!avoiding_obstacle) previous_movement_mode = movement_mode;
      //     movement_mode = 'm';
      //     avoiding_obstacle = true;
      //     break;
      //   case 'b':
      //     velocity_setpoint = 1;
      //     if (!avoiding_obstacle) previous_movement_mode = movement_mode;
      //     movement_mode = 'm';
      //     avoiding_obstacle = true;
      //     break;
      //   case 'l':
      //     if (!avoiding_obstacle | finished_turning) {
      //       previous_movement_mode = movement_mode;
      //       angle_setpoint += 10;
      //       movement_mode = 't';
      //       avoiding_obstacle = true;
      //     }
      //     break;
      //   case 'r':
      //     if (!avoiding_obstacle | finished_turning) {
      //       previous_movement_mode = movement_mode;
      //       angle_setpoint -= 10;
      //       movement_mode = 't';
      //       avoiding_obstacle = true;
      //     }
      //     break;
      //   case 'n':
      //     avoiding_obstacle = false;
      //     movement_mode = previous_movement_mode;
      //     break;
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
        Sense_Array.add(lightF1);
        Sense_Array.add(lightF2);
        Sense_Array.add(lightF3);
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
        for(int i = 72; i = 0; i++){
          Sample_Angle_Array.add(scan_array_angles[i]);
          Left_Scan_Array.add(scan_array_left_light[i]);
          Right_Scan_Array.add(scan_array_right_light[i]);
        }
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
    angle_setpoint = normaliseAngle(angle_setpoint + value);
  }
  if (strcmp(message, "stabiliser") == 0) {
    STABILITY_MODE = !STABILITY_MODE;
  }
  if (strcmp(message, "move") == 0) {
    movement_mode = 'm';
    velocity_setpoint = -3;
  }
  if (strcmp(message, "stop") == 0) {
    movement_mode = 'm';
    velocity_setpoint = 0;
  }
  if (strcmp(message, "sense") == 0) {
    sense_light_levels();
  }
  if (strcmp(message, "read data") == 0) {
    //Send back position, light sensors, angle
    Serial.print("x = ");  //position
    Serial.print(positionX);
    Serial.print("y = ");
    Serial.println(positionY);
    Serial.println(lightL);  //left LDR
    Serial.println(lightR);  //right LDR
    Serial.println(alpha);   //angle
  }
  if (strcmp(message, "print values") == 0) {
    Serial.println("Stability mode value");
    Serial.println(STABILITY_MODE);
    Serial.println("Velocity setpoint");
    Serial.println(velocity_setpoint);
    Serial.println("theta value");
    Serial.println(theta);
  }
  if (strcmp(message, "print setpoints") == 0) {
    Serial.println(velocity_setpoint);
    Serial.println(angle_setpoint);
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
    CC_light_threshold = value;
  }
  //Serial.println(message);
}

//NB: NEED TO STORE VERTEX OF SCAN FOR COURSE CORRECTION
void handleScan(const char* message) {
  // handle scan command here
  Serial.printf("Handling scan command: %s\n", message);

  //Record current vertex coordinate; using dead reckoning currently
  cc_prev_vertex_x = positionX;
  cc_prev_vertex_y = positionY;

  //check whether can see beacons
  startBeaconScan();
  //store current angle
  //turn to centre beacons
  //stop turning when beacon in frame (angle_setpoint = alpha)
  //send current angle
}
///-----COMMUNICATIONS FUNCTIONS | END-----///