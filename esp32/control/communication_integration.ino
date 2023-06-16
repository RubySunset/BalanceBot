#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

///-----LIGHT SENSOR PINS | START-----///
#define F1 27
#define F2 26
#define F3 25
// #define B1 33
// #define B2 14
// #define B3 4
#define L 15
#define R 2
///-----LIGHT SENSOR PINS | END-----///

///-----CONTROL GLOBALS & FUNCTIONS | START----///
bool STABILITY_MODE = true; //IF STABILITY MODE IS TRUE, STABILISING FRAME ATTACHED TO ROBOT

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

AccelStepper s1(AccelStepper::DRIVER, STRR, DIRR); //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, STRL, DIRL); //STRL, DIRL

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

//Note that for multiple PID controllers, we use an array for sum_e and e_n_1
//index 0 is for the theta_dash controller, index 1 is for x', and index 2 is for alpha_dash
double sum_e[3] = {0,0,0}, e_n_1[3] = {0,0,0}; //sum of errors and previous error initialisation
double control_max = 10; //absolute maximum value of actuators
double speed_max = 5;
char movement_mode = 'm'; //current movement mode of robot; s = stationary, m = moving, t = turning
bool finished_turning = true;

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
double sample_interval = 0.002;

void IRAM_ATTR doSample(){
  sample_flag = true;
}

void IRAM_ATTR doSample2(){
  sample_flag_2 = true;
}

double control_speed = 0;
double speed_left = 0;
double speed_right = 0;
int cycle_number = 0; //temp variable

//These parameters (apart from theta_setpoint) will be controlled by commands received from higher levels
double theta_setpoint = 0;
double velocity_setpoint = 0;
double angle_setpoint = 0; //decide that clockwise is positive direction; so could be +20deg, -35deg, etc

void setSpeed(int r, int l){
 s1.setSpeed(r*100/PI); //r*100/PI is steps per second
 s2.setSpeed(-l*100/PI);
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
double find_distance_per_step(){
  double wheel_radius = 0.033; //wheel radius is 33mm
	double wheel_circumference = TWO_PI * wheel_radius;
	// This is unnecessary to name but I'm just doing it for clarity
	double steps_per_revolution = 200;
	// As one revolution is done, it will go distance of 1 circumference, divided by steps per revolution would give distance per step
	return wheel_circumference / steps_per_revolution;
}

double distance_moved(double wheel_speed){
	// if speed is in m/s, conversion to steps:
    //double step_speed = wheel_speed / distance_per_step

    //converting wheel_speed to steps per second
    wheel_speed = wheel_speed*800/PI;

	// Unsure on last step, speed if in steps per second * distance per step will get distance travelled in 1 second??
	// Might work better if wheel_speed is actually steps for steps * distance per steps but I think this works
	//double distance = wheel_speed * distance_per_step;
	// if needed:    steps/second *   metres/step     *    seconds         = metres
    double distance = wheel_speed * distance_per_step * sample_interval;
	return distance;
}

//carries out appropriate actions once turning done
//figure out what- send message to node? retrieve instructions from some stack?
void check_turning_finished(double alpha_error){
    if(alpha_error < 3){
        Serial.println('Finished turning');
        angle_setpoint = alpha;
        finished_turning = true;
        movement_mode = 'm';
        velocity_setpoint = 0;
    }
}

int detectionThreshold = 1100;
bool avoiding_obstacle = false;
//Decide whether to reroute; return direction of obstacle. This is a failsafe if the routing controller fails
char obstacleDetected(){
    if((analogRead(F1) > detectionThreshold)|(analogRead(F2) > detectionThreshold)|(analogRead(F3) > detectionThreshold)){
        return 'f';
    }
    else if((analogRead(B1) > detectionThreshold)|(analogRead(B2) > detectionThreshold)|(analogRead(B3) > detectionThreshold)){
        return 'b';
    }
    else if(analogRead(L) > detectionThreshold){
        return 'l';
    }
    else if(analogRead(R) > detectionThreshold){
        return 'r';
    }
    else{
        return 'n';
    }
}

//-----Course correction controller-----//
double get_distance(double x1, double y1, double x2, double y2){
    return(sqrt(pow((x2-x1),2) + pow((y2-y1),2)));
}

//gains
double P_CC = 0.3;
double I_CC = 0.0001;
double D_CC = 0.5;
double MAX_OFFSET = 10; //max course correction, degrees
double UNBOUND_DIST = 0.5; //distance for which unbounded CC is appled
// Unbounded CC ignores the condition imposed by MAX_OFFSET. in testing, I found that the robot tended to move
// better if it was allowed to course correct itself by larger amounts shortly after leaving a vertex

//CC controller variables
bool cc_active = false;
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

double CC_light_threshold = 300;
//Apply course correction. Returns the target angle, alpha
double course_correct(/*pos, alpha, LDR readings are already global variables*/){
    double left_light = analogRead(L);
    double right_light = analogRead(R);
    //only apply course correction in a corridor
    if((left_light > CC_light_threshold) && (right_light > CC_light_threshold)){
        //use the inverse square law to estimate the distance to the walls
        //will require a lot of fine tuning to work
        double left_dist = 1/sqrt(left_light);
        double right_dist = 1/sqrt(right_light);

        double balance = left_dist - right_dist; //measure of closeness to right wall compared to left wall
        double width = left_dist - right_dist; //estimated width of corridor

        if(cc_active){
            //Rates of change of distances to respective walls
            double left_diff = left_dist - cc_prev_left_dist;
            double right_diff = right_dist - cc_prev_right_dist;

            if((abs(width - cc_prev_width) < 1.5) && (abs(abs(left_diff) - abs(right_diff))< 1.5)){
                //A link to the side is inferred if:
                //1. The width of the passage is increasing rapidly
                //2. One wall is receeding much more quickly than the other
                //This condition also needs tuning
                cc_sum += balance;
                double diff = balance - cc_prev_balance;
                double delta = P_CC*balance + I_CC*cc_sum + D_CC*diff;
                if(get_distance(cc_prev_vertex_x, cc_prev_vertex_y, positionX, positionY) <= UNBOUND_DIST){
                    //if we're close enough to the last vertex (aka likely junction), neglect to limit CC
                    cc_alpha -= delta;
                }
                else{
                    cc_offset -= delta;
                    if((abs(cc_offset) <= MAX_OFFSET)){
                        cc_alpha -= delta; //if we're within our allowance, proceed as normal
                    }
                    else if((abs(cc_offset) >= MAX_OFFSET + delta)){
                        cc_offset += delta; //if we're completely out, revert the change to cc_offset
                    }
                    else{
                        //we can reduce delta to match the offset limit
                        if(delta > 0){
                            delta -= abs(cc_offset) - MAX_OFFSET;
                        }
                        else{
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
    }
    else{
        //Reset the variables once the controller is inative
        cc_active = false;
        cc_sum = 0;
        cc_offset = 0;
    }
    return cc_alpha;
}
///-----CONTROL GLOBALS & FUNCTIONS | END-----///

///-----COMMUNICATIONS GLOBALS & FUNCTIONS | START-----///
const char *WIFI_SSID = "BalanceBot";
const char *WIFI_PASS = "Ajanthan";

const int BUFFER_SIZE = 28;
char buf[BUFFER_SIZE + 1];
char hexBuf[BUFFER_SIZE * 2 + 1];
char LDR_chars[2+1];

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
///-----COMMUNICATIONS GLOBALS & FUNCTIONS | END-----///

///-----MAIN BODY | START-----///
void setup() {
  //-----CONTROL SETUP | START-----//
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

  distance_per_step = find_distance_per_step();;
  //-----CONTROL SETUP | END-----//

  //-----COMMUNICATIONS SETUP | START-----//
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Connect to wifi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // server address, port and URL
  webSocket.begin("13.43.40.216", 8000, "/ws/rover");

  // event handler
	webSocket.onEvent(webSocketEvent);


  //xTaskCreatePinnedToCore(Task1Code, "Task1", 10000, NULL, 0, &Task1,  1); ;
  xTaskCreatePinnedToCore(Task2Code, "Task2", 10000, NULL, 0, &Task2,  0); 
  // try every 5000 again if connection has failed
  // idk why but its duplicating connections
	// webSocket.setReconnectInterval(5000);

  //-----COMMUNICATIONS SETUP | END-----//
}

///-----CONTROL LOOP | START-----///
void loop() {
  if(sample_flag == true){
    //Sensor data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //angle speeds in degrees/s
    theta_dash = -gy/131; 
    alpha_dash = gx/131;
    theta += sample_interval*theta_dash;
    alpha += sample_interval*alpha_dash;
    //velocity and position in m/s and m. displacement is since last sample
    double displacement = distance_moved(control_speed);
    sensed_speed = displacement/sample_interval;
    positionX += displacement*sin(alpha*PI/180);
    positionY += displacement*cos(alpha*PI/180);

    //Course-correction takes priority
    //double course_correction_setpoint = course_correct();
    if(cc_active){
      movement_mode = 't';
      angle_setpoint = cc_alpha;
    }

    //LDR-BASED WALL AVOIDANCE FAILSAFE
    //- Needs much more work, but should stop crashes
    //- Better to rely on wall detection controller
    // char obstacle_direction = 'n'; /*obstacleDetected();*/
    // switch (obstacle_direction){
    //     case 'f':
    //         velocity_setpoint = -1;
    //         movement_mode = 'm';
    //         avoiding_obstacle = true;
    //         break;
    //     case 'b':
    //         velocity_setpoint = 1;
    //         movement_mode = 'm';
    //         avoiding_obstacle = true;
    //         break;
    //     case 'l':
    //         if(!avoiding_obstacle){
    //             angle_setpoint += 30;
    //             movement_mode = 't';
    //             avoiding_obstacle = true;
    //         }
    //         break;
    //     case 'r':
    //         if(!avoiding_obstacle){
    //             angle_setpoint -= 30;
    //             movement_mode = 't';
    //             avoiding_obstacle = true;
    //         }
    //         break;
    //     case 'n':
    //         avoiding_obstacle = false;
    //         break;
    }

    //When stabilising wheels and frame attached
    if(STABILITY_MODE == true){
        if(movement_mode == 'm'){
            //control_speed = simple_PID_calc(sample_interval, velocity_setpoint, sensed_speed, 1, 1, 0.5, 0.01); //values to be tuned
            control_speed = velocity_setpoint;
            speed_right = control_speed;
            speed_left = control_speed;
        }
        else if(movement_mode == 't'){
            finished_turning = false;
            double wheel_speed_difference = simple_PID_calc(sample_interval, angle_setpoint, alpha, 2, 0.01, 0, 0);
            check_turning_finished(abs(alpha-angle_setpoint));
            speed_right -= wheel_speed_difference;
            speed_left += wheel_speed_difference;
            control_speed = 0;
        }
        else{
            speed_right = 0;
            speed_left = 0;
        }
    }
    //No stabilising wheels and frame
    else{
        //Movement control- outer loop changes theta_dash setpoint slightly
        if(abs(theta) > 0.5){ //but only when theta_dash is decently small
            velocity_setpoint = 0;
        }
        if(movement_mode == 'm'){
            theta_setpoint = simple_PID_calc(sample_interval, velocity_setpoint, sensed_speed, 1, 1, 0, 0.01);
        }

        //theta_dash control- always necessary, present in both of other modes too
        control_speed = simple_PID_calc(sample_interval, theta_setpoint, theta, 0, 2.13, 0.053, 0.0102);
        speed_right = control_speed;
        speed_left = control_speed;
        
        //Turning control- slightly modify left and right wheel speeds, but only when theta_dash is decently small
        if((movement_mode == 't') && (abs(theta) < 0.5)){
            finished_turning = false;
            double wheel_speed_difference = simple_PID_calc(sample_interval, angle_setpoint, alpha, 2, 0.01, 0, 0);
            check_turning_finished(abs(alpha-angle_setpoint));
            speed_right -= wheel_speed_difference;
            speed_left += wheel_speed_difference;
            control_speed = 0;
        }
    }
    sample_flag = false;
  }
  //Set speed and increment loop number
  setSpeed(speed_right*8, speed_left*8);
  //cycle_number += 1;
}
///-----CONTROL LOOP | END-----///

///-----COMMUNICATIONS LOOP | START-----///
const unsigned long SEND_INTERVAL_MS = 200;
static unsigned long lastSendMillis = 1;
void Task2Code(void* pvParameters) {
  while(true){
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

    // send beacon every half second
    if (millis() - lastSendMillis >= SEND_INTERVAL_MS) {
        lastSendMillis = millis();

        int LDR_value_L = analogRead(L);  // read LDR connected to pin L
        int LDR_value_R = analogRead(R);  // read LDR connected to pin R
        int LDR_value_F1 = analogRead(F1); // read LDR connected to pin R
        int LDR_value_F2 = analogRead(F2); // read LDR connected to pin R
        int LDR_value_F3 = analogRead(F3); // read LDR connected to pin R

        // JSON doc fixed memory allocation on stack
        StaticJsonDocument<128> doc;
        doc["type"] = "LDR";
        doc["time"] = millis();

        // store LDR readings in JSON array
        JsonArray LDR_Array = doc.createNestedArray("LDR_Array");
        LDR_Array.add(LDR_value_L);
        LDR_Array.add(LDR_value_R);
        LDR_Array.add(LDR_value_F1);
        LDR_Array.add(LDR_value_F2);
        LDR_Array.add(LDR_value_F3);

        char message[128];
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
  if(strcmp(message, "turn") == 0){
    movement_mode = 't';
    angle_setpoint += value;
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
  if(strcmp(message, "read data") == 0){
    //Send back position, light sensors, angle
    Serial.print("x = "); //position
    Serial.print(positionX);
    Serial.print("y = ");
    Serial.println(positionY);
    Serial.println(analogRead(L)); //left LDR
    Serial.println(analogRead(R)); //right LDR
    Serial.println(alpha); //angle
  }
  if(strcmp(message, "print values") == 0){
    Serial.println("Stability mode value");
    Serial.println(STABILITY_MODE);
    Serial.println("Velocity setpoint");
    Serial.println(velocity_setpoint);
    Serial.println("theta value");
    Serial.println(theta);
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
  //store current angle
  //turn to centre beacons
  //stop turning when beacon in frame (angle_setpoint = alpha)
  //send current angle

}
///-----COMMUNICATIONS FUNCTIONS | END-----///