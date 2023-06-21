#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

// Ultrasonic sensor pins
int trigPin = 16;
int echoPin_F = 35;
int echoPin_L = 33;
int echoPin_R = 39;

// Motor pins
const int STRR = 23;
const int DIRR = 19;
const int STRL = 18;
const int DIRL = 5;

// Ultrasonic sensor variables - duration is in us, distance is in cm.
float dur_F, dist_F, dur_L, dist_L, dur_R, dist_R;

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

//Note that for multiple PID controllers, we use an array for sum_e and e_n_1
//index 0 is for the theta_dash controller, index 1 is for x', and index 2 is for alpha_dash
double sum_e[3] = { 0, 0, 0 }, e_n_1[3] = { 0, 0, 0 };  //sum of errors and previous error initialisation
double control_max = 10;                                //absolute maximum value of actuators
double speed_max = 5;
char movement_mode = 's';  //current movement mode of robot; s = stationary, m = moving, t = turning
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
double angle_setpoint = 0;  //decide that clockwise is positive direction; so could be +20deg, -35deg, etc

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
double calibrateMPU(){
  for(int i = 0; i < 1000; i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    compl_alpha = 0.999*(compl_alpha+gy*0.002/131)+0.001*atan2(ax,az)*RAD_TO_DEG;
    delay(2);
  }
  return compl_alpha;
}
double compl_theta = 0;

//carries out appropriate actions once turning done
//figure out what- send message to node? retrieve instructions from some stack?
void check_turning_finished(double alpha_error) {
  if (alpha_error < 0.7) {
    Serial.println("Finished turning");
    angle_setpoint = alpha;
    finished_turning = true;
    movement_mode = 'm';
    velocity_setpoint = 0;
  }
}

// //-----Course correction controller-----//
// double get_distance(double x1, double y1, double x2, double y2) {
//   return (sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)));
// }

//gains
// double P_CC = 0.3;
// double I_CC = 0.0001;
// double D_CC = 0.5;
// double MAX_OFFSET = 60;     //max course correction, degrees
// double UNBOUND_DIST = 0.5;  //distance for which unbounded CC is appled
// // Unbounded CC ignores the condition imposed by MAX_OFFSET. in testing, I found that the robot tended to move
// // better if it was allowed to course correct itself by larger amounts shortly after leaving a vertex

// // CC controller variables
// bool cc_active = false;
// double cc_sum = 0;
// double cc_prev_balance = 0;
// double cc_prev_left_dist = 0;
// double cc_prev_right_dist = 0;
// double cc_offset = 0;
// double cc_prev_vertex_x = 0;
// double cc_prev_vertex_y = 0;
// double cc_prev_width = 0;
// double cc_alpha = 0;

//Apply course correction. Returns the target angle, alpha
// double course_correct() {
//   //only apply course correction in a corridor
//   if ((dist_L <= 25) && (dist_R <= 25)) {
//     double balance = dist_L - dist_R;  //measure of closeness to right wall compared to left wall
//     double width = dist_L - dist_R;    //estimated width of corridor

//     if (cc_active) {
//       //Rates of change of distances to respective walls
//       double left_diff = dist_L - cc_prev_left_dist;
//       double right_diff = dist_R - cc_prev_right_dist;

//       if ((abs(width - cc_prev_width) < 30) && (abs(abs(left_diff) - abs(right_diff)) < 30)) {
//         //A link to the side is inferred if:
//         //1. The width of the passage is increasing rapidly
//         //2. One wall is receeding much more quickly than the other
//         //This condition also needs tuning
//         cc_sum += balance;
//         double diff = balance - cc_prev_balance;
//         double delta = P_CC * balance + I_CC * cc_sum + D_CC * diff;
//         // if (get_distance(cc_prev_vertex_x, cc_prev_vertex_y, positionX, positionY) <= UNBOUND_DIST) {
//         //   //if we're close enough to the last vertex (aka likely junction), neglect to limit CC
//         //   cc_alpha -= delta;
//         // } else {
//           cc_offset -= delta;
//           if ((abs(cc_offset) <= MAX_OFFSET)) {
//             cc_alpha -= delta;  //if we're within our allowance, proceed as normal
//           } else if ((abs(cc_offset) >= MAX_OFFSET + delta)) {
//             cc_offset += delta;  //if we're completely out, revert the change to cc_offset
//           } else {
//             //we can reduce delta to match the offset limit
//             if (delta > 0) {
//               delta -= abs(cc_offset) - MAX_OFFSET;
//             } else {
//               delta += abs(cc_offset) - MAX_OFFSET;
//             }
//             cc_alpha -= delta;
//           }
//         // }
//       }
//     }
//     cc_active = true;
//     cc_prev_balance = balance;
//     cc_prev_left_dist = dist_L;
//     cc_prev_right_dist = dist_R;
//     cc_prev_width = width;
//     return true
//   } else {
//     //Reset the variables once the controller is inative
//     cc_active = false;
//     cc_sum = 0;
//     cc_offset = 0;
//     return false
//   }
//   return cc_alpha;
// }

void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(trigPin, OUTPUT);
  // configure the echo pins to input mode
  pinMode(echoPin_F, INPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(echoPin_R, INPUT);

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
}

void loop() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure duration of pulse from ECHO pins
  dur_F = pulseIn(echoPin_F, HIGH);
  dur_L = pulseIn(echoPin_L, HIGH);
  dur_R = pulseIn(echoPin_R, HIGH);

  // calculate the distances
  dist_F = 0.017 * dur_F;
  dist_L = 0.017 * dur_L;
  dist_R = 0.017 * dur_R;

  // print the value to Serial Monitor
//   Serial.print("Front distance: ");
//   Serial.print(dist_F);
//   Serial.println(" cm");
//   Serial.print("Left distance: ");
//   Serial.print(dist_L);
//   Serial.println(" cm");
//   Serial.print("Right distance: ");
//   Serial.print(dist_R);
//   Serial.println(" cm");

  if (sample_flag == true) {
      //Sensor data
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //angle speeds in degrees/s
      theta_dash = -gx / 131;
      alpha_dash = -gz / 131;
      theta += sample_interval * theta_dash;
      alpha += sample_interval * alpha_dash;

      //alpha complementary filter
      int accy = ay/1638;
      int accz = az/1638;   
      compl_alpha = COMPL_ALPHA*(compl_alpha + alpha_dash*0.002)-(1-COMPL_ALPHA)*(atan2(accy,accz)+PI)*RAD_TO_DEG;
      if(k>filter_adjustment_interval){
        alpha = compl_alpha;
        k=0;
      }
      compl_theta = 0.98*(compl_theta+gx*0.002/131)+0.02*atan(ay / sqrt(pow(ax, 2.0) + pow(az, 2.0)))*RAD_TO_DEG;

      //velocity and position in m/s and m. displacement is since last sample
      double displacement = distance_moved(control_speed);
      sensed_speed = displacement / sample_interval;
      positionX += displacement * sin(alpha * PI / 180);
      positionY += displacement * cos(alpha * PI / 180);

      //Course-correction takes priority
      if (movement_mode == 'm') {
        if (dist_L <= 10) {
            movement_mode = 't';
            angle_setpoint = alpha + 10;
        } else if (dist_R <= 10) {
            movement_mode = 't';
            angle_setpoint = alpha - 10;
        }
      }
    }

    // Assume stabilising wheels and frame attached
    if (movement_mode == 'm') {
        //control_speed = simple_PID_calc(sample_interval, velocity_setpoint, sensed_speed, 1, 1, 0.5, 0.01); //values to be tuned
        control_speed = velocity_setpoint;
        speed_right = control_speed;
        speed_left = control_speed;
    } else if (movement_mode == 't') {
        finished_turning = false;
        double wheel_speed_difference = simple_PID_calc(sample_interval, angle_setpoint, alpha, 2, 0.01, 0, 0);
        check_turning_finished(abs(alpha - angle_setpoint));
        speed_right -= wheel_speed_difference;
        speed_left += wheel_speed_difference;
        control_speed = 0;
    } else {
        speed_right = 0;
        speed_left = 0;
    }
    sample_flag = false;
    //Set speed and increment loop number
    setSpeed(speed_right * 8, speed_left * 8);
    //cycle_number += 1;
}