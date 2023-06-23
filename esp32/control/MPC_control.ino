#include <AccelStepper.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <ElementStorage.h>

AccelStepper s1(AccelStepper::DRIVER, 23, 19); //STRR, DIRR
AccelStepper s2(AccelStepper::DRIVER, 18, 5); //STRL, DIRL

Adafruit_MPU6050 mpu;

#define DIRR 1 //set to +-1 to compensate for rotating direction
#define DIRL 1

double theta;
double alpha;

void setSpeed(int r, int l){
  s1.setSpeed(r*DIRR*100/PI);
  s2.setSpeed(l*DIRL*100/PI);
  s1.runSpeed();
  s2.runSpeed();
}

//Variables
//Matrices
BLA::Matrix<2, 1> u;
BLA::Matrix<6, 1> xi;
BLA::Matrix<2, 1> y_r; //(setpoint x, alpha)
BLA::Matrix<2, 1> y; //(sensor readings x, alpha)
//Scalars
float v = 0;
float p = 0;

//Constants (from MATLAB)
BLA::Matrix<2, 2> y_d_multiplier = {-1.09839127042025, 1.92478087144943, -1.12318464989123, -1.79298180707917};
BLA::Matrix<6, 6> xi_multiplier = {1.73451495505341, 0.0734873224445586, -0.00877732414000678, 0, 2.07794814152054e-05, 0.000875327078722625, 
                                   0.215151515855384, 0.522704602840403, -0.0282586134224009, 0, 0.000735109154177438, 0.0157715275876819,
                                   -0.00515276261333028, 0, 1.79883849181184, 0.0853843258614709, 0, 0,
                                   -0.00304018482436085, 0, -0.114399729832995, 0.722672776897566,0, 0,
                                   24.9957919218476, 0.0191151894989558, -0.579229113431333, 0, 1.00559229004050, 0.0995557432505031,
                                   26.2730778225256, 0.344524553032446, -0.594852585252632, 0, 0.111720116802753, 0.994222979790427};
BLA::Matrix<6, 2> L = {0.734514955053406, -0.00877732414000678,
                       0.215151515855384, -0.0282586134224009,
                       -0.00515276261333028,  0.798838491811838,
                       -0.00304018482436085, -0.114399729832995,
                       24.9957919218476,  -0.579229113431333,
                       26.2730778225256,  -0.594852585252632};
BLA::Matrix<2, 6> K = {-1.09839127042025, -15.6955031647759, 1.92478087144942,  0.570465479930875, -28.4177452362045, -26.2831229841478,
                       -1.12318464989123, -15.8193731863461, -1.79298180707918, -0.438866299633290,  -28.5689865515644, -26.4335662345272};
BLA::Matrix<6, 2> B_d = {0.00437459179664784, 0.00437459179664784,
                         0.0787537405313335,  0.0787537405313335,
                         0.0371013266593431,  -0.0371013266593431,
                         0.703984489413872, -0.703984489413872,
                         -0.00315400626732771,  -0.00315400626732771,
                         -0.0568465512503537, -0.0568465512503537};

void setup(){
  Serial.begin(115200);  
  s1.setMaxSpeed(700);
  s2.setMaxSpeed(700);
  mpu.begin();
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //initialising matrices
  xi.Fill(0);
  y.Fill(0);
}
                                      
void loop(){
  //sensor data in rad/s
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    theta = -g.gyro.y; //tilt forward = positive
    alpha = g.gyro.x; //rotate right = positive
    Serial.println(theta);
  
  //sets speed of motors, in rad/s
    //setSpeed(2, 2);
  
  //to simulate sampling time, motor doesn't work well for 0.1s for now
    delay(100);

  //Control variables
  //setpoint, change later
  y_r = (0.05, 0);
  //suvat- but might not be enough :/
  v = v - a.acceleration.z;
  p = p + v;
  y = (p, alpha);`
  //Control
  u = -K*xi + y_d_multiplier*y_r;
  xi = xi_multiplier*xi + B_d*u + L*y;
  //Actuate
  setSpeed(u(1)/0.33,u(2)/0.33);
}
