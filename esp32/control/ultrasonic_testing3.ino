// Ultrasonic sensor pins
#define trigPin_L 33
#define trigPin_F 32
#define trigPin_R 17                
#define echoPin_L 34
#define echoPin_F 36
#define echoPin_R 39 

// Ultrasonic sensor variables - duration is in us, distance is in cm.
float dur_F, dist_F, dur_L, dist_L, dur_R, dist_R;

void setup() {
  // begin serial port
  Serial.begin(9600);

  // configure the trigger pin to output mode
  pinMode(trigPin_F, OUTPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(trigPin_R, OUTPUT);
  // configure the echo pins to input mode
  pinMode(echoPin_F, INPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(echoPin_R, INPUT);
}

void loop() {
  digitalWrite(trigPin_F, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_F, LOW);
  dur_F = pulseIn(echoPin_F, HIGH);
  dist_F = 0.017 * dur_F;

  digitalWrite(trigPin_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_L, LOW);
  dur_L = pulseIn(echoPin_L, HIGH);
  dist_L = 0.017 * dur_L;

  digitalWrite(trigPin_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_R, LOW);
  dur_R = pulseIn(echoPin_R, HIGH);
  dist_R = 0.017 * dur_R;

  Serial.print("Front distance: ");
  Serial.print(dist_F);
  Serial.println(" cm");
  Serial.print("Left distance: ");
  Serial.print(dist_L);
  Serial.println(" cm");
  Serial.print("Right distance: ");
  Serial.print(dist_R);
  Serial.println(" cm");

  delay(500);
}
