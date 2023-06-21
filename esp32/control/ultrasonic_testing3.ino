// Ultrasonic sensor pins
int trigPin = 16;
int echoPin_F = 35;
int echoPin_L = 33;
int echoPin_R = 39;

// Ultrasonic sensor variables - duration is in us, distance is in cm.
float dur_F, dist_F, dur_L, dist_L, dur_R, dist_R;

void setup() {
  // begin serial port
  Serial.begin (9600);

  // configure the trigger pin to output mode
  pinMode(trigPin, OUTPUT);
  // configure the echo pins to input mode
  pinMode(echoPin_F, INPUT);
  pinMode(echoPin_L, INPUT);
  pinMode(echoPin_R, INPUT);
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
