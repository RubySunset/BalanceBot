#define F1 27
#define F2 26
#define F3 25
#define B1 33
#define B2 14
#define B3 4
#define L 15
#define R 2


void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  Serial.print("F1: ");
  Serial.print(analogRead(F1));
  Serial.print(" F2: ");
  Serial.print(analogRead(F2));
  Serial.print(" F3: ");
  Serial.print(analogRead(F3));
  Serial.print(" B1: ");
  Serial.print(analogRead(B1));
  Serial.print(" B2: ");
  Serial.print(analogRead(B2));
  Serial.print(" B3: ");
  Serial.print(analogRead(B3));
  Serial.print(" L: ");
  Serial.print(analogRead(L));
  Serial.print(" R: ");
  Serial.println(analogRead(R));
  
  delay(500);
}
