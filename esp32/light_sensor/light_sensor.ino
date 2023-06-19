///-----LIGHT SENSOR PINS | START-----///
#define F1 32  //A4
#define F2 35  //A5
#define F3 34  //A3
#define L 33  //soldered connection
#define R 39  //soldered connection
///-----LIGHT SENSOR PINS | END-----///

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
  Serial.print(" L: ");
  Serial.print(analogRead(L));
  Serial.print(" R: ");
  Serial.println(analogRead(R));
  
  delay(100);
}