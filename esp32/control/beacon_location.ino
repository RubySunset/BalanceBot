#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050.h>

//format RYB xmin_r xmax_r xmin_y xmax_y xmin_b xmax_b
int boundingBoxArray[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
void retrieveBoundingBoxes() {
  // static const int BUFFER_SIZE = 28;  // 4 for RBY and 16 for the rest
  // char buf[BUFFER_SIZE];
  int beacon_array[6];

  const int BUFFER_SIZE = 36;
  char buf[BUFFER_SIZE];

  if (Serial.available() > 0) {
    // read incoming bytes

    int rlen = Serial.readBytesUntil('Y', buf, BUFFER_SIZE);
    // int rlen = Serial.readBytes(buf, BUFFER_SIZE);

    // prints the received data
    // Serial.print("Received: ");
    // for(int i = 0; i < rlen; i++) {
    //   Serial.print(buf[i], DEC);
    //   Serial.printf(" ");
    // }

    // Serial.println();
    // Serial.println();

    // RBY
    beacon_array[0] = (int)buf[5] + (int)(buf[6] << 8);
    beacon_array[1] = (int)buf[9] + (int)(buf[10] << 8);

    beacon_array[2] = (int)buf[13] + (int)(buf[14] << 8);
    beacon_array[3] = (int)buf[17] + (int)(buf[18] << 8);

    beacon_array[4] = (int)buf[21] + (int)(buf[22] << 8);
    beacon_array[5] = (int)buf[25] + (int)(buf[26] << 8);

    // print array
    // Serial.printf("%d %d %d %d %d %d", beacon_array[0], beacon_array[1], beacon_array[2], beacon_array[3], beacon_array[4], beacon_array[5]);
    // Serial.println();

    //format: xmin_r[0] xmax_r[1] xmax_b[2] xmin_b[3] xmin_y[4] xmax_y[5] -> r_present y_present b_present xmin_r xmax_r xmin_y xmax_y xmin_b xmax_b
    boundingBoxArray[3] = beacon_array[0];                                                 //xmin_r
    boundingBoxArray[4] = beacon_array[1];                                                 //xmax_r
    boundingBoxArray[5] = beacon_array[4];                                                 //xmin_y
    boundingBoxArray[6] = beacon_array[5];                                                 //xmax_y
    boundingBoxArray[7] = beacon_array[2];                                                 //xmin_b
    boundingBoxArray[8] = beacon_array[3];                                                 //xmax_b
    boundingBoxArray[0] = (abs(boundingBoxArray[3] - boundingBoxArray[4]) > 300) ? 0 : 1;  //r_present
    boundingBoxArray[1] = (abs(boundingBoxArray[5] - boundingBoxArray[6]) > 300) ? 0 : 1;  //y_present
    boundingBoxArray[2] = (abs(boundingBoxArray[7] - boundingBoxArray[8]) > 300) ? 0 : 1;  //b_present
    if((boundingBoxArray[0] == 1) && (boundingBoxArray[1] == 1)){
      if(abs(boundingBoxArray[3] - boundingBoxArray[4]) > abs(boundingBoxArray[5] - boundingBoxArray[6])){ //if red wider than yellow
        boundingBoxArray[1] = 0; //no yellow in frame, only red
        boundingBoxArray[0] = 1;
      }
      else if (abs(boundingBoxArray[3] - boundingBoxArray[4]) < abs(boundingBoxArray[5] - boundingBoxArray[6])){ //if yellow wider than red
        boundingBoxArray[0] = 0; //no red in frame, only yellow
        boundingBoxArray[1] = 1;
      }
    }
    // Serial.printf("%d %d %d %d %d %d", boundingBoxArray[3], boundingBoxArray[4], boundingBoxArray[5], boundingBoxArray[6], boundingBoxArray[7], boundingBoxArray[8]);
    // Serial.println();
    // Serial.printf("%d %d %d", boundingBoxArray[0], boundingBoxArray[1], boundingBoxArray[2]);
    // Serial.println();
  }
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
    //if (facingBeaconMiddle(boundingBoxArray[3], boundingBoxArray[4])) redBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  } else if (boundingBoxArray[1] == 1) {  //YELLOW BEACON PRESENT
    //if (facingBeaconMiddle(boundingBoxArray[5], boundingBoxArray[6])) yellowBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  } else if (boundingBoxArray[2] == 1) {  //BLUE BEACON PRESENT
    //if (facingBeaconMiddle(boundingBoxArray[7], boundingBoxArray[8])) blueBeaconAngle = normaliseAngle(alpha);
    foundBeacon = true;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  check_for_beacon();
  Serial.println(String((boundingBoxArray[7]+boundingBoxArray[8])/2));
  if (facingBeaconMiddle(boundingBoxArray[7], boundingBoxArray[8]) && (boundingBoxArray[2] == 1)){
    Serial.println("Blue in middle");
  }
  else{
    Serial.println("nothing");
  }
  
  //Serial.println("Beacons present: R[" + String(boundingBoxArray[0]) + "] Y[" + String(boundingBoxArray[1]) + "] B[" + String(boundingBoxArray[2]) + "]");
  //Serial.println("xmin and xmax of red: [" + String(boundingBoxArray[3]) + ", " + String(boundingBoxArray[4]) + "]");

  delay(50);
}
