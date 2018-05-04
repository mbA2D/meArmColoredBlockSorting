#include "meArmControlGit.h"
#include <Servo.h>
#include "Adafruit_VL6180X.h"

#define basePin 11
#define shoulderPin 10
#define elbowPin 9
#define gripperPin 6

meArmControlGit arm;
Adafruit_VL6180X vl = Adafruit_VL6180X();
void setup() {
  // put your setup code here, to run once:
  //attach all servos
  arm.beginArm(basePin, shoulderPin, elbowPin, gripperPin);
  delay(500);
  Serial.begin(115200);
  
   while (!Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");

  arm.moveBaseServo(0);
  arm.moveGripperServo(90);
  arm.moveShoulderServo(1500);
  arm.moveElbowServo(1500);
}
void loop() {
  int degree [181];
  int minDistance = 10000;
  int minDistanceDegree;
  for (int v = 0; v <= 180; v++){
    arm.moveBaseServo(v);
    degree[v] = checkDistance();
    if (degree[v] < minDistance){
      minDistance = degree[v];
      minDistanceDegree = v;
    }
    delay(10);
  }
  arm.moveBaseServo(minDistanceDegree);
  delay(2000);
}
int checkDistance(){
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
    return range;
  }
  else {
    Serial.println("Nothing");
    return 10000;
  }
}

