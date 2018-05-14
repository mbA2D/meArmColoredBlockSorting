#include "meArmControlGit.h"
#include <Servo.h>
#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

#define basePin 6
#define shoulderPin 10
#define elbowPin 9
#define gripperPin 11

bool SW = false;

meArmControlGit arm;

void setup() {
  // put your setup code here, to run once:
  //attach all servos
  arm.beginArm(basePin, shoulderPin, elbowPin, gripperPin);
  delay(500);
  Serial.begin(115200);
  delay(1000);
   Wire.begin();

  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  arm.moveBaseServo(0);
  arm.moveGripperServo(90);
  arm.moveShoulderServo(90);
  arm.moveElbowServo(90);
}
void loop() {
  arm.moveArm(120,30,90);
  arm.openClaw();
  delay(1000);

  int degree [46];
  int minDistance = 10000;
  int minDistanceDegree;
  for (int v = 180; v >= 135; v --){
    arm.moveBaseServo(v);
    degree[v] = checkDistance();
    if (degree[v] < minDistance){
      minDistance = degree[v];
      minDistanceDegree = v;
    }
    //delay(10);
  }
  arm.moveBaseServo(minDistanceDegree);
  delay(1000);
  arm.moveArm(30,minDistance,minDistanceDegree);
  delay(500);
  arm.closeClaw();
  delay(1000000);
  /*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }*/
  
  /*
  arm.moveArm(120,50,0);
  if(SW){waitSerial();}
  else{delay(500);}
  
  arm.openClaw();
  if(SW){waitSerial();}
  else{delay(200);}
  
  int dist = checkDistance();
  arm.moveArm(50,40,0);
  delay(500);
  arm.moveArm(50,dist,0);
  if(SW){waitSerial();}
  else{delay(700);}
  
  arm.closeClaw();
  if(SW){waitSerial();}
  else{delay(500);}

  Serial.println("test");
   
  while (1)
  {delay(2000);}
  */
  
}

void waitSerial(){
  while(!Serial.available())
  {delay(10);}
  int none = Serial.read();
  Serial.flush();
}

int checkDistance(){
 
  int range = 0;
  for(int i = 0; i < 5; i++){
    range += sensor.readRangeSingleMillimeters();
  }
  range /= 5;

  if(SP)Serial.print(range);

  if(range > 100 && range <= 150){
    range = ((range - 100) * 1.58) + 100;
  }
  else if (range > 150){
    range = ((range - 150) * 8) + 150;
  }

  if(SP)
  {
    Serial.print("   "); 
    Serial.println(range);
  }
  
  if (sensor.timeoutOccurred()) { if(SP)Serial.print(" TIMEOUT"); }
  
  //Serial.println();
  return(range);
}

