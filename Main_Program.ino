
#include <meArmControlGit.h>

#include<EEPROM.h>

#include <Servo.h>
#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;
meArmControlGit arm;

#define basePin 6
#define shoulderPin 10
#define elbowPin 9
#define gripperPin 11
#define S2 4
#define S3 3
#define sensorOut 5
#define blockHeight 30
#define armHeight 100
int Rr, Rg, Rb, Gr, Gg, Gb, Br, Bg, Bb, Kr, Kg, Kb;//Read values from EEPROM instead
int r, g, b;
int closestDegree, closestDistance;
bool clawOpen = true;
bool over = false;

void setup() {
  Serial.begin(115200);
  arm.beginArm(basePin, shoulderPin, elbowPin, gripperPin);
  delay(500);
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  resetServos();
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  readEEPROM();
}

void loop() {
  /*
   r = calibrate red
   g = calibrate green
   b = calibrate blue
   k = calibrate black
   c = read color
   a = base radar CHECK
   d = check distance 
   m = move arm
   e = output EEPROM values
   w = change claw state
   p = pick block
  */
  if (!over){
  if(Serial.available()){
    switch(Serial.read()){
      case 'r':
        Serial.println("Calibrate red");
        calibrateRed();
        break;
      case 'g':
        Serial.println("Calibrate green");
        calibrateGreen();
        break;
      case 'b':
        Serial.println("Calibrate blue");
        calibrateBlue();
        break;
      case 'k':
        Serial.println("Calibrate black");
        calibrateBlack();
        break;
      case 'c':
        Serial.println("Reading");
        Serial.println(checkColor());
        break;
      case 'a':
        Serial.println("Base radar");
        radar();
        arm.moveBaseServo(closestDegree);
        break;
      case 'd':
      Serial.println("Check distance");
      checkDistance();
        break;
      case 'm':
      int height, distance, degree;
      Serial.println("Type height");
        while(1){
          if (Serial.available()){
            height = Serial.parseInt();
            break;
          }
        }
      Serial.println("Type distance");
        while(1){
          if (Serial.available()){
            distance = Serial.parseInt();
            break;
          }
        }
      Serial.println("Type degree");
      while(1){
          if (Serial.available()){
            degree = Serial.parseInt();
            break;
          }
        }
        arm.moveArm(height, distance, degree);
        break;
      case 'e':
      Serial.println("EEPROM values:");
      for (int i = 1; i <= 12; i++){
        Serial.println(EEPROM.read(i));
      }
        break;
      case 'w':
        if (clawOpen){
          arm.closeClaw();
          clawOpen = false;
        }
        else if (!clawOpen){
          arm.openClaw();
          clawOpen = true;
        }
        break;
       case 't':
         resetServos();
         break;
       case 'p':
        int blockNum = 1;
        Serial.println("Number of blocks: ");
        while(1){
          if(Serial.available()){
            blockNum = Serial.parseInt();
            break;
          }
        }
       for (int k = 0; k < blockNum;k++){
         radar();
         arm.moveArm(armHeight, closestDistance -20 , closestDegree);
         delay(500);
        arm.moveArm(blockHeight, closestDistance -20 , closestDegree);
         delay(1000);
         arm.closeClaw();
         clawOpen = false;
         delay(1000);
         Serial.println(checkColor());//DOESNT WORK RIGHT NOW
         arm.moveArm(armHeight,closestDistance -30, closestDegree );
         delay(500);
         arm.moveArm(armHeight,armHeight,70 + k * 20);
         delay(500);
         arm.moveArm(blockHeight,armHeight,70 + k * 20);
         delay(500);
         arm.openClaw();
         clawOpen = true;
         delay(500);
         arm.moveArm(armHeight,armHeight,70 + k * 20);
         delay(500);
         resetServos();
       }
         break;
    }
  }
  }
}

char checkColor(){
  readColor();
  double diffR = sq(r-Rr) + sq(g-Rg) + sq(b-Rb);
  double diffG = sq(r-Gr) + sq(g-Gg) + sq(b-Gb);
  double diffB = sq(r-Br) + sq(g-Bg) + sq(b-Bb);
  double diffK = sq(r-Kr) + sq(g-Kg) + sq(b-Kb);

  double c = min(min(diffR, diffG), min(diffB, diffK));

  if(c == diffR){
    return 'r';
  }
  if(c == diffG){
    return 'g';
  }
  if(c == diffB){
    return 'b';
  }
  if(c == diffK){
    return'k';
  }
}
void readColor(){
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  r = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(r);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  g = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(g);//printing GREEN color frequency
  Serial.print("  ");
  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  b = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.println(b);//printing BLUE color frequency
}
void calibrateRed(){
  readColor();
  EEPROM.update(1,r);
  EEPROM.update(2,g);
  EEPROM.update(3,b);
  readEEPROM();
}
void calibrateGreen(){
  readColor();
  EEPROM.update(4,r);
  EEPROM.update(5,g);
  EEPROM.update(6,b);
  readEEPROM();
}
void calibrateBlue(){
  readColor();
  EEPROM.update(7,r);
  EEPROM.update(8,g);
  EEPROM.update(9,b);
  readEEPROM();
}
void calibrateBlack(){
  readColor();
  EEPROM.update(10,r);
  EEPROM.update(11,g);
  EEPROM.update(12,b);
  readEEPROM();
}
void readEEPROM(){
  Rr = EEPROM.read(1);
  Rg = EEPROM.read(2);
  Rb = EEPROM.read(3);
  Gr = EEPROM.read(4);
  Gg = EEPROM.read(5);
  Gb = EEPROM.read(6);
  Br = EEPROM.read(7);
  Bg = EEPROM.read(8);
  Bb = EEPROM.read(9);
  Kr = EEPROM.read(10);
  Kg = EEPROM.read(11);
  Kb = EEPROM.read(12);
}
void resetServos(){
  arm.moveBaseServo(90);
  arm.moveGripperServo(armHeight);
  arm.moveShoulderServo(armHeight);
  arm.moveElbowServo(armHeight);
  arm.openClaw();
}
int checkDistance(){
 
  int range = 0;
  for(int i = 0; i < 5; i++){
    range += sensor.readRangeSingleMillimeters();
  }
  range /= 5;


  if(range > 100 && range <= 150){
    range = ((range - 100) * 1.58) + 100;
  }
  else if (range > 150){
    range = ((range - 150) * 8) + 150;
  }

  if(SP)
  {
    Serial.print("Distance: "); 
    Serial.println(range);
  }
  
  if (sensor.timeoutOccurred()) { if(SP)Serial.print(" TIMEOUT"); }
  
  //Serial.println();
  return(range);
}
void radar(){
  int dist;
  closestDistance = 500;
  for (int v = 180-90; v <= 180; v+= 3){
    arm.moveBaseServo(v);
    dist = checkDistance();
    if (dist < closestDistance && dist < 300){
      closestDistance = dist;
      closestDegree = v;
      nothing = false;
    }
    delay(10);
  }
  if (closestDistance > 300){
    closestDegree = 90;
    Serial.println("END");
    //over = true;
  }
}

