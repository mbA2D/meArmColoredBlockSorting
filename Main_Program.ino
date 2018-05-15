
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
int Rr, Rg, Rb, Gr, Gg, Gb, Br, Bg, Bb, Kr, Kg, Kb;//Read values from EEPROM instead
int r, g, b;
bool clawOpen = true;

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
   d = check distance CHECK
   y = move y-axis CHECK
   x = move x-axis CHECK
   t = rotate CHECK
   e = output EEPROM values
   w = change claw state
  */
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
        break;
      case 'd':
      Serial.println("Check distance");
        break;
      case 'x':
      Serial.println("Type distance");
        while(1){
          if (Serial.available()){
            Serial.println("move foward or backwards x cm");
            break;
          }
        }
        break;
      case 'y':
      Serial.println("Type height");
        while(1){
          if (Serial.available()){
            Serial.println("move up or down x cm");
            break;
          }
        }
        break;
      case 't':
      Serial.println("Type degree");
      while(1){
          if (Serial.available()){
            arm.moveBaseServo(Serial.parseInt());
            break;
          }
        }
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
  arm.moveBaseServo(0);
  arm.moveGripperServo(90);
  arm.moveShoulderServo(90);
  arm.moveElbowServo(90);
  arm.openClaw();
}

