#include <EEPROMex.h>
#include <meArmControlGit.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;
meArmControlGit arm;
LiquidCrystal_I2C lcd(0x27, 20, 4);//Change depending on the size

#define basePot A0
#define distancePot A1
#define heightPot A2
#define ClawSwitch A3
#define basePin 6
#define shoulderPin 10
#define elbowPin 9
#define gripperPin 11
#define S2 4
#define S3 3
#define sensorOut 5
#define blockHeight 30
#define armHeight 100
#define UpButton 7
#define DownButton 8
#define BackButton 2
#define EnterButton 12
#define ButtonDelay 300
#define LedPin 13

int y_arrow = 0;
byte screen;
byte deepness = 0;
bool back, enter = false;
int Rr, Rg, Rb, Gr, Gg, Gb, Br, Bg, Bb, Kr, Kg, Kb;
int r, g, b;
byte closestDegree; 
int closestDistance;
bool clawOpen = true;
bool over = false;
bool working = false;
bool manually = false;
bool manualOnce = false;
long lastButtonMillis;


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
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);
  
  pinMode(basePot, INPUT);
  pinMode(distancePot, INPUT);
  pinMode(heightPot, INPUT);
  pinMode(ClawSwitch, INPUT_PULLUP);
  
  pinMode(BackButton, INPUT_PULLUP);
  pinMode(UpButton, INPUT_PULLUP);
  pinMode(DownButton, INPUT_PULLUP);
  pinMode(EnterButton, INPUT_PULLUP);
  lastButtonMillis = millis();
  readEEPROM();
  lcd.begin();
  lcd.home();
  update_menu();
}

void loop() {
    control_menu();
    if (manually){
      manualMove();
    }
}

char checkColor(){
  readColor();
  /*double diffR = sq(r-Rr) + sq(g-Rg) + sq(b-Rb);
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
  }*/
  if (r < Gr && r < Br && r < Kr){
    return 'r';
  }
  else if (g < Rg && g < Bg && g < Kg){
    return 'g';
  }
  else if (b < Rb && b < Gb && b < Kb){
    return 'b';
  }
  else{
    return 'k';
  }
}
void readColor(){
  digitalWrite(LedPin, HIGH);//turn the LED on
  delay(50);
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
  digitalWrite(LedPin, LOW);
}
void calibrateRed(){
  readColor();
  EEPROM.updateInt(1,r);
  EEPROM.updateInt(2,g);
  EEPROM.updateInt(3,b);
  readEEPROM();
}
void calibrateGreen(){
  readColor();
  EEPROM.updateInt(4,r);
  EEPROM.updateInt(5,g);
  EEPROM.updateInt(6,b);
  readEEPROM();
}
void calibrateBlue(){
  readColor();
  EEPROM.updateInt(7,r);
  EEPROM.updateInt(8,g);
  EEPROM.updateInt(9,b);
  readEEPROM();
}
void calibrateBlack(){
  readColor();
  EEPROM.updateInt(10,r);
  EEPROM.updateInt(11,g);
  EEPROM.updateInt(12,b);
  readEEPROM();
}
void readEEPROM(){
  Rr = EEPROM.readInt(1);
  Rg = EEPROM.readInt(2);
  Rb = EEPROM.readInt(3);
  Gr = EEPROM.readInt(4);
  Gg = EEPROM.readInt(5);
  Gb = EEPROM.readInt(6);
  Br = EEPROM.readInt(7);
  Bg = EEPROM.readInt(8);
  Bb = EEPROM.readInt(9);
  Kr = EEPROM.readInt(10);
  Kg = EEPROM.readInt(11);
  Kb = EEPROM.readInt(12);
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
    Serial.print(F("Distance: ")); 
    Serial.println(range);
  }
  
  if (sensor.timeoutOccurred()) { if(SP)Serial.print(F(" TIMEOUT")); }
  
  //Serial.println();
  return(range);
}
void radar(){
  int dist;
  closestDistance = 700;
  for (int v = 135; v <= 180; v+= 3){
    arm.moveBaseServo(v);
    dist = checkDistance();
    if (dist < closestDistance && dist < 600){
      closestDistance = dist;
      closestDegree = v;
    }
    delay(10);
  }
  if (closestDistance > 300){
    closestDegree = 90;
    over = true;
  }
}
void control_menu(){
  if (Serial.available()){
    move_menu(Serial.read());
  }
  if((millis() - lastButtonMillis) > ButtonDelay){
    if(!digitalRead(UpButton)){
      move_menu('u');
      lastButtonMillis = millis();
    }
    if(!digitalRead(DownButton)){
      move_menu('d');
      lastButtonMillis = millis();
    }
    if(!digitalRead(BackButton)){
      move_menu('b');
      lastButtonMillis = millis();
    }
    if(!digitalRead(EnterButton)){
      move_menu('e');
      lastButtonMillis = millis();
    }
    
  }
}
void move_menu(char letter){
  if (!working){
    switch(letter){
      case 'u':
      y_arrow--;
      break;
      case 'd':
      y_arrow++;
      break;
      case 'e':
      enter = true;
      break;
      case 'b':
      back = true;
      break;
    }
    }
    if (letter == 'b'){
      back = true;
    }
    fix_position();
    update_menu();
}
void fix_position(){
  if (y_arrow < 0){
    y_arrow = 3;
  }
  if (y_arrow > 3){
    y_arrow = 0;
  }
}
void update_menu(){
  if(working && back){
    working = false;
    lcd.clear();
    draw_menu();
    back = false;
    manually = false;
  }
  else if (!working){
    lcd.clear();
    enter_menu();
    if (!working){
    draw_menu();
    }
  }
}
void draw_menu(){
  lcd.setCursor(0,y_arrow);
  lcd.print(F("==>"));
  switch(deepness){
    case 0:
    lcd.setCursor(3,0);
    lcd.print(F("COLOR"));
    lcd.setCursor(3,1);
    lcd.print(F("MOVEMENT"));
    lcd.setCursor(3,2);
    lcd.print(F("CHECK POSITION"));
    lcd.setCursor(3,3);
    lcd.print(F("CHECK DISTANCE"));
    break;
    case 1:
      switch(screen){
        case 0:
          lcd.setCursor(3,0);
          lcd.print(F("CALIBRATE"));
          lcd.setCursor(3,1);
          lcd.print(F("READ COLOR"));
          lcd.setCursor(3,2);
          lcd.print(F("EEPROM VALUES"));
          break;
        case 1:
          lcd.setCursor(3,0);
          lcd.print(F("MOVE MANUALLY"));
          lcd.setCursor(3,1);
          lcd.print(F("AUTOMATIC"));
          lcd.setCursor(3,2);
          lcd.print(F("OPEN/CLOSE CLAW"));
          lcd.setCursor(3,3);
          lcd.print(F("RESET SERVOS"));
          break;
      }
      break;
    case 2:
      switch(screen){
        case 0:
          lcd.setCursor(3,0);
          lcd.print(F("CALIBRATE RED"));
          lcd.setCursor(3,1);
          lcd.print(F("CALIBRATE BLUE"));
          lcd.setCursor(3,2);
          lcd.print(F("CALIBRATE GREEN"));
          lcd.setCursor(3,3);
          lcd.print(F("CALIBRATE BLACK"));
          break;
        case 1:
          lcd.setCursor(3,0);
          lcd.print(F("RADAR"));
          lcd.setCursor(3,1);
          lcd.print(F("PICK"));
          lcd.setCursor(3,2);
          lcd.print(F("LEAVE"));
          lcd.setCursor(3,3);
          lcd.print(F("SORT"));
          break;
      }
      break;
  }
}
void enter_menu(){
  switch(deepness){
    case 0:
    if (enter){
      switch (y_arrow){
        case 0:
          deepness++;
          screen = 0;
          break;
        case 1:
          deepness++;
          screen = 1;
          break;
        case 2:
          Serial.println(F("Check position"));
          lcd.setCursor(0,0);
          lcd.print(F("Distance: "));
          lcd.print(arm.getDistance());
          lcd.setCursor(0,1);
          lcd.print(F("Height: "));
          lcd.print(arm.getHeight());
          lcd.setCursor(0,2);
          lcd.print(F("Degree: "));
          lcd.print(arm.getBase());
          working = true;
          break;
        case 3:
          Serial.println(F("Check distance"));
          lcd.setCursor(0,0);
          lcd.print(F("Distance: "));
          lcd.print(checkDistance());
          lcd.setCursor(0,2);
          lcd.print(F("Degree: "));
          lcd.print(arm.getBase());
          working = true;
          break;
      }
    }
    break;
    case 1:
      switch(screen){
        case 0:
          if (enter){
            switch(y_arrow){
              case 0:
                deepness++;
                break;
              case 1:
                Serial.println(F("Read color"));
                char colorSensor;
                colorSensor= checkColor();
                lcd.setCursor(0,0);
                lcd.print(F("R= "));
                lcd.print(r);
                lcd.setCursor(0,1);
                lcd.print(F("G= "));
                lcd.print(g);
                lcd.setCursor(0,2);
                lcd.print(F("B= "));
                lcd.print(b);
                lcd.setCursor(0,3);
                lcd.print(F("Color: "));
                lcd.print(colorSensor);
                working = true;
                break;
              case 2:
                Serial.println(F("EEPROM values"));
                lcd.setCursor(0,0);
                lcd.print(F("Rr:"));
                lcd.print(EEPROM.readInt(1));
                lcd.setCursor(0,1);
                lcd.print(F("Rg:"));
                lcd.print(EEPROM.readInt(2));
                lcd.setCursor(0,2);
                lcd.print(F("Rb:"));
                lcd.print(EEPROM.readInt(3));
                lcd.setCursor(7,0);
                lcd.print(F("Gr:"));
                lcd.print(EEPROM.readInt(4));
                lcd.setCursor(7,1);
                lcd.print(F("Gg:"));
                lcd.print(EEPROM.readInt(5));
                lcd.setCursor(7,2);
                lcd.print(F("Gb:"));
                lcd.print(EEPROM.readInt(6));
                lcd.setCursor(14,0);
                lcd.print(F("Br:"));
                lcd.print(EEPROM.readInt(7));
                lcd.setCursor(14,1);
                lcd.print(F("Bg:"));
                lcd.print(EEPROM.readInt(8));
                lcd.setCursor(14,2);
                lcd.print(F("Bb:"));
                lcd.print(EEPROM.readInt(9));
                lcd.setCursor(0,3);
                lcd.print(F("Kr:"));
                lcd.print(EEPROM.readInt(10));
                lcd.setCursor(7,3);
                lcd.print(F("Kg:"));
                lcd.print(EEPROM.readInt(11));
                lcd.setCursor(14,3);
                lcd.print(F("Kb:"));
                lcd.print(EEPROM.readInt(12));
                working = true;
                break;
            }
          }
          break;
        case 1:
          if (enter){
            switch(y_arrow){
              case 0:
                Serial.println(F("Move manually"));
                lcd.setCursor(0,0);
                lcd.print(F("Distance: "));
                lcd.setCursor(0,1);
                lcd.print(F("Height: "));
                lcd.setCursor(0,2);
                lcd.print(F("Degree: "));
                manually = true;
                working = true;
                break;
              case 1:
                deepness++;
                break;
              case 2:
                Serial.println(F("Open/close claw"));
                if (clawOpen){
                  arm.closeClaw();
                  clawOpen = false;
                }
                 else if (!clawOpen){
                  arm.openClaw();
                  clawOpen = true;
                 }
                break;
              case 3:
                Serial.println(F("Reset servos"));
                resetServos();
                break;
            }
          }
          break;
      }
      if (back){
         deepness--;
         manualOnce = false;
      }
      break;
    case 2:
      switch(screen){
        case 0:
          if (enter){
            switch (y_arrow){
              case 0:
                Serial.println(F("Calibrate red"));
                calibrateRed();
                break;
              case 1:
                Serial.println(F("Calibrate blue"));
                calibrateBlue();
                break;
              case 2:
                Serial.println(F("Calibrate green"));
                calibrateGreen();
                break;
              case 3:
                Serial.println(F("Calibrate black"));
                calibrateBlack();
                break;
            }
            lcd.setCursor(0,0);
            lcd.print(F("R= "));
            lcd.print(r);
            lcd.setCursor(0,1);
            lcd.print(F("G= "));
            lcd.print(g);
            lcd.setCursor(0,2);
            lcd.print(F("B= "));
            lcd.print(b);
            working = true;
          }
          break;
        case 1:
          if (enter){
            switch (y_arrow){
              case 0:
                Serial.println(F("Radar"));
                radar();
                arm.moveBaseServo(closestDegree);
                lcd.setCursor(0,0);
                lcd.print(F("Degree: "));
                lcd.print(closestDegree);
                lcd.setCursor(0,2);
                lcd.print(F("Distance: "));
                lcd.print(closestDistance);
                working = true;
                break;
              case 1:
                Serial.println(F("Pick"));
                pick();
                break;
              case 2:
                Serial.println(F("Leave"));
                leave();
                break;
              case 3:
                Serial.println(F("Sort"));
                do {
                pick();
                if (over){
                  break;
                }
                leave();
                } while (!over);
                over = false;
                break;
            }
          } 
          break;
      }
      if (back){
        deepness--;
      }
      break;
  }
  enter = false;
  back = false;
}
void pick(){
  radar();
  if (!over){
         arm.moveArm(armHeight, 70 , closestDegree);
         delay(500);
         arm.moveArm(blockHeight, closestDistance -20 , closestDegree);
         delay(500);
         arm.closeClaw();
         clawOpen = false;
         delay(500);
         Serial.println(checkColor());//DOESNT WORK RIGHT NOW
         arm.moveArm(armHeight,closestDistance -20, closestDegree );
  }
}
void leave(){
  int colorDegree;
  switch(checkColor()){
    case 'k':
      colorDegree = closestDegree;
      break;
    case 'g':
      colorDegree = 100;
      break;
    case 'b':
      colorDegree = 70;
      break;
    case 'r':
      colorDegree = 20;
      break;
  }
  delay(500);
         arm.moveArm(armHeight,armHeight,colorDegree);
         delay(500);
         arm.moveArm(blockHeight,armHeight,colorDegree);
         delay(500);
         arm.openClaw();
         clawOpen = true;
         delay(500);
         arm.moveArm(armHeight,armHeight,colorDegree);
         delay(500);
         resetServos();
}
void manualMove(){
  long timer;
    if(!digitalRead(ClawSwitch))
    arm.openClaw();
  else
    arm.closeClaw();

  int HPV = analogRead(heightPot);
  int BPV = analogRead(basePot);
  int DPV = analogRead(distancePot);

  int heightManual = map(HPV, 0, 1023, 0, 160);
  int baseManual = map(BPV, 0, 1023, 0, 180); 
  int distanceManual = map(DPV, 0, 1023, 0, 160);
   
   if(canReach(heightManual, baseManual, distanceManual)){
    if(!manualOnce){
    manualOnce = true;
    timer = millis();
    }
    else if (millis() - timer > 1000){
      timer = millis();
      lcd.setCursor(10,0);
      lcd.print(F("   "));
      lcd.setCursor(10,0);
      lcd.print(distanceManual);
      lcd.setCursor(8,1);
      lcd.print(F("   "));
      lcd.setCursor(8,1);
      lcd.print(heightManual);
      lcd.setCursor(8,2);
      lcd.print(F("   "));
      lcd.setCursor(8,2);
      lcd.print(baseManual);
    }
    arm.moveArm(heightManual, distanceManual, baseManual);
  }
}
bool canReach(int heightManual,int baseManual,int distanceManual){
  return(sqrt(sq(heightManual) + sq(baseManual)) >= 160);
}


