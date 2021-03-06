
//Which sensor is being used
//#define Sensor_VL6180X
#define Sensor_VL53L0X

#include <EEPROMex.h>
#include <meArmControlGit.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>

#ifdef Sensor_VL6180X
  #include <VL6180X.h>
  VL6180X sensor;
#endif
#ifdef Sensor_VL53L0X
  #include <VL53L0X.h>
  VL53L0X sensor;
#endif
  

meArmControlGit arm;
LiquidCrystal_I2C lcd(0x27, 20, 4);//Change depending on the size

//Pin Assignments
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
#define UpButton 7
#define DownButton 8
#define BackButton 2
#define EnterButton 12
#define LedPin 13

#define ButtonDelay 300
#define blockPickHeight 20
#define blockDropHeight 30
#define armHeight 140

#define maxAllowedWrites 1000//for EEPROMex
#define memBase 350

#define Distance_Offset 20

long timer = 0;
int y_arrow = 0;
byte screen;
byte deepness = 0;
bool back, enter = false;
int CC, CO; //for calibrating the claw on the spot
int Rr, Rg, Rb, Gr, Gg, Gb, Br, Bg, Bb, Kr, Kg, Kb;
int r, g, b;
byte closestDegree; 
int closestDistance;
bool clawOpen = true;
bool over = false;
bool working = false;
bool manually = false;
long lastButtonMillis;
bool bannedDegree[46];


uint8_t blockCount[3] = {0,0,0};

#define rO 0 //Red offset - the rightmost side of the red area
#define gO 45
#define bO 90
#define sO 0 //squeeze offset - if any blocks are touching the edges of the area, make this bigger (a few degrees). it will squeeze the blocks closer together
#define bD1 210
#define bD2 170
#define bD3 130
#define bPD1 3
#define bPD2 2
#define bPD3 1
uint8_t rP[bPD1 + bPD2 + bPD3][2]; //2 being distance, then angle
uint8_t gP[bPD1 + bPD2 + bPD3][2];
uint8_t bP[bPD1 + bPD2 + bPD3][2];
uint8_t kP[bPD1 + bPD2 + bPD3][2];//final black positions
uint8_t kTP[bPD1 + bPD2 + bPD3][2]; //temporary black positions


void setup() {
  
  //setting up the places to put the blocks
  for(uint8_t i = 0; i < bPD1; i++){
    rP[i][0] = bD1;
    rP[i][1] = rO + (sO/2) + (0.5 + i)*((45-sO)/bPD1);
    gP[i][0] = bD1;
    gP[i][1] = gO + (sO/2) +(0.5 + i)*((45-sO)/bPD1);
    bP[i][0] = bD1;
    bP[i][1] = bO + (sO/2) +(0.5 + i)*((45-sO)/bPD1);
  }
  for(uint8_t i = 0; i < bPD2; i++){
    rP[i+bPD1][0] = bD2;
    rP[i+bPD1][1] = rO + (sO/2) +(0.5 + i)*((45-sO)/bPD2);
    gP[i+bPD1][0] = bD2;
    gP[i+bPD1][1] = gO + (sO/2) +(0.5 + i)*((45-sO)/bPD2);
    bP[i+bPD1][0] = bD2;
    bP[i+bPD1][1] = bO + (sO/2) +(0.5 + i)*((45-sO)/bPD2);
  }
  for(uint8_t i = 0; i < bPD3; i++){
    rP[i+bPD1+bPD2][0] = bD3;
    rP[i+bPD1+bPD2][1] = rO + (sO/2) +(0.5 + i)*((45-sO)/bPD3);
    gP[i+bPD1+bPD2][0] = bD3;
    gP[i+bPD1+bPD2][1] = gO + (sO/2) +(0.5 + i)*((45-sO)/bPD3);
    bP[i+bPD1+bPD2][0] = bD3;
    bP[i+bPD1+bPD2][1] = bO + (sO/2) +(0.5 + i)*((45-sO)/bPD3);
  }
  
  //EEPROM setup
  EEPROM.setMemPool(memBase, EEPROMSizeUno);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  
  Serial.begin(115200);
  arm.beginArm(basePin, shoulderPin, elbowPin, gripperPin);
  delay(500);
  Wire.begin();
  
  //Laser distance sensor
  sensor.init();
  #ifdef Sensor_VL6180X
  sensor.configureDefault();
  #endif
  sensor.setTimeout(500);
  sensor.startContinuous(20);
  
  resetServos();
  
  //Color Sensor
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  //High Power LED
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, LOW);
  
  //Manual Control
  pinMode(basePot, INPUT);
  pinMode(distancePot, INPUT);
  pinMode(heightPot, INPUT);
  pinMode(ClawSwitch, INPUT_PULLUP);
  
  //Buttons for Menu Control
  pinMode(BackButton, INPUT_PULLUP);
  pinMode(UpButton, INPUT_PULLUP);
  pinMode(DownButton, INPUT_PULLUP);
  pinMode(EnterButton, INPUT_PULLUP);
  lastButtonMillis = millis();
  
  timer = millis();
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


//or we can check everything as a multiple of another. for g - b = xg, r = xg

char small = 'k';
//pick the smallest value 
if(r <= (g-20) && r <= (b-30))
  small = 'r';
else if (g <= r && g <= (b-5))
  small = 'g';
else if (b <= g && b <= r)
  small = 'b';
if(r > 100 && g > 100 && b > 100) //the 100 values might need to be changed.
  small = 'k';
  return small;
  
  
  /*
  if (r < Gr && r < Br){
    return 'r';
  }
  else if (g < Rg && g < Bg){
    return 'g';
  }
  else if (b < Rb && b < Gb){
    return 'b';
  }
  else{
    return 'k';
  }*/
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
  EEPROM.updateInt(350,r);
  EEPROM.updateInt(352,g);
  EEPROM.updateInt(354,b);
  readEEPROM();
}
void calibrateGreen(){
  readColor();
  EEPROM.updateInt(356,r);
  EEPROM.updateInt(358,g);
  EEPROM.updateInt(360,b);
  readEEPROM();
}
void calibrateBlue(){
  readColor();
  EEPROM.updateInt(362,r);
  EEPROM.updateInt(364,g);
  EEPROM.updateInt(366,b);
  readEEPROM();
}
void calibrateBlack(){
  readColor();
  EEPROM.updateInt(368,r);
  EEPROM.updateInt(370,g);
  EEPROM.updateInt(372,b);
  readEEPROM();
}
void readEEPROM(){
  Rr = EEPROM.readInt(350);
  Rg = EEPROM.readInt(352);
  Rb = EEPROM.readInt(354);
  Gr = EEPROM.readInt(356);
  Gg = EEPROM.readInt(358);
  Gb = EEPROM.readInt(360);
  Br = EEPROM.readInt(362);
  Bg = EEPROM.readInt(364);
  Bb = EEPROM.readInt(366);
  Kr = EEPROM.readInt(368);
  Kg = EEPROM.readInt(370);
  Kb = EEPROM.readInt(372);
}
void resetServos(){
  //arm.moveBaseServo(90);
  //arm.moveGripperServo(armHeight);
  arm.moveArm(150, 140, 90);
  //arm.moveShoulderServo(armHeight);
  //arm.moveElbowServo(armHeight);
  arm.openClaw();
}
int checkDistance(){
  //read 5 values, if any are over 1000 then nothing there.
  int range = 0; //= sensor.readRangeContinuousMillimeters();
  //range = sensor.readRangeSingleMillimeters();
  //if(!sensor.timeoutOccurred()){
    //return range;
  //}
  //return 3000;
  int rangeArr[5] = {0,0,0,0,0}; //fill array with sensor readings
  for(int i = 0; i < 5; i++){
    //range += sensor.readRangeSingleMillimeters();
    range += sensor.readRangeContinuousMillimeters();
  #ifdef Sensor_VL53L0X
    rangeArr[i] = range;
  #endif
  delay(20);
  }
  #ifdef Sensor_VL53L0X
  //range = 0;
  
  /*for(int i = 0; i < 5; i++){
    if(rangeArr[i] > 400 || rangeArr[i] < 30){
      range = 2000;
      
      //rangeArr[i] = 3000;
    }
    //range += rangeArr[i];
  }*/
  
    //median of array
    range = min(min(min(rangeArr[0], rangeArr[1]),min(rangeArr[2], rangeArr[3])),rangeArr[4]);
    //range = rangeArr[3];
  
  int count = 0;
  for(int i = 0; i < 5; i++){
    if(rangeArr[i] > 1000){
      count++;
    }
  }
  if(count >= 2){
    return 8888;
  }
  if(range > 250){
	  return 7777;
  }
  return range;
  #endif
  range /= 3;

  #ifdef Sensor_VL6180X
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
  #endif
  
  return(range);
}
void radar(){
  int dist;
  bool degreeBanned = false;
  closestDistance = 700;
  arm.moveArm(armHeight, 70 , 135);
  delay(300);
  for (int v = 135; v <= 180; v+= 3){//135 and 180 could be switched to defines if that would be useful.
    arm.moveBaseServo(v);
    dist = checkDistance();
    for(int g = v - 2; g <= v+2; g++){
      if (v >= 137 && v <= 178 && bannedDegree[180-g])
        degreeBanned = true;
    }
    if ((dist < closestDistance) && (dist < 300) && !degreeBanned){
      closestDistance = dist;
      closestDegree = v;
    }
    degreeBanned = false;
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
                lcd.print(EEPROM.readInt(350));
                lcd.setCursor(0,1);
                lcd.print(F("Rg:"));
                lcd.print(EEPROM.readInt(352));
                lcd.setCursor(0,2);
                lcd.print(F("Rb:"));
                lcd.print(EEPROM.readInt(354));
                lcd.setCursor(7,0);
                lcd.print(F("Gr:"));
                lcd.print(EEPROM.readInt(356));
                lcd.setCursor(7,1);
                lcd.print(F("Gg:"));
                lcd.print(EEPROM.readInt(358));
                lcd.setCursor(7,2);
                lcd.print(F("Gb:"));
                lcd.print(EEPROM.readInt(360));
                lcd.setCursor(14,0);
                lcd.print(F("Br:"));
                lcd.print(EEPROM.readInt(362));
                lcd.setCursor(14,1);
                lcd.print(F("Bg:"));
                lcd.print(EEPROM.readInt(364));
                lcd.setCursor(14,2);
                lcd.print(F("Bb:"));
                lcd.print(EEPROM.readInt(366));
                lcd.setCursor(0,3);
                lcd.print(F("Kr:"));
                lcd.print(EEPROM.readInt(368));
                lcd.setCursor(7,3);
                lcd.print(F("Kg:"));
                lcd.print(EEPROM.readInt(370));
                lcd.setCursor(14,3);
                lcd.print(F("Kb:"));
                lcd.print(EEPROM.readInt(372));
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
                blockCount[0] = 0;
        blockCount[1] = 0;
        blockCount[2] = 0; //reset the block count.
        do {
          pick();
          if (over){
            break;
          }
          leave();
                } while (!over);
                over = false;
        resetServos();
        Serial.println("BlockCount:");
        Serial.println(blockCount[0]);
        Serial.println(blockCount[1]);
        Serial.println(blockCount[2]);
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
     arm.openClaw();
     clawOpen = true;
     arm.moveArm(armHeight, 50 , closestDegree);
	 delay(500);
	 //arm.moveArm((60), 40, closestDegree);
	 arm.moveArm(60,50,closestDegree);
     delay(500);
     arm.moveArm(blockPickHeight, closestDistance + Distance_Offset, closestDegree);
     delay(700);
     arm.closeClaw();
     clawOpen = false;
     delay(500);
     //Serial.println(checkColor());
     //arm.moveArm(armHeight,closestDistance + Distance_Offset, closestDegree );
	 arm.moveArm(armHeight,100, closestDegree );
	 delay(500);//move back out of the way of the blocks
  }
}
void leave(){
  //the block is already in the claw
  int BlockDegree;
  int BlockDistance;
  switch(checkColor()){
    case 'k':
      BlockDegree = closestDegree; //leave it where it is (for now) UPDATE
      BlockDistance = 130;
      bannedDegree[180-closestDegree] = true;
      break;
    case 'g':
      //BlockDegree = 100;
    BlockDistance = gP[blockCount[1]][0];
    BlockDegree = gP[blockCount[1]][1];
    blockCount[1]++;
      break;
    case 'b':
      //BlockDegree = 70;
    BlockDistance = bP[blockCount[2]][0];
    BlockDegree = bP[blockCount[2]][1];
    blockCount[2]++;
      break;
    case 'r':
      //BlockDegree = 20;
    BlockDistance = rP[blockCount[0]][0];
    BlockDegree = rP[blockCount[0]][1];
    blockCount[0]++;
      break;
  }
  delay(500);
         arm.moveArm(armHeight, 100, BlockDegree);
		 delay(500);
		 //arm.moveArm(armHeight,armHeight,BlockDegree);//move to area
         //delay(500);
         //arm.moveArm(blockDropHeight,armHeight,BlockDegree);//move down to the table - armHeight is being used as distance
		 arm.moveArm(blockDropHeight, BlockDistance, BlockDegree);
         delay(400);
         arm.releaseClaw();//release the block
         delay(300);
         arm.moveArm(armHeight,100,BlockDegree);//move back out of the way
		 arm.openClaw(); //open the claw fully
		 clawOpen = true;
         delay(400);//all the delays might not be needed.
         resetServos();//why reset the servos? UPDATE
}
void manualMove(){
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
    if (millis() - timer > 1000){
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
