#define BasePot A0
#define DistancePot A1
#define HeightPot A2
#define ClawSwitch A3

#include "meArmControlGit.h"
#include <Servo.h>

#define basePin 6
#define shoulderPin 10
#define elbowPin 9
#define gripperPin 11

meArmControlGit arm;

#define CLOSED false
#define OPEN true

int HPV = 0;
int BPV = 0;
int DPV = 0;
int oldBPV = 0;
int oldHPV = 0;
int oldDPV = 0;
int height = 0;
int base = 0;
int distance = 0;
bool claw = OPEN;

void setup() {
  // put your setup code here, to run once:
pinMode(BasePot, INPUT);
pinMode(DistancePot, INPUT);
pinMode(HeightPot, INPUT);
pinMode(ClawSwitch, INPUT_PULLUP);

  arm.beginArm(basePin, shoulderPin, elbowPin, gripperPin);
  delay(500);
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!digitalRead(ClawSwitch))
    arm.openClaw();
  else
    arm.closeClaw();

  oldHPV = HPV;
  oldBPV = BPV;
  oldDPV = DPV;

  HPV = analogRead(HeightPot);
  BPV = analogRead(BasePot);
  DPV = analogRead(DistancePot);

  height = map(HPV, 0, 1023, 0, 160);
   base = map(BPV, 0, 1023, 0, 180); 
   distance = map(DPV, 0, 1023, 0, 160);

  

  if(HPV != oldHPV){
    
  }
  if(BPV != oldBPV){
    
  }
  if(DPV != oldBPV){
    
  }

  if(canReach){
    arm.moveArm(height, distance, base);
  }

   
}

bool canReach(){
  return(sqrt(sq(height) + sq(base)) >= 160);
}

