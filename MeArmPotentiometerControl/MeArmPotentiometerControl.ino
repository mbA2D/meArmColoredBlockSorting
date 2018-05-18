#define baseManualPot A0
#define distanceManualPot A1
#define heightManualPot A2
#define ClawSwitch A3

#include "meArmControlGit.h"
#include <Servo.h>

#define baseManualPin 6
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
int heightManual = 0;
int baseManual = 0;
int distanceManual = 0;
bool claw = OPEN;

void setup() {
  // put your setup code here, to run once:
pinMode(baseManualPot, INPUT);
pinMode(distanceManualPot, INPUT);
pinMode(heightManualPot, INPUT);
pinMode(ClawSwitch, INPUT_PULLUP);

  arm.beginArm(baseManualPin, shoulderPin, elbowPin, gripperPin);
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

  HPV = analogRead(heightManualPot);
  BPV = analogRead(baseManualPot);
  DPV = analogRead(distanceManualPot);

  heightManual = map(HPV, 0, 1023, 0, 160);
   baseManual = map(BPV, 0, 1023, 0, 180); 
   distanceManual = map(DPV, 0, 1023, 0, 160);

  

  if(HPV != oldHPV){
    
  }
  if(BPV != oldBPV){
    
  }
  if(DPV != oldBPV){
    
  }

  if(canReach){
    arm.moveArm(heightManual, distanceManual, baseManual);
  }

   
}

bool canReach(){
  return(sqrt(sq(heightManual) + sq(baseManual)) >= 160);
}

