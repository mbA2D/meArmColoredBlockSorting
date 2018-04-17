//I am doing this - TODO - slow down height and distance movements.


//DONE TOO many global variable - 69 bytes too much.
//Use print(F("")) to store strings in program memory instead of SRAM.
//for the color sensor - if we read a 255 value - there is a problem, it is not the right thing - memory overflow?
//next step - get servo calibration on the LCD.
//during calibration, get RGB values on the bottom of the screen?
//why is 100 the middle of the base?
//get BASE_45, 90, 135 values during calibration and save them - to EEPROM.

//DONE SERVOS do not work after the color sensor is used
    //not using too much ram - still 1075 
    //confirmed that it is after the color sensor
    //does freqcount or anything else use timer 2 or  (timer0 is pin 6)
    //freqcount uses timers 1 and 2 (pins 3, 9, 10, 11 - all servos)
    //test - put base on 6, see if it still works.
    //servo uses timer1.
    //FIXED - using a mega and modified the servo library to use timer 3.
    
//now can calibrate the servos as well.
//when calibrating shoulder and elbow servos, detach the other one.
//how far back does the shoulder servo really need to go?
    //find the angle and calibrate the uS value, then use that.
    //to go all the way back, you need to remove some screws.
    
//DONE get a sweep 45 function - find blocks.
  //ultrasonic to mm
    //get uS ping
    //convert to mm
  //go to smallest distance angle
  
  //go out to distance, BLOCK_GRAB_HEIGHT:45
  
    
    
//#define ARDUINO_UNO
//#define ARDUINO_MEGA

/*
#ifdef ARDUINO_UNO
//uno pins
#endif

#ifdef ARDUINO_MEGA
//mega pins
#endif
*/

#define DEBUG 1 //print out to serial - this should probably be read from a switch instead.

//******************LCD SETUP / LIBRARIES*********************
#include <Wire.h>
//#include <LiquidCrystal_SI2C.h>
//LiquidCrystal_SI2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#include <LiquidCrystal_I2C.h> //SDA - A4, SCL - A5
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#include <Servo.h>
#include <math.h>
#include <EEPROMex.h>
#include <MD_TCS230.h>
#include <FreqCount.h>
#include <RunningMedian.h>
//#include <NewPing.h> //__vector_13 conflict with FreqCount NewPing.cpp ln 334 - don't use NewPing, just do it in a function.


#define SMALL_DELAY 200
#define VERY_SMALL_DELAY 75


#define ANGLE_OFFSET 8
#define DISTANCE_OFFSET 0

#define DONE_HEIGHT 90 //mm once a block is placed, go to this height to avoid knocking it over.
#define DONE_DISTANCE 110
#define GRAB_RELEASE_HEIGHT 30 //mm
#define TRAVEL_HEIGHT 45 //bring it a bit off the ground
int ULTRA_OFFSET; // - end of transducers to the edge of the base
int MinDist;
int MinDistAng;


int UCAL_5, UCAL_10, UCAL_15, UCAL_20, UCAL_25;


//**********************LCD********************
//LCD size and cursor position
#define LCD_Chars 20
#define LCD_Rows 4
uint8_t CursorChar = 0;
uint8_t CursorRow = 0;

//LCD Positioning
#define Base_Row 0
#define Height_Row 1 //shoulder
#define Distance_Row 2 //elbpw
#define Claw_Row 3
#define PositionVal_Col 1 //ServoVal_Col
#define PositionNam_Col 0 //ServoNam_Col

#define CALIBRATE_CHAR 17
#define CALIBRATE_ROW 0
#define CALIBRATE_TEXT (F("CCo"))
#define FINDCOLOR_CHAR 17
#define FINDCOLOR_ROW 1
#define FINDCOLOR_TEXT (F("FCo"))
#define DISTANCE_CHAR 17
#define DISTANCE_ROW 2
#define DISTANCE_TEXT (F("Dis"))
#define CALISERVO_CHAR 17
#define CALISERVO_ROW 3
#define CALISERVO_TEXT (F("CSe"))
#define CALTEMP_CHAR 15
#define CALTEMP_ROW 0
#define CALTEMP_TEXT (F("T"))

#define CALR_CHAR 9
#define CALR_ROW 3
#define CALR_TEXT (F("R"))
#define CALG_CHAR 10
#define CALG_ROW 3
#define CALG_TEXT (F("G"))
#define CALB_CHAR 11
#define CALB_ROW 3
#define CALB_TEXT (F("B"))

#define GOTOR_CHAR 9
#define GOTOR_ROW 2
#define GOTOR_TEXT (F("R"))
#define GOTOG_CHAR 10
#define GOTOG_ROW 2
#define GOTOG_TEXT (F("G"))
#define GOTOB_CHAR 11
#define GOTOB_ROW 2
#define GOTOB_TEXT (F("B"))

#define SWEEP_CHAR 13
#define SWEEP_ROW 3
#define SWEEP_TEXT (F("SW"))

#define UOFF_CHAR 14
#define UOFF_ROW 0
#define UOFF_TEXT (F("U"))

#define UCAL_CHAR 13
#define UCAL_ROW 0
#define UCAL_TEXT (F("C"))

#define SORT_CHAR 6
#define SORT_ROW 0
#define SORT_TEXT (F("SO"))

#define MIN_DIST_CHAR 5
#define MIN_DIST_ROW 1
#define MIN_DIST_TEXT (F("D"))

#define MIN_ANG_CHAR 9
#define MIN_ANG_ROW 1
#define MIN_ANG_TEXT (F("A"))

#define FINDMDA_CHAR 9
#define FINDMDA_ROW 0
#define FINDMDA_TEXT (F("F"))

#define GOTOMDA_CHAR 10
#define GOTOMDA_ROW 0
#define GOTOMDA_TEXT (F("G"))

#define MOVEBLOCK_CHAR 11
#define MOVEBLOCK_ROW 0
#define MOVEBLOCK_TEXT (F("M"))


/*
#ifdef ARUINO_UNO
    //control board switches (pins)
    //#define Enable_Sw 18 //A4 - do not use these for i2c, soft serial on 12SDA,13SCL
    //#define Left_Sw 19 //A5
    #define Enable_Sw 12 //D12
    #define Left_Sw 8 //D13 //testing on pin 8 for now.

    #define Right_Sw 15 //A1
    #define Center_Sw 16 //A2
    #define Bottom_Sw 14 //A0
    #define Top_Sw 17 //A3
#endif
*/
//#ifdef ARDUINO_MEGA
    #define Enable_Sw 12 //D12
    #define Left_Sw 9 //D13 //testing on pin 9 for now.

    #define Right_Sw 55 //A1
    #define Center_Sw 56 //A2
    #define Bottom_Sw 54 //A0
    #define Top_Sw 57 //A3
//#endif

//for button pushes in menu
#define LeftButton 0
#define TopButton 1
#define CenterButton 2
#define BottomButton 3
#define RightButton 4
#define NoButton 5

//debounce time delay for button input (non interrupt)
#define Button_Delay 150
//these are used for software debouncing
long lastButton = 0; //the time that the last button was pushed
long lastInterrupt = 0; //time that the interrupt was last triggered

//interrupt to enter menu
#define interruptPin 2 //top left corner on board
volatile byte inMenu = LOW;
bool lastMenu = LOW;
bool LCD_Enabled;

#define UP 1
#define DOWN (-1) //instead of using these numbers to call increment servo

#define SERVO_LCD_MIN 0//
#define SERVO_LCD_MAX 180//
        
#define MAX_HEIGHT 150//
#define MIN_HEIGHT 20//
#define MAX_DISTANCE 200//
#define MIN_DISTANCE (CLAW_OFFSET_X)//

#define MAX_CLAW SERVO_LCD_MAX //these are all just for the LCD
#define MIN_CLAW SERVO_LCD_MIN//

#define MAX_BASE SERVO_LCD_MAX //
#define MIN_BASE SERVO_LCD_MIN//
        

//values for calibrating from serial or from LCD
#define FromLCD 0
#define FromSERIAL 1


//*********************ULTRASONIC****************
//pins must be changed - no, they don't
#define TRIGGER_PIN  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 1000 //mm // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm
#define MAX_PLAY_DISTANCE 350 //mm (350 cm) - too big, but better to be big.
#define SIGNAL_PIN 10 //for parallax ping


//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define SPEED_OF_SOUND_BASE 331.3 //meters/second
//#define AMBIENT_TEMP 23 //degreeds C //possibly read from a color sensor? //temp read from EEPROM set by LCD?
#define SOUND_TEMP_MULTIPLIER 0.606

uint8_t AMBIENT_TEMP; //uint8_t - EEPROM.readByte


//******************************DEFINES FOR HD*************************

//should have an update on change value to avoid extra lcd time.
bool CLAW_CHANGE = false;
bool HEIGHT_CHANGE = false;
bool DISTANCE_CHANGE = false;
bool BASE_CHANGE = false;

/*
//CALIBRATION VALUES for mearm - just used for mapping the angle
//School's acrylic - still sticking
#define SHOULDER_MIN_VALUE 2400 //when shoulder servo is as far forward as it can be
#define SHOULDER_MAX_VALUE 600 //when shoulder servo is as far backwards as it can be
#define ELBOW_MIN_VALUE 1800 //when elbow servo arm is at -30 degrees to X axis
#define ELBOW_MAX_VALUE 600 //when elbow servo arm is at 90 degress to X axis

//My wooden mearm

//#define SHOULDER_MIN_VALUE 2400 //when shoulder servo is as far forward as it can be
//#define SHOULDER_MAX_VALUE 600 //when shoulder servo is as far backwards as it can be
//#define ELBOW_MIN_VALUE 1800 //when elbow servo arm is at -30 degrees to X axis
//#define ELBOW_MAX_VALUE 600 //when elbow servo arm is at 90 degress to X axis

//END OF CALIBRATION for mearm
*/

#define START_BASE 90 //deg If this is changed, change the currentPos in the setup (roughly ln 445)
#define START_CLAW 90 //deg

#define START_HEIGHT 50 //mm
#define START_DISTANCE 100 //mm

//Claw Offset
//#define CLAW_OFFSET_X 70 //mm
#define CLAW_OFFSET_X 67 //mm
#define CLAW_OFFSET_Y (-15) //mm bigger to account for 'play' in the base servo
//#define CLAW_OFFSET_Y (-5) //school acrylic - less 'play' in the base servo

//Shoulder Offset
#define SHOULDER_X (-17) //mm  point of rotation offset from the point measured from
#define SHOULDER_Y 53 //mm

//Joint Lengths
#define SHOULDER_LENGTH 80 //mm  section from shoulder to elbow
#define ELBOW_LENGTH 80 //mm  section from elbow to claw

//Elbow degrees positions
#define ELBOW_MIN_POSITION (-30) //relative to X
#define ELBOW_MAX_POSITION 90 //90 degrees relative to X (straight up)

//Shoulder degrees positions
#define SHOULDER_MIN_POSITION 0 //relative to X - all the way forwards
#define SHOULDER_MAX_POSITION 179 //all the way backwards

//value of pi
#define pi 3.14159

//***********************************END OF HD DEFINES***********************




//SERVO CALIBRATION VALUES
#define CLAW_US_MIN 800
#define CLAW_US_MAX 2000
//#define ELBOW_US_MIN
//#define ELBOW_US_MAX
//#define BASE_US_MIN 550
//#define BASE_US_MAX 2320


int ElbowServoAngle;
int ShoulderServoAngle;

Servo BaseServo;
Servo ClawServo;
Servo ElbowServo;
Servo ShoulderServo;

//Servo pins - must all be PWM
#define BASE_PIN 8
//#define SHOULDER_PIN 10
//#define ELBOW_PIN 9
#define SHOULDER_PIN 6 //for mega
#define ELBOW_PIN 5 //for mega

#define CLAW_PIN 3 //usually 6 - changed to 3 to not interfere with timer1

//Servo values
int     Base_Value = START_BASE, 
        Height_Value = START_HEIGHT,  //shoulder
        Distance_Value = START_DISTANCE,  //distance
        Claw_Value = START_CLAW; //defalt configuration for testing purposes



//****************************************COLOR SENSOR*******************
// Pin definitions
//D5 is always the input pin.
#define  S2_OUT  4 //usually 6
#define  S3_OUT  7 //done

#define	BLACK_CAL	0
#define	WHITE_CAL	1
#define	READ_VAL	2

//EEPROM addresses for color min and max
//350-374 are used for white calibration
//int - 2 long - 4
#define R_Rmin_Add 375
#define R_Gmin_Add 377
#define R_Bmin_Add 379
#define R_Rmax_Add 381
#define R_Gmax_Add 383
#define R_Bmax_Add 385

#define G_Rmin_Add 387
#define G_Gmin_Add 389
#define G_Bmin_Add 391
#define G_Rmax_Add 393
#define G_Gmax_Add 395
#define G_Bmax_Add 397

#define B_Rmin_Add 399
#define B_Gmin_Add 401
#define B_Bmin_Add 403
#define B_Rmax_Add 405
#define B_Gmax_Add 407
#define B_Bmax_Add 409

//#define interruptPin 2 //interruptPin to calibrateWB on the color sensor.

//bool DEBUG = true; //print out more things to serial or not

#define maxAllowedWrites 80//for EEPROMex
#define memBase 350

bool JustRead = false; //for color sensor - the readSensor function needs this

bool CalibrateNow = LOW; //to calibrate with button push

static uint8_t	runState = 0; //for color sensor		
static uint8_t	readState = 0;


uint8_t R = 255, G = 255, B = 255; //values for RGB color testing data - added by me These should get put after EEPROM is initialised
uint8_t     R_Rmax, R_Gmax, R_Bmax, //make the sensor learn these values from samples of colors.
		R_Rmin, R_Gmin, R_Bmin, //R,G,B values all get changed with calibration.
		G_Rmax, G_Gmax, G_Bmax,
		G_Rmin, G_Gmin, G_Bmin,
		B_Rmax, B_Gmax, B_Bmax, 
		B_Rmin, B_Gmin, B_Bmin,
		K_Rmax = 30, K_Gmax = 30, K_Bmax = 30, //black not needed - only need to move block if it is one of the other colors.
		K_Rmin = 0, K_Gmin = 0, K_Bmin = 0; 
		//if there is a block, and it is not red, green, or blue, then move it out of the way.


MD_TCS230  CS(S2_OUT, S3_OUT); //declare color sensor object

//*****************************END OF COLOR SENSOR**************************


//*******************SERVO ACCEL*******************



//#define SERVO_0 2400 //uS
//#define SERVO_180 640 //uS
//#define BASE_0 BASE_US_MAX //uS
//#define BASE_180 BASE_US_MIN //uS

#define MAX_SPEED 600
#define MAX_REFERENCE 1000
#define ACCEL 3 //slope of velocity graph - make it steeper?

int TargetPos, CurrentPos;
//int pos = SERVO_0;



//****************END OF SERVO ACCEL****************

//****************BASE SERVO CALIBRATION***********
//integers in EEPROM (take up 2 bytes)
#define BASE_0_ADD 411
#define BASE_45_ADD 413
#define BASE_90_ADD 415
#define BASE_135_ADD 417
#define BASE_180_ADD 419

#define SHOULDER_MIN_VALUE_ADD 421 //when shoulder servo is as far forward as it can be
#define SHOULDER_MAX_VALUE_ADD 423 //when shoulder servo is as far backwards as it can be

#define ELBOW_MIN_VALUE_ADD 425 //when elbow servo arm is at -30 degrees to X axis
#define ELBOW_MAX_VALUE_ADD 427 //when elbow servo arm is at 90 degress to X axis

#define CLAW_GRAB_ADD 429 //really grab and open are the only ones needed.
#define CLAW_OPEN_ADD 431
#define CLAW_RELEASE_ADD 433
#define CLAW_PREPARE_ADD 435

//TEMP ADDRESS
#define AMBIENT_TEMP_ADD 437

#define R_BASE_ADD 438
#define R_DIST_ADD 439
#define G_BASE_ADD 440
#define G_DIST_ADD 441
#define B_BASE_ADD 442
#define B_DIST_ADD 443

#define ULTRA_OFFSET_ADD 444 //this is int

#define UCAL_5_ADD 446
#define UCAL_10_ADD 448
#define UCAL_15_ADD 450
#define UCAL_20_ADD 452
#define UCAL_25_ADD 454

/*
#define BASE_45 45 //deg
#define BASE_90 90 // deg
#define BASE_135 135 //deg
*/
int BASE_0, BASE_45, BASE_90, BASE_135, BASE_180, //will all be in uS
    SHOULDER_MIN_VALUE, SHOULDER_MAX_VALUE,
    ELBOW_MIN_VALUE, ELBOW_MAX_VALUE,
    CLAW_GRAB, CLAW_OPEN, CLAW_RELEASE, CLAW_PREPARE;
uint8_t R_BASE, R_DIST, G_BASE, G_DIST, B_BASE, B_DIST;

//*******************END OF BASE SERVO CALIBRATION***

#define ALL_SERVO_MAX_US 2500
#define ALL_SERVO_MIN_US 500 //to attach servos.


//*************************SETUP**********************************

void setup(){
    Serial.begin(9600);
    if(DEBUG){
        Serial.println(F("LCD HD control of meArm"));
        Serial.println(F(""));
    }
    
    //Servo setup
    ElbowServo.attach(ELBOW_PIN, ALL_SERVO_MIN_US, ALL_SERVO_MAX_US); //these extra conditions just constrain the pulse width.
    ShoulderServo.attach(SHOULDER_PIN, ALL_SERVO_MIN_US, ALL_SERVO_MAX_US);
    BaseServo.attach(BASE_PIN, ALL_SERVO_MIN_US, ALL_SERVO_MAX_US);
    ClawServo.attach(CLAW_PIN, ALL_SERVO_MIN_US, ALL_SERVO_MAX_US);
    
    pinMode(BASE_PIN, OUTPUT);
    pinMode(SHOULDER_PIN, OUTPUT);
    pinMode(ELBOW_PIN, OUTPUT);
    pinMode(CLAW_PIN, OUTPUT);
  
    //control board setup
    pinMode(Enable_Sw, INPUT_PULLUP); //with Input Pullup, must use inverted logic
    pinMode(Left_Sw, INPUT_PULLUP);
    pinMode(Right_Sw, INPUT_PULLUP);
    pinMode(Center_Sw, INPUT_PULLUP);
    pinMode(Bottom_Sw, INPUT_PULLUP);
    pinMode(Top_Sw, INPUT_PULLUP);

    
    
    
    //LCD Setup
    LCD_Enabled = !(digitalRead(Enable_Sw)); //inverted logic with pullup resistors.
    if (DEBUG){
        Serial.print(F("LCD Enabled: "));
        Serial.println(LCD_Enabled);
    }
      if (LCD_Enabled){
        LCDSetup();
        //printLCDTime(); //function to print the running time on the LCD
        LCDinfoSetup(); //This must be called in the setup.
    }
  
  //interrupt is working now - use a debounce capacitor or similar software method(implemented)
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), menuState, FALLING);
  
  //*******COLOR SENSOR SETUP*******
    //Serial.begin(57600);
   
  //EEPROM.setMemPool(memBase, EEPROMSizeUno); //EEPROM Setup
  EEPROM.setMemPool(memBase, EEPROMSizeMega);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  
  CS.begin(); //Color Sensor Setup
  
  R_Rmax = EEPROM.readInt(R_Rmax_Add), R_Gmax = EEPROM.readInt(R_Gmax_Add), R_Bmax = EEPROM.readInt(R_Bmax_Add), //make the sensor learn these values from samples of colors.
		R_Rmin = EEPROM.readInt(R_Rmin_Add), R_Gmin = EEPROM.readInt(R_Gmin_Add), R_Bmin = EEPROM.readInt(R_Bmin_Add), //R,G,B values all get changed with calibration.
		G_Rmax = EEPROM.readInt(G_Rmax_Add), G_Gmax = EEPROM.readInt(G_Gmax_Add), G_Bmax = EEPROM.readInt(G_Bmax_Add),
		G_Rmin = EEPROM.readInt(G_Rmin_Add), G_Gmin = EEPROM.readInt(G_Gmin_Add), G_Bmin = EEPROM.readInt(G_Bmin_Add),
		B_Rmax = EEPROM.readInt(B_Rmax_Add), B_Gmax = EEPROM.readInt(B_Gmax_Add), B_Bmax = EEPROM.readInt(B_Bmax_Add), 
		B_Rmin = EEPROM.readInt(B_Rmin_Add), B_Gmin = EEPROM.readInt(B_Gmin_Add), B_Bmin = EEPROM.readInt(B_Bmin_Add),
		K_Rmax = 30, K_Gmax = 30, K_Bmax = 30, //black not needed - only need to move block if it is one of the other colors.
		K_Rmin = 0, K_Gmin = 0, K_Bmin = 0; //if there is a block, and it is not red, green, or blue, then move it out of the way.
  
  //calibrateColor(true, true, FromSERIAL); //this needs to be commented out later.
  //******END OF COLOR SENSOR SETUP*********
  
  //Servo Calibration Setup*********
  BASE_0 = EEPROM.readInt(BASE_0_ADD); //these will all be in uS
  BASE_45 = EEPROM.readInt(BASE_45_ADD);
  BASE_90 = EEPROM.readInt(BASE_90_ADD);
  BASE_135 = EEPROM.readInt(BASE_135_ADD);
  BASE_180 = EEPROM.readInt(BASE_180_ADD);
  
  ELBOW_MIN_VALUE = EEPROM.readInt(ELBOW_MIN_VALUE_ADD);
  ELBOW_MAX_VALUE = EEPROM.readInt(ELBOW_MAX_VALUE_ADD);
  
  SHOULDER_MIN_VALUE = EEPROM.readInt(SHOULDER_MIN_VALUE_ADD);
  SHOULDER_MAX_VALUE = EEPROM.readInt(SHOULDER_MAX_VALUE_ADD);
  
  CLAW_GRAB = EEPROM.readInt(CLAW_GRAB_ADD);
  CLAW_OPEN = EEPROM.readInt(CLAW_OPEN_ADD);
  CLAW_RELEASE = EEPROM.readInt(CLAW_RELEASE_ADD);
  CLAW_PREPARE = EEPROM.readInt(CLAW_PREPARE_ADD);
  
  AMBIENT_TEMP = EEPROM.readByte(AMBIENT_TEMP_ADD);
  if((AMBIENT_TEMP > 40) || (AMBIENT_TEMP < 10))
    AMBIENT_TEMP = 22;
  
  ULTRA_OFFSET = EEPROM.readInt(ULTRA_OFFSET_ADD);
  if((ULTRA_OFFSET > 99) || (ULTRA_OFFSET < 0))
    ULTRA_OFFSET = 16;
  
  R_BASE = EEPROM.readByte(R_BASE_ADD);
  R_DIST = EEPROM.readByte(R_DIST_ADD);
  G_BASE = EEPROM.readByte(G_BASE_ADD);
  G_DIST = EEPROM.readByte(G_DIST_ADD);
  B_BASE = EEPROM.readByte(B_BASE_ADD);
  B_DIST = EEPROM.readByte(B_DIST_ADD);
  
  UCAL_5 = EEPROM.readInt(UCAL_5_ADD);
  UCAL_10 = EEPROM.readInt(UCAL_10_ADD);
  UCAL_25 = EEPROM.readInt(UCAL_15_ADD);
  UCAL_20 = EEPROM.readInt(UCAL_20_ADD);
  UCAL_25 = EEPROM.readInt(UCAL_25_ADD);
  
  //block placement calibration should be changed to a 2D array to hold multiple blocks.
  
  
  if(DEBUG){
    Serial.println(F(""));
    Serial.print(F("BASE_0: "));
    Serial.println(BASE_0);
    Serial.print(F("BASE_45: "));
    Serial.println(BASE_45);
    Serial.print(F("BASE_90: "));
    Serial.println(BASE_90);
    Serial.print(F("BASE_135: "));
    Serial.println(BASE_135);
    Serial.print(F("BASE_180: "));
    Serial.println(BASE_180);
    Serial.println(F(""));
    Serial.print(F("ELBOW_MIN(-30): "));
    Serial.println(ELBOW_MIN_VALUE);
    Serial.print(F("ELBOW_MAX(90): "));
    Serial.println(ELBOW_MAX_VALUE);
    Serial.println(F(""));
    Serial.print(F("SHOULDER_MIN(F): "));
    Serial.println(SHOULDER_MIN_VALUE);
    Serial.print(F("SHOULDER_MAX(B): "));
    Serial.println(SHOULDER_MAX_VALUE);
    Serial.println(F(""));
    Serial.print(F("CLAW_GRAB: "));
    Serial.println(CLAW_GRAB);
    Serial.print(F("CLAW_OPEN: "));
    Serial.println(CLAW_OPEN);
    Serial.print(F("CLAW_RELEASE: "));
    Serial.println(CLAW_RELEASE);
    Serial.print(F("CLAW_PREPARE: "));
    Serial.println(CLAW_PREPARE);
    Serial.println(F(""));
  }
  
  
  //shoulder, elbow, and claw in here too.
  
  
  
  
  CurrentPos = BASE_90;

    GoToHD(START_HEIGHT, START_DISTANCE); //in here, must also update shoulder and elbow values
    
    Serial.print(F("CurrentPos: "));
    Serial.println(CurrentPos);
    
    BaseServo.writeMicroseconds(CurrentPos);
    ClawServo.write(START_CLAW);
  
  
  
  
  if(DEBUG)
    Serial.println(F("Done Setup"));
    
    /*
    accelerateBase(BASE_0);
    delay(1000);
    accelerateBase(BASE_180);
    delay(1000);
    accelerateBase(CurrentPos);
    delay(1000);
    */
    
}
//****************************************************************



//****************************LOOP********************************
void loop(){
    //Nothing in the loop - no wonder it does not work!
    //put in one simple updateLCD() function called at the start of every loop. if(LCD) if(inMenu)
    //if(LCD_Enabled)
    if(LCD_Enabled){
       UpdateLCD();
    }
    //****COLOR CALIBRATION
    //if(CalibrateNow == HIGH){ //always leave this in, push button to calibrate, (will wait for serial input - should change to button push on another pin)
      //calibrateColor(true, true, FromSERIAL); //....or it could be this button again, changing CalibrateNow will do nothing until calibration is complete. LEDs to indicate which color to calibrate.
    //}
    //print out color
    //Serial.println(getMultiColor()); //sometimes still show no color - see how much of a tolerance we need to put in
    //*******
    
    /*
    accelerateBase(BASE_0);
    delay(1000);
    accelerateBase(BASE_180);
    delay(1000);
    accelerateBase(CurrentPos);
    delay(1000);
    */
}
//*****************************************************************
//****************************************************************


bool findMinDistAng(){
  MinDist = 1000; //way bigger than it ever will get
  MinDistAng = 100; //again bigger than it will be
  
  int Dist;
  
  ///height should be greater than 100 if putting ultrasonic underneath
  GoToHD(110, 100);
  
  for(int i = 45; i >= 0; i--){
    accelerateBase(i);
    //Dist = get distance(); //in mm
    Dist = pingSonarMedian(false);// + ULTRA_OFFSET; we were accounting for the OFFSET twice!
    delay(50);
    if(Dist < MinDist){
      MinDist = Dist;   // + ULTRA_OFFSET; //account for the offset here
      MinDistAng = i;
    }
  }  
  
  if(MinDist >= MAX_DISTANCE){
    //no block
    lcd.clear();
    delay(3);
    return false;
  }
  else
  {
    if(MinDistAng - ANGLE_OFFSET > 0)
      MinDistAng -= ANGLE_OFFSET;
    if(MinDist - DISTANCE_OFFSET > 0)
      MinDist -= DISTANCE_OFFSET;
    return true;
  }
}


bool GoToBlockPos(){
  //if(findMinDistAng()){
    /*
    moveClawServo(CLAW_OPEN);
    GoToHD(100, 70);
    accelerateBase(MinDistAng);
    GoToHD(GRAB_RELEASE_HEIGHT, 70);
    accelerateBase(MinDistAng);
    GoToHD(GRAB_RELEASE_HEIGHT, MinDist);
    */
    
    moveClawServo(CLAW_OPEN);
    GoToHD(100, 70);
    accelerateBase(22); //middle
    GoToHD(GRAB_RELEASE_HEIGHT, 70);
    GoToHD(GRAB_RELEASE_HEIGHT, 130);
    moveClawServo(CLAW_GRAB);
    return true;
  //}
  //return false;
}


bool FindAndDropBlock(){
  
    if(GoToBlockPos()){
  
    char BlockColor = getMultiColor(); //find the color
    
    //move up to travel height
    GoToHD(TRAVEL_HEIGHT, 100);
    //small delay
    
    switch(BlockColor){
      case('R'):
      //move the block and drop it (for each one)
      accelerateBase(R_BASE);
      GoToHD(TRAVEL_HEIGHT, R_DIST);
      GoToHD(GRAB_RELEASE_HEIGHT, R_DIST);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, R_DIST);
      break;
      case('G'):
      accelerateBase(G_BASE);
      GoToHD(TRAVEL_HEIGHT, G_DIST);
      GoToHD(GRAB_RELEASE_HEIGHT, G_DIST);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, G_DIST);
      break;
      case('B'):
      accelerateBase(B_BASE);
      GoToHD(TRAVEL_HEIGHT, B_DIST);
      GoToHD(GRAB_RELEASE_HEIGHT, B_DIST);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, B_DIST);
      break;
      case('K'):
      
        //leave it there
      GoToHD(GRAB_RELEASE_HEIGHT, Distance_Value);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, DONE_DISTANCE);
      
      break;
      case('N'):
      /*
      GoToHD(GRAB_RELEASE_HEIGHT, Distance_Value);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, DONE_DISTANCE);
      */
      accelerateBase(R_BASE);
      GoToHD(TRAVEL_HEIGHT, R_DIST);
      GoToHD(GRAB_RELEASE_HEIGHT, R_DIST);
      moveClawServo(CLAW_OPEN);
      GoToHD(DONE_HEIGHT, R_DIST);
      break;
      default:
      break;
      
    }
    return true;
    }
  return false;
}






//SWEEP FUNCTION
bool Sweep_Base(){ //moves from 45 to 0
  MinDist = 1000; //way bigger than it ever will get
  MinDistAng = 100; //again bigger than it will be
  
  int Dist;
  
  ///height should be greater than 100 if putting ultrasonic underneath
  GoToHD(110, 100);
  
  for(int i = 45; i >= 0; i--){
    accelerateBase(i);
    //Dist = get distance(); //in mm
    Dist = pingSonarMedian(false);// + ULTRA_OFFSET; we were accounting for the OFFSET twice!
    delay(50);
    if(Dist < MinDist){
      MinDist = Dist;   // + ULTRA_OFFSET; //account for the offset here
      MinDistAng = i;
    }
  }

  if(MinDist > MAX_DISTANCE ){ //more than the arm can reach
    //no block found - infinite loop, or print DONE on screen.
    /*
    lcd.clear();
    lcd.setCursor(7,1);
    lcd.print(F("DONE!"));
    while(1){
      delay(9999);//should actually go back to the LCD menu here.
    }
    */
    return false;
  }
  else{ //a block was found
    accelerateBase(MinDistAng); //move the base back to that spot
    //go to block height, and minDistance
    GoToHD(GRAB_RELEASE_HEIGHT, MIN_DISTANCE); //prepare to get the block
    moveClawServo(CLAW_OPEN); //open the claw
    //should be a small delay here
    delay(SMALL_DELAY);
    GoToHD(GRAB_RELEASE_HEIGHT, MinDist); //distance of the block
    //another small delay here
    delay(SMALL_DELAY);
    moveClawServo(CLAW_GRAB); //grab the block
    //very small delay
    delay(VERY_SMALL_DELAY);
    char BlockColor = getMultiColor(); //find the color
    
    //move up to travel height
    GoToHD(TRAVEL_HEIGHT, 100);
    //small delay
    
    switch(BlockColor){
      case('R'):
      //move the block and drop it (for each one)
      accelerateBase(R_BASE);
      GoToHD(TRAVEL_HEIGHT, R_DIST);
      //small delay
      delay(SMALL_DELAY);
      GoToHD(GRAB_RELEASE_HEIGHT, R_DIST);
      moveClawServo(CLAW_OPEN);
      //small delay
      delay(SMALL_DELAY);
      //then move up, start again
      GoToHD(DONE_HEIGHT, R_DIST);
      //small delay
      break;
      case('G'):
      
      break;
      case('B'):
      
      break;
      case('K'):
      
        //leave it there
      
      break;
      case('N'):
      
      break;
      default:
      break;
      
    }
    
  }
  return true;
}


void SortBlocks(){
  while(FindAndDropBlock()){
    //keep doing it.
  }
}



//**********************************LCD FUNCTIONS*****************


//everything from loop in old menu program.
void UpdateLCD(){
  while(inMenu){
    
      //update cursor - turn it on
      if ((!lastMenu) && (inMenu)){ //turn the cursor on if we were not in menu before
        lastMenu = inMenu;
        lcd.cursor();
      if(DEBUG)
        Serial.println(F("In Menu"));
      }
      
      //if any values have changed, update the LCD
      if(BASE_CHANGE){
          //lcd.setCursor(PositionVal_Col, Base_Row);
          print3DigNoLead0(PositionVal_Col, Base_Row, Base_Value);
          BASE_CHANGE = false;
      }
      if(CLAW_CHANGE){
          //print3DigNoLead0(PositionVal_Col, Claw_Row, Claw_Value);
          CLAW_CHANGE = false;
      }
      if(HEIGHT_CHANGE){
          print3DigNoLead0(PositionVal_Col, Height_Row, Height_Value);
          HEIGHT_CHANGE = false;
      }
      if(DISTANCE_CHANGE){
          print3DigNoLead0(PositionVal_Col, Distance_Row, Distance_Value);
          DISTANCE_CHANGE = false;
      }
      
      
      
      //check for a button push if the debounce time has passed.
      if ((millis() - lastButton) > Button_Delay){
        uint8_t ButtonPushed = checkButtons(); //so this is returning a 0
        if (ButtonPushed == NoButton){}
        else
          lastButton = millis(); //if a button was pushed, change the time of the last button
          if (DEBUG && (ButtonPushed != NoButton)){
            Serial.print(F("Button Pushed: ")); //which button was pushed
            Serial.println(ButtonPushed);
            Serial.print(F("Cursor Location: "));//the new cursor location
            Serial.print(CursorChar);
            Serial.print(F(","));
            Serial.println(CursorRow);
          }
          MoveCursorMenu(ButtonPushed); //always trigerring left button
      }
    }
    //turn the cursor off if we were in the menu before
    if ((lastMenu) && (!inMenu)){
      lastMenu = inMenu;
      lcd.noCursor(); 
    if(DEBUG)
      Serial.println(F("Not in menu"));
    } 
}


void LCDSetup(){
  lcd.begin(LCD_Chars,LCD_Rows);
  lcd.clear();
  lcd.backlight();
  lcd.noCursor();
  lcd.setCursor(0,0);
  //lcd.createChar(2, newChar1); //create the arrow character
}


void LCDinfoSetup(){
  lcd.clear();
  lcd.noCursor(); 
  lcd.setCursor(0,0);
  //lcd.write(uint8_t(2)); //draw the arrow

  //height, distance, and angle
  lcd.setCursor(PositionNam_Col, Base_Row); //not necessary, cursor is already here
  lcd.print(F("B"));
  print3DigNoLead0(PositionVal_Col, Base_Row, Base_Value);
  
  lcd.setCursor(PositionNam_Col,Height_Row);
  lcd.print(F("H"));
  print3DigNoLead0(PositionVal_Col, Height_Row, Height_Value);
  
  lcd.setCursor(PositionNam_Col,Distance_Row);
  lcd.print(F("D"));
  print3DigNoLead0(PositionVal_Col, Distance_Row, Distance_Value);
  
  lcd.setCursor(PositionNam_Col,Claw_Row);
  lcd.print(F("C"));
  //print3DigNoLead0(PositionVal_Col, Claw_Row, Claw_Value);
  lcd.setCursor(PositionVal_Col,Claw_Row);
  lcd.print(F("O"));//open
  lcd.setCursor(PositionVal_Col+1,Claw_Row);
  lcd.print(F("G")); //grab
  lcd.setCursor(PositionVal_Col+2,Claw_Row);
  lcd.print(F("R"));//release
  lcd.setCursor(PositionVal_Col+3,Claw_Row);
  lcd.print(F("P"));//prepare
  
  
  //print calibrate
  lcd.setCursor(CALIBRATE_CHAR, CALIBRATE_ROW);
  lcd.print(CALIBRATE_TEXT);
  
  //print find color
  lcd.setCursor(FINDCOLOR_CHAR, FINDCOLOR_ROW);
  lcd.print(FINDCOLOR_TEXT);
  
  //print distance
  lcd.setCursor(DISTANCE_CHAR, DISTANCE_ROW);
  lcd.print(DISTANCE_TEXT);
  
  //print caliservo
  lcd.setCursor(CALISERVO_CHAR, CALISERVO_ROW);
  lcd.print(CALISERVO_TEXT);
  
  //cal temp
  lcd.setCursor(CALTEMP_CHAR, CALTEMP_ROW);
  lcd.print(CALTEMP_TEXT);
  
  lcd.setCursor(UOFF_CHAR, UOFF_ROW);
  lcd.print(UOFF_TEXT);
  
  lcd.setCursor(CALR_CHAR, CALR_ROW);
  lcd.print(CALR_TEXT);
  lcd.setCursor(CALG_CHAR, CALG_ROW);
  lcd.print(CALG_TEXT);
  lcd.setCursor(CALB_CHAR, CALB_ROW);
  lcd.print(CALB_TEXT);
  lcd.setCursor(CALR_CHAR-3, CALR_ROW);
  lcd.print(F("Cal"));
  
  lcd.setCursor(GOTOR_CHAR, GOTOR_ROW);
  lcd.print(GOTOR_TEXT);
  lcd.setCursor(GOTOG_CHAR, GOTOG_ROW);
  lcd.print(GOTOG_TEXT);
  lcd.setCursor(GOTOB_CHAR, GOTOB_ROW);
  lcd.print(GOTOB_TEXT);
  lcd.setCursor(GOTOR_CHAR-4, GOTOR_ROW);
  lcd.print(F("GoTo"));
  
  lcd.setCursor(SWEEP_CHAR, SWEEP_ROW);
  lcd.print(SWEEP_TEXT);
  
  lcd.setCursor(SORT_CHAR, SORT_ROW);
  lcd.print(SORT_TEXT);
  
  lcd.setCursor(UCAL_CHAR, UCAL_ROW);
  lcd.print(UCAL_TEXT);
  
  lcd.setCursor(FINDMDA_CHAR, FINDMDA_ROW);
  lcd.print(FINDMDA_TEXT);
  
  lcd.setCursor(GOTOMDA_CHAR, GOTOMDA_ROW);
  lcd.print(GOTOMDA_TEXT);
  
  lcd.setCursor(MOVEBLOCK_CHAR, MOVEBLOCK_ROW);
  lcd.print(MOVEBLOCK_TEXT);
  
  //rest of stuff in here
 
  lcd.setCursor(0,0);
  CursorChar = 0;
  CursorRow = 0;
}




//move up down, left, right
void MoveCursorMenu(uint8_t Button){
  //if(DEBUG)
    //Serial.println(F("MoveCursorMenu Function"));
  switch (Button){
    case(LeftButton): //move cursor left (always)
      if(DEBUG)
        Serial.println(F("   Left Button"));
      if (CursorChar != 0) //decrement if we are not already at 0
        CursorChar --;
      lcd.setCursor(CursorChar, CursorRow);
      break;
      
    case (RightButton): //move cursor right (always)
      if(DEBUG)
        Serial.println(F("   Right Button"));
      if (CursorChar != LCD_Chars-1)//increment if not at the right of the screen
        CursorChar ++;
      else if(CursorChar >= CALIBRATE_CHAR+1 && CursorRow == CALIBRATE_ROW)//in the calibrate name
        CursorChar = CALIBRATE_CHAR;
      else if (CursorChar >= FINDCOLOR_CHAR+1 && CursorRow == FINDCOLOR_ROW) //in the find color
        CursorChar = FINDCOLOR_CHAR;
      else if (CursorChar >= DISTANCE_CHAR+1 && CursorRow == DISTANCE_ROW) //in distance
        CursorChar = DISTANCE_CHAR;
      else if (CursorChar >= CALISERVO_CHAR+1 && CursorRow == CALISERVO_ROW) //in caliservo
        CursorChar = CALISERVO_CHAR;
      lcd.setCursor(CursorChar, CursorRow);
      break;
      
    case (TopButton):
      if(DEBUG)
        Serial.println(F("   Top Button"));
    //increment servo values if in certain columns
      if (CursorOnServoVal())
        IncrementOnServo(UP); //Up
      else{ //else move the cursor
        if (CursorRow != 0) //decrement if not already at 0
          CursorRow --;
        else if ((CursorChar >= CALIBRATE_CHAR) && (CursorRow == CALIBRATE_ROW)) //if in the calibrate name
          CursorRow = CALIBRATE_ROW + 1;
        else if ((CursorChar >= FINDCOLOR_CHAR) && (CursorRow == FINDCOLOR_ROW)) //in color
          CursorRow = FINDCOLOR_ROW + 1;
        else if ((CursorChar >= DISTANCE_CHAR) && (CursorRow == DISTANCE_ROW)) //in distance
          CursorRow = DISTANCE_ROW + 1;
        else if ((CursorChar >= CALISERVO_CHAR) && (CursorRow == CALISERVO_ROW)) //in distance
          CursorRow = CALISERVO_ROW + 1;
        lcd.setCursor(CursorChar, CursorRow);
      }
      break;
      
    case (CenterButton):
      if(DEBUG)
        Serial.println(F("   Center Button"));
      //nothing right now
      //enter calibration if on the right row and column
      if (CursorChar == CALIBRATE_CHAR && CursorRow == CALIBRATE_ROW)
        LCDCalibrate(); //calibrate, then reset the LCD
      else if (CursorChar == FINDCOLOR_CHAR && CursorRow == FINDCOLOR_ROW)
        LCDFindColor(); //find the color, print it out below
      else if (CursorChar == DISTANCE_CHAR && CursorRow == DISTANCE_ROW)
        LCDDistance();
      else if (CursorChar == CALISERVO_CHAR && CursorRow == CALISERVO_ROW)
        LCDCalibrateServos();
        
      else if((CursorChar == PositionVal_Col) && (CursorRow == Claw_Row)){
        //CLAW_OPEN
        moveClawServo(CLAW_OPEN);
      }
      else if((CursorChar == PositionVal_Col+1) && (CursorRow == Claw_Row)){
        //grab claw
        moveClawServo(CLAW_GRAB);
      }
      else if((CursorChar == PositionVal_Col+2) && (CursorRow == Claw_Row)){
        //release claw
        moveClawServo(CLAW_RELEASE);
      }
      else if((CursorChar == PositionVal_Col+3) && (CursorRow == Claw_Row)){
        //prepare claw
        moveClawServo(CLAW_PREPARE);
      }
      else if ((CursorChar == CALTEMP_CHAR) && (CursorRow == CALTEMP_ROW))
        LCDCalTemp();
      
      else if ((CursorChar == CALR_CHAR) && (CursorRow == CALR_ROW))
        LCDCalR();
      else if ((CursorChar == CALG_CHAR) && (CursorRow == CALG_ROW))
        LCDCalG();
      else if ((CursorChar == CALB_CHAR) && (CursorRow == CALB_ROW))
        LCDCalB();
      else if ((CursorChar == GOTOR_CHAR) && (CursorRow == GOTOR_ROW))
        LCDGoToR();
      else if ((CursorChar == GOTOG_CHAR) && (CursorRow == GOTOG_ROW))
        LCDGoToG();
      else if ((CursorChar == GOTOB_CHAR) && (CursorRow == GOTOB_ROW))
        LCDGoToB();
      else if ((CursorChar == UOFF_CHAR) && (CursorRow == UOFF_ROW))
        LCDCalUOFF();
      
      else if ((CursorChar == SWEEP_CHAR) && (CursorRow == SWEEP_ROW))
        Sweep_Base();
      else if ((CursorChar == SORT_CHAR) && (CursorRow == SORT_ROW))
        SortBlocks();
      else if ((CursorChar == UCAL_CHAR) && (CursorRow == UCAL_ROW))
        LCDCalUDIST();
      
      else if ((CursorChar == FINDMDA_CHAR) && (CursorRow == FINDMDA_ROW))
        LCDFMDA();
      else if ((CursorChar == GOTOMDA_CHAR) && (CursorRow == GOTOMDA_ROW))
        GoToBlockPos();
      else if ((CursorChar == MOVEBLOCK_CHAR) && (CursorRow == MOVEBLOCK_ROW))
        FindAndDropBlock(); 
      
      
      break;
      
    case(BottomButton):
      if(DEBUG)
        Serial.println(F("   Bottom Button"));
    //increment servo values if in certain columns
    if (CursorOnServoVal())
        IncrementOnServo(DOWN); //Down
    else{ //else move the cursor up or down
      if (CursorRow != LCD_Rows-1) //increment if not already at max
        CursorRow ++;
      lcd.setCursor(CursorChar, CursorRow);
    }
      break;
    case(NoButton):
      //if(DEBUG)
        //Serial.println(F("   No Button"));
      break;
    default:
      if(DEBUG)
        Serial.println(F("DEFAULT"));
      break;
  }
}


//is the cursor on a servo value
bool CursorOnServoVal(){
  if ((CursorChar == PositionVal_Col || CursorChar == PositionVal_Col +1 || CursorChar == PositionVal_Col+2)  && CursorRow != Claw_Row){
    return true;
    if(DEBUG)
      Serial.println(F("On Servo"));
  }
  if(DEBUG)
    Serial.println(F("Not on Servo"));
  return false;
}

//up or down buttons when on a servo value
void IncrementOnServo(int8_t UpDown){ //UpDown is 1 or -1 for up or down button
  if (CursorChar == PositionVal_Col)
    incrementServoValue(CursorRow, 100*UpDown);
  else if (CursorChar == PositionVal_Col+1)
    incrementServoValue(CursorRow, 10*UpDown);
  else if (CursorChar == PositionVal_Col+2)
    incrementServoValue(CursorRow, 1*UpDown);
}

//for servo values - print 3 digits without a leading 0
void print3DigNoLead0(uint8_t Char, uint8_t Row, uint8_t Value){ 
  Value = constrain(Value, 0, 999);//constrained to 3 digs.
  lcd.setCursor(Char,Row);
  if (Value / 100 == 0)
    lcd.print(" ");
  else
    lcd.print(Value / 100);
  lcd.setCursor(Char+1,Row);
  if (Value / 100 >= 1)
    lcd.print((Value % 100) / 10);
  else if ((Value % 100)/10 == 0)
    lcd.print(" ");
  else
    lcd.print((Value % 100) / 10);
  lcd.setCursor(Char+2,Row);
  lcd.print(Value % 10);
  
  lcd.setCursor(CursorChar, CursorRow); //put the cursor back where it is expected to be
}

//here we should write to the servos as well.
void incrementServoValue(uint8_t CursorRow, int8_t Value){
  switch (CursorRow){
    case(Base_Row):
      if(Base_Value + Value >= MAX_BASE)
        Base_Value = MAX_BASE;
      else if (Base_Value + Value <= MIN_BASE)
        Base_Value = MIN_BASE;
      else
        Base_Value += Value;
      //Base_Value = constrain(Base_Value, 0, 180);
      print3DigNoLead0(PositionVal_Col, Base_Row, Base_Value);
      //moveBaseServo(Base_Value);
      accelerateBase(Base_Value);
      break;
    case(Height_Row):
      if(Height_Value + Value >= MAX_HEIGHT)
        Height_Value = MAX_HEIGHT;
      else if (Height_Value + Value <= MIN_HEIGHT)
        Height_Value = MIN_HEIGHT;
      else
        Height_Value += Value;
      //Height_Value = constrain(Height_Value, 0, 180);
      print3DigNoLead0(PositionVal_Col, Height_Row, Height_Value);
      //moveShoulderElbowServos(Height_Value, Distance_Value);
      GoToHD(Height_Value, Distance_Value);
      break;
    case(Distance_Row):
      if(Distance_Value + Value >= MAX_DISTANCE)
        Distance_Value = MAX_DISTANCE;
      else if (Distance_Value + Value <= MIN_DISTANCE)
        Distance_Value = MIN_DISTANCE;
      else
        Distance_Value += Value;
      //Distance_Value = constrain(Distance_Value, 0, 180);
      print3DigNoLead0(PositionVal_Col, Distance_Row, Distance_Value);
      //moveShoulderElbowServos(Height_Value, Distance_Value);
      GoToHD(Height_Value, Distance_Value);
      break;
    case(Claw_Row): //claw is no longer controlled like this.
      /*
      if(Claw_Value + Value >= MAX_CLAW)
        Claw_Value = MAX_CLAW;
      else if (Claw_Value + Value <= MIN_CLAW)
        Claw_Value = MIN_CLAW;
      else
        Claw_Value += Value;
      //Claw_Value = constrain(Claw_Value, 0, 180);
      print3DigNoLead0(PositionVal_Col, Claw_Row, Claw_Value); //update LCD
      moveClawServo(Claw_Value); //update servo
      */
      break;
    default:
      break;
  }
}

uint8_t getButtonPressed(){
    delay(Button_Delay);
    return checkButtons();
}


uint8_t checkButtons(){//check all the buttons and return the button that was pressed
  if(!(digitalRead(Left_Sw)))
    return LeftButton;
  else if(!(digitalRead(Top_Sw)))
    return TopButton;
  else if(!(digitalRead(Center_Sw)))
    return CenterButton;
  else if(!(digitalRead(Bottom_Sw)))
    return BottomButton;
  else if(!(digitalRead(Right_Sw)))
    return RightButton;
  else
    return NoButton;
}



void menuState(){
  
  if (LCD_Enabled && ((millis() - lastInterrupt) > Button_Delay)){
    lastMenu = inMenu;
    lastInterrupt = millis();
    inMenu = !inMenu;
    //lcd.setCursor(10,0);
    //lcd.print(inMenu);
    //lcd.setCursor(10,0);
    //lcd.print(" ");
  }
}


void LCDGoToMDA(){
  LCDFMDA();
  moveClawServo(CLAW_OPEN);
  accelerateBase(MinDistAng);
  GoToHD(GRAB_RELEASE_HEIGHT, 70);
  GoToHD(GRAB_RELEASE_HEIGHT, MinDist);
}


void LCDFMDA(){
  if(findMinDistAng()){
    lcd.setCursor(MIN_DIST_CHAR, MIN_DIST_ROW);
    lcd.print(MIN_DIST_TEXT);
    lcd.setCursor(MIN_DIST_CHAR+1, MIN_DIST_ROW);
    lcd.print(F("   "));
    lcd.setCursor(MIN_DIST_CHAR+1, MIN_DIST_ROW);
    lcd.print(MinDist);
    
    lcd.setCursor(MIN_ANG_CHAR, MIN_ANG_ROW);
    lcd.print(MIN_ANG_TEXT);
    lcd.setCursor(MIN_ANG_CHAR+1, MIN_ANG_ROW);
    lcd.print(F("   "));
    lcd.setCursor(MIN_ANG_CHAR+1, MIN_ANG_ROW);
    lcd.print(MinDistAng);
  }
}

void LCDCalUOFF(){
  lcd.clear();
  lcd.setCursor(0,0);
  ULTRA_OFFSET = 16;
  lcd.print(ULTRA_OFFSET);
  
  lcd.setCursor(0,3);
  lcd.print(F("Done - Press Center"));
  lcd.setCursor(0,2);
  lcd.print(F("Ultra Offset"));
  lcd.setCursor(0,1);
  lcd.print(F("Top or Bottom Change"));
  
  uint8_t BUTTONPressed;
  
  do{
    BUTTONPressed = waitButtonReturn();
    switch(BUTTONPressed){
    case(TopButton):
      if(ULTRA_OFFSET +1 <= 99)
        ULTRA_OFFSET += 1;
    break;
    case(BottomButton):
      if(ULTRA_OFFSET -1 >= 0)
        ULTRA_OFFSET -= 1;
    break;
    }
    lcd.setCursor(0,0);
    lcd.print("  ");
    lcd.setCursor(0,0);
    lcd.print(ULTRA_OFFSET);
  }while (BUTTONPressed != CenterButton);
  
  EEPROM.writeInt(ULTRA_OFFSET_ADD, ULTRA_OFFSET);
  
  LCDinfoSetup(); //reset the LCD
}

void LCDCalUDIST(){
  //measurements of blocks at 5, 10, 15, 20, 25cm.
  //5
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Block at 5cm"));
  lcd.setCursor(0,3);
  lcd.print(F("Push A Button"));
  waitButton();
  
  UCAL_5 = pingSonarMedian(true);
  lcd.setCursor(0,1);
  lcd.print(UCAL_5);
  delay(700);
  
  //10
  lcd.setCursor(0,0);
  lcd.print(F("Block at 10cm"));
  waitButton();
  
  UCAL_10 = pingSonarMedian(true);
  lcd.setCursor(0,1);
  lcd.print(F("          "));
  lcd.setCursor(0,1);
  lcd.print(UCAL_10);
  delay(700);
  
  //15
  lcd.setCursor(0,0);
  lcd.print(F("Block at 15cm"));
  waitButton();
  
  UCAL_15 = pingSonarMedian(true);
  lcd.setCursor(0,1);
  lcd.print(F("          "));
  lcd.setCursor(0,1);
  lcd.print(UCAL_15);
  delay(700);
  
  //20
  lcd.setCursor(0,0);
  lcd.print(F("Block at 20cm"));
  waitButton();
  
  UCAL_20 = pingSonarMedian(true);
  lcd.setCursor(0,1);
  lcd.print(F("          "));
  lcd.setCursor(0,1);
  lcd.print(UCAL_20);
  delay(700);
  
  //10
  lcd.setCursor(0,0);
  lcd.print(F("Block at 25cm"));
  waitButton();
  
  UCAL_25 = pingSonarMedian(true);
  lcd.setCursor(0,1);
  lcd.print(F("          "));
  lcd.setCursor(0,1);
  lcd.print(UCAL_25);
  delay(700);
  
  
  EEPROM.writeInt(UCAL_5_ADD, UCAL_5);
  EEPROM.writeInt(UCAL_10_ADD, UCAL_10);
  EEPROM.writeInt(UCAL_15_ADD, UCAL_15);
  EEPROM.writeInt(UCAL_20_ADD, UCAL_20);
  EEPROM.writeInt(UCAL_25_ADD, UCAL_25);
  
  
  LCDinfoSetup();
}

void LCDCalR(){
  //save current base and distance and red block pos
  R_BASE = Base_Value;
  R_DIST = Distance_Value;
  
  EEPROM.writeByte(R_BASE_ADD, R_BASE);
  EEPROM.writeByte(R_DIST_ADD, R_DIST);
  
  if(DEBUG){
    Serial.print(F("R_BASE: "));
    Serial.println(R_BASE);
    Serial.print(F("R_DIST: "));
    Serial.println(R_DIST);
  }
    
}
void LCDCalG(){
  //save current base and distance and green block pos
  G_BASE = Base_Value;
  G_DIST = Distance_Value;
  
  EEPROM.writeByte(G_BASE_ADD, G_BASE);
  EEPROM.writeByte(G_DIST_ADD, G_DIST);
  
  if(DEBUG){
    Serial.print(F("G_BASE: "));
    Serial.println(G_BASE);
    Serial.print(F("G_DIST: "));
    Serial.println(G_DIST);
  }
}
void LCDCalB(){
  //save current base and distance and blue block pos
  B_BASE = Base_Value;
  B_DIST = Distance_Value;
  
  EEPROM.writeByte(B_BASE_ADD, B_BASE);
  EEPROM.writeByte(B_DIST_ADD, B_DIST);
  
  if(DEBUG){
    Serial.print(F("B_BASE: "));
    Serial.println(B_BASE);
    Serial.print(F("B_DIST: "));
    Serial.println(B_DIST);
  }
}

void LCDGoToR(){
  accelerateBase(R_BASE);
  GoToHD(Height_Value, R_DIST); //careful not to go too far/high
}
void LCDGoToG(){
  accelerateBase(G_BASE);
  GoToHD(Height_Value, G_DIST); //careful not to go too far/high
}
void LCDGoToB(){
  accelerateBase(B_BASE);
  GoToHD(Height_Value, B_DIST); //careful not to go too far/high
}

void LCDCalibrate(){
    Serial.println(F("Calibrating from LCD"));
    lcd.clear();
    calibrateColor(true, true, FromLCD);
}

void LCDFindColor(){
    lcd.setCursor(FINDCOLOR_CHAR-1, FINDCOLOR_ROW);
    lcd.print(F(" "));
    lcd.setCursor(FINDCOLOR_CHAR-1, FINDCOLOR_ROW);
    lcd.print(getMultiColor());
    lcd.setCursor(FINDCOLOR_CHAR-1, FINDCOLOR_ROW);
}

void LCDDistance(){
  //clear the 3 digits, 
  //get distance from ultrasonic sensor
  //print in mm in the 3 spaces to the left of Distance
  
  lcd.setCursor(DISTANCE_CHAR - 3, DISTANCE_ROW);
  lcd.print("   ");
  //print3DigNoLead0(DISTANCE_CHAR-3, DISTANCE_ROW, pingSonarMedian());
  lcd.setCursor(DISTANCE_CHAR - 3, DISTANCE_ROW);
  lcd.print(pingSonarMedian(false));
}

void LCDCalTemp(){
  lcd.clear();
  lcd.setCursor(0,0);
  AMBIENT_TEMP = 23;
  lcd.print(AMBIENT_TEMP);
  
  lcd.setCursor(0,3);
  lcd.print(F("Done - Press Center"));
  lcd.setCursor(0,2);
  lcd.print(F("Ambient temprature"));
  lcd.setCursor(0,1);
  lcd.print(F("Top or Bottom Change"));
  
  uint8_t BUTTONPressed;
  
  do{
    BUTTONPressed = waitButtonReturn();
    switch(BUTTONPressed){
    case(TopButton):
      if(AMBIENT_TEMP +1 <= 40)
        AMBIENT_TEMP += 1;
    break;
    case(BottomButton):
      if(AMBIENT_TEMP -1 >= 10)
        AMBIENT_TEMP -= 1;
    break;
    }
    lcd.setCursor(0,0);
    lcd.print("  ");
    lcd.setCursor(0,0);
    lcd.print(AMBIENT_TEMP);
  }while (BUTTONPressed != CenterButton);
  
  EEPROM.writeByte(AMBIENT_TEMP_ADD, AMBIENT_TEMP);
  
  LCDinfoSetup(); //reset the LCD
}

void LCDCalibrateServos(){
  uint8_t BUTTONPressed;
  
  lcd.clear();
  
  lcd.setCursor(10,0);
  lcd.print(F("B"));
  lcd.setCursor(2,1);
  lcd.print(F("E"));
  lcd.setCursor(17,1);
  lcd.print(F("S"));
  lcd.setCursor(10,2);
  lcd.print(F("C"));
  
  lcd.noCursor();
  
  BUTTONPressed = waitButtonReturn();
  
  lcd.clear();
  
  switch(BUTTONPressed){
    case(TopButton):
      //calibrate Base
    
      //BASE_0
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("BASE_0"));
      
      BASE_0 = CalServo('B', 1500);
      
      //BASE_45
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("BASE_45"));
      
      BASE_45 = CalServo('B', 1500);
      
      //BASE_90
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("BASE_90"));
      
      BASE_90 = CalServo('B', 1500);
      
      //BASE_135
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("BASE_135"));
      
      BASE_135 = CalServo('B', 1500);
      
      //BASE_180
      
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("BASE_180"));
      
      BASE_180 = CalServo('B', 1500);
      
      EEPROM.writeInt(BASE_0_ADD, BASE_0);
      EEPROM.writeInt(BASE_45_ADD, BASE_45);
      EEPROM.writeInt(BASE_90_ADD, BASE_90);
      EEPROM.writeInt(BASE_135_ADD, BASE_135);
      EEPROM.writeInt(BASE_180_ADD, BASE_180);
      
    break;
  
    case(LeftButton):
      //calibrate Elbow
      
      //ELBOW_MIN_VALUE //at -30 to X
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("ELBOW_MIN(-30)"));
      
      ELBOW_MIN_VALUE = CalServo('E', 1500);
      
      
      //ELBOW_MAX_VALUE //at 90 to X straight up.
    
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("ELBOW_MAX(90)"));
      
      ELBOW_MAX_VALUE = CalServo('E', 1500);
    
      EEPROM.writeInt(ELBOW_MIN_VALUE_ADD, ELBOW_MIN_VALUE);
      EEPROM.writeInt(ELBOW_MAX_VALUE_ADD, ELBOW_MAX_VALUE);
    
    break;
    
    case(RightButton):
      //calibrate Shoulder
      
      //SHOULDER_MIN_VALUE //as far forwards
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("SHOULDER_MIN(F)"));
      
      SHOULDER_MIN_VALUE = CalServo('S', 1500);
      
      //SHOULDER_MAX_VALUE //as far backwards
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("SHOULDER_MAX(B)"));
      
      SHOULDER_MAX_VALUE = CalServo('S', 1500);
      
      EEPROM.writeInt(SHOULDER_MIN_VALUE_ADD, SHOULDER_MIN_VALUE);
      EEPROM.writeInt(SHOULDER_MAX_VALUE_ADD, SHOULDER_MAX_VALUE);
  
    break;  
  
    case(BottomButton):
      //Calibrate Claw
    
      //CLAW_GRAB
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("CLAW_GRAB"));
      
      CLAW_GRAB = CalServo('C', 1500);
      
      //CLAW_OPEN
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("CLAW_OPEN"));
      
      CLAW_OPEN = CalServo('C', 1500);
      
      //CLAW_RELEASE
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("CLAW_RELEASE"));
      
      CLAW_RELEASE = CalServo('C', 1500);
      
      //CLAW_PREPARE //prepare to grab the block - will probably not be used.
      lcd.setCursor(4,0);
      lcd.print(F("              "));
      lcd.setCursor(4,0);
      lcd.print(F("CLAW_PREPARE"));
      
      CLAW_PREPARE = CalServo('C', 1500);
      
      EEPROM.writeInt(CLAW_PREPARE_ADD, CLAW_PREPARE);
      EEPROM.writeInt(CLAW_GRAB_ADD, CLAW_GRAB);
      EEPROM.writeInt(CLAW_OPEN_ADD, CLAW_OPEN);
      EEPROM.writeInt(CLAW_RELEASE_ADD, CLAW_RELEASE);
      
    break;
  
    default:
      //exit
    break;
  
  }
  
  LCDinfoSetup(); //reset the LCD
  
}


//wait for a button to be pushed and return the button
uint8_t waitButtonReturn(){
  uint8_t bUTTON;
  do{
    bUTTON = getButtonPressed();
  }while (bUTTON == NoButton);
  return bUTTON;
}


//wait for a button to be pushed.
void waitButton(){
    do{
        delay(Button_Delay);
    } while (checkButtons() == NoButton);
}

//******************************END OF LCD FUNCTIONS****************************


//***************************ULTRASONIC*********************

int pingSonaruS(){//returns the ping in uS Length
  
  //int getDistance(int trigPin, int echoPin) // returns the distance (cm)
//{
    long duration;

    pinMode(SIGNAL_PIN, OUTPUT);
    digitalWrite(SIGNAL_PIN, LOW);
    delayMicroseconds(7);
    digitalWrite(SIGNAL_PIN, HIGH);
    delayMicroseconds(12);
    digitalWrite(SIGNAL_PIN, LOW);
    delayMicroseconds(1);
    
    pinMode(SIGNAL_PIN, INPUT);
    duration = pulseIn(SIGNAL_PIN, HIGH, 20000);
    if(duration == 0)
      duration = 20000;
    else{
        duration /= 2;
    }

    //keep as uS
    //distance = (duration/2) / 29.1; // We calculate the distance (sound speed in air is aprox. 291m/s), /2 because of the pulse going and coming
    return (int)duration; //We return the result. returns max distance if timed out. (2000mm)
//}
 
  //return 980; //test value
}

int pingSonarmm(){ //return the distance to an object in mm.
  long Distance, SpeedOfSound;
  
  Distance = pingSonaruS();
  
  //if(Distance == 20000)
    //return MAX_DISTANCE;
  
  SpeedOfSound = SPEED_OF_SOUND_BASE + (SOUND_TEMP_MULTIPLIER * AMBIENT_TEMP);
  
  Distance *= 0.001 * SpeedOfSound;
  //Distance *= SpeedOfSound;
  
  Distance -= ULTRA_OFFSET;
  
  //if(Distance > MAX_PLAY_DISTANCE)
    //Distance = MAX_PLAY_DISTANCE;
  
  
  
  return constrain((int)Distance, 0, 999);
}


int pingSonarMedian(bool CAL){
  double DISTance;
  
  RunningMedian Distances(5);
  for(int i = 0; i < 5; i++){
    Distances.add(pingSonarmm());
    delay(5);
  }
  
  DISTance = (double)Distances.getMedian();
  
  if(!CAL){
    //map the values
    
    if((DISTance >= UCAL_5) && (DISTance <= UCAL_10))
      DISTance = map(DISTance, UCAL_5, UCAL_10, 50, 100);
      
    else if((DISTance >= UCAL_10) && (DISTance <= UCAL_15))
      DISTance = map(DISTance, UCAL_10, UCAL_15, 100, 150);
      
    else if((DISTance >= UCAL_15) && (DISTance <= UCAL_20))
      DISTance = map(DISTance, UCAL_15, UCAL_20, 150, 200);
      
    else if((DISTance >= UCAL_20) && (DISTance <= UCAL_25))
      DISTance = map(DISTance, UCAL_20, UCAL_25, 200, 250);
      
    else
      DISTance = 500;

  }
  
  return (int)DISTance;
}






//****************************************SERVO MOVEMENT FUNCTIONS******************

int CalServo(char SERVOInitial, int STARTVal){
  int CalVal = STARTVal;
  uint8_t ButtonPressed;
  
  uint8_t Character;
  
  //lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Done - Push Center"));
  lcd.setCursor(0,0);
  lcd.print(SERVOInitial);
  lcd.cursor();
  
  //set up cursor
  lcd.setCursor(0,1);
  Character = 0;
  
  do{
  
    CalVal = constrain(CalVal, 0,3000);
  
    //print value on LCD
    lcd.setCursor(0,1);
    lcd.print(F("    "));
    
    if(CalVal >= 1000){
      lcd.setCursor(0,1);
      lcd.print(CalVal);
    }
    else if (CalVal >= 100){
      lcd.setCursor(1,1);
      lcd.print(CalVal);
    }
    else if (CalVal >= 10){
      lcd.setCursor(2,1);
      lcd.print(CalVal);
    }
    else{
      lcd.setCursor(3,1);
      lcd.print(CalVal);
    }
  
    //write value to the correct servo
    switch(SERVOInitial){
      case('B'):
        BaseServo.writeMicroseconds(CalVal);
      break;
      case('S'):
        ShoulderServo.writeMicroseconds(CalVal);
      break;
      case('E'):
        ElbowServo.writeMicroseconds(CalVal);
      break;
      case('C'):
        ClawServo.writeMicroseconds(CalVal);
      break;
      default:
      break;
    }
    
    //update cursor
    lcd.setCursor(Character,1);
    
    //change value with buttons
    ButtonPressed = waitButtonReturn();
    
    switch(ButtonPressed){
      case(TopButton):
        if(Character == 0)
          CalVal += 1000;
        else if(Character == 1)
          CalVal += 100;
        else if(Character == 2)
          CalVal += 10;
        else if(Character == 3)
          CalVal += 1;
      
      break;
      case(BottomButton):
        if(Character == 0)
          CalVal -= 1000;
        else if(Character == 1)
          CalVal -= 100;
        else if(Character == 2)
          CalVal -= 10;
        else if(Character == 3)
          CalVal -= 1;
      
      break;
      case(LeftButton):
        if(Character > 0)
          Character -= 1;
      
      break;
      case(RightButton):
        if(Character < 3)
          Character += 1;
      
      break;
      default:
      break;
    }
  
  }while (ButtonPressed != CenterButton);
  
  
  return CalVal;
}


/* Now accelerateBase(target);
void moveBaseServo(int targetVal){ //base servo will always be individual of the other three - use smoothing
  if(DEBUG){
    Serial.print(F("Base Value:"));
    Serial.println(targetVal); 
  }
  BaseServo.write(targetVal);
}
*/

void moveClawServo(int targetVal){ //no smoothing needed
  if(targetVal != Claw_Value){
        Claw_Value = targetVal;
        CLAW_CHANGE = true;
        //update LCD - no, don't update it here, wait to go to the updateLCD
    }
  

    int ClawServoValue = ClawServo.readMicroseconds();
    
    int ClawDifference = targetVal - ClawServoValue;
    int ClawDir = ((targetVal > ClawServoValue) ? 1 : (-1));
    if(ClawDifference == 0)
      ClawDir = 0;
    
    while(ClawServoValue != targetVal){
      ClawDifference = targetVal - ClawServoValue;
      ClawDir = ((targetVal > ClawServoValue) ? 1 : (-1));
      if(ClawDifference == 0)
        ClawDir = 0;

     ClawServoValue += 1*ClawDir;
     
     ClawServo.writeMicroseconds(ClawServoValue);
     
     delayMicroseconds(1100);
    }
  
  //ClawServo.writeMicroseconds(targetVal);
}


//************************************END OF SERVO MOVEMENT FUNCTIONS******************





//******************************************************HEIGHT DISTANCE FUNCTIONS*********************************

void GoToHD(float Height, float Distance){
    if(DEBUG)
        Serial.println(F("In HeightDistance"));
        
    //height Value and distance value become height and distance if they are not already.
    if((int)Height != Height_Value){
        Height_Value = (int)Height;
        HEIGHT_CHANGE = true;
        //update LCD - no, don't update it here, wait to go to the updateLCD
    }
    if((int)Distance != Distance_Value){
        Distance_Value = (int)Distance;
        DISTANCE_CHANGE = true;
        //update LCD - no, don't update it here, wait to go to the updateLCD
    }
    
    
    float CalcShoulderAngle; //to the shoulder servo
    float CalcClawAngle; //to the elbow servo

    if(DEBUG){
        Serial.print(F("Height: "));
        Serial.println(Height);
        Serial.print(F("Distance: "));
        Serial.println(Distance);
        Serial.println(F(""));
    }
    
    //Shoulder Offset
    Height -= SHOULDER_Y;
    Distance -= SHOULDER_X;
    //Claw Offset
    Height -= CLAW_OFFSET_Y;
    Distance -= CLAW_OFFSET_X;
    
    /*
    Serial.println(RadToDeg(atan(1)));
    Serial.println(RadToDeg(atan(0.333333)));
    Serial.println(RadToDeg(atan(Height / Distance)));
    Serial.println("");
    */
    
    float ShoulderToClawDist = sqrt((sq(Distance))+(sq(Height))); //hypotenuse of distance, height triangle.
    //angles in the triangle
    
    if(DEBUG){
        Serial.print(F("Shoulder to Claw Distance: "));
        Serial.println(ShoulderToClawDist);
        Serial.println(F(""));
    }
    
    //It will ALWAYS bew an isoceles triangle because the two sides are the same.
    CalcShoulderAngle = acos((sq(ELBOW_LENGTH) - sq(ShoulderToClawDist) - sq(SHOULDER_LENGTH)) / (-2 * (ShoulderToClawDist) * (SHOULDER_LENGTH)));
    //CalcClawAngle = acos((sq(SHOULDER_LENGTH) - sq(ShoulderToClawDist) - sq(ELBOW_LENGTH)) / (-2 * (ShoulderToClawDist) * (ELBOW_LENGTH)));
    CalcClawAngle = CalcShoulderAngle;
    
    if(DEBUG){
        Serial.print(F("Radians Raw Shoulder Angle: "));
        Serial.println(CalcShoulderAngle);
        Serial.print(F("Radians Raw Claw Angle: "));
        Serial.println(CalcClawAngle);
        Serial.println(F(""));
    }
    
    //convert to degrees from radians
    CalcShoulderAngle = RadToDeg(CalcShoulderAngle);
    //CalcClawAngle = RadToDeg(CalcClawAngle);
    CalcClawAngle = CalcShoulderAngle;
    
    if(DEBUG){
        Serial.print(F("Degrees Raw Shoulder Angle: "));
        Serial.println(CalcShoulderAngle);
        Serial.print(F("Degrees Raw Claw Angle: "));
        Serial.println(CalcClawAngle);
        Serial.println(F(""));
    }
    
    //angles from triangle relative to X-axis
    //float HDivD = (Height / Distance); //return 0 - Why - int instead of float for Height and Distance
    //Serial.println(Height / Distance);
    
    if(DEBUG){
        Serial.print(F("Radians: atan (Height / Distance): "));
        Serial.println(atan(Height / Distance));
        Serial.print(F("Degrees: atan (Height / Distance): "));
        Serial.println(RadToDeg(atan(Height / Distance)));
        Serial.println(F(""));
    }
    
    CalcShoulderAngle += RadToDeg(atan(Height / Distance));
    CalcClawAngle -= RadToDeg(atan(Height / Distance));

    //convert to degrees from radians
    //CalcShoulderAngle = RadToDeg(CalcShoulderAngle);
    //CalcClawAngle = RadToDeg(CalcClawAngle);
    
    if(DEBUG){
        Serial.print(F("Degrees X axis Shoulder Angle: "));
        Serial.println(CalcShoulderAngle);
        Serial.print(F("Degrees X axis Claw Angle: "));
        Serial.println(CalcClawAngle);
        Serial.println(F(""));
    }
    
    //map to actual servo angles
    
    ElbowServoAngle = map(CalcClawAngle, ELBOW_MIN_POSITION, ELBOW_MAX_POSITION, ELBOW_MIN_VALUE, ELBOW_MAX_VALUE); //these should be defines
    ShoulderServoAngle = map(CalcShoulderAngle, SHOULDER_MIN_POSITION, SHOULDER_MAX_POSITION, SHOULDER_MIN_VALUE, SHOULDER_MAX_VALUE);
    
    
    
    int ElbowServoValue = ElbowServo.readMicroseconds();
    int ShoulderServoValue = ShoulderServo.readMicroseconds();
    
    int ShoulderDifference = ShoulderServoAngle - ShoulderServoValue;
    int ShoulderDir = ((ShoulderServoAngle > ShoulderServoValue) ? 1 : (-1));
    if(ShoulderDifference == 0)
      ShoulderDir = 0;
      
    int ElbowDifference = ElbowServoAngle - ElbowServoValue;
    int ElbowDir = ((ElbowServoAngle > ElbowServoValue) ? 1 : (-1));
    if(ElbowDifference == 0)
      ElbowDir = 0;
    
    while((ElbowServoValue != ElbowServoAngle) || (ShoulderServoValue != ShoulderServoAngle)){
      ShoulderDifference = ShoulderServoAngle - ShoulderServoValue;
      ShoulderDir = ((ShoulderServoAngle > ShoulderServoValue) ? 1 : (-1));
      if(ShoulderDifference == 0)
        ShoulderDir = 0;
      
      ElbowDifference = ElbowServoAngle - ElbowServoValue;
      ElbowDir = ((ElbowServoAngle > ElbowServoValue) ? 1 : (-1));
      if(ElbowDifference == 0)
        ElbowDir = 0;
     
     ElbowServoValue += 1*ElbowDir;
     ShoulderServoValue += 1* ShoulderDir;
     
     ElbowServo.writeMicroseconds(ElbowServoValue);
     ShoulderServo.writeMicroseconds(ShoulderServoValue);
     
     delayMicroseconds(1100);
    }
    
    //ElbowServo.writeMicroseconds(ElbowServoAngle);
    //ShoulderServo.writeMicroseconds(ShoulderServoAngle);
    
    
    
    
    if(DEBUG){
        Serial.print(F("Shoulder Servo uS: "));
        Serial.println(ShoulderServoAngle);
        Serial.print(F("Elbow Servo uS: "));
        Serial.println(ElbowServoAngle);
        Serial.println(F(""));
    }
    
}

float RadToDeg(float Rad){
    return (Rad * (180/pi));
}
//***********************************END OF HEIGHT DISTANCE FUNCTIONS*************************************


//***************************************COLOR SENSOR FUNCTIONS********************************
//*******************************************************************************************************************
//***************************************************MAIN CALIBRATION FUNCTION**********************************
void calibrateColor(bool WB, bool saveEEPROM, bool Method){
 //This will learn the Red, Green, and Blue color presented to it, optionally save it to EERPOM.
    
//calibrate individual color data
//calibrateWB first
  if (WB)
    calibrateWB(Method);
  
  
  Serial.println(F("Calibrating Min and Max values for individual colors"));
  Serial.println(F(" Enter number of times to sample colors: "));
  
  uint8_t ColorReadings = 5;
  
  if(Method == FromLCD){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("#Times to read color"));
      lcd.setCursor(0,1);
      lcd.print(F("5"));
  }
  
  if(Method == FromSERIAL){
  clearInput();
  while(Serial.available() == 0) {}
  ColorReadings = Serial.parseInt(); //number of times to read each color of block - use at least 3-4 times
  }
  
  else if(Method == FromLCD){
      do{
          if(checkButtons() == TopButton){
              ColorReadings ++;
              lcd.setCursor(0,1);
              if(ColorReadings > 9)
                ColorReadings = 9;
              lcd.print(ColorReadings);
              
          }
          else if (checkButtons() == BottomButton){
              ColorReadings --;
              lcd.setCursor(0,1);
              if(ColorReadings < 1)
                ColorReadings = 1;
              lcd.print(ColorReadings);
          }
          
      }while (getButtonPressed() != CenterButton);
      
      
  }
  
  ColorReadings = constrain(ColorReadings, 1, 9);//keep it to reasonable values 
  
  //put a 10 second timeout - will default to 5 if surpassed.
  //If the board is not connected to serial, then it can still work - then all wait serial would need timeout as well.
  
//each color close and far.

//************************RED************************
//red close - get baseline readings for red, do not compare to anything.
    Serial.println(F("RED block calibration"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("RED"));
        lcd.setCursor(10,0);
        lcd.print(F("1"));
        lcd.setCursor(0,2);
        lcd.print(F("Press a button"));
        waitButton();
    }
    
    updateRGBValues();
    R_Rmax = R; R_Gmax = G; R_Bmax = B; //make the sensor learn these values from samples of colors.
    R_Rmin = R; R_Gmin = G; R_Bmin = B;
    
    if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
    
//red far - change baseline readings to reflect proper values    

    for (int i = 1; i < ColorReadings; i++){
      Serial.println(F(""));
    Serial.println(F("RED block at various positions"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.setCursor(10,0);
        lcd.print(i+1);
        
        waitButton();
    }
    
    updateRGBValues();

    if (R < R_Rmin)
      R_Rmin = R;
    if (R > R_Rmax)
      R_Rmax = R;
    if (G < R_Gmin)
      R_Gmin = G;
    if (G > R_Gmax)
      R_Gmax = G;
    if (B < R_Bmin)
      R_Bmin = B;
    if (B > R_Bmax)
      R_Bmax = B;
      
      if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
      
    }
      Serial.println("");
//************************GREEN************************
//green close - get baseline readings for red, do not compare to anything.
    Serial.println(F("GREEN block calibration"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("GREEN"));
        lcd.setCursor(10,0);
        lcd.print(F("1"));
        lcd.setCursor(0,2);
        lcd.print(F("Press a button"));
        waitButton();
    }
    
    updateRGBValues();
    G_Rmax = R; G_Gmax = G; G_Bmax = B; //make the sensor learn these values from samples of colors.
    G_Rmin = R; G_Gmin = G; G_Bmin = B;
    
    if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
      
//green far - change baseline readings to reflect proper values    
    for (int i = 1; i < ColorReadings; i++){
    Serial.println(F(""));
    Serial.println(F("GREEN block at various positions"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.setCursor(10,0);
        lcd.print(i+1);
        
        waitButton();
    }
    
    updateRGBValues();

    if (R < G_Rmin)
      G_Rmin = R;
    if (R > G_Rmax)
      G_Rmax = R;
    if (G < G_Gmin)
      G_Gmin = G;
    if (G > G_Gmax)
      G_Gmax = G;
    if (B < G_Bmin)
      G_Bmin = B;
    if (B > G_Bmax)
      G_Bmax = B;
    
    if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
    
    }
    Serial.println("");
//************************BLUE************************
//green close - get baseline readings for red, do not compare to anything.
    Serial.println(F("BLUE block calibration"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("BLUE"));
        lcd.setCursor(10,0);
        lcd.print(F("1"));
        lcd.setCursor(0,2);
        lcd.print(F("Press a button"));
        waitButton();
    }
    
    updateRGBValues();
    B_Rmax = R; B_Gmax = G; B_Bmax = B; //make the sensor learn these values from samples of colors.
    B_Rmin = R; B_Gmin = G; B_Bmin = B;
    
    if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
    
//green far - change baseline readings to reflect proper values    
for (int i = 0; i < ColorReadings-1; i++){
  Serial.println(F(""));
    Serial.println(F("BLUE block at various positions"));
    if (DEBUG)
      Serial.println(F("Enter character to continue"));
    
    if(Method == FromSERIAL)
        waitSerial();
    else{
        lcd.setCursor(10,0);
        lcd.print(i+1);
        
        waitButton();
    }
    
    updateRGBValues();

    if (R < B_Rmin)
      B_Rmin = R;
    if (R > B_Rmax)
      B_Rmax = R;
    if (G < B_Gmin)
      B_Gmin = G;
    if (G > B_Gmax)
      B_Gmax = G;
    if (B < B_Bmin)
      B_Bmin = B;
    if (B > B_Bmax)
      B_Bmax = B;
      
      if(Method == FromLCD){
          lcd.setCursor(0,3);
          lcd.print(F("R"));
          print3DigNoLead0(1,3,R);
          lcd.setCursor(6,3);
          lcd.print(F("G"));
          print3DigNoLead0(7,3,G);
          lcd.setCursor(12,3);
          lcd.print(F("B"));
          print3DigNoLead0(13,3,B);
          delay(1000);
      }
      
    
}
Serial.println(F(""));

//Do not need black

Serial.println(F("Calibration Finished"));

//Any way to remove outliers - values falsely read close to 255?

incrementDecrementRGBminmax(20, 20); //to provide a 'failsafe of sorts - some blocks will be slightly greater or less than the values measured.
  /*
  int value = 10;
  R_Rmax += value; R_Gmax += value; R_Bmax += value;
  R_Rmin -= value; R_Gmin -= value; R_Bmin -= value;
  
  G_Rmax += value; G_Gmax += value; G_Bmax += value;
  G_Rmin -= value; G_Gmin -= value; G_Bmin -= value;
  
  B_Rmax += value; B_Gmax += value; B_Bmax += value;
  B_Rmin -= value; B_Gmin -= value; B_Bmin -= value;
  */
  
  
//save values to EEPROM
//EEPROM address up to 374 are used for B and W, up to 1024 is available.

  if(saveEEPROM){
    EEPROM.writeInt(R_Rmin_Add, R_Rmin);
    EEPROM.writeInt(R_Gmin_Add, R_Gmin);
    EEPROM.writeInt(R_Bmin_Add, R_Bmin);
    EEPROM.writeInt(R_Rmax_Add, R_Rmax);
    EEPROM.writeInt(R_Gmax_Add, R_Gmax);
    EEPROM.writeInt(R_Bmax_Add, R_Bmax);

    EEPROM.writeInt(G_Rmin_Add, G_Rmin);
    EEPROM.writeInt(G_Gmin_Add, G_Gmin);
    EEPROM.writeInt(G_Bmin_Add, G_Bmin);
    EEPROM.writeInt(G_Rmax_Add, G_Rmax);
    EEPROM.writeInt(G_Gmax_Add, G_Gmax);
    EEPROM.writeInt(G_Bmax_Add, G_Bmax);
  
    EEPROM.writeInt(B_Rmin_Add, B_Rmin);
    EEPROM.writeInt(B_Gmin_Add, B_Gmin);
    EEPROM.writeInt(B_Bmin_Add, B_Bmin);
    EEPROM.writeInt(B_Rmax_Add, B_Rmax);
    EEPROM.writeInt(B_Gmax_Add, B_Gmax);
    EEPROM.writeInt(B_Bmax_Add, B_Bmax);
  }

    if(DEBUG){
        serialprintMinMaxRGB();
        SerialPrintRGBCalibrationEEPROM();
    }

    if(Method == FromLCD)
        LCDinfoSetup();

}

//**********************************************END OF MAIN CALIBRATION*********************



void incrementDecrementRGBminmax(int minvalue, int maxvalue){
    //DONE - must get fixed - if value is less than 10, will get bumped to other end of int - 65000ish - not good for comparisons.
    //DONE - To Do - check if value will go below 0 before decrementing. If it will, set it to 0.
 //increment and decrement the max and min values by 5 respectively 
 //do not need to worry about constraining values - will not affect comparisons
  R_Rmax += maxvalue; R_Gmax += maxvalue; R_Bmax += maxvalue;
  
  if (R_Rmin - minvalue > 0)
    R_Rmin -= minvalue; 
  else
    R_Rmin = 0;

  if (R_Gmin - minvalue > 0)
    R_Gmin -= minvalue; 
  else
    R_Gmin = 0;
   
  if (R_Bmin - minvalue > 0)
    R_Bmin -= minvalue; 
  else
    R_Bmin = 0; 
  
  G_Rmax += maxvalue; G_Gmax += maxvalue; G_Bmax += maxvalue;
  
  if (G_Rmin - minvalue > 0)
    G_Rmin -= minvalue; 
  else
    G_Rmin = 0; 
    
  if (G_Gmin - minvalue > 0)
    G_Gmin -= minvalue; 
  else
    G_Gmin = 0;
 
  if (G_Bmin - minvalue > 0)
    G_Bmin -= minvalue; 
  else
    G_Bmin = 0;
  
  B_Rmax += maxvalue; B_Gmax += maxvalue; B_Bmax += maxvalue;
  
  if (B_Rmin - minvalue > 0)
    B_Rmin -= minvalue; 
  else
    B_Rmin = 0;
    
  if (B_Gmin - minvalue > 0)
    B_Gmin -= minvalue; 
  else
    B_Gmin = 0;
    
  if (B_Bmin - minvalue > 0)
    B_Bmin -= minvalue; 
  else
    B_Bmin = 0;
}

void serialprintMinMaxRGB(){
  //print out the minimum and maximum valued for detecting each color.
  if(DEBUG){
  Serial.println(F(""));
  Serial.println(F(""));
  
  Serial.println(F("Minimum and Maximum RGB Color Detection values:"));
  
  Serial.println(F(""));
  Serial.println(F("     RED"));
  Serial.print(F("R: "));
  Serial.print(R_Rmin);
  Serial.print(F(" - "));
  Serial.println(R_Rmax);
  Serial.print(F("G: "));
  Serial.print(R_Gmin);
  Serial.print(F(" - "));
  Serial.println(R_Gmax);
  Serial.print(F("B: "));
  Serial.print(R_Bmin);
  Serial.print(F(" - "));
  Serial.println(R_Bmax);
  
  Serial.println(F(""));
  Serial.println(F("     GREEN"));
  Serial.print(F("R: "));
  Serial.print(G_Rmin);
  Serial.print(F(" - "));
  Serial.println(G_Rmax);
  Serial.print(F("G: "));
  Serial.print(G_Gmin);
  Serial.print(F(" - "));
  Serial.println(G_Gmax);
  Serial.print(F("B: "));
  Serial.print(G_Bmin);
  Serial.print(F(" - "));
  Serial.println(G_Bmax);
  
  Serial.println(F(""));
  Serial.println(F("     BLUE"));
  Serial.print(F("R: "));
  Serial.print(B_Rmin);
  Serial.print(F(" - "));
  Serial.println(B_Rmax);
  Serial.print(F("G: "));
  Serial.print(B_Gmin);
  Serial.print(F(" - "));
  Serial.println(B_Gmax);
  Serial.print(F("B: "));
  Serial.print(B_Bmin);
  Serial.print(F(" - "));
  Serial.println(B_Bmax);
  
  Serial.println(F(""));
  Serial.println(F(""));
  }
}

//print out the calibration values stored in EEPROM
void SerialPrintRGBCalibrationEEPROM(){
  if(DEBUG){
  Serial.println(F(""));
  Serial.println(F(""));
  
  Serial.println(F("Minimum and Maximum RGB Calibration Values from EEPROM:"));
  
  Serial.println(F(""));
  Serial.println(F("     RED"));
  Serial.print(F("R: "));
  Serial.print(EEPROM.readInt(R_Rmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(R_Rmax_Add));
  Serial.print(F("G: "));
  Serial.print(EEPROM.readInt(R_Gmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(R_Gmax_Add));
  Serial.print(F("B: "));
  Serial.print(EEPROM.readInt(R_Bmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(R_Bmax_Add));
  
  Serial.println(F(""));
  Serial.println(F("     GREEN"));
  Serial.print(F("R: "));
  Serial.print(EEPROM.readInt(G_Rmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(G_Rmax_Add));
  Serial.print(F("G: "));
  Serial.print(EEPROM.readInt(G_Gmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(G_Gmax_Add));
  Serial.print(F("B: "));
  Serial.print(EEPROM.readInt(G_Bmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(G_Bmax_Add));
  
  Serial.println(F(""));
  Serial.println(F("     BLUE"));
  Serial.print(F("R: "));
  Serial.print(EEPROM.readInt(B_Rmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(B_Rmax_Add));
  Serial.print(F("G: "));
  Serial.print(EEPROM.readInt(B_Gmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(B_Gmax_Add));
  Serial.print(F("B: "));
  Serial.print(EEPROM.readInt(B_Bmin_Add));
  Serial.print(F(" - "));
  Serial.println(EEPROM.readInt(B_Bmax_Add));
  
  Serial.println(F(""));
  Serial.println(F(""));
  }
}


//****************************************GET Best Color From Multiple Readings***********************************
char getMultiColor(){ //read color NumReadings times and make sure the values are consistent
 const int NumReadings = 4;
 char Readings[NumReadings] = {0};
 
 do{
 
   for (int i = 0; i < NumReadings; i++){
    Readings[i] = getSingleColor(); // get a number of readings into a char array.
   }
   
 }while (Readings[0] != Readings[NumReadings-1]); //while frist and last readings are not the same - should remove any possiblity of repeatability error
 
 if(DEBUG){
   Serial.print(Readings[0]);
   Serial.println(F("  Multiple Color Correction Reading"));
 }
 
 return Readings[0];
  
}

//****************************************UPDATE RGB VALUES - Check sensor again************************
//easy way to call readSensor() function
void updateRGBValues(){
  do
    {
      readSensor();
    }while (JustRead == false); //read from sensor, wait until it completes
}

//****************************GET Color From A Single Reading*************************************
char getSingleColor(){
  updateRGBValues();
  
  if ((R >= R_Rmin) && (R <= R_Rmax) &&
      (G >= R_Gmin) && (G <= R_Gmax) &&
      (B >= R_Bmin) && (B <= R_Bmax)){
          if (DEBUG){
          Serial.println(F("Red Color Detected"));
          }
          return 'R';
  }

  else if ((R >= G_Rmin) && (R <= G_Rmax) &&
           (G >= G_Gmin) && (G <= G_Gmax) &&
           (B >= G_Bmin) && (B <= G_Bmax)){
            if (DEBUG) {
            Serial.println(F("Green Color Detected"));
            }
            return 'G';
	}
	
  else if ((R >= B_Rmin) && (R <= B_Rmax) &&
           (G >= B_Gmin) && (G <= B_Gmax) &&
           (B >= B_Bmin) && (B <= B_Bmax)){
             if (DEBUG){
              Serial.println(F("Blue Color Detected"));
             }
              return 'B';
	}

    //black also seems to be 255 sometimes. If in the range or 255.
  else if ((R >= K_Rmin) && (R <= K_Rmax) &&
          (G >= K_Gmin) && (G <= K_Gmax) &&
          (B >= K_Bmin) && (B <= K_Bmax)){
            if (DEBUG) {
              Serial.println(F("Black Color Detected"));
            }
              return 'K';
          }
  else{
    if (DEBUG){
    Serial.println(F("No Color Detected"));
    }
    return 'N';
        }
    return 'E'; //should never get here - if so, problem in if statements, return E for Error
  }


//***********************Calibrate on interrupt - no longer needed.*********************************
void ChangeCalibrate(){ //function activated by interrupt button
 CalibrateNow = HIGH;
}



//************************Calibrate White and Black values ************************
void calibrateWB(bool CalMethod){ //read black and white values, and save to EEPROM via library
  // calibrate black
  do{
    readState = fsmReadValue(readState, BLACK_CAL, 2, CalMethod);
  }while (readState != 0);
  
  
	  //if (readState == 0) runState++;
	  //break;

  // calibrate white
  do{
    readState = fsmReadValue(readState, WHITE_CAL, 2, CalMethod);
  }while (readState != 0);

        //if (readState == 0) runState++;
        //break;
  CalibrateNow = LOW;
}


void readSensor()
{
  
  static bool waiting;
  
  JustRead = false;
  if (!waiting)
  {
    CS.read();
    waiting = true;
  }
  else
  {
    if (CS.available()) 
    {
      colorData  rgb;
      
      CS.getRGB(&rgb);
      
      R = rgb.value[TCS230_RGB_R];
      G = rgb.value[TCS230_RGB_G];
      B = rgb.value[TCS230_RGB_B];
      
      if (DEBUG){
      Serial.print(F("RGB ["));
      Serial.print(R);
      Serial.print(F(","));
      Serial.print(G);
      Serial.print(F(","));
      Serial.print(B);
      Serial.println(F("]"));
      }
      
      waiting = false;
      JustRead = true;
    }
  }
}

//***********************************Wait for a character to be sent over serial*******************
//ToDo - with the LCD, wait for any button push
void waitSerial(){
  getChar();
  clearInput();
}

char getChar()
// blocking wait for an input character from the input stream
{
	while (Serial.available() == 0)
		;
	return(toupper(Serial.read()));
}

void clearInput()
// clear all characters from the serial input
{
	while (Serial.read() != -1)
		;
}

//************************************************Get Raw Sensor Data********************
//Only used for Calibrating white and black - the ends of the RGB line.
uint8_t fsmReadValue(uint8_t state, uint8_t valType, uint8_t maxReads, bool CalibMethod) //I believe this is only used to calibrate (this is from examples)
// Finite State Machine for reading a value from the sensor
// Current FSM state is passed in and returned
// Type of value being read is passed in
{
	static	uint8_t	selChannel;
	static	uint8_t	readCount;
	static	sensorData	sd;

	switch(state)
	{
	case 0:	// Prompt for the user to start
		Serial.print(F("\n\nReading value for "));
		switch(valType)
		{
		case BLACK_CAL:	Serial.print(F("BLACK calibration"));	break;
		case WHITE_CAL:	Serial.print(F("WHITE calibration"));	break;
		case READ_VAL:	Serial.print(F("DATA"));				break;
		default:		Serial.print(F("??"));					break;
		}
		
		if(CalibMethod == FromSERIAL)
		    Serial.print(F("\nPress any key to start ..."));
		if(CalibMethod == FromLCD){
		    lcd.setCursor(0,0);
		    lcd.print(F("Press any key"));
		    lcd.setCursor(0,1);
		    switch(valType)
		    {
		    case BLACK_CAL:	lcd.print(F("BLACK calibration"));	break;
		    case WHITE_CAL:	lcd.print(F("WHITE calibration"));	break;
		    case READ_VAL:	lcd.print(F("DATA"));				break;
		    default:		lcd.print(F("??"));					break;
		    }
		}
		
		state++;
		break;

	case 1:	// Wait for user input
		if(CalibMethod == FromSERIAL)
		    waitSerial();
		else
		    waitButton();
		state++;
		break;

	case 2:	// start the reading process
		CS.read();
		state++;
		break;

	case 3:	// wait for a read to complete
		if (CS.available()) 
		{
			sensorData	sd;
			colorData	rgb;

			switch(valType)
			{
			case BLACK_CAL:	
				CS.getRaw(&sd);	
				CS.setDarkCal(&sd);		
				break;

			case WHITE_CAL:	
				CS.getRaw(&sd);	
				CS.setWhiteCal(&sd);	
				break;

			case READ_VAL:	
				CS.getRGB(&rgb);
				Serial.print(F("\nRGB is ["));
				Serial.print(rgb.value[TCS230_RGB_R]);
				Serial.print(F(","));
				Serial.print(rgb.value[TCS230_RGB_G]);
				Serial.print(F(","));
				Serial.print(rgb.value[TCS230_RGB_B]);
				Serial.print(F("]"));
				break;
			}
			state++;
		}
		break;

	default:	// reset fsm
		state = 0;
		break;
	}

	return(state);
}

//***************************END OF COLOR SENSOR FUNCTIONS**************************


//********************************SERVO ACCEL FUNCTION**********************
void accelerateBase(int Target){ //pass in maxspeed and accel as well - target could be passed as deg.
    if(Target != Base_Value){
        Base_Value = Target;
        BASE_CHANGE = true;
        //update LCD - no, don't update it here, wait to go to the updateLCD
    }
    
    if(DEBUG)
      Serial.println(F("In AccelerateBase"));
    
    if(Target <= 180){ //if in degrees, convert to uS.
        if(Target <= 45)
          Target = map(Target, 0, 45, BASE_0, BASE_45);
        else if(Target <= 90)
          Target = map(Target, 45, 90, BASE_45, BASE_90);
        else if(Target <= 135)
          Target = map(Target, 90, 135, BASE_90, BASE_135);
        else if(Target <= 180)
          Target = map(Target, 135, 180, BASE_135, BASE_180);
    }
    
    int Start = CurrentPos;
    int Difference = Target - CurrentPos;
    int Dir = ((Target > CurrentPos) ? 1 : (-1));
    Difference = abs(Difference);
    int speed = 0;
    int fromStart;
    int fromEnd;
    int NumInc = MAX_SPEED / ACCEL;
    
    //increment speed by 1 until at max speed
    //increment currentPos by 1*Dir
    //delayMicroseconds by 1001 - speed
    
    while (CurrentPos != Target){
        fromStart = Start - CurrentPos;
        fromStart = abs(fromStart);
        fromEnd = CurrentPos - Target;
        fromEnd = abs(fromEnd);
        
        if ((fromStart <= NumInc) && (fromStart <= fromEnd)){
            //accelerate
            speed =+ ACCEL;
        }
        
        else if ((fromStart >= NumInc) && (fromEnd >= NumInc)){
            //go max speed
            speed = MAX_SPEED;
        }
        
        //while both points less than MAX_SPEED
            //if from end is less than from start, break
            //travel at max
        
        
        //if from end is less, decelerate.
        else if (fromEnd <= NumInc){
            //decelerate
            speed -= ACCEL;
        }
        
        //speed += accel; //speed *= accel?
        if(speed > MAX_SPEED)
            speed = MAX_SPEED;
        CurrentPos += 1*Dir;
        delayMicroseconds(MAX_REFERENCE+1 - speed);
        BaseServo.writeMicroseconds(CurrentPos);
    }
}

//*********************************END OF SERVO ACCEL**************************
