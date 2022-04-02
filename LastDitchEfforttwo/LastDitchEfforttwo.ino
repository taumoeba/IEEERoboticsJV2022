// Currently configured for testing

/*******************************************************************************************
 * Main program for LU JV IEEE Robotics 2022
 * Written by Ben Powell and Stephen Fulton
 * April 2022
 *
 * Microcontroller: Arduino Mega 2560
 * Components List:
 * - Adafruit Motor Control Shield v2.3
 * - 4x DC Motor
 * - 3x Stepper Motor
 * - 1x Servo Motor
 * - Pixy2 Smart Vision Sensor
 *
 * Reference schematic.png for connection details
 ******************************************************************************************/

//#include "MainRobotControl.h"

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy2.h>
#include "Adafruit_VL53L0X.h"

#define MOTOR_STEPS 200
#define RPM 120
#define GRABBER_SERVO_PIN 10
#define CLAW_SERVO_PIN 9
#define OUT1 5
#define OUT2 6
#define D1 13

//defining arm lengths:
#define ARM_EXTEND_LENGTH      500    //this raises the arm up to tree height (untested)
#define SCREW_EXTEND_LENGTH    1000   //this fully extends the lead screw (untested)
#define ROTATEY_DOWN 10
#define ROTATEY_UP 100
#define ROTATEY_FULL_UP 150
#define ROTATEY_CUP 40

//number of steps to get a 90degree turn from susan
#define QUARTER_TURN 460  //testing needed to get precise amount

#define XSHUT1 4
#define XSHUT2 7
#define XSHUT3 11
#define XSHUT4 12

#define CLAWOPEN 0 // Need to test
#define CLAWCLOSE 160 // Need to test

#define TOMM(inches) (double)(inches/0.0393701) //convert from inches to mm
#define MM(mm) (int)(mm*7.55) //turns millimeters into milliseconds

Adafruit_MotorShield AFMSstep(0x61); // Rightmost jumper closed, bottom, stepper motors
Adafruit_MotorShield AFMSdc(0x60); // Default address, no jumpers, top, dc motors
Adafruit_StepperMotor *susan = AFMSstep.getStepper(200, 1); // lazy susan
Adafruit_StepperMotor *extendo = AFMSstep.getStepper(200, 2); // raise arm

Adafruit_DCMotor *M2 = AFMSdc.getMotor(1);
Adafruit_DCMotor *M3 = AFMSdc.getMotor(2);
Adafruit_DCMotor *M1 = AFMSdc.getMotor(3);
Adafruit_DCMotor *M4 = AFMSdc.getMotor(4);

Servo grabberServo;
Servo clawServo;

// distance sensor setup
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

bool allClear = true; // set to false in final version, true for testing
bool allClearOld = false;

void allStop() {
  M1->run(RELEASE);
  M2->run(RELEASE);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

// Drive motors will continue driving until allStop() is called or an individual motor is stopped
void driveBackward() {
  allStop();
  if(allClear) {
    M1->run(BACKWARD);
    M3->run(BACKWARD);
  }
}

void driveForward() {
  allStop();
  if(allClear) {
    M1->run(FORWARD);
    M3->run(FORWARD);
  }
}

void driveLeft() {
  allStop();
  if(allClear) {
    M2->run(BACKWARD);
    M4->run(BACKWARD);
  }
}

void driveRight() {
  allStop();
  if(allClear) {
    M2->run(FORWARD);
    M4->run(FORWARD);
  }
}

// Lead screw will continue moving until leadScrewStop() is called
void leadScrewOut() {
  digitalWrite(D1, LOW);
  analogWrite(OUT1, 127);
  digitalWrite(OUT2, LOW);
}

void leadScrewIn() {
  digitalWrite(D1, LOW);
  analogWrite(OUT2, 127);
  digitalWrite(OUT1, LOW);
}

void leadScrewStop() {
  digitalWrite(D1, HIGH);
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
}

void setMotorSpeed(int speed) {
  M1->setSpeed(speed);
  M2->setSpeed(speed);
  M3->setSpeed(speed);
  M4->setSpeed(speed);
}

// To turn stepper:
// step1->step(steps, FORWARD, SINGLE);
// Change FORWARD to BACKWARD for other direction

// To set servo:
// servoName.write(degrees);

enum direction{up, down, left, right};    //using the same names as the moter library names

struct position{
    double x;
    double y;
  direction looking;  //for robot, direction is where arm is pointing
            //for trees and cups, it is where the arm should be to interact with it
};

position cups[4];   //in case there are more than two cups
char cupindex;
position trees[2];
char treeindex;
position currentpos;
position precups[7];
bool foundcups[4] = {0}; //this indicates wether or not we have gotten the cup locations yet.

//coord_system.cpp
#define tocords(steps) (double)(steps/2) //experimentation will make this more precise
#define toinches(millimeters) (double)(millimeters*0.0393701) //current assumption is that distance sensors read mm and i'm using inches

const int cupdistance = 12;
const int fromcenter = 5.75;
direction pixyDir = left;

// this is the code for how the robot moves around and logs the position of the cups
void coordSetup(){
    //these are not where these objects ARE, but where the ROBOT NEEDS TO BE
    //in order to interact with the objects.
    trees[0].x = 12;
    trees[0].y = 7;
    trees[0].looking = left;
    trees[1].x = 75;
    trees[1].y = 36;
    trees[1].looking = up;

    precups[0].x = 24;
    precups[0].y = 7;
    precups[0].looking = left;
    precups[1].x = 24;
    precups[1].y = 27;
    precups[1].looking = left;
    precups[2].x = 24;
    precups[2].y = 24;
    precups[2].looking = up;
    precups[3].x = 24;
    precups[3].y = 24;
    precups[3].looking = up;
    precups[4].x = 84;
    precups[4].y = 24;
    precups[4].looking = up;
    precups[5].x = 48;
    precups[5].y = 24;
    precups[5].looking = down;
    precups[6].x = 84;
    precups[6].y = 24;
    precups[6].looking = down;


    //initial position of the robot
    currentpos.x = 24;
    currentpos.y = 6;
    //these need to be backed up with distance sensors.
}

void currentPosLog(){
    //all of these can change to satisfy overall design

    //up and left are garunteed to be seen
    double up = measure1.RangeMilliMeter/25.4 + fromcenter;
    double left = measure4.RangeMilliMeter/25.4 + fromcenter;

    //there might not be a visible wall for these sensors,
    //use these later on for double checking
    //double down = measure2.RangeMilliMeter/25.4 + fromcenter;
    //double right = measure4.RangeMilliMeter/25.4 + fromcenter;

    //these gives the distances of the robot from the center


    currentpos.x = left;
    currentpos.y = 48 - up;
    //looking must be logged outside of code since the arm can look in any direction
}

void logCup() {
  currentPosLog();
  int cupLogIndex = 0;
  while(foundcups[cupLogIndex] == 1) cupLogIndex++;
  foundcups[cupLogIndex] = 1;
  cups[cupLogIndex].x = currentpos.x;
  cups[cupLogIndex].y = currentpos.y;
  Serial.println("Cup found!");
  Serial.print("X: ");
  Serial.print(cups[cupLogIndex].x);
  Serial.print(", Y: ");
  Serial.println(cups[cupLogIndex].y);
}

Pixy2 pixy;

bool distDebug = false;
bool pixyDebug = false;

int motorDelay = 10000;
unsigned long lastMillis;
bool motorOn = false;
byte incomingByte;
byte storedByte;

enum logicState{startup, noBeads, grabbingBeads, haveBeads, droppingBeads};
logicState motorState = startup;

void centerCup() {
  while(pixy.ccc.blocks[0].m_x != 158) {
    pixy.ccc.getBlocks();
    if(currentpos.looking == up) driveForward();
    else if(currentpos.looking == down) driveBackward();
    else if(currentpos.looking == left) driveLeft();
    else if(currentpos.looking == right) driveRight();
  }
  allStop();
}

void resetSensors(){
  // reset distance sensors
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  delay(10);
  digitalWrite(XSHUT1, HIGH);
  digitalWrite(XSHUT2, HIGH);
  digitalWrite(XSHUT3, HIGH);
  digitalWrite(XSHUT4, HIGH);

  // set sensor addresses one-by-one
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  lox1.begin(0x30);
  digitalWrite(XSHUT2, HIGH);
  lox2.begin(0x31);
  digitalWrite(XSHUT3, HIGH);
  lox3.begin(0x32);
  digitalWrite(XSHUT4, HIGH);
  lox4.begin(0x33);
}

// movement speed (at 120 RPM): 1mm every 7.55ms
// so to go one inch, run motor for 191.77ms (round to 192)
const int inchMotorDelay = 192;
void driveOneInch(char dir) {
  // f for forward, b for backward, l for left, r for right
  switch(dir) {
    case 'f':
      driveForward();
      // I prefer not using delays, but it should work for now.
      // It's less than a fifth of a second, so it's not using up too much processing time
      delay(inchMotorDelay);
      allStop();
      break;
    case 'b':
      driveBackward();
      // I prefer not using delays, but it should work for now.
      // It's less than a fifth of a second, so it's not using up too much processing time
      delay(inchMotorDelay);
      allStop();
      break;
    case 'l':
      driveLeft();
      // I prefer not using delays, but it should work for now.
      // It's less than a fifth of a second, so it's not using up too much processing time
      delay(inchMotorDelay);
      allStop();
      break;
    case 'r':
      driveRight();
      // I prefer not using delays, but it should work for now.
      // It's less than a fifth of a second, so it's not using up too much processing time
      delay(inchMotorDelay);
      allStop();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  // initialize motor shields
  AFMSstep.begin();
  AFMSdc.begin();

  susan->setSpeed(30);
  extendo->setSpeed(30);
  M1->setSpeed(120);
  M2->setSpeed(120);
  M3->setSpeed(120);
  M4->setSpeed(120);

  pixy.init();
  pixy.changeProg("color_connected_components");
  pixy.setLamp(1,1);

  coordSetup();
  lastMillis = millis();

  // Initialize servo
  grabberServo.attach(GRABBER_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

  // setup MC33926 H-bridge outputs and disable
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(D1,OUTPUT);
  digitalWrite(D1, HIGH); // high is disabled, low is enabled

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  pinMode(XSHUT4, OUTPUT);

  // reset distance sensors
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  delay(10);
  digitalWrite(XSHUT1, HIGH);
  digitalWrite(XSHUT2, HIGH);
  digitalWrite(XSHUT3, HIGH);
  digitalWrite(XSHUT4, HIGH);

  // set sensor addresses one-by-one
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  lox1.begin(0x30);
  digitalWrite(XSHUT2, HIGH);
  lox2.begin(0x31);
  digitalWrite(XSHUT3, HIGH);
  lox3.begin(0x32);
  digitalWrite(XSHUT4, HIGH);
  lox4.begin(0x33);

  //M1->run(FORWARD);
  //motorOn = true;
  grabberServo.write(45); // get grabber out of the way for turning
  //extendo->step(100, FORWARD, SINGLE);

  coordSetup(); //set up coords
  
  cupindex = 0;

  clawServo.write(CLAWCLOSE);
  //while(!Serial.available()){
    
  //}
  //char incomingByte = Serial.read();
  //Serial.println(incomingByte);
  
  susan->step(QUARTER_TURN, BACKWARD, SINGLE);
  delay(1000);
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks) {
    Serial.println("CUP!");
    cups[cupindex++] = precups[0];
  }  
  delay(1000);
  driveLeft();
  delay(MM(TOMM(18)));
  allStop();
  delay(1000);
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks) {
    Serial.println("CUP!");
    cups[cupindex++] = precups[1];
  }  
  delay(1000);
  
  susan->step(QUARTER_TURN, FORWARD, SINGLE);
  susan->step(QUARTER_TURN, FORWARD, SINGLE);
  delay(1000);
  pixy.ccc.getBlocks();
  if(pixy.ccc.numBlocks) {
    //Serial.println("CUP!");
    cups[cupindex++] = precups[1];
  }
  delay(1000);
  driveRight();
  delay(MM(TOMM(18)));
  allStop();
  //.println("done");
  grabberServo.write(100);
  delay(1000);
  grabberServo.write(45);
  delay(1000);
  //Serial.println("wave1");

//*
  if(cupindex == 0){
    //wave if no cups
    grabberServo.write(1000);
    delay(1000);
    grabberServo.write(45);
    //Serial.println("NOCUPS");
  }
  else{
    //GRABBEADS
    //Serial.println("CUPS!");
    driveLeft();
    delay(MM(TOMM(14)));
    allStop();
    grabberServo.write(140);
    leadScrewOut();
    delay(6000);
    leadScrewStop();
    delay(1000);
    clawServo.write(0);
    delay(1000);
    grabberServo.write(150);
    delay(6000);
    leadScrewIn();
    delay(6000);
    leadScrewStop();
    grabberServo.write(100);
    //deal with cup in a bit
    if(cups[0].y == 7){
      driveRight();
      delay(MM(TOMM(11)));
      allStop();
      delay(1000);
      //Serial.println("DROP BEADS");
      grabberServo.write(45);
      delay(1000);
      clawServo.write(160);
      //Serial.println("done");
    }
    else if(cups[0].looking == left){
      driveLeft();
      delay(MM(TOMM(6)));
      allStop();
      delay(1000);
      //Serial.println("DROP BEADS");
      grabberServo.write(45);
      delay(1000);
      clawServo.write(160);
      delay(1000);
      //Serial.println("done");
    }
    else if(cups[0].looking == up){
      driveLeft();
      delay(MM(TOMM(6)));
      allStop();
      delay(1000);
      susan->step(QUARTER_TURN, FORWARD, SINGLE);
      susan->step(QUARTER_TURN, FORWARD, SINGLE);
      //Serial.println("DROP BEADS");
      delay(1000);
      grabberServo.write(45);
      delay(1000);
      clawServo.write(160);
      //Serial.println("done");
    }
  }//*/

  /*
  driveForward();
  delay(3452*2);
  allStop();
  //Serial.println("CLOCKWISE");
  //susan->step(QUARTER_TURN, BACKWARD, SINGLE);
  //delay(2000);
  //Serial.println("RIGHT");
  driveRight();
  delay(MM(TOMM(60)));
  allStop();
  //Serial.println("COUNTER-CLOCKWISE");
  //susan->step(QUARTER_TURN, FORWARD, SINGLE);
  //delay(2000);
  //susan->step(QUARTER_TURN, FORWARD, SINGLE);
  //delay(2000);
  //Serial.println("LEFT");
  driveLeft();
  delay(MM(TOMM(60)));
  allStop();
  //Serial.println("COUNTER-CLOCKWISE");
  //susan->step(QUARTER_TURN, FORWARD, SINGLE);
  //delay(2000);
  driveBackward();
  delay(3452*2);
  allStop();*/
}

void loop() {
  /*****************************************
   * DISTANCE SENSORS
   *****************************************/
    // set up distance sensors
    /*
    VL53L0X_RangingMeasurementData_t measure1;
    VL53L0X_RangingMeasurementData_t measure2;
    VL53L0X_RangingMeasurementData_t measure3;
    VL53L0X_RangingMeasurementData_t measure4;

    lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
    lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
    lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
*/
  /*************************************************************
  * STATE CONTROL
  **************************************************************/
  //startup noBeads haveBeads grabbingBeads droppingBeads

  //this will only drive the field
  //Serial.println("FORWARD");
  
  //susan->step(QUARTER_TURN, BACKWARD, SINGLE);
  //delay(2000);
  //susan->step(QUARTER_TURN, BACKWARD, SINGLE);
  //delay(2000);
  /*
  switch(motorState) {
    case startup: // survey the track, finding cups
      while(currentpos.y <= 21) {
        driveOneInch('f');
        allStop();

        resetSensors();
        
        currentPosLog();
        pixy.ccc.getBlocks();
        if(pixy.ccc.numBlocks) { // if blocks are detected
          allStop();
          centerCup();
          logCup();
        }
      }
      allStop();
      //susan->step(QUARTER_TURN, FORWARD, SINGLE); // turn to the right
      currentpos.looking = right;
      while(currentpos.x <= 90) {
        driveOneInch('r');
        allStop();
        resetSensors();
        currentPosLog();
        pixy.ccc.getBlocks();
        if(pixy.ccc.numBlocks) {
          allStop();
          centerCup();
          logCup();
        }
      }
      allStop();
      //susan->step(QUARTER_TURN*2, BACKWARD, SINGLE); // turn around
      currentpos.looking = left;
      while(currentpos.x > 24) {
        driveOneInch('l');
        allStop();
        resetSensors();
        currentPosLog();
        pixy.ccc.getBlocks();
        if(pixy.ccc.numBlocks) {
          allStop();
          centerCup();
          logCup();
        }
      }
      allStop();
      //susan->step(QUARTER_TURN, BACKWARD, SINGLE); // face backward
      currentpos.looking = down;
      while(currentpos.y > 6) {
        driveOneInch('b');
        allStop();
        resetSensors();
        currentPosLog();
        pixy.ccc.getBlocks();
        if(pixy.ccc.numBlocks) {
          allStop();
          centerCup();
          logCup();
        }
      }
      allStop();
      //motorState = noBeads;
      break;
    case noBeads:
      //go to first tree
      //susan->step(QUARTER_TURN*2, FORWARD, SINGLE); // face forward
      currentpos.looking = up;
      while(currentpos.y < trees[0].y) {
        currentPosLog();
        driveForward();
      }
      allStop();
      motorState = grabbingBeads;
      break;
    case grabbingBeads:
      clawServo.write(clawClosedDegrees);
      grabberServo.write(ROTATEY_FULL_UP);
      leadScrewOut();
      delay(7000);
      leadScrewStop();
      clawServo.write(clawOpenDegrees);
      leadScrewIn();
      delay(3500);
      leadScrewStop();
      grabberServo.write(ROTATEY_UP);
      leadScrewIn();
      delay(3500);
      leadScrewStop();
      grabberServo.write(ROTATEY_CUP);
      motorState = haveBeads;
      break;
    case haveBeads:
      // go to nearest cup
      break;
    case droppingBeads:
      // put in cup
      break;
    default:
      break;
  }*/

}
