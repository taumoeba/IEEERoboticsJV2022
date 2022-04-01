// Currently configured for testing
// (also yes I know this is poorly written)

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

#include <Adafruit_MotorShield.h>
#include <Servo.h>
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
#define ROTATEY_CUP 40

//number of steps to get a 90degree turn from susan
#define QUARTER_TURN 450  //testing needed to get precise amount

#define XSHUT1 4
#define XSHUT2 7
#define XSHUT3 11
#define XSHUT4 12

#define clawOpenDegrees 45 // Tested
#define clawClosedDegrees 105 // Tested

Adafruit_MotorShield AFMSstep(0x61); // Rightmost jumper closed, bottom, stepper motors
Adafruit_MotorShield AFMSdc(0x60); // Default address, no jumpers, top, dc motors
Adafruit_StepperMotor *susan = AFMSstep.getStepper(200, 1); // lazy susan
Adafruit_StepperMotor *extendo = AFMSstep.getStepper(200, 2); // raise arm

Adafruit_DCMotor *M1 = AFMSdc.getMotor(1);
Adafruit_DCMotor *M2 = AFMSdc.getMotor(2);
Adafruit_DCMotor *M3 = AFMSdc.getMotor(3);
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

void allStop() {
  M1->run(RELEASE);
  M2->run(RELEASE);
  M3->run(RELEASE);
  M4->run(RELEASE);
}

// Drive motors will continue driving until allStop() is called or an individual motor is stopped
void driveBackward() {
  allStop();
  M1->run(FORWARD);
  M3->run(BACKWARD);
}

void driveForward() {
  allStop();
  M1->run(BACKWARD);
  M3->run(FORWARD);
}

void driveLeft() {
  allStop();
  M2->run(BACKWARD);
  M4->run(FORWARD);
}

void driveRight() {
  allStop();
  M2->run(FORWARD);
  M4->run(BACKWARD);
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
    precups[3].x = 48;
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

//when a cup is found, log the position
void logcup(){
    //this would be after centering on the cup
    position newcup;
    newcup = currentpos;
    newcup.looking = pixyDir;
    switch(newcup.looking){
        case left:
            if(abs(newcup.y - precups[0].y) < abs(newcup.y - precups[1].y)){
                cups[cupindex++] = precups[0];
            }
            else{
                cups[cupindex++] = precups[1];
            }
            break;
        case up:
            if(abs(newcup.x - precups[2].x) < abs(newcup.x - precups[3].x && abs(newcup.x - precups[2].x) < abs(newcup.x - precups[4].x))){
                cups[cupindex++] = precups[2];
            }
            else if(abs(newcup.x - precups[3].x) < abs(newcup.x - precups[2].x && abs(newcup.x - precups[3].x) < abs(newcup.x - precups[4].x))){
                cups[cupindex++] = precups[3];
            }
            else{
                cups[cupindex++] = precups[4];
            }
            break;
        case down:
            if(abs(newcup.x - precups[5].x) < abs(newcup.x - precups[6].x)){
                cups[cupindex++] = precups[5];
            }
            else{
                cups[cupindex++] = precups[6];
            }
            break;
    }
}

void currentPosLog(){
    //all of these can change to satisfy overall design

    //up and left are garunteed to be seen
    double up = measure2.RangeMilliMeter/25.4 + fromcenter;
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

Pixy2 pixy;

bool allClear = true; // set to false in final version, true for testing
bool allClearOld = false;
bool distDebug = false;
bool pixyDebug = false;

//bool foundCup();    //to be written with pixy camera
bool foundcups = false; //this indicates wether or not we have gotten the cup locations yet.
bool foundcupsState = false;

int motorDelay = 10000;
unsigned long lastMillis;
bool motorOn = false;
byte incomingByte;
byte storedByte;

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
  clawServo.write(145);
}

/*
  The robot has two modes, cup hunt/traverse board and grabbing beads/placing in cups
*/

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
  */
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!

/*
  if (measure3.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure3.RangeMilliMeter);
  } else {
    Serial.println(" sensor 3 out of range ");
  }

  if (measure4.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure4.RangeMilliMeter);
  } else {
    Serial.println(" sensor 4 out of range ");
  }
*/
  // If the range on Sensor1 is less than 450mm, slow down. If it's less than 250mm, stop
  if(measure1.RangeMilliMeter >= 450) { //change to RangeInches
      allClear = true;
      setMotorSpeed(100);
    }
    else if(measure1.RangeMilliMeter >= 250) {
      allClear = true;
      setMotorSpeed(60);
    }
    else if(measure1.RangeMilliMeter < 250) {
      allClear = false;
    }

  /************************************************
   * PIXY CAMERA
   ***********************************************/

   pixy.ccc.getBlocks();
   
   // If there are detected blocks, stop driving
   if (pixy.ccc.numBlocks)
   {
    foundcups = true; // not final
    //Serial.println("********CUP DETECTED*****");
    //allClear = false; // stop driving
    /*
     Serial.print("Detected ");
     Serial.println(pixy.ccc.numBlocks);
     for (int i=0; i<pixy.ccc.numBlocks; i++)
     {
       Serial.print("  block ");
       Serial.print(i);
       Serial.print(": ");
       pixy.ccc.blocks[i].print();
     }
     */

   }

  /******************************************
   * MOTORS
   *****************************************/
   /*
   // only drive if sensors reporting all clear
   if(allClear != allClearOld) {
    if(allClear) {
      drive.forward();
    } else {
      drive.allStop();
    }
    allClearOld = allClear;
   }
   */

  /************************************************
   * ROBOT STATE CONTROL
   ***********************************************/

  //we must first raise the arm so we can rotate
  grabberServo.write(20);

  if(!foundcupsState){
    //start here, this is where we go through the board
    //we should be foward facing
    driveForward();
    while(currentpos.y <= 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        allStop();
        Serial.println("Here be cups"); // replace with actual code
        foundcups = false;
        delay(1000);
        driveForward(); //continue moving
      }
    }
    allStop();

    //are going to turn to face top wall
    susan->step(QUARTER_TURN, FORWARD, SINGLE); // test
    currentpos.looking = right;

    driveLeft();
    while(currentpos.x <= 90){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        allStop();
        Serial.println("Here be cups"); // replace with actual code
        foundcups = false;
        delay(1000);
        driveRight(); //continue moving
      }
    }
    allStop();

    susan->step(QUARTER_TURN*2, BACKWARD, SINGLE);
    currentpos.looking = left;

    driveRight();
    while(currentpos.x > 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        allStop();
        Serial.println("Here be cups"); // replace with actual code
        foundcups = false;
        delay(1000);
        driveRight(); //continue moving
      }
    }
    allStop();

    susan->step(QUARTER_TURN, BACKWARD, SINGLE);
    currentpos.looking = down;

    driveBackward();
    while(currentpos.y > 6){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        allStop();
        Serial.println("Here be cups"); // replace with actual code
        foundcups = false;
        delay(1000);
        driveBackward(); //continue moving
      }
    }
    allStop();
    foundcupsState = 1;
  }
  else{
    //this is where we will grab the beads off the trees and put into the cups,
    //we will probably want to loop through this multiple times to grab all beads, maybe 4 times at most?

    //if in short part, move up/down, else move left right
    for(int i = 0; i < cupindex; ++i){
      //we need to go to the first tree
      treeindex = i % 2; //in case there is more than 2 cups, make sure that i is either 0 or 1

      //we need to figure out where we are on the board and where we are in relation to the trees
      int xdif = trees[treeindex].x - currentpos.x;
      int ydif = trees[treeindex].y - currentpos.y;

      //check to see if we need to move in two directions both ways
      if(xdif > 0 && ydif > 0){
        driveForward();
        while(currentpos.y <= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        driveLeft();
        while(currentpos.x <= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0 && ydif < 0){
        driveRight();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        driveBackward();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      //single directional movement
      else if(xdif > 0){
        driveForward();
        while(currentpos.y <= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0){
        driveBackward();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif > 0){
        driveLeft();
        while(currentpos.x <= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif < 0){
        driveRight();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }

      //GRAB BEADS CODE HERE!!!

      xdif = cups[i].x - currentpos.x;
      ydif = cups[i].y - currentpos.y;

      //now need to get to cups

      if(xdif > 0 && ydif > 0){
        driveForward();
        while(currentpos.y <= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        driveLeft();
        while(currentpos.x <= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0 && ydif < 0){
        driveRight();
        while(currentpos.x >= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        driveBackward();
        while(currentpos.y >= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      //single directional movement
      else if(xdif > 0){
        driveForward();
        while(currentpos.y <= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0){
        driveBackward();
        while(currentpos.y >= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif > 0){
        driveLeft();
        while(currentpos.x <= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif < 0){
        driveRight();
        while(currentpos.x >= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }

      //BEAD DROP CODE

      //once we loop, we will go back to the first tree to grab
      //beads that we have missed.
    }
  }

}
