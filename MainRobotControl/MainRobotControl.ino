#include <Adafruit_MotorShield.h>

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

/* TODO:
 * - implement classes for motor and arm libaries
 * - build wrapper libraries for distance sensors, camera
 * - Write out main control algorithm
 * - Verify all motor numbers, directions, step counts, etc.
 */

#include "simple_motor.h"
#include "coord_system.h"
#include "MainRobotControl.h"
#include "Adafruit_VL53L0X.h"
#include <Pixy2.h>


void setup() {
  Serial.begin(115200);
  initializeMotors();

  pixy.init();
  pixy.changeProg("color_connected_components");

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  pinMode(XSHUT4, OUTPUT);

  // reset TOF sensors
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

/*******************************************
 * FUNCTION DEFINITIONS -- MOVE TO LIBRARY
 *******************************************/
/*
 void distSenseSetup() {
  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;
  VL53L0X_RangingMeasurementData_t measure4;

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
 }

 void printRanges() {
  if (measure1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure1.RangeMilliMeter);
  } else {
    Serial.println(" sensor 1 out of range ");
  }
  if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 2 (mm): "); Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(" sensor 2 out of range ");
  }
  
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
 }
 */

/*
  The robot has two modes, cup hunt/traverse board and grabbing beads/placing in cups
*/
bool foundcups = false; //this indicates wether or not we have gotten the cup locations yet.

void loop() {
/*****************************************
 * DISTANCE SENSORS
 *****************************************/
  // The distance sensor setup code doesn't like being in a function. Just leave it here.
  // set up distance sensors
  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;
  VL53L0X_RangingMeasurementData_t measure4;

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!

  // print out distances to serial
  if (measure1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure1.RangeMilliMeter);
  } else {
    Serial.println(" sensor 1 out of range ");
  }
  if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 2 (mm): "); Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(" sensor 2 out of range ");
  }
  
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

  // If the range on Sensor1 is less than 450mm, slow down. If it's less than 250mm, stop
  if(measure1.RangeMilliMeter >= 450) { //change to RangeInches
      allClear = true;
      setSpeed(1,100);
      setSpeed(2,100);
      setSpeed(3,100);
      setSpeed(4,100);
    }
    else if(measure1.RangeMilliMeter >= 250) {
      allClear = true;
      setSpeed(1,60);
      setSpeed(2,60);
      setSpeed(3,60);
      setSpeed(4,60);
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
    Serial.println("********CUP DETECTED*****");
    allClear = false; // stop driving
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
   // only drive if sensors reporting all clear
   if(allClear != allClearOld) {
    if(allClear) {
      driveUp();
    } else {
      allStop();
    }
    allClearOld = allClear;
   }
  /*  // Testing all motor functions


  // Main motors
  setSpeed(1,100);
  setSpeed(2,100);
  setSpeed(3,100);
  setSpeed(4,100);
  
  driveUp();
  delay(6000);
  driveDown();
  delay(6000);
  allStop();
  delay(3000);
  driveRight();
  delay(1500);
  driveLeft();
  delay(1500);
  allStop();
  
  
  driveRight();
  delay(1500);
  driveLeft();
  delay(1500);
  allStop();
  
  setSpeed(1,120);
  setSpeed(2,120);
  //setSpeed(3,120);
  setSpeed(4,120);
  driveUp();
  delay(1000);
  stopMotor(1);
  //delay(1000);
  //stopMotor(3);
  delay(1000);
  driveRight();
  delay(1000);
  stopMotor(2);
  
  delay(1000);
  stopMotor(4);
  delay(1000);
  driveDown();
  delay(2000);
  allStop();
  
  // grabber arm
  delay(5000);
  clockwiseSusan(100);
  delay(2000);
  counterSusan(100);
  delay(2000);
  extendArm(50);
  delay(2000);
  retractArm(50);
  delay(2000);
  raiseGrabber();
  delay(2000);
  lowerGrabber();
  delay(2000);
  extendScrew(100);
  delay(2000);
  retractScrew(100);
  delay(2000);
  extendSolenoid();
  delay(2000);
  retractSolenoid();
  delay(6000);
  */
  delay(50);

  /************************************************
   * ROBOT STATE CONTROL
   ***********************************************/

  if(!foundcups){
    //start here, this is where we go through the board
    //we should be foward facing
    foward();
    while(currentpos.y =< 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundCup()){
        allStop();
          //do cuplog stuff here
        foward(); //continue moving
      }
    }
    allStop();

    //are going to turn to face top wall
    turnSusan(0);

    left();
    while(currentpos.x =< 90                         ){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundCup()){
        allStop();
          //do cuplog stuff here
        left(); //continue moving
      }
    }
    allStop();

    turnSusan(0);
    turnSusan(0);

    right();
    while(currentpos.x > 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundCup()){
        allStop();
          //do cuplog stuff here
        right(); //continue moving
      }
    }
    allStop();

    turnSusan(0);

    reverse();
    while(currentpos.y > 6){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundCup()){
        allStop();
          //do cuplog stuff here
        reverse(); //continue moving
      }
    }
    allStop();
    foundcups = 1;
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
        foward();
        while(currentpos.y =< trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        left();
        while(currentpos.x =< trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0 && ydif < 0){
        right();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
        reverse();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      //single directional movement
      else if(xdif > 0){
        foward();
        while(currentpos.y =< trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(xdif < 0){
        reverse();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif > 0){
        left();
        while(currentpos.x =< trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      else if(ydif < 0){
        right();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        allStop();
      }
      
      //GRAB BEADS CODE HERE!!!

      xdif = cups[i].x - currentpos.x;
      ydif = cups[i].y - currentpos.y;
    }
  }

}
