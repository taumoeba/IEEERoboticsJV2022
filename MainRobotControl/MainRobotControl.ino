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
 * - build wrapper libraries for distance sensors, camera
 * - Write out main control algorithm
 * - Verify all motor numbers, directions, step counts, etc.
 */

#include "MainRobotControl.h"


void setup() {
  Serial.begin(115200);

  pixy.init();
  pixy.changeProg("color_connected_components");

  coordSetup();

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
      drive.setSpeed(100);
    }
    else if(measure1.RangeMilliMeter >= 250) {
      allClear = true;
      drive.setSpeed(60);
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
      drive.forward();
    } else {
      drive.allStop();
    }
    allClearOld = allClear;
   }

  /************************************************
   * ROBOT STATE CONTROL
   ***********************************************/

  //we must first raise the arm so we can rotate
  raiseArm(10);

  if(!foundcups){
    //start here, this is where we go through the board
    //we should be foward facing
    drive.forward();
    while(currentpos.y <= 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        drive.allStop();
          //do cuplog stuff here
        drive.forward(); //continue moving
      }
    }
    drive.allStop();

    //are going to turn to face top wall
    arm.turnSusan(0);
    currentpos.looking = right;

    drive.left();
    while(currentpos.x <= 90                         ){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        drive.allStop();
          //do cuplog stuff here
        drive.left(); //continue moving
      }
    }
    drive.allStop();

    arm.turnSusan(0);
    arm.turnSusan(0);
    currentpos.looking = left;

    drive.right();
    while(currentpos.x > 24){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        drive.allStop();
          //do cuplog stuff here
        drive.right(); //continue moving
      }
    }
    drive.allStop();

    arm.turnSusan(1);
    currentpos.looking = down;

    drive.reverse();
    while(currentpos.y > 6){  //the turning point of the robot
      currentPosLog();  //update position
      if(foundcups){
        drive.allStop();
          //do cuplog stuff here
        drive.reverse(); //continue moving
      }
    }
    drive.allStop();
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
        drive.forward();
        while(currentpos.y <= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
        drive.left();
        while(currentpos.x <= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(xdif < 0 && ydif < 0){
        drive.right();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
        drive.reverse();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      //single directional movement
      else if(xdif > 0){
        drive.forward();
        while(currentpos.y <= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(xdif < 0){
        drive.reverse();
        while(currentpos.y >= trees[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(ydif > 0){
        drive.left();
        while(currentpos.x <= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(ydif < 0){
        drive.right();
        while(currentpos.x >= trees[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }

      //GRAB BEADS CODE HERE!!!

      xdif = cups[i].x - currentpos.x;
      ydif = cups[i].y - currentpos.y;

      //now need to get to cups

      if(xdif > 0 && ydif > 0){
        drive.forward();
        while(currentpos.y <= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
        drive.left();
        while(currentpos.x <= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(xdif < 0 && ydif < 0){
        drive.right();
        while(currentpos.x >= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
        drive.reverse();
        while(currentpos.y >= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      //single directional movement
      else if(xdif > 0){
        drive.forward();
        while(currentpos.y <= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(xdif < 0){
        drive.reverse();
        while(currentpos.y >= cups[i].y){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(ydif > 0){
        drive.left();
        while(currentpos.x <= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }
      else if(ydif < 0){
        drive.right();
        while(currentpos.x >= cups[i].x){  //the turning point of the robot
          currentPosLog();  //update position
        }
        drive.allStop();
      }

      //BEAD DROP CODE

      //once we loop, we will go back to the first tree to grab
      //beads that we have missed.
    }
  }

}
