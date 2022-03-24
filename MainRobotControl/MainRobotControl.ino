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
 * - 4x Adafruit VL53L0X Time-of-Flight Sensor
 * - Pixy2 Smart Vision Sensor
 * 
 * Reference schematic.png for connection details
 ******************************************************************************************/

/* TODO:
 * - implement actual classes for motor and arm libaries
 * - build wrapper libraries for distance sensors, camera ??
 * - Write out main control algorithm
 * - Verify all motor numbers, directions, step counts, etc.
 */

#include "simple_motor.h"
#include "coord_system.h"
#include "Adafruit_VL53L0X.h"
#include <Pixy2.h>

Pixy2 pixy;

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

#define XSHUT1 16
#define XSHUT2 17
#define XSHUT3 18
#define XSHUT4 19

bool allClear = true; // set to false in final version, true for testing
bool allClearOld = false;



void currentPosLog(){
    //all of these can change to satisfy overall design

    //up and left are garunteed to be seen
    double up = measure1.RangeInches + fromcenter;
    double left = measure3.RangeInches + fromcenter;

    //there might not be a visible wall for these sensors,
    //use these later on for double checking
    //double down = measure2.RangeInches + fromcenter;
    //double right = measure4.RangeInches + fromcenter;

    //these gives the distances of the robot from the center


    currentpos.x = left;
    currentpos.y = 48 - up;
}

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
  /*
  // Testing all motor functions

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
}
