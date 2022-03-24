#include <Adafruit_MotorShield.h>

/*******************************************************************************************
 * Program for testing 2022 Lipscomb IEEE Robotics robot
 ******************************************************************************************/

#include "simple_motor.h"
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

bool pixyDebug = false;
bool distDebug = false;
byte incomingByte;

// testing needed
int grabberPosition = 30;

void setup() {
  Serial.begin(115200);
  initializeMotors();

/*
  pixy.init();
  pixy.changeProg("color_connected_components");
*/
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

void printPixyInfo() {
    if (pixy.ccc.numBlocks)
   {
    //Serial.println("********CUP DETECTED*****");
    //allClear = false; // stop driving
     Serial.print("Detected ");
     Serial.println(pixy.ccc.numBlocks);
     for (int i=0; i<pixy.ccc.numBlocks; i++)
     {
       Serial.print("  block ");
       Serial.print(i);
       Serial.print(": ");
       pixy.ccc.blocks[i].print();
     }
   }
}


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

  // print out distances to serial if debug is enabled
  if(distDebug) {
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

  // If the range on Sensor1 is less than 450mm, slow down. If it's less than 250mm, stop
  if(measure1.RangeMilliMeter >= 450) {
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

   //pixy.ccc.getBlocks();

   if(pixyDebug) {
    printPixyInfo();
   }

   /**************************************************
    * Serial
    * 
    * Commands:
    * w: Move forwards
    * a: Move left
    * s: Move backwards
    * d: Move right
    * n: Turn lazy susan counter-clockwise 1 step
    * N: Turn lazy susan counter-clockwise 10 steps
    * m: Turn lazy susan clockwise 1 step
    * M: Turn lazy susan clockwise 10 steps
    * t: Raise arm 1 step
    * T: Raise arm 10 steps
    * g: Lower arm 1 step
    * G: Lower arm 10 steps
    * y: Raise grabber 10 degrees
    * Y: Raise grabber all the way
    * h: Lower grabber 10 degrees
    * H: Lower grabber all the way
    * u: Extend lead screw 1 step
    * U: Extend lead screw 10 steps
    * j: Retract lead screw 1 step
    * J: Retract lead screw 10 steps
    * i: Open claw
    * k: Close claw
    * q: Stop All Driving Motors
    * z: Emergency abort: Stop motors, turn on all debugs, move servos to neutral positions
    * p: Print single Pixy debug line
    * P: Toggle continuous pixy debug
    * o: Print single distance sensor debug line
    * O: Toggle continuous distance sensor debug
    ***************************************************/
  if(Serial.available()) {
    incomingByte = Serial.read();
    switch(incomingByte) {
      case 'w':
      case'W':
        allStop();
        driveUp();
        break;
      case 'a':
      case 'A':
        allStop();
        driveLeft();
        break;
      case 's':
      case 'S':
        allStop();
        driveDown();
        break;
      case 'd':
      case 'D':
        allStop();
        driveRight();
        break;
      case 'n':
        counterSusan(1);
        break;
      case 'N':
        counterSusan(10);
        break;
      case 'm':
        clockwiseSusan(1);
        break;
      case 'M':
        clockwiseSusan(10);
        break;
      case 't':
        raiseArm(1);
        break;
      case 'T':
        raiseArm(10);
        break;
      case 'g':
        lowerArm(1);
        break;
      case 'G':
        lowerArm(10);
        break;
      case 'y':
        grabberPosition += 10;
        setGrabber(grabberPosition);
        break;
      case 'Y':
        grabberPosition = 180;
        setGrabber(grabberPosition);
        break;
      case 'h':
        grabberPosition -= 10;
        setGrabber(grabberPosition);
        break;
      case 'H':
        grabberPosition = 30;
        setGrabber(grabberPosition);
        break;
      case 'u':
        extendScrew(1);
        break;
      case 'U':
        extendScrew(10);
        break;
      case 'j':
        retractScrew(1);
        break;
      case 'J':
        retractScrew(10);
        break;
      case 'i':
      case 'I':
        openClaw();
        break;
      case 'k':
      case 'K':
        closeClaw();
        break;
      case 'q':
      case 'Q':
        allStop();
        break;
      case 'z':
      case 'Z':
        allStop();
        setGrabber(45);
        pixyDebug = true;
        distDebug = true;
        break;
      case 'p':
        printPixyInfo();
        break;
      case 'P':
        pixyDebug = !pixyDebug;
        break;
      case 'o':
      // FIX
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
        break;
      case 'O':
        distDebug = !distDebug;
        break;
      default:
        break;
    }
  }
}
