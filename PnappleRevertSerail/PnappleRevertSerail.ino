// Currently configured for testing

/*******************************************************************************************
   Main program for LU JV IEEE Robotics 2022
   Written by Ben Powell and Stephen Fulton
   April 2022

   Microcontroller: Arduino Mega 2560
   Components List:
   - Adafruit Motor Control Shield v2.3
   - 4x DC Motor
   - 2x Stepper Motor
   - 2x Servo Motor
   - Pixy2 Smart Vision Sensor

   Reference schematic.png for connection details
 ******************************************************************************************/

//#include "MainRobotControl.h"

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <Pixy2.h>
#include "Adafruit_VL53L0X.h"
//#include "music.h"

// peizo buzzer pin
//#define MUSIC_PIN 47

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
#define ROTATEY_RIGHT 100
#define ROTATEY_CUP 40
#define ROTATEY_TREE1 140
#define ROTATEY_TREE2 180

//number of steps to get a 90degree turn from susan
#define QUARTER_TURN 560  //testing needed to get precise amount

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

enum direction {up, down, left, right};   //using the same names as the moter library names

struct position {
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
#define TOCORDS(steps) (double)(steps/2) //experimentation will make this more precise
#define TOINCHES(millimeters) (double)(millimeters*0.0393701) //current assumption is that distance sensors read mm and i'm using inches
#define TOMM(inches) (double)(inches/0.0393701) //convert from inches to mm
#define MM(mm) (int)(mm*7.55) //turns millimeters into milliseconds

const int cupdistance = 12;
const int fromcenter = 5.75;
direction pixyDir = left;

// this is the code for how the robot moves around and logs the position of the cups
void coordSetup() {
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
void logcup() {
  //this would be after centering on the cup
  position newcup;
  newcup = currentpos;
  newcup.looking = pixyDir;
  switch (newcup.looking) {
    case left:
      if (abs(newcup.y - precups[0].y) < abs(newcup.y - precups[1].y)) {
        cups[cupindex++] = precups[0];
      }
      else {
        cups[cupindex++] = precups[1];
      }
      break;
    case up:
      if (abs(newcup.x - precups[2].x) < abs(newcup.x - precups[3].x && abs(newcup.x - precups[2].x) < abs(newcup.x - precups[4].x))) {
        cups[cupindex++] = precups[2];
      }
      else if (abs(newcup.x - precups[3].x) < abs(newcup.x - precups[2].x && abs(newcup.x - precups[3].x) < abs(newcup.x - precups[4].x))) {
        cups[cupindex++] = precups[3];
      }
      else {
        cups[cupindex++] = precups[4];
      }
      break;
    case down:
      if (abs(newcup.x - precups[5].x) < abs(newcup.x - precups[6].x)) {
        cups[cupindex++] = precups[5];
      }
      else {
        cups[cupindex++] = precups[6];
      }
      break;
  }
}

void currentPosLog() {
  //all of these can change to satisfy overall design

  //up and left are garunteed to be seen
  double up = measure1.RangeMilliMeter / 25.4 + fromcenter;
  double left = measure3.RangeMilliMeter / 25.4 + fromcenter;

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

bool foundCup();    //to be written with pixy camera

int motorDelay = 10000;
unsigned long lastMillis;
bool motorOn = false;
byte incomingByte;
byte storedByte;
int rotateyPosition = 45;

void setup() {
  Serial.begin(115200);
  // initialize motor shields
  AFMSstep.begin();
  AFMSdc.begin();

  susan->setSpeed(30);
  extendo->setSpeed(30);
  M1->setSpeed(RPM);
  M2->setSpeed(RPM);
  M3->setSpeed(RPM);
  M4->setSpeed(RPM);

  pixy.init();
  pixy.changeProg("color_connected_components");
  pixy.setLamp(1, 1);

  coordSetup();
  lastMillis = millis();

  // Initialize servo
  grabberServo.attach(GRABBER_SERVO_PIN);
  clawServo.attach(CLAW_SERVO_PIN);

   // setup piezo buzzer
  pinMode(MUSIC_PIN, OUTPUT);

  // setup MC33926 H-bridge outputs and disable
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(D1, OUTPUT);
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
  grabberServo.write(30);
  pixy.setLamp(0, 0);
  digitalWrite(SDA,0);
  digitalWrite(SCL,0);
}

/*
  The robot has two modes, cup hunt/traverse board and grabbing beads/placing in cups
*/
bool foundcups = false; //this indicates wether or not we have gotten the cup locations yet.

void loop() {
  /*****************************************
     DISTANCE SENSORS
   *****************************************/
  // set up distance sensors
  //Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
  //Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
  //Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
  //Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;
  VL53L0X_RangingMeasurementData_t measure4;

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  /*
    if(distDebug) {
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
    // If the range on Sensor1 is less than 450mm, slow down. If it's less than 250mm, stop
    if(measure4.RangeMilliMeter >= 450) { //change to RangeInches
        allClear = true;
        setMotorSpeed(120);
        //Serial.println(F("All clear"));
      }
      else if(measure4.RangeMilliMeter >= 250) {
        allClear = true;
        setMotorSpeed(60);
        //Serial.println(F("Caution"));
      }
      else if(measure4.RangeMilliMeter < 250) {
        allClear = false;
        //Serial.println(F("STOP!"));
      }
      //Serial.println(measure4.RangeMilliMeter);
  */

  if (distDebug) {
    Serial.print("Sensor 1: ");
    Serial.println(measure1.RangeMilliMeter);
    Serial.print("Sensor 2: ");
    Serial.println(measure2.RangeMilliMeter);
    Serial.print("Sensor 3: ");
    Serial.println(measure3.RangeMilliMeter);
    Serial.print("Sensor 4: ");
    Serial.println(measure4.RangeMilliMeter);
  }

  /************************************************
     PIXY CAMERA
   ***********************************************/

  pixy.ccc.getBlocks();
  /*   if(pixy.ccc.numBlocks) {
       //Serial.println("**********CUP DETECTED***********");
       allClear = false;
     }
  */

  if (pixyDebug) {
    // If there are detected blocks, stop driving
    if (pixy.ccc.numBlocks)
    {
      //Serial.println("********CUP DETECTED*****");
      //allClear = false; // stop driving

      Serial.print("Detected ");
      Serial.println(pixy.ccc.numBlocks);
      for (int i = 0; i < pixy.ccc.numBlocks; i++)
      {
        Serial.print("  block ");
        Serial.print(i);
        Serial.print(": ");
        pixy.ccc.blocks[i].print();
      }
    }


  }


  /******************************************
     MOTORS
   *****************************************/
  // only drive if sensors reporting all clear
  /*
    if(allClear != allClearOld) {
    if(allClear) {
     driveForward();
     //leadScrewOut();
     //M1->run(FORWARD);
    } else {
     allStop();
     //leadScrewStop();
    }
    allClearOld = allClear;
    }
  */
  /*
    delay(3000);
    driveForward();
    delay(5000);
    allStop();
    delay(3000);
    driveBackward();
    delay(5000);
    allStop();
  */
  /*
    if(millis() - lastMillis >= motorDelay) {
      if(!motorOn) {
        M1->run(FORWARD);
        motorOn = true;
      }
      else {
        //M1->run(RELEASE);
        //motorOn = false;
        soft_restart();
      }

      //digitalWrite(XSHUT4, LOW);
      //delay(10);
      //digitalWrite(XSHUT4, HIGH);
      lastMillis = millis();

    }
  */

  /************************************************
     ROBOT STATE CONTROL VIA SERIAL
    Commands:
      w: Move forwards
      a: Move left
      s: Move backwards
      d: Move right
      n: Turn lazy susan counter-clockwise 1 position
      m: Turn lazy susan clockwise 90 degrees
      t: Susan Sweep
      g: Raise grabber more
      y: Raise grabber
      h: Lower grabber
      b: Extend lead screw
      j: Retract lead screw
      i: Open claw
      k: Close claw
      f: Claw sweep
      q: Stop All Driving Motors
      z: Emergency abort: Stop motors, turn on all debugs, move servos to neutral positions
      P: Toggle continuous pixy debug
      O: Toggle continuous distance sensor debug
      c: buzzer?
      b: Reset distance sensor I2C addresses
   ***********************************************/
  if (Serial.available()) {
    incomingByte = Serial.read();                  // read in character
    if (incomingByte == char(13)) {
      switch (storedByte) {
        case 'f':
        case 'F':
          Serial.println("Claw Sweep");
          for(int i=0; i<360; i+=10){
            clawServo.write(i);
            Serial.println(i);
            delay(1000);
          }
          Serial.println("Claw Sweep");
          for(int i=360; i>0; i-=10){
            clawServo.write(i);
            Serial.println(i);
            delay(1000);
          }
        case 'i':
        case 'I':
          Serial.println("claw open");
          clawServo.write(20);
          break;
        case 'k':
        case 'K':
          Serial.println("claw close");
          clawServo.write(160);
          break;
        case 'w':
        case 'W':
          Serial.println("Forward");
          driveForward();
          break;
        case 'a':
        case 'A':
          Serial.println("Left");
          driveLeft();
          break;
        case 's':
        case 'S':
          Serial.println("Reverse");
          driveBackward();
          break;
        case 'd':
        case 'D':
          Serial.println("Right");
          driveRight();
          break;
        case 'q':
        case 'Q':
          Serial.println("Drive Stop");
          allStop();
          break;
        case 'n':
        case 'N':
          Serial.println("Counter-clockwise susan");
          susan->step(QUARTER_TURN, FORWARD, SINGLE);
          break;
        case 'm':
        case 'M':
          Serial.println("Clockwise susan");
          susan->step(QUARTER_TURN, BACKWARD, SINGLE);
          break;
        /*case 't':
          case 'T':
          extendo->step(ARM_EXTEND_LENGTH, BACKWARD, SINGLE); // backward is up
          break;
          case 'g':
          case 'G':
          extendo->step(ARM_EXTEND_LENGTH, FORWARD, SINGLE);
          break;*/
        case 't':
        case 'T':
          for(int i=0; i<1000; i += 10){
            susan->step(10, FORWARD, SINGLE);
            Serial.println(i);
            delay(100);
          }
          break;
        case 'y':
        case 'Y':
          grabberServo.write(ROTATEY_RIGHT);
          break;
        case 'h':
        case 'H':
          grabberServo.write(ROTATEY_DOWN);
          break;
        case 'g':
        case 'G':
          grabberServo.write(ROTATEY_TREE1);
          break;
        case 'p':
        case 'P':
          pixyDebug = !pixyDebug;
          break;
        case 'o':
        case 'O':
          distDebug = !distDebug;
          break;
        case 'u':
        case 'U':
          leadScrewOut();
          delay(1000);
          leadScrewStop();
          break;
        case 'j':
        case 'J':
          leadScrewIn();
          delay(1000);
          leadScrewStop();
          break;
        case 'z':
        case 'Z':
          allStop();
          pixyDebug = 0;
          distDebug = 0;
        case 'c':
        case 'C':
          Serial.println("MUSIC!");
          play_song(); // play music using music.h
          Serial.println("MUSIC DONE!");
          break;
        case 'b':
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
          break;
        default:
          Serial.println("Unknown command");
          break;
      }
    }
    else {
      Serial.print(char(incomingByte));
    }
    storedByte = incomingByte;
  }


}
