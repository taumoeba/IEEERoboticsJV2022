/**************************************************************************
 * Lipscomb IEEE Robotics 2022
 * Motor Control Library
 * Written by Ben Powell
 * Using Adafruit Motor Shield to control 4 DC motors.
 * M2 and M4 are aligned with the 'X' axis, and M1 and M3 with the 'Y' axis.
 *************************************************************************/

//  THIS IS UNFINISHED, DO NOT USE

 /* Shape of field. "Up" is y+. "Down" is y-. "Left" is x-. "Right" is x+.
 Robot starts at A.

    ----------------------------------------------------
	|         ^                                          |
	|         y+                                         |
	|    < x-   x+ >                                     |
	|         y-	                                       |
	|         v                                          |
	|                -------------------------------------
	|                |
	|                |
	|                |
	|       A        |
	|                |
	|                |
	------------------

*/

#include "simple_motor.h"

// 200 steps per revolution (probably)
#define MOTOR_STEPS 200
#define RPM 120
#define DIR1 26
#define STEP1 27
#define DIR2 32
#define STEP2 33
#define DIR3 38
#define STEP3 39
#define GRABBER_SERVO_PIN 9
#define CLAW_SERVO_PIN 10
#define SOLENOID_PIN 12

//number of steps to get a 90degree turn from susan
#define QUARTER_TURN 450	//testing needed to get precise amount

const int raiseSteps = 0;  // number of steps to raise arm. unkown as of yet
const int extendSteps = 0; // number of steps to extend leadscrew. unknown as of yet
const int clawOpenDegrees = 45; // DEFINITELY WRONG. TEST!
const int clawClosedDegrees = 0; // DEFINITELY WRONG. TEST!

armMotors::armMotors(int motorSteps, int dir1, int dir2, int dir3, int step1, int step2, int step3) {
	A4988 turnStepper(MOTOR_STEPS, DIR1, STEP1);
	A4988 raiseStepper(MOTOR_STEPS, DIR2, STEP2);
	A4988 extendStepper(MOTOR_STEPS, DIR3, STEP3);

	Servo grabberServo;
	Servo clawServo;
}

// driveMotor class constructor
driveMotors::driveMotors() {
	Adafruit_MotorShield AFMS = Adafruit_MotorShield();
	Adafruit_DCMotor *M1 = AFMS.getMotor(1);
	Adafruit_DCMotor *M2 = AFMS.getMotor(2);
	Adafruit_DCMotor *M3 = AFMS.getMotor(3);
	Adafruit_DCMotor *M4 = AFMS.getMotor(4);
}

// Create motor object and link pin numbers to correct motors. Run once in setup
void driveMotors::initialize() {

	if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
		// if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
		Serial.println("Could not find Motor Shield. Check wiring.");
		while (1);
	}
	Serial.println("Motor Shield found.");
}

void armMotors::initialize() {
  // Set target RPMs
  turnStepper.begin(RPM);
  raiseStepper.begin(RPM);
  extendStepper.begin(RPM);

  // Enable motors. TODO: CHECK IF FUNCTION USES CORRECT PIN
  turnStepper.setEnableActiveState(LOW);
  raiseStepper.setEnableActiveState(LOW);
  extendStepper.setEnableActiveState(LOW);

  // Initialize servo
  grabberServo.attach(GRABBER_SERVO_PIN);
	clawServo.attach(CLAW_SERVO_PIN);

  // Set solenoid as output
  pinMode(SOLENOID_PIN, OUTPUT);
}
// TODO: Test motor directions (forward, backward)

// Drive up (y+)
void driveMotors::forward() {
	M1->run(FORWARD);
	M3->run(FORWARD);
}
// Drive down (y-)
void driveMotors::backward() {
	M1->run(BACKWARD);
	M3->run(BACKWARD);
}
// Drive left (x-)
void driveMotors::left() {
	M2->run(BACKWARD);
	M4->run(BACKWARD);
}
// Drive right (x+)
void driveMotors::right() {
	M2->run(FORWARD);
	M4->run(FORWARD);
}
// Set speed of specified motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4. Speed is 0-255.
void driveMotors::setSpeed(int speed) {
	M1->setSpeed(speed);
	M2->setSpeed(speed);
	M3->setSpeed(speed);
	M4->setSpeed(speed);
}
// Stop specific motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4.
void driveMotors::stopMotor(int motorNum) {
	switch(motorNum) {
		case 1:
			M1->run(RELEASE);
			break;
		case 2:
			M2->run(RELEASE);
			break;
		case 3:
			M3->run(RELEASE);
			break;
		case 4:
			M4->run(RELEASE);
			break;
		default:
			break;
	}
}
// Stop ALL motors
void driveMotors::allStop() {
	M1->run(RELEASE);
	M2->run(RELEASE);
	M3->run(RELEASE);
	M4->run(RELEASE);
}

// Note: Clockwise and counterclockwise may be reversed. Testing needed

void armMotors::clockwiseSusan(int steps) {
	turnStepper.move(steps);
}

void armMotors::counterSusan(int steps) {
	turnStepper.move(-steps);
}

void turnSusan(bool dir){	// 0 = c, 1 = cc
	if(dir)
		counterSusan(QUARTER_TURN);
	else
		clockwiseSusan(QUARTER_TURN);
}

// Step amounts for extend/retract are not yet determined
void armMotors::raiseArm(int steps) {
	raiseStepper.move(steps);
}

void armMotors::lowerArm(int steps) {
	raiseStepper.move(-steps);
}

void armMotors::setGrabber(int degrees) {
	grabberServo.write(degrees);
}

// Once again, step number is as-of-yet undetermined.
void armMotors::extendScrew(int steps) {
	extendStepper.move(steps);
}

void armMotors:retractScrew(int steps) {
	extendStepper.move(-steps);
}

void armMotors::openClaw() {
	clawServo.write(clawOpenDegrees);
}

void armMotors::closeClaw() {
	clawServo.write(clawClosedDegrees);
}
