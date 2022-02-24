#include <grabber_arm.h>

/*******************************************************************************
* Library for controlling grabber arm.
* Stepper 1 is turntable. Stepper 2 is arm raising. Stepper 3 is leadscrew.
* Solenoid is believed to be extended when unpowered.
********************************************************************************/

// INCOMPLETE, DO NOT USE!!

// 200 steps per revolution (probably)
#define MOTOR_STEPS 200
#define RPM 120
#define MICROSTEPS 16

#define DIR1 26
#define STEP1 27
#define DIR2 32
#define STEP2 33
#define DIR3 38
#define STEP3 39

#define servoPin 9
#define solenoidPin 12

const int raiseSteps = 0;  // number of steps to raise arm. unkown as of yet
const int extendSteps = 0; // number of steps to extend leadscrew. unknown as of yet

#define turnMS1 29
#define turnMS2 30
#define turnMS3 31
A4988 turnStepper(MOTOR_STEPS, DIR1, STEP1, turnMS1, turnMS2, turnMS3);

#define raiseMS1 35
#define raiseMS2 36
#define raiseMS3 37
A4988 raiseStepper(MOTOR_STEPS, DIR2, STEP2, raiseMS1, raiseMS2, raiseMS3);

#define extendMS1 41
#define extendMS2 42
#define extendMS3 43
A4988 extendStepper(MOTOR_STEPS, DIR3, STEP3, extendMS1, extendMS2, extendMS3);

Servo grabberServo;

void initializeGrabber() {
	// Set target RPMs
	turnStepper.begin(RPM);
	raiseStepper.begin(RPM);
	extendStepper.begin(RPM);
	
	// Enable motors. TODO: CHECK IF FUNCTION USES CORRECT PIN
	turnStepper.setEnableActiveState(LOW);
	raiseStepper.setEnableActiveState(LOW);
	extendStepper.setEnableActiveState(LOW);
	
	// Initialize servo
	grabberServo.attach(servoPin);
	
	// Set solenoid as output
	pinMode(solenoidPin, OUTPUT);
}

// Note: Clockwise and counterclockwise may be reversed. Testing needed
void clockwiseSusan(int steps) {
	turnStepper.move(steps);
}

void counterSusan(int steps) {
	turnStepper.move(-steps);
}

// Step amounts for extend/retract are not yet determined
void extendArm(int steps) {
	raiseStepper.move(steps);
}

void retractArm(int steps) {
	raiseStepper.move(-steps);
}

// Servo rotation distances are as of yet undetermined
void raiseGrabber() {
	for(int pos=0; pos<=180; pos++) {
		grabberServo.write(pos); // Raising the grabber will probably take 2.7 sec.
		delay(15);
	}
}

void lowerGrabber() {
	for(int pos=180; pos>=0; pos--) {
		grabberServo.write(pos); // Raising the grabber will probably take 2.7 sec.
		delay(15);
	}
}

// Once again, step number is as-of-yet undetermined.
void extendScrew(int steps) {
	extendStepper.move(steps);
}

void retractScrew(int steps) {
	extendStepper.move(-steps);
}

// If solenoid is normally retracted, reverse HIGH and LOW here
void extendSolenoid() {
	digitalWrite(solenoidPin, LOW);
}

void retractSolenoid() {
	digitalWrite(solenoidPin, HIGH);
}