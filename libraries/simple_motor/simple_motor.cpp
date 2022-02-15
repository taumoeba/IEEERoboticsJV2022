/**************************************************************************
 * Lipscomb IEEE Robotics 2022
 * Motor Control Library
 * Written by Ben Powell
 * Using Adafruit Motor Shield to control 4 DC motors.
 * M2 and M4 are aligned with the 'X' axis, and M1 and M3 with the 'Y' axis.
 *************************************************************************/

//  THIS IS UNFINISHED, DO NOT USE!!!

 /* Shape of field. "Up" is y+. "Down" is y-. "Left" is x-. "Right" is x+.
 Robot starts at A.
 
    ------------------------------------------------------
	|         ^                                          |
	|         y+                                         |
	|    < x-   x+ >                                     |
	|         y-	                                     |
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

// Create motor object and link pin numbers to correct motors. Run once in setup
void initializeMotors() {
	Adafruit_Motorshield AFMS = Adafruit_Motorshield();
	Adafruit_DCMotor *M1 = AFMS.getMotor(1)
	Adafruit_DCMotor *M2 = AFMS.getMotor(2)
	Adafruit_DCMotor *M3 = AFMS.getMotor(3)
	Adafruit_DCMotor *M4 = AFMS.getMotor(4)
	
	if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
		// if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
		Serial.println("Could not find Motor Shield. Check wiring.");
		while (1);
	}
	Serial.println("Motor Shield found.");
}
// TODO: Test motor directions (forward, backward)

// Drive up (y+) 
void driveUp() {
	M1->run(FORWARD);
	M3->run(FORWARD);
}
// Drive down (y-) 
void driveDown() {
	M1->run(BACKWARD);
	M3->run(BACKWARD);
}
// Drive left (x-)
void driveLeft() {
	M2->run(BACKWARD);
	M4->run(BACKWARD);
}
// Drive right (x+)
void driveRight() {
	M2->run(FORWARD);
	M4->run(FORWARD);
}
// Set speed of specified motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4. Speed is 0-255.
void setSpeed(int motorNum, int speed) {
	switch(motorNum) {
		case 1:
			M1->setSpeed(speed);
			break;
		case 2:
			M2->setSpeed(speed);
			break;
		case 3:
			M3->setSpeed(speed);
			break;
		case 4:
			M4->setSpeed(speed);
			break;
		default:
			break;
	}
}
// Stop specific motor. 1 is M1, 2 is M2, 3 is M3, 4 is M4. 
void stopMotor(int motorNum) {
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
void allStop() {
	M1->run(RELEASE);
	M2->run(RELEASE);
	M3->run(RELEASE);
	M4->run(RELEASE);
}