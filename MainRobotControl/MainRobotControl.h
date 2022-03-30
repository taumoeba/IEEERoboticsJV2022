#include "Adafruit_VL53L0X.h
#include <Pixy2.h>
#include "simple_motor.h"
#include "coord_system.h"
#include <Adafruit_MotorShield.h>

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

bool foundCup();    //to be written with pixy camera

driveMotors drive = driveMotors();
armMotors arm = armMotors();
