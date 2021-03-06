//this is the h file for the coord system


/* Shape of field. "Up" is y+. "Down" is y-. "Left" is x-. "Right" is x+.
 Robot starts at A.

                                                        96,48
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
	0,0
*/
/*
defining the distances makes modifing measuring systems easier.

board dimensions: 48" by 96"

first tree 12,7
second tree 75,36

initial center of the robot 24,6

*/

#ifndef COORD_SYSTEM_H
#define COORD_SYSTEM_H

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

enum direction{up, down, left, right};    //using the same names as the moter library names

struct position{
    double x;
    double y;
	direction looking;	//for robot, direction is where arm is pointing
						//for trees and cups, it is where the arm should be to interact with it
};

position cups[4];   //in case there are more than two cups
char cupindex;
position trees[2];
char treeindex;

position currentpos;
position precups[7];

void logcup();
void currentPosLog();
void coordSetup();

#endif
