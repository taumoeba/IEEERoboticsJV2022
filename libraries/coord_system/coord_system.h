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

//#define cupdistance 12  	//this is currently in inches, can change later
//#define fromcenter 5.75		//distance sensors are from center of robot


#define tocords(steps) (double)(steps/2) //experimentation will make this more precise
#define toinches(millimeters) (double)(millimeters*0.0393701) //current assumption is that distance sensors read mm and i'm using inches

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


direction pixy = left;    //initialize to left, where it should be pointed at.
                        //change this inside of susan functions

void logcup();
void currentPosLog();