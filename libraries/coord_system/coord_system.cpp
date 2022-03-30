#include "coord_system.h"
#include "MainRobotControl.h"

// this is the code for how the robot moves around and logs the position of the cups
void coordSetup(){
    trees[0].x = 12;
    trees[0].y = 7;
    trees[0].looking = left;
    trees[1].x = 75;
    trees[1].y = 36;
    trees[1].looking = up;

    //initial position of the robot
    currentpos.x = 24;
    currentpos.y = 6;
    //these need to be backed up with distance sensors.
}

//when a cup is found, log the position
void logcup(){
    //this would be after centering on the cup
    switch(pixyDir){
        case up:
            cups[cupindex] = currentpos;
            cups[cupindex].looking = up;
            ++cupindex;
            break;
        case down:
            cups[cupindex] = currentpos;
            cups[cupindex].looking = down;
            ++cupindex;
            break;
        case left:
            cups[cupindex] = currentpos;
            cups[cupindex].looking = left;
            ++cupindex;
            break;
        case right:
            cups[cupindex] = currentpos;
            cups[cupindex].looking = right;
            ++cupindex;
            break;
    }
}

void currentPosLog(){
    //all of these can change to satisfy overall design

    //up and left are garunteed to be seen
    double up = measure1.RangeMilliMeter/25.4 + fromcenter;
    double left = measure3.RangeMilliMeter/25.4 + fromcenter;

    //there might not be a visible wall for these sensors,
    //use these later on for double checking
    //double down = measure2.RangeMilliMeter/25.4 + fromcenter;
    //double right = measure4.RangeMilliMeter/25.4 + fromcenter;

    //these gives the distances of the robot from the center


    currentpos.x = left;
    currentpos.y = 48 - up;
    //looking must be logged outside of code since the arm can look in any direction
}
