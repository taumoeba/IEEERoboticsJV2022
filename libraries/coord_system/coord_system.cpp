#include "coord_system.h"
#include "MainRobotControl.h"

// this is the code for how the robot moves around and logs the position of the cups
void coordSetup(){
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
    precups[0].looking left;
    precups[1].x = 24;
    precups[1].y = 27;
    precups[1].looking left;
    precups[2].x = 24;
    precups[2].y = 24;
    precups[2].looking up;
    precups[3].x = 48;
    precups[3].y = 24;
    precups[3].looking up;
    precups[4].x = 84;
    precups[4].y = 24;
    precups[4].looking up;
    precups[5].x = 48;
    precups[5].y = 24;
    precups[5].looking down;
    precups[6].x = 84;
    precups[6].y = 24;
    precups[6].looking down;
    

    //initial position of the robot
    currentpos.x = 24;
    currentpos.y = 6;
    //these need to be backed up with distance sensors.
}

//when a cup is found, log the position
void logcup(){
    //this would be after centering on the cup
    position newcup;
    newcup = currentpos;
    newcup.looking = pixyDir;
    switch(newcup.looking){
        case left:
            if(abs(newcup.y - precups[0].y) < abs(newcup.y - precups[1])){
                cups[cupindex++] = precups[0];
            }
            else{
                cups[cupindex++] = precups[1];
            }
            break;
        case up:
            if(abs(newcup.x - precups[2]) < abs(newcup.x - precups[3] && abs(newcup.x - precups[2]) < abs(newcup.x - precups[4]))){
                cups[cupindex++] = precups[2];
            }
            else if(abs(newcup.x - precups[3]) < abs(newcup.x - precups[2] && abs(newcup.x - precups[3]) < abs(newcup.x - precups[4]))){
                cups[cupindex++] = precups[3];
            }
            else{
                cups[cupindex++] = precusp[4];
            }
            break;
        case down:
            if(abs(newcup.x - precups[5]) < abs(newcup.x - precups[6])){
                cups[cupindex++] = precups[5];
            }
            else{
                cups[cupindex++] = precups[6];
            }
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
