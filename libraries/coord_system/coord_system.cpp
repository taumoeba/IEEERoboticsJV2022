#include "coord_system.h"

// this is the code for how the robot moves around and logs the position of the cups
void setup(){
    trees[0].x = 12;
    trees[0].y = 7;
    trees[1].x = 75;
    trees[1].y = 36;

    //initial position of the robot
    currentpos.x = 24;
    currentpos.y = 6;
    //these need to be backed up with distance sensors.
}

//when a cup is found, log the position
void logcup(){
    //this would be after centering on the cup
    switch(pixy){
        case up:
            cups[cupindex] = currentpos;
            cups[cupindex].y += cupdistance;
            ++cupindex;
            break;
        case down:
            cups[cupindex] = currentpos;
            cups[cupindex].y -= cupdistance;
            ++cupindex;
            break;
        case left:
            cups[cupindex] = currentpos;
            cups[cupindex].x -= cupdistance;
            ++cupindex;
            break;
        case right:
            cups[cupindex] = currentpos;
            cups[cupindex].x += cupdistance;
            ++cupindex;
            break;
    }
}