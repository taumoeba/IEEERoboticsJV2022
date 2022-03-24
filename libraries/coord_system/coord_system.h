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

*/

#define cupdistance 4   //this is currently in inches, can change later

struct position{
    double x;
    double y;
};


position cups[4];   //in case there are more than two cups
char cupindex;
position trees[2];
char treeindex;

position currentpos;


enum pixypos{up, down, left, right};    //using the same names as the moter library names
pixypos pixy = left;    //initialize to left, where it should be pointed at.
                        //change this inside of susan functions

