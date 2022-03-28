/*
 * This code has been butchered from the pixy2 hello_world program.
*/
  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      delay(300);
      Serial.print(" x: ");
      Serial.print(pixy.ccc.blocks[i].m_x);
      Serial.print(" width: ");
      Serial.print(pixy.ccc.blocks[i].m_width);
      Serial.print("\n");
    }
  }

  /*
    This code was moved from mainrobotcontrol to make it easier to read
    but still keep all of this demo code
  */

  // print out distances to serial
  if (measure1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure1.RangeMilliMeter);
  } else {
    Serial.println(" sensor 1 out of range ");
  }
  if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 2 (mm): "); Serial.println(measure2.RangeMilliMeter);
  } else {
    Serial.println(" sensor 2 out of range ");
  }
  
  if (measure3.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure3.RangeMilliMeter);
  } else {
    Serial.println(" sensor 3 out of range ");
  }
  
  if (measure4.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance 1 (mm): "); Serial.println(measure4.RangeMilliMeter);
  } else {
    Serial.println(" sensor 4 out of range ");
  }

  // If the range on Sensor1 is less than 450mm, slow down. If it's less than 250mm, stop
  if(measure1.RangeMilliMeter >= 450) { //change to RangeInches
      allClear = true;
      setSpeed(1,100);
      setSpeed(2,100);
      setSpeed(3,100);
      setSpeed(4,100);
    }
    else if(measure1.RangeMilliMeter >= 250) {
      allClear = true;
      setSpeed(1,60);
      setSpeed(2,60);
      setSpeed(3,60);
      setSpeed(4,60);
    }
    else if(measure1.RangeMilliMeter < 250) {
      allClear = false;
    }

  /************************************************
   * PIXY CAMERA
   ***********************************************/

   pixy.ccc.getBlocks();
   
   // If there are detected blocks, stop driving
   if (pixy.ccc.numBlocks)
   {
    Serial.println("********CUP DETECTED*****");
    allClear = false; // stop driving
    /*
     Serial.print("Detected ");
     Serial.println(pixy.ccc.numBlocks);
     for (int i=0; i<pixy.ccc.numBlocks; i++)
     {
       Serial.print("  block ");
       Serial.print(i);
       Serial.print(": ");
       pixy.ccc.blocks[i].print();
     }
     */
   }

  /******************************************
   * MOTORS
   *****************************************/
   // only drive if sensors reporting all clear
   if(allClear != allClearOld) {
    if(allClear) {
      driveUp();
    } else {
      allStop();
    }
    allClearOld = allClear;
   }
  /*  // Testing all motor functions


  // Main motors
  setSpeed(1,100);
  setSpeed(2,100);
  setSpeed(3,100);
  setSpeed(4,100);
  
  driveUp();
  delay(6000);
  driveDown();
  delay(6000);
  allStop();
  delay(3000);
  driveRight();
  delay(1500);
  driveLeft();
  delay(1500);
  allStop();
  
  
  driveRight();
  delay(1500);
  driveLeft();
  delay(1500);
  allStop();
  
  setSpeed(1,120);
  setSpeed(2,120);
  //setSpeed(3,120);
  setSpeed(4,120);
  driveUp();
  delay(1000);
  stopMotor(1);
  //delay(1000);
  //stopMotor(3);
  delay(1000);
  driveRight();
  delay(1000);
  stopMotor(2);
  
  delay(1000);
  stopMotor(4);
  delay(1000);
  driveDown();
  delay(2000);
  allStop();
  
  // grabber arm
  delay(5000);
  clockwiseSusan(100);
  delay(2000);
  counterSusan(100);
  delay(2000);
  extendArm(50);
  delay(2000);
  retractArm(50);
  delay(2000);
  raiseGrabber();
  delay(2000);
  lowerGrabber();
  delay(2000);
  extendScrew(100);
  delay(2000);
  retractScrew(100);
  delay(2000);
  extendSolenoid();
  delay(2000);
  retractSolenoid();
  delay(6000);
  */
  delay(50);


}
