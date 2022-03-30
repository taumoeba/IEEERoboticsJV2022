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
      drive.setSpeed(100);
    }
    else if(measure1.RangeMilliMeter >= 250) {
      allClear = true;
      drive.setSpeed(60);
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

  /***************************************
   * MOTORS
   *****************************************/
   // only drive if sensors reporting all clear
   if(allClear != allClearOld) {
    if(allClear) {
      drive.forward();
    } else {
      drive.allStop();
    }
    allClearOld = allClear;
   }
  delay(50);


}
