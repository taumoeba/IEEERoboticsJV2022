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
}
