#include <Wire.h> 
#define SLAVE_ADDRESS 0x04

void SetupI2C(){
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(readRaspberry);
  Wire.onRequest(sendRaspberry);
  instMode = 0;
}

void readRaspberry(int byteCount)
{ 
  int Temp[5];
  
  while(Wire.available())
  {
    for (int i = 0; i < 5; i++)
    {
      Temp[i] = Wire.read();      
    }
  }
  InputX = Temp[0];
  InputX = (Temp[0] <<8) | Temp[1];
  InputY = Temp[2];
  InputZ = Temp[3];
  
  if (((Temp[4] == 0) && (instMode != 2) && (instMode != 1)) || (Temp[4] != 0))
    instMode = Temp[4];
  //DebugI2C;
}

void sendRaspberry()
{ 
 flyMode = 0;
 if ((aux1_in > 1500) && (rudd_in <= 1000) && (rudd_in > 0) && (aleo_in <= 1000)) 
   flyMode = 8; // exit raspberry program
 else if ((aux1_in > 1500) && (rudd_in <= 1000) && (rudd_in > 0) && (eleo_in <= 1000))
   flyMode = 9; // shutdown raspberry
 else if (aux1_in > 1500)
   flyMode = 1;  // autonomous mode on
 else
   flyMode = 0; // autonomous mode off
   
 Wire.write(flyMode);
}
