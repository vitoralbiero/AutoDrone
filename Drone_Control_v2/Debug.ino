void DebugInit(){
  if (DEBUG == 1)
  {
    Serial.begin(9600);
    Serial.println("initializing......");
  }
}

void DebugAutonomousMode(){
  if (DEBUG == 1) {
    Serial.print("iX: ");
    Serial.print(InputX);
    Serial.print(" - iY: "); 
    Serial.print(InputY);
    Serial.print(" - iZ:"); 
    Serial.print(InputZ);    
    Serial.print("    |     oX: ");
    Serial.print(OutputX);
    Serial.print(" - oY: "); 
    Serial.print(OutputY);    
    Serial.print(" - oZ: "); 
    Serial.print(OutputZ);   
    Serial.print("  |  t: ");
    Serial.print(thrSpeed);
    Serial.print(" - e: ");
    Serial.print(ele);
    Serial.print(" - r: ");
    Serial.print(rud);      
    Serial.print(" - Mode: "); 
    Serial.println(instMode);
  }    
}

void DebugManualMode(){
  if (DEBUG == 1)
  {
    Serial.print("Thro:  ");
    Serial.print(thro_in);
    Serial.print("  Rudd:  ");
    Serial.print(rudd_in);
    Serial.print("  Eleo:  ");
    Serial.print(eleo_in);
    Serial.print("  Aleo:  ");
    Serial.println(aleo_in);        
  }
}

void DebugSensorMode(){
  if (DEBUG == 1) {
    Serial.print("iL: ");
    Serial.print(InputLeft);
    Serial.print(" - iR: "); 
    Serial.print(InputRight);
    Serial.print(" - iG: "); 
    Serial.print(InputGround);
    Serial.print("    |     oL: ");
    Serial.print(OutputLeft);
    Serial.print(" - oR: "); 
    Serial.print(OutputRight);
    Serial.print(" - oG: "); 
    Serial.print(OutputGround);
    Serial.print("  |  t: ");
    Serial.print(thrSpeed);
    Serial.print(" - e: ");
    Serial.print(ele);
    Serial.print(" - a: ");
    Serial.println(rud);   
  }    
}

void DebugI2C(){
  if (DEBUG == 1) {
    Serial.print("X: ");
    Serial.print(InputX);
    Serial.print(" - Y: "); 
    Serial.print(InputY);
    Serial.print(" - Z: "); 
    Serial.print(InputZ);
    Serial.print(" - Mode: ");
    Serial.println(instMode);   
  }    
}
  
