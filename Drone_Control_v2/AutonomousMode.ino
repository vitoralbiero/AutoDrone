void AutonomousMode()
{
  CameraCalc(); // calculates the direction (X, Y, Z) of the tracked object
  
  SpeedCalc(); // calculates the speed

  thr = thrSpeed;
  ele = 1500 + OutputZ; // 1500 is the middle, less go back, more go front
  rud = 1500 + OutputX; // 1500 is the middle, less turn left, more turn right
  
  CheckCollision(1500); // check if has object in the path
             
  SendCommands(thr, ele, aleo_in, rud);
   
  DebugAutonomousMode();

  OutputX = 0;
  OutputY = 0;
  OutputZ = 0;  
}

void CameraCalc(){
  if (InputX > 0)           // 160 is the mean then when reading 160 returns 0,
    OutputX = InputX - 160; //  then when reading 0 returns -160 and when reading 320 returns 160,
  if (InputY > 0)           // 120 is the mean then when reading 120 returns 0,
    OutputY = InputY - 120; // then when reading 0 returns -120 and when reading 240 returns 120,
  if (InputZ > 0)           // value calculate by the size of the square captured by the camera
    OutputZ = InputZ;

  if (OutputX > 0)
    OutputX = map(OutputX, -160, 160, -300, 300);  // remape outpux to -300 and 300
  if (OutputY > 0)
    OutputY = map(OutputY, -120, 120, -100, 100);  // remape outpux to -10 and 10

  if ((OutputZ <= 100) && (OutputZ > 10)) // if the object is less then 100cm and more then 10cm the drone goes back
    OutputZ = -200;
  else if ((OutputZ > 100) && (OutputZ < 500)) // if the object is more then 100cm the drone goes front
    OutputZ = 200;
}

void SpeedCalc(){
  InputGround = sGround.ping_cm();
  groundPID.Compute();
    
  if (instMode == 1) { //take off    
    Arm();    
    if (InputGround < 90)
      thrSpeed += OutputGround;
    else if (InputGround > 100)
      thrSpeed -= OutputGround;
  }
  else if (instMode == 2) { //land
    Disarm();
    if (InputGround > 5)
      thrSpeed -= OutputGround;
    else
      thrSpeed = 1064;  
  }
  else if (instMode == 3){ //follow object
    thrSpeed += OutputY;
  }
  if (thrSpeed > 1600)
    thrSpeed = 1600;
}

