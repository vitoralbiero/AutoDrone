void SensorMode()
{
  //CalculateDistances(1500, true);

  //SendCommands(thrSpeed, eleo_in, ale, rudd_in);

  //DebugSensorMode();
}

void CheckCollision(int _ale){
  CalculateDistances(_ale, false);  
}

void CalculateDistances(int _ale, bool SensorMode){
  //read sensors
  InputLeft = sLeft.ping_cm(); 
  InputRight = sRight.ping_cm();
  InputGround = sGround.ping_cm();

  leftPID.Compute(); // pid calculations
  rightPID.Compute();
  groundPID.Compute();

  if ((InputLeft < InputRight) && (InputLeft < 50)) // check which side has the closest object
    ale = _ale + OutputLeft;                           // if less then 50cm go away
  else if (InputRight < 50)                    
    ale = _ale - OutputRight;

  if (SensorMode){    
    if (InputGround < 20)
      thrSpeed += OutputGround;
    else if (InputGround > 30)
      thrSpeed -= OutputGround;
      
    if (thrSpeed > 1900)
      thrSpeed = 1900;   
  }
}

