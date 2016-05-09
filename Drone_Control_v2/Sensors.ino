void SetupSensors(){
  Setpoint = 50;
  Setpoint2 = 20;
  
  InputLeft = sLeft.ping_cm();
  InputRight = sRight.ping_cm();
  InputGround = sGround.ping_cm();

  leftPID.SetOutputLimits(0, 300);
  rightPID.SetOutputLimits(0, 300);
  groundPID.SetOutputLimits(1, 5);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
  groundPID.SetMode(AUTOMATIC);  
}

