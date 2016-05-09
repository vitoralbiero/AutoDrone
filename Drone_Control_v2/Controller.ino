void SetupController(){
  thrSpeed = 1064;
  
  thro.attach(10);
  aleo.attach(11);
  eleo.attach(12);
  rudd.attach(13);  
}

void ReadController()
{
  thro_in = pulseIn(8,HIGH,20000); // read speed
  aleo_in = pulseIn(5,HIGH,20000); // read left/right
  eleo_in = pulseIn(6,HIGH,20000); // read front/back
  rudd_in = pulseIn(7,HIGH,20000); // read axis spin
  aux1_in = pulseIn(9,HIGH,20000); // read fly mode
}

void SendCommands(int _thr, int _ele, int _ale, int _rud){
  thro.writeMicroseconds(_thr); // speed
  eleo.writeMicroseconds(_ele); // front/back
  aleo.writeMicroseconds(_ale); // left/right
  rudd.writeMicroseconds(_rud); // fly mode  
}

void Arm(){
  if (thrSpeed <= 1064){
    for (int i = 1; i < 30000; i++) {
      SendCommands(1064, 1500, 1500, 2000);
    }
  }
}

void Disarm(){
  if (thrSpeed <= 1064){
    for (int i = 1; i < 30000; i++) {
      SendCommands(1064, 1500, 1500, 1000);
    }
  }
}

