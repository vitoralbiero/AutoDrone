#include <PID_v1.h>
#include <NewPing.h>
#include <Servo.h>
#define Kp 2
#define Ki 5
#define Kd 1
#define DEBUG 1

Servo aleo, eleo, thro, rudd;

double Setpoint, Setpoint2;
double InputLeft, InputRight, InputBack, InputGround, InputX, InputY, InputZ;
double OutputLeft, OutputRight, OutputBack, OutputGround, OutputX, OutputY, OutputZ;

NewPing sLeft(A0,A1,200), sRight(A2,A3,200), sGround(2,3,200); //sFront(A4,A5,200), sBack(A0,A1,200)

PID leftPID(&InputLeft, &OutputLeft, &Setpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&InputRight, &OutputRight, &Setpoint, Kp, Ki, Kd, DIRECT);
PID groundPID(&InputGround, &OutputGround, &Setpoint2, Kp, Ki, Kd, DIRECT);

int thr, ele, ale, rud;
int thro_in, eleo_in, aleo_in, rudd_in, aux1_in;
int flyMode, instMode, thrSpeed;

void setup()
{
  SetupI2C();
  SetupSensors();
  SetupController();

  DebugInit();
}

void loop()
{
  ReadController();
  
  if((aux1_in <= 1000) && (aux1_in > 0)) // manual mode
    ManualMode();
  
  if((aux1_in > 1000) && (aux1_in < 1500)) // sensor mode
    SensorMode();
    
  if (aux1_in >= 1500) // computer vision mode
    AutonomousMode();     
}
