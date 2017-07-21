#include "Drive.h"


// Right motor
#define RightIN1 20   // AIN1
#define RightIN2 21   // AIN2
#define RightPWM 5

// Left motor
#define LeftIN1 18    // BIN1
#define LeftIN2 17    // BIN2
#define LeftPWM 6

//Standby Pin
#define STBY 19       // Standby Pin, Low = Standby. Documentation https://www.pololu.com/file/0J86/TB6612FNG.pdf

// Encoders           //Encoder declarations, eventually move it into a struct and hopefully a class on it's own.
// - Right wheel
#define RightA 0
#define RightB 1
// - Left wheel
#define LeftA 2
#define LeftB 3

// Sensor pin assignments  - Changed rear left & front left pin assignments
#define rearLeftSensor   23 
#define rearRightSensor  22 

//Front Left sensor
#define frontRightSensor 15 
//Front right sensor
#define frontLeftSensor  16

//Initialization for the encoders; choose large negative # to prevent overflow
long positionLeft  = -999;
long positionRight = -999;

static float encoderKP = 1;
static float encoderKI = 0.75;
static float encoderKD = 3;

static float KpIR = 2, KiIR = 0.5, KdIR = 4;   //PID constants using IR to drive straight

static float KpA = .25, KiA = 0.05, KdA = 1.2;   //PID constants using IR to drive straight

//Encoders
int encR, encL;

// Sensor Objects
Sensor rearLeft(rearLeftSensor);
Sensor frontLeft(frontLeftSensor);
Sensor rearRight(rearRightSensor);
Sensor frontRight(frontRightSensor);


//Encoder Objects
Encoder encRight(RightA, RightB);
Encoder encLeft(LeftA, LeftB);

// Motor Objects
Motor left(LeftIN1, LeftIN2, LeftPWM);
Motor right(RightIN1, RightIN2, RightPWM);

// Signal for ultrasonic sensor
#define Echo 14
#define Trig 4



PID encoderPID(encoderKP, encoderKI, encoderKD);
PID sensorPID(KpIR,KiIR,KdIR);
PID anglePID(KpA, KiA, KdA);
PID leftPID(0.125,0,0);
PID rightPID(0.125,0,0);
PID leftAnglePID(0.125,0,0);
PID rightAnglePID(0.125,0,0);

Ultrasonic front(Echo, Trig);

int Spd = 250;
Drive Robot(Spd);

double distanceTraveled() // Returns Distance in cm
{
  //Constants
  double ticksPerRotationRight = -615; //Ticks to rotate forward
  double ticksPerRotationLeft = 615; //Ticks to rotate forward
  double wheelDiameter = 40.15; //40.15mm
  //Calculations
  double circumference = 3.14152965*wheelDiameter;
  double distanceR = abs(encR*circumference/ticksPerRotationRight);
  double distanceL = abs(encL*circumference/ticksPerRotationLeft);
  double distance = (distanceR + distanceL)/2;
  return distance/10;
}
void resetEncoders()
{
  encRight.write(0);
  encLeft.write(0);
}

int counter = 0;
void travelBlocks(int blocks) // Should increment counter through here, make an object
{
  resetEncoders();
  int encError = encL-encR;
  int moved = 0;
  while(moved < blocks)
  {
    encL = encLeft.read();
    encR = encRight.read() *-1;
    encError = encL-encR;
    if(distanceTraveled() < 18.0)
    {
        Robot.moveForward(counter, encError);  
    }
    if(distanceTraveled() >= 18.0)
    {
      encRight.write(0);
      encLeft.write(0);
      moved++;
      Robot.Stop();
      Wait(1);
    }
    debug();
  }
  Robot.Stop();


  
}
void Wait(int sec)
{
  int prevmillis = millis();
  while(millis() - prevmillis < sec*1000)
  {
    continue;
  }
}

void SetUpPinModes()
{
  left.SetPinMode();
  right.SetPinMode();
  rearLeft.SetPinMode();
  frontLeft.SetPinMode();
  rearRight.SetPinMode();
  frontRight.SetPinMode();
  front.SetPinMode();  
  pinMode(STBY, OUTPUT); 
}
void SetUp()
{
  Robot.setMotors(&left, &right);
  Robot.setSensors(&rearLeft, &frontLeft, &rearRight, &frontRight);
  Robot.setUltrasonic(&front);
  Robot.setPIDencoder(&encoderPID);
  Robot.setPIDsensor(&sensorPID);
  Robot.setPIDangle(&anglePID);
  Robot.setPIDleft(&leftPID, &leftAnglePID);
  Robot.setPIDright(&rightPID, &rightAnglePID);
  //Robot.setEncoders(&encLeft, &encRight);
  digitalWrite(STBY, HIGH); // Third enable for Motors, STANDBY = LOW -> motor don't run
}

void debug(){
//  Serial.print("Current time is:   ");
//  Serial.print(millis());
  Serial.print("\nRight Ticks: ");
  Serial.print(encRight.read());
  Serial.print("\tLeft Ticks: ");
  Serial.print(encLeft.read());
//  Serial.print("THis is the error:  ");
//  Serial.print((int)(Spd+total));
//  Serial.print("Sonic Value:  ");
//  Serial.print(sonicDist);
//  Serial.print(" Rear left: ");
//  Serial.print(rLeftFiltered);
//  Serial.print(" Rear right: ");
//  Serial.print(rRightFiltered);
//  Serial.print(" front left: ");
//  Serial.print(fLeftFiltered);
//  Serial.print(" front right: ");
//  Serial.print(fRightFiltered);
//  Serial.print("IRERROR:   ");
//  Serial.print(total_A);
 Serial.print("\tDist Trav:  ");
 Serial.println(distanceTraveled());
}



void setup()
{
  Serial.begin(9600);
  SetUpPinModes();
  SetUp();
  Serial.println("This thing does work");
  delay(8000);
}

int blocks = 3;

int encError;


void loop()
{ 
encR = encRight.read() * -1;
encL = encLeft.read();

travelBlocks(blocks);
//Wait(2);
////Robot.turnLeft(LeftA, LeftB, RightA, RightB);
////Wait(2);
//travelBlocks(blocks);  
//Wait(2);
////Robot.turnRight(LeftA, LeftB, RightA, RightB);
//Wait(2);
resetEncoders();



//blocks = 1;
//  debug();
//  counter++;
}
