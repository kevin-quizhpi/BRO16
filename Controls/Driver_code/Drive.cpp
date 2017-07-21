#include "Drive.h"

Drive::Drive(int Speed)
{
	SetSpeed(Speed);

}


// Left Clockwise & Right CounterClockwise = Backwards
// Right Clockwise & Left CounterClockwise = Forwards
void Drive::setForward()
{
	right.setClockwise();
	left.setCounterClockwise();
}
void Drive::Stop()
{
	right.stopMotor();
	left.stopMotor();
}
void Drive::setRotateRight()
{
	right.setCounterClockwise();
	left.setCounterClockwise();
}
void Drive::setRotateLeft()
{
	right.setClockwise();
	left.setClockwise();
}
bool Drive::checkFrontSensorsWalls(int counter)	// True if Wall is there
{
	if((frontLeft.readSensor(counter) && frontRight.readSensor(counter)) < SensorThreshold )
		return false;
	else 
		return true;
}
bool Drive::checkRearSensorsWalls(int counter)	// True if Wall is there
{
	if((rearLeft.readSensor(counter) && rearRight.readSensor(counter)) < SensorThreshold)
		return false;
	else
		return true;
}

bool Drive::checkBothWalls(int counter)
{
	if((checkRearSensorsWalls(counter) && checkFrontSensorsWalls(counter)) == true)
		return true;
	else 
		return false;
}
bool Drive::checkFrontWall(int counter)
{
	if(front.Read(counter) < 4.0)
		return true;
	else 
		return false;
}
bool Drive::checkLeftWall(int counter)
{
	if((rearLeft.readSensor(counter) && frontLeft.readSensor(counter)) < SensorThreshold)
		return true;
	else 
		return false;
}
bool Drive::checkRightWall(int counter)
{
	if(rearRight.readSensor(counter) && frontRight.readSensor(counter) < SensorThreshold)
		return true;
	else 
		return false;
}
void Drive::moveForward(int counter, int encError)
{	
	setForward();
	//setSpeedBoth(Spd);

	if(checkBothWalls(counter) == true)		// If both walls use Centering & Angular PID
	{
		float changeCenter = sensorPID.getAdjustment(getCenteringError(counter));
		float changeAngle = anglePID.getAdjustment(getAngleError(counter));
		left.setSpeed(Spd - changeCenter - changeAngle);

	}
	if(checkLeftWall(counter) == true && checkRightWall(counter) == false)
	{
		float changeCenter = leftPID.getAdjustment(getLeftCenterError(counter));
		float changeAngle = leftAnglePID.getAdjustment(getLeftAngleError(counter));
		left.setSpeed(Spd - changeCenter - changeAngle);
	}
	if(checkLeftWall(counter) == false && checkRightWall(counter) == true)
	{
		float changeCenter = rightPID.getAdjustment(getRightCenterError(counter));
		float changeAngle = rightAnglePID.getAdjustment(getRightAngleError(counter));
		left.setSpeed(Spd - changeCenter - changeAngle);
	}
 	if(checkBothWalls(counter) == false)  // If no walls use Encoder PID
	{
	  	float change = encoderPID.getAdjustment(encError);
		Serial.print("Change: ");
	    Serial.print(change);
	   
	    left.setSpeed(Spd - change);
	}




}
void Drive::setSpeedBoth(int Speed = 0)
{
	right.setSpeed(Speed);
	left.setSpeed(Speed);
}
void Drive::SetSpeed(int Speed)
{
	Spd = Speed;

}

//All values for TurnTicks are arbitrary and need to be tested 
void Drive::turnRight(int LA, int LB, int RA, int RB)
{
	Stop();
    Encoder Left(LA, LB);     //Pointers for encoders
    Encoder Right(RA,RB);
    Left.write(0);            //Write encoder values to 0
    Right.write(0);
 
   
    long leftTicks = Left.read();  //Read current encoder values
    long rightTicks = Right.read() * -1;

	Serial.print("\nBefore while loop-");
	Serial.print("\tLeft Ticks: ");    
	Serial.print(leftTicks);
	Serial.print("\t Ticks: ");
	Serial.print(rightTicks);



    while(leftTicks < rightTurnLeftTicks && rightTurnRightTicks > rightTicks) //Need to check ticks
    {
      	setRotateRight();
      	leftTicks = Left.read();
      	rightTicks = Right.read() * -1;
      	Serial.print("\n\nDuring while loop-");
		Serial.print("\tLeft Ticks: ");    
		Serial.print(leftTicks);
		Serial.print("\t Ticks: ");
		Serial.print(rightTicks);
    }

    Stop();
    Left.write(0);
    Right.write(0);  

}
// NEEDS WORK
void Drive::turnLeft(int LA, int LB, int RA, int RB)
{
    Stop();
    Encoder Left(LA, LB);     
    Encoder Right(RA,RB);
    Left.write(0);           //Write encoder values to 0
    Right.write(0);

    long leftTicks = Left.read();      //Read current encoder values
    long rightTicks = Right.read() * -1;


    Serial.print("\nBefore while loop-");
	Serial.print("\tLeft Ticks: ");    
	Serial.print(leftTicks);
	Serial.print("\t Ticks: ");
	Serial.print(rightTicks);
    while(leftTicks > leftTurnLeftTicks && leftTurnRightTicks < rightTicks)   //Need to check ticks
    {
      setRotateLeft();
      leftTicks = Left.read();
      rightTicks = Right.read() * -1;
    }
 
//      //For debugging
//      Serial.println(Left.read());
//      Serial.println(Right.read());
    Stop();
    Left.write(0);
    Right.write(0);  

}
void Drive::turnAround(int LA, int LB, int RA, int RB)
{
    Stop();
    Encoder Left(LA, LB);     //Pointers for encoders
    Encoder Right(RA,RB);
    Left.write(0);           //Write encoder values to 0
    Right.write(0);
 
   
    long leftTicks = Left.read();      //Read current encoder values
    long rightTicks = Right.read();
  
    while(leftTicks < (rightTurnLeftTicks * 2) && (rightTurnRightTicks * 2) < rightTicks)   //Need to check ticks
    {
      setRotateRight();
      leftTicks = Left.read();
      rightTicks = Right.read();
    }
 
//      //For debugging
//      Serial.println(Left.read());
//      Serial.println(Right.read());
    Stop();
    Left.write(0);
    Right.write(0);  

}

// Error getting methods
int Drive::getCenteringError(int counter)
{
	int fError = frontRight.readSensor(counter) - frontLeft.readSensor(counter);
	int rError = rearRight.readSensor(counter) - rearLeft.readSensor(counter);
	return fError + rError;
}
int Drive::getAngleError(int counter)
{
	int error = getLeftAngleError(counter) + getRightAngleError(counter);
	return error;
}

int Drive::getLeftCenterError(int counter)
{
	int error = frontLeft.readSensor(counter) + rearLeft.readSensor(counter);
	error = error /2;
	return LeftDistanceFromWall - error;
}
int Drive::getRightCenterError(int counter)
{
	int error = frontRight.readSensor(counter) + rearRight.readSensor(counter);
	error = error /2;
	return RightDistanceFromWall - error;
}
int Drive::getLeftAngleError(int counter)
{
	int error = frontLeft.readSensor(counter) - rearLeft.readSensor(counter);
	return -1 * error;
}
int Drive::getRightAngleError(int counter)
{
	int error = frontRight.readSensor(counter) - rearRight.readSensor(counter);
	return -1 * error;
}



// Passing pointers to Drive Class
void Drive::setMotors(Motor *l, Motor *r)
{
	left = *l;
	right = *r;
	setSpeedBoth(Spd);
}

void Drive::setSensors(Sensor *rl, Sensor *fl, Sensor *rr, Sensor *fr)
{	
	rearLeft = *rl;
	frontLeft = *fl;
	rearRight = *rr;
	frontRight = *fr;
}
void Drive::setUltrasonic(Ultrasonic *f)
{
	front = *f;
}
void Drive::setPIDencoder(PID *enc)
{
	encoderPID = *enc;
}
void Drive::setPIDsensor(PID *sens)
{
	sensorPID = *sens;
}
void Drive::setPIDangle(PID *ang)
{
	anglePID = *ang;
}
void Drive::setPIDright(PID *right, PID *rightAngle)
{
	rightPID = *right;
	rightAnglePID = *rightAngle;
}
void Drive::setPIDleft(PID *left, PID *leftAngle)
{
	leftPID = *left;
	leftAnglePID = *leftAngle;
}

