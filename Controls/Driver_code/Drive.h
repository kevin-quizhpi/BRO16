#pragma once

#include "Sensor.h"
#include "Motor.h"
#include "PID.h"
#include "Ultrasonic.h"
#include <Encoder.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#define SensorThreshold 400

// Ideally the distance should be translated from the raw data so they're both
#define LeftDistanceFromWall 300		// Temporary has to be tested
#define RightDistanceFromWall 300		// Temporary has to be tested 

// Encoders           //Encoder declarations, eventually move it into a struct and hopefully a class on it's own.
// - Right wheel

// #define RightA 0

// #define RightB 1

// - Left wheel

// #define LeftA 2

// #define LeftB 3

class Drive
{
	private:
		Motor right;
		Motor left;
		Sensor rearLeft;
		Sensor frontLeft;
		Sensor rearRight;
		Sensor frontRight;
		Ultrasonic front;
		PID encoderPID;
		PID sensorPID;
		PID anglePID;
		PID leftPID;
		PID rightPID;
		PID leftAnglePID;
		PID rightAnglePID;
		int Spd;
		long rightTurnLeftTicks = 300; //Number of encoder ticks in a 90 degree rotation
		long rightTurnRightTicks = 270;
		long leftTurnLeftTicks = -270; //Number of encoder ticks in a 90 degree rotation
    	long leftTurnRightTicks = -270;


	public:
		Drive(int Speed);
		// Passing pointers to Class
		void setMotors(Motor *l, Motor *r);
		void setSensors(Sensor *rl, Sensor *fl, Sensor *rr, Sensor *fr);
		void setUltrasonic(Ultrasonic *f);
		void setPIDencoder(PID *enc);
		void setPIDsensor(PID *sens);
		void setPIDangle(PID *ang);
		void setPIDright(PID *right, PID *rightAngle);
		void setPIDleft(PID *left, PID *leftAngle);


		// Setting direction and Speed
		void setForward();
		void Stop();
		void setRotateRight();
		void setRotateLeft();
		void setSpeedBoth(int Speed);
		void SetSpeed(int Speed);	// Changes Speed value globalling

		// Check for walls
		bool checkFrontSensorsWalls(int counter); 	// Front two sensors
		bool checkRearSensorsWalls(int counter);	// Rear two sensors
		bool checkBothWalls(int counter);			// If there's walls on both sides
		bool checkFrontWall(int counter);			// Checks if there's a wall in front
		bool checkLeftWall(int counter);		
		bool checkRightWall(int counter);


		// Movement Methods
		void moveForward(int counter, int encError); 	// Moves forward for all cases
		void turnRight(int LA, int LB, int RA, int RB);
		void turnLeft(int LA, int LB, int RA, int RB);
		void turnAround(int LA, int LB, int RA, int RB);

		// Error fetching methods
		int getCenteringError(int counter);
		int getAngleError(int counter);
		int getLeftCenterError(int counter);
		int getRightCenterError(int counter);
		int getLeftAngleError(int counter);
		int getRightAngleError(int counter);
		




};
