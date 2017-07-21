#pragma once
#include "Arduino.h"

class Motor
{
	private: 
		int IN1;
		int IN2;
		int PWM;

	public:
		Motor(int in1, int in2, int pwm);	// Sets values of pins, and declares pinModes
		Motor();
		void SetPinMode();
		void setClockwise();
		void setCounterClockwise();
		void stopMotor();
		void setSpeed(int Speed);

};