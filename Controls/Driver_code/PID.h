#pragma once
#include "Arduino.h"

class PID
{
	private:
		float Kp;
		float Ki;
		float Kd;
		float sum;
		float P,I,D;
		float prevError;
		float total;

	public:

		PID(int p, int i, int d);
		PID();
		float getAdjustment(int error);
		void resetError();

};