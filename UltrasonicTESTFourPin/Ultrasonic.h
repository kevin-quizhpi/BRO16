#pragma once
#include "Arduino.h"
#define CAPACITY 50


class Ultrasonic
{
	private:

    int signal;
    
	public:
		Ultrasonic(int sig);
    Ultrasonic();
		int input();
		void SetPinMode();

};
