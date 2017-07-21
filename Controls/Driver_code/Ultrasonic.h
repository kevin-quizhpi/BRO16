#pragma once
#include "Arduino.h"
#define CAPACITY 50


class Ultrasonic
{
	private:
		float pingtime;
		float pingShoot;
		int sonicState;
		int Trig;
		int Echo;
		unsigned long echo;
		float prevValues[CAPACITY];
		float sum;
		float avg;
		float pinNum;


	public:
		Ultrasonic(int trig, int Echo);
    	Ultrasonic();
		float input();
		float Read(int counter);
		void SetPinMode();

};
