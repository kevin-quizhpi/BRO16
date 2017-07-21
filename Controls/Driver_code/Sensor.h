#pragma once
#include "Arduino.h"
#define CAPACITY 50


class Sensor
{
	private:
		int prevValues[CAPACITY];
		int sum;
		int avg;
		int pinNum;


	public:
		int readSensor(int counter);
		void SetPinMode();
		Sensor(int pin);
		Sensor();
};