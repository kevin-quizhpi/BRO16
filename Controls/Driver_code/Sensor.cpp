# include "Sensor.h"


Sensor::Sensor(int pin)
{
	pinNum = pin;
	sum = 0;
	avg = 0;
}

void Sensor::SetPinMode()
{
	pinMode(pinNum, INPUT);
}
Sensor::Sensor()
{
	
}

int Sensor::readSensor(int counter)
{
	int current = analogRead(pinNum);
	int temp = prevValues[counter % CAPACITY];
	prevValues[counter % CAPACITY] = current;
	sum += temp + current;
	avg = sum/CAPACITY;
	return avg;
}