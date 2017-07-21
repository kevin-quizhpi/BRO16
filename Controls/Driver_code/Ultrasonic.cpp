#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trig, int echo)
{
	Trig = trig;
	Echo = echo;
}
Ultrasonic::Ultrasonic()
{
  
}
void Ultrasonic::SetPinMode()
{
	pinMode(Echo,INPUT);
  	pinMode(Trig,OUTPUT);
}

float Ultrasonic::input()
{
	int dist = 0;
	if((millis() - pingShoot) > 100)
	{
		if(sonicState == 0) 
		{
			pingtime = millis();
			sonicState = 1;
		}

		if((millis() - pingtime) <= 2 && sonicState == 1)
		{
			//Serial.println("Sonic 0");
			digitalWrite(Trig,LOW);
			sonicState = 2;
		}
		if((millis() - pingtime) > 2 && (millis() - pingtime) < 5 && sonicState == 2)
		{
			//Serial.println("Sonic 1");
			digitalWrite(Trig, HIGH);
			sonicState = 3;
		}
		if((millis() - pingtime) > 5 || sonicState == 3) 
		{
			//Serial.println("Sonic 2");
			digitalWrite(Trig, LOW);
			echo = pulseIn(Echo,HIGH);
			dist = (echo/58.138);
			sonicState = 0;
			pingtime = millis();
			pingShoot = millis();
		}
	}
	return dist;

}
float Ultrasonic::Read(int counter)
{
	float current = input();
	float temp = prevValues[counter % CAPACITY];
	prevValues[counter % CAPACITY] = current;
	sum += temp + current;
	avg = sum/CAPACITY;
	return avg;
}
