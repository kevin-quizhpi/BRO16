#include "Motor.h"


Motor::Motor(int in1, int in2, int pwm) // Contructor parameters correspond to pins
{
	IN1 = in1;
	IN2 = in2;
  	PWM = pwm;
}

void Motor::SetPinMode()
{
	pinMode(IN1, OUTPUT);
  	pinMode(IN2, OUTPUT);
  	pinMode(PWM, OUTPUT);
}

void Motor::setClockwise()
{
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
}
void Motor::setCounterClockwise()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
}
void Motor::stopMotor()
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
}
void Motor::setSpeed(int Speed)
{
  Speed = constrain(Speed,0,255);
	analogWrite(PWM, Speed);
}
Motor::Motor()
{
	
}
