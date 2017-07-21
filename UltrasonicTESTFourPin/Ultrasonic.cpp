#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int sig)
{
  signal = sig;
}
Ultrasonic::Ultrasonic()
{
  
}
void Ultrasonic::SetPinMode()
{
  pinMode(signal,OUTPUT);
}

int Ultrasonic::input()
{
  int ultrasoundValue;
  int echo;
  pinMode(signal,OUTPUT); 
  digitalWrite(signal,LOW);
  delayMicroseconds(2);
  digitalWrite(signal,HIGH);
  delayMicroseconds(5);
  digitalWrite(signal,LOW);
  pinMode(signal,INPUT);
  digitalWrite(signal,HIGH);
  echo = pulseIn(signal,HIGH);
  ultrasoundValue = (echo/58.138);
  return ultrasoundValue;

}

