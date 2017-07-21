// Signal for ultrasonic sensor
#include "Ultrasonic.h"

#define signal 14






Ultrasonic front(signal);


void setup() {
  //front.SetPinMode();
  Serial.begin(9600);
  Serial.println("This thing does work");

}
int counter = 0;
void loop() 
{
  Serial.print("\nData: ");
  int dist = front.input();
  if(dist != 0)
    Serial.print(dist);
  counter++;

}
