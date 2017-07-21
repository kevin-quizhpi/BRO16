#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> // USED Encoder library included with Tinsyduino. Documentation:  https://www.pjrc.com/teensy/td_libs_Encoder.html

// Constants
#define CIRCUMFERENCE 12.61349    // Wheel circumference
#define MotorGearRatio 29.86      // Exact motor gear ratio


// Right motor
#define RightIN1 20   // AIN1
#define RightIN2 21   // AIN2
#define RightPWM 5

// Left motor
#define LeftIN1 18    // BIN1
#define LeftIN2 17    // BIN2
#define LeftPWM 6

#define STBY 19       // Standby Pin, Low = Standby. Documentation https://www.pololu.com/file/0J86/TB6612FNG.pdf

// Signal for ultrasonic sensor
#define signal 14
 unsigned long echo = 0;
 unsigned long ultrasoundValue = 0;


// Encoders           //Encoder declarations, eventually move it into a struct and hopefully a class on it's own.
// - Right wheel
#define RightA 0
#define RightB 1
// - Left wheel
#define LeftA 2
#define LeftB 3

//Encoder Objects
Encoder encRight(RightA, RightB);
Encoder encLeft(LeftA, LeftB);

//Initialization for the encoders; choose large negative # to prevent overflow
long positionLeft  = -999;
long positionRight = -999;

// Variables used to get time
unsigned long prevMillis = 0;
unsigned long interval = 1500;

//PID
static float previous_error = 0; //Initially no previous Error
static float Kp = 11000, Ki =1 , Kd = 4000;      // constants for scaling P I D effects (will need adjusting) **K- Changed to thousands.
static float error, P, I = 0,  D;      // error variables

float total; //Keeps track of total PID Error

// Ultra sonic test code
/*
 * Need to change to use micros(), even though it's a minor delay it can still effect
 */
unsigned long ping()
{
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



// Need to account for Standby somehow, probably a method.
struct Motor
{
  int IN1;      // Enable 1 for motor
  int IN2;      // Enable 2 for motor
  int PWM;      // PWM input for motor
};

void SetPinMode(struct Motor *motor, int in1, int in2, int Pwm) // Becareful with the Capitalization of Set
{
  motor->IN1 = in1;
  motor->IN2 = in2;
  motor->PWM = Pwm;
  pinMode(motor->IN1, OUTPUT);
  pinMode(motor->IN2, OUTPUT);
  pinMode(motor->PWM, OUTPUT);

}

// IN1(High) & IN2(Low) = Clockwise
// IN1(Low) IN2(High) = CounterClockwise

// Left Clockwise & Right CounterClockwise = Backwards
// Right Clockwise & Left CounterClockwise = Forwards

void setDirection(struct Motor *left, struct Motor *right, bool direc)
{
  // boolean direc sets direction, if true forwards, false -> backwards
  // Set IN variables to opposites for left & right

  // Set direction to backwards
  if (direc == false)
  {
    digitalWrite(left->IN2, LOW);
    digitalWrite(left->IN1, HIGH);
    digitalWrite(right->IN2, HIGH);
    digitalWrite(right->IN1, LOW);
  }
  // Set direction to forwards
  else
  {
    digitalWrite(left->IN1, LOW);
    digitalWrite(left->IN2, HIGH);
    digitalWrite(right->IN1, HIGH);
    digitalWrite(right->IN2, LOW);
  }
}
// Set all enables = low to stop motor
// Need to stop before changing direction
// If we don't stop we can drift
void stopMotor(struct Motor *left, struct Motor *right)
{
  digitalWrite(left->IN1, LOW);
  digitalWrite(left->IN2, LOW);
  digitalWrite(right->IN2, LOW);
  digitalWrite(right->IN1, LOW);
}

// Temporary method to test using structs
void setSpeedBoth(struct Motor *left, struct Motor *right, int Spd)
{
  analogWrite(left->PWM, Spd);
  analogWrite(right->PWM, Spd);
}

//Sets the speed of right motor individually
void setSpeedRight(struct Motor *right, int Spd)
{
  analogWrite(right->PWM, Spd);
}

//Sets the speed of left motor individually
void setSpeedLeft(struct Motor *left, int Spd)
{
  analogWrite(left->PWM, Spd);
}

//NEED TO ADD ANALOGSENSORS TO THIS STILL***** KEVIN IS WORKING ON THIS I BELIEVE
/** Drives robot in a straight line using PID to control encoder ticks
   @param   left : motor struct for the left motor
   @param   right: motor struct for the right motor
   @param   leftTicks: number of ticks for left encoder
   @param   rightTicks: number of ticks for right encoder
   @param   spd: desired straight speed
*/
void driveStraight(struct Motor *left, struct Motor *right, long leftTicks, long rightTicks, int Spd)
{
  //  stopMotor(left, right); //Stops motors -- Removings because motors should stop before changing direction
  //setDirection(left, right, true);  //Sets direction of Motors to run forward -- Driving forward should be set outside of drive straight function

  error = rightTicks - leftTicks; //Using number of ticks as error

  P = error * Kp; //Proprotional Error
  I = (I + error) * Ki; //Integral/Accumulated Error

  D = (error - previous_error) * Kd;//Derivative; looks at current error compared to previous error
  previous_error = error;

  total = P + I + D; //Total "Error"; used as an adjusting factor
  //total = (int)total%Spd;

  //Sets speed of motors based on total error
  //setSpeedRight(right, Spd - total);  -- THis PID should adjuct proportional to the right wheel
  setSpeedLeft(left, Spd + total);

  //Prints to serial for debugging; comment this out once PID works
  Serial.print("\nSpeed Right: ");
  Serial.print(Spd);
  Serial.print("\t- Right Ticks: ");
  Serial.print(rightTicks);
  Serial.print("\t||\tSpeed Left: ");
  Serial.print(Spd + total);
  Serial.print("\t- Left Ticks: ");
  Serial.print(leftTicks);

}

/*  For now ramps speed by increments of 1 until desired speed is reached
    - SHould ramp up quicker than that.
    - Or be able to ramp up in difference increments
*/
void ramp(struct Motor *left, struct Motor *right, int nowSpd, int newSpd, boolean dir)
{
  //accelearates or deccelerates both Motors at the same time(dictated by boolean value)
  if (dir) {
    for (int k = nowSpd; k < newSpd; k++) {
      Serial.print(encRight.read());
      Serial.print(encLeft.read());
      setSpeedRight(right, k);
      Serial.print("\nRight: ");
      setSpeedLeft(left, k);
      Serial.print("\nLeft: ");
      delay(10);
    }
  }
  else {
    for (int k = nowSpd; k > newSpd; k--) {
      Serial.print(encRight.read());
      Serial.print(encLeft.read());
      setSpeedRight(right, k);
      Serial.print("\nRight: ");
      setSpeedLeft(left, k);
      Serial.print("\nLeft: ");
      delay(10);
    }
  }
}

int motorSpeed(int prev, int current)
{
  return (CIRCUMFERENCE / MotorGearRatio) / (current - prev) * 1000;
}


// Intialized Motor Structs
Motor Left;
Motor Right;


void setup()
{
  Serial.begin(9600);
  
  pinMode(signal,OUTPUT);

  // intiallizing Motors
  SetPinMode(&Left, LeftIN1 , LeftIN2, LeftPWM);
  SetPinMode(&Right, RightIN1, RightIN2, RightPWM);
  pinMode(STBY, OUTPUT);
  setDirection(&Left, &Right, true);
  setSpeedBoth(&Left, &Right, 75);

}

void loop()
{
  int Spd = 50; // Sets speed to drive robot straight

  digitalWrite(STBY, HIGH); // Third enable for Motors, STANDBY = LOW -> motor don't run


  /*READ ENCODERS*/
  long newLeft, newRight; //stores values of the encoders
  //Get the new values of the encoders



  /*
     Using Micros to find time it takes to get an value from encoder and then from there find the speed of the wheel
     Almost there. Gear ratio & circumference need to be double checked
  */

  newRight = encRight.read();
  newLeft = encLeft.read();
  //  unsigned long prevMillis = micros();
  //  Serial.print("\nPrevious Millis values: ");
  //  Serial.print(prevMillis);
  //
  //  Serial.print("\t|\t Current Millis values: ");
  //  unsigned long currentMillis = micros();
  //  Serial.print(currentMillis);
  //  Serial.print("\t|\tVelocity Right: ");
  //  Serial.print(motorSpeed(prevMillis,currentMillis));
  //
  //
  //  prevMillis = millis();
  //
  //  currentMillis = millis();
  //  Serial.print("\t| Velocity Left: ");
  //  Serial.print(motorSpeed(prevMillis,currentMillis));


  /*DRIVE IN STRAIGHT LINE*/

  //Drives robot straight at speed Spd
  driveStraight(&Left, &Right, newLeft, newRight, Spd);
  //if(ping() < 15)
  {
   // stopMotor(&Left,&Right);
  }
 // else
  {
   // setDirection(&Left, &Right, true);
  }
    


  /* PRINT DATA FOR DEBUGGING*/
  /*
    if (newLeft != positionLeft || newRight != positionRight)
    {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
    }
  */

  /* Measurements:

      Wheel Diameter: 40.15mm
      Circumference: 12.56637cm w/ D = 4cm
      Circumference: 12.61349 exact
      Approximate Gear ratio: 30:1
      Exact Gear ratio: 29.86:1     Meaning everytime the wheel rotates once, the extended shaft rotates 29.86 times


  */
}

