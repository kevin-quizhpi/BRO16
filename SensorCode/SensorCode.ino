#define CAPACITY 50       // Order of the Moving Avg filter
                          // The higher the order the longer it takes to change output. Helps smooth erratic sensor inputs

// Struct for Sensor has parameters: Sum, Avg & prevValues      
struct SensorV2
{

  int prevValues[CAPACITY];     // A circular buffer that has the CAPACITY previous values of Input Sensor
  int sum;                      // Holds the sum of the previous values of the Input
  int avg;                      // Avg of the values in the buffer | Output of Moving Avg Filter
};

/* Implements the Moving Avg Filter
 *  Because we have the sum of the buffer in a variable 
 *  we can update the avg when a new Input
*/
int updateAvg(struct SensorV2 *sensor, int current, int counter)
{  
  int temp = sensor->prevValues[counter % CAPACITY]; // Value in buffer that will be replaced
  sensor->prevValues[counter % CAPACITY] = current;
  sensor->sum += -temp + current;
  return sensor->avg = sensor->sum / CAPACITY;
}

int counter = 0;    
double BLa = 0.0001;
double BLb = -0.0695;
double BLc = 16.4856;


// Sensor pin assignments  - Changed rear left & front left pin assignments
#define rearLeftSensor  23  
#define rearRightSensor  22 

//Front Left sensor
#define frontRightSensor 14 
//Front right sensor
#define frontLeftSensor  15


// Using old motor driver, so we can't use two enables, one is an enable and the other is pwm
// Right motor - A
#define RightPWM 20   // AIN1 // PWM in old driver
#define RightIN 21   // AIN2 // Enable in Old driver
//#define RightPWM 5  // not using in b/c old driver


// Left motor - B
#define LeftPWM 18    // BIN1 // PWM in old driver
#define LeftIN 17    // BIN2 // Enable in old driver
//#define LeftPWM 6   // Not using b/c old driver

#define STBY 19       // Standby Pin, Low = Standby. Documentation https://www.pololu.com/file/0J86/TB6612FNG.pdf


// Initialize Structs

struct Motor
{
  int IN;      // Enable 1 for motor
  //int IN2;      // Enable 2 for motor // No second enable for motor
  int PWM;      // PWM input for motor
};

void SetPinMode(struct Motor *motor, int in1, int Pwm) // Becareful with the Capitalization of Set, // Removed second enable 
{
  motor->IN = in1;
  //motor->IN2 = in2;
  motor->PWM = Pwm;
  pinMode(motor->IN, OUTPUT);
  //pinMode(motor->IN2, OUTPUT);
  pinMode(motor->PWM, OUTPUT);

}

//Sets the speed of motor's individually
void setSpeed(struct Motor *motor, int Spd)
{
  analogWrite(motor->PWM, Spd);
}


// IN1(High) & IN2(Low) = Clockwise
// IN1(Low) IN2(High) = CounterClockwise

// Left Clockwise & Right CounterClockwise = Backwards
// Right Clockwise & Left CounterClockwise = Forwards

void setDirection(struct Motor *left, struct Motor *right, bool direc)    // Changed for old driver, have to switch back eventually
{
  // boolean direc sets direction, if true forwards, false -> backwards
  // Set IN variables to opposites for left & right

  // Set direction to backwards
  if (direc == false)
  {
    //digitalWrite(left->IN2, LOW);
    digitalWrite(left->IN, LOW);
    //digitalWrite(right->IN2, HIGH);
    digitalWrite(right->IN, LOW);
  }
  // Set direction to forwards
  else
  {
    digitalWrite(left->IN, HIGH);
    //digitalWrite(left->IN2, HIGH);
    digitalWrite(right->IN, HIGH);
    //digitalWrite(right->IN2, LOW);
  }
}

// Initializing Sensor Structs
SensorV2 rearLeft;
SensorV2 frontLeft;
SensorV2 rearRight;
SensorV2 frontRight;

// Intializing Motor Structs
Motor Left;
Motor Right;




int Spd = 230;
void setup()
{
  Serial.begin(9600);

  pinMode(rearLeftSensor,INPUT);
  pinMode(frontLeftSensor,INPUT);
  pinMode(rearRightSensor, INPUT);
  pinMode(frontRightSensor,INPUT); 


  SetPinMode(&Left, LeftIN , LeftPWM);
  SetPinMode(&Right, RightIN, RightPWM);
  pinMode(STBY, OUTPUT);
  
  setDirection(&Left, &Right, true);
  setSpeed(&Left, Spd);   // Set Speed Left
  setSpeed(&Right, Spd);  // Set Speed Right


}

double analogToDistance(double a, double b, double c, unsigned int AnalogVal)
{
  return a * AnalogVal * AnalogVal + b * AnalogVal + c;
}


void loop()
{

  digitalWrite(STBY, HIGH); // Third enable for Motors, STANDBY = LOW -> motor don't run
    setDirection(&Left, &Right, true);
  setSpeed(&Left, Spd);   // Set Speed Left
  setSpeed(&Right, Spd);  // Set Speed Right

  int distRearLeft = analogRead(rearLeftSensor); // rear left sensor
  int distRearRight = analogRead(rearRightSensor); // rear right sensor
  int distFrontLeft = analogRead(frontLeftSensor); // front left sensor
  int distFrontRight = analogRead(frontRightSensor); // front right sensor


  Serial.print("\nRear left: ");
  Serial.print(distRearLeft);
  int rLeftFiltered = updateAvg(&rearLeft, distRearLeft, counter);
  Serial.print("\t Rear Left filtered : " );
  Serial.print(rLeftFiltered);
  
  int rRightFiltered = updateAvg(&rearRight, distRearRight, counter);
  Serial.print("\t Rear Right filtered: ");
  Serial.print(rRightFiltered);

  int fRightFiltered = updateAvg(&frontRight, distFrontRight, counter);
  Serial.print("\t Front Right Filtered: ");
  Serial.print(fRightFiltered);

  //Serial.print("\t || Front left: ");
  //Serial.print(distFrontLeft);
  int fLeftFiltered = updateAvg(&frontLeft,distFrontLeft, counter);
  Serial.print("\t Front Left filtered: ");
  Serial.print(fLeftFiltered);

  counter++;
//  Serial.print("\t Rear Left in cm: " );
  //Serial.print( analogToDistance(BLa, BLb, BLc, rLeftFiltered) );



}
