#include <Arduino.h>

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

#include <Servo.h>

#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

#define LN_SENS_PIN_RIGHTEDGE 22 // right edge sensor - Connected to D1 pin of the sensor
// #define LN_SENS_PIN_RIGHT 23       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_RIGHT 25  // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_MIDDLE 24 // middle sensor - Connected to D3 pin of the sensor
// #define LN_SENS_PIN_LEFT 25       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFT 23     // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFTEDGE 26 // left edge sensor - Connected to D5 pin of the sensor
#define LN_SENS_CALIB_PIN 27    // Connected to CAL pin of the sensor
#define LN_SENS_ANALOG_PIN A15  // Connected to AN pin of the sensor

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
// Robot length measured on the robot is 25.0 cm.
// Robot width measured on the robot is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.
const int MazeCorridorWidth = 50;

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth / 2;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

const float CenterLineTolerance = 2.5; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;  // Measured by experiments with the robot
const int WallFollowingSide = -90;     //Set: -90 for right wall following or +90 for left wall following
                                       //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                       // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
//const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int RightServoAngle = 0;
const int LeftServoAngle = 180;
const int FrontServoDelay = 150;
const int SideServoDelay = 150;

const int LeftSpeed = 95;
const int RightSpeed = 95;

int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
bool getTurn = true;
bool moveFront = true;
bool lineFollower = true;

Servo myservo;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle
float frontThreshold();

int left();
int mid();
int right();
int leftEdge();
int rightEdge();

//-----------------------------------------------

void setup()
{
  pinMode(LN_SENS_PIN_RIGHTEDGE, INPUT);
  pinMode(LN_SENS_PIN_RIGHT, INPUT);
  pinMode(LN_SENS_PIN_MIDDLE, INPUT);
  pinMode(LN_SENS_PIN_LEFT, INPUT);
  pinMode(LN_SENS_PIN_LEFTEDGE, INPUT);
  // pinMode(LN_SENS_CALIB_PIN, OUTPUT);
  pinMode(LN_SENS_ANALOG_PIN, INPUT);

  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);
  myservo.attach(ServoPin);
  myservo.write(90); //Move the servo to center position

  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();
  //delay(4000);
}

//---------------------------------------------------------

void loop()
{
  float frontDistance;
  float rightSideDistance = 0.0;
  float leftSideDistance = 0.0;

  frontDistance = getDistance(FrontServoAngle, FrontServoDelay);


  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();

  if (frontDistance <= 15.0 && getTurn) //Стената отпред е близко
  {
    Serial.print("left turn");
    turnLeft90Degrees();
    delay(360);
    while (rightSideDistance <= 50)
    {
      rightSideDistance = getDistance(RightServoAngle, SideServoDelay);
    }
    speedRight = RightSpeed * .75;
    speedLeft = LeftSpeed * .75;
    moveForward();
    delay(500);
    turnRight90Degrees();
    delay(280);
    getTurn = !getTurn;
  }

  else if (frontDistance <= 15.0 && !getTurn) //Стената отпред е близко
  {
    Serial.print("right turn");
    turnRight90Degrees();
    delay(280);
    while (leftSideDistance <= 50)
    {
      leftSideDistance = getDistance(LeftServoAngle, SideServoDelay);
    }
    speedRight = RightSpeed * .75;
    speedLeft = LeftSpeed * .75;
    moveForward();
    delay(500);
    turnLeft90Degrees();
    delay(360);
    getTurn = !getTurn;
  }
}

//==================================== FUNCTIONS =====================================================

void moveForward() // Move forward
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedRight);
  analogWrite(RIGHT_BACK, LOW);
}

void turnLeft90Degrees() // Turn Left90Degrees
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, 255);
  analogWrite(RIGHT_FOR, 255);
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void turnRight90Degrees() // Turn Right90Degrees
{
  analogWrite(LEFT_FOR, 255);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, 255);
}

void stopMoving() // Stop movement
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance(int servoAngle, int delayAfterServoMovement)
{
  float distance;
  myservo.write(servoAngle);
  //---------------------------150 millis
  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();
  delay(35);
  stopMoving();
  delay(20);
  moveForward();
  delay(35);
  stopMoving();
  delay(20);
  moveForward();
  delay(40);
  stopMoving();
  //-----------------------
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}

int left()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_RIGHT);
  return distance;
}

int mid()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_MIDDLE);
  return distance;
}

int right()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_LEFT);
  return distance;
}

int leftEdge()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_RIGHTEDGE);
  return distance;
}

int rightEdge()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_LEFTEDGE);
  return distance;
}

float frontThreshold()
{
  float distance;
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}
