#include <NewPing.h>

#define TRIGGER_PINL  A3  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A2  // Arduino pin tied to echo pin on ping sensor.


int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4


float P = 0.7 ;
float D = 0.5 ;
float I = 0.4 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;

int wall_threshold = 13 ;
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 7 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;


const int E1 = 10;
const int M1 = 12;
const int E2 = 11;
const int M2 = 13;

int baseSpeed = 70 ;

int RMS ;
int LMS ;

//int LED = 13 ;
//int led1 = 8 ;
//int led2 = 9 ;



NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;
//int TestNUM = 1  ;



// new variable. made by habib
int pLeftError = 0, pRightError = 0, pFrontError = 0;
float pidFrwd[2] = {4, 10};
float pidTurn[2] = {2, 5};
int DIRECTION;

float leftMotor;
float rightMotor;

int leftError = 0, rightError = 0, frontError = 0;


void setup() {

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


  //  for (int i = 2; i <= 13; i++) //For Motor Shield
  //    pinMode(i, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);


  first_turn = false ;
  rightWallFollow = false ;
  leftWallFollow = false ;


}

void loop() {
  // variable declaration
  int nextDir;

  //========================================START========================================//


  ReadSensors();

  // getting error value from all sensor
  leftError, rightError, frontError = walls();

  // get next direction
  nextDir = getDirection();

  // get motor speed
  getSpeed(nextDir, leftError, rightError, frontError);

  // give an action to the motor
  setDirection(DIRECTION, leftMotor, rightMotor);

  // read sensors & print the result to the serial monitor //
  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" cm ");
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.print(" cm ");
  Serial.print(" Front : ");
  Serial.print(frontSensor);
  Serial.println(" cm ");

  Serial.print(" Fronterror : " + frontError);
  Serial.print(" Lefterror : " + leftError);
  Serial.println(" Righterror : " + rightError);

  Serial.print("Left Motor : ");
  Serial.print(leftMotor);

  Serial.print("Right Motor : ");
  Serial.println(rightMotor);

  Serial.print("Direction : ");
  Serial.println(DIRECTION);




  //measure error & print the result to the serial monitor
  Serial.print("error=");
  Serial.println(totalError);
}





//--------------------------------- speed control ---------------------------------//

void getSpeed(int nextDir, int leftError, int rightError, int frontError) {     // motor will not stop except in a STOP direction
  // variable declaration
  float forward; //, leftMotor, rightMotor;

  // set speed by frontError
  forward = pidFrwd[0] * frontError + pidFrwd[1] * (frontError - pFrontError);
  forward = constrain(forward, 10, 255);
  // update previous front error
  pFrontError = frontError;

  // determine speed of left motor
  if (nextDir ==  LEFT) {
    rightMotor = forward;
    leftMotor = pidTurn[0] * leftError + pidTurn[1] * (leftError - pLeftError);
    leftMotor = constrain(leftMotor, 0, 30);      // limitation. minimum value is 0 and maximum value is 30.
    leftMotor = forward - leftMotor;


  } else if (nextDir == RIGHT) {
    leftMotor = forward;
    rightMotor = pidTurn[0] * rightError + pidTurn[1] * (rightError - pRightError);
    rightMotor = constrain(rightMotor, 0, 30);
    rightMotor = forward - rightMotor;

  } else if (nextDir == FORWARD) {
    leftMotor = forward;
    rightMotor = forward;

  } else {        // backward
    leftMotor = forward * -1;       // just an assumption
    rightMotor = forward * -1;
  }

}



//--------------------------------- direction control ---------------------------------//

int getDirection() {

//  if (leftwall == true && rightwall == false && frontwall == true ) {
//    DIRECTION = RIGHT; //4;
//
//  } else if (leftwall == false && rightwall == false && frontwall == true ) {
//    DIRECTION = BACKWARD;//2
//
//  } else if (leftwall == false && rightwall == true && frontwall == true ) {
//    DIRECTION = LEFT; // 3;
//
//  } else if ( leftSensor <= 5 || leftSensor > 100 && rightSensor <= 5 || rightSensor > 100 && frontSensor <= 5 || frontSensor > 100 ) {
//    //jika senso kurang
//    DIRECTION = STOP ;//0
//
//  } else {
//    DIRECTION = FORWARD; // 1
//  }

    DIRECTION = FORWARD;

  return DIRECTION;
}


void setDirection(int dir, int left, int right) {

  if ( dir == FORWARD || dir == LEFT || dir == RIGHT) { //maju
    analogWrite(E1, right);   // E1 = RIGHT MOTOR
    analogWrite(E2, left);   // E2 = LEFT MOTOR
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
  }
  else if ( dir == STOP ) { //stop
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
  }
  else if ( dir == BACKWARD ) { //mundur
    analogWrite(E1, left);
    analogWrite(E2, right);
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

  //leftSensor = sonarLeft.ping_median(TestNUM);     //accurate but slow
  //rightSensor = sonarRight.ping_median(TestNUM);     //accurate but slow
  //frontSensor = sonarFront.ping_median(TestNUM);     //accurate but slow

  //leftSensor = sonarLeft.convert_cm(leftSensor);
  //rightSensor = sonarRight.convert_cm(rightSensor);
  //frontSensor = sonarFront.convert_cm(frontSensor);

  lSensor = sonarLeft.ping_cm(); //ping in cm
  rSensor = sonarRight.ping_cm();
  fSensor = sonarFront.ping_cm();


  leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  rightSensor = (rSensor + oldRightSensor) / 2;
  frontSensor = (fSensor + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;

}

//---------------------------------------------------------------------------//


//--------------------------------- control ---------------------------------//




//--------------------------- wall detection --------------------------------//

int walls() {
  // pendeteksian apakah dinding ada atau tidak. jika kurang dari wall treshold baru dikatakan dinding
  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
    leftError = leftSensor - wall_threshold;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;
    rightError = rightSensor - wall_threshold;

  }

  if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
    frontError = frontSensor - front_threshold;
  }

}



//---------------------------------------------------------------------------/


//---------------------------------------------------------------------------//
