#include <EncoderLib123_Fall2016.h>
#include <SoftwareSerial.h>
#include "CPE123_Fall16.h"
#include "CPE123_EspAPI_Fall16.h"

enum{NO_COMMAND, LEFT, RIGHT, FORWARD, BACKWARD, STOP};


Led ledRight(10);
Led ledCenter(11);
Led ledLeft(12);
Led ledExtremeRight(4);
Led ledExtremeLeft(13);

const int irRightPin = A0;
const int irCenterPin = A1;
const int irLeftPin = A2;
const int irExtremeRightPin = A3;
const int irExtremeLeftPin = A4;

const int motorLA = 8;
const int motorLB = 9; //positive

const int motorRA = 7; //positive
const int motorRB = 6;

const int rightPin1 = 21;
const int rightPin2 = 20;

const int leftPin1 = 2;
const int leftPin2 = 3;

Button startButton = 5;

const static int kp = 30;
const static int ki = 20;
static int I = 0;



void setup() {
  Serial.begin(9600);
  Serial.println(__FILE__);
  printLibVersion;
  motorsSetup();

  encoderSetup(rightPin1, rightPin2, leftPin1, leftPin2);

  while(!startButton.wasPushed()){
  }
}

void loop() {
  // put your main code here, to run repeatedly: 
  motorControl2();
}

void motorControl2(){
  int PIVal = CalculatePI2();
  int initialSpeed = 120;
  int leftMotorSpeed = initialSpeed + PIVal;
  int rightMotorSpeed = initialSpeed - PIVal;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  if ( PIVal != -10000 && PIVal != 10000 && PIVal != 20000){
    leftMotorForward(leftMotorSpeed);
    rightMotorForward(rightMotorSpeed);
  } else if(PIVal == 10000) {
    leftMotorForward(100);
     rightMotorForward(100);
     delay(300);
     leftMotorForward(255);
     rightMotorBackward(255);
     delay(400);
  } else if(PIVal == -10000) {
     rightMotorForward(100);
     leftMotorForward(100);
     delay(300);
     leftMotorForward(255);
     rightMotorBackward(255);
     delay(400);
  } else if (PIVal == 20000){
    rightMotorBackward(100);
    leftMotorBackward(100);
  }
  
} 

int CalculatePI2(){
   int errorVal = determineErrorVal2();
   if (errorVal == -100){
    return -10000;
   } else if (errorVal == 100){
    return 10000;
   } else if (errorVal == 200){
    return 20000;
   }
   static int prevError = 0;
   int P = errorVal;
   I += errorVal - prevError;

   int piVal = (kp * P) + (ki * I);
   prevError = errorVal;
   return piVal;
}


int determineErrorVal2(){
  int sensorVal = determineBW();
  static int prevSensorVal = 0;
  static int prepreSensorVal = 0;
  static int errorVal = 0;
  
  if (sensorVal == 1 || sensorVal == 7 || sensorVal == 15){ //00001 or 00111 or 01111
    errorVal = 4;
  } else if (sensorVal == 3){ //00011
    errorVal = 3;
  }else if (sensorVal == 2){ //00010
    errorVal = 2;
  } else if (sensorVal == 6){//00110
    errorVal = 1;
  } else if (sensorVal == 4){//00100
    errorVal = 0;
  } else if (sensorVal == 12){//01100
    errorVal = -1;
  } else if (sensorVal == 8){//01000
    errorVal = -2;
  } else if (sensorVal == 24){//11000
    errorVal = -3;
  } else if (sensorVal == 16 || sensorVal == 28 || sensorVal == 30){//10000 or 11100 or 11110
    errorVal = -4;
  } else if (sensorVal == 0 && prevSensorVal < 0 ){
    errorVal = -5;
  } else if (sensorVal == 0 && prevSensorVal > 0){
    errorVal = 5;
  } else if (sensorVal == 0 && prevSensorVal == 0 && prepreSensorVal > 0){
    return -100;
  } else if (sensorVal == 0 && prevSensorVal == 0 && prepreSensorVal < 0){
    return 100;
  } else if (sensorVal == 0 && prevSensorVal == 0 && prepreSensorVal == 0){
    return 200;
  }else if ( sensorVal == 13 || sensorVal == 5){ // 01101 or 00101
    errorVal = 100;
  } else if (sensorVal == 22 || sensorVal == 20){ // 10110 0r 10100
    errorVal = -100;
  }

  prevSensorVal = sensorVal;
  prepreSensorVal = prevSensorVal;
  
  return errorVal;
}



void motorControl(){
  int PIVal = CalculatePI();
  int initialSpeed = 80;
  int leftMotorSpeed = initialSpeed + PIVal;
  int rightMotorSpeed = initialSpeed - PIVal;
  Serial.print("PID Value: ");
  Serial.println(PIVal);

  Serial.print("Left Speed: ");
  Serial.println(leftMotorSpeed);

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  if ( PIVal != -10000 && PIVal != 10000){
    leftMotorForward(leftMotorSpeed);
    rightMotorForward(rightMotorSpeed);
  } else if(PIVal == -10000) {
     rightMotorBackward(80);
     leftMotorBackward(80);
  }
  
} 

int CalculatePI(){
   int errorVal = determineErrorVal();
   if (errorVal == -100){
    return -10000;
   } else if (errorVal == 100){
    return 10000;
   }
   static int prevError = 0;
   int P = errorVal;
   I += errorVal - prevError;

   int piVal = (kp * P) + (ki * I);
   prevError = errorVal;
   return piVal;
}


int determineErrorVal(){
  int sensorVal = determineBW();
  static int prevSensorVal = 0;
  static int prepreSensorVal = 0;
  static int errorVal = 0;
  
  if (sensorVal == 1 || sensorVal == 7 || sensorVal == 15){ //00001 or 00111 or 01111
    errorVal = 4;
  } else if (sensorVal == 3){ //00011
    errorVal = 3;
  }else if (sensorVal == 2){ //00010
    errorVal = 2;
  } else if (sensorVal == 6){//00110
    errorVal = 0;
  } else if (sensorVal == 4){//00100
    errorVal = 0;
  } else if (sensorVal == 12){//01100
    errorVal = 0;
  } else if (sensorVal == 8){//01000
    errorVal = -2;
  } else if (sensorVal == 24){//11000
    errorVal = -3;
  } else if (sensorVal == 16 || sensorVal == 28 || sensorVal == 30){//10000 or 11100 or 11110
    errorVal = -4;
  } else if (sensorVal == 0 && prevSensorVal < 0 ){
    errorVal = 5;
  } else if (sensorVal == 0 && prevSensorVal > 0){
    errorVal = -5;
  } else if (sensorVal == 0 && prevSensorVal == 0 ){
    return -100;
  } 
  prevSensorVal = sensorVal;
  prepreSensorVal = prevSensorVal;
  
  return errorVal;
}


int determineBW(){
  static MSTimer timer(500);
  int sensorValue = 0;
  static int rightVal;
  static int centerVal;
  static int leftVal;
  static int extremeLeftVal;
  static int extremeRightVal;

  if ( analogRead(irExtremeRightPin) >= 500){
    //black
    extremeRightVal = 1;
    ledExtremeRight.on();
    sensorValue += 1;
  } else {
    //white
    extremeRightVal = 0;
    ledExtremeRight.off();
  }
  if ( analogRead(irRightPin) >= 500){
    //black
    rightVal = 1;
    ledRight.on();
    sensorValue += 2;
  } else {
    //white
    rightVal = 0;
    ledRight.off();
  }
  
  if ( analogRead(irCenterPin) >= 500){
    //black
    centerVal = 1;
    ledCenter.on(); 
    sensorValue += 4;
  } else {
    //white
    centerVal = 0;
    ledCenter.off();
  }

  if ( analogRead(irLeftPin) >= 400){
    leftVal = 1;
    ledLeft.on();
    sensorValue += 8;
  } else {
    leftVal = 0;
    ledLeft.off();
  }
  if ( analogRead(irExtremeLeftPin) >= 500){
    extremeLeftVal = 1;
    ledExtremeLeft.on();
    sensorValue += 16;
  } else {
    extremeLeftVal = 0;
    ledExtremeLeft.off();
  }
 
    if( timer.done() == true){
      Serial.print("Sensor Value: ");
      Serial.println(sensorValue);
  }
  
  return sensorValue;
}



















void stateMachine(){
  static int angle = 0;
  static int mSpeed = 0;
  static int distance = 0;
  static int state = NO_COMMAND;
  switch (state){
    case NO_COMMAND:
      state = processInput1(angle, mSpeed, distance);
    break;
    case LEFT:
      while(robotLeft(angle) != true){}
      state = processInput1(angle, mSpeed, distance);
    break;
    case RIGHT:
      while(robotRight(angle) != true){}
      state = processInput1(angle, mSpeed, distance);
    break;
    case FORWARD:
      while(robotForward(mSpeed, distance) != true){}
      state = processInput1(angle, mSpeed, distance);
    break;
    case BACKWARD:
      while(robotBackward(mSpeed, distance) != true){}
      state = processInput1(angle, mSpeed, distance);
    break;
    case STOP:
      robotStop();
      state = processInput1(angle, mSpeed, distance);
    break;
    default:
    break;
  }
}


void testInput(){
  int angle = 0;
  int mSpeed = 0;
  int distance = 0;
  int state = processInput1(angle, mSpeed, distance);
  if (state != NO_COMMAND){
    Serial.print("Command ");
    Serial.println(state);
    Serial.print("Angle: ");
    Serial.println(angle);
    Serial.print("Speed: ");
    Serial.println(mSpeed);
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.println();
  }
  
  
  
}
int processInput1(int & angle, int & mSpeed, int & distance){
  int returnValue = 0;
  while(!Serial.available()){}
  char command = Serial.read();
  if (command == 'L' || command == 'l'){
      angle = Serial.parseInt();
      if ( constrain(angle, 0, 361) == 0 || constrain(angle, 0, 361)== 361){
        returnValue = NO_COMMAND;
      } else {
        returnValue = LEFT;
      }
  } else if (command == 'R' || command == 'r'){
      angle = Serial.parseInt();
      if ( constrain(angle,0 , 361) == 0 || constrain(angle,0 , 361)== 361){
        returnValue = NO_COMMAND;
      } else {
        returnValue = RIGHT;
      }
  } else if (command == 'F' || command == 'f'){
      mSpeed = Serial.parseInt();
      distance = Serial.parseInt();
      if ( constrain(mSpeed, 149, 256) == 149 || constrain(mSpeed, 149, 256) == 256){
        returnValue = NO_COMMAND;
      } else if ( constrain(distance, 0, 101) == 0 || constrain(distance, 0, 101) == 101){
        returnValue = NO_COMMAND;
      } else {
        returnValue = FORWARD;
      }
  } else if (command == 'B' || command == 'b'){
      mSpeed = Serial.parseInt();
      distance = Serial.parseInt();
      if ( constrain(mSpeed, 149, 256) == 149 || constrain(mSpeed, 149, 256) == 256){
        returnValue = NO_COMMAND;
      } else if ( constrain(distance, 0, 101) == 0 || constrain(distance, 0, 101) == 101){
        returnValue = NO_COMMAND;
      } else {
        returnValue = BACKWARD;
      }
  } else if (command == 'S' || command == 's'){
      angle = 0;
      mSpeed = 0;
      distance = 0;
      returnValue = STOP;
  } else{
    returnValue = NO_COMMAND;
  }
  return returnValue;
}

 void printSamples(){
  int data[100];
  static MSTimer timer;
  timer.set(100);
  for (int i = 0; i < 100; i++){
    while(timer.done() == false){
      
    }
    data[i] = leftEncoderCount() - rightEncoderCount();
    resetRightEncoder();
    resetLeftEncoder();
    timer.set(100);
  }
  for ( int i = 0; i < 100; i++){
    Serial.print("Sample: ");
    Serial.print(i);
    Serial.print(" Difference: ");
    Serial.println(data[i]);
  }
}

int robotBackward(int mSpeed, int distanceInCm){
  static int returnValue = 0;
  static unsigned long neededTransitions = 0;
  static int temp = 0;
  if (returnValue == 1){
    returnValue = 0;
    temp = 0;
  }
  
  if(temp == 0){
    neededTransitions =  calcDistance(distanceInCm);
    temp = 1;
    resetRightEncoder();
    resetLeftEncoder();
  }

  unsigned long currentEncoderCount = (rightEncoderCount() + leftEncoderCount()) / 2;
  
  if (currentEncoderCount < neededTransitions){
    rightMotorBackward(mSpeed);
    leftMotorBackward(mSpeed);
  } else if (currentEncoderCount >= neededTransitions){
    robotStop();
    returnValue = 1;
  }
  return returnValue;
  
}

int robotForward(int mSpeed, int distanceInCm){
  static int returnValue = 0;
  static unsigned long neededTransitions = 0;
  static int temp = 0;
  if (returnValue == 1){
    returnValue = 0;
    temp = 0;
  }
  if(temp == 0){
    neededTransitions = calcDistance(distanceInCm);
    temp = 1;
    resetRightEncoder();
    resetLeftEncoder();
  }
  
  unsigned long currentEncoderCount = (rightEncoderCount() + leftEncoderCount()) / 2;
  
  if (currentEncoderCount < neededTransitions){
    rightMotorForward(mSpeed);
    leftMotorForward(mSpeed);
  } else if (currentEncoderCount >= neededTransitions){
    robotStop();
    returnValue = 1;
  }
  return returnValue;
  
}
int calcDistance(int cm){
  //int transitions = 176.2947 * cm;
  int transitions = 180.6204559 * cm; //updated
  return transitions;
}

int calcAngle(int angle ){
    //int transitions = 55.555555555555555555555555 * angle;
    int transitions = 60.25185185 * angle; //updated
    return transitions;
}

void encoderCount()
{ 
  int pinValue = 0;
  static int count = 0;
 
  pinValue = digitalRead(rightPin1);
  
  // encoder state machine here
  enum {HIGHER,LOWER};
  static int state = HIGHER;
  switch (state){
    case HIGHER:
      if ( pinValue == LOW){
        state = LOWER;
        count++;
      }
    break;
    case LOWER:
      if ( pinValue == HIGH){
        state = HIGHER;
        count++;
      }
    break;
  }
  Serial.println(count);
}

void motorsSetup(){
  pinMode(motorRA, OUTPUT);
  pinMode(motorRB, OUTPUT);

  pinMode(motorLA, OUTPUT);
  pinMode(motorLB, OUTPUT);
  
  robotStop();
}
void robotStop(){
  analogWrite(motorRA, 0);
  analogWrite(motorRB, 0);

  analogWrite(motorLA, 0);
  analogWrite(motorLB, 0);
}

void motorControl (int pin1, int pin2, int mSpeed){
  analogWrite(pin1, mSpeed);
  analogWrite(pin2, 0);
}



int forwardTimedDrive(int mSpeed, unsigned long Time){
  static MSTimer timer;
  static int temp = 0;
  static int returnValue = 0;
  //allows function to be run more than once
  if (returnValue == 1){
    returnValue = 0;
  }
  //make sure doesnt reset timer everytime function is run
  if ( temp == 0){
    timer.set(Time);
    temp = 1;
  }

  if (timer.done()== false ){
    leftMotorForward(mSpeed);
    rightMotorForward(mSpeed);
    return returnValue;
  }else if (timer.done() == true){
    returnValue = 1;
    robotStop();
    temp = 0;
  }
  return returnValue;
}

int backwardTimedDrive(int mSpeed, unsigned long Time){
  static MSTimer timer;
  static int temp = 0;
  static int returnValue = 0;
  if ( temp == 0){
    timer.set(Time);
    temp = 1;
  }
  if (returnValue == 1){
    returnValue = 0;
  }

  if (timer.done()== false ){
    leftMotorBackward(mSpeed);
    rightMotorBackward(mSpeed);
    return returnValue;
  }else if (timer.done() == true){
    returnValue = 1;
    robotStop();
    temp = 0;
  }
  return returnValue;
}

int robotLeft(int turnAngle){
  static int returnValue = 0;
  static int temp = 0;
  static unsigned long encoderCountNeeded;

  if (returnValue == 1){
    returnValue = 0;
    temp = 0;
  }

  if (temp == 0){
    encoderCountNeeded =  calcAngle(turnAngle);
    temp = 1;
    resetRightEncoder();
  }

  long currentEncoderCount = rightEncoderCount();
  
  if(currentEncoderCount < encoderCountNeeded){
    rightMotorForward(250);
    return returnValue;
  } else if(currentEncoderCount >= encoderCountNeeded){
    returnValue = 1;
    robotStop();
  }
  return returnValue;
}


int robotRight(int turnAngle){
  static int temp = 0;
  static int returnValue = 0;
  static long encoderCountNeeded;
  if (returnValue == 1){
    returnValue = 0;
    temp = 0;
  }
  
  if (temp == 0){
    encoderCountNeeded = calcAngle(turnAngle);
    temp = 1;
    resetLeftEncoder();
  }

  long currentEncoderCount = leftEncoderCount();

  if(currentEncoderCount < encoderCountNeeded){
    leftMotorForward(250);
    return returnValue;
  } else if(currentEncoderCount >= encoderCountNeeded){
    returnValue = 1;
    robotStop();
  }
  return returnValue;
}

void leftMotorForward(int mSpeed){
  motorControl(motorLB, motorLA, mSpeed);
}

void rightMotorForward(int mSpeed){
  motorControl(motorRB, motorRA, mSpeed);
}

void leftMotorBackward(int mSpeed){
  motorControl(motorLA, motorLB, mSpeed);
}

void rightMotorBackward(int mSpeed){
  motorControl(motorRA, motorRB, mSpeed);
}


