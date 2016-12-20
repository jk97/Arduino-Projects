#include <EncoderLib123_Fall2016.h>
#include <SoftwareSerial.h>
#include "CPE123_Fall16.h"
#include "CPE123_EspAPI_Fall16.h"

enum{NO_COMMAND, LEFT, RIGHT, FORWARD, BACKWARD, STOP};

// Pins - also assumes using Serial1 to talk with ESP board
const int espResetPin = 48;
const int boardLedPin = 42;
const int connectedLedPin = 40;
const int tcpLedPin = 38;

// ESP8266 Leds
Led boardLed(boardLedPin);
Led connectedLed(connectedLedPin);
Led tcpLed(tcpLedPin);

// WiFi and TCP information 
char ssid[] = "robot123";
char wifPassword[] = "RedBot123";
char tcpHost[] = "129.65.158.6";
unsigned int tcpPort = 57111;

// Used to communicate between the Mega and the ESP device
HardwareSerial & espSerial = Serial1;

const int motorLA = 8;
const int motorLB = 9; //positive

const int motorRA = 7; //positive
const int motorRB = 6;

const int rightPin1 = 21;
const int rightPin2 = 20;

const int leftPin1 = 2;
const int leftPin2 = 3;

Button startButton = 5;

void setup() {

  char buf[MAX_PACKET_LEN];
  int wifiStatus = 0;
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  espSerial.begin(9600);
  Serial.println(__FILE__);
  printLibVersion;
  motorsSetup();

  encoderSetup(rightPin1, rightPin2, leftPin1, leftPin2);

  setupEsp(boardLed, connectedLed, tcpLed, espResetPin);
    
  // Reset the wifi board
  ESPHardwareReset(espSerial, espResetPin, buf);
  
  // Output ESP Firmware version 
  reportEspFirmwareVersion(espSerial, buf);
    
  // Connect to WiFi
  // Note - The Esp hardware automatically tries to reconnect when it boots
  setupAndReportWifi(espSerial, ssid, wifPassword, buf);
  
  // Connect to the TCP server
  setupAndReportTcp(espSerial,tcpHost, tcpPort, buf, HALT);
  
  // Put ESP board into serial mode
  Serial.println("Putting ESP into serial mode");
  setupSerialTextMode(espSerial, buf);

}

void loop() {
  // put your main code here, to run repeatedly: 
  stateMachine();
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
  while(!espSerial.available()){}
  char command = espSerial.read();
  if (command == 'L' || command == 'l'){
      angle = espSerial.parseInt();
      if ( constrain(angle, 0, 361) == 0 || constrain(angle, 0, 361)== 361){
        returnValue = NO_COMMAND;
      } else {
        returnValue = LEFT;
      }
  } else if (command == 'R' || command == 'r'){
      angle = espSerial.parseInt();
      if ( constrain(angle,0 , 361) == 0 || constrain(angle,0 , 361)== 361){
        returnValue = NO_COMMAND;
      } else {
        returnValue = RIGHT;
      }
  } else if (command == 'F' || command == 'f'){
      mSpeed = espSerial.parseInt();
      distance = espSerial.parseInt();
      if ( constrain(mSpeed, 149, 256) == 149 || constrain(mSpeed, 149, 256) == 256){
        returnValue = NO_COMMAND;
      } else if ( constrain(distance, 0, 101) == 0 || constrain(distance, 0, 101) == 101){
        returnValue = NO_COMMAND;
      } else {
        returnValue = FORWARD;
      }
  } else if (command == 'B' || command == 'b'){
      mSpeed = espSerial.parseInt();
      distance = espSerial.parseInt();
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

  /*Serial.print("Current Encoder Count: ");
  Serial.println(currentEncoderCount);
  Serial.print("Transitions Needed: ");
  Serial.println(neededTransitions);*/
  
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

  /*Serial.print("RIGHT Count Needed: ");
  Serial.println(encoderCountNeeded);
  Serial.print("RIGHT Current Count: ");
  Serial.println(currentEncoderCount);*/

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

  /*Serial.print("LEFT Count Needed: ");
  Serial.println(encoderCountNeeded);
  Serial.print("LEFT Current Count: ");
  Serial.println(currentEncoderCount);*/

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


