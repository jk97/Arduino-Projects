#include <EncoderLib123_Fall2016.h>
#include <SoftwareSerial.h>
#include "CPE123_Fall16.h"
#include "CPE123_EspAPI_Fall16.h"

////////////////////////////////////////////////////////////////////////////////
/*                     Global Variables(mainly for wifi)                      */
////////////////////////////////////////////////////////////////////////////////

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
unsigned int tcpPort = 61998;

// Used to communicate between the Mega and the ESP device
HardwareSerial & espSerial = Serial1;

//Led and Button components on the watch
Led watchLed(5);
Button watchButton(7);

////////////////////////////////////////////////////////////////////////////////
/*                              Setup and Loop                                */
////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:

  char buf[MAX_PACKET_LEN];
  int wifiStatus = 0;
  
  pinMode(A14, OUTPUT);//vibrator
  pinMode(A13, OUTPUT); //buzzer
  
  Serial.begin(9600);
  espSerial.begin(9600);
  Serial.println(__FILE__);
  printLibVersion;

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

//////////////////////////////////////////////////////////////////////////////////
/*                            Main State Machine                                */
//////////////////////////////////////////////////////////////////////////////////

void stateMachine(){
  enum{WATCH_ON, WATCH_OFF};
  static int state = WATCH_OFF;
  static int doorbellRinged = 0;
  static MSTimer totalTime;
  switch(state){
    case WATCH_OFF:
      if (espSerial.available()){
          doorbellRinged = espSerial.parseInt();
        }
      if(doorbellRinged == 1){
        state = WATCH_ON;
        totalTime.set(60000); //max time on is 60 seconds unless pushed
      }
    break;
    case WATCH_ON:
      if (totalTime.done() == false){
        ledFunction();
        viberate();
        buzzer();
        if(watchButton.wasPushed()){ //push button turns off watch
          state = WATCH_OFF;
          turnoff();
          espSerial.write('2');
          doorbellRinged = 0;
        }
      }else{ //total timer done (60 sec && button not pressed) -> turn off watch
         turnoff();
         doorbellRinged = 0; // off
         state = WATCH_OFF;
      }
      
    break;
    default:
    break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
/*                           Smaller Functions                                      */
//////////////////////////////////////////////////////////////////////////////////////

void ledFunction(){
  static MSTimer timer;
  static int flag = 0;
  
  //LED alternate on/off every 1 sec
  if(timer.done() == true){
    if (flag == 0){
      watchLed.on();
      flag = 1;
      timer.set(1000);
    } else if (flag == 1){
      watchLed.off();
      flag = 0;
      timer.set(1000);
    }
  }
}

void viberate(){
  //Continuous vibration
  analogWrite(A14, 200);
}

void buzzer(){
  static MSTimer timer;
  static int flag = 0;
  
  //Buzzer creates tone between notes C and E every one sec
  if(timer.done() == true){
   if(flag == 0){
     tone(A13, 837);
     flag = 1;
     timer.set(1000);
   } else if (flag == 1){
      tone(A13, 1054);
      flag = 0;
      timer.set(1000);
   }
  }
}

void turnoff(){ //turns off all functions of the watch
  watchLed.off();
  noTone(A13);
  analogWrite(A14,0);
}

