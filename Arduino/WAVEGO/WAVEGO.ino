// WIFI settings in app_httpd.cpp

// OLED screen display.
// === PAGE 1 ===
// Row1: [WIFI_MODE]:IP ADDRESS
//       [WIFI_MODE]: 1 as [AP] mode, it will not connect other wifi.
//                    2 as [STA] mode, it will connect to known wifi.
// Row2: [RSSI]
// Row3: [STATUS] [A] [B] [C] [D]
//       [A]: 1 as forward. -1 as backward.
//       [B]: 1 as turn right. -1 as turn left.
//       [C]: 0 as not in the debugMode. Robot can be controled.
//            1 as in the debugMode. You can debug the servos.
//       [D]: 0 as not in any function mode. Robot can be controled to move around.
//            1 as in steady mode. keep balancing.
//            2 as stayLow action.
//            3 as handshake action.
//            4 as jump action.
//            5, 6 and 7 as ActionA, ActionB and ActionC.
//            8 as servos moving to initPos, initPos is the middle angle for servos.
//            9 as servos moving to middlePos, middlePos is the middle angle for program.
// Row4: [BATTERY]
// === PAGE 2 ===
// [SHOW] DebugMode via wire config.
// [ . . . o o ]  LED G21 G15 G12 3V3
// [ . . . . . ]  TX  RX  GND  5V  5V
//    <SWITCH>
extern IPAddress IP_ADDRESS = (0, 0, 0, 0);
extern byte WIFI_MODE = 0; // select WIFI_MODE in app_httpd.cpp
extern void getWifiStatus();
extern int WIFI_RSSI = 0;

// gait type ctrl
// 0: simpleGait(DiagonalGait).
// 1: triangularGait.
extern int GAIT_TYPE = 0;

int CODE_DEBUG = 0;


// ctrl interface.
// refer to OLED screen display for more detail information.
extern int moveFB = 0;
extern int moveLR = 0;
extern int debugMode = 0;
extern int funcMode  = 0;
float gestureUD = 0;
float gestureLR = 0;
float gestureOffSetMax = 15;
float gestureSpeed = 2;
int STAND_STILL = 0;

const char* UPPER_IP = "";
int UPPER_TYPE = 0;
unsigned long LAST_JSON_SEND;
int JSON_SEND_INTERVAL;


// import libraries.
#include "InitConfig.h"
#include "ServoCtrl.h"
#include "PreferencesConfig.h"
#include <ArduinoJson.h>

StaticJsonDocument<200> docReceive;
StaticJsonDocument<100> docSend;
TaskHandle_t threadings;

// placeHolders.
void webServerInit();


// var(variable), val(value).                  
void serialCtrl(){
  if (Serial.available()){
    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(docReceive, Serial);

    if (err == DeserializationError::Ok){
      UPPER_TYPE = 1;
      docReceive["val"].as<int>();

      int val = docReceive["val"];

      if(docReceive["var"] == "funcMode"){
        debugMode = 0;
        gestureUD = 0;
        gestureLR = 0;
        if(val == 1){
          if(funcMode == 1){funcMode = 0;Serial.println("Steady OFF");}
          else if(funcMode == 0){funcMode = 1;Serial.println("Steady ON");}
        }
        else{
          funcMode = val;
          Serial.println(val);
        }
      }

      else if(docReceive["var"] == "move"){
        debugMode = 0;
        funcMode  = 0;
        digitalWrite(BUZZER, HIGH);
        switch(val){
          case 1: moveFB = 1; Serial.println("Forward");break;
          case 2: moveLR =-1; Serial.println("TurnLeft");break;
          case 3: moveFB = 0; Serial.println("FBStop");break;
          case 4: moveLR = 1; Serial.println("TurnRight");break;
          case 5: moveFB =-1; Serial.println("Backward");break;
          case 6: moveLR = 0; Serial.println("LRStop");break;
        }
      }

      else if(docReceive["var"] == "ges"){
        debugMode = 0;
        funcMode  = 0;
        switch(val){
          case 1: gestureUD += gestureSpeed;if(gestureUD > gestureOffSetMax){gestureUD = gestureOffSetMax;}break;
          case 2: gestureUD -= gestureSpeed;if(gestureUD <-gestureOffSetMax){gestureUD =-gestureOffSetMax;}break;
          case 3: break;
          case 4: gestureLR -= gestureSpeed;if(gestureLR <-gestureOffSetMax){gestureLR =-gestureOffSetMax;}break;
          case 5: gestureLR += gestureSpeed;if(gestureLR > gestureOffSetMax){gestureLR = gestureOffSetMax;}break;
          case 6: break;
        }
        pitchYawRollHeightCtrl(gestureUD, gestureLR, 0, 0);
      }

      else if(docReceive["var"] == "light"){
        switch(val){
          case 0: setSingleLED(0,matrix.Color(0, 0, 0));setSingleLED(1,matrix.Color(0, 0, 0));break;
          case 1: setSingleLED(0,matrix.Color(0, 32, 255));setSingleLED(1,matrix.Color(0, 32, 255));break;
          case 2: setSingleLED(0,matrix.Color(255, 32, 0));setSingleLED(1,matrix.Color(255, 32, 0));break;
          case 3: setSingleLED(0,matrix.Color(32, 255, 0));setSingleLED(1,matrix.Color(32, 255, 0));break;
          case 4: setSingleLED(0,matrix.Color(255, 255, 0));setSingleLED(1,matrix.Color(255, 255, 0));break;
          case 5: setSingleLED(0,matrix.Color(0, 255, 255));setSingleLED(1,matrix.Color(0, 255, 255));break;
          case 6: setSingleLED(0,matrix.Color(255, 0, 255));setSingleLED(1,matrix.Color(255, 0, 255));break;
          case 7: setSingleLED(0,matrix.Color(255, 64, 32));setSingleLED(1,matrix.Color(32, 64, 255));break;
        }
      }

      else if(docReceive["var"] == "buzzer"){
        switch(val){
          case 0: digitalWrite(BUZZER, HIGH);break;
          case 1: digitalWrite(BUZZER, LOW);break;
        }
      }
    }


      // else if(docReceive['var'] == "ip"){
      //     UPPER_IP = docReceive['ip'];
      // }
     

    else {
      while (Serial.available() > 0)
        Serial.read();
    }
  }
}


void jsonSend(){
  if(millis() - LAST_JSON_SEND > JSON_SEND_INTERVAL || millis() < LAST_JSON_SEND){
    docSend["vol"] = loadVoltage_V;
    serializeJson(docSend, Serial);
    LAST_JSON_SEND = millis();
  }
}


void robotThreadings(void *pvParameter){
  delay(3000);
  while(1){
    serialCtrl();
    delay(25);
  }
}


void threadingsInit(){
  xTaskCreate(&robotThreadings, "RobotThreadings", 4000, NULL, 5, &threadings);
}


void setup() {
  Wire.begin(S_SDA, S_SCL);
  Serial.begin(115200);

  // WIRE DEBUG INIT.
  wireDebugInit();
  
  // INA219 INIT.
  InitINA219();

  // BUZZER INIT.
  InitBuzzer();

  // RGB INIT
  InitRGB();

  // PCA9685 INIT.
  ServoSetup();

  // SSD1306 INIT.
  InitScreen();

  // EEPROM INIT.
  preferencesSetup();

  // Standup for ICM20948 calibrating.
  delay(100);
  setSingleLED(0,matrix.Color(0, 128, 255));
  setSingleLED(1,matrix.Color(0, 128, 255));
  standMassCenter(0, 0);GoalPosAll();delay(1000);
  setSingleLED(0,matrix.Color(255, 128, 0));
  setSingleLED(1,matrix.Color(255, 128, 0));
  delay(500);

  // ICM20948 INIT.
  InitICM20948();

  // WEBCTRL INIT. WIFI settings included.
  webServerInit();

  // RGB LEDs on.
  delay(500);
  setSingleLED(0,matrix.Color(0, 32, 255));
  setSingleLED(1,matrix.Color(255, 32, 0));

  // update data on screen.
  allDataUpdate();

  // threadings start.
  threadingsInit();
}


// main loop.
void loop() {
  robotCtrl();
  allDataUpdate();
  wireDebugDetect();
}


// <<<<<<<<<<=== Devices on Board ===>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// ICM20948 init. --- 9-axis sensor for motion tracking.
// InitICM20948();

// read and update the pitch, raw and roll data from ICM20948.
// accXYZUpdate();


// --- --- ---   --- --- ---   --- --- ---
// INA219 init. --- DC current/voltage sensor.
// InitINA219();

// read and update the voltage and current data from INA219.
// InaDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// RGB INIT. --- the 2 RGB LEDs in front of the robot. LED_NUM = 0, 1.
// InitRGB();

// control RGB LED. 0 <= R, G, B <= 255.
// setSingleLED(LED_NUM, matrix.Color(R, G, B));


// --- --- ---   --- --- ---   --- --- ---
// SSD1306 INIT. --- OLED Screen
// InitScreen();

// show the newest data on the OLED screen.
// for more information you can refer to <OLED screen display>.
// screenDataUpdate();


// --- --- ---   --- --- ---   --- --- ---
// BUZZER INIT. --- the device that make a sound.
// InitBuzzer();

// BUZZER on.
// digitalWrite(BUZZER, HIGH);

// BUZZER off.
// digitalWrite(BUZZER, LOW);


// --- --- ---   --- --- ---   --- --- ---
// PCA9685 INIT.
// ServoSetup();

// all servos move to the middle position of the servos.
// initPosAll();


// <<<<<<<<<<<<=== Servos/Legs/Motion Ctrl ===>>>>>>>>>>>>>>>

// --- --- ---   --- --- ---   --- --- ---
// all servos move to the middle position of the program. 
// the position that you have to debug it to make it moves to.
// middlePosAll();

// control a single servo by updating the angle data in GoalPWM[].
// once it is called, call GoalPosALL() to move all of the servos.
// goalPWMSet(servoNum, angleInput);

// all servos move to goal position(GoalPWM[]).
// GoalPosAll();

// Ctrl a single leg of WAVEGO, once it is called, call GoalPosALL() to move all of the servos.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O                 O ------ [BODY]      I(1)  ^  III(3)
//    /         .              |          |               |
//   /    |        O           |          |               |
//  O     y     .              |          |         II(2) ^  IV(4)
//   \.   |  .                 |          |
//    \.  .                    |          |
//     O  |                    |          |
//  .                          |          |
//   \.   |                    |          |
//    \-x-X                    X----z-----O
// ---------------------------------------------------------------
// x, y, z > 0
// singleLegCtrl(LEG_NUM, X, Y, Z);


// --- --- ---   --- --- ---   --- --- ---
// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// simpleGait(GlobalInput, directionAngle);

// a triangular gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// once it is called, call GoalPosALL() to move all of the servos.
// triangularGait(GlobalInput, directionAngle);


// --- --- ---   --- --- ---   --- --- ---
// Stand and adjust mass center.
//     ^
//     a
//     |
// <-b-M
// a,b > 0
// standMassCenter(aInput, bInput);


// --- --- ---   --- --- ---   --- --- ---
// ctrl pitch yaw and roll.
// pitchInput (-, +), if > 0, look up.
// yawInput   (-, +), if > 0, look right.
// rollInput  (-, +), if > 0, lean right.
// 75 < input < 115
// pitchYawRoll(pitchInput, yawInput, rollInput);


// --- --- ---   --- --- ---   --- --- ---
// balancing function.
// once it is called, call GoalPosALL() to move all of the servos.
// balancing();


// --- --- ---   --- --- ---   --- --- ---
// the default function to control robot.
// robotCtrl();


// <<<<<<<<<<<<<<<=== Save Data Permanently ===>>>>>>>>>>>>>>>>>>

// EEPROM INIT.
// preferencesSetup();

// save the current position of the servoNum in EEPROM.
// servoConfigSave(servoNum);

// read the saved middle position data of the servos from EEPROM.
// middleUpdate();