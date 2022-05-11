#include <Wire.h>

TaskHandle_t dataUpdateHandle;

// select the core for OLED threading.
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#define GAIT_RUNNING_CORE    1
#else
#define ARDUINO_RUNNING_CORE 1
#define GAIT_RUNNING_CORE    0
#endif

// Define GPIO.
#define S_SCL   33
#define S_SDA   32
#define RGB_LED 26
#define BUZZER  21
#define WIRE_DEBUG 12

// the middle position of the servos.
extern int MiddlePosition = 300;

// 
extern int CurrentPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                             MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                             MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                             MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};



// <<<<<<<<<<=========Wire Debug Init=========>>>>>>>>>
// [SHOW] DebugMode via wire config.
//         [ . . . o o ]  LED G21 G15 G12 3V3
//         [ . . . . . ]  TX  RX  GND  5V  5V
//            <SWITCH>
// connect this two pins, and the robot go into debug mode.
void wireDebugInit(){
  pinMode(WIRE_DEBUG, INPUT_PULLDOWN);
}



// <<<<<<<=====ICM20948: 0x68=========>>>>>>>>>>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

float ACC_X;
float ACC_Y;
float ACC_Z;

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

void InitICM20948(){
  // if(!myIMU.init()){
  //   Serial.println("ICM20948 does not respond");
  // }
  // else{
  //   Serial.println("ICM20948 is connected");
  // }
  // Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  myIMU.init();
  delay(200);
  myIMU.autoOffsets();
  // Serial.println("Done!"); 

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);
}

void accXYZUpdate(){
  myIMU.readSensor();
  xyzFloat accRaw = myIMU.getAccRawValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  xyzFloat gVal = myIMU.getGValues();

  ACC_X = corrAccRaw.x;
  ACC_Y = corrAccRaw.y;
  ACC_Z = corrAccRaw.z;
}



// <<<<<<<<<========INA219:0x42========>>>>>>>>
#include <INA219_WE.h>
#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

void InitINA219(){
  // if(!ina219.init()){
  //   Serial.println("INA219 not connected!");
  // }
  ina219.init();
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}



// <<<<<<<<<<=========SSD1306: 0x3C===========>>>>>>>>>>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int CURRENT_PAGE = 1;
int PAGE_NUM = 2;
int PAGE_FLASH = 3000;
unsigned long LAST_FLASH;

void InitScreen(){
  // if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
  //   Serial.println(F("SSD1306 allocation failed"));
  // }
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("WAVEGO"));
  display.setTextSize(1);
  display.println(F("ICM20948 calibrating..."));
  display.display();

  LAST_FLASH = millis();
}

void xyzScreenUpdate(float xInput, float yInput, float zInput){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  display.print(int(map(xInput, -17000, 17000, 0, 255)));display.print(F("-"));
  display.print(int(map(yInput, -17000, 17000, 0, 255)));display.print(F("-"));
  display.println(int(map(zInput, -17000, 17000, 0, 255)));

  display.print(F("LoadVoltage:"));display.println(loadVoltage_V);
  display.print(F("Current[mA]:"));display.println(current_mA);
  display.print(F("power[mW]:"));display.println(power_mW);

  display.display();
}

// Updata all data and flash the screen.
void allDataUpdate(){
  if(millis() - LAST_FLASH > PAGE_FLASH && !debugMode){
    CURRENT_PAGE += 1;
    if(CURRENT_PAGE > PAGE_NUM){
      CURRENT_PAGE = 1;
    }
    LAST_FLASH = millis();

    getWifiStatus();
    InaDataUpdate();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    if(CURRENT_PAGE == 1){
      if(WIFI_MODE == 1){
        display.print(F("[AP] "));
        if(!UPPER_TYPE){
          display.println(IP_ADDRESS);
        }
        else if(UPPER_TYPE){
          display.println(UPPER_IP);
        }
      }
      else if(WIFI_MODE == 2){display.print(F("[STA] "));display.println(IP_ADDRESS);}
      else if(WIFI_MODE == 3){display.print(F("[CONNECTING]"));display.println(IP_ADDRESS);}

      display.print(F("[RSSI] "));display.println(WIFI_RSSI);

      display.print(F("[FB]"));display.print(moveFB);display.print(F(" [LR]"));display.print(moveLR);
      display.print(F(" [D]"));display.print(debugMode);display.print(F(" [F]"));display.println(funcMode);

      display.print(F("[BATTERY] "));display.println(loadVoltage_V);
    }
    else if(CURRENT_PAGE == 2){
      display.println(F("  InitPos: G12 - 3V3"));
      display.println(F("  LED G21 G15 G12 3V3"));
      display.println(F("  TX  RX  GND  5V 5V"));
      display.println(F("     [[SWTICH--]]   "));
    }

    display.display();
  }
  else if(millis() < LAST_FLASH && !debugMode){
    LAST_FLASH = millis();
  }
  else if(debugMode){
    display.print(F("0-"));display.print(CurrentPWM[0]);
    display.print(F("1-"));display.print(CurrentPWM[1]);
    display.print(F("2-"));display.print(CurrentPWM[2]);
    display.print(F("3-"));display.println(CurrentPWM[3]);

    display.print(F("4-"));display.print(CurrentPWM[4]);
    display.print(F("5-"));display.print(CurrentPWM[5]);
    display.print(F("6-"));display.print(CurrentPWM[6]);
    display.print(F("7-"));display.println(CurrentPWM[7]);

    display.print(F("8-"));display.print(CurrentPWM[8]);
    display.print(F("9-"));display.print(CurrentPWM[9]);
    display.print(F("10-"));display.print(CurrentPWM[10]);
    display.print(F("11-"));display.println(CurrentPWM[11]);

    display.print(F("12-"));display.print(CurrentPWM[12]);
    display.print(F("13-"));display.print(CurrentPWM[13]);
    display.print(F("14-"));display.print(CurrentPWM[14]);
    display.print(F("15-"));display.println(CurrentPWM[15]);
    delay(600);
  }
}



// <<<<<<<<<<========BUZZER==========>>>>>>>>>>
void InitBuzzer(){
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
}



// <<<<<<<<=========WS2812. (RGB LED)========>>>>>>>>>>
#include <Adafruit_NeoPixel.h>
#define NUMPIXELS   6
#define BRIGHTNESS  200
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, RGB_LED, NEO_GRB + NEO_KHZ800);

void InitRGB(){
  matrix.setBrightness(BRIGHTNESS);
  matrix.begin();
  matrix.show();
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<matrix.numPixels(); i++) {
    matrix.setPixelColor(i, c);
    matrix.show();
    delay(wait);
  }
}

void setSingleLED(uint16_t LEDnum, uint32_t c){
  matrix.setPixelColor(LEDnum, c);
  matrix.show();
}
