#include <Adafruit_PWMServoDriver.h>
#include <math.h>

TwoWire IIC = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, IIC);

#define SERVOMIN  263 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  463 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_RANGE 90
#define M_PI 3.1415926

// 0  Forward 6
// 1  -1|^|3- 7
//<2> --|^|-- <8>
//      |||
//<3> --|^|-- <9>
// 4  -2|^|4- 10
// 5  --|^|-- 11

//   [S-W]---L-W---O
//                 |
//                 |
//                 |
//                 |
//                 |
//               Ground
double Linkage_W = 19.15;   // The distance between wiggle servo and the plane of the leg linkages.
double wiggleError = 0;

//     <<<[S-A][S-B]<<<
//         /     |
//       L-A    L-A
//       /       |
//      O        O
//      |       /
//     L-B    L-C
//      |     /
//      |    /
//      |   /
//      |  /
//      | /
//      O
//     /  
//   L-D
//   /
//  <---90°
//      .L-E
//           .
// ------------------------
double Linkage_S = 12.2;    // The distance between two servos.
double Linkage_A = 40.0;    // The linkage that connected with the servo.
double Linkage_B = 40.0;    // The linkage that limit the direction.
double Linkage_C = 39.8153; // The upper part of the leg.
double Linkage_D = 31.7750; // The lower part of the leg.
double Linkage_E = 30.8076; // The foot.

double WALK_HEIGHT_MAX  = 110;
double WALK_HEIGHT_MIN  = 75;
double WALK_HEIGHT      = 95;
double WALK_LIFT        = 9; // WALK_HEIGHT + WALK_LIFT <= WALK_HEIGHT_MAX.
double WALK_RANGE       = 40;
double WALK_ACC         = 5;
double WALK_EXTENDED_X  = 16;
double WALK_EXTENDED_Z  = 25;
double WALK_SIDE_MAX    = 30;
double WALK_MASS_ADJUST = 21;
double STAND_HEIGHT     = 95;

uint8_t WALK_STATUS     = 0;
float WALK_CYCLE_GLOBAL = 0;    // 0-1.
float WALK_LIFT_PROP    = 0.25; // WALK_LIFT_TIME<1.

float BALANCE_PITCHU_BUFFER;
float BALANCE_ROLL_BUFFER;
float BALANCE_PITCHU_BASE = 0;
float BALANCE_ROLL_BASE   = 0;
float BALANCE_P = 0.00018;

float GLOBAL_STEP  = 0;
int   STEP_DELAY   = 4;
// int   STEP_DELAY   = 2;
float STEP_ITERATE = 0.04;
// float STEP_ITERATE = 0.01;

int SERVO_MOVE_EVERY = 0;
int MAX_TEST = 125;

#define LEG_A_FORE 8
#define LEG_A_BACK 9
#define LEG_A_WAVE 10

#define LEG_B_WAVE 13
#define LEG_B_FORE 14
#define LEG_B_BACK 15

#define LEG_C_FORE 7
#define LEG_C_BACK 6
#define LEG_C_WAVE 5

#define LEG_D_WAVE 2
#define LEG_D_FORE 1
#define LEG_D_BACK 0


extern int ServoMiddlePWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                                 MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int legPosBuffer[12] = { WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                         WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z,
                        -WALK_EXTENDED_X,  STAND_HEIGHT, WALK_EXTENDED_Z};

int GoalPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int LastPWM[16] = {MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition,
                   MiddlePosition, MiddlePosition, MiddlePosition, MiddlePosition};

int ServoDirection[16] = {-1,  1,  1,  1,
                           1, -1, -1,  1,
                          -1,  1,  1,  1,
                           1, -1, -1,  1};

double linkageBuffer[32] = {0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0};


// pre render.
float LAxLA = Linkage_A*Linkage_A;
float LBxLB = Linkage_B*Linkage_B;
float LWxLW = Linkage_W*Linkage_W;
float LExLE = Linkage_E*Linkage_E;
float LAxLA_LBxLB = LAxLA - LBxLB;
float LBxLB_LAxLA = LBxLB - LAxLA;
float L_CD = (Linkage_C+Linkage_D)*(Linkage_C+Linkage_D);
float LAx2  = 2 * Linkage_A;
float LBx2  = 2 * Linkage_B;
float E_PI  = 180 / M_PI;
float LSs2  = Linkage_S/2;
float aLCDE = atan((Linkage_C + Linkage_D)/Linkage_E);
float sLEDC = sqrt(Linkage_E*Linkage_E + (Linkage_D+Linkage_C)*(Linkage_D+Linkage_C));
float O_WLP = 1 - WALK_LIFT_PROP;
float WALK_ACCx2 = WALK_ACC*2;
float WALK_H_L = WALK_HEIGHT - WALK_LIFT;


void ServoSetup(){
  IIC.begin(S_SDA, S_SCL, 26000000);
  pwm.begin();
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Wire.setClock(100000);
  // Serial.println("16 channel PWM Setup!");
  delay(10);
}


extern void initPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, MiddlePosition);
    CurrentPWM[i] = MiddlePosition;
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


extern void middlePosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, ServoMiddlePWM[i]);
    CurrentPWM[i] = ServoMiddlePWM[i];
    Serial.print(CurrentPWM[i]);
    Serial.print(" ");
    delay(SERVO_MOVE_EVERY);
  }
  Serial.println(" ");
}


extern void servoDebug(byte servoID, int offset){
  CurrentPWM[servoID] += offset;
  pwm.setPWM(servoID, 0, CurrentPWM[servoID]);  
}


void GoalPosAll(){
  for(int i = 0; i < 16; i++){
    pwm.setPWM(i, 0, GoalPWM[i]);
  }
}


void goalPWMSet(uint8_t servoNum, double angleInput){
  int pwmGet;
  if (angleInput == 0){pwmGet = 0;}
  else{pwmGet = round((SERVOMAX - SERVOMIN) * angleInput / SERVO_RANGE);}
  pwmGet = pwmGet * ServoDirection[servoNum] + ServoMiddlePWM[servoNum];
  GoalPWM[servoNum] = pwmGet;
}


// Simple Linkage IK:
// input the position of the end and return angle.
//   O----O
//  /
// O
// ---------------------------------------------------
// |       /beta           /delta                    |
//        O----LB---------X------                    |
// |     /       omega.   |       \LB                |
//      LA        .                < ----------------|
// |alpha     .          bIn         \LB -EP  <delta |
//    /psi.                           \LB -EP        |
// | /.   lambda          |                          |
// O- - - - - aIn - - - - X -                        |
// ---------------------------------------------------
// alpha, beta > 0 ; delta <= 0 ; aIn, bIn > 0
// simpleLinkageIK(Linkage_A, Linkage_B, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-Linkage_S/2), betaOut, betaB, betaC);
void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
  double psi;
  double alpha;
  double omega;
  double beta;
  double L2C;
  double LC;
  double lambda;
  double delta;
  if(bIn == 0){
    psi   = acos((LAxLA_LBxLB + aIn * aIn)/(LAx2 * aIn)) * E_PI;
    alpha = 90 - psi;
    omega = acos((aIn * aIn + LBxLB_LAxLA)/(LBx2 * aIn)) * E_PI;
    beta  = psi + omega;
  }
  else{
    L2C = aIn * aIn + bIn * bIn;
    LC  = sqrt(L2C);
    lambda = atan(bIn/aIn) * 180 / M_PI;
    psi    = acos((LAxLA_LBxLB + L2C)/(2 * LA * LC)) * E_PI;
    alpha  = 90 - lambda - psi;
    omega  = acos((LBxLB_LAxLA + L2C)/(2 * LC * LB)) * E_PI;
    beta   = psi + omega;
  }
  delta = 90 - alpha - beta;
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputBeta]  = beta;
  linkageBuffer[outputDelta] = delta;
}

// ||| ||| ||| ||| ||| |||
// void simpleLinkageIK(double LA, double LB, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputBeta, uint8_t outputDelta){
//   double psi;
//   double alpha;
//   double omega;
//   double beta;
//   double L2C;
//   double LC;
//   double lambda;
//   double delta;
//   if(bIn == 0){
//     psi   = acos((LA * LA + aIn * aIn - LB * LB)/(2 * LA * aIn)) * 180 / M_PI;
//     alpha = 90 - psi;
//     omega = acos((aIn * aIn + LB * LB - LA * LA)/(2 * aIn * LB)) * 180 / M_PI;
//     beta  = psi + omega;
//   }
//   else{
//     L2C = aIn * aIn + bIn * bIn;
//     LC  = sqrt(L2C);
//     lambda = atan(bIn/aIn) * 180 / M_PI;
//     psi    = acos((LA * LA + L2C - LB * LB)/(2 * LA * LC)) * 180 / M_PI;
//     alpha = 90 - lambda - psi;
//     omega = acos((LB * LB + L2C - LA * LA)/(2 * LC * LB)) * 180 / M_PI;
//     beta  = psi + omega;
//   }
//   delta = 90 - alpha - beta;
//   linkageBuffer[outputAlpha] = alpha;
//   linkageBuffer[outputBeta]  = beta;
//   linkageBuffer[outputDelta] = delta;
// }


// Wiggle Plane IK:
// input the position of the end and return angle.
// O-----X
//       |
//       |
//       O
// ------------------------------
//     X                        |
//    /    .                    
//  LA         .                |
//  /alpha         .LB         
// O- - - - - - - - - -.- - - -X|
//                         .  bIn
// ------------aIn-------------X|
// ------------------------------
// alpha, aIn, bIn > 0
// wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
  double LB;
  double L2C;
  double LC;
  double alpha;
  double beta;
  double lambda;
  double psi;
  if(bIn > 0){
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    psi = acos(LA/LC) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = psi + lambda - 90;
  }
  else if(bIn == 0){
    alpha = asin(LA/aIn) * E_PI;
    L2C = aIn * aIn + bIn * bIn;
    LB = sqrt(L2C);
  }
  else if(bIn < 0){
    bIn = -bIn;
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan(aIn/bIn) * E_PI;
    psi = acos(LA/LC) * E_PI;
    LB = sqrt(L2C - LWxLW);
    alpha = 90 - lambda + psi;
  }
  linkageBuffer[outputAlpha] = alpha;
  linkageBuffer[outputLen]  = LB - wiggleError;
}

// ||| ||| ||| ||| ||| |||
// void wigglePlaneIK(double LA, double aIn, double bIn, uint8_t outputAlpha, uint8_t outputLen){
//   double LB;
//   double L2C;
//   double LC;
//   double alpha;
//   double beta;
//   double lambda;
//   double psi;
//   if(bIn > 0){
//     L2C = aIn * aIn + bIn * bIn;
//     LC = sqrt(L2C);
//     lambda = atan(aIn/bIn) * 180 / M_PI;
//     psi = acos(LA/LC) * 180 / M_PI;
//     LB = sqrt(L2C - LA * LA);
//     alpha = psi + lambda - 90;
//   }
//   else if(bIn == 0){
//     alpha = asin(LA/aIn) * 180 / M_PI;
//     L2C = aIn * aIn + bIn * bIn;
//     LB = sqrt(L2C);
//   }
//   else if(bIn < 0){
//     bIn = -bIn;
//     L2C = aIn * aIn + bIn * bIn;
//     LC = sqrt(L2C);
//     lambda = atan(aIn/bIn) * 180 / M_PI;
//     psi = acos(LA/LC) * 180 / M_PI;
//     LB = sqrt(L2C - LA * LA);
//     alpha = 90 - lambda + psi;
//   }
//   linkageBuffer[outputAlpha] = alpha;
//   linkageBuffer[outputLen]  = LB - wiggleError;
// }


// Ctrl single leg plane IK:
// input the args of the leg, return angle and position.
//  SE--------LS(O)-----SE
//             |        |  \.
//                      |   LA
//             |        |beta\.
//       \.                   \.
//        \.   |               O
//         \.              .
//          LB |       LC
//           \.    .
//             O
//         LD
//     .       |
// F
//  \.         |
//   LE       yIn
//    \.       |
//     \.--xIn-X  
// beta > 0 ; xIn, yIn > 0
//   singleLegPlaneIK(Linkage_S, Linkage_A, Linkage_C, Linkage_D, Linkage_E, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
  double bufferS = sqrt((xIn + LSs2)*(xIn + LSs2) + yIn*yIn);
  double lambda = acos(((xIn + LSs2)*(xIn + LSs2) + yIn*yIn + LAxLA - L_CD - LExLE)/(2*bufferS*LA));
  double delta = atan((xIn + LSs2)/yIn);
  double beta = lambda - delta;
  double betaAngle = beta * E_PI;

  double theta = aLCDE;
  double omega = asin((yIn - cos(beta)*LA)/sLEDC);
  double nu = M_PI - theta - omega;
  double dFX = cos(nu)*LE;
  double dFY = sin(nu)*LE;

  double mu = M_PI/2 - nu;
  double dEX = cos(mu)*LD;
  double dEY = sin(mu)*LD;

  double positionX = xIn + dFX - dEX;
  double positionY = yIn - dFY - dEY;
  
  linkageBuffer[outputBeta] = betaAngle;
  linkageBuffer[outputX]  = positionX;
  linkageBuffer[outputY]  = positionY;
}

// ||| ||| ||| ||| ||| |||
// void singleLegPlaneIK(double LS, double LA, double LC, double LD, double LE, double xIn, double yIn, uint8_t outputBeta, uint8_t outputX, uint8_t outputY){
//   double bufferS = sqrt((xIn + LS/2)*(xIn + LS/2) + yIn*yIn);
//   double lambda = acos(((xIn + LS/2)*(xIn + LS/2) + yIn*yIn + LA*LA - (LC+LD)*(LC+LD) - LE*LE)/(2*bufferS*LA));
//   double delta = atan((xIn + LS/2)/yIn);
//   double beta = lambda - delta;
//   double betaAngle = beta * 180 / M_PI;

//   double theta = atan((LC+LD)/LE);
//   double omega = asin((yIn - cos(beta)*LA)/sqrt(LE*LE + (LD+LC)*(LD+LC)));
//   double nu = M_PI - theta - omega;
//   double dFX = cos(nu)*LE;
//   double dFY = sin(nu)*LE;

//   double mu = M_PI/2 - nu;
//   double dEX = cos(mu)*LD;
//   double dEY = sin(mu)*LD;

//   double positionX = xIn + dFX - dEX;
//   double positionY = yIn - dFY - dEY;
  
//   linkageBuffer[outputBeta] = betaAngle;
//   linkageBuffer[outputX]  = positionX;
//   linkageBuffer[outputY]  = positionY;
// }


// Ctrl a single leg of WAVEGO, it moves in a plane.
// input (x,y) position and return angle alpha and angle beta.
//     O  X  O
//    /         .
//   /    |        O
//  O     y     .
//   \.   |  .
//    \.  .
//     O  |
//  .
//   \.   |
//    \-x-X
// ------------------
// x, y > 0
void singleLegCtrl(uint8_t LegNum, double xPos, double yPos, double zPos){
  uint8_t alphaOut;
  uint8_t xPosBuffer;
  uint8_t yPosBuffer;
  uint8_t betaOut;
  uint8_t betaB;
  uint8_t betaC;
  uint8_t NumF;
  uint8_t NumB;
  uint8_t wiggleAlpha;
  uint8_t wiggleLen;
  uint8_t NumW;
  if(LegNum == 1){
    NumF = LEG_A_FORE;
    NumB = LEG_A_BACK;
    NumW = LEG_A_WAVE;
    alphaOut   = 0;
    xPosBuffer = 1;
    yPosBuffer = 2;
    betaOut = 3;
    betaB   = 4;
    betaC   = 5;
    wiggleAlpha = 6;
    wiggleLen   = 7;
  }
  else if(LegNum == 2){
    NumF = LEG_B_FORE;
    NumB = LEG_B_BACK;
    NumW = LEG_B_WAVE;
    alphaOut   = 8;
    xPosBuffer = 9;
    yPosBuffer = 10;
    betaOut = 11;
    betaB   = 12;
    betaC   = 13;
    wiggleAlpha = 14;
    wiggleLen   = 15;
  }
  else if(LegNum == 3){
    NumF = LEG_C_FORE;
    NumB = LEG_C_BACK;
    NumW = LEG_C_WAVE;
    alphaOut   = 16;
    xPosBuffer = 17;
    yPosBuffer = 18;
    betaOut = 19;
    betaB   = 20;
    betaC   = 21;
    wiggleAlpha = 22;
    wiggleLen   = 23;
  }
  else if(LegNum == 4){
    NumF = LEG_D_FORE;
    NumB = LEG_D_BACK;
    NumW = LEG_D_WAVE;
    alphaOut   = 24;
    xPosBuffer = 25;
    yPosBuffer = 26;
    betaOut = 27;
    betaB   = 28;
    betaC   = 29;
    wiggleAlpha = 30;
    wiggleLen   = 31;
  }

  wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen);
  singleLegPlaneIK(Linkage_S, Linkage_A, Linkage_C, Linkage_D, Linkage_E, xPos, linkageBuffer[wiggleLen], alphaOut, xPosBuffer, yPosBuffer);
  simpleLinkageIK(Linkage_A, Linkage_B, linkageBuffer[yPosBuffer], (linkageBuffer[xPosBuffer]-Linkage_S/2), betaOut, betaB, betaC);

  goalPWMSet(NumW, linkageBuffer[wiggleAlpha]);
  goalPWMSet(NumF, (90 - linkageBuffer[betaOut]));
  goalPWMSet(NumB, linkageBuffer[alphaOut]);
}


void standUp(double cmdInput){
  singleLegCtrl(1, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(2, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
  singleLegCtrl(4, -WALK_EXTENDED_X, cmdInput, WALK_EXTENDED_Z);
}


// Ctrl the gait of a single leg with the variable cycleInput change between 0-1.
// when the directionInput > 0, the front point in the gait cycle move away from the middle line of the robot.
// use the extendedX and extendedZ to adjust the position of the middle point in a wiggle cycle.
// statusInput used to reduce the WALK_RANGE.
void singleGaitCtrl(uint8_t LegNum, uint8_t statusInput, float cycleInput, float directionInput, double extendedX, double extendedZ){
  double rDist;
  double xGait;
  double yGait;
  double zGait;
  double rDiection = directionInput * M_PI / 180;

  if(cycleInput < (1 - WALK_LIFT_PROP)){
    if(cycleInput <= (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
      yGait = (WALK_HEIGHT - WALK_LIFT) + cycleInput/(1-WALK_LIFT_PROP-((WALK_ACC+WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))*WALK_LIFT;
    }
    else if(cycleInput > (WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput <= ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
      yGait = WALK_HEIGHT;
    }
    else if(cycleInput > ((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP) && cycleInput < ((WALK_ACC*2 + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)){
      yGait = WALK_HEIGHT - ((cycleInput-((WALK_ACC + WALK_RANGE*statusInput)/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP))/((WALK_ACC/(WALK_ACC*2 + WALK_RANGE*statusInput))*(1-WALK_LIFT_PROP)))*WALK_LIFT;
    }

    rDist = (WALK_RANGE*statusInput/2 + WALK_ACC) - (cycleInput/(1 - WALK_LIFT_PROP))*(WALK_RANGE*statusInput + WALK_ACC*2);
  }
  else if(cycleInput >= (1 - WALK_LIFT_PROP)){
    yGait = WALK_HEIGHT - WALK_LIFT;
    rDist = - (WALK_RANGE*statusInput/2 + WALK_ACC) + ((cycleInput-(1-WALK_LIFT_PROP))/WALK_LIFT_PROP)*(WALK_RANGE*statusInput + WALK_ACC*2);
  }

  xGait = cos(rDiection) * rDist;
  zGait = sin(rDiection) * rDist;
  singleLegCtrl(LegNum, (xGait + extendedX), yGait, (zGait + extendedZ));
}


// a simple gait to ctrl the robot.
// GlobalInput changes between 0-1.
// use directionAngle to ctrl the direction.
// turnCmd-> -1:left 1:right
void simpleGait(float GlobalInput, float directionAngle, int turnCmd){
  float Group_A;
  float Group_B;

  Group_A = GlobalInput;
  Group_B = GlobalInput+0.5;
  if(Group_B>1){Group_B--;}

  if(!turnCmd){
    singleGaitCtrl(1, 1, Group_A, directionAngle,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1, Group_A, -directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1, Group_B, directionAngle, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1, Group_B, -directionAngle, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, Group_A, 90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, -90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, Group_A, -90,  WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(4, 1.5, Group_A, -90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);

    singleGaitCtrl(2, 1.5, Group_B, 90, -WALK_EXTENDED_X, WALK_EXTENDED_Z);
    singleGaitCtrl(3, 1.5, Group_B, 90, WALK_EXTENDED_X, WALK_EXTENDED_Z);
  }
}


// Triangular gait generater.
void triangularGait(float GlobalInput, float directionAngle, int turnCmd){
  float StepA;
  float StepB;
  float StepC;
  float StepD;

  float aInput = 0;
  float bInput = 0;
  float adProp;

  StepB = GlobalInput;
  StepC = GlobalInput + 0.25;
  StepD = GlobalInput + 0.5;
  StepA = GlobalInput + 0.75;

  if(StepA>1){StepA--;}
  if(StepB>1){StepB--;}
  if(StepC>1){StepC--;}
  if(StepD>1){StepD--;}

  if(GlobalInput <= 0.25){
    adProp = GlobalInput;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.25 && GlobalInput <= 0.5){
    adProp = GlobalInput-0.25;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.5 && GlobalInput <= 0.75){
    adProp = GlobalInput-0.5;
    aInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST;
  }
  else if(GlobalInput > 0.75 && GlobalInput <= 1){
    adProp = GlobalInput-0.75;
    aInput = -WALK_MASS_ADJUST + (adProp/0.125)*WALK_MASS_ADJUST;
    bInput =  WALK_MASS_ADJUST - (adProp/0.125)*WALK_MASS_ADJUST;
  }

  if(!turnCmd){
    singleGaitCtrl(1, 1, StepA,  directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1, StepD, -directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1, StepB,  directionAngle, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1, StepC, -directionAngle,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == -1){
    singleGaitCtrl(1, 1.5, StepA,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
  else if(turnCmd == 1){
    singleGaitCtrl(1, 1.5, StepA, -90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(4, 1.5, StepD, -90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);

    singleGaitCtrl(2, 1.5, StepB,  90, -WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z - bInput);
    singleGaitCtrl(3, 1.5, StepC,  90,  WALK_EXTENDED_X - aInput, WALK_EXTENDED_Z + bInput);
  }
}


// Gait Select
void gaitTypeCtrl(float GlobalStepInput, float directionCmd, int turnCmd){
  if(GAIT_TYPE == 0){
    simpleGait(GlobalStepInput, directionCmd, turnCmd);
  }
  else if(GAIT_TYPE == 1){
    triangularGait(GlobalStepInput, directionCmd, turnCmd);
  }
}


// Stand and adjust mass center.
//    |
//    a
//    |
// -b-M  a,b > 0
void standMassCenter(float aInput, float bInput){
  singleLegCtrl(1, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));
  singleLegCtrl(2, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z - bInput));

  singleLegCtrl(3, ( WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
  singleLegCtrl(4, (-WALK_EXTENDED_X - aInput), STAND_HEIGHT, ( WALK_EXTENDED_Z + bInput));
}


// ctrl pitch yaw and roll.
// pitchInput (-, +), if > 0, look up.
// yawInput   (-, +), if > 0, look right.
// rollInput  (-, +), if > 0, lean right.
void pitchYawRoll(float pitchInput, float yawInput, float rollInput){
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
  
  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}


void pitchYawRollHeightCtrl(float pitchInput, float yawInput, float rollInput, float heightInput){
  legPosBuffer[1]  = STAND_HEIGHT + pitchInput + rollInput + heightInput;
  legPosBuffer[4]  = STAND_HEIGHT - pitchInput + rollInput + heightInput;
  legPosBuffer[7]  = STAND_HEIGHT + pitchInput - rollInput + heightInput;
  legPosBuffer[10] = STAND_HEIGHT - pitchInput - rollInput + heightInput;

  if(legPosBuffer[1] > WALK_HEIGHT_MAX){legPosBuffer[1] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[4] > WALK_HEIGHT_MAX){legPosBuffer[4] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[7] > WALK_HEIGHT_MAX){legPosBuffer[7] = WALK_HEIGHT_MAX;}
  if(legPosBuffer[10] > WALK_HEIGHT_MAX){legPosBuffer[10] = WALK_HEIGHT_MAX;}
  
  if(legPosBuffer[1] < WALK_HEIGHT_MIN){legPosBuffer[1] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[4] < WALK_HEIGHT_MIN){legPosBuffer[4] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[7] < WALK_HEIGHT_MIN){legPosBuffer[7] = WALK_HEIGHT_MIN;}
  if(legPosBuffer[10] < WALK_HEIGHT_MIN){legPosBuffer[10] = WALK_HEIGHT_MIN;}

  legPosBuffer[2]  = WALK_EXTENDED_Z + yawInput - rollInput;
  legPosBuffer[5]  = WALK_EXTENDED_Z - yawInput - rollInput;
  legPosBuffer[8]  = WALK_EXTENDED_Z - yawInput + rollInput;
  legPosBuffer[11] = WALK_EXTENDED_Z + yawInput + rollInput;

  if(legPosBuffer[2] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[5] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[8] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}
  if(legPosBuffer[11] > WALK_EXTENDED_Z + WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z + WALK_SIDE_MAX;}

  if(legPosBuffer[2] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[2] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[5] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[5] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[8] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[8] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}
  if(legPosBuffer[11] < WALK_EXTENDED_Z - WALK_SIDE_MAX){legPosBuffer[11] = WALK_EXTENDED_Z - WALK_SIDE_MAX;}

  singleLegCtrl(1,  WALK_EXTENDED_X, legPosBuffer[1] , legPosBuffer[2]);
  singleLegCtrl(2, -WALK_EXTENDED_X, legPosBuffer[4] , legPosBuffer[5]);
  singleLegCtrl(3,  WALK_EXTENDED_X, legPosBuffer[7] , legPosBuffer[8]);
  singleLegCtrl(4, -WALK_EXTENDED_X, legPosBuffer[10], legPosBuffer[11]);
}


// BALANCE
void balancing(){
  BALANCE_PITCHU_BUFFER += ACC_Y * BALANCE_P;
  BALANCE_ROLL_BUFFER -= ACC_X * BALANCE_P;

  if(BALANCE_PITCHU_BUFFER > 21){BALANCE_PITCHU_BUFFER = 21;}
  if(BALANCE_PITCHU_BUFFER < -21){BALANCE_PITCHU_BUFFER = -21;}

  if(BALANCE_ROLL_BUFFER > 21){BALANCE_ROLL_BUFFER = 21;}
  if(BALANCE_ROLL_BUFFER < -21){BALANCE_ROLL_BUFFER = -21;}
  
  pitchYawRoll(BALANCE_PITCHU_BUFFER, 0, BALANCE_ROLL_BUFFER);
}


// mass center adjust test.
void massCenerAdjustTestLoop(){
  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-20+i, i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(i, 20-i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(20-i, -i);
    GoalPosAll();
    delay(0);
  }

  for(float i = 0; i<=20; i = i+0.6){
    standMassCenter(-i, -20+i);
    GoalPosAll();
    delay(0);
  }
}


// pitch yaw roll test.
void pitchYawRollTestLoop(){
  for(int i = -23; i<23; i++){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(0, 0, i);
    GoalPosAll();
    delay(10);
  }

  for(int i = -23; i<23; i++){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
  for(int i = 23; i>-23; i--){
    pitchYawRoll(i, 0, 0);
    GoalPosAll();
    delay(10);
  }
}


// key frames ctrl.
// 0 <= rateInput <= 1
// use it fuction to ctrl a number change from numStart to numEnd.
float linearCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*rateInput + numStart;
  return numOut;
}


// linearCtrl() function is a simple example, which shows how besselCtrl works.
float besselCtrl(float numStart, float numEnd, float rateInput){
  float numOut;
  numOut = (numEnd - numStart)*((cos(rateInput*M_PI-M_PI)+1)/2) + numStart;
  return numOut;
}


// Functions.
void functionStayLow(){
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i));
    GoalPosAll();
    delay(1);
  }
  delay(300);
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MAX, i));
    GoalPosAll();
    delay(1);
  }
  for(float i = 0; i<=1; i+=0.02){
    standUp(besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i));
    GoalPosAll();
    delay(1);
  }
}


void functionHandshake(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), besselCtrl(WALK_EXTENDED_Z, -15, i));
    singleLegCtrl(3,  besselCtrl(WALK_EXTENDED_X, 0, i), besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MAX, i), WALK_EXTENDED_Z);

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN-10, i), besselCtrl(WALK_EXTENDED_Z, 2*WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }


  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(3,  besselCtrl(0, WALK_RANGE/2+WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT_MIN, i), besselCtrl(WALK_EXTENDED_Z, 0, i));

    GoalPosAll();
    delay(1);
  }

  for(int shakeTimes = 0; shakeTimes < 3; shakeTimes++){
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT_MIN+30, i), 0);

      GoalPosAll();
      delay(1);
    }
    for(float i = 0; i<=1; i+=0.03){
      singleLegCtrl(3,  WALK_RANGE/2+WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN+30, WALK_HEIGHT_MIN, i), 0);

      GoalPosAll();
      delay(1);
    }
  }

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1,  besselCtrl(0, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MAX, WALK_HEIGHT, i), besselCtrl(-15, WALK_EXTENDED_Z, i));
    singleLegCtrl(3,  besselCtrl(WALK_RANGE/2+WALK_EXTENDED_X, WALK_EXTENDED_X, i), besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), besselCtrl(0, WALK_EXTENDED_Z, i));

    singleLegCtrl(2,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));
    singleLegCtrl(4,  -WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN-10, WALK_HEIGHT, i), besselCtrl(2*WALK_EXTENDED_Z, WALK_EXTENDED_Z, i));

    GoalPosAll();
    delay(1);
  }
}


void functionJump(){
  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT, WALK_HEIGHT_MIN, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }

  singleLegCtrl(1, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(2,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(3, WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  singleLegCtrl(4,-WALK_EXTENDED_X, WALK_HEIGHT_MAX, WALK_EXTENDED_Z);
  GoalPosAll();
  delay(70);

  for(float i = 0; i<=1; i+=0.02){
    singleLegCtrl(1, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(2,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(3, WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    singleLegCtrl(4,-WALK_EXTENDED_X, besselCtrl(WALK_HEIGHT_MIN, WALK_HEIGHT, i), WALK_EXTENDED_Z);
    GoalPosAll();
    delay(1);
  }
}


void functionActionA(){
  WALK_LIFT = 9;
  WALK_LIFT_PROP = 0.25;
  STEP_ITERATE = 0.04;
  GAIT_TYPE = 0;
}


void functionActionB(){
  WALK_LIFT = 9;
  WALK_LIFT_PROP=0.18;
  STEP_ITERATE = 0.01;
  GAIT_TYPE = 1;
}


void functionActionC(){

}


// base move ctrl.
void robotCtrl(){
  // move ctrl.
  if(!debugMode && !funcMode){
    if(moveFB == 0 && moveLR == 0 && STAND_STILL == 0){
      standMassCenter(0, 0);
      GoalPosAll();
      STAND_STILL = 1;
      GLOBAL_STEP = 0;
      delay(STEP_DELAY);
    }
    else if(moveFB == 0 && moveLR == 0 && STAND_STILL == 1){
      GoalPosAll();
      delay(STEP_DELAY);
    }
    else{
      STAND_STILL = 0;
      gestureUD = 0;
      gestureLR = 0;
      if(GLOBAL_STEP > 1){GLOBAL_STEP = 0;}
      if(moveFB == 1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 0, 0);}
      else if(moveFB == -1 && moveLR == 0){gaitTypeCtrl(GLOBAL_STEP, 180, 0);}
      else if(moveFB == 1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 30, 0);}
      else if(moveFB == 1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -30, 0);}
      else if(moveFB == -1 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, -120, 0);}
      else if(moveFB == -1 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 120, 0);}
      else if(moveFB == 0 && moveLR == -1){gaitTypeCtrl(GLOBAL_STEP, 0, -1);}
      else if(moveFB == 0 && moveLR == 1){gaitTypeCtrl(GLOBAL_STEP, 0, 1);}
      GoalPosAll();
      GLOBAL_STEP += STEP_ITERATE;
      delay(STEP_DELAY);
    }
  }
  // function ctrl.
  else if(!debugMode && funcMode){
    if(funcMode == 1){
      accXYZUpdate();
      balancing();
      GoalPosAll();
    }
    else if (funcMode == 2){
      Serial.println("stayLow");
      functionStayLow();
      funcMode = 0;
    }
    else if (funcMode == 3){
      Serial.println("handshake");
      functionHandshake();
      funcMode = 0;
    }
    else if (funcMode == 4){
      Serial.println("Jump");
      functionJump();
      funcMode = 0;
    }
    else if (funcMode == 5){
      Serial.println("ActionA");
      functionActionA();
      funcMode = 0;
    }
    else if (funcMode == 6){
      Serial.println("ActionB");
      functionActionB();
      funcMode = 0;
    }
    else if (funcMode == 7){
      Serial.println("ActionC");
      functionActionC();
      funcMode = 0;
    }
    else if (funcMode == 8){
      Serial.println("InitPos");
      initPosAll();
    }
    else if (funcMode == 9){
      Serial.println("MiddlePos");
      middlePosAll();
    }
    setSingleLED(0,matrix.Color(0, 128, 255));
    setSingleLED(1,matrix.Color(0, 128, 255));
  }
  else if(debugMode){
    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));
    delay(100);
  }
}


// if the level of IO12 is HIGH, go into debugMode.
void wireDebugDetect(){
  if(digitalRead(WIRE_DEBUG) == HIGH){
    initPosAll();
    debugMode = 1;

    setSingleLED(0,matrix.Color(255, 64, 0));
    setSingleLED(1,matrix.Color(255, 64, 0));

    while(digitalRead(WIRE_DEBUG) == HIGH){
      delay(100);
    }
    delay(1000);
  }
}