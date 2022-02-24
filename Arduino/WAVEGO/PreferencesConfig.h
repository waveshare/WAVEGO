#include <Preferences.h>

Preferences preferences;

void middleUpdate(){
  ServoMiddlePWM[0] = preferences.getInt("PWM0", ServoMiddlePWM[0]);
  ServoMiddlePWM[1] = preferences.getInt("PWM1", ServoMiddlePWM[1]);
  ServoMiddlePWM[2] = preferences.getInt("PWM2", ServoMiddlePWM[2]);
  ServoMiddlePWM[3] = preferences.getInt("PWM3", ServoMiddlePWM[3]);

  ServoMiddlePWM[4] = preferences.getInt("PWM4", ServoMiddlePWM[4]);
  ServoMiddlePWM[5] = preferences.getInt("PWM5", ServoMiddlePWM[5]);
  ServoMiddlePWM[6] = preferences.getInt("PWM6", ServoMiddlePWM[6]);
  ServoMiddlePWM[7] = preferences.getInt("PWM7", ServoMiddlePWM[7]);

  ServoMiddlePWM[8] = preferences.getInt("PWM8", ServoMiddlePWM[8]);
  ServoMiddlePWM[9] = preferences.getInt("PWM9", ServoMiddlePWM[9]);
  ServoMiddlePWM[10] = preferences.getInt("PWM10", ServoMiddlePWM[10]);
  ServoMiddlePWM[11] = preferences.getInt("PWM11", ServoMiddlePWM[11]);

  ServoMiddlePWM[12] = preferences.getInt("PWM12", ServoMiddlePWM[12]);
  ServoMiddlePWM[13] = preferences.getInt("PWM13", ServoMiddlePWM[13]);
  ServoMiddlePWM[14] = preferences.getInt("PWM14", ServoMiddlePWM[14]);
  ServoMiddlePWM[15] = preferences.getInt("PWM15", ServoMiddlePWM[15]);
}


extern void servoConfigSave(byte activeServo){
  ServoMiddlePWM[activeServo] = CurrentPWM[activeServo];

  if(activeServo == 0){preferences.putInt("PWM0", CurrentPWM[activeServo]);}
  else if(activeServo == 1){preferences.putInt("PWM1", CurrentPWM[activeServo]);}
  else if(activeServo == 2){preferences.putInt("PWM2", CurrentPWM[activeServo]);}

  else if(activeServo == 3){preferences.putInt("PWM3", CurrentPWM[activeServo]);}
  else if(activeServo == 4){preferences.putInt("PWM4", CurrentPWM[activeServo]);}
  else if(activeServo == 5){preferences.putInt("PWM5", CurrentPWM[activeServo]);}

  else if(activeServo == 6){preferences.putInt("PWM6", CurrentPWM[activeServo]);}
  else if(activeServo == 7){preferences.putInt("PWM7", CurrentPWM[activeServo]);}
  else if(activeServo == 8){preferences.putInt("PWM8", CurrentPWM[activeServo]);}
  
  else if(activeServo == 9){preferences.putInt("PWM9", CurrentPWM[activeServo]);}
  else if(activeServo == 10){preferences.putInt("PWM10", CurrentPWM[activeServo]);}
  else if(activeServo == 11){preferences.putInt("PWM11", CurrentPWM[activeServo]);}

  else if(activeServo == 12){preferences.putInt("PWM12", CurrentPWM[activeServo]);}
  else if(activeServo == 13){preferences.putInt("PWM13", CurrentPWM[activeServo]);}
  else if(activeServo == 14){preferences.putInt("PWM14", CurrentPWM[activeServo]);}

  else if(activeServo == 15){preferences.putInt("PWM15", CurrentPWM[activeServo]);}
}



void preferencesSetup(){
  preferences.begin("ServoConfig", false);
  middleUpdate();
  delay(500);
  // for (int i = 0; i < 16; i++){
  //   Serial.print(ServoMiddlePWM[i]);
  //   Serial.print(" ");
  // }
  // Serial.println("");
  // Serial.println("Middle PWM Setup!");
}