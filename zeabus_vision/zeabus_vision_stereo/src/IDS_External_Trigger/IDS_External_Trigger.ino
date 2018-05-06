#include "TimerOne.h"  //too lazy to write everything from ground up
#include <EEPROM.h>
#include <stdlib.h> 
#define TRIGGER_PIN_P 13
#define TRIGGER_PIN_N 12

#define INIT_INTERVAL 25000  // 20 hzat startup
#define ONE_SEC_IN_MICROSEC 1000000 // 1 second in microseconds

int address = 0;
byte serialInput[4];
uint8_t triggerRate = 12;

void updateTriggerRate(uint8_t rate){
  unsigned long interval = ONE_SEC_IN_MICROSEC / rate / 2;
  Timer1.setPeriod(interval);
}

volatile uint8_t triggerState = LOW;
void trigger(){
  if (triggerState == LOW) {
    triggerState = HIGH;
  } else {
    triggerState = LOW;
  }
  digitalWrite(TRIGGER_PIN_P, triggerState);
}

void setup(void){
  Serial.begin(115200);
  pinMode(TRIGGER_PIN_P, OUTPUT);
  pinMode(TRIGGER_PIN_N, OUTPUT);
  digitalWrite(TRIGGER_PIN_P, LOW);
  digitalWrite(TRIGGER_PIN_N, LOW);

  EEPROM.get(address, triggerRate);
  Timer1.initialize(INIT_INTERVAL);  
  Timer1.attachInterrupt(trigger);
  updateTriggerRate(triggerRate);
  while (!Serial){
    ;  
  }
}


void loop(){
  if (Serial.available()){
    String str = Serial.readString();
    char *cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());
    uint8_t temp;
    sscanf(cstr,"%s %d",cstr,&temp);
    if (strncmp(cstr,"set",3)==0){
      triggerRate = constrain(temp,1,30);
      updateTriggerRate(triggerRate);
      EEPROM.write(address,triggerRate);
      Serial.println(triggerRate);
    }
    delete [] cstr;
  }
  triggerRate = constrain(triggerRate,1,30);
  Serial.print("frame rate:");
  Serial.println(triggerRate);
  Serial.flush();

}

