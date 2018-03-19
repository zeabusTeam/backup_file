#include <Servo.h>

#include <MBProtocol.h>
#include <MBDatalink.h>
#include <MBDatatypes.h>
#include <MBSerialMega328.h>
#include <MBDevice.h>

/* Regulator Main */

#define TIMEOUT  500
#define DEV_ADDR   0x30
#define LED_PIN    13
#define DIR_PIN    A4
#define CHANNEL_COUNT    8
uint16_t prev_speed[8];
uint16_t curr_speed[8];

Servo servo[8];
unsigned long prev_millis = 0;
unsigned long curr_millis = 0;
uint8_t i;

void setup() {
  uint8_t i;
  MBCore.Begin(0,0,0,CHANNEL_COUNT,DEV_ADDR);
  Set_dir_pin(DIR_PIN);  
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Fixed control
  digitalWrite(LED_PIN,HIGH);
  
  //Initial
  digitalWrite(DIR_PIN,LOW);  
  
  // Attach server to I/O pin
  for (i=0;i<CHANNEL_COUNT;i++)
  {
     servo[i].attach(i+2, 1000, 2000); 
     prev_speed[i] = 1500;
  }
  MBCore.Set_holding_register(prev_speed, 0, CHANNEL_COUNT);
}

void loop() {
    
    curr_millis = millis();
    if (curr_millis - prev_millis > TIMEOUT)
    {
        for(i=0;i<8;i++)
        {
            curr_speed[i] = 1500;
        }
        digitalWrite(LED_PIN, LOW);
        prev_millis = curr_millis;
        MBCore.Set_holding_register(curr_speed, 0, CHANNEL_COUNT);
    }else{
  
    }
    if (MBCore.Run())
    {
       digitalWrite(LED_PIN, HIGH);
       prev_millis = curr_millis; // Must call at least every 1ms 
    }
    MBCore.Read_holding_register(curr_speed, 0, CHANNEL_COUNT);
    for (i=0;i<CHANNEL_COUNT;i++)
    {
      if(curr_speed[i] != prev_speed[i])
      {
         //servo[i].write(curr_speed[i]);
         servo[i].writeMicroseconds(curr_speed[i]);
         prev_speed[i] = curr_speed[i];
      } 
    }
    
}


