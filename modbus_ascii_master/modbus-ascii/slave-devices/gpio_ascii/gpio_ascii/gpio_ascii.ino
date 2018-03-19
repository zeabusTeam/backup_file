#include <MBProtocol.h>
#include <MBDatalink.h>
#include <MBDatatypes.h>
#include <MBSerialMega328.h>
#include <MBDevice.h>

/* GPIO */

// Microcontroller's pin defination.
#define DEV_ADDR      0x12
#define RC_A          A0
#define RC_B          A1
#define RC_C          A2
#define RC_D          A3
#define RC_E          A4
#define RC_F          A5
#define RC_G          3
#define LED_PIN       8
#define LED0          7
#define LED1          9
#define LED2          10
#define D11           11
#define D12           12
#define SW0           4
#define SW1           5
#define SW2           6
#define DIR_PIN       2  

#define SINK_A        RC_A
#define SINK_B        RC_B
#define SINK_C        RC_C
#define SINK_D        RC_D
#define SINK_E        RC_E
#define SINK_F        RC_F
#define SINK_G        RC_G
#define LED3          D11
#define LED4          D12

uint16_t pressure;
uint16_t ro_data[2];
uint8_t sw_status_bitstream;

struct gpio_control_t
{
   union
   {
       struct
       {
          uint16_t sink_a           :1;
          uint16_t sink_b           :1;
          uint16_t sink_c           :1;
          uint16_t sink_d           :1;
          uint16_t sink_e           :1;
          uint16_t sink_f           :1;
          uint16_t sink_g           :1;
          uint16_t led0             :1;
          uint16_t led1             :1;
          uint16_t led2             :1;
          uint16_t led3             :1;
          uint16_t led4             :1;
          uint16_t unused           :4;         
       };
       uint16_t rawData;
   };
};
gpio_control_t control;

void setup() {
  pinMode(SINK_A, OUTPUT);
  pinMode(SINK_B, OUTPUT); 
  pinMode(SINK_C, OUTPUT);
  pinMode(SINK_D, OUTPUT);
  pinMode(SINK_E, OUTPUT);
  pinMode(SINK_F, OUTPUT);
  pinMode(SINK_G, OUTPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(SW0, INPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  Set_dir_pin(DIR_PIN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  MBCore.Begin(3,0,2,1,DEV_ADDR);  // 3 discrete input bits, 1 input registers, 1 holding register.
  control.rawData = 0;
  digitalWrite(LED_PIN,HIGH);
  digitalWrite(DIR_PIN,LOW);
  pressure = 0;
  sw_status_bitstream = 0;
  
}

void loop() {
    MBCore.Run(); // Must call at least every 1ms 
    /* read pressure */
    ro_data[0] = analogRead(A6);
    //pressure = analogRead(A6);
    /*---------------*/
    MBCore.Set_input_register(&pressure, 0, 1);
    /* read switch status */
    if (digitalRead(SW0) == HIGH) sw_status_bitstream |= (1<<0);
    else sw_status_bitstream &= ~(1<<0);
    if (digitalRead(SW1) == HIGH) sw_status_bitstream |= (1<<1);
    else sw_status_bitstream &= ~(1<<1);
    if (digitalRead(SW2) == HIGH) sw_status_bitstream |= (1<<2);
    else sw_status_bitstream &= ~(1<<2);   
    
    ro_data[1] = sw_status_bitstream;
    /*--------------------*/
    
    MBCore.Set_discrete_input(&sw_status_bitstream, 0, 3);
    MBCore.Set_input_register(ro_data, 0, 2);
    MBCore.Read_holding_register(&(control.rawData), 0, 1);
    digitalWrite(SINK_A, (control.sink_a)?HIGH:LOW);
    digitalWrite(SINK_B, (control.sink_b)?HIGH:LOW);
    digitalWrite(SINK_C, (control.sink_c)?HIGH:LOW);
    digitalWrite(SINK_D, (control.sink_d)?HIGH:LOW);
    digitalWrite(SINK_E, (control.sink_e)?HIGH:LOW);
    digitalWrite(SINK_F, (control.sink_f)?HIGH:LOW);
    digitalWrite(SINK_G, (control.sink_g)?HIGH:LOW);
    digitalWrite(LED0, (control.led0)?HIGH:LOW);
    digitalWrite(LED1, (control.led1)?HIGH:LOW);
    digitalWrite(LED2, (control.led2)?HIGH:LOW);
    digitalWrite(LED3, (control.led3)?HIGH:LOW);
    digitalWrite(LED4, (control.led4)?HIGH:LOW);
  }


