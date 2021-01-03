//================================================================================================//
//                                                                                                //
// FILE : imu_in_interrupt_loop.ino                                                               //
// MEMO : Read imu (MPU-9250) and transmit read data via serial                                   //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 - NC                                       1 - NC                                         //
//    2 - M0ENA Front Right L298N ENA (OC3B)       3 - M1ENB Front Left L298N ENB (OC3C)          //
//    4 - NC                                       5 - NC                                         //
//    6 - M2ENA Rear Right L298N ENA (OC4A)        7 - M3ENB Rear Left L298N ENB (OC4B)           //
//    8 - NC                                       9 - NC                                         //
//   10 - ECHO0 Echo input from US Sensor 0       11 - ECHO1 Echo input from US Sensor 1          //
//   12 - ECHO2 Echo input from US Sensor 2       13 - ECHO3 Echo input from US Sensor 3          //
//   14 - ECHO6 Echo input from US Sensor 6       15 - ECHO5 Echo input from US Sensor 5          //
//   16 - NC                                      17 - NC                                         //
//   18 - NC                                      19 - NC                                         //
//   20 - SDA                                     21 - SCL                                        //
//   22 - M0-IN1 Front Right L298N IN1            23 - M0-IN2 Front Right L298N IN2               //
//   24 - M1-IN3 Front Left L298N IN3             25 - M1-IN4 Front Left L298N IN4                //
//   26 - M2-IN1 Rear Right L298N IN1             27 - M2-IN2 Rear Right L298N IN2                //
//   28 - M3-IN3 Rear Left L298N IN3              29 - M3-IN4 Rear Left L298N IN4                 //
//   30 - TRIG0 Trigger signal to US Sensor 0     31 - TRIG1 Trigger signal to US Sensor 1        //
//   32 - TRIG2 Trigger signal to US Sensor 2     33 - TRIG3 Trigger signal to US Sensor 3        //
//   34 - TRIG4 Trigger signal to US Sensor 4     35 - TRIG5 Trigger signal to US Sensor 5        //
//   36 - TRIG6 Trigger signal to US Sensor 6     37 - TRIG7 Trigger signal to US Sensor 7        //
//   38 - IR0                                     39 - IR1                                        //
//   40 - IR2                                     41 - IR3                                        //
//   42 - IR4                                     43 - IR5                                        //
//   44 - IR6                                     45 - IR7                                        //
//   46 - IR8                                     47 - IR9                                        //
//   48 - IR10                                    49 - IR11                                       //
//   50 - MISO                                    51 - MOSI                                       //
//   52 - ECHO4 Echo input from US Sensor 4       53 - ECHO7 Echo input from US Sensor 7          //
//   54 - A0 Battery voltage / 2                  55 - A1 PS2_DAT (DO for PS2 controller)         //
//   56 - A2 PS2_SEL (DO for PS2 controller)      57 - A3 PS2_CMD (DO for PS2 controller)         //
//   58 - A4 PS2_CLK (DO for PS2 controller)      59 - A5 NC                                      //
//   60 - A6 NC                                   61 - A7 NC                                      //
//   62 - A8  ENC0AF Front Right encoder phase A  63 - A9  ENC0BF Front Right encoder phase B     //
//   64 - A10 ENC1AF Front Left encoder phase A   65 - A11 ENC1BF Front Left encoder phase B      //
//   66 - A12 ENC2AF Rear Right encoder phase A   67 - A13 ENC2BF Rear Right encoder phase B      //
//   68 - A14 ENC3AF Rear Left encoder phase A    69 - A15 ENC3BF Rear Left encoder phase B       //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <avr/interrupt.h>
#include <TimerOne.h>
#include "mpu9250.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For MPU-9250
Mpu9250 mpu9250;

// Time variables
long start_time = 0;
long stop_time = 0;
const long sampling_time = 10000;   // Sampling time[us]
uint8_t start_bit = 0;              // Start bit
uint16_t n = 0;                     // Sample counter

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
  Serial.print("n,ax,ay,az,gx,gy,gz,mx,my,mz,temp,measure_time\n");
}
//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  char str[64];
  interrupts();                   // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  // Read sensor value and output if start_bit is 1
  if (start_bit == 1) {
    start_time = micros();
    mpu9250.reload_data();
    stop_time = micros();

    sprintf(str, "%6d,", n); Serial.print(str);
    sprintf(str, "%6d,%6d,%6d,", mpu9250.ax, mpu9250.ay, mpu9250.az); Serial.print(str);
    sprintf(str, "%6d,%6d,%6d,", mpu9250.gx, mpu9250.gy, mpu9250.gz); Serial.print(str);
    sprintf(str, "%6d,%6d,%6d,", mpu9250.mx, mpu9250.my, mpu9250.mz); Serial.print(str);
    sprintf(str, "%6d,", mpu9250.tc); Serial.print(str);
    sprintf(str, "%10ld\n", stop_time - start_time); Serial.print(str);
    
    n++;  // Increment sample counter
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin(230400);   // Start Serial communication
  mpu9250.init();         // Initialize MPU-9250

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;

  while (Serial.available()) {
    a = char(Serial.read());
    if (a == 's') {
      n = 0;
      print_label();
      start_bit = 1;
    }
    else if (a == 't') {
      start_bit = 0;
    }
  }
}
