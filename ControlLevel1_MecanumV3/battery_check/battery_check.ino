//================================================================================================//
//                                                                                                //
// FILE : battery_check.ino                                                                       //
// MEMO : Battery voltage reading program                                                         //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 -                                          1 -                                            //
//    2 - M0ENA Front Right L298N ENA (OC3B)       3 - M1ENB Front Left L298N ENB (OC3C)          //
//    4 -                                          5 -                                            //
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
//   60 - A6                                      61 - A7                                         //
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
#include <TimerOne.h>
// #define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For battery check
const uint8_t BAT_AD_PIN = A0;                      // Battery voltage pin
long bat_vol_raw = 0;                               // Battery voltage A/D conversion result
long bat_vol_x100 = 0;                              // Battery voltage [x100 V]

const long sampling_time = 100000L;                 // Sampling time [us]
byte start_bit = 0;                                 // Periodic process is enabled or not

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  interrupts();                   // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  // Read battery voltage
  bat_vol_raw = analogRead(A0);
  bat_vol_x100 = bat_vol_raw * 1000 / 1024;

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    Serial.print("bat_vol_raw = ");
    Serial.print(bat_vol_raw);
    Serial.print("  bat_vol_x100 = ");
    Serial.print(bat_vol_x100);
    Serial.print("\n");
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin (230400);    // Initialize Serial
  delay(100);               // added delay to give wireless ps2 module some time to startup, before configuring it

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;

  // Reading serial communication command --------------------------------------------------------//
  while (Serial.available()) {
    a = char(Serial.read());
    
    // Check start, stop charater
    if (a == 's') {
      start_bit = 1;
    }
    else if (a == 't') {
      start_bit = 0;
    }
  }
}
