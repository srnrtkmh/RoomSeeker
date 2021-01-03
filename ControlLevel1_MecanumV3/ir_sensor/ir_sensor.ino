//================================================================================================//
//                                                                                                //
// FILE : ir_sensor.ino                                                                           //
// MEMO : 赤外線近接センサを使用するテストプログラム                                                      //
//        赤外線センサはActive Low (0でON、1でOFF)                                                    //
// Update Log                                                                                     //
//   2020/12/11 : Start this project                                                              //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 -                                          1 -                                            //
//    2 - M0ENA Front Right L298N ENA (OC3B)       3 - M1ENB Front Left L298N ENB (OC3C)          //
//    4 -                                          5 -                                            //
//    6 - M2ENA Rear Right L298N ENA (OC4A)        7 - M3ENB Rear Left L298N ENB (OC4B)           //
//    8 - NC                                       9 - NC                                         //
//   10 - ECHO0 Echo input from US Sensor 0       11 - ECHO1 Echo input from US Sensor 1          //
//   12 - ECHO2 Echo input from US Sensor 2       13 - ECHO3 Echo input from US Sensor 3          //
//   14 - ECHO6 Echo input from US Sensor 6       15 - ECHO4 Echo input from US Sensor 4          //
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
//   52 - SCK                                     53 - ECHO7 Echo input from US Sensor 7          //
//   54 - A0                                      55 - A1                                         //
//   56 - A2                                      57 - A3                                         //
//   58 - A4                                      59 - A5                                         //
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
// #define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For ir sensor
#define IRNUM 12
const uint8_t IrPIN[IRNUM] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49};
uint8_t ir_pin_state[IRNUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
  uint8_t i;
  for (i = 0; i < IRNUM; i++) {
    pinMode(IrPIN[i], INPUT_PULLUP);
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  gpio_init();                      // Initialize GPIO
  Serial.begin (230400);            // Initialize Serial
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  uint8_t i;
  char str[128];

  for (i = 0; i < IRNUM; i++) {
    ir_pin_state[i] = digitalRead(IrPIN[i]);
  }

  sprintf(str, "ir_state : %d%d%d%d %d%d%d%d %d%d%d%d", ir_pin_state[0], ir_pin_state[1], ir_pin_state[2], ir_pin_state[3], ir_pin_state[4], ir_pin_state[5], ir_pin_state[6], ir_pin_state[7], ir_pin_state[8], ir_pin_state[9], ir_pin_state[10], ir_pin_state[11]);
  Serial.print(str);
  Serial.print('\n');
  delay(200);
}
