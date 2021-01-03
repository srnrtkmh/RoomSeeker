//================================================================================================//
//                                                                                                //
// FILE : us_sensor_in_interrupt.ino                                                              //
// MEMO : 超音波センサを割り込みプログラム中で使用するテストプログラム                                       //
// Update Log                                                                                     //
//   2020/12/11 : Start this project                                                              //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 - ECHO4 Echo input from US Sensor 4        1 -                                            //
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
// For ultra sonic sensor
const uint8_t EchoPIN[8] = {10, 11, 12, 13,  0, 15, 14, 53};  // Ultrasonic Echo pin
const uint8_t TrigPIN[8] = {30, 31, 32, 33, 34, 35, 36, 37};  // Ultrasonic Trig pin

//================================================================================================//
// int watch(void) --- detection of ultrasonic distance                                           //
//================================================================================================//
int watch(uint8_t trig, uint8_t echo) {
  long echo_distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(15);
  digitalWrite(trig, LOW);
  echo_distance = pulseIn(echo, HIGH, 100000);
  echo_distance = echo_distance * 0.1657; //how far away is the object in mm
  return round(echo_distance);
}

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    pinMode(TrigPIN[i], OUTPUT);
    pinMode(EchoPIN[i], INPUT);
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
  int16_t dist_us[8];

  for (i = 0; i < 8; i++) {
    dist_us[i] = watch(TrigPIN[i], EchoPIN[i]);
  }

  sprintf(str, "dist_us : %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d [mm]", dist_us[0], dist_us[1], dist_us[2], dist_us[3], dist_us[4], dist_us[5], dist_us[6], dist_us[7]);
  Serial.print(str);
  Serial.print('\n');
  delay(1);
}
