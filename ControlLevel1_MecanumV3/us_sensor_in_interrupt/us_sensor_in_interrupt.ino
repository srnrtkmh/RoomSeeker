//================================================================================================//
//                                                                                                //
// FILE : us_sensor_in_interrupt.ino                                                              //
// MEMO : This program is test program to use ultra sonic distance sensor with pin change         //
//        interrupt in order to disturb the main process e.g. timer periodic loop                 //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/17 : Start this project based on us_sensor_in_loop.ino                               //
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
// For ultra sonic sensor
const uint8_t EchoPIN[8] = {10, 11, 12, 13, 52, 15, 14, 53};  // Ultrasonic Echo pin
const uint8_t TrigPIN[8] = {30, 31, 32, 33, 34, 35, 36, 37};  // Ultrasonic Trig pin
const uint8_t N2PCI_P[8] = {0, 0, 0, 0, 0, 1, 1, 0};          // Convert the pin number to pin change interrupt port
const uint8_t N2PCI_N[8] = {4, 5, 6, 7, 1, 1, 2, 0};          // Convert the pin number to pin change interrupt pin

uint8_t us_target = 0;                            // The target us sensor
uint8_t us_reload_flag = 0;                       // This flag is set 1 loop of distance detection
unsigned long us_time_1 = 0, us_time_2 = 0;       // Temp variables using us reflection time
unsigned long us_time_diff = 0;                   // Differential time between us transmission and reception
unsigned long us_dist[8] = {0};                   // Distance value from us sensor
uint8_t us_ok[8] = {0};                           // Distance value can be used or not
uint8_t us_ok_hex = 0;                            // Distance value can be used or not
const uint16_t a_us_x1000 = 1742, b_us = 381;     // Co-efficient of approximation straight line

uint8_t us_ref_cnt = 0;                           // US sensor reflection cycle counter
const uint8_t US_REF_CNT_MAX = 3;                 // Max count of us_ref_cnt

const uint8_t US_1_WAIT_NEXT = 0;                 // This shows us sensor is waiting next transmission
const uint8_t US_2_WAIT_REF = 1;                  // This shows us sensor is waiting for reflection
uint8_t us_status = US_1_WAIT_NEXT;               // US sensor status

// Useful constants for calculation
long n = 0;                                       // Sample counter variable
byte start_bit = 0;                               // Periodic process is enabled or not
long start_time = 0, stop_time = 0, interval = 0; // For measuring interval time
const long sampling_time = 20000;                 // Sampling time[us]
const long baudrate = 115200;                     // Baudrate setting

//================================================================================================//
// int watch(uint8_t trig, uint8_t echo) --- detection of ultrasonic distance                     //
//   Arguments : none                                                                             //
//   Return    : Measured value from ultra sonic sensor                                           //
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
// void us_transmit(void)                                                                         //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void us_transmit(void) {
  digitalWrite(TrigPIN[us_target], HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPIN[us_target], LOW);
  us_time_1 = micros();
  noInterrupts();
  if (N2PCI_P[us_target] == 0) {
    PCMSK0 = (1 << N2PCI_N[us_target]);
  } else if (N2PCI_P[us_target] == 1) {
    PCMSK1 = (1 << N2PCI_N[us_target]);
  }
  interrupts();
  us_ref_cnt = 0;
  us_status = US_2_WAIT_REF;
}

//================================================================================================//
// void us_receive(void)                                                                          //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void us_receive(void) {
  us_time_2 = micros();
  noInterrupts();
  if (N2PCI_P[us_target] == 0) {
    PCMSK0 &= ~(1 << N2PCI_N[us_target]);
  } else if (N2PCI_P[us_target] == 1) {
    PCMSK1 &= ~(1 << N2PCI_N[us_target]);
  }
  interrupts();
  us_time_diff = us_time_2 - us_time_1;
  us_dist[us_target] = us_time_diff * a_us_x1000 / 10000 - b_us;
  if (++us_target == 8) {
    us_target = 0;
    us_reload_flag = 1;
    stop_time = micros();
    interval = stop_time - start_time;
    start_time = micros();
  }
  us_status = US_1_WAIT_NEXT;
}

//================================================================================================//
// void pcint(void)                                                                               //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void pcint(void) {
  if (digitalRead(EchoPIN[us_target]) == LOW) {
    us_ok[us_target] = 1;
    us_ok_hex |= (1 << us_target);
    us_receive();
    us_transmit();
  }
  interrupts();
}

//================================================================================================//
// Pin Change Interrupt Function                                                                  //
// MEMO : Used for detecting ultrasonic reflection                                                //
//================================================================================================//
ISR(PCINT0_vect) {
  pcint();
}
ISR(PCINT1_vect) {
  pcint();
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : Used for periodic processing                                                            //
//================================================================================================//
void flash() {
  int i;
  char str[256];
  interrupts();     // Enable interrupts

  // Ultrasonic sequence
  if (us_status == US_2_WAIT_REF) {
    if (++us_ref_cnt == US_REF_CNT_MAX) {
      us_ref_cnt = 0;
      us_ok[us_target] = 0;
      us_ok_hex &= ~(1 << us_target);
      us_receive();
      us_transmit();
    }
  }

  // Transmit log data
  if (us_reload_flag == 1) {
    us_reload_flag = 0;
    for (i = 0; i < 8 ; i++) {
      sprintf(str, "%5ld,", us_dist[i]);
      Serial.print(str);
      sprintf(str, "%d,", us_ok[i]);
      Serial.print(str);
    }
    Serial.print(interval);
    Serial.print(",");
    Serial.print(us_ok_hex);
    Serial.print("\n");
  }

  // Debug
  //sprintf(str, "n = %ld, us_status = %d, us_target = %d, us_ref_cnt = %d\n", n, us_status, us_target, us_ref_cnt);
  //Serial.print(str);

  if (++n == 1000) n = 0; // Increment sample counter
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  uint8_t i;
  PCICR |= _BV(PCIE0) | _BV(PCIE1); // Enables PCI0 & PCI1
  for (i = 0; i < 8; i++) {         // Initialize I/O pins
    pinMode(TrigPIN[i], OUTPUT);
    pinMode(EchoPIN[i], INPUT);
  }
  Serial.begin (baudrate);          // Initialize Serial
  Timer1.initialize(sampling_time); // Initialize Timer 1
  Timer1.attachInterrupt(flash);    // Attach interrupt function
  us_transmit();                    // Transmit first us transmission
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
}
