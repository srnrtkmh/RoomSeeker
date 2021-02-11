//================================================================================================//
//                                                                                                //
// FILE : sensor_all.ino                                                                          //
// MEMO : Read sensor data and transmit them to upper controller                                  //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/17 : Start this project based on control_all.ino                                     //
//                Added us sensor reading program                                                 //
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
//                                         Copyright (c) 2021 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <avr/interrupt.h>
#include <TimerOne.h>
#include <avr/wdt.h>
#include "mpu9250.h"
#include "PS2X_lib.h"
#include "command.h"

#define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For ir sensor
#define IRNUM 12
const uint8_t IrPIN[IRNUM] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49};
uint8_t ir_state[IRNUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t ir_hex = 0;

// For MPU-9250 IMU sensor
Mpu9250 mpu9250;

// For battery check
const uint8_t BAT_AD_PIN = A0;                      // Battery voltage pin
long bat_vol_raw = 0;                               // Battery voltage A/D conversion result
long bat_vol_x100 = 0;                              // Battery voltage [x100 V]

// For ultra sonic sensor
const uint8_t USNUM = 8;
const uint8_t EchoPIN[USNUM] = {10, 11, 12, 13, 52, 15, 14, 53};  // Ultrasonic Echo pin
const uint8_t TrigPIN[USNUM] = {30, 31, 32, 33, 34, 35, 36, 37};  // Ultrasonic Trig pin
const uint8_t N2PCI_P[USNUM] = {0, 0, 0, 0, 0, 1, 1, 0};          // Convert the pin number to pin change interrupt port
const uint8_t N2PCI_N[USNUM] = {4, 5, 6, 7, 1, 1, 2, 0};          // Convert the pin number to pin change interrupt pin

uint8_t us_target = 0;                            // The target us sensor
uint8_t us_reload_flag = 0;                       // This flag is set 1 loop of distance detection
unsigned long us_time_1 = 0, us_time_2 = 0;       // Temp variables using us reflection time
unsigned long us_time_diff = 0;                   // Differential time between us transmission and reception
unsigned long us_dist[USNUM] = {0};               // Distance value from us sensor
uint8_t us_ok[USNUM] = {0};                       // Distance value can be used or not
uint8_t us_ok_hex = 0;                            // Distance value can be used or not
const uint16_t a_us_x1000 = 1742, b_us = 381;     // Co-efficient of approximation straight line

uint8_t us_ref_cnt = 0;                           // US sensor reflection cycle counter
const uint8_t US_REF_CNT_MAX = 3;                 // Max count of us_ref_cnt

const uint8_t US_1_WAIT_NEXT = 0;                 // This shows us sensor is waiting next transmission
const uint8_t US_2_WAIT_REF = 1;                  // This shows us sensor is waiting for reflection
uint8_t us_status = US_1_WAIT_NEXT;               // US sensor status

// Useful constants for calculation
long n = 0;                                         // Sample counter variable
byte start_bit = 0;                                 // Periodic process is enabled or not
long start_time = 0, stop_time = 0, interval = 0;   // For measuring interval time
const long sampling_time = 20000;                   // Sampling time[us]
const long dt_x1000 = 20;                           // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
const long _dt_x10 = 500;                           // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                             // Pi x 100
const long baudrate = 115200;                       // Baudrate setting

// For PS2
PROGMEM const uint8_t PS2_DAT = 55;                 // DAT pin settings
PROGMEM const uint8_t PS2_SEL = 56;                 // SEL pin settings
PROGMEM const uint8_t PS2_CMD = 57;                 // CMD pin settings
PROGMEM const uint8_t PS2_CLK = 58;                 // CLK pin settings
PROGMEM const bool pressures = true;                // pressures is enabled or not
PROGMEM const bool rumble    = true;                // rumble is enabled or not
byte ps2_error = 0;                                 // PS2 controller can communicate or not
byte ps2_type = 0;                                  // PS2 controller type
byte ps2_vibrate = 0;                               // vibration is enabled or not
uint16_t ps2_button = 0;                            // This flag shows command from PS2 controller is enable or not
uint8_t ps2_analogRX = 0;                           // This flag shows command from PS2 controller is enable or not
uint8_t ps2_analogRY = 0;                           // This flag shows command from PS2 controller is enable or not
uint8_t ps2_analogLX = 0;                           // This flag shows command from PS2 controller is enable or not
uint8_t ps2_analogLY = 0;                           // This flag shows command from PS2 controller is enable or not
PS2X ps2x;                                          // create PS2 Controller Object

// Reset func
void (* resetFunc) (void) = 0;                      // Reset function

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
  Serial.print("n,ir_state,mpu9250_ax,mpu9250_ay,mpu9250_az,mpu9250_gx,mpu9250_gy,mpu9250_gz,mpu9250_mx,mpu9250_my,mpu9250_mz,mpu9250_tc,us_dist[0],us_dist[1],us_dist[2],us_dist[3],us_dist[4],us_dist[5],us_dist[6],us_dist[7],us_ok_hex,bat_vol_x100,ps2_button,ps2_analogRX,ps2_analogRY,ps2_analogLX,ps2_analogLY,interval\n");
}

//================================================================================================//
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  uint8_t i;
  char str[64];
  sprintf(str, "%3d,", (int16_t)n);
  Serial.print(str);
  sprintf(str, "%4x,", ir_hex);
  Serial.print(str);
  sprintf(str, "%6d,%6d,%6d,", mpu9250.ax, mpu9250.ay, mpu9250.az);
  Serial.print(str);
  sprintf(str, "%6d,%6d,%6d,", mpu9250.gx, mpu9250.gy, mpu9250.gz);
  Serial.print(str);
  sprintf(str, "%6d,%6d,%6d,", mpu9250.mx, mpu9250.my, mpu9250.mz);
  Serial.print(str);
  sprintf(str, "%6d,", mpu9250.tc);
  Serial.print(str);
  for (i = 0; i < USNUM; i++) {
    sprintf(str, "%4ld,", us_dist[i]);
    Serial.print(str);
  }
  sprintf(str, "%2x,", us_ok_hex);
  Serial.print(str);
  sprintf(str, "%4d,", (uint16_t)bat_vol_x100);
  Serial.print(str);
  sprintf(str, "%4x,%4d,%4d,%4d,%4d,", ps2_button, ps2_analogRX, ps2_analogRY, ps2_analogLX, ps2_analogLY);
  Serial.print(str);
  sprintf(str, "%6d", (uint16_t)interval);
  Serial.print(str);
  Serial.print("\n");
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
  if (++us_target == USNUM) {
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
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  int i;

  start_time = micros();
  interrupts();     // Enable interrupts
  wdt_reset();      // Reset WDT

  // Read ir sensor state
  ir_hex = 0;
  for (i = 0; i < IRNUM; i++) {
    ir_state[i] = digitalRead(IrPIN[i]);
    ir_hex |= (ir_state[i] << i);
  }

  // Read MPU-9250 IMU sensor value
  mpu9250.reload_data();

  // Read battery voltage
  bat_vol_raw = analogRead(A0);
  bat_vol_x100 = bat_vol_raw * 1000 / 1024;

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

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    print_data();           // Output log data to upper system
    if (++n == 1000) n = 0; // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {

  }
  stop_time = micros();
  interval = stop_time - start_time;
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  uint8_t i;
  Serial.begin (baudrate);  // Initialize Serial
  delay(100);               // added delay to give wireless ps2 module some time to startup, before configuring it

  // Ir sensor input pin setting
  for (i = 0; i < IRNUM; i++) {
    pinMode(IrPIN[i], INPUT_PULLUP);
  }

  // Initialize MPU-9250
  mpu9250.init();

  // Initialize ultra sonic sensor
  PCICR |= _BV(PCIE0) | _BV(PCIE1); // Enables PCI0 & PCI1
  for (i = 0; i < USNUM; i++) {     // Initialize I/O pins
    pinMode(TrigPIN[i], OUTPUT);
    pinMode(EchoPIN[i], INPUT);
  }
  us_transmit();                    // Transmit first us transmission

  // Setup of PS2 Controller
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  ps2_type = ps2x.readType();

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録

  // Enable watch dog timer
  wdt_enable(WDTO_1S);
}

//================================================================================================//
// polling()                                                                                      //
//================================================================================================//
void polling(void) {
  char a;

  while (Serial.available()) {
    a = char(Serial.read());

    // Check start, stop charater
    if (a == 's') {
      print_label();
      start_bit = 1;
    }
    else if (a == 't') {
      start_bit = 0;
    }
  }
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  uint16_t ps2_tmp = 0;

  // Reading serial communication command --------------------------------------------------------//
  polling();

  // Reading PS2 Controller key ------------------------------------------------------------------//
  if (ps2_error == 1) {
    // resetFunc();
    ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    ps2_type = ps2x.readType();
  }
  if (ps2_type != 2) { // DualShock Controller
    ps2x.read_gamepad(false, ps2_vibrate); //read controller and set large motor to spin at 'vibrate' speed

    // Command to linear direction X
    ps2_tmp = 0;
    if (ps2x.Button(PSB_SELECT))     ps2_tmp |= PSB_SELECT;
    if (ps2x.Button(PSB_L3))         ps2_tmp |= PSB_L3;
    if (ps2x.Button(PSB_R3))         ps2_tmp |= PSB_R3;
    if (ps2x.Button(PSB_START))      ps2_tmp |= PSB_START;
    if (ps2x.Button(PSB_PAD_UP))     ps2_tmp |= PSB_PAD_UP;
    if (ps2x.Button(PSB_PAD_RIGHT))  ps2_tmp |= PSB_PAD_RIGHT;
    if (ps2x.Button(PSB_PAD_DOWN))   ps2_tmp |= PSB_PAD_DOWN;
    if (ps2x.Button(PSB_PAD_LEFT))   ps2_tmp |= PSB_PAD_LEFT;
    if (ps2x.Button(PSB_L2))         ps2_tmp |= PSB_L2;
    if (ps2x.Button(PSB_R2))         ps2_tmp |= PSB_R2;
    if (ps2x.Button(PSB_L1))         ps2_tmp |= PSB_L1;
    if (ps2x.Button(PSB_R1))         ps2_tmp |= PSB_R1;
    if (ps2x.Button(PSB_TRIANGLE))   ps2_tmp |= PSB_TRIANGLE;
    if (ps2x.Button(PSB_CIRCLE))     ps2_tmp |= PSB_CIRCLE;
    if (ps2x.Button(PSB_CROSS))      ps2_tmp |= PSB_CROSS;
    if (ps2x.Button(PSB_SQUARE))     ps2_tmp |= PSB_SQUARE;
    ps2_button = ps2_tmp;

    ps2_analogRX = ps2x.Analog(PSS_RX);
    ps2_analogRY = ps2x.Analog(PSS_RY);
    ps2_analogLX = ps2x.Analog(PSS_LX);
    ps2_analogLY = ps2x.Analog(PSS_LY);
  }
}
