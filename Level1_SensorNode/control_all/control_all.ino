//================================================================================================//
//                                                                                                //
// FILE : control_all.ino                                                                         //
// MEMO : Control 4 motors velocity by PI control                                                 //
//        Added feedforward control with friction compensation                                    //
//        Added reading process ir sensor on/off                                                  //
//                                                                                                //
// Update Log                                                                                     //
//   2020/12/11 : Start this project based on motor_control.ino                                   //
//                Added ir sensor reading process                                                 //
//   2021/01/02 : Added rate limit process                                                        //
//   2021/01/03 : Added IMU reading process                                                       //
//                Added battery voltage checking process                                          //
//                Purged command reading function to the other file                               //
//                Embeded watch dog timer                                                         //
//   2021/01/10 : 送信側をSerial.printを使わないように変更                                             //
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
#include <avr/interrupt.h>
#include <TimerOne.h>
#include <avr/wdt.h>
#include "rotary_encoders.h"
#include "mpu9250.h"
#include "PS2X_lib.h"
#include "acceleration.h"
#include "command.h"

#define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
PROGMEM const int FR_ENC_A = 62;                    // Input pin number of Front Right wheel encoder phase A
PROGMEM const int FR_ENC_B = 63;                    // Input pin number of Front Right wheel encoder phase B
PROGMEM const int FL_ENC_A = 64;                    // Input pin number of Front Left wheel encoder phase A
PROGMEM const int FL_ENC_B = 65;                    // Input pin number of Front Left wheel encoder phase B
PROGMEM const int RR_ENC_A = 66;                    // Input pin number of Rear Right wheel encoder phase A
PROGMEM const int RR_ENC_B = 67;                    // Input pin number of Rear Right wheel encoder phase B
PROGMEM const int RL_ENC_A = 68;                    // Input pin number of Rear Left wheel encoder phase A
PROGMEM const int RL_ENC_B = 69;                    // Input pin number of Rear Left wheel encoder phase B
PROGMEM const uint8_t FR_ENC = 0;                   // Index for Front Right wheel encoder
PROGMEM const uint8_t FL_ENC = 1;                   // Index for Front Left wheel encoder
PROGMEM const uint8_t RR_ENC = 2;                   // Index for Rear Right wheel encoder
PROGMEM const uint8_t RL_ENC = 3;                   // Index for Rear Left wheel encoder
PROGMEM const uint8_t ENC_RESO = 44;                // Encoder resolusion 11 pulse dual edge count mode
PROGMEM const uint8_t GEAR_REDU = 30;               // Gear reduction
RotaryEncoders enc;

// For Motor
PROGMEM const byte FR_PWM_PIN = 2;                  // Front Right Motor PWM pin
PROGMEM const byte FR_DIR_PIN1  = 22;               // Front Right Motor direction pin 1
PROGMEM const byte FR_DIR_PIN2  = 23;               // Front Right Motor direction pin 2
PROGMEM const byte FL_PWM_PIN = 3;                  // Front Left Motor PWM pin
PROGMEM const byte FL_DIR_PIN1  = 24;               // Front Left Motor direction pin 1
PROGMEM const byte FL_DIR_PIN2  = 25;               // Front Left Motor direction pin 2
PROGMEM const byte RR_PWM_PIN = 6;                  // Rear Right Motor PWM pin
PROGMEM const byte RR_DIR_PIN1 = 26;                // Rear Right Motor direction pin 1
PROGMEM const byte RR_DIR_PIN2 = 27;                // Rear Right Motor direction pin 2
PROGMEM const byte RL_PWM_PIN = 7;                  // Rear Left Motor PWM pin
PROGMEM const byte RL_DIR_PIN1 = 28;                // Rear Left Motor direction pin 1
PROGMEM const byte RL_DIR_PIN2 = 29;                // Rear Left Motor direction pin 2
PROGMEM const uint8_t FR_MOTOR = 0;                 // Front Right Motor index
PROGMEM const uint8_t FL_MOTOR = 1;                 // Front Left Motor index
PROGMEM const uint8_t RR_MOTOR = 2;                 // Rear Right Motor index
PROGMEM const uint8_t RL_MOTOR = 3;                 // Rear Left Motor index

// Control variables
int8_t cnt_dir[4] = { -1, 1, -1, 1};                // Direction correction co-efficient of rotation
uint16_t cnt_now[4] = {0, 0, 0, 0};                 // Current encoder counter for 4 motors [-]
uint16_t cnt_pre[4] = {0, 0, 0, 0};                 // Previous encoder counter for 4 motors [-]
int16_t diff_cnt[4] = {0, 0, 0, 0};                 // Difference between current and previous counter
long omega_res_x10[4] = {0, 0, 0, 0};               // Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf[4] = {0, 0, 0, 0};           // LPF output of Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf_tmp[4] = {0, 0, 0, 0};       // Temporary for LPF
long omega_cmd_x10[4] = {0, 0, 0, 0};               // Rotation speed command [10^-3 deg/sec]
long e_omega[4] = {0, 0, 0, 0};                     // Rotation speed error
long int_e_omega[4] = {0, 0, 0, 0};                 // Integral of rotation speed error
int16_t vout[4] = {0, 0, 0, 0};                     // Voltage output for 4 motor drivers - index 1, 2, 4 : [0 - 1023], index3 : [0 - 255]
int16_t vout_ff[4] = {0, 0, 0, 0};                  // Feedforward control output
int16_t vout_fb[4] = {0, 0, 0, 0};                  // Feedback control output
int16_t vout_ll[4] = { -1000, -1000, -1000, -1000}; // Lower limit of voltage output
int16_t vout_ul[4] = { 1000,  1000,  1000,  1000};  // Upper limit of voltage output

// Feecback parameters
long g_omega[4] = {60, 60, 60, 60};                 // Cutoff angular frequency [rad/sec]
int16_t Kp[4] = {3, 3, 3, 3};                       // P gain for PI control
// int16_t Ki[4] = {20, 20, 20, 20};                // I gain for PI control
int16_t Ki[4] = {30, 30, 30, 30};                   // I gain for PI control

// Feedforward parameters
// int16_t Fc_p[4] = {  785,  691,  699,  687};     // Coulonb friction compensation parameter for positive direction
// int16_t Fc_n[4] = { -778, -692, -720, -699};     // Coulonb friction compensation parameter for negative direction
// int16_t Fc_p[4] = {  392,  345,  349,  343};     // Coulonb friction compensation parameter for positive direction
// int16_t Fc_n[4] = { -389, -346, -360, -349};     // Coulonb friction compensation parameter for negative direction
int16_t Fc_p[4] = {  0,  0,  0,  0};             // Coulonb friction compensation parameter for positive direction
int16_t Fc_n[4] = { -0, -0, -0, -0};             // Coulonb friction compensation parameter for negative direction
int16_t Fd_p_x100[4] = {141, 173, 158, 149};     // Dynamic friction compensation parameter for negative direction
int16_t Fd_n_x100[4] = {148, 176, 154, 149};     // Dynamic friction compensation parameter for negative direction
// int16_t Fd_p_x100[4] = { 50,  50,  50,  50};     // Dynamic friction compensation parameter for negative direction
// int16_t Fd_n_x100[4] = { 50,  50,  50,  50};     // Dynamic friction compensation parameter for negative direction

// Chassis parameter
long work_vel_cmd_x10[3] = {0, 0, 0};               // Workspace control command
long work_vel_cmd_x10_rl[3] = {0, 0, 0};            // Workspace control command applied rate limit
uint16_t rate_limit_work[3] = {75, 75, 15};         // Rate limit value for workspace velocity
uint8_t workspace_ctrl = 0;                         // Workspace control flag
const int16_t W = 215;                              // Tread width
const int16_t L = 162;                              // Wheel base
const int16_t R = 40;                               // Wheel Radius

// For MPU-9250 IMU sensor
Mpu9250 mpu9250;

// For ir sensor
#define IRNUM 12
const uint8_t IrPIN[IRNUM] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49};
uint8_t ir_state[IRNUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// For battery check
const uint8_t BAT_AD_PIN = A0;                      // Battery voltage pin
long bat_vol_raw = 0;                               // Battery voltage A/D conversion result
long bat_vol_x100 = 0;                              // Battery voltage [x100 V]

// Useful constants for calculation
long n = 0;                                         // Sample counter variable
const long sampling_time = 20000;                   // Sampling time[us]
long start_time = 0, stop_time = 0, interval = 0;   //
const long dt_x1000 = 20;                           // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
const long _dt_x10 = 500;                           // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                             // Pi x 100

// For sequence
byte start_bit = 0;                                 // Periodic process is enabled or not
char rec[128] = {0};                                // Received serial characters
uint8_t rec_vel_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_vol_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr1_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr2_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr3_flag = 0;                           // The flag specifing the received data is velocity command

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
uint8_t ps2_ctrl = 0;                               // This flag shows command from PS2 controller is enable or not
PS2X ps2x;                                          // create PS2 Controller Object

// Reset func
void (* resetFunc) (void) = 0;                      // Reset function

//================================================================================================//
// FR_fwd(int speed), FR_bck(int speed) --- this is the same for FL, RR, RL motor                 //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void FR_fwd(int speed) {
  digitalWrite(FR_DIR_PIN2, HIGH);
  digitalWrite(FR_DIR_PIN1, LOW);
  OCR3B = speed;
}

void FR_bck(int speed) {
  digitalWrite(FR_DIR_PIN2, LOW);
  digitalWrite(FR_DIR_PIN1, HIGH);
  OCR3B = speed;
}

void FL_fwd(int speed) {
  digitalWrite(FL_DIR_PIN1, HIGH);
  digitalWrite(FL_DIR_PIN2, LOW);
  OCR3C = speed;
}

void FL_bck(int speed) {
  digitalWrite(FL_DIR_PIN1, LOW);
  digitalWrite(FL_DIR_PIN2, HIGH);
  OCR3C = speed;
}

void RR_fwd(int speed) {
  digitalWrite(RR_DIR_PIN2, HIGH);
  digitalWrite(RR_DIR_PIN1, LOW);
  OCR4A = speed;
}
void RR_bck(int speed) {
  digitalWrite(RR_DIR_PIN2, LOW);
  digitalWrite(RR_DIR_PIN1, HIGH);
  OCR4A = speed;
}
void RL_fwd(int speed) {
  digitalWrite(RL_DIR_PIN1, HIGH);
  digitalWrite(RL_DIR_PIN2, LOW);
  OCR4B = speed;
}
void RL_bck(int speed) {
  digitalWrite(RL_DIR_PIN1, LOW);
  digitalWrite(RL_DIR_PIN2, HIGH);
  OCR4B = speed;
}

//================================================================================================//
// stop_all(void) --- Stop all motors                                                             //
//================================================================================================//
void stop_all() {
  OCR3B = 0;    // OCR3B -> Front Right motor
  OCR3C = 0;    // OCR3C -> Front Left motor
  OCR4A = 0;    // OCR4A -> Rear Right motor
  OCR4B = 0;    // OCR4B -> Rear Left motor
}

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
#ifdef DEBUG
  // For normal
  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], ir_state, mpu9250_ax, mpu9250_ay, mpu9250_az, mpu9250_gx, mpu9250_gy, mpu9250_gz, mpu9250_mx, mpu9250_my, mpu9250_mz, mpu9250_tc, bat_vol_x100, ps2_ctrl\n");

  // For debug
  //  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], enc.read_test_cnt\n");
#endif
}

//================================================================================================//
//================================================================================================//
void transmit(char a) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = a;
}

//================================================================================================//
//================================================================================================//
void test(char *str) {
  uint8_t i = 0;
  while (str[i] != '\0') transmit(str[i++]);
}

//================================================================================================//
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  char str[64];
  sprintf(str, "%3d,", (int16_t)n);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  start_time = micros();
  sprintf(str, "%+6d,%+6d,%+6d,%+6d,", (uint16_t)cnt_now[0], (uint16_t)cnt_now[1], (uint16_t)cnt_now[2], (uint16_t)cnt_now[3]);
  stop_time = micros();
  interval = stop_time - start_time;
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)omega_res_x10[0], (int16_t)omega_res_x10[1], (int16_t)omega_res_x10[2], (int16_t)omega_res_x10[3]);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)omega_cmd_x10[0], (int16_t)omega_cmd_x10[1], (int16_t)omega_cmd_x10[2], (int16_t)omega_cmd_x10[3]);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)vout[0], (int16_t)vout[1], (int16_t)vout[2], (int16_t)vout[3]);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%d%d%d%d%d%d%d%d%d%d%d%d,", ir_state[0], ir_state[1], ir_state[2], ir_state[3], ir_state[4], ir_state[5], ir_state[6], ir_state[7], ir_state[8], ir_state[9], ir_state[10], ir_state[11]);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%6d,%6d,%6d,", mpu9250.ax, mpu9250.ay, mpu9250.az);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%6d,%6d,%6d,", mpu9250.gx, mpu9250.gy, mpu9250.gz);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%6d,%6d,%6d,", mpu9250.mx, mpu9250.my, mpu9250.mz);
  polling();                                          // #### Polling #####
  Serial.print(str);
  polling();                                          // #### Polling #####
  sprintf(str, "%6d,", mpu9250.tc);
  polling();                                          // #### Polling #####
  Serial.print(str);
  sprintf(str, "%4d,", (uint16_t)bat_vol_x100);
  polling();                                          // #### Polling #####
  Serial.print(str);
  sprintf(str, "%2d,", ps2_ctrl);
  polling();                                          // #### Polling #####
  Serial.print(str);
  sprintf(str, "%6d", (uint16_t)interval);
  polling();                                          // #### Polling #####
  Serial.print(str);
  Serial.print("\n");
}

//================================================================================================//
// Inverse kinematics                                                                             //
//   only work_vel_cmd_x10[2] is devided 10                                                       //
//================================================================================================//
void inv_kinematics(void) {
  omega_cmd_x10[0] = (work_vel_cmd_x10_rl[0] + work_vel_cmd_x10_rl[1] + (W + L) / 2 * work_vel_cmd_x10_rl[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[1] = (work_vel_cmd_x10_rl[0] - work_vel_cmd_x10_rl[1] - (W + L) / 2 * work_vel_cmd_x10_rl[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[2] = (work_vel_cmd_x10_rl[0] - work_vel_cmd_x10_rl[1] + (W + L) / 2 * work_vel_cmd_x10_rl[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[3] = (work_vel_cmd_x10_rl[0] + work_vel_cmd_x10_rl[1] - (W + L) / 2 * work_vel_cmd_x10_rl[2] / 180 * 314 / 100) / R * 18000 / 314;
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  int i;
  long tmp;

  start_time = micros();
  interrupts();     // Enable interrupts
  wdt_reset();      // Reset WDT

  // Read ir sensor state
  for (i = 0; i < IRNUM; i++) {
    ir_state[i] = digitalRead(IrPIN[i]);
  }

  // Update encoder counter value
  for (i = 0; i < 4; i++) cnt_pre[i] = cnt_now[i];
  noInterrupts();
  cnt_now[0] = cnt_dir[0] * enc.read_cnt(FR_ENC);
  cnt_now[1] = cnt_dir[1] * enc.read_cnt(FL_ENC);
  cnt_now[2] = cnt_dir[2] * enc.read_cnt(RR_ENC);
  cnt_now[3] = cnt_dir[3] * enc.read_cnt(RL_ENC);
  interrupts();
  polling();                                          // #### Polling #####

  // Read MPU-9250 IMU sensor value
  mpu9250.reload_data();
  polling();                                          // #### Polling #####

  // Read battery voltage
  bat_vol_raw = analogRead(A0);
  bat_vol_x100 = bat_vol_raw * 1000 / 1024;
  polling();                                          // #### Polling #####

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    for (i = 0; i < 3; i++) {
      if (work_vel_cmd_x10[i] > work_vel_cmd_x10_rl[i] + rate_limit_work[i])      work_vel_cmd_x10_rl[i] += rate_limit_work[i];
      else if (work_vel_cmd_x10[i] < work_vel_cmd_x10_rl[i] - rate_limit_work[i]) work_vel_cmd_x10_rl[i] -= rate_limit_work[i];
      else                                                                        work_vel_cmd_x10_rl[i] = work_vel_cmd_x10[i];
    }

    // Update inverse kinematics
    if (workspace_ctrl == 1) inv_kinematics();
    polling();                                          // #### Polling #####

    // Calculate difference between current and previous counter value
    for (i = 0; i < 4; i++) {
      tmp = cnt_now[i] - cnt_pre[i];
      if (tmp > 32767L)       diff_cnt[i] = tmp - 65536L;
      else if (tmp < -32768L) diff_cnt[i] = tmp + 65536L;
      else                    diff_cnt[i] = tmp;
    }

    // Calculate rotation speed
    for (i = 0; i < 4; i++) {
      omega_res_x10[i] = (diff_cnt[i] * 360) * _dt_x10 / ENC_RESO / GEAR_REDU;
      omega_res_x10_lpf_tmp[i] += (omega_res_x10[i] - omega_res_x10_lpf[i]) * dt_x1000;
      omega_res_x10_lpf[i] = omega_res_x10_lpf_tmp[i] * g_omega[i] / 1000;
      polling();                                          // #### Polling #####
    }

    // Calculate control output
    for (i = 0; i < 4; i++) {
      e_omega[i] = omega_cmd_x10[i] - omega_res_x10_lpf[i];
      // Feedforward control
      if (omega_cmd_x10[i] > 10) {
        vout_ff[i] = Fc_p[i] + omega_cmd_x10[i] * Fd_p_x100[i] / 1000;
      } else if (omega_cmd_x10[i] < -10) {
        vout_ff[i] = Fc_n[i] + omega_cmd_x10[i] * Fd_n_x100[i] / 1000;
      }

      // Integral is enabled when control output don't reach each limit
      if (vout_ll[i] < vout[i] && vout[i] < vout_ul[i])
        int_e_omega[i] += e_omega[i] * dt_x1000;

      // Feedback control
      vout_fb[i] = Kp[i] * e_omega[i] / 10 + Ki[i] * int_e_omega[i] / 10000;

      // Add Feedforward and Feedback
      vout[i] = vout_ff[i] + vout_fb[i];

      // Apply saturation
      if (vout[i] > vout_ul[i]) vout[i] = vout_ul[i];
      else if (vout[i] < vout_ll[i]) vout[i] = vout_ll[i];
      polling();                                          // #### Polling #####
    }

    // Apply control output to the motor
    // Front Right motor
    if (vout[0] > 0) FR_fwd(vout[0]);
    else             FR_bck(-vout[0]);

    // Front Left motor
    if (vout[1] > 0) FL_fwd(vout[1]);
    else             FL_bck(-vout[1]);
    polling();                                          // #### Polling #####

    // Rear Right motor
    if (vout[2] > 0) RR_fwd(vout[2]);
    else             RR_bck(-vout[2]);

    // Rear Left motor
    if (vout[3] > 0) RL_fwd(vout[3]);
    else             RL_bck(-vout[3]);
    polling();                                          // #### Polling #####

    print_data();           // Output log data to upper system
    if (++n == 1000) n = 0; // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }
  stop_time = micros();
  interval = stop_time - start_time;
}

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
  uint8_t i;

  // Motor control port setting
  pinMode(FR_DIR_PIN1, OUTPUT);
  pinMode(FR_DIR_PIN2, OUTPUT);
  pinMode(FL_PWM_PIN, OUTPUT);

  pinMode(FL_DIR_PIN1, OUTPUT);
  pinMode(FL_DIR_PIN2, OUTPUT);
  pinMode(FR_PWM_PIN, OUTPUT);

  pinMode(RR_DIR_PIN1, OUTPUT);
  pinMode(RR_DIR_PIN2, OUTPUT);
  pinMode(RL_PWM_PIN, OUTPUT);

  pinMode(RL_DIR_PIN1, OUTPUT);
  pinMode(RL_DIR_PIN2, OUTPUT);
  pinMode(RR_PWM_PIN, OUTPUT);

  stop_all();

  // Ir sensor input pin setting
  for (i = 0; i < IRNUM; i++) {
    pinMode(IrPIN[i], INPUT_PULLUP);
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin (230400);    // Initialize Serial
  delay(100);               // added delay to give wireless ps2 module some time to startup, before configuring it

  // Setup of PS2 Controller
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  ps2_type = ps2x.readType();

  // Initialize GPIO
  gpio_init();

  // Setup of encoder object
  enc.attach(0, FR_ENC_A, FR_ENC_B, DUAL_MULTI);
  enc.attach(1, FL_ENC_A, FL_ENC_B, DUAL_MULTI);
  enc.attach(2, RR_ENC_A, RR_ENC_B, DUAL_MULTI);
  enc.attach(3, RL_ENC_A, RL_ENC_B, DUAL_MULTI);

  // Initialize MPU-9250
  mpu9250.init();

  // Timer 3 PWM output compare settings
  OCR3A = 0;                        // Clear OCR3A
  OCR3B = 0;                        // Clear OCR3B
  OCR3C = 0;                        // Clear OCR3B
  TCNT3 = 0;                        // Clear timer 3
  TIMSK3 = 0x00;                    // None interrupts
  TCCR3A = 0xAB;                    // OC3A, OC3B, OC3C is none inverting mode, PWM mode is Fast PWM 10bit
  TCCR3B = 0x09;                    // PWM mode is Fast PWM, Clock is clkI/O/1024 (From prescaler)
  TCCR3C = 0x00;                    // Force compare match is disabled

  // Timer 4 PWM output compare settings
  OCR4A = 0;                        // Clear OCR4A
  OCR4B = 0;                        // Clear OCR4B
  OCR4C = 0;                        // Clear OCR4B
  TCNT4 = 0;                        // Clear timer 0
  TIMSK4 = 0x00;                    // None interrupts
  TCCR4A = 0xAB;                    // OC4A, OC4B, OC4C is none inverting mode, PWM mode is Fast PWM 10bit
  TCCR4B = 0x09;                    // PWM mode is Fast PWM, Clock is clkI/O/1024 (From prescaler)
  TCCR4C = 0x00;                    // Force compare match is disabled

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
  uint8_t i;
  static uint8_t rec_ptr = 0;     // Received serial data pointer

  while (Serial.available()) {
    a = char(Serial.read());
    rec[rec_ptr] = a;

    // check if the start command received or not
    if (rec_ptr >= 2) {
      if (rec[rec_ptr - 2] == 'v' && rec[rec_ptr - 1] == 'e' && rec[rec_ptr] == 'l') {
        rec[0] = 'v'; rec[1] = 'e'; rec[2] = 'l';
        for (i = 3; i < sizeof(rec) - 1; i++) rec[i] = 0;
        rec_ptr = 2;
      }
      else if (rec[rec_ptr - 2] == 'w' && rec[rec_ptr - 1] == 'r' && rec[rec_ptr] == 'k') {
        rec[0] = 'w'; rec[1] = 'r'; rec[2] = 'k';
        for (i = 3; i < sizeof(rec) - 1; i++) rec[i] = 0;
        rec_ptr = 2;
      }
      else if (rec[rec_ptr - 2] == 'v' && rec[rec_ptr - 1] == 'o' && rec[rec_ptr] == 'l') {
        rec[0] = 'v'; rec[1] = 'o'; rec[2] = 'l';
        for (i = 3; i < sizeof(rec) - 1; i++) rec[i] = 0;
        rec_ptr = 2;
      }
      else if (rec[rec_ptr - 2] == 'p' && rec[rec_ptr - 1] == 'r' && rec[rec_ptr] == '1') {
        rec[0] = 'p'; rec[1] = 'r'; rec[2] = '1';
        for (i = 3; i < sizeof(rec) - 1; i++) rec[i] = 0;
        rec_ptr = 2;
      }
      else if (rec[rec_ptr - 2] == 'p' && rec[rec_ptr - 1] == 'r' && rec[rec_ptr] == '2') {
        rec[0] = 'p'; rec[1] = 'r'; rec[2] = '2';
        for (i = 3; i < sizeof(rec) - 1; i++) rec[i] = 0;
        rec_ptr = 2;
      }
      else if (rec_ptr >= 127) {
        for (i = 0; i < sizeof(rec) - 1; i++) rec[i] = 0;
      }
    }

    // check if the end command received or not
    if (rec[rec_ptr - 2] == 'e' && rec[rec_ptr - 1] == 'n' && rec[rec_ptr] == 'd') {
      if (ps2_ctrl == 0) {
        if (rec[0] == 'v' && rec[1] == 'e' && rec[2] == 'l')      read_wheel_vel_cmd(rec, rec_ptr, omega_cmd_x10);
        else if (rec[0] == 'w' && rec[1] == 'r' && rec[2] == 'k') read_work_vel_cmd(rec, rec_ptr, work_vel_cmd_x10);
        else if (rec[0] == 'v' && rec[1] == 'o' && rec[2] == 'l') read_vol_cmd(rec, rec_ptr);
        else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '1') read_pr1_cmd(rec, rec_ptr);
        else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '2') read_pr2_cmd(rec, rec_ptr);
        else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '3') read_pr3_cmd(rec, rec_ptr);
        start_bit = 1;
        workspace_ctrl = 1;
      }
      for (i = 0; i < sizeof(rec) - 1; i++) rec[i] = 0;
    }

    // Increment the receive data pointer
    rec_ptr++;

    // Check start, stop charater
    if (a == 's') {
      print_label();
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      cnt_now[0] = cnt_dir[0] * enc.read_cnt(FR_ENC);
      cnt_now[1] = cnt_dir[1] * enc.read_cnt(FL_ENC);
      cnt_now[2] = cnt_dir[2] * enc.read_cnt(RR_ENC);
      cnt_now[3] = cnt_dir[3] * enc.read_cnt(RL_ENC);
      for (i = 0; i < 4; i++) cnt_pre[i] = cnt_now[i];
      start_bit = 1;
      workspace_ctrl = 0;
      ps2_ctrl = 0;
    }
    else if (a == 't') {
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      for (i = 0; i < 4; i++) int_e_omega[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf_tmp[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf[i] = 0;
      start_bit = 0;
      workspace_ctrl = 0;
      ps2_ctrl = 0;
      stop_all();
    }
    else if (a == 'f') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] += 1000;
        if (omega_cmd_x10[i] > 5000) omega_cmd_x10[i] = 5000;
      }
      workspace_ctrl = 0;
      ps2_ctrl = 0;
    }
    else if (a == 'g') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] -= 1000;
        if (omega_cmd_x10[i] < -5000) omega_cmd_x10[i] = -5000;
      }
      workspace_ctrl = 0;
      ps2_ctrl = 0;
    }
  }
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  uint8_t i;

  // Reading serial communication command --------------------------------------------------------//
  polling();

  // Reading PS2 Controller key ------------------------------------------------------------------//
  if (ps2_error == 1) { //skip loop if no controller found
    resetFunc();
  }

  if (ps2_type != 2) { //DualShock Controller
    ps2x.read_gamepad(false, ps2_vibrate); //read controller and set large motor to spin at 'vibrate' speed

    // if (ps2x.Button(PSB_START))
    // if (ps2x.Button(PSB_SELECT))
    // if (ps2x.Button(PSB_PAD_UP))
    // if (ps2x.Button(PSB_PAD_RIGHT))
    // if (ps2x.Button(PSB_PAD_LEFT))
    // if (ps2x.Button(PSB_PAD_DOWN))

    // if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    //   if (ps2x.Button(PSB_L3))
    //   if (ps2x.Button(PSB_R3))
    //   if (ps2x.Button(PSB_L2))
    //   if (ps2x.Button(PSB_R2))
    // }

    // if (ps2x.Button(PSB_TRIANGLE))
    // if (ps2x.ButtonPressed(PSB_CIRCLE))              //will be TRUE if button was JUST pressed
    // if (ps2x.NewButtonState(PSB_CROSS))              //will be TRUE if button was JUST pressed OR released
    // if (ps2x.ButtonReleased(PSB_SQUARE))             //will be TRUE if button was JUST released

    // print stick values if either is TRUE, Left stick, Y axis. Other options: LX, RY, RX
    // if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
    //   Serial.print(F("Stick Values:")); Serial.print(ps2x.Analog(PSS_LY), DEC);
    //   Serial.print(","); Serial.print(ps2x.Analog(PSS_LX), DEC);
    //   Serial.print(","); Serial.print(ps2x.Analog(PSS_RY), DEC);
    //   Serial.print(","); Serial.println(ps2x.Analog(PSS_RX), DEC);
    // }

    // Command to linear direction X
    if (ps2x.Button(PSB_PAD_UP)) {          // Go forward if the up is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[0] = 3000;
        else                         work_vel_cmd_x10[0] = 2000;
      } else                         work_vel_cmd_x10[0] = 1000;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else if (ps2x.Button(PSB_PAD_DOWN)) {   // Go back if the down is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[0] = -3000;
        else                         work_vel_cmd_x10[0] = -2000;
      } else                         work_vel_cmd_x10[0] = -1000;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else {
      if (ps2_ctrl == 1) {
        work_vel_cmd_x10[0] = 0;
      }
    }

    // Command to linear direction Y
    if (ps2x.Button(PSB_PAD_RIGHT)) {  // Go right if the right is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[1] = -3000;
        else                         work_vel_cmd_x10[1] = -2000;
      } else                         work_vel_cmd_x10[1] = -1000;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else if (ps2x.Button(PSB_PAD_LEFT)) {   // Go left if the left is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[1] = 3000;
        else                         work_vel_cmd_x10[1] = 2000;
      } else                         work_vel_cmd_x10[1] = 1000;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else {
      if (ps2_ctrl == 1) {
        work_vel_cmd_x10[1] = 0;
      }
    }

    // Command to rotational direction
    if (ps2x.Button(PSB_L1)) {         // Turn left if the L1 is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[2] = 900;
        else                         work_vel_cmd_x10[2] = 600;
      } else                         work_vel_cmd_x10[2] = 300;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else if (ps2x.Button(PSB_R1)) {         // Turn right if the R1 is pressed
      if (ps2x.Button(PSB_CROSS)) {
        if (ps2x.Button(PSB_SQUARE)) work_vel_cmd_x10[2] = -900;
        else                         work_vel_cmd_x10[2] = -600;
      } else                         work_vel_cmd_x10[2] = -300;
      start_bit = 1;
      workspace_ctrl = 1;
      ps2_ctrl = 1;
    }
    else {
      if (ps2_ctrl == 1) {
        work_vel_cmd_x10[2] = 0;
      }
    }

    // Stop all
    if (ps2x.Button(PSB_START)) {
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      for (i = 0; i < 4; i++) int_e_omega[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf_tmp[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf[i] = 0;
      start_bit = 0;
      workspace_ctrl = 0;
      ps2_ctrl = 0;
      stop_all();
    }
  }
}
