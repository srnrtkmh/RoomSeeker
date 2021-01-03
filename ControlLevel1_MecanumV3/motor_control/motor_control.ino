//================================================================================================//
//                                                                                                //
// FILE : motor_control.ino                                                                       //
// MEMO : Control 4 motors velocity by PI control                                                 //
//        Added feedforward control with friction compensation                                    //
//                                                                                                //
// Update Log                                                                                     //
//   2020/11/03 : Start this project                                                              //
//                Rotary Encoder object is applied from MecanumV2                                 //
//                Only constant voltage output                                                    //
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
#include <avr/interrupt.h>
#include <TimerOne.h>
#include "rotary_encoders.h"

// #define DEBUG

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
long g_omega[4] = {30, 30, 30, 30};                 // Cutoff angular frequency [rad/sec]
int16_t Kp[4] = {4, 4, 4, 4};                       // P gain for PI control
int16_t Ki[4] = {20, 20, 20, 20};                   // I gain for PI control

// Feedforward parameters
int16_t Fc_p[4] = { 785,  691,  699,  687};         // Coulonb friction compensation parameter for positive direction
int16_t Fc_n[4] = { -778, -692, -720, -699};        // Coulonb friction compensation parameter for negative direction
int16_t Fd_p_x100[4] = {141, 173, 158, 149};        // Dynamic friction compensation parameter for negative direction
int16_t Fd_n_x100[4] = {148, 176, 154, 149};        // Dynamic friction compensation parameter for negative direction

// Useful constants for calculation
const long sampling_time = 10000;                   // サンプリング時間[us]
const long dt_x1000 = 10;                           // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
const long _dt_x10 = 1000;                          // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                             // Pi x 100

// Sample number
long n = 0;                                         // サンプルカウンタ変数

// For sequence
byte start_bit = 0;                                 // サンプリング開始ビット

//================================================================================================//
// go_advance(int speed) --- drive all motors with the same PWM value                             //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void go_advance(int speed) {
  FR_fwd(speed);
  FL_fwd(speed);
  RR_fwd(speed);
  RL_fwd(speed);
}

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
// stop_all_motor(void) --- Stop all motors                                                       //
//================================================================================================//
void stop_all_motor() {
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
  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3]\n");

  // For debug
  //  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], enc.read_test_cnt\n");
#endif
}

//================================================================================================//
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  char str[64];
  sprintf(str, "%3d,", (int16_t)n);
  Serial.print(str);
  sprintf(str, "%+6d,%+6d,%+6d,%+6d,", (uint16_t)cnt_now[0], (uint16_t)cnt_now[1], (uint16_t)cnt_now[2], (uint16_t)cnt_now[3]);
  Serial.print(str);
  sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)omega_res_x10[0], (int16_t)omega_res_x10[1], (int16_t)omega_res_x10[2], (int16_t)omega_res_x10[3]);
  Serial.print(str);
  // sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)omega_res_x10_lpf[0], (int16_t)omega_res_x10_lpf[1], (int16_t)omega_res_x10_lpf[2], (int16_t)omega_res_x10_lpf[3]);
  // Serial.print(str);
  sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)omega_cmd_x10[0], (int16_t)omega_cmd_x10[1], (int16_t)omega_cmd_x10[2], (int16_t)omega_cmd_x10[3]);
  Serial.print(str);
  sprintf(str, "%+5d,%+5d,%+5d,%+5d", (int16_t)vout[0], (int16_t)vout[1], (int16_t)vout[2], (int16_t)vout[3]);
  Serial.print(str);
  // sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)vout_ff[0], (int16_t)vout_ff[1], (int16_t)vout_ff[2], (int16_t)vout_ff[3]);
  // Serial.print(str);
  // sprintf(str, "%+5d,%+5d,%+5d,%+5d,", (int16_t)vout_fb[0], (int16_t)vout_fb[1], (int16_t)vout_fb[2], (int16_t)vout_fb[3]);
  // Serial.print(str);
  // Serial.print(enc.read_test_cnt());
  // sprintf(str, "%+5d,%+5d,%+5d", (int16_t)work_vel_cmd_x10[0], (int16_t)work_vel_cmd_x10[1], (int16_t)work_vel_cmd_x10[2]);
  // Serial.print(str);
  Serial.print("\n");
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  int i;
  long tmp;
  interrupts();                   // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    // Update encoder counter value
    for (i = 0; i < 4; i++) cnt_pre[i] = cnt_now[i];
    noInterrupts();
    cnt_now[0] = cnt_dir[0] * enc.read_cnt(FR_ENC);
    cnt_now[1] = cnt_dir[1] * enc.read_cnt(FL_ENC);
    cnt_now[2] = cnt_dir[2] * enc.read_cnt(RR_ENC);
    cnt_now[3] = cnt_dir[3] * enc.read_cnt(RL_ENC);
    interrupts();

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
    }

    // Apply control output to the motor
    // Front Right motor
    if (vout[0] > 0) FR_fwd(vout[0]);
    else             FR_bck(-vout[0]);

    // Front Left motor
    if (vout[1] > 0) FL_fwd(vout[1]);
    else             FL_bck(-vout[1]);

    // Rear Right motor
    if (vout[2] > 0) RR_fwd(vout[2]);
    else             RR_bck(-vout[2]);

    // Rear Left motor
    if (vout[3] > 0) RL_fwd(vout[3]);
    else             RL_bck(-vout[3]);

    print_data();           // Output log data to upper system
    if (++n == 1000) n = 0; // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }
}

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
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

  stop_all_motor();
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  gpio_init();                      // Initialize GPIO
  Serial.begin (230400);            // Initialize Serial

  // Setup of encoder object
  enc.attach(0, FR_ENC_A, FR_ENC_B, DUAL_MULTI);
  enc.attach(1, FL_ENC_A, FL_ENC_B, DUAL_MULTI);
  enc.attach(2, RR_ENC_A, RR_ENC_B, DUAL_MULTI);
  enc.attach(3, RL_ENC_A, RL_ENC_B, DUAL_MULTI);

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
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;
  uint8_t i;

  // Reading serial communication command --------------------------------------------------------//
  if (Serial.available()) {
    a = char(Serial.read());

    if (a == 's') {
      print_label();
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      cnt_now[0] = cnt_dir[0] * enc.read_cnt(FR_ENC);
      cnt_now[1] = cnt_dir[1] * enc.read_cnt(FL_ENC);
      cnt_now[2] = cnt_dir[2] * enc.read_cnt(RR_ENC);
      cnt_now[3] = cnt_dir[3] * enc.read_cnt(RL_ENC);
      for (i = 0; i < 4; i++) cnt_pre[i] = cnt_now[i];
      start_bit = 1;
    }
    else if (a == 't') {  
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      for (i = 0; i < 4; i++) int_e_omega[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf_tmp[i] = 0;
      for (i = 0; i < 4; i++) omega_res_x10_lpf[i] = 0;
      start_bit = 0;
      stop_all_motor();
    }
    else if (a == 'u') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] += 1000;
        if (omega_cmd_x10[i] > 7000) omega_cmd_x10[i] = 7000;
      }
    }
    else if (a == 'd') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] -= 1000;
        if (omega_cmd_x10[i] < -7000) omega_cmd_x10[i] = -7000;
      }
    }
  }
}
