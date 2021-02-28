//================================================================================================//
//                                                                                                //
// FILE : motor_control_node.ino                                                                  //
// MEMO : 2 motors velocity by PI control                                                         //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/10 : Start this project based on motor_control.ino                                   //
//   2021/01/15 : Modified sampling time to 20ms -> 7ms                                           //
//   2021/01/17 : If stop command received, PWM timer stop to completely off PWM                  //
//                Clean the code e.g. separate functions, add comments                            //
//                Change upper velocity value 5000 -> 10000                                       //
//                Added votage command mode                                                       //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 - UART RX                                  1 - UART TX                                    //
//    2 - M0-IN1 L298N                             3 - M0-IN2 L298N                               //
//    4 - M1-IN3 L298N                             5 - M1-IN4 L298N                               //
//    6 - NC                                       7 - NC                                         //
//    8 - NC                                       9 - M0ENA L298N ENA (OC1A)                     //
//   10 - M1ENB L298N ENB (OC1A)                  11 - NC                                         //
//   12 - NC                                      13 - NC                                         //
//   14 - A0 ENC0AF encoder phase A               15 - A1 ENC0BF encoder phase B                  //
//   16 - A2 ENC1AF encoder phase A               17 - A3 ENC1BF encoder phase B                  //
//   18 - NC                                      19 - NC                                         //
//   20 - NC                                      21 - NC                                         //
//                                                                                                //
//                                         Copyright (c) 2021 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <avr/interrupt.h>
#include <MsTimer2.h>
#include <avr/wdt.h>
#include "rotary_encoders.h"
#include "command.h"

//#define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
#define N_MOT 2                             // The number of motor
#define M0_ENC_A 14                         // Input pin number of Front Right wheel encoder phase A
#define M0_ENC_B 15                         // Input pin number of Front Right wheel encoder phase B
#define M1_ENC_A 16                         // Input pin number of Front Left wheel encoder phase A
#define M1_ENC_B 17                         // Input pin number of Front Left wheel encoder phase B
#define M0_ENC 0                            // Index for Front Right wheel encoder
#define M1_ENC 1                            // Index for Front Left wheel encoder
#define ENC_RESO 44                         // Encoder resolusion 11 pulse dual edge count mode
#define GEAR_REDU 30                        // Gear reduction
RotaryEncoders enc;

// For Motor
#define M0_PWM_PIN  9                       // Front Right Motor PWM pin
#define M0_DIR_PIN1 2                       // Front Right Motor direction pin 1
#define M0_DIR_PIN2 3                       // Front Right Motor direction pin 2
#define M1_PWM_PIN 10                       // Front Left Motor PWM pin
#define M1_DIR_PIN1 4                       // Front Left Motor direction pin 1
#define M1_DIR_PIN2 5                       // Front Left Motor direction pin 2
#define M0 0                                // Front Right Motor index
#define M1 1                                // Front Left Motor index

// Control variables
uint8_t mode = 0;                           // Control of motor
const uint8_t VEL_CTRL = 0;                 // Velocity control mode
const uint8_t VOL_CTRL = 1;                 // Voltage control mode
int8_t mot_dir[N_MOT] = { -1, -1};          // Direction of motor rotation
int8_t cnt_dir[N_MOT] = { 1, 1};            // Direction of encoder rotation
uint16_t cnt_now[N_MOT] = {0, 0};           // Current encoder counter for 4 motors [-]
uint16_t cnt_pre[N_MOT] = {0, 0};           // Previous encoder counter for 4 motors [-]
int16_t diff_cnt[N_MOT] = {0, 0};           // Difference between current and previous counter
long omega_res_x10[N_MOT] = {0, 0};         // Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf[N_MOT] = {0, 0};     // LPF output of Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf_tmp[N_MOT] = {0, 0}; // Temporary for LPF
long omega_cmd_x10[N_MOT] = {0, 0};         // Rotation speed command [10^-3 deg/sec]
long e_omega[N_MOT] = {0, 0};               // Rotation speed error
long int_e_omega[N_MOT] = {0, 0};           // Integral of rotation speed error
int16_t vout[N_MOT] = {0, 0};               // Voltage output for 4 motor drivers - index 1, 2, 4 : [0 - 1023], index3 : [0 - 255]
int16_t vout_ff[N_MOT] = {0, 0};            // Feedforward control output
int16_t vout_fb[N_MOT] = {0, 0};            // Feedback control output
int16_t vout_ll[N_MOT] = { -1000, -1000};   // Lower limit of voltage output
int16_t vout_ul[N_MOT] = { 1000,  1000};    // Upper limit of voltage output
const long omega_cmd_x10_ll[N_MOT] = { -10000, -10000}; // Lower limit of velocity
const long omega_cmd_x10_ul[N_MOT] = { 10000,  10000};  // Upper limit of velocity
const long omega_cmd_x10_inc = 1000;                    // Increment value of velocity command

int16_t vol_cmd[N_MOT] = {0, 0};                        // Voltage command
const int16_t vol_cmd_ll[N_MOT] = { -1000, -1000};      // Lower limit of voltage command
const int16_t vol_cmd_ul[N_MOT] = { 1000,  1000};       // Upper limit of voltage command
const int16_t vol_cmd_inc = 100;                        // Increment value of velocity command

// Feecback parameters
long g_omega[N_MOT] = {60, 60};             // Cutoff angular frequency [rad/sec]
int16_t Kp[N_MOT] = {3, 3};                 // P gain for PI control
// int16_t Ki[N_MOT] = {20, 20};            // I gain for PI control
int16_t Ki[N_MOT] = {30, 30};               // I gain for PI control

// Feedforward parameters
// int16_t Fc_p[N_MOT] = {  785,  691};     // Coulonb friction compensation parameter for positive direction
// int16_t Fc_n[N_MOT] = { -778, -692};     // Coulonb friction compensation parameter for negative direction
// int16_t Fc_p[N_MOT] = {  392,  345};     // Coulonb friction compensation parameter for positive direction
// int16_t Fc_n[N_MOT] = { -389, -346};     // Coulonb friction compensation parameter for negative direction
int16_t Fc_p[N_MOT] = {  0,  0};         // Coulonb friction compensation parameter for positive direction
int16_t Fc_n[N_MOT] = { -0, -0};         // Coulonb friction compensation parameter for negative direction
int16_t Fd_p_x100[N_MOT] = {141, 173};   // Dynamic friction compensation parameter for negative direction
int16_t Fd_n_x100[N_MOT] = {148, 176};   // Dynamic friction compensation parameter for negative direction
// int16_t Fd_p_x100[N_MOT] = { 50, 50};    // Dynamic friction compensation parameter for negative direction
// int16_t Fd_n_x100[N_MOT] = { 50, 50};    // Dynamic friction compensation parameter for negative direction

// Useful constants for calculation
long n = 0;                                           // Sample counter variable
const long sampling_time_us = 7000;                   // Sampling time[us]
const long setMsTimer2 = sampling_time_us / 1000;     // Set value for MsTimer2
const long dt_x1000 = sampling_time_us / 1000;        // (sampling_time_us / 1,000,000) * 1000 [10^-3 sec]
const long _dt_x10 = 1000000 / sampling_time_us * 10; // (sampling_time_us / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                               // Pi x 100
long start_time = 0, stop_time = 0, interval = 0;     //

// For sequence
byte start_bit = 0;                                 // Periodic process is enabled or not

//================================================================================================//
// M0_fwd(int speed), M0_bck(int speed) --- this is the same for M1                               //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void M0_fwd(int speed) {
  if (mot_dir[M0] == 1) {
    digitalWrite(M0_DIR_PIN1, HIGH);
    digitalWrite(M0_DIR_PIN2, LOW);
  } else {
    digitalWrite(M0_DIR_PIN1, LOW);
    digitalWrite(M0_DIR_PIN2, HIGH);
  }
  OCR1A = speed;
}

void M0_bck(int speed) {
  if (mot_dir[M0] == 1) {
    digitalWrite(M0_DIR_PIN1, LOW);
    digitalWrite(M0_DIR_PIN2, HIGH);
  } else {
    digitalWrite(M0_DIR_PIN1, HIGH);
    digitalWrite(M0_DIR_PIN2, LOW);
  }
  OCR1A = speed;
}

void M1_fwd(int speed) {
  if (mot_dir[M1] == 1) {
    digitalWrite(M1_DIR_PIN1, HIGH);
    digitalWrite(M1_DIR_PIN2, LOW);
  } else {
    digitalWrite(M1_DIR_PIN1, LOW);
    digitalWrite(M1_DIR_PIN2, HIGH);
  }
  OCR1B = speed;
}

void M1_bck(int speed) {
  if (mot_dir[M1] == 1) {
    digitalWrite(M1_DIR_PIN1, LOW);
    digitalWrite(M1_DIR_PIN2, HIGH);
  } else {
    digitalWrite(M1_DIR_PIN1, HIGH);
    digitalWrite(M1_DIR_PIN2, LOW);
  }
  OCR1B = speed;
}

//================================================================================================//
// stop_all(void) --- Stop all motors                                                             //
//================================================================================================//
void stop_all() {
  OCR1A = 0;
  OCR1B = 0;
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
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  char str[64];
  sprintf(str, "%3d,", (int16_t)n);
  Serial.print(str);
  sprintf(str, "%+6d,%+6d,", (uint16_t)cnt_now[0], (uint16_t)cnt_now[1]);
  Serial.print(str);
  sprintf(str, "%+5d,%+5d,", (int16_t)omega_res_x10[0], (int16_t)omega_res_x10[1]);
  Serial.print(str);
  sprintf(str, "%+5d,%+5d,", (int16_t)omega_cmd_x10[0], (int16_t)omega_cmd_x10[1]);
  Serial.print(str);
  sprintf(str, "%+5d,%+5d,", (int16_t)vout[0], (int16_t)vout[1]);
  Serial.print(str);
  sprintf(str, "%6d", (uint16_t)interval);
  Serial.print(str);
  Serial.print("\n");
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  int i;
  long tmp;

  start_time = micros();  // Get starting time of periodic loop
  interrupts();           // Enable interrupts
  wdt_reset();            // Reset WDT

  // Update encoder counter value
  for (i = 0; i < N_MOT; i++) cnt_pre[i] = cnt_now[i];
  noInterrupts();
  for (i = 0; i < N_MOT; i++) cnt_now[i] = cnt_dir[i] * enc.read_cnt(i);
  interrupts();

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    // Calculate difference between current and previous counter value
    for (i = 0; i < N_MOT; i++) {
      tmp = cnt_now[i] - cnt_pre[i];
      if (tmp > 32767L)       diff_cnt[i] = tmp - 65536L;
      else if (tmp < -32768L) diff_cnt[i] = tmp + 65536L;
      else                    diff_cnt[i] = tmp;
    }

    // Calculate rotation speed
    for (i = 0; i < N_MOT; i++) {
      omega_res_x10[i] = (diff_cnt[i] * 360) * _dt_x10 / ENC_RESO / GEAR_REDU;
      omega_res_x10_lpf_tmp[i] += (omega_res_x10[i] - omega_res_x10_lpf[i]) * dt_x1000;
      omega_res_x10_lpf[i] = omega_res_x10_lpf_tmp[i] * g_omega[i] / 1000;
    }

    if (mode == VEL_CTRL) {
      // Calculate control output
      for (i = 0; i < N_MOT; i++) {
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
    }
    else  if (mode == VOL_CTRL) {
      for (i = 0; i < N_MOT; i++) {
        vout[i] = vol_cmd[i];

        // Apply saturation
        if (vout[i] > vout_ul[i]) vout[i] = vout_ul[i];
        else if (vout[i] < vout_ll[i]) vout[i] = vout_ll[i];
      }
    }

    // Apply control output to the motor
    if (vout[0] > 0) M0_fwd(vout[0]);
    else             M0_bck(-vout[0]);
    if (vout[1] > 0) M1_fwd(vout[1]);
    else             M1_bck(-vout[1]);

    print_data();           // Output log data to upper system
    if (++n == 1000) n = 0; // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < N_MOT; i++) vout[i] = 0;
  }
  stop_time = micros();
  interval = stop_time - start_time;
}

//================================================================================================//
// void start_pwm_timer(void)                                                                     //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void start_pwm_timer(void) {
  OCR1A = 0;                          // Clear OCR1A
  OCR1B = 0;                          // Clear OCR1B
  TCNT1 = 0;                          // Clear timer 1
  TIMSK1 = 0x00;                      // None interrupts
  TCCR1A = 0xA3;                      // OC1A, OC1B is none inverting mode, PWM mode is Fast PWM 10bit
  TCCR1B = 0x0A;                      // PWM mode is Fast PWM, Clock is clkI/O/1024 (From prescaler)
  TCCR1C = 0x00;                      // Force compare match is disabled
}

//================================================================================================//
// void stop_pwm_timer(void)                                                                      //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void stop_pwm_timer(void) {
  OCR1A = 0;                          // Clear OCR1A
  OCR1B = 0;                          // Clear OCR1B
  TCNT1 = 0;                          // Clear timer 1
  TIMSK1 = 0x00;                      // None interrupts
  TCCR1A = 0x00;                      // Clear TCCR1A
  TCCR1B = 0x00;                      // Clear TCCR1B
  TCCR1C = 0x00;                      // Clear TCCR1C
}

//================================================================================================//
// void start_routine(void)                                                                       //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void start_routine() {
  uint8_t i;
  print_label();
  start_bit = 1;
  for (i = 0; i < N_MOT; i++) {
    omega_cmd_x10[i] = 0;
    int_e_omega[i] = 0;
    omega_res_x10_lpf_tmp[i] = 0;
    omega_res_x10_lpf[i] = 0;
    cnt_now[i] = cnt_dir[i] * enc.read_cnt(i);
    cnt_pre[i] = cnt_now[i];
    vol_cmd[i] = 0;
  }
  start_pwm_timer();
}

//================================================================================================//
// void stop_routine(void)                                                                        //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void stop_routine() {
  start_bit = 0;
  stop_pwm_timer();
}

//================================================================================================//
// void check_header(char *head, char *str, uint8_t *bot)                                         //
//   Arguments : char *head   - Header string (3 chars) to compare                                //
//               char *str    - Received data string                                              //
//               uint8_t *bot - The pointer to access the bottom of str                           //
//   Return    : none                                                                             //
//================================================================================================//
void check_header(const char *head, char *str, uint8_t *bot) {
  if (str[*bot - 2] == head[0] && str[*bot - 1] == head[1] && str[*bot] == head[2]) {
    strcpy(str, head);
    *bot = 2;
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin (115200);    // Initialize Serial
  delay(100);               // added delay to give wireless ps2 module some time to startup, before configuring it

  // Initialize GPIO
  // Motor control port setting
  pinMode(M0_DIR_PIN1, OUTPUT);
  pinMode(M0_DIR_PIN2, OUTPUT);
  pinMode(M0_PWM_PIN, OUTPUT);

  pinMode(M1_DIR_PIN1, OUTPUT);
  pinMode(M1_DIR_PIN2, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  stop_all();

  // Setup of encoder object
  enc.attach(0, M0_ENC_A, M0_ENC_B, QUAD_MULTI);
  enc.attach(1, M1_ENC_A, M1_ENC_B, QUAD_MULTI);

  // Timer 1 PWM output compare settings
  start_pwm_timer();

  // Timer2割込の設定
  MsTimer2::set(setMsTimer2, flash);  // Sampling time settings
  MsTimer2::start();                  // Start timer2

  // Enable watch dog timer
  wdt_enable(WDTO_1S);
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;
  uint8_t i;
  static char rec[128];
  static uint8_t rec_ptr = 0;     // Received serial data pointer

  const char VEL[4] = "vel";
  const char VOL[4] = "vol";
  const char PR1[4] = "pr1";
  const char PR2[4] = "pr2";
  const char PR3[4] = "pr3";

  while (Serial.available()) {
    a = char(Serial.read());
    rec[rec_ptr] = a;

    // Command reading process -------------------------------------------------------------------//
    // check if the start command received or not
    if (rec_ptr >= 2) {
      check_header(VEL, rec, &rec_ptr);
      check_header(VOL, rec, &rec_ptr);
      check_header(PR1, rec, &rec_ptr);
      check_header(PR2, rec, &rec_ptr);
      check_header(PR3, rec, &rec_ptr);
    }

    // check if the end command received or not
    if (rec[rec_ptr - 2] == 'e' && rec[rec_ptr - 1] == 'n' && rec[rec_ptr] == 'd') {
      // Serial.print(rec);
      // Serial.print("\n");
      if (rec[0] == 'v' && rec[1] == 'e' && rec[2] == 'l')      read_wheel_vel_cmd(rec, rec_ptr, omega_cmd_x10);
      else if (rec[0] == 'v' && rec[1] == 'o' && rec[2] == 'l') read_vol_cmd(rec, rec_ptr, vol_cmd);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '1') read_pr1_cmd(rec, rec_ptr);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '2') read_pr2_cmd(rec, rec_ptr);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '3') read_pr3_cmd(rec, rec_ptr);
      mode = VEL_CTRL;
      start_bit = 1;
      start_pwm_timer();
      for (i = 0; i < sizeof(rec) - 1; i++) rec[i] = 0;
    }

    // Increment the receive data pointer
    if (++rec_ptr == 128) rec_ptr = 0;

    // Single character command reading ----------------------------------------------------------//
    // Check start, stop charater
    if (a == 's') {
      mode = VEL_CTRL;
      start_routine();
    }
    else if (a == 't') {
      stop_routine();
    }
    else if (a == 'f') {
      mode = VEL_CTRL;
      for (i = 0; i < N_MOT; i++) {
        omega_cmd_x10[i] += omega_cmd_x10_inc;
        if (omega_cmd_x10[i] > omega_cmd_x10_ul[i]) omega_cmd_x10[i] = omega_cmd_x10_ul[i];
      }
    }
    else if (a == 'g') {
      mode = VEL_CTRL;
      for (i = 0; i < N_MOT; i++) {
        omega_cmd_x10[i] -= omega_cmd_x10_inc;
        if (omega_cmd_x10[i] < omega_cmd_x10_ll[i]) omega_cmd_x10[i] = omega_cmd_x10_ll[i];
      }
    }
    else if (a == 'h') {
      mode = VOL_CTRL;
      for (i = 0; i < N_MOT; i++) {
        vol_cmd[i] += vol_cmd_inc;
        if (vout[i] > vol_cmd_ul[i]) vout[i] = vol_cmd_ul[i];
      }
    }
    else if (a == 'j') {
      mode = VOL_CTRL;
      for (i = 0; i < N_MOT; i++) {
        vol_cmd[i] -= vol_cmd_inc;
        if (vout[i] < vol_cmd_ll[i]) vout[i] = vol_cmd_ll[i];
      }
    }
  }
}
