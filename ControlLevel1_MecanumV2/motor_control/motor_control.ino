//================================================================================================//
//                                                                                                //
// FILE : motor_control_with_PS2.ino                                                              //
// MEMO : Control 4 motors velocity by PI control                                                 //
//        Added feedforward control with friction compensation                                    //
//                                                                                                //
// Update Log                                                                                     //
//   2020/09/19 : Start this project based on previous project "motor_control.ino"                //
//                Add workspace command                                                           //
//   2020/09/20 : Reduce memory usage usin F Macro                                                //
//                Optimise PS2 communication delaying                                             //
//                Change encoder reading method quad multiply to dual multiply                    //
//   2020/10/31 : Changed friction conpensation parameter got from identification test            //
//                                                                                                //
// Pin Assign                                                                                     //
//    3 - Front Right encoder phase A              2 - Front Right encoder phase B                //
//    6 - Front Left encoder phase A               4 - Front Left encoder phase B                 //
//    5 - Rear Right encoder phase A               7 - Rear Right encoder phase B                 //
//    8 - Rear Left encoder phase A                9 - Rear Left encoder phase B                  //
//                                                                                                //
//   13 - PS2_DAT (DO for PS2 controller)         11 - PS2_CMD (DO for PS2 controller)            //
//   10 - PS2_SEL (DO for PS2 controller)         12 - PS2_CLK (DO for PS2 controller)            //
//                                                                                                //
//   18 - SDA (I2C communication for PCA9685PW)   19 - SCL (I2C communication for PCA9685PW)      //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <avr/interrupt.h>
#include <TimerOne.h>
#include "rotary_encoders_uno.h"
#include "FaBoPWM_PCA9685.h"
#include "PS2X_lib.h"

//#define DEBUG

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
PROGMEM const uint8_t FR_ENC_A = 3;   // Input pin number of Front Right wheel encoder phase A
PROGMEM const uint8_t FR_ENC_B = 2;   // Input pin number of Front Right wheel encoder phase B
PROGMEM const uint8_t FL_ENC_A = 6;   // Input pin number of Front Left wheel encoder phase A
PROGMEM const uint8_t FL_ENC_B = 4;   // Input pin number of Front Left wheel encoder phase B
PROGMEM const uint8_t RR_ENC_A = 5;   // Input pin number of Rear Right wheel encoder phase A
PROGMEM const uint8_t RR_ENC_B = 7;   // Input pin number of Rear Right wheel encoder phase B
PROGMEM const uint8_t RL_ENC_A = 8;   // Input pin number of Rear Left wheel encoder phase A
PROGMEM const uint8_t RL_ENC_B = 9;   // Input pin number of Rear Left wheel encoder phase B
PROGMEM const uint8_t FR_ENC = 0;     // Index for Front Right wheel encoder
PROGMEM const uint8_t FL_ENC = 1;     // Index for Front Left wheel encoder
PROGMEM const uint8_t RR_ENC = 2;     // Index for Rear Right wheel encoder
PROGMEM const uint8_t RL_ENC = 3;     // Index for Rear Left wheel encoder
PROGMEM const uint8_t ENC_RESO = 22;  // Encoder resolusion 11 pulse dual edge count mode
PROGMEM const uint8_t GEAR_REDU = 30; // Gear reduction
RotaryEncoders enc;

// For Motor
PROGMEM const uint8_t FR1 = 6;        // Front Right wheel index
PROGMEM const uint8_t FR2 = 7;        // Front Right wheel index
PROGMEM const uint8_t FL1 = 5;        // Front Left wheel index
PROGMEM const uint8_t FL2 = 4;        // Front Left wheel index
PROGMEM const uint8_t RR1 = 2;        // Rear Right wheel index
PROGMEM const uint8_t RR2 = 3;        // Rear Right wheel index
PROGMEM const uint8_t RL1 = 1;        // Rear Left wheel index
PROGMEM const uint8_t RL2 = 0;        // Rear Left wheel index
FaBoPWM faboPWM;                      // PWM controller object

// Control variables
int8_t cnt_dir[4] = {1, -1, 1, -1};                 // Direction correction co-efficient of rotation
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
int16_t vout_ll[4] = { -4095, -4095, -4095, -4095}; // Lower limit of voltage output
int16_t vout_ul[4] = { 4095,  4095,  4095,  4095};  // Upper limit of voltage output

// Feecback parameters
int16_t g_omega[4] = {60, 60, 60, 60};              // Cutoff angular frequency [rad/sec]
int16_t Kp[4] = {12, 12, 12, 12};                   // P gain for PI control
int16_t Ki[4] = {40, 40, 40, 40};                   // I gain for PI control

// Feedforward parameters
int16_t Fc_p[4] = { 785,  691,  699,  687};         // Coulonb friction compensation parameter for positive direction
int16_t Fc_n[4] = {-778, -692, -720, -699};         // Coulonb friction compensation parameter for negative direction
int16_t Fd_p_x100[4] = {141, 173, 158, 149};        // Dynamic friction compensation parameter for negative direction
int16_t Fd_n_x100[4] = {148, 176, 154, 149};        // Dynamic friction compensation parameter for negative direction

// Useful constants for calculation
const long sampling_time = 20000;                   // sampling time [us]
const long dt_x1000 = 20;                           // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
const long _dt_x10 = 500;                           // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                             // Pi x 100

// Chassis parameter
long work_vel_cmd_x10[3] = {0, 0, 0};               // Workspace control command
uint8_t workspace_ctrl = 0;                         // Workspace control flag
const int16_t W = 215;                              // Tread width
const int16_t L = 162;                              // Wheel base
const int16_t R = 40;                               // Wheel Radius

// Sample number
long n = 0;                                         // サンプルカウンタ変数

// For sequence
byte start_bit = 0;                                 // サンプリング開始ビット
char rec[128] = {0};                                // Received serial characters
uint8_t rec_ptr = 0;                                // Received serial pointer
uint8_t rec_vel_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_vol_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr1_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr2_flag = 0;                           // The flag specifing the received data is velocity command
uint8_t rec_pr3_flag = 0;                           // The flag specifing the received data is velocity command

// For PS2
PROGMEM const uint8_t PS2_DAT = 13;                 // DAT pin settings
PROGMEM const uint8_t PS2_CMD = 11;                 // CMD pin settings
PROGMEM const uint8_t PS2_SEL = 10;                 // SEL pin settings
PROGMEM const uint8_t PS2_CLK = 12;                 // CLK pin settings
PROGMEM const bool pressures = true;                //
PROGMEM const bool rumble    = true;                //
byte ps2_error = 0;                                 //
byte ps2_type = 0;                                  //
byte ps2_vibrate = 0;                               //
uint8_t ps2_ctrl = 0;                               // This flag shows command from PS2 controller is enable or not
PS2X ps2x;                                          // create PS2 Controller Object

// Reset func
void (* resetFunc) (void) = 0;                      // 

//================================================================================================//
// FR_fwd(int speed), FR_bck(int speed) --- this is the same for FL, RR, RL motor                 //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void FR_fwd(int speed) {
  faboPWM.set_channel_value(FR1, speed);
  faboPWM.set_channel_value(FR2, 0);
}
void FR_bck(int speed) {
  faboPWM.set_channel_value(FR1, 0);
  faboPWM.set_channel_value(FR2, speed);
}
void FL_fwd(int speed) {
  faboPWM.set_channel_value(FL1, speed);
  faboPWM.set_channel_value(FL2, 0);
}
void FL_bck(int speed) {
  faboPWM.set_channel_value(FL1, 0);
  faboPWM.set_channel_value(FL2, speed);
}
void RR_fwd(int speed) {
  faboPWM.set_channel_value(RR1, speed);
  faboPWM.set_channel_value(RR2, 0);
}
void RR_bck(int speed) {
  faboPWM.set_channel_value(RR1, 0);
  faboPWM.set_channel_value(RR2, speed);
}
void RL_fwd(int speed) {
  faboPWM.set_channel_value(RL1, speed);
  faboPWM.set_channel_value(RL2, 0);
}
void RL_bck(int speed) {
  faboPWM.set_channel_value(RL1, 0);
  faboPWM.set_channel_value(RL2, speed);
}

//================================================================================================//
// stop_all(void) --- Stop all motors                                                             //
//   Argument : none                                                                              //
//   Return   : none                                                                              //
//================================================================================================//
void stop_all(void) {
  FR_fwd(0);
  FL_fwd(0);
  RR_fwd(0);
  RL_fwd(0);
}

//================================================================================================//
// Wheel velocity command                                                                         //
//   Argument : *str - received character array, ptr - the last pointer of the character array    //
//   Return   : none                                                                              //
//================================================================================================//
void read_wheel_vel_cmd(char *str, uint8_t ptr) {
  if (ptr == 34) {
    if (str[0] == 'v' && str[1] == 'e' && str[2] == 'l') {
      if (str[32] == 'e' && str[33] == 'n' && str[34] == 'd') {
        if (str[3] == ',' && str[7] == ',' && str[13] == ',' && str[19] == ',' && str[25] == ',' && str[31] == ',') {
          if (str[8] == '+')       omega_cmd_x10[0] =   ((int16_t)str[ 9] - 48) * 1000 + ((int16_t)str[10] - 48) * 100 + ((int16_t)str[11] - 48) * 10 + ((int16_t)str[12] - 48);
          else if (str[8] == '-')  omega_cmd_x10[0] = -(((int16_t)str[ 9] - 48) * 1000 + ((int16_t)str[10] - 48) * 100 + ((int16_t)str[11] - 48) * 10 + ((int16_t)str[12] - 48));
          if (str[14] == '+')      omega_cmd_x10[1] =   ((int16_t)str[15] - 48) * 1000 + ((int16_t)str[16] - 48) * 100 + ((int16_t)str[17] - 48) * 10 + ((int16_t)str[18] - 48);
          else if (str[14] == '-') omega_cmd_x10[1] = -(((int16_t)str[15] - 48) * 1000 + ((int16_t)str[16] - 48) * 100 + ((int16_t)str[17] - 48) * 10 + ((int16_t)str[18] - 48));
          if (str[20] == '+')      omega_cmd_x10[2] =   ((int16_t)str[21] - 48) * 1000 + ((int16_t)str[22] - 48) * 100 + ((int16_t)str[23] - 48) * 10 + ((int16_t)str[24] - 48);
          else if (str[20] == '-') omega_cmd_x10[2] = -(((int16_t)str[21] - 48) * 1000 + ((int16_t)str[22] - 48) * 100 + ((int16_t)str[23] - 48) * 10 + ((int16_t)str[24] - 48));
          if (str[26] == '+')      omega_cmd_x10[3] =   ((int16_t)str[27] - 48) * 1000 + ((int16_t)str[28] - 48) * 100 + ((int16_t)str[29] - 48) * 10 + ((int16_t)str[30] - 48);
          else if (str[26] == '-') omega_cmd_x10[3] = -(((int16_t)str[27] - 48) * 1000 + ((int16_t)str[28] - 48) * 100 + ((int16_t)str[29] - 48) * 10 + ((int16_t)str[30] - 48));
          start_bit = 1;
          workspace_ctrl = 0;
          ps2_ctrl = 0;
        }
      }
    }
  }
}

//================================================================================================//
// Workspace velocity command                                                                     //
//   Argument : *str - received character array, ptr - the last pointer of the character array    //
//   Return   : none                                                                              //
//================================================================================================//
void read_work_vel_cmd(char *str, uint8_t ptr) {
  if (ptr == 28) {
    if (str[0] == 'w' && str[1] == 'r' && str[2] == 'k') {
      if (str[26] == 'e' && str[27] == 'n' && str[28] == 'd') {
        if (str[3] == ',' && str[7] == ',' && str[13] == ',' && str[19] == ',' && str[25] == ',') {
          if (str[8] == '+')       work_vel_cmd_x10[0] =   ((int16_t)str[ 9] - 48) * 1000 + ((int16_t)str[10] - 48) * 100 + ((int16_t)str[11] - 48) * 10 + ((int16_t)str[12] - 48);
          else if (str[8] == '-')  work_vel_cmd_x10[0] = -(((int16_t)str[ 9] - 48) * 1000 + ((int16_t)str[10] - 48) * 100 + ((int16_t)str[11] - 48) * 10 + ((int16_t)str[12] - 48));
          if (str[14] == '+')      work_vel_cmd_x10[1] =   ((int16_t)str[15] - 48) * 1000 + ((int16_t)str[16] - 48) * 100 + ((int16_t)str[17] - 48) * 10 + ((int16_t)str[18] - 48);
          else if (str[14] == '-') work_vel_cmd_x10[1] = -(((int16_t)str[15] - 48) * 1000 + ((int16_t)str[16] - 48) * 100 + ((int16_t)str[17] - 48) * 10 + ((int16_t)str[18] - 48));
          if (str[20] == '+')      work_vel_cmd_x10[2] =   ((int16_t)str[21] - 48) * 1000 + ((int16_t)str[22] - 48) * 100 + ((int16_t)str[23] - 48) * 10 + ((int16_t)str[24] - 48);
          else if (str[20] == '-') work_vel_cmd_x10[2] = -(((int16_t)str[21] - 48) * 1000 + ((int16_t)str[22] - 48) * 100 + ((int16_t)str[23] - 48) * 10 + ((int16_t)str[24] - 48));
          start_bit = 1;
          workspace_ctrl = 1;
          ps2_ctrl = 0;
        }
      }
    }
  }
}

//================================================================================================//
// Voltage command                                                                                //
//================================================================================================//
void read_vol_cmd(char *str, uint8_t ptr) {
  if (ptr == 34) {
    if (str[0] == 'v' && str[1] == 'o' && str[2] == 'l') {
      // Future works
    }
  }
}

//================================================================================================//
// Parameter change command No.1                                                                  //
//================================================================================================//
void read_pr1_cmd(char *str, uint8_t ptr) {
  if (ptr == 100) {
    if (str[0] == 'p' && str[1] == 'r' && str[2] == '1') {
      // Future works
    }
  }
}

//================================================================================================//
// Parameter change command No.2                                                                  //
//================================================================================================//
void read_pr2_cmd(char *str, uint8_t ptr) {
  if (ptr == 100) {
    if (str[0] == 'p' && str[1] == 'r' && str[2] == '2') {
      // Future works
    }
  }
}

//================================================================================================//
// Parameter change command No.3                                                                  //
//================================================================================================//
void read_pr3_cmd(char *str, uint8_t ptr) {
  if (ptr == 100) {
    if (str[0] == 'p' && str[1] == 'r' && str[2] == '3') {
      // Future works
    }
  }
}

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
#ifdef DEBUG
  // For normal
  Serial.print(F("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3]\n"));

  // For detail
  // Serial.print(F("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], vout_ff[0], vout_ff[1], vout_ff[2], vout_ff[3], vout_fb[0], vout_fb[1], vout_fb[2], vout_fb[3], test_cnt\n"));
#endif
}

//================================================================================================//
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  int i;
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
  // Serial.print('\n')
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
// print_state                                                                                    //
//================================================================================================//
void print_state(void) {

}

//================================================================================================//
// Inverse kinematics                                                                             //
//   only work_vel_cmd_x10[2] is devided 10                                                       //
//================================================================================================//
void inv_kinematics(void) {
  omega_cmd_x10[0] = (work_vel_cmd_x10[0] + work_vel_cmd_x10[1] + (W + L) / 2 * work_vel_cmd_x10[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[1] = (work_vel_cmd_x10[0] - work_vel_cmd_x10[1] - (W + L) / 2 * work_vel_cmd_x10[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[2] = (work_vel_cmd_x10[0] - work_vel_cmd_x10[1] + (W + L) / 2 * work_vel_cmd_x10[2] / 180 * 314 / 100) / R * 18000 / 314;
  omega_cmd_x10[3] = (work_vel_cmd_x10[0] + work_vel_cmd_x10[1] - (W + L) / 2 * work_vel_cmd_x10[2] / 180 * 314 / 100) / R * 18000 / 314;
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
    // Update inverse kinematics
    if (workspace_ctrl == 1) inv_kinematics();

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

    print_data();                  // Output log data to upper system
    if (++n == 1000) n = 0;       // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin(230400);  // Initialize Serial
  delay(100);             //added delay to give wireless ps2 module some time to startup, before configuring it

  // Setup of PS2 Controller
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  if (ps2_error == 0) {
#ifdef DEBUG
    Serial.println(F("Found Controller, configured successful"));
    Serial.print(F("pressures = "));
    if (pressures)
      Serial.println(F("true"));
    else
      Serial.println(F("false"));
    Serial.print(F("rumble = "));
    if (rumble)    Serial.println(F("true"));
    else           Serial.println(F("false"));
#endif
  }
  else if (ps2_error == 1) Serial.println(F("No controller found"));
  else if (ps2_error == 2) Serial.println(F("Controller found but not accepting commands"));
  else if (ps2_error == 3) Serial.println(F("Controller refusing to enter Pressures mode"));

  ps2_type = ps2x.readType();
  switch (ps2_type) {
#ifdef DEBUG
    case 0: Serial.println(F("Unknown Controller type found")); break;
    case 1: Serial.println(F("DualShock Controller found"));    break;
    case 2: Serial.println(F("GuitarHero Controller found"));   break;
    case 3: Serial.println(F("Wireless Sony DualShock Controller found")); break;
#endif
  }

  // Setup of encoder object
  enc.attach(0, FR_ENC_A, FR_ENC_B, DUAL_MULTI);
  enc.attach(1, FL_ENC_A, FL_ENC_B, DUAL_MULTI);
  enc.attach(2, RR_ENC_A, RR_ENC_B, DUAL_MULTI);
  enc.attach(3, RL_ENC_A, RL_ENC_B, DUAL_MULTI);

  // Setup of PWM Controller
  if (faboPWM.begin()) {
#ifdef DEBUG
    Serial.println(F("Find PCA9685"));
#endif
    faboPWM.init(0);
  }
  faboPWM.set_hz(1000);
  stop_all();

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
      if (rec[0] == 'v' && rec[1] == 'e' && rec[2] == 'l')      read_wheel_vel_cmd(rec, rec_ptr);
      else if (rec[0] == 'w' && rec[1] == 'r' && rec[2] == 'k') read_work_vel_cmd(rec, rec_ptr);
      else if (rec[0] == 'v' && rec[1] == 'o' && rec[2] == 'l') read_vol_cmd(rec, rec_ptr);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '1') read_pr1_cmd(rec, rec_ptr);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '2') read_pr2_cmd(rec, rec_ptr);
      else if (rec[0] == 'p' && rec[1] == 'r' && rec[2] == '3') read_pr3_cmd(rec, rec_ptr);
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

  // Reading PS2 Controller key ------------------------------------------------------------------//
  if (ps2_error == 1) { //skip loop if no controller found
    resetFunc();
  }

  if (ps2_type != 2) {    //DualShock Controller
    ps2x.read_gamepad(false, ps2_vibrate);  //read controller and set large motor to spin at 'vibrate' speed

    // These function will be TRUE as long as button is pressed
    // if (ps2x.Button(PSB_START))
    // if (ps2x.Button(PSB_SELECT))
    // if (ps2x.Button(PSB_PAD_UP))
    // if (ps2x.Button(PSB_PAD_RIGHT))
    // if (ps2x.Button(PSB_PAD_LEFT))
    // if (ps2x.Button(PSB_PAD_DOWN))

    // ps2_vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button

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

    // Command for robot workspace from PS2 controller -------------------------------------------//
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
