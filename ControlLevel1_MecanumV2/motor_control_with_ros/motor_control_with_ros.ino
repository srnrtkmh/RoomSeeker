//================================================================================================//
//                                                                                                //
// FILE : motor_control_with_ros.ino                                                              //
// MEMO : Control 4 motors velocity by PI control                                                 //
//        Added feedforward control with friction compensation                                    //
//        Control commands are input via ros topic communication                                  //
//                                                                                                //
// Update Log                                                                                     //
//   2020/09/18 : Start this project based on "motor_control_with_ros.ino"                        //
//                                                                                                //
// Board : Arduino Uno Rev 3                                                                      //
//                                                                                                //
// Pin Assign                                                                                     //
//    3 - Front Right encoder phase A              2 - Front Right encoder phase B                //
//    6 - Front Left encoder phase A               4 - Front Left encoder phase B                 //
//    5 - Rear Right encoder phase A               7 - Rear Right encoder phase B                 //
//    8 - Rear Left encoder phase A                9 - Rear Left encoder phase B                  //
//                                                                                                //
//   13 - RC Servo command to control US Sensor direction                                         //
//   30 - Trigger signal to US Sensor (HC-SR04)   31 - Echo input from US Sensor (HC-SR04)        //
//   32 - Left line sensor input                  33 - Center line sensor input                   //
//   34 - Right line sensor input                                                                 //
//                                                                                                //
//   18 - (TXD1) ESP UART RXD                     19 - (RXD1) ESP UART TXD                        //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "rotary_encoders_uno.h"
#include "FaBoPWM_PCA9685.h"

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void messageCb( const std_msgs::String &message);

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
PROGMEM const uint8_t FR_ENC_A = 3;
PROGMEM const uint8_t FR_ENC_B = 2;
PROGMEM const uint8_t FL_ENC_A = 6;
PROGMEM const uint8_t FL_ENC_B = 4;
PROGMEM const uint8_t RR_ENC_A = 5;
PROGMEM const uint8_t RR_ENC_B = 7;
PROGMEM const uint8_t RL_ENC_A = 8;
PROGMEM const uint8_t RL_ENC_B = 9;
PROGMEM const uint8_t FR_ENC = 0;
PROGMEM const uint8_t FL_ENC = 1;
PROGMEM const uint8_t RR_ENC = 2;
PROGMEM const uint8_t RL_ENC = 3;
PROGMEM const uint8_t ENC_RESO = 44;
PROGMEM const uint8_t GEAR_REDU = 30;
RotaryEncoders enc;

// For Motor - index 0:FR, 1:FL, 2:RR, 3:RL
FaBoPWM faboPWM;                    // PWM controller object
PROGMEM const uint8_t FR1 = 6;      // Front Right wheel index
PROGMEM const uint8_t FR2 = 7;      // Front Right wheel index
PROGMEM const uint8_t FL1 = 5;      // Front Left wheel index
PROGMEM const uint8_t FL2 = 4;      // Front Left wheel index
PROGMEM const uint8_t RR1 = 2;      // Rear Right wheel index
PROGMEM const uint8_t RR2 = 3;      // Rear Right wheel index
PROGMEM const uint8_t RL1 = 1;      // Rear Left wheel index
PROGMEM const uint8_t RL2 = 0;      // Rear Left wheel index

// Control variables
int8_t cnt_dir[4] = {1, -1, 1, -1};             // Direction correction co-efficient of rotation
uint16_t cnt_now[4] = {0, 0, 0, 0};             // Current encoder counter for 4 motors [-]
uint16_t cnt_pre[4] = {0, 0, 0, 0};             // Previous encoder counter for 4 motors [-]
int16_t diff_cnt[4] = {0, 0, 0, 0};             // Difference between current and previous counter
long omega_res_x10[4] = {0, 0, 0, 0};           // Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf[4] = {0, 0, 0, 0};       // LPF output of Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf_tmp[4] = {0, 0, 0, 0};   // Temporary for LPF
long omega_cmd_x10[4] = {0, 0, 0, 0};           // Rotation speed command [10^-3 deg/sec]
long e_omega[4] = {0, 0, 0, 0};                 // Rotation speed error
long int_e_omega[4] = {0, 0, 0, 0};             // Integral of rotation speed error
int16_t vout[4] = {0, 0, 0, 0};                 // Voltage output for 4 motor drivers - index 1, 2, 4 : [0 - 1023], index3 : [0 - 255]
int16_t vout_ff[4] = {0, 0, 0, 0};              // Feedforward control output
int16_t vout_fb[4] = {0, 0, 0, 0};              // Feedback control output

// Feecback parameters
PROGMEM const int16_t g_omega[4] = {60, 60, 60, 60};              // Cutoff angular frequency [rad/sec]
PROGMEM const int16_t Kp[4] = {15, 15, 15, 15};                   // P gain for PI control
PROGMEM const int16_t Ki[4] = {40, 40, 40, 40};                   // I gain for PI control
PROGMEM const int16_t vout_ll[4] = { -4095, -4095, -4095, -4095}; // Lower limit of voltage output
PROGMEM const int16_t vout_ul[4] = { 4095,  4095,  4095,  4095};  // Upper limit of voltage output

// Feedforward parameters
PROGMEM const int16_t Fc_p[4] = {  0,  0,  0,  0};  // Coulonb friction compensation parameter for positive direction
PROGMEM const int16_t Fc_n[4] = { -0, -0, -0, -0};  // Coulonb friction compensation parameter for negative direction
PROGMEM const int16_t Fd_p_x100[4] = {0, 0, 0, 0};  // Dynamic friction compensation parameter for negative direction
PROGMEM const int16_t Fd_n_x100[4] = {0, 0, 0, 0};  // Dynamic friction compensation parameter for negative direction

// Useful constants for calculation
PROGMEM const long sampling_time = 20000;       // sampling time [us]
PROGMEM const long dt_x1000 = 20;               // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
PROGMEM const long _dt_x10 = 500;               // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
PROGMEM const long pi100 = 314;                 // Pi x 100

// Sample number
uint16_t n = 0;                                 // サンプルカウンタ変数

// For sequence
byte start_bit = 0;                             // サンプリング開始ビット

// For ROS
ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::String tmp_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Subscriber<std_msgs::String> sub("toggle_led", &messageCb);

//================================================================================================//
// Call back function of subscribing ros message                                                  //
//================================================================================================//
void messageCb(const std_msgs::String &message){
  tmp_msg.data = message.data;
  
  if(omega_cmd_x10[0] == 1000){
    omega_cmd_x10[0] = 0;
    start_bit = 1;
  }
  else{
    omega_cmd_x10[0] = 1000;
    start_bit = 0;
  }
}

//================================================================================================//
// FR_fwd(int speed), FR_bck(int speed) --- this is the same for FL, RR, RL motor                 //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void FR_fwd(int speed) {
  do {
    faboPWM.set_channel_value(FR1, speed);
    faboPWM.set_channel_value(FR2, 0);
  } while (0);
}
void FR_bck(int speed) {
  do {
    faboPWM.set_channel_value(FR1, 0);
    faboPWM.set_channel_value(FR2, speed);
  } while (0);
}
void FL_fwd(int speed) {
  do {
    faboPWM.set_channel_value(FL1, speed);
    faboPWM.set_channel_value(FL2, 0);
  } while (0);
}
void FL_bck(int speed) {
  do {
    faboPWM.set_channel_value(FL1, 0);
    faboPWM.set_channel_value(FL2, speed);
  } while (0);
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
//================================================================================================//
void stop_all() {
  FR_fwd(0);
  FL_fwd(0);
  RR_fwd(0);
  RL_fwd(0);
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin (230400);            // Initialize Serial

  // Setup of encoder object
  enc.attach(0, FR_ENC_A, FR_ENC_B);
  enc.attach(1, FL_ENC_A, FL_ENC_B);
  enc.attach(2, RR_ENC_A, RR_ENC_B);
  enc.attach(3, RL_ENC_A, RL_ENC_B);

  // Setup of PWM Controller
  if (faboPWM.begin()) {
    Serial.println("Find PCA9685");
    faboPWM.init(0);
  }
  faboPWM.set_hz(1000);
  stop_all();

  // Setup of ros node handler
  nh.getHardware()->setBaud(230400);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  str_msg.data = "Hello World!!";

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録
}

//================================================================================================//
// void print_label(void) --- print data label                                                    //
//================================================================================================//
void print_label(void) {
  // For normal
  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3], vout_ff[0], vout_ff[1], vout_ff[2], vout_ff[3], vout_fb[0], vout_fb[1], vout_fb[2], vout_fb[3]\n");

  // For debug
  //  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], diff_cnt[0], diff_cnt[1], diff_cnt[2], diff_cnt[3], omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3], omega_res_x10_lpf[0], omega_res_x10_lpf[1], omega_res_x10_lpf[2], omega_res_x10_lpf[3], omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3], vout[0], vout[1], vout[2], vout[3]\n");
}

//================================================================================================//
// void print_data(void) --- print counted value                                                  //
//================================================================================================//
void print_data(void) {
  int i;
  Serial.print(n);
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(cnt_now[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_res_x10[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_res_x10_lpf[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_cmd_x10[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout_ff[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout_fb[i]);
  }
  Serial.print(",");
//  Serial.print(enc.read_test_cnt());
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

    print_data();                  // Output log data to upper system
    if (++n == 1000) n = 0;       // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }
}

//================================================================================================//
// kaishaku_sentence()                                                                            //
//================================================================================================//
void read_vel_cmd(char *str, uint8_t ptr) {
  if (ptr == 30) {
    if (str[0] == 'v' && str[1] == 'e' && str[2] == 'l') {
      if (str[28] == 'e' && str[29] == 'n' && str[30] == 'd') {
        if (str[3] == ',' && str[7] == ',' && str[12] == ',' && str[17] == ',' && str[22] == ',' && str[27] == ',') {
          omega_cmd_x10[0] = ((long)str[8] - 48)  * 1000 + ((long)str[9] - 48)  * 100 + ((long)str[10] - 48) * 10 + ((long)str[11] - 48);
          omega_cmd_x10[1] = ((long)str[13] - 48) * 1000 + ((long)str[14] - 48) * 100 + ((long)str[15] - 48) * 10 + ((long)str[16] - 48);
          omega_cmd_x10[2] = ((long)str[18] - 48) * 1000 + ((long)str[19] - 48) * 100 + ((long)str[20] - 48) * 10 + ((long)str[21] - 48);
          omega_cmd_x10[3] = ((long)str[23] - 48) * 1000 + ((long)str[24] - 48) * 100 + ((long)str[25] - 48) * 10 + ((long)str[26] - 48);
        }
      }
    }
  }
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(100);
}
