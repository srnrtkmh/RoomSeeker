//================================================================================================//
//                                                                                                //
// FILE : motor_control_2.ino                                                                     //
// MEMO : Control 4 motors velocity by PI control                                                 //
//        Added feedforward control with friction compensation                                    //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/02 : Start this project based on previous project "motor_control_1.ino"              //
//                                                                                                //
// Pin Assign                                                                                     //
//    2 - (OC3B) Front Right L298N ENA             3 - (OC3C) Front Left L298N ENB                //
//   22 - Front Right L298N IN1                   23 - Front Right L298N IN2                      //
//   24 - Front Left L298N IN3                    25 - Front Left L298N IN4                       //
//    4 - (OC0B) Rear Right L298N ENA              5 - (OC3A) Rear Left L298N ENB                 //
//   26 - Rear Right L298N IN1                    27 - Rear Right L298N IN2                       //
//   28 - Rear Left L298N IN3                     29 - Rear Left L298N IN4                        //
//                                                                                                //
//   62 - Front Right encoder phase A             63 - Front Right encoder phase B                //
//   64 - Front Left encoder phase A              65 - Front Left encoder phase B                 //
//   66 - Rear Right encoder phase A              67 - Rear Right encoder phase B                 //
//   68 - Rear Left encoder phase A               69 - Rear Left encoder phase B                  //
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
#include <avr/interrupt.h>
#include <TimerOne.h>

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
const int FR_ENC_A = 62;
const int FR_ENC_B = 63;
const int FL_ENC_A = 64;
const int FL_ENC_B = 65;
const int RR_ENC_A = 66;
const int RR_ENC_B = 67;
const int RL_ENC_A = 68;
const int RL_ENC_B = 69;

uint16_t rl_cnt = 0, rr_cnt = 0, fl_cnt = 0, fr_cnt = 0;
uint16_t fr_a_now, fr_b_now, fl_a_now, fl_b_now, rr_a_now, rr_b_now, rl_a_now, rl_b_now;
uint16_t fr_a_pre, fr_b_pre, fl_a_pre, fl_b_pre, rr_a_pre, rr_b_pre, rl_a_pre, rl_b_pre;

// For Motor - index 0:FR, 1:FL, 2:RR, 3:RL
// Port pin settings for Front Right motor
const byte speedPinR = 2;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte RightMotorDirPin1  = 22;             // Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
const byte RightMotorDirPin2  = 23;             // Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)

// Port pin settings for Front Left motor
const byte speedPinL = 3;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte LeftMotorDirPin1  = 24;              // Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
const byte LeftMotorDirPin2  = 25;              // Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)

// Port pin settings for Rear Right motor
const byte speedPinRB = 4;                      // Rear Wheel PWM pin connect Left MODEL-X ENA
const byte RightMotorDirPin1B = 26;             // Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
const byte RightMotorDirPin2B = 27;             // Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1)

// Port pin settings for Rear Left motor
const byte speedPinLB = 5;                      // Rear Wheel PWM pin connect Left MODEL-X ENB
const byte LeftMotorDirPin1B = 28;              // Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
const byte LeftMotorDirPin2B = 29;              // Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)

// Control variables
uint16_t cnt_now[4] = {0, 0, 0, 0};             // Current encoder counter for 4 motors [-]
uint16_t cnt_pre[4] = {0, 0, 0, 0};             // Previous encoder counter for 4 motors [-]
int16_t diff_cnt[4] = {0, 0, 0, 0};             // Difference between current and previous counter
long omega_res_x10[4] = {0, 0, 0, 0};           // Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf[4] = {0, 0, 0, 0};       // LPF output of Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x10_lpf_tmp[4] = {0, 0, 0, 0};   // Temporary for LPF
long g_omega[4] = {30, 30, 30, 30};             // Cutoff angular frequency [rad/sec]
long omega_cmd_x10[4] = {0, 0, 0, 0};           // Rotation speed command [10^-3 deg/sec]
long e_omega[4] = {0, 0, 0, 0};                 // Rotation speed error
long int_e_omega[4] = {0, 0, 0, 0};             // Integral of rotation speed error
long Kp[4] = {3, 3, 1, 3};                      // P gain for PI control
long Ki[4] = {20, 20, 3, 20};                   // I gain for PI control
int16_t vout[4] = {0, 0, 0, 0};                 // Voltage output for 4 motor drivers - index 1, 2, 4 : [0 - 1023], index3 : [0 - 255]
int16_t vout_ff[4] = {0, 0, 0, 0};              // Feedforward control output
int16_t vout_fb[4] = {0, 0, 0, 0};              // Feedback control output
int16_t vout_ll[4] = { -1000, -1000, -250, -1000};  // Lower limit of voltage output
int16_t vout_ul[4] = {1000, 1000, 250, 1000};       // Upper limit of voltage output

// Feedforward parameters
//long Fc_p[4] = { 536,  505,  73,  464};         // Coulonb friction compensation parameter for positive direction
//long Fc_n[4] = {-542, -482, -69, -461};         // Coulonb friction compensation parameter for negative direction
long Fc_p[4] = { 375,  354,  51,  325};         // Coulonb friction compensation parameter for positive direction
long Fc_n[4] = {-379, -337, -48, -323};         // Coulonb friction compensation parameter for negative direction
long Fd_p_x100[4] = {70, 79, 20, 82};           // Dynamic friction compensation parameter for negative direction
long Fd_n_x100[4] = {73, 77, 24, 74};           // Dynamic friction compensation parameter for negative direction

// Useful constants for calculation
long sampling_time = 9999;                      // サンプリング時間[us]
long dt1000 = 10;                               // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
long _dt_x10 = 1000;                              // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                         // Pi x 100

// Sample number
long n = 0;                                     // サンプルカウンタ変数

// For sequence
byte start_bit = 0;                             // サンプリング開始ビット

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
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  OCR3B = speed;
}

void FR_bck(int speed) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  OCR3B = speed;
}

void FL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  OCR3C = speed;
}

void FL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  OCR3C = speed;
}

void RR_fwd(int speed) {
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  OCR0B = speed;
}
void RR_bck(int speed) {
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  OCR0B = speed;
}
void RL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  OCR3A = speed;
}
void RL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  OCR3A = speed;
}

//================================================================================================//
// stop_Stop(void) --- Stop all motors                                                            //
//================================================================================================//
void stop_Stop() {
  OCR3B = 0;    // OCR3B -> Front Right motor
  OCR3C = 0;    // OCR3C -> Front Left motor
  OCR0B = 0;    // OCR0B -> Rear Right motor
  OCR3A = 0;    // OCR3A -> Rear Left motor
}

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
  // Encoder pin setting
  pinMode (RL_ENC_A, INPUT);
  pinMode (RL_ENC_B, INPUT);
  pinMode (RR_ENC_A, INPUT);
  pinMode (RR_ENC_B, INPUT);
  pinMode (FL_ENC_A, INPUT);
  pinMode (FL_ENC_B, INPUT);
  pinMode (FR_ENC_A, INPUT);
  pinMode (FR_ENC_B, INPUT);

  // Motor control port setting
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);

  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  stop_Stop();
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  gpio_init();                      // Initialize GPIO
  Serial.begin (230400);            // Initialize Serial

  // Initialize Pin change interrupt
  cli();
  PCICR = 0x04;                     // Enables PORT K Pin Change Interrupt
  PCMSK0 = 0x00;                    // Disables all PORT B Pin Change Interrupt
  PCMSK1 = 0x00;                    // Disables all PORT J Pin Change Interrupt
  PCMSK2 = 0xff;                    // Enables all PORT K Pin Change Interrupt
  sei();

  // Timer 0 PWM output compare settings
  OCR0A = 0;                        // Clear OCR0A
  OCR0B = 0;                        // Clear OCR0B
  TCNT0 = 0;                        // Clear timer 0
  TIMSK0 = 0x00;                    // None interrupts
  TCCR0A = 0x23;                    // OC0B is none inverting mode, PWM mode is Fast PWM
  TCCR0B = 0x02;                    // PWM mode is Fast PWM, Clock is clkI/O/8 (From prescaler)

  // Timer 3 PWM output compare settings
  OCR3A = 0;                        // Clear OCR0A
  OCR3B = 0;                        // Clear OCR0B
  OCR3C = 0;                        // Clear OCR0B
  TCNT3 = 0;                        // Clear timer 0
  TIMSK3 = 0x00;                    // None interrupts
  TCCR3A = 0xAB;                    // OC3A, OC3B, OC3C is none inverting mode, PWM mode is Fast PWM 10bit
  TCCR3B = 0x09;                    // PWM mode is Fast PWM, Clock is clkI/O/1024 (From prescaler)
  TCCR3C = 0x00;                    // Force compare match is disabled

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録

  // 各ピンの初期値を設定
  rl_a_pre = digitalRead(RL_ENC_A);
  rl_b_pre = digitalRead(RL_ENC_B);
  rr_a_pre = digitalRead(RR_ENC_A);
  rr_b_pre = digitalRead(RR_ENC_B);
  fl_a_pre = digitalRead(FL_ENC_A);
  fl_b_pre = digitalRead(FL_ENC_B);
  fr_a_pre = digitalRead(FR_ENC_A);
  fr_b_pre = digitalRead(FR_ENC_B);
}

//================================================================================================//
// Pin Change Interrupt Function                                                                  //
// MEMO : This function counts the encoder pulse                                                  //
//================================================================================================//
ISR(PCINT2_vect) {
  // Get current input value of the each encoder output
  fr_a_now = digitalRead(FR_ENC_A);
  fr_b_now = digitalRead(FR_ENC_B);
  fl_a_now = digitalRead(FL_ENC_A);
  fl_b_now = digitalRead(FL_ENC_B);
  rr_a_now = digitalRead(RR_ENC_A);
  rr_b_now = digitalRead(RR_ENC_B);
  rl_a_now = digitalRead(RL_ENC_A);
  rl_b_now = digitalRead(RL_ENC_B);

  // If the previous and the current state of the phase A are different, that means a Pulse has occured
  // If the phase B state is different to the phase A state, that means the encoder is rotating clockwise
  // Front Right Encoder
  if (fr_a_now != fr_a_pre) {
    if (fr_a_now == 0) {
      if (fr_b_now == 0) fr_cnt--;
      else               fr_cnt++;
    } else {
      if (fr_b_now == 0) fr_cnt++;
      else               fr_cnt--;
    }
  } else if (fr_b_now != fr_b_pre) {
    if (fr_b_now == 0) {
      if (fr_a_now == 0) fr_cnt++;
      else               fr_cnt--;
    } else {
      if (fr_a_now == 0) fr_cnt--;
      else               fr_cnt++;
    }
  }

  // Front Left Encoder
  else if (fl_a_now != fl_a_pre) {
    if (fl_a_now == 0) {
      if (fl_b_now == 0) fl_cnt++;
      else               fl_cnt--;
    } else {
      if (fl_b_now == 0) fl_cnt--;
      else               fl_cnt++;
    }
  } else if (fl_b_now != fl_b_pre) {
    if (fl_b_now == 0) {
      if (fl_a_now == 0) fl_cnt--;
      else               fl_cnt++;
    } else {
      if (fl_a_now == 0) fl_cnt++;
      else               fl_cnt--;
    }
  }

  // Rear Right Encoder
  else if (rr_a_now != rr_a_pre) {
    if (rr_a_now == 0) {
      if (rr_b_now == 0) rr_cnt--;
      else               rr_cnt++;
    } else {
      if (rr_b_now == 0) rr_cnt++;
      else               rr_cnt--;
    }
  } else if (rr_b_now != rr_b_pre) {
    if (rr_b_now == 0) {
      if (rr_a_now == 0) rr_cnt++;
      else               rr_cnt--;
    } else {
      if (rr_a_now == 0) rr_cnt--;
      else               rr_cnt++;
    }
  }

  // Rear Left Encoder
  else if (rl_a_now != rl_a_pre) {
    if (rl_a_now == 0) {
      if (rl_b_now == 0) rl_cnt++;
      else               rl_cnt--;
    } else {
      if (rl_b_now == 0) rl_cnt--;
      else               rl_cnt++;
    }
  } else if (rl_b_now != rl_b_pre) {
    if (rl_b_now == 0) {
      if (rl_a_now == 0) rl_cnt--;
      else               rl_cnt++;
    } else {
      if (rl_a_now == 0) rl_cnt++;
      else               rl_cnt--;
    }
  }

  fr_a_pre = fr_a_now; // Updates the previous state of the phase A with the current state
  fr_b_pre = fr_b_now; // Updates the previous state of the phase B with the current state
  fl_a_pre = fl_a_now; // Updates the previous state of the phase A with the current state
  fl_b_pre = fl_b_now; // Updates the previous state of the phase B with the current state
  rr_a_pre = rr_a_now; // Updates the previous state of the phase A with the current state
  rr_b_pre = rr_b_now; // Updates the previous state of the phase B with the current state
  rl_a_pre = rl_a_now; // Updates the previous state of the phase A with the current state
  rl_b_pre = rl_b_now; // Updates the previous state of the phase B with the current state
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
// void print_cnt(void) --- print counted value                                                   //
//================================================================================================//
void print_cnt(void) {
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
    cnt_pre[0] = cnt_now[0];
    cnt_pre[1] = cnt_now[1];
    cnt_pre[2] = cnt_now[2];
    cnt_pre[3] = cnt_now[3];
    cnt_now[0] = fr_cnt;
    cnt_now[1] = fl_cnt;
    cnt_now[2] = rr_cnt;
    cnt_now[3] = rl_cnt;

    // Calculate difference between current and previous counter value
    for (i = 0; i < 4; i++) {
      tmp = cnt_now[i] - cnt_pre[i];
      if (tmp > 32767L)       diff_cnt[i] = tmp - 65536L;
      else if (tmp < -32768L) diff_cnt[i] = tmp + 65536L;
      else                    diff_cnt[i] = tmp;
    }

    // Calculate rotation speed
    for (i = 0; i < 4; i++) {
      omega_res_x10[i] = (diff_cnt[i] * 360) * _dt_x10 / 300;
      omega_res_x10_lpf_tmp[i] += (omega_res_x10[i] - omega_res_x10_lpf[i]) * dt1000;
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
        int_e_omega[i] += e_omega[i] * dt1000;

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

    print_cnt();                  // Output log data to upper system
    if (++n == 1000) n = 0;       // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;
  uint8_t i;

  if (Serial.available()) {
    a = char(Serial.read());

    if (a == 's') {
      print_label();
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      delay(50);
      start_bit = 1;
    } else if (a == 't') {
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      start_bit = 0;
      stop_Stop();
    } else if (a == 'u') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] += 1000;
        if (omega_cmd_x10[i] > 600000) omega_cmd_x10[i] = 600000;
      }
    } else if (a == 'd') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] -= 1000;
        if (omega_cmd_x10[i] < -600000) omega_cmd_x10[i] = -600000;
      }
    }
  }
}
