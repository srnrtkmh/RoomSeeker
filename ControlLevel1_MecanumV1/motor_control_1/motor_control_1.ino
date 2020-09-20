//================================================================================================//
//                                                                                                //
// FILE : motor_control_1.ino                                                                     //
// MEMO : Control 4 motors velocity by PI control                                                 //
//                                                                                                //
// Update Log                                                                                     //
//   2020/07/25 : Start this project based on previous project "encoder_3.ino"                    //
//   2020/07/26 : Added angular velocity and apply LPF                                            //
//                Roughly control velocity by PI controller                                       //
//   2020/07/31 : Added the counter error collection process if the encoder counter overflows     //
//   2020/08/01 : Added stop I control if the control output reach upper limit                    //
//                Changed PWM frequency to control motor                                          //
//                Changed pin assign to reduce timer module usage                                 //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
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
const byte SPEED = 100;
const byte TURN_SPEED = 160;

const byte speedPinR = 2;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte RightMotorDirPin1  = 22;             // Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
const byte RightMotorDirPin2  = 23;             // Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)

const byte speedPinL = 3;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte LeftMotorDirPin1  = 24;              // Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
const byte LeftMotorDirPin2  = 25;              // Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)

const byte speedPinRB = 4;                      // Rear Wheel PWM pin connect Left MODEL-X ENA
const byte RightMotorDirPin1B = 26;             // Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
const byte RightMotorDirPin2B = 27;             // Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1)

const byte speedPinLB = 5;                      // Rear Wheel PWM pin connect Left MODEL-X ENB
const byte LeftMotorDirPin1B = 28;              // Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
const byte LeftMotorDirPin2B = 29;              // Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)

uint16_t cnt_now[4] = {0, 0, 0, 0};             // Current encoder counter for 4 motors [-]
uint16_t cnt_pre[4] = {0, 0, 0, 0};             // Previous encoder counter for 4 motors [-]
int16_t diff_cnt[4] = {0, 0, 0, 0};             // Difference between current and previous counter
long omega_res_x1000[4] = {0, 0, 0, 0};         // Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x1000_lpf[4] = {0, 0, 0, 0};     // LPF output of Rotation speed for 4 motors [10^-3 deg/sec]
long omega_res_x1000_lpf_tmp[4] = {0, 0, 0, 0}; // Temporary for LPF
long g_omega[4] = {30, 30, 30, 30};             // Cutoff angular frequency [rad/sec]
long omega_cmd_x1000[4] = {0, 0, 0, 0};         // Rotation speed command [10^-3 deg/sec]
long e_omega[4] = {0, 0, 0, 0};                 //
long int_e_omega[4] = {0, 0, 0, 0};             //
long Kp[4] = {2, 3, 2, 4};                      //
long Ki[4] = {20, 20, 2, 20};                   //
int16_t vout[4] = {0, 0, 0, 0};                 // Voltage output for 4 motor drivers [0 - 255]
long pi100 = 314;                               // Pi x 100
long _dt10 = 1000;                              // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
long dt1000 = 10;                               // (sampling_time / 1,000,000) * 1000 [10^-3 sec]

// 周期データ取得用
long n = 0;                                     // サンプルカウンタ変数
byte start_bit = 0;                             // サンプリング開始ビット
long sampling_time = 9999;                      // サンプリング時間[us]

//================================================================================================//
// go_advance(int speed )                                                                         //
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
// FR_fwd(int speed), FL                                                                          //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void FR_fwd(int speed) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  OCR3B = speed;
//  analogWrite(speedPinR, speed);
}

void FR_bck(int speed) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  OCR3B = speed;
//  analogWrite(speedPinR, speed);
}

void FL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  OCR3C = speed;
//  analogWrite(speedPinL, speed);
}

void FL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  OCR3C = speed;
//  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed) {
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  OCR0B = speed;
//  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed) {
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  OCR0B = speed;
//  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  OCR3A = speed;
//  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  OCR3A = speed;
//  analogWrite(speedPinLB, speed);
}

void stop_Stop() {
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
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
    Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], omega_res_x1000[0], omega_res_x1000[1], omega_res_x1000[2], omega_res_x1000[3], omega_res_x1000_lpf[0], omega_res_x1000_lpf[1], omega_res_x1000_lpf[2], omega_res_x1000_lpf[3], omega_cmd_x1000[0], omega_cmd_x1000[1], omega_cmd_x1000[2], omega_cmd_x1000[3], vout[0], vout[1], vout[2], vout[3]\n");

  // For debug
//  Serial.print("n, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3], diff_cnt[0], diff_cnt[1], diff_cnt[2], diff_cnt[3], omega_res_x1000[0], omega_res_x1000[1], omega_res_x1000[2], omega_res_x1000[3], omega_res_x1000_lpf[0], omega_res_x1000_lpf[1], omega_res_x1000_lpf[2], omega_res_x1000_lpf[3], omega_cmd_x1000[0], omega_cmd_x1000[1], omega_cmd_x1000[2], omega_cmd_x1000[3], vout[0], vout[1], vout[2], vout[3]\n");
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
    Serial.print(omega_res_x1000[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_res_x1000_lpf[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(omega_cmd_x1000[i]);
  }
  for (i = 0; i < 4; i++) {
    Serial.print(",");
    Serial.print(vout[i]);
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
      omega_res_x1000[i] = (diff_cnt[i] * 360) * _dt10 / 300;
      omega_res_x1000_lpf_tmp[i] += (omega_res_x1000[i] - omega_res_x1000_lpf[i]) * dt1000;
      omega_res_x1000_lpf[i] = omega_res_x1000_lpf_tmp[i] * g_omega[i] / 1000;
    }

    // Calculate control output
    for (i = 0; i < 4; i++) {
      e_omega[i] = omega_cmd_x1000[i] - omega_res_x1000_lpf[i];
      if (i == 2) {
        if (-250 < vout[i] && vout[i] < 250)
          int_e_omega[i] += e_omega[i] * dt1000;
        vout[i] = Kp[i] * e_omega[i] / 10 + Ki[i] * int_e_omega[i] / 10000;
        if (vout[i] > 250) vout[i] = 250;
        else if (vout[i] < -250) vout[i] = -250;
      } else {
        if (-1000 < vout[i] && vout[i] < 1000)
          int_e_omega[i] += e_omega[i] * dt1000;
        vout[i] = Kp[i] * e_omega[i] / 10 + Ki[i] * int_e_omega[i] / 10000;
        if (vout[i] > 1000) vout[i] = 1000;
        else if (vout[i] < -1000) vout[i] = -1000;
      }
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
      for (i = 0; i < 4; i++) omega_cmd_x1000[i] = 0;
      delay(50);
      start_bit = 1;
    } else if (a == 't') {
      for (i = 0; i < 4; i++) omega_cmd_x1000[i] = 0;
      start_bit = 0;
      stop_Stop();
    } else if (a == 'u') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x1000[i] += 1000;
        if (omega_cmd_x1000[i] > 600000) omega_cmd_x1000[i] = 600000;
      }
    } else if (a == 'd') {
      for (i = 0; i < 4; i++) {
        omega_cmd_x1000[i] -= 1000;
        if (omega_cmd_x1000[i] < -600000) omega_cmd_x1000[i] = -600000;
      }
    }
  }
}
