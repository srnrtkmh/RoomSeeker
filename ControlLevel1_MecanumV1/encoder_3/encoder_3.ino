//================================================================================================//
//                                                                                                //
// FILE : encoder_3.ino                                                                           //
// MEMO : Count 4 encoder signal and transmit counted values to serial                            //
//                                                                                                //
// Update Log                                                                                     //
//   2020/07/25 : Start this project based on previous project "encoder_2.ino"                    //
//                                                                                                //
//                                         Copyright (c) 2019 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <TimerOne.h>

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
const int RL_ENC_A = 40;
const int RL_ENC_B = 41;
const int RR_ENC_A = 42;
const int RR_ENC_B = 43;
const int FL_ENC_A = 44;
const int FL_ENC_B = 45;
const int FR_ENC_A = 46;
const int FR_ENC_B = 47;

int rl_cnt = 0, rr_cnt = 0, fl_cnt = 0, fr_cnt = 0;
int rl_a_now, rl_b_now, rr_a_now, rr_b_now, fl_a_now, fl_b_now, fr_a_now, fr_b_now;
int rl_a_pre, rl_b_pre, rr_a_pre, rr_b_pre, fl_a_pre, fl_b_pre, fr_a_pre, fr_b_pre;

// For Motor
#define SPEED 100
#define TURN_SPEED 160
#define speedPinR 9           // Front Wheel PWM pin connect Right MODEL-X ENA
#define RightMotorDirPin1  22 // Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2  24 // Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)
#define LeftMotorDirPin1  26  // Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2  28  // Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL 10          // Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB 3          // Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B 5  // Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B 6  // Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B 7   // Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B 8   // Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB 4          // Rear Wheel PWM pin connect Left MODEL-X ENB

// 周期データ取得用
int n = 0;                    // サンプルカウンタ変数
int start_bit = 0;            // サンプリング開始ビット
long sampling_time = 99999;  // サンプリング時間[us]
int spd = 0;                  // モータ回転速度[0～255]

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
// go_advance(int speed )                                                                         //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void go_advance(int speed) {
  RL_fwd(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_fwd(speed);
}

//================================================================================================//
// FR_fwd(int speed), FL                                                                              //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void FR_fwd(int speed) {
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}

void FR_bck(int speed) {
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}

void FL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}

void FL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed) {
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed) {
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed) {
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed) {
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}

void stop_Stop() {
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}

//================================================================================================//
// void print_cnt(void) --- print counted value                                                   //
//================================================================================================//
void print_cnt(void) {
  Serial.print(rl_cnt);
  Serial.print(",");
  Serial.print(rr_cnt);
  Serial.print(",");
  Serial.print(fl_cnt);
  Serial.print(",");
  Serial.print(fr_cnt);
  Serial.print("\n");
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  gpio_init();
  Serial.begin (230400);

  // 各ピンの初期値を設定
  rl_a_pre = digitalRead(RL_ENC_A);
  rl_b_pre = digitalRead(RL_ENC_B);
  rr_a_pre = digitalRead(RR_ENC_A);
  rr_b_pre = digitalRead(RR_ENC_B);
  fl_a_pre = digitalRead(FL_ENC_A);
  fl_b_pre = digitalRead(FL_ENC_B);
  fr_a_pre = digitalRead(FR_ENC_A);
  fr_b_pre = digitalRead(FR_ENC_B);

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
//================================================================================================//
void flash() {
  interrupts();                             // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  if (start_bit == 1) {
    Serial.print(n);
    Serial.print(",");
    print_cnt();

    // サンプルカウンタの更新
    if (++n == 1000) n = 0;
  }
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char a;

  if (Serial.available()) {
    a = char(Serial.read());

    if (a == 's') {
      start_bit = 1;
      go_advance(0);
    } else if (a == 't') {
      start_bit = 0;
      go_advance(0);
    } else if (a == 'u') {
      spd += 10;
      if (spd > 250) spd = 250;
      go_advance(spd);
    } else if (a == 'd') {
      spd -= 10;
      if (spd < 0) spd = 0;
      go_advance(spd);
    }
  }

  if (start_bit == 1) {
    // Get current input value of the each encoder output
    rl_a_now = digitalRead(RL_ENC_A);
    rl_b_now = digitalRead(RL_ENC_B);
    rr_a_now = digitalRead(RR_ENC_A);
    rr_b_now = digitalRead(RR_ENC_B);
    fl_a_now = digitalRead(FL_ENC_A);
    fl_b_now = digitalRead(FL_ENC_B);
    fr_a_now = digitalRead(FR_ENC_A);
    fr_b_now = digitalRead(FR_ENC_B);

    // If the previous and the current state of the phase A are different, that means a Pulse has occured
    // If the phase B state is different to the phase A state, that means the encoder is rotating clockwise
    // Rear Left Encoder
    if (rl_a_now != rl_a_pre) {
      if (rl_a_now == 0) {
        if (rl_b_now == 0) rl_cnt--;
        else               rl_cnt++;
      } else {
        if (rl_b_now == 0) rl_cnt++;
        else               rl_cnt--;
      }
    } else if (rl_b_now != rl_b_pre) {
      if (rl_b_now == 0) {
        if (rl_a_now == 0) rl_cnt++;
        else               rl_cnt--;
      } else {
        if (rl_a_now == 0) rl_cnt--;
        else               rl_cnt++;
      }
    }

    // Rear Right Encoder
    if (rr_a_now != rr_a_pre) {
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

    // Front Left Encoder
    if (fl_a_now != fl_a_pre) {
      if (fl_a_now == 0) {
        if (fl_b_now == 0) fl_cnt--;
        else               fl_cnt++;
      } else {
        if (fl_b_now == 0) fl_cnt++;
        else               fl_cnt--;
      }
    } else if (fl_b_now != fl_b_pre) {
      if (fl_b_now == 0) {
        if (fl_a_now == 0) fl_cnt++;
        else               fl_cnt--;
      } else {
        if (fl_a_now == 0) fl_cnt--;
        else               fl_cnt++;
      }
    }

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

    rl_a_pre = rl_a_now; // Updates the previous state of the phase A with the current state
    rl_b_pre = rl_b_now; // Updates the previous state of the phase B with the current state
    rr_a_pre = rr_a_now; // Updates the previous state of the phase A with the current state
    rr_b_pre = rr_b_now; // Updates the previous state of the phase B with the current state
    fl_a_pre = fl_a_now; // Updates the previous state of the phase A with the current state
    fl_b_pre = fl_b_now; // Updates the previous state of the phase B with the current state
    fr_a_pre = fr_a_now; // Updates the previous state of the phase A with the current state
    fr_b_pre = fr_b_now; // Updates the previous state of the phase B with the current state
  }
}
