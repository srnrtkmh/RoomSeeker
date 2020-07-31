//================================================================================================//
//                                                                                                //
// FILE : encoder_4.ino                                                                           //
// MEMO : Count 4 encoder signal and transmit counted values to serial                            //
//        Catch the encoder output change by Pin change interrupt                                 //
//                                                                                                //
// Update Log                                                                                     //
//   2020/07/25 : Start this project based on previous project "encoder_3.ino"                    //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
// Pin Assign                                                                                     //
//    9 - Front Right L298N ENA                                                                   //
//   22 - Front Right L298N IN1                   24 - Front Right L298N IN2                      //
//   10 - Front Left L298N ENB                                                                    //
//   26 - Front Left L298N IN3                    28 - Front Left L298N IN4                       //
//    3 - Rear Right L298N ENA                                                                    //
//    5 - Rear RIght L298N IN1                     6 - Rear Right L298N IN2                       //
//    4 - Rear Left L298N ENB                                                                     //
//    7 - Rear Left L298N IN3                      8 - Rear Left L298N IN4                        //
//                                                                                                //
//   62 - Front Right encoder phase A             63 - Front Right encoder phase B                //
//   64 - Front Left encoder phase A              65 - Front Left encoder phase B                 //
//   66 - Rear Right encoder phase A              67 - Rear Right encoder phase B                 //
//   68 - Rear Left encoder phase A               69 - Rear Left encoder phase B                  //
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

int rl_cnt = 0, rr_cnt = 0, fl_cnt = 0, fr_cnt = 0;
int fr_a_now, fr_b_now, fl_a_now, fl_b_now, rr_a_now, rr_b_now, rl_a_now, rl_b_now;
int fr_a_pre, fr_b_pre, fl_a_pre, fl_b_pre, rr_a_pre, rr_b_pre, rl_a_pre, rl_b_pre;

// For Motor
const byte SPEED = 100;
const byte TURN_SPEED = 160;

const byte speedPinR = 9;           // Front Wheel PWM pin connect Right MODEL-X ENA
const byte RightMotorDirPin1  = 22; // Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
const byte RightMotorDirPin2  = 24; // Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)

const byte speedPinL = 10;          // Front Wheel PWM pin connect Right MODEL-X ENB
const byte LeftMotorDirPin1  = 26;  // Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
const byte LeftMotorDirPin2  = 28;  // Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)

const byte speedPinRB = 3;          // Rear Wheel PWM pin connect Left MODEL-X ENA
const byte RightMotorDirPin1B = 5;  // Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
const byte RightMotorDirPin2B = 6;  // Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1)

const byte speedPinLB = 4;          // Rear Wheel PWM pin connect Left MODEL-X ENB
const byte LeftMotorDirPin1B = 7;   // Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
const byte LeftMotorDirPin2B = 8;   // Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)

// 周期データ取得用
int n = 0;                          // サンプルカウンタ変数
byte start_bit = 0;                 // サンプリング開始ビット
long sampling_time = 9999;         // サンプリング時間[us]
byte spd = 0;                       // モータ回転速度[0～255]

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
// FR_fwd(int speed), FL                                                                          //
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
  Serial.print(fr_cnt);
  Serial.print(",");
  Serial.print(fl_cnt);
  Serial.print(",");
  Serial.print(rr_cnt);
  Serial.print(",");
  Serial.print(rl_cnt);
  Serial.print(",");
  Serial.print(spd);
  Serial.print("\n");
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

  rl_a_pre = rl_a_now; // Updates the previous state of the phase A with the current state
  rl_b_pre = rl_b_now; // Updates the previous state of the phase B with the current state
  rr_a_pre = rr_a_now; // Updates the previous state of the phase A with the current state
  rr_b_pre = rr_b_now; // Updates the previous state of the phase B with the current state
  fl_a_pre = fl_a_now; // Updates the previous state of the phase A with the current state
  fl_b_pre = fl_b_now; // Updates the previous state of the phase B with the current state
  fr_a_pre = fr_a_now; // Updates the previous state of the phase A with the current state
  fr_b_pre = fr_b_now; // Updates the previous state of the phase B with the current state
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
}
