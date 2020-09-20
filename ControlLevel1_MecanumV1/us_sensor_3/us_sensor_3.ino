//================================================================================================//
//                                                                                                //
// FILE : us_sensor_3.ino                                                                         //
// MEMO : get ultra sonic distance sensor value in front of the robot                             //
//        us sensor servo moves smoothly by commanding more precise and high sampling rate        //
//        us tranmit timing is along with the command of servo                                    //
//        us reflection is detected by pin change interrupt not to wait in timer1 interrupt func  //
//        waiting us reflection occupy too long time if detection is in timer1 interrupt func     //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/16 : Start this project based on previous project "us_sensor_3.ino"                  //
//                                                                                                //
// Pin Assign                                                                                     //
//    5 - (OC3A) Front Right L298N ENA             2 - (OC3B) Front Left L298N ENB                //
//   22 - Front Right L298N IN1                   23 - Front Right L298N IN2                      //
//   24 - Front Left L298N IN3                    25 - Front Left L298N IN4                       //
//    6 - (OC4A) Rear Right L298N ENA              7 - (OC4B) Rear Left L298N ENB                 //
//   26 - Rear Right L298N IN1                    27 - Rear Right L298N IN2                       //
//   28 - Rear Left L298N IN3                     29 - Rear Left L298N IN4                        //
//                                                                                                //
//   62 - Front Right encoder phase A             63 - Front Right encoder phase B                //
//   64 - Front Left encoder phase A              65 - Front Left encoder phase B                 //
//   66 - Rear Right encoder phase A              67 - Rear Right encoder phase B                 //
//   68 - Rear Left encoder phase A               69 - Rear Left encoder phase B                  //
//                                                                                                //
//   31 - RC Servo command to control US Sensor direction                                         //
//   30 - Trigger signal to US Sensor (HC-SR04)   14 - Echo input from US Sensor (HC-SR04)        //
//   32 - Left line sensor input                  33 - Center line sensor input                   //
//   34 - Right line sensor input                                                                 //
//                                                                                                //
//   18 - (TXD1) ESP UART RXD                     19 - (RXD1) ESP UART TXD                        //
//                                                                                                //
// Timer Assign                                                                                   //
//   Timer 0 : Global timer counter we can use mills(), micros(), delay()...etc                   //
//   Timer 1 : Periodic interrupt process                                                         //
//   Timer 2 : none                                                                               //
//   Timer 3 : PWM for Front motor driver                                                         //
//   Timer 4 : PWM for Rear motor driver                                                          //
//   Timer 5 : (maybe) Servo.h                                                                    //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <avr/interrupt.h>
#include <TimerOne.h>
#include <Servo.h>

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For ultra sonic sensor
const uint8_t SERVO_PIN = 31;               // Servo connect
const uint8_t Echo_PIN  = 14;               // Ultrasonic Echo pin connect
const uint8_t Trig_PIN  = 30;               // Ultrasonic Trig pin connect
Servo head;
const uint16_t SERVO_MAX = 2300;            // Max count in Servo.writeMicroseconds()
const uint16_t SERVO_MIN = 430;             // Min count in Servo.writeMicroseconds()
const uint8_t HEAD_DIR_POS = 0;             // Head driving positive direction
const uint8_t HEAD_DIR_NEG = 1;             // Head driving negative direction
uint8_t head_dir = HEAD_DIR_POS;            // Head direction variables
const uint16_t HEAD_CNT_MAX = 100;          // Max count of head_cnt
const uint16_t HEAD_CNT_MIN = 0;            // Min count of head_cnt
const uint16_t HEAD_CNT_INIT = 50;          // Initial count of head_cnt
uint16_t head_cnt = 0;                      // Head position counter
const uint8_t HEAD_INDEX_MAX = 18;          // Max count of head_index
const uint8_t HEAD_INDEX_MIN = 0;           // Min count of head_index
uint8_t head_index = 9;                     // Head position index which means detecting direction number

const uint16_t us_tx[19] = {0, 6, 11, 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100}; // US transmit timing index
unsigned long us_dist[19] = {0};            // US sensor distance
unsigned long us_time_1 = 0, us_time_2 = 0; // Temp variables using us reflection time
unsigned long us_time_diff = 0;             // Differential time between us transmit and receive
uint8_t us_reload_flag = 0;                 // This flag is set 1 loop of distance detection
uint16_t us_reload_cnt = 0;                 // This count value is for debugging
const uint8_t US_1_WAIT_NEXT = 0;           // This shows us sensor is waiting next transmission
const uint8_t US_2_WAIT_REF = 1;            // This shows us sensor is waiting for reflection
uint8_t us_status = US_1_WAIT_NEXT;         // US sensor status
uint8_t us_ref_cnt = 0;                     // US sensor reflection cycle counter
const uint8_t US_REF_CNT_MAX = 2;           // Max count of us_ref_cnt

// Useful constants for calculation
int i = 0;
long sampling_time = 9999;                  // サンプリング時間[us]
long dt1000 = 10;                           // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
long _dt_x10 = 1000;                        // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                     // Pi x 100

// Sample number
long n = 0;                                 // サンプルカウンタ変数
byte start_bit = 0;                         // サンプリング開始ビット

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  // HC-SR04 setting
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  Serial.begin (230400);            // Initialize Serial

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録

  // Initialize us sensor servo
  digitalWrite(Trig_PIN, LOW);
  head.attach(SERVO_PIN, SERVO_MIN, SERVO_MAX);
  head_cnt = HEAD_CNT_INIT;
  head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
  delay(500);
}

//================================================================================================//
// Pin Change Interrupt Function - Port J                                                         //
// MEMO : This function counts the encoder pulse                                                  //
//================================================================================================//
ISR(PCINT1_vect) {
  interrupts();
  if (digitalRead(Echo_PIN) == LOW) {
    us_time_2 = micros();
    cli();
    PCICR &= ~(1 << 1);             // Disable PORT J Pin Change Interrupt
    PCMSK1 &= ~(1 << 2);            // Disable PJ1(PCINT10) Pin Change Interrupt
    sei();
    us_time_diff = us_time_2 - us_time_1;
    us_dist[head_index] = us_time_diff * 1742 / 10000 - 381;
    us_reload_cnt++;
    if (head_index == HEAD_INDEX_MIN || head_index == HEAD_INDEX_MAX) {
      us_reload_flag = 1;
    }
  }
}

//================================================================================================//
// int watch(void) --- detection of ultrasonic distance                                           //
//================================================================================================//
int watch() {
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657; //how far away is the object in cm
  return round(echo_distance);
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
// MEMO : This periodic function calculate motor rotation speed and output log data               //
//================================================================================================//
void flash() {
  interrupts();                   // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  if (start_bit == 1) {
    // Servo command sequence
    head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
    if (head_dir == HEAD_DIR_POS) {
      if (++head_cnt == HEAD_CNT_MAX) head_dir = HEAD_DIR_NEG;
    }
    else if (head_dir == HEAD_DIR_NEG) {
      if (--head_cnt == HEAD_CNT_MIN) head_dir = HEAD_DIR_POS;
    }

    // Ultrasonic sequence
    if (us_status == US_1_WAIT_NEXT) {
      for (i = 0; i <= HEAD_INDEX_MAX; i++) {
        if (head_cnt == us_tx[i]) {
          head_index = i;
          us_ref_cnt = 0;
          digitalWrite(Trig_PIN, LOW);
          delayMicroseconds(5);
          digitalWrite(Trig_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(Trig_PIN, LOW);
          us_time_1 = micros();
          cli();
          PCICR |= (1 << 1);                // Enables PORT J Pin Change Interrupt
          PCMSK1 |= (1 << 2);               // Enables PJ1(PCINT10) Pin Change Interrupt
          sei();
          break;
        }
      }
    }
    else if (us_status == US_2_WAIT_REF) {
      if (++us_ref_cnt == US_REF_CNT_MAX) {
        us_ref_cnt = 0;
        cli();
        PCICR &= ~(1 << 1);             // Disable PORT J Pin Change Interrupt
        PCMSK1 &= ~(1 << 2);            // Disable PJ1(PCINT10) Pin Change Interrupt
        sei();
      }
    }

    // Debug
    if (us_reload_flag == 1) {
      us_reload_flag = 0;
      us_reload_cnt = 0;
      for (i = 0; i <= HEAD_INDEX_MAX ; i++) {
        Serial.print(us_dist[i]);
        Serial.print(",");
      }
      Serial.print(us_reload_cnt);
      Serial.print(",");
      Serial.print(head_cnt);
      Serial.print("\n");
    }

    // Execute motor control if start_bit is 1
    if (start_bit == 1) {
      if (++n == 1000) n = 0;       // Increment sample counter
    }
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
      head_cnt = HEAD_CNT_INIT;
      head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
      delay(100);
    } else if (a == 't') {
      start_bit = 0;
      head_cnt = HEAD_CNT_INIT;
      head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
      delay(100);
    } else if (a == 'u') {
      head_cnt++;
      if (head_cnt >= HEAD_CNT_MAX) head_cnt = HEAD_CNT_MAX;
      head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
    } else if (a == 'd') {
      head_cnt--;
      if (head_cnt <= HEAD_CNT_MIN) head_cnt = HEAD_CNT_MIN;
      head.writeMicroseconds(map(head_cnt, HEAD_CNT_MIN, HEAD_CNT_MAX, SERVO_MIN, SERVO_MAX));
    }
  }
}
