//================================================================================================//
//                                                                                                //
// FILE : auto_run_1.ino                                                                          //
// MEMO : get ultra sonic distance sensor value in front of the robot                             //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project based on previous project "us_sensor_2.ino"                  //
//                Pin assign changed to adapt motor control library                               //
//                motor control object added                                                      //
//                rotary encoder object added                                                     //
//                acceleration object added                                                       //
//                setting.h　added                                                                //
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
#include <TimerFive.h>
#include <WiFiEsp.h>
#include <WiFiEspUDP.h>
#include <Servo.h>
#include "L298N.h"
#include "rotary_encoders.h"
#include "acceleration.h"
#include "setting.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For Encoder
const uint8_t FR_ENC_A = 62;
const uint8_t FR_ENC_B = 63;
const uint8_t FL_ENC_A = 64;
const uint8_t FL_ENC_B = 65;
const uint8_t RR_ENC_A = 66;
const uint8_t RR_ENC_B = 67;
const uint8_t RL_ENC_A = 68;
const uint8_t RL_ENC_B = 69;
const uint8_t FR_ENC = 0;
const uint8_t FL_ENC = 1;
const uint8_t RR_ENC = 2;
const uint8_t RL_ENC = 3;
RotaryEncoders enc;

// For Motor - index 0:FR, 1:FL, 2:RR, 3:RL
// Port pin settings for Front Right motor
const byte speedPinR = 5;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte RightMotorDirPin1  = 22;             // Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
const byte RightMotorDirPin2  = 23;             // Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)

// Port pin settings for Front Left motor
const byte speedPinL = 2;                       // Front Wheel PWM pin connect Right MODEL-X ENA
const byte LeftMotorDirPin1  = 24;              // Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
const byte LeftMotorDirPin2  = 25;              // Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)

// Port pin settings for Rear Right motor
const byte speedPinRB = 6;                      // Rear Wheel PWM pin connect Left MODEL-X ENA
const byte RightMotorDirPin1B = 26;             // Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
const byte RightMotorDirPin2B = 27;             // Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1)

// Port pin settings for Rear Left motor
const byte speedPinLB = 7;                      // Rear Wheel PWM pin connect Left MODEL-X ENB
const byte LeftMotorDirPin1B = 28;              // Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
const byte LeftMotorDirPin2B = 29;              // Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
L298N front_motor, rear_motor;

// Control variables
int8_t cnt_dir[4] = {1, -1, 1, -1};             // Direction correction co-efficient of rotation
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
long Kp[4] = {3, 3, 3, 3};                      // P gain for PI control
long Ki[4] = {5, 5, 5, 5};                      // I gain for PI control
int16_t vout[4] = {0, 0, 0, 0};                 // Voltage output for 4 motor drivers - index 1, 2, 4 : [0 - 1023], index3 : [0 - 255]
int16_t vout_ff[4] = {0, 0, 0, 0};              // Feedforward control output
int16_t vout_fb[4] = {0, 0, 0, 0};              // Feedback control output
int16_t vout_ll[4] = { -1000, -1000, -1000, -1000};  // Lower limit of voltage output
int16_t vout_ul[4] = {1000, 1000, 1000, 1000};       // Upper limit of voltage output

// Feedforward parameters
//long Fc_p[4] = { 536,  505,  73,  464};         // Coulonb friction compensation parameter for positive direction
//long Fc_n[4] = {-542, -482, -69, -461};         // Coulonb friction compensation parameter for negative direction
long Fc_p[4] = { 524,  519,  518,  494};        // Coulonb friction compensation parameter for positive direction
long Fc_n[4] = { -532, -519, -512, -495};       // Coulonb friction compensation parameter for negative direction
long Fd_p_x100[4] = {70, 79, 20, 82};           // Dynamic friction compensation parameter for negative direction
long Fd_n_x100[4] = {73, 77, 24, 74};           // Dynamic friction compensation parameter for negative direction

// Acceleration parameters
uint8_t acc_stat = 0;
long omega_cmd_const_x10 = 2000;
long acc_time_ms = 2000;
acceleration acc1, acc2;
long test_wait_cnt = 0;

// For ultra sonic sensor
const uint8_t SERVO_PIN = 31; //servo connect
const uint8_t Echo_PIN  = 14; // Ultrasonic Echo pin connect
const uint8_t Trig_PIN  = 30; // Ultrasonic Trig pin connect
Servo head;
String str;
char str2[128];
uint8_t HEAD_1_WAIT_NEXT = 1;
uint8_t HEAD_2_WAIT_SERVO_MOVE = 2;
uint8_t HEAD_3_SHOT_US = 3;
uint8_t HEAD_4_WAIT_US_REF = 4;
uint8_t HEAD_5_DIST_CALC = 5;
uint8_t head_status = HEAD_1_WAIT_NEXT;
uint8_t head_angle = 90;
uint8_t head_index = 9;
uint8_t head_index_max = 17;
uint8_t head_index_min = 0;
uint8_t head_b = 0;
uint8_t head_a = 10;
unsigned long head_servo_delay = 100000;
unsigned long head_us_time_out = 30000;
uint8_t head_srv_dir = 0;
unsigned long head_time_1 = 0, head_time_2 = 0;
unsigned long echo_dist_val = 0;
int dist[200] = {0};
int us_start_bit = 0;
int obstacle_status = 0;

// Useful constants for calculation
long sampling_time = 9999;                      // sampling time in micro seconds
long sampling_time_ms = 10;                     // sampling_time in milli seconds
long dt1000 = 10;                               // (sampling_time / 1,000,000) * 1000 [10^-3 sec]
long _dt_x10 = 1000;                              // (sampling_time / 1,000,000)^-1 [10^-1 sec^-1]
const long pi100 = 314;                         // Pi x 100

// Sample number
long n = 0;                                     // サンプルカウンタ変数
byte start_bit = 0;                             // サンプリング開始ビット

// For WiFi
/*char ssid[] = "HG8045-579E-bg"; // replace ****** with your network SSID (name)
  char pass[] = "mgrvy6kr";       // replace ****** with your network password
  int status = WL_IDLE_STATUS;    //
  char packetBuffer[5];           // use a ring buffer to increase speed and reduce memory allocation

  WiFiEspUDP Udp;                 //
  unsigned int localPort = 8890;  // local port to listen on
  static const char *udpReturnAddr = "192.168.1.100";
  static const int udpReturnPort = 8890;
*/

//================================================================================================//
// Serial print functions to transmit debug and log data                                          //
//================================================================================================//
#include "print.h"

//================================================================================================//
// go_advance(int speed) --- drive all motors with the same PWM value                             //
//   Argument : speed - PWM value                                                                 //
//   Return   : none                                                                              //
//================================================================================================//
void go_advance(int speed) {
  front_motor.writeA(speed);
  front_motor.writeB(speed);
  rear_motor.writeA(speed);
  rear_motor.writeB(speed);
}

//================================================================================================//
// stop_Stop(void) --- Stop all motors                                                            //
//================================================================================================//
void stop_Stop(void) {
  front_motor.writeA(0);
  front_motor.writeB(0);
  rear_motor.writeA(0);
  rear_motor.writeB(0);
}

//================================================================================================//
// printWifiStatus(void)                                                                          //
//================================================================================================//
void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
  Serial.println();
}

//================================================================================================//
// gpio_init() --- Initialize GPIO input/output settings                                          //
// Argument : none                                                                                //
// Return   : none                                                                                //
//================================================================================================//
void gpio_init() {
  // HC-SR04 setting
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  gpio_init();                      // Initialize GPIO
  Serial.begin (230400);            // Initialize Serial

  // Initialize motor driver
  front_motor.attach(RightMotorDirPin1, RightMotorDirPin2, LeftMotorDirPin1, LeftMotorDirPin2, 3);
  rear_motor.attach(RightMotorDirPin1B, RightMotorDirPin2B, LeftMotorDirPin1B, LeftMotorDirPin2B, 4);
  stop_Stop();

  // エンコーダ各ピンの初期値を設定
  enc.attach(0, FR_ENC_A, FR_ENC_B);
  enc.attach(1, FL_ENC_A, FL_ENC_B);
  enc.attach(2, RR_ENC_A, RR_ENC_B);
  enc.attach(3, RL_ENC_A, RL_ENC_B);

  // 加速オブジェクトの設定
  acc1.init(0, omega_cmd_const_x10, acc_time_ms, sampling_time_ms);
  acc2.init(omega_cmd_const_x10, 0, acc_time_ms, sampling_time_ms);

  // Timer1割込の設定
  Timer1.initialize(sampling_time); // サンプリングタイムを設定して初期化
  Timer1.attachInterrupt(flash);    // 割り込み関数を登録

  // Setup WiFi
  /*  Serial1.begin(115200);
    Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
    delay(200);
    Serial1.write("AT+RST\r\n");
    delay(200);
    Serial1.begin(9600);  // initialize serial for ESP module
    WiFi.init(&Serial1);    // initialize ESP module

    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      // don't continue
      while (true);
    }

    // attempt to connect to WiFi network
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);
    }

    Serial.println("You're connected to the network");
    printWifiStatus();
    Udp.begin(localPort);

    Serial.print("Listening on port ");
    Serial.println(localPort);
  */
  // Initialize us sensor servo
  digitalWrite(Trig_PIN, LOW);
  head.attach(SERVO_PIN);
  head.write(90);
  delay(500);
}

//================================================================================================//
// Pin Change Interrupt Function - Port J                                                         //
// MEMO : This function counts the encoder pulse                                                  //
//================================================================================================//
ISR(PCINT1_vect) {
  interrupts();
  if (digitalRead(Echo_PIN) == LOW) {
    head_time_2 = micros();
    head_status = HEAD_5_DIST_CALC;
#ifdef DEBUG
    print_us();
#endif
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
  int i;
  long tmp;
  interrupts();                   // 次回の割り込みを許可(これがないと次のタイマ割り込み周期がずれる)

  // Execute motor control if start_bit is 1
  if (start_bit == 1) {
    // Acceleration sequence
    // 0. 停止
    if (acc_stat == 0) {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] = 0;
      }
      if (obstacle_status <= 1) {
        test_wait_cnt++;
      } else {
        test_wait_cnt = 0;
      }
      
      if (test_wait_cnt >= 100) {
        test_wait_cnt = 0;
        acc_stat = 1;
      }
    }
    // 1. 加速度テーブルによって加速
    else if (acc_stat == 1) {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] = acc1.get_cmd(acc1.n++);
      }
      if (acc1.n >= acc1.acc_num) {
        acc1.clear();
        acc_stat = 2;
      }
    }
    // 2. 定常速度運転
    else if (acc_stat == 2) {
      // test_wait_cnt++;
      // if (test_wait_cnt >= 200) {
      //   test_wait_cnt = 0;
      //   acc_stat = 3;
      // }
      if (obstacle_status == 0) {
        for (i = 0; i < 4; i++) {
          omega_cmd_x10[i] = omega_cmd_const_x10;
        }
      }
      else if (obstacle_status == 1) {
        omega_cmd_x10[FR_ENC] = omega_cmd_const_x10 - 1000;
        omega_cmd_x10[FL_ENC] = omega_cmd_const_x10 + 1000;
        omega_cmd_x10[RR_ENC] = omega_cmd_const_x10 - 1000;
        omega_cmd_x10[RL_ENC] = omega_cmd_const_x10 + 1000;
      }
      else if (obstacle_status == 2) {
        omega_cmd_x10[FR_ENC] = omega_cmd_const_x10 - 1500;
        omega_cmd_x10[FL_ENC] = omega_cmd_const_x10;
        omega_cmd_x10[RR_ENC] = omega_cmd_const_x10 - 1500;
        omega_cmd_x10[RL_ENC] = omega_cmd_const_x10;
      }
      else if (obstacle_status == 3) {
        omega_cmd_x10[FR_ENC] = omega_cmd_const_x10 - 3000;
        omega_cmd_x10[FL_ENC] = omega_cmd_const_x10 - 1000;
        omega_cmd_x10[RR_ENC] = omega_cmd_const_x10 - 3000;
        omega_cmd_x10[RL_ENC] = omega_cmd_const_x10 - 1000;
      }
      else if (obstacle_status == 4) {
        acc_stat = 3;
      }
    }
    // 3. 減速度テーブルによって減速
    else if (acc_stat == 3) {
      for (i = 0; i < 4; i++) {
        omega_cmd_x10[i] = acc2.get_cmd(acc2.n++);
      }
      if (acc2.n >= acc2.acc_num) {
        acc2.clear();
        acc_stat = 0;
      }
    }

    // Update encoder counter value
    for (i = 0; i < 4; i++) cnt_pre[i] = cnt_now[i];
    cnt_now[0] = cnt_dir[0] * enc.read_cnt(FR_ENC);
    cnt_now[1] = cnt_dir[1] * enc.read_cnt(FL_ENC);
    cnt_now[2] = cnt_dir[2] * enc.read_cnt(RR_ENC);
    cnt_now[3] = cnt_dir[3] * enc.read_cnt(RL_ENC);

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

#ifdef LOG
    print_log();                  // Output log data to upper system
#endif
    if (++n > 999999) n = 0;      // Increment sample counter
  }
  // Control output is zero if start_bit is not 1
  else {
    for (i = 0; i < 4; i++) vout[i] = 0;
  }

  // Apply control output to the motor
  front_motor.writeA(vout[0]);
  front_motor.writeB(vout[1]);
  rear_motor.writeA(vout[2]);
  rear_motor.writeB(vout[3]);
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
#ifdef LOG
      print_label();
#endif
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      delay(50);
      start_bit = 1;
      us_start_bit = 1;
    } else if (a == 't') {
      for (i = 0; i < 4; i++) omega_cmd_x10[i] = 0;
      start_bit = 0;
      us_start_bit = 0;
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
    } else if (a == 'k') {
      us_start_bit = 1;
    }
  }

  /*  int packetSize = Udp.parsePacket();
    if (packetSize) {                               // if you get a client,
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      char c = packetBuffer[0];
      switch (c) {
        case 'A': Serial.print("A received\n"); break;
        case 'B':
          Udp.beginPacket(udpReturnAddr, udpReturnPort);
          Udp.write("ok");
          Udp.endPacket();
      }
    }
  */
  // 1. 次に測定するヘッドの角度に設定
  if (us_start_bit == 1) {
    if (head_status == HEAD_1_WAIT_NEXT) {
      if (head_srv_dir == 0) {
        head_index++;
      } else if (head_srv_dir == 1) {
        head_index--;
      }
      head_angle = head_index * head_a + head_b;
      head.write(head_angle);
      head_time_1 = micros();
      head_status = HEAD_2_WAIT_SERVO_MOVE;
    }
    // 2. 次に測定するヘッド位置に動くまで決め打ち時間分待つ
    else if (head_status == HEAD_2_WAIT_SERVO_MOVE) {
      head_time_2 = micros();
      if (head_time_2 - head_time_1 >= head_servo_delay) {
        head_status = HEAD_3_SHOT_US;
      }
    }
    // 3. 超音波を発射する　→　次の処理はピンチェンジインタラプトの割り込み処理
    else if (head_status == HEAD_3_SHOT_US) {
      digitalWrite(Trig_PIN, LOW);
      delayMicroseconds(5);
      digitalWrite(Trig_PIN, HIGH);
      delayMicroseconds(15);
      digitalWrite(Trig_PIN, LOW);
      head_time_1 = micros();
      cli();
      PCICR |= (1 << 1);                // Enables PORT J Pin Change Interrupt
      PCMSK1 |= (1 << 2);               // Enables PJ1(PCINT10) Pin Change Interrupt
      sei();
      head_status = HEAD_4_WAIT_US_REF;
    }
    // 4. 超音波の反射を待つ
    else if (head_status == HEAD_4_WAIT_US_REF) {
      head_time_2 = micros();
      if (head_time_2 - head_time_1 >= head_us_time_out) {
        cli();
        PCICR &= ~(1 << 1);             // Enables PORT K Pin Change Interrupt
        PCMSK1 &= ~(1 << 2);            // Disables all PORT J Pin Change Interrupt
        sei();
        head_status = HEAD_5_DIST_CALC;
      }
    }
    // 5. 超音波の反射時間を取得できたら時間差を距離に換算する
    else if (head_status == HEAD_5_DIST_CALC) {
      echo_dist_val = head_time_2 - head_time_1;    // time difference between transmit & receive us
      dist[head_index] = echo_dist_val * 1742 / 10000 - 381;
      if (head_index <= head_index_min || head_index_max <= head_index) {
        if (head_index >= head_index_max)      head_srv_dir = 1;
        else if (head_index <= head_index_min) head_srv_dir = 0;
        str = "";
        for (i = 0; i <= head_index_max; i++) {
          str += String(dist[i], DEC) + ",";
        }
        str.toCharArray(str2, sizeof(str2));
        Serial.print(str2);
        Serial.print(obstacle_status);
        Serial.print(",");
        Serial.print(acc_stat);
        Serial.print("\n");
        /*    Udp.beginPacket(udpReturnAddr, udpReturnPort);
            Udp.write(str2);
            Udp.endPacket();
        */
      }
      head_status = HEAD_1_WAIT_NEXT;

      // Test
      noInterrupts();
      obstacle_status = 0;
      for (i = 0; i <= head_index_max; i++) {
        if (6 <= i && i <= 11) {
          if (dist[i] <= 150) {
            obstacle_status = 4;
          } else if (dist[i] <= 250) {
            if (obstacle_status < 3) obstacle_status = 3;
          } else if (dist[i] <= 400) {
            if (obstacle_status < 2) obstacle_status = 2;
          } else if (dist[i] <= 600) {
            if (obstacle_status < 1) obstacle_status = 1;
          }
        }
      }
      interrupts();
    }
  }
}
