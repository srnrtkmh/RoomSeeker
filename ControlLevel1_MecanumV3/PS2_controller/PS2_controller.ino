//================================================================================================//
//                                                                                                //
// FILE : motor_drive.ino                                                                         //
// MEMO : motor drive test (no control, drive with constant pwm duty)                             //
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
//   54 - A0 Battery voltage / 2                  55 - A1 PS2_DAT (DO for PS2 controller)         //
//   56 - A2 PS2_SEL (DO for PS2 controller)      57 - A3 PS2_CMD (DO for PS2 controller)         //
//   58 - A4 PS2_CLK (DO for PS2 controller)      59 - A5 NC                                      //
//   60 - A6 NC                                   61 - A7 NC                                      //
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
#include "PS2X_lib.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For PS2
PROGMEM const uint8_t PS2_DAT = 55;                 // DAT pin settings
PROGMEM const uint8_t PS2_SEL = 56;                 // SEL pin settings
PROGMEM const uint8_t PS2_CMD = 57;                 // CMD pin settings
PROGMEM const uint8_t PS2_CLK = 58;                 // CLK pin settings
PROGMEM const bool pressures = true;                //
PROGMEM const bool rumble    = true;                //
byte ps2_error = 0;                                 //
byte ps2_type = 0;                                  //
byte ps2_vibrate = 0;                               //
uint8_t ps2_ctrl = 0;                               // This flag shows command from PS2 controller is enable or not
PS2X ps2x;                                          // create PS2 Controller Object

// Reset func
void (* resetFunc) (void) = 0;                      //

// Useful constants for calculation
long sampling_time = 10000;                     // サンプリング時間[us]

// Sample number
long n = 0;                                     // サンプルカウンタ変数

// For sequence
byte start_bit = 0;                             // サンプリング開始ビット

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Serial.begin (230400);    // Initialize Serial
  delay(100);               //added delay to give wireless ps2 module some time to startup, before configuring it

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
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  // Reading PS2 Controller key ------------------------------------------------------------------//
  if (ps2_error == 1) { //skip loop if no controller found
    resetFunc();
  }

  if (ps2_type != 2) {    //DualShock Controller
    ps2x.read_gamepad(false, ps2_vibrate); //read controller and set large motor to spin at 'vibrate' speed

    if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");

    if (ps2x.Button(PSB_PAD_UP)) {     //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }

    ps2_vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");
    }

    if (ps2x.ButtonPressed(PSB_CIRCLE))              //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if (ps2x.NewButtonState(PSB_CROSS))              //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if (ps2x.ButtonReleased(PSB_SQUARE))             //will be TRUE if button was JUST released
      Serial.println("Square just released");

    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }
  }
  delay(50);
}
