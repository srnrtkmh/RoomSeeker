//================================================================================================//
//                                                                                                //
// FILE : rotary_encoders.h                                                                       //
// MEMO : This library enables you to get rotary encoder value with pin change interrupt          //
//        Up to 4 encoders can be used in this library                                            //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project                                                              //
//   2020/11/03 : Added Arduino Mega 2560 process                                                 //
//                                                                                                //
// Pin usage                                                                                      //
//   IN1 : digital output                                                                         //
//   IN2 : digital output                                                                         //
//   IN3 : digital output                                                                         //
//   IN4 : digital output                                                                         //
//   IN5 : digital output                                                                         //
//   IN6 : digital output                                                                         //
//   IN7 : digital output                                                                         //
//   IN8 : digital output                                                                         //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef ROTARY_ENCODER_H

#include <inttypes.h>

//#define DEBUG
//#define _usePCI0
//#define _usePCI1
#define _usePCI2

#define SINGLE_PHASE 1
#define DUAL_MULTI   2
#define QUAD_MULTI   4

class RotaryEncoders {
  public:
    uint8_t attach(int n, uint8_t pin1, uint8_t pin2, uint8_t mode);  // Attach the given pins to the rotary encoder phase A and B
    void detach(void);                  // Dettach registered pins
    uint16_t read_cnt(uint8_t n);       // Return current counted value
    int16_t read_vel(uint8_t n);        // Return current angular velocity value
    int16_t read_test_cnt(void);
    bool attached(uint8_t n);           // Return true if this motor_control is attached, otherwise false
  private:
    uint8_t isActive[4];
    uint8_t pinA[4];
    uint8_t pinB[4];
    uint8_t mode[4];
};

#endif // ROTARY_ENCODER_H
