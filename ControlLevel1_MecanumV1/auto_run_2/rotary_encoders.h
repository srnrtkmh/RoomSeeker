//================================================================================================//
//                                                                                                //
// FILE : rotary_encoders.h                                                                       //
// MEMO : This library enables you to get rotary encoder value with pin change interrupt          //
//        Up to 4 encoders can be used in this library                                            //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project                                                              //
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

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  #define _usePCI0
//  #define _usePCI1
  #define _usePCI2
#endif

class RotaryEncoders {
  public:
    uint8_t attach(int n, uint8_t pin1, uint8_t pin2); // Attach the given pins to the rotary encoder phase A and B
    void detach(void);                  // Dettach registered pins
    uint16_t read_cnt(uint8_t n);       // Return current counted value
    int16_t read_vel(uint8_t n);        // Return current angular velocity value
    bool attached(uint8_t n);           // Return true if this motor_control is attached, otherwise false
  private:
    uint8_t enc1_a_pin, enc1_b_pin;
    uint8_t enc2_a_pin, enc2_b_pin;
    uint8_t enc3_a_pin, enc3_b_pin;
    uint8_t enc4_a_pin, enc4_b_pin;
    uint8_t isActive;
};

#endif // ROTARY_ENCODER_H
