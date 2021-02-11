//================================================================================================//
//                                                                                                //
// FILE : rotary_encoders.h                                                                       //
// MEMO : This library enables you to get rotary encoder value with pin change interrupt          //
//        Up to 4 encoders can be used in this library                                            //
//                                                                                                //
// Update Log                                                                                     //
//   2020/09/05 : Start this project                                                              //
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

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#if defined(ARDUINO_ARCH_AVR)

#include <avr/interrupt.h>
#include <Arduino.h>
#include "rotary_encoders.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
uint8_t enc_isActive[4] = {false, false, false, false};
uint8_t enc_a_pin[4] = {0, 0, 0, 0}, enc_b_pin[4] = {0, 0, 0, 0};
uint8_t enc_a_now[4] = {0, 0, 0, 0}, enc_a_pre[4] = {0, 0, 0, 0};
uint8_t enc_b_now[4] = {0, 0, 0, 0}, enc_b_pre[4] = {0, 0, 0, 0};
uint16_t enc_cnt[4] = {0, 0, 0, 0};
uint16_t enc_cnt_pre[4] = {0, 0, 0, 0};
int16_t enc_vel[4] = {0, 0, 0, 0};
uint16_t test_count = 0;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
PROGMEM const uint8_t d2pcint[70] = {
  8,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  4,  5,  6,  7, 10,  9,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  3,  2,  1,  0,  0,  0,
  0,  0,  0,  0,  0,  0, 16, 17,
  18, 19, 20, 21, 22, 23
};
#endif
#if defined(__AVR_ATmega328__) || (__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__)
PROGMEM const uint8_t d2pcint[24] = {16, 17, 18, 19, 20, 21, 22, 23, 0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 0, 0, 0, 0};
#endif

//================================================================================================//
// Pin Change Interrupt Function                                                                  //
// MEMO : This function counts the encoder pulse                                                  //
//================================================================================================//
static inline void interrupt_handler() {
  uint8_t i;

  // Get current input value of the each encoder output
  noInterrupts();
  for (i = 0; i < 4; i++) {
    if (enc_isActive[i] == true) {
      enc_a_now[i] = digitalRead(enc_a_pin[i]);
      enc_b_now[i] = digitalRead(enc_b_pin[i]);
    }
  }
  interrupts();

  // If the previous and the current state of the phase A are different, that means a Pulse has occured
  // If the phase B state is different to the phase A state, that means the encoder is rotating clockwise
  for (i = 0; i < 4; i++) {
    if (enc_a_now[i] != enc_a_pre[i]) {
      if (enc_a_now[i] == 0) {
        if (enc_b_now[i] == 0) enc_cnt[i]--;
        else                   enc_cnt[i]++;
      } else {
        if (enc_b_now[i] == 0) enc_cnt[i]++;
        else                   enc_cnt[i]--;
      }
    }
    else if (enc_b_now[i] != enc_b_pre[i]) {
      if (enc_b_now[i] == 0) {
        if (enc_a_now[i] == 0) enc_cnt[i]++;
        else                   enc_cnt[i]--;
      } else {
        if (enc_a_now[i] == 0) enc_cnt[i]--;
        else                   enc_cnt[i]++;
      }
    }
    enc_a_pre[i] = enc_a_now[i];
    enc_b_pre[i] = enc_b_now[i];
  }
  test_count++;
}

ISR(PCINT0_vect) {
  interrupts();
  interrupt_handler();
}

ISR(PCINT1_vect) {
  interrupts();
  interrupt_handler();
}

ISR(PCINT2_vect) {
  interrupts();
  interrupt_handler();
}

//================================================================================================//
// Attach the given pins to the rotary encoder input pins                                         //
//================================================================================================//
uint8_t RotaryEncoders::attach(int n, uint8_t pin1, uint8_t pin2, uint8_t mode) {
  uint8_t pin1_pcint = 0, pin2_pcint = 0;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#ifdef _usePCI0
#endif
#ifdef _usePCI1
#endif
#ifdef _usePCI2
  if (0 <= n && n <= 3) {
    if (62 <= pin1 && pin1 <= 69 && 62 <= pin2 && pin2 <= 69) {
      // pin direction settings
      pinMode(pin1, INPUT);
      pinMode(pin2, INPUT);
      enc_a_pin[n] = pin1;
      enc_b_pin[n] = pin2;
      enc_isActive[n] = true;
      this->isActive = true;

      // Initialize Pin change interrupt
      noInterrupts();
      PCICR = _BV(PCIE2);                               // Enables PORT K Pin Change Interrupt
      PCMSK2 |= (1 << (pin1 - 62)) | (1 << (pin2 - 62));  // Enables all PORT K Pin Change Interrupt
      interrupts();

      return (0);
    } else {
      return (1);
    }
  } else {
    return (1);
  }
#endif
#endif
#if defined(__AVR_ATmega328__) || (__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__)
  if (0 <= n && n <= 3) {
    if (pin1 <= 23 && pin2 <= 23) {
      // pin direction settings
      pinMode(pin1, INPUT);
      pinMode(pin2, INPUT);
      enc_a_pin[n] = pin1;
      enc_b_pin[n] = pin2;
      // enc_isActive[n] = true;

      // Initialize Pin change interrupt
      noInterrupts();
      PCICR |= _BV(PCIE2) | _BV(PCIE1) | _BV(PCIE0);       // Enables PORT B, C, D Pin Change Interrupt
      pin1_pcint = d2pcint[pin1];
      if (pin1_pcint <= 7) {
        PCMSK0 |= (1 << pin1_pcint);
      }
      else if (8 <= pin1_pcint && pin1_pcint <= 15) {
        PCMSK1 |= (1 << (pin1_pcint - 8));
      }
      else if (16 <= pin1_pcint && pin1_pcint <= 23) {
        PCMSK2 |= (1 << (pin1_pcint - 16));
      }

      if (mode == QUAD_MULTI) {
        pin2_pcint = d2pcint[pin2];
        if (pin2_pcint <= 7) {
          PCMSK0 |= (1 << pin2_pcint);
        }
        else if (8 <= pin2_pcint && pin2_pcint <= 15) {
          PCMSK1 |= (1 << (pin2_pcint - 8));
        }
        else if (16 <= pin2_pcint && pin2_pcint <= 23) {
          PCMSK2 |= (1 << (pin2_pcint - 16));
        }
      }
      interrupts();

#ifdef DEBUG
      Serial.print(PCICR, HEX);
      Serial.print(",");
      Serial.print(PCMSK0, HEX);
      Serial.print(",");
      Serial.print(PCMSK1, HEX);
      Serial.print(",");
      Serial.print(PCMSK2, HEX);
      Serial.print("\n");
#endif
      return (0);
    } else {
      return (1);
    }
  } else {
    return (1);
  }
#endif
}

//================================================================================================//
// Dettach registered pins                                                                        //
//================================================================================================//
void RotaryEncoders::detach(void) {
  // Future Works
}

//================================================================================================//
// Return current counted value                                                                   //
//================================================================================================//
uint16_t RotaryEncoders::read_cnt(uint8_t n) {
  return enc_cnt[n];
}

//================================================================================================//
// Return current angular velocity value                                                          //
//================================================================================================//
int16_t RotaryEncoders::read_vel(uint8_t n) {
  enc_vel[n] = (long)enc_cnt[n] - (long)enc_cnt_pre[n];
  enc_cnt_pre[n] = enc_cnt[n];
  return (enc_vel[n]);
}

//================================================================================================//
//================================================================================================//
int16_t RotaryEncoders::read_test_cnt(void) {
  return (test_count);
}

//================================================================================================//
// Return true if this motor_control is attached, otherwise false                                 //
//================================================================================================//
bool RotaryEncoders::attached(uint8_t n) {
  return (enc_isActive[n]);
}

#endif // ARDUINO_ARCH_AVR
