//================================================================================================//
//                                                                                                //
// FILE : L298N.h                                                                                 //
// MEMO : motor control library to control 4 motors using L298N motor driver                      //
//                                                                                                //
// Update Log                                                                                     //
//   2020/08/11 : Start this project                                                              //
//                                                                                                //
// Pin usage                                                                                      //
//   IN1 : digital output                                                                         //
//   IN2 : digital output                                                                         //
//   IN3 : digital output                                                                         //
//   IN4 : digital output                                                                         //
//   ENA : PWM output                                                                             //
//   ENB : PWM output                                                                             //
//                                                                                                //
// Timer Assign                                                                                   //
//   Timer 4 : PWM for Rear motor driver                                                          //
//   Timer 5 : none                                                                               //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef MOTOR_H

#include <inttypes.h>

class L298N {
  public:
    void attach(int pin1, int pin2, int pin3, int pin4, int timer);		// Attach the given pins to the L298N control pins
    void detach();          // Dettach registered pins
    void writeA(int value);	// Write given value to the OCRNA to change PWM duty
    void writeB(int value);	// Write given value to the OCRNB to change PWM duty
    int readA();						// Return current PWM duty of A system
    int readB();						// Return current PWM duty of B system
    bool attached();				// Return true if this motor_control is attached, otherwise false
  private:
    int16_t min;						// minimum is this value
    int16_t max;						// maximum is this value
    uint8_t used_timer;			// The timer number used in this object
    uint8_t IN1;
    uint8_t IN2;
    uint8_t IN3;
    uint8_t IN4;
    uint8_t ENA;
    uint8_t ENB;
    uint8_t isActive;
    volatile uint16_t *pOCRnA;
    volatile uint16_t *pOCRnB;
};

#endif	// MOTOR_H
