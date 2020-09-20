//================================================================================================//
//                                                                                                //
// FILE : L298N.cpp                                                                               //
// MEMO : Motor control library to control 4 motors using L298N motor driver                      //
//        timer 1, 3, 4, 5 (16bit timer) in this library                               //
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
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#if defined(ARDUINO_ARCH_AVR)

#include <avr/interrupt.h>
#include <Arduino.h>
#include "L298N.h"

//================================================================================================//
// Attach the given pins to the L298N control pins                                                //
//================================================================================================//
void L298N::attach(int pin1, int pin2, int pin3, int pin4, int timer){
	pinMode(pin1, OUTPUT);
	pinMode(pin2, OUTPUT);
	pinMode(pin3, OUTPUT);
	pinMode(pin4, OUTPUT);
	this->IN1 = pin1;
	this->IN2 = pin2;
	this->IN3 = pin3;
	this->IN4 = pin4;

	if(timer == 1){
		pinMode(11, OUTPUT);			// OC1A pin is 11th pin in arduino mega 2560
		pinMode(12, OUTPUT);			// OC1B pin is 12th pin in arduino mega 2560
		OCR1A = 0;						// Clear OCR1A
		OCR1B = 0;						// Clear OCR1B
		OCR1C = 0;						// Clear OCR1C
		TCNT1 = 0;						// Clear timer 1
		TIMSK1 = 0x00;					// None interrupts
		TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1) | _BV(WGM11) | _BV(WGM10);	// OC1A, OC1B, OC1C is none inverting mode, PWM mode is Fast PWM 10bit
		TCCR1B = _BV(WGM12)  | _BV(CS10);											// PWM mode is Fast PWM, Clock is clkI/O/1 (no prescaling)
		TCCR1C = 0x00;					// Force compare match is disabled
		this->used_timer = 1;
		this->pOCRnA = &OCR1A;
		this->pOCRnB = &OCR1B;
		this->min = -1000;
		this->max = 1000;
	}
	else if(timer == 3){
		pinMode(5, OUTPUT);				// OC3A pin is 5th pin in arduino mega 2560
		pinMode(2, OUTPUT);				// OC3B pin is 2nd pin in arduino mega 2560
		OCR3A = 0;						// Clear OCR3A
		OCR3B = 0;						// Clear OCR3B
		OCR3C = 0;						// Clear OCR3C
		TCNT3 = 0;						// Clear timer 3
		TIMSK3 = 0x00;					// None interrupts
		TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31) | _BV(WGM30);	// OC3A, OC3B, OC3C is none inverting mode, PWM mode is Fast PWM 10bit
		TCCR3B = _BV(WGM32)  | _BV(CS30);											// PWM mode is Fast PWM, Clock is clkI/O/1 (no prescaling)
		TCCR3C = 0x00;					// Force compare match is disabled
		this->used_timer = 3;			// Timer 3 is used
		this->pOCRnA = &OCR3A;
		this->pOCRnB = &OCR3B;
		this->min = -1000;
		this->max = 1000;
	}
	else if(timer == 4){
		pinMode(6, OUTPUT);				// OC4A pin is 6th pin in arduino mega 2560
		pinMode(7, OUTPUT);				// OC4B pin is 7th pin in arduino mega 2560
		OCR4A = 0;						// Clear OCR4A
		OCR4B = 0;						// Clear OCR4B
		OCR4C = 0;						// Clear OCR4C
		TCNT4 = 0;						// Clear timer 0
		TIMSK4 = 0x00;					// None interrupts
		TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41) | _BV(WGM40);	// OC4A, OC4B, OC4C is none inverting mode, PWM mode is Fast PWM 10bit
		TCCR4B = _BV(WGM42)  | _BV(CS40);											// PWM mode is Fast PWM, Clock is clkI/O/1 (no prescaling)
		TCCR4C = 0x00;					// Force compare match is disabled
		this->used_timer = 4;
		this->pOCRnA = &OCR4A;
		this->pOCRnB = &OCR4B;
		this->min = -1000;
		this->max = 1000;
	}
	else if(timer == 5){
		pinMode(46, OUTPUT);			// OC5A pin is 46th pin in arduino mega 2560
		pinMode(45, OUTPUT);			// OC5B pin is 45th pin in arduino mega 2560
		OCR5A = 0;						// Clear OCR4A
		OCR5B = 0;						// Clear OCR4B
		OCR5C = 0;						// Clear OCR4C
		TCNT5 = 0;						// Clear timer 0
		TIMSK5 = 0x00;					// None interrupts
		TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51) | _BV(WGM50);	// OC5A, OC5B, OC5C is none inverting mode, PWM mode is Fast PWM 10bit
		TCCR5B = _BV(WGM52)  | _BV(CS50);											// PWM mode is Fast PWM, Clock is clkI/O/1 (no prescaling)
		TCCR5C = 0x00;					// Force compare match is disabled
		this->used_timer = 5;
		this->pOCRnA = &OCR5A;
		this->pOCRnB = &OCR5B;
		this->min = -1000;
		this->max = 1000;
	}
}

//================================================================================================//
// Dettach registered pins                                                                        //
//================================================================================================//
void L298N::detach(void){
}

//================================================================================================//
// Write given value to the OCRNA to change PWM duty                                              //
//================================================================================================//
void L298N::writeA(int value){
	if(value > 0){
		digitalWrite(this->IN1, HIGH);
		digitalWrite(this->IN2, LOW);
		if(value > max) value = max;
		*(this->pOCRnA) = value;
	}else if(value < 0){
		digitalWrite(this->IN1, LOW);
		digitalWrite(this->IN2, HIGH);
		if(value < min) value = min;
		*(this->pOCRnA) = -value;
	}else{
		digitalWrite(this->IN1, LOW);
		digitalWrite(this->IN2, LOW);
		*(this->pOCRnA) = 0;
	}
}

//================================================================================================//
// Write given value to the OCRNB to change PWM duty                                              //
//================================================================================================//
void L298N::writeB(int value){
	if(value > 0){
		digitalWrite(this->IN3, HIGH);
		digitalWrite(this->IN4, LOW);
		if(value > max) value = max;
		*(this->pOCRnB) = value;
	}else if(value < 0){
		digitalWrite(this->IN3, LOW);
		digitalWrite(this->IN4, HIGH);
		if(value < min) value = min;
		*(this->pOCRnB) = -value;
	}else{
		digitalWrite(this->IN3, LOW);
		digitalWrite(this->IN4, LOW);
		*(this->pOCRnB) = 0;
	}
}

//================================================================================================//
// Return current PWM duty of A system                                                            //
//================================================================================================//
int L298N::readA(void){
	return(*(this->pOCRnA));
}

//================================================================================================//
// Return current PWM duty of B system                                                            //
//================================================================================================//
int L298N::readB(void){
	return(*(this->pOCRnB));
}

//================================================================================================//
// Return true if this motor_control is attached, otherwise false                                 //
//================================================================================================//
bool L298N::attached(){
	return(this->isActive);
}

#endif	// ARDUINO_ARCH_AVR
