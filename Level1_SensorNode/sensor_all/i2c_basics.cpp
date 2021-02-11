//================================================================================================//
//                                                                                                //
// FILE : i2c_basics.cpp                                                                          //
// MEMO : This library enables you to read and write basic method for I2C                         //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
// Pin usage                                                                                      //
//   20 - SDA                                                                                     //
//   21 - SCL                                                                                     //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#ifndef I2C_BASICS_H

#include "i2c_basics.h"

//================================================================================================//
// init_i2c() --- Initialize I2C                                                                  //
// Argument : long clk  - Clock to use I2C                                                        //
// Returtn  : None                                                                                //
//================================================================================================//
void init_i2c(long clk){
  // Initialize I2C module
  Wire.begin();         // Start I2C module
  Wire.setClock(clk);   // Set I2C clock 400kHz
}

//================================================================================================//
// read_i2c() --- Read bytes from i2c device                                                      //
// Argument : uint8_t addr  - The address of the device to communicate with                       //
//            uint8_t reg   - The register value to send to the i2c device                        //
//            uint8_t n     - The number of bytes to receive from the i2c device                  //
//            uint8_t *data - The data array pointer to write received data                       //
// Returtn  : None                                                                                //
//================================================================================================//
void read_i2c(uint8_t addr, uint8_t reg, uint8_t n, uint8_t* data) {
  uint8_t i = 0;
  Wire.beginTransmission(addr); // Set address of the device
  Wire.write(reg);              // Write the register
  Wire.endTransmission();       // End transmission
  Wire.requestFrom(addr, n);    // Request the i2c device to send n bytes
  while (Wire.available()) data[i++] = Wire.read();
}

//================================================================================================//
// write_i2c() --- Write a bytte to i2c device                                                    //
// Argument : uint8_t addr  - The address of the device to communicate with                       //
//            uint8_t reg   - The register value to send to the i2c device                        //
//            uint8_t n     - The number of bytes to receive from the i2c device                  //
//            uint8_t *data - The data array pointer to write received data                       //
// Returtn  : None                                                                                //
//================================================================================================//
void write_i2c(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr); // Set address of the device
  Wire.write(reg);              // Write the register
  Wire.write(data);             // Write the data
  Wire.endTransmission();       // End transmission
}

#endif // I2C_BASICS_H
