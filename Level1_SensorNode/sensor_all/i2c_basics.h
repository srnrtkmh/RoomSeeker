//================================================================================================//
//                                                                                                //
// FILE : i2c_basics.h                                                                            //
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

#include <Wire.h>

//================================================================================================//
// init_i2c() --- Initialize I2C                                                                  //
// Argument : long clk  - Clock to use I2C                                                        //
// Returtn  : None                                                                                //
//================================================================================================//
void init_i2c(long clk);

//================================================================================================//
// read_i2c() --- Read bytes from i2c device                                                      //
// Argument : uint8_t addr  - The address of the device to communicate with                       //
//            uint8_t reg   - The register value to send to the i2c device                        //
//            uint8_t n     - The number of bytes to receive from the i2c device                  //
//            uint8_t *data - The data array pointer to write received data                       //
// Returtn  : None                                                                                //
//================================================================================================//
void read_i2c(uint8_t addr, uint8_t reg, uint8_t n, uint8_t* data);

//================================================================================================//
// write_i2c() --- Write a bytte to i2c device                                                    //
// Argument : uint8_t addr  - The address of the device to communicate with                       //
//            uint8_t reg   - The register value to send to the i2c device                        //
//            uint8_t n     - The number of bytes to receive from the i2c device                  //
//            uint8_t *data - The data array pointer to write received data                       //
// Returtn  : None                                                                                //
//================================================================================================//
void write_i2c(uint8_t addr, uint8_t reg, uint8_t data);

#endif // I2C_BASICS_H
