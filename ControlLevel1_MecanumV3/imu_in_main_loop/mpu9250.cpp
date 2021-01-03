//================================================================================================//
//                                                                                                //
// FILE : mpu9250.cpp                                                                             //
// MEMO : This library enables you to get MPU-9250 sensor data                                    //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
// Pin usage                                                                                      //
//   20 - SDA                                     21 - SCL                                        //
//                                                                                                //
//                                         Copyright (c) 2020 Kyohei Umemoto All Rights reserved. //
//                                                                                                //
//================================================================================================//

#if defined(ARDUINO_ARCH_AVR)

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include "mpu9250.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//

//================================================================================================//
// Initialize MPU-9250 and AK8963
//================================================================================================//
void Mpu9250::init(void) {
  // Initialize I2C
  init_i2c(400000L);
  
  // Initialize MPU-9250
  write_i2c(MPU9250_ADDR, PWR_MGMT_1, 0x00);                  // Reset sleep mode
  write_i2c(MPU9250_ADDR, ACCEL_CONFIG, ACCEL_FS_SEL_16G);    // Set the range of accelerometer
  write_i2c(MPU9250_ADDR, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);  // Set the range of gyrometer
  write_i2c(MPU9250_ADDR, INT_PIN_CFG, 0x02);                 // Enable bypass mode (enable magnetometer)
  write_i2c(AK8963_ADDR, CNTL1, CNTL1_MODE_SEL_100HZ);        // Start sampling
}

//================================================================================================//
// Get MPU-9250 sensor data                                                                       //
//================================================================================================//
void Mpu9250::reload_data() {
  // Read MPU-9250 and AK8963 data through I2C
  read_i2c(MPU9250_ADDR, ACCEL_XOUT_H, 14, mpu9250_data);
  read_i2c(AK8963_ADDR, ST1, 1, &ST1_data);
  if ((ST1_data & 0x01)) read_i2c(AK8963_ADDR, HXL, 7, magneticData);

  // Acceleration data formatting
  this->ax = (mpu9250_data[0] << 8) | mpu9250_data[1];
  this->ay = (mpu9250_data[2] << 8) | mpu9250_data[3];
  this->az = (mpu9250_data[4] << 8) | mpu9250_data[5];

  // Temperature data formatting
  this->tc = (mpu9250_data[6] << 8) | mpu9250_data[7];

  // Gyro data formatting
  this->gx = (mpu9250_data[8] << 8) | mpu9250_data[9];
  this->gy = (mpu9250_data[10] << 8) | mpu9250_data[11];
  this->gz = (mpu9250_data[12] << 8) | mpu9250_data[13];

  // Magneto data formatting (fit the axis with accelerometer)
  this->mx = (magneticData[3] << 8) | magneticData[2];
  this->my = (magneticData[1] << 8) | magneticData[0];
  this->mz = -((magneticData[5] << 8) | magneticData[4]);
}

//================================================================================================//
// Return accelerometer value
//================================================================================================//
int16_t Mpu9250::read_acc(uint8_t n) {
  if(n == 0) return(this->ax);
  else if(n == 1) return(this->ay);
  else if(n == 2) return(this->az);
  return(-1);
}

//================================================================================================//
// Return gyrometer value
//================================================================================================//
int16_t Mpu9250::read_gyro(uint8_t n) {
  if(n == 0) return(this->gx);
  else if(n == 1) return(this->gy);
  else if(n == 2) return(this->gz);
  return(-1);
}

//================================================================================================//
// Return magnetometer value
//================================================================================================//
int16_t Mpu9250::read_mag(uint8_t n) {
  if(n == 0) return(this->mx);
  else if(n == 1) return(this->my);
  else if(n == 2) return(this->mz);
  return(-1);
}

//================================================================================================//
// Return temperature value
//================================================================================================//
int16_t Mpu9250::read_temp(void) {
  return(this->tc);
}

#endif // ARDUINO_ARCH_AVR
