//================================================================================================//
//                                                                                                //
// FILE : mpu9250.h                                                                               //
// MEMO : This library enables you to get MPU-9250 sensor data                                    //
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

#ifndef MPU9250_H

#include <inttypes.h>
#include "i2c_basics.h"

//#define DEBUG

// For MPU-9250 I2C address settings
#define MPU9250_ADDR  0x68        // Slave address of MPU9250
#define GYRO_CONFIG   0x1b        // Register address of gyrometer configuration byte
#define ACCEL_CONFIG  0x1c        // Register address of accelerometer configuration byte
#define INT_PIN_CFG   0x37        // Register address of INT Pin / Bypass Enable Configuration
#define PWR_MGMT_1    0x6b        // Register address of power management
#define ACCEL_XOUT_H  0x3b        // Register address of x-axis acceleration data of upper byte

#define AK8963_ADDR   0x0c        // Slave address of AK8963
#define ST1           0x02        // �f�[�^�ǂݍ��ݗp�t���b�O�̃A�h���X
#define HXL           0x03        // Register address of x-axis magnetometer data of upper byte
#define CNTL1         0x0a        // Register address of control 1
#define CNTL2         0x0b        // Register address of control 2

// For MPU-9250 I2C parameter settings
#define ACCEL_FS_SEL_2G 0x00      // The range of accelerometer is 2G
#define ACCEL_FS_SEL_4G 0x08      // The range of accelerometer is 4G
#define ACCEL_FS_SEL_8G 0x10      // The range of accelerometer is 8G
#define ACCEL_FS_SEL_16G 0x18     // The range of accelerometer is 16G
#define GYRO_FS_SEL_250DPS 0x00   // The range of gyrometer is 250 deg/sec
#define GYRO_FS_SEL_500DPS 0x08   // The range of gyrometer is 500 deg/sec
#define GYRO_FS_SEL_1000DPS 0x10  // The range of gyrometer is 1000 deg/sec
#define GYRO_FS_SEL_2000DPS 0x18  // The range of gyrometer is 2000 deg/sec
#define CNTL1_MODE_SEL_8HZ 0x12   // Magnetometer resolution 16bit, Continuous sampling mode (8Hz)
#define CNTL1_MODE_SEL_100HZ 0x16 // Magnetometer resolution 16bit, Continuous sampling mode (100Hz)

class Mpu9250 {
  public:
    void init(void);              // Initialize MPU-9250 and AK8963
    void reload_data();           // Reload sensor data
    int16_t read_acc(uint8_t n);  // Return accelerometer value
    int16_t read_gyro(uint8_t n); // Return gyrometer value
    int16_t read_mag(uint8_t n);  // Return magnetometer value
    int16_t read_temp(void);      // Return temperature value

    uint8_t mpu9250_data[14];     // Read data array for MPU-9250
    uint8_t magneticData[7];      // Read data array for AK8963
    uint8_t ST1_data;             // Read data for AK8963 ST1 register
    int16_t ax = 0;               // Acc x-axis data
    int16_t ay = 0;               // Acc y-axis data
    int16_t az = 0;               // Acc z-axis data
    int16_t gx = 0;               // Gyro x-axis data
    int16_t gy = 0;               // Gyro y-axis data
    int16_t gz = 0;               // Gyro z-axis data
    int16_t tc = 0;               // Temperature data
    int16_t mx = 0;               // Mag x-axis data
    int16_t my = 0;               // Mag y-axis data
    int16_t mz = 0;               // Mag z-axis data
};

#endif // MPU9250_H
