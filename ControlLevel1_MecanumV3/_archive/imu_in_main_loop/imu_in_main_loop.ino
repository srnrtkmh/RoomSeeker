//================================================================================================//
//                                                                                                //
// FILE : imu_in_main_loop.ino                                                                    //
// MEMO : Read imu (MPU-9250) and transmit read data via serial                                   //
//                                                                                                //
// Update Log                                                                                     //
//   2021/01/03 : Start this project                                                              //
//                                                                                                //
// Pin Assign                                                                                     //
//    0 - NC                                       1 - NC                                         //
//    2 - M0ENA Front Right L298N ENA (OC3B)       3 - M1ENB Front Left L298N ENB (OC3C)          //
//    4 - NC                                       5 - NC                                         //
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
//   52 - ECHO4 Echo input from US Sensor 4       53 - ECHO7 Echo input from US Sensor 7          //
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
#include <Wire.h>

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For MPU-9250 I2C address settings
#define MPU9250_ADDR  0x68        // Slave address of MPU9250
#define GYRO_CONFIG   0x1b        // Register address of gyrometer configuration byte
#define ACCEL_CONFIG  0x1c        // Register address of accelerometer configuration byte
#define INT_PIN_CFG   0x37        // Register address of INT Pin / Bypass Enable Configuration
#define PWR_MGMT_1    0x6b        // Register address of power management
#define ACCEL_XOUT_H  0x3b        // Register address of x-axis acceleration data of upper byte

#define AK8963_ADDR   0x0c        // Slave address of AK8963
#define ST1           0x02        // データ読み込み用フラッグのアドレス
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

// For MPU-9250 data variables
uint8_t mpu9250_data[14]; // Read data array for MPU-9250
uint8_t magneticData[7];  // Read data array for AK8963
uint8_t ST1_data;         // Read data for AK8963 ST1 register
int16_t ax = 0;           // Acc x-axis data
int16_t ay = 0;           // Acc y-axis data
int16_t az = 0;           // Acc z-axis data
int16_t gx = 0;           // Gyro x-axis data
int16_t gy = 0;           // Gyro y-axis data
int16_t gz = 0;           // Gyro z-axis data
int16_t tc = 0;           // Temperature data
int16_t mx = 0;           // Mag x-axis data
int16_t my = 0;           // Mag y-axis data
int16_t mz = 0;           // Mag z-axis data

// Time variables
long start_time = 0;
long stop_time = 0;

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup() {
  Wire.begin();           // Start I2C module
  //Wire.setClock(400000);  // Set I2C clock 400kHz
  Serial.begin(230400);   // Start Serial communication

  // Initialize MPU-9250
  write_i2c(MPU9250_ADDR, PWR_MGMT_1, 0x00);                  // Reset sleep mode
  write_i2c(MPU9250_ADDR, ACCEL_CONFIG, ACCEL_FS_SEL_16G);    // Set the range of accelerometer
  write_i2c(MPU9250_ADDR, GYRO_CONFIG, GYRO_FS_SEL_2000DPS);  // Set the range of gyrometer
  write_i2c(MPU9250_ADDR, INT_PIN_CFG, 0x02);                 // Enable bypass mode (enable magnetometer)
  write_i2c(AK8963_ADDR, CNTL1, CNTL1_MODE_SEL_100HZ);        // Start sampling
}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
  char str[64];
  start_time = micros();
  getMPU9250();
  stop_time = micros();

  sprintf(str, "%6d,%6d,%6d,", ax, ay, az); Serial.print(str);
  sprintf(str, "%6d,%6d,%6d,", gx, gy, gz); Serial.print(str);
  sprintf(str, "%6d,%6d,%6d,", mx, my, mz); Serial.print(str);
  sprintf(str, "%6d,", tc); Serial.print(str);
  sprintf(str, "%10ld\n", stop_time - start_time); Serial.print(str);
}

//================================================================================================//
// Get MPU-9250 sensor data                                                                       //
//================================================================================================//
void getMPU9250() {

  // Read MPU-9250 and AK8963 data through I2C
  read_i2c(MPU9250_ADDR, ACCEL_XOUT_H, 14, mpu9250_data);
  read_i2c(AK8963_ADDR, ST1, 1, &ST1_data);
  if ((ST1_data & 0x01)) read_i2c(AK8963_ADDR, HXL, 7, magneticData);

  // Acceleration data formatting
  ax = (mpu9250_data[0] << 8) | mpu9250_data[1];
  ay = (mpu9250_data[2] << 8) | mpu9250_data[3];
  az = (mpu9250_data[4] << 8) | mpu9250_data[5];
  
  // Temperature data formatting
  tc = (mpu9250_data[6] << 8) | mpu9250_data[7];
  
  // Gyro data formatting
  gx = (mpu9250_data[8] << 8) | mpu9250_data[9];
  gy = (mpu9250_data[10] << 8) | mpu9250_data[11];
  gz = (mpu9250_data[12] << 8) | mpu9250_data[13];
  
  // Magneto data formatting (fit the axis with accelerometer)
  mx = (magneticData[3] << 8) | magneticData[2];
  my = (magneticData[1] << 8) | magneticData[0];
  mz = -((magneticData[5] << 8) | magneticData[4]);
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
