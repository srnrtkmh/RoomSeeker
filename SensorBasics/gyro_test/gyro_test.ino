//================================================================================================//
//                                                                                                //
// FILE : gyro_test.ino                                                                           //
// MEMO : Test program for gyro sensor L3GD20 and ENC-03R                                         //
//        Read each sensor value every 10ms                                                       //
//                                                                                                //
// 20/07/18 : Start to edit this program                                                          //
// 20/07/19 : Add reading program of accelerometer LIS3DH                                         //
//                                                                                                //
//                                         Copyright (C) 2020 Kyohei Umemoto All Rights Reserved. //
// Pin assign :                                                                                   //
//   A0 : ENC-03R #1                            A4(SDA) : L3GD20 SDA                              //
//   A1 : ENC-03R #2                            A5(SCL) : L3GD20 SCL                              //
//   D2 : DHT22                                                                                   //
//                                                                                                //
// Attention :                                                                                    //
//   *L3GD20 SA0 is connected to GND (LSB is zero)                                                //
//                                                                                                //
//================================================================================================//
#include <MsTimer2.h>
#include <TimerOne.h>
#include <Wire.h>
//#include "SparkFunLIS3DH.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// For L3GD20
const byte L3GD20_ADDR = B1101010;  // SA0 = GND
//const byte L3GD20_ADDR = B1101011;// SA0 = VDD_IO
const byte L3GD20_WHOAMI = 0x0f;
const byte L3GD20_CTRL1 = 0x20;
const byte L3GD20_CTRL2 = 0x21;
const byte L3GD20_CTRL3 = 0x22;
const byte L3GD20_CTRL4 = 0x23;
const byte L3GD20_CTRL5 = 0x24;
const byte L3GD20_X_L = 0x28;
const byte L3GD20_X_H = 0x29;
const byte L3GD20_Y_L = 0x2A;
const byte L3GD20_Y_H = 0x2B;
const byte L3GD20_Z_L = 0x2C;
const byte L3GD20_Z_H = 0x2D;

// For LIS3DH
const byte LIS3DH_ADDR = B0011000;  // SA0 = GND
//const byte LIS3DH_ADDR = B0011001;  // SA0 = VDD_IO
const byte LIS3DH_WHOAMI = 0x0f;
const byte LIS3DH_CTRL_REG1 = 0x20;
const byte LIS3DH_CTRL_REG2 = 0x21;
const byte LIS3DH_CTRL_REG3 = 0x22;
const byte LIS3DH_CTRL_REG4 = 0x23;
const byte LIS3DH_CTRL_REG5 = 0x24;
const byte LIS3DH_CTRL_REG6 = 0x25;
const byte LIS3DH_OUT_X_L = 0x28;
const byte LIS3DH_OUT_X_H = 0x29;
const byte LIS3DH_OUT_Y_L = 0x2a;
const byte LIS3DH_OUT_Y_H = 0x2b;
const byte LIS3DH_OUT_Z_L = 0x2c;
const byte LIS3DH_OUT_Z_H = 0x2d;
//LIS3DH myIMU; //Default constructor is I2C, addr 0x19.

// ループカウント用変数
int n = 0;
int start_bit = 0;
long sampling_time = 9999;

//================================================================================================//
// L3DG20_write :                                                                                 //
//================================================================================================//
void L3GD20_write(byte reg, byte val){
    Wire.beginTransmission(L3GD20_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

//================================================================================================//
// L3DG20_read :                                                                                  //
//================================================================================================//
byte L3GD20_read(byte reg){
  byte ret = 0;
  // request the registor
  Wire.beginTransmission(L3GD20_ADDR);
  Wire.write(reg);
  Wire.endTransmission();  

  // read
  Wire.requestFrom((unsigned int)L3GD20_ADDR, 1);
  
  while (Wire.available()) {
    ret = Wire.read();
  }
  
  return ret;
}

//================================================================================================//
// LIS3DH_write :                                                                                 //
//================================================================================================//
void LIS3DH_write(byte reg, byte val){
    Wire.beginTransmission(LIS3DH_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

//================================================================================================//
// LIS3DH_read :                                                                                  //
//================================================================================================//
byte LIS3DH_read(byte reg){
  byte ret = 0;
  // request the registor
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.endTransmission();  

  // read
  Wire.requestFrom((unsigned int)LIS3DH_ADDR, 1);
  
  while (Wire.available()) {
    ret = Wire.read();
  }
  
  return ret;
}

//================================================================================================//
// Setups                                                                                         //
//================================================================================================//
void setup(){
    // Initialize UART
    Serial.begin(115200);                   // Baudrate : 115.2kbps
    while (!Serial){}                       // Wait until Serial port is ready

    // Initialize I2C (L3GD20 and LIS3DH)
    Wire.begin();
    Serial.println(L3GD20_read(L3GD20_WHOAMI), HEX); // should show 0xD4
    L3GD20_write(L3GD20_CTRL1, B00001111);
                           //   |||||||+ X axis enable
                           //   ||||||+- Y axis enable
                           //   |||||+-- Z axis enable
                           //   ||||+--- PD: 0: power down, 1: active
                           //   ||++---- BW1-BW0: cut off 12.5[Hz]
                           //   ++------ DR1-DR0: ODR 95[HZ]
    
    Serial.println(LIS3DH_read(LIS3DH_WHOAMI), HEX); // should show 0x33
    LIS3DH_write(LIS3DH_CTRL_REG1, B01010111);
                           //   |||||||+ Xen :  1: axis enable
                           //   ||||||+- Yen :  1: axis enable
                           //   |||||+-- Zen :  1: axis enable
                           //   ||||+--- LPen:  0: low power, 1: normal
                           //   ++++---- ODR3-0: 0101: Normal / low power mode (100 Hz)
    
    // Timer1割込の設定
    Timer1.initialize(sampling_time);
    Timer1.attachInterrupt(flash);
    Timer1.stop();
    
    delay(100) ;                              // 0.1Sしたら開始
}

//================================================================================================//
// Timer Interrupt Funtion                                                                        //
//================================================================================================//
void flash() {
  short X, Y, Z;
  float x, y, z;
  short x_enc, y_enc;

  sei();    // Allow interrupt (If this is not executed, I2C library cannot use)

  // Output sample number to serial
  Serial.print(n);    // Sample Number
  n++;
  if(n >= 10000) n = 0;
  Serial.print("\t");

  // Read L3GD20 value and transmit the data to serial
  X = L3GD20_read(L3GD20_X_H);
  x = X = (X << 8) | L3GD20_read(L3GD20_X_L);
  Y = L3GD20_read(L3GD20_Y_H);
  y = Y = (Y << 8) | L3GD20_read(L3GD20_Y_L);
  Z = L3GD20_read(L3GD20_Z_H);
  z = Z = (Z << 8) | L3GD20_read(L3GD20_Z_L);
  
  x *= 0.00875; // +-250dps
  //x *= 0.0175;// +-500dps
  //x *= 0.07;  // +-2000dps
  y *= 0.00875; // +-250dps
  z *= 0.00875; // +-250dps
  
  Serial.print(X);    // X axis (reading)
  Serial.print("\t");
  Serial.print(Y);    // Y axis (reading)
  Serial.print("\t");
  Serial.print(Z);    // Z axis (reading)
  Serial.print("\t");
  Serial.print(x);    // X axis (deg/sec)
  Serial.print("\t");
  Serial.print(y);    // Y axis (deg/sec)
  Serial.print("\t");
  Serial.print(z);    // Z axis (deg/sec)
  Serial.print("\t");

  // Read ENC-03R value and transmit the data to serial
  x_enc = analogRead(A0);
  y_enc = analogRead(A1);
  Serial.print(x_enc);
  Serial.print("\t");
  Serial.print(y_enc);
  Serial.print("\t");

  // Get all parameters of LIS3DH
  X = LIS3DH_read(LIS3DH_OUT_X_H);
  x = X = (X << 8) | LIS3DH_read(LIS3DH_OUT_X_L);
  Y = LIS3DH_read(LIS3DH_OUT_Y_H);
  y = Y = (Y << 8) | LIS3DH_read(LIS3DH_OUT_Y_L);
  Z = LIS3DH_read(LIS3DH_OUT_Z_H);
  z = Z = (Z << 8) | LIS3DH_read(LIS3DH_OUT_Z_L);
  Serial.print(X);    // X axis (reading)
  Serial.print("\t");
  Serial.print(Y);    // X axis (reading)
  Serial.print("\t");
  Serial.print(Z);    // X axis (reading)
  Serial.print("\t");
  
/*  Serial.print(myIMU.readFloatAccelX(), 4);
  Serial.print("\t");
  Serial.print(myIMU.readFloatAccelY(), 4);
  Serial.print("\t");
  Serial.print(myIMU.readFloatAccelZ(), 4);
*/  Serial.print("\n");

}

//================================================================================================//
// Main Loop                                                                                      //
//================================================================================================//
void loop() {
    char a;
    
    if(Serial.available()){
        a = char(Serial.read());
        if(a == 's'){
            n = 0;
            start_bit = 1;
            Timer1.start();
        }else if(a == 't'){
            start_bit = 0;
            Timer1.stop();
        }
    }
}
