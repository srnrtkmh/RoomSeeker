#!/usr/bin/env python
# coding: utf-8
#==================================================================================================#
#                                                                                                  #
# FILE : turtlebot3_core_v2.py                                                                     #
# Memo : Turtlebot3_core program used by turtlebot3 instead of OpenCR                              #
#                                                                                                  #
# Updated : 2020/09/20 Started this project based on "MotorControlTest_v2.py"                      #
#                                                                       (C) 2020 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     rosrun turtlebot3_bringup turtlebot3_core_v1.py [Serial device name]                         #
#         or                                                                                       #
#     nohup python turtlebot3_core_v1.py [Serial device name] > [tty????.txt] &                    #
#                                                                                                  #
# Argument Discription :                                                                           #
#     Serial device name : Example "/dev/ttyACM0", "/dev/ttyUSB1"                                  #
#                                                                                                  #
#==================================================================================================#

#==================================================================================================#
# Import Module                                                                                    #
#==================================================================================================#
import sys            # sys(exit()の使用, 引数取得のために必要)
import os             # OS(ファイルの各種処理のために必要)
import locale         # local(!)
import datetime       # datetimeモジュール
import time           # 時刻取得用モジュール
import threading      # マルチスレッドプログラミング用モジュール
from math import *    # 算術演算用モジュール
import serial         # Arduinoとの通信で使用するシリアル通信用モジュール
import signal         # シグナル受信に使用するモジュール
import shutil         # ディレクトリが空でなくても削除するために使うモジュール
import numpy as np    # For matrix calculation

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
serDevName = "COM5"         # シリアルデバイスの設定
baudrate = 230400                   # ボーレート設定
data_file = []                      # CSV出力用ファイルオブジェクト
strCsvName = ""                     # ファイル名用変数
preffix = "magnetometer_gui"        # ファイル名プレフィックス
suffix = ""                         # ファイル名サフィックス
before_nokori = ""
sample_err = 0
sample_err_num = 0

# For MPU-9250
[ax, ay, az] = [0.0, 0.0, 0.0]
[gx, gy, gz] = [0.0, 0.0, 0.0]
[mx, my, mz] = [0.0, 0.0, 0.0]
temp = 0.0
interval = 0.0

[mx_lpf, my_lpf, mz_lpf] = [0.0, 0.0, 0.0]
g_mag = 6.0
dt = 0.02

[mx_off, my_off, mz_off] = [33.89, 10.70, 0.0]

#==================================================================================================#
# Descriminate the literal is integer or not                                                       #
#==================================================================================================#
def is_int(s):
  try:
    int(s)
    return True
  except ValueError:
    return False

#==================================================================================================#
# Signal Receive Function                                                                          #
#==================================================================================================#
def receive_signal(signum, stack):
  print("Received signal :", signum)
  currentDate = datetime.datetime.today()  # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  data_file.close()
  sys.exit(0)

#==================================================================================================#
# Main Function                                                                                    #
#==================================================================================================#
if __name__ == '__main__':
  # 引数処理
  param = sys.argv
  if len(param) >= 2:
    serDevName = param[1]
  if len(param) >= 3:
    baudrate = int(param[2])
  
  # 初期設定
  currentDate = datetime.datetime.today()           # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  signal.signal(signal.SIGINT, receive_signal)      # SIGINTを受け取ったら指定関数を呼び出すように設定
  
  # Open data file
  strCsvName = preffix + str(currentDate.strftime('%Y_%m_%d_%H_%M_%S')) + suffix + ".csv"
  data_file = open(strCsvName, 'w')
  data_file.write("time,ax,ay,az,gx,gy,gz,mx,my,mz,temp,interval,mx_lpf,my_lpf,atan2\n")
  
  # Setup serial communication with Arduino -------------------------------------------------------#
  try:
    # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    con = serial.Serial(serDevName, baudrate, timeout=0)
    # con = serial.Serial(serDevName, baudrate, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(0.5)
  except serial.SerialException as e:
    print("SerialException : " + str(e))
    sys.exit(1)
  tmp_str = con.read(con.inWaiting())     # 読み取りバッファを空にしておく
  
  # 周期処理開始
  try:
    while True:
      # 現在の日時を取得
      currentDate = datetime.datetime.today()
      
      # 読み取りデータをファイルに書き込み
      tmp_str = con.read(con.inWaiting()) # シリアルのバッファー内のデータを取得
      tmp_list = (before_nokori + str(tmp_str, 'utf-8')).split('\n')    # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          tmp_val_list = tmp_list[i].split(',')
          sample_err = 0
          if len(tmp_val_list) == 11:
            if is_int(tmp_val_list[0]):  ax = int(tmp_val_list[0])
            else:                        sample_err = 1
            if is_int(tmp_val_list[1]):  ay = int(tmp_val_list[1])
            else:                        sample_err = 2
            if is_int(tmp_val_list[2]):  az = int(tmp_val_list[2])
            else:                        sample_err = 3
            if is_int(tmp_val_list[3]):  gx = float(int(tmp_val_list[3]))
            else:                        sample_err = 4
            if is_int(tmp_val_list[4]):  gy = float(int(tmp_val_list[4]))
            else:                        sample_err = 5
            if is_int(tmp_val_list[5]):  gz = float(int(tmp_val_list[5]))
            else:                        sample_err = 6
            if is_int(tmp_val_list[6]):  mx = float(int(tmp_val_list[6]))
            else:                        sample_err = 7
            if is_int(tmp_val_list[7]):  my = float(int(tmp_val_list[7]))
            else:                        sample_err = 8
            if is_int(tmp_val_list[8]):  mz = float(int(tmp_val_list[8]))
            else:                        sample_err = 9
            if is_int(tmp_val_list[9]):  temp = float(int(tmp_val_list[9]))
            else:                        sample_err = 10
            if is_int(tmp_val_list[10]): interval = float(int(tmp_val_list[10]))
            else:                        sample_err = 111
          else:
            sample_err = 99
          
          if sample_err != 0:
            print("Error : " + str(sample_err_num))
            print(tmp_list[i])
          
      before_nokori = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存
      
      # Apply LPF
      mx_lpf += g_mag * dt * (mx - mx_lpf)
      my_lpf += g_mag * dt * (my - my_lpf)
      
      print('mx = {:=5}, my = {:=5}, atan2 = {:=5}'.format(mx, my, atan2(mx_lpf, my_lpf)/3.141592*180.0))
      data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000)
        + ',{:=6},{:=6},{:=6},{:=6},{:=6},{:=6},{:=6},{:=6},{:=6},{:=6},{:=6}'.format(ax, ay, az, gx, gy, gz, mx, my, mz, temp, interval)
        + ',{:=6.2f},{:=6.2f},{:=6.3f}\n'.format(mx_lpf, my_lpf, atan2(mx_lpf, my_lpf)/3.141592*180.0))
      
      time.sleep(0.02)
  
  except KeyboardInterrupt:
    receive_signal();

