#!/usr/bin/env python
# coding: utf-8
#==================================================================================================#
#                                                                                                  #
# FILE : turtlebot3_core_v2.py                                                                     #
# Memo : Turtlebot3_core program used by turtlebot3 instead of OpenCR                              #
#                                                                                                  #
# Updated : 2020/09/20 Started this project based on "MotorControlTest_v2.py"                      #
#           2020/10/24                                                                             #
#           2020/10/31 Added odometry publishing process                                           #
#           2021/01/02 Renamed "turtlebot3_core_v2.py". Added IR sensor data to received data      #
#           2021/01/03 Added IMU sensor data to received data                                      #
#           2021/01/09 arduino_motor_controlクラスを別ファイルに分離                               #
#                                                                                                  #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     rosrun turtlebot3_bringup turtlebot3_core_v1.py [Serial device name]                         #
#         or                                                                                       #
#     nohup python turtlebot3_core_v1.py [Serial device name] > [tty????.txt] &                    #
#                                                                                                  #
# Argument Discription :                                                                           #
#     Serial device name : Example "/dev/ttyACM0", "/dev/ttyUSB1"                                  #
#                                                                                                  #
# I/O assign :                                                                                     #
# 01 : 3.3V                                       02 : 5.0V                                        #
# 03 : NC                                         04 : 5.0V                                        #
# 05 : NC                                         06 : GND                                         #
# 07 : NC                                         08 : NC                                          #
# 09 : NC                                         10 : NC                                          #
# 11 : NC                                         12 : NC                                          #
# 13 : NC                                         14 : NC                                          #
# 15 : NC                                         16 : NC                                          #
# 17 : NC                                         18 : NC                                          #
# 19 : NC                                         20 : NC                                          #
# 21 : NC                                         22 : NC                                          #
# 23 : NC                                         24 : NC                                          #
# 25 : GND                                        26 : NC                                          #
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

import room_seeker_l1

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
serDevName = "COM5"         # シリアルデバイスの設定
baudrate = 115200                   # ボーレート設定
data_file = []                      # CSV出力用ファイルオブジェクト
strCsvName = ""                     # ファイル名用変数
preffix = "MotorControl_with_Lidar" # ファイル名プレフィックス
suffix = ""                         # ファイル名サフィックス
before_nokori = ""
sample_err = 0
sample_err_num = 0

# The others
WHEEL_NUM = 4
FRONT_RIGHT = 0
FRONT_LEFT  = 1
REAR_RIGHT  = 2
REAR_LEFT   = 3
BAT_CAPA    = 6.5
MAX_VOLT    = 8.4
HARDWARE_VER = "0.0.0"
SOFTWARE_VER = "0.0.0"
FIRMWARE_VER = "0.0.0"
pub10cnt = 0

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
  arduino.con.write(str.encode("t"))
  time.sleep(0.01)
  arduino.con.write(str.encode("t"))
  time.sleep(0.01)
  arduino.con.write(str.encode("t"))
  sys.exit(0)

#==================================================================================================#
# Keyboard input function                                                                          #
#==================================================================================================#
def key_input():
  print("key_input started.")
  while True:
    test = input()
    
    if test == 'w':
      arduino.dx_cmd_x10[0] += 1000
    elif test == 'x':
      arduino.dx_cmd_x10[0] -= 1000
    elif test == 'a':
      arduino.dx_cmd_x10[1] += 1000
    elif test == 'd':
      arduino.dx_cmd_x10[1] -= 1000
    elif test == 's':
      arduino.dx_cmd_x10[0] = 0
      arduino.dx_cmd_x10[1] = 0
      arduino.dx_cmd_x10[2] = 0
    else:
      arduino.con.write(test)
      print(test)

#==================================================================================================#
# Main Function                                                                                    #
#==================================================================================================#
if __name__ == '__main__':
  # 引数処理
  param = sys.argv                                  # 現時点何もなし
  
  # 初期設定
  currentDate = datetime.datetime.today()           # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  arduino_pre = room_seeker_l1.RoomSeekerLevel1()   # Robot control variables (previous)
  arduino = room_seeker_l1.RoomSeekerLevel1()       # Robot control variables (current)
  signal.signal(signal.SIGINT, receive_signal)      # SIGINTを受け取ったら指定関数を呼び出すように設定
  
  # キーボード入力用スレッド作成
  th_key = threading.Thread(target=key_input)
  th_key.setDaemon(True)
  th_key.start()
  
  # Setup serial communication with Arduino -------------------------------------------------------#
  try:
    # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    arduino.con = serial.Serial(serDevName, baudrate, timeout=0)
    # arduino.con = serial.Serial(serDevName, baudrate, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(1.0)
  except serial.SerialException as e:
    print("SerialException : " + str(e))
    sys.exit(1)
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # 読み取りバッファを空にしておく
  
  # Send start command to arduino
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # 読み取りバッファを空にしておく(前に記述した1回だけだと空にならない時がある(?))
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  arduino.con.write(str.encode("s"))                  # スタートコマンドを送信(1回目)
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  tmp_str = arduino.con.read()            # スタートコマンドのコールバックを確認
  if len(tmp_str) >= 10:
    arduino.con.write(str.encode("s"))                # スタートコマンドを送信(2回目)
    time.sleep(1.0)                       # スタートコマンドのコールバックまで1秒間待ち
    tmp_str = arduino.con.read()          # スタートコマンドのコールバックを確認
    if len(tmp_str) >= 10:
      arduino.con.write(str.encode("s"))              # スタートコマンドを送信(3回目)
      time.sleep(1.0)                     # スタートコマンドのコールバックまで1秒間待ち
      tmp_str = arduino.con.read()        # スタートコマンドのコールバックを確認
      if len(tmp_str) >= 10:
        print("Serial Device did not transmit call back : " + tmp_str)
        arduino.con.write("t")            # 念のためストップコマンドを送っておく
        sys.exit(1)
  
  # 制御変数初期化
  ctrl_cnt = 0
  ctrl_state = 0
  ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
  
  
  xx = 0
  yy = 0
  dir = 0
  # 周期処理開始
  try:
    while True:
      # 制御処理
      # arduino.cmd_wrk(arduino.dx_cmd_x10[0], arduino.dx_cmd_x10[1], arduino.dx_cmd_x10[2])
      
      if(dir == 0):
        xx += 50
        yy += 50
      else:
        xx -= 50
        yy -= 50
      if(xx >= 5000): dir = 1
      if(xx <= 2500): dir = 0
      
      arduino.con.write(str.encode('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(xx)) + ',' + '{:0=+5}'.format(int(yy)) + ',end'))
      print('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(xx)) + ',' + '{:0=+5}'.format(int(yy)) + ',end')
      
      # 現在の日時を取得
      currentDate = datetime.datetime.today()
      
      # 読み取りデータをファイルに書き込み
      tmp_str = arduino.con.read(arduino.con.inWaiting()) # シリアルのバッファー内のデータを取得
      tmp_list = (before_nokori + str(tmp_str, 'utf-8')).split('\n')    # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          tmp_val_list = tmp_list[i].split(',')
          arduino_pre = arduino
          sample_err = 0
          if len(tmp_val_list) == 10:
            if is_int(tmp_val_list[0]):  arduino.sample_num_node = int(tmp_val_list[0])
            else:                        sample_err = 1
            if is_int(tmp_val_list[1]):  arduino.cnt_now[0] = int(tmp_val_list[1])
            else:                        sample_err = 1
            if is_int(tmp_val_list[2]):  arduino.cnt_now[1] = int(tmp_val_list[2])
            else:                        sample_err = 1
            if is_int(tmp_val_list[3]):  arduino.omega_res_x10[0] = float(int(tmp_val_list[3]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[4]):  arduino.omega_res_x10[1] = float(int(tmp_val_list[4]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[5]):  arduino.omega_cmd_x10[0] = float(int(tmp_val_list[5]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[6]):  arduino.omega_cmd_x10[1] = float(int(tmp_val_list[6]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[7]):  arduino.vout[0] = float(int(tmp_val_list[7]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[8]):  arduino.vout[1] = float(int(tmp_val_list[8]))
            else:                        sample_err = 1
            if is_int(tmp_val_list[9]):  arduino.interval = float(int(tmp_val_list[9]))
            else:                        sample_err = 1
          else:
            sample_err = 1
            sample_err_num += 1
            arduino = arduino_pre
          
          arduino.apply_filter()
          arduino.odometry_update_cmd()
          arduino.odometry_update_res()
          
          if sample_err != 0:
            print("Error : " + str(sample_err_num))
            print(tmp_list[i])
          
          # data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000)
            # + ',' + str(arduino.sample_num_node) + ',' + str(arduino.cnt_now[0]) + ',' + str(arduino.cnt_now[1]) + ',' + str(arduino.cnt_now[2]) + ',' + str(arduino.cnt_now[3])
            # + ',' + str(arduino.omega_res_x10[0]) + ',' + str(arduino.omega_res_x10[1]) + ',' + str(arduino.omega_res_x10[2]) + ',' + str(arduino.omega_res_x10[3])
            # + ',' + str(arduino.omega_cmd_x10[0]) + ',' + str(arduino.omega_cmd_x10[1]) + ',' + str(arduino.omega_cmd_x10[2]) + ',' + str(arduino.omega_cmd_x10[3])
            # + ',' + str(arduino.vout[0]) + ',' + str(arduino.vout[1]) + ',' + str(arduino.vout[2]) + ',' + str(arduino.vout[3])
            # + ',' + str(arduino.dx_res_x10[0]) + ',' + str(arduino.dx_res_x10[1]) + ',' + str(arduino.dx_res_x10[2])
            # + ',' + str(arduino.dx_cmd_x10[0]) + ',' + str(arduino.dx_cmd_x10[1]) + ',' + str(arduino.dx_cmd_x10[2])
            # + ',' + str(arduino.x_res[0]) + ',' + str(arduino.x_res[1]) + ',' + str(arduino.x_res[2])
            # + ',' + str(arduino.x_res2[0]) + ',' + str(arduino.x_res2[1]) + ',' + str(arduino.x_res2[2])
            # + ',' + str(arduino.ir_hex)
            # + '\n')
      
      before_nokori = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存
      
      # Publish information
      pub10cnt += 1
      if pub10cnt >= 10:
        pub10cnt = 0
      
      # arduino.print_odom()
      # arduino.print_state()
      # print("omega_cmd[0] = " + '{:0=5}'.format(int(arduino.omega_cmd_x10[0])) + ", omega_res[0] = " + '{:0=5}'.format(int(arduino.omega_res_x10[0])))
      time.sleep(0.01)
  
  except KeyboardInterrupt:
    receive_signal();

