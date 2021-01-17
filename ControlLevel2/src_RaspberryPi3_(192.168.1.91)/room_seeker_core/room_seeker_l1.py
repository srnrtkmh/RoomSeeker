#!/usr/bin/env python
# coding: utf-8
#==================================================================================================#
#                                                                                                  #
# FILE : room_seeker_l1.py                                                                         #
# Memo : RoomSeeker level 1 node class                                                             #
#                                                                                                  #
# Updated : 2021/01/09 Started this project based on "turtlebot3_core_v2.py"                       #
#                      プロジェクトから分離して独立したモジュールとする                            #
#                      "arduino_motor_control"クラスを"RoomSeekerLevel1"に名前変更                 #
#           2021/01/16 モーターコントロールノードを追加し始める                                    #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
#                                                                                                  #
#==================================================================================================#

#==================================================================================================#
# Import Module                                                                                    #
#==================================================================================================#
import numpy as np    # For matrix calculation
from math import *    # 算術演算用モジュール

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
# RoomSeeker level 1 class                                                                         #
#==================================================================================================#
class RoomSeekerLevel1():
  def __init__(self):
    self.sample_num_node_MEGA = 0                       # Sample number of the Arduino MEGA
    self.sample_num_node_FM = 0                         # Sample number of the Front Motor Controller
    self.sample_num_node_RM = 0                         # Sample number of the Rear Motor Controller
    self.sample_num_host = 0                            # Sample number of the host
    self.sample_num_ul = 999                            # Upper limit of sample number
    self.sample_num_ll = 0                              # Lower limit of sample number
    self.dt = 0.02                                      # Sampling time of the motor controller
    self.cnt_now = np.array([0.0, 0.0, 0.0, 0.0])       # Wheel encoder counted value
    self.omega_res_x10 = np.array([0.0, 0.0, 0.0, 0.0]) # Wheel space velocity response [10^-1deg/s]
    self.omega_cmd_x10 = np.array([0.0, 0.0, 0.0, 0.0]) # Wheel space velocity command [10^-1deg/s]
    self.vout = np.array([0, 0, 0, 0])                  # Voltage output (PWM width : 0 - 4095)
    self.x_res = np.array([0.0, 0.0, 0.0])              # Workspace pose response calculated from velocity response [0:m, 1:m, 2:rad]
    self.x_res_ = np.array([0.0, 0.0, 0.0])
    self.x_res2 = np.array([0.0, 0.0, 0.0])             # Workspace pose response calculated from velocity command [0:m, 1:m, 2:rad]
    self.dx_res = np.array([0.0, 0.0, 0.0])             # Workspace velocity response [0 : m/s, 1 : m/s, 2 : deg/s]
    self.dx_res_x10 = np.array([0.0, 0.0, 0.0])         # Workspace velocity response [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    self.x_cmd = np.array([0.0, 0.0, 0.0])              # Workspace pose command [0:m, 1:m, 2:rad]
    self.dx_cmd_x10 = np.array([0.0, 0.0, 0.0])         # Workspace velocity command [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    self.wheel_radius = 40.0                            # Robot wheel radius [mm]
    self.base_width =  215.0                            # Robot wheel base width [mm]
    self.base_length = 162.0                            # Robot wheel base length (between front and rear wheel shaft) [mm]
    self.J_inv = np.array([[1.0/(self.wheel_radius/1000.0),  1.0/(self.wheel_radius/1000.0),  ((self.base_width/1000.0) + (self.base_length/1000.0))/2.0/(self.wheel_radius/1000.0)]  # Inverse jacobian
                          ,[1.0/(self.wheel_radius/1000.0), -1.0/(self.wheel_radius/1000.0), -((self.base_width/1000.0) + (self.base_length/1000.0))/2.0/(self.wheel_radius/1000.0)]
                          ,[1.0/(self.wheel_radius/1000.0), -1.0/(self.wheel_radius/1000.0),  ((self.base_width/1000.0) + (self.base_length/1000.0))/2.0/(self.wheel_radius/1000.0)]
                          ,[1.0/(self.wheel_radius/1000.0),  1.0/(self.wheel_radius/1000.0), -((self.base_width/1000.0) + (self.base_length/1000.0))/2.0/(self.wheel_radius/1000.0)]])
    self.J_inv_plus = np.dot(np.linalg.inv(np.dot(self.J_inv.transpose(), self.J_inv)), self.J_inv.transpose())
    self.x_ul = [ 5000.0,  5000.0,  5000.0]             # Upper limit of the Workspace
    self.x_ll = [-5000.0, -5000.0, -5000.0]             # Lower limit of the Workspace
    self.ir_hex = 0                                     # Ir sensor date in hex
    self.ax = 0                                         # Acc x-axis
    self.ay = 0                                         # Acc y-axis
    self.az = 0                                         # Acc z-axis
    self.gx = 0                                         # Gyro x-axis
    self.gy = 0                                         # Gyro y-axis
    self.gz = 0                                         # Gyro z-axis
    self.mx = 0                                         # Mag x-axis
    self.my = 0                                         # Mag y-axis
    self.mz = 0                                         # Mag z-axis
    self.mx_lpf = 0                                     # Mag x-axis applied LPF
    self.my_lpf = 0                                     # Mag y-axis applied LPF
    self.mz_lpf = 0                                     # Mag z-axis applied LPF
    self.mx_offset =  27.715769                         # Offset of Mag x-axis
    self.my_offset = -81.559509                         # Offset of Mag y-axis
    self.mr_offset = 177.26162                          # Radius of Mag x-y circle
    self.g_mag = 10.0                                   # Cutoff angular frequency of LPF for magnetosensor [rad/s]
    self.temp = 0                                       # Temperature
    self.conFM = 0                                      # Serial console connected to arduino motor controller
    self.conRM = 0                                      # Serial console connected to arduino motor controller
    self.conMEGA = 0                                    # Serial console connected to arduino motor controller
    self.bat_vol_x100 = 0                               # Battery voltage
    self.ps2_ctrl = 0                                   # PS2 controller is enabled or not
    self.interval = 0                                   # Interval between samples
    self.pi = 3.141592
    self.kkk = 0
    self.before_nokoriMEGA = ''
    self.before_nokoriFM = ''
    self.before_nokoriRM = ''
  
  def print_state(self):
    print('sample_num_node = ' + str(self.sample_num_node_FM))
    print('cnt_now         = ' + str(self.cnt_now[0]) + ', ' + str(self.cnt_now[1]) + ', ' + str(self.cnt_now[2]) + ', ' + str(self.cnt_now[3]))
    print('omega_res_x10   = ' + str(self.omega_res_x10[0]) + ', ' + str(self.omega_res_x10[1]) + ', ' + str(self.omega_res_x10[2]) + ', ' + str(self.omega_res_x10[3]))
    print('omega_cmd_x10   = ' + str(self.omega_cmd_x10[0]) + ', ' + str(self.omega_cmd_x10[1]) + ', ' + str(self.omega_cmd_x10[2]) + ', ' + str(self.omega_cmd_x10[3]))
    print('vout            = ' + str(self.vout[0]) + ', ' + str(self.vout[1]) + ', ' + str(self.vout[2]) + ', ' + str(self.vout[3]))
    print('dx_cmd_x10      = ' + str(self.dx_cmd_x10[0]) + ', ' + str(self.dx_cmd_x10[1]) + ', ' + str(self.dx_cmd_x10[2]))
  
  def cmd_wrk(self, x, y, th):
    self.sample_num_host += 1
    if self.sample_num_host <= self.sample_num_ll:   self.sample_num_host = self.sample_num_ll
    elif self.sample_num_host >= self.sample_num_ul: self.sample_num_host = 0
    if x <= self.x_ll[0]:    x = self.x_ll[0]
    elif x >= self.x_ul[0]:  x = self.x_ul[0]
    if y <= self.x_ll[1]:    y = self.x_ll[1]
    elif y >= self.x_ul[1]:  y = self.x_ul[1]
    if th <= self.x_ll[2]:   th = self.x_ll[2]
    elif th >= self.x_ul[2]: th = self.x_ul[2]
    # print('wrk,' + '{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(x)) + ',' + '{:0=+5}'.format(int(y)) + ',' + '{:0=+5}'.format(int(th)) + ',end')
    # self.con.write('wrk,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(x)) + ',' + '{:0=+5}'.format(int(y)) + ',' + '{:0=+5}'.format(int(th)) + ',end')
    
  
  def apply_filter(self):
    self.mx_lpf = self.mx_lpf + self.g_mag * self.dt * (self.mx - self.mx_lpf)
    self.my_lpf = self.my_lpf + self.g_mag * self.dt * (self.my - self.my_lpf)
  
  def odometry_update_res(self):
    omega_rad = np.array(
      [  self.omega_res_x10[0] * self.pi / 180.0
       , self.omega_res_x10[1] * self.pi / 180.0
       , self.omega_res_x10[2] * self.pi / 180.0
       , self.omega_res_x10[3] * self.pi / 180.0
      ])
    self.dx_res_x10 = np.dot(self.J_inv_plus, omega_rad)
    self.dx_res = self.dx_res_x10 / 10.0
    self.x_res[0] += self.dt * (self.dx_res_x10[0] / 10.0 * cos(self.x_res[2]) - self.dx_res_x10[1] / 10.0 * sin(self.x_res[2]))
    self.x_res[1] += self.dt * (self.dx_res_x10[0] / 10.0 * sin(self.x_res[2]) + self.dx_res_x10[1] / 10.0 * cos(self.x_res[2]))
    self.x_res_[2] += self.dt * (self.dx_res_x10[2] / 10.0)
    self.x_res[2] = atan2(self.my_lpf - self.my_offset, self.mx_lpf - self.mx_offset)
  
  def odometry_update_cmd(self):
    self.x_res2[0] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * cos(self.x_res[2]) - self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[1] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * sin(self.x_res[2]) + self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[2] += self.dt * (self.dx_cmd_x10[2] / 10.0 * self.pi / 180.0)
  
  def print_odom(self):
    print('OdomRes : {:=7.2f}, {:=7.2f}, {:=7.3f}, {:=7.3f}'.format(self.x_res[0], self.x_res[1], self.x_res[2], self.x_res_[2], self.interval))
    # print('OdomCmd : {:=7.2f}, {:=7.2f}, {:=7.3f}'.format(self.x_res2[0], self.x_res2[1], self.x_res2[2]))
  
  def fwd_kine(self):
    self.dx_res[0] = 0
    self.dx_res[1] = 0
    self.dx_res[2] = 0
  
  def inv_kine(self):
    self.omega_cmd_x10[0] = 0
    self.omega_cmd_x10[1] = 0
    self.omega_cmd_x10[2] = 0

  def readMEGA(self):
    tmp_str = arduino.conMEGA.read(arduino.con.inWaiting()) # シリアルのバッファー内のデータを取得
    tmp_list = (before_nokori + tmp_str).split('\n')    # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
    if len(tmp_list) >= 2:
      for i in range(0, len(tmp_list) - 1):
        tmp_val_list = tmp_list[i].split(',')
        arduino_pre = arduino
        sample_err = 0
        if len(tmp_val_list) == 31:
          if is_int(tmp_val_list[0]):  arduino.sample_num_node = int(tmp_val_list[0])
          else:                        sample_err = 1
          if is_int(tmp_val_list[1]):  arduino.cnt_now[0] = int(tmp_val_list[1])
          else:                        sample_err = 1
          if is_int(tmp_val_list[2]):  arduino.cnt_now[1] = int(tmp_val_list[2])
          else:                        sample_err = 1
          if is_int(tmp_val_list[3]):  arduino.cnt_now[2] = int(tmp_val_list[3])
          else:                        sample_err = 1
          if is_int(tmp_val_list[4]):  arduino.cnt_now[3] = int(tmp_val_list[4])
          else:                        sample_err = 1
          if is_int(tmp_val_list[5]):  arduino.omega_res_x10[0] = float(int(tmp_val_list[5]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[6]):  arduino.omega_res_x10[1] = float(int(tmp_val_list[6]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[7]):  arduino.omega_res_x10[2] = float(int(tmp_val_list[7]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[8]):  arduino.omega_res_x10[3] = float(int(tmp_val_list[8]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[9]):  arduino.omega_cmd_x10[0] = float(int(tmp_val_list[9]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[10]): arduino.omega_cmd_x10[1] = float(int(tmp_val_list[10]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[11]): arduino.omega_cmd_x10[2] = float(int(tmp_val_list[11]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[12]): arduino.omega_cmd_x10[3] = float(int(tmp_val_list[12]))
          else:                        sample_err = 1
          if is_int(tmp_val_list[13]): arduino.vout[0] = int(tmp_val_list[13])
          else:                        sample_err = 1
          if is_int(tmp_val_list[14]): arduino.vout[1] = int(tmp_val_list[14])
          else:                        sample_err = 1
          if is_int(tmp_val_list[15]): arduino.vout[2] = int(tmp_val_list[15])
          else:                        sample_err = 1
          if is_int(tmp_val_list[16]): arduino.vout[3] = int(tmp_val_list[16])
          else:                        sample_err = 1
          if is_int(tmp_val_list[17]): arduino.ir_hex = int(tmp_val_list[17])
          else:                        sample_err = 1
          if is_int(tmp_val_list[18]): arduino.ax = int(tmp_val_list[18])
          else:                        sample_err = 1
          if is_int(tmp_val_list[19]): arduino.ay = int(tmp_val_list[19])
          else:                        sample_err = 1
          if is_int(tmp_val_list[20]): arduino.az = int(tmp_val_list[20])
          else:                        sample_err = 1
          if is_int(tmp_val_list[21]): arduino.gx = int(tmp_val_list[21])
          else:                        sample_err = 1
          if is_int(tmp_val_list[22]): arduino.gy = int(tmp_val_list[22])
          else:                        sample_err = 1
          if is_int(tmp_val_list[23]): arduino.gz = int(tmp_val_list[23])
          else:                        sample_err = 1
          if is_int(tmp_val_list[24]): arduino.mx = int(tmp_val_list[24])
          else:                        sample_err = 1
          if is_int(tmp_val_list[25]): arduino.my = int(tmp_val_list[25])
          else:                        sample_err = 1
          if is_int(tmp_val_list[26]): arduino.mz = int(tmp_val_list[26])
          else:                        sample_err = 1
          if is_int(tmp_val_list[27]): arduino.temp = int(tmp_val_list[27])
          else:                        sample_err = 1
          if is_int(tmp_val_list[28]): arduino.bat_vol_x100 = int(tmp_val_list[28])
          else:                        sample_err = 1
          if is_int(tmp_val_list[29]): arduino.ps2_ctrl = int(tmp_val_list[29])
          else:                        sample_err = 1
          if is_int(tmp_val_list[30]): arduino.interval = int(tmp_val_list[30])
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

  def readFM(self):
    # Initialize variables
    err_flag = 0
    sample_num_node_FM = 0
    cnt_now_0 = 0
    cnt_now_1 = 0
    omega_res_x10_0 = 0
    omega_res_x10_1 = 0
    omega_cmd_x10_0 = 0
    omega_cmd_x10_1 = 0
    vout_0 = 0
    vout_1 = 0
    interval = 0
    
    # Read serial data & Put it to class
    tmp_str = self.conFM.read(self.conFM.inWaiting())       # Read serial data
    tmp_list = (self.before_nokoriFM + tmp_str).split('\n') # Connect previous rest sample and this sample -> split by new line char
    if len(tmp_list) >= 2:
      for i in range(0, len(tmp_list) - 1):
        tmp_val_list = tmp_list[i].split(',')
        sample_err = 0
        if len(tmp_val_list) == 10:
          if is_int(tmp_val_list[0]):  sample_num_node_FM = int(tmp_val_list[0])
          else:                        sample_err = 1
          if is_int(tmp_val_list[1]):  cnt_now_0 = int(tmp_val_list[1])
          else:                        sample_err = 1
          if is_int(tmp_val_list[2]):  cnt_now_1 = int(tmp_val_list[2])
          else:                        sample_err = 1
          if is_int(tmp_val_list[3]):  omega_res_x10_0 = int(tmp_val_list[3])
          else:                        sample_err = 1
          if is_int(tmp_val_list[4]):  omega_res_x10_1 = int(tmp_val_list[4])
          else:                        sample_err = 1
          if is_int(tmp_val_list[5]):  omega_cmd_x10_0 = int(tmp_val_list[5])
          else:                        sample_err = 1
          if is_int(tmp_val_list[6]):  omega_cmd_x10_1 = int(tmp_val_list[6])
          else:                        sample_err = 1
          if is_int(tmp_val_list[7]):  vout_0 = int(tmp_val_list[7])
          else:                        sample_err = 1
          if is_int(tmp_val_list[8]):  vout_1 = int(tmp_val_list[8])
          else:                        sample_err = 1
          if is_int(tmp_val_list[9]):  interval = int(tmp_val_list[9])
          else:                        sample_err = 1
        else:
          err_flag = 1
        
        # If error not occured, put the data to class
        if sample_err == 0:
          self.sample_num_node_FM = sample_num_node_FM
          self.cnt_now[0] = cnt_now_0
          self.cnt_now[1] = cnt_now_1
          self.omega_res_x10[0] = omega_res_x10_0
          self.omega_res_x10[1] = omega_res_x10_1
          self.omega_cmd_x10[0] = omega_cmd_x10_0
          self.omega_cmd_x10[1] = omega_cmd_x10_1
          self.vout[0] = vout_0
          self.vout[1] = vout_1
          # self.interval[0] = interval
        else:
          print("Error in readFM: ")
          print(tmp_list[i])
    
    self.before_nokoriFM = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存

  def readRM(self):
    # Initialize variables
    err_flag = 0
    sample_num_node_RM = 0
    cnt_now_2 = 0
    cnt_now_3 = 0
    omega_res_x10_2 = 0
    omega_res_x10_3 = 0
    omega_cmd_x10_2 = 0
    omega_cmd_x10_3 = 0
    vout_2 = 0
    vout_3 = 0
    interval = 0
    
    # Read serial data & Put it to class
    tmp_str = self.conRM.read(self.conRM.inWaiting())       # Read serial data
    tmp_list = (self.before_nokoriRM + tmp_str).split('\n') # Connect previous rest sample and this sample -> split by new line char
    if len(tmp_list) >= 2:
      for i in range(0, len(tmp_list) - 1):
        tmp_val_list = tmp_list[i].split(',')
        sample_err = 0
        if len(tmp_val_list) == 10:
          if is_int(tmp_val_list[0]):  sample_num_node_RM = int(tmp_val_list[0])
          else:                        sample_err = 1
          if is_int(tmp_val_list[1]):  cnt_now_2 = int(tmp_val_list[1])
          else:                        sample_err = 1
          if is_int(tmp_val_list[2]):  cnt_now_3 = int(tmp_val_list[2])
          else:                        sample_err = 1
          if is_int(tmp_val_list[3]):  omega_res_x10_2 = int(tmp_val_list[3])
          else:                        sample_err = 1
          if is_int(tmp_val_list[4]):  omega_res_x10_3 = int(tmp_val_list[4])
          else:                        sample_err = 1
          if is_int(tmp_val_list[5]):  omega_cmd_x10_2 = int(tmp_val_list[5])
          else:                        sample_err = 1
          if is_int(tmp_val_list[6]):  omega_cmd_x10_3 = int(tmp_val_list[6])
          else:                        sample_err = 1
          if is_int(tmp_val_list[7]):  vout_2 = int(tmp_val_list[7])
          else:                        sample_err = 1
          if is_int(tmp_val_list[8]):  vout_3 = int(tmp_val_list[8])
          else:                        sample_err = 1
          if is_int(tmp_val_list[9]):  interval = int(tmp_val_list[9])
          else:                        sample_err = 1
        else:
          err_flag = 1
        
        # If error not occured, put the data to class
        if sample_err == 0:
          self.sample_num_node_RM = sample_num_node_RM
          self.cnt_now[2] = cnt_now_2
          self.cnt_now[3] = cnt_now_3
          self.omega_res_x10[2] = omega_res_x10_2
          self.omega_res_x10[3] = omega_res_x10_3
          self.omega_cmd_x10[2] = omega_cmd_x10_2
          self.omega_cmd_x10[3] = omega_cmd_x10_3
          self.vout[2] = vout_2
          self.vout[3] = vout_3
          # self.interval[0] = interval
        else:
          print("Error in readRM : ")
          print(tmp_list[i])
    
    self.before_nokoriRM = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存


  # def cmdFM(self):
    # if(dir == 0):
      # xx += 50
      # yy += 50
    # else:
      # xx -= 50
      # yy -= 50
    # if(xx >= 5000): dir = 1
    # if(xx <= 2500): dir = 0
    
    # kkk += 10
    # self.conFM.write('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(xx)) + ',' + '{:0=+5}'.format(int(yy)) + ',end')
  
  # def cmdRM(self):
    # self.conRM.write('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(xx)) + ',' + '{:0=+5}'.format(int(yy)) + ',end')
  