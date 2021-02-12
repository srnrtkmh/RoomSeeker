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
#           2021/01/18 Added inverse kinematics program                                            #
#           2021/01/21 Added sensor node communication program                                     #
#           2021/01/28 Bug fix in reading process from sensor node                                 #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
#                                                                                                  #
#==================================================================================================#

#==================================================================================================#
# Import Module                                                                                    #
#==================================================================================================#
import numpy as np    # For matrix calculation
from math import *    # For mathematical operation

#==================================================================================================#
# Constants                                                                                        #
#==================================================================================================#
# PS2 controller constants
PSB_SELECT      = 0x0001
PSB_L3          = 0x0002
PSB_R3          = 0x0004
PSB_START       = 0x0008
PSB_PAD_UP      = 0x0010
PSB_PAD_RIGHT   = 0x0020
PSB_PAD_DOWN    = 0x0040
PSB_PAD_LEFT    = 0x0080
PSB_L2          = 0x0100
PSB_R2          = 0x0200
PSB_L1          = 0x0400
PSB_R1          = 0x0800
PSB_GREEN       = 0x1000
PSB_RED         = 0x2000
PSB_BLUE        = 0x4000
PSB_PINK        = 0x8000
PSB_TRIANGLE    = 0x1000
PSB_CIRCLE      = 0x2000
PSB_CROSS       = 0x4000
PSB_SQUARE      = 0x8000

# Control Mode constants
MODE_TWIST = 0
MODE_PS2   = 1

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
# Descriminate the literal is Hexadecimal or not                                                   #
#==================================================================================================#
def is_hex(s):
  try:
    int(s, 16)
    return True
  except ValueError:
    return False

#==================================================================================================#
# RoomSeeker level 1 class                                                                         #
#   Index of each wheelspace value : 0 - Front Right, 1 - Fron Left, 2 - Rear Right, 3 - Rear Left #
#   Index of each workspace value  : 0 - Long dir, 1 - Width dir, 2 - Yaw angle                    # 
#==================================================================================================#
class RoomSeekerLevel1():
  def __init__(self):
    self.mode = MODE_TWIST                              # Controll mode 0 : command mode , 1 : PS2 controller mode
    self.sample_num_node_MEGA = 0                       # Sample number of the Arduino MEGA
    self.sample_num_node_FM = 0                         # Sample number of the Front Motor Controller
    self.sample_num_node_RM = 0                         # Sample number of the Rear Motor Controller
    self.sample_num_host = 0                            # Sample number of the host
    self.sample_num_ul = 999                            # Upper limit of sample number
    self.sample_num_ll = 0                              # Lower limit of sample number
    self.dt = 0.01                                      # Sampling time of the motor controller
    #self.dt = 0.1                                       # Sampling time of the motor controller
    self.ros_rate = 1/self.dt                           # Set value for rospy.Rate()
    self.ws_dir = np.array([-1.0, 1.0, -1.0, 1.0])      # Wheelspace direction co-efficient
    self.cnt_now = np.array([0.0, 0.0, 0.0, 0.0])       # Wheel encoder counted value
    self.omega_res = np.array([0.0, 0.0, 0.0, 0.0])     # Wheel space velocity response [rad/s]
    self.omega_res_x10 = np.array([0.0, 0.0, 0.0, 0.0]) # Wheel space velocity response [10^-1deg/s]
    self.omega_cmd = np.array([0.0, 0.0, 0.0, 0.0])     # Wheel space velocity command [rad/s]
    self.omega_cmd_x10 = np.array([0.0, 0.0, 0.0, 0.0]) # Wheel space velocity command [10^-1deg/s]
    self.vout = np.array([0, 0, 0, 0])                  # Voltage output (PWM width : 0 - 4095)
    self.x_res = np.array([0.0, 0.0, 0.0])              # Workspace pose response calculated from velocity response [0:m, 1:m, 2:rad]
    self.x_res_ = np.array([0.0, 0.0, 0.0])             # 
    self.x_res2 = np.array([0.0, 0.0, 0.0])             # Workspace pose response calculated from velocity command [0:m, 1:m, 2:rad]
    self.dx_res = np.array([0.0, 0.0, 0.0])             # Workspace velocity response [0 : m/s, 1 : m/s, 2 : deg/s]
    self.dx_res_x10 = np.array([0.0, 0.0, 0.0])         # Workspace velocity response [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    self.x_cmd = np.array([0.0, 0.0, 0.0])              # Workspace pose command [0:m, 1:m, 2:rad]
    self.dx_cmd = np.array([0.0, 0.0, 0.0])             # Workspace velocity command [0 : m/s, 1 : m/s, 2 : rad/s]
    self.dx_rate_limit = np.array([0.010, 0.010, 0.050])# Rate limit value in workspace [0 : m/20ms, 1 : m/20ms, 2 : rad/20ms]
    self.dx_cmd_rl = np.array([0.0, 0.0, 0.0])          # Workspace velocity command applied rate limit [0 : m/s, 1 : m/s, 2 : rad/s]
    self.dx_cmd_x10 = np.array([0.0, 0.0, 0.0])         # Workspace velocity command x10ver [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    self.wheel_radius = 45.0                            # Robot wheel radius [mm]
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
    self.gc = -0.06097 / 180.0 * 3.141592               # Gyro constants to transform raw data to rad/s [rad/s]
    self.gx = 0                                         # Gyro x-axis [rad/s]
    self.gy = 0                                         # Gyro y-axis [rad/s]
    self.gz = 0                                         # Gyro z-axis [rad/s]
    self.gz_hpf = 0.0                                   # Gyro z-axis applied HPF [rad/s]
    self.gz_hpf_tmp = 0.0                               # Temporary for HPF [rad/s]
    self.int_gz_hpf = 0.0                               # Integral of gz applied HPF [rad]
    self.g_gz = 1.0                                     # Cutoff angular frequency of HPF for gyro sensor [rad/s]
    self.mx = 0                                         # Mag x-axis
    self.my = 0                                         # Mag y-axis
    self.mz = 0                                         # Mag z-axis
    self.mx_lpf = 0                                     # Mag x-axis applied LPF
    self.my_lpf = 0                                     # Mag y-axis applied LPF
    self.mz_lpf = 0                                     # Mag z-axis applied LPF
    #self.mx_offset =  27.715769                         # Offset of Mag x-axis
    self.mx_offset =  70.0                               # Offset of Mag x-axis
    #self.my_offset = -81.559509                         # Offset of Mag y-axis
    self.my_offset = -25.0                              # Offset of Mag y-axis
    self.mz_offset = 0.0                                # Offset of Mag y-axis
    self.mr_offset = 177.26162                          # Radius of Mag x-y circle
    self.g_mag = 1.0                                    # Cutoff angular frequency of LPF for magnetosensor [rad/s]
    self.temp = 0                                       # Temperature
    self.us_dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Distance from ultra sonic sensor
    self.us_ok = 0                                      # Ultrasonic sensor is enabled or not
    self.ps2_button = 0                                 # PS2 controller button state
    self.ps2_analogRX = 0                               # PS2 controller button state
    self.ps2_analogRY = 0                               # PS2 controller button state
    self.ps2_analogLX = 0                               # PS2 controller button state
    self.ps2_analogRY = 0                               # PS2 controller button state
    self.conFM = 0                                      # Serial console connected to arduino motor controller
    self.conRM = 0                                      # Serial console connected to arduino motor controller
    self.conMEGA = 0                                    # Serial console connected to arduino motor controller
    self.bat_vol = 0                                    # Battery voltage
    self.bat_vol_x100 = 0                               # Battery voltage x100 (raw data)
    self.ps2_ctrl = 0                                   # PS2 controller is enabled or not
    self.interval_MEGA = 0                              # Interval between samples
    self.interval_FM = 0                                # Interval between samples
    self.interval_RM = 0                                # Interval between samples
    self.interval = 0                                   # Interval
    self.pi = 3.141592
    self.kkk = 0
    self.before_nokoriMEGA = ''
    self.before_nokoriFM = ''
    self.before_nokoriRM = ''
    
    # Power control
    self.control_on = 0
    self.no_input_from_ps2 = 0
    self.no_input_from_ps2_cnt = 0
    self.no_input_from_twist = 0
    self.no_input_from_twist_cnt = 0
  
  def print_state(self):
    print('sample_num_node = ' + str(self.sample_num_node_FM))
    print('cnt_now         = ' + str(self.cnt_now[0]) + ', ' + str(self.cnt_now[1]) + ', ' + str(self.cnt_now[2]) + ', ' + str(self.cnt_now[3]))
    print('omega_res_x10   = ' + str(self.omega_res_x10[0]) + ', ' + str(self.omega_res_x10[1]) + ', ' + str(self.omega_res_x10[2]) + ', ' + str(self.omega_res_x10[3]))
    print('omega_cmd_x10   = ' + str(self.omega_cmd_x10[0]) + ', ' + str(self.omega_cmd_x10[1]) + ', ' + str(self.omega_cmd_x10[2]) + ', ' + str(self.omega_cmd_x10[3]))
    print('vout            = ' + str(self.vout[0]) + ', ' + str(self.vout[1]) + ', ' + str(self.vout[2]) + ', ' + str(self.vout[3]))
    print('dx_cmd_x10      = ' + str(self.dx_cmd_x10[0]) + ', ' + str(self.dx_cmd_x10[1]) + ', ' + str(self.dx_cmd_x10[2]))
  
  def print_sensor(self):
    print('sample_num_node_MEGA = ' + str(self.sample_num_node_MEGA))
    print('ir_hex = ' + hex(self.ir_hex) + ', ir_bin = ' + bin(self.ir_hex))
    print('ax = ' + '{:=+6}'.format(int(self.ax)) + ', ay = ' + '{:=+6}'.format(int(self.ay)) + ', az = ' + '{:=+6}'.format(int(self.az)))
    print('gx = ' + '{:=+6}'.format(int(self.gx)) + ', gy = ' + '{:=+6}'.format(int(self.gy)) + ', gz = ' + '{:=+6}'.format(int(self.gz)))
    print('mx = ' + '{:=+6}'.format(int(self.mx)) + ', my = ' + '{:=+6}'.format(int(self.my)) + ', mz = ' + '{:=+6}'.format(int(self.mz)) + ', temp = {:=+6}'.format(int(self.temp)))
    print('us_dist = ' + '{:=5}'.format(int(self.us_dist[0])) + ', ' + '{:=6}'.format(int(self.us_dist[2])) + ', ' + '{:=6}'.format(int(self.us_dist[2])) + ', ' + '{:=6}'.format(int(self.us_dist[3])))
    print('us_dist = ' + '{:=5}'.format(int(self.us_dist[4])) + ', ' + '{:=6}'.format(int(self.us_dist[5])) + ', ' + '{:=6}'.format(int(self.us_dist[6])) + ', ' + '{:=6}'.format(int(self.us_dist[7])))
    print('us_ok = ' + hex(self.us_ok) + ', bat_vol_x100 = ' + '{:=4}'.format(int(self.bat_vol_x100)))
    print('ps2_button = ' + hex(self.ps2_button) + ', ' + '{:=3}'.format(int(self.ps2_analogRX)) + ', ' + '{:=3}'.format(int(self.ps2_analogRY)) + ', ' + '{:=3}'.format(int(self.ps2_analogLX)) + ', ' + '{:=3}'.format(int(self.ps2_analogLY)))
    print('interval_MEGA = ' + '{:=6}'.format(int(self.interval_MEGA)))
    
  def print_wrk(self):
    print('arduino.mode = ' + str(self.mode))
    print('dx_cmd = {:=+4}, {:=+4}, {:=+4}'.format(self.dx_cmd[0], self.dx_cmd[1], self.dx_cmd[2]))
    print('omega_cmd     = {:=+4}, {:=+4}, {:=+4}, {:=+4}'.format(self.omega_cmd[0], self.omega_cmd[1], self.omega_cmd[2], self.omega_cmd[3]))
    print('omega_cmd_x10 = {:=+4}, {:=+4}, {:=+4}, {:=+4}'.format(self.omega_cmd_x10[0], self.omega_cmd_x10[1], self.omega_cmd_x10[2], self.omega_cmd_x10[3]))
  
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
    self.gz_hpf_tmp += self.dt * self.gz_hpf
    self.gz_hpf = self.gz - self.g_gz * self.gz_hpf_tmp
    self.int_gz_hpf += self.dt * self.gz_hpf
  
  def odometry_update_res(self):
    omega_rad = np.array(
      [  self.omega_res_x10[0] * self.pi / 1800.0
       , self.omega_res_x10[1] * self.pi / 1800.0
       , self.omega_res_x10[2] * self.pi / 1800.0
       , self.omega_res_x10[3] * self.pi / 1800.0
      ])
    self.dx_res = np.dot(self.J_inv_plus, omega_rad)
    self.x_res[0] += self.dt * (self.dx_res[0] * cos(self.x_res[2]) - self.dx_res[1] * sin(self.x_res[2]))
    self.x_res[1] += self.dt * (self.dx_res[0] * sin(self.x_res[2]) + self.dx_res[1] * cos(self.x_res[2]))
    self.x_res_[2] = atan2(self.my_lpf - self.my_offset, self.mx_lpf - self.mx_offset)
    # self.x_res[2] = atan2(self.my_lpf - self.my_offset, self.mx_lpf - self.mx_offset) + self.int_gz_hpf
    self.x_res[2] += self.dt * (self.dx_res[2])
  
  def odometry_update_cmd(self): # Not in use it cannnot estimate pose enough precision
    self.x_res2[0] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * cos(self.x_res[2]) - self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[1] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * sin(self.x_res[2]) + self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[2] += self.dt * (self.dx_cmd_x10[2] / 10.0 * self.pi / 180.0)
  
  def print_odom(self):
    print('OdomRes : {:=7.2f}, {:=7.2f}, {:=7.3f}, Battery = {:=2.2f}'.format(self.x_res[0], self.x_res[1], self.x_res[2]/3.141592*180.0, self.bat_vol))
    # print('OdomCmd : {:=7.2f}, {:=7.2f}, {:=7.3f}'.format(self.x_res2[0], self.x_res2[1], self.x_res2[2]))
  
  def fwd_kine(self):
    self.dx_res[0] = 0
    self.dx_res[1] = 0
    self.dx_res[2] = 0
    
  def rate_limit_work(self):
    for i in range(0, len(self.dx_cmd)):
      if(self.dx_cmd[i] > self.dx_cmd_rl[i] + self.dx_rate_limit[i]):     self.dx_cmd_rl[i] += self.dx_rate_limit[i];
      elif (self.dx_cmd[i] < self.dx_cmd_rl[i] - self.dx_rate_limit[i]):  self.dx_cmd_rl[i] -= self.dx_rate_limit[i];
      else:                                                               self.dx_cmd_rl[i]  = self.dx_cmd[i];
  
  def inv_kine(self):
    self.omega_cmd[0] = (self.dx_cmd_rl[0] + self.dx_cmd_rl[1] + (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    self.omega_cmd[1] = (self.dx_cmd_rl[0] - self.dx_cmd_rl[1] - (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    self.omega_cmd[2] = (self.dx_cmd_rl[0] - self.dx_cmd_rl[1] + (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    self.omega_cmd[3] = (self.dx_cmd_rl[0] + self.dx_cmd_rl[1] - (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
  
  def command_motor_node(self):
    self.omega_cmd_x10[0] = int(self.omega_cmd[0] / 3.141592 * 1800)
    self.omega_cmd_x10[1] = int(self.omega_cmd[1] / 3.141592 * 1800)
    self.omega_cmd_x10[2] = int(self.omega_cmd[2] / 3.141592 * 1800)
    self.omega_cmd_x10[3] = int(self.omega_cmd[3] / 3.141592 * 1800)
    self.conFM.write('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[0] * self.omega_cmd_x10[0])) + ',' + '{:0=+5}'.format(int(self.ws_dir[1] * self.omega_cmd_x10[1])) + ',end')
    self.conRM.write('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[2] * self.omega_cmd_x10[2])) + ',' + '{:0=+5}'.format(int(self.ws_dir[3] * self.omega_cmd_x10[3])) + ',end')
    # print('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[0] * self.omega_cmd_x10[0])) + ',' + '{:0=+5}'.format(int(self.ws_dir[1] * self.omega_cmd_x10[1])) + ',end')
    # print('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[2] * self.omega_cmd_x10[2])) + ',' + '{:0=+5}'.format(int(self.ws_dir[3] * self.omega_cmd_x10[3])) + ',end')

  def readMEGA(self):
    sample_num_node_MEGA = 0
    ir_hex = 0
    ax = 0
    ay = 0
    az = 0
    gx = 0
    gy = 0
    gz = 0
    mx = 0
    my = 0
    mz = 0
    temp = 0
    us_dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    us_ok = 0
    bat_vol_x100 = 0
    ps2_button = 0
    ps2_analogRX = 0
    ps2_analogRY = 0
    ps2_analogLX = 0
    ps2_analogLY = 0
    interval_MEGA = 0
  
    tmp_str = self.conMEGA.read(self.conMEGA.inWaiting()) # シリアルのバッファー内のデータを取得
    tmp_list = (self.before_nokoriMEGA + tmp_str).split('\n')      # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
    if len(tmp_list) >= 2:
      for i in range(0, len(tmp_list) - 1):
        tmp_val_list = tmp_list[i].split(',')
        sample_err = 0
        if len(tmp_val_list) == 28:
          if is_int(tmp_val_list[0]):   sample_num_node_MEGA = int(tmp_val_list[0])
          else:                         sample_err = 1
          if is_hex(tmp_val_list[1]):   ir_hex = int(tmp_val_list[1], 16)
          else:                         sample_err = 2
          if is_int(tmp_val_list[2]):   ax = int(tmp_val_list[2])
          else:                         sample_err = 3
          if is_int(tmp_val_list[3]):   ay = int(tmp_val_list[3])
          else:                         sample_err = 4
          if is_int(tmp_val_list[4]):   az = int(tmp_val_list[4])
          else:                         sample_err = 5
          if is_int(tmp_val_list[5]):   gx = int(tmp_val_list[5])
          else:                         sample_err = 6
          if is_int(tmp_val_list[6]):   gy = int(tmp_val_list[6])
          else:                         sample_err = 7
          if is_int(tmp_val_list[7]):   gz = int(tmp_val_list[7])
          else:                         sample_err = 8
          if is_int(tmp_val_list[8]):   mx = int(tmp_val_list[8])
          else:                         sample_err = 9
          if is_int(tmp_val_list[9]):   my = int(tmp_val_list[9])
          else:                         sample_err = 10
          if is_int(tmp_val_list[10]):  mz = int(tmp_val_list[10])
          else:                         sample_err = 11
          if is_int(tmp_val_list[11]):  temp = int(tmp_val_list[11])
          else:                         sample_err = 12
          if is_int(tmp_val_list[12]):  us_dist[0] = float(tmp_val_list[12])
          else:                         sample_err = 13
          if is_int(tmp_val_list[13]):  us_dist[1] = float(tmp_val_list[13])
          else:                         sample_err = 14
          if is_int(tmp_val_list[14]):  us_dist[2] = float(tmp_val_list[14])
          else:                         sample_err = 15
          if is_int(tmp_val_list[15]):  us_dist[3] = float(tmp_val_list[15])
          else:                         sample_err = 16
          if is_int(tmp_val_list[16]):  us_dist[4] = float(tmp_val_list[16])
          else:                         sample_err = 17
          if is_int(tmp_val_list[17]):  us_dist[5] = float(tmp_val_list[17])
          else:                         sample_err = 18
          if is_int(tmp_val_list[18]):  us_dist[6] = float(tmp_val_list[18])
          else:                         sample_err = 19
          if is_int(tmp_val_list[19]):  us_dist[7] = float(tmp_val_list[19])
          else:                         sample_err = 20
          if is_hex(tmp_val_list[20]):  us_ok = int(tmp_val_list[20], 16)
          else:                         sample_err = 21
          if is_int(tmp_val_list[21]):  bat_vol_x100 = int(tmp_val_list[21])
          else:                         sample_err = 22
          if is_hex(tmp_val_list[22]):  ps2_button = int(tmp_val_list[22], 16)
          else:                         sample_err = 23
          if is_int(tmp_val_list[23]):  ps2_analogRX = int(tmp_val_list[23])
          else:                         sample_err = 24
          if is_int(tmp_val_list[24]):  ps2_analogRY = int(tmp_val_list[24])
          else:                         sample_err = 25
          if is_int(tmp_val_list[25]):  ps2_analogLX = int(tmp_val_list[25])
          else:                         sample_err = 26
          if is_int(tmp_val_list[26]):  ps2_analogLY = int(tmp_val_list[26])
          else:                         sample_err = 27
          if is_int(tmp_val_list[27]):  interval_MEGA = int(tmp_val_list[27])
          else:                         sample_err = 28
        else:
          sample_err = 99
        
        # If error not occured, put the data to class
        if sample_err == 0:
          self.sample_num_node_MEGA = sample_num_node_MEGA
          self.ir_hex = ir_hex
          self.ax = ax
          self.ay = ay
          self.az = az
          self.gx = self.gc * gx
          self.gy = self.gc * gy
          self.gz = self.gc * gz
          self.mx = mx
          self.my = my
          self.mz = mz
          self.temp = temp
          self.us_dist[0] = us_dist[0]
          self.us_dist[1] = us_dist[1]
          self.us_dist[2] = us_dist[2]
          self.us_dist[3] = us_dist[3]
          self.us_dist[4] = us_dist[4]
          self.us_dist[5] = us_dist[5]
          self.us_dist[6] = us_dist[6]
          self.us_dist[7] = us_dist[7]
          self.us_ok = us_ok
          self.bat_vol_x100 = bat_vol_x100
          self.bat_vol = self.bat_vol_x100 / 100.0
          self.ps2_button = ps2_button
          self.ps2_analogRX = ps2_analogRX
          self.ps2_analogRY = ps2_analogRY
          self.ps2_analogLX = ps2_analogLX
          self.ps2_analogLY = ps2_analogLY
          self.interval_MEGA = interval_MEGA
        else:
          print("Error in read MEGA: " + str(sample_err) + ": ")
          print(tmp_list[i])
      
    self.before_nokoriMEGA = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存

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
          self.cnt_now[0] = self.ws_dir[0] * cnt_now_0
          self.cnt_now[1] = self.ws_dir[1] * cnt_now_1
          self.omega_res_x10[0] = self.ws_dir[0] * omega_res_x10_0
          self.omega_res[0] = self.omega_res_x10[0] / 1800.0 * 3.141592
          self.omega_res_x10[1] = self.ws_dir[1] * omega_res_x10_1
          self.omega_res[1] = self.omega_res_x10[1] / 1800.0 * 3.141592
          self.omega_cmd_x10[0] = self.ws_dir[0] * omega_cmd_x10_0
          self.omega_cmd_x10[1] = self.ws_dir[1] * omega_cmd_x10_1
          self.vout[0] = self.ws_dir[0] * vout_0
          self.vout[1] = self.ws_dir[1] * vout_1
          # self.interval[0] = interval
        else:
          print("Error in read FM: ")
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
          self.cnt_now[2] = self.ws_dir[2] * cnt_now_2
          self.cnt_now[3] = self.ws_dir[3] * cnt_now_3
          self.omega_res_x10[2] = self.ws_dir[2] * omega_res_x10_2
          self.omega_res[2] = self.omega_res_x10[2] / 1800.0 * 3.141592
          self.omega_res_x10[3] = self.ws_dir[3] * omega_res_x10_3
          self.omega_res[3] = self.omega_res_x10[3] / 1800.0 * 3.141592
          self.omega_cmd_x10[2] = self.ws_dir[2] * omega_cmd_x10_2
          self.omega_cmd_x10[3] = self.ws_dir[3] * omega_cmd_x10_3
          self.vout[2] = self.ws_dir[2] * vout_2
          self.vout[3] = self.ws_dir[3] * vout_3
          # self.interval[0] = interval
        else:
          print("Error in read RM : ")
          print(tmp_list[i])
    
    self.before_nokoriRM = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存

