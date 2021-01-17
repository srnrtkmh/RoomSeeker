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
# RoomSeeker level 1 class                                                                         #
#==================================================================================================#
class RoomSeekerLevel1():
  def __init__(self):
    self.sample_num_node = 0                            # Sample number of the motor controller
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
    self.con = 0                                        # Serial console connected to arduino motor controller
    self.bat_vol_x100 = 0                               # Battery voltage
    self.ps2_ctrl = 0                                   # PS2 controller is enabled or not
    self.interval = 0                                   # Interval between samples
    self.pi = 3.141592
  
  def print_state(self):
    print('sample_num_node = ' + str(self.sample_num_node))
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
    self.con.write('wrk,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(x)) + ',' + '{:0=+5}'.format(int(y)) + ',' + '{:0=+5}'.format(int(th)) + ',end')
  
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


