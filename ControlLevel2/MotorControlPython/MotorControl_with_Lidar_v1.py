#!/usr/bin/env python
# coding: cp932
#==================================================================================================#
#                                                                                                  #
# FILE : MotorControl_with_Lidar_v1.py                                                             #
# Memo : Motor control test program in constant velocity command                                   #
#                                                                                                  #
# Updated : 2020/09/20 Started this project based on "MotorControlTest_v2.py"                      #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     python MotorControl_with_Lidar_v1.py [Serial device name]                                    #
#         or                                                                                       #
#     nohup python MotorControl_with_Lidar_v1.py [Serial device name] > [tty????.txt] &            #
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
import sys            # sys(exit()�̎g�p, �����擾�̂��߂ɕK�v)
import os             # OS(�t�@�C���̊e�폈���̂��߂ɕK�v)
import locale         # local(!)
import datetime       # datetime���W���[��
import time           # �����擾�p���W���[��
import threading      # �}���`�X���b�h�v���O���~���O�p���W���[��
from math import *    # �Z�p���Z�p���W���[��
import serial         # Arduino�Ƃ̒ʐM�Ŏg�p����V���A���ʐM�p���W���[��
import signal         # �V�O�i����M�Ɏg�p���郂�W���[��
import shutil         # �f�B���N�g������łȂ��Ă��폜���邽�߂Ɏg�����W���[��

import rospy
from std_msgs.msg import String

#==================================================================================================#
# Arduino motor control class                                                                      #
#==================================================================================================#
class arduino_motor_control():
  def __init__(self):
    self.sample_num_node = 0
    self.sample_num_host = 0
    self.sample_num_ul = 999
    self.sample_num_ll = 0
    self.dt = 0.02
    self.cnt_now = [0, 0, 0, 0]
    self.omega_res_x10 = [0, 0, 0, 0]
    self.omega_cmd_x10 = [0, 0, 0, 0]
    self.vout = [0, 0, 0, 0]
    self.x_res = [0, 0, 0]
    self.x_res2 = [0, 0, 0]
    self.dx_res_x10 = [0, 0, 0]
    self.x_cmd = [0, 0, 0]
    self.dx_cmd_x10 = [0, 0, 0]
    self.x_ul = [ 5000,  5000,  5000]
    self.x_ll = [-5000, -5000, -5000]
    self.con = 0
  
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
    # print('wrk,' + '{:0=3}'.format(self.sample_num_host) + ',' + '{:0=+5}'.format(x) + ',' + '{:0=+5}'.format(y) + ',' + '{:0=+5}'.format(th) + ',end')
    self.con.write('wrk,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(x) + ',' + '{:0=+5}'.format(y) + ',' + '{:0=+5}'.format(th) + ',end')
  
  def odometry_update_res(self):
    self.x_res[0] += self.dt * self.dx_res_x10[0] / 10.0
    self.x_res[1] += self.dt * self.dx_res_x10[1] / 10.0
    self.x_res[2] += self.dt * self.dx_res_x10[2] / 100.0
  
  def odometry_update_cmd(self):
    self.x_res2[0] += self.dt * self.dx_cmd_x10[0] / 10.0
    self.x_res2[1] += self.dt * self.dx_cmd_x10[1] / 10.0
    self.x_res2[2] += self.dt * self.dx_cmd_x10[2] / 100.0
  
  def print_odom(self):
    print('OdomRes : {:=7.2f}, {:=7.2f}, {:=7.3f}'.format(self.x_res[0], self.x_res[1], self.x_res[2]))
    print('OdomCmd : {:=7.2f}, {:=7.2f}, {:=7.3f}'.format(self.x_res2[0], self.x_res2[1], self.x_res2[2]))
  
  def fwd_kine(self):
    self.dx_res[0] = 0
    self.dx_res[1] = 0
    self.dx_res[2] = 0
  
  def inv_kine(self):
    self.omega_cmd_x10[0] = 0
    self.omega_cmd_x10[1] = 0
    self.omega_cmd_x10[2] = 0

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
serDevName = "/dev/ttyACM0" # �V���A���f�o�C�X�̐ݒ�
baudrate = 230400                 # �{�[���[�g�ݒ�
data_file = []                    # CSV�o�͗p�t�@�C���I�u�W�F�N�g
strCsvName = ""                   # �t�@�C�����p�ϐ�
preffix = "MotorControl_with_Lidar" # �t�@�C�����v���t�B�b�N�X
suffix = ""                       # �t�@�C�����T�t�B�b�N�X
before_nokori = ""

# Robot control variables
arduino_pre = arduino_motor_control()
arduino = arduino_motor_control()

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
  print "Received signal :", signum
  currentDate = datetime.datetime.today()  # today()���\�b�h�Ō��ݓ��t�E������datetime�^�f�[�^�̕ϐ����擾
  log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  arduino.con.write("t")
  time.sleep(0.01)
  arduino.con.write("t")
  time.sleep(0.01)
  arduino.con.write("t")
  data_file.close()
  log_file.close()
  sys.exit(0)

#==================================================================================================#
# Keyboard input function                                                                          #
#==================================================================================================#
def key_input():
  print("key_input started.")
  while True:
    test = raw_input()
    
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
  # ��������
  param = sys.argv
  if len(param) >= 2:
    serDevName = param[1]    # �V���A���f�o�C�X�ւ̃p�X
  else:
    print "Invalid argument. Valid command : \"" + param[0] + " [Serial device name]\""
    sys.exit(1)
  
  # �����ݒ�
  currentDate = datetime.datetime.today() # today()���\�b�h�Ō��ݓ��t�E������datetime�^�f�[�^�̕ϐ����擾
  
  # �V�O�i���n���h���֌W�̐ݒ�
  signal.signal(signal.SIGINT, receive_signal)                # SIGINT���󂯎������w��֐����Ăяo���悤�ɐݒ�
  
  # �L�[�{�[�h���͗p�X���b�h�쐬
  th_key = threading.Thread(target=key_input)
  th_key.setDaemon(True)
  th_key.start()
  
  # ���O�t�@�C�����J��
  log_file_name = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
  log_file = open(log_file_name, 'w')     # ���O�t�@�C����V�K�쐬
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # �f�[�^�t�@�C�����J��
  strCsvName = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  data_file = open(strCsvName, 'w')       # �t�@�C�����쐬
  data_file.write("time,n,cnt_now[0],cnt_now[1],cnt_now[2],cnt_now[3],omega_res_x10[0],omega_res_x10[1],omega_res_x10[2],omega_res_x10[3],omega_cmd_x10[0],omega_cmd_x10[1],omega_cmd_x10[2],omega_cmd_x10[3],vout[0],vout[1],vout[2],vout[3]\n")
  log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
  
  # �p�u���b�V���[�ݒ�
  pub = rospy.Publisher('test_pub', String, queue_size=100) # Publisher settings
  rospy.init_node('level1_node', anonymous=True)            # Initialize node
  rate = rospy.Rate(10)                                     # Sampling time [Hz]
  log_file.write(str(currentDate) + " : ROS Settings finished.\n")
  
  # �V���A���ʐM�|�[�g���Z�b�g�A�b�v
  try:
    arduino.con = serial.Serial(serDevName, baudrate, timeout=0, rtscts=False, dsrdtr=False)  # �|�[�g�̃I�[�v��(timeout�̐ݒ�͓K���Artscts��dsrdtr��True�ɂ��Ă����Ȃ���ArduinoMicro�Ƃ͒ʐM�ł��Ȃ�)
    time.sleep(1.0)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # �ǂݎ��o�b�t�@����ɂ��Ă���
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + arduino.con.portstr + "\n")
  
  # �ŏ���1�񂾂��X�^�[�g�R�}���h�𑗐M
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # �ǂݎ��o�b�t�@����ɂ��Ă���(�O�ɋL�q����1�񂾂����Ƌ�ɂȂ�Ȃ���������(?))
  time.sleep(1.0)                         # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
  arduino.con.write("s")                          # �X�^�[�g�R�}���h�𑗐M(1���)
  log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")      # �X�^�[�g�R�}���h�𑗂����|���O
  time.sleep(1.0)                         # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
  tmp_str = arduino.con.read()                    # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
  if len(tmp_str) >= 10:
    arduino.con.write("s")                        # �X�^�[�g�R�}���h�𑗐M(2���)
    log_file.write(str(currentDate) + " : Start command transmitted (2nd try).\n")    # �X�^�[�g�R�}���h�𑗂����|���O
    time.sleep(1.0)                       # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
    tmp_str = arduino.con.read()                  # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
    if len(tmp_str) >= 10:
      arduino.con.write("s")                      # �X�^�[�g�R�}���h�𑗐M(3���)
      log_file.write(str(currentDate) + " : Start command transmitted (3rd try).\n")  # �X�^�[�g�R�}���h�𑗂����|���O
      time.sleep(1.0)                     # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
      tmp_str = arduino.con.read()                # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
      if len(tmp_str) >= 10:
        print "Serial Device did not transmit call back : " + tmp_str
        arduino.con.write("t")                    # �O�̂��߃X�g�b�v�R�}���h�𑗂��Ă���
        sys.exit(1)
  log_file.write(str(currentDate) + " : Start command echo back received.\n")
  
  # ����ϐ�������
  ctrl_cnt = 0
  ctrl_state = 0
  ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
  
  # ���������J�n
  try:
    while True:
      # ���䏈��
      arduino.cmd_wrk(arduino.dx_cmd_x10[0], arduino.dx_cmd_x10[1], arduino.dx_cmd_x10[2])
      
      # ���݂̓������擾
      currentDate = datetime.datetime.today()
      
      # �ǂݎ��f�[�^���t�@�C���ɏ�������
      tmp_str = arduino.con.read(arduino.con.inWaiting())               # 
      tmp_list = (before_nokori + tmp_str).split('\n')  # �O��T���v���c��ƍ���T���v�����Ȃ����킹�ĉ��s�ŕ���
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          tmp_val_list = tmp_list[i].split(',')
          arduino_pre = arduino
          sample_err = 0
          if len(tmp_val_list) == 17:
            if is_int(tmp_val_list[0]):  arduino.sample_num_node = int(tmp_val_list[0])
            else:                        sample_err = 1
            if is_int(tmp_val_list[1]):  arduino.cnt_now[0] = int(tmp_val_list[1])
            else:                        arduino.sample_err = 1
            if is_int(tmp_val_list[2]):  arduino.cnt_now[1] = int(tmp_val_list[2])
            else:                        sample_err = 1
            if is_int(tmp_val_list[3]):  arduino.cnt_now[2] = int(tmp_val_list[3])
            else:                        sample_err = 1
            if is_int(tmp_val_list[4]):  arduino.cnt_now[3] = int(tmp_val_list[4])
            else:                        sample_err = 1
            if is_int(tmp_val_list[5]):  arduino.omega_res_x10[0] = int(tmp_val_list[5])
            else:                        sample_err = 1
            if is_int(tmp_val_list[6]):  arduino.omega_res_x10[1] = int(tmp_val_list[6])
            else:                        sample_err = 1
            if is_int(tmp_val_list[7]):  arduino.omega_res_x10[2] = int(tmp_val_list[7])
            else:                        sample_err = 1
            if is_int(tmp_val_list[8]):  arduino.omega_res_x10[3] = int(tmp_val_list[8])
            else:                        sample_err = 1
            if is_int(tmp_val_list[9]):  arduino.omega_cmd_x10[0] = int(tmp_val_list[9])
            else:                        sample_err = 1
            if is_int(tmp_val_list[10]): arduino.omega_cmd_x10[1] = int(tmp_val_list[10])
            else:                        sample_err = 1
            if is_int(tmp_val_list[11]): arduino.omega_cmd_x10[2] = int(tmp_val_list[11])
            else:                        sample_err = 1
            if is_int(tmp_val_list[12]): arduino.omega_cmd_x10[3] = int(tmp_val_list[12])
            else:                        sample_err = 1
            if is_int(tmp_val_list[13]): arduino.vout[0] = int(tmp_val_list[13])
            else:                        sample_err = 1
            if is_int(tmp_val_list[14]): arduino.vout[1] = int(tmp_val_list[14])
            else:                        sample_err = 1
            if is_int(tmp_val_list[15]): arduino.vout[2] = int(tmp_val_list[15])
            else:                        sample_err = 1
            if is_int(tmp_val_list[16]): arduino.vout[3] = int(tmp_val_list[16])
            else:                        sample_err = 1
          else:
            sample_err = 1
            arduino = arduino_pre
          
          arduino.odometry_update_cmd()
          arduino.odometry_update_res()
          
          if sample_err != 0: print("Error occured : " + tmp_list[i])
          data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000)
            + ',' + str(arduino.sample_num_node) + ',' + str(arduino.cnt_now[0]) + ',' + str(arduino.cnt_now[1]) + ',' + str(arduino.cnt_now[2]) + ',' + str(arduino.cnt_now[3])
            + ',' + str(arduino.omega_res_x10[0]) + ',' + str(arduino.omega_res_x10[1]) + ',' + str(arduino.omega_res_x10[2]) + ',' + str(arduino.omega_res_x10[3])
            + ',' + str(arduino.omega_cmd_x10[0]) + ',' + str(arduino.omega_cmd_x10[1]) + ',' + str(arduino.omega_cmd_x10[2]) + ',' + str(arduino.omega_cmd_x10[3])
            + ',' + str(arduino.vout[0]) + ',' + str(arduino.vout[1]) + ',' + str(arduino.vout[2]) + ',' + str(arduino.vout[3]) + '\n')
      
      arduino.print_odom()
      before_nokori = tmp_list[len(tmp_list) - 1] # �����[�v�ɉ񂷗]���ۑ�
      log_file.flush()                            # ���O�t�@�C���ւ̏������݂𔽉f
      
      hello_str = tmp_str
      # rospy.loginfo(hello_str)
      pub.publish(tmp_str)
      
      rate.sleep()
  
  except KeyboardInterrupt:
    receive_signal();
