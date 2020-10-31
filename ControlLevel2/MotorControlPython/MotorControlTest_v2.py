#!/usr/bin/env python
# coding: cp932
#==================================================================================================#
#                                                                                                  #
# FILE : MotorControlTest_v2.py                                                                    #
# Memo : Motor control test program in constant velocity command                                   #
#                                                                                                  #
# Updated : 2020/09/20 Started this project based on "MotorControlTest_v1.py"                      #
#                                                                                                  #
#                                                                       (C) 2020 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     python MotorControlTest_v2.py [Serial device name]                                           #
#         or                                                                                       #
#     nohup python MotorControlTest_v2.py [Serial device name] > [tty????.txt] &                   #
#                                                                                                  #
# Argument Discription :                                                                           #
#     Serial device name : Example "/dev/ttyACM0", "/dev/ttyUSB1"                                  #
#                                                                                                  #
# How to end                                                                                       #
#     kill -INT [PID]                                                                              #
#     * PID is written [Devicename]_PID.txt                                                        #
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

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
# SignalTraceInterface�p�ϐ�
data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
before_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

serialDeviceName = "/dev/ttyACM0" # �V���A���f�o�C�X�̐ݒ�
serialChNum = 32                  # I/O�`�����l�����̐ݒ�
baudrate = 230400                 # �{�[���[�g�ݒ�
parentPath = ""                   # �e�f�B���N�g���̐ݒ�
dataPath = ""                     # �f�[�^�p�f�B���N�g���̐ݒ�
labelFileName = ""                # ���x���ݒ�t�@�C����
data_file = []                    # CSV�o�͗p�t�@�C���I�u�W�F�N�g
strCsvName = ""                   # �t�@�C�����p�ϐ�
file_header = ""                  # CSV�t�@�C���̃w�b�_�[
preffix = "MotorControlTest_"     # �t�@�C�����v���t�B�b�N�X
suffix = ""                       # �t�@�C�����T�t�B�b�N�X
dataPath = "./"                   # �f�[�^�p�f�B���N�g���̐ݒ�
parentPath = "../"                # �e�f�B���N�g���̐ݒ�

before_nokori = ""
one_hour_cnt = 0
dt = 100000                       # �T���v�����O�^�C���̐ݒ�(�f�t�H���g��100ms�A�����ɂĐݒ肷��̂Ŏ�������)
plus_cnt = 0                      # �T���v���^�C���ƒ�������s���Ԃ̍����v���X�����ɃI�[�o�[���Ă���񐔂𐔂���J�E���^
minus_cnt = 0                     # �T���v���^�C���ƒ�������s���Ԃ̍����}�C�i�X�����ɃI�[�o�[���Ă���񐔂𐔂���J�E���^

dateCurrentMonth = 0              # �����Ƀ��O�t�@�C����؂�ւ��邽�߂̕ϐ�
dateBeforeMonth = 0               # �����Ƀ��O�t�@�C����؂�ւ��邽�߂̕ϐ�

current_sample_number1 = 999      # �ŏ���1���R�[�h�ڂ̓T���v���i���o�["0"������͂��Ȃ̂ŏ�������l��"999"
current_sample_number2 = 999      # �ŏ���1���R�[�h�ڂ̓T���v���i���o�["0"������͂��Ȃ̂ŏ�������l��"999"
before_sample_number1 = 999       # �ǂ��������������Ă��܂��̂ŏ����l�͓K��
before_sample_number2 = 999       # �ǂ��������������Ă��܂��̂ŏ����l�͓K��

# �V�O�i����M�����p�ϐ�
pid = 0
pid_file = []

#==================================================================================================#
# Signal Receive Function                                                                          #
#==================================================================================================#
def receive_signal(signum, stack):
  print "Received signal :", signum
  currentDate = datetime.datetime.today()  # today()���\�b�h�Ō��ݓ��t�E������datetime�^�f�[�^�̕ϐ����擾
  log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  con.write("t")
  time.sleep(0.01)
  con.write("t")
  time.sleep(0.01)
  con.write("t")
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
    print(test)

#==================================================================================================#
# Main Function                                                                                    #
#==================================================================================================#
if __name__ == '__main__':
  # ��������
  param = sys.argv
  if len(param) >= 2:
    serialDeviceName = param[1]    # �V���A���f�o�C�X�ւ̃p�X
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
  log_file = open(log_file_name, 'w')      # ���O�t�@�C����V�K�쐬
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # �f�[�^�t�@�C�����J��
  strCsvName = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  data_file = open(strCsvName, 'w')                # �t�@�C�����쐬
  log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
  
  # �V���A���ʐM�|�[�g���Z�b�g�A�b�v
  try:
    con = serial.Serial(serialDeviceName, baudrate, timeout=0.2, rtscts=False, dsrdtr=False)  # �|�[�g�̃I�[�v��(timeout�̐ݒ�͓K���Artscts��dsrdtr��True�ɂ��Ă����Ȃ���ArduinoMicro�Ƃ͒ʐM�ł��Ȃ�)
    time.sleep(1.0)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = con.read(con.inWaiting())    # �ǂݎ��o�b�t�@����ɂ��Ă���
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + con.portstr + "\n")
  
  # �ŏ���1�񂾂��X�^�[�g�R�}���h�𑗐M
  tmp_str = con.read(con.inWaiting())    # �ǂݎ��o�b�t�@����ɂ��Ă���(�O�ɋL�q����1�񂾂����Ƌ�ɂȂ�Ȃ���������(?))
  time.sleep(1.0)                         # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
  con.write("s")                          # �X�^�[�g�R�}���h�𑗐M(1���)
  log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")      # �X�^�[�g�R�}���h�𑗂����|���O
  time.sleep(1.0)                         # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
  tmp_str = con.read()                    # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
  if len(tmp_str) >= 10:
    con.write("s")                        # �X�^�[�g�R�}���h�𑗐M(2���)
    log_file.write(str(currentDate) + " : Start command transmitted (2nd try).\n")    # �X�^�[�g�R�}���h�𑗂����|���O
    time.sleep(1.0)                       # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
    tmp_str = con.read()                  # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
    if len(tmp_str) >= 10:
      con.write("s")                      # �X�^�[�g�R�}���h�𑗐M(3���)
      log_file.write(str(currentDate) + " : Start command transmitted (3rd try).\n")  # �X�^�[�g�R�}���h�𑗂����|���O
      time.sleep(1.0)                     # �X�^�[�g�R�}���h�̃R�[���o�b�N�܂�1�b�ԑ҂�
      tmp_str = con.read()                # �X�^�[�g�R�}���h�̃R�[���o�b�N���m�F
      if len(tmp_str) >= 10:
        print "Serial Device did not transmit call back : " + tmp_str
        con.write("t")                    # �O�̂��߃X�g�b�v�R�}���h�𑗂��Ă���
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
      if ctrl_cnt >= 100:
        ctrl_cnt = 0
        if ctrl_state == 0:
          ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
          ctrl_state = 1
        elif ctrl_state == 1:
          ctrl_cmd = "wrk,000,+0000,+1000,+0000,end"
          ctrl_state = 2
        elif ctrl_state == 2:
          ctrl_cmd = "wrk,000,-1000,+0000,+0000,end"
          ctrl_state = 3
        elif ctrl_state == 3:
          ctrl_cmd = "wrk,000,+0000,-1000,+0000,end"
          ctrl_state = 4
        elif ctrl_state == 4:
          ctrl_cmd = "wrk,000,+1000,+1000,+1000,end"
          ctrl_state = 5
        elif ctrl_state == 5:
          ctrl_cmd = "wrk,000,-1000,+1000,-1000,end"
          ctrl_state = 6
        elif ctrl_state == 6:
          ctrl_cmd = "wrk,000,-1000,-1000,+1000,end"
          ctrl_state = 7
        elif ctrl_state == 7:
          ctrl_cmd = "wrk,000,+1000,-1000,-1000,end"
          ctrl_state = 0
      ctrl_cnt = ctrl_cnt + 1
      
      con.write(ctrl_cmd)
      
      # ���݂̓������擾
      currentDate = datetime.datetime.today()
      
      # �ǂݎ��f�[�^���t�@�C���ɏ�������
      tmp_str = con.read(con.inWaiting())
      tmp_list = []                                     # �z��̒��g���N���A���Ă���(�K�v���ǂ����͖��m�ł͂Ȃ�)
      tmp_list = (before_nokori + tmp_str).split('\n')  # �O��T���v���c��ƍ���T���v�����Ȃ����킹�ĉ��s�ŕ���
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          # data_file.write(dateCurrentSample.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (dateCurrentSample.microsecond // 1000) + ',' + tmp_list[i] + '\n')
          data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000) + ',' + tmp_list[i] + '\n')
      before_nokori = tmp_list[len(tmp_list) - 1]
       
      log_file.flush()          # ���O�t�@�C���ւ̏������݂𔽉f
      beforeDate = currentDate  # �O�񏈗��������X�V
      time.sleep(0.01)           # ��莞�ԃX���[�v
  
  except KeyboardInterrupt:
    con.write("t")
    time.sleep(0.01)
    con.write("t")
    time.sleep(0.01)
    con.write("t")
    data_file.close()
    currentDate = datetime.datetime.today()                    # ���݂̓������擾
    log_file.write(str(currentDate) + " : Exit by the keyboard input" + "\n")  # �L�[�{�[�h���͂ŕ������Ƃ����O�ɋL��
    log_file.close()
    sys.exit(0)
