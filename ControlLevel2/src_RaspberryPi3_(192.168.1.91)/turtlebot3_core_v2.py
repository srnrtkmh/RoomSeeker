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
#           2021/01/02 Renamed "turtlebot3_core_v2.py"                                             #
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

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu, MagneticField, BatteryState
from turtlebot3_msgs.msg import SensorState, VersionInfo

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
serDevName = "/dev/ttyACM0" # シリアルデバイスの設定
baudrate = 230400                 # ボーレート設定
data_file = []                    # CSV出力用ファイルオブジェクト
strCsvName = ""                   # ファイル名用変数
preffix = "MotorControl_with_Lidar" # ファイル名プレフィックス
suffix = ""                       # ファイル名サフィックス
before_nokori = ""

# ROS publish variables ---------------------------------------------------------------------------#
# Message variables
odom = Odometry()
br = tf.TransformBroadcaster()
joint_states = JointState()
imu_msg = Imu()
mag_msg = MagneticField()
battery_state_msg = BatteryState()
battery_state = 0
sensor_state_msg = SensorState()
version_info_msg = VersionInfo()

# Frame ID
odom_header_frame_id = "/odom"
odom_child_frame_id = "/base_footprint"
joint_state_header_frame_id = "/base_link"
imu_frame_id = "/imu_link"
mag_frame_id = "/mag_link"

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
# Arduino motor control class                                                                      #
#==================================================================================================#
class arduino_motor_control():
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
    self.con = 0                                        # Serial console connected to arduino motor controller
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
    self.x_res[2] += self.dt * (self.dx_res_x10[2] / 10.0)
  
  def odometry_update_cmd(self):
    self.x_res2[0] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * cos(self.x_res[2]) - self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[1] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * sin(self.x_res[2]) + self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    self.x_res2[2] += self.dt * (self.dx_cmd_x10[2] / 10.0 * self.pi / 180.0)
  
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
# twistCallBack                                                                                    #
#==================================================================================================#
def twistCallBack(twist):
  arduino.dx_cmd_x10[0] = twist.linear.x * 1000.0 * 10.0;
  arduino.dx_cmd_x10[2] = twist.angular.z / 3.141592 * 180.0 * 10.0;
  # rospy.loginfo("x:%f, y:%f, z:%f, a:%f, b:%f, c:%f", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)

#==================================================================================================#
# Publish msgs (IMU data: angular velocity, linear acceleration, orientation)                      #
#   Future Works                                                                                   #
#==================================================================================================#
def publishImuMsg():
  imu_msg.header.stamp          = rospy.Time.now()
  imu_msg.header.frame_id       = imu_frame_id
  imu_msg.orientation.x         = 0.0
  imu_msg.orientation.y         = 0.0
  imu_msg.orientation.z         = 1.0
  imu_msg.orientation.w         = 0.0
  imu_msg.angular_velocity.x    = 0.0
  imu_msg.angular_velocity.y    = 0.0
  imu_msg.angular_velocity.z    = 0.0
  imu_msg.linear_acceleration.x = 0.0
  imu_msg.linear_acceleration.y = 0.0
  imu_msg.linear_acceleration.z = 0.0
  imu_pub.publish(imu_msg);

#==================================================================================================#
# Publish msgs (Magnetic data)                                                                     #
#==================================================================================================#
def publishMagMsg():
  mag_msg.header.stamp    = rospy.Time.now()
  mag_msg.header.frame_id = mag_frame_id;
  mag_msg.magnetic_field.x = 1.0;
  mag_msg.magnetic_field.y = 2.0;
  mag_msg.magnetic_field.z = 3.0;
  mag_pub.publish(mag_msg);

#==================================================================================================#
# Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)                         #
#==================================================================================================#
def publishSensorStateMsg():
  sensor_state_msg.header.stamp = rospy.Time.now()
  sensor_state_msg.battery = 0.0                      # ToDo
  sensor_state_msg.left_encoder = arduino.cnt_now[0]  
  sensor_state_msg.right_encoder = arduino.cnt_now[1]
  sensor_state_msg.bumper = 0                         # ToDo
  sensor_state_msg.cliff = 0.0;                       # ToDo
  # sensor_state_msg.sonar = sensors.getSonarData();    # ToDo
  sensor_state_msg.illumination = 0.0                 # ToDo
  sensor_state_msg.led = 0                            # ToDo
  sensor_state_msg.button = 0                         # ToDo
  sensor_state_msg.torque = False                     # ToDo
  sensor_state_pub.publish(sensor_state_msg)          # ToDo

#==================================================================================================#
# Publish msgs (version info)                                                                      #
#==================================================================================================#
def publishVersionInfoMsg():
  version_info_msg.hardware = HARDWARE_VER
  version_info_msg.software = SOFTWARE_VER
  version_info_msg.firmware = FIRMWARE_VER
  version_info_pub.publish(version_info_msg)

#==================================================================================================#
# Publish msgs (battery_state)                                                                     #
#==================================================================================================#
def publishBatteryStateMsg():
  battery_state_msg.header.stamp = rospy.Time.now()
  battery_state_msg.design_capacity = BAT_CAPA
  battery_state_msg.voltage = 7.4                 # ToDo
  battery_state_msg.percentage = battery_state_msg.voltage / MAX_VOLT
  if battery_state == 0:
    battery_state_msg.present = False
  else:
    battery_state_msg.present = true  
  battery_state_pub.publish(battery_state_msg);

#==================================================================================================#
# Publish msgs (odometry, joint states, tf)                                                        #
#==================================================================================================#
def publishDriveInformation():
  # odometry
  odom.header.stamp = rospy.Time.now()
  odom.header.frame_id = odom_header_frame_id
  odom.child_frame_id  = odom_child_frame_id
  odom.pose.pose.position.x = arduino.x_res[0]
  odom.pose.pose.position.y = arduino.x_res[1]
  odom.pose.pose.position.z = 0.0
  q_tmp = tf.transformations.quaternion_from_euler(0.0, 0.0, arduino.x_res[2])
  odom.pose.pose.orientation.x = q_tmp[0]
  odom.pose.pose.orientation.y = q_tmp[1]
  odom.pose.pose.orientation.z = q_tmp[2]
  odom.pose.pose.orientation.w = q_tmp[3]
  odom.twist.twist.linear.x  = arduino.dx_res[0]
  odom.twist.twist.linear.y  = arduino.dx_res[1]
  odom.twist.twist.linear.z  = 0.0
  odom.twist.twist.angular.z = arduino.dx_res[2]
  odom_pub.publish(odom)

  # odometry tf
  br.sendTransform((arduino.x_res[0], arduino.x_res[1], 0.0),
                   tf.transformations.quaternion_from_euler(0.0, 0.0, arduino.x_res[2]),
                   rospy.Time.now(),
                   odom_child_frame_id,
                   "world")

  # joint states
  joint_states.header.stamp = rospy.Time.now();
  joint_states.position = arduino.cnt_now / 22.0 / 30.0
  joint_states.velocity = arduino.omega_res_x10 / 10.0
  joint_states_pub.publish(joint_states)

#==================================================================================================#
# Initialize Joint State                                                                           #
#==================================================================================================#
def initJointStates():
  joint_states.header.frame_id = joint_state_header_frame_id
  joint_states.name            = ['Wheel_front_right_joint', 'Wheel_front_left_joint', 'Wheel_rear_right_joint', 'Wheel_rear_left_joint']
  joint_states.position        = [0.0, 0.0, 0.0, 0.0]
  joint_states.velocity        = [0.0, 0.0, 0.0, 0.0]
  joint_states.effort          = [0.0, 0.0, 0.0, 0.0]

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
  currentDate = datetime.datetime.today()  # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
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
  # 引数処理
  param = sys.argv
  # if len(param) >= 2:
    # serDevName = param[1]    # シリアルデバイスへのパス
  # else:
  #   print "Invalid argument. Valid command : \"" + param[0] + " [Serial device name]\""
  #   sys.exit(1)
    
  print(param)
  
  # 初期設定
  currentDate = datetime.datetime.today() # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  
  # Robot control variables
  arduino_pre = arduino_motor_control()
  arduino = arduino_motor_control()
  
  # シグナルハンドル関係の設定
  signal.signal(signal.SIGINT, receive_signal)                # SIGINTを受け取ったら指定関数を呼び出すように設定
  
  # キーボード入力用スレッド作成
  th_key = threading.Thread(target=key_input)
  th_key.setDaemon(True)
  th_key.start()
  
  # ログファイルを開く
  log_file_name = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
  log_file = open(log_file_name, 'w')     # ログファイルを新規作成
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # データファイルを開く
  strCsvName = unicode("./" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  data_file = open(strCsvName, 'w')       # ファイルを作成
  data_file.write("time,n,cnt_now[0],cnt_now[1],cnt_now[2],cnt_now[3]"
    + ",omega_res_x10[0],omega_res_x10[1],omega_res_x10[2],omega_res_x10[3]"
    + ",omega_cmd_x10[0],omega_cmd_x10[1],omega_cmd_x10[2],omega_cmd_x10[3]"
    + ",vout[0],vout[1],vout[2],vout[3]"
    + ",dx_res_x10[0],dx_res_x10[1],dx_res_x10[2]"
    + ",dx_cmd_x10[0],dx_cmd_x10[1],dx_cmd_x10[2]"
    + ",x_res[0],x_res[1],x_res[2]"
    + ",x_res2[0],x_res2[1],x_res2[2]"
    + "\n")
  log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
  
  # ROS settings ----------------------------------------------------------------------------------#
  rospy.init_node('level1_node', anonymous=True)            # Initialize node
  rate = rospy.Rate(10)                                     # Sampling time [Hz]

  # Publisher settings
  initJointStates()
  odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
  joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=50);
  sensor_state_pub = rospy.Publisher('sensor_state', SensorState, queue_size=50)
  version_info_pub = rospy.Publisher('firmware_version', VersionInfo, queue_size=50)
  imu_pub = rospy.Publisher('imu', Imu, queue_size=50)
  mag_pub = rospy.Publisher('magnetic_field', MagneticField, queue_size=50)
  battery_state_pub = rospy.Publisher('battery_state', BatteryState, queue_size=50)
  
  # Subscriber settings
  sub = rospy.Subscriber('cmd_vel', Twist, twistCallBack)
  log_file.write(str(currentDate) + " : ROS Settings finished.\n")
  
  # Setup serial communication with Arduino -------------------------------------------------------#
  try:
    # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    arduino.con = serial.Serial(serDevName, baudrate, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(1.0)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # 読み取りバッファを空にしておく
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + arduino.con.portstr + "\n")
  
  # Send start command to arduino
  tmp_str = arduino.con.read(arduino.con.inWaiting())     # 読み取りバッファを空にしておく(前に記述した1回だけだと空にならない時がある(?))
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  arduino.con.write("s")                  # スタートコマンドを送信(1回目)
  log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")      # スタートコマンドを送った旨ログ
  time.sleep(1.0)                         # スタートコマンドのコールバックまで1秒間待ち
  tmp_str = arduino.con.read()            # スタートコマンドのコールバックを確認
  if len(tmp_str) >= 10:
    arduino.con.write("s")                # スタートコマンドを送信(2回目)
    log_file.write(str(currentDate) + " : Start command transmitted (2nd try).\n")    # スタートコマンドを送った旨ログ
    time.sleep(1.0)                       # スタートコマンドのコールバックまで1秒間待ち
    tmp_str = arduino.con.read()          # スタートコマンドのコールバックを確認
    if len(tmp_str) >= 10:
      arduino.con.write("s")              # スタートコマンドを送信(3回目)
      log_file.write(str(currentDate) + " : Start command transmitted (3rd try).\n")  # スタートコマンドを送った旨ログ
      time.sleep(1.0)                     # スタートコマンドのコールバックまで1秒間待ち
      tmp_str = arduino.con.read()        # スタートコマンドのコールバックを確認
      if len(tmp_str) >= 10:
        print "Serial Device did not transmit call back : " + tmp_str
        arduino.con.write("t")                    # 念のためストップコマンドを送っておく
        sys.exit(1)
  log_file.write(str(currentDate) + " : Start command echo back received.\n")
  
  # 制御変数初期化
  ctrl_cnt = 0
  ctrl_state = 0
  ctrl_cmd = "wrk,000,+1000,+0000,+0000,end"
  
  # 周期処理開始
  try:
    while True:
      # 制御処理
      arduino.cmd_wrk(arduino.dx_cmd_x10[0], arduino.dx_cmd_x10[1], arduino.dx_cmd_x10[2])
      
      # 現在の日時を取得
      currentDate = datetime.datetime.today()
      
      # 読み取りデータをファイルに書き込み
      tmp_str = arduino.con.read(arduino.con.inWaiting()) # シリアルのバッファー内のデータを取得
      tmp_list = (before_nokori + tmp_str).split('\n')    # 前回サンプル残りと今回サンプルをつなぎ合わせて改行で分割
      if len(tmp_list) >= 2:
        for i in range(0, len(tmp_list) - 1):
          tmp_val_list = tmp_list[i].split(',')
          arduino_pre = arduino
          sample_err = 0
          if len(tmp_val_list) == 18:
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
            + ',' + str(arduino.vout[0]) + ',' + str(arduino.vout[1]) + ',' + str(arduino.vout[2]) + ',' + str(arduino.vout[3])
            + ',' + str(arduino.dx_res_x10[0]) + ',' + str(arduino.dx_res_x10[1]) + ',' + str(arduino.dx_res_x10[2])
            + ',' + str(arduino.dx_cmd_x10[0]) + ',' + str(arduino.dx_cmd_x10[1]) + ',' + str(arduino.dx_cmd_x10[2])
            + ',' + str(arduino.x_res[0]) + ',' + str(arduino.x_res[1]) + ',' + str(arduino.x_res[2])
            + ',' + str(arduino.x_res2[0]) + ',' + str(arduino.x_res2[1]) + ',' + str(arduino.x_res2[2])
            + ',' + str(arduino.ir_hex)
            + '\n')
      
      before_nokori = tmp_list[len(tmp_list) - 1] # 次ループに回す余りを保存
      log_file.flush()                            # ログファイルへの書き込みを反映
      
      # Publish information
      publishDriveInformation()
      publishImuMsg()
      publishMagMsg()
      pub10cnt += 1
      if pub10cnt >= 10:
        pub10cnt = 0
        publishBatteryStateMsg()
        publishSensorStateMsg()
        publishVersionInfoMsg()
      
      # arduino.print_odom()
      # arduino.print_state()
      rate.sleep()
  
  except KeyboardInterrupt:
    receive_signal();

