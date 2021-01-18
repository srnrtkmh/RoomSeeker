#!/usr/bin/env python
# coding: utf-8
#==================================================================================================#
#                                                                                                  #
# FILE : room_seeker_core_v1.py                                                                    #
# Memo : The core program for RoomSeeker                                                           #
#        *前後輪4輪分の制御                                                                        #
#        *オドメトリ計算                                                                           #
#        *センサデータ収集(IRセンサ、超音波センサ、IMU、PS2)                                       #
#                                                                                                  #
# Updated : 2021/01/16 : Startded this project based on "turtlebot3_core_v2.py"                    #
#           2021/01/18 : Added inverse kinematics program                                          #
#                                                                                                  #
#                                                                       (C) 2021 Kyohei Umemoto    #
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

import room_seeker_l1

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
serDevNameMEGA = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0"      # Upper Right USB port : Arduino Mega
serDevNameFM = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0"  # Upper Left USB port  : Front Motor Controller
serDevNameRM = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0"  # Lower Left USB port  : Rear Motor Controller
baudrate = 230400                   # ボーレート設定
data_file = []                      # CSV出力用ファイルオブジェクト
strCsvName = ""                     # ファイル名用変数
preffix = "MotorControl_with_Lidar" # ファイル名プレフィックス
suffix = ""                         # ファイル名サフィックス
before_nokori = ""
sample_err = 0
sample_err_num = 0

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
  arduino.conMEGA.write("t")
  arduino.conFM.write("t")
  arduino.conRM.write("t")
  # time.sleep(0.01)
  # arduino.conMEGA.write("t")
  # arduino.conFM.write("t")
  # arduino.conRM.write("t")
  # time.sleep(0.01)
  # arduino.conMEGA.write("t")
  # arduino.conFM.write("t")
  # arduino.conRM.write("t")
  # data_file.close()
  log_file.close()
  sys.exit(0)

#==================================================================================================#
# #
#==================================================================================================#
def rospy_shutdown():
  # sys.exit(0)
  # print "Rospy shutdown"
  # currentDate = datetime.datetime.today()  # today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  # log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  # arduino.conMEGA.write("t")
  # arduino.conFM.write("t")
  # arduino.conRM.write("t")
  # time.sleep(0.01)
  # arduino.conMEGA.write("t")
  # arduino.conFM.write("t")
  # arduino.conRM.write("t")
  # time.sleep(0.01)
  # arduino.conMEGA.write("t")
  # arduino.conFM.write("t")
  # arduino.conRM.write("t")
  # data_file.close()
  # log_file.close()
  # sys.exit(0)
  rospy.sleep(1)
  sys.exit(0)


#==================================================================================================#
# Keyboard input function                                                                          #
#==================================================================================================#
def key_input():
  print("key_input started.")
  while True:
    time.sleep(0.01)
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
      arduino.conMEGA.write(test)
      print(test)

#==================================================================================================#
# Send start command to arduino via serial port                                                    #
#==================================================================================================#
def send_start(con):
  tmp_str = con.read(con.inWaiting())     # Read buffer to empty it
  time.sleep(0.2)                         # Wait a while to stabilize(?)
  con.write("s")                          # 1st : Send start command 's' to serial port
  log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")
  time.sleep(0.5)                         # 1st : Wait 0.5sec to receive callback
  tmp_str = con.read(con.inWaiting())     # 1st : Check if the callback is sent or not
  if len(tmp_str) <= 10:                  # 2nd : Re-send start command if don't get callback
    con.write("s")                        # 2nd : Send start command 's' to serial port
    log_file.write(str(currentDate) + " : Start command transmitted (2nd try). Previous tmp_str : " + tmp_str + "\n")
    time.sleep(0.5)                       # 2nd : Wait 0.5sec to receive callback
    tmp_str = con.read(con.inWaiting())   # 2nd : Check if the callback is sent or not
    if len(tmp_str) <= 10:                # 2nd : Re-send start command if don't get callback
      con.write("s")                      # 3rd : Send start command 's' to serial port
      log_file.write(str(currentDate) + " : Start command transmitted (3rd try). Previous tmp_str : " + tmp_str + "\n")
      time.sleep(0.5)                     # 3rd : Wait 0.5sec to receive callback (3rd)
      tmp_str = con.read(con.inWaiting()) # 3rd : Send start command 's' to serial port
      if len(tmp_str) <= 10:              # 3rd : Re-send start command if don't get callback
        print "Serial Device did not transmit call back : " + tmp_str
        con.write("t")                    # 3rd : Send stop command for make sure
        sys.exit(1)
  log_file.write(str(currentDate) + " : Start command echo back received.\n")

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
  
  # ログファイルを開く
  log_file_name = unicode("/home/kyohei/catkin_ws/src/room_seeker/room_seeker_test/scripts/" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
  log_file = open(log_file_name, 'w')               # ログファイルを新規作成
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # データファイルを開く
  # strCsvName = unicode("/home/kyohei/catkin_ws/src/turtlebot3/turtlebot3_bringup/scripts/" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  # data_file = open(strCsvName, 'w')                 # ファイルを作成
  # data_file.write("time,n,cnt_now[0],cnt_now[1],cnt_now[2],cnt_now[3]"
    # + ",omega_res_x10[0],omega_res_x10[1],omega_res_x10[2],omega_res_x10[3]"
    # + ",omega_cmd_x10[0],omega_cmd_x10[1],omega_cmd_x10[2],omega_cmd_x10[3]"
    # + ",vout[0],vout[1],vout[2],vout[3]"
    # + ",ir_state,ax,ay,az,gx,gy,gz,mx,my,mz,tc,bat_vol_x100,ps2_ctrl"
    # + "\n")
  # log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
  
  # ROS settings ----------------------------------------------------------------------------------#
  rospy.init_node('level1_node', anonymous=True)            # Initialize node
  rospy.on_shutdown(rospy_shutdown)
  rate = rospy.Rate(20)                                     # Sampling time [Hz]

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
  
  # Setup serial communication with ArduinoMEGA ---------------------------------------------------#
  try: # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    arduino.conMEGA = serial.Serial(serDevNameMEGA, 115200, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(0.5)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = arduino.conMEGA.read(arduino.conMEGA.inWaiting())     # 読み取りバッファを空にしておく
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + arduino.conMEGA.portstr + "\n")
  
  # Setup serial communication with ArduinoFM -----------------------------------------------------#
  try: # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    arduino.conFM = serial.Serial(serDevNameFM, 115200, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(0.5)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = arduino.conFM.read(arduino.conFM.inWaiting())     # 読み取りバッファを空にしておく
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + arduino.conFM.portstr + "\n")
  
  # Setup serial communication with ArduinoRM -----------------------------------------------------#
  try: # ポートのオープン(timeoutの設定は適当、rtsctsとdsrdtrはTrueにしておかないとArduinoMicroとは通信できない)
    arduino.conRM = serial.Serial(serDevNameRM, 115200, timeout=0, rtscts=False, dsrdtr=False)
    time.sleep(0.5)
  except serial.SerialException as e:
    print "SerialException : " + str(e)
    sys.exit(1)
  tmp_str = arduino.conRM.read(arduino.conRM.inWaiting())     # 読み取りバッファを空にしておく
  log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + arduino.conRM.portstr + "\n")
  
  # Send start command to arduino controller
  send_start(arduino.conMEGA) # Send start command to arduino MEGA
  send_start(arduino.conFM)   # Send start command to arduino Front Motor
  send_start(arduino.conRM)   # Send start command to arduino Rear Motor
  
  xx = 0
  yy = 0
  dir = 0
  # 周期処理開始
  try:
    while True:
      start_date = datetime.datetime.today()
      
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
      
      arduino.conFM.write('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(arduino.ws_dir[0] * xx)) + ',' + '{:0=+5}'.format(int(arduino.ws_dir[1] * yy)) + ',end')
      arduino.conRM.write('vel,{:0=3}'.format(int(arduino.sample_num_host)) + ',' + '{:0=+5}'.format(int(arduino.ws_dir[2] * xx)) + ',' + '{:0=+5}'.format(int(arduino.ws_dir[3] * yy)) + ',end')
      
      # 現在の日時を取得
      currentDate = datetime.datetime.today()
      
      # 読み取りデータをファイルに書き込み
      # arduino.readMEGA()
      arduino.readFM()
      arduino.readRM()
      
      # log_file.flush()                            # ログファイルへの書き込みを反映
      
      # Publish information
      # publishDriveInformation()
      # publishImuMsg()
      # publishMagMsg()
      # pub10cnt += 1
      # if pub10cnt >= 10:
        # pub10cnt = 0
        # publishBatteryStateMsg()
        # publishSensorStateMsg()
        # publishVersionInfoMsg()
      
      # arduino.print_odom()
      # arduino.print_state()
      
      stop_date = datetime.datetime.today()
      print((stop_date - start_date).microseconds)
      
      rate.sleep()
  
  except KeyboardInterrupt:
    receive_signal();

