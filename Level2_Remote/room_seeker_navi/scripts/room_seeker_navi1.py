#!/usr/bin/env python
# coding: utf-8
#==================================================================================================#
#                                                                                                  #
# FILE : room_seeker_navi1.py                                                                      #
# Memo : The core program for RoomSeeker                                                           #
#          * Go straight and stop in front of the obstacle sensed by Lidar                         #
#                                                                                                  #
# Updated : 2021/01/16 : Startded this project                                                     #
#                                                                                                  #
#                                                                       (C) 2021 Kyohei Umemoto    #
# How to execute :                                                                                 #
#     rosrun room_seeker_navi room_seeker_navi1.py                                                 #
#                                                                                                  #
# Argument Discription :                                                                           #
#     none                                                                                         #
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
import sys            # To use exit() and to get command line arguments
import datetime       # To get datetime
import threading      # For multi thread programming
from math import *    # For mathematical operation
import signal         # To receive signal
import numpy as np    # For matrix calculation

import select
import tty
import termios

import rospy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

#==================================================================================================#
# Global variables                                                                                 #
#==================================================================================================#
# File settings -----------------------------------------------------------------------------------#
path_to_log = '/home/kyohei/catkin_ws/src/room_seeker/room_seeker_navi/scripts/log/'
path_to_data = '/home/kyohei/catkin_ws/src/room_seeker/room_seeker_navi/scripts/data/'
preffix = 'RoomSeekerData'    # Prefix of the data and log file
suffix = ''                   # Suffix of the data and log file

# ROS publish variables ---------------------------------------------------------------------------#
## Message variables
odom = Odometry()
br = tf2_ros.TransformBroadcaster()

## Frame ID
odom_header_frame_id = "odom"
odom_child_frame_id = "base_footprint"

## The others
pub10cnt = 0
SAMPLING_FREQUENCY = 50
old_settings = termios.tcgetattr(sys.stdin)
control_on = 0

#==================================================================================================#
# odomCallBack(odom)                                                                               #
#==================================================================================================#
def odomCallBack(odom):
  test = odom.pose.pose.position.x
#  print('x = ' + str(odom.pose.pose.position.x))

#==================================================================================================#
# scanCallBack(odom)                                                                               #
#==================================================================================================#
def scanCallBack(scan):
  global control_on
  if(control_on == 1):
    min_range = 9999
    for i in range(150, 211):
      if(scan.ranges[i] <= min_range):
        min_range = scan.ranges[i]
    
    if min_range <= 0.5:
      control_on = 0
    
    print 'scan_150-210 : ',
    for i in range(150, 211):
      print '{:=2.2f} '.format(scan.ranges[i]),
    print '\n'
  
#==================================================================================================#
# command_cmd_vel()                                                                                 #
#==================================================================================================#
def command_cmd_vel():
  global control_on
  
  if control_on == 1:
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    print 'twist published'
  else:
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

  cmd_vel_pub.publish(twist)

#==================================================================================================#
# Signal Receive Function                                                                          #
#==================================================================================================#
def receive_signal(signum, stack):
  print "Received signal :", signum
  currentDate = datetime.datetime.today()
  log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  data_file.close()
  log_file.close()
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
  sys.exit(0)

#==================================================================================================#
# Keyboard input function                                                                          #
#==================================================================================================#
def isData():
  return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def key_input():
  global control_on
  print("key_input started.")
  try:
    tty.setcbreak(sys.stdin.fileno())
    while True:
      if isData():
        c = sys.stdin.read(1)
        if c == 's':
          control_on = 1
          print 'Started\n',
        elif c == 't':
          control_on = 0
          print 'Stopped\n',
        else:
          test = 0
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

#==================================================================================================#
# Main Function                                                                                    #
#==================================================================================================#
if __name__ == '__main__':
  # Initialize
  signal.signal(signal.SIGINT, receive_signal)  # Signal setting
  
  # Create the thread to get keyboard input
  th_key = threading.Thread(target=key_input)
  th_key.setDaemon(True)
  th_key.start()
  
  # Open log file
  currentDate = datetime.datetime.today() # Get current date
  log_file_name = unicode(path_to_log + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
  log_file = open(log_file_name, 'w')
  log_file.write(str(currentDate) + " : Started to logging." + "\n")
  
  # Open data file
  data_file_name = unicode(path_to_data + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
  data_file = open(data_file_name, 'w')
  data_file.write("time,test\n")
  currentDate = datetime.datetime.today()
  log_file.write(str(currentDate) + " : New file opened successfully. File name = " + data_file_name + "\n")
  
  # ROS settings ----------------------------------------------------------------------------------#
  rospy.init_node('navi1_node', anonymous=True)             # Initialize node
  rate = rospy.Rate(SAMPLING_FREQUENCY)                     # Sampling frequency [Hz]

  # Publisher settings
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)
  
  # Subscriber settings
  odom_sub = rospy.Subscriber('odom', Odometry, odomCallBack)
  scan_sub = rospy.Subscriber('scan', LaserScan, scanCallBack)
  
  currentDate = datetime.datetime.today()
  log_file.write(str(currentDate) + " : ROS Settings finished.\n")
  
  # Start periodic process
  try:
    while True:
      start_date = datetime.datetime.today()

      # Sensing process ---------------------------------------------------------------------------#
      
      # Publish information
      
      # Control process ---------------------------------------------------------------------------#
      command_cmd_vel()
      
      # Data output process -----------------------------------------------------------------------#
      currentDate = datetime.datetime.today()
      data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000)
        + '0' + '\n')
          
      # For debug information ---------------------------------------------------------------------#
      stop_date = datetime.datetime.today()
      # print((stop_date - start_date).microseconds)

      rate.sleep()
  
  except KeyboardInterrupt:
    receive_signal();

