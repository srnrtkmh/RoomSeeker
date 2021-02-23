//================================================================================================//
//                                                                                                //
// FILE : room_seeker_core.cpp                                                                    //
// Memo : The core program for RoomSeeker                                                         //
//          * Control the all motors of mecanum wheels                                            //
//          * Calculate the odometry                                                              //
//          * Collect sensor data (IR sensor, Ultra sonic sensor, IMU, PS2 controller)            //
//                                                                                                //
// Updated : 2021/01/16 : Startded this project based on "turtlebot3_core_v2.py"                  //
//                                                                                                //
//                                                                      (C) 2021 Kyohei Umemoto   //
// How to execute :                                                                               //
//     rosrun room_seeker_core room_seeker_core_v1.py                                             //
//                                                                                                //
// Argument Discription :                                                                         //
//     none                                                                                       //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <stdio.h>
#include <ros/ros.h>

#include "room_seeker_level1.h"

//================================================================================================//
// Global variables                                                                               //
//================================================================================================//
// Serial settings -------------------------------------------------------------------------------//
char serDevNameMEGA[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0";        // Upper Right USB port : Arduino Mega
int serBaudrateMEGA = 115200;
char serDevNameFM[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0";    // Upper Left USB port  : Front Motor Controller
// serDevNameFM[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3.1:1.0-port0";  // Upper Left USB port  : Front Motor Controller
int serBaudrateFM = 115200;
char serDevNameRM[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0";    // Lower Left USB port  : Rear Motor Controller
// serDevNameRM[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3.2:1.0-port0";  // Lower Left USB port  : Rear Motor Controller
int serBaudrateRM = 115200;

// Data file settings ----------------------------------------------------------------------------//
char preffix[] = "RoomSeekerData"; // Prefix of the file name
char suffix[] = "";                // Suffix of the file name
char logFileName[] = "/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/scripts/data/";
char dataFileName[] = "/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/scripts/data/";

int pub10cnt = 0;

//================================================================================================//
// Signal Receive Function                                                                        //
//================================================================================================//
// void receive_signal(signum, stack){
  // print "Received signal :", signum
  // currentDate = datetime.datetime.today()  // today()メソッドで現在日付・時刻のdatetime型データの変数を取得
  // log_file.write(str(currentDate) + " : Received signal " + str(signum) + ". End this process/\n")
  // rsl1.conMEGA.write("tttt")
  // rsl1.conFM.write("tttt")
  // rsl1.conRM.write("tttt")
  // data_file.close()
  // log_file.close()
  // sys.exit(0)
// }

//================================================================================================//
// Keyboard input function                                                                        //
//================================================================================================//
void key_input(void){
  // print("key_input started.")
  // while True:
    // time.sleep(0.01)
    // test = raw_input()
    
    // if test == 'w':
      // rsl1.dx_cmd_x10[0] += 1000
    // elif test == 'x':
      // rsl1.dx_cmd_x10[0] -= 1000
    // elif test == 'a':
      // rsl1.dx_cmd_x10[1] += 1000
    // elif test == 'd':
      // rsl1.dx_cmd_x10[1] -= 1000
    // elif test == 's':
      // rsl1.dx_cmd_x10[0] = 0
      // rsl1.dx_cmd_x10[1] = 0
      // rsl1.dx_cmd_x10[2] = 0
    // else:
      // rsl1.conMEGA.write(test)
      // print(test)
}

//================================================================================================//
// Main Function                                                                                  //
//================================================================================================//
int main(int argc, char **argv){
  // Initialize
  // signal.signal(signal.SIGINT, receive_signal)  // Signal setting
  
  // Create the thread to get keyboard input
  // th_key = threading.Thread(target=key_input)
  // th_key.setDaemon(True)
  // th_key.start()
  
  // ROS settings --------------------------------------------------------------------------------//
  ros::init(argc, argv, "level1_node");
  
  RoomSeekerLevel1 rsl1;                            // Robot control variables (current)
  if(!rsl1.open_logfile()) return(1);               // Open log file
  if(!rsl1.open_datafile(dataFileName)) return(1);  // Open data file
  ros::Rate loop_rate(rsl1.ros_rate);               // Sampling rate [Hz]
  
  // Setup serial communication ------------------------------------------------------------------//
  rsl1.initMEGA();
  rsl1.initFM();
  rsl1.initRM();
  
  // Send start command to rsl1 controller
  rsl1.send_start(rsl1.fdMEGA);  // Send start command to rsl1 MEGA
  // rsl1.send_start(rsl1.fdFM);    // Send start command to rsl1 Front Motor
  // rsl1.send_start(rsl1.fdRM);    // Send start command to rsl1 Rear Motor
  // rsl1.stop_control();            // Stop control until input command
  
  // Start periodic process
  ros::Time t_start = ros::Time::now();
  ros::Time t_finish = ros::Time::now();
  ros::Duration t_period(0.0);
  ros::Duration t_interval(0.0);
  while(ros::ok()){
    t_start = ros::Time::now();
    
    // Sensing process -------------------------------------------------------------------------//
    rsl1.readMEGA();
    rsl1.readFM();
    rsl1.readRM();
    rsl1.apply_filter();
    rsl1.odometry_update_res();
    
    // Publish information -----------------------------------------------------------------------//
    rsl1.publishDriveInformation();
    rsl1.publishImuMsg();
    rsl1.publishMagMsg();
    pub10cnt += 1;
    if(pub10cnt >= 10){
      pub10cnt = 0;
      rsl1.publishBatteryStateMsg();
      rsl1.publishSensorStateMsg();
      rsl1.publishVersionInfoMsg();
    }
    
    // Select control mode ---------------------------------------------------------------------//
    rsl1.select_control_mode();
    
    // Control process -------------------------------------------------------------------------//
    rsl1.control_command();
    rsl1.rate_limit_work();
    rsl1.inv_kine();
    if(rsl1.control_on == 1) rsl1.command_motor_node();
    
    // Data output process ---------------------------------------------------------------------//
    rsl1.data_output();
    
    // For debug information -------------------------------------------------------------------//
    if(rsl1.control_on == 1){
      // rsl1.print_sensor();
      // rsl1.print_wrk();
      // rsl1.print_state();
      // rsl1.print_odom();
    }
    
    t_finish = ros::Time::now();
    t_interval = t_finish - t_start;
    ROS_INFO("period = %lf, interval = %lf", t_period.toSec(), t_interval.toSec());
    ros::spinOnce();
    loop_rate.sleep();
    t_period = ros::Time::now() - t_start;
  }
}
