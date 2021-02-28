//================================================================================================//
//                                                                                                //
// FILE : room_seeker_core.cpp                                                                    //
// Memo : The core program for RoomSeeker                                                         //
//          * Control the all motors of mecanum wheels                                            //
//          * Calculate the odometry                                                              //
//          * Collect sensor data (IR sensor, Ultra sonic sensor, IMU, PS2 controller)            //
//                                                                                                //
// Updated : 2021/01/16 : Startded this project based on "turtlebot3_core_v2.py"                  //
//           2021/02/28 : Main process is ported to room_seeker_level1 class                      //
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
#include <ros/ros.h>
#include "room_seeker_level1.h"

//================================================================================================//
// Main Function                                                                                  //
//================================================================================================//
int main(int argc, char **argv){
  ros::init(argc, argv, "level1_node");
  RoomSeekerLevel1 rsl1;
  int ret = rsl1.startRoomSeekerLevel1();
  ros::spin();
  
  return(ret);
}
