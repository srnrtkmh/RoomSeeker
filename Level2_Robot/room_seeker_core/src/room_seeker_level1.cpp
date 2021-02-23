//================================================================================================//
//                                                                                                //
// FILE : room_seeker_level1.cpp                                                                  //
// Memo : RoomSeeker level 1 node class                                                           //
//                                                                                                //
// Updated : 2021/02/22 Started this project based on "room_seeker_l1.py"                         //
//                                                                                                //
//                                                                       (C) 2021 Kyohei Umemoto  //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include "room_seeker_level1.h"

//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// Frame ID
char odom_header_frame_id[] = "odom";
char odom_child_frame_id[] = "base_footprint";
char joint_state_header_frame_id[] = "base_link";
char imu_frame_id[] = "imu_link";
char mag_frame_id[] = "mag_link";

//================================================================================================//
// Constructor                                                                                    //
//================================================================================================//
RoomSeekerLevel1::RoomSeekerLevel1() : dt(0.01), ros_rate(100){
  // Publisher settings
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
  sensor_state_pub = nh.advertise<room_seeker_msgs::SensorState>("sensor_state", 100);
  version_info_pub = nh.advertise<room_seeker_msgs::VersionInfo>("firmware_version", 100);
  mag_pub = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 100);
  battery_state_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 100);
  tf2_ros::TransformBroadcaster br;
  
  // Subscriber settings
  twist_sub = nh.subscribe("cmd_vel", 1000, &RoomSeekerLevel1::twistCallBack, this);
  // currentDate = datetime.datetime.today()
  // log_file.write(str(currentDate) + " : ROS Settings finished.\n")
  
  initJointStates();
}

//================================================================================================//
// Destructor                                                                                     //
//================================================================================================//
RoomSeekerLevel1::~RoomSeekerLevel1(){
  tcsetattr(fdMEGA, TCSANOW, &oldtioMEGA);
  // ioctl(fdMEGA, TCSETS, &oldtioMEGA);
  close(fdMEGA);
}

//================================================================================================//
// initMEGA()                                                                                     //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initMEGA(void){
  int baudRate = B115200;
  fdMEGA = open("/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0", O_RDWR);
  if (fdMEGA < 0) {
      printf("open error\n");
  }
  
  tcgetattr(fdMEGA, &oldtioMEGA);
  bzero(&newtioMEGA, sizeof(newtioMEGA));
  newtioMEGA.c_cflag = CS8 | CLOCAL | CREAD;
  newtioMEGA.c_iflag = IGNPAR | ICRNL;
  newtioMEGA.c_oflag = 0;
  // newtio.c_lflag = ICANON;
  cfsetispeed(&oldtioMEGA, baudRate);
  cfsetospeed(&oldtioMEGA, baudRate);
  // cfmakeraw(&oldtioMEGA);
  
  tcflush(fdMEGA, TCIFLUSH);
  tcsetattr(fdMEGA, TCSANOW, &newtioMEGA);
  // ioctl(fdMEGA, TCSETS, &oldtioMEGA);
  
  // currentDate = datetime.datetime.today()
  // log_file.write(str(currentDate) + " : Serial port opened successfully. Port name = " + rsl1.conMEGA.portstr + "\n")
}

//================================================================================================//
// initFM()                                                                                       //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initFM(void){
}

//================================================================================================//
// initRM()                                                                                       //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initRM(void){
}

//================================================================================================//
// twistCallBack                                                                                  //
//================================================================================================//
void RoomSeekerLevel1::twistCallBack(const geometry_msgs::Twist::ConstPtr& twist){
  dx_cmd[0] = twist->linear.x;
  dx_cmd[1] = twist->linear.y;
  dx_cmd[2] = twist->angular.z;
  if(control_on == 0){
    control_on = 1;
    start_rplidar_motor();
  }
  no_input_from_twist = 0;
  no_input_from_twist_cnt = 0;
}

//================================================================================================//
// start_rplidar_motor()                                                                          //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::start_rplidar_motor(void){
  // rospy.wait_for_service('start_motor')
  // try:
    // service_call = rospy.ServiceProxy('start_motor', Empty)
    // service_call()
    // print('called start_motor')
  // except rospy.ServiceException, e:
    // print ("Service call failed: %s" % e)
}

//================================================================================================//
// open_logfile()                                                                                 //
//   Arguments : none                                                                             //
//   Return    : bool - Whether log file successfully opened or not                               //
//================================================================================================//
bool RoomSeekerLevel1::open_logfile(void){
  try{
    // currentDate = datetime.datetime.today() // Get current date
    // log_file_name = unicode("/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/scripts/log/" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".log", encoding='shift-jis')
    // log_file = open(log_file_name, 'w')
    // log_file.write(str(currentDate) + " : Started to logging." + "\n")
    return(true);
  }
  catch(...){
    return(false);
  }
}

//================================================================================================//
// open_datafile()                                                                                //
//   Arguments : none                                                                             //
//   Return    : bool - Whether data file successfully opened or not                              //
//================================================================================================//
bool RoomSeekerLevel1::open_datafile(char *strCsvName){
  try{
    // strCsvName = unicode("/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/scripts/data/" + preffix + currentDate.strftime('%Y_%m_%d_%H_%M_%S') + suffix + ".csv", encoding='shift-jis')
    // data_file = open(strCsvName, 'w')
    // data_file.write("time,nMEGA,nFM,nRM,cnt_now[0],cnt_now[1],cnt_now[2],cnt_now[3]"
      // + ",omega_res[0],omega_res[1],omega_res[2],omega_res[3]"
      // + ",omega_cmd[0],omega_cmd[1],omega_cmd[2],omega_cmd[3]"
      // + ",vout[0],vout[1],vout[2],vout[3]"
      // + ",dx_res[0],dx_res[1],dx_res[2]"
      // + ",dx_cmd[0],dx_cmd[1],dx_cmd[2]"
      // + ",x_res[0],x_res[1],x_res[2],x_res_[2],int_gz_hpf"
      // + ",ir_state,ax,ay,az,gx,gy,gz,gz_hpf,mx,my,mz,temp,bat_vol,ps2_button,ps2_analogRX,ps2_analogRY,ps2_analogLX,ps2_analogLY"
      // + "\n")
    // currentDate = datetime.datetime.today()
    // log_file.write(str(currentDate) + " : New file opened successfully. File name = " + strCsvName + "\n")
    return(true);
  }
  catch(...){
    printf("open_datafile() : cannot open data file \"%s\"\n", strCsvName);
    return(false);
  }
}

//================================================================================================//
// RoomSeeker level 1 class                                                                       //
//   Index of wheelspace value : 0 - Front Right, 1 - Fron Left, 2 - Rear Right, 3 - Rear Left    //
//   Index of workspace value  : 0 - Long dir, 1 - Width dir, 2 - Yaw angle                       //
//================================================================================================//
void RoomSeekerLevel1::print_state(void){
    // print('sample_num_node = ' + str(self.sample_num_node_FM))
    // print('cnt_now         = ' + str(self.cnt_now[0]) + ', ' + str(self.cnt_now[1]) + ', ' + str(self.cnt_now[2]) + ', ' + str(self.cnt_now[3]))
    // print('omega_res_x10   = ' + str(self.omega_res_x10[0]) + ', ' + str(self.omega_res_x10[1]) + ', ' + str(self.omega_res_x10[2]) + ', ' + str(self.omega_res_x10[3]))
    // print('omega_cmd_x10   = ' + str(self.omega_cmd_x10[0]) + ', ' + str(self.omega_cmd_x10[1]) + ', ' + str(self.omega_cmd_x10[2]) + ', ' + str(self.omega_cmd_x10[3]))
    // print('vout            = ' + str(self.vout[0]) + ', ' + str(self.vout[1]) + ', ' + str(self.vout[2]) + ', ' + str(self.vout[3]))
    // print('dx_cmd_x10      = ' + str(self.dx_cmd_x10[0]) + ', ' + str(self.dx_cmd_x10[1]) + ', ' + str(self.dx_cmd_x10[2]))
}

void RoomSeekerLevel1::print_sensor(void){
    // print('sample_num_node_MEGA = ' + str(self.sample_num_node_MEGA))
    // print('ir_hex = ' + hex(self.ir_hex) + ', ir_bin = ' + bin(self.ir_hex))
    // print('ax = ' + '{:=+6}'.format(int(self.ax)) + ', ay = ' + '{:=+6}'.format(int(self.ay)) + ', az = ' + '{:=+6}'.format(int(self.az)))
    // print('gx = ' + '{:=+6}'.format(int(self.gx)) + ', gy = ' + '{:=+6}'.format(int(self.gy)) + ', gz = ' + '{:=+6}'.format(int(self.gz)))
    // print('mx = ' + '{:=+6}'.format(int(self.mx)) + ', my = ' + '{:=+6}'.format(int(self.my)) + ', mz = ' + '{:=+6}'.format(int(self.mz)) + ', temp = {:=+6}'.format(int(self.temp)))
    // print('us_dist = ' + '{:=5}'.format(int(self.us_dist[0])) + ', ' + '{:=6}'.format(int(self.us_dist[2])) + ', ' + '{:=6}'.format(int(self.us_dist[2])) + ', ' + '{:=6}'.format(int(self.us_dist[3])))
    // print('us_dist = ' + '{:=5}'.format(int(self.us_dist[4])) + ', ' + '{:=6}'.format(int(self.us_dist[5])) + ', ' + '{:=6}'.format(int(self.us_dist[6])) + ', ' + '{:=6}'.format(int(self.us_dist[7])))
    // print('us_ok = ' + hex(self.us_ok) + ', bat_vol_x100 = ' + '{:=4}'.format(int(self.bat_vol_x100)))
    // print('ps2_button = ' + hex(self.ps2_button) + ', ' + '{:=3}'.format(int(self.ps2_analogRX)) + ', ' + '{:=3}'.format(int(self.ps2_analogRY)) + ', ' + '{:=3}'.format(int(self.ps2_analogLX)) + ', ' + '{:=3}'.format(int(self.ps2_analogLY)))
    // print('interval_MEGA = ' + '{:=6}'.format(int(self.interval_MEGA)))
}

void RoomSeekerLevel1::print_wrk(void){
    // print('arduino.mode = ' + str(self.mode))
    // print('dx_cmd = {:=+4}, {:=+4}, {:=+4}'.format(self.dx_cmd[0], self.dx_cmd[1], self.dx_cmd[2]))
    // print('omega_cmd     = {:=+4}, {:=+4}, {:=+4}, {:=+4}'.format(self.omega_cmd[0], self.omega_cmd[1], self.omega_cmd[2], self.omega_cmd[3]))
    // print('omega_cmd_x10 = {:=+4}, {:=+4}, {:=+4}, {:=+4}'.format(self.omega_cmd_x10[0], self.omega_cmd_x10[1], self.omega_cmd_x10[2], self.omega_cmd_x10[3]))
}

void RoomSeekerLevel1::apply_filter(void){
    // self.mx_lpf = self.mx_lpf + self.g_mag * self.dt * (self.mx - self.mx_lpf)
    // self.my_lpf = self.my_lpf + self.g_mag * self.dt * (self.my - self.my_lpf)
    // self.gz_hpf_tmp += self.dt * self.gz_hpf
    // self.gz_hpf = self.gz - self.g_gz * self.gz_hpf_tmp
    // self.int_gz_hpf += self.dt * self.gz_hpf
}

void RoomSeekerLevel1::odometry_update_res(void){
    // omega_rad = np.array(
      // [  self.omega_res_x10[0] * self.pi / 1800.0
       // , self.omega_res_x10[1] * self.pi / 1800.0
       // , self.omega_res_x10[2] * self.pi / 1800.0
       // , self.omega_res_x10[3] * self.pi / 1800.0
      // ])
    // self.dx_res = np.dot(self.J_inv_plus, omega_rad)
    // self.x_res[0] += self.dt * (self.dx_res[0] * cos(self.x_res[2]) - self.dx_res[1] * sin(self.x_res[2]))
    // self.x_res[1] += self.dt * (self.dx_res[0] * sin(self.x_res[2]) + self.dx_res[1] * cos(self.x_res[2]))
    // self.x_res_[2] = atan2(self.my_lpf - self.my_offset, self.mx_lpf - self.mx_offset)
    // self.x_res[2] = atan2(self.my_lpf - self.my_offset, self.mx_lpf - self.mx_offset) + self.int_gz_hpf
    // self.x_res[2] += self.dt * (self.dx_res[2])
}

void RoomSeekerLevel1::odometry_update_cmd(void){ // Not in use it cannnot estimate pose enough precision
    // self.x_res2[0] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * cos(self.x_res[2]) - self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    // self.x_res2[1] += self.dt * (self.dx_cmd_x10[0] / 10.0 / 1000.0 * sin(self.x_res[2]) + self.dx_cmd_x10[1] / 10.0 / 1000.0 * sin(self.x_res[2]))
    // self.x_res2[2] += self.dt * (self.dx_cmd_x10[2] / 10.0 * self.pi / 180.0)
}

void RoomSeekerLevel1::print_odom(void){
    // print('OdomRes : {:=7.2f}, {:=7.2f}, {:=7.3f}, Battery = {:=2.2f}'.format(self.x_res[0], self.x_res[1], self.x_res[2]/3.141592*180.0, self.bat_vol))
    // print('OdomCmd : {:=7.2f}, {:=7.2f}, {:=7.3f}'.format(self.x_res2[0], self.x_res2[1], self.x_res2[2]))
}

void RoomSeekerLevel1::rate_limit_work(void){
    // for i in range(0, len(self.dx_cmd)):
      // if(self.dx_cmd[i] > self.dx_cmd_rl[i] + self.dx_rate_limit[i]):     self.dx_cmd_rl[i] += self.dx_rate_limit[i];
      // elif (self.dx_cmd[i] < self.dx_cmd_rl[i] - self.dx_rate_limit[i]):  self.dx_cmd_rl[i] -= self.dx_rate_limit[i];
      // else:                                                               self.dx_cmd_rl[i]  = self.dx_cmd[i];
}

void RoomSeekerLevel1::inv_kine(void){
    // self.omega_cmd[0] = (self.dx_cmd_rl[0] + self.dx_cmd_rl[1] + (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    // self.omega_cmd[1] = (self.dx_cmd_rl[0] - self.dx_cmd_rl[1] - (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    // self.omega_cmd[2] = (self.dx_cmd_rl[0] - self.dx_cmd_rl[1] + (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
    // self.omega_cmd[3] = (self.dx_cmd_rl[0] + self.dx_cmd_rl[1] - (self.base_width + self.base_length) / 1000.0 / 2.0 * self.dx_cmd_rl[2]) / (self.wheel_radius / 1000.0);
}

void RoomSeekerLevel1::command_motor_node(void){
    // self.omega_cmd_x10[0] = int(self.omega_cmd[0] / 3.141592 * 1800)
    // self.omega_cmd_x10[1] = int(self.omega_cmd[1] / 3.141592 * 1800)
    // self.omega_cmd_x10[2] = int(self.omega_cmd[2] / 3.141592 * 1800)
    // self.omega_cmd_x10[3] = int(self.omega_cmd[3] / 3.141592 * 1800)
    // self.conFM.write('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[0] * self.omega_cmd_x10[0])) + ',' + '{:0=+5}'.format(int(self.ws_dir[1] * self.omega_cmd_x10[1])) + ',end')
    // self.conRM.write('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[2] * self.omega_cmd_x10[2])) + ',' + '{:0=+5}'.format(int(self.ws_dir[3] * self.omega_cmd_x10[3])) + ',end')
    // print('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[0] * self.omega_cmd_x10[0])) + ',' + '{:0=+5}'.format(int(self.ws_dir[1] * self.omega_cmd_x10[1])) + ',end')
    // print('vel,{:0=3}'.format(int(self.sample_num_host)) + ',' + '{:0=+5}'.format(int(self.ws_dir[2] * self.omega_cmd_x10[2])) + ',' + '{:0=+5}'.format(int(self.ws_dir[3] * self.omega_cmd_x10[3])) + ',end')
}

//================================================================================================//
// select_control_mode()                                                                          //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::select_control_mode(void){
  if((ps2_button & PSB_SELECT) != 0){
    printf("PS2 mode reset");
    mode = MODE_TWIST;
    dx_cmd[0] = 0;
    dx_cmd[1] = 0;
    dx_cmd[2] = 0;
    no_input_from_ps2 = 1;
    no_input_from_twist = 0;
    no_input_from_twist_cnt = 0;
  }
  
  if((ps2_button & PSB_START) != 0){
    printf("PS2 mode set");
    mode = MODE_PS2;
    dx_cmd[0] = 0;
    dx_cmd[1] = 0;
    dx_cmd[2] = 0;
    no_input_from_ps2 = 0;
    no_input_from_ps2_cnt = 0;
    no_input_from_twist = 1;
  }
}

//================================================================================================//
// continue_control_ps2()                                                                         //
//   Argument : none                                                                              //
//   Return   : none                                                                              //
//================================================================================================//
void RoomSeekerLevel1::continue_control_ps2(void){
  if(control_on == 0){
    control_on = 1;
    start_rplidar_motor();
  }
  no_input_from_ps2 = 0;
  no_input_from_ps2_cnt = 0;
}

//================================================================================================//
// stop_control()                                                                                 //
//================================================================================================//
void RoomSeekerLevel1::stop_control(void){
  control_on = 0;
  write(fdFM, "ttttt", 5);
  write(fdRM, "ttttt", 5);
  // rospy.wait_for_service('stop_motor')
  // try:
    // service_call = rospy.ServiceProxy('stop_motor', Empty)
    // service_call()
    // print('called stop_motor')
  // except rospy.ServiceException, e:
    // print ("Service call failed: %s" % e)
}

//================================================================================================//
// control_command()                                                                              //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::control_command(void){
  if(mode == MODE_TWIST){
    if(no_input_from_twist == 0){
      no_input_from_twist_cnt += 1;
      if(no_input_from_twist_cnt >= 3000) no_input_from_twist = 1;
    }
  }
  
  if(mode == MODE_PS2){
    // X axis command
    if((ps2_button & PSB_PAD_UP) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[0] = 0.3;
        else                               dx_cmd[0] = 0.2;
      }else                                dx_cmd[0] = 0.1;
    }
    else if((ps2_button & PSB_PAD_DOWN) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[0] = -0.3;
        else                               dx_cmd[0] = -0.2;
      }else                                dx_cmd[0] = -0.1;
    }
    else                                   dx_cmd[0] = 0.0;
      
    // Y axis command
    if((ps2_button & PSB_PAD_LEFT) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[1] = 0.3;
        else                               dx_cmd[1] = 0.2;
      }else                                dx_cmd[1] = 0.1;
    }
    else if((ps2_button & PSB_PAD_RIGHT) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[1] = -0.3;
        else                               dx_cmd[1] = -0.2;
      }else                                dx_cmd[1] = -0.1;
    }
    else                                   dx_cmd[1] = 0.0;
  
    // Rotation command
    if((ps2_button & PSB_L1) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[2] = 135.0 / 180.0 * 3.14;
        else                               dx_cmd[2] = 90.0 / 180.0 * 3.14;
      }else                                dx_cmd[2] = 45.0 / 180.0 * 3.14;
    }
    else if((ps2_button & PSB_R1) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd[2] = -135.0 / 180.0 * 3.14;
        else                               dx_cmd[2] = -90.0 / 180.0 * 3.14;
      }else                                dx_cmd[2] = -45.0 / 180.0 * 3.14;
    }
    else                                   dx_cmd[2] = 0.0;
    
    // PS2 no input judge
    if(no_input_from_ps2 == 0){
      no_input_from_ps2_cnt += 1;
      if(no_input_from_ps2_cnt >= 3000) no_input_from_ps2 = 1;
    }
  }
  
  if(control_on == 1){
    if(no_input_from_ps2 == 1 && no_input_from_twist == 1) stop_control();
  }
}

//================================================================================================//
// data_output()                                                                                  //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::data_output(void){
  if(control_on == 1){
    // currentDate = datetime.datetime.today()
    // data_file.write(currentDate.strftime("%Y/%m/%d %H:%M:%S") + ".%03d" % (currentDate.microsecond // 1000)
      // + ',' + str(sample_num_node_MEGA) + ',' + str(sample_num_node_FM) + ',' + str(sample_num_node_RM)
      // + ',' + str(cnt_now[0]) + ',' + str(cnt_now[1]) + ',' + str(cnt_now[2]) + ',' + str(cnt_now[3])
      // + ',' + str(omega_res[0]) + ',' + str(omega_res[1]) + ',' + str(omega_res[2]) + ',' + str(omega_res[3])
      // + ',' + str(omega_cmd[0]) + ',' + str(omega_cmd[1]) + ',' + str(omega_cmd[2]) + ',' + str(omega_cmd[3])
      // + ',' + str(vout[0]) + ',' + str(vout[1]) + ',' + str(vout[2]) + ',' + str(vout[3])
      // + ',' + str(dx_res[0]) + ',' + str(dx_res[1]) + ',' + str(dx_res[2])
      // + ',' + str(dx_cmd[0]) + ',' + str(dx_cmd[1]) + ',' + str(dx_cmd[2])
      // + ',' + str(x_res[0]) + ',' + str(x_res[1]) + ',' + str(x_res[2]) + ',' + str(x_res_[2]) + ',' + str(int_gz_hpf)
      // + ',' + bin(ir_hex) + ',' + str(ax) + ',' + str(ay) + ',' + str(az)
      // + ',' + str(gx) + ',' + str(gy) + ',' + str(gz) + ',' + str(gz_hpf)
      // + ',' + str(mx) + ',' + str(my) + ',' + str(mz) + ',' + str(temp)
      // + ',' + str(bat_vol) + ',' + str(ps2_button) + ',' + str(ps2_analogRX) + ',' + str(ps2_analogRY) + ',' + str(ps2_analogLX) + ',' + str(ps2_analogLY)
      // + '\n')
  }
}

//================================================================================================//
// Send start command to rsl1 via serial port                                                  //
//   Argument con : serial console object to send start command                                   //
//   Return       : None                                                                          //
//================================================================================================//
void RoomSeekerLevel1::send_start(int fd){
  char buf[256];
  int len;
  len = read(fd, buf, sizeof(buf));       // Read buffer to empty it
  usleep(200000);                         // Wait a while to stabilize(?)
  write(fd, "s", 1);                      // 1st : Send start command 's' to serial port
  // log_file.write(str(currentDate) + " : Start command transmitted (1st try).\n")
  usleep(500000);                         // 1st : Wait 0.5sec to receive callback
  len = read(fd, buf, sizeof(buf));       // 1st : Check if the callback is sent or not
  if(len <= 10){                          // 2nd : Re-send start command if don't get callback
    write(fd, "s", 1);                    // 2nd : Send start command 's' to serial port
    // log_file.write(str(currentDate) + " : Start command transmitted (2nd try). Previous tmp_str : " + tmp_str + "\n")
    usleep(500000);                       // 2nd : Wait 0.5sec to receive callback
    len = read(fd, buf, sizeof(buf));     // 2nd : Check if the callback is sent or not
    if(len <= 10){                        // 2nd : Re-send start command if don't get callback
      write(fd, "s", 1);                  // 3rd : Send start command 's' to serial port
      // log_file.write(str(currentDate) + " : Start command transmitted (3rd try). Previous tmp_str : " + tmp_str + "\n")
      usleep(500000);                     // 3rd : Wait 0.5sec to receive callback (3rd)
      len = read(fd, buf, sizeof(buf));   // 3rd : Send start command 's' to serial port
      if(len <= 10){                      // 3rd : Re-send start command if don't get callback
        printf("xxxx : %d did not transmit call back : ", fd);
        write(fd, "t", 1);                // 3rd : Send stop command for make sure
      }
    }
  }
  len = read(fd, buf, sizeof(buf));       // Empty the receive buffer
  // log_file.write(str(currentDate) + " : Start command echo back received.\n")
}

//================================================================================================//
// is_int()                                                                                       //
//   Arguments : char *str - The pointer to char string to check integer or not                   //
//               int n     - How many characters to check                                         //
//               int *data - The variable to store transformed integer                            //
//   Return    : bool - Whether *str is int or not  int - true, other - false                     //
//================================================================================================//
bool is_int(char *str, int n, int *data){
  int tmp[10], tmp_cnt = 0;
  int sign = 1, sign_fix = 0;
  for(int i = 0; i < n; i++){
    if(0x30 <= *(str + i) && *(str + i) <= 0x39){
      tmp[tmp_cnt] = *(str + i) - 0x30;
      tmp_cnt++;
      if(tmp_cnt >= 10) return(false);
      sign_fix = 1;
    }
    else if(*(str + i) == '+'){
      if(sign_fix == 0){
        sign = 1;
        sign_fix = 1;
      }else{
        return(false);
      }
    }
    else if(*(str + i) == '-'){
      if(sign_fix == 0){
        sign = -1;
        sign_fix = 1;
      }else{
        return(false);
      }
    }
    else if(*(str + i) == ' ');
    else if(*(str + i) == 0) break;
    else return(false);
  }
  
  if(tmp_cnt == 0) return(false);
  
  int data_tmp = 0;
  for(int i = 0; i < tmp_cnt; i++){
    data_tmp += pow(10, tmp_cnt - i - 1) * tmp[i];
  }
  *data = sign * data_tmp;
  
  return(true);
}

//================================================================================================//
// is_hex()                                                                                       //
//   Arguments : char *str - The pointer to char string to check integer or not                   //
//               int n     - How many characters to check                                         //
//               int *data - The variable to store transformed integer                            //
//   Return    : bool - Whether *str is int or not  int - true, other - false                     //
//================================================================================================//
bool is_hex(char *str, int n, int *data){
}

//================================================================================================//
// interpretMEGA                                                                                  //
//================================================================================================//
void RoomSeekerLevel1::interpretMEGA(char *split_data){
  int data_tmp[28];
  int int_or_hex[28] = {INT, HEX, INT, INT, INT, INT, INT, INT, INT, INT, 
                        INT, INT, INT, INT, INT, INT, INT, INT, INT, INT, 
                        HEX, INT, INT, INT, INT, INT, INT, INT};
  int sample_err = 0;
  
  for(int i = 0; i < 28; i++){
    if(int_or_hex[i] == INT)
      if(!is_int((split_data + 32 * i), 32, (data_tmp + i))) sample_err = i;
    else
      if(!is_hex((split_data + 32 * i), 32, (data_tmp + i))) sample_err = i;
  }
  
  // If error not occured, put the data to class
  if(sample_err == 0){
    sample_num_node_MEGA = data_tmp[0];
    ir_hex = data_tmp[1];
    ax = data_tmp[2];
    ay = data_tmp[3];
    az = data_tmp[4];
    gx = gc * data_tmp[5];
    gy = gc * data_tmp[6];
    gz = gc * data_tmp[7];
    mx = data_tmp[8];
    my = data_tmp[9];
    mz = data_tmp[10];
    temp = data_tmp[11];
    us_dist[0] = data_tmp[12];
    us_dist[1] = data_tmp[13];
    us_dist[2] = data_tmp[14];
    us_dist[3] = data_tmp[15];
    us_dist[4] = data_tmp[16];
    us_dist[5] = data_tmp[17];
    us_dist[6] = data_tmp[18];
    us_dist[7] = data_tmp[19];
    us_ok = data_tmp[20];
    bat_vol_x100 = data_tmp[21];
    bat_vol = data_tmp[21] / 100.0;
    ps2_button = data_tmp[22];
    ps2_analogRX = data_tmp[23];
    ps2_analogRY = data_tmp[24];
    ps2_analogLX = data_tmp[25];
    ps2_analogLY = data_tmp[26];
    interval_MEGA = data_tmp[27];
  }else{
    printf("Error in read MEGA: %d\n", sample_err);
  }
}

//================================================================================================//
// readMEGA()                                                                                     //
//================================================================================================//
void RoomSeekerLevel1::readMEGA(void){
  static char buf[2048];
  static char split_data[28][32];
  static int chptr = 0;
  static int strptr = 0;
  static int err_flag = 0;
  int i, j, len;
  
  // Read the received data from serial device
  len = read(fdMEGA, buf, sizeof(buf));
  for(i = 0; i < len; i++){
    // Split the string at ','
    if(buf[i] == ','){
      chptr = 0;
      if(strptr < 27){
        strptr++;
      }
      else{
        err_flag = 1;
        strptr = 27;
        printf("Error in strptr : the last str = %s", split_data[strptr]);
      }
    }
    // Transform char strings to data at '\n'
    else if(buf[i] == '\n'){
      if(strptr == 27 && err_flag == 0){
        printf("Got data is below\n");
        for(j = 0; j < 28; j++) printf("%s\n", split_data[j]);
        printf("\n");
        interpretMEGA(&(split_data[0][0]));
      }else{
        printf("Error at received new line code, strptr = %d, chptr = %d\n", strptr, chptr);
        for(j = 0; j < 28; j++) printf("%s\n", split_data[j]);
      }
      memset(split_data, 0, sizeof(split_data));  // Clear related variables
      chptr = 0;
      strptr = 0;
      err_flag = 0;
    }
    // Store the other character at the end of the word
    else{
      if(chptr < 31){
        split_data[strptr][chptr] = buf[i];
        chptr++;
      }
      else{
        printf("Error in chptr : %c\n", buf[i]);
        chptr = 31;
      }
    }
  }
}

//================================================================================================//
// readFM()                                                                                       //
//================================================================================================//
void RoomSeekerLevel1::readFM(void){
    // Initialize variables
    int err_flag = 0;
    int sample_num_node_FM = 0;
    int cnt_now_0 = 0;
    int cnt_now_1 = 0;
    int omega_res_x10_0 = 0;
    int omega_res_x10_1 = 0;
    int omega_cmd_x10_0 = 0;
    int omega_cmd_x10_1 = 0;
    int vout_0 = 0;
    int vout_1 = 0;
    int interval = 0;
    
    // Read serial data & Put it to class
    // tmp_str = self.conFM.read(self.conFM.inWaiting())       // Read serial data
    // tmp_list = (self.before_nokoriFM + tmp_str).split('\n') // Connect previous rest sample and this sample -> split by new line char
    // if len(tmp_list) >= 2:
      // for i in range(0, len(tmp_list) - 1):
        // tmp_val_list = tmp_list[i].split(',')
        // sample_err = 0
        // if len(tmp_val_list) == 10:
          // if is_int(tmp_val_list[0]):  sample_num_node_FM = int(tmp_val_list[0])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[1]):  cnt_now_0 = int(tmp_val_list[1])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[2]):  cnt_now_1 = int(tmp_val_list[2])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[3]):  omega_res_x10_0 = int(tmp_val_list[3])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[4]):  omega_res_x10_1 = int(tmp_val_list[4])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[5]):  omega_cmd_x10_0 = int(tmp_val_list[5])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[6]):  omega_cmd_x10_1 = int(tmp_val_list[6])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[7]):  vout_0 = int(tmp_val_list[7])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[8]):  vout_1 = int(tmp_val_list[8])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[9]):  interval = int(tmp_val_list[9])
          // else:                        sample_err = 1
        // else:
          // err_flag = 1
        
        // If error not occured, put the data to class
        // if sample_err == 0:
          // self.sample_num_node_FM = sample_num_node_FM
          // self.cnt_now[0] = self.ws_dir[0] * cnt_now_0
          // self.cnt_now[1] = self.ws_dir[1] * cnt_now_1
          // self.omega_res_x10[0] = self.ws_dir[0] * omega_res_x10_0
          // self.omega_res[0] = self.omega_res_x10[0] / 1800.0 * 3.141592
          // self.omega_res_x10[1] = self.ws_dir[1] * omega_res_x10_1
          // self.omega_res[1] = self.omega_res_x10[1] / 1800.0 * 3.141592
          // self.omega_cmd_x10[0] = self.ws_dir[0] * omega_cmd_x10_0
          // self.omega_cmd_x10[1] = self.ws_dir[1] * omega_cmd_x10_1
          // self.vout[0] = self.ws_dir[0] * vout_0
          // self.vout[1] = self.ws_dir[1] * vout_1
          // self.interval[0] = interval
        // else:
          // print("Error in read FM: ")
          // print(tmp_list[i])
    
    // self.before_nokoriFM = tmp_list[len(tmp_list) - 1] // 次ループに回す余りを保存
}

//================================================================================================//
// readRM()                                                                                       //
//================================================================================================//
void RoomSeekerLevel1::readRM(void){
    // Initialize variables
    int err_flag = 0;
    int sample_num_node_RM = 0;
    int cnt_now_2 = 0;
    int cnt_now_3 = 0;
    int omega_res_x10_2 = 0;
    int omega_res_x10_3 = 0;
    int omega_cmd_x10_2 = 0;
    int omega_cmd_x10_3 = 0;
    int vout_2 = 0;
    int vout_3 = 0;
    int interval = 0;
    
    // Read serial data & Put it to class
    // tmp_str = self.conRM.read(self.conRM.inWaiting())       // Read serial data
    // tmp_list = (self.before_nokoriRM + tmp_str).split('\n') // Connect previous rest sample and this sample -> split by new line char
    // if len(tmp_list) >= 2:
      // for i in range(0, len(tmp_list) - 1):
        // tmp_val_list = tmp_list[i].split(',')
        // sample_err = 0
        // if len(tmp_val_list) == 10:
          // if is_int(tmp_val_list[0]):  sample_num_node_RM = int(tmp_val_list[0])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[1]):  cnt_now_2 = int(tmp_val_list[1])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[2]):  cnt_now_3 = int(tmp_val_list[2])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[3]):  omega_res_x10_2 = int(tmp_val_list[3])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[4]):  omega_res_x10_3 = int(tmp_val_list[4])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[5]):  omega_cmd_x10_2 = int(tmp_val_list[5])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[6]):  omega_cmd_x10_3 = int(tmp_val_list[6])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[7]):  vout_2 = int(tmp_val_list[7])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[8]):  vout_3 = int(tmp_val_list[8])
          // else:                        sample_err = 1
          // if is_int(tmp_val_list[9]):  interval = int(tmp_val_list[9])
          // else:                        sample_err = 1
        // else:
          // err_flag = 1
        
        // If error not occured, put the data to class
        // if sample_err == 0:
          // self.sample_num_node_RM = sample_num_node_RM
          // self.cnt_now[2] = self.ws_dir[2] * cnt_now_2
          // self.cnt_now[3] = self.ws_dir[3] * cnt_now_3
          // self.omega_res_x10[2] = self.ws_dir[2] * omega_res_x10_2
          // self.omega_res[2] = self.omega_res_x10[2] / 1800.0 * 3.141592
          // self.omega_res_x10[3] = self.ws_dir[3] * omega_res_x10_3
          // self.omega_res[3] = self.omega_res_x10[3] / 1800.0 * 3.141592
          // self.omega_cmd_x10[2] = self.ws_dir[2] * omega_cmd_x10_2
          // self.omega_cmd_x10[3] = self.ws_dir[3] * omega_cmd_x10_3
          // self.vout[2] = self.ws_dir[2] * vout_2
          // self.vout[3] = self.ws_dir[3] * vout_3
          // self.interval[0] = interval
        // else:
          // print("Error in read RM : ")
          // print(tmp_list[i])
    
    // self.before_nokoriRM = tmp_list[len(tmp_list) - 1] // 次ループに回す余りを保存
}

//================================================================================================//
// Initialize Joint State                                                                         //
//================================================================================================//
void RoomSeekerLevel1::initJointStates(void){
  // static char *joint_states_name[] = {(char*)"front_right_wheel_joint", (char*)"front_left_wheel_joint", (char *)"rear_right_wheel_joint", (char *)"rear_left_wheel_joint"};
  joint_states.header.frame_id = joint_state_header_frame_id;
  // joint_states.name            = joint_states_name;
  joint_states.name.resize(4);
  joint_states.name[0] = "front_right_wheel_joint";
  joint_states.name[1] = "front_left_wheel_joint";
  joint_states.name[2] = "rear_right_wheel_joint";
  joint_states.name[3] = "rear_left_wheel_joint";
  joint_states.position.resize(4);
  joint_states.velocity.resize(4);
  for(int i = 0; i < 4; i++){
    joint_states.position[i] = 0.0;
    joint_states.velocity[i] = 0.0;
  }
}

//================================================================================================//
// Publish msgs (odometry, joint states, tf)                                                      //
//================================================================================================//
void RoomSeekerLevel1::publishDriveInformation(void){
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_res[2]);
  
  // odometry
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;
  odom.pose.pose.position.x = x_res[0];
  odom.pose.pose.position.y = x_res[1];
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q[0];
  odom.pose.pose.orientation.y = q[1];
  odom.pose.pose.orientation.z = q[2];
  odom.pose.pose.orientation.w = q[3];
  odom.twist.twist.linear.x  = dx_res[0];
  odom.twist.twist.linear.y  = dx_res[1];
  odom.twist.twist.linear.z  = 0.0;
  odom.twist.twist.angular.z = dx_res[2];
  odom_pub.publish(odom);
  
  // odometry tf
  geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.header.frame_id = odom.header.frame_id;
  t.child_frame_id = odom.child_frame_id;
  t.transform.translation.x = x_res[0];
  t.transform.translation.y = x_res[1];
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q[0];
  t.transform.rotation.y = q[1];
  t.transform.rotation.z = q[2];
  t.transform.rotation.w = q[3];
  br.sendTransform(t);
  
  // joint states
  joint_states.header.stamp = ros::Time::now();
  joint_states.position[0] = cnt_now[0] / 22.0 / 30.0;
  joint_states.position[1] = cnt_now[1] / 22.0 / 30.0;
  joint_states.position[2] = cnt_now[2] / 22.0 / 30.0;
  joint_states.position[3] = cnt_now[3] / 22.0 / 30.0;
  joint_states.velocity[0]  = omega_res[0];
  joint_states.velocity[1]  = omega_res[1];
  joint_states.velocity[2]  = omega_res[2];
  joint_states.velocity[3]  = omega_res[3];
  joint_states_pub.publish(joint_states);
}

//================================================================================================//
// Publish msgs (IMU data: angular velocity, linear acceleration, orientation)                    //
//   Future Works                                                                                 //
//================================================================================================//
void RoomSeekerLevel1::publishImuMsg(void){
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp          = ros::Time::now();
  imu_msg.header.frame_id       = imu_frame_id;
  imu_msg.orientation.x         = mx - mx_offset;
  imu_msg.orientation.y         = my - my_offset;
  imu_msg.orientation.z         = mz - mz_offset;
  imu_msg.orientation.w         = sqrt(pow(mx, 2) + pow(my, 2) + pow(mz, 2));
  imu_msg.angular_velocity.x    = gx;
  imu_msg.angular_velocity.y    = gy;
  imu_msg.angular_velocity.z    = gz;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_pub.publish(imu_msg);
}

//================================================================================================//
// Publish msgs (Magnetic data)                                                                   //
//================================================================================================//
void RoomSeekerLevel1::publishMagMsg(void){
  sensor_msgs::MagneticField mag_msg;
  mag_msg.header.stamp    = ros::Time::now();
  mag_msg.header.frame_id = mag_frame_id;
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;
  mag_pub.publish(mag_msg);
}

//================================================================================================//
// Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)                       //
//================================================================================================//
void RoomSeekerLevel1::publishSensorStateMsg(void){
  room_seeker_msgs::SensorState sensor_state_msg;
  sensor_state_msg.header.stamp = ros::Time::now();
  sensor_state_msg.battery = bat_vol;
  sensor_state_msg.left_encoder = cnt_now[0];
  sensor_state_msg.right_encoder = cnt_now[1];
  sensor_state_msg.bumper = 0;                        // ToDo
  sensor_state_msg.cliff = 0.0;                       // ToDo
  // sensor_state_msg.sonar = sensors.getSonarData();    // ToDo
  sensor_state_msg.illumination = 0.0;                // ToDo
  sensor_state_msg.led = 0;                           // ToDo
  sensor_state_msg.button = 0;                        // ToDo
  sensor_state_msg.torque = false;                    // ToDo
  sensor_state_pub.publish(sensor_state_msg);         // ToDo
}

//================================================================================================//
// Publish msgs (version info)                                                                    //
//================================================================================================//
void RoomSeekerLevel1::publishVersionInfoMsg(void){
  room_seeker_msgs::VersionInfo version_info_msg;
  version_info_msg.hardware = HARDWARE_VER;
  version_info_msg.software = SOFTWARE_VER;
  version_info_msg.firmware = FIRMWARE_VER;
  version_info_pub.publish(version_info_msg);
}

//================================================================================================//
// Publish msgs (battery_state)                                                                   //
//================================================================================================//
void RoomSeekerLevel1::publishBatteryStateMsg(void){
  sensor_msgs::BatteryState battery_state_msg;
  battery_state_msg.header.stamp = ros::Time::now();
  battery_state_msg.design_capacity = BAT_CAPA;
  battery_state_msg.voltage = bat_vol;
  battery_state_msg.percentage = (battery_state_msg.voltage - MIN_VOLT) / (MAX_VOLT - MIN_VOLT);
  if(battery_state == 0) battery_state_msg.present = false;
  else                        battery_state_msg.present = true;
  battery_state_pub.publish(battery_state_msg);
}
