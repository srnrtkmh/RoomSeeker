//================================================================================================//
//                                                                                                //
// FILE : room_seeker_level1.cpp                                                                  //
// Memo : RoomSeeker level 1 node class                                                           //
//                                                                                                //
// Updated : 2021/02/22 Started this project based on "room_seeker_l1.py"                         //
//           2021/02/27 Bug fixes. Serial communication program is ported                         //
//           2021/02/28 Bug fixes, Serial communication program fixed.                            //
//                      Data output and logging is ported                                         //
//                                                                                                //
//                                                                       (C) 2021 Kyohei Umemoto  //
// MEMO :                                                                                         //
//   Index of wheelspace value : 0 - Front Right, 1 - Fron Left, 2 - Rear Right, 3 - Rear Left    //
//   Index of workspace value  : 0 - Long dir, 1 - Width dir, 2 - Yaw angle                       //
//                                                                                                //
//================================================================================================//

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
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
RoomSeekerLevel1::RoomSeekerLevel1() : 
mode(0),sample_num_node_MEGA(0), sample_num_node_FM(0), sample_num_node_RM(0),sample_num_host(0),sample_num_ul(999),sample_num_ll(0),
wheel_radius(45.0), base_width(215.0), base_length(162.0), x_ul(10.0), x_ll(10.0),
serBaudrateMEGA(B115200), serBaudrateFM(B115200), serBaudrateRM(B115200),
dt(0.01), ros_rate(100), gc(-0.06097 / 180.0 * 3.141592)
{
  // Inverse jacobian settings
  J_inv.resize(4, 3);
  J_inv << 1.0/(wheel_radius/1000.0), 1.0/(wheel_radius/1000.0),  ((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0),
           1.0/(wheel_radius/1000.0), -1.0/(wheel_radius/1000.0), -((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0),
           1.0/(wheel_radius/1000.0), -1.0/(wheel_radius/1000.0), ((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0),
           1.0/(wheel_radius/1000.0), 1.0/(wheel_radius/1000.0),  -((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0);
  
  // Pseudo inverse jacobian settings
  J_inv_plus.resize(3, 4);
  J_inv_plus = (J_inv.transpose() * J_inv).inverse() * J_inv.transpose();
  
  ws_dir[0] = -1.0; ws_dir[1] = 1.0; ws_dir[2] = -1.0; ws_dir[3] = 1.0;
  dx_rate_limit << 0.010, 0.010, 0.050;
  
  cnt_now[0] = 0; cnt_now[1] = 0; cnt_now[2] = 0; cnt_now[3] = 0;
  omega_res << 0.0, 0.0, 0.0, 0.0;
  omega_res_x10[0] = 0; omega_res_x10[1] = 0; omega_res_x10[2] = 0; omega_res_x10[3] = 0;
  omega_cmd << 0.0, 0.0, 0.0, 0.0;
  omega_cmd_x10[0] = 0; omega_cmd_x10[1] = 0; omega_cmd_x10[2] = 0; omega_cmd_x10[3] = 0;
  vout[0] = 0; vout[1] = 0; vout[2] = 0; vout[3] = 0;
  x_res << 0.0, 0.0, 0.0;
  x_res_ << 0.0, 0.0, 0.0;
  x_res2 << 0.0, 0.0, 0.0;
  dx_res << 0.0, 0.0, 0.0;
  x_cmd[0] = 0.0; x_cmd[1] = 0.0; x_cmd[2] = 0.0;
  dx_cmd << 0.0, 0.0, 0.0;
  dx_cmd_rl << 0.0, 0.0, 0.0;
  
  initJointStates();
}

//================================================================================================//
// Destructor                                                                                     //
//================================================================================================//
RoomSeekerLevel1::~RoomSeekerLevel1(){
  write(fdMEGA, "ttttt", 5);
  tcsetattr(fdMEGA, TCSANOW, &oldtioMEGA);
  close(fdMEGA);
  
  write(fdFM, "ttttt", 5);
  tcsetattr(fdFM, TCSANOW, &oldtioFM);
  close(fdFM);
  
  write(fdRM, "ttttt", 5);
  tcsetattr(fdRM, TCSANOW, &oldtioRM);
  close(fdRM);
}

//================================================================================================//
// startRoomSeekerLevel1()                                                                        //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
int RoomSeekerLevel1::startRoomSeekerLevel1(void){
  static int pub10cnt = 0;
  
  // Set parameters ------------------------------------------------------------------------------//
  ros::NodeHandle pnh("~");
  pnh.getParam("logFileDir", logFileDir);
  pnh.getParam("dataFileDir", dataFileDir);
  pnh.getParam("serDevNameMEGA", serDevNameMEGA);
  pnh.getParam("serDevNameFM", serDevNameFM);
  pnh.getParam("serDevNameRM", serDevNameRM);
  
  // Data file settings --------------------------------------------------------------------------//
  sprintf(preffix, "RoomSeeker"); // Prefix of the file name
  sprintf(suffix, "");            // Suffix of the file name
  if(logFileDir == "")  logFileDir = "/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/log/";
  if(dataFileDir == "") dataFileDir = "/home/kyohei/catkin_ws/src/room_seeker/room_seeker_core/data/";
  if(!open_logfile()) return(1);  // Open log file
  if(!open_datafile()) return(1); // Open data file
  
  // ROS settings --------------------------------------------------------------------------------//
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
  
  // Service client settings
  client_start_rplidar = nh.serviceClient<std_srvs::Empty>("start_motor");
  client_stop_rplidar = nh.serviceClient<std_srvs::Empty>("stop_motor");
  write_log(fp_log, "ROS Settings finished.\n");
  
  // Setup serial communication ------------------------------------------------------------------//
  if(serDevNameMEGA == "") serDevNameMEGA = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0";        // Upper Right USB port : Arduino Mega
  if(serDevNameFM == "")   serDevNameFM   = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0";  // Upper Left USB port  : Front Motor Controller
  if(serDevNameRM == "")   serDevNameRM   = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0";  // Lower Left USB port  : Rear Motor Controller
  initMEGA();
  initFM();
  initRM();
  
  // Send start command to rsl1 controller
  send_start(fdMEGA);  // Send start command to rsl1 MEGA
  send_start(fdFM);    // Send start command to rsl1 Front Motor
  send_start(fdRM);    // Send start command to rsl1 Rear Motor
  stop_control();      // Stop control until input command
  write_log(fp_log, "Serial communication setup compleded\n");
  
  // Periodic process
  ros::Rate loop_rate(ros_rate);          // Sampling rate [Hz]
  ros::Time t_start = ros::Time::now();
  ros::Time t_finish = ros::Time::now();
  ros::Duration t_period(0.0);
  ros::Duration t_interval(0.0);
  write_log(fp_log, "Periodic control loop started\n");
  while(ros::ok()){
    t_start = ros::Time::now();
    
    // Sensing process -------------------------------------------------------------------------//
    readMEGA();
    readFM();
    readRM();
    apply_filter();
    odometry_update_res();
    
    // Publish information -----------------------------------------------------------------------//
    publishDriveInformation();
    publishImuMsg();
    publishMagMsg();
    pub10cnt += 1;
    if(pub10cnt >= 10){
      pub10cnt = 0;
      publishBatteryStateMsg();
      publishSensorStateMsg();
      publishVersionInfoMsg();
    }
    
    // Control process -------------------------------------------------------------------------//
    select_control_mode();
    control_command();
    rate_limit_work();
    inv_kine();
    if(control_on == 1) command_motor_node();
    
    // Data output process ---------------------------------------------------------------------//
    if(control_on == 1) write_data();
    
    // For debug information -------------------------------------------------------------------//
    t_finish = ros::Time::now();
    t_interval = t_finish - t_start;
    if(control_on == 1 && pub10cnt == 0){
      // print_sensor();
      // print_wrk();
      // print_state();
      print_odom();
      ROS_INFO("period = %lf, interval = %lf", t_period.toSec(), t_interval.toSec());
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    t_period = ros::Time::now() - t_start;
  }
}

//================================================================================================//
// initMEGA()                                                                                     //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initMEGA(void){
  // char str[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0";
  fdMEGA = open(serDevNameMEGA.c_str(), O_RDWR);
  if (fdMEGA < 0) printf("Error opening MEGA\n");
  
  tcgetattr(fdMEGA, &oldtioMEGA);
  bzero(&newtioMEGA, sizeof(newtioMEGA));
  newtioMEGA.c_cflag = CS8 | CLOCAL | CREAD;
  newtioMEGA.c_iflag = IGNPAR | ICRNL;
  newtioMEGA.c_oflag = 0;
  cfsetispeed(&newtioMEGA, serBaudrateMEGA);
  cfsetospeed(&newtioMEGA, serBaudrateMEGA);
  
  tcflush(fdMEGA, TCIFLUSH);
  tcsetattr(fdMEGA, TCSANOW, &newtioMEGA);
  
  char tmp_str[256];
  sprintf(tmp_str, "Serial port opened successfully. Port name = %s\n", serDevNameMEGA.c_str());
  write_log(fp_log, tmp_str);
  printf("Opening serial port %s -> %d succeeded\n", serDevNameMEGA.c_str(), fdMEGA);
}

//================================================================================================//
// initFM()                                                                                       //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initFM(void){
  int baudRate = B115200;
  // char str[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0";
  fdFM = open(serDevNameFM.c_str(), O_RDWR);
  if (fdFM < 0) printf("Error opening FM\n");
  
  tcgetattr(fdFM, &oldtioFM);
  bzero(&newtioFM, sizeof(newtioFM));
  newtioFM.c_iflag &= ~(         BRKINT |          PARMRK | INPCK | ISTRIP | INLCR |                 IUCLC | IXON | IXANY | IXOFF | IMAXBEL);
  newtioFM.c_iflag |=  (IGNBRK |          IGNPAR |                                   IGNCR | ICRNL                                         );
  newtioFM.c_oflag = 0;
  newtioFM.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  newtioFM.c_cflag &= ~(CSIZE | PARENB);
  newtioFM.c_cflag |= CS8 | CLOCAL | CREAD;
  cfsetispeed(&newtioFM, baudRate);
  cfsetospeed(&newtioFM, baudRate);
  
  tcflush(fdFM, TCIFLUSH);
  tcsetattr(fdFM, TCSANOW, &newtioFM);
  
  char tmp_str[256];
  sprintf(tmp_str, "Serial port opened successfully. Port name = %s\n", serDevNameFM.c_str());
  write_log(fp_log, tmp_str);
  printf("Opening serial port %s -> %d succeeded\n", serDevNameFM.c_str(), fdFM);
}

//================================================================================================//
// initRM()                                                                                       //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::initRM(void){
  int baudRate = B115200;
  // char str[] = "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0";
  fdRM = open(serDevNameRM.c_str(), O_RDWR);
  if (fdRM < 0) printf("Error opening RM\n");
  
  tcgetattr(fdRM, &oldtioRM);
  bzero(&newtioRM, sizeof(newtioRM));
  newtioRM.c_iflag &= ~(         BRKINT |          PARMRK | INPCK | ISTRIP | INLCR |                 IUCLC | IXON | IXANY | IXOFF | IMAXBEL);
  newtioRM.c_iflag |=  (IGNBRK |          IGNPAR |                                   IGNCR | ICRNL                                         );
  newtioRM.c_oflag =  0;
  newtioRM.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  newtioRM.c_cflag &= ~(CSIZE | PARENB);
  newtioRM.c_cflag |= CS8 | CLOCAL | CREAD;
  cfsetispeed(&newtioRM, baudRate);
  cfsetospeed(&newtioRM, baudRate);
  
  tcflush(fdRM, TCIOFLUSH);
  tcsetattr(fdRM, TCSANOW, &newtioRM);
  
  char tmp_str[256];
  sprintf(tmp_str, "Serial port opened successfully. Port name = %s\n", serDevNameRM.c_str());
  write_log(fp_log, tmp_str);
  printf("Opening serial port %s -> %d succeeded\n", serDevNameRM.c_str(), fdRM);
}

//================================================================================================//
// twistCallBack                                                                                  //
//================================================================================================//
void RoomSeekerLevel1::twistCallBack(const geometry_msgs::Twist::ConstPtr& twist){
  dx_cmd(0) = twist->linear.x;
  dx_cmd(1) = twist->linear.y;
  dx_cmd(2) = twist->angular.z;
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
  std_srvs::Empty req;
  if(client_start_rplidar.call(req)) printf("Service call successed\n");
  else                               printf("Service call failed\n");
}

//================================================================================================//
// open_logfile()                                                                                 //
//   Arguments : none                                                                             //
//   Return    : bool - Whether log file successfully opened or not                               //
//================================================================================================//
bool RoomSeekerLevel1::open_logfile(void){
  char log_file_name[1024];
  char time_str[64];
  struct tm tm;
  
  time_t t = time(NULL);
  localtime_r(&t, &tm);
  sprintf(time_str, "%04d_%02d_%02d__%02d_%02d_%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  sprintf(log_file_name, "%s%s_%s_%s.log", logFileDir.c_str(), preffix, time_str, suffix);
  fp_log = fopen(log_file_name, "w");
  if(fp_log == NULL){
    printf("File open failed. filename = %s\n", log_file_name);
    return(false);
  }
  write_log(fp_log, "Started to logging.\n");
  return(true);
}

//================================================================================================//
// write_log()                                                                                    //
//   Arguments : FILE *fp - file pointer to log file                                              //
//               char *str - char string to write log file                                        //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::write_log(FILE *fp, const char *str){
  char time_str[64];
  time_t t = time(NULL);
  struct tm tm;
  localtime_r(&t, &tm);
  sprintf(time_str, "%04d/%02d/%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  fprintf(fp, "%s : %s", time_str, str);
}

//================================================================================================//
// open_datafile()                                                                                //
//   Arguments : none                                                                             //
//   Return    : bool - Whether data file successfully opened or not                              //
//================================================================================================//
bool RoomSeekerLevel1::open_datafile(void){
  char data_file_name[1024];
  char time_str[64];
  struct tm tm;
  
  time_t t = time(NULL);
  localtime_r(&t, &tm);
  sprintf(time_str, "%04d_%02d_%02d__%02d_%02d_%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  sprintf(data_file_name, "%s%s_%s_%s.csv", dataFileDir.c_str(), preffix, time_str, suffix);
  fp_data = fopen(data_file_name, "w");
  if(fp_data == NULL){
    printf("File open failed. filename = %s\n", data_file_name);
    write_log(fp_log, "File open failed. filename = \n");
    write_log(fp_log, data_file_name);
    return(false);
  }
  write_log(fp_log, "New file opened successfully. File name = \n");
  write_log(fp_log, data_file_name);
  
  fprintf(fp_data, "time,nMEGA,nFM,nRM,cnt_now[0],cnt_now[1],cnt_now[2],cnt_now[3],omega_res[0],omega_res[1],omega_res[2],omega_res[3],omega_cmd[0],omega_cmd[1],omega_cmd[2],omega_cmd[3],vout[0],vout[1],vout[2],vout[3],dx_res[0],dx_res[1],dx_res[2],dx_cmd[0],dx_cmd[1],dx_cmd[2],x_res[0],x_res[1],x_res[2],x_res_[2],int_gz_hpf,ir_state,ax,ay,az,gx,gy,gz,gz_hpf,mx,my,mz,temp,bat_vol,ps2_button,ps2_analogRX,ps2_analogRY,ps2_analogLX,ps2_analogLY\n");
  return(true);
}

//================================================================================================//
// write_data()                                                                                   //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::write_data(void){
  char str[2048];
  char time_str[64];
  struct timeval tm_val;
  struct tm *tm_st;
  gettimeofday(&tm_val, NULL);
  tm_st = localtime(&tm_val.tv_sec);
  sprintf(time_str, "%04d/%02d/%02d %02d:%02d:%02d.%03ld", tm_st->tm_year + 1900, tm_st->tm_mon + 1, tm_st->tm_mday, tm_st->tm_hour, tm_st->tm_min, tm_st->tm_sec, tm_val.tv_usec/1000);
  
  if(control_on == 1){
    fprintf(fp_data, "%s,%d,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%x,%d,%d,%d,%lf,%lf,%lf,%lf,%d,%d,%d,%d,%lf,%x,%d,%d,%d,%d\n",
      time_str, sample_num_node_MEGA, sample_num_node_FM, sample_num_node_RM, cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3],
      omega_res(0), omega_res(1), omega_res(2), omega_res(3), omega_cmd(0), omega_cmd(1), omega_cmd(2), omega_cmd(3),
      vout[0], vout[1], vout[2], vout[3], dx_res(0), dx_res(1), dx_res(2), dx_cmd(0), dx_cmd(1), dx_cmd(2),
      x_res(0), x_res(1), x_res(2), x_res_(2), int_gz_hpf, ir_hex, ax, ay, az, gx, gy, gz, gz_hpf, mx, my, mz, temp,
      bat_vol, ps2_button, ps2_analogRX, ps2_analogRY, ps2_analogLX, ps2_analogLY);
  }
}

//================================================================================================//
// print_state()                                                                                  //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::print_state(void){
  printf("sample_num_node = %d, %d\n", sample_num_node_FM, sample_num_node_RM);
  printf("cnt_now         = %d, %d, %d, %d\n", cnt_now[0], cnt_now[1], cnt_now[2], cnt_now[3]);
  printf("omega_res_x10   = %d, %d, %d, %d\n", omega_res_x10[0], omega_res_x10[1], omega_res_x10[2], omega_res_x10[3]);
  printf("omega_cmd_x10   = %d, %d, %d, %d\n", omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3]);
  printf("vout            = %d, %d, %d, %d\n", vout[0], vout[1], vout[2], vout[3]);
}

//================================================================================================//
// print_sensor()                                                                                 //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::print_sensor(void){
  printf("sample_num_node_MEGA = %d, ir_hex = %x\n", sample_num_node_MEGA, ir_hex);
  printf("a = %d, %d, %d\n", ax, ay, az);
  printf("g = %lf, %lf, %lf\n", gx, gy, gz);
  printf("m = %d, %d, %d\n", mx, my, mz);
  printf("us_dist = %d, %d, %d, %d\n", us_dist[0], us_dist[1], us_dist[2], us_dist[3]);
  printf("us_dist = %d, %d, %d, %d\n", us_dist[4], us_dist[5], us_dist[6], us_dist[7]);
  printf("us_ok = %d, bat_vol_x100 = %d\n", us_ok, bat_vol_x100);
  printf("ps2_button = %x, %d, %d, %d, %d\n", ps2_button, ps2_analogRX, ps2_analogRY, ps2_analogLX, ps2_analogLY);
  printf("interval_MEGA = %d\n", interval_MEGA);
}

//================================================================================================//
// print_work()                                                                                   //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::print_wrk(void){
  printf("mode = %d, control_on = %d\n", mode, control_on);
  printf("dx_cmd        = %lf, %lf, %lf\n", dx_cmd(0), dx_cmd(1), dx_cmd(2));
  printf("omega_cmd     = %lf, %lf, %lf, %lf\n", omega_cmd(0), omega_cmd(1), omega_cmd(2), omega_cmd(3));
  printf("omega_cmd_x10 = %d, %d, %d, %d\n", omega_cmd_x10[0], omega_cmd_x10[1], omega_cmd_x10[2], omega_cmd_x10[3]);
}

//================================================================================================//
// print_odom()                                                                                   //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::print_odom(void){
  printf("OdomRes : %5.2lf, %5.2lf, %5.2lf, Battery = %2.2lf\n", x_res(0), x_res(1), x_res(2)/3.141592*180.0, bat_vol);
}

//================================================================================================//
// apply_filter()                                                                                 //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::apply_filter(void){
  mx_lpf = mx_lpf + g_mag * dt * (mx - mx_lpf);
  my_lpf = my_lpf + g_mag * dt * (my - my_lpf);
  gz_hpf_tmp += dt * gz_hpf;
  gz_hpf = gz - g_gz * gz_hpf_tmp;
  int_gz_hpf += dt * gz_hpf;
}

//================================================================================================//
// odometry_update_res()                                                                          //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::odometry_update_res(void){
  dx_res = J_inv_plus * omega_res;
  x_res(0) += dt * (dx_res(0) * cos(x_res(2)) - dx_res(1) * sin(x_res(2)));
  x_res(1) += dt * (dx_res(0) * sin(x_res(2)) + dx_res(1) * cos(x_res(2)));
  // x_res_[2] = atan2(my_lpf - my_offset, mx_lpf - mx_offset);
  x_res_(2) = atan2(my_lpf - my_offset, mx_lpf - mx_offset) + int_gz_hpf;
  x_res(2) += dt * (dx_res(2));
}

void RoomSeekerLevel1::rate_limit_work(void){
  for(int i = 0; i < 3; i++){
    if(dx_cmd(i) > dx_cmd_rl(i) + dx_rate_limit(i))      dx_cmd_rl(i) += dx_rate_limit(i);
    else if(dx_cmd(i) < dx_cmd_rl(i) - dx_rate_limit(i)) dx_cmd_rl(i) -= dx_rate_limit(i);
    else                                                 dx_cmd_rl(i)  = dx_cmd(i);
  }
}

void RoomSeekerLevel1::inv_kine(void){
  omega_cmd = J_inv * dx_cmd_rl;
}

void RoomSeekerLevel1::command_motor_node(void){
  char str[64] = {0};
  omega_cmd_x10[0] = int(omega_cmd(0) / 3.141592 * 1800);
  omega_cmd_x10[1] = int(omega_cmd(1) / 3.141592 * 1800);
  omega_cmd_x10[2] = int(omega_cmd(2) / 3.141592 * 1800);
  omega_cmd_x10[3] = int(omega_cmd(3) / 3.141592 * 1800);
  sample_num_host++;
  if(sample_num_host >= sample_num_ul) sample_num_host = sample_num_ll;
  sprintf(str, "vel,%3d,%+05d,%+05d,end", sample_num_host, ws_dir[0] * omega_cmd_x10[0], ws_dir[1] * omega_cmd_x10[1]);
  write(fdFM, str, 23);
  sprintf(str, "vel,%3d,%+05d,%+05d,end", sample_num_host, ws_dir[2] * omega_cmd_x10[2], ws_dir[3] * omega_cmd_x10[3]);
  write(fdRM, str, 23);
}

//================================================================================================//
// select_control_mode()                                                                          //
//   Arguments : none                                                                             //
//   Return    : none                                                                             //
//================================================================================================//
void RoomSeekerLevel1::select_control_mode(void){
  if((ps2_button & PSB_SELECT) != 0 && mode != MODE_TWIST){
    printf("PS2 mode reset\n");
    mode = MODE_TWIST;
    dx_cmd(0) = 0;
    dx_cmd(1) = 0;
    dx_cmd(2) = 0;
    no_input_from_ps2 = 1;
    no_input_from_twist = 0;
    no_input_from_twist_cnt = 0;
  }
  
  if((ps2_button & PSB_START) != 0 && mode != MODE_PS2){
    printf("PS2 mode set\n");
    mode = MODE_PS2;
    dx_cmd(0) = 0;
    dx_cmd(1) = 0;
    dx_cmd(2) = 0;
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
  printf("Called stop_control\n");
  control_on = 0;
  write(fdFM, "ttttt", 5);
  write(fdRM, "ttttt", 5);
  std_srvs::Empty req;
  if(client_stop_rplidar.call(req)) printf("Service call \"stop_motor\" successed\n");
  else                              printf("Service call \"stop_motor\" failed\n");
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
      if(no_input_from_twist_cnt >= 300) no_input_from_twist = 1;
    }
  }
  
  if(mode == MODE_PS2){
    // X axis command
    if((ps2_button & PSB_PAD_UP) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(0) = 0.3;
        else                               dx_cmd(0) = 0.2;
      }else                                dx_cmd(0) = 0.1;
    }
    else if((ps2_button & PSB_PAD_DOWN) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(0) = -0.3;
        else                               dx_cmd(0) = -0.2;
      }else                                dx_cmd(0) = -0.1;
    }
    else                                   dx_cmd(0) = 0.0;
      
    // Y axis command
    if((ps2_button & PSB_PAD_LEFT) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(1) = 0.3;
        else                               dx_cmd(1) = 0.2;
      }else                                dx_cmd(1) = 0.1;
    }
    else if((ps2_button & PSB_PAD_RIGHT) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(1) = -0.3;
        else                               dx_cmd(1) = -0.2;
      }else                                dx_cmd(1) = -0.1;
    }
    else                                   dx_cmd(1) = 0.0;
  
    // Rotation command
    if((ps2_button & PSB_L1) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(2) = 135.0 / 180.0 * 3.14;
        else                               dx_cmd(2) = 90.0 / 180.0 * 3.14;
      }else                                dx_cmd(2) = 45.0 / 180.0 * 3.14;
    }
    else if((ps2_button & PSB_R1) != 0){
      continue_control_ps2();
      if((ps2_button & PSB_CROSS) != 0){
        if((ps2_button & PSB_SQUARE) != 0) dx_cmd(2) = -135.0 / 180.0 * 3.14;
        else                               dx_cmd(2) = -90.0 / 180.0 * 3.14;
      }else                                dx_cmd(2) = -45.0 / 180.0 * 3.14;
    }
    else                                   dx_cmd(2) = 0.0;
    
    // PS2 no input judge
    if(no_input_from_ps2 == 0){
      no_input_from_ps2_cnt += 1;
      if(no_input_from_ps2_cnt >= 300) no_input_from_ps2 = 1;
    }
  }
  
  if(control_on == 1){
    if(no_input_from_ps2 == 1 && no_input_from_twist == 1) stop_control();
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
  write_log(fp_log, "Start command transmitted (1st try).\n");
  usleep(500000);                         // 1st : Wait 0.5sec to receive callback
  len = read(fd, buf, sizeof(buf));       // 1st : Check if the callback is sent or not
  if(len <= 10){                          // 2nd : Re-send start command if don't get callback
    write(fd, "s", 1);                    // 2nd : Send start command 's' to serial port
    write_log(fp_log, "Start command transmitted (2nd try).\n");
    usleep(500000);                      // 2nd : Wait 0.5sec to receive callback
    len = read(fd, buf, sizeof(buf));     // 2nd : Check if the callback is sent or not
    if(len <= 10){                        // 2nd : Re-send start command if don't get callback
      write(fd, "s", 1);                  // 3rd : Send start command 's' to serial port
      write_log(fp_log, "Start command transmitted (3rd try).\n");
      usleep(500000);                    // 3rd : Wait 0.5sec to receive callback (3rd)
      len = read(fd, buf, sizeof(buf));   // 3rd : Send start command 's' to serial port
      if(len <= 10){                      // 3rd : Re-send start command if don't get callback
        printf("xxxx : %d did not transmit call back.\n", fd);
        write(fd, "t", 1);                // 3rd : Send stop command for make sure
      }
    }
  }
  len = read(fd, buf, sizeof(buf));       // Empty the receive buffer
  write_log(fp_log, "Start command echo back received.\n");
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
  int tmp[10], tmp_cnt = 0;
  int sign = 1, sign_fix = 0;
  for(int i = 0; i < n; i++){
    if(0x30 <= *(str + i) && *(str + i) <= 0x39){       // 0x30 = '0', 0x39 = '9'
      tmp[tmp_cnt] = *(str + i) - 0x30;
      tmp_cnt++;
      if(tmp_cnt >= 10){ printf("Error at 1st-1\n"); return(false);}
      sign_fix = 1;
    }
    else if(0x41 <= *(str + i) && *(str + i) <= 0x46){  // 0x41 = 'A', 0x46 = 'F'
      tmp[tmp_cnt] = *(str + i) - 0x41 + 10;
      tmp_cnt++;
      if(tmp_cnt >= 10){ printf("Error at 1st-2\n"); return(false);}
      sign_fix = 1;
    }
    else if(0x61 <= *(str + i) && *(str + i) <= 0x66){  // 0x61 = 'a', 0x66 = 'f'
      tmp[tmp_cnt] = *(str + i) - 0x61 + 10;
      tmp_cnt++;
      if(tmp_cnt >= 10){ printf("Error at 1st-3\n"); return(false);}
      sign_fix = 1;
    }
    else if(*(str + i) == ' ');
    else if(*(str + i) == 0) break;
    else{ printf("Error at 4th. *(str + %d) = %c\n", i, *(str + i)); return(false); }
  }
  
  if(tmp_cnt == 0){
    printf("Error at 5th\n");
    return(false);
  }
  
  int data_tmp = 0;
  for(int i = 0; i < tmp_cnt; i++){
    data_tmp += pow(16, tmp_cnt - i - 1) * tmp[i];
  }
  *data = sign * data_tmp;
  
  return(true);
}

//================================================================================================//
// interpretMEGA                                                                                  //
//================================================================================================//
void RoomSeekerLevel1::interpretMEGA(char *split_data){
  int data_tmp[28];
  int int_or_hex[28] = {INT_, HEX_, INT_, INT_, INT_, INT_, INT_, INT_, INT_, INT_, 
                        INT_, INT_, INT_, INT_, INT_, INT_, INT_, INT_, INT_, INT_, 
                        HEX_, INT_, HEX_, INT_, INT_, INT_, INT_, INT_};
  int sample_err = 0;
  
  for(int i = 0; i < 28; i++){
    if(int_or_hex[i] == INT_){
      if(!is_int((split_data + 32 * i), 32, (data_tmp + i))){
        printf("error : i = %d, str = %s\n", i, (split_data + 32 * i));
        sample_err = i;
      }
    }else{
      if(!is_hex((split_data + 32 * i), 32, (data_tmp + i))){
        printf("error : int_or_hex = %d, i = %d, str = %s\n", int_or_hex[i], i, (split_data + 32 * i));
        sample_err = i;
      }
    }
  }
  
  // If error not occured, put the data to member variables
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
  char buf[2048] = {0};
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
        printf("Error in strptr : the last str = %s\n", split_data[strptr]);
      }
    }
    // Transform char strings to data at '\n'
    else if(buf[i] == '\n'){
      if(strptr == 27 && err_flag == 0){
        interpretMEGA(&(split_data[0][0]));
      }else{
        printf("Error in processing received data from MEGA, strptr = %d, chptr = %d, err_flag = %d\n", strptr, chptr, err_flag);
        // for(j = 0; j < 28; j++) printf("%s\n", split_data[j]);
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
// interpretFM                                                                                    //
//================================================================================================//
void RoomSeekerLevel1::interpretFM(char *split_data){
  int data_tmp[10];
  int sample_err = 0;
  
  for(int i = 0; i < 10; i++){
    if(!is_int((split_data + 32 * i), 32, (data_tmp + i))){
      printf("error : i = %d, str = %s\n", i, (split_data + 32 * i));
      sample_err = i;
    }
  }
  
  // If error not occured, put the data to member variables
  if(sample_err == 0){
    sample_num_node_FM = data_tmp[0];
    cnt_now[0] = data_tmp[1];
    cnt_now[1] = data_tmp[2];
    omega_res_x10[0] = data_tmp[3];
    omega_res(0) = data_tmp[3] / 10.0;
    omega_res_x10[1] = data_tmp[4];
    omega_res(1) = data_tmp[4] / 10.0;
    omega_cmd_x10[0] = data_tmp[5];
    omega_cmd_x10[1] = data_tmp[6];
    vout[0] = data_tmp[7];
    vout[1] = data_tmp[8];
    interval_FM = data_tmp[9];
  }else{
    printf("Error in read FM: %d\n", sample_err);
  }
}

//================================================================================================//
// readFM()                                                                                       //
//================================================================================================//
void RoomSeekerLevel1::readFM(void){
  char buf[2048] = {0};
  static char split_data[10][32];
  static int chptr = 0;
  static int strptr = 0;
  static int err_flag = 0;
  int i, j, len;
  
  // Read the received data from serial device
  len = read(fdFM, buf, sizeof(buf));
  for(i = 0; i < len; i++){
    // Split the string at ','
    if(buf[i] == ','){
      chptr = 0;
      if(strptr < 9){
        strptr++;
      }
      else{
        err_flag = 1;
        strptr = 9;
        printf("Error in strptr : the last str = %s\n", split_data[strptr]);
      }
    }
    // Transform char strings to data at '\n'
    else if(buf[i] == '\n'){
      if(strptr == 9 && err_flag == 0){
        interpretFM(&(split_data[0][0]));
      }else{
        printf("Error in processing received data from FM, strptr = %d, chptr = %d, err_flag = %d\n", strptr, chptr, err_flag);
        // for(j = 0; j < 10; j++) printf("%s\n", split_data[j]);
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
// interpretRM                                                                                    //
//================================================================================================//
void RoomSeekerLevel1::interpretRM(char *split_data){
  int data_tmp[10];
  int sample_err = 0;
  
  for(int i = 0; i < 10; i++){
    if(!is_int((split_data + 32 * i), 32, (data_tmp + i))){
      printf("error : i = %d, str = %s\n", i, (split_data + 32 * i));
      sample_err = i;
    }
  }
  
  // If error not occured, put the data to member variables
  if(sample_err == 0){
    sample_num_node_RM = data_tmp[0];
    cnt_now[2] = data_tmp[1];
    cnt_now[3] = data_tmp[2];
    omega_res_x10[2] = data_tmp[3];
    omega_res(2) = data_tmp[3] / 10.0;
    omega_res_x10[3] = data_tmp[4];
    omega_res(3) = data_tmp[4] / 10.0;
    omega_cmd_x10[2] = data_tmp[5];
    omega_cmd_x10[3] = data_tmp[6];
    vout[2] = data_tmp[7];
    vout[3] = data_tmp[8];
    interval_RM = data_tmp[9];
  }else{
    printf("Error in read RM: %d\n", sample_err);
  }
}

//================================================================================================//
// readRM()                                                                                       //
//================================================================================================//
void RoomSeekerLevel1::readRM(void){
  char buf[2048] = {0};
  static char split_data[10][32];
  static int chptr = 0;
  static int strptr = 0;
  static int err_flag = 0;
  int i, j, len;
  
  // Read the received data from serial device
  len = read(fdRM, buf, sizeof(buf));
  for(i = 0; i < len; i++){
    // Split the string at ','
    if(buf[i] == ','){
      chptr = 0;
      if(strptr < 9){
        strptr++;
      }
      else{
        err_flag = 1;
        strptr = 9;
        printf("Error in strptr : the last str = %s\n", split_data[strptr]);
      }
    }
    // Transform char strings to data at '\n'
    else if(buf[i] == '\n'){
      if(strptr == 9 && err_flag == 0){
        interpretRM(&(split_data[0][0]));
      }else{
        printf("Error in processing received data from RM, strptr = %d, chptr = %d, err_flag = %d\n", strptr, chptr, err_flag);
        // for(j = 0; j < 10; j++) printf("%s\n", split_data[j]);
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
// Initialize Joint State                                                                         //
//================================================================================================//
void RoomSeekerLevel1::initJointStates(void){
  joint_states.header.frame_id = joint_state_header_frame_id;
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
