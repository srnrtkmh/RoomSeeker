//================================================================================================//
//                                                                                                //
// FILE : room_seeker_level1.h                                                                    //
// Memo : RoomSeeker level 1 node class                                                           //
//                                                                                                //
// Updated : 2021/02/22 Started this project based on "room_seeker_l1.py"                         //
//                                                                                                //
//                                                                       (C) 2021 Kyohei Umemoto  //
//                                                                                                //
//================================================================================================//

#ifndef _ROOM_SEEKER_LEVEL1_H_
#define _ROOM_SEEKER_LEVEL1_H_

//================================================================================================//
// Import Module                                                                                  //
//================================================================================================//
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
// #include <Eigen/Dense>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>
#include <room_seeker_msgs/SensorState.h>
#include <room_seeker_msgs/VersionInfo.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <room_seeker_msgs/SensorState.h>
#include <room_seeker_msgs/VersionInfo.h>

// For serial communication
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

//================================================================================================//
// Constants                                                                                      //
//================================================================================================//
// PS2 controller constants
#define PSB_SELECT    0x0001
#define PSB_L3        0x0002
#define PSB_R3        0x0004
#define PSB_START     0x0008
#define PSB_PAD_UP    0x0010
#define PSB_PAD_RIGHT 0x0020
#define PSB_PAD_DOWN  0x0040
#define PSB_PAD_LEFT  0x0080
#define PSB_L2        0x0100
#define PSB_R2        0x0200
#define PSB_L1        0x0400
#define PSB_R1        0x0800
#define PSB_GREEN     0x1000
#define PSB_RED       0x2000
#define PSB_BLUE      0x4000
#define PSB_PINK      0x8000
#define PSB_TRIANGLE  0x1000
#define PSB_CIRCLE    0x2000
#define PSB_CROSS     0x4000
#define PSB_SQUARE    0x8000

// Control Mode constants
#define MODE_TWIST    0
#define MODE_PS2      1

// The data type receive from level 1 node
#define INT 0
#define HEX 1

// The others
#define BAT_CAPA     6.5
#define MIN_VOLT     6.8
#define MAX_VOLT     8.4
#define HARDWARE_VER "0.0.0"
#define SOFTWARE_VER "0.0.0"
#define FIRMWARE_VER "0.0.0"

//================================================================================================//
// Function Declarations                                                                          //
//================================================================================================//
bool check_int(std::string str);  // Descriminate the literal is integer or not
bool is_hex(std::string str);     // Descriminate the literal is Hexadecimal or not

//================================================================================================//
// RoomSeeker level 1 class                                                                       //
//   Index of wheelspace value : 0 - Front Right, 1 - Fron Left, 2 - Rear Right, 3 - Rear Left    //
//   Index of workspace value  : 0 - Long dir, 1 - Width dir, 2 - Yaw angle                       //
//================================================================================================//
class RoomSeekerLevel1{
  public:
    int mode;                   // Controll mode 0 : command mode , 1 : PS2 controller mode
    int sample_num_node_MEGA;   // Sample number of the Arduino MEGA
    int sample_num_node_FM;     // Sample number of the Front Motor Controller
    int sample_num_node_RM;     // Sample number of the Rear Motor Controller
    int sample_num_host;        // Sample number of the host
    int sample_num_ul;          // Upper limit of sample number
    int sample_num_ll;          // Lower limit of sample number
    double ws_dir[4];           // Wheelspace direction co-efficient
    int cnt_now[4];             // Wheel encoder counted value
    double omega_res[4];        // Wheel space velocity response [rad/s]
    int omega_res_x10[4];       // Wheel space velocity response [10^-1deg/s]
    double omega_cmd[4];        // Wheel space velocity command [rad/s]
    double omega_cmd_x10[4];    // Wheel space velocity command [10^-1deg/s]
    int vout[4];                // Voltage output (PWM width : 0 - 4095)
    double x_res[3];            // Workspace pose response calculated from velocity response [0:m, 1:m, 2:rad]
    double x_res_[3];           // 
    double x_res2[3];           // Workspace pose response calculated from velocity command [0:m, 1:m, 2:rad]
    double dx_res[3];           // Workspace velocity response [0 : m/s, 1 : m/s, 2 : deg/s]
    double dx_res_x10[3];       // Workspace velocity response [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    double x_cmd[3];            // Workspace pose command [0:m, 1:m, 2:rad]
    double dx_cmd[3];           // Workspace velocity command [0 : m/s, 1 : m/s, 2 : rad/s]
    double dx_rate_limit[3];    // Rate limit value in workspace [0 : m/20ms, 1 : m/20ms, 2 : rad/20ms]
    double dx_cmd_rl[3];        // Workspace velocity command applied rate limit [0 : m/s, 1 : m/s, 2 : rad/s]
    double dx_cmd_x10[3];       // Workspace velocity command x10ver [0 : 10^-1m/s, 1 : 10^-1m/s, 2 : 10^-2deg/s]
    double wheel_radius;        // Robot wheel radius [mm]
    double base_width;          // Robot wheel base width [mm]
    double base_length;         // Robot wheel base length (between front and rear wheel shaft) [mm]
    // double J_inv[4][4] = [[1.0/(wheel_radius/1000.0),  1.0/(wheel_radius/1000.0),  ((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0)]  // Inverse jacobian
                          // ,[1.0/(wheel_radius/1000.0), -1.0/(wheel_radius/1000.0), -((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0)]
                          // ,[1.0/(wheel_radius/1000.0), -1.0/(wheel_radius/1000.0),  ((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0)]
                          // ,[1.0/(wheel_radius/1000.0),  1.0/(wheel_radius/1000.0), -((base_width/1000.0) + (base_length/1000.0))/2.0/(wheel_radius/1000.0)]])
    // double J_inv_plus = np.dot(np.linalg.inv(np.dot(J_inv.transpose(), J_inv)), J_inv.transpose())
    double x_ul;            // Upper limit of the Workspace
    double x_ll;           // Lower limit of the Workspace
    int ir_hex;                                      // Ir sensor date in hex
    double ax;                                          // Acc x-axis
    double ay;                                        // Acc y-axis
    double az;                                         // Acc z-axis
    double gc;               // Gyro constants to transform raw data to rad/s [rad/s]
    double gx;                                         // Gyro x-axis [rad/s]
    double gy;                                          // Gyro y-axis [rad/s]
    int gz;                                          // Gyro z-axis [rad/s]
    double gz_hpf;                                // Gyro z-axis applied HPF [rad/s]
    double gz_hpf_tmp;                                // Temporary for HPF [rad/s]
    double int_gz_hpf;                               // Integral of gz applied HPF [rad]
    double g_gz;                                    // Cutoff angular frequency of HPF for gyro sensor [rad/s]
    double mx;                                         // Mag x-axis
    double my;                                         // Mag y-axis
    double mz;                                      // Mag z-axis
    double mx_lpf;                                     // Mag x-axis applied LPF
    double my_lpf;                                     // Mag y-axis applied LPF
    double mz_lpf;                                    // Mag z-axis applied LPF
    double mx_offset;                              // Offset of Mag x-axis
    double my_offset;                             // Offset of Mag y-axis
    double mz_offset;                              // Offset of Mag y-axis
    double mr_offset;                          // Radius of Mag x-y circle
    double g_mag;                                  // Cutoff angular frequency of LPF for magnetosensor [rad/s]
    double temp;                                      // Temperature
    double us_dist[8];                              // Distance from ultra sonic sensor
    double us_ok;                                   // Ultrasonic sensor is enabled or not
    int ps2_button;                                // PS2 controller button state
    int ps2_analogRX;                              // PS2 controller button state
    int ps2_analogRY;                               // PS2 controller button state
    int ps2_analogLX;                             // PS2 controller button state
    int ps2_analogLY;                              // PS2 controller button state
    int fdMEGA, fdFM, fdRM;                         // File Descriptor
    struct termios oldtioMEGA, oldtioFM, oldtioRM;  // 
    struct termios newtioMEGA, newtioFM, newtioRM;  // 
    int conFM;                                      // Serial console connected to arduino motor controller
    int conRM;                                     // Serial console connected to arduino motor controller
    int conMEGA;                                     // Serial console connected to arduino motor controller
    double bat_vol;                                   // Battery voltage
    int bat_vol_x100;                              // Battery voltage x100 (raw data)
    int ps2_ctrl;                                   // PS2 controller is enabled or not
    int interval_MEGA;                           // Interval between samples
    int interval_FM;                              // Interval between samples
    int interval_RM;                               // Interval between samples
    int interval;                                    // Interval
    double pi; 
    char before_nokoriMEGA[1024];
    char before_nokoriFM[1024];
    char before_nokoriRM[1024];
    int pub10cnt;
    
    ros::NodeHandle nh;
  
    // Publisher
    ros::Publisher odom_pub;
    ros::Publisher joint_states_pub;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher sensor_state_pub;
    ros::Publisher version_info_pub;
    ros::Publisher battery_state_pub;
    tf2_ros::TransformBroadcaster br;
    
    // Subscriber
    ros::Subscriber twist_sub;
    
    // Message variables
    nav_msgs::Odometry odom;
    sensor_msgs::JointState joint_states;
    sensor_msgs::MagneticField mag_msg;
    sensor_msgs::BatteryState battery_state_msg;
    int battery_state = 0;
    room_seeker_msgs::SensorState sensor_state_msg;
    room_seeker_msgs::VersionInfo version_info_msg;
    
    // Power control
    double dt;                              // Sampling time of the motor controller
    double ros_rate;                        // Set value for rospy.Rate()
    bool control_on;
    bool no_input_from_ps2;
    int  no_input_from_ps2_cnt;
    bool no_input_from_twist;
    int  no_input_from_twist_cnt;
    
    // Member functions --------------------------------------------------------------------------//
    RoomSeekerLevel1();
    ~RoomSeekerLevel1();
    
    void process_one_step(void);
    
    bool open_logfile(void);
    bool open_datafile(char *strCsvName);
    
    void print_state(void);
    void print_sensor(void);
    void print_wrk(void);
    void print_odom(void);
    
    void apply_filter(void);
    void odometry_update_res(void);
    void odometry_update_cmd(void); // Not in use it cannnot estimate pose enough precision
    void rate_limit_work(void);
    void inv_kine(void);
    void command_motor_node(void);
    
    void select_control_mode(void);
    void continue_control_ps2(void);
    void stop_control(void);
    void control_command(void);
    void data_output(void);
    
    void initMEGA(void);
    void initFM(void);
    void initRM(void);
    void readMEGA(void);
    void readFM(void);
    void readRM(void);
    void interpretMEGA(char *split_data);
    void interpretFM(char *split_data);
    void interpretRM(char *split_data);
    void send_start(int fd);
    
    void initJointStates(void);
    void publishDriveInformation(void);
    void publishImuMsg(void);
    void publishMagMsg(void);
    void publishSensorStateMsg(void);
    void publishVersionInfoMsg(void);
    void publishBatteryStateMsg(void);
    
    void twistCallBack(const geometry_msgs::Twist::ConstPtr& twist);
    void start_rplidar_motor(void);
};

#endif
