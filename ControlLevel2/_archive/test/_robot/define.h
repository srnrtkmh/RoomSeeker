/**************************************************************************************************/
/*                                                                                                */
/* FILE       : define.h                                                                          */
/* MEMO       : Definition of the variables used in nozamani experiment or simulation             */
/*                                                                                                */
/* 2012/10/03 : Start to edit this file                                                           */
/* 2012/11/12 : Changed structure of this project to use global variables                         */
/*                                                                                                */
/**************************************************************************************************/

#ifndef DEFINE
#define DEFINE

//================================================================================================//
// Constant parameter of nozamani                                                                 //
//================================================================================================//
// Physical Parameters
static const double g = 9.80655;			// Acceleration of gravity
static const double RW = 0.25;				// Wheel Radius
static const double MW = 2.76;				// Wheel Mass
static const double JmW = 0.47;				// Wheel intertia of motor's rotor
static const double Tread = 0.66;			// Tread between both wheel
static const double D1 = 0.0;				// distance between the center of the body and Motor1

static const double L[7] = {0.613, 0.0, 0.20, 0.20, 0.10, 0.20, 0.0};					// Length of each link
//static const double Lg[7] = {0.613, 0.0, 0.20, 0.20, 0.10, 0.18, 0.12};				// Length of each link
static const double Lg[7] = {0.30, 0.0, 0.192, 0.161, 0.0659, 0.125, 0.0};				// Length between motor shaft and COG of each link (Identified value from gravity torque)
//static const double Lg[7] = {0.206, 0.0, 0.192, 0.161, 0.0659, 0.125, 0.0};			// Length between motor shaft and COG of each link (Identified value from gravity torque)
static const double Jm[7] = {0.0, 0.081, 0.081, 0.081, 0.081, 0.015, 0.015};			// Inertia of each motor's rotor

static const double sum_m = (38.4+1.12+1.35+1.35+0.82+0.66+0.34);						// Summation of each links' mass
vec9 m(38.4, 1.12, 1.35, 1.35, 0.82, 0.66, 0.34, 2.76, 2.76);							// Mass of each link
static const vec3 ex(1.0, 0.0, 0.0), ey(0.0, 1.0, 0.0), ez(0.0, 0.0, 1.0);				// Unit vector

// Parameters for Numerical Calculation
static const double dt = 0.001;			// Sampling time 1ms
static const double TOL = 1.0e-6;			// Tolerance of numerical simulation result

// psc_data[0] configuration
static const int PSC_SELECT  = 0;
static const int PSC_LSW     = 1;
static const int PSC_RSW     = 2;
static const int PSC_START   = 3;
static const int PSC_UP      = 4;
static const int PSC_RIGHT   = 5;
static const int PSC_DOWN    = 6;
static const int PSC_LEFT    = 7;

// psc_data[1] configuration
static const int PSC_L2      = 0;
static const int PSC_R2      = 1;
static const int PSC_L1      = 2;
static const int PSC_R1      = 3;
static const int PSC_SANKAKU = 4;
static const int PSC_MARU    = 5;
static const int PSC_BATSU   = 6;
static const int PSC_SHIKAKU = 7;

//================================================================================================//
// Global variables for nozamani                                                                  //
//================================================================================================//
// time variables
int mst_cnt, data_cnt, stick_cnt;							// master counter and counter used in recording main data and stick diagram data
double t;													// time

// Flags
int seq, seq_comp;
int dob_flag, pado_flag, wob_flag, rtob_flag, workspace_flag, fc_flag, output_flag, emergency_flag;
int srv_on_flag[8];

// Response variables
vec9 q_cmd, dq_cmd, ddq_cmd, ddq_ref;						// Joint space Command and Reference value
vec9 q_res, dq_res, ddq_res, q_res_buf[100], dq_res_dir;	// Joint space Response value
vec9 tau_ref, tau_dis, tau_dis_hat, tau_fric, tau_fric_n;	// Torque variables
vec3 xee_cmd, dxee_cmd, ddxee_cmd, ddxee_ref;				// Work space command and reference value in robot coordinate
vec3 xee_res, dxee_res, ddxee_res, ddxee_dis;				// Work space response value in robot coordinate
vec3 a_cmd, da_cmd, dda_cmd, dda_ref;						// Posture space command and reference value in robot coordinate
vec3 a_res, da_res, dda_res, dda_dis;						// Posture space response value in robot coordinate
vec3 m_xee_cmd, m_dxee_cmd, m_ddxee_cmd, m_ddxee_ref;		// Work space command and reference value in manipulator coordinate
vec3 m_xee_res, m_dxee_res, m_ddxee_res, m_ddxee_dis;		// Work space response value in manipulator coordinate
vec3 m_a_cmd, m_da_cmd, m_dda_cmd, m_dda_ref;				// Posture space command and reference value in manipulator coordinate
vec3 m_a_res, m_da_res, m_dda_res, m_dda_dis;				// Posture space response value in manipulator coordinate
vec3 Fee_cmd, Fee_res, Fee_ref, Fee_reac_hat;				// Reaction force variables of the end-effector in robot coordinate
vec3 Fa_cmd, Fa_res, Fa_ref, Fa_reac_hat;					// Reaction torque variables of the end-effector in robot coordinate
vec3 xg_cmd, dxg_cmd, ddxg_cmd, ddxg_ref;					// COG command and reference in robot coordinate
vec3 xg_res, dxg_res, ddxg_res;								// COG response in robot coordinate
vec3 xrb_cmd, dxrb_cmd, ddxrb_cmd, ddxrb_ref;				// Robot position command and response in world coordinate
vec3 xrb_res, dxrb_res, ddxrb_res;							// Robot position response in world coordinate
vec3 R_xrb_cmd, R_dxrb_cmd, R_ddxrb_cmd, R_ddxrb_ref;		// Robot position command in robot coordinate
vec3 R_xrb_res, R_dxrb_res, R_ddxrb_res;					// Robot position response in robot coordinate

// For reaction torque observer
vec9 tau_reac, tau_reac_hat;			// Reaction torque
mat99 Mn_rtob;							// Inertia matrix for RTOB
mat69 Jreac;							// Jacobian used in RTOB
vec6 F_reac_hat;						// Reaction force and torque at end-effector

// Model variables
double DH[8][4];						// Link parameters using Denavit-Hartenberg method
vec3 i_1_p_hat_[8], i_s_hat_[7];		// Relative position vector of link coordinate position and COG
vec3 o_p_hat_r, o_p_hat_l;				// Relative position vector of wheel coordinate position
vec3 i_omega[7], i_domega[7];			// Relative angular velocity of i th coordinate and its derivative viewed from i th coordinate
vec3 R_omega_R;							// 
vec3 R_p_[8], R_dp_[8], R_omega_[8];	// Relative position, velocity and angular velocity of i th coordinate viewed from robot coordinate
vec3 R_z_[7], m_z_[7], m_p_[8];			// unit z-axis vector of each link coordinate viewed from robot coordinate and manipulator coordinate
mat33 i_1_R_[8], i_1_RT_[8];			// Rotation matrix ^{i-1}R_{i} and its transposition (i-1 -> i)
mat33 o_R_wh, o_RT_wh;					// Rotation matrix 0_R_wh and its transposition (0 -> wheel)
mat33 W_R_R;							// Rotation matrix ^{W}R_{R} (W -> R)
mat33 R_R_[8], R_RT_[8];				// Rotation matrix and its transposition ^{R}R_{i} (R -> i)
mat44 i_1_Trans_[8];					// Homogeneous transformation matrix (^{i-1}T_{i}) (i-1 -> i)
mat44 R_Trans_[8];						// Homogeneous transformation matrix (^{R}T_{i})   (R -> i)
mat39 Jee, dJee, m_Jee, m_dJee;			// Jacobian from joint angle to end-effector position
mat39 Ja, dJa, m_Ja, m_dJa;				// Jacobian from joint angle to end-effector posture
mat39 Jg, dJg;							// Jacobian from joint angle to COG
mat39 Jrb, dJrb;						// Jacobian and Inverse Jacobian from joint angle to robot position
mat39 R_Jrb, R_dJrb;					// Jacobian and Inverse Jacobian from joint angle to robot position in robot axis
mat99 M, M_inv, Mn, Mn_inv;				// Inertia matrix and Nominal inertia matrix
vec9 G, Gn, H, Hn;						// Gravity and Nonlinear vector
vec9 JT_F_res, JT_F_ref;				// External Force vector

// Controller Parameters
vec9 mn, Jmn, Ktn, Dn, Clmbn;				// Nominal mass, inertia of each motor's rotor, torque coefficient, dumping coefficient, coulomb coefficient
mat33 I[9], In[9];							// True(in simulation) and Nominal intertia tensor of each link
double sum_mn;								// Summation of all links' nominal mass (0, 1, ..., 6)
vec9 Kp_j, Kv_j;							// Joint space control gain
vec3 Kp_w, Kv_w;							// Work space position control gain
vec3 Kp_a, Kv_a;							// Work space posture control gain
vec3 Kp_cog, Kv_cog;						// COG control gain
vec9 Kv_null;								// Null space dumping gain
double Kp_phi, Kv_phi;						// Yaw motion control gain
double Kp_rb, Kv_rb, Ki_rb;					// Robot position control gain
double e_xrb_0, Kp_rb2, Kv_rb2, Ki_rb2;		// Robot position control gain (method2)
vec4 p;										// Pole placement for stabilization controller using single inverted pendulum model
vec9 g_dob, g_rtob, g_dq, g_ddq;			// DOB gain and derivative gain
vec3 g_wob;									// WOB gain

// Variables used in offset estimation
double q0_off_hat, dq0_off_hat, q0_off_dis;		// Offset value of q0 (To compensate gyro offset and drift)
double g_p_off, g_i_off;						// Observer gain for gyro sensor offset estimation
double x[7] = {0.0}, z[7] = {0.0}, X = 0.0;		// Used for offset calculation
double tau0_dis_hat_int = 0.0;					// Integral of disturbance torque at link 0

unsigned char status = 0;								// 
unsigned int x_accl = 0, y_accl = 0, z_accl = 0;		// 
unsigned int roll_pos = 0, pitch_pos = 0, yaw_pos = 0;	// 
unsigned int roll_vel = 0, pitch_vel = 0, yaw_vel = 0;	// 
double q0_datatec = 0.0;								// Passive joint angle obtained from datatec gyro sensor

// Single inverted pendulum model parameter
double mw, mg, lgg;						// Wheel mass, pendulum mass and pendulum length
double Mnaa, Mnau, Mnua, Mnuu, detMn;	// Parameters of inertia matrix
vec4 pp, xx, KK;						// State variable : pp is desired poles, xx = [qw, qg, dqw, dqg]^T, KK is feedback gain

// For experiment
int cnt_in[8], cnt_in_pre[8], cnt_page[8], da_out[8];	// Counter input and D/A output
double q_res_enc[8], dq_res_enc[8];						// Joint angle converted from encoder input
double g_enc[8];										// Gain for pseudo derivative
int gyro_in[2], pre_gyro_in[2];							// Gyro input and its previous sample
int gyro_set_flag;										// Gyro value set flag
vec9 q_org, q_init;										// Origin and initial position of joint angle
vec3 xee_init, a_init;									// Initial position of end-effector
int psc_data[6];										// Play station controller data
unsigned long MappedAddress;							// For force sensor
extern int errno;										// For force sensor
vec9 tau_offset;

// For safety mechanism
const double tau_limit[9] = {1.0, 20.0, 20.0, 20.0, 20.0, 3.5, 3.5, 67.2, 67.2};	// Torque limit
const int SATURATION_MAX_CNT = 50;													// Sample number allowed to saturate continuously
int saturation_counter[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};							// Saturation counter for each joint
int saturation_num = 0;																// The joint number continuous saturation detected
const double dq_limit[9] = {0.5, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};	// Overspeed limit
const int OVERSPEED_MAX_CNT = 150;													// Sample number allowed to overspeed continuously
int overspeed_counter[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};								// Overspeed counter for each joint
int overspeed_num = 0;																// The joint number continuous overspeed detected

// For identification
double id_q_init, id_q_range, id_rsl;								// Initial position, data saving range, resolution of velocity
int id_jnt, id_dir, id_vel, id_cnt, id_vel_num, id_cnt_num;			// Joint number to be identified
int id_smp_num_pos[20], id_smp_num_neg[20];							// The number of sample successfully obtained
double id_dq[20], id_dq_tol;										// Setting joint velocity in identification
double id_tau_dis_hat_pos[20][5000], id_tau_dis_hat_neg[20][5000];	// Disturbance torque in identification
double id_q_res_pos[20][5000], id_q_res_neg[20][5000];				// Joint angle in identification
double id_dq_res_pos[20][5000], id_dq_res_neg[20][5000];			// Joint angle in identification
double id_tau_pos_ave[20], id_tau_neg_ave[20];

// Temporary variables before saving simulation or experimental result
#define DATA_SIZE 100000																					// Max sample number of simulation or experimental result
int mst_cnt_tmp[DATA_SIZE];																					// master counter
double t_tmp[DATA_SIZE];																					// time
vec9 q_cmd_tmp[DATA_SIZE], dq_cmd_tmp[DATA_SIZE], ddq_cmd_tmp[DATA_SIZE];									// Joint space Command and Reference value
vec9 q_res_tmp[DATA_SIZE], dq_res_tmp[DATA_SIZE], ddq_res_tmp[DATA_SIZE];									// Joint space Response value
vec9 tau_ref_tmp[DATA_SIZE], tau_dis_tmp[DATA_SIZE], tau_dis_hat_tmp[DATA_SIZE];							// Torque variables
vec9 tau_fric_tmp[DATA_SIZE], tau_int_tmp[DATA_SIZE], tau_reac_tmp[DATA_SIZE];								// Torque variables
vec3 xee_cmd_tmp[DATA_SIZE], dxee_cmd_tmp[DATA_SIZE], ddxee_cmd_tmp[DATA_SIZE];								// Work space command and reference value
vec3 xee_res_tmp[DATA_SIZE], dxee_res_tmp[DATA_SIZE], ddxee_res_tmp[DATA_SIZE];								// Work space response value
vec3 a_cmd_tmp[DATA_SIZE], da_cmd_tmp[DATA_SIZE], dda_cmd_tmp[DATA_SIZE], dda_ref_tmp[DATA_SIZE];			// Posture space command and reference value in robot coordinate
vec3 a_res_tmp[DATA_SIZE], da_res_tmp[DATA_SIZE], dda_res_tmp[DATA_SIZE], dda_dis_tmp[DATA_SIZE];			// Posture space response value in robot coordinate
vec3 Fee_cmd_tmp[DATA_SIZE], Fee_res_tmp[DATA_SIZE], Fee_ref_tmp[DATA_SIZE], Fee_reac_hat_tmp[DATA_SIZE];	// Force variables of the end-effector
vec3 Fa_cmd_tmp[DATA_SIZE], Fa_res_tmp[DATA_SIZE], Fa_ref_tmp[DATA_SIZE], Fa_reac_hat_tmp[DATA_SIZE];		// Torque variables of the end-effector
vec3 xg_cmd_tmp[DATA_SIZE], dxg_cmd_tmp[DATA_SIZE], ddxg_cmd_tmp[DATA_SIZE];								// COG command and reference
vec3 xg_res_tmp[DATA_SIZE], dxg_res_tmp[DATA_SIZE], ddxg_res_tmp[DATA_SIZE];								// COG response
vec3 xrb_cmd_tmp[DATA_SIZE], dxrb_cmd_tmp[DATA_SIZE], ddxrb_cmd_tmp[DATA_SIZE], ddxrb_ref_tmp[DATA_SIZE];	// Robot position command and reference
vec3 xrb_res_tmp[DATA_SIZE], dxrb_res_tmp[DATA_SIZE], ddxrb_res_tmp[DATA_SIZE];								// Robot position response
double q0_datatec_tmp[DATA_SIZE], q0_off_hat_tmp[DATA_SIZE], q0_off_dis_tmp[DATA_SIZE];						// Passive joint angle obtained from datatec gyro sensor

// Stick diagram data
double stick[5][2];
double xee_cmd_stick[2];

// Parameters of Experimental equipment ----------------------------------------------------------//
// PCI board address
#define PCI6205C0	0xd000
#define PCI6205C1	0xd100
#define PCI6205C2	0xd200
#define PCI6205C3	0xd300
#define PCI6205C4	0xd400
#define PCI3329		0xd500

// Encoder parameter
#define WHEEL_ENC_RSL		1000.0	// Encoder resolution
#define WHEEL_GEAR_RATIO	50.0	// Gear ratio
#define MANI_ENC_RSL		1000.0	// Encoder resolution
#define MANI_GEAR_RATIO		100.0	// Gear ratio
#define ENC_MULTI			4.0		// Encoder multiplying setting

// D/A parameter
#define DA_RSL		0x1000	// D/A resolution
#define DA_OFFSET	0x0800	// D/A offset value (command value)
#define DA_MAX		0x0fff	// Maximum D/A digital value
#define DA_MIN		0x0000	// Minimum D/A digital value
static const double I_MAX[8]={6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 15.0, 15.0};
static const double I_MIN[8]={-6.0, -6.0, -6.0, -6.0, -6.0, -6.0, -15.0, -15.0};

// Gyro parameter
#define GYRO_RSL		0x10000	// Gyro resolution
#define GYRO_CMD_ONE	'D'		// Command to gyro to send its value only one time
#define GYRO_CMD_CONT	'T'		// Command to gyro to send its value continuously
#define GYRO_CMD_STOP	'S'		// Command to gyro to stop sending its value if it is set to send continuously

#define COM_PORT	0			// Serial port number

// Force sensor parameter
#define FORCE_A		568.5378*(9.80655/1000)			// A in the "y=A*x+b"
#define FORCE_B		(68.1702*(9.80665/1000)-0.7)	// B in the "y=A*x+b"

// For force sensor (contents of "jr3.h")
#ifndef _SCANINFO_H
#define _SCANINFO_H

#define JR3_READ		(200)
#define JR3_WRITE		(201)
#define JR3_SET_ADDR		(202)
#define JR3_GET_BADDR		(203)

#endif

// Constants used in force sensor function
#define FRAMESIZE		0xff
#define Jr3DmAddrMask	0x6000
//#define PAGE_SIZE		0x1000
#define IFS_A			(-0.0323776)
//#define IFS_B			(+4.4566715)
//#define IFS_B			(-4.586182)
//#define IFS_B			(-2.2095105)
#define IFS_B			(+1.7095105)

//================================================================================================//
// Initialize variables that is common among all projects                                         //
//================================================================================================//
void init_variables_common(void)
{
	int i;
	
	// Initial joint space response
	q_org.clear();
	q_res = q_org; dq_res.clear(); ddq_res.clear();
	q_cmd = q_org; dq_cmd.clear(); ddq_cmd.clear();
	tau_ref.clear(); tau_dis.clear(); tau_dis_hat.clear();
	
	// Initial end-effector position and posture response in robot coordinate
	xee_res.clear(); dxee_res.clear(); ddxee_res.clear();
	xee_cmd.clear(); dxee_cmd.clear(); ddxee_cmd.clear();
	a_res.clear(); da_res.clear(); dda_res.clear();
	a_cmd.clear(); da_cmd.clear(); dda_cmd.clear();
	Fee_res.clear(); Fee_cmd.clear(); Fee_ref.clear(); Fee_reac_hat.clear();
	Fa_res.clear(); Fa_cmd.clear(); Fa_ref.clear(); Fa_reac_hat.clear();
	
	// Initial end-effector position and posture response in manipulator coordinate
	m_xee_res.clear(); m_dxee_res.clear(); m_ddxee_res.clear();
	m_xee_cmd.clear(); m_dxee_cmd.clear(); m_ddxee_cmd.clear();
	m_a_res.clear(); m_da_res.clear(); m_dda_res.clear();
	m_a_cmd.clear(); m_da_cmd.clear(); m_dda_cmd.clear();
	
	// Initial COG response
	xg_res.clear(); dxg_res.clear(); ddxg_res.clear();
	xg_cmd.clear(); dxg_cmd.clear(); ddxg_cmd.clear();
	
	// Initial robot position response
	xrb_res.clear(); dxrb_res.clear(); ddxrb_res.clear();
	xrb_cmd.clear(); dxrb_cmd.clear(); ddxrb_cmd.clear();
	R_xrb_res.clear(); R_dxrb_res.clear(); R_ddxrb_res.clear();
	R_xrb_cmd.clear(); R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
	
	// Inertia tensor of manipulator and wheel
	for(i=0; i<=6; i++){
		I[i].row_set(0, 0.0, 0.0, 0.0);
		I[i].row_set(1, 0.0, 0.0, 0.0);
//		I[i].row_set(2, 0.0, 0.0, Jm[i] + m[i]*L[i]*L[i]);
		I[i].row_set(2, 0.0, 0.0, Jm[i] + m[i]*Lg[i]*Lg[i]);
	}
	I[7].row_set(0, MW*RW*RW/4.0, 0.0, 0.0);
	I[7].row_set(1, 0.0, MW*RW*RW/4.0, 0.0);
//	I[7].row_set(2, 0.0, 0.0, MW*RW*RW/2.0);
	I[7].row_set(2, 0.0, 0.0, JmW + MW*RW*RW/2.0);
	I[8] = I[7];
	
	// Model parameters
	i_s_hat_[0].set(0.0, -Lg[0], 0.0);
	i_s_hat_[1].set(Lg[1], 0.0, 0.0);
	i_s_hat_[2].set(Lg[2], 0.0, 0.0);
	i_s_hat_[3].set(Lg[3], 0.0, 0.0);
	i_s_hat_[4].set(Lg[4], 0.0, 0.0);
	i_s_hat_[5].set(0.0, -Lg[5], 0.0);
	i_s_hat_[6].set(0.0,   0.0, 0.0);
	
	i_1_p_hat_[0].set(0.0,   0.0, 0.0);
	i_1_p_hat_[1].set(0.0, -L[0], 0.0);
	i_1_p_hat_[2].set(L[1],  0.0, 0.0);
	i_1_p_hat_[3].set(L[2],  0.0, 0.0);
	i_1_p_hat_[4].set(L[3],  0.0, 0.0);
	i_1_p_hat_[5].set(L[4],  0.0, 0.0);
	i_1_p_hat_[6].set(0.0, -L[5], 0.0);
	i_1_p_hat_[7].set(0.0,   0.0, 0.0);
	o_p_hat_r.set(0.0, 0.0, -Tread/2.0);
	o_p_hat_l.set(0.0, 0.0,  Tread/2.0);
	
	// Reset nominal parameter
	mn.clear(); Jmn.clear(); Ktn.clear(); Dn.clear(); Clmbn.clear();
	for(i=0; i<9; i++){
		In[i].row_set(0, null_vec3);
		In[i].row_set(1, null_vec3);
		In[i].row_set(2, null_vec3);
	}
	sum_mn = 0.0;
	
	// Control gains (Initially zero for safety)
	Kp_j.clear(), Kv_j.clear();						// Joint space control gain
	Kp_w.clear(), Kv_w.clear();						// Work space position control gain
	Kp_a.clear(), Kv_a.clear();						// Work space posture control gain
	Kp_cog.clear(), Kv_cog.clear();					// COG control gain
	Kv_null.clear();								// Null space dumping gain
	Kp_phi = 0.0; Kv_phi = 0.0;						// Yaw motion control gain
	Kp_rb = 0.0;  Kv_rb = 0.0;  Ki_rb = 0.0;		// Robot position control gain
	Kp_rb2 = 0.0; Kv_rb2 = 0.0; Ki_rb2 = 0.0;		// Robot position control gain (method2)
	e_xrb_0 = 0.0;									// Integral of robot position error
	p.clear();										// Pole place for state feedback control of robot position and COG position
	g_dob.clear(); g_wob.clear(); g_rtob.clear();	// DOB gain and derivative gain
	g_dq.clear(); g_ddq.clear();					// WOB gain
	
	// Variables used in offset estimation
	q0_off_hat = 0.0; dq0_off_hat = 0.0;	// Offset value of q0 (To compensate gyro offset and drift)
	q0_off_dis = 0.0;						// Offset value of q0 (Disturbance in simulation and experiment)
	tau0_dis_hat_int = 0.0;					// Integral of disturbance torque at link 0
	
	// Flags
	seq = 0;
	seq_comp = 0;
	mst_cnt = 0;
	data_cnt = 0;
	stick_cnt = 0;
	gyro_set_flag = 0;
	emergency_flag = 0;
	workspace_flag = 0;
	dob_flag = 0;
	pado_flag = 0;
	wob_flag = 0;
	fc_flag = 0;
	output_flag = 0;
	for(i=0; i<8; i++) srv_on_flag[i] = 0;
	for(i=0; i<6; i++) psc_data[i] = 0xff;
}

#endif
