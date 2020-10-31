/**************************************************************************************************/
/*                                                                                                */
/* FILE : nzmn_move_by_cog.h                                                                      */
/* MEMO : Stabilization Control with Nozamani in 3D space (Only COG control is targeted)          */
/*        Try stabilization with COG command fed back robot position                              */
/* SEQUENCE :                                                                                     */
/*   0th : Join space PD control to the origin                                                    */
/*   1st : Moving to Initial Position and stay there with Joint space PD control                  */
/*   2nd : Switch to the joint space controller to work space controller                          */
/*   3rd : Joint space PD control along the path to go back the origin                            */
/*   4th : Joint space PD control to the origin and stabilization                                 */
/*                                                                                                */
/* 2012/07/17 : Start to edit this file                                                           */
/* 2012/08/26 : Add command from ps controller                                                    */
/* 2012/09/11 : Add end-effector posture control into workspace control                           */
/* 2012/12/05 :                                                                                   */
/* 2012/12/10 :                                                                                   */
/* 2013/01/23 :                                                                                   */
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_CONTROL
#define NZMN_CONTROL

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include "../exe_opt.h"
#include "../_lib/basic_calc.h"
#include "../_lib/basic_matrix.h"
#include "../_nzmn/nzmn_model.h"

#ifdef EXP
	#include "../_nzmn/nzmn_io.h"
	#include <rtai.h>
	#include <rtai_lxrt.h>
	#include <rtai_serial.h>
#endif

#include "../_nzmn/common_function.h"

//================================================================================================//
// Function declaration                                                                           //
//================================================================================================//
void init_variables_special(void);
void reload_nom_model(void);
void gen_xg_cmd_ref(int i_flag);
void reload_cmd_ref(void);
void reload_torque(void);
void apply_nis(void);
void apply_dis(void);
void post_processing(void);
void disp(void);

//================================================================================================//
// Constants and Variables                                                                        //
//================================================================================================//
// Select one of three methods 1:COGpath1, 2:COGpath2, 3:xg=0, 4:modern_control
//#define METHOD1
//#define METHOD2
//#define METHOD3
//#define METHOD4
#define METHOD5

//#define MANIPULATOR_OFF

mat99 Atorj, Atorw;
vec9 Btorj, Btorw;

//================================================================================================//
// Initialize various variables                                                                   //
//================================================================================================//
void init_variables_special(void)
{
	int i, j;
	double fn, zeta, Kp, Kv, g1, g2;
	
	// Initial response
	q_org.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	q_init.set(0.0, 0.0, -0.592557, 1.45122, 1.35057, -0.0, 0.0, 0.0, 0.0);			// Determined by Mathematica as the manipulator's COG is on the center of robot body
//	q_init.set((M_PI/180.0)*(-2.0), 0.0, (M_PI/180.0)*(-15.0), (M_PI/180.0)*135.0, (M_PI/180.0)*(-30.0), 0.0, 0.0, 0.0, 0.0);			// Determined heuristically
	q_res = q_org;
	
	// Initialize nominal parameters
	mn.set(38.4*1.0, 1.12, 1.35, 1.35, 0.82, 0.66, 0.34, 2.76, 2.76);
	mn = mn*1.0;
	Jmn.set(0.0, 0.081, 0.081, 0.081, 0.081, 0.015, 0.015, 0.47, 0.47);
	Ktn.set(1.0, 5.76, 5.76, 5.76, 5.76, 4.2, 4.2, 10.5, 10.5);
	Dn.set(0.0, 1.987, 2.314, 1.580, 1.381, 0.316, 0.421, 6.666, 8.843);
	Clmbn.set(0.0,  1.250,  1.320,  1.092, 0.974, 0.630, 0.432, 3.443, 2.629);
	Clmbn.set(0.0,  1.250,  1.320,  1.092, 0.974, 0.630, 0.432, 3.443/2.0, 2.629/2.0);
	tau_offset.set(0.0, -0.172, -0.280, -0.051, 0.271, 1.5, 0.588, 0.0952, 0.137);
	sum_mn = mn[0] + mn[1] + mn[2] + mn[3] + mn[4] + mn[5] + mn[6];
	for(i=0; i<7; i++){
//		In[i].row_set(0, 0.0, 0.0, 0.0); In[i].row_set(1, 0.0, 0.0, 0.0); In[i].row_set(2, 0.0, 0.0, Jmn[i] + mn[i]*L[i]*L[i]);			// Conventional settings
		In[i].row_set(0, 0.0, 0.0, 0.0); In[i].row_set(1, 0.0, 0.0, 0.0); In[i].row_set(2, 0.0, 0.0, Jmn[i] + mn[i]*Lg[i]*Lg[i]);		// Current settings
	}
//	In[7].row_set(0, MW*RW*RW/4.0, 0.0, 0.0); In[7].row_set(1, 0.0, MW*RW*RW/4.0, 0.0); In[7].row_set(2, 0.0, 0.0, MW*RW*RW/2.0);		// Conventional settings
	In[7].row_set(0, MW*RW*RW/4.0, 0.0, 0.0); In[7].row_set(1, 0.0, MW*RW*RW/4.0, 0.0); In[7].row_set(2, 0.0, 0.0, JmW + MW*RW*RW/2.0);	// Current settings
//	In[7].row_set(0, 0.0, 0.0, 0.0); In[7].row_set(1, 0.0, 0.0, 0.0); In[7].row_set(2, 0.0, 0.0, JmW + MW*RW*RW/2.0);	// Current settings
	In[8] = In[7];
	
	// Reload model elements (Inertia matrix is the one at initial position)
	q_res = q_init;			// Set joint angle initial position
	reload_trans();			// Reload homogeneous transformation matrix
	reload_model();			// Reload kinematics and dynamics real element in simulation with the original position
	reload_nom_model();		// Reload kinematics and dynamics nominal element with the original position
	reload_nom_dynamics();	// Reload the nominal dynamics elements (inertia matrix)
	xee_init = xee_res;		// Set initial end-effector position corresponding to q_init
	a_init = a_res;			// Set initial end-effector posture corresponding to q_init
	q_res = q_org;			// Reset joint angle to original position
	reload_nom_model();		// Reload kinematics and dynamics element with the original position
	
	// Nominal inertia for RTOB
	Mn_rtob = Mn;
	
	// Nominal inertia modification
	for(i=0; i<9; i++){
		for(j=0; j<9; j++){
			if(i!=j && 1<=i && i<=6)		Mn.A[i][j] = 0.0;
			else if(i!=j && 1<=j && j<=6)	Mn.A[i][j] = 0.0;
		}
	}
	
	// Initialize controller parameters
	fn = 5.0; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
	Kp_j.set(0.0, Kp, Kp, Kp, Kp, Kp, Kp, Kp, Kp);  Kv_j.set(0.0, Kv, Kv, Kv, Kv, Kv, Kv, Kv, Kv);
	
//	fn = 4.5; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
//	fn = 3.0; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
	fn = 2.0; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
	Kp_w.set(Kp, Kp, Kp);  Kv_w.set(Kv, Kv, Kv);
	Kp_a.set(Kp, Kp, Kp);  Kv_a.set(Kv, Kv, Kv);
	Kv_null.set(0.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 0.0, 0.0);
	
	// Robot position and attitude cotnroller parameters
	#ifdef METHOD1
		fn = 2.0; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
		Kp_cog.set(Kp, Kp, Kp);  Kv_cog.set(Kv, Kv, Kv);
		Kp_rb = 2.8; Kv_rb = 0.7; Ki_rb = 0.0;  e_xrb_0 = 0.0;
	#elif defined METHOD2
		fn = 2.3; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
		Kp_cog.set(Kp, Kp, Kp);  Kv_cog.set(Kv, Kv, Kv);
		Kp_rb2 = 0.2; Kv_rb2 = 0.4; Ki_rb2 = 0.00003; e_xrb_0 = 0.0;
	#elif defined METHOD3
		fn = 2.3; zeta = 1.0; Kp = pow((2.0*M_PI*fn), 2.0); Kv = 2.0*zeta*(2.0*M_PI*fn);
		Kp_cog.set(Kp, Kp, Kp);  Kv_cog.set(Kv, Kv, Kv);
	#elif defined METHOD4
//		g1 = -4.0;		// 安定するが少し定常振動大きい（DOB付きで±2cmくらい）
		g1 = -4.2;		// 少し振動的になる（DOB付きで±2cm弱くらい　→　TCCでの外乱の入れ方変更後±1.1cmくらい)
//		g1 = -4.5;		// 振動的で共振っぽくなる（リミットかかって止まる） DOB入れると安定する(g1=18.0時)
//		g1 = -5.0;		// 振動的で共振っぽくなる（リミットかかって止まる） DOB入れると安定する(g1=18.0時)が、外乱または移動指令を与えると発散する
		pp.set(g1, g1, g1, g1);
		mg = mn[0] + mn[1] + mn[2] + mn[3] + mn[4] + mn[5] + mn[6];
		mw = mn[7] + mn[8];
		lgg = sqrt(xg_res[0]*xg_res[0] + xg_res[2]*xg_res[2]);
		Mnaa = (mw + mg)*RW*RW + 2.0*(JmW + mw*RW*RW/2.0);
		Mnau = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnua = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnuu = mw*RW*RW + mg*(RW*RW + 2.0*RW*lgg + lgg*lgg) + mg*lgg*lgg;
		detMn = Mnaa*Mnuu - Mnau*Mnua;
		KK.x[0] = -(detMn*pp[0]*pp[1]*pp[2]*pp[3]*RW)/(g*lgg*mg);
		KK.x[1] = -((g*lgg*mg*(detMn*(pp[2]*pp[3]+pp[1]*pp[3]+pp[0]*pp[3]+pp[1]*pp[2]+pp[0]*pp[2]+pp[0]*pp[1]))+detMn*Mnuu*pp[0]*pp[1]*pp[2]*pp[3]+g*g*lgg*lgg*mg*mg*Mnaa)*RW)/(g*lgg*mg*Mnua);
		KK.x[2] = (detMn*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2])*RW)/(g*lgg*mg);
		KK.x[3] = ((g*lgg*mg*detMn*(pp[3]+pp[2]+pp[1]+pp[0])+detMn*Mnuu*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2]))*RW)/(g*lgg*mg*Mnua);
	#elif defined METHOD5
		g1 = -4.2;		// 少し振動的になる（DOB付きで±2cm弱くらい　→　TCCでの外乱の入れ方変更後±1.1cmくらい)
		pp.set(g1, g1, g1, g1);
		mg = mn[0] + mn[1] + mn[2] + mn[3] + mn[4] + mn[5] + mn[6];
		mw = mn[7] + mn[8];
		lgg = sqrt(xg_res[0]*xg_res[0] + xg_res[2]*xg_res[2]);
		Mnaa = (mw + mg)*RW*RW + 2.0*(JmW + mw*RW*RW/2.0);
		Mnau = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnua = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnuu = mw*RW*RW + mg*(RW*RW + 2.0*RW*lgg + lgg*lgg) + mg*lgg*lgg;
		detMn = Mnaa*Mnuu - Mnau*Mnua;
		KK.x[0] = -(detMn*pp[0]*pp[1]*pp[2]*pp[3]*RW)/(g*lgg*mg);
		KK.x[1] = -((g*lgg*mg*(detMn*(pp[2]*pp[3]+pp[1]*pp[3]+pp[0]*pp[3]+pp[1]*pp[2]+pp[0]*pp[2]+pp[0]*pp[1]))+detMn*Mnuu*pp[0]*pp[1]*pp[2]*pp[3]+g*g*lgg*lgg*mg*mg*Mnaa)*RW)/(g*lgg*mg*Mnua);
		KK.x[2] = (detMn*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2])*RW)/(g*lgg*mg);
		KK.x[3] = ((g*lgg*mg*detMn*(pp[3]+pp[2]+pp[1]+pp[0])+detMn*Mnuu*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2]))*RW)/(g*lgg*mg*Mnua);
	#endif
	fn = 4.0; zeta = 1.0;
	Kp_phi = pow((2.0*M_PI*fn), 2.0); Kv_phi = 2.0*zeta*(2.0*M_PI*fn);
	
	// Gains for disturbance observer
	g1 = 2.0*M_PI*2.0; g2 = 2.0*M_PI*15.0;
//	g1 = 2.0*M_PI*3.0; g2 = 2.0*M_PI*15.0;
//	g1 = 2.0*M_PI*15.0; g2 = 2.0*M_PI*15.0;
	g_dob.set(g1, g2, g2, g2, g2, g2, g2, g1, g1);
	
	g1 = 2.0*M_PI*1.0; g2 = 2.0*M_PI*2.0;
	g_rtob.set(g1, g2, g2, g2, g2, g2, g2, g1, g1);
	
	// Gains for pseudo derivative
	g1 = 2.0*M_PI*20.0; g2 = 2.0*M_PI*20.0;				// Normal
//	g1 = 2.0*M_PI*50.0; g2 = 2.0*M_PI*50.0;				// Allow high frequency element (Noisy)
	g_dq.set(g2, g2, g2, g2, g2, g2, g2, g2, g2);
	g_ddq.set(g2, g2, g2, g2, g2, g2, g2, g2, g2);
	for(i=0; i<8; i++) g_enc[i] = g2;
	
	// Gain for gyro sensor offset estimation
	g_p_off = 0.2, g_i_off = 0.01;						// Cut off angular frequency is 0.1 [rad/s]
//	g_p_off = 0.1, g_i_off = 0.0025;					// Cut off angular frequency is 0.05 [rad/s]
	
	// Flags
	#ifdef DOB
		dob_flag = 1;
	#endif
	#ifdef PADO
		pado_flag = 1;
	#endif
	#ifdef WOB
		wob_flag = 1;
	#endif
	#ifdef RTOB
		rtob_flag = 1;
	#endif
	#ifdef FRIC_COMP
		fc_flag = 1;
	#endif
	#ifdef DATA
		output_flag = 1;
	#endif
	for(i=0; i<8; i++) srv_on_flag[i] = 1;	// Servo on flag is set from starting this program
}

//================================================================================================//
// Reload nominal model                                                                           //
//================================================================================================//
void reload_nom_model(void)
{
	int i;
	vec9 dq, ddq;	// Joint acceleration in motion equation
	vec9 mass;		// Mass of each link
	vec9 tau_int;	// Internal(?) torque
	mat33 I[9];		// Inertia tensor of each link
	
	// Reload kinematic model --------------------------------------------------------------------//
	reload_trans();
	reload_kinematics_ee();			// Kinematics from joint angle to end-effector position
	reload_kinematics_cog(mn);		// Kinematics from joint angle to COG position
	reload_kinematics_rb();			// Kinematics from joint angle to robot position
	
	// Reload dynamic model ----------------------------------------------------------------------//
	// Internal torque
	mass = mn;
	for(i=0; i<9; i++) I[i] = In[i];
	dq = dq_res;
	ddq.clear();
	tau_int = newton_euler(dq, ddq, mass, I);
	
	// Gravity term and Centrifugal and Corioli term
	dq.clear();
	ddq.clear();
	Gn = newton_euler(dq, ddq, mass, I);
	Hn = tau_int - Gn;
}

//================================================================================================//
// Generate COG command and reference for                                                         //
//================================================================================================//
void gen_xg_cmd_ref(int i_flag)
{
	xg_cmd.clear(); dxg_cmd.clear(); ddxg_cmd.clear();
	if(i_flag == 1) e_xrb_0 += (R_xrb_cmd[0] - R_xrb_res[0]);
	
	#ifdef METHOD1
		xg_cmd.x[0] = (1.0/g)*(R_ddxrb_cmd[0] + Kp_rb*(R_xrb_cmd[0] - R_xrb_res[0]) + Kv_rb*(R_dxrb_cmd[0] - R_dxrb_res[0]) + Ki_rb*e_xrb_0);
		dxg_cmd.x[0] = 0.0;
	#elif defined METHOD2
		xg_cmd.x[0] = Kp_rb2*(R_xrb_cmd[0] - R_xrb_res[0]) + Ki_rb2*e_xrb_0;
		dxg_cmd.x[0] = Kv_rb2*(R_dxrb_cmd[0] - R_dxrb_res[0]);
	#elif defined METHOD3
		xg_cmd.x[0] = 0.0;
		dxg_cmd.x[0] = 0.0;
	#endif
	
	ddxg_cmd.x[0] = 0.0;
	ddxg_ref = ddxg_cmd + Kp_cog.diag()*(xg_cmd - xg_res) + Kv_cog.diag()*(dxg_cmd - dxg_res);
}

//================================================================================================//
// Reload Command and Reference Value                                                             //
//================================================================================================//
void reload_cmd_ref(void)
{
	static int init_flag = 0;
	static double tt = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t_cmd = 0.0;
	static vec9 q_tmp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec9 q_dst(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec3 xrb_tmp(0.0, 0.0, 0.0);
	static double xrb = 0.0, dxrb = 0.0, ddxrb = 0.0, dir = 0.0;
	
	#ifdef SIM
		// If sequence completion flag is set, automatically increment sequence
		if(seq_comp == 1 && seq < 3){
			seq++;
			seq_comp = 0;
			printf("Go ahead the sequence. Sequence : %d \n", seq);
		}
	#endif
	
	// Reflect psc_data to command data
	if(((unsigned char)psc_data[0] | (unsigned char)~(1<<PSC_UP)) != 0xff){
		R_ddxrb_cmd.x[0] = 0.0;
		R_dxrb_cmd.x[0] += 0.0005;
		if(R_dxrb_cmd[0] > 0.15) R_dxrb_cmd.x[0] = 0.15;
		R_xrb_cmd.x[0] += dt*R_dxrb_cmd[0];
	}else if(((unsigned char)psc_data[0] | (unsigned char)~(1<<PSC_DOWN)) != 0xff){
		R_ddxrb_cmd.x[0] = 0.0;
		R_dxrb_cmd.x[0] -= 0.0005;
		if(R_dxrb_cmd[0] < -0.15) R_dxrb_cmd.x[0] = -0.15;
		R_xrb_cmd.x[0] += dt*R_dxrb_cmd[0];
	}else{
		R_ddxrb_cmd.x[0] = 0.0;
		if(R_dxrb_cmd[0] > 0.0)      R_dxrb_cmd.x[0] -= 0.0005;
		else if(R_dxrb_cmd[0] < 0.0) R_dxrb_cmd.x[0] += 0.0005;
		else                         R_dxrb_cmd.x[0] = 0.0;
		R_xrb_cmd.x[0] += dt*R_dxrb_cmd[0];
	}
	
	if(((unsigned char)psc_data[0] | (unsigned char)~(1<<PSC_LEFT)) != 0xff){
		R_dxrb_cmd.x[2] = 0.2;
		R_xrb_cmd.x[2] += dt*R_dxrb_cmd[2];
	}else if(((unsigned char)psc_data[0] | (unsigned char)~(1<<PSC_RIGHT)) != 0xff){
		R_dxrb_cmd.x[2] = -0.2;
		R_xrb_cmd.x[2] += dt*R_dxrb_cmd[2];
	}
	
	// Estimate gyro sensor offset
	// Now coding
	
	switch(seq){
		// 0th Procedure -------------------------------------------------------------------------//
		// Join space PD control to the origin ---------------------------------------------------//
		case 0:
			// Flag setting
			seq_comp = 1;
			init_flag = 1;
			
			// Command and Reference for manipulator
			workspace_flag = 0;
			q_cmd.clear(); dq_cmd.clear(); ddq_cmd.clear();
			dq_cmd.clear(); ddq_cmd.clear();
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
//			R_xrb_cmd.clear(); R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
//			xrb_cmd.clear(); dxrb_cmd.clear(); ddxrb_cmd.clear();
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// 1st Procedure -------------------------------------------------------------------------//
		// Moving to Initial Position and stay there with Joint space PD control -----------------//
		case 1:
			// Initializing process
			if(init_flag == 1 && seq_comp == 0){
				init_flag = 0;
				tt = 0.0;
				q_tmp = q_res;
				t_cmd = 3.0;
				q_dst = q_init;
			}
			
			// Timer control
			if(tt < t_cmd){
				tt += dt;
			}else{
				seq_comp = 1;		// Set sequence completion flag
				init_flag = 1;			// Set initialization flag for next sequence
			}
			
			// Command and Reference for manipulator part controller
			workspace_flag = 0;
			q_cmd = 0.5*(q_dst - q_tmp)*(1.0 - cos(M_PI*tt/t_cmd)) + q_tmp;
			dq_cmd = 0.5*(q_dst - q_tmp)*(M_PI/t_cmd)*sin(M_PI*tt/t_cmd);
			ddq_cmd = (seq_comp == 0) ? 0.5*(q_dst - q_tmp)*((M_PI*M_PI)/(t_cmd*t_cmd))*cos(M_PI*tt/t_cmd) : null_vec9;
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
			R_xrb_cmd.clear(); R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
			xrb_cmd.clear(); dxrb_cmd.clear(); ddxrb_cmd.clear();
			ddxrb_ref.x[2] = ddxrb_cmd[2] + Kp_phi*(xrb_cmd[2] - xrb_res[2]) + Kv_phi*(dxrb_cmd[2] - dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// 2nd Procedure -------------------------------------------------------------------------//
		// Switch to the joint space controller to work space controller -------------------------//
		case 2:
			// Flag settings
			// Initializing process
			if(init_flag == 1 && seq_comp == 0){
				init_flag = 0;
				tt = 0.0;
				t_cmd = 1.0;
				xee_cmd = xee_res; dxee_cmd.clear(); ddxee_cmd.clear();
				a_cmd = a_res;   da_cmd.clear();  dda_cmd.clear();
			}
			
			// Timer control
			if(tt < t_cmd){
				tt += dt;
			}else{
				seq_comp = 1;
				init_flag = 1;
			}
			
			// Command and Reference for end-effector position controller
			#ifdef WORKSPACE_CONTROL
				workspace_flag = 1;
				ddxee_ref = ddxee_cmd + Kp_w.diag()*(xee_cmd - xee_res) + Kv_w.diag()*(dxee_cmd - dxee_res);
				dda_ref = dda_cmd + Kp_a.diag()*(a_cmd - a_res) + Kv_a.diag()*(da_cmd - da_res);
			#else
				workspace_flag = 0;
				q_cmd = q_init;  dq_cmd.clear();  ddq_cmd .clear();
				ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			#endif
			
			// Robot position command -> COG command and reference
			R_xrb_cmd.clear(); R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
			xrb_cmd.clear(); dxrb_cmd.clear(); ddxrb_cmd.clear();
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// 3rd Procedure -------------------------------------------------------------------------//
		// Move the mobile platform along the commanded path  ------------------------------------//
		case 3:
			// Initializing process
			if(init_flag == 1 && seq_comp == 0){
				init_flag = 0;
				tt = 0.0;
				xrb_tmp = R_xrb_cmd;
				xrb = 1.0;
				dir = ((xrb - xrb_tmp[0]) >= 0) ? 1.0: -1.0;
				dxrb = dir*0.1;
				ddxrb = dir*0.05;
				if((xrb - xrb_tmp[0])*ddxrb > dxrb*dxrb){ t1 = dxrb/ddxrb;                      t2 = (xrb - xrb_tmp[0])/dxrb;  t3 = t1 + t2; }
				else{                                     t1 = sqrt((xrb - xrb_tmp[0])/ddxrb);  t2 = t1;                       t3 = 2.0*t1;  }
				t4 = t3 + 1.0;
				
				xee_cmd = xee_cmd; dxee_cmd.clear(); ddxee_cmd.clear();
				a_cmd = a_cmd;   da_cmd.clear();  dda_cmd.clear();
			}
			
			// Command and Reference for end-effector position controller
			#ifdef WORKSPACE_CONTROL
				workspace_flag = 1;
				ddxee_ref = ddxee_cmd + Kp_w.diag()*(xee_cmd - xee_res) + Kv_w.diag()*(dxee_cmd - dxee_res);
				dda_ref = dda_cmd + Kp_a.diag()*(a_cmd - a_res) + Kv_a.diag()*(da_cmd - da_res);
			#else
				workspace_flag = 0;
				q_cmd = q_init; dq_cmd.clear(); ddq_cmd.clear();
				ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			#endif
			
			// Command and Reference for robot position controller (Constant accl -> Constant vel -> Constant accl)
			R_xrb_cmd.clear(); R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
			if((tt+=dt) < t1){ R_xrb_cmd.x[0] = (ddxrb/2.0)*tt*tt + xrb_tmp[0];            R_dxrb_cmd.x[0] = ddxrb*tt;                  R_ddxrb_cmd.x[0] = ddxrb;  }
			else if(tt < t2){  R_xrb_cmd.x[0] = dxrb*t1/2.0 + dxrb*(tt - t1) + xrb_tmp[0]; R_dxrb_cmd.x[0] = dxrb;                      R_ddxrb_cmd.x[0] = 0.0;    }
			else if(tt < t3){  R_xrb_cmd.x[0] = xrb - (ddxrb/2.0)*(tt - t3)*(tt - t3);     R_dxrb_cmd.x[0] = ddxrb*t1 - ddxrb*(tt-t2);  R_ddxrb_cmd.x[0] = -ddxrb; }
			else if(tt < t4){  R_xrb_cmd.x[0] = xrb; R_dxrb_cmd.x[0] = 0.0; R_ddxrb_cmd.x[0] = 0.0; }
			else{              R_xrb_cmd.x[0] = xrb; R_dxrb_cmd.x[0] = 0.0; R_ddxrb_cmd.x[0] = 0.0; seq_comp=1; init_flag = 1; }
			xrb_cmd = R_xrb_cmd; dxrb_cmd = R_dxrb_cmd; ddxrb_cmd = R_ddxrb_cmd;
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			
			// COG command and reference
			if(seq_comp == 1) gen_xg_cmd_ref(1);
			else              gen_xg_cmd_ref(0);
			break;
		
		// 4th Procedure -------------------------------------------------------------------------//
		// Joint space PD control along the path to go back the origin ---------------------------//
		case 4:
			// Initializing process
			if(init_flag == 1 && seq_comp == 0){
				init_flag = 0;
				tt = 0.0;
				t_cmd = 3.0;
				q_tmp = q_res;
				q_dst = q_org;
			}
			
			// Timer and Sequence control
			if(tt < t_cmd){
				tt += dt;
			}else{
				seq_comp = 1;
				init_flag = 1;
			}
			
			// Command and Reference for manipulator part controller
			workspace_flag = 0;
			q_cmd = 0.5*(q_dst - q_tmp)*(1.0 - cos(M_PI*tt/t_cmd)) + q_tmp;
			dq_cmd = 0.5*(q_dst - q_tmp)*(M_PI/t_cmd)*sin(M_PI*tt/t_cmd);
			ddq_cmd = (seq_comp == 0) ? 0.5*(q_dst - q_tmp)*((M_PI*M_PI)/(t_cmd*t_cmd))*cos(M_PI*tt/t_cmd) : null_vec9;
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
//			R_xrb_cmd = R_xrb_cmd; R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
//			xrb_cmd = R_xrb_cmd; dxrb_cmd = R_dxrb_cmd; ddxrb_cmd = R_ddxrb_cmd;
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// 5th Procedure -------------------------------------------------------------------------//
		// Joint space PD control to the origin and stabilization --------------------------------//
		case 5:
			// Command and Reference for manipulator part controller
			workspace_flag = 0;
			q_cmd = q_org;
			dq_cmd.clear();
			ddq_cmd.clear();
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
//			R_xrb_cmd = R_xrb_cmd; R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
//			xrb_cmd = R_xrb_cmd; dxrb_cmd = R_dxrb_cmd; ddxrb_cmd = R_ddxrb_cmd;
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// Extra Procedure for emergency ---------------------------------------------------------//
		// Joint space PD control along the path to go back the origin ---------------------------//
		case 100:
			// Initializing process
			if(init_flag == 1 && seq_comp == 0){
				init_flag = 0;
				tt = 0.0;
				t_cmd = 3.0;
				q_tmp = q_res;
				q_dst = q_org;
			}
			
			// Timer and Sequence control
			if(tt < t_cmd){
				tt += dt;
			}else{
				seq_comp = 1;
				init_flag = 1;
			}
			
			// Command and Reference for manipulator part controller
			workspace_flag = 0;
			q_cmd = 0.5*(q_dst - q_tmp)*(1.0 - cos(M_PI*tt/t_cmd)) + q_tmp;
			dq_cmd = 0.5*(q_dst - q_tmp)*(M_PI/t_cmd)*sin(M_PI*tt/t_cmd);
			ddq_cmd = (seq_comp == 0) ? 0.5*(q_dst - q_tmp)*((M_PI*M_PI)/(t_cmd*t_cmd))*cos(M_PI*tt/t_cmd) : null_vec9;
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
			R_xrb_cmd = R_xrb_cmd; R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
			xrb_cmd = R_xrb_cmd; dxrb_cmd = R_dxrb_cmd; ddxrb_cmd = R_ddxrb_cmd;
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		// Extra Procedure for emergency ---------------------------------------------------------//
		// Joint space PD control to the origin and stabilization --------------------------------//
		case 101:
			// Command and Reference for manipulator part controller
			workspace_flag = 0;
			q_cmd = q_org;
			dq_cmd.clear();
			ddq_cmd.clear();
			ddq_ref = ddq_cmd + Kp_j.diag()*(q_cmd - q_res) + Kv_j.diag()*(dq_cmd - dq_res);
			
			// Robot position command -> COG command and reference
			R_xrb_cmd = R_xrb_cmd; R_dxrb_cmd.clear(); R_ddxrb_cmd.clear();
			xrb_cmd = R_xrb_cmd; dxrb_cmd = R_dxrb_cmd; ddxrb_cmd = R_ddxrb_cmd;
			ddxrb_ref.x[2] = R_ddxrb_cmd[2] + Kp_phi*(R_xrb_cmd[2] - R_xrb_res[2]) + Kv_phi*(R_dxrb_cmd[2] - R_dxrb_res[2]);
			gen_xg_cmd_ref(1);
			break;
		
		default :
			emergency_flag = 1;
			break;
	}
}

//================================================================================================//
// Reload Torque Input Value for Main Task                                                        //
//================================================================================================//
void reload_torque(void)
{
	// Declaration of variables used in control argorithm ----------------------------------------//
	int i;
	vec9 tau_dis;
	
	// Select if we use estimated disturbance torque in controller
	if(dob_flag == 1) tau_dis = tau_dis_hat;
	else              tau_dis.clear();
//	tau_dis_hat.x[7] *= 0.8;
//	tau_dis_hat.x[8] *= 0.8;
	
	// Select if we use estimated pitch angle disturbance torque in controller
	if(pado_flag == 1) tau_dis.x[0] = tau_dis_hat[0];
//	if(pado_flag == 1) tau_dis.x[0] = tau_dis_hat[0]*0.5;
	else               tau_dis.x[0] = 0.0;
	
	// Calculate compensation torque
	if(fc_flag == 1){
		tau_fric_n.clear();
		for(i=1; i<9; i++){
			tau_fric_n.x[i] = Clmbn[i]*sign(dq_res[i]);
		}
		tau_fric_n = tau_fric_n + Dn.diag()*dq_res;
	}else{
		tau_fric_n.clear();
	}
	
	// Reload feedback gain for modern control
	#ifdef METHOD4
		lgg = sqrt(xg_res[0]*xg_res[0] + xg_res[2]*xg_res[2]);
		Mnaa = (mw + mg)*RW*RW + 2.0*(JmW + mw*RW*RW/2.0);  Mnau = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnua = mw*RW*RW + mg*(RW*RW + RW*lgg);              Mnuu = mw*RW*RW + mg*(RW*RW + 2.0*RW*lgg + lgg*lgg) + mg*lgg*lgg;
		detMn = Mnaa*Mnuu - Mnau*Mnua;
		
		KK.x[0] = -(detMn*pp[0]*pp[1]*pp[2]*pp[3]*RW)/(g*lgg*mg);
		KK.x[1] = -((g*lgg*mg*(detMn*(pp[2]*pp[3]+pp[1]*pp[3]+pp[0]*pp[3]+pp[1]*pp[2]+pp[0]*pp[2]+pp[0]*pp[1]))+detMn*Mnuu*pp[0]*pp[1]*pp[2]*pp[3]+g*g*lgg*lgg*mg*mg*Mnaa)*RW)/(g*lgg*mg*Mnua);
		KK.x[2] = (detMn*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2])*RW)/(g*lgg*mg);
		KK.x[3] = ((g*lgg*mg*detMn*(pp[3]+pp[2]+pp[1]+pp[0])+detMn*Mnuu*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2]))*RW)/(g*lgg*mg*Mnua);
		
		xx.x[0] = (R_xrb_res[0] - R_xrb_cmd[0])/RW;
//		xx.x[0] = (R_xrb_res[0] - RW*q_res[0] - R_xrb_cmd[0])/RW;			// Kotti no houga genmitsuna kiga surukedo jikken kekka yokunai
		xx.x[1] = asin(xg_res[0]/lgg);
		xx.x[2] = (R_dxrb_res[0] - R_dxrb_cmd[0])/RW;
//		xx.x[2] = (R_dxrb_res[0] - RW*dq_res[0] - R_dxrb_cmd[0])/RW;		// Kotti no houga genmitsuna kiga surukedo jikken kekka yokunai
		xx.x[3] = 1.0/sqrt(1.0 - pow(xg_res[0]/lgg, 2.0)) * dxg_res[0]/lgg;
	#elif defined METHOD5
		lgg = sqrt(xg_res[0]*xg_res[0] + xg_res[2]*xg_res[2]);
		Mnaa = (mw + mg)*RW*RW + 2.0*(JmW + mw*RW*RW/2.0);  Mnau = mw*RW*RW + mg*(RW*RW + RW*lgg);
		Mnua = mw*RW*RW + mg*(RW*RW + RW*lgg);              Mnuu = mw*RW*RW + mg*(RW*RW + 2.0*RW*lgg + lgg*lgg) + mg*lgg*lgg;
		detMn = Mnaa*Mnuu - Mnau*Mnua;
		
		KK.x[0] = -(detMn*pp[0]*pp[1]*pp[2]*pp[3]*RW)/(g*lgg*mg);
		KK.x[1] = -((g*lgg*mg*(detMn*(pp[2]*pp[3]+pp[1]*pp[3]+pp[0]*pp[3]+pp[1]*pp[2]+pp[0]*pp[2]+pp[0]*pp[1]))+detMn*Mnuu*pp[0]*pp[1]*pp[2]*pp[3]+g*g*lgg*lgg*mg*mg*Mnaa)*RW)/(g*lgg*mg*Mnua);
		KK.x[2] = (detMn*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2])*RW)/(g*lgg*mg);
		KK.x[3] = ((g*lgg*mg*detMn*(pp[3]+pp[2]+pp[1]+pp[0])+detMn*Mnuu*(pp[1]*pp[2]*pp[3]+pp[0]*pp[2]*pp[3]+pp[0]*pp[1]*pp[3]+pp[0]*pp[1]*pp[2]))*RW)/(g*lgg*mg*Mnua);
		
		xx.x[0] = (R_xrb_res[0] - R_xrb_cmd[0])/RW;
//		xx.x[0] = (R_xrb_res[0] - RW*q_res[0] - R_xrb_cmd[0])/RW;			// Kotti no houga genmitsuna kiga surukedo jikken kekka yokunai
		xx.x[1] = asin(xg_res[0]/lgg);
		xx.x[2] = (R_dxrb_res[0] - R_dxrb_cmd[0])/RW;
//		xx.x[2] = (R_dxrb_res[0] - RW*dq_res[0] - R_dxrb_cmd[0])/RW;		// Kotti no houga genmitsuna kiga surukedo jikken kekka yokunai
		xx.x[3] = 1.0/sqrt(1.0 - pow(xg_res[0]/lgg, 2.0)) * dxg_res[0]/lgg;
	#endif
	
	// Joint space PD control of Manipulator part and COG stabilization control ------------------//
	if(workspace_flag == 0){
		// COG stabilization control and Joint space PD control of Manipulator part
		#ifdef METHOD1
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD2
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD3
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD4
			Atorj.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
		#elif defined METHOD5
			Atorj.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
		#endif
		Atorj.row_set(1, Jrb.row(2)*Mn_inv);		// Yaw motion control
		Atorj.row_set(2, Mn_inv.row(1));			// Manipulator's 1st joint control
		Atorj.row_set(3, Mn_inv.row(2));			// Manipulator's 2nd joint control
		Atorj.row_set(4, Mn_inv.row(3));			// Manipulator's 3rd joint control
		Atorj.row_set(5, Mn_inv.row(4));			// Manipulator's 4th joint control
		Atorj.row_set(6, Mn_inv.row(5));			// Manipulator's 5th joint control
		Atorj.row_set(7, Mn_inv.row(6));			// Manipulator's 6th joint control
		Atorj.row_set(8, unit9(0));					// Canceling passive joint torque
		
		#ifdef METHOD1
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD2
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD3
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD4
			Btorj.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);			// Modern control of robot position and COG position
		#elif defined METHOD5
			Btorj.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);			// Modern control of robot position and COG position
		#endif
		Btorj.x[1] = ddxrb_ref[2] + Jrb.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// Yaw motion control
		Btorj.x[2] = ddq_ref[1] + Mn_inv.row(1)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 1st joint control
		Btorj.x[3] = ddq_ref[2] + Mn_inv.row(2)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 2nd joint control
		Btorj.x[4] = ddq_ref[3] + Mn_inv.row(3)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 3rd joint control
		Btorj.x[5] = ddq_ref[4] + Mn_inv.row(4)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 4th joint control
		Btorj.x[6] = ddq_ref[5] + Mn_inv.row(5)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 5th joint control
		Btorj.x[7] = ddq_ref[6] + Mn_inv.row(6)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 6th joint control
		Btorj.x[8] = 0.0;																// Canceling passive joint torque
		
		tau_ref = Atorj.inv()*Btorj;
	}
	
	// Joint space PD control of Manipulator part and COG stabilization control ------------------//
	if(workspace_flag == 0){
		// COG stabilization control and Joint space PD control of Manipulator part
		#ifdef METHOD1
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD2
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD3
			Atorj.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD4
			Atorj.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
		#elif defined METHOD5
			Atorj.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
		#endif
		Atorj.row_set(1, Jrb.row(2)*Mn_inv);		// Yaw motion control
		Atorj.row_set(2, Mn_inv.row(1));			// Manipulator's 1st joint control
		Atorj.row_set(3, Mn_inv.row(2));			// Manipulator's 2nd joint control
		Atorj.row_set(4, Mn_inv.row(3));			// Manipulator's 3rd joint control
		Atorj.row_set(5, Mn_inv.row(4));			// Manipulator's 4th joint control
		Atorj.row_set(6, Mn_inv.row(5));			// Manipulator's 5th joint control
		Atorj.row_set(7, Mn_inv.row(6));			// Manipulator's 6th joint control
		Atorj.row_set(8, unit9(0));					// Canceling passive joint torque
		
		#ifdef METHOD1
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD2
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD3
			Btorj.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// COG control
		#elif defined METHOD4
			Btorj.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);			// Modern control of robot position and COG position
		#elif defined METHOD5
			Btorj.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);			// Modern control of robot position and COG position
		#endif
		Btorj.x[1] = ddxrb_ref[2] + Jrb.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// Yaw motion control
		Btorj.x[2] = ddq_ref[1] + Mn_inv.row(1)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 1st joint control
		Btorj.x[3] = ddq_ref[2] + Mn_inv.row(2)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 2nd joint control
		Btorj.x[4] = ddq_ref[3] + Mn_inv.row(3)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 3rd joint control
		Btorj.x[5] = ddq_ref[4] + Mn_inv.row(4)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 4th joint control
		Btorj.x[6] = ddq_ref[5] + Mn_inv.row(5)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 5th joint control
		Btorj.x[7] = ddq_ref[6] + Mn_inv.row(6)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 6th joint control
		Btorj.x[8] = 0.0;																// Canceling passive joint torque
		
		tau_ref = Atorj.inv()*Btorj;
	}
	
	// COG stabilization control and Work space end-effector PD control --------------------------//
	else{
		#ifdef METHOD1
			Atorw.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD2
			Atorw.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD3
			Atorw.row_set(0, Jg.row(0)*Mn_inv);		// COG control
		#elif defined METHOD4
			Atorw.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
		#endif
		Atorw.row_set(1, Jrb.row(2)*Mn_inv);	// Yaw motion control
		Atorw.row_set(2, Jee.row(0)*Mn_inv);	// End-effector position control (X-direction)
		Atorw.row_set(3, Jee.row(1)*Mn_inv);	// End-effector position control (Y-direction)
		Atorw.row_set(4, Jee.row(2)*Mn_inv);	// End-effector position control (Z-direction)
		Atorw.row_set(5, Ja.row(0)*Mn_inv);		// End-effector posture control (X-direction)
		Atorw.row_set(6, Ja.row(1)*Mn_inv);		// End-effector posture control (Y-direction)
		Atorw.row_set(7, Ja.row(2)*Mn_inv);		// End-effector posture control (Z-direction)
		Atorw.row_set(8, unit9(0));				// Canceling passive joint torque
		
		#ifdef METHOD1
			Btorw.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);	// COG control
		#elif defined METHOD2
			Btorw.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);	// COG control
		#elif defined METHOD3
			Btorw.x[0] = ddxg_ref[0] + Jg.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);	// COG control
		#elif defined METHOD4
			Btorw.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);							// Modern control of robot position and COG position
		#endif
		Btorw.x[1] = ddxrb_ref[2] + Jrb.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// Yaw motion control
		Btorw.x[2] = ddxee_ref[0] + Jee.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// End-effector position control (X-direction)
		Btorw.x[3] = ddxee_ref[1] + Jee.row(1)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// End-effector position control (Y-direction)
		Btorw.x[4] = ddxee_ref[2] + Jee.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// End-effector position control (Z-direction)
		Btorw.x[5] = dda_ref[0] + Ja.row(0)*Mn_inv*(Gn + tau_fric_n + tau_dis);			// End-effector posture control (X-direction)
		Btorw.x[6] = dda_ref[1] + Ja.row(1)*Mn_inv*(Gn + tau_fric_n + tau_dis);			// End-effector posture control (Y-direction)
		Btorw.x[7] = dda_ref[2] + Ja.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);			// End-effector posture control (Z-direction)
		Btorw.x[8] = 0.0;																// Canceling passive joint torque
		
		#ifdef METHOD5
			mat66 Jaco;
			vec6 ddqm_ref, ddxm_ref;
			
			Jaco.row_set(0, Jee.A[0][1], Jee.A[0][2], Jee.A[0][3], Jee.A[0][4], Jee.A[0][5], Jee.A[0][6]);
			Jaco.row_set(1, Jee.A[1][1], Jee.A[1][2], Jee.A[1][3], Jee.A[1][4], Jee.A[1][5], Jee.A[1][6]);
			Jaco.row_set(2, Jee.A[2][1], Jee.A[2][2], Jee.A[2][3], Jee.A[2][4], Jee.A[2][5], Jee.A[2][6]);
			Jaco.row_set(3, Ja.A[0][1], Ja.A[0][2], Ja.A[0][3], Ja.A[0][4], Ja.A[0][5], Ja.A[0][6]);
			Jaco.row_set(4, Ja.A[1][1], Ja.A[1][2], Ja.A[1][3], Ja.A[1][4], Ja.A[1][5], Ja.A[1][6]);
			Jaco.row_set(5, Ja.A[2][1], Ja.A[2][2], Ja.A[2][3], Ja.A[2][4], Ja.A[2][5], Ja.A[2][6]);
			ddxm_ref.set(ddxee_ref[0], ddxee_ref[1], ddxee_ref[2], dda_ref[0], dda_ref[1], dda_ref[2]);
			ddqm_ref = Jaco.inv()*ddxm_ref;
			
//			for(i=0; i<6; i++) tau_ref.x[i] = Mn.A[i][i]*ddqm_ref[i];
//			tau_ref.x[7] = 
//			tau_ref.x[8] = 
			
			Atorw.row_set(0, unit9(7) + unit9(8));	// Modern control of robot position and COG position
			Atorw.row_set(1, Jrb.row(2)*Mn_inv);	// Yaw motion control
			Atorw.row_set(2, Mn_inv.row(1));		// End-effector position control (X-direction)
			Atorw.row_set(3, Mn_inv.row(2));		// End-effector position control (Y-direction)
			Atorw.row_set(4, Mn_inv.row(3));		// End-effector position control (Z-direction)
			Atorw.row_set(5, Mn_inv.row(4));		// End-effector posture control (X-direction)
			Atorw.row_set(6, Mn_inv.row(5));		// End-effector posture control (Y-direction)
			Atorw.row_set(7, Mn_inv.row(6));		// End-effector posture control (Z-direction)
			Atorw.row_set(8, unit9(0));				// Canceling passive joint torque
			Btorw.x[0] = -KK*xx + (tau_dis[7] + tau_dis[8]);								// Modern control of robot position and COG position
			Btorw.x[1] = ddxrb_ref[2] + Jrb.row(2)*Mn_inv*(Gn + tau_fric_n + tau_dis);		// Yaw motion control
			Btorw.x[2] = ddqm_ref[0] + Mn_inv.row(1)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 1st joint control
			Btorw.x[3] = ddqm_ref[1] + Mn_inv.row(2)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 2nd joint control
			Btorw.x[4] = ddqm_ref[2] + Mn_inv.row(3)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 3rd joint control
			Btorw.x[5] = ddqm_ref[3] + Mn_inv.row(4)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 4th joint control
			Btorw.x[6] = ddqm_ref[4] + Mn_inv.row(5)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 5th joint control
			Btorw.x[7] = ddqm_ref[5] + Mn_inv.row(6)*(Gn + tau_fric_n + tau_dis);			// Manipulator's 6th joint control
			Btorw.x[8] = 0.0;																// Canceling passive joint torque
		#endif
		
		tau_ref = Atorw.inv()*Btorw;
	}
	
	torque_limit();
}

//================================================================================================//
// Apply noise caused by sensor resolution to each joint response                                 //
//================================================================================================//
void apply_nis(void)
{
	#ifdef NOISE
		int i;
		static vec9 dq_res1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 dq_res1_pre(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 dq_res2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 ddq_res(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 q_res_tmp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 dq_res_tmp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		static vec9 ddq_res_pre(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		vec9 rsl(65536/(2.0*M_PI), 400000/(2.0*M_PI), 400000/(2.0*M_PI), 400000/(2.0*M_PI), 400000/(2.0*M_PI), 400000/(2.0*M_PI), 400000/(2.0*M_PI), 200000/(2.0*M_PI), 200000/(2.0*M_PI));
		
		static vec9 q_res_pre(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);		// Previous joint angle
		static vec9 dq_res_pre(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);	// Previous joint velocity
		static double gyro_cnt = 0;							// Gyro sampling counter
		double gyro_smpt = (0.02-0.0000001);				// Sampling time of gyro
		static double q0_res_pre = 0.0, dq0_res_pre = 0.0;
		
		// Apply rounding caused by sensor resolution against each joint angle
		for(i=0; i<9; i++){
			if(q_res[i] > 0.0)	q_res.x[i] = ((int)(q_res.x[i]*rsl[i] + 0.5)) / rsl[i];
			else				q_res.x[i] = ((int)(-q_res.x[i]*rsl[i] + 0.5)) / (-rsl[i]);
		}
		
		// Apply gyro sampling lack (20ms)
		if((gyro_cnt+=dt) < gyro_smpt){
			q_res.x[0] = q0_res_pre;
			dq_res.x[0] = dq0_res_pre;
			gyro_set_flag = 0;
/*			if(dob_flag == 1) dq_res.x[0] += dt*Mn_inv.row(0)*(tau_ref - Gn - tau_dis_hat);
			else              dq_res.x[0] += dt*Mn_inv.row(0)*(tau_ref - Gn);
			q_res.x[0] += dt*dq_res.x[0];
*/		}else{
			gyro_set_flag = 1;
			gyro_cnt = 0.0;
			q_res.x[0] = q_res.x[0];
			dq_res.x[0] = dq_res.x[0];
			q0_res_pre = q_res[0];
			dq0_res_pre = dq_res[0];
		}
		// printf("gyro_cnt = %lf\n", gyro_cnt);
		// printf("dq_res_pre = "); dq_res_pre.disp();
		// printf("dq_res = "); dq_res.disp();
		// printf("q_res = "); q_res.disp();
		// printf("q_res_pre = "); q_res_pre.disp();
		// getchar();
		// q_res_pre.x[0] = q_res.x[0];
		// dq_res_pre.x[0] = dq_res.x[0];
	
		// Calculate dq_res by pseudo derivative of q_res
		for(i=1; i<9; i++){
			// Trapezoidal approximation
			// dq_res.x[i] = (2.0*g_dq[i])/(2+g_dq[i]*dt) * (q_res[i] - q_res_tmp[i] - (dt/2)*dq_res[i]);
			// q_res_tmp.x[i] = q_res_tmp[i] + (dt/2)*(dq_res_pre[i] + dq_res[i]);
			// dq_res_pre[i] = dq_res[i];
			
			// Rectangular approximation
			q_res_tmp.x[i] += dt*dq_res_pre[i];
			dq_res.x[i] = g_dq[i]*(q_res[i]-q_res_tmp[i]);
			
			// First order pseudo derivative + First order LPF
			// dq_res1_pre.x[i] = dq_res1[i];
			// q_res_tmp.x[i] = q_res_tmp[i] + (dt/2)*(dq_res1_pre[i] + dq_res1[i]);
			// dq_res1.x[i] = g_dq[i]*(q_res[i]-q_res_tmp[i]);
			// dq_res1.x[i] = (2.0*g_dq[i])/(2+g_dq[i]*dt) * (q_res[i] - q_res_tmp[i] - (dt/2)*dq_res1[i]);
			// dq_res2.x[i] = (2/(2+g_dq[i]*dt))*dq_res2[i] + (g*dt/(2+g_dq[i]*dt))*(dq_res1_pre[i] - dq_res2[i] + dq_res1[i]);
			// dq_res2.x[i] += g*dt*(dq_res1[i] - dq_res2[i]);
		}
		dq_res_pre = dq_res;
		
		// Calculate ddq_res by pseudo derivative of dq_res
		for(i=0; i<9; i++){
			// Rectangular approximation
			dq_res_tmp.x[i] += dt*ddq_res_pre[i];
			ddq_res.x[i] = g_ddq[i]*(dq_res[i]-dq_res_tmp[i]);
		}
		ddq_res_pre = ddq_res;
	#endif
}

//================================================================================================//
// Apply disturbance                                                                              //
//================================================================================================//
void apply_dis(void)
{
	vec9 tau_ext(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	vec3 f_ext(0.0, 0.0, 0.0);
	
	#ifdef FRICTION
		int i;
		vec9 D(0.0, 1.987, 2.314, 1.580, 1.381, 0.316, 0.421, 6.666, 8.843);
		vec9 Clmb(0.0,  1.250,  1.320,  1.092, 0.974, 0.630, 0.432, 3.443, 2.629);
//		tau_fric = Clmb.diag()*dq_res;
		 for(i=0; i<5; i++)	tau_fric.x[i] = D[i]*dq_res[i] + Clmb[i]*sign(dq_res[i]);
	#else
		tau_fric.set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	#endif
	
	#ifdef DISTURBANCE
		if(5.0 <= t && t<= 10.0){
			rtob_flag = 1;
			tau_ext.set(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0);
		}else{
			rtob_flag = 0;
			tau_ext.clear();
		}
	#else
		tau_ext.clear();
	#endif
	
	tau_dis = tau_fric + tau_ext;
}

//================================================================================================//
// Post processing after main control loop is finished                                            //
//================================================================================================//
void post_processing(void)
{
}

//================================================================================================//
// Display data when simulation                                                                   //
//================================================================================================//
void disp(void)
{
	#ifdef EXP
		if((int)(t/dt)%(int)(0.5/dt)==0){
			rt_printk("t = %lf, master_cnt = %d\n", t, mst_cnt);
			rt_printk("q_cmd    = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf [deg]\n", q_cmd[0]*180.0/M_PI, q_cmd[1]*180.0/M_PI, q_cmd[2]*180.0/M_PI, q_cmd[3]*180.0/M_PI, q_cmd[4]*180.0/M_PI, q_cmd[5]*180.0/M_PI, q_cmd[6]*180.0/M_PI, q_cmd[7]*180.0/M_PI, q_cmd[8]*180.0/M_PI);
			rt_printk("q_res    = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf [deg]\n", q_res[0]*180.0/M_PI, q_res[1]*180.0/M_PI, q_res[2]*180.0/M_PI, q_res[3]*180.0/M_PI, q_res[4]*180.0/M_PI, q_res[5]*180.0/M_PI, q_res[6]*180.0/M_PI, q_res[7]*180.0/M_PI, q_res[8]*180.0/M_PI);
//			rt_printk("dq_res   = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf [deg/s]\n", dq_res[0]*180.0/M_PI, dq_res[1]*180.0/M_PI, dq_res[2]*180.0/M_PI, dq_res[3]*180.0/M_PI, dq_res[4]*180.0/M_PI, dq_res[5]*180.0/M_PI, dq_res[6]*180.0/M_PI, dq_res[7]*180.0/M_PI, dq_res[8]*180.0/M_PI);
//			rt_printk("ddq_res  = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf [deg/s]\n", ddq_res[0]*180.0/M_PI, ddq_res[1]*180.0/M_PI, ddq_res[2]*180.0/M_PI, ddq_res[3]*180.0/M_PI, ddq_res[4]*180.0/M_PI);
//			rt_printk("ddq_ref  = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf [deg/s]\n", ddq_ref[0]*180.0/M_PI, ddq_ref[1]*180.0/M_PI, ddq_ref[2]*180.0/M_PI, ddq_ref[3]*180.0/M_PI, ddq_ref[4]*180.0/M_PI);
//			rt_printk("xee_cmd  = %.3lf, %.3lf [m]\n", xee_cmd[0], xee_cmd[1]);
			rt_printk("xee_res  = %.3lf, %.3lf, %.3lf [m]\n", xee_res[0], xee_res[1], xee_res[2]);
			rt_printk("xg_cmd   = %.3lf, %.3lf, %.3lf [m]\n", xg_cmd[0], xg_cmd[1], xg_cmd[2]);
			rt_printk("xg_res   = %.3lf, %.3lf, %.3lf [m]\n", xg_res[0], xg_res[1], xg_res[2]);
//			rt_printk("ddxg_ref = %.3lf, %.3lf [m]\n", ddxg_ref[0], ddxg_ref[1]);
			rt_printk("xrb_cmd = %.3lf[m], dxrb_cmd = %.3lf[m/s], xrb_cmd[2] = %.3lf[rad]\n", xrb_cmd[0], dxrb_cmd[0], xrb_cmd[2]);
			rt_printk("xrb_res = %.3lf[m], dxrb_res = %.3lf[m/s], xrb_res[2] = %.3lf[rad]\n", xrb_res[0], dxrb_res[0], xrb_res[2]);
			rt_printk("tau_ref      = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", tau_ref[0], tau_ref[1], tau_ref[2], tau_ref[3], tau_ref[4], tau_ref[5], tau_ref[6], tau_ref[7], tau_ref[8]);
			rt_printk("tau_dis_hat  = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", tau_dis_hat[0], tau_dis_hat[1], tau_dis_hat[2], tau_dis_hat[3], tau_dis_hat[4], tau_dis_hat[5], tau_dis_hat[6], tau_dis_hat[7], tau_dis_hat[8]);
			rt_printk("tau_reac_hat = %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf\n", tau_reac_hat[0], tau_reac_hat[1], tau_reac_hat[2], tau_reac_hat[3], tau_reac_hat[4], tau_reac_hat[5], tau_reac_hat[6], tau_reac_hat[7], tau_reac_hat[8]);
			rt_printk("Fee_reac_hat = %.3lf, %.3lf, %.3lf, Fa_reac_hat = %.3lf, %.3lf, %.3lf\n", Fee_reac_hat[0], Fee_reac_hat[1], Fee_reac_hat[2], Fa_reac_hat[0], Fa_reac_hat[1], Fa_reac_hat[2]);
//			rt_printk("xx = %.3lf, %.3lf, %.3lf, %.3lf\n", xx[0], xx[1], xx[2], xx[3]);
//			rt_printk("KK = %.3lf, %.3lf, %.3lf, %.3lf\n", KK[0], KK[1], KK[2], KK[3]);
			rt_printk("dob_flag = %d, pado_flag = %d, workspace_flag = %d, fc_flag = %d\n", dob_flag, pado_flag, workspace_flag, fc_flag);
//			rt_printk("rb_m2s = %.3lf, %.3lf, %.3lf, %.3lf\n", t, xrb_cmd[0], xee_cmd[0], xee_cmd[1]);
			rt_printk("seq = %d, seq_comp = %d, output_flag = %d, emergency_flag = %d\n", seq, seq_comp, output_flag, emergency_flag);
			rt_printk("psc_data = %x, %x, %x, %x, %x, %x\n", (unsigned char)psc_data[0], (unsigned char)psc_data[1], (unsigned char)psc_data[2], (unsigned char)psc_data[3], (unsigned char)psc_data[4], (unsigned char)psc_data[5]);
//			rt_printk("q0_off_dis = %lf, q0_off_hat = %lf\n", q0_off_dis, q0_off_hat);
//			rt_printk("q0_datatec = %.3lf, roll = %d, pitch = %d, yaw = %d\n", q0_datatec, roll_pos, pitch_pos, yaw_pos);
			if(emergency_flag == 1){
				rt_printk("\n!!!!!!!!!! Emergency !!!!!!!!!!\n");
				rt_printk("saturation_num = %d, overspeed_num = %d\n", saturation_num, overspeed_num);
			}
			rt_printk("\n");
		}
	#else
//		int i;
		if((int)(t/dt)%(int)(0.1/dt)==0){
//		if((int)(t/dt)%(int)(0.001/dt)==0){
//		if((int)(t/dt)%(int)(0.001/dt)==0 && t > 1.3){
//		if(fabs(tau_ref[7]) > 0.5){
			printf("t = %lf\n", t);
			if(emergency_flag != 0){
				printf("\n!!!!!!!!!! Emergency !!!!!!!!!!\n");
				printf("saturation_num = %d, overspeed_num = %d\n", saturation_num, overspeed_num);
			}
			#ifdef DEBUG
//				if(seq == 2){
//					printf("q_cmd = ");        q_cmd.disp();
					printf("q_res = ");        q_res.disp();
//					printf("dq_cmd = ");       dq_cmd.disp();
					printf("dq_res = ");       dq_res.disp();
//					printf("ddq_cmd = ");      ddq_cmd.disp();
//					printf("ddq_res = ");      ddq_res.disp();
//					printf("ddq_ref = ");      ddq_ref.disp();
//					printf("tau_ref     = ");  tau_ref.disp();
//					printf("tau_int     = ");  tau_int.disp();
//					printf("tau_fric    = ");  tau_fric.disp();
//					printf("tau_dis     = ");  tau_dis.disp();
//					printf("tau_dis_hat = ");  tau_dis_hat.disp();
//					printf("tau_main    = ");  tau_main.disp();
//					printf("tau_null    = ");  tau_null.disp();
//					printf("xee_cmd = ");      xee_cmd.disp();
//					printf("xee_res = ");      xee_res.disp();
//					printf("ddxee_ref = ");    ddxee_ref.disp();
//					printf("a_cmd = ");        a_cmd.disp();
//					printf("a_res = ");        a_res.disp();
//					printf("dda_ref = ");      dda_ref.disp();
//					printf("dda_ref = %lf, %lf, %lf\n", );
//					printf("xg_cmd = ");       xg_cmd.disp();
//					printf("xg_res = ");       xg_res.disp();
//					printf("dxg_cmd = ");      dxg_cmd.disp();
//					printf("dxg_res = ");      dxg_res.disp();
//					printf("ddxg_cmd = ");     ddxg_cmd.disp();
//					printf("ddxg_res = ");     ddxg_res.disp();
//					printf("ddxg_ref = ");     ddxg_ref.disp();
//					printf("xrb_res = ");      xrb_res.disp();
//					printf("dxrb_res = ");     dxrb_res.disp();
//					printf("ddxrb_res = ");    ddxrb_res.disp();
//					printf("ddxrb_ref = ");    ddxrb_ref.disp();
//					printf("xrb_cmd = ");      xrb_cmd.disp();
//					printf("dxrb_cmd = ");     dxrb_cmd.disp();
//					printf("ddxrb_cmd = ");    ddxrb_cmd.disp();
//					printf("Jee\n");           Jee.disp();
//					(Jee*Jee.trans()).disp();
//					printf("det(Jee*Jee_T) = %lf\n", (Jee*Jee.trans()).det());
//					printf("dJee\n");          dJee.disp();
//					printf("Ja\n");            Ja.disp();
//					(Ja*Ja.trans()).disp();
//					printf("det(Ja*Ja_T) = %lf\n", (Ja*Ja.trans()).det());
//					printf("dJa\n");           dJa.disp();
					printf("M\n");             M.disp();
//					printf("M_inv\n");         M_inv.disp();
					printf("H\n");             H.disp();
					printf("G\n");             G.disp();
//					printf("tau_dis     = ");  tau_dis.disp();
//					printf("M_inv*tau_ref = "); (M_inv*tau_ref).disp();
//					printf("M_inv*H       = "); (M_inv*H).disp();
//					printf("M_inv*G       = "); (M_inv*G).disp();
//					printf("M_inv*tau_dis = "); (M_inv*tau_dis).disp();
//					printf("M_inv*(tau_ref - H - G) = "); (M_inv*(tau_ref - H - G - tau_dis)).disp();
//					printf("Mn\n");            Mn.disp();
//					printf("Mn_inv\n");        Mn_inv.disp();
//					printf("Hn\n");            Hn.disp();
//					printf("Gn\n");            Gn.disp();
//					printf("JT_F_ref\n");      JT_F_ref.disp();
//					printf("Jg\n");            Jg.disp();
//					printf("det(Jg*Jg_T) = %lf\n", (Jg.row(0)*Jg.row(0)));
//					printf("Atorj\n");         Atorj.disp();
//					printf("Atorj_inv\n");     Atorj.inv().disp();
//					printf("det(Atorj) = %lf\n", Atorj.det());
//					printf("Btorj\n");         Btorj.disp();
//					printf("Atorw\n");         Atorw.disp();
//					printf("Atorw_pseinv\n");  Atorw_pseinv.disp();
//					printf("WAtorw_pseinv\n"); WAtorw_pseinv.disp();
//					printf("Btorw\n");         Btorw.disp();
//					printf("C_tor\n");         C_tor.disp();
//					printf("D_tor\n");         D_tor.disp();
//					printf("Det J*JT = %lf\n", det(&J_JT[0][0], 4));
//					printf("Evaluation Function = %lf\n", valval);
//					printf("C_tor = ");        C_tor.disp();
//					printf("D_tor = ");        D_tor.disp();
//					printf("D_tor_pseinv = "); D_tor_pseinv.disp();
//					for(i=0; i<8; i++)  printf("DH[%d] = %lf, %lf, %lf, %lf\n", i, DH[i][0], DH[i][1], DH[i][2], DH[i][3]);
//					for(i=0; i<8; i++){ printf("Trans[%d] = \n", i);   Trans[i].disp();  }
//					for(i=0; i<8; i++){ printf("TransW[%d] = \n", i);  TransW[i].disp(); }
//					for(i=0; i<8; i++){ printf("Rotate[%d] = \n", i);  Rotate[i].disp(); }
//					for(i=0; i<8; i++){ printf("RotateW[%d] = \n", i); RotateW[i].disp(); }
//					printf("R_R_[7]\n"); R_R_[7].disp();
//					printf("Jreac\n"); Jreac.disp();
//					printf("Jreac_pseinv\n"); ((Jreac*Jreac.trans()).inv()*Jreac).disp();
//					printf("tau_reac_hat = "); tau_reac_hat.disp();
//					printf("Fee_reac_hat = "); Fee_reac_hat.disp();
					
//					printf("Atorw2\n"); Atorw2.disp();
//					printf("det(Atorw2) = %lf\n", Atorw2.det());
//					printf("Atorw2_inv\n"); Atorw2.inv().disp();
//					printf("Btorw2 = \n"); Btorw2.disp();
//					printf("dob_flag = %d, pado_flag = %d, workspace_flag = %d, fc_flag = %d\n", dob_flag, pado_flag, workspace_flag, fc_flag);
//					printf("seq = %d, seq_comp = %d, output_flag = %d, emergency_flag = %d\n", seq, seq_comp, output_flag, emergency_flag);
//					printf("saturation_num = %d, overspeed_num = %d\n", saturation_num, overspeed_num);
					
//					printf("q0_off_dis = %lf, q0_off_hat = %lf\n", q0_off_dis, q0_off_hat);
					
//					printf("KK = %.3lf, %.3lf, %.3lf, %.3lf\n", KK[0], KK[1], KK[2], KK[3]);
					getchar();
//				}
			#endif
		}
	#endif
}

#endif
