/**************************************************************************************************/
/*                                                                                                */
/* FILE : common_function.h                                                                       */
/* MEMO : Common functions used by almost all types of controller                                 */
/*                                                                                                */
/* 2012/12/17 : Start to edit this file                                                           */
/*                                                                                                */
/**************************************************************************************************/

#ifndef COMMON_FUNCTION
#define COMMON_FUNCTION

//================================================================================================//
// Included Files                                                                                 //
//================================================================================================//
#include <math.h>
#include "../exe_opt.h"
#include "../_lib/basic_calc.h"
#include "../_lib/basic_matrix.h"
#include "../_nzmn/nzmn_model.h"

#ifdef EXP
	#include "../_nzmn/nzmn_io.h"
#endif

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void reload_input(void);
void reload_dis(void);
void reload_reac(void);
void reload_output(void);
void torque_limit(void);

//================================================================================================//
// Measure the sensor values and reload each variable (input the counter and gyro value)          //
//================================================================================================//
void reload_input(void)
{
	int i;
	#ifdef EXP
		static int gyro_in[2]={0, 0};
		static double q0[2] = {0.0, 0.0}, q0_pre[2] = {0.0, 0.0};
		static double q_res_lpf[9]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		static double dq_res_lpf[9]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		static double q_res_lpf_enc[9]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		
		// Measure each environmental information ------------------------------------------------//
		// Read encoder value and Calculate actuated joint response
		for(i=1; i<=8; i++) cnt_read(i, cnt_in);
		cnt2angle(cnt_in, q_res_enc);
		for(i=0; i<=8; i++){
			q_res_lpf_enc[i] += dt*dq_res_enc[i];
			dq_res_enc[i] = g_enc[i]*(q_res_enc[i]-q_res_lpf_enc[i]);
		}
		
		// Substitute joint angle converted from encoder to joint response variables
		for(i=1; i<=8; i++) q_res.x[i] = q_res_enc[i-1];
		
		// Read gyro value and calculate passive joint response
		pre_gyro_in[0] = gyro_in[0];
		pre_gyro_in[1] = gyro_in[1];
		gyro_set_flag = gyro_read(gyro_in);
		if(gyro_set_flag == 1){
			gyro2angle(gyro_in, q0);
			q_res.x[0] = q0[0];
			dq_res.x[0] = q0[1];
			q0_pre[0] = q0[0];
			q0_pre[1] = q0[1];
		}else{
			q_res.x[0] = q0_pre[0];
			dq_res.x[0] = q0_pre[1];
		}
		
		// Calculate pseudo derivative from obtained data ----------------------------------------//
		// Angular velocity of the active joints filtered by LPF (Joint0 velocity is already obtained)
		for(i=1; i<9; i++){
			q_res_lpf[i] += dt*dq_res.x[i];
			dq_res.x[i] = g_dq[i]*(q_res[i]-q_res_lpf[i]);
		}
		
		// Angular acceleration of the active and passive joints filtered by LPF
		for(i=0; i<9; i++){
			dq_res_lpf[i] += dt*ddq_res.x[i];
			ddq_res.x[i] = g_ddq[i]*(dq_res.x[i]-dq_res_lpf[i]);
		}
	#endif
	
	// Gyro sensor offset canceling --------------------------------------------------------------//
	// Add disturbance deliberately
	#ifdef OFFSET_DISTURBANCE
		q0_off_dis -= (M_PI/180.0)*(100.0/60.0/60.0/1000.0);		// Gyro offset is 100[deg/h]
		q_res.x[0] += q0_off_dis;									// Tsugi kurutoki ha Shinchi mata ha Sokuteichi ni natteiru node kore de yoi 
	#endif
	
	// Calculate offset value from disturbance at passive joint
	X = 0;
	for(i=0; i<=6; i++){
		x[i] = R_Trans_[i].A[0][3] + R_R_[i].row(0)*i_s_hat_[i];	// In the case that COG is at the identified position
		z[i] = R_Trans_[i].A[2][3] + R_R_[i].row(2)*i_s_hat_[i];	// In the case that COG is at the identified position
//		x[i] = R_Trans_[i+1].A[0][3];								// In the case that COG is at the tip of the link
//		z[i] = R_Trans_[i+1].A[2][3];								// In the case that COG is at the tip of the link
		X += mn[i]*g*sqrt(x[i]*x[i] + z[i]*z[i]);
	}
	tau0_dis_hat_int += dt*tau_dis_hat[0];
	dq0_off_hat = (g_p_off*tau_dis_hat[0] + g_i_off*tau0_dis_hat_int)/X;	// LPF + Integral estimation
//	dq0_off_hat = g_p_off*tau_dis_hat[0]/X;									// Only LPF
	#ifdef OFFSET_CANCELING
		q0_off_hat += dt*dq0_off_hat;
		q_res.x[0] -= q0_off_hat;
	#endif
}

//================================================================================================//
// Insert Disturbance Observer                                                                    //
//================================================================================================//
void reload_dis(void)
{
	static vec9 dob_tmp0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec9 dob_tmp1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	#ifdef EXP
		static double dob0_tmp0 = 0.0, dob0_tmp1 = 0.0;
		double gyro_dt = 0.02;
	#endif
	
	// DOB based on 1st order pseudo derivative with gravity and friction compensation -----------//
	dob_tmp0 = g_dob.diag()*Mn*dq_res;
	dob_tmp1 = dob_tmp1 + dt*(g_dob.diag()*(tau_ref - Gn - tau_fric_n + dob_tmp0 - dob_tmp1));
	tau_dis_hat = dob_tmp1 - dob_tmp0;
	
	#ifdef EXP
		// DOB based on 1st order pseudo derivative with gravity and friction compensation -------//
		// Zero order hold of pitch angle disturbance torque (due to gyro sampling time)
		if(gyro_set_flag !=0){
			// DOB based on 1st order pseudo derivative with gravity compensation --------------------//
			dob0_tmp0 = g_dob[0]*Mn.row(0)*dq_res;
			dob0_tmp1 = dob0_tmp1 + gyro_dt*(g_dob[0]*(0.0 - Gn[0] - tau_fric_n[0] + dob0_tmp0 - dob0_tmp1));
			tau_dis_hat.x[0] = dob0_tmp1 - dob0_tmp0;
		}else{
			tau_dis_hat.x[0] = dob0_tmp1 - dob0_tmp0;
		}
	#endif
}

//================================================================================================//
// Reaction Torque Observer                                                                       //
//================================================================================================//
void reload_reac(void)
{
	int i;
	static vec9 rtob_tmp0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec9 rtob_tmp1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	#ifdef EXP
		static double rtob0_tmp0 = 0.0, rtob0_tmp1 = 0.0;
		double gyro_dt = 0.02;
	#endif
//	mat66 Jmani;
	vec6 tau_reac_hat_mani;
	
	// RTOB based on 1st order pseudo derivative with gravity and friction compensation ----------//
	if(rtob_flag != 0){
		for(i=0; i<9; i++){
			if(dq_res[i] > 0.0){
//				tau_fric_n.x[i] = Dn[i]*dq_res[i] + Clmbn[i];
				tau_fric_n.x[i] = Dn[i]*dq_res[i];
			}else if(dq_res[i] < 0.0){
//				tau_fric_n.x[i] = Dn[i]*dq_res[i] - Clmbn[i];
				tau_fric_n.x[i] = Dn[i]*dq_res[i];
			}
		}
		rtob_tmp0 = g_rtob.diag()*Mn_rtob*dq_res;
		rtob_tmp1 = rtob_tmp1 + dt*(g_rtob.diag()*(tau_ref - Gn - Hn - tau_fric_n + rtob_tmp0 - rtob_tmp1));
		tau_reac_hat = rtob_tmp1 - rtob_tmp0;
		
		#ifdef EXP
			// RTOB based on 1st order pseudo derivative with gravity and friction compensation ------//
			// Zero order hold of pitch angle disturbance torque (due to gyro sampling time)
			if(gyro_set_flag !=0){
				// DOB based on 1st order pseudo derivative with gravity compensation ----------------//
				rtob0_tmp0 = g_rtob[0]*Mn.row(0)*dq_res;
				rtob0_tmp1 = rtob0_tmp1 + gyro_dt*(g_rtob[0]*(0.0 - Gn[0] - Hn[0] - tau_fric_n[0] + rtob0_tmp0 - rtob0_tmp1));
				tau_reac_hat.x[0] = rtob0_tmp1 - rtob0_tmp0;
			}else{
				tau_reac_hat.x[0] = rtob0_tmp1 - rtob0_tmp0;
			}
		#endif
		
		// Jacobian for reaction force estimation
		Jreac.row_set(0, Jee.row(0));
		Jreac.row_set(1, Jee.row(1));
		Jreac.row_set(2, Jee.row(2));
		Jreac.row_set(3, Ja.row(0));
		Jreac.row_set(4, Ja.row(1));
		Jreac.row_set(5, Ja.row(2));
		Jreac.set(0, 7, RW/2.0);              Jreac.set(0, 8, RW/2.0);
		Jreac.set(1, 7, RW/Tread*xee_res[0]); Jreac.set(1, 8, -RW/Tread*xee_res[0]);
		Jreac.set(2, 7, 0.0);                 Jreac.set(2, 8, 0.0);
		Jreac.set(0, 7, 0.0);                 Jreac.set(0, 8, 0.0);
		Jreac.set(1, 7, 0.0);                 Jreac.set(1, 8, 0.0);
		Jreac.set(2, 7, RW/Tread);            Jreac.set(2, 8, -RW/Tread);
		
		Jreac.col_set(0, null_vec6);
		Jreac.col_set(7, null_vec6);
		Jreac.col_set(8, null_vec6);
		
		// Reaction force derived by pseudo inverse matrix
		F_reac_hat = (Jreac*Jreac.trans()).inv()*Jreac*tau_reac_hat;
		Fee_reac_hat.set(F_reac_hat[0], F_reac_hat[1], F_reac_hat[2]);
		Fa_reac_hat.set(F_reac_hat[3], F_reac_hat[4], F_reac_hat[5]);
		
		
/*		Jmani.row_set(0, Jee.A[0][1], Jee.A[0][2], Jee.A[0][3], Jee.A[0][4], Jee.A[0][5], Jee.A[0][6]);
		Jmani.row_set(1, Jee.A[1][1], Jee.A[1][2], Jee.A[1][3], Jee.A[1][4], Jee.A[1][5], Jee.A[1][6]);
		Jmani.row_set(2, Jee.A[2][1], Jee.A[2][2], Jee.A[2][3], Jee.A[2][4], Jee.A[2][5], Jee.A[2][6]);
		Jmani.row_set(3, Ja.A[0][1], Ja.A[0][2], Ja.A[0][3], Ja.A[0][4], Ja.A[0][5], Ja.A[0][6]);
		Jmani.row_set(4, Ja.A[1][1], Ja.A[1][2], Ja.A[1][3], Ja.A[1][4], Ja.A[1][5], Ja.A[1][6]);
		Jmani.row_set(5, Ja.A[2][1], Ja.A[2][2], Ja.A[2][3], Ja.A[2][4], Ja.A[2][5], Ja.A[2][6]);
		
		tau_reac_hat_mani.set(tau_reac_hat[1], tau_reac_hat[2], tau_reac_hat[3], tau_reac_hat[4], tau_reac_hat[5], tau_reac_hat[6]);
		F_reac_hat = (Jmani.trans()).inv()*tau_reac_hat_mani;
		Fee_reac_hat.set(F_reac_hat[0], F_reac_hat[1], F_reac_hat[2]);
		Fa_reac_hat.set(F_reac_hat[3], F_reac_hat[4], F_reac_hat[5]);
*/		
/*		Jmani.disp();
		(Jmani.trans()).disp();
		Fee_reac_hat.disp();
		Fa_reac_hat.disp();
		getchar();
*/	}
}

//================================================================================================//
// Reload output value of the motor torque (output the command to the motor driver)               //
//================================================================================================//
void reload_output(void)
{
	#ifdef EXP
		int i;
		int da_out[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		double I_command[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//		double tau_offset[8] = {-0.047, -0.287, -0.098, 0.077, 0.238, 0.178, -0.062, 0.005};
//		double tau_offset[8] = {-0.047, -0.287, -0.098, 0.077, 0.438, 0.178, -0.062, 0.005};	Before 1217
		double tau_offset[8] = {-0.047, -0.287, -0.098, 0.077, 0.438, 0.178, 0.624, -0.082};	// 1218
		vec9 g_output(0.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 60.0, 60.0);
		static vec9 tau_ref_pre(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		
		#ifdef OUTPUT_FILTER
			tau_ref = tau_ref_pre + dt*(g_output.diag()*(tau_ref - tau_ref_pre));
			tau_ref_pre = tau_ref;
		#endif
		
		// Common output in all sequence
		if(emergency_flag == 0){
			for(i=0; i<8; i++) I_command[i] = (tau_ref[i+1] + tau_offset[i]) / Ktn[i+1];
		}else{
			for(i=0; i<8; i++) I_command[i] = 0.0;
		}
		
		// output D/A value
		current2da(I_command, da_out);
		da_write(da_out);
	#endif
}

//================================================================================================//
// Torque limitation for safety                                                                   //
//================================================================================================//
void torque_limit(void)
{
	int i;
	
	// Soft servo on mechanism
	#ifdef EXP
		for(i=0; i<8; i++){
			if(srv_on_flag[i] == 0){
					tau_ref.x[i+1] = 0;
			}
		}
	#endif
	
	#ifdef MANIPULATOR_OFF
		for(i=0; i<=6; i++) tau_ref.x[i] = 0.0;
	#endif
	
	// Torque limitation
	#ifndef NO_LIMIT
		// Saturation of torque input
		if(emergency_flag == 0){
			for(i=1; i<9; i++){
					if(fabs(tau_ref[i]) >= tau_limit[i]){
					tau_ref.x[i] = sign(tau_ref[i])*tau_limit[i];
					saturation_counter[i]++;
					if(saturation_counter[i] >= SATURATION_MAX_CNT){
						emergency_flag = 1;
						saturation_num = i;
						overspeed_num = -1;
					}
				}else saturation_counter[i] = 0;
			}
			
			// Limitation of joint velocity
			for(i=0; i<9; i++){
				if(fabs(dq_res[i]) >= dq_limit[i]){
					if(++overspeed_counter[i] >= OVERSPEED_MAX_CNT){
						emergency_flag = 1;
						overspeed_num = i;
						saturation_num = -1;
					}
				}
			}
		}
	#endif
	
	#ifdef FREE_MOTION
		tau_ref.clear();							// No torque input
	#endif
	
	// Restrict torque input if emergency occured
	if(emergency_flag == 1) tau_ref.clear();	// No torque input
}

#endif
