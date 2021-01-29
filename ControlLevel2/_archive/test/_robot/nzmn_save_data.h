/**************************************************************************************************/
/*                                                                                                */
/* FILE : nzmn_save_data.h                                                                        */
/* MEMO : Functions to output various data to the files                                           */
/*                                                                                                */
/* 2010/11/30 : Start to edit this file                                                           */
/* 2011/09/02 : Change "nzmn2D_var.h" to "nzmn2D_class.h"                                         */
/* 2011/09/10 : Add functions "output_tmp()" and "output_file()"                                  */
/*              Remove function "output_res()"                                                    */
/*              Rename function "output_stick()" to "output_file_stick()"                         */
/* 2011/09/29 : Add argumentation "nozamani2D_t *rb" in function "output_file()"                  */
/*              in order to record experimental parameter                                         */
/* 2012/04/23 : Modify this file to adapt NZMN_V3 project                                         */
/* 2012/10/03 : Change the project not to use nozamani_t class. Use global variables instead of it*/
/* 2012/11/12 : Rename this file from "output_data.h" to "nzmn_save_data.h"                       */
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_SAVE_DATA
#define NZMN_SAVE_DATA

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void output_tmp(void);					// Output data into the temporal memory
void output_file(FILE *fp);				// Output reseponse data into file

void output_tmp_stick(void);			// Output stick diagram data into the temporal memory
void output_file_stick(FILE *fp_stick);	// Output stick diagram data into file

void output_debug(FILE *fp);			// For debugging

//================================================================================================//
// Write the experimental result data to temporary variables                                      //
//================================================================================================//
void output_tmp(void)
{
	// Store response variables
	t_tmp[data_cnt] = t;
	q_cmd_tmp[data_cnt]   = q_cmd*(180.0/M_PI);
	dq_cmd_tmp[data_cnt]  = dq_cmd*(180.0/M_PI);
	ddq_cmd_tmp[data_cnt] = ddq_cmd*(180.0/M_PI);
	q_res_tmp[data_cnt]   = q_res*(180.0/M_PI);
	dq_res_tmp[data_cnt]  = dq_res*(180.0/M_PI);
	ddq_res_tmp[data_cnt] = ddq_res*(180.0/M_PI);
	tau_ref_tmp[data_cnt] = tau_ref;
	tau_dis_tmp[data_cnt] = tau_dis;
	tau_dis_hat_tmp[data_cnt] = tau_dis_hat;
	tau_fric_tmp[data_cnt]    = tau_fric;
	
	xee_cmd_tmp[data_cnt]   = xee_cmd;
	dxee_cmd_tmp[data_cnt]  = dxee_cmd;
	ddxee_cmd_tmp[data_cnt] = ddxee_cmd;
	xee_res_tmp[data_cnt]   = xee_res;
	dxee_res_tmp[data_cnt]  = dxee_res;
	ddxee_res_tmp[data_cnt] = ddxee_res;
	Fee_reac_hat_tmp[data_cnt] = Fee_reac_hat;
	Fa_reac_hat_tmp[data_cnt] = Fa_reac_hat;
	
	a_cmd_tmp[data_cnt]   = a_cmd;
	da_cmd_tmp[data_cnt]  = da_cmd;
	dda_cmd_tmp[data_cnt] = dda_cmd;
	a_res_tmp[data_cnt]   = a_res;
	da_res_tmp[data_cnt]  = da_res;
	dda_res_tmp[data_cnt] = dda_res;
	
	xg_cmd_tmp[data_cnt]   = xg_cmd;
	dxg_cmd_tmp[data_cnt]  = dxg_cmd;
	ddxg_cmd_tmp[data_cnt] = ddxg_cmd;
	xg_res_tmp[data_cnt]   = xg_res;
	dxg_res_tmp[data_cnt]  = dxg_res;
	ddxg_res_tmp[data_cnt] = ddxg_res;
	
	xrb_cmd_tmp[data_cnt]   = R_xrb_cmd;
	dxrb_cmd_tmp[data_cnt]  = R_dxrb_cmd;
	ddxrb_cmd_tmp[data_cnt] = R_ddxrb_cmd;
	xrb_res_tmp[data_cnt]   = R_xrb_res;
	dxrb_res_tmp[data_cnt]  = R_dxrb_res;
	ddxrb_res_tmp[data_cnt] = R_ddxrb_res;
	ddxrb_ref_tmp[data_cnt] = R_ddxrb_ref;
	
	q0_datatec_tmp[data_cnt] = q0_datatec;
	q0_off_hat_tmp[data_cnt] = q0_off_hat;
	q0_off_dis_tmp[data_cnt] = q0_off_dis;
}

//================================================================================================//
// Write the experimental result data to the file                                                 //
//================================================================================================//
void output_file(FILE *fp)
{
	int i;
	
	printf("The number of data size is %d\n", data_cnt);
	printf("Starting data writing process\n");
	fprintf(fp, "# mn        , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", mn[0], mn[1], mn[2], mn[3], mn[4], mn[5], mn[6], mn[7], mn[8]);
	fprintf(fp, "# Jmn       , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", Jmn[0], Jmn[1], Jmn[2], Jmn[3], Jmn[4], Jmn[5], Jmn[6], Jmn[7], Jmn[8]);
	fprintf(fp, "# dob_flag  , %d\n", dob_flag);
	fprintf(fp, "# pado_flag , %d\n", pado_flag);
	fprintf(fp, "# dob_flag  , %d\n", rtob_flag);
	fprintf(fp, "# rtob_flag , %d\n", wob_flag);
	fprintf(fp, "# fc_flag   , %d\n", fc_flag);
	fprintf(fp, "# Kp_j      , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", Kp_j[0], Kp_j[1], Kp_j[2], Kp_j[3], Kp_j[4], Kp_j[5], Kp_j[6], Kp_j[7], Kp_j[8]);
	fprintf(fp, "# Kv_j      , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", Kv_j[0], Kv_j[1], Kv_j[2], Kv_j[3], Kv_j[4], Kp_j[5], Kp_j[6], Kp_j[7], Kp_j[8]);
	fprintf(fp, "# Kp_w      , %.5lf, %.5lf, %.5lf\n", Kp_w[0], Kp_w[1], Kp_w[2]);
	fprintf(fp, "# Kv_w      , %.5lf, %.5lf, %.5lf\n", Kv_w[0], Kv_w[1], Kv_w[2]);
	fprintf(fp, "# Kp_cog    , %.5lf, %.5lf\n", Kp_cog[0], Kp_cog[1]);
	fprintf(fp, "# Kv_cog    , %.5lf, %.5lf\n", Kv_cog[0], Kv_cog[1]);
	fprintf(fp, "# Kp_rb, %.5lf, Kv_rb, %.5lf, Ki_rb, %.5lf\n", Kp_rb, Kv_rb, Ki_rb);
	fprintf(fp, "# Kp_rb2, %.5lf, Kv_rb2, %.5lf, Ki_rb2, %.5lf\n", Kp_rb2, Kv_rb2, Ki_rb2);
	fprintf(fp, "# p         , %.5lf, %.5lf, %.5lf, %.5lf\n", pp[0], pp[1], pp[2], pp[3]);
	fprintf(fp, "# Kp_phi, %.5lf, Kv_phi, %.5lf\n", Kp_phi, Kv_phi);
	fprintf(fp, "# Kv_null   , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", Kv_null[0], Kv_null[1], Kv_null[2], Kv_null[3], Kv_null[4], Kv_null[5], Kv_null[6], Kv_null[7], Kv_null[8]);
	fprintf(fp, "# g_dob     , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", g_dob[0], g_dob[1], g_dob[2], g_dob[3], g_dob[4], g_dob[5], g_dob[6], g_dob[7], g_dob[8]);
	fprintf(fp, "# g_rtob    , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", g_rtob[0], g_rtob[1], g_rtob[2], g_rtob[3], g_rtob[4], g_rtob[5], g_rtob[6], g_rtob[7], g_rtob[8]);
	fprintf(fp, "# g_wob     , %.5lf, %.5lf, %.5lf\n", g_wob[0], g_wob[1], g_wob[2]);
	fprintf(fp, "# g_dq      , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", g_dq[0], g_dq[1], g_dq[2], g_dq[3], g_dq[4], g_dq[5], g_dq[6], g_dq[7], g_dq[8]);
	fprintf(fp, "# g_ddq     , %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf, %.5lf\n", g_ddq[0], g_ddq[1], g_ddq[2], g_ddq[3], g_ddq[4], g_ddq[5], g_ddq[6], g_ddq[7], g_ddq[8]);
	
	#include "fprintf_data_format.txt"
	for(i=0; i<data_cnt; i++){
		#include "fprintf_data.txt"
	}
	printf("Finished!!\n");
}

//================================================================================================//
// Output stick diagram data into the temporal memory                                             //
//================================================================================================//
void output_tmp_stick(void)
{
// Now coding
/*	// Time
	rb_data_stick[stick_cnt].t = t;
	
	// Wheel part
	rb_data_stick[stick_cnt].stick[0][0] = RW*q_res[4];
	rb_data_stick[stick_cnt].stick[0][1] = RW;
	// Passive Joint
	rb_data_stick[stick_cnt].stick[1][0] = RW*q_res[4]+L[0]*sin(q_res[0]);
	rb_data_stick[stick_cnt].stick[1][1] = RW             +L[0]*cos(q_res[0]);
	// Link1
	rb_data_stick[stick_cnt].stick[2][0] = RW*q_res[4] + L[0]*sin(q_res[0]) + L[1]*sin(q_res[0]+q_res[1]);
	rb_data_stick[stick_cnt].stick[2][1] = RW              + L[0]*cos(q_res[0]) + L[1]*cos(q_res[0]+q_res[1]);
	// Link2
	rb_data_stick[stick_cnt].stick[3][0] = RW*q_res[4] + L[0]*sin(q_res[0]) + L[1]*sin(q_res[0]+q_res[1]) + L[2]*sin(q_res[0]+q_res[1]+q_res[2]);
	rb_data_stick[stick_cnt].stick[3][1] = RW              + L[0]*cos(q_res[0]) + L[1]*cos(q_res[0]+q_res[1]) + L[2]*cos(q_res[0]+q_res[1]+q_res[2]);
	// Link3
	rb_data_stick[stick_cnt].stick[4][0] = RW*q_res[4] + L[0]*sin(q_res[0]) + L[1]*sin(q_res[0]+q_res[1]) + L[2]*sin(q_res[0]+q_res[1]+q_res[2]) + L[3]*sin(q_res[0]+q_res[1]+q_res[2]+q_res[3]);
	rb_data_stick[stick_cnt].stick[4][1] = RW              + L[0]*cos(q_res[0]) + L[1]*cos(q_res[0]+q_res[1]) + L[2]*cos(q_res[0]+q_res[1]+q_res[2]) + L[3]*cos(q_res[0]+q_res[1]+q_res[2]+q_res[3]);
	
	// Command
	rb_data_stick[stick_cnt].xee_cmd[0] = xee_cmd[0] + xrb_res[0];
	rb_data_stick[stick_cnt].xee_cmd[1] = xee_cmd[1];
*/}

//================================================================================================//
// Output stick diagram data to the file                                                          //
//================================================================================================//
void output_file_stick(FILE *fp_stick)
{
// Now coding
/*	int i, j;
	
	for(i=0; i<stick_cnt; i++){
		 fprintf(fp_stick, "# t=%lf\n", rb_data_stick[i].t);
		 for(j=0; j<5; j++){
			 fprintf(fp_stick, "%lf,%lf,%lf,%lf\n", rb_data_stick[i].stick[j][0], rb_data_stick[i].stick[j][1], rb_data_stick[i].xee_cmd[0], rb_data_stick[i].xee_cmd[1]);
		 }
		 fprintf(fp_stick, "\n");
	}
*/}

// Output debugging data to the file
void output_debug(FILE *fp)
{
	// Now not in use
}

#endif

