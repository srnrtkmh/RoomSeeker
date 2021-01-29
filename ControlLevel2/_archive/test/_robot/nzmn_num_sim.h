/**************************************************************************************************/
/*                                                                                                */
/* FILE : nzmn_num_sim.h                                                                          */
/* MEMO : Functions to execute numerical simulation with Abemani model                            */
/*                                                                                                */
/* 2010/12/25 : Start to edit this file copied from "abemani_num_sim.h"                           */
/* 2011/09/02 : Rename "nzmn2D_num_sim.h" from "nozamani2D_num_sim.h"                             */
/* 2011/09/03 : Add "fric_flag" in argumentation to function "reload_res_free" to select          */
/*              whether friction is enabled or not                                                */
/* 2011/09/10 : Add constants "FRICTION_ON" and "FRICTION_OFF"                                    */
/* 2012/04/23 : Rename this file "nzmn_num_sim.h" to adapt NZMN_V3 project                        */
/* 2012/10/03 : Change the project not to use nozamani_t class. Use global variables instead of it*/
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_NUM_SIM
#define NZMN_NUM_SIM

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void reload_res_free(void);			// Reload response of nozamani without any constraint condition
void reload_res_forw_wall(void);	// Reload Response using Forward Euler Method
void reload_res_back_wall(void);	// Reload Response using Backward Euler Method
void reload_res_hybrid_wall(void);	// Reload Response using Hybrid(?) Euler Method
void reload_res_rk4(void);			// Reload Response using Runge Kutta Method (4th order (?))

//================================================================================================//
// Function Definition                                                                            //
//================================================================================================//
// Reload response of nozamani in no restriction
void reload_res_free(void)
{
	static vec9 q_res_sim(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec9 dq_res_sim(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	static vec9 ddq_res_sim(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	// Calculate Joint Space response ------------------------------------------------------------//
	q_res = q_res_sim;
	reload_trans();
	reload_model();
	ddq_res_sim = M_inv*(tau_ref - H - G - tau_dis);
	dq_res_sim = dq_res_sim + dt*ddq_res_sim;
	q_res_sim = q_res_sim + dt*dq_res_sim;
	
	// Store true value as we can apply sensor resolution limitation in other location -----------//
	ddq_res = ddq_res_sim;
	dq_res = dq_res_sim;
	q_res = q_res_sim;
}

// Reload Response using Forward Euler Method
void reload_res_forw_wall(void)
{
}

// Reload Response using Backward Euler Method
void reload_res_back_wall(void)
{
}

// Reload Response using Hybrid(?) Euler Method
// Using Forward Euler Method for M, H, G and Jaco, and Backward Method for q_res and F_res
void reload_res_hybrid_wall(void)
{
}

// Reload Response using Runge Kutta Method (4th order (?))
void reload_res_rk4(void)
{
}

#endif
