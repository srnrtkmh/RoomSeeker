/**************************************************************************************************/
/*                                                                                                */
/* FILE : nzmn_model.h                                                                            */
/* MEMO : Functions to calculate nozamani's model elements                                        */
/*                                                                                                */
/* 2011/01/09 : Start to edit this file                                                           */
/* 2011/09/03 : Rename this file "nzmn2D_model.cpp" from "nozamani_model.cpp"                     */
/* 2011/09/11 : Add functions "reload_jaco_cog()" and "reload_kinematics_cog()"                   */
/* 2011/09/18 : Change type of "Jg" and "dJg" from mat24 to mat25                                 */
/* 2012/04/23 :                                                                                   */
/* 2012/04/26 :                                                                                   */
/* 2012/04/27 :                                                                                   */
/* 2012/05/07 :                                                                                   */
/* 2012/05/09 :                                                                                   */
/* 2012/09/11 : Add kinematics about end-effector posture                                         */
/* 2012/10/03 : Change the project not to use nozamani_t class. Use global variables instead of it*/
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_MODEL
#define NZMN_MODEL

//================================================================================================//
// Structure definition of variable set for Nozamani                                              //
//================================================================================================//
class nozamani_network_m2s{
	public:
		double t;				// Time
		double xrb_cmd[3];		// Robot position command
		double xee_cmd[3];		// End-effector position command
};

class nozamani_network_s2m{
	public:
		double t;				// Time
		double xrb_res[3];		// Robot position response
		double xee_res[3];		// End-effector position response
};

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void reload_trans(void);									// Reload homo transformation matrix

void reload_jaco_ee(void);									// Reload Jacobian
void reload_kinematics_ee(void);							// Reload work space variables from joint space response and Jacobian

void reload_jaco_m_ee(void);								// Reload Jacobian in manipulator coordinate
void reload_kinematics_m_ee(void);							// Reload work space variables in manipulator coordinatefrom joint space response and Jacobian

void reload_jaco_cog(vec9 mass);							// Reload Jacobian between joint and COG velocity
void reload_kinematics_cog(vec9 mass);						// Reload COG position, velocity and acceleration

void reload_jaco_rb(void);									// Reload Jacobian between joint and robot velocity
void reload_kinematics_rb(void);							// Reload robot position, velocity and acceleration

vec9 newton_euler(vec9 dq, vec9 ddq, vec9 m, mat33 I[]);	// Derive torque when q, dq, ddq are given in Newton-Euler method
void reload_dynamics(void);									// Reload nominal dynamics element such as M, H, G...etc
void reload_nom_dynamics(void);								// Reload nominal dynamics element such as Mn, Hn, Gn...etc
void reload_model(void);									// Reload Kinematics and Dynamics
void reload_nom_vdip_model(void);							// Reload virtual double inverted pendulum

//================================================================================================//
// Reload homo transformation matrix                                                              //
//================================================================================================//
void reload_trans(void)
{
	int i;
	
	// Link parameters ---------------------------------------------------------------------------//
	// a_i - 1            // alpha_i - 1             // d_i                // theta_i
	DH[0][0] = 0.0;       DH[0][1] = -M_PI/2.0;  DH[0][2] = 0.0;       DH[0][3] = q_res[0];				// Link0(passive joint) coordinates
	DH[1][0] = 0.0;       DH[1][1] =  M_PI/2.0;  DH[1][2] = L[0];      DH[1][3] = q_res[1];				// Link1 coordinates
	DH[2][0] = L[1];      DH[2][1] = -M_PI/2.0;  DH[2][2] = 0.0;       DH[2][3] = q_res[2] - M_PI/2.0;		// Link2 coordinates
	DH[3][0] = L[2];      DH[3][1] = 0.0;        DH[3][2] = 0.0;       DH[3][3] = q_res[3];				// Link3 coordinates
	DH[4][0] = L[3];      DH[4][1] = 0.0;        DH[4][2] = 0.0;       DH[4][3] = q_res[4];				// Link4 coordinates
	DH[5][0] = L[4];      DH[5][1] = M_PI/2.0;   DH[5][2] = 0.0;       DH[5][3] = q_res[5] + M_PI/2.0;		// Link5 coordinates
	DH[6][0] = 0.0;       DH[6][1] = M_PI/2.0;   DH[6][2] = L[5]+L[6]; DH[6][3] = q_res[6];				// Link6 coordinates
	DH[7][0] = 0.0;       DH[7][1] = -M_PI/2.0;  DH[7][2] = 0.0;       DH[7][3] = -M_PI/2.0;				// end-effector coordinates
	
	// Homogeneous transformation matrix and Rotation matrix -------------------------------------//
	W_R_R.row_set(0, cos(xrb_res[2]), -sin(xrb_res[2]), 0.0);
	W_R_R.row_set(1, sin(xrb_res[2]), cos(xrb_res[2]),  0.0);
	W_R_R.row_set(2, 0.0,             0.0,              1.0);
	for(i=0; i<8; i++){
		i_1_Trans_[i].row_set(0, cos(DH[i][3]),               -sin(DH[i][3]),              0.0,            DH[i][0]);
		i_1_Trans_[i].row_set(1, cos(DH[i][1])*sin(DH[i][3]), cos(DH[i][1])*cos(DH[i][3]), -sin(DH[i][1]), -DH[i][2]*sin(DH[i][1]));
		i_1_Trans_[i].row_set(2, sin(DH[i][1])*sin(DH[i][3]), sin(DH[i][1])*cos(DH[i][3]), cos(DH[i][1]),  DH[i][2]*cos(DH[i][1]));
		i_1_Trans_[i].row_set(3, 0.0,                         0.0,                         0.0,            1.0);
		
		i_1_R_[i].row_set(0, i_1_Trans_[i].A[0][0], i_1_Trans_[i].A[0][1], i_1_Trans_[i].A[0][2]);
		i_1_R_[i].row_set(1, i_1_Trans_[i].A[1][0], i_1_Trans_[i].A[1][1], i_1_Trans_[i].A[1][2]);
		i_1_R_[i].row_set(2, i_1_Trans_[i].A[2][0], i_1_Trans_[i].A[2][1], i_1_Trans_[i].A[2][2]);
		i_1_RT_[i] = i_1_R_[i].trans();
	}
	
	// Homogeneous transformation matrix (R -> i) ------------------------------------------------//
	R_Trans_[0] = i_1_Trans_[0];
	R_R_[0] = i_1_R_[0];
	R_RT_[0] = R_R_[0].trans();
	for(i=1; i<8; i++){
		R_Trans_[i] = R_Trans_[i-1]*i_1_Trans_[i];
		R_R_[i] = R_R_[i-1]*i_1_R_[i];
		R_RT_[i] = R_R_[i].trans();
	}
	
	// Angular velocity, position and velocity of each link coordinate ---------------------------//
	R_omega_R = ez*((RW/Tread)*(dq_res[7] - dq_res[8]));
	R_omega_[0] = R_R_[0]*ez*dq_res[0];
	R_p_[0].set(R_Trans_[0].A[0][3], R_Trans_[0].A[1][3], R_Trans_[0].A[2][3]);
	R_dp_[0].clear();
	for(i=1; i<=7; i++){
		R_omega_[i] = R_omega_[i-1] + R_R_[i]*ez*dq_res[i];
		R_p_[i].set(R_Trans_[i].A[0][3], R_Trans_[i].A[1][3], R_Trans_[i].A[2][3]);
		R_dp_[i] = R_dp_[i-1] + outer_product(R_omega_[i-1], R_R_[i-1]*i_1_p_hat_[i]);
	}
	for(i=0; i<7; i++) R_z_[i] = R_R_[i]*ez;	// ^{R}R_{i} * e_z
	
	// Rotation matrix (0 -> wheel)
	o_R_wh.row_set(0, cos(-q_res[0]), -sin(-q_res[0]), 0.0);
	o_R_wh.row_set(1, sin(-q_res[0]),  cos(-q_res[0]), 0.0);
	o_R_wh.row_set(2, 0.0           ,  0.0           , 1.0);
	o_RT_wh = o_R_wh.trans();
}

//================================================================================================//
// Reload Jacobian from joint angle to end-effector position and posture                          //
//================================================================================================//
void reload_jaco_ee(void)
{
	int i;
	
	// Jee = [z[0] x (^{R}p_{E} - ^{R}p_{i}) ... z[6] x (^{R}p_{E} - ^{R}p_{6})]
	for(i=0; i<7; i++) Jee.col_set(i, outer_product(R_z_[i], R_p_[7] - R_p_[i]));
	Jee.col_set(7, null_vec3);
	Jee.col_set(8, null_vec3);
	
	// Ja = [z[0], ... , z[6]]
	for(i=0; i<7; i++) Ja.col_set(i, R_z_[i]);
	Ja.col_set(7, null_vec3);
	Ja.col_set(8, null_vec3);
	
	// The derivative of Jacobian (Future works)
	dJee.row_set(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dJee.row_set(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dJee.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

//================================================================================================//
// Reload end-effector position, velocity and acceleration                                        //
//================================================================================================//
void reload_kinematics_ee(void)
{
	// Reload jacobian
	reload_jaco_ee();
	
	// Position, Velocity and Approximate acceleration
	xee_res.set(R_Trans_[7].A[0][3], R_Trans_[7].A[1][3], R_Trans_[7].A[2][3]);
	dxee_res = Jee*dq_res;
	ddxee_res = Jee*ddq_res + dJee*dq_res;
	
	// End-effector Posture, angular velocity, angular acceleration
	a_res.x[0] = atan2(-R_R_[7].A[1][2], R_R_[7].A[2][2]);
	if(R_R_[7].A[0][0] == 0.0)                                     a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[0][0]*R_R_[7].A[0][0] - R_R_[7].A[0][1]*R_R_[7].A[0][1]));
	else if(fabs(R_R_[7].A[0][0]) - fabs(R_R_[7].A[0][1]) >= 0.0)  a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[0][0]*R_R_[7].A[0][0] - R_R_[7].A[0][1]*R_R_[7].A[0][1]));
	else if(fabs(R_R_[7].A[2][2]) - fabs(R_R_[7].A[1][2]) >= 0.0)  a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[2][2]*R_R_[7].A[2][2] - R_R_[7].A[1][2]*R_R_[7].A[1][2]));
//	else{ printf("Error. Cannot calculate end-effector posture.\n"); }
//	a_res.x[1] = asin(R_R_[7].A[0][2]);
	a_res.x[2] = atan2(-R_R_[7].A[0][1], R_R_[7].A[0][0]);
	
	da_res = Ja*dq_res;
}

//================================================================================================//
// Reload Jacobian in manipulator coordinate                                                      //
//================================================================================================//
// Now coding
void reload_jaco_m_ee(void)
{
	int i;
	
	// Jee = [z[0] x (^{R}p_{E} - ^{R}p_{i}) ... z[6] x (^{R}p_{E} - ^{R}p_{6})]
	for(i=1; i<=6; i++) m_Jee.col_set(i, outer_product(m_z_[i], m_p_[7] - m_p_[i]));
	m_Jee.col_set(0, null_vec3);
	m_Jee.col_set(7, null_vec3);
	m_Jee.col_set(8, null_vec3);
	
	// Ja = [z[0], ... , z[6]]
	for(i=0; i<7; i++) Ja.col_set(i, R_z_[i]);
	
	// The derivative of Jacobian (Future works)
	dJee.row_set(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dJee.row_set(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dJee.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

//================================================================================================//
// Reload work space variables in manipulator coordinatefrom joint space response and Jacobian    //
//================================================================================================//
// Now coding
void reload_kinematics_m_ee(void)
{
	// Reload jacobian
	reload_jaco_ee();
	
	// Position, Velocity and Approximate acceleration
	m_xee_res.set(R_Trans_[7].A[0][3], R_Trans_[7].A[1][3], R_Trans_[7].A[2][3]);
	m_dxee_res = m_Jee*dq_res;
	m_ddxee_res = m_Jee*ddq_res + m_dJee*dq_res;
	
	// End-effector Posture, angular velocity, angular acceleration
	m_a_res.x[0] = atan2(-R_R_[7].A[1][2], R_R_[7].A[2][2]);
	if(R_R_[7].A[0][0] == 0.0)                                     m_a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[0][0]*R_R_[7].A[0][0] - R_R_[7].A[0][1]*R_R_[7].A[0][1]));
	else if(fabs(R_R_[7].A[0][0]) - fabs(R_R_[7].A[0][1]) >= 0.0)  m_a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[0][0]*R_R_[7].A[0][0] - R_R_[7].A[0][1]*R_R_[7].A[0][1]));
	else if(fabs(R_R_[7].A[2][2]) - fabs(R_R_[7].A[1][2]) >= 0.0)  m_a_res.x[1] = atan2(R_R_[7].A[0][2], sqrt(R_R_[7].A[2][2]*R_R_[7].A[2][2] - R_R_[7].A[1][2]*R_R_[7].A[1][2]));
//	else{ printf("Error. Cannot calculate end-effector posture.\n"); }
	m_a_res.x[2] = atan2(-R_R_[7].A[0][1], R_R_[7].A[0][0]);
	m_da_res = m_Ja*dq_res;
}

//================================================================================================//
// Reload Jacobian between joint and COG velocity                                                 //
//================================================================================================//
void reload_jaco_cog(vec9 mass)
{
	int i, j;
	double sum_mass;
	vec3 tmp;
	
	// COG Jacobian
	sum_mass = mass[0] + mass[1] + mass[2] + mass[3] + mass[4] + mass[5] + mass[6];
	for(i=0; i<=6; i++){
		tmp.clear();
		for(j=i; j<=6; j++){
//			tmp = tmp + outer_product(mass[j]*R_z_[i], R_p_[j+1] - R_p_[i]);
			tmp = tmp + outer_product(mass[j]*R_z_[i], R_p_[j] + R_R_[j]*i_s_hat_[j] - R_p_[i]);
		}
		Jg.col_set(i, tmp*(1/sum_mass));
	}
	Jg.col_set(7, null_vec3);
	Jg.col_set(8, null_vec3);
	
	// The derivative of COG Jacobian (Future works)
	dJg.row_set(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	dJg.row_set(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

//================================================================================================//
// Reload COG position, velocity and acceleration                                                 //
//================================================================================================//
void reload_kinematics_cog(vec9 mass)
{
	int i;
	vec3 xg[7];			// COG position of each link
	double sum_mass;	// Sum of mass
	vec3 tmp3;
	
	// Reload COG Jacobian
	reload_jaco_cog(mass);
	
	// COG Position
	xg_res.clear();
	sum_mass = mass[0] + mass[1] + mass[2] + mass[3] + mass[4] + mass[5] + mass[6];
//	for(i=0; i<=6; i++) xg[i].set(R_Trans_[i+1].A[0][3], R_Trans_[i+1].A[1][3], R_Trans_[i+1].A[2][3]);
	for(i=0; i<=6; i++){
		tmp3 = R_R_[i]*i_s_hat_[i];
		xg[i].set(R_Trans_[i].A[0][3] + tmp3[0], R_Trans_[i].A[1][3] + tmp3[1], R_Trans_[i].A[2][3] + tmp3[2]);
	}
	for(i=0; i<=6; i++) xg_res = xg_res + mass[i]*xg[i];
	xg_res = xg_res*(1.0/sum_mass);
	
	// COG velocity and approximate acceleration
	dxg_res = Jg*dq_res;
	ddxg_res = Jg*ddq_res + dJg*dq_res;
}

//================================================================================================//
// Reload Jacobian between joint and robot velocity                                               //
//================================================================================================//
void reload_jaco_rb(void)
{
	// Robot position Jacobian
	Jrb.row_set(0, RW*cos(xrb_res[2]), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5*RW*cos(xrb_res[2]), 0.5*RW*cos(xrb_res[2]));
	Jrb.row_set(1, RW*sin(xrb_res[2]), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5*RW*sin(xrb_res[2]), 0.5*RW*sin(xrb_res[2]));
	Jrb.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, RW/Tread, -RW/Tread);
	
	// The derivative of robot position Jacobian (Future works)
	dJrb.row_set(0, -RW*sin(xrb_res[2])*dxrb_res[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -(RW/2.0)*sin(xrb_res[2])*dxrb_res[2], -(RW/2.0)*sin(xrb_res[2])*dxrb_res[2]);
	dJrb.row_set(1,  RW*cos(xrb_res[2])*dxrb_res[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  (RW/2.0)*cos(xrb_res[2])*dxrb_res[2],  (RW/2.0)*cos(xrb_res[2])*dxrb_res[2]);
	dJrb.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	// Robot position Jacobian in robot axis
	R_Jrb.row_set(0, RW,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5*RW,   0.5*RW);
	R_Jrb.row_set(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      0.0);
	R_Jrb.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, RW/Tread, -RW/Tread);
	
	// The derivative of robot position Jacobian in robot axis(Future works)
	R_dJrb.row_set(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	R_dJrb.row_set(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	R_dJrb.row_set(2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

//================================================================================================//
// Reload robot position, velocity and acceleration                                               //
//================================================================================================//
void reload_kinematics_rb(void)
{
	// Reload robot position Jacobian
	reload_jaco_rb();
	
	// Robot position, velocity, acceleration
	ddxrb_res = Jrb*ddq_res + dJrb*dq_res;
	dxrb_res = Jrb*dq_res;
	xrb_res = xrb_res + dt*dxrb_res;
	
	// Robot velocity and acceleration viewed from robot coordinate
	R_ddxrb_res.set(RW*(ddq_res[0] + 0.5*(ddq_res[7] + ddq_res[8])), 0.0, (RW/Tread)*(ddq_res[7] - ddq_res[8]));
	R_dxrb_res.set(RW*(dq_res[0] + 0.5*(dq_res[7] + dq_res[8])), 0.0, (RW/Tread)*(dq_res[7] - dq_res[8]));
	R_xrb_res = R_xrb_res + dt*R_dxrb_res;
}

//================================================================================================//
// Reload dynamics element such as inertia matrix, nonlinear vector ...                           //
//================================================================================================//
vec9 newton_euler(vec9 dq, vec9 ddq, vec9 m, mat33 I[])
{
	int i;
	double dphi, ddphi;
	vec3 R_omega_R, r_omega_r, l_omega_l, i_omega_[7];		// i_omega_[i] : ^{i}omega_{i}
	vec3 R_domega_R, r_domega_r, l_domega_l, i_domega_[7];	// i_domega_[i] : ^{i}domega_{i}
	vec3 R_ddp_R, r_ddp_r, l_ddp_l, i_ddp_[7];				// i_ddp_[i] : ^{i}ddp_{i}
	vec3 R_dds_R, r_dds_r, l_dds_l, i_dds_[7];				// i_dds_[i] : ^{i}dds_{i}
	vec3 r_f_hat_r, l_f_hat_l, i_f_hat_[7];					// i_f_hat_[i] : ^{i}f_hat_{i}
	vec3 r_n_hat_r, l_n_hat_l, i_n_hat_[7];					// i_n_hat_[i] : ^{i}n_hat_{i}
	vec3 r_f_r, l_f_l, i_f_[7];								// i_f_[i] : ^{i}f_{i}
	vec3 r_n_r, l_n_l, i_n_[7];								// i_n_[i] : ^{i}n_{i}
	vec3 W_ddp_W, R_ddp_hat_R;								// W_ddp_W : , R_ddp_hat_R : 
	vec3 wh_f_1, wh_f_hat_o, o_f_hat_o, o_f_1, wh_n_hat_o, wh_n_1;	// 
	vec9 tau;												// Torque loaded to each link of the coordinate used in newton-euler method
	
	// Robot coordinate configuration
	dphi = (RW/Tread)*(dq[7] - dq[8]);
	ddphi = (RW/Tread)*(ddq[7] - ddq[8]);
	R_omega_R = ez*dphi;
	R_domega_R = ez*ddphi;
	W_ddp_W = ez*g;
	R_ddp_hat_R.set(RW*(ddq[0] + 0.5*(ddq[7] + ddq[8])), 0.0, 0.0);
	R_ddp_R = W_R_R.trans()*W_ddp_W + R_ddp_hat_R;
	
	// Forward recursion -------------------------------------------------------------------------//
	// About link 0
	i_omega_[0] = i_1_RT_[0]*R_omega_R + ez*dq[0];
	i_domega_[0] = i_1_RT_[0]*R_domega_R + ez*ddq[0] + outer_product(i_1_RT_[0]*R_omega_R, ez*dq[0]);
	i_ddp_[0] = i_1_RT_[0]*(R_ddp_R + outer_product(R_domega_R, i_1_p_hat_[0]) + outer_product(R_omega_R, outer_product(R_omega_R, i_1_p_hat_[0])));
	i_dds_[0] = i_ddp_[0] + outer_product(i_domega_[0], i_s_hat_[0]) + outer_product(i_omega_[0], outer_product(i_omega_[0], i_s_hat_[0]));
	i_f_hat_[0] = m[0]*i_dds_[0];
	i_n_hat_[0] = I[0]*i_domega_[0] + outer_product(i_omega_[0], I[0]*i_omega_[0]);
//	i_n_hat_[0].set(0.0, 0.0, 0.0);
	
	// In the case of right wheel
	r_omega_r = o_RT_wh*i_omega_[0] + ez*dq[7];
	r_domega_r = o_RT_wh*i_domega_[0] + ez*ddq[7] + outer_product(o_RT_wh*i_omega_[0], ez*dq[7]);
	r_ddp_r = o_RT_wh*(i_ddp_[0] + outer_product(i_domega_[0], o_p_hat_r) + outer_product(i_omega_[0], outer_product(i_omega_[0], o_p_hat_r)));
	r_dds_r = r_ddp_r;
	r_f_hat_r = m[7]*r_dds_r;
	r_n_hat_r = I[7]*r_domega_r + outer_product(r_omega_r, I[7]*r_omega_r);
	
	// In the case of left wheel
	l_omega_l = o_RT_wh*i_omega_[0] + ez*dq[8];
	l_domega_l = o_RT_wh*i_domega_[0] + ez*ddq[8] + outer_product(o_RT_wh*i_omega_[0], ez*dq[8]);
	l_ddp_l = o_RT_wh*(i_ddp_[0] + outer_product(i_domega_[0], o_p_hat_l) + outer_product(i_omega_[0], outer_product(i_omega_[0], o_p_hat_l)));
	l_dds_l = l_ddp_l;
	l_f_hat_l = m[8]*l_dds_l;
	l_n_hat_l = I[8]*l_domega_l + outer_product(l_omega_l, I[8]*l_omega_l);
	
	// In the case of link 1 - link 6
	for(i=1; i<=6; i++){
		i_omega_[i] = i_1_RT_[i]*i_omega_[i-1] + ez*dq[i];
		i_domega_[i] = i_1_RT_[i]*i_domega_[i-1] + ez*ddq[i] + outer_product(i_1_RT_[i]*i_omega_[i-1], ez*dq[i]);
		i_ddp_[i] = i_1_RT_[i]*(i_ddp_[i-1] + outer_product(i_domega_[i-1], i_1_p_hat_[i]) + outer_product(i_omega_[i-1], outer_product(i_omega_[i-1], i_1_p_hat_[i])));
		i_dds_[i] = i_ddp_[i] + outer_product(i_domega_[i], i_s_hat_[i]) + outer_product(i_omega_[i], outer_product(i_omega_[i], i_s_hat_[i]));
		i_f_hat_[i] = m[i]*i_dds_[i];
		i_n_hat_[i] = I[i]*i_domega_[i] + outer_product(i_omega_[i], I[i]*i_omega_[i]);
	}
	
	// Backward recursion ------------------------------------------------------------------------//
	// In the case of link 1 - link6
	i_f_[6] = i_f_hat_[6];
	i_n_[6] = outer_product(i_s_hat_[6], i_f_hat_[6]) + i_n_hat_[6];
	tau.x[6] = ez*i_n_[6];
	for(i=5; i>=1; i--){
		i_f_[i] = i_1_R_[i+1]*i_f_[i+1] + i_f_hat_[i];
		i_n_[i] = i_1_R_[i+1]*i_n_[i+1] + outer_product(i_s_hat_[i], i_f_hat_[i]) + outer_product(i_1_p_hat_[i+1], i_1_R_[i+1]*i_f_[i+1]) + i_n_hat_[i];
		tau.x[i] = ez*i_n_[i];
	}
	
	// In the case of link 0, right wheel and left wheel
	
	// Now coding -->
	vec3 o_f_o_tmp, o_n_o_tmp;
	double o_f_orx, o_f_olx, o_f_ory, o_f_oly;
	double r_f_rx, l_f_lx;
	
	o_f_o_tmp = i_1_R_[1]*i_f_[1] + i_f_hat_[0];
	o_n_o_tmp = i_1_R_[1]*i_n_[1] + outer_product(i_s_hat_[0], i_f_hat_[0]) + outer_product(i_1_p_hat_[1], i_1_R_[1]*i_f_[1])+ i_n_hat_[0];
	o_f_orx = (1.0/2.0)*o_f_o_tmp[0] - (1.0/Tread)*o_n_o_tmp[1];
	o_f_olx = (1.0/2.0)*o_f_o_tmp[0] + (1.0/Tread)*o_n_o_tmp[1];
	o_f_ory = (1.0/2.0)*o_f_o_tmp[1] + (1.0/Tread)*o_n_o_tmp[0];
	o_f_oly = (1.0/2.0)*o_f_o_tmp[1] - (1.0/Tread)*o_n_o_tmp[0];
	
	r_f_rx = cos(q_res[0])*o_f_orx - sin(q_res[0])*o_f_ory + r_f_hat_r[0];
	l_f_lx = cos(q_res[0])*o_f_olx - sin(q_res[0])*o_f_oly + l_f_hat_l[0];
	
	tau.x[7] = r_n_hat_r[2] + RW*r_f_rx;
	tau.x[8] = l_n_hat_l[2] + RW*l_f_lx;
	tau.x[0] = o_n_o_tmp[2] + tau[7] + tau[8];
	
/*	if(ddq[0] > 0.9 || ddq[7] > 0.9 || ddq[8] > 0.9){
		printf("q  = "); q_res.disp();
		printf("dq = "); dq.disp();
		printf("ddq = "); ddq.disp();
		printf("i_f_hat_[0] = "); i_f_hat_[0].disp();
		printf("i_n_hat_[0] = "); i_n_hat_[0].disp();
		printf("r_f_hat_r = ");   r_f_hat_r.disp();
		printf("r_n_hat_r = ");   r_n_hat_r.disp();
		printf("l_f_hat_l = ");   l_f_hat_l.disp();
		printf("l_n_hat_l = ");   l_n_hat_l.disp();
		
		printf("o_f_o_tmp = ");   o_f_o_tmp.disp();
		printf("o_n_o_tmp = ");   o_n_o_tmp.disp();
		printf("o_f_orx = %lf, o_f_olx = %lf, o_f_ory = %lf, o_f_oly = %lf\n", o_f_orx, o_f_olx, o_f_ory, o_f_oly);
		
		printf("RW*r_f_rx = %lf\n", RW*r_f_rx);
		printf("RW*l_f_lx = %lf\n", RW*l_f_lx);
		printf("r_n_hat_r = ");   r_n_hat_r.disp();
		printf("l_n_hat_l = ");   l_n_hat_l.disp();
		printf("tau = ");         tau.disp();
		printf("\n");
		getchar();
	}
*/	
	// Now coding <--
	
/*	o_f_1 = i_1_R_[1] * i_f_[1];
	wh_f_1 = o_RT_wh * o_f_1;
	wh_n_1 = o_RT_wh * i_1_R_[1] * i_n_[1];
	o_f_hat_o = i_f_hat_[0];
	wh_f_hat_o = o_RT_wh * o_f_hat_o;
	wh_n_hat_o = o_RT_wh * i_n_hat_[0];
	
	r_f_r.x[0] = 0.5*(- wh_f_hat_o[0] - wh_f_1[0]) + (1.0/Tread)*(r_n_hat_r[1] + l_n_hat_l[1] + wh_n_1[1] - Lg[0]*sin(q_res[0])*(o_f_hat_o[2] + o_f_1[2]) + wh_n_hat_o[1]);
	l_f_l.x[0] = 0.5*(- wh_f_hat_o[0] - wh_f_1[0]) - (1.0/Tread)*(r_n_hat_r[1] + l_n_hat_l[1] + wh_n_1[1] - Lg[0]*sin(q_res[0])*(o_f_hat_o[2] + o_f_1[2]) + wh_n_hat_o[1]);
	tau.x[7] = -RW*(r_f_r[0] - r_f_hat_r[0]) + r_n_hat_r[2];
	tau.x[8] = -RW*(l_f_l[0] - l_f_hat_l[0]) + l_n_hat_l[2];
	tau.x[0] = tau[7] + tau[8] + wh_n_1[2] + Lg[0]*(o_f_hat_o[0] + o_f_1[0]) + wh_n_hat_o[2];
*/	
	return(tau);
}

//================================================================================================//
// Reload dynamics element such as inertia matrix, nonlinear vector using Newton-Euler method     //
//================================================================================================//
void reload_dynamics(void)
{
	int i, j;
	vec9 dq, ddq;	// Joint acceleration in motion equation
	vec9 tau_int;	// Internal(?) torque
	
	// Internal torque
	dq = dq_res;
	ddq.clear();
	tau_int = newton_euler(dq, ddq, m, I);
	
	// Gravity term and Centrifugal and Corioli term
	dq.clear();
	ddq.clear();
	G = newton_euler(dq, ddq, m, I);
	H = tau_int - G;
	
	// Inertia matrix
	dq = dq_res;
	for(i=0; i<9; i++){
		for(j=0; j<9; j++){
			if(i==j) ddq.x[j] = 1.0;
			else     ddq.x[j] = 0.0;
		}
		M.col_set(i, newton_euler(dq, ddq, m, I) - tau_int);
	}
	M_inv = M.inv();
}

//================================================================================================//
// Reload nominal dynamics element such as nominal inertia matrix, nonlinear vector ...           //
//================================================================================================//
void reload_nom_dynamics(void)
{
	int i, j;
	vec9 dq, ddq;	// Joint acceleration in motion equation
	vec9 tau_int;	// Internal(?) torque
	
	// Internal torque
	dq = dq_res;
	ddq.clear();
	tau_int = newton_euler(dq, ddq, mn, In);
	
	// Gravity term and Centrifugal and Corioli term
	dq.clear();
	ddq.clear();
	Gn = newton_euler(dq, ddq, mn, In);
	Hn = tau_int - Gn;
	
	// Inertia matrix
	dq = dq_res;
	for(i=0; i<9; i++){
		for(j=0; j<9; j++){
			if(i==j) ddq.x[j] = 1.0;
			else     ddq.x[j] = 0.0;
		}
		Mn.col_set(i, newton_euler(dq, ddq, mn, In) - tau_int);
	}
	Mn_inv = Mn.inv();
}

// Reload model (Both Kinematics and Dynamics) ---------------------------------------------------//
void reload_model(void)
{
	reload_dynamics();
}

// Reload virtual double inverted pendulum -------------------------------------------------------//
void reload_nom_vdip_model(void)
{
/*	double mG, LG;
	mG = mn[0] + mn[1] + mn[2];
	LG = sqrt((xg_res[0] - L[0]*sin(q_res[0]))*(xg_res[0] - L[0]*sin(q_res[0])) + (xg_res[1] - L[0]*cos(q_res[0]))*(xg_res[1] - L[0]*cos(q_res[0])));
	
	Mn_vdip.A[0][0]	= RW*RW*(mn[4] + mn[0] + mG);
	Mn_vdip.A[0][1] = RW*L[0]*(mn[0] + mG);
	Mn_vdip.A[0][2] = RW*LG*mG;
	Mn_vdip.A[1][0] = RW*L[0]*(mn[0] + mG);
	Mn_vdip.A[1][1] = L[0]*L[0]*(mn[0] + mG);
	Mn_vdip.A[1][2] = L[0]*LG*mG;
	Mn_vdip.A[2][0] = RW*LG*mG;
	Mn_vdip.A[2][1] = L[0]*LG*mG;
	Mn_vdip.A[2][2] = LG*LG*mG;
*/}

#endif
