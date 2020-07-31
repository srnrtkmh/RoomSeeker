/**************************************************************************************************/
/*                                                                                                */
/* FILE : nozamani_object.h                                                                       */
/* MEMO : Header File for "nozamani_object.c"                                                     */
/*                                                                                                */
/* 2010/12/25 : Start editing this file                                                           */
/* 2010/01/09 : Rename each function                                                              */
/* 2011/09/02 : Rename this file "nozamani_object.h" to "nzmn_object.h"                           */
/* 2012/04/27 : Modify to adapt NZMN_V3 project                                                   */
/* 2012/10/03 : Change the project not to use nozamani_t class. Use global variables instead of it*/
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_OBJECT
#define NZMN_OBJEcT

/**************************************************************************************************/
/* Function Prototypes                                                                            */
/**************************************************************************************************/
void nozamani_object(double q_res[], double x_rb[]);	// Draw nozamani object
void nozamani_info_2D(void);					// Draw nozamani information in the 2D drawing area

void nozamani_in_free(void);					// Draw nozamani object in free condition
void nozamani_in_wall_pushing(void);			// Draw nozamani object in the situation of wall pushing

//================================================================================================//
// Function Definition                                                                            //
//================================================================================================//
// Draw nozamani object in free condition
void nozamani_in_free(void)
{
	pre_draw();
	
	// 3D Graphics ###############################################################################//
	glPushMatrix();
		// Draw Floor
//		floor_panel(100.0, 1.0, DarkKhaki, brown);		// Normally use this function
		floor_line(100.0, 1.0, red);					// If light processing is needed, use this function
		
		// Draw nozamani object
		nozamani_object(q_res.x, xrb_res.x);
	glPopMatrix();
	
	// 2D Graphics ###############################################################################//
	nozamani_info_2D();
	
	// Exchange the old screen info with new one #################################################//
	glutSwapBuffers();
}

// Draw nozamani object in the situation of wall pushing
void nozamani_in_wall_pushing(void)
{
	// Previous drawing to set basic variables
	pre_draw();
	
	// Store the World coordination
	glPushMatrix();
		// Draw Floor
//		floor_panel(100.0, 1.0, DarkKhaki, brown);		// Normally use this function
		floor_line(100.0, 1.0, red);					// If light processing is needed, use this function
		
		// Draw Wall Object
		glPushMatrix();
			glTranslated(1.60, 0.0, 1.5);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white);				// ï\ñ ëÆê´(åıÇÃîΩéÀì¡ê´Å‡êF)ÇÃê›íË(ägéUåı)
			glScaled(3.0, 3.0, 3.0);								// É{ÉfÉBÇÃ(x, y, z)ÇÃî{ó¶
			glutSolidCube(1.0);												// àÍï”ÇÃí∑Ç≥Ç™1.0ÇÃóßï˚ëÃÇÃï`âÊ
		glPopMatrix();														// É{ÉfÉBç¿ïWån9ÇÃåƒÇ—èoÇµ
		
		// Draw force sensor
		glPushMatrix();
			glTranslated(0.07, 0, xee_res[1]);
			glPushMatrix();
				glRotated(90, 0.0, 0.0, 1.0);
				Cylinder(0.05, 0.06, NavyBlue);
			glPopMatrix();
			glPushMatrix();
				glTranslated(-0.035-0.03, 0.02, 0.0);
				glScaled(0.06, 0.015, 0.05);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, silver_diffuse);
				glutSolidCube(1.0);
			glPopMatrix();
			glPushMatrix();
				glTranslated(-0.035-0.03, -0.02, 0.0);
				glScaled(0.06, 0.015, 0.05);
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, silver_diffuse);
				glutSolidCube(1.0);
			glPopMatrix();
			glPushMatrix();
				glTranslated(-0.07, 0.0, 0.0);
				Cylinder(0.01, 0.1, gray70);
			glPopMatrix();
		glPopMatrix();
		
		// Draw nozamani object
		nozamani_object(q_res.x, xrb_res.x);
	glPopMatrix();

	// 2D Drawing ################################################################################//
	nozamani_info_2D();
	
	// Exchange the old screen info with new one #################################################//
	glutSwapBuffers();
}

// Draw nozamani object
void nozamani_object(double q_res[], double xrb_res[])
{
	// Parameters related to the illustration of the Robot #######################################//
	// Wheel
	double rubber_thickness = 0.06;						// thickness of a rubber part of the tire
	double rubber_width = 0.069;						// width of a tire
	double wheel_thickness = 0.02;						// thickness of a rubber part of the tire
	double wheel_width = 0.039;							// width of a tire
	double Hub_radius = 0.037;
	double Hub_width = 0.067;
	double Shaft_radius = 0.008;
	double Shaft_width = 0.168;
	double Frange_width1 = 0.0100;
	double Frange_width2 = 0.0150;
	double Frange_radius1 = 0.0350;
	double Frange_radius2 = 0.0175;

	// Body
	double Plate_width = 0.0050;
	double Wheelplate_length = 0.20;
	double Wheelplate_height = 0.20;
	double Wheelmotor_width1 = 0.012;
	double Wheelmotor_radius1 = 0.064;
	double Wheelmotor_width2 = 0.045;
	double Wheelmotor_radius2 = 0.0575;
	double body_width = 0.38;							// width of the body
	double body_height = 0.64;							// height of the body
	double body_length = 0.35;							// length of the body
	double Frame_width = 0.030;
	double Bottom_height = 0.10;
	double Floor_height1 = 0.17;
	double Floor_height2 = 0.26;
	double plastic_width = 0.0030;
	double plastic_length = 0.380;
	double plastic_height = 0.470;

	// Motor RH-14D
	double RH14_width1 = 0.0655;
	double RH14_radius1 = 0.025;
	double RH14_width2 = 0.0825;
	double RH14_radius2 = 0.01625;
	double RH14_shaft_width = 0.028;
	double RH14_shaft_radius = 0.0050;

	// Motor RH-8D
	double RH8_width = 0.1072;
	double RH8_radius = 0.0165;
	double RH8_shaft_width = 0.0218;
	double RH8_shaft_radius = 0.0040;

	// Motor 1 plate
	double Motor1_plate_length = 0.070;
	
	// Motor 2 plate
	double Motor2_plate_length = 0.090;
	double Motor2_plate_width = 0.070;

	// Link
	double Link_width = 0.010;
	double Link_length_14 = 0.070;
	double Link_length_8 = 0.053;
//	double Link[6] = {0.0, L[1]+0.070, L[2]+0.070, L[3]+0.070};

	// Handling object
//	double L_object = 1.0, L_width_object = 0.030, L_length_object = 0.080;

	int i;
//	int i, j, len, list;
//	char tmp_str[256];
	
	// Store the World Coordination System ///////////////////////////////////////////////////////////////
	glPushMatrix();									// Push World Coordination System
		// Move to the Robot Coordination System #############################################//
		glTranslated(xrb_res[0], xrb_res[1], 0.0);	// Move robot in world coordinate
		glRotated(xrb_res[2]*(180.0/M_PI), 0.0, 0.0, 1.0);		// Rotate robot along yaw motion
		
		// Right wheel /////////////////////////////////////////////////////
		glPushMatrix();											// store of the Robot coordinates 1
		glTranslated(0.0, -Tread/2.0, RW);					// Move to the center of the right wheel
			Pipe(RW-rubber_thickness, wheel_width, wheel_thickness, silver_diffuse);		// wheel
			Pipe(RW, rubber_width, rubber_thickness, black_rubber_diffuse);				// rubber
			Cylinder(Hub_radius, Hub_width, silver_diffuse);							// hub
			Cylinder(Shaft_radius, Shaft_width, silver_diffuse);						// shaft
			for(i=0; i<18; i++){														// spokes * 18
				glPushMatrix();															// store of the Wheel coordinates
					glRotated((q_res[0]+q_res[4])*(180.0/M_PI)+20.0*i, 0.0, 1.0, 0.0);	// rotate only the angle of the right wheel
					glRotated(-90.0, 1.0, 0.0, 0.0);									// rotate only the angle of the right wheel
				glTranslated(0.0, RW/2.0-0.001, 0.0);
					Cylinder(0.002, RW-0.002, silver_diffuse);
				glPopMatrix();															// call of the Wheel coordinates
			}
		glPopMatrix();											// call of the Robot coordinates 1
		
		// Frange //////////////////////////////////////////////////////////
		glPushMatrix();											// store of the Robot coordinates 2
			glTranslated(0.0, -body_width/2.0-Frange_width1/2.0, RW);
			Cylinder(Frange_radius1, Frange_width1, silver_diffuse);
			glTranslated(0.0, -Frange_width2/2.0, 0.0);
			Cylinder(Frange_radius2, Frange_width2, silver_diffuse);
		glPopMatrix();											// call of the Robot coordinates 2
		
		// Left Wheel //////////////////////////////////////////////////////
		glPushMatrix();											// store of the Robot coordinates 3
			glTranslated(0.0, Tread/2.0, RW);					// Move to the center of the left wheel
			Pipe(RW-rubber_thickness, wheel_width, wheel_thickness, silver_diffuse);		// wheel
			Pipe(RW, rubber_width, rubber_thickness, black_rubber_diffuse);				// rubber
			Cylinder(Hub_radius, Hub_width, silver_diffuse);							// hub
			Cylinder(Shaft_radius, Shaft_width, silver_diffuse);						// shaft
			for(i=0; i<18; i++){														// spokes * 18
				glPushMatrix();															// store of the Wheel coordinates
					glRotated((q_res[0]+q_res[4])*(180.0/M_PI)+20.0*i, 0.0, 1.0, 0.0);	// rotate only the angle of the right wheel
					glRotated(-90.0, 1.0, 0.0, 0.0);									// rotate only the angle of the right wheel
					glTranslated(0.0, RW/2.0-0.001, 0.0);
					Cylinder(0.002, RW-0.002, silver_diffuse);
				glPopMatrix();															// call of the Wheel coordinates
			}
		glPopMatrix();											// call of the Robot coordinates 3
		
		// Frange //////////////////////////////////////////////////////////
		glPushMatrix();											// store of the Robot coordinates 4
			glTranslated(0.0, body_width/2.0+Frange_width1/2.0, RW);
			Cylinder(Frange_radius1, Frange_width1, silver_diffuse);
			glTranslated(0.0, Frange_width2/2.0, 0.0);
			Cylinder(Frange_radius2, Frange_width2, silver_diffuse);
		glPopMatrix();											// call of the Robot coordinates 4
		
		// Body ////////////////////////////////////////////////////////////
		// Right aluminium plate
		glPushMatrix();											// store of the Robot coordinates 5
			glTranslated(0.0, -body_width/2.0-Plate_width/2.0, RW);
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, silver_diffuse);	// setting of the attribute of the surface
			glScaled(Wheelplate_length, Plate_width, Wheelplate_height);
			glutSolidCube(1.0);
		glPopMatrix();											// call of the Robot coordinates 5
		
		// Left aluminium plate
		glPushMatrix();											// store of the Robot coordinates 6
			glTranslated(0.0, body_width/2.0+Plate_width/2.0, RW);
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, silver_diffuse);	// setting of the attribute of the surface
			glScaled(Wheelplate_length, Plate_width, Wheelplate_height);
			glutSolidCube(1.0);
		glPopMatrix();											// call of the Robot coordinates 6
		
		// Right motor
		glPushMatrix();											// store of the Robot coordinates 7
			glTranslated(0.0, -body_width/2.0+Wheelmotor_width1/2.0, RW);
			Cylinder(Wheelmotor_radius1, Wheelmotor_width1, silver_diffuse);
			glTranslated(0.0, Wheelmotor_width1/2.0+Wheelmotor_width2/2.0, 0.0);
			Cylinder(Wheelmotor_radius2, Wheelmotor_width2, silver_diffuse);
		glPopMatrix();											// call of the Robot coordinates 7
		
		// Left motor
		glPushMatrix();											// store of the Robot coordinates 8
			glTranslated(0.0, body_width/2.0-Wheelmotor_width1/2.0, RW);
			Cylinder(Wheelmotor_radius1, Wheelmotor_width1, silver_diffuse);
			glTranslated(0.0, -Wheelmotor_width1/2.0-Wheelmotor_width2/2.0, 0.0);
			Cylinder(Wheelmotor_radius2, Wheelmotor_width2, silver_diffuse);
		glPopMatrix();											// call of the Robot coordinates 8
		
		// Move to the body coordinates ######################################################//
		glTranslated(0.0, 0.0, RW);
		glRotated(q_res[0]*(180.0/M_PI), 0.0, 1.0, 0.0);		// make "body inclination = inclination of passive joint"
		
		// 1st floor frame
		glPushMatrix();														// store of the Body coordinates 1
			glTranslated(0.0, 0.0, -Bottom_height+Frame_width/2.0);
			Square_frame(body_length, body_width, Frame_width, silver_diffuse);
		glPopMatrix();														// call of the Body coordinates 1
		
		// 2nd floor frame
		glPushMatrix();														// store of the Body coordinates 2
			glTranslated(0.0, 0.0, -Bottom_height+Floor_height1+Frame_width/2.0);
			Square_frame(body_length, body_width, Frame_width, silver_diffuse);
		glPopMatrix();														// call of the Body coordinates 2
			
		// 3rd floor frame
		glPushMatrix();														// store of the Body coordinates 3
			glTranslated(0.0, 0.0, -Bottom_height+Floor_height1+Floor_height2+Frame_width/2.0);
			Square_frame(body_length, body_width, Frame_width, silver_diffuse);
		glPopMatrix();														// call of the Body coordinates 3
			
		// Ceiling frame
		glPushMatrix();														// store of the Body coordinates 4
			glTranslated(0.0, 0.0, -Bottom_height+body_height-Frame_width/2.0);
			Square_frame(body_length, body_width, Frame_width, silver_diffuse);
		glPopMatrix();														// call of the Body coordinates 4
			
		// Frame in z direction
		glPushMatrix();														// store of the Body coordinates 5
			glTranslated(body_length/2.0-Frame_width/2.0, -body_width/2.0+Frame_width/2.0, -Bottom_height+body_height/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Frame_width, Frame_width, body_height);				// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 5
		glPushMatrix();														// store of the Body coordinates 6
			glTranslated(body_length/2.0-Frame_width/2.0, body_width/2.0-Frame_width/2.0, -Bottom_height+body_height/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Frame_width, Frame_width, body_height);				// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 6
		glPushMatrix();														// store of the Body coordinates 7
			glTranslated(-body_length/2.0+Frame_width/2.0, -body_width/2.0+Frame_width/2.0, -Bottom_height+body_height/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Frame_width, Frame_width, body_height);				// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 7
		glPushMatrix();														// store of the Body coordinates 8
			glTranslated(-body_length/2.0+Frame_width/2.0, body_width/2.0-Frame_width/2.0, -Bottom_height+body_height/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Frame_width, Frame_width, body_height);				// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 8
		
		// 1st floor plate
		glPushMatrix();														// store of the Body coordinates 9
			glTranslated(0.0, 0.0, -Bottom_height+Frame_width+Plate_width/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(body_length, body_width, Plate_width);					// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 9
		
		// 2nd floor plate
		glPushMatrix();														// store of the Body coordinates 10
			glTranslated(0.030, 0.0, -Bottom_height+Floor_height1+Frame_width+Plate_width/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(body_length-0.060, body_width, Plate_width);			// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 10
			
		// 3rd floor plate
		glPushMatrix();														// store of the Body coordinates 11
			glTranslated(0.030, 0.0, -Bottom_height+Floor_height1+Floor_height2+Frame_width+Plate_width/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(body_length-0.060, body_width, Plate_width);			// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Body coordinates 11
			
		// Clear plastic board
		glPushMatrix();														// store of the Body coordinates 12
			glTranslated(-body_length/2.0-plastic_width/2.0, 0.0, 0.305);
			glEnable(GL_BLEND);
			glBlendFunc(GL_DST_ALPHA, GL_SRC_COLOR);
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, gray16);			// setting of the attribute of the surface
			glScaled(plastic_width, plastic_length, plastic_height);
			glutSolidCube(1.0);
			glDisable(GL_BLEND);
		glPopMatrix();														// call of the Body coordinates 12
		
		// Move to the body top coordinates ##############################################//
		glPushMatrix();														// store of the Body coordinates 13
			glTranslated(D1, 0.0, -Bottom_height+body_height);
			
			// Motor 1
			glPushMatrix();														// store of the Body top coordinates 1
				glRotated(-90.0, 1.0, 0.0, 0.0);
				glTranslated(0.0, RH14_width1/2.0, 0.0);
				Cylinder(RH14_radius1, RH14_width1, gray5);
				glTranslated(0.0, RH14_width1/2.0+RH14_width2/2.0, 0.0);
				Cylinder(RH14_radius2, RH14_width2, gray5);
				glTranslated(0.0, -RH14_width1-RH14_width2/2.0-RH14_shaft_width/2.0, 0.0);
				Cylinder(RH14_shaft_radius, RH14_shaft_width, silver_diffuse);
			glPopMatrix();														// call of the Body top coordinates 1
			
			// Motor 1 plate
			glPushMatrix();														// store of the Body top coordinates 2
				glTranslated(0.0, 0.0, Plate_width/2.0);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
				glScaled(Motor1_plate_length, body_width, Plate_width);			// scale of the body (x, y, z)
				glutSolidCube(1.0);												// painting of the cube 1.0 on a side
			glPopMatrix();														// call of the Body top coordinates 2	
		glPopMatrix();														// call of the Body coordinates 12
		
		// Move to the Motor 1 coordinates ###############################################//
		glTranslated(D1, 0.0, -Bottom_height+body_height);
		glRotated(q_res[1]*(180.0/M_PI), 0.0, 0.0, 1.0);		// Rotation is not needed in the case of 2D dimension
		
		// Link 1
		glPushMatrix();														// store of the Motor1 coordinates 1
			glTranslated(0.0, -0.005, RH14_shaft_width+Plate_width/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Motor2_plate_width, Motor2_plate_length, Plate_width);	// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Motor1 coordinates 1
		glPushMatrix();														// store of the Motor1 coordinates 2
			glTranslated(0.0, -0.005-Motor2_plate_length/2.0+Plate_width/2.0, RH14_shaft_width+Motor2_plate_length/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);				// setting of the attribute of the surface
			glScaled(Motor2_plate_width, Plate_width, Motor2_plate_length);	// scale of the body (x, y, z)
			glutSolidCube(1.0);												// painting of the cube 1.0 on a side
		glPopMatrix();														// call of the Motor 1 coordinates 2
		
		// Move to the Motor 2 coordinates ###########################################//
		glTranslated(0.0, -0.005-Motor2_plate_length/2.0+Plate_width, RH14_shaft_width+0.055);
		glRotated(q_res[2]*(180.0/M_PI), 0.0, 1.0, 0.0);
		
		// Motor 2
		glPushMatrix();
			glTranslated(0.0, RH14_width1/2.0, 0.0);
			Cylinder(RH14_radius1, RH14_width1, gray5);
			glTranslated(0.0, RH14_width1/2.0+RH14_width2/2.0, 0.0);
			Cylinder(RH14_radius2, RH14_width2, gray5);
			glTranslated(0.0, -RH14_width1-RH14_width2/2.0-RH14_shaft_width/2.0, 0.0);
			Cylinder(RH14_shaft_radius, RH14_shaft_width, silver_diffuse);
		glPopMatrix();														// call of the Motor2 coordinates 1
			
		// Link 2
		glPushMatrix();														// store of the Motor2 coordinates 2
			glTranslated(0.0, -RH14_shaft_width-Link_width/2.0, L[2]/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(Link_length_14, Link_width, L[2]+0.07);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor2 coordinates 2
		glPushMatrix();														// store of the Motor2 coordinates 3
			glTranslated(0.0, -RH14_shaft_width-Link_width/2.0, L[2]/2.0);
			glRotated(90.0, 1.0, 0.0, 0.0);
			Square_frame(Link_length_14+0.02, L[2]+0.09, Link_width, black_rubber_diffuse);
		glPopMatrix();														// call of the Motor2 coordinates 3
		
		// Motor 3 coordinates ###############################################################//
		glTranslated(0.0, -Plate_width-RH14_shaft_width, L[2]);
		glRotated(q_res[3]*(180.0/M_PI), 0.0, 1.0, 0.0);
		
		// Motor 3
		glPushMatrix();														// store of the Motor3 coordinates 1
			glTranslated(0.0, -RH14_width1/2.0, 0.0);
			Cylinder(RH14_radius1, RH14_width1, gray5);
			glTranslated(0.0, -RH14_width1/2.0-RH14_width2/2.0, 0.0);
			Cylinder(RH14_radius2, RH14_width2, gray5);
			glTranslated(0.0, RH14_width1+RH14_width2/2.0+RH14_shaft_width/2.0, 0.0);
			Cylinder(RH14_shaft_radius, RH14_shaft_width, silver_diffuse);
		glPopMatrix();														// call of the Motor3 coordinates 1
		
		// Link 3
		glPushMatrix();														// store of the Motor3 coordinates 2
			glTranslated(0.0, RH14_shaft_width+Link_width/2.0, L[3]/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(Link_length_14, Link_width, L[3]+0.07);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor3 coordinates 2
		
		glPushMatrix();														// store of the Motor3 coordinates 3
			glTranslated(0.0, RH14_shaft_width+Link_width/2.0, L[3]/2.0);
			glRotated(90.0, 1.0, 0.0, 0.0);
			Square_frame(Link_length_14+0.02, L[3]+0.09, Link_width, black_rubber_diffuse);
		glPopMatrix();														// call of the Motor3 coordinates 3
		
		// Move to the Motor 4 coordinates ###################################################//
		glTranslated(0.0, Plate_width+RH14_shaft_width, L[3]);
		glRotated(q_res[4]*(180.0/M_PI), 0.0, 1.0, 0.0);
		
		// Motor 4
		glPushMatrix();														// store of the Motor4 coordinates 1
			glTranslated(0.0, RH14_width1/2.0, 0.0);
			Cylinder(RH14_radius1, RH14_width1, gray5);
			glTranslated(0.0, RH14_width1/2.0+RH14_width2/2.0, 0.0);
			Cylinder(RH14_radius2, RH14_width2, gray5);
			glTranslated(0.0, -RH14_width1-RH14_width2/2.0-RH14_shaft_width/2.0, 0.0);
			Cylinder(RH14_shaft_radius, RH14_shaft_width, silver_diffuse);
		glPopMatrix();														// call of the Motor4 coordinates 1
		
		// Link 4
		glPushMatrix();														// store of the Motor4 coordinates 2
			glTranslated(0.0, -RH14_shaft_width-Link_width/2.0, L[4]/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(Link_length_14, Link_width, L[4]+0.07);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor4 coordinates 2
		glPushMatrix();														// store of the Motor4 coordinates 3
			glTranslated(0.0, -RH14_shaft_width-Link_width/2.0, L[4]/2.0);
			glRotated(90.0, 1.0, 0.0, 0.0);
			Square_frame(Link_length_14+0.02, L[4]+0.09, Link_width, black_rubber_diffuse);
		glPopMatrix();														// call of the Motor4 coordinates 3
		
		// Motor5 plate
		glPushMatrix();														// store of the Motor4 coordinates 4
			glTranslated(Link_length_14/2.0-Link_width/2.0, RH14_shaft_width-RH8_radius-0.020, L[4]);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(Link_width, RH8_radius*2.0+0.020, RH14_radius1*2.0+0.020);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor4 coordinates 4
		
		// Move to the  Motor 5 coordinates ##################################################//
		glTranslated(Link_length_14/2.0-Link_width, RH14_shaft_width-0.020-RH8_radius, L[4]);
		glRotated(-90.0, 0.0, 0.0, 1.0);
		glRotated(q_res[5]*(180.0/M_PI), 0.0, 1.0, 0.0);
		
		// Motor 5
		glPushMatrix();														// store of the Motor5 coordinates 1
			glTranslated(0.0, -RH8_width/2.0, 0.0);
			Cylinder(RH8_radius, RH8_width, gray5);
			glTranslated(0.0, RH8_width/2.0+RH8_shaft_width/2.0, 0.0);
			Cylinder(RH8_shaft_radius, RH8_shaft_width, silver_diffuse);
		glPopMatrix();														// call of the Motor5 coordinates 1
		
		// Link 5
		glPushMatrix();														// store of the Motor5 coordinates 2
			glTranslated(0.0, RH8_shaft_width+Link_width/2.0, L[5]/2.0);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
//			glScaled(Link_length_8, Link_width, L[5]+0.07);
			glScaled(Link_length_8, Link_width, L[5]-0.05);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor5 coordinates 2
		glPushMatrix();														// store of the Motor5 coordinates 3
			glTranslated(0.0, RH8_shaft_width+Link_width/2.0, L[5]/2.0);
			glRotated(90.0, 1.0, 0.0, 0.0);
//			Square_frame(Link_length_8+0.02, L[5]+0.09, Link_width, black_rubber_diffuse);
			Square_frame(Link_length_8+0.02, L[5]-0.03, Link_width, black_rubber_diffuse);
		glPopMatrix();														// call of the Motor5 coordinates 3
		
		// Motor6 plate
		glPushMatrix();														// store of the Motor5 coordinates 4
			glTranslated(Link_length_8/2.0-Link_width/2.0-0.02, RH8_shaft_width-RH8_radius-0.010, L[5]-0.02);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(RH8_radius*2.0+0.020, RH8_radius*2.0+0.020, Link_width);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor5 coordinates 4
		
		// Move to the Motor 6 coordinates ###################################################//
		glTranslated(Link_length_8/2.0-Link_width, RH8_shaft_width-0.010-RH8_radius, L[5]-0.07);
		glRotated(-90.0, 1.0, 0.0, 0.0);
		glRotated(q_res[6]*(180.0/M_PI), 0.0, 1.0, 0.0);
		
		// Motor 6
		glPushMatrix();														// store of the Motor6 coordinates 1
			glTranslated(-0.015, 0.0, -RH8_radius/2.0);
			Cylinder(RH8_radius, RH8_width, gray5);
			glTranslated(0.0, -RH8_width/2.0-RH8_shaft_width/2.0, 0.0);
			Cylinder(RH8_shaft_radius, RH8_shaft_width, silver_diffuse);
		glPopMatrix();														// call of the Motor6 coordinates 1
		
		// Force sensor
		glPushMatrix();
			glRotated(-90.0, 0.0, 0.0, 1.0);
			glTranslated(0.1, -0.015, -0.01);
			glPushMatrix();
				glRotated(-90, 0.0, 0.0, 1.0);
				Cylinder(0.05, 0.06, NavyBlue);
			glPopMatrix();
		glPopMatrix();
		
		// Link 6
/*		glPushMatrix();														// store of the Motor6 coordinates 2
			glTranslated(0.0, -RH8_shaft_width-Link_width/2.0, L[6]/2.0-0.0265);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, silver_diffuse);
			glScaled(Link_length_8, Link_width, L[6]);
			glutSolidCube(1.0);
		glPopMatrix();														// call of the Motor6 coordinates 2
		glPushMatrix();														// store of the Motor6 coordinates 3
			glTranslated(0.0, -RH8_shaft_width-Link_width/2.0, L[6]/2.0-0.0265);
			glRotated(90.0, 1.0, 0.0, 0.0);
			Square_frame(Link_length_8+0.02, L[6]+0.02, Link_width, black_rubber_diffuse);
		glPopMatrix();														// call of the Motor6 coordinates 3
*/		
		// Move to the End-effector coordinates ######################################//
//		glTranslated(0.0, -RH8_shaft_width-Link_width/2.0, L[5]);
		
		// Handling object
//		glTranslated(0.0, 0.0, L_object/2.0);
//		glMaterialfv(GL_FRONT, GL_DIFFUSE, red3);
//		glScaled(L_width_object, L_length_object, L_object);
//		glutSolidCube(1.0);

	glPopMatrix();
}

// Draw nozamani information in the 2D drawing area
void nozamani_info_2D(void)
{
	int i;
	char tmp_str[256];
	
	// Disable Light Souce <- Light must be not reset to draw 2D object properly
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHT0);
	
	glPushMatrix();
		glPushAttrib(GL_ENABLE_BIT);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
			glLoadIdentity();
			gluOrtho2D(0.0, WX_SIZE, 0.0, WY_SIZE);
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
				glLoadIdentity();
				// Draw square to draw 2D object
				glBegin(GL_QUADS);
					// Gradation square part
					glColor3f(1.0, 1.0, 1.0);	glVertex2f(    0.0,  75.0);
					glColor3f(1.0, 1.0, 1.0);	glVertex2f(WX_SIZE,  75.0);
					glColor3f(0.8, 0.8, 0.8);	glVertex2f(WX_SIZE, 150.0);
					glColor3f(0.8, 0.8, 0.8);	glVertex2f(    0.0, 150.0);
					
					// Purely white space part
					glColor3f(1.0, 1.0, 1.0);
					glVertex2f(    0.0,  0.0);	glVertex2f(WX_SIZE,  0.0);
					glVertex2f(WX_SIZE, 75.0);	glVertex2f(    0.0, 75.0);
				glEnd();
				
				// Draw Character String
				sprintf(tmp_str, "t =  %-3.2lf, xee_res = %-+3.3lf, yee_res = %-+3.3lf, zee_res = %-+3.3lf", t, xee_res[0], xee_res[1], xee_res[2]);
				glColor3f(0.0, 0.0, 0.0);
				draw_string(10.0, 130.0, tmp_str, GLUT_BITMAP_9_BY_15);
				
				sprintf(tmp_str, "q0=%-+3.3lf[deg], q1=%-+3.3lf[deg], q2=%-+3.3lf[deg], q3=%-+3.3lf[deg], qw=%-+3.3lf[deg]", q_res[0]*180.0/M_PI, q_res[1]*180.0/M_PI, q_res[2]*180.0/M_PI, q_res[3]*180.0/M_PI, q_res[4]*180.0/M_PI);
				glColor3f(0.0, 0.0, 0.0);
				draw_string(10.0, 110.0, tmp_str, GLUT_BITMAP_9_BY_15);
				
				// Draw q_res Bars
				glColor3fv(green);
				for(i=0; i<4; i++){
					if(q_res[i]>0.0){
						glBegin(GL_QUADS);
							glVertex2f(    10.0+40.0*i, 55.0);
							glVertex2f(10.0+40.0*(i+1), 55.0);
							glVertex2f(10.0+40.0*(i+1), 55.0+q_res[i]/M_PI*(95.0-15.0));
							glVertex2f(    10.0+40.0*i, 55.0+q_res[i]/M_PI*(95.0-15.0));
						glEnd();
					}else{
						glBegin(GL_QUADS);
							glVertex2f(    10.0+40.0*i, 55.0+q_res[i]/M_PI*(95.0-15.0));
							glVertex2f(10.0+40.0*(i+1), 55.0+q_res[i]/M_PI*(95.0-15.0));
							glVertex2f(10.0+40.0*(i+1), 55.0);
							glVertex2f(    10.0+40.0*i, 55.0);
						glEnd();
					}
				}
				if((q_res[0]+q_res[4])>0.0){
					glBegin(GL_QUADS);
						glVertex2f(    10.0+40.0*i, 55.0);
						glVertex2f(10.0+40.0*(i+1), 55.0);
						glVertex2f(10.0+40.0*(i+1), 55.0+RW*(q_res[0]+q_res[4])/2.0/(L[0]+L[1]+L[2]+L[3])*(95.0-15.0));
						glVertex2f(    10.0+40.0*i, 55.0+RW*(q_res[0]+q_res[4])/2.0/(L[0]+L[1]+L[2]+L[3])*(95.0-15.0));
					glEnd();
				}else{
					glBegin(GL_QUADS);
						glVertex2f(    10.0+40.0*i, 55.0+RW*(q_res[0]+q_res[4])/2.0/(L[0]+L[1]+L[2]+L[3])*(95.0-15.0));
						glVertex2f(10.0+40.0*(i+1), 55.0+RW*(q_res[0]+q_res[4])/2.0/(L[0]+L[1]+L[2]+L[3])*(95.0-15.0));
						glVertex2f(10.0+40.0*(i+1), 55.0);
						glVertex2f(    10.0+40.0*i, 55.0);
					glEnd();
				}
				
				glColor3fv(black);
				draw_line(10.0, 15.0, 210.0, 15.0);
				draw_line(10.0, 55.0, 210.0, 55.0);
				draw_line(10.0, 95.0, 210.0, 95.0);
				for(i=0; i<=5; i++){
					draw_line(10.0+i*40.0, 15.0, 10.0+i*40.0, 95.0);
				}
				draw_string(80.0, 5.0, "q_res", GLUT_BITMAP_9_BY_15);
				
				// Draw tau_res Bars
				glColor3fv(green);
				for(i=0; i<5; i++){
					if(tau_ref[i]>0.0){
						glBegin(GL_QUADS);
							glVertex2f(   220.0+40.0*i, 55.0);
							glVertex2f(220.0+40.0*(i+1), 55.0);
							glVertex2f(220.0+40.0*(i+1), 55.0+tau_ref[i]/tau_limit[i]*(95.0-15.0));
							glVertex2f(    220.0+40.0*i, 55.0+tau_ref[i]/tau_limit[i]*(95.0-15.0));
						glEnd();
					}else{
						glBegin(GL_QUADS);
							glVertex2f(    220.0+40.0*i, 55.0+tau_ref[i]/tau_limit[i]*(95.0-15.0));
							glVertex2f(220.0+40.0*(i+1), 55.0+tau_ref[i]/tau_limit[i]*(95.0-15.0));
							glVertex2f(220.0+40.0*(i+1), 55.0);
							glVertex2f(    220.0+40.0*i, 55.0);
						glEnd();
					}
				}
				
				glColor3fv(black);
				draw_line(220.0, 15.0, 420.0, 15.0);
				draw_line(220.0, 55.0, 420.0, 55.0);
				draw_line(220.0, 95.0, 420.0, 95.0);
				for(i=0; i<=5; i++){
					draw_line(220.0+i*40.0, 15.0, 220.0+i*40.0, 95.0);
				}
				draw_string(280.0, 5.0, "tau_ref", GLUT_BITMAP_9_BY_15);
				
				// F_res Plot
			glPopMatrix();
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glPopAttrib();
			glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	glPopMatrix();	
}

#endif
