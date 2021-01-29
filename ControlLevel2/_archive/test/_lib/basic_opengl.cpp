/**************************************************************************************************/
/*                                                                                                */
/* FILE        : basic_opengl.cpp                                                                 */
/* MEMO        : Basic Functions to Control OpenGL (the dimension of length is [m])               */
/* LAST UPDATE : 2011/12/04                                                                       */
/*                                                                                                */
/* 2010/10/01 : Start to make this file                                                           */
/* 2010/11/17 : Port "idle_func()" and "timer_func()" from main source                            */
/*              to toggle the timer start or toggle                                               */
/* 2011/09/03 : Rename this file "basic_opengl.cpp" from "basic_opengl.c"                         */
/* 2011/09/10 : Remove file close process and add new flag "terminate flag" is defined            */
/*              Processes after "Esc" key is pressed is done in the display function              */
/* 2011/09/10 : Add "seq_flag" to pass key input from GLUT to display function                    */
/* 2011/12/04 : Rename timer_stop -> timer_flag and make its logic inverted                       */
/* 2012/09/22 : Add capturing process to output OpneGL graphics into bitmap file                  */
/*                                                                                                */
/**************************************************************************************************/

#define TIMER

/**************************************************************************************************/
/* Include Files                                                                                  */
/**************************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define GLUT_BUILDING_LIB			// To avoid warning "'int glutCreateWindow_ATEXIT_HACK(const char*)' defined but not used"
#include <GL/glut.h>
#include "basic_opengl.h"
#include "bmp_output.h"

/**************************************************************************************************/
/* Global Variables                                                                               */
/**************************************************************************************************/
// OpenGL light position
GLfloat light0pos[4];	// Position of light0 source
GLfloat light1pos[4];	// Position of light1 source

// OpenGL perspective data (camera position [m] & target position [m])
double planex, planey, planez;								// Observer's point location
double planex_min, planex_max;								// Limitation of Observer Location (X axis)
double planey_min, planey_max;								// Limitation of Observer Location (Y axis)
double planez_min, planez_max;								// Limitation of Observer Location (Z axis)
double targetx, targety, targetz;

extern int timer_flag;		// Flag to select whether to increment the timer in simulation
extern int seq_flag;		// Flag to transmit sequence forward signal from keyboard read by glut
extern int terminate_flag;	// Flag to transmit termination signal from keyboard read by glut

/**************************************************************************************************/
/* Display Initialization                                                                         */
/**************************************************************************************************/
void display_init(void)
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);	// (double buffer | RGBA mode | z buffer)
	glutInitWindowSize(WX_SIZE, WY_SIZE);						// window size(length,width)(pixel)
	glutInitWindowPosition(80, 100);							// window position(pixel)
	glutCreateWindow("Nozamani Simulation");					// creation of the window (title of the window)
	
//	glClearColor(1.0, 1.0, 1.0, 0.0);							// color after the execution of glClear(white)
	glClearColor(0.0, 0.0, 0.0, 1.0);							// color after the execution of glClear(black)
	
	glEnable(GL_NORMALIZE);										// normalization of the normal vector
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	
	// execution of smooth shading
/*	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	
	glShadeModel(GL_SMOOTH);
*/	
	// OpenGL Perspective Configuration
	planex = -1.0; planey = 4.0;planez = 1.8;	// Observer's point location
	planex = 1.0; planey = 4.0;planez = 1.8;	// Observer's point location
	planex_min=-50.0; planex_max=50.0;			// Limitation of Observer Location (X axis)
	planey_min=-50.0; planey_max=50.0;			// Limitation of Observer Location (Y axis)
	planez_min=0.1; planez_max=50.0;			// Limitation of Observer Location (Z axis)
	targetx=0.0; targety=0.0; targetz=0.4;
	
	// OpenGL Light Configuration
	light0pos[0]=0.0;  light0pos[1];      light0pos[2]=100.0; light0pos[3]=1.0;		// Position of light0 source
	light1pos[0]=20.0; light1pos[1]=30.0; light1pos[2]=20.0;  light1pos[3]=1.0;		// Position of light1 source

	printf("Display Initialization Complete!\n");
}

/**************************************************************************************************/
/* Initialization of Light in OpenGL Graphics                                                     */
/**************************************************************************************************/
void light_init(void)
{
	GLfloat white[4] = { 1.000000, 1.000000, 1.000000, 1.0}; 
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
	glLightfv(GL_LIGHT1, GL_SPECULAR, white);
	glLightfv(GL_LIGHT0, GL_POSITION, light0pos);			// Setting of Light0 Position
	glLightfv(GL_LIGHT1, GL_POSITION, light1pos);			// Setting of Light1 Position
	
	printf("Light Initialization Complete!\n");
}

/**************************************************************************************************/
/* Setting functions for each event                                                               */
/**************************************************************************************************/
void event_init(void)
{
	glutReshapeFunc(resize_func);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutKeyboardFunc(keyboard_func);
	
	printf("Event Registration Complete!\n");
}

/**************************************************************************************************/
/* Set variables                                                                    */
/**************************************************************************************************/
void pre_draw(void)
{
	// Setting perspective property and light source
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear buffers
	
	// 3D Drawing ################################################################################//
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLoadIdentity();
	gluLookAt(planex, planey, planez, targetx, targety, targetz, 0.0, 0.0, 1.0);
}

/**************************************************************************************************/
/* Event when Window is Resized                                                                   */
/**************************************************************************************************/
void resize_func(int w, int h)
{
	glViewport(0, 0, w, h);									// Viewport range is over the whole window
	
	glMatrixMode(GL_PROJECTION);							// Setting projection transformation matrix
	glLoadIdentity();										// Initialize transformation matrix
	gluPerspective(30.0, (double)w/(double)h, 0.1, 100.0);	// Angle : 30deg, Aspect : Fit the window
	
	glMatrixMode(GL_MODELVIEW);								// Setting modelview transformation matrix
	glLoadIdentity();										// Initialize transformation matrix
	gluLookAt(planex, planey, planez, targetx, targety, targetz, 0.0, 0.0, 1.0);	// Perspective position and direction
}

/**************************************************************************************************/
/* Event at Mouse Input                                                                           */
/**************************************************************************************************/
void mouse_func(int button, int state, int x, int y)
{
/*	switch(button){
		case GLUT_LEFT_BUTTON:
			break;
			
		case GLUT_RIGHT_BUTTON:
			break;
			
		default: break;
	}
*/
}

/**************************************************************************************************/
/* Event at Mouse Drug Input                                                                      */
/**************************************************************************************************/
void motion_func(int x, int y)
{
	
}

/**************************************************************************************************/
/* Event at Keyboard Input                                                                        */
/**************************************************************************************************/
void keyboard_func(unsigned char key, int x, int y)
{
	double tmpx, tmpy, tmpz;
	char filename[256];
	static int num = 0;
	
	switch (key) {
		case 'x':
			planex  += 0.1;
			if(planex>planex_max)	planex=planex_max;
			break;
		case 'X':
			planex  -= 0.1;
			if(planex<planex_min)	planex=planex_min;
			break;
		case 'y':
			planey  += 0.1;
			if(planey>planey_max)	planey=planey_max;
			break;
		case 'Y':
			planey  -= 0.1;
			if(planey<planey_min)	planey=planey_min;
			break;
		case 'z':
			planez  += 0.1;
			if(planez>planez_max)	planez=planez_max;
			break;
		case 'Z':
			planez  -= 0.1;
			if(planez<planez_min)	planez=planez_min;
			break;
		case 'r':
			tmpy=planey;
			tmpz=planez;
			planey = tmpy*cos(M_PI/180.0*10.0) - tmpz*sin(M_PI/180.0*10.0);
			planez = tmpy*sin(M_PI/180.0*10.0) + tmpz*cos(M_PI/180.0*10.0);
			break;
		case 'R':
			tmpy=planey;
			tmpz=planez;
			planey = tmpy*cos(-M_PI/180.0*10.0) - tmpz*sin(-M_PI/180.0*10.0);
			planez = tmpy*sin(-M_PI/180.0*10.0) + tmpz*cos(-M_PI/180.0*10.0);
			break;
		case 'p':
			tmpz=planez;
			tmpx=planex;
			planez = tmpz*cos(M_PI/180.0*10.0) - tmpx*sin(M_PI/180.0*10.0);
			planex = tmpz*sin(M_PI/180.0*10.0) + tmpx*cos(M_PI/180.0*10.0);
			break;
		case 'P':
			tmpz=planez;
			tmpx=planex;
			planez = tmpz*cos(-M_PI/180.0*10.0) - tmpx*sin(-M_PI/180.0*10.0);
			planex = tmpz*sin(-M_PI/180.0*10.0) + tmpx*cos(-M_PI/180.0*10.0);
			break;
		case 'h':
			tmpx=planex;
			tmpy=planey;
			planex = tmpx*cos(M_PI/180.0*10.0) - tmpy*sin(M_PI/180.0*10.0);
			planey = tmpx*sin(M_PI/180.0*10.0) + tmpy*cos(M_PI/180.0*10.0);
			break;
		case 'H':
			tmpx=planex;
			tmpy=planey;
			planex = tmpx*cos(-M_PI/180.0*10.0) - tmpy*sin(-M_PI/180.0*10.0);
			planey = tmpx*sin(-M_PI/180.0*10.0) + tmpy*cos(-M_PI/180.0*10.0);
			break;
		case 'i':
			if(sqrt(planex*planex + planey*planey + planez*planez) >=0.5){
				planex*=0.9;
				planey*=0.9;
				planez*=0.9;
			}
			break;
		case 'o':
			if(sqrt(planex*planex + planey*planey + planez*planez) <= 500.0){
				planex*=1.1;
			}
			break;
		case 't':
			if(sqrt(planex*planex + planey*planey + planez*planez) <= 500.0){
				targetz += 0.1;
			}
			break;
		case 'T':
			if(sqrt(planex*planex + planey*planey + planez*planez) <= 500.0){
				targetz -= 0.1;
			}
			break;
		case 'k':
			seq_flag = 1;
			break;
		case 'c':
			sprintf(filename, "Capture%02d.bmp", num);
			record_bmp(filename, WX_SIZE, WY_SIZE);
			num++;
			break;
		case 's':
		case 'S':
			if(timer_flag == 1)	timer_flag = 0;
			else{
				timer_flag = 1;		// Clear timer_flag flag
				#ifdef TIMER
					glutTimerFunc(33, timer_func, 0);
				#else
					glutIdleFunc(idle_func);
				#endif
			}
			break;
		case 'q':
		case 'Q':
		case '\033':	// '\033' ? ESC ? ASCII O[e
		case 0x03:		// 0x03 is 'Ctrl + C' in ASCII code
			if(terminate_flag == 0) terminate_flag = 1;
		default : break;
	}
	
	glutPostRedisplay();
}

//================================================================================================//
// Periodic Timer Event                                                                           //
//================================================================================================//
void timer_func(int value)
{
	if(timer_flag == 1){
		glutTimerFunc(33, timer_func, 0);
		glutPostRedisplay();
	}
}

//================================================================================================//
// Event in Idling                                                                                //
//================================================================================================//
void idle_func(void)
{
	if(timer_flag == 1) glutPostRedisplay();
	else                glutIdleFunc(NULL);
}

