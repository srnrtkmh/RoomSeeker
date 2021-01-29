/**************************************************************************************************/
/*                                                                                                */
/* FILE        : mecanum_car_sim.cpp                                                              */
/* MEMO        : Trajectory tracking simulation program of mecanum car                            */
/*                                                                                                */
/* UPDATE LOG  : 2020/7/24                                                                        */
/*                                                                                                */
/* 2020/07/24 : Established this project                                                          */
/*                                                                                                */
/*                                                                                                */
/**************************************************************************************************/

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
// GCC standard library
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <pthread.h>
#include <GL/glut.h>
#include <time.h>
#include <termios.h>
#include <errno.h>

// Original header file
#include "exe_opt.h"				// Executing options
#include "_lib/basic_calc.h"		// Supply basic mathematics functions not included in "math.h"
#include "_lib/basic_matrix.h"		// Supply basic matrix class
#include "_lib/basic_figure.h"		// Drawing basic figure by OpenGL
#include "_lib/basic_opengl.h"		// Supply basic structure to control GLUT fucntions
#include "_lib/colors.h"			// Color profile for OpenGL

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
int main(int argc, char *argv[]);		// Main function
static void catch_signal(int sig);		// Signal handler
static void *opengl_thread(void *arg);	// Independent thread to display OpenGL graphics
void display_func(void);				// Function registered for displaying OpenGL graphics

//================================================================================================//
//================================================================================================//
// Global Variables                                                                               //
//================================================================================================//
// File pointer to record simulation result
FILE *fp_res, *fpfp_res, *fp_stick;

// Flags used in simulation
volatile int timer_flag;		// Flag to select whether to increment the timer in simulation
volatile int seq_flag;			// Flag to transmit sequence forward signal from keyboard read by glut
volatile int terminate_flag;	// Flag to transmit termination signal from keyboard (read by glut in simulation)

// Used in network communication
int sock, sock0, len, n, yes = 1, val = 1, revd_size = 0;
struct sockaddr_in addr;
struct sockaddr_in client;
int connection_flag = 0;
char buf[2048];

// Time string for naming file name 
time_t timer;
struct tm *date;
char str[256];

//================================================================================================//
// Main Routine                                                                                   //
//================================================================================================//
int main(int argc, char *argv[]){
	// Initialize Variables ----------------------------------------------------------------------//
	init_variables_common();	// Initialize variables that is common among all projects
	data_cnt++;			// Increment data counter
	printf("Complete to initialize variables\n");
	
	// Open file to write result -----------------------------------------------------------------//
	if((fp_res = fopen("./_data/data.csv", "w")) == NULL){       printf("error!\ndata.csv cannot open. \n");  exit(1); }	// For response data
	timer = time(NULL);				// Get current time
	date = localtime(&timer);		// Transform current time to the structure
	strftime(str, 255, "./_data/at_%y%m%d_%H%M.csv", date);
	if((fpfp_res = fopen(str, "w")) == NULL){ printf("error!\n%s cannot open. \n", str); exit(1); }
	printf("Complete to open files.\n");
	
	// Register the function executed when the signal is catched ---------------------------------//
	signal(SIGTERM, catch_signal);
	signal(SIGINT,  catch_signal);
	signal(SIGKILL, catch_signal);
	printf("Complete to set signal trap function.\n");
	
	// Main periodic loop ------------------------------------------------------------------------//
	while(!terminate_flag){
		// Wait until next sampling time
		#ifdef SLOW
			usleep(2000);
		#elif defined FAST
			usleep(100);
		#elif defined NO_DELAY
			;
		#else
			usleep(800);
		#endif
		if(t > 300.0 || terminate_flag == 1) break;	// Escape from this loop if time is over or terminate flag is set
		if(timer_flag == 0) continue;				// Skip if draw flag is reset
		
		// Periodic control loop (skipped in test mode)
		t += dt;
		mst_cnt++;
		
		reload_input();			// Measure the sensor values and reload each variable (input the counter and gyro value)
		reload_dis();			// Reload disturbance variables
		reload_reac();			// Reload reaction variables
		reload_nom_model();		// Reload parameter of kinematics and nominal dynamics model element
		reload_cmd_ref();		// Set the command and reference value
		reload_torque();		// Reload torque input with base torque dimension designed controller
		reload_output();		// Reload output value of the motor torque (output the command to the motor driver)
		
		// Apply disturbance -> Calculate response in simulation -> Apply noise
		apply_dis();
		reload_res_free();
		apply_nis();
		
		// Save result to prepared variables
		if(output_flag == 1){
			// Save main data every 0.01s
			if(data_cnt < DATA_SIZE){
				if((int)(t/dt)%(int)(0.02/dt) == 0){
					q_res.x[0] = q_res[0] - q0_off_dis + q0_off_hat;
					output_tmp();
					q_res.x[0] = q_res[0] + q0_off_dis - q0_off_hat;
					data_cnt++;
				}
			}else{
				output_flag = 0;
			}
			// Save stick diagram data every 0.2s
			if(stick_cnt < DATA_SIZE){
				if((int)(t/dt)%(int)(0.2/dt) == 0){
					output_tmp_stick();
					stick_cnt++;
				}
			}else{
				output_flag = 0;
				#ifdef EXP
					rt_printk("\n\n!!!!!!!!!! DATA FULL !!!!!!!!!!\n\n");
				#endif
			}
		}
		
		// Display various variable
		disp();
	}
	catch_signal(0);					// Execute reset procedure of the equipment before finish the program
	
	return(0);
}

//================================================================================================//
// Procedure when catch the signal such as "Ctrl + C"                                             //
//================================================================================================//
static void catch_signal(int sig)
{
	#ifdef EXP
		char tmp_ch;
		
		// Reset current command as the motor torque is 0
		da_init();
		printf("\nD/A value initialized!\n");
		
		// Close serial port for PSC
		tcsetattr(fd, TCSANOW,&oldtio);	// Restore the old port settings
		close(fd);
		printf("Serial port for play station controller was closed\n");
		
/*		// Close serial port for datatec gyro sensor
		tcsetattr(fd2, TCSANOW,&oldtio2);
		close(fd2);
		printf("Serial port for datatec gyro sensor was closed\n");
*/		
		// Send Command to gyro
		rt_spwrite_timed(COM_PORT, &(tmp_ch = 'S'), 1, DELAY_FOREVER);
		rt_spclose(COM_PORT);
		printf("Serial Port for gyro sensor was closed\n");
	#endif
	
	// Record experimental result to file
	printf("Recording process starts\n");
	output_tmp();					// Write experimental result to prepared variables
	output_file(fp_res);			// Write experimental data into the file
	output_file_stick(fp_stick);	// Write stick diagram data into the file
	printf("Recording process finished\n");
	
	// Close the file recorded experimental result
	fclose(fp_res);
	printf("exp_res.csv was closed\n");
	fclose(fp_stick);
	printf("exp_stick.csv was closed\n");
	
	// Post processing after main control loop is finished
	post_processing();
	
	#ifdef EXP
		#ifdef NETWORK_CMD
			// End socket in listening
			close(sock);
			close(sock0);
		#endif
		
		// Clear key reading thread
		terminate_flag = 1;
		printf("Press any key to end the sub threads. \n");
		pthread_join(pkey, NULL);
		pthread_join(ppsc, NULL);
	#endif
	
	exit(0);
}

#ifdef EXP
//================================================================================================//
// Read keyboard input while real time processing executing                                       //
//================================================================================================//
static void *key_read(void *arg)
{
	char ch, tmp_ch;
	static struct termios oldt, newt;
	RT_TASK *key;
	
	// tcgetattr gets the parameters of the current terminal
	// STDIN_FILENO will tell tcgetattr that it should write the settings of stdin to oldt
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;								// now the settings will be copied
	newt.c_lflag &= ~(ICANON);					// ICANON normally takes care that one line at a time will be processed that means it will return if it sees a "\n" or an EOF or an EOL
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);	// Those new settings will be set to STDIN. TCSANOW tells tcsetattr to change attributes immediately.
	
	// Create real time task for key reading function
	key = rt_task_init_schmod(nam2num("KEY"), 1, 0, 0, SCHED_FIFO, 0xF);
	mlockall(MCL_CURRENT | MCL_FUTURE);
	rt_sleep(nano2count(KEY_READ_PERIOD));			// Wait until main thread finishes initializing process
	printf("\nKeyboard reading task starting\n");
	
	// Periodic loop to read keyboard input
	while(!terminate_flag) {
		printf("Command : ");
		ch=getchar();
		printf("\n");
		
		// Increment sequence number and clear sequence compeltion flag
		if(ch=='k' && seq_comp==1){
			seq++;
			seq_comp=0;
			printf("Sequence Number = %d. Please input 'k' to go ahead the sequence\n", seq);
		}
		// Go back to the origin whenever 'b' is input
		else if(ch == 'b'){
			seq = 100;
			seq_comp = 0;
		}
		// Toggle DOB
		else if(ch=='d'){
			if(dob_flag==1){
				dob_flag=0;
				printf("DOB flag is reset\n");
			}
			else{
				dob_flag=1;
				tau_dis_hat.clear();
				printf("DOB flag is set\n");
			}
		}
		// Toggle PADO
		else if(ch=='p'){
			if(pado_flag==1){
				pado_flag=0;
				printf("PADO flag is reset\n");
			}
			else{
				pado_flag=1;
				tau_dis_hat.x[0] = 0.0;
				printf("PADO flag is set\n");
			}
		}
		// Toggle WOB
		else if(ch=='w'){
			if(wob_flag==1){
				wob_flag=0;
				printf("WOB flag is reset\n");
			}
			else{
				wob_flag=1;
				ddxee_dis.clear();
				printf("WOB flag is set\n");
			}
		}
		// Toggle RTOB
		else if(ch=='r'){
			if(rtob_flag==1){
				rtob_flag=0;
				printf("RTOB flag is reset\n");
			}
			else{
				rtob_flag=1;
				Fee_reac_hat.clear();
				Fa_reac_hat.clear();
				printf("RTOB flag is set\n");
			}
		}
		// Toggle friction compensation
		else if(ch == 'f'){
			if(fc_flag == 1){
				fc_flag = 0;
				printf("Friction compensation flag is set\n");
			}else{
				fc_flag = 1;
				printf("Friction compensation flag is reset\n");
			}
		}
		// Toggle output_flag to select whether output data to temporary memory
		else if(ch == 's'){
			if(output_flag == 0){
				output_flag = 1;
				printf("Now, start recording!!\n");
			}
			else{
				output_flag = 0;
				printf("Stop recording.\n");
			}
		}
		// Toggle soft servo on flag
		else if(ch == '1'){
			if(srv_on_flag[0] == 0) srv_on_flag[0] = 1;
			else                       srv_on_flag[0] = 0;
		}else if(ch == '2'){
			if(srv_on_flag[1] == 0) srv_on_flag[1] = 1;
			else                       srv_on_flag[1] = 0;
		}else if(ch == '3'){
			if(srv_on_flag[2] == 0) srv_on_flag[2] = 1;
			else                       srv_on_flag[2] = 0;
		}else if(ch == '4'){
			if(srv_on_flag[3] == 0) srv_on_flag[3] = 1;
			else                       srv_on_flag[3] = 0;
		}else if(ch == '5'){
			if(srv_on_flag[4] == 0) srv_on_flag[4] = 1;
			else                       srv_on_flag[4] = 0;
		}else if(ch == '6'){
			if(srv_on_flag[5] == 0) srv_on_flag[5] = 1;
			else                       srv_on_flag[5] = 0;
		}else if(ch == '7'){
			if(srv_on_flag[6] == 0) srv_on_flag[6] = 1;
			else                       srv_on_flag[6] = 0;
		}else if(ch == '8'){
			if(srv_on_flag[7] == 0) srv_on_flag[7] = 1;
			else                       srv_on_flag[7] = 0;
		}
		// Start offset canceling of the gyro
		else if(ch=='O'){
			rt_spwrite_timed(COM_PORT, &(tmp_ch = 'O'), 1, DELAY_FOREVER);
			printf("Start gyro bias sampling\n");
		}
		// End offset canceling of the gyro and restart measuring angular variables
		else if(ch=='F'){
		rt_spwrite_timed(COM_PORT, &(tmp_ch = 'F'), 1, DELAY_FOREVER);
			printf("End gyro bias sampling\n");
			printf("Restart sampling angular position and velocity\n");
		}
		// Clear position output of the gyro
		else if(ch=='C'){
			rt_spwrite_timed(COM_PORT, &(tmp_ch = 'C'), 1, DELAY_FOREVER);
			printf("Gyro value is cleared\n");
		}
		
		rt_sleep(nano2count(KEY_READ_PERIOD));			// Wait until next key reading process
	}
	rt_task_delete(key);								// delete real time task to read keyboard input
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);		    // restore the old settings of terminal
	printf("Keyboard reading task ends itself\n");
	return(0);
}

#ifdef SIM
//================================================================================================//
// OpenGL thread                                                                                  //
//================================================================================================//
static void *opengl_thread(void *arg)
{
	int argc = 0;
	char *argv[1];
	
	printf("\nOpenGL task is starting\n");
	
	// Initialize OpenGL Libraries
	glutInit(&argc, argv);	// Initialization of glut
	display_init();			// Initialization of display
	light_init();			// Initialization of light
	event_init();			// Initialization of event function
	glutDisplayFunc(display_func);
	glutIdleFunc(idle_func);
	
	// Start of main loop to draw OpenGL graphics
	glutMainLoop();
	
	return(0);
}

//================================================================================================//
// Display Function                                                                               //
//================================================================================================//
void display_func(void){
	#ifdef GRAPHICS_FREE
		nozamani_in_free();
	#endif
	#ifdef GRAHPICS_WITH_WALL
		nozamani_in_wall_pushing(&rb);
	#endif
}
#endif
