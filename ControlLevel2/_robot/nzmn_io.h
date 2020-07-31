/**************************************************************************************************/
/*                                                                                                */
/* File : nzmn_io.h                                                                               */
/* MEMO : Functions to Control the Interface Board such as counter, d/a, a/d                      */
/*        Comfirm that your main function can access the I/O address to control PCI boards        */
/*                                                                                                */
/* 2010/09/22 : Start to edit this file                                                           */
/* 2010/10/18 : koushin                                                                           */
/* 2010/12/11 : Modify da_init() to reset the da value as the command to motor torque is zero     */
/*              Delete DIO functions because the DIO board is removed from PC                     */
/*              Add "cnt2angle()" and "current2da" functions                                      */
/* 2010/12/13 : Add constant macros and function prototypes about Gyro                            */
/*              "gyro_init()", "gyro_read()" and "gyro2angle()"                                   */
/* 2011/01/06 : Add functions "ad_init()" and "ad_read()" to use A/D board (PCI3133)              */
/* 2011/01/12 : Add function "ad2force()"                                                         */
/* 2011/01/14 : Add function "cnt_write()"                                                        */
/* 2011/09/16 : Rename this file from "ume_header.c" to "ume_header.cpp"                          */
/* 2012/07/17 : Modify function "current2da()" to adjust dead band characteristics of HA-655-SP   */
/* 2012/11/12 : Rename this file from "ume_header.cpp" to "nzmn_io.h"                             */
/*              Changed structure of this project to use global variables                         */
/* 2012/12/18 : Move functions to handle serial port here                                         */
/*                                                                                                */
/**************************************************************************************************/

#ifndef NZMN_IO
#define NZMN_IO

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <sys/io.h>
//#include <rtai.h>
//#include <rtai_lxrt.h>
#include <rtai_serial.h>
#define PAGE_SIZE		0x1000
struct termios oldtio;					// Current serial port settings

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
void cnt_init(void);
void da_init(void);
void cnt_read(int n, int dat_cnt[]);
void cnt_write(double q_res[]);
void da_write(int dat_da[]);
void pci_boards_init(void);
void pci_boards_end(void);
void gyro_init(void);
int gyro_read(int gyro_in[]);
void cnt2angle(int cnt_in[], double q_res[]);
void current2da(double I_cmd[], int da_out[]);
void gyro2angle(int gyro_in[], double q0[]);

// For force sensor
void ifs_init(void);
unsigned short IFS_Read(unsigned int addr);
void IFS_Write(unsigned int addr, unsigned short data);
double ifs_read(int ch);
/*
// For Playstation controller
int open_serial_port(char *modem_dev);	// Open serial port (used for play station controller)
void close_serial_port(int fd);			// Close serial port (used for play station controller)
int get_serial_char(int fd);			// Get a character from serial port (used for play station controller)
*/
//================================================================================================//
// Initialization Functions                                                                       //
//================================================================================================//
// PCI-6205C Initialization
void cnt_init(void)
{
	int i;

	// If the mode register is written, counter is cleared
	outb(0x06, PCI6205C0+0x04);	// CH1
	outb(0x06, PCI6205C0+0x14);	// CH2
	outb(0x06, PCI6205C1+0x04);	// CH3
	outb(0x06, PCI6205C1+0x14);	// CH4
	outb(0x06, PCI6205C2+0x04);	// CH5
	outb(0x06, PCI6205C2+0x14);	// CH6
	outb(0x06, PCI6205C3+0x04);	// CH7
	outb(0x06, PCI6205C3+0x14);	// CH8
	
	for(i=0; i<8; i++) cnt_page[i]=0;
}

// PCI-3329 Initialization
void da_init(void)
{
	int da_out[8]={0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800};
	da_write(da_out);
}


//================================================================================================//
// Read & Write Functions                                                                         //
//================================================================================================//
// Read Counter Value from PCI-6205C
void cnt_read(int n, int dat_cnt[])
{
	switch(n){
		case 1 : 
			outb(0x02, PCI6205C0+0x06);
			dat_cnt[0]=((int)inb(PCI6205C0+0x02)<<16)+((int)inb(PCI6205C0+0x01)<<8)+(int)inb(PCI6205C0+0x00);
			break;
		case 2 : 
			outb(0x02, PCI6205C0+0x16);
			dat_cnt[1]=((int)inb(PCI6205C0+0x12)<<16)+((int)inb(PCI6205C0+0x11)<<8)+(int)inb(PCI6205C0+0x10);
			break;
		case 3 : 
			outb(0x02, PCI6205C1+0x06);
			dat_cnt[2]=((int)inb(PCI6205C1+0x02)<<16)+((int)inb(PCI6205C1+0x01)<<8)+(int)inb(PCI6205C1+0x00);
			break;
		case 4 : 
			outb(0x02, PCI6205C1+0x16);
			dat_cnt[3]=((int)inb(PCI6205C1+0x12)<<16)+((int)inb(PCI6205C1+0x11)<<8)+(int)inb(PCI6205C1+0x10);
			break;
		case 5 : 
			outb(0x02, PCI6205C2+0x06);
			dat_cnt[4]=((int)inb(PCI6205C2+0x02)<<16)+((int)inb(PCI6205C2+0x01)<<8)+(int)inb(PCI6205C2+0x00);
			break;
		case 6 : 
			outb(0x02, PCI6205C2+0x16);
			dat_cnt[5]=((int)inb(PCI6205C2+0x12)<<16)+((int)inb(PCI6205C2+0x11)<<8)+(int)inb(PCI6205C2+0x10);
			break;
		case 7 : 
			outb(0x02, PCI6205C3+0x06);
			dat_cnt[6]=((int)inb(PCI6205C3+0x02)<<16)+((int)inb(PCI6205C3+0x01)<<8)+(int)inb(PCI6205C3+0x00);
			break;
		case 8 : 
			outb(0x02, PCI6205C3+0x16);
			dat_cnt[7]=((int)inb(PCI6205C3+0x12)<<16)+((int)inb(PCI6205C3+0x11)<<8)+(int)inb(PCI6205C3+0x10);
			break;
		default : break;
	}
}

// Write Counter Value to PCI-6205C
void cnt_write(double q_res[])
{
	int i;
	long cnt_val[8];
	double dir[8]={-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0};		// Direction modifying value 
	
	for(i=0; i<6; i++)	cnt_val[i]=(long)(dir[i]*q_res[i]*(MANI_ENC_RSL*ENC_MULTI)*MANI_GEAR_RATIO/2.0/M_PI);
	for(i=6; i<8; i++)	cnt_val[i]=(long)(dir[i]*q_res[i]*(WHEEL_ENC_RSL*ENC_MULTI)*WHEEL_GEAR_RATIO/2.0/M_PI);
	
	for(i=0; i<8; i++){
		cnt_page[i]=0;
		while(cnt_val[i]>0xffffff){
			cnt_val[i]-=0xffffff;
			cnt_page[i]++;
		}
		while(cnt_val[i]<0x000000){
			cnt_val[i]+=0xffffff;
			cnt_page[i]--;
		}
		cnt_in_pre[i]=cnt_val[i];
	}
	outb((cnt_val[0]&0x0000ff), PCI6205C0+0x00); outb((cnt_val[0]&0x00ff00)>>8, PCI6205C0+0x01); outb((cnt_val[0]&0xff0000)>>16, PCI6205C0+0x02);
	outb((cnt_val[1]&0x0000ff), PCI6205C0+0x10); outb((cnt_val[1]&0x00ff00)>>8, PCI6205C0+0x11); outb((cnt_val[1]&0xff0000)>>16, PCI6205C0+0x12);
	outb((cnt_val[2]&0x0000ff), PCI6205C1+0x00); outb((cnt_val[2]&0x00ff00)>>8, PCI6205C1+0x01); outb((cnt_val[2]&0xff0000)>>16, PCI6205C1+0x02);
	outb((cnt_val[3]&0x0000ff), PCI6205C1+0x10); outb((cnt_val[3]&0x00ff00)>>8, PCI6205C1+0x11); outb((cnt_val[3]&0xff0000)>>16, PCI6205C1+0x12);
	outb((cnt_val[4]&0x0000ff), PCI6205C2+0x00); outb((cnt_val[4]&0x00ff00)>>8, PCI6205C2+0x01); outb((cnt_val[4]&0xff0000)>>16, PCI6205C2+0x02);
	outb((cnt_val[5]&0x0000ff), PCI6205C2+0x10); outb((cnt_val[5]&0x00ff00)>>8, PCI6205C2+0x11); outb((cnt_val[5]&0xff0000)>>16, PCI6205C2+0x12);
	outb((cnt_val[6]&0x0000ff), PCI6205C3+0x00); outb((cnt_val[6]&0x00ff00)>>8, PCI6205C3+0x01); outb((cnt_val[6]&0xff0000)>>16, PCI6205C3+0x02);
	outb((cnt_val[7]&0x0000ff), PCI6205C3+0x10); outb((cnt_val[7]&0x00ff00)>>8, PCI6205C3+0x11); outb((cnt_val[7]&0xff0000)>>16, PCI6205C3+0x12);
//	rt_printk("cnt_val = %d, %d, %d, %d, %d, %d, %d, %d\n", cnt_val[0], cnt_val[1], cnt_val[2], cnt_val[3], cnt_val[4], cnt_val[5], cnt_val[6], cnt_val[7]);
//	rt_printk("cnt_page[i] = %d, %d, %d, %d, %d, %d, %d, %d\n", cnt_page[0], cnt_page[1], cnt_page[2], cnt_page[3], cnt_page[4], cnt_page[5], cnt_page[6], cnt_page[7]);
}

// Write DA Value into PCI-3329
void da_write(int dat_da[])
{
	int i;
	
	outb(0x01, PCI3329+0x0b);		// アナログ出力 ON
	outb(0x03, PCI3329+0x0d);		// 全チャンネル同時変換イネーブル
	for(i=0; i<8; i++){
		outb(i, PCI3329+0x02);					// D/Aチャンネル設定
		outb((dat_da[i]&0x00ff), PCI3329+0x00);	// D/A変換データ書き込み(下位)
		outb((dat_da[i]>>8), PCI3329+0x01);		// D/A変換データ書き込み(上位)
	}
	outb(0x01, PCI3329+0x0d);		// 全チャンネル同時変換出力→全チャンネルのD/A電圧出力更新
}

//================================================================================================//
// Sum up the board Initializing and Ending function                                              //
//================================================================================================//
// Initialize all PCI boards
void pci_boards_init(void)
{
	// Initialize Counter
	cnt_init();
	printf("Counter Initialized!!\n");
	
	// Initialize D/A board
	da_init();
	printf("D/A Initialized!!\n");
}

// End to use all pci boards
void pci_boards_end(void)
{
	// Reset D/A board
	da_init();
	printf("DA Reset!!\n");
}

//================================================================================================//
// Functionos to control gyro sensor                                                              //
//================================================================================================//
void gyro_init(void)
{
	// initialization of Serial Port to read Gyro value
	if (rt_spopen(COM_PORT, 19200, 7, 1, RT_SP_PARITY_EVEN, RT_SP_NO_HAND_SHAKE, RT_SP_FIFO_SIZE_4) < 0) {
		printf("serial rt_spopen error\n");
		pci_boards_end();
		exit(1);
	}
}

int gyro_read(int gyro_in[])
{
	int buf_exist = 0, gyro_set_flag = 0, i, j;
	static char serial_rc_buf[1000]={0x00}, serial_rc_tmp[1000]={0x00};
	static int  serial_rc_ptr=0;
	
	// Check Serial Port
	buf_exist = rt_spget_rxavbs(COM_PORT);
	if(buf_exist>0){
		rt_spread_timed(COM_PORT, serial_rc_tmp, buf_exist, DELAY_FOREVER);
		for(i=0; i<buf_exist; i++)	serial_rc_buf[serial_rc_ptr++]=serial_rc_tmp[i];
	}
	
	for(i=0; i<serial_rc_ptr; i++){
		// Search character '$' to sort the array "serial_rc_buf"
		if(serial_rc_buf[i]=='$'){
			// Sort the array as the character '$' is at the top of the buffer
			for(j=0; j<serial_rc_ptr-i; j++){
				serial_rc_buf[j]=serial_rc_buf[i+j];
			}
			serial_rc_ptr=serial_rc_ptr-i;
			
			// reload gyro value if gyro sensor returns enough data
			if(serial_rc_ptr>=8){
				gyro_set_flag = 1;	// Enable gyro_set_flag
				for(j=1; j<=8; j++){
					if(('0'<=serial_rc_buf[j])&&(serial_rc_buf[j]<='9'))		serial_rc_buf[j]-='0';
					else if(('A'<=serial_rc_buf[j])&&(serial_rc_buf[j]<='F'))	serial_rc_buf[j]+=10-'A';
				}
				gyro_in[0] =(int)serial_rc_buf[1]*16*16*16;
				gyro_in[0]+=(int)serial_rc_buf[2]*16*16;
				gyro_in[0]+=(int)serial_rc_buf[3]*16;
				gyro_in[0]+=(int)serial_rc_buf[4];
				gyro_in[1] =(int)serial_rc_buf[5]*16*16*16;
				gyro_in[1]+=(int)serial_rc_buf[6]*16*16;
				gyro_in[1]+=(int)serial_rc_buf[7]*16;
				gyro_in[1]+=(int)serial_rc_buf[8];
//				if((gyro_in[1]&&0x8000)!=0)	gyro_in[1]=-(0x10000-gyro_in[1]);	// If anguler velocity is negative value, transformation is needed
//				else						gyro_in[1]=(0x10000-gyro_in[1]);	// If anguler velocity is negative value, transformation is needed
				if(gyro_in[1]>=0x8000)		gyro_in[1]=-(0x10000-gyro_in[1]);	// If anguler velocity is negative value, transformation is needed
				serial_rc_buf[0]='X';						// Change the top of the buffer from '$' to any character
			}
			break;
		}
	}
	return(gyro_set_flag);
}

//================================================================================================//
// Transform natural variable to digital value for each board                                     //
//================================================================================================//
// Transform counter data [cnt] to joint angle [rad]
void cnt2angle(int cnt_in[], double q_res[])
{
	int i;
	double dir[8]={-1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0};		// Direction modifying value 
	
	for(i=0; i<8; i++){
		// If overflow or underflow happened, page variable is incremented or decremented
		if((cnt_in[i]-cnt_in_pre[i])>0x800000)			cnt_page[i]--;
		else if((cnt_in[i]-cnt_in_pre[i])<-0x800000)	cnt_page[i]++;
		
		// Store the current counter value to previous one
		cnt_in_pre[i]=cnt_in[i];
	}
	for(i=0; i<6; i++){	
		// Substitute joint angle
		q_res[i]=dir[i]*((double)(cnt_page[i]*0x1000000)+(double)cnt_in[i])*(2.0*M_PI)/(MANI_ENC_RSL*ENC_MULTI)/MANI_GEAR_RATIO;
	}
	for(i=6; i<8; i++){
		q_res[i]=dir[i]*((double)(cnt_page[i]*0x1000000)+(double)cnt_in[i])*(2.0*M_PI)/(WHEEL_ENC_RSL*ENC_MULTI)/WHEEL_GEAR_RATIO;
	}
}

// Transform current command [A] to digital value for DA [?]
void current2da(double I_cmd[], int da_out[])
{
	int i;
	static int dir[8]={1, 1, -1, -1, -1, -1, -1, 1};
	
	for(i=0; i<8; i++){
		// Transform current value to D/A digital one
		da_out[i] = dir[i]*(int)(I_cmd[i]*DA_RSL / (I_MAX[i] - I_MIN[i])) + DA_OFFSET;
		
		// If the digital value override the limitation, the digital value is saturated
		if(da_out[i]>DA_MAX)		da_out[i]=DA_MAX;
		else if(da_out[i]<DA_MIN)	da_out[i]=DA_MIN;
	}
}

// Transform gyro data [binary] to passive joint angle [rad]
void gyro2angle(int gyro_in[], double q0[])
{
	static int pre_gyro_in=0.0;			// Previous value of the gyro
	static int gyro_page=0;				// Page for gyro value
	static double dir[2]={1.0, 1.0};	// Direction modifying value 
	
	// Reload page number from previous and current gyro value
	// If overflow or underflow happened, page variable is incremented or decremented
	if((gyro_in[0]-pre_gyro_in)>0x8000)			gyro_page--;
	else if((gyro_in[0]-pre_gyro_in)<-0x8000)	gyro_page++;
	
	// Store the previous gyro value
	pre_gyro_in=gyro_in[0];
	
	// Substitute joint angle
	q0[0]=dir[0]*((double)(gyro_page*0x10000)+(double)gyro_in[0])*(2.0*M_PI)/65536.0;
	q0[1]=dir[1]*gyro_in[1]*(2.0*M_PI)/65536.0;
}

//================================================================================================//
// Functionos to read force sensor IFS-100M40A50-I63 manufactured by nitta                        //
//================================================================================================//
// Initialize force sensor
void ifs_init(void)
{
	int fd;
	int err = 0;
	int fdMem;
	short *mmaped;
	unsigned long ifs_base_addr;
	unsigned long st, len, poff;

	fd = open("/dev/jr3chars", O_RDWR);
	if (fd < 0) printf("open\n");

	err = ioctl(fd, JR3_GET_BADDR, &ifs_base_addr);
	if (err < 0){
		err = close(fd);
		printf("ioctl error\n");
		exit(0);
	}
	st = ifs_base_addr;
	len = ((0xff | Jr3DmAddrMask) << 2)+2;
	poff = st % PAGE_SIZE;
	fdMem = open("/dev/mem", O_RDWR);

	mmaped = (short *)mmap(0, len+poff, PROT_READ|PROT_WRITE, MAP_SHARED, fdMem, st-poff);
	if (mmaped == MAP_FAILED){
		fprintf(stderr, "cannot mmap\n");
		exit(0);
	}
	else{
		MappedAddress = (unsigned long)mmaped;
	}
}

// Read force sensor value manufactured by nitta
unsigned short IFS_Read(unsigned int addr)
{
	unsigned short data = 0;
	unsigned long address = 0;
	
	address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
	data = *(unsigned short *)address;
	return data;
}

// Read force sensor value manufactured by nitta
void IFS_Write(unsigned int addr, unsigned short data)
{
	unsigned long address = 0;
	
	address = MappedAddress + ((addr | Jr3DmAddrMask) << 2);
	*(unsigned short *)address = data;
}

// Read force sensor 
// ch = 0:Fx, 1:Fy, 2:Fz, 3:Mx, 4:My, 5:Mz
// ima ha ch 2 (Fz) shika tsukaenai
double ifs_read(int ch)
{
	int data;
	double ret;
	
	IFS_Write(0x80, 0x1111);
	data = IFS_Read(ch+0x90);
	if(data & 0x8000) data=data-0x10000;
	ret = IFS_A*(double)data + IFS_B;
	
	return(ret);
}
/*
//================================================================================================//
// Open serial port                                                                               //
//================================================================================================//
int open_serial_port(char *modem_dev)
{
	struct termios newtio;
	int fd;
	
	// Open modem device for reading and writing and not as controlling tty
	fd = open(modem_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(fd < 0){
		perror(modem_dev);
		exit(-1);
	}
	tcgetattr(fd,&oldtio);

	// Clear the struct for new port settings
	newtio.c_iflag = 0;
	newtio.c_oflag = 0;
	newtio.c_cflag = 0;
	newtio.c_lflag = 0;
	newtio.c_line = 0;
	bzero(newtio.c_cc, sizeof(newtio.c_cc));

	// Settings for new port
	newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD ;
	newtio.c_iflag = IGNPAR;	// IGNPAR : ignore bytes with parity errors
	newtio.c_oflag = 0;			// Raw output (Rawモードでの出力)
	newtio.c_lflag = 0;			// Set input mode (non-canonical,no echo,....)
	newtio.c_cc[VTIME] = 0; 	// Amout of time to wait for incoming characters in tenth of seconds
	newtio.c_cc[VMIN] = 0;		// Minimum number of characters to read
	
	// Now clean the modem line and activate the settings for the port
	tcflush(fd,TCIFLUSH);
	// Apply new settings
	tcsetattr(fd,TCSANOW,&newtio);

	return(fd);
}

//================================================================================================//
// Close serial port                                                                              //
//================================================================================================//
void close_serial_port(int fd){
	// Restore the old port settings
	tcsetattr(fd, TCSANOW,&oldtio);
	close(fd);
}

//================================================================================================//
// Get a character from current serial port                                                       //
//================================================================================================//
int get_serial_char(int fd){
	unsigned char c = 0;
//	unsigned char c = 0xff;
	int res;
	res = read(fd, (char *)&c, 1);
	return(c);
}
*/
#endif

