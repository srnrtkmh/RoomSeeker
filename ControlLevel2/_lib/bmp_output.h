/**************************************************************************************************/
/*                                                                                                */
/* FILE : bmp_output.h                                                                            */
/* MEMO : output bmp picture file from OpenGL Graphics                                            */
/*                                                                                                */
/* 2010/11/17 : Start to edit this file                                                           */
/*                                                                                                */
/**************************************************************************************************/

#ifndef BMP_OUTPUT
#define BMP_OUTPUT

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <stdio.h>
#define GLUT_BUILDING_LIB			// To avoid warning "'int glutCreateWindow_ATEXIT_HACK(const char*)' defined but not used"
#include <GL/glut.h>

//================================================================================================//
// Structure Definition                                                                           //
//================================================================================================//
typedef struct _BitmapHeader{
	char	distinct1;			// 'B'
		char	distinct2;		// 'M'
		int	 filesize;			// ���t�@�C���T�C�Y
		short   reserve1;		// �\��̈�E��ɂO
		short   reserve2;		// �\��̈�E��ɂO
		int	 offset;			// �t�@�C���̐擪����f�[�^�܂ł̃I�t�Z�b�g
}BitmapHeader;

typedef struct _BitmapInfoHeader{
	int	 header;				// ���̍\���̂̃T�C�Y
	int	 width;					// Bitmap�̃T�C�Y�E����
	int	 height;				// Bitmap�̃T�C�Y�E�c��
	short   plane;				// plane���E��ɂP
	short   bits;				// Bitmap�̐F��(bit�P��)
	int	 compression;			// ���k����Ă��邩�H
	int	 comp_image_size;		// �摜�S�̂̃T�C�Y�E�g��Ȃ�
	int	 x_resolution;			// Bitmap�̉𑜓x�E�g��Ȃ�
	int	 y_resolution;			// 72�Ɛݒ肵�Ă����Ă����܂�Ȃ�
	int	 pallet_num;			// �p���b�g��
								// 24bit�J���[���̏ꍇ�g��Ȃ�
	int	 important_pallet_num;	// �d�v�ȃp���b�g���E�g��Ȃ�
								// 24bit�J���[���̏ꍇ�g��Ȃ�
}BitmapInfoHeader;

//================================================================================================//
// Function Prototypes                                                                            //
//================================================================================================//
int  record_bmp(const char* filename, int width, int height);
int  WriteBitmap(const char* filename, GLubyte* data, int width, int height);
void InitHeaders(BitmapHeader *header,BitmapInfoHeader *info);
void WriteHeader(BitmapHeader *header,FILE *fp);
void WriteInfoHeader(BitmapInfoHeader *info,FILE *fp);

#endif
