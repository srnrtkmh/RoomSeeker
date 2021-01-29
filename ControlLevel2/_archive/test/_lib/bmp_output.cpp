/**************************************************************************************************/
/*                                                                                                */
/* FILE : bmp_output.c                                                                            */
/* MEMO : output bmp picture file from OpenGL Graphics                                            */
/*                                                                                                */
/* 2010/11/17 : Start to edit this file                                                           */
/*                                                                                                */
/**************************************************************************************************/

//================================================================================================//
// Include Files                                                                                  //
//================================================================================================//
#include <stdio.h>
#include <stdlib.h>
#define GLUT_BUILDING_LIB			// To avoid warning "'int glutCreateWindow_ATEXIT_HACK(const char*)' defined but not used"
#include <GL/glut.h>
#include "bmp_output.h"

//================================================================================================//
// Display Function                                                                               //
//================================================================================================//
int record_bmp(const char* filename, int width, int height)
{
    // �������m��
	GLubyte* pixel_data = (GLubyte*)malloc((width)*(height)*3*(sizeof(GLubyte)));
	
	glReadBuffer(GL_FRONT);														// �ǂݏo���o�b�t�@�̎w��
	glPixelStorei(GL_PACK_ALIGNMENT ,1);	    								// �f�[�^�i�[�̃T�C�Y��ݒ�
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);	// �f�[�^�̓ǂݏo��
	
	// �����Ńf�[�^���r�b�g�}�b�v�t�@�C���֏�������
	WriteBitmap(filename, pixel_data, width, height);
	free(pixel_data);
}

// Copied from "http://code.google.com/p/sonson-code/"
int WriteBitmap(const char* filename, GLubyte* data, int width, int height)
{
	FILE *fp;
	BitmapHeader header;
	BitmapInfoHeader info;
	int i=0;
	int j=0;
	int x;
	int y;
	// �t�@�C���I�[�v��
	if( ( fp = fopen(filename, "wb") )==NULL){
		return -1;
	}
	//�w�b�_�\���̂̏�����
	InitHeaders(&header, &info);
	//Bitmap�T�C�Y
	info.width = width;
	info.height = height;
	int writeWidth;
	
	// �f�[�^�̕��̃o�C�g����4�̔{���ł��邩���`�F�b�N
	if( width*3%4 == 0)
		writeWidth = width*3;
	else
		// �����łȂ���΁C4�̔{���ɂ��킹�����̃o�C�g�T�C�Y�ɂ���
		writeWidth = width*3 + 4 - (width*3)%4;
	//�t�@�C���e��
	header.filesize =
		writeWidth*height		//�r�b�g����
		+ 14					//BitmapHeader�\���̃T�C�Y
		+ 40;					//BitmapInfoHeader�\���̃T�C�Y
	//�w�b�_��������
	WriteHeader(&header,fp);
	WriteInfoHeader(&info,fp);
	
	unsigned char zero=0;
	// �C���[�W�f�[�^��������
	for( y=0 ; y < height ; y++ ){
		// �f�[�^��BGR�̏��ŏ�������
		for( x=0 ; x < width ; x++ ){
			j=fwrite((data+x*3+3*y*width+2),sizeof(GLubyte),1,fp);
			j=fwrite((data+x*3+3*y*width+1),sizeof(GLubyte),1,fp);
			j=fwrite((data+x*3+3*y*width),sizeof(GLubyte),1,fp);
		}
		// ���̃o�C�g����4�̔{���łȂ��Ƃ��͂O�Ŗ��߂�
		if( width*3%4 != 0)
			for(j=0; j<4-(width*3)%4; j++)
				fwrite(&zero,sizeof(GLubyte),1,fp);
	}
	// �t�@�C���N���[�Y
	fclose(fp);
	return 0;
}

// Copied from "http://code.google.com/p/sonson-code/"
void InitHeaders(BitmapHeader *header,BitmapInfoHeader *info)
{
	header->distinct1 = 'B';
	header->distinct2 = 'M';
	header->filesize = 0;
	header->reserve1 = 0;
	header->reserve2 = 0;
	header->offset = 54;
	info->header = 40;
	info->width = 0;
	info->height = 0;
	info->plane = 1;
	info->bits = 24;
	info->compression = 0;
	info->comp_image_size = 0;
	info->x_resolution = 0;
	info->y_resolution = 0;
	info->pallet_num = 0;
	info->important_pallet_num = 0;
};

// Copied from "http://code.google.com/p/sonson-code/"
void WriteHeader(BitmapHeader *header,FILE *fp)
{
	fwrite(&(header->distinct1), sizeof(char),1,fp);
	fwrite(&(header->distinct2), sizeof(char),1,fp);
	fwrite(&(header->filesize), sizeof(int),1,fp);
	fwrite(&(header->reserve1), sizeof(short),1,fp);
	fwrite(&(header->reserve2), sizeof(short),1,fp);
	fwrite(&(header->offset), sizeof(int),1,fp);
}

// Copied from "http://code.google.com/p/sonson-code/"
void WriteInfoHeader(BitmapInfoHeader *info,FILE *fp)
{
	fwrite(&(info->header), sizeof(int),1,fp);
	fwrite(&(info->width), sizeof(int),1,fp);
	fwrite(&(info->height), sizeof(int),1,fp);
	fwrite(&(info->plane), sizeof(short),1,fp);
	fwrite(&(info->bits), sizeof(short),1,fp);
	fwrite(&(info->compression), sizeof(int),1,fp);
	fwrite(&(info->comp_image_size), sizeof(int),1,fp);
	fwrite(&(info->x_resolution), sizeof(int),1,fp);
	fwrite(&(info->y_resolution), sizeof(int),1,fp);
	fwrite(&(info->pallet_num), sizeof(int),1,fp);
	fwrite(&(info->important_pallet_num), sizeof(int),1,fp);
}

