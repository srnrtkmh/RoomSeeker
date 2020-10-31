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
    // メモリ確保
	GLubyte* pixel_data = (GLubyte*)malloc((width)*(height)*3*(sizeof(GLubyte)));
	
	glReadBuffer(GL_FRONT);														// 読み出すバッファの指定
	glPixelStorei(GL_PACK_ALIGNMENT ,1);	    								// データ格納のサイズを設定
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);	// データの読み出し
	
	// ここでデータをビットマップファイルへ書き込む
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
	// ファイルオープン
	if( ( fp = fopen(filename, "wb") )==NULL){
		return -1;
	}
	//ヘッダ構造体の初期化
	InitHeaders(&header, &info);
	//Bitmapサイズ
	info.width = width;
	info.height = height;
	int writeWidth;
	
	// データの幅のバイト数が4の倍数であるかをチェック
	if( width*3%4 == 0)
		writeWidth = width*3;
	else
		// そうでなければ，4の倍数にあわせた幅のバイトサイズにする
		writeWidth = width*3 + 4 - (width*3)%4;
	//ファイル容量
	header.filesize =
		writeWidth*height		//ビット情報量
		+ 14					//BitmapHeader構造体サイズ
		+ 40;					//BitmapInfoHeader構造体サイズ
	//ヘッダ書き込み
	WriteHeader(&header,fp);
	WriteInfoHeader(&info,fp);
	
	unsigned char zero=0;
	// イメージデータ書き込み
	for( y=0 ; y < height ; y++ ){
		// データをBGRの順で書き込み
		for( x=0 ; x < width ; x++ ){
			j=fwrite((data+x*3+3*y*width+2),sizeof(GLubyte),1,fp);
			j=fwrite((data+x*3+3*y*width+1),sizeof(GLubyte),1,fp);
			j=fwrite((data+x*3+3*y*width),sizeof(GLubyte),1,fp);
		}
		// 幅のバイト数が4の倍数でないときは０で埋める
		if( width*3%4 != 0)
			for(j=0; j<4-(width*3)%4; j++)
				fwrite(&zero,sizeof(GLubyte),1,fp);
	}
	// ファイルクローズ
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

