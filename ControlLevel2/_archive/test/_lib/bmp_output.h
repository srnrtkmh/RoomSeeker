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
		int	 filesize;			// 総ファイルサイズ
		short   reserve1;		// 予約領域・常に０
		short   reserve2;		// 予約領域・常に０
		int	 offset;			// ファイルの先頭からデータまでのオフセット
}BitmapHeader;

typedef struct _BitmapInfoHeader{
	int	 header;				// この構造体のサイズ
	int	 width;					// Bitmapのサイズ・横幅
	int	 height;				// Bitmapのサイズ・縦幅
	short   plane;				// plane数・常に１
	short   bits;				// Bitmapの色数(bit単位)
	int	 compression;			// 圧縮されているか？
	int	 comp_image_size;		// 画像全体のサイズ・使わない
	int	 x_resolution;			// Bitmapの解像度・使わない
	int	 y_resolution;			// 72と設定しておいてもかまわない
	int	 pallet_num;			// パレット数
								// 24bitカラー等の場合使わない
	int	 important_pallet_num;	// 重要なパレット数・使わない
								// 24bitカラー等の場合使わない
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
