/**************************************************************************************************/
/*                                                                                                */
/* FILE : basic_figure.c                                                                          */
/* MEMO : Library of Drawing Basic Figure Functions                                               */
/*                                                                                                */
/* 2010/09/23 : Start to edit this file                                                           */
/* 2010/10/03 : Add functions to draw 2D Object                                                   */
/*              Integrate "basic_figure.c" and "floor.c" to reduce include files                  */
/* 2010/10/31 : Modify "floor_line" function                                                      */
/*                                                                                                */
/**************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define GLUT_BUILDING_LIB			// To avoid warning "'int glutCreateWindow_ATEXIT_HACK(const char*)' defined but not used"
#include <GL/glut.h>
#include "basic_figure.h"

/**************************************************************************************************/
/* Functions to Draw 3D Object                                                                    */
/**************************************************************************************************/
// Draw cube 
void draw_cube(double a)
{
	// 立方体の頂点
	GLdouble vertex[][3]={
		{ 0.0, 0.0, 0.0 },
		{   a, 0.0, 0.0 },
		{   a,   a, 0.0 },
		{ 0.0,   a, 0.0 },
		{ 0.0, 0.0,   a },
		{   a, 0.0,   a },
		{   a,   a,   a },
		{ 0.0,   a,   a }
	};
	
	// 立方体の面を構成する頂点番号
	int face[][4]={
		{ 0, 3, 2, 1 }, /* A-B-C-D を結ぶ面 */
		{ 1, 2, 6, 5 }, /* B-F-G-C を結ぶ面 */
		{ 4, 5, 6, 7 }, /* F-E-H-G を結ぶ面 */
		{ 0, 4, 7, 3 }, /* E-A-D-H を結ぶ面 */
		{ 0, 1, 5, 4 }, /* E-F-B-A を結ぶ面 */
		{ 2, 3, 7, 6 }  /* D-C-G-H を結ぶ面 */
	};
	
	// 立方体各面の法線ベクトル
	GLdouble normal[][3] = {
		{ 0.0, 0.0,-1.0 },
		{ 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0 },
		{-1.0, 0.0, 0.0 },
		{ 0.0,-1.0, 0.0 },
		{ 0.0, 1.0, 0.0 }
	};
	
	int i, j;
	
	glBegin(GL_QUADS);
		for(i=0; i<6; i++){
			glNormal3dv(normal[i]);
			for(j=0; j<4; j++){
				glVertex3dv(vertex[face[i][j]]);
			}
		}
	glEnd();
}

// Draw cuboid
void draw_cuboid(double x, double y, double z, double offset_x, double offset_y, double offset_z)
{
	// 立方体の頂点
	GLdouble vertex[][3]={
		{  offset_x,   offset_y,   offset_z},
		{x+offset_x,   offset_y,   offset_z},
		{x+offset_x, y+offset_y,   offset_z},
		{  offset_x, y+offset_y,   offset_z},
		{offset_x,     offset_y, z+offset_z},
		{x+offset_x,   offset_y, z+offset_z},
		{x+offset_x, y+offset_y, z+offset_z},
		{offset_x,   y+offset_y, z+offset_z}
	};
	
	// 立方体の面を構成する頂点番号
	int face[][4]={
		{ 0, 3, 2, 1 }, /* A-B-C-D を結ぶ面 */
		{ 1, 2, 6, 5 }, /* B-F-G-C を結ぶ面 */
		{ 4, 5, 6, 7 }, /* F-E-H-G を結ぶ面 */
		{ 0, 4, 7, 3 }, /* E-A-D-H を結ぶ面 */
		{ 0, 1, 5, 4 }, /* E-F-B-A を結ぶ面 */
		{ 2, 3, 7, 6 }  /* D-C-G-H を結ぶ面 */
	};
	
	// 立方体各面の法線ベクトル
	GLdouble normal[][3] = {
		{ 0.0, 0.0,-1.0 },
		{ 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0 },
		{-1.0, 0.0, 0.0 },
		{ 0.0,-1.0, 0.0 },
		{ 0.0, 1.0, 0.0 }
	};
	
	int i, j;
	
	glBegin(GL_QUADS);
		for(i=0; i<6; i++){
			glNormal3dv(normal[i]);
			for(j=0; j<4; j++){
				glVertex3dv(vertex[face[i][j]]);
			}
		}
	glEnd();
}

// Draw cylinder
void draw_cylinder_z(double radius, double h, int res)
{
	int i;
	double tmp_x, tmp_y;
	
	// Draw lower circle
	glNormal3f(0.0, 0.0, -1.0);
	glBegin(GL_TRIANGLE_FAN);
		for(i=res; i>=0; i--){
				glVertex3f(radius*cos(2*M_PI/res*i), radius*sin(2*M_PI/res*i), 0.0);
		}
    glEnd();
	
	// Draw side surface
	glBegin(GL_QUAD_STRIP);
		for(i=0; i<=res; i++){
			tmp_x=radius*cos(2*M_PI/res*i);
			tmp_y=radius*sin(2*M_PI/res*i);
			glNormal3f(tmp_x, tmp_y, 0.0);
			glVertex3f(tmp_x, tmp_y, h);
			glVertex3f(tmp_x, tmp_y, 0.0);
 	   }
    glEnd();
	
	// Draw upper circle
	glNormal3f(0.0, 0.0, 1.0);
	glBegin(GL_TRIANGLE_FAN);
		for(i=0; i<=res; i++){
			glVertex3f(radius*cos(2*M_PI/res*i), radius*sin(2*M_PI/res*i), h);
		}
	glEnd();
}

// Draw sphere
void draw_sphere(double radius)
{
	
}

// Draw coordination axes
void draw_axes(double x, double y, double z)
{
	GLdouble tmp;
	GLfloat red[4] = { 1.000000, 0.000000, 0.000000, 1.0};
	GLfloat blue[4] = { 0.000000, 0.000000, 1.000000, 1.0};
	GLfloat green[4] = { 0.000000, 1.000000, 0.000000, 1.0};
	
	glGetDoublev(GL_LINE_WIDTH, &tmp);
	glLineWidth(3.0);
	glBegin(GL_LINES);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, red);
		glVertex3d(0.0, 0.0, 0.0);	glVertex3d(x, 0.0, 0.0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, green);
		glVertex3d(0.0, 0.0, 0.0);	glVertex3d(0.0, y, 0.0);
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, blue);
		glVertex3d(0.0, 0.0, 0.0);	glVertex3d(0.0, 0.0, z);
	glEnd();
	
	glLineWidth(tmp);
}

// Draw floor with square panels
void floor_panel(GLfloat floor_size, GLfloat panel_size, GLfloat color1[4], GLfloat color2[4])
{
	int i, j, point;					// i, jは何番目の格子かを表す．pointは(i, j)が共に偶数か，異なるかを表す．
	double x, y, tmp[1];				// (x, y)は格子の正方形の左下の座標
	
	glGetDoublev(GL_LINE_WIDTH, tmp);
	glLineWidth(3.0);
	
	for(i = -(int)(floor_size/(panel_size*2.0)); i <= (int)(floor_size/(panel_size*2.0)); i++){
		x = i * panel_size;
		for(j = -(int)(floor_size/(panel_size*2.0)); j <= (int)(floor_size/(panel_size*2.0)); j++){
			y = j * panel_size;
			if((i&1)==(j&1))	point=1;
			else				point=0;
			
			glBegin(GL_QUADS);
				glNormal3d(0.0, 0.0, 1.0);						// 陰影付けのためには面ごとに法線ベクトルが必要！！
				if(point)	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color1);
				else		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color2);
				glNormal3d(0.0, 0.0, 1.0);
				glVertex3d(           x,            y, 0.0);
				glVertex3d(x+panel_size,            y, 0.0);
				glVertex3d(x+panel_size, y+panel_size, 0.0);
				glVertex3d(           x, y+panel_size, 0.0);
			glEnd();
		}
	}
	
	glLineWidth(tmp[0]);
}

// Draw floor with only square outline
void floor_line(GLfloat floor_size, GLfloat square_size, GLfloat color[4])
{
	double x_min=-floor_size/2.0, x_max=floor_size/2.0;
	double y_min=-floor_size/2.0, y_max=floor_size/2.0;
	double tmp[1];
	double cnt;
	
	glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
	glGetDoublev(GL_LINE_WIDTH, tmp);
	glLineWidth(3.0);
	
	for(cnt=y_min; cnt<=y_max; cnt+=square_size)
	{
		glNormal3d(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
			glVertex3d(x_min, cnt, 0.0);
			glVertex3d(x_max, cnt, 0.0);
		glEnd();
	}
	
	for(cnt=x_min; cnt<=x_max; cnt+=square_size)
	{
		glNormal3d(0.0, 0.0, 1.0);
		glBegin(GL_LINES);
			glVertex3d(cnt, y_min, 0.0);
			glVertex3d(cnt, y_max, 0.0);
		glEnd();
	}
	glLineWidth(tmp[0]);
}

// Draw 2D Character String in 3D Space
void draw_string_3d(double x, double y, double z, char *s)
{
	glRasterPos3f(x, y, z);
	for(; *s != '\0'; s++)	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, *s);
}



/**************************************************************************************************/
/* Functions used in senior's OpenGL graphics                                                     */
/**************************************************************************************************/
/**************************************** 円柱の描画関数 ****************************************/
void Cylinder(GLdouble Radius, GLdouble Height, GLfloat color[4])
{
	int i;
	
	glMaterialfv(GL_FRONT, GL_DIFFUSE, color);					// 表面属性(光の反射特性≒色)の設定(拡散光)
	
	/**************************************** 上面の描画 ****************************************/
	glNormal3d(0.0, 1.0, 0.0);									// 陰影付けのためには面ごとに法線ベクトルが必要！！
	glBegin(GL_TRIANGLE_FAN);									// 三角形(扇形)の連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glVertex3d(Radius*sin(i*(M_PI/180.0)), (Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();

	/**************************************** 下面の描画 ****************************************/
	glNormal3d(0.0, -1.0, 0.0);									// 陰影付けのためには面ごとに法線ベクトルが必要！！
	glBegin(GL_TRIANGLE_FAN);									// 三角形(扇形)を連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glVertex3d(Radius*sin(i*(M_PI/180.0)), -(Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();

	/**************************************** 側面の描画 ****************************************/
	glBegin(GL_QUAD_STRIP);										// 四角形を連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glNormal3d(sin(i*(M_PI/180.0)), 0.0, cos(i*(M_PI/180.0)));	// 陰影付けのためには面ごとに法線ベクトルが必要！！
		glVertex3d(Radius*sin(i*(M_PI/180.0)), (Height/2.0), Radius*cos(i*(M_PI/180.0)));
		glVertex3d(Radius*sin(i*(M_PI/180.0)), -(Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();
	
}


/******************************* パイプ(中が空洞の円柱)の描画関数 *******************************/
void Pipe(GLdouble Radius, GLdouble Height, GLdouble Thickness, GLfloat color[4])
{
	int i;
	
	glMaterialfv(GL_FRONT, GL_DIFFUSE, color);					// 表面属性(光の反射特性≒色)の設定(拡散光)
	
	/**************************************** 上面の描画 ****************************************/
	glNormal3d(0.0, 1.0, 0.0);									// 陰影付けのためには面ごとに法線ベクトルが必要！！
	glBegin(GL_TRIANGLE_STRIP);									// 三角形の連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glVertex3d((Radius-Thickness)*sin(i*(M_PI/180.0)), (Height/2.0), (Radius-Thickness)*cos(i*(M_PI/180.0)));
		glVertex3d(Radius*sin(i*(M_PI/180.0)), (Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();
	
	/**************************************** 下面の描画 ****************************************/
	glNormal3d(0.0, 1.0, 0.0);									// 陰影付けのためには面ごとに法線ベクトルが必要！！
	glBegin(GL_TRIANGLE_STRIP);									// 三角形の連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glVertex3d((Radius-Thickness)*sin(i*(M_PI/180.0)), -(Height/2.0), (Radius-Thickness)*cos(i*(M_PI/180.0)));
		glVertex3d(Radius*sin(i*(M_PI/180.0)), -(Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();

	/************************************* 側面(内側)の描画 *************************************/
	glBegin(GL_QUAD_STRIP);											// 四角形を連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glNormal3d(-sin(i*(M_PI/180.0)), 0.0, cos(-i*(M_PI/180.0)));	// 陰影付けのためには面ごとに法線ベクトルが必要！！
		glVertex3d((Radius-Thickness)*sin(i*(M_PI/180.0)), (Height/2.0), (Radius-Thickness)*cos(i*(M_PI/180.0)));
		glVertex3d((Radius-Thickness)*sin(i*(M_PI/180.0)), -(Height/2.0), (Radius-Thickness)*cos(i*(M_PI/180.0)));
	}
	glEnd();
	
	/************************************* 側面(外側)の描画 *************************************/
	glBegin(GL_QUAD_STRIP);											// 四角形を連結した多角形の描画
	for (i = 0; i <= 360; i += 1)
	{
		glNormal3d(sin(i*(M_PI/180.0)), 0.0, cos(i*(M_PI/180.0)));		// 陰影付けのためには面ごとに法線ベクトルが必要！！
		glVertex3d(Radius*sin(i*(M_PI/180.0)), (Height/2.0), Radius*cos(i*(M_PI/180.0)));
		glVertex3d(Radius*sin(i*(M_PI/180.0)), -(Height/2.0), Radius*cos(i*(M_PI/180.0)));
	}
	glEnd();

}

/************************************* Square frame ************************************/
void Square_frame(GLdouble x, GLdouble y, GLdouble Thickness, GLfloat color[4])
{
	glPushMatrix();														// store of the initial coordinates 1
		glTranslated(x/2.0-Thickness/2.0, 0.0, 0.0);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);						// setting of the attribute of the surface
		glScaled(Thickness, y, Thickness);								// scale of the body (x, y, z)
		glutSolidCube(1.0);												// painting of the cube 1.0 on a side
	glPopMatrix();														// call of the initial coordinates 1
	glPushMatrix();														// store of the initial coordinates 2
		glTranslated(0.0, y/2.0-Thickness/2.0, 0.0);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);						// setting of the attribute of the surface
		glScaled(x, Thickness, Thickness);								// scale of the body (x, y, z)
		glutSolidCube(1.0);												// painting of the cube 1.0 on a side
	glPopMatrix();														// call of the initial coordinates 2
	glPushMatrix();														// store of the initial coordinates 3
		glTranslated(-x/2.0+Thickness/2.0, 0.0, 0.0);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);						// setting of the attribute of the surface
		glScaled(Thickness, y, Thickness);								// scale of the body (x, y, z)
		glutSolidCube(1.0);												// painting of the cube 1.0 on a side
	glPopMatrix();														// call of the initial coordinates 3
	glPushMatrix();														// store of the initial coordinates 2
		glTranslated(0.0, -y/2.0+Thickness/2.0, 0.0);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, color);						// setting of the attribute of the surface
		glScaled(x, Thickness, Thickness);								// scale of the body (x, y, z)
		glutSolidCube(1.0);												// painting of the cube 1.0 on a side
	glPopMatrix();														// call of the initial coordinates 2
}

/**************************************************************************************************/
/* Functions to Draw 2D Object                                                                    */
/**************************************************************************************************/
// Draw Line
void draw_line(double posx1, double posy1, double posx2, double posy2)
{
	glBegin(GL_LINES);
		glVertex2d(posx1, posy1);
		glVertex2d(posx2, posy2);
	glEnd();
}

// Draw Square
void draw_square(double posx, double posy, double a, double b)
{
	glBegin(GL_QUADS);
		glVertex2d(posx-a/2.0, posy-b/2.0);
		glVertex2d(posx+a/2.0, posy-b/2.0);
		glVertex2d(posx+a/2.0, posy+b/2.0);
		glVertex2d(posx-a/2.0, posy+b/2.0);
	glEnd();
}

// Draw Circle
void draw_circle(double posx, double posy, double r, int res)
{
	int i;
	
	glBegin(GL_TRIANGLE_FAN);
		glVertex2d(posx, posy);
		for(i=0; i<=res; i++){
			glVertex2d(posx+r*cos(2*M_PI/res*(double)i), posy+r*sin(2*M_PI/res*(double)i));
		}
	glEnd();
}

// Draw String
void draw_string(double x, double y, const char *s, void *font)
{
	glRasterPos2f(x, y);
	for(; *s != '\0'; s++)	glutBitmapCharacter(font, *s);
}

