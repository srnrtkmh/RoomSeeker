/************************************************************************************************************/
/*                                                                                                          */
/* FILE : basic_figure.h                                                                                    */
/* MEMO : Library of Drawing Basic Figure Functions (Header)                                                */
/*                                                                                                          */
/* 2010/09/15 : Start to edit this file                                                                     */
/* 2010/10/03 : Add function prototypes to draw 2D Object                                                   */
/*                                                                                                          */
/************************************************************************************************************/

#ifndef BASIC_FIGURE
#define BASIC_FIGURE

#define GLUT_BUILDING_LIB			// To avoid warning "'int glutCreateWindow_ATEXIT_HACK(const char*)' defined but not used"
#include <GL/glut.h>

/************************************************************************************************************/
/* Function Prototypes                                                                                      */
/************************************************************************************************************/
// Draw 3D Objects
void draw_cube(double a);
void draw_cuboid(double x, double y, double z, double offset_x, double offset_y, double offset_z);

void draw_cylinder_x(double radius, double h, int res);
void draw_cylinder_y(double radius, double h, int res);
void draw_cylinder_z(double radius, double h, int res);

void draw_pipe_x(double radius, double r, int res);
void draw_pipe_y(double radius, double r, int res);
void draw_pipe_z(double radius, double r, int res);

void draw_sphere(double radius);

void draw_axes(double x, double y, double z);
void floor_panel(GLfloat floor_size, GLfloat panel_size, GLfloat color1[4], GLfloat color2[4]);		// Draw floor with square panels
void floor_line(GLfloat floor_size, GLfloat square_size, GLfloat color[4]);							// Draw floor with only square outline

void draw_string_3d(double x, double y, double z, char *s);

// Used in senior opengl graphics
void Cylinder(GLdouble Radius, GLdouble Height, GLfloat color[4]);
void Pipe(GLdouble Radius, GLdouble Height, GLdouble Thickness, GLfloat color[4]);
void Square_frame(GLdouble x, GLdouble y, GLdouble Thickness, GLfloat color[4]);

// Draw 2D Objects
void draw_line(double posx1, double posy1, double posx2, double posy2);
void draw_square(double posx, double posy, double a, double b);
void draw_circle(double posx, double posy, double r, int res);
void draw_string(double x, double y, const char *s, void *font);

#endif

