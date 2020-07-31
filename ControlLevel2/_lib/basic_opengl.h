/**************************************************************************************************/
/*                                                                                                */
/* FILE : basic_opengl.h                                                                          */
/* MEMO : Header File for "basic_opengl.c"                                                        */
/*                                                                                                */
/* 2010/10/01 : Start to make this file                                                           */
/* 2010/10/03 : Remove timer_func() and idle_func() to select                                     */
/*                                                                                                */
/**************************************************************************************************/

#ifndef BASIC_OPENGL
#define BASIC_OPENGL

/**************************************************************************************************/
/* Constant Macros                                                                                */
/**************************************************************************************************/
#define WX_SIZE 800
#define WY_SIZE 600

/**************************************************************************************************/
/* Function Prototypes                                                                            */
/**************************************************************************************************/
// OpenGL initialization function
void display_init(void);
void light_init(void);
void event_init(void);
void pre_draw(void);

// OpenGL event functions
void resize_func(int w, int h);
void mouse_func(int button, int state, int x, int y);
void motion_func(int x, int y);
void keyboard_func(unsigned char key, int x, int y);
void timer_func(int value);
void idle_func(void);

#endif
