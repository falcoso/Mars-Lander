#ifndef GRAPHICS_SETUP
#define GRAPHICS_SETUP

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "math_utils.h"

#ifdef DECLARE_GLOBAL_VARIABLES
bool static_lighting = false;

GLfloat plus_y[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat minus_y[] = { 0.0, -1.0, 0.0, 0.0 };
GLfloat plus_z[] = { 0.0, 0.0, 1.0, 0.0 };
GLfloat top_right[] = { 1.0, 1.0, 1.0, 0.0 };
GLfloat straight_on[] = { 0.0, 0.0, 1.0, 0.0 };

track_t track;
#else
extern bool static_lighting;

extern GLfloat plus_y[];
extern GLfloat minus_y[];
extern GLfloat plus_z[];
extern GLfloat top_right[];
extern GLfloat straight_on[];

extern track_t track;
#endif


void enable_lights(void);
void setup_lights(void);

#endif