#ifndef LANDER_GRAPHICS
#define LANDER_GRAPHICS
#include "lander.h"
#include "orbiter.h"
#include "math_utils.h"

#ifdef DECLARE_GLOBAL_VARIABLES
// GL windows and objects
int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
GLUquadricObj *quadObj;
GLuint terrain_texture;
short throttle_control;
// track_t track;
bool texture_available;

// Simulation parameters
bool help = false;
bool paused = false;
bool landed = false;
bool crashed = false;
int last_click_x = -1;
int last_click_y = -1;
short simulation_speed = 5;
double delta_t, simulation_time;
unsigned short scenario = 0;
std::string scenario_description[10];
closeup_coords_t closeup_coords;
float randtab[N_RAND];
bool do_texture = true;
unsigned long long time_program_started;

// Lander state - the visualization routines use velocity_from_positions, so not sensitive to
// any errors in the velocity update in numerical_dynamics
bool wind_enabled;
bool tuning_mode = true;
bool lag_enabled = false;
bool delay_enabled = false;
lander mars_lander;

// Orbital and closeup view parameters
double orbital_zoom, save_orbital_zoom, closeup_offset, closeup_xr, closeup_yr, terrain_angle;
quat_t orbital_quat;

#endif

// Function prototypes
void fghCircleTable(double **sint, double **cost, const int n);
void glutOpenHemisphere(GLdouble radius, GLint slices, GLint stacks);
void glutMottledSphere(GLdouble radius, GLint slices, GLint stacks);
void glutCone(GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void glut_print(float x, float y, std::string s);
double atmospheric_density(vector3d pos);
void draw_dial(double cx, double cy, double val, std::string title, std::string units);
void draw_control_bar(double tlx, double tly, double val, double red, double green, double blue, std::string title);
void draw_indicator_lamp(double tcx, double tcy, std::string off_text, std::string on_text, bool on);
void draw_instrument_window(void);
void display_help_arrows(void);
void display_help_prompt(void);
void display_help_text(void);
void draw_orbital_window(void);
void draw_parachute_quad(double d);
void draw_parachute(double d);
bool generate_terrain_texture(void);
void update_closeup_coords(void);
void draw_closeup_window(void);
void draw_main_window(void);
void refresh_all_subwindows(void);
void update_visualization(void);
void update_lander_state(void);
void reset_simulation(void);
void set_orbital_projection_matrix(void);
void reshape_main_window(int width, int height);
void orbital_mouse_button(int button, int state, int x, int y);
void orbital_mouse_motion(int x, int y);
void closeup_mouse_button(int button, int state, int x, int y);
void closeup_mouse_motion(int x, int y);
void glut_special(int key, int x, int y);
void glut_key(unsigned char k, int x, int y);
#endif
