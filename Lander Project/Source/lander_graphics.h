#ifndef LANDER_GRAPHICS
#define LANDER_GRAPHICS
#include "lander.h"
#include "Orbiter class.h"

#ifdef DECLARE_GLOBAL_VARIABLES
// GL windows and objects
int main_window, closeup_window, orbital_window, instrument_window, view_width, view_height, win_width, win_height;
GLUquadricObj *quadObj;
GLuint terrain_texture;
short throttle_control;
track_t track;
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
bool static_lighting = false;
closeup_coords_t closeup_coords;
float randtab[N_RAND];
bool do_texture = true;
unsigned long throttle_buffer_length, throttle_buffer_pointer;
double *throttle_buffer = NULL;
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

// For GL lights
GLfloat plus_y[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat minus_y[] = { 0.0, -1.0, 0.0, 0.0 };
GLfloat plus_z[] = { 0.0, 0.0, 1.0, 0.0 };
GLfloat top_right[] = { 1.0, 1.0, 1.0, 0.0 };
GLfloat straight_on[] = { 0.0, 0.0, 1.0, 0.0 };
#endif

// Function prototypes
void invert(double m[], double mout[]);
void xyz_euler_to_matrix(vector3d ang, double m[]);
vector3d matrix_to_xyz_euler(double m[]);
void normalize_quat(quat_t &q);
quat_t axis_to_quat(vector3d a, const double phi);
double project_to_sphere(const double r, const double x, const double y);
quat_t add_quats(quat_t q1, quat_t q2);
void quat_to_matrix(double m[], const quat_t q);
quat_t track_quats(const double p1x, const double p1y, const double p2x, const double p2y);
void microsecond_time(unsigned long long &t);
void fghCircleTable(double **sint, double **cost, const int n);
void glutOpenHemisphere(GLdouble radius, GLint slices, GLint stacks);
void glutMottledSphere(GLdouble radius, GLint slices, GLint stacks);
void glutCone(GLdouble base, GLdouble height, GLint slices, GLint stacks, bool closed);
void enable_lights(void);
void setup_lights(void);
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
