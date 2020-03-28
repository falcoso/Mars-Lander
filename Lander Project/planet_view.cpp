#include "lander.h"
#include "math_utils.h"
#include "graphics_setup.h"
// #include "lander_graphics.h"
#include "planet_view.h"

BEGIN_EVENT_TABLE(PlanetCanvas, wxGLCanvas)
    EVT_PAINT    (PlanetCanvas::Paintit)
END_EVENT_TABLE()

PlanetCanvas::PlanetCanvas(wxFrame *parent, bool init=false)
		:wxGLCanvas (parent, wxID_ANY, NULL, wxDefaultPosition, wxDefaultSize, 0, "GLCanvas", wxNullPalette)
{
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };

	context = new wxGLContext(this);

    if(init)
    {
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    }
}


void PlanetCanvas::Paintit(wxPaintEvent& WXUNUSED(event))
{
    Render();
}

void PlanetCanvas::draw_orbital_window(void)
// Draws the orbital view
{
	unsigned short i, j;
	double m[16], sf;
	GLint slices, stacks;

	// glutSetWindow(orbital_window);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Viewing transformation
	quat_to_matrix(m, orbital_quat);
	glMultMatrixd(m);
	if (orbital_zoom > 2.0)
	{ // gradual pan towards the lander when zoomed in
		sf = 1.0 - exp((2.0 - orbital_zoom) / 5.0);
		glTranslated(-sf*mars_lander.get_position().x, -sf*mars_lander.get_position().y, -sf*mars_lander.get_position().z);
	}

	if (static_lighting)
	{
		// Specify light positions here, to fix them in the world coordinate system
		glLightfv(GL_LIGHT2, GL_POSITION, minus_y);
		glLightfv(GL_LIGHT3, GL_POSITION, plus_y);
	}

	// Draw planet
	glColor3f((GLfloat)0.63, (GLfloat)0.33, (GLfloat)0.22);
	glLineWidth(1.0);
	glPushMatrix();
	glRotated(360.0*simulation_time / MARS_DAY, 0.0, 0.0, 1.0); // to make the planet spin
	if (orbital_zoom > 1.0)
	{
		slices = (int)(16 * orbital_zoom); if (slices > 160) slices = 160;
		stacks = (int)(10 * orbital_zoom); if (stacks > 100) stacks = 100;
	}
	else
	{
		slices = 16; stacks = 10;
	}
	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluSphere(quadObj, (1.0 - 0.01 / orbital_zoom)*MARS_RADIUS, slices, stacks);
	glColor3f((GLfloat)0.31, (GLfloat)0.16, (GLfloat)0.11);
	gluQuadricDrawStyle(quadObj, GLU_LINE);
	gluSphere(quadObj, MARS_RADIUS, slices, stacks);
	glPopMatrix();

	// Draw previous lander positions in cyan that fades with time
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glLineWidth(1.0);

	glBegin(GL_LINE_STRIP);
		glColor3f(0.0, 1.0, 1.0);
		glVertex3d(mars_lander.get_position().x, mars_lander.get_position().y, mars_lander.get_position().z);
		j = (track.p + N_TRACK - 1) % N_TRACK;

		for (i = 0; i<track.n; i++)
		{
			glColor4f(0.0f, (GLfloat)(0.75*(N_TRACK - i) / N_TRACK),
						(GLfloat)(0.75*(N_TRACK - i) / N_TRACK),
						(GLfloat)(1.0*(N_TRACK - i) / N_TRACK));

			glVertex3d(track.pos[j].x, track.pos[j].y, track.pos[j].z);
			j = (j + N_TRACK - 1) % N_TRACK;
		}
	glEnd();
	glDisable(GL_BLEND);

	// Draw lander as a cyan dot
	glColor3f(0.0, 1.0, 1.0);
	glPointSize(3.0);
	glBegin(GL_POINTS);
		glVertex3d(mars_lander.get_position().x, mars_lander.get_position().y, mars_lander.get_position().z);
	glEnd();
	glEnable(GL_LIGHTING);

	// Help information
	// if (help) display_help_text();

	glutSwapBuffers();
}

void PlanetCanvas::Render()
{
    SetCurrent(*context);
    wxPaintDC(this);

	glDrawBuffer(GL_BACK);
	setup_lights();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE); // since the only polygons in this view define a solid sphere
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_NORMALIZE);
	glDepthFunc(GL_LEQUAL);
	glShadeModel(GL_SMOOTH);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glutDisplayFunc(draw_orbital_window);
	// glutMouseFunc(orbital_mouse_button);
	// glutMotionFunc(orbital_mouse_motion);
	// glutKeyboardFunc(glut_key);
	// glutSpecialFunc(glut_special);
	quadObj = gluNewQuadric();
	orbital_quat.v.x = 0.53; orbital_quat.v.y = -0.21;
	orbital_quat.v.z = 0.047; orbital_quat.s = 0.82;
	normalize_quat(orbital_quat);
	save_orbital_zoom = 1.0;
	orbital_zoom = 1.0;
    glFlush();
    SwapBuffers();
}

