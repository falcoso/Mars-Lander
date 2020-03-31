#include "lander.h"
#include "instrument_window.h"
// #include "math_utils.h"
#include "graphics_setup.h"
// #include "lander_graphics.h"
// #include "planet_view.h"

BEGIN_EVENT_TABLE(InstrumentCanvas, wxGLCanvas)
    EVT_PAINT    (InstrumentCanvas::Paintit)
END_EVENT_TABLE()

InstrumentCanvas::InstrumentCanvas(wxFrame *parent, lander* mars_lander_ptr, bool init)
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
    mars_lander = mars_lander_ptr;
    mars_lander->set_position (vector3d(1.2*MARS_RADIUS, 0.0, 0.0));
    mars_lander->set_velocity (vector3d(0.0, -std::sqrt(GRAVITY*MARS_MASS / (1.2*MARS_RADIUS)), 0.0));
    mars_lander->set_orientation(vector3d(0.0, 90.0, 0.0));
    mars_lander->stabilized_attitude = false;
    mars_lander->autopilot_status = ORBIT_RE_ENTRY;
}


void InstrumentCanvas::Paintit(wxPaintEvent& WXUNUSED(event))
{
    Render();
}

void InstrumentCanvas::draw_dial(double cx, double cy, double val, std::string title, std::string units)
// Draws a single instrument dial, position (cx, cy), value val, title
{
	int i, e;
	double a;
	std::ostringstream s;

	// Work out value mantissa and exponent
	if (val <= 0.0)
	{
		e = 0;
		a = 0.0;
	}
	else
	{
		e = 0;
		a = val;
		while (a >= 10.0)
		{
			e++;
			a /= 10.0;
		}
	}

	// Draw dial ticks
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
	for (i = 30; i <= 330; i += 30)
	{
		glVertex2d(cx - OUTER_DIAL_RADIUS * sin(i*M_PI / 180.0), cy - OUTER_DIAL_RADIUS * cos(i*M_PI / 180.0));
		glVertex2d(cx - INNER_DIAL_RADIUS * sin(i*M_PI / 180.0), cy - INNER_DIAL_RADIUS * cos(i*M_PI / 180.0));
	}
	glEnd();

	// Draw dial needle
	glColor3f(0.0, 1.0, 1.0);
	glBegin(GL_LINES);
	glVertex2d(cx, cy);
	glVertex2d(cx - INNER_DIAL_RADIUS * sin((a * 30 + 30)*M_PI / 180.0), cy - INNER_DIAL_RADIUS * cos((a * 30 + 30)*M_PI / 180.0));
	glEnd();

	// Draw exponent indicator, value and title
	s.precision(1);
	glColor3f(1.0, 1.0, 1.0);
	s.str(""); s << "x 10 ^ " << e << " " << units;
	glut_print((GLfloat)(cx + 10 - 3.2*s.str().length()), (GLfloat)(cy + 10), s.str());
	glut_print((GLfloat)(cx + 10 - 3.2f*title.length()), (GLfloat)(cy - OUTER_DIAL_RADIUS - 15), title);
	s.str(""); s << std::fixed << val << " " << units;
	glut_print((GLfloat)(cx + 10 - 3.2f*s.str().length()), (GLfloat)(cy - OUTER_DIAL_RADIUS - 30), s.str());

	// Draw tick labels
	for (i = 0; i <= 10; i++)
	{
		switch (i)
		{
		case 0:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 8),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 9), "0");
			break;
		case 1:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 7),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 6), "1");
			break;
		case 2:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 9),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 4), "2");
			break;
		case 3:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 9),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 1), "3");
			break;
		case 4:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 8),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) + 3), "4");
			break;
		case 5:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) - 3),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) + 4), "5");
			break;
		case 6:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) + 2),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) + 3), "6");
			break;
		case 7:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) + 4),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180)), "7");
			break;
		case 8:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) + 3),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 4), "8");
			break;
		case 9:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) + 3),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180.0) - 6), "9");
			break;
		case 10:
			glut_print((GLfloat)(cx - OUTER_DIAL_RADIUS * sin((i * 30 + 30)*M_PI / 180) + 3),
						(GLfloat)(cy - OUTER_DIAL_RADIUS * cos((i * 30 + 30)*M_PI / 180) - 8), "10");
			break;
		}
	}
}

void InstrumentCanvas::draw_control_bar(double tlx, double tly, double val, double red, double green, double blue, std::string title)
// Draws control bar, top left (tlx, tly), val (fraction, range 0-1), colour (red, green, blue), title
{
	glColor3f((GLfloat)(red), (GLfloat)(green), (GLfloat)(blue));
	glBegin(GL_QUADS);
	glVertex2d(tlx + 0.5, tly - 19.5);
	glVertex2d(tlx + 0.5 + 239.0*val, tly - 19.5);
	glVertex2d(tlx + 0.5 + 239.0*val, tly - 0.5);
	glVertex2d(tlx + 0.5, tly - 0.5);
	glEnd();
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(tlx, tly - 20.0);
	glVertex2d(tlx + 240.0, tly - 20.0);
	glVertex2d(tlx + 240.0, tly);
	glVertex2d(tlx, tly);
	glEnd();
	glut_print((GLfloat)(tlx), (GLfloat)(tly - 40), title);
}

void InstrumentCanvas::draw_indicator_lamp(double tcx, double tcy, std::string off_text, std::string on_text, bool on)
// Draws indicator lamp, top centre (tcx, tcy), appropriate text and background colour depending on on/off
{
	if (on) glColor3f(0.5, 0.0, 0.0);
	else glColor3f(0.0, 0.5, 0.0);
	glBegin(GL_QUADS);
	glVertex2d(tcx - 74.5, tcy - 19.5);
	glVertex2d(tcx + 74.5, tcy - 19.5);
	glVertex2d(tcx + 74.5, tcy - 0.5);
	glVertex2d(tcx - 74.5, tcy - 0.5);
	glEnd();
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(tcx - 75.0, tcy - 20.0);
	glVertex2d(tcx + 75.0, tcy - 20.0);
	glVertex2d(tcx + 75.0, tcy);
	glVertex2d(tcx - 75.0, tcy);
	glEnd();
	if (on) glut_print((GLfloat)(tcx - 70.0), (GLfloat)(tcy - 14.0), on_text);
	else glut_print((GLfloat)(tcx - 70.0), (GLfloat)(tcy - 14.0), off_text);
}

void InstrumentCanvas::Render()
{
    SetCurrent(*context);
    wxPaintDC(this);

	std::ostringstream s;

	s.precision(1);

	glClear(GL_COLOR_BUFFER_BIT);
    // glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
    glEnable(GL_TEXTURE_2D);   // textures
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glViewport(0, 0, GetSize().x, GetSize().y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluOrtho2D(0, GetSize().x, GetSize().y, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	// Draw altimeter
	draw_dial(GetSize().x + GAP - 400, INSTRUMENT_HEIGHT / 2,
				mars_lander->get_altitude(), "Altitude", "m");

	// Draw auto-pilot lamp
	draw_indicator_lamp(GetSize().x + GAP - 400, INSTRUMENT_HEIGHT - 18,
						"Auto-pilot off", "Auto-pilot on",
						mars_lander->autopilot_enabled);

	// Draw climb rate meter
	if (mars_lander->get_climb_speed() >= 0.0)
	{
		draw_dial(GetSize().x + GAP - 150,
				INSTRUMENT_HEIGHT / 2,
				mars_lander->landed ? 0.0 : mars_lander->get_climb_speed(),
				"Climb rate", "m/s");
	}
	else
	{
		draw_dial(GetSize().x + GAP - 150,
					INSTRUMENT_HEIGHT / 2,
					mars_lander->landed ? 0.0 : -mars_lander->get_climb_speed(),
					"Descent rate", "m/s");
	}

	// Draw attitude stabilizer lamp
	draw_indicator_lamp(GetSize().x + GAP - 150,
						INSTRUMENT_HEIGHT - 18,
						"Attitude stabilizer off", "Attitude stabilizer on",
						mars_lander->stabilized_attitude);

	// Draw ground speed meter
	draw_dial(GetSize().x + GAP + 100, INSTRUMENT_HEIGHT / 2,
			  mars_lander->landed ? 0.0 : mars_lander->get_ground_speed(),
			  "Ground speed", "m/s");

	// Draw parachute lamp
	switch (mars_lander->parachute_status)
	{
	case NOT_DEPLOYED:
		draw_indicator_lamp(GetSize().x + GAP + 100, INSTRUMENT_HEIGHT - 18,
							"Parachute not deployed", "Do not deploy parachute",
							!mars_lander->safe_to_deploy_parachute);
		break;
	case DEPLOYED:
		draw_indicator_lamp(GetSize().x + GAP + 100, INSTRUMENT_HEIGHT - 18,
							"Parachute deployed", "", false);
		break;
	case LOST:
		draw_indicator_lamp(GetSize().x + GAP + 100, INSTRUMENT_HEIGHT - 18, "",
							"Parachute lost", true);
		break;
	}

	// Draw speed bar
	draw_control_bar(GetSize().x + GAP + 240, INSTRUMENT_HEIGHT - 18,
					simulation_speed / 10.0, 0.0, 0.0, 1.0, "Simulation speed");

	// Draw digital clock
	glColor3f(1.0, 1.0, 1.0);
	s.str(""); s << "Time " << std::fixed << simulation_time << " s";
	glut_print((GLfloat)(GetSize().x + GAP + 400), (GLfloat)(INSTRUMENT_HEIGHT - 58), s.str());
	if (paused)
	{
		glColor3f(1.0, 0.0, 0.0);
		glut_print((GLfloat)(GetSize().x + GAP + 338), (GLfloat)(INSTRUMENT_HEIGHT - 32), "PAUSED");
	}

	// Display coordinates
	glColor3f(1.0, 1.0, 1.0);
	s.str(""); s << "x position " << std::fixed << mars_lander->get_position().x << " m";
	glut_print((GLfloat)(GetSize().x + GAP + 240), INSTRUMENT_HEIGHT - 97, s.str());
	s.str(""); s << "velocity " << std::fixed << mars_lander->get_velocity().x << " m/s";
	glut_print((GLfloat)(GetSize().x + GAP + 380), INSTRUMENT_HEIGHT - 97, s.str());
	s.str(""); s << "y position " << std::fixed << mars_lander->get_position().y << " m";
	glut_print((GLfloat)(GetSize().x + GAP + 240), INSTRUMENT_HEIGHT - 117, s.str());
	s.str(""); s << "velocity " << std::fixed << mars_lander->get_velocity().y << " m/s";
	glut_print((GLfloat)(GetSize().x + GAP + 380), INSTRUMENT_HEIGHT - 117, s.str());
	s.str(""); s << "z position " << std::fixed << mars_lander->get_position().z << " m";
	glut_print((GLfloat)(GetSize().x + GAP + 240), INSTRUMENT_HEIGHT - 137, s.str());
	s.str(""); s << "velocity " << std::fixed << mars_lander->get_velocity().z << " m/s";
	glut_print((GLfloat)(GetSize().x + GAP + 380), INSTRUMENT_HEIGHT - 137, s.str());

	// Draw thrust bar
	s.str(""); s << "Thrust " << std::fixed << mars_lander->thrust_wrt_world().abs() << " N";
	draw_control_bar(GetSize().x + GAP + 240, INSTRUMENT_HEIGHT - 170, mars_lander->throttle, 1.0, 0.0, 0.0, s.str());

	// Draw fuel bar
	s.str(""); s << "Fuel " << std::fixed << mars_lander->fuel*FUEL_CAPACITY << " litres";
	if (mars_lander->fuel > 0.5)
	{
		draw_control_bar(GetSize().x + GAP + 240, INSTRUMENT_HEIGHT - 242,
						mars_lander->fuel, 0.0, 1.0, 0.0, s.str());
	}
	else if (mars_lander->fuel > 0.2)
	{
		draw_control_bar(GetSize().x + GAP + 240, INSTRUMENT_HEIGHT - 242,
						mars_lander->fuel, 1.0, 0.5, 0.0, s.str());
	}
	else
	{
		draw_control_bar(GetSize().x + GAP + 240, INSTRUMENT_HEIGHT - 242,
						mars_lander->fuel, 1.0, 0.0, 0.0, s.str());
	}

	// Display simulation status
	if (mars_lander->landed) glColor3f(1.0, 1.0, 0.0);
	else glColor3f(1.0, 1.0, 1.0);
	s.str(""); s << "Scenario " << scenario;
	if (!mars_lander->landed) s << ": " << scenario_description[scenario];
	glut_print((GLfloat)(GetSize().x + GAP - 488), (GLfloat)17, s.str());
	if (mars_lander->landed)
	{
		if (mars_lander->get_altitude() < LANDER_SIZE / 2.0)
		{
			glut_print(80, 17, "Lander is below the surface!");
		}
		else
		{
			s.str(""); s << "Fuel consumed " << std::fixed << FUEL_CAPACITY*(1.0 - mars_lander->fuel) << " litres";
			glut_print((GLfloat)(GetSize().x + GAP - 427), (GLfloat)17, s.str());
			s.str(""); s << "Descent rate at touchdown " << std::fixed << -mars_lander->get_climb_speed() << " m/s";
			glut_print((GLfloat)(GetSize().x + GAP - 232), (GLfloat)17, s.str());
			s.str(""); s << "Ground speed at touchdown " << std::fixed << mars_lander->get_ground_speed() << " m/s";
			glut_print((GLfloat)(GetSize().x + GAP + 16), (GLfloat)17, s.str());
		}
	}

    glFlush();
	SwapBuffers();

}