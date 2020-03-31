#ifndef PLANET_VIEW
#define PLANET_VIEW

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include "orbiter.h"

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

class PlanetCanvas: public wxGLCanvas
{
    void Render();
public:
	wxGLContext *context;

    PlanetCanvas(wxFrame* parent, lander *mars_lander_ptr, bool init=false);
    void Paintit(wxPaintEvent& event);
	void draw_orbital_window();
protected:
    DECLARE_EVENT_TABLE()
	double orbital_zoom, save_orbital_zoom;
	quat_t orbital_quat;
	GLUquadricObj *quadObj; // may need to be accessed elsewhere??
    lander *mars_lander;
};

#endif