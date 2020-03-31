#ifndef INSTRUMENT_WINDOW
#define INSTRUMENT_WINDOW

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include "orbiter.h"

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

class InstrumentCanvas: public wxGLCanvas
{
    void Render();
public:
	wxGLContext *context;

    InstrumentCanvas(wxFrame* parent, lander *mars_lander_ptr, bool init=false);
    void Paintit(wxPaintEvent& event);
	void draw_dial(double cx, double cy, double val, std::string title, std::string units);
	void draw_control_bar(double tlx, double tly, double val, double red, double green, double blue, std::string title);
	void draw_indicator_lamp(double tcx, double tcy, std::string off_text, std::string on_text, bool on);

protected:
    DECLARE_EVENT_TABLE()
    lander *mars_lander;
};

#endif