#ifndef CANVAS
#define CANVAS

#include <wx/wx.h>
#include <wx/glcanvas.h>

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

class MyCanvas: public wxGLCanvas
{
        void Render();
public:
	wxGLContext *context;

    MyCanvas(wxFrame* parent, bool init=false);
    void Paintit(wxPaintEvent& event);
protected:
    DECLARE_EVENT_TABLE()
};

#endif