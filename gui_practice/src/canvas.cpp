#include <wx/wx.h>
#include <wx/glcanvas.h>

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "canvas.h"

BEGIN_EVENT_TABLE(MyCanvas, wxGLCanvas)
    EVT_PAINT    (MyCanvas::Paintit)
END_EVENT_TABLE()

MyCanvas::MyCanvas(wxFrame *parent, bool init=false)
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


void MyCanvas::Paintit(wxPaintEvent& WXUNUSED(event))
{
    Render();
}

void MyCanvas::Render()
{
    SetCurrent(*context);
    wxPaintDC(this);

    glDrawBuffer(GL_BACK);
    glLineWidth(2.0);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);

    glBegin(GL_POLYGON);
        glColor3f(1.0, 1.0, 1.0);
        glVertex2f(-0.5, -0.5);
        glVertex2f(-0.5, 0.5);
        glVertex2f(0.5, 0.5);
        glVertex2f(0.5, -0.5);
        glColor3f(0.4, 0.5, 0.4);
        glVertex2f(0.0, -0.8);
    glEnd();

    glBegin(GL_POLYGON);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2f(0.1, 0.1);
        glVertex2f(-0.1, 0.1);
        glVertex2f(-0.1, -0.1);
        glVertex2f(0.1, -0.1);
    glEnd();

	// using a little of glut
    glColor4f(0,0,1,1);
    glutWireTeapot(0.4);

    glLoadIdentity();
    glColor4f(2,0,1,1);
    glutWireTeapot(0.6);
	// done using glut
    glFlush();
    SwapBuffers();
}