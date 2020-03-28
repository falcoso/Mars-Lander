// wxWidgets "Hello world" Program
// For compilers that support precompilation, includes "wx/wx.h".
#include "gui.h"

bool MyApp::OnInit()
{
	MyFrame *frame = new MyFrame( "Hello World", wxPoint(50, 50), wxSize(900, 700) );
	frame->Show( true );
	return true;
}

MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
		: wxFrame(NULL, wxID_ANY, title, pos, size)
{
	wxMenu *menuFile = new wxMenu;
	menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
					 "Help string shown in status bar for this menu item");
	menuFile->AppendSeparator();
	menuFile->Append(wxID_EXIT);
	wxMenu *menuHelp = new wxMenu;
	menuHelp->Append(wxID_ABOUT);

	menuBar = new wxMenuBar;
	menuBar->Append( menuFile, "&File" );
	menuBar->Append( menuHelp, "&Help" );
	SetMenuBar( menuBar );
	CreateStatusBar();
	SetStatusText( "Welcome to wxWidgets!" );
	wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
	sizer->Add(new MyCanvas(this));
	sizer->Add(new MyPanel(this));
	SetSizer(sizer);

}
void MyFrame::OnExit(wxCommandEvent& event)
{
	Close( true );
}
void MyFrame::OnAbout(wxCommandEvent& event)
{
	wxMessageBox( "This is a wxWidgets' Hello world sample",
				  "About Hello World", wxOK | wxICON_INFORMATION );
}
void MyFrame::OnHello(wxCommandEvent& event)
{
	wxLogMessage("Hello world from wxWidgets!");
}

MyPanel::MyPanel(wxFrame* parent)
		:wxPanel(parent)
{
	wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
	sizer->Add(new wxButton(this, -1, "A Really Really Big Button"), 0, 0, 0);
	sizer->Add(new wxButton(this, -1, "Tiny Button"), 0, 0, 0);
	SetSizer(sizer);
}

MyCanvas::MyCanvas(wxFrame *parent)
		:wxGLCanvas (parent, wxID_ANY, NULL, wxDefaultPosition, wxDefaultSize, 0, "GLCanvas", wxNullPalette)
{
    int argc = 1;
    char* argv[1] = { wxString((wxTheApp->argv)[0]).char_str() };

	/*
	NOTE: this example uses GLUT in order to have a free teapot model
	to display, to show 3D capabilities. GLUT, however, seems to cause problems
	on some systems. If you meet problems, first try commenting out glutInit(),
	then try comeenting out all glut code
	*/
    glutInit(&argc, argv);
}


void MyCanvas::Paintit(wxPaintEvent& WXUNUSED(event))
{
    Render();
}

void MyCanvas::Render()
{
    // SetCurrent();
    wxPaintDC(this);
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
    // SwapBuffers();
}