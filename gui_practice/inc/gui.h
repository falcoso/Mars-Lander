#include <wx/wx.h>
#include <wx/glcanvas.h>

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


class MyApp: public wxApp
{
public:
	virtual bool OnInit();
};


class MyFrame: public wxFrame
{
public:
	MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
private:
	wxMenuBar *menuBar;
	wxBoxSizer *mainsizer;
	void OnHello(wxCommandEvent& event);
	void OnExit(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);
	wxDECLARE_EVENT_TABLE();
};


enum
{
	ID_Hello = 1
};


wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
	EVT_MENU(ID_Hello,   MyFrame::OnHello)
	EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
	EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()
wxIMPLEMENT_APP(MyApp);


class MyPanel: public wxPanel
{
public:
	MyPanel(wxFrame* parent);
};


class MyCanvas: public wxGLCanvas
{
        void Render();
public:
    MyCanvas(wxFrame* parent);
    void Paintit(wxPaintEvent& event);
protected:
    DECLARE_EVENT_TABLE()
};


BEGIN_EVENT_TABLE(MyCanvas, wxGLCanvas)
    EVT_PAINT    (MyCanvas::Paintit)
END_EVENT_TABLE()