#ifndef GUI
#define GUI

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


class MyPanel: public wxPanel
{
public:
	MyPanel(wxFrame* parent);
};

#endif