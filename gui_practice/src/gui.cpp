#include "gui.h"
#include "canvas.h"

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
	EVT_MENU(ID_Hello,   MyFrame::OnHello)
	EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
	EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()
wxIMPLEMENT_APP(MyApp);

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

	wxBoxSizer *glsizer = new wxBoxSizer(wxVERTICAL);
	wxBoxSizer *viewsizer = new wxBoxSizer(wxHORIZONTAL);
	viewsizer->Add(new MyCanvas(this, true), 1, wxEXPAND | wxALL, 1);
	viewsizer->Add(new MyCanvas(this), 1, wxEXPAND | wxALL, 1);

	glsizer->Add(viewsizer, 1, wxEXPAND | wxALL, 1);
	glsizer->Add(new MyCanvas(this), 1, wxEXPAND | wxALL, 1);

	wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
	sizer->Add(glsizer, 1, wxEXPAND | wxALL, 1);
	// sizer->Add(new MyCanvas(this), 1, wxEXPAND | wxALL, 10);
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