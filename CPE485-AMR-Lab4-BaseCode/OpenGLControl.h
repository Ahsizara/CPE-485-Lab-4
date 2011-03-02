#if !defined(AFX_OPENGLCONTROL_H__52A6B63B_01A2_449D_8691_1FF59EECAB71__INCLUDED_)
#define AFX_OPENGLCONTROL_H__52A6B63B_01A2_449D_8691_1FF59EECAB71__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// OpenGLControl.h : Header-Datei
//

#include "OpenGLDevice.h"
#include "Robot.h"
#include "Landmark.h"
#include <GL\gl.h>
#include <GL\glu.h>


const float gridCellWidth=				0.2f;
const float floorWidth=					4.0f;
const int numCols=					(int)(1+ floorWidth/gridCellWidth);
const int numRows=					(int)(1+ floorWidth/gridCellWidth);



/////////////////////////////////////////////////////////////////////////////
// Fenster COpenGLControl 

class COpenGLControl : public CWnd
{
// Konstruktion
public:
	COpenGLControl(Robot* _robot);

// Attribute
public:
	Robot* robot_ctl;
	float tricolor;
	bool terminate;
	float	gridLineOffsetVertical;
	float	gridLineOffsetHorizontal;
	float	xCamera;
	float	yCamera;
	float	zCamera;
	float	xCameraTarget;
	float	yCameraTarget;
	float	zCameraTarget;
	float	xCameraUp;
	float	yCameraUp;
	float	zCameraUp;

	// Display List 
	GLuint X80Robot;
	GLuint Cone;
	GLuint Particle;
	GLuint X80RobotEstimate;
	GLuint PRMNode;


// Operationen
public:

// Überschreibungen
	// Vom Klassen-Assistenten generierte virtuelle Funktionsüberschreibungen
	//{{AFX_VIRTUAL(COpenGLControl)
	public:
	//}}AFX_VIRTUAL

// Implementierung
public:
	void Create(CRect rect,CWnd* parent);
	virtual ~COpenGLControl();
	GLvoid BuildLists();





	// Generierte Nachrichtenzuordnungsfunktionen
protected:
	void InitGL();
	void DrawGLScene();
	OpenGLDevice openGLDevice;
	CClientDC* dc;
	float rotation;

	//{{AFX_MSG(COpenGLControl)
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ fügt unmittelbar vor der vorhergehenden Zeile zusätzliche Deklarationen ein.

#endif // AFX_OPENGLCONTROL_H__52A6B63B_01A2_449D_8691_1FF59EECAB71__INCLUDED_
