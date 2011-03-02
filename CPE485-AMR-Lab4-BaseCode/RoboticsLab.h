// RoboticsLab.h : main header file for the X80MULTIROBOT application
//

#if !defined(AFX_X80MULTIROBOT_H__5B97B6B0_6545_48CB_BA0A_EB5F82A27FF2__INCLUDED_)
#define AFX_X80MULTIROBOT_H__5B97B6B0_6545_48CB_BA0A_EB5F82A27FF2__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef __AFXWIN_H__
	#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"		// main symbols
#include "Robot.h"
#include "RoboticsLabDlg.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////////////
// CRoboticsLabApp:
// See RobitcsLab.cpp for the implementation of this class
//

class CRoboticsLabApp : public CWinApp
{
public:
	CRoboticsLabApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CX80MultiRobotApp)
	public:

		virtual BOOL InitInstance();
		Robot* robot_app;
		CRoboticsLabDlg* dlg;

		static UINT StartAppThread (LPVOID param);	//controlling function header

		typedef struct APPTHREADSTRUCT				//structure for passing to the controlling function
		{
			CRoboticsLabApp*	_this;
		} APPTHREADSTRUCT;

	//}}AFX_VIRTUAL

// Implementation

	//{{AFX_MSG(CRoboticsLabApp)
		// NOTE - the ClassWizard will add and remove member functions here.
		//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};


/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_X80MULTIROBOT_H__5B97B6B0_6545_48CB_BA0A_EB5F82A27FF2__INCLUDED_)
