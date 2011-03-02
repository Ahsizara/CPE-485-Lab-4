// RoboticsLabDlg.h : header file
//
//{{AFX_INCLUDES()
#include <iostream>
#include <fstream>
using namespace std;
#include "OpenGLControl.h"
#include "Robot.h"
#include "afxwin.h"
#include "wirobotsdk.h"
//}}AFX_INCLUDES

#if !defined(AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_)
#define AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


/////////////////////////////////////////////////////////////////////////////
// CRoboticsLabDlg dialog

class CRoboticsLabDlg : public CDialog
{
// Construction
public:
	bool musicInPlay;
	short m_headVer;
	short m_headHor;

	CRoboticsLabDlg(CWnd* pParent = NULL, Robot* _robot = NULL);	// standard constructor
	static UINT StartDlgUpdateThread (LPVOID param);	//controlling function header
	static UINT StartRobotControlThread (LPVOID param);	//controlling function header
	//void robotMove(int robotType, short _cmd1, short _cmd2);
	void UpdateTextBoxes();

// Dialog Data
	//{{AFX_DATA(CX80MultiRobotDlg)
	enum { IDD = IDD_X80MULTIROBOT_DIALOG };
	CButton	m_playMusic;
	CWiRobotSDK*	m_MOTSDK;
	CWiRobotSDK	m_MOTSDK_real;
	CWiRobotSDK_Simulator	m_MOTSDK_simulator;
	short	m_Encoder1;
	short	m_Encoder2;
	short	m_IR1;
	short	m_IR2;
	short	m_IR3;
	short	m_IR4;
	short	m_IR5;
	short	m_IR6;
	short	m_IR7;
	short	m_Sonar1;
	short	m_Sonar2;
	short	m_Sonar3;
	Robot* robot_dlg;
	ofstream myfile;
	bool StopThreads;


	typedef struct DLGTHREADSTRUCT				//structure for passing to the controlling function
	{
		CRoboticsLabDlg*	_this;
	} DLGTHREADSTRUCT;

	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CRoboticsLabDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;
	COpenGLControl* openGLControl;

	// Generated message map functions
	//{{AFX_MSG(CRoboticsLabDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnStandardSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnCustomSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnMotorSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnForward();
	afx_msg void OnBack();
	afx_msg void OnStop();
	afx_msg void OnTurnLeft();
	afx_msg void OnTurnRight();
	afx_msg void OnPlayAudio();
	virtual void OnCancel();
	afx_msg void OnTimer(UINT nIDEvent);
	DECLARE_EVENTSINK_MAP()
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	CEdit desiredXPosition;
public:
	afx_msg void OnTrackPoint();
public:
	CEdit desiredYPosition;
public:
	CEdit desiredTPosition;
public:
	CButton robotTypeSimulator;
public:
	CButton robotTypeX80;
public:
	afx_msg void OnRobotTypeSimulator();
public:
	afx_msg void OnRobotTypeX80();
public:
	afx_msg void OnCameraUp();
public:
	afx_msg void OnCameraDown();
public:
	afx_msg void OnCameraRight();
public:
	afx_msg void OnCameraLeft();
public:
	afx_msg void OnCameraHome();
public:
	afx_msg void OnEnChangeIr7();
public:
	afx_msg void OnZoomPlus();
public:
	afx_msg void OnZoomMinus();
public:
	afx_msg void OnTiltPlus();
public:
	afx_msg void OnTiltMinus();
public:
	afx_msg void OnDisplayParticles();
public:
	BOOL _DisplayParticles;
public:
	BOOL _DisplaySimRobot;
public:
	BOOL _DisplayNodes;
public:
	afx_msg void OnDisplaySimrobot();
public:
	afx_msg void OnDisplayNodes();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_)
