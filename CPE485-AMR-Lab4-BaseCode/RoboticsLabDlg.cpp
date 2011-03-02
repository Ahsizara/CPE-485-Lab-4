// RoboticsLabDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RoboticsLab.h"
#include "RoboticsLabDlg.h"
#include "Math.h"



#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define headVerReset 2750//3375 //1.5ms *2250 
#define headHorReset 3375
#define headHorMax 4375
#define headHorMin 2375
#define headVerMax 4375
#define headVerMin 2375
#define headMoveTime 300 //200ms
#define headMoveStep 150

/////////////////////////////////////////////////////////////////////////////
// CRoboticsLabDlg dialog

CRoboticsLabDlg::CRoboticsLabDlg(CWnd* pParent /*=NULL*/, Robot* _robot)
	: CDialog(CRoboticsLabDlg::IDD, pParent)
	, _DisplayParticles(TRUE)
	, _DisplaySimRobot(TRUE)
	, _DisplayNodes(TRUE)
{
	//{{AFX_DATA_INIT(CX80MultiRobotDlg)
	m_Encoder1 = 0;
	m_Encoder2 = 0;
	m_IR1 = 0;
	m_IR2 = 0;
	m_IR3 = 0;
	m_IR4 = 0;
	m_IR5 = 0;
	m_IR6 = 0;
	m_IR7 = 0;
	m_Sonar1 = 0;
	m_Sonar2 = 0;
	m_Sonar3 = 0;
	StopThreads = false;


	// Open logging data file
	myfile.open ("dataLog.txt");


	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	robot_dlg = _robot;
	openGLControl = new COpenGLControl(robot_dlg);
	
	// Create thread to continue to update OpenGL window
    DLGTHREADSTRUCT *_paramUpdate = new DLGTHREADSTRUCT;
    _paramUpdate->_this = this;
    AfxBeginThread (StartDlgUpdateThread, _paramUpdate);

	// Create thread to run a robot control thread
    DLGTHREADSTRUCT *_paramControl = new DLGTHREADSTRUCT;
    _paramControl->_this = this;
    AfxBeginThread (StartRobotControlThread, _paramControl);

}

void CRoboticsLabDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CX80MultiRobotDlg)
	DDX_Control(pDX, IDC_PlayAudio, m_playMusic);
	DDX_Control(pDX, IDC_DRROBOTSDKCONTROLCTRL1, m_MOTSDK_real);
	DDX_Text(pDX, IDC_Encoder1, m_Encoder1);
	DDX_Text(pDX, IDC_Encoder2, m_Encoder2);
	DDX_Text(pDX, IDC_IR1, m_IR1);
	DDX_Text(pDX, IDC_IR2, m_IR2);
	DDX_Text(pDX, IDC_IR3, m_IR3);
	DDX_Text(pDX, IDC_IR4, m_IR4);
	DDX_Text(pDX, IDC_IR5, m_IR5);
	DDX_Text(pDX, IDC_IR6, m_IR6);
	DDX_Text(pDX, IDC_IR7, m_IR7);
	DDX_Text(pDX, IDC_Sonar1, m_Sonar1);
	DDX_Text(pDX, IDC_Sonar2, m_Sonar2);
	DDX_Text(pDX, IDC_Sonar3, m_Sonar3);
	//}}AFX_DATA_MAP
	DDX_Control(pDX, IDC_EDIT_DESIREDX, desiredXPosition);
	DDX_Control(pDX, IDC_EDIT_DESIREDY, desiredYPosition);
	DDX_Control(pDX, IDC_EDIT_DESIREDT, desiredTPosition);
	DDX_Control(pDX, IDC_RADIO_ROBOTTYPE_SIMULATOR, robotTypeSimulator);
	DDX_Control(pDX, IDC_RADIO_ROBOTTYPE_X80, robotTypeX80);
	DDX_Check(pDX, IDC_DISPLAY_PARTICLES, _DisplayParticles);
	DDX_Check(pDX, IDC_DISPLAY_SIMROBOT, _DisplaySimRobot);
	DDX_Check(pDX, IDC_DISPLAY_NODES, _DisplayNodes);
}

BEGIN_MESSAGE_MAP(CRoboticsLabDlg, CDialog)
	//{{AFX_MSG_MAP(CRoboticsLabDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_Forward, OnForward)
	ON_BN_CLICKED(IDC_Back, OnBack)
	ON_BN_CLICKED(IDC_Stop, OnStop)
	ON_BN_CLICKED(IDC_TurnLeft, OnTurnLeft)
	ON_BN_CLICKED(IDC_TurnRight, OnTurnRight)
	ON_BN_CLICKED(IDC_PlayAudio, OnPlayAudio)
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON1, &CRoboticsLabDlg::OnTrackPoint)
	ON_BN_CLICKED(IDC_RADIO_ROBOTTYPE_SIMULATOR, &CRoboticsLabDlg::OnRobotTypeSimulator)
	ON_BN_CLICKED(IDC_RADIO_ROBOTTYPE_X80, &CRoboticsLabDlg::OnRobotTypeX80)
	ON_BN_CLICKED(IDC_CameraUp, &CRoboticsLabDlg::OnCameraUp)
	ON_BN_CLICKED(IDC_CameraDown, &CRoboticsLabDlg::OnCameraDown)
	ON_BN_CLICKED(IDC_CameraRight, &CRoboticsLabDlg::OnCameraRight)
	ON_BN_CLICKED(IDC_CameraLeft, &CRoboticsLabDlg::OnCameraLeft)
	ON_BN_CLICKED(IDC_CameraHome, &CRoboticsLabDlg::OnCameraHome)
	ON_BN_CLICKED(IDC_ZOOM_PLUS, &CRoboticsLabDlg::OnZoomPlus)
	ON_BN_CLICKED(IDC_ZOOM_MINUS, &CRoboticsLabDlg::OnZoomMinus)
	ON_BN_CLICKED(IDC_TILT_PLUS, &CRoboticsLabDlg::OnTiltPlus)
	ON_BN_CLICKED(IDC_TILT_MINUS, &CRoboticsLabDlg::OnTiltMinus)
	ON_BN_CLICKED(IDC_DISPLAY_PARTICLES, &CRoboticsLabDlg::OnDisplayParticles)
	ON_BN_CLICKED(IDC_DISPLAY_SIMROBOT, &CRoboticsLabDlg::OnDisplaySimrobot)
	ON_BN_CLICKED(IDC_DISPLAY_NODES, &CRoboticsLabDlg::OnDisplayNodes)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CRoboticsLabDlg message handlers

BOOL CRoboticsLabDlg::OnInitDialog()
{

	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
			
	//OpenGL window setup
	CRect rect;
	GetDlgItem(IDC_OPENGL_WINDOW)->GetWindowRect(rect);
	ScreenToClient(rect);
	openGLControl->Create(rect,this);

	// Connect to robot
	m_MOTSDK_real.connectRobot ("drrobot1");
	musicInPlay = false;
	SetTimer(1, 450, NULL);


	// Update the point tracking desired states
	UpdateTextBoxes();

	// Set default radio buttons
	robotTypeSimulator.SetCheck(1);
	robotTypeX80.SetCheck(0);

	//Reset Head servo
	m_headVer = headVerReset;
	m_headHor = headHorReset;
	m_MOTSDK_real.EnableServo (0);
	m_MOTSDK_real.EnableServo (1);
	m_MOTSDK_real.ServoTimeCtrAll (headVerReset, headHorReset, NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,headMoveTime);



	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRoboticsLabDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRoboticsLabDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRoboticsLabDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

BEGIN_EVENTSINK_MAP(CRoboticsLabDlg, CDialog)
    //{{AFX_EVENTSINK_MAP(CX80MultiRobotDlg)
	ON_EVENT(CRoboticsLabDlg, IDC_DRROBOTSDKCONTROLCTRL1, 1 /* StandardSensorEvent */, OnStandardSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	ON_EVENT(CRoboticsLabDlg, IDC_DRROBOTSDKCONTROLCTRL1, 3 /* CustomSensorEvent */, OnCustomSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	ON_EVENT(CRoboticsLabDlg, IDC_DRROBOTSDKCONTROLCTRL1, 2 /* MotorSensorEvent */, OnMotorSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	//}}AFX_EVENTSINK_MAP
END_EVENTSINK_MAP()

void CRoboticsLabDlg::OnStandardSensorEventDrrobotsdkcontrolctrl1() 
{
}

void CRoboticsLabDlg::OnCustomSensorEventDrrobotsdkcontrolctrl1() 
{
}

void CRoboticsLabDlg::OnMotorSensorEventDrrobotsdkcontrolctrl1() 
{
}

void CRoboticsLabDlg::OnForward() 
{

	// Set Control Mode to 
	robot_dlg->controllerType = CONTROLLERTYPE_MANUALCONTROL;
	
	// Set Desired Speed	
	double desiredWheelSpeed1 = -0.1*encoderResolution/(2*3.14*wheelRadius);//Left Wheel
	double desiredWheelSpeed2 = +0.1*encoderResolution/(2*3.14*wheelRadius);//Right Wheel
	m_MOTSDK_real.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (0, 30, 30, 0);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (1, 30, 30, 0);
	m_MOTSDK->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeed1,(short)desiredWheelSpeed2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);

}

void CRoboticsLabDlg::OnBack() 
{
	// TODO: Add your control notification handler code here

	// Set Control Mode to 
	robot_dlg->controllerType = CONTROLLERTYPE_MANUALCONTROL;
	
	// Set Desired Speed	
	double desiredWheelSpeed1 = +0.1*encoderResolution/(2*3.14*wheelRadius);//Left Wheel
	double desiredWheelSpeed2 = -0.1*encoderResolution/(2*3.14*wheelRadius);//Right Wheel
	m_MOTSDK_real.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (1, 30, 10, 0);
	m_MOTSDK->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeed1,(short)desiredWheelSpeed2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);
}

void CRoboticsLabDlg::OnStop() 
{
	// TODO: Add your control notification handler code here

	// Set Control Mode to 
	robot_dlg->controllerType = CONTROLLERTYPE_MANUALCONTROL;
	
	// Set Desired Speed	
	m_MOTSDK->DcMotorVelocityNonTimeCtrAll (0,0,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL); 
	m_MOTSDK_real.DisableDcMotor (0);
	m_MOTSDK_real.DisableDcMotor (1);

}

void CRoboticsLabDlg::OnTurnLeft() 
{
	// Set Control Mode to 
	robot_dlg->controllerType = CONTROLLERTYPE_MANUALCONTROL;
	
	// Set Desired Speed	
	double desiredWheelSpeed1 = +0.05*encoderResolution/(2*3.14*wheelRadius);//Left Wheel
	double desiredWheelSpeed2 = +0.05*encoderResolution/(2*3.14*wheelRadius);//Right Wheel
	m_MOTSDK_real.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (1, 30, 10, 0);
	m_MOTSDK->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeed1,(short)desiredWheelSpeed2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);

}

void CRoboticsLabDlg::OnTurnRight() 
{
	// Set Control Mode to 
	robot_dlg->controllerType = CONTROLLERTYPE_MANUALCONTROL;
	
	// Set Desired Speed	
	double desiredWheelSpeed1 = -0.05*encoderResolution/(2*3.14*wheelRadius);//Left Wheel
	double desiredWheelSpeed2 = -0.05*encoderResolution/(2*3.14*wheelRadius);//Right Wheel
	m_MOTSDK_real.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK_real.SetDcMotorVelocityControlPID (1, 30, 10, 0);
	m_MOTSDK->DcMotorVelocityNonTimeCtrAll((short)desiredWheelSpeed1,(short)desiredWheelSpeed2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);
}


void CRoboticsLabDlg::OnPlayAudio() 
{
	// TODO: Add your control notification handler code here
	char buffer[200];
	
	if (!musicInPlay)
	{ 
		musicInPlay = TRUE;
	
		m_playMusic.SetWindowText ("Stop Music");
		
		::GetCurrentDirectory(200,buffer);
		strcat_s(buffer,"\\happy8k.wav");

		m_MOTSDK_real.PlayAudioFile(buffer);
		
	} else
	{
	m_MOTSDK_real.StopAudioPlay ();
	m_playMusic.SetWindowText ("Play Music");
	musicInPlay = FALSE;
	
	}	
}

void CRoboticsLabDlg::OnCancel() 
{
	// TODO: Add extra cleanup here
	m_MOTSDK_real.StopAudioPlay ();
	m_MOTSDK_real.DisableDcMotor (0);
	m_MOTSDK_real.DisableDcMotor (1);

	// Default Point to Simulator
	m_MOTSDK = &m_MOTSDK_simulator;

	// End Threads
	StopThreads = true;
	//AfxEndThread( 
		//(StartDlgUpdateThread);

	
	// Close data log file
	myfile.close();

	//Sleep(5000);

	//openGLControl->~COpenGLControl();
	
	CDialog::OnCancel();
	//EndDialog(5);
}

void CRoboticsLabDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	m_MOTSDK_real.TakePhoto ();
	CDialog::OnTimer(nIDEvent);
	UpdateData(false);
}

UINT CRoboticsLabDlg::StartDlgUpdateThread (LPVOID param){

	DLGTHREADSTRUCT*	ts = (DLGTHREADSTRUCT*)param;

	Sleep(3000);

	// Default Point to Simulator so that encoders can point to something
	ts->_this->m_MOTSDK = &(ts->_this->m_MOTSDK_simulator);


	// Update openGL screen until terminated from dialog OK
	while(!ts->_this->openGLControl->terminate && !ts->_this->StopThreads)
	{

		// Update Encoders
		ts->_this->m_Encoder1 = ts->_this->m_MOTSDK->GetEncoderPulse1();
		ts->_this->m_Encoder2 = ts->_this->m_MOTSDK->GetEncoderPulse2();

		// Udpate Sonars
		ts->_this->m_Sonar1= ts->_this->m_MOTSDK->GetSensorSonar1 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_Sonar2= ts->_this->m_MOTSDK->GetSensorSonar2 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_Sonar3= ts->_this->m_MOTSDK->GetSensorSonar3 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);

		// Update IR sensors
		ts->_this->m_IR1 = ts->_this->m_MOTSDK->GetSensorIRRange (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);	
		ts->_this->m_IR2 = ts->_this->m_MOTSDK->GetCustomAD3 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_IR3 = ts->_this->m_MOTSDK->GetCustomAD4 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_IR4 = ts->_this->m_MOTSDK->GetCustomAD5 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_IR5 = ts->_this->m_MOTSDK->GetCustomAD6 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_IR6 = ts->_this->m_MOTSDK->GetCustomAD7 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);
		ts->_this->m_IR7 = ts->_this->m_MOTSDK->GetCustomAD8 (ts->_this->robot_dlg->x, ts->_this->robot_dlg->y, ts->_this->robot_dlg->t);

		//ts->_this->openGLControl->Invalidate();
		//ts->_this->openGLControl->UpdateWindow();

		// Send WM_PAINT Message
		RECT DialogRegion;
		CRgn Region; 
		Region.CreateRectRgnIndirect(&DialogRegion);
		//ts->_this->InvalidateRgn(&Region,TRUE);
		ts->_this->openGLControl->Invalidate(TRUE);
		//ts->_this->OnPaint();
		
		// Sleep to approximate 10 Hz update rate
		Sleep(100);
	}
	//AfxEndThread(0);
	
	return 1;
}

UINT CRoboticsLabDlg::StartRobotControlThread (LPVOID param){

	// Point to dialog
	DLGTHREADSTRUCT*	ts = (DLGTHREADSTRUCT*)param;

	// Default Point to Simulator
	ts->_this->m_MOTSDK = &(ts->_this->m_MOTSDK_simulator);

	// Run Control Loop
	Sleep(500);
	CWiRobotSDK* m_MOTSDK_rob = ts->_this->m_MOTSDK;

	// Send stop Control signals
	m_MOTSDK_rob->DcMotorVelocityNonTimeCtrAll(0,0,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);


	// Run Control Loop
	while(!ts->_this->StopThreads) {

		// Call control loop function
		ts->_this->robot_dlg->RunControlLoop(ts->_this->m_MOTSDK);

		// Log data 
		if(true)
			ts->_this->myfile << ts->_this->robot_dlg->x <<" "<< ts->_this->robot_dlg->y <<" "<< ts->_this->robot_dlg->t <<" "<< ts->_this->robot_dlg->x_est <<" "<< ts->_this->robot_dlg->y_est <<" "<< ts->_this->robot_dlg->t_est <<"\n";

	}

	//AfxEndThread(0);
	
		//ts->_this->InvalidateRgn(&Region,TRUE);
	return 1;
}




void CRoboticsLabDlg::OnTrackPoint()
{
	// TODO: Add your control notification handler code here

	// Grab text from window
	CString testString;
	desiredXPosition.GetWindowTextA(testString);
	robot_dlg->desiredX = atof(testString);
	desiredYPosition.GetWindowTextA(testString);
	robot_dlg->desiredY = atof(testString);
	desiredTPosition.GetWindowTextA(testString);
	robot_dlg->desiredT = atof(testString);

	// Put limits on desired points
	robot_dlg->desiredX = min(1.8, robot_dlg->desiredX);
	robot_dlg->desiredX = max(-1.8, robot_dlg->desiredX);
	robot_dlg->desiredY = min(1.8, robot_dlg->desiredY);
	robot_dlg->desiredY = max(-1.8, robot_dlg->desiredY);
	robot_dlg->desiredT = fmod(robot_dlg->desiredT,2*pi);
	if ( robot_dlg->desiredT > pi) {robot_dlg->desiredT -= 2*pi;} else if (robot_dlg->desiredT < -pi) {robot_dlg->desiredT += 2*pi;}


	// Set start and goal points
	robot_dlg->x_start = robot_dlg->x_est; 
	robot_dlg->y_start = robot_dlg->y_est; 
	robot_dlg->t_start = robot_dlg->t_est; 
	robot_dlg->x_goal = robot_dlg->desiredX; 
	robot_dlg->y_goal = robot_dlg->desiredY; 
	robot_dlg->t_goal = robot_dlg->desiredT;//atan2(robot_dlg->y_goal - robot_dlg->y_start, robot_dlg->x_goal - robot_dlg->x_start); 

	UpdateTextBoxes();

	// Turn Point tracker on
	robot_dlg->controllerType = CONTROLLERTYPE_POINTTRACKER;

	// Call for motion plan to be constructured
	robot_dlg->motionPlanRequired = true;

}

void CRoboticsLabDlg::UpdateTextBoxes()
{
	CString testString;
	testString.Format("%7.3f", robot_dlg->desiredX); 
	desiredXPosition.SetWindowTextA(testString);
	testString.Format("%7.3f", robot_dlg->desiredY); 
	desiredYPosition.SetWindowTextA(testString);
	testString.Format("%7.3f", robot_dlg->desiredT); 
	desiredTPosition.SetWindowTextA(testString);
	UpdateData(false);
}
void CRoboticsLabDlg::OnRobotTypeSimulator()
{
	// TODO: Add your control notification handler code here

	// Set radio buttons
	robotTypeSimulator.SetCheck(1);
	robotTypeX80.SetCheck(0);

	// Default Point to Simulator
	m_MOTSDK = &m_MOTSDK_simulator;
	robot_dlg->x = initialX;
	robot_dlg->y = initialY;
	robot_dlg->t = initialT;
	robot_dlg->	InitializeParticles();
	Sleep(100);

}

void CRoboticsLabDlg::OnRobotTypeX80()
{
	// TODO: Add your control notification handler code here

	// Set radio buttons
	robotTypeSimulator.SetCheck(0);
	robotTypeX80.SetCheck(1);


	// Try to Point to X80 connection
	m_MOTSDK = &m_MOTSDK_real;
	Sleep(100);
	robot_dlg->x = initialX;
	robot_dlg->y = initialY;
	robot_dlg->t = initialT;
	robot_dlg->	InitializeParticles();
	Sleep(100);

}

void CRoboticsLabDlg::OnCameraUp()
{
	// Tilt head up
	if (m_headVer < headVerMax - headMoveStep)
	{
		m_headVer += headMoveStep;
		m_MOTSDK_real.ServoTimeCtrAll (m_headVer, m_headHor, NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,headMoveTime);
	}
}

void CRoboticsLabDlg::OnCameraDown()
{
	// Tilt Head down
	if (m_headVer > headVerMin + headMoveStep)
	{
		m_headVer -= headMoveStep;
		m_MOTSDK_real.ServoTimeCtrAll (m_headVer, m_headHor, NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL, headMoveTime);
	}
}

void CRoboticsLabDlg::OnCameraRight()
{
	// Pan Camera Right
	if (m_headHor > headHorMin + headMoveStep)
	{
		m_headHor -= headMoveStep;
		m_MOTSDK_real.ServoTimeCtrAll (m_headVer, m_headHor, NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL, headMoveTime);
	}
}

void CRoboticsLabDlg::OnCameraLeft()
{
	// Pan Camera Left
	if (m_headHor < headHorMax - headMoveStep)
	{
		m_headHor += headMoveStep;
		m_MOTSDK_real.ServoTimeCtrAll (m_headVer, m_headHor,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,headMoveTime);
	}
}

void CRoboticsLabDlg::OnCameraHome()
{
	// Reset head position
	m_headVer = headVerReset;
	m_headHor = headHorReset;
	m_MOTSDK_real.ServoTimeCtrAll (headVerReset, headHorReset, NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,3*headMoveTime);
}

void CRoboticsLabDlg::OnZoomPlus()
{
	// TODO: Add your control notification handler code here
	robot_dlg->zoom -= 0.25;
	robot_dlg->zoom = max(0.5, robot_dlg->zoom);
}

void CRoboticsLabDlg::OnZoomMinus()
{
	// TODO: Add your control notification handler code here
	robot_dlg->zoom += 0.25;
	robot_dlg->zoom = min(2.5, robot_dlg->zoom);

}

void CRoboticsLabDlg::OnTiltPlus()
{
	// TODO: Add your control notification handler code here
	robot_dlg->tiltAngle -= 5.0;
}

void CRoboticsLabDlg::OnTiltMinus()
{
	// TODO: Add your control notification handler code here
	robot_dlg->tiltAngle += 5.0;
}

void CRoboticsLabDlg::OnDisplayParticles()
{
	// TODO: Add your control notification handler code here
	_DisplayParticles = !_DisplayParticles;
	robot_dlg->displayParticles = _DisplayParticles;
}

void CRoboticsLabDlg::OnDisplaySimrobot()
{
	// TODO: Add your control notification handler code here
	_DisplaySimRobot = !_DisplaySimRobot;
	robot_dlg->displaySimRobot = _DisplaySimRobot;
}

void CRoboticsLabDlg::OnDisplayNodes()
{
	// TODO: Add your control notification handler code here
	_DisplayNodes = !_DisplayNodes;
	robot_dlg->displayNodes = _DisplayNodes;
}
