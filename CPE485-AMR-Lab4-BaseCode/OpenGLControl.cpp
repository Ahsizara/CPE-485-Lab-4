// OpenGLControl.cpp: Implementierungsdatei
//

#include "stdafx.h"
#include "RoboticsLab.h"
#include "OpenGLControl.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// COpenGLControl

COpenGLControl::COpenGLControl(Robot* _robot)
{
	dc = NULL;
	rotation = 5.0f;
	tricolor = 0.0f;
	terminate = false;
	robot_ctl = _robot;

}

COpenGLControl::~COpenGLControl()
{
	if (dc)
	{
		delete dc;
	}
}


BEGIN_MESSAGE_MAP(COpenGLControl, CWnd)
	//{{AFX_MSG_MAP(COpenGLControl)
	ON_WM_PAINT()
	ON_WM_SIZE()
	ON_WM_CREATE()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// Behandlungsroutinen für Nachrichten COpenGLControl 


void COpenGLControl::InitGL()
{
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);							
	//glEnable(GL_DEPTH_TEST);					
	glDepthFunc(GL_LEQUAL);	
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_BACK, GL_FILL);
	BuildLists();
}

void COpenGLControl::DrawGLScene()
{
	// Change zoom
	//glViewport(0,0,cx,cy);	
	glMatrixMode(GL_PROJECTION);						
	glLoadIdentity();					
	glOrtho(-robot_ctl->zoom,robot_ctl->zoom,-robot_ctl->zoom,robot_ctl->zoom,50.0f,-50.0f);
	glMatrixMode(GL_MODELVIEW);						
	glLoadIdentity();




	glClear(GL_COLOR_BUFFER_BIT
		 |  GL_DEPTH_BUFFER_BIT);


	glLoadIdentity();

	//***************************
	// DRAWING CODE
	//***************************
	glPushMatrix();
	glRotatef(180.0,1.0f,0.0f,0.0f);
	glRotatef(45.0,0.0f,0.0f,1.0f);
	glRotatef((GLfloat)robot_ctl->tiltAngle,1.0f,-1.0f,0.0f);
	
	// Floor		
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	glBegin(GL_QUADS);
		glVertex3f(-floorWidth/2, +floorWidth/2, 0.0f);
		glVertex3f(+floorWidth/2, +floorWidth/2, 0.0f);
		glVertex3f(+floorWidth/2, -floorWidth/2, 0.0f);
		glVertex3f(-floorWidth/2, -floorWidth/2, 0.0f);
	glEnd();
	

	// Grid
	glPushMatrix();
	glColor4f(0.15f,0.15f,0.15f,0.1f);
	glLineWidth(1.0f);							
	glDisable(GL_LINE_SMOOTH);						// Disable Antialiasing
	glBegin(GL_LINES);
		for (int row=0; row<=numRows; row++) {
			glVertex3f(-floorWidth/2, -floorWidth/2+gridCellWidth*row,0.01f);
			glVertex3f(+floorWidth/2,  -floorWidth/2+gridCellWidth*row,0.01f);
		}
		for (int col=0; col<=numCols; col++){
			glVertex3f(-floorWidth/2+gridCellWidth*col, -floorWidth/2,0.01f);
			glVertex3f(-floorWidth/2+gridCellWidth*col, +floorWidth/2,0.01f);
		}
    glEnd();	
	glPopMatrix();
	
	// Draw Robot
	if (robot_ctl->displaySimRobot){

		glPushMatrix();
		glTranslatef((GLfloat)robot_ctl->x,reflectY*(GLfloat)robot_ctl->y,0.0f);
		glRotatef(reflectY*(GLfloat)(-1.57 + robot_ctl->t*180/3.14),0.0f, 0.0f, 1.0f);
		glCallList(X80Robot);
		glPopMatrix();
	}

	// Draw State Estimate;
	glPushMatrix();
	glTranslatef((GLfloat)robot_ctl->x_est,(GLfloat)(reflectY*robot_ctl->y_est), (GLfloat)(robotHeight+0.01));
	glRotatef(reflectY*(GLfloat)(-1.57 + robot_ctl->t_est*180/3.14),0.0f, 0.0f, 1.0f);
	glCallList(X80RobotEstimate);
	glPopMatrix();
	
	// Draw Landmarks
	for (int i=0; i<robot_ctl->robotMap.numLandmarks; i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)robot_ctl->robotMap.landmarks[i].x,reflectY*(GLfloat)robot_ctl->robotMap.landmarks[i].y,0.0f);
		glCallList(Cone);
		glPopMatrix();
	}
	
	// Draw Particles;
	if (robot_ctl->displayParticles){
		for (int i=1; i<numParticles; i++)
		{
			glPushMatrix();
			glTranslatef((GLfloat)robot_ctl->particles[i].x, reflectY*(GLfloat)robot_ctl->particles[i].y, robotHeight);
			glRotatef(reflectY*(GLfloat)(-1.57 + robot_ctl->particles[i].t*180/3.14), 0.0f, 0.0f, 1.0f);
			glCallList(Particle);
			glPopMatrix();
		}
	}

	// Draw Walls
	glColor4f(0.25f,0.25f,0.25f,0.5f);
	for (int i=0; i< robot_ctl->robotMap.numMapSegments; i++)
	{
		glPushMatrix();
		glBegin(GL_POLYGON);
			double X1 = robot_ctl->robotMap.mapSegmentCorners[i][0][0];
			double Y1 = robot_ctl->robotMap.mapSegmentCorners[i][0][1];
			double X2 = robot_ctl->robotMap.mapSegmentCorners[i][1][0];
			double Y2 = robot_ctl->robotMap.mapSegmentCorners[i][1][1];
			//glNormal3d((GLfloat)fabs(Y1-Y2),(GLfloat)fabs(X1-X2),0);
			glVertex3d( (GLfloat)X1, (GLfloat)(reflectY*Y1), (GLfloat)0.0f);
			glVertex3d( (GLfloat)X1, (GLfloat)(reflectY*Y1), (GLfloat)wallHeight);
			glVertex3d( (GLfloat)X2, (GLfloat)(reflectY*Y2), (GLfloat)wallHeight);
			glVertex3d( (GLfloat)X2, (GLfloat)(reflectY*Y2), (GLfloat)0.0f);
		glEnd();
		glPopMatrix();
	}


	// Draw PRM Nodes;
	if (robot_ctl->displayNodes){

		for (int i=0; i<robot_ctl->numNodes; i++)
		{
			glPushMatrix();
			glTranslatef((GLfloat)(robot_ctl->nodeList[i].x), (GLfloat)(reflectY*robot_ctl->nodeList[i].y), (GLfloat)(robotHeight + 0.01));
			glCallList(PRMNode);
			glPopMatrix();

			glBegin(GL_LINES);
				glVertex3f((GLfloat)(robot_ctl->nodeList[robot_ctl->nodeList[i].lastNode].x), (GLfloat)(reflectY*robot_ctl->nodeList[robot_ctl->nodeList[i].lastNode].y), (GLfloat)(robotHeight + 0.01));
				glVertex3f((GLfloat)(robot_ctl->nodeList[i].x), (GLfloat)(reflectY*robot_ctl->nodeList[i].y), (GLfloat)(robotHeight + 0.01));
			glEnd();
		}
	}

	// Draw Traj
	glLineWidth(2.0f);							
	glColor4f(0.25f,0.25f,1.00f,0.5f);
	for (int i=1; i<robot_ctl->trajSize; i++)
	{

		glBegin(GL_LINES);
			glVertex3f((GLfloat)(robot_ctl->trajList[i-1].x), (GLfloat)(reflectY*robot_ctl->trajList[i-1].y), (GLfloat)(robotHeight + 0.05));
			glVertex3f((GLfloat)(robot_ctl->trajList[i].x), (GLfloat)(reflectY*robot_ctl->trajList[i].y), (GLfloat)(robotHeight + 0.05));
		glEnd();

	}

	glPopMatrix();

	SwapBuffers(dc->m_hDC);
}



void COpenGLControl::Create(CRect rect, CWnd *parent)
{
	CString className = AfxRegisterWndClass(
		CS_HREDRAW | CS_VREDRAW | CS_OWNDC,
		NULL,
		(HBRUSH)GetStockObject(BLACK_BRUSH),
		NULL);

	CreateEx(
		0,
		className,
		"OpenGL",
		WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
		rect,
		parent,
		0);

}

void COpenGLControl::OnPaint() 
{
	rotation += 0.01f;

	if (rotation >= 360.0f)
	{
		rotation -= 360.0f;
	}

	/** OpenGL section **/

	openGLDevice.makeCurrent();

	DrawGLScene();
	
	CWnd::OnPaint(); 
}

void COpenGLControl::OnSize(UINT nType, int cx, int cy) 
{
	CWnd::OnSize(nType, cx, cy);
	
	if (cy == 0)								
	{
		cy = 1;						
	}

	// Set up lighting
	//float Light_Ambient[]=  { 0.1f, 0.1f, 0.1f, 1.0f };
	float Light_Diffuse[]=  { 1.2f, 1.2f, 1.2f, 1.0f }; 
	float Light_Ambient[]=  { 0, 0, 0, 1.0f };
	//float Light_Diffuse[]=  { 0, 0, 0, 1.0f }; 
	float Light_Specular[]= {0.95f, 0.95f, 0.95f, 1.0f };
	float Light_Position[]= {-1.0f, -1.0f, -5.0f, 0.0f };

	glEnable(GL_LIGHTING);

	// Set up a light, turn it on.
	glLightfv(GL_LIGHT1, GL_POSITION, Light_Position);
	glLightfv(GL_LIGHT1, GL_AMBIENT,  Light_Ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE,  Light_Diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, Light_Specular);
	glEnable (GL_LIGHT1); 

	glEnable (GL_DEPTH_TEST);

   // A handy trick -- have surface material mirror the color.
   glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
   glEnable(GL_COLOR_MATERIAL);


	glViewport(0,0,cx,cy);	

	glMatrixMode(GL_PROJECTION);						
	glLoadIdentity();						

	
	//glOrtho(-1.5f,1.5f,-1.5f,1.5f,50.0f,-50.0f);
	glOrtho(-robot_ctl->zoom,robot_ctl->zoom,-robot_ctl->zoom,robot_ctl->zoom,50.0f,-50.0f);

	glMatrixMode(GL_MODELVIEW);						
	glLoadIdentity();


}


int COpenGLControl::OnCreate(LPCREATESTRUCT lpCreateStruct) 
{
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	dc = new CClientDC(this);

	openGLDevice.create(dc->m_hDC);
	InitGL();

	return 0;
}

BOOL COpenGLControl::OnEraseBkgnd(CDC* pDC) 
{
	return TRUE;
}

GLvoid COpenGLControl::BuildLists(){
	X80Robot=glGenLists(3);
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();			// Create A Pointer To The Quadric Object
	gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals
	gluQuadricDrawStyle(quadratic, GLU_FILL);	// Create Smooth Normals
	glNewList(X80Robot,GL_COMPILE);
		glPushMatrix();


		// Body
		glColor4f(0.4f,0.4f,0.6f,1.0f);
		glPushMatrix();
		glTranslatef(0.0f,0.0f,robotGroundClearance );
		gluCylinder(quadratic,robotWidth/2,robotWidth/2,robotHeight/2-robotGroundClearance,32,8);
		glPopMatrix();

		// Antenna
		glColor4f(0.0f,0.0f,0.0f,1.0f);
		glPushMatrix();
		glTranslatef(-robotWidth/2 + 0.04f, 0.0f,robotHeight/2 );
		gluCylinder(quadratic,antennaRadius,antennaRadius,antennaHeight,32,8);
		glPopMatrix();


		// Wheel 1
		glColor4f(0.0f,0.0f,0.0f,1.0f);
		glPushMatrix();
		glRotatef(90.0,1.0f,0.0f,0.0f);
		glTranslatef(0.0f,wheelRadius,robotWidth/2 );
		gluCylinder(quadratic,wheelRadius,wheelRadius,wheelWidth/2,32,8);
		glTranslatef(0.0f,0.0f,wheelWidth/2);
		glColor4f(0.3f,0.3f,0.3f,1.0f);
		gluDisk(quadratic,0.0f,wheelRadius,32,8);
		glColor4f(0.6f,0.6f,0.6f,1.0f);
		glTranslatef(0.0f,0.0f,0.001f);
		gluDisk(quadratic,0.0f,0.6*wheelRadius,32,8);

			// Wheel 1 Cover
			glPushMatrix();
			glColor4f(0.6f,0.6f,0.8f,1.0f);
			glTranslatef(0.0f,0.0f,0.001f);
			glBegin(GL_POLYGON);
				glNormal3d(0,0,1);
				glVertex3d( +wheelCoverdim1, -wheelCoverdim2, 0.0f);
				glVertex3d( +wheelCoverdim1, +wheelCoverdim3, 0.0f);
				glVertex3d( -wheelCoverdim1, +wheelCoverdim3, 0.0f);
				glVertex3d( -wheelCoverdim1, -wheelCoverdim2, 0.0f);
			glEnd();
			glBegin(GL_POLYGON);
				glNormal3d(0,1,0);
				glVertex3d( +wheelCoverdim1, +wheelCoverdim3, 0);
				glVertex3d( +wheelCoverdim1, +wheelCoverdim3, -wheelCoverdim4);
				glVertex3d( -wheelCoverdim1, +wheelCoverdim3, -wheelCoverdim4);
				glVertex3d( -wheelCoverdim1, +wheelCoverdim3, 0);
			glEnd();
			glPopMatrix();
		glPopMatrix();

		// Wheel 2
		glColor4f(0.0f,0.0f,0.0f,1.0f);
		glPushMatrix();
		glRotatef(-90.0,1.0f,0.0f,0.0f);
		glTranslatef(0.0f,-wheelRadius,robotWidth/2 );
		gluCylinder(quadratic,wheelRadius,wheelRadius,wheelWidth/2,32,8);
		glTranslatef(0.0f,0.0f,wheelWidth/2);
		glColor4f(0.3f,0.3f,0.3f,1.0f);
		gluDisk(quadratic,0.0f,wheelRadius,32,8);
		glColor4f(0.6f,0.6f,0.6f,1.0f);
		glTranslatef(0.0f,0.0f,0.001f);
		gluDisk(quadratic,0.0f,0.6*wheelRadius,32,8);

			// Wheel 2 Cover
			glPushMatrix();
			glColor4f(0.6f,0.6f,0.8f,1.0f);
			glTranslatef(0.0f,0.0f,0.001f);
			glBegin(GL_POLYGON);
				glNormal3d(0,0,1);
				glVertex3d( +wheelCoverdim1, -wheelCoverdim3, 0.0f);
				glVertex3d( +wheelCoverdim1, +wheelCoverdim2, 0.0f);
				glVertex3d( -wheelCoverdim1, +wheelCoverdim2, 0.0f);
				glVertex3d( -wheelCoverdim1, -wheelCoverdim3, 0.0f);
			glEnd();
			glPopMatrix();
			glBegin(GL_POLYGON);
				glNormal3d(0,-1,0);
				glVertex3d( +wheelCoverdim1, -wheelCoverdim3, 0);
				glVertex3d( +wheelCoverdim1, -wheelCoverdim3, -wheelCoverdim4);
				glVertex3d( -wheelCoverdim1, -wheelCoverdim3, -wheelCoverdim4);
				glVertex3d( -wheelCoverdim1, -wheelCoverdim3, 0);
			glEnd();


		glPopMatrix();

		// Middle Layer	
		glPushMatrix();
		glColor4f(0.4f,0.4f,0.6f,1.0f);
		glTranslatef(0.0f,0.0f,robotHeight/2);
		glNormal3f(0.0f,0.0f,1.0f);
		gluDisk(quadratic,0.0f,robotWidth/2,32,8);
		glPopMatrix();



		// Top
		glPushMatrix();
		glColor4f(0.6f,0.6f,0.8f,1.0f);
		glTranslatef(0.0f,0.0f,robotHeight);
		glBegin(GL_POLYGON);
			glNormal3d(0,0,1);
			glVertex3d( +topdim1, -robotWidth/2, 0.0f);
			glVertex3d( +topdim1, +robotWidth/2, 0.0f);
			glVertex3d( -topdim1, +robotWidth/2, 0.0f);
			glVertex3d( -robotWidth/2*sin(0.99f),  +robotWidth/2*cos(0.99f), 0.0f);
			glVertex3d( -robotWidth/2*sin(0.99f),  -robotWidth/2*cos(0.99f), 0.0f);
			glVertex3d( -topdim1, -robotWidth/2, 0.0f);
		glEnd();
		glPopMatrix();


		// Camera	
		glPushMatrix();
		glTranslatef((GLfloat) (robotWidth*0.4),0.0f,(GLfloat)(robotHeight+0.001));
		glColor4f(0.6f,0.6f,0.8f,1.0f);
		glBegin(GL_POLYGON);
			glNormal3d(0,0,1);
			glVertex3d( +cameradim1, -cameraWidth/2, 0.0f);
			glVertex3d( +cameradim1, +cameraWidth/2, 0.0f);
			glVertex3d( -cameradim1, +cameraWidth/2, 0.0f);
			glVertex3d( -cameradim1-cameraWidth/2*sin(1.19f),  +cameraWidth/2*cos(1.19f), 0.0f);
			glVertex3d( -cameradim1-cameraWidth/2*sin(1.19f),  -cameraWidth/2*cos(1.19f), 0.0f);
			glVertex3d( -cameradim1, -cameraWidth/2, 0.0f);
		glEnd();

		// Camera side
		glBegin(GL_POLYGON);
			glNormal3d(0,1,0);
			glVertex3d( -cameradim1, +cameraWidth/2, 0.0f);
			glVertex3d( +cameradim1, +cameraWidth/2, 0.0f);
			glVertex3d( +cameradim1, +cameraWidth/2, -cameraWidth);
			glVertex3d( -cameradim1, +cameraWidth/2, -cameraWidth);
		glEnd();
		glBegin(GL_POLYGON);
			glNormal3d(0,-1,0);
			glVertex3d( -cameradim1, -cameraWidth/2, 0.0f);
			glVertex3d( +cameradim1, -cameraWidth/2, 0.0f);
			glVertex3d( +cameradim1, -cameraWidth/2, -cameraWidth);
			glVertex3d( -cameradim1, -cameraWidth/2, -cameraWidth);
		glEnd();

		glBegin(GL_POLYGON);
			glNormal3d(0,0,1);
			glVertex3d( +cameradim1, -cameraWidth/2, -cameraWidth);
			glVertex3d( +cameradim1, +cameraWidth/2, -cameraWidth);
			glVertex3d( -cameradim1, +cameraWidth/2, -cameraWidth);
			glVertex3d( -cameradim1-cameraWidth/2*sin(1.19f),  +cameraWidth/2*cos(1.19f), 0.0f);
			glVertex3d( -cameradim1-cameraWidth/2*sin(1.19f),  -cameraWidth/2*cos(1.19f), 0.0f);
			glVertex3d( -cameradim1, -cameraWidth/2, -cameraWidth);
		glEnd();

		glPopMatrix();

		glPopMatrix();
	glEndList();

	
	// Cone List
	Cone = X80Robot + 1;
	glNewList(Cone,GL_COMPILE);
		glPushMatrix();

		// Set Color
		glColor4f(1.0f,0.6f,0.0f,1.0f);

		// Base
		glBegin(GL_QUADS);
			glNormal3f(0,0,1);
			glVertex3f(-coneBaseRadius, +coneBaseRadius, 0.015f);
			glVertex3f(-coneBaseRadius, -coneBaseRadius, 0.015f);
			glVertex3f(+coneBaseRadius, -coneBaseRadius, 0.015f);
			glVertex3f(+coneBaseRadius, +coneBaseRadius, 0.015f);
		glEnd();

		// Body
		gluCylinder(quadratic,coneRadius,0,coneHeight,32,8);

		glPopMatrix();
	glEndList();



	// Particle List
	Particle = Cone + 1;
	glNewList(Particle,GL_COMPILE);
		glPushMatrix();

		// Set Color
		glColor4f(0.0f,0.0f,0.0f,1.0f);

		// Base
		glBegin(GL_POLYGON);
			glNormal3f(0,0,1);
			glVertex3f(-particleBaseRadius, +particleBaseRadius, 0.010f);
			glVertex3f(-particleBaseRadius, -particleBaseRadius, 0.010f);
			glVertex3f(2*particleBaseRadius, 0.0f, 0.010f);
		glEnd();
		
		// Set Color
		glColor4f(0.0f,1.0f,1.0f,1.0f);

		// Base
		glBegin(GL_POLYGON);
			glNormal3f(0,0,1);
			glVertex3f(-particleBaseRadius*pColorSize, +particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(-particleBaseRadius*pColorSize, -particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(2*particleBaseRadius*pColorSize, 0.0f, 0.015f);
		glEnd();

		glPopMatrix();
	glEndList();

	// X80RobotEstimate List
	X80RobotEstimate = Particle + 1;
	glNewList(X80RobotEstimate,GL_COMPILE);
		glPushMatrix();

		// Set Color
		glColor4f(0.0f,0.0f,0.0f,1.0f);

		// Base
		glBegin(GL_POLYGON);
			glNormal3f(0,0,1);
			glVertex3f(-estimateBaseRadius, +estimateBaseRadius, 0.010f);
			glVertex3f(-estimateBaseRadius, -estimateBaseRadius, 0.010f);
			glVertex3f(2*estimateBaseRadius, 0.0f, 0.010f);
		glEnd();
		
		// Set Color
		glColor4f(1.0f,0.0f,0.0f,1.0f);

		// Base
		glBegin(GL_POLYGON);
			glNormal3f(0,0,1);
			glVertex3f(-particleBaseRadius*pColorSize, +particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(-particleBaseRadius*pColorSize, -particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(2*particleBaseRadius*pColorSize, 0.0f, 0.015f);
		glEnd();

		glPopMatrix();
	glEndList();

	// PRMNode List
	PRMNode = X80RobotEstimate + 1;
	glNewList(PRMNode,GL_COMPILE);
		glPushMatrix();

		// Set Color
		glColor4f(1.0f,0.0f,0.0f,1.0f);

		// Base
		glBegin(GL_POLYGON);
			glVertex3f(-particleBaseRadius*pColorSize, +particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(-particleBaseRadius*pColorSize, -particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(+particleBaseRadius*pColorSize, -particleBaseRadius*pColorSize, 0.015f);
			glVertex3f(+particleBaseRadius*pColorSize, +particleBaseRadius*pColorSize, 0.015f);
		glEnd();

		glPopMatrix();
	glEndList();




	gluDeleteQuadric(quadratic);		
}
