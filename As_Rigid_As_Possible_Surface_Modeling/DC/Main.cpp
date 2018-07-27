
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <limits>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>
#include <ctime>
#include <cstring>
#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"
#include "float.h"
#include <armadillo>

/*
#include "Structure/Structure.h"
#include "Mesh/iterators.h"
#include "Conformal/HarmonicMapper/HarmonicMapperMesh.h" 
#include "Conformal/HarmonicMapper/HarmonicMapper.h"
*/
#include <time.h>

//#include <Eigen/Dense>
//#include <Eigen/Sparse>

#include "LeastSquaresSparseSolver.h"

#ifdef WIN32
	#define WIN32_LEAN_AND_MEAN
	#include <Windows.h>
#endif


using namespace arma;
using namespace std;
//using namespace MeshLib;



//using namespace Eigen;
// ----------------------------------------------------------------------------------------------------
// global variables






_GLMmodel *mesh;
_GLMmodel * mesh2;


bool test = false;
bool firstDeform = true;
bool Factorized2 = false;
bool converge = true;

float scale = 1.0;
int WindWidth, WindHeight;
int last_x, last_y;
int select_x, select_y;

bool FirstTime = true;

typedef enum { SELECT_MODE, DEFORM_MODE } ControlMode;
ControlMode current_mode = SELECT_MODE;

void Deform();
void Deform2();
void CalcNeighbor2();
void CalcCotangentWeight2();

bool DeformDone = true;
bool noPoint = false;
bool firstPutMap = true;

vector<float*> colors;
vector<vector<int> > handles;
int selected_handle_id = -1;
bool deform_mesh_flag = false;

bool flag = false;
bool Factorized = false;
bool UsingMultiRes = false;
bool testNaiveLaplacian = false;
bool continuous = true;
int edgeNum = 0;
int edgeNum2 = 0;

struct e{
	int v1 ;
	int v2 ;
	float weight ;
	float weight2 ;
	int o1 ;
	int o2 ;
};
typedef struct e edge;

float constraintWeight = 0.1;


vector3 pos[100000];
vector3 posPrime[100000];
vector3 pos2[100000];
vector3 posPrime2[100000];
matrix33 tempRot;
vector<int> neighbor[100000];
mat S[100000];
mat R[100000];
mat S2[100000];
mat R2[100000];

edge edges[100000];
edge edges2[100000];

int handleSize2 = 0;
int numNeighbor[100000];
int mapping[100000];
int inverseMapping[100000];
int numV , numV2;
int handleSize= 0;
int handleCount = 0;
vector3 deltaVec;
LeastSquaresSparseSolver solver;
int DeformCount = 0;

vector<int> mapHandles;
// ----------------------------------------------------------------------------------------------------
// render related functions

int winw = 800;
int winh = 600;


/*----------------------------------------------------------------------------------------
 *	Button Stuff
 */

/*
 *	We will define a function pointer type. ButtonCallback is a pointer to a function that
 *	looks a bit like this :
 *
 *	void func() { 
 *	}
 */
typedef void (*ButtonCallback)();

/*
 *	This is a simple structure that holds a button.
 */
struct Button 
{
	int   x;							/* top left x coord of the button */
	int   y;							/* top left y coord of the button */
	int   w;							/* the width of the button */
	int   h;							/* the height of the button */
	int	  state;						/* the state, 1 if pressed, 0 otherwise */
	int	  highlighted;					/* is the mouse cursor over the control? */
	char* label;						/* the text label of the button */
	ButtonCallback callbackFunction;	/* A pointer to a function to call if the button is pressed */
};
typedef struct Button Button;

struct Mouse 
{
	int x;		/*	the x coordinate of the mouse cursor	*/
	int y;		/*	the y coordinate of the mouse cursor	*/
	int lmb;	/*	is the left button pressed?		*/
	int mmb;	/*	is the middle button pressed?	*/
	int rmb;	/*	is the right button pressed?	*/

	/*
	 *	These two variables are a bit odd. Basically I have added these to help replicate
	 *	the way that most user interface systems work. When a button press occurs, if no
	 *	other button is held down then the co-ordinates of where that click occured are stored.
	 *	If other buttons are pressed when another button is pressed it will not update these
	 *	values. 
	 *
	 *	This allows us to "Set the Focus" to a specific portion of the screen. For example,
	 *	in maya, clicking the Alt+LMB in a view allows you to move the mouse about and alter
	 *	just that view. Essentually that viewport takes control of the mouse, therefore it is
	 *	useful to know where the first click occured.... 
	 */
	int xpress; /*	stores the x-coord of when the first button press occurred	*/
	int ypress; /*	stores the y-coord of when the first button press occurred	*/
};

/*
 *	rename the structure from "struct Mouse" to just "Mouse"
 */
typedef struct Mouse Mouse;

/*
 *	Create a global mouse structure to hold the mouse information.
 */

void SetMapping();
void LoadMesh(string s);
void LoadMesh2(string s);
void CallBack1();
void CallBack2();
void CallBack3();
void CallBack4();
void CallBack5();
void CallBack6();
void CallBack7();
void CallBack8();
void CallBack9();
void CallBack10();
void CallBack11();
void CallBack12();
void CalcCotangentWeight();
Mouse TheMouse = {0,0,0,0,0};
Button MyButton = {420,20, 165,30, 0,0, "Cotangent Weight", CallBack1 };
Button MyButton2 = {40,20, 165,30, 0,0, "Load Model", CallBack2 };
Button MyButton3 = {40,550, 165,30, 0,0, "Un-Select Handles", CallBack3 };
Button MyButton4 = {40,60, 165,30, 0,0, "Selection Mode", CallBack4 };
Button MyButton5 = {230,550,165,30,0,0,"Draw Handles",CallBack5};
Button MyButton6 = {230,20,165,30,0,0,"Reset Model",CallBack6};
Button MyButton7 = {420 ,550,165,30,0,0,"MultiRes Model",CallBack7};
Button MyButton8 = {610 ,20,165,30,0,0,"ARAP",CallBack8};
Button MyButton9 = { 610,550,165,30,0,0,"Save Model",CallBack9};
Button MyButton10 = {230,60,165,30,0,0,"Wait for Converge",CallBack10};
Button MyButton11 = {420,60,165,30,0,0,"Soft Constraint",CallBack11};
Button MyButton12 = {610,60,165,30,0,0,"Continuous Motion",CallBack12};
/*----------------------------------------------------------------------------------------
 *	This is an example callback function. Notice that it's type is the same
 *	an the ButtonCallback type. We can assign a pointer to this function which
 *	we can store and later call.
 */


bool IsCotangentWeight = true;
bool IsSelectHandle = true;
bool selectFlag = false;

string  GetFileName( const string & prompt ) { 
    const int BUFSIZE = 1024;
    char buffer[BUFSIZE] = {0};
    OPENFILENAME ofns = {0};
    ofns.lStructSize = sizeof( ofns );
	 wchar_t wtext[1000] ;
	wchar_t wwtext[1000];
	mbstowcs(wtext, buffer, strlen(buffer)+1);
    ofns.lpstrFile = wtext;
    ofns.nMaxFile = BUFSIZE;
	mbstowcs(wwtext,prompt.c_str(), strlen(prompt.c_str())+1);
    ofns.lpstrTitle = wwtext;
    GetOpenFileName( & ofns );
	wstring ws(wtext);
	string str(ws.begin(), ws.end());
    return str;
}

void CallBack1()
{
	if(IsCotangentWeight)
	MyButton.label = "Umbrella Weight";
	else if(!IsCotangentWeight)
	MyButton.label = "Cotangent Weight";

	IsCotangentWeight = ! IsCotangentWeight;
	Factorized = false;
	CalcCotangentWeight();
	
	if(UsingMultiRes){
		Factorized2 = false;
		CalcCotangentWeight2();
	}

}



void CallBack2(){
	//printf("qqqqq\n");
	string ss ,sss ;
	sss = GetFileName(ss);
	
	char check[200] = {0};
	cout<<sss<<endl;
	for(int i = sss.length()-1;i>=0;i--){
		check[sss.length() - i - 1] = sss[i];
		if(sss[i] =='.')
			break;
	}
	
	if(strcmp(check,"jbo.")){
		cout<<" You didn't select an obj file !!!"<<endl;
		return ;
	}

	edgeNum = 0;
	for(int i=0;i<handles.size();i++)
		handles[i].clear();
	handles.clear();
	handleCount = 0;
	handleSize = 0;
	Factorized = false;
	UsingMultiRes = false;
	selected_handle_id = -1;
	LoadMesh(sss);
}

void CallBack3(){
	for(int i=0;i<handles.size();i++)
		handles[i].clear();
	handles.clear();
	handleCount = 0;
	handleSize = 0;
	selected_handle_id = -1;
	Factorized = false;
}

void CallBack4(){
	if(current_mode==SELECT_MODE){
		current_mode = DEFORM_MODE;
		MyButton4.label = "Deform Mode";
	}
	else if(current_mode==DEFORM_MODE){
		current_mode = SELECT_MODE;
		MyButton4.label = "Selection Mode";
		selected_handle_id = -1;
	}
}
void CallBack5(){
	if(noPoint)
		MyButton5.label = "Draw Handles";
	else if(!noPoint)
		MyButton5.label = "No Handles";
	noPoint = !noPoint;
}
void CallBack6(){

	for(int i=1;i<=numV;i++){
		mesh->vertices[ i * 3 + 0] = pos[i].x;
		mesh->vertices[ i * 3 + 1] = pos[i].y;
		mesh->vertices[ i * 3 + 2] = pos[i].z;
		posPrime[i] = pos[i];
	}
	if(UsingMultiRes){
		
		for(int i=1;i<=numV2;i++){
			mesh2->vertices[ i * 3 + 0] = pos2[i].x;
			mesh2->vertices[ i * 3 + 1] = pos2[i].y;
			mesh2->vertices[ i * 3 + 2] = pos2[i].z;
			posPrime2[i] = pos2[i];
		}
		UsingMultiRes = false;
		Factorized2 = false;
	}

}

void CallBack7(){
	if(!UsingMultiRes){
		string ss ,sss ;
		sss = GetFileName(ss);	
		char check[200] = {0};
		cout<<sss<<endl;
		for(int i = sss.length()-1;i>=0;i--){
			check[sss.length() - i - 1] = sss[i];
			if(sss[i] =='.')
				break;
		}
	
		if(strcmp(check,"jbo.")){
			cout<<" You didn't select an obj file !!!"<<endl;
			return ;
		}

		edgeNum2 = 0;
	
		LoadMesh2(sss);
		SetMapping();

		UsingMultiRes = true;
		MyButton7.label = "Cancel MultiResolution";
	}
	else{
		UsingMultiRes = false;
		MyButton7.label = "MultiRes Model";
	}

}

void CallBack8(){

	if(!testNaiveLaplacian){
		testNaiveLaplacian = true;
		MyButton8.label = "Naive Laplacian";
	}
	else{
		testNaiveLaplacian = false;
		MyButton8.label = "ARAP";
	}
}

void CallBack9(){
	glmWriteOBJ(mesh,"output.obj",0);

}
void CallBack10(){
	if(converge){
		MyButton10.label = "No Converge";
	}
	else{
		MyButton10.label = "Wait for Converge";
	}
	converge = !converge;
}

void CallBack11(){
	if(!test){
		MyButton11.label = "Hard Constraint";
	}
	else{
		MyButton11.label = " Soft Constraint";
	}
	test = !test;
}


void CallBack12(){
	if(continuous){
		MyButton12.label = "Non-Continuous";
	}
	else{
		MyButton12.label = "Continuous";
	}
	continuous = !continuous;
}

void SetMapping(){

	for(int i=0;i<30000;i++){
		mapping[i] = 0;
		inverseMapping[i] = 0;
	}

	for(int i=1;i<=mesh->numvertices;i++){
		float minDist = 999999;
		float dist;
		
		for(int j=1;j<=mesh2->numvertices;j++){
			dist = (pos[i]-pos2[j]).length();
			//dist = sqrt(dist);
			if(dist<minDist){
				minDist = dist;
				mapping[i] = j;
				inverseMapping[j] = i;
			}	
		}

	}

}


/*----------------------------------------------------------------------------------------
 *	This is the button visible in the viewport. This is a shorthand way of 
 *	initialising the structure's data members. Notice that the last data
 *	member is a pointer to the above function. 
 */


/*----------------------------------------------------------------------------------------
 *	\brief	This function draws a text string to the screen using glut bitmap fonts.
 *	\param	font	-	the font to use. it can be one of the following : 
 *
 *					GLUT_BITMAP_9_BY_15		
 *					GLUT_BITMAP_8_BY_13			
 *					GLUT_BITMAP_TIMES_ROMAN_10	
 *					GLUT_BITMAP_TIMES_ROMAN_24	
 *					GLUT_BITMAP_HELVETICA_10	
 *					GLUT_BITMAP_HELVETICA_12	
 *					GLUT_BITMAP_HELVETICA_18	
 *
 *	\param	text	-	the text string to output
 *	\param	x		-	the x co-ordinate
 *	\param	y		-	the y co-ordinate
 */
void Font(void *font,char *text,int x,int y)
{
	glRasterPos2i(x, y);

	while( *text != '\0' )
	{
		glutBitmapCharacter( font, *text );
		++text;
	}
}


/*----------------------------------------------------------------------------------------
 *	\brief	This function is used to see if a mouse click or event is within a button 
 *			client area.
 *	\param	b	-	a pointer to the button to test
 *	\param	x	-	the x coord to test
 *	\param	y	-	the y-coord to test
 */
int ButtonClickTest(Button* b,int x,int y) 
{
	if( b) 
	{
		/*
		 *	If clicked within button area, then return true
		 */
	    if( x > b->x      && 
			x < b->x+b->w &&
			y > b->y      && 
			y < b->y+b->h ) {
				return 1;
		}
	}

	/*
	 *	otherwise false.
	 */
	return 0;
}

/*----------------------------------------------------------------------------------------
 *	\brief	This function draws the specified button.
 *	\param	b	-	a pointer to the button to check.
 *	\param	x	-	the x location of the mouse cursor.
 *	\param	y	-	the y location of the mouse cursor.
 */
void ButtonRelease(Button *b,int x,int y)
{
	if(b) 
	{
		/*
		 *	If the mouse button was pressed within the button area
		 *	as well as being released on the button.....
		 */
		
		if( ButtonClickTest(b,TheMouse.xpress,TheMouse.ypress) && 
			ButtonClickTest(b,x,y) )
		{
			/*
			 *	Then if a callback function has been set, call it.
			 */
				//printf("aaa %s\n",b->label);
			if (b->callbackFunction) {
				//printf("%s\n",b->label);
				b->callbackFunction();
			}
		}

		/*
		 *	Set state back to zero.
		 */
		b->state = 0;
	}
}

/*----------------------------------------------------------------------------------------
 *	\brief	This function draws the specified button.
 *	\param	b	-	a pointer to the button to check.
 *	\param	x	-	the x location of the mouse cursor.
 *	\param	y	-	the y location of the mouse cursor.
 */
void ButtonPress(Button *b,int x,int y)
{
	if(b)
	{
		/*
		 *	if the mouse click was within the buttons client area, 
		 *	set the state to true.
		 */
		if( ButtonClickTest(b,x,y) )
		{
			b->state = 1;
			//printf("bbb %s\n",b->label);
		}
	}
}


/*----------------------------------------------------------------------------------------
 *	\brief	This function draws the specified button.
 *	\param	b	-	a pointer to the button to check.
 *	\param	x	-	the x location of the mouse cursor.
 *	\param	y	-	the y location of the mouse cursor.
 */
void ButtonPassive(Button *b,int x,int y)
{
	if(b)
	{
		/*
		 *	if the mouse moved over the control
		 */
		if( ButtonClickTest(b,x,y) )
		{
			/*
			 *	If the cursor has just arrived over the control, set the highlighted flag
			 *	and force a redraw. The screen will not be redrawn again until the mouse
			 *	is no longer over this control
			 */
			if( b->highlighted == 0 ) {
				b->highlighted = 1;
				glutPostRedisplay();
			}
		}
		else

		/*
		 *	If the cursor is no longer over the control, then if the control
		 *	is highlighted (ie, the mouse has JUST moved off the control) then
		 *	we set the highlighting back to false, and force a redraw. 
		 */
		if( b->highlighted == 1 )
		{
			b->highlighted = 0;
			glutPostRedisplay();
		}
	}
}

/*----------------------------------------------------------------------------------------
 *	\brief	This function draws the specified button.
 *	\param	b	-	a pointer to the button to draw.
 */
void ButtonDraw(Button *b)
{
	int fontx;
	int fonty;

	if(b)
	{
		/*
		 *	We will indicate that the mouse cursor is over the button by changing its
		 *	colour.
		 */
		if (b->highlighted) 
			glColor3f(0.7f,0.7f,0.8f);
		else 
			glColor3f(0.6f,0.6f,0.6f);

		/*
		 *	draw background for the button.
		 */
		glBegin(GL_QUADS);
			glVertex2i( b->x     , b->y      );
			glVertex2i( b->x     , b->y+b->h );
			glVertex2i( b->x+b->w, b->y+b->h );
			glVertex2i( b->x+b->w, b->y      );
		glEnd();

		/*
		 *	Draw an outline around the button with width 3
		 */
		glLineWidth(3);

		/*
		 *	The colours for the outline are reversed when the button. 
		 */
		if (b->state) 
			glColor3f(0.4f,0.4f,0.4f);
		else 
			glColor3f(0.8f,0.8f,0.8f);

		glBegin(GL_LINE_STRIP);
			glVertex2i( b->x+b->w, b->y      );
			glVertex2i( b->x     , b->y      );
			glVertex2i( b->x     , b->y+b->h );
		glEnd();

		if (b->state) 
			glColor3f(0.8f,0.8f,0.8f);
		else 
			glColor3f(0.4f,0.4f,0.4f);

		glBegin(GL_LINE_STRIP);
			glVertex2i( b->x     , b->y+b->h );
			glVertex2i( b->x+b->w, b->y+b->h );
			glVertex2i( b->x+b->w, b->y      );
		glEnd();

		glLineWidth(1);


		/*
		 *	Calculate the x and y coords for the text string in order to center it.
		 */
		
		const unsigned char* t = reinterpret_cast<const unsigned char *>( b->label);
		fontx = b->x + (b->w - glutBitmapLength(GLUT_BITMAP_HELVETICA_10,t)) / 2 -40;
		fonty = b->y + (b->h+10)/2;

		/*
		 *	if the button is pressed, make it look as though the string has been pushed
		 *	down. It's just a visual thing to help with the overall look....
		 */
		if (b->state) {
			fontx+=2;
			fonty+=2;
		}

		/*
		 *	If the cursor is currently over the button we offset the text string and draw a shadow
		 */
		if(b->highlighted)
		{
			glColor3f(0,0,0);
			Font(GLUT_BITMAP_HELVETICA_18,b->label,fontx,fonty);
			fontx--;
			fonty--;
		}

		glColor3f(1,1,1);
		Font(GLUT_BITMAP_HELVETICA_18,b->label,fontx,fonty);
	}
}
void Draw2D()
{
	ButtonDraw(&MyButton);
	ButtonDraw(&MyButton2);
	ButtonDraw(&MyButton3);
	ButtonDraw(&MyButton4);
	ButtonDraw(&MyButton5);
	ButtonDraw(&MyButton6);
	ButtonDraw(&MyButton7);
	ButtonDraw(&MyButton8);
	ButtonDraw(&MyButton9);
	ButtonDraw(&MyButton10);
	ButtonDraw(&MyButton11);
	ButtonDraw(&MyButton12);

}



void Reshape(int width, int height)
{
	int base = min(width, height);

	tbReshape(width, height);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)width / (GLdouble)height, 1.0, 128.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -3.5);

	WindWidth = width;
	WindHeight = height;
}


vector2 linePos;

void Display(void)
{
	
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	

	
	glPushMatrix();

	tbMatrix();

	// render solid model


	glColor3f(1.0, 1.0, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glmDraw(mesh, GLM_SMOOTH);

	// render wire model
	
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glLineWidth(1.0f);
	glColor3f(0.6, 0.0, 0.8);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glmDraw(mesh, GLM_SMOOTH);
	
	// render handle points
	glPointSize(10.0);
	glEnable(GL_POINT_SMOOTH);
	glDisable(GL_LIGHTING);

	if (!noPoint){
		glBegin(GL_POINTS);
		for (int handleIter = 0; handleIter < handles.size(); handleIter++)
		{
			glColor3fv(colors[handleIter%colors.size()]);
			for (int vertIter = 0; vertIter < handles[handleIter].size(); vertIter++)
			{
				int idx = handles[handleIter][vertIter];
				glVertex3fv((float *)&mesh->vertices[3 * idx]);
			}
		}
		glEnd();
	}
	glPopMatrix();

	//glutSolidTeapot(1);

	glDisable(GL_DEPTH_TEST);
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0,winw,winh,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	Draw2D();
	if(selectFlag){
		//cout<<"YOOOO  "<<select_x<< " "<<select_y<<" "<<linePos.x<<"  "<<linePos.y<<endl;
		glLineWidth(2.0f);
		glBegin(GL_LINES);
		glColor3fv(colors[handleCount%colors.size()]);
		glVertex2i(select_x,select_y);			
		glVertex2i(select_x,linePos.y);

		glVertex2i(select_x,linePos.y);
		glVertex2i(linePos.x,linePos.y);

		glVertex2i(linePos.x,linePos.y);
		glVertex2i(linePos.x,select_y);

		glVertex2i(linePos.x,select_y);
		glVertex2i(select_x,select_y);	
		glEnd();
	}
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	
	glFlush();
	
	glutSwapBuffers();

	//DeformDone = true;
	
}

// ----------------------------------------------------------------------------------------------------
// mouse related functions

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x, viewport[3] - (int)_2Dpos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = { 0.0, 0.0, 0.0 };

	gluUnProject(X, ((double)viewport[3] - Y), (double)Depth, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

	return vector3(wpos[0], wpos[1], wpos[2]);
}

vector2 projection_helper(vector3 _3Dpos)
{
	int viewport[4];
	double ModelViewMatrix[16];    // Model_view matrix
	double ProjectionMatrix[16];   // Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	double wpos[3] = { 0.0, 0.0, 0.0 };
	gluProject(_3Dpos.x, _3Dpos.y, _3Dpos.z, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

	return vector2(wpos[0], (double)viewport[3] - wpos[1]);
}

void SetSolver(){


}

void mouse(int button, int state, int x, int y)
{


	tbMouse(button, state, x, y);
	
	//cout<<"mouse!"<<endl;
	if (current_mode == SELECT_MODE && button == GLUT_RIGHT_BUTTON)
	{
		
		if (state == GLUT_DOWN)
		{
			select_x = x;
			select_y = y;
			selectFlag = true;
			linePos.set(x,y);
		}
		else
		{
			vector<int> this_handle;
			
			//cout<<"here"<<endl;
			// project all mesh vertices to current viewport
			for (int vertIter = 1; vertIter<=mesh->numvertices; vertIter++)
			{
				vector3 pt(mesh->vertices[3 * vertIter + 0], mesh->vertices[3 * vertIter + 1], mesh->vertices[3 * vertIter + 2]);
				vector2 pos = projection_helper(pt);
				

				// if the projection is inside the box specified by mouse click&drag, add it to current handle
				if ((pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y)||(pos.x<=select_x&&pos.y<=select_y&&pos.x>=x&&pos.y>=y)
					|| (pos.x >= select_x && pos.y <= select_y && pos.x <= x && pos.y >= y) || (pos.x <= select_x && pos.y >= select_y && pos.x >= x && pos.y <= y))
				{
					this_handle.push_back(vertIter);
				}
			}
			if(!this_handle.empty()){
				handles.push_back(this_handle);
				handleCount++;
				handleSize += this_handle.size();
				Factorized = false;
			}

		
		}
	}
	// select handle
	else if (current_mode == DEFORM_MODE && button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN && handleCount!=0)
	{
		// project all handle vertices to current viewport
		// see which is closest to selection point
		
	//	flag = false;

		deltaVec.x = 0;
		deltaVec.y = 0;
		deltaVec.z = 0;
		//cout << "aaa" << endl;
		double min_dist = 999999;
		int handle_id = -1;
		for (int handleIter = 0; handleIter<handles.size(); handleIter++)
		{
			for (int vertIter = 0; vertIter<handles[handleIter].size(); vertIter++)
			{
				int idx = handles[handleIter][vertIter];
				vector3 pt(mesh->vertices[3 * idx + 0], mesh->vertices[3 * idx + 1], mesh->vertices[3 * idx + 2]);
				vector2 pos = projection_helper(pt);

				double this_dist = sqrt((double)(pos.x - x)*(pos.x - x) + (double)(pos.y - y)*(pos.y - y));
				if (this_dist<min_dist)
				{
					min_dist = this_dist;
					handle_id = handleIter;
				}
			}
		}

		selected_handle_id = handle_id;
		//handleSize = handles[selected_handle_id].size();
		deform_mesh_flag = true;
		firstDeform = true;
		DeformCount = 0;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP && selectFlag){
		selectFlag = false;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP && deform_mesh_flag){
		deform_mesh_flag = false;
			if(!continuous){
			clock_t t1,t2;
			t1 = clock();
			deltaVec *= 5.0;
			Deform();
			t2 = clock();
			printf("%lf\n", (t2-t1)/(double)(CLOCKS_PER_SEC));
			deltaVec.set(0,0,0);
		}
	}

	last_x = x;
	last_y = y;

	TheMouse.x = x;
	TheMouse.y = y;

	/*
	 *	has the button been pressed or released?
	 */
	if (state == GLUT_DOWN) 
	{
		/*
		 *	This holds the location of the first mouse click
		 */
		//if ( !(TheMouse.lmb || TheMouse.mmb || TheMouse.rmb) ) {
			TheMouse.xpress = x;
			TheMouse.ypress = y;
			//printf("NOOO\n");
		//}

		/*
		 *	Which button was pressed?
		 */
		switch(button) 
		{
		case GLUT_LEFT_BUTTON:
			TheMouse.lmb = 1;
			ButtonPress(&MyButton,x,y);
			ButtonPress(&MyButton2,x,y);
			ButtonPress(&MyButton3,x,y);
			ButtonPress(&MyButton4,x,y);
			ButtonPress(&MyButton5,x,y);
			ButtonPress(&MyButton6,x,y);
			ButtonPress(&MyButton7,x,y);
			ButtonPress(&MyButton8,x,y);
			ButtonPress(&MyButton9,x,y);
			ButtonPress(&MyButton10,x,y);
			ButtonPress(&MyButton11,x,y);
			ButtonPress(&MyButton12,x,y);
		case GLUT_MIDDLE_BUTTON:
			TheMouse.mmb = 1;
			break;
		case GLUT_RIGHT_BUTTON:
			TheMouse.rmb = 1;
			break;
		}
	}
	else 
	{
		/*
		 *	Which button was released?
		 */
		switch(button) 
		{
		case GLUT_LEFT_BUTTON:
			//printf("YYYY\n");
			TheMouse.lmb = 0;
			ButtonRelease(&MyButton,x,y);
			ButtonRelease(&MyButton2,x,y);
			ButtonRelease(&MyButton3,x,y);
			ButtonRelease(&MyButton4,x,y);
			ButtonRelease(&MyButton5,x,y);
			ButtonRelease(&MyButton6,x,y);
			ButtonRelease(&MyButton7,x,y);
			ButtonRelease(&MyButton8,x,y);
			ButtonRelease(&MyButton9,x,y);
			ButtonRelease(&MyButton10,x,y);
			ButtonRelease(&MyButton11,x,y);
			ButtonRelease(&MyButton12,x,y);
			break;
		case GLUT_MIDDLE_BUTTON:
			TheMouse.mmb = 0;
			break;
		case GLUT_RIGHT_BUTTON:
			TheMouse.rmb = 0;
			break;
		}
	}

	/*
	 *	Force a redraw of the screen. If we later want interactions with the mouse
	 *	and the 3D scene, we will need to redraw the changes.
	 */
	glutPostRedisplay();



}



void CalcNeighbor(){

	memset(numNeighbor, 0, sizeof(numNeighbor));

	numV = mesh->numvertices;

	int a, b, c;

	for (int i = 0; i < mesh->numtriangles; i++){
		a = mesh->triangles[i].vindices[0];
		b = mesh->triangles[i].vindices[1];
		c = mesh->triangles[i].vindices[2];
		if (a <= 0 || a > numV ||b<=0 || b>numV || c<=0 ||c>numV)
			cout << a << " " << b << " " << c << endl;

		if (std::find(neighbor[a].begin(), neighbor[a].end(), b) == neighbor[a].end()){
			neighbor[a].push_back(b);
			neighbor[b].push_back(a);
			numNeighbor[a]++;
			numNeighbor[b]++;
		}
		if (std::find(neighbor[a].begin(), neighbor[a].end(), c) == neighbor[a].end()){
			neighbor[a].push_back(c);
			neighbor[c].push_back(a);
			numNeighbor[a]++;
			numNeighbor[c]++;
		}
		if (std::find(neighbor[b].begin(), neighbor[b].end(), c) == neighbor[b].end()){
			neighbor[b].push_back(c);
			neighbor[c].push_back(b);
			numNeighbor[b]++;
			numNeighbor[c]++;
		}

		bool check = false;
		for (int j = 0; j < edgeNum; j++){
			if ((edges[j].v1 == a&&edges[j].v2 == b) || (edges[j].v1 == b&&edges[j].v2 == a)){
				check = true;
				if (edges[j].o1 == 0)
					edges[j].o1 = c;
				else {
					edges[j].o2 = c;

				}
			}
		}
		if (!check){
		
			edges[edgeNum].v1 = a;
			edges[edgeNum].v2 = b;
			edges[edgeNum].o1 = c;
			edges[edgeNum].o2 = 0;

			if (edges[edgeNum].v1 > numV || edges[edgeNum].v2 > numV)
				cout << "@@@ " << edgeNum << endl;
			edgeNum++;
		}
	
		check = false;

		for (int j = 0; j < edgeNum; j++){
			if ((edges[j].v1 == a&&edges[j].v2 == c) || (edges[j].v1 == c&&edges[j].v2 == a)){
				check = true;
				if (edges[j].o1 == 0)
					edges[j].o1 = b;
				else {
					edges[j].o2 = b;
				}
			}
		}
		if (!check){
		
			edges[edgeNum].v1 = a;
			edges[edgeNum].v2 = c;
			edges[edgeNum].o1 = b;
			edges[edgeNum].o2 = 0;

			if (edges[edgeNum].v1 > numV || edges[edgeNum].v2 > numV)
				cout << "@@@ " << edgeNum << endl;
			edgeNum++;

		}

		check = false;

		for (int j = 0; j < edgeNum; j++){
			if ((edges[j].v1 == b&&edges[j].v2 == c) || (edges[j].v1 == c&&edges[j].v2 == b)){
				check = true;
				if (edges[j].o1 == 0)
					edges[j].o1 = a;
				else{
					edges[j].o2 = a;
				}
			}
		}
		if (!check){
		
			edges[edgeNum].v1 = b;
			edges[edgeNum].v2 = c;
			edges[edgeNum].o1 = a;
			edges[edgeNum].o2 = 0;
			if (edges[edgeNum].v1 > numV || edges[edgeNum].v2 > numV)
				cout << "@@@ " << edgeNum << endl;

			edgeNum++;
			
		}

		

	}

	

}






void CalcCotangentWeight(){
	srand(time(NULL));

	for (int i = 0; i < edgeNum; i++){

		vector3 t1, t2;
		vector3 a1, a2, b1, b2;
		if (edges[i].o2 != 0){

			if(IsCotangentWeight){
				t1.set(pos[edges[i].o1].x, pos[edges[i].o1].y, pos[edges[i].o1].z);
				t2.set(pos[edges[i].o2].x, pos[edges[i].o2].y, pos[edges[i].o2].z);
				a1 = (pos[edges[i].v1] - t1);
				a2 = (pos[edges[i].v2] - t1);
				b1 = (pos[edges[i].v1] - t2);
				b2 = (pos[edges[i].v2] - t2);
				float aaR = acos(DotProduct(a1, a2)/(a1.length()*a2.length()));
				float bbR = acos(DotProduct(b1, b2) / (b1.length()*b2.length()));
				edges[i].weight = 0.5f * ((1.0 / tan(aaR)) + (1.0 / tan(bbR)));
				if(edges[i].weight<0) edges[i].weight = 0;
			}
			else{
				edges[i].weight = 1.0;
			}
			//cout << edges[i].weight << endl;
		
		}
		else{
			if(IsCotangentWeight){
				t1.set(pos[edges[i].o1].x, pos[edges[i].o1].y, pos[edges[i].o1].z);
				a1 = pos[edges[i].v1] - t1;
				a2 = pos[edges[i].v2] - t1;
				float aR = acos(DotProduct(a1, a2) / (a1.length()*a2.length()));
				edges[i].weight = 0.5 * (1.0 / tan(aR) );
				if(edges[i].weight<0)edges[i].weight = 0;
			}
			else{
			edges[i].weight = 1.0;
			}
			//cout << "BOUNDARY " << i << " " << edges[i].v1 << " " << edges[i].v2 << "   " << edges[i].weight << endl;
		}
		
		//if(edges[i].weight>=10)cout<<"edge"<<i<<"  "<<edges[i].weight<<endl;

	}


}

float CalcEnergy(int which){
	
	float energy = 0;
	if(which ==1){
		for(int k=0;k<edgeNum;k++){
			int i,j;
			i = edges[k].v1;
			j = edges[k].v2;
			float ww = edges[k].weight;
			vec Pi(3); Pi(0) = pos[i].x; Pi(1) = pos[i].y; Pi(2) = pos[i].z;
			vec Primei(3); Primei(0) = posPrime[i].x; Primei(1) = posPrime[i].y; Primei(2) = posPrime[i].z;
			vec Pj(3); Pj(0) = pos[j].x; Pj(1) = pos[j].y; Pj(2) = pos[j].z;
			vec Primej(3); Primej(0) = posPrime[j].x; Primej(1) = posPrime[j].y; Primej(2) = posPrime[j].z;
		
			 energy  += ww* pow(norm(((Primei - Primej) - (R[i]*(Pi-Pj))),2),2);
			 energy  += ww* pow(norm(((Primej - Primei) - (R[j]*(Pj-Pi))),2),2);

		}
	}
	else if(which==2){
		for(int k=0;k<edgeNum;k++){
			int i,j;
			i = edges2[k].v1;
			j = edges2[k].v2;
			float ww = edges2[k].weight;
			vec Pi(3); Pi(0) = pos2[i].x; Pi(1) = pos2[i].y; Pi(2) = pos2[i].z;
			vec Primei(3); Primei(0) = posPrime2[i].x; Primei(1) = posPrime2[i].y; Primei(2) = posPrime2[i].z;
			vec Pj(3); Pj(0) = pos2[j].x; Pj(1) = pos2[j].y; Pj(2) = pos2[j].z;
			vec Primej(3); Primej(0) = posPrime2[j].x; Primej(1) = posPrime2[j].y; Primej(2) = posPrime2[j].z;
		
			 energy  += ww* pow(norm(((Primei - Primej) - (R2[i]*(Pi-Pj))),2),2);
			 energy  += ww* pow(norm(((Primej - Primei) - (R2[j]*(Pj-Pi))),2),2);

		}

	}
		cout<<"Energy: "<<energy<<endl;
		return energy;

}

bool handleTable[30000];
bool CFLAG = true;
bool testSpeed = false;


void Deform(){
	float energy1,energy2;
	energy1 = 999999;

	for (int i = 0; i <= numV; i++)
		handleTable[i] = false;


		for (int i = 0; i < handleCount; i++){

			for (int j = 0; j < handles[i].size(); j++){
				int t = handles[i][j];
				handleTable[t] = true;
			}

		}  
	

	int numIter , iterCount ;
	iterCount = 0;
	if(converge){
		numIter = 500;
	}
	else{
		numIter = 6;
	}
	
	for (int vertIter = 0; vertIter < handles[selected_handle_id].size(); vertIter++)
	{
		int idx = handles[selected_handle_id][vertIter];
		vector3 pt(mesh->vertices[idx*3+0]+deltaVec.x,mesh->vertices[idx*3+1]+deltaVec.y,mesh->vertices[idx*3+2]+deltaVec.z);
		posPrime[idx].x = pt[0];
		posPrime[idx].y = pt[1];
		posPrime[idx].z = pt[2];
	}


	float** b = new float*[3];
	if (!test){


		b[0] = new float[numV + handleSize];
		b[1] = new float[numV + handleSize];
		b[2] = new float[numV + handleSize];

	}
	else{

		b[0] = new float[numV];
		b[1] = new float[numV];
		b[2] = new float[numV];
	}


	//LeastSquaresSparseSolver solver;


	if (!Factorized){

		if (!test){
			solver.Create(numV + handleSize, numV, 3);
		}
		else{
			solver.Create(numV, numV, 3);
			//solver.ResetSolver(numV, numV, 3);
		}


		float www[30000];
		for (int k = 0; k < 30000; k++){
			www[k] = 0.0;
		}




		if (!test){

			for (int k = 0; k < edgeNum; k++){

				int i = edges[k].v1;
				int j = edges[k].v2;
				float ww = edges[k].weight;   // 這邊超慢的啦~~~
				if (i > numV || i<1 || j>numV || j < 1)
					cout << "QQ: " << i << " " << j << " " << ww << endl;
				solver.AddSysElement(i - 1, j - 1, (-1.0 * ww));
				solver.AddSysElement(j - 1, i - 1, (-1.0 * ww));
				//solver.AddSysElement(i - 1, j - 1, -1.0 / numNeighbor[i]);
				//solver.AddSysElement(j - 1, i - 1, -1.0 / numNeighbor[j]);
				www[i] += ww;
				www[j] += ww;

			}
		}
		else{
			for (int k = 0; k < edgeNum; k++){

				int i = edges[k].v1;
				int j = edges[k].v2;
				float ww = edges[k].weight;   // 這邊超慢的啦~~~
				if (i > numV || i<1 || j>numV || j < 1)
					cout << "QQ: " << i << " " << j << " " << ww << endl;

				if (!handleTable[i]){
					solver.AddSysElement(i - 1, j - 1, (-ww));
					www[i] += ww;
				}
				if (!handleTable[j]){
					solver.AddSysElement(j - 1, i - 1, (-ww));
					www[j] += ww;
				}
				//solver.AddSysElement(i - 1, j - 1, -1.0 / numNeighbor[i]);
				//solver.AddSysElement(j - 1, i - 1, -1.0 / numNeighbor[j]);


			}

		}




		if (!test){
			for (int k = 0; k < numV; k++){
				//cout << "www " << k << " " << www[k] << endl;
				solver.AddSysElement(k, k, www[k + 1]);

			}
		}
		else{
			for (int k = 0; k < numV; k++){
				if (!handleTable[k + 1])
					solver.AddSysElement(k, k, www[k + 1]);
				else{
					solver.AddSysElement(k, k, 1.0);
				}
			}
		}

		int count = numV;
		if(!test){
			for (int i = 0; i < handleCount; i++){
					for (int j = 0; j < handles[i].size(); j++){
						int v = handles[i][j];
						solver.AddSysElement(count,v-1,1.0);
						count++;
					}
			}
		}

	}

	//Factorized = false;
	//system("pause");

	while (iterCount++ < numIter){

		if(!UsingMultiRes){
		
			for (int i = 0; i <= numV; i++){
				S[i].zeros();
				R[i].zeros();
			}
		


			for (int k = 0; k < edgeNum; k++){
				int i = edges[k].v1;
				int j = edges[k].v2;
				if (i ==0|| j==0)
					cout <<"000000 "<< i << " " << j << endl;
				vector3 E = pos[i] - pos[j];
				vector3 EPrime = posPrime[i] - posPrime[j];
				float wij = edges[k].weight;

				//wij = -1.0 / numNeighbor[i];

				S[i](0, 0) += wij * E.x * EPrime.x;
				S[i](0, 1) += wij * E.x * EPrime.y;
				S[i](0, 2) += wij * E.x * EPrime.z;
				S[i](1, 0) += wij * E.y * EPrime.x;
				S[i](1, 1) += wij * E.y * EPrime.y;
				S[i](1, 2) += wij * E.y * EPrime.z;
				S[i](2, 0) += wij * E.z * EPrime.x;
				S[i](2, 1) += wij * E.z * EPrime.y;
				S[i](2, 2) += wij * E.z * EPrime.z;
				E = pos[j] - pos[i];
				EPrime = posPrime[j] - posPrime[i];

				//wij = -1.0 / numNeighbor[j];

				S[j](0, 0) += wij * E.x * EPrime.x;
				S[j](0, 1) += wij * E.x * EPrime.y;
				S[j](0, 2) += wij * E.x * EPrime.z;
				S[j](1, 0) += wij * E.y * EPrime.x;
				S[j](1, 1) += wij * E.y * EPrime.y;
				S[j](1, 2) += wij * E.y * EPrime.z;
				S[j](2, 0) += wij * E.z * EPrime.x;
				S[j](2, 1) += wij * E.z * EPrime.y;
				S[j](2, 2) += wij * E.z * EPrime.z;
			
			
			
			}


			int QQ = 0;
			for (int i = 0; i <= numV; i++){

			
				mat U, V;
				vec ss;
				svd(U, ss, V, S[i]);

				mat I = eye<mat>(3,3);
				float determination = det(V*U.t());
				I(2, 2) = determination;
				if (abs(determination - (-1.0)) < 0.001)
					QQ++;
			
				if (iterCount > 1)
				R[i] = V * I* U.t();
				else
				R[i].eye();
				//R[i].print();
				if(testNaiveLaplacian)	
				R[i].eye();
			}
		}
		else{
			if(iterCount==0){
				for(int i=1;i<=numV;i++)
					R[i].eye();
			}
			else{
				Deform2();
				for(int i=1;i<=numV;i++){
					if(!testNaiveLaplacian){
						R[i] = R2[mapping[i]];
						//cout<<i<<"  "<<mapping[i]<<endl;
					}
					else{
						R[i].eye();
					}
				}
			}

			

		}
		
	

		if (!test){
			for (int i = 0; i < numV + handleSize; i++){
				b[0][i] = 0;
				b[1][i] = 0;
				b[2][i] = 0;
			}
		}
		else{
			for (int i = 0; i < numV ; i++){
				b[0][i] = 0;
				b[1][i] = 0;
				b[2][i] = 0;
			}
		}

		for (int k = 0; k < edgeNum; k++){

			int i = edges[k].v1;
			int j = edges[k].v2;
			float ww = edges[k].weight;   // 這邊超慢的啦~~~
			vec pi(3); pi(0) = pos[i].x; pi(1) = pos[i].y; pi(2) = pos[i].z;
			vec pj(3); pj(0) = pos[j].x; pj(1) = pos[j].y; pj(2) = pos[j].z;


			//ww = -1.0 / numNeighbor[i];
			
			vec temp =(R[i] + R[j]) * (pi - pj);
			
			//ww = -1.0 / numNeighbor[j];

			vec temp2 = (R[j] + R[i]) * (pj - pi);
			b[0][i - 1] += (ww / 2.0) * temp(0);
			b[1][i - 1] += (ww / 2.0) * temp(1);
			b[2][i - 1] += (ww / 2.0) * temp(2);
			b[0][j - 1] += (ww / 2.0) * temp2(0);
			b[1][j - 1] += (ww / 2.0) * temp2(1);
			b[2][j - 1] += (ww / 2.0) * temp2(2);

		}

	
		if (!test){
			int count;
			count = numV;
			for (int i = 0; i < handleCount; i++){
				for (int j = 0; j < handles[i].size(); j++){
					int v = handles[i][j];

					b[0][count] = posPrime[v].x;
					b[1][count] = posPrime[v].y;
					b[2][count] = posPrime[v].z;
					count++;
				}
			}
		}
		else{

			for (int i = 0; i < handleCount; i++){
				for (int j = 0; j < handles[i].size(); j++){
					int v = handles[i][j];

					b[0][v-1] = posPrime[v].x;
					b[1][v-1] = posPrime[v].y;
					b[2][v-1] = posPrime[v].z;
				}
			}

		}
	



		solver.SetRightHandSideMatrix(b);
		
		if (!Factorized){
			solver.CholoskyFactorization();
			Factorized = true;
		}
		
		solver.CholoskySolve(0);
		solver.CholoskySolve(1);
		solver.CholoskySolve(2);
	
		for (int i = 1; i <= numV; i++){
			posPrime[i].x = solver.GetSolution(0, i - 1);
			posPrime[i].y = solver.GetSolution(1, i - 1);
			posPrime[i].z = solver.GetSolution(2, i - 1);
		}
		if(UsingMultiRes)
			break;
		if(converge){
			energy2 = CalcEnergy(1);
			if(abs(energy2 - energy1) < 0.0001)break;
			else{
				energy1 = energy2;
			}
		}
	}
	
	


	for (int i = 1; i <= numV; i++){
	
		
		mesh->vertices[3 * i + 0] = posPrime[i].x;
		mesh->vertices[3 * i + 1] = posPrime[i].y;
		mesh->vertices[3 * i + 2] = posPrime[i].z;
	}




	if (!test){
		solver.ResetSolver(numV + handleSize, numV, 3);
	}
	else{
		solver.ResetSolver(numV , numV, 3);
	}
	Factorized = false;

	//cout << "DDD" << endl;
	delete[] b[0];
	delete[] b[1];
	delete[] b[2];
	delete[] b;
	//cout << "EEE" << endl;

	
}

void motion(int x, int y)
{

	tbMotion(x, y);
	if(current_mode==SELECT_MODE&&selectFlag){
		linePos.set(x,y);
	}


	if (current_mode == DEFORM_MODE && deform_mesh_flag == true)
	{
		matrix44 m;
		vector4 vec = vector4((float)(x - last_x) / 1000.0f, (float)(y - last_y) / 1000.0f, 0.0, 1.0);
		gettbMatrix((float *)&m);
		vec = m * vec;
		
		if(continuous){
			deltaVec.x =  vec.x * 10.0;
			deltaVec.y = -1.0* vec.y * 10.0;
			deltaVec.z =  -1.0*vec.z * 10.0;

			//deltaVec = m * deltaVec;
		
			if(DeformCount<5){
				DeformCount++;
			}
			else{
				clock_t t1, t2;
				t1 = clock();
			
				Deform();
				DeformCount = 0;
				t2 = clock();
				printf("%lf\n", (t2-t1)/(double)(CLOCKS_PER_SEC));
				firstPutMap = true;
				mapHandles.clear();
			}
		}
		else if(!continuous){
			deltaVec.x += -1.0 * vec.x;
			deltaVec.y += -1.0 * vec.y ;
			deltaVec.z += -1.0 * vec.z ;
		}
	
	}

	last_x = x;
	last_y = y;

	int dx = x - TheMouse.x;
	int dy = y - TheMouse.y;

	/*
	 *	update the mouse position
	 */
	TheMouse.x = x;
	TheMouse.y = y;

	/*
	 *	Check MyButton to see if we should highlight it cos the mouse is over it
	 */
	ButtonPassive(&MyButton,x,y);
	ButtonPassive(&MyButton2,x,y);
	ButtonPassive(&MyButton3,x,y);
	ButtonPassive(&MyButton4,x,y);
	ButtonPassive(&MyButton5,x,y);
	ButtonPassive(&MyButton6,x,y);
	ButtonPassive(&MyButton7,x,y);
	ButtonPassive(&MyButton8,x,y);
	ButtonPassive(&MyButton9,x,y);
	ButtonPassive(&MyButton10,x,y);
	ButtonPassive(&MyButton11,x,y);
	ButtonPassive(&MyButton12,x,y);
	/*
	 *	Force a redraw of the screen
	 */
	glutPostRedisplay();
}
void MousePassiveMotion(int x, int y)
{
	/*
	 *	Calculate how much the mouse actually moved
	 */
	int dx = x - TheMouse.x;
	int dy = y - TheMouse.y;

	/*
	 *	update the mouse position
	 */
	TheMouse.x = x;
	TheMouse.y = y;

	/*
	 *	Check MyButton to see if we should highlight it cos the mouse is over it
	 */
	ButtonPassive(&MyButton,x,y);
	ButtonPassive(&MyButton2,x,y);
	ButtonPassive(&MyButton3,x,y);
	ButtonPassive(&MyButton4,x,y);
	ButtonPassive(&MyButton5,x,y);
	ButtonPassive(&MyButton6,x,y);
	ButtonPassive(&MyButton7,x,y);
	ButtonPassive(&MyButton8,x,y);
	ButtonPassive(&MyButton9,x,y);
	ButtonPassive(&MyButton10,x,y);
	ButtonPassive(&MyButton11,x,y);
	ButtonPassive(&MyButton12,x,y);
	/*
	 *	Note that I'm not using a glutPostRedisplay() call here. The passive motion function 
	 *	is called at a very high frequency. We really don't want much processing to occur here.
	 *	Redrawing the screen every time the mouse moves is a bit excessive. Later on we 
	 *	will look at a way to solve this problem and force a redraw only when needed. 
	 */
}
// ----------------------------------------------------------------------------------------------------
// keyboard related functions

void keyboard(unsigned char key, int x, int y)
{
	
	switch (key)
	{
	case 'd':
		//current_mode = DEFORM_MODE;
		//cout << "!!!" << endl;
		break;
	default:
	case 's':
		//current_mode = SELECT_MODE;
		break;
	case 'a':
		//noPoint = !noPoint;
		UsingMultiRes = !UsingMultiRes;
		break;
	}
	
}

// ----------------------------------------------------------------------------------------------------
// main function

void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
}

void LoadMesh(string s){

	char cstr[200] ;
	strcpy(cstr, s.c_str());
	mesh = glmReadOBJ(cstr);
	int a, b, c;
	numV = mesh->numvertices;

	glmUnitize(mesh);


	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);

	for(int i=0;i<30000;i++){
		pos[i].x=0;
		pos[i].y = 0;
		pos[i].z = 0;
	}

	for (int i = 1; i < numV; i++){
		pos[i].x = mesh->vertices[i * 3 + 0];
		pos[i].y = mesh->vertices[i * 3 + 1];
		pos[i].z = mesh->vertices[i * 3 + 2];
	}
	for (int i = 0; i < 30000; i++){
		edges[i].o1 = 0;
		edges[i].o2 = 0;
		edges[i].v1 = 0;
		edges[i].v2 = 0;
		edges[i].weight = 0;
		edges[i].weight2 = 0;
	}
	CalcNeighbor();

	for (int i = 0; i < 30000; i++){
		S[i].zeros();
		R[i].zeros();
	
	}


	for (int i = 1; i <= numV; i++){
		pos[i].x = mesh->vertices[i * 3 + 0];
		pos[i].y = mesh->vertices[i * 3 + 1];
		pos[i].z = mesh->vertices[i * 3 + 2];
	}
	for (int i = 1; i <= numV; i++){
		posPrime[i].x = pos[i].x;
		posPrime[i].y = pos[i].y;
		posPrime[i].z = pos[i].z;
	}
	
	CalcCotangentWeight();
}

int main(int argc, char *argv[])
{
	mesh = glmReadOBJ("../tigerC.obj");
	int a, b, c;
	numV = mesh->numvertices;

	glmUnitize(mesh);


	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);

	

	for (int i = 1; i < numV; i++){
		pos[i].x = mesh->vertices[i * 3 + 0];
		pos[i].y = mesh->vertices[i * 3 + 1];
		pos[i].z = mesh->vertices[i * 3 + 2];
	}
	for (int i = 0; i < 30000; i++){
		edges[i].o1 = 0;
		edges[i].o2 = 0;
		edges[i].v1 = 0;
		edges[i].v2 = 0;
		edges[i].weight = 0;
		edges[i].weight2 = 0;
	
	}
	CalcNeighbor();

	for (int i = 0; i < 30000; i++){
		S[i].set_size(3, 3);
		R[i].set_size(3, 3);
	
	}


	for (int i = 1; i <= numV; i++){
		pos[i].x = mesh->vertices[i * 3 + 0];
		pos[i].y = mesh->vertices[i * 3 + 1];
		pos[i].z = mesh->vertices[i * 3 + 2];
	}
	for (int i = 1; i <= numV; i++){
		posPrime[i].x = pos[i].x;
		posPrime[i].y = pos[i].y;
		posPrime[i].z = pos[i].z;
	}
	
	CalcCotangentWeight();
	
	
	WindWidth = 800;
	WindHeight = 600;

	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };

	// color list for rendering handles
	float red[] = { 1.0, 0.0, 0.0 };
	colors.push_back(red);
	float yellow[] = { 1.0, 1.0, 0.0 };
	colors.push_back(yellow);
	float blue[] = { 0.0, 1.0, 1.0 };
	colors.push_back(blue);
	float green[] = { 0.0, 1.0, 0.0 };
	colors.push_back(green);

	glutInit(&argc, argv);
	glutInitWindowSize(WindWidth, WindHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("ARAP");

	glutReshapeFunc(Reshape);
	glutDisplayFunc(Display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutPassiveMotionFunc(MousePassiveMotion);
	glutKeyboardFunc(keyboard);
	glClearColor(0, 0, 0, 0);

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	tbInit(GLUT_LEFT_BUTTON);
	tbAnimate(GL_TRUE);

	glutTimerFunc(40, timf, 0); // Set up timer for 40ms, about 25 fps

	// load 3D model
	//mesh = glmReadOBJ("../dolphinC.obj");
	//cout << mesh->vertices[3] << " " << mesh->vertices[1] << " " << mesh->vertices[2]<<endl;


	//MeshLap();

	//cout << "aaa" << endl;

	glutMainLoop();

	

	return 0;

}

void CalcNeighbor2(){

	numV2 = mesh2->numvertices;

	int a, b, c;

	for (int i = 0; i < mesh2->numtriangles; i++){
		a = mesh2->triangles[i].vindices[0];
		b = mesh2->triangles[i].vindices[1];
		c = mesh2->triangles[i].vindices[2];
	
		bool check = false;
		for (int j = 0; j < edgeNum2; j++){
			if ((edges2[j].v1 == a&&edges2[j].v2 == b) || (edges2[j].v1 == b&&edges2[j].v2 == a)){
				check = true;
				if (edges2[j].o1 == 0)
					edges2[j].o1 = c;
				else {
					edges2[j].o2 = c;

				}
			}
		}
		if (!check){
		
			edges2[edgeNum2].v1 = a;
			edges2[edgeNum2].v2 = b;
			edges2[edgeNum2].o1 = c;
			edges2[edgeNum2].o2 = 0;

			edgeNum2++;
		}
	
		check = false;

		for (int j = 0; j < edgeNum2; j++){
			if ((edges2[j].v1 == a&&edges2[j].v2 == c) || (edges2[j].v1 == c&&edges2[j].v2 == a)){
				check = true;
				if (edges2[j].o1 == 0)
					edges2[j].o1 = b;
				else {
					edges2[j].o2 = b;
				}
			}
		}
		if (!check){
		
			edges2[edgeNum2].v1 = a;
			edges2[edgeNum2].v2 = c;
			edges2[edgeNum2].o1 = b;
			edges2[edgeNum2].o2 = 0;

	
			edgeNum2++;

		}

		check = false;

		for (int j = 0; j < edgeNum2; j++){
			if ((edges2[j].v1 == b&&edges2[j].v2 == c) || (edges2[j].v1 == c&&edges2[j].v2 == b)){
				check = true;
				if (edges2[j].o1 == 0)
					edges2[j].o1 = a;
				else{
					edges2[j].o2 = a;
				}
			}
		}
		if (!check){
		
			edges2[edgeNum2].v1 = b;
			edges2[edgeNum2].v2 = c;
			edges2[edgeNum2].o1 = a;
			edges2[edgeNum2].o2 = 0;
		
			edgeNum2++;
			
		}
	}
}

void CalcCotangentWeight2(){
	srand(time(NULL));

	for (int i = 0; i < edgeNum2; i++){

		vector3 t1, t2;
		vector3 a1, a2, b1, b2;
		if (edges2[i].o2 != 0){

			if(IsCotangentWeight){
				t1.set(pos2[edges2[i].o1].x, pos2[edges2[i].o1].y, pos2[edges2[i].o1].z);
				t2.set(pos2[edges2[i].o2].x, pos2[edges2[i].o2].y, pos2[edges2[i].o2].z);
				a1 = (pos2[edges2[i].v1] - t1);
				a2 = (pos2[edges2[i].v2] - t1);
				b1 = (pos2[edges2[i].v1] - t2);
				b2 = (pos2[edges2[i].v2] - t2);
				float aaR = acos(DotProduct(a1, a2)/(a1.length()*a2.length()));
				float bbR = acos(DotProduct(b1, b2) / (b1.length()*b2.length()));
				edges2[i].weight = 0.5f * ((1.0 / tan(aaR)) + (1.0 / tan(bbR)));
				if(edges2[i].weight<0) edges2[i].weight = 0;
			}
			else{
				edges2[i].weight = 1.0;
			}
			//cout << edges[i].weight << endl;
		
		}
		else{
			if(IsCotangentWeight){
				t1.set(pos2[edges2[i].o1].x, pos2[edges2[i].o1].y, pos2[edges2[i].o1].z);
				a1 = pos2[edges2[i].v1] - t1;
				a2 = pos2[edges2[i].v2] - t1;
				float aR = acos(DotProduct(a1, a2) / (a1.length()*a2.length()));
				edges2[i].weight = 0.5 * (1.0 / tan(aR) );
				if(edges2[i].weight<0)edges2[i].weight = 0;
			}
			else{
			edges2[i].weight = 1.0;
			}
			cout << "BOUNDARY " << i << " " << edges2[i].v1 << " " << edges2[i].v2 << "   " << edges2[i].weight << endl;
		}
	}
}

void LoadMesh2(string s){

	char cstr[200] ;
	strcpy(cstr, s.c_str());
	mesh2 = glmReadOBJ(cstr);
	int a, b, c;
	numV2 = mesh2->numvertices;

	glmUnitize(mesh2);


	glmFacetNormals(mesh2);
	glmVertexNormals(mesh2, 90.0);

	for(int i=0;i<30000;i++){
		pos2[i].x=0;
		pos2[i].y = 0;
		pos2[i].z = 0;
	}

	for (int i = 1; i < numV2; i++){
		pos2[i].x = mesh2->vertices[i * 3 + 0];
		pos2[i].y = mesh2->vertices[i * 3 + 1];
		pos2[i].z = mesh2->vertices[i * 3 + 2];
	}
	for (int i = 0; i < 30000; i++){
		edges2[i].o1 = 0;
		edges2[i].o2 = 0;
		edges2[i].v1 = 0;
		edges2[i].v2 = 0;
		edges2[i].weight = 0;
		edges2[i].weight2 = 0;
	}
	CalcNeighbor2();

	for (int i = 0; i < 30000; i++){
		S2[i].set_size(3,3);
		R2[i].set_size(3,3);
		S2[i].zeros();
		R2[i].zeros();
	}
	

	for (int i = 1; i <= numV2; i++){
		pos2[i].x = mesh2->vertices[i * 3 + 0];
		pos2[i].y = mesh2->vertices[i * 3 + 1];
		pos2[i].z = mesh2->vertices[i * 3 + 2];
	}
	for (int i = 1; i <= numV2; i++){
		posPrime2[i].x = pos2[i].x;
		posPrime2[i].y = pos2[i].y;
		posPrime2[i].z = pos2[i].z;
	}
	
	CalcCotangentWeight2();
}





void CalcMapHandle(){
	
	bool tempFlag;

	for (int vertIter = 0; vertIter < handles[selected_handle_id].size(); vertIter++)
	{
		int idx = mapping[handles[selected_handle_id][vertIter]];
		tempFlag = true;
		for (int j = 0;j<mapHandles.size();j++){
			if(mapHandles[j]==idx){
				tempFlag = false;
				break;
			}
		}
		if(tempFlag){
			mapHandles.push_back(idx);
			
		}
	}

}



LeastSquaresSparseSolver solver2;

void Deform2(){
	float energy1,energy2;
	energy1 = 99999;
	//int numMapping[30000];
	//memset(numMapping,sizeof(numMapping),0);

	/*
	
	for (int i = 1; i <= numV2; i++){
		posPrime2[i].x = 0;
		posPrime2[i].y = 0;
		posPrime2[i].z = 0;
	}

	for(int i = 1 ; i<= numV ; i++){
		posPrime2[mapping[i]] =  posPrime2[mapping[i]] + posPrime[i];
		numMapping[mapping[i]] += 1;
	}
	for(int i=1;i<=numV2;i++){
		posPrime2[i] = posPrime2[i] / ((float)numMapping[i]);
	}
	*/

	int numIter = 500;
	int iterCount = 0;

	
	if(firstPutMap){
		CalcMapHandle();
		for (int vertIter = 0; vertIter < mapHandles.size(); vertIter++)
		{
			int idx = mapHandles[vertIter];
			vector3 pt(mesh2->vertices[idx*3+0]+deltaVec.x,mesh2->vertices[idx*3+1]+deltaVec.y,mesh2->vertices[idx*3+2]+deltaVec.z);
			posPrime2[idx].x = pt[0];
			posPrime2[idx].y = pt[1];
			posPrime2[idx].z = pt[2];
		}
		firstPutMap = false;
	}
		handleSize2 = 0;

			for(int i =1;i<=numV2;i++){
					int v = inverseMapping[i];
					if(handleTable[v]){
						handleSize2 += 1;
					}
			}



	float** b = new float*[3];

	if (!test){


		b[0] = new float[numV2 + handleSize2];
		b[1] = new float[numV2 + handleSize2];
		b[2] = new float[numV2 + handleSize2];

	}
	else{
		b[0] = new float[numV2];
		b[1] = new float[numV2];
		b[2] = new float[numV2];	
	}

	if (!Factorized2){

		if(!test){
			//cout<<handleSize2<<endl;
			solver2.Create(numV2 + handleSize2,numV2 ,3);
		}
		else{
			solver2.Create(numV2, numV2, 3);
		}



		float www[30000];
		for (int k = 0; k < 30000; k++){
			www[k] = 0.0;
		}

		if (!test){

			for (int k = 0; k < edgeNum2; k++){

				int i = edges2[k].v1;
				int j = edges2[k].v2;
				float ww = edges2[k].weight;   // 這邊超慢的啦~~~
				if (i > numV2 || i<1 || j>numV2 || j < 1)
					cout << "QQ: " << i << " " << j << " " << ww << endl;
				solver2.AddSysElement(i - 1, j - 1, (-1.0 * ww));
				solver2.AddSysElement(j - 1, i - 1, (-1.0 * ww));
				//solver.AddSysElement(i - 1, j - 1, -1.0 / numNeighbor[i]);
				//solver.AddSysElement(j - 1, i - 1, -1.0 / numNeighbor[j]);
				www[i] += ww;
				www[j] += ww;

			}
		}
		else{

			for (int k = 0; k < edgeNum2; k++){

				int i = edges2[k].v1;
				int j = edges2[k].v2;
				float ww = edges2[k].weight;   // 這邊超慢的啦~~~
				if (i > numV2 || i<1 || j>numV2 || j < 1)
					cout << "QQ: " << i << " " << j << " " << ww << endl;

				if (!handleTable[inverseMapping[i]]){
					solver2.AddSysElement(i - 1, j - 1, (-ww));
					www[i] += ww;
				}
				if (!handleTable[inverseMapping[j]]){
					solver2.AddSysElement(j - 1, i - 1, (-ww));
					www[j] += ww;
				}
				//solver.AddSysElement(i - 1, j - 1, -1.0 / numNeighbor[i]);
				//solver.AddSysElement(j - 1, i - 1, -1.0 / numNeighbor[j]);

			}
		}
		
			if (!test){
				for (int k = 0; k < numV2; k++){
					//cout << "www " << k << " " << www[k] << endl;
					solver2.AddSysElement(k, k, www[k + 1]);

				}
			}
			else{
				for (int k = 0; k < numV2; k++){
					if (!handleTable[inverseMapping[k + 1]])
						solver2.AddSysElement(k, k, www[k + 1]);
					else{
						solver2.AddSysElement(k, k, 1.0);
					}
			
				}
			}
		cout<<"aaaa"<<endl;
		int count = numV2;
		if(!test){
			for(int i =1;i<=numV2;i++){
					int v = inverseMapping[i];
					if(handleTable[v]){
						solver2.AddSysElement(count,i-1,1.0);
						count++;
					}
				
			}
		}
		cout<<"bbbb"<<endl;

	}

	
	while(iterCount++<=numIter){
	
		for (int i = 1; i <= numV2; i++){
			S2[i].zeros();
			R2[i].zeros();
		}
		


		for (int k = 0; k < edgeNum2; k++){
			int i = edges2[k].v1;
			int j = edges2[k].v2;
			if (i ==0|| j==0)
				cout <<"000000 "<< i << " " << j << endl;
			vector3 E = pos2[i] - pos2[j];
			vector3 EPrime = posPrime2[i] - posPrime2[j];
			float wij = edges2[k].weight;

			//wij = -1.0 / numNeighbor[i];
			//cout<<i<<" "<<j<<" "<<wij<<" "<<E.x<<" "<<EPrime.x<<endl;
			S2[i](0, 0) += wij * E.x * EPrime.x;
			S2[i](0, 1) += wij * E.x * EPrime.y;
			S2[i](0, 2) += wij * E.x * EPrime.z;
			S2[i](1, 0) += wij * E.y * EPrime.x;
			S2[i](1, 1) += wij * E.y * EPrime.y;
			S2[i](1, 2) += wij * E.y * EPrime.z;
			S2[i](2, 0) += wij * E.z * EPrime.x;
			S2[i](2, 1) += wij * E.z * EPrime.y;
			S2[i](2, 2) += wij * E.z * EPrime.z;
			E = pos2[j] - pos2[i];
			EPrime = posPrime2[j] - posPrime2[i];

			//wij = -1.0 / numNeighbor[j];

			S2[j](0, 0) += wij * E.x * EPrime.x;
			S2[j](0, 1) += wij * E.x * EPrime.y;
			S2[j](0, 2) += wij * E.x * EPrime.z;
			S2[j](1, 0) += wij * E.y * EPrime.x;
			S2[j](1, 1) += wij * E.y * EPrime.y;
			S2[j](1, 2) += wij * E.y * EPrime.z;
			S2[j](2, 0) += wij * E.z * EPrime.x;
			S2[j](2, 1) += wij * E.z * EPrime.y;
			S2[j](2, 2) += wij * E.z * EPrime.z;
				
		}


		int QQ = 0;
		for (int i = 1; i <= numV2; i++){

			mat U, V;
			vec ss;
			svd(U, ss, V, S2[i]);

			mat I = eye<mat>(3,3);
			float determination = det(V*U.t());
			I(2, 2) = determination;
			if (abs(determination - (-1.0)) < 0.001)
				QQ++;
			
			R2[i] = V * I* U.t();
			
		}
	

		if (!test){
			for (int i = 0; i < numV2 + handleSize2; i++){
				b[0][i] = 0;
				b[1][i] = 0;
				b[2][i] = 0;
			}
		}
		else{
			for (int i = 0; i < numV2 ; i++){
				b[0][i] = 0;
				b[1][i] = 0;
				b[2][i] = 0;
			}
		}
		

		for (int k = 0; k < edgeNum2; k++){

			int i = edges2[k].v1;
			int j = edges2[k].v2;
			float ww = edges2[k].weight;   // 這邊超慢的啦~~~
			vec pi(3); pi(0) = pos2[i].x; pi(1) = pos2[i].y; pi(2) = pos2[i].z;
			vec pj(3); pj(0) = pos2[j].x; pj(1) = pos2[j].y; pj(2) = pos2[j].z;


			//ww = -1.0 / numNeighbor[i];
			
			vec temp =(R2[i] + R2[j]) * (pi - pj);
			
			//ww = -1.0 / numNeighbor[j];

			vec temp2 = (R2[j] + R2[i]) * (pj - pi);
			b[0][i - 1] += (ww / 2.0) * temp(0);
			b[1][i - 1] += (ww / 2.0) * temp(1);
			b[2][i - 1] += (ww / 2.0) * temp(2);
			b[0][j - 1] += (ww / 2.0) * temp2(0);
			b[1][j - 1] += (ww / 2.0) * temp2(1);
			b[2][j - 1] += (ww / 2.0) * temp2(2);

		}

	
	
		if (!test){
			int count;
			count = numV2;
			for(int i =1;i<=numV2;i++){
				int v = inverseMapping[i];
				if(handleTable[v]){
					b[0][count] = posPrime2[i].x;
					b[1][count] = posPrime2[i].y;
					b[2][count] = posPrime2[i].z;
					count++;
				}
			}
		}
		else{
			for(int i =1;i<=numV2;i++){
						int v = inverseMapping[i];
						if(handleTable[v]){
						b[0][i-1] = posPrime2[i].x;
						b[1][i-1] = posPrime2[i].y;
						b[2][i-1] = posPrime2[i].z;
						}
			}
		}
		//cout << "AAA" << endl;
		solver2.SetRightHandSideMatrix(b);

		if (!Factorized2){
			solver2.CholoskyFactorization();
			Factorized2 = true;
		}

		//cout << "BBB" << endl;

		

		solver2.CholoskySolve(0);
		solver2.CholoskySolve(1);
		solver2.CholoskySolve(2);
		//cout << "Ccc" << endl;

		for (int i = 1; i <= numV2; i++){
			posPrime2[i].x = solver2.GetSolution(0, i - 1);
			posPrime2[i].y = solver2.GetSolution(1, i - 1);
			posPrime2[i].z = solver2.GetSolution(2, i - 1);
		}
			if(converge){
			energy2 = CalcEnergy(2);
			if(abs(energy2 - energy1) < 0.005)break;
			else{
				energy1 = energy2;
			}
		}

	}
	//cout<<iterCount<<endl;
	for (int i = 1; i <= numV2; i++){
		mesh2->vertices[3 * i + 0] = posPrime2[i].x;
		mesh2->vertices[3 * i + 1] = posPrime2[i].y;
		mesh2->vertices[3 * i + 2] = posPrime2[i].z;
	}
	
	Factorized2 = false;
	if(!test)
		solver2.ResetSolver(numV2+handleSize2 , numV2, 3);
	else
		solver2.ResetSolver(numV2,numV2,3);

	delete[] b[0];
	delete[] b[1];
	delete[] b[2];
	delete[] b;
}