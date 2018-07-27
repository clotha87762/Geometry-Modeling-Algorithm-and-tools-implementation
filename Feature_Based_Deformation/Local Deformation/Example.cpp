#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <Windows.h>
#include <gl/GL.h>
#include <glut.h>

#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

using namespace std;

_GLMmodel *mesh;
int WindWidth, WindHeight;

int last_x , last_y;
int selectedFeature = -1;
vector<int> featureList;

matrix44 rbfMat = { 0 };
matrix44 invRBFMat = { 0 };
matrix44 weightMat = { 0 };

int whichFeature = 0;

double sigma = 1.0;
void ComputeEBF();

vector3 fpOriPos[4];

matrix44 d(0);
double scale = 30.0;
float oriMesh[10000];

float Gaussian(double r){
	return (float)exp(-1.0*r*r / (2.0 * sigma*sigma));
}

void ComputeRBF(){

	if (featureList.size() != 4)
		return;

	for (int i = 0; i < 4; i++){

		for (int j = 0; j < 4; j++){
			if (i == j)
				rbfMat[i][j] = 1.0;
			else{
				vector3 pt1(mesh->vertices[featureList[i] * 3 + 0], mesh->vertices[featureList[i] * 3 + 1], mesh->vertices[featureList[i] * 3 + 2]);
				vector3 pt2(mesh->vertices[featureList[j] * 3 + 0], mesh->vertices[featureList[j] * 3 + 1], mesh->vertices[featureList[j] * 3 + 2]);
				rbfMat[i][j] = Gaussian((pt1 - pt2).length());
				//norm += rbfMat[i][j];
			}
		}


	}
	invRBFMat = rbfMat;
	invRBFMat.invert();

	double norm = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++){
			norm += invRBFMat[i][j] * invRBFMat[i][j];
		}
	scale = sqrt(norm);
	cout << "scale: " << scale << endl;
}

void Reshape(int width, int height)
{
  int base = min(width , height);

  tbReshape(width, height);
  glViewport(0 , 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(70.0,(GLdouble)width / (GLdouble)height ,1.0, 128.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -3.5);

  WindWidth = width;
  WindHeight = height;
}

void Display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  tbMatrix();
  
  // render solid model
  glEnable(GL_LIGHTING);
  glColor3f(1.0 , 1.0 , 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
  glmDraw(mesh , GLM_SMOOTH);

  // render wire model
  glPolygonOffset(1.0 , 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glLineWidth(1.0f);
  glColor3f(0.6 , 0.0 , 0.8);
  glPolygonMode(GL_FRONT_AND_BACK , GL_LINE);
  glmDraw(mesh , GLM_SMOOTH);

  // render features
  glPointSize(10.0);
  glColor3f(1.0 , 0.0 , 0.0);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
	for (int i = 0 ; i < featureList.size() ; i++)
	{
		int idx = featureList[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
	}
  glEnd();
  
  glPopMatrix();

  glFlush();  
  glutSwapBuffers();
}

vector3 Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];				//Model_view matrix
	double ProjectionMatrix[16];			//Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x , viewport[3] - (int)_2Dpos.y , 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = {0.0 , 0.0 , 0.0};

	gluUnProject(X , ((double)viewport[3] - Y) , (double)Depth , ModelViewMatrix , ProjectionMatrix , viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector3(wpos[0] , wpos[1] , wpos[2]);
}

void mouse(int button, int state, int x, int y)
{
  tbMouse(button, state, x, y);

  // add feature
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < mesh->numvertices ; i++)
	  {
		  vector3 pt(mesh->vertices[3 * i + 0] , mesh->vertices[3 * i + 1] , mesh->vertices[3 * i + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = i;
			  fpOriPos[featureList.size()] = pt;
		  }
	  }

	  featureList.push_back(minIdx);
	  if (featureList.size() == 4)
		  ComputeRBF();

  }

  // manipulate feature
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < featureList.size() ; i++)
	  {
		  int idx = featureList[i];
		  vector3 pt(mesh->vertices[3 * idx + 0] , mesh->vertices[3 * idx + 1] , mesh->vertices[3 * idx + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = featureList[i];
			  whichFeature = i;
		  }
	  }

	  selectedFeature = minIdx;
  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	  selectedFeature = -1;

  last_x = x;
  last_y = y;
}



void motion(int x, int y)
{
  tbMotion(x, y);

  if (selectedFeature != -1)
  {
	  //ComputeRBF();

	  matrix44 m;
	  vector4 vec = vector4((float)(x - last_x) / 100.0f , (float)(y - last_y) / 100.0f , 0.0 , 1.0);
	  
	  gettbMatrix((float *)&m);
	  vec = m * vec;
	  std::cout << vec.w<< endl;
	  //mesh->vertices[3 * selectedFeature + 0] += vec.x;
	  //mesh->vertices[3 * selectedFeature + 1] -= vec.y;
	  //mesh->vertices[3 * selectedFeature + 2] += vec.z;
	  //matrix44 d(0);

	  //d[3][0] = 0;
	  //d[3][1] = 0;
	  //d[3][2] = 0;
	  //d[3][3] = 0;

	  d[0][whichFeature] += vec.x/scale;
	  d[1][whichFeature] += vec.y/scale;
	  d[2][whichFeature] += vec.z/scale;
	  
	 cout << "selectedFeature:  "<<whichFeature<< endl;
	  matrix44 a = invRBFMat * d;
	  for (int i = 0; i < 4; i++){
		  for (int j = 0; j < 4; j++){
			  //std::cout << (invRBFMat)[j][i] << " ";
		  }
		 // std::cout << endl;
	  }
	  //std::cout << "=========================" << endl;

		  for (int i = 0; i < mesh->numvertices; i++){
			  bool flag = true;

			  for (int j = 0; j < featureList.size(); j++)
				  if (featureList[j] == i)
					  flag = false;
			  
			  //if (!flag)continue;
			  
			  

			 

			  for (int j = 0; j < featureList.size(); j++){
				  //vector3 fp(mesh->vertices[featureList[j] * 3 + 0], mesh->vertices[featureList[j] * 3 + 1], mesh->vertices[featureList[j] * 3 + 2]);
				  //vector3 pt(mesh->vertices[i * 3 + 0], mesh->vertices[i * 3 + 1], mesh->vertices[i * 3 + 2]);
				  vector3 pt(oriMesh[i * 3 + 0], oriMesh[i * 3 + 1], oriMesh[i * 3 + 2]);
				  

				  mesh->vertices[3 * i + 0] = oriMesh[ 3 * i + 0] + (a[0][j]) *Gaussian((pt - fpOriPos[j]).length());
				  mesh->vertices[3 * i + 1] = oriMesh[3 * i + 1] - (a[1][j])*Gaussian((pt - fpOriPos[j]).length());
				  mesh->vertices[3 * i + 2] = oriMesh[3 * i + 2] + (a[2][j])*Gaussian((pt - fpOriPos[j]).length());
				  //mesh->vertices[3 * i + 0] += a[0][j]/a[3][j] * Gaussian((pt - fp).length());
				  //mesh->vertices[3 * i + 1] -= a[1][j]/a[3][j] * Gaussian((pt - fp).length());
				  //mesh->vertices[3 * i + 2] += a[2][j]/a[3][j] * Gaussian((pt - fp).length());
				  
				 // if ( i == mesh->numvertices - 100)
					//  std::cout << norm << endl;
				//  std::cout << vec.x << " " << vec.y << " " << vec.z << "   //" << (a[0][j] / a[3][j])*Gaussian((pt - fp).length()) << " " << (a[1][j] / a[3][j])*Gaussian((pt - fp).length()) << " " << (a[2][j] / a[3][j])*Gaussian((pt - fp).length()) << endl;

			  }

			  
		  
		   }
  }

  last_x = x;
  last_y = y;
}

void timf(int value)
{
  glutPostRedisplay();
  glutTimerFunc(1, timf, 0);
}

int main(int argc, char *argv[])
{
  WindWidth = 400;
  WindHeight = 400;
	
  GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
  GLfloat light_diffuse[] = {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position[] = {0.0, 0.0, 1.0, 0.0};

  glutInit(&argc, argv);
  glutInitWindowSize(WindWidth, WindHeight);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow("Trackball Example");

  glutReshapeFunc(Reshape);
  glutDisplayFunc(Display);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
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
  mesh = glmReadOBJ("../data/head.obj");
  
  glmUnitize(mesh);
  glmFacetNormals(mesh);
  glmVertexNormals(mesh , 90.0);

  for (int i = 0; i < mesh->numvertices; i++){
	  oriMesh[3*i + 0] = mesh->vertices[3*i + 0];
	  oriMesh[3 * i + 1] = mesh->vertices[3 * i + 1];
	  oriMesh[3 * i + 2] = mesh->vertices[3 * i + 2];
  }
  glutMainLoop();

  return 0;

}

