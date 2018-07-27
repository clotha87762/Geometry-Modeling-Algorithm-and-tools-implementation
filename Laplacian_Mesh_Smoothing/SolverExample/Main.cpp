#include <stdlib.h>
#include <iostream>
#include "LeastSquaresSparseSolver.h"
#include "glm.h"
#include <vector>
#include <ctime>
#include "trackball.h"
#include <cstring>
_GLMmodel* mesh;

int last_x, last_y;
int cp[1000];

int compare(const void *a, const void *b)
{
	int c = *(int *)a;
	int d = *(int *)b;
	if (c < d) { return -1; }               
	else if (c == d) { return 0; }     
	else return 1;                          
}

void Reshape(int width, int height)
{
	int base = min(width, height);

	tbReshape(width, height);
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0, (GLdouble)width / (GLdouble)height, 1.0, 128.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -3.5);

	//WindWidth = width;
	//WindHeight = height;
}

void Display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	tbMatrix();

	// render solid model
	glEnable(GL_LIGHTING);
	glColor3f(1.0, 1.0, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glmDraw(mesh, GLM_SMOOTH);

	// render wire model
	/*
	glPolygonOffset(1.0, 1.0);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glLineWidth(1.0f);
	glColor3f(0.6, 0.0, 0.8);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glmDraw(mesh, GLM_SMOOTH);
	*/
	// render features
	glPointSize(0.3);
	glColor3f(1.0, 0.0, 0.0);
	glDisable(GL_LIGHTING);
	glBegin(GL_POINTS);
	int t = 0;
	for (int i = 1; i <mesh->numvertices ; i++)
	{
		if (cp[t] == i){
			t++;
			continue;
		}
		//int idx = cp[i];
		//glVertex3fv((float *)&mesh->vertices[3 * idx]);
		glVertex3fv((float *)&mesh->vertices[3 * i]);
	}
	glColor3f(0.0, 0.0, 1.0);
	for (int i = 1; i <=1000; i++)
	{
		
		int idx = cp[i];

		glVertex3fv((float *)&mesh->vertices[3 * idx]);
		//glVertex3fv((float *)&mesh->vertices[3 * i]);
	}

	glEnd();
	
	glPopMatrix();

	glFlush();
	glutSwapBuffers();
}


void timf(int value)
{
	glutPostRedisplay();
	glutTimerFunc(1, timf, 0);
}

void mouse(int button, int state, int x, int y)
{
	tbMouse(button, state, x, y);


	last_x = x;
	last_y = y;
}



void motion(int x, int y)
{
	tbMotion(x, y);
	
	last_x = x;
	last_y = y;
}



void main(int argc,char* argv[])
{	

	

	//int connect_graph[20000][20000];
	GLfloat pos[3][20000];
	float oriCP[1000][3];
	memset(pos, 0, sizeof(pos));
	mesh = glmReadOBJ("../Dino.obj");

	clock_t t1, t2;


	int numV = mesh->numvertices;
	
	
	//pos[0] = new float[numV];
	//pos[1] = new float[numV];
	//pos[2] = new float[numV];

	std::vector<int> neighbor[12000];
	int numNeighbor[12000];
	

	float weight = 1.0f;

	memset(numNeighbor, 0, sizeof(numNeighbor));

	int a, b, c;

	for (int i = 0; i < mesh->numtriangles; i++){
		a = mesh->triangles[i].vindices[0];
		b = mesh->triangles[i].vindices[1];
		c = mesh->triangles[i].vindices[2];
		

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

		//cout << a << " " << numNeighbor[a] << " ||" << b << " " << numNeighbor[b] << " ||" << c << " " << numNeighbor[c] << " ||" << "i=" << i << endl;
	}

	//for (int i = 0; i < 50;i++)
	//cout << mesh->vertices[i*3]<<" "<<mesh->vertices[i*3+1]<<" "<<mesh->vertices[i*3+2]<<endl;

	//for (int i = 9950; i < 10002; i++)
		//cout << mesh->vertices[i * 3] << " " << mesh->vertices[i * 3 + 1] << " " << mesh->vertices[i * 3 + 2] << endl;
	
	

	unsigned seed;
	seed = (unsigned)time(NULL); 
	srand(seed);
	int temp;
	bool flag;
	for (int i = 0; i < 1000; i++){
		
		temp = (rand() % 10002) +1;
		flag = true;
		
		for (int j = 0; j < i; j++)
			if (cp[j] == temp){
				flag = false;
				i--;
				break;
			}

		if (flag)
		cp[i] = temp;
		if (temp == 0)
			cout << "FUCK!!" << endl;
		//cout <<i<<" "<< cp[i] << endl;
	}
	std::qsort(cp,1000,sizeof(int),compare);
	
	
	t1 = clock();

	LeastSquaresSparseSolver solverX;
	LeastSquaresSparseSolver solverY;
	LeastSquaresSparseSolver solverZ;

	solverX.Create(numV+1000, numV, 1);
	solverY.Create(numV + 1000, numV, 1);
	solverZ.Create(numV + 1000, numV, 1);

	for (int i = 0; i < numV; i++){	
	
		
		for (int j = 0; j < numNeighbor[i+1];j++){
			solverX.AddSysElement(i, neighbor[i+1].at(j)-1, ( (-1.0) / ((double)numNeighbor[i+1])));
			solverY.AddSysElement(i, neighbor[i+1].at(j) - 1, ((-1.0) / ((double)numNeighbor[i+1])));
			solverZ.AddSysElement(i, neighbor[i+1].at(j) - 1, ((-1.0) / ((double)numNeighbor[i+1])));
			//cout << "it  " << i << " " << (*it);
		}
	
		solverX.AddSysElement(i, i, 1.0);
		solverY.AddSysElement(i, i, 1.0);
		solverZ.AddSysElement(i, i, 1.0);
		
	}
	
	for (int i = numV; i < numV + 1000; i++){
		solverX.AddSysElement(i, cp[i - numV] -1, 1.0);
		solverY.AddSysElement(i, cp[i - numV] - 1, 1.0);
		solverZ.AddSysElement(i, cp[i - numV] - 1, 1.0);
	}
	




//	system("Pause");
	float **x = new float*[1];
	float **y = new float*[1];
	float **z = new float*[1];
	x[0] = new float[numV+1000];
	y[0] = new float[numV+1000];
	z[0] = new float[numV+1000];


	for (int i = numV; i < numV + 1000; i++){
		x[0][i] =  mesh->vertices[((cp[i - numV]) * 3) + 0];
		y[0][i] =  mesh->vertices[((cp[i - numV]) * 3) + 1];
		z[0][i] =  mesh->vertices[((cp[i - numV]) * 3) + 2];
	//	oriCP[i - numV][0] = mesh->vertices[((cp[i - numV]) * 3) + 0];
	}


	for (int i = 0; i < numV; i++){
		x[0][i] = 0.0;
		y[0][i] = 0.0;
		z[0][i] = 0.0;
	}

	//b[0][0] = 14.0f;
	//b[0][1] = 10.0f;
	//b[0][2] = 12.0f;
	//b[0][3] = 13.0f;

	//system("Pause");

	solverX.SetRightHandSideMatrix(x);
	solverY.SetRightHandSideMatrix(y);
	solverZ.SetRightHandSideMatrix(z);
	//cout << "???" << endl;
	// direct solver
	solverX.CholoskyFactorization();
	solverY.CholoskyFactorization();
	solverZ.CholoskyFactorization();
	//cout << "???" << endl;

	//system("Pause");
	solverX.CholoskySolve(0);
	solverY.CholoskySolve(0);
	solverZ.CholoskySolve(0);

	//system("Pause");
	int t = 0;
	for (int i = 0; i < numV; i++){
		pos[0][i] = solverX.GetSolution(0, i);
		pos[1][i] = solverY.GetSolution(0, i);
		pos[2][i] = solverZ.GetSolution(0, i);
		
	
	}
	// cout << "x = " << solver.GetSolution(0, numV) << " y = " << solver.GetSolution(1, numV) << " z = " << solver.GetSolution(2, numV) << endl;
	mesh->vertices[0] = 0;
	mesh->vertices[1] = 0;
	mesh->vertices[2] = 0;
	t = 0;
	double RMS = 0;
	for (int i = 1; i <= numV; i++){
		RMS += pow(mesh->vertices[i * 3 + 0] - pos[0][i - 1],2);
		RMS += pow(mesh->vertices[i * 3 + 1] - pos[1][i - 1],2);
		RMS += pow(mesh->vertices[i * 3 + 2] - pos[2][i - 1],2);
	}
	RMS /= (numV * 3);
	RMS = sqrt(RMS);
	for (int i = 1; i <= numV; i++){
		
			mesh->vertices[i * 3 + 0] = pos[0][i - 1];
			mesh->vertices[i * 3 + 1] = pos[1][i - 1];
			mesh->vertices[i * 3 + 2] = pos[2][i - 1];
			//cout << pos[0][i] << " " << pos[1][i] << " " << pos[2][i] << endl;
	}

	t2 = clock();
	cout << "RMS = " << RMS << endl;
	cout << "ReconstructionTime = " << (t2 - t1) / (double)(CLOCKS_PER_SEC) << endl;
	
	// release
	solverX.ResetSolver(0, 0, 0);
	solverY.ResetSolver(0, 0, 0);
	solverZ.ResetSolver(0, 0, 0);


	delete[] x[0];
	delete[] y[0];
	delete[] z[0];
	delete[] x;
	delete[] y;
	delete[] z;


//char* name = "dino2.obj";
//	glmWriteOBJ(mesh, name, GL_SMOOTH);

	
	
	int WindWidth = 400;
	int WindHeight = 400;

	GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };

	glutInit(&argc, argv);
	glutInitWindowSize(WindWidth, WindHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH );
	glutCreateWindow("Show Model");

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
	//mesh = glmReadOBJ("../data/head.obj");

	glmUnitize(mesh);
	glmFacetNormals(mesh);
	glmVertexNormals(mesh, 90.0);


	glutMainLoop();



	
	


	















	


	// solve the linear system
	// 5x + 2y + 6z = 14
	// 3x +      9z = 10
	//      8y + 2z = 12
	// 2x + 8y      = 13
/*
	LeastSquaresSparseSolver solver;

	solver.Create(4 , 3 , 1);

	solver.AddSysElement(0 , 0 , 5.0f);		// first row
	solver.AddSysElement(0 , 1 , 2.0f);
	solver.AddSysElement(0 , 2 , 6.0f);
	solver.AddSysElement(1 , 0 , 3.0f);		// second row
	solver.AddSysElement(1 , 2 , 9.0f);
	solver.AddSysElement(2 , 1 , 8.0f);		// third row
	solver.AddSysElement(2 , 2 , 2.0f);
	solver.AddSysElement(3 , 0 , 2.0f);		// fourth row
	solver.AddSysElement(3 , 1 , 8.0f);

	float **b = new float*[1];
	b[0] = new float[4];

	b[0][0] = 14.0f;
	b[0][1] = 10.0f;
	b[0][2] = 12.0f;
	b[0][3] = 13.0f;

	solver.SetRightHandSideMatrix(b);
	
	// direct solver
	solver.CholoskyFactorization();
	solver.CholoskySolve(0);

	//// iterative solver
	//solver.SetInitialGuess(0 , 0 , 0.0f);
	//solver.SetInitialGuess(0 , 1 , 0.0f);
	//solver.SetInitialGuess(0 , 2 , 0.0f);
	//solver.ConjugateGradientSolve();

	// get result
	cout << solver.GetSolution(0 , 0) << endl;
	cout << solver.GetSolution(0 , 1) << endl;
	cout << solver.GetSolution(0 , 2) << endl;
	
	// release
	solver.ResetSolver(0 , 0 , 0);
	system("Pause");
	delete [] b[0];
	delete [] b;
	*/
}
