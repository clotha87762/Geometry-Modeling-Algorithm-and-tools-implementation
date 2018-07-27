

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



_GLMmodel *mesh2;
int numV2;


typedef enum { SELECT_MODE, DEFORM_MODE } ControlMode;
ControlMode current_mode = SELECT_MODE;

void Deform2();



bool flag2 = false;
bool Factorized2 = false;
int edgeNum2 = 0;

struct e2{
	int v1 ;
	int v2 ;
	float weight ;
	float weight2 ;
	int o1 ;
	int o2 ;
};
typedef struct e edge2;


vector3 pos2[30000];
vector3 posPrime2[30000];
mat S2[30000];
mat R2[30000];


bool IsCotangentWeight2 ;


edge edges2[30000];


int numV2;





void CalcNeighbor(){

	numV2 = mesh2->numvertices;

	int a, b, c;

	for (int i = 0; i < mesh->numtriangles; i++){
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
		
			edges2[edgeNum].v1 = a;
			edges2[edgeNum].v2 = b;
			edges2[edgeNum].o1 = c;
			edges2[edgeNum].o2 = 0;

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






void CalcCotangentWeight(){
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
	//CalcNeighbor();

	for (int i = 0; i < 30000; i++){
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
	
	//CalcCotangentWeight();
}