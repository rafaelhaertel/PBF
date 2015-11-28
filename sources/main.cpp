// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <time.h>

#define PI 3.1415f
#define EPSILON  600.0f
#define ITER 1
#define NPART 1000
#define H 0.1f
#define REST 6378.0f
#define DT 0.0083f
#define BOUNCE 0.01f
#define PARTICLE_COUNT_X 10
#define PARTICLE_COUNT_Y 10
#define PARTICLE_COUNT_Z 5
#define GRID_RESOLUTION 80


// Include GLEW
#include <GL/glew.h>
#include <unordered_map>
#include <math.h>

// Include GLFW
#include <glfw3.h>
GLFWwindow* g_pWindow;
unsigned int g_nWidth = 1024, g_nHeight = 768;

// Include AntTweakBar
#include <AntTweakBar.h>
TwBar *g_pToolBar;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <shader.hpp>
#include <texture.hpp>
#include <controls.hpp>
#include <objloader.hpp>
#include <vboindexer.hpp>
#include <glerror.hpp>

void WindowSizeCallBack(GLFWwindow *pWindow, int nWidth, int nHeight) {

	g_nWidth = nWidth;
	g_nHeight = nHeight;
	glViewport(0, 0, g_nWidth, g_nHeight);
	TwWindowSize(g_nWidth, g_nHeight);
}

//struct Particle
//{
//	glm::vec2 pos; 					// position
//	glm::vec2 vel; 					// velocity
//	float m; 						// mass
//	glm::vec2 dp;					// Delta p, to be used during the particle projection
//	float lambda;					// particle lambda
//	float rho;						// density at the particle
//	float C;						// density constraint for the particle
//	std::vector< unsigned int > n;	// neighbor particles' indexes
//	float hash;						// the hash for the particle
//};

//typedef std::unordered_multimap< int, int > Hash;
//Hash hash_table;

class Particle
{
   public:
      glm::vec3 position;		// Posição Inicial
      glm::vec3 pred_position;  // Posição Prevista durante o passo
      glm::vec3 velocity;
	  glm::vec3 delta_p;
	  float mass;
	  float lambda;
	  float rho;
	  float C;
	  float hash;
	  std::vector<unsigned int> neighbors;
};

std::vector<Particle> particles;
std::vector< Particle > predict_p_list;
//std::vector<float> g_grid;
float g_xmax = 1;
float g_xmin = 0;
float g_ymax = 10;
float g_ymin = 0;
float g_zmax = 0.5;
float g_zmin = 0;
float h_sphere = 0.05;
float g_h = 0.1;
float POW_H_9 =(g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h); // h^9
float POW_H_6 =(g_h*g_h*g_h*g_h*g_h*g_h); // h^6
//float wall = 10;
//time_t g_initTime = time(0);
//float rotatey = 0;
//float rotatex = 0;
//float rotatez = 0;
float g_k = 0.0001f;
float g_dq = 0.3f;
float gota = 0;
glm::vec3 gravity = glm::vec3(0.0, -9.8, 0.0);
//int npart = PARTICLE_COUNT_X*PARTICLE_COUNT_Y*PARTICLE_COUNT_Z;

glm::vec3 fext = glm::vec3(0.0, -9.8, 0.0);


//void initializeParticles () {
//	particles.reserve(NPART);
//	srand((unsigned int) 1);
//
//	for (int i = 0 ; i < NPART ; i++){
//		Particle p;
//		p.position.x = (float)rand()/(float)(RAND_MAX/g_xmax);
//		p.position.y = (float)rand()/(float)(RAND_MAX/g_ymax);
//		p.position.z = (float)rand()/(float)(RAND_MAX/g_zmax);;
//
//		p.velocity = glm::vec3(0.0f);
//		p.mass = 1;
//		p.delta_p = glm::vec3(0.0f);
//		p.rho = 0.0;
//		p.C = 1;
//		p.pred_position = glm::vec3(0.0f);
//		p.lambda = 0.0f;
//		
//		particles.push_back(p);
//	}
//};

void InitParticleList()
{
	particles.clear();
	//start positioning particles at some distance from the left and bottom walls
	float x_ini_pos = 0.01f + g_xmin;
	float y_ini_pos = 0.0000001f + g_ymin;
	float z_ini_pos = 0.1f + g_zmin;

	// deltas for particle distribution
	float d_x = 0.05f;
	float d_y = 0.05f;
	float d_z = 0.05f;

	printf("Number of particles in the simulation: %i.\n", PARTICLE_COUNT_X * PARTICLE_COUNT_Y * PARTICLE_COUNT_Z);

	float x_pos = x_ini_pos;
	particles.reserve(PARTICLE_COUNT_X*PARTICLE_COUNT_Y*PARTICLE_COUNT_Z);

	for (unsigned int x = 0; x < PARTICLE_COUNT_X; x++)
	{
		float y_pos = y_ini_pos;

		for (unsigned int y = 0; y < PARTICLE_COUNT_Y; y++)
		{
			float z_pos = z_ini_pos;

			for (unsigned int z = 0; z < PARTICLE_COUNT_Z; z++)
			{
				Particle p;

				float r = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) / 100.0f;

				//float v = ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) -0.5f) / 100.0f;

				
				p.position.x = x_pos + r;
				p.position.y = y_pos + r;
				p.position.z = z_pos + r;

				p.velocity = glm::vec3(0.0f);
				p.mass = 1;
				p.delta_p = glm::vec3(0.0f);
				p.rho = 0.0;
				p.C = 1;
				p.pred_position = glm::vec3(0.0f);
				p.lambda = 0.0f;

				particles.push_back(p);
				z_pos += d_z;
			}
			y_pos += d_y;
		}
		x_pos += d_x;
	}
}

void teardrop()
{
	//start positioning particles at some distance from the left and bottom walls
	float x_ini_pos = 0.2;
	float y_ini_pos = 2;
	float z_ini_pos = 0.2;

	// deltas for particle distribution
	float d_x = 0.05f;
	float d_y = 0.05f;
	float d_z = 0.05f;

	float x_pos = x_ini_pos;

	for (unsigned int x = 0; x < 2; x++)
	{
		float y_pos = y_ini_pos;

		for (unsigned int y = 0; y < 2; y++)
		{
			float z_pos = z_ini_pos;

			for (unsigned int z = 0; z < 2; z++)
			{
				Particle p;

				float r = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) / 100.0f;

				//float v = ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) -0.5f) / 100.0f;


				p.position.x = x_pos + r;
				p.position.y = y_pos + r;
				p.position.z = z_pos + r;

				p.velocity = glm::vec3(0.0f);
				p.mass = 1;
				p.delta_p = glm::vec3(0.0f);
				p.rho = 0.0;
				p.C = 0;
				p.pred_position = glm::vec3(0.0f);
				p.lambda = 0.0f;

				particles.push_back(p);
				z_pos += d_z;
			}
			y_pos += d_y;
		}
		x_pos += d_x;
	}
}

float wPoly6(glm::vec3 r, float h) {
	float dot_rr = glm::dot(r, r);
	float h2 = h*h;
	float h4 = h2*h2;
	float h2_dot_rr = h2 - dot_rr;

	if (length(r) <= h)
		return 315.0f / (64.0f * 3.1415f * h4*h4*h) * h2_dot_rr * h2_dot_rr * h2_dot_rr;
	else
		return 0.0f;
}

glm::vec3 wSpiky(glm::vec3 r, float h) {
	float spiky = 45.0f / (PI * pow(h, 6));
	float rlen = glm::length(r);
	if (rlen >= h)
		return glm::vec3(0.0f);
	float coeff = (h - rlen) * (h - rlen);
	coeff *= spiky;
	coeff /= rlen;
	return r * -coeff;
}

float DensityEstimator(Particle p, std::vector<Particle> predict_p) {
	float rho = 0.0;
	int neighborsize = p.neighbors.size();
	for (int j = 0 ; j < neighborsize ; j++) {
		glm::vec3 r = p.position - predict_p[p.neighbors[j]].position;
		rho += predict_p[p.neighbors[j]].mass * wPoly6(r, g_h);
	}
	return rho;
}

float NablaCSquaredSumFunction(Particle p, std::vector<Particle> predict_p) {

	std::vector<glm::vec3> NablaC;
	float res = 0.0f;
	int neighborsize = p.neighbors.size();

	if (neighborsize > 0) {
		for (int j = 0; j < neighborsize; j++) {													//for k != i
			glm::vec3 r = p.position - predict_p[p.neighbors[j]].position;
			glm::vec3 nablac= -wSpiky(r, g_h) / REST;
			NablaC.push_back(nablac);
		}

		NablaC.push_back(glm::vec3(0.0f));																	//for k = i
		int last = NablaC.size()-1; 

		for (int j = 0; j < neighborsize; j++) {
			glm::vec3 r = p.position - predict_p[p.neighbors[j]].position;
			NablaC[last] = NablaC[last] + wSpiky(r, g_h);
		}
		NablaC[last] = NablaC[last] / REST;

		for (int k = 0 ; k < NablaC.size() ; k++) {
			float norm_nablac = length(NablaC[k]);
			res += norm_nablac * norm_nablac;
		}
	}

	return res;
}

glm::vec3 CalculateDp(Particle p, std::vector<Particle> predict_p) {
	glm::vec3 res = glm::vec3(0.0f);
	int neighborsize = p.neighbors.size();
	float dqMag = g_dq * g_h;
	float kpoly = 315.0f / (64.0f * PI * pow(g_h, 9));
	float wQH = kpoly * pow((g_h * g_h - dqMag * dqMag), 3);
	for (int j = 0; j < neighborsize; j++) {
		glm::vec3 r = p.position - predict_p[p.neighbors[j]].position;
		float corr = wPoly6(r, g_h) / wQH;
		corr *= corr * corr * corr;
		float scorr =  -g_k * corr;
		float lambdaSum = p.lambda + predict_p[p.neighbors[j]].lambda;
		res += (lambdaSum + scorr) * wSpiky(r, g_h);
	}

	res = res / REST;
	return res;
}

glm::vec3 eta(Particle p, std::vector<Particle> predict_p, float &vorticityMag) {
	glm::vec3 eta = glm::vec3(0.0f);
	int neighborsize = p.neighbors.size();
	if (neighborsize > 0) {
		for (int j = 0; j < neighborsize; j++) {
				glm::vec3 r = p.position - predict_p[p.neighbors[j]].position;
				eta += wSpiky(r, g_h) * vorticityMag;
		}
	}

	return eta;
}

glm::vec3 VorticityConfinement(Particle p, std::vector< Particle > &p_list) {
	glm::vec3 omega = glm::vec3(0.0f);
	float vorticityEps = 0.0001f;
	unsigned int num_neighbors = p.neighbors.size();

	for (unsigned int k = 0; k < num_neighbors; k++) {
		glm::vec3 v_ij(p_list[p.neighbors[k]].velocity - p.velocity);
		glm::vec3 r(p.position - p_list[p.neighbors[k]].position);
		omega += glm::cross(v_ij, wSpiky(r, g_h));
	}

	float omegaLength = glm::length(omega);
	if (omegaLength == 0.0f) {
		//No direction for eta
		return glm::vec3(0.0f);
	}

	glm::vec3 etaVal = eta(p, p_list, omegaLength);
	if (etaVal.x == 0 && etaVal.y == 0 && etaVal.z == 0) {
		//Particle is isolated or net force is 0
		return glm::vec3(0.0f);
	}

	glm::vec3 n = normalize(etaVal);

	return (glm::cross(n, omega) * vorticityEps);
}

glm::vec3 XSPHViscosity(Particle p, std::vector< Particle > &p_list)
{
	float c = 0.01f;
	unsigned int num_neighbors = p.neighbors.size();
	glm::vec3 visc = glm::vec3(0.0f);
	if (num_neighbors > 0) {
		for (unsigned int k = 0; k < num_neighbors; k++)
		{
			glm::vec3 v_ij(p_list[p.neighbors[k]].velocity - p.velocity);
			glm::vec3 r(p.position - p_list[p.neighbors[k]].position);

			visc += v_ij * wPoly6(r, g_h);
		}
	}
	return visc * c;
}

//int ComputeHash(int &grid_x, int &grid_y, int &grid_z)
//{
//	return grid_x + grid_y * GRID_RESOLUTION;
//}
//
////------------------------------------------------------------------------------
//void BuildHashTable(std::vector< Particle > &p_list, Hash &hash_table)
//{
//	unsigned int num_particles = p_list.size();
//	int grid_x;
//	int grid_y;
//	int grid_z;
//
//	float cell_size = (g_xmax - g_ymin) / GRID_RESOLUTION;
//
//	hash_table.clear();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//	{
//		grid_x = floor(p_list[i].position[0] / cell_size);
//		grid_y = floor(p_list[i].position[1] / cell_size);
//		grid_z = floor(p_list[i].position[2] / cell_size);
//
//		p_list[i].hash = ComputeHash(grid_x, grid_y, grid_z);
//
//		hash_table.insert(Hash::value_type(p_list[i].hash, i));
//	}
//}
//
////------------------------------------------------------------------------------
//void SetUpNeighborsLists(std::vector< Particle > &p_list, Hash &hash_table)
//{
//	unsigned int num_particles = p_list.size();
//
//	float cell_size = (g_xmax - g_ymin) / GRID_RESOLUTION;
//
//	int x_idx;
//	int y_idx;
//	int z_idx;
//
//	int grid_x_min;
//	int grid_y_min;
//	int grid_x_max;
//	int grid_y_max;
//	int grid_z_max;
//	int grid_z_min;
//
//	int hash;
//
//	for (unsigned int i = 0; i < num_particles; i++)
//	{
//		p_list[i].neighbors.clear();
//
//		grid_x_min = floor((p_list[i].position[0] - g_h) / cell_size);
//		grid_y_min = floor((p_list[i].position[1] - g_h) / cell_size);
//		grid_z_min = floor((p_list[i].position[2] - g_h) / cell_size);
//
//		grid_x_max = floor((p_list[i].position[0] + g_h) / cell_size);
//		grid_y_max = floor((p_list[i].position[1] + g_h) / cell_size);
//		grid_z_max = floor((p_list[i].position[2] + g_h) / cell_size);
//
//		for (z_idx = grid_z_min; z_idx <= grid_z_max; z_idx++){
//			for (y_idx = grid_y_min; y_idx <= grid_y_max; y_idx++){
//				for (x_idx = grid_x_min; x_idx <= grid_x_max; x_idx++) {
//					hash = ComputeHash(x_idx, y_idx, z_idx);
//					auto its = hash_table.equal_range(hash);
//
//					for (auto it = its.first; it != its.second; ++it)
//					if (it->second != i)
//					if (length(p_list[i].position - p_list[it->second].position) <= g_h)
//						p_list[i].neighbors.push_back(it->second);
//				}
//			}
//		}
//	}
//}


void Algorithm() {
	if (gota == 1){
		teardrop();
		gota = 2;
	}

	int npart = particles.size();
	std::vector<Particle> predict_p = particles;
	
	for (int i = 0; i < npart ; i++) {
		predict_p[i].velocity = particles[i].velocity + DT * gravity;
		predict_p[i].position = particles[i].position + DT * predict_p[i].velocity;
	}
	
	for (int ineigh = 0; ineigh < npart; ineigh++) {
		for (int j = 0; j < npart; j++) {
			if (ineigh != j) {
				glm::vec3 r = predict_p[ineigh].position - predict_p[j].position;
				if(glm::length(r) <= g_h) {
					predict_p[ineigh].neighbors.push_back(j);
				}
			}
		}
	}
	//BuildHashTable(predict_p_list, hash_table);
	//SetUpNeighborsLists(predict_p_list, hash_table);


	int iter = 0;
	while(iter < ITER) {
		for (int i = 0; i < npart; i++) {
			predict_p[i].rho = DensityEstimator(predict_p[i], predict_p);
			predict_p[i].C = predict_p[i].rho / REST - 1;
			float sumNabla = NablaCSquaredSumFunction(predict_p[i], predict_p);
			predict_p[i].lambda = - predict_p[i].C / (sumNabla + EPSILON);
		}
		
		for (int i = 0; i < npart; i++) {
			predict_p[i].delta_p = CalculateDp(predict_p[i], predict_p);
			
			if( predict_p[i].position.z < g_zmin){
				predict_p[i].position.z = g_zmin;
				/*glm::vec3 normal = glm::vec3(0,0,1);
				predict_p[i].velocity.z = glm::reflect(predict_p[i].velocity, normal).z * DT;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			if( predict_p[i].position.z > g_zmax){
				predict_p[i].position.z = g_zmax;
				/*glm::vec3 normal = glm::vec3(0,0,-1);
				predict_p[i].velocity.z = glm::reflect(predict_p[i].velocity, normal).z * DT;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			if( predict_p[i].position.y < g_ymin){
				predict_p[i].position.y = g_ymin;
				/*glm::vec3 normal = glm::vec3(0,1,0);
				predict_p[i].velocity.y = glm::reflect(predict_p[i].velocity, normal).y * DT;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			
			if( predict_p[i].position.x < g_xmin){
				predict_p[i].position.x = g_xmin;
				/*glm::vec3 normal = glm::vec3(1,0,0);
				predict_p[i].velocity.x = glm::reflect(predict_p[i].velocity, normal).x * DT;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
				
			}
			if( predict_p[i].position.x > g_xmax){
				predict_p[i].position.x = g_xmax;
				/*glm::vec3 normal = glm::vec3(-1,0,0);
				predict_p[i].velocity.x = glm::reflect(predict_p[i].velocity, normal).x * DT;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
				
			}	
		}

		for (int i = 0; i < npart; i++) {
			predict_p[i].position = predict_p[i].position + predict_p[i].delta_p;
		}

		iter++;
	}

	for (int i = 0; i < npart; i++) {
		predict_p[i].velocity = (1/DT) * (predict_p[i].position - particles[i].position);
		/*predict_p[i].velocity += VorticityConfinement(predict_p[i], predict_p) * DT;
		predict_p[i].delta_p += XSPHViscosity(predict_p[i], predict_p);*/
		predict_p[i].neighbors.clear();
	}

	particles = predict_p;
}


int main(void)
{
	
	int nUseMouse = 0;
	InitParticleList();
	/*initializeParticles();*/
	

	// Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
	g_pWindow = glfwCreateWindow(g_nWidth, g_nHeight, "CG UFFS", NULL, NULL);
	if (g_pWindow == NULL){
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(g_pWindow);

	// Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		return -1;
	}

	check_gl_error();//OpenGL error from GLEW

	// Initialize the GUI
	TwInit(TW_OPENGL_CORE, NULL);
	TwWindowSize(g_nWidth, g_nHeight);

	// Set GLFW event callbacks. I removed glfwSetWindowSizeCallback for conciseness
	glfwSetMouseButtonCallback(g_pWindow, (GLFWmousebuttonfun)TwEventMouseButtonGLFW); // - Directly redirect GLFW mouse button events to AntTweakBar
	glfwSetCursorPosCallback(g_pWindow, (GLFWcursorposfun)TwEventMousePosGLFW);          // - Directly redirect GLFW mouse position events to AntTweakBar
	glfwSetScrollCallback(g_pWindow, (GLFWscrollfun)TwEventMouseWheelGLFW);    // - Directly redirect GLFW mouse wheel events to AntTweakBar
	glfwSetKeyCallback(g_pWindow, (GLFWkeyfun)TwEventKeyGLFW);                         // - Directly redirect GLFW key events to AntTweakBar
	glfwSetCharCallback(g_pWindow, (GLFWcharfun)TwEventCharGLFW);                      // - Directly redirect GLFW char events to AntTweakBar
	glfwSetWindowSizeCallback(g_pWindow, WindowSizeCallBack);

	//create the toolbar
	g_pToolBar = TwNewBar("CG UFFS ToolBar");
	// Add 'speed' to 'bar': it is a modifable (RW) variable of type TW_TYPE_DOUBLE. Its key shortcuts are [s] and [S].
	double speed = 0.0;
	TwAddVarRW(g_pToolBar, "speed", TW_TYPE_DOUBLE, &speed, " label='Rot speed' min=0 max=2 step=0.01 keyIncr=s keyDecr=S help='Rotation speed (turns/second)' ");
	// Add 'bgColor' to 'bar': it is a modifable variable of type TW_TYPE_COLOR3F (3 floats color)
	vec3 oColor(0.0f);
	TwAddVarRW(g_pToolBar, "bgColor", TW_TYPE_COLOR3F, &oColor[0], " label='Background color' ");
	TwAddVarRW(g_pToolBar, "g_h", TW_TYPE_FLOAT, &g_h, " label='H radius' min=0.1 max=5 step=0.01 keyIncr=h keyDecr=H help='Rotation speed (turns/second)' ");
	/*TwAddVarRW(g_pToolBar, "rotatey", TW_TYPE_FLOAT, &rotatey, " label='rotation y of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "rotatex", TW_TYPE_FLOAT, &rotatex, " label='rotation x of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "rotatez", TW_TYPE_FLOAT, &rotatez, " label='rotation y of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");*/
	TwAddVarRW(g_pToolBar, "g_zmin", TW_TYPE_FLOAT, &g_zmin, " label='position z of wall' min=-13 max=13 step=0.05 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "g_xmax", TW_TYPE_FLOAT, &g_xmax, " label='position x of wall' min=-40 max=40 step=0.01 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "g_k", TW_TYPE_FLOAT, &g_k, " label='k for scorr' min=-13 max=13 step=0.0001 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "g_dq", TW_TYPE_FLOAT, &g_dq, " label='dq for scorr' min=-13 max=13 step=0.01 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "h_sphere", TW_TYPE_FLOAT, &h_sphere, " label='h_sphere' min=0 max=5 step=0.01 keyIncr=h keyDecr=H help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "gota", TW_TYPE_FLOAT, &gota, " label='gota' min=0 max=2 step=1 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(g_pWindow, GLFW_STICKY_KEYS, GL_TRUE);
	glfwSetCursorPos(g_pWindow, g_nWidth / 2, g_nHeight / 2);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);

	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// Create and compile our GLSL program from the shaders
	GLuint standardProgramID = LoadShaders("shaders/StandardShading.vertexshader", "shaders/StandardShading.fragmentshader");
	/*GLuint wallProgramID = LoadShaders("shaders/wallShading.vertexshader", "shaders/wallShading.fragmentshader");*/
	/*BuildGrid();*/

	// Load the texture
	//GLuint Texture = loadDDS("mesh/uvmap.DDS");

	// Get a handle for our "myTextureSampler" uniform
	//GLuint TextureID = glGetUniformLocation(standardProgramID, "myTextureSampler");

	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	bool res = loadOBJ("mesh/esfera.obj", vertices, uvs, normals);

	std::vector<unsigned short> indices;
	std::vector<glm::vec3> indexed_vertices;
	std::vector<glm::vec2> indexed_uvs;
	std::vector<glm::vec3> indexed_normals;
	indexVBO(vertices, uvs, normals, indices, indexed_vertices, indexed_uvs, indexed_normals);

	// Load it into a VBO

	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3), &indexed_vertices[0], GL_STATIC_DRAW);

	//paredes

	/*std::vector<glm::vec3> wall_vertices;
	std::vector<glm::vec2> wall_uvs;
	std::vector<glm::vec3> wall_normals;
	bool wallres = loadOBJ("mesh/esfera.obj", wall_vertices, wall_uvs, wall_normals);*/

	//std::vector<unsigned short> wall_indices;
	//std::vector<glm::vec3> wall_indexed_vertices;
	//std::vector<glm::vec2> wall_indexed_uvs;
	//std::vector<glm::vec3> wall_indexed_normals;
	//indexVBO(wall_vertices, wall_uvs, wall_normals, wall_indices, wall_indexed_vertices, wall_indexed_uvs, wall_indexed_normals);

	//// Load it into a VBO

	//GLuint wallbuffer;
	//glGenBuffers(1, &wallbuffer);
	//glBindBuffer(GL_ARRAY_BUFFER, wallbuffer);
	//glBufferData(GL_ARRAY_BUFFER, wall_indexed_vertices.size() * sizeof(glm::vec3), &wall_indexed_vertices[0], GL_STATIC_DRAW);

	//GLuint wallnormalbuffer;
	//glGenBuffers(1, &wallnormalbuffer);
	//glBindBuffer(GL_ARRAY_BUFFER, wallnormalbuffer);
	//glBufferData(GL_ARRAY_BUFFER, wall_indexed_normals.size() * sizeof(glm::vec3), &wall_indexed_normals[0], GL_STATIC_DRAW);

	//GLuint wallelementbuffer;
	//glGenBuffers(1, &wallelementbuffer);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wallelementbuffer);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, wall_indices.size() * sizeof(unsigned short), &wall_indices[0], GL_STATIC_DRAW);

	/*GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_uvs.size() * sizeof(glm::vec2), &indexed_uvs[0], GL_STATIC_DRAW);*/

	GLuint normalbuffer;
	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_normals.size() * sizeof(glm::vec3), &indexed_normals[0], GL_STATIC_DRAW);

	// Generate a buffer for the indices as well
	GLuint elementbuffer;
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);

	// Get a handle for our "LightPosition" uniform
	glUseProgram(standardProgramID);

	GLuint LightID = glGetUniformLocation(standardProgramID, "LightPosition_worldspace");

	// For speed computation
	double lastTime = glfwGetTime();
	int nbFrames    = 0;

	do {   
		
		check_gl_error();

        //use the control key to free the mouse
		if (glfwGetKey(g_pWindow, GLFW_KEY_LEFT_CONTROL) != GLFW_PRESS)
			nUseMouse = 1;
		else
			nUseMouse = 0;

		// Measure speed
		double currentTime = glfwGetTime();
		nbFrames++;
		if (currentTime - lastTime >= 1.0){ // If last prinf() was more than 1sec ago
			// printf and reset
			printf("%f ms/frame\n", 1000.0 / double(nbFrames));
			nbFrames  = 0;
			lastTime += 1.0;
		}


		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Compute the MVP matrix from keyboard and mouse input
		computeMatricesFromInputs(nUseMouse, g_nWidth, g_nHeight);
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix       = getViewMatrix();

		// Use our shader
		GLuint MatrixID = glGetUniformLocation(standardProgramID, "MVP");
		glm::mat4 MVP				= ProjectionMatrix * ViewMatrix;
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		check_gl_error();

		glUseProgram(standardProgramID);
		MatrixID = glGetUniformLocation(standardProgramID, "MVP");
		GLuint ViewMatrixID		 = glGetUniformLocation(standardProgramID, "V");
		GLuint ModelMatrixID	 = glGetUniformLocation(standardProgramID, "M");


		glm::vec3 lightPos = glm::vec3(0, 100, 0);
		glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

		// Bind our texture in Texture Unit 0
		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, Texture);
		//// Set our "myTextureSampler" sampler to user Texture Unit 0
		//glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
		glVertexAttribPointer(
			0,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);
		// 2nd attribute buffer : UVs
		//glEnableVertexAttribArray(1);
		//glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		//glVertexAttribPointer(
		//	1,                                // attribute
		//	2,                                // size
		//	GL_FLOAT,                         // type
		//	GL_FALSE,                         // normalized?
		//	0,                                // stride
		//	(void*)0                          // array buffer offset
		//	);

		// 3rd attribute buffer : normals
		glEnableVertexAttribArray(2);
		glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
		glVertexAttribPointer(
			2,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
			);

		// Index buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
		
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
	
		glm::mat4 ModelMatrix      = glm::mat4(1.0f); //Usar posição aleatória
		ModelMatrix[0][0]		   = h_sphere; //Escala do modelo (x)
		ModelMatrix[1][1]		   = h_sphere; //Escala do modelo (y)
		ModelMatrix[2][2]		   = h_sphere; //Escala do modelo (z)

		//simulação
		
		Algorithm();
		
		//Render
		
		for(int teste = 0 ; teste < particles.size() ; teste++) {
		//for
			
			ModelMatrix[3][0]		   = particles[teste].position.x; //posição x
			ModelMatrix[3][1]		   = particles[teste].position.y; //posição y
			ModelMatrix[3][2]		   = particles[teste].position.z; //posição 

			glm::mat4 MVP              = ProjectionMatrix * ViewMatrix * ModelMatrix;

			// Send our transformation to the currently bound shader,
			// in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
			glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);


			// Draw the triangles !
			glDrawElements(
				GL_TRIANGLES,        // mode
				indices.size(),      // count
				GL_UNSIGNED_SHORT,   // type
				(void*)0             // element array buffer offset
				);
		//endfor
		}

		/*glUseProgram(wallProgramID);
		MatrixID				 = glGetUniformLocation(standardProgramID, "MVP");
		ViewMatrixID			 = glGetUniformLocation(standardProgramID, "V");
		ModelMatrixID			 = glGetUniformLocation(standardProgramID, "M");*/

		// Bind our texture in Texture Unit 0
		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, Texture);
		//// Set our "myTextureSampler" sampler to user Texture Unit 0
		//glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		//glEnableVertexAttribArray(0);
		//glBindBuffer(GL_ARRAY_BUFFER, wallbuffer);
		//glVertexAttribPointer(
		//	0,                  // attribute
		//	3,                  // size
		//	GL_FLOAT,           // type
		//	GL_FALSE,           // normalized?
		//	0,                  // stride
		//	(void*)0            // array buffer offset
		//	);
		// 2nd attribute buffer : UVs
		//glEnableVertexAttribArray(1);
		//glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
		//glVertexAttribPointer(
		//	1,                                // attribute
		//	2,                                // size
		//	GL_FLOAT,                         // type
		//	GL_FALSE,                         // normalized?
		//	0,                                // stride
		//	(void*)0                          // array buffer offset
		//	);

		// 3rd attribute buffer : normals
		//glEnableVertexAttribArray(2);
		//glBindBuffer(GL_ARRAY_BUFFER, wallnormalbuffer);
		//glVertexAttribPointer(
		//	2,                                // attribute
		//	3,                                // size
		//	GL_FLOAT,                         // type
		//	GL_FALSE,                         // normalized?
		//	0,                                // stride
		//	(void*)0                          // array buffer offset
		//	);

		// Index buffer
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wallelementbuffer);
		//
		//glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
	
		//ModelMatrix				   = glm::mat4(1.0f); //Usar posição aleatória
		//ModelMatrix[0][0]		   = 0; //Escala do modelo (x)
		//ModelMatrix[1][1]		   = 0; //Escala do modelo (y)
		//ModelMatrix[2][2]		   = 0; //Escala do modelo (z)
		//
		////Render
		//	
		//ModelMatrix[3][0]		   = -g_xmax; //posição x
		//ModelMatrix[3][1]		   = -g_ymax;//posição y
		//ModelMatrix[3][2]		   = g_zmin;//posição z

		//	

		//ModelMatrix = glm::rotate(ModelMatrix, rotatex, glm::vec3(1, 0, 0));
		//ModelMatrix = glm::rotate(ModelMatrix, rotatey, glm::vec3(0, 1, 0));
		//ModelMatrix = glm::rotate(ModelMatrix, rotatez, glm::vec3(0, 0, 1));

		//MVP						   = ProjectionMatrix * ViewMatrix * ModelMatrix;

		//// Send our transformation to the currently bound shader,
		//// in the "MVP" uniform
		//glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		//glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);


		//// Draw the triangles !
		//glDrawElements(
		//	GL_TRIANGLES,				// mode
		//	wall_indices.size(),		// count
		//	GL_UNSIGNED_SHORT,			// type
		//	(void*)0					// element array buffer offset
		//		);
		////endfor

		glDisableVertexAttribArray(0);
		// glDisableVertexAttribArray(1); Canal das texturas
		glDisableVertexAttribArray(2);

		// Draw tweak bars
		TwDraw();
		
		// Swap buffers
		glfwSwapBuffers(g_pWindow);
		glfwPollEvents();
	

	} // Check if the ESC key was pressed or the window was closed
	while (glfwGetKey(g_pWindow, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
	glfwWindowShouldClose(g_pWindow) == 0);

	
	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	//glDeleteBuffers(1, &uvbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteBuffers(1, &elementbuffer);
	glDeleteProgram(standardProgramID);
	//glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Terminate AntTweakBar and GLFW
	TwTerminate();
	glfwTerminate();

	return 0;
}

