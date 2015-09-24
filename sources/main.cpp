// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <time.h>

#define PI 3.1415f
#define EPSILON  600.0f
#define ITER 2
#define NPART 1000
#define H 0.1f
#define REST 6378.0f
#define DT 0.0083f
#define BOUNCE 0.01f
#define PARTICLE_COUNT_X 3
#define PARTICLE_COUNT_Y 2
#define PARTICLE_COUNT_Z 3
#define GRID_RESOLUTION 1
#define DOMAIN_MIN_X 0.0f
#define DOMAIN_MAX_X 13.0f
#define DOMAIN_MIN_Y 0.0f
#define DOMAIN_MAX_Y 13.0f


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

typedef std::unordered_multimap< int, int > Hash;
Hash hash_table;

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
	  /*float hash;*/
	  std::vector<Particle> neighbors;
};

std::vector<Particle> particles;
std::vector< Particle > predict_p_list;
std::vector<float> g_grid;
float g_xmax = 11.5;
float g_xmin = 0;
float g_ymax = 11.5;
float g_ymin = 0;
float g_zmax = 11.5;
float g_zmin = 0;
float h_sphere = 1.05;
float g_h = 0.2;
float POW_H_9 =(g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h); // h^9
float POW_H_6 =(g_h*g_h*g_h*g_h*g_h*g_h); // h^6
//float wall = 10;
//time_t g_initTime = time(0);
//float rotatey = 0;
//float rotatex = 0;
//float rotatez = 0;
float g_k = 0.00001f;
float g_dq = 2.0f;
float dqMag = g_dq * g_h;
float kpoly = 315.0f / (64.0f * PI * pow(g_h, 9));
float wQH = kpoly * pow((g_h * g_h - dqMag * dqMag), 3);
int npart = PARTICLE_COUNT_X*PARTICLE_COUNT_Y*PARTICLE_COUNT_Z;

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
	//start positioning particles at some distance from the left and bottom walls
	float x_ini_pos = 1.0f + g_xmin;
	float y_ini_pos = 0.1f + g_ymin;
	float z_ini_pos = 1.0f + g_zmin;

	// deltas for particle distribution
	float d_x = 1.04f;
	float d_y = 1.04f;
	float d_z = 1.04f;

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

				//p.position = glm::vec3(, y_pos + r, z_pos + r);
				//p.velocity = glm::vec3(0.0f);
				///*p.vel     = glm::vec2(0.5f, 0.5f);*/

				//p.mass = 1.0f;
				//p.delta_p = glm::vec3(0.0f);
				//p.lambda = 0.0f;
				//p.rho = 0.0f;
				//p.C = 1.0f;
				//p.neighbors.clear();
				///*p.hash = 0;*/

				//particles.push_back(p);

				z_pos += d_z;
			}
			y_pos += d_y;
		}
		x_pos += d_x;
	}

	//// copies p_list particle data to predict_p_list
	//unsigned int num_particles = p_list.size();

	//for (unsigned int i = 0; i < num_particles; i++)
	//	predict_p_list.push_back(p_list[i]);

	//printf("--> # of particles : %i\n", p_list.size());
}

//void InitParticleList()
//{
//	//start positioning particles at some distance from the left and bottom walls
//	float x_ini_pos = 1.0f + g_xmin;
//	float y_ini_pos = 1.0f + g_ymin;
//	float z_ini_pos = 1.0f + g_zmin;
//
//	// deltas for particle distribution
//	float d_x = 2.04f;
//	float d_y = 2.04f;
//	float d_z = 2.04f;
//	printf("Number of particles in the simulation: %i.\n", PARTICLE_COUNT_X * PARTICLE_COUNT_Y * PARTICLE_COUNT_Z);
//
//	float x_pos = x_ini_pos;
//
//	for (unsigned int x = 0; x < PARTICLE_COUNT_X; x++)
//	{
//		float y_pos = y_ini_pos;
//
//		for (unsigned int y = 0; y < PARTICLE_COUNT_Y; y++)
//		{
//			float z_pos = z_ini_pos;
//			for (unsigned int y = 0; y < PARTICLE_COUNT_Y; y++)
//			{
//				
//				Particle p;
//
//				float r = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) / 100.0f;
//
//				//float v = ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) -0.5f) / 100.0f;
//
//				p.position = glm::vec3(x_pos + r, y_pos + r, z_pos + r);
//				p.velocity = glm::vec3(0.0f);
//				/*p.vel     = glm::vec2(0.5f, 0.5f);*/
//
//				p.mass = 1.0f;
//				p.delta_p = glm::vec3(0.0f);
//				p.lambda = 0.0f;
//				p.rho = 0.0f;
//				p.C = 1.0f;
//				p.neighbors.clear();
//				p.hash = 0;
//
//				particles.push_back(p);
//
//				z_pos += d_z;
//			}
//			y_pos += d_y;
//		}
//		x_pos += d_x;
//	}
//	unsigned int num_particles = particles.size();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//		predict_p_list.push_back(particles[i]);
//
//	printf("--> # of particles : %i\n", particles.size());
//}
//
//int ComputeHash(int &grid_x, int &grid_y, int &grid_z)
//{
//	return (grid_x + grid_y + grid_z) * GRID_RESOLUTION;
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
//	float cell_size = (DOMAIN_MAX_X - DOMAIN_MIN_X) / GRID_RESOLUTION;
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
//	float cell_size = (DOMAIN_MAX_X - DOMAIN_MIN_X) / GRID_RESOLUTION;
//
//	int x_idx;
//	int y_idx;
//	int z_idx;
//
//	int grid_x_min;
//	int grid_y_min;
//	int grid_x_max;
//	int grid_y_max;
//	int grid_z_min;
//	int grid_z_max;
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
//		for (z_idx = grid_z_min; z_idx <= grid_z_max; z_idx++) {
//			for (y_idx = grid_y_min; y_idx <= grid_y_max; y_idx++) {
//				for (x_idx = grid_x_min; x_idx <= grid_x_max; x_idx++) {
//					hash = ComputeHash(x_idx, y_idx, y_idx);
//					auto its = hash_table.equal_range(hash);
//
//					for (auto it = its.first; it != its.second; ++it)
//						if (it->second != i)
//							if (length(p_list[i].position - p_list[it->second].position) <= g_h)
//								p_list[i].neighbors.push_back(it->second);
//				}
//			}
//		}
//	}
//}
//
////------------------------------------------------------------------------------
//// Poly6 function
//float WP(glm::vec3 &r, float h)
//{
//	float dot_rr = glm::dot(r, r);
//	float h2 = h*h;
//	float h4 = h2*h2;
//	float h2_dot_rr = h2 - dot_rr;
//
//	if (length(r) <= H)
//		return 315.0f / (64.0f * 3.1415f * h4*h4*h) * h2_dot_rr * h2_dot_rr * h2_dot_rr;
//	else
//		return 0.0f;
//}
//
////------------------------------------------------------------------------------
//// Spiky function
//glm::vec3 NablaW(glm::vec3 &r, float h)
//{
//	float spiky = 45.0f / (3.1415f * pow(h, 6));
//	float rlen = glm::length(r);
//	if (rlen >= h)
//		return glm::vec3(0.0f);
//	float coeff = (h - rlen) * (h - rlen);
//	coeff *= spiky;
//	coeff /= rlen;
//	return r * -coeff;
//	/*float norm_r = length(r);
//	float h2 = h*h;
//	float h_norm_r = h - norm_r;
//
//	if (norm_r <= H)
//	return -45.0f / (3.1415f * h2*h2*h2) * h_norm_r *  h_norm_r * (r/(norm_r + EPSILON));
//	else
//	return glm::vec2(0.0f, 0.0f);*/
//}
//
////------------------------------------------------------------------------------
//void DensityEstimator(std::vector< Particle > &p_list, int i)
//{
//	p_list[i].rho = 0.0f;
//	glm::vec3 r;
//
//	int neighbor_count = p_list[i].neighbors.size();
//
//	for (int k = 0; k < neighbor_count; k++)
//	{
//		r = p_list[i].position - p_list[p_list[i].neighbors[k]].position;
//		p_list[i].rho += p_list[p_list[i].neighbors[k]].mass * WP(r, H);
//	}
//}
//
////------------------------------------------------------------------------------
//float NablaCSquaredSumFunction(std::vector< Particle > &p_list, int i)
//{
//	unsigned int num_neighbors = p_list[i].neighbors.size();
//	glm::vec3 r;
//	std::vector<glm::vec3> NablaC;
//
//	//glm::vec2 NablaC[num_neighbors + 1];
//	float res = 0.0f;
//
//	if (num_neighbors > 0)
//	{
//		for (unsigned int k = 0; k < num_neighbors; k++)						// for k != i
//		{
//			r = p_list[i].position - p_list[p_list[i].neighbors[k]].position;
//			glm::vec3 nablac = -NablaW(r, H) / REST;
//			NablaC.push_back(nablac);
//		}
//
//		NablaC.push_back(glm::vec3(0.0f));																	//for k = i
//		int last = NablaC.size() - 1;
//		//NablaC[num_neighbors] = glm::vec2(0.0f, 0.0f);										// for k = i
//
//		for (unsigned int k = 0; k < num_neighbors; k++)						// for k != i
//		{
//			r = p_list[i].position - p_list[p_list[i].neighbors[k]].position;
//			/*NablaC[num_neighbors] += NablaW(r, H);*/
//			NablaC[last] = NablaC[last] + NablaW(r, H);
//		}
//		NablaC[last] = NablaC[last] / REST;
//		/*NablaC[num_neighbors] = NablaC[num_neighbors] / RHO_REST;*/
//
//		for (unsigned int k = 0; k < num_neighbors + 1; k++)					// for k != i
//		{
//			float norm_nablac = length(NablaC[k]);
//			res += norm_nablac * norm_nablac;
//		}
//	}
//
//	return res;
//}
//
////------------------------------------------------------------------------------
////glm::vec3 vmin(glm::vec3 a, glm::vec3 b, glm::vec3 c)
////{
////	return glm::vec2(std::min(a[0], b[0]), std::min(a[1], b[1]));
////}
////
//////------------------------------------------------------------------------------
////glm::vec2 vmax(glm::vec2 a, glm::vec2 b)
////{
////	return glm::vec2(std::max(a[0], b[0]), std::max(a[1], b[1]));
////}
//
////------------------------------------------------------------------------------
//void CalculateDp(std::vector< Particle > &p_list)
//{
//	unsigned int num_particles = p_list.size();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//	{
//		unsigned int num_neighbors = p_list[i].neighbors.size();
//
//		glm::vec3 r;
//		glm::vec3 res(0.0f);
//
//		for (unsigned int k = 0; k < num_neighbors; k++)
//		{
//			r = p_list[i].position - p_list[p_list[i].neighbors[k]].position;
//
//			float corr = WP(r, H) / wQH;
//			corr *= corr * corr * corr;
//			float scorr = -g_k * corr;
//			/*float scorr = 0.0f;*/
//
//			//glm::vec2 dq = 0.1f * H * glm::vec2(1.0f, 1.0f);
//			//glm::vec2 dq(0.0f, 0.0f);
//
//			//float kk = 0.1f;
//			//float w_ratio = W(r, H) / W(dq, H);
//			//scorr = -kk * w_ratio * w_ratio * w_ratio * w_ratio;
//
//			res += (p_list[i].lambda + p_list[p_list[i].neighbors[k]].lambda/* + scorr*/) * NablaW(r, H);
//		}
//
//		p_list[i].delta_p = res / REST;
//	}
//}
//
////------------------------------------------------------------------------------
//// TODO: finish this function
////void VorticityConfinement(std::vector< Particle > &p_list)
////{
////	unsigned int num_particles = p_list.size();
////
////	for (unsigned int i = 0; i < num_particles; i++)
////	{
////		unsigned int num_neighbors = p_list[i].n.size();
////
////		float omega = 0.0f;
////
////		for (unsigned int k = 0; k < num_neighbors; k++)
////		{
////			glm::vec2 v_ij(p_list[p_list[i].n[k]].vel - p_list[i].vel);
////			glm::vec2 r(p_list[i].pos - p_list[p_list[i].n[k]].pos);
////
////			//omega += cross(v_ij, NablaW(r, H));
////		}
////	}
////}
//
////------------------------------------------------------------------------------
////void XSPHViscosity(std::vector< Particle > &p_list)
////{
////	unsigned int num_particles = p_list.size();
////
////	float c = 0.01f;
////
////	for (unsigned int i = 0; i < num_particles; i++)
////	{
////		unsigned int num_neighbors = p_list[i].n.size();
////
////		glm::vec2 v_new = glm::vec2(0.0f, 0.0f);
////
////		for (unsigned int k = 0; k < num_neighbors; k++)
////		{
////			glm::vec2 v_ik(p_list[p_list[i].n[k]].vel - p_list[i].vel);
////			glm::vec2 r(p_list[i].pos - p_list[p_list[i].n[k]].pos);
////
////			v_new += v_ik * W(r, H);
////		}
////
////
////		v_new *= c;
////
////		//printf("v_new : [%f, %f]\n", v_new[0], v_new[1]);
////
////		v_new += p_list[i].vel;
////
////
////		p_list[i].vel = v_new;
////	}
////}
//
////------------------------------------------------------------------------------
//void CollisionDetectionResponse(std::vector< Particle > &p_list)
//{
//	unsigned int num_particles = p_list.size();
//
//	//glm::vec2 w_min(wall_min_x, wall_min_y);
//	//glm::vec2 w_max(wall_max_x, wall_max_y);
//
//	for (unsigned int i = 0; i < num_particles; i++){
//		if (predict_p_list[i].position.z < g_zmin){
//			predict_p_list[i].position.z = g_zmin + 0.000001f;
//		}
//		if (predict_p_list[i].position.z > g_zmax){
//			predict_p_list[i].position.z = g_zmax - 0.000001f;
//		}
//		if (predict_p_list[i].position.y < g_ymin){
//			predict_p_list[i].position.y = g_ymin + 0.000001f;
//		}
//		if (predict_p_list[i].position.y > g_ymax){
//			predict_p_list[i].position.y = g_ymax - 0.000001f;
//		}
//		if (predict_p_list[i].position.x < g_xmin){
//			predict_p_list[i].position.x = g_xmin + 0.000001f;
//		}
//		if (predict_p_list[i].position.x > g_xmax){
//			predict_p_list[i].position.x = g_xmax - 0.000001f;
//		}
//	}
//}
//
////------------------------------------------------------------------------------
//void UpdatePositions(std::vector< Particle > &p_list)
//{
//	unsigned int num_particles = p_list.size();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//		p_list[i].position += p_list[i].delta_p;
//}
//
////------------------------------------------------------------------------------
//void UpdateVelocity(std::vector< Particle > &predict_p_list, std::vector< Particle > &p_list)
//{
//	unsigned int num_particles = p_list.size();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//		predict_p_list[i].velocity = (1.0f / DT) * (predict_p_list[i].position - p_list[i].position);
//}
//
////------------------------------------------------------------------------------
//void Algorithm(void)
//{
//	unsigned int num_particles = particles.size();
//
//	for (unsigned int i = 0; i < num_particles; i++)
//	{
//		predict_p_list[i].velocity = particles[i].velocity + DT * fext;					// apply forces (line 2)
//		predict_p_list[i].position = particles[i].position + DT * predict_p_list[i].velocity;// predict position (line 3)
//	}
//
//	BuildHashTable(predict_p_list, hash_table);
//	SetUpNeighborsLists(predict_p_list, hash_table);							// find neighbors(line 6)
//
//	unsigned int curr_iteration = 0;
//
//	while (curr_iteration < ITER)
//	{
//		for (unsigned int i = 0; i < num_particles; i++)
//		{
//			DensityEstimator(predict_p_list, i);								// Equation 2
//			predict_p_list[i].C = predict_p_list[i].rho / REST - 1.0f;			// Equation 1
//			float SummationNablaCSquared = NablaCSquaredSumFunction(predict_p_list, i); // Equation 8 and part of the Equation 9
//			predict_p_list[i].lambda = -predict_p_list[i].C / (SummationNablaCSquared + EPSILON); // compute lambda (line 10)
//		}
//
//		CalculateDp(predict_p_list);											// Calculate dp (line 13)
//		CollisionDetectionResponse(predict_p_list);								// Particle-solid collision test and response (line 14)
//		UpdatePositions(predict_p_list);										// Update position (line 17)
//
//		curr_iteration++;
//	}
//
//	UpdateVelocity(predict_p_list, particles);										// Update velocity (line 21)
//	//XSPHViscosity(p_list);														// XSPH viscosity (line 22)
//
//	for (unsigned int i = 0; i < num_particles; i++)
//	{
//		particles[i].velocity = predict_p_list[i].velocity;
//		particles[i].position = predict_p_list[i].position;									// Update positions (line 23)
//		particles[i].mass = predict_p_list[i].mass;
//		particles[i].delta_p = predict_p_list[i].delta_p;
//		particles[i].lambda = predict_p_list[i].lambda;
//		particles[i].rho = predict_p_list[i].rho;
//		particles[i].C = predict_p_list[i].C;
//		particles[i].neighbors = predict_p_list[i].neighbors;
//
//		//printf("[%f, %f]\n", p_list[i].vel[0], p_list[i].vel[1]);
//	}
//
//	//printf("-------------\n");
//}


//IMPLEMENTAÇÃO ANTIGA

float wPoly6(glm::vec3 r, float h) {
	float dot_rr = glm::dot(r, r);
	float h2 = h*h;
	float h4 = h2*h2;
	float h2_dot_rr = h2 - dot_rr;

	if (length(r) <= h)
		return 315.0f / (64.0f * 3.1415f * h4*h4*h) * h2_dot_rr * h2_dot_rr * h2_dot_rr;
	else
		return 0.0f;
	/*float rlen = length(r);
	if (rlen >= h)
		return 0;
	float teste = 315.0 / (64.0 * PI * POW_H_9) * (h*h - glm::dot(r,r)) * (h*h - glm::dot(r,r)) * (h*h - glm::dot(r,r));
	teste = teste;
	return teste;*/
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
	/*float rlen = glm::length(r);
	if (rlen > h || rlen == 0)
		return glm::vec3(0.0f);*/
	//glm::vec3 teste = (45.0f / (PI * POW_H_6)) * (((h - glm::length(r)) * (h - glm::length(r))) * (r/(glm::length(r)+EPSILON)));
	//return teste;
}

float DensityEstimator(Particle p) {
	float rho = 0.0;
	int neighborsize = p.neighbors.size();
	for (int j = 0 ; j < neighborsize ; j++) {
		glm::vec3 r = p.position - p.neighbors[j].position;
		rho += + p.neighbors[j].mass * wPoly6(r, g_h);
	}
	return rho;
}

float NablaCSquaredSumFunction(Particle p) {

	std::vector<glm::vec3> NablaC;
	float res = 0.0f;
	int neighborsize = p.neighbors.size();

	if (neighborsize > 0) {
		for (int j = 0; j < neighborsize; j++) {													//for k != i
			glm::vec3 r = p.position - p.neighbors[j].position;
			glm::vec3 nablac= -wSpiky(r, g_h) / REST;
			NablaC.push_back(nablac);
		}

		NablaC.push_back(glm::vec3(0.0f));																	//for k = i
		int last = NablaC.size()-1; 

		for (int j = 0; j < neighborsize; j++) {
			glm::vec3 r = p.position - p.neighbors[j].position;
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

glm::vec3 CalculateDp(Particle p) {
	glm::vec3 res = glm::vec3(0.0f);
	int neighborsize = p.neighbors.size();
	float g_dq = 0.1*g_h;
	glm::vec3 poly_dq =  g_dq * glm::vec3(1.0f) + p.position;
	for (int j = 0; j < neighborsize; j++) {
		glm::vec3 r = p.position - p.neighbors[j].position;
		float corr = wPoly6(r, g_h) / wQH;
		corr *= corr * corr * corr;
		float scorr =  -g_k * corr;
		float lambdaSum = p.lambda + p.neighbors[j].lambda;
		res += (lambdaSum + scorr) * wSpiky(r, g_h);
	}

	res = res / REST;
	return res;
}

void Algorithm() {

	std::vector<Particle> predict_p = particles;
	glm::vec3 gravity = glm::vec3(0.0, -9.8, 0.0);

	

	/*time_t nowTime = time(0);*/

	/*if((difftime(nowTime, g_initTime) > 15) && (wall <= 13.1) ) {
		if(g_zmax > 10)
			g_zmax = g_zmax - 0.05;
		else {
			wall = 14;
		}
	}

	if(wall >= 14) {
		g_zmax = g_zmax + 0.05;
		if (g_zmax > 13.1)
			wall = 10;
	}*/

	for (int i = 0; i < npart ; i++) {
		predict_p[i].velocity = particles[i].velocity + DT * gravity;
		predict_p[i].position = particles[i].position + DT * predict_p[i].velocity;
	}
	

	/*double currentTimeBefore = glfwGetTime();

	printf("%f tempo antes de entrar no for \n", double(currentTimeBefore));*/
	for (int ineigh = 0; ineigh < npart; ineigh++) {
		for (int j = 0; j < npart; j++) {
			if (ineigh != j) {
				glm::vec3 r = predict_p[ineigh].position - predict_p[j].position;
				if(glm::length(r) <= g_h) {
					predict_p[ineigh].neighbors.push_back(predict_p[j]);
				}
			}
		}
	}

	/*double currentTimeNeigh = glfwGetTime();

	printf("%f tempo depois de entrar no for \n", double(currentTimeNeigh));*/
	

	int iter = 0;

	while(iter < ITER) {
		for (int i = 0; i < npart; i++) {
			predict_p[i].rho = DensityEstimator(predict_p[i]);
			predict_p[i].C = predict_p[i].rho / REST - 1;
			float sumNabla = NablaCSquaredSumFunction(predict_p[i]);
			predict_p[i].lambda = - predict_p[i].C / (sumNabla + EPSILON);
		}
		int start = clock();
		for (int i = 0; i < npart; i++) {
			predict_p[i].delta_p = CalculateDp(predict_p[i]);
			
			if( predict_p[i].position.z < g_zmin){
				predict_p[i].position.z = g_zmin+0.000001f;
				/*glm::vec3 normal = glm::vec3(0,0,1);
				predict_p[i].velocity.z = glm::reflect(predict_p[i].velocity, normal).z * BOUNCE;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			if( predict_p[i].position.z > g_zmax){
				predict_p[i].position.z = g_zmax-0.000001f;
				/*glm::vec3 normal = glm::vec3(0,0,-1);
				predict_p[i].velocity.z = glm::reflect(predict_p[i].velocity, normal).z * BOUNCE;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			if( predict_p[i].position.y < -g_ymax){
				predict_p[i].position.y = -g_ymax+0.000001f;
				/*glm::vec3 normal = glm::vec3(0,1,0);
				predict_p[i].velocity.y = glm::reflect(predict_p[i].velocity, normal).y;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
			}
			
			if( predict_p[i].position.x < -g_xmax){
				predict_p[i].position.x = -g_xmax+0.000001f;
				/*glm::vec3 normal = glm::vec3(1,0,0);
				predict_p[i].velocity.x = glm::reflect(predict_p[i].velocity, normal).x * BOUNCE;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
				
			}
			if( predict_p[i].position.x > g_xmax){
				predict_p[i].position.x = g_xmax-0.000001f;
				/*glm::vec3 normal = glm::vec3(-1,0,0);
				predict_p[i].velocity.x = glm::reflect(predict_p[i].velocity, normal).x * BOUNCE;*/
				/*predict_p[i].position = particles[i].position + predict_p[i].velocity * BOUNCE;*/
				
			}

			
		}
		int end = clock();
		std::cout << "it took " << end - start << "ticks, or " << ((float)end - start) / CLOCKS_PER_SEC << "seconds." << std::endl;

		for (int i = 0; i < npart; i++) {
			predict_p[i].position = predict_p[i].position + predict_p[i].delta_p;
		}

		iter++;
	}

	//g_xmax = g_xmax - 0.1;

	

	for (int i = 0; i < npart; i++) {
		predict_p[i].velocity = (1/DT) * (predict_p[i].position - particles[i].position);
		predict_p[i].neighbors.clear();
	}

	particles = predict_p;
}

//void BuildGrid() {
//	for(int i=-g_xmax;i<=g_xmax;i++)
//	{
//		g_grid.push_back(i); g_grid.push_back(-g_ymax); g_grid.push_back(-g_xmax);
//		g_grid.push_back(i); g_grid.push_back(-g_ymax); g_grid.push_back(g_xmax);
//		g_grid.push_back(-g_xmax); g_grid.push_back(-g_ymax); g_grid.push_back(i);
//		g_grid.push_back(g_xmax); g_grid.push_back(-g_ymax); g_grid.push_back(i);
//	}
//}

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
	TwAddVarRW(g_pToolBar, "g_k", TW_TYPE_FLOAT, &g_k, " label='k for scorr' min=-13 max=13 step=0.01 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "g_dq", TW_TYPE_FLOAT, &g_dq, " label='dq for scorr' min=-13 max=13 step=0.01 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "h_sphere", TW_TYPE_FLOAT, &h_sphere, " label='h_sphere' min=0 max=5 step=0.01 keyIncr=h keyDecr=H help='Rotation speed (turns/second)' ");

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

