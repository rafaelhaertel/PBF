// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <time.h>

#define PI 3.1415926536f
#define EPSILON  0.001f
#define ITER 4
#define NPART 500
#define H 1.5
#define REST 10000.0f
#define DT 0.01f

// Include GLEW
#include <GL/glew.h>

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

class Particle
{
   public:
      glm::vec3 position;   // Posição Inicial
      glm::vec3 pred_position;  // Posição Prevista durante o passo
      glm::vec3 velocity;
	  glm::vec3 delta_p;
	  float mass;
	  float lambda;
	  float rho;
	  float C;
	  std::vector<Particle> neighbors;
};

std::vector<Particle> particles;
std::vector<float> g_grid;
float g_xmax = 13;
float g_ymax = 20;
float g_zmax = 13;
float g_zmin = -13;
float g_h = 1.5;
float POW_H_9 =(g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h*g_h); // h^9
float POW_H_6 =(g_h*g_h*g_h*g_h*g_h*g_h); // h^6
float wall = 10;
time_t g_initTime = time(0);
float rotatey = 0;
float rotatex = 0;
float rotatez = 0;

void initializeParticles () {
	particles.reserve(NPART);
	srand((unsigned int) 1);

	for (int i = 0 ; i < NPART ; i++){
		Particle p;
		p.position.x = (float)rand()/(float)(RAND_MAX/g_xmax);
		p.position.y = (float)rand()/(float)(RAND_MAX/g_ymax);
		p.position.z = (float)rand()/(float)(RAND_MAX/g_zmax);;

		p.velocity = glm::vec3(0.0f);
		p.mass = 1;
		p.delta_p = glm::vec3(0.0f);
		p.rho = 0.0;
		p.C = 1;
		p.pred_position = glm::vec3(0.0f);
		p.lambda = 0.0f;
		
		particles.push_back(p);
	}


};

float wPoly6(glm::vec3 r, float h) {
	return 315.0 / (64.0 * PI * POW_H_9) * (h*h - glm::dot(r,r)) * (h*h - glm::dot(r,r)) * (h*h - glm::dot(r,r));
}

glm::vec3 wSpiky(glm::vec3 r, float h) {;
	/*float hr_term = h - glm::length(r);
	float gradient_magnitude = 45.0f / (PI * POW_H_6) * hr_term * hr_term;
	float div = (glm::length(r) + EPSILON);*/
	glm::vec3 teste = (45.0f / (PI * POW_H_6)) * (((h - glm::length(r)) * (h - glm::length(r))) * (r/(glm::length(r)+EPSILON)));
	return teste;
	/*return r = 45.0 / (3.1415 * POW_H_6) * ((h - glm::length(r)) *  (h - glm::length(r))) * (r/(glm::length(r)+EPSILON));*/
}

float DensityEstimator(Particle p) {
	float rho = 0.0;

	for (int j = 0 ; j < p.neighbors.size() ; j++) {
		glm::vec3 r = p.position - p.neighbors[j].position;
		rho = rho + p.neighbors[j].mass * wPoly6(r, g_h);
	}
	return rho;
}

float NablaCSquaredSumFunction(Particle p) {

	std::vector<glm::vec3> NablaC;

	if (p.neighbors.size() > 0) {
		for (int j = 0 ; j < p.neighbors.size() ; j++) {													//for k != i
			glm::vec3 r = p.position - p.neighbors[j].position;
			glm::vec3 nablac= -wSpiky(r, g_h) / REST;
			NablaC.push_back(nablac);
		}

		NablaC.push_back(glm::vec3(0.0f));																	//for k = i
		int last = NablaC.size()-1; 
		for (int j = 0 ; j < p.neighbors.size() ; j++) {
			glm::vec3 r = p.position - p.neighbors[j].position;
			NablaC[last] = NablaC[last] + wSpiky(r, g_h);
		}
		NablaC[last] = NablaC[last] / REST;

		float res = 0;
		for (int k = 0 ; k < NablaC.size() ; k++) {
			res = res + length(NablaC[k]) * length(NablaC[k]);
		}
		return res;
	}
	else
		return  0.0f;
}

glm::vec3 CalculateDp(Particle p) {
	glm::vec3 res = glm::vec3(0.0f);
	float k = 0.1;
	float d_q = 0.1*g_h;
	glm::vec3 poly_dq =  d_q * glm::vec3(1.0f) + p.position;
	for (int j = 0 ; j < p.neighbors.size() ; j++) {
		glm::vec3 r = p.position - p.neighbors[j].position;
		float scorr = -k * ((wPoly6(r, g_h))/wPoly6(poly_dq, g_h));
		scorr = scorr * scorr * scorr * scorr;
		res = res + (p.lambda + p.neighbors[j].lambda + scorr) * wSpiky(r, g_h);
	}

	res = res / REST;
	return res;
}

void Algorithm() {

	std::vector<Particle> predict_p = particles;
	glm::vec3 gravity = glm::vec3(0.0, -9.8, 0.0);

	time_t nowTime = time(0);

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

	for (int i = 0; i < NPART ; i++) {
		predict_p[i].velocity = particles[i].velocity + DT * gravity;
		predict_p[i].position = particles[i].position + DT * predict_p[i].velocity;
	}

	for (int i = 0 ; i < NPART ; i++) {
		for (int j = 0 ; j < NPART ; j++) {
			int alloc = 0;
			if(i != j) {
				glm::vec3 r = predict_p[i].position - predict_p[j].position;
				if(glm::length(r) <= g_h) {
					predict_p[i].neighbors.push_back(predict_p[j]);
				}
			}
		}
	}

	int iter = 0;

	while(iter < ITER) {
		for (int i = 0 ; i < NPART ; i++) {
			predict_p[i].rho = DensityEstimator(predict_p[i]);
			predict_p[i].C = predict_p[i].rho / REST - 1;
			float sumNabla = NablaCSquaredSumFunction(predict_p[i]);
			predict_p[i].lambda = - predict_p[i].C / (sumNabla + EPSILON);
		}

		for (int i = 0; i < NPART ; i++) {
			predict_p[i].delta_p = CalculateDp(predict_p[i]);
			
			if( predict_p[i].position.z < g_zmin){
				predict_p[i].position.z = g_zmin+0.0001f;
			}
			if( predict_p[i].position.z > g_zmax){
				predict_p[i].position.z = g_zmax-0.0001f;
			}
			if( predict_p[i].position.y < -g_ymax){
				predict_p[i].position.y = -g_ymax+0.0001f;
			}
			if( predict_p[i].position.y > g_ymax){
				predict_p[i].position.y = g_ymax-0.0001f;
			}
			if( predict_p[i].position.x < -g_xmax){
				predict_p[i].position.x = -g_xmax+0.0001f;
				
			}
			if( predict_p[i].position.x > g_xmax){
				predict_p[i].position.x = g_xmax-0.0001f;
				
			}
		}

		for (int i = 0 ; i < NPART ; i++) {
			predict_p[i].position = predict_p[i].position + predict_p[i].delta_p;
		}

		iter++;
	}

	//g_xmax = g_xmax - 0.1;

	for (int i = 0 ; i < NPART ; i++) {
		predict_p[i].velocity = (1/DT) * (predict_p[i].position - particles[i].position);
		predict_p[i].neighbors.clear();
	}

	particles = predict_p;
}

void BuildGrid() {
	for(int i=-g_xmax;i<=g_xmax;i++)
	{
		g_grid.push_back(i); g_grid.push_back(-g_ymax); g_grid.push_back(-g_xmax);
		g_grid.push_back(i); g_grid.push_back(-g_ymax); g_grid.push_back(g_xmax);
		g_grid.push_back(-g_xmax); g_grid.push_back(-g_ymax); g_grid.push_back(i);
		g_grid.push_back(g_xmax); g_grid.push_back(-g_ymax); g_grid.push_back(i);
	}
}




int main(void)
{
	int nUseMouse = 0;
	initializeParticles();

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
	TwAddVarRW(g_pToolBar, "g_h", TW_TYPE_FLOAT, &g_h, " label='H radius' min=1.5 max=5 step=0.01 keyIncr=h keyDecr=H help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "rotatey", TW_TYPE_FLOAT, &rotatey, " label='rotation y of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "rotatex", TW_TYPE_FLOAT, &rotatex, " label='rotation x of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "rotatez", TW_TYPE_FLOAT, &rotatez, " label='rotation y of wall' min=-360 max=360 step=1.0 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");
	TwAddVarRW(g_pToolBar, "g_zmin", TW_TYPE_FLOAT, &g_zmin, " label='position z of wall' min=-13 max=13 step=0.05 keyIncr=r keyDecr=R help='Rotation speed (turns/second)' ");

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
	GLuint wallProgramID = LoadShaders("shaders/wallShading.vertexshader", "shaders/wallShading.fragmentshader");
	GLuint linesProgramID = LoadShaders( "shaders/SimpleVertexShader.vertexshader", "shaders/SimpleFragmentShader.fragmentshader" );
	BuildGrid();

	GLuint vertexbuffergrid;
	glGenBuffers(1, &vertexbuffergrid);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffergrid);
	glBufferData(GL_ARRAY_BUFFER, g_grid.size() * sizeof(float), &g_grid[0], GL_STATIC_DRAW);


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

	std::vector<glm::vec3> wall_vertices;
	std::vector<glm::vec2> wall_uvs;
	std::vector<glm::vec3> wall_normals;
	bool wallres = loadOBJ("mesh/wall.obj", wall_vertices, wall_uvs, wall_normals);

	std::vector<unsigned short> wall_indices;
	std::vector<glm::vec3> wall_indexed_vertices;
	std::vector<glm::vec2> wall_indexed_uvs;
	std::vector<glm::vec3> wall_indexed_normals;
	indexVBO(wall_vertices, wall_uvs, wall_normals, wall_indices, wall_indexed_vertices, wall_indexed_uvs, wall_indexed_normals);

	// Load it into a VBO

	GLuint wallbuffer;
	glGenBuffers(1, &wallbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, wallbuffer);
	glBufferData(GL_ARRAY_BUFFER, wall_indexed_vertices.size() * sizeof(glm::vec3), &wall_indexed_vertices[0], GL_STATIC_DRAW);

	GLuint wallnormalbuffer;
	glGenBuffers(1, &wallnormalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, wallnormalbuffer);
	glBufferData(GL_ARRAY_BUFFER, wall_indexed_normals.size() * sizeof(glm::vec3), &wall_indexed_normals[0], GL_STATIC_DRAW);

	GLuint wallelementbuffer;
	glGenBuffers(1, &wallelementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wallelementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, wall_indices.size() * sizeof(unsigned short), &wall_indices[0], GL_STATIC_DRAW);

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
		glUseProgram(linesProgramID);

		GLuint MatrixID				= glGetUniformLocation(linesProgramID, "MVP");
		glm::mat4 MVP				= ProjectionMatrix * ViewMatrix;
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexbuffergrid);
		glVertexAttribPointer(
			0,                  // attribute
			2,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);

		glLineWidth(3.f);
		
		glDrawArrays(GL_LINES, 0, g_grid.size()/3); // 3 indices starting at 0 -> 1 triangle

		glDisableVertexAttribArray(0);

		check_gl_error();

		glUseProgram(standardProgramID);
		MatrixID				 = glGetUniformLocation(standardProgramID, "MVP");
		GLuint ViewMatrixID		 = glGetUniformLocation(standardProgramID, "V");
		GLuint ModelMatrixID	 = glGetUniformLocation(standardProgramID, "M");


		glm::vec3 lightPos = glm::vec3(0, 20, 0);
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
		ModelMatrix[0][0]		   = 1.5f; //Escala do modelo (x)
		ModelMatrix[1][1]		   = 1.5f; //Escala do modelo (y)
		ModelMatrix[2][2]		   = 1.5f; //Escala do modelo (z)

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

		glUseProgram(wallProgramID);
		MatrixID				 = glGetUniformLocation(standardProgramID, "MVP");
		ViewMatrixID			 = glGetUniformLocation(standardProgramID, "V");
		ModelMatrixID			 = glGetUniformLocation(standardProgramID, "M");

		// Bind our texture in Texture Unit 0
		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_2D, Texture);
		//// Set our "myTextureSampler" sampler to user Texture Unit 0
		//glUniform1i(TextureID, 0);

		// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, wallbuffer);
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
		glBindBuffer(GL_ARRAY_BUFFER, wallnormalbuffer);
		glVertexAttribPointer(
			2,                                // attribute
			3,                                // size
			GL_FLOAT,                         // type
			GL_FALSE,                         // normalized?
			0,                                // stride
			(void*)0                          // array buffer offset
			);

		// Index buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wallelementbuffer);
		
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
	
		ModelMatrix				   = glm::mat4(1.0f); //Usar posição aleatória
		ModelMatrix[0][0]		   = 15*2; //Escala do modelo (x)
		ModelMatrix[1][1]		   = g_ymax*2; //Escala do modelo (y)
		ModelMatrix[2][2]		   = 0; //Escala do modelo (z)
		
		//Render
			
		ModelMatrix[3][0]		   = -g_xmax; //posição x
		ModelMatrix[3][1]		   = -g_ymax;//posição y
		ModelMatrix[3][2]		   = g_zmin;//posição z

			

		ModelMatrix = glm::rotate(ModelMatrix, rotatex, glm::vec3(1, 0, 0));
		ModelMatrix = glm::rotate(ModelMatrix, rotatey, glm::vec3(0, 1, 0));
		ModelMatrix = glm::rotate(ModelMatrix, rotatez, glm::vec3(0, 0, 1));

		MVP						   = ProjectionMatrix * ViewMatrix * ModelMatrix;

		// Send our transformation to the currently bound shader,
		// in the "MVP" uniform
		glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);


		// Draw the triangles !
		glDrawElements(
			GL_TRIANGLES,				// mode
			wall_indices.size(),		// count
			GL_UNSIGNED_SHORT,			// type
			(void*)0					// element array buffer offset
				);
		//endfor

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
	glDeleteBuffers(1, &vertexbuffergrid);
	//glDeleteBuffers(1, &uvbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteBuffers(1, &elementbuffer);
	glDeleteProgram(standardProgramID);
	glDeleteProgram(linesProgramID);
	//glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Terminate AntTweakBar and GLFW
	TwTerminate();
	glfwTerminate();

	return 0;
}

