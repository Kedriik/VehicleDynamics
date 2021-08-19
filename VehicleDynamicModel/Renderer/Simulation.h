#pragma once
#ifndef Simulation_h
#define Simulation_h
#include "LinearMath/btIDebugDraw.h"
#include "glew-2.0.0\include\GL\glew.h"
#include "glfw-3.2.1.bin.WIN64\include\GLFW\glfw3.h"
#include "glm\glm\gtx\transform.hpp"
#include "glm\glm\gtc\matrix_transform.hpp"
#include "glm\glm\gtc\quaternion.hpp"
#include "glm\glm\gtx\quaternion.hpp"
#include <iostream>
#include "glm\glm\glm.hpp"
#include <string>
#include <fstream>
#include "LoadShaders\include\LoadShaders.h"
#include <vector>
#include <time.h>
#include <conio.h>
#include <vector>
#include <chrono>
#include "glm\glm\glm.hpp"
#include "camera.h"
#include "../Definitions.h"
#include "../VehicleDynamicModel.h"
#include "OpenDriveDocument.h"
class Simulation
{
protected:
	int debugMode = 0;
	int width;
	int height;
	GLFWwindow* window;
	mat4 ProjectionMatrix;
	mat4 ViewMatrix;
	Camera camera;
	GLuint RenderProgram;
	GLuint DefaultFrameBuffer;
	GLuint PerFrameBuffer;
	GLuint ConstantBuffer;
	GLuint ColorBuffer;
	GLuint VBO;
	int VBOSize = 2;
	std::vector<VerticesObject*> verticesObjects;
	std::vector<IndexedVerticesObject*> indexedVerticesObjects;
	vec4* vbo;
	GLuint DebugBuffer;
	struct perFrameData
	{
		mat4 ViewMatrix;
	} *perFrameData;
	struct constantData
	{
		mat4 ProjectionMatrix;
	} *constantData;
	struct color {
		vec4 color;
	} *color;
	std::vector<LoopObject*> loopObjects;
	OpenDriveDocument odd;

public:
	Simulation(std::string _openDriveFilePath):odd(_openDriveFilePath) {

	}
	~Simulation() {
		for (auto vo : verticesObjects) {
			delete vo;
		}
		for (auto ivo : indexedVerticesObjects) {
			delete ivo;
		}
	}
	virtual int initOpenDriveDocument() {
		odd.parseOpenDriveDocument();
		odd.GenerateRoadsLanes();
		return 0;
	}
	virtual void initRenderObjects() {
		using clock = std::chrono::system_clock;
		using sec = std::chrono::duration<double>;
		auto before_all = clock::now();
		std::cout << "Generating Render Objects" << std::endl;
		auto before = clock::now();
		std::vector<VerticesObject*> verticesObjects;
		std::vector<IndexedVerticesObject*> indexedVerticesObjects;
		std::vector<glm::dvec4> documentVertices;
		for (int i = 0; i < odd.getRoads().size(); i++) {
			Road r = odd.getRoads().at(i);
			VerticesObject* obj = new VerticesObject(r.getReferencePoints(), GL_LINE_STRIP, glm::vec4(1, 0, 0, 1));
			obj->generateVBO();
			verticesObjects.push_back(obj);
			obj = new VerticesObject(r._vertices, GL_POINTS, glm::vec4(0, 1, 0, 1));
		}
		for (auto r : odd.getRoads()) {
			for (auto l : r.debugLanesRenderObject) {
				l->generateVBO();
				verticesObjects.push_back(l);
			}
			for (auto lane : r.lanesRenderObjects) {
				lane->generateVBO();
				indexedVerticesObjects.push_back(lane);
			}

			for (auto edge : r.debugEdgeRenderObjects) {
				edge->generateVBO();
				indexedVerticesObjects.push_back(edge);
			}
		}
		for (auto r : odd.getRoads()) {
			IndexedVerticesObject* idobj = new IndexedVerticesObject(r.debugVertices, r.debugIndexes, GL_LINES, glm::dvec4(238.0 / 255.0, 130.0 / 255.0, 238.0 / 255.0, 1));
			idobj->generateVBO();
			indexedVerticesObjects.push_back(idobj);
		}

		for (auto ro : odd.roadRenderObjects) {
			ro->generateVBO();
			indexedVerticesObjects.push_back(ro);

		}
		for (auto v : odd.debug) {
			v->generateVBO();
			verticesObjects.push_back(v);
		}
		for (auto v : odd.indexedDebug) {
			v->generateVBO();
			indexedVerticesObjects.push_back(v);
		}
		sec duration = clock::now() - before;
		sec duration_all = clock::now() - before_all;
		std::cout << "Generating finished in " << duration.count() << "s" << std::endl;
		std::cout << "All finished in " << duration_all.count() << "s" << std::endl;
		double gap = 5.0;

		std::vector<glm::dvec4> sphereVertices;
		std::vector<glm::dvec3> sphereNormals;
		std::vector<glm::dvec2> texCoords;
		std::vector<unsigned int> sphereindices;
		std::vector<unsigned int> spherelinesIndices;
		ShapesGenerator::generateSphereShape(sphereVertices, sphereNormals, texCoords, sphereindices, spherelinesIndices);
		IndexedVerticesObject* sphereObj = new IndexedVerticesObject(sphereVertices, sphereindices, GL_TRIANGLES, glm::dvec4(1, 1, 0, 1));
		sphereObj->generateVBO();
		//indexedVerticesObjects.push_back(sphereObj);

		std::vector<glm::dvec4> cylinderVertices;
		std::vector<glm::dvec3> cylinderNormals;
		std::vector<glm::dvec2> cylinderTexCoords;
		std::vector<unsigned int> cylinderindices;
		std::vector<unsigned int> cylinderlinesIndices;
		ShapesGenerator::generateCylinderShape(cylinderVertices, cylinderNormals, cylinderTexCoords, cylinderindices, cylinderlinesIndices);
		IndexedVerticesObject* cylinderObj = new IndexedVerticesObject(cylinderVertices, cylinderindices, GL_TRIANGLES, glm::dvec4(1, 1, 0, 1));
		cylinderObj->generateVBO();
		indexedVerticesObjects.push_back(cylinderObj);

		addVerticesObjects(verticesObjects);
		addIndexedVerticesObjects(indexedVerticesObjects);

	}
	virtual int initVisualisation() {
		if (!glfwInit())
		{
			fprintf(stderr, "Failed to initialize GLFW\n");
			return -1;
		}
		std::cout << "GLFW init completed" << std::endl;

		glfwWindowHint(GLFW_SAMPLES, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
		//	glfwWindowHint(GL_CONTEXT_FLAGS,GL_CONTEXT_FLAG_DEBUG_BIT);

		window = glfwCreateWindow(width, height, "Visu", NULL, NULL);
		glfwMakeContextCurrent(window);

		// Initialize GLEW
		glewExperimental = true; // Needed for core profile
		if (glewInit() != GLEW_OK) {
			fprintf(stderr, "Failed to initialize GLEW\n");
			return -1;
		}
		std::cout << "GLEW init completed" << std::endl;
		glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
		ProjectionMatrix = glm::infinitePerspective(45.0f, float(width) / float(height), 0.01f);

		vec3 camPos = vec3(0, 15.1211f, 0); vec3(51.2, 250, 51.2);
		camera = Camera(window, width, height, camPos, vec3(0, -1, 0.0), vec3(1, 0, 0), 0.01);
		camera.setSens(0.01f, 10.1f);
		camera.setPosition(vec3(-5, 10, 0));
		camera.setUp(vec3(0, 1, 0));
		camera.setForward(vec3(1, 0, 0));

		ShaderInfo  RenderProgramSource[] = {
			{ GL_VERTEX_SHADER,  "Renderer\\VBOBenchmarkCommons\\Vertex.shader" },
			{ GL_FRAGMENT_SHADER,"Renderer\\VBOBenchmarkCommons\\Fragment.shader" },
			{ GL_NONE, NULL },
			{ GL_NONE, NULL },
			{ GL_NONE, NULL }
		};
		RenderProgram = LoadShaders(RenderProgramSource);

		glGenBuffers(1, &PerFrameBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, PerFrameBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(struct perFrameData), NULL, GL_DYNAMIC_COPY);
		perFrameData = (struct perFrameData*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct perFrameData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

		glGenBuffers(1, &ConstantBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ConstantBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(struct constantData), NULL, GL_DYNAMIC_COPY);
		constantData = (struct constantData*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct constantData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		constantData->ProjectionMatrix = ProjectionMatrix;
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

		glGenBuffers(1, &ColorBuffer);

		glGenBuffers(1, &VBO);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, VBO);
		glBufferData(GL_SHADER_STORAGE_BUFFER, VBOSize * sizeof(vec4), NULL, GL_DYNAMIC_COPY);

		glGenFramebuffers(1, &DefaultFrameBuffer);
		glBindFramebuffer(GL_FRAMEBUFFER, DefaultFrameBuffer);

		glGenBuffers(1, &DebugBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, DebugBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(vec4), NULL, GL_DYNAMIC_COPY);
		vec4* debug = (vec4*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(vec4), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
		return 0;
	}
	virtual int init(int _width, int _height)
	{
		width = _width;
		height = _height;
		initVisualisation();
		initOpenDriveDocument();
		initRenderObjects();
		return 0;
	}
	virtual void addLoopObject(LoopObject* loopObject) {
		this->loopObjects.push_back(loopObject);
	}
	virtual void initLoopObjects() {
		for (auto lp : this->loopObjects) {
			lp->init();
		}
	}
	virtual void updateLoopObjects() {
		for (auto lp : this->loopObjects) {
			lp->update();
		}
	}
	virtual void drawLoopObjects() {
		for (auto lp : this->loopObjects) {
			lp->draw();
		}
	}
	virtual void addVerticesObject(VerticesObject* verticesObject) {
		this->verticesObjects.push_back(verticesObject);
	}
	virtual void addVerticesObjects(std::vector< VerticesObject*> verticesObjects) {
		this->verticesObjects = verticesObjects;
	}
	virtual void addIndexedVerticesObjects(std::vector<IndexedVerticesObject*> verticesObjects) {
		this->indexedVerticesObjects = verticesObjects;
	}
	virtual void launchLoop() {
		double loopTotalTime = 0;
		double deltaTime = 0;
		double stallTime = 1.0;
		camera.setPosition(vec3(0, 0, 20));
		camera.setUp(vec3(0, 1, 0));
		camera.setForward(vec3(0, 0, -1));
		ViewMatrix = camera.cameraPositionKeyboard(0);
		this->initLoopObjects();
		glEnable(GL_DEPTH_TEST);
		GLuint drawMode = GL_FILL;
		double iddleTime = 0;

		bool doPhysics = false;

		btVehicleDynamics vehicleDynamics;
		if (doPhysics) {
			std::vector<IndexedVerticesObject*> ivos;
			for (auto road : odd.roads) {
				for (auto lane : road.lanesRenderObjects) {
					ivos.push_back(lane);
				}
			}
			vehicleDynamics.initPhysics(ivos);
		}
		do
		{
			static double lastTime = glfwGetTime();
			double currentTime = glfwGetTime();
			deltaTime = double(currentTime - lastTime);
			loopTotalTime += deltaTime;
			///////////////
			if (doPhysics) {
				vehicleDynamics.m_dynamicsWorld->stepSimulation(deltaTime);
			}
			//////////////
			this->updateLoopObjects();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glPointSize(5.0f);
			glLineWidth(2.0f);
			ViewMatrix = camera.cameraPositionKeyboard(deltaTime);
			GLuint drawMode = GL_LINE;
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS){
				glLineWidth(1.0f);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			}
			else{
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glLineWidth(2.5f);
			}
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, PerFrameBuffer);
			perFrameData = (struct perFrameData*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct perFrameData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
			perFrameData->ViewMatrix = ViewMatrix;
			glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
			glUseProgram(RenderProgram);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, PerFrameBuffer);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ConstantBuffer);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, DebugBuffer);

			for(int i=0;i<this->verticesObjects.size();i++){
				//glLineWidth(5);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, this->verticesObjects.at(i)->getColor());
				glEnableVertexAttribArray(0);
				glBindBuffer(GL_ARRAY_BUFFER, this->verticesObjects.at(i)->getVBO());
				glVertexAttribPointer(
					0,                  // attribute. No particular reason for 3, but must match the layout in the shader.
					4,                  // size
					GL_DOUBLE,           // type
					GL_FALSE,           // normalized?
					0,                  // stride
					(void*)0            // array buffer offset
				);

				//GLuint _vbo = this->verticesObjects.at(i)->getVBO();
				glDrawArrays(this->verticesObjects.at(i)->getDrawMode(), 0, this->verticesObjects.at(i)->getVertices().size());
			}
			if(glfwGetKey(window, GLFW_KEY_G) != GLFW_PRESS){
				//glLineWidth(1);
				for (int i = 0; i < this->indexedVerticesObjects.size(); i++) {
					glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, this->indexedVerticesObjects.at(i)->getColor());
					glEnableVertexAttribArray(0);
					glBindBuffer(GL_ARRAY_BUFFER, this->indexedVerticesObjects.at(i)->getVBO());
					glVertexAttribPointer(
						0,                  // attribute. No particular reason for 3, but must match the layout in the shader.
						4,                  // size
						GL_DOUBLE,           // type
						GL_FALSE,           // normalized?
						0,                  // stride
						(void*)0            // array buffer offset
					);

					//GLuint _vbo = this->verticesObjects.at(i)->getVBO();
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->indexedVerticesObjects.at(i)->IndexBuffer);
					glDrawElements(this->indexedVerticesObjects.at(i)->getDrawMode(), this->indexedVerticesObjects.at(i)->indexes.size(), GL_UNSIGNED_INT, (void*)0);
					//glDrawArrays(this->indexedVerticesObjects.at(i)->getDrawMode(), 0, this->indexedVerticesObjects.at(i)->getVertices().size());
				}
			}
			///////////////
			lastTime = currentTime;
			glfwSwapBuffers(window);
			glfwPollEvents();
		} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
			glfwWindowShouldClose(window) == 0);
		glfwDestroyWindow(window);
	}
};
#endif

