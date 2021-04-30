#pragma once

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
#include "windows.h"
#include "glm\glm\glm.hpp"
#include "camera.h"

class GLMBulletUtils {
	static glm::vec3 bulletToGlm(const btVector3& v) { return glm::vec3(v.getX(), v.getY(), v.getZ()); }

	static btVector3 glmToBullet(const glm::vec3& v) { return btVector3(v.x, v.y, v.z); }

	static glm::quat bulletToGlm(const btQuaternion& q) { return glm::quat(q.getW(), q.getX(), q.getY(), q.getZ()); }

	static btQuaternion glmToBullet(const glm::quat& q) { return btQuaternion(q.x, q.y, q.z, q.w); }

	static btMatrix3x3 glmToBullet(const glm::mat3& m) { return btMatrix3x3(m[0][0], m[1][0], m[2][0], m[0][1], m[1][1], m[2][1], m[0][2], m[1][2], m[2][2]); }

	// btTransform does not contain a full 4x4 matrix, so this transform is lossy.
	// Affine transformations are OK but perspective transformations are not.
	static btTransform glmToBullet(const glm::mat4& m)
	{
		glm::mat3 m3(m);
		return btTransform(glmToBullet(m3), glmToBullet(glm::vec3(m[3][0], m[3][1], m[3][2])));
	}

	static glm::mat4 bulletToGlm(const btTransform& t)
	{
		glm::mat4 m;
		const btMatrix3x3& basis = t.getBasis();
		// rotation
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				m[c][r] = basis[r][c];
			}
		}
		// traslation
		btVector3 origin = t.getOrigin();
		m[3][0] = origin.getX();
		m[3][1] = origin.getY();
		m[3][2] = origin.getZ();
		// unit scale
		m[0][3] = 0.0f;
		m[1][3] = 0.0f;
		m[2][3] = 0.0f;
		m[3][3] = 1.0f;
		return m;
	}
};




class VerticesObject {
protected:
	std::vector<glm::dvec4> vertices;
	GLuint vbo = 0;
	GLuint color;
	GLuint drawMode;
	glm::vec4 vcolor;
public:
	VerticesObject(std::vector<glm::dvec4> _vertices, GLuint _drawMode, glm::vec4 _color = glm::vec4(0.0,1.0,0.0,1.0)){
		this->vertices = _vertices;
		this-> vcolor = _color;
		this->drawMode = _drawMode;
	}
	virtual std::vector<dvec4> getVertices() {
		return this->vertices;
	}
	virtual void generateVBO() {
		glGenBuffers(1, &this->vbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->vbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, this->vertices.size() * sizeof(dvec4), this->vertices.data(), GL_DYNAMIC_COPY);
		glGenBuffers(1, &this->color);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->color);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(vec4), &this->vcolor.data, GL_DYNAMIC_COPY);
	}
	virtual GLuint getVBO() {
		if (vbo == 0) {
			throw "VBO not generated";
		}
		return this->vbo;
	}
	virtual GLuint getColor() {
		if (color == 0) {
			throw "Color not generated";
		}
		return this->color;
	}
	virtual GLuint getDrawMode() {
		return this->drawMode;
	}
};
class LoopObject {
public:
	virtual void init() {
		
	}
	virtual void update() {
		
	}
	virtual void draw() {
	}

};
class ReferenceLine :public VerticesObject {
public:
	ReferenceLine(std::vector<glm::dvec4> _vertices):VerticesObject(_vertices, GL_LINE_STRIP) {
		
	}
	void generateVBO() {
		glGenBuffers(1, &this->vbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER,this->vbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, this->vertices.size() * sizeof(glm::dvec4), this->vertices.data(), GL_STATIC_DRAW);

		/*this->pvbo = (glm::dvec4*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, this->vertices.size() * sizeof(glm::dvec4),
			GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		for (int i = 0; i < this->vertices.size(); i++)
		{
			pvbo[i] = this->vertices.at(i);
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);*/
	}
};
class Renderer
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

public:
	virtual int init(int _width, int _height)
	{
		width = _width;
		height = _height;
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



		window = glfwCreateWindow(width, height, "Benchmarks", NULL, NULL);
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
		camera.setSens(0.01f, 100.1f);
		camera.setPosition(vec3(-20, 10, 0));
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
		perFrameData = (struct perFrameData*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct perFrameData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

		glGenBuffers(1, &ConstantBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ConstantBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(struct constantData), NULL, GL_DYNAMIC_COPY);
		constantData = (struct constantData*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct constantData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
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
	virtual void launchLoop() {
		double loopTotalTime = 0;
		double deltaTime = 0;
		double stallTime = 1.0;
		//camera.setPosition(vec3(0, 0, 200));
		//camera.setUp(vec3(0, 1, 0));
		//camera.setForward(vec3(0, 0, -1));
		camera.setPosition(vec3(0, 0, 200));
		camera.setUp(vec3(0, 1, 0));
		camera.setForward(vec3(0, 0, -1));
		ViewMatrix = camera.cameraPositionKeyboard(0);
		this->initLoopObjects();
		GLuint drawMode = GL_FILL;
		double iddleTime = 0;
		do
		{
			static double lastTime = glfwGetTime();
			double currentTime = glfwGetTime();
			deltaTime = double(currentTime - lastTime);
			loopTotalTime += deltaTime;
			///////////////
			this->updateLoopObjects();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glPointSize(2.0f);
			ViewMatrix = camera.cameraPositionKeyboard(deltaTime);
			GLuint drawMode = GL_LINE;
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, PerFrameBuffer);
			perFrameData = (struct perFrameData*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct perFrameData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
			perFrameData->ViewMatrix = ViewMatrix;
			glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
			glUseProgram(RenderProgram);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, PerFrameBuffer);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ConstantBuffer);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, DebugBuffer);

			for(int i=0;i<this->verticesObjects.size();i++){
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
			///////////////
			lastTime = currentTime;
			glfwSwapBuffers(window);
			glfwPollEvents();
		} while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
			glfwWindowShouldClose(window) == 0);
		glfwDestroyWindow(window);
	}
};

