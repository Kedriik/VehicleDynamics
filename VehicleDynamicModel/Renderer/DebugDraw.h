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
class DebugDraw:public btIDebugDraw {
public:
	int debugMode = 0;
	int width;
	int height;
	 GLFWwindow* window;
	mat4 ProjectionMatrix;
	Camera camera;
	GLuint RenderProgram;
	GLuint DefaultFrameBuffer;
	GLuint VBO;
	int VBOSize = 2;
	mat4 ViewMatrix;
	GLuint PerFrameBuffer;
	GLuint ConstantBuffer;
	GLuint ColorBuffer;
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
public:
	DebugDraw() {

	};
	~DebugDraw() {

	}
	int init(int _width, int _height)
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
		camera.setSens(0.01f, 10.1f);
		camera.setPosition(vec3(-20, 10, 0));
		camera.setUp(vec3(0, 1, 0));
		camera.setForward(vec3(1, 0, 0));
		//glEnable(GL_DEPTH_TEST);
		//glDisable(GL_DEPTH_TEST);
		//glEnable(GL_BLEND);
		//glBlendFunc(GL_ONE, GL_ONE);

		ShaderInfo  RenderProgramSource[] = {
			{ GL_VERTEX_SHADER,  "..\\Renderer\\VBOBenchmarkCommons\\Vertex.shader" },
			{ GL_FRAGMENT_SHADER,"..\\Renderer\\VBOBenchmarkCommons\\Fragment.shader" },
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
	void gpuDebug(std::string Nazwa = "bez nazwy")
	{
		float debug[4];
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, DebugBuffer);
		glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, 4 * sizeof(float), &debug);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
		std::cout << Nazwa << ::std::endl;
		std::cout << "\r debug.x:" << debug[0] << std::endl;
		std::cout << "\r debug.y:" << debug[1] << std::endl;
		std::cout << "\r debug.z:" << debug[2] << std::endl;
		std::cout << "\r debug.w:" << debug[3] << std::endl;


	}
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& _color) override {
		ViewMatrix = camera.cameraPositionKeyboard(0.001);
		GLuint drawMode = GL_LINE;
		//glBindFramebuffer(GL_FRAMEBUFFER, DefaultFrameBuffer);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, PerFrameBuffer);
		perFrameData = (struct perFrameData*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct perFrameData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		perFrameData->ViewMatrix = ViewMatrix;
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ColorBuffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(struct constantData), NULL, GL_DYNAMIC_COPY);
		color = (struct color*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, sizeof(struct color), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		color->color = vec4(_color.x(), _color.y(), _color.z(), _color.w());
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
		glPointSize(2);
		glUseProgram(RenderProgram);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, PerFrameBuffer);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ConstantBuffer);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, ColorBuffer);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, DebugBuffer);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, VBO);
		glBufferData(GL_SHADER_STORAGE_BUFFER, VBOSize * sizeof(vec4), NULL, GL_DYNAMIC_COPY);
		vbo = (vec4*) glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, VBOSize * sizeof(vec4),
			GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
		vbo[0] = vec4(from.x(), from.y(), from.z(), 1.0);
		vbo[1] = vec4(to.x(), to.y(), to.z(), 1.0);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);



		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glVertexAttribPointer(
			0,                  // attribute. No particular reason for 3, but must match the layout in the shader.
			4,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
		);


		glDrawArrays(GL_LINES, 0, VBOSize);
;
	}
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& _color)override {
	
	}

	virtual void reportErrorWarning(const char* warningString) override {

	}

	virtual void draw3dText(const btVector3& location, const char* textString) override {

	}

	virtual void setDebugMode(int debugMode) override {
		this->debugMode = debugMode;
	}

	virtual int getDebugMode() const override {
		return this->debugMode;
	}

};