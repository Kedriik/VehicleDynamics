#pragma once
#ifndef Definitions_h
#define Definitions_h
#include "LinearMath/btIDebugDraw.h"
#include "..\VehicleDynamicModel\Renderer\glew-2.0.0\include\GL\glew.h"
#include "..\VehicleDynamicModel\Renderer\glfw-3.2.1.bin.WIN64\include\GLFW\glfw3.h"
#include "..\VehicleDynamicModel\Renderer\glm\glm\gtx\transform.hpp"
#include "..\VehicleDynamicModel\Renderer\glm\glm\gtc\matrix_transform.hpp"
#include "..\VehicleDynamicModel\Renderer\glm\glm\gtc\quaternion.hpp"
#include "..\VehicleDynamicModel\Renderer\glm\glm\gtx\quaternion.hpp"
#include <iostream>
#include "..\VehicleDynamicModel\Renderer\glm\glm\glm.hpp"
#include <string>
#include <fstream>
#include "..\VehicleDynamicModel\Renderer\LoadShaders\include\LoadShaders.h"
#include <vector>
#include <time.h>
#include <conio.h>
#include <vector>
#include <chrono>
#include "objloader.hpp"
#include "..\VehicleDynamicModel\Renderer\glm\glm\glm.hpp"
#include "btBulletDynamicsCommon.h"

struct ModelData
{
	GLuint ModelVertexBuffer;
	GLuint ModelUVBuffer;
	GLuint ModelNormalBuffer;

	std::vector<glm::dvec4> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::dvec3> normals;

	int size;
};
struct ModelTransformation {
	glm::dvec3 rotation;
	glm::dvec3 position;
};
class ShapesGenerator {
public:
	static void generateSphereShape(
		std::vector<glm::dvec4>& vertices, std::vector<glm::dvec3>& normals,
		std::vector<glm::dvec2>& texCoords, std::vector<unsigned int>& indices,
		std::vector<unsigned int>& lineIndices, int radius = 1, int stackCount = 100, int sectorCount = 100) {
		double PI = 3.14159265359;
		// clear memory of prev arrays
		std::vector<glm::dvec4>().swap(vertices);
		std::vector<glm::dvec3>().swap(normals);
		std::vector<glm::dvec2>().swap(texCoords);
		std::vector<unsigned int>().swap(indices);
		std::vector<unsigned int>().swap(lineIndices);

		double x, y, z, xy;                              // vertex position
		double nx, ny, nz, lengthInv = 1.0 / radius;    // vertex normal
		double s, t;                                     // vertex texCoord

		double sectorStep = 2 * PI / sectorCount;
		double stackStep = PI / stackCount;
		double sectorAngle, stackAngle;

		for (int i = 0; i <= stackCount; ++i)
		{
			stackAngle = PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
			xy = radius * cosf(stackAngle);             // r * cos(u)
			z = radius * sinf(stackAngle);              // r * sin(u)

			// add (sectorCount+1) vertices per stack
			// the first and last vertices have same position and normal, but different tex coords
			for (int j = 0; j <= sectorCount; ++j)
			{
				sectorAngle = j * sectorStep;           // starting from 0 to 2pi

				// vertex position (x, y, z)
				x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
				y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
				vertices.push_back(glm::dvec4(x, y, z, 1));

				// normalized vertex normal (nx, ny, nz)
				nx = x * lengthInv;
				ny = y * lengthInv;
				nz = z * lengthInv;
				normals.push_back(glm::dvec3(nx, ny, nz));

				// vertex tex coord (s, t) range between [0, 1]
				s = (double)j / sectorCount;
				t = (double)i / stackCount;
				texCoords.push_back(glm::dvec2(s, t));
			}
		}
		// generate CCW index list of sphere triangles
		// k1--k1+1
		// |  / |
		// | /  |
		// k2--k2+1

		int k1, k2;
		for (int i = 0; i < stackCount; ++i)
		{
			k1 = i * (sectorCount + 1);     // beginning of current stack
			k2 = k1 + sectorCount + 1;      // beginning of next stack

			for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
			{
				// 2 triangles per sector excluding first and last stacks
				// k1 => k2 => k1+1
				if (i != 0)
				{
					indices.push_back(k1);
					indices.push_back(k2);
					indices.push_back(k1 + 1);
				}

				// k1+1 => k2 => k2+1
				if (i != (stackCount - 1))
				{
					indices.push_back(k1 + 1);
					indices.push_back(k2);
					indices.push_back(k2 + 1);
				}

				// store indices for lines
				// vertical lines for all stacks, k1 => k2
				lineIndices.push_back(k1);
				lineIndices.push_back(k2);
				if (i != 0)  // horizontal lines except 1st stack, k1 => k+1
				{
					lineIndices.push_back(k1);
					lineIndices.push_back(k1 + 1);
				}
			}
		}
	}
	static std::vector<glm::dvec4> getUnitCircleVertices(int sectorCount = 100)
	{
		const float PI = 3.1415926f;
		float sectorStep = 2 * PI / sectorCount;
		float sectorAngle;  // radian

		std::vector<glm::dvec4> unitCircleVertices;
		for (int i = 0; i <= sectorCount; ++i)
		{
			sectorAngle = i * sectorStep;
			unitCircleVertices.push_back(glm::dvec4(cos(sectorAngle), sin(sectorAngle), 0, 1));
			//unitCircleVertices.push_back(); // x
			//unitCircleVertices.push_back(); // y
			//unitCircleVertices.push_back(0);                // z
		}
		return unitCircleVertices;
	}
	static void generateCylinderShape(
		std::vector<glm::dvec4>& vertices, std::vector<glm::dvec3>& normals,
		std::vector<glm::dvec2>& texCoords, std::vector<unsigned int>& indices,
		std::vector<unsigned int>& lineIndices, double height = 1, int radius = 1, int sectorCount = 100, int stackCount = 10)
	{
		// clear memory of prev arrays
		std::vector<glm::dvec4>().swap(vertices);
		std::vector<glm::dvec3>().swap(normals);
		std::vector<glm::dvec2>().swap(texCoords);
		std::vector<unsigned int>().swap(indices);
		std::vector<unsigned int>().swap(lineIndices);

		// get unit circle vectors on XY-plane
		std::vector<glm::dvec4> unitVertices = getUnitCircleVertices();

		// put side vertices to arrays
		for (int i = 0; i < 2; ++i)
		{
			double h = -height / 2.0f + i * height;           // z value; -h/2 to h/2
			double t = 1.0f - i;                              // vertical tex coord; 1 to 0

			for (int j = 0, k = 0; j <= sectorCount; ++j, k++)
			{
				vertices.push_back(glm::dvec4(unitVertices.at(k).x * radius, h, unitVertices.at(k).y * radius, 1));
				normals.push_back(glm::dvec3(unitVertices.at(k).x, unitVertices.at(k).y, unitVertices.at(k).z));
				texCoords.push_back(glm::dvec2((double)j / sectorCount, t));

			}
		}

		// the starting index for the base/top surface
		//NOTE: it is used for generating indices later
		int baseCenterIndex = (int)vertices.size() / 3;
		int topCenterIndex = baseCenterIndex + sectorCount + 1; // include center vertex

		// put base and top vertices to arrays
		for (int i = 0; i < 2; ++i)
		{
			double h = -height / 2.0 + i * height;           // z value; -h/2 to h/2
			double nz = -1 + i * 2;                           // z value of normal; -1 to 1

			// center point
			//vertices.push_back(0);     vertices.push_back(0);     vertices.push_back(h);
			vertices.push_back(glm::dvec4(0, h, 0, 1));
			//normals.push_back(0);      normals.push_back(0);      normals.push_back(nz);
			normals.push_back(glm::dvec3(0, 0, nz));
			//texCoords.push_back(0.5f); texCoords.push_back(0.5f);
			texCoords.push_back(glm::dvec2(0.5, 0.5));

			for (int j = 0, k = 0; j < sectorCount; ++j, k++)
			{                    // vz
				vertices.push_back(glm::dvec4(unitVertices.at(k).x * radius, h, unitVertices.at(k).y * radius, 1));
				// normal vector
				normals.push_back(glm::dvec3(0, 0, nz));
				// texture coordinate
				texCoords.push_back(glm::dvec2(-unitVertices.at(k).x * 0.5 + 0.5, -unitVertices.at(k).y * 0.5 + 0.5));
			}
		}
		// generate CCW index list of cylinder triangles
		//std::vector<int> indices;
		int k1 = 0;                         // 1st vertex index at base
		int k2 = sectorCount + 1;           // 1st vertex index at top

		// indices for the side surface
		for (int i = 0; i < sectorCount; ++i, ++k1, ++k2)
		{
			// 2 triangles per sector
			// k1 => k1+1 => k2
			indices.push_back(k1);
			indices.push_back(k1 + 1);
			indices.push_back(k2);

			// k2 => k1+1 => k2+1
			indices.push_back(k2);
			indices.push_back(k1 + 1);
			indices.push_back(k2 + 1);
		}

		// indices for the base surface
		//NOTE: baseCenterIndex and topCenterIndices are pre-computed during vertex generation
		//      please see the previous code snippet
		for (int i = 0, k = baseCenterIndex + 1; i < sectorCount; ++i, ++k)
		{
			if (i < sectorCount - 1)
			{
				indices.push_back(baseCenterIndex);
				indices.push_back(k + 1);
				indices.push_back(k);
			}
			else // last triangle
			{
				indices.push_back(baseCenterIndex);
				indices.push_back(baseCenterIndex + 1);
				indices.push_back(k);
			}
		}

		// indices for the top surface
		for (int i = 0, k = topCenterIndex + 1; i < sectorCount; ++i, ++k)
		{
			if (i < sectorCount - 1)
			{
				indices.push_back(topCenterIndex);
				indices.push_back(k);
				indices.push_back(k + 1);
			}
			else // last triangle
			{
				indices.push_back(topCenterIndex);
				indices.push_back(k);
				indices.push_back(topCenterIndex + 1);
			}
		}
	}
	static void generateCube(std::vector<glm::dvec4>& _vertices, std::vector<unsigned int>& _indices) {
		// unit cube      
		// A cube has 6 sides and each side has 4 vertices, therefore, the total number
		// of vertices is 24 (6 sides * 4 verts), and 72 floats in the vertex array
		// since each vertex has 3 components (x,y,z) (= 24 * 3)
		//    v6----- v5  
		//   /|      /|   
		//  v1------v0|   
		//  | |     | |   
		//  | v7----|-v4  
		//  |/      |/    
		//  v2------v3    

		// vertex position array
		GLfloat vertices[] = {
			 .5f, .5f, .5f,  -.5f, .5f, .5f,  -.5f,-.5f, .5f,  .5f,-.5f, .5f, // v0,v1,v2,v3 (front)
			 .5f, .5f, .5f,   .5f,-.5f, .5f,   .5f,-.5f,-.5f,  .5f, .5f,-.5f, // v0,v3,v4,v5 (right)
			 .5f, .5f, .5f,   .5f, .5f,-.5f,  -.5f, .5f,-.5f, -.5f, .5f, .5f, // v0,v5,v6,v1 (top)
			-.5f, .5f, .5f,  -.5f, .5f,-.5f,  -.5f,-.5f,-.5f, -.5f,-.5f, .5f, // v1,v6,v7,v2 (left)
			-.5f,-.5f,-.5f,   .5f,-.5f,-.5f,   .5f,-.5f, .5f, -.5f,-.5f, .5f, // v7,v4,v3,v2 (bottom)
			 .5f,-.5f,-.5f,  -.5f,-.5f,-.5f,  -.5f, .5f,-.5f,  .5f, .5f,-.5f  // v4,v7,v6,v5 (back)
		};
		for (int i = 0; i < sizeof(vertices) / sizeof(*vertices); i+=3) {
			_vertices.push_back(glm::dvec4(vertices[i], vertices[i+1], vertices[i+2],1.0));

		}
		
		// normal array
		GLfloat normals[] = {
			 0, 0, 1,   0, 0, 1,   0, 0, 1,   0, 0, 1,  // v0,v1,v2,v3 (front)
			 1, 0, 0,   1, 0, 0,   1, 0, 0,   1, 0, 0,  // v0,v3,v4,v5 (right)
			 0, 1, 0,   0, 1, 0,   0, 1, 0,   0, 1, 0,  // v0,v5,v6,v1 (top)
			-1, 0, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0, 0,  // v1,v6,v7,v2 (left)
			 0,-1, 0,   0,-1, 0,   0,-1, 0,   0,-1, 0,  // v7,v4,v3,v2 (bottom)
			 0, 0,-1,   0, 0,-1,   0, 0,-1,   0, 0,-1   // v4,v7,v6,v5 (back)
		};

		// colour array
		GLfloat colors[] = {
			 1, 1, 1,   1, 1, 0,   1, 0, 0,   1, 0, 1,  // v0,v1,v2,v3 (front)
			 1, 1, 1,   1, 0, 1,   0, 0, 1,   0, 1, 1,  // v0,v3,v4,v5 (right)
			 1, 1, 1,   0, 1, 1,   0, 1, 0,   1, 1, 0,  // v0,v5,v6,v1 (top)
			 1, 1, 0,   0, 1, 0,   0, 0, 0,   1, 0, 0,  // v1,v6,v7,v2 (left)
			 0, 0, 0,   0, 0, 1,   1, 0, 1,   1, 0, 0,  // v7,v4,v3,v2 (bottom)
			 0, 0, 1,   0, 0, 0,   0, 1, 0,   0, 1, 1   // v4,v7,v6,v5 (back)
		};

		// texture coord array
		GLfloat texCoords[] = {
			1, 0,   0, 0,   0, 1,   1, 1,               // v0,v1,v2,v3 (front)
			0, 0,   0, 1,   1, 1,   1, 0,               // v0,v3,v4,v5 (right)
			1, 1,   1, 0,   0, 0,   0, 1,               // v0,v5,v6,v1 (top)
			1, 0,   0, 0,   0, 1,   1, 1,               // v1,v6,v7,v2 (left)
			0, 1,   1, 1,   1, 0,   0, 0,               // v7,v4,v3,v2 (bottom)
			0, 1,   1, 1,   1, 0,   0, 0                // v4,v7,v6,v5 (back)
		};

		// index array for glDrawElements()
		// A cube requires 36 indices = 6 sides * 2 tris * 3 verts
		GLuint indices[] = {
			 0, 1, 2,   2, 3, 0,    // v0-v1-v2, v2-v3-v0 (front)
			 4, 5, 6,   6, 7, 4,    // v0-v3-v4, v4-v5-v0 (right)
			 8, 9,10,  10,11, 8,    // v0-v5-v6, v6-v1-v0 (top)
			12,13,14,  14,15,12,    // v1-v6-v7, v7-v2-v1 (left)
			16,17,18,  18,19,16,    // v7-v4-v3, v3-v2-v7 (bottom)
			20,21,22,  22,23,20     // v4-v7-v6, v6-v5-v4 (back)
		};
		for (int i = 0; i < sizeof(indices) / sizeof(*indices); i++) {
			_indices.push_back(indices[i]);
		}
	}
	static ModelData LoadModel(std::string pathModelPath)
	{
		std::vector<glm::dvec4> vertices;
		std::vector<glm::vec2> uvs;
		std::vector<glm::dvec3> normals;
		loadOBJ(pathModelPath.c_str(), vertices, uvs, normals);

		GLuint ModelVertexBuffer;
		GLuint ModelUVBuffer;
		GLuint ModelNormalBuffer;

		glGenBuffers(1, &ModelVertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, ModelVertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

		glGenBuffers(1, &ModelUVBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, ModelUVBuffer);
		glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), uvs.data(), GL_STATIC_DRAW);

		glGenBuffers(1, &ModelNormalBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, ModelNormalBuffer);
		glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);

		ModelData data = ModelData();
		data.ModelNormalBuffer = ModelNormalBuffer;
		data.ModelUVBuffer = ModelUVBuffer;
		data.ModelVertexBuffer = ModelVertexBuffer;
		data.vertices = vertices;
		data.normals = normals;
		data.uvs = uvs;
		data.size = vertices.size();
		return data;
	}
};
class VerticesObject:ModelTransformation {
public:
	std::vector<glm::dvec4> vertices;
	glm::mat4 ModelMatrix;
	GLuint vbo = 0;
	GLuint color;
	GLuint drawMode;
	glm::vec4 vcolor;
public:
	VerticesObject(std::vector<glm::dvec4> _vertices, GLuint _drawMode, glm::vec4 _color = glm::vec4(0.0, 1.0, 0.0, 1.0),
		glm::dmat4 modelMatrix = glm::dmat4()) {
		this->vertices = _vertices;
		this->vcolor = _color;
		this->drawMode = _drawMode;
		this->ModelMatrix = modelMatrix;
	}
	virtual std::vector<glm::dvec4> getVertices() {
		return this->vertices;
	}
	virtual void generateVBO() {
		glGenBuffers(1, &this->vbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->vbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, this->vertices.size() * sizeof(glm::dvec4), this->vertices.data(), GL_DYNAMIC_COPY);
		glGenBuffers(1, &this->color);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->color);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(glm::vec4), &this->vcolor.data, GL_DYNAMIC_COPY);
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
class IndexedVerticesObject:ModelTransformation {
public:
	std::vector<glm::dvec4> vertices;
	std::vector<unsigned int> indexes;
	std::vector<unsigned int> _indexes;
	GLuint vbo = 0;
	GLuint color;
	GLuint drawMode;
	GLuint IndexBuffer;
	glm::vec4 vcolor;
	glm::mat4 ModelMatrix;
public:
	IndexedVerticesObject(std::vector<glm::dvec4> _vertices, std::vector<unsigned int> _indexes,
		GLuint _drawMode, glm::vec4 _color = glm::vec4(0.0, 1.0, 0.0, 1.0), glm::dmat4 modelMatrix = glm::dmat4()) {
		this->vertices = _vertices;
		this->indexes = _indexes;
		this->vcolor = _color;
		this->drawMode = _drawMode;
		this->ModelMatrix = modelMatrix;
	}
	virtual std::vector<glm::dvec4> getVertices() {
		return this->vertices;
	}
	virtual void generateVBO() {
		glGenBuffers(1, &this->vbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->vbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, this->vertices.size() * sizeof(glm::dvec4), this->vertices.data(), GL_DYNAMIC_COPY);
		glGenBuffers(1, &this->color);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->color);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(glm::vec4), &this->vcolor.data, GL_DYNAMIC_COPY);
		glGenBuffers(1, &this->IndexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBuffer);
		for (int i = 0; i < this->indexes.size(); i++) {
			this->_indexes.push_back(this->indexes.at(i));
		}
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->_indexes.size() * sizeof(unsigned int), this->_indexes.data(), GL_DYNAMIC_COPY);
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
	ReferenceLine(std::vector<glm::dvec4> _vertices) :VerticesObject(_vertices, GL_LINE_STRIP) {

	}
	void generateVBO() {
		glGenBuffers(1, &this->vbo);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->vbo);
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
class IVehicleDynamics {
public:
	btDiscreteDynamicsWorld* dynamicsWorld;
	virtual void initPhysics(std::vector<IndexedVerticesObject*>& rro) = 0;
	virtual void setInitialTransform(btTransform _initialTransform) = 0;
	virtual void exitPhysics() = 0;
	virtual void stepSimulation(float deltaTime) = 0;
	virtual bool keyboardCallback(GLFWwindow* window) = 0;
	virtual btTransform getWheelTransformWS(int index) = 0;
	virtual btTransform getChassisTransform() = 0;
	virtual btVector3 getChassisCentreOfMass() = 0;
	virtual btVector3 getChassisDimensions() = 0;
	virtual btScalar getWheelWidth() = 0;
	virtual btScalar getWheelRadius() = 0;
};
class btUtils {
public:
	static btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape, btDiscreteDynamicsWorld* dynamicsWorld)
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif  //

		dynamicsWorld->addRigidBody(body);
		return body;
	}
	static void rotate_vector_by_quaternion(const btVector3& v, const btQuaternion& q, btVector3& vprime)
	{
		// Extract the vector part of the quaternion
		btVector3 u(q.x(), q.y(), q.z());

		// Extract the scalar part of the quaternion
		float s = q.w();

		// Do the math
		vprime = 2.0f * u.dot(v) * u
			+ (s * s - u.dot(u)) * v
			+ 2.0f * s * u.cross(v);
	}
	static btCollisionShape* generateRoads(std::vector<IndexedVerticesObject*>& rro) {
		btTriangleMesh* tMesh = new btTriangleMesh();
		for (int i = 0; i < rro.size(); i++)
		{
			IndexedVerticesObject* ivo = rro.at(i);
			for (int j = 0; j < ivo->indexes.size(); j += 3) {
				glm::dvec4 t1, t2, t3;
				t1 = ivo->vertices.at(ivo->indexes.at(j));
				t2 = ivo->vertices.at(ivo->indexes.at(j + 1));
				t3 = ivo->vertices.at(ivo->indexes.at(j + 2));

				tMesh->addTriangle(btVector3(t1.x, t1.y, t1.z), btVector3(t2.x, t2.y, t2.z), btVector3(t3.x, t3.y, t3.z));
			}
		}
		return new btBvhTriangleMeshShape(tMesh, true);
	}
};
#endif