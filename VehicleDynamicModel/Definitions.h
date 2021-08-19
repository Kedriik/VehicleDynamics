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
//#include "windows.h"
#include "..\VehicleDynamicModel\Renderer\glm\glm\glm.hpp"
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
				vertices.push_back(glm::dvec4(unitVertices.at(k).x * radius, unitVertices.at(k).y * radius, h, 1));
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
			vertices.push_back(glm::dvec4(0, 0, h, 1));
			//normals.push_back(0);      normals.push_back(0);      normals.push_back(nz);
			normals.push_back(glm::dvec3(0, 0, nz));
			//texCoords.push_back(0.5f); texCoords.push_back(0.5f);
			texCoords.push_back(glm::dvec2(0.5, 0.5));

			for (int j = 0, k = 0; j < sectorCount; ++j, k++)
			{                    // vz
				vertices.push_back(glm::dvec4(unitVertices.at(k).x * radius, unitVertices.at(k).y * radius, h, 1));
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
};
class VerticesObject {
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
class IndexedVerticesObject {
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
#endif