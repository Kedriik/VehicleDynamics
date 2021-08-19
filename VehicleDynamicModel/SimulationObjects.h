#pragma once
#include "Renderer/Renderer.h"

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
		dmat4 modelMatrix = dmat4()) {
		this->vertices = _vertices;
		this->vcolor = _color;
		this->drawMode = _drawMode;
		this->ModelMatrix = modelMatrix;
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
		GLuint _drawMode, glm::vec4 _color = glm::vec4(0.0, 1.0, 0.0, 1.0), dmat4 modelMatrix = dmat4()) {
		this->vertices = _vertices;
		this->indexes = _indexes;
		this->vcolor = _color;
		this->drawMode = _drawMode;
		this->ModelMatrix = modelMatrix;
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
class PhysicsObject {
public:
	IndexedVerticesObject* rederObject;
	btTransform transform;
	btTransform initialTransform;
	PhysicsObject(IndexedVerticesObject* _rederObject) {
		this->rederObject = _rederObject;
	}
	~PhysicsObject() {

	}
};