#pragma once
//#include "../VehicleDynamicModel/VehicleDynamicModel/VehicleDynamicModel.h"

#include "VehicleDynamicModel.h"
#include "OpenDriveDocument.h"
#include "Renderer/Renderer.h"
#include <exception>
int main(int argc, char** argv)
{
	std::string openDriveFilePath;
	if (argc > 1) {
		openDriveFilePath = std::string(argv[1]);
	}
	else {
		openDriveFilePath = "Ex_Line-Spiral-Arc.xodr";
	}
	int generate_gaps = false;
	if (argc > 3) {
		Road::S_STEP = std::stod(argv[2]);
		Road::T_STEP = std::stod(argv[3]);
	}
	else {
		Road::S_STEP = 10.1;
		Road::T_STEP = 1.1;
	}
	OpenDriveDocument odd = OpenDriveDocument(openDriveFilePath);
	//try {
		odd.parseOpenDriveDocument();
		//odd.generateReferenceLines();
		//odd.generateRoads();
		odd.GenerateRoadsLanes();
	//}
	//catch (std::exception e)
	//{
	//	std::cerr << "Error: " << e.what() << std::endl;
	//	return -1;
	//}
	
	Renderer renderer;
	renderer.init(1500, 1000);
	std::cout << "Generating Render Objects" << std::endl;
	using clock = std::chrono::system_clock;
	using sec = std::chrono::duration<double>;
	auto before = clock::now();
	std::vector<VerticesObject*> verticesObjects;
	std::vector<IndexedVerticesObject*> indexedVerticesObjects;
	std::vector<glm::dvec4> documentVertices;
	for (int i = 0; i < odd.getRoads().size(); i++) {
		Road r = odd.getRoads().at(i);
		VerticesObject * obj = new VerticesObject(r.getReferencePoints(), GL_LINE_STRIP,glm::vec4(1,0,0,1));
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
	std::cout << "Generating finished in " << duration.count() << "s" << std::endl;
	//idobj->generateVBO();
	//indexedVerticesObjects.push_back(idobj);
	double gap = 5.0;
	
	if(generate_gaps){
		for (int i = 0; i < odd.getRoads().size(); i++) {
			Road r = odd.getRoads().at(i);
			for (int j = 0; j < r.getPlanView().geometries.size()-1; j++) {
				Geometry* g = r.getPlanView().geometries.at(j);
				Geometry* ng = r.getPlanView().geometries.at(j+1);
				glm::dvec4 curr_end_point, next_start_point;
				curr_end_point = g->vertices.at(g->vertices.size() - 1);
				next_start_point = ng->vertices.at(0);
				if (glm::distance(curr_end_point, next_start_point) > gap) {
					std::vector<glm::dvec4> v;
					v.push_back(curr_end_point);
					VerticesObject* obj = new VerticesObject(v, GL_POINTS, glm::vec4(1, 0, 0, 1));
					obj->generateVBO();
					verticesObjects.push_back(obj);
				}
				else {
					std::vector<glm::dvec4> v;
					v.push_back(curr_end_point);
					VerticesObject* obj = new VerticesObject(v, GL_POINTS);
					obj->generateVBO();
					verticesObjects.push_back(obj);
				}
			}
		}
	}
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
	
	
	renderer.addVerticesObjects(verticesObjects);
	renderer.addIndexedVerticesObjects(indexedVerticesObjects);
	renderer.launchLoop();
	bool doPhysics = false;
	if(doPhysics){
		btVehicleDynamics vehicleDynamics;
		vehicleDynamics.initPhysics(odd);
		DebugDraw* dd = new DebugDraw();
		dd->init(800, 600);
		vehicleDynamics.m_dynamicsWorld->setDebugDrawer(dd);
		vehicleDynamics.m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		for (int i = 0; i < 10000; i++) {
			vehicleDynamics.m_dynamicsWorld->stepSimulation(10);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			dd->prepareFrame();
			vehicleDynamics.m_dynamicsWorld->debugDrawWorld();
			glDisableVertexAttribArray(0);
			//gpuDebug();
			glfwSwapBuffers(dd->window);
			glfwPollEvents();
		}
	}
	for (auto vo : verticesObjects) {
		delete vo;
	}
	for (auto ivo : indexedVerticesObjects) {
		delete ivo;
	}
}