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
	if (argc > 2) {
		generate_gaps = int(argv[2]);
	}
	OpenDriveDocument odd = OpenDriveDocument(openDriveFilePath);
	//try {
		odd.parseOpenDriveDocument();
		//odd.generateReferenceLines();
		odd.generateRoads();
	//}
	//catch (std::exception e)
	//{
	//	std::cerr << "Error: " << e.what() << std::endl;
	//	return -1;
	//}
	
	
	Renderer renderer;
	renderer.init(1500, 1000);

	std::vector<VerticesObject*> verticesObjects;
	std::vector<IndexedVerticesObject*> indexedVerticesObjects;
	std::vector<glm::dvec4> documentVertices;
	for (int i = 0; i < odd.getRoads().size(); i++) {
		Road r = odd.getRoads().at(i);
		VerticesObject * obj = new VerticesObject(r.getReferencePoints(), GL_LINE_STRIP,glm::vec4(1,0,0,1));
		obj->generateVBO();
		verticesObjects.push_back(obj);

		obj = new VerticesObject(r._vertices, GL_POINTS, glm::vec4(0, 1, 0, 1));
		obj->generateVBO();
		verticesObjects.push_back(obj);
	}

	/*VerticesObject* obj = new VerticesObject(odd.road_vertices, GL_TRIANGLES, glm::dvec4(0, 0.7, 0, 1));
	obj->generateVBO();
	///verticesObjects.push_back(obj);

	VerticesObject* robj = new VerticesObject(odd.reference_line_vertices, GL_LINE_STRIP, glm::dvec4(1,0,0,1));
	robj->generateVBO();
	//verticesObjects.push_back(robj);

	VerticesObject* reobj = new VerticesObject(odd.road_right_edge, GL_LINE_STRIP, glm::dvec4(1, 1, 0, 1));
	reobj->generateVBO();
	//verticesObjects.push_back(reobj);

	VerticesObject* rvobj = new VerticesObject(odd.road_vertices, GL_POINTS, glm::dvec4(238.0 / 255.0, 130.0 / 255.0, 238.0 / 255.0, 1));
	rvobj->generateVBO();
	//verticesObjects.push_back(rvobj);

	IndexedVerticesObject* iobj = new IndexedVerticesObject(odd.road_vertices, odd.roadIndexes, GL_TRIANGLES);
	iobj->generateVBO();
	indexedVerticesObjects.push_back(iobj);
	*/
	for (auto r : odd.getRoads()) {
		IndexedVerticesObject* idobj = new IndexedVerticesObject(r.debugVertices, r.debugIndexes, GL_LINES, glm::dvec4(238.0 / 255.0, 130.0 / 255.0, 238.0 / 255.0, 1));
		idobj->generateVBO();
		indexedVerticesObjects.push_back(idobj);
	}
	for (auto ro : odd.roadRenderObjects) {
		ro->generateVBO();
		indexedVerticesObjects.push_back(ro);
		
	}
	
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
	renderer.addVerticesObjects(verticesObjects);
	renderer.addIndexedVerticesObjects(indexedVerticesObjects);
	renderer.launchLoop();
	btVehicleDynamics vehicleDynamics;
	vehicleDynamics.initPhysics();
	/*DebugDraw* dd = new DebugDraw();
	dd->init(800, 600);
	vehicleDynamics.m_dynamicsWorld->setDebugDrawer(dd);
	vehicleDynamics.m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);*/
	//for (int i = 0; i < 10000; i++) {
	//	vehicleDynamics.m_dynamicsWorld->stepSimulation(10);
		/*glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		vehicleDynamics.m_dynamicsWorld->debugDrawWorld();
		glDisableVertexAttribArray(0);
		//gpuDebug();
		glfwSwapBuffers(dd->window);
		glfwPollEvents();*/
	//}
	
	for (auto vo : verticesObjects) {
		delete vo;
	}
}