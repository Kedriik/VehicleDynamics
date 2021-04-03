#pragma once
#include "../VehicleDynamicModel/VehicleDynamicModel/VehicleDynamicModel.h"
#include "OpenDriveDocument.h"
#include "Renderer/Renderer.h"


int main(int argc, char** argv)
{
	std::string openDriveFilePath;
	if (argc > 1) {
		openDriveFilePath = std::string(argv[1]);
	}
	else {
		throw "No opendrive document";
	}

	OpenDriveDocument odd = OpenDriveDocument(openDriveFilePath);
	odd.generateReferenceLines();
	
	Renderer renderer;
	renderer.init(1500, 1000);

	std::vector<VerticesObject*> verticesObjects;
	
	for (int i = 0; i < odd.getRoads().size(); i++) {
		Road r = odd.getRoads().at(i);
		for (int j = 0; j < r.getPlanView().geometries.size(); j++) {
			Geometry* g = r.getPlanView().geometries.at(j);
			VerticesObject * obj = new VerticesObject(g->vertices, GL_LINE_STRIP);
			obj->generateVBO();
			verticesObjects.push_back(obj);
		}

	}
	double gap = 5.0;
	int generate_gaps = false;
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
	std::cout << "Hello World!\n";
	for (auto vo : verticesObjects) {
		delete vo;
	}
}