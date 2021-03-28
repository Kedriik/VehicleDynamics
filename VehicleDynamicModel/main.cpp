#pragma once
#include "../VehicleDynamicModel/VehicleDynamicModel/VehicleDynamicModel.h"
#include "OpenDriveDocument.h"
#include "Renderer/Renderer.h"


int main()
{
	OpenDriveDocument odd = OpenDriveDocument();
	odd.generateReferenceLines();
	
	Renderer renderer;
	renderer.init(1500, 1000);

	std::vector<VerticesObject*> verticesObjects;
	
	for (int i = 0; i < odd.getGeometries().size(); i++) {
		VerticesObject* obj = new VerticesObject(odd.getGeometries().at(i)->vertices, GL_LINE_STRIP);
		obj->generateVBO();
		verticesObjects.push_back(obj);
	}
	double gap = 1.0;
	for (int i = 0; i < odd.getGeometries().size() - 1; i++) {
		glm::dvec4 curr_end_point, next_start_point;
		curr_end_point = odd.getGeometries().at(i)->vertices.at(odd.getGeometries().at(i)->vertices.size()-1);
		next_start_point = odd.getGeometries().at(i+1)->vertices.at(0);
		if (glm::distance(curr_end_point,next_start_point) >gap) {
			std::vector<glm::dvec4> v;
			v.push_back(curr_end_point);
			VerticesObject* obj = new VerticesObject(v, GL_POINTS,glm::vec4(1,0,0,1));
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
	
	renderer.addVerticesObjects(verticesObjects);
	renderer.launchLoop();
	btVehicleDynamics vehicleDynamics;
	vehicleDynamics.initPhysics();
	/*DebugDraw* dd = new DebugDraw();
	dd->init(800, 600);
	vehicleDynamics.m_dynamicsWorld->setDebugDrawer(dd);
	vehicleDynamics.m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);*/
	for (int i = 0; i < 10000; i++) {
		vehicleDynamics.m_dynamicsWorld->stepSimulation(10);
		/*glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		vehicleDynamics.m_dynamicsWorld->debugDrawWorld();
		glDisableVertexAttribArray(0);
		//gpuDebug();
		glfwSwapBuffers(dd->window);
		glfwPollEvents();*/
	}
	std::cout << "Hello World!\n";
	for (auto vo : verticesObjects) {
		delete vo;
	}
}