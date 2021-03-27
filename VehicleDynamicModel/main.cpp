#pragma once
#include "../VehicleDynamicModel/VehicleDynamicModel/VehicleDynamicModel.h"
#include "OpenDriveDocument.h"
#include "Renderer/Renderer.h"


int main()
{
	OpenDriveDocument odd = OpenDriveDocument();
	odd.generateReferenceLines();

	Renderer renderer;
	renderer.init(1000, 1000);

	std::vector<VerticesObject*> verticesObjects;
	for (int i = 0; i < odd.getGeometries().size(); i++) {
		VerticesObject* obj = new VerticesObject(odd.getGeometries().at(i)->vertices);
		obj->generateVBO();
		verticesObjects.push_back(obj);
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