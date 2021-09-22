#pragma once
#ifndef VehicleDynamicHeader
#define VehicleDynamicHeader
#include <iostream>
#include <chrono>
#include <thread>     
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include <map>
#include <math.h>
class btVehicleTuning;
class btCollisionShape;

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "Renderer/DebugDraw.h"
#include "Definitions.h"
//keep the collision shapes, for deletion/cleanup

class btVehicleDynamics {
public:
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	std::vector<btRigidBody*> wheels;
	btRigidBody* m_carChassis;
	btCollisionShape* m_wheelShape;
	///////////////////////////
	btVector3 chassisDimensions = btVector3(1.5f, 1.0f, 0.5f);
	float gVehicleSteering = 0.f;
	float steeringIncrement = 0.04f;
	float steeringClamp = 0.3f;
	float wheelRadius = 0.5f;
	float wheelWidth = 0.4f;
	//data for picking objects
	class btRigidBody* m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	int m_savedState;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btVector3 initialCarPosition = btVector3(5, -10, 2);
	//btBvhTriangleMeshShape p;
	btScalar m_oldPickingDist;
	bool useMCLPSolver = true;  //true
	std::vector<btHinge2Constraint*> wheelsHinges;

	btRigidBody* generateRoad(std::vector<IndexedVerticesObject*>& rro) {
		btTriangleMesh* tMesh = new btTriangleMesh();
		for (int i = 0; i < rro.size(); i ++)
		{
			IndexedVerticesObject *ivo = rro.at(i);
			for (int j = 0; j < ivo->indexes.size(); j+=3){
				glm::dvec4 t1, t2, t3;
				t1 = ivo->vertices.at(ivo->indexes.at(j  ));
				t2 = ivo->vertices.at(ivo->indexes.at(j+1));
				t3 = ivo->vertices.at(ivo->indexes.at(j+2));
				
				tMesh->addTriangle(btVector3(t1.x,t1.y,t1.z), btVector3(t2.x,t2.y,t2.z), btVector3(t3.x,t3.y,t3.z));
			}
		}
		btCollisionShape* groundShape = new btBvhTriangleMeshShape(tMesh, true);
		btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0));
		auto localRigidBody = btUtils::localCreateRigidBody(0, tr, groundShape, m_dynamicsWorld);
		localRigidBody->setRestitution(1.0);
		//localRigidBody->setGravity(btVector3(10,0,0));
		return localRigidBody;
	}
	void initPhysics(std::vector<IndexedVerticesObject*>& rro)
	{
		/*numTriangles: number of triangles
		triangleIndexBase : the array of vertex indices that makes up the triangles
		triangleIndexStride : the number of bytes to skip in the vertex indices array to go from the start of one triangle to the start of the next triangle.Typically this is 3 times the sizeof the vertex index type.
		numVertices : number of vertices
		vertexBase : the array of vertex positions
		vertexStride : the number of bytes to skip in the vertex position array to go from the start of one vertex to the start of the next vertex.If the vertex position is composed of 3 floats for example, then this would be 3 * sizeof(float).If it is 3 doubles with 1 double as padding, then 4 * sizeof(double), for example
		*/
		//btStridingMeshInterface* roadInterface = new btStridingMeshInterface();
		//btCollisionShape* road = new btBvhTriangleMeshShape()
		
		//btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
		//this->m_collisionShapes.push_back(groundShape);
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
		btVector3 worldMin(-1000, -1000, -1000);
		btVector3 worldMax(1000, 1000, 1000);
		m_broadphase = new btAxisSweep3(worldMin, worldMax);
		
		if (useMCLPSolver)
		{
			btDantzigSolver* mlcp = new btDantzigSolver();
			//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
			btMLCPSolver* sol = new btMLCPSolver(mlcp);
			m_solver = sol;
		}
		else
		{
			m_solver = new btSequentialImpulseConstraintSolver();
		}
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
		if (useMCLPSolver)
		{
			m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;  //for direct solver it is better to have a small A matrix
		}
		else
		{
			m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
		}
		m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
		//m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

		m_dynamicsWorld->setGravity(btVector3(0,0,-9.81));
		this->generateRoad(rro);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0.3)+ initialCarPosition);
		//tr.setOrigin(-chassisDimensions);

		//either use heightfield or triangle mesh

		//create ground object
		//btUtils::localCreateRigidBody(0, tr, groundShape, m_dynamicsWorld)->setRestitution(1.0);

		btCollisionShape* chassisShape = new btBoxShape(chassisDimensions);
		m_collisionShapes.push_back(chassisShape);

		btCompoundShape* compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);
		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0, 0, 0));

		compound->addChildShape(localTrans, chassisShape);
		{
			btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.1f));
			btTransform suppLocalTrans;
			suppLocalTrans.setIdentity();
			//localTrans effectively shifts the center of mass with respect to the chassis
			suppLocalTrans.setOrigin(btVector3(0.0, 0.0, 0.3));
			compound->addChildShape(suppLocalTrans, suppShape);
		}
		
		
		//const btScalar xOffset = 20;
		//const btScalar zOffset = 0;
		//const btScalar FALLHEIGHT = 10;
		//tr.setOrigin(btVector3(xOffset, zOffset, FALLHEIGHT));
		
		const btScalar chassisMass = 2.0f;
		const btScalar wheelMass = 1.0f;
		m_carChassis = btUtils::localCreateRigidBody(chassisMass, tr, compound, m_dynamicsWorld);
		m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

		btVector3 wheelPos[4] = {
			btVector3(btScalar( 1.5), btScalar( 1.0), 0) + initialCarPosition,
			btVector3(btScalar( 1.5), btScalar(-1.0), 0) + initialCarPosition,
			btVector3(btScalar(-1.5), btScalar( 1.0), 0) + initialCarPosition,
			btVector3(btScalar(-1.5), btScalar(-1.0), 0) + initialCarPosition };

		for (int i = 0; i < 4; i++)
		{
			// create a Hinge2 joint
			// create two rigid bodies
			// static bodyA (parent) on top:

			btRigidBody* pBodyA = m_carChassis;
			pBodyA->setActivationState(DISABLE_DEACTIVATION);
			// dynamic bodyB (child) below it :
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(wheelPos[i]);
			btQuaternion initialRot = btQuaternion(btVector3(0, 0,1), 1.5708);
			tr.setRotation(initialRot);

			btRigidBody* pBodyB = btUtils::localCreateRigidBody(wheelMass, tr, m_wheelShape, m_dynamicsWorld);
			wheels.push_back(pBodyB);
			pBodyB->setFriction(1110);
			pBodyB->setActivationState(DISABLE_DEACTIVATION);
			pBodyB->setRestitution(0.5);
			// add some data to build constraint frames
			btVector3 parentAxis(0.f, 0.f, 1.f);
			btVector3 childAxis(0.f, 1.f, 0.f);
			btVector3 anchor = tr.getOrigin();
			btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
			wheelsHinges.push_back(pHinge2);
			//m_guiHelper->get2dCanvasInterface();
			//pHinge2->setLowerLimit(-SIMD_HALF_PI * 0.5f);
			//pHinge2->setUpperLimit(SIMD_HALF_PI * 0.5f);
			// add constraint to world
			m_dynamicsWorld->addConstraint(pHinge2, true);
			// Drive engine.
			
			//pHinge2->enableMotor(3, true);
			//pHinge2->setMaxMotorForce(3, 1000);
			//pHinge2->setTargetVelocity(3, 0);
//			pHinge2->setTargetVelocity(3, 2);
			
			// Steering engine.
			pHinge2->enableMotor(5, true);
			pHinge2->setMaxMotorForce(5, 1000);
			pHinge2->setTargetVelocity(5, 0);
			pHinge2->setParam(BT_CONSTRAINT_CFM, 0.15f, 2);
			pHinge2->setParam(BT_CONSTRAINT_ERP, 0.35f, 2);
			pHinge2->setDamping(2, 2.0);
			pHinge2->setStiffness(2, 40.0);
			pHinge2->setDbgDrawSize(btScalar(5.f));
		}

		
	}
};

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

class CarHandlingDemo: public IVehicleDynamics
{
public:
	CarHandlingDemo() {};

	void initPhysics(std::vector<IndexedVerticesObject*>& rro);
	void setInitialTransform(btTransform _initialTransform);
	void exitPhysics()override;

	virtual ~CarHandlingDemo();

	virtual void stepSimulation(float deltaTime)override;

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool keyboardCallback(GLFWwindow* window);

	virtual void physicsDebugDraw(int debugFlags);

	virtual btTransform getWheelTransformWS(int index);

	virtual btTransform getChassisTransform();

	virtual btVector3 getChassisCentreOfMass();

	virtual void resetCamera()
	{
		float dist = 5 * 8;
		float pitch = -45;
		float yaw = 32;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
		//guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	virtual btVector3 getChassisDimensions() {
		return chassisDimensions;
	}
	virtual btScalar getWheelWidth() {
		return wheelWidth;
	}
	virtual btScalar getWheelRadius() {
		return wheelRadius;
	}


public:
	btRigidBody* generateRoad(std::vector<IndexedVerticesObject*>& rro);

	btRaycastVehicle* vehicle;

	btAlignedObjectArray<btCollisionShape*> collisionShapes;


	btRigidBody* createGroundRigidBodyFromShape(btCollisionShape* groundShape);

	btRigidBody* createChassisRigidBodyFromShape(btCollisionShape* chassisShape);
	btTransform initialTransform;
	btRigidBody* chassisRigidBody;
	btVector3 chassisDimensions = btVector3(1.5f, 1.0f, 0.7f);
	btScalar wheelWidth = btScalar(0.4);
	btScalar wheelRadius = btScalar(0.5);
	void addWheels(
		btVector3* halfExtents,
		btRaycastVehicle* vehicle,
		btRaycastVehicle::btVehicleTuning tuning);
};


class CarHandling: public IVehicleDynamics
{
public:

	CarHandling() {
	
	};

	void initPhysics(std::vector<IndexedVerticesObject*>& rro);

	void setInitialTransform(btTransform _initialTransform);

	void exitPhysics();

	virtual ~CarHandling();

	virtual void stepSimulation(float deltaTime);

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool keyboardCallback(GLFWwindow* window);

	virtual void physicsDebugDraw(int debugFlags);

	virtual btTransform getWheelTransformWS(int index);

	virtual btTransform getChassisTransform();

	virtual btVector3 getChassisCentreOfMass();

	virtual btVector3 getChassisDimensions() {
		return chassisDimensions;
	}
	virtual btScalar getWheelWidth() {
		return wheelWidth;
	}
	virtual btScalar getWheelRadius() {
		return wheelRadius;
	}
	virtual float getCurrentSpeedKmHour()override {
		return this->vehicle->getCurrentSpeedKmHour();
	}
	virtual void resetCamera()
	{
		float dist = 5 * 8;
		float pitch = -45;
		float yaw = 32;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
		//guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
	}
	btTransform getChassisMassCenter() {
		return this->chassisRigidBody->getCenterOfMassTransform();
	}
	void updateVehiclePhysics();
public:
	btRigidBody* generateRoad(std::vector<IndexedVerticesObject*>& rro);

	btRaycastVehicle* vehicle;

	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	btRigidBody* createGroundRigidBodyFromShape(btCollisionShape* groundShape);

	btRigidBody* createChassisRigidBodyFromShape(btCollisionShape* chassisShape);
	btTransform initialTransform;
	btRigidBody* chassisRigidBody;
	btVector3 chassisDimensions = btVector3(1.5f, 1.0f, 0.7f);
	btScalar wheelWidth = btScalar(0.4);
	btScalar wheelRadius = btScalar(0.5);
	btScalar brakingForceIncrement = 1000;
	btScalar engineForceIncrement = 5000;
	btScalar steeringIncrement = 1;
	btScalar maxBrakingForce = 5000;
	btScalar minBrakingForce = 10;
	btScalar maxEngineForce = 5000;
	btScalar minEngineForce = 0;
	btScalar maxSteering = 0.6;
	btScalar currentSteeringAngle = 0;
	btScalar currentAcceleratorPosition = 0;
	btScalar currentAccelerationForce = 0;
	btScalar currentBrakingForce = 0;
	btScalar currentBrakePosition = 0;
	std::map<int, double> input;
	double deltaTime = 0;
	void steerLeft() {
		currentSteeringAngle = std::min(currentSteeringAngle + deltaTime * steeringIncrement, double(maxSteering));
	}
	void steerRight() {
		currentSteeringAngle = std::max(currentSteeringAngle - deltaTime * steeringIncrement, -double(maxSteering));
	}
	void letOffWheel() {
		//currentSteeringAngle = steeringIncrement*deltaTime
		if (currentSteeringAngle > 0) {
			currentSteeringAngle = std::max(0.0, currentSteeringAngle - deltaTime * steeringIncrement);
		}
		else {
			currentSteeringAngle = std::min(0.0, currentSteeringAngle + deltaTime * steeringIncrement);
		}
	}
	void accelerate() {
		letOffBrake();
		currentAccelerationForce = std::min(double(maxEngineForce), currentAccelerationForce + deltaTime * engineForceIncrement);
	}
	void letOffAccelerator() {
		currentAccelerationForce = 0;
	}
	void brake() {
		letOffAccelerator();
		currentBrakingForce = std::min(double(maxBrakingForce), currentBrakingForce + deltaTime * brakingForceIncrement);
	}
	void letOffBrake() {
		currentBrakingForce = 0;
	}
	void addWheels(
		btVector3* halfExtents,
		btRaycastVehicle* vehicle,
		btRaycastVehicle::btVehicleTuning tuning);
};

#endif



