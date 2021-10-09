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
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "Renderer/DebugDraw.h"
#include "Definitions.h"
class btVehicleTuning;
class btCollisionShape;
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
	btScalar Cdrag = 1.0;
	btVector3 lastCarPosition;
	std::map<int, double> input;
	double deltaTime = 0;
	btVector3 CarVelocity{ 0, 0, 0 };
	btVector3 getVelocity() {
		return CarVelocity;
	}
	void updateVelocity() {
		btVector3 currentCarPosition = this->vehicle->getChassisWorldTransform().getOrigin();
		btVector3 translation = currentCarPosition - lastCarPosition;
		CarVelocity = translation / deltaTime;
		lastCarPosition = currentCarPosition;
		//std::cout << "Last:(" << lastCarPosition.x() << "," << lastCarPosition.y() << "," << lastCarPosition.z() << ") ";
		//std::cout << "Current:(" << currentCarPosition.x() << "," << currentCarPosition.y() << "," << currentCarPosition.z() << ") ";
		//std::cout << "T:(" << translation.x() << "," << translation.y() << "," << translation.z() << ")" << "|" << deltaTime << std::endl ;
	}
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



