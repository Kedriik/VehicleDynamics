/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2015 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///May 2015: implemented the wheels using the Hinge2Constraint
///todo: add controls for the motors etc.

#include "Definitions.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
class btVehicleTuning;
class btCollisionShape;
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616
#endif

class Hinge2Vehicle : public IVehicleDynamics
{
public:
	btRigidBody* m_carChassis;
	std::vector<btRigidBody*> m_wheels;
	std::vector< btHinge2Constraint*> m_wheelsHinges;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);
	int m_wheelInstances[4];
	bool m_useDefaultCamera;
	//----------------------------
	class btTriangleIndexVertexArray* m_indexVertexArrays;
	btVector3* m_vertices;
	btCollisionShape* m_wheelShape;
	float m_cameraHeight;
	float m_minCameraDistance;
	float m_maxCameraDistance;
	std::vector<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	virtual void initPhysics(std::vector<IndexedVerticesObject*>& rro);
	virtual void setInitialTransform(btTransform _initialTransform) {
		this->initialTransform = _initialTransform;
	};
	virtual bool keyboardCallback(GLFWwindow* window);
	virtual btTransform getWheelTransformWS(int index) {
		return this->m_wheels.at(index)->getWorldTransform();
	}
	virtual btTransform getChassisTransform() {
		return this->m_carChassis->getWorldTransform();
	};
	virtual btVector3 getChassisCentreOfMass() {
		return this->m_carChassis->getCenterOfMassPosition();
	};
	virtual btVector3 getChassisDimensions() {
		return chassisDimensions;
	}
	virtual btScalar getWheelWidth() {
		return wheelWidth;
	}
	virtual btScalar getWheelRadius() {
		return wheelRadius;
	}
	void enableDrive() {
		/*
		// Drive engine.
		pHinge2->enableMotor(3, true);
		pHinge2->setMaxMotorForce(3, 1000);
		pHinge2->setTargetVelocity(3, 0);

		// Steering engine.
		pHinge2->enableMotor(5, true);
		pHinge2->setMaxMotorForce(5, 1000);
		pHinge2->setTargetVelocity(5, 0);
		*/


		m_wheelsHinges.at(2)->enableMotor(3, true);
		m_wheelsHinges.at(2)->setMaxMotorForce(3, 10);
		m_wheelsHinges.at(2)->setTargetVelocity(3, 0);

		m_wheelsHinges.at(3)->enableMotor(3, true);
		m_wheelsHinges.at(3)->setMaxMotorForce(3, 10);
		m_wheelsHinges.at(3)->setTargetVelocity(3, 0);

		m_wheelsHinges.at(2)->enableMotor(5, true);
		m_wheelsHinges.at(2)->setMaxMotorForce(5, 1000);
		m_wheelsHinges.at(2)->setTargetVelocity(5, 0);

		m_wheelsHinges.at(3)->enableMotor(5, true);
		m_wheelsHinges.at(3)->setMaxMotorForce(5, 1000);
		m_wheelsHinges.at(3)->setTargetVelocity(5, 0);
	}

	void enableSteer() {
		m_wheelsHinges.at(0)->enableMotor(5, true);
		m_wheelsHinges.at(0)->setMaxMotorForce(5, 1000);
		m_wheelsHinges.at(0)->setTargetVelocity(5, 0);

		m_wheelsHinges.at(1)->enableMotor(5, true);
		m_wheelsHinges.at(1)->setMaxMotorForce(5, 1000);
		m_wheelsHinges.at(1)->setTargetVelocity(5, 0);
	}
	void steerLeft() {
		m_wheelsHinges.at(0)->setTargetVelocity(5, -0.3);
		m_wheelsHinges.at(1)->setTargetVelocity(5, -0.3);
	}
	void steerRight() {
		m_wheelsHinges.at(0)->setTargetVelocity(5, 0.3);
		m_wheelsHinges.at(1)->setTargetVelocity(5, 0.3);
	}
	void letOffWheel() {
		m_wheelsHinges.at(0)->setTargetVelocity(5, 0);
		m_wheelsHinges.at(1)->setTargetVelocity(5, 0);
	}
	void accelerateToV(float V) {
		m_wheelsHinges.at(2)->enableMotor(3, true);
		m_wheelsHinges.at(3)->enableMotor(3, true);
		m_wheelsHinges.at(2)->setTargetVelocity(3, -V);
		m_wheelsHinges.at(3)->setTargetVelocity(3, -V);
	}
	void letOffAccelerator() {
		m_wheelsHinges.at(2)->enableMotor(3, false);
		m_wheelsHinges.at(3)->enableMotor(3, false);
	}
	void brake() {
		m_wheelsHinges.at(2)->setTargetVelocity(3, 0);
		m_wheelsHinges.at(3)->setTargetVelocity(3, 0);
	}
	Hinge2Vehicle();

	virtual ~Hinge2Vehicle();

	virtual void stepSimulation(float deltaTime);

	void exitPhysics();

	btTransform initialTransform;
	btScalar maxMotorImpulse = 4000.f;
	btVector3 wheelDirectionCS0 = btVector3(0, -1, 0);
	btVector3 wheelAxleCS = btVector3(-1, 0, 0);
	bool useMCLPSolver = false;
	float gEngineForce = 0.f;
	float defaultBreakingForce = 10.f;
	float gBreakingForce = 100.f;
	float maxEngineForce = 1000.f;  //this should be engine/velocity dependent
	float gVehicleSteering = 0.f;
	float steeringIncrement = 0.04f;
	float steeringClamp = 0.3f;
	float wheelRadius = 0.5f;
	float wheelWidth = 0.4f;
	btVector3 chassisDimensions = btVector3(1.5f, 1.0f, 0.7f);
};



#define CUBE_HALF_EXTENTS 1

////////////////////////////////////

