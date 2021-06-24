#include <iostream>
#include <chrono>
#include <thread>     
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
class btVehicleTuning;
class btCollisionShape;

#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "Renderer/DebugDraw.h"
//#include "OpenDriveDocument.h"
#include "OpenDriveDocument.h"
//keep the collision shapes, for deletion/cleanup

class btUtils {
public:
	static btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape, btDiscreteDynamicsWorld* dynamicsWorld)
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif  //

		dynamicsWorld->addRigidBody(body);
		return body;
	}
};
class btVehicleDynamics {
public:
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btRigidBody* m_carChassis;
	btCollisionShape* m_wheelShape;
	///////////////////////////

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
	//btBvhTriangleMeshShape p;
	btScalar m_oldPickingDist;
	bool useMCLPSolver = true;  //true
	
	void initPhysics(OpenDriveDocument& openDriveDocument)
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
		btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
		this->m_collisionShapes.push_back(groundShape);
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

		//m_dynamicsWorld->setGravity(btVector3(0,0,0));
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3(0, 0, 0));

		//either use heightfield or triangle mesh

		//create ground object
		btUtils::localCreateRigidBody(0, tr, groundShape, m_dynamicsWorld)->setRestitution(1.0);

		btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
		m_collisionShapes.push_back(chassisShape);

		btCompoundShape* compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);
		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0, 1, 0));

		compound->addChildShape(localTrans, chassisShape);
		btCollisionShape* boxShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
		btTransform box_tr;
		box_tr.setIdentity();
		box_tr.setOrigin(btVector3(0, 200, 10));
		btRigidBody* box = btUtils::localCreateRigidBody(100, box_tr, boxShape, m_dynamicsWorld);
		box->getOrientation();
		box->setRestitution(0.5);
		{
			btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
			btTransform suppLocalTrans;
			suppLocalTrans.setIdentity();
			//localTrans effectively shifts the center of mass with respect to the chassis
			suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
			compound->addChildShape(suppLocalTrans, suppShape);
		}

		const btScalar FALLHEIGHT = 15;
		tr.setOrigin(btVector3(0, FALLHEIGHT, 0));

		const btScalar chassisMass = 2.0f;
		const btScalar wheelMass = 1.0f;
		m_carChassis = btUtils::localCreateRigidBody(chassisMass, tr, compound, m_dynamicsWorld);  //chassisShape);
		//m_carChassis->setDamping(0.2,0.2);

		//m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
		m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

		btVector3 wheelPos[4] = {
			btVector3(btScalar(-1.), btScalar(FALLHEIGHT - 0.25), btScalar(1.25)),
			btVector3(btScalar(1.), btScalar(FALLHEIGHT - 0.25), btScalar(1.25)),
			btVector3(btScalar(1.), btScalar(FALLHEIGHT - 0.25), btScalar(-1.25)),
			btVector3(btScalar(-1.), btScalar(FALLHEIGHT - 0.25), btScalar(-1.25)) };

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

			btRigidBody* pBodyB = btUtils::localCreateRigidBody(wheelMass, tr, m_wheelShape, m_dynamicsWorld);
			pBodyB->setFriction(1110);
			pBodyB->setActivationState(DISABLE_DEACTIVATION);
			pBodyB->setRestitution(0.5);
			// add some data to build constraint frames
			btVector3 parentAxis(0.f, 1.f, 0.f);
			btVector3 childAxis(1.f, 0.f, 0.f);
			btVector3 anchor = tr.getOrigin();
			btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
			//m_guiHelper->get2dCanvasInterface();
			//pHinge2->setLowerLimit(-SIMD_HALF_PI * 0.5f);
			//pHinge2->setUpperLimit(SIMD_HALF_PI * 0.5f);
			// add constraint to world
			m_dynamicsWorld->addConstraint(pHinge2, true);
			// Drive engine.
			pHinge2->enableMotor(3, true);
			pHinge2->setMaxMotorForce(3, 1000);
			pHinge2->setTargetVelocity(3, 0);
			pHinge2->setTargetVelocity(3, 2);
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

		//resetForklift();

		//m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	}
};



