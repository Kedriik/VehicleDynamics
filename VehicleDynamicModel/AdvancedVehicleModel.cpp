#include "AdvancedVehicleModel.h"
Hinge2Vehicle::Hinge2Vehicle()
{
}

void Hinge2Vehicle::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			dynamicsWorld->removeRigidBody(body);
		}
		else
		{
			dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete dynamicsWorld;
	dynamicsWorld = 0;

	delete m_wheelShape;
	m_wheelShape = 0;

	//delete solver
	delete m_solver;
	m_solver = 0;

	//delete broadphase
	delete m_broadphase;
	m_broadphase = 0;

	//delete dispatcher
	delete m_dispatcher;
	m_dispatcher = 0;

	delete m_collisionConfiguration;
	m_collisionConfiguration = 0;
}

Hinge2Vehicle::~Hinge2Vehicle()
{
	//exitPhysics();
}

void Hinge2Vehicle::initPhysics(std::vector<IndexedVerticesObject*>& rro)
{
	btCollisionShape* groundShape = btUtils::generateRoads(rro);
	m_collisionShapes.push_back(groundShape);
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
	dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	if (useMCLPSolver)
	{
		dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;  //for direct solver it is better to have a small A matrix
	}
	else
	{
		dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;  //for direct solver, it is better to solve multiple objects together, small batches have high overhead
	}
	dynamicsWorld->getSolverInfo().m_numIterations = 100;

	dynamicsWorld->setGravity(btVector3(0,0,-9.810));
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -3, 0));

	//either use heightfield or triangle mesh

	//create ground object
	localCreateRigidBody(0, tr, groundShape);

	btVector3 temp_dimensions = btVector3(chassisDimensions.y(), chassisDimensions.z(), chassisDimensions.x()); //TODO: INVESTIGATE !
	btCollisionShape* chassisShape = new btBoxShape(temp_dimensions);
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 1, 0));

	compound->addChildShape(localTrans, chassisShape);

	{
		btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
		btTransform suppLocalTrans;
		suppLocalTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));
		compound->addChildShape(suppLocalTrans, suppShape);
	}

	const btScalar FALLHEIGHT = 5;
	tr.setOrigin(btVector3(0, 0, 0));

	const btScalar chassisMass = 2.0f;
	const btScalar wheelMass = 1.0f;
	m_carChassis = localCreateRigidBody(chassisMass, tr, compound);  //chassisShape);
	//m_carChassis->setDamping(0.2,0.2);

	//m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

	btVector3 wheelPos[4] = {
		btVector3(btScalar(-1.), btScalar( - 0.25), btScalar(1.25)),
		btVector3(btScalar(1.), btScalar( - 0.25), btScalar(1.25)),
		btVector3(btScalar(1.), btScalar( - 0.25), btScalar(-1.25)),
		btVector3(btScalar(-1.), btScalar( - 0.25), btScalar(-1.25)) };
	this->m_carChassis->setWorldTransform(initialTransform);
	for (int i = 0; i < 4; i++)
	{
		// create a Hinge2 joint
		// create two rigid bodies
		// static bodyA (parent) on top:
		btRigidBody* pBodyA = this->m_carChassis;
		pBodyA->setActivationState(DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(wheelPos[i]);
		tr = initialTransform * tr; 
		btRigidBody* pBodyB = localCreateRigidBody(wheelMass, tr, m_wheelShape);
		//pBodyB->translate(initialTransform.getOrigin());
		this->m_wheels.push_back(pBodyB);
		pBodyB->setFriction(1110);
		pBodyB->setActivationState(DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		btVector3 parentAxis(0.f, 1.f, 0.f); 
		btVector3 childAxis(1.f, 0.f, 0.f);
		btVector3 _parentAxis(0.f, 1.f, 0.f);
		btVector3 _childAxis(1.f, 0.f, 0.f);
		
		btUtils::rotate_vector_by_quaternion(_parentAxis, initialTransform.getRotation(), parentAxis);
		btUtils::rotate_vector_by_quaternion(_childAxis, initialTransform.getRotation(), childAxis);
		btVector3 anchor = tr.getOrigin();
		btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis.normalize(), childAxis.normalize());
		dynamicsWorld->addConstraint(pHinge2, true);

		// Drive engine.
		pHinge2->enableMotor(3, true);
		pHinge2->setMaxMotorForce(3, 1000);
		pHinge2->setTargetVelocity(3, -5);

		// Steering engine.
		pHinge2->enableMotor(5, true);
		pHinge2->setMaxMotorForce(5, 1000);
		pHinge2->setTargetVelocity(5, 0);

		pHinge2->setParam(BT_CONSTRAINT_CFM, 0.15f, 2);
		pHinge2->setParam(BT_CONSTRAINT_ERP, 0.35f, 2);

		pHinge2->setDamping(2, 10.0);
		pHinge2->setStiffness(2, 40.0);

		//pHinge2->setDbgDrawSize(btScalar(5.f));
	}
	
}

void Hinge2Vehicle::stepSimulation(float deltaTime)
{
	float dt = deltaTime;

	if (dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 2;

		int numSimSteps;
		numSimSteps = dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

		if (dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*)dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures += numFallbacks;
				printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
			}
			sol->setNumFallbacks(0);
		}

		//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
			}
			else
			{
				printf("Simulated (%i) steps\n", numSimSteps);
			}
		}
#endif  //VERBOSE_FEEDBACK
	}
}

bool Hinge2Vehicle::keyboardCallback(GLFWwindow* window)
{
	/*
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

	if (state)
	{
		if (isShiftPressed)
		{
		}
		else
		{
			switch (key)
			{
			case B3G_LEFT_ARROW:
			{
				handled = true;
				gVehicleSteering += steeringIncrement;
				if (gVehicleSteering > steeringClamp)
					gVehicleSteering = steeringClamp;

				break;
			}
			case B3G_RIGHT_ARROW:
			{
				handled = true;
				gVehicleSteering -= steeringIncrement;
				if (gVehicleSteering < -steeringClamp)
					gVehicleSteering = -steeringClamp;

				break;
			}
			case B3G_UP_ARROW:
			{
				handled = true;
				gEngineForce = maxEngineForce;
				gBreakingForce = 0.f;
				break;
			}
			case B3G_DOWN_ARROW:
			{
				handled = true;
				gEngineForce = -maxEngineForce;
				gBreakingForce = 0.f;
				break;
			}

			case B3G_F7:
			{
				handled = true;
				btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)dynamicsWorld;
				world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
				printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
				break;
			}
			case B3G_F6:
			{
				handled = true;
				//switch solver (needs demo restart)
				useMCLPSolver = !useMCLPSolver;
				printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

				delete m_solver;
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

				dynamicsWorld->setConstraintSolver(m_solver);

				//exitPhysics();
				//initPhysics();
				break;
			}

			case B3G_F5:
				handled = true;
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				break;
			}
		}
	}
	else
	{
	}*/
	return true;
}

btRigidBody* Hinge2Vehicle::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
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

