#include "VehicleDynamicModel/VehicleDynamicModel.h"
#include <math.h>

void CarHandling::initPhysics(std::vector<IndexedVerticesObject*>& rro)
{
	//Collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	//Use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	//btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	//The default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	//Creates the dynamics world, wich will be responsable for managing our physics objects and constraints
	this->dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	this->dynamicsWorld->setGravity(btVector3(0, 0, -9.81));
	btQuaternion(btVector3(1, 0, 0), 1.57);


	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	{
		//Creates the ground shape
		btTriangleMesh* tMesh = new btTriangleMesh();
		for (int i = 0; i < rro.size(); i++)
		{
			IndexedVerticesObject* ivo = rro.at(i);
			for (int j = 0; j < ivo->indexes.size(); j += 3) {
				glm::dvec4 t1, t2, t3;
				t1 = ivo->vertices.at(ivo->indexes.at(j));
				t2 = ivo->vertices.at(ivo->indexes.at(j + 1));
				t3 = ivo->vertices.at(ivo->indexes.at(j + 2));

				tMesh->addTriangle(btVector3(t1.x, t1.y, t1.z), btVector3(t2.x, t2.y, t2.z), btVector3(t3.x, t3.y, t3.z));
			}
		}
		btCollisionShape* groundShape = new btBvhTriangleMeshShape(tMesh, true);
		//Stores on an array for reusing
		collisionShapes.push_back(groundShape);

		//Creates the ground rigidbody
		btRigidBody* groundRigidBody = this->createGroundRigidBodyFromShape(groundShape);
		
		//Adds it to the world
		this->dynamicsWorld->addRigidBody(groundRigidBody);
	}

	{
		//the dimensions for the boxShape are half extents, so 0,5 equals to 
		btVector3 halfExtends(1, btScalar(0.5), 2);
		btVector3 temp_dimensions = btVector3(chassisDimensions.y(), chassisDimensions.z(), chassisDimensions.x()); //TODO: INVESTIGATE !
		//The btBoxShape is centered at the origin
		btCollisionShape* chassisShape = new btBoxShape(temp_dimensions);

		//Stores on an array for reusing
		collisionShapes.push_back(chassisShape);

		//A compound shape is used so we can easily shift the center of gravity of our vehicle to its bottom
		//This is needed to make our vehicle more stable
		btCompoundShape* compound = new btCompoundShape();

		collisionShapes.push_back(compound);

		btTransform localTransform;
		localTransform.setIdentity();
		localTransform.setOrigin(btVector3(0, 1, 0));

		//The center of gravity of the compound shape is the origin. When we add a rigidbody to the compound shape
		//it's center of gravity does not change. This way we can add the chassis rigidbody one unit above our center of gravity
		//keeping it under our chassis, and not in the middle of it
		compound->addChildShape(localTransform, chassisShape);
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
			suppLocalTrans.setOrigin(btVector3(0, 0.5, 0));
			compound->addChildShape(suppLocalTrans, suppShape);
		}

		//Creates a rigid body
		chassisRigidBody = this->createChassisRigidBodyFromShape(compound);

		//Adds the vehicle chassis to the world
		this->dynamicsWorld->addRigidBody(chassisRigidBody);

		btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(this->dynamicsWorld);

		btRaycastVehicle::btVehicleTuning tuning;
		

		//Creates a new instance of the raycast vehicle
		this->vehicle = new btRaycastVehicle(tuning, chassisRigidBody, vehicleRayCaster);

		//Never deactivate the vehicle
		chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);

		//Adds the vehicle to the world
		this->dynamicsWorld->addVehicle(this->vehicle);

		//Adds the wheels to the vehicle
		this->addWheels(&temp_dimensions, this->vehicle, tuning);
	}
	vehicle->setCoordinateSystem(0, 1, 2);
	lastCarPosition = this->vehicle->getChassisWorldTransform().getOrigin();

}
void CarHandling::setInitialTransform(btTransform _initialTransform)
{
	this->initialTransform = _initialTransform;
}
btRigidBody* CarHandling::createChassisRigidBodyFromShape(btCollisionShape* chassisShape)
{
	btTransform chassisTransform;
	chassisTransform.setIdentity();
	chassisTransform.setOrigin(btVector3(0, 0, 0));

	{
		//chassis mass 
		btScalar mass(1200);

		//since it is dynamic, we calculate its local inertia
		btVector3 localInertia(0, 0, 0);
		chassisShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* groundMotionState = new btDefaultMotionState(initialTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, groundMotionState, chassisShape, localInertia);

		return new btRigidBody(rbInfo);
	}
}
btRigidBody* CarHandling::createGroundRigidBodyFromShape(btCollisionShape* groundShape)
{
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	{
		//The ground is not dynamic, we set its mass to 0
		btScalar mass(0);

		//No need for calculating the inertia on a static object
		btVector3 localInertia(0, 0, 0);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* groundMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, groundMotionState, groundShape, localInertia);
		
		return new btRigidBody(rbInfo);
	}
}
void CarHandling::updateVehiclePhysics()
{
	updateVelocity();
	//std::cout << '\r' << "                     " <<  std::flush;
	//std::cout << vehicle->getCurrentSpeedKmHour() / 3.6 << "," << std::sqrt(std::pow(CarVelocity.x(),2)+ std::pow(CarVelocity.y(), 2)+ std::pow(CarVelocity.z(), 2)) << " m/s" << std::endl;
	
	//swaybar below
	if (this->vehicle->getWheelInfo(0).m_raycastInfo.m_suspensionLength < this->vehicle->getWheelInfo(1).m_raycastInfo.m_suspensionLength) {
		this->vehicle->getWheelInfo(1).m_raycastInfo.m_suspensionLength = this->vehicle->getWheelInfo(0).m_raycastInfo.m_suspensionLength;
	}
	else {
		this->vehicle->getWheelInfo(0).m_raycastInfo.m_suspensionLength = this->vehicle->getWheelInfo(1).m_raycastInfo.m_suspensionLength;
	}
	if (this->vehicle->getWheelInfo(2).m_raycastInfo.m_suspensionLength < this->vehicle->getWheelInfo(3).m_raycastInfo.m_suspensionLength) {
		this->vehicle->getWheelInfo(3).m_raycastInfo.m_suspensionLength = this->vehicle->getWheelInfo(2).m_raycastInfo.m_suspensionLength;
	}
	else {
		this->vehicle->getWheelInfo(2).m_raycastInfo.m_suspensionLength = this->vehicle->getWheelInfo(3).m_raycastInfo.m_suspensionLength;
	}
}
btRigidBody* CarHandling::generateRoad(std::vector<IndexedVerticesObject*>& rro) {
	btTriangleMesh* tMesh = new btTriangleMesh();
	for (int i = 0; i < rro.size(); i++)
	{
		IndexedVerticesObject* ivo = rro.at(i);
		for (int j = 0; j < ivo->indexes.size(); j += 3) {
			glm::dvec4 t1, t2, t3;
			t1 = ivo->vertices.at(ivo->indexes.at(j));
			t2 = ivo->vertices.at(ivo->indexes.at(j + 1));
			t3 = ivo->vertices.at(ivo->indexes.at(j + 2));

			tMesh->addTriangle(btVector3(t1.x, t1.y, t1.z), btVector3(t2.x, t2.y, t2.z), btVector3(t3.x, t3.y, t3.z));
		}
	}
	btCollisionShape* groundShape = new btBvhTriangleMeshShape(tMesh, true);
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, 0, 0));
	auto localRigidBody = btUtils::localCreateRigidBody(0, tr, groundShape, this->dynamicsWorld);
	//localRigidBody->setRestitution(1.0);
	//localRigidBody->setGravity(btVector3(10,0,0));
	return localRigidBody;
}
void CarHandling::addWheels(
	btVector3* halfExtents,
	btRaycastVehicle* vehicle,
	btRaycastVehicle::btVehicleTuning tuning)
{
	//The direction of the raycast, the btRaycastVehicle uses raycasts instead of simiulating the wheels with rigid bodies
	btVector3 wheelDirectionCS0(0, -1, 0);

	//The axis which the wheel rotates arround
	btVector3 wheelAxleCS(-1, 0, 0);

	btScalar suspensionRestLength(0.2);

	//The height where the wheels are connected to the chassis
	btScalar connectionHeight(0.7);

	//All the wheel configuration assumes the vehicle is centered at the origin and a right handed coordinate system is used
	btVector3 wheelConnectionPoint(2*halfExtents->x() - wheelRadius, connectionHeight, halfExtents->z() - wheelWidth);

	//Adds the front wheels
	vehicle->addWheel(wheelConnectionPoint * btVector3(1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

	vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, 1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, true);

	//Adds the rear wheels
	vehicle->addWheel(wheelConnectionPoint * btVector3(1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

	vehicle->addWheel(wheelConnectionPoint * btVector3(-1, 1, -1), wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, false);

	//Configures each wheel of our vehicle, setting its friction, damping compression, etc.
	//For more details on what each parameter does, refer to the docs
	for (int i = 0; i < vehicle->getNumWheels(); i++)
	{
		btWheelInfo& wheel = vehicle->getWheelInfo(i);
		wheel.m_suspensionStiffness = 150;
		wheel.m_wheelsDampingCompression =  btScalar(0.1) * 2 * btSqrt(wheel.m_suspensionStiffness);//btScalar(0.8);
		wheel.m_wheelsDampingRelaxation =  btScalar(0.5) * 2 * btSqrt(wheel.m_suspensionStiffness);//1;
		//Larger friction slips will result in better handling
		wheel.m_frictionSlip = btScalar(1.2);
		wheel.m_rollInfluence = 1;
	}
}
void CarHandling::exitPhysics()
{
	for (int i = this->dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
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
			this->dynamicsWorld->removeRigidBody(body);
		}
		else
		{
			this->dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		delete shape;
	}
	collisionShapes.clear();

	btConstraintSolver* constraintSolver = this->dynamicsWorld->getConstraintSolver();
	btBroadphaseInterface* broadphaseInterface = this->dynamicsWorld->getBroadphase();
	btDispatcher* collisionDispatcher = this->dynamicsWorld->getDispatcher();

	delete this->dynamicsWorld;
	delete this->vehicle;

	delete constraintSolver;
	delete broadphaseInterface;
	delete collisionDispatcher;
}
CarHandling::~CarHandling()
{
}
void CarHandling::physicsDebugDraw(int debugFlags)
{
	if (this->dynamicsWorld && this->dynamicsWorld->getDebugDrawer())
	{
		this->dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		this->dynamicsWorld->debugDrawWorld();
	}
}
btTransform CarHandling::getWheelTransformWS(int index)
{
	//this->vehicle->updateWheelTransform(index, true);
	return this->vehicle->getWheelInfo(index).m_worldTransform;// this->vehicle->getWheelTransformWS(index);
}
btTransform CarHandling::getChassisTransform()
{
	return this->chassisRigidBody->getWorldTransform();
}
void CarHandling::stepSimulation(float _deltaTime)
{
	deltaTime = _deltaTime;

	this->vehicle->setSteeringValue(currentSteeringAngle,0);
	this->vehicle->setSteeringValue(currentSteeringAngle,1);
	this->vehicle->setBrake(currentBrakingForce, 0);
	this->vehicle->setBrake(currentBrakingForce, 1);
	this->vehicle->setBrake(currentBrakingForce, 2);
	this->vehicle->setBrake(currentBrakingForce, 3);
	this->vehicle->applyEngineForce(this->currentAccelerationForce, 2);
	this->vehicle->applyEngineForce(this->currentAccelerationForce, 3);
	
	dynamicsWorld->stepSimulation(deltaTime, 1);
	this->updateVehiclePhysics();
}
bool CarHandling::keyboardCallback(GLFWwindow* window)
{
	bool handled = false;

	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		steerLeft();
		handled = true;
	}

	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		steerRight();
		handled = true;
	}

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		accelerate();
		handled = true;
	}

	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		brake();
		handled = true;
	}

	//Handbrake
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
	{
		handled = true;
	}
	//Key released events

	if (glfwGetKey(window, GLFW_KEY_LEFT) != GLFW_PRESS && glfwGetKey(window, GLFW_KEY_RIGHT) != GLFW_PRESS)
	{
		letOffWheel();
		handled = true;
	}

	if (glfwGetKey(window, GLFW_KEY_DOWN) != GLFW_PRESS && glfwGetKey(window, GLFW_KEY_UP) != GLFW_PRESS)
	{
		letOffAccelerator();
		letOffBrake();
		handled = true;
	}

	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) != GLFW_PRESS)
	{
		handled = true;
	}

	return handled;
}
btVector3 CarHandling::getChassisCentreOfMass()
{
	return this->chassisRigidBody->getCenterOfMassPosition();
}