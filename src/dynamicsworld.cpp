#include "dynamicsworld.h"
#include "fracturebody.h"

DynamicsWorld::DynamicsWorld(
	btDispatcher* dispatcher,
	btBroadphaseInterface* broadphase,
	btConstraintSolver* constraintSolver,
	btCollisionConfiguration* collisionConfig,
	btScalar timeStep,
	int maxSubSteps) :
	btDiscreteDynamicsWorld(dispatcher, broadphase, constraintSolver, collisionConfig),
	timeStep(timeStep),
	maxSubSteps(maxSubSteps),
	rayTestProc(0)
{
	setGravity(btVector3(0.0, 0.0, -9.81));
	setForceUpdateAllAabbs(false);
}

DynamicsWorld::~DynamicsWorld()
{
	reset();
}

void DynamicsWorld::rayTest(
	const btVector3& rayFromWorld,
	const btVector3& rayToWorld,
	RayResultCallback& resultCallback) const
{
	if (rayTestProc)
	{
		rayTestProc->rayTest(rayFromWorld, rayToWorld, resultCallback);
		btDiscreteDynamicsWorld::rayTest(rayFromWorld, rayToWorld, *rayTestProc);
	}
	else
	{
		btDiscreteDynamicsWorld::rayTest(rayFromWorld, rayToWorld, resultCallback);
	}
}

void DynamicsWorld::update(btScalar dt)
{
	stepSimulation(dt, maxSubSteps, timeStep);
	//CProfileManager::dumpAll();
}

void DynamicsWorld::setRayTestProcessor(RayTestProcessor & rtp)
{
	rayTestProc = &rtp;
}

void DynamicsWorld::debugPrint(std::ostream & out) const
{
	out << "Collision objects: " << getNumCollisionObjects() << std::endl;
}

void DynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	// todo: after fracture we should run the solver again for better realism
	// for example
	//	save all velocities and if one or more objects fracture:
	//	1) revert all velocties
	//	2) apply impulses for the fracture bodies at the contact locations
	//	3) and run the constaint solver again
	btDiscreteDynamicsWorld::solveConstraints(solverInfo);
	fractureCallback();
}

void DynamicsWorld::addRigidBody(btRigidBody* body)
{
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		FractureBody* fbody = (FractureBody*)body;
		m_fractureBodies.push_back(fbody);
	}
	btDiscreteDynamicsWorld::addRigidBody(body);
}

void DynamicsWorld::removeRigidBody(btRigidBody* body)
{
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		FractureBody* fbody = (FractureBody*)body;
		m_fractureBodies.remove(fbody);
	}
	btDiscreteDynamicsWorld::removeRigidBody(body);
}

void DynamicsWorld::reset()
{
	getBroadphase()->resetPool(getDispatcher());
	m_nonStaticRigidBodies.resize(0);
	m_collisionObjects.resize(0);
	//rayTestProc = 0;
}

void DynamicsWorld::fractureCallback()
{
	m_activeConnections.resize(0);

	int numManifolds = getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; ++i)
	{
		btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts()) continue;

		if (((btCollisionObject*)manifold->getBody0())->getInternalType() & CUSTOM_FRACTURE_TYPE)
		{
			FractureBody* body = static_cast<FractureBody*>(manifold->getBody0());
			for (int k = 0; k < manifold->getNumContacts(); ++k)
			{
				btManifoldPoint& point = manifold->getContactPoint(k);
				int shape_id = point.m_index0;
				if (point.m_appliedImpulse > 1E-3 &&
					body->applyImpulse(shape_id, point.m_appliedImpulse))
				{
					m_activeConnections.push_back(ActiveCon(body, shape_id));
				}
			}
		}

		if (((btCollisionObject*)manifold->getBody1())->getInternalType() & CUSTOM_FRACTURE_TYPE)
		{
			FractureBody* body = static_cast<FractureBody*>(manifold->getBody1());
			for (int k = 0; k < manifold->getNumContacts(); ++k)
			{
				btManifoldPoint& point = manifold->getContactPoint(k);
				int shape_id = point.m_index1;
				if (point.m_appliedImpulse > 1E-3 &&
					body->applyImpulse(shape_id, point.m_appliedImpulse))
				{
					m_activeConnections.push_back(ActiveCon(body, shape_id));
				}
			}
		}
	}

	// Update active connections.
	btAlignedObjectArray<ActiveCon> brokenConnections;
	for (int i = 0; i < m_activeConnections.size(); ++i)
	{
		FractureBody* body = m_activeConnections[i].body;
		int shape_id = m_activeConnections[i].id;
		int broken_con = -1;
		if (body->updateConnection(shape_id, broken_con))
		{
			brokenConnections.push_back(ActiveCon(body, broken_con));
		}
	}

	// Separate pass to break connections, due to shape swapping on removal.
	for (int i = 0; i < brokenConnections.size(); ++i)
	{
		FractureBody* body = brokenConnections[i].body;
		int broken_con = brokenConnections[i].id;
		addRigidBody(body->breakConnection(broken_con));
	}
}
