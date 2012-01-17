/************************************************************************/
/*                                                                      */
/* This file is part of VDrift.                                         */
/*                                                                      */
/* VDrift is free software: you can redistribute it and/or modify       */
/* it under the terms of the GNU General Public License as published by */
/* the Free Software Foundation, either version 3 of the License, or    */
/* (at your option) any later version.                                  */
/*                                                                      */
/* VDrift is distributed in the hope that it will be useful,            */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of       */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        */
/* GNU General Public License for more details.                         */
/*                                                                      */
/* You should have received a copy of the GNU General Public License    */
/* along with VDrift.  If not, see <http://www.gnu.org/licenses/>.      */
/*                                                                      */
/************************************************************************/

#include "sim/world.h"
#include "sim/fracturebody.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

namespace sim
{

World::World(Config & config) :
	btDiscreteDynamicsWorld(
		&config.dispatch,
		&config.broadphase,
		&config.solver,
		&config.config),
	timeStep(config.timeStep),
	maxSubSteps(config.maxSubSteps),
	rayTestProc(0)
{
	setGravity(btVector3(0.0, 0.0, -9.81));
	setForceUpdateAllAabbs(false);
}

World::~World()
{
	reset();
}

void World::rayTest(
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

void World::setRayTestProcessor(RayTestProcessor & rtp)
{
	rayTestProc = &rtp;
}

void World::addCollisionObject(btCollisionObject * object)
{
	// disable shape drawing for meshes
	if (object->getCollisionShape()->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		int flags = object->getCollisionFlags();
		//flags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;
		flags |= btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT;
		object->setCollisionFlags(flags);
	}
	btDiscreteDynamicsWorld::addCollisionObject(object);
}

void World::update(btScalar dt)
{
	btDiscreteDynamicsWorld::stepSimulation(dt, maxSubSteps, timeStep);
	//CProfileManager::dumpAll();
}

void World::reset()
{
	getBroadphase()->resetPool(getDispatcher());
	m_nonStaticRigidBodies.resize(0);
	m_collisionObjects.resize(0);
	//rayTestProc = 0;
}

void World::solveConstraints(btContactSolverInfo& solverInfo)
{
	btDiscreteDynamicsWorld::solveConstraints(solverInfo);
	fractureCallback();
}

void World::fractureCallback()
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

}
