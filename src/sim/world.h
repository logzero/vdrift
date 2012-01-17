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

#ifndef _SIM_WORLD_H
#define _SIM_WORLD_H

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

namespace sim
{

class FractureBody;

struct Config
{
	btDefaultCollisionConfiguration config;
	btCollisionDispatcher dispatch;
	btDbvtBroadphase broadphase;
	btSequentialImpulseConstraintSolver solver;
	btScalar timeStep;
	int maxSubSteps;
	Config(btScalar timeStep = 1/60.0f, int maxSubSteps = 10) :
		dispatch(&config),
		timeStep(timeStep),
		maxSubSteps(maxSubSteps)
	{
		// ctor
	}
};

class World : public btDiscreteDynamicsWorld
{
public:
	World(Config & config);

	~World();

	struct RayResultCallback;
	void rayTest(const btVector3 & rayFromWorld, const btVector3 & rayToWorld, RayResultCallback & resultCallback) const;

	struct RayTestProcessor;
	void setRayTestProcessor(RayTestProcessor & rtp);

	void addCollisionObject(btCollisionObject * object);

	void update(btScalar dt);

	void reset();

protected:
	struct ActiveCon
	{
		ActiveCon() : body(0), id(-1) {}
		ActiveCon(FractureBody * body, int id) : body(body), id(id) {}
		FractureBody * body;
		int id;
	};
	btAlignedObjectArray<ActiveCon> m_activeConnections;
	btScalar timeStep;
	int maxSubSteps;

	RayTestProcessor * rayTestProc;

	void solveConstraints(btContactSolverInfo& solverInfo);

	void fractureCallback();
};

struct World::RayResultCallback : public btCollisionWorld::RayResultCallback
{
	RayResultCallback() : userPointer(0) {}
	void * userPointer;
};

struct World::RayTestProcessor : public World::RayResultCallback
{
	virtual void rayTest(const btVector3 & rayFromWorld, const btVector3 & rayToWorld, World::RayResultCallback& cb) = 0;
};

}

#endif
