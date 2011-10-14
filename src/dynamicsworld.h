#ifndef _DYNAMICSWORLD_H
#define _DYNAMICSWORLD_H

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

#include <ostream>

class FractureBody;

class DynamicsWorld  : public btDiscreteDynamicsWorld
{
public:
	DynamicsWorld(
		btDispatcher* dispatcher,
		btBroadphaseInterface* broadphase,
		btConstraintSolver* constraintSolver,
		btCollisionConfiguration* collisionConfig,
		btScalar timeStep = 1/60.0,
		int maxSubSteps = 10);

	~DynamicsWorld();

	void addRigidBody(btRigidBody * body);

	void removeRigidBody(btRigidBody * body);

	void update(btScalar dt);

	void reset();

	void rayTest(const btVector3 & rayFromWorld, const btVector3 & rayToWorld, RayResultCallback & resultCallback) const;

	// allow to register an external ray result processor
	struct RayTestProcessor : public RayResultCallback
	{
		virtual void rayTest(const btVector3 & rayFromWorld, const btVector3 & rayToWorld, RayResultCallback& cb) = 0;
	};
	void setRayTestProcessor(RayTestProcessor & rtp);

	void debugPrint(std::ostream & out) const;

protected:
	struct ActiveCon
	{
		ActiveCon() : body(0), id(-1) {}
		ActiveCon(FractureBody* body, int id) : body(body), id(id) {}
		FractureBody* body;
		int id;
	};
	btAlignedObjectArray<ActiveCon> m_activeConnections;
	btAlignedObjectArray<FractureBody*> m_fractureBodies;
	btScalar timeStep;
	int maxSubSteps;

	RayTestProcessor* rayTestProc;

	void solveConstraints(btContactSolverInfo& solverInfo);

	void fractureCallback();
};

#endif // _DYNAMICSWORLD_H
