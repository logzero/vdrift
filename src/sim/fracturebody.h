#ifndef _SIM_FRACTURE_BODY
#define _SIM_FRACTURE_BODY

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btAlignedObjectArray.h"

#define CUSTOM_FRACTURE_TYPE (btRigidBody::CO_USER_TYPE*2)

class btDynamicsWorld;
class btCompoundShape;

namespace sim
{

struct MotionState;
struct FractureBodyInfo;

class FractureBody : public btRigidBody
{
public:
	FractureBody(const FractureBodyInfo & info);

	// aply impulse to connection of the shape, return true if connection is activated
	bool applyImpulse(int shape_id, btScalar impulse);

	// if accumulated impulse exceeds connection strength, return true and set broken_con
	bool updateConnection(int shape_id, int& broken_con);

	// invalidate connection, separate child shape, return child
	btRigidBody* breakConnection(int con_id);

	// true if child connected
	bool isChildConnected(int i) const;

	// only applied if child is connected to body
	void setChildTransform(int i, const btTransform& transform);

	// synchronize child body states
	void updateState();

	// remove connections, remove child shapes from world
	void clear(btDynamicsWorld& world);

	// center of mass offset from original shape coordinate system
	const btVector3 & getCenterOfMassOffset() const
	{
		return m_centerOfMassOffset;
	}

	struct Connection;

private:
	btAlignedObjectArray<Connection> m_connections;
	btVector3 m_centerOfMassOffset;

	// motion state wrapper, updates children motion states
	class FrMotionState : public btMotionState
	{
	public:
		FrMotionState(FractureBody & body, btMotionState * state = 0);
		~FrMotionState();
		void getWorldTransform(btTransform & worldTrans) const;
		void setWorldTransform(const btTransform & worldTrans);

	private:
		FractureBody & m_body;
		btMotionState * m_state;
	};
	FrMotionState m_motionState;
};

struct FractureBodyInfo
{
	void addMass(
		const btVector3& position,
		btScalar mass);

	void addBody(
		const btTransform& localTransform,
		btCollisionShape* shape,
		const btVector3& inertia,
		btScalar mass,
		btScalar elasticLimit,
		btScalar plasticLimit);

	void addBody(
		int shape_id,
		const btVector3& inertia,
		btScalar mass,
		btScalar elasticLimit,
		btScalar plasticLimit);

    FractureBodyInfo(btAlignedObjectArray<MotionState*>& m_states);

	btCompoundShape* m_shape;
    btAlignedObjectArray<MotionState*>& m_states;
	btAlignedObjectArray<FractureBody::Connection> m_connections;
	btVector3 m_inertia;
	btVector3 m_massCenter;
	btScalar m_mass;
};

struct FractureBody::Connection
{
	btRigidBody* m_body;
	btScalar m_elasticLimit;
	btScalar m_plasticLimit;
	btScalar m_accImpulse;
	int m_shapeId;

	Connection() :
		m_body(0),
		m_elasticLimit(0),
		m_plasticLimit(0),
		m_accImpulse(0),
		m_shapeId(-1)
	{
		// ctor
	}
};

}

#endif