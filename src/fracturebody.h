#ifndef BT_FRACTURE_BODY
#define BT_FRACTURE_BODY

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#define CUSTOM_FRACTURE_TYPE (btRigidBody::CO_USER_TYPE*2)

struct MotionState;
class btDynamicsWorld;
class btCompoundShape;

class FractureBody : public btRigidBody
{
public:
	class Info;
	struct Connection;

	FractureBody(const Info& info);

	// aply impulse to connection of the shape, return true if connection is activated
	bool applyImpulse(int shape_id, btScalar impulse);

	// if accumulated impulse exceeds connection strength, return true and set broken_con
	bool updateConnection(int shape_id, int& broken_con);

	// invalidate connection, separate child shape, return child
	btRigidBody* breakConnection(int con_id);

	// true if child connected
	bool isChildConnected(int i) const;

	// only applied if child is connected to body
	void updateChildTransform(int i, const btTransform& transform);

	// to be called after adding bodies
	void updateMass();

	// synchronize child body states
	void updateState();

	// remove connections, remove child shapes from world
	void clear(btDynamicsWorld& world);

private:
	btAlignedObjectArray<Connection> m_connections;
};

class FractureBody::Info
{
public:
	void addMass(
		const btVector3& position,
		btScalar mass);

	void addBody(
		const btTransform& localTransform,
		btCollisionShape* shape,
		const btVector3& inertia,
		btScalar mass,
		btScalar strength,
		btScalar elasticLimit);

	void addBody(
		int shape_id,
		const btVector3& inertia,
		btScalar mass,
		btScalar strength,
		btScalar elasticLimit);

    // to be called before passing to fracture body
    void init(
        const btVector3& position,
        const btQuaternion& rotation);

    Info(btAlignedObjectArray<MotionState>& m_states);

	btCompoundShape* m_shape;
    btAlignedObjectArray<MotionState>& m_states;
	btAlignedObjectArray<Connection> m_connections;
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

#endif //BT_FRACTURE_BODY
