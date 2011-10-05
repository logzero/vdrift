#include "fracturebody.h"
#include "motionstate.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include <iostream>

template <typename T0, typename T1>
inline T0 cast(const T1 & t)
{
	union {T0 t0; T1 t1;} cast;
	cast.t1 = t;
	return cast.t0;
}

inline int getConId(const btCollisionShape & shape)
{
	return cast<int>(shape.getUserPointer()) - 1;
}
inline void setConId(btCollisionShape & shape, int id)
{
	shape.setUserPointer(cast<void*>(id + 1));
}

inline btVector3 getPrincipalInertia(const btVector3 & p, const btScalar & m)
{
	return m * btVector3(
		p.y() * p.y() + p.z() * p.z(),
		p.x() * p.x() + p.z() * p.z(),
		p.x() * p.x() + p.y() * p.y());
}

FractureBody::FractureBody(const Info& info) :
	btRigidBody(btRigidBodyConstructionInfo(info.m_mass, &info.m_states[0], info.m_shape, info.m_inertia)),
	m_connections(info.m_connections)
{
	m_internalType = CUSTOM_FRACTURE_TYPE | CO_RIGID_BODY;
}

bool FractureBody::applyImpulse(int shape_id, btScalar impulse)
{
	btCompoundShape* compound = static_cast<btCompoundShape*>(m_collisionShape);
	btAssert(shape_id >= 0 && shape_id < compound->getNumChildShapes());

	btCollisionShape* child_shape = compound->getChildShape(shape_id);
	int con_id = getConId(*child_shape);

	if (con_id >= 0)
	{
		btAssert(con_id < m_connections.size());
		bool activate = m_connections[con_id].m_accImpulse < 1E-3;
		m_connections[con_id].m_accImpulse += impulse;
		return activate;
	}
	return false;
}

bool FractureBody::updateConnection(int shape_id, int& broken_con)
{
	btCompoundShape* compound = static_cast<btCompoundShape*>(m_collisionShape);
	btAssert(shape_id >= 0 && shape_id < compound->getNumChildShapes());

	btCollisionShape* child_shape = compound->getChildShape(shape_id);
	int con_id = getConId(*child_shape);
	btAssert(con_id >= 0 && con_id < m_connections.size());

	Connection& connection = m_connections[con_id];
	if (connection.m_accImpulse > connection.m_elasticLimit)
	{
		if (connection.m_accImpulse > connection.m_plasticLimit)
		{
			broken_con = con_id;
			return true;
		}
		btScalar damage = connection.m_accImpulse - connection.m_elasticLimit;
		connection.m_elasticLimit -= damage * 0.5;
		connection.m_plasticLimit -= damage * 0.5;
	}
	connection.m_accImpulse = 0;
	return false;
}

btRigidBody* FractureBody::breakConnection(int con_id)
{
	btAssert(con_id >= 0 && con_id < m_connections.size());
	int shape_id = m_connections[con_id].m_shapeId;
	btCompoundShape* compound = static_cast<btCompoundShape*>(m_collisionShape);

	// Init child body.
	btRigidBody* child = m_connections[con_id].m_body;
	btTransform trans = getWorldTransform() * compound->getChildTransform(shape_id);
	child->setWorldTransform(trans);
	child->setLinearVelocity(getVelocityInLocalPoint(trans.getOrigin()-getCenterOfMassPosition()));
	child->setAngularVelocity(btVector3(0, 0, 0));
	setConId(*child->getCollisionShape(), -1);

	// Remove child shape.
	btAssert(child->getCollisionShape() == compound->getChildShape(shape_id));
	compound->removeChildShapeByIndex(shape_id);

	// Update shape id due to shape swapping.
	if (shape_id < compound->getNumChildShapes())
	{
		btCollisionShape* child_shape = compound->getChildShape(shape_id);
		con_id = getConId(*child_shape);
		if (con_id >= 0 && con_id < m_connections.size())
		{
			m_connections[con_id].m_shapeId = shape_id;
		}
	}

	return child;
}

void FractureBody::updateState()
{
	const btCompoundShape* compound = static_cast<btCompoundShape*>(m_collisionShape);
	for (int i = 0; i < compound->getNumChildShapes(); ++i)
	{
		int con_id = getConId(*compound->getChildShape(i));
		if (con_id >= 0)
		{
			btAssert(con_id < m_connections.size());
			btTransform transform;
			getMotionState()->getWorldTransform(transform);
			transform = transform * compound->getChildTransform(i);
			m_connections[con_id].m_body->getMotionState()->setWorldTransform(transform);
		}
	}
}

bool FractureBody::isChildConnected(int i) const
{
	btAssert(i >= 0 && i < m_connections.size());
	return !m_connections[i].m_body->isInWorld();
}

void FractureBody::updateChildTransform(int i, const btTransform& transform)
{
	btAssert(i >= 0 && i < m_connections.size());
	if (!m_connections[i].m_body->isInWorld())
	{
		btCompoundShape* compound = static_cast<btCompoundShape*>(m_collisionShape);
		int shape_id = m_connections[i].m_shapeId;
		compound->updateChildTransform(shape_id, transform, false);
	}
}

void FractureBody::clear(btDynamicsWorld& world)
{
	for (int i = 0; i < m_connections.size(); ++i)
	{
		btRigidBody* cb = m_connections[i].m_body;
		btAssert(!cb->getCollisionShape()->isCompound());
		if (cb->isInWorld())
		{
			world.removeRigidBody(cb);
		}
		delete cb->getCollisionShape();
		delete cb;
	}
	m_connections.clear();
}

FractureBody::Info::Info(btAlignedObjectArray<MotionState>& states) :
	m_shape(new btCompoundShape(false)),
	m_states(states),
	m_inertia(0, 0, 0),
	m_massCenter(0, 0, 0),
	m_mass(0)
{
	// ctor
}

void FractureBody::Info::addMass(
	const btVector3& position,
	btScalar mass)
{
	m_inertia += getPrincipalInertia(position, mass);
	m_massCenter += position * mass;
	m_mass += mass;
}

void FractureBody::Info::addBody(
	int shape_id,
	const btVector3& inertia,
	btScalar mass,
	btScalar elasticLimit,
	btScalar plasticLimit)
{
	btCollisionShape* shape = m_shape->getChildShape(shape_id);
	btAssert(!shape->isCompound()); // compound children not suported

	setConId(*shape, m_connections.size());

	btVector3 shape_inertia(inertia);
	if (inertia.isZero())
	{
		shape->calculateLocalInertia(mass, shape_inertia);
	}

	btRigidBodyConstructionInfo info(mass, 0, shape, shape_inertia);
	btRigidBody* body = new btRigidBody(info);

	Connection connection;
	connection.m_body = body;
	connection.m_elasticLimit = elasticLimit;
	connection.m_plasticLimit = plasticLimit;
	connection.m_accImpulse = 0;
	connection.m_shapeId = shape_id;
	m_connections.push_back(connection);
}

void FractureBody::Info::addBody(
	const btTransform& localTransform,
	btCollisionShape* shape,
	const btVector3& inertia,
	btScalar mass,
	btScalar strength,
	btScalar elasticLimit)
{
	m_shape->addChildShape(localTransform, shape);
	addBody(m_shape->getNumChildShapes(), inertia, mass, strength, elasticLimit);
}

void FractureBody::Info::init(
	const btVector3& position,
	const btQuaternion& rotation)
{
	m_massCenter = m_massCenter / m_mass;
	m_inertia = m_inertia - getPrincipalInertia(m_massCenter, m_mass);

	for (int i = 0; i < m_shape->getNumChildShapes(); ++i)
	{
		m_shape->getChildTransform(i).getOrigin() -= m_massCenter;
	}
	m_shape->recalculateLocalAabb();

	m_states.resize(1 + m_connections.size());
	m_states[0].massCenterOffset = -m_massCenter;
	m_states[0].setWorldTransform(btTransform(rotation, position));
	for (int i = 0; i < m_connections.size(); ++i)
	{
		m_connections[i].m_body->setMotionState(&m_states[i+1]);
	}
}
