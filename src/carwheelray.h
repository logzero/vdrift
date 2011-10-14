#ifndef _CARWHEELRAY_H
#define _CARWHEELRAY_H

#include "pntriangle.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

class BEZIER;
struct TRACKSURFACE;

struct CarWheelRay : public btCollisionWorld::RayResultCallback
{
	btVector3 m_rayFrom;
	btVector3 m_rayTo;
	btScalar m_rayLen;

	btVector3 m_hitNormal;
	btVector3 m_hitPoint;
	btScalar m_depth;

	const btCollisionObject * m_exclude;
	const TRACKSURFACE * m_surface;
	const BEZIER * m_patch;

	// Cached surface triangle
	Triangle m_triangle;
	int m_triangleId;

	CarWheelRay();

	const btVector3 & getPoint() const {return m_hitPoint;}

	const btVector3 & getNormal() const {return m_hitNormal;}

	const btScalar getDepth() const {return m_depth;}

	const TRACKSURFACE & getSurface() const {return *m_surface;}

	const BEZIER * getPatch() const {return m_patch;}

	void set(const btVector3 & rayFrom, const btVector3 & rayDir, btScalar rayLen);

	// Interpolate new contact from existing data (plane approximation).
	bool interpolate(const btVector3 & rayFrom, const btVector3 & rayDir, btScalar rayLen);

	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace);
};

#endif
