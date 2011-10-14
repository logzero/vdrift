#include "carwheelray.h"
#include "tracksurface.h"
#include "trackshapeinfo.h"
#include "model.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

CarWheelRay::CarWheelRay() :
	m_exclude(0),
	m_surface(TRACKSURFACE::None()),
	m_patch(0),
	m_triangleId(-1)
{
	// Constructor
}

void CarWheelRay::set(const btVector3 & rayFrom, const btVector3 & rayDir, btScalar rayLen)
{
	m_rayFrom = rayFrom;
	m_rayTo = rayFrom + rayDir * rayLen;
	m_rayLen = rayLen;
	m_depth = rayLen;
	m_surface = TRACKSURFACE::None();
}

bool CarWheelRay::interpolate(const btVector3 & rayFrom, const btVector3 & rayDir, btScalar rayLen)
{
	btScalar nd = m_hitNormal.dot(rayDir);
	if (nd < 0)
	{
		m_depth = m_hitNormal.dot(m_hitPoint - rayFrom) / nd;
		m_hitPoint = rayFrom + rayDir * m_depth;
		return true;
	}
	m_hitPoint = rayFrom + rayDir * rayLen;
	m_depth = rayLen;
	return false;
}

btScalar CarWheelRay::addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
{
	if (rayResult.m_collisionObject == m_exclude) return 1.0;

	btCollisionShape * s = rayResult.m_collisionObject->getCollisionShape();
	TrackShapeInfo * si = static_cast<TrackShapeInfo*>(s->getUserPointer());

	if (normalInWorldSpace)
	{
		m_hitNormal = rayResult.m_hitNormalLocal;
	}
	else
	{
		m_hitNormal = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
	}
	m_hitPoint.setInterpolate3(m_rayFrom, m_rayTo, rayResult.m_hitFraction);
	m_collisionObject = rayResult.m_collisionObject;
	m_surface = si->surface;
	m_patch = si->patch;
/*
	// Point and normal interpolation
	const VERTEXARRAY & va = (si->model)->GetVertexArray();
	int i = rayResult.m_localShapeInfo->m_triangleIndex;
	if (m_triangleId != i)
	{
		std::vector<float> t0 = va.GetNormal(i, 0);
		std::vector<float> t1 = va.GetNormal(i, 1);
		std::vector<float> t2 = va.GetNormal(i, 2);
		std::vector<float> t3 = va.GetVertex(i, 0);
		std::vector<float> t4 = va.GetVertex(i, 1);
		std::vector<float> t5 = va.GetVertex(i, 2);
		btVector3 n0(t0[0], t0[1], t0[2]);
		btVector3 n1(t1[0], t1[1], t1[2]);
		btVector3 n2(t2[0], t2[1], t2[2]);
		btVector3 p0(t3[0], t3[1], t3[2]);
		btVector3 p1(t4[0], t4[1], t4[2]);
		btVector3 p2(t5[0], t5[1], t5[2]);
		m_triangle.set(p0, p1, p2, n0, n1, n2);
		m_triangleId = i;
	}

	btScalar u, v;
	m_triangle.getBarycentric(m_hitPoint, u, v);
	m_hitNormal = m_triangle.getNormal(u, v);
	m_hitPoint = m_triangle.getPoint(u, v);
*/
	btScalar f = getFraction(m_rayFrom, m_rayTo, m_hitPoint, m_hitNormal);
	rayResult.m_hitFraction = f;
	m_depth = m_rayLen * f;
	//std::cerr << rayResult.m_hitFraction << "/n";

	return rayResult.m_hitFraction;
}
