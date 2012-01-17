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

#ifndef _SIM_RAY_H
#define _SIM_RAY_H

#include "sim/world.h"

class BEZIER;

namespace sim
{

struct Surface;

struct Ray : public btCollisionWorld::RayResultCallback
{
	btVector3 m_rayFrom;
	btVector3 m_rayTo;
	btScalar m_rayLen;

	btVector3 m_hitNormal;
	btVector3 m_hitPoint;
	btScalar m_depth;

	const btCollisionObject * m_exclude;
	const Surface * m_surface;
	const BEZIER * m_patch;

	Ray();

	const btVector3 & getPoint() const {return m_hitPoint;}

	const btVector3 & getNormal() const {return m_hitNormal;}

	const btScalar getDepth() const {return m_depth;}

	const Surface * getSurface() const {return m_surface;}

	const BEZIER * getPatch() const {return m_patch;}

	void set(const btVector3 & rayFrom, const btVector3 & rayDir, btScalar rayLen);

	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace);
};

}

#endif
