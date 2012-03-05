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

#include "sim/suspension.h"

namespace sim
{

SuspensionInfo::SuspensionInfo() :
	antiroll(8000),
	stiffness(50000),
	bounce(2500),
	rebound(4000),
	travel(0.2),
	max_steering_angle(0),
	ackermann(0),
	camber(0),
	caster(0),
	toe(0)
{
	// ctor
}

void Suspension::init(const SuspensionInfo & info)
{
	SuspensionInfo::operator=(info);
	position = info.hub;
	damping = info.bounce;
}

Suspension::Suspension() :
	orientation(btQuaternion::getIdentity()),
	position(0, 0, 0),
	steering_angle(0),
	displacement(0),
	damping(0)
{
	// ctor
}

void Suspension::setSteering(btScalar value)
{
	btScalar alpha = -value * max_steering_angle * SIMD_RADS_PER_DEG;
	steering_angle = 0.0;
	if (alpha != 0.0)
	{
		steering_angle = btAtan(1.0 / (1.0 / btTan(alpha) - btTan(ackermann * SIMD_RADS_PER_DEG)));
	}
	btVector3 caster_axis(btSin(-caster * SIMD_RADS_PER_DEG), 0, btCos(-caster * SIMD_RADS_PER_DEG));
	btVector3 toe_axis(0, 0, 1);
	btVector3 camber_axis(-1, 0, 0);
	btQuaternion s(caster_axis, steering_angle);
	btQuaternion t(toe_axis, toe * SIMD_RADS_PER_DEG);
	btQuaternion c(camber_axis, camber * SIMD_RADS_PER_DEG);
	hub_orientation = c * t * s;
}

void Suspension::setDisplacement(btScalar value)
{
	btScalar delta = value - displacement;
	damping = (delta > 0) ? bounce : rebound;
	displacement = value;
	btClamp(displacement, btScalar(0), travel);

	btVector3 up(0, 0, 1);
	btVector3 old_dir = hub - lower_arm.anchor;
	btVector3 new_dir = old_dir + up * displacement;
	new_dir.normalize();
	position = lower_arm.anchor + new_dir * lower_arm.length;
	orientation = hub_orientation;
/*
	//if (type == HINGE) fixme
	{
		btVector3 up(0, 0, 1);
		btVector3 old_dir = hub - lower_arm.anchor;
		btVector3 new_dir = old_dir + up * displacement;
		old_dir.normalize();
		new_dir.normalize();

		btVector3 xyz = old_dir.cross(new_dir);
		btScalar w = 1 + old_dir.dot(new_dir);
		btQuaternion rot(xyz.x(), xyz.y(), xyz.z(), w);

		position = lower_arm.anchor + new_dir * lower_arm.length;
		orientation = rot * hub_orientation;
	}
	else if (type == MACPHERSON)
	{
		// todo
	}
	else if (type == DWISHBONE)
	{
		// todo
	}
	else
	{
		btAssert(0);
	}*/
}

}
