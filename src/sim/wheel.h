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

#ifndef _SIM_WHEEL_H
#define _SIM_WHEEL_H

#include "sim/suspension.h"
#include "sim/shaft.h"
#include "sim/brake.h"
#include "sim/tire.h"
#include "sim/ray.h"

class btCollisionWorld;

namespace sim
{

struct WheelContact;
class FractureBody;

struct WheelInfo
{
	SuspensionInfo suspension;
	BrakeInfo brake;
	TireInfo tire;
	btScalar inertia;
	btScalar radius;
	btScalar width;
	btScalar mass;
};

class Wheel
{
public:
	Wheel();

	// execute before usage
	void init(
		const WheelInfo & info,
		btCollisionWorld & world,
		FractureBody & body);
	
	// wheel ray test
	// returns false if there is no contact
	bool updateContact(btScalar raylen = 2);
	
	// update contact and setup wheel constraint
	// returns false if there is no contact
	bool update(btScalar dt, WheelContact & contact);

	// wheel contact velocity
	//const btVector3 & getVelocity() const { return vAB; }

	// wheel (center) world space position
	const btVector3 &  getPosition() const { return transform.getOrigin(); }

	// wheel radius
	btScalar getRadius() const { return radius; }

	// wheel width
	btScalar getWidth() const { return width; }

	Suspension suspension;
	Shaft shaft;
	Brake brake;
	Tire tire;
	Ray ray;

private:
	btCollisionWorld * world;
	FractureBody * body;
	btTransform transform;
	btScalar radius;
	btScalar width;
	btScalar mass;
};

}

#endif
