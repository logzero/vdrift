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

#ifndef _SIM_WHEELCONTACT_H
#define _SIM_WHEELCONTACT_H

#include "sim/constraintrow.h"

namespace sim
{

struct WheelContact
{
	ConstraintRow response;
	ConstraintRow friction1;
	ConstraintRow friction2;
	btRigidBody * bodyA;
	btRigidBody * bodyB;
	btVector3 rA;			// contact position relative to bodyA
	btVector3 rB;			// contact position relative to bodyB
	btScalar v1;			// velocity along longitudinal constraint
	btScalar v2;			// velocity along lateral constraint
	btScalar frictionCoeff; // surface friction coefficient
	btScalar camber;		// wheel camber in degrees
	btScalar vR;			// wheel rim velocity w * r
	int id;					// custom id
};

}

#endif