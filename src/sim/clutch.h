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

#ifndef _SIM_CLUTCH_H
#define _SIM_CLUTCH_H

#include "LinearMath/btScalar.h"

namespace sim
{

struct ClutchInfo
{
	btScalar friction;		///< clutch sliding friction
	btScalar radius;		///< effective clutch radius
	btScalar area;			///< clutch surface area
	btScalar max_pressure;	///< maximum clutch pressure
	ClutchInfo() :			///< default constructor makes an S2000-like car
		friction(0.27),
		radius(0.15),
		area(0.75),
		max_pressure(11079.26)
	{
		// ctor
	}
};

class Clutch
{
public:

	Clutch() :
		torquemax(0),
		torque(0),
		position(0)
	{
		// ctor
	}

	void init(const ClutchInfo & info)
	{
		torquemax = info.friction * info.max_pressure * info.area * info.radius;
	}

	/// set the clutch engagement, where 1.0 is fully engaged
	void setPosition(const btScalar & value)
	{
		position = value;
		torque = torquemax * position;
	}

	/// clutch engage position
	btScalar getPosition() const
	{
		return position;
	}

	/// maximum supported torque
	btScalar getTorque() const
	{
		return torque;
	}

	/// clutch torque capacity
	btScalar getTorqueMax() const
	{
		return torquemax;
	}

private:
	btScalar torquemax;	///< max supported torque when fully engaged
	btScalar torque;	///< max supported torque at current postion
	btScalar position;	///< current clutch position value
};

}

#endif
