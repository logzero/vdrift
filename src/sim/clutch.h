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
