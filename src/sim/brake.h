#ifndef _SIM_BRAKE_H
#define _SIM_BRAKE_H

#include "LinearMath/btScalar.h"

namespace sim
{

struct BrakeInfo
{
	btScalar friction;		///< sliding coefficient of friction for the brake pads on the rotor
	btScalar max_pressure;	///< maximum allowed pressure
	btScalar radius;		///< effective radius of the rotor
	btScalar area;			///< area of the brake pads
	btScalar bias;			///< the fraction of the pressure to be applied to the brake
	btScalar handbrake;		///< the friction factor that is applied when the handbrake is pulled

	/// default constructor makes an S2000-like car
	BrakeInfo() :
		friction(0.73),
		max_pressure(4e6),
		radius(0.14),
		area(0.015),
		bias(1.0),
		handbrake(0.0)
	{
		// ctor
	}
};

class Brake
{
public:
	Brake() :
		max_torque(0),
		brake_factor(0),
		handbrake_factor(0)
	{
		// ctor
	}

	void init(const BrakeInfo & info)
	{
		max_torque = info.bias * info.max_pressure * info.area * info.friction * info.radius;
	}

	/// brake_factor ranges from 0.0 (no brakes applied) to 1.0 (brakes applied)
	void setBrakeFactor(btScalar value)
	{
		brake_factor = value;
	}

	/// from 0.0 (no brakes applied) to 1.0 (brakes applied)
	void setHandbrakeFactor(btScalar value)
	{
		handbrake_factor = value;
	}

	/// ciurrent brake factor
	btScalar getBrakeFactor() const
	{
		return brake_factor;
	}

	/// brake torque magnitude
	btScalar getTorque()
	{
		btScalar brake = brake_factor > handbrake * handbrake_factor ? brake_factor : handbrake * handbrake_factor;
		return max_torque * brake;
	}

private:
	btScalar max_torque;		///< maximum supported torque
	btScalar handbrake;			///< handbrake support, 0.0 no handbbrake
	btScalar brake_factor;		///< current brake factor
	btScalar handbrake_factor;	///< separate so that ABS does not get applied to the handbrake

};

}

#endif
