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

#ifndef _SIM_SHAFT_H
#define _SIM_SHAFT_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btMinMax.h"

namespace sim
{

class Shaft
{
public:
	Shaft(btScalar inertia = 1E37) :
		inertia(inertia),
		inertiaInv(1 / inertia),
		angularVelocity(0),
		angle(0)
    {
		// ctor
    }

    btScalar getAngle() const
    {
		return angle;
    }

	btScalar getAngularVelocity() const
	{
		return angularVelocity;
	}

	btScalar getInertiaInv() const
	{
		return inertiaInv;
	}

	btScalar getInertia() const
	{
		return inertia;
	}

	void setInertia(btScalar value)
	{
		inertia = value;
		inertiaInv = 1 / value;
	}

    void applyImpulse(btScalar impulse)
    {
        angularVelocity += inertiaInv * impulse;
    }

	void integrate(btScalar dt)
	{
		angle += angularVelocity * dt;
	}

	static Shaft & getFixed()
	{
		static Shaft fixed;
		return fixed;
	}

private:
    btScalar inertia;
    btScalar inertiaInv;
    btScalar angularVelocity;
    btScalar angle;
};

}

#endif
