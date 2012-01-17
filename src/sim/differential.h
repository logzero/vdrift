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

#ifndef _SIM_DIFFERENTIAL_H
#define _SIM_DIFFERENTIAL_H

#include "sim/shaft.h"

namespace sim
{

struct DifferentialInfo
{
	btScalar final_drive;			///< Gear ratio of the differential.
	btScalar anti_slip;				///< Maximum anti_slip torque.
	btScalar anti_slip_factor;		///< Anti_slip torque factor [0,1] for torque sensitive LSDs.
	btScalar deceleration_factor;	///< 0.0 for 1-way LSD, 1.0 for 2-way LSD, in between for 1.5-way LSD.
	btScalar torque_split;			///< Torque split factor, 0.0 applies all torque to side1, for epicyclic differentials.
	btScalar inertia;				///< Rotational inertia of differential + driving shaft
	DifferentialInfo();				///< Default constructor makes an S2000-like car.
};

class Differential : private DifferentialInfo
{
public:
	Differential();

	void init(const DifferentialInfo & info, Shaft & sha, Shaft & shb);

	btScalar getAntiSlipTorque() const { return anti_slip; }

	btScalar getFinalDrive() const { return final_drive; }

	Shaft & getShaft() { return shaft; }
	
	Shaft & getShaft1() { return *shaft_a; }
	
	Shaft & getShaft2() { return *shaft_b; }

private:
	Shaft shaft;
	Shaft * shaft_a;
	Shaft * shaft_b;
};

}

#endif
