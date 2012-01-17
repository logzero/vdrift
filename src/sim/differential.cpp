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

#include "sim/differential.h"

namespace sim
{

DifferentialInfo::DifferentialInfo() :
	final_drive(4.1),
	anti_slip(600.0),
	anti_slip_factor(0),
	deceleration_factor(0),
	torque_split(0.5),
	inertia(0.25)
{
	// ctor
}

Differential::Differential()
{
	init(DifferentialInfo(), Shaft::getFixed(), Shaft::getFixed());
}

void Differential::init(const DifferentialInfo & info, Shaft & sha, Shaft & shb)
{
	DifferentialInfo::operator=(info);
	shaft.setInertia(info.inertia);
	shaft_a = &sha;
	shaft_b = &shb;
}

}
/*
void Differential::ComputeWheelTorques(btScalar driveshaft_torque)
{
	// Determine torque from the anti-slip mechanism.
	btScalar anti_slip_torque = anti_slip;

    // Linear torque sensitivity.
    if (anti_slip_factor > 0)
		anti_slip_torque = anti_slip_factor * driveshaft_torque;

    // Determine behavior for deceleration.
	if (anti_slip_torque < 0)
		anti_slip_torque *= -deceleration_factor;

	anti_slip_torque = std::max(btScalar(0), anti_slip_torque);

	btScalar drag = current_anti_slip * (side1_speed - side2_speed);
	btClamp(drag, -anti_slip, anti_slip);

	btScalar torque = driveshaft_torque * final_drive;
	side1_torque = torque * (1 - torque_split) - drag;
	side2_torque = torque * torque_split + drag;
}
*/
