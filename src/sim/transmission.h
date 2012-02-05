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

#ifndef _SIM_TRANSMISSION_H
#define _SIM_TRANSMISSION_H

#include "sim/shaft.h"

namespace sim
{

struct TransmissionInfo
{
	std::vector<btScalar> gear_ratios;	///< reverse + 1 neutral + forward
	btScalar shift_time;				///< transmission shift time
	int forward_gears;					///< number of consecutive forward gears
	int reverse_gears;					///< number of consecutive reverse gears

	TransmissionInfo() :
		gear_ratios(1, 0),
		shift_time(0),
		forward_gears(0),
		reverse_gears(0)
	{
		// ctor
	}
};

class Transmission : private TransmissionInfo
{
public:
	Transmission() :
		drive_shaft(&Shaft::getFixed()),
		gear(0)
	{
		// ctor
	}

	void init(const TransmissionInfo & info, Shaft & shaft)
	{
		TransmissionInfo::operator=(info);
		drive_shaft = &shaft;
	}

	// newgear in [-reverse_gears, forward_gears]
	void shift(int newgear)
	{
		if (newgear != gear &&
			newgear <= forward_gears &&
			newgear >= -reverse_gears)
		{
			gear = newgear;
		}
	}

	Shaft & getShaft() const
	{
		return *drive_shaft;
	}

	int getGear() const
	{
		return gear;
	}

	int getForwardGears() const
	{
		return forward_gears;
	}

	int getReverseGears() const
	{
		return reverse_gears;
	}

	btScalar getGearRatio(int gear) const
	{
		return gear_ratios[gear + reverse_gears];
	}

	btScalar getGearRatio() const
	{
		return gear_ratios[gear + reverse_gears];
	}

	btScalar getShiftTime() const
	{
		return shift_time;
	}

	btScalar getClutchRPM() const
	{
		return getGearRatio(gear) * drive_shaft->getAngularVelocity() * 30 / M_PI;
	}

private:
	Shaft * drive_shaft;	///< linked  driven shaft
	int gear;				///< selected gear
};

}

#endif
