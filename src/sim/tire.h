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

#ifndef _SIM_TIRE_H
#define _SIM_TIRE_H

#include "LinearMath/btVector3.h"
#include <vector>

namespace sim
{

struct TireInfo
{
	btScalar tread;						///< 1.0 means a pure off-road tire, 0.0 is a pure road tire
	btScalar max_load;					///< maximum tire load
	btScalar max_camber;				///< maximum tire camber
	std::vector<btScalar> longitudinal;	///< the parameters of the longitudinal pacejka equation.  this is series b
	std::vector<btScalar> lateral;		///< the parameters of the lateral pacejka equation.  this is series a
	std::vector<btScalar> aligning;		///< the parameters of the aligning moment pacejka equation.  this is series c
	std::vector<btScalar> sigma_hat;	///< maximum grip in the longitudinal direction
	std::vector<btScalar> alpha_hat;	///< maximum grip in the lateral direction
	TireInfo();
};

class Tire : private TireInfo
{
public:
	Tire();

	void init(const TireInfo & info);

	btScalar getTread() const {return tread;}

	/// normal_force: tire load in newton
	/// friction_coeff: contact surface friction coefficient
	/// inclination: wheel inclination (camber) in degrees
	/// ang_velocity: tire angular velocity (w * r)
	/// lon_velocty: tire longitudinal velocity relative to surface
	/// lat_velocty: tire lateral velocity relative to surface
	btVector3 getForce(
		btScalar normal_force,
		btScalar friction_coeff,
		btScalar inclination,
		btScalar ang_velocity,
		btScalar lon_velocty,
		btScalar lat_velocity);

	/// cached state, modified by getForce
	btScalar getFeedback() const {return mz;}
	btScalar getSlide() const {return slide;}
	btScalar getSlip() const {return slip;}
	btScalar getIdealSlide() const {return ideal_slide;}
	btScalar getIdealSlip() const {return ideal_slip;}

	/// load is the normal force in newtons.
	btScalar getMaxFx(btScalar load) const;

	/// load is the normal force in newtons, camber is in degrees
	btScalar getMaxFy(btScalar load, btScalar camber) const;

	/// load is the normal force in newtons, camber is in degrees
	btScalar getMaxMz(btScalar load, btScalar camber) const;

private:
	btScalar camber;		///< tire camber relative to track surface
	btScalar slide;			///< ratio of tire contact patch speed to road speed, minus one
	btScalar slip;			///< the angle (in degrees) between the wheel heading and the wheel's actual velocity
	btScalar ideal_slide;	///< ideal slide ratio
	btScalar ideal_slip;	///< ideal slip angle
	btScalar fx, fy, fz, mz;

	/// pacejka magic formula longitudinal friction
	btScalar PacejkaFx(btScalar sigma, btScalar Fz, btScalar friction_coeff, btScalar & max_Fx) const;

	/// pacejka magic formula lateral friction
	btScalar PacejkaFy(btScalar alpha, btScalar Fz, btScalar gamma, btScalar friction_coeff, btScalar & max_Fy) const;

	/// pacejka magic formula aligning torque
	btScalar PacejkaMz(btScalar sigma, btScalar alpha, btScalar Fz, btScalar gamma, btScalar friction_coeff, btScalar & max_Mz) const;

	/// get ideal slide ratio, slip angle
	void getSigmaHatAlphaHat(btScalar load, btScalar & sh, btScalar & ah) const;

	/// find ideal slip, slide for given parameters
	void findSigmaHatAlphaHat(btScalar load, btScalar & output_sigmahat, btScalar & output_alphahat, int iterations=400);

	/// calculate sigma_hat, alpha_hat tables
	void calculateSigmaHatAlphaHat(int tablesize=20);
};

}

#endif
