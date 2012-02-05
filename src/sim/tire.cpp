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

#include "sim/tire.h"

namespace sim
{

TireInfo::TireInfo() :
	tread(0),
	longitudinal(11),
	lateral(15),
	aligning(18)
{
	// ctor
}

Tire::Tire() :
	camber(0),
	slide(0),
	slip(0),
	ideal_slide(0),
	ideal_slip(0)
{
	// ctor
}

void Tire::init(const TireInfo & info)
{
	TireInfo::operator=(info);
	calculateSigmaHatAlphaHat();
}

void Tire::getSigmaHatAlphaHat(btScalar load, btScalar & sh, btScalar & ah) const
{
	btAssert(!sigma_hat.empty());
	btAssert(!alpha_hat.empty());

	int HAT_ITERATIONS = sigma_hat.size();

	btScalar HAT_LOAD = 0.5;
	btScalar nf = load * 0.001;
	if (nf < HAT_LOAD)
	{
		sh = sigma_hat[0];
		ah = alpha_hat[0];
	}
	else if (nf >= HAT_LOAD * HAT_ITERATIONS)
	{
		sh = sigma_hat[HAT_ITERATIONS - 1];
		ah = alpha_hat[HAT_ITERATIONS - 1];
	}
	else
	{
		int lbound = int(nf / HAT_LOAD) - 1;
		btClamp(lbound, 0, (int)sigma_hat.size() - 1);
		btScalar blend = (nf - HAT_LOAD * (lbound + 1)) / HAT_LOAD;
		sh = sigma_hat[lbound] * (1.0 - blend) + sigma_hat[lbound + 1] * blend;
		ah = alpha_hat[lbound] * (1.0 - blend) + alpha_hat[lbound + 1] * blend;
	}
}

btVector3 Tire::getForce(
	btScalar normal_force,
	btScalar friction_coeff,
	btScalar inclination,
	btScalar ang_velocity,
	btScalar lon_velocity,
	btScalar lat_velocity)
{
	if (normal_force < 1E-3 || friction_coeff < 1E-3)
	{
		return btVector3(0, 0, 0);
	}

	// get ideal slip ratio
	btScalar sigma_hat(0);
	btScalar alpha_hat(0);
	getSigmaHatAlphaHat(normal_force, sigma_hat, alpha_hat);

	// limit input
	btScalar Fz = normal_force * 0.001;
	btClamp(Fz, btScalar(0), btScalar(15));
	btClamp(inclination, btScalar(-15), btScalar(15));

	btScalar gamma = inclination;									// positive when tire top tilts to the right, viewed from rear
	btScalar denom = btMax(btFabs(lon_velocity), btScalar(1E-3));
	btScalar sigma = (ang_velocity - lon_velocity) / denom;			// longitudinal slip: negative in braking, positive in traction
	btScalar alpha = -btAtan(lat_velocity / denom) * 180.0 / M_PI; 	// sideslip angle: positive in a right turn(opposite to SAE tire coords)
	btScalar max_Fx(0), max_Fy(0), max_Mz(0);

	//combining method 1: beckman method for pre-combining longitudinal and lateral forces
	btScalar s = sigma / sigma_hat;
	btScalar a = alpha / alpha_hat;
	btScalar rho = btMax(btScalar(sqrt(s * s + a * a)), btScalar(1E-4)); // avoid divide-by-zero
	btScalar Fx = (s / rho) * PacejkaFx(rho * sigma_hat, Fz, friction_coeff, max_Fx);
	btScalar Fy = (a / rho) * PacejkaFy(rho * alpha_hat, Fz, gamma, friction_coeff, max_Fy);

/*
	//combining method 2: orangutan
	btScalar alpha_rad = alpha * M_PI / 180.0;
	btScalar Fx = PacejkaFx(sigma, Fz, friction_coeff, max_Fx);
	btScalar Fy = PacejkaFy(alpha, Fz, gamma, friction_coeff, max_Fy);
	btScalar x = cos(alpha_rad);
	x = fabs(Fx) / (fabs(Fx) + fabs(Fy));
	btScalar y = 1.0 - x;
	btScalar one = 1.0;
	if (sigma < 0.0) one = -1.0;
	btScalar pure_sigma = sigma / (one + sigma);
	btScalar pure_alpha = tan(alpha_rad) / (one + sigma);
	btScalar pure_combined = sqrt(pure_sigma * pure_sigma + pure_alpha * pure_alpha);
	btScalar kappa_combined = pure_combined / (1.0 - pure_combined);
	btScalar alpha_combined = atan((one + sigma) * pure_combined);
	btScalar Flimit_lng = PacejkaFx(kappa_combined, Fz, friction_coeff, max_Fx);
	btScalar Flimit_lat = PacejkaFy(alpha_combined * 180.0 / M_PI, Fz, gamma, friction_coeff, max_Fy);

	btScalar Flimit = (fabs(x * Flimit_lng) + fabs(y * Flimit_lat));
	btScalar Fmag = sqrt(Fx * Fx + Fy * Fy);

	if (Fmag > Flimit)
	{
		btScalar scale = Flimit / Fmag;
		Fx *= scale;
		Fy *= scale;
	}
*/
/*
	// combining method 3: traction circle
	btScalar Fx = PacejkaFx(sigma, Fz, friction_coeff, max_Fx);
	btScalar Fy = PacejkaFy(alpha, Fz, gamma, friction_coeff, max_Fy);

	// determine to what extent the tires are long (x) gripping vs lat (y) gripping
	// 1.0 when Fy is zero, 0.0 when Fx is zero
	btScalar longfactor = 1.0;
	btScalar combforce = btFabs(Fx) + btFabs(Fy);
	if (combforce > 1) longfactor = btFabs(Fx) / combforce;

	// determine the maximum force for this amount of long vs lat grip by linear interpolation
	btScalar maxforce = btFabs(max_Fx) * longfactor + (1.0 - longfactor) * btFabs(max_Fy);

	// scale down forces to fit into the maximum
	if (combforce > maxforce)
	{
		Fx *= maxforce / combforce;
		Fy *= maxforce / combforce;
	}
*/
/*
	//combining method 4: traction ellipse (prioritize Fx)
	//std::cout << "Fy0=" << Fy << ", ";
	if (Fx >= max_Fx)
	{
		Fx = max_Fx;
		Fy = 0;
	}
	else
		Fy = Fy*sqrt(1.0-(Fx/max_Fx)*(Fx/max_Fx));
	//std::cout << "Fy=" << Fy << ", Fx=Fx0=" << Fx << ", Fxmax=" << max_Fx << ", Fymax=" << max_Fy << std::endl;
*/
/*
	//combining method 5: traction ellipse (prioritize Fy)
	if (Fy >= max_Fy)
	{
		Fy = max_Fy;
		Fx = 0;
	}
	else
	{
		btScalar scale = sqrt(1.0-(Fy/max_Fy)*(Fy/max_Fy));
		if (scale != scale)
			Fx = 0;
		else
			Fx = Fx*scale;
	}
*/
/*
	// modified Nicolas-Comstock Model(Modeling Combined Braking and Steering Tire Forces)
	btScalar Fx0 = PacejkaFx(sigma, Fz, friction_coeff, max_Fx);
	btScalar Fy0 = PacejkaFy(alpha, Fz, gamma, friction_coeff, max_Fy);
	// 0 <= a <= pi/2 and 0 <= s <= 1
	// Cs = a0 and Ca = b0 longitudinal and lateral slip stiffness ?
	btScalar Fc = Fx0 * Fy0 / sqrt(s * s * Fy0 * Fy0 + Fx0 * Fx0 * tana * tana);
	btScalar Fx = Fc * sqrt(s * s * Ca * Ca + (1-s) * (1-s) * cosa * cosa * Fx0 * Fx0) / Ca;
	btScalar Fy = Fc * sqrt((1-s) * (1-s) * cosa * cosa * Fy0 * Fy0 + sina * sina * Cs * Cs) / (Cs * cosa);
*/

	btScalar Mz = PacejkaMz(sigma, alpha, Fz, gamma, friction_coeff, max_Mz);

	camber = inclination;
	slide = sigma;
	slip = alpha;
	ideal_slide = sigma_hat;
	ideal_slip = alpha_hat;
	fx = Fx;
	fy = Fy;
	fz = Fz;
	mz = Mz;

	// Fx positive during traction, Fy positive in a right turn, Mz positive in a left turn
	return btVector3(Fx, Fy, Mz);
}

btScalar Tire::getMaxFx(btScalar load) const
{
	const std::vector<btScalar> & b = longitudinal;
	btScalar Fz = load * 0.001;
	return (b[1] * Fz + b[2]) * Fz;
}

btScalar Tire::getMaxFy(btScalar load, btScalar camber) const
{
	const std::vector<btScalar> & a = lateral;
	btScalar Fz = load * 0.001;
	btScalar gamma = camber;

	btScalar D = (a[1] * Fz + a[2]) * Fz;
	btScalar Sv = ((a[11] * Fz + a[12]) * gamma + a[13] ) * Fz + a[14];

	return D + Sv;
}

btScalar Tire::getMaxMz(btScalar load, btScalar camber) const
{
	const std::vector<btScalar> & c = aligning;
	btScalar Fz = load * 0.001;
	btScalar gamma = camber;

	btScalar D = (c[1] * Fz + c[2]) * Fz;
	btScalar Sv = (c[14] * Fz * Fz + c[15] * Fz) * gamma + c[16] * Fz + c[17];

	return -(D + Sv);
}

btScalar Tire::PacejkaFx(btScalar sigma, btScalar Fz, btScalar friction_coeff, btScalar & max_Fx) const
{
	const std::vector<btScalar> & b = longitudinal;

	// shape factor
	btScalar C = b[0];

	// peak factor
	btScalar D = (b[1] * Fz + b[2]) * Fz;

	btScalar BCD = (b[3] * Fz + b[4]) * Fz * exp(-b[5] * Fz);

	// stiffness factor
	btScalar B =  BCD / (C * D);

	// curvature factor
	btScalar E = b[6] * Fz * Fz + b[7] * Fz + b[8];

	// horizontal shift
	btScalar Sh = 0;//beckmann//b[9] * Fz + b[10];

	// composite
	btScalar S = 100 * sigma + Sh;

	// longitudinal force
	btScalar Fx = D * sin(C * atan(B * S - E * (B * S - atan(B * S))));

	// scale by surface friction
	Fx = Fx * friction_coeff;
	max_Fx = D * friction_coeff;

	return Fx;
}

btScalar Tire::PacejkaFy(btScalar alpha, btScalar Fz, btScalar gamma, btScalar friction_coeff, btScalar & max_Fy) const
{
	const std::vector<btScalar> & a = lateral;

	// shape factor
	btScalar C = a[0];

	// peak factor
	btScalar D = (a[1] * Fz + a[2]) * Fz;

	btScalar BCD = a[3] * sin(2.0 * atan(Fz / a[4])) * (1.0 - a[5] * btFabs(gamma));

	// stiffness factor
	btScalar B = BCD / (C * D);

	// curvature factor
	btScalar E = a[6] * Fz + a[7];

	// horizontal shift
	btScalar Sh = 0;//beckmann//a[8] * gamma + a[9] * Fz + a[10];

	// vertical shift
	btScalar Sv = ((a[11] * Fz + a[12]) * gamma + a[13]) * Fz + a[14];

	// composite
	btScalar S = alpha + Sh;

	// lateral force
	btScalar Fy = D * sin(C * atan(B * S - E * (B * S - atan(B * S)))) + Sv;

	// scale by surface friction
	Fy = Fy * friction_coeff;
	max_Fy = (D + Sv) * friction_coeff;

	btAssert(Fy == Fy);
	return Fy;
}

btScalar Tire::PacejkaMz(btScalar sigma, btScalar alpha, btScalar Fz, btScalar gamma, btScalar friction_coeff, btScalar & max_Mz) const
{
	const std::vector<btScalar> & c = aligning;

	btScalar C = c[0];

	// peak factor
	btScalar D = (c[1] * Fz + c[2]) * Fz;

	btScalar BCD = (c[3] * Fz + c[4]) * Fz * (1.0 - c[6] * btFabs(gamma)) * exp (-c[5] * Fz);

	// stiffness factor
	btScalar B =  BCD / (C * D);

	// curvature factor
	btScalar E = (c[7] * Fz * Fz + c[8] * Fz + c[9]) * (1.0 - c[10] * btFabs(gamma));

	// slip angle + horizontal shift
	btScalar S = alpha + c[11] * gamma + c[12] * Fz + c[13];

	// vertical shift
	btScalar Sv = (c[14] * Fz * Fz + c[15] * Fz) * gamma + c[16] * Fz + c[17];

	// self-aligning torque
	btScalar Mz = D * sin(c[0] * atan(B * S - E * (B * S - atan(B * S)))) + Sv;

	// scale by surface friction
	Mz = Mz * friction_coeff;
	max_Mz = (D + Sv) * friction_coeff;

	btAssert(Mz == Mz);
	return Mz;
}

void Tire::findSigmaHatAlphaHat(btScalar load, btScalar & output_sigmahat, btScalar & output_alphahat, int iterations)
{
	btScalar x, y, ymax, junk;
	ymax = 0;
	for (x = -2; x < 2; x += 4.0/iterations)
	{
		y = PacejkaFx(x, load, 1.0, junk);
		if (y > ymax)
		{
			output_sigmahat = x;
			ymax = y;
		}
	}

	ymax = 0;
	for (x = -20; x < 20; x += 40.0/iterations)
	{
		y = PacejkaFy(x, load, 0, 1.0, junk);
		if (y > ymax)
		{
			output_alphahat = x;
			ymax = y;
		}
	}
}

void Tire::calculateSigmaHatAlphaHat(int tablesize)
{
	btScalar HAT_LOAD = 0.5;
	sigma_hat.resize(tablesize, 0);
	alpha_hat.resize(tablesize, 0);
	for (int i = 0; i < tablesize; i++)
	{
		findSigmaHatAlphaHat((btScalar)(i+1)*HAT_LOAD, sigma_hat[i], alpha_hat[i]);
	}
}

}
