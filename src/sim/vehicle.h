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

#ifndef _SIM_VEHICLE_H
#define _SIM_VEHICLE_H

#include "sim/aerodevice.h"
#include "sim/differential.h"
#include "sim/wheel.h"
#include "sim/transmission.h"
#include "sim/clutch.h"
#include "sim/engine.h"
#include "sim/wheelcontact.h"
#include "sim/differentialjoint.h"
#include "sim/clutchjoint.h"
#include "sim/motorjoint.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <ostream>

namespace sim
{

class World;
class FractureBody;
struct VehicleInfo;

class Vehicle : public btActionInterface
{
public:
	Vehicle();

	~Vehicle();

	// vehicle info has to be valid, no error checking here!
	void init(
		const VehicleInfo & info,
		const btVector3 & position,
		const btQuaternion & rotation,
		World & world);

	// car controls
	void setSteering(btScalar value);	// [-1, 1] left, right
	void setGear(int value);			// [nrev, nfwd]
	void setThrottle(btScalar value);	// [0, 1]
	void setNOS(btScalar value);		// [0, 1]
	void setClutch(btScalar value);		// [0, 1]
	void setBrake(btScalar value);		// [0, 1]
	void setHandBrake(btScalar value);	// [0, 1]
	void setAutoClutch(bool value);
	void setAutoShift(bool value);
	void setABS(bool value);
	void setTCS(bool value);
	void startEngine();

	// set body position
	void setPosition(const btVector3 & pos);

	// move the car along z-axis until it is touching the ground
	void alignWithGround();

	// rotate car back onto it's wheels after rollover
	void rolloverRecover();

	// bullet interface
	void updateAction(btCollisionWorld * collisionWorld, btScalar dt);

	// unused
	void debugDraw(btIDebugDraw * debugDrawer);

	// body state
	const btVector3 & getCenterOfMass() const;
	const btVector3 & getVelocity() const;
	btScalar getInvMass() const;
	btScalar getSpeed() const;

	// speedometer reading first wheel(front left) in m/s
	btScalar getSpeedMPS() const;

	// based on transmission, engine rpm limit and wheel radius in m/s
	btScalar getMaxSpeedMPS() const;

	// engine rpm
	btScalar getTachoRPM() const;

	// get the maximum steering angle in degrees
	btScalar getMaxSteeringAngle() const;

	// minimum distance to reach target speed
	btScalar getBrakingDistance(btScalar target_speed) const;

	// maximum velocity for given curve radius at current speed
	btScalar getMaxVelocity(btScalar radius) const;

	// caculate aerodynamic force in world space
	btVector3 getTotalAero() const;

	// steering feedback
	btScalar getFeedback() const;

	// driveline state access
	int getWeelCount() const						{ return wheel.size(); }
	const Engine & getEngine() const				{ return engine; }
	const Clutch & getClutch() const				{ return clutch; }
	const Transmission & getTransmission() const	{ return transmission; }
	const Wheel & getWheel(int i) const				{ return wheel[i]; }
	btScalar getNosAmount() const					{ return engine.getNos(); }
	bool getOutOfGas() const						{ return engine.getFuel() < 1E-3; }

	// traction control state
	bool getABSEnabled() const	{ return abs; }
	bool getTCSEnabled() const	{ return tcs; }
	bool getABSActive() const	{ return abs_active; }
	bool getTCSActive() const	{ return tcs_active; }

	// debugging
	void print(std::ostream & out) const;

protected:
	World * world;
	FractureBody * body;
	btTransform transform;
	btAlignedObjectArray<AeroDevice> aero_device;
	btAlignedObjectArray<Differential> differential;
	btAlignedObjectArray<Wheel> wheel;
	Transmission transmission;
	Clutch clutch;
	Engine engine;

	// vehicle solver state
	btAlignedObjectArray<WheelContact> wheel_contact;
	btAlignedObjectArray<DifferentialJoint> diff_joint;
	btAlignedObjectArray<ClutchJoint> clutch_joint;
	btAlignedObjectArray<MotorJoint> motor_joint;

	// cache wheel angular velocity to be able to calculate applied torque
	btAlignedObjectArray<btScalar> wheel_angvel;

	btScalar last_auto_clutch;
	btScalar remaining_shift_time;
	btScalar tacho_rpm;
	int shift_gear;
	bool autoclutch;
	bool autoshift;
	bool shifted;
	bool abs_active;
	bool tcs_active;
	bool abs;
	bool tcs;

	// aerodynamic force and torque for debugging
	btVector3 aero_force;
	btVector3 aero_torque;

	// approximate total lateral and longitudinal friction coefficients
	// used for braking distance and max curve velocity estimation
	btScalar lon_friction_coeff;
	btScalar lat_friction_coeff;

	btScalar maxangle;
	btScalar maxspeed;
	btScalar feedback;

	btVector3 getDownVector() const;

	// calculate throttle, clutch, gear
	void updateTransmission(btScalar dt);

	// apply aerodynamic forces to body
	void updateAerodynamics(btScalar dt);

	// update wheel position, rotation
	void updateWheelTransform(btScalar dt);

	btScalar autoClutch(btScalar clutch_rpm, btScalar last_clutch, btScalar dt) const;

	btScalar shiftAutoClutch() const;

	btScalar shiftAutoClutchThrottle(btScalar clutch_rpm, btScalar throttle, btScalar dt);

	// return the gear change (0 for no change, -1 for shift down, 1 for shift up)
	int getNextGear(btScalar clutch_rpm) const;

	// calculate downshift rpm based on gear, engine rpm
	btScalar getDownshiftRPM(int gear) const;

	// max speed in m/s calculated from maxrpm, maxgear, finalgear ratios
	btScalar calculateMaxSpeed() const;

	// calculate total longitudinal and lateral friction coefficients
	void calculateFrictionCoefficient(btScalar & lon_mu, btScalar & lat_mu) const;

	// aerodynamics
	btScalar getLiftCoefficient() const;

	btScalar getDragCoefficient() const;
};

}

#endif
