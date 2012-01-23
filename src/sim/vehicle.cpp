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

#include "sim/vehicle.h"
#include "sim/vehicleinfo.h"
#include "sim/solveconstraintrow.h"
#include "sim/world.h"
#include "coordinatesystem.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

namespace sim
{

// helper function to retrieve shaft from id
static Shaft * LinkShaft(
	int shaft_id,
	btAlignedObjectArray<Wheel> & wheel,
	btAlignedObjectArray<Differential> & diff)
{
	if (shaft_id < wheel.size())
	{
		btAssert(shaft_id >= 0);
		return &wheel[shaft_id].shaft;
	}
	shaft_id -= wheel.size();
	btAssert(shaft_id < diff.size());
	return &diff[shaft_id].getShaft();
}

Vehicle::Vehicle() :
	world(0),
	body(0),
	last_auto_clutch(1),
	remaining_shift_time(0),
	tacho_rpm(0),
	shift_gear(0),
	autoclutch(true),
	autoshift(false),
	shifted(true),
	abs_active(false),
	tcs_active(false),
	abs(false),
	tcs(false),
	maxangle(0),
	maxspeed(0)
{
	// Constructor
}

Vehicle::~Vehicle()
{
	if (!world) return;

	body->clear(*world);
	world->removeAction(this);
	world->removeRigidBody(body);
	delete body->getCollisionShape();
	delete body;
}

void Vehicle::init(
	const VehicleInfo & info,
	const btVector3 & position,
	const btQuaternion & rotation,
	World & world)
{
	transform.setRotation(rotation);
	transform.setOrigin(position+direction::up);

	body = new FractureBody(info.body);
	body->setCenterOfMassTransform(transform);
	body->setActivationState(DISABLE_DEACTIVATION);
	body->setContactProcessingThreshold(0.0);
	world.addRigidBody(body);
	world.addAction(this);
	this->world = &world;

	aero_device.resize(info.aerodevice.size());
	for (int i = 0; i < info.aerodevice.size(); ++i)
	{
		aero_device[i] = AeroDevice(info.aerodevice[i]);
	}

	differential.resize(info.differential.size());
	wheel.resize(info.wheel.size());
	wheel_contact.resize(wheel.size());
	diff_joint.resize(differential.size());
	clutch_joint.resize(differential.size() + 1);
	motor_joint.resize(wheel.size() * 2 + 1);

	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].init(info.wheel[i], world, *body);
		maxangle = btMax(maxangle, wheel[i].suspension.getMaxSteeringAngle());
	}

	for (int i = 0; i < differential.size(); ++i)
	{
		Shaft * shaft_a = LinkShaft(info.differential_link_a[i], wheel, differential);
		Shaft * shaft_b = LinkShaft(info.differential_link_b[i], wheel, differential);
		differential[i].init(info.differential[i], *shaft_a, *shaft_b);
	}

	Shaft * shaft_t = LinkShaft(info.transmission_link, wheel, differential);
	transmission.init(info.transmission, *shaft_t);
	clutch.init(info.clutch);
	engine.init(info.engine);

	calculateFrictionCoefficient(lon_friction_coeff, lat_friction_coeff);
	maxspeed = calculateMaxSpeed();

	// the position can be a point on the surface not at car center of mass
	// move wheel[0] center to position level to make sure the ray test doesn't fail
//	btVector3 down = getDownVector();
//	btVector3 wheelpos = LocalToWorld(wheel[0].suspension.getWheelPosition()); fixme
//	btVector3 offset = down * down.dot(position - wheelpos);
//	setPosition(body->getCenterOfMassPosition() + offset);
//	AlignWithGround();
}

void Vehicle::debugDraw(btIDebugDraw*)
{
	// nothing to do here
}

const btVector3 & Vehicle::getCenterOfMass() const
{
	return body->getCenterOfMassPosition();
}

btScalar Vehicle::getInvMass() const
{
	return body->getInvMass();
}

btScalar Vehicle::getSpeed() const
{
	return body->getLinearVelocity().length();
}

const btVector3 & Vehicle::getVelocity() const
{
	return body->getLinearVelocity();
}

void Vehicle::startEngine()
{
	engine.start();
}

void Vehicle::setGear(int value)
{
	if (shifted &&
		value != transmission.getGear() &&
		value <= transmission.getForwardGears() &&
		value >= -transmission.getReverseGears())
	{
		remaining_shift_time = transmission.getShiftTime();
		shift_gear = value;
		shifted = false;
	}
}

void Vehicle::setThrottle(btScalar value)
{
	engine.setThrottle(value);
}

void Vehicle::setNOS(btScalar value)
{
	engine.setNosBoost(value);
}

void Vehicle::setClutch(btScalar value)
{
	clutch.setPosition(value);
}

void Vehicle::setBrake(btScalar value)
{
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].brake.setBrakeFactor(value);
	}
}

void Vehicle::setHandBrake(btScalar value)
{
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].brake.setHandbrakeFactor(value);
	}
}

void Vehicle::setAutoClutch(bool value)
{
	autoclutch = value;
}

void Vehicle::setAutoShift(bool value)
{
	autoshift = value;
}

btScalar Vehicle::getSpeedMPS() const
{
	return wheel[0].getRadius() * wheel[0].shaft.getAngularVelocity();
}

btScalar Vehicle::getMaxSpeedMPS() const
{
	return maxspeed;
}

btScalar Vehicle::getTachoRPM() const
{
	return tacho_rpm;
}

void Vehicle::setABS(bool value)
{
	abs = value;
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].setABS(value);
	}
}

void Vehicle::setTCS(bool value)
{
	tcs = value;
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].setTCS(value);
	}
}

void Vehicle::setPosition(const btVector3 & position)
{
	body->translate(position - body->getCenterOfMassPosition());
}

void Vehicle::alignWithGround()
{
	btScalar ray_len = 8;
	btScalar min_height = 0;
	bool no_min_height = true;
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].updateContact(ray_len);
		btScalar height = wheel[i].ray.getDepth() - ray_len;
		if (height < min_height || no_min_height)
		{
			min_height = height;
			no_min_height = false;
		}
	}

	btVector3 delta = getDownVector() * min_height;
	btVector3 trimmed_position = body->getCenterOfMassPosition() + delta;
	setPosition(trimmed_position);
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].updateContact(ray_len);
	}

	body->setAngularVelocity(btVector3(0, 0, 0));
	body->setLinearVelocity(btVector3(0, 0, 0));
}

// ugh, ugly code
void Vehicle::rolloverRecover()
{
	btTransform transform = body->getCenterOfMassTransform();

	btVector3 z(direction::up);
	btVector3 y_car = transform.getBasis() * direction::forward;
	y_car = y_car - z * z.dot(y_car);
	y_car.normalize();

	btVector3 z_car = transform.getBasis() * direction::up;
	z_car = z_car - y_car * y_car.dot(z_car);
	z_car.normalize();

	btScalar angle = z_car.angle(z);
	if (fabs(angle) < M_PI / 4.0) return;

	btQuaternion rot(y_car, angle);
	rot = rot * transform.getRotation();
	transform.setRotation(rot);

	body->setCenterOfMassTransform(transform);

	alignWithGround();
}

void Vehicle::setSteering(const btScalar value)
{
	for (int i = 0; i < wheel.size(); ++i)
	{
		wheel[i].suspension.setSteering(value);
	}
}

btScalar Vehicle::getMaxSteeringAngle() const
{
	return maxangle;
}

btScalar Vehicle::getBrakingDistance(btScalar target_speed) const
{
	btScalar g = 9.81;
	btScalar c = lon_friction_coeff * g;
	btScalar d = (getDragCoefficient() - getLiftCoefficient() * lon_friction_coeff) * getInvMass();
	btScalar v1sqr = getVelocity().length2();
	btScalar v2sqr = target_speed * target_speed;
	return -log((c + v2sqr * d) / (c + v1sqr * d)) / (2 * d);
}

btScalar Vehicle::getMaxVelocity(btScalar radius) const
{
	btScalar g = 9.81;
	btScalar d = 1.0 - btMin(btScalar(1.01), -radius * getLiftCoefficient() * lat_friction_coeff * getInvMass());
	btScalar real = lat_friction_coeff * g * radius / d;
	btScalar v = 1000.0;
	if (real > 0) v = sqrt(real);
	return v;
}

btVector3 Vehicle::getTotalAero() const
{
	btVector3 force(0, 0, 0);
	for (int i = 0; i < aero_device.size(); ++i)
	{
		force = force + aero_device[i].getLift() + aero_device[i].getDrag();
	}
	return force;
}

btScalar Vehicle::getLiftCoefficient() const
{
	btScalar coeff = 0.0;
	for (int i = 0; i < aero_device.size(); ++i)
	{
		coeff += aero_device[i].getLiftCoefficient();
	}
	return coeff;
}

btScalar Vehicle::getDragCoefficient() const
{
	btScalar coeff = 0.0;
	for (int i = 0; i < aero_device.size(); ++i)
	{
		coeff += aero_device[i].getDragCoefficient();
	}
	return coeff;
}

btScalar Vehicle::getFeedback() const
{
	return feedback;
}

btVector3 Vehicle::getDownVector() const
{
	return -body->getCenterOfMassTransform().getBasis().getColumn(2);
}

// executed as last function(after integration) in bullet singlestepsimulation
void Vehicle::updateAction(btCollisionWorld * collisionWorld, btScalar dt)
{
	//static std::ofstream log("log.txt");

	updateAerodynamics(dt);

	updateTransmission(dt);

	engine.update(dt);

	// differentials (constant, should happen at initialisation maybe?)
	for (int i = 0; i < differential.size(); ++i)
	{
		DifferentialJoint & djoint = diff_joint[i];
		djoint.shaft1 = &differential[i].getShaft();
		djoint.shaft2a = &differential[i].getShaft1();
		djoint.shaft2b = &differential[i].getShaft2();
		djoint.gearRatio = differential[i].getFinalDrive();
		djoint.init();

		ClutchJoint & cjoint = clutch_joint[i];
		cjoint.shaft1 = &differential[i].getShaft1();
		cjoint.shaft2 = &differential[i].getShaft2();
		cjoint.gearRatio = 1;
		cjoint.impulseLimit = differential[i].getAntiSlipTorque() * dt;
		cjoint.init();
	}

	// transmission and clutch
	int dcount = differential.size();
	int ccount = differential.size();
	if (btFabs(transmission.getGearRatio()) > 0.0)
	{
		ClutchJoint & cjoint = clutch_joint[ccount];
		cjoint.shaft1 = &engine.getShaft();
		cjoint.shaft2 = &transmission.getShaft();
		cjoint.gearRatio = transmission.getGearRatio();
		cjoint.impulseLimit = clutch.getTorque() * dt;
		cjoint.init();
		ccount++;
	}

	// wheels
	int mcount = 0;
	int wcount = 0;
	abs_active = false;
	tcs_active = false;
	for (int i = 0; i < wheel.size(); ++i)
	{
		if (wheel[i].update(dt, wheel_contact[wcount]))
		{
			wheel_contact[wcount].id = i;

			MotorJoint & joint = motor_joint[mcount];
			joint.shaft = &wheel[i].shaft;
			joint.targetVelocity = wheel_contact[wcount].v1 / wheel[i].getRadius();
			joint.accumulatedImpulse = 0;
			abs_active |= wheel[i].getABS();
			tcs_active |= wheel[i].getTCS();
			mcount++;
			wcount++;
		}
	}

	// engine
	MotorJoint & joint = motor_joint[mcount];
	joint.shaft = &engine.getShaft();
	joint.targetVelocity = engine.getTorque() > 0 ? engine.getRPMLimit() * M_PI / 30 : 0;
	joint.impulseLimit = btFabs(engine.getTorque()) * dt;
	joint.accumulatedImpulse = 0;
	mcount++;

	// brakes
	for (int i = 0; i < wheel.size(); ++i)
	{
		btScalar torque = wheel[i].brake.getTorque();
		if (torque > 0)
		{
			MotorJoint & joint = motor_joint[mcount];
			joint.shaft = &wheel[i].shaft;
			joint.targetVelocity = 0;
			joint.impulseLimit = torque * dt;
			joint.accumulatedImpulse = 0;
			mcount++;
		}
	}

	// solver loop
	const int iterations = 8;
	for (int n = 0; n < iterations; ++n)
	{
		// wheel
		for (int i = 0; i < wcount; ++i)
		{
			WheelContact & c = wheel_contact[i];
			Wheel & w = wheel[c.id];

			SolveConstraintRow(c.response, *c.bodyA, *c.bodyB, c.rA, c.rB);

			btScalar load = c.response.accumImpulse / dt;
			btVector3 friction = w.tire.getForce(load, c.frictionCoeff, c.camber, c.vR, c.v1, c.v2);

			if (friction[1] > 0)
			{
				c.friction2.lowerLimit = 0;
				c.friction2.upperLimit = friction[1] * dt;
			}
			else
			{
				c.friction2.lowerLimit = friction[1] * dt;
				c.friction2.upperLimit = 0;
			}
			//log << friction[0] * dt << " ";

			btScalar impulseLimit = btFabs(friction[0]) * w.getRadius() * dt;
			motor_joint[i].impulseLimit = impulseLimit;
		}

		// driveline
		for (int i = 0; i < mcount; ++i)
		{
			motor_joint[i].solve();
		}
		for (int i = 0; i < dcount; ++i)
		{
			diff_joint[i].solve();
		}
		for (int i = 0; i < ccount; ++i)
		{
			clutch_joint[i].solve();
		}
		//log << "  ";

		// wheel friction
		for (int i = 0; i < wcount; ++i)
		{
			WheelContact & c = wheel_contact[i];
			Wheel & w = wheel[c.id];

			// friction torque limit seems to be always reached
			// so this seems superflous
			btScalar impulseLimit = motor_joint[i].accumulatedImpulse;
			if (impulseLimit > 0)
			{
				c.friction1.upperLimit = 0;
				c.friction1.lowerLimit = -impulseLimit / w.getRadius();
			}
			else
			{
				c.friction1.upperLimit = -impulseLimit / w.getRadius();
				c.friction1.lowerLimit = 0;
			}
			//log << -impulseLimit / w.getRadius() << " ";

			btScalar vel = w.shaft.getAngularVelocity() * w.getRadius();
			SolveConstraintRow(c.friction1, *c.bodyA, *c.bodyB, c.rA, c.rB, -vel);
			SolveConstraintRow(c.friction2, *c.bodyA, *c.bodyB, c.rA, c.rB);
		}
		//log << "\n";
		//feedback += 0.5 * (wheel[0].tire.getFeedback() + wheel[1].tire.getFeedback()); fixme
	}
	//log << "\n";
	//feedback /= (steps + 1);

	// tacho low pass
	tacho_rpm = engine.getRPM() * 0.3 + tacho_rpm * 0.7;

	// update body
	body->setCenterOfMassTransform(transform);
	body->predictIntegratedTransform(dt, transform);
	body->proceedToTransform(transform);
	updateWheelTransform(dt);
}

void Vehicle::updateAerodynamics(btScalar dt)
{
	aero_force.setValue(0, 0, 0);
	aero_torque.setValue(0, 0, 0);
	const btMatrix3x3 inv = body->getCenterOfMassTransform().getBasis().transpose();
	btVector3 air_velocity = -(inv * getVelocity());
	for (int i = 0; i < aero_device.size(); ++i)
	{
		btVector3 force = aero_device[i].getForce(air_velocity);
		btVector3 position = aero_device[i].getPosition() + body->getCenterOfMassOffset();
		aero_force = aero_force + force;
		aero_torque = aero_torque + position.cross(force);
	}
	btVector3 force = body->getCenterOfMassTransform().getBasis() * aero_force;
	btVector3 torque = body->getCenterOfMassTransform().getBasis() * aero_torque;
	body->applyCentralImpulse(force * dt);
	body->applyTorqueImpulse(torque * dt);
}

void Vehicle::updateWheelTransform(btScalar dt)
{
	for (int i = 0; i < wheel.size(); ++i)
	{
		if (!body->isChildConnected(i)) continue;

		wheel[i].shaft.integrate(dt);
		btQuaternion rot = wheel[i].suspension.getOrientation();
		rot *= btQuaternion(direction::right, -wheel[i].shaft.getAngle());
		btVector3 pos = wheel[i].suspension.getPosition() + body->getCenterOfMassOffset();
		body->setChildTransform(i, btTransform(rot, pos));
	}
}

void Vehicle::updateTransmission(btScalar dt)
{
	btScalar clutch_rpm = transmission.getClutchRPM();

	if (autoshift)
	{
		int gear = getNextGear(clutch_rpm);
		setGear(gear);
	}

	remaining_shift_time -= dt;
	if (remaining_shift_time < 0)
	{
		remaining_shift_time = 0;
	}

	if (!shifted && remaining_shift_time < transmission.getShiftTime() * 0.5f)
	{
		transmission.shift(shift_gear);
		shifted = true;
	}

	if (autoclutch)
	{
		if (!engine.getCombustion())
		{
		    engine.start();
		}

		btScalar throttle = engine.getThrottle();
		throttle = shiftAutoClutchThrottle(clutch_rpm, throttle, dt);
		if (engine.getRPM() < engine.getStartRPM() &&
			throttle < engine.getIdleThrottle())
		{
			throttle = engine.getIdleThrottle();
		}
		engine.setThrottle(throttle);

		btScalar new_clutch = autoClutch(clutch_rpm, last_auto_clutch, dt);
		clutch.setPosition(new_clutch);
		last_auto_clutch = new_clutch;
	}
}

btScalar Vehicle::autoClutch(btScalar clutch_rpm, btScalar last_clutch, btScalar dt) const
{
	// limit clutch load to keep engine rpm above stall
	btScalar clutch_value = 1;
	btScalar rpm_min = engine.getStartRPM();
	btScalar rpm_clutch = transmission.getClutchRPM();
	if (rpm_clutch < rpm_min)
	{
		btScalar rpm = engine.getRPM();
		btScalar inertia = engine.getShaft().getInertia();
		btScalar torque_limit = inertia * (rpm - rpm_min) / dt;
		if (torque_limit > engine.getTorque())
		{
			torque_limit = engine.getTorque();
		}
		clutch_value = torque_limit / clutch.getTorqueMax();
		btClamp(clutch_value, btScalar(0), btScalar(1));
	}

	// shift time
	clutch_value *= shiftAutoClutch();

	// rate limit the autoclutch
	btScalar engage_limit = 20 * dt;
	btScalar clutch_delta = clutch_value - last_clutch;
	btClamp(clutch_delta, -engage_limit, engage_limit);
	clutch_value = last_clutch + clutch_delta;

	return clutch_value;
}

btScalar Vehicle::shiftAutoClutch() const
{
	const btScalar shift_time = transmission.getShiftTime();
	btScalar shift_clutch = 1.0;
	if (remaining_shift_time > shift_time * 0.5)
	    shift_clutch = 0.0;
	else if (remaining_shift_time > 0.0)
	    shift_clutch = 1.0 - remaining_shift_time / (shift_time * 0.5);
	return shift_clutch;
}

btScalar Vehicle::shiftAutoClutchThrottle(btScalar clutch_rpm, btScalar throttle, btScalar dt)
{
	if (remaining_shift_time > 0.0)
	{
		if (engine.getRPM() < clutch_rpm && engine.getRPM() < engine.getRedline())
		{
			remaining_shift_time += dt;
			return 1.0;
		}
		else
		{
			return 0.5 * throttle;
		}
	}
	return throttle;
}

int Vehicle::getNextGear(btScalar clutch_rpm) const
{
	int gear = transmission.getGear();

	// only autoshift if a shift is not in progress
	if (shifted && clutch.getPosition() == 1.0)
	{
		// shift up when driveshaft speed exceeds engine redline
		// we do not shift up from neutral/reverse
		if (clutch_rpm > engine.getRedline() && gear > 0)
		{
			return gear + 1;
		}
		// shift down when driveshaft speed below shift_down_point
		// we do not auto shift down from 1st gear to neutral
		if (clutch_rpm < getDownshiftRPM(gear) && gear > 1)
		{
			return gear - 1;
		}
	}
	return gear;
}

btScalar Vehicle::getDownshiftRPM(int gear) const
{
	btScalar shift_down_point = 0.0;
	if (gear > 1)
	{
        btScalar current_gear_ratio = transmission.getGearRatio(gear);
        btScalar lower_gear_ratio = transmission.getGearRatio(gear - 1);
		btScalar peak_engine_speed = engine.getRedline();
		shift_down_point = 0.7 * peak_engine_speed / lower_gear_ratio * current_gear_ratio;
	}
	return shift_down_point;
}

btScalar Vehicle::calculateMaxSpeed() const
{
	btScalar maxspeed = 250.0f/3.6f; // fixme
	return maxspeed;
}

void Vehicle::calculateFrictionCoefficient(btScalar & lon_mu, btScalar & lat_mu) const
{
	btScalar lon_friction_factor = 0.68;
	btScalar lat_friction_factor = 0.62;
	btScalar gravity = 9.81;
	btScalar force = 0.25 * gravity / getInvMass();

	btScalar lon_friction = 0.0;
	btScalar lat_friction = 0.0;
	for (int i = 0; i < wheel.size(); ++i)
	{
		lon_friction += wheel[i].tire.getMaxFx(force) / force;
		lat_friction += wheel[i].tire.getMaxFy(force, 0.0) / force;
	}

	lon_mu = lon_friction_factor * lon_friction;
	lat_mu = lat_friction_factor * lat_friction;
}

void Vehicle::print(std::ostream & out) const
{
	const btVector3 & v = body->getLinearVelocity();
	const btVector3 & p = body->getCenterOfMassPosition();
	const btVector3 & c = body->getCenterOfMassOffset();

	out << std::fixed << std::setprecision(3);
	out << "velocity " << v.x() << " " << v.y() << " " << v.z() << "\n";
	out << "position " << p.x() << " " << p.y() << " " << p.z() << "\n";
	out << "mass center " << -c.x() << " " << -c.y() << " " << -c.z() << "\n";
	out << "mass " << 1 / body->getInvMass() << "\n\n";

	out << "aero force " << aero_force.x() << " " << aero_force.y() << " " << aero_force.z() << "\n";
	out << "aero torque " << aero_torque.x() << " " << aero_torque.y() << " " << aero_torque.z() << "\n";
	out << "lift/drag " << aero_force.z() / aero_force.y() << "\n\n";

	const ClutchJoint & cjoint = clutch_joint[differential.size()];
	out << "engine rpm " << engine.getRPM() << "\n";
	out << "clutch rpm " << transmission.getClutchRPM() << "\n";
	out << "engine torque " << engine.getTorque() << "\n";
	out << "clutch torque " << cjoint.accumulatedImpulse * 90 << "\n";
	out << "clutch engaged " << last_auto_clutch << "\n\n";//clutch.getPosition() << "\n\n";

	for (int i = 0; i < wheel.size(); ++i)
	{
		const Wheel & w = wheel[i];
		out << "wheel " << i << "\n";
		out << "load " << w.suspension.getDisplacement() * w.suspension.getStiffness() << "\n";
		out << "ideal slide " << w.tire.getIdealSlide() << "\n";
		out << "slide " << w.tire.getSlide() << "\n";
		out << "rpm " << w.shaft.getAngularVelocity() * 30 / M_PI << "\n\n";
	}
}

}
