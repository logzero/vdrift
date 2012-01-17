#include "performance_testing.h"
#include "sim/world.h"
#include "sim/surface.h"
#include "carinput.h"
#include "cfg/ptree.h"

#include <vector>
#include <iostream>
#include <sstream>

static inline float ConvertToMPH(float ms)
{
	return ms * 2.23693629;
}

static inline float ConvertToFeet(float meters)
{
	return meters * 3.2808399;
}

PERFORMANCE_TESTING::PERFORMANCE_TESTING(sim::World & world) :
	world(world)
{
	surface.bumpWaveLength = 1;
	surface.bumpAmplitude = 0;
	surface.frictionNonTread = 1;
	surface.frictionTread = 1;
	surface.rollResistanceCoefficient = 1;
	surface.rollingDrag = 0;
}

void PERFORMANCE_TESTING::Test(
	const std::string & carpath,
	const std::string & carname,
	const std::string & partspath,
	std::ostream & info_output,
	std::ostream & error_output)
{
/*
	info_output << "Beginning car performance test on " << carname << std::endl;
	const std::string carfile = carpath+"/"+carname+".car";

	//load the car dynamics
	std::ifstream carstream(carfile.c_str());
	if (!carstream)
	{
		error_output << "Error loading car configuration file: " << carfile << std::endl;
		return;
	}

	PTree cfg;
	read_ini(carstream, cfg);

	btVector3 size(0, 0, 0), center(0, 0, 0), pos(0, 0, 0); // collision shape from wheel data
	btQuaternion rot = btQuaternion::getIdentity();
	bool damage = false;
	if (!car.dynamics.Load(cfg, size, center, pos, rot, damage, world, error_output))
	{
		error_output << "Error during car dynamics load: " << carfile << std::endl;
		return;
	}
	info_output << "Car dynamics loaded" << std::endl;

	info_output << carname << " Summary:\n" <<
			"Mass (kg) including driver and fuel: " << 1/car.GetInvMass() << "\n" <<
			"Center of mass (m): " << car.GetCenterOfMassPosition() << std::endl;

	std::stringstream statestream;
	joeserialize::BinaryOutputSerializer serialize_output(statestream);
	if (!car.Serialize(serialize_output))
	{
		error_output << "Serialization error" << std::endl;
	}
	//else info_output << "Car state: " << statestream.str();
	carstate = statestream.str();

	TestMaxSpeed(info_output, error_output);
	TestStoppingDistance(false, info_output, error_output);
	TestStoppingDistance(true, info_output, error_output);

	info_output << "Car performance test complete." << std::endl;
*/
	// fixme
	info_output << "Car performance test broken - exiting." << std::endl;
	return;
}

void PERFORMANCE_TESTING::ResetCar()
{
//	std::stringstream statestream(carstate);
//	joeserialize::BinaryInputSerializer serialize_input(statestream);
//	car.Serialize(serialize_input);
	car.setPosition(btVector3(0, 0, 0));
	car.setAutoShift(true);
	car.setAutoClutch(true);
	car.setTCS(true);
	car.setABS(true);
}

///designed to be called inside a test's main loop  // broken, fixme
void PERFORMANCE_TESTING::SimulateFlatRoad()
{
/*	//simulate an infinite, flat road
	for (int i = 0; i < 4; i++)
	{
		btVector3 wp = car.dynamics.GetWheelPosition(WHEEL_POSITION(i));
		btScalar depth = wp.z() - car.GetTireRadius(WHEEL_POSITION(i)); //should really project along the car's down vector, but... oh well
		btVector3 pos(wp[0], wp[1], 0);
		btVector3 norm(0, 0, 1);
		car.GetWheelContact(WHEEL_POSITION(i)).Set(pos, norm, depth, &surface, 0, 0);
	}*/
}

void PERFORMANCE_TESTING::TestMaxSpeed(std::ostream & info_output, std::ostream & error_output)
{
	info_output << "Testing maximum speed" << std::endl;

	ResetCar();

	double maxtime = 300.0;
	double t = 0.;
	double dt = .004;
	int i = 0;

	std::pair <float, float> maxspeed;
	maxspeed.first = 0;
	maxspeed.second = 0;
	float lastsecondspeed = 0;
	float stopthreshold = 0.001; //if the accel (in m/s^2) is less than this value, discontinue the testing

	float timeto60start = 0; //don't start the 0-60 clock until the car is moving at a threshold speed to account for the crappy launch that the autoclutch gives
	float timeto60startthreshold = 2.23; //threshold speed to start 0-60 clock in m/s
	float timeto60 = maxtime;

	float timetoquarter = maxtime;
	float quarterspeed = 0;

	std::string downforcestr = "N/A";
	while (t < maxtime)
	{
		car.setThrottle(1);

		world.update(dt);

		if (car.getSpeed() > maxspeed.second)
		{
			maxspeed.first = t;
			maxspeed.second = car.getSpeed();
			std::stringstream dfs;
			btVector3 aero = car.getTotalAero();
			dfs << -aero[0] << " N; " << -aero[0] / aero[2] << ":1 lift/drag";
			downforcestr = dfs.str();
		}

		if (car.getSpeed() < timeto60startthreshold)
			timeto60start = t;

		if (car.getSpeed() < 26.8224)
			timeto60 = t;

		if (car.getCenterOfMass().length() > 402.3 && timetoquarter == maxtime)
		{
			//quarter mile!
			timetoquarter = t - timeto60start;
			quarterspeed = car.getSpeed();
		}

		if (i % (int)(1.0/dt) == 0) //every second
		{
			if (1)
			{
				if (car.getSpeed() - lastsecondspeed < stopthreshold && car.getSpeed() > 26.0)
				{
					//info_output << "Maximum speed attained at " << maxspeed.first << " s" << std::endl;
					break;
				}
				if (car.getTachoRPM() < 1)
				{
					error_output << "Car stalled during launch, t=" << t << std::endl;
					break;
				}
			}
			lastsecondspeed = car.getSpeed();
			//std::cout << t << ", " << car.GetSpeed() << ", " << car.GetGear() << ", " << car.GetEngineRPM() << std::endl;
		}

		t += dt;
		i++;
	}

	info_output << "Maximum speed: " << ConvertToMPH(maxspeed.second) << " MPH at " << maxspeed.first << " s" << std::endl;
	info_output << "Downforce at maximum speed: " << downforcestr << std::endl;
	info_output << "0-60 MPH time: " << timeto60-timeto60start << " s" << std::endl;
	info_output << "1/4 mile time: " << timetoquarter << " s" << " at " << ConvertToMPH(quarterspeed) << " MPH" << std::endl;
}

void PERFORMANCE_TESTING::TestStoppingDistance(bool abs, std::ostream & info_output, std::ostream & error_output)
{
	info_output << "Testing stopping distance" << std::endl;

	ResetCar();

	car.setABS(abs);

	double maxtime = 300.0;
	double t = 0.;
	double dt = .004;
	int i = 0;

	float stopthreshold = 0.1; //if the speed (in m/s) is less than this value, discontinue the testing
	btVector3 stopstart; //where the stopping starts
	float brakestartspeed = 26.82; //speed at which to start braking, in m/s (26.82 m/s is 60 mph)

	bool accelerating = true; //switches to false once 60 mph is reached
	while (t < maxtime)
	{
		if (accelerating)
		{
			car.setThrottle(1);
		}
		else
		{
			car.setThrottle(0);
			car.setBrake(1);
			car.setClutch(1);
		}

		world.update(dt);

		if (car.getSpeed() >= brakestartspeed && accelerating) //stop accelerating and hit the brakes
		{
			accelerating = false;
			stopstart = car.getCenterOfMass();
			//std::cout << "hitting the brakes at " << t << ", " << car.GetSpeed() << std::endl;
		}

		if (!accelerating && car.getSpeed() < stopthreshold)
		{
			break;
		}

		if (car.getTachoRPM() < 1)
		{
			error_output << "Car stalled during launch, t=" << t << std::endl;
			break;
		}

		if (i % (int)(1.0/dt) == 0) //every second
		{
			//std::cout << t << ", " << car.GetSpeed() << ", " << car.GetGear() << ", " << car.GetEngineRPM() << std::endl;
		}

		t += dt;
		i++;
	}

	btVector3 stopend = car.getCenterOfMass();

	info_output << "60-0 stopping distance ";
	if (abs)
		info_output << "(ABS)";
	else
		info_output << "(no ABS)";
	info_output << ": " << ConvertToFeet((stopend-stopstart).length()) << " ft" << std::endl;
}
