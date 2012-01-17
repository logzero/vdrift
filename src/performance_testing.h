#ifndef _PERFORMANCE_TESTING_H
#define _PERFORMANCE_TESTING_H

#include "sim/vehicle.h"
#include "sim/surface.h"

#include <string>
#include <ostream>

class DynamicsWorld;

class PERFORMANCE_TESTING
{
public:
	PERFORMANCE_TESTING(sim::World & world);

	void Test(
		const std::string & carpath,
		const std::string & carname,
		const std::string & partspath,
		std::ostream & info_output,
		std::ostream & error_output);

private:
	sim::World & world;
	sim::Surface surface;
	sim::Vehicle car;
	std::string carstate;

	void SimulateFlatRoad();

	void ResetCar();

	void TestMaxSpeed(std::ostream & info_output, std::ostream & error_output);

	void TestStoppingDistance(bool abs, std::ostream & info_output, std::ostream & error_output);
};

#endif
