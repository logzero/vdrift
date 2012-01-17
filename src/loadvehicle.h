#ifndef _LOADVEHICLE_H
#define _LOADVEHICLE_H

#include <ostream>

class PTree;
class btVector3;
namespace sim { struct VehicleInfo; }

bool LoadVehicle(
	const PTree & cfg,
	const bool damage,
	const btVector3 & modelcenter,
	const btVector3 & modelsize,
	sim::VehicleInfo & info,
	std::ostream & error);

#endif // _LOADVEHICLE_H
