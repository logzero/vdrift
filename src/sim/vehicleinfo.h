#ifndef _SIM_VEHICLEINFO_H
#define _SIM_VEHICLEINFO_H

#include "sim/fracturebody.h"
#include "sim/aerodevice.h"
#include "sim/differential.h"
#include "sim/wheel.h"
#include "sim/transmission.h"
#include "sim/clutch.h"
#include "sim/engine.h"

namespace sim
{

struct VehicleInfo
{
	// has to contain at least wheels + body, first n bodies are the wheels
	// motion states are for: vehicle body + n wheels + m children bodies
	// minimum number of wheels is 2
	FractureBodyInfo body;
	btAlignedObjectArray<MotionState*> motionstate;
	btAlignedObjectArray<AeroDeviceInfo> aerodevice;
	btAlignedObjectArray<DifferentialInfo> differential;
	btAlignedObjectArray<WheelInfo> wheel;
	TransmissionInfo transmission;
	ClutchInfo clutch;
	EngineInfo engine;

	// driveline link targets are n wheels + m differentials
	// 0 <= shaft id < wheel.size() + differential.size()
	// the link graph has to be without cycles (tree)
	// differential link count is equal to differential count
	btAlignedObjectArray<int> differential_link_a;
	btAlignedObjectArray<int> differential_link_b;
	int transmission_link;

	VehicleInfo() : body(motionstate) {}
};

}

#endif // _SIM_VEHICLEINFO_H
