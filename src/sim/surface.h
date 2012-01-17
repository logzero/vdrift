#ifndef _SIM_SURFACE_H
#define _SIM_SURFACE_H

namespace sim
{

struct Surface
{
	float bumpWaveLength;
	float bumpAmplitude;
	float frictionNonTread;
	float frictionTread;
	float rollResistanceCoefficient;
	float rollingDrag;

	Surface() :
		bumpWaveLength(1),
		bumpAmplitude(0),
		frictionNonTread(0),
		frictionTread(0),
		rollResistanceCoefficient(0),
		rollingDrag(0)
	{
		// ctor
	}

	static const Surface * None()
	{
		static const Surface s;
		return &s;
	}
};

}

#endif

