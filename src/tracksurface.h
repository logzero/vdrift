#ifndef _TRACKSURFACE_H
#define _TRACKSURFACE_H

#include "sim/surface.h"

class TRACKSURFACE : public sim::Surface
{
public:
	float pitch_variation;
	float max_gain;
	int sound_id; // hack, available sounds are asphalt = 0, gravel = 1, grass = 2

	TRACKSURFACE() :
		pitch_variation(0),
		max_gain(0),
		sound_id(0)
	{
		// ctor
	}

	static const TRACKSURFACE * None()
	{
		static const TRACKSURFACE s;
		return &s;
	}
};

#endif //_TRACKSURFACE_H

