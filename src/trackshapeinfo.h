#ifndef _TRACKSHAPEINFO_H
#define _TRACKSHAPEINFO_H

struct TRACKSURFACE;
class BEZIER;
class MODEL;

struct TrackShapeInfo
{
	const TRACKSURFACE * surface;
	const MODEL * model;
	const BEZIER * patch;
	int patchid;

	TrackShapeInfo() :
		surface(0),
		model(0),
		patch(0),
		patchid(-1)
	{
		// Constructor
	}
};

#endif