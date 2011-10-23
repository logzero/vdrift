#ifndef _TRACK_H
#define _TRACK_H

#include "scenenode.h"
#include "tracksurface.h"
#include "trackshapeinfo.h"
#include "roadstrip.h"
#include "mathvector.h"
#include "quaternion.h"
#include "motionstate.h"
#include "dynamicsworld.h"

#include <string>
#include <iostream>
#include <list>
#include <memory>
#include <vector>

class ROADSTRIP;
class ContentManager;

class TRACK
{
public:
	TRACK(float timestep);

	~TRACK();

	/// Use Loaded() to see if loading is complete yet
	bool StartDeferredLoad(
		ContentManager & content,
		std::ostream & info_output,
		std::ostream & error_output,
		const std::string & trackpath,
		const std::string & trackdir,
		const std::string & effects_texturepath,
		const std::string & sharedobjectpath,
		const int anisotropy,
		const bool reverse,
		const bool dynamicobjects,
		const bool dynamicshadows,
		const bool doagressivecombining);

	bool ContinueDeferredLoad();

	/// Number of objects to load in total
	int ObjectsNum() const;

	/// Number of objects loaded
	int ObjectsNumLoaded() const;

	/// Track loading status
	bool Loaded() const
	{
		return data.loaded;
	}

	void Clear();

	void Update(float dt);

	std::pair<MATHVECTOR<float, 3>, QUATERNION<float> > GetStart(unsigned int index) const;

	int GetNumStartPositions() const
	{
		return data.start_positions.size();
	}

	const std::vector<ROADSTRIP> & GetRoads() const
	{
		return data.roads;
	}

	unsigned int GetSectors() const
	{
		return data.lap.size();
	}

	const BEZIER * GetLapSequence(unsigned int sector) const
	{
		assert (sector < data.lap.size());
		return data.lap[sector];
	}

	DynamicsWorld & GetDynamics()
	{
		return data.dynamics;
	}

	void SetRacingLineVisibility(bool newvis)
	{
		data.racingline_visible = newvis;
	}

	bool IsReversed() const
	{
		return data.reverse_direction;
	}

	SCENENODE & GetRacinglineNode()
	{
		if (data.racingline_visible)
			return data.racingline_node;
		else
			return data.empty_node;
	}

	SCENENODE & GetTrackNode()
	{
		return data.static_node;
	}

	SCENENODE & GetBodyNode()
	{
		return data.dynamic_node;
	}

private:
	// Track ray test processor
	struct RayTestProcessor; 

	// Track loader
	class LOADER;
	
	// Track data
	struct DATA
	{
		// Track dynamics
		btDefaultCollisionConfiguration collisionConfig;
		btCollisionDispatcher collisionDispatch;
		btDbvtBroadphase collisionBroadphase;
		btSequentialImpulseConstraintSolver collisionSolver;
		DynamicsWorld dynamics;

		// Static track objects
		SCENENODE static_node;
		std::vector<TRACKSURFACE> surfaces;
		std::vector<std::tr1::shared_ptr<MODEL> > models;
		std::vector<btStridingMeshInterface*> meshes;
		std::vector<btCollisionShape*> shapes;
		std::vector<btCollisionObject*> objects;
		btAlignedObjectArray<TrackShapeInfo> shape_info;

		// Dynamic track objects
		SCENENODE dynamic_node;
		std::vector<keyed_container<SCENENODE>::handle> body_nodes;
		std::list<MotionState> body_transforms;

		// Road information
		std::vector<const BEZIER*> lap;
		std::vector<ROADSTRIP> roads;
		std::vector<std::pair<MATHVECTOR<float, 3>, QUATERNION<float> > > start_positions;
		RayTestProcessor * rayTestProcessor;

		// Racing line data
		SCENENODE empty_node;
		SCENENODE racingline_node;
		std::tr1::shared_ptr<TEXTURE> racingline_texture;

		// Track state
		bool reverse_direction;
		bool vertical_tracking_skyboxes;
		bool racingline_visible;
		bool loaded;
		bool cull;

		DATA(float timestep);
		~DATA();
	};

	DATA data;
	std::auto_ptr<LOADER> loader;
};

#endif
