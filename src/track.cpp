#include "track.h"
#include "trackloader.h"
#include "dynamicsworld.h"
#include "tobullet.h"

#include <map>
#include <algorithm>
#include <fstream>
#include <sstream>

struct TRACK::RayTestProcessor : public DynamicsWorld::RayTestProcessor
{
	DATA & data;
	btCollisionWorld::RayResultCallback * rayCb;
	btVector3 rayFrom;
	btVector3 rayTo;

	RayTestProcessor(DATA & data) : data(data), rayCb(0)
	{
		// Constructor
	}

	void rayTest(const btVector3 & rayFromWorld, const btVector3 & rayToWorld, RayResultCallback& cb)
	{
		rayFrom = rayFromWorld;
		rayTo = rayToWorld;
		rayCb = &cb;
	}

	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		// We only support static mesh collision shapes
		if (!(rayResult.m_collisionObject->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT) &&
			(rayResult.m_collisionObject->getCollisionShape()->getShapeType() != TRIANGLE_MESH_SHAPE_PROXYTYPE))
		{
			return 1.0;
		}
		//std::cerr << rayResult.m_hitFraction << "    ";

		const btCollisionShape * s = rayResult.m_collisionObject->getCollisionShape();
		TrackShapeInfo * si = static_cast<TrackShapeInfo*>(s->getUserPointer());

		// Road bezier patch ray test
		btVector3 rayVec = rayTo - rayFrom;
		float rayLen = rayVec.length();

		MATHVECTOR<float, 3> from(rayFrom[1], rayFrom[2], rayFrom[0]);
		MATHVECTOR<float, 3> to(rayTo[1], rayTo[2], rayTo[0]);
		MATHVECTOR<float, 3> dir(rayVec[1], rayVec[2], rayVec[0]);
		dir = dir * 1 / rayLen;

		MATHVECTOR<float, 3> colPoint;
		MATHVECTOR<float, 3> colNormal;
		int patchid = si->patchid;
		const BEZIER * colBez = 0;
		for (std::vector<ROADSTRIP>::const_iterator i = data.roads.begin(); i != data.roads.end(); ++i)
		{
			const BEZIER * bez(0);
			MATHVECTOR<float, 3> norm;
			MATHVECTOR<float, 3> point(to);
			if (i->Collide(from, dir, rayLen, patchid, point, bez, norm) &&
				((point - from).MagnitudeSquared() < (colPoint - from).MagnitudeSquared()))
			{
				colPoint = point;
				colNormal = norm;
				colBez = bez;
			}
		}

		if (colBez)
		{
			btVector3 hitPoint(colPoint[2], colPoint[0], colPoint[1]);
			btVector3 hitNormal(colNormal[2], colNormal[0], colNormal[1]);
			btScalar dist_p = hitNormal.dot(hitPoint);
			btScalar dist_a = hitNormal.dot(rayFrom);
			btScalar dist_b = hitNormal.dot(rayTo);
			btScalar fraction = (dist_a - dist_p) / (dist_a - dist_b);
			rayResult.m_hitFraction = fraction;
			rayResult.m_hitNormalLocal = hitNormal;
			si->patch = colBez;
			si->patchid = patchid;
		}
		//std::cerr << rayResult.m_hitFraction << "    ";

		return rayCb->addSingleResult(rayResult, true);
	}
};

TRACK::TRACK(float timestep) :
	data(timestep)
{
	// Constructor
}

TRACK::~TRACK()
{
	Clear();
}

TRACK::DATA::DATA(float timestep) :
	collisionDispatch(&collisionConfig),
	dynamics(&collisionDispatch, &collisionBroadphase, &collisionSolver, &collisionConfig, timestep),
	reverse_direction(false),
	vertical_tracking_skyboxes(false),
	racingline_visible(false),
	loaded(false),
	cull(true)
{
	rayTestProcessor = new RayTestProcessor(*this);
	dynamics.setRayTestProcessor(*rayTestProcessor);
}

TRACK::DATA::~DATA()
{
	delete rayTestProcessor;
}

bool TRACK::StartDeferredLoad(
	ContentManager & content,
	std::ostream & info_output,
	std::ostream & error_output,
	const std::string & trackpath,
	const std::string & trackdir,
	const std::string & texturedir,
	const std::string & sharedobjectpath,
	const int anisotropy,
	const bool reverse,
	const bool dynamicobjects,
	const bool dynamicshadows,
	const bool agressivecombine)
{
	Clear();

	data.dynamics.reset();

	loader.reset(
		new LOADER(
			content, data,
			info_output, error_output,
			trackpath, trackdir,
			texturedir,	sharedobjectpath,
			anisotropy, reverse,
			dynamicobjects,
			dynamicshadows,
			agressivecombine));

	return loader->BeginLoad();
}

bool TRACK::ContinueDeferredLoad()
{
	assert(loader.get());
	return loader->ContinueLoad();
}

int TRACK::ObjectsNum() const
{
	assert(loader.get());
	return loader->GetNumObjects();
}

int TRACK::ObjectsNumLoaded() const
{
	assert(loader.get());
	return loader->GetNumLoaded();
}

void TRACK::Clear()
{
	for(int i = 0, n = data.objects.size(); i < n; ++i)
	{
		data.dynamics.removeCollisionObject(data.objects[i]);
		delete data.objects[i];
	}
	data.objects.clear();

	for(int i = 0, n = data.shapes.size(); i < n; ++i)
	{
		delete data.shapes[i];
	}
	data.shapes.clear();

	for(int i = 0, n = data.meshes.size(); i < n; ++i)
	{
		delete data.meshes[i];
	}
	data.meshes.clear();

	data.static_node.Clear();
	data.surfaces.clear();
	data.models.clear();
	data.dynamic_node.Clear();
	data.body_nodes.clear();
	data.body_transforms.clear();
	data.lap.clear();
	data.roads.clear();
	data.start_positions.clear();
	data.racingline_node.Clear();
	data.loaded = false;
}

optional <const BEZIER *> ROADSTRIP::FindBezierAtOffset(const BEZIER * bezier, int offset) const
{
	std::vector<ROADPATCH>::const_iterator it = patches.end(); //this iterator will hold the found ROADPATCH

	//search for the roadpatch containing the bezier and store an iterator to it in "it"
	for (std::vector<ROADPATCH>::const_iterator i = patches.begin(); i != patches.end(); ++i)
	{
		if (&i->GetPatch() == bezier)
		{
			it = i;
			break;
		}
	}

	if (it == patches.end())
		return optional <const BEZIER *>(); //return nothing
	else
	{
		//now do the offset
		int curoffset = offset;
		while (curoffset != 0)
		{
			if (curoffset < 0)
			{
				//why is this so difficult?  all i'm trying to do is make the iterator loop around
				std::vector<ROADPATCH>::const_reverse_iterator rit(it);
				if (rit == patches.rend())
					rit = patches.rbegin();
				rit++;
				if (rit == patches.rend())
					rit = patches.rbegin();
				it = rit.base();
				if (it == patches.end())
					it = patches.begin();

				curoffset++;
			}
			else if (curoffset > 0)
			{
				it++;
				if (it == patches.end())
					it = patches.begin();

				curoffset--;
			}
		}

		assert(it != patches.end());
		return optional <const BEZIER *>(&it->GetPatch());
	}
}

std::pair <MATHVECTOR <float, 3>, QUATERNION <float> > TRACK::GetStart(unsigned int index) const
{
	assert(!data.start_positions.empty());
	unsigned int laststart = data.start_positions.size()-1;
	if (index > laststart)
	{
		std::pair <MATHVECTOR <float, 3>, QUATERNION <float> > sp = data.start_positions[laststart];
		MATHVECTOR <float, 3> backward(6,0,0);
		backward = backward * (index-laststart);
		sp.second.RotateVector(backward);
		sp.first = sp.first + backward;
		return sp;
	}
	return data.start_positions[index];
}

void TRACK::Update(float dt)
{
	if (!data.loaded) return;

	data.dynamics.update(dt);

	// sync graphics
	std::list<MotionState>::const_iterator t = data.body_transforms.begin();
	for (int i = 0, e = data.body_nodes.size(); i < e; ++i, ++t)
	{
		TRANSFORM & vt = data.dynamic_node.GetNode(data.body_nodes[i]).GetTransform();
		vt.SetRotation(ToMathQuaternion<float>(t->rotation));
		vt.SetTranslation(ToMathVector<float>(t->position));
	}
}
