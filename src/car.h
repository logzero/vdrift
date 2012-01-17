#ifndef _CAR_H
#define _CAR_H

#include "sim/vehicle.h"
#include "sim/motionstate.h"
#include "tobullet.h"
#include "scenenode.h"
#include "soundsource.h"
#include "camera_system.h"
#include "joeserialize.h"
#include "macros.h"
#include "suspensionbumpdetection.h"
#include "crashdetection.h"
#include "enginesoundinfo.h"

class BEZIER;
class PERFORMANCE_TESTING;
class MODEL;
class ContentManager;
class PTree;

class CAR
{
friend class PERFORMANCE_TESTING;
friend class joeserialize::Serializer;
public:
	CAR();

	bool LoadGraphics(
		const PTree & cfg,
		const std::string & carpath,
		const std::string & carname,
		const std::string & partspath,
		const MATHVECTOR <float, 3> & carcolor,
		const std::string & carpaint,
		const int anisotropy,
		const float camerabounce,
		const bool damage,
		const bool debugmode,
		ContentManager & content,
		std::ostream & info_output,
		std::ostream & error_output);

	bool LoadSounds(
		const PTree & cfg,
		const std::string & carpath,
		const std::string & carname,
		ContentManager & content,
		std::ostream & info_output,
		std::ostream & error_output);

	bool LoadPhysics(
		const PTree & cfg,
		const std::string & carpath,
		const MATHVECTOR <float, 3> & position,
		const QUATERNION <float> & orientation,
		const bool defaultabs,
		const bool defaulttcs,
		const bool damage,
		ContentManager & content,
		sim::World & world,
		std::ostream & info_output,
		std::ostream & error_output);

	// change car color
	void SetColor(float r, float g, float b);

	// enable/disable transparent parts rendering
	void EnableGlass(bool enable);

	// should  be input, select first gear at race start
	void SetGear(int gear) { dynamics.setGear(gear); }

	// should be car input or property(not modifiyable after creation)?
	void SetAutoClutch(bool value) { dynamics.setAutoClutch(value); }

	// should be car input or property?
	void SetAutoShift(bool value) { dynamics.setAutoShift(value); }

	// unrelated to car, should be removed
	void SetSector(int value) { sector = value; }
	int GetSector() const { return sector; }

	// will align car relative to track surface
	void SetPosition(const MATHVECTOR<float, 3> & position);

	void ProcessInputs(const std::vector<float> & inputs, float dt);

	void Update(double dt);

	void GetSoundList(std::list <SOUNDSOURCE *> & outputlist);

	void GetEngineSoundList(std::list <SOUNDSOURCE *> & outputlist);

	int GetWheelCount() const
	{
		return dynamics.getWeelCount();
	}

	MATHVECTOR<float, 3> GetCenterOfMassPosition() const
	{
		return ToMathVector<float>(dynamics.getCenterOfMass());
	}

	MATHVECTOR<float, 3> GetPosition() const
	{
		return ToMathVector<float>(motion_state[0].position);
	}

	QUATERNION<float> GetOrientation() const
	{
		return ToMathQuaternion<float>(motion_state[0].rotation);
	}

	MATHVECTOR<float, 3> GetWheelPosition(int i) const
	{
		return ToMathVector<float>(motion_state[i+1].position);
	}

	float GetTireRadius(int i) const
	{
		return dynamics.getWheel(i).getRadius();
	}

	CAMERA_SYSTEM & Cameras()
	{
		return cameras;
	}

	int GetEngineRedline() const
	{
		return dynamics.getEngine().getRedline();
	}

	int GetEngineRPMLimit() const
	{
		return dynamics.getEngine().getRPMLimit();
	}

	bool GetOutOfGas() const
	{
		return dynamics.getOutOfGas();
	}

	float GetNosAmount() const
	{
		return dynamics.getNosAmount();
	}

	bool GetNosActive() const
	{
		return nosactive;
	}

	int GetGear() const
	{
		return dynamics.getTransmission().getGear();
	}

	float GetClutch()
	{
		return dynamics.getClutch().getPosition();
	}

	bool GetABSEnabled() const
	{
		return dynamics.getABSEnabled();
	}

	bool GetABSActive() const
	{
		return dynamics.getABSActive();
	}

	bool GetTCSEnabled() const
	{
		return dynamics.getTCSEnabled();
	}

	bool GetTCSActive() const
	{
		return dynamics.getTCSActive();
	}

	float GetSpeedMPS()
	{
		return dynamics.getSpeedMPS();
	}

	float GetMaxSpeedMPS()
	{
		return dynamics.getMaxSpeedMPS();
	}

	float GetBrakingDistance(float target_velocity)
	{
		return dynamics.getBrakingDistance(target_velocity);
	}

	float GetMaxVelocity(float radius)
	{
		return dynamics.getMaxVelocity(radius);
	}

	const std::string & GetCarType() const
	{
		return cartype;
	}

	const BEZIER * GetCurPatch(int i) const
	{
		return dynamics.getWheel(i).ray.getPatch();
	}

	float GetLastSteer() const
	{
		return last_steer;
	}

	float GetSpeed()
	{
		return dynamics.getSpeed();
	}

	float GetFeedback();

	// returns a float from 0.0 to 1.0 with the amount of tire squealing going on
	float GetTireSquealAmount(int i) const;

	int GetEngineRPM() const
	{
		return dynamics.getTachoRPM();
	}

	int GetEngineStallRPM() const
	{
		return dynamics.getEngine().getStallRPM();
	}

	float GetInvMass() const
	{
		return dynamics.getInvMass();
	}

	MATHVECTOR<float, 3> GetVelocity() const
	{
		return ToMathVector<float>(dynamics.getVelocity());
	}

	// ideal steering angle in degrees
	float GetIdealSteeringAngle() const
	{
		return dynamics.getWheel(0).tire.getIdealSlip();
	}

	// maximum steering angle in degrees
	float GetMaxSteeringAngle() const
	{
		return dynamics.getMaxSteeringAngle();
	}

	SCENENODE & GetNode()
	{
		return topnode;
	}

	void DebugPrint(std::ostream & out, bool p1, bool p2, bool p3, bool p4) const;

	bool Serialize(joeserialize::Serializer & s);

protected:
	SCENENODE topnode;

	// body + n wheels + m children shapes
	btAlignedObjectArray<sim::MotionState> motion_state;
	sim::Vehicle dynamics;

	keyed_container<SCENENODE>::handle bodynode;
	keyed_container<SCENENODE>::handle steernode;
	keyed_container<DRAWABLE>::handle brakelights;
	keyed_container<DRAWABLE>::handle reverselights;

	struct LIGHT
	{
		keyed_container<SCENENODE>::handle node;
		keyed_container<DRAWABLE>::handle draw;
	};
	std::vector<LIGHT> lights;
	std::list<std::tr1::shared_ptr<MODEL> > models;

	CRASHDETECTION crashdetection;
	CAMERA_SYSTEM cameras;

	std::vector<std::pair<ENGINESOUNDINFO, SOUNDSOURCE> > enginesounds;
	std::vector<SOUNDSOURCE> tiresqueal;
	std::vector<SOUNDSOURCE> grasssound;
	std::vector<SOUNDSOURCE> gravelsound;
	std::vector<SOUNDSOURCE> tirebump;
	SOUNDSOURCE crashsound;
	SOUNDSOURCE gearsound;
	SOUNDSOURCE brakesound;
	SOUNDSOURCE handbrakesound;
	SOUNDSOURCE roadnoise;

	int gearsound_check;
	bool brakesound_check;
	bool handbrakesound_check;

	// relative engine position, used for engine sounds
	MATHVECTOR<float, 3> engine_position;

	// steering wheel
	QUATERNION<float> steer_orientation;
	QUATERNION<float> steer_rotation;
	float steer_angle_max;

	//internal variables that might change during driving (so, they need to be serialized)
	float last_steer;
	bool lookbehind;
	bool nosactive;

	std::string cartype;
	int sector; //the last lap timing sector that the car hit
	std::vector<const BEZIER *> curpatch; //the last bezier patch that each wheel hit

	float applied_brakes; ///< cached so we can update the brake light

	float mz_nominalmax; //the nominal maximum Mz force, used to scale force feedback

	void UpdateSounds(float dt);

	void UpdateCameras(float dt);

	void UpdateGraphics();

	bool LoadLight(
		const PTree & cfg,
		ContentManager & content,
		std::ostream & error_output);

	bool LoadSoundConfig(
		const std::string & carpath,
		const std::string & carname,
		ContentManager & content,
		std::ostream & info_output,
		std::ostream & error_output);
};

#endif
