#include "camera_free.h"
#include "coordinatesystem.h"

CAMERA_FREE::CAMERA_FREE(const std::string & name) :
	CAMERA(name),
	offset(0, 0, 2),
	leftright_rotation(0),
	updown_rotation(0)
{
	rotation.LoadIdentity();
}

void CAMERA_FREE::Reset(const MATHVECTOR <float, 3> & newpos, const QUATERNION <float> &)
{
	position = newpos + offset;
	leftright_rotation = 0;
	updown_rotation = 0;
	
	Rotate(0, 0);
	Move(0, -8, 0);
}

void CAMERA_FREE::Rotate(float up, float left)
{
	updown_rotation += up;
	if (updown_rotation > 1.0) updown_rotation = 1.0;
	if (updown_rotation <-1.0) updown_rotation =-1.0;
	leftright_rotation += left;
	
	rotation.LoadIdentity();
	rotation.Rotate(updown_rotation, direction::Right);
	rotation.Rotate(leftright_rotation, direction::Up);
}

void CAMERA_FREE::Move(float dx, float dy, float dz)
{
	MATHVECTOR <float, 3> move(dx, dy, dz);
	MATHVECTOR <float, 3> forward = direction::Forward * direction::Forward.dot(move);
	rotation.RotateVector(forward);
	position = position + forward;
}
