#include "soundfactory.h"
#include "sound.h"
#include <fstream>

Factory<SOUND>::Factory() :
	info(0, 0, 0, 0)
{
	// ctor
}

void Factory<SOUND>::init(const SOUNDINFO& value)
{
	info = value;
}

template <>
bool Factory<SOUND>::create(
	std::tr1::shared_ptr<SOUND>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const empty&)
{
	const std::string abspath = basepath + "/" + path + "/" + name;
	std::string filepath = abspath + ".ogg";
	if (!std::ifstream(filepath.c_str()))
	{
		filepath = abspath + ".wav";
	}
	if (std::ifstream(filepath.c_str()))
	{
		std::tr1::shared_ptr<SOUND> temp(new SOUND());
		if (temp->Load(filepath, info, error))
		{
			sptr = temp;
			return true;
		}
	}
	return false;
}
