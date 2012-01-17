#include "texturefactory.h"
#include "texture.h"
#include <fstream>

Factory<TEXTURE>::Factory() :
	size(TEXTUREINFO::LARGE), srgb(false)
{
	// ctor
}

void Factory<TEXTURE>::init(int max_size, bool use_srgb)
{
	size = max_size;
	srgb = use_srgb;
}

template <>
bool Factory<TEXTURE>::create(
	std::tr1::shared_ptr<TEXTURE>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const TEXTUREINFO& info)
{
	const std::string abspath = basepath + "/" + path + "/" + name;
	if (info.data || std::ifstream(abspath.c_str()))
	{
		TEXTUREINFO info_temp = info;
		info_temp.srgb = srgb;
		info_temp.maxsize = TEXTUREINFO::Size(size);
		std::tr1::shared_ptr<TEXTURE> temp(new TEXTURE());
		if (temp->Load(abspath, info_temp, error))
		{
			sptr = temp;
			return true;
		}
	}
	return false;
}
