#include "modelfactory.h"
#include "model_joe03.h"
#include <fstream>

Factory<MODEL>::Factory() :
	vbo(false)
{
	// ctor
}

void Factory<MODEL>::init(bool use_vbo)
{
	vbo = use_vbo;
}

template <>
bool Factory<MODEL>::create(
	std::tr1::shared_ptr<MODEL>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const empty&)
{
	const std::string abspath = basepath + "/" + path + "/" + name;
	//error << "Loading: " << abspath << std::endl;
	if (std::ifstream(abspath.c_str()))
	{
		std::tr1::shared_ptr<MODEL_JOE03> temp(new MODEL_JOE03());
		if (temp->Load(abspath, error, !vbo))
		{
			sptr = temp;
			return true;
		}
	}
	return false;
}

template <>
bool Factory<MODEL>::create(
	std::tr1::shared_ptr<MODEL>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const JOEPACK& pack)
{
	std::tr1::shared_ptr<MODEL_JOE03> temp(new MODEL_JOE03());
	if (temp->Load(name, error, !vbo, &pack))
	{
		sptr = temp;
		return true;
	}
	return false;
}

template <>
bool Factory<MODEL>::create(
	std::tr1::shared_ptr<MODEL>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const VERTEXARRAY& varray)
{
	std::tr1::shared_ptr<MODEL> temp(new MODEL());
	if (temp->Load(varray, error, !vbo))
	{
		sptr = temp;
		return true;
	}
	return false;
}
