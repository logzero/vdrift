#include "configfactory.h"
#include "contentmanager.h"
#include "cfg/ptree.h"
#include <fstream>

class ConfigInclude : public Include
{
public:
	ConfigInclude(
		ContentManager & content,
		const std::string & basepath,
		const std::string & path) :
		content(content),
		basepath(basepath),
		path(path)
	{
		// ctor
	}

	void operator()(PTree & node, std::string & value)
	{
		std::tr1::shared_ptr<PTree> sptr;
		if (content.load(sptr, path, value))
		{
			node.set(*sptr);
		}
	}

private:
	ContentManager & content;
	const std::string & basepath;
	const std::string & path;
};

Factory<PTree>::Factory() :
	m_read(&read_ini),
	m_write(&write_ini),
	m_content(0)
{
	// ctor
}

void Factory<PTree>::init(
	void (&read)(std::istream &, PTree &, Include *),
	void (&write)(const PTree &, std::ostream &),
	ContentManager & content)
{
	m_read = &read;
	m_write = &write;
	m_content = &content;
}

template <>
bool Factory<PTree>::create(
	std::tr1::shared_ptr<PTree>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const empty&)
{
	const std::string abspath = basepath + "/" + path + "/" + name;
	std::ifstream file(abspath.c_str());
	if (file)
	{
		sptr.reset(new PTree());
		if (m_content)
		{
			// include support
			ConfigInclude include(*m_content, basepath, path);
			m_read(file, *sptr, &include);
		}
		else
		{
			m_read(file, *sptr, 0);
		}
		return true;
	}
	return false;
}
// replace file string with stream
template <>
bool Factory<PTree>::create(
	std::tr1::shared_ptr<PTree>& sptr,
	std::ostream& error,
	const std::string& basepath,
	const std::string& path,
	const std::string& name,
	const std::string& file)
{
	std::stringstream sstream(file);
	if (sstream.good())
	{
		sptr.reset(new PTree());
		m_read(sstream, *sptr, 0);
		return true;
	}
	return false;
}
