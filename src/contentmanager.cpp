#include "contentmanager.h"

ContentManager::ContentManager(std::ostream & error) :
	error(error)
{
	// ctor
}

ContentManager::~ContentManager()
{
	sweep();
}

void ContentManager::addSharedPath(const std::string & path)
{
	sharedpaths.push_back(path);
}

void ContentManager::addPath(const std::string & path)
{
	basepaths.push_back(path);
}

void ContentManager::sweep()
{
	getFactoryCache.sweep();
}

bool ContentManager::_logerror(
	const std::string & path,
	const std::string & name)
{
	error << "Failed to load \"" << name << "\" from:";
	for (size_t i = 0; i < basepaths.size(); ++i)
	{
		error << "\n\t" << basepaths[i] + '/' + path;
	}
	for (size_t i = 0; i < sharedpaths.size(); ++i)
	{
		error << "\n\t" << sharedpaths[i];
	}
	error << std::endl;
	return false;
}