#ifndef _CONTENTMANAGER_H
#define _CONTENTMANAGER_H

#include "soundfactory.h"
#include "texturefactory.h"
#include "modelfactory.h"
#include "configfactory.h"

#include <map>
#include <vector>

class ContentManager
{
public:
	ContentManager(std::ostream & error);

	~ContentManager();

	/// retrieve shared object if in cache
	template <class T>
	bool get(
		std::tr1::shared_ptr<T> & sptr,
		const std::string & path,
		const std::string & name);

	/// retrieve shared object, load if not in cache
	template <class T>
	bool load(
		std::tr1::shared_ptr<T> & sptr,
		const std::string & path,
		const std::string & name);

	/// support additional optional parameters
	template <class T, class P>
	bool load(
		std::tr1::shared_ptr<T> & sptr,
		const std::string & path,
		const std::string & name,
		const P & param);

	/// add shared content directory path
	void addSharedPath(const std::string & path);

	/// add content directory path
	void addPath(const std::string & path);

	/// purge unused content
	void sweep();

	/// factories access
	template <class T>
	Factory<T> & getFactory() { return getFactoryCache; }

private:
	template <class T>
	class Cache : public std::map<std::string, std::tr1::shared_ptr<T> >
	{
	public:
		void sweep();
	};

	/// register content factories
	/// sweep(garbage collection) is mandatory
	struct FactoryCache
	{
		#define REGISTER(T)\
		Factory<T> T ## _factory;\
		Cache<T> T ## _cache;\
		operator Factory<T>&() {return T ## _factory;}\
		operator Cache<T>&() {return T ## _cache;}
		REGISTER(TEXTURE)
		REGISTER(SOUND)
		REGISTER(MODEL)
		REGISTER(PTree)
		#undef REGISTER

		void sweep()
		{
			#define SWEEP(T) T ## _cache.sweep();
			SWEEP(TEXTURE)
			SWEEP(SOUND)
			SWEEP(MODEL)
			SWEEP(PTree)
			#undef SWEEP
		}
	} getFactoryCache;

	/// content paths
	std::vector<std::string> sharedpaths;
	std::vector<std::string> basepaths;

	/// error log
	std::ostream & error;

	/// error logger
	bool _logerror(
		const std::string & path,
		const std::string & name);

	/// get implementation
	template <class T>
	bool _get(
		std::tr1::shared_ptr<T> & sptr,
		const std::string & name);

	/// load implementation
	template <class T, class P>
	bool _load(
		std::tr1::shared_ptr<T> & sptr,
		const std::vector<std::string> & basepaths,
		const std::string & relpath,
		const std::string & name,
		const P & param);
};

template <class T>
inline bool ContentManager::get(
	std::tr1::shared_ptr<T> & sptr,
	const std::string & path,
	const std::string & name)
{
	// check for the specialised version
	// fall back to the generic one
	return 	_get(sptr, path + name) ||
			_get(sptr, name);
}

template <class T>
inline bool ContentManager::load(
	std::tr1::shared_ptr<T> & sptr,
	const std::string & path,
	const std::string & name)
{
	return load(sptr, path, name, typename Factory<T>::empty());
}

template <class T, class P>
inline bool ContentManager::load(
	std::tr1::shared_ptr<T> & sptr,
	const std::string & path,
	const std::string & name,
	const P & param)
{
	// check for the specialised version in basepaths
	// fall back to the generic one in shared paths
	return 	_load(sptr, basepaths, path, name, param) ||
			_load(sptr, sharedpaths, "", name, param) ||
			_logerror(path, name);
}

template <class T>
inline bool ContentManager::_get(
	std::tr1::shared_ptr<T> & sptr,
	const std::string & name)
{
	// retrieve from cache
	Cache<T> & cache = getFactoryCache;
	typename Cache<T>::const_iterator i = cache.find(name);
	if (i != cache.end())
	{
		sptr = i->second;
		return true;
	}
	return false;
}

template <class T, class P>
inline bool ContentManager::_load(
	std::tr1::shared_ptr<T> & sptr,
	const std::vector<std::string> & basepaths,
	const std::string & relpath,
	const std::string & name,
	const P & param)
{
	// check cache
	if (_get(sptr, relpath + name))
	{
		return true;
	}

	// load from basepaths
	Factory<T>& factory = getFactory<T>();
	for (size_t i = 0; i < basepaths.size(); ++i)
	{
		if (factory.create(sptr, error, basepaths[i], relpath, name, param))
		{
			// cache loaded content
			Cache<T> & cache = getFactoryCache;
			cache[relpath + name] = sptr;
			return true;
		}
	}

	return false;
}

template <class T>
inline void ContentManager::Cache<T>::sweep()
{
	// garbage collection here
	typename Cache<T>::iterator it = Cache<T>::begin();
	while (it != Cache<T>::end())
	{
		if (it->second.unique())
		{
			Cache<T>::erase(it++);
		}
		else
		{
			++it;
		}
	}
}

#endif // _CONTENTMANAGER_H
