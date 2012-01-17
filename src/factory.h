#ifndef _FACTORY_H
#define _FACTORY_H

#include "memory.h"
#include <ostream>
#include <string>

template <class T>
class Factory
{
public:
	struct empty {};

	template <class P>
	bool create(
		std::tr1::shared_ptr<T>& sptr,
		std::ostream& error,
		const std::string& basepath,
		const std::string& path,
		const std::string& name,
		const P& param)
	{
		return false;
	}
};

#endif // _FACTORY_H
