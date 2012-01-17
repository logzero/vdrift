#ifndef _SOUNDFACTORY_H
#define _SOUNDFACTORY_H

#include "factory.h"
#include "soundinfo.h"

class SOUND;

template <>
class Factory<SOUND>
{
public:
	struct empty {};

	Factory();

	/// sound device setting
	void init(const SOUNDINFO& value);

	template <class P>
	bool create(
		std::tr1::shared_ptr<SOUND>& sptr,
		std::ostream& error,
		const std::string& basepath,
		const std::string& path,
		const std::string& name,
		const P& param);

private:
	SOUNDINFO info;
};

#endif // _SOUNDFACTORY_H
