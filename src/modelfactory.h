#ifndef _MODELFACTORY_H
#define _MODELFACTORY_H

#include "factory.h"

class MODEL;

template <>
class Factory<MODEL>
{
public:
	struct empty {};

	Factory();

	/// use VBOs instead of draw lists for models
	void init(bool use_vbo);

	template <class P>
	bool create(
		std::tr1::shared_ptr<MODEL>& sptr,
		std::ostream& error,
		const std::string& basepath,
		const std::string& path,
		const std::string& name,
		const P& param);

private:
	bool vbo;
};

#endif // _MODELFACTORY_H
