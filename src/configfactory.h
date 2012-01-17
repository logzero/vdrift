#ifndef _CONFIGFACTORY_H
#define _CONFIGFACTORY_H

#include "factory.h"
#include <istream>

class PTree;
class ContentManager;
struct Include;

template <>
class Factory<PTree>
{
public:
	struct empty {};

	Factory();

	// content manager is needed for include functionality
	void init(
		void (&read)(std::istream &, PTree &, Include *),
		void (&write)(const PTree &, std::ostream &),
		ContentManager & content);

	template <class P>
	bool create(
		std::tr1::shared_ptr<PTree>& sptr,
		std::ostream& error,
		const std::string& basepath,
		const std::string& path,
		const std::string& name,
		const P& param);

private:
	void (*m_read)(std::istream &, PTree &, Include *);
	void (*m_write)(const PTree &, std::ostream &);
	ContentManager * m_content;
};

#endif // _CONFIGFACTORY_H
