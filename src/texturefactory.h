#ifndef _TEXTUREFACTORY_H
#define _TEXTUREFACTORY_H

#include "factory.h"
#include "textureinfo.h"

class TEXTURE;

template <>
class Factory<TEXTURE>
{
public:
	struct empty {};

	Factory();

	/// in general all textures on disk will be in the SRGB colorspace, so if the renderer wants to do
	/// gamma correct lighting, it will want all textures to be gamma corrected using the SRGB flag
	/// limit texture size to max size
	void init(int max_size, bool use_srgb);

	template <class P>
	bool create(
		std::tr1::shared_ptr<TEXTURE>& sptr,
		std::ostream& error,
		const std::string& basepath,
		const std::string& path,
		const std::string& name,
		const P& param);

private:
	int size;
	bool srgb;
};

#endif // _TEXTUREFACTORY_H
