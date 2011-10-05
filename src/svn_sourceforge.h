#ifndef _SVN_SOURCEFORGE_H
#define _SVN_SOURCEFORGE_H

#include <string>
#include <map>

/// A cheesy HTML parser that mines sourceforge SVN web viewer pages to get repo info
class SVN_SOURCEFORGE
{
public:
	static std::string GetCarFolderUrl() {return data_url + "/cars/";}
	static std::string GetCarDownloadLink(const std::string & car) {return data_url + "/cars/" + car + "/?view=tar";}
	static std::string GetRemoteUpdateConfigUrl() {return data_url + "/settings/updates.config";}

	/// given a sourceforge web svn folder view, return a map of folder names and revisions
	std::map <std::string, int> ParseFolderView(const std::string & folderfile);

private:
	static const std::string data_url;
};

#endif
