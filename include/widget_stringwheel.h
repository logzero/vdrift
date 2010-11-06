#ifndef _WIDGET_STRINGWHEEL_H
#define _WIDGET_STRINGWHEEL_H

#include "widget.h"
#include "widget_label.h"
#include "widget_button.h"

class FONT;
class SCENENODE;

class WIDGET_STRINGWHEEL : public WIDGET
{
public:
	WIDGET_STRINGWHEEL();
	
	virtual WIDGET * clone() const;
	
	virtual void SetAlpha(SCENENODE & scene, float newalpha);
	
	virtual void SetVisible(SCENENODE & scene, bool newvis);
	
	virtual bool ProcessInput(SCENENODE & scene, float cursorx, float cursory, bool cursordown, bool cursorjustup);
	
	virtual void SetName(const std::string & newname);
	
	virtual std::string GetDescription() const;
	
	virtual void SetDescription(const std::string & newdesc);
	
	///set the local option pointer to the associated optionmap
	virtual void UpdateOptions(
		SCENENODE & scene,
		bool save_to_options,
		std::map<std::string,
		GUIOPTION> & optionmap,
		std::ostream & error_output);
	
	virtual void AddHook(WIDGET * other);
	
	virtual void HookMessage(SCENENODE & scene, const std::string & message, const std::string & from);
	
	void SetAction(const std::string & newaction) {action = newaction;}
	virtual std::string GetAction() const {return active_action;}

	void SetSetting(const std::string & newsetting);
	
	void SetupDrawable(
		SCENENODE & scene,
		const std::string & newtitle,
		std::tr1::shared_ptr<TEXTURE> teximage_left_up,
		std::tr1::shared_ptr<TEXTURE> teximage_left_down,
		std::tr1::shared_ptr<TEXTURE> teximage_right_up,
		std::tr1::shared_ptr<TEXTURE> teximage_right_down,
		const FONT * font,
		const float scalex,
		const float scaley,
		const float centerx,
		const float centery);
	
private:
	WIDGET_LABEL title;
	WIDGET_LABEL label;
	WIDGET_BUTTON button_left;
	WIDGET_BUTTON button_right;
	GUIOPTION * option;
	std::string name;
	std::string description;
	std::string setting;
	std::list <WIDGET *> hooks;
	std::string action;
	std::string active_action;
	
	void SyncOption(SCENENODE & scene);
};

#endif // _WIDGET_STRINGWHEEL_H
