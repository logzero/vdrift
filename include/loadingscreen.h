#ifndef _LOADINGSCREEN_H
#define _LOADINGSCREEN_H

#include "texture.h"
#include "scenenode.h"
#include "vertexarray.h"

#include <ostream>
#include <string>

class LOADINGSCREEN
{
private:
	SCENENODE root;
	keyed_container <DRAWABLE>::handle bardraw;
	VERTEXARRAY barverts;
	keyed_container <DRAWABLE>::handle barbackdraw;
	VERTEXARRAY barbackverts;
	keyed_container <DRAWABLE>::handle boxdraw;
	VERTEXARRAY boxverts;
	float w, h, hscale;
	
public:
	SCENENODE & GetNode() {return root;}
	
	///initialize the loading screen given the root node for the loading screen
	bool Initialize(const std::string & texturepath, int displayw, int displayh, const std::string & texsize, TEXTUREMANAGER & textures, std::ostream & error_output)
	{
		TEXTUREINFO boxtexinfo(texturepath+"/loadingbox.png");
		boxtexinfo.SetMipMap(false);
		boxtexinfo.SetSize(texsize);
		TEXTUREPTR boxtex = textures.Get(boxtexinfo);
		if (!boxtex->Loaded())
		{
			error_output << "Error loading graphic for loading screen." << std::endl;
			return false;
		}
		
		TEXTUREINFO bartexinfo(texturepath+"/loadingbar.png");
		bartexinfo.SetMipMap(false);
		bartexinfo.SetSize(texsize);
		TEXTUREPTR bartex = textures.Get(bartexinfo);
		if (!bartex->Loaded())
		{
			error_output << "Error loading graphic for loading screen." << std::endl;
			return false;
		}
		
		bardraw = root.GetDrawlist().twodim.insert(DRAWABLE());
		boxdraw = root.GetDrawlist().twodim.insert(DRAWABLE());
		barbackdraw = root.GetDrawlist().twodim.insert(DRAWABLE());
		DRAWABLE & bardrawref = root.GetDrawlist().twodim.get(bardraw);
		DRAWABLE & boxdrawref = root.GetDrawlist().twodim.get(boxdraw);
		DRAWABLE & barbackdrawref = root.GetDrawlist().twodim.get(barbackdraw);
		
		boxdrawref.SetDiffuseMap(boxtex);
		boxdrawref.SetVertArray(&boxverts);
		boxdrawref.SetDrawOrder(0);
		boxdrawref.SetLit(false);
		boxdrawref.Set2D(true);
		boxdrawref.SetCull(false, false);
		boxdrawref.SetColor(1,1,1,1);
		
		w = 128.0/displayw;
		h = 128.0/displayw;
		boxverts.SetTo2DButton(0.5,0.5,w,h,w*0.5,false);
		
		barbackdrawref.SetDiffuseMap(bartex);
		barbackdrawref.SetVertArray(&barbackverts);
		barbackdrawref.SetDrawOrder(1);
		barbackdrawref.SetLit(false);
		barbackdrawref.Set2D(true);
		barbackdrawref.SetCull(false, false);
		barbackdrawref.SetColor(0.3, 0.3, 0.3, 0.4);
		
		hscale = 0.3;
		barbackverts.SetToBillboard(0.5-w*0.5,0.5-h*0.5*hscale,0.5+w*0.5, 0.5+h*0.5*hscale);
		
		bardrawref.SetDiffuseMap(bartex);
		bardrawref.SetVertArray(&barverts);
		bardrawref.SetDrawOrder(2);
		bardrawref.SetLit(false);
		bardrawref.Set2D(true);
		bardrawref.SetCull(false, false);
		bardrawref.SetColor(1,1,1, 0.7);
		
		return true;
	}
	
	void Update(float percentage)
	{
		if (percentage < 0)
			percentage = 0;
		if (percentage > 1.0)
			percentage = 1.0;
		
		barverts.SetToBillboard(0.5-w*0.5,0.5-h*0.5*hscale,0.5-w*0.5+w*percentage, 0.5+h*0.5*hscale);
	}
};

#endif
