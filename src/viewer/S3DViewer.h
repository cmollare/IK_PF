#ifndef S3DVIEWER_H
#define S3DVIEWER_H

#include <Ogre.h>
#include <vector>
#include "InputListener.h"
#include "../3DModel/S3DModel.h"

using namespace Ogre;

class S3DViewer
{
public:
    S3DViewer();
    ~S3DViewer();
    
	void createFrameListener();
    bool start();
    
    void setOptions(bool displayJoint=true, bool displayAxis=true, bool displayBone=false);
    void initModels(vector<S3DModel*> models);
    
    //Drawing functions
    void defineMaterials();
    ManualObject* createAxis(const std::string& strName, int scale=3);

private:
    Ogre::Root *mRoot;
    Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	InputListener *mInputListener;
	Ogre::LogManager* mLogMgr;
	//Avatar *mAvatar;
	
	//Display options
	bool mDisplayJoint;
	bool mDisplayBone;
	bool mDisplayAxis;
	//End display options

};

#endif
