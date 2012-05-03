#ifndef S3DVIEWER_H
#define S3DVIEWER_H

#include <Ogre.h>
#include "InputListener.h"
#include "../3DModel/S3DModel.h"

class S3DViewer
{
public:
    S3DViewer();
    ~S3DViewer();
    
	void createFrameListener();
    bool start();
    
    void setOptions(bool displayJoint=true, bool displayAxis=true, bool displayBone=false);
    void initModels();

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
