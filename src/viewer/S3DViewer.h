#ifndef S3DVIEWER_H
#define S3DVIEWER_H

#include <Ogre.h>
#include "InputListener.h"

class S3DViewer
{
public:
    S3DViewer();
    ~S3DViewer();
    
	void createFrameListener();

    bool start();

private:
    Ogre::Root *mRoot;
    Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	InputListener *mInputListener;
	//Avatar *mAvatar;

};

#endif
