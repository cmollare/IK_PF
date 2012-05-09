#ifndef INPUTLISTENER_H
#define INPUTLISTENER_H

#include <Ogre.h>
#include <OIS/OIS.h>

#include <iostream>
using namespace std;

class InputListener: public Ogre::FrameListener, public Ogre::WindowEventListener, public OIS::KeyListener, public OIS::MouseListener
{
public:
	InputListener(Ogre::SceneManager* scMgr, Ogre::RenderWindow *wnd, Ogre::Camera *camera);
	~InputListener();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	
	//OIS
	void startOIS();
	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

private:
	virtual bool mouseMoved(const OIS::MouseEvent &e);
	virtual bool mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id);
	virtual bool mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id);
	virtual bool keyPressed(const OIS::KeyEvent &e);
	virtual bool keyReleased(const OIS::KeyEvent &e);

	Ogre::SceneManager* mSceneMgr;
    Ogre::RenderWindow* mWindow;
    Ogre::Camera*       mCamera;

    OIS::InputManager*  mInputManager;
    OIS::Mouse*         mMouse;
    OIS::Keyboard*      mKeyboard;
    
    bool mContinue;
    
    //Params for trackBallCamera
    float mYaw;
    float mPitch;
    Ogre::Real mZoom;
    bool mMouseLPressed;
    bool mMouseRPressed;
    
    //Avatar *mAvatar;
    //float mCounterAnimAvatar;
};

#endif
