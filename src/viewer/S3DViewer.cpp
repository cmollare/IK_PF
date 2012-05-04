#include "S3DViewer.h"

S3DViewer::S3DViewer() : mRoot(0)
{
	mDisplayJoint = true;
	mDisplayBone = true;
	mDisplayAxis = true;
}

S3DViewer::~S3DViewer()
{
	delete mInputListener;
	//delete mAvatar;
    delete mRoot;
    delete mLogMgr;
}

bool S3DViewer::init()
{
	mLogMgr = new Ogre::LogManager();
	Ogre::LogManager::getSingleton().createLog("../config/Ogre.log", true, false, false);
	mRoot = new Ogre::Root("../config/plugins.cfg", "../config/ogre.cfg");
	Ogre::ConfigFile configFile;
	configFile.load("../config/resources.cfg");
	Ogre::ConfigFile::SectionIterator seci = configFile.getSectionIterator();
	Ogre::String secName, typeName, archName;
	while (seci.hasMoreElements())
	{
		secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator i;
		for (i = settings->begin(); i != settings->end(); ++i)
		{
			typeName = i->first;
			archName = i->second;
			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
				archName, typeName, secName);
		}
	}
	
	if(!(mRoot->restoreConfig() || mRoot->showConfigDialog()))
	{
		return false;
	}

	mWindow = mRoot->initialise(true, "Viewer");
	
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	mSceneMgr = mRoot->createSceneManager("DefaultSceneManager", "Scene Manager");
	mSceneMgr->setAmbientLight(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

	mCamera = mSceneMgr->createCamera("Default Camera");
	mCamera->setPosition(Ogre::Vector3(0,0,100));
	mCamera->lookAt(Ogre::Vector3(0,0,0));
	mCamera->setNearClipDistance(5);

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
	
	//mAvatar = new Avatar(mSceneMgr);
	defineMaterials();
	mSceneMgr->getRootSceneNode()->createChildSceneNode("Particles");
	mSceneMgr->getRootSceneNode()->createChildSceneNode("lol")->attachObject(createAxis("Axis"));
	mSceneMgr->getSceneNode("lol")->rotate(Vector3(1,1,1), Radian(0.5));
	
	createFrameListener();
	
	return true;
}
	
bool S3DViewer::start()
{
	
	while(true)
	{
		Ogre::WindowEventUtilities::messagePump();
	 
		if(mWindow->isClosed())
			return false;
	 
		if(!mRoot->renderOneFrame())
			return false;
	}

    return true;
}

void S3DViewer::createFrameListener()
{
    mInputListener = new InputListener(mWindow, mCamera);
    mRoot->addFrameListener(mInputListener);
}

void S3DViewer::setOptions(bool displayJoint, bool displayAxis, bool displayBone)
{
	mDisplayJoint = displayJoint;
	mDisplayBone = displayBone;
	mDisplayAxis = displayAxis;
}

void S3DViewer::initModels(vector<S3DModel*>& models)
{
	int i=0;
	if (models.size()>0)
	{
		for(int i=0 ; i<models.size() ; i++)
		{
			cout << "lol" << endl;
			Joint* jt = models[i]->getRootJoint();
			ostringstream oss;
			oss << jt->getName() << "_" << i;
			SceneNode *node = mSceneMgr->getSceneNode("Particles")->createChildSceneNode(oss.str());
			if (jt->hasChildren())
			{
				initModels(jt->getChildren(), node, i);
			}
		}
	}
}

void S3DViewer::initModels(vector<Joint*>& jts, SceneNode *node, int modelNum)
{
	if (jts.size()>0)
	{
		for (int i=0 ; i<jts.size() ; i++)
		{
			ostringstream oss;
			oss << jts[i]->getName() << "_" << modelNum;
			SceneNode *childNode = node->createChildSceneNode(oss.str());
			if (jts[i]->hasChildren())
			{
				initModels(jts[i]->getChildren(), childNode, modelNum);
			}
		}
	}
}

void S3DViewer::defineMaterials()
{
	MaterialPtr myManualObjectMaterial = MaterialManager::getSingleton().create("Red","axis"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(1,0,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(1,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,0);
	
	myManualObjectMaterial = MaterialManager::getSingleton().create("Green","axis"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,1,0,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,1,0); 
	
	myManualObjectMaterial = MaterialManager::getSingleton().create("Blue","axis"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1); 
}

ManualObject* S3DViewer::createAxis(const std::string& strName, int scale)
{
	ManualObject* manual = mSceneMgr->createManualObject(strName);
 
	// specify the material (by name) and rendering type
	manual->begin("Red", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(scale, 0, 0);
	manual->end();
	
	manual->begin("Green", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(0, scale, 0);
	manual->end();
	
	manual->begin("Blue", RenderOperation::OT_LINE_LIST);
	manual->position(0, 0, 0);
	manual->position(0, 0, scale);
	manual->end();
	 
	return manual;
}

