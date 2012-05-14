#include "S3DViewer.h"

S3DViewer::S3DViewer() : mRoot(0)
{
	mDisplayJoint = true;
	mDisplayBone = true;
	mDisplayAxis = true;
}

S3DViewer::~S3DViewer()
{
	//ajouter les destructions des Line3D !!!
	delete mInputListener;
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
	mCamera->setPosition(Ogre::Vector3(0,0,-10));
	mCamera->lookAt(Ogre::Vector3(0,0,0));
	mCamera->setNearClipDistance(0.1);

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);
	vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
	mCamera->setAspectRatio(Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
	
	//Load models
	defineMaterials();
	SceneNode *filteringNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Filtering");
	filteringNode->createChildSceneNode("Particles");
	filteringNode->createChildSceneNode("Observations");
	mSceneMgr->getRootSceneNode()->createChildSceneNode("Debug")->attachObject(createAxis("Axis_ref_debug",5));
	
	//mSceneMgr->getRootSceneNode()->createChildSceneNode("lol")->attachObject(createAxis("Axis"));
	//mSceneMgr->getSceneNode("lol")->rotate(Vector3(1,1,1), Radian(0.5));
	
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

bool S3DViewer::isRendering()
{
	Ogre::WindowEventUtilities::messagePump();
	 
	if(mWindow->isClosed())
		return false;
	 
	if(!mRoot->renderOneFrame())
		return false;
		
	return true;
}

void S3DViewer::createFrameListener()
{
    mInputListener = new InputListener(mSceneMgr, mWindow, mCamera);
    mRoot->addFrameListener(mInputListener);
}

void S3DViewer::displaySampling(Eigen::Quaternionf quat)
{
	std::ostringstream oss;
	Ogre::Vector3 vec(quat.x(), quat.y(), quat.z());
	vec.normalise();
	vec=vec*5;
	SceneNode *node = mSceneMgr->getSceneNode("Particles")->createChildSceneNode(vec);
	oss << "Axis_particule_" << node->getName() << endl;
	node->attachObject(createAxis(oss.str(),1));
}

void S3DViewer::setOptions(bool displayJoint, bool displayAxis, bool displayBone)
{
	mDisplayJoint = displayJoint;
	mDisplayBone = displayBone;
	mDisplayAxis = displayAxis;
}

void S3DViewer::initModels(std::vector<S3DModel*>& models)
{
	mModelSNNames.clear();
	int i=0;
	if (models.size()>0)
	{
		for(int i=0 ; i<models.size() ; i++)
		{
			std::vector<std::string> snNames;
			Joint* jt = models[i]->getRootJoint();
			ostringstream oss;
			oss << jt->getName() << "_" << i;
			
			//Orientation of Joint
			Eigen::Quaternionf vQuat = *(jt->getOrientation());
			Ogre::Quaternion quat((float)vQuat.w(), (float)vQuat.x(), (float)vQuat.y(), (float)vQuat.z());
			
			//offset
			Eigen::Translation3f vOff = *(jt->getOffset());
			Vector3 offset(vOff.x(), vOff.y(), vOff.z());
			
			//Creation of sceneNode with orientation and offset
			SceneNode *node = mSceneMgr->getSceneNode("Particles")->createChildSceneNode(oss.str(), offset, quat);

			snNames.push_back(oss.str());

			
			if (mDisplayAxis)//option to display axis
			{
				ostringstream oss2;
				oss2 << "Axis_" << oss.str();
				node->attachObject(createAxis(oss2.str()));
			}
			if (jt->hasChildren())
			{
				initModels(jt->getChildren(), node, i, snNames);//Recursivity
			}
			mModelSNNames.push_back(snNames);
		}
	}

}

void S3DViewer::initObservations(std::vector<std::string> jtNames, std::vector<std::vector<double> > frame)
{
	mObsNameVec = jtNames;
	mObsCurrentFrame = frame;
	mObservationSNNames.clear();
	
	Ogre::SceneNode *obsNode = mSceneMgr->getSceneNode("Observations");
	
	for (int i=0 ; i<mObsNameVec.size() ; i++)
	{
		ostringstream oss;
		mObsMap[mObsNameVec[i]]=i;
		oss << "obs_" << mObsNameVec[i];
		Ogre::SceneNode *tempoNode = obsNode->createChildSceneNode(oss.str(), Ogre::Vector3(mObsCurrentFrame[i][1], mObsCurrentFrame[i][2], mObsCurrentFrame[i][3]));
		mObservationSNNames.push_back(oss.str());
		oss.clear();
		oss << "axisObs_" << mObsNameVec[i];
		tempoNode->attachObject(createAxis(oss.str()));
	}
}

void S3DViewer::initModels(std::vector<Joint*>& jts, SceneNode *node, int modelNum, std::vector<std::string>& snNames)
{
	if (jts.size()>0)
	{
		for (int i=0 ; i<jts.size() ; i++)
		{
			ostringstream oss;
			oss << jts[i]->getName() << "_" << modelNum;
			
			Eigen::Quaternionf vQuat = *(jts[i]->getOrientation());
			Ogre::Quaternion quat(vQuat.w(), vQuat.x(), vQuat.y(), vQuat.z());
			
			//offset
			Eigen::Translation3f vOff = *(jts[i]->getOffset());
			Vector3 offset(vOff.x(), vOff.y(), vOff.z());
			
			//Creation of node with orientation and offset
			SceneNode *childNode = node->createChildSceneNode(oss.str(), offset, quat);
			snNames.push_back(oss.str());
			
			if (mDisplayAxis)//option to display axis
			{
				ostringstream oss2;
				oss2 << "Axis_" << oss.str();
				childNode->attachObject(createAxis(oss2.str()));
			}
			if (mDisplayBone)//option to display bones
			{
				ostringstream oss2;
				oss2 << "Line_" << oss.str();
				Line3D *line = new Line3D(oss2.str());
				mLine3D.push_back(line);//mapping
				line->setLine(childNode->getPosition(), Vector3(0,0,0));
				node->attachObject(line);
			}
			if (jts[i]->hasChildren())
			{
				initModels(jts[i]->getChildren(), childNode, modelNum, snNames);//Recursivity
			}
		}
	}
}

void S3DViewer::defineMaterials()
{
	ResourceGroupManager::getSingleton().createResourceGroup("axis");
	MaterialPtr myManualObjectMaterial = MaterialManager::getSingleton().create("Red","axis", true);
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
	
	ResourceGroupManager::getSingleton().createResourceGroup("bone");
	myManualObjectMaterial = MaterialManager::getSingleton().create("Purple","bone"); 
	myManualObjectMaterial->setReceiveShadows(false); 
	myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(1,0,1,0); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(1,0,1); 
	myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1,0,1);
}

ManualObject* S3DViewer::createAxis(const std::string& strName, float scale)
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

