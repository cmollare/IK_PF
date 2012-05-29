#ifndef S3DVIEWER_H
#define S3DVIEWER_H

/*!
 * \file S3DViewer.h
 * 
 */

#include <Ogre.h>
#include <vector>
#include <sstream>
#include "InputListener.h"
#include "Line3D.h"
#include "../3DModel/S3DModel.h"

using namespace Ogre;

/*!
 * \class S3DViewer
 * \brief class to display the model and several extra informations
 * 
 * This class display the Joint, and the information displayed can be personnalised with the setOptions function.
 * The possibility to send command during the execution will be added.
 * 
 */

class S3DViewer
{
public:

	/*!
	 * \brief S3DViewer constructor
	 */
    S3DViewer();
    
    /*!
     * \brief S3DViewer destructor
     */
    ~S3DViewer();
    
    /*!
     * \brief Setup the FrameListener
     * 
     * The frameListener allow us to catch keyboard and mouse events.
     * 
     */
	void createFrameListener();
	
	/*!
	 * \brief Setup the Ogre3D environment
	 */
	bool init();
	
	/*!
	 * \brief Start the infinite loop for realTime rendering
	 */
    bool start();
    
    bool isRendering();
    
    /*!
     * \brief Set the viewer options
     * 
     * \param displayJoint If true, display a sphere at the Joint position.
     * \param displayAxis If true, display the local axis on the Joint position.
     * \param displayBone If true, display a bone between the Joint and its parent.
     */
    void setOptions(bool displayJoint=true, bool displayAxis=true, bool displayBone=true);
    
    /*!
     * \brief Initiate the model following Ogre3D hierarchy.
     * 
     * This function has to be called before the start function.
     * 
     * \param models A vector of pointers on previously created S3Model.
     */
    void initModels(std::vector<S3DModel*>& models);
    
    void initObservations(std::vector<std::string> jtNames, std::vector<std::vector<double> > frame);
    
    void update(std::vector<S3DModel*>& models, std::vector<std::vector<double> >& frame);
    
    //debug functions
    /*!
     * \brief Display a quaternion in Angle-Axis space
     * 
     * [debug function]
     * This function allow us to display a quaternion in Angle-Axis space
     * 
     * \param quat Quaternion to be displayed.
     */
    void displaySampling(Eigen::Quaternionf quat);//ajouter la rotation
    
    //Drawing functions
    /*!
     * \brief define Materials used by Ogre::ManualObject
     */
    void defineMaterials();
    
    /*!
     * \brief create an Axis
     * 
     * [Will be replaced by a class].
     * 
     * \param strName Name of the axis.
     * \param scale Scale of the axis : minimum 1.
     */
    ManualObject* createAxis(const std::string& strName, float scale=0.05);

private:

	/*!
	 * \brief initModels recursively
	 * 
	 * This function is called by initModels public function.
	 * 
	 * \param jts Children Joints.
	 * \param node Parent Ogre::SceneNode.
	 * \param modelNum Index of the S3DModel.
	 */
	void initModels(std::vector<Joint*>& jts, SceneNode *node, int modelNum, std::map<std::string, std::string>& snNames);
	void updateLine3D();
	void updateObs(std::vector<std::vector<double> >& frame);
	
    Ogre::Root *mRoot; /*!< Ogre::Root */
    Ogre::RenderWindow* mWindow; /*!< Ogre::RenderWindow */
	Ogre::SceneManager* mSceneMgr; /*!< Ogre::SceneManager */
	Ogre::Camera* mCamera; /*!< Ogre::Camera */
	InputListener *mInputListener; /*!< InputListener for event handling */
	Ogre::LogManager* mLogMgr; /*!< For Ogre.log file management */
	
	//Mapping
	std::map<Line3D*, std::string> mLine3DToSNName; /*!< Mapping from Line3D* to associated SceneNode name */
	std::map<Line3D*, Joint*> mLine3DToJoint; /*!< Mapping from Line3D* to associated Joint* */
	std::vector<std::map<std::string, std::string> > mModelSNNames; /*!< Mapping from S3DModel* to associated SceneNode name */
	std::vector<std::string> mObservationSNNames; /*!< List of the observations SceneNode Names */
	//End mapping
	
	//For observations
	std::map<std::string, int> mObsMap;
	std::vector<std::string> mObsNameVec;
	std::vector<std::vector<double> > mObsCurrentFrame;
	
	//Display options
	bool mDisplayJoint; /*!< set/clear in setOptions function */
	bool mDisplayBone; /*!< set/clear in setOptions function */
	bool mDisplayAxis; /*!< set/clear in setOptions function */
	//End display options

};

#endif
