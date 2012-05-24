#ifndef JOINT_H
#define JOINT_H

/*!
 * \file Joint.h
 * 
 */

#include <cstdlib> 
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;//A supprimer

//Constraints on orientations
#define ORIENT_CONST_FREE "Free"
#define ORIENT_CONST_TWIST "Twist"
#define ORIENT_CONST_FLEX "Flex"
#define ORIENT_CONST_TFLEX "TFlex"
#define ORIENT_CONST_BIFLEX "BiFlex"
#define ORIENT_CONST_FIXED "Fixed"

//Constraints on offsets
#define OFFSET_CONST_FREE "Free"
#define OFFSET_CONST_BONE "Bone"
#define OFFSET_CONST_FIXED "Fixed"
#define OFFSET_CONST_PLANARXY "PlanarXY"
#define OFFSET_CONST_PLANARYZ "PlanarYZ"
#define OFFSET_CONST_PLANARXZ "PlanarXZ"

//Signs constraints
#define CONST_POSITIVE "Pos"
#define CONST_NEGATIVE "Neg"
#define CONST_NONE "NULL"


/*!
 * \class Joint
 * \brief class representing model joints
 * 
 * Joints are created following a hierarchical structure. The root Joint has a NULL parent.
 * Each Joint has a UNIQUE name. The root Joint has to be passed to the S3DModel class constructor.
 */

class Joint
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		/*!
		 * \brief Joint constructor
		 * 
		 * Constructor of the Joint class.
		 * 
		 * \param name Name of the joint.
		 * \param parent Pointer to the parent of the joint, the root joint has a NULL parent.
		 * \param offset Position relative to the parent joint.
		 * \param quat Local orientation of the joint.
		 */
		Joint(string name, Joint *parent=NULL, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0));
		
		/*!
		 * \brief Joint copy constructor
		 * 
		 * Copy constructor of the Joint class.
		 * 
		 * \param jtCopy Joint to be copied.
		 */
		Joint(const Joint& jtCopy);
		
		/*!
		 * \brief Joint destructor
		 * 
		 * Destructor of the Joint class. All joints under the root one are deleted.
		 * Be carefull to not delete them elsewhere, or use them after !
		 */
		~Joint();
		
		//void setDof(vector<bool> dof);
		/*!
		 * \brief Gets the root Joint pointer
		 * 
		 * This function gets the root Joint pointer
		 * 
		 * \return pointer on the root Joint
		 */
		Joint *getRoot();
		void setParentIfChild(Joint *jt);
		
		/*!
		 * \brief Gets the parent pointer of the Joint
		 * 
		 * This function return a pointer on the parent Joint of this Joint
		 * 
		 * \return pointer on the parent Joint
		 */
		Joint *getParent();
		
		/*!
		 * \brief Gets the pointer on the Joint with given name
		 * 
		 * This function finds the pointer on the Joint with the name taken in parameter.
		 * If the Joint is not found, the function return a NULL pointer.
		 * 
		 * \param name Name of the searched Joint.
		 * \return Pointer on Joint, NULL if not found.
		 */
		Joint *getJointFromName(std::string name);
		
		/*!
		 * \brief Gets the pointer on the Joint with given name
		 * 
		 * This function finds the pointer on the Joint with the name taken in parameter.
		 * If the Joint is not found, the function return a NULL pointer.
		 * The search is a descending search in the hierarchy.
		 * 
		 * \param jt Pointer on the Joint to start from.
		 * \param name Name of the searched Joint.
		 * \return Pointer on Joint, NULL if not found.
		 */
		Joint *getJointFromName(Joint *jt, std::string name);
		
		/*!
		 * \brief Return the name of the Joint
		 * \return std::string name
		 */
		std::string getName();
		
		/*!
		 * \brief Test for children
		 * \return True if the Joint has children, false otherwise.
		 */
		bool hasChildren();
		
		/*!
		 * \brief Gets Joint children
		 * \return A vector on pointer on children.
		 */
		std::vector<Joint*>& getChildren();
		
		/*!
		 * \brief Add a child to this Joint
		 * 
		 * Allocate a new Joint, calling the constructor.
		 * This Joint will be automatically deleted during the destructor call.
		 * 
		 * \param name Name of the child.
		 * \param offset Relative position to the parent.
		 * \param quat Local orientation of the Joint.
		 * \return Pointer on the new created Joint.
		 */
		Joint* addChild(std::string name, vector<double> offset=vector<double>(3,0), Eigen::Quaterniond quat=Eigen::Quaterniond(1,0,0,0));
		
		/*!
		 * \brief Add a child to this Joint
		 * 
		 * Add a new child to this Joint.
		 * The new child will be automatically deleted during the destructor call.
		 * 
		 * \param jt Pointer on the Joint to add. It has to be dynamically allocated.
		 * \return Pointer on the new added Joint.
		 */
		Joint* addChild(Joint* jt);
		
		/*!
		 * \brief Set the Joint new local orientation
		 * \param quat Joint local orientation.
		 */
		void setOrientation(const Eigen::Quaterniond quat);
		
		Joint* setConstraints(const std::string offset=OFFSET_CONST_FREE, const std::string orientation=ORIENT_CONST_FREE);
		
		Joint* setLimits(const std::vector<std::string>& signConst);
		
		bool checkValidity(const Eigen::Vector3d& offset);
		
		/*!
		 * \brief Get the Joint local orientation.
		 * \return Quaternion of Joint local orientation.
		 */
		Eigen::Quaterniond* getOrientation();
		
		const Eigen::Quaterniond getDefaultOrientation();
		
		/*!
		 * \brief Get relative position from parent
		 * \return Translation vector
		 */
		Eigen::Translation3d* getOffset();
		
		const Eigen::Translation3d getDefaultOffset();
		
		/*!
		 * \brief Get world position
		 * 
		 * Get the Joint world position using forward kinematic.
		 * 
		 * \return A vector of (X,Y,Z) position.
		 */
		const Eigen::Vector3d getXYZVect();
		
		/*!
		 * \brief Get the transformation Matrix
		 * 
		 * This function retrieve the transformation Matrix of the Joint.
		 * 
		 * \return Return the 4x4 transformation Matrix.
		 */
		Eigen::Transform<double, 3, Eigen::Affine> getTransformationMatrix();
		
		Eigen::Transform<double, 3, Eigen::Affine> getGlobalTransformationMatrix();
		
		std::string getOffsetConstraint();
		std::string getOrientationConstraint();
		void setPrincipal(bool isPrincipal);
		std::vector<float> getColor();
		
	private:
	
		std::string mName; /*!< Joint unique name */
		Joint *mParentJoint; /*!< parent Joint, NULL if root */
		std::vector<Joint*> mChildrenJoint; /*!< vector of child Joints */
		std::vector<float> mColors; /*!< Color of the Joint */
		bool mIsPrincipal;
		
		Eigen::Quaterniond mQDefault;
		Eigen::Quaterniond mQLocal; /*!< Joint local orientation */
		//Eigen::Matrix2f mQCurrent;
		Eigen::Translation3d mDefaultOffset;
		Eigen::Translation3d mLocalOffset;/*!< Joint relative position to parent */
		//float mCurrentOffset;
		
		std::string mOffsetConst; /*!< Constraint on offset */
		std::vector<std::string> mOffsetSignConst; /*!< Sign constraints */
		std::string mOrientationConst; /*!< Constraint on orientation */
};

#endif
