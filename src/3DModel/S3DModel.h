#ifndef S3DMODEL_H
#define S3DMODEL_H

/*!
 * \file S3DModel.h
 * 
 */

#include <iostream>
#include <map>
#include <Eigen/Dense>
#include "Joint.h"

/*!
 * \class S3DModel
 * \brief class representing the 3D model
 * 
 * The class S3DModel contains the Joint hierarchy and can be used as an interface to work on the model.
 * This class also map some informations for efficiency.
 */

class S3DModel
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW	//For Eigen3
		
		/*!
		 * \brief S3DModel constructor
		 * 
		 * This constructor create a single empty Joint.
		 * 
		 * \param id Unique id of the model.
		 */
		S3DModel(int id);
		
		/*!
		 * \brief S3DModel constructor
		 * 
		 * This constructor takes a complete hierarchy as paramater, and map the model.
		 * 
		 * \param jt Pointer on the model root Joint.
		 * \param id Unique id of the model.
		 */
		S3DModel(const Joint* jt, unsigned int id=0);
		
		/*!
		 * \brief S3DModel copy constructor
		 * 
		 * \param model S3DModel to be copied.
		 */
		S3DModel(const S3DModel& model);
		
		/*!
		 * \brief S3DModel destructor
		 * 
		 * Delete all the Joint, you don't have to do it yourself.
		 * 
		 */
		~S3DModel();
		
		/*!
		 * \brief Get the root Joint
		 * \return Return a pointer on the model root Joint.
		 */
		Joint* getRootJoint();
		
		/*!
		 * \brief Map the model
		 */
		void createMaps();
		
		/*!
		 * \brief Get the number of Joints
		 * \return Return the number of Joints int the model
		 */
		int getNumberJoint();
		
		Joint* getJoint(std::string);
		
		/*!
		 * \brief Get the orientation vector
		 * 
		 * Return the local orientation of each Joints in a vector.
		 * Warning : If you modify the orientation, it will be modified in the corresponding Joint.
		 * 
		 * \return A vector of Eigen::quaternionf
		 */
		vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > getOrientationVec();
		
		vector<Eigen::Translation3f*, Eigen::aligned_allocator<Eigen::Translation3f*> > getOffsetVector();
		
		vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf> > getDefaultOrientationVec();
		
		vector<Eigen::Translation3f, Eigen::aligned_allocator<Eigen::Translation3f> > getDefaultOffsetVector();
		
		/*!
		 * \fn vector<std::string> getNameVec()
		 * \brief Return the name of each Joints in a vector
		 * \return vector of names
		 */
		vector<std::string> getNameVec();
		
		vector<std::string> getConstOffsetVec();
		vector<std::string> getConstOrientVec();
		
		void setPrincipal(bool isPrincipal=true);
		
		void debug()//fonction temporaire
		{
			for (int i=0 ; i<mOrientationVec.size() ; i++)
			{
				cout << "****************" << endl;
				cout << Eigen::Matrix3f(*mOrientationVec[i]) << endl;
				cout << "****************" << endl;
			}
		}
		
	private:
	
		/*!
		 * \brief map recursively
		 * 
		 * For recursivity of createMaps public function.
		 * 
		 * \param jts Vector of pointer on child Joints.
		 */
		void createMaps(vector<Joint*>& jts);
		
		/*!
		 * \fn void createOrientationVec()
		 * \brief create the local orientation vector of the Joints.
		 * 
		 * This function is used in the S3DModel constructor.
		 * 
		 */
		void createOrientationVec();
		
		/*!
		 * \fn void createOffsetVec()
		 * \brief create the local offset vector of the Joints.
		 * 
		 * This function is used in the S3DModel constructor.
		 * 
		 */
		void createOffsetVec();
		
		/*!
		 * \fn void createNameVec()
		 * \brief Create a vector of Joint names
		 */
		void createNameVec();
		
		void createConstraintVecs();
		void createDefaultVecs();
		
		unsigned int mId; /*!< id of the model */
		Joint *mRootJoint; /*!< pointer on the root Joint */
		bool mIsPrincipal;
		
		std::map<std::string, Joint*> mStringToJoint; /*!< mapping from Joints name to Joint pointers (for speed) */
		std::map<std::string, int> mStringToInt; /*!< mapping from Joints name to Joint indexes (for speed) */
		std::map<int, Joint*> mIntToJoint; /*!< mapping from Joints index to Joint pointers (for speed) */
		int mNbJoints; /*!< total number of Joints */
		
		vector<Eigen::Quaternionf*, Eigen::aligned_allocator<Eigen::Quaternionf*> > mOrientationVec; /*!< Orientation vector */
		vector<Eigen::Translation3f*, Eigen::aligned_allocator<Eigen::Translation3f*> > mOffsetVec;
		vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf> > mDefaultOrientationVec;
		vector<Eigen::Translation3f, Eigen::aligned_allocator<Eigen::Translation3f> > mDefaultOffsetVec;
		vector<std::string> mNameVec; /*!< Vector of Joint names */
		vector<std::string> mConstOffsetVec;
		vector<std::string> mConstOrientVec;
};

#endif
