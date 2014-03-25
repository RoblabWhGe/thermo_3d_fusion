#ifndef CNULLPOSEADAPTER_H__
#define CNULLPOSEADAPTER_H__

#include <iostream>
#include <stdio.h>
#include "../../../fair/core/math/CMatrix.h"
#include "../../../fair/core/base/CTimer.h"
#include "../../../fair/core/io/ISystemPoseAdapter.h"

using namespace std;

namespace fair
{

/**
 * @class CNullPoseAdapter
 * @brief return an Identity Matrix
 * @author 
 **/


class CNullPoseAdapter : public ISystemPoseAdapter {
	public:
		/**
		 * Standard constructor
		 */
		CNullPoseAdapter();
		/**
		 * Default destructor
		 */
		~CNullPoseAdapter();
              
		/*
		 * return the pos-Matrix
		 * Note: This method is provided to implement ISystemPoseAdapter.
		 */
		CMatrix44 getPos();


		/*
		 * return the Adapter type
		 * Note: This method is provided to implement ISystemPoseAdapter.
		 */
		EnumPoseAdapterType getAdapterType();


	private:
		EnumPoseAdapterType _type;
		CMatrix44 _poseMatrix; 
};

}

#endif /*_CNULLPOSEADAPTOR_H_*/
