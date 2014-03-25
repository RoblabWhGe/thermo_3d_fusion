#include "CNullPoseAdapter.h"

//using namespace std;
namespace fair{

CNullPoseAdapter::CNullPoseAdapter()
{
	_type = eNullPose;
	_poseMatrix=IdentityMatrix44();
}

CNullPoseAdapter::~CNullPoseAdapter()
{
	//delete _pPoseMatrix;
}

CMatrix44 CNullPoseAdapter::getPos(){
	return _poseMatrix;

}


EnumPoseAdapterType CNullPoseAdapter::getAdapterType(){
	return eNullPose;
}

}
