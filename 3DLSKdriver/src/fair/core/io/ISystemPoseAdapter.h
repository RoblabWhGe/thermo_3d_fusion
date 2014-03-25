#ifndef IPOSESYSTEMADAPTER_H_
#define IPOSESYSTEMADAPTER_H_

#include "../../../fair/core/math/CMatrix.h"

namespace fair
{

/**
 * @brief Represents the adapter type of an IPOSESYSTEMADAPTER_H_
 * 
 * There are three importart type:
 *
 * 1- NullPoseAdapter	   - the system is not moving
 * 2- ConstantSpeedAdapter - the system is moving with a constant speed in the x axe (in front of the car)
 * 3- ApplanixAdapter	   - the system is indipendent and we do not know about the velocity and position
 * 
 * Every implementation of IPoseSystemAdapter should be distinguishable by an enumeration.
 * eUNKNOWN should not be used in any concrete implementation but is
 * reserved for abstract classes.
 * 
 * @author Alexander Grothkast
 * @date July 2006
 */
enum EnumPoseAdapterType {
	/**
	 * An unknown adapter type. Reserved for anstract calsses.
	 */
	//eUNKNOWN = 1,
	/**
	 * Implements a NullPoseadApter. Used by CNullPoseAdapter.
	 */
	eNullPose = 2,
	/**
	 * Implements a ConstantSpeedAdapter. Used by CConstantSpeedAdapter.
	 */
	eConstantSpeed = 3,
	/**
	 * Implements a ApplanixAdapter. Used by CApplanixAdapter.
	 */
	eApplanixPose = 4
};

/**
 * @class IPoseSystemAdapter
 * @brief Interface for communication about the position and orientation of the system
 * 
 * The only public function is getPos.
 * Possible class are:
 * - no change in the position
 * - considering a constant velocity on the x axe
 * - calling an external function to have the position of the system (the car in the Urban Challenge) 
 * 
 * @author 
 * @date April 2007s
 */
class ISystemPoseAdapter {
	public:
		/**
		 * Standard constructor. 
		 */
		ISystemPoseAdapter() {};
		
		
		/**
		 * Default destructor. This should clean up.
		 * @post The object must be cleaned up.
		 */
		virtual ~ISystemPoseAdapter() {};
		
		/**
		 * Get type of adapter. The intention of this method is to allow
		 * someone to distinguish between different implementations.
		 * @return Type of connection as EnumPoseAdapterType
		 */
		virtual EnumPoseAdapterType getAdapterType() = 0;

	    /**
		 * Return a 4x4 matrix, representing the position and orientation of the system 
		 */
		virtual CMatrix44 getPos() = 0;

	
	private:
		

};

}

#endif /*IPOSESYSTEMADAPTER_H_*/
