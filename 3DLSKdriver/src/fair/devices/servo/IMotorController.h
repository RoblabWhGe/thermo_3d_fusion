/**
 * @brief Interface for MotorController
 * 
 * 
 * 
 * @author 
 * @date April 2007
 */
#ifndef IMOTORCONTROLLER_H_

#define IMOTORCONTROLLER_H_

#include "../../../fair/core/io/IDeviceAdapter.h"

namespace fair
{

	/**
	 * @class IMotorController
	 * @brief An Interface for motor controllers.
	 *
	 * This interface describes all functionality a motor controller has to
	 * implement. This interface inherits the IDevice marker interface, so all
	 * motor controller are devices.
	 *
	 * Every motor controller connects the device through an IDeviceAdapter so an
	 * implementation must provide a constructor (IMotorController(IDeviceAdapter* pAdapter)
	 * and methods (setDeviceAdapter(), getDeviceAdapter()) to manipulate the
	 * adapter.
	 *
	 * Every implementation must implement the methods connect() and disconnect() to
	 * establish or close a connection to the motor controller through the previously set
	 * IDeviceAdapter. This methods must be robust against repeated calls if there
	 * is already a connection established (or already closed), and against calls
	 * when no IDeviceAdapter has been set.
	 *
	 * An implementation must also provide several methods that allow an user to get
	 * information about the construction and configuration of the motorcontroller.
	 *
	 * Each motorcontroller can be described by some properties:
	 *
	 * [to do]
	 *
	 * This leads us to the following methods, which must be provided by every
	 * implementation:
	 * - getPosition() returns an element of float, which represents the angle in radiants.
	 * - getRateVelocity() returns an element of int, which represents the rate of convertion
	 *										 (e.g. 89 for 1:89)
	 * - setVelocity(float fVelocity)
	 * - start() returns an element of boolean, true if the motor starts to move
	 * - stop()  returns an element of boolean, true if the motor stops
	 *
	 * @author
	 * @date April 2007
	 */
	class IMotorController
	{
		public:
			/**
			 * Standard constructor. An implementation of this constructor must do
			 * the same as the default constructor IMotorController(). In addition it must
			 * set the IDeviceAdapter pAdapter in the same way a call of the method
			 * setDeviceAdapter() with the same argument would do. So this means a
			 * call of this constructor must be equivalent to a call of the default
			 * constructor IScanner() followed by a call of setDeviceAdapter().
			 * @param pAdapter Deviceadapter.
			 * @post The object must be in the same state as if IScanner() and
			 * setDeviceAdapter(pAdapter) would have been called instead.
			 */
			IMotorController(IDeviceAdapter *pAdapter) {
			}
			;

			/**
			 * Default constructor. This must initialize the object and must do the
			 * same as the standard constructor IScanner(IDeviceAdapter* pAdapter)
			 * except setting the device adapter.
			 * @post The object must be initialized
			 */
			IMotorController() {
			}
			;

			/**
			 * Default destructor. This should clean up.
			 * @post The object must be cleaned up.
			 */
			virtual ~IMotorController() {
			}
			;

			/**
			 * Get the current deviceadapter. An implementation must return a
			 * pointer to the current deviceadapter. If no deviceadapter has been
			 * set NULL must be returned.
			 * @return Pointer to the adapter.
			 */
			virtual IDeviceAdapter *getDeviceAdapter() = 0;

			/**
			 * Set a new deviceadapter. An implementation must set the deviceadapter
			 * to the adapter pAdapter points to. If pAdapter is NULL the adapter
			 * has to be unset like it never has been set before.
			 *
			 * If the scanner is already connected (by a successfully call of
			 * connect()) when calling this method it must be disconnected before
			 * changing the deviceadapter.
			 * @param pAdapter Pointer to the adapter.
			 * @post The deviceadapter is set to pAdaper or unset if pAdaper is
			 * NULL.
			 */
			virtual void setDeviceAdapter(IDeviceAdapter *pAdapter) = 0;

			/**
			 * Connect the motor controller through the deviceadapter. An implementation must
			 * open a connection to the motorcontroller through the deviceadapter set by the
			 * constructor IMotorController(IDeviceAdapter* pAdapter) or by
			 * setDeviceAdapter(IDeviceAdapter* pAdapter). If no adapter has been
			 * set or an error occurs 0 must be returned.
			 * @return state of succession (success != 0)
			 * @pre Deviceadapter must not be NULL and the scanner must not be
			 * connected already.
			 * @post The laserscanner must be connected or 0 must be returned.
			 */
			virtual int connect() = 0;

			/**
			 * Disconnect the scanner. An implementation has to close the connection
			 * to the scanner. If an error occures 0 must be returned.
			 * @return state of succession (success != 0)
			 * @pre The scanner must be connected.
			 * @post The laserscanner must be disconnected or 0 must be returned.
			 */
			virtual int disconnect() = 0;

			/**
			 * Returns an element of float, which represents the angle in radiants.
			 * If an error occures -1 must be returned.
			 * @return angle position in radiants
			 */
			virtual double getPosition() =0;

			/**
			 * Returns an element of int, which represents the rate of convertion
			 * (e.g. 89 for 1:89)
			 * If an error occures -1 must be returned.
			 * @return the rate of convertion
			 */
			virtual int getRateVelocity() =0;

			/**
			 * Set the velocity of the motor
			 * @param the desired velocity
			 */
			virtual void setVelocity(float fVelocity) =0;

			virtual double getRotationPeriod() =0;

			/**
			 * Get the velocity of the motor
			 *
			 */
			virtual float getVelocity() =0;

		private:

	};

}

#endif /*ISCANNER_H_*/
