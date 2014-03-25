/*
*
* libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
* 
* Ownership Fraunhofer Gesellschaft e. V., Munich, Germany
*
* The FAIR library [both binary and source code (if released)] is intellectual
* property owned by Fraunhofer Gesellschaft and is protected by the Creative
* Commons license by-nc-sa (http://creativecommons.org/licenses/by-nc-sa/3.0/de/).
* The ownership remains with Fraunhofer Gesellschaft.
*/

#ifndef ISCANNER_H_
#define ISCANNER_H_

#include "../../../fair/devices/factory/IDevice.h"
#include "../../../fair/core/base/CCartesianCloud.h"
#include "../../../fair/core/io/IDeviceAdapter.h"

namespace fair
{

/**
 * @brief Unit of range data.
 * 
 * This enum represents the unit of range data. eUnitMM means a measurement unit
 * of milimetres and eUnitCM means a unit of centimetres.
 * 
 * @author Alexander Grothkast
 * @date July 2006
 */
enum EnumUnit {
	/**
	 * Range values are given in multiples of milimetres.
	 */
	eUnitMM = 0,
	/**
	 * Range values are given in multiples of centimetres.
	 */
	eUnitCM = 1
};

/**
 * @brief Scanning direction (top view).
 * 
 * This enum represents a rotation direction from a top point of view.
 * eCLOCKWISE means a clockwise rotation and eCOUNTERCLOCKWISE a
 * counterclockwise one.
 * 
 * @author Alexander Grothkast
 * @date July 2006
 */
enum EnumScanDirection {
	/**
	 * Laserscan takes place in a clockwise rotation from a top point of view.
	 */
	eCLOCKWISE,
	/**
	 * Laserscan takes place in a counterclockwise rotation from a top point of
	 * view.
	 */
	eCOUNTERCLOCKWISE
};


/**
 * Sick scanner resoltion
 */
enum EnumResolution { eRes1Degree = 1, eRes05Degree = 2, eRes025Degree = 4 };

/**
 * Apex angle of sick scanner
 */
enum EnumApexAngle { eApex180Degree = 180, eApex100Degree = 100};

/**
 * Operation modes
 */
enum EnumOperationMode { eOpConfiguration, eOpOperating };

/**
 * Transmission mode
 */
enum EnumTransmissionMode { eTransRequestSending = 0, eTransContinuousSending, eTransInterlaced, eTransRemission };


/**
 * @class IScanner
 * @brief An Interface for laser scanners.
 * 
 * This interface describes all functionality a laserscanner has to
 * implement. This interface inherits the IDevice marker interface, so all
 * laser scanners are devices.
 * 
 * Every laserscanner connects the device through an IDeviceAdapter so an
 * implementation must provide a constructor (IScanner(IDeviceAdapter* pAdapter)
 * and methods (setDeviceAdapter(), getDeviceAdapter()) to manipulate the
 * adapter.
 * 
 * Every implementation must implement the methods connect() and disconnect() to
 * establish or close a connection to the scanner throug the previously set
 * IDeviceAdapter. This methods must be robust against repeated calls if there
 * is already a connection established (or already closed), and against calls
 * when no IDeviceAdapter has been set.
 * 
 * An implementation must also provide several methods that allow an user to get
 * information about the construction and configuration of the laserscanner.
 * Each laserscanner can be described by some properties:
 * 
 * A laserscanner is able to scan the enivronment within a apex angle (e.g. 240
 * degree) with a resolution (e.g. 0.25 degree/step). Objects outside the apex
 * angle cannot be detected. The apex angle and resolution together describe how
 * many scan positions are taken during one whole laserscan. The scan takes
 * place either in a clockwise or counterclockwise rotation. Clockwise means
 * here, that (from a top point of view) each scan position is clockwise
 * oriented to all previous positions during that laserscan. All scan positions
 * (steps) can be numbered in ascending order.
 * So the laserscanner's resolution together with the minimal and maximal
 * allowed scan positions describe the maximal apex angle possible. So this
 * means also that every connected scan area part can be described by the start
 * and end position's numbers.
 * 
 * Beside that information concerning the measurement unit of the range data and
 * the time needed to perform one laserscan are also useful.
 * 
 * This leads us to the following methods, which must be provided by every
 * implementation:
 * - getUnit() returns an element of EnumUnit. All range values returned by any
 * methods of the class must be given in multiples of that unit.
 * - getScannerApexAngle() must return the current apex angle of the scanner.
 * - getScannerResolution() must return the number of steps the laserscanner
 * would do in a full circle.
 * - getDirection() returns an element of EnumScanDirection and must be that
 * direction (from a top point of view), in which the laserscan takes place.
 * - getScanningTime() must return a time in miliseconds.microseconds. That time
 * must be at least the time one laserscan lasts. It must not be smaller than
 * the duration of one laserscan.
 * - getMaxRange() must return the number of scan positions (steps) maximal
 * allowed. It must set the minimal start and maximal end position allowed by
 * the scanner.
 * All this values may change during lifetime, e.g. when the scanner offers
 * different operation modes and the mode is changed.
 * 
 * The implementation of the methods getRange() and setRange() must allow an
 * user to set the exact range of a laserscan within the bounds given by
 * getMaxRange(). The method getValuesPerScan() must return the number of range
 * values someone has to expect when calling getScan(). It must not return a
 * number smaller than the number of values returned by getScan().
 * 
 * The implementation of the getScan() methods must request a laserscan from the
 * scanner and return either a bunch of range values or a CartesianCloud3D
 * filled with 3D-points.
 * 
 * @author Alexander Grothkast, Stefan May, Marco Hartich
 * @date July 2006
 */
class IScanner : public IDevice {
	public:
		/**
		 * Standard constructor. An implementation of this constructor must do
		 * the same as the default constructor IScanner(). In addition it must
		 * set the IDeviceAdapter pAdapter in the same way a call of the method
		 * setDeviceAdapter() with the same argument would do. So this means a
		 * call of this constructor must be equivalent to a call of the default
		 * constructor IScanner() followed by a call of setDeviceAdapter().
		 * @param pAdapter Deviceadapter.
		 * @post The object must be in the same state as if IScanner() and
		 * setDeviceAdapter(pAdapter) would have been called instead.
		 */
		IScanner(IDeviceAdapter *pAdapter) {};
		
		/**
		 * Default constructor. This must initialize the object and must do the
		 * same as the standard constructor IScanner(IDeviceAdapter* pAdapter)
		 * except setting the device adapter.
		 * @post The object must be initialized
		 */
		IScanner() {};
		
		/**
		 * Default destructor. This should clean up.
		 * @post The object must be cleaned up.
		 */
		virtual ~IScanner() {};

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
         * Get time for taking one scan line. An implementation must return a
         * time in the format miliseconds.microseconds. This value must be at
         * least the time the laserscanner needs to take one scan. It must not
         * be smaller.
         * 
         * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
         * @return Time in miliseconds.mircoseconds.
         */
		virtual double getScanningTime() = 0;

		/**
		 * Get unit of range data. An implementation must return an element of
		 * EnumUnit depending on the measurement unit the range values are
		 * returned. That means all range values returned by any methods of the
		 * class must be given in multiples of that unit.
		 * 
		 * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
		 * @returns Unit
		 */
		virtual EnumUnit getUnit() = 0;
		
		/**
		 * Get current Apex Angle of scanner. An implementation must return the
		 * current apex angle of the laserscanner in degree.
		 * 
		 * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
		 * @return ApexAngle in degree
		 */
		virtual int getScannerApexAngle() = 0;
		
		/**
		 * Get resoultion of scanner in steps per full circle. An implementation
		 * must return the current resolution of the laserscanner.
		 * The resolution must be given in the number of steps (scan positions)
		 * the laserscanner would take during a 360 degree full circle scan.
		 * That means that the actual resolution in degree per step equals 360
		 * divided by the return value.
		 * 
		 * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
		 * @return Number of steps per full circle.
		 */
		virtual int getScannerResolution() = 0;
		
		/**
		 * Get scanning direction. An implementation must return a value of
		 * EnumScanDirection depending on the rotation direction in which the
		 * laserscan takes place. From a top point of view this could be in a
		 * clockwise or a counterclockwise direction.
		 * 
		 * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
		 * @returns Direction from top view.
		 */
		virtual EnumScanDirection getDirection() = 0;

		/**
		 * Get the scan range which is maximal allowed. An implementation must
		 * set pnStart to the minimal start and pnStop to the maximal end scan
		 * position allowed by the laserscanner. It must return the number of
		 * scan positions between the minimal start and maximal end positions
		 * (including the start and end position).
		 * 
		 * The return value of this method may change during lifetime, e.g. when
         * the scanner offers different operation modes and the mode is changed.
		 * @param pnStart Value is set to the maximal start position.
		 * @param pnStop Value is set to the maximal stop position.
		 * @return Number of scan positions between start and stop.
		 */
		virtual int getMaxRange(int *pnStart, int *pnStop) = 0;

		/**
		 * Get the current scan range. An implementation must set pnStart to the
		 * current start and pnStop to the current end scan position. It must
		 * return the number of scan positions between the start and end
		 * positions (including the start and end position).
		 * 
		 * That means that the return value must equal the return value of a
		 * call of getValuesPerScan() at the same time.
		 * @param pnStart Value is set to the current start position.
		 * @param pnStop Value is set to the current stop position.
		 * @return Number of scan positions between start and stop.
		 */
		virtual int getRange(int *pnStart, int *pnStop) = 0;

		/**
		 * Set a new scan range. An implementation must set the new start scan
		 * position to nStart and the new end scan position to nStop if those
		 * are valid values. These positions must be used for all future
		 * laserscans (using the getScan() method) until they are changed again
		 * with this method.
		 * 
		 * The method must return the number of scan positions between the new
		 * start and end positions (including the start and end position) if
		 * they are changed. If the start and stop positions are not changed
		 * because an error occured (e.g. the provided positions are out of the
		 * allowed range) 0 must be returned.
		 * @param nStart Value of the new start position.
		 * @param nStop Value of the new stop position.
		 * @return Number of steps between the new start and end position.
		 * @post The start and end scan positions must be set to nStart and
		 * nStop and must be used in future laserscans or 0 must be returned.
		 */
		virtual int setRange(int nStart, int nStop) = 0;

		/**
		 * Returns the number of values in a scan. An implementation must return
		 * the number of scan positions between the start and end positions 
		 * (including the start and end position).
		 * 
		 * That means that the return value must equal the return value of a
		 * call of getRange() at the same time.
		 * @return Number of values per one scan.
		 */
		virtual int getValuesPerScan() = 0;

        /**
         * Connect the scanner through the deviceadapter. An implementation must
         * open a connection to the scanner through the deviceadapter set by the
         * constructor IScanner(IDeviceAdapter* pAdapter) or by
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
		 * Get one scan from the laserscanner. An implementation must do a
		 * laserscan starting at the given start point and ending at the given
		 * end point (see setRange() method).
		 * It must write the range values sequentially into pdPoints. It must
		 * return the number of range values written. If an error occurs and no
		 * data are written 0 must be returned.
		 * 
		 * Information returned by getUnit(), getScannerApexAngle(),
		 * getScannerResolution(), getDirection() and getRange() at the time of
		 * the laserscan must be correct.
		 * @param pdPoints The results will be stored in an array of doubles.
		 * @return The number of points scanned or 0 on failure.
		 * @pre pdPoints must be an array of double values which can hold at
		 * least getValuesPerScan() range values.
		 * @post Must return the number of range values written to pdPoints.
		 */
		virtual unsigned long getScan(double *pdPoints) = 0;		

		/**
		 * Get one scan from the laserscanner. An implementation must do a
		 * laserscan starting at the given start point and ending at the given
		 * ending point (see setRange() method).
		 * 
		 * It must transform the range data a 2D coordinate system with a
		 * x-axis and a z-axis. The point of origin must be the focal point of
		 * the scanner. The z-axis must be oriented to the front of the scanner.
		 * The x-axis must be oriented 90 degree clockwise to the z-axis. As the
		 * points are described in a 3D coordinate system the y-value of all
		 * points must be set to the same value.
		 * 
		 * It must return the number of range values written. If an error occurs
		 * and no data are written 0 must be returned.
		 * @param pCloud The results will be stored in this cloud.
		 * @return The number of points scanned or 0 on failure.
		 * @pre pCloud must be a CCartesianCloud3D in which enough points are
		 * initialized to hold at least getValuesPerScan() range points.
		 * @post Must return the number of scan points set in pCloud.
		 */
		//virtual unsigned long getScan(CCartesianCloud3D *pCloud) = 0;

		/**
         * Get one scan from the laserscenner. Note that this method is a blocking method.
         * For non-blocking access use getScan(double* pdScan, double* pdRemission, unsigned int* pnScan)
         * @param pdScan buffer to be filled with distance values
         * @param pdRemission buffer to be filled with remission values (only in remission mode!)
         * @return number of scan points returned
         */
		virtual unsigned long getScan(double *pdScan, double *pdRemission) = 0; 

	private:
	
};

}

#endif /*ISCANNER_H_*/
