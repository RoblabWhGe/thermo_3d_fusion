#ifndef CRotating3DScanner_H__
#define CRotating3DScanner_H__

#include "../../../fair/devices/scanner/IScanner.h"
#include "CSickLms100Scanner.h"
#include "../../../fair/devices/servo/IMotorController.h"
#include "../../../fair/core/io/CNullPoseAdapter.h"
#include "../../../fair/core/io/ISystemPoseAdapter.h"

#include "../../../fair/core/base/CTimer.h"
#include "../../../fair/core/math/CVector.h"
#include "../../../fair/core/math/CMatrix.h"
#include "../../../fair/core/io/com/CComAdapter.h"
#include <iostream>
#include <queue>
#include <pthread.h>
#include <sched.h>

#include "../../../fair/devices/servo/CEposPMotorController.h"

namespace fair {

/**
 * @class CRotatingSICKLMS100
 * @brief 3D laserscanner (compositum of two IScanner, one IMotorController and one ISystemPoseAdapter)
 * 
 * This class provides the functionality for a rotating 3D laserscanner which consists of
 * two normal 2D laserscanner, a motorcontroller to set the velocity and to get the position, and a SystemPoseAdapter
 * which provides the position of the entire system, to take in accout the rotation and the translation of it during the
 * different scans.
 *
 * The use of the interface is quite important, to abstract the real classes and to provide some functionalities, 
 * indipendently, for example, from the type of the MotorController.
 * 
 * To perform a 3D laserscan this class uses the
 * IScanner::getScan(double* pdPoints)
 * method to get 2D laserscans and transforms the respective 2D scans in a 3D
 * coordinate system where all 2D scans are merged into one 3D scan, considering both the rotation of the RotatingScanner
 * and the position (rototranslation) of the entire system - for our purpose, the car.
 * 
 * To set the IScanner, IMotorController and ISystemPoseAdapter devices which are used by the 3DScanner,
 * someone can use the methods getScanner1(), getScanner2, setScanner(), getMotorController(), setMotorController(), 
 * getSystemPoseAdapter(), setSystemPoseAdapter()
 * or can use the standard constructor CRotatingSICKLMS100(IScanner* pScanner1, IScanner* pScanner2, double dTilt, double nResolution,
 *					   IMotorController*    pMotorController,
 *					   ISystemPoseAdapter*  pSystemPoseAdapter);
 * 
 * To configure the 3D scanner several methods are provided.
 * 
 * All configurations settings set by the methods described above affect only
 * the 3D laserscanner functionality. That means they influence the way in which
 * the 2D scanner is rotated and how often a 2D scan is taken. To configure the
 * 2D scanner use the appropiate methods of the used 2D laserscanner class.
 * 
 * @author 
 * @date April 2007
 */
class CRotatingSICKLMS100 {
	public:
		/**
		 * Standard constructor
		 *
		 * @param pScanner<n>		 Pointer to the scanner<n>
		 * @param dTilt				 Tilting of the scanners, if the angular adjuster plate is used
		 * @param nResolution		 Resolution of the rotating scanner.
		 * @param pMotorController	 Pointer to the motor controller
		 * @param pSystemPoseAdapter Pointer to the system pose adapter
		 */
		CRotatingSICKLMS100(CSickLms100Scanner* pScanner1, double dTilt, double nResolution, IMotorController* pMotorController,
				ISystemPoseAdapter* pSystemPoseAdapter);

		/**
		 * Default constructor
		 */
		CRotatingSICKLMS100();

		/**
		 * Default destructor
		 */
		~CRotatingSICKLMS100();

		/**
		 * Get the scanner1.
		 * @return Pointer to the scanner1.
		 */
		IScanner *getScanner1();

		/**
		 * Get the motorcontroller.
		 * @return Pointer to the motorcontroller.
		 */
		IMotorController *getMotorController();

		/**
		 * Get the SystemPoseAdapter.
		 * @return Pointer to the SystemPoseAdapter.
		 */
		ISystemPoseAdapter *getSystemPoseAdapter();

		/**
		 * Set two new scanners. If the scanners are already connected the
		 * connection has to be reseted.
		 * @param pScanner<n> Pointer to the scanner<n>.
		 *        dTilt		  Tilting angle of the scanner with angular adjuster plate
		 */
		void setScanner(CSickLms100Scanner *pScanner1, double dTilt);

		/**
		 * Set the motorcontroller.
		 * @param pMotorController Pointer to the motorcontroller.
		 */
		void setMotorController(IMotorController *pMotorController);

		/**
		 * Set the system pose adapter..
		 * @param pScanner Pointer to the SystemPoseAdapter
		 */
		void setSystemPoseAdapter(ISystemPoseAdapter *pSystemPoseAdapter);

		/**
		 * Set timeout.
		 * @param dTimeout Timeout
		 */
		void setTimeout(double dTimeout);

		/**
		 * Get timeput.
		 * @return Timeout
		 */
		double getTimeout();

		/**
		 * Get time for taking one scan line
		 */
		double getScanningTime();

		/**
		 * Get unit of range data.
		 * @returns Unit
		 */
		EnumUnit getUnit();

		/**
		 * Get direction of vertical (servo) rotation
		 * @return direction
		 */
		EnumScanDirection getDirection();

		/**
		 * Set direction of vertical (servo) rotation.
		 * @param direction Direction of servo.
		 */
		void setDirection(EnumScanDirection direction);

		/**
		 * Set vertical apex angle
		 * @param nApexAngle apex angle
		 */
		void setScannerApexAngle(int nApexAngle);

		/**
		 * Get vertical apex angle.
		 * @return vertical apex angle
		 */
		int getScannerApexAngle();

		/**
		 * Set vertical resolution
		 * @param eVertRes resolution
		 */
		//void setScannerResolution(int nRes);

		/**
		 * Get vertical resolution
		 * @return vertical resolution
		 */
		//int getScannerResolution();

		/**
		 * Gets the servo range which is maximal allowed.
		 * @param pnStart Value is set to the maximal start position.
		 * @param pnStop Value is set to the maximal stop position.
		 * @return Number of servo positions between start and stop.
		 */
		int getMaxRange(int *pnStart, int *pnStop);

		/**
		 * Gets the current servo range.
		 * @param pnStart Value is set to the current start position.
		 * @param pnStop Value is set to the current stop position.
		 * @return Number of servo positions between start and stop.
		 */
		int getRange(int *pnStart, int *pnStop);

		/**
		 * Sets a new servo range. Invalid values should be ignored. On Success
		 * (pnStop-pnStart+1) should be the return value.
		 * @param nStart Value of the new start position.
		 * @param nStop Value of the new stop position.
		 * @return Number of servo positions between the new start and stop position.
		 */
		int setRange(int nStart, int nStop);

		/**
		 * Get the current axis of rotation.
		 * @param pdX x-axis.
		 * @param pdY y-axis.
		 * @param pdZ z-axis.
		 */
		void getRotationAxis(double *pdX, double *pdY, double *pdZ);

		/**
		 * Set a new axis of rotation.
		 * @param dX x-axis.
		 * @param dY y-axis.
		 * @param dZ z-axis.
		 */
		void setRotationAxis(double dX, double dY, double dZ);

		/**
		 * Get the orientation of the z-axis of the scanner.
		 * @param pdPitch rotation around x-axis (radiant)
		 * @param pdYaw rotation around y-axis (radiant)
		 * @param pdRoll rotation around z-axis (radiant)
		 */
		void getOrientation(double *pdPitch, double *pdYaw, double *pdRoll);

		/**
		 * Set the new orientation of the z-axis of the scanner.
		 * @param dPitch rotation around x-axis (radiant).
		 * @param dYaw rotation around y-axis (radiant).
		 * @param dRoll rotation around z-axis (radiant).
		 */
		void setOrientation(double dPitch, double dYaw, double dRoll);

		/**
		 * Returns the number of values in a scan. This is important for the
		 * size of the buffer in the getScan-method.
		 * @return Number of values per one scan.
		 */
		int getValuesPerScan();

		/**
		 * Connect device
		 * @return state of succession (success != 0)
		 */
		int connect();

		/**
		 * Disconnect device
		 * @return state of succession (success != 0)
		 */
		int disconnect();

		/**
		 * Get one 3D scan
		 * @param pdScan pointer to buffer to store distance measurement values
		 * @return state of scanning
		 */
		//	unsigned long getScan(double* pdScan);

		/**
		 * Get one 3D Scan from the Scanner. pCloud must point to a
		 * CCartesianCloud3D. The cloud must have enough initialized points to
		 * store all values of the scan. See also the getValuesPerScan()-
		 * method.
		 * N.B. : it will take 360 scans
		 * @param pCloud The results will be stored in this cloud.
		 * @return The number of points scanned or 0 on failure.
		 */
		unsigned long getScan(CCartesianCloud3D *pCloud);

		/**
		 * Get one 3D Scan from the Scanner. pCloud must point to a
		 * CCartesianCloud3D. The cloud must have enough initialized points to
		 * store all values of the scan. See also the getValuesPerScan()-
		 * method. See also private function get1Scan called by this function
		 * N.B. : it will take 360 scans / nResolution
		 * @param pCloud The results will be stored in this cloud.
		 * @param nResolution The resolution of the cloud
		 * @return The number of points scanned or 0 on failure.
		 */
		//unsigned long getScan(CCartesianCloud3D *pCloud, int nResolution);
		//resolution is a global variable now!


		/**
		 * get the resolution
		 */
		double getResolution();

		/**
		 * set the resolution
		 * @param nResolution The desired resolution
		 */
		void setResolution(int nResolution);

		void setResolution(double nResolution);

		//beta = values in the scan
		unsigned long thread_getScan(CCartesianCloud3D *pCloud);

		//first beta = value in the scan, afterwards increment with resolution
		unsigned long thread2_getScan(CCartesianCloud3D *pCloud);

		/**
		 * it takes a new complete scan and save it in a CImage object
		 * @param vImage The "empty" Image to be filled
		 */
		//void getImages(vector<CImage<double>*>* vImages);
		/**
		 * it transform the Image to a CartesianCloud
		 * @param vImage The image with the data
		 * @param cloud	 The "empty" cloud to be filled
		 */
		//void imageVectorToCloud(vector<CImage<double>*> vImages, CCartesianCloud3D* cloud);


		typedef struct {
				double* pdDist;
				double* pdRems;
				double dMotorControllerPos;
				CMatrix44 systemPosMatrix;
				//double dMotorControllerPos_before;
				//double dMotorControllerPos_after;
				//CMatrix44 systemPosMatrix_before;
				//CMatrix44 systemPosMatrix_after;
		} SScan;

	private:

		/**
		 * Get one 3D Scan from the Scanner. pCloud must point to a
		 * CCartesianCloud3D. The cloud must have enough initialized points to
		 * store all values of the scan.
		 * We call this function to acquire data from the two scanners. We need a function,
		 * because the code for the two scanners is almost the same, but with some different parameters
		 * in the conversione <double* -> Cloud>
		 * N.B. : it will take 360 scans
		 * @param pCloud The results will be stored in this cloud.
		 * @param dist Where the distance values are stored
		 * @param rems Where the remission values are stored
		 * @param nResolution The resolution of the cloud
		 * @param nScanner The number of the scanner to be used (1 or 2)
		 * @param j The position of the scan in the whole cloud
		 * @param nNrPointsHor How many values in each scan
		 * @param delta angle between two consecutive points in the same scan -
		 *		  the scanners rotate continuously and so we have to pay attention to the offset
		 delta is function of the speed of rotation, resolution and "apex angle" (how many points for each scan)
		 * @return The number of points scanned or 0 on failure.
		 */
		unsigned long get1Scan(CCartesianCloud3D *pCloud, double *dist, double *rems, int nScanner, int j, int nNrPointsHor, double delta);

		void analyzeScan(CRotatingSICKLMS100::SScan scan2, CCartesianCloud3D *pCloud, int j, double delta);

		/**
		 * Internal initialization method.
		 */
		void init();

		/**
		 * Scanner device
		 */
		CSickLms100Scanner* _pScanner1;
		/**
		 * Servo device
		 */
		//IServo* _pServo;

		/**
		 * Motor Controller
		 */
		IMotorController* _pMotorController;

		/**
		 * SystemPoseAdapter
		 */
		ISystemPoseAdapter* _pSystemPoseAdapter;

		/**
		 * Scanner connected?
		 */
		bool _bConnected1;

		/**
		 * MotorController connected?
		 */
		bool _bConnectedMC;

		/**
		 * Maximum vertical apex angle
		 */
		int _nMaxApexAngle;

		/**
		 * Vertical apex angle
		 */
		int _nApexAngle;

		/**
		 * Maximum vertical resolution in Steps per 360 degree
		 */
		int _nMaxResolution;

		/**
		 * Vertical resolution in Steps per 360 degree
		 */
		//	int _nResolution;

		/**
		 * Minimum position of servo device
		 */
		//int _nServoPosMin;

		/**
		 * Maximum position of servo device
		 */
		//int _nServoPosMax;

		/**
		 * Horizontal position of servo device
		 */
		//int _nServoPosMid;

		/**
		 * Start position of servo device
		 */
		//int _nServoPosStart;

		/**
		 * Stop position of servo device
		 */
		//int _nServoPosStop;

		/**
		 *  Vector describing the axis of rotation.
		 */
		CVector3 _rotation;

		/**
		 * Axis of rotation.
		 */
		double _dRotationX;

		/**
		 * Axis of rotation.
		 */
		double _dRotationY;

		/**
		 * Axis of rotation.
		 */
		double _dRotationZ;

		/** orientation (rotation around x-axis) */
		double _dPitch;
		/** orientation (rotation around y-axis) */
		double _dYaw;
		/** orientation (rotation around z-axis) */
		double _dRoll;

		CMatrix44 _pitchYawRollTransformMatrix;

		/**
		 * Direction of vertical (servo) rotation.
		 */
		EnumScanDirection _direction;

		/**
		 * Timeout
		 */
		double _dTimeout;

		/**
		 * Resolution
		 */
		double _nResolution;

		/**
		 * for Scanners with Angular Adjuster Plate
		 */
		double _dTilt;

		/**
		 * remission data buffer
		 */
		double* _adRemission1;
		double* _adRemission2;

		/**
		 * distance data buffer
		 */
		double* _adScan1;
		double* _adScan2;

		int _nNrPointsVertical;
		double _delta;

		//double _dOffsetRotationPositionMarker;


		/**
		 * Thread of the scanner
		 */
		pthread_t _thScanner;

		pthread_t _thScanner1;

		pthread_t _thScanner2;

		/**
		 * Thread of controller
		 */
		//	pthread_t _thController;

		/**
		 * Thread attributes
		 */
		pthread_attr_t _attr;

};

}

#endif
