/*
*
* libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
* 
* Copyright Fraunhofer Gesellschaft e. V., Munich, Germany
*
* The FAIR library [both binary and source code (if released)] is intellectual
* property owned by Fraunhofer Gesellschaft and is protected by copyright;
* the ownership remains with Fraunhofer Gesellschaft.
*
*/

#ifndef CSICKSCANNER_H
#define CSICKSCANNER_H

#include <iostream>
#include <stdio.h>
#include "../../../fair/core/io/com/CComAdapter.h"
#include "../../../fair/core/base/CTimer.h"
#include "../../../fair/devices/scanner/IScanner.h"
#include "sickmsg.h"

using namespace std;

namespace fair
{

/**
 * Maximum wait time for acknowlegde byte
 */
#define SICKMAXACKWAITTIME 70
/**
 * Maximum wait time for data transmission
 */
#define SICKMAXDATAWAITTIME 27


/**
 * @class CSickScanner
 * @brief Class encapsules sick scanner functionality
 * @author Daniela Weiss (Windows specific code), Stefan May (Linux specific code)
 **/
class CSickScanner : public IScanner {
	public:
		/**
		 * Standard constructor
		 */
		CSickScanner(IDeviceAdapter* pAdapter);
		/**
		 * Default destructor
		 */
		~CSickScanner();
        /**
         * Connect device
         * @return state of succession
         */
        int connect();
        /**
         * Disconnect device
         * @return state of succession
         */
        int disconnect();
        
        /**
         * Set a new device-adapter. If the scanner is already connected the
         * connection will be reseted.
         * @param pAdapter Pointer to the new adapter.
         */
        void setDeviceAdapter(IDeviceAdapter *pAdapter);
        
        /**
         * Get the current device-adapter.
         * @return Pointer to the adapter.
         */
        IDeviceAdapter *getDeviceAdapter();
        
        /**
         * Set apex angle of sick scanner.
         * Note: A resolution of a quarter degree is only provided with an apex angle of 100 degree.
         * In interlaced mode, the scanner then provides 180 degrees, NOT 100 degrees!
         * @param eApexAngle the apex angle
         */
        int setApexAngle(EnumApexAngle eApexAngle);
        /**
         * Get apex angle of sick scanner.
         * Note: A resolution of a quarter degree is only provided with an apex angle of 100 degree.
         * In interlaced mode, the scanner then provides 180 degrees, NOT 100 degrees!
         * @return the apex angle
         */
        EnumApexAngle getApexAngle();
        
        /**
         * Get apex angle of sick scanner.
         * Note: This method is provided to implement IScanner.
         * @return ApexAngle in degree
         */
        int getScannerApexAngle();
        
        /**
         * Set resolution of sick scanner.
         * Note: A resolution of a quarter degree is only provided with an apex angle of 100 degree
         * (except in interlaced mode).
         * @param eResolution the resolution
         */
        int setResolution(EnumResolution eResolution);
        /**
         * Get resolution of sick scanner.
         * Note: A resolution of a quarter degree is only provided with an apex angle of 100 degree
         * (except in interlaced mode).
         * @return the resolution
         */
        EnumResolution getResolution();
        
        /** Get resolution of sick scanner in steps per full circle. So the
         * resolution is 360 divided by the return value.
         * Note: This method is provided to implement IScanner.
         * @return Number of steps per full circle.
         */
        int getScannerResolution();
        
        /**
         * Get time for taking one scan line
         */
		double getScanningTime();
        /**
         * Set unit of distance data.
         * @param eUnit the unit
         * @return success
         */
		int setUnit(EnumUnit eUnit);
		/**
		 * Get unit of distance data.
		 * @return unit
		 */
		EnumUnit getUnit();
		
		/**
		 * Get scanning direction.
		 * @returns Direction from top-view.
		 */
		EnumScanDirection getDirection();
		
		/**
		 * Get the scan range which is maximal allowed.
		 * Note: This method is provided to implement IScanner.
		 * @param pnStart Value is set to the minimal start position.
		 * @param pnStop Value is set to the maximal stop position.
		 * @return Number of scan positions between start and stop.
		 */
		int getMaxRange(int *pnStart, int *pnStop);
		
		/**
		 * Get the current scan range.
		 * Note: This method is provided to implement IScanner.
		 * @param pnStart Value is set to the current start position.
		 * @param pnStop Value is set to the current stop position.
		 * @return Number of scan positions between start and stop.
		 */
		int getRange(int *pnStart, int *pnStop);
		
		/**
		 * Set a new scan range. Invalid values will be ignored. On success
		 * (pnStop-pnStart+1) should be the return value.
		 * Note: This method is provided to implement IScanner.
		 * @param nStart Value of the new start position.
		 * @param nStop Value of the new stop position.
		 * @return Number of scan positions between the new start and stop.
		 */
		int setRange(int nStart, int nStop);
		
		/**
		 * Returns the number of values in a scan.
		 * @return Number of values per one scan.
		 */
		int getValuesPerScan();
				
		/**
		 * Set remission range. The LMS 200 does only support a maximum amount of 200 remission values
		 * due to limited internal buffer. This means, that the difference between the start
		 * point index and the end point index can not differ by values greater than 200.
		 * @param nStart Start value of remission range (point index, not degree!)
		 * @param nEnd End value of remission range (point index, not degree!)
		 * @return success state of setting range (fails, if range is invalid)
		 */
		int setRemissionRange(unsigned int unStart, unsigned int unEnd);
		
		/**
		 * Get remission range.
		 * @param unStart pointer to start point index
		 * @param unEnd pointer to end point index
		 */
		void getRemissionRange(unsigned int* unStart, unsigned int* unEnd);
		
        /**
         * Switch operation mode of scanner
         * @return state of switching mode (success != 0)
         */
        int setOperationMode(EnumOperationMode eMode);
        
        /**
         * Get operation mode
         * @return operation mode
         */
		EnumOperationMode getOperationMode();
  
  		/**
  		 * Switch transmission mode of scanner
  		 * @return state of switching mode (success!=0)
  		 */
		int setTransmissionMode(EnumTransmissionMode eMode);
		
		/**
		 * Get transmission mode
		 * @return transmission mode
		 */
		EnumTransmissionMode getTransmissionMode();
		 
		/**
		 * Request one scan (RequestSending mode!)
		 * @return state of request 
		 */		
		int requestScan();
		
        /**
         * Get one scan
         * @param pdScan Buffer to be filled within this function
         * @return success state
         */
		unsigned long getScan(double* pdScan);
		
        /**
         * Get one scan. Note that this method is a blocking method.
         * For non-blocking access use getScan(double* pdScan, double* pdRemission, unsigned int* pnScan)
         * @param pdScan buffer to be filled with distance values
         * @param pdRemission buffer to be filled with remission values (only in remission mode!)
         * @return success state
         */
		unsigned long getScan(double* pdScan, double* pdRemission); 
		
        /**
         * Get one scan
         * @param pdScan buffer to be filled with distance values
         * @param pdRemission buffer to be filled with remission values (only in remission mode!)
         * @param pnScan number of scan part
         * @return success state
         */
		unsigned long getScan(double* pdScan, double* pdRemission, unsigned int* pnScan);
		
	private:
		/**
		 * Helper method to provide a stable communication.
		 * This function tries to communicate with the device for several times.
		 * @return State of communication
		 * @param pucMsg message to be send to device
		 * @param ulMsgLen length of message to be send to device
		 * @param pucExpectedAnswer expected answer from device
		 * @param ulAnswerLen length of expected answer
		 * @param ulWaitTime wait time between two communication tries
		 * @param ulRetries maximum amount of communication retries
		 */
		int adapterSend(unsigned char* pucMsg,
						unsigned long ulMsgLen,
						unsigned char* pucExpectedAnswer,
						unsigned long ulAnswerLen,
						unsigned long ulWaitTime,
						unsigned long ulRetries);

		/**
		 * Helper method. Build telegram for sick scanner (including check sum).
		 * @param szMsg message
		 * @param ulMsgSize size of message
		 * @param ucAddress receiver address (for communication with multiple scanner; a broadcast adress is also available).
		 * @param szTelegram telegram as out parameter
		 */
		void buildTelegram(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char ucAddress, unsigned char* szTelegram);

		/**
		 * Wrapper method for building up send telegrams (uses broadcast adress).
		 * @param szMsg message
		 * @param ulMsgSize length of message
		 * @param szTelegram telegram as out parameter
		 */
		void buildMessage(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char* szTelegram);

		/**
		 * Wrapper method for building up receive telegrams.
		 * @param szMsg message
		 * @param ulMsgSize length of message
		 * @param szTelegram telegram as out parameter
		 */
		void buildAnswer(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char* szTelegram);

		/**
		 * Send telegram to device
		 * @param pucMsg message
		 * @param ulMsgLen length of message
		 * @param pucExpectedAnswer expected answer to message
		 * @param ulAnswerLen expected length of answer message
		 * @param ulWaitTime wait time between read retries
		 * @param ulRetries maximum amout of retries
		 */
		int sendTelegram(unsigned char* pucMsg,
						unsigned long ulMsgLen,
						unsigned char* pucExpectedAnswer,
						unsigned long ulAnswerLen,
						unsigned long ulWaitTime,
						unsigned long ulRetries);

		/**
		 * Adapter to device
		 */
		IDeviceAdapter* _pAdapter;

		/**
		 * Telegram buffer
		 */
		unsigned char* _szTelegram;

		/**
		 * Answer buffer
		 */
		unsigned char* _szAnswer;

		/**
		 * Distance mask (bit-coded mask for calculting real distances)
		 */
		unsigned char _ucDistMask;

		/**
		 * Current resolution
		 */
		EnumResolution _eResolution;
		
		/**
		 * Current apex angle
		 */
		EnumApexAngle _eApexAngle;
		
        /**
		 * Current distance data unit
		 */
		EnumUnit _eUnit;
		
		/**
		 * Current operation mode
		 */
		EnumOperationMode _eOperationMode;
		
		/**
		 * Current transmission mode
		 */
		EnumTransmissionMode _eTransmissionMode;
			
		/**
		 * Number of data values receiving from adapter
		 */
		int _nDataValues;
		
		/**
		 * Number of scan parts
		 */
		int _nScanParts;
		int _nScanPartsAcquired;
		
		/**
		 * Timeout for adapter access tries in milliseconds
		 */
		int _nTimeout;
		
		/**
		 * Timer for device synchronisation
		 */
		CTimer* _timer;
		
		/*
		 * Start point index of remission range
		 */
		unsigned int _unRemissionStart;
		
		/**
		 * End point index of remission range
		 */
		unsigned int _unRemissionEnd;
		
		/**
		 * buffer to store remission values if no valid external buffer is provided
		 */
		double _adRemission[1024];
		
		double _adScanTmp[4][361];
		double _adRemTmp[4][361];
		
		/**
		 * Device connected?
		 */
		bool _bConnected;
		
		char* _pszBuffer;
		
		bool _bMsgComplete;
		
		bool _bMsgCorrect;
		
		int _nMessageLen;

        /**
         * Local copies of the standard protocol messages used to communicate with
         * the scanner hardware.
         */
        unsigned char _aucMsgSetupBaudrate[sizeof(g_aucMsgSetupBaudrate)];
        unsigned char _aucMsgSetupConfigMode[sizeof(g_aucMsgSetupConfigMode)];
        unsigned char _aucMsgSetupTrans[sizeof(g_aucMsgSetupTrans)];
        unsigned char _aucMsgSetupTransRem[sizeof(g_aucMsgSetupTransRem)];
        unsigned char _aucAnswSetup[sizeof(g_aucAnswSetup)];
        unsigned char _aucMsgOpDataRequest[sizeof(g_aucMsgOpDataRequest)];
        unsigned char _aucMsgVariant[sizeof(g_aucMsgVariant)];
        unsigned char _aucAnswVariant[sizeof(g_aucAnswVariant)];
        unsigned char _aucMsgTrans[sizeof(g_aucMsgTrans)];
        unsigned char _aucAnswTrans[sizeof(g_aucAnswTrans)];
};

}

#endif /*CSICKSCANNER_H*/
