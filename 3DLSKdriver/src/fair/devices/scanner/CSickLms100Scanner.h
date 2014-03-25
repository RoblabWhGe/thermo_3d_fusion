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

#ifndef CSICKLMS100SCANNER_H_
#define CSICKLMS100SCANNER_H_

#include <string>
#include <iostream>
#include "../../../fair/devices/factory/IDevice.h"
#include "../../../fair/core/io/IDeviceAdapter.h"
#include "../../../fair/devices/scanner/IScanner.h"

using namespace std;

namespace fair
{

class CSickLms100Scanner : public IScanner
{
public:
  /**
   * Standard constructor. This initializes the object and sets the IDeviceAdapter.
   * @param pAdapter Deviceadapter normally a socketconnection
   */

  CSickLms100Scanner(IDeviceAdapter *pAdapter);

  /**
   * Default constructor. This initializes the object.
   * @post The object must be initialized
   */
  CSickLms100Scanner();

  /**
   * Default destructor. This cleans up.
   */
  virtual ~CSickLms100Scanner();

  /**
   * Get the current deviceadapter. Returns a pointer to the current
   * deviceadapter. If no deviceadapter has been set NULL is returned.
   * @return Pointer to the adapter.
   */
  virtual IDeviceAdapter *getDeviceAdapter();

  /**
   * Set a new deviceadapter. Sets the deviceadapter to the adapter
   * pAdapter points to. If pAdapter is NULL the adapter
   * has is unset.
   *
   * @param pAdapter Pointer to the adapter.
   */
  virtual void setDeviceAdapter(IDeviceAdapter *pAdapter);

  /**
   * Get time for taking one scan line. Returns a time in the format
   * milliseconds.microseconds.
   *
   * @return Time in milliseconds.mircoseconds.
   */
  virtual double getScanningTime();

  /**
   * Get unit of range data. Returns an element of EnumUnit depending on
   * the measurement unit the range values are returned.
   *
   * @returns Unit
   */
  virtual EnumUnit getUnit();

  /**
   * Get current Apex Angle of scanner.Returns the current apex angle of
   * the laserscanner in degree.
   *
   * @return ApexAngle in degree
   */
  virtual int getScannerApexAngle();

  /**
   * Get resolution of scanner in steps per full circle. An implementation
   * returns the current resolution of the laserscanner.
   *
   * @return Number of steps per full circle.
   */
  virtual int getScannerResolution();

  /**
   * Get scanning direction. Returns a value of EnumScanDirection depending
   * on the rotation direction in which the laserscan takes place. From a top
   * point of view this could be in a clockwise or a counterclockwise direction.
   *
   * @returns Direction from top view.
   */
  virtual EnumScanDirection getDirection();

  /**
   * Get the scan range which is maximal allowed. Sets pnStart to the minimal
   * start and pnStop to the maximal end scan position allowed by the laserscanner.
   * Returns the number ofscan positions between the minimal start and maximal end
   * positions (including the start and end position).
   *
   * @param pnStart Value is set to the maximal start position.
   * @param pnStop Value is set to the maximal stop position.
   * @return Number of scan positions between start and stop.
   */
  virtual int getMaxRange(int *pnStart, int *pnStop);

  /**
   * Get the current scan range. Sets pnStart to the current start and pnStop
   * to the current end scan position. Returns the number of scan positions
   * between the start and end positions (including the start and end position).
   *
   * @param pnStart Value is set to the current start position.
   * @param pnStop Value is set to the current stop position.
   * @return Number of scan positions between start and stop.
   */
  virtual int getRange(int *pnStart, int *pnStop);

  /**
   * Set a new scan range. Sets the new start scan position to nStart and the
   * new end scan position to nStop if those are valid values.
   *
   * Returns the number of scan positions between the new start and end positions
   * (including the start and end position) if they are changed. If the start and
   * stop positions are not changed because an error occured (e.g. the provided
   * positions are out of the allowed range) 0 must be returned.
   * @param nStart Value of the new start position.
   * @param nStop Value of the new stop position.
   * @return Number of steps between the new start and end position.
   * @post The start and end scan positions are set to nStart and
   * nStop and are used in future laserscans or 0 must be returned.
   */
  virtual int setRange(int nStart, int nStop);

  /**
   * Returns the number of values in a scan. Returns the number of scan positions
   * between the start and end positions (including the start and end position).
   *
   * That means that the return value must equal the return value of a
   * call of getRange() at the same time.
   * @return Number of values per one scan.
   */
  virtual int getValuesPerScan();
  /**
   * Set resolution of sick scanner.
   * @param eResolution the resolution
   */
  int setResolution(fair::EnumResolution resolution);

  /**
   * Connect the scanner through the deviceadapter.
   * Opens a connection to the scanner through the deviceadapter set by the
   * constructor IScanner(IDeviceAdapter* pAdapter) or by
   * setDeviceAdapter(IDeviceAdapter* pAdapter). If no adapter has been
   * set or an error occurs 0 must be returned.
   * @return state of succession (success !)
   * @pre Deviceadapter must not be NULL and the scanner must not be
   * connected already.
   * @post The laserscanner must be connected or 0 must be returned.
   */
  virtual int connect();

  /**
   * Disconnect the scanner. Closes the connection
   * to the scanner. If an error occures 0 is returned.
   * @return state of succession (success !)
   * @pre The scanner is connected.
   * @post The laserscanner is disconnected or 0 is returned.
   */
  virtual int disconnect();

  /**
   * Get one scan from the laserscanner. Does a laserscan starting at the
   * given start point and ending at the given end point (see setRange() method).
   * It writes the range values sequentially into pdPoints. It returns the number
   * of range values written. If an error occurs and no data are written 0 is
   * returned.
   *
   * @param pdPoints The results will be stored in an array of doubles.
   * @return The number of points scanned or 0 on failure.
   * @pre pdPoints is an array of double values which can hold at
   * least getValuesPerScan() range values.
   */
  virtual unsigned long getScan(double *pdPoints);

  /**
   * Get one scan from the laserscenner. Note that this method is a blocking method.
   * For non-blocking access use getScan(double* pdScan, double* pdRemission, unsigned int* pnScan)
   * @param pdScan buffer to be filled with distance values
   * @param pdRemission buffer to be filled with remission values (only in remission mode!)
   * @return number of scan points returned
   */
  virtual unsigned long getScan(double *pdScan, double *pdRemission);

  /**
   * Get resolution of sick scanner.
   * Note: A resolution of a quarter degree is only provided with an apex angle of 100 degree
   * (except in interlaced mode).
   * @return the resolution
   */
  virtual fair::EnumResolution getResolution();
private:

  /*
   * Easy interface for sending and receiving messages from the laser-scanner.
   * @param sCommand The command that should be send to the laser-scanner (without frames)
   * @param nReturnLenght The maximum lenght of the returned string.
   * @return Command that has been received from the laser-scanner
   * 		   (most of the times this has already been decoded by decodeScan)
   */
  string CallCommand(string sCommand, int nReturnLenght);

  /*
   * Initializes the LMS100 by sending initialization messages.
   * @return Returns true of the initialization is successful
   */
  bool initalize();

  /*
   * This function generates message frames and sends them over the pAdapter
   * to the laser-scanner.
   * @param sCommand Command that should be send to the laser-scanner.
   * @return Returns true if message was successfully sent, false if an error occured
   */
  bool sendMessage(string command);

  /*
   * This function receives messages from the LMS laser-scanner and runs the
   * according decoding sequence
   * @param sResult Is set to the received message form the laser-scanner
   * @param ulLength The maximum length of the message received from the laser-scanner
   * @return Returns true if message was successfully received, false if an error occured
   */
  bool receiveMessage(string* sResult, unsigned long ulLength);

  /*
   * Decodes the scan message that has been received from the laser-scanner
   * and writes the data in the according object variables.
   * @param sBuffer The message from the laser-scanner that should be decoded
   * @param pdScandata Is set to the scanning data of the laserscanner
   * @return Returns true if the message has been successfully decoded
   */
  bool decodeScan(string sBuffer, double* pdScandata, double* pdRemissionData);

  /*
   * Device connected.
   */
  bool _bConnected;

  /*
   * IDeviceAdapter Handle
   */
  IDeviceAdapter* _pAdapter;

  /*
   * The resolution of the scanner
   */
  int _nResolution;

  /*
   * The Unit in which the scanner is measuring
   */
  EnumUnit _eUnit;

  /*
   * The ApexAngle of the laserscanner
   */
  int _nApexAngle;

  /*
   * Scanning time for one scan
   */
  int _nScanningTime;

  /*
   * The scanning direction of the laserscanner
   */
  EnumScanDirection _eScanDirection;

  /*
   * Defines the starting index of the laserscanner
   */
  int _nStartScanNumber;

  /*
   * Scan counter, number of last scan received
   */
  int _nScanCounter;
  int _nOldScanNumber;

  /*
   * Scanner serial number
   */
  int _nSerialNumber;

  /**
   * buffer to store remission values if no valid external buffer is provided
   */
  double _adRemission[1024];

  unsigned int _scannerSufaceToCenterOffset;

  /**
   * Current resolution
   */
  EnumResolution _eResolution;

  bool setAccessMode();
  string ACCESS_MODE_MAINTAINER;
  string ACCESS_MODE_CLIENT;
  string ACCESS_MODE_SERVICE;
  string USER_LEVEL_MAINTAINER;
  string USER_LEVEL_CLIENT;
  string SOPAS_METHODE_BY_NAME;
  string CMD_SET_ACCESS_MODE_CMD;
  string CMD_STATUS;
  string SOPAS_READ_ANSWER;
  string SOPAS_READ_BY_NAME;
  enum EnumStatus
  {
    eStatusUndef = 0, eStatusInit = 1, eStatusConfig = 2, eStatusIdle = 3, eStatusRotating = 4, eStatusInProgress = 5,
    eStatusEnabled = 6, eStatusReady = 7
  };

};

}

#endif /*CSICKLMS100SCANNER_H_*/
