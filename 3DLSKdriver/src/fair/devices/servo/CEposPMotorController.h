#ifndef CEPOSPMC_H__
#define CEPOSPMC_H__



#include <stdio.h>
#include "../../../fair/core/io/com/CComAdapter.h"
#include "../../../fair/core/base/CTimer.h"
#include "../../../fair/devices/servo/IMotorController.h"
#include <iostream>
//#include <pthread.h>

using namespace std;

namespace fair
{

/**
 * @class CEposPMC
 * @brief Class implements Maxonmotor EPOS P 24/5 motor controller control functions
 * @author David Droeschel, Thorsten Linder
 */
class CEposPMotorController : public IMotorController {

public:

	/**
	 * standard constructor
	 * @param adapter com adapter instance
	 */
  	CEposPMotorController(fair::CComAdapter* adapter);

	/** Default Constructor. Does nothing. */
  	//CEposPMotorController(){}
  	/**
  	 * standard destructor
  	 */
	~CEposPMotorController();

	/**
	 * to implement IMotorController
	 */
	float getVelocity();

 	// to implement IMotorController
	double getPosition();

	 
	double getRotationPeriod();

	// to implement IMotorController
	int getRateVelocity();


	// to implement IMotorController
	IDeviceAdapter *getDeviceAdapter();

	// to implement IMotorController
	void setDeviceAdapter(IDeviceAdapter *pAdapter);

	// to implement IMotorController
	void setVelocity(float fVelocity);

	int connect();
	int disconnect();
	int getEncoderPosition();

	CTimer* _timer;
	double _timePeriod;
	double _offset;
	double _nNumberEncoderPosition;
		
private:
	CComAdapter* _adapter;

	int _rateVelocity;

	double _dOffsetRotationPositionMarker;

	
	  
	/**
	 * variable stores the time within the servo has to be set
	 */
	unsigned long _ulTime;
	 
	
	
	CTimer* _timerSetTime;
//	double  _timePeriod;

	int init();
//	int getEncoderPosition();

	/**
	 * Thread of the controller
	 */
	pthread_t _thController;
	
	/**
	 * Thread attributes
	 */
	pthread_attr_t _attr;


};

}

#endif
