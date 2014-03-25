#include "CRotatingSICKLMS100.h"

//#define VELOCITY 3300.0


namespace fair {

	/*
	 * Struct for the information for each scan taken from the SickScanners
	 * pdDist					  = array of double with distance values
	 * pdRems					  = array of double with remission values
	 * dMotorControllerPos = angle in radiant about the position
	 * systemPosMatrix     = 4x4 matrix with pos information (rotation and transation) of the system
	 *

	 struct CRotatingSICKLMS100::SScan {
	 double* pdDist;
	 double* pdRems;
	 double dMotorControllerPos;
	 CMatrix44 systemPosMatrix;
	 //double dMotorControllerPos_before;
	 //double dMotorControllerPos_after;
	 //CMatrix44 systemPosMatrix_before;
	 //CMatrix44 systemPosMatrix_after;
	 };
	 */

	CSickLms100Scanner* g_scanner1 = NULL;
	ISystemPoseAdapter* g_systemPoseAdapter = NULL;
	IMotorController* g_motorController = NULL;

	double g_dEncoderPos = 0;

	bool g_bRun = true;
	bool g_bScannerExit = false;
	double* g_pdScan1 = NULL;
	double* g_pdRemission1 = NULL;

	double* g_pdScan2 = NULL;
	double* g_pdRemission2 = NULL;

	bool g_bScanReceived = false;

	struct thread_arguments {
		CSickLms100Scanner* scanner;
		double scannerOffset;
	};

	typedef struct thread_arguments thread_args;

	queue<CRotatingSICKLMS100::SScan> g_queueScans;

	// with the access to the Controller
	void* scannerCapture(void* arg) {

		CRotatingSICKLMS100::SScan scan1;

		//scanner1 has to update beta_controller, scanner2 only add 180°
		double beta_controller = 0.0;

		beta_controller = g_motorController->getPosition();
		//cout << "beta controller: " << beta_controller << endl;
		//scan1.dMotorControllerPos = beta_controller;
		//scan2.dMotorControllerPos = beta_controller + M_PI;

		//scan1.dMotorControllerPos = beta1*M_PI/180;
		//scan2.dMotorControllerPos = beta2*M_PI/180;
		//beta += 2;


		while (g_bRun == true) {

			//		std::cout << "enclosing_method--beta_controller "<< beta_controller << std::endl;
			//			CTimer* t = new CTimer();
			//		scan1.dMotorControllerPos = beta1*M_PI/180.0;


			if (g_scanner1 != NULL) {
				//std::cout << " enclosing_method::g_scanner1->getValuesPerScan() " << g_scanner1->getValuesPerScan() << std::endl;
				//scan.dMotorControllerPos_before = g_motorController->getPosition();
				//scan.systemPosMatrix_before     = g_systemPoseAdapter->getPos();
				/*		if((g_scanner1->getScanningTime()-0.01)>timer->getTime())
				 sched_yield();
				 else
				 {*/
				int result = g_scanner1->getScan(g_pdScan1, g_pdRemission1);

				if (result > 0) {
					// data received
					g_bScanReceived = true;
					scan1.dMotorControllerPos = g_motorController->getPosition();
					//std::cout << "scan1.dMotorControllerPos " << scan1.dMotorControllerPos <<endl;				//cout << "scan1.dMotorControllerPos = g_motorController->getPosition()" << scan1.dMotorControllerPos<<endl;

					// getSystemPos
					scan1.systemPosMatrix = g_systemPoseAdapter->getPos();

					//save into the queue:
					scan1.pdDist = g_pdScan1;
					scan1.pdRems = g_pdRemission1;

					g_queueScans.push(scan1);

					//beta_controller=g_motorController->getPosition();
					//scan1.dMotorControllerPos = beta_controller;

					sched_yield();
				} else {
					sched_yield();
					usleep(1);

				}
				//	}
			}

		}
		g_bScannerExit = true;
		return NULL;
	}

	// scannerCapture for one scanner, seperate thread for each scanner. Offset M_PI
	void* singleScannerCapture(void* data) {
		cout << "initialization begin" << endl;
		const thread_args* args = static_cast<thread_args*> (data);

		CSickLms100Scanner* _scanner = (CSickLms100Scanner*) args->scanner;
		double _scannerOffset = args->scannerOffset;
		cout << "scanner offssset: " << static_cast<int> (_scannerOffset) << endl;
		cout << "scanner connected: " << _scanner->getValuesPerScan() << endl;

		CTimer* timer = new CTimer();
		CRotatingSICKLMS100::SScan scan;

		//scanner1 has to update beta_controller, scanner2 only add 180°
		//double beta_controller = 0.0;

		double* _pdScan = NULL;
		double* _pdRemission = NULL;
		//scan2.dMotorControllerPos = beta_controller + ;

		//scan1.dMotorControllerPos = beta1*M_PI/180;
		//scan2.dMotorControllerPos = beta2*M_PI/180;
		//beta += 2;

		cout << "initialization end" << endl;
		while (g_bRun == true) {

			//		cout << "beta_controller: " <<beta_controller<< endl;
			//CTimer* t = new CTimer();
			//		scan1.dMotorControllerPos = beta1*M_PI/180.0;
			//		scan2.dMotorControllerPos = beta2*M_PI/180.0;


			if (_scanner != NULL) {

				cout << "if(_scanner!=NULL)" << endl;

				//scan.dMotorControllerPos_before = g_motorController->getPosition();
				//scan.systemPosMatrix_before     = g_systemPoseAdapter->getPos();
				/*		if((g_scanner1->getScanningTime()-0.01)>timer->getTime())
				 sched_yield();
				 else
				 {
				 */if (_scanner->getScan(_pdScan, _pdRemission) > 0) {
					cout << "if(_scanner->getScan(_pdScan, _pdRemission)>0)" << endl;
					//beta_controller=g_motorController->getPosition();
					//scan.dMotorControllerPos = beta_controller+_scannerOffset;

					// data received
					g_bScanReceived = true;

					// getSystemPos
					scan.systemPosMatrix = g_systemPoseAdapter->getPos();

					//save into the queue:
					scan.pdDist = _pdScan;
					scan.pdRems = _pdRemission;
					//scan.dMotorControllerPos_after = g_motorController->getPosition();
					//scan.systemPosMatrix_after     = g_systemPoseAdapter->getPos();

					g_queueScans.push(scan);

					timer->reset();

					//beta_controller=g_motorController->getPosition();
					scan.dMotorControllerPos = g_motorController->getPosition() + _scannerOffset;

					sched_yield();
				} else {
					sched_yield();

				}
				//	}
			}

			//cout << "First scanner - time: " << t->getTime() <<endl;

		}
		cout << "exiting in singleScannerCapture" << endl;
		g_bScannerExit = true;
		delete timer;
		return NULL;
	}

	/*
	 void* controllerPos(void* arg){
	 while(true){
	 //		cout << "------------in thread MotorController"<<endl;
	 //	CEposPMotorController* mc = (CEposPMotorController*) arg;
	 //	cout<<"---after casting"<<endl;
	 //		double dEncoderPos=((CEposPMotorController*)g_motorController)->getEncoderPosition();
	 //	cout << "g_dEncoderPos: "<<g_dEncoderPos<<" - dEncoderPos: " <<g_dEncoderPos<<endl;
	 //		if(dEncoderPos<g_dEncoderPos){
	 if(g_adapter->getRingIndicator()){

	 /			cout<<"g_adapter->getRingIndicator(): true"<<endl;
	 //			cout << "dEncoderPos: "<<dEncoderPos<<endl;
	 //			((CEposPMotorController*)g_motorController)->_offset=((CEposPMotorController*)g_motorController)->_timePeriod*dEncoderPos/((CEposPMotorController*)g_motorController)->_nNumberEncoderPosition;

	 ((CEposPMotorController*)g_motorController)->_timePeriod = (((CEposPMotorController*)g_motorController)->_timer)->reset();
	 cout<<"-------------------period: "<<((CEposPMotorController*)g_motorController)->_timePeriod<<endl;
	 //		printf("period: %f",g_timePeriod);
	 }
	 SLEEP(1);

	 //		g_dEncoderPos=dEncoderPos;
	 }
	 }
	 */

	CRotatingSICKLMS100::CRotatingSICKLMS100(CSickLms100Scanner* pScanner1, double dTilt, double nResolution, IMotorController* pMotorController, ISystemPoseAdapter* pSystemPoseAdapter) {
		init();
		setScanner(pScanner1, dTilt);
		setMotorController(pMotorController);
		setSystemPoseAdapter(pSystemPoseAdapter);

		std::cout << " CRotatingSICKLMS100::CRotatingSICKLMS100::pScanner1->getValuesPerScan() " << pScanner1->getValuesPerScan() << std::endl;

		_adRemission1 = new double[pScanner1->getValuesPerScan()];
		_adScan1 = new double[pScanner1->getValuesPerScan()];
		g_pdRemission1 = _adRemission1;
		g_pdScan1 = _adScan1;

		_nNrPointsVertical = pScanner1->getValuesPerScan();
		_nResolution = nResolution;

		//_dOffsetRotationPositionMarker=12.0*M_PI/180.0;
		//cout <<"_dOffsetRotationPositionMarker: "<< _dOffsetRotationPositionMarker<<endl;

		sleep(1);
		pthread_attr_init(&_attr);
		pthread_create(&_thScanner, &_attr, scannerCapture, NULL);
		sleep(1);
		//thread_args* args1 = (thread_args *) malloc (sizeof (thread_args));
		thread_args args1;
		args1.scanner = g_scanner1;
		args1.scannerOffset = 3;
		//args1->scanner = g_scanner1;
		//args1->scannerOffset = 2;
		/*
		 thread_args* args2 = (thread_args *) malloc (sizeof (thread_args));
		 args2->scanner = g_scanner2;
		 args2->scannerOffset = M_PI;
		 */
		//thread_args* args1 = {g_scanner1, 0};
		//thread_args* args2 = {g_scanner2, M_PI};

		//pthread_create( &_thScanner1, &_attr, singleScannerCapture, &args1);

		//	while(true){
		//		sleep(100);
		//	}
		//
		//pthread_create( &_thScanner2, &_attr, singleScannerCapture, &args2);


		//	pthread_create( &_thController, &_attr, controllerPos, NULL);
		//cout << "end of constructor" << endl;
	}

	/*
	 CRotatingSICKLMS100::CRotatingSICKLMS100() {
	 init();
	 _pScanner1 = NULL;
	 _pScanner2 = NULL;

	 }
	 */

	void CRotatingSICKLMS100::init() {
		_bConnected1 = false;
		//	_nApexAngle = 0;
		//	_nMaxApexAngle = 0;
		//	_nResolution = 0;
		//	_nMaxResolution = 0;
		setRotationAxis(1, 0, 0);
		setOrientation(0, 0, 0);
		_dTimeout = 1 / 50.0 * 1000 + 10;
		_nResolution = 1.0;
		_dTilt = 180;

	}

	CRotatingSICKLMS100::~CRotatingSICKLMS100() {
		if (_bConnected1)
			disconnect();

		if (_adRemission1 != NULL) {
			delete[] _adRemission1;
			_adRemission1 = NULL;
		}
		if (_adScan1 != NULL) {
			delete[] _adScan1;
			_adScan1 = NULL;
		}

		if (_adRemission2 != NULL) {
			delete[] _adRemission2;
			_adRemission2 = NULL;
		}
		if (_adScan2 != NULL) {
			delete[] _adScan2;
			_adScan2 = NULL;
		}

		// signal thread termination
		g_bRun = false;
		// wait for scanner termination
		while (g_bScannerExit == false)
			sched_yield();
	}

	void CRotatingSICKLMS100::setScanner(CSickLms100Scanner *pScanner1, double dTilt) {
		if (_bConnected1) {
			_pScanner1->disconnect();
			_pScanner1 = pScanner1;
			if (_pScanner1 != NULL)
				connect();
		} else {
			_pScanner1 = pScanner1;
		}
		_dTilt = dTilt * M_PI / 180;
		g_scanner1 = pScanner1;
	}

	void CRotatingSICKLMS100::setMotorController(IMotorController *pMotorController) {
		if (_pMotorController != NULL) {
			_pMotorController->disconnect();
			_pMotorController->connect();
		}

		_pMotorController = pMotorController;
		g_motorController = pMotorController;

		//check if it's needed connect() or ->connect() or nothing else...

	}

	void CRotatingSICKLMS100::setSystemPoseAdapter(ISystemPoseAdapter *pSystemPoseAdapter) {
		_pSystemPoseAdapter = pSystemPoseAdapter;
		g_systemPoseAdapter = pSystemPoseAdapter;

	}

	IScanner *CRotatingSICKLMS100::getScanner1() {
		return _pScanner1;
	}

	IMotorController *CRotatingSICKLMS100::getMotorController() {
		return _pMotorController;
	}

	ISystemPoseAdapter *CRotatingSICKLMS100::getSystemPoseAdapter() {
		return _pSystemPoseAdapter;
	}

	double CRotatingSICKLMS100::getResolution() {
		return _nResolution;
	}

	void CRotatingSICKLMS100::setResolution(int nResolution) {
		_nResolution = nResolution;
	}

	void CRotatingSICKLMS100::setResolution(double nResolution) {
		_nResolution = nResolution;
	}

	void CRotatingSICKLMS100::setTimeout(double dTimeout) {
		if (_pScanner1 == NULL) {
			if (dTimeout < 0.0)
				dTimeout = 0.0;
		} else {
			if (dTimeout < _pScanner1->getScanningTime())
				dTimeout = _pScanner1->getScanningTime();
		}

		_dTimeout = dTimeout;
	}

	double CRotatingSICKLMS100::getTimeout() {
		return _dTimeout;
	}

	double CRotatingSICKLMS100::getScanningTime() {
		if (_pScanner1 == NULL)
			return 0.0;

		int numberOfScanners = 1;
		return ((360.0 * (1.0 / _nResolution)) * _pScanner1->getScanningTime()) / numberOfScanners;
	}

	EnumUnit CRotatingSICKLMS100::getUnit() {
		if (_pScanner1 != NULL)
			return _pScanner1->getUnit();

		return eUnitMM;
	}

	EnumScanDirection CRotatingSICKLMS100::getDirection() {
		return _direction; //direction of servo
	}

	/*void CRotatingSICKLMS100::setDirection(EnumScanDirection direction) {
	 _direction = direction;
	 }*/

	void CRotatingSICKLMS100::setScannerApexAngle(int nApexAngle) {
		if (nApexAngle < 0)
			nApexAngle = 0;
		if (nApexAngle > _nMaxApexAngle)
			nApexAngle = _nMaxApexAngle;

		_nApexAngle = nApexAngle;
	}

	int CRotatingSICKLMS100::getScannerApexAngle() {
		return _nApexAngle;
	}

	void CRotatingSICKLMS100::getRotationAxis(double *pdX, double *pdY, double *pdZ) {
		if (pdX != NULL)
			*pdX = _rotation[0];
		if (pdY != NULL)
			*pdY = _rotation[1];
		if (pdZ != NULL)
			*pdZ = _rotation[2];
	}

	void CRotatingSICKLMS100::setRotationAxis(double dX, double dY, double dZ) {
		if (dX == 0.0 && dY == 0.0 && dZ == 0.0)
			return;

		_rotation[0] = dX;
		_rotation[1] = dY;
		_rotation[2] = dZ;

		_rotation.normalize();
	}

	void CRotatingSICKLMS100::getOrientation(double *pdPitch, double *pdYaw, double *pdRoll) {
		if (pdPitch != NULL)
			*pdPitch = _dPitch;
		if (pdYaw != NULL)
			*pdYaw = _dYaw;
		if (pdRoll != NULL)
			*pdRoll = _dRoll;
	}

	void CRotatingSICKLMS100::setOrientation(double dPitch, double dYaw, double dRoll) {
		_dPitch = dPitch;
		_dYaw = dYaw;
		_dRoll = dRoll;

		double sin_c = sin(dPitch);
		double cos_c = cos(dPitch);
		double sin_b = sin(dYaw);
		double cos_b = cos(dYaw);
		double sin_a = sin(dRoll);
		double cos_a = cos(dRoll);

		_pitchYawRollTransformMatrix[0][0] = cos_a * cos_b;
		_pitchYawRollTransformMatrix[0][1] = cos_a * sin_b * sin_c - sin_a * cos_c;
		_pitchYawRollTransformMatrix[0][2] = cos_a * sin_b * cos_c + sin_a * sin_c;
		_pitchYawRollTransformMatrix[0][3] = 0.0;

		_pitchYawRollTransformMatrix[1][0] = sin_a * cos_b;
		_pitchYawRollTransformMatrix[1][1] = sin_a * sin_b * sin_c + cos_a * cos_c;
		_pitchYawRollTransformMatrix[1][2] = sin_a * sin_b * cos_c - cos_a * sin_c;
		_pitchYawRollTransformMatrix[1][3] = 0.0;

		_pitchYawRollTransformMatrix[2][0] = -sin_b;
		_pitchYawRollTransformMatrix[2][1] = cos_b * sin_c;
		_pitchYawRollTransformMatrix[2][2] = cos_b * cos_c;
		_pitchYawRollTransformMatrix[2][3] = 0.0;

		_pitchYawRollTransformMatrix[3][0] = 0.0;
		_pitchYawRollTransformMatrix[3][1] = 0.0;
		_pitchYawRollTransformMatrix[3][2] = 0.0;
		_pitchYawRollTransformMatrix[3][3] = 1.0;
	}

	int CRotatingSICKLMS100::getValuesPerScan() {
		if (_pScanner1 == NULL) {
			return 0;
		}
		return _pScanner1->getValuesPerScan() * (360 * 1 / _nResolution);
	}

	int CRotatingSICKLMS100::connect() {
		if (_bConnected1 == true)
			return 0;
		if (_pScanner1 == NULL)
			return 0;

		if (_bConnected1 == false) {
			if (_pScanner1->connect() != 0) {
				_bConnected1 = true;
				//_bConnected2 = true;
			} else
				return 0;
		}

		return 1;
	}

	int CRotatingSICKLMS100::disconnect() {
		if (_bConnected1 == false)
			return 0;
		if (_pScanner1 == NULL)
			return 0;

		if (_pScanner1->disconnect() != 0) {
			_bConnected1 = false;
			return 1;
		}

		return 0;
	}

	unsigned long CRotatingSICKLMS100::getScan(CCartesianCloud3D *pCloud) {

		cout << "in getScan(CCartesianCloud3D *pCloud)" << endl;

		if (pCloud == NULL || pCloud->size() == 0)
			return 0;
		if (_pScanner1 == NULL)
			return 0;

		cout << "pcloud->size(): " << pCloud->size() << endl;

		//int i = 0;
		int j = 0;

		int nNrPointsHor = _pScanner1->getValuesPerScan(); //181 if we consider the complete scan
		cout << "nNrPointsHor: " << nNrPointsHor << endl;
		int nNrPointsVert = 360 / _nResolution;
		cout << "nNrPointsVert: " << nNrPointsVert << endl;
		int nNrValuesExp = nNrPointsHor * nNrPointsVert;

		double* pDist = new double[nNrPointsHor];
		double* pRems = new double[nNrPointsHor];

		//	double dVelocity  = VELOCITY / 89.0 / 60.0 /2.0;	// complete turns / s
		int numberOfScanners = 1;

		double dVelocity = _pMotorController->getVelocity() / numberOfScanners; // complete turns / s

		double dTime1turn = 1 / dVelocity; // time for 1 turn
		double dTime1scan = dTime1turn / nNrPointsVert; // time for 1 scan
		double dTimeDelta = dTime1scan / nNrPointsHor; // time between 2 different values in the same scan

		//delta = angle "covered" by the scanners between two consecutive measurements on the same scan
		double delta = dVelocity * dTimeDelta * 2 * M_PI;
		cout << "delta : " << dVelocity * dTimeDelta * 360 << endl;

		CTimer* t = new CTimer();

		//the first 120 scans are not good
		/*	for(int i=0; i<120; i++){
		 //cout << "in for to discard the bad scans" << endl;
		 _pScanner1->getScan(pDist,pRems);
		 while(t->getTime()<13);
		 cout << t->reset() << endl;
		 _pScanner2->getScan(pDist,pRems);

		 }
		 return 0;
		 */
		int nNrValues = 0;
		//	int nValues =0;

		cout << "reset timer" << endl;
		t->reset();
		for (j = 0; j < nNrPointsVert / 2/*45*/; j++) {
			nNrValues += get1Scan(pCloud, pDist, pRems, 1, j, nNrPointsHor, delta);
			/*cout << endl;
			 for(int i=0;i<181;i++)
			 cout << pDist[i] << " ";
			 */
			nNrValues += get1Scan(pCloud, pDist, pRems, 2, j, nNrPointsHor, delta);
			//cout << endl;
			/*for(i=0;i<181;i++)
			 cout << pDist[i] << " ";
			 */
			//break;
		}
		cout << "time - in getScan(): " << t->getTime() << endl;

		delete pDist;
		delete pRems;
		delete t;
		return nNrValues;

	}

	unsigned long CRotatingSICKLMS100::get1Scan(CCartesianCloud3D *pCloud, double *pDist, double *pRems, int nScanner, int j, int nNrPointsHor, double delta) {

		if (nScanner != 1 && nScanner != 2) {
			cerr << "scanner is only 1 or 2!" << endl;
			exit(1);
		}

		int nValues;
		int nNrValues = 0;
		int nPoint;
		double beta;
		double alfa;

		unsigned int i_start, i_end;
		i_start = 0;
		i_end = _pScanner1->getValuesPerScan();
		//_pScanner1->getRemissionRange(&i_start, &i_end);
		//cout<< "i_start: " << i_start << endl;
		//cout<< "i_end: " << i_end << endl;

		if (nScanner == 1) {

			nPoint = j * nNrPointsHor;
			beta = j * _nResolution * M_PI / 180;
			//cout << endl << "beta: "<< j*_nResolution << endl;
			nValues = _pScanner1->getScan(pDist, pRems);
		}

		if (nValues == nNrPointsHor) {

			for (int i = 0; i < nNrPointsHor/*20*/; i++, nPoint++, beta += delta) {

				//cout<<"i: " << i << endl;
				//	cout << "alfa: " << i << " ; ";
				// "+8": 8cm - we consider the offset between the two scanners
				StrCartesianPoint3D* point = pCloud->getPoint(nPoint);

				alfa = (i + i_start) * M_PI / 180;
				//	point->dX=(pDist[i]*sin((i+i_start)*M_PI/180)+8)*cos(beta)/100;
				//	point->dY=(pDist[i]*sin((i+i_start)*M_PI/180)+8)*sin(beta)/100;
				//	point->dZ= pDist[i]*cos((i+i_start)*M_PI/180)/100;

				point->dX = ((pDist[i] * sin(alfa) + 10) * cos(beta) + pDist[i] * cos(alfa) * sin(_dTilt) * sin(beta)) / 100;
				point->dY = ((pDist[i] * sin(alfa) + 10) * sin(beta) - pDist[i] * cos(alfa) * sin(_dTilt) * cos(beta)) / 100;
				point->dZ = pDist[i] * cos(alfa) * cos(_dTilt) / 100;

				StrPointInfo* info = pCloud->getInfo(nPoint);
				info->afRGB[0] = 255.0;
				info->afRGB[1] = 255.0;
				info->afRGB[2] = 255.0;
				info->fIntensity = pRems[i] / 32767;
				info->bValid = true;

				info->dDistance = pDist[i];
				info->dAlpha = alfa;
				info->dBeta = beta;

				if (pDist[i] >= 8183 && pDist[i] <= 8191)
					info->bValid = false;
				if (pDist[i] >= 16378 && pDist[i] <= 16385)
					info->bValid = false;
				if (pDist[i] >= 32759)
					info->bValid = false;

			}
			nNrValues += nValues;
		}
		return nNrValues;
	}

	//thread function

	unsigned long CRotatingSICKLMS100::thread_getScan(CCartesianCloud3D *pCloud) {

		//cout << "in thread_getScan(CCartesianCloud3D *pCloud)" << endl;

		CTimer* t = new CTimer();

		//int nValues;
		int nNrValues = 0;
		//int nPoint;
		double alfa;
		//double beta = _pMotorController->getPosition();

		//	cout << "_pMotorController->getRateVelocity(): " <<_pMotorController->getRateVelocity() <<endl;
		//	cout << "_nResolution: " << _nResolution << "----------- _nNrPointsHor: " << _nNrPointsHor << endl;
		//double delta = (_pMotorController->getRateVelocity() * 60.0 * 2.0 / (360.0 / _nResolution * _nNrPointsVertical)) * M_PI / 180;

		//	cout << "(_pMotorController->getRateVelocity()*60.0*2.0) / (360.0/_nResolution * _nNrPointsHor)" << (_pMotorController->getRateVelocity()*60.0*2.0) / (360.0/_nResolution * _nNrPointsHor) << endl;

		//scanning time is the half -> mirror
		//		delta = delta / 2.0;
		//		delta = delta / 2.0;
		//	cout << "delta: " << delta<< endl;

		unsigned int i_start, i_end;
		unsigned int ratio;
		i_start = 1;
		i_end = _pScanner1->getValuesPerScan();
		ratio = 1;
		//	_pScanner1->getRemissionRange(&i_start, &i_end);

		//clean the queue - intereseting on actual values, not past
		//while(!g_queueScans.empty()) g_queueScans.pop();


		//This computation is not generically correct, but a god approximation
		//int HorizontalScansPerCloud = static_cast<int>(360*_nResolution);

		//Use the period for one turn to compute the correct numbers of Horizontal Scans plus a small buffer
		//int HorizontalScansPerCloud = static_cast<int> (_pMotorController->getRotationPeriod() / 13.33) + 2;
		int HorizontalScansPerCloud = static_cast<int> (_pMotorController->getRotationPeriod() / _pScanner1->getScanningTime()) + 2;
		for (int j = 0; j < HorizontalScansPerCloud/*1*/; j++) {
			//		cout << "j: " << j << endl;
			while (g_queueScans.empty()) {
				//cout << "queue empty" << endl;
				usleep(10);
			}

			CRotatingSICKLMS100::SScan scan = g_queueScans.front();
			g_queueScans.pop();
			double beta = scan.dMotorControllerPos;

			CMatrix44 posMatrix = scan.systemPosMatrix;

			for (int i = 0; i < _nNrPointsVertical; i++) {
				StrCartesianPoint3D* point = pCloud->getPoint(j * _nNrPointsVertical + i);

				switch (_pScanner1->getResolution()) {
					case fair::eRes1Degree: {
						ratio = 1;
						break;
					}
					case fair::eRes05Degree: {
						ratio = 2;
						break;
					}
					case fair::eRes025Degree: {
						ratio = 4;
						break;
					}
				}
				//ratio = 2;
				alfa = (i + i_start + (45 * ratio)) * M_PI / 180;
				alfa = alfa / ratio;

				if (point == NULL)
					cout << "point null" << endl;

				//transformation taking in account the position in a single scan, the rotation from the motorcontroller and the tilting of the scanners
				double dX = ((scan.pdDist[i] * sin(alfa) /*+ 10*/) * cos(beta) + scan.pdDist[i] * cos(alfa) * sin(_dTilt) * sin(beta)) / 1000;
				double dY = ((scan.pdDist[i] * sin(alfa) /*+ 10*/) * sin(beta) - scan.pdDist[i] * cos(alfa) * sin(_dTilt) * cos(beta)) / 1000;
				double dZ = scan.pdDist[i] * cos(alfa) * cos(_dTilt) / 1000;

				//	cout << "scanner reference system   X: " << dX << " - Y: " << dY << " - Z: " << dZ << endl;

				//transformation considering the position and orientation of the entire system (car)
				point->dX = posMatrix[0][0] * dX + posMatrix[0][1] * dY + posMatrix[0][2] * dZ + posMatrix[0][3];
				point->dY = posMatrix[1][0] * dX + posMatrix[1][1] * dY + posMatrix[1][2] * dZ + posMatrix[1][3];
				point->dZ = posMatrix[2][0] * dX + posMatrix[2][1] * dY + posMatrix[2][2] * dZ + posMatrix[2][3];
				double om_coord = posMatrix[3][0] * dX + posMatrix[3][1] * dY + posMatrix[3][2] * dZ + posMatrix[3][3];

				if (om_coord != 1) {
					point->dX = point->dX / om_coord;
					point->dY = point->dY / om_coord;
					point->dZ = point->dZ / om_coord;
					//om_coord = 1;
				}

				//cout << "system reference system    X: " << point->dX << " - Y: " << point->dY << " - Z: " << point->dZ << endl;


				StrPointInfo* info = pCloud->getInfo(j * _nNrPointsVertical + i);
				info->afRGB[0] = 255.0;
				info->afRGB[1] = 255.0;
				info->afRGB[2] = 255.0;
				//info->fIntensity = scan.pdRems[i]/32767;
				info->bValid = true;
				info->dDistance = scan.pdDist[i];
				info->dAlpha = alfa;
				info->dBeta = beta;

				//info->fIntensity = 10000;//(scan.pdRems[i] - 7900) / 3000.0;
				info->fIntensity = scan.pdRems[i];

				//TODO CHECKING this function. Looks like it drops a lot of values including all over 32 meters
				//			if (scan.pdDist[i] >= 8183 && scan.pdDist[i] <= 8191)
				//				info->bValid = false;
				//			if (scan.pdDist[i] >= 16378 && scan.pdDist[i] <= 16385)
				//				info->bValid = false;
				//			if (scan.pdDist[i] >= 32759)
				//				info->bValid = false;

				nNrValues += _nNrPointsVertical;
			}

		}
		//cout << "Number of elements in queue: " << g_queueScans.size() << endl;
		//cout << "Time reading queue and building cloud: " << t->reset();
		//delete t;
		return nNrValues;
	}

	void CRotatingSICKLMS100::analyzeScan(CRotatingSICKLMS100::SScan scan2, CCartesianCloud3D *pCloud, int j, double delta) {
		//cout<<"analzing scans..."<<endl;
		CMatrix44 posMatrix = scan2.systemPosMatrix;
		double beta = scan2.dMotorControllerPos;
		double alfa = 0;

		unsigned int i_start, i_end;
		i_start = 0;
		i_end = 720;
		//	_pScanner1->getRemissionRange(&i_start, &i_end);

		for (int i = 0; i < _nNrPointsVertical; i++, /*nPoint++,*/beta += delta) {

			StrCartesianPoint3D* point = pCloud->getPoint(j * _nNrPointsVertical + i);

			alfa = (i + i_start) * M_PI / 180;

			//	cout << "scan.pdDist[" << i << "]: " << scan.pdDist[i] << " ";
			//	cout << "- alfa: " << i+i_start << endl;


			//transformation taking in account the position in a single scan, the rotation from the motorcontroller and the tilting of the scanners
			double dX = ((scan2.pdDist[i] * sin(alfa) + 10) * cos(beta) + scan2.pdDist[i] * cos(alfa) * sin(_dTilt) * sin(beta)) / 100;
			double dY = ((scan2.pdDist[i] * sin(alfa) + 10) * sin(beta) - scan2.pdDist[i] * cos(alfa) * sin(_dTilt) * cos(beta)) / 100;
			double dZ = scan2.pdDist[i] * cos(alfa) * cos(_dTilt) / 100;

			//	cout << "scanner reference system   X: " << dX << " - Y: " << dY << " - Z: " << dZ << endl;

			//transformation considering the position and orientation of the entire system (car)
			point->dX = posMatrix[0][0] * dX + posMatrix[0][1] * dY + posMatrix[0][2] * dZ + posMatrix[0][3];
			point->dY = posMatrix[1][0] * dX + posMatrix[1][1] * dY + posMatrix[1][2] * dZ + posMatrix[1][3];
			point->dZ = posMatrix[2][0] * dX + posMatrix[2][1] * dY + posMatrix[2][2] * dZ + posMatrix[2][3];
			double om_coord = posMatrix[3][0] * dX + posMatrix[3][1] * dY + posMatrix[3][2] * dZ + posMatrix[3][3];

			if (om_coord != 1) {
				point->dX = point->dX / om_coord;
				point->dY = point->dY / om_coord;
				point->dZ = point->dZ / om_coord;
				//om_coord = 1;
			}

			//	cout << "system reference system    X: " << point->dX << " - Y: " << point->dY << " - Z: " << point->dZ << endl;


			StrPointInfo* info = pCloud->getInfo(j * _nNrPointsVertical + i);
			info->afRGB[0] = 255.0;
			info->afRGB[1] = 255.0;
			info->afRGB[2] = 255.0;
			info->fIntensity = scan2.pdRems[i] / 32767;
			info->bValid = true;
			info->dDistance = scan2.pdDist[i];
			info->dAlpha = alfa;
			info->dBeta = beta;

			if (scan2.pdDist[i] >= 8183 && scan2.pdDist[i] <= 8191)
				info->bValid = false;
			if (scan2.pdDist[i] >= 16378 && scan2.pdDist[i] <= 16385)
				info->bValid = false;
			if (scan2.pdDist[i] >= 32759)
				info->bValid = false;

		}
	}

}
