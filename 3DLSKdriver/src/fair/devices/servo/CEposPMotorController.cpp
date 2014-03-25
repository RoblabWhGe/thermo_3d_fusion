#define nNumberEncoderPosition 4300

#include "CEposPMotorController.h"
#include "math.h"
namespace fair {

CTimer* g_timer;
double g_timePeriod = 2400;
//double g_dEncoderPos=0;
//double g_offset=0;
bool b_init = false;
CComAdapter* g_adapter;

void* encoderTimerFunction(void* arg) {
	int test=0;
    //cout << "Thread of CEposPMotorController"<<endl;
    //	if(b_init){
    while (true) {
		//	CEposPMotorController* mc = (CEposPMotorController*) arg;
        //	double dEncoderPos=mc->getEncoderPosition();
        //	cout << "g_dEncoderPos: "<<g_dEncoderPos<<" - dEncoderPos: " <<g_dEncoderPos<<endl;
        //	cout << "g_adapter->getReceiveMode(): " << g_adapter->getReceiveMode()<<endl;


        if (g_adapter->getRingIndicator()) {
            //cout << "dEncoderPos: "<<dEncoderPos<<endl;
            //g_offset=g_timePeriod*dEncoderPos/nNumberEncoderPosition;
            g_timePeriod = g_timer->reset();
        }

        //	g_dEncoderPos=dEncoderPos;
    }
}

/**
 * standard constructor
 * @param adapter com adapter
 */
CEposPMotorController::CEposPMotorController(CComAdapter* adapter) {
    //cout << "in the constructot of CEposPMotorController"<<endl;
    _adapter = adapter;
    _rateVelocity = 89;

    //_timerSetTime = new CTimer();


    _timer = new CTimer();
    _timePeriod = 2400;
    g_adapter = adapter;

    _offset = 0;
    _nNumberEncoderPosition = nNumberEncoderPosition;

    g_timer = _timer;
    _dOffsetRotationPositionMarker = 12.0 * M_PI / 180.0;

    pthread_attr_init(&_attr);
    pthread_create(&_thController, &_attr, encoderTimerFunction, NULL);
    //cout << "end of constructot of CEposPMotorController"<<endl;

}

/**
 * standard destructor. does nothing
 */
CEposPMotorController::~CEposPMotorController() {
}

int CEposPMotorController::disconnect() {

    char acTempString[18];
    char acBuffer[256];
    int nRead;
    int nOpen;
    int nSend;

    acTempString[0] = 0xF0;
    acTempString[1] = 0x06;
    acTempString[2] = 0x00;
    acTempString[3] = 0x00;
    acTempString[4] = 0x00;
    acTempString[5] = 0x06;
    acTempString[6] = 0x00;
    acTempString[7] = 0xFF;
    acTempString[8] = 0xD1;
    acTempString[9] = 0x01;
    acTempString[10] = 0x00;
    acTempString[11] = 0x00;
    acTempString[12] = 0x01;
    acTempString[13] = 0x16;
    acTempString[14] = 0x02;
    acTempString[15] = 0x00;
    acTempString[16] = 0x00;
    acTempString[17] = 0x06;

    nRead = 0;
    nSend = _adapter->send(acTempString, 18);

    nRead += _adapter->receive(acBuffer, 16);

    acTempString[0] = 0xF0;
    acTempString[1] = 0x04;
    acTempString[2] = 0x00;
    acTempString[3] = 0x00;
    acTempString[4] = 0x00;
    acTempString[5] = 0x04;
    acTempString[6] = 0x00;
    acTempString[7] = 0xFF;
    acTempString[8] = 0x75;
    acTempString[9] = 0xB1;
    acTempString[10] = 0x00;
    acTempString[11] = 0x00;
    acTempString[12] = 0x01;
    acTempString[13] = 0x0A;
    acTempString[14] = 0x00;
    acTempString[15] = 0x00;

    nRead = 0;
    nSend = _adapter->send(acTempString, 16);

    nRead += _adapter->receive(acBuffer, 13);

    acTempString[0] = 0xF0;
    acTempString[1] = 0x04;
    acTempString[2] = 0x00;
    acTempString[3] = 0x00;
    acTempString[4] = 0x00;
    acTempString[5] = 0x04;
    acTempString[6] = 0x00;
    acTempString[7] = 0xFF;
    acTempString[8] = 0xD4;
    acTempString[9] = 0x18;
    acTempString[10] = 0x00;
    acTempString[11] = 0x00;
    acTempString[12] = 0x01;
    acTempString[13] = 0x02;
    acTempString[14] = 0x00;
    acTempString[15] = 0x00;

    nRead = 0;
    nSend = _adapter->send(acTempString, 16);

    nRead += _adapter->receive(acBuffer, 18);

    acTempString[0] = 0xF0;

    nRead = 0;
    nSend = _adapter->send(acTempString, 1);

    cout << "closing device.." << endl;

    CTimer *timer = new CTimer();
    timer->reset();
    while (timer->getTime() < 1000)
        ;

    _adapter->closeDevice();

    timer->reset();
    while (timer->getTime() < 6000)
        ;
    delete g_timer;
    return 0;
}

int CEposPMotorController::connect() {
    init();
    //	g_dEncoderPos=getEncoderPosition();
    //	b_init=true;
    return 0;
}

double CEposPMotorController::getPosition() {
    //	cout << "(_timerSetTime->getTime()-g_offset) / g_timePeriod * M_PI: " <<(_timerSetTime->getTime()-g_offset) / g_timePeriod * M_PI<<endl;
    return (_timer->getTime()/*-_offset*/) / g_timePeriod * 2 * M_PI /*+ _dOffsetRotationPositionMarker*/;
    //return (double)getEncoderPosition()/nNumberEncoderPosition * M_PI * 2;
    //return (double)getEncoderPosition();
}
/*
 double CEposPMotorController::getPosition2(){
 //	cout << "(_timerSetTime->getTime()-g_offset) / g_timePeriod * M_PI: " <<(_timerSetTime->getTime()-g_offset) / g_timePeriod * M_PI<<endl;
 return ((_timer->getTime()+g_timePeriod/2/*-_offset*) / g_timePeriod * 2*M_PI + _dOffsetRotationPositionMarker;
 //return (double)getEncoderPosition()/nNumberEncoderPosition * M_PI *  2;
 //return (double)getEncoderPosition();
 }
 */

double CEposPMotorController::getRotationPeriod() {
    return g_timePeriod;
}

IDeviceAdapter* CEposPMotorController::getDeviceAdapter() {
    return _adapter;
}



void CEposPMotorController::setVelocity(float fVelocity) {
    //TO DO
}

float CEposPMotorController::getVelocity() {
    //TO DO
    //return 2200;
	return 1.0/(g_timePeriod/1000.0);
}

int CEposPMotorController::getRateVelocity() {
    return _rateVelocity;
}

int CEposPMotorController::getEncoderPosition() {

    char acTempString[18];
    char acBuffer[256];
    int nRead;
    int nOpen;
    int nSend;

    acTempString[0] = 0xF0;
    acTempString[1] = 0x06;
    acTempString[2] = 0x00;
    acTempString[3] = 0x00;
    acTempString[4] = 0x00;
    acTempString[5] = 0x06;
    acTempString[6] = 0x00;
    acTempString[7] = 0xFF;
    acTempString[8] = 0xBE;
    acTempString[9] = 0x17;
    acTempString[10] = 0x00;
    acTempString[11] = 0x00;
    acTempString[12] = 0x01;
    acTempString[13] = 0x20;
    acTempString[14] = 0x02;
    acTempString[15] = 0xFF;
    acTempString[16] = 0x7F;
    acTempString[17] = 0x06;

    CTimer *timer = new CTimer();
    CTimer *timer2 = new CTimer();
    CTimer *rotationTimer = new CTimer();
    timer->reset();
    rotationTimer->reset();

    bool marker = false;

    timer->reset();

    nRead = 0;
    nSend = _adapter->send(acTempString, 18);
    timer2->reset();

    nRead += _adapter->receive(acBuffer, 22);

    // verify answer message


    while (timer2->getTime() < 10)
        ;
    if ((nRead >= 22) && (true)) {

        if ((((int) ((unsigned char)(acBuffer[0]))) == 240) && (((int)((unsigned char)(acBuffer[1]))) == 13) && (((int)((unsigned char)(acBuffer[9]))) == 3) && (((int)((unsigned char)(acBuffer[10]))) == 0) && (((int)((unsigned char)(acBuffer[11]))) == 9) && (((int)((unsigned char)(acBuffer[12]))) == 0) && (((int)((unsigned char)(acBuffer[13]))) == 6) && (((int)((unsigned char)(acBuffer[14]))) == 6)){
                return ((int)((unsigned char)(acBuffer[19]))) * pow(2.0, 8.0) + ((int)((unsigned char)(acBuffer[18])));
            }else{
                cout << "bad message, have to renitialize" << endl;
                this->disconnect();
                this->connect();
            }
        }
        else{
            cout << " --- read " << nRead << endl;
            //this->disconnect();
            //this->connect();
        }
        return 0;
    }

    void CEposPMotorController::setDeviceAdapter(IDeviceAdapter *pAdapter)
    {
    	// to implement IMotorController
    	if(pAdapter->getAdapterType()==fair::eCOM)
    	{
    		_adapter = (CComAdapter*)pAdapter;
    	}
    }

int CEposPMotorController::init() {

    char acSetString[16];
    char acTempString[18];
    char acBuffer[256];
    int nRead;
    int nOpen = FAIR_EXIT_FAILURE;
    int nSend;

    _adapter->setBaudrate(eB115200);

    nOpen = _adapter->openDevice();

    if (FAIR_EXIT_FAILURE == nOpen) {
       cerr <<"Failed to open Device on port " << _adapter->getDeviceName() <<endl;
        return 2;
    }
    return 0;
}

//int main(int argc, char** argv) {
//
//    //	printf("test");
//    // Get scanner instance from factory
//    //CDeviceFactory* factory = new CDeviceFactory("C:\test.conf");
//    //g_scanner3D = factory->get3DScanner();
//
//
//}
}
