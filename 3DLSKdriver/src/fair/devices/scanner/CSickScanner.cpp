#include "CSickScanner.h"

namespace fair {

#define CRC16_GEN_POL 0x8005
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))

static unsigned short CreateCRC(unsigned char* ucCommData, unsigned int unLen) {
    unsigned short usCrc16;
    unsigned char ucAbData[2];

    usCrc16 = 0;
    ucAbData[0] = 0;
    while (unLen--) {
        ucAbData[1] = ucAbData[0];
        ucAbData[0] = *ucCommData++;
        if (usCrc16 & 0x8000) {
            usCrc16 = (usCrc16 & 0x7fff) << 1;
            usCrc16 ^= CRC16_GEN_POL;
        } else {
            usCrc16 <<= 1;
        }
        usCrc16 ^= MKSHORT(ucAbData[0], ucAbData[1]);
    }
    return (usCrc16);
}

CSickScanner::CSickScanner(IDeviceAdapter* pAdapter) {
    _bConnected = false;
    setDeviceAdapter(pAdapter);
    _eOperationMode = eOpOperating;
    _eTransmissionMode = eTransRequestSending;
    _eResolution = eRes1Degree;
    _eApexAngle = eApex180Degree;
    _unRemissionStart = 1;
    _unRemissionEnd = 181;
    _nScanParts = 1;

    _ucDistMask = 0x1f;

    _szTelegram = new unsigned char[MAX_TELEGRAM_LENGTH];
    _szAnswer = new unsigned char[MAX_TELEGRAM_LENGTH];
    _timer = new CTimer();
    _pszBuffer = new char[1024];
    _nMessageLen = 0;
    _bMsgComplete = true;
    _bMsgCorrect = false;
    _nScanPartsAcquired = 0;

    memcpy(_aucMsgSetupBaudrate, g_aucMsgSetupBaudrate, sizeof(g_aucMsgSetupBaudrate));
    memcpy(_aucMsgSetupConfigMode, g_aucMsgSetupConfigMode, sizeof(g_aucMsgSetupConfigMode));
    memcpy(_aucMsgSetupTrans, g_aucMsgSetupTrans, sizeof(g_aucMsgSetupTrans));
    memcpy(_aucMsgSetupTransRem, g_aucMsgSetupTransRem, sizeof(g_aucMsgSetupTransRem));
    memcpy(_aucAnswSetup, g_aucAnswSetup, sizeof(g_aucAnswSetup));
    memcpy(_aucMsgOpDataRequest, g_aucMsgOpDataRequest, sizeof(g_aucMsgOpDataRequest));
    memcpy(_aucMsgVariant, g_aucMsgVariant, sizeof(g_aucMsgVariant));
    memcpy(_aucAnswVariant, g_aucAnswVariant, sizeof(g_aucAnswVariant));
    memcpy(_aucMsgTrans, g_aucMsgTrans, sizeof(g_aucMsgTrans));
    memcpy(_aucAnswTrans, g_aucAnswTrans, sizeof(g_aucAnswTrans));
}

CSickScanner::~CSickScanner() {
    if (_bConnected) disconnect();
    delete[] _pszBuffer;
}

int CSickScanner::connect() {
    int nRetVal = FAIR_EXIT_FAILURE;

    if (_bConnected) return FAIR_EXIT_SUCCESS;
    /* Device is already connected but returning 1 (success) because
     * the postconditions of connect() are fulfilled.
     */

    if (_pAdapter == NULL) return FAIR_EXIT_FAILURE;

    if (_pAdapter->getAdapterType() == eCOM) {
        //try to communicate with 9600 Baud
        ((CComAdapter*) _pAdapter)->setBaudrate(eB9600);
        ((CComAdapter*) _pAdapter)->openDevice();
        ((CComAdapter*) _pAdapter)->clearReceiveBuffer();

        // then switch to 500k Baud
        _aucMsgSetupBaudrate[1] = BAUD500K;
        sendTelegram(_aucMsgSetupBaudrate, sizeof(_aucMsgSetupBaudrate), _aucAnswSetup, sizeof(_aucAnswSetup), 100, 10);
        ((CComAdapter*) _pAdapter)->closeDevice();
        ((CComAdapter*) _pAdapter)->setBaudrate(eB500000);
        nRetVal = ((CComAdapter*) _pAdapter)->openDevice();
    } else {
        //not supported AdapterType
        return FAIR_EXIT_FAILURE;
    }

    if (nRetVal == FAIR_EXIT_SUCCESS) _bConnected = true;

    return nRetVal;
}

int CSickScanner::disconnect() {
    int nRetVal = FAIR_EXIT_FAILURE;

    if (!_bConnected) return FAIR_EXIT_SUCCESS;
    /* Device is not connected but returning 1 (success) because
     * the postconditions of disconnect() are fulfilled.
     */

    if (_pAdapter == NULL) return FAIR_EXIT_FAILURE;

    // stop continuous sending
    setOperationMode(eOpConfiguration);

    FAIR_SLEEP(50);

    if (_pAdapter->getAdapterType() == eCOM) {
        // switch back to 9600 Baud
        _aucMsgSetupBaudrate[1] = BAUD9600;
        nRetVal = sendTelegram(_aucMsgSetupBaudrate, sizeof(_aucMsgSetupBaudrate), _aucAnswSetup, sizeof(_aucAnswSetup), 100, 10);
    } else {
        //not supported AdapterType
        return FAIR_EXIT_FAILURE;
    }

    if (nRetVal == FAIR_EXIT_SUCCESS) {
        _bConnected = false;
        ((CComAdapter*) _pAdapter)->closeDevice();
        ((CComAdapter*) _pAdapter)->setBaudrate(eB9600);
    }

    return nRetVal;
}

void CSickScanner::setDeviceAdapter(IDeviceAdapter *pAdapter) {
    if (_bConnected) {
        disconnect();
        _pAdapter = pAdapter;
        if (_pAdapter != NULL) connect();
    } else {
        _pAdapter = pAdapter;
    }
}

IDeviceAdapter *CSickScanner::getDeviceAdapter() {
    return _pAdapter;
}

int CSickScanner::setApexAngle(EnumApexAngle eApexAngle) {
    int nRetVal = FAIR_EXIT_FAILURE;

    unsigned char ucApex = APEX180;
    switch (eApexAngle) {
        case eApex180Degree:
            ucApex = APEX180;
            break;
        case eApex100Degree:
            ucApex = APEX100;
            break;
    }

    _aucMsgVariant[1] = ucApex;
    _aucAnswVariant[2] = ucApex;

    nRetVal = sendTelegram(_aucMsgVariant, sizeof(_aucMsgVariant), _aucAnswVariant, sizeof(_aucAnswVariant), 100, 10);

    if (nRetVal == FAIR_EXIT_SUCCESS) _eApexAngle = eApexAngle;
    ;

    return nRetVal;
}

EnumApexAngle CSickScanner::getApexAngle() {
    return _eApexAngle;
}

int CSickScanner::getScannerApexAngle() {
    if ((_eTransmissionMode == eTransInterlaced) && ((int) _eApexAngle == 100)) return 180;
    // consider remission range
    if (_eTransmissionMode == eTransRemission) {
        return (_unRemissionEnd - _unRemissionStart) / (int) _eResolution;
    }
    return (int) _eApexAngle;
}

int CSickScanner::setResolution(EnumResolution eResolution) {
    int nRetVal = FAIR_EXIT_FAILURE;

    unsigned char ucRes = RES1;
    switch (eResolution) {
        case eRes1Degree:
            ucRes = RES1;
            if (_eTransmissionMode == eTransInterlaced) _nScanParts = 1;
            break;
        case eRes05Degree:
            ucRes = RES05;
            if (_eTransmissionMode == eTransInterlaced) _nScanParts = 2;
            break;
        case eRes025Degree:
            ucRes = RES025;
            if (_eTransmissionMode == eTransInterlaced) _nScanParts = 4;
            break;
    }

    _aucMsgVariant[3] = ucRes;
    _aucAnswVariant[4] = ucRes;

    nRetVal = sendTelegram(_aucMsgVariant, sizeof(_aucMsgVariant), _aucAnswVariant, sizeof(_aucAnswVariant), 100, 10);

    if (nRetVal == FAIR_EXIT_SUCCESS) _eResolution = eResolution;
    return nRetVal;
}

EnumResolution CSickScanner::getResolution() {
    return _eResolution;
}

int CSickScanner::getScannerResolution() {
    return (int) _eResolution * 360;
}

double CSickScanner::getScanningTime() {
    double dRetval = 0.0;

    switch (_eResolution) {
        case eRes1Degree:
            dRetval = 13.32;
            if (_eTransmissionMode == eTransRemission) {
                if (_unRemissionEnd - _unRemissionStart + 1 > 123) dRetval = 26.64;
            }
            break;
        case eRes05Degree:
            dRetval = 26.64;
            break;
        case eRes025Degree:
            dRetval = 53.28;
            break;
    }
    return dRetval;
}

int CSickScanner::setUnit(EnumUnit eUnit) {
    int nRetval = FAIR_EXIT_FAILURE;

    unsigned char ucUnit = UNITMM;

    switch (eUnit) {

        case eUnitMM:
            ucUnit = UNITMM;
            break;

        case eUnitCM:
            ucUnit = UNITCM;
            break;
    }

    _aucMsgTrans[7] = ucUnit;
    _aucAnswTrans[8] = ucUnit;

    if (_eOperationMode != eOpOperating) {
        nRetval = sendTelegram(_aucMsgTrans, sizeof(_aucMsgTrans), _aucAnswTrans, sizeof(_aucAnswTrans), 1000, 15);
    }

    if (FAIR_EXIT_SUCCESS == nRetval) {

        _eUnit = eUnit;

    }

    return nRetval;
}

EnumUnit CSickScanner::getUnit() {
    return _eUnit;
}

EnumScanDirection CSickScanner::getDirection() {
    return eCLOCKWISE;
}

int CSickScanner::getMaxRange(int *pnStart, int *pnStop) {
    int nStart = 0;
    int nStop = 720;

    if (getApexAngle() == eApex100Degree) {
        nStart = 160;
        nStop = 560;
    }

    if (pnStart != NULL) *pnStart = nStart;
    if (pnStop != NULL) *pnStop = nStop;

    return nStop - nStart + 1;
}

int CSickScanner::getRange(int *pnStart, int *pnStop) {
    if (_eTransmissionMode == eTransRemission) {
        if (pnStart != NULL) *pnStart = (int) _unRemissionStart;
        if (pnStop != NULL) *pnStop = (int) _unRemissionEnd;
        return _unRemissionEnd - _unRemissionStart + 1;
    } else {
        return getMaxRange(pnStart, pnStop);
    }
}

int CSickScanner::setRange(int nStart, int nStop) {
    return FAIR_EXIT_FAILURE;
}

int CSickScanner::getValuesPerScan() {
    if (_eTransmissionMode == eTransRemission) {
        return _unRemissionEnd - _unRemissionStart + 1;
    } else {
        return (getScannerApexAngle() * getScannerResolution() / 360) + 1;
    }
}

int CSickScanner::setRemissionRange(unsigned int unStart, unsigned int unEnd) {
    int nRetval = FAIR_EXIT_FAILURE;
    if ((unStart < unEnd) & (unEnd < 401)) {
        if ((unEnd - unStart) < 200) {
            _unRemissionStart = unStart;
            _unRemissionEnd = unEnd;
            _aucMsgSetupTransRem[4] = (char) (unStart & 0xff);
            _aucMsgSetupTransRem[5] = (char) (unStart / 256);
            _aucMsgSetupTransRem[6] = (char) (unEnd & 0xff);
            _aucMsgSetupTransRem[7] = (char) (unEnd / 256);

            if ((_eTransmissionMode == eTransRemission) && (_eOperationMode == eOpOperating)) {
                nRetval = sendTelegram(_aucMsgSetupTransRem, sizeof(_aucMsgSetupTransRem), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
            } else {
                nRetval = FAIR_EXIT_SUCCESS;
            }
        }
    }
    return nRetval;
}

void CSickScanner::getRemissionRange(unsigned int* unStart, unsigned int* unEnd) {
    *unStart = _unRemissionStart;
    *unEnd = _unRemissionEnd;
}

int CSickScanner::setOperationMode(EnumOperationMode eMode) {
    int nRetVal = FAIR_EXIT_FAILURE;
    switch (eMode) {
        case eOpConfiguration:
            nRetVal = sendTelegram(_aucMsgSetupConfigMode, sizeof(_aucMsgSetupConfigMode), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
            break;

        case eOpOperating:
            nRetVal = sendTelegram(_aucMsgTrans, sizeof(_aucMsgTrans), _aucAnswTrans, sizeof(_aucAnswTrans), 300, 10);
            // Handle different telegrams for data acquisition with or without remission values
            if (_eTransmissionMode == eTransRemission) {
                nRetVal = sendTelegram(_aucMsgSetupTransRem, sizeof(_aucMsgSetupTransRem), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
            } else {
                nRetVal = sendTelegram(_aucMsgSetupTrans, sizeof(_aucMsgSetupTrans), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
            }

            break;
    }

    if (nRetVal == FAIR_EXIT_SUCCESS) _eOperationMode = eMode;

    return nRetVal;
}

EnumOperationMode CSickScanner::getOperationMode() {
    return _eOperationMode;
}

int CSickScanner::setTransmissionMode(EnumTransmissionMode eMode) {
    int nRetVal = FAIR_EXIT_FAILURE;
    _nScanParts = 1;
    switch (eMode) {
        case eTransRequestSending:
            _aucMsgSetupTrans[1] = TRANSREQ;
            _aucMsgTrans[6] = MSMODEDEF;
            _aucAnswTrans[7] = MSMODEDEF;
            _ucDistMask = 0x1f;
            break;

        case eTransContinuousSending:
            _aucMsgSetupTrans[1] = TRANSCONT;
            _aucMsgTrans[6] = MSMODEIL;
            _aucAnswTrans[7] = MSMODEIL;
            _ucDistMask = 0x7f;
            break;

        case eTransInterlaced:
            _aucMsgSetupTrans[1] = TRANSIL;
            _aucMsgTrans[6] = MSMODEIL;
            _aucAnswTrans[7] = MSMODEIL;
            _ucDistMask = 0x7f;
            switch (_eResolution) {
                case (eRes05Degree):
                    _nScanParts = 2;
                    break;
                case (eRes025Degree):
                    _nScanParts = 4;
                    break;
                case (eRes1Degree):
                    _nScanParts = 1;
                    break;
            }
            break;

        case eTransRemission:
            _aucMsgTrans[6] = MSMODEILR;
            _aucAnswTrans[7] = MSMODEILR;
            _ucDistMask = 0x7f;
            break;
    }

    // Handle different telegrams for data acquisition with or without remission values
    if (eMode == eTransRemission) {

    } else {
        nRetVal = sendTelegram(_aucMsgTrans, sizeof(_aucMsgTrans), _aucAnswTrans, sizeof(_aucAnswTrans), 300, 10);
    }

    // do not send in configuration mode
    if (_eOperationMode != eOpConfiguration) {
        // Handle different telegrams for data acquisition with or without remission values
        if (eMode == eTransRemission) {
            nRetVal = sendTelegram(_aucMsgSetupTransRem, sizeof(_aucMsgSetupTransRem), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
        } else {
            nRetVal = sendTelegram(_aucMsgSetupTrans, sizeof(_aucMsgSetupTrans), _aucAnswSetup, sizeof(_aucAnswSetup), 300, 10);
        }
    } else {
        nRetVal = FAIR_EXIT_SUCCESS;
    }

    if (nRetVal == FAIR_EXIT_SUCCESS) _eTransmissionMode = eMode;

    return nRetVal;
}

EnumTransmissionMode CSickScanner::getTransmissionMode() {
    return _eTransmissionMode;
}

int CSickScanner::requestScan() {
    unsigned char ucAck = '0';
    unsigned long ulMsgLen = sizeof(g_aucMsgOpDataRequest);
    buildMessage(_aucMsgOpDataRequest, ulMsgLen, _szTelegram);

    ulMsgLen += 6;

    /*printf("Send telegram ");
     for(unsigned int i=0; i<ulMsgLen; i++)
     printf("%x ",_szTelegram[i]);
     printf("\n");*/

    while (ucAck != ACK) {

        // ensure, that telegram is sent
        // every query interupts data transmission from scanner to host
        if (_pAdapter->send((char*) _szTelegram, ulMsgLen))

        // maximum wait time for acknoledge is 60 ms (but sometimes, this constraint might not be fullfilled! -> 70-80 ms fits better)
        _timer->reset();
        while (_timer->getTime() < SICKMAXACKWAITTIME)
            ;
        _pAdapter->setReceiveMode(eUnbuffered);
        _pAdapter->receive((char*) &ucAck, 1);

        // if acknolegde is not received, we have to wait for 30 ms until repeating the query
        if (ucAck != ACK) {
            _timer->reset();
            while (_timer->getTime() < 30)
                ;
        }
    }
    return (ucAck == ACK);
}

// Sample for scanning with high resolution, large apex angles and remission values
// This approach is slow, due to the config messages, that have to be send to the scanner
// Each config message takes up to 70 ms
/*unsigned long CSickScanner::getScan_Rem(double* pdScan, double* pdRemission)
 {
 unsigned int unRemStart = _unRemissionStart;
 unsigned int unRemEnd = _unRemissionEnd;

 this->setRemissionRange(1, 181);
 this->getScan(pdScan,pdRemission);
 this->setRemissionRange(182, 361);
 this->getScan(pdScan+182,pdRemission+182);
 return 361;
 }*/

unsigned long CSickScanner::getScan(double* pdScan) {
    return getScan(pdScan, _adRemission);
}

unsigned long CSickScanner::getScan(double* pdScan, double* pdRemission) {
    // Scan parts are numbered in the following scheme:
    // 1 Part: 0, 0, 0, 0, ...
    // 2 Parts: 0, 2, 0 , 2, 0, 2, ...
    // 4 Parts: 0, 1, 2, 3 , 0, 1, 2, 3, ...
    unsigned int nIncr = 0;
    unsigned int nLastScan = 0;
    if (_nScanParts == 2) {
        nIncr = 2;
        nLastScan = 2;
    }
    if (_nScanParts == 4) {
        nIncr = 1;
        nLastScan = 3;
    }
    unsigned int nScanExpected = nIncr * _nScanPartsAcquired;

    unsigned int nPoints = 0;
    unsigned int nPointsCompl = 0;
    unsigned int nScan = 0;

    nPoints = getScan(_adScanTmp[_nScanPartsAcquired], _adRemTmp[_nScanPartsAcquired], &nScan);

    if (nPoints > 0) {
        _nScanPartsAcquired++;

        if (nScan != nScanExpected) {
            _nScanPartsAcquired = 0;
        } else {
            if (nScan == nLastScan) {
                nPointsCompl = getValuesPerScan();
                _nScanPartsAcquired = 0;
                for (int i = 0; i < _nScanParts; i++) {
                    for (unsigned int j = 0; j < nPoints; j++) {
                        pdScan[j * _nScanParts + i] = _adScanTmp[i][j];
                        pdRemission[j * _nScanParts + i] = _adRemTmp[i][j];
                    }
                }
            }
        }
    }
    return nPointsCompl;
}

unsigned long CSickScanner::getScan(double* pdScan, double* pdRemission, unsigned int* pnScan) {
    unsigned long ulRetval = 0;
    unsigned int nValue = 0;
    unsigned int nRemissionVal = 0;

    //CTimer* timer = new CTimer();

    // synchronize
    int nLoop = 0;
    bool bGrabMsg = true;
    int nReceived = 0;

    // check if message read isn't complete last calling
    if (_bMsgComplete) {
        _pAdapter->setReceiveMode(eUnbuffered);
        _bMsgCorrect = false;
        //cout << "Grab new message" << endl;
        _nMessageLen = 0;
        //timer->reset();
        while (bGrabMsg == true) {
            // grab data
            nReceived = _pAdapter->receive(_pszBuffer, 1);
            bGrabMsg = (nReceived == 1);
            if (bGrabMsg) {
                //cout << "ok" << endl;
                // check for STX
                if ((unsigned char) _pszBuffer[0] == STX) {
                    // check for Address
                    nReceived = _pAdapter->receive(&_pszBuffer[1], 1);
                    bGrabMsg = (nReceived == 1);
                    if (bGrabMsg) {
                        if ((unsigned char) _pszBuffer[1] == (BRC + ADROFF)) {
                            // check message and length
                            nReceived = _pAdapter->receive(&_pszBuffer[2], 3);
                            bGrabMsg = (nReceived == 3);
                            if (bGrabMsg) {
                                if (((unsigned char) _pszBuffer[4] == 0xB0) || ((unsigned char) _pszBuffer[4] == 0xF5)) {
                                    _bMsgComplete = false;
                                    // get data and check sum
                                    _nMessageLen = (int) ((unsigned char) _pszBuffer[3] << 8 | (unsigned char) _pszBuffer[2]) & 0x3ff;
                                    _pAdapter->setReceiveMode(eBuffered);
                                    nReceived = _pAdapter->receive(&_pszBuffer[5], _nMessageLen + 1);
                                    //cout << "Try to read " << _nMessageLen+1 << " bytes" << endl;
                                    _pAdapter->setReceiveMode(eUnbuffered);
                                    bGrabMsg = false;

                                    if (nReceived == _nMessageLen + 1) {
                                        _bMsgComplete = true;
                                        // if checksum fails, discard scan
                                        unsigned short usCheck = CreateCRC((unsigned char*) _pszBuffer, _nMessageLen + 4);
                                        //printf("Check: %x %x\n",(usCheck & 0xff),((usCheck>>8)&0xff));
                                        if (((usCheck >> 8) & 0xff) == (unsigned char) _pszBuffer[_nMessageLen + 5] && (usCheck & 0xff)
                                                == (unsigned char) _pszBuffer[_nMessageLen + 4])
                                        {
                                            *pnScan = (unsigned int) (_pszBuffer[6] >> 3 & 0x03);
                                            _bMsgCorrect = true;
                                        }
                                    } // Data received?
                                } // MSG and Length received?
                            } // Data received?
                        } // ADR received?
                    } // Data received?
                } // STX received ?
            }

            nLoop++;

            if (nLoop == MAX_TELEGRAM_LENGTH) bGrabMsg = false;

        }

    } else // try to complete previously read message part
    {
        _pAdapter->setReceiveMode(eBuffered);
        nReceived = _pAdapter->receive(&_pszBuffer[5], _nMessageLen + 1);

        if (nReceived > 0) {
            _bMsgComplete = true;
        }

        if (nReceived == _nMessageLen + 1) {
            if (((unsigned char) _pszBuffer[4] == 0xB0) || ((unsigned char) _pszBuffer[4] == 0xF5)) {
                // if checksum fails, discard scan
                unsigned short usCheck = CreateCRC((unsigned char*) _pszBuffer, _nMessageLen + 4);
                if (((usCheck >> 8) & 0xff) == (unsigned char) _pszBuffer[_nMessageLen + 5] && (usCheck & 0xff)
                        == (unsigned char) _pszBuffer[_nMessageLen + 4])
                {
                    *pnScan = (unsigned int) (_pszBuffer[6] >> 3 & 0x03);
                    _bMsgCorrect = true;
                }
            }
        }

    }

    if (_bMsgCorrect) {
        //calculate distance Values
        unsigned int nIndex = 0;
        unsigned long ulNrPoints = 0;
        if ((unsigned char) _pszBuffer[4] == 0xB0) {
            ulNrPoints = (_nMessageLen - 4) / 2;

            //for(int i=0; i<_nMessageLen-4; i=i+2)
            for (int i = 0; i < (int) ulNrPoints; i++) {
                nValue = (((((unsigned char) _pszBuffer[7 + 2 * i + 1]) & _ucDistMask) << 8) | ((unsigned char) _pszBuffer[7 + 2 * i]));
                //nIndex = i/2;
                //pdScan[nIndex] = (double)nValue;
                pdScan[i] = (double) nValue;
            }
        } else {
            ulNrPoints = (unsigned long) (((unsigned char) _pszBuffer[12] << 8 | (unsigned char) _pszBuffer[11]) & 0xff);
            for (unsigned int i = 0; i < ulNrPoints; i++) {

                nValue = (((((unsigned char) _pszBuffer[13 + (i * 4) + 1]) & _ucDistMask) << 8)
                        | ((unsigned char) _pszBuffer[13 + (i * 4)]));
                nIndex = i;
                pdScan[nIndex] = (double) nValue;
                nRemissionVal = ((((unsigned char) _pszBuffer[15 + (i * 4) + 1]) << 8) | ((unsigned char) _pszBuffer[15 + (i * 4)]));
                if (nRemissionVal > 32759) nRemissionVal = 0;
                pdRemission[nIndex] = (double) nRemissionVal;
            }
        }
        ulRetval = ulNrPoints;
    }
    return ulRetval;
}

int CSickScanner::sendTelegram(unsigned char* pucMsg, unsigned long ulMsgLen, unsigned char* pucExpectedAnswer, unsigned long ulAnswerLen,
        unsigned long ulWaitTime, unsigned long ulRetries) {
    int nRetval = FAIR_EXIT_FAILURE;
    unsigned char szAnswer[1024];
    unsigned long ulLoop = 0;
    unsigned char ucAck = '0';

    buildMessage(pucMsg, ulMsgLen, _szTelegram);
    buildAnswer(pucExpectedAnswer, ulAnswerLen, _szAnswer);
    ulMsgLen += 6;
    ulAnswerLen += 6;

    _pAdapter->setReceiveMode(eUnbuffered);
    while ((ucAck != ACK) & (ulLoop < ulRetries)) {
        // ensure, that telegram is sent
        // every query interupts data transmission from scanner to host
        if (_pAdapter->send((char*) _szTelegram, ulMsgLen)) {
            // maximum wait time for acknoledge is 60 ms (but sometimes, this constraint is not fullfilled! -> 80 ms fits better)
            _timer->reset();
            while (_timer->getTime() < SICKMAXACKWAITTIME)
                ;

            //cout << _pAdapter->receive((char*)&ucAck,1) << endl;
            unsigned char ucAckTmp;
            while (_pAdapter->receive((char*) &ucAckTmp, 1) != 0) {
                ucAck = ucAckTmp;
                if (ucAck == ACK) break;
            }

            // if acknolegde is not received, we have to wait for 30 ms until repeating the query
            if (ucAck != ACK) {
                _timer->reset();
                while (_timer->getTime() < 30.0)
                    ;
            } else {
                nRetval = FAIR_EXIT_SUCCESS;
            }

        } else {
            _timer->reset();
            while (_timer->getTime() < 100.0)
                ;
        }
        ulLoop++;
    }

    if (ucAck == ACK) {
        ulLoop = 0;
        while (ulLoop <= ulRetries) {
            _timer->reset();
            while (_timer->getTime() < ulWaitTime)
                ;
            _pAdapter->setReceiveMode(eUnbuffered);
            unsigned long ulBytes = _pAdapter->receive((char*) &szAnswer, ulAnswerLen);

            if (ulBytes > 0) {
                // compare received checksum with expected checksum
                if ((szAnswer[ulAnswerLen - 2] == _szAnswer[ulAnswerLen - 2]) && (szAnswer[ulAnswerLen - 1] == _szAnswer[ulAnswerLen - 1]))
                {
                    nRetval = FAIR_EXIT_SUCCESS;
                }
                break;

            }
            ulLoop++;
        }
    }
    return nRetval;
}

void CSickScanner::buildMessage(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char* szTelegram) {
    buildTelegram(szMsg, ulMsgSize, BRC, szTelegram);
}

void CSickScanner::buildAnswer(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char* szTelegram) {
    buildTelegram(szMsg, ulMsgSize, BRC + ADROFF, szTelegram);
}

void CSickScanner::buildTelegram(unsigned char szMsg[], unsigned long ulMsgSize, unsigned char ucAddress, unsigned char* szTelegram) {
    unsigned char ucLowByte = (unsigned char) (ulMsgSize & 0xff);
    unsigned char ucHighByte = (unsigned char) ((ulMsgSize >> 8) & 0xff);

    szTelegram[0] = STX;
    szTelegram[1] = ucAddress;
    szTelegram[2] = ucLowByte;
    szTelegram[3] = ucHighByte;
    for (unsigned long i = 0; i < ulMsgSize; i++)
        szTelegram[i + 4] = szMsg[i];

    unsigned short usCheckSum = CreateCRC(szTelegram, ulMsgSize + 4);
    szTelegram[ulMsgSize + 4] = (unsigned char) (usCheckSum & 0xff);
    szTelegram[ulMsgSize + 5] = (unsigned char) ((usCheckSum >> 8) & 0xff);
    szTelegram[ulMsgSize + 6] = '\0';

}

}
