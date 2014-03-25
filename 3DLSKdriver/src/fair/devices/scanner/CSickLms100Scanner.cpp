/*
 * LMSScanner.cpp
 *
 *  Created on: Jan 20, 2010
 *      Author: christoph
 */

#include "CSickLms100Scanner.h"
#include "../../../fair/devices/factory/IDevice.h"
#include "../../../fair/core/io/IDeviceAdapter.h"
#include "../../../fair/devices/scanner/IScanner.h"
#include "sstream"

fair::CSickLms100Scanner::CSickLms100Scanner(IDeviceAdapter *pAdapter)
{

  _scannerSufaceToCenterOffset = 57;
  _pAdapter = pAdapter;
  _nResolution = 540;
  _eUnit = eUnitMM;
  _nApexAngle = 270;
  _nScanningTime = 20; // in milliseconds = 50hz
  _eScanDirection = eCLOCKWISE;
  _nStartScanNumber = 0; // defines the starting index of the laserscanner
  _bConnected = false;
}

fair::CSickLms100Scanner::CSickLms100Scanner()
{
  _scannerSufaceToCenterOffset = 57;
  _nResolution = 540;
  _eUnit = eUnitMM;
  _nApexAngle = 270;
  _nScanningTime = 20; // in milliseconds = 50hz
  _eScanDirection = eCLOCKWISE;
  _nStartScanNumber = 0; // defines the starting index of the laserscanner
  _bConnected = false;
}

fair::CSickLms100Scanner::~CSickLms100Scanner()
{
  _bConnected = false;
}

fair::IDeviceAdapter * fair::CSickLms100Scanner::getDeviceAdapter()
{
  return _pAdapter;
}

void fair::CSickLms100Scanner::setDeviceAdapter(IDeviceAdapter *pAdapter)
{
  _pAdapter = pAdapter;
}

double fair::CSickLms100Scanner::getScanningTime()
{
  return _nScanningTime;
}

fair::EnumUnit fair::CSickLms100Scanner::getUnit()
{
  return _eUnit;
}

int fair::CSickLms100Scanner::getScannerApexAngle()
{
  return _nApexAngle;
}

int fair::CSickLms100Scanner::getScannerResolution()
{
  return _nResolution;
}

fair::EnumScanDirection fair::CSickLms100Scanner::getDirection()
{
  return _eScanDirection;
}

int fair::CSickLms100Scanner::getMaxRange(int *pnStart, int *pnStop)
{

  *pnStart = _nStartScanNumber;
  *pnStop = _nStartScanNumber + _nResolution;

  return _nResolution;
}

int fair::CSickLms100Scanner::getRange(int *pnStart, int *pnStop)
{
  return getMaxRange(pnStart, pnStop);
}

int fair::CSickLms100Scanner::setRange(int nStart, int nStop)
{
  return FAIR_EXIT_FAILURE;
}

int fair::CSickLms100Scanner::getValuesPerScan()
{
  return 541;//(getScannerApexAngle() * getScannerResolution() / 360) + 1;
}

int fair::CSickLms100Scanner::connect()
{
  if (initalize())
    return 0;
  return -1;
}

int fair::CSickLms100Scanner::disconnect()
{
  _bConnected = false;
  return 1;
}

unsigned long fair::CSickLms100Scanner::getScan(double *pdPoints)
{
  return getScan(pdPoints, _adRemission);
}

unsigned long fair::CSickLms100Scanner::getScan(double *pdScan, double *pdRemission)
{
  while (_nScanCounter == _nOldScanNumber)
  {
    string result = CallCommand("sRN LMDscandata", 5000);
    if (decodeScan(result, pdScan, pdRemission) == false)
    {
      return -1;
    }
  }
  _nOldScanNumber = _nScanCounter;
  return _nResolution;
}
bool fair::CSickLms100Scanner::setAccessMode()
{
  std::stringstream cmd;
  char *pchNext;
  char *pTempStore;
  string status_ready;
  status_ready = eStatusReady;

  cmd << SOPAS_METHODE_BY_NAME << " " << CMD_SET_ACCESS_MODE_CMD << " " << ACCESS_MODE_CLIENT << " " << USER_LEVEL_CLIENT;
  string tmp = CallCommand(cmd.str(), 100);
  if ( -1 == tmp.find("01"))
    return false;
  do
  {

    cmd.clear();
    cmd << SOPAS_READ_BY_NAME <<" "<< CMD_STATUS;
    tmp= CallCommand(cmd.str(), 100).c_str();
    int pos = tmp.find_first_of(CMD_STATUS);

  } while (strcmp(pchNext, status_ready.c_str()) != 0);

}


bool fair::CSickLms100Scanner::initalize()
{
  ACCESS_MODE_MAINTAINER = "02";
  ACCESS_MODE_CLIENT = "03";
  ACCESS_MODE_SERVICE = "04";
  USER_LEVEL_MAINTAINER = "B21ACE26h";
  USER_LEVEL_CLIENT = "F4724744h";
  SOPAS_METHODE_BY_NAME = "sMN";
  SOPAS_READ_ANSWER = "sRA";
  SOPAS_READ_BY_NAME = "sRN";
  CMD_SET_ACCESS_MODE_CMD = "SetAccessMode";
  CMD_STATUS = "STlms";

  _eResolution = fair::eRes05Degree;
  char *pchNext;
  char *pTempStore;
  if (_bConnected != true)
  {

    if (CallCommand("sMN SetAccessMode 03 F4724744", 100) == "")
      return false;
    do
    {
      pTempStore = (char*)CallCommand("sRN STlms", 100).c_str();
      pchNext = strtok(pTempStore, " ");//Kommandoart sRA
      pchNext = strtok(NULL, " ");//Kommando STlms
      pchNext = strtok(NULL, " ");//Status 0-7 [7==ready]
    } while (strcmp(pchNext, "7") != 0);

    if (CallCommand("sMN mLMPsetscancfg +5000 1 +5000 -450000 +2250000", 100) == "")
    {
      return false;
    }
    else
    {
      _eResolution = fair::eRes05Degree;
    }

    do
    {
      pTempStore = (char*)CallCommand("sRN STlms", 100).c_str();
      pchNext = strtok(pTempStore, " ");//Kommandoart sRA
      pchNext = strtok(NULL, " ");//Kommando STlms
      pchNext = strtok(NULL, " ");//Status 0-7 [7==ready]
    } while (strcmp(pchNext, "7") != 0);

    //cout << " setting Remission " << endl;
    if (CallCommand("sWN LMDscandatacfg 01 01 1 1 0 00 00 0 0 0 0 +1", 100) == "")
      return false;

    do
    {
      pTempStore = (char*)CallCommand("sRN STlms", 100).c_str();
      pchNext = strtok(pTempStore, " ");//Kommandoart sRA
      pchNext = strtok(NULL, " ");//Kommando STlms
      pchNext = strtok(NULL, " ");//Status 0-7 [7==ready]
    } while (strcmp(pchNext, "7") != 0);

    //		if (CallCommand("sMN SetAccessMode 02 B21ACE26", 1000) == "0")
    //			return false;

    if (CallCommand("sMN Run", 100) == "")
    {
      return false;
    }

    _bConnected = true;

    return true;
  }
  else
  {
    return false;
  }
}

string fair::CSickLms100Scanner::CallCommand(string sCommand, int nLength)
{

  string sResult;

  sendMessage(sCommand);
  int nReturnVal = receiveMessage(&sResult, nLength);

  if (nReturnVal != -1)
  {
    //cout << " Successfully received the message :" << nReturnVal << endl;
    return sResult;
  }
  else
  {
    //cout << " An error occured receiving a message :" << nReturnVal << endl;
    return "";
  }
}

bool fair::CSickLms100Scanner::sendMessage(string sCommand)
{

  char begin = 0x02;
  char end = 0x03;
  stringstream chTemp;

  chTemp << begin << sCommand << end;

  unsigned long nRet = _pAdapter->send(chTemp.str().c_str(), chTemp.str().size());

  if (nRet > 0)
  {

    //cout << " Submitted the message " << chTemp.str() << "successfully" << endl;
    return true;
  }
  else
  {
    //cout  << " An Error occured submitting the message " << chTemp.str() << endl;
    return false;
  }

  return true;

}

bool fair::CSickLms100Scanner::receiveMessage(string* pResult, unsigned long ulLength)
{

  char chInputBuffer[ulLength + 1];
  int nStatus = 1;

  memset(chInputBuffer, 0, ulLength + 1);

  // read until the end Token is read
  while (chInputBuffer[nStatus - 1] != 0x03)
  {

    nStatus = _pAdapter->receive(chInputBuffer, ulLength);
    *pResult = *pResult + chInputBuffer;
  }
  //std::cout << "fair::CSickLms100Scanner::receiveMessage " << chInputBuffer << std::endl;
  if (nStatus > 0 || nStatus != -1)
  {
    return nStatus;
  }
  else
  {
    return -1;
  }
}

fair::EnumResolution fair::CSickLms100Scanner::getResolution()
{
  return _eResolution;
}

int setResolution(fair::EnumResolution resolution)
{

}

bool fair::CSickLms100Scanner::decodeScan(string sBuffer, double* pdScan, double* pdRemission)
{

  char *pchNext;
  unsigned int uiIndex = 0;
  //double dFactor;
  unsigned int unResolution = 0;
  char* pTempStore;
  int numberOf16Bit = 0;

  int angularStepWidth = 0;
  int startingAngle = 0;
  double scalingOffset = 0;
  double scalingFactor = 0;

  //		std::cout << "fair::CSickLms100Scanner::decodeScan " << sBuffer << std::endl;
  pTempStore = (char*)sBuffer.c_str();

  pchNext = strtok(pTempStore, " ");

  while (pchNext != NULL && unResolution == 0)
  {

    switch (++uiIndex)
    {

      case 1:
      { //Kommandoart
        if (strncmp(&pchNext[1], "sRA", 3) && strncmp(&pchNext[1], "sSN", 3))
          return false;
        break;
      }
      case 2:
      { // Name of Command
        if (strcmp(pchNext, "LMDscandata") != 0)
          return false;
        break;
      }
      case 3: //VersionsNummer
        break;
      case 4: //Geraetenummer
        break;
      case 5:
      { //Seriennummer
        sscanf(pchNext, "%x", &_nSerialNumber);
        break;
      }
      case 6:
      { // firts Status of LMS
        if (strcmp(pchNext, "0") != 0)
        {
          //cout << " Wrong LMS status message" << endl;
          return false;
        }
        break;
      }
      case 7:
      {// second Status of LMS
        if (strcmp(pchNext, "0") != 0)
        {
          //cout << " Wrong LMS status message" << endl;
          return false;
        }
        break;
      }
      case 8://Telegrammzaehler
        break;
      case 9: //ScanZaehler
        sscanf(pchNext, "%x", &_nScanCounter);
        break;
      case 10: //Einschaltdauer
        break;
      case 11: //Uebertragungsdauer
        break;
      case 12: //EingangsStatus I/O
        break;
      case 13: //AusgangsStatus I/O
        break;
      case 14: //ReserviertesByteA not in use
        break;
      case 15: //ScanFrequenz
        break;
      case 16: //MessFrequenz
        break;
      case 17: //AnzahlDrehgeber
        break;
      case 18: //DrehgeberPosition
        break;
      case 19: //DrehgeberGeschwindigkeit
        break;
      case 20: //AnzahlKanaele16Bit
        numberOf16Bit = atoi(pchNext);
        break;
      case 21:
      {// MeasuredDataContent
        if (0 != strcmp(pchNext, "DIST1"))
        {
          //cout << " Wrong LMS measuredDataContent message" << endl;
          return false;
        }
        break;
      }
      case 22: // ScalingFactor
        sscanf(pchNext, "%x", &scalingFactor);
        scalingFactor = 1;
        //scalingFactor = strtod(pchNext, NULL);
        break;
      case 23: // SkalierungsOffset
        scalingOffset = strtod(pchNext, NULL);
        break;
      case 24: //Startwinkel
        startingAngle = strtol(pchNext, NULL, 16);
        break;
      case 25: //Winkelschrittweite
        angularStepWidth = strtoul(pchNext, NULL, 16);
        break;
      case 26: // NumberData
        unResolution = strtoul(pchNext, NULL, 16);
        break;
      default:
      {
        break;
      }
    }
    pchNext = strtok(NULL, " ");
  }

  _nResolution = unResolution;
  for (unsigned int i = 0; i < unResolution; i++, pchNext = strtok(NULL, " "))
  {
    uiIndex++;
    pdScan[i] = scalingFactor * double(strtoul(pchNext, NULL, 16)) + scalingOffset + _scannerSufaceToCenterOffset;
  }

  if ((0 == strcmp(pchNext, "RSSI1")) && pdRemission != NULL)
  {
    angularStepWidth = 0;
    startingAngle = 0;
    scalingOffset = 0;
    scalingFactor = 0;
    uiIndex = 0;

    while (pchNext && angularStepWidth == 0)
    {

      switch (++uiIndex)
      {

        case 1:
          if (strcmp(pchNext, "RSSI1") != 0)
          {
            std::cout << " no remission values available " << std::endl;
            return false;
          }
          break;

        case 2: // ScalingFactor
          sscanf(pchNext, "%x", &scalingFactor);
          scalingFactor = 1;
          break;

        case 3: // scalingOffset
          scalingOffset = strtod(pchNext, NULL);
          break;
        case 4:
          startingAngle = strtol(pchNext, NULL, 16);
          break;

        case 5: // NumberData
          angularStepWidth = strtoul(pchNext, NULL, 16);
          break;

        default:
          break;
      }
      pchNext = strtok(NULL, " ");
    }

    for (unsigned int i = 0; i < unResolution; i++, pchNext = strtok(NULL, " "))
    {
      pdRemission[i] = scalingFactor * double(strtoul(pchNext, NULL, 16)) + scalingOffset;
    }

  }
  return true;
}

