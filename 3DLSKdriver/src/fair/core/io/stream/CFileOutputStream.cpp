#include "CFileOutputStream.h"

using namespace std;

namespace fair
{

CFileOutputStream::CFileOutputStream()
{
	
}

CFileOutputStream::~CFileOutputStream()
{
    // Close file
    if(_pFile.is_open())
    {
    	_pFile.close();
    }
}

int CFileOutputStream::open(char* szStreamLocation, bool bBinary)
{
	int nRetval = FAIR_EXIT_FAILURE;
	
	if(bBinary==1)
    	_pFile.open(szStreamLocation, ios::out|ios::binary);
    else
    	_pFile.open(szStreamLocation, ios::out);
    	
  	if (_pFile.is_open())
        nRetval = FAIR_EXIT_SUCCESS;
        
    return nRetval;
}

int CFileOutputStream::close()
{
	int nRetval = FAIR_EXIT_FAILURE;
	
	if(_pFile.is_open())
	{
		_pFile.close();
		nRetval = FAIR_EXIT_SUCCESS;	
	}
	
	return nRetval;
}

void CFileOutputStream::write(char cVal)
{
	if(_pFile.is_open())
		_pFile.write(&cVal,1);
}

void CFileOutputStream::write(unsigned short usVal)
{
	write((short)usVal);
}

void CFileOutputStream::write(short sVal)
{
	if(_pFile.is_open())
	{
		char* acBuf = new char[sizeof(short)];
		acBuf = (char*) &sVal;
		_pFile.write(acBuf,sizeof(short));
	}
}

void CFileOutputStream::write(int nVal)
{
	if(_pFile.is_open())
	{
		char* acBuf = new char[sizeof(int)];
		acBuf = (char*) &nVal;
		_pFile.write(acBuf,sizeof(int));
	}
}

void CFileOutputStream::write(float fVal)
{
	if(_pFile.is_open())
	{
		char* acBuf = new char[sizeof(float)];
		acBuf = (char*) &fVal;
		_pFile.write(acBuf,sizeof(fVal));
	}
}

void CFileOutputStream::write(long lVal)
{
	if(_pFile.is_open())
	{
		char* acBuf = new char[sizeof(long)];
		acBuf = (char*) &lVal;
		_pFile.write(acBuf,sizeof(long));
	}	
}

void CFileOutputStream::write(double dVal)
{
	if(_pFile.is_open())
	{
		char* acBuf = new char[sizeof(double)];
		acBuf = (char*) &dVal;
		_pFile.write(acBuf,sizeof(double));
	}
}

void CFileOutputStream::write(char* acBuffer, unsigned int nLen)
{
	if(_pFile.is_open())
		_pFile.write(acBuffer,nLen);
}


void CFileOutputStream::write(char* szBuffer)
{
	if(_pFile.is_open())
		_pFile.write(szBuffer,strlen(szBuffer));
}

}
