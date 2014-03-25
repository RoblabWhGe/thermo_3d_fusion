#include "CFileInputStream.h"

namespace fair
{
	
CFileInputStream::CFileInputStream()
{
	
}
	
CFileInputStream::~CFileInputStream()
{
    if(_pFile.is_open())
    {
    	_pFile.close();
    }
}

int CFileInputStream::open(char* szStreamLocation, bool bBinary)
{
	int nRetval = FAIR_EXIT_FAILURE;
	
	if(bBinary==1)
    	_pFile.open(szStreamLocation, ios::in|ios::binary);
    else
    	_pFile.open(szStreamLocation, ios::in);
    	
  	if (_pFile.is_open())
        nRetval = FAIR_EXIT_SUCCESS;
        
    return nRetval;
}
	
int CFileInputStream::close()
{
	int nRetval = FAIR_EXIT_FAILURE;
	
	if(_pFile.is_open())
	{
		_pFile.close();
		nRetval = FAIR_EXIT_SUCCESS;	
	}
	
	return nRetval;
}

int CFileInputStream::eof()
{
	int nRetval = 0;
	
	if(_pFile.is_open())
	 	nRetval = _pFile.eof();
	 	
	return nRetval;
}

int CFileInputStream::rewind()
{
	int nRetval = 0;
	
	if(_pFile.is_open())
	{
		_pFile.clear();
		_pFile.seekg(0);
		nRetval = 1;
	}
		
	return nRetval;
	
}
	
int CFileInputStream::read(char& cVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read(&cVal, 1);
		if(_pFile.gcount()==1)
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

int CFileInputStream::read(char* acBuffer, unsigned int nLen)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read(acBuffer, nLen);
		if(_pFile.gcount()==(int)nLen)
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

int CFileInputStream::read(unsigned short& usVal)
{
	return this->read((short&)usVal);	
}

int CFileInputStream::read(short& sVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read ((char*)&sVal, sizeof(short));
		if(_pFile.gcount()==sizeof(short))
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

int CFileInputStream::read(int& nVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read ((char*)&nVal, sizeof(int));
		if(_pFile.gcount()==sizeof(int))
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

int CFileInputStream::read(long& lVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read ((char*)&lVal, sizeof(long));
		if(_pFile.gcount()==sizeof(long))
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

int CFileInputStream::read(float& fVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read ((char*)&fVal, sizeof(float));
		if(_pFile.gcount()==sizeof(float))
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}
	
int CFileInputStream::read(double& dVal)
{
	int nRetval = FAIR_EXIT_FAILURE;
	if(_pFile.is_open())
	{
		_pFile.read ((char*)&dVal, sizeof(double));
		if(_pFile.gcount()==sizeof(double))
			nRetval = FAIR_EXIT_SUCCESS;
	}
	return nRetval;
}

}
