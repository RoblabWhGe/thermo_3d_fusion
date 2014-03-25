#ifndef CFILEOUTPUTSTREAM_H_
#define CFILEOUTPUTSTREAM_H_

#include "fair/core/io/stream/IOutputStream.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <sstream>

using namespace std;
/**
 * @namespace fair
 */
namespace fair
{
	
class CFileOutputStream : public IOutputStream
{
public:
	/**
	 * Default constructor
	 */
	CFileOutputStream();
	
	/**
	 * Default destructor
	 */
	virtual ~CFileOutputStream();
	
	/**
	 * Open output stream
	 * @param szStreamLocation identifier or location of stream source
	 * @return success
	 */
	virtual int open(char* szStreamLocation, bool bBinary = 1);
	
	/**
	 * Close output stream
	 * @return success
	 */
	virtual int close();
	
	/**
	 * Write one value
	 * @param cVal value to be written
	 */
	virtual void write(char cVal);
	virtual void write(unsigned short usVal);
	virtual void write(short sVal);
	virtual void write(int nVal);
	virtual void write(float dVal);
	virtual void write(long lVal);
	virtual void write(double dVal);
	
	/**
	 * Write an array
	 * @param acBuffer data buffer to be written
	 * @param nLen length of data buffer
	 */
	virtual void write(char* acBuffer, unsigned int nLen);


	/**
	 * Write null-terminated strings
	 * @param szBuffer data buffer to be written
	 */
	virtual void write(char* szBuffer);
	
private:

	/**
	 * output stream
	 */
    ofstream _pFile;
};
}

#endif //CFILEOUTPUTSTREAM_H_
