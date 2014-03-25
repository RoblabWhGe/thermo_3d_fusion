#ifndef IOUTPUTSTREAM_H_
#define IOUTPUTSTREAM_H_

#include "../../../../fair/core/base/common.h"

/**
 * @namespace fair
 */
namespace fair
{
class IOutputStream
{
public:
	/**
	 * Default constructor
	 */
	IOutputStream(){};
	
	/**
	 * Default destructor
	 */
	virtual ~IOutputStream(){};
	
	/**
	 * Open output stream
	 * @param szStreamLocation identifier or location of stream source
	 * @return success
	 */
	virtual int open(char* szStreamLocation, bool bBinary = 1) = 0;
	
	/**
	 * Close output stream
	 * @return success
	 */
	virtual int close() = 0;
	
	/**
	 * Write one byte
	 * @param cVal write value
	 */
	virtual void write(char cVal) = 0;
	
	virtual void write(unsigned short usVal) = 0;
	virtual void write(short sVal) = 0;
	
	/**
	 * Write one byte
	 * @param nVal write value
	 */
	virtual void write(int nVal) = 0;
	
	virtual void write(float fVal) = 0;
	
	virtual void write(long lVal) = 0;
	
	virtual void write(double dVal) = 0;
	/**
	 * Write an array
	 * @param acBuffer data buffer to be written
	 * @param nLen length of data buffer
	 */
	virtual void write(char* acBuffer, unsigned int nLen) = 0;
	
	/**
	 * Write null-terminated strings
	 * @param szBuffer data buffer to be written
	 */
	virtual void write(char* szBuffer) = 0;
};
}

#endif //IOUTPUTSTREAM_H_
