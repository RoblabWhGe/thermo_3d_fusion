#ifndef IINPUTSTREAM_H_
#define IINPUTSTREAM_H_

#include "../../../../fair/core/base/common.h"

/**
 * @namespace fair
 */
namespace fair
{
class IInputStream
{
public:
	/**
	 * Default constructor
	 */
	IInputStream(){};
	
	/**
	 * Default destructor
	 */
	virtual ~IInputStream(){};
	
	/**
	 * Open input stream
	 * @param szStreamLocation identifier or location of stream source
	 * @return success
	 */
	virtual int open(char* szStreamLocation, bool bBinary = 1) = 0;
	
	/**
	 * Close input stream
	 * @return success
	 */
	virtual int close() = 0;
	
	/**
	 * Is file end reached?
	 * @return file end reached = true else false
	 */
	virtual int eof() = 0;
	
	/**
	 * Rewind the file to the beginning
	 * @return success
	 */
	virtual int rewind() = 0;
	
	/**
	 * Read one byte
	 * @return readed value
	 */
	virtual int read(char& cVal) = 0;
	
	virtual int read(char* acBuffer, unsigned int nLen) = 0;
	
	virtual int read(unsigned short& usVal) = 0;
	virtual int read(short& sVal) = 0;
	
	/**
	 * Read one integer
	 * @return readed value
	 */
	virtual int read(int& nVal) = 0;

	virtual int read(long& lVal) = 0;

	virtual int read(float& fVal) = 0;
	
	virtual int read(double& dVal) = 0;
};
}

#endif //IINPUTSTREAM_H_
