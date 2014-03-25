#ifndef CFILEINPUTSTREAM_H_
#define CFILEINPUTSTREAM_H_

#include <iostream>
#include <fstream>
#include "fair/core/io/stream/IInputStream.h"

using namespace std;

/**
 * @namespace fair
 */
namespace fair
{
class CFileInputStream : public IInputStream
{
public:
	/**
	 * Default constructor
	 */
	CFileInputStream();
	
	/**
	 * Default destructor
	 */
	virtual ~CFileInputStream();
	
	/**
	 * Open input stream
	 * @param szStreamLocation identifier or location of stream source
	 * @return success
	 */
	virtual int open(char* szStreamLocation, bool bBinary = 1);
	
	/**
	 * Close input stream
	 * @return success
	 */
	virtual int close();
	
	/**
	 * Is file end reached?
	 * @return file end reached = true else false
	 */
	int eof();
	
	/**
	 * Rewind the file to the beginning
	 * @return success
	 */
	int rewind();	
	
	/**
	 * Read one byte
	 * @return readed value
	 */
	virtual int read(char& cVal);
	
	virtual int read(char* acBuffer, unsigned int nLen);
	
	virtual int read(unsigned short& usVal);
	virtual int read(short& sVal);
	
	/**
	 * Read one integer
	 * @return readed value
	 */
	virtual int read(int& nVal);

	virtual int read(long& lVal);

	virtual int read(float& fVal);
	
	virtual int read(double& dVal);
	
private:

	/**
	 * input stream
	 */
    ifstream _pFile;
};
}

#endif //CFILEINPUTSTREAM_H_
