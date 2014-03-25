/*
 *
 * libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
 *
 * Copyright Fraunhofer Gesellschaft e. V., Munich, Germany
 *
 * The FAIR library [both binary and source code (if released)] is intellectual
 * property owned by Fraunhofer Gesellschaft and is protected by copyright;
 * the ownership remains with Fraunhofer Gesellschaft.
 *
 */

#ifndef CSOCKETADAPTER_H_
#define CSOCKETADAPTER_H_

#include <iostream>
#include <string>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../../../fair/core/io/IDeviceAdapter.h"
#include "../../../../fair/core/base/common.h"

using namespace std;

namespace fair {

/**
 * @brief Represents the adapter type of an Socket Connection
 * 
 * @author Christoph Brauers
 * @date January 2009
 */

class CSocketAdapter: public IDeviceAdapter {
public:

	/**
	 * Standard constructor
	 */
	CSocketAdapter(const char *szDeviceName);

	/**
	 * Default constructor
	 */
	CSocketAdapter();

	/**
	 * Default destructor. This should clean up.
	 * @post The object must be cleaned up.
	 */
	virtual ~CSocketAdapter();

	/**
	 * Open the device.
	 * @return !0 on success
	 */
	int openDevice();

	/**
	 * Close the device.
	 * @return !0 on succes
	 */
	int closeDevice();

	/**
	 * Send data.
	 * @param szBuffer Data to be sent
	 * @param ulLength Length of data in bytes
	 * @return Number of bytes sent or negative on error.
	 */
	unsigned long send(const char *szBuffer, unsigned long ulLength);

	/**
	 * Receive data.
	 * @param szBuffer Buffer where received data will be stored
	 * @param ulLength Maximum number of bytes which will be received
	 * @return Number of bytes received or negative on error.
	 */
	unsigned long receive(char *szBuffer, unsigned long ulLength);

	/**
	 * Get name of the device
	 * @return Devicename
	 */
	const char *getDeviceName() const;

	/**
	 * Set name of the device.
	 * If a new devicename is provided the device should be reopened to
	 * make sure that someone is using the new devicename.
	 * @param szDeviceName Devicename
	 */
	void setDeviceName(const char *szDeviceName);

	/**
	 * Get type of adapter. The intention of this method is to allow
	 * someone to distinguish between different implementations.
	 * @return Type of connection as EnumAdapterType
	 */
	EnumAdapterType getAdapterType() const;

	/**
	 * Set receive mode
	 * @param eMode receive mode
	 */
	void setReceiveMode(EnumReceiveMode eMode);

	/**
	 * Get receive mode
	 * @return receive mode
	 */
	EnumReceiveMode getReceiveMode() const;

private:

	/*
	 * Sockethandle used to connect to
	 */
	int _socketHandle;

	/*
	 * Structure describing an Internet socket address.
	 */
	sockaddr_in _tAddress;

	/**
	 * Device name is equal to the address and the prot that should be connected to
	 * (e.g. 192.168.0.1:1001)
	 */
	string _sDeviceName;

	/*
	 * Establishing a socket TCP connection
	 */
	bool _connect(string host, int port);

	/*
	 * Disconnecting the socket TCP connection
	 */
	bool _disconnect();

};

}

#endif /*CSOCKETADAPTER_H_*/
