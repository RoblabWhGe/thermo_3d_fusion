#include "CSocketAdapter.h"

fair::CSocketAdapter::CSocketAdapter() {
}

fair::CSocketAdapter::CSocketAdapter(const char *szDeviceName) {
	_sDeviceName = szDeviceName;
}

fair::CSocketAdapter::~CSocketAdapter() {
}


bool fair::CSocketAdapter::_connect(string sHostAdress = "192.168.0.1", int nHostPort = 1001) {

	_socketHandle = ::socket(AF_INET, SOCK_STREAM, 0);

	if (_socketHandle < 0) {
#ifdef DEBUG
		cout << " ! Connection failed to established to host: " << sHostAdress << " : " << nHostPort << endl;
#endif
		return false;
	}

	int nY = 1;
	setsockopt(_socketHandle, SOL_SOCKET, SO_REUSEADDR, &nY, sizeof(int));

	if (_socketHandle == -1) {
#ifdef DEBUG
		cout << " ! Connection failed to established to host: " << sHostAdress << " : " << nHostPort << endl;
#endif
		return false;
	}

	struct hostent * strHostInfo;
	unsigned long ulAdress;

	memset(&_tAddress, 0, sizeof(_tAddress));

	if ((ulAdress = inet_addr(sHostAdress.c_str())) != INADDR_NONE) {

		memcpy((char*) &_tAddress.sin_addr, &ulAdress, sizeof(ulAdress));

	} else {

		strHostInfo = gethostbyname(sHostAdress.c_str());

		if (NULL == strHostInfo) {
#ifdef DEBUG
			cout << " ! Unknown Server" << endl;
#endif
		}

		memcpy((char *) &_tAddress.sin_addr, strHostInfo->h_addr, strHostInfo->h_length);

	}

	_tAddress.sin_family = AF_INET;
	_tAddress.sin_port = htons(nHostPort);
	int status = ::connect(_socketHandle, (sockaddr *) &_tAddress, sizeof(_tAddress));

	if (status == 0) {
#ifdef DEBUG
		cout << " ! Connection successfully established " << endl;
#endif
		return 0;
	} else {
#ifdef DEBUG
		cout << " ! Connection failed : connect returned errorcode "<< endl;
#endif
		return -1;
	}
}

bool fair::CSocketAdapter::_disconnect() {

	::close(_socketHandle);

	return true;
}

int fair::CSocketAdapter::openDevice() {

	int nRetval = FAIR_EXIT_FAILURE;
	if (_sDeviceName.empty())
		return nRetval;

	// extracting the IP-Adress and the Port from the _sDeviceName
	int nStringDevider = _sDeviceName.find_first_of(':', 0);
	string sAdress = _sDeviceName.substr(0, nStringDevider);
	int nPort = atoi(_sDeviceName.substr(nStringDevider + 1, _sDeviceName.length()).c_str());

	_connect(sAdress, nPort);

	return 0;
}

int fair::CSocketAdapter::closeDevice() {
	_disconnect();
	return 0;
}

void fair::CSocketAdapter::setDeviceName(const char *szDeviceName) {
	_sDeviceName = szDeviceName;
}

const char *fair::CSocketAdapter::getDeviceName() const {
	return _sDeviceName.c_str();
}

unsigned long fair::CSocketAdapter::send(const char *szBuffer, unsigned long ulLength) {

	int nStatus = ::send(_socketHandle, szBuffer, ulLength, 0);

	if (nStatus == -1) {
#ifdef DEBUG
		cout << " there occured an error sending string : " << szBuffer << endl;
#endif
		// TASK : replace by ErrorMessage
		return -1;
	} else {
#ifdef DEBUG
		cout << " CSocketAdapter::send : " << szBuffer << "  was successfully submitted " << endl;
#endif
		return nStatus;
	}
}

unsigned long fair::CSocketAdapter::receive(char *szBuffer, unsigned long ulLength) {

	int nStatus = ::recv(_socketHandle, szBuffer, ulLength, 0);
	if (nStatus > 0 || nStatus != -1) {

		return nStatus;
	} else {

#ifdef DEBUG
		cout << " ! An error receiving a message occured!" << endl;
#endif
		return -1;
	}

}

void fair::CSocketAdapter::setReceiveMode(EnumReceiveMode eMode) {
}

fair::EnumAdapterType fair::CSocketAdapter::getAdapterType() const {

	return eSOCKET;
}

fair::EnumReceiveMode fair::CSocketAdapter::getReceiveMode() const {
	return eUnbuffered;
}
