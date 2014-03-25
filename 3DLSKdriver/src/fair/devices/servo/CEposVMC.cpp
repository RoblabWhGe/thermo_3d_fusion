/*
 * File:   CEposVMC.cpp
 * Author: viatcheslav
 *
 * Created on November 10, 2009, 3:44 PM
 */

#include "CEposVMC.h"
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>
#include <termio.h>

using namespace std;
using namespace epos;

//#define DEBUG 1
//#define DDEBUG 1
//#define USBMODE 1 //DOES NOT WORK SO FAR!!! DO NOT USE

/*! starting point for (slow!) homing movement. If the zero point is
 not off too far, this will speed things up enormously!
 */
#define E_STARTPOS_HOMING -200000

/* EPOS codes */

#define E_OK      0x4f  ///< EPOS answer code for <em>all fine</em>
#define E_FAIL    0x46  ///< EPOS answer code to indicate a <em>failure</em>
#define E_ANS     0x00  ///< EPOS code to indicate an answer <em>frame</em>
/* EPOS error codes (Communication Guide, 6.4)  */

/* CANopen defined error codes */
#define E_NOERR         0x00000000   ///< Error code: no error
#define E_ONOTEX        0x06020000   ///< Error code: object does not exist
#define E_SUBINEX       0x06090011   ///< Error code: subindex does not exist
#define E_OUTMEM        0x05040005   ///< Error code: out of memory
#define E_NOACCES       0x06010000   ///< Error code: Unsupported access to an object
#define E_WRITEONLY     0x06010001   ///< Error code: Attempt to read a write-only object
#define E_READONLY      0x06010002   ///< Error code: Attempt to write a read-only object
#define E_PARAMINCOMP   0x06040043   ///< Error code: general parameter incompatibility
#define E_INTINCOMP     0x06040047   ///< Error code: general internal incompatibility in the device
#define E_HWERR         0x06060000   ///< Error code: access failed due to an hardware error
#define E_PRAGNEX       0x06090030   ///< Error code: value range of parameter exeeded
#define E_PARHIGH       0x06090031   ///< Error code: value of parameter written is too high
#define E_PARLOW        0x06090032   ///< Error code: value of parameter written is too low
#define E_PARREL        0x06090036   ///< Error code: maximum value is less than minimum value
/* maxon specific error codes */
#define E_NMTSTATE      0x0f00ffc0   ///< Error code: wrong NMT state
#define E_RS232         0x0f00ffbf   ///< Error code: rs232 command illegeal
#define E_PASSWD        0x0f00ffbe   ///< Error code: password incorrect
#define E_NSERV         0x0f00ffbc   ///< Error code: device not in service mode
#define E_NODEID        0x0f00fb9    ///< Error code: error in Node-ID
/* EPOS Statusword -- singe bits, see firmware spec 14.1.58 */
#define E_BIT15        0x8000      ///< bit code: position referenced to home position
#define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
#define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
#define E_BIT12        0x1000      ///< bit code: OpMode specific
#define E_BIT11        0x0800      ///< bit code: NOT USED
#define E_BIT10        0x0400      ///< bit code: Target reached
#define E_BIT09        0x0200      ///< bit code: Remote (?)
#define E_BIT08        0x0100      ///< bit code: offset current measured (?)
#define E_BIT07        0x0080      ///< bit code: WARNING
#define E_BIT06        0x0040      ///< bit code: switch on disable
#define E_BIT05        0x0020      ///< bit code: quick stop
#define E_BIT04        0x0010      ///< bit code: voltage enabled
#define E_BIT03        0x0008      ///< bit code: FAULT
#define E_BIT02        0x0004      ///< bit code: operation enable
#define E_BIT01        0x0002      ///< bit code: switched on
#define E_BIT00        0x0001      ///< bit code: ready to switch on
/* EPOS modes of operation, firmware spec 14.1.59 (p.133, tbl. 72) */
#define E_HOMING      6 ///< EPOS operation mode: homing
#define E_PROFVEL     3 ///< EPOS operation mode: profile velocity mode
#define E_PROFPOS     1 ///< EPOS operation mode: profile position mode
// the modes below should not be used by user, defined here only for
// completeness
#define E_POSMOD     -1 ///< EPOS operation mode: position mode
#define E_VELMOD     -2 ///< EPOS operation mode: velocity mode
#define E_CURRMOD    -3 ///< EPOS operation mode: current mode
#define E_DIAGMOD    -4 ///< EPOS operation mode: diagnostics mode
#define E_MASTERENCMOD -5 ///< EPOS operation mode:internal
#define E_STEPDIRECMOD -6 ///< EPOS operation mode:internal
int debugLevel = 1;

int sp; ///<serial port file descriptor
DWORD E_error_new; ///< EPOS global error status


/* globals */

static int ep = -1; ///< \brief EPOS file descriptor
char gMarker = 0; ///< \brief global; for internal handling
bool gMarkerEO = false;
BYTE NodeId = 0x01; //starting node ID or default one
int nodesQuantity = 1;

//using namespace epos;

CEposVMC::CEposVMC(int numOfArgs, char ** argument) {
	CEposVMC::numOfArgs = numOfArgs;
	CEposVMC::argument = argument;
	CEposVMC::faultsCounter = 0;
	InitDevice(numOfArgs, argument);

}

CEposVMC::CEposVMC(const CEposVMC& orig) {
}

CEposVMC::~CEposVMC() {

	CloseDevice();
}

void CEposVMC::Help(char *arg) {

	printf("\n\n   ### filter wheel control ###\n\n");
	printf("usage: %s <devicename>\n", arg);
	printf(
			"valid names are full path to serial device (RS232) and 'IP:tcp port' to connect via rs232 over TCP/IP\n\n");
	printf("example: '%s /dev/ttyUSB0'\n\n", arg);
	printf(
			"         '%s 192.168.1.100:2572' (serial port A on main camera)\n\n",
			arg);
	exit(0);

}

int CEposVMC::PrintHelp() {

	cout << "\tDriver for Epos 24/1, 24/5 moto-controller" << endl;
	//	cout << "please refer to test.cpp for basic usage example" << endl;

	return (0);
}

int CEposVMC::kbhit() {

	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}

int CEposVMC::getch() {

	int ch;
	struct termios oldt;
	struct termios newt;
	tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
	newt = oldt; /* copy old settings to new settings */
	newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
	ch = getchar(); /* standard getchar call */
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
	return ch; /*return received char */
}

int CEposVMC::InitDevice(int argc, char **arg) {

	if (argc != 2) {
		Help(arg[0]);
	}
	/* open EPOS device */

	// check wether connection is made via TCP/IP or ser. device

	// is argument starting with a '/'?
	if (!strncmp(arg[1], "/", 1)) {
		printf("\ttrying to open '%s'...", arg[1]);
		if (openEPOS(arg[1]) < 0) {
			cerr << "\terror openEPOS" << endl;
			exit(-1);
		}
		printf("\tdone.\n");
	}
	// is argument starting with a digit? => assume IP adress!
	else if (isdigit(*arg[1])) {
		//    printf("%s\n", arg[1]);
		string_pos = strspn(arg[1], "1234567890.");
		if (string_pos == strlen(arg[1])) {
			fprintf(stderr, "TCP/IP PORT NOT GIVEN!\n");
			return (1);
		}
		//    fprintf(stderr,">>>%d<<<\n", string_pos);
		strncpy(ip, arg[1], string_pos);
		ip[string_pos] = '\0';
		//printf("%s; %d  -- >%s<\n", ip, string_pos, strpbrk(arg[1],":"));

		if (sscanf(strpbrk(arg[1], ":"), ":%hu", &port) == EOF) {
			fprintf(stderr, "TCP/IP PORT NOT GIVEN!\n");
			return (1);
		}
		printf("\ttrying to open %s, port %hu...", ip, port);
		if (openTCPEPOS(ip, port) < 0)
			exit(-1);
		printf("done.\n");
	}

	else {
		fprintf(stderr, "INVALID ARGUMET!\n");
		Help(arg[0]);
	}

	/******************************************************
	 switch on
	 ******************************************************/
	int quantity = nodesQuantity;
	BYTE ID = NodeId;

	printEPOSstate();
	if (nodesQuantity == 2) {
		cout << "\tstate of node: " << ID + 1;
		printEPOSstate(ID + 1);

	}

	do {
		// 1. check if we are in FAULT state
		cout << "\t1. check if we are in FAULT state" << endl;
		cout << "\tstate of node: " << (int) ID << endl;
		n = checkEPOSstate();
		if (n == 11) {
			printf("\tEPOS is in FAULT state, doing FAULT RESET... ");
			// should check which fault this is, assume it's a CAN error from
			// power-on


			// 1a. reset FAULT
			cout << "\t1a. reset FAULT" << endl;
			cout << "\tchange state of node: " << (int) ID << endl;
			changeEPOSstate(6);

			// 1b. check status
			if (checkEPOSstate() == 11) {
				cout << "\t1b. check status" << endl;
				cout << "\tstate of node: " << (int) ID << endl;
				fprintf(stderr, "\n\tEPOS still in FAULT state, quit!\n");
				exit(1);
			} else
				printf("\tsuccess for node %x!\n", ID); // now in state 'switch on disabled'
		}

		else {
			printf("\tEPOS in state %d, issuing 'disable voltage'\n", n);
			cout << "\tchange state of node: " << (int) ID << endl;
			changeEPOSstate(2); // now in state 'switch on disabled'
		}

		// 2. we should now be in 'switch on disabled' (2)
		cout << "\t2. we should now be in 'switch on disabled'" << endl;
		cout << "\tstate of node: " << (int) ID << endl;
		n = checkEPOSstate();
		if (n != 2) {

			printf(
					"\tEPOS is still NOT in 'switch on disabled' state, but in %d -> quit!\n",
					n);
			printf("\t(%s(), %s line %d)\n", __func__, __FILE__, __LINE__);
			// cout << "EPOS is NOT in 'switch on disabled' state" << endl <<"trying to reconnect second time! " << endl
			//       << "Number of faults: " << CEposVMC::faultsCounter + 1 << endl;
			//  SecondInitDevice(CEposVMC::numOfArgs, CEposVMC::argument);
			exit(1);

		}
		// issue a 'shutdown'
		cout << "\tchange state of node: " << (int) ID << endl;
		changeEPOSstate(0); // should now be in state 'ready to switch on (3)'


		// 3. switch on
		cout << "\t3. switch on" << endl;
		cout << "\tstate of node: " << (int) ID << endl;
		n = checkEPOSstate();
		if (n != 3) {
			printf(
					"\tEPOS is NOT in 'ready to switch on' state, but in %d -> quit!\n",
					n);
			printf("\t(%s(), %s line %d)\n", __func__, __FILE__, __LINE__);
			exit(1);
		}
		printf("\n\tswitching on... ");
		cout << "\tchange state of node: " << (int) ID << endl;
		changeEPOSstate(1); // should now be in state 'switched on (4)'


		// 4. enable operation
		cout << "\t4. enable operation" << endl;
		cout << "\tstate of node: " << (int) ID << endl;
		n = checkEPOSstate();
		if (n != 4) {
			printf(
					"\n\tEPOS is NOT in 'switched on' state, but in state %d -> quit!\n",
					n);
			printf("\t( %s(), %s line %d)\n", __func__, __FILE__, __LINE__);
			exit(1);
		}
		printf("\n\tenable operation, ");

		cout << "\tchange state of node: " << (int) ID << endl;
		changeEPOSstate(5);

		cout << "\tstate of node: " << (int) ID << endl;
		n = checkEPOSstate();
		if (n != 7) {
			printf(
					"\n\tEPOS is NOT in 'Operation enable' state, but in state %d -> quit!\n",
					n);
			printf("\t( %s(), %s line %d)\n", __func__, __FILE__, __LINE__);
			exit(1);
		}

		// ok, up and running...
		cout << "\tEPOS state: " << n << endl;
		//printf("EPOS with node ID %x is up and running, have fun!\n", ID);
		//printf("********************************************\n");
		faultsCounter = 0;
		ID++;
		quantity--;
	} while (quantity >= 1);

	PrintHelp();
}

int CEposVMC::SecondInitDevice(int argc, char **arg) {

	ep = -1;
	sleep(2);
	CEposVMC::faultsCounter++;

	if (argc != 2) {
		Help(arg[0]);
	}
	/* open EPOS device */

	// check wether connection is made via TCP/IP or ser. device

	// is argument starting with a '/'?
	if (!strncmp(arg[1], "/", 1)) {
		printf("trying to open '%s'...", arg[1]);
		if (openEPOS(arg[1]) < 0) {
			cout << "error openEPOS" << endl;
			exit(-1);
		}
		printf("done.\n");
	}
	// is argument starting with a digit? => assume IP adress!
	else if (isdigit(*arg[1])) {
		//    printf("%s\n", arg[1]);
		string_pos = strspn(arg[1], "1234567890.");
		if (string_pos == strlen(arg[1])) {
			fprintf(stderr, "TCP/IP PORT NOT GIVEN!\n");
			return (1);
		}
		//    fprintf(stderr,">>>%d<<<\n", string_pos);
		strncpy(ip, arg[1], string_pos);
		ip[string_pos] = '\0';
		//printf("%s; %d  -- >%s<\n", ip, string_pos, strpbrk(arg[1],":"));

		if (sscanf(strpbrk(arg[1], ":"), ":%hu", &port) == EOF) {
			fprintf(stderr, "TCP/IP PORT NOT GIVEN!\n");
			return (1);
		}
		printf("trying to open %s, port %hu...", ip, port);
		if (openTCPEPOS(ip, port) < 0)
			exit(-1);
		printf("done.\n");
	}

	else {
		fprintf(stderr, "INVALID ARGUMET!\n");
		Help(arg[0]);
	}

	/******************************************************
	 switch on
	 ******************************************************/

	// 1. check if we are in FAULT state
	n = checkEPOSstate(NodeId);
	if (n == 11) {
		printf("EPOS is in FAULT state, doing FAULT RESET... ");
		// should check which fault this is, assume it's a CAN error from
		// power-on


		// 1a. reset FAULT
		changeEPOSstate(6, NodeId);

		// 1b. check status
		if (checkEPOSstate(NodeId) == 11) {
			fprintf(stderr, "\nEPOS still in FAULT state, quit!\n");
			exit(1);
		} else
			printf("success!\n"); // now in state 'switch on disabled'
	}

	else {
		printf("EPOS in state %d, issuing 'disable voltage'\n", n);
		changeEPOSstate(2, NodeId); // now in state 'switch on disabled'
	}

	// 2. we should now be in 'switch on disabled' (2)
	n = checkEPOSstate(NodeId);
	if (n != 2) {
		cout << "check the system! " << endl << "Number of faults: "
				<< CEposVMC::faultsCounter + 1 << endl;
		printf(
				"EPOS is still NOT in 'switch on disabled' state, but in %d -> quit!\n",
				n);
		printf("(%s(), %s line %d)\n", __func__, __FILE__, __LINE__);
		exit(1);
	}
	// issue a 'shutdown'
	changeEPOSstate(0, NodeId); // should now be in state 'ready to switch on (3)'

	return (0);
}

/*
 void CEposVMC::CloseDevice() {

 cout << "initiating shutdown"  << endl;
 changeEPOSstate(2);
 printEPOSstate();
 closeEPOS();
 printf("*****************bye bye!***********************\n");

 }
 */

void CEposVMC::CloseDevice() {

	/*****************************************/
	/*  all done, shutting off               */
	/*****************************************/

	/*   printf("\initiating shutdown "); */
	int quantity = nodesQuantity;
	BYTE ID = NodeId;
	cout << "initiating shutdown" << endl;
	do {
		cout << "changing epos state of node: " << (int) ID << endl;
		changeEPOSstate(2, ID);
		cout << "printing epos state of node: " << (int) ID << endl;
		printEPOSstate(ID);
		ID++;
		quantity--;
	} while (quantity >= 1);

	closeEPOS();
	printf("*****************bye bye!***********************\n");

}

/************************************************************/
/*            open/close device                             */
/************************************************************/

// von hans (atomsps.c, opensps() )
/*! establish the connection to EPOS via RS232 connection

 \param dev string describing the device on which the EPOS is connected
 to, e.g. "/dev/ttyS0"

 \retval 0 success
 \retval -1 failure

 */
int CEposVMC::openEPOS(char* device) { //pure rs232
	struct termios options;
	int i;

	/* EPOS transfer format is:
	 1 start bit
	 8 data bits
	 no parity
	 1 stop bit
	 */

	if (ep >= 0) {
		cout << "ep>=0 | " << ep << endl;
		return (-1);
	}

	for (i = 0; i < 5; i++) {
		if ((ep = open(device, O_RDWR | O_NOCTTY | O_NDELAY)) >= 0)
			break;
		sleep(1);
	}

	if (ep == -1) {
		perror("open serial port");
		return (-1);
	}

	if (tcgetattr(ep, &options) < 0) {
		perror("tcgetattr");
		return (-1);
	}

	memset(&options, 0, sizeof(options));

	options.c_cflag |= B115200;//B38400;
	options.c_cflag |= CS8; //8 bits per byte


	options.c_cflag |= CLOCAL | CREAD;

	tcflush(ep, TCIFLUSH);

	if (tcsetattr(ep, TCSANOW, &options) < 0) {
		perror("tcsetattr");
		return (-1);
	}

	if (fcntl(ep, F_SETFL, FNDELAY) < 0) { //FNDELAY enyspricht grob O_NONBLOCK
		perror("fcntl");
		return (-1);
	}

	return (0);
}

/*! establish the connection to EPOS via a TCP/IP tunneled RS232
 connection

 \param ip string describing the IP address on which the EPOS is
 connected to, e.g. "192.168.1.100"

 \param port short unsigned int giving the TCP port number on the
 device 'IP'

 \retval 0 success
 \retval -1 failure

 */
int CEposVMC::openTCPEPOS(char* ip, short unsigned port) {

	struct sockaddr_in address;
	const int y = 1;

	if ((ep = socket(AF_INET, SOCK_STREAM, 0)) > 0) {
		setsockopt(ep, SOL_SOCKET, SO_REUSEADDR, &y, sizeof(int));
		printf("socket open.\n");
	} else {
		perror("socket");
		return (-1);
	}

	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	inet_aton(ip, &address.sin_addr); //

	if (connect(ep, (struct sockaddr *) &address, sizeof(address)) == 0)
		printf("connection to %s  established.\n", inet_ntoa(address.sin_addr));
	else {
		fprintf(
				stderr,
				"connect() to >%s<, port >%d< failed in %s()\
 (file %s, line %d)\n\n",
				inet_ntoa(address.sin_addr), port, __func__, __FILE__, __LINE__);
		return (-1);
	}

	// set socket to non-blocking mode
	if (fcntl(ep, F_SETFD, O_NONBLOCK) < 0) {
		perror("fcntl");
		return (-1);
	}

	return (0);
}

/*! closes connection socket of EPOS device
 \retval 0 success
 \retval -1 failure
 */
int CEposVMC::closeEPOS() {
	return (close(ep));
}

/*! check whether the socket connection to EPOS is established
 \retval 0 success
 \retval -1 failure
 */
int CEposVMC::checkEPOS() {
	if (ep < 0) {
		fprintf(stderr, "ERROR: EPOS device not open!");
		return (-1);
	}
	return (0);
}

/************************************************************/
/*          high-level read functions */
/************************************************************/

/*! read EPOS status word

 \param status pointer to WORD, the content of EPOS statusword will be placed there

 \retval 0 success
 \retval -1 failure

 */
int CEposVMC::readStatusword(WORD* eposStatus) {

	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6041, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}

	// check error code
	if ((n = checkEPOS()) < 0)
		return n;

#ifdef DEBUG
	printf("==> EPOS status word: %#06x\n", answer[3]);
#endif
	*eposStatus = answer[3];
	free(answer); // memory is allocated by readAnswer()
	return (0);
}

int CEposVMC::readStatusword(WORD* eposStatus, BYTE nodeAddress) {

	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6041, nodeAddress, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}

	// check error code
	if ((n = checkEPOS()) < 0)
		return n;

#ifdef DEBUG
	printf("==> EPOS status word: %#06x\n", answer[3]);
#endif
	*eposStatus = answer[3];
	free(answer); // memory is allocated by readAnswer()
	return (0);
}

/*! pretty-print Statusword to stdout

 \param s WORD variable holding the statusword

 */
int CEposVMC::printEPOSstatusword(WORD statusword) {

	printf("\nmeaning of EPOS statusword %#06x is:\n", statusword);

	printf("15: position referenced to home position: ");
	if ((statusword & E_BIT15) == E_BIT15)
		printf("true\n");
	else
		printf("false\n");

	printf("14: refresh cycle of power stage:         ");
	if ((statusword & E_BIT14) == E_BIT14)
		printf("true\n");
	else
		printf("false\n");

	printf("13: OpMode specific, some error:          ");
	if ((statusword & E_BIT13) == E_BIT13)
		printf("true\n");
	else
		printf("false\n");

	printf("12: OpMode specific:                      ");
	if ((statusword & E_BIT12) == E_BIT12)
		printf("true\n");
	else
		printf("false\n");

	printf("11: NOT USED                              ");
	if ((statusword & E_BIT11) == E_BIT11)
		printf("true\n");
	else
		printf("false\n");

	printf("10: Target reached:                       ");
	if ((statusword & E_BIT10) == E_BIT10)
		printf("true\n");
	else
		printf("false\n");

	printf("09: Remote (?)                            ");
	if ((statusword & E_BIT09) == E_BIT09)
		printf("true\n");
	else
		printf("false\n");

	printf("08: offset current measured (?)           ");
	if ((statusword & E_BIT08) == E_BIT08)
		printf("true\n");
	else
		printf("false\n");

	printf("07: WARNING                               ");
	if ((statusword & E_BIT07) == E_BIT07)
		printf("true\n");
	else
		printf("false\n");

	printf("06: switch on disable                     ");
	if ((statusword & E_BIT06) == E_BIT06)
		printf("true\n");
	else
		printf("false\n");

	printf("05: quick stop                            ");
	if ((statusword & E_BIT05) == E_BIT05)
		printf("true\n");
	else
		printf("false\n");

	printf("04: voltage enabled                       ");
	if ((statusword & E_BIT04) == E_BIT04)
		printf("true\n");
	else
		printf("false\n");

	printf("03: FAULT                                 ");
	if ((statusword & E_BIT03) == E_BIT03)
		printf("true\n");
	else
		printf("false\n");

	printf("02: operation enable                      ");
	if ((statusword & E_BIT02) == E_BIT02)
		printf("true\n");
	else
		printf("false\n");

	printf("01: switched on                           ");
	if ((statusword & E_BIT01) == E_BIT01)
		printf("true\n");
	else
		printf("false\n");

	printf("00: ready to switch on                    ");
	if ((statusword & E_BIT00) == E_BIT00)
		printf("true\n");
	else
		printf("false\n");

	return (0);
}

/*! check EPOS state, firmware spec 8.1.1

 \return EPOS status as defined in firmware specification 8.1.1

 */
int CEposVMC::checkEPOSstate() {

	WORD w = 0x0;
	int n;

	if ((n = readStatusword(&w)) < 0) {
		fprintf(stderr, " *** %s: readStatusword() returned %d **\n", __func__,
				n);
		return (-1);
	}

	/* state 'start' (0)
	 fedc ba98  7654 3210
	 w == x0xx xxx0  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06)
			&& !bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (0);

	/* state 'not ready to switch on' (1)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
			&& !bitcmp(w, E_BIT14))
		return (1);

	/* state 'switch on disabled' (2)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x100 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
			&& !bitcmp(w, E_BIT14))
		return (2);

	/* state 'ready to switch on' (3)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0001 */
	if (bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (3);

	/* state 'switched on' (4)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (4);

	/* state 'refresh' (5)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (5);

	/* state 'measure init' (6)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x011 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (6);

	/* state 'operation enable' (7)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x011 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (7);

	/* state 'quick stop active' (8)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (8);

	/* state 'fault reaction active (disabled)' (9)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (9);

	/* state 'fault reaction active (enabled)' (10)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (10);

	/* state 'fault' (11)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (11);

	// if we get down here, statusword has a unknown value!
	fprintf(stderr, "WARNING: EPOS status word %#06x is an unkown state!\n", w);
	fprintf(stderr, "(function %s() in file %s, line %d)\n", __func__,
			__FILE__, __LINE__);

	return (-2);
}

int CEposVMC::checkEPOSstate(BYTE nodeAddress) {

	WORD w = 0x0;
	int n;

	if ((n = readStatusword(&w, nodeAddress)) < 0) {
		fprintf(stderr, " *** %s: readStatusword() returned %d **\n", __func__,
				n);
		return (-1);
	}

	/* state 'start' (0)
	 fedc ba98  7654 3210
	 w == x0xx xxx0  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06)
			&& !bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (0);

	/* state 'not ready to switch on' (1)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
			&& !bitcmp(w, E_BIT14))
		return (1);

	/* state 'switch on disabled' (2)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x100 0000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04)
			&& !bitcmp(w, E_BIT05) && bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08)
			&& !bitcmp(w, E_BIT14))
		return (2);

	/* state 'ready to switch on' (3)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0001 */
	if (bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (3);

	/* state 'switched on' (4)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (4);

	/* state 'refresh' (5)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x010 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (5);

	/* state 'measure init' (6)
	 fedc ba98  7654 3210
	 w == x1xx xxx1  x011 0011 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && bitcmp(w, E_BIT14))
		return (6);

	/* state 'operation enable' (7)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x011 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (7);

	/* state 'quick stop active' (8)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 0111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& !bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (8);

	/* state 'fault reaction active (disabled)' (9)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (9);

	/* state 'fault reaction active (enabled)' (10)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x001 1111 */
	if (bitcmp(w, E_BIT00) && bitcmp(w, E_BIT01) && bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (10);

	/* state 'fault' (11)
	 fedc ba98  7654 3210
	 w == x0xx xxx1  x000 1000 */
	if (!bitcmp(w, E_BIT00) && !bitcmp(w, E_BIT01) && !bitcmp(w, E_BIT02)
			&& bitcmp(w, E_BIT03) && !bitcmp(w, E_BIT04) && !bitcmp(w, E_BIT05)
			&& !bitcmp(w, E_BIT06) && bitcmp(w, E_BIT08) && !bitcmp(w, E_BIT14))
		return (11);

	// if we get down here, statusword has a unknown value!
	fprintf(stderr, "WARNING: EPOS status word %#06x is an unkown state!\n", w);
	fprintf(stderr, "(function %s() in file %s, line %d)\n", __func__,
			__FILE__, __LINE__);

	return (-2);
}

/* pretty-print EPOS state */
int CEposVMC::printEPOSstate() {

	printf("\tEPOS is in state ");

	switch (checkEPOSstate()) {
	case 0:
		printf("start\n");
		break;
	case 1:
		printf("Not ready to switch on.\n");
		break;
	case 2:
		printf("Switch on disabled.\n");
		break;
	case 3:
		printf("Ready to switch on.\n");
		break;
	case 4:
		printf("Switched on.\n");
		break;
	case 5:
		printf("Refresh.\n");
		break;
	case 6:
		printf("Measure init.\n");
		break;
	case 7:
		printf("Operation enable.\n");
		break;
	case 8:
		printf("Quick stop active\n");
		break;
	case 9:
		printf("Fault reaction active (disabled)\n");
		break;
	case 10:
		printf("Fault reaction active (enabled)\n");
		break;
	case 11:
		printf("FAULT\n");
		break;

	default:
		printf("UNKNOWN!\n");
		return (-1);
	}
	return (0);
}

/* pretty-print EPOS state */
int CEposVMC::printEPOSstate(BYTE nodeAddress) {

	printf("\nEPOS is in state ");

	switch (checkEPOSstate()) {
	case 0:
		printf("start\n");
		break;
	case 1:
		printf("Not ready to switch on.\n");
		break;
	case 2:
		printf("Switch on disabled.\n");
		break;
	case 3:
		printf("Ready to switch on.\n");
		break;
	case 4:
		printf("Switched on.\n");
		break;
	case 5:
		printf("Refresh.\n");
		break;
	case 6:
		printf("Measure init.\n");
		break;
	case 7:
		printf("Operation enable.\n");
		break;
	case 8:
		printf("Quick stop active\n");
		break;
	case 9:
		printf("Fault reaction active (disabled)\n");
		break;
	case 10:
		printf("Fault reaction active (enabled)\n");
		break;
	case 11:
		printf("FAULT\n");
		break;

	default:
		printf("UNKNOWN!\n");
		return (-1);
	}
	return (0);
}

/* change EPOS state according to firmware spec 8.1.3 */
int CEposVMC::changeEPOSstate(int state) {
	WORD dw[2];
	int n;

	dw[1] = 0x0000; // high WORD of DWORD is not used here

	/* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
	 this way, but does NOT work otherways! -- mh, 07.07.06
	 */

	dw[0] = 0x0000;

	switch (state) {
	case 0: //shutdown, controlword: 0xxx x110
		dw[0] &= ~E_BIT15; // bit 15 ->0
		dw[0] |= E_BIT02; // bit 02 ->1
		dw[0] |= E_BIT01;
		dw[0] &= ~E_BIT00;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 1: // switch on, controllword: 0xxx x111
		dw[0] &= ~E_BIT15;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 2: // disable voltage, controllword: 0xxx xx0x
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT02;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 3: // quick stop, controllword: 0xxx x01x
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT02;
		dw[0] |= E_BIT02;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 4: // disable operation, controllword: 0xxx 0111
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT03;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 5: // enable operation, controllword: 0xxx 1111
		dw[0] &= ~E_BIT15;
		dw[0] |= E_BIT03;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 6: // fault reset, controllword: 1xxx xxxx

		//dw[0] |= E_BIT15; this is according to firmware spec 8.1.3,
		//but does not work!
		dw[0] |= E_BIT07; // this is according to firmware spec 14.1.57
		// and IS working!


		/*       WORD estatus = 0x0; */
		/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
		/*       printEPOSstatusword(estatus); */

		n = WriteObject(0x6040, NodeId, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
		/*       printEPOSstatusword(estatus); */

		break;

	default:
		fprintf(stderr, "ERROR: demanded state %d is UNKNOWN!\n", state);
		return (-1);
	}
	return (0);
}

int CEposVMC::changeEPOSstate(int state, BYTE nodeAddress) {
	WORD dw[2];
	int n;

	dw[1] = 0x0000; // high WORD of DWORD is not used here

	/* ! DO NOT READ OLD CONTROLWORD BACK, JUST SET THE BITS. It works
	 this way, but does NOT work otherways! -- mh, 07.07.06
	 */

	dw[0] = 0x0000;

	switch (state) {
	case 0: //shutdown, controlword: 0xxx x110
		dw[0] &= ~E_BIT15; // bit 15 ->0
		dw[0] |= E_BIT02; // bit 02 ->1
		dw[0] |= E_BIT01;
		dw[0] &= ~E_BIT00;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 1: // switch on, controllword: 0xxx x111
		dw[0] &= ~E_BIT15;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 2: // disable voltage, controllword: 0xxx xx0x
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT02;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 3: // quick stop, controllword: 0xxx x01x
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT02;
		dw[0] |= E_BIT02;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 4: // disable operation, controllword: 0xxx 0111
		dw[0] &= ~E_BIT15;
		dw[0] &= ~E_BIT03;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 5: // enable operation, controllword: 0xxx 1111
		dw[0] &= ~E_BIT15;
		dw[0] |= E_BIT03;
		dw[0] |= E_BIT02;
		dw[0] |= E_BIT01;
		dw[0] |= E_BIT00;

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}
		break;

	case 6: // fault reset, controllword: 1xxx xxxx

		//dw[0] |= E_BIT15; this is according to firmware spec 8.1.3,
		//but does not work!
		dw[0] |= E_BIT07; // this is according to firmware spec 14.1.57
		// and IS working!


		/*       WORD estatus = 0x0; */
		/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
		/*       printEPOSstatusword(estatus); */

		n = WriteObject(0x6040, nodeAddress, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		/*       if ( ( n = readStatusword(&estatus) ) < 0) checkEPOSerror(); */
		/*       printEPOSstatusword(estatus); */

		break;

	default:
		fprintf(stderr, "ERROR: demanded state %d is UNKNOWN!\n", state);
		return (-1);
	}
	return (0);
}

/* returns software version as HEX  --  14.1.33*/
int CEposVMC::readSWversion() {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x2003, NodeId, 0x01, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("=x=> SW-Version: %x\n", answer[3]);
#endif

	n = (int) answer[3];

	free(answer); // memory is allocated by readAnswer()
	return (n);
}

int CEposVMC::readSWversion(BYTE nodeAddress) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x2003, nodeAddress, 0x01, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("=x=> SW-Version: %x\n", answer[3]);
#endif

	n = (int) answer[3];

	free(answer); // memory is allocated by readAnswer()
	return (n);
}

/* read digital input functionality polarity -- firmware spec 14.1.47 */
int CEposVMC::readDInputPolarity(WORD* w) {

	WORD *answer = NULL;
	int n = 0;

	checkEPOS();
	checkPtr(&answer);

	if ((n = ReadObject(0x2071, NodeId, 0x03, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("==> polarity mask: %x\n", answer[3]);
#endif

	*w = answer[3];

	free(answer); // memory is allocated by readAnswer()
	return (0);
}

int CEposVMC::readDInputPolarity(WORD* w, BYTE nodeAddress) {

	WORD *answer = NULL;
	int n = 0;

	checkEPOS();
	checkPtr(&answer);

	if ((n = ReadObject(0x2071, nodeAddress, 0x03, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("==> polarity mask: %x\n", answer[3]);
#endif

	*w = answer[3];

	free(answer); // memory is allocated by readAnswer()
	return (0);
}

/* set home switch polarity -- firmware spec 14.1.47 */
int CEposVMC::setHomePolarity(int pol) {
	WORD* answer = NULL;
	WORD mask = 0x00;
	WORD dw[2] = { 0x0, 0x0 };
	int n = 0;

	if (pol != 0 && pol != 1) {
		fprintf(stderr,
				"ERROR: polarity must be 0 (hight active) or 1 (low active)\n");
		return (-1);
	}

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	// read present functionalities polarity mask
	if (readDInputPolarity(&mask)) {
		fprintf(stderr, "\aERROR while reading digital input polarity!\n");
		return (-2);
	}

	// set bit 2 (==home switch) to 0 or 1:
	if (pol == 0)
		mask &= ~E_BIT02;
	else if (pol == 1)
		mask |= E_BIT02;

	dw[1] = 0x0000; // high WORD of DWORD is not used here
	dw[0] = mask;

	n = WriteObject(0x2071, NodeId, 0x03, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	return (0);
}

int CEposVMC::setHomePolarity(int pol, BYTE nodeAddress) {
	WORD* answer = NULL;
	WORD mask = 0x00;
	WORD dw[2] = { 0x0, 0x0 };
	int n = 0;

	if (pol != 0 && pol != 1) {
		fprintf(stderr,
				"ERROR: polarity must be 0 (hight active) or 1 (low active)\n");
		return (-1);
	}

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	// read present functionalities polarity mask
	if (readDInputPolarity(&mask, nodeAddress)) {
		fprintf(stderr, "\aERROR while reading digital input polarity!\n");
		return (-2);
	}

	// set bit 2 (==home switch) to 0 or 1:
	if (pol == 0)
		mask &= ~E_BIT02;
	else if (pol == 1)
		mask |= E_BIT02;

	dw[1] = 0x0000; // high WORD of DWORD is not used here
	dw[0] = mask;

	n = WriteObject(0x2071, nodeAddress, 0x03, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	return (0);
}

/* read EPOS control word (firmware spec 14.1.57) */
int CEposVMC::readControlword(WORD* w) {

	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6040, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("==> EPOS control word: %#06x\n", answer[3]);
#endif
	*w = answer[3];
	free(answer); // memory is allocated by readAnswer()
	return (0);
}

int CEposVMC::readControlword(WORD* w, BYTE nodeAddress) {

	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6040, nodeAddress, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}

	// check error code
	checkEPOSerror();

#ifdef DEBUG
	printf("==> EPOS control word: %#06x\n", answer[3]);
#endif
	*w = answer[3];
	free(answer); // memory is allocated by readAnswer()
	return (0);
}

/* pretty-print Controlword */
int CEposVMC::printEPOScontrolword(WORD controlword) {
	printf("\nmeaning of EPOS controlword %#06x is:\n", controlword);
	// bit 15..11 not in use
	// bit 10, 9 reserved
	printf("  HALT:                                 ");
	if ((controlword & E_BIT08) == E_BIT08)
		printf("true\n");
	else
		printf("false\n");

	printf("  fault reset                           ");
	if ((controlword & E_BIT07) == E_BIT07)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((controlword & E_BIT06) == E_BIT06)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((controlword & E_BIT05) == E_BIT05)
		printf("true\n");
	else
		printf("false\n");

	printf("  Op mode specific                      ");
	if ((controlword & E_BIT04) == E_BIT04)
		printf("true\n");
	else
		printf("false\n");

	printf("  enable operation                      ");
	if ((controlword & E_BIT03) == E_BIT03)
		printf("true\n");
	else
		printf("false\n");

	printf("  quick stop                            ");
	if ((controlword & E_BIT02) == E_BIT02)
		printf("true\n");
	else
		printf("false\n");

	printf("  enable voltage                        ");
	if ((controlword & E_BIT01) == E_BIT01)
		printf("true\n");
	else
		printf("false\n");

	printf("  switch on                             ");
	if ((controlword & E_BIT00) == E_BIT00)
		printf("true\n");
	else
		printf("false\n");

	return (0);
}

/* set mode of operation --- 14.1.59 */
int CEposVMC::setOpMode(int OpMode) {

	WORD dw[2] = { 0x0, 0x0 };
	int n = 0;

	dw[1] = 0x0000; // high WORD of DWORD is not used here
	dw[0] = OpMode;

	n = WriteObject(0x6060, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	return (0);
}

int CEposVMC::setOpMode(int OpMode, BYTE nodeAddress) {

	WORD dw[2] = { 0x0, 0x0 };
	int n = 0;

	dw[1] = 0x0000; // high WORD of DWORD is not used here
	dw[0] = OpMode;

	n = WriteObject(0x6060, nodeAddress, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	return (0);
}

/** read mode of operation --- 14.1.60

 \return RETURN(0) MEANS ERROR! -1 is a valid OpMode, but 0 is not!

 */
int CEposVMC::readOpMode() {
	WORD *answer = NULL;
	//short int *i;
	int8_t aa;
	int n = 0;

	//cout<< "readopMode"<<endl;
	if ((n = ReadObject(0x6061, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (0);
	}
	//cout<< "ReadObject return "<< n <<endl;

	//w = answer[3];
	aa = answer[3];
	free(answer);
	// check error code
	checkEPOSerror();

	/*
	 // return value is a 8bit signed integer. To convert it to a 16bit
	 // signed int, move bit 7 (old signum) to bit 15 (new signum). Set
	 // bit 7 to 0
	 if ((w & (~E_BIT07)) == 0xFFFF){     //bit 07 is set
	 w &= ~E_BIT07;    // set bit  7 to '0'
	 w |= E_BIT15;     // set bit 15 to '1'

	 }
	 // the compiler must give a warning about signedness here, but it is
	 // ok! Don't know how to suppress it...
	 i = &w;

	 */
	// give warning, if internal mode is used
	if (aa < 0)
		fprintf(
				stderr,
				"WARNING: EPOS is set to internal mode of operation (%hd).\n Make sure that this was really intended!\n",
				aa);

	//return(*i);
	return (aa);
}

int CEposVMC::readOpMode(BYTE nodeAddress) {
	WORD *answer = NULL;
	//short int *i;
	int8_t aa;
	int n = 0;

	//cout<< "readopMode"<<endl;
	if ((n = ReadObject(0x6061, nodeAddress, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (0);
	}
	//cout<< "ReadObject return "<< n <<endl;

	//w = answer[3];
	aa = answer[3];
	free(answer);
	// check error code
	checkEPOSerror();

	/*
	 // return value is a 8bit signed integer. To convert it to a 16bit
	 // signed int, move bit 7 (old signum) to bit 15 (new signum). Set
	 // bit 7 to 0
	 if ((w & (~E_BIT07)) == 0xFFFF){     //bit 07 is set
	 w &= ~E_BIT07;    // set bit  7 to '0'
	 w |= E_BIT15;     // set bit 15 to '1'

	 }
	 // the compiler must give a warning about signedness here, but it is
	 // ok! Don't know how to suppress it...
	 i = &w;

	 */
	// give warning, if internal mode is used
	if (aa < 0)
		fprintf(
				stderr,
				"WARNING: EPOS is set to internal mode of operation (%hd).\n Make sure that this was really intended!\n",
				aa);

	//return(*i);
	return (aa);
}

/* read demand position; 14.1.61 */
int CEposVMC::getDemandPosition(long* dpos) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6062, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*dpos = answer[3] | (answer[4] << 16);
	free(answer);
#ifdef DEBUG
	printf("==> EPOS actual position: %ld\n", *dpos);
#endif
	return (0);
}

/*! read actual position; firmware description 14.1.62

 \retval 0 success
 \retval <0 some error, check with checkEPOSerror()
 */

int CEposVMC::getActualPosition(long* apos) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6064, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*apos = answer[3] | (answer[4] << 16);
	free(answer);
#ifdef DEBUG
	printf("==> %s(): EPOS actual position: %ld\n", __func__, *apos);
#endif

	return (0);
}

/* read position window; 14.1.64 */
int CEposVMC::readPositionWindow(unsigned long int* posw) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6067, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*posw = answer[3] | (answer[4] << 16);
	free(answer);

#ifdef DEBUG
	printf("==> %s(): EPOS position window is %ld\n", __func__, *posw);
#endif

	return (0);
}

/* write  position window; 14.1.64 */
int CEposVMC::writePositionWindow(unsigned long int posWindow) {

	WORD dw[2];
	int n = 0;

	// write intended position window
	dw[0] = (WORD) (posWindow & 0x0000FFFF);
	dw[1] = (WORD) (posWindow >> 16);

	n = WriteObject(0x6067, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

/* read demand position; 14.1.67 */
int CEposVMC::getDemandVelocity(long* dvel) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x606b, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*dvel = answer[3] | (answer[4] << 16);

#ifdef DEBUG
	printf("==> EPOS demand velocity: %ld\n", *dvel);
#endif

	return (0);
}

/* read actual position; 14.1.68 */
int CEposVMC::getActualVelocity(long* avel) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x606c, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*avel = answer[3] | (answer[4] << 16);

#ifdef DEBUG
	printf("==> EPOS actual velocity: %ld\n", *avel);
#endif

	return (0);
}

/*! read actual motor current, see firmware description 14.1.69

 \param val pointer to short int where the actual motor current will be
 placed.

 \retval 0 success
 \retval -1 error

 */
int CEposVMC::getActualCurrent(short int* current) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x6078, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	*current = answer[3];
	free(answer);
#ifdef DEBUG
	printf("==> EPOS actual current: %dmA\n", *current);
#endif

	return (0);
}

/*!  read EPOS target position; firmware description 14.1.70

 \param val pointer to long int, will be filled with EPOS target position
 \retval 0 success
 \retval -1 error

 */
int CEposVMC::getTargetPosition(long* tpos) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	checkPtr(&answer);

	if ((n = ReadObject(0x607a, NodeId, 0x00, &answer)) < 0) {

		fprintf(stderr, " *** %s: ReadObject() returned %d **\n", __func__, n);
		free(answer); // memory is allocated by readAnswer()
		return (-1);
	}
	// check error code
	checkEPOSerror();

	// return value is a 32bit integer (==long int)
	*tpos = (DWORD) answer[3] | (answer[4] << 16);
	free(answer);
#ifdef DEBUG
	printf("==> EPOS target position: %ld\n", *tpos);
#endif

	return (0);
}

/*! readDeviceName: read manufactor device name string firmware

 \param str previously allocated string, will be filled with device name
 \retval 0 success
 \retval -1 error


 */
int CEposVMC::readDeviceName(char* name) {
	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;
	memset(&answer, 0, sizeof(answer));

	if ((n = ReadObject(0x1008, NodeId, 0x00, &answer)) < 0) {
		printf(" *** readObject returned %d at %s, line %d ***\n", n, __func__,
				__LINE__);
	}

	name[0] = (answer[3] & 0x00FF);
	name[1] = (answer[3] & 0xFF00) >> 8;
	name[2] = (answer[4] & 0x00FF);
	name[3] = (answer[4] & 0xFF00) >> 8;
	name[4] = '\0'; // end of string

#ifdef DEBUG
	printf("%s: %s \n", __func__, name);
#endif

	free(answer); // memory is allocated by readAnswer()
	return (0);
}

/* firmware spec 14.1.35 */
int CEposVMC::readRS232timeout() {

	WORD *answer = NULL;
	int n = 0;

	if ((n = checkEPOS()) < 0)
		return n;

	if ((n = ReadObject(0x2005, NodeId, 0x00, &answer)) < 0) {
		printf(" *** readObject returned %d at %s, line %d ***\n", n, __func__,
				__LINE__);
	}

#ifdef DEBUG
	printf("%s: RS232 timeout is %d msec\n", __func__, answer[3]);
#endif

	n = (int) answer[3];
	free(answer); // memory is allocated by ReadAnswer()
	return (n);
}

/* run the HomingMode, get the coordinate system zeropoint correct

 this is done as shown in "EPOS Application Note: device Programming,
 3: Homing Mode"

 */
int CEposVMC::doHoming(int method, long int start) {

	WORD dw[2] = { 0x0000, 0x0000 };
	WORD w = 0x0000;
	int n, status = 0;
	//move motor to a pre-defined position before the reference
	//point. This will speed-up things if the coordinates are not too
	//wrong.

	if (moveAbsolute(start, 1000, 3000, 3000, 6400)) {
		fprintf(stderr, "ERROR: could not move to homing starting point!\n");
		fprintf(stderr, "       (problem at %s; %s line %d)\n", __func__,
				__FILE__, __LINE__);
		return (-1);
	}
	// wait for positioning to finish, set timeout to approx. 30sec
	// CAUSES BIG PROBLEMS IF WE DO NOT WAIT!
	waitForTarget(30);
	//monitorStatus();


	// switch to homing mode
	if (setOpMode(E_HOMING)) {
		fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
				__FILE__, __LINE__);
		return (-1);
	}

	// homing speeds are left at default values.. (firmware 14.1.86)


	// set homing method
	dw[0] = method; // NO hex number here!
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6098, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	// switch on
	dw[0] = 0x000f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	// start homing mode
	dw[0] = 0x001f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	checkEPOSerror();

	//read/print status
	status = monitorHomingStatus();
	if (status) {
		// something was wrong during homing...
		if (status == 1) {
			fprintf(stderr,
					"We did more that 2 complete turns without finding the home switch!\n");
			fprintf(stderr, "\aDEVICE IS BROKEN!!!\n");
			exit(2);
		} else {
			fprintf(stderr,
					"got %d as response from monitorHoming()...this is BAD!\n",
					status);
			fprintf(stderr, "[ %s: at %s, line %d ]\n", __func__, __FILE__,
					__LINE__);
		}
	}

	readStatusword(&w);
	if ((w & E_BIT13) == E_BIT13) {
		fprintf(stderr, "\a *** got a HomingError! ***\n");
		return (-1);
	}

	if ((w & E_BIT12) == E_BIT12) {
		printf("homing finished!\n");
		return (0);
	} else {
		//  can this be reached? position finished, no homing error but
		//  homing NOT finished? I guess not..
		return (-5);
	}
}

int CEposVMC::moveRelative(long int steps) {

	WORD dw[2];
	int n = 0;

	// check, if we are in Profile Position Mode
	if (readOpMode() != E_PROFPOS) {
		if (setOpMode(E_PROFPOS)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	// write intended target position
	// firmware 14.1.70
	dw[0] = (WORD) (steps & 0x0000FFFF);
	dw[1] = (WORD) (steps >> 16);

	n = WriteObject(0x607A, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	// switch to relative positioning BY WRITING TO CONTROLWORD, finish
	// possible ongoing operation first!  ->maxon applicattion note:
	// device programming 2.1
	dw[0] = 0x005f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::moveAbsolute(long int steps, long int speed, int acceleration,
		int deceleration, int maxVelocity) {

	WORD dw[2];
	int n = 0;

#ifdef DEBUG
	printf("-> %s(): will move to %ld (%#010lx)\n", __func__, steps, steps);
#endif

	// check, if we are in Profile Position Mode
	if (readOpMode() != E_PROFPOS) {
		if (setOpMode(E_PROFPOS)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}
#ifdef DEBUG
	printf("-> OpMode is (now) 'Profile Position Mode'. That's OK!\n");
#endif

	//----------Maximal profile velocity--------------
	dw[0] = (WORD) (maxVelocity & 0x0000FFFF);
	dw[1] = (WORD) (maxVelocity >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x607F, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	// write intended target position, is signed 32bit int
	// firmware 14.1.70
	dw[0] = (WORD) (steps & 0x0000FFFF);
	dw[1] = (WORD) (steps >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x607A, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//------speed--------------------
	// write intended target speed, is signed 32bit int
	// firmware 14.1.70
	dw[0] = (WORD) (speed & 0x0000FFFF);
	dw[1] = (WORD) (speed >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6081, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//------acceleration--------------------

	dw[0] = (WORD) (acceleration & 0x0000FFFF);
	dw[1] = (WORD) (acceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6083, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//------deceleration--------------------

	dw[0] = (WORD) (deceleration & 0x0000FFFF);
	dw[1] = (WORD) (deceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6084, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//-----------enabling operation Control Word----------------------------------
	// switch to absolute positioning, cancel possible ongoing operation
	// first!  ->maxon application note: device programming 2.1
	dw[0] = 0x3f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::profileVelocityMode(long int targetVelocity, int maxVelocity,
		int acceleration, int deceleration) {
	// check, if we are in Profile Velocity Mode

	if (readOpMode() != E_PROFVEL) {
		if (setOpMode(E_PROFVEL)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];
	int n = 0;
	dw[0] = (WORD) (targetVelocity & 0x0000FFFF);
	dw[1] = (WORD) (targetVelocity >> 16);
	n = WriteObject(0x60FF, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	//--------configuration - maximal profile velocity-----

	dw[0] = (WORD) (maxVelocity & 0x0000FFFF);
	dw[1] = (WORD) (maxVelocity >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x607F, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//------acceleration--------------------

	dw[0] = (WORD) (acceleration & 0x0000FFFF);
	dw[1] = (WORD) (acceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6083, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//-------deceleration---------

	dw[0] = (WORD) (deceleration & 0x0000FFFF);
	dw[1] = (WORD) (deceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6084, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	//---------------------------------------------
	// enable operation
	dw[0] = 0xf;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::setVelocity(long int targetVelocity) {
	// check, if we are in Profile Velocity Mode

	if (readOpMode() != E_PROFVEL) {
		cout << "Read of node. Not in Profile Mode" << endl;
		if (setOpMode(E_PROFVEL)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];
	int n = 0;
	dw[0] = (WORD) (targetVelocity & 0x0000FFFF);
	dw[1] = (WORD) (targetVelocity >> 16);
	n = WriteObject(0x60FF, NodeId, 0x00, dw);
	if (debugLevel == 1) {
		cout << "setting the speed to " << targetVelocity << endl;
	}
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	//---------------------------------------------
	// enable operation
	dw[0] = 0x000f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);

}

int CEposVMC::setVelocity(long int targetVelocity, BYTE nodeAddress) {
	// check, if we are in Profile Velocity Mode

	/*
	 struct timeval start, end, end2, end3;

	 long mtime, seconds, useconds;

	 gettimeofday(&start, NULL);
	 */

	if (readOpMode(nodeAddress) != E_PROFVEL) {
		cout << "Read of node. Not in Profile Mode" << endl;
		if (setOpMode(E_PROFVEL, nodeAddress)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	/*
	 gettimeofday(&end, NULL);
	 seconds  = end.tv_sec  - start.tv_sec;
	 useconds = end.tv_usec - start.tv_usec;

	 mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

	 printf("Elapsed time for readOpMode: %ld milliseconds\n", mtime);
	 */

	WORD dw[2];
	int n = 0;
	dw[0] = (WORD) (targetVelocity & 0x0000FFFF);
	dw[1] = (WORD) (targetVelocity >> 16);
	n = WriteObject(0x60FF, nodeAddress, 0x00, dw);
	if (debugLevel == 1) {
		cout << "setting the speed to " << targetVelocity << endl;
	}
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	/*
	 gettimeofday(&end2, NULL);
	 seconds  = end2.tv_sec  - end.tv_sec;
	 useconds = end2.tv_usec - end.tv_usec;

	 mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	 printf("Elapsed time for setting target velocity: %ld milliseconds\n", mtime);
	 */
	//---------------------------------------------
	// enable operation
	dw[0] = 0x000f;
	dw[1] = 0x0000; // high WORD of DWORD is not used here
	n = WriteObject(0x6040, nodeAddress, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);

}

int CEposVMC::setRPM(long int targetVelocity1, long int targetVelocity2,
		BYTE nodeAddress1, BYTE nodeAddress2) {
	// check, if we are in Profile Velocity Mode

	/*
	 struct timeval start, end;
	 long mtime, seconds, useconds;
	 */

	WORD dw[2];
	int n = 0;
	dw[0] = (WORD) (targetVelocity1 & 0x0000FFFF);
	dw[1] = (WORD) (targetVelocity1 >> 16);
	n = WriteObject(0x60FF, nodeAddress1, 0x00, dw);
	if (debugLevel == 1) {
		cout << "setting the speed to " << targetVelocity1 << endl;
	}
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	//        WORD dw[2];
	//	int n = 0;
	dw[0] = (WORD) (targetVelocity2 & 0x0000FFFF);
	dw[1] = (WORD) (targetVelocity2 >> 16);
	n = WriteObject(0x60FF, nodeAddress2, 0x00, dw);
	if (debugLevel == 1) {
		cout << "setting the speed to " << targetVelocity2 << endl;
	}
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}

	//        gettimeofday(&start, NULL);

	//---------------------------------------------
	// enable operation

	if (gMarkerEO == false) {
		//printf("/\b");
		//fflush(stdout);
		dw[0] = 0x000f;
		dw[1] = 0x0000; // high WORD of DWORD is not used here
		n = WriteObject(0x6040, nodeAddress1, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		dw[0] = 0x000f;
		dw[1] = 0x0000; // high WORD of DWORD is not used here
		n = WriteObject(0x6040, nodeAddress2, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		gMarkerEO = true;
	} else {
		dw[0] = 0x000f;
		dw[1] = 0x0000; // high WORD of DWORD is not used here
		n = WriteObject(0x6040, nodeAddress2, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		dw[0] = 0x000f;
		dw[1] = 0x0000; // high WORD of DWORD is not used here
		n = WriteObject(0x6040, nodeAddress1, 0x00, dw);
		if (n < 0) {
			fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
					__func__, n, __FILE__, __LINE__);
			return (-1);
		}

		gMarkerEO = false;
	}

	checkEPOSerror();

	/*
	 gettimeofday(&end, NULL);
	 seconds  = end.tv_sec  - start.tv_sec;
	 useconds = end.tv_usec - start.tv_usec;

	 mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;


	 printf("\n\nElapsed time for setRPM enable operation: %ld milliseconds\n\n\n", mtime);
	 */
	return (0);

}

int CEposVMC::setAcceleration(int acceleration) {

	// check, if we are in Profile Velocity Mode

	if (readOpMode() != E_PROFVEL) {
		if (setOpMode(E_PROFVEL)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];

	//------acceleration--------------------

	dw[0] = (WORD) (acceleration & 0x0000FFFF);
	dw[1] = (WORD) (acceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6083, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::setAcceleration(int acceleration, BYTE nodeAddress) {

	// check, if we are in Profile Velocity Mode

	if (readOpMode(nodeAddress) != E_PROFVEL) {
		if (setOpMode(E_PROFVEL, nodeAddress)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];

	//------acceleration--------------------

	dw[0] = (WORD) (acceleration & 0x0000FFFF);
	dw[1] = (WORD) (acceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6083, nodeAddress, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::setDeceleration(int deceleration) {
	// check, if we are in Profile Velocity Mode

	if (readOpMode() != E_PROFVEL) {
		if (setOpMode(E_PROFVEL)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];

	//-------deceleration---------

	dw[0] = (WORD) (deceleration & 0x0000FFFF);
	dw[1] = (WORD) (deceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6084, NodeId, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

int CEposVMC::setDeceleration(int deceleration, BYTE nodeAddress) {
	// check, if we are in Profile Velocity Mode

	if (readOpMode(nodeAddress) != E_PROFVEL) {
		if (setOpMode(E_PROFVEL, nodeAddress)) {
			fprintf(stderr, "ERROR: problem at %s; %s line %d\n", __func__,
					__FILE__, __LINE__);
			return (-1);
		}
	}

	WORD dw[2];

	//-------deceleration---------

	dw[0] = (WORD) (deceleration & 0x0000FFFF);
	dw[1] = (WORD) (deceleration >> 16);

#ifdef DEBUG
	printf("-> %s(): dw[0,1] = %#06x  %#06x\n", __func__, dw[0], dw[1]);
#endif

	n = WriteObject(0x6084, nodeAddress, 0x00, dw);
	if (n < 0) {
		fprintf(stderr, "%s: writeObject() returned %d at %s, line %d\n",
				__func__, n, __FILE__, __LINE__);
		return (-1);
	}
	checkEPOSerror();

	return (0);
}

// monitor device status
int CEposVMC::monitorStatus() {
	int n;
	long int postarget, posactual, veldemand, velactual;
	short curactual;
	WORD status;
	if (debugLevel == 1) {
		printf("monitorStatus() function is called\n");
	}
	printf(
			"\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
	int i = 0;
	do {
		i++;
		if ((n = getTargetPosition(&postarget))) {
			printf("ERROR while getActualPosition() [%d]\n", n);
			break;
		}
		/*     if  ( (n=getDemandPosition( &posdemand ) ) ){ */
		/*       printf("ERROR while getDemandPosition() [%d]\n", n); */
		/*       break; */
		/*     } */
		if ((n = getActualPosition(&posactual))) {
			printf("ERROR while getActualPosition() [%d]\n", n);
			break;
		}
		if ((n = getDemandVelocity(&veldemand))) {
			printf("ERROR while readDemandVelocity() [%d]\n", n);
			break;
		}
		if ((n = getActualVelocity(&velactual))) {
			printf("ERROR while readActualVelicity() [%d]\n", n);
			break;
		}
		if ((n = getActualCurrent(&curactual))) {
			printf("ERROR while readActualCurrent() [%d]\n", n);
			break;
		}

		printf(
				"\rEPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA",
				postarget, posactual, postarget - posactual, veldemand,
				velactual, curactual);
		fflush(stdout);

		readStatusword(&status);
	} while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!

	// update values a last time to get a nicer output:
	i++;
	if ((n = getTargetPosition(&postarget))) {
		printf("ERROR while getActualPosition() [%d]\n", n);

	}
	if ((n = getActualPosition(&posactual))) {
		printf("ERROR while getActualPosition() [%d]\n", n);
	}
	if ((n = getDemandVelocity(&veldemand))) {
		printf("ERROR while readDemandVelocity() [%d]\n", n);
	}
	if ((n = getActualVelocity(&velactual))) {
		printf("ERROR while readActualVelicity() [%d]\n", n);
	}
	if ((n = getActualCurrent(&curactual))) {
		printf("ERROR while readActualCurrent() [%d]\n", n);
	}

	printf(
			"\r%d EPOS: pos=%+10ld |%+10ld (%ld to go); v= %+4ld | %+4ld[rpm]; I=%+4dmA\n",
			i, postarget, posactual, postarget - posactual, veldemand,
			velactual, curactual);
	printf("target reached\n");

	return (0);
}

int CEposVMC::monitorHomingStatus() {
	int n;
	long int posactual, velactual;
	short curactual;
	WORD status = 0x0;
	if (debugLevel == 1) {
		printf("monitorHomingStatus() function is called\n");
	}
	printf(
			"\nEPOS operating figures (note: update here is done AS FAST AS POSSIBLE!):\n");
	int i = 0;
	do {
		i++;
		if ((n = getActualPosition(&posactual))) {
			printf("ERROR while getActualPosition() [%d]\n", n);
			break;
		}
		if ((n = getActualVelocity(&velactual))) {
			printf("ERROR while readActualVelicity() [%d]\n", n);
			break;
		}
		if ((n = getActualCurrent(&curactual))) {
			printf("ERROR while readActualCurrent() [%d]\n", n);
			break;
		}

		readStatusword(&status);

		printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x ",
				i, posactual, velactual, curactual, status);

		fflush(stdout);

		readStatusword(&status);

		if ((status & E_BIT13) == E_BIT13) {
			printf("\aHOMING ERROR!\n");
			return (-2);
		}

	} while (((status & E_BIT10) != E_BIT10) && ((status & E_BIT12) != E_BIT12));
	// bit 10 says: target reached!, bit 12: homing attained
	//printEPOSstatusword(status);

	i++;
	if ((n = getActualPosition(&posactual))) {
		printf("ERROR while getActualPosition() [%d]\n", n);
	}
	if ((n = getActualVelocity(&velactual))) {
		printf("ERROR while readActualVelicity() [%d]\n", n);
	}
	if ((n = getActualCurrent(&curactual))) {
		printf("ERROR while readActualCurrent() [%d]\n", n);
	}

	readStatusword(&status);

	printf("\r%d EPOS: pos=%+10ld; v =  %+4ldrpm I=%+3dmA status = %#06x\n", i,
			posactual, velactual, curactual, status);
	printf("homing finished! Position should now be '0'\n");

	return (0);
}

/* waits for positoning to finish, argument is timeout in
 seconds. give timeout==0 to disable timeout */
int CEposVMC::waitForTarget(unsigned int t) {

	WORD status;
	unsigned int i = 0, st = (unsigned int) 1e4;//1e4

	do {
		if (t != 0) { // use timeout?
			if (++i > t * 1e2)
				return (1);
		}
		usleep(st);
		readStatusword(&status);
	} while ((status & E_BIT10) != E_BIT10); // bit 10 says: target reached!


	return (0);
}

/*
 *************************************************************
 check EPOS error code
 ****************************************************************
 */

/* check the global variable E_error_new for EPOS error code */
int CEposVMC::checkEPOSerror() {

	switch (E_error_new) {
	case E_NOERR:
		return (0);

	case E_ONOTEX:
		printf("EPOS responds with error: requested object does not exist!\n");
		break;
	case E_SUBINEX:
		printf("EPOS responds with error: requested subindex does not exist!\n");
		break;
	case E_OUTMEM:
		printf("EPOS responds with error: out of memory!\n");
		break;
	case E_NOACCES:
		printf("EPOS responds with error: unsupported access to an object!\n");
		break;
	case E_WRITEONLY:
		printf(
				"EPOS responds with error: attempt to read a write-only object!\n");
		break;
	case E_READONLY:
		printf(
				"EPOS responds with error: attempt to write a read-only object!\n");
		break;
	case E_PARAMINCOMP:
		printf("EPOS responds with error: general parameter incompatibility!\n");
		break;
	case E_INTINCOMP:
		printf(
				"EPOS responds with error: general internal incompatibility in the device!\n");
		break;
	case E_HWERR:
		printf(
				"EPOS responds with error: access failed due to an HARDWARE ERROR!\n");
		break;
	case E_PRAGNEX:
		printf("EPOS responds with error: value range of parameter exeeded!\n");
		break;
	case E_PARHIGH:
		printf(
				"EPOS responds with error: value of parameter written is too high!\n");
		break;
	case E_PARLOW:
		printf(
				"EPOS responds with error: value of parameter written is too low!\n");
		break;
	case E_PARREL:
		printf(
				"EPOS responds with error: maximum value is less than minimum value!\n");
		break;
	case E_NMTSTATE:
		printf("EPOS responds with error: wrong NMT state!\n");
		break;
	case E_RS232:
		printf("EPOS responds with error: rs232 command illegeal!\n");
		break;
	case E_PASSWD:
		printf("EPOS responds with error: password incorrect!\n");
		break;
	case E_NSERV:
		printf("EPOS responds with error: device not in service mode!\n");
		break;
	case E_NODEID:
		printf("EPOS responds with error: error in Node-ID!\n");
		break;
	default:
		fprintf(stderr,
				"EPOS responds with error: unknown EPOS error code: %#lx\n",
				E_error_new);
		break;
	}
	return (-1);
}

/*
 *************************************************************
 basic I/O functions
 ****************************************************************
 */

/*  write a single BYTE to EPOS */
int CEposVMC::writeBYTE(BYTE* c) {
#ifdef DDEBUG
	printf("sending %#04x \n", *c);
#endif
	if (write(ep, c, 1) <= 0) {
		perror("write ");
		return (-1);
	}
	tcdrain(ep); //Blocks until it were sended
	return (0);
}

/*  write a single WORD to EPOS */
int CEposVMC::writeWORD(WORD *w) {
#ifdef DDEBUG
	printf("sending %#06x \n", *w);
#endif

	if (write(ep, w, 2) <= 0) {
		perror("write ");
		return (-1);
	}
	//tcdrain(ep); //Blocks until it were sended
	return (0);
}

/*!  read a single BYTE from EPOS, timeout implemented */
int CEposVMC::readBYTE(BYTE *c) {

	int i, n;

	for (i = 0; i < NTRY; i++) {
		ioctl(ep, FIONREAD, &n);
		if (n > 0) {
			n = read(ep, c, 1);
		}
		int errsv = errno;
		if (n < 0 && errsv != EAGAIN) {
			perror("read ");
			return (-2);
		}

		if (n > 0) {
#ifdef DDEBUG
			printf("<< receiving: %#04x\n", *c);
#endif
			return (0);
		} else { // sleeping and spining coursor "/"
			if (gMarker == 0) {
				//printf("/\b");
				fflush(stdout);
				gMarker = 1;
			} else {
				//printf("\\\b");
				fflush(stdout);
				gMarker = 0;
			}
			usleep(TRYSLEEP); /* sleep 100ms; EPOS gives timeout after 500ms*/
			//  cout << "++++++++we were sleeping " << i << "times +++++++++" << endl;
		}

	}

	// timeout
	return (-1);
}

/*  read a single WORD from EPOS, timeout implemented */
int CEposVMC::readWORD(WORD *w) {

	int i, n;

	for (i = 0; i < NTRY; i++) {
		ioctl(ep, FIONREAD, &n);
		if (n > 0) {
			n = read(ep, w, sizeof(WORD));
		}
		int errsv = errno;
		if (n < 0 && errsv != EAGAIN) {
			perror("read ");
			return (-2);
		}
		if (n > 0) {
#ifdef DDEBUG
			printf("<<  receiving: %#04x\n", *w);
#endif
			return (0);
		} else {
			if (gMarker == 0) {
				//printf("/\b");
				fflush(stdout);
				gMarker = 1;
			} else {
				//printf("\\\b");
				fflush(stdout);
				gMarker = 0;
			}
			usleep(TRYSLEEP); /* sleep 100ms; EPOS gives timeout after 500ms*/
		}
	}
	// timeout
	return (-1);
}

/* copied from EPOS Communication Guide, p.8 */
WORD CEposVMC::CalcFieldCRC(WORD *pDataArray, WORD numberOfWords) {
	WORD shifter, c;
	WORD carry;
	WORD CRC = 0;

	//Calculate pDataArray Word by Word
	while (numberOfWords--) {
		shifter = 0x8000; //Initialize BitX to Bit15
		c = *pDataArray++; //Copy next DataWord to c
		do {
			carry = CRC & 0x8000; //Check if Bit15 of CRC is set
			CRC <<= 1; //CRC = CRC * 2
			if (c & shifter)
				CRC++; //CRC = CRC + 1, if BitX is set in c
			if (carry)
				CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
			shifter >>= 1; //Set BitX to next lower Bit,
			//shifter = shifter/2
		} while (shifter);
	}

	//printf("checksum == %#06x\n", CRC);
	return CRC;
}

/*  send command to EPOS, taking care of all neccessary 'ack' and
 checksum tests*/
int CEposVMC::sendCom(WORD *frame) {
	/*
	 struct timeval start, end;
	 long mtime, seconds, useconds;
	 gettimeofday(&start, NULL);
	 */
	ioctl(ep, TCFLSH);
	BYTE c = 0x00;
	short i, len;
	int n = 0;

	// need LSB of header WORD, contains (len-1). Complete Frame is
	// (len-1) +3 WORDS long
	len = ((frame[0] & 0x00FF)) + 3;
	/*
	 printf("frame[0] = %#x; shifted = %#x; framelength = %d\n",
	 frame[0], (frame[0] & 0x00FF),  len);
	 */

	// add checksum to frame
	frame[len - 1] = CalcFieldCRC(frame, len);

#ifdef DEBUG
	printf(">> ");
	for (i = 0; i < len; ++i) {
		printf("%#06x ", frame[i]);
	}
	printf("\n");
#endif

	/* sending to EPOS */
	//send header:
#ifdef USBMODE
	c = 0x90;
	writeBYTE(&c); //Sending Data Link Escape (DLE)
	c = 0x02;
	writeBYTE(&c); //Sending Start Of Text (STX)
#endif
	c = (frame[0] & 0xFF00) >> 8; //LSB
	if (writeBYTE(&c))
		perror("writeByte");

	c = 0x77; // 0x77 is not used by EPOS
	// wait for "Ready Ack 'O'"
	if ((n = readBYTE(&c)) < 0)
		fprintf(stderr, "readBYTE() returnd %d at %s, line %d\n", n, __func__,
				__LINE__);

	if (c != E_OK) {
		if (c == 0x77) {
			fprintf(stderr,
					"ERROR: no reply from EPOS recieved, is it online?\n");
			//exit(2);
		}
		printf("EPOS not ready, reply was: %#04x\n", c);
		return (-1);

	}
#ifdef USBMODE
	c = (frame[0] & 0x00FF) + 1; //MSB
#else
	c = (frame[0] & 0x00FF); //MSB
#endif
	if (writeBYTE(&c))
		perror("writeBYTE");

	// header done, data + CRC will follow
	for (i = 1; i < len; i++) {
		if (writeWORD(frame + i))
			perror("writeWORD");
	}

	// wait for "End Ack 'O'"
	if (readBYTE(&c) < 0)
		perror("readBYTE");
	if (c != E_OK) {
		printf("EPOS says: %#04x | CRCerror!\n", c);
		return (-1);
	}
	/*
	 gettimeofday(&end, NULL);
	 seconds  = end.tv_sec  - start.tv_sec;
	 useconds = end.tv_usec - start.tv_usec;
	 mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	 printf("\n\n\n*****Elapsed time for sendCom: %ld milliseconds*******\n\n\n", mtime);*/
	ioctl(ep, TCFLSH);
	return (0);
}

/*!  int readAnswer(WORD **ptr) - read an answer frame from EPOS

 \param ptr WORD **ptr; pointer address where answer frame is placed.

 \retval >0 number of WORDs recieved from EPOS. ptr points now to
 answer frame.
 \retval <0 failure; ptr points to NULL. Global E_error_new is also set to
 returnd EPOS ErrorCode

 */
int CEposVMC::readAnswer(WORD **ptr) {

	int i;
	BYTE c;
	WORD first = 0x00, w, crc, framelen;
	static WORD *ans;

	E_error_new = 0x00;

	/*
	 printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
	 printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
	 */

	readBYTE(&c);
	first = (0xFF00 & c) << 8;
	//printf("first answer: %#04x; first: %#06x\n", c, first);


	if (c != E_ANS) {
		fprintf(stderr, "EPOS says: %#04x. This is no answer frame! \n", c);
		ptr = NULL;
		return (-1);
	}
	c = E_OK;
	writeBYTE(&c);

	// here is the (len-1) value coming
	readBYTE(&c);

	first = (0x00FF & c);
	//printf("second answer: %#04x; first: %#06x\n", c, first);

	framelen = c + 3;

	checkPtr(ans = (WORD*) malloc(framelen * sizeof(WORD)));

	ans[0] = first;

	for (i = 1; i < framelen; i++) {
		readWORD(&w);
		ans[i] = w;
	}
#ifdef DEBUG
	printf("\n<< ");
	for (i = 0; i < (framelen); i++) {
		printf("%#06x ", ans[i]);
	}
	printf("\n");
	fflush(stdout);

#endif

	// compute checksum
	crc = ans[framelen - 1];
#ifdef DDEBUG
	printf("got this CRC: %#06x\n", crc);
#endif
	ans[framelen - 1] = 0x0000;
	ans[framelen - 1] = CalcFieldCRC(ans, framelen);

	if (crc == ans[framelen - 1]) {
		c = E_OK;
		writeBYTE(&c);
#ifdef DEBUG
		printf("CRC test OK!\n");
#endif
	} else {
		c = E_FAIL;
		writeBYTE(&c);
		fprintf(stderr, "CRC test FAILED!\n");
		ptr = NULL;
		return (-1);
	}

	/* check for error code */

	/* just to get the bit's at the right place...*/
	//ans[1] = 0x1234; ans[2] = 0xABCD;
	E_error_new = ans[1] | (ans[2] << 16);
	//printf(" xxxxxxx ->%#010x<-\n", E_error_new);


	*ptr = ans;
	/*
	 printf("******** sub: ptr= %p  &ptr = %p\n", ptr, &ptr);
	 printf("******** sub: ans   = %p  &ans = %p\n", ans, &ans);
	 */
	ioctl(ep, TCFLSH);
	return (framelen);
}

//int CEposVMC::ReadObject(WORD index, BYTE subindex, WORD **ptr ){
//
// WORD frame[4];
//  int n = 0;
//
//  frame[0] = 0x1001; // fixed, ReadObject, (len-1) == 1
//  frame[1] = index;
//  frame[2] = (0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
//				      low BYTE: subindex */
//  frame[3] = 0x000; // ZERO word, will be filled with checksum
//
//  if( (n = sendCom(frame)) < 0){
//    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",
//	    __func__, n);
//    return(-1);
// }
//
//  // read response
//  return( readAnswer(ptr) );
//
//}

int CEposVMC::ReadObject(WORD index, BYTE NodeId, BYTE subindex, WORD **ptr) {

	struct timeval start, end;
	long mtime, seconds, useconds;
	gettimeofday(&start, NULL);

	WORD frame[4];
	int n = 0;
	int temp = 0;

	frame[0] = 0x1001; // fixed, ReadObject, (len-1) == 1
	frame[1] = index;
	frame[2] = NodeId;
	frame[2] = frame[2] << 8;
	frame[2] = frame[2] | subindex;
	//frame[2] = (0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
	// low BYTE: subindex */
	frame[3] = 0x000; // ZERO word, will be filled with checksum

	if ((n = sendCom(frame)) < 0) {
		fprintf(stderr,
				" *** %s: problems with sendCom(), return value was %d ***\n ",
				__func__, n);
		return (-1);
	}

	temp = readAnswer(ptr);

	gettimeofday(&end, NULL);
	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;
	mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
	//printf("\n\n\n\n\n**********************************************\n");
	//printf("Time for ReadObject call is %ld milliseconds\n", mtime);
	//printf("**********************************************\n\n\n\n\n");
	// read response
	return (temp);
	//return( readAnswer(ptr) );

}

/*! Low-level function to write an object to EPOS memory. Is called by
 writing libEPOS functions.

 \param index WORD describing EPOS memory index for writing. See
 firmware documentation for valid values

 \param subindex BYTE describing EPOS memory subindex for writing. See
 firmware documentation for valid values

 \param data pointer to WORD array holding the data to be written to EPOS memory

 \retval 0 success
 \retval -1 error
 */
//int CEposVMC::WriteObject(WORD index, BYTE subindex, WORD *data) {
//
//  WORD frame[6];
//  WORD *ans = NULL;
//  int n = 0;
//
//
//  frame[0] = 0x1103; // fixed, WriteObject, (len-1) == 3
//  frame[1] = index;
//  frame[2] = (0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
//				      low BYTE: subindex */
//  // data to transmit
//  frame[3] = data[0];
//  frame[4] = data[1];
//
//  frame[5] = 0x000; // ZERO word, will be filled with checksum
//
//  if( (n = sendCom(frame)) < 0){
//    fprintf(stderr, " *** %s: problems with sendCom(), return value was %d ***\n ",  __func__, n);
//    return(-1);
//  }
//
//
//  // read response
//  checkPtr( ans = (WORD*)calloc(3, sizeof(WORD) ) );
//
//  if ( (n = readAnswer(&ans) )  <0 ){
//    fprintf(stderr, " *** %s: problems with readAnswer(), return value was %d ***\n ",  __func__, n);
//    free(ans);
//    return(-1);
//  }
//
//  return( checkEPOSerror() );
//
//}

int CEposVMC::WriteObject(WORD index, BYTE NodeId, BYTE subindex, WORD *data) {

	/*
	 struct timeval start, end;
	 long mtime, seconds, useconds;
	 gettimeofday(&start, NULL);
	 */

	WORD frame[6];
	WORD *ans = NULL;
	int n = 0;

	frame[0] = 0x1103; // fixed, WriteObject, (len-1) == 3
	frame[1] = index;
	frame[2] = NodeId;
	frame[2] = frame[2] << 8;
	frame[2] = frame[2] | subindex;
	//frame[2] = ((0x0000 | (NodeId<<2)) | subindex);  //(0x0000 | subindex); /* high BYTE: 0x00(Node-ID == 0)
	//low BYTE: subindex */
	//cout << frame[2]<<endl;
	//printf("%x \n",frame[2]);
	// data to transmit
	frame[3] = data[0];
	frame[4] = data[1];

	frame[5] = 0x000; // ZERO word, will be filled with checksum

	if ((n = sendCom(frame)) < 0) {
		fprintf(stderr,
				" *** %s: problems with sendCom(), return value was %d ***\n ",
				__func__, n);
		return (-1);
	}

	// read response
	checkPtr(ans = (WORD*) calloc(3, sizeof(WORD)));

	if ((n = readAnswer(&ans)) < 0) {
		fprintf(
				stderr,
				" *** %s: problems with readAnswer(), return value was %d ***\n ",
				__func__, n);
		free(ans);
		return (-1);
	}

	/*
	 gettimeofday(&end, NULL);
	 seconds  = end.tv_sec  - start.tv_sec;
	 useconds = end.tv_usec - start.tv_usec;
	 mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	 printf("<<<<<Elapsed time of WriteObject: %ld milliseconds>>>>>\n", mtime);

	 */
	return (checkEPOSerror());

}

/* compare WORD a with WORD b bitwise */
int CEposVMC::bitcmp(WORD a, WORD b) {
	if ((a & b) == b)
		return (1);
	else
		return (0);
}

void CEposVMC::checkPtr(void* ptr) {
	if (ptr == NULL) {
		fprintf(stderr, "malloc failed!\n");
		exit(-1);
	}
}

/*! NOT USED IN libEPOS so far -> untested!
 */
int CEposVMC::InitiateSegmentedRead(WORD index, BYTE subindex) {

	WORD frame[4], **ptr = NULL;
	int n = 0;

	frame[0] = 0x1201; // fixed, opCode==0x12, (len-1) == 1
	frame[1] = index;
	frame[2] = 0x0000 | subindex; /* high BYTE: 0x00 (Node-ID == 0)
	 low BYTE: subindex */
	frame[3] = 0x000; // ZERO word, will be filled with checksum

	if ((n = sendCom(frame)) < 0) {
		fprintf(stderr,
				" *** %s: problems with sendCom(), return value was %d ***\n ",
				__func__, n);
		return (-1);
	}

	// read response
	return (readAnswer(ptr)); // answer contains only DWORD ErrorCode
	// here...
}

/*! NOT USED IN libEPOS so far -> untested!

 \retval >0  means sucess. Number of WORDs recieved from EPOS will be returned
 (*ptr) points to answer frame

 \retval <0 means failure: (*ptr) points to NULL

 */
int CEposVMC::SegmentRead(WORD **ptr) {

	WORD frame[3];
	int n = 0;

	frame[0] = 0x1400; // fixed, opCode==0x14, (len-1) == 0
	frame[1] = 0x0000; // WHAT IS THE 'TOGGLE' BIT????
	frame[2] = 0x0000; // ZERO word, will be filled with checksum

	if ((n = sendCom(frame)) < 0) {
		fprintf(stderr,
				" *** %s: problems with sendCom(), return value was %d ***\n ",
				__func__, n);
		return (-1);
	}

	if ((n = readAnswer(ptr)) < 0) {
		fprintf(stderr,
				" *** %s: problems with readAns(), return value was %d ***\n ",
				__func__, n);
		return (-1);
	}

	return (0);
}

