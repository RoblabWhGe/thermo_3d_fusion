/*
 * File:   CEposVMC.h
 * Author: viatcheslav
 *
 * Created on November 10, 2009, 3:44 PM
 */

#ifndef _CEPOSVMC_H
#define	_CEPOSVMC_H

#include <cstring>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>

/* added oct06 for openTCPEPOS() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


/*!serial port file descriptor*/
extern int sp;


namespace epos {
/* all EPOS data exchange is based on 16bit words, but other types are
   also used...*/
typedef unsigned long DWORD ; ///< \brief 32bit type for EPOS data exchange
typedef unsigned short WORD ; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE; ///< \brief 8bit type for EPOS data exchange
#endif

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      100
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
#define TRYSLEEP  (unsigned int) 3e3 //1e5

/* all high-level functions return <0 in case of error */

/*! EPOS global error status */
extern DWORD E_error;



#define ZERO_OFFSET 452700    //< steps between reference point and 1st filter
#define FILTER_FILTER 666667  //< steps between 2 filters


class CEposVMC {
public:
   
    CEposVMC(int numOfArgs, char ** argument);
    CEposVMC(const CEposVMC& orig);
    virtual ~CEposVMC();
    char c;
   
    int  n;
    int faultsCounter;
    long int pos;
    int getch(void);
    int kbhit(void);
    int PrintHelp();
    /*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
     position */
    int doHoming(int method, long int start);


    /*! \brief set OpMode to ProfilePosition and make relative movement */
    int moveRelative(long int steps);
    /*! \brief set OpMode to ProfilePosition and make absolute movement */
    int moveAbsolute(long int steps, long int speed, int acceleration, int deceleration, int maxVelocity);

    int profileVelocityMode(long int targetVelocity, int maxVelocity, int acceleration, int deceleration);

    int setVelocity(long int targetVelocity); // settings are kept untill shutdown
    int setVelocity(long int targetVelocity, BYTE nodeAddress); // settings are kept untill shutdown
    int setRPM(long int targetVelocity1, long int targetVelocity2, BYTE nodeAddress1, BYTE nodeAddress2);

    int setAcceleration(int acceleration); //settings are kept untill shutdown
    int setAcceleration(int acceleration, BYTE nodeAddress); //settings are kept untill shutdown

    int setDeceleration(int deceleration); //settings are kept untill shutdown
    int setDeceleration(int deceleration, BYTE nodeAddress); //settings are kept untill shutdown

    /*! \brief reads position, velocity and current and displays them in an
   endless loop. Returns after target position has been reached */
    int monitorStatus();

    /*! \brief set home switch polarity -- firmware spec 14.1.47 */
    int setHomePolarity(int pol);
     int setHomePolarity(int pol, BYTE nodeAddress);

    /*! \brief read actual position; 14.1.61 */
    int getDemandPosition(long *dpos);
    
    /*! \brief read actual position; 14.1.62 */
    int getActualPosition(long *apos);

    /*! \brief read actual position; 14.1.67 */
    int getDemandVelocity(long *dvel);

    /*! \brief read actual position; 14.1.68 */
    int getActualVelocity(long *avel);

    /*! \brief read actual current; 14.1.69 */
    int getActualCurrent(short int *current);

    /*! \brief read target position; 14.1.70 */
    int getTargetPosition(long *tpos);

    
    void CloseDevice(void);

private:
    char ip[16];
    unsigned short port;
    size_t string_pos;
    int InitDevice(int argc, char **arg);
    int SecondInitDevice(int argc, char **arg);
    int numOfArgs;
    char ** argument;
    //void CloseDevice(void);
  
    void Help(char *arg);


    /*! open the connection to EPOS */
    int openEPOS(char *device);
    /*! open the connection to EPOS via RS232-over-TCP/IP (LSW special) */
    int openTCPEPOS(char *ip, short unsigned port);
    /*! close the connection to EPOS */
    int closeEPOS();
    /*! check if the connection to EPOS is alive */
    int checkEPOS();


    /*! \brief check global variable E_error for EPOS error code */
    int checkEPOSerror();

    /*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
    int checkEPOSstate();
    int checkEPOSstate(BYTE nodeAddress);
    /*! \brief pretty-print EPOS state */
    int printEPOSstate();
    int printEPOSstate(BYTE nodeAddress);
    /*! \brief change EPOS state   ==> firmware spec 8.1.3 */
    int changeEPOSstate(int state);
    int changeEPOSstate(int state, BYTE nodeAddress);



    /*! \brief example from EPOS com. guide: ask EPOS for software version

    firmware spec 14.1.33
    ** returns software version as HEX **

    */
    int readSWversion();
    int readSWversion(BYTE nodeAddress);


    /*! \brief ask for device name,
    device name is placed in 'name' (string must be big enough, NO CHECKING!!)
    */
    int readDeviceName(char *name);


    /*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
    int readRS232timeout();

    /*! \brief read digital input polarity mask */
    int readDInputPolarity(WORD* w);
    int readDInputPolarity(WORD* w, BYTE nodeAdress);





    /*! \brief read Statusword; 14.1.58 */
    int readStatusword(WORD *eposStatus);
    int readStatusword(WORD *eposStatus, BYTE nodeAddress);
    /*! \brief pretty-print Statusword */
    int printEPOSstatusword(WORD statusword);



    /*! \brief read EPOS control word (firmware spec 14.1.57) */
    int readControlword(WORD *w);
     int readControlword(WORD *w, BYTE nodeAddress);
    /*! \brief pretty-print Controlword */
    int printEPOScontrolword(WORD controlword);


    /*! \brief set EPOS mode of operation -- 14.1.59 */
    int setOpMode(int OpMode);
    int setOpMode(int OpMode, BYTE nodeAddress);
    /*! \brief read and returns  EPOS mode of operation -- 14.1.60
        here, RETURN(0) MEANS ERROR!
        '-1' is a valid OpMode, but 0 is not!
    */
    int readOpMode();
    int readOpMode(BYTE nodeAddress);

    /*! \brief read position window; 14.1.64 */
    int readPositionWindow(unsigned long int *posw);
    /*! \brief write position window; 14.1.64 */
    int writePositionWindow(unsigned long int posWindow);

    /*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
    int monitorHomingStatus();

    /*! \brief waits for positoning to finish, argument is timeout in
       seconds. give timeout==0 to disable timeout */
    int waitForTarget(unsigned int t);

    /*! \brief  write a single BYTE to EPOS */
    int writeBYTE(BYTE *c);

    /*! \brief  write a single WORD to EPOS */
    int writeWORD(WORD *w);

    /*! \brief  read a single BYTE from EPOS, timeout implemented */
    int readBYTE(BYTE *c);

    /*! \brief  read a single WORD from EPOS, timeout implemented */
    int readWORD(WORD *w);

    /*! \brief Checksum calculation;
    copied from EPOS Communication Guide, p.8
     */
    WORD CalcFieldCRC(WORD *pDataArray, WORD numberOfWords);

    /*! \brief  send command to EPOS, taking care of all neccessary 'ack' and
       checksum tests*/
    int sendCom(WORD *frame);

    /*! \brief  int readAnswer(WORD **ptr) - read an answer frame from EPOS */
    int readAnswer(WORD **ptr);

    /* Implement read functions defined in EPOS Communication Guide, 6.3.1 */
    /* [ one simplification: Node-ID is always 0] */

    /*! \brief Read Object from EPOS memory, firmware definition 6.3.1.1*/
    //int ReadObject(WORD index, BYTE subindex, WORD **answer );

    int ReadObject(WORD index, BYTE NodeId, BYTE subindex, WORD **ptr );

    /*! \brief Read Object from EPOS memory, firmware definition 6.3.1.2 */
    int InitiateSegmentedRead(WORD index, BYTE subindex );

    /*! \brief int SegmentRead(WORD **ptr) - read data segment of the object
       initiated with 'InitiateSegmentedRead()'
    */
    int SegmentRead(WORD **ptr);


    /* 6.3.2:  write functions */

    /*! 6.3.2.1 WriteObject()

       WORD *data is a pointer to a 2 WORDs array (== 4 BYTES)
       holding data to transmit
    */
    //int WriteObject(WORD index, BYTE subindex, WORD data[2]);
    int WriteObject(WORD index,BYTE NodeId, BYTE subindex, WORD *data);
   

    /*! \brief exit(-1) if ptr == NULL */
    void checkPtr(void* ptr);


    /*! \brief compare two 16bit bitmasks, return 1 (true) or 0 (false) */
    int bitcmp(WORD a, WORD b);

};
}
#endif	/* _CEPOSVMC_H */
