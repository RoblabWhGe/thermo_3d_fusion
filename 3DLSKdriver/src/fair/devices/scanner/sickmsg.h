/*
*
* libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
* 
* Ownership Fraunhofer Gesellschaft e. V., Munich, Germany
*
* The FAIR library [both binary and source code (if released)] is intellectual
* property owned by Fraunhofer Gesellschaft and is protected by the Creative
* Commons license by-nc-sa (http://creativecommons.org/licenses/by-nc-sa/3.0/de/).
* The ownership remains with Fraunhofer Gesellschaft.
*/

#ifndef SICKMSG_H_
#define SICKMSG_H_

#define MAX_TELEGRAM_LENGTH 2048

/**
 * Start frame of each telegram
 */
#define STX		0x02
/**
 * Broadcast address
 */
#define BRC		0x00
/**
 * Address offset for answers
 */
#define ADROFF		0x80
/**
 * Acknowledge byte (Answer before start frame)
 */
#define ACK		0x06
/**
 * Negative acknowledge (Answer before start frame)
 */
#define NACK		0x15

/**
 * Start sequence of setup commands
 */
#define CMDSETUP	0x20
/**
 * Start sequence of operation commands
 */
#define CMDOP		0x30
/**
 * Start sequence of configuration commands
 */
#define CMDCONF	0x77
/**
 * Start sequence of variant setup commands
 */
#define CMDVAR		0x3b

/**
 * Baudrate message for 9600 baud
 */
#define BAUD9600	0x42
/**
 * Baudrate message for 500k baud
 */
#define BAUD500K	0x48

/**
 * Message for setting up scanner ranges
 */
#define RANGE8M		0x00
#define RANGE16M	0x03
#define RANGE32M	0x06

/**
 * Apex angles
 */
#define APEX100	0x64
#define APEX180	0xb4

/**
 * Scanner resolution
 */
#define RES1		0x64
#define RES05		0x32
#define RES025		0x19

/**
 * Units for distances
 */
#define UNITCM		0x00
#define UNITMM		0x01

/**
 * Tansmission modes (continous sending, request sending, Interlaced mode)
 */
#define TRANSCONT	0x24
#define TRANSREQ	0x25
#define TRANSIL		0x2a
#define TRANSILR	0x2b

/**
 * Transmission mode
 */
#define MSMODEDEF	0x00
#define MSMODEIL	0x0f
#define MSMODEILR	0x0e

/***** telegram 0x20 *****/
 // Requests
const unsigned char g_aucMsgSetupBaudrate[]   = { CMDSETUP, BAUD9600 };
const unsigned char g_aucMsgSetupConfigMode[] = { CMDSETUP, 0x00, 0x53, 0x49, 0x43, 0x4b, 0x5f, 0x4c, 0x4d, 0x53 };
const unsigned char g_aucMsgSetupTrans[]      = { CMDSETUP, TRANSREQ };
const unsigned char g_aucMsgSetupTransRem[]   = { CMDSETUP, TRANSILR, 0x01, 0x00, 0x01, 0x00, 0xb5, 0x00 };
 // Answer
const unsigned char g_aucAnswSetup[]          = { CMDSETUP+ADROFF, 0x00, 0x10 };
/*************************/

/***** telegramm 0x30 *****/
const unsigned char g_aucMsgOpDataRequest[]   = { CMDOP, 0x01 };
/**************************/

/***** telegram 0x3b *****/
 // Request
const unsigned char g_aucMsgVariant[]         = { CMDVAR,APEX180,0x00,RES05,0x00 };
 // Answer
const unsigned char g_aucAnswVariant[]        = { CMDVAR+ADROFF,0x01,APEX180,0x00,RES05,0x00,0x10 };
/*************************/

/***** telegram 0x77 *****/
 // Request
/*unsigned char g_aucMsgRange[] = { CMDCONF, 0x00, 0x00, 0x46, 0x00, 0x00, MSMODEDEF, UNITMM, 0x00, 0x00, 0x01, 0x02,
									0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x0a, 0x0a, 0x50, 0x64,
									0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 };*/
const unsigned char g_aucMsgTrans[]  = { CMDCONF, 0x00, 0x00, 0x46, 0x00, 0x00, MSMODEDEF, UNITMM, 0x00, 0x00, 0x01,
                                         0x02, 0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x0a, 0x0a, 0x50,
                                         0x64, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 };
 // Answers
/*unsigned char g_aucAnswRange[]= { CMDCONF+ADROFF, 0x01, 0x00, 0x00, 0x46, 0x00, 0x00, MSMODEDEF, UNITMM, 0x00, 0x00, 0x01, 0x02,
									0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x0a, 0x0a, 0x50, 0x64,
									0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10 };*/
									// Scanner sends 0x10 as last byte -> not in documentation!
const unsigned char g_aucAnswTrans[] = { CMDCONF+ADROFF, 0x01, 0x00, 0x00, 0x46, 0x00, 0x00, TRANSREQ,UNITMM, 0x00, 0x00,
                                         0x01, 0x02, 0x02, 0x00, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x0a, 0x0a, 0x50,
                                         0x64, 0x00, 0x0a, 0x0a, 0x50, 0x64, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10 };
/*************************/

#endif /*SICKMSG_H_*/
