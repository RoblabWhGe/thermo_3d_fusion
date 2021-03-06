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


/**
 * Basic error message output functions.
 * Don't use printf or cout anymore by yourself.
 * The purpose of these procedures is to centralize program output.
 * @author Stefan May
 */

#ifndef ERROR_H
#define ERROR_H 1

#include "../../../fair/core/base/common.h"

namespace fair
{

void verbosity	(int nVerbosity);
void info		(const char* szMessage);
void warning	(const char* szMessage);
void error		(const char* szMessage);
void fatal		(const char* szMessage);
void percentage(unsigned int unPercentage);

}


#endif /* !ERROR_H */
