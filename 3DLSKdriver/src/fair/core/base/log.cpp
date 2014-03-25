#include "log.h"

namespace fair
{
	
static int g_nVerbosity = 1;

static void _error (int nExitStatus, const char* szMode, const char* szMessage)
{
	if(g_nVerbosity>0) fprintf (stderr, "%s: %s.\n", szMode, szMessage);
	if(nExitStatus >= 0)
		exit (nExitStatus);
}

void verbosity (int nVerbosity)
{
	g_nVerbosity = nVerbosity;	
}

void info (const char* szMessage)
{
	_error (-1, "info", szMessage);
}

void warning (const char* szMessage)
{
	_error (-1, "warning", szMessage);	
}

void error (const char* szMessage)
{
	_error (-1, "ERROR", szMessage);	
}

void fatal (const char* szMessage)
{
	_error (FAIR_EXIT_FAILURE, "FATAL", szMessage);	
}

void percentage(unsigned int unPercentage)
{
	unsigned int i = 0;
	for(i=0; i<unPercentage/2+5; i++)
		printf("\b");
	for(i=0; i<unPercentage/2; i++)
		printf("=");
	printf("> ");
	printf("%d%%",unPercentage);
	fflush(stdout);
	if(unPercentage==100) printf("\n");
}

}
