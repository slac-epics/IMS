#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <math.h>
#include <time.h>

#include "imsRecord.h"

extern int imsRecordDebug;

/******************************************************************************/
void Debug( int level, const char *fmt, ... )
{
    timespec  ts;
    struct tm timeinfo;
    char      timestamp[40], msec[4], msg[512];

    va_list   args;

    if ( level > imsRecordDebug ) return;

    clock_gettime( CLOCK_REALTIME, &ts );
    localtime_r( &ts.tv_sec, &timeinfo );

    strftime( timestamp, 40, "%m/%d %H:%M:%S", &timeinfo );
    sprintf ( msec, "%03d", int(ts.tv_nsec*1.e-6 + 0.5) );

    va_start( args, fmt      );
    vsprintf( msg, fmt, args );
    va_end  ( args           );

    printf  ( "%s.%s %s\n", timestamp, msec, msg );

    return;
}

