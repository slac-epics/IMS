#define VERSION 1.0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alarm.h>
#include <dbDefs.h>
#include <callback.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <recGbl.h>
#include <recSup.h>
#include <dbEvent.h>
#include <devSup.h>
#include <math.h>
#include <time.h>
#include <errlog.h>

#define GEN_SIZE_OFFSET
#include "imsRecord.h"
#undef GEN_SIZE_OFFSET

#include "drvAsynIPPort.h"
#include "asynOctetSyncIO.h"

#include "epicsExport.h"

#include "misc.h"

/*** All db_post_events() calls set both VALUE and LOG bits ***/
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)


/*** Forward references ***/


/*** Record Support Entry Table (RSET) functions ***/

static long init_record       ( dbCommon *precord, int pass                );
static long process           ( dbCommon *precord                          );
static long special           ( dbAddr   *pDbAddr, int after               );
static long get_units         ( dbAddr   *pDbAddr, char *                  );
static long get_precision     ( dbAddr   *pDbAddr, long *                  );
static long get_graphic_double( dbAddr   *pDbAddr, struct dbr_grDouble   * );
static long get_control_double( dbAddr   *pDbAddr, struct dbr_ctrlDouble * );
static long get_alarm_double  ( dbAddr   *pDbAddr, struct dbr_alDouble   * );


rset imsRSET =
{
    RSETNUMBER,
    NULL,
    NULL,
    (RECSUPFUN) init_record,
    (RECSUPFUN) process,
    (RECSUPFUN) special,
    NULL,
    NULL,
    NULL,
    NULL,
    (RECSUPFUN) get_units,
    (RECSUPFUN) get_precision,
    NULL,
    NULL,
    NULL,
    (RECSUPFUN) get_graphic_double,
    (RECSUPFUN) get_control_double,
    (RECSUPFUN) get_alarm_double
};

extern "C" { epicsExportAddress( rset, imsRSET ); }


struct ims_info
{
    epicsMutex *cMutex;
    asynUser   *pasynUser;
    int         S1;                                                 // switch S1
    int         S2;
    int         S3;
    int         msta;
    long        count;
    bool        newData;
};

#define MAX_MSG_SIZE 64
#define FLUSH        -1

static int  connect_motor( void *precord                             );
static int  init_motor   ( void *precord                             );
static void post_fields  ( void *precord, unsigned short alarm_mask,
                                          unsigned short all         );

static int send_msg  ( asynUser *pasynUser, char const *msg     );
static int recv_reply( asynUser *pasynUser, char *buf, int flag );


/*** Debugging ***/

volatile int imsRecordDebug = 0;

extern "C" { epicsExportAddress( int, imsRecordDebug ); }


/******************************************************************************/
static long init_record( dbCommon *precord, int pass )
{
    imsRecord *prec = (imsRecord *)precord;
    ims_info  *mInfo;

    int        status = 0;

    if ( pass > 0 ) return( status );

    mInfo = (ims_info *) malloc( sizeof(ims_info) );
    mInfo->cMutex = new epicsMutex();

    prec->dpvt    = mInfo;

    connect_motor( prec );
    init_motor   ( prec );

    return( status );
}


/******************************************************************************/
static int connect_motor( void *precord )
{
    imsRecord         *prec  = (imsRecord *)precord;
    ims_info          *mInfo = (ims_info  *)prec->dpvt;

    asynUser          *pasynUser;
    char               serialPort[80];
    static const char  output_terminator[] = "\n";
    static const char  input_terminator[]  = "\r\n";

    asynStatus         asyn_rtn;
    int                status = 0;

    sprintf( serialPort, "%s TCP", prec->port );

    mInfo->cMutex->lock();

    drvAsynIPPortConfigure              ( prec->asyn, serialPort, 0, 0, 0 );
    asyn_rtn = pasynOctetSyncIO->connect( prec->asyn, 0, &pasynUser, NULL );

    if ( asyn_rtn != asynSuccess )
    {
    }

    pasynOctetSyncIO->setOutputEos( pasynUser, output_terminator,
                                    strlen(output_terminator)     );
    pasynOctetSyncIO->setInputEos ( pasynUser, input_terminator,
                                    strlen(input_terminator)      );

    // flush any junk at input port - should not be any data yet
    pasynOctetSyncIO->flush( pasynUser );

    mInfo->pasynUser = pasynUser;

    mInfo->cMutex->unlock();

    return( status );
}


/******************************************************************************/
static int init_motor( void *precord )
{
    imsRecord *prec  = (imsRecord *)precord;
    ims_info  *mInfo = (ims_info  *)prec->dpvt;

    asynUser  *pasynUser = mInfo->pasynUser;
    char       buf[MAX_MSG_SIZE];
    int        status = 0, retry, rtnval, byv, failed;

    mInfo->cMutex->lock();

    // determine the part number
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR PN" );
        status = recv_reply( pasynUser, prec->pn, 1 );
        epicsThreadSleep( 0.5 );
    } while ( status <= 0 && retry++ < 3 );

    if ( status <= 0 )
    {
        Debug( 0, "Failed to read the part number" );

        failed = 1;
        goto finish_up;
    }

    // determine the switch settings
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"S1=\",S1,\", S2=\",S2,\", S3=\",S3" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            // status = sscanf( buf, "S1=%d, %d, %d, S2=%d, %d, %d, S3=%d, %d, %d", &rtnval, &byv );
            if ( status == 9 )
            {
                // motor_info->mcode_version = (rtnval << 1) + (byv & 1);
                break;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 9 )
    {
        Debug( 0, "Failed to read the switch settings" );
        // motor_info->mcode_version = 0;

        failed = 1;
    }

    // determine if encoder enabled
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"EE=\",EE" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "EE=%d", &rtnval );
            if ( (status == 1) && (rtnval == 0 || rtnval ==1) )
            {
                prec->ee = rtnval;                        // 1 : encoder present
                break;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 1 )
    {
        Debug( 0, "Failed to read the encoder status" );
        prec->ee = 0;

        failed = 1;
    }

    // determine the stall mode
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"SM=\",SM" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "SM=%d", &rtnval );
            if ( (status == 1) && (rtnval == 0 || rtnval ==1) )
            {
                prec->sm = rtnval;
                break;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 1 )
    {
        Debug( 0, "Failed to read the stall mode" );
        prec->sm = 0;

        failed = 1;
    }

    // check the MCode program version and running status
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"VE=\",VE,\",BY=\",BY" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "VE=%d,BY=%d", &rtnval, &byv );
            if ( status == 2 ) break;
            else               status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 2 )
    {
        Debug( 0, "Failed to read the MCode version" );

        failed = 1;
    }

    if ( rtnval != prec->dver )
    {
        char line[256];

        send_msg( pasynUser, "E" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "CP" );
        epicsThreadSleep( 1 );

        strcpy( line, getenv("IMS") );
        strcat( line, "/misc/"      );
        strcat( line, prec->mpgm    );

        FILE *fp = fopen( line, "r" );
        while( 1 )
        {
            fgets( line, 64, fp );
            if ( ferror(fp) || feof(fp) ) break;

            send_msg( pasynUser, line );
            Debug(3, "%s", line);

            epicsThreadSleep( 0.1 );
        }

        fclose(fp);
        epicsThreadSleep( 3 );

        send_msg( pasynUser, "EX=1" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "PU=0" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "S" );
        epicsThreadSleep( 1 );
    }
    else if ( byv == 0 )
    {
        send_msg( pasynUser, "EX=1" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "PU=0" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "S" );
        epicsThreadSleep( 1 );
    }

    // send a status update in 1 second
    send_msg( pasynUser, "R2=19500" );

    finish_up:
    mInfo->cMutex->unlock();

    return( 0 );
}


/******************************************************************************/
static long process( dbCommon *precord )
{
    imsRecord *prec = (imsRecord *)precord;
    ims_info  *mInfo = (ims_info *)prec->dpvt;

    Debug( 1, "%s -- process", prec->name );

    return( 0 );
}


/******************************************************************************/
static long special( dbAddr *pDbAddr, int after )
{
    imsRecord *prec = (imsRecord *) pDbAddr->precord;
    ims_info  *mInfo = (ims_info *)prec->dpvt;
    double     nval, hllm, hhlm, dval;
    long       msta, count;
    int        fieldIndex = dbGetFieldIndex( pDbAddr ), status = 0;

    if ( after == 0 )
    {
        if      ( fieldIndex == imsRecordDIR  ) prec->oval = prec->dir ;
        else if ( fieldIndex == imsRecordOFF  ) prec->oval = prec->off ;

        return( 0 );
    }

    switch( fieldIndex )
    {
        case imsRecordSSTR:
            Debug( 1, "%s -- %s", prec->name, prec->sstr );

            status = sscanf( prec->sstr, "Status=%ld,P=%ldEOS", &msta, &count );
            if ( status == 2 )
            {
                Debug( 2, "%s -- MSTA = %d, P = %d", prec->name, msta, count );

                mInfo->cMutex->lock();

                mInfo->msta    = msta;
                mInfo->count   = count;
                mInfo->newData = 1;
            }
            else
            {
                mInfo->cMutex->lock();

                mInfo->newData = 0;
            }

            mInfo->cMutex->unlock();

            break;
        case imsRecordERES:
            if ( prec->eres <= 0. )
                prec->eres = prec->mres;

            goto set_res;
        case imsRecordSREV:
        case imsRecordUREV:
            nval = prec->urev / prec->srev;
            if ( prec->mres != nval )
            {
                prec->mres = nval;
                db_post_events( prec, &prec->mres, DBE_VAL_LOG );
            }

            set_res:
            prec->oval = prec->res;
            if ( prec->ee ) prec->res = prec->eres;
            else            prec->res = prec->mres;

            if ( prec->res != prec->oval )
                db_post_events( prec, &prec->res,  DBE_VAL_LOG );

            break;
        case ( imsRecordDIR  ):
            if ( prec->dir == prec->oval ) break;

            if ( prec->dir == motorDIR_Positive )
            {
                hllm      = prec->off - prec->hlm;
                hhlm      = prec->off - prec->llm;
                prec->llm = prec->off + hllm;
                prec->hlm = prec->off + hhlm;
            }
            else
            {
                hllm      = prec->llm - prec->off;
                hhlm      = prec->hlm - prec->off;
                prec->llm = prec->off - hhlm;
                prec->hlm = prec->off - hllm;
            }

            dval       = (prec->val - prec->off ) * (1. - 2.*prec->oval);

            goto change_dir_off;
        case ( imsRecordOFF  ):
            prec->llm += prec->off - prec->oval;
            prec->hlm += prec->off - prec->oval;

            dval       = (prec->val - prec->oval) * (1. - 2.*prec->dir);

            change_dir_off:
            prec->val  =       dval * (1. - 2.*prec->dir) + prec->off;
            prec->rbv  = prec->drbv * (1. - 2.*prec->dir) + prec->off;

            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
//          MARK( M_VAL  );
//          MARK( M_RBV  );

//          check_software_limits( prec );

            break;
    }

    return( 0 );
}


/******************************************************************************/
static long get_units( dbAddr *paddr, char *units)
{
    return (0);
}

/******************************************************************************/
static long get_graphic_double( dbAddr *paddr, struct dbr_grDouble * pgd)
{
    return (0);
}

/******************************************************************************/
static long get_control_double( dbAddr *paddr, struct dbr_ctrlDouble * pcd)
{
    return (0);
}

/******************************************************************************/
static long get_precision( dbAddr *paddr, long *precision)
{
    return (0);
}

/******************************************************************************/
static long get_alarm_double( dbAddr  *paddr, struct dbr_alDouble * pad)
{
    return (0);
}

/******************************************************************************/
static int send_msg( asynUser *pasynUser, char const *msg )
{
    char         local_buf[MAX_MSG_SIZE];
    const double timeout = 1.0;
    size_t       nwrite;

    sprintf( local_buf, "1%s", msg );
    pasynOctetSyncIO->write( pasynUser, local_buf, strlen(local_buf),
                             timeout, &nwrite );

    return( 0 );
}

/******************************************************************************/
static int recv_reply( asynUser *pasynUser, char *buf, int flag )
{
    const double timeout  = 1.0;
    size_t       nread    = 0;
    asynStatus   asyn_rtn = asynError;
    int          eomReason;

    if ( flag == FLUSH )
        pasynOctetSyncIO->flush( pasynUser );
    else
        asyn_rtn = pasynOctetSyncIO->read( pasynUser, buf, MAX_MSG_SIZE,
                                           timeout, &nread, &eomReason );

    if ( (asyn_rtn != asynSuccess) || (nread <= 0) )
    {
        buf[0] = '\0';
        nread  = 0;
    }

    return( nread );
}

