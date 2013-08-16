#define VERSION 1.0

#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <alarm.h>
#include <dbDefs.h>
#include <callback.h>
#include <dbStaticLib.h>
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


/*** Forward references ***/


/*** Record Support Entry Table (RSET) functions ***/

static long init_record       ( dbCommon *precord, int pass                );
static long process           ( dbCommon *precord                          );
static long special           ( dbAddr   *pDbAddr, int after               );
static long cvt_dbaddr        ( dbAddr   *pDbAddr                          );
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
    (RECSUPFUN) cvt_dbaddr,
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


#define MIP_DONE     0x0000    // No motion is in progress
#define MIP_HOMF     0x0008    // A home-forward command is in progress
#define MIP_HOMR     0x0010    // A home-reverse command is in progress
#define MIP_HOME     (MIP_HOMF | MIP_HOMR)
#define MIP_MOVE     0x0020    // A move not resulting from Jog* or Hom*
#define MIP_RETRY    0x0040    // A retry is in progress
#define MIP_NEW      0x0080    // Stop current move for a new move
#define MIP_BL       0x0100    // Done moving, now take out backlash
#define MIP_STOP     0x0200    // We're trying to stop.  If a home command
                               // is issued when the motor is moving, we
                               // stop the motor first
#define MIP_PAUSE    0x0400    // Move is paused

#define MAX_MSG_SIZE 61
#define FLUSH        -1

#define OK            0


static long connect_motor     ( imsRecord *precord                            );
static long init_motor        ( imsRecord *precord                            );
static long process_motor_info( imsRecord *precord, status_word sword,
                                                    long count                );
static void new_move          ( imsRecord *precord                            );
static void post_fields       ( imsRecord *precord, unsigned short alarm_mask,
                                                    unsigned short all        );

static long send_msg          ( asynUser *pasynUser, char const *msg          );
static long recv_reply        ( asynUser *pasynUser, char *rbbuf, int flag    );

static long log_msg  ( imsRecord *prec, int dlvl, const char *fmt, ... );
static void post_msgs( imsRecord *prec                                 );


/*** Debugging ***/

volatile int imsRecordDebug = 0;

extern "C" { epicsExportAddress( int, imsRecordDebug ); }

using namespace std;


/******************************************************************************/
static long init_record( dbCommon *precord, int pass )
{
    imsRecord *prec = (imsRecord *)precord;
    ims_info  *mInfo;

    long       status = 0;

    if ( pass > 0 ) return( status );

    mInfo = (ims_info *)malloc( sizeof(ims_info) );
    mInfo->cMutex    = new epicsMutex();

    mInfo->lMutex    = new epicsMutex();
    mInfo->nMessages = 8;
    mInfo->mLength   = 61;
    mInfo->cIndex    = 0;
    mInfo->sAddr     = (char *)calloc( 8*61, sizeof(char) );

    prec->loga       = mInfo->sAddr;
    prec->logb       = mInfo->sAddr + mInfo->mLength * 1;
    prec->logc       = mInfo->sAddr + mInfo->mLength * 2;
    prec->logd       = mInfo->sAddr + mInfo->mLength * 3;
    prec->loge       = mInfo->sAddr + mInfo->mLength * 4;
    prec->logf       = mInfo->sAddr + mInfo->mLength * 5;
    prec->logg       = mInfo->sAddr + mInfo->mLength * 6;
    prec->logh       = mInfo->sAddr + mInfo->mLength * 7;

    prec->dpvt = mInfo;

    connect_motor ( prec );
    init_motor    ( prec );

    return( status );
}


/******************************************************************************/
static long connect_motor( imsRecord *prec )
{
    ims_info          *mInfo = (ims_info *)prec->dpvt;

    asynUser          *pasynUser;
    char               serialPort[80];
    static const char  output_terminator[] = "\n";
    static const char  input_terminator[]  = "\r\n";

    asynStatus         asyn_rtn;
    motor_status       msta;

    int                status = 0;

    msta.All = 0;

    sprintf( serialPort, "%s TCP", prec->port );

    mInfo->cMutex->lock();

    drvAsynIPPortConfigure              ( prec->asyn, serialPort, 0, 0, 0 );
    asyn_rtn = pasynOctetSyncIO->connect( prec->asyn, 0, &pasynUser, NULL );

    if ( asyn_rtn != asynSuccess )
    {
        log_msg( prec, 0, "Failed to connect to the port" );

        msta.Bits.RA_PROBLEM = 1;
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
static long init_motor( imsRecord *prec )
{
    ims_info *mInfo = (ims_info *)prec->dpvt;

    asynUser       *pasynUser = mInfo->pasynUser;
    char            msg[MAX_MSG_SIZE], rbbuf[MAX_MSG_SIZE];
    int             s1, s2, s3, a1, a2, a3, d1, d2, d3;
    int             rbne, rblm, rbsm, rbve, rbby;
    int             retry, status = 0;
    epicsUInt32     old_msta = prec->msta;
    motor_status    msta;
    unsigned short  alarm_mask;

    mInfo->cMutex->lock();

    msta.All = 0;

    // read the part number
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"PN=\",PN" );
        status = recv_reply( pasynUser, rbbuf, 1 );
        if ( (status > 0) && (strlen(rbbuf) > 3) )
        {
            if      ( strncmp(rbbuf,                   "PN=", 3) == 0 )
                strncpy( prec->pn, rbbuf+3, 61              );
            else if ( strncmp(rbbuf+(strlen(rbbuf)-3), "PN=", 3) == 0 )
                strncpy( prec->pn, rbbuf,   strlen(rbbuf)-3 );
            else
                status = 0;
        }
        else
            status = 0;

        epicsThreadSleep( 0.2 );
    } while ( (status <= 0) && (retry++ < 3) );

    if ( status <= 0 )
    {
        log_msg( prec, 0, "Failed to read the part number" );

        msta.Bits.RA_PROBLEM = 1;
        goto finished;
    }

    // read the serial number
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"SN=\",SN" );
        status = recv_reply( pasynUser, rbbuf, 1 );
        if ( (status > 0) && (strlen(rbbuf) > 3) )
        {
            if      ( strncmp(rbbuf,                   "SN=", 3) == 0 )
                strncpy( prec->sn, rbbuf+3, 61              );
            else if ( strncmp(rbbuf+(strlen(rbbuf)-3), "SN=", 3) == 0 )
                strncpy( prec->sn, rbbuf,   strlen(rbbuf)-3 );
            else
                status = 0;
        }
        else
            status = 0;

        epicsThreadSleep( 0.2 );
    } while ( (status <= 0) && (retry++ < 3) );

    if ( status <= 0 )
    {
        log_msg( prec, 0, "Failed to read the serial number" );

        msta.Bits.RA_PROBLEM = 1;
        goto finished;
    }

    // read the switch settings
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"S1=\",S1,\", S2=\",S2,\", S3=\",S3" );
        status = recv_reply( pasynUser, rbbuf, 1 );
        if ( status > 0 )
        {
            status = sscanf( rbbuf, "S1=%d, %d, %d, S2=%d, %d, %d, S3=%d, %d, %d",
                                    &s1, &a1, &d1, &s2, &a2, &d2, &s3, &a3, &d3 );
            if ( status == 9 )
            {
                if ( (a1 < 0) || (a1 > 1) ||
                     (s1 < 0) || ((s1 > 3) && (s1 != 16) && (s1 != 17)) )
                    status = 0;
                else if ( (a1 == 1) && (s1 > 0) && (s1 < 4) ) mInfo->S1 = s1;
                else                                          mInfo->S1 = 0;

                if ( (a2 < 0) || (a2 > 1) ||
                     (s2 < 0) || ((s2 > 3) && (s2 != 16) && (s2 != 17)) )
                    status = 0;
                else if ( (a2 == 1) && (s2 > 0) && (s2 < 4) ) mInfo->S2 = s2;
                else                                          mInfo->S2 = 0;

                if ( (a3 < 0) || (a3 > 1) ||
                     (s3 < 0) || ((s3 > 3) && (s3 != 16) && (s3 != 17)) )
                    status = 0;
                else if ( (a3 == 1) && (s3 > 0) && (s3 < 4) ) mInfo->S3 = s3;
                else                                          mInfo->S3 = 0;
            }
            else
                status = 0;

            if ( status == 9 )
            {
                sprintf( prec->s1, "%d, %d, %d", s1, a1, d1 );
                sprintf( prec->s2, "%d, %d, %d", s2, a2, d2 );
                sprintf( prec->s3, "%d, %d, %d", s3, a3, d3 );
            }
        }

        epicsThreadSleep( 0.2 );
    } while ( (status != 9) && (retry++ < 3) );

    if ( status != 9 )
    {
        log_msg( prec, 0, "Failed to read the switch settings" );

        msta.Bits.RA_PROBLEM = 1;
        goto finished;
    }

    // read the numeric enable, limit stop mode and stall mode
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"NE=\",NE,\", LM=\",LM,\", SM=\",SM" );
        status = recv_reply( pasynUser, rbbuf, 1 );
        if ( status > 0 )
        {
            status = sscanf( rbbuf, "NE=%d, LM=%d, SM=%d", &rbne, &rblm, &rbsm);
            if ( status == 3 )
            {
                if ( rbne == 0 || rbne == 1 ) prec->ne = rbne;    // 1 : enabled
                else                          status = 0;

                if ( rblm >  0 || rblm <  7 ) prec->lm = rblm;
                else                          status = 0;

                if ( rbsm == 0 || rbsm == 1 ) prec->sm = rbsm;    // 1 : no stop
                else                          status = 0;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.2 );
    } while ( (status != 3) && (retry++ < 3) );

    if ( status != 3 )
    {
        log_msg( prec, 0, "Failed to read NE, LM and SM" );

        prec->ne = 0;
        prec->lm = 0;
        prec->sm = 0;

        msta.Bits.RA_PROBLEM = 1;
        goto finished;
    }

    // read the MCode version and running status
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"VE=\",VE,\", BY=\",BY" );
        status = recv_reply( pasynUser, rbbuf, 1 );
        if ( status > 0 )
        {
            status = sscanf( rbbuf, "VE=%d, BY=%d", &rbve, &rbby );
            if ( (status != 2) || (rbby != 0 && rbby != 1) ) status = 0;
        }

        epicsThreadSleep( 0.2 );
    } while ( (status != 2) && (retry++ < 3) );

    if ( status != 2 )
    {
        log_msg( prec, 0, "Failed to read the MCode version and BY" );

        msta.Bits.RA_PROBLEM = 1;
        goto finished;
    }

    if ( rbve != prec->dver )
    {
        char line[256];

        send_msg( pasynUser, "E"  );
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
            Debug( 3, "%s", line );

            epicsThreadSleep( 0.1 );
        }

        fclose(fp);
        epicsThreadSleep( 3 );

        send_msg( pasynUser, "EX=1" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "PU=0" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "S"    );
        epicsThreadSleep( 1 );
    }
    else if ( rbby == 0 )
    {
        send_msg( pasynUser, "EX=1" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "PU=0" );
        epicsThreadSleep( 1 );

        send_msg( pasynUser, "S"    );
        epicsThreadSleep( 1 );
    }

    // set the parameters
    sprintf( msg, "DE=%d", prec->de );
    send_msg( mInfo->pasynUser, msg );

    epicsThreadSleep( 0.1 );

    sprintf( msg, "EE=%d", prec->ee );
    send_msg( mInfo->pasynUser, msg );

    epicsThreadSleep( 0.1 );

    if ( prec->el <= 0 ) prec->el = 1;
    if ( prec->ms <= 0 ) prec->ms = 1;
    if ( prec->ee == motorAble_Enable ) sprintf( msg, "EL=%d", prec->el );
    else                                sprintf( msg, "MS=%d", prec->ms );
    send_msg( mInfo->pasynUser, msg );

    epicsThreadSleep( 0.1 );

    sprintf( msg, "HC=%d", prec->hc );
    send_msg( mInfo->pasynUser, msg );

    epicsThreadSleep( 0.1 );

    sprintf( msg, "RC=%d", prec->rc );
    send_msg( mInfo->pasynUser, msg );

    mInfo->initialized = 0;
    msta.Bits.NOT_INIT = 1;

    if ( prec->ee == motorAble_Enable )
    {
        prec->elms = prec->el;
        prec->res  = prec->urev / prec->el /   4.;
    }
    else
    {
        prec->elms = prec->ms;
        prec->res  = prec->urev / prec->ms / 200.;
    }

    // make sure the limits are consistent
    if ( prec->dir == motorDIR_Positive )
    {
        prec->dllm = prec->llm - prec->off;
        prec->dhlm = prec->hlm - prec->off;
    }
    else
    {
        prec->dllm = prec->off - prec->hlm;
        prec->dhlm = prec->off - prec->llm;
    }

    // make sure the accelerations and velocities are consistent
    if ( prec->vbas > prec->vmax ) prec->vbas = prec->vmax;
    if ( prec->velo > prec->vmax ) prec->velo = prec->vmax;
    if ( prec->hvel > prec->vmax ) prec->hvel = prec->vmax;
    if ( prec->bvel > prec->vmax ) prec->bvel = prec->vmax;

    prec->dmov = 1;
    prec->mip  = MIP_DONE;

    // let controller send a status update every ~5 seconds until first process
    send_msg( pasynUser, "Us=300" );

    finished:
    mInfo->cMutex->unlock();

    if      ( msta.Bits.RA_PROBLEM                             )     // hardware
        recGblSetSevr( (dbCommon *)prec, COMM_ALARM,  INVALID_ALARM );
    else if ( msta.Bits.RA_BY0     || msta.Bits.RA_COMM_ERR ||  // BY=0 or wrong
              msta.Bits.NOT_INIT                               )     // NOT_INIT
    {
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );

        log_msg( prec, 0, "Wait for first status update" );
    }

    prec->msta = msta.All;

    if ( old_msta != prec->msta ) MARK( M_MSTA );

    recGblGetTimeStamp( prec );

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 1 );
    post_msgs  ( prec                );

    return( 0 );
}


/******************************************************************************/
static long process( dbCommon *precord )
{
    imsRecord      *prec = (imsRecord *)precord;
    ims_info       *mInfo = (ims_info *)prec->dpvt;
    char            msg[MAX_MSG_SIZE];
    unsigned short  old_mip,  alarm_mask;
    short           old_dmov, old_rcnt, old_miss;
    double          old_val,  old_dval, old_diff, diff;
    long            count, old_rval, old_msta = prec->msta, status = OK;
    int             VI, VM, A, retry, rbby;
    status_word     sword;
    motor_status    msta;
    bool            first;

    if ( prec->pact ) return( OK );

    prec->pact = 1;
    log_msg( prec, 1, "process" );

    mInfo->cMutex->lock();
    if ( ! mInfo->newData )                                // no new status data
    {
        mInfo->cMutex->unlock();

        msta.All = prec->msta;

        // check the MCode running status
        retry = 0;
        do
        {
            send_msg( mInfo->pasynUser, "PR \"BY=\",BY" );
            status = recv_reply( mInfo->pasynUser, msg, 1 );
            if ( status > 0 )
            {
                status = sscanf( msg, "BY=%d", &rbby );
                if ( status == 1 ) break;
            }

            epicsThreadSleep( 0.2 );
        } while ( retry++ < 3 );

        if ( status == 1 )                                       // read back BY
        {
            if      ( rbby == 0 )                           // MCode not running
            {
                log_msg( prec, 0, "MCode not running"   );
                msta.Bits.RA_BY0      = 1;
            }
            else if ( rbby != 1 )                              // wrong BY value
            {
                log_msg( prec, 0, "Invalid BY readback" );
                msta.Bits.RA_COMM_ERR = 1;
            }
        }
        else
        {
            log_msg( prec, 0, "Failed to read BY" );
            msta.Bits.RA_PROBLEM = 1;
        }

        goto finished;
    }

    sword.All = mInfo->sword;
    count     = mInfo->count;
    first     = ! mInfo->initialized;

    if ( first ) mInfo->initialized = 1;
    mInfo->newData = 0;

    mInfo->cMutex->unlock();

    process_motor_info( prec, sword, count );

    msta.All = prec->msta;

    if ( first ) send_msg( mInfo->pasynUser, "Us=18000" );// first status update
    else if ( prec->mip == MIP_DONE )       // done moving, check for slip_stall
    {
        old_diff   = prec->diff;
        prec->diff = prec->rbv - prec->val;
        if ( fabs(prec->diff) > prec->pdbd )
        {
            log_msg( prec, 0, "slipped, diff = %f", prec->diff );
            msta.Bits.EA_SLIP_STALL = 1;
        }

        if ( old_diff != prec->diff ) MARK( M_DIFF );

        goto finished;
    }

    if ( prec->movn ) goto finished;                                   // moving

    old_mip  = prec->mip ;
    old_dmov = prec->dmov;
    old_rcnt = prec->rcnt;
    old_miss = prec->miss;
    old_val  = prec->val ;
    old_dval = prec->dval;
    old_rval = prec->rval;

    diff     = prec->rbv - prec->val;
    if ( first ||                                         // first status update
         (msta.Bits.RA_STALL && (! msta.Bits.RA_SM)) ||               // stalled
         prec->lls || prec->hls ||                         // hit a limit switch
         (prec->mip  & MIP_STOP) || (prec->mip  & MIP_PAUSE) || // stop or pause
         (prec->mip == MIP_HOME)                                ) // done homing
    {
        short newval = 1;

        if ( msta.Bits.RA_STALL && (! msta.Bits.RA_SM) )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                log_msg( prec, 0, "stalled, missed home" );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    log_msg( prec, 0, "desired %f, reached %f",
                                      prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    log_msg( prec, 0, "desired %f, reached %f, missed due to stall",
                                      prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( first )                             // first status update
                log_msg( prec, 0, "initialization completed" );
        }
        else if ( prec->lls )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                log_msg( prec, 0, "hit low limit, missed home" );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    log_msg( prec, 0, "desired %f, reached %f",
                                      prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    log_msg( prec, 0, "desired %f, reached %f, missed due to low limit",
                                      prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( first )                             // first status update
                log_msg( prec, 0, "initialization completed" );
        }
        else if ( prec->hls )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                log_msg( prec, 0, "hit high limit, missed home" );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    log_msg( prec, 0, "desired %f, reached %f",
                                      prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    log_msg( prec, 0, "desired %f, reached %f, missed due to high limit",
                                      prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( first )                             // first status update
                log_msg( prec, 0, "initialization completed" );
        }
        else if ( prec->mip & MIP_STOP  )
        {
            log_msg( prec, 0, "stopped" );
            prec->miss = 0;
        }
        else if ( prec->mip & MIP_PAUSE )
        {
            log_msg( prec, 0, "paused" );
            newval = 0;
        }
        else if ( prec->mip == MIP_HOME )
        {
            log_msg( prec, 0, "homed" );
            prec->miss = 0;
        }
        else if ( first )                                 // first status update
            log_msg( prec, 0, "initialization completed" );

        if ( newval )
        {
            prec->mip  = MIP_DONE;
            prec->dmov = 1;
            prec->val  = prec->rbv;
            prec->dval = prec->drbv;
            prec->rval = prec->rrbv;
            prec->diff = 0;
        }

        prec->rcnt = 0;
    }
    else if ( prec->mip == MIP_NEW ) new_move( prec );
    else if ( prec->mip == MIP_BL  )                              // do backlash
    {
        log_msg( prec, 0, "move to: %f (DVAL: %f), with BACC and BVEL",
                          prec->val, prec->dval );

        prec->mip  = MIP_MOVE;
        prec->rval = NINT(prec->dval / prec->res);

        VI = NINT( prec->bbas / prec->res );
        VM = NINT( prec->bvel / prec->res );
        A  = NINT( (prec->bvel - prec->bbas) / prec->res / prec->bacc );
        sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1MA %d",
                      VI, VM, A, prec->rval );
        send_msg( mInfo->pasynUser, msg    );
        send_msg( mInfo->pasynUser, "Us=0" );
    }
    else if ( (fabs(prec->bdst) <= prec->res ) &&      // no backlash, can retry
              (fabs(diff)       >= prec->rdbd) &&           // not closed enough
              (prec->rtry > 0) && (prec->rcnt < prec->rtry) )  // can retry more
    {
        log_msg( prec, 0, "desired %f, reached %f, retrying %d ...",
                          prec->val, prec->rbv, prec->rcnt++ );

        prec->mip  |= MIP_RETRY;

        sprintf( msg, "MA %d", prec->rval );
        send_msg( mInfo->pasynUser, msg    );
        send_msg( mInfo->pasynUser, "Us=0" );
    }
    else          // finished backlash, close enough, or no (more) retry allowed
    {
        prec->diff = diff;
        if ( fabs(diff) < prec->rdbd )
        {
            log_msg( prec, 0, "desired %f, reached %f",
                              prec->val, prec->rbv             );
            prec->miss = 0;
        }
        else
        {
            log_msg( prec, 0, "desired %f, reached %f after %d retries",
                              prec->val, prec->rbv, prec->rcnt );
            prec->miss = 1;
        }

        prec->mip  = MIP_DONE;
        prec->dmov = 1;
        prec->rcnt = 0;
    }

    if ( old_mip  != prec->mip  ) MARK( M_MIP  );
    if ( old_dmov != prec->dmov ) MARK( M_DMOV );
    if ( old_rcnt != prec->rcnt ) MARK( M_RCNT );
    if ( old_miss != prec->miss ) MARK( M_MISS );
    if ( old_val  != prec->val  ) MARK( M_VAL  );
    if ( old_dval != prec->dval ) MARK( M_DVAL );
    if ( old_rval != prec->rval ) MARK( M_RVAL );

    finished:
    if      ( msta.Bits.RA_PROBLEM                             )     // hardware
        recGblSetSevr( (dbCommon *)prec, COMM_ALARM,  INVALID_ALARM );
    else if ( msta.Bits.RA_BY0     || msta.Bits.RA_COMM_ERR ||  // BY=0 or wrong
              msta.Bits.NOT_INIT                               )     // NOT_INIT
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );
    else if ( msta.Bits.RA_POWERUP || msta.Bits.EA_SLIP_STALL  )
    {
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MINOR_ALARM   );

        if ( msta.Bits.RA_POWERUP    ) log_msg( prec, 0, "power cycled" );
        if ( msta.Bits.EA_SLIP_STALL ) log_msg( prec, 0, "slip_stall"   );
    }
    else if ( msta.Bits.RA_STALL                               )      // stalled
    {
        if ( prec->stsv > NO_ALARM )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, prec->stsv    );

        log_msg( prec, 0, "stall detected" );
    }

    prec->msta = msta.All;
    if ( old_msta != prec->msta ) MARK( M_MSTA );

    recGblGetTimeStamp( prec );

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 0 );
    post_msgs  ( prec                );

    recGblFwdLink     ( prec );                 // process the forward scan link

    prec->proc = 0;
    prec->pact = 0;

    return( status );
}

/******************************************************************************/
static long process_motor_info( imsRecord *prec, status_word sword, long count )
{
    ims_info     *mInfo = (ims_info *)prec->dpvt;
    short         old_movn, old_rlls, old_rhls, old_lls, old_hls;
    double        old_drbv, old_rbv;
    long          old_rrbv;
    motor_status  msta;

    int           dir = (prec->dir == motorDIR_Positive) ? 1 : -1;

    old_movn     = prec->movn;
    old_rlls     = prec->rlls;
    old_rhls     = prec->rhls;
    old_lls      = prec->lls ;
    old_hls      = prec->hls ;
    old_rrbv     = prec->rrbv;
    old_drbv     = prec->drbv;
    old_rbv      = prec->rbv ;

    msta.All             = 0;

    msta.Bits.RA_MOVING  = sword.Bits.MOVING;
    msta.Bits.RA_EE      = sword.Bits.EE    ;
    msta.Bits.RA_SM      = sword.Bits.SM    ;
    msta.Bits.RA_STALL   = sword.Bits.ST    ;
    msta.Bits.RA_POWERUP = sword.Bits.PU    ;
    msta.Bits.RA_NE      = sword.Bits.NE    ;
    msta.Bits.RA_ERR     = sword.Bits.ERR & 127 ;

    prec->movn = msta.Bits.RA_MOVING;

    prec->rlls = 0;
    prec->rhls = 0;
    if ( mInfo->S1 == 3 ) prec->rlls = sword.Bits.I1;
    if ( mInfo->S2 == 3 ) prec->rlls = sword.Bits.I2;
    if ( mInfo->S3 == 3 ) prec->rlls = sword.Bits.I3;

    if ( mInfo->S1 == 2 ) prec->rhls = sword.Bits.I1;
    if ( mInfo->S2 == 2 ) prec->rhls = sword.Bits.I2;
    if ( mInfo->S3 == 2 ) prec->rhls = sword.Bits.I3;

    msta.Bits.RA_MINUS_LS = prec->rlls;
    msta.Bits.RA_PLUS_LS  = prec->rhls;

    prec->lls  = (dir == 1) ? prec->rlls : prec->rhls;
    prec->hls  = (dir == 1) ? prec->rhls : prec->rlls;

    prec->rrbv = count;
    prec->drbv = prec->rrbv * prec->res;

    prec->rbv  = dir * prec->drbv + prec->off;

    prec->msta = msta.All;

    if ( old_movn != prec->movn) MARK( M_MOVN );
    if ( old_rlls != prec->rlls) MARK( M_RLLS );
    if ( old_rhls != prec->rhls) MARK( M_RHLS );
    if ( old_lls  != prec->lls ) MARK( M_LLS  );
    if ( old_hls  != prec->hls ) MARK( M_HLS  );
    if ( old_rrbv != prec->rrbv) MARK( M_RRBV );
    if ( old_drbv != prec->drbv) MARK( M_DRBV );
    if ( old_rbv  != prec->rbv ) MARK( M_RBV  );

    return( 0 );
}

/******************************************************************************/
static void new_move( imsRecord *prec )
{
    ims_info       *mInfo = (ims_info *)prec->dpvt;
    char            msg[MAX_MSG_SIZE];
    int             VI, VM, A;

    if ( fabs(prec->bdst) > prec->res )                              // backlash
    {
        if ( ((prec->bdst > 0) && (prec->drbv > prec->dval)) ||
             ((prec->bdst < 0) && (prec->drbv < prec->dval)) ||
             (fabs(prec->drbv - prec->dval) > prec->bdst   )    )
        {           // opposite direction, or long move, use ACCL and VELO first
            log_msg( prec, 0, "move to: %f (DVAL: %f), with ACCL and VELO",
                              prec->val, prec->dval-prec->bdst );

            prec->mip  = MIP_BL  ;
            prec->rval = NINT((prec->dval - prec->bdst) / prec->res);

            VI = NINT( prec->vbas / prec->res );
            VM = NINT( prec->velo / prec->res );
            A  = NINT( (prec->velo - prec->vbas) / prec->res / prec->accl );
            sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1MA %d",
                          VI, VM, A, prec->rval );
        }
        else                // same direction and within BDST, use BACC and BVEL
        {
            log_msg( prec, 0, "move to: %f (DVAL: %f), with BACC and BVEL",
                              prec->val, prec->dval            );

            prec->mip  = MIP_MOVE;
            prec->rval = NINT(prec->dval                / prec->res);

            VI = NINT( prec->bbas / prec->res );
            VM = NINT( prec->bvel / prec->res );
            A  = NINT( (prec->bvel - prec->bbas) / prec->res / prec->bacc );
            sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1MA %d",
                          VI, VM, A, prec->rval );
        }
    }
    else                                       // no backlash, use ACCL and VELO
    {
        log_msg( prec, 0, "move to: %f (DVAL: %f), with ACCL and VELO",
                          prec->val, prec->dval );

        prec->mip  = MIP_MOVE;
        prec->rval = NINT(prec->dval / prec->res);

        VI = NINT( prec->vbas / prec->res );
        VM = NINT( prec->velo / prec->res );
        A  = NINT( (prec->velo - prec->vbas) / prec->res / prec->accl );
        sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1MA %d",
                      VI, VM, A, prec->rval );
    }

    prec->dmov = 0;
    send_msg( mInfo->pasynUser, msg    );
    send_msg( mInfo->pasynUser, "Us=0" );

    return;
}

/******************************************************************************/
static long special( dbAddr *pDbAddr, int after )
{
    imsRecord      *prec = (imsRecord *) pDbAddr->precord;
    ims_info       *mInfo = (ims_info *)prec->dpvt;
    char            MI = (prec->hmtp == motorHMTP_Switch) ? 'M' : 'I';
    char            msg[MAX_MSG_SIZE], rbbuf[MAX_MSG_SIZE];
    char            fmt[MAX_MSG_SIZE], rbstr[MAX_MSG_SIZE];
    long            sword, count, old_rval, new_rval;
    short           old_dmov, old_rcnt, old_lvio;
    double          nval, old_val, old_dval, old_rbv, new_dval;
    unsigned short  old_mip, alarm_mask = 0;

    int             VI, VM, A, fieldIndex = dbGetFieldIndex( pDbAddr );
    int             retry, status = OK;

    if ( after == 0 )
    {
        if      ( fieldIndex == imsRecordVAL  ) prec->oval = prec->val ;
        else if ( fieldIndex == imsRecordDVAL ) prec->oval = prec->dval;
        else if ( fieldIndex == imsRecordSPG  ) prec->oval = prec->spg ;
        else if ( fieldIndex == imsRecordDIR  ) prec->oval = prec->dir ;
        else if ( fieldIndex == imsRecordOFF  ) prec->oval = prec->off ;
        else if ( fieldIndex == imsRecordSET  ) prec->oval = prec->set ;
        else if ( fieldIndex == imsRecordLLM  ) prec->oval = prec->llm ;
        else if ( fieldIndex == imsRecordHLM  ) prec->oval = prec->hlm ;
        else if ( fieldIndex == imsRecordDLLM ) prec->oval = prec->dllm;
        else if ( fieldIndex == imsRecordDHLM ) prec->oval = prec->dhlm;
        else if ( fieldIndex == imsRecordEL   ) prec->oval = prec->el  ;
        else if ( fieldIndex == imsRecordMS   ) prec->oval = prec->ms  ;
        else if ( fieldIndex == imsRecordELMS ) prec->oval = prec->elms;
        else if ( fieldIndex == imsRecordUREV ) prec->oval = prec->urev;

        return( OK );
    }

    old_val  = prec->val ;
    old_dval = prec->dval;
    old_rval = prec->rval;
    old_rbv  = prec->rbv ;
    old_mip  = prec->mip ;
    old_dmov = prec->dmov;
    old_rcnt = prec->rcnt;
    old_lvio = prec->lvio;

    switch( fieldIndex )
    {
        case imsRecordSSTR:
            log_msg( prec, 1, "%s", prec->sstr );

            status = sscanf( prec->sstr, "%ld,P=%ldEOS", &sword, &count );
            if ( status == 2 )
            {
                log_msg( prec, 2, "Status=%d,P=%d", sword, count );

                mInfo->cMutex->lock();

                mInfo->sword   = sword;
                mInfo->count   = count;
                mInfo->newData = 1;
            }
            else
            {
                log_msg( prec, 0, "Erroneous status string" );

                mInfo->cMutex->lock();

                mInfo->newData = 0;
            }

            mInfo->cMutex->unlock();

            break;
        case imsRecordVAL :
            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
            {                                    // violated the software limits
                prec->lvio = 1;                     // set limit violation alarm
                prec->val  = prec->oval;

                break;
            }

            do_move1:
            if ( prec->set == motorSET_Use )            // do it only when "Use"
                prec->dval = (prec->val - prec->off) * (1. - 2.*prec->dir);

            goto do_move2;
        case imsRecordDVAL:
            if ( (prec->dval < prec->dllm) || (prec->dval > prec->dhlm) )
            {                                    // violated the hardware limits
                prec->lvio = 1;                     // set limit violation alarm
                prec->dval = prec->oval;

                break;
            }

            prec->val  = prec->dval * (1. - 2.*prec->dir) + prec->off;

            do_move2:
            if ( prec->set == motorSET_Set )                          // no move
            {
                break;
            }

            prec->lvio = 0;
            prec->rcnt = 0;

            if ( prec->spg != motorSPG_Go ) break;

            if ( prec->dmov == 0 )                               // still moving
            {
                if ( prec->mip != MIP_NEW )           // stop current move first
                {
                    log_msg( prec, 0, "stop current move" );
                    prec->mip  = MIP_NEW;

                    send_msg( mInfo->pasynUser, "SL 0\r\n1Us=0" );
                }

                break;
            }

            new_move( prec );

            break;
        case imsRecordTWF :
            nval = prec->val + prec->twv;
            goto tweak;
        case imsRecordTWR :
            nval = prec->val - prec->twv;

            tweak:
            if ( (nval < prec->llm) || (nval > prec->hlm) )
            {                                    // violated the software limits
                prec->lvio = 1;                     // set limit violation alarm

                break;
            }

            prec->val = nval;
            goto do_move1;
        case imsRecordSPG :
            if ( (prec->spg == prec->oval) || (prec->mip == MIP_DONE) ) break;

            if ( prec->spg == motorSPG_Go )
            {
                log_msg( prec, 0, "resume moving" );
                prec->mip &= ~( MIP_STOP | MIP_PAUSE );

                sprintf( msg, "MA %d", prec->rval );
                send_msg( mInfo->pasynUser, msg );

                break;
            }
            else
            {
                if ( prec->spg == motorSPG_Stop )
                {
                    log_msg( prec, 0, "stop, with deceleration" );
                    prec->mip &= ~MIP_PAUSE;
                    prec->mip |=  MIP_STOP ;
                }
                else
                {
                    log_msg( prec, 0, "pause, with deceleration" );
                    prec->mip |=  MIP_PAUSE;
                }

                send_msg( mInfo->pasynUser, "SL 0\r\n1Us=0" );
            }

            break;
        case ( imsRecordDIR  ):
            if ( prec->dir == prec->oval ) break;

            if ( prec->dir == motorDIR_Positive )                // was negative
            {
                prec->llm = prec->off + prec->dllm;
                prec->hlm = prec->off + prec->dhlm;
            }
            else
            {
                prec->llm = prec->off - prec->dhlm;
                prec->hlm = prec->off - prec->dllm;
            }

            nval       = prec->lls;
            prec->lls  = prec->hls;
            prec->hls  = nval;
            if ( prec->lls != nval )
            {
                db_post_events( prec, &prec->lls,  DBE_VAL_LOG );
                db_post_events( prec, &prec->hls,  DBE_VAL_LOG );
            }

            goto change_dir_off;
        case ( imsRecordOFF  ):
            prec->llm += prec->off - prec->oval;
            prec->hlm += prec->off - prec->oval;

            change_dir_off:
            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

            prec->rbv  = prec->drbv * (1. - 2.*prec->dir) + prec->off;
            prec->val  = prec->rbv;

//          check_software_limits( prec );

            break;
        case imsRecordSET :
            if ( (prec->set == prec->oval  ) ||
                 (prec->set == motorSET_Set)    ) break;

            prec->rbv  = prec->val;
            if ( prec->foff == motorOFF_Variable )
            {
                prec->off  = prec->rbv - prec->drbv * (1. - 2.*prec->dir);
                db_post_events( prec, &prec->off,  DBE_VAL_LOG );
            }
            else
            {
                new_dval = (prec->rbv - prec->off) * (1. - 2.*prec->dir);
                new_rval = NINT(new_dval / prec->res);

                if ( prec->ee == motorAble_Enable )
                    sprintf( msg, "P=%ld\r\n1C2=%ld\r\n1Us=0", new_rval,
                                                               new_rval );
                else
                    sprintf( msg, "P=%ld\r\n1Us=0",            new_rval );

                send_msg( mInfo->pasynUser, msg );
            }

            break;
        case imsRecordHOMF:
            if ( prec->spg != motorSPG_Go ) break;

            VI = NINT( prec->hbas / prec->res );
            VM = NINT( prec->hvel / prec->res );
            A  = NINT( (prec->hvel - prec->hbas) / prec->res / prec->hacc );
            if      ( (prec->dir  == motorDIR_Positive) &&
                      (prec->hmdr == motorDIR_Positive)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 4\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Positive) &&
                      (prec->hmdr == motorDIR_Negative)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 3\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Negative) &&
                      (prec->hmdr == motorDIR_Positive)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 2\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Negative) &&
                      (prec->hmdr == motorDIR_Negative)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 1\r\n1Us=0",
                              VI, VM, A, MI );
 
            prec->mip  = MIP_HOMF;
            send_msg( mInfo->pasynUser, msg );

            break;
        case imsRecordHOMR:
            if ( prec->spg != motorSPG_Go ) break;

            VI = NINT( prec->hbas / prec->res );
            VM = NINT( prec->hvel / prec->res );
            A  = NINT( (prec->hvel - prec->hbas) / prec->res / prec->hacc );
            if      ( (prec->dir  == motorDIR_Positive) &&
                      (prec->hmdr == motorDIR_Positive)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 1\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Positive) &&
                      (prec->hmdr == motorDIR_Negative)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 2\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Negative) &&
                      (prec->hmdr == motorDIR_Positive)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 3\r\n1Us=0",
                              VI, VM, A, MI );
            else if ( (prec->dir  == motorDIR_Negative) &&
                      (prec->hmdr == motorDIR_Negative)    )
                sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1H%c 4\r\n1Us=0",
                              VI, VM, A, MI );
 
            prec->mip  = MIP_HOMR;
            send_msg( mInfo->pasynUser, msg );

            break;
        case imsRecordLLM :
            if ( prec->llm > prec->hlm )           // illegal, restore old value
            {
                prec->llm  = prec->oval;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
                break;
            }

            if ( prec->dir == motorDIR_Positive )
            {
                prec->dllm = prec->llm - prec->off;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
            }
            else
            {
                prec->dhlm = prec->off - prec->llm;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
            }

            check_limit_violation:
            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
                prec->lvio = 1;                   // set limit violation warning

            break;
        case imsRecordHLM :
            if ( prec->hlm < prec->llm )           // illegal, restore old value
            {
                prec->hlm  = prec->oval;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
                break;
            }

            if ( prec->dir == motorDIR_Positive )
            {
                prec->dhlm = prec->hlm - prec->off;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
            }
            else
            {
                prec->dllm = prec->off - prec->hlm;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case imsRecordDLLM:
            if ( prec->dllm > prec->dhlm )         // illegal, restore old value
            {
                prec->dllm = prec->oval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
                break;
            }

            if ( prec->dir == motorDIR_Positive )
            {
                prec->llm  = prec->off + prec->dllm;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }
            else
            {
                prec->hlm  = prec->off - prec->dllm;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case imsRecordDHLM:
            if ( prec->dhlm < prec->dllm )         // illegal, restore old value
            {
                prec->dhlm = prec->oval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
                break;
            }

            if ( prec->dir == motorDIR_Positive )
            {
                prec->hlm  = prec->off + prec->dhlm;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }
            else
            {
                prec->llm  = prec->off - prec->dhlm;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case imsRecordEL  :
            if ( prec->el   <= 0 )
            {
                prec->el   = prec->oval;
                db_post_events( prec, &prec->el  , DBE_VAL_LOG );
                break;
            }

            set_el:
            sprintf( msg, "EL=%d", prec->el );
            send_msg( mInfo->pasynUser, msg );

            if ( prec->ee == motorAble_Enable ) goto set_res;

            break;
        case imsRecordMS  :
            if ( prec->ms   <= 0 )
            {
                prec->ms   = prec->oval;
                db_post_events( prec, &prec->ms  , DBE_VAL_LOG );
                break;
            }

            set_ms:
            sprintf( msg, "MS=%d", prec->ms );
            send_msg( mInfo->pasynUser, msg );

            if ( prec->ee != motorAble_Enable ) goto set_res;

            break;
        case imsRecordELMS:
            if ( prec->elms <= 0 )
            {
                prec->elms = prec->oval;
                db_post_events( prec, &prec->elms, DBE_VAL_LOG );
                break;
            }

            if ( prec->ee == motorAble_Enable )
            {
                prec->el   = prec->elms;
                db_post_events( prec, &prec->el  , DBE_VAL_LOG );
                goto set_el;
            }
            else
            {
                prec->ms   = prec->elms;
                db_post_events( prec, &prec->ms  , DBE_VAL_LOG );
                goto set_ms;
            }
        case imsRecordUREV:
            if ( prec->urev <= 0. )
            {
                prec->urev = prec->oval;
                db_post_events( prec, &prec->urev, DBE_VAL_LOG );
                break;
            }

            set_res:
            nval = prec->res;
            if ( prec->ee == motorAble_Enable )
                prec->res = prec->urev / prec->el /   4.;
            else
                prec->res = prec->urev / prec->ms / 200.;

            if ( prec->res != nval )
            {
                prec->drbv = prec->rrbv * prec->res;
                prec->rbv  = (1. - 2.*prec->dir) * prec->drbv + prec->off;

                db_post_events( prec, &prec->res,  DBE_VAL_LOG );
            }

            break;
        case imsRecordVBAS:
        case imsRecordVMAX:
            if ( prec->vbas > prec->vmax )
            {
                prec->vbas = prec->vmax;
                db_post_events( prec, &prec->vbas, DBE_VAL_LOG );
            }

            prec->sbas = prec->vbas / prec->urev;
            prec->smax = prec->vmax / prec->urev;
            db_post_events( prec, &prec->sbas, DBE_VAL_LOG );
            db_post_events( prec, &prec->smax, DBE_VAL_LOG );
        case imsRecordVELO:
            force_velo_in_range:
            if ( prec->velo < prec->vbas )
            {
                prec->velo = prec->vbas;
                db_post_events( prec, &prec->velo, DBE_VAL_LOG );
            }

            if ( prec->velo > prec->vmax )
            {
                prec->velo = prec->vmax;
                db_post_events( prec, &prec->velo, DBE_VAL_LOG );
            }

            prec->s    = prec->velo / prec->urev;
            db_post_events( prec, &prec->s,    DBE_VAL_LOG );

            if ( prec->bbas > prec->vmax )
            {
                prec->bbas = prec->vmax;
                db_post_events( prec, &prec->bbas, DBE_VAL_LOG );
            }

            if ( prec->bvel > prec->vmax )
            {
                prec->bvel = prec->vmax;
                db_post_events( prec, &prec->bvel, DBE_VAL_LOG );
            }

            if ( prec->bvel < prec->bbas )
            {
                prec->bvel = prec->bbas;
                db_post_events( prec, &prec->bvel, DBE_VAL_LOG );
            }

            if ( prec->hbas > prec->vmax )
            {
                prec->hbas = prec->vmax;
                db_post_events( prec, &prec->hbas, DBE_VAL_LOG );
            }

            if ( prec->hvel > prec->vmax )
            {
                prec->hvel = prec->vmax;
                db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
            }

            if ( prec->hvel < prec->hbas )
            {
                prec->hvel = prec->hbas;
                db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
            }

            break;
        case imsRecordSBAS:
        case imsRecordSMAX:
            if ( prec->sbas > prec->smax )
            {
                prec->sbas = prec->smax;
                db_post_events( prec, &prec->sbas, DBE_VAL_LOG );
            }

            prec->vbas = prec->sbas * prec->urev;
            prec->vmax = prec->smax * prec->urev;
            db_post_events( prec, &prec->vbas, DBE_VAL_LOG );
            db_post_events( prec, &prec->vmax, DBE_VAL_LOG );

            goto force_velo_in_range;
        case imsRecordS   :
            if ( prec->s    < prec->sbas )
            {
                prec->s    = prec->sbas;
                db_post_events( prec, &prec->s,    DBE_VAL_LOG );
            }

            if ( prec->s    > prec->smax )
            {
                prec->s    = prec->smax;
                db_post_events( prec, &prec->s,    DBE_VAL_LOG );
            }

            prec->velo = prec->s    * prec->urev;
            db_post_events( prec, &prec->velo, DBE_VAL_LOG );

            break;
        case imsRecordBBAS:
        case imsRecordBVEL:
            if ( prec->bbas > prec->vmax )
            {
                prec->bbas = prec->vmax;
                db_post_events( prec, &prec->bbas, DBE_VAL_LOG );
            }

            if ( prec->bvel > prec->vmax )
            {
                prec->bvel = prec->vmax;
                db_post_events( prec, &prec->bvel, DBE_VAL_LOG );
            }

            if ( prec->bvel < prec->bbas )
            {
                prec->bvel = prec->bbas;
                db_post_events( prec, &prec->bvel, DBE_VAL_LOG );
            }

            break;
        case imsRecordHBAS:
        case imsRecordHVEL:
            if ( prec->hbas > prec->vmax )
            {
                prec->hbas = prec->vmax;
                db_post_events( prec, &prec->hbas, DBE_VAL_LOG );
            }

            if ( prec->hvel > prec->vmax )
            {
                prec->hvel = prec->vmax;
                db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
            }

            if ( prec->hvel < prec->hbas )
            {
                prec->hvel = prec->hbas;
                db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
            }

            break;
        case imsRecordWRTE:
            send_msg( mInfo->pasynUser, prec->wrte );

            break;
        case imsRecordREAD:
            sprintf( msg, "PR \"%s=\",%s", prec->read, prec->read );
            sprintf( fmt, "%s=%%s",        prec->read             );

            retry = 0;
            do
            {
                send_msg( mInfo->pasynUser, msg );
                status = recv_reply( mInfo->pasynUser, rbbuf, 1 );
                if ( status > 0 )
                {
                    status = sscanf( rbbuf, fmt, rbstr );
                    if ( status == 1 )
                    {
                        strncpy( prec->read, rbstr, 61 );
                        db_post_events( prec,  prec->read, DBE_VAL_LOG );

                        break;
                    }
                }

                epicsThreadSleep( 0.3 );
            } while ( retry++ < 3 );

            break;
        case imsRecordRINI:
            init_motor( prec );

            prec->rini = 0;
            break;
    }

    if ( prec->val  != old_val  ) MARK( M_VAL  );
    if ( prec->dval != old_dval ) MARK( M_DVAL );
    if ( prec->rval != old_rval ) MARK( M_RVAL );
    if ( prec->rbv  != old_rbv  ) MARK( M_RBV  );
    if ( prec->mip  != old_mip  ) MARK( M_MIP  );
    if ( prec->dmov != old_dmov ) MARK( M_DMOV );
    if ( prec->rcnt != old_rcnt ) MARK( M_RCNT );
    if ( prec->lvio != old_lvio ) MARK( M_LVIO );

    post_fields( prec, alarm_mask, 0 );

    return( 0 );
}

/******************************************************************************/
static long cvt_dbaddr( dbAddr *pDbAddr )
{
    imsRecord *prec = (imsRecord *)pDbAddr->precord;
    ims_info  *mInfo = (ims_info *)prec->dpvt;

    int        fieldIndex = dbGetFieldIndex( pDbAddr );
    long       status = 0;

    switch ( fieldIndex )
    {
        case imsRecordLOGA:
        {
            pDbAddr->pfield         = (char *)prec->loga;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGB:
        {
            pDbAddr->pfield         = (char *)prec->logb;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGC:
        {
            pDbAddr->pfield         = (char *)prec->logc;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGD:
        {
            pDbAddr->pfield         = (char *)prec->logd;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGE:
        {
            pDbAddr->pfield         = (char *)prec->loge;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGF:
        {
            pDbAddr->pfield         = (char *)prec->logf;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGG:
        {
            pDbAddr->pfield         = (char *)prec->logg;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case imsRecordLOGH:
        {
            pDbAddr->pfield         = (char *)prec->logh;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
    }

    return( status );
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
static long send_msg( asynUser *pasynUser, char const *msg )
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
static long recv_reply( asynUser *pasynUser, char *rbbuf, int flag )
{
    const double timeout  = 1.0;
    size_t       nread    = 0;
    asynStatus   asyn_rtn = asynError;
    int          eomReason;

    if ( flag == FLUSH )
        pasynOctetSyncIO->flush( pasynUser );
    else
        asyn_rtn = pasynOctetSyncIO->read( pasynUser, rbbuf, MAX_MSG_SIZE,
                                           timeout, &nread, &eomReason );

    if ( (asyn_rtn != asynSuccess) || (nread <= 0) )
    {
        rbbuf[0] = '\0';
        nread    = 0;
    }

    return( nread );
}

/******************************************************************************/
static void post_fields( imsRecord *prec, unsigned short alarm_mask,
                                          unsigned short all )
{
    unsigned short  field_mask;
    changed_fields  cmap;

    cmap.All = prec->cmap;

    if ( (field_mask = alarm_mask | (all                  ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->vers, field_mask );
        db_post_events( prec,  prec->port, field_mask );
        db_post_events( prec,  prec->asyn, field_mask );
        db_post_events( prec,  prec->pn,   field_mask );
        db_post_events( prec, &prec->ee,   field_mask );
        db_post_events( prec, &prec->sm,   field_mask );
        db_post_events( prec,  prec->egu,  field_mask );
        db_post_events( prec,  prec->desc, field_mask );
        db_post_events( prec, &prec->dllm, field_mask );
        db_post_events( prec, &prec->dhlm, field_mask );
        db_post_events( prec, &prec->llm,  field_mask );
        db_post_events( prec, &prec->hlm,  field_mask );
/*
        db_post_events( prec, &prec->svel, field_mask );
        db_post_events( prec, &prec->sacc, field_mask );
        db_post_events( prec, &prec->velo, field_mask );
        db_post_events( prec, &prec->accl, field_mask );
        db_post_events( prec, &prec->shve, field_mask );
        db_post_events( prec, &prec->shac, field_mask );
        db_post_events( prec, &prec->hvel, field_mask );
        db_post_events( prec, &prec->hacc, field_mask );
        db_post_events( prec, &prec->bdst, field_mask );
        db_post_events( prec, &prec->dir,  field_mask );
        db_post_events( prec, &prec->off,  field_mask );
        db_post_events( prec, &prec->set,  field_mask );
        db_post_events( prec, &prec->twv,  field_mask );
        db_post_events( prec, &prec->twf,  field_mask );
        db_post_events( prec, &prec->twr,  field_mask );
        db_post_events( prec, &prec->rtry, field_mask );
        db_post_events( prec, &prec->rdbd, field_mask );
        db_post_events( prec, &prec->pdbd, field_mask );
        db_post_events( prec, &prec->spg,  field_mask );
        db_post_events( prec, &prec->home, field_mask );
        db_post_events( prec, &prec->refr, field_mask );
        db_post_events( prec, &prec->init, field_mask );
*/
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_VAL ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->val,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DVAL) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dval, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RVAL) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rval, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RRBV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rrbv, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DRBV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->drbv, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RBV ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rbv,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DIFF) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->diff, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MIP ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->mip,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MOVN) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->movn, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DMOV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dmov, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RCNT) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rcnt, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MISS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->miss, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LVIO) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lvio, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MSTA) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->msta, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RLLS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rlls, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RHLS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rhls, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lls,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_HLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->hls,  field_mask );

    UNMARK_ALL;

    return;
}

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

/******************************************************************************/
static long log_msg( imsRecord *prec, int dlvl, const char *fmt, ... )
{
    ims_info  *mInfo = (ims_info *)prec->dpvt;
    timespec   ts;
    struct tm  timeinfo;
    char       timestamp[40], msec[4], msg[512];

    va_list    args;

    if ( (dlvl > prec->dlvl) && (dlvl > imsRecordDebug) ) return( 0 );

    clock_gettime( CLOCK_REALTIME, &ts );
    localtime_r( &ts.tv_sec, &timeinfo );

    strftime( timestamp, 40, "%m/%d %H:%M:%S", &timeinfo );
    sprintf ( msec, "%03d", int(ts.tv_nsec*1.e-6 + 0.5) );

    va_start( args, fmt      );
    vsprintf( msg, fmt, args );
    va_end  ( args           );

    if ( dlvl <= prec->dlvl )
    {
        mInfo->lMutex->lock();

        if ( mInfo->cIndex > 7 )
            memmove( mInfo->sAddr,
                     mInfo->sAddr+mInfo->mLength, mInfo->mLength*7 );

        snprintf( mInfo->sAddr+mInfo->mLength*min(mInfo->cIndex,7), 61,
                  "%s %s", timestamp+6, msg );

        if ( mInfo->cIndex <= 7 ) mInfo->cIndex++;

        mInfo->lMutex->unlock();
    }

    if ( dlvl <= imsRecordDebug )
        printf( "%s.%s %s -- %s\n", timestamp, msec, prec->name, msg );

    return( 1 );
}

/******************************************************************************/
static void post_msgs( imsRecord *prec )
{
    db_post_events( prec,  prec->loga, DBE_VAL_LOG );
    db_post_events( prec,  prec->logb, DBE_VAL_LOG );
    db_post_events( prec,  prec->logc, DBE_VAL_LOG );
    db_post_events( prec,  prec->logd, DBE_VAL_LOG );
    db_post_events( prec,  prec->loge, DBE_VAL_LOG );
    db_post_events( prec,  prec->logf, DBE_VAL_LOG );
    db_post_events( prec,  prec->logg, DBE_VAL_LOG );
    db_post_events( prec,  prec->logh, DBE_VAL_LOG );

    return;
}

