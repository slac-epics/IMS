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

static long send_msg  ( asynUser *pasynUser, char const *msg     );
static long recv_reply( asynUser *pasynUser, char *buf, int flag );


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
static long connect_motor( imsRecord *prec )
{
    ims_info          *mInfo = (ims_info *)prec->dpvt;

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
static long init_motor( imsRecord *prec )
{
    ims_info *mInfo = (ims_info *)prec->dpvt;

    asynUser *pasynUser = mInfo->pasynUser;
    char      buf[MAX_MSG_SIZE];
    int       s1, s2, s3, a1, a2, a3, d1, d2, d3;
    int       status = 0, retry, rtnval, byv, failed;

    mInfo->cMutex->lock();

    // read the part number
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

    // read the serial number
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR SN" );
        status = recv_reply( pasynUser, prec->sn, 1 );
        epicsThreadSleep( 0.5 );
    } while ( status <= 0 && retry++ < 3 );

    if ( status <= 0 )
    {
        Debug( 0, "Failed to read the serial number" );

        failed = 1;
        goto finish_up;
    }

    // read the switch settings
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"S1=\",S1,\", S2=\",S2,\", S3=\",S3" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "S1=%d, %d, %d, S2=%d, %d, %d, S3=%d, %d, %d",
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

            if ( status == 9 ) break;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 9 )
    {
        Debug( 0, "Failed to read the switch settings" );

        failed = 1;
    }

    // determine if numeric enabled
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"NE=\",NE" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "NE=%d", &rtnval );
            if ( (status == 1) && (rtnval == 0 || rtnval ==1) )
            {
                prec->ne = rtnval;                        // 1 : numeric enabled
                break;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 1 )
    {
        Debug( 0, "Failed to read the numeric status" );
        prec->ne = 0;

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
                prec->ee = rtnval;                        // 1 : encoder enabled
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
        prec->ee = motorAble_Disable;

        failed = 1;
    }

    // determine the limit stop mode
    retry = 0;
    do
    {
        send_msg( pasynUser, "PR \"LM=\",LM" );
        status = recv_reply( pasynUser, buf, 1 );
        if ( status > 0 )
        {
            status = sscanf( buf, "LM=%d", &rtnval );
            if ( (status == 1) && (rtnval > 0 && rtnval < 7) )
            {
                prec->lm = rtnval;
                break;
            }
            else
                status = 0;
        }

        epicsThreadSleep( 0.5 );
    } while ( retry++ < 3 );

    if ( status != 1 )
    {
        Debug( 0, "Failed to read the limit stop mode" );
        prec->lm = 0;

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
            if ( (status == 1) && (rtnval == 0 || rtnval == 1) )
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

    mInfo->init = 0;

    if ( prec->ee == motorAble_Enable ) prec->res = prec->eres;
    else                                prec->res = prec->mres;

    // make sure the limits are consistent

    // make sure the velo's are consistent

    prec->dmov = 1;
    prec->mip  = MIP_DONE;

    // let controller send a status update every ~5 seconds until first process
    send_msg( pasynUser, "Us=300" );

    finish_up:
    mInfo->cMutex->unlock();

    return( 0 );
}


/******************************************************************************/
static long process( dbCommon *precord )
{
    imsRecord      *prec = (imsRecord *)precord;
    ims_info       *mInfo = (ims_info *)prec->dpvt;
    status_word     sword;
    char            msg[MAX_MSG_SIZE];
    unsigned short  old_mip,  alarm_mask;
    short           init, old_dmov, old_rcnt, old_miss;
    double          old_val,  old_dval, old_diff, diff;
    long            count, old_rval, status = OK;
    int             VI, VM, A;

    if ( prec->pact ) return( OK );

    prec->pact = 1;
    Debug( 1, "%s -- process", prec->name );

    // old_utcs = prec->time;
    // old_msta = prec->msta;

    mInfo->cMutex->lock();
    if ( ! mInfo->newData )                                // no new status data
    {
        mInfo->cMutex->unlock();

        //check time difference since last update, set communication warning bit
        goto finished;
    }

    init      = mInfo->init;
    sword.All = mInfo->sword;
    count     = mInfo->count;

    mInfo->newData = 0;
    if ( mInfo->init == 0 )                               // first status update
    {
        mInfo->init = 1;
        send_msg( mInfo->pasynUser, "Us=18000" );
    }

    mInfo->cMutex->unlock();

    // reset communication warning bit

    recGblGetTimeStamp( prec );

    process_motor_info( prec, sword, count );

    if ( sword.Bits.ST )                                              // stalled
    {  // set msta stall bit
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, prec->stsv );
    }

    old_diff = prec->diff;
    if ( init && (prec->mip == MIP_DONE) )  // done moving, check for slip_stall
    {
        prec->diff = prec->rbv - prec->val;
        if ( fabs(prec->diff) > prec->pdbd )                          // slipped
        {  // set slip_stall bit
            recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MINOR_ALARM );
        }

        goto finished;
    }

    if ( prec->movn ) goto finished;                             // still moving

    old_mip  = prec->mip ;
    old_dmov = prec->dmov;
    old_rcnt = prec->rcnt;
    old_miss = prec->miss;
    old_val  = prec->val ;
    old_dval = prec->dval;
    old_rval = prec->rval;

    diff     = prec->rbv - prec->val;
    if ( (! init) ||                                      // first status update
         (sword.Bits.ST && (! sword.Bits.SM)) ||                      // stalled
         prec->lls || prec->hls ||                         // hit a limit switch
         (prec->mip  & MIP_STOP) || (prec->mip  & MIP_PAUSE) || // stop or pause
         (prec->mip == MIP_HOME)                                ) // done homing
    {
        short newval = 1;

        if ( sword.Bits.ST && (! sword.Bits.SM) )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                Debug( 0, "%s -- stalled, missed home", prec->name );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    Debug( 0, "%s -- desired %f, reached %f",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    Debug( 0, "%s -- desired %f, reached %f, missed due to stall",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( ! init )                            // first status update
                Debug( 0, "%s -- initialized", prec->name );
        }
        else if ( prec->lls )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                Debug( 0, "%s -- hit low limit, missed home", prec->name );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    Debug( 0, "%s -- desired %f, reached %f",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    Debug( 0, "%s -- desired %f, reached %f, missed due to low limit",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( ! init )                            // first status update
                Debug( 0, "%s -- initialized", prec->name );
        }
        else if ( prec->hls )
        {
            if ( prec->mip == MIP_HOME )                           // was homing
            {
                Debug( 0, "%s -- hit high limit, missed home", prec->name );
                prec->miss = 1;
            }
            else if ( (prec->mip != MIP_DONE) &&
                      !(prec->mip & (MIP_STOP || MIP_PAUSE)) )     // was moving
            {
                if ( fabs(diff) < prec->rdbd )
                {
                    Debug( 0, "%s -- desired %f, reached %f",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 0;
                }
                else
                {
                    Debug( 0, "%s -- desired %f, reached %f, missed due to high limit",
                              prec->name, prec->val, prec->rbv );
                    prec->miss = 1;
                }
            }
            else if ( ! init )                            // first status update
                Debug( 0, "%s -- initialized", prec->name );
        }
        else if ( prec->mip & MIP_STOP  )
        {
            Debug( 0, "%s -- stopped",     prec->name );
            prec->miss = 0;
        }
        else if ( prec->mip & MIP_PAUSE )
        {
            Debug( 0, "%s -- paused",      prec->name );
            newval = 0;
        }
        else if ( prec->mip == MIP_HOME )
        {
            Debug( 0, "%s -- homed",       prec->name );
            prec->miss = 0;
        }
        else if ( ! init )                                // first status update
            Debug( 0, "%s -- initialized", prec->name );

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
        Debug( 0, "%s -- move to: %f (DVAL: %f), with BACC and BVEL",
                  prec->name, prec->val, prec->dval );

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
        Debug( 0, "%s -- desired %f, reached %f, retrying %d ...",
                  prec->name, prec->val, prec->rbv, prec->rcnt++ );

        sprintf( msg, "MA %d", prec->rval );
        send_msg( mInfo->pasynUser, msg    );
        send_msg( mInfo->pasynUser, "Us=0" );

        prec->mip  |= MIP_RETRY;
    }
    else                 // finished backlash, close enough, or no retry allowed
    {
        prec->diff = diff;
        if ( fabs(diff) < prec->rdbd )
        {
            Debug( 0, "%s -- desired %f, reached %f",
                      prec->name, prec->val, prec->rbv             );
            prec->miss = 0;
        }
        else
        {
            Debug( 0, "%s -- desired %f, reached %f after %d retries",
                      prec->name, prec->val, prec->rbv, prec->rcnt );
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
    if ( old_diff != prec->diff ) MARK( M_DIFF );

    // check the alarms
/*  if      ( pinfo->usocket < 0                            )
        recGblSetSevr( (dbCommon *)prec, UDF_ALARM,   INVALID_ALARM );
    else if ( msta.Bits.RA_PROBLEM  ||
              msta.Bits.RA_MINUS_LS || msta.Bits.RA_PLUS_LS )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );
    else if ( msta.Bits.EA_SLIP_STALL                       )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MINOR_ALARM   );
*/
    recGblFwdLink( prec );                      // process the forward scan link

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 0 );

    prec->proc = 0;
    prec->pact = 0;

    return( status );
}

/******************************************************************************/
static long process_motor_info( imsRecord *prec, status_word sword, long count )
{
    ims_info *mInfo = (ims_info *)prec->dpvt;
    short     old_movn, old_rlls, old_rhls, old_lls, old_hls;
    double    old_drbv, old_rbv;
    long      old_rrbv;

    int       dir = (prec->dir == motorDIR_Positive) ? 1 : -1;

    old_movn   = prec->movn;
    old_rlls   = prec->rlls;
    old_rhls   = prec->rhls;
    old_lls    = prec->lls ;
    old_hls    = prec->hls ;
    old_rrbv   = prec->rrbv;
    old_drbv   = prec->drbv;
    old_rbv    = prec->rbv ;

    prec->movn = sword.Bits.MOVING;

    prec->rlls = 0;
    prec->rhls = 0;
    if ( mInfo->S1 == 3 ) prec->rlls = sword.Bits.I1;
    if ( mInfo->S2 == 3 ) prec->rlls = sword.Bits.I2;
    if ( mInfo->S3 == 3 ) prec->rlls = sword.Bits.I3;

    if ( mInfo->S1 == 2 ) prec->rhls = sword.Bits.I1;
    if ( mInfo->S2 == 2 ) prec->rhls = sword.Bits.I2;
    if ( mInfo->S3 == 2 ) prec->rhls = sword.Bits.I3;

    prec->lls  = (dir == 1) ? prec->rlls : prec->rhls;
    prec->hls  = (dir == 1) ? prec->rhls : prec->rlls;

    prec->rrbv = count;
    prec->drbv = prec->rrbv * prec->res;

    prec->rbv  = dir * prec->drbv + prec->off;

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
            Debug( 0, "%s -- move to: %f (DVAL: %f), with ACCL and VELO",
                      prec->name, prec->val, prec->dval-prec->bdst );

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
            Debug( 0, "%s -- move to: %f (DVAL: %f), with BACC and BVEL",
                      prec->name, prec->val, prec->dval            );

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
        Debug( 0, "%s -- move to: %f (DVAL: %f), with ACCL and VELO",
                  prec->name, prec->val, prec->dval );

        prec->mip  = MIP_MOVE;
        prec->rval = NINT(prec->dval / prec->res);

        VI = NINT( prec->vbas / prec->res );
        VM = NINT( prec->velo / prec->res );
        A  = NINT( (prec->velo - prec->vbas) / prec->res / prec->accl );
        sprintf( msg, "VI %d\r\n1VM %d\r\n1A %d\r\n1D=A\r\n1MA %d", prec->rval );
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
        else if ( fieldIndex == imsRecordERES ) prec->oval = prec->eres;
        else if ( fieldIndex == imsRecordSREV ) prec->oval = prec->srev;
        else if ( fieldIndex == imsRecordLLM  ) prec->oval = prec->llm ;
        else if ( fieldIndex == imsRecordHLM  ) prec->oval = prec->hlm ;
        else if ( fieldIndex == imsRecordDLLM ) prec->oval = prec->dllm;
        else if ( fieldIndex == imsRecordDHLM ) prec->oval = prec->dhlm;

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
            Debug( 1, "%s -- %s", prec->name, prec->sstr );

            status = sscanf( prec->sstr, "Status=%ld,P=%ldEOS", &sword,&count );
            if ( status == 2 )
            {
                Debug( 2, "%s -- sword =%d,p=%d", prec->name, sword, count );

                mInfo->cMutex->lock();

                mInfo->sword   = sword;
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

            if ( prec->dmov == 0 )                    // stop current move first
            {
                Debug( 0, "%s -- stop current move", prec->name );
                prec->mip  = MIP_NEW;

                send_msg( mInfo->pasynUser, "SL 0\r\n1Us=0" );

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
                Debug( 0, "%s -- resume moving",                prec->name );
                prec->mip &= ~( MIP_STOP | MIP_PAUSE );

                sprintf( msg, "MA %d", prec->rval );
                send_msg( mInfo->pasynUser, msg );

                break;
            }
            else
            {
                if ( prec->spg == motorSPG_Stop )
                {
                    Debug( 0, "%s -- stop, with deceleration",  prec->name );
                    prec->mip &= ~MIP_PAUSE;
                    prec->mip |=  MIP_STOP ;
                }
                else
                {
                    Debug( 0, "%s -- pause, with deceleration", prec->name );
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
        case imsRecordERES:
            if ( prec->eres <= 0. )
            {
                prec->eres = prec->oval;
                db_post_events( prec, &prec->eres, DBE_VAL_LOG );
                break;
            }

            goto set_res;
        case imsRecordSREV:
            if ( prec->srev <= 0. )
            {
                prec->srev = prec->oval;
                db_post_events( prec, &prec->srev, DBE_VAL_LOG );
                break;
            }
        case imsRecordUREV:
            nval = prec->urev / prec->srev;
            if ( prec->mres != nval )
            {
                prec->mres = nval;
                db_post_events( prec, &prec->mres, DBE_VAL_LOG );
            }

            set_res:
            nval = prec->res;
            if ( prec->ee == motorAble_Enable ) prec->res = prec->eres;
            else                                prec->res = prec->mres;

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
            sprintf( msg, "pr \"%s=\",%s", prec->read, prec->read );
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
static long recv_reply( asynUser *pasynUser, char *buf, int flag )
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

