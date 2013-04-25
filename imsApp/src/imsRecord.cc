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


#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)             // Nearest integer

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
    int         sword;
    long        count;
    bool        newData;
};

typedef union
{
    unsigned long All;
    struct
    {
        unsigned int MOVING : 1;
        unsigned int EE     : 1; // encoder enable
        unsigned int SM     : 1; // stall mode
        unsigned int I1     : 1; // I1
        unsigned int I2     : 1; // I2
        unsigned int I3     : 1; // I3
        unsigned int I4     : 1; // I4
        unsigned int ERR    : 7; // error number
        unsigned int ST     : 1; // stall detected
        unsigned int PU     : 1; // power-cycled
        unsigned int NA     :16; // not used
    } Bits;
} status_word;

#define MIP_DONE     0x0000    // No motion is in progress
#define MIP_HOMF     0x0008    // A home-forward command is in progress
#define MIP_HOMR     0x0010    // A home-reverse command is in progress
#define MIP_HOME     (MIP_HOMF | MIP_HOMR)
#define MIP_MOVE     0x0020    // A move not resulting from Jog* or Hom*
#define MIP_RETRY    0x0040    // A retry is in progress
#define MIP_NEWM     0x0080    // Stop current move for a new move
#define MIP_BL       0x0100    // Done moving, now take out backlash
#define MIP_STOP     0x0200    // We're trying to stop.  If a home command
                               // is issued when the motor is moving, we
                               // stop the motor first
#define MIP_PAUSE    0x0400    // Move is paused

#define MAX_MSG_SIZE 64
#define FLUSH        -1

#define OK            0

static long connect_motor     ( imsRecord *precord                            );
static long init_motor        ( imsRecord *precord                            );
static long process_motor_info( imsRecord *precord, status_word sword,
                                                    long count                );
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

    prec->dmov = 1;
    prec->mip  = MIP_DONE;

    // send a status update in 1 second
    send_msg( pasynUser, "R2=17500" );

    finish_up:
    mInfo->cMutex->unlock();

    return( 0 );
}


/******************************************************************************/
static long process( dbCommon *precord )
{
    imsRecord   *prec = (imsRecord *)precord;
    ims_info    *mInfo = (ims_info *)prec->dpvt;
    status_word  sword;

    char         msg[MAX_MSG_SIZE];
    long         count, status = OK;

    if ( prec->pact ) return( OK );

    prec->pact = 1;
    Debug( 1, "%s -- process", prec->name );

    mInfo->cMutex->lock();
    if ( ! mInfo->newData )
    {
        mInfo->cMutex->unlock();
        goto finished;
    }

    sword.All = mInfo->sword;
    count     = mInfo->count;

    mInfo->newData = 0;
    mInfo->cMutex->unlock();

    process_motor_info( prec, sword, count );

    if ( prec->movn )                                            // still moving
    {
        // stalled or not?

        goto finished;
    }

    if ( sword.Bits.ST && (! sword.Bits.SM) )                          //stalled
    {
        // msta.Bits.RA_PROBLEM = 1;

        prec->dmov = 1;
        prec->mip  = MIP_DONE;
    }

    if ( prec->mip == MIP_DONE ) goto finish_up;

    prec->diff = prec->rbv - prec->val;
    if ( (prec->mip == MIP_HOME) ||
         (prec->mip  & MIP_STOP) || (prec->mip  & MIP_PAUSE) )
    {                                             // are we homing or stopping ?
        if ( prec->mip == MIP_HOME )                    // set the at-home bit ?
        {
        }

        prec->dmov = 1;
        prec->diff = 0;
        if ( (prec->mip == MIP_HOME) || (prec->mip  & MIP_STOP) )
        {
            prec->mip  = MIP_DONE;
            prec->val  = prec->rbv;
            prec->dval = prec->drbv;
        }
    }
    else if ( prec->mip == MIP_NEWM )
    {
        Debug( 0, "%s -- move to: %f (DVAL: %f)", prec->name, prec->val,
                                                  prec->dval );

        sprintf( msg, "MA %d", prec->rval );
        send_msg( mInfo->pasynUser, msg );

        prec->mip  = MIP_MOVE;
    }
    else if ( (prec->bdst != 0.) && !(prec->mip & MIP_BL ) )      // do backlash
    {
        // send the move command
        prec->mip |= MIP_BL;

        Debug( 0, "%s -- take out the backlash ...", prec->name );
    }
    else if ( (prec->bdst == 0.) && (fabs(prec->diff) >= prec->rdbd) &&
              (prec->rtry >  0 ) && (prec->rcnt       <  prec->rtry)    )//retry
    {
        prec->mip |= MIP_RETRY;
        prec->rcnt++;

        Debug( 0, "%s -- desired %f, reached %f, retrying %d ...",
                  prec->name, prec->val, prec->rbv, prec->rcnt );
    }
    else                 // finished backlash, close enough, or no retry allowed
    {
        prec->dmov = 1;
        prec->mip  = MIP_DONE;

        if ( fabs(prec->diff) < prec->rdbd )
            Debug( 0, "%s -- desired %f, reached %f",
                      prec->name, prec->val, prec->rbv             );
        else
        {
            Debug( 0, "%s -- desired %f, reached %f after %d retries",
                      prec->name, prec->val, prec->rbv, prec->rcnt );
            prec->miss = 1;
        }
    }

    finish_up:

    finished:
    prec->pact = 0;

    return( status );
}


/******************************************************************************/
static long process_motor_info( imsRecord *prec, status_word sword, long count )
{
    ims_info *mInfo = (ims_info *)prec->dpvt;
    int       dir = (prec->dir == motorDIR_Positive) ? 1 : -1;

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

    prec->diff = prec->dval - prec->drbv;
//  prec->rdif = NINT(prec->diff / prec->res);

    Debug( 3, "%s -- rlls %d, rhls %d %d %d %d", prec->name, prec->rlls, prec->rhls, sword.Bits.I1, sword.Bits.I2, sword.Bits.I3 );
    return( 0 );
}


/******************************************************************************/
static long special( dbAddr *pDbAddr, int after )
{
    imsRecord *prec = (imsRecord *) pDbAddr->precord;
    ims_info  *mInfo = (ims_info *)prec->dpvt;
    long       sword, count;
    char       msg[MAX_MSG_SIZE];
    double     nval;

    int        fieldIndex = dbGetFieldIndex( pDbAddr ), status = OK;

    if ( after == 0 )
    {
        if      ( fieldIndex == imsRecordVAL  ) prec->oval = prec->val ;
        else if ( fieldIndex == imsRecordDVAL ) prec->oval = prec->dval;
        else if ( fieldIndex == imsRecordERES ) prec->oval = prec->eres;
        else if ( fieldIndex == imsRecordSREV ) prec->oval = prec->srev;
        else if ( fieldIndex == imsRecordDIR  ) prec->oval = prec->dir ;
        else if ( fieldIndex == imsRecordOFF  ) prec->oval = prec->off ;

        return( OK );
    }

    switch( fieldIndex )
    {
        case imsRecordSSTR:
            Debug( 1, "%s -- %s", prec->name, prec->sstr );

            status = sscanf( prec->sstr, "Status=%ld,P=%ldEOS", &sword, &count);
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
                prec->val  = prec->oval;
                prec->lvio = 1;                     // set limit violation alarm

                // MARK( M_ERR  );
                // MARK( M_VAL  );
                // MARK( M_LVIO );

                break;
            }

            do_move1:
            if ( prec->set == motorSET_Use )            // do it only when "Use"
            {
                prec->dval = (prec->val - prec->off) * (1. - 2.*prec->dir);
                // MARK( M_DVAL );
            }
            goto do_move2;
        case imsRecordDVAL:
            if ( (prec->dval < prec->dllm) || (prec->dval > prec->dhlm) )
            {                                    // violated the hardware limits
                prec->dval = prec->oval;
                prec->lvio = 1;                     // set limit violation alarm

                // MARK( M_ERR  );
                // MARK( M_DVAL );
                // MARK( M_LVIO );

                break;
            }

            prec->val = prec->dval * (1. - 2.*prec->dir) + prec->off;
            // MARK( M_VAL  );

            do_move2:
            prec->rval = NINT(prec->dval / prec->res);
            if ( (prec->set != motorSET_Use) ||               // do it only when
                 (prec->spg != motorSPG_Go )    ) break;      // "Set" and "Go"

            prec->lvio = 0;
            prec->movn = 1;
            prec->rcnt = 0;

            if ( prec->dmov == 0 )                    // stop current move first
            {
                Debug( 0, "%s -- stop current move", prec->name );

                send_msg( mInfo->pasynUser, "SL 0" );
                prec->mip  = MIP_NEWM;

                break;
            }

            Debug( 0, "%s -- move to: %f (DVAL: %f)", prec->name, prec->val,
                                                      prec->dval );

            sprintf( msg, "MA %d", prec->rval );
            send_msg( mInfo->pasynUser, msg );

            prec->mip  = MIP_MOVE;
            prec->dmov = 0;

            // MARK( M_LVIO );
            // MARK( M_MIP  );
            // MARK( M_MOVN );
            // MARK( M_DMOV );
            // MARK( M_RCNT );

            epicsThreadSleep( 0.1 );

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

                // MARK( M_ERR  );
                // MARK( M_LVIO );

                break;
            }

            prec->val = nval;
            // MARK( M_VAL  );
            goto do_move1;
        case imsRecordSPG :
            break;
        case imsRecordERES:
            if ( prec->eres <= 0. )
            {
                prec->eres = prec->oval;
                db_post_events( prec, &prec->eres, DBE_VAL_LOG );
                break;  // dhz show warning
            }

            goto set_res;
        case imsRecordSREV:
            if ( prec->srev <= 0. )
            {
                prec->srev = prec->oval;
                db_post_events( prec, &prec->srev, DBE_VAL_LOG );
                break;  // dhz show warning
            }
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
            {
                prec->drbv = prec->rrbv * prec->res;
                prec->rbv  = (1. - 2.*prec->dir) * prec->drbv + prec->off;

                db_post_events( prec, &prec->res,  DBE_VAL_LOG );
                // MARK( M_DRBV );
                // MARK( M_RBV  );
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
                // MARK( M_LLS  );
                // MARK( M_HLS  );
            }

            goto change_dir_off;
        case ( imsRecordOFF  ):
            prec->llm += prec->off - prec->oval;
            prec->hlm += prec->off - prec->oval;

            change_dir_off:
            prec->val  = prec->dval * (1. - 2.*prec->dir) + prec->off;
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

