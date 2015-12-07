struct ims_info
{
    struct imsRecord *precord;

    epicsEvent       *pEvent;
    epicsEvent       *sEvent;
    epicsMutex       *cMutex;
    asynUser         *pasynUser;
    bool              initialized;

    long              csr;
    long              count;
    int               newData;

    epicsMutex       *lMutex;
    int               mLength;
    char             *sAddr;
    int               cIndex;
    bool              newMsg;
};

typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int MOVING : 1;
        unsigned int EE     : 1; // encoder enable
        unsigned int SM     : 1; // stall mode
        unsigned int I1     : 1; // I1
        unsigned int I2     : 1; // I2
        unsigned int I4     : 1; // I4
        unsigned int ERR    : 7; // error number
        unsigned int ST     : 1; // stall detected
        unsigned int TE     : 1; // trip enable
        unsigned int PU     : 1; // power-cycled
        unsigned int NE     : 1; // numeric enable
        unsigned int BY0    : 1; // MCode not running
        unsigned int NA     :14; // not used
    } Bits;
} status_word;

typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int RA_DIRECTION   :1; // (last) 0=Negative, 1=Positive
        unsigned int RA_DONE        :1; // a motion is complete
        unsigned int RA_PLUS_LS     :1; // plus limit switch has been hit
        unsigned int RA_HOME        :1; // home signal is on
        unsigned int RA_SM          :1; // continue on stall detect
        unsigned int EA_POSITION    :1; // position maintenence enabled
        unsigned int EA_SLIP_STALL  :1; // slip/stall detected
        unsigned int EA_HOME        :1; // encoder home signal on
        unsigned int RA_EE          :1; // encoder enable
        unsigned int RA_PROBLEM     :1; // driver stopped polling
        unsigned int RA_MOVING      :1; // non-zero velocity present
        unsigned int GAIN_SUPPORT   :1; // support closed-loop position control
        unsigned int RA_COMM_ERR    :1; // controller communication error
        unsigned int RA_MINUS_LS    :1; // minus limit switch has been hit
        unsigned int RA_HOMED       :1; // axis has been homed
        unsigned int RA_ERR         :7; // error number
        unsigned int RA_STALL       :1; // stall detected
        unsigned int RA_TE          :1; // trip enable
        unsigned int RA_POWERUP     :1; // power-cycled
        unsigned int RA_NE          :1; // numeric enable
        unsigned int RA_BY0         :1; // MCode not running (BY = 0)
        unsigned int NA             :4; // un-used bits
        unsigned int NOT_INIT       :1; // initialization not finished
    } Bits;
} motor_status;

/* Bit map of changed fields */
typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int M_VAL      :1;
        unsigned int M_DVAL     :1;
        unsigned int M_RVAL     :1;
        unsigned int M_RRBV     :1;
        unsigned int M_DRBV     :1;
        unsigned int M_RBV      :1;
        unsigned int M_DIFF     :1;
        unsigned int M_MIP      :1;
        unsigned int M_MOVN     :1;
        unsigned int M_DMOV     :1;
        unsigned int M_RCNT     :1;
        unsigned int M_MISS     :1;
        unsigned int M_RLLS     :1;
        unsigned int M_RHLS     :1;
        unsigned int M_LLS      :1;
        unsigned int M_HLS      :1;
        unsigned int M_LVIO     :1;
        unsigned int M_MSTA     :1;
        unsigned int M_MSTR     :1;
    } Bits;
} changed_fields;

#define   MARK(FIELD) { changed_fields temp; temp.All = prec->cmap;            \
                        temp.Bits.FIELD = 1; prec->cmap = temp.All; }

#define MARKED(FIELD) ( cmap.Bits.FIELD )

#define UNMARK_ALL      prec->cmap = 0

/*** All db_post_events() calls set both VALUE and LOG bits ***/
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)             // Nearest integer


