struct ims_info
{
    epicsMutex *cMutex;
    asynUser   *pasynUser;
    int         S1;                                                 // switch S1
    int         S2;
    int         S3;
    int         init;
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
        unsigned int M_BUTC     :1;
    } Bits;
} changed_fields;

#define MARK(FIELD)   { changed_fields temp; temp.All = prec->cmap;            \
                        temp.Bits.FIELD = 1; prec->cmap = temp.All; }

#define MARKED(FIELD) ( cmap.Bits.FIELD )

#define UNMARK_ALL      prec->cmap = 0

/*** All db_post_events() calls set both VALUE and LOG bits ***/
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)             // Nearest integer


void Debug( int level, const char *fmt, ... );

