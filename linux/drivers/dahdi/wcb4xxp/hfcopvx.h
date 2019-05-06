#ifndef __HFCOPVX_H_
#define __HFCOPVX_H_


#include <linux/types.h>

#define CS_1 		0x01
#define CS_2		0x02

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;

/** This is the unsigned 8-bit datatype. */
typedef unsigned char   uchar;
/** This is the unsigned 16-bit datatype. */
typedef unsigned short  uint16;
/** This is the signed 16-bit datatype. */
typedef signed short    int16;
/** This is the unsigned 32-bit datatype. */
typedef unsigned int    uint32;
/** This is the signed 32-bit datatype. */
typedef signed int      int32;

#define MAX_MODULE_SIGNAL_INPUTS 5

/** This type has two states, success and error. */
typedef enum {
   /** Operation failed. */
   OPV_ERROR   = (-1),
   /** Operation succeeded. */
   OPV_SUCCESS = 0
} OPV_return_t;

typedef enum {
   /** False. */
   OPV_FALSE = 0,
   /** True. */
   OPV_TRUE = 1
} OPV_boolean_t;

/** Module types of the firmware */
typedef enum
{
   VINDSP_MT_ALM,
   VINDSP_MT_SIG,
   VINDSP_MT_COD,
   VINDSP_MT_PCM
} VINDSP_MT;

typedef enum
{
   REMOTE_SIG_OUT = 0,
   LOCAL_SIG_OUT = 1
} SIG_OUTPUT_SIDE;

/** which signaling input is used for local (with auto suppression) and remote
    (with event playout) connections. This is fixed by firmware */
enum
{
   REMOTE_SIG_IN = 1,
   LOCAL_SIG_IN = 0
};

#define OPV_NULL         ((void *)0)

typedef enum
{
   VIN_EVENT_SUPPORT_NONE         = 0,
   VIN_EVENT_SUPPORT_BASIC        = 1,
   VIN_EVENT_SUPPORT_EXTENDED     = 2
} VIN_EVENT_SUPPORT;

typedef union
{
   unsigned short value[20];
   struct
   {
      /* cmd 1 */
      unsigned ch                            : 4;
      unsigned res                           : 4;
      unsigned cmd                           : 5;
      unsigned bc                            : 1;
      unsigned res0                          : 1;
      unsigned rw                            : 1;
      /* cmd 2 */
      unsigned length                        : 8;
      unsigned ecmd                          : 5;
      unsigned mod                           : 3;
      /* data */
      unsigned BLEN                          : 8;
      unsigned VERS                          : 8;
      unsigned NALI                          : 8;
      unsigned NPCM                          : 8;
      unsigned NCOD                          : 8;
      unsigned NSIG                          : 8;
      unsigned NEQ                           : 8;
      unsigned NAGC                          : 8;
      unsigned NWLEC                         : 8;
      unsigned NNLEC                         : 8;
      unsigned NWWLEC                        : 8;
      unsigned NNWLEC                        : 8;
      unsigned NDTMFG                        : 8;
      unsigned NUTG                          : 8;
      unsigned NCIDR                         : 8;
      unsigned NCIDS                         : 8;
      unsigned NMFTD                         : 8;
      unsigned NCPTD                         : 8;
      unsigned Res01                         : 8;
      unsigned NFAX                          : 8;
      unsigned Res02                         : 16;
      unsigned CODECS                        : 16;
      unsigned CMID                          : 8;
      unsigned CLOW                          : 8;
      unsigned PCOD                          : 8;
      unsigned CMAX                          : 8;
      unsigned Res03                         : 4;
      unsigned ES_1                          : 1;
      unsigned Res04                         : 3;
      unsigned MFTDV                         : 4;
      unsigned Res05                         : 4;
      unsigned TONES                         : 16;
      unsigned OVL                           : 8;
      unsigned FEAT                          : 8;
      unsigned ETC                           : 8;
      unsigned EPC                           : 8;


   } bit;
} FWM_CAPS;

typedef enum _OPVX_FW_RAM
{
   OPVX_DRAM = 0,
   OPVX_PRAM
} OPVX_FW_RAM;

typedef enum _OPV_TAPI_CONN_ACTION
{
   OPV_TAPI_CONN_ACTION_CREATE = 0,
   OPV_TAPI_CONN_ACTION_REMOVE = 1
} OPV_TAPI_CONN_ACTION_t;

typedef enum
{
   /** Default. It depends on the device and configures the best applicable. */
   OPV_TAPI_MAP_TYPE_DEFAULT = 0,
   /** Type is a coder packet channel. */
   OPV_TAPI_MAP_TYPE_CODER = 1,
   /** Type is a PCM channel. */
   OPV_TAPI_MAP_TYPE_PCM = 2,
   /** Type is a phone channel. */
   OPV_TAPI_MAP_TYPE_PHONE = 3,
   /** Type is a audio channel .*/
   OPV_TAPI_MAP_TYPE_AUDIO = 4,
   /** Type is a audio channel as a auxiliary input for incall anouncement. */
   OPV_TAPI_MAP_TYPE_AUDIO_AUX = 5,
   /** Type is a diagnostic channel attached to data stream 'behind' ADC0. */
   OPV_TAPI_MAP_TYPE_AUDIO_DIAG0_IN=6,
   /** Type is a diagnostic channel attached to data stream 'before' DAC0. */
   OPV_TAPI_MAP_TYPE_AUDIO_DIAG0_OUT=7,
   /** Type is a diagnostic channel attached to data stream 'behind' ADC1. */
   OPV_TAPI_MAP_TYPE_AUDIO_DIAG1_IN=8,
   /** Type is a diagnostic channel attached to data stream 'before' DAC1. */
   OPV_TAPI_MAP_TYPE_AUDIO_DIAG1_OUT=9,
   /** Type is an audio loop that assigns audio module's signals
       Audio_LOOP_I1 and Audio_LOOP_O3 to an additional data channel. */
   OPV_TAPI_MAP_TYPE_AUDIO_LOOP0=10,
   /** Type is an audio loop that assigns audio module's signals
       Audio_LOOP_I2 and Audio_LOOP_O1 to an additional data channel. */
   OPV_TAPI_MAP_TYPE_AUDIO_LOOP1=11,
   /** Type is a DECT channel. */
   OPV_TAPI_MAP_TYPE_DECT = 12
} OPV_TAPI_MAP_TYPE_t;

typedef struct
{
   /** Channel number to which this channel should be mapped.
   Channels numbers start from 0. */
   unsigned char                 nDstCh;
   /** Type of the destination channel.

   - 0: OPV_TAPI_MAP_TYPE_DEFAULT, Default selected (phone channel)
   - 1: OPV_TAPI_MAP_TYPE_CODER, not supported
   - 2: OPV_TAPI_MAP_TYPE_PCM, type is PCM
   - 3: OPV_TAPI_MAP_TYPE_PHONE, type is phone channel
   - 4: OPV_TAPI_MAP_TYPE_AUDIO, type is audio channel
   - 5: OPV_TAPI_MAP_TYPE_AUDIO_AUX, type is audio channel auxiliary input
   - 6: OPV_TAPI_MAP_TYPE_DECT, type is DECT */
   OPV_TAPI_MAP_TYPE_t           nChType;
} OPV_TAPI_MAP_PCM_t;

struct _OPVX_CAPABILITIES
{
   /* Number of UTG resources per channel (== SIG module), either 1 or 2 */
   unsigned int  nUtgPerCh : 8;
   /** Support for extended jitter buffer statistics 0=no 1=yes */
   unsigned int  bExtendedJBsupported : 1;
   /** Support for two UTDs per SIG module */
   unsigned int  bUtd2supported : 1;
   /** RTP Protocol support: 0 = no / 1 = yes */
   unsigned int  bProtocolRTP : 1;
   /** AAL Protocol support: 0 = no / 1 = yes */
   unsigned int  bProtocolAAL : 1;
   /** Event support level */
   VIN_EVENT_SUPPORT nEventSupport;

   /* Fields below are a direct copy from the firmware capability message */
   /** Number of PCM Channels */
   unsigned int  nPCM : 8;
   /** Number of Analog Line Channels */
   unsigned int  nALI : 8;
   /** Number of Signaling Channels */
   unsigned int  nSIG : 8;
   /** Number of Coder Channels */
   unsigned int  nCOD : 8;
   /** Number of AGCs */
   unsigned int  nAGC : 8;
   /** Number of Equalizers */
   unsigned int  nEQ : 8;
   /** Number of Near-End LECs */
   unsigned int  nNLEC : 8;
   /** Number of Combined Near-End/Far-End LECs */
   unsigned int  nWLEC : 8;
   /** Number of Near-End Wideband LECs */
   unsigned int  nNWLEC : 8;
   /** Number of Combined Near-End/Far-End Wideband LECs */
   unsigned int  nWWLEC : 8;
   /** Number of Universal Tone Generators */
   unsigned int  nUTG : 8;
   /** Number of DTMF Generators */
   unsigned int  nDTMFG : 8;
   /** Number of Caller ID Senders */
   unsigned int  nCIDS : 8;
   /** Number of Caller ID Receivers */
   unsigned int  nCIDR : 8;
   /** Number of Call Progress Tone Detectors */
   unsigned int  nCPTD : 8;
   /** Number of Modem and Fax Tone Discriminators (MFTDs) */
   unsigned int  nMFTD : 8;
   /** Number of FAX Channels with FAX Relay (T.38) Support */
   unsigned int  nFAX : 8;
   /** Codecs */
   unsigned int  CODECS : 16;
   /** Maximum Number of Low Complexity Coders for the Coder Channel */
   unsigned int  CLOW : 8;
   /** Maximum Number of Mid Complexity Coders for the Coder Channel */
   unsigned int  CMID : 8;
   /** Maximum Number of High Complexity Coders for the Coder Channel*/
   unsigned int  CMAX : 8;
   /** PCM Channel Coders */
   unsigned int  PCOD : 8;
   /** MFTD Version */
   unsigned int  MFTDV : 4;
   /** Tone Detection Capabilities */
   unsigned int  TONES : 16;
   /** Features */
   unsigned int  FEAT : 8;
   /** Overlays */
   unsigned int  OVL : 8;
   /** Event Playout Capabilities */
   unsigned int  EPC : 8;
   /** Event Transmission Capabilities */
   unsigned int  ETC : 8;
   /** Echo Suppressor supported */
   unsigned int  ES_1 : 1;
};
typedef struct _OPVX_CAPABILITIES OPVX_CAPABILITIES_t;

typedef struct _OPVX_CHANNEL OPVX_CHANNEL;
typedef struct OPVX_PCMCH 	OPVX_PCMCH_t;
typedef struct __OPVXEC_HW OPVXEC_HW;
typedef struct semaphore*  OPVOS_mutex_t;

#define PCM_HIGHWAY	1
#define PCM_MAX_TS	127
#define PCM_TS_ARRAY	((PCM_MAX_TS + 3) / 32)

typedef struct _MODULE_SIGNAL
{
   /** Signal array index value from ECMD_IX_SIG connected to this input. */
   uchar i;
   /** Signal array index value used when this input is muted. */
   uchar i_mute;
   /** If not 0 this input is in muted state and uses the index from i_mute
       instead of i.  */
   uchar mute;
   /* Parent module. In case of muting, it must be set to modified */
   struct VINDSP_MODULE *pParent;
   /** Which output this input is connected to. */
   struct VINDSP_MODULE *pOut;
   /** Points to the next input to which the outSig is connected to.
       This input signal is within a linked list of input signals that are
       all connected to one output signal. */
   struct _MODULE_SIGNAL *pNext;
} VINDSP_MODULE_SIGNAL; 

typedef struct VINDSP_MODULE
{
   /** array of structures for each input of this module */
   VINDSP_MODULE_SIGNAL in[MAX_MODULE_SIGNAL_INPUTS];
   /** flag that indicates changes to the inputs (0 means no change) */
   uchar modified;
   /** the signal array index of this module's standard output */
   uchar nSignal;
   /** pointer to the first input signal which connects to the output */
   VINDSP_MODULE_SIGNAL* pInputs;
   /** flag that indicates that the standard output is muted */
   uchar nMute;
   /** the signal array index of this module's second output
       (only used for signaling modules) */
   uchar nSignal2;
   /** pointer to the first input signal which connects to the second output
       (only used for signaling modules) */
   VINDSP_MODULE_SIGNAL* pInputs2;
   /** flag that indicates that the second output is muted
       (only used for signaling modules) */
   uchar nMute2;
   /** defines the module type, value out of \ref VINDSP_MT */
   uchar nModType;
   /** channel that this module is assigned to. */
   OPVX_CHANNEL *pCh;
}VINDSP_MODULE_t;

typedef struct OPVX_CON
{
   struct VINDSP_MODULE  modAlm;
   struct VINDSP_MODULE  modCod;
   struct VINDSP_MODULE  modSig;
   struct VINDSP_MODULE  modPcm;
}OPVX_CON_t;

struct _OPVX_CHANNEL
{
   /* Actual  OPVX channel, starting with 1, a channel number 0 indicates
      the control device structure OPVX_DEVICE */
   uchar              nChannel;
   /* tracking in use of the device */
   uint16             nInUse;

   /* ptr to actual device */
    OPVXEC_HW          *pParent;
   /* overall channel protection ( read/write/ioctl level)
      PS: Avoid nested locking of this mutex. It can lead to a deadlock */
   OPVOS_mutex_t            chAcc;

   /* PCM Channel management structure*/
   OPVX_PCMCH_t          *pPCM;
   /* Connection module structure */
   OPVX_CON_t            *pCON;

   /* PCM channel mapping */
   uchar             nPcmCh;
   /* PCM channel maximum support Bit Mode (8-Bit or 16-Bit) */
   uchar              nPcmMaxResolution;

   OPV_boolean_t            bVoiceConnect;
   uchar                    nEvtPT;
};

struct __OPVXEC_HW{
	 /* EDSP feature and version registers */ 
	 uint16  	nEdspVers[3];
	 OPVX_CAPABILITIES_t   caps;
	 OPVX_CHANNEL          pChannel[2][8];	//0907
	 uint16             availLecRes[2];			//0907
	 OPVOS_mutex_t            mbxAcc;
   		/* OPVX share variables concurent access protection mutex.
      		PS: Avoid nested locking of this mutex. It can lead to a deadlock */
   	OPVOS_mutex_t            memberAcc;
   	uint32             PcmRxTs[PCM_HIGHWAY][PCM_TS_ARRAY];
   	/* value PcmTxTs need protection */
   	uint32           PcmTxTs[PCM_HIGHWAY][PCM_TS_ARRAY];
};

typedef union
{
//   uint16 value[CMD_HEADER_CNT + CMD_PCM_CH_LEN];	2+5
   uint16 value[7];
   struct {
      /* cmd 1 */
      unsigned ch                            : 4;
      unsigned res                           : 4;
      unsigned cmd                           : 5;
      unsigned bc                            : 1;
      unsigned res0                          : 1;
      unsigned rw                            : 1;
      /* cmd 2 */
      unsigned length                        : 8;
      unsigned ecmd                          : 5;
      unsigned mod                           : 3;
      /* data */
      unsigned i1                            : 6;
      unsigned bp                            : 1;
      unsigned hp                            : 1;
      unsigned codnr                         : 4;
      unsigned cod                           : 3;
      unsigned en                            : 1;

      unsigned rts                           : 7;
      unsigned r_hw                          : 1;
      unsigned xts                           : 7;
      unsigned x_hw                          : 1;

      unsigned gain_2                        : 8;
      unsigned gain_1                        : 8;

      unsigned i3                            : 6;
      unsigned res2                          : 2;
      unsigned i2                            : 6;
      unsigned res1                          : 2;

      unsigned i5                            : 6;
      unsigned res4                          : 2;
      unsigned i4                            : 6;
      unsigned res3                          : 2;

   } bit;
} FWM_PCM_CH;

struct OPVX_PCMCH
{
   FWM_PCM_CH        fw_pcm_ch;
   uint16      pcm_nelec;
   /* The LEC window length specification is channel specific */
   uchar       lec_window_coefs[3];
};

typedef enum
{
   /** LEC and Echo Suppressor turned off. */
   OPV_TAPI_WLEC_TYPE_OFF = 0x00,
   /** LEC using fixed window. No Echo Suppressor. */
   OPV_TAPI_WLEC_TYPE_NE  = 0x01,
   /** LEC using fixed and moving window. No Echo Suppressor. */
   OPV_TAPI_WLEC_TYPE_NFE = 0x02,
   /** LEC using fixed window + Echo Suppressor. */
   OPV_TAPI_WLEC_TYPE_NE_ES  = 0x03,
   /** LEC using fixed and moving window + Echo Suppressor. */
   OPV_TAPI_WLEC_TYPE_NFE_ES = 0x04,
   /** Echo Suppressor */
   OPV_TAPI_WLEC_TYPE_ES     = 0x05
} OPV_TAPI_WLEC_TYPE_t;

typedef enum
{
   /** LEC window size 4 ms .*/
   OPV_TAPI_WLEN_WSIZE_4 = 4,
   /** LEC window size 6 ms. */
   OPV_TAPI_WLEN_WSIZE_6 = 6,
   /** LEC window size 8 ms. */
   OPV_TAPI_WLEN_WSIZE_8 = 8,
   /** LEC window size 16 ms. */
   OPV_TAPI_WLEN_WSIZE_16 = 16
} OPV_TAPI_WLEC_WIN_SIZE_t;

typedef enum
{
   /** Reserved. Default NLP on.*/
   OPV_TAPI_LEC_NLP_DEFAULT = 0,
   /** Switches on NLP. */
   OPV_TAPI_LEC_NLP_ON = 1,
   /** Switches off NLP. */
   OPV_TAPI_LEC_NLP_OFF = 2
} OPV_TAPI_LEC_NLP_t;


/** Defines the coding for the PCM channel. */
typedef enum
{
   /** G.711 A-Law 8 bit, narrowband.
       This resolution requires 1 PCM timeslot. */
   OPV_TAPI_PCM_RES_NB_ALAW_8BIT = 0,
   /** G.711 u-Law 8 bit, narrowband.
       This resolution requires 1 PCM timeslot. */
   OPV_TAPI_PCM_RES_NB_ULAW_8BIT = 1,
   /** Linear 16 bit, narrowband.
       This resolution requires 2 consecutive PCM timeslots. */
   OPV_TAPI_PCM_RES_NB_LINEAR_16BIT = 2,
   /** G.711 A-Law 8 bit, wideband.
       This resolution requires 2 consecutive PCM timeslots. */
   OPV_TAPI_PCM_RES_WB_ALAW_8BIT = 3,
   /** G.711 u-Law 8 bit, wideband.
       This resolution requires 2 consecutive PCM timeslots. */
   OPV_TAPI_PCM_RES_WB_ULAW_8BIT = 4,
   /** Linear 16 bit, wideband.
       This resolution requires 4 consecutive PCM timeslots. */
   OPV_TAPI_PCM_RES_WB_LINEAR_16BIT = 5
} OPV_TAPI_PCM_RES_t;

/* map the old names to the NB names */
#define OPV_TAPI_PCM_RES_ALAW_8BIT   OPV_TAPI_PCM_RES_NB_ALAW_8BIT
#define OPV_TAPI_PCM_RES_ULAW_8BIT   OPV_TAPI_PCM_RES_NB_ULAW_8BIT
#define OPV_TAPI_PCM_RES_LINEAR_16BIT   OPV_TAPI_PCM_RES_NB_LINEAR_16BIT

/** WLEC NLP Settings. */
typedef enum
{
   /** Reserved. Default NLP on. */
   OPV_TAPI_WLEC_NLP_DEFAULT = OPV_TAPI_LEC_NLP_DEFAULT,
   /** Switches on NLP. */
   OPV_TAPI_WLEC_NLP_ON = OPV_TAPI_LEC_NLP_ON,
   /** Switches off NLP. */
   OPV_TAPI_WLEC_NLP_OFF = OPV_TAPI_LEC_NLP_OFF
} OPV_TAPI_WLEC_NLP_t;

typedef struct
{
   /* LEC operating mode */
   OPV_TAPI_WLEC_TYPE_t nOpMode;
   /* Non Linear Processing on or off */
   char bNlp;
   /* Gain for input or LEC off */
   char nGainIn;
   /* Gain for ouput or LEC off */
   char nGainOut;
   /** LEC tail length - unused only a storage needed for get function */
   char nLen;
   /** Size of the near-end window in narrowband sampling mode. */
   OPV_TAPI_WLEC_WIN_SIZE_t   nNBNEwindow;
   /** Size of the far-end window in narrowband sampling mode.
       Note: this is used only if nOpMode is set to OPV_TAPI_LEC_TYPE_NFE */
   OPV_TAPI_WLEC_WIN_SIZE_t   nNBFEwindow;
   /** Size of the near-end window in wideband sampling mode. */
   OPV_TAPI_WLEC_WIN_SIZE_t   nWBNEwindow;
   
}TAPI_LEC_DATA_t;

typedef struct _OPV_TAPI_PCM_CFG
{
	/** PCM timeslot for the receive direction */
	unsigned long             nTimeslotRX;
	/** PCM timeslot for the transmit direction */
	unsigned long             nTimeslotTX;
   	/** Defines the PCM highway number which is connected to the channel */
   	unsigned long             nHighway;
   	/** Defines the PCM interface coding

   	- 0: OPV_TAPI_PCM_RES_ALAW_8BIT, 8 bit A law
   	- 1: OPV_TAPI_PCM_RES_ULAW_8BIT, 8 bit u Law
   	- 2: OPV_TAPI_PCM_RES_LINEAR_16BIT, 16 bit linear */
   	unsigned long             nResolution;
   	/** Defines the PCM sample rate in kHz. */
//   	unsigned long             nRate;
} OPV_TAPI_PCM_CFG_t;

typedef struct
{
   /* configuration data for pcm services */
   OPV_TAPI_PCM_CFG_t   PCMConfig;
   /* save activation status */
   OPV_boolean_t        bTimeSlotActive;
}TAPI_PCM_DATA_t;

typedef struct
{
   DWORD 	nEdspFlags;  
   BYTE	 	*pPRAMfw ;	 /** valid Firmware PRAM byte pointer */
   DWORD	pram_size;	/** size of PRAM firmware in bytes */
   BYTE 		*pDRAMfw ;	/** valid Firmware DRAM byte pointer */
   DWORD 	dram_size;	/** size of DRAM firmware in bytes */
   WORD		nPramCRC;	 /** return values of PRAM CRC after firmware download */
   WORD		nDramCRC;	/** return values of DRAM CRC after firmware download */
} OPVX_EDSP_FWDWLD;

/** allocate memory */
//#define OPVOS_MALLOC(size) (in_interrupt()?kmalloc(size, GFP_ATOMIC):kmalloc(size, GFP_KERNEL))
#define OPVOS_MALLOC(size) kmalloc(size, GFP_KERNEL)
/** free memory */
#define OPVOS_FREE(ptr)       if (ptr != NULL) {kfree((void*)ptr); ptr = NULL;}

extern void OPVOS_Udelay(unsigned long usecs);
extern void OPVOS_Mdelay(unsigned long msecs);
extern void OPVOS_Ndelay(unsigned long nsecs);
//for semaphore
extern void OPVOS_MutexInit(OPVOS_mutex_t *mutex);
extern void OPVOS_MutexLock(OPVOS_mutex_t *mutex);
extern void OPVOS_MutexLockInterruptible(OPVOS_mutex_t *mutex);
extern void OPVOS_MutexUnlock(OPVOS_mutex_t *mutex);
extern void OPVOS_MutexDelete(OPVOS_mutex_t *mutex);

extern int OPVXXP_INIT(u_int pci_iobase,OPVXEC_HW *ec,BYTE cs);
extern void OPVXXP_EXIT(OPVXEC_HW *ec);
extern int opvx_set_ec(u_int pci_iobase, OPVXEC_HW *ec, int cs, int ch, int onoff);
#endif
