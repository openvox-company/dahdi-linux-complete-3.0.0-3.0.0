#ifndef _X2FXO200M_H
#define _X2FXO200M_H


#include <dahdi/kernel.h>
#include "x200_hal.h"

#define BIT_RW      7                   //BIT 7=0/1代表读/写
#define BIT_CTRL    6                   //BIT 6代表是读写控制器内部寄存器还是外部寄存器
#define BIT_BURST   5       // burst read or write

#define FLAG_R          (0<<BIT_RW)
#define FLAG_W          (1<<BIT_RW)
#define FLAG_CTRL_REG   (1<<BIT_CTRL)
#define FLAG_BUS        (0<<BIT_CTRL)
#define FLAG_BURST      (1<<BIT_BURST)


#define X2FXO200M_SPI_DELAY   50      /* need delay 15ns between spi operate */
#define MAX_NUM_CHANS_PER_CARD	4
#define CHIPS_PER_CARD	2
#define CHANS_PER_MODULE 2

#define NUM_FXO_REGS 60
#define DEFAULT_RING_DEBOUNCE		64		/* Ringer Debounce (64 ms) */
#define DEFAULT_DAA_CHECK_INTERVAL	4		/* call voicedaa_check_hook every 4 ms */

/* For ring on detect, if in DEFAULT_RING_DEBOUNCE period,
   ring signal detected >= DEFAULT_RINGON_COUNT, status chage to RINGON. */
#define DEFAULT_RINGON_COUNT		32

/* For ring off detect, if in DEFAULT_RING_DEBOUNCE period,
   ring signal detected <= DEFAULT_RINGOFF_COUNT, status chage to RINGOFF. */
#define DEFAULT_RINGOFF_COUNT		4		/* MiaoLin, count for ring off detect*/

#define DEFAULT_POLARITY_DEBOUNCE 	64		/* Polarity debounce (64 ms) */

/* the constants below control the 'debounce' periods enforced by the
   check_hook routines; these routines are called once every 4 interrupts
   (the interrupt cycles around the four modules), so the periods are
   specified in _4 millisecond_ increments
*/
#define DEFAULT_BATT_DEBOUNCE	4		/* Battery debounce (64 ms) */
#define POLARITY_DEBOUNCE 	64		/* Polarity debounce (64 ms) */
#define DEFAULT_BATT_THRESH	3		/* Anything under this is "no battery" */

//Si3050 sillicon id reg
#define SI3050_LSID	11


enum battery_state {
	BATTERY_UNKNOWN = 0,
	BATTERY_PRESENT,
	BATTERY_LOST,
};


#define FRAME_SHORT_SIZE        (DAHDI_CHUNKSIZE * 20)
#define SAMPLE_PER_SEC		8000

struct x200fxo {
	struct x200_dev *dev;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	struct dahdi_device *ddev;
#endif
	
    struct _fxo {
        int wasringing;
        int lastrdtx;
        int fastringoffhooktimer;	/* timer to send ringoffhook event faster */
        int ringoffhooksent;		/* ringoffhook already sent for this ring? */
        int ringdebounce;
        int offhook;
        unsigned int battdebounce;
        unsigned int battalarm;
        enum battery_state battery;
        int lastpol;
        int polarity;
        int polaritydebounce;
        int callout;                  /*if not 0, fxo call fxs */
        int polaritycountwhenoffhook; /* polarity counter when offhook, use for two-way charge */

			unsigned char reg0shadow;
			unsigned char reg1shadow;
	} fxoconfig[CHANS_PER_MODULE];
	int usecount;

	// dahdi API
	struct dahdi_span span;
	struct dahdi_chan _chans[CHANS_PER_MODULE];
	struct dahdi_chan *chans[CHANS_PER_MODULE];
	struct dahdi_echocan_state *ec[2];                 /* Echcan state for each channel */
	unsigned int index;

	int dead;
	int exist;
};

/* busydetect.c */
void parser_busy_silent_process(void* dev_id, int is_write);

int init_busydetect(void* dev_id, const char *opermode);
void destroy_busydetect(void* dev_id);

#endif
