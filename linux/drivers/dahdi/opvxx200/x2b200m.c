/*
 * OpenVox B100M/B200M module for x200 telephony card driver.
 *
 * Written by Miao Lin<lin.miao@openvox.cn>
 *
 * Copyright (C) 2011, OpenVox Communication Co Ltd..
 *
 * All rights reserved.
 */

/*
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <dahdi/kernel.h>


#include "x2b200m.h"
#include "x200_hal.h"
#include "ec3000.h"

#define DEBUG_GENERAL   (1 << 0)    /* general debug messages */
#define DEBUG_DTMF      (1 << 1)    /* emit DTMF detector messages */
#define DEBUG_REGS      (1 << 2)    /* emit register read/write, but only if the kernel's DEBUG is defined */
#define DEBUG_FOPS      (1 << 3)    /* emit file operation messages */
#define DEBUG_ECHOCAN   (1 << 4)
#define DEBUG_ST_STATE  (1 << 5)    /* S/T state machine */
#define DEBUG_HDLC      (1 << 6)    /* HDLC controller */
#define DEBUG_ALARM     (1 << 7)    /* alarm changes */

#define DBG             (debug & DEBUG_GENERAL)
#define DBG_DTMF        (debug & DEBUG_DTMF)
#define DBG_REGS        (debug & DEBUG_REGS)
#define DBG_FOPS        (debug & DEBUG_FOPS)
#define DBG_EC          (debug & DEBUG_ECHOCAN)
#define DBG_ST          (debug & DEBUG_ST_STATE)
#define DBG_HDLC        (debug & DEBUG_HDLC)
#define DBG_ALARM       (debug & DEBUG_ALARM)

extern int vpmsupport;
extern int tdm_mode;
extern int spi_fifo_mode;
extern int interface_num;

unsigned int debug = 0;
unsigned int te_nt_override = 0x3; /*mode settings:  0:NT, 1:TE */
unsigned int alarmdebounce = 500;
unsigned int teignorered = 0;

static const int TIMER_3_MS = 30000;
#define XHFC_T1                 0
#define XHFC_T2                 1
#define XHFC_T3                 2

#define B200M_SPANS_PER_MODULE      2       /* 2 Spans for B200M module */
#define B200M_CHANNELS_PER_SPAN     3       /* 2 B-channels and 1 D-Channel for each BRI span */
#define B200M_HDLC_BUF_LEN          128     /* arbitrary, just the max of byts we will send to DAHDI per call */

struct x2b200m;
struct b200m_span {
    struct x2b200m *parent;
    unsigned int port;          /* which S/T port this span belongs to */

    int sync;                   /* sync */
    int syncpos;                /* sync priority */

    int oldstate;               /* old state machine state */
    int newalarm;               /* alarm to send to DAHDI once alarm timer expires */
    unsigned long alarmtimer;

    unsigned int te_mode:1;         /* 1=TE, 0=NT */
    unsigned int term_on:1;         /* 1= 390 ohm termination enable, 0 = disabled */
    unsigned long hfc_timers[B200M_CHANNELS_PER_SPAN];  /* T1, T2, T3 */
    int hfc_timer_on[B200M_CHANNELS_PER_SPAN];      /* 1=timer active */
    int fifos[B200M_CHANNELS_PER_SPAN];             /* B1, B2, D <--> host fifo numbers */

    /* DAHDI core API  */
    struct dahdi_span span;  /* pointer to the actual dahdi_span */
    struct dahdi_chan *chans[B200M_CHANNELS_PER_SPAN];    /* Individual channels */
    struct dahdi_echocan_state ec[B200M_CHANNELS_PER_SPAN-1]; /* echocan state for each channel */
    struct dahdi_chan _chans[B200M_CHANNELS_PER_SPAN]; /* Backing memory */

    /* HDLC controller fields */
    struct dahdi_chan *sigchan;     /* pointer to the signalling channel for this span */

    atomic_t hdlc_pending;          /* hdlc_hard_xmit() increments, decrements */

    unsigned char writechunk[B200M_CHANNELS_PER_SPAN * DAHDI_CHUNKSIZE];
    unsigned char readchunk[B200M_CHANNELS_PER_SPAN * DAHDI_CHUNKSIZE];
};

/* This structure exists one per module */
struct x2b200m {
    struct x200_dev *dev;              /* parent structure */
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
    struct dahdi_device *ddev;
#endif
    spinlock_t fifolock;                /* lock for all FIFO accesses (reglock must be available) */
    unsigned long ticks;
    unsigned long fifo_en_rxint;        /* each bit is the RX int enable for that FIFO */
    unsigned long fifo_en_txint;        /* each bit is the TX int enable for that FIFO */
    unsigned int fifo_irqmsk;           /* top-half ORs in new interrupts, bottom-half ANDs them out */

    //int syncspan;                       /* Span reported from HFC for sync on this card */
    unsigned int running:1;             /* interrupts are enabled */

    struct b200m_span spans[B200M_SPANS_PER_MODULE];        /* Individual spans */
    int dead;
    int usecount;
    int exist;
    unsigned int te_mode ;           /* 1=TE, 0=NT */
    int prio;
};


struct device_driver x200_dev_driver[WC_MAX_INTERFACES];

static int b200m_unset_clk_src(struct x200_dev *dev);

typedef unsigned short int uint16;
uint16 bit_invert(uint16 val)
{
    uint16 invval;
    int i;

    invval = 0x0000;
    for(i=0;i<16;i++)
    {
        if((val >> i) & 0x0001)
            invval = invval | (0x1 << (15-i));
    }

    return invval;
}

uint16 hdlc_crc(unsigned char *pMsg,uint16 mLen)
{
    uint16 i,bit_cnt;

    unsigned char data;
    unsigned char *p;
    uint16 crcReg=0x84CF;

    p = pMsg;
    for(i=0;i<mLen+2;i++)
    {
        bit_cnt = 0;
        if(i<mLen)
            data = *p++;
        else
            data = 0;

        while(bit_cnt < 8)
        {
            if((crcReg & 0x8000) != 0)
            {
                crcReg = crcReg ^ 0x0810;
                data = data ^ 1;
            }

            crcReg = crcReg << 1;
            if((data & 1) != 0)
                 crcReg++;

            data = data >> 1;
            bit_cnt++;
    		}
    }
    crcReg =~ crcReg;
    crcReg = bit_invert(crcReg);

    return crcReg;
}

void hfc_start_st(struct b200m_span *s);
void hfc_stop_st(struct b200m_span *s);

void wait_just_a_bit(int foo)
{
    long newjiffies;
    newjiffies = jiffies + foo;
    while (jiffies < newjiffies)
        ;
}

inline void flush_hw(void)
{
}


static void __xhfc_write_waitbusy(struct x200_dev *dev, unsigned char addr, unsigned char value)
{
	int timeout = 0;
	unsigned long start;
	const int TIMEOUT = HZ/4; /* 250ms */

	start = jiffies;
	while ( unlikely((__b200m_read_xhfc(dev,R_STATUS,1) & V_BUSY )) ) {
		if (time_after(jiffies, start + TIMEOUT)) {
			timeout = 1;
			break;
		}
	};

	__b200m_write_xhfc(dev,addr,value);

	start = jiffies;
	while (likely((__b200m_read_xhfc(dev,R_STATUS,1) & V_BUSY ))) {
		if (time_after(jiffies, start + TIMEOUT)) {
			timeout = 1;
			break;
		}
	};

	if (timeout) {
		printk(KERN_INFO "b200m: __xhfc_write_waitbusy(write 0x%02x to 0x%02x) timed out waiting for busy flag to clear!\n",value,addr);
	}
}

static void xhfc_write_waitbusy(struct x200_dev *dev, unsigned char addr, unsigned char value)
{
	int timeout = 0;
	unsigned long start;
	const int TIMEOUT = HZ/4; /* 250ms */

	start = jiffies;
	while ( unlikely((b200m_read_xhfc(dev,R_STATUS,1) & V_BUSY )) ) {
		if (time_after(jiffies, start + TIMEOUT)) {
			timeout = 1;
			break;
		}
	};

	b200m_write_xhfc(dev,addr,value);

	start = jiffies;
	while (likely((b200m_read_xhfc(dev,R_STATUS,1) & V_BUSY ))) {
		if (time_after(jiffies, start + TIMEOUT)) {
			timeout = 1;
			break;
		}
	};

	if (timeout) {
		printk(KERN_INFO "b200m: xhfc_write_waitbusy(write 0x%02x to 0x%02x) timed out waiting for busy flag to clear!\n",value,addr);
	}
}

inline void hfc_reset_fifo(struct x200_dev *dev)
{
    xhfc_write_waitbusy(dev, A_INC_RES_FIFO, V_RES_FIFO | V_RES_LOST | V_RES_FIFO_ERR);
}

/* takes a read/write fifo pair and optionally resets it, optionally enabling the rx/tx interrupt */
void hfc_reset_fifo_pair(struct x200_dev *dev, int fifo, int reset, int force_no_irq)
{
    unsigned char b;
    struct x2b200m *b2 = NULL;
    b2 = dev->drv_data;
		
    xhfc_write_waitbusy(dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT));
		
		b = (!force_no_irq && (b2->fifo_en_txint & (1 << fifo))) ?V_FIFO_IRQMSK:0;
    b |= V_MIX_IRQ;
    xhfc_write_waitbusy(dev, A_FIFO_CTRL, b);

    if (reset)
        hfc_reset_fifo(dev);

    xhfc_write_waitbusy(dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT) | V_FIFO_DIR);
    
		b = (!force_no_irq && (b2->fifo_en_rxint & (1 << fifo))) ?V_FIFO_IRQMSK:0;
    b |= V_MIX_IRQ;
    xhfc_write_waitbusy(dev, A_FIFO_CTRL, b);

    if (reset)
        hfc_reset_fifo(dev);
}

/* enable FIFO RX int and reset the FIFO */
int hdlc_start(struct x200_dev *dev, int fifo)
{
    struct  x2b200m *b2=NULL;

    b2 = dev->drv_data;
    b2->fifo_en_txint |= (1 << fifo);
    b2->fifo_en_rxint |= (1 << fifo);
		
    hfc_reset_fifo_pair(dev, fifo, 1, 0);
    return 0;
}

/* enable FIFO RX int and reset the FIFO */
void hdlc_stop(struct x200_dev *dev, int fifo)
{
    struct  x2b200m *b2=NULL;

    b2 = dev->drv_data;
    b2->fifo_en_txint &= ~(1 << fifo);
    b2->fifo_en_rxint &= ~(1 << fifo);
	
    hfc_reset_fifo_pair(dev, fifo, 1, 0);
}

void b200m_set_sync_src(struct x2b200m *b2, int port);
int b200m_find_sync(struct x2b200m *b2);
void b200m_reset_span(struct b200m_span *bspan)
{
    int i;
    struct x2b200m *b2 = bspan->parent;

    for (i = 0; i < 3; i++)
        hfc_reset_fifo_pair(b2->dev, bspan->fifos[i], (i == 2) ? 1 : 0, 0);

    b200m_set_sync_src(b2, b200m_find_sync(b2));
}

void xhfc_enable_fifo_irqs(struct x2b200m *b2)
{
    b200m_write_xhfc(b2->dev, R_IRQ_CTRL, V_FIFO_IRQ_EN | V_GLOB_IRQ_EN);
    flush_hw();
}

void xhfc_enable_interrupts(struct x2b200m *b2)
{
    b2->running = 1;

    /* mask all misc interrupts */
    b200m_write_xhfc(b2->dev, R_MISC_IRQMSK, 0x01);

    /* clear any pending interrupts */
    b200m_read_xhfc(b2->dev, R_STATUS,1);                     
    b200m_read_xhfc(b2->dev, R_MISC_IRQ,1);                   
    b200m_read_xhfc(b2->dev, R_FIFO_BL0_IRQ,1);               
    b200m_read_xhfc(b2->dev, R_FIFO_BL1_IRQ,1);               
    b200m_read_xhfc(b2->dev, R_FIFO_BL2_IRQ,1);               
    b200m_read_xhfc(b2->dev, R_FIFO_BL3_IRQ,1);               

    xhfc_enable_fifo_irqs(b2);
}

void xhfc_disable_interrupts(struct x2b200m *b2)
{
    b200m_write_xhfc(b2->dev, R_MISC_IRQMSK, 0);
    b200m_write_xhfc(b2->dev, R_IRQ_CTRL, 0);
    flush_hw();
    b2->running = 0;
}

/******************************************************************************/
/*
 * Filesystem and DAHDI interfaces
 */
/******************************************************************************/
int b200m_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
    return  0;
}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
int b200m_startup(struct file *file,struct dahdi_span *span)
#else
int b200m_startup(struct dahdi_span *span)
#endif
{
    struct b200m_span *bspan = container_of(span, struct b200m_span, span);
    struct x2b200m *b2 = bspan->parent;

    if (!b2->running)
    {
        xhfc_enable_interrupts(b2);
    }
    
    return  0;
}

int b200m_shutdown(struct dahdi_span *span)
{
    int i ;
    struct b200m_span *bspan = container_of(span, struct b200m_span, span);
    struct x2b200m *b2 = bspan->parent;

		for ( i=0; i < B200M_SPANS_PER_MODULE; i++) {
	    	__x200_set_led(b2->dev->bus, b2->dev->slot_id*2+i, LED_OFF);
		}

    if (b2->running)
    {
        xhfc_disable_interrupts(bspan->parent);
    }
    return  0;
}

/* spanconfig for us means to set up the HFC FIFO and channel mapping */
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
int b200m_spanconfig(struct file *file,struct dahdi_span *span, struct dahdi_lineconfig *lc)
#else
int b200m_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc)
#endif
{
    struct b200m_span *bspan = container_of(span, struct b200m_span, span);
    struct x2b200m *b2 = bspan->parent;

    if (DBG)
        printk("b200m_spanconfig line=%d, span->spanno=%d,slot=%d,port=%d, lc->sync=%d.\n",__LINE__, span->spanno,b2->dev->slot_id,bspan->port,lc->sync);

    if (lc->sync < 0 || lc->sync > X200_MAX_CLOCK_SOURCE) {
        printk("Span %d has invalid sync priority (%d), removing from sync source list\n", span->spanno, lc->sync);
        lc->sync = 0;
    }
		
 		bspan->syncpos = lc->sync;
 		//printk(KERN_INFO "----B200M:%s spanno=%d\n",__func__,span->spanno);
 		
    b200m_reset_span(bspan);
    
    /* call startup() manually here, because DAHDI won't call the startup function
     * unless it receives an IOCTL to do so, and dahdi_cfg doesn't.
     */
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
    b200m_startup(file,&bspan->span);
#else
		b200m_startup(&bspan->span);
#endif

    span->flags |= DAHDI_FLAG_RUNNING;
    b2->dev->bus->checktiming |= 1<<b2->dev->slot_id ;  //?????
    return  0;
}

/* chanconfig for us means to configure the HDLC controller, if appropriate */
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
int b200m_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
#else
int b200m_chanconfig(struct dahdi_chan *chan, int sigtype)
#endif
{
    int alreadyrunning;
    struct x2b200m *b2 = chan->pvt;
    struct b200m_span *bspan = &b2->spans[chan->span->offset];
    int fifo = bspan->fifos[2];

    alreadyrunning = bspan->span.flags & DAHDI_FLAG_RUNNING;

    if (DBG_FOPS) {
        printk("%s channel %d (%s) sigtype %08x\n",
            alreadyrunning ? "Reconfigured" : "Configured", chan->channo, chan->name, sigtype);
    }

    /* (re)configure signalling channel */
    if ((sigtype == DAHDI_SIG_HARDHDLC) || (bspan->sigchan == chan)) {
        if (DBG_FOPS)
            printk("%sonfiguring hardware HDLC on %s\n",
                ((sigtype == DAHDI_SIG_HARDHDLC) ? "C" : "Unc"), chan->name);

        if (alreadyrunning && bspan->sigchan) {
            hdlc_stop(b2->dev, fifo);
            bspan->sigchan = NULL;
        }

        if (sigtype == DAHDI_SIG_HARDHDLC) {
            if (hdlc_start(b2->dev, fifo)) {
                printk("Error initializing signalling controller\n");
                return -1;
            }
        }

        bspan->sigchan = (sigtype == DAHDI_SIG_HARDHDLC) ? chan : NULL;
        atomic_set(&bspan->hdlc_pending, 0);
    } else {
        /* FIXME: shouldn't I be returning an error? */
    }
		
    return  0;
}

int _b200m_open(struct dahdi_chan *chan)
{
    struct x2b200m *b2 = chan->pvt;
    struct b200m_span *bspan = &b2->spans[chan->span->offset];
	
    if (DBG_FOPS)
        printk("B200M: b200m_open() on chan %s (%i/%i)\n", chan->name, chan->channo, chan->chanpos);

    if (b2->dead)
        return -ENODEV;

    b2->usecount++;
		
    hfc_reset_fifo_pair(b2->dev, bspan->fifos[chan->chanpos-1], 0, 0);

    return 0;
}

static int b200m_open(struct dahdi_chan *chan)
{
	unsigned long flags;
	int res;
	spin_lock_irqsave(&chan->lock, flags);
	res = _b200m_open(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
	return res;
}

void b200m_release(struct x2b200m *b2)
{
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)
		int i;
		struct b200m_span *bspan = NULL;
#endif
		if(!b2)
			return;
			
    if(b2->exist)
    {
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
        dahdi_unregister_device(b2->ddev);
        kfree(b2->ddev->location);
				dahdi_free_device(b2->ddev);
#else
				for(i=0; i<B200M_SPANS_PER_MODULE; i++)
        {
            bspan = &b2->spans[i];
            dahdi_unregister(&bspan->span);
        }
#endif
				
				b200m_unset_clk_src(b2->dev);
				
        b2->exist = 0 ;
        printk("b200m: bus %d, blade %d, slot %d Freed a OpenVox B200M card.\n",b2->dev->bus->bus_id, b2->dev->parent->blade_id, b2->dev->slot_id);
        kfree(b2);
        b2 = NULL;
    }
		
}

int b200m_close(struct dahdi_chan *chan)
{
    struct x2b200m *b2 = chan->pvt;

    //printk(KERN_EMERG"B200M: line:%d, b200m close on chan %s (%i/%i)\n",__LINE__, chan->name, chan->channo, chan->chanpos);
    b2->usecount--;

    /* If we're dead, release us now */
    if (!b2->usecount && b2->dead)
        b200m_release(b2);
    return 0;
}

/* DAHDI calls this when it has data it wants to send to the HDLC controller */
void b200m_hdlc_hard_xmit(struct dahdi_chan *chan)
{
    struct x2b200m *b2 = chan->pvt;
    int span = chan->span->offset;
    struct b200m_span *bspan = &b2->spans[span];

    // increment pending counter and trigger the bottom-half so it will be picked up and sent.
    if (bspan->sigchan == chan) {
        atomic_inc(&bspan->hdlc_pending);
    }
}

const struct dahdi_echocan_features vpm_ec_features = {
    .NLP_automatic = 1,
    .CED_tx_detect = 1,
    .CED_rx_detect = 1,
};

void b200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

const struct dahdi_echocan_ops vpm_ec_ops = {
#if DAHDI_VERSION_CODE < VERSION_CODE(2,5,0)
		.name = "B200M VPM",
#endif
    .echocan_free = b200m_echocan_free,
};

int b200m_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
    int channel;
    struct x2b200m *b2 = chan->pvt;
    struct b200m_span *bspan = &b2->spans[chan->span->offset];
    struct x200* x2 = b2->dev->bus;

    if (!vpmsupport || !x2->vpm_present ) {
        return -ENODEV;
    }

    if (ecp->param_count > 0) {
        printk(KERN_ERR "error at line:%d,  echo canceller does not support parameters; failing request\n",__LINE__);
        return -EINVAL;
    }

    if (x2->vpm450m) {
        *ec = &bspan->ec[chan->chanpos-1];
        (*ec)->ops = &vpm_ec_ops;
        (*ec)->features = vpm_ec_features;
    } else {
        printk(KERN_ERR "error at line:%d\n",__LINE__);
        return -EINVAL;
    }

    channel = bspan->span.offset*8 + chan->chanpos*4 + b2->dev->slot_id  ;

    //printk("b200m echocan_create,line=%d,bspan->span.offset=%d, chan->chanpos=%d, b2->dev->slot_id=%d,channel=%d,ecp->tap_length=%d. \n",
    //    __LINE__,bspan->span.offset, chan->chanpos,b2->dev->slot_id, channel, ecp->tap_length);
    vpm450m_setec(x2, channel, b2->dev->slot_id, ecp->tap_length);
    return 0;
}

void b200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
    int channel;
    struct x2b200m *b2 = chan->pvt;
    struct b200m_span *bspan = &b2->spans[chan->span->offset];
    struct x200* x2 = b2->dev->bus;
    memset(ec, 0, sizeof(*ec));
    channel = bspan->span.offset*8 + chan->chanpos*4 + b2->dev->slot_id  ;
    //printk("b200m echocan_free, line=%d,bspan->span.offset=%d, chan->chanpos=%d, b2->dev->slot_id=%d,channel=%d \n",
    //    __LINE__,bspan->span.offset, chan->chanpos,b2->dev->slot_id, channel);
    if (x2->vpm450m) {
        vpm450m_setec(x2, channel, b2->dev->slot_id, 0);
    }
}

struct dahdi_span_ops b200m_span_ops = {
    .owner = THIS_MODULE,
    .spanconfig = b200m_spanconfig,
    .chanconfig = b200m_chanconfig,
    .startup = b200m_startup,
    .shutdown = b200m_shutdown,
    .open = b200m_open,
    .close  = b200m_close,
    .ioctl = b200m_ioctl,
    .hdlc_hard_xmit = b200m_hdlc_hard_xmit,
    .echocan_create = b200m_echocan_create,
};

// resets an S/T interface to a given NT/TE mode
void hfc_reset_st(struct b200m_span *s) 
{
    int b;
    struct x2b200m *b2;

    b2 = s->parent;

/* force state G0/F0 (reset), then force state 1/2 (deactivated/sensing) */
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, V_SU_LD_STA);
    flush_hw();         /* make sure write hit hardware */

/* set up the clock control register.  Must be done before we activate the interface. */
    if (s->te_mode)
        b = 0x0e;
    else
        b = 0x0c | (6 << V_SU_SMPL_SHIFT);

    //b200m_write_xhfc(b2->dev, A_SU_CLK_DLY, b);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_CLK_DLY, b);

    /* set TE/NT mode, enable B and D channels. */
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_CTRL0, V_B1_TX_EN | V_B2_TX_EN |(s->te_mode ? 0 : V_SU_MD) | V_ST_PU_CTRL);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_CTRL1, V_G2_G3_EN);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_CTRL2, V_B1_RX_EN | V_B2_RX_EN);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_ST_CTRL3, (0x7c << 1));

    /* enable the state machine. */
    //b200m_write_xhfc(b2->dev, A_SU_WR_STA, 0x00);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 0x00);
    flush_hw();

    udelay(100);
}

void hfc_start_st(struct b200m_span *s)
{
    struct x2b200m *b2 = s->parent;
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, V_SU_ACT_ACTIVATE);

/* start T1 if in NT mode, T3 if in TE mode */
    if (s->te_mode) {
        s->hfc_timers[XHFC_T3] = b2->ticks + 500;   /* 500ms wait first time, timer_t3_ms afterward. */
        s->hfc_timer_on[XHFC_T3] = 1;
        s->hfc_timer_on[XHFC_T1] = 0;
        if (DBG_ST)
            printk("setting port %d t3 timer to %lu\n", s->port + 1, s->hfc_timers[XHFC_T3]);

    } else {
        s->hfc_timers[XHFC_T1] = b2->ticks + 2000;
        s->hfc_timer_on[XHFC_T1] = 1;
        s->hfc_timer_on[XHFC_T3] = 0;
        if (DBG_ST)
            printk("setting port %d t1 timer to %lu\n", s->port + 1, s->hfc_timers[XHFC_T1]);
    }
}

void hfc_stop_all_timers(struct b200m_span *s)
{
    s->hfc_timer_on[XHFC_T3] = 0;
    s->hfc_timer_on[XHFC_T2] = 0;
    s->hfc_timer_on[XHFC_T1] = 0;
}

void hfc_stop_st(struct b200m_span *s)
{
    struct x2b200m *b2 = s->parent;

    hfc_stop_all_timers(s);

    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, V_SU_ACT_DEACTIVATE);
}

/*
 * read in the HFC GPIO to determine each port's mode (TE or NT).
 * Then, reset and start the port.
 * the flow controller should be set up before this is called.
 */
void hfc_init_all_st(struct x200_dev *dev)
{
    unsigned int i;
    struct  x2b200m *b2 = dev->drv_data;
    struct  b200m_span *s=NULL;
    for (i=0; i < B200M_SPANS_PER_MODULE; i++) {
        s = &b2->spans[i];
        s->parent = b2;
        s->port = i;
        s->te_mode =  ((b2->te_mode)>>i) & 0x1 ;
        printk("b200m: bus %d,blade %d,slot %d module %d te_mode=%d, set to %s \n",
        													dev->bus_id,dev->blade_id,dev->slot_id, i, s->te_mode, s->te_mode? "TE" : "NT");
        hfc_reset_st(s);
        hfc_start_st(s);
    }
}

char *hfc_decode_st_state(struct x2b200m *b2, int port, unsigned char state, int full);
void hfc_force_st_state(struct x2b200m *b2, int port, int state)
{
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, port, A_SU_WR_STA, state | V_SU_LD_STA);
    b200m_write_xhfc_ra(b2->dev, R_SU_SEL, port, A_SU_WR_STA, state);
}

/* figures out what to do when an S/T port's timer expires. */
void hfc_timer_expire(struct b200m_span *s, int t_no)
{
    struct x2b200m *b2 = s->parent;

    if (DBG_ST)
        printk("%lu: hfc_timer_expire, Port %d T%d expired (value=%lu ena=%d)\n", b2->ticks, s->port + 1, t_no + 1, s->hfc_timers[t_no], s->hfc_timer_on[t_no]);

    /*
     * there are three timers associated with every HFC S/T port.
     *
     * T1 is used by the NT state machine, and is the maximum time the NT
     * side should wait for G3 (active) state.
     *
     * T2 is not actually used in the driver, it is handled by the HFC-4S
     * internally.
     *
     * T3 is used by the TE state machine; it is the maximum time the TE
     * side should wait for the INFO4 (activated) signal.
     */

    /* First, disable the expired timer;  may activate it again. */
    s->hfc_timer_on[t_no] = 0;

    switch (t_no) {
    case XHFC_T1:   /* switch to G4 (pending deact.), resume auto mode */
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 4 | V_SU_LD_STA);
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 4);
        break;
    case XHFC_T2:   /* switch to G1 (deactivated), resume auto mode */
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 1 | V_SU_LD_STA);
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 1);
        break;
    case XHFC_T3:   /* switch to F3 (deactivated), resume auto mode */
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 3 | V_SU_LD_STA);
        b200m_write_xhfc_ra(b2->dev, R_SU_SEL, s->port, A_SU_WR_STA, 3);
        break;
    default:
        if (printk_ratelimit()) {
            printk("hfc_timer_expire found an unknown expired timer (%d)??\n", t_no);
        }
    }
}

/*
 * allocates memory and pretty-prints a given S/T state engine state to it.
 * calling routine is responsible for freeing the pointer returned!  Performs
 * no hardware access whatsoever, but does use GFP_KERNEL so do not call from
 * IRQ context.  if full == 1, prints a "full" dump; otherwise just prints
 * current state.
 */
char *hfc_decode_st_state(struct x2b200m *b2, int port, unsigned char state, int full)
{
    int nt, sta;
    char s[128], *str;
    const char *ststr[2][16] = {    /* TE, NT */
        { "RESET",       "?",           "SENSING", "DEACT.",
          "AWAIT.SIG",   "IDENT.INPUT", "SYNCD",    "ACTIVATED",
          "LOSTFRAMING", "?",           "?",        "?",
          "?",           "?",           "?",        "?" },
        { "RESET",       "DEACT.",      "PEND.ACT", "ACTIVE",
          "PEND.DEACT",  "?",           "?",        "?",
          "?",           "?",           "?",        "?",
          "?",           "?",           "?",        "?" }
    };

    str = kmalloc(256, GFP_KERNEL);
    if (!str) {
        printk("could not allocate mem for ST state decode string!\n");
        return NULL;
    }

    nt = (b2->spans[port].te_mode == 0);
    sta = (state & V_SU_STA_MASK);
    sprintf(str, "P%d: %s state %c%d (%s)", port + 1,
        (nt ? "NT" : "TE"), (nt ? 'G' : 'F'), sta,
        ststr[nt][sta]);

    if (full) {
        sprintf(s, " SYNC: %s, RX INFO0: %s",
            ((state & V_SU_FR_SYNC) ? "yes" : "no"),
            ((state & V_SU_INFO0) ? "yes" : "no"));
        strcat(str, s);
        if (nt) {
            sprintf(s, ", T2 %s, auto G2->G3: %s",
                ((state & V_SU_T2_EXP) ? "expired" : "OK"),
                ((state & V_G2_G3) ? "yes" : "no"));
            strcat(str, s);
        }
    }
    return str;
}

/*
 * Run through the active timers on a card and deal with any expiries.
 * Also see if the alarm debounce time has expired and if it has, tell DAHDI.
 */
void hfc_update_st_timers(struct x2b200m *b2)
{
    int i, j;
    struct b200m_span *s;
    struct x200* x2 = b2->dev->bus;

    for (i=0; i < 2; i++) {
        s = &b2->spans[i];

        for (j=XHFC_T1; j <= XHFC_T3; j++) {

            /* we don't really do timer2, it is expired by the state change handler */
            if (j == XHFC_T2)
                continue;

            if (s->hfc_timer_on[j] && time_after_eq(b2->ticks, s->hfc_timers[j]))
            	{
                hfc_timer_expire(s, j);
              }
        }

        if (s->newalarm != s->span.alarms && time_after_eq(b2->ticks, s->alarmtimer)) {
            s->span.alarms = s->newalarm;
            //printk("b200m:line %d,b2->dev->slot_id=%d,i=%d,s->span.alarms=0x%x,\n",__LINE__,b2->dev->slot_id,i,s->span.alarms);
            if ((!s->newalarm && teignorered) || (!teignorered)) {
                dahdi_alarm_notify(&s->span);
            }
            x2->checktiming |= 1<<b2->dev->slot_id ;
            if (DBG_ALARM) {
                printk(KERN_INFO "span %d: alarm %d debounced\n",i + 1, s->newalarm);
            }
        }
    }
}
/* this is the driver-level state machine for an S/T port */
void hfc_handle_state(struct b200m_span *s, unsigned char state )
{
    struct x2b200m *b2;
    unsigned char sta;
    int nt,oldalarm;
    unsigned long oldtimer;

    b2 = s->parent;
    nt = !s->te_mode;

    sta = (state & V_SU_STA_MASK);

    oldalarm = s->newalarm;
    oldtimer = s->alarmtimer;
      	
    if (nt) {
        switch(sta) {
        default:            // Invalid NT state 
        case 0x0:           // NT state G0: Reset 
        case 0x1:           // NT state G1: Deactivated 
        case 0x4:           // NT state G4: Pending Deactivation 
            s->newalarm = DAHDI_ALARM_RED;
            break;
        case 0x2:           // NT state G2: Pending Activation 
            s->newalarm = DAHDI_ALARM_YELLOW;
            break;
        case 0x3:           // NT state G3: Active 
            s->hfc_timer_on[XHFC_T1] = 0;
            s->newalarm = 0;
            break;
        }
    } else {
        switch(sta) {
        default:            // Invalid TE state 
        case 0x0:           // TE state F0: Reset 
        case 0x2:           // TE state F2: Sensing 
        case 0x3:           // TE state F3: Deactivated 
        case 0x4:           // TE state F4: Awaiting Signal 
        case 0x8:           // TE state F8: Lost Framing 
            s->newalarm = DAHDI_ALARM_RED;
            break;
        case 0x5:           //TE state F5: Identifying Input
        case 0x6:           // TE state F6: Synchronized 
            s->newalarm = DAHDI_ALARM_YELLOW;
            break;
        case 0x7:           // TE state F7: Activated 
            s->hfc_timer_on[XHFC_T3] = 0;
            s->newalarm = 0;
            break;
        }
    }
			
    s->alarmtimer = b2->ticks + alarmdebounce;
    s->oldstate = state;
		
    // we only care about T2 expiry in G4.
    if (nt && (sta == 4) && (state & V_SU_T2_EXP)) {
        if (s->hfc_timer_on[XHFC_T2])
        {
            hfc_timer_expire(s, XHFC_T2);   // handle T2 expiry 
         }
    }

    // If we're in F3 and receiving INFO0, start T3 and jump to F4 
    if (!nt && (sta == 3) && (state & V_SU_INFO0)) {
            s->hfc_timers[XHFC_T3] = b2->ticks + TIMER_3_MS;
        s->hfc_timer_on[XHFC_T3] = 1;
        hfc_force_st_state(b2, s->port, 4);
    }    
}

int hdlc_tx_frame(struct b200m_span *bspan)
{
    struct x2b200m *b2 = bspan->parent;
    int res, i, fifo;
    unsigned char buf[B200M_HDLC_BUF_LEN];
    unsigned int size = sizeof(buf) / sizeof(buf[0]);
       
    fifo = bspan->fifos[2];
    while(1) {
        res = dahdi_hdlc_getbuf(bspan->sigchan, buf, &size);
        xhfc_write_waitbusy(b2->dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT));
        if (size > 0) {
            for (i=0; i < size; i++) {
                b200m_write_xhfc(b2->dev, A_FIFO_DATA, buf[i]);
                //printk("%02x",buf[i]);
            }
            //printk("\n");
            if (res != 0) {
                xhfc_write_waitbusy(b2->dev, A_INC_RES_FIFO, V_INC_F);
                atomic_dec(&bspan->hdlc_pending);
            } else {           		
                xhfc_write_waitbusy(b2->dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT));
                xhfc_write_waitbusy(b2->dev, A_FIFO_CTRL, V_FIFO_IRQMSK);
            }
        }

        /* if there are no more frames pending, disable the interrupt. */
        if (res == -1) {
            xhfc_write_waitbusy(b2->dev, A_FIFO_CTRL, 0);
        }
        if ( res ) {
            break;
        }
    }    
    
    //printk("\n");
    return 0;
}

int hdlc_rx_frame(struct b200m_span *bspan)
{
    int fifo, i, j; //
    int z1, z2, zlen, zleft, f1, f2;
    unsigned char buf[B200M_HDLC_BUF_LEN],*p;
    struct x2b200m *b2 = bspan->parent;
    unsigned char stat;
    unsigned int rdata;
    unsigned char spi_rx_pendings_local = 0x0;
    unsigned char spi_rx_pendings_min = 0x20;
    
    uint16 crcchecksum;
    
    unsigned long flags;
    struct x200* x2 = b2->dev->bus;

    if (!bspan->sigchan) {
        return 0;
    }
    fifo = bspan->fifos[2];
		
		
    xhfc_write_waitbusy(b2->dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT) | V_FIFO_DIR);
    if ( spi_fifo_mode  ) {
        b200m_read_xhfc(b2->dev, A_F1,0);
        b200m_read_xhfc(b2->dev, A_F2,0);
        rdata = get_spi_fifo_data(b2->dev,4) ;
        f1 = rdata >> 24 ;
        rdata = get_spi_fifo_data(b2->dev,4) ;
        f2 = rdata >> 24 ;
    } else {
        f1 = b200m_read_xhfc(b2->dev, A_F1,0);
        f2 = b200m_read_xhfc(b2->dev, A_F2,0);
    }
		
		if (f1 == f2) { //normal exit
        return 0 ;
    }
    
    if ( spi_fifo_mode  ) {
        b200m_read_xhfc(b2->dev, A_Z1,0);
        b200m_read_xhfc(b2->dev, A_Z2,0);
        rdata = get_spi_fifo_data(b2->dev,4) ;
        z1 = rdata>>24 ;
        rdata = get_spi_fifo_data(b2->dev,4) ;
        z2 = rdata>>24 ;
    } else {
        z1 = b200m_read_xhfc(b2->dev, A_Z1,0);
        z2 = b200m_read_xhfc(b2->dev, A_Z2,0);
    }
    
    zlen = z1 - z2;
    if(zlen < 0) {
        zlen += 0x40 ;
    }

    /* include STAT byte that the HFC injects after FCS */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    //printk("slot=%d,span=%d,F1=0x%02x,F2=0x%02x, z1=0x%02x, z2=0x%02x, zlen=0x%02x:",b2->dev->slot_id,bspan->port + 1,f1,f2,z1,z2,zlen);
    spin_lock_irqsave(&x2->lock, flags);
    __xhfc_write_waitbusy(b2->dev, R_FIFO, (fifo << V_FIFO_NUM_SHIFT) | V_FIFO_DIR);  
    if ( spi_fifo_mode  ) {
        j=0 ;    		
    		__wait_spi_wr_fifo(b2->dev, 0);
    		__x200_setcreg(x2,X200_SPI_BASE, 0x10, 0x0);
    		
    		/**/ 	 		
        for (i=0; i < zlen + 1; i++) {
            __b200m_read_xhfc(b2->dev, A_FIFO_DATA,0);     
            spi_rx_pendings_local += 4 ;
            if( spi_rx_pendings_local > spi_rx_pendings_min ) {
                rdata = __get_spi_fifo_data(b2->dev,4) ;
                buf[j] = rdata>>24 ;
                //printk(" 0x%02x",buf[j]);
                spi_rx_pendings_local -= 4 ;
                j++ ;
            }
        }    
               
        while ( (j < i)&&(spi_rx_pendings_local>0) ) {
            rdata = __get_spi_fifo_data(b2->dev,4) ;
            buf[j] = rdata>>24 ;
            //printk(" 0x%02x",buf[j]);
            spi_rx_pendings_local -= 4 ;
            j++ ;
        }
        //printk("\n");        
    } else {
        for (i=0; i < zlen + 1; i++) {
            buf[i] = __b200m_read_xhfc(b2->dev, A_FIFO_DATA,0);
            //printk("%02x",buf[i]);
        }
    }
    spin_unlock_irqrestore(&x2->lock, flags);
       
    /* don't send STAT byte to DAHDI */
    zleft = zlen;
    p = buf;
    do{
    	 if (zleft > 32) {
					j = 32;
			 } else {
			    j = zleft;
		   }
		   
		   if (j > 0)
			 		dahdi_hdlc_putbuf(bspan->sigchan, p, j);
			 
			 zleft -= j;
			 p += j;
    }while(zleft > 0);

    /* Frame received, increment F2 and get an updated count of frames left */
    xhfc_write_waitbusy(b2->dev, A_INC_RES_FIFO, V_INC_F);

    stat = buf[zlen];
    //printk("%s:b%ds%dp%d:hdlc_rxlen=0x%02x,stat=0x%02x\n",__func__,b2->dev->blade_id,b2->dev->slot_id,bspan->port,zlen,stat);
    if (zlen < 3) {
        printk("odd, zlen less then 3?\n");
        dahdi_hdlc_abort(bspan->sigchan, DAHDI_EVENT_ABORT);
    } else {
        /* if STAT != 0, indicates bad frame */
        if (stat != 0x00) {
        		crcchecksum = hdlc_crc(buf,zlen-2);
            if(((crcchecksum & 0x00FF) != buf[zlen-2]) ||
            	 (((crcchecksum >> 8) & 0x00FF) != buf[zlen-1]) )
            {
            		printk( "(span %d) STAT=0x%02x indicates frame problem: ", bspan->port + 1, stat);
            		if (stat == 0xff) {
                		printk("HDLC Abort\n");
                		dahdi_hdlc_abort(bspan->sigchan, DAHDI_EVENT_ABORT);
            		} else {
              			printk("Bad FCS.\n");
                		dahdi_hdlc_abort(bspan->sigchan, DAHDI_EVENT_BADFCS);
            		}
            }
            else
            {
            		//CRC OK, means frame was OK.
            		dahdi_hdlc_finish(bspan->sigchan);
            }
        /* STAT == 0, means frame was OK */
        } else {
            dahdi_hdlc_finish(bspan->sigchan);
        }
    }
    return 0;
}

void b200m_set_sync_src(struct x2b200m *b2, int port)
{
    int b;

    if (port == -1)         /* automatic */
        b = 0;
    else
        b = (port & V_SYNC_SEL_MASK) | V_MAN_SYNC;

    //printk("\t B200M %s:Setting sync to be port %d\n", __func__, port);
    b200m_write_xhfc(b2->dev, R_SU_SYNC, b);
}

/*
 * Finds the highest-priority sync span that is not in alarm and returns it.
 * Note: the span #s in b2->spans[].sync are 1-based, and this returns
 * a 0-based span, or -1 if no spans are found.
 */
int b200m_find_sync(struct x2b200m *b2)
{
    int i, psrc, src;
    src = -1;       /* default to automatic */
    for (i=0; i <B200M_SPANS_PER_MODULE; i++) {
        psrc = b2->spans[i].sync;
        if (psrc > 0 && !b2->spans[psrc - 1].span.alarms) {
            src = psrc;
            break;
        }
    }

    if (src >= 0)
    {
    	printk("\t B200M %s:Find sync span : %d\n", __func__, i);
        return src - 1;
    }
    else
        return src;
}

/*********************************************************/
/* Setup Fifo using A_CON_HDLC, A_SUBCH_CFG, A_FIFO_CTRL */
/*********************************************************/
void setup_fifo(struct x200_dev *dev, unsigned char fifo, unsigned char conhdlc, unsigned char subcfg, unsigned char fifoctrl, unsigned char enable)
{
    struct x2b200m *b2 = NULL;
    b2 = dev->drv_data;
    
    xhfc_write_waitbusy(dev, R_FIFO, fifo);
    b200m_write_xhfc(dev, A_CON_HDLC, conhdlc);
    b200m_write_xhfc(dev, A_SUBCH_CFG, subcfg);
    xhfc_write_waitbusy(dev, A_FIFO_CTRL, fifoctrl);

    if (enable)
        b2->fifo_irqmsk |= (1 << fifo);
    else
        b2->fifo_irqmsk &= ~(1 << fifo);

    hfc_reset_fifo(dev);
    xhfc_write_waitbusy(dev, R_FIFO, fifo);
}

/*
 * Stage 2 hardware init.
 * Sets up the flow controller, PCM and FIFOs.
 * Initializes the echo cancellers.
 * S/T interfaces are not initialized here, that is done later, in hfc_init_all_st().
 * Interrupts are enabled and once the s/t interfaces are configured, chip should be pretty much operational.
 */
void b200m_init_stage2(struct x200_dev *dev)
{
    int span;
    int b, channel, pcmslot;
    struct x2b200m *b2 = dev->drv_data;

    /*
     * set up PCM bus.
     * XHFC is PCM slave.C2IO is the clock,auto sync,
     * C4IO, SYNC_I and SYNC_O unused.
     * 32 channels, frame signal negtive polarity, active for 2 C4 clocks.
     * only the first two timeslots in each quad are active
     * STIO0 is transmit-only, STIO1 is receive-only.
     */
    b200m_write_xhfc(dev, R_PCM_MD0, V_PCM_IDX_MD1);
    b200m_write_xhfc(dev, R_PCM_MD1, V_PCM_DR_8192 | V_PLL_ADJ_11);

    b200m_write_xhfc(dev, R_PCM_MD0, V_PCM_IDX_MD2);
    b200m_write_xhfc(dev, R_PCM_MD2, V_C2I_EN);
    b200m_write_xhfc(dev, R_PCM_MD0, 0 );		//V_F0_LEN || V_C4_POL 

    //-b200m_write_xhfc(dev, R_SU_SYNC, V_SYNC_SEL_PORT0);

    b200m_write_xhfc(dev, R_PWM_MD, 0xa0);
    b200m_write_xhfc(dev, R_PWM0, 0x1b);
    b200m_write_xhfc(dev, R_SL_MAX, 0x7f);
    b200m_write_xhfc(dev, R_CLK_CFG, V_PCM_CLK);

    /*  01 = fSYS/4  */
    b200m_write_xhfc(dev, R_CTRL, 0x40);
    
    for (span=0; span < B200M_SPANS_PER_MODULE; span++) {
        for(b = 0; b < 3; b++) {
            pcmslot = span*8 + (1+b)*4 + dev->slot_id;
            channel = span*4 + b;

            if(b == 2) {
                b2->spans[span].fifos[b] = channel;
                setup_fifo(dev, (channel << 1), 5, 2, 0x08, 1);     /* D TX fifo */
                setup_fifo(dev, (channel << 1) + 1, 5, 2, 0x09, 1); /* D RX fifo */
            }
            else {
                printk("b200m: bus:%d,blade:%d,slot:%d, span:%d, b:%d,selecting pcmslot: %d.\n",
                																	dev->bus_id,dev->blade_id,dev->slot_id,span,b,pcmslot);
                b2->spans[span].fifos[b] = pcmslot;
                setup_fifo(dev, (channel << 1), 0xc6, 0, 0, 1); /* B-TX Fifo */
                setup_fifo(dev, (channel << 1) + 1, 0xc6, 0, 0, 1); /* B-RX Fifo */

                b200m_write_xhfc(dev, R_SLOT, (pcmslot << 1));              /* PCM timeslot B1 TX */
                b200m_write_xhfc(dev, A_SL_CFG, (channel << 1) + 0xc0);     /* enable B1 TX timeslot on STIO1 */

                b200m_write_xhfc(dev, R_SLOT, (pcmslot << 1) + 1);      /* PCM timeslot B1 RX */
                b200m_write_xhfc(dev, A_SL_CFG, (channel << 1) + 1 + 0xc0);       /* enable B1 RX timeslot on STIO2*/
            }
        }
    }

    /* set up the timer interrupt for 1ms intervals */
    b200m_write_xhfc(dev, R_TI_WD, (2 << V_EV_TS_SHIFT));
}

void xhfc_reset(struct x200_dev *dev)
{
    unsigned long start;
    int TIMEOUT = HZ;           /* 1s */
    struct x2b200m *b2 = NULL;

    b2 = dev->drv_data;

    /* Set the FIFOs to 8 128 bytes FIFOs, bidirectional, and set up the
     * flow controller for channel select mode. */
    /* Note, this reg has to be set *before* the SW reset */
    b200m_write_xhfc(dev, R_FIFO_MD, 0x04);
    msleep(1);      /* wait a bit for clock to settle */

  /* reset everything, wait 100ms, then allow the XHFC to come out of reset */
    b200m_write_xhfc(dev, R_CIRM, V_SRES);
    wait_just_a_bit(HZ/2);
    
		b200m_write_xhfc(dev, R_CIRM, 0x00);
		
    /* wait for XHFC to come out of reset. */
    start = jiffies;
    while (b200m_read_xhfc(dev, R_STATUS,1) & (V_BUSY | V_PCM_INIT)) {                    //xhfc_reset
        if (time_after(jiffies, start + TIMEOUT)) {
            printk("B200M: xhfc_reset() Module won't come out of reset... continuing.\n");
            break;
        }
    };

    /* Disable the output clock pin, and also the PLL (it's not needed) */
    b200m_write_xhfc(dev, R_CTRL, 0x00);
}


 /*
    gpio0   s2(te/nt) select, use as input
    gpio1   (gpio_ctl_1),enable port1 TE 38v output
    gpio2   (sw1),0: fsync and clock from pcm bus to chip, 1: from chip to bus(feedback clock)
    gpio3   (sw_pwr),enable 38v to be generated,
    gpio4   (pcm_tx)
    gpio5   (pcm_rx)
    gpio6   s1(te/nt) select, use as input
    gpio7   (gpio_ctl_2),enable port2 TE 38v output

*/
void xhfc_gpio_init(struct x200_dev *dev)
{
    unsigned char gpio_value = 0x00;
    struct  x2b200m *b2 = dev->drv_data;

    b2->te_mode=0x3 ; // in default, it's TE mode
    b200m_write_xhfc(dev, 0x4C, 0xcf);  //R_GPIO_SEL=0x4C,  gpio7,gpio6,gpio3,gpio2,gpio1,gpio0 use as gpio.
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x48, 0x00);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x4A, 0x8e);  //R_GPIO_EN0=0x4A,  set gpio7=0,gpio3=0,gpio2=0,gpio1=0

    //for new_board
    gpio_value = b200m_read_xhfc(dev, 0x48,1);
    //printk("B200M:gpio=0x%x, gpio_6=0x%x, gpio_0=0x%x.\n",gpio_value,(gpio_value>>6)&0x1,gpio_value&0x1);
    if( (gpio_value & 0x41) != 0x41 ) {             /*gpio0 and gpio6 determine the TE/NT mode, 0 means NT mode.*/
        gpio_value |= 0x08;                         /*gpio3(sw_pwr)  enabled */
    
        //printk("B200M: gpio_value set to 0x%x to switch on 38V.\n",gpio_value );
        b200m_write_xhfc(dev, 0x48, gpio_value);    /* write out GPIO output data bits */
        b200m_write_xhfc(dev, 0x4A, 0x8e);          /* write out GPIO output enable bit */
        mdelay(50);

        if((gpio_value & 0x01) == 0 ) {     /* gpio0==0, port1 is NT port */
        		gpio_value |= 0x02;             /* gpio1(gpio_ctl_1) enabled */
            b2->te_mode &= 0xfe ;
        }
        
        if((gpio_value & 0x40) == 0 ) {     /*  gpio6==0, port2 is NT port */        
            gpio_value |= 0x80;             /*gpio7(gpio_ctl_2) enabled. */
            b2->te_mode &= 0xfd ;
        }
        
        b200m_write_xhfc(dev, 0x48, gpio_value);    /* write out GPIO output data bits */
        b200m_write_xhfc(dev, 0x4A, 0x8e);          /* write out  GPIO output enable bit */   
    }
}

void b200m_init_stage1(struct x200_dev *dev)
{
    int i;
    struct  x2b200m *b2=NULL;

    b2 = dev->drv_data;
        /* xhfc init stage 1*/
    xhfc_reset(dev);

    xhfc_gpio_init(dev);


    /* make sure interrupts are disabled */
    b200m_write_xhfc(dev, R_IRQ_CTRL, 0x00);

    /* make sure write hits hardware */
    flush_hw();

    /* disable all FIFO interrupts */
    for (i = 0; i < HFC_NR_FIFOS; i++) {
        xhfc_write_waitbusy(dev, R_FIFO, (i << V_FIFO_NUM_SHIFT));
        /* disable the interrupt */
        xhfc_write_waitbusy(dev, A_FIFO_CTRL, 0x00);
        xhfc_write_waitbusy(dev, R_FIFO, (i << V_FIFO_NUM_SHIFT) | V_FIFO_DIR);
        /* disable the interrupt */
        xhfc_write_waitbusy(dev, A_FIFO_CTRL, 0x00);
        flush_hw();
    }

    /* set fill threshhold to 16 bytes */
    b200m_write_xhfc(dev, R_FIFO_THRES, 0x11);

    /* clear any pending FIFO interrupts */
    b200m_read_xhfc(dev, R_FIFO_BL2_IRQ,1);
    b200m_read_xhfc(dev, R_FIFO_BL3_IRQ,1);

    b200m_write_xhfc(dev, R_MISC_IRQMSK, 0);
    b200m_write_xhfc(dev, R_IRQ_CTRL, 0);
}

int b200m_hardware_init(struct x200_dev *dev)
{
    b200m_init_stage1(dev);
    b200m_init_stage2(dev);
    hfc_init_all_st(dev);
    return 0;
}

int b200m_software_init(struct x200_dev *dev)
{
    int i, j,channel;
    struct x200* x2 = dev->bus;
    struct x2b200m *b2 = dev->drv_data;
    struct b200m_span *bspan;
    struct dahdi_chan *chan;

    /* for each span on the card */
    for (i=0; i <B200M_SPANS_PER_MODULE; i++) {
        bspan = &b2->spans[i];
        bspan->parent = b2;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,7,0)
				bspan->span.spantype = (bspan->te_mode) ? SPANTYPE_DIGITAL_BRI_TE : SPANTYPE_DIGITAL_BRI_NT;
#else
        bspan->span.spantype = (bspan->te_mode) ? "TE" : "NT";
#endif
        bspan->span.offset = i;
        bspan->span.channels = B200M_CHANNELS_PER_SPAN;
        bspan->span.flags = 0;

        bspan->span.deflaw = DAHDI_LAW_ALAW;

        /* For simplicty, we'll accept all line modes since BRI
         * ignores this setting anyway.*/
        bspan->span.linecompat = DAHDI_CONFIG_AMI |
            DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 |
            DAHDI_CONFIG_ESF | DAHDI_CONFIG_HDB3 |
            DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;

        sprintf(bspan->span.name, "B200M/%d", i+1);
        sprintf(bspan->span.desc, "OpenVox B200M (NT|TE) Card %d Span %d", dev->slot_id, i+1);
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)
        bspan->span.manufacturer = "OpenVox";
        dahdi_copy_string(bspan->span.devicetype, "OpenVox B200M Card", sizeof(bspan->span.devicetype));
        snprintf(bspan->span.location, sizeof(bspan->span.location) - 1, "X200 Bus %02d Blade %d Slot %02d", dev->bus->bus_id, dev->parent->blade_id, dev->slot_id);
#ifdef DAHDI_SPAN_MODULE
        bspan->span.owner = THIS_MODULE;
#endif
#endif 
        bspan->span.ops = &b200m_span_ops;

        /* HDLC stuff */
        bspan->sigchan = NULL;
        bspan->span.chans = bspan->chans;
        //- init_waitqueue_head(&bspan->span.maintq);

        /* now initialize each channel in the span */
        for (j=0; j < bspan->span.channels; j++) {
            bspan->chans[j] = &bspan->_chans[j];
            chan = bspan->chans[j];
            chan->pvt = b2;

            sprintf(chan->name, "B200M1/%d/%d", i+1, j+1);
            /* The last channel in the span is the D-channel */
            if (j == 2) {
                chan->sigcap = DAHDI_SIG_HARDHDLC;
            } else {
                chan->sigcap = DAHDI_SIG_CLEAR | DAHDI_SIG_DACS;
            }
            chan->chanpos = j+1;
            chan->writechunk = (void *)(bspan->writechunk + j * DAHDI_CHUNKSIZE);
            chan->readchunk = (void *)(bspan->readchunk + j * DAHDI_CHUNKSIZE);
            
            //init ec channels
            channel = bspan->span.offset*8 + chan->chanpos*4 + dev->slot_id  ;
            if(x2->vpm_present){
							if(init_vpm450m_chan(x2, channel,dev->slot_id)){
    						return -1;
    					}
						}
        }
    }
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)
    for (i=0; i <B200M_SPANS_PER_MODULE; i++) {
        if (dahdi_register(&b2->spans[i].span, 0)) {
            return  -2;
        }
    }
#endif	
		
		return 0;
}

int b200m_init_one(struct x200_dev *dev)
{
    struct x200* x2 = dev->bus;
    int retval = -ENODEV;
    struct x2b200m *b2 = NULL;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
    int x;
#endif

    b2 = kmalloc(sizeof(*b2), GFP_KERNEL);
    if(NULL==b2)
        return -ENOMEM;

    memset(b2, 0, sizeof(*b2));

    b2->dev = dev;
    dev->drv_data = b2;
    spin_lock_init(&b2->fifolock);
		
    retval = b200m_hardware_init(dev);
    if(retval)
    {
      goto b200m_init_one_exit_err;
    }
    x2->card_type[dev->blade_id][dev->slot_id] = DEV_TYPE_B200M ;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)    
    b2->ddev = dahdi_create_device();
    if(!b2->ddev){
    	retval = -ENOMEM;
    	goto b200m_init_one_exit_err;
    }
#endif
    	
    retval = b200m_software_init(dev);

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
    for (x=0; x < B200M_SPANS_PER_MODULE; x++) {
			struct dahdi_span *const s = &b2->spans[x].span;
			list_add_tail(&s->device_node, &b2->ddev->spans);
		}
		
		b2->ddev->manufacturer = "OpenVox";
		b2->ddev->devicetype = "OpenVox B200M Card";
		b2->ddev->location = kasprintf(GFP_KERNEL, "X200 Bus %02d Blade %d Slot %02d",
				       dev->bus->bus_id,dev->parent->blade_id,
				       dev->slot_id);
		if(!b2->ddev->location) {
				retval = -ENOMEM;
				goto b200m_init_one_exit_err0;
		}
		
		if(dahdi_register_device(b2->ddev, &dev->dev)){  
			printk("B200M: Unable to register device.\n");
		}
		else{
			goto 	b200m_init_one_exit;
		}

b200m_init_one_exit_err0:
		kfree(b2->ddev->location);
		dahdi_free_device(b2->ddev);
#else
		if(retval == 0 )
			 goto b200m_init_one_exit;
#endif
		
b200m_init_one_exit_err:
    dev->drv_data = NULL;
    kfree(b2);
    return retval;    

b200m_init_one_exit:
    b2->exist = 1;
    return retval;
}

void b200m_remove_one(struct x200_dev *dev)
{
    struct x2b200m *b2 ;
    struct x200* x2 ;

    if(NULL==dev) {
        return;
    }
    
    b2 = dev->drv_data;  
    x2 = dev->bus;
    
    if(NULL==b2) {
        return;
    }

    if(!b2->exist) {
    		dev->drv_data = NULL;
    		dev->ops = NULL;
        return;
    }

    if((b2->te_mode & 0x3) != 0x3 ) {
        // printk("b200m:line %d,dev->slot_id=%d,set all gpio out to 0, to close 38V.\n",__LINE__,dev->slot_id);
         wait_spi_wr_fifo(dev,0);
         b200m_write_xhfc(dev, 0x4C, 0xcf);  //R_GPIO_SEL=0x4C,  gpio7,gpio6,gpio3,gpio2,gpio1,gpio0 use as gpio.
         wait_spi_wr_fifo(dev,0);
         b200m_write_xhfc(dev, 0x48, 0x0);    /* write out GPIO output data bits */
         wait_spi_wr_fifo(dev,0);
         b200m_write_xhfc(dev, 0x4A, 0x8e);   /* write out GPIO output enable bit */
         wait_spi_wr_fifo(dev,0);
                
        b200m_write_xhfc(dev, 0x48, 0x0);    /* write out GPIO output data bits */
        b200m_write_xhfc(dev, 0x4A, 0x8e);   /* write out GPIO output enable bit */
    }
		
		b2->dead = 1; 
		dev->drv_data = NULL; 		  
    if (!b2->usecount) {
        b200m_release(b2);
    }    		
    
    dev->ops = NULL;
    
    return;
}

/* So far only tested for OpenVox cards. Please test it for other hardware */
static void b200m_update_leds(struct x2b200m *b2)
{
	int i;
    struct b200m_span *bspan;
    struct x200* x2 = b2->dev->bus;
    unsigned long flags;

    if(x2->intcount&0xff) {
        return ;
    }

	for ( i=0; i < B200M_SPANS_PER_MODULE; i++) {
		bspan = &b2->spans[i];
		if (!(bspan->span.flags & DAHDI_FLAG_RUNNING)) {
			continue; /* Leds are off */
		}
		//printk("b200m: line %d,  slot=0x%x port=0x%x alarms=0x%x. \n",__LINE__, b2->dev->slot_id,i,bspan->span.alarms);
		spin_lock_irqsave(&x2->lock, flags);
		if (bspan->span.alarms) {   /* Red blinking -> Alarm */
		     __x200_set_led(x2, b2->dev->slot_id*2+i, LED_RED);
		} else {        /* Steady grean -> No Alarm */
		    __x200_set_led(x2, b2->dev->slot_id*2+i, LED_GREEN);
		}
		spin_unlock_irqrestore(&x2->lock, flags);
	}

}

int b200m_get_clk_src(struct x200_dev *dev, int * syncpos, int * port) {
    struct  x2b200m *b2=NULL;
    int port_id, new_port_id ;
    int new_syncpos ;
    struct b200m_span *bspan=NULL ;

    b2 = dev->drv_data;
    new_port_id = 0 ;
    new_syncpos = X200_MAX_CLOCK_SOURCE+1 ;
    
    for(port_id=0; port_id< B200M_SPANS_PER_MODULE; port_id++) {
        bspan = &b2->spans[port_id];
        if (!bspan || !(bspan->span.flags & DAHDI_FLAG_RUNNING)) {
            continue ;
        }
				
        if ( bspan->span.alarms )  {
            continue ;
        }
				
        if ( (bspan->syncpos > 0) && (bspan->syncpos < new_syncpos) )  {
            new_port_id = port_id ;
            new_syncpos = bspan->syncpos ;
        }
    }

    if ( new_syncpos == X200_MAX_CLOCK_SOURCE+1 )  { // no avalible clock source find
        new_port_id = 0 ;
        new_syncpos = 0 ;
        return -1 ;
    } else {
        *syncpos = new_syncpos ;
        *port = new_port_id ;
        return 0 ;
    }
}

int b200m_set_clk_src(struct x200_dev *dev, int syncpos, int port) {
    unsigned char gpio_before, gpio_after ;
    struct x200* x2 = dev->bus;
    int card_type = x2->card_type[dev->blade_id][dev->slot_id] ;
    	
		wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_SU_SYNC, (port & V_SYNC_SEL_MASK) | V_MAN_SYNC );
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD0, V_PCM_IDX_MD2);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD2, V_C2O_EN);          //V_SYNC_SRC | 
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD0, V_PCM_MD); //? V_F0_LEN | V_C4_POL  
		
    wait_spi_wr_fifo(dev,0);
    gpio_before = b200m_read_xhfc(dev, 0x48,1);
    gpio_after = gpio_before | 0x04 ;  //set gpio2(sw1)
    wait_spi_wr_fifo(dev,0); // we need to add this operation, otherwise will fail sometimes.
    b200m_write_xhfc(dev, 0x4C, 0xcf);  //R_GPIO_SEL=0x4C,  gpio7,gpio6,gpio3,gpio2,gpio1,gpio0 use as gpio.
    wait_spi_wr_fifo(dev,0); // we need to add this operation, otherwise will fail sometimes.
    b200m_write_xhfc(dev, 0x48, gpio_after);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x4A, 0x8e);  //R_GPIO_EN0=0x4A,  set gpio7=0,gpio3=0,gpio2=0,gpio1=0 
    wait_spi_wr_fifo(dev,0);
          
    printk("b200m: %s,line %d,bus:%d,  slot:%d  port:%d card_type:%d syncpos:%d set as master. gpio_before:0x%0x,gpio_after:0x%x.\n",
        __func__,__LINE__, x2->bus_id, dev->slot_id,port,card_type,syncpos,gpio_before,gpio_after);
        
    return 0 ;
}

static int b200m_unset_clk_src(struct x200_dev *dev) {
    unsigned char gpio_before, gpio_after ;
    struct x200* x2 = dev->bus;
    int card_type = x2->card_type[dev->blade_id][dev->slot_id] ;
    
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_SU_SYNC, 0 );
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD0, V_PCM_IDX_MD2);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD2, V_C2I_EN);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, R_PCM_MD0, 0 );	//V_F0_LEN | | V_C4_POL
    
    wait_spi_wr_fifo(dev,0);
    gpio_before = b200m_read_xhfc(dev, 0x48,1);
    gpio_after = gpio_before & ~0x4 ;  //unset gpio2(sw1)
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x4C, 0xcf);  //R_GPIO_SEL=0x4C,  gpio7,gpio6,gpio3,gpio2,gpio1,gpio0 use as gpio.
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x48, gpio_after);
    wait_spi_wr_fifo(dev,0);
    b200m_write_xhfc(dev, 0x4A, 0x8e);
    wait_spi_wr_fifo(dev,0);
     
    if(dev == x2->clock_master)
    	x200_set_clock_master(x2, NULL);
    	   
    printk("b200m: %s,line %d,  bus:%d slot:%d  car_type is %d unset as master. gpio_before:0x%0x,gpio_after:0x%x.\n",
        __func__,__LINE__, x2->bus_id, dev->slot_id,card_type, gpio_before, gpio_after);
    
    return 0 ;
}

int b200m_interrupt(int intcount, struct x200_dev *dev)
{
    struct x2b200m *b2 = dev->drv_data;
    unsigned char  b=0, port=0, state=0;
    int x, y;
    struct x200* x2 = dev->bus;
    struct  b200m_span *bspan = NULL;
	
    if ( NULL == dev ) {
        printk(KERN_EMERG"b200m: line %d. \n",__LINE__ );
        return 0 ;
    }
    
    if ( NULL == b2 ) {
        printk(KERN_EMERG"b200m: line %d. \n",__LINE__ );
        return 0 ;
    }
    
    if (b2->dead ) {
        printk(KERN_EMERG"b200m: line %d. \n",__LINE__ );
        return 0 ;
    }
 		
    for(port = 0; port < 2; port++) {
        bspan = &b2->spans[port];
        if (bspan->span.flags & DAHDI_FLAG_RUNNING) {
            for (x=0; x<2; x++) {
                for (y=0; y<DAHDI_CHUNKSIZE; y++) {
                    bspan->chans[x]->readchunk[y] = x2->current_readchunk[bspan->fifos[x]+y*128];
                    x2->current_writechunk[bspan->fifos[x]+y*128] = bspan->chans[x]->writechunk[y];
 								}
            }
            dahdi_receive(&bspan->span);
            dahdi_transmit(&bspan->span);
        }
    }
		
    //hdlc tx and rx        
    for(port = 0; port < 2; port++){
        struct  b200m_span *bspan = &b2->spans[port];
        b = b200m_read_xhfc(dev, R_FIFO_BL0_IRQ+port,1);  //hdlc
				       
        if (b & 0x10) {
            hdlc_tx_frame(bspan);
        }

        if (b & 0x20){
            hdlc_rx_frame(bspan);
        }

        if (atomic_read(&bspan->hdlc_pending))  {
            hdlc_tx_frame(bspan);
        }
    }
		
    for(port = 0; port < 2; port++){
        struct  b200m_span *bspan = &b2->spans[port];
        // if we're ignoring TE red alarms and we are in alarm, restart the S/T state machine 
        if (bspan->te_mode && teignorered && bspan->newalarm == DAHDI_ALARM_RED) {
            hfc_force_st_state(b2, bspan->port, 3);
        }
    }
    
    // every 30ms or so, look at the S/T interfaces to see if they changed state 
    if ( (b2->ticks % 30 ) == 0 ) {
        b = b200m_read_xhfc(dev, R_SU_IRQ,1);
        for(port = 0; port < 2; port++) {
            if (b & (1 << port)) {
                struct  b200m_span *bspan = &b2->spans[port];
                b200m_write_xhfc(dev, R_SU_SEL, port);
                state = b200m_read_xhfc(dev, A_SU_RD_STA,1);
                hfc_handle_state(bspan, state);
            }
        }
    }
		
    hfc_update_st_timers(b2);
    b200m_update_leds(b2);
    b2->ticks++;
    return  1;
}


struct ints_ops b200m_ops={
		.irq_handler = b200m_interrupt,
    .get_clk_src = b200m_get_clk_src,
    .set_clk_src = b200m_set_clk_src,
    .unset_clk_src = b200m_unset_clk_src,
};

static int x200_dev_probe(struct device *dev)
{
		int ret;
		struct x200_dev *x2dev = to_x200_dev(dev);
		ret = b200m_init_one(x2dev);
		if(!ret){
				x2dev->ops = &b200m_ops;
		}
		return ret;
}

static int x200_dev_remove(struct device *dev)
{
		struct x200_dev *x2dev = to_x200_dev(dev);
		b200m_remove_one(x2dev);
		return 0;
}

static int __init b200m_init(void)
{
    int i;
    struct bus_type *x200bus; 
     
    for(i=0;i<interface_num;i++){
    		x200bus = get_bus(i);
    		x200_dev_driver[i].name = "b200m_driver";
    		x200_dev_driver[i].bus = x200bus;
    		x200_dev_driver[i].probe = x200_dev_probe;
    		x200_dev_driver[i].remove = x200_dev_remove;
    		    		
    		if(driver_register(&x200_dev_driver[i]))
    		{
    				printk("x200 bus %d register d100m driver error!\n",i);
    		}
    }
    return 0;
}

static void __exit b200m_cleanup(void)
{
    int i;
		
		for(i=0;i<interface_num;i++){
				driver_unregister(&x200_dev_driver[i]);
		}
}

module_param(debug, int, S_IRUGO | S_IWUSR);
module_param(teignorered, int, S_IRUGO | S_IWUSR);
module_param(alarmdebounce, int, S_IRUGO | S_IWUSR);
module_param(te_nt_override, int, S_IRUGO | S_IWUSR);

MODULE_PARM_DESC(debug, "bitmap: 1=general 2=dtmf 4=regops 8=fops 16=ec 32=st state 64=hdlc 128=alarm");
MODULE_PARM_DESC(teignorered, "1=ignore (do not inform DAHDI) if a red alarm exists in TE mode");
MODULE_PARM_DESC(alarmdebounce, "msec to wait before set/clear alarm condition");
MODULE_PARM_DESC(te_nt_override, "Overrides driver mode for B200M.\n"\
                             "You need a bitmap as follows 10 (all spans in TE except the 1st)\n"\
                             "You must configure the jumpper and turn on the terminator for each port you expect to work in NT mode.");

MODULE_DESCRIPTION("OpenVox B200M Driver");
MODULE_AUTHOR("Miao Lin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(b200m_init);
module_exit(b200m_cleanup);
