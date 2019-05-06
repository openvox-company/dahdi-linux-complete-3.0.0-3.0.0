/*
 * OpenVox D100M module for x200 telephony card driver.
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
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>

#include <dahdi/kernel.h>

#include "ec3000.h"

/* XXX: fix this */
#include "wct4xxp/wct4xxp.h"	/* For certain definitions */

#include "x200_hal.h"
#include "x2d100m.h"

extern int vpmsupport;
extern int tdm_mode;
extern int spi_fifo_mode;
extern int interface_num;


static int debug = 0;
static int t1e1override = -1;	/* 0xff for E1, 0x00 for T1 */
static int j1mode = 0;
static int loopback = 0;
static int alarmdebounce = 25; /* LOS def to 2.5s AT&T TR54016*/


#define	TYPE_T1	1		/* is a T1 card */
#define	TYPE_E1	2		/* is an E1 card */
#define	TYPE_J1	3		/* is an J1 card */

struct x2d100m {
	struct x200_dev *dev;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	struct dahdi_device *ddev;
#endif

	/*spinlock_t lock; */
	int spantype;
	int spanflags;
	unsigned char txsigs[16];  /* Copy of tx sig registers */
	int alarmcount;			/* How much red alarm we've seen */
	int usecount;
	int clocktimeout;
	int sync;
	int dead;
	int blinktimer;
	int alarmpos;
	int alarmtimer;
  int loopupcnt;
	int loopdowncnt;

	unsigned char ec_chunk1[32][DAHDI_CHUNKSIZE];
	unsigned char ec_chunk2[32][DAHDI_CHUNKSIZE];
	struct dahdi_span span;						        /* Span */
	struct dahdi_chan *chans[32];				        /* Channels */
	struct dahdi_echocan_state *ec[32];                 /* Echcan state for each channel */

  unsigned char controller_ver;
  int falc_ver;
  unsigned int tdm_slot_start;
	int	exist;
};


struct device_driver x200_dev_driver[WC_MAX_INTERFACES];

static int d100m_unset_clk_src(struct x200_dev *dev);

static const struct dahdi_echocan_features vpm_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static void d100m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_ops vpm_ec_ops = {
#if DAHDI_VERSION_CODE < VERSION_CODE(2,5,0)
	.name = "OpenVox VPMOCT128",
#endif
	.echocan_free = d100m_echocan_free,
};


static int d100m_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	int channel;
	struct x2d100m *d1 = chan->pvt;
    struct x200* x2 = d1->dev->bus;

	if (!vpmsupport || !x2->vpm_present ) {
		return -ENODEV;
	}

	if (ecp->param_count > 0) {
		printk(KERN_ERR "error at line:%d,  echo canceller does not support parameters; failing request\n",__LINE__);
		return -EINVAL;
	}

	if (x2->vpm450m) {
	  *ec = d1->ec[chan->chanpos - 1];  
	  (*ec)->ops = &vpm_ec_ops;
	  (*ec)->features = vpm_ec_features;
	} else {
		printk(KERN_ERR "error at line:%d\n",__LINE__);
		return -EINVAL;
	}

  channel = chan->chanpos * 4 + d1->dev->slot_id;
  printk("d100m : %s,line:%d: chan->chanpos=0x%x, d1->dev->slot_id=0x%x,channel=0x%x, ecp->tap_length=0x%x. \n",
        __FUNCTION__, __LINE__,chan->chanpos,d1->dev->slot_id, channel, ecp->tap_length);
    
  vpm450m_setec(x2, channel, d1->dev->slot_id, ecp->tap_length);
	return 0;
}

static void d100m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	int channel;

	struct x2d100m *d1 = chan->pvt;
  struct x200* x2 = d1->dev->bus;
	memset(ec, 0, sizeof(*ec));
	if (x2->vpm450m) {
    channel = chan->chanpos * 4 + d1->dev->slot_id;
    //printk("d100m echocan_free, line:%d, chan->chanpos=0x%x, d1->dev->slot_id=0x%x,channel=0x%x \n", __LINE__,chan->chanpos,d1->dev->slot_id, channel);
    vpm450m_setec(x2, channel, d1->dev->slot_id, 0);
  }
}

static int d100m_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	struct t4_regs regs;
	int x;
	struct x2d100m *d1;

	switch(cmd) {
	case WCT4_GET_REGS:
		d1 = chan->pvt;
		
		for (x=0;x<NUM_PCI;x++)
      regs.pci[x] = ((volatile unsigned int*)(d1->dev->bus->mem32))[x];
		
		for (x=0;x<NUM_REGS;x++)
			regs.regs[x] = t1_framer_in(d1->dev, x);
		if (copy_to_user((__user void *)data, &regs, sizeof(regs)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static inline struct x2d100m *t1_from_span(struct dahdi_span *span)
{
	return container_of(span, struct x2d100m, span);
}

static int d100m_maint(struct dahdi_span *span, int cmd)
{
	struct x2d100m *d1 = t1_from_span(span);

    if(debug) {
        printk("D100M: d100m_maint called\n");
    }
	if (d1->spantype == TYPE_E1) {
		switch(cmd) {
		case DAHDI_MAINT_NONE:
			printk(KERN_INFO "XXX Turn off local and remote loops E1 XXX\n");
			break;
		case DAHDI_MAINT_LOCALLOOP:
			printk(KERN_INFO "XXX Turn on local loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			printk(KERN_INFO "XXX Turn on remote loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			printk(KERN_INFO "XXX Send loopup code E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPDOWN:
			printk(KERN_INFO "XXX Send loopdown code E1 XXX\n");
			break;
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)
		case DAHDI_MAINT_LOOPSTOP:
			printk(KERN_INFO "XXX Stop sending loop codes E1 XXX\n");
			break;
#endif
		default:
			printk(KERN_NOTICE "TE110P: Unknown E1 maint command: %d\n", cmd);
			break;
		}
	} else {
		switch(cmd) {
	    case DAHDI_MAINT_NONE:
			printk(KERN_INFO "XXX Turn off local and remote loops T1 XXX\n");
			break;
	    case DAHDI_MAINT_LOCALLOOP:
			printk(KERN_INFO "XXX Turn on local loop and no remote loop XXX\n");
			break;
	    case DAHDI_MAINT_REMOTELOOP:
			printk(KERN_INFO "XXX Turn on remote loopup XXX\n");
			break;
	    case DAHDI_MAINT_LOOPUP:
			t1_framer_out(d1->dev, 0x21, 0x50);	/* FMR5: Nothing but RBS mode */
			break;
	    case DAHDI_MAINT_LOOPDOWN:
			t1_framer_out(d1->dev, 0x21, 0x60);	/* FMR5: Nothing but RBS mode */
			break;
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)
	    case DAHDI_MAINT_LOOPSTOP:
			t1_framer_out(d1->dev, 0x21, 0x40);	/* FMR5: Nothing but RBS mode */
			break;
#endif
	    default:
			printk(KERN_NOTICE "D100M: Unknown T1 maint command: %d\n", cmd);
			break;
	   }
    }
	return 0;
}

static int d100m_rbsbits(struct dahdi_chan *chan, int bits)
{
	u_char m,c;
	int n,b;
	struct x2d100m *d1 = chan->pvt;
	unsigned long flags;

	if(debug) 
		printk(KERN_DEBUG "Setting bits to %d on channel %s\n", bits, chan->name);
		
	spin_lock_irqsave(&d1->dev->bus->lock, flags);
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	if (dahdi_is_e1_span(&d1->span)) { /* do it E1 way d1->spantype == TYPE_E1*/
#else
	if (d1->spantype == TYPE_E1) {
#endif
		if (chan->chanpos == 16) {
			spin_unlock_irqrestore(&d1->dev->bus->lock, flags);
			return 0;
		}
		n = chan->chanpos - 1;
		if (chan->chanpos > 15) n--;
		b = (n % 15);
		c = d1->txsigs[b];
		m = (n / 15) << 2; /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		d1->txsigs[b] = c;
		  /* output them to the chip */
		__t1_framer_out(d1->dev,0x71 + b,c);
	} else if (d1->span.lineconfig & DAHDI_CONFIG_D4) {
		n = chan->chanpos - 1;
		b = (n/4);
		c = d1->txsigs[b];
		m = ((3 - (n % 4)) << 1); /* nibble selector */
		c &= ~(0x3 << m); /* keep the other nibble */
		c |= ((bits >> 2) & 0x3) << m; /* put our new nibble here */
		d1->txsigs[b] = c;
		  /* output them to the chip */
		__t1_framer_out(d1->dev,0x70 + b,c);
		__t1_framer_out(d1->dev,0x70 + b + 6,c);
	} else if (d1->span.lineconfig & DAHDI_CONFIG_ESF) {
		n = chan->chanpos - 1;
		b = (n/2);
		c = d1->txsigs[b];
		m = ((n % 2) << 2); /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		d1->txsigs[b] = c;
		  /* output them to the chip */
		__t1_framer_out(d1->dev,0x70 + b,c);
	}
	spin_unlock_irqrestore(&d1->dev->bus->lock, flags);
	if (debug)
		printk(KERN_DEBUG "Finished setting RBS bits\n");

    return 0;
}

static void d100m_check_sigbits(struct x2d100m *d1)
{
	int rxs;
	unsigned char a, i;
    struct x200_dev *dev = d1->dev;
    unsigned char regs[24];

	if (!(d1->span.flags & DAHDI_FLAG_RUNNING))
		return;

	if (d1->spantype == TYPE_E1) {
        t1_framer_burst_in(dev, 0x71, 16, regs);
		for (i = 0; i < 15; i++) {
            a = regs[i];
      //a = t1_framer_in(dev, 0x71+i);
			/* Get high channel in low bits */
			rxs = (a & 0xf);
			if (!(d1->span.chans[i+16]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i+16]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i+16], rxs);
				}
			}
			rxs = (a >> 4) & 0xf;
			if (!(d1->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i], rxs);
				}
			}
		}
	} else if (d1->span.lineconfig & DAHDI_CONFIG_D4) {
        t1_framer_burst_in(dev, 0x70, 6, regs);
		for (i = 0; i < 24; i+=4) {
            a = regs[i>>2];
      //a = t1_framer_in(dev, 0x70+(i>>2));   
			/* Get high channel in low bits */
			rxs = (a & 0x3) << 2;
			if (!(d1->span.chans[i+3]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i+3]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i+3], rxs);
				}
			}
			rxs = (a & 0xc);
			if (!(d1->span.chans[i+2]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i+2]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i+2], rxs);
				}
			}
			rxs = (a >> 2) & 0xc;
			if (!(d1->span.chans[i+1]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i+1]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i+1], rxs);
				}
			}
			rxs = (a >> 4) & 0xc;
			if (!(d1->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i], rxs);
				}
			}
		}
	} else {
        t1_framer_burst_in(dev, 0x70, 12, regs);
		for (i = 0; i < 24; i+=2) {
            a = regs[i];
      //a = t1_framer_in(dev, 0x70+(i>>1));
			/* Get high channel in low bits */
			rxs = (a & 0xf);
			if (!(d1->span.chans[i+1]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i+1]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i+1], rxs);
				}
			}
			rxs = (a >> 4) & 0xf;
			if (!(d1->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (d1->span.chans[i]->rxsig != rxs) {
					dahdi_rbsbits(d1->span.chans[i], rxs);
				}
			}
		}
	}
}
#if 0
	/* clock source checking - start */
static	void	d100m_set_timing_src(struct x200_dev *dev)
{
		if (debug)
			printk("Evaluating spans for timing source\n");

		for (x=0;x<wc->numspans;x++) {
			if ((wc->tspans[x]->span.flags & DAHDI_FLAG_RUNNING) &&
			   !(wc->tspans[x]->span.alarms & (DAHDI_ALARM_RED |
							   DAHDI_ALARM_BLUE))) {
				if (debug)
					dev_info(&wc->dev->dev, "span %d is green : syncpos %d\n", x+1,	wc->tspans[x]->syncpos);
			}
		}
}
	/* clock source checking - end   */
#endif

static void d100m_configure_t1(struct x2d100m *d1, int lineconfig, int txlevel)
{
	unsigned int fmr4, fmr2, fmr1, fmr0, lim2;
	char *framing, *line;
	int mytxlevel;
    struct x200_dev *dev = d1->dev;

	if ((txlevel > 7) || (txlevel < 4))
		mytxlevel = 0;
	else
		mytxlevel = txlevel - 4;
	fmr1 = 0x9c; /* FMR1: Mode 1, T1 mode, CRC on for ESF, 2.048 Mhz system data rate, no XAIS */
	fmr2 = 0x22; /* FMR2: no payload loopback, auto send yellow alarm */
	if (loopback)
		fmr2 |= 0x4;

	if (j1mode)
		fmr4 = 0x1c;
	else
		fmr4 = 0x0c; /* FMR4: Lose sync on 2 out of 5 framing bits, auto resync */


	lim2 = 0x21; /* LIM2: 50% peak is a "1", Advanced Loss recovery */
	lim2 |= (mytxlevel << 6);	/* LIM2: Add line buildout */
	t1_framer_out(dev, 0x1d, fmr1);
	t1_framer_out(dev, 0x1e, fmr2);

	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		line = "AMI";
		fmr0 = 0xa0;
	} else {
		line = "B8ZS";
		fmr0 = 0xf0;
	}
	if (lineconfig & DAHDI_CONFIG_D4) {
		framing = "D4";
	} else {
		framing = "ESF";
		fmr4 |= 0x2;
		fmr2 |= 0xc0;
	}
	t1_framer_out(dev, 0x1c, fmr0);

	t1_framer_out(dev, 0x20, fmr4);
	t1_framer_out(dev, 0x21, 0x40);	/* FMR5: Enable RBS mode */

	t1_framer_out(dev, 0x37, 0xf8);	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), No remote loop, no DRS */
	t1_framer_out(dev, 0x36, 0x08);	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */

	t1_framer_out(dev, 0x02, 0x50);	/* CMDR: Reset the receiver and transmitter line interface */
	t1_framer_out(dev, 0x02, 0x00);	/* CMDR: Reset the receiver and transmitter line interface */

	t1_framer_out(dev, 0x3a, lim2);	/* LIM2: 50% peak amplitude is a "1" */
	t1_framer_out(dev, 0x38, 0x0a);	/* PCD: LOS after 176 consecutive "zeros" */
	t1_framer_out(dev, 0x39, 0x15);	/* PCR: 22 "ones" clear LOS */

	if (j1mode)
		t1_framer_out(dev, 0x24, 0x80); /* J1 overide */

	/* Generate pulse mask for T1 */
	switch(mytxlevel) {
	case 3:
		t1_framer_out(dev, 0x26, 0x07);	/* XPM0 */
		t1_framer_out(dev, 0x27, 0x01);	/* XPM1 */
		t1_framer_out(dev, 0x28, 0x00);	/* XPM2 */
		break;
	case 2:
		t1_framer_out(dev, 0x26, 0x8c);	/* XPM0 */
		t1_framer_out(dev, 0x27, 0x11);	/* XPM1 */
		t1_framer_out(dev, 0x28, 0x01);	/* XPM2 */
		break;
	case 1:
		t1_framer_out(dev, 0x26, 0x8c);	/* XPM0 */
		t1_framer_out(dev, 0x27, 0x01);	/* XPM1 */
		t1_framer_out(dev, 0x28, 0x00);	/* XPM2 */
		break;
	case 0:
	default:
		t1_framer_out(dev, 0x26, 0xd7);	/* XPM0 */
		t1_framer_out(dev, 0x27, 0x22);	/* XPM1 */
		t1_framer_out(dev, 0x28, 0x01);	/* XPM2 */
		break;
	}
	printk(KERN_INFO "d100m: bus %d, blade %d, slot %d,Span configured for %s/%s\n", 
																	dev->bus_id, dev->blade_id,dev->slot_id,framing, line);
}

static void d100m_configure_e1(struct x2d100m* d1, int lineconfig)
{
	unsigned int fmr2, fmr1, fmr0;
	unsigned int cas = 0;
	char *crc4 = "";
	char *framing, *line;
    struct x200_dev* dev = d1->dev;

    fmr1 = 0x44; /* FMR1: E1 mode, Automatic force resync, PCM30 mode, 8.192 Mhz backplane, no XAIS */
	fmr2 = 0x03; /* FMR2: Auto transmit remote alarm, auto loss of multiframe recovery, no payload loopback */

    if (loopback)
		fmr2 |= 0x4;
	if (lineconfig & DAHDI_CONFIG_CRC4) {
		fmr1 |= 0x08;	/* CRC4 transmit */
		fmr2 |= 0xc0;	/* CRC4 receive */
		crc4 = "/CRC4";
	}
	t1_framer_out(dev, 0x1d, fmr1);
	t1_framer_out(dev, 0x1e, fmr2);

	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		line = "AMI";
		fmr0 = 0xa0;
	} else {
		line = "HDB3";
		fmr0 = 0xf0;
	}
	if (lineconfig & DAHDI_CONFIG_CCS) {
		framing = "CCS";
	} else {
		framing = "CAS";
		cas = 0x40;
	}

  t1_framer_out(dev, 0x1c, fmr0);

	t1_framer_out(dev, 0x37, 0xf0 /*| 0x6 */ );	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), No remote loop, no DRS */
	t1_framer_out(dev, 0x36, 0x08);	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */

	t1_framer_out(dev, 0x02, 0x50);	/* CMDR: Reset the receiver and transmitter line interface */
	t1_framer_out(dev, 0x02, 0x00);	/* CMDR: Reset the receiver and transmitter line interface */

	/* Condition receive line interface for E1 after reset */
	t1_framer_out(dev, 0xbb, 0x17);
	t1_framer_out(dev, 0xbc, 0x55);
	t1_framer_out(dev, 0xbb, 0x97);
	t1_framer_out(dev, 0xbb, 0x11);
	t1_framer_out(dev, 0xbc, 0xaa);
	t1_framer_out(dev, 0xbb, 0x91);
	t1_framer_out(dev, 0xbb, 0x12);
	t1_framer_out(dev, 0xbc, 0x55);
	t1_framer_out(dev, 0xbb, 0x92);
	t1_framer_out(dev, 0xbb, 0x0c);
	t1_framer_out(dev, 0xbb, 0x00);
	t1_framer_out(dev, 0xbb, 0x8c);

	t1_framer_out(dev, 0x3a, 0x20);	/* LIM2: 50% peak amplitude is a "1" */
	t1_framer_out(dev, 0x38, 0x0a);	/* PCD: LOS after 176 consecutive "zeros" */
	t1_framer_out(dev, 0x39, 0x15);	/* PCR: 22 "ones" clear LOS */

	t1_framer_out(dev, 0x20, 0x9f);	/* XSW: Spare bits all to 1 */

  t1_framer_out(dev, 0x21, 0x1c|cas);	/* XSP: E-bit set when async. AXS auto, XSIF to 1 */

	/* Generate pulse mask for E1 */
	t1_framer_out(dev, 0x26, 0x54);	/* XPM0 */
	t1_framer_out(dev, 0x27, 0x02);	/* XPM1 */
	t1_framer_out(dev, 0x28, 0x00);	/* XPM2 */
	printk(KERN_INFO "D100M: Span configured for %s/%s%s\n", framing, line, crc4);
}

static void d100m_do_counters(struct x2d100m* d1)
{
	unsigned long flags;

	spin_lock_irqsave(&d1->dev->bus->lock, flags);
	if (d1->alarmtimer) {
		if (!--d1->alarmtimer) {
			d1->span.alarms &= ~(DAHDI_ALARM_RECOVER);
			spin_unlock_irqrestore(&d1->dev->bus->lock, flags);
			dahdi_alarm_notify(&d1->span);
			spin_lock_irqsave(&d1->dev->bus->lock, flags);
		}
	}
	spin_unlock_irqrestore(&d1->dev->bus->lock, flags);
}


static void d100m_set_clear(struct x2d100m *d1)
{
	int i,j;
	unsigned short val=0;
	for (i=0;i<24;i++) {
		j = (i/8);
		if (d1->span.chans[i]->flags & DAHDI_FLAG_CLEAR)
			val |= 1 << (7 - (i % 8));
		if ((i % 8)==7) {
			if (debug > 1)
				printk(KERN_DEBUG "Putting %d in register %02x\n",
			       val, 0x2f + j);
			    t1_framer_out(d1->dev, 0x2f + j, val);
			val = 0;
		}
	}
}

static void d100m_framer_start(struct x2d100m *d1, struct dahdi_span *span)
{
	int alreadyrunning = d1->span.flags & DAHDI_FLAG_RUNNING;

	if (d1->spantype == TYPE_E1) { /* if this is an E1 card */
		d100m_configure_e1(d1, span->lineconfig);
	} else { /* is a T1 card */
		d100m_configure_t1(d1, span->lineconfig, span->txlevel);
		d100m_set_clear(d1);
	}

	if (!alreadyrunning)
		d1->span.flags |= DAHDI_FLAG_RUNNING;
}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
static int d100m_startup(struct file *file, struct dahdi_span *span)
#else
static int d100m_startup(struct dahdi_span *span)
#endif
{
	struct x2d100m *d1 = t1_from_span(span);
    int i,alreadyrunning = span->flags & DAHDI_FLAG_RUNNING;

    if(debug) {
        printk("D100M: d100m_startup called\n");
    }
	/* initialize the start value for the entire chunk of last ec buffer */
	for(i = 0; i < span->channels; i++)
	{
		memset(d1->ec_chunk1[i],DAHDI_LIN2X(0,span->chans[i]),DAHDI_CHUNKSIZE);
		memset(d1->ec_chunk2[i],DAHDI_LIN2X(0,span->chans[i]),DAHDI_CHUNKSIZE);
	}

	/* Reset framer with proper parameters and start */
	d100m_framer_start(d1, span);
	//printk(KERN_INFO "D100M: Calling startup (flags is %lu)\n", span->flags);

	if (!alreadyrunning) {
		/* Only if we're not already going */
		span->flags |= DAHDI_FLAG_RUNNING;
		d1->dev->bus->checktiming = 1;
	}
	return 0;
}

static int d100m_shutdown(struct dahdi_span *span)
{
	struct x2d100m *d1 = t1_from_span(span);
	struct x200* x2 = d1->dev->bus;
  unsigned long flags;

    if(debug) {
        printk("D100M: d100m_shutdown called\n");
    }
	t1_framer_out(d1->dev, 0x46, 0x41);	/* GCR: Interrupt on Activation/Deactivation of AIX, LOS */

	span->flags &= ~DAHDI_FLAG_RUNNING;
	
	spin_lock_irqsave(&x2->lock, flags);
	__x200_set_led(d1->dev->bus, d1->dev->slot_id*2, LED_OFF);
	spin_unlock_irqrestore(&x2->lock, flags);

	return 0;
}

static void d100m_chan_set_sigcap(struct dahdi_span *span, int x)
{
    struct x2d100m *d1 = t1_from_span(span);
	struct dahdi_chan *chan = d1->chans[x];

	chan->sigcap = DAHDI_SIG_CLEAR;
	/* E&M variant supported depends on span type */
	if (d1->spantype == TYPE_E1) {
		/* E1 sigcap setup */
		if (span->lineconfig & DAHDI_CONFIG_CCS) {
			/* CCS setup */
			chan->sigcap |= DAHDI_SIG_MTP2 | DAHDI_SIG_SF /*|DAHDI_SIG_HARDHDLC */;
			return;
		}
		/* clear out sig and sigcap for channel 16 on E1 CAS
		 * lines, otherwise, set it correctly */ 
		if (x == 15) {
			// CAS signaling channel setup 
			d1->chans[15]->sigcap = 0;
			d1->chans[15]->sig = 0;
			return;
		}
		
		/* normal CAS setup*/ 
		chan->sigcap |= DAHDI_SIG_EM_E1 | DAHDI_SIG_FXSLS |
			DAHDI_SIG_FXSGS | DAHDI_SIG_FXSKS | DAHDI_SIG_SF |
			DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS |
			DAHDI_SIG_CAS | DAHDI_SIG_DACS_RBS; 
	} else {
		/* T1 sigcap setup */
		chan->sigcap |= DAHDI_SIG_EM | DAHDI_SIG_FXSLS |
			DAHDI_SIG_FXSGS | DAHDI_SIG_FXSKS | DAHDI_SIG_MTP2 |
			DAHDI_SIG_SF | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS |
			DAHDI_SIG_FXOKS | DAHDI_SIG_CAS | DAHDI_SIG_DACS_RBS /*| DAHDI_SIG_HARDHDLC*/;
	}
}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
static int d100m_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
#else
static int d100m_chanconfig(struct dahdi_chan *chan, int sigtype)
#endif
{
	struct x2d100m *d1 = chan->pvt;

	int alreadyrunning = chan->span->flags & DAHDI_FLAG_RUNNING;

    if(debug) {
        printk("D100M: d100m_chanconfig called\n");
    }
	if (alreadyrunning && (d1->spantype != TYPE_E1))
		d100m_set_clear(d1);

	return 0;
}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
static int d100m_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc)
#else
static int d100m_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc)
#endif
{
	  struct x2d100m *d1 = t1_from_span(span);
    int i;

    if(debug)
        printk("D100M: d100m_spanconfig on span %d\n", span->spanno);

    if(lc->sync<0)
        lc->sync=0;
    if(lc->sync>X200_MAX_CLOCK_SOURCE)
        lc->sync=0;

    for(i=0; i<span->channels; i++) {
        d100m_chan_set_sigcap(span, i);
    }
	/* Do we want to SYNC on receive or not */
	d1->sync = lc->sync;

	/* If already running, apply changes immediately */
	if (span->flags & DAHDI_FLAG_RUNNING)
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0)
		return d100m_startup(file,span);
#else
		return d100m_startup(span);
#endif
  return 0;
}

static int _d100m_open(struct dahdi_chan *chan)
{
	struct x2d100m *d1 = chan->pvt;

    if(debug)
        printk("D100M: d100m_open called\n");

    if (d1->dead)
		return -ENODEV;
	d1->usecount++;
	return 0;
}

static int d100m_open(struct dahdi_chan *chan)
{
	unsigned long flags;
	int res;
	spin_lock_irqsave(&chan->lock, flags);
	res = _d100m_open(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
	return res;
}

static void d100m_release(struct x2d100m *d1)
{
	unsigned int x;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	dahdi_unregister_device(d1->ddev);
#else
	dahdi_unregister(&d1->span);
#endif
	
	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) {
		kfree(d1->chans[x]);
	}
	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) {
		kfree(d1->ec[x]);
	}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	kfree(d1->ddev->location);
	dahdi_free_device(d1->ddev);
#endif
	
	d100m_unset_clk_src(d1->dev);
	
	d1->exist = 0;
  printk("d100m: bus %d, blade %d, slot %d Freed a OpenVox D100M card.\n",d1->dev->bus->bus_id, d1->dev->parent->blade_id, d1->dev->slot_id);

	kfree(d1);
    d1 = NULL;
}

static int d100m_close(struct dahdi_chan *chan)
{
	struct x2d100m *d1 = chan->pvt;

    if(debug)
        printk("d100m: d100m_close called\n");

    d1->usecount--;

	/* If we're dead, release us now */
	if (!d1->usecount && d1->dead)
		d100m_release(d1);

    return 0;
}

static const struct dahdi_span_ops d100m_span_ops = {
	.owner = THIS_MODULE,
	.startup = d100m_startup,
	.shutdown = d100m_shutdown,
	.rbsbits = d100m_rbsbits,
	.maint = d100m_maint,
	.open = d100m_open,
	.close = d100m_close,
	.spanconfig = d100m_spanconfig,
	.chanconfig = d100m_chanconfig,
	.ioctl = d100m_ioctl,
	.echocan_create = d100m_echocan_create,
};

static inline void start_alarm(struct x2d100m * d1)
{
	d1->blinktimer = 0;
}

static int d100m_hardware_init(struct x200_dev *dev)
{
    int x;           
    unsigned char falcver;
    struct x2d100m *d1 = dev->drv_data;

    d1->controller_ver = d100m_read_reg(dev, REG_VER);
    printk(KERN_DEBUG "d100m: bus %d, blade %d, slot %d, Controller version: %02x\n", 
    												dev->bus_id,dev->blade_id,dev->slot_id,d1->controller_ver);

    if(t1e1override)
      d1->spantype = TYPE_E1;
    else {
	    if (j1mode)
				d1->spantype = TYPE_J1;
			else
		    d1->spantype = TYPE_T1;
	  }
    printk(KERN_INFO "d100m: bus %d, blade %d, slot %d, Spantype = 0x%02x\n", 
    												dev->bus_id,dev->blade_id,dev->slot_id,d1->spantype);

	/* Setup clock appropriately */
    d1->clocktimeout = 100;

	/* Perform register test on FALC */
	for (x=0;x<256;x++) {
	    t1_framer_out(dev, 0x14, x);
	    falcver = t1_framer_in(dev, 0x14);
	    if (falcver != x)
		    printk(KERN_DEBUG "Wrote '0x%02x' but read '0x%02x'\n", x, falcver);
	}
	falcver = t1_framer_in(dev ,0x4a);
	
	printk(KERN_INFO "d100m: bus %d, blade %d, slot %d, FALC version: %02x\n", 
														dev->bus_id,dev->blade_id,dev->slot_id,falcver);

	start_alarm(d1);
	return 0;
}

static void d100m_serial_setup(struct x200_dev* dev)
{
    struct x2d100m *d1 = dev->drv_data;

	printk(KERN_INFO "d100m: bus %d, blade %d, slot %d, Setting up global serial parameters for %s\n", 
												dev->bus_id,dev->blade_id,dev->slot_id,d1->spantype == TYPE_E1 ? "E1" : "T1");
	serial_setup(dev);
	
  if(debug)
  	printk(KERN_INFO "d100m: Successfully initialized serial bus for card\n");
}

static int d100m_software_init(struct x200_dev *dev)
{    
    struct x200* x2 = dev->bus;
    int retval;
    int x,y;
    struct x2d100m *d1 = dev->drv_data;

	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) {
		if (!(d1->chans[x] = kmalloc(sizeof(*d1->chans[x]), GFP_KERNEL))) {
			while (x) {
				kfree(d1->chans[--x]);
			}
			return -ENOMEM;
		}
		memset(d1->chans[x], 0, sizeof(*d1->chans[x]));
	}


	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) {
		if (!(d1->ec[x] = kmalloc(sizeof(*d1->ec[x]), GFP_KERNEL))) {
			while (x) {
				kfree(d1->ec[--x]);
			}
			for(y=0; y < (d1->spantype == TYPE_E1 ? 31 : 24); y++)
				kfree(d1->chans[y]);
				
			return -ENOMEM;
		}
		memset(d1->ec[x], 0, sizeof(*d1->ec[x]));
	}

  d100m_serial_setup(dev);

	sprintf(d1->span.name, "D100M1/%d", dev->slot_id);
  snprintf(d1->span.desc, sizeof(d1->span.desc) - 1, "OpenVox D100M (E1|T1) Card %d/%d", dev->bus->bus_id, dev->slot_id);
	
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	d1->ddev->manufacturer = "OpenVox";
	d1->ddev->devicetype = "OpenVox D100M Card.";
	d1->ddev->location = kasprintf(GFP_KERNEL,"X200 Bus %02d Blade %d Slot %02d",x2->bus_id, dev->parent->blade_id, dev->slot_id);
	if(!d1->ddev->location) {
			retval = -ENOMEM;
			goto d100m_init_one_exit_err0;
	}
#else
	d1->span.manufacturer = "OpenVox";
	dahdi_copy_string(d1->span.devicetype, "OpenVox D100M Card", sizeof(d1->span.devicetype));
	snprintf(d1->span.location, sizeof(d1->span.location) - 1,"X200 Bus %02d Blade %d Slot %02d", x2->bus_id, dev->parent->blade_id, dev->slot_id);
	d1->span.irq = x2->irq;
#endif	
	
		
	if (d1->spantype == TYPE_E1) {
		d1->span.channels = 31;
		d1->span.deflaw = DAHDI_LAW_ALAW;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,7,0)
		d1->span.spantype = SPANTYPE_DIGITAL_E1;
#else
		d1->span.spantype = "E1";
#endif
		d1->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4; // 
	} else {
		d1->span.channels = 24;
		d1->span.deflaw = DAHDI_LAW_MULAW;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,7,0)
		d1->span.spantype = SPANTYPE_DIGITAL_T1;
#else
		d1->span.spantype = "T1";
#endif
		d1->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
	}
	d1->span.chans = d1->chans;
	d1->span.flags = DAHDI_FLAG_RBS;	
	for (x=0;x<d1->span.channels;x++) {
		sprintf(d1->chans[x]->name, "D100M1/%d/%d", dev->slot_id, x + 1);
    d100m_chan_set_sigcap(&d1->span, x);
		d1->chans[x]->pvt = d1;
		d1->chans[x]->chanpos = x + 1;
	}

  d1->span.ops = &d100m_span_ops;
	
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	list_add_tail(&d1->span.device_node, &d1->ddev->spans);	
	retval = dahdi_register_device(d1->ddev, &dev->dev);
	if(retval){
		printk(KERN_NOTICE "Unable to register span with DAHDI\n");
		retval = -ENOMEM;
		goto d100m_init_one_exit_err0;
	}
#else
	init_waitqueue_head(&d1->span.maintq);
  if (dahdi_register(&d1->span, 0)) {
		printk(KERN_NOTICE "Unable to register span with DAHDI\n");
		retval = -ENOMEM;
		goto d100m_init_one_exit_err0;
	}
#endif
	
	//init ec channels
	if (x2->vpm_present)  {
    for (x=0;x<d1->span.channels;x++) {
    	if(init_vpm450m_chan(x2, d1->chans[x]->chanpos * 4 + dev->slot_id,dev->slot_id)){
    		retval = -1;
    		goto d100m_init_one_exit_err0;
    	}
    }
  }
	
	return 0;
	
d100m_init_one_exit_err0:
	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) 
		kfree(d1->chans[x]);

	for (x = 0; x < (d1->spantype == TYPE_E1 ? 31 : 24); x++) 
		kfree(d1->ec[x]);

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	kfree(d1->ddev->location);
#endif

	return retval;
}

int d100m_init_one(struct x200_dev *dev)
{
    struct x200* x2 = dev->bus;
    int retval = -ENODEV;
    struct x2d100m *d1 = NULL;

    d1 = kmalloc(sizeof(*d1), GFP_KERNEL);
    if(NULL==d1)
        return -ENOMEM;

    memset(d1, 0, sizeof(*d1));
    d1->dev = dev;
	
    x2->card_type[dev->blade_id][dev->slot_id] = DEV_TYPE_D100M; 
    dev->drv_data = d1;   
					
    retval = d100m_hardware_init(dev);
    if(retval) {
        goto d100m_init_err1;
    }    

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)          
    d1->ddev = dahdi_create_device();
    if(!d1->ddev){
    	  retval = -ENOMEM;
    	  goto d100m_init_err1;
    }
#endif

    retval = d100m_software_init(dev);
    if(retval)
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
    	goto d100m_init_err0;
#else
			goto d100m_init_err1;
#endif
	  
	  d1->exist = 1;
	  return 0;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)  
d100m_init_err0:	  
	  dahdi_free_device(d1->ddev);
#endif
	  
d100m_init_err1:				
		kfree(d1);
		dev->drv_data = NULL;
		return retval;
}

void d100m_remove_one(struct x200_dev *dev)
{
    struct x2d100m *d1 = NULL;

    d1 = dev->drv_data;
		if(NULL==d1) {
			return;
		}

		if(!d1->exist) {
			dev->drv_data = NULL;
			dev->ops = NULL;
			return;
		}

    if (!d1->usecount)
			d100m_release(d1);
		else
			d1->dead = 1;

    dev->drv_data = NULL;
    dev->ops = NULL;
		
		dev->bus->checktiming = 1;
}

static void d100m_check_alarms(struct x2d100m *d1)
{
	unsigned char c,d;
	int alarms;
	int x,j;
	//unsigned long flags;
    struct x200_dev* dev = d1->dev;
    static int cnt = 0;
    struct x200* x2 = d1->dev->bus;

	if (!(d1->span.flags & DAHDI_FLAG_RUNNING))
		return;

	c = t1_framer_in(dev, 0x4c);  /* Framer Receive Status Register 0--FRS0 */
	d = t1_framer_in(dev, 0x4d);  /* FRS1 */

  
    if(debug) {
        cnt++;
        if( (cnt%100) == 0 || (cnt%100) == 1 )
            printk("D100M: Card %d FRS0 is 0x%x, FRS1 is 0x%x\n", dev->slot_id, c, d);
    }
		
    //spin_lock_irqsave(&dev->bus->lock, flags);
	/* Assume no alarms */
	alarms = 0;

	/* And consider only carrier alarms */
	d1->span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);

	if (d1->spantype == TYPE_E1) {
		if (c & 0x04) { /* NMF is only valid when CRC4 is opened */
			/* No multiframe found, force RAI high after 400ms only if
			   we haven't found a multiframe since last loss
			   of frame */
			if (!(d1->spanflags & FLAG_NMF)) {
				t1_framer_out(dev, 0x20, 0x9f | 0x20);	/* LIM0: Force RAI High */
				d1->spanflags |= FLAG_NMF;
                printk(KERN_DEBUG "D100M: NMF workaround on!\n");
			}
			t1_framer_out(dev, 0x1e, 0xc3);	/* Reset to CRC4 mode */
			t1_framer_out(dev, 0x1c, 0xf2);	/* Force Resync */
			t1_framer_out(dev, 0x1c, 0xf0);	/* Force Resync */
		} else if (!(c & 0x02)) {
			if ((d1->spanflags & FLAG_NMF)) {
				t1_framer_out(dev, 0x20, 0x9f);	/* LIM0: Clear forced RAI */
				d1->spanflags &= ~FLAG_NMF;
				printk(KERN_DEBUG "D100M: NMF workaround off!\n");
			}
		}
	} else {
		/* Detect loopup code if we're not sending one */
		if ((!d1->span.mainttimer) && (d & 0x08)) {
			/* Loop-up code detected */
			if ((d1->loopupcnt++ > 80)  && (d1->span.maintstat != DAHDI_MAINT_REMOTELOOP)) {
				t1_framer_out(dev, 0x36, 0x08);	/* LIM0: Disable any local loop */
				t1_framer_out(dev, 0x37, 0xf6 );	/* LIM1: Enable remote loop */
				d1->span.maintstat = DAHDI_MAINT_REMOTELOOP;
			}
		} else
			d1->loopupcnt = 0;
		/* Same for loopdown code */
		if ((!d1->span.mainttimer) && (d & 0x10)) {
			/* Loop-down code detected */
			if ((d1->loopdowncnt++ > 80)  && (d1->span.maintstat == DAHDI_MAINT_REMOTELOOP)) {
				t1_framer_out(dev, 0x36, 0x08);	/* LIM0: Disable any local loop */
				t1_framer_out(dev, 0x37, 0xf0 );	/* LIM1: Disable remote loop */
				d1->span.maintstat = DAHDI_MAINT_NONE;
			}
		} else
			d1->loopdowncnt = 0;
	}

	if (d1->span.lineconfig & DAHDI_CONFIG_NOTOPEN) {
		for (x=0,j=0;x < d1->span.channels;x++)
			if ((d1->span.chans[x]->flags & DAHDI_FLAG_OPEN) ||
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,5,0) 
			     dahdi_have_netdev(d1->span.chans[x]))
#else
					 (d1->span.chans[x]->flags & DAHDI_FLAG_NETDEV))
#endif
				j++;
		if (!j)
			alarms |= DAHDI_ALARM_NOTOPEN;
	}

	if (c & 0xa0) {
		if (d1->alarmcount >= alarmdebounce) {
				alarms |= DAHDI_ALARM_RED;
		} else
			d1->alarmcount++;
	} else
		d1->alarmcount = 0;
		
	if (c & 0x4)
		alarms |= DAHDI_ALARM_BLUE;

	//check sync timing source		
	if((!d1->span.alarms &&( alarms))||
		 (d1->span.alarms && (!alarms)) ) //
	{
			x2->checktiming = 1;	
	}
	
	/* Keep track of recovering */
	if ((!alarms) && d1->span.alarms)
		d1->alarmtimer = DAHDI_ALARMSETTLE_TIME;
	if (d1->alarmtimer)
		alarms |= DAHDI_ALARM_RECOVER;

	/* If receiving alarms, go into Yellow alarm state */
	if (alarms && !(d1->spanflags & FLAG_SENDINGYELLOW)) {
		unsigned char fmr4;
#if 1
		printk(KERN_INFO "d100m: Bus %d Blade %d Slot %d Setting yellow alarm\n", dev->bus_id, dev->blade_id, dev->slot_id);
#endif
		/* We manually do yellow alarm to handle RECOVER and NOTOPEN, otherwise it's auto anyway */
		fmr4 = t1_framer_in(dev, 0x20);
		t1_framer_out(dev, 0x20, fmr4 | 0x20);
		d1->spanflags |= FLAG_SENDINGYELLOW;
	} else if ((!alarms) && (d1->spanflags & FLAG_SENDINGYELLOW)) {
		unsigned char fmr4;
#if 1
		printk(KERN_INFO "d100m: Bus %d Blade %d Slot %d Clearing yellow alarm\n", dev->bus_id, dev->blade_id, dev->slot_id);
#endif
		/* We manually do yellow alarm to handle RECOVER  */
		fmr4 = t1_framer_in(dev, 0x20);
		t1_framer_out(dev, 0x20, fmr4 & ~0x20);
		d1->spanflags &= ~FLAG_SENDINGYELLOW;
	}
	
	/* Re-check the timing source when we enter/leave alarm, not withstanding
	   yellow alarm */
	if (c & 0x10)
		alarms |= DAHDI_ALARM_YELLOW;
	if (d1->span.mainttimer || d1->span.maintstat)
		alarms |= DAHDI_ALARM_LOOPBACK;
	d1->span.alarms = alarms;

	dahdi_alarm_notify(&d1->span);
}

static void d100m_transmitprep(struct x2d100m* d1)
{
	int x,y;
	int pos;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	_dahdi_transmit(&d1->span);
#else
	dahdi_transmit(&d1->span);
#endif

	/* TODO: maybe need special operate for T1 */
	if (tdm_mode & 0x40) {   //pakdata
	    //
	} else {
    	for (x=0;x<d1->span.channels;x++) {
        for (y=0;y<DAHDI_CHUNKSIZE;y++) {
    			pos = y * 128 + (x+1) * 4 + d1->dev->slot_id;
    			/* Put channel number as outgoing data */
    			d1->dev->bus->current_writechunk[pos] = d1->chans[x]->writechunk[y];
    		}
    	}
    }
}

static void d100m_receiveprep(struct x2d100m* d1)
{
	volatile unsigned char *rxbuf;
	int x;
	int y;

  rxbuf = d1->dev->bus->current_readchunk;

	for (y=0;y<DAHDI_CHUNKSIZE;y++) {
		for (x=0;x<d1->span.channels;x++) {
			d1->chans[x]->readchunk[y] = rxbuf[128 * y + (x+1) * 4 + d1->dev->slot_id];	
		}
	}
	for (x=0;x<d1->span.channels;x++) {
		dahdi_ec_chunk(d1->chans[x], d1->chans[x]->readchunk, d1->ec_chunk2[x]);
		memcpy(d1->ec_chunk2[x],d1->ec_chunk1[x],DAHDI_CHUNKSIZE);
		memcpy(d1->ec_chunk1[x],d1->chans[x]->writechunk,DAHDI_CHUNKSIZE);
	}
	
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	_dahdi_receive(&d1->span);
#else
	dahdi_receive(&d1->span);
#endif
}

static int altab[] = {
0, 0, 0, 1, 2, 3, 4, 6, 8, 9, 11, 13, 16, 18, 20, 22, 24, 25, 27, 28, 29, 30, 31, 31, 32, 31, 31, 30, 29, 28, 27, 25, 23, 22, 20, 18, 16, 13, 11, 9, 8, 6, 4, 3, 2, 1, 0, 0,
};

static inline void __handle_leds(struct x2d100m *d1)
{
    struct x200* x2 = d1->dev->bus;
    unsigned long flags;
    //struct x200_dev *dev = d1->dev;

    d1->blinktimer++;
    if (d1->blinktimer > 0xf) {
    	d1->blinktimer = 0x0 ;
    	d1->alarmpos++;
    	if (d1->alarmpos >= (sizeof(altab) / sizeof(altab[0])))
    		d1->alarmpos = 0;
    }
		
		spin_lock_irqsave(&x2->lock, flags);
    if (d1->span.flags & DAHDI_FLAG_RUNNING) {
    	if ((d1->span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE)) || d1->alarmcount) {
    		if (d1->blinktimer == (altab[d1->alarmpos] >> 1)) {
    			__x200_set_led(x2, d1->dev->slot_id*2, LED_RED);
    		}
    		if (d1->blinktimer == 0xf) {
    			__x200_set_led(x2, d1->dev->slot_id*2, LED_OFF);
    		}
    	} else if (d1->span.alarms & DAHDI_ALARM_YELLOW) {
    		/* Yellow Alarm */
    		__x200_set_led(x2, d1->dev->slot_id*2, LED_YELLOW);
    	} else if ( d1->span.maintstat) {
    		if (d1->blinktimer == (altab[d1->alarmpos] >> 1)) {
    			__x200_set_led(x2, d1->dev->slot_id*2, LED_GREEN);
    		}
    		if (d1->blinktimer == 0xf) {
    			__x200_set_led(x2, d1->dev->slot_id*2, LED_OFF);
    		}
    	} else {
    		/* No Alarm */
    		__x200_set_led(x2, d1->dev->slot_id*2, LED_GREEN);
    	}
    } else {
        __x200_set_led(x2, d1->dev->slot_id*2, LED_OFF);
    }
    spin_unlock_irqrestore(&x2->lock, flags);
}

int d100m_get_clk_src(struct x200_dev *dev, int * syncpos, int * port)
{ 
    struct x2d100m *d1 = dev->drv_data;       
    *port = 0 ;
    *syncpos = 0 ;
    if ((d1->span.flags & DAHDI_FLAG_RUNNING) &&  
    	!(d1->span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN)) && 
    	(d1->sync > 0 ) ) {
        *syncpos = d1->sync ; 
    }   
    return 0 ;         
}

int d100m_set_clk_src(struct x200_dev *dev, int syncpos, int port)
{
    struct x200* x2 = dev->bus;
    int card_type = x2->card_type[dev->blade_id][dev->slot_id] ;
    
    printk("d100m: line %d,  slot:%d,  card_type:%d, syncpos:%d set as master. \n",__LINE__, dev->slot_id, card_type,syncpos );    
    return 0 ;  
}

static int d100m_unset_clk_src(struct x200_dev *dev)
{
    struct x200* x2 = dev->bus;
    int card_type = x2->card_type[dev->blade_id][dev->slot_id] ;
    
    if(dev == x2->clock_master)
    	x200_set_clock_master(x2, NULL);
    
    printk("d100m: line %d,  slot:%d car_type is %d unset as master. \n",__LINE__, dev->slot_id, card_type );
    return 0 ;  
}

int d100m_interrupt(int intcount, struct x200_dev *dev)
{
    int x;
    struct x2d100m* d1 = dev->drv_data;

    d100m_do_counters(d1);
    d100m_receiveprep(d1);
		d100m_transmitprep(d1);
		__handle_leds(d1);
  	x = (intcount + dev->slot_id) & 0x07;
		switch(x) {
	    case 0:
	    case 1:
	    	break;
	    case 2:
        d100m_check_sigbits(d1);
		    break;
		  case 4:
		    /* Check alarms 1/4 as frequently */
		    if (!(intcount & 0x30))
			    d100m_check_alarms(d1);
		    break;
		}
	
		return 0;	
}

struct ints_ops d100m_ops={
		.irq_handler = d100m_interrupt,
    .get_clk_src = d100m_get_clk_src,
    .set_clk_src = d100m_set_clk_src,
    .unset_clk_src = d100m_unset_clk_src,
};

static int x200_dev_probe(struct device *dev)
{
		int ret;
		struct x200_dev *x2dev = to_x200_dev(dev);
		ret = d100m_init_one(x2dev);
		if(!ret){
				x2dev->ops = &d100m_ops;
		}
		return ret;
}

static int x200_dev_remove(struct device *dev)
{
		struct x200_dev *x2dev = to_x200_dev(dev);
		d100m_remove_one(x2dev);
		return 0;
}

static int __init d100m_init(void)
{
    int i;
    struct bus_type *x200bus; 
     
    for(i=0;i<interface_num;i++){
    		x200bus = get_bus(i);
    		x200_dev_driver[i].name = "d100m_driver";
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

static void __exit d100m_cleanup(void)
{
		int i;
		
		for(i=0;i<interface_num;i++){
				driver_unregister(&x200_dev_driver[i]);
		}
}

module_param(alarmdebounce,     int,    0600);
module_param(debug,     		int,    0600);
module_param(t1e1override,      int,  	0600);
module_param(loopback,  		int,    0600);


MODULE_DESCRIPTION("OpenVox D100M Driver");
MODULE_AUTHOR("Miao Lin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(d100m_init);
module_exit(d100m_cleanup);

