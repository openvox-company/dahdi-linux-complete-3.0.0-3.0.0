/*
 * OpenVox FXO module on x200 hybrid card
 *
 * Written by Miao Lin<lin.miao@openvox.cn>
 *
 * Copyright (C) 2011-2012 OpenVox Communication Co Ltd..
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
#include <linux/errno.h>
#include <asm/io.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <dahdi/kernel.h>
#include <dahdi/wctdm_user.h>

#include "fxo_modes.h"
#include "x200_hal.h"
#include "x2fxo200m.h"
#include "ec3000.h"

extern int vpmsupport;
extern int tdm_mode;
extern int spi_fifo_mode;
extern int interface_num;

int debug = 0;
int alawoverride = 1;
int fastpickup = 0;
int fxotxgain = 0;
int fxorxgain = 0;
int fxofullscale = 0;   /* fxo full scale tx/rx, register 30, acim */
int cidbuflen = 3000;   /* in msec, default 3000 */
int cidtimeout = 6*1000;    /* in msec, default 7000 */

int _opermode = 0;
char *opermode = "FCC";
int boostringer = 0;
int fxshonormode = 0;

unsigned int battdebounce;
unsigned int battalarm;
unsigned int battthresh;
int fwringdetect = 0;


int ringdebounce = DEFAULT_RING_DEBOUNCE;
int ringoncount = DEFAULT_RINGON_COUNT;
int ringoffcount = DEFAULT_RINGOFF_COUNT;
/* fastringoffhook is used to let FXO port send DAHDI_EVENT_RINGOFFHOOK ealier before
    the actual ring signal stop.
	Asterisk use DAHDI_EVENT_RINGOFFHOOK to start callerid detect,
    But DAHDI_EVENT_RINGOFFHOOK sent by DAHDI driver have about 250ms delay than actual ring signal.
    In some case, CID is start sent only 70ms after ring signal stop. this usually cause asterisk
    can not get start part of the callerid signal.
    fastringoffhook is ms after ring begin timer, if it is not zero, DAHDI will generate
    DAHDI_EVENT_RINGOFFHOOK event after fastringoffhook ms when ring begin detected
    fastringoffhook is a temporary solution, final solution will modify ring detect
    state machine, to shorten the detect period.
    */
int fastringoffhook = 0;

/* during ring, polarity also change, final polarity detect must consider ring signal.*/
int polaritydebounce = DEFAULT_POLARITY_DEBOUNCE; /* default 64ms */
/* >0 : freeze polarity detect when ringing. when fxo->wasring == 1 */
int freezepolaritywhilering = 1;

/* IF fxo connect to a two-way charge gateway(FXS), FXO must igore two-way charge signal--polarity reverse signal, sent after offhook,
 * otherwise fxo goes to onhook.
 */
static int twowaychargeflag = 0; 

struct device_driver x200_dev_driver[WC_MAX_INTERFACES];


const struct dahdi_echocan_features vpm_ec_features = {
    .NLP_automatic = 1,
    .CED_tx_detect = 1,
    .CED_rx_detect = 1,
};

void a200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

const struct dahdi_echocan_ops vpm_ec_ops = {
#if DAHDI_VERSION_CODE < VERSION_CODE(2,5,0)
	.name = "FXO200M VPM",
#endif
    .echocan_free = a200m_echocan_free,
};

int a200m_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
    int channel;
    struct x200fxo *a200m = chan->pvt;
    struct x200* x2 = a200m->dev->bus;
    
    if (!vpmsupport || !x2->vpm_present ) {
        return -ENODEV;
    }

    if (ecp->param_count > 0) {
        printk(KERN_ERR "error at line:%d,  echo canceller does not support parameters; failing request\n",__LINE__);
        return -EINVAL;
    }

    if (x2->vpm450m) {
        *ec = a200m->ec[chan->chanpos-1];
        (*ec)->ops = &vpm_ec_ops;
        (*ec)->features = vpm_ec_features;
    } else {
        printk(KERN_ERR "error at line:%d\n",__LINE__);
        return -EINVAL;
    }

    channel = (chan->chanpos * 4) + a200m->dev->slot_id  ;
    vpm450m_setec(x2, channel, a200m->dev->slot_id, ecp->tap_length);
    return 0;
}
void a200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
    int channel;
    struct x200fxo *a200m = chan->pvt;
    struct x200* x2 = a200m->dev->bus;
    memset(ec, 0, sizeof(*ec));
    channel = (chan->chanpos * 4) + a200m->dev->slot_id  ;
    if (x2->vpm450m) {
        //printk("fxo200m a200m_echocan_free line:%d, channel:%d.\n", __LINE__, channel);
        vpm450m_setec(x2, channel, a200m->dev->slot_id, 0);
    }
}

void wait_just_a_bit(int foo)
{
        long newjiffies;
        newjiffies = jiffies + foo;
        while(jiffies < newjiffies);
}


/*********************************************************************
 * Set the hwgain on the analog modules
 *
 * card = the card position for this module (0-23)
 * gain = gain in dB x10 (e.g. -3.5dB  would be gain=-35)
 * tx = (0 for rx; 1 for tx)
 *
 *******************************************************************/
int si3050_set_hwgain(struct x200_dev *dev, unsigned int card, __s32 gain, __u32 tx)
{
        if (tx) {
            if (debug) {
                printk("setting FXO tx gain for card=%d to %d\n", card, gain);
            }
            if (gain >=  -150 && gain <= 0) {
                fxo200m_write_reg(dev, 38, 16 + (gain/-10), card);
                fxo200m_write_reg(dev, 40, 16 + (-gain%10), card);
            } else if (gain <= 120 && gain > 0) {
                fxo200m_write_reg(dev, 38, gain/10, card);
                fxo200m_write_reg(dev, 40, (gain%10), card);
            } else {
                printk("FXO tx gain is out of range (%d)\n", gain);
                return -1;
            }
        } else { /* rx */
            if (debug) {
                printk("setting FXO rx gain for card=%d to %d\n", card, gain);
            }
            if (gain >=  -150 && gain <= 0) {
                fxo200m_write_reg(dev, 39, 16+ (gain/-10), card);
                fxo200m_write_reg(dev, 41, 16 + (-gain%10), card);
            } else if (gain <= 120 && gain > 0) {
                fxo200m_write_reg(dev, 39, gain/10, card);
                fxo200m_write_reg(dev, 41, (gain%10), card);
            } else {
                printk("FXO rx gain is out of range (%d)\n", gain);
                return -1;
            }
        }
        return 0;
}

int si3050_init_voicedaa(struct x200_dev *dev, unsigned int chanx)
{
        unsigned char reg16=0, reg26=0, reg30=0, reg31=0;
        long newjiffies;

        /* Software reset */
        fxo200m_write_reg(dev, 1, 0x80, chanx);

        /* Wait just a bit */
        wait_just_a_bit(HZ/10);

        //fxo200m_write_reg(dev, 10, 1, chanx); // digital loopback, miaolin

        /* Enable PCM, ulaw */
        if (alawoverride)
            fxo200m_write_reg(dev, 33, 0x20, chanx);
        else
            fxo200m_write_reg(dev, 33, 0x28, chanx);

        /* Set On-hook speed, Ringer impedence, and ringer threshold */
        reg16 |= (fxo_modes[_opermode].ohs << 6);
        reg16 |= (fxo_modes[_opermode].rz << 1);
        reg16 |= (fxo_modes[_opermode].rt);
        fxo200m_write_reg(dev, 16, reg16, chanx);

        if(fwringdetect) {
            /* Enable ring detector full-wave rectifier mode */
            fxo200m_write_reg(dev, 18, 2, chanx);
            fxo200m_write_reg(dev, 24, 0, chanx);
        } else {
            /* Set to the device defaults */
        		fxo200m_write_reg(dev, 18, 0, chanx);
        		fxo200m_write_reg(dev, 24, 0x19, chanx);
        }

        /* Set DC Termination:
           Tip/Ring voltage adjust, minimum operational current, current limitation */
        reg26 |= (fxo_modes[_opermode].dcv << 6);
        reg26 |= (fxo_modes[_opermode].mini << 4);
        reg26 |= (fxo_modes[_opermode].ilim << 1);
        fxo200m_write_reg(dev, 26, reg26, chanx);

        /* Set AC Impedence */
        reg30 = (fxofullscale==1) ? (fxo_modes[_opermode].acim|0x10) :  (fxo_modes[_opermode].acim);
        fxo200m_write_reg(dev, 30, reg30, chanx);

        /* Misc. DAA parameters */
        if (fastpickup)
            reg31 = 0xb3;
        else
            reg31 = 0xa3;
        reg31 |= (fxo_modes[_opermode].ohs2 << 3);
        fxo200m_write_reg(dev, 31, reg31, chanx);

        /* Set Transmit/Receive timeslot */
        //printk("set module-%d chip[%d] to %d\n", dev->slot_id, chanx, (chanx%2) * 8 + (dev->slot_id * 16));
        fxo200m_write_reg(dev, 34, (dev->slot_id+(1+chanx)*4)*8, chanx);
        fxo200m_write_reg(dev, 35, 0, chanx);
        fxo200m_write_reg(dev, 36, (dev->slot_id+(1+chanx)*4)*8, chanx);
        fxo200m_write_reg(dev, 37, 0, chanx);

        /* Enable ISO-Cap */
        fxo200m_write_reg(dev, 6, 0x00, chanx);

        if (fastpickup)
            fxo200m_write_reg(dev, 17, fxo200m_read_reg(dev,17,chanx)|0x20, chanx);

        /* Wait 1000ms for ISO-cap to come up */
        newjiffies = jiffies;
        newjiffies += 2 * HZ;
        while((jiffies < newjiffies) && !(fxo200m_read_reg(dev, 11, chanx) & 0xf0)) {
            wait_just_a_bit(HZ/10);
        }

        if (!(fxo200m_read_reg(dev, 11, chanx) & 0xf0)) {
            printk("VoiceDAA did not bring up ISO link properly!\n");
            return -1;
        }
        if (debug) {
            printk("ISO-Cap is now up, line side: %02x rev %02x\n",
                    fxo200m_read_reg(dev, 11, chanx) >> 4,
                    (fxo200m_read_reg(dev, 13, chanx) >> 2) & 0xf);
        }
        /* Enable on-hook line monitor */
        fxo200m_write_reg(dev, 5, 0x08, chanx);

        /* Take values for fxotxgain and fxorxgain and apply them to module */
        si3050_set_hwgain(dev, chanx, fxotxgain, 1);
        si3050_set_hwgain(dev, chanx, fxorxgain, 0);

        /* NZ -- crank the tx gain up by 7 dB */
        if (!strcmp(fxo_modes[_opermode].name, "NEWZEALAND")) {
            printk("Adjusting gain\n");
            si3050_set_hwgain(dev, chanx, 7, 1);
        }

        if(debug) {
            printk("DEBUG fxotxgain:%i.%i fxorxgain:%i.%i\n",
                (fxo200m_read_reg(dev, 38, chanx)/16) ? -(fxo200m_read_reg(dev, 38, chanx) - 16) : (fxo200m_read_reg(dev, 38, chanx)),
                (fxo200m_read_reg(dev, 40, chanx)/16) ? -(fxo200m_read_reg(dev, 40, chanx) - 16) : (fxo200m_read_reg(dev, 40, chanx)),
                (fxo200m_read_reg(dev, 39, chanx)/16) ? -(fxo200m_read_reg(dev, 39, chanx) - 16) : (fxo200m_read_reg(dev, 39, chanx)),
                (fxo200m_read_reg(dev, 41, chanx)/16) ? -(fxo200m_read_reg(dev, 41, chanx) - 16) : (fxo200m_read_reg(dev, 41, chanx)));
        }

        /* battery state still unknown */
        return 0;
}

void fxo200m_release(struct x200fxo *a200m)
{
    unsigned int x;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		dahdi_unregister_device(a200m->ddev);
#else
		dahdi_unregister(&a200m->span);
#endif

    for (x = 0; x < CHANS_PER_MODULE; x++) {
        kfree(a200m->ec[x]);
    }

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)    
		kfree(a200m->ddev->location);
		dahdi_free_device(a200m->ddev);
#endif
    printk("fxo200m: bus %d, blade %d, slot %d Freed a OpenVox fxo200m card.\n",a200m->dev->bus->bus_id, a200m->dev->parent->blade_id, a200m->dev->slot_id);
    kfree(a200m);
    a200m = NULL;
    
}

static int _fxo200m_open(struct dahdi_chan *chan)
{
    struct x200fxo *a200m = chan->pvt;
    if(debug)
        printk("fxo200m: fxo module_open called\n");
    if (a200m->dead)
                return -ENODEV;
    a200m->usecount++;
    return 0;
}
static int fxo200m_open(struct dahdi_chan *chan)
{
	unsigned long flags;
	int res;
	spin_lock_irqsave(&chan->lock, flags);
	res = _fxo200m_open(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
	return res;
}

static int fxo200m_close(struct dahdi_chan *chan)
{
    struct x200fxo *a200m = chan->pvt;
    if(debug)
        printk("fxo200m: fxo module_close called\n");

    a200m->usecount--;
    /* If we're dead, release us now */
    if (!a200m->usecount && a200m->dead)
        fxo200m_release(a200m);

    return 0;
}

static int fxo200m_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
    struct wctdm_stats stats;
    struct wctdm_regs regs;
    struct dahdi_hwgain hwgain;
    struct wctdm_regop regop;
    int x;
    struct x200fxo *a200m=chan->pvt;

    switch(cmd) {
        case WCTDM_GET_REGS:
            for (x=0;x<NUM_FXO_REGS;x++) {
                regs.direct[x] = fxo200m_read_reg(a200m->dev, chan->chanpos - 1, x);
            }
            if (copy_to_user((__user void *)data, &regs, sizeof(regs)))
                return -EFAULT;
            break;
        case WCTDM_GET_STATS:
            stats.tipvolt = (signed char)fxo200m_read_reg(a200m->dev, 29,  chan->chanpos - 1) * 1000;
            stats.ringvolt = (signed char)fxo200m_read_reg(a200m->dev, 29, chan->chanpos - 1) * 1000;
            stats.batvolt = (signed char)fxo200m_read_reg(a200m->dev, 29,  chan->chanpos - 1) * 1000;
            if (copy_to_user((__user void *)data, &stats, sizeof(stats)))
                return -EFAULT;
            break;
        case WCTDM_SET_REG:
        	  if (copy_from_user(&regop, (__user void *)data, sizeof(regop))) {
			      	return -EFAULT;
						}
						if (regop.indirect) {
								return -EINVAL;
						} else {
								regop.val &= 0xff;
								printk(KERN_INFO "Setting direct %d to %04x on %d\n", regop.reg, regop.val, chan->chanpos);
								fxo200m_write_reg(a200m->dev, regop.reg, regop.val, chan->chanpos - 1);
						}
						break;
        case DAHDI_SET_HWGAIN:
            if (copy_from_user(&hwgain, (__user void *) data, sizeof(hwgain))) {
                return -EFAULT;
            }
            si3050_set_hwgain(a200m->dev, chan->chanpos-1, hwgain.newgain, hwgain.tx);
            if (debug) {
                printk(KERN_DEBUG "Setting hwgain on channel %d to %d for %s direction\n",
                    chan->chanpos-1, hwgain.newgain, hwgain.tx ? "tx" : "rx");
            }
            break;
        default:
            return -ENOTTY;
    }
    return 0;
}

static int fxo200m_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
    struct x200fxo *a200m = chan->pvt;
    struct x200_dev *dev = a200m->dev;
    unsigned int  channo=chan->chanpos-1;
    switch(txsig) {
        case DAHDI_TXSIG_START:
            a200m->fxoconfig[channo].callout = 1;
        case DAHDI_TXSIG_OFFHOOK:
            /* Set fxo.wasringing to 0, otherwise if offhook in ringing time and FXS onhook first,
             * FXO can't check the polarity signal sent by FXS to tell FXO that FXS has onhook,
             * because when freezepolaritywhilering is 1 FXO don't check polarity when ringing.
             */
            a200m->fxoconfig[channo].wasringing = 0;
            a200m->fxoconfig[channo].offhook = 1;
            a200m->fxoconfig[channo].polaritycountwhenoffhook = 0;
            fxo200m_write_reg(dev, 5, 0x9, channo);
            break;
        case DAHDI_TXSIG_ONHOOK:
            a200m->fxoconfig[channo].offhook =0;
            a200m->fxoconfig[channo].callout = 0;
            fxo200m_write_reg(dev, 5, 0x08, channo);
            break;
        default:
            printk(KERN_NOTICE "fxo200m: Can't set tx state to %d\n", txsig);
    }
    return 0;
}

static int fxo200m_watchdog(struct dahdi_span *span, int event)
{
        printk(KERN_INFO "fxo200m: watch dog!\n");
        return 0;
}

static const struct dahdi_span_ops a200m_span_ops = {
    .owner = THIS_MODULE,
    .hooksig = fxo200m_hooksig,
    .open = fxo200m_open,
    .close = fxo200m_close,
    .ioctl = fxo200m_ioctl,
    .watchdog = fxo200m_watchdog,
    .echocan_create = a200m_echocan_create,
};

static int fxo200m_hardware_init(struct x200_dev *dev)
{
    int i, x;
    struct x200* x2 = dev->bus;
    unsigned long flags;
    //struct x200fxo *a200m = x200_get_drvdata(dev);

    for(i=0; i<CHANS_PER_MODULE; i++)
    {
        si3050_init_voicedaa(dev, i);
        
        spin_lock_irqsave(&x2->lock, flags);
        __x200_set_led(dev->bus, dev->slot_id*CHANS_PER_MODULE+i, LED_GREEN);
        spin_unlock_irqrestore(&x2->lock, flags);
        
        if(debug) {
            printk("!below: debugging info for si3050! \n");
            // dump the chip[0] and chip[1] registers
            for (x=0; x<64; x++)
            {
                if (! (x & 15))
                   printk("%02x:", x);
                printk(" %02x", fxo200m_read_reg(dev, x, i));
                if ((x & 15) == 15)
                   printk("\n");
            }
        }
    }
    return 0;
}

static int fxo200m_software_init(struct x200_dev *dev)
{
    struct x200fxo *a200m = dev->drv_data;
    struct x200* x2 = dev->bus;
    int x,channel;
    int retval;

    for (x=0; x < sizeof(a200m->chans)/sizeof(a200m->chans[0]); ++x) {
			a200m->chans[x] = &a200m->_chans[x];
		}

    for (x = 0; x <CHANS_PER_MODULE; x++) {
        if (!(a200m->ec[x] = kmalloc(sizeof(*a200m->ec[x]), GFP_KERNEL))) {
            while (x) {
                kfree(a200m->ec[--x]);
            }
            return -ENOMEM;
        }
        memset(a200m->ec[x], 0, sizeof(*a200m->ec[x]));
    }

    sprintf(a200m->span.name, "FXO200M/%d", dev->slot_id);
    snprintf(a200m->span.desc, sizeof(a200m->span.desc) - 1, "OpenVox FXO200M Card %d", dev->slot_id);

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)    
    a200m->ddev->manufacturer = "OpenVox";
    a200m->ddev->devicetype = "OpenVox FXO200M Card";
    a200m->ddev->location = kasprintf(GFP_KERNEL,"X200 Bus %02d Blade %d Slot %02d", 
    																						x2->bus_id, dev->parent->blade_id, dev->slot_id);
   
    if(!a200m->ddev->location){
    		retval = -ENOMEM;
    		goto a200m_init_err;
    }
#else
		a200m->span.manufacturer = "OpenVox";
    dahdi_copy_string(a200m->span.devicetype, "OpenVox FXO200M Card", sizeof(a200m->span.devicetype));
    snprintf(a200m->span.location, sizeof(a200m->span.location) - 1, "X200 Bus %02d Blade %d Slot %02d", x2->bus_id, dev->parent->blade_id, dev->slot_id);
    a200m->span.irq = x2->irq;
#endif
    
    a200m->span.channels = CHANS_PER_MODULE;
    if (alawoverride)
        a200m->span.deflaw = DAHDI_LAW_ALAW;
    else
        a200m->span.deflaw = DAHDI_LAW_MULAW;
        
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,7,0)		
		a200m->span.spantype = SPANTYPE_ANALOG_FXO;
#endif
    a200m->span.chans = a200m->chans;
    a200m->span.flags = DAHDI_FLAG_RBS;
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)    
    init_waitqueue_head(&a200m->span.maintq);
#endif

    for (x=0;x<a200m->span.channels;x++) {
        sprintf(a200m->chans[x]->name, "FXO200M");
        a200m->chans[x]->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
        a200m->chans[x]->sigcap |= DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS | DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
        a200m->chans[x]->pvt = a200m;
        a200m->chans[x]->chanpos = x + 1;
    }
    a200m->span.ops = &a200m_span_ops;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		list_add_tail(&a200m->span.device_node, &a200m->ddev->spans);
		retval = dahdi_register_device(a200m->ddev, &dev->dev);
		if(retval){
				printk(KERN_NOTICE "Unable to register span with DAHDI\n");
				goto a200m_init_loc_err;
		}
#else
		if (dahdi_register(&a200m->span, 0)) {
        printk(KERN_NOTICE "Unable to register span with DAHDI\n");
        retval = -1;
        goto a200m_init_err;
    }
#endif	
    //set signalling capability
    for (x=0;x<a200m->span.channels;x++) {
        a200m->chans[x]->sigcap = DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS | DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
    }
    
    //init ec channels
    if(x2->vpm_present){
    	for (x=0;x<a200m->span.channels;x++){
    		channel = dev->slot_id + a200m->chans[x]->chanpos*4;
				if(init_vpm450m_chan(x2, channel,dev->slot_id)){
    			return -2;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
					goto a200m_init_loc_err;
#else
					goto a200m_init_err;
#endif
    		}
    	}
		}
		
    return 0;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
a200m_init_loc_err:
		kfree(a200m->ddev->location);
#endif	

a200m_init_err:
    for (x = 0; x < CHANS_PER_MODULE; x++) 
        kfree(a200m->ec[x]);    
 
		return retval;
}

int fxo200m_init_one(struct x200_dev *dev)
{
    struct x200* x2 = dev->bus;
    int retval = -ENODEV;
    struct x200fxo *a200m = NULL;
    int i=0;

    a200m = kmalloc(sizeof(*a200m), GFP_KERNEL);
    if(NULL==a200m)
        return -ENOMEM;

    memset(a200m, 0, sizeof(*a200m));
    a200m->dev = dev;
    dev->drv_data = a200m;

    //printk("fxo200m: 2 tdm slots allocated in instance %d\n" , dev->instance_id);
    x2->card_type[dev->blade_id][dev->slot_id] = DEV_TYPE_FXO200M ;

    retval = 0;
    i = fxo200m_hardware_init(dev);
    if(i) {
        retval = i;
        goto a200m_init_one_err0;
    }
		
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		a200m->ddev = dahdi_create_device();
		if(!a200m->ddev){
			retval = -ENOMEM;
			goto a200m_init_one_err0;
		}
#endif
			
    retval = fxo200m_software_init(dev);
    if(retval)
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
    	goto a200m_init_software_err;
#else
			goto a200m_init_one_err0;
#endif

		init_busydetect(a200m,opermode);
    a200m->exist = 1;

    printk("fxo200m driver loaded : init device on bus %d blade %d slot %d successfully.\n",dev->bus_id,dev->blade_id,dev->slot_id);
        
    return 0;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
a200m_init_software_err:
	  dahdi_free_device(a200m->ddev);
#endif

a200m_init_one_err0:
	  kfree(a200m);
	  dev->drv_data = NULL;
		return retval;
}

void fxo200m_remove_one(struct x200_dev *dev)
{
    int i;
    struct x200fxo *a200m;
    struct x200* x2 = dev->bus;
		unsigned long flags;
		
		a200m = dev->drv_data;

    if(NULL==a200m) {
        return;
    }

    if(!a200m->exist) {
        dev->drv_data = NULL;
        dev->ops = NULL;
        return;
    }

    for(i=1; i<CHANS_PER_MODULE+1; i++) {
    		spin_lock_irqsave(&x2->lock, flags);
        __x200_set_led(dev->bus, dev->slot_id*CHANS_PER_MODULE+i, LED_OFF);
        spin_unlock_irqrestore(&x2->lock, flags);
    }

    destroy_busydetect(a200m);

    if (!a200m->usecount)
        fxo200m_release(a200m);
    else
        a200m->dead = 1;

    dev->drv_data = NULL;
    dev->ops = NULL;
    
    return;
}

static void fxo200m_voicedaa_check_hook(struct x200_dev *dev, unsigned int chanx)
{
#ifndef AUDIO_RINGCHECK
    unsigned char res;
#endif
    signed char b;
    int errors = 0;
    struct x200fxo *a200m = dev->drv_data;
    

    b = a200m->fxoconfig[chanx].reg0shadow;

    if ((b & 0x2) || !(b & 0x8)) {
        /* Not good -- don't look at anything else */
        if (debug) {
            printk(KERN_DEBUG "module %d:Errors (%02x) on channel %d!\n", dev->slot_id, b, chanx + 1);
        }
        errors++;
    }
    b &= 0x9b;
    if (a200m->fxoconfig[chanx].offhook) {
        if (b != 0x9) {
            fxo200m_write_reg(dev, 5, 0x9, chanx);
        }
    } else {
        if (b != 0x8) {
            fxo200m_write_reg(dev, 5, 0x8, chanx);
        }
    }
    
    if (errors) {
        return;
    }
    
    if (!a200m->fxoconfig[chanx].offhook) {
        if (fwringdetect) {
            res = a200m->fxoconfig[chanx].reg0shadow & 0x60;
            if (a200m->fxoconfig[chanx].ringdebounce) {
            		--a200m->fxoconfig[chanx].ringdebounce;
                if (res && (res != a200m->fxoconfig[chanx].lastrdtx)
                    && (a200m->fxoconfig[chanx].battery == BATTERY_PRESENT)) {
                    if (!a200m->fxoconfig[chanx].wasringing) {
                        a200m->fxoconfig[chanx].wasringing = 1;
                        if (debug) {
                            printk(KERN_DEBUG "%s: RING on %d/%d!\n", __func__, a200m->span.spanno, chanx + 1);
                        }
                     		dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_RING);
                    }
                    a200m->fxoconfig[chanx].lastrdtx = res;
                    a200m->fxoconfig[chanx].ringdebounce = 10;
                } else if (!res) {
                    if ((a200m->fxoconfig[chanx].ringdebounce == 0)
                        && a200m->fxoconfig[chanx].wasringing) {
                        a200m->fxoconfig[chanx].wasringing = 0;
                        if (debug) {
                            printk(KERN_DEBUG "%s: NO RING on %d/%d!\n", __func__, a200m->span.spanno, chanx + 1);
                        }
                     		dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_OFFHOOK);
                    }
                }
            } else if (res && (a200m->fxoconfig[chanx].battery == BATTERY_PRESENT)) {
                a200m->fxoconfig[chanx].lastrdtx = res;
                a200m->fxoconfig[chanx].ringdebounce = 10;
            }
        } else {
            res = a200m->fxoconfig[chanx].reg0shadow;	
            if ((res & 0x60) && (a200m->fxoconfig[chanx].battery == BATTERY_PRESENT)) {
                a200m->fxoconfig[chanx].ringdebounce ++;
                if (a200m->fxoconfig[chanx].ringdebounce >= ringoncount) {
                    if (!a200m->fxoconfig[chanx].wasringing) {
                        a200m->fxoconfig[chanx].wasringing = 1;
                     		dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_RING);
                     		a200m->fxoconfig[chanx].fastringoffhooktimer = fastringoffhook;
												a200m->fxoconfig[chanx].ringoffhooksent = 0;
                        if (debug) {
                            printk(KERN_DEBUG "%s: %lu RING on %d/%d!\n", __func__, jiffies,a200m->span.spanno, chanx + 1);
                        }
                    }else{
                    		if (fastringoffhook && !a200m->fxoconfig[chanx].ringoffhooksent){
                    			  a200m->fxoconfig[chanx].fastringoffhooktimer -= DEFAULT_DAA_CHECK_INTERVAL;
														if (a200m->fxoconfig[chanx].fastringoffhooktimer <= 0) {
																dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_OFFHOOK);
																a200m->fxoconfig[chanx].ringoffhooksent = 1;
														}
                    		}
                    }
                    a200m->fxoconfig[chanx].ringdebounce = ringdebounce;
                }
            } else {
                a200m->fxoconfig[chanx].ringdebounce--;
                if (a200m->fxoconfig[chanx].ringdebounce <= ringoffcount) { 
                    if (a200m->fxoconfig[chanx].wasringing) {
                        a200m->fxoconfig[chanx].wasringing = 0;
                        if ( ! a200m->fxoconfig[chanx].ringoffhooksent ){
                     				dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_OFFHOOK);
                     				a200m->fxoconfig[chanx].ringoffhooksent = 1;
                     		}
												
                        if (debug) {
                            printk(KERN_DEBUG "%s: %lu NO RING on %d/%d!\n", __func__, jiffies,a200m->span.spanno, chanx + 1);
                        }
                    }
                    a200m->fxoconfig[chanx].ringdebounce = 0;
                }
            }
        }
    }

		if (unlikely(DAHDI_RXSIG_INITIAL == a200m->chans[chanx]->rxhooksig)) {
		/*
		 * dahdi-base will set DAHDI_RXSIG_INITIAL after a
		 * DAHDI_STARTUP or DAHDI_CHANCONFIG ioctl so that new events
		 * will be queued on the channel with the current received
		 * hook state.  Channels that use robbed-bit signalling always
		 * report the current received state via the dahdi_rbsbits
		 * call. Since we only call dahdi_hooksig when we've detected
		 * a change to report, let's forget our current state in order
		 * to force us to report it again via dahdi_hooksig.
		 *
		 */
			a200m->fxoconfig[chanx].battery = BATTERY_UNKNOWN;
		}
	
    b = a200m->fxoconfig[chanx].reg1shadow;
    if (abs(b) < battthresh) {
        /* possible existing states:
           battery lost, no debounce timer
           battery lost, debounce timer (going to battery present)
           battery present or unknown, no debounce timer
           battery present or unknown, debounce timer (going to battery lost)
        */
        
        if (a200m->fxoconfig[chanx].battery == BATTERY_LOST) {
            if (a200m->fxoconfig[chanx].battdebounce) {
                /* we were going to BATTERY_PRESENT, but battery was lost again,
                   so clear the debounce timer */
                a200m->fxoconfig[chanx].battdebounce = 0;
            }
        } else {
            if (a200m->fxoconfig[chanx].battdebounce) {
                /* going to BATTERY_LOST, see if we are there yet */
                if (--a200m->fxoconfig[chanx].battdebounce == 0) {
                    a200m->fxoconfig[chanx].battery = BATTERY_LOST;
                    if (debug)
                        printk(KERN_DEBUG "%s: NO BATTERY on %d/%d!\n", __func__, a200m->span.spanno, chanx + 1);
#ifdef  JAPAN
                    if (!a200m->fxoconfig[chanx].ohdebounce && a200m->fxoconfig[chanx].offhook) {
                        dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_ONHOOK);
                        if (debug)
                            printk(KERN_DEBUG "%s: Signalled On Hook\n", __func__);
#ifdef  ZERO_BATT_RING
                        a200m->fxoconfig[chanx].onhook++;
#endif
                    }
#else
                    dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_ONHOOK);
                    /* set the alarm timer, taking into account that part of its time
                       period has already passed while debouncing occurred */
                    a200m->fxoconfig[chanx].battalarm = (battalarm - battdebounce) / DEFAULT_DAA_CHECK_INTERVAL;
#endif
                }
            } else {
                /* start the debounce timer to verify that battery has been lost */
                a200m->fxoconfig[chanx].battdebounce = battdebounce / DEFAULT_DAA_CHECK_INTERVAL;
            }
        }
    } else {
        /* possible existing states:
           battery lost or unknown, no debounce timer
           battery lost or unknown, debounce timer (going to battery present)
           battery present, no debounce timer
           battery present, debounce timer (going to battery lost)
        */
        if (a200m->fxoconfig[chanx].battery == BATTERY_PRESENT) {
            if (a200m->fxoconfig[chanx].battdebounce) {
                /* we were going to BATTERY_LOST, but battery appeared again,
                   so clear the debounce timer */
                a200m->fxoconfig[chanx].battdebounce = 0;
            }
        } else {
            if (a200m->fxoconfig[chanx].battdebounce) {
                /* going to BATTERY_PRESENT, see if we are there yet */
                if (--a200m->fxoconfig[chanx].battdebounce == 0) {
                    a200m->fxoconfig[chanx].battery = BATTERY_PRESENT;
                    if (debug)
                        printk(KERN_DEBUG "%s: BATTERY on %d/%d (%s)!\n", __func__, a200m->span.spanno, chanx + 1,
                               (b < 0) ? "-" : "+");
#ifdef  ZERO_BATT_RING
                    if (a200m->fxoconfig[chanx].onhook) {
                        a200m->fxoconfig[chanx].onhook = 0;
                        dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
                        if (debug)
                            printk(KERN_DEBUG "%s: Signalled Off Hook\n", __func__);
                    }
#else
                    dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_OFFHOOK);
#endif
                    /* set the alarm timer, taking into account that part of its time
                       period has already passed while debouncing occurred */
                    a200m->fxoconfig[chanx].battalarm = (battalarm - battdebounce) / DEFAULT_DAA_CHECK_INTERVAL;
                }
            } else {
                /* start the debounce timer to verify that battery has appeared */
                a200m->fxoconfig[chanx].battdebounce = battdebounce / DEFAULT_DAA_CHECK_INTERVAL;
            }
        }
    }
    
    if (a200m->fxoconfig[chanx].battalarm) {
        if (--a200m->fxoconfig[chanx].battalarm == 0) {
            /* the alarm timer has expired, so update the battery alarm state
               for this channel */
            dahdi_alarm_channel(a200m->chans[chanx], a200m->fxoconfig[chanx].battery == BATTERY_LOST ? DAHDI_ALARM_RED : DAHDI_ALARM_NONE);
        }
    }
    
    /**/
    if (freezepolaritywhilering) {
				if ( a200m->fxoconfig[chanx].wasringing )
						return;
		}
        
    if (a200m->fxoconfig[chanx].lastpol >= 0) {
        if (b < 0) {
            a200m->fxoconfig[chanx].lastpol = -1;
            a200m->fxoconfig[chanx].polaritydebounce =  polaritydebounce / DEFAULT_DAA_CHECK_INTERVAL;
        }
    }
    if (a200m->fxoconfig[chanx].lastpol <= 0) {
        if (b > 0) {
            a200m->fxoconfig[chanx].lastpol = 1;
            a200m->fxoconfig[chanx].polaritydebounce = polaritydebounce / DEFAULT_DAA_CHECK_INTERVAL;
        }
    }
    		
    if (a200m->fxoconfig[chanx].polaritydebounce) {
        if (--a200m->fxoconfig[chanx].polaritydebounce == 0) {        		
            if (a200m->fxoconfig[chanx].lastpol != a200m->fxoconfig[chanx].polarity) {                   	  	 
                if (debug)
                    printk(KERN_DEBUG "%s: %lu chan[%d] Polarity reversed (%d -> %d)\n", __func__, jiffies,chanx,
                       a200m->fxoconfig[chanx].polarity,
                       a200m->fxoconfig[chanx].lastpol);

                if (a200m->fxoconfig[chanx].polarity) {
                    if(a200m->fxoconfig[chanx].offhook &&
                       !a200m->fxoconfig[chanx].callout &&
                       twowaychargeflag) {
                        /*If gate way(test on XUNSHI MX60) open two-way charge, gate way send out a polarity reverse
                         * signal to indicate start charging in a specified time(3sec default) after offhooking, this
                         * can cause FXO onhook, so we should ignore it. In addition, if FXS onhook before this reverse signal,
                         * FXS will NOT send polarity reverse signals until the specified time(3sec default) reach.
                         */
                        a200m->fxoconfig[chanx].polaritycountwhenoffhook++;
                        if(1 != a200m->fxoconfig[chanx].polaritycountwhenoffhook){ //ignore two-way charge signal
                            dahdi_qevent_lock(a200m->chans[chanx], DAHDI_EVENT_POLARITY);
                        } else {
#ifdef DEBUG_FXO_EVT
                            printk("===FXO<%d/span%d>, Ignore two-way charge polarity reverse signal \t%s:%d<%s>\n", 
                                    chanx + 1, a200m->span.spanno, __SFILE__, __LINE__, __func__);
#endif /*DEBUG_FXO_EVT*/
                        }
                    } else {

                        dahdi_qevent_lock(a200m->chans[chanx], DAHDI_EVENT_POLARITY);
                    }
                }
            	  
             		a200m->fxoconfig[chanx].polarity = a200m->fxoconfig[chanx].lastpol;
            }
        }
    }
}

static void fxo200m_transmitprep(struct x200_dev *dev)
{
    int x, y;
    struct x200fxo* a200m = dev->drv_data;

    struct x200* x2 = dev->bus;
    /* Calculate Transmission */
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		_dahdi_transmit(&a200m->span);
#else
    dahdi_transmit(&a200m->span);
#endif
    parser_busy_silent_process(dev->drv_data, 1);
    for (x=0;x<CHANS_PER_MODULE; x++) {
        for (y=0;y<DAHDI_CHUNKSIZE;y++) {
            x2->current_writechunk[y*128+dev->slot_id+(1+x)*4] = a200m->chans[x]->writechunk[y];
        }
    }
}

static void fxo200m_receiveprep(struct x200_dev *dev)
{
    int x, y;
    struct x200fxo* a200m = dev->drv_data;
    struct x200* x2 = dev->bus;

    for (x=0; x<CHANS_PER_MODULE; x++) {
        for (y=0;y<DAHDI_CHUNKSIZE;y++) {
            a200m->chans[x]->readchunk[y] = x2->current_readchunk[y*128+dev->slot_id+(1+x)*4];
        }
    }
		
    parser_busy_silent_process(dev->drv_data, 0);

    for (x=0; x<CHANS_PER_MODULE; x++) {
        dahdi_ec_chunk(a200m->chans[x], a200m->chans[x]->readchunk, a200m->chans[x]->writechunk);
    }
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		_dahdi_receive(&a200m->span);
#else
    dahdi_receive(&a200m->span);
#endif
}

int fxo200m_interrupt(int intcount, struct x200_dev *dev)
{
    struct x200fxo* a200m = dev->drv_data;
    int chan_sel;
			
    chan_sel = intcount&0x1;
 
    fxo200m_transmitprep(dev);
    fxo200m_receiveprep(dev);
    
    a200m->fxoconfig[chan_sel].reg0shadow = fxo200m_read_reg(dev, 5, chan_sel);
    a200m->fxoconfig[chan_sel].reg1shadow = fxo200m_read_reg(dev, 29, chan_sel);
    fxo200m_voicedaa_check_hook(dev, chan_sel);
    
    return IRQ_RETVAL(1);
}


struct ints_ops fxo200m_ops={
		.irq_handler = fxo200m_interrupt,
    .get_clk_src = NULL,
    .set_clk_src = NULL,
    .unset_clk_src = NULL,
};

static int x200_dev_probe(struct device *dev)
{
		int ret;
		struct x200_dev *x2dev = to_x200_dev(dev);
		ret = fxo200m_init_one(x2dev);
		if(!ret){
				x2dev->ops = &fxo200m_ops;
		}
		return ret;
}

static int x200_dev_remove(struct device *dev)
{
		struct x200_dev *x2dev = to_x200_dev(dev);
		fxo200m_remove_one(x2dev);
		return 0;
}

static int __init fxo200m_init(void)
{
    int x;
    struct bus_type *x200bus;

    for (x = 0; x < (sizeof(fxo_modes) / sizeof(fxo_modes[0])); x++) {
        if (!strcmp(fxo_modes[x].name, opermode))
            break;
    }
    if (x < sizeof(fxo_modes) / sizeof(fxo_modes[0])) {
        _opermode = x;
    } else {
        printk(KERN_NOTICE "Invalid/unknown operating mode '%s' specified.  Please choose one of:\n", opermode);
        for (x = 0; x < sizeof(fxo_modes) / sizeof(fxo_modes[0]); x++)
            printk(KERN_INFO "  %s\n", fxo_modes[x].name);
        printk(KERN_INFO "Note this option is CASE SENSITIVE!\n");
        return -ENODEV;
    }

    if (!strcmp(opermode, "AUSTRALIA")) {
        boostringer = 1;
        fxshonormode = 1;
    }

    /* for the voicedaa_check_hook defaults, if the user has not overridden
       them by specifying them as module parameters, then get the values
       from the selected operating mode
    */
    if (battdebounce == 0) {
        battdebounce = fxo_modes[_opermode].battdebounce;
    }
    if (battalarm == 0) {
        battalarm = fxo_modes[_opermode].battalarm;
    }
    if (battthresh == 0) {
        battthresh = fxo_modes[_opermode].battthresh;
    }
          
    for(x=0;x<interface_num;x++){
    		x200bus = get_bus(x);
    		x200_dev_driver[x].name = "fxo200m_driver";
    		x200_dev_driver[x].bus = x200bus;
    		x200_dev_driver[x].probe = x200_dev_probe;
    		x200_dev_driver[x].remove = x200_dev_remove;
    		    		
    		if(driver_register(&x200_dev_driver[x]))
    		{
    				printk("x200 bus %d register d100m driver error!\n",x);
    		}
    }
       
    return 0;
}

static void __exit fxo200m_cleanup(void)
{
    int i;
		
		for(i=0;i<interface_num;i++){
				driver_unregister(&x200_dev_driver[i]);
		}
}

module_param(debug, int, 0600);
module_param(_opermode, int, 0600);
module_param(opermode, charp, 0600);
module_param(fxshonormode, int, 0600);
module_param(battdebounce, uint, 0600);
module_param(battalarm, uint, 0600);
module_param(boostringer, int, 0600);
module_param(battthresh, uint, 0600);
module_param(ringdebounce, int, 0600);
module_param(ringoncount, int, 0600);
module_param(ringoffcount, int, 0600);
module_param(fastringoffhook, int, 0600);
module_param(fwringdetect, int, 0600);
module_param(alawoverride, int, 0600);
module_param(fastpickup, int, 0600);
module_param(fxotxgain, int, 0600);
module_param(fxorxgain, int, 0600);
module_param(fxofullscale, int, 0600);
module_param(cidbuflen, int, 0600);
module_param(cidtimeout, int, 0600);

module_param(polaritydebounce, int, 0600);
module_param(freezepolaritywhilering, int, 0600);
module_param(twowaychargeflag, int, 0600);

MODULE_DESCRIPTION("OpenVox FXO module of x200 card Driver ");
MODULE_AUTHOR("Miao Lin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(fxo200m_init);
module_exit(fxo200m_cleanup);
