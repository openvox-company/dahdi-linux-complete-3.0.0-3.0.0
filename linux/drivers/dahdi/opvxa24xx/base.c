/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * $Id: base.c 359 2011-04-06 06:11:39Z yangshugang $
 
 * Copyright (C) 2005-2010 OpenVox Communication Co. Ltd,
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* Rev histroy
 *
 * Rev 0.10 initial version, modified from opvxa1200.c
 * Rev 0.20 add octasic echo canceller support.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/nmi.h>
#endif /* 4.11.0 */

#ifdef TEST_LOG_INCOME_VOICE
#include <linux/string.h>
#include <asm/uaccess.h> 	/* get_fs(), set_fs(), KERNEL_DS */
#include <linux/file.h> 	/* fput() */
#endif

#include <dahdi/kernel.h>
#include <dahdi/wctdm_user.h>
#include "fxo_modes.h"

#include "proslic.h"
#include "base.h"
#include "ec3000.h"
#include "busydetect.h"

/* module parameters */
int debug;
int spi_cmd=0x223;

int reversepolarity = 0;
int alawoverride = 0;
int fxotxgain = 0;
int fxorxgain = 0;
int fastpickup = 0;
int fxofullscale = 0;	/* fxo full scale tx/rx, register 30, acim */
int fwringdetect = 0;
int _opermode = 0;
int cidsupport = 1;     /* default support caller id analysis */
int cidbuflen = 3000;	/* in msec, default 3000 */
int cidtimeout = 6*1000;	/* in msec, default 6000 */
int fxstxgain = 0;
int fxsrxgain = 0;
int fxshonormode = 0;
int boostringer = 0;
int fastringer = 0;
int lowpower = 0;
int loopcurrent = 20;
int timingonly = 0;
int fixedtimepolarity=0;	/* time delay in ms when send polarity after rise edge of 1st ring.*/
int ringdebounce = DEFAULT_RING_DEBOUNCE;
unsigned int battdebounce;
unsigned int battalarm;
unsigned int battthresh;
int robust = 0;
#ifdef VPM_SUPPORT
/* ec debug */
int ec_debug = 0;
int vpmsupport = 1;
#endif

#define MS_PER_HOOKCHECK	(1)

int ec_spans = 4;

static char* A2410P_Name = "A2410P";
static struct a24xx_desc a2410a = { "OpenVox A2410", 0 };
static char* A1610P_Name = "A1610P";
static struct a24xx_desc a1610a = { "OpenVox A1610", 0 };
static char* A810P_Name = "A810P";
static struct a24xx_desc a810a = { "OpenVox A810", 0 };
static struct a24xx *ifaces[WC_MAX_IFACES];
static char *opermode = "FCC";

/* If set to auto, vpmdtmfsupport is enabled for VPM400M and disabled for VPM450M */
static int vpmdtmfsupport = -1; /* -1=auto, 0=disabled, 1=enabled*/
static unsigned int ms_per_irq = 1;	// 1/2/4/8/16 allowed
static unsigned int max_iface_index = 0;
static unsigned int irq_stub = 0;

#define POLARITY_XOR(card) ( \
		(reversepolarity != 0) ^ (wc_dev->mod[(card)].fxs.reversepolarity != 0) ^ \
		(wc_dev->mod[(card)].fxs.vmwi_lrev != 0) ^\
		((wc_dev->mod[(card)].fxs.vmwisetting.vmwi_type & DAHDI_VMWI_HVAC) != 0)\
		)

static const struct dahdi_echocan_features vpm_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static void a24xx_release(struct a24xx *wc);
static void a24xx_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_ops vpm_ec_ops = {
//	.name = "OPENVOX VPM",
	.echocan_free = a24xx_echocan_free,
};

static int a24xx_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			  struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	int channel;
	const struct dahdi_echocan_ops *ops;
	const struct dahdi_echocan_features *features;


	if (!wc_dev->vpm) {
		return -ENODEV;
	}

	if (chan->span->offset >= ec_spans) {
		return -ENODEV;
	}

	if (wc_dev->vpm_ec) {
		ops = &vpm_ec_ops;
		features = &vpm_ec_features;
	} 

	if (ecp->param_count > 0) {
		printk(KERN_WARNING "OpenVox VPM echo canceller does not support parameters; failing request\n");
		return -EINVAL;
	}

	*ec = wc->ec[chan->chanpos - 1];
	(*ec)->ops = ops;
	(*ec)->features = *features;

	channel = chan->chanpos;

	if (wc_dev->vpm_ec) {
		channel -= 1;
		if(ec_debug) {
			printk(KERN_DEBUG "echocan: Card is %d, Channel is %d, offset is %d, length %d\n",
				wc_dev->pos, chan->chanpos, channel, ecp->tap_length);
		}
		opvx_vpm_setec(wc_dev->vpm_ec, channel, ecp->tap_length);
		msleep(10);
	}

	return 0;
}

static void a24xx_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;
	int channel;

	memset(ec, 0, sizeof(*ec));

	channel = chan->chanpos;

	if (wc_dev->vpm_ec) {
		channel -= 1;
		if (ec_debug)
			printk(KERN_DEBUG "echocan: Card is %d, Channel is %d, Span is %d, offset is %d length 0\n",
					wc_dev->pos, chan->chanpos, chan->span->offset, channel);
		opvx_vpm_setec(wc_dev->vpm_ec, channel, 0);
	} 
}

static void free_wc(struct a24xx *wc)
{
	int i;
	struct a24xx_dev *wc_dev = &wc->dev;

	for (i = 0; i < wc_dev->max_cards; i++) {
		if (wc->ec[i]) {
			kfree(wc->ec[i]);
			wc->ec[i] = NULL;
		}
	}
	kfree(wc);
	wc = NULL;
}

static void a24xx_vpm_init(struct a24xx *wc)
{
	int x;
	struct a24xx_dev *wc_dev = &wc->dev;
	int laws[4] = { 0, };
	int res;

	unsigned int vpm_capacity;
	struct firmware embedded_firmware;
	const struct firmware *firmware = &embedded_firmware;

#if !defined(HOTPLUG_FIRMWARE)
	extern void _binary_dahdi_fw_oct6114_032_bin_size;
	extern void _binary_dahdi_fw_oct6114_064_bin_size;
	extern void _binary_dahdi_fw_oct6114_128_bin_size;
	extern u8 _binary_dahdi_fw_oct6114_032_bin_start[];
	extern u8 _binary_dahdi_fw_oct6114_064_bin_start[];
	extern u8 _binary_dahdi_fw_oct6114_128_bin_start[];
#else
	static const char oct032_firmware[] = "dahdi-fw-oct6114-032.bin";
	static const char oct064_firmware[] = "dahdi-fw-oct6114-064.bin";
	static const char oct128_firmware[] = "dahdi-fw-oct6114-128.bin";
#endif

    if  ( (wc_dev->fwversion&0xffff) > 0x3) {
        res = __opvx_a24xx_check_vpm_v2(wc_dev->mem32);
    } else {
        res = __opvx_a24xx_check_vpm(wc_dev->mem32);
    }   
	if (res < 0) {
		if (-1 == res) {
			printk("OpenVox VPM: Support Disabled\n");		
		} else if (-2 == res) {
			printk("OpenVox VPM: Not Present\n");		
		}
		return;
	}
	
	/* Setup alaw vs ulaw rules */
	for (x = 0;x < 1; x++) {
		if(wc->span.deflaw == DAHDI_LAW_ALAW) {
			laws[x] = 1;
		}
		else {
			laws[x] = 0;
		}
	}

	vpm_capacity = opvx_vpm_getcapacity(wc_dev);
	printk("OpenVox VPM: echo cancellation supports %d channels\n", vpm_capacity);

	switch (vpm_capacity) {
	case 32:
#if defined(HOTPLUG_FIRMWARE)
		if ((request_firmware(&firmware, oct032_firmware, &wc_dev->dev->dev) != 0) ||
		    !firmware) {
			printk("OpenVox VPM: firmware %s not available from userspace\n", oct032_firmware);
			return;
		}
#else
		embedded_firmware.data = _binary_dahdi_fw_oct6114_032_bin_start;
		/* Yes... this is weird. objcopy gives us a symbol containing
		   the size of the firmware, not a pointer a variable containing
		   the size. The only way we can get the value of the symbol
		   is to take its address, so we define it as a pointer and
		   then cast that value to the proper type.
	      */
		embedded_firmware.size = (size_t) &_binary_zaptel_fw_oct6114_032_bin_size;
#endif
		break;
	case 64:
#if defined(HOTPLUG_FIRMWARE)
		if ((request_firmware(&firmware, oct064_firmware, &wc_dev->dev->dev) != 0) ||
		    !firmware) {
			printk("OpenVox VPM: firmware %s not available from userspace\n", oct064_firmware);
			return;
		}
#else
		embedded_firmware.data = _binary_dahdi_fw_oct6114_064_bin_start;
		/* Yes... this is weird. objcopy gives us a symbol containing
		   the size of the firmware, not a pointer a variable containing
		   the size. The only way we can get the value of the symbol
		   is to take its address, so we define it as a pointer and
		   then cast that value to the proper type.
	      */
		embedded_firmware.size = (size_t) &_binary_dahdi_fw_oct6114_064_bin_size;
#endif
		break;
	case 128:
#if defined(HOTPLUG_FIRMWARE)
		if ((request_firmware(&firmware, oct128_firmware, &wc_dev->dev->dev) != 0) ||
		    !firmware) {
			printk("OpenVox VPM: firmware %s not available from userspace\n", oct128_firmware);
			return;
		}
#else
		embedded_firmware.data = _binary_dahdi_fw_oct6114_128_bin_start;
		/* Yes... this is weird. objcopy gives us a symbol containing
		   the size of the firmware, not a pointer a variable containing
		   the size. The only way we can get the value of the symbol
		   is to take its address, so we define it as a pointer and
		   then cast that value to the proper type.
		*/
		embedded_firmware.size = (size_t) &_binary_dahdi_fw_oct6114_128_bin_size;
#endif
		break;
	default:
		printk("Unsupported channel capacity found on VPM module (%d).\n", vpm_capacity);
		return;
	}

	if (!(wc_dev->vpm_ec = opvx_vpm_init(wc_dev, laws, /*wc_dev->numspans*/1, firmware))) {
		printk("OpenVox VPM: Failed to initialize\n");
		if (firmware != &embedded_firmware) {
			release_firmware(firmware);
		}
		return;
	}

	if (firmware != &embedded_firmware) {
		release_firmware(firmware);
	}

	if (vpmdtmfsupport == -1) {
		printk("OpenVox VPM: hardware DTMF disabled.\n");
		vpmdtmfsupport = 0;
	}
	
	__a24xx_vpm_setpresent(wc_dev);
	
}

#ifdef AUDIO_RINGCHECK
static inline void ring_check(struct a24xx *wc, int card)
{
	int x;
	short sample;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	if (wc_dev->modtype[card] != MOD_TYPE_FXO) {
		return;
	}
	wc_dev->mod[card].fxo.pegtimer += DAHDI_CHUNKSIZE;
	for (x=0;x<DAHDI_CHUNKSIZE;x++) {
		/* Look for pegging to indicate ringing */
		sample = DAHDI_XLAW(wc->chans[card].readchunk[x], (&(wc->chans[card])));
		if ((sample > 10000) && (wc_dev->mod[card].fxo.peg != 1)) {
			if (debug > 1) {
				printk(KERN_DEBUG "High peg!\n");
			}
			if ((wc_dev->mod[card].fxo.pegtimer < PEGTIME) && (wc_dev->mod[card].fxo.pegtimer > MINPEGTIME)) {
				wc_dev->mod[card].fxo.pegcount++;
			}
			wc_dev->mod[card].fxo.pegtimer = 0;
			wc_dev->mod[card].fxo.peg = 1;
		} else if ((sample < -10000) && (wc_dev->mod[card].fxo.peg != -1)) {
			if (debug > 1) {
				printk(KERN_DEBUG "Low peg!\n");
			}
			if ((wc_dev->mod[card].fxo.pegtimer < (PEGTIME >> 2)) && (wc_dev->mod[card].fxo.pegtimer > (MINPEGTIME >> 2))) {
				wc_dev->mod[card].fxo.pegcount++;
			}
			wc_dev->mod[card].fxo.pegtimer = 0;
			wc_dev->mod[card].fxo.peg = -1;
		}
	}
	if (wc_dev->mod[card].fxo.pegtimer > PEGTIME) {
		/* Reset pegcount if our timer expires */
		wc_dev->mod[card].fxo.pegcount = 0;
	}
	/* Decrement debouncer if appropriate */
	if (wc_dev->mod[card].fxo.ringdebounce) {
		wc_dev->mod[card].fxo.ringdebounce--;
	}
	if (!wc_dev->mod[card].fxo.offhook && !wc_dev->mod[card].fxo.ringdebounce) {
		if (!wc_dev->mod[card].fxo.ring && (wc_dev->mod[card].fxo.pegcount > PEGCOUNT)) {
			/* It's ringing */
			if (debug) {
				printk(KERN_DEBUG "RING on %d/%d!\n", wc->span.spanno, card + 1);
			}
			if (!wc_dev->mod[card].fxo.offhook) {
				dahdi_hooksig(&wc->chans[card], DAHDI_RXSIG_RING);
			}
			wc_dev->mod[card].fxo.ring = 1;
		}
		if (wc_dev->mod[card].fxo.ring && !wc_dev->mod[card].fxo.pegcount) {
			/* No more ring */
			if (debug) {
				printk(KERN_DEBUG "NO RING on %d/%d!\n", wc->span.spanno, card + 1);
			}
			dahdi_hooksig(&wc->chans[card], DAHDI_RXSIG_OFFHOOK);
			wc_dev->mod[card].fxo.ring = 0;
		}
	}
}
#endif

#if 0
static int a24xx_hardware_init(struct a24xx_dev *wc_dev, char *sigcap)
{
	unsigned int x;
	unsigned char ch;

	/* Signal Reset */
    if  ( (wc_dev->fwversion&0xffff) > 0x3)
	    __opvx_a24xx_reset_modules_v2(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;
	else 
	    __opvx_a24xx_reset_modules(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;

	/* Check OpenVox version */
	printk("OpenVox %s version: %01x.%01x\n", wc_dev->card_name, wc_dev->fwversion>>16, wc_dev->fwversion&0xffff);

	/* Clear interrupts */
	__opvx_a24xx_clear_irqs(wc_dev->mem32);

	/* Wait 1/4 of a second more */
	//opvx_wait_just_a_bit(HZ/4);

	/* test cards exist and type at first */
	for(x=0; x<wc_dev->max_cards; x+=CARDS_PER_MODULE) {
		__a24xx_setcard(wc_dev, x);
		wc_dev->modtype[x] = MOD_TYPE_FXO;
		ch = __a24xx_spi_getreg(wc_dev, x, 2);	/* read register 2, 3050 return 0x3, 3210 return 0x0 */
		if(0x03 == ch) {
			wc_dev->modtype[x+1] = MOD_TYPE_FXO;
			wc_dev->modtype[x+2] = MOD_TYPE_FXO;
			wc_dev->modtype[x+3] = MOD_TYPE_FXO;
			if(debug) {
				printk("module %d is a FXO\n", x);
			}
		} else {
			wc_dev->modtype[x] = MOD_TYPE_FXS;
			wc_dev->modtype[x+1] = MOD_TYPE_FXS;
			wc_dev->modtype[x+2] = MOD_TYPE_FXS;
			wc_dev->modtype[x+3] = MOD_TYPE_FXS;
			if(debug) {
				printk("module %d is a FXS or Not Installed\n", x);
			}
		}
	}

    if  ( (wc_dev->fwversion&0xffff) > 0x3)
	    __opvx_a24xx_reset_modules_v2(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;
	else 
	    __opvx_a24xx_reset_modules(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;

	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
		int sane=0,ret=0,readi=0;

		if( (x%4) == 0 && wc_dev->modtype[x]==MOD_TYPE_FXS) {	/* set 3215 to daisy chain mode */
			__a24xx_setcard(wc_dev, x);
			__opvx_a24xx_write_8bits(wc_dev->mem32, 0x00);
			__opvx_a24xx_write_8bits(wc_dev->mem32, 0x80);
		}

#if 1
		touch_softlockup_watchdog(); // avoid showing CPU softlock message

		/* Init with Auto Calibration */
		if (!(ret=si321x_init_proslic(wc_dev, x, 0, 0, sane))) {
			wc_dev->cardflag |= (1 << x);
			if (debug) {
					readi = a24xx_spi_getreg(wc_dev,x,LOOP_I_LIMIT);
					printk("Proslic module %d loop current is %dmA\n",x,
					((readi*3)+20));
			}
			printk("Module %d: Installed -- AUTO FXS/DPO\n",x);
		} else {
			if(ret!=-2) {
				sane=1;

				printk("Init ProSlic with Manual Calibration \n");
				/* Init with Manual Calibration */
				if (!si321x_init_proslic(wc_dev, x, 0, 1, sane)) {
					wc_dev->cardflag |= (1 << x);
					if (debug) {
							readi = a24xx_spi_getreg(wc_dev,x,LOOP_I_LIMIT);
							printk("Proslic module %d loop current is %dmA\n",x,
							((readi*3)+20));
					}
					printk("Module %d: Installed -- MANUAL FXS\n",x);
				} else {
					printk("Module %d: FAILED FXS (%s)\n", x, fxshonormode ? fxo_modes[_opermode].name : "FCC");	
					sigcap[x] = 1;
				}
			} else if (!(ret = si3050_init_voicedaa(wc_dev, x, 0, 0, sane))) {
				wc_dev->cardflag |= (1 << x);
				printk("Module %d: Installed -- AUTO FXO (%s mode)\n",x, fxo_modes[_opermode].name);
			} else
				printk("Module %d: Not installed\n", x);
		}
#endif
	}

	/* Return error if nothing initialized okay. */
	if (!wc_dev->cardflag && !timingonly) {
		return -1;
	}

	return 0;
}
#endif

static int a24xx_hardware_init_all(struct a24xx_dev *wc_dev, char *sigcap)
{
	unsigned int x;
	unsigned char ch;
	int sane=0,ret=0;
	int flag=0,tmp_flag=0,blank_flag=0;

	/* Signal Reset */
    if  ( (wc_dev->fwversion&0xffff) > 0x3)
	    __opvx_a24xx_reset_modules_v2(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;
	else 
	    __opvx_a24xx_reset_modules(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;

	/* Check OpenVox version */
	printk("OpenVox %s version: %01x.%01x\n", wc_dev->card_name, wc_dev->fwversion>>16, wc_dev->fwversion&0xffff);

	/* Clear interrupts */
	__opvx_a24xx_clear_irqs(wc_dev->mem32);

	/* Wait 1/4 of a second more */
	//opvx_wait_just_a_bit(HZ/4);

	/* test cards exist and type at first */
	for(x=0; x<wc_dev->max_cards; x+=CARDS_PER_MODULE) {
		__a24xx_setcard(wc_dev, x);
		wc_dev->modtype[x] = MOD_TYPE_FXO;
		ch = __a24xx_spi_getreg(wc_dev, x, 2);	/* read register 2, 3050 return 0x3, 3210 return 0x0 */
		if(0x03 == ch) {
			wc_dev->modtype[x+1] = MOD_TYPE_FXO;
			wc_dev->modtype[x+2] = MOD_TYPE_FXO;
			wc_dev->modtype[x+3] = MOD_TYPE_FXO;
			if(debug) {
				printk("module %d is a FXO\n", x);
			}
		} else {
			wc_dev->modtype[x] = MOD_TYPE_FXS;
			wc_dev->modtype[x+1] = MOD_TYPE_FXS;
			wc_dev->modtype[x+2] = MOD_TYPE_FXS;
			wc_dev->modtype[x+3] = MOD_TYPE_FXS;
			if(debug) {
				printk("module %d is a FXS or Not Installed\n", x);
			}
		}
	}

    if  ( (wc_dev->fwversion&0xffff) > 0x3)
	    __opvx_a24xx_reset_modules_v2(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;
	else 
	    __opvx_a24xx_reset_modules(wc_dev->mem32, __a24xx_wait_just_a_bit, HZ); // reset again;

	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
		sane=1,ret=0;
		touch_softlockup_watchdog(); // avoid showing CPU softlock message

		/* Init with Auto Calibration */
		if(wc_dev->modtype[x] == MOD_TYPE_FXO){
				if (!(ret = si3050_init_voicedaa(wc_dev, x, 0, 0, sane))) {
						wc_dev->cardflag |= (1 << x);
				}
		}
	}
	
	/*initial FXS modules*/
	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
			if(wc_dev->modtype[x] == MOD_TYPE_FXS){
					flag |=(1 << x);
			}
	}
	
	touch_softlockup_watchdog();
	if((tmp_flag=si321x_init_proslic_all(wc_dev,flag,0,0,0,&blank_flag))>0)
	{
			 tmp_flag=si321x_init_proslic_all(wc_dev,tmp_flag,0,1,1,NULL);
	}
	flag &=~blank_flag;
	flag &=~tmp_flag;
	wc_dev->cardflag |=flag;
	
	for (x = 0; x < wc_dev->max_cards; x++){
			if(wc_dev->cardflag & (1<<x)){
					spin_lock_init(&wc_dev->mod[x].fxs.lasttxhooklock);
					if(wc_dev->modtype[x] == MOD_TYPE_FXS){
							printk("Module %d: Installed -- AUTO FXS/DPO\n",x);
					}else
							printk("Module %d: Installed -- AUTO FXO (%s mode)\n",x, fxo_modes[_opermode].name);	
			}else{
					if(tmp_flag & (1 << x)){
							printk("Module %d: FAILED FXS (%s)\n", x, fxshonormode ? fxo_modes[_opermode].name : "FCC");
							sigcap[x] = 1;      //************
					}
					else
							printk("Module %d: Not installed\n", x);
			}	
	}
	
	/* Return error if nothing initialized okay. */
	if (!wc_dev->cardflag && !timingonly) {
		return -1;
	}
	
	return 0;
}

static void callerid_ring_on_deal(struct a24xx *wc, int card)
{
        struct a24xx_dev *wc_dev = &wc->dev;

        if(wc_dev->cid_state[card] == CID_STATE_IDLE) {
				reset_parser_variable_from_chan_num(wc->span.spanno, wc->chans[card]->channo);
                if (is_ring_delay_operation(wc->span.spanno, wc->chans[card]->channo)) {
                        wc_dev->cid_state[card] = CID_STATE_RING_DELAY;
                } else {
                        wc_dev->cid_state[card] = CID_STATE_RING_ON;
                        wc_dev->cid_history_clone_cnt[card] = cidbuflen;
                        dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_RING);
                }
        } else if(wc_dev->cid_state[card] == CID_STATE_RING_DELAY) {
                wc_dev->cid_state[card] = CID_STATE_RING_ON;
                wc_dev->cid_history_clone_cnt[card] = cidbuflen;
                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_RING);
        } else {
                if (wc_dev->cid_state[card] != CID_STATE_WAIT_RING_FINISH) {
                        set_signal_unknown_from_chan_num(wc->span.spanno, wc->chans[card]->channo);
                        wc_dev->cid_state[card] = CID_STATE_WAIT_RING_FINISH;
                }
                wc_dev->cid_history_clone_cnt[card] = cidtimeout;
                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_RING);
        }
}

static void callerid_ring_off_deal(struct a24xx *wc, int card)
{
        struct a24xx_dev *wc_dev = &wc->dev;

        if(wc_dev->cid_state[card] == CID_STATE_RING_ON) {
                wc_dev->cid_state[card] = CID_STATE_RING_OFF;
        }

        if(wc_dev->cid_state[card] != CID_STATE_RING_DELAY) {
                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
        }
}

static void a24xx_voicedaa_check_hook(struct a24xx *wc, int card)
{
#define MS_PER_CHECK_HOOK 16
#ifndef AUDIO_RINGCHECK
	unsigned char res;
#endif
	signed char b;
	int errors = 0;
	struct a24xx_dev *wc_dev = &wc->dev;
	struct fxo *fxo = &wc_dev->mod[card].fxo;

	/* Try to track issues that plague slot one FXO's */
	b = wc_dev->reg0shadow[card];
	if ((b & 0x2) || !(b & 0x8)) {
		/* Not good -- don't look at anything else */
		if (debug) {
			printk(KERN_DEBUG "Errors (%02x) on card %d!\n", b, card + 1);
		}
		errors++;
	}
	b &= 0x9b;
	if (fxo->offhook) {
		if (b != 0x9) {
			a24xx_spi_setreg(wc_dev, card, 5, 0x9);
		}
	} else {
		if (b != 0x8) {
			a24xx_spi_setreg(wc_dev, card, 5, 0x8);
		}
	}
	if (errors) {
		return;
	}

	if (!fxo->offhook) {
		if (fwringdetect) {
			res = wc_dev->reg0shadow[card] & 0x60;
			if (fxo->ringdebounce--) {
				if (res && (res != fxo->lastrdtx)
					&& (fxo->battery == BATTERY_PRESENT)) {
					if (!fxo->wasringing) {
						fxo->wasringing = 1;
						if (debug) {
							printk(KERN_DEBUG "RING on %d/%d!\n", wc->span.spanno, card + 1);
						}
                                                if(cidsupport) {
							callerid_ring_on_deal(wc, card);
                                                } else {
	                                                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_RING);
						}
					}
					fxo->lastrdtx = res;
					fxo->ringdebounce = 10;
				} else if (!res) {
					if ((fxo->ringdebounce == 0)
						&& fxo->wasringing) {
						fxo->wasringing = 0;
						if (debug) {
							printk(KERN_DEBUG "NO RING on %d/%d!\n", wc->span.spanno, card + 1);
						}
                                                if(cidsupport) {
							callerid_ring_off_deal(wc, card);
                                                } else {
	                                                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
						}
					}
				}
			} else if (res && (fxo->battery == BATTERY_PRESENT)) {
				fxo->lastrdtx = res;
				fxo->ringdebounce = 10;
			}
		} else {
			res = wc_dev->reg0shadow[card];
			if ((res & 0x60) && (fxo->battery == BATTERY_PRESENT)) {
				fxo->ringdebounce += (DAHDI_CHUNKSIZE * 16);
				if (fxo->ringdebounce >= DAHDI_CHUNKSIZE * ringdebounce) {
					if (!fxo->wasringing) {
						fxo->wasringing = 1;
                                                if(cidsupport) {
							callerid_ring_on_deal(wc, card);
                                                } else {
	                                                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_RING);
						}
						if (debug) {
							printk(KERN_DEBUG "RING on %d/%d!\n", wc->span.spanno, card + 1);
						}
					}
					fxo->ringdebounce = DAHDI_CHUNKSIZE * ringdebounce;
				}
			} else {
				fxo->ringdebounce -= DAHDI_CHUNKSIZE * 4;
				if (fxo->ringdebounce <= 0) {
					if (fxo->wasringing) {
						fxo->wasringing = 0;
                                                if(cidsupport) {
							callerid_ring_off_deal(wc, card);
                                                } else {
	                                                dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
						}
						if (debug) {
							printk(KERN_DEBUG "NO RING on %d/%d!\n", wc->span.spanno, card + 1);
						}
					}
					fxo->ringdebounce = 0;
				}
			}
		}
	}

	b = wc_dev->reg1shadow[card];
	if (abs(b) < battthresh) {
		/* possible existing states:
		   battery lost, no debounce timer
		   battery lost, debounce timer (going to battery present)
		   battery present or unknown, no debounce timer
		   battery present or unknown, debounce timer (going to battery lost)
		*/

		if (fxo->battery == BATTERY_LOST) {
			if (fxo->battdebounce) {
				/* we were going to BATTERY_PRESENT, but battery was lost again,
				   so clear the debounce timer */
				fxo->battdebounce = 0;
			}
		} else {
			if (fxo->battdebounce) {
				/* going to BATTERY_LOST, see if we are there yet */
				if (--fxo->battdebounce == 0) {
					fxo->battery = BATTERY_LOST;
					if (debug) {
						printk(KERN_DEBUG "NO BATTERY on %d/%d!\n", wc->span.spanno, card + 1);
					}
#ifdef	JAPAN
					if (!wc_dev->ohdebounce && wc_dev->offhook) {
						dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_ONHOOK);
						if (debug) {
							printk(KERN_DEBUG "Signalled On Hook\n");
						}
#ifdef	ZERO_BATT_RING
						wc_dev->onhook++;
#endif
					}
#else
					dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_ONHOOK);
					/* set the alarm timer, taking into account that part of its time
					   period has already passed while debouncing occurred */
					fxo->battalarm = (battalarm - battdebounce) / MS_PER_CHECK_HOOK;
#endif
				}
			} else {
				/* start the debounce timer to verify that battery has been lost */
				fxo->battdebounce = battdebounce / MS_PER_CHECK_HOOK;
			}
		}
	} else {
		/* possible existing states:
		   battery lost or unknown, no debounce timer
		   battery lost or unknown, debounce timer (going to battery present)
		   battery present, no debounce timer
		   battery present, debounce timer (going to battery lost)
		*/
		if (fxo->battery == BATTERY_PRESENT) {
			if (fxo->battdebounce) {
				/* we were going to BATTERY_LOST, but battery appeared again,
				   so clear the debounce timer */
				fxo->battdebounce = 0;
			}
		} else {
			if (fxo->battdebounce) {
				/* going to BATTERY_PRESENT, see if we are there yet */
				if (--fxo->battdebounce == 0) {
					fxo->battery = BATTERY_PRESENT;
					if (debug) {
						printk(KERN_DEBUG "BATTERY on %d/%d (%s)!\n", wc->span.spanno, card + 1,
							(b < 0) ? "-" : "+");
					}
#ifdef	ZERO_BATT_RING
					if (wc_dev->onhook) {
						wc_dev->onhook = 0;
						dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
						if (debug) {
							printk(KERN_DEBUG "Signalled Off Hook\n");
						}
					}
#else
					dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
#endif
					/* set the alarm timer, taking into account that part of its time
					   period has already passed while debouncing occurred */
					fxo->battalarm = (battalarm - battdebounce) / MS_PER_CHECK_HOOK;
				}
			} else {
				/* start the debounce timer to verify that battery has appeared */
				fxo->battdebounce = battdebounce / MS_PER_CHECK_HOOK;
			}
		}
	}

	if (fxo->lastpol >= 0) {
		if (b < 0) {
			fxo->lastpol = -1;
			fxo->polaritydebounce = POLARITY_DEBOUNCE / MS_PER_CHECK_HOOK;
		}
	}
	if (fxo->lastpol <= 0) {
		if (b > 0) {
			fxo->lastpol = 1;
			fxo->polaritydebounce = POLARITY_DEBOUNCE / MS_PER_CHECK_HOOK;
		}
	}
	if (fxo->battalarm) {
		if (--fxo->battalarm == 0) {
			/* the alarm timer has expired, so update the battery alarm state
			   for this channel */
			dahdi_alarm_channel(wc->chans[card], fxo->battery== BATTERY_LOST ? DAHDI_ALARM_RED:DAHDI_ALARM_NONE );
		}
	}
	if (fxo->polaritydebounce) {
		if (--fxo->polaritydebounce == 0) {
			if (fxo->lastpol != fxo->polarity) {
				if (debug) {
					printk(KERN_DEBUG "%lu Polarity reversed (%d -> %d)\n", jiffies,
						fxo->polarity,
						fxo->lastpol);
				}
				if (fxo->polarity) {
                                        if (cidsupport) {
						set_cidstart_desc_from_chan_num(wc->span.spanno, wc->chans[card]->channo, wc_dev->cid_state[card]);
                                                if (is_callerid_disable(wc->span.spanno, wc->chans[card]->channo)) {
                                                        dahdi_qevent_lock(wc->chans[card], DAHDI_EVENT_POLARITY);
                                                }
                                        } else {
                                                dahdi_qevent_lock(wc->chans[card], DAHDI_EVENT_POLARITY);
                                        }
				}
				fxo->polarity = fxo->lastpol;
			}
		}
	}
#undef MS_PER_CHECK_HOOK
}


static void a24xx_post_initialize(struct a24xx *wc)
{
	int x;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	/* Finalize signalling  */
	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
		if (wc_dev->cardflag & (1 << x)) {
			if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
				wc->chans[x]->sigcap = DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS | DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
			}
			else {
				wc->chans[x]->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
			}
		} else if (!(wc->chans[x]->sigcap & DAHDI_SIG_BROKEN)) {
			wc->chans[x]->sigcap = 0;
		}
	}
}

static void a24xx_proslic_check_hook(struct a24xx *wc, int card)
{
	char res;
	int hook;
	struct a24xx_dev *wc_dev = &wc->dev;

	/* For some reason we have to debounce the
	   hook detector.  */

	res = wc_dev->reg0shadow[card];
	hook = (res & 1);
	if (hook != wc_dev->mod[card].fxs.lastrxhook) {
		/* Reset the debounce (must be multiple of 4ms) */
		wc_dev->mod[card].fxs.debounce = 8 * (4 * 8);
#if 0
		printk(KERN_DEBUG "Resetting debounce card %d hook %d, %d\n", card, hook, wc_dev->mod[card].fxs.debounce);
#endif
	} else {
		if (wc_dev->mod[card].fxs.debounce > 0) {
			wc_dev->mod[card].fxs.debounce-= 16 * DAHDI_CHUNKSIZE;
#if 0
			printk(KERN_DEBUG "Sustaining hook %d, %d\n", hook, wc_dev->mod[card].fxs.debounce);
#endif
			if (!wc_dev->mod[card].fxs.debounce) {
#if 0
				printk(KERN_DEBUG "Counted down debounce, newhook: %d...\n", hook);
#endif
				wc_dev->mod[card].fxs.debouncehook = hook;
			}
			if (!wc_dev->mod[card].fxs.oldrxhook && wc_dev->mod[card].fxs.debouncehook) {
				/* Off hook */
				if (debug) {
					printk(KERN_DEBUG "opvxa24xx: Card %d Going off hook\n", card);
				}
				switch (wc_dev->mod[card].fxs.lasttxhook) {
				case SLIC_LF_RINGING:		/* Ringing */
				case SLIC_LF_OHTRAN_FWD:	/* Forward On Hook Transfer */
				case SLIC_LF_OHTRAN_REV:	/* Reverse On Hook Transfer */
						/* just detected OffHook, during Ringing or OnHookTransfer */
						wc_dev->mod[card].fxs.idletxhookstate = POLARITY_XOR(card) ?
								SLIC_LF_ACTIVE_REV :
								SLIC_LF_ACTIVE_FWD;
						break;
				}
				dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_OFFHOOK);
				if (robust) {
					si321x_init_proslic(wc_dev, card, 1, 0, 1);
				}
				wc_dev->mod[card].fxs.oldrxhook = 1;

			} else if (wc_dev->mod[card].fxs.oldrxhook && !wc_dev->mod[card].fxs.debouncehook) {
				/* On hook */
				if (debug) {
					printk(KERN_DEBUG "opvxa24xx: Card %d Going on hook\n", card);
				}
				dahdi_hooksig(wc->chans[card], DAHDI_RXSIG_ONHOOK);
				wc_dev->mod[card].fxs.oldrxhook = 0;
			}
		}
	}
	wc_dev->mod[card].fxs.lastrxhook = hook;
}

static void a24xx_transmit(struct a24xx *wc,unsigned int order)
{
	int x, y, pos;
	volatile unsigned char *txbuf;
	struct a24xx_dev *wc_dev = &wc->dev;

	__opvx_a24xx_transmit(wc_dev->mem32, wc_dev->writechunk, &txbuf,ms_per_irq,order);
	//printk("ints is %d\n", ints);
	/* Calculate Transmission */

	for (y=0;y<DAHDI_CHUNKSIZE;y++) {
#ifdef __BIG_ENDIAN
	// operation pending...
#else
		for (x=0;x<wc_dev->max_cards;x++) {
			pos = y * MAX_NUM_CARDS + x;
			txbuf[pos] = wc->chans[x]->writechunk[y];
		}
#endif
	}
}

static void a24xx_receive(struct a24xx *wc,unsigned int order)
{
	int x, y;
	volatile unsigned char *rxbuf;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	__opvx_a24xx_receive(wc_dev->mem32, wc_dev->readchunk, &rxbuf ,ms_per_irq,order);
	for (x=0;x<DAHDI_CHUNKSIZE;x++) {
#ifdef __BIG_ENDIAN
	// operation pending...
#else
		for (y=0;y<wc_dev->max_cards/*MAX_NUM_CARDS*/;y++) {
			if (wc_dev->cardflag & (1 << y)) {
				wc->chans[y]->readchunk[x] = rxbuf[MAX_NUM_CARDS * x + y];
			}
#ifdef TEST_LOG_INCOME_VOICE
			wc_dev->voc_buf[y][wc_dev->voc_ptr[y]] = rxbuf[MAX_NUM_CARDS * x + y];
			wc_dev->voc_ptr[y]++;
			if(wc_dev->voc_ptr[y] >= voc_buffer_size) {
				wc_dev->voc_ptr[y] = 0;
			}
#endif
		}
#endif
	}
	
#if 0
	if( (ints&0x0f) == 0x0f) {    // dump buffer every 16 times; 
		printk("dump 0x%x\n", ints);
		for(x=0; x<48; x++) {
			printk("0x%02x, ", (int)rxbuf[x]);
			//if( 15== (x%15) )
		}
		printk("\n");
	}
#endif

        if(cidsupport) {
                parser_callerid_process(wc, cidbuflen, cidtimeout);
        }

#ifdef AUDIO_RINGCHECK
	for (x=0;x<wc_dev->max_cards;x++) {
		ring_check(wc, x);
	}
#endif
	/* XXX We're wasting 8 taps.  We should get closer :( */
	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
		if (wc_dev->cardflag & (1 << x)) {
			dahdi_ec_chunk(wc->chans[x], wc->chans[x]->readchunk, wc->chans[x]->writechunk);
		}
	}
}

#if 0
DAHDI_IRQ_HANDLER(a24xx_interrupt)
{
	unsigned int ints;
	int x, y, z,order;
	int mode;
	struct a24xx *wc = dev_id;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	struct fxs * fxs ;
	
	ints = __opvx_a24xx_get_irqstatus(wc_dev->mem32);

	if (!ints)
		return IRQ_NONE;

	__opvx_a24xx_set_irqstatus(wc_dev->mem32, ints);    // clear interrupt register.
			
	for (x=0;x<wc_dev->max_cards; x++) {
		if (wc_dev->cardflag & (1 << x) && (wc_dev->modtype[x] == MOD_TYPE_FXS)) {
			 
			fxs = &wc_dev->mod[x].fxs;

			if (SLIC_LF_RINGING == fxs->lasttxhook && !fxs->neonringing) {
					/* RINGing, prepare for OHT */
					fxs->ohttimer = OHT_TIMER << 3;
					/* OHT mode when idle */
					fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_OHTRAN_REV :
							    SLIC_LF_OHTRAN_FWD;
			} else if (fxs->ohttimer) {
		 			/* check if still OnHook */
					if (!fxs->oldrxhook) {
							fxs->ohttimer -= DAHDI_CHUNKSIZE;
							if (fxs->ohttimer)
									continue;

							/* Switch to active */
							fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_ACTIVE_REV :
								    SLIC_LF_ACTIVE_FWD;
							/* if currently OHT */
							if ((fxs->lasttxhook == SLIC_LF_OHTRAN_FWD) || (fxs->lasttxhook == SLIC_LF_OHTRAN_REV)) {
									if (fxs->vmwi_hvac) {
											/* force idle polarity Forward if ringing */
											fxs->idletxhookstate = SLIC_LF_ACTIVE_FWD;
											/* Set ring generator for neon */
											si321x_set_ring_generator_mode(wc_dev, x, 1);
											fxs->lasttxhook = SLIC_LF_RINGING;
									} else {
											fxs->lasttxhook = fxs->idletxhookstate;
									}
									/* Apply the change as appropriate */
									a24xx_spi_setreg(wc_dev, x, LINE_STATE, fxs->lasttxhook);
							}
					} else {
							fxs->ohttimer = 0;
							/* Switch to active */
							fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_ACTIVE_REV : SLIC_LF_ACTIVE_FWD;
							printk("Channel %d OnHookTransfer abort\n",x);
					}
			}	
		}
	}

	if (ints & (1<<16)) {       /* it is our interrupts */
		wc_dev->intcount++;
		switch(ms_per_irq){
		case 1:
				z = wc_dev->intcount & 0x3;
				mode = wc_dev->intcount & 0xc;
				for(y = 0; y < wc_dev->max_cards / 4; y++) { /* do some detect operate every 4ms */ 
						x = z + y*4;
						if (wc_dev->cardflag & (1 << x ) ) {
								switch(mode) {
								case 0:
										/* Rest */
										break;
								case 4:
										/* Read first shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												//if(x==0)
						    						//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
										break;
								case 8:
										/* Read second shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												//if(x==1)
												//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
										break;
								case 12:
										/* Perform processing */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & 0xf0)) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
										} else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
												a24xx_voicedaa_check_hook(wc, x);
										}
										break;
								}
						}
				}
				break;
		case 2:
				z = wc_dev->intcount & 0x1;
				mode = wc_dev->intcount & 0x6;
				for(y = 0; y < wc_dev->max_cards / 2; y++) { /* do some detect operate every 4ms */ 
						x = z + y*2;
						if (wc_dev->cardflag & (1 << x ) ) {
								switch(mode) {
								case 0:
										/* Rest */
										break;
								case 2:
										/* Read first shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												//if(x==0)
						    						//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
										break;
								case 4:
										/* Read second shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												//if(x==1)
												//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
										break;
								case 6:
										/* Perform processing */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & (0xf0>>1))) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
										} else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
												a24xx_voicedaa_check_hook(wc, x);
										}
										break;
								}
						}
				}
				break;
		case 4:
				mode = wc_dev->intcount & 0x3;
				for(x=0;x<wc_dev->max_cards;x++){
						if (wc_dev->cardflag & (1 << x ) ) {
								switch(mode) {
								case 0:
										/* Rest */
										break;
								case 1:
										/* Read first shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												//if(x==0)
						    						//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
										break;
								case 2:
										/* Read second shadow reg */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												//if(x==1)
												//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
										break;
								case 3:
										/* Perform processing */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & (0xf0>>2))) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
										} 
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
												a24xx_voicedaa_check_hook(wc, x);
										}
										break;
								}
						}
				}
				break;
		case 8:
				mode = wc_dev->intcount & 0x1;
				for(x=0;x<wc_dev->max_cards;x++){
						if (wc_dev->cardflag & (1 << x ) ) {
								switch(mode) {
								case 1:
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO){
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
										}							
										break;
								case 0:
										/* Perform processing */
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & (0xf0>>3))) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
										} 
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
												a24xx_voicedaa_check_hook(wc, x);
										}
										break;
								}
						}
				}
				break;
		case 16:
				for(x=0;x<wc_dev->max_cards;x++){
						if (wc_dev->cardflag & (1 << x ) ) {
									if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & (0xf0>>4))) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
									}
									else if (wc_dev->modtype[x] == MOD_TYPE_FXO){
											wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
											wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
											a24xx_voicedaa_check_hook(wc, x);
									}
						}
				}
				break;	
		}
		
		if (!(wc_dev->intcount % ( 10000/ms_per_irq ))) {
			/* Accept an alarm once per 10 seconds */
			for (x=0;x<wc_dev->max_cards;x++)
				if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
					if (wc_dev->mod[x].fxs.palarms) {
						wc_dev->mod[x].fxs.palarms--;
					}
				}
		}
		/**/
		for(order=0;order < ms_per_irq;order++){
				a24xx_receive(wc, order);
				dahdi_receive(&wc->span);

				dahdi_transmit(&wc->span);
				a24xx_transmit(wc, order);
		}
	}
	
	return IRQ_RETVAL(1);
}
#endif

#if 1
static int interrupt_onecard_handler(struct a24xx *wc)
{		
		unsigned int ints=0;
		int x, y, z,order;
		int mode;
		struct a24xx_dev *wc_dev = &wc->dev;
		
		struct fxs * fxs ;
		
		if((wc_dev->fwversion < ((1 << 16)|3)) || (wc_dev->master)){
				ints = __opvx_a24xx_get_irqstatus(wc_dev->mem32);
				if (!ints)
						return IRQ_NONE;
		}

		__opvx_a24xx_set_irqstatus(wc_dev->mem32, ints);    // clear interrupt register.

		for (x=0;x<wc_dev->max_cards; x++) {
				if (wc_dev->cardflag & (1 << x) && (wc_dev->modtype[x] == MOD_TYPE_FXS)) {
						fxs = &wc_dev->mod[x].fxs;

				if (SLIC_LF_RINGING == fxs->lasttxhook && !fxs->neonringing) {
					/* RINGing, prepare for OHT */
					fxs->ohttimer = OHT_TIMER << 3;
					/* OHT mode when idle */
					fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_OHTRAN_REV :
							    SLIC_LF_OHTRAN_FWD;
				} else if (fxs->ohttimer) {
		 			/* check if still OnHook */
					if (!fxs->oldrxhook) {
							fxs->ohttimer -= DAHDI_CHUNKSIZE;
							if (fxs->ohttimer)
									continue;

							/* Switch to active */
							fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_ACTIVE_REV :
								    			SLIC_LF_ACTIVE_FWD;
							/* if currently OHT */
							if ((fxs->lasttxhook == SLIC_LF_OHTRAN_FWD) || (fxs->lasttxhook == SLIC_LF_OHTRAN_REV)) {
									if (fxs->vmwi_hvac) {
											/* force idle polarity Forward if ringing */
											fxs->idletxhookstate = SLIC_LF_ACTIVE_FWD;
											/* Set ring generator for neon */
											si321x_set_ring_generator_mode(wc_dev, x, 1);
											fxs->lasttxhook = SLIC_LF_RINGING;
									} else {
											fxs->lasttxhook = fxs->idletxhookstate;
									}
									/* Apply the change as appropriate */
									a24xx_spi_setreg(wc_dev, x, LINE_STATE, fxs->lasttxhook);
							}
					} else {
							fxs->ohttimer = 0;
							/* Switch to active */
							fxs->idletxhookstate = POLARITY_XOR(x) ? SLIC_LF_ACTIVE_REV : SLIC_LF_ACTIVE_FWD;
							printk("Channel %d OnHookTransfer abort\n",x);
					}
				}	
			}
		}

		if ( ((ints & (1<<16)) && wc_dev->master) || (!wc_dev->master)) {       /* it is our interrupts */
				wc_dev->intcount++;
				switch(ms_per_irq){
				case 1:
						z = wc_dev->intcount & 0x3;
						mode = wc_dev->intcount & 0xc;
						for(y = 0; y < wc_dev->max_cards / 4; y++) { /* do some detect operate every 4ms */ 
								x = z + y*4;
								if (wc_dev->cardflag & (1 << x ) ) {
										switch(mode) {
										case 0:
												/* Rest */
												break;
										case 4:
												/* Read first shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
														//if(x==0)
						    								//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
												break;
										case 8:
												/* Read second shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
														//if(x==1)
														//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
												break;
										case 12:
												/* Perform processing */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														a24xx_proslic_check_hook(wc, x);
														if (!(wc_dev->intcount & 0xf0)) {
																si321x_proslic_recheck_sanity(wc_dev, x);
														}
												} else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
														a24xx_voicedaa_check_hook(wc, x);
												}
												break;
										}
								}
						}
						break;
				case 2:
						z = wc_dev->intcount & 0x1;
						mode = wc_dev->intcount & 0x6;
						for(y = 0; y < wc_dev->max_cards / 2; y++) { /* do some detect operate every 4ms */ 
								x = z + y*2;
								if (wc_dev->cardflag & (1 << x ) ) {
										switch(mode) {
										case 0:
												/* Rest */
												break;
										case 2:
												/* Read first shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
														//if(x==0)
						    								//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
												break;
										case 4:
												/* Read second shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
														//if(x==1)
														//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
												break;
										case 6:
												/* Perform processing */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														a24xx_proslic_check_hook(wc, x);
														if (!(wc_dev->intcount & (0xf0>>1))) {
																si321x_proslic_recheck_sanity(wc_dev, x);
														}
												} else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
														a24xx_voicedaa_check_hook(wc, x);
												}
												break;
										}
								}
						}
						break;
				case 4:
						mode = wc_dev->intcount & 0x3;
						for(x=0;x<wc_dev->max_cards;x++){
								if (wc_dev->cardflag & (1 << x ) ) {
										switch(mode) {
										case 0:
												/* Rest */
												break;
										case 1:
												/* Read first shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
														//if(x==0)
						    								//printk("reg 68 of %x is 0x%x\n", x, wc_dev->reg0shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
												break;
										case 2:
												/* Read second shadow reg */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
														//if(x==1)
														//    printk("reg 64 of %x is 0x%x\n", x, wc_dev->reg1shadow[x]);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO)
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
												break;
										case 3:
												/* Perform processing */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														a24xx_proslic_check_hook(wc, x);
														if (!(wc_dev->intcount & (0xf0>>2))) {
																si321x_proslic_recheck_sanity(wc_dev, x);
														}
												} 
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
														a24xx_voicedaa_check_hook(wc, x);
												}
												break;
										}
								}
						}
						break;
				case 8:
						mode = wc_dev->intcount & 0x1;
						for(x=0;x<wc_dev->max_cards;x++){
								if (wc_dev->cardflag & (1 << x ) ) {
										switch(mode) {
										case 1:
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												}
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO){
														wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
														wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
												}							
												break;
										case 0:
												/* Perform processing */
												if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
														a24xx_proslic_check_hook(wc, x);
														if (!(wc_dev->intcount & (0xf0>>3))) {
																si321x_proslic_recheck_sanity(wc_dev, x);
														}
												} 
												else if (wc_dev->modtype[x] == MOD_TYPE_FXO) {
														a24xx_voicedaa_check_hook(wc, x);
												}
												break;
										}
								}
						}
						break;
				case 16:
						for(x=0;x<wc_dev->max_cards;x++){
								if (wc_dev->cardflag & (1 << x ) ) {
										if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 68);
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 64);
												a24xx_proslic_check_hook(wc, x);
												if (!(wc_dev->intcount & (0xf0>>4))) {
														si321x_proslic_recheck_sanity(wc_dev, x);
												}
										}
										else if (wc_dev->modtype[x] == MOD_TYPE_FXO){
												wc_dev->reg0shadow[x] = a24xx_spi_getreg(wc_dev, x, 5);
												wc_dev->reg1shadow[x] = a24xx_spi_getreg(wc_dev, x, 29);
												a24xx_voicedaa_check_hook(wc, x);
										}
								}
						}
						break;	
				}
		
				if (!(wc_dev->intcount % ( 10000/ms_per_irq ))) {
						/* Accept an alarm once per 10 seconds */
						for (x=0;x<wc_dev->max_cards;x++)
						if (wc_dev->modtype[x] == MOD_TYPE_FXS) {
								if (wc_dev->mod[x].fxs.palarms) {
										wc_dev->mod[x].fxs.palarms--;
								}
						}
				}
				/**/
				for(order=0;order < ms_per_irq;order++){
						dahdi_transmit(&wc->span);
						parser_busy_silent_process(wc, 1);
						a24xx_transmit(wc, order);
						
						a24xx_receive(wc, order);
						parser_busy_silent_process(wc, 0);
						dahdi_receive(&wc->span);	
				}		
		}
		return 0;	
}

DAHDI_IRQ_HANDLER(a24xx_interrupt)
{
	/**/
	struct a24xx *wc = dev_id;
	struct a24xx_dev *wc_dev = &wc->dev;
	int i;
		
	if(!wc_dev->master){
			interrupt_onecard_handler(wc);
	}else{ 
		if(irq_stub){
			for(i=0;i<max_iface_index;i++){
					wc = ifaces[i];
					if(wc){
							wc_dev = &wc->dev;
							if( wc_dev->fwversion >= ((1 << 16)|2) ){
									interrupt_onecard_handler(wc);
							}
					}
			}
		}else
			interrupt_onecard_handler(wc);
	}
		
		
	return IRQ_RETVAL(1);
}
#endif
/* Must be called from within an interruptible context */
static int set_vmwi(struct a24xx *wc, int chan_idx)
{
	struct a24xx_dev *wc_dev = &wc->dev;
	struct fxs *const fxs = &wc_dev->mod[chan_idx].fxs;

	if (fxs->vmwi_active_messages) {
		fxs->vmwi_lrev =
		    (fxs->vmwisetting.vmwi_type & DAHDI_VMWI_LREV) ? 1 : 0;
		fxs->vmwi_hvdc =
		    (fxs->vmwisetting.vmwi_type & DAHDI_VMWI_HVDC) ? 1 : 0;
		fxs->vmwi_hvac =
		    (fxs->vmwisetting.vmwi_type & DAHDI_VMWI_HVAC) ? 1 : 0;
	} else {
		fxs->vmwi_lrev = 0;
		fxs->vmwi_hvdc = 0;
		fxs->vmwi_hvac = 0;
	}
	
	if (debug) {
		printk(KERN_DEBUG "Setting VMWI on channel %d, messages=%d, "
				"lrev=%d, hvdc=%d, hvac=%d\n",
				chan_idx,
				fxs->vmwi_active_messages,
				fxs->vmwi_lrev,
				fxs->vmwi_hvdc,
				fxs->vmwi_hvac
			  );
	}
	
	if (fxs->vmwi_hvac) {
		/* Can't change ring generator while in On Hook Transfer mode*/
		if (!fxs->ohttimer) {
			if (POLARITY_XOR(chan_idx))
				fxs->idletxhookstate |= SLIC_LF_REVMASK;
			else
				fxs->idletxhookstate &= ~SLIC_LF_REVMASK;
			/* Set ring generator for neon */
			si321x_set_ring_generator_mode(wc_dev, chan_idx, 1);
			/* Activate ring to send neon pulses */
			fxs->lasttxhook = SLIC_LF_RINGING;
			a24xx_spi_setreg(wc_dev, chan_idx, LINE_STATE, fxs->lasttxhook);
		}
	} else {
		if (fxs->neonringing) {
			/* Set ring generator for normal ringer */
			si321x_set_ring_generator_mode(wc_dev, chan_idx, 0);
			/* ACTIVE, polarity determined later */
			fxs->lasttxhook = SLIC_LF_ACTIVE_FWD;
		} else if ((fxs->lasttxhook == SLIC_LF_RINGING) ||
					(fxs->lasttxhook == SLIC_LF_OPEN)) {
			/* Can't change polarity while ringing or when open,
				set idlehookstate instead */
			if (POLARITY_XOR(chan_idx))
				fxs->idletxhookstate |= SLIC_LF_REVMASK;
			else
				fxs->idletxhookstate &= ~SLIC_LF_REVMASK;
			if(debug)
					printk(KERN_DEBUG "Unable to change polarity on channel"
					    "%d, lasttxhook=0x%X\n",
				chan_idx,
				fxs->lasttxhook
			);
			return 0;
		}
		if (POLARITY_XOR(chan_idx)) {
			fxs->idletxhookstate |= SLIC_LF_REVMASK;
			fxs->lasttxhook |= SLIC_LF_REVMASK;
		} else {
			fxs->idletxhookstate &= ~SLIC_LF_REVMASK;
			fxs->lasttxhook &= ~SLIC_LF_REVMASK;
		}
		a24xx_spi_setreg(wc_dev, chan_idx, LINE_STATE, fxs->lasttxhook);
	}
	return 0;
}

static int a24xx_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	struct wctdm_stats stats;
	struct wctdm_regs regs;
	struct wctdm_regop regop;
	struct wctdm_echo_coefs echoregs;
	struct dahdi_hwgain hwgain;
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;	
	int x;
	
	struct fxs *const fxs = &wc_dev->mod[chan->chanpos - 1].fxs;
	
	switch (cmd) {
	case DAHDI_ONHOOKTRANSFER:
		if (wc_dev->modtype[chan->chanpos - 1] != MOD_TYPE_FXS) {
			return -EINVAL;
		}
		if (get_user(x, (__user int *)data)) {
			return -EFAULT;
		}
#if 0
		/**/
		wc_dev->mod[chan->chanpos - 1].fxs.ohttimer = x << 3;
		if (reversepolarity) {
			wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = 0x6;	/* OHT mode when idle */
		} else {
			wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = 0x2;
		}
		if (wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook == 0x1 || wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook == 0x5) {
			/* Apply the change if appropriate */
			if (reversepolarity) {
				wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook = 0x6;
			} else {
				wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook = 0x2;
			}
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 64, wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook);
		}
#endif
#if 1
		/* Active mode when idle */
		wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = POLARITY_XOR(chan->chanpos - 1) ?
				SLIC_LF_ACTIVE_REV : SLIC_LF_ACTIVE_FWD;
		if (wc_dev->mod[chan->chanpos - 1].fxs.neonringing) {
			/* keep same Forward polarity */
			wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook = SLIC_LF_OHTRAN_FWD;
			//printk(KERN_INFO "ioctl: Start OnHookTrans, card %d\n",chan->chanpos - 1);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1,
					LINE_STATE, wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook);
		} else if (wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook == SLIC_LF_ACTIVE_FWD ||
			    wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook == SLIC_LF_ACTIVE_REV) {
			/* Apply the change if appropriate */
			wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook = POLARITY_XOR(chan->chanpos - 1) ?
				SLIC_LF_OHTRAN_REV : SLIC_LF_OHTRAN_FWD;
			//printk(KERN_INFO "ioctl: Start OnHookTrans, card %d\n",chan->chanpos - 1);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1,
					LINE_STATE, wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook);
		}
#endif		
		break;
#if 1
	case DAHDI_VMWI_CONFIG:
		if (wc_dev->modtype[chan->chanpos - 1] != MOD_TYPE_FXS)
			return -EINVAL;
		if (copy_from_user(&(fxs->vmwisetting),
				   (__user void *)data,
				   sizeof(fxs->vmwisetting)))
			return -EFAULT;
		//printk("--opvxa24xx:DAHDI_VMWI_CONFIG,card=%d--\n",chan->chanpos - 1);
		set_vmwi(wc, chan->chanpos - 1);
		break;
	case DAHDI_VMWI:
		if (wc_dev->modtype[chan->chanpos - 1] != MOD_TYPE_FXS)
			return -EINVAL;
		if (get_user(x, (__user int *) data))
			return -EFAULT;
		if (0 > x)
			return -EFAULT;
		fxs->vmwi_active_messages = x;
		set_vmwi(wc, chan->chanpos - 1);
		//printk("-----opvxa24xx,DAHDI_VMWI,card=%d,set_vmwi------\n",chan->chanpos - 1);
		break;
#endif
	case DAHDI_SETPOLARITY:
		if (get_user(x, (__user int *)data)) {
			return -EFAULT;
		}
		if (wc_dev->modtype[chan->chanpos - 1] != MOD_TYPE_FXS) {
			return -EINVAL;
		}
		/* Can't change polarity while ringing or when open */
		if ((wc_dev->mod[chan->chanpos -1 ].fxs.lasttxhook == SLIC_LF_RINGING) ||
			(wc_dev->mod[chan->chanpos -1 ].fxs.lasttxhook == SLIC_LF_OPEN)) {
			return -EINVAL;
		}
		wc_dev->mod[chan->chanpos -1 ].fxs.reversepolarity = x;
#if 0
		if ((x && !reversepolarity) || (!x && reversepolarity)) {
			wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook |= 0x04;
		} else {
			wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook &= ~0x04;
		}
#endif
		if (POLARITY_XOR(chan->chanpos - 1)) {
			wc_dev->mod[chan->chanpos -1 ].fxs.lasttxhook |= SLIC_LF_REVMASK;
			printk(KERN_INFO "ioctl: Reverse Polarity, card %d\n",
					chan->chanpos - 1);
		} else {
			wc_dev->mod[chan->chanpos -1 ].fxs.lasttxhook &= ~SLIC_LF_REVMASK;
			printk(KERN_INFO "ioctl: Normal Polarity, card %d\n",
					chan->chanpos - 1);
		}
		a24xx_spi_setreg(wc_dev, chan->chanpos - 1, LINE_STATE, wc_dev->mod[chan->chanpos - 1].fxs.lasttxhook);
		break;
	case WCTDM_GET_STATS:
		if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXS) {
			stats.tipvolt = a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 80) * -376;
			stats.ringvolt = a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 81) * -376;
			stats.batvolt = a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 82) * -376;
		} else if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXO) {
			stats.tipvolt = (signed char)a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 29) * 1000;
			stats.ringvolt = (signed char)a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 29) * 1000;
			stats.batvolt = (signed char)a24xx_spi_getreg(wc_dev, chan->chanpos - 1, 29) * 1000;
		} else {
			return -EINVAL;
		}
		if (copy_to_user((__user void*)data, &stats, sizeof(stats))) {
			return -EFAULT;
		}
		break;
	case WCTDM_GET_REGS:
		if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXS) {
			for (x=0;x<NUM_INDIRECT_REGS;x++) {
				regs.indirect[x] = si321x_proslic_getreg_indirect(wc_dev, chan->chanpos -1, x);
			}
			for (x=0;x<NUM_REGS;x++) {
				regs.direct[x] = a24xx_spi_getreg(wc_dev, chan->chanpos - 1, x);
			}
		} else {
			memset(&regs, 0, sizeof(regs));
			for (x=0;x<NUM_FXO_REGS;x++)
				regs.direct[x] = a24xx_spi_getreg(wc_dev, chan->chanpos - 1, x);
		}
		if (copy_to_user((__user void *)data, &regs, sizeof(regs))) {
			return -EFAULT;
		}
		break;
	case WCTDM_SET_REG:
		if (copy_from_user(&regop, (__user void *)data, sizeof(regop))) {
			return -EFAULT;
		}
		if (regop.indirect) {
			if (wc_dev->modtype[chan->chanpos - 1] != MOD_TYPE_FXS) {
				return -EINVAL;
			}
			printk(KERN_INFO "Setting indirect %d to 0x%04x on %d\n", regop.reg, regop.val, chan->chanpos);
			si321x_proslic_setreg_indirect(wc_dev, chan->chanpos - 1, regop.reg, regop.val);
		} else {
			regop.val &= 0xff;
			printk(KERN_INFO "Setting direct %d to %04x on %d\n", regop.reg, regop.val, chan->chanpos);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, regop.reg, regop.val);
		}
		break;
	case WCTDM_SET_ECHOTUNE:
		printk(KERN_INFO "-- Setting echo registers: \n");
		if (copy_from_user(&echoregs, (__user void *)data, sizeof(echoregs))) {
			return -EFAULT;
		}

		if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXO) {
			/* Set the ACIM register */
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 30, (fxofullscale==1) ? (echoregs.acim|0x10) : echoregs.acim);

			/* Set the digital echo canceller registers */
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 45, echoregs.coef1);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 46, echoregs.coef2);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 47, echoregs.coef3);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 48, echoregs.coef4);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 49, echoregs.coef5);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 50, echoregs.coef6);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 51, echoregs.coef7);
			a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 52, echoregs.coef8);

			printk(KERN_INFO "-- Set echo registers successfully\n");

			break;
		} else {
			return -EINVAL;
		}
		break;
	case DAHDI_SET_HWGAIN:
		if (copy_from_user(&hwgain, (__user void *) data, sizeof(hwgain))) {
			return -EFAULT;
		}

		si3050_set_hwgain(wc_dev, chan->chanpos-1, hwgain.newgain, hwgain.tx);

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

static int _a24xx_open(struct dahdi_chan *chan)
{
	
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	if (!(wc_dev->cardflag & (1 << (chan->chanpos - 1)))) {
		return -ENODEV;
	}
	if (wc_dev->dead) {
		return -ENODEV;
	}
	wc_dev->usecount++;
	
	try_module_get(THIS_MODULE);
	
	return 0;
}

static int a24xx_open(struct dahdi_chan *chan)
{
	unsigned long flags;
	int res;
	spin_lock_irqsave(&chan->lock, flags);
	res = _a24xx_open(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
	return res;
}

static inline struct a24xx *a24xx_from_span(struct dahdi_span *span)
{
	return container_of(span, struct a24xx, span);
}

static int a24xx_watchdog(struct dahdi_span *span, int event)
{
	struct a24xx *wc = a24xx_from_span(span);
	struct a24xx_dev *wc_dev = &wc->dev;
	
	printk(KERN_INFO "opvxa24xx: Restarting DMA\n");
	__opvx_a24xx_restart_dma(wc_dev->mem32);
	
	return 0;
}

static int a24xx_close(struct dahdi_chan *chan)
{
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	wc_dev->usecount--;

	module_put(THIS_MODULE);

	if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXS) {
		/*
		if (reversepolarity) {
			wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = 5;
		}
		else {
			wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = 1;
		}*/
		int idlehookstate;
		idlehookstate = POLARITY_XOR(chan->chanpos - 1)?
						SLIC_LF_ACTIVE_REV :
						SLIC_LF_ACTIVE_FWD;
		wc_dev->mod[chan->chanpos - 1].fxs.idletxhookstate = idlehookstate;
	}
	/* If we're dead, release us now */
	if (!wc_dev->usecount && wc_dev->dead) {
		a24xx_release(wc);
	}
	return 0;
}

static int a24xx_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
	int reg=0;
	struct a24xx *wc = chan->pvt;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	if (wc_dev->modtype[chan->chanpos - 1] == MOD_TYPE_FXO) {
		/* XXX Enable hooksig for FXO XXX */
		switch(txsig) {
			case DAHDI_TXSIG_START:
			case DAHDI_TXSIG_OFFHOOK:
				wc_dev->mod[chan->chanpos - 1].fxo.offhook = 1;
				a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 5, 0x9);
				if(cidsupport) {
					wc_dev->cid_state[chan->chanpos - 1] = CID_STATE_IDLE;
					wc_dev->cid_history_clone_cnt[chan->chanpos - 1] = 0;
					wc_dev->cid_history_ptr[chan->chanpos - 1] = 0;
					memset(wc_dev->cid_history_buf[chan->chanpos - 1], DAHDI_LIN2X(0, chan), cidbuflen * DAHDI_MAX_CHUNKSIZE);
				}
				break;
			case DAHDI_TXSIG_ONHOOK:
				wc_dev->mod[chan->chanpos - 1].fxo.offhook = 0;
				a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 5, 0x8);
				break;
			default:
				printk(KERN_NOTICE "wcfxo: Can't set tx state to %d\n", txsig);
			}
	} else {
		switch(txsig) {
			case DAHDI_TXSIG_ONHOOK:
				switch(chan->sig) {
					case DAHDI_SIG_EM:
					case DAHDI_SIG_FXOKS:
					case DAHDI_SIG_FXOLS:
						//wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = wc_dev->mod[chan->chanpos-1].fxs.idletxhookstate;
						/* Can't change Ring Generator during OHT */
						if (!wc_dev->mod[chan->chanpos-1].fxs.ohttimer) {
								si321x_set_ring_generator_mode(wc_dev,
					    			chan->chanpos-1, wc_dev->mod[chan->chanpos-1].fxs.vmwi_hvac);
								wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = wc_dev->mod[chan->chanpos-1].fxs.vmwi_hvac ? SLIC_LF_RINGING : wc_dev->mod[chan->chanpos-1].fxs.idletxhookstate;
						} else {
								wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = wc_dev->mod[chan->chanpos-1].fxs.idletxhookstate;
						}
						break;
					case DAHDI_SIG_FXOGS:
						wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = 3;
						break;
				}
				break;
			case DAHDI_TXSIG_OFFHOOK:
				switch(chan->sig) {
				case DAHDI_SIG_EM:
					wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = 5;
					break;
				default:
					wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = wc_dev->mod[chan->chanpos-1].fxs.idletxhookstate;
					break;
				}
				break;
			case DAHDI_TXSIG_START:
				//wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = 4;
				si321x_set_ring_generator_mode(wc_dev,
					    			chan->chanpos-1, 0);
				wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = SLIC_LF_RINGING;
				break;
			case DAHDI_TXSIG_KEWL:
				wc_dev->mod[chan->chanpos-1].fxs.lasttxhook = 0;
				break;
			default:
				printk(KERN_NOTICE "opvxa24xx: Can't set tx state to %d\n", txsig);
		}
		if (debug) {
			printk(KERN_DEBUG "Setting FXS hook state to %d (%02x)\n", txsig, reg);
		}

#if 1
		a24xx_spi_setreg(wc_dev, chan->chanpos - 1, 64, wc_dev->mod[chan->chanpos-1].fxs.lasttxhook);
#endif
	}
	return 0;
}

#ifdef DAHDI_SPAN_OPS
static const struct dahdi_span_ops a24xx_span_ops = {
	.owner = THIS_MODULE,
	.hooksig = a24xx_hooksig,
	.open = a24xx_open,
	.close = a24xx_close,
	.ioctl = a24xx_ioctl,
	.watchdog = a24xx_watchdog,
#ifdef VPM_SUPPORT
	.echocan_create = a24xx_echocan_create,
#endif

};
#endif

static int a24xx_init_spans(struct a24xx *wc)
{
	int x;
	struct a24xx_dev *wc_dev = &(wc->dev);

	/* Zapata stuff */
	sprintf(wc->span.name, "OPVXA24XX/%d", wc_dev->pos);
	snprintf(wc->span.desc, sizeof(wc->span.desc)-1, "%s Board %d", wc_dev->variety, wc_dev->pos + 1);
//	snprintf(wc->span.location, sizeof(wc->span.location) - 1,
//		"PCI Bus %02d Slot %02d", wc_dev->dev->bus->number, PCI_SLOT(wc_dev->dev->devfn) + 1);
  wc_dev->ddev->location = kasprintf(GFP_KERNEL,
				      "PCI Bus %02d Slot %02d",
				      wc_dev->dev->bus->number,
				      PCI_SLOT(wc_dev->dev->devfn) + 1);
				      
  //printk("wc is 0x%x, wc_dev ix 0x%x, ddev is 0x%x\n", wc, &(wc->dev), wc_dev->ddev);
	if (!wc_dev->ddev->location) {
		dahdi_free_device(wc_dev->ddev);
		wc_dev->ddev = NULL;
		return -ENOMEM;
	}

	wc_dev->ddev->manufacturer = "OpenVox";    //Dennis
	wc_dev->ddev->devicetype = wc_dev->variety;

	if (alawoverride) {
		printk(KERN_INFO "ALAW override parameter detected.  Device will be operating in ALAW\n");
		wc->span.deflaw = DAHDI_LAW_ALAW;
	} else {
		wc->span.deflaw = DAHDI_LAW_MULAW;
	}

	for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
		sprintf(wc->chans[x]->name, "OPVXA24XX/%d/%d", wc_dev->pos, x);
		wc->chans[x]->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
		wc->chans[x]->sigcap |= DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS | DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
		wc->chans[x]->chanpos = x+1;
		wc->chans[x]->pvt = wc;
	}

#ifdef DAHDI_SPAN_MODULE	
	wc->span.owner = THIS_MODULE;
#endif
#ifdef DAHDI_SPAN_OPS
	wc->span.ops = &a24xx_span_ops;
#else
	wc->span.hooksig = a24xx_hooksig;
	wc->span.open = a24xx_open;
	wc->span.close = a24xx_close;
	wc->span.ioctl = a24xx_ioctl;
	wc->span.watchdog = a24xx_watchdog;
#ifdef VPM_SUPPORT
	if (vpmsupport)
		wc->span.echocan_create = a24xx_echocan_create;
#endif			
	wc->span.pvt = wc;
#endif
	wc->span.chans = wc->chans;
	wc->span.channels = wc_dev->max_cards;	/*MAX_NUM_CARDS;*/
	wc->span.flags = DAHDI_FLAG_RBS;
//	init_waitqueue_head(&wc->span.maintq);
	list_add_tail(&wc->span.device_node, &wc->dev.ddev->spans);
//	init_waitqueue_head(&wc->regq)
	
	if (dahdi_register_device(wc_dev->ddev, &wc->dev.dev->dev)) {
		printk(KERN_NOTICE "Unable to register span with Dahdi\n");   
		kfree(wc_dev->ddev->location);
		dahdi_free_device(wc_dev->ddev);
		wc_dev->ddev = NULL;
		printk("dahdi_register_device fail\n");
		mdelay(3000);
		return -1;
	}
	return 0;
}

static int __devinit a24xx_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int res;
	int x;
	int y;
	struct a24xx *wc;
	struct a24xx_desc *d = (struct a24xx_desc *)ent->driver_data;
	struct a24xx_dev *wc_dev;
	unsigned int fwbuild;
	char span_flags[MAX_NUM_CARDS] = { 0, };
	int z,index;
	static int initd_ifaces=0;
	
	if(!initd_ifaces){
		memset((void *)ifaces,0,(sizeof(struct a24xx *))*WC_MAX_IFACES);
		initd_ifaces=1;
	}
	for (x = 0; x < WC_MAX_IFACES; x++) {
		if (!ifaces[x]) {
			break;
		}
	}
	if (x >= WC_MAX_IFACES) {
		printk("Too many interfaces\n");
		return -EIO;
	}
	index = x;
	
	if (pci_enable_device(pdev)) {
		res = -EIO;
	} else {
	  int cardcount = 0;
	  
		wc = kmalloc(sizeof(struct a24xx), GFP_KERNEL);
		if(!wc)
		    return -ENOMEM;
		
		memset(wc, 0, sizeof(struct a24xx));
		wc->dev.ddev = dahdi_create_device();
#if 0		
		printk("wc is 0x%x, ddev is 0x%x\n", wc, wc->dev.ddev);
#endif
		if (!wc->dev.ddev) {
		  kfree(wc);
			return -ENOMEM;
		}
		wc_dev = &wc->dev;
		wc_dev->ledstate = 0;
		
		for (x=0; x < sizeof(wc->chans)/sizeof(wc->chans[0]); ++x) {
			wc->chans[x] = &wc->_chans[x];
		}

		spin_lock_init(&wc_dev->lock);
		wc_dev->curcard = -1;

		if(ent->device == 0x0810)
				wc_dev->max_cards = 8;
		else if(ent->device == 0x1610)
				wc_dev->max_cards = 16;
		else
				wc_dev->max_cards = 24;
				
		wc_dev->mem_region = pci_resource_start(pdev, 0);
		wc_dev->mem_len = pci_resource_len(pdev, 0);
		wc_dev->mem32 = (unsigned long)ioremap(wc_dev->mem_region, wc_dev->mem_len);
		wc_dev->dev = pdev;
		wc_dev->pos = x;
		wc_dev->variety = d->name;
		for (y = 0; y < wc_dev->max_cards; y++) {
			wc_dev->flags[y] = d->flags;
		}

		/* Keep track of whether we need to free the region */
		if (request_mem_region(wc_dev->mem_region, wc_dev->mem_len, "opvxa24xx")) {
			wc_dev->freeregion = 1;
		}

		if(debug) {
			printk("======= find a card @ mem32 0x%x, size %ud\n", (unsigned int)wc_dev->mem32, (unsigned int)wc_dev->mem_len);
		}
#if 0
    printk("======= manual exit\n");
		if(wc_dev->freeregion) {
			release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
			iounmap((void *)wc_dev->mem32);
			return -ENOMEM;
		}
#endif
		
		wc_dev->fwversion = __opvx_a24xx_get_version(wc_dev->mem32);
		if(wc_dev->max_cards == 24)
				wc_dev->card_name = A2410P_Name;
		else if(wc_dev->max_cards == 16)
				wc_dev->card_name = A1610P_Name;
		else if(wc_dev->max_cards == 8)
				wc_dev->card_name = A810P_Name;
		
		if(wc_dev->fwversion < ((1 << 16)|1) )
				ms_per_irq = 1;
		
		if((ms_per_irq != 1)&&
			(ms_per_irq != 2)&&
			(ms_per_irq != 4)&&
			(ms_per_irq != 8)&&
			(ms_per_irq != 16))
			ms_per_irq = 1;
			
		if(wc_dev->fwversion > ((1 << 16)|2)){	
				if(!index)
			 			wc_dev->master = 1;		//master card
				else
						wc_dev->master = 0;
				if(!irq_stub)
						wc_dev->master = 1;
		}else
				wc_dev->master = 0;	
				
		wc->index = index;
			
		/* Allocate enough memory for two dahdi chunks, receive and transmit.  Each sample uses
		   8 bits.  */	
		wc_dev->writechunk = pci_alloc_consistent(pdev, ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, &wc_dev->writedma);
		if (!wc_dev->writechunk) {
			printk("opvxa24xx: Unable to allocate DMA-able memory\n");
			if (wc_dev->freeregion) {
				release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
				iounmap((void *)wc_dev->mem32);
			}
			return -ENOMEM;
		}

		if(debug) {
			printk("opvxa24xx: dma buffer allocated at %p, pci(%p)\n", wc_dev->writechunk, (void *)wc_dev->writedma);
		}

		__a24xx_malloc_chunk(wc_dev,ms_per_irq);

		if (a24xx_init_spans(wc)) {
			printk(KERN_NOTICE "opvxa24xx: Unable to intialize hardware\n");
			/* Free Resources */
			if (wc_dev->freeregion) {
				release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
				iounmap((void *)wc_dev->mem32);
			}
			pci_free_consistent(pdev,  ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, (void *)wc_dev->writechunk, wc_dev->writedma);
			return -ENOMEM;
		}
		
		/* Enable bus mastering */
		pci_set_master(pdev);

		/* Keep track of which device we are */
		pci_set_drvdata(pdev, wc);

		/* kmalloc ec */
		for (x = 0; x < wc_dev->max_cards; x++) {
			if (!(wc->ec[x] = kmalloc(sizeof(*wc->ec[x]), GFP_KERNEL))) {
				free_wc(wc);
				return -ENOMEM;
			}
		}

		/* init hardware */
#if 0
		a24xx_hardware_init(wc_dev, span_flags);
		
    __opvx_a24xx_setcreg(wc_dev->mem32, 0x4a0, 7, spi_cmd );    
		int spi_cmd_from_fpga = __opvx_a24xx_getcreg(wc_dev->mem32, 0x4a0, 7);
    printk("%s:current spi_cmd is 0x%x\n", __FUNCTION__ , spi_cmd_from_fpga );
#endif
  
		res = a24xx_hardware_init_all(wc_dev, span_flags);
		if (res < 0) {
			/* Free Resources */
			if (wc_dev->freeregion) {
				release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
				iounmap((void *)wc_dev->mem32);
			}
			pci_free_consistent(pdev,  ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, (void *)wc_dev->writechunk, wc_dev->writedma);
			pci_set_drvdata(pdev, NULL);
			dahdi_unregister_device(wc->dev.ddev);   //Dennis
			kfree(wc->dev.ddev->location);
			dahdi_free_device(wc->dev.ddev);
			free_wc(wc);
			return -EIO;
		}
		for (z = 0; z < wc_dev->max_cards/*MAX_NUM_CARDS*/; z++) {
			if (span_flags[z]) {
				wc->chans[z]->sigcap = __DAHDI_SIG_FXO | DAHDI_SIG_BROKEN;
			}
		}
		
		/* init ec module */
		if (!wc_dev->vpm) {
			a24xx_vpm_init(wc);
		}

		if(wc_dev->master || ( wc_dev->fwversion < ((1 << 16)|2) ) ){
				if (request_irq(pdev->irq, a24xx_interrupt, IRQF_SHARED, "opvxa24xx"/*wc_dev->card_name*/, wc)) {
						printk(KERN_NOTICE "Unable to request IRQ %d\n", pdev->irq);
						if (wc_dev->freeregion) {
								release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
								iounmap((void *)wc_dev->mem32);
						}
						pci_free_consistent(pdev,  ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, (void *)wc_dev->writechunk, wc_dev->writedma);
						pci_set_drvdata(pdev, NULL);
						free_wc(wc);
						return -EIO;
				}
		}
		
		if(0) {    // for debug;
		  
		  printk("exit after alloc irq\n");
			/* Free Resources */
			if(wc_dev->master || ( wc_dev->fwversion < ((1 << 16)|2) ))
					free_irq(pdev->irq, wc);
			if (wc_dev->freeregion) {
				release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
				iounmap((void *)wc_dev->mem32);
			}
			pci_free_consistent(pdev,  ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, (void *)wc_dev->writechunk, wc_dev->writedma);
			pci_set_drvdata(pdev, NULL);
			dahdi_unregister_device(wc->dev.ddev);   //Dennis
			kfree(wc->dev.ddev->location);
			dahdi_free_device(wc->dev.ddev);
			free_wc(wc);
			return -EIO;
		}

#ifdef TEST_LOG_INCOME_VOICE
		for(x=0; x<wc_dev->max_cards; x++) {
			wc_dev->voc_buf[x] = kmalloc(voc_buffer_size, GFP_KERNEL);
			wc_dev->voc_ptr[x] = 0;
		}
#endif
		if(cidsupport) {
			int len = cidbuflen * DAHDI_MAX_CHUNKSIZE;
			if(debug) {
				printk("cid support enabled, length is %d msec\n", cidbuflen);
			}
			for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
				wc_dev->cid_history_buf[x] = kmalloc(len, GFP_KERNEL);
				wc_dev->cid_history_ptr[x] = 0;
				wc_dev->cid_history_clone_cnt[x] = 0;
				wc_dev->cid_state[x] = CID_STATE_IDLE;
			}
		}

		/* set channel sigcap */
		a24xx_post_initialize(wc);

		init_busydetect(wc, opermode);
		init_callerid(wc);

		/* Enable interrupts */
		__opvx_a24xx_enable_interrupts(wc_dev->mem32);
		
		/* Initialize Write/Buffers to all blank data */
		memset((void *)wc_dev->writechunk,0, ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2);

		/*set irq frequence*/
		if(wc_dev->fwversion >= ((1 << 16)|1) )
				__opvx_a24xx_set_irq_frq(wc_dev->mem32,ms_per_irq);
				
		if(wc_dev->fwversion > ((1 << 16)|2)){
				__opvx_a24xx_set_master(wc_dev->mem32,wc_dev->master^0x01);
		}
		/* Start DMA */
		__opvx_a24xx_start_dma(wc_dev->mem32, wc_dev->writedma);

		/* module count */
		for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
			if (wc_dev->cardflag & (1 << x)) {
				cardcount++;
			}
		}
          fwbuild = __opvx_a24xx_getcreg(wc_dev->mem32, 0x0, 0xe);
		printk(KERN_NOTICE "Found an OpenVox %s: Version %x.%x (%d modules),Build 0x%08x\n", wc_dev->card_name, wc_dev->fwversion>>16, wc_dev->fwversion&0xffff, cardcount,fwbuild );
		if(debug) {
			printk(KERN_DEBUG "OpenVox %s debug On\n", wc_dev->card_name);
		}
		
		ifaces[index] = wc;
		max_iface_index++;
		
		res = 0;
	}

	return res;
}


static void a24xx_release(struct a24xx *wc)
{
#ifdef TEST_LOG_INCOME_VOICE
	struct file * f = NULL;
	mm_segment_t orig_fs;
	int i;
	char fname[20];
#endif
	int y;
	char *s;
	struct a24xx_dev *wc_dev = &wc->dev;
	
	/*Unregister dahdi_span*/
	dahdi_unregister_device(wc_dev->ddev);   //Dennis
	/* Release mem region and iounmap mem */
	if (wc_dev->freeregion) {
		release_mem_region(wc_dev->mem_region, wc_dev->mem_len);
		iounmap((void *)wc_dev->mem32);
	}

#ifdef TEST_LOG_INCOME_VOICE
	for(i=0; i<wc_dev->max_cards; i++) {
		sprintf(fname, "//usr//%d.pcm", i);
		f = filp_open(fname, O_RDWR|O_CREAT, 00);

		if (!f || !f->f_op || !f->f_op->read) {
			printk("WARNING: File (read) object is a null pointer!!!\n");
			continue;
		}

		f->f_pos = 0;

		orig_fs = get_fs();
		set_fs(KERNEL_DS);

		if(wc_dev->voc_buf[i]) {
			f->f_op->write(f, wc_dev->voc_buf[i], voc_buffer_size, &f->f_pos);
			kfree(wc_dev->voc_buf[i]);
		}

		set_fs(orig_fs);
		fput(f);
	}
#endif

	/* Release cid history buffer */
	if(cidsupport) {
		int x;
		for (x = 0; x < wc_dev->max_cards/*MAX_NUM_CARDS*/; x++) {
			kfree(wc_dev->cid_history_buf[x]);
		}
	}

	/* Release a24xx */
	s = wc_dev->card_name;
	for(y = 0; y < max_iface_index; y++)
			if(ifaces[y] == wc)
					break;
					
					
	kfree(wc_dev->ddev->location);       //Dennis
	dahdi_free_device(wc_dev->ddev);
	free_wc(wc);
	ifaces[y]=NULL;
	
	printk(KERN_INFO "Free an OpenVox %s card\n", s);
}

static void __devexit a24xx_remove_one(struct pci_dev *pdev)
{
	struct a24xx *wc = pci_get_drvdata(pdev);
	struct a24xx_dev *wc_dev = &wc->dev;
	
	if (wc) {
		/* In case hardware is still there */
		__opvx_a24xx_disable_interrupts(wc_dev->mem32);

		/* Wait some time to handle the last irq */
		__a24xx_wait_just_a_bit(HZ/10);   /* delay 1/10 sec */

		/* Stop any DMA */
		__opvx_a24xx_stop_dma(wc_dev->mem32);

		/* Reset tdm controller */
		__opvx_a24xx_reset_tdm(wc_dev->mem32);

		/* Release echo canceller */
		if (wc_dev->vpm_ec) {
			opvx_vpm_release(wc_dev->vpm_ec);
		}
		wc_dev->vpm_ec = NULL;

		if(wc_dev->master || ( wc_dev->fwversion < ((1 << 16)|2) )){
				free_irq(pdev->irq, wc);
		}
		/* Immediately free resources */
		pci_free_consistent(pdev,  ms_per_irq * DAHDI_MAX_CHUNKSIZE * MAX_NUM_CARDS * 2 * 2, (void *)wc_dev->writechunk, wc_dev->writedma);	

		destroy_callerid(wc);
		destroy_busydetect(wc);

		/* Release span, possibly delayed */
		if (!wc_dev->usecount) {
			a24xx_release(wc);
		} else {
			wc_dev->dead = 1;
		}
	}
}

static struct pci_device_id a24xx_pci_tbl[] = {
	{ 0x1B74, 0x2410, 0x1B74, 0x0001, 0, 0, (unsigned long) &a2410a },  // Information for A2410 revsion A.
	{ 0x1B74, 0x1610, 0x1B74, 0x0001, 0, 0, (unsigned long) &a1610a },  // Information for A1610 revsion A.
	{ 0x1B74, 0x0810, 0x1B74, 0x0001, 0, 0, (unsigned long) &a810a },  // Information for A810 revsion A.
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, a24xx_pci_tbl);

static struct pci_driver a24xx_driver = {
	.name = "opvxa24xx",
	.probe = a24xx_init_one,
	.remove = __devexit_p(a24xx_remove_one),
	.suspend = NULL,
	.resume = NULL,
	.id_table = a24xx_pci_tbl,
};

static int __init a24xx_init(void)
{
	int res;
	int x;
	for (x=0;x<(sizeof(fxo_modes) / sizeof(fxo_modes[0])); x++) {
		if (!strcmp(fxo_modes[x].name, opermode)) {
			break;
		}
	}
	if (x < sizeof(fxo_modes) / sizeof(fxo_modes[0])) {
		_opermode = x;
	} else {
		printk(KERN_NOTICE "Invalid/unknown operating mode '%s' specified.  Please choose one of:\n", opermode);
		for (x=0;x<sizeof(fxo_modes) / sizeof(fxo_modes[0]); x++) {
			printk(KERN_INFO "  %s\n", fxo_modes[x].name);
		}
		printk(KERN_INFO "Note this option is CASE SENSITIVE!\n");
		return -ENODEV;
	}
	if (!strcmp(fxo_modes[_opermode].name, "AUSTRALIA")) {
		boostringer=1;
		fxshonormode=1;
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

	res = dahdi_pci_module(&a24xx_driver);
	if (res) {
		return -ENODEV;
	}
	return 0;
}

static void __exit a24xx_cleanup(void)
{
	pci_unregister_driver(&a24xx_driver);
}

module_param(spi_cmd, int, 0600);
module_param(debug, int, 0600);
module_param(ec_debug, int, 0600);
module_param(vpmsupport, int, 0600);
module_param(loopcurrent, int, 0600);
module_param(reversepolarity, int, 0600);
module_param(robust, int, 0600);
module_param(_opermode, int, 0600);
module_param(opermode, charp, 0600);
module_param(timingonly, int, 0600);
module_param(lowpower, int, 0600);
module_param(boostringer, int, 0600);
module_param(fastringer, int, 0600);
module_param(fxshonormode, int, 0600);
module_param(battdebounce, uint, 0600);
module_param(battalarm, uint, 0600);
module_param(battthresh, uint, 0600);
module_param(ringdebounce, int, 0600);
module_param(fwringdetect, int, 0600);
module_param(alawoverride, int, 0600);
module_param(fastpickup, int, 0600);
module_param(fxotxgain, int, 0600);
module_param(fxorxgain, int, 0600);
module_param(fxstxgain, int, 0600);
module_param(fxsrxgain, int, 0600);
module_param(cidsupport, int, 0600);
module_param(cidbuflen, int, 0600);
module_param(cidtimeout, int, 0600);
module_param(fxofullscale, int, 0600);
module_param(fixedtimepolarity, int, 0600);
module_param(ms_per_irq, int, 0600);
module_param(irq_stub, int, 0600);

MODULE_DESCRIPTION("OpenVox A2410 Dahdi Driver");
MODULE_AUTHOR("MiaoLin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(a24xx_init);
module_exit(a24xx_cleanup);

