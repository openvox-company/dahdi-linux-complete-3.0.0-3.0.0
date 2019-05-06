/*
 * OpenVox FXS module on x200 hybrid card
 *
 * Written by Miao Lin<lin.miao@openvox.cn>
 *
 * Copyright (C) 2011-2013 OpenVox Communication Co Ltd..
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
#include "x2fxs200m.h"
#include "ec3000.h"

extern int vpmsupport;
extern int tdm_mode;
extern int spi_fifo_mode;
extern int interface_num;

#define X2FXS200M_SPI_DELAY   50        /* need delay 15ns between spi operate */
#define	CHANS_PER_MODULE	2
#define	MAX_NUM_CHANS_PER_CARD	4
#define MAX_ALARMS 10
#define NUM_CAL_REGS 12

#define OHT_TIMER		6000	/* How long after RING to retain OHT */
int debug = 0;
int fastringer = 0;
int boostringer = 0;
int lowpower = 0;
int alawoverride = 1;
int fxstxgain = 0;
int fxsrxgain = 0;
int fxshonormode = 0;
int loopcurrent = 20;
int _opermode = 0;
int reversepolarity = 0;
static int dialdebounce = 8 * 4;

int acim2tiss[16] = { 0x0, 0x1, 0x4, 0x5, 0x7, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x2, 0x0, 0x3 };

/* indirect_resg */
alpha  indirect_regs[] =
{
{0,255,"DTMF_ROW_0_PEAK",0x55C2},
{1,255,"DTMF_ROW_1_PEAK",0x51E6},
{2,255,"DTMF_ROW2_PEAK",0x4B85},
{3,255,"DTMF_ROW3_PEAK",0x4937},
{4,255,"DTMF_COL1_PEAK",0x3333},
{5,255,"DTMF_FWD_TWIST",0x0202},
{6,255,"DTMF_RVS_TWIST",0x0202},
{7,255,"DTMF_ROW_RATIO_TRES",0x0198},
{8,255,"DTMF_COL_RATIO_TRES",0x0198},
{9,255,"DTMF_ROW_2ND_ARM",0x0611},
{10,255,"DTMF_COL_2ND_ARM",0x0202},
{11,255,"DTMF_PWR_MIN_TRES",0x00E5},
{12,255,"DTMF_OT_LIM_TRES",0x0A1C},
{13,0,"OSC1_COEF",0x7B30},
{14,1,"OSC1X",0x0063},
{15,2,"OSC1Y",0x0000},
{16,3,"OSC2_COEF",0x7870},
{17,4,"OSC2X",0x007D},
{18,5,"OSC2Y",0x0000},
{19,6,"RING_V_OFF",0x0000},
{20,7,"RING_OSC",0x7EF0},
{21,8,"RING_X",0x0160},
{22,9,"RING_Y",0x0000},
{23,255,"PULSE_ENVEL",0x2000},
{24,255,"PULSE_X",0x2000},
{25,255,"PULSE_Y",0x0000},
//{26,13,"RECV_DIGITAL_GAIN",0x4000},	// playback volume set lower
{26,13,"RECV_DIGITAL_GAIN",0x2000},	// playback volume set lower
{27,14,"XMIT_DIGITAL_GAIN",0x4000},
//{27,14,"XMIT_DIGITAL_GAIN",0x2000},
{28,15,"LOOP_CLOSE_TRES",0x1000},
{29,16,"RING_TRIP_TRES",0x3600},
{30,17,"COMMON_MIN_TRES",0x1000},
{31,18,"COMMON_MAX_TRES",0x0200},
{32,19,"PWR_ALARM_Q1Q2",0x0ff4},
{33,20,"PWR_ALARM_Q3Q4",0x6e7e},
{34,21,"PWR_ALARM_Q5Q6",0x0ff4},
{35,22,"LOOP_CLOSURE_FILTER",0x8000},
{36,23,"RING_TRIP_FILTER",0x0320},
{37,24,"TERM_LP_POLE_Q1Q2",0x0012},
{38,25,"TERM_LP_POLE_Q3Q4",0x0012},
{39,26,"TERM_LP_POLE_Q5Q6",0x0012},
{40,27,"CM_BIAS_RINGING",0x0C00},
{41,64,"DCDC_MIN_V",0x0C00},
{42,255,"DCDC_XTRA",0x1000},
{43,66,"LOOP_CLOSE_TRES_LOW",0x1000},
};

char *opermode = "FCC";

struct calregs {
	unsigned char vals[NUM_CAL_REGS];
};

enum proslic_power_warn {
	PROSLIC_POWER_UNKNOWN = 0,
	PROSLIC_POWER_ON,
	PROSLIC_POWER_WARNED,
};

struct x200fxs {
	struct x200_dev *dev;
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
	struct dahdi_device *ddev;
#endif
	
	struct _fxs {
			int oldrxhook;
			int debouncehook;
			int lastrxhook;
			int debounce;
			int ohttimer;
			int idletxhookstate;		/* IDLE changing hook state */
			int lasttxhook;
			int palarms;
			struct calregs calregs;
			struct dahdi_vmwi_info vmwisetting;
			int vmwi_active_messages;
			u32 vmwi_lrev:1; /*MWI Line Reversal*/
			u32 vmwi_hvdc:1; /*MWI High Voltage DC Idle line*/
			u32 vmwi_hvac:1; /*MWI Neon High Voltage AC Idle line*/
			u32 neonringing:1;/*Ring Generator is set for NEON*/
			int reversepolarity;	/* polarity reversal */
			spinlock_t lasttxhooklock;
			enum proslic_power_warn proslic_power;
			unsigned char reg0shadow;
			unsigned char reg1shadow;
	} fxsconfig[CHANS_PER_MODULE];
	int usecount;
	// dahdi API
	struct dahdi_span span;
	struct dahdi_chan _chans[CHANS_PER_MODULE];
	struct dahdi_chan *chans[CHANS_PER_MODULE];
	struct dahdi_echocan_state *ec[2];                 /* Echcan state for each channel */
	int dead;
	int	exist;
};

struct device_driver x200_dev_driver[WC_MAX_INTERFACES];

#define POLARITY_XOR(chanx) ( \
		(reversepolarity != 0) ^ (a200m->fxsconfig[chanx].reversepolarity != 0) ^ \
		(a200m->fxsconfig[chanx].vmwi_lrev != 0) ^\
		((a200m->fxsconfig[chanx].vmwisetting.vmwi_type & DAHDI_VMWI_HVAC) != 0)\
		)

const struct dahdi_echocan_features vpm_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

void a200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

const struct dahdi_echocan_ops vpm_ec_ops = {
#if DAHDI_VERSION_CODE < VERSION_CODE(2,5,0)
	.name = "FXS200M VPM",
#endif
	.echocan_free = a200m_echocan_free,
};

int a200m_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	int channel;
	struct x200fxs *a200m = chan->pvt;
	struct x200* x2 = a200m->dev->bus;

	if (!vpmsupport || !x2->vpm_present ) {
		return -ENODEV;
	}

	if (ecp->param_count > 0) {
		printk(KERN_ERR "error at line:%d,  echo canceller does not support parameters; failing request\n",__LINE__);
		return -EINVAL;
	}
	channel = a200m->dev->slot_id+(1+chan->chanpos-1)*4;

	if (x2->vpm450m) {
	    *ec = a200m->ec[chan->chanpos-1];
	    (*ec)->ops = &vpm_ec_ops;
	    (*ec)->features = vpm_ec_features;
	} else {
		printk(KERN_ERR "error at line:%d\n",__LINE__);
		return -EINVAL;
	}	
    vpm450m_setec(x2, channel, a200m->dev->slot_id, ecp->tap_length);
	return 0;
}

void a200m_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	int channel;
	struct x200fxs *a200m = chan->pvt;
	struct x200* x2 = a200m->dev->bus;

	memset(ec, 0, sizeof(*ec));
	channel = a200m->dev->slot_id+(1+chan->chanpos-1)*4;

	if (x2->vpm450m) {	    
		//printk("fxs200m a200m_echocan_free line:%d, channel:%d.\n", __LINE__, channel);
        vpm450m_setec(x2, channel, a200m->dev->slot_id, 0);
    }
}

void wait_just_a_bit(int foo)
{
		long newjiffies;
		newjiffies = jiffies + foo;
		while(jiffies < newjiffies);
}

unsigned char __translate_3215(unsigned char address)
{
		int x;
		for (x=0;x<sizeof(indirect_regs)/sizeof(indirect_regs[0]);x++) {
			if (indirect_regs[x].address == address) {
				address = indirect_regs[x].altaddr;
				break;
			}
		}
		return address;
}

inline void fxs200m_set_regbit(struct x200_dev *dev, unsigned char addr, unsigned char bitmap, unsigned int chanx)
{
		unsigned char data1;
		data1 = fxs200m_read_reg(dev, addr, chanx);
		data1 |= bitmap;
		fxs200m_write_reg(dev, addr, data1, chanx);
}

static int __wait_access(struct x200_dev *dev, int i)
{
    unsigned char data = 0;
    long origjiffies;
    int count = 0;

    #define MAX 6000 /* attempts */

    origjiffies = jiffies;
    /* Wait for indirect access */
    while (count++ < MAX)
	 {
		data = fxs200m_read_reg(dev, REG_IA_STS, i);
		if (!data)
			return 0;
	 }
    if(count > (MAX-1)) {
    	printk(KERN_NOTICE " ##### Loop error (%02x) #####\n", data);
    }

	return 0;
}

static int fxs200m_proslic_setreg_indirect(struct x200_dev *dev, unsigned char addr, unsigned int data, unsigned int chanx)
{
	int res = -1;

		addr = __translate_3215(addr);
		if (addr == 255) {
			return 0;
		}

	if(!__wait_access(dev, chanx)) {
		if (0)		printk(" Write indirect data=0x%x data1=0x%x data2=0x%x reg-%x chan[%x]\n", data, (unsigned char)(data & 0xFF), (unsigned char)((data & 0xFF00)>>8), addr, chanx);
		fxs200m_write_reg(dev, REG_IDA_LO, (unsigned char)(data & 0xFF), chanx);
		fxs200m_write_reg(dev, REG_IDA_HI, (unsigned char)((data & 0xFF00)>>8), chanx);
		fxs200m_write_reg(dev, REG_IA, addr, chanx);
		res = 0;
	}
	return res;
}

int fxs200m_proslic_getreg_indirect(struct x200_dev *dev, unsigned char addr, unsigned int chanx)
{
	int res = -1;
	char *p=NULL;

		addr = __translate_3215(addr);
		if (addr == 255) {
			return 0;
		}

	if(!__wait_access(dev, chanx)) {
		fxs200m_write_reg(dev, REG_IA, addr, chanx);
		if(!__wait_access(dev, chanx)) {
			unsigned char data1, data2;
			data1 = fxs200m_read_reg(dev, REG_IDA_LO, chanx);
			data2 = fxs200m_read_reg(dev, REG_IDA_HI, chanx);
			res = data1 | (data2 << 8);
			if (0)		printk(" Read indirect data1=0x%x data2=0x%x reg-%x=0x%x chan[%x]\n", data1, data2, addr, res, chanx);
		} else {
			p = "Failed to wait inside\n";
		}
	} else {
		p = "failed to wait\n";
	}
	if (p) {
		printk("%s\n", p);
	}
	return res;
}

int fxs200m_set_ring_generator_mode(struct x200_dev *dev, unsigned int chanx, int mode)
{
	int reg20, reg21, reg74; /* RCO, RNGX, VBATH */
	struct x200fxs *a200m = dev->drv_data;

	a200m->fxsconfig[chanx].neonringing = mode;	/* track ring generator mode */

	if (mode) { /* Neon */
		if (debug)
			printk(KERN_DEBUG "NEON ring on chan %d lasttxhook was 0x%x\n", chanx, a200m->fxsconfig[chanx].lasttxhook);
		/* Must be in FORWARD ACTIVE before setting ringer */
		a200m->fxsconfig[chanx].lasttxhook = SLIC_LF_ACTIVE_FWD;
		fxs200m_write_reg(dev, REG_LFCTRL, a200m->fxsconfig[chanx].lasttxhook, chanx);

		fxs200m_proslic_setreg_indirect(dev, IREG_RNGY, NEON_MWI_RNGY_PULSEWIDTH, chanx);
		fxs200m_proslic_setreg_indirect(dev, IREG_RNGX, 0x7bef, chanx);
		fxs200m_proslic_setreg_indirect(dev, IREG_RCO, 0x009f, chanx);


		fxs200m_write_reg(dev, REG_ROCTRL, 0x19, chanx);  /* Ringing Osc. Control */
		fxs200m_write_reg(dev, REG_BAT_V_HI, 0x3f, chanx); /* VBATH 94.5V */
		fxs200m_proslic_setreg_indirect(dev, IREG_RPTP, 0x4600, chanx); /* RPTP */

		/* A write of 0x04 to register 64 will turn on the VM led */
	} else {
		fxs200m_write_reg(dev, REG_ROCTRL, 0, chanx);  /* Ringing Osc. Control */
		/* RNGY Initial Phase */
		fxs200m_proslic_setreg_indirect(dev, IREG_RNGY, 0, chanx);
		fxs200m_proslic_setreg_indirect(dev, IREG_RPTP, 0x3600, chanx); /* RPTP */
		/* A write of 0x04 to register 64 will turn on the ringer */

		if (fastringer) {
				/* Speed up Ringer */
				reg20 =  0x7e6d;
				reg74 = 0x32;	/* Default */
				/* Beef up Ringing voltage to 89V */
				if (boostringer) {
					reg74 = 0x3f;
					reg21 = 0x0247;	/* RNGX */
					if (debug)
						printk(KERN_DEBUG "Boosting fast ringer"
							" on chan %d (89V peak)\n",
							chanx);
				} else if (lowpower) {
					reg21 = 0x014b;	/* RNGX */
					if (debug)
						printk(KERN_DEBUG "Reducing fast ring "
						    "power on chan %d (50V peak)\n",
						    chanx);
				} else if (fxshonormode &&
							fxo_modes[_opermode].ring_x) {
					reg21 = fxo_modes[_opermode].ring_x;
					if (debug)
						printk(KERN_DEBUG "fxshonormode: fast "
							"ring_x power on chan %d\n",
							chanx);
				} else {
					reg21 = 0x01b9;
					if (debug)
						printk(KERN_DEBUG "Speeding up ringer "
							"on chan %d (25Hz)\n",
							chanx);
				}
					/* VBATH */
					fxs200m_write_reg(dev, REG_BAT_V_HI, reg74, chanx);
					/* RCO */
					fxs200m_proslic_setreg_indirect(dev, IREG_RCO, reg20, chanx);
					/* RNGX */
					fxs200m_proslic_setreg_indirect(dev, IREG_RNGX, reg21, chanx);
			} else {
				/* Ringer Speed */
				if (fxo_modes[_opermode].ring_osc) {
					reg20 = fxo_modes[_opermode].ring_osc;
					if (debug)
						printk(KERN_DEBUG "fxshonormode: ring_osc speed on chan %d\n", chanx);
				} else {
					reg20 = 0x7ef0;	/* Default */
				}
					reg74 = 0x32;	/* Default */
				/* Beef up Ringing voltage to 89V */
				if (boostringer) {
					reg74 = 0x3f;
					reg21 = 0x1d1;
					if (debug)
						printk(KERN_DEBUG "Boosting ringer on "
							"chan %d (89V peak)\n",
							chanx);
				} else if (lowpower) {
					reg21 = 0x108;
					if (debug)
						printk(KERN_DEBUG "Reducing ring power "
							"on chan %d (50V peak)\n",
							chanx);
				} else if (fxshonormode && fxo_modes[_opermode].ring_x) {
						reg21 = fxo_modes[_opermode].ring_x;
						if (debug)
							printk(KERN_DEBUG "fxshonormode: ring_x power on chan %d\n", chanx);
					} else {
						reg21 = 0x160;
						if (debug)
							printk(KERN_DEBUG "Normal ring power on chan %d\n", chanx);
					}
					/* VBATH */
					fxs200m_write_reg(dev, REG_BAT_V_HI, reg74, chanx);
					/* RCO */
					fxs200m_proslic_setreg_indirect(dev, IREG_RCO, reg20, chanx);
					/* RNGX */
					fxs200m_proslic_setreg_indirect(dev, IREG_RNGX, reg21, chanx);
		}
	}
	return 0;
}

int fxs200m_init_ring_generator_mode(struct x200_dev *dev, unsigned int chanx)
{
	fxs200m_write_reg(dev, REG_ROCTRL, 0, chanx);

	/* neon trapezoid timers */
	fxs200m_write_reg(dev, REG_RAT_LO, 0xe0, chanx);	/* Active Timer low byte */
	fxs200m_write_reg(dev, REG_RAT_HI, 0x01, chanx);	/* Active Timer high byte */
	fxs200m_write_reg(dev, REG_RIT_LO, 0xF0, chanx);	/* Inactive Timer low byte */
	fxs200m_write_reg(dev, REG_RIT_HI, 0x05, chanx);	/* Inactive Timer high byte */

	fxs200m_set_ring_generator_mode(dev, chanx, 0);

	return 0;
}

int fxs200m_proslic_powerleak_test(struct x200_dev *dev, unsigned int chanx)
{
	unsigned long origjiffies;
	unsigned char vbat;

	/* Turn off linefeed */
	fxs200m_write_reg(dev, REG_LFCTRL, 0, chanx);

	/* Power down */
	fxs200m_write_reg(dev, REG_PWRDOWNCTRL1, 0x10, chanx);

	/* Wait for one second */
	origjiffies = jiffies;

	vbat = fxs200m_read_reg(dev, REG_VBATS1, chanx);

	while(vbat>0x6) {
		if ((jiffies - origjiffies) >= (HZ/2))
			break;;
	}

	if (vbat < 0x06) {
		printk(KERN_NOTICE "Excessive leakage detected on chan %d: %d volts (%02x) after %d ms\n", chanx,
		       376 * vbat / 1000, vbat, (int)((jiffies - origjiffies) * 1000 / HZ));
		return -1;
	} else if (debug) {
		printk(KERN_NOTICE "Post-leakage voltage: %d volts\n", 376 * vbat / 1000);
	}
	return 0;
}

int fxs200m_powerup_proslic(struct x200_dev *dev, unsigned int chanx, int fast)
{
	unsigned char vbat;
	unsigned long origjiffies;
	int lim;
	struct	x200fxs *a200m = dev->drv_data;

	/* Set period of DC-DC converter to 1/64 khz */
	fxs200m_write_reg(dev, REG_DCPWM, 0xFF, chanx);

	/* Wait for VBat to powerup */
	origjiffies = jiffies;

	/* Disable powerdown */
	fxs200m_write_reg(dev, REG_PWRDOWNCTRL1, 0, chanx);

	/* If fast, don't bother checking anymore */
	if (fast)
		return 0;

	vbat = fxs200m_read_reg(dev, REG_VBATS1, chanx);
//	while(vbat<0xc0) {
	while((fxs200m_read_reg(dev, REG_VBATS1, chanx))<0xc0) {
		/* Wait no more than 500ms */
		if ((jiffies - origjiffies) > HZ/2) {
			break;
		}
	}
	vbat = fxs200m_read_reg(dev, REG_VBATS1, chanx);
/*
	printk("fxs200m-debug: vbat=0x%x !!! --------wujc\n", vbat);
			int i;
			printk("!below: debugging info for si3215! \n");
		// dump the chip[0] and chip[1] registers
			for (i=0; i<110; i++)
			{
				if (! (i & 15))
				   printk("%02x:", i);
				printk(" %02x", fxs200m_read_reg(dev, i, chanx));
				if ((i & 15) == 15)
				   printk("\n");
			}
			 printk("\n");
*/
	if (vbat<0xc0) {
		if (a200m->fxsconfig[chanx].proslic_power == PROSLIC_POWER_UNKNOWN)
			 printk(KERN_NOTICE "ProSLIC on chanx %d failed to powerup within %d ms (%d mV only)\n\n -- DID YOU REMEMBER TO PLUG IN THE HD POWER CABLE TO THE TDM400P??\n", chanx, (int)(((jiffies - origjiffies) * 1000 / HZ)), vbat * 375);
		a200m->fxsconfig[chanx].proslic_power = PROSLIC_POWER_WARNED;
		return -1;
	} else if (debug) {
		printk(KERN_DEBUG "ProSLIC on module %d powered up to -%d volts (%02x) in %d ms\n",
		       chanx, vbat * 376 / 1000, vbat, (int)(((jiffies - origjiffies) * 1000 / HZ)));
	}
	a200m->fxsconfig[chanx].proslic_power = PROSLIC_POWER_ON;

        /* Proslic max allowed loop current, reg 71 LOOP_I_LIMIT */
        /* If out of range, just set it to the default value     */
        lim = (loopcurrent - 20) / 3;
        if ( loopcurrent > 41 ) {
                lim = 0;
                if (debug)
                        printk(KERN_DEBUG "Loop current out of range! Setting to default 20mA!\n");
        }
        else if (debug)
                        printk(KERN_DEBUG "Loop current set to %dmA!\n",(lim*3)+20);

	fxs200m_write_reg(dev, REG_LOOP_I_LIMIT, lim, chanx);

	/* Engage DC-DC converter */
	fxs200m_write_reg(dev, REG_DCDELAY, 0x19, chanx);

	return 0;

}

static int fxs200m_proslic_manual_calibrate(struct x200_dev *dev, unsigned int chanx){
	unsigned long origjiffies;
	unsigned char i;

	//Disable all interupts
	fxs200m_write_reg(dev, REG_INT1_EN, 0, chanx);
	fxs200m_write_reg(dev, REG_INT2_EN, 0, chanx);
	fxs200m_write_reg(dev, REG_INT3_EN, 0, chanx);
	fxs200m_write_reg(dev, REG_LFCTRL, 0, chanx);

	//(0x18)Calibrations without the ADC and DAC offset and without common mode calibration.
	fxs200m_write_reg(dev, REG_CALIBR2, 0x18, chanx);
	//(0x47)Calibrate common mode and differential DAC mode DAC + ILIM
	fxs200m_write_reg(dev, REG_CALIBR1, 0x47, chanx);

	origjiffies=jiffies;
	while( fxs200m_read_reg(dev, REG_CALIBR1, chanx)!=0 ){
		if((jiffies-origjiffies)>80)
			return -1;
	}
//Initialized DR 98 and 99 to get consistant results.
// 98 and 99 are the results registers and the search should have same intial conditions.

/*******************************The following is the manual gain mismatch calibration****************************/
/*******************************This is also available as a function *******************************************/
	// Delay 10ms
	origjiffies=jiffies;
	while((jiffies-origjiffies)<1);
/*
	fxs200m_proslic_setreg_indirect(dev, REG_IQ5, 0, chanx);
	fxs200m_proslic_setreg_indirect(dev, REG_IQ6, 0, chanx);
	fxs200m_proslic_setreg_indirect(dev, REG_RESERVED90, 0, chanx);
	fxs200m_proslic_setreg_indirect(dev, REG_RESERVED91, 0, chanx);
	fxs200m_proslic_setreg_indirect(dev, REG_DCPWM, 0, chanx);
	fxs200m_proslic_setreg_indirect(dev, REG_DCDELAY, 0, chanx);
*/
	// This is necessary if the calibration occurs other than at reset time
	fxs200m_write_reg(dev, REG_RING_GAIN_CAL, 0x10, chanx);
	fxs200m_write_reg(dev, REG_TIP_GAIN_CAL, 0x10, chanx);

	for ( i=0x1f; i>0; i--)
	{
		fxs200m_write_reg(dev, REG_RING_GAIN_CAL, i, chanx);
		origjiffies=jiffies;
		while((jiffies-origjiffies)<4);
		if((fxs200m_read_reg(dev, REG_IQ5, chanx)) == 0)
			break;
	} // for

	for ( i=0x1f; i>0; i--)
	{
		fxs200m_write_reg(dev, REG_TIP_GAIN_CAL, i, chanx);
		origjiffies=jiffies;
		while((jiffies-origjiffies)<4);
		if((fxs200m_read_reg(dev, REG_IQ6, chanx)) == 0)
			break;
	}//for

/*******************************The preceding is the manual gain mismatch calibration****************************/
/**********************************The following is the longitudinal Balance Cal***********************************/
	fxs200m_write_reg(dev, REG_LFCTRL, 0x01, chanx);

	while((jiffies-origjiffies)<10); // Sleep 100?

	fxs200m_write_reg(dev, REG_LFCTRL, 0, chanx);
	// enable interrupt for the balance Cal
	fxs200m_write_reg(dev, REG_INT3_EN, 0x4, chanx);
	// this is a singular calibration bit for longitudinal calibration
	fxs200m_write_reg(dev, REG_CALIBR2, 1, chanx);
	fxs200m_write_reg(dev, REG_CALIBR1, 0x40, chanx);

	/* Read Reg 96 just cause */
	fxs200m_read_reg(dev, REG_CALIBR1, chanx);

	fxs200m_write_reg(dev, REG_INT1_EN, 0xff, chanx);
	fxs200m_write_reg(dev, REG_INT2_EN, 0xff, chanx);
	fxs200m_write_reg(dev, REG_INT3_EN, 0xff, chanx);

	/**The preceding is the longitudinal Balance Cal***/
	return(0);

}

static int fxs200m_proslic_calibrate(struct x200_dev *dev, unsigned int chanx)
{
	unsigned long origjiffies;
	int x;
	/* Perform all calibrations */
	fxs200m_write_reg(dev, REG_CALIBR2, 0x1f, chanx);

	/* Begin, no speedup */
	fxs200m_write_reg(dev, REG_CALIBR1, 0x5f, chanx);

	/* Wait for it to finish */
	origjiffies = jiffies;
	while(fxs200m_read_reg(dev, REG_CALIBR1, chanx)) {
		if ((jiffies - origjiffies) > 2 * HZ) {
			printk(KERN_NOTICE "Timeout waiting for calibration of chan %d\n", chanx);
			return -1;
		}
	}

	if (debug) {
		/* Print calibration parameters */
		printk(KERN_DEBUG "Calibration Vector Regs 98 - 107: \n");
		for (x=98;x<108;x++) {
			printk(KERN_DEBUG "%d: %02x\n", x, fxs200m_read_reg(dev, x, chanx));
		}
	}
	return 0;
}

int fxs200m_proslic_insane(struct x200_dev *dev, unsigned int chanx)
{
	int blah,insane_report;
	insane_report=0;

	blah = fxs200m_read_reg(dev, REG_SPIMODE, chanx);
	if (debug)
		printk(KERN_DEBUG "ProSLIC on chan %d, product %d, version %d\n", chanx, (blah & 0x30) >> 4, (blah & 0xf));

#if 0
	if ((blah & 0x30) >> 4) {
		printk(KERN_DEBUG "ProSLIC on module %d is not a 3210.\n", card);
		return -1;
	}
#endif
	if (((blah & 0xf) == 0) || ((blah & 0xf) == 0xf)) {
		return -1; /* SLIC not loaded */
	}
	if ((blah & 0xf) < 2) {
		printk(KERN_NOTICE "ProSLIC 3215 version %d is too old\n", blah & 0xf);
		return -1;
	}

	blah = fxs200m_read_reg(dev, REG_AUDIO_LOOPBACK, chanx);
	if (blah != 0x2) {
		printk(KERN_NOTICE "ProSLIC on chan %d insane (1) %d should be 2\n",chanx, blah);
		return -1;
	} else if ( insane_report)
		printk(KERN_NOTICE "ProSLIC on chan %d Reg 8 Reads %d Expected is 0x2\n", chanx, blah);

	blah = fxs200m_read_reg(dev, REG_LFCTRL, chanx);
	if (blah != 0x0) {
		printk(KERN_NOTICE "ProSLIC on chan %d insane (2)\n", chanx);
		return -1;
	} else if ( insane_report)
		printk(KERN_NOTICE "ProSLIC on chan %d Reg 64 Reads %d Expected is 0x0\n", chanx, blah);

	blah = fxs200m_read_reg(dev, REG_HYBRID_CTRL, chanx);

	if (blah != 0x33) {
		printk(KERN_NOTICE "ProSLIC on chan %d insane (3)\n", chanx);
		return -1;
	} else if ( insane_report)
		printk(KERN_NOTICE "ProSLIC on chan %d Reg 11 Reads %d Expected is 0x33\n", chanx, blah);

	/* Just be sure it's setup right. */
	fxs200m_write_reg(dev, REG_IA, 0, chanx);

	if (debug)
		printk(KERN_DEBUG "ProSLIC on chan %d seems sane.\n", chanx);
	return 0;
}

int fxs200m_proslic_verify_indirect_regs(struct x200_dev *dev, unsigned int chanx)
{
	int passed = 1;
	unsigned short i, initial;
	int j;

	for (i=0; i<sizeof(indirect_regs) / sizeof(indirect_regs[0]); i++) {
		if((j = fxs200m_proslic_getreg_indirect(dev, (unsigned char)indirect_regs[i].address, chanx)) < 0) {
			printk("Failed to read indirect register %d\n", i);
			return -1;
		}
		initial= indirect_regs[i].initial;

		if (j != initial && (indirect_regs[i].altaddr != 255)) {
			 printk("!!!!!!! %s  iREG %X = %X  should be %X\n",indirect_regs[i].name,indirect_regs[i].address,j,initial );
			 passed = 0;
		}
	}

	if (passed) {
		if (debug) {
			printk("Init Indirect Registers completed successfully.\n");
		}
	} else {
		printk(" !!!!! Init Indirect Registers UNSUCCESSFULLY.\n");
		return -1;
	}
	return 0;
}

int fxs200m_init_proslic(struct x200_dev *dev, unsigned int chanx, int fast, int manual, int sane)
{
	unsigned short tmp[5];
	unsigned char r19,r9;
	int x;
	int fxsmode=0;
	struct x200fxs *a200m = dev->drv_data;

	/* Sanity check the ProSLIC */
	if (!sane && fxs200m_proslic_insane(dev, chanx))
		return -2;

	/* default messages to none and method to FSK */
	memset(&a200m->fxsconfig[chanx].vmwisetting, 0, sizeof(a200m->fxsconfig[chanx].vmwisetting));
	a200m->fxsconfig[chanx].vmwi_lrev = 0;
	a200m->fxsconfig[chanx].vmwi_hvdc = 0;
	a200m->fxsconfig[chanx].vmwi_hvac = 0;

	/* By default, don't send on hook */
	if (!reversepolarity != !a200m->fxsconfig[chanx].reversepolarity)
		a200m->fxsconfig[chanx].idletxhookstate = SLIC_LF_ACTIVE_REV;
	else
		a200m->fxsconfig[chanx].idletxhookstate = SLIC_LF_ACTIVE_FWD;

	if (sane) {
		/* Make sure we turn off the DC->DC converter to prevent anything from blowing up */
		fxs200m_write_reg(dev, REG_PWRDOWNCTRL1, 0x10, chanx);
	}

	// init the indirect registers
	for (x=0; x<sizeof(indirect_regs)/sizeof(indirect_regs[0]); x++) {
		if (0) printk("!!!debug: init indirect reg x=%x addr=0x%x val=0x%x\n", x, indirect_regs[x].address, indirect_regs[x].initial);
		if(fxs200m_proslic_setreg_indirect(dev, indirect_regs[x].address, indirect_regs[x].initial, chanx))	{
	        printk("fxs200m: init PROSLIC indirect registers ERROR !!!\n");
		    return -1;
		}
	}

	/* Clear scratch pad area */
	//wctdm_proslic_setreg_indirect(wc, card, 97,0);
	fxs200m_proslic_setreg_indirect(dev, 97, 0, chanx);

	/* Clear digital loopback */
	fxs200m_write_reg(dev, REG_AUDIO_LOOPBACK, 0, chanx);

	/* Revision C optimization */
	fxs200m_write_reg(dev, REG_ENHANCE_EN, 0xeb, chanx);

	/* Disable automatic VBat switching for safety to prevent Q7 from accidently turning on and burning out. */
	fxs200m_write_reg(dev, REG_AUTO_CTRL, 0x7, chanx);  /* Note, if pulse dialing has problems at high REN loads change this to 0x17 */

	/* Turn off Q7 */
	fxs200m_write_reg(dev, REG_BATFEED_CTRL, 1, chanx);

	/* Flush ProSLIC digital filters by setting to clear, while saving old values */
	for (x=0;x<5;x++) {
		tmp[x] = fxs200m_proslic_getreg_indirect(dev,  x + 35, chanx);
		fxs200m_proslic_setreg_indirect(dev, x + 35, 0x8000, chanx);
	}

	/* Power up the DC-DC converter */
	if (fxs200m_powerup_proslic(dev, chanx, fast)) {
		printk(KERN_NOTICE "Unable to do INITIAL ProSLIC powerup on chan %d\n", chanx);
		return -3;
	}

	if (!fast) {
		/* Check for power leaks */
		if (fxs200m_proslic_powerleak_test(dev, chanx)) {
			printk(KERN_NOTICE "ProSLIC chan %d failed leakage test.  Check for short circuit\n", chanx);
		}
		/* Power up again */
		if (fxs200m_powerup_proslic(dev, chanx, fast)) {
			printk(KERN_NOTICE "Unable to do FINAL ProSLIC powerup on chan %d\n", chanx);
			return -3;
		}
#ifndef NO_CALIBRATION
		/* Perform calibration */
		if(manual) {
			if (fxs200m_proslic_manual_calibrate(dev, chanx)) {
				printk(KERN_NOTICE "Proslic failed on Manual Calibration\n");
				if (fxs200m_proslic_manual_calibrate(dev, chanx)) {
					printk(KERN_NOTICE "Proslic Failed on Second Attempt to Calibrate Manually. (Try -DNO_CALIBRATION in Makefile)\n");
					return -1;
				}
				printk(KERN_NOTICE "Proslic Passed Manual Calibration on Second Attempt\n");
			}
		}
		else {
			if(fxs200m_proslic_calibrate(dev, chanx))  {
				printk(KERN_NOTICE "ProSlic died on Auto Calibration.\n");
				if (fxs200m_proslic_calibrate(dev, chanx)) {
					printk(KERN_NOTICE "Proslic Failed on Second Attempt to Auto Calibrate\n");
					return -1;
				}
				printk(KERN_NOTICE "Proslic Passed Auto Calibration on Second Attempt\n");
			}
		}
		/* Perform DC-DC calibration */
		fxs200m_write_reg(dev, REG_DCDELAY, 0x99, chanx);
		r19 = fxs200m_read_reg(dev, REG_DC_PEAK_CAL, chanx);
		if ((r19 < 0x2) || (r19 > 0xd)) {
			printk(KERN_NOTICE "DC-DC cal has a surprising direct 107 of 0x%02x!\n", r19);
			fxs200m_write_reg(dev, REG_DC_PEAK_CAL, 0x8, chanx);
		}

		/* Save calibration vectors */
		for (x=0;x<NUM_CAL_REGS;x++)
			a200m->fxsconfig[chanx].calregs.vals[x] = fxs200m_read_reg(dev, REG_CALIBR1+x, chanx);
#endif

	} else {
		/* Restore calibration registers */
		for (x=0;x<NUM_CAL_REGS;x++)
			fxs200m_write_reg(dev, REG_CALIBR1+x, a200m->fxsconfig[chanx].calregs.vals[x], chanx);
	}
	/* Calibration complete, restore original values */
	for (x=0;x<5;x++) {
		fxs200m_proslic_setreg_indirect(dev, x+35, tmp[x], chanx);
	}

	// init the indirect registers
	for (x=0; x<sizeof(indirect_regs)/sizeof(indirect_regs[0]); x++)
	{
			if (0) printk("!!!debug: init indirect reg x=%x addr=0x%x val=0x%x\n", x, indirect_regs[x].address, indirect_regs[x].initial);
			if(fxs200m_proslic_setreg_indirect(dev, indirect_regs[x].address, indirect_regs[x].initial, chanx))
				{
					printk("fxs200m: init PROSLIC indirect registers ERROR !!!\n");
					return -1;
				}
	}

	if (fxs200m_proslic_verify_indirect_regs(dev, chanx)) {
		printk(KERN_INFO "Indirect Registers failed verification.\n");
		return -1;
	}

#if 0
    /* Disable Auto Power Alarm Detect and other "features" */
    wctdm_setreg(wc, card, 67, 0x0e);
    blah = wctdm_getreg(wc, card, 67);
#endif

#if 0
    if (wctdm_proslic_setreg_indirect(wc, card, 97, 0x0)) { // Stanley: for the bad recording fix
		 printk(KERN_INFO "ProSlic IndirectReg Died.\n");
		 return -1;
	}
#endif

   if (alawoverride)
		fxs200m_write_reg(dev, 1, 0x20, chanx);
   else
		fxs200m_write_reg(dev, 1, 0x28, chanx);
    // U-Law 8-bit interface
    fxs200m_write_reg(dev, 2, (dev->slot_id+(1+chanx)*4)*8, chanx); // Tx Start count low byte  0
    fxs200m_write_reg(dev, 3, 0, chanx);	// Tx Start count high byte 0
    fxs200m_write_reg(dev, 4, (dev->slot_id+(1+chanx)*4)*8, chanx); // Rx Start count low byte  0
    fxs200m_write_reg(dev, 5, 0, chanx);	// Rx Start count high byte 0

    // clear all interrupt
    fxs200m_write_reg(dev, 18, 0xff, chanx);
    fxs200m_write_reg(dev, 19, 0xff, chanx);
    fxs200m_write_reg(dev, 20, 0xff, chanx);
    fxs200m_write_reg(dev, 73, 4, chanx);

	if (fxshonormode) {
		fxsmode = acim2tiss[fxo_modes[_opermode].acim];
		fxs200m_write_reg(dev, 10, 0x08|fxsmode, chanx);
	}
    if (lowpower)
			fxs200m_write_reg(dev, 72, 0x10, chanx);

#if 0
    fxs200m_write_reg(dev, 21, 0, chanx); // enable interrupt
    fxs200m_write_reg(dev, 22, 2, chanx); // Loop detection interrupt
    fxs200m_write_reg(dev, 23, 1, chanx); // DTMF detection interrupt
#endif

#if 0
    /* Enable loopback */
    fxs200m_write_reg(dev, 8, 2, chanx);
    fxs200m_write_reg(dev, 14, 0, chanx);
    fxs200m_write_reg(dev, 64, 0, chanx);
    fxs200m_write_reg(dev, 1, 8, chanx);
#endif
	if (fxs200m_init_ring_generator_mode(dev, chanx)) {
		return -1;
	}

	if (fastringer) {
		/* Speed up Ringer */
		fxs200m_proslic_setreg_indirect(dev, 20, 0x7e6d, chanx);
		fxs200m_proslic_setreg_indirect(dev, 21, 0x01b9, chanx);
		/* Beef up Ringing voltage to 89V */
		if (boostringer) {
	    fxs200m_write_reg(dev, 74, 0x3f, chanx);
			if (fxs200m_proslic_setreg_indirect(dev, 21, 0x247, chanx)) {
				return -1;
			}
			printk("Boosting fast ringer on slot %d (89V peak)\n", chanx + 1);
		} else if (lowpower) {
			if (fxs200m_proslic_setreg_indirect(dev, 21, 0x14b, chanx)) {
				return -1;
			}
			printk("Reducing fast ring power on slot %d (50V peak)\n", chanx + 1);
		} else {
			printk("Speeding up ringer on slot %d (25Hz)\n", chanx + 1);
		}
	} else {
		/* Beef up Ringing voltage to 89V */
		if (boostringer) {
	    fxs200m_write_reg(dev, 74, 0x3f, chanx);
			if (fxs200m_proslic_setreg_indirect(dev, 21, 0x1d1, chanx)) {
				return -1;
			}
			printk("Boosting ringer on slot %d (89V peak)\n", chanx + 1);
		} else if (lowpower) {
			if (fxs200m_proslic_setreg_indirect(dev, 21, 0x108, chanx)) {
				return -1;
			}
			printk("Reducing ring power on slot %d (50V peak)\n", chanx + 1);
		}
	}
	if (fxs200m_init_ring_generator_mode(dev, chanx)) {
		return -1;
	}

	if(fxstxgain || fxsrxgain) {
		r9 = fxs200m_read_reg(dev, 9, chanx);
		switch (fxstxgain) {

			case 35:
				r9+=8;
				break;
			case -35:
				r9+=4;
				break;
			case 0:
				break;
		}

		switch (fxsrxgain) {

			case 35:
				r9+=2;
				break;
			case -35:
				r9+=1;
				break;
			case 0:
				break;
		}
    fxs200m_write_reg(dev, 9, r9, chanx);
	}

	if(debug)
			printk(KERN_DEBUG "DEBUG: fxstxgain:%s fxsrxgain:%s\n",((fxs200m_read_reg(dev, 9, chanx)/8) == 1)?"3.5":(((fxs200m_read_reg(dev, 9, chanx)/4) == 1)?"-3.5":"0.0"),((fxs200m_read_reg(dev, 9, chanx)/2) == 1)?"3.5":((fxs200m_read_reg(dev, 9, chanx)%2)?"-3.5":"0.0"));

	a200m->fxsconfig[chanx].lasttxhook=a200m->fxsconfig[chanx].idletxhookstate;
	fxs200m_write_reg(dev, REG_LFCTRL, a200m->fxsconfig[chanx].lasttxhook, chanx);
	return 0;
}

static inline void fxs200m_proslic_recheck_sanity(struct x200_dev *dev, unsigned int chanx)
{
	struct x200fxs *a200m = dev->drv_data;
	int res;
	/* Check loopback */
	res = a200m->fxsconfig[chanx].reg1shadow;
	if (!res && (res != a200m->fxsconfig[chanx].lasttxhook))
		res = fxs200m_read_reg(dev, 64, chanx);
	if (!res && (res != a200m->fxsconfig[chanx].lasttxhook)) {
		res = fxs200m_read_reg(dev, 8, chanx);
		if (res) {
			printk(KERN_NOTICE "Ouch, part reset, quickly restoring reality (%d)\n", chanx);
			fxs200m_init_proslic(dev, chanx, 1, 0, 1);
		} else {
			if (a200m->fxsconfig[chanx].palarms++ < MAX_ALARMS) {
				printk(KERN_NOTICE "Power alarm on module %d, resetting!\n", chanx + 1);
				if (a200m->fxsconfig[chanx].lasttxhook == SLIC_LF_RINGING)
					a200m->fxsconfig[chanx].lasttxhook = SLIC_LF_ACTIVE_FWD;
				fxs200m_write_reg(dev, 64, a200m->fxsconfig[chanx].lasttxhook, chanx);
			} else {
				if (a200m->fxsconfig[chanx].palarms == MAX_ALARMS)
					printk(KERN_NOTICE "Too many power alarms on card %d, NOT resetting!\n", chanx + 1);
			}
		}
	}
}

int fxs200m_fxs_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
    struct x200fxs *a200m = chan->pvt;
    struct x200_dev *dev = a200m->dev;
		unsigned int  channo=chan->chanpos-1;
		if(0)
			printk(KERN_INFO "!!!fxs200m: hooksig dev=%p tx state=0x%x on chan[0x%x]!!!\n", dev, txsig, (chan->chanpos - 1));

		switch (txsig) {
			case DAHDI_TXSIG_ONHOOK:
				switch (chan->sig) {
					case DAHDI_SIG_FXOKS:
					case DAHDI_SIG_FXOLS:
						/* Can't change Ring Generator during OHT */
						if (!a200m->fxsconfig[channo].ohttimer) {
							fxs200m_set_ring_generator_mode(dev, channo, a200m->fxsconfig[channo].vmwi_hvac);
							a200m->fxsconfig[channo].lasttxhook = a200m->fxsconfig[channo].vmwi_hvac ? SLIC_LF_RINGING :a200m->fxsconfig[channo].idletxhookstate;
						} else {
							a200m->fxsconfig[channo].lasttxhook = a200m->fxsconfig[channo].idletxhookstate;
						}
						break;
					case DAHDI_SIG_EM:
						a200m->fxsconfig[channo].lasttxhook = a200m->fxsconfig[channo].idletxhookstate;
						break;
					case DAHDI_SIG_FXOGS:
						a200m->fxsconfig[channo].lasttxhook = SLIC_LF_TIP_OPEN;
						break;
			}
			break;
		case DAHDI_TXSIG_OFFHOOK:
			switch (chan->sig) {
					case DAHDI_SIG_EM:
						a200m->fxsconfig[channo].lasttxhook = SLIC_LF_ACTIVE_REV;
						break;
					default:
						a200m->fxsconfig[channo].lasttxhook = a200m->fxsconfig[channo].idletxhookstate;
						break;
			}
			break;
		case DAHDI_TXSIG_START:
				/* Set ringer mode */
				fxs200m_set_ring_generator_mode(dev, channo, 0);
				a200m->fxsconfig[channo].lasttxhook = SLIC_LF_RINGING;
				break;
		case DAHDI_TXSIG_KEWL:
				a200m->fxsconfig[channo].lasttxhook = SLIC_LF_OPEN;
				break;
		default:
				printk(KERN_NOTICE "fxs200m: Can't set tx state to %d\n", txsig);
		}
		if (debug) {
			printk(KERN_DEBUG
			       "Setting FXS hook state to %d (%02x)\n", txsig, a200m->fxsconfig[channo].lasttxhook);
		}

		fxs200m_write_reg(dev, REG_LFCTRL, a200m->fxsconfig[channo].lasttxhook, channo);
	  return 0;
}

inline void fxs200m_proslic_check_hook(struct x200_dev *dev, unsigned int chanx)
{
	struct x200fxs *a200m = dev->drv_data;

	char res;
	int hook;

	/* For some reason we have to debounce the hook detector.  */
	res = a200m->fxsconfig[chanx].reg0shadow;
	hook = (res & 1);
	if (hook !=  a200m->fxsconfig[chanx].lastrxhook) {
		/* Reset the debounce (must be multiple of 4ms) */
		 a200m->fxsconfig[chanx].debounce = dialdebounce * 4;
	} else {
		if ( a200m->fxsconfig[chanx].debounce > 0) {
			 a200m->fxsconfig[chanx].debounce -= 16 * DAHDI_CHUNKSIZE;
			if (! a200m->fxsconfig[chanx].debounce) {
				 a200m->fxsconfig[chanx].debouncehook = hook;
			}

			if (! a200m->fxsconfig[chanx].oldrxhook && a200m->fxsconfig[chanx].debouncehook) {
				/* Off hook */
				if (debug)
					printk(KERN_DEBUG "fxs200m: Chan %d Going off hook\n", chanx);

				switch ( a200m->fxsconfig[chanx].lasttxhook) {
					case SLIC_LF_RINGING:
					case SLIC_LF_OHTRAN_FWD:
					case SLIC_LF_OHTRAN_REV:
					    /* just detected OffHook, during Ringing or OnHookTransfer */
						a200m->fxsconfig[chanx].idletxhookstate = POLARITY_XOR(chanx) ? SLIC_LF_ACTIVE_REV:SLIC_LF_ACTIVE_FWD;
						break;
				}

				fxs200m_fxs_hooksig(a200m->chans[chanx], DAHDI_TXSIG_OFFHOOK);
				dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_OFFHOOK);
				a200m->fxsconfig[chanx].oldrxhook = 1;
			} else if ( a200m->fxsconfig[chanx].oldrxhook && ! a200m->fxsconfig[chanx].debouncehook) {
				/* On hook */
				if (debug)
					printk(KERN_DEBUG "fxs200m: Chan %d Going on hook\n", chanx);
				fxs200m_fxs_hooksig(a200m->chans[chanx], DAHDI_TXSIG_ONHOOK);
				dahdi_hooksig(a200m->chans[chanx], DAHDI_RXSIG_ONHOOK);
				a200m->fxsconfig[chanx].oldrxhook = 0;
			}

		}
	}
    a200m->fxsconfig[chanx].lastrxhook = hook;
}

void fxs200m_release(struct x200fxs *a200m)
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

    printk("fxs200m: bus %d, blade %d, slot %d Freed a OpenVox fxs200m card.\n",a200m->dev->bus->bus_id, a200m->dev->parent->blade_id, a200m->dev->slot_id);
    kfree(a200m);
    a200m = NULL;
}

int _fxs200m_open(struct dahdi_chan *chan)
{
    struct x200fxs *a200m = chan->pvt;

   	//printk(KERN_INFO "fxs200m: Calling fxs200m_open at [%p]\n", a200m);
    if(debug)
        printk("fxs200m: fxs module_open called\n");
    if (a200m->dead)
                return -ENODEV;
    a200m->usecount++;
    return 0;
}

static int fxs200m_open(struct dahdi_chan *chan)
{
	unsigned long flags;
	int res;
	spin_lock_irqsave(&chan->lock, flags);
	res = _fxs200m_open(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
	return res;
}

int fxs200m_close(struct dahdi_chan *chan)
{
		struct x200fxs *a200m = chan->pvt;

   	//printk(KERN_INFO "fxs200m: Calling fxs200m_close at [%p]\n", a200m);
    if(debug)
        printk("fxs200m: fxs module_close called\n");

    a200m->usecount--;
        /* If we're dead, release us now */
    if (!a200m->usecount && a200m->dead)
        fxs200m_release(a200m);

    return 0;
}

int set_vmwi(struct x200_dev *dev, int chan_idx)
{
	struct x200fxs *a200m = dev->drv_data;

	if (a200m->fxsconfig[chan_idx].vmwi_active_messages) {
		a200m->fxsconfig[chan_idx].vmwi_lrev =
		    (a200m->fxsconfig[chan_idx].vmwisetting.vmwi_type & DAHDI_VMWI_LREV) ? 1 : 0;
		a200m->fxsconfig[chan_idx].vmwi_hvdc =
		    (a200m->fxsconfig[chan_idx].vmwisetting.vmwi_type & DAHDI_VMWI_HVDC) ? 1 : 0;
		a200m->fxsconfig[chan_idx].vmwi_hvac =
		    (a200m->fxsconfig[chan_idx].vmwisetting.vmwi_type & DAHDI_VMWI_HVAC) ? 1 : 0;
	} else {
		a200m->fxsconfig[chan_idx].vmwi_lrev = 0;
		a200m->fxsconfig[chan_idx].vmwi_hvdc = 0;
		a200m->fxsconfig[chan_idx].vmwi_hvac = 0;
	}

	if (debug) {
		printk(KERN_DEBUG "Setting VMWI on channel %d, messages=%d, "
				"lrev=%d, hvdc=%d, hvac=%d\n",
				chan_idx,
				a200m->fxsconfig[chan_idx].vmwi_active_messages,
				a200m->fxsconfig[chan_idx].vmwi_lrev,
				a200m->fxsconfig[chan_idx].vmwi_hvdc,
				a200m->fxsconfig[chan_idx].vmwi_hvac
			  );
	}

	if (a200m->fxsconfig[chan_idx].vmwi_hvac) {
		/* Can't change ring generator while in On Hook Transfer mode*/
		if (!a200m->fxsconfig[chan_idx].ohttimer) {
			if (POLARITY_XOR(chan_idx))
				a200m->fxsconfig[chan_idx].idletxhookstate |= SLIC_LF_REVMASK;
			else
				a200m->fxsconfig[chan_idx].idletxhookstate &= ~SLIC_LF_REVMASK;
			/* Set ring generator for neon */
			fxs200m_set_ring_generator_mode(dev, chan_idx, 1);
			/* Activate ring to send neon pulses */
			a200m->fxsconfig[chan_idx].lasttxhook = SLIC_LF_RINGING;
			fxs200m_write_reg(dev, 64, a200m->fxsconfig[chan_idx].lasttxhook, chan_idx);
		}
	} else {
		if (a200m->fxsconfig[chan_idx].neonringing) {
			/* Set ring generator for normal ringer */
			fxs200m_set_ring_generator_mode(dev, chan_idx, 0);
			/* ACTIVE, polarity determined later */
			a200m->fxsconfig[chan_idx].lasttxhook = SLIC_LF_ACTIVE_FWD;
		} else if ((a200m->fxsconfig[chan_idx].lasttxhook == SLIC_LF_RINGING) ||
					(a200m->fxsconfig[chan_idx].lasttxhook == SLIC_LF_OPEN)) {
			/* Can't change polarity while ringing or when open,
				set idlehookstate instead */
			if (POLARITY_XOR(chan_idx))
				a200m->fxsconfig[chan_idx].idletxhookstate |= SLIC_LF_REVMASK;
			else
				a200m->fxsconfig[chan_idx].idletxhookstate &= ~SLIC_LF_REVMASK;
			if(debug)
					printk(KERN_DEBUG "Unable to change polarity on channel"
					    "%d, lasttxhook=0x%X\n",
				chan_idx,
				a200m->fxsconfig[chan_idx].lasttxhook
			);
			return 0;
		}
		if (POLARITY_XOR(chan_idx)) {
			a200m->fxsconfig[chan_idx].idletxhookstate |= SLIC_LF_REVMASK;
			a200m->fxsconfig[chan_idx].lasttxhook |= SLIC_LF_REVMASK;
		} else {
			a200m->fxsconfig[chan_idx].idletxhookstate &= ~SLIC_LF_REVMASK;
			a200m->fxsconfig[chan_idx].lasttxhook &= ~SLIC_LF_REVMASK;
		}
		fxs200m_write_reg(dev, 64, a200m->fxsconfig[chan_idx].lasttxhook, chan_idx);
	}
	return 0;
}

int fxs200m_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
		struct wctdm_stats stats;
		struct wctdm_regs regs;
		struct wctdm_regop regop;
		int x;
		struct x200fxs *a200m=chan->pvt;

		if(debug)
	        printk("fxs200m: fxs200m_ioctl called\n");
		switch(cmd) {
			case DAHDI_ONHOOKTRANSFER:
					if (get_user(x, (__user int *)data)) {
						return -EFAULT;
					}
					/* Active mode when idle */
					a200m->fxsconfig[chan->chanpos - 1].idletxhookstate = POLARITY_XOR(chan->chanpos - 1) ?
							SLIC_LF_ACTIVE_REV : SLIC_LF_ACTIVE_FWD;
					if (a200m->fxsconfig[chan->chanpos - 1].neonringing) {
						/* keep same Forward polarity */
						a200m->fxsconfig[chan->chanpos - 1].lasttxhook = SLIC_LF_OHTRAN_FWD;
						//printk(KERN_INFO "ioctl: Start OnHookTrans, card %d\n",chan->chanpos - 1);
						fxs200m_write_reg(a200m->dev,	64, a200m->fxsconfig[chan->chanpos - 1].lasttxhook, chan->chanpos - 1);
					} else if (a200m->fxsconfig[chan->chanpos - 1].lasttxhook == SLIC_LF_ACTIVE_FWD ||
						    a200m->fxsconfig[chan->chanpos - 1].lasttxhook == SLIC_LF_ACTIVE_REV) {
						/* Apply the change if appropriate */
						a200m->fxsconfig[chan->chanpos - 1].lasttxhook = POLARITY_XOR(chan->chanpos - 1) ?
							SLIC_LF_OHTRAN_REV : SLIC_LF_OHTRAN_FWD;
						//printk(KERN_INFO "ioctl: Start OnHookTrans, card %d\n",chan->chanpos - 1);
						fxs200m_write_reg(a200m->dev,	64, a200m->fxsconfig[chan->chanpos - 1].lasttxhook, chan->chanpos - 1);
					}
					break;
			case DAHDI_VMWI_CONFIG:
					if (copy_from_user(&(a200m->fxsconfig[chan->chanpos - 1].vmwisetting),
							   (__user void *)data,
							   sizeof(a200m->fxsconfig[chan->chanpos - 1].vmwisetting)))
						return -EFAULT;
					//printk("----fxs200m:DAHDI_VMWI_CONFIG,card=%d--\n",chan->chanpos - 1);
					set_vmwi(a200m->dev, chan->chanpos - 1);
					break;
			case DAHDI_VMWI:
					if (get_user(x, (__user int *) data))
						return -EFAULT;
					if (0 > x)
						return -EFAULT;
					a200m->fxsconfig[chan->chanpos - 1].vmwi_active_messages = x;
					set_vmwi(a200m->dev, chan->chanpos - 1);
					//printk("-----fxs200m,DAHDI_VMWI,card=%d,set_vmwi------\n",chan->chanpos - 1);
					break;
			case DAHDI_SETPOLARITY:
					if (get_user(x, (__user int *)data)) {
						return -EFAULT;
					}
					/* Can't change polarity while ringing or when open */
					if ((a200m->fxsconfig[chan->chanpos - 1].lasttxhook == SLIC_LF_RINGING) ||
						(a200m->fxsconfig[chan->chanpos - 1].lasttxhook == SLIC_LF_OPEN)) {
						return -EINVAL;
					}
					a200m->fxsconfig[chan->chanpos - 1].reversepolarity = x;
					if (POLARITY_XOR(chan->chanpos - 1)) {
						a200m->fxsconfig[chan->chanpos - 1].lasttxhook |= SLIC_LF_REVMASK;
						printk(KERN_INFO "ioctl: Reverse Polarity, card %d\n",
								chan->chanpos - 1);
					} else {
						a200m->fxsconfig[chan->chanpos - 1].lasttxhook &= ~SLIC_LF_REVMASK;
						printk(KERN_INFO "ioctl: Normal Polarity, card %d\n",
								chan->chanpos - 1);
					}
					fxs200m_write_reg(a200m->dev, 64, a200m->fxsconfig[chan->chanpos - 1].lasttxhook, chan->chanpos - 1);
					break;
			case WCTDM_GET_REGS:
					for (x=0;x<NUM_INDIRECT_REGS;x++) {
						regs.indirect[x] = fxs200m_proslic_getreg_indirect(a200m->dev, x, chan->chanpos - 1);
					}
					for (x=0;x<NUM_REGS;x++) {
						regs.direct[x] = fxs200m_read_reg(a200m->dev, x, chan->chanpos - 1);
					}

					if (copy_to_user((__user void *)data, &regs, sizeof(regs)))
						return -EFAULT;
					break;
			case WCTDM_GET_STATS:
					stats.tipvolt = (signed char)fxs200m_read_reg(a200m->dev, 80, chan->chanpos - 1) * -376;
					stats.ringvolt = (signed char)fxs200m_read_reg(a200m->dev, 81, chan->chanpos - 1) * -376;
					stats.batvolt = (signed char)fxs200m_read_reg(a200m->dev, 82, chan->chanpos - 1) * -376;
					if (copy_to_user((__user void *)data, &stats, sizeof(stats)))
						return -EFAULT;
					break;
			case WCTDM_SET_REG:
					if (copy_from_user(&regop, (__user void *)data, sizeof(regop))) {
						return -EFAULT;
					}
					if (regop.indirect) {
						printk(KERN_INFO "Setting indirect %d to 0x%04x on %d\n", regop.reg, regop.val, chan->chanpos);
						fxs200m_proslic_setreg_indirect(a200m->dev, regop.reg, regop.val, chan->chanpos - 1);
					} else {
						regop.val &= 0xff;
						printk(KERN_INFO "Setting direct %d to %04x on %d\n", regop.reg, regop.val, chan->chanpos);
						fxs200m_write_reg(a200m->dev, regop.reg, regop.val, chan->chanpos - 1);
					}
					break;
		default:
					return -ENOTTY;
		}
		return 0;
}

int fxs200m_watchdog(struct dahdi_span *span, int event)
{
	printk(KERN_INFO "fxo200m: watch dog!\n");
	return 0;
}

struct dahdi_span_ops a200m_span_ops = {
		.owner = THIS_MODULE,
		.hooksig = fxs200m_fxs_hooksig,
		.open = fxs200m_open,
		.close = fxs200m_close,
		.ioctl = fxs200m_ioctl,
		.watchdog = fxs200m_watchdog,
		.echocan_create = a200m_echocan_create,
};

int fxs200m_hardware_init(struct x200_dev *dev)
{
	int i, x;
	struct x200fxs *a200m = dev->drv_data;
	int sane=0,ret=0,readi=0;
	
	struct x200 *x2 = dev->bus;
	unsigned long flags;

    for (i = 0; i < CHANS_PER_MODULE; i++){
		//printk("fxs200m: detect chan[%d] at module %d.\n", i, dev->slot_id);
		if (0)	{
			printk("!below: debugging info for si3215 on module %d! \n", dev->slot_id);
		    // dump the chip[0] and chip[1] registers
			for (x=0; x<110; x++) {
				printk(" %02x", fxs200m_read_reg(dev, x, i));
				if ((x & 15) == 15) printk("\n");
			}
		    printk("\n");
		}

		// init the indirect registers
		for (x=0; x<sizeof(indirect_regs)/sizeof(indirect_regs[0]); x++) {
            //printk("!!!debug: init indirect reg x=%x addr=0x%x val=0x%x\n", x, indirect_regs[x].address, indirect_regs[x].initial);
		    if(fxs200m_proslic_setreg_indirect(dev, indirect_regs[x].address, indirect_regs[x].initial, i)) {
		        printk("fxs200m: init PROSLIC indirect registers ERROR !!!\n");
			    return -1;
		    }
		}
	}

    for (i = 0; i < CHANS_PER_MODULE; i++) {
        /* Init with Auto Calibration */
        ret=fxs200m_init_proslic(dev, i, 0, 0, sane) ;
        if (!ret) {
			//printk(KERN_INFO "fxs200m: Module %d Installed -- AUTO FXS/DPO\n", i);
					spin_lock_irqsave(&x2->lock, flags);
            __x200_set_led(dev->bus, dev->slot_id*CHANS_PER_MODULE+i, LED_GREEN);
          spin_unlock_irqrestore(&x2->lock, flags);
		} else {
	        printk("-----------------!!!!!!! wujc-debug: init_proslic ret=%x  !!!!!!!!!!!!!!!!!-------------------\n", ret);
		 	if(ret==-3) {
		 		//flag means powerup failure
				continue;
			}

			if(ret!=-2) {
				sane=1;
		 	    printk("-----------------!!!!!!! wujc-debug: Init with Manual Calibration  !!!!!!!!!!!!!!!!!-------------------\n");
				/* Init with Manual Calibration */
				if (!fxs200m_init_proslic(dev, i, 0, 1, sane)) {
                    if (debug) {
                        readi = fxs200m_read_reg(dev, REG_LOOP_I_LIMIT, i);
                        printk(KERN_DEBUG "Proslic module %d loop current is %dmA\n",i, ((readi*3)+20));
                    }
					printk(KERN_INFO "Module %d: Installed -- MANUAL FXS\n", i);
				} else {
					printk(KERN_NOTICE "Module %d: FAILED FXS (%s)\n", i, fxshonormode ? fxo_modes[_opermode].name : "FCC");
					printk(KERN_NOTICE "Module %d: FAILED FXS\n", i);
					a200m->chans[i]->sigcap = __DAHDI_SIG_FXO | DAHDI_SIG_BROKEN;
				}
			}
		}
    }
	return ret;
}

int fxs200m_software_init(struct x200_dev *dev)
{
		struct x200fxs *a200m = dev->drv_data;
		struct x200* x2 = dev->bus;
		int x,channel;
		int retval;
		
		if (debug)
			printk("!!!DEBUG: a200m=%p dev=%p slot_id=%d !!!\n", a200m, dev, dev->slot_id);

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

    sprintf(a200m->span.name, "FXS200M/%d", dev->slot_id);
    snprintf(a200m->span.desc, sizeof(a200m->span.desc) - 1, "OpenVox FXS200M Card %d", dev->slot_id);

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)    
    a200m->ddev->manufacturer = "OpenVox";
    a200m->ddev->devicetype = "OpenVox FXS200M Card";
    a200m->ddev->location = kasprintf(GFP_KERNEL,"X200 Bus %02d Blade %d Slot %02d", 
    																						x2->bus_id, dev->parent->blade_id, dev->slot_id);
		if(!a200m->ddev->location){
    		retval = -ENOMEM;
    		goto a200m_init_loc_err;
    }
#else
		a200m->span.manufacturer = "OpenVox";
    dahdi_copy_string(a200m->span.devicetype, "OpenVox FXS200M Card", sizeof(a200m->span.devicetype));
    snprintf(a200m->span.location, sizeof(a200m->span.location) - 1,"X200 Bus %02d Blade %d Slot %02d", x2->bus_id, dev->parent->blade_id, dev->slot_id);
    a200m->span.irq = x2->irq;
#endif 
   
    a200m->span.channels = CHANS_PER_MODULE;
	  if (alawoverride) {
			a200m->span.deflaw = DAHDI_LAW_ALAW;
		}else {
			a200m->span.deflaw = DAHDI_LAW_MULAW;
		}
		
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,7,0)		
		a200m->span.spantype = SPANTYPE_ANALOG_FXS;
#endif

		a200m->span.chans = a200m->chans;
		a200m->span.flags = DAHDI_FLAG_RBS;
#if DAHDI_VERSION_CODE < VERSION_CODE(2,6,0)	
		init_waitqueue_head(&a200m->span.maintq);
#endif

		for (x=0;x<a200m->span.channels;x++) {
			sprintf(a200m->chans[x]->name, "FXS200M");
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
			a200m->chans[x]->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
		}
		
		//init ec channels
    if(x2->vpm_present){
    	for (x=0;x<a200m->span.channels;x++){
    		channel = dev->slot_id + a200m->chans[x]->chanpos*4;
				if(init_vpm450m_chan(x2, channel,dev->slot_id)){
    			retval =  -2;
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
#else
a200m_init_err:		
#endif
	
		for (x = 0; x < CHANS_PER_MODULE; x++) 
			kfree(a200m->ec[x]);
			
		return retval;
}

int fxs200m_init_one(struct x200_dev *dev)
{
  struct x200* x2 = dev->bus;
	int retval = 0;
	struct x200fxs *a200m = NULL;

	a200m = kmalloc(sizeof(*a200m), GFP_KERNEL);
	if(NULL==a200m)
	    return -ENOMEM;

	memset(a200m, 0, sizeof(*a200m));
	a200m->dev = dev;
	dev->drv_data = a200m;

  x2->card_type[dev->blade_id][dev->slot_id] = DEV_TYPE_FXS200M ;

	retval = fxs200m_hardware_init(dev);
	if( retval ) {
		  kfree(a200m);
		  dev->drv_data = NULL;
	    return retval ;
	}

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)	
	a200m->ddev = dahdi_create_device();
	if(!a200m->ddev){
			kfree(a200m);
			dev->drv_data = NULL;
			retval = -ENOMEM;
	}
#endif
		
	retval = fxs200m_software_init(dev);
	if(retval){
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)	
			dahdi_free_device(a200m->ddev);
#endif
			kfree(a200m);
			dev->drv_data = NULL;
			return retval;
	}
	
	a200m->exist = 1;
  
  printk("fxs200m driver loaded : init device on bus %d blade %d slot %d successfully.\n",dev->bus_id,dev->blade_id,dev->slot_id);
      
	return retval;
}

void fxs200m_remove_one(struct x200_dev *dev)
{
	struct x200* x2 = dev->bus;
	unsigned long flags;
	struct x200fxs *a200m;
	int i;

	a200m = dev->drv_data;

	if(NULL==a200m) {
		return;
	}

	if(debug)
			printk(KERN_INFO "fxs200m: Calling fxs200m_remove_one [%p]=0x%p\n", dev, a200m);

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
	if (!a200m->usecount)
    fxs200m_release(a200m);
	else
    a200m->dead = 1;

	dev->drv_data = NULL;
	dev->ops = NULL;
	return;
}

void fxs200m_transmitprep(struct x200_dev *dev)
{
	int x, y;
	struct x200fxs* a200m = dev->drv_data;
	struct x200* x2 = dev->bus;

#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		_dahdi_transmit(&a200m->span);
#else
    dahdi_transmit(&a200m->span);
#endif
	for (x=0; x<CHANS_PER_MODULE; x++) {
	    for (y=0;y<DAHDI_CHUNKSIZE;y++) {
				x2->current_writechunk[y*128+dev->slot_id+(1+x)*4] = a200m->chans[x]->writechunk[y];
		}
	}
}

void fxs200m_receiveprep(struct x200_dev *dev)
{
	int x, y;
	struct x200fxs* a200m = dev->drv_data;
	struct x200* x2 = dev->bus;
	
	for (x=0; x<CHANS_PER_MODULE; x++) {
	    for (y=0;y<DAHDI_CHUNKSIZE;y++) {
			a200m->chans[x]->readchunk[y] = x2->current_readchunk[y*128+dev->slot_id+(1+x)*4];
		}
	}
	
	for (x=0; x<CHANS_PER_MODULE; x++) {
		dahdi_ec_chunk(a200m->chans[x], a200m->chans[x]->readchunk, a200m->chans[x]->writechunk);
	}
#if DAHDI_VERSION_CODE >= VERSION_CODE(2,6,0)
		_dahdi_receive(&a200m->span);
#else
    dahdi_receive(&a200m->span);
#endif
}

int fxs200m_interrupt(int intcount, struct x200_dev *dev)
{
	struct x200fxs* a200m = dev->drv_data;
	int chan_sel, mode;
	chan_sel = intcount&0x1;
    mode = ((intcount>>1) + dev->slot_id)&0x7 ;

	switch(mode) {
		case 0:
			/* Rest */
			if (a200m->fxsconfig[chan_sel].lasttxhook == SLIC_LF_RINGING && !a200m->fxsconfig[chan_sel].neonringing) {
				/* RINGing, prepare for OHT */
				a200m->fxsconfig[chan_sel].ohttimer = OHT_TIMER << 3;

				/* logical XOR 3 variables
					module parameter 'reversepolarity', global reverse all FXS lines.
					ioctl channel variable fxs 'reversepolarity', Line Reversal Alert Signal if required.
					ioctl channel variable fxs 'vmwi_lrev', VMWI pending.
				*/
				/* OHT mode when idle */
				a200m->fxsconfig[chan_sel].idletxhookstate = POLARITY_XOR(chan_sel) ?	SLIC_LF_OHTRAN_REV:SLIC_LF_OHTRAN_FWD;
			} else if (a200m->fxsconfig[chan_sel].ohttimer) {
		        /* check if still OnHook */
				if (!a200m->fxsconfig[chan_sel].oldrxhook) {
					a200m->fxsconfig[chan_sel].ohttimer -= DAHDI_CHUNKSIZE;
					if (!a200m->fxsconfig[chan_sel].ohttimer) {
						a200m->fxsconfig[chan_sel].idletxhookstate = POLARITY_XOR(chan_sel)?SLIC_LF_ACTIVE_REV:SLIC_LF_ACTIVE_FWD; /* Switch to Active, Rev or Fwd */
						/* if currently OHT */
						if ((a200m->fxsconfig[chan_sel].lasttxhook == SLIC_LF_OHTRAN_FWD) || (a200m->fxsconfig[chan_sel].lasttxhook == SLIC_LF_OHTRAN_REV)) {
							if (a200m->fxsconfig[chan_sel].vmwi_hvac) {
								/* force idle polarity Forward if ringing */
								a200m->fxsconfig[chan_sel].idletxhookstate = SLIC_LF_ACTIVE_FWD;
								/* Set ring generator for neon */
								fxs200m_set_ring_generator_mode(dev, 1, chan_sel);
								a200m->fxsconfig[chan_sel].lasttxhook = SLIC_LF_RINGING;
						    } else {
							    a200m->fxsconfig[chan_sel].lasttxhook = a200m->fxsconfig[chan_sel].idletxhookstate;
						    }
						    /* Apply the change as appropriate */
							fxs200m_write_reg(dev, REG_LFCTRL, a200m->fxsconfig[chan_sel].lasttxhook, chan_sel);
						}
					}
				} else {
					a200m->fxsconfig[chan_sel].ohttimer = 0;
					/* Switch to Active, Rev or Fwd */
					a200m->fxsconfig[chan_sel].idletxhookstate = POLARITY_XOR(chan_sel)?SLIC_LF_ACTIVE_REV:SLIC_LF_ACTIVE_FWD;
				}
		    }
			break;
		case 1:
			break;
		case 2:
			/* Read first shadow reg */
			a200m->fxsconfig[chan_sel].reg0shadow = fxs200m_read_reg(dev, REG_LOOPRTD_STS, chan_sel);
			break;
		case 3:
			break;
		case 4:
			/* Read second shadow reg */
			a200m->fxsconfig[chan_sel].reg1shadow = fxs200m_read_reg(dev, REG_LFCTRL, chan_sel);
			break;
		case 5:
			break;
		case 6:
			/* Perform processing */
			fxs200m_proslic_check_hook(dev, chan_sel);
			if (!(intcount & 0xf0)) {
			    fxs200m_proslic_recheck_sanity(dev, chan_sel);
			}
			break;
		case 7:
			break;
	}

	if (!(intcount % 10000)) {
		/* Accept an alarm once per 10 seconds */
		for (chan_sel=0;chan_sel<2;chan_sel++)
			if (a200m->fxsconfig[chan_sel].palarms)
				a200m->fxsconfig[chan_sel].palarms--;
	}

	fxs200m_transmitprep(dev);
	fxs200m_receiveprep(dev);
	return IRQ_RETVAL(1);
}

struct ints_ops fxs200m_ops={
		.irq_handler = fxs200m_interrupt,
    .get_clk_src = NULL,
    .set_clk_src = NULL,
    .unset_clk_src = NULL,
};

static int x200_dev_probe(struct device *dev)
{
		int ret;
		struct x200_dev *x2dev = to_x200_dev(dev);
		ret = fxs200m_init_one(x2dev);
		if(!ret){
				x2dev->ops = &fxs200m_ops;
		}
		return ret;
}

static int x200_dev_remove(struct device *dev)
{
		struct x200_dev *x2dev = to_x200_dev(dev);
		fxs200m_remove_one(x2dev);
		return 0;
}

int __init fxs200m_init(void)
{
	int x;
	struct bus_type *x200bus;
	
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

   for(x=0;x<interface_num;x++){
    	x200bus = get_bus(x);
    	x200_dev_driver[x].name = "fxs200m_driver";
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

void __exit fxs200m_cleanup(void)
{
		int i;
		
		for(i=0;i<interface_num;i++){
				driver_unregister(&x200_dev_driver[i]);
		}
}

module_param(debug, int, 0600);
module_param(alawoverride, int, 0600);
module_param(loopcurrent, int, 0600);
module_param(lowpower, int, 0600);
module_param(boostringer, int, 0600);
module_param(fastringer, int, 0600);
module_param(reversepolarity, int, 0600);
module_param(fxstxgain, int, 0600);
module_param(fxsrxgain, int, 0600);
module_param(fxshonormode, int, 0600);
module_param(dialdebounce, int, 0600);

MODULE_DESCRIPTION("OpenVox FXS module of x200 card Driver ");
MODULE_AUTHOR("Miao Lin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(fxs200m_init);
module_exit(fxs200m_cleanup);
