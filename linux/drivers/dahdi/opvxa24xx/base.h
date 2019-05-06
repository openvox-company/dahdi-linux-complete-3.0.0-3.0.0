/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * $Id: base.h 360 2011-04-06 06:11:45Z yangshugang $
 
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


#ifndef _OPVXA2410P_H
#define _OPVXA2410P_H

#include <dahdi/kernel.h>
#include <linux/firmware.h>

#ifndef SLIC_LF_OPEN
/* Proslic Linefeed options for register 64 - Linefeed Control */
#define SLIC_LF_OPEN		0x0
#define SLIC_LF_ACTIVE_FWD	0x1
#define SLIC_LF_OHTRAN_FWD	0x2 /* Forward On Hook Transfer */
#define SLIC_LF_TIP_OPEN	0x3
#define SLIC_LF_RINGING		0x4
#define SLIC_LF_ACTIVE_REV	0x5
#define SLIC_LF_OHTRAN_REV	0x6 /* Reverse On Hook Transfer */
#define SLIC_LF_RING_OPEN	0x7

#define SLIC_LF_SETMASK		0x7
#define SLIC_LF_OPPENDING 	0x10

/* Mask used to reverse the linefeed mode between forward and
 * reverse polarity. */
#define SLIC_LF_REVMASK 	0x4
#endif

struct ec;

#define VPM_SUPPORT

#define CARDS_PER_MODULE    4
#define MOD_TYPE_FXS	0
#define MOD_TYPE_FXO	1

#define NUM_FXO_REGS 60
#define WC_MAX_IFACES 128
#define DEFAULT_RING_DEBOUNCE		64		/* Ringer Debounce (64 ms) */

/* the constants below control the 'debounce' periods enforced by the
   check_hook routines; these routines are called once every 4 interrupts
   (the interrupt cycles around the four modules), so the periods are
   specified in _4 millisecond_ increments
*/
#define DEFAULT_BATT_DEBOUNCE	4		/* Battery debounce (64 ms) */
#define POLARITY_DEBOUNCE 	64		/* Polarity debounce (64 ms) */
#define DEFAULT_BATT_THRESH	3		/* Anything under this is "no battery" */

#define OHT_TIMER		6000	/* How long after RING to retain OHT */

#define NEON_MWI_RNGY_PULSEWIDTH	0x3e8	/*=> period of 250 mS */

#define FLAG_3215	(1 << 0)

#define MAX_NUM_CARDS 24

enum cid_hook_state {
	CID_STATE_IDLE = 0,
	CID_STATE_RING_DELAY,
	CID_STATE_RING_ON,
	CID_STATE_RING_OFF,
	CID_STATE_WAIT_RING_FINISH
};

/* if you want to record the last 8 sec voice before the driver unload, uncomment it and rebuild. */
/* #define TEST_LOG_INCOME_VOICE */
#define voc_buffer_size (8000*8)
#define MAX_ALARMS 10
#define MINPEGTIME	10 * 8		/* 30 ms peak to peak gets us no more than 100 Hz */
#define PEGTIME		50 * 8		/* 50ms peak to peak gets us rings of 10 Hz or more */
#define PEGCOUNT	5			/* 5 cycles of pegging means RING */

#define NUM_CAL_REGS 12


#define __CMD_RD   (1 << 20)		/* Read Operation */
#define __CMD_WR   (1 << 21)		/* Write Operation */
#define __CMD_FIN  (1 << 22)		/* Has finished receive */
#define __CMD_TX   (1 << 23)		/* Has been transmitted */

#define CMD_WR(a,b) (((a) << 8) | (b) | __CMD_WR)
#define CMD_RD(a) (((a) << 8) | __CMD_RD)


struct calregs {
	unsigned char vals[NUM_CAL_REGS];
};

enum proslic_power_warn {
	PROSLIC_POWER_UNKNOWN = 0,
	PROSLIC_POWER_ON,
	PROSLIC_POWER_WARNED,
};

enum battery_state {
	BATTERY_UNKNOWN = 0,
	BATTERY_PRESENT,
	BATTERY_LOST,
};

struct a24xx_dev {
	struct pci_dev *dev;
	char *variety;
	struct dahdi_device *ddev;
	unsigned char ios;
	int usecount;
	unsigned int intcount;
	int dead;
	int pos;
	int flags[MAX_NUM_CARDS];
	int freeregion;
	int alt;
	int curcard;
	int cardflag;		/* Bit-map of present cards */
	enum proslic_power_warn proslic_power;
	spinlock_t lock;
	
	union {
		struct fxo {
#ifdef AUDIO_RINGCHECK
			unsigned int pegtimer;
			int pegcount;
			int peg;
			int ring;
#else
			int wasringing;
			int lastrdtx;
#endif
			int ringdebounce;
			int offhook;
			unsigned int battdebounce;
			unsigned int battalarm;
			enum battery_state battery;
			int lastpol;
			int polarity;
			int polaritydebounce;
		} fxo;
		struct fxs {
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
		} fxs;
	} mod[MAX_NUM_CARDS];

	/* Receive hook state and debouncing */
	int modtype[MAX_NUM_CARDS];
	unsigned char reg0shadow[MAX_NUM_CARDS];
	unsigned char reg1shadow[MAX_NUM_CARDS];
	
	unsigned long mem_region;	/* 32 bit Region allocated to tiger320 */
	unsigned long mem_len;		/* Length of 32 bit region */
	volatile unsigned long mem32;	/* Virtual representation of 32 bit memory area */

	dma_addr_t 	readdma;
	dma_addr_t	writedma;
	volatile unsigned char *writechunk;					/* Double-word aligned write memory */
	volatile unsigned char *readchunk;					/* Double-word aligned read memory */

#ifdef TEST_LOG_INCOME_VOICE
	char * voc_buf[MAX_NUM_CARDS];
	int voc_ptr[MAX_NUM_CARDS];
#endif
	unsigned short ledstate;
	unsigned int fwversion;
	int max_cards;
	char *card_name;
	unsigned int master;

	char 	*cid_history_buf[MAX_NUM_CARDS];
	int	    cid_history_ptr[MAX_NUM_CARDS];
	int  	cid_history_clone_cnt[MAX_NUM_CARDS];
	enum 	cid_hook_state cid_state[MAX_NUM_CARDS];
	int 	cid_ring_on_time[MAX_NUM_CARDS];

#ifdef VPM_SUPPORT
	struct ec *vpm_ec;
	int vpm;

	unsigned long dtmfactive;
	unsigned long dtmfmask;
	unsigned long dtmfmutemask;

#endif

};

struct a24xx_desc {
	char *name;
	int flags;
};

struct a24xx {
        struct a24xx_dev dev;
        struct dahdi_span span;
        struct dahdi_chan _chans[MAX_NUM_CARDS];
        struct dahdi_chan *chans[MAX_NUM_CARDS];
        struct dahdi_echocan_state *ec[32];                 /* Echcan state for each channel */
        unsigned int index;
};

/* 
 * from bin 
 */
extern void __opvx_a24xx_setcreg(unsigned long mem32, unsigned int offset, unsigned int reg, unsigned int val);
extern unsigned int __opvx_a24xx_getcreg(unsigned long mem32, unsigned int offset, unsigned char reg);
extern unsigned char __opvx_a24xx_read_8bits(unsigned long mem32);

extern void __opvx_a24xx_reset_modules(unsigned long mem32, void (*func)(int), int data);
extern void __opvx_a24xx_reset_modules_v2(unsigned long mem32, void (*func)(int), int data);
extern void __opvx_a24xx_setcard(unsigned long mem32, int card);
extern void __opvx_a24xx_reset_spi(void *wc_dev, int card, void (*func)(void*, int));
extern void __opvx_a24xx_write_8bits(unsigned long mem32, unsigned char bits);

extern void __opvx_a24xx_set_master(unsigned long mem32,unsigned int master);
extern unsigned int __opvx_a24xx_get_master(unsigned long mem32);
extern void __opvx_a24xx_set_irq_frq(unsigned long mem32,unsigned int frq);
extern unsigned int __opvx_a24xx_get_version(unsigned long mem32);
extern unsigned int __opvx_a24xx_get_irqstatus(unsigned long mem32);
extern void __opvx_a24xx_set_irqstatus(unsigned long mem32, unsigned int value);
extern void __opvx_a24xx_clear_irqs(unsigned long mem32);
extern void __opvx_a24xx_enable_interrupts(unsigned long mem32);
extern void __opvx_a24xx_disable_interrupts(unsigned long mem32);
extern unsigned int __opvx_a24xx_get_irqcnt_lo(unsigned long mem32);

extern void __opvx_a24xx_restart_dma(unsigned long mem32);
extern void __opvx_a24xx_start_dma(unsigned long mem32, unsigned int data);
extern void __opvx_a24xx_stop_dma(unsigned long mem32);
extern void __opvx_a24xx_reset_tdm(unsigned long mem32);

extern void __opvx_a24xx_spi_setreg(void *wc_dev, unsigned long mem32, int card, int modtype, unsigned char reg, unsigned char value, void (*func)(void*, int));
extern unsigned char __opvx_a24xx_spi_getreg(void *wc_dev, unsigned long mem32, int card, int modtype, unsigned char reg, void (*func)(void*, int));

extern unsigned int __opvx_a24xx_oct_in(unsigned long mem32, unsigned int addr);
extern unsigned int __opvx_a24xx_oct_in_v2(unsigned long mem32, unsigned int addr);
extern void __opvx_a24xx_oct_out(unsigned long mem32, unsigned int addr, unsigned int value);
extern void __opvx_a24xx_oct_out_v2(unsigned long mem32, unsigned int addr, unsigned int value);
extern int __opvx_a24xx_check_vpm(unsigned long mem32);
extern int __opvx_a24xx_check_vpm_v2(unsigned long mem32);
extern void __opvx_a24xx_vpm_setpresent(unsigned long mem32);
extern void __opvx_a24xx_vpm_setpresent_v2(unsigned long mem32);

extern void __opvx_a24xx_transmit(unsigned long mem32, volatile unsigned char *writechunk, volatile unsigned char **txbuf,unsigned int irq_frq , unsigned int order);
extern void __opvx_a24xx_receive(unsigned long mem32, volatile unsigned char *readchunk, volatile unsigned char **rxbuf,unsigned int irq_frq , unsigned int order);

extern void __opvx_a24xx_set_chunk(void *readchunk, void *writechunk,unsigned int frq);

/*
 * from a24xx.c
 */
extern void __a24xx_wait_just_a_bit(int foo);
extern void __a24xx_spi_setreg(struct a24xx_dev *wc_dev, int card, unsigned char reg, unsigned char value);
extern unsigned char __a24xx_spi_getreg(struct a24xx_dev *wc_dev, int card, unsigned char reg);
extern void a24xx_spi_setreg(struct a24xx_dev *wc_dev, int card, unsigned char reg, unsigned char value);
extern unsigned char a24xx_spi_getreg(struct a24xx_dev *wc_dev, int card, unsigned char reg);
extern void a24xx_reset_spi(struct a24xx_dev *wc_dev, int card);
extern void __a24xx_setcard(void *wc_dev, int card);
extern void oct_set_reg(void *data, unsigned int reg, unsigned int val);
extern unsigned int oct_get_reg(void *data, unsigned int reg);
extern void __a24xx_vpm_setpresent(struct a24xx_dev *wc_dev);
extern int __a24xx_proslic_setreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address, unsigned short data);
extern int __a24xx_proslic_getreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address);
extern int __a24xx_malloc_chunk(struct a24xx_dev *wc_dev,unsigned int frq);

/*
 * from si321x.c
 */
extern int si321x_init_ring_generator_mode(struct a24xx_dev *wc_dev, int card);
extern int si321x_set_ring_generator_mode(struct a24xx_dev *wc_dev, int card, int mode);
extern int si321x_init_proslic(struct a24xx_dev *wc_dev, int card, int fast, int manual, int sane);
extern int si321x_init_proslic_all(struct a24xx_dev *wc_dev, int fxs_flag,int fast, int manual, int sane,int *blk_flag);
extern int si321x_proslic_setreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address, unsigned short data);
extern int si321x_proslic_getreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address);
extern void si321x_proslic_recheck_sanity(struct a24xx_dev *wc_dev, int card);

/* 
 * from si3050.c 
 */
int si3050_set_hwgain(struct a24xx_dev *wc_dev, int card, __s32 gain, __u32 tx);
int si3050_init_voicedaa(struct a24xx_dev *wc_dev, int card, int fast, int manual, int sane);

#endif

