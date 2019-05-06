/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * Written by mark.liu<mark.liu@openvox.cn>
 * $Id: si3050.c 301 2011-01-19 05:20:32Z yangshugang $
 *
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
 
#include <linux/string.h>
#include <linux/param.h>
#include <linux/jiffies.h>

#include "fxo_modes.h"
#include "base.h"

extern int debug;
extern int alawoverride;
extern int fxofullscale;	/* fxo full scale tx/rx, register 30, acim */
extern int fwringdetect;
extern int _opermode;
extern int fxotxgain;
extern int fxorxgain;
extern int fastpickup;

static int si3050_voicedaa_insane(struct a24xx_dev *wc_dev, int card)
{
	int blah;
	blah = a24xx_spi_getreg(wc_dev, card, 2);
	if (blah != 0x3) {
		return -2;
	}
	blah = a24xx_spi_getreg(wc_dev, card, 11);
	if (debug) {
		printk("VoiceDAA System: %02x\n", blah & 0xf);
	}
	return 0;
}

/*********************************************************************
 * Set the hwgain on the analog modules
 *
 * card = the card position for this module (0-23)
 * gain = gain in dB x10 (e.g. -3.5dB  would be gain=-35)
 * tx = (0 for rx; 1 for tx)
 *
 *******************************************************************/
int si3050_set_hwgain(struct a24xx_dev *wc_dev, int card, __s32 gain, __u32 tx)
{
	if (!(wc_dev->modtype[card] == MOD_TYPE_FXO)) {
		printk("Cannot adjust gain.  Unsupported module type!\n");
		return -1;
	}
	if (tx) {
		if (debug) {
			printk("setting FXO tx gain for card=%d to %d\n", card, gain);
		}
		if (gain >=  -150 && gain <= 0) {
			a24xx_spi_setreg(wc_dev, card, 38, 16 + (gain/-10));
			a24xx_spi_setreg(wc_dev, card, 40, 16 + (-gain%10));
		} else if (gain <= 120 && gain > 0) {
			a24xx_spi_setreg(wc_dev, card, 38, gain/10);
			a24xx_spi_setreg(wc_dev, card, 40, (gain%10));
		} else {
			printk("FXO tx gain is out of range (%d)\n", gain);
			return -1;
		}
	} else { /* rx */
		if (debug) {
			printk("setting FXO rx gain for card=%d to %d\n", card, gain);
		}
		if (gain >=  -150 && gain <= 0) {
			a24xx_spi_setreg(wc_dev, card, 39, 16+ (gain/-10));
			a24xx_spi_setreg(wc_dev, card, 41, 16 + (-gain%10));
		} else if (gain <= 120 && gain > 0) {
			a24xx_spi_setreg(wc_dev, card, 39, gain/10);
			a24xx_spi_setreg(wc_dev, card, 41, (gain%10));
		} else {
			printk("FXO rx gain is out of range (%d)\n", gain);
			return -1;
		}
	}

	return 0;
}

int si3050_init_voicedaa(struct a24xx_dev *wc_dev, int card, int fast, int manual, int sane)
{
	unsigned char reg16=0, reg26=0, reg30=0, reg31=0;
	unsigned char ch;
	long newjiffies;

	wc_dev->modtype[card] = MOD_TYPE_FXO;
	/* Sanity check the ProSLIC */
	a24xx_reset_spi(wc_dev, card);
	if (!sane && si3050_voicedaa_insane(wc_dev, card)) {
		return -2;
	}

	/* Software reset */
	a24xx_spi_setreg(wc_dev, card, 1, 0x80);

	/* Wait just a bit */
	__a24xx_wait_just_a_bit(HZ/10);

	/* Enable PCM, ulaw */
	if (alawoverride) {
		a24xx_spi_setreg(wc_dev, card, 33, 0x20);
	} else {
		a24xx_spi_setreg(wc_dev, card, 33, 0x28);
	}

	/* Set On-hook speed, Ringer impedence, and ringer threshold */
	reg16 |= (fxo_modes[_opermode].ohs << 6);
	reg16 |= (fxo_modes[_opermode].rz << 1);
	reg16 |= (fxo_modes[_opermode].rt);
	a24xx_spi_setreg(wc_dev, card, 16, reg16);

	if(fwringdetect) {
		/* Enable ring detector full-wave rectifier mode */
		a24xx_spi_setreg(wc_dev, card, 18, 2);
		a24xx_spi_setreg(wc_dev, card, 24, 0);
	} else {
		/* Set to the device defaults */
		a24xx_spi_setreg(wc_dev, card, 18, 0);
		a24xx_spi_setreg(wc_dev, card, 24, 0x19);
	}

	/* Set DC Termination:
	   Tip/Ring voltage adjust, minimum operational current, current limitation */
	reg26 |= (fxo_modes[_opermode].dcv << 6);
	reg26 |= (fxo_modes[_opermode].mini << 4);
	reg26 |= (fxo_modes[_opermode].ilim << 1);
	a24xx_spi_setreg(wc_dev, card, 26, reg26);

	/* Set AC Impedence */
	reg30 = (fxofullscale==1) ? (fxo_modes[_opermode].acim|0x10) :  (fxo_modes[_opermode].acim);
	a24xx_spi_setreg(wc_dev, card, 30, reg30);

	/* Misc. DAA parameters */
	if (fastpickup) {
		reg31 = 0xb3;
	} else {
		reg31 = 0xa3;
	}

	reg31 |= (fxo_modes[_opermode].ohs2 << 3);
	a24xx_spi_setreg(wc_dev, card, 31, reg31);

	/* Set Transmit/Receive timeslot */
	//printk("set card %d to %d\n", card, (3-(card%4)) * 8 + (card/4) * 64);
	a24xx_spi_setreg(wc_dev, card, 34, (card%4) * 8 + (card/4) * 32);
	a24xx_spi_setreg(wc_dev, card, 35, 0x00);
	a24xx_spi_setreg(wc_dev, card, 36, (card%4) * 8 + (card/4) * 32);
	a24xx_spi_setreg(wc_dev, card, 37, 0x00);

	/* Enable ISO-Cap */
	a24xx_spi_setreg(wc_dev, card, 6, 0x00);

	if (fastpickup) {
		a24xx_spi_setreg(wc_dev, card, 17, a24xx_spi_getreg(wc_dev, card, 17) | 0x20);
	}

	/* Wait 1000ms for ISO-cap to come up */
	newjiffies = jiffies;
	newjiffies += 2 * HZ;
	while((jiffies < newjiffies) && !(a24xx_spi_getreg(wc_dev, card, 11) & 0xf0)) {
		__a24xx_wait_just_a_bit(HZ/10);
	}

	/*if (!(a24xx_spi_getreg(wc_dev, card, 11) & 0xf0)) {*/
	ch = a24xx_spi_getreg(wc_dev, card, 11);
	if( ch == 0xff ) {
                printk("VoiceDAA not installed at card %d\n", card);
                return -1;
        }
	if (!(ch & 0xf0)) {
		printk("VoiceDAA did not bring up ISO link properly!\n");
		return -1;
	}
	if (debug) {
		printk("ISO-Cap is now up, line side: %02x rev %02x\n",
				a24xx_spi_getreg(wc_dev, card, 11) >> 4,
				(a24xx_spi_getreg(wc_dev, card, 13) >> 2) & 0xf);
	}
	/* Enable on-hook line monitor */
	a24xx_spi_setreg(wc_dev, card, 5, 0x08);

	/* Take values for fxotxgain and fxorxgain and apply them to module */
	si3050_set_hwgain(wc_dev, card, fxotxgain, 1);
	si3050_set_hwgain(wc_dev, card, fxorxgain, 0);

	/* NZ -- crank the tx gain up by 7 dB */
	if (!strcmp(fxo_modes[_opermode].name, "NEWZEALAND")) {
		printk("Adjusting gain\n");
		si3050_set_hwgain(wc_dev, card, 7, 1);
	}

	if(debug) {
		printk("DEBUG fxotxgain:%i.%i fxorxgain:%i.%i\n", 
			(a24xx_spi_getreg(wc_dev, card, 38)/16) ? -(a24xx_spi_getreg(wc_dev, card, 38) - 16) : a24xx_spi_getreg(wc_dev, card, 38), 
			(a24xx_spi_getreg(wc_dev, card, 40)/16) ? -(a24xx_spi_getreg(wc_dev, card, 40) - 16) : a24xx_spi_getreg(wc_dev, card, 40), 
			(a24xx_spi_getreg(wc_dev, card, 39)/16) ? -(a24xx_spi_getreg(wc_dev, card, 39) - 16) : a24xx_spi_getreg(wc_dev, card, 39),
			(a24xx_spi_getreg(wc_dev, card, 41)/16) ? -(a24xx_spi_getreg(wc_dev, card, 41) - 16) : a24xx_spi_getreg(wc_dev, card, 41));
	}

	/* battery state still unknown */
	return 0;

}
