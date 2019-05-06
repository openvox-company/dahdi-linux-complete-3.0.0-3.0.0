/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * Written by mark.liu<mark.liu@openvox.cn>
 * $Id: si321x.c 482 2011-06-02 08:58:56Z liuyuan $
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

#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/nmi.h>
#endif /* 4.11.0 */
 
#include "proslic.h"
#include "fxo_modes.h"
#include "base.h"

/* module param */
extern int debug;
extern int loopcurrent;
extern int reversepolarity;
extern int fxstxgain;
extern int fxsrxgain;
extern int boostringer;
extern int fastringer;
extern int lowpower;
extern int _opermode;
extern int alawoverride;
extern int fxshonormode;

static int acim2tiss[16] = { 0x0, 0x1, 0x4, 0x5, 0x7, 0x0, 0x0, 0x6, 0x0, 0x0, 0x0, 0x2, 0x0, 0x3 };

/* indirect_resg */
static alpha  indirect_regs[] =
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

static int si321x_powerup_proslic(struct a24xx_dev *wc_dev, int card, int fast)
{
	unsigned char vbat;
	unsigned long origjiffies;
	int lim;

	/* Set period of DC-DC converter to 1/64 khz */
	a24xx_spi_setreg(wc_dev, card, 92, 0xff /* was 0xff */);

	/* Wait for VBat to powerup */
	origjiffies = jiffies;

	/* Disable powerdown */
	a24xx_spi_setreg(wc_dev, card, 14, 0);

	/* If fast, don't bother checking anymore */
	if (fast)
		return 0;

	while((vbat = a24xx_spi_getreg(wc_dev, card, 82)) < 0xc0) {
		/* Wait no more than 500ms */
		if ((jiffies - origjiffies) > HZ/2) {
			break;
		}
	}

	if (vbat < 0xc0) {
		if (wc_dev->proslic_power == PROSLIC_POWER_UNKNOWN) {
			printk("ProSLIC on module %d failed to powerup within %d ms (%d mV only)\n\n -- DID YOU REMEMBER TO PLUG IN THE HD POWER CABLE TO THE A24xxP??\n",
					card, (int)(((jiffies - origjiffies) * 1000 / HZ)),
					vbat * 375);
		}
		wc_dev->proslic_power = PROSLIC_POWER_WARNED;
		return -1;
	} else if (debug) {
		printk("ProSLIC on module %d powered up to -%d volts (%02x) in %d ms\n",
			card, vbat * 376 / 1000, vbat, (int)(((jiffies - origjiffies) * 1000 / HZ)));
	}
	wc_dev->proslic_power = PROSLIC_POWER_ON;

	/* Proslic max allowed loop current, reg 71 LOOP_I_LIMIT */
	/* If out of range, just set it to the default value     */
	lim = (loopcurrent - 20) / 3;
	if ( loopcurrent > 41 ) {
		lim = 0;
		if (debug) {
			printk("Loop current out of range! Setting to default 20mA!\n");
		}
	}
	else if (debug) {
		printk("Loop current set to %dmA!\n",(lim*3)+20);
	}
	a24xx_spi_setreg(wc_dev,card,LOOP_I_LIMIT,lim);

	/* Engage DC-DC converter */
	a24xx_spi_setreg(wc_dev, card, 93, 0x19 /* was 0x19 */);
#if 0
	origjiffies = jiffies;
	while(0x80 & opvx_a24xx_spi_getreg(wc_dev, card, 93)) {
		if ((jiffies - origjiffies) > 2 * HZ) {
			printk("Timeout waiting for DC-DC calibration on module %d\n", card);
			return -1;
		}
	}

#if 0
	/* Wait a full two seconds */
	while((jiffies - origjiffies) < 2 * HZ);

	/* Just check to be sure */
	vbat = opvx_a24xx_spi_getreg(wc_dev, card, 82);
	printk("ProSLIC on module %d powered up to -%d volts (%02x) in %d ms\n",
		       card, vbat * 376 / 1000, vbat, (int)(((jiffies - origjiffies) * 1000 / HZ)));
#endif
#endif
	return 0;

}

/*return not OK modules flag*/
static int si321x_powerup_proslic_all(struct a24xx_dev *wc_dev, int flag, int fast)
{
	unsigned long origjiffies;
	struct stat{
			unsigned char vbat;
			unsigned long jifs;
	};
	
	struct stat stats[24];
	int lim;
	int tmp_flag,x;


	for(x=0; x < wc_dev->max_cards; x++){
			if(flag & (1 << x)){
					/* Set period of DC-DC converter to 1/64 khz */
					a24xx_spi_setreg(wc_dev, x, 92, 0xff /* was 0xff */);

					/* Disable powerdown */
					a24xx_spi_setreg(wc_dev, x, 14, 0);
					
					stats[x].vbat = 0;
					stats[x].jifs = 0;
			}
	}
	
	/* Wait for VBat to powerup */
	origjiffies = jiffies;
	
	/* If fast, don't bother checking anymore */
	if (fast)
		return 0;
	
	tmp_flag = flag;
	while( ((jiffies - origjiffies) <= HZ/2) && tmp_flag){	/* Wait no more than 500ms */
			for(x=0;x<wc_dev->max_cards;x++){
					if(tmp_flag & (1 << x)){
							//stats[x].vbat = a24xx_spi_getreg(wc_dev, x, 82);
							stats[x].vbat = a24xx_spi_getreg(wc_dev, x, 82);
							if(stats[x].vbat >= 0xc0){
									stats[x].jifs = jiffies - origjiffies;
									tmp_flag &=~(1 << x);
							}
					}
			}
	}

	for(x=0; x < wc_dev->max_cards; x++){
			if(flag & (1 << x)){
					if(stats[x].vbat < 0xc0){
							if (wc_dev->proslic_power == PROSLIC_POWER_UNKNOWN){
									printk("ProSLIC on module %d failed to powerup within %d ms (%d mV only)\n\n -- DID YOU REMEMBER TO PLUG IN THE HD POWER CABLE TO THE A24xxP??\n",
											x, (int)(((jiffies - origjiffies) * 1000 / HZ)),
													stats[x].vbat * 375);
							}
							//wc_dev->proslic_power = PROSLIC_POWER_WARNED;
					}
					else if(debug){
							printk("ProSLIC on module %d powered up to -%d volts (%02x) in %d ms\n",
									x, stats[x].vbat * 376 / 1000, stats[x].vbat, (int)((stats[x].jifs * 1000 / HZ)));
					}
			}
	}		
	if(tmp_flag == flag){
			wc_dev->proslic_power = PROSLIC_POWER_WARNED;
			return tmp_flag;
	}
	
	wc_dev->proslic_power = PROSLIC_POWER_ON;

	/* Proslic max allowed loop current, reg 71 LOOP_I_LIMIT */
	/* If out of range, just set it to the default value     */
	lim = (loopcurrent - 20) / 3;
	if ( loopcurrent > 41 ) {
		lim = 0;
		if (debug) {
			printk("Loop current out of range! Setting to default 20mA!\n");
		}
	}
	else if (debug) {
			printk("Loop current set to %dmA!\n",(lim*3)+20);
	}
	flag &= ~tmp_flag;
	
	for(x=0;x < wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					a24xx_spi_setreg(wc_dev,x,LOOP_I_LIMIT,lim);

					/* Engage DC-DC converter */
					a24xx_spi_setreg(wc_dev, x, 93, 0x19 /* was 0x19 */);
			}
	}

	return tmp_flag;
}

static int si321x_proslic_insane(struct a24xx_dev *wc_dev, int card)
{
	int blah,insane_report;
	insane_report=0;

	blah = a24xx_spi_getreg(wc_dev, card, 0);
	if (debug) {
		printk("ProSLIC on module %d, product %d, version %d (0x%x)\n", card, (blah & 0x30) >> 4, (blah & 0xf), blah&0xff);
	}

#if 0
	if ((blah & 0x30) >> 4) {
		printk("ProSLIC on module %d is not a 3210.\n", card);
		return -1;
	}
#endif
	if (((blah & 0xf) == 0) || ((blah & 0xf) == 0xf)) {
		/* SLIC not loaded */
		return -1;
	}
	if ((blah & 0xf) < 2) {
		printk("ProSLIC 3210 version %d is too old\n", blah & 0xf);
		return -1;
	}
	if (a24xx_spi_getreg(wc_dev, card, 1) & 0x80) {
	/* ProSLIC 3215, not a 3210 */
		wc_dev->flags[card] |= FLAG_3215;
	}

	blah = a24xx_spi_getreg(wc_dev, card, 8);
	if (blah != 0x2) {
		printk("ProSLIC on module %d insane (1) %d should be 2\n", card, blah);
		return -1;
	} else if ( insane_report) {
		printk("ProSLIC on module %d Reg 8 Reads %d Expected is 0x2\n",card,blah);
	}

	blah = a24xx_spi_getreg(wc_dev, card, 64);
	if (blah != 0x0) {
		printk("ProSLIC on module %d insane (2)\n", card);
		return -1;
	} else if ( insane_report) {
		printk("ProSLIC on module %d Reg 64 Reads %d Expected is 0x0\n",card,blah);
	}
	
	blah = a24xx_spi_getreg(wc_dev, card, 11);
	if (blah != 0x33) {
		printk("ProSLIC on module %d insane (3)\n", card);
		return -1;
	} else if ( insane_report) {
		printk("ProSLIC on module %d Reg 11 Reads %d Expected is 0x33\n",card,blah);
	}

	/* Just be sure it's setup right. */
	a24xx_spi_setreg(wc_dev, card, 30, 0);

	if (debug) {
		printk("ProSLIC on module %d seems sane.\n", card);
	}

	return 0;
}


#if 1
static int si321x_proslic_calibrate(struct a24xx_dev *wc_dev, int card)
{
	unsigned long origjiffies;
	int x;
	/* Perform all calibrations */
	a24xx_spi_setreg(wc_dev, card, 97, 0x1f);

	/* Begin, no speedup */
	a24xx_spi_setreg(wc_dev, card, 96, 0x5f);

	/* Wait for it to finish */
	origjiffies = jiffies;
	while(a24xx_spi_getreg(wc_dev, card, 96)) {
		if ((jiffies - origjiffies) > 2 * HZ) {
			printk("Timeout waiting for calibration of module %d\n", card);
			return -1;
		}
	}

	if (debug) {
		/* Print calibration parameters */
		printk("Calibration Vector Regs 98 - 107: \n");
		for (x=98;x<108;x++) {
			printk("%d: %02x\n", x, a24xx_spi_getreg(wc_dev, card, x));
		}
	}
	return 0;
}
#endif

/*return not OK cards flag*/
static int si321x_proslic_calibrate_all(struct a24xx_dev *wc_dev, int flag)
{
	unsigned long origjiffies;
	int i,x,tmp_flag;
	
	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					/* Perform all calibrations */
					a24xx_spi_setreg(wc_dev, x, 97, 0x1f);

					/* Begin, no speedup */
					a24xx_spi_setreg(wc_dev, x, 96, 0x5f);
			}
	}

	/* Wait for it to finish */
	origjiffies = jiffies;
	tmp_flag = flag;
	
	while(((jiffies - origjiffies) < 2 * HZ) && tmp_flag){
			for(x=0;x<wc_dev->max_cards;x++){
					if(tmp_flag & (1 << x)){
							if(!a24xx_spi_getreg(wc_dev, x, 96)){
									tmp_flag &= ~(1 << x);
							}
					}
			}
	}
	
	if (debug) {
		/* Print calibration parameters */
		for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					printk("Calibration Vector Regs 98 - 107: \n");
					for (i=98;i<108;i++) {
							printk("Module %d %d: %02x\n", x,i, a24xx_spi_getreg(wc_dev, x, i));
					}
			}
		}
	}
	return tmp_flag;
}

static int si321x_proslic_powerleak_test(struct a24xx_dev *wc_dev, int card)
{
	unsigned long origjiffies;
	unsigned char vbat;

	/* Turn off linefeed */
	a24xx_spi_setreg(wc_dev, card, 64, 0);

	/* Power down */
	a24xx_spi_setreg(wc_dev, card, 14, 0x10);

	/* Wait for one second */
	origjiffies = jiffies;

	while((vbat = a24xx_spi_getreg(wc_dev, card, 82)) > 0x6) {
		if ((jiffies - origjiffies) >= (HZ/2)) {
			break;
		}
	}

	if (vbat < 0x06) {
		printk("Excessive leakage detected on module %d: %d volts (%02x) after %d ms\n", card,
			376 * vbat / 1000, vbat, (int)((jiffies - origjiffies) * 1000 / HZ));
		return -1;
	} else if (debug) {
		printk("Post-leakage voltage: %d volts\n", 376 * vbat / 1000);
	}
	return 0;
}

/* return OK cards flag*/
static int si321x_proslic_powerleak_test_all(struct a24xx_dev *wc_dev, int flag)
{
	unsigned long origjiffies;
	struct stat{
			unsigned char vbat;
			unsigned long jifs;
	};
	
	struct stat stats[24];
	int x,tmp_flag=0;

	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					/* Turn off linefeed */
					a24xx_spi_setreg(wc_dev, x, 64, 0);

					/* Power down */
					a24xx_spi_setreg(wc_dev, x, 14, 0x10);
			}
			stats[x].vbat=0;
			stats[x].jifs=0;
	}

	/* Wait for one second */
	origjiffies = jiffies;

	tmp_flag = flag;
	while(((jiffies - origjiffies) < (HZ/2)) && tmp_flag){
			for(x=0;x<wc_dev->max_cards;x++){
					if(tmp_flag & (1 << x)){
							if((stats[x].vbat = a24xx_spi_getreg(wc_dev, x, 82)) < 0x6){
									tmp_flag &= ~(1 << x);
									stats[x].jifs = jiffies - origjiffies;
							}
					}
			}
	}
	
	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					if (stats[x].vbat < 0x06) {
							printk("Excessive leakage detected on module %d: %d volts (%02x) after %d ms\n", x,
								 376 * stats[x].vbat / 1000, stats[x].vbat, (int)(stats[x].jifs * 1000 / HZ));
					} else if (debug) {
							printk("Post-leakage voltage: %d volts\n", 376 * stats[x].vbat / 1000);
					}
			}
	}
	
	return tmp_flag;
}

static int si321x_proslic_manual_calibrate(struct a24xx_dev *wc_dev, int card)
{
	unsigned long origjiffies;
	unsigned char i;

	a24xx_spi_setreg(wc_dev, card, 21, 0);//(0)  Disable all interupts in DR21
	a24xx_spi_setreg(wc_dev, card, 22, 0);//(0)Disable all interupts in DR21
	a24xx_spi_setreg(wc_dev, card, 23, 0);//(0)Disable all interupts in DR21
	a24xx_spi_setreg(wc_dev, card, 64, 0);//(0)

	a24xx_spi_setreg(wc_dev, card, 97, 0x18); //(0x18)Calibrations without the ADC and DAC offset and without common mode calibration.
	a24xx_spi_setreg(wc_dev, card, 96, 0x47); //(0x47)	Calibrate common mode and differential DAC mode DAC + ILIM

	origjiffies=jiffies;
	while( a24xx_spi_getreg(wc_dev,card,96)!=0 ){
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
	si321x_proslic_setreg_indirect(wc_dev, card, 88,0);
	si321x_proslic_setreg_indirect(wc_dev,card,89,0);
	si321x_proslic_setreg_indirect(wc_dev,card,90,0);
	si321x_proslic_setreg_indirect(wc_dev,card,91,0);
	si321x_proslic_setreg_indirect(wc_dev,card,92,0);
	si321x_proslic_setreg_indirect(wc_dev,card,93,0);

	a24xx_spi_setreg(wc_dev, card, 98,0x10); // This is necessary if the calibration occurs other than at reset time
	a24xx_spi_setreg(wc_dev, card, 99,0x10);

	for ( i=0x1f; i>0; i--) {
		a24xx_spi_setreg(wc_dev, card, 98,i);
		origjiffies=jiffies;
		while((jiffies-origjiffies)<4);
		if((a24xx_spi_getreg(wc_dev,card,88)) == 0)
			break;
	} // for

	for ( i=0x1f; i>0; i--) {
		a24xx_spi_setreg(wc_dev, card, 99,i);
		origjiffies=jiffies;
		while((jiffies-origjiffies)<4);
		if((a24xx_spi_getreg(wc_dev,card,89)) == 0)
			break;
	}//for

	/*******************************The preceding is the manual gain mismatch calibration****************************/
	/**********************************The following is the longitudinal Balance Cal***********************************/
	a24xx_spi_setreg(wc_dev,card,64,1);
	while((jiffies-origjiffies)<10); // Sleep 100?

	a24xx_spi_setreg(wc_dev, card, 64, 0);
	a24xx_spi_setreg(wc_dev, card, 23, 0x4);  // enable interrupt for the balance Cal
	a24xx_spi_setreg(wc_dev, card, 97, 0x1); // this is a singular calibration bit for longitudinal calibration
	a24xx_spi_setreg(wc_dev, card, 96,0x40);

	a24xx_spi_getreg(wc_dev,card,96); /* Read Reg 96 just cause */

	a24xx_spi_setreg(wc_dev, card, 21, 0xFF);
	a24xx_spi_setreg(wc_dev, card, 22, 0xFF);
	a24xx_spi_setreg(wc_dev, card, 23, 0xFF);

	/**The preceding is the longitudinal Balance Cal***/
	return(0);

}

/*return not OK cards flag*/
static int si321x_proslic_manual_calibrate_all(struct a24xx_dev *wc_dev, int flag)
{
	unsigned long origjiffies;
	unsigned char i;
	int x,tmp_flag;

	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					a24xx_spi_setreg(wc_dev, x, 21, 0);//(0)  Disable all interupts in DR21
					a24xx_spi_setreg(wc_dev, x, 22, 0);//(0)Disable all interupts in DR21
					a24xx_spi_setreg(wc_dev, x, 23, 0);//(0)Disable all interupts in DR21
					a24xx_spi_setreg(wc_dev, x, 64, 0);//(0)

					a24xx_spi_setreg(wc_dev, x, 97, 0x18); //(0x18)Calibrations without the ADC and DAC offset and without common mode calibration.
					a24xx_spi_setreg(wc_dev, x, 96, 0x47); //(0x47)	Calibrate common mode and differential DAC mode DAC + ILIM
			}
	}

	origjiffies=jiffies;
	
	/*remove not OK cards flag*/
	tmp_flag = flag;
	while(((jiffies-origjiffies)< 80) && tmp_flag){
			for(x=0;x<wc_dev->max_cards;x++){
					if(tmp_flag & (1 << x)){
							if(a24xx_spi_getreg(wc_dev,x,96) == 0){
									tmp_flag &= ~(1 << x);
							}
					}
			}
	}
	
	flag &= ~tmp_flag;
	
	//Initialized DR 98 and 99 to get consistant results.
	// 98 and 99 are the results registers and the search should have same intial conditions.

	/*******************************The following is the manual gain mismatch calibration****************************/
	/*******************************This is also available as a function *******************************************/
	// Delay 10ms
	origjiffies=jiffies;
	while((jiffies-origjiffies)<1);
	
	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					si321x_proslic_setreg_indirect(wc_dev,x,88,0);
					si321x_proslic_setreg_indirect(wc_dev,x,89,0);
					si321x_proslic_setreg_indirect(wc_dev,x,90,0);
					si321x_proslic_setreg_indirect(wc_dev,x,91,0);
					si321x_proslic_setreg_indirect(wc_dev,x,92,0);
					si321x_proslic_setreg_indirect(wc_dev,x,93,0);

					a24xx_spi_setreg(wc_dev, x, 98,0x10); // This is necessary if the calibration occurs other than at reset time
					a24xx_spi_setreg(wc_dev, x, 99,0x10);

					for ( i=0x1f; i>0; i--) {
							a24xx_spi_setreg(wc_dev, x, 98,i);
							origjiffies=jiffies;
							while((jiffies-origjiffies)<4);
							if((a24xx_spi_getreg(wc_dev,x,88)) == 0)
									break;
					} // for

					for ( i=0x1f; i>0; i--) {
							a24xx_spi_setreg(wc_dev, x, 99,i);
							origjiffies=jiffies;
							while((jiffies-origjiffies)<4);
							if((a24xx_spi_getreg(wc_dev,x,89)) == 0)
									break;
					}//for

					/*******************************The preceding is the manual gain mismatch calibration****************************/
					/**********************************The following is the longitudinal Balance Cal***********************************/
					a24xx_spi_setreg(wc_dev,x,64,1);
			}
	}
	
	while((jiffies-origjiffies)<10); // Sleep 100?

	for(x=0;x<wc_dev->max_cards;x++){
			if(flag & (1 << x)){
					a24xx_spi_setreg(wc_dev, x, 64, 0);
					a24xx_spi_setreg(wc_dev, x, 23, 0x4);  // enable interrupt for the balance Cal
					a24xx_spi_setreg(wc_dev, x, 97, 0x1); // this is a singular calibration bit for longitudinal calibration
					a24xx_spi_setreg(wc_dev, x, 96,0x40);

					a24xx_spi_getreg(wc_dev,x,96); /* Read Reg 96 just cause */

					a24xx_spi_setreg(wc_dev, x, 21, 0xFF);
					a24xx_spi_setreg(wc_dev, x, 22, 0xFF);
					a24xx_spi_setreg(wc_dev, x, 23, 0xFF);
			}
	}

	/**The preceding is the longitudinal Balance Cal***/
	return tmp_flag;
}

static int si321x_proslic_verify_indirect_regs(struct a24xx_dev *wc_dev, int card)
{
	int passed = 1;
	unsigned short i, initial;
	int j;

	for (i=0; i<sizeof(indirect_regs) / sizeof(indirect_regs[0]); i++) {
		if((j = si321x_proslic_getreg_indirect(wc_dev, card, (unsigned char) indirect_regs[i].address)) < 0) {
			printk("Failed to read indirect register %d\n", i);
			return -1;
		}
		initial= indirect_regs[i].initial;

		if ( j != initial && (!(wc_dev->flags[card] & FLAG_3215) || (indirect_regs[i].altaddr != 255))) {
			 printk("!!!!!!! %s  iREG %X = %X  should be %X\n",
				indirect_regs[i].name,indirect_regs[i].address,j,initial );
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

static int si321x_proslic_init_indirect_regs(struct a24xx_dev *wc_dev, int card)
{
	unsigned char i;

	for (i=0; i<sizeof(indirect_regs) / sizeof(indirect_regs[0]); i++) {
		if(si321x_proslic_setreg_indirect(wc_dev, card, indirect_regs[i].address,indirect_regs[i].initial)) {
			return -1;
		}
	}

	return 0;
}

int si321x_set_ring_generator_mode(struct a24xx_dev *wc_dev, int card, int mode)
{
	int reg20, reg21, reg74; /* RCO, RNGX, VBATH */
	struct fxs *const fxs = &wc_dev->mod[card].fxs;

	fxs->neonringing = mode;	/* track ring generator mode */

	if (mode) { /* Neon */
		if (debug)
			printk(KERN_DEBUG "NEON ring on chan %d, "
			"lasttxhook was 0x%x\n", card, fxs->lasttxhook);
		/* Must be in FORWARD ACTIVE before setting ringer */
		fxs->lasttxhook = SLIC_LF_ACTIVE_FWD;
		a24xx_spi_setreg(wc_dev, card, LINE_STATE, fxs->lasttxhook);

		si321x_proslic_setreg_indirect(wc_dev, card, 22,
					       NEON_MWI_RNGY_PULSEWIDTH);
		si321x_proslic_setreg_indirect(wc_dev, card, 21,
					       0x7bef);	/* RNGX (91.5Vpk) */
		si321x_proslic_setreg_indirect(wc_dev, card, 20,
					       0x009f);	/* RCO (RNGX, t rise)*/

		a24xx_spi_setreg(wc_dev, card, 34, 0x19); /* Ringing Osc. Control */
		a24xx_spi_setreg(wc_dev, card, 74, 0x3f); /* VBATH 94.5V */
		si321x_proslic_setreg_indirect(wc_dev, card, 29, 0x4600); /* RPTP */
		/* A write of 0x04 to register 64 will turn on the VM led */
	} else {
		a24xx_spi_setreg(wc_dev, card, 34, 0x00); /* Ringing Osc. Control */
		/* RNGY Initial Phase */
		si321x_proslic_setreg_indirect(wc_dev, card, 22, 0x0000);
		si321x_proslic_setreg_indirect(wc_dev, card, 29, 0x3600); /* RPTP */
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
						card);
			} else if (lowpower) {
				reg21 = 0x014b;	/* RNGX */
				if (debug)
					printk(KERN_DEBUG "Reducing fast ring "
					    "power on chan %d (50V peak)\n",
					    card);
			} else if (fxshonormode &&
						fxo_modes[_opermode].ring_x) {
				reg21 = fxo_modes[_opermode].ring_x;
				if (debug)
					printk(KERN_DEBUG "fxshonormode: fast "
						"ring_x power on chan %d\n",
						card);
			} else {
				reg21 = 0x01b9;
				if (debug)
					printk(KERN_DEBUG "Speeding up ringer "
						"on chan %d (25Hz)\n",
						card);
			}
			/* VBATH */
			a24xx_spi_setreg(wc_dev, card, 74, reg74);
			/*RCO*/
			si321x_proslic_setreg_indirect(wc_dev, card, 20, reg20);
			/*RNGX*/
			si321x_proslic_setreg_indirect(wc_dev, card, 21, reg21);

		} else {
			/* Ringer Speed */
			if (fxshonormode && fxo_modes[_opermode].ring_osc) {
				reg20 = fxo_modes[_opermode].ring_osc;
				if (debug)
					printk(KERN_DEBUG "fxshonormode: "
						"ring_osc speed on chan %d\n",
						card);
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
						card);
			} else if (lowpower) {
				reg21 = 0x108;
				if (debug)
					printk(KERN_DEBUG "Reducing ring power "
						"on chan %d (50V peak)\n",
						card);
			} else if (fxshonormode &&
						fxo_modes[_opermode].ring_x) {
				reg21 = fxo_modes[_opermode].ring_x;
				if (debug)
					printk(KERN_DEBUG "fxshonormode: ring_x"
						" power on chan %d\n",
						card);
			} else {
				reg21 = 0x160;
				if (debug)
					printk(KERN_DEBUG "Normal ring power on"
						" chan %d\n",
						card);
			}
			/* VBATH */
			a24xx_spi_setreg(wc_dev, card, 74, reg74);
			/* RCO */
			si321x_proslic_setreg_indirect(wc_dev, card, 20, reg20);
			  /* RNGX */
			si321x_proslic_setreg_indirect(wc_dev, card, 21, reg21);
		}
	}
	return 0;
}

int si321x_init_ring_generator_mode(struct a24xx_dev *wc_dev, int card){
		a24xx_spi_setreg(wc_dev, card, 34, 0x00);	/* Ringing Osc. Control */
		/* neon trapezoid timers */
		a24xx_spi_setreg(wc_dev, card, 48, 0xe0);	/* Active Timer low byte */
		a24xx_spi_setreg(wc_dev, card, 49, 0x01);	/* Active Timer high byte */
		a24xx_spi_setreg(wc_dev, card, 50, 0xF0);	/* Inactive Timer low byte */
		a24xx_spi_setreg(wc_dev, card, 51, 0x05);	/* Inactive Timer high byte */

		si321x_set_ring_generator_mode(wc_dev, card, 0);

		return 0;
}

int si321x_init_proslic(struct a24xx_dev *wc_dev, int card, int fast, int manual, int sane)
{
	unsigned short tmp[5];
	unsigned char r19, r9;
	int x;
	int fxsmode=0;

	/* Sanity check the ProSLIC */
	if (!sane && si321x_proslic_insane(wc_dev, card)) {
		return -2;
	}
	
	/* default messages to none and method to FSK */
	memset(&wc_dev->mod[card].fxs.vmwisetting, 0, sizeof(wc_dev->mod[card].fxs.vmwisetting));
	wc_dev->mod[card].fxs.vmwi_lrev = 0;
	wc_dev->mod[card].fxs.vmwi_hvdc = 0;
	wc_dev->mod[card].fxs.vmwi_hvac = 0;
	
	/* By default, don't send on hook */
	if (reversepolarity) {
		wc_dev->mod[card].fxs.idletxhookstate = 5;
	} else {
		wc_dev->mod[card].fxs.idletxhookstate = 1;
	}

	if (sane) {
		/* Make sure we turn off the DC->DC converter to prevent anything from blowing up */
		a24xx_spi_setreg(wc_dev, card, 14, 0x10);
	}

	if (si321x_proslic_init_indirect_regs(wc_dev, card)) {
		printk(KERN_INFO "Indirect Registers failed to initialize on module %d.\n", card);
		return -1;
	}

	/* Clear scratch pad area */
	si321x_proslic_setreg_indirect(wc_dev, card, 97,0);

	/* Clear digital loopback */
	a24xx_spi_setreg(wc_dev, card, 8, 0);

	/* Revision C optimization */
	a24xx_spi_setreg(wc_dev, card, 108, 0xeb);

	/* Disable automatic VBat switching for safety to prevent
	   Q7 from accidently turning on and burning out. */
	a24xx_spi_setreg(wc_dev, card, 67, 0x07);  /* Note, if pulse dialing has problems at high REN loads
					      change this to 0x17 */

	/* Turn off Q7 */
	a24xx_spi_setreg(wc_dev, card, 66, 1);

	/* Flush ProSLIC digital filters by setting to clear, while
	   saving old values */
	for (x=0;x<5;x++) {
		tmp[x] = si321x_proslic_getreg_indirect(wc_dev, card, x + 35);
		si321x_proslic_setreg_indirect(wc_dev, card, x + 35, 0x8000);
	}

	/* Power up the DC-DC converter */
	if (si321x_powerup_proslic(wc_dev, card, fast)) {	////////////****************
		printk("Unable to do INITIAL ProSLIC powerup on module %d\n", card);
		return -1;
	}

	if (!fast) {
		/* Check for power leaks */
		if (si321x_proslic_powerleak_test(wc_dev, card)) {///////****************
			printk("ProSLIC module %d failed leakage test.  Check for short circuit\n", card);
		}
		/* Power up again */
		if (si321x_powerup_proslic(wc_dev, card, fast)) {	////////////****************
			printk("Unable to do FINAL ProSLIC powerup on module %d\n", card);
			return -1;
		}
#ifndef NO_CALIBRATION
		/* Perform calibration */
		if(manual) {
			if (si321x_proslic_manual_calibrate(wc_dev, card)) {	//////////****************
				//printk("Proslic failed on Manual Calibration\n");
				if (si321x_proslic_manual_calibrate(wc_dev, card)) {	////////////****************
					printk("Proslic Failed on Second Attempt to Calibrate Manually. (Try -DNO_CALIBRATION in Makefile)\n");
					return -1;
				}
				printk("Proslic Passed Manual Calibration on Second Attempt\n");
			}
		}
		else {
			if(si321x_proslic_calibrate(wc_dev, card))  {	///////****************
				//printk("ProSlic died on Auto Calibration.\n");
				if (si321x_proslic_calibrate(wc_dev, card)) {	////////////****************
					printk("Proslic Failed on Second Attempt to Auto Calibrate\n");
					return -1;
				}
				printk("Proslic Passed Auto Calibration on Second Attempt\n");
			}
		}
		/* Perform DC-DC calibration */
		a24xx_spi_setreg(wc_dev, card, 93, 0x99);
		r19 = a24xx_spi_getreg(wc_dev, card, 107);
		if ((r19 < 0x2) || (r19 > 0xd)) {
			printk("DC-DC cal has a surprising direct 107 of 0x%02x!\n", r19);
			a24xx_spi_setreg(wc_dev, card, 107, 0x8);
		}

		/* Save calibration vectors */
		for (x=0;x<NUM_CAL_REGS;x++) {
			wc_dev->mod[card].fxs.calregs.vals[x] = a24xx_spi_getreg(wc_dev, card, 96 + x);
		}
#endif

	} else {
		/* Restore calibration registers */
		for (x=0;x<NUM_CAL_REGS;x++) {
			a24xx_spi_setreg(wc_dev, card, 96 + x, wc_dev->mod[card].fxs.calregs.vals[x]);
		}
	}
	/* Calibration complete, restore original values */
	for (x=0;x<5;x++) {
		si321x_proslic_setreg_indirect(wc_dev, card, x + 35, tmp[x]);
	}

	if (si321x_proslic_verify_indirect_regs(wc_dev, card)) {
		printk(KERN_INFO "Indirect Registers failed verification.\n");
		return -1;
	}


#if 0
	/* Disable Auto Power Alarm Detect and other "features" */
	a24xx_spi_setreg(wc_dev, card, 67, 0x0e);
	blah = opvx_a24xx_spi_getreg(wc_dev, card, 67);
#endif

#if 0
	if (si321x_proslic_setreg_indirect(wc_dev, card, 97, 0x0)) { // Stanley: for the bad recording fix
		printk(KERN_INFO "ProSlic IndirectReg Died.\n");
		return -1;
	}
#endif

	if (alawoverride) {
		a24xx_spi_setreg(wc_dev, card, 1, 0x20);
	} else {
		a24xx_spi_setreg(wc_dev, card, 1, 0x28);
	}
	// U-Law 8-bit interface
	a24xx_spi_setreg(wc_dev, card, 2, (card%4) * 8 + (card/4) * 32);    // Tx Start count low byte  0
	a24xx_spi_setreg(wc_dev, card, 3, 0);    // Tx Start count high byte 0
	a24xx_spi_setreg(wc_dev, card, 4, (card%4) * 8 + (card/4) * 32);    // Rx Start count low byte  0
	a24xx_spi_setreg(wc_dev, card, 5, 0);    // Rx Start count high byte 0
	a24xx_spi_setreg(wc_dev, card, 18, 0xff);     // clear all interrupt
	a24xx_spi_setreg(wc_dev, card, 19, 0xff);
	a24xx_spi_setreg(wc_dev, card, 20, 0xff);
	a24xx_spi_setreg(wc_dev, card, 73, 0x04);
	if (fxshonormode) {
		fxsmode = acim2tiss[fxo_modes[_opermode].acim];
		a24xx_spi_setreg(wc_dev, card, 10, 0x08 | fxsmode);
		if (fxo_modes[_opermode].ring_osc) {
			si321x_proslic_setreg_indirect(wc_dev, card, 20, fxo_modes[_opermode].ring_osc);
		}
		if (fxo_modes[_opermode].ring_x) {
			si321x_proslic_setreg_indirect(wc_dev, card, 21, fxo_modes[_opermode].ring_x);
		}
	}
	if (lowpower) {
		a24xx_spi_setreg(wc_dev, card, 72, 0x10);
	}

#if 0
	a24xx_spi_setreg(wc_dev, card, 21, 0x00); 	// enable interrupt
	a24xx_spi_setreg(wc_dev, card, 22, 0x02); 	// Loop detection interrupt
	a24xx_spi_setreg(wc_dev, card, 23, 0x01); 	// DTMF detection interrupt
#endif

#if 0
	/* Enable loopback */
	a24xx_spi_setreg(wc_dev, card, 8, 0x2);
	a24xx_spi_setreg(wc_dev, card, 14, 0x0);
	a24xx_spi_setreg(wc_dev, card, 64, 0x0);
	a24xx_spi_setreg(wc_dev, card, 1, 0x08);
#endif

	if (fastringer) {
		/* Speed up Ringer */
		si321x_proslic_setreg_indirect(wc_dev, card, 20, 0x7e6d);
		si321x_proslic_setreg_indirect(wc_dev, card, 21, 0x01b9);
		/* Beef up Ringing voltage to 89V */
		if (boostringer) {
			a24xx_spi_setreg(wc_dev, card, 74, 0x3f);
			if (si321x_proslic_setreg_indirect(wc_dev, card, 21, 0x247)) {
				return -1;
			}
			printk("Boosting fast ringer on slot %d (89V peak)\n", card + 1);
		} else if (lowpower) {
			if (si321x_proslic_setreg_indirect(wc_dev, card, 21, 0x14b)) {
				return -1;
			}
			printk("Reducing fast ring power on slot %d (50V peak)\n", card + 1);
		} else {
			printk("Speeding up ringer on slot %d (25Hz)\n", card + 1);
		}
	} else {
		/* Beef up Ringing voltage to 89V */
		if (boostringer) {
			a24xx_spi_setreg(wc_dev, card, 74, 0x3f);
			if (si321x_proslic_setreg_indirect(wc_dev, card, 21, 0x1d1)) {
				return -1;
			}
			printk("Boosting ringer on slot %d (89V peak)\n", card + 1);
		} else if (lowpower) {
			if (si321x_proslic_setreg_indirect(wc_dev, card, 21, 0x108)) {
				return -1;
			}
			printk("Reducing ring power on slot %d (50V peak)\n", card + 1);
		}
	}
	if (si321x_init_ring_generator_mode(wc_dev, card)) {
		return -1;
	}
	if(fxstxgain || fxsrxgain) {
		r9 = a24xx_spi_getreg(wc_dev, card, 9);
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
		a24xx_spi_setreg(wc_dev,card,9,r9);
	}

	if(debug) {
		printk("DEBUG: fxstxgain:%s fxsrxgain:%s\n",
			((a24xx_spi_getreg(wc_dev, card, 9)/8) == 1) ? "3.5":
			     (((a24xx_spi_getreg(wc_dev, card, 9)/4) == 1) ? "-3.5":"0.0"),
                  ((a24xx_spi_getreg(wc_dev, card, 9)/2) == 1) ?"3.5":((a24xx_spi_getreg(wc_dev,card,9)%2) ? "-3.5":"0.0")
		);
	}

	a24xx_spi_setreg(wc_dev, card, 64, 0x01);
	
	return 0;
}

/*** 
*return
 ret_flag :	not OK cards flag
 blk_flag :	not installed cards flag
***/
int si321x_init_proslic_all(struct a24xx_dev *wc_dev, int fxs_flag,int fast, int manual, int sane,int *blk_flag)
{
	int flag=fxs_flag,tmp_flag=0,ret_flag=0;
	unsigned short tmp[24][5];
	unsigned char r19, r9;
	int x,i;
	int fxsmode=0;

	
	for(i=0;i<wc_dev->max_cards;i++)
	{
			if(flag & (1 << i)){
					if((i%4) == 0){
							__a24xx_setcard(wc_dev, i);
							__opvx_a24xx_write_8bits(wc_dev->mem32, 0x00);
							__opvx_a24xx_write_8bits(wc_dev->mem32, 0x80);
					}
					
					/* Sanity check the ProSLIC */
					if (!sane && si321x_proslic_insane(wc_dev, i)) {
							tmp_flag |= 1<<i;
							continue;
					}
					
					/* default messages to none and method to FSK */
					memset(&wc_dev->mod[i].fxs.vmwisetting, 0, sizeof(wc_dev->mod[i].fxs.vmwisetting));
					wc_dev->mod[i].fxs.vmwi_lrev = 0;
					wc_dev->mod[i].fxs.vmwi_hvdc = 0;
					wc_dev->mod[i].fxs.vmwi_hvac = 0;
					
					/* By default, don't send on hook */
					if (reversepolarity) {
							wc_dev->mod[i].fxs.idletxhookstate = 5;
					} else {
							wc_dev->mod[i].fxs.idletxhookstate = 1;
					}

					if (sane) {
							/* Make sure we turn off the DC->DC converter to prevent anything from blowing up */
							a24xx_spi_setreg(wc_dev, i, 14, 0x10);
					}

					if (si321x_proslic_init_indirect_regs(wc_dev, i)) {
							printk(KERN_INFO "Indirect Registers failed to initialize on module %d.\n", i);
							tmp_flag |= 1<<i;
							continue;
					}

					/* Clear scratch pad area */
					si321x_proslic_setreg_indirect(wc_dev, i, 97,0);

					/* Clear digital loopback */
					a24xx_spi_setreg(wc_dev, i, 8, 0);

					/* Revision C optimization */
					a24xx_spi_setreg(wc_dev, i, 108, 0xeb);

					/* Disable automatic VBat switching for safety to prevent
	   					Q7 from accidently turning on and burning out. */
					a24xx_spi_setreg(wc_dev, i, 67, 0x07);  /* Note, if pulse dialing has problems at high REN loads
					      change this to 0x17 */

					/* Turn off Q7 */
					a24xx_spi_setreg(wc_dev, i, 66, 1);

					/* Flush ProSLIC digital filters by setting to clear, while
	   				saving old values */
					for (x=0;x<5;x++) {
							tmp[i][x] = si321x_proslic_getreg_indirect(wc_dev, i, x + 35);
							si321x_proslic_setreg_indirect(wc_dev, i, x + 35, 0x8000);
					}
			}
	}

	/*remove not installed cards*/
	flag &= ~(tmp_flag);
	if(blk_flag)
			*blk_flag = tmp_flag;
	
	
	touch_softlockup_watchdog();
	
	/* Power up the DC-DC converter */
	tmp_flag = si321x_powerup_proslic_all(wc_dev, flag, fast);
	for(i=0;i<wc_dev->max_cards;i++){
			if(tmp_flag & (1 << i))
					printk("Unable to do INITIAL ProSLIC powerup on module %d\n",i);	
	}
	
	/*remove not OK cards flag*/
	flag &= ~(tmp_flag);
	ret_flag |= tmp_flag;
	
	
	if (!fast){
			/* Check for power leaks */
			tmp_flag=si321x_proslic_powerleak_test_all(wc_dev,flag);
			for(i=0;i<wc_dev->max_cards;i++){
					if( (flag & (1 << i)) && !(tmp_flag & (1 << i)) ){
							printk("ProSLIC module %d failed leakage test.  Check for short circuit\n", i);
					}
			}
			
			/* Power up again */
			tmp_flag = si321x_powerup_proslic_all(wc_dev, flag, fast);
			for(i=0;i<wc_dev->max_cards;i++){
					if(tmp_flag & (1 << i))
							printk("Unable to do FINAL ProSLIC powerup on module %d\n",i);	
			}
			
			/*remove not OK cards flag*/
			flag &= ~(tmp_flag);
			ret_flag |= tmp_flag;
			
#ifndef NO_CALIBRATION
			if(manual) {
					tmp_flag = si321x_proslic_manual_calibrate_all(wc_dev,flag);
					if(tmp_flag){
							tmp_flag = si321x_proslic_manual_calibrate_all(wc_dev,tmp_flag);
							for(i=0;i<wc_dev->max_cards;i++){
									if(flag & (1 << i)){
											if(tmp_flag & (1 << i))
													printk("Proslic Failed on Second Attempt to Calibrate Manually module %d . (Try -DNO_CALIBRATION in Makefile)\n",i);
											else
													printk("Proslic Passed Manual Calibration on Second Attempt on module %d \n",i);
									}
							}
					}
			}else{
					tmp_flag = si321x_proslic_calibrate_all(wc_dev,flag);
					if(tmp_flag){
							tmp_flag = si321x_proslic_calibrate_all(wc_dev,tmp_flag);
							for(i=0;i<wc_dev->max_cards;i++){
									if(flag & (1 << i)){
											if(tmp_flag & (1 << i))
													printk("Proslic Failed on Second Attempt to Auto Calibrate module %d . (Try -DNO_CALIBRATION in Makefile)\n",i);
											else
													printk("Proslic Passed Auto Calibration on Second Attempt on module %d \n",i);
									}
							}
					}
			}
			
			/*remove not OK cards flag*/
			flag &= ~(tmp_flag);
			ret_flag |= tmp_flag;
			
			for(i=0;i<wc_dev->max_cards;i++){
					if(flag & (1 << i)){
							/* Perform DC-DC calibration */
							a24xx_spi_setreg(wc_dev, i, 93, 0x99);
							r19 = a24xx_spi_getreg(wc_dev, i, 107);
							if ((r19 < 0x2) || (r19 > 0xd)) {
									printk("DC-DC cal has a surprising direct 107 of 0x%02x!\n", r19);
									a24xx_spi_setreg(wc_dev, i, 107, 0x8);
							}
							/* Save calibration vectors */
							for (x=0;x<NUM_CAL_REGS;x++) {
									wc_dev->mod[i].fxs.calregs.vals[x] = a24xx_spi_getreg(wc_dev, i, 96 + x);
							}
					}
			}
#endif		
	}else{
			for(i=0;i<wc_dev->max_cards;i++){
					if(flag & (1 << i)){
							/* Restore calibration registers */
							for (x=0;x<NUM_CAL_REGS;x++) {
									a24xx_spi_setreg(wc_dev, i, 96 + x, wc_dev->mod[i].fxs.calregs.vals[x]);
							}
					}
			}
	}
	
	tmp_flag = 0;
	for(i=0;i<wc_dev->max_cards;i++){
		if(flag & (1 << i)){
			/* Calibration complete, restore original values */
			for (x=0;x<5;x++) {
					si321x_proslic_setreg_indirect(wc_dev, i, x + 35, tmp[i][x]);
			}

			if (si321x_proslic_verify_indirect_regs(wc_dev, i)) {
					printk(KERN_INFO "Indirect Registers failed verification on module %d.\n",i);
					tmp_flag |=(1<<i);
					continue;
			}


			if (alawoverride) {
					a24xx_spi_setreg(wc_dev, i, 1, 0x20);
			} else {
					a24xx_spi_setreg(wc_dev, i, 1, 0x28);
			}
			// U-Law 8-bit interface
			a24xx_spi_setreg(wc_dev, i, 2, (i%4) * 8 + (i/4) * 32);    // Tx Start count low byte  0
			a24xx_spi_setreg(wc_dev, i, 3, 0);    // Tx Start count high byte 0
			a24xx_spi_setreg(wc_dev, i, 4, (i%4) * 8 + (i/4) * 32);    // Rx Start count low byte  0
			a24xx_spi_setreg(wc_dev, i, 5, 0);    // Rx Start count high byte 0
			a24xx_spi_setreg(wc_dev, i, 18, 0xff);     // clear all interrupt
			a24xx_spi_setreg(wc_dev, i, 19, 0xff);
			a24xx_spi_setreg(wc_dev, i, 20, 0xff);
			a24xx_spi_setreg(wc_dev, i, 73, 0x04);
			if (fxshonormode) {
					fxsmode = acim2tiss[fxo_modes[_opermode].acim];
					a24xx_spi_setreg(wc_dev, i, 10, 0x08 | fxsmode);
					if (fxo_modes[_opermode].ring_osc) {
							si321x_proslic_setreg_indirect(wc_dev, i, 20, fxo_modes[_opermode].ring_osc);
					}
					if (fxo_modes[_opermode].ring_x) {
							si321x_proslic_setreg_indirect(wc_dev, i, 21, fxo_modes[_opermode].ring_x);
					}
			}
			if (lowpower) {
					a24xx_spi_setreg(wc_dev, i, 72, 0x10);
			}

			if (fastringer) {
					/* Speed up Ringer */
					si321x_proslic_setreg_indirect(wc_dev, i, 20, 0x7e6d);
					si321x_proslic_setreg_indirect(wc_dev, i, 21, 0x01b9);
					/* Beef up Ringing voltage to 89V */
					if (boostringer) {
							a24xx_spi_setreg(wc_dev, i, 74, 0x3f);
							if (si321x_proslic_setreg_indirect(wc_dev, i, 21, 0x247)) {
										tmp_flag |=(1<<i);
										continue;
							}
							printk("Boosting fast ringer on slot %d (89V peak)\n", i + 1);
					} else if (lowpower) {
							if (si321x_proslic_setreg_indirect(wc_dev, i, 21, 0x14b)) {
									tmp_flag |=(1<<i);
									continue;
							}
							printk("Reducing fast ring power on slot %d (50V peak)\n", i + 1);
					} else {
							printk("Speeding up ringer on slot %d (25Hz)\n", i + 1);
					}
			} else {
					/* Beef up Ringing voltage to 89V */
					if (boostringer) {
							a24xx_spi_setreg(wc_dev, i, 74, 0x3f);
							if (si321x_proslic_setreg_indirect(wc_dev, i, 21, 0x1d1)) {
									tmp_flag |=(1<<i);
									continue;
							}
							printk("Boosting ringer on slot %d (89V peak)\n", i + 1);
					} else if (lowpower) {
							if (si321x_proslic_setreg_indirect(wc_dev, i, 21, 0x108)) {
									tmp_flag |=(1<<i);
									continue;
							}
							printk("Reducing ring power on slot %d (50V peak)\n", i + 1);
					}
			}
			
			if (si321x_init_ring_generator_mode(wc_dev, i)) {
					tmp_flag |=(1<<i);
					continue;
			}
			
			if(fxstxgain || fxsrxgain) {
					r9 = a24xx_spi_getreg(wc_dev, i, 9);
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
					a24xx_spi_setreg(wc_dev,i,9,r9);
			}

			if(debug) {
					printk("DEBUG: fxstxgain:%s fxsrxgain:%s\n",
							((a24xx_spi_getreg(wc_dev, i, 9)/8) == 1) ? "3.5":
			     				(((a24xx_spi_getreg(wc_dev, i, 9)/4) == 1) ? "-3.5":"0.0"),
                  				((a24xx_spi_getreg(wc_dev, i, 9)/2) == 1) ?"3.5":((a24xx_spi_getreg(wc_dev,i,9)%2) ? "-3.5":"0.0")
					);
			}

			a24xx_spi_setreg(wc_dev, i, 64, 0x01);
		}
	}
	ret_flag |= tmp_flag;
	
	return ret_flag;
}

int si321x_proslic_setreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address, unsigned short data)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&wc_dev->lock, flags);
	ret = __a24xx_proslic_setreg_indirect(wc_dev, card, address, data);
	spin_unlock_irqrestore(&wc_dev->lock, flags);

	return ret;	
}

int si321x_proslic_getreg_indirect(struct a24xx_dev *wc_dev, int card, unsigned char address)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&wc_dev->lock, flags);
	ret = __a24xx_proslic_getreg_indirect(wc_dev, card, address);
	spin_unlock_irqrestore(&wc_dev->lock, flags);

	return ret;	
}

void si321x_proslic_recheck_sanity(struct a24xx_dev *wc_dev, int card)
{
	int res;
	
	/* Check loopback */
	res = wc_dev->reg1shadow[card];
	if (!res && (res != wc_dev->mod[card].fxs.lasttxhook))     // read real state from register   By wx
		res=a24xx_spi_getreg(wc_dev, card, 64);
	if (!res && (res != wc_dev->mod[card].fxs.lasttxhook)) {
		res = a24xx_spi_getreg(wc_dev, card, 8);
		if (res) {
			printk("Ouch, part reset, quickly restoring reality (%d)\n", card);
			si321x_init_proslic(wc_dev, card, 1, 0, 1);
		} else {
			if (wc_dev->mod[card].fxs.palarms++ < MAX_ALARMS) {
				printk("Power alarm on module %d, resetting!\n", card + 1);
				if (wc_dev->mod[card].fxs.lasttxhook == 4)
					wc_dev->mod[card].fxs.lasttxhook = 1;
				a24xx_spi_setreg(wc_dev, card, 64, wc_dev->mod[card].fxs.lasttxhook);
			} else {
				if (wc_dev->mod[card].fxs.palarms == MAX_ALARMS)
					printk("Too many power alarms on card %d, NOT resetting!\n", card + 1);
			}
		}
	}
}
