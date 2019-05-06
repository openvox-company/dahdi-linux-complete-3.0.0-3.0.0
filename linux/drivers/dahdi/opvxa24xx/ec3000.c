/*
 * Copyright (C) 2005-2006 Digium, Inc.
 *
 * Mark Spencer <markster@digium.com>
 * Mark liu <mark.liu@openvox.cn>
 *
 * $Id: ec3000.c 159 2010-12-08 03:27:04Z liuyuan $
 * All Rights Reserved
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

#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/version.h>

#include "ec3000.h"
#include "oct6100api/oct6100_api.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif

/* API for Octasic access */
UINT32 Oct6100UserGetTime(tPOCT6100_GET_TIME f_pTime)
{
	/* Why couldn't they just take a timeval like everyone else? */
	struct timeval tv;
	unsigned long long total_usecs;
	unsigned int mask = ~0;
	
	do_gettimeofday(&tv);
	total_usecs = (((unsigned long long)(tv.tv_sec)) * 1000000) + 
				  (((unsigned long long)(tv.tv_usec)));
	f_pTime->aulWallTimeUs[0] = (total_usecs & mask);
	f_pTime->aulWallTimeUs[1] = (total_usecs >> 32);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserMemSet(PVOID f_pAddress, UINT32 f_ulPattern, UINT32 f_ulLength)
{
	memset(f_pAddress, f_ulPattern, f_ulLength);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserMemCopy(PVOID f_pDestination, const void *f_pSource, UINT32 f_ulLength)
{
	memcpy(f_pDestination, f_pSource, f_ulLength);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserCreateSerializeObject(tPOCT6100_CREATE_SERIALIZE_OBJECT f_pCreate)
{
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDestroySerializeObject(tPOCT6100_DESTROY_SERIALIZE_OBJECT f_pDestroy)
{
#ifdef OCTASIC_DEBUG
	printk("I should never be called! (destroy serialize object)\n");
#endif
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserSeizeSerializeObject(tPOCT6100_SEIZE_SERIALIZE_OBJECT f_pSeize)
{
	/* Not needed */
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserReleaseSerializeObject(tPOCT6100_RELEASE_SERIALIZE_OBJECT f_pRelease)
{
	/* Not needed */
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteApi(tPOCT6100_WRITE_PARAMS f_pWriteParams)
{
	oct_set_reg(f_pWriteParams->pProcessContext, f_pWriteParams->ulWriteAddress, f_pWriteParams->usWriteData);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteSmearApi(tPOCT6100_WRITE_SMEAR_PARAMS f_pSmearParams)
{
	unsigned int x;
	for (x=0;x<f_pSmearParams->ulWriteLength;x++) {
		oct_set_reg(f_pSmearParams->pProcessContext, f_pSmearParams->ulWriteAddress + (x << 1), f_pSmearParams->usWriteData);
	}
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteBurstApi(tPOCT6100_WRITE_BURST_PARAMS f_pBurstParams)
{
	unsigned int x;
	for (x=0;x<f_pBurstParams->ulWriteLength;x++) {
		oct_set_reg(f_pBurstParams->pProcessContext, f_pBurstParams->ulWriteAddress + (x << 1), f_pBurstParams->pusWriteData[x]);
	}
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverReadApi(tPOCT6100_READ_PARAMS f_pReadParams)
{
	*(f_pReadParams->pusReadData) = oct_get_reg(f_pReadParams->pProcessContext, f_pReadParams->ulReadAddress);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverReadBurstApi(tPOCT6100_READ_BURST_PARAMS f_pBurstParams)
{
	unsigned int x;
	for (x=0;x<f_pBurstParams->ulReadLength;x++) {
		f_pBurstParams->pusReadData[x] = oct_get_reg(f_pBurstParams->pProcessContext, f_pBurstParams->ulReadAddress + (x << 1));
	}
	return cOCT6100_ERR_OK;
}

#define SOUT_G168_1100GB_ON 0x40000004
#define SOUT_DTMF_1 0x40000011
#define SOUT_DTMF_2 0x40000012
#define SOUT_DTMF_3 0x40000013
#define SOUT_DTMF_A 0x4000001A
#define SOUT_DTMF_4 0x40000014
#define SOUT_DTMF_5 0x40000015
#define SOUT_DTMF_6 0x40000016
#define SOUT_DTMF_B 0x4000001B
#define SOUT_DTMF_7 0x40000017
#define SOUT_DTMF_8 0x40000018
#define SOUT_DTMF_9 0x40000019
#define SOUT_DTMF_C 0x4000001C
#define SOUT_DTMF_STAR 0x4000001E
#define SOUT_DTMF_0 0x40000010
#define SOUT_DTMF_POUND 0x4000001F
#define SOUT_DTMF_D 0x4000001D

#define ROUT_G168_2100GB_ON 0x10000000
#define ROUT_G168_2100GB_WSPR 0x10000002
#define ROUT_SOUT_G168_2100HB_END 0x50000003
#define ROUT_G168_1100GB_ON 0x10000004

#define ROUT_DTMF_1 0x10000011
#define ROUT_DTMF_2 0x10000012
#define ROUT_DTMF_3 0x10000013
#define ROUT_DTMF_A 0x1000001A
#define ROUT_DTMF_4 0x10000014
#define ROUT_DTMF_5 0x10000015
#define ROUT_DTMF_6 0x10000016
#define ROUT_DTMF_B 0x1000001B
#define ROUT_DTMF_7 0x10000017
#define ROUT_DTMF_8 0x10000018
#define ROUT_DTMF_9 0x10000019
#define ROUT_DTMF_C 0x1000001C
#define ROUT_DTMF_STAR 0x1000001E
#define ROUT_DTMF_0 0x10000010
#define ROUT_DTMF_POUND 0x1000001F
#define ROUT_DTMF_D 0x1000001D

#if 0 
#define cOCT6100_ECHO_OP_MODE_DIGITAL cOCT6100_ECHO_OP_MODE_HT_FREEZE
#else
#define cOCT6100_ECHO_OP_MODE_DIGITAL cOCT6100_ECHO_OP_MODE_POWER_DOWN
#endif

struct ec {
	tPOCT6100_INSTANCE_API pApiInstance;
	UINT32 aulEchoChanHndl[ 128 ];
	int chanflags[128];
	int ecmode[128];
	int numchans;
};

#define FLAG_DTMF	 (1 << 0)
#define FLAG_MUTE	 (1 << 1)
#define FLAG_ECHO	 (1 << 2)

static unsigned int tones[] = {
	SOUT_DTMF_1,
	SOUT_DTMF_2,
	SOUT_DTMF_3,
	SOUT_DTMF_A,
	SOUT_DTMF_4,
	SOUT_DTMF_5,
	SOUT_DTMF_6,
	SOUT_DTMF_B,
	SOUT_DTMF_7,
	SOUT_DTMF_8,
	SOUT_DTMF_9,
	SOUT_DTMF_C,
	SOUT_DTMF_STAR,
	SOUT_DTMF_0,
	SOUT_DTMF_POUND,
	SOUT_DTMF_D,
	SOUT_G168_1100GB_ON,

	ROUT_DTMF_1,
	ROUT_DTMF_2,
	ROUT_DTMF_3,
	ROUT_DTMF_A,
	ROUT_DTMF_4,
	ROUT_DTMF_5,
	ROUT_DTMF_6,
	ROUT_DTMF_B,
	ROUT_DTMF_7,
	ROUT_DTMF_8,
	ROUT_DTMF_9,
	ROUT_DTMF_C,
	ROUT_DTMF_STAR,
	ROUT_DTMF_0,
	ROUT_DTMF_POUND,
	ROUT_DTMF_D,
	ROUT_G168_1100GB_ON,
};
static void opvx_vpm_setecmode(struct ec *ec, int channel, int mode)
{
	tOCT6100_CHANNEL_MODIFY *modify;
	UINT32 ulResult;

	if (ec->ecmode[channel] == mode)
		return;
	modify = kmalloc(sizeof(tOCT6100_CHANNEL_MODIFY), GFP_ATOMIC);
	if (!modify) {
		printk("opvxa24xx: Unable to allocate memory for setec!\n");
		return;
	}
	Oct6100ChannelModifyDef(modify);
	modify->ulEchoOperationMode = mode;
	modify->ulChannelHndl = ec->aulEchoChanHndl[channel];
	ulResult = Oct6100ChannelModify(ec->pApiInstance, modify);
	if (ulResult != GENERIC_OK) {
		printk("Failed to apply echo can changes on channel %d, 0x%x!\n", channel, ulResult);
	} else {
#ifdef OCTASIC_DEBUG
		printk("Echo can on channel %d set to %d\n", channel, mode);
#endif
		ec->ecmode[channel] = mode;
	}
	kfree(modify);
}

void opvx_vpm_setdtmf(struct ec *ec, int channel, int detect, int mute)
{
	tOCT6100_CHANNEL_MODIFY *modify;
	UINT32 ulResult;

	modify = kmalloc(sizeof(tOCT6100_CHANNEL_MODIFY), GFP_KERNEL);
	if (!modify) {
		printk("opvxa24xx: Unable to allocate memory for setdtmf!\n");
		return;
	}
	Oct6100ChannelModifyDef(modify);
	modify->ulChannelHndl = ec->aulEchoChanHndl[channel];
	if (mute) {
		ec->chanflags[channel] |= FLAG_MUTE;
		modify->VqeConfig.fDtmfToneRemoval = TRUE;
	} else {
		ec->chanflags[channel] &= ~FLAG_MUTE;
		modify->VqeConfig.fDtmfToneRemoval = FALSE;
	}
	if (detect)
		ec->chanflags[channel] |= FLAG_DTMF;
	else
		ec->chanflags[channel] &= ~FLAG_DTMF;
	if (ec->chanflags[channel] & (FLAG_DTMF|FLAG_MUTE)) {
		if (!(ec->chanflags[channel] & FLAG_ECHO)) {
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_HT_FREEZE);
		}
	} else {
		if (!(ec->chanflags[channel] & FLAG_ECHO))
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_DIGITAL);
	}

	ulResult = Oct6100ChannelModify(ec->pApiInstance, modify);
	if (ulResult != GENERIC_OK) {
		printk("Failed to apply dtmf mute changes on channel %d!\n", channel);
	}
/*	printk("VPM450m: Setting DTMF on channel %d: %s / %s\n", channel, (detect ? "DETECT" : "NO DETECT"), (mute ? "MUTE" : "NO MUTE")); */
	kfree(modify);
}


void opvx_vpm_setec(struct ec *ec, int channel, int eclen)
{
	if (eclen) {
		ec->chanflags[channel] |= FLAG_ECHO;
		opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
		opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_NORMAL);
	} else {
		ec->chanflags[channel] &= ~FLAG_ECHO;
		if (ec->chanflags[channel] & (FLAG_DTMF | FLAG_MUTE)) {
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_HT_FREEZE);
		} else
			opvx_vpm_setecmode(ec, channel, cOCT6100_ECHO_OP_MODE_DIGITAL);
	}
/*	printk("VPM450m: Setting EC on channel %d to %d\n", channel, eclen); */
}

int opvx_vpm_checkirq(struct ec *ec)
{
	tOCT6100_INTERRUPT_FLAGS InterruptFlags;
	
	Oct6100InterruptServiceRoutineDef(&InterruptFlags);
	Oct6100InterruptServiceRoutine(ec->pApiInstance, &InterruptFlags);

	return InterruptFlags.fToneEventsPending ? 1 : 0;
}

int opvx_vpm_getdtmf(struct ec *ec, int *channel, int *tone, int *start)
{
	tOCT6100_TONE_EVENT tonefound;
	tOCT6100_EVENT_GET_TONE tonesearch;
	UINT32 ulResult;
	
	Oct6100EventGetToneDef(&tonesearch);
	tonesearch.pToneEvent = &tonefound;
	tonesearch.ulMaxToneEvent = 1;
	ulResult = Oct6100EventGetTone(ec->pApiInstance, &tonesearch);
	if (tonesearch.ulNumValidToneEvent) {
		if (channel)
			*channel = tonefound.ulUserChanId;
		if (tone) {
			switch(tonefound.ulToneDetected) {
			case SOUT_DTMF_1:
				*tone = '1';
				break;
			case SOUT_DTMF_2:
				*tone = '2';
				break;
			case SOUT_DTMF_3:
				*tone = '3';
				break;
			case SOUT_DTMF_A:
				*tone = 'A';
				break;
			case SOUT_DTMF_4:
				*tone = '4';
				break;
			case SOUT_DTMF_5:
				*tone = '5';
				break;
			case SOUT_DTMF_6:
				*tone = '6';
				break;
			case SOUT_DTMF_B:
				*tone = 'B';
				break;
			case SOUT_DTMF_7:
				*tone = '7';
				break;
			case SOUT_DTMF_8:
				*tone = '8';
				break;
			case SOUT_DTMF_9:
				*tone = '9';
				break;
			case SOUT_DTMF_C:
				*tone = 'C';
				break;
			case SOUT_DTMF_STAR:
				*tone = '*';
				break;
			case SOUT_DTMF_0:
				*tone = '0';
				break;
			case SOUT_DTMF_POUND:
				*tone = '#';
				break;
			case SOUT_DTMF_D:
				*tone = 'D';
				break;
			case SOUT_G168_1100GB_ON:
				*tone = 'f';
				break;
			default:
#ifdef OCTASIC_DEBUG
				printk("Unknown tone value %08x\n", tonefound.ulToneDetected);
#endif
				*tone = 'u';
				break;
			}
		}
		if (start)
			*start = (tonefound.ulEventType == cOCT6100_TONE_PRESENT);
		return 1;
	}
	return 0;
}

unsigned int opvx_vpm_getcapacity(void *wc)
{
	UINT32 ulResult;

	tOCT6100_API_GET_CAPACITY_PINS CapacityPins;

	Oct6100ApiGetCapacityPinsDef(&CapacityPins);
	CapacityPins.pProcessContext = wc;
	CapacityPins.ulMemoryType = cOCT6100_MEM_TYPE_DDR;
	CapacityPins.fEnableMemClkOut = TRUE;
	CapacityPins.ulMemClkFreq = cOCT6100_MCLK_FREQ_133_MHZ;

	ulResult = Oct6100ApiGetCapacityPins(&CapacityPins);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to get chip capacity, code %08x!\n", ulResult);
		return 0;
	}

	return CapacityPins.ulCapacityValue;
}

struct ec *opvx_vpm_init(void *wc, int *isalaw, int numspans, const struct firmware *firmware)
{
	tOCT6100_CHIP_OPEN *ChipOpen;
	tOCT6100_GET_INSTANCE_SIZE InstanceSize;
	tOCT6100_CHANNEL_OPEN *ChannelOpen;
	UINT32 ulResult;
	struct ec *ec;
	int x,y,law;
#ifdef CONFIG_4KSTACKS
	unsigned long flags;
#endif
	
	if (!(ec = kmalloc(sizeof(struct ec), GFP_KERNEL)))
		return NULL;

	memset(ec, 0, sizeof(struct ec));

	if (!(ChipOpen = kmalloc(sizeof(tOCT6100_CHIP_OPEN), GFP_KERNEL))) {
		kfree(ec);
		return NULL;
	}

	memset(ChipOpen, 0, sizeof(tOCT6100_CHIP_OPEN));

	if (!(ChannelOpen = kmalloc(sizeof(tOCT6100_CHANNEL_OPEN), GFP_KERNEL))) {
		kfree(ec);
		kfree(ChipOpen);
		return NULL;
	}

	memset(ChannelOpen, 0, sizeof(tOCT6100_CHANNEL_OPEN));

	for (x=0;x<128;x++)
		ec->ecmode[x] = -1;

	ec->numchans = numspans * 32;
	printk("OpenVox VPM: echo cancellation for %d channels\n", ec->numchans);
		
	Oct6100ChipOpenDef(ChipOpen);

	/* Setup Chip Open Parameters */
	ChipOpen->ulUpclkFreq = cOCT6100_UPCLK_FREQ_33_33_MHZ;
	Oct6100GetInstanceSizeDef(&InstanceSize);

	ChipOpen->pProcessContext = wc;

	ChipOpen->pbyImageFile = firmware->data;
	ChipOpen->ulImageSize = firmware->size;
	ChipOpen->fEnableMemClkOut = TRUE;
	ChipOpen->ulMemClkFreq = cOCT6100_MCLK_FREQ_133_MHZ;
	ChipOpen->ulMaxChannels = ec->numchans;
	ChipOpen->ulMemoryType = cOCT6100_MEM_TYPE_DDR;
	ChipOpen->ulMemoryChipSize = cOCT6100_MEMORY_CHIP_SIZE_32MB;
	ChipOpen->ulNumMemoryChips = 1;
	ChipOpen->ulMaxTdmStreams = 4;
	ChipOpen->aulTdmStreamFreqs[0] = cOCT6100_TDM_STREAM_FREQ_8MHZ;
	ChipOpen->ulTdmSampling = cOCT6100_TDM_SAMPLE_AT_FALLING_EDGE;
#if 0
	ChipOpen->fEnableAcousticEcho = TRUE;
#endif		

	ulResult = Oct6100GetInstanceSize(ChipOpen, &InstanceSize);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to get instance size, code %08x!\n", ulResult);
		kfree(ec);
		kfree(ChipOpen);
		kfree(ChannelOpen);
		return NULL;
	}
	
	
	ec->pApiInstance = vmalloc(InstanceSize.ulApiInstanceSize);
	if (!ec->pApiInstance) {
		printk("Out of memory (can't allocate %d bytes)!\n", InstanceSize.ulApiInstanceSize);
		kfree(ec);
		kfree(ChipOpen);
		kfree(ChannelOpen);
		return NULL;
	}

	/* I don't know what to curse more in this comment, the problems caused by
	 * the 4K kernel stack limit change or the octasic API for being so darn
	 * stack unfriendly.  Stupid, stupid, stupid.  So we disable IRQs so we
	 * don't run the risk of overflowing the stack while we initialize the
	 * octasic. */
#ifdef CONFIG_4KSTACKS
	local_irq_save(flags);
#endif
	ulResult = Oct6100ChipOpen(ec->pApiInstance, ChipOpen);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to open chip, code %08x!\n", ulResult);
#ifdef CONFIG_4KSTACKS
		local_irq_restore(flags);
#endif
		kfree(ec);
		kfree(ChipOpen);
		kfree(ChannelOpen);
		return NULL;
	}
	for (x=0;x<128;x++) {
		/* execute this loop always on 4 span cards but
		*  on 2 span cards only execute for the channels related to our spans */
		//if (( numspans > 2) || ((x & 0x03) <2)) {
		//if ((x & 0x03) < numspans) {
		if ( x < ec->numchans) {
			if (isalaw[x / 24])     // each span have 24 channels, it is spec for the 24 channel card. miaolin
				law = cOCT6100_PCM_A_LAW;
			else
				law = cOCT6100_PCM_U_LAW;
			Oct6100ChannelOpenDef(ChannelOpen);
			ChannelOpen->pulChannelHndl = &ec->aulEchoChanHndl[x];
			ChannelOpen->ulUserChanId = x;
			ChannelOpen->TdmConfig.ulRinPcmLaw = law;
			ChannelOpen->TdmConfig.ulRinStream = 0;
			ChannelOpen->TdmConfig.ulRinTimeslot = x;
			ChannelOpen->TdmConfig.ulSinPcmLaw = law;
			ChannelOpen->TdmConfig.ulSinStream = 1;
			ChannelOpen->TdmConfig.ulSinTimeslot = x;
			ChannelOpen->TdmConfig.ulSoutPcmLaw = law;
			ChannelOpen->TdmConfig.ulSoutStream = 2;
			ChannelOpen->TdmConfig.ulSoutTimeslot = x;
			ChannelOpen->TdmConfig.ulRoutPcmLaw = law;
			ChannelOpen->TdmConfig.ulRoutStream = 3;
			ChannelOpen->TdmConfig.ulRoutTimeslot = x;
			ChannelOpen->VqeConfig.fEnableNlp = TRUE;
			ChannelOpen->VqeConfig.fRinDcOffsetRemoval = TRUE;
			ChannelOpen->VqeConfig.fSinDcOffsetRemoval = TRUE;
			
			ChannelOpen->fEnableToneDisabler = TRUE;
			ChannelOpen->ulEchoOperationMode = cOCT6100_ECHO_OP_MODE_DIGITAL;
			
			ulResult = Oct6100ChannelOpen(ec->pApiInstance, ChannelOpen);
			if (ulResult != GENERIC_OK) {
				printk("Failed to open channel %d!\n", x);
			}
			for (y=0;y<sizeof(tones) / sizeof(tones[0]); y++) {
				tOCT6100_TONE_DETECTION_ENABLE enable;
				Oct6100ToneDetectionEnableDef(&enable);
				enable.ulChannelHndl = ec->aulEchoChanHndl[x];
				enable.ulToneNumber = tones[y];
				if (Oct6100ToneDetectionEnable(ec->pApiInstance, &enable) != GENERIC_OK) 
					printk("Failed to enable tone detection on channel %d for tone %d!\n", x, y);
			}
		}
	}

#ifdef CONFIG_4KSTACKS
	local_irq_restore(flags);
#endif
	kfree(ChipOpen);
	kfree(ChannelOpen);
	return ec;
}

void opvx_vpm_release(struct ec *ec)
{
	UINT32 ulResult;
	tOCT6100_CHIP_CLOSE ChipClose;

	Oct6100ChipCloseDef(&ChipClose);
	ulResult = Oct6100ChipClose(ec->pApiInstance, &ChipClose);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to close chip, code %08x!\n", ulResult);
	}
	vfree(ec->pApiInstance);
	kfree(ec);
}

