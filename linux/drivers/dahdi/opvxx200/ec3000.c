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

static void oct_set_reg(void *data, unsigned int reg, unsigned int val)
{
    struct x200 *x2 = data;
    unsigned long flags;
    int count = 1000;

    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, V2_EC_BASE, 0x0008, (reg >> 20));
    __x200_setcreg(x2, V2_EC_BASE, 0x000a, (reg >> 4) & ((1 << 16) - 1));
    __x200_setcreg(x2, V2_EC_BASE, 0x0004, val);
    __x200_setcreg(x2, V2_EC_BASE, 0x0000, (((reg >> 1) & 0x7) << 9) | (1 << 8) | (3 << 12) | 1);
    while((__x200_getcreg(x2,V2_EC_BASE, 0x0000) & (1 << 8)) && --count);
    if (!count) {
        printk("Write timed out!\n");
    }
    spin_unlock_irqrestore(&x2->lock, flags);
}

static unsigned int oct_get_reg(void *data, unsigned int reg)
{
    struct x200 *x2 = data;
    unsigned int ret;
    unsigned long flags;
    int count = 1000;

    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, V2_EC_BASE, 0x0008, (reg >> 20));
    __x200_setcreg(x2, V2_EC_BASE, 0x000a, (reg >> 4) & ((1 << 16) - 1));
    __x200_setcreg(x2, V2_EC_BASE, 0x0000, (((reg >> 1) & 0x7) << 9) | (1 << 8) | (1));
    while((__x200_getcreg(x2, V2_EC_BASE, 0x0000) & (1 << 8)) && --count);
    if (!count) {
        printk("Read timed out!\n");
    }
    ret = __x200_getcreg(x2,V2_EC_BASE, 0x0004);
    spin_unlock_irqrestore(&x2->lock, flags);
    return ret;
}

/* API for Octasic access */
UINT32 Oct6100UserGetTime(tPOCT6100_GET_TIME f_pTime)
{
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

#define cOCT6100_ECHO_OP_MODE_DIGITAL cOCT6100_ECHO_OP_MODE_POWER_DOWN

struct vpm450m {
    tPOCT6100_INSTANCE_API pApiInstance;
    UINT32 aulEchoChanHndl[128];
    int chanflags[128];
    int ecmode[128];

    int inited_chan_num ;
    int inited_chans[128];
    int numchans;
    int isalaw[4];
    tOCT6100_CHIP_OPEN *ChipOpen; 
    tOCT6100_CHANNEL_OPEN *ChannelOpen; 
};

#define FLAG_DTMF    (1 << 0)
#define FLAG_MUTE    (1 << 1)
#define FLAG_ECHO    (1 << 2)


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


static void vpm450m_setecmode(struct vpm450m *vpm450m, int channel, int mode)
{
    tOCT6100_CHANNEL_MODIFY *modify;
    UINT32 ulResult;

    if (vpm450m->ecmode[channel] == mode)
        return;
    modify = kmalloc(sizeof(tOCT6100_CHANNEL_MODIFY), GFP_ATOMIC);
    if (!modify) {
        printk("Unable to allocate memory for setec!\n");
        return;
    }
    Oct6100ChannelModifyDef(modify);
    modify->ulEchoOperationMode = mode;
    modify->ulChannelHndl = vpm450m->aulEchoChanHndl[channel];
    ulResult = Oct6100ChannelModify(vpm450m->pApiInstance, modify);
    if (ulResult != GENERIC_OK) {
        printk("Failed to apply echo can changes on channel %d, 0x%x!\n", channel, ulResult);
    } else {
#ifdef OCTASIC_DEBUG
		printk("Echo can on channel %d set to %d\n", channel, mode);
#endif
        vpm450m->ecmode[channel] = mode;
    }
    kfree(modify);
}

void vpm450m_setdtmf(struct vpm450m *vpm450m, int channel, int detect, int mute)
{
    tOCT6100_CHANNEL_MODIFY *modify;
    UINT32 ulResult;

    modify = kmalloc(sizeof(tOCT6100_CHANNEL_MODIFY), GFP_KERNEL);
    if (!modify) {
        printk("Unable to allocate memory for setdtmf!\n");
        return;
    }
    Oct6100ChannelModifyDef(modify);
    modify->ulChannelHndl = vpm450m->aulEchoChanHndl[channel];
    if (mute) {
        vpm450m->chanflags[channel] |= FLAG_MUTE;
        modify->VqeConfig.fDtmfToneRemoval = TRUE;
    } else {
        vpm450m->chanflags[channel] &= ~FLAG_MUTE;
        modify->VqeConfig.fDtmfToneRemoval = FALSE;
    }
    if (detect)
        vpm450m->chanflags[channel] |= FLAG_DTMF;
    else
        vpm450m->chanflags[channel] &= ~FLAG_DTMF;
    if (vpm450m->chanflags[channel] & (FLAG_DTMF|FLAG_MUTE)) {
        if (!(vpm450m->chanflags[channel] & FLAG_ECHO)) {
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_HT_FREEZE);
        }
    } else {
        if (!(vpm450m->chanflags[channel] & FLAG_ECHO)) {
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_DIGITAL);
        }
    }

    ulResult = Oct6100ChannelModify(vpm450m->pApiInstance, modify);
    if (ulResult != GENERIC_OK) {
        printk("Failed to apply dtmf mute changes on channel %d!\n", channel);
    }
    kfree(modify);
}


void vpm450m_setec(struct x200 *x2, int channel, int slot_id, int eclen)
{
    struct vpm450m *vpm450m=x2->vpm450m;
    if (eclen) {
        vpm450m->chanflags[channel] |= FLAG_ECHO;
        vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
        vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_NORMAL);
    } else {
        vpm450m->chanflags[channel] &= ~FLAG_ECHO;
        if (vpm450m->chanflags[channel] & (FLAG_DTMF | FLAG_MUTE)) {
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_HT_FREEZE);
        } else {
            vpm450m_setecmode(vpm450m, channel, cOCT6100_ECHO_OP_MODE_DIGITAL);
        }
    }
}

int vpm450m_checkirq(struct vpm450m *vpm450m)
{
    tOCT6100_INTERRUPT_FLAGS InterruptFlags;

    Oct6100InterruptServiceRoutineDef(&InterruptFlags);
    Oct6100InterruptServiceRoutine(vpm450m->pApiInstance, &InterruptFlags);
    return InterruptFlags.fToneEventsPending ? 1 : 0;
}

int vpm450m_getdtmf(struct vpm450m *vpm450m, int *channel, int *tone, int *start)
{
    tOCT6100_TONE_EVENT tonefound;
    tOCT6100_EVENT_GET_TONE tonesearch;
    UINT32 ulResult;

    Oct6100EventGetToneDef(&tonesearch);
    tonesearch.pToneEvent = &tonefound;
    tonesearch.ulMaxToneEvent = 1;
    ulResult = Oct6100EventGetTone(vpm450m->pApiInstance, &tonesearch);
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

unsigned int get_vpm450m_capacity(void * x2)
{
    UINT32 ulResult;

    tOCT6100_API_GET_CAPACITY_PINS CapacityPins;

    Oct6100ApiGetCapacityPinsDef(&CapacityPins);
    CapacityPins.pProcessContext = x2;
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

struct vpm450m *init_vpm450m(struct x200 *x2, int *isalaw, int numchans,const struct firmware *firmware)
{
    tOCT6100_GET_INSTANCE_SIZE InstanceSize;
    UINT32 ulResult;
    struct vpm450m *vpm450m;
    int tdm_slot_id;

    if (!(vpm450m = kmalloc(sizeof(struct vpm450m), GFP_KERNEL)))
        return NULL;
    memset(vpm450m, 0, sizeof(struct vpm450m));

    if (!(vpm450m->ChipOpen = kmalloc(sizeof(tOCT6100_CHIP_OPEN), GFP_KERNEL))) {
        kfree(vpm450m);
        return NULL;
    }
    memset(vpm450m->ChipOpen, 0, sizeof(tOCT6100_CHIP_OPEN));

    if (!(vpm450m->ChannelOpen = kmalloc(sizeof(tOCT6100_CHANNEL_OPEN), GFP_KERNEL))) {
        kfree(vpm450m->ChipOpen);
        kfree(vpm450m);
        return NULL;
    }

    memset(vpm450m->ChannelOpen, 0, sizeof(tOCT6100_CHANNEL_OPEN));

    for (tdm_slot_id=0;tdm_slot_id<128;tdm_slot_id++) {
        vpm450m->ecmode[tdm_slot_id] = -1;
        vpm450m->inited_chans[tdm_slot_id]=0 ;
    }
    vpm450m->inited_chan_num = 0 ;
    for (tdm_slot_id = 0;tdm_slot_id < 4; tdm_slot_id++) {
        vpm450m->isalaw[tdm_slot_id] = isalaw[tdm_slot_id];
    }

    vpm450m->numchans = numchans ;
    //printk("OpenVox VPM: echo cancellation for %d channels\n", vpm450m->numchans);
    Oct6100ChipOpenDef(vpm450m->ChipOpen);

    /* Setup Chip Open Parameters */
    vpm450m->ChipOpen->ulUpclkFreq = cOCT6100_UPCLK_FREQ_33_33_MHZ;
    Oct6100GetInstanceSizeDef(&InstanceSize);
    
    vpm450m->ChipOpen->pProcessContext = x2;
    
    vpm450m->ChipOpen->pbyImageFile = firmware->data;
    vpm450m->ChipOpen->ulImageSize = firmware->size;
    vpm450m->ChipOpen->fEnableMemClkOut = TRUE;
    vpm450m->ChipOpen->ulMemClkFreq = cOCT6100_MCLK_FREQ_133_MHZ;
    vpm450m->ChipOpen->ulMaxChannels = vpm450m->numchans;
    vpm450m->ChipOpen->ulMemoryType = cOCT6100_MEM_TYPE_DDR;
    vpm450m->ChipOpen->ulMemoryChipSize = cOCT6100_MEMORY_CHIP_SIZE_32MB;
    vpm450m->ChipOpen->ulNumMemoryChips = 1;
    vpm450m->ChipOpen->ulMaxTdmStreams = 4;
    vpm450m->ChipOpen->aulTdmStreamFreqs[0] = cOCT6100_TDM_STREAM_FREQ_8MHZ;
    vpm450m->ChipOpen->ulTdmSampling = cOCT6100_TDM_SAMPLE_AT_FALLING_EDGE;
    vpm450m->ChipOpen->ulMaxFlexibleConfParticipants = 0;
		vpm450m->ChipOpen->ulMaxConfBridges = 0;
		vpm450m->ChipOpen->ulMaxRemoteDebugSessions = 0;
		vpm450m->ChipOpen->fEnableChannelRecording = FALSE;
		vpm450m->ChipOpen->ulSoftToneEventsBufSize = 64;

    ulResult = Oct6100GetInstanceSize(vpm450m->ChipOpen, &InstanceSize);
    if (ulResult != cOCT6100_ERR_OK) {
        printk("Failed to get instance size, code %08x!\n", ulResult);
        kfree(vpm450m->ChannelOpen);
        kfree(vpm450m->ChipOpen);
        kfree(vpm450m);
        return NULL;
    }

    vpm450m->pApiInstance = vmalloc(InstanceSize.ulApiInstanceSize);
    if (!vpm450m->pApiInstance) {
        printk("Out of memory (can't allocate %d bytes)!\n", InstanceSize.ulApiInstanceSize);
        kfree(vpm450m->ChannelOpen);
        kfree(vpm450m->ChipOpen);
        kfree(vpm450m);
        return NULL;
    }

    ulResult = Oct6100ChipOpen(vpm450m->pApiInstance, vpm450m->ChipOpen);
    if (ulResult != cOCT6100_ERR_OK) {
        printk("Failed to open chip, code %08x!\n", ulResult);
        kfree(vpm450m->ChannelOpen);
        kfree(vpm450m->ChipOpen);
        kfree(vpm450m);
        return NULL;
    }
		
    return vpm450m;
}

int init_vpm450m_chan(struct x200 *x2, int channel,int slot_id)
{
    UINT32 ulResult;
    struct vpm450m *vpm450m=x2->vpm450m;
    int tones_id,law;

    if (vpm450m->inited_chans[channel]) {
        printk("Openvox X200 vpm450m : channel:%d already inited.\n",channel );
        return 0 ; 
    }
    if (vpm450m->inited_chan_num >= vpm450m->numchans) {
        printk("Openvox X200 vpm450m : numchans:%d, but needed chans:%d.\n", vpm450m->numchans,vpm450m->inited_chan_num );
        return -1 ; 
    }
    vpm450m->inited_chans[channel] = 1 ;
    vpm450m->inited_chan_num++ ;

    if (vpm450m->isalaw[slot_id]) {
        law = cOCT6100_PCM_A_LAW;
    } else {
        law = cOCT6100_PCM_U_LAW;
    }

    Oct6100ChannelOpenDef(vpm450m->ChannelOpen);
    vpm450m->ChannelOpen->pulChannelHndl = &vpm450m->aulEchoChanHndl[channel];
    vpm450m->ChannelOpen->ulUserChanId = channel;
    vpm450m->ChannelOpen->TdmConfig.ulRinPcmLaw = law;
    vpm450m->ChannelOpen->TdmConfig.ulRinStream = 0;
    vpm450m->ChannelOpen->TdmConfig.ulRinTimeslot = channel;
    vpm450m->ChannelOpen->TdmConfig.ulSinPcmLaw = law;
    vpm450m->ChannelOpen->TdmConfig.ulSinStream = 1;
    vpm450m->ChannelOpen->TdmConfig.ulSinTimeslot = channel;
    vpm450m->ChannelOpen->TdmConfig.ulSoutPcmLaw = law;
    vpm450m->ChannelOpen->TdmConfig.ulSoutStream = 2;
    vpm450m->ChannelOpen->TdmConfig.ulSoutTimeslot = channel;
    vpm450m->ChannelOpen->TdmConfig.ulRoutPcmLaw = law;
    vpm450m->ChannelOpen->TdmConfig.ulRoutStream = 3;
    vpm450m->ChannelOpen->TdmConfig.ulRoutTimeslot = channel;
    vpm450m->ChannelOpen->VqeConfig.fEnableNlp = TRUE;
    vpm450m->ChannelOpen->VqeConfig.fRinDcOffsetRemoval = TRUE;
    vpm450m->ChannelOpen->VqeConfig.fSinDcOffsetRemoval = TRUE;
    vpm450m->ChannelOpen->fEnableToneDisabler = TRUE;
    vpm450m->ChannelOpen->ulEchoOperationMode = cOCT6100_ECHO_OP_MODE_DIGITAL;
    ulResult = Oct6100ChannelOpen(vpm450m->pApiInstance, vpm450m->ChannelOpen);
    if (ulResult != GENERIC_OK) {
        printk("Openvox X200 vpm450m : Failed to open channel %d!\n", channel);
        return -2;
    }
    for (tones_id=0;tones_id<sizeof(tones) / sizeof(tones[0]); tones_id++) {
        tOCT6100_TONE_DETECTION_ENABLE enable;
        Oct6100ToneDetectionEnableDef(&enable);
        enable.ulChannelHndl = vpm450m->aulEchoChanHndl[channel];
        enable.ulToneNumber = tones[tones_id];
        if (Oct6100ToneDetectionEnable(vpm450m->pApiInstance, &enable) != GENERIC_OK) {
            printk("Openvox X200 vpm450m : Failed to enable tone detection on channel %d for tone %d!\n", channel, tones_id);
            return -3;
        }
    }
    					
    return 0 ; 
}

void release_vpm450m(struct vpm450m *vpm450m)
{
    UINT32 ulResult;
    tOCT6100_CHIP_CLOSE ChipClose;

    Oct6100ChipCloseDef(&ChipClose);
    ulResult = Oct6100ChipClose(vpm450m->pApiInstance, &ChipClose);
    if (ulResult != cOCT6100_ERR_OK) {
        printk("Failed to close chip, code %08x!\n", ulResult);
    }
    vfree(vpm450m->pApiInstance);
    kfree(vpm450m->ChannelOpen); 
    kfree(vpm450m->ChipOpen); 
    kfree(vpm450m);
}
