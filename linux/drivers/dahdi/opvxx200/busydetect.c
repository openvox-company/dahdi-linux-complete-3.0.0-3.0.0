/*
 * OpenVox FXO Detect Busy Voice Driver for DAHDI Telephony interface
 *
 * Written by kevin.chen

 * Copyright (C) 2012 OpenVox Communication Co. Ltd,
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

/* Rev history
 *
 * Rev 0.10 remove structure country_busytone
 *  	    add GEN_BUSYTONE_ONTIME and GEN_BUSYTONE_OFFTIME
 * Rev 0.20 fix bug
 *  	    after frequency is set to a valid value, cannot reset the zero value
 *
 */

#include <linux/proc_fs.h>
#include "x2fxo200m.h"

static const char *module_name = "opvxdsp";
static int ref_count = 0;

#define MAX_TONE_NUM            3
#define TONE_FREQ_NUM           2

#define GEN_BUSYTONE_ONTIME	500
#define GEN_BUSYTONE_OFFTIME	500

#define TONE_THRESHOLD		20548
#define SILENT_THRESHOLD	2560
#define ENERGY_SCALE		64
#define DAHDI_CHUNKSIZE		8

#define TOTAL_CARDS		120

#define SIN_DIVISION		128

static const short sin_table[] =
{
	201,
	603,
	1005,
	1407,
	1809,
	2210,
	2611,
	3012,
	3412,
	3812,
	4211,
	4609,
	5007,
	5404,
	5800,
	6195,
	6590,
	6983,
	7376,
	7767,
	8157,
	8546,
	8933,
	9319,
	9704,
	10088,
	10469,
	10850,
	11228,
	11605,
	11980,
	12354,
	12725,
	13095,
	13463,
	13828,
	14192,
	14553,
	14912,
	15269,
	15624,
	15976,
	16326,
	16673,
	17018,
	17361,
	17700,
	18037,
	18372,
	18703,
	19032,
	19358,
	19681,
	20001,
	20318,
	20632,
	20943,
	21251,
	21555,
	21856,
	22154,
	22449,
	22740,
	23028,
	23312,
	23593,
	23870,
	24144,
	24414,
	24680,
	24943,
	25202,
	25457,
	25708,
	25956,
	26199,
	26439,
	26674,
	26906,
	27133,
	27357,
	27576,
	27791,
	28002,
	28209,
	28411,
	28610,
	28803,
	28993,
	29178,
	29359,
	29535,
	29707,
	29875,
	30038,
	30196,
	30350,
	30499,
	30644,
	30784,
	30920,
	31050,
	31177,
	31298,
	31415,
	31527,
	31634,
	31737,
	31834,
	31927,
	32015,
	32099,
	32177,
	32251,
	32319,
	32383,
	32442,
	32496,
	32546,
	32590,
	32629,
	32664,
	32693,
	32718,
	32738,
	32753,
	32762,
	32767,
	32767
};

/* Level index range -30 ~ 0, step 1 */
#define MIN_INDEX_LEVEL         -30
#define MAX_INDEX_LEVEL         0
#define STEP_LEVEL              1
static const short gain_table[] = {
        722,
        810,
        909,
        1020,
        1144,
        1284,
        1440,
        1616,
        1813,
        2034,
        2283,
        2561,
        2874,
        3224,
        3618,
        4059,
        4554,
        5110,
        5734,
        6433,
        7218,
        8099,
        9087,
        10196,
        11440,
        12836,
        14402,
        16160,
        18132,
        20344,
        22826,
};

/* Frequency index range 300 ~ 700, step 5 */
#define MIN_INDEX_FREQ          300
#define MAX_INDEX_FREQ          700
#define STEP_FREQ               5
static const short fac_table[] = {
        31861,
        31830,
        31800,
        31768,
        31737,
        31704,
        31672,
        31638,
        31605,
        31570,
        31536,
        31501,
        31465,
        31429,
        31392,
        31355,
        31318,
        31279,
        31241,
        31202,
        31162,
        31122,
        31082,
        31041,
        30999,
        30958,
        30915,
        30872,
        30829,
        30785,
        30741,
        30696,
        30651,
        30605,
        30559,
        30512,
        30465,
        30417,
        30369,
        30321,
        30272,
        30222,
        30172,
        30122,
        30071,
        30020,
        29968,
        29916,
        29863,
        29810,
        29756,
        29702,
        29648,
        29593,
        29537,
        29481,
        29425,
        29368,
        29311,
        29253,
        29195,
        29136,
        29077,
        29017,
        28957,
        28897,
        28836,
        28775,
        28713,
        28651,
        28588,
        28525,
        28462,
        28398,
        28333,
        28268,
        28203,
        28137,
        28071,
        28005,
	27938,
};

struct freq_state {
	int freq;
	/* Calculate the sampling data*/
        short prev2;
        short prev;
        short fac;
};

struct tone {
        int busycount;
        int threshold;
        int ontime;
        int offtime;
        struct freq_state fs[TONE_FREQ_NUM];
	struct proc_dir_entry *subentry;
};

struct silent_detect {
        int detect_tx;
        int detect_rx;
	/* Mute timeout */
        int length;
        int threshold;
	struct proc_dir_entry *subentry;

	/* Statistics mute sample */
	u32 tx_samples;
	u32 rx_samples;	
};

struct param {
        struct tone tone[MAX_TONE_NUM];
        struct silent_detect sd;
};

struct detect_state {
        int index;
        u32 length;
};

struct dsp_info {
	/* The read data cache */
	short rdata[FRAME_SHORT_SIZE];	
	int rlen;
	/* The write data cache */
	short wdata[FRAME_SHORT_SIZE];	
	int wlen;

        struct param param;
	struct proc_dir_entry *entry;

	u32 energy;
        int count;
        int detected_tone;
        struct detect_state state[3];

	/* The busy tone appear */
	int appear_busy;

	/* Calculate parameters for generate waveform */
	/* Read and write two direction */
        int phase[2][TONE_FREQ_NUM];
	int is_offtime[2];
	int offset[2];
};

struct detect_info {
        struct dsp_info dsp[TOTAL_CARDS];

        /* Variable for generater waveform */
        u32 phase_rate[TONE_FREQ_NUM];
        short gain[TONE_FREQ_NUM];
        int ontime;
        int offtime;
};

static struct detect_info *di = NULL;

static DECLARE_BITMAP(fxo_cardflag, TOTAL_CARDS);
static DECLARE_BITMAP(fxs_cardflag, TOTAL_CARDS);

static struct proc_dir_entry *opvxdsp_entry;
static int detect_is_close(int channo)
{
	int i;
	struct param *param = &di->dsp[channo - 1].param;

	for (i = 0; i < MAX_TONE_NUM; i++) {
		if (param->tone[i].busycount > 0) {
			return 0;
		}
	}

	if (param->sd.length > 0 && (param->sd.detect_tx || param->sd.detect_rx)) {
		return 0;
	}

	return 1;
}

static void reset_dsp_detect_param(struct dsp_info *dsp)
{
	int i;

	dsp->param.sd.tx_samples = 0;
	dsp->param.sd.rx_samples = 0;

	dsp->detected_tone = -1;
	dsp->count = 0;
	for (i = 0; i < 3; i++) {
		dsp->state[i].index = -1;
		dsp->state[i].length = 0;
	}
}

static void reset_dsp_generate_param(struct dsp_info *dsp)
{
	int i, j;

	if (dsp->appear_busy) {
		dsp->appear_busy = 0;

		for (i = 0; i < 2; i++) {
			dsp->is_offtime[i] = 0;
			dsp->offset[i] = 0;
			for (j = 0; j < TONE_FREQ_NUM; j++) {
				dsp->phase[i][j] = 0;
			}
		}
	}	
}

static void update_detect(struct dsp_info *dsp, short s)
{
	int i, j;
	short tmp;
	struct freq_state *fs;

	for (i = 0; i < MAX_TONE_NUM; i++) {
		if (dsp->param.tone[i].busycount <= 0) {
			continue;
		}

		for (j = 0; j < TONE_FREQ_NUM; j++) {
			fs = &dsp->param.tone[i].fs[j];
			tmp = fs->prev2;
			fs->prev2 = fs->prev;
			fs->prev = (((int)fs->fac * fs->prev2) >> 14) - tmp + (s >> 7);
		}
	}
}

static u32 detect_result(struct freq_state *fs)
{
	u32 val;

	val = fs->prev * fs->prev + fs->prev2 * fs->prev2 - ((fs->fac * fs->prev) >> 14) * fs->prev2;
	/* Reset */
	fs->prev = fs->prev2 = 0;

	return val;
}

static int busy_detect(struct dsp_info *dsp, int len)
{
	int i, j;
	u32 power, max_power = 0;
	int index = -1;

	for (i = 0; i < MAX_TONE_NUM; i++) {
		power = 0;
		for (j = 0; j < TONE_FREQ_NUM; j++) {
			power += detect_result(&dsp->param.tone[i].fs[j]);
		}

		if (dsp->param.tone[i].busycount > 0 &&
			dsp->energy > dsp->param.tone[i].threshold &&
			power > ENERGY_SCALE * dsp->energy) {
			if (power > max_power) {
				max_power = power;
				index = i;
			}
		}
	}

        if (index != -1 && dsp->detected_tone != index) {
		dsp->detected_tone = index;
		dsp->count = 0;
		for (i = 0; i < 3; i++) {
			dsp->state[i].index = -1;
			dsp->state[i].length = 0;
		}
        }

        if (dsp->state[2].index != index) {
                dsp->state[2].index = index;
                dsp->state[1].length += len;
        } else {
                if (dsp->state[1].index != index) {
                        if (dsp->detected_tone >= 0) {
                                if (dsp->state[0].index == dsp->detected_tone && dsp->state[1].index == -1) {
                                        if ((dsp->state[0].length >= dsp->param.tone[dsp->detected_tone].ontime * 8 - FRAME_SHORT_SIZE) && \
					(dsp->state[0].length <= dsp->param.tone[dsp->detected_tone].ontime * 8 + FRAME_SHORT_SIZE) && \
					(dsp->state[1].length >= dsp->param.tone[dsp->detected_tone].offtime * 8 - FRAME_SHORT_SIZE)) {
                                                dsp->count++;
                                                if (dsp->count >= dsp->param.tone[dsp->detected_tone].busycount) {
                                                        return 1;
                                                }
                                        }
                                }
                        }
                        memmove(&dsp->state[0], &dsp->state[1], 2 * sizeof(dsp->state[0]));
                        dsp->state[1].index = index;
                        dsp->state[1].length = len;
                } else {
                        dsp->state[1].length += len;
                }
        }

	return 0;
}

static int silent_detect(struct dsp_info *dsp, int len, int is_write)
{
	int res = 0;

	if (dsp->param.sd.length <= 0) {
		return res;
	}

	if (dsp->energy < dsp->param.sd.threshold) {
		if (is_write) {
			dsp->param.sd.tx_samples += len;
			if (dsp->param.sd.tx_samples >= dsp->param.sd.length * SAMPLE_PER_SEC) {
				res = 1;
			}
		} else {
			dsp->param.sd.rx_samples += len;
			if (dsp->param.sd.rx_samples >= dsp->param.sd.length * SAMPLE_PER_SEC) {
				res = 1;
			}
		}
	} else {
		dsp->param.sd.tx_samples = 0;
		dsp->param.sd.rx_samples = 0;
	}

	return res;
}

static short calc_amp(u32 *acc, u32 rate, short scale)
{
	u32 phase, step;
	short amp;

	phase = *acc;
	phase >>= 23;
	step = phase & (SIN_DIVISION - 1);
	if ((phase & SIN_DIVISION)) {
		step = SIN_DIVISION - step;
	}

	amp = sin_table[step];
	if ((phase & (2 * SIN_DIVISION))) {
		amp = -amp;
	}

	*acc += rate;
	return (short)(((u32)amp * scale) >> 15);
}

static short generater_amp(struct dsp_info *dsp, int is_write)
{
	int i;
	short amp = 0;
	int index = (is_write == 0) ? 0 : 1;

	dsp->offset[index]++;
	if (!dsp->is_offtime[index]) {
		for (i = 0; i < TONE_FREQ_NUM; i++) {
			if (di->phase_rate[i] > 0) {
				amp += calc_amp(&dsp->phase[index][i], di->phase_rate[i], di->gain[i]);
			}
		}
		if (dsp->offset[index] >= di->ontime * 8) {
			dsp->offset[index] = 0;
			dsp->is_offtime[index] = 1;
		}
	} else {
		if (dsp->offset[index] >= di->offtime * 8) {
			dsp->offset[index] = 0;
			dsp->is_offtime[index] = 0;
		}
	}

	return amp;
}

static void analysis_dsp(struct dsp_info *dsp, short s[], int len, int is_write)
{
	int i, res = 0;
	u16 temp;

        for (i = 0; i < len; i++) {
		temp = (s[i] < 0 ? -s[i] : s[i]) >> 7;
                dsp->energy += temp * temp;
		if (is_write) {
	                update_detect(dsp, s[i]);
		}
        }

	if (is_write) {
		res = busy_detect(dsp, len);

		if (!res && dsp->param.sd.detect_tx) {
			res = silent_detect(dsp, len, 1);
		}
	} else {
		/* Read only support the silent detect */
		if (dsp->param.sd.detect_rx) {
			res = silent_detect(dsp, len, 0);
		}
	}

        dsp->energy = 0;

	if (res) {
		dsp->appear_busy = 1;
		reset_dsp_detect_param(dsp);
	}
}

void parser_busy_silent_process(void* dev_id, int is_write)
{
	int i, j;
//	struct a24xx_dev *wc_dev = &wc->dev;
	struct x200fxo *a200m = dev_id;
	struct dahdi_chan *chan;
	struct dsp_info *dsp;

	if (!di) { 
		/* Initialize this module failed */
		return;
	}

	for (i = 0; i < 2; i++) {
		chan = a200m->chans[i];
//		dsp = &di->dsp[chan->channo - 1];
		dsp = &di->dsp[a200m->chans[i]->chanpos - 1];
//		if (chan->channo > TOTAL_CARDS) {
		if (a200m->chans[i]->chanpos - 1 > TOTAL_CARDS) {
			continue;
		}
		if (!a200m->fxoconfig[i].offhook) {
			reset_dsp_generate_param(dsp);
			continue;
		}

//		if (detect_is_close(chan->channo)) {
		if (detect_is_close(a200m->chans[i]->channo)) {
			/* The busy tone and silence detection has been closed */
			continue;
		}

		if (is_write) {
			if (dsp->appear_busy) {
				for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
					//chan->writechunk[j] = DAHDI_LIN2X(generater_amp(dsp, 1), chan);		
					a200m->chans[i]->writechunk[j] = DAHDI_LIN2X(generater_amp(dsp, 1), chan);		
				}
			} else {
				for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
					//dsp->wdata[dsp->wlen++] = DAHDI_XLAW(chan->writechunk[j], chan);
					dsp->wdata[dsp->wlen++] = DAHDI_XLAW(a200m->chans[i]->writechunk[j], chan);
				}
				if (dsp->wlen == FRAME_SHORT_SIZE) {
					dsp->wlen = 0;
					analysis_dsp(dsp, dsp->wdata, FRAME_SHORT_SIZE, 1);
				}
			}
		} else {
			if (dsp->appear_busy) {
				for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
					chan->readchunk[j] = DAHDI_LIN2X(generater_amp(dsp, 0), chan);		
				}
			} else {
				for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
					dsp->rdata[dsp->rlen++] = DAHDI_XLAW(chan->readchunk[j], chan);
				}
				if (dsp->rlen == FRAME_SHORT_SIZE) {
					dsp->rlen = 0;
					analysis_dsp(dsp, dsp->rdata, FRAME_SHORT_SIZE, 0);
				}
			}
		}
	}
}

static short get_fac(int freq)
{
        if (freq < MIN_INDEX_FREQ) {
                freq = MIN_INDEX_FREQ;
        } else if (freq > MAX_INDEX_FREQ) {
                freq = MAX_INDEX_FREQ;
        }

        return fac_table[(freq - MIN_INDEX_FREQ) / STEP_FREQ];
}

static u32 get_phase_rate(int freq)
{
        return (freq * 65536 / SAMPLE_PER_SEC) * 65536;
}

static short get_gain(int level)
{
        if (level < MIN_INDEX_LEVEL) {
                level = MIN_INDEX_LEVEL;
        } else if (level > MAX_INDEX_LEVEL) {
                level = MAX_INDEX_LEVEL;
        }

        return gain_table[(level - MIN_INDEX_LEVEL) / STEP_LEVEL];
}

static void config_proc_param(struct param *param)
{
	int i, j;

	for (i = 0; i < MAX_TONE_NUM; i++) {
		param->tone[i].busycount = 0;
		param->tone[i].threshold = TONE_THRESHOLD;
		param->tone[i].ontime = 0;
		param->tone[i].offtime = 0;
		for (j = 0; j < TONE_FREQ_NUM; j++) {
			param->tone[i].fs[j].freq = 0;
			param->tone[i].fs[j].fac = 0;
		}
	}
	param->sd.detect_tx = 0;
	param->sd.detect_rx = 0;
	param->sd.length = 30;
	param->sd.threshold = SILENT_THRESHOLD;
}

static void init_detect_info(struct detect_info *di, const char *opermode)
{
	int i, j;
	
	di->ontime = GEN_BUSYTONE_ONTIME;
	di->offtime = GEN_BUSYTONE_OFFTIME;
        di->phase_rate[0] = get_phase_rate(450);
        di->gain[0] = get_gain(-12);

	/* Set default parameters */
	for (i = 0; i < TOTAL_CARDS; i++) {
		config_proc_param(&di->dsp[i].param);

		di->dsp[i].detected_tone = -1;
		for (j = 0; j < 3; j++) {
                	di->dsp[i].state[j].index = -1;
		}
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int read_param_proc(char *buf, char **start, off_t off, int count,
			int *eof, void *data)
{
	int res;
	int *param = (int *)data;

	if (off > 0) {
		/* We have finished to read, return 0 */
		res = 0;
	} else {
		res = sprintf(buf, "%d", *param);
	}

	return res;
}

static int write_param_proc(struct file *filp, const char __user *buf,
			unsigned long count, void *data)
{
	char temp[24];
	int *param = (int *)data;
	int value;
	int len;
	
	len = count > (sizeof(temp) - 1) ? (sizeof(temp) - 1) : count;

	if (copy_from_user(temp, buf, len)) {
		return -EFAULT;
	}

	temp[len] = '\0';

	value = simple_strtoul(temp, NULL, 10);
	if (value >= 0) {
		*param = value;
	}

	return count;
}

static void create_param_proc(const char *name, struct proc_dir_entry *base, void *data)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry(name, 0644, base);
	if (entry) {
		entry->data = data;
		entry->read_proc = read_param_proc;
		entry->write_proc = write_param_proc;
	}
}
#else //LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
int open_param_proc (struct inode *inode, struct file *filp)
{
    
    filp->private_data = PDE_DATA(inode); 

    return 0;
}
static ssize_t read_param_proc (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int res;
	int *param = (int *) filp->private_data;

	if (*f_pos> 0) {
		/* We have finished to read, return 0 */
		res = 0;
	} else {
		res = sprintf(buf, "%d", *param);
	}

	*f_pos += res;
	return res;
}

static ssize_t write_param_proc (struct file *filp, const char __user * buf, size_t count , loff_t *f_pos)
{
	char temp[24];
	int *param = (int *)filp->private_data;
	int value;
	int len;
    
	len = count > (sizeof(temp) - 1) ? (sizeof(temp) - 1) : count;

	if (copy_from_user(temp, buf, len)) {
		return -EFAULT;
	}

	temp[len] = '\0';

	value = simple_strtoul(temp, NULL, 10);
	if (value >= 0) {
		*param = value;
	}
    *f_pos += count;
	return count;
}

static void create_param_proc(const char *name, struct proc_dir_entry *base, void *data)
{

	static const struct file_operations param_proc_fops = {
		.owner = THIS_MODULE,
		.open = open_param_proc,
		.read = read_param_proc,
		.write = write_param_proc,
	};
	proc_create_data(name, 0644, base, &param_proc_fops, data);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int read_param_freq_proc(char *buf, char **start, off_t off, int count,
			int *eof, void *data)
{
	int res;
	struct freq_state *fs = (struct freq_state *)data;

	if (off > 0) {
		/* We have finished to read, return 0 */
		res = 0;
	} else {
		res = sprintf(buf, "%d", fs->freq);
	}

	return res;
}

static int write_param_freq_proc(struct file *filp, const char __user *buf,
			unsigned long count, void *data)
{
	char temp[24];
	struct freq_state *fs = (struct freq_state *)data;
	int value;
	int len;
	
	len = count > (sizeof(temp) - 1) ? (sizeof(temp) - 1) : count;

	if (copy_from_user(temp, buf, len)) {
		return -EFAULT;
	}

	temp[len] = '\0';

	value = simple_strtoul(temp, NULL, 10);
	if (!value || (value >= MIN_INDEX_FREQ && value <= MAX_INDEX_FREQ)) {
		fs->freq = value;
		if (fs->freq) {
			fs->fac = get_fac(fs->freq);
		} else {
			fs->fac = 0;
		}
	}

	return count;
}

static void create_param_freq_proc(const char *name, struct proc_dir_entry *base, void *data)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry(name, 0644, base);
	if (entry) {
		entry->data = data;
		entry->read_proc = read_param_freq_proc;
		entry->write_proc = write_param_freq_proc;
	}
}

#else 
int open_param_freq_proc (struct inode *inode, struct file *filp)
{
    
    filp->private_data = PDE_DATA(inode); 

    return 0;
}
static ssize_t read_param_freq_proc (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int res;
	struct freq_state *fs = (struct freq_state *)filp->private_data;

	if (*f_pos > 0) {
		/* We have finished to read, return 0 */
		res = 0;
	} else {
		res = sprintf(buf, "%d", fs->freq);
	}

    *f_pos += res;

	return res;
}

static ssize_t write_param_freq_proc (struct file *filp, const char __user * buf, size_t count , loff_t *f_pos)
{
	char temp[24];
	struct freq_state *fs = (struct freq_state *)filp->private_data;
	int value;
	int len;
	
	len = count > (sizeof(temp) - 1) ? (sizeof(temp) - 1) : count;

	if (copy_from_user(temp, buf, len)) {
		return -EFAULT;
	}

	temp[len] = '\0';

	value = simple_strtoul(temp, NULL, 10);
	if (!value || (value >= MIN_INDEX_FREQ && value <= MAX_INDEX_FREQ)) {
		fs->freq = value;
		if (fs->freq) {
			fs->fac = get_fac(fs->freq);
		} else {
			fs->fac = 0;
		}
	}
    
    *f_pos += count;
	return count;
}

static void create_param_freq_proc(const char *name, struct proc_dir_entry *base, void *data)
{

    static const struct file_operations param_freq_proc_fops = {
        .owner  = THIS_MODULE,
        .open   = open_param_freq_proc,
        .read   = read_param_freq_proc,
        .write  = write_param_freq_proc,
    };

    proc_create_data(name, 0644, base, &param_freq_proc_fops, data);
}

#endif
/* 
* \brief parameter
* flag: DECLARE_BITMAP structure definition, TOTAL_CARDS length
* is_clean: 0 is to create, 1 is to remove 
*/
static void rebuild_recur_proc(unsigned long *flag, int is_clean)
{
	int i, j, k;
	char temp[24];
	struct param *param;
	struct proc_dir_entry *entry, *subentry;

	if (!opvxdsp_entry) {
		return;
	}

	if (is_clean) {
		for (i = 0; i < TOTAL_CARDS; i++) {
			if (test_bit(i, flag)) {
				entry = di->dsp[i].entry;
				if (entry) {
					param = &di->dsp[i].param;
					for (j = 0; j < MAX_TONE_NUM; j++) {
						subentry = param->tone[j].subentry;
						if (subentry) {
							remove_proc_entry("busycount", subentry);
							remove_proc_entry("threshold", subentry);
							remove_proc_entry("ontime", subentry);
							remove_proc_entry("offtime", subentry);
							for (k = 0; k < TONE_FREQ_NUM; k++) {
								sprintf(temp, "frequency%d", k + 1);
								remove_proc_entry(temp, subentry);
							}

							sprintf(temp, "tone%d", j + 1);
							remove_proc_entry(temp, entry);
						}
					}
					subentry = param->sd.subentry;
					if (subentry) {
						remove_proc_entry("detect_tx", subentry);
						remove_proc_entry("detect_rx", subentry);
						remove_proc_entry("length", subentry);
						remove_proc_entry("threshold", subentry);
						remove_proc_entry("silent_detect", entry);
					}
				
					sprintf(temp, "%d", i + 1);
					remove_proc_entry(temp, opvxdsp_entry);
				}
			}
		}		
	} else {
		for (i = 0; i < TOTAL_CARDS; i++) {
			if (test_bit(i, flag)) {
				sprintf(temp, "%d", i + 1);		
				entry = proc_mkdir(temp, opvxdsp_entry);
				di->dsp[i].entry = entry;
				if (entry) {
					param = &di->dsp[i].param;
					for (j = 0; j < MAX_TONE_NUM; j++) {
						sprintf(temp, "tone%d", j + 1);
						subentry = proc_mkdir(temp, entry);
						param->tone[j].subentry = subentry;
						if (subentry) {
							create_param_proc("busycount", subentry, &param->tone[j].busycount);
							create_param_proc("threshold", subentry, &param->tone[j].threshold);
							create_param_proc("ontime", subentry, &param->tone[j].ontime);
							create_param_proc("offtime", subentry, &param->tone[j].offtime);
							for (k = 0; k < TONE_FREQ_NUM; k++) {
								sprintf(temp, "frequency%d", k + 1);
								create_param_freq_proc(temp, subentry, &param->tone[j].fs[k]);
							}
						}
					}
					subentry = proc_mkdir("silent_detect", entry);
					param->sd.subentry = subentry;
					if (subentry) {
						create_param_proc("detect_tx", subentry, &param->sd.detect_tx);
						create_param_proc("detect_rx", subentry, &param->sd.detect_rx);
						create_param_proc("length", subentry, &param->sd.length);
						create_param_proc("threshold", subentry, &param->sd.threshold);
					}
				}
			}
		}		
	}
}

static void set_chan_cards_bit(void* dev_id)
{
	int i, channo;
	DECLARE_BITMAP(tempflag, TOTAL_CARDS);
	//struct a24xx_dev *wc_dev = &wc->dev;
	struct x200fxo *a200m = dev_id;

	bitmap_zero(tempflag, TOTAL_CARDS);

	for (i = 0; i < 2; i++) {
		channo = a200m->chans[i]->channo;
		if (channo <= TOTAL_CARDS) {
				set_bit(channo - 1, fxo_cardflag);
				set_bit(channo - 1, tempflag);
		}
	}
	
	rebuild_recur_proc(tempflag, 0);
}

static void clear_chan_cards_bit(void* dev_id)
{
	int i, channo;
	DECLARE_BITMAP(tempflag, TOTAL_CARDS);
	//struct a24xx_dev *wc_dev = &wc->dev;
	struct x200fxo *a200m = dev_id;

	bitmap_zero(tempflag, TOTAL_CARDS);

	for (i = 0; i < 2; i++) {
		channo = a200m->chans[i]->channo;
		if (channo <= TOTAL_CARDS) {
				clear_bit(channo - 1, fxo_cardflag);
				set_bit(channo - 1, tempflag);
		}
	}		

	rebuild_recur_proc(tempflag, 1);
}

int init_busydetect(void* dev_id, const char *opermode)
{
	int res = 0;

	if (!ref_count++) {
		bitmap_zero(fxo_cardflag, TOTAL_CARDS);
		bitmap_zero(fxs_cardflag, TOTAL_CARDS);

		di = kzalloc(sizeof(*di), GFP_KERNEL);
		if (!di) {
			printk(KERN_ERR "Not enough memory, FXO200M not support the busy tone and silence detection\n");
			res = -ENOMEM;
			goto out;	
		}

		init_detect_info(di, opermode);

		opvxdsp_entry = proc_mkdir(module_name, NULL);

		printk(KERN_INFO "FXO200M start the busy tone and silence detection\n");	
	}

	set_chan_cards_bit(dev_id);
out:
	return res;
}

void destroy_busydetect(void *dev_id)
{
	if (ref_count) {
		clear_chan_cards_bit(dev_id);

		if (!--ref_count) {
			if (di) {
				remove_proc_entry(module_name, NULL);
				kfree(di);
				printk(KERN_INFO "FXO200M stop the busy tone and silence detection\n");	
			}
		}
	}
}
