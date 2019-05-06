/*
 * OpenVox FXO Detect Busy Voice Driver for DAHDI Telephony interface
 *
 * Written by kevin.chen

 * Copyright (C) 2012-2013 OpenVox Communication Co. Ltd,
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
 * Rev 0.30 support new kernel version 3.10.0
 *
 */

#include <linux/proc_fs.h>
#include "busydetect.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#include <linux/seq_file.h>
#endif

static const char *module_name = "opvxdsp";
static int ref_count = 0;

#define MAX_TONE_NUM            3
#define TONE_FREQ_NUM           2

#define GEN_BUSYTONE_ONTIME	500
#define GEN_BUSYTONE_OFFTIME	500

#define TONE_THRESHOLD		20548
#define SILENT_THRESHOLD	2560
#define ENERGY_SCALE		(FRAME_SHORT_SIZE >> 1)

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

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kevin.chen");
MODULE_DESCRIPTION("DAHDI detect busy voice");

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
                    if ((dsp->state[0].length >= dsp->param.tone[dsp->detected_tone].ontime * 8 - FRAME_SHORT_SIZE * 2) && \
                            (dsp->state[0].length <= dsp->param.tone[dsp->detected_tone].ontime * 8 + FRAME_SHORT_SIZE * 2) && \
                            (dsp->state[1].length >= dsp->param.tone[dsp->detected_tone].offtime * 8 - FRAME_SHORT_SIZE * 2)) {
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
        if (!is_write) {
            update_detect(dsp, s[i]);
        }
    }

    if (is_write) {

        if (dsp->param.sd.detect_tx) {
            res = silent_detect(dsp, len, 1);
        }
    } else {
        res = busy_detect(dsp, len);

        /* Read only support the silent detect */
        if (!res && dsp->param.sd.detect_rx) {
            res = silent_detect(dsp, len, 0);
        }
    }

    dsp->energy = 0;

    if (res) {
        dsp->appear_busy = 1;
        reset_dsp_detect_param(dsp);
    }
}

void parser_busy_silent_process(struct a24xx *wc, int is_write)
{
    int i, j;
    struct a24xx_dev *wc_dev = &wc->dev;
    struct dahdi_chan *chan;
    struct dsp_info *dsp;

    if (!di) { 
        /* Initialize this module failed */
        return;
    }

    for (i = 0; i < wc_dev->max_cards; i++) {
        chan = wc->chans[i];
        dsp = &di->dsp[chan->channo - 1];
        if (chan->channo > TOTAL_CARDS) {
            continue;
        }
        if ((wc_dev->modtype[i] != MOD_TYPE_FXO) ||
                (wc_dev->modtype[i] == MOD_TYPE_FXO && !wc_dev->mod[i].fxo.offhook)) {
            reset_dsp_generate_param(dsp);
            continue;
        }

        if (detect_is_close(chan->channo)) {
            /* The busy tone and silence detection has been closed */
            continue;
        }

        if (is_write) {
            if (dsp->appear_busy) {
                for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
                    chan->writechunk[j] = DAHDI_LIN2X(generater_amp(dsp, 1), chan);		
                }
            } else {
                for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
                    dsp->wdata[dsp->wlen++] = DAHDI_XLAW(chan->writechunk[j], chan);
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int write_param_proc(struct file *filp, const char __user *buf,
        unsigned long count, void *data)
#else
static ssize_t write_param_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *pos)
#endif
{
    char temp[24];
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    int *param = (int *)data;
#else
    int *param = PDE_DATA(file_inode(file));
#endif
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
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
#else
static int param_proc_show(struct seq_file *m, void *v)
{   
    int *param = (int *)m->private;

    seq_printf(m, "%d", *param);
    return 0;
}   

static int open_param_proc(struct inode *inode, struct file *file)
{   
    return single_open(file, param_proc_show, PDE_DATA(inode));
}

static struct file_operations proc_param_fops = {
    .open = open_param_proc, 
    .read = seq_read,
    .write = write_param_proc,
    .llseek = seq_lseek,
    .release = single_release,
};  

static void create_param_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    proc_create_data(name, 0644, base, &proc_param_fops, data);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int write_param_freq_proc(struct file *filp, const char __user *buf,
        unsigned long count, void *data)
#else
static ssize_t write_param_freq_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *pos)
#endif
{
    char temp[24];
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    struct freq_state *fs = (struct freq_state *)data;
#else
    struct freq_state *fs = PDE_DATA(file_inode(file));
#endif
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
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
static int param_freq_proc_show(struct seq_file *m, void *v)
{   
    struct freq_state *fs = (struct freq_state *)m->private;

    seq_printf(m, "%d", fs->freq);
    return 0;
}   

static int open_param_freq_proc(struct inode *inode, struct file *file)
{   
    return single_open(file, param_freq_proc_show, PDE_DATA(inode));
}

static struct file_operations proc_param_freq_fops = {
    .open = open_param_freq_proc, 
    .read = seq_read,
    .write = write_param_freq_proc,
    .llseek = seq_lseek,
    .release = single_release,
};  

static void create_param_freq_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    proc_create_data(name, 0644, base, &proc_param_freq_fops, data);
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

static void set_chan_cards_bit(struct a24xx *wc)
{
    int i, channo;
    DECLARE_BITMAP(tempflag, TOTAL_CARDS);
    struct a24xx_dev *wc_dev = &wc->dev;

    bitmap_zero(tempflag, TOTAL_CARDS);

    for (i = 0; i < wc_dev->max_cards; i++) {
        channo = wc->chans[i]->channo;
        if (channo <= TOTAL_CARDS) {
            if (wc_dev->modtype[i] == MOD_TYPE_FXO) {
                set_bit(channo - 1, fxo_cardflag);
                set_bit(channo - 1, tempflag);
            }
        }
    }

    rebuild_recur_proc(tempflag, 0);
}

static void clear_chan_cards_bit(struct a24xx *wc)
{
    int i, channo;
    DECLARE_BITMAP(tempflag, TOTAL_CARDS);
    struct a24xx_dev *wc_dev = &wc->dev;

    bitmap_zero(tempflag, TOTAL_CARDS);

    for (i = 0; i < wc_dev->max_cards; i++) {
        channo = wc->chans[i]->channo;
        if (channo <= TOTAL_CARDS) {
            if (wc_dev->modtype[i] == MOD_TYPE_FXO)	{
                clear_bit(channo - 1, fxo_cardflag);
                set_bit(channo - 1, tempflag);
            }
        }
    }		

    rebuild_recur_proc(tempflag, 1);
}

int init_busydetect(struct a24xx *wc, const char *opermode)
{
    int res = 0;

    if (!ref_count++) {
        bitmap_zero(fxo_cardflag, TOTAL_CARDS);
        bitmap_zero(fxs_cardflag, TOTAL_CARDS);

        di = kzalloc(sizeof(*di), GFP_KERNEL);
        if (!di) {
            printk(KERN_ERR "Not enough memory, A2410P not support the busy tone and silence detection\n");
            res = -ENOMEM;
            goto out;	
        }

        init_detect_info(di, opermode);

        opvxdsp_entry = proc_mkdir(module_name, NULL);

        printk(KERN_INFO "A2410P start the busy tone and silence detection\n");	
    }

    set_chan_cards_bit(wc);
out:
    return res;
}

void destroy_busydetect(struct a24xx *wc)
{
    if (ref_count) {
        clear_chan_cards_bit(wc);

        if (!--ref_count) {
            if (di) {
                remove_proc_entry(module_name, NULL);
                kfree(di);
                printk(KERN_INFO "A2410P stop the busy tone and silence detection\n");	
            }
        }
    }
}
