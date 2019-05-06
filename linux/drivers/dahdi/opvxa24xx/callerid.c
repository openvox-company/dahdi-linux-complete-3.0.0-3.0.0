/*
 * OpenVox Calling Identity Delivery Analysis Driver for DAHDI Telephony interface
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
 * Rev 0.10 add ringdly_flag variable, deal with special caller id case.
 * Rev 0.30 support new kernel version 3.10.0
 *
 */

#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/kmod.h>
#include "base.h"
#include "busydetect.h" /* use sin_table[] */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
#include <linux/seq_file.h>
#endif

static int cid_debug = 0;
module_param(cid_debug, int, 0600);

/* Before using this directory, the directory must had been created */
static const char *module_name = "opvxdsp";

#define MAX_CID_LEN		32
#define DTMF_BLOCK_SIZE		102	
#define MAX_DTMF_DIGITS		64

#define DTMF_TIMEOUT_SAMPLES	(300 * DAHDI_CHUNKSIZE) /* default 300 msec */

/* Support signals for caller id */
enum {
    /* Signal type for caller id fsk */
    CALLERID_BELL202_OR_V23,
    CALLERID_V23JP,
    MAX_FSK_NUM,
    /* Dtmf signal */
    CALLERID_DTMF,
};

/* type for caller id start */
enum {
    CIDSTART_RING = 1,
    CIDSTART_POLARITY,
    CIDSTART_POLARITY_IN,
    CIDSTART_DTMF,
    MAX_CIDSTART,
};

/* Unknown caller id signal */
#define UNKNOWN_CID_SIGNAL	-1

static const char *signal_desc[] = {
    [CALLERID_BELL202_OR_V23] = "bell or v23", 
    [CALLERID_V23JP] = "v23_jp", 
    [CALLERID_DTMF] = "dtmf",
};
static const char *cidstart_desc[] = {
    [CIDSTART_RING] = "ring", 
    [CIDSTART_POLARITY] = "polarity", 
    [CIDSTART_POLARITY_IN] = "polarity_in", 
    [CIDSTART_DTMF] = "dtmf",
};
static const char *unknown_desc = "unknown";

static const int dtmf_fac[] = {27978, 26955, 25700, 24217, 19072, 16324, 13084, 9314};
static const char dtmf_positions[] = "123A" "456B" "789C" "*0#D";

typedef struct freq_state {
    short prev2;
    short prev;
    short fac;	
}freq_state_t;

typedef struct callerid_dtmf {
    freq_state_t row[4];
    freq_state_t col[4];

    int cur_sample;
    u32 energy;
    u8 digit;
    u8 last_hit;

    int timeout;

    /* Analytical results */
    char digits[MAX_DTMF_DIGITS + 1];
    int num;
}__attribute__((packed))callerid_dtmf_t;

typedef struct complex {
    int re;
    int im;
}complex_t;

typedef struct callerid_fsk {
    int baud_rate;
    int baud_frac;
    int sig_present;
    int last_bit;

    int phase_rate[2];
    u32 phase_acc[2];

    complex_t window[2][24];
    complex_t dot[2];	
    int dot_pos;
    int span;

    short last_amp;
    int power;	

    /* Analytical results */
    u8 name[MAX_CID_LEN];
    u8 number[MAX_CID_LEN];

    int bit_pos;
    u8 msg[256];
    int msg_len;
    int continuous_one;
    int val;
}__attribute__((packed))callerid_fsk_t;	

typedef struct gen_wave {
    int bit_no;
    int bit_pos;
    int byte_no;

    int occupied_len;
    int premark_len;
    int postmark_len;

    int msg_len;
    u8 msg[256];

    int baud_frac;
    int baud_rate;

    int scaling;
    int phase_rates[2];
    u32 phase_acc;
}__attribute__((packed))gen_wave_t;

typedef struct callerid {
    /* Initial current signal unknown */
    int cur_sig;
    /* Generate the signal */
    int appear;

    /* DTMF signal analysis */
    callerid_dtmf_t dtmf;
    /* Variety of fsk signal analysis */
    callerid_fsk_t fsk[MAX_FSK_NUM];

    /* Structuration for generator waveform */
    gen_wave_t gw;
}__attribute__((packed))callerid_t;

struct param {
    /* The pointer of signal description from the last detection */
    const char *last_signal;
    /* The pointer of signal description from the current detection */
    const char *detect_signal;
    /* Signal the start of caller id */
    const char *cidstart;
    /* Close the current channel caller id support */
    u8 disable;
};

struct detect_info {
    int channo;
    callerid_t cid;

    /* Initialized to zero do not find any cidstart signal */
    char cidstart_type;

    /*
     * Sometimes time interval is shorter between signal of caller id end and the next 
     * bell before, so unable to provide a complete signal to asterisk 
     * Set flag to postpone ring tones appears
     */
    char ringdly_flag;

    struct param param;
    struct proc_dir_entry *entry;

    struct list_head list;
}__attribute__((packed));

#define MAX_LIST_SPAN	20
static struct list_head di_list[MAX_LIST_SPAN];

static void reset_parser_variable_result(callerid_t *cid);

static const u16 crc16_table[] = {
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

static u16 check_crc16(const u8 *buf, int len)
{
    int i;
    u16 crc = 0;

    for (i = 0; i < len; i++) {
        crc =  (crc >> 8) ^ crc16_table[(crc ^ buf[i]) & 0xff];
    }
    return crc;
}

static int get_phase_rate(int freq)
{
    return (freq * 65536 / SAMPLE_PER_SEC) * 65536;
}

static short calc_amp(u32 acc)
{
    u32 phase, step;
    short amp;

    phase = acc;
    phase >>= 23;
    step = phase & (SIN_DIVISION - 1);
    if ((phase & SIN_DIVISION)) {
        step = SIN_DIVISION - step;
    }

    amp = sin_table[step];
    if ((phase & (2 * SIN_DIVISION))) {
        amp = -amp;
    }
    return amp;
}

static void fsk_put_msg(callerid_fsk_t *fsk, const u8 msg[], int len, int sig_type)
{
    int i, pos;
    int res;

    memset(fsk->name, 0, sizeof(fsk->name));
    memset(fsk->number, 0, sizeof(fsk->number));
    if (sig_type == CALLERID_V23JP) {
        pos = 7;
        for (i = 0; i < msg[6]; i++) {
            if (msg[i + pos] == 0x02) {
                i++;
                res = (msg[i + pos] <= MAX_CID_LEN) ? msg[i + pos] : MAX_CID_LEN;
                memcpy(fsk->number, msg + i + pos + 1, res);
                i += msg[i + pos] + 1;
            } else {
                i++;
                i += msg[i + pos] + 1;
            }
        }
    } else {
        if (msg[0] == 0x80 || msg[0] == 0x82) {
            /* MDMF */
            pos = 2;
            for (i = 0; i < msg[1];) {
                switch (msg[i + pos]) {
                    case 2:
                    case 4:
                        i++;
                        res = (msg[i + pos] <= MAX_CID_LEN) ? msg[i + pos] : MAX_CID_LEN;
                        memcpy(fsk->number, msg + i + pos + 1, res);
                        i += msg[i + pos] + 1;
                        break;
                    case 7:
                    case 8:
                        i++;
                        res = (msg[i + pos] <= MAX_CID_LEN) ? msg[i + pos] : MAX_CID_LEN;
                        memcpy(fsk->name, msg + i + pos + 1, res);
                        i += msg[i + pos] + 1;
                        break;
                    default:
                        i++;
                        i += msg[i + pos] + 1;
                        break;
                }
            }
        } else if (msg[0] == 0x04) {
            /* SDMF */
            if (msg[1] > 8) {
                memcpy(fsk->number, msg + 10, (msg[1] - 8) <= MAX_CID_LEN ? (msg[1] - 8) : MAX_CID_LEN);
            }
        }
    }
}

static void fsk_put_bit(callerid_fsk_t *fsk, int bit, int sig_type)
{
    int i, sum;

    if (fsk->bit_pos == 0) {
        if (!bit) {
            /* Start bit */
            fsk->bit_pos++;
            if (fsk->continuous_one > 10) {
                fsk->msg_len = 0;
            }	
            fsk->continuous_one = 0;
        } else {
            fsk->continuous_one++;
        }
    } else if (fsk->bit_pos <= 8) {
        fsk->val >>= 1;
        if (bit) {
            fsk->val |= 0x80;
        }
        fsk->bit_pos++;
    } else {
        /* Stop bit */
        if (bit && fsk->msg_len < 256) {
            if (sig_type == CALLERID_V23JP) {
                if (fsk->msg_len == 0) {
                    if (fsk->val == 0x90) {
                        fsk->msg[fsk->msg_len++] = (u8)fsk->val;
                    }	
                } else {
                    fsk->msg[fsk->msg_len++] = (u8)fsk->val;
                }
                if (fsk->msg_len >= 11 && fsk->msg_len == ((fsk->msg[6] & 0x7f) + 11)) {
                    if (check_crc16(fsk->msg + 2, fsk->msg_len - 2) == 0) {
                        for (i = 0; i < fsk->msg_len - 2; i++) {
                            fsk->msg[i] &= 0x7f;
                        }
                        fsk_put_msg(fsk, fsk->msg, fsk->msg_len - 2, sig_type);				
                    }
                    fsk->msg_len = 0;
                }
            } else { 
                fsk->msg[fsk->msg_len++] = (u8)fsk->val;
                if (fsk->msg_len >= 3 && fsk->msg_len == (fsk->msg[1] + 3)) {	
                    sum = 0;
                    for (i = 0; i < fsk->msg_len - 1; i++) {
                        sum += fsk->msg[i];
                    }
                    if (256 - (sum & 0xff) == fsk->msg[i]) {
                        fsk_put_msg(fsk, fsk->msg, fsk->msg_len - 1, sig_type);				
                    }
                    fsk->msg_len = 0;
                }
            }
        } /* if (bit && fsk->msg_len < 256) */

        fsk->bit_pos = 0;
        fsk->val = 0;
    }
}

static void analysis_fsk(callerid_fsk_t *fsk, short amp[], int len, int sig_type)
{
    int i, j;
    int dot_pos;
    int hit_bit;
    short x;
    int dot;
    int sum[2];
    complex_t ph;

    dot_pos = fsk->dot_pos;

    for (i = 0; i < len; i++) {
        for (j = 0; j < 2; j++) {
            fsk->dot[j].re -= fsk->window[j][dot_pos].re;
            fsk->dot[j].im -= fsk->window[j][dot_pos].im;

            ph.re = calc_amp(fsk->phase_acc[j] + (1 << 30));
            ph.im = calc_amp(fsk->phase_acc[j]);
            fsk->phase_acc[j] += fsk->phase_rate[j];
            fsk->window[j][dot_pos].re = (ph.re * amp[i]) >> 3;
            fsk->window[j][dot_pos].im = (ph.im * amp[i]) >> 3;

            fsk->dot[j].re += fsk->window[j][dot_pos].re;
            fsk->dot[j].im += fsk->window[j][dot_pos].im;

            dot = fsk->dot[j].re >> 15;
            sum[j] = dot * dot;	
            dot = fsk->dot[j].im >> 15;
            sum[j] += dot * dot;	
        }

        x = (amp[i] >> 1) - fsk->last_amp;
        fsk->power += ((x * x - fsk->power) >> 4);
        fsk->last_amp = amp[i] >> 1;
        if (fsk->sig_present) {
            /* calc result 36380=pow(10.0,(level-14.7)/10.0)*32767.0*32767.0, level=-30 */
            if (fsk->power < 36380) {
                fsk->sig_present = 0;
                fsk->baud_frac = 0;
                continue;
            }
        } else {
            /* calc result 115046=pow(10.0,(level-9.7)/10.0)*32767.0*32767.0, level=-30 */
            if (fsk->power < 115046) {
                fsk->baud_frac = 0;
                continue;
            }

            fsk->sig_present = 1;
            fsk->baud_frac = 0;
            fsk->last_bit = 0;
        }

        hit_bit = (sum[0] < sum[1]);
        if (fsk->last_bit != hit_bit) {
            fsk->last_bit = hit_bit;
            fsk->baud_frac = (SAMPLE_PER_SEC >> 1);
        } 

        fsk->baud_frac += fsk->baud_rate;
        if (fsk->baud_frac >= SAMPLE_PER_SEC) {
            fsk->baud_frac -= SAMPLE_PER_SEC;
            fsk_put_bit(fsk, hit_bit, sig_type);
        }

        if (++dot_pos >= fsk->span) {
            dot_pos = 0;
        }
    }

    fsk->dot_pos = dot_pos;
}

static inline u32 detect_result(freq_state_t *fs)
{
    u32 val;

    val = fs->prev * fs->prev + fs->prev2 * fs->prev2 - ((fs->fac * fs->prev) >> 14) * fs->prev2;
    fs->prev = fs->prev2 = 0;

    return val;
}

static inline void update_detect(freq_state_t *fs, short s) 
{
    short tmp;

    tmp = fs->prev2;
    fs->prev2 = fs->prev;
    fs->prev = (((int)fs->fac * fs->prev2) >> 14) - tmp + (s >> 7);
}

static void get_dtmf_number(callerid_dtmf_t *dtmf, u8 number[])
{
    int i;
    int code;
    char *p = dtmf->digits;

    if (dtmf->num < 2) {
        return;
    }

    if (p[0] == 'B') {
        code = simple_strtoul(p + 1, NULL, 10);
        if (code == 0) {
            /* Unknown caller id number */
            number[0] = 'O';
        } else if (code == 10) {
            /* Private caller id number */
            number[0] = 'P';
        } 
    } else if (p[0] == 'D' && p[2] == '#') {
        if (p[1] == '1') {
            /* Private caller id number */
            number[0] = 'P';
        } else if (p[1] == '2' || p[2] == '3') {
            /* Unknown caller id number */
            number[0] = 'O';
        }
    } else if (p[0] == 'D' || p[0] == 'A') {
        for (i = 1; i < dtmf->num; i++)	{
            if (p[i] == 'C' || p[i] == '#') {
                break;
            }
            if (isdigit(p[i]) && i <= MAX_CID_LEN) {
                number[i - 1] = p[i];
            }
        }
    } else if (isdigit(p[0])) {
        for (i = 0; i < dtmf->num; i++) {
            if (isdigit(p[i]) && i < MAX_CID_LEN) {
                number[i] = p[i];	
            } else {
                break;
            }
        }
    } else {
        /* Unknown caller id number */
        number[0] = 'O';
    }
}

static void analysis_dtmf(callerid_dtmf_t *dtmf, short s[], int len)
{
    int i, j, k;
    int limit;
    short temp;
    int row_energy[4];
    int col_energy[4];
    int best_row, best_col;
    u8 hit;

    for (i = 0; i < len; i = limit) {
        if (len - i >= DTMF_BLOCK_SIZE - dtmf->cur_sample) {
            limit = i + DTMF_BLOCK_SIZE - dtmf->cur_sample;
        } else {
            limit = len;
        }	

        for (j = i; j < limit; j++) {
            temp = (s[j] < 0 ? -s[j] : s[j]) >> 7;
            dtmf->energy += temp * temp;		
            for (k = 0; k < 4; k++) {
                update_detect(&dtmf->row[k], s[j]);
                update_detect(&dtmf->col[k], s[j]);
            }
        }

        dtmf->cur_sample += (limit - i);
        if (dtmf->cur_sample < DTMF_BLOCK_SIZE) {
            break;
        }

        row_energy[0] = detect_result(&dtmf->row[0]);
        col_energy[0] = detect_result(&dtmf->col[0]);
        best_row = 0;
        best_col = 0;
        for (j = 1; j < 4; j++) {
            row_energy[j] = detect_result(&dtmf->row[j]);
            if (row_energy[j] > row_energy[best_row]) {
                best_row = j;
            }
            col_energy[j] = detect_result(&dtmf->col[j]);
            if (col_energy[j] > col_energy[best_col]) {
                best_col = j;
            }
        }

        hit = 0;
        if (row_energy[best_row] >= 10438 &&
                col_energy[best_col] >= 10438 &&
                col_energy[best_col] < row_energy[best_row] * 2 &&
                col_energy[best_col] * 6 > row_energy[best_row]) {
            for (j = 0; j < 4; j++) {
                if ((j != best_row && row_energy[j] * 6 > row_energy[best_row]) ||
                        (j != best_col && col_energy[j] * 6 > col_energy[best_col])) {
                    break;
                }
            }
            if (j >= 4 && ((row_energy[best_row] + col_energy[best_col]) > 42 * dtmf->energy)) {
                hit = dtmf_positions[(best_row << 2) + best_col];
            }
        }

        if (hit) {
            dtmf->timeout = 0;
        } else {
            dtmf->timeout += DTMF_BLOCK_SIZE;
        }

        if (hit != dtmf->digit) {
            if (dtmf->last_hit != dtmf->digit) {
                hit = (hit && hit == dtmf->last_hit) ? hit : 0;
                if (hit) {
                    if (dtmf->num < MAX_DTMF_DIGITS) {
                        dtmf->digits[dtmf->num++] = (char)hit;
                        dtmf->digits[dtmf->num] = '\0';
                    }
                }
                dtmf->digit = hit;
            }
        }
        dtmf->last_hit = hit;
        dtmf->energy = 0;
        dtmf->cur_sample = 0;
    }
}

static const u16 mon_yday[2][13] = {
    /* Normal year */
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
    /* Leap year */
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366},
};

static void get_systime_val(int *mon, int *mday, int *hour, int *min)
{
    struct timeval tv;
    const u16 *ip;
    int days, rem, y;
    int yg;

    do_gettimeofday(&tv);

#define SECS_PER_HOUR	(60 * 60)
#define SECS_PER_DAY	(SECS_PER_HOUR * 24)
    days = tv.tv_sec / SECS_PER_DAY;
    rem = tv.tv_sec % SECS_PER_DAY;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
    extern struct timezone sys_tz;
#endif
    rem += (-sys_tz.tz_minuteswest * 60);
    while (rem < 0) {
        rem += SECS_PER_DAY;
        days--;
    }
    while (rem > SECS_PER_DAY) {
        rem -= SECS_PER_DAY;
        days++;
    }

    *hour = rem / SECS_PER_HOUR;
    rem %= SECS_PER_HOUR;
    *min = rem / 60;

#define IS_LEAP(y) (((y) % 4 == 0) && ((y) % 100 != 0 || (y) % 400 == 0))
#define DIV(a, b) ((a) / (b) - ((a) % (b) < 0))
#define LEAP_BETWEEN(y)	(DIV(y, 4) - DIV(y, 100) + DIV(y, 400))
    y = 1970;
    while (days < 0 || days >= (IS_LEAP(y) ? 366 : 365)) {
        yg = y + DIV(days, 365);	
        days -= ((yg - y) * 365 + LEAP_BETWEEN(yg - 1) - LEAP_BETWEEN(y - 1));
        y = yg;
    }

    ip = mon_yday[IS_LEAP(y)];	
    for (y = 11; days < ip[y]; y--) {
        continue;
    }
    days -= ip[y];

    *mon = y + 1;
    *mday = days + 1;
}

static int build_msg(u8 *msg, const char *number, const char *name)
{
    int i, res;
    int len;
    int mon, mday, hour, min;
    int sum = 0;
    u8 *ptr = msg + 2;

    get_systime_val(&mon, &mday, &hour, &min);
    res = sprintf(ptr, "\001\010%02d%02d%02d%02d", mon, mday, hour, min);
    ptr += res;

    if (strlen(number)) {
        len = strlen(number);
        res = sprintf(ptr, "\002%c", len);
        ptr += res;
        for (i = 0; i < len; i++) {
            ptr[i] = number[i];
        }
        ptr[i] = '\0';
        ptr += len;
    } else {
        res = sprintf(ptr, "\004\001O");
        ptr += res;
    }

    if (strlen(name)) {
        len = strlen(name);
        res = sprintf(ptr, "\007%c", len);
        ptr += res;
        for (i = 0; i < len; i++) {
            ptr[i] = name[i];
        }
        ptr[i] = '\0';
        ptr += len;
    } else {
        res = sprintf(ptr, "\010\001O");
        ptr += res;
    }

    msg[0] = 0x80;
    msg[1] = ptr - msg - 2;

    for (i = 0; i < ptr - msg; i++) {
        sum += msg[i];
    }
    msg[ptr - msg] = 256 - (sum & 0xff);
    ptr++;

    return (ptr - msg);
}

static int fsk_get_bit(gen_wave_t *gw)
{
    int bit;

    if (gw->bit_no < gw->occupied_len) {
        bit = gw->bit_no & 1;
        gw->bit_no++;
    } else if (gw->bit_no < gw->occupied_len + gw->premark_len) {
        bit = 1;
        gw->bit_no++;
    } else if (gw->bit_no == gw->occupied_len + gw->premark_len) {
        if (gw->bit_pos == 0) {
            /* Start bit */
            bit = 0;
            gw->bit_pos++;
        } else if (gw->bit_pos <= 8) {
            bit = (gw->msg[gw->byte_no] >> (gw->bit_pos - 1)) & 1;
            gw->bit_pos++;
        } else {
            /* Stop bit */
            bit = 1;
            gw->bit_pos = 0;
            if (++gw->byte_no >= gw->msg_len) {
                gw->bit_no++;
            }
        }
    } else if (gw->bit_no <= gw->occupied_len + gw->premark_len + gw->postmark_len) {
        bit = 1;
        gw->bit_no++;
    } else {
        /* Completion */
        bit = -1;
    }

    return bit;
}

static short generater_amp(u32 *phase_acc, int phase_rate, int scale)
{
    short amp;

    amp = (short)(((int)calc_amp(*phase_acc) * scale) >> 15);
    *phase_acc += phase_rate;
    return amp;
}

static int fsk_gen(gen_wave_t *gw, short amp[], int len)
{
    int i = 0;
    int bit = 0;
    static int cur_bit = 1;
    int cur_phase_rate;

    cur_phase_rate = gw->phase_rates[cur_bit];
    while (i < len) {
        if ((gw->baud_frac += gw->baud_rate) >= SAMPLE_PER_SEC) {
            gw->baud_frac -= SAMPLE_PER_SEC;
            bit = fsk_get_bit(gw);
            if (bit == -1) {
                /* Completion */
                cur_bit = 1;
                break;
            }
            cur_bit = bit & 1;
            cur_phase_rate = gw->phase_rates[cur_bit];
        }
        amp[i++] = generater_amp(&gw->phase_acc, cur_phase_rate, gw->scaling);
    }

    return bit;
}

static void analysis_all_signal(callerid_t *cid, short s[], int len)
{
    int i;

    analysis_dtmf(&cid->dtmf, s, len);
    for (i = 0; i < MAX_FSK_NUM; i++) {
        analysis_fsk(&cid->fsk[i], s, len, i);
    }
}

static void analysis_current_signal(callerid_t *cid, short s[], int len)
{
    switch (cid->cur_sig) {
        case CALLERID_BELL202_OR_V23:
        case CALLERID_V23JP:
            analysis_fsk(&cid->fsk[cid->cur_sig], s, len, cid->cur_sig);
            break;
        case CALLERID_DTMF:
            analysis_dtmf(&cid->dtmf, s, len);
            break;
        default: /* UNKNOWN_CID_SIGNAL */
            /* signal of caller id is unknown, re-check */
            analysis_all_signal(cid, s, len);
            break;
    }
}

static struct detect_info *get_detect_info_from_chan_num(int spanno, int channo)
{
    struct detect_info *di;

    if (spanno <= 0 || spanno > MAX_LIST_SPAN) {
        return NULL;
    }

    list_for_each_entry(di, &di_list[spanno - 1], list) {
        if (di->channo == channo) {
            /* Find the corresponding matching */
            return di;
        }
    }

    return NULL;
}

static void set_signal_desc(struct detect_info *di)
{
    di->param.last_signal = di->param.detect_signal;

    switch (di->cid.cur_sig) {
        case CALLERID_BELL202_OR_V23:
        case CALLERID_V23JP:
        case CALLERID_DTMF:
            di->param.detect_signal = signal_desc[di->cid.cur_sig];
            break;
        default: /* UNKNOWN_CID_SIGNAL */
            di->param.detect_signal = unknown_desc;
            break;
    }
}

void set_signal_unknown_from_chan_num(int spanno, int channo)
{
    struct detect_info *di;

    di = get_detect_info_from_chan_num(spanno, channo);
    if (di) {
        /* Default standard does not analyze the current signal */
        di->cid.cur_sig = UNKNOWN_CID_SIGNAL;
        set_signal_desc (di);
        /* set the ring delay flag */
        if (di->cid.dtmf.num > 1) {
            di->ringdly_flag = 1;
        } else {
            di->ringdly_flag = di->cid.appear > 0 ? 1 : 0;
        }
    }
}

char is_ring_delay_operation(int spanno, int channo)
{
    struct detect_info *di;

    di = get_detect_info_from_chan_num(spanno, channo);
    if (di) {
        return di->ringdly_flag;
    } else {
        return 0;
    }
}

static void clear_cidstart_type(struct detect_info *di)
{
    di->cidstart_type = 0;
}

static void set_cidstart_desc_force(struct detect_info *di, int type)
{
    if (type >= CIDSTART_RING && type < MAX_CIDSTART) {
        di->cidstart_type = type;
        di->param.cidstart = cidstart_desc[type];
    }
}

static void set_cidstart_desc(struct detect_info *di, int type)
{
    if (!di->cidstart_type) {
        /* Cidstart signal has not yet appeared */
        if (type >= CIDSTART_RING && type < MAX_CIDSTART) {
            di->cidstart_type = type;
            di->param.cidstart = cidstart_desc[type];
        }
    }
}

void set_cidstart_desc_from_chan_num(int spanno, int channo, int cid_state)
{
    int type;
    struct detect_info *di;

    if(cid_state == CID_STATE_IDLE) { 
        type = CIDSTART_POLARITY;
    } else {
        type = CIDSTART_POLARITY_IN;
    }

    di = get_detect_info_from_chan_num(spanno, channo);
    if (di) {
        set_cidstart_desc(di, type);
    }
}

int is_callerid_disable(int spanno, int channo)
{
    int res = 0;
    struct detect_info *di;

    di = get_detect_info_from_chan_num(spanno, channo);
    if (di) {
        res = di->param.disable;
    }

    return res;
}

static void print_cidinfo(u8 *number, u8 *name)
{
    struct timeval tv;
    int mon, mday, hour, min;

    do_gettimeofday(&tv);
    get_systime_val(&mon, &mday, &hour, &min);
    printk(KERN_INFO "cid infor: %02d-%02d %02d:%02d number=%s, name=%s\n", mon, mday, hour, min, number, name);
}

static void check_callerid_signal(struct detect_info *di, int timeout)
{
    int i, res = 0;
    callerid_fsk_t *fsk;
    u8 name[MAX_CID_LEN + 1];
    u8 number[MAX_CID_LEN + 1];
    callerid_t *cid = &di->cid;

    if (cid->appear) {
        /* Last time we have found signal of caller id, directly to exit */
        return;
    }
    memset(name, 0, sizeof(name));
    memset(number, 0, sizeof(number));
    if (cid->dtmf.num > 1) {
        if (cid->dtmf.timeout >= timeout) {
            get_dtmf_number(&cid->dtmf, number);
            cid->cur_sig = CALLERID_DTMF;
            set_cidstart_desc(di, CIDSTART_DTMF);
            res = 1;
        }
    } else {
        for (i = 0; i < MAX_FSK_NUM; i++) {
            fsk = &cid->fsk[i];
            if (strlen(fsk->number)) {
                memcpy(number, fsk->number, MAX_CID_LEN);
                memcpy(name, fsk->name, MAX_CID_LEN);
                if (cid->dtmf.num == 1) {
                    set_cidstart_desc_force(di, CIDSTART_DTMF);
                }
                cid->cur_sig = i;
                res = 1;
                break;
            }
        }
    }

    if (res) {
        if (strlen(number)) {
            cid->gw.msg_len = build_msg(cid->gw.msg, number, name);
            cid->appear = 1;
            set_signal_desc(di);
            if (cid_debug) {
                print_cidinfo(number, name);
            }
        }
    }
}

void reset_parser_variable_from_chan_num(int spanno, int channo)
{
    struct detect_info *di;

    di = get_detect_info_from_chan_num(spanno, channo);
    if (di) {
        reset_parser_variable_result(&di->cid);
    }
}

static int generate_callerid(callerid_t *cid, short s[], int len)
{
    if (fsk_gen(&cid->gw, s, len) == -1) {
        /* Signal of caller id stop transmission */
        reset_parser_variable_result(cid);
        return 0;
    } else {
        return 1;
    }
}

void parser_callerid_process(struct a24xx *wc, int cidbuflen, int cidtimeout)
{
    int i, j;
    /* Analytical work to ensure that within the ring tones */
#define LENGTH_PER_PTR	(10 * DAHDI_CHUNKSIZE)
    short data[LENGTH_PER_PTR];
    struct a24xx_dev *wc_dev = &wc->dev;
    struct dahdi_chan *chan;
    struct detect_info *di;

    for (i = 0; i < wc_dev->max_cards; i++) {
        if (wc_dev->modtype[i] == MOD_TYPE_FXO && !wc_dev->mod[i].fxo.offhook) {
            chan = wc->chans[i];
            if (wc_dev->cid_state[i] == CID_STATE_IDLE ||
                    wc_dev->cid_state[i] == CID_STATE_RING_DELAY) {
                /* We need copy data to the caller id voice buffer */
                memcpy(wc_dev->cid_history_buf[i] + wc_dev->cid_history_ptr[i], chan->readchunk, DAHDI_CHUNKSIZE);
                wc_dev->cid_history_ptr[i] = (wc_dev->cid_history_ptr[i] + DAHDI_CHUNKSIZE)%(cidbuflen * DAHDI_MAX_CHUNKSIZE);
                di = get_detect_info_from_chan_num(wc->span.spanno, chan->channo);
                if (di && !di->param.disable) {
                    /* Empty data to prevent interference */
                    memset(chan->readchunk, DAHDI_LIN2X(0, chan), DAHDI_CHUNKSIZE);
                }
            } else if (wc_dev->cid_state[i] == CID_STATE_RING_ON) {
                di = get_detect_info_from_chan_num(wc->span.spanno, chan->channo);
                if (di) {
                    if (wc_dev->cid_history_clone_cnt[i] > 0) {
                        for (j = 0; j < LENGTH_PER_PTR; j++) {
                            data[j] = DAHDI_XLAW(*(u8 *)(wc_dev->cid_history_buf[i] + wc_dev->cid_history_ptr[i] + j), chan);
                        }
                        analysis_current_signal(&di->cid, data, LENGTH_PER_PTR);
                        wc_dev->cid_history_clone_cnt[i] -= (LENGTH_PER_PTR / DAHDI_CHUNKSIZE);
                        if (wc_dev->cid_history_clone_cnt[i] < 0) {
                            wc_dev->cid_history_clone_cnt[i] = 0;
                        }
                        wc_dev->cid_history_ptr[i] = (wc_dev->cid_history_ptr[i] + LENGTH_PER_PTR)%(cidbuflen * DAHDI_MAX_CHUNKSIZE);
                    } else if (wc_dev->cid_history_clone_cnt[i] == 0) {
                        check_callerid_signal(di, 0);
                        wc_dev->cid_history_clone_cnt[i] = -1;
                    }
                }
            } else if (wc_dev->cid_state[i] == CID_STATE_RING_OFF) {
                di = get_detect_info_from_chan_num(wc->span.spanno, chan->channo);
                if (di) {
                    if (di->cid.appear) {
                        set_cidstart_desc(di, CIDSTART_RING);
                        if (generate_callerid(&di->cid, data, DAHDI_CHUNKSIZE)) {
                            if (!di->param.disable) {
                                /* Create effective caller id signal */
                                for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
                                    chan->readchunk[j] = DAHDI_LIN2X(data[j], chan);
                                }
                            }
                        } else {
                            clear_cidstart_type(di);
                            wc_dev->cid_state[i] = CID_STATE_WAIT_RING_FINISH;
                            wc_dev->cid_history_clone_cnt[i] = cidtimeout;
                        }
                    } else {
                        for (j = 0; j < DAHDI_CHUNKSIZE; j++) {
                            data[j] = DAHDI_XLAW(chan->readchunk[j], chan);
                            if (!di->param.disable) {
                                /* Empty data to prevent repeated parsing */
                                chan->readchunk[j] = DAHDI_LIN2X(0, chan);
                            }
                        }
                        analysis_current_signal(&di->cid, data, DAHDI_CHUNKSIZE);
                        check_callerid_signal(di, DTMF_TIMEOUT_SAMPLES);
                    } /* if (di->cid.appear) */
                }
            } else if (wc_dev->cid_state[i] == CID_STATE_WAIT_RING_FINISH) {
                if (wc_dev->cid_history_clone_cnt[i] > 0) {
                    wc_dev->cid_history_clone_cnt[i]--;
                } else {
                    wc_dev->cid_state[i] = CID_STATE_IDLE;
                }
            }
        } 
    }
}

static void reset_parser_variable_result(callerid_t *cid)
{
    int i;
    callerid_dtmf_t *dtmf;
    callerid_fsk_t *fsk;
    gen_wave_t *gw;

    /* Generate the signal reset */
    cid->appear = 0;

    /* Reset the dtmf analytical variables and results */
    dtmf = &cid->dtmf;
    memset(dtmf, 0, sizeof(*dtmf));
    for (i = 0; i < 4; i++) {
        dtmf->row[i].fac = dtmf_fac[i];
        dtmf->col[i].fac = dtmf_fac[4 + i];
    }

    /* Reset the variety of fsk analytical variables and results */
    for (i = 0; i < MAX_FSK_NUM; i++) {
        fsk = &cid->fsk[i];
        memset(fsk, 0, sizeof(*fsk));
        fsk->baud_rate = 1200;
        fsk->span = SAMPLE_PER_SEC / fsk->baud_rate;
        if (i == CALLERID_BELL202_OR_V23) {
            fsk->phase_rate[0] = get_phase_rate(2200);
            fsk->phase_rate[1] = get_phase_rate(1200);
        } else {
            fsk->phase_rate[0] = get_phase_rate(2100);
            fsk->phase_rate[1] = get_phase_rate(1300);
        }
    }

    /* Reset the waveform analytical variables and results */
    fsk = &cid->fsk[CALLERID_BELL202_OR_V23];
    gw = &cid->gw;
    memset(gw, 0, sizeof(*gw));
    gw->occupied_len = 300;
    gw->premark_len = 180;
    gw->postmark_len = 60;
    gw->scaling = 2873; /* pow(10.0,(level-3.14)/20.0)*32767.0, level=-14 */
    gw->baud_rate = fsk->baud_rate;
    gw->phase_rates[0] = fsk->phase_rate[0];
    gw->phase_rates[1] = fsk->phase_rate[1];
}

static void init_callerid_info(struct detect_info *di, int channo)
{
    /* Save channel number */
    di->channo = channo;

    INIT_LIST_HEAD(&di->list);

    /* Initial description of the parameters are unknown */
    di->param.detect_signal = unknown_desc;
    di->param.last_signal = unknown_desc;
    di->param.cidstart = unknown_desc;

    /* Initial signal is unknown */
    di->cid.cur_sig = UNKNOWN_CID_SIGNAL;

    /* Initialize dtmf, variety of fsk, waveform parameters */
    reset_parser_variable_result(&di->cid);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int read_param_proc(char *buf, char **start, off_t off, int count,
        int *eof, void *data)
{
    int res;
    const char **p = (const char **)data;

    if (off > 0) {
        /* We have finished to read, return 0 */
        res = 0;
    } else {
        res = sprintf(buf, "%s", *p);
    }

    return res;
}

static void create_param_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry(name, 0444, base);
    if (entry) {
        entry->data = (void *)data;
        entry->read_proc = read_param_proc;
    }
}
#else
static int param_proc_show(struct seq_file *m, void *v)
{   
    const char **p = (const char **)m->private;

    seq_printf(m, "%s", *p);
    return 0;
}   

static int open_param_proc(struct inode *inode, struct file *file)
{   
    return single_open(file, param_proc_show, PDE_DATA(inode));
}

static struct file_operations proc_param_fops = {
    .open = open_param_proc, 
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};  

static void create_param_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    proc_create_data(name, 0444, base, &proc_param_fops, data);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int write_param_off_proc(struct file *file, const char __user *buf,
        unsigned long count, void *data)
#else
static ssize_t write_param_off_proc(struct file *file, const char __user *buf,
        size_t count, loff_t *pos)
#endif
{
    char temp[24];
    int newval, len;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    u8 *val = (u8 *)data;
#else
    u8 *val = PDE_DATA(file_inode(file));
#endif

    len = count > (sizeof(temp) - 1) ? (sizeof(temp) - 1) : count;

    if (copy_from_user(temp, buf, len)) {
        return -EFAULT;
    }

    temp[len] = '\0';

    newval = simple_strtoul(temp, NULL, 10);
    *val = newval > 0 ? 1 : 0;

    return count;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int read_param_off_proc(char *buf, char **start, off_t off, int count,
        int *eof, void *data)
{
    int res;
    u8 *val = (u8 *)data;

    if (off > 0) {
        /* We have finished to read, return 0 */
        res = 0;
    } else {
        res = sprintf(buf, "%d", *val);
    }

    return res;
}

static void create_param_off_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    struct proc_dir_entry *entry;

    entry = create_proc_entry(name, 0644, base);
    if (entry) {
        entry->data = data;
        entry->read_proc = read_param_off_proc;
        entry->write_proc = write_param_off_proc;
    }
}
#else
static int param_off_proc_show(struct seq_file *m, void *v)
{   
    u8 *val = (u8 *)m->private;

    seq_printf(m, "%d", *val);
    return 0;
}   

static int open_param_off_proc(struct inode *inode, struct file *file)
{   
    return single_open(file, param_off_proc_show, PDE_DATA(inode));
}

static struct file_operations proc_param_off_fops = {
    .open = open_param_off_proc, 
    .read = seq_read,
    .write = write_param_off_proc,
    .llseek = seq_lseek,
    .release = single_release,
};  

static void create_param_off_proc(const char *name, struct proc_dir_entry *base, void *data)
{
    proc_create_data(name, 0644, base, &proc_param_off_fops, data);
}
#endif

/*
 * \brief parameter
 * is_clean: 0 is to create, 1 is to remove
 */
static void rebuild_callerid_proc(struct detect_info *di, int is_clean)
{
    char temp[24];
    struct proc_dir_entry *entry;

    if (is_clean) {
        entry = di->entry;
        if (entry) {
            remove_proc_entry("last_signal", entry);
            remove_proc_entry("detect_signal", entry);
            remove_proc_entry("cidstart", entry);
            remove_proc_entry("disable", entry);

            sprintf(temp, "%s/%d/opencid", module_name, di->channo);
            remove_proc_entry(temp, NULL);
        }
    } else {
        sprintf(temp, "%s/%d/opencid", module_name, di->channo);
        entry = proc_mkdir(temp, NULL);
        di->entry = entry;
        if (entry) {
            create_param_proc("last_signal", entry, &di->param.last_signal);
            create_param_proc("detect_signal", entry, &di->param.detect_signal);
            create_param_proc("cidstart", entry, &di->param.cidstart);
            create_param_off_proc("disable", entry, &di->param.disable);
        }
    }
}

static void release_callerid_resource(struct a24xx *wc)
{
    int i;
    struct detect_info *cur, *next;
    struct a24xx_dev *wc_dev = &wc->dev;

    if (wc->span.spanno <= 0 || wc->span.spanno > MAX_LIST_SPAN) {
        return;
    }

    for (i = 0; i < wc_dev->max_cards; i++) {
        if (wc_dev->modtype[i] == MOD_TYPE_FXO) {
            list_for_each_entry_safe(cur, next, &di_list[wc->span.spanno - 1], list) {
                if (cur->channo == wc->chans[i]->channo) {
                    /* Find the corresponding matching */
                    list_del(&cur->list);
                    rebuild_callerid_proc(cur, 1);		
                    kfree(cur);
                    break;
                }
            }
        }
    }
}

/* Called after created the top-level parameters directory "module_name" */
int init_callerid(struct a24xx *wc)
{
    int i, res = 0;
    struct detect_info *di;
    struct a24xx_dev *wc_dev = &wc->dev;
    static int first_in = 1;

    if (first_in) {
        first_in = 0;
        for (i = 0; i < MAX_LIST_SPAN; i++) {
            INIT_LIST_HEAD(&di_list[i]);
        }
    }

    if (wc->span.spanno <= 0 || wc->span.spanno > MAX_LIST_SPAN) {
        return -ENXIO;
    }

    for (i = 0; i < wc_dev->max_cards; i++) {
        if (wc_dev->modtype[i] == MOD_TYPE_FXO) {
            di = kzalloc(sizeof(*di), GFP_KERNEL);
            if (!di) {
                printk(KERN_ERR "Not enough memory, A2410P not support the calling identity delivery analysis");
                res = -ENOMEM;
                goto out;
            }

            init_callerid_info(di, wc->chans[i]->channo);
            rebuild_callerid_proc(di, 0);

            list_add(&di->list, &di_list[wc->span.spanno - 1]);
        }
    }

    return 0;
out:
    release_callerid_resource(wc);
    return res;	
}

/* Called before the release of the top-level parameters directory "module_name" */
void destroy_callerid(struct a24xx *wc)
{
    release_callerid_resource(wc);
}
