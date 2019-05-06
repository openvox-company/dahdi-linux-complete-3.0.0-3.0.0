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

#ifndef _BUSYDETECT_H
#define _BUSYDETECT_H

#include "base.h"

#define FRAME_SHORT_SIZE        (DAHDI_CHUNKSIZE * 20)
#define SAMPLE_PER_SEC		8000

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

/* busydetect.c */
void parser_busy_silent_process(struct a24xx *wc, int is_write);

int init_busydetect(struct a24xx *wc, const char *opermode);
void destroy_busydetect(struct a24xx *wc);

/* callerid.c */
void parser_callerid_process(struct a24xx *wc, int cidbuflen, int cidtimeout);
void set_cidstart_desc_from_chan_num(int spanno, int channo, int cid_state);
void set_signal_unknown_from_chan_num(int spanno, int channo);
int is_callerid_disable(int spanno, int channo);
char is_ring_delay_operation(int spanno, int channo);
void reset_parser_variable_from_chan_num(int spanno, int channo);

int init_callerid(struct a24xx *wc);
void destroy_callerid(struct a24xx *wc);

#endif /* _BUSYDETECT_H */
