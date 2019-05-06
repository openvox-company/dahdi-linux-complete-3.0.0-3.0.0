/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * Written by mark.liu<mark.liu@openvox.cn>
 * $Id: ec3000.h 165 2010-12-09 05:38:49Z liuyuan $
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

#ifndef _EC3000_H_
#define _EC3000_H_

#include <linux/firmware.h>

struct ec;

/* From driver */
unsigned int oct_get_reg(void *data, unsigned int reg);
void oct_set_reg(void *data, unsigned int reg, unsigned int val);

/* From vpm450m */
extern void opvx_vpm_setec(struct ec *ec, int channel, int eclen);
extern void opvx_vpm_setdtmf(struct ec *ec, int channel, int detect, int mute);
extern int opvx_vpm_getdtmf(struct ec *ec, int *channel, int *tone, int *start);
extern int opvx_vpm_checkirq(struct ec *ec);
extern unsigned int opvx_vpm_getcapacity(void *wc);

extern struct ec *opvx_vpm_init(void *wc, int *isalaw, int numspans, const struct firmware *firmware);
extern void opvx_vpm_release(struct ec *ec);

#endif

