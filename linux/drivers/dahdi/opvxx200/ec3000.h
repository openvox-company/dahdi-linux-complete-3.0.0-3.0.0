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
#include "x200_hal.h"

struct vpm450m;

/* From vpm450m */
extern void vpm450m_setec(struct x200 *x2, int channel, int slot_id, int vpm450mlen);
extern void vpm450m_setdtmf(struct vpm450m *vpm450m, int channel, int detvpm450mt, int mute);
extern int vpm450m_getdtmf(struct vpm450m *vpm450m, int *channel, int *tone, int *start);
extern int opvx_vpm_chvpm450mkirq(struct vpm450m *vpm450m);
extern unsigned int get_vpm450m_capacity(void* x2);
extern struct vpm450m *init_vpm450m(struct x200 *x2, int *isalaw, int numchans,const struct firmware *firmware);
extern int init_vpm450m_chan(struct x200 *x2, int channel, int slot_id);
extern void release_vpm450m(struct vpm450m *vpm450m);

#endif

