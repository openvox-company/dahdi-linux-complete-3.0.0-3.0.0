/*
 * OpenVox A24xx FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by MiaoLin<miaolin@openvox.cn>
 * Written by mark.liu<mark.liu@openvox.cn>
 * $Id: private.c 446 2011-05-12 04:01:57Z liuyuan $
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

//#ifdef VPM_SUPPORT
/* ec debug */
extern int ec_debug;
extern int vpmsupport;
//#endif
#define PEDANTIC_OCTASIC_CHECKING

#define ZT_CHUNKSIZE			8
#define ZT_MIN_CHUNKSIZE		ZT_CHUNKSIZE
#define ZT_DEFAULT_CHUNKSIZE	ZT_CHUNKSIZE
#define ZT_MAX_CHUNKSIZE		ZT_CHUNKSIZE

#define MAX_NUM_CARDS 24

#define CARDS_PER_MODULE    4
#define MOD_TYPE_FXS	0
#define MOD_TYPE_FXO	1


/* register base address */
#define REG_BASE        0x00000000
#define PIO_BASE        0x000004e0
#define TDM0_BASE       0x00000400
#define SPI_PCM_BASE    0x000004a0
#define TDM_MEM_BASE    0x00001000
#define PCI_BASE        0x00004000

/* swap memory offset */
#define OPVX_RUN                0x0
#define OPVX_FWREADY            0x1
#define OPVX_FWVERSION          0x2
#define OPVX_DMA_REG            0x3
#define OPVX_ERR_REG            0x4
#define OPVX_IRQ_CNT_LO         0x5
#define OPVX_IRQ_CNT_HI         0x6
#define OPVX_BURST_SIZE         0x7
#define OPVX_BURST_INTERVAL     0x8
#define OPVX_PCI_IRQ_FRQ		0x9
#define OPVX_CARD_MASTER		0xa 
#define OPVX_IRQ_COMMAND   		0xb 
#define OPVX_VPM_PRESENT        0x12
#define V2_OPVX_PIO_DATA        0x1c   
#define OPVX_TEST               0x1f   
#define V2_EC_BASE              0x00000100

/* irq status register */
#define OPVX_IRQ_STATUS (0x40>>2)       /* irq status register */
#define OPVX_IRQ_ENABLE (0x50>>2)       /* irq status register */

/* PIO register offset */
#define OPVX_PIO_DATA   0
#define OPVX_PIO_DIR    1
#define OPVX_PIO_CNTL   2

/* SPI register offset */
#define OPVX_SPI_IN     0
#define OPVX_SPI_OUT    1
#define OPVX_SPI_STATUS 2
#define OPVX_SPI_CNTL   3
#define OPVX_SPI_CS     5
/* ec controller base addr */
#define EC_BASE         0x00000540

#define OPVX_EC_CNTL    0
#define OPVX_EC_DATA    1
#define OPVX_EC_VPM     2

/* echo canceller stuff */
#define BIT_EC_ADDR_STAGE   (1<<0)
#define BIT_EC_CS           (1<<1)
#define BIT_EC_WR           (1<<2)
#define BIT_EC_RD           (1<<3)
#define BIT_EC_ALE          (1<<4)
#define BIT_EC_RDY          (1<<5)
#define BIT_EC_DAS          (1<<6)
#define BIT_EC_IRQ          (1<<7)

#define BIT_EC_PRESENT      (1<<0)

void __opvx_a24xx_setcreg(unsigned long mem32, unsigned int offset, unsigned int reg, unsigned int val)
{
	//printk("writing offset %d, reg %d at %d, value %d\n", offset, reg, offset + (reg<<2), val);
	unsigned int *p = (unsigned int*)(mem32 + offset + (reg<<2));
	*p = val;
}

unsigned int __opvx_a24xx_getcreg(unsigned long mem32, unsigned int offset, unsigned char reg)
{
	//printk("reding offset %d, reg %d at %d\n", offset, reg, offset + (reg<<2));
	volatile unsigned int *p = (unsigned int*)(mem32 + offset + (reg<<2));
	return (*p);
}

unsigned char __opvx_a24xx_read_8bits(unsigned long mem32)
{
	unsigned int res=0;

	while((__opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_STATUS)&0x40)!=0x40);
	__opvx_a24xx_setcreg(mem32, SPI_PCM_BASE, OPVX_SPI_OUT, 0);                              /* we have to write something so the spi can work */
	while( (__opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_STATUS)&0x80) != 0x80);       /* wait rx finish */
	res = __opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_IN);

	return res&0xff;
}

void __opvx_a24xx_start_dma(unsigned long mem32, unsigned int data)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_DMA_REG, data);
}

void __opvx_a24xx_stop_dma(unsigned long mem32)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_DMA_REG, 0xffffffff); // -1 means stop dma.
}

void __opvx_a24xx_restart_dma(unsigned long mem32)
{
	/* Reset Master and TDM */
	// TODO: do our work here.
}

void __opvx_a24xx_reset_tdm(unsigned long mem32)
{
	/* Reset TDM */
	//TODO: do our work here;
}

void __opvx_a24xx_set_irq_frq(unsigned long mem32,unsigned int frq)
{
		__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_PCI_IRQ_FRQ,frq);
}
void __opvx_a24xx_set_master(unsigned long mem32,unsigned int master)
{
		__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_CARD_MASTER,master);
}
unsigned int __opvx_a24xx_get_master(unsigned long mem32)
{
		return __opvx_a24xx_getcreg(mem32, REG_BASE, OPVX_CARD_MASTER);
}

unsigned int __opvx_a24xx_get_version(unsigned long mem32) 
{
	return __opvx_a24xx_getcreg(mem32, REG_BASE, OPVX_FWVERSION);
}

unsigned int __opvx_a24xx_get_irqstatus(unsigned long mem32) 
{
	return __opvx_a24xx_getcreg(mem32, PCI_BASE, OPVX_IRQ_STATUS);
}

void __opvx_a24xx_set_irqstatus(unsigned long mem32, unsigned int value) 
{
	__opvx_a24xx_setcreg(mem32, PCI_BASE, OPVX_IRQ_STATUS, value);    // clear interrupt register.
}

void __opvx_a24xx_clear_irqs(unsigned long mem32) 
{
	__opvx_a24xx_setcreg(mem32, PCI_BASE, OPVX_IRQ_STATUS, 0xffffffff);      /* clear all pending irqs */
	__opvx_a24xx_setcreg(mem32, SPI_PCM_BASE, OPVX_SPI_CNTL, 0);             /* init spi port */
}

void __opvx_a24xx_enable_interrupts(unsigned long mem32)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_BURST_INTERVAL, 0);
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_BURST_SIZE, 2);
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_RUN, 1);
}

void __opvx_a24xx_disable_interrupts(unsigned long mem32)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_RUN, 0);
}

unsigned int __opvx_a24xx_get_irqcnt_lo(unsigned long mem32)
{
	return __opvx_a24xx_getcreg(mem32, REG_BASE, OPVX_IRQ_CNT_LO);
}

void __opvx_a24xx_reset_modules(unsigned long mem32, void (*func)(int), int data)
{
	__opvx_a24xx_setcreg(mem32, PIO_BASE, OPVX_PIO_DIR, 0xffffffff);     /* all io as output */
	__opvx_a24xx_setcreg(mem32, PIO_BASE, OPVX_PIO_CNTL, 0);             /* disable irq */

//	if(debug) {
//		printk("opvxa24xx: raise reset\n");
//	}

	__opvx_a24xx_setcreg(mem32, PIO_BASE, OPVX_PIO_DATA, 0x1);           /* GPIO0 As reset*/
	/* Wait for 1 second */

	(*func)(data/2);                                      /* delay 1/2 sec */

	__opvx_a24xx_setcreg(mem32, PIO_BASE, OPVX_PIO_DATA, 0x0);           /* GPIO0 As reset*/
//	if(debug) {
//		printk("opvxa24xx: pull down reset\n");
//	}

	(*func)(data/2);                                      /* delay 1/2 sec */

	__opvx_a24xx_setcreg(mem32, PIO_BASE, OPVX_PIO_DATA, 0x1);           /* GPIO0 As reset*/
//	if(debug) {
//		printk("opvxa24xx: raise reset finally\n");
//	}

	(*func)(data/2);                                      /* delay 1/2 sec */
}


void __opvx_a24xx_write_8bits(unsigned long mem32, unsigned char bits)
{
	volatile unsigned int t;

	while((__opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_STATUS)&0x40)!=0x40);
	__opvx_a24xx_setcreg(mem32, SPI_PCM_BASE, OPVX_SPI_OUT, bits);                           /* we have to write something so the spi can work */
	while( (__opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_STATUS)&0x40) != 0x40);       /* wait tx finish */
	t = __opvx_a24xx_getcreg(mem32, SPI_PCM_BASE, OPVX_SPI_IN);
}

static inline void __reset_spi(void *wc_dev)
{
	return;     /* we do nothing here */
}


void __opvx_a24xx_setcard(unsigned long mem32, int card)
{
	__opvx_a24xx_setcreg(mem32, SPI_PCM_BASE, OPVX_SPI_CS, 1<<(card/CARDS_PER_MODULE));
}

void __opvx_a24xx_reset_spi(void *wc_dev, int card, void (*func)(void*, int))
{
	(*func)(wc_dev, card);
	__reset_spi(wc_dev);
	__reset_spi(wc_dev);
}

static inline int __opvx_a24xx_hit_fxo_daisy(int number_daisy)
{
	int cid;

	if (number_daisy==0) {
		cid=0;
	} else if (number_daisy==1) {
		cid=0x8;
	} else if (number_daisy==2) {
		cid=0x4;
	} else if (number_daisy==3) {
		cid=0xc;
	} else {
		cid= -1;
	}

	return cid;
}

void __opvx_a24xx_spi_setreg(void *wc_dev, unsigned long mem32, int card, int modtype, unsigned char reg, unsigned char value, void (*func)(void*, int))
{
	(*func)(wc_dev, card);
	if (modtype == MOD_TYPE_FXO) {
		__opvx_a24xx_write_8bits(mem32, 0x20 | __opvx_a24xx_hit_fxo_daisy(card%CARDS_PER_MODULE));  // fxo daisy operate.
		__opvx_a24xx_write_8bits(mem32, reg & 0x7f);
	} else {
		__opvx_a24xx_write_8bits(mem32, 1<<(card%CARDS_PER_MODULE));  // fxs daisy operate.
		__opvx_a24xx_write_8bits(mem32, reg & 0x7f);
	}
	__opvx_a24xx_write_8bits(mem32, value);
}

unsigned char __opvx_a24xx_spi_getreg(void *wc_dev, unsigned long mem32, int card, int modtype, unsigned char reg, void (*func)(void*, int))
{
	(*func)(wc_dev, card);
	if (modtype == MOD_TYPE_FXO) {
		__opvx_a24xx_write_8bits(mem32, 0x60 | __opvx_a24xx_hit_fxo_daisy(card%CARDS_PER_MODULE));  // fxo daisy operate.
		__opvx_a24xx_write_8bits(mem32, reg & 0x7f);
	} else {
		__opvx_a24xx_write_8bits(mem32, 1<<(card%CARDS_PER_MODULE));  // fxs daisy operate.
		__opvx_a24xx_write_8bits(mem32, reg | 0x80);
	}
	return __opvx_a24xx_read_8bits(mem32);
}


static inline void __a24xx_raw_oct_out(unsigned long mem32, const unsigned int addr, const unsigned int value)
{
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE | BIT_EC_WR);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE | BIT_EC_WR | BIT_EC_ALE);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_DATA, value);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, BIT_EC_WR | BIT_EC_ALE | BIT_EC_CS);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, 0);
}

static inline unsigned int __a24xx_raw_oct_in(unsigned long mem32, const unsigned int addr)
{
	unsigned int ret;
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE | BIT_EC_WR);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE | BIT_EC_WR | BIT_EC_ALE);
#ifdef PEDANTIC_OCTASIC_CHECKING
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, (addr<<16) | BIT_EC_ADDR_STAGE | BIT_EC_ALE);
#endif
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, BIT_EC_RD | BIT_EC_ALE | BIT_EC_CS);
	ret = __opvx_a24xx_getcreg(mem32, EC_BASE, OPVX_EC_DATA);
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_CNTL, 0);

	return ret&0xffff;
}

unsigned int __opvx_a24xx_oct_in(unsigned long mem32, unsigned int addr)
{
#ifdef PEDANTIC_OCTASIC_CHECKING
	int count = 1000;
#endif
	__a24xx_raw_oct_out(mem32, 0x0008, (addr >> 20));
	__a24xx_raw_oct_out(mem32, 0x000a, (addr >> 4) & ((1 << 16) - 1));
	__a24xx_raw_oct_out(mem32, 0x0000, (((addr >> 1) & 0x7) << 9) | (1 << 8) | (1));
#ifdef PEDANTIC_OCTASIC_CHECKING
	while((__a24xx_raw_oct_in(mem32, 0x0000) & (1 << 8)) && --count);
	if (count != 1000) {
//		printk("Yah, read can be slow...\n");
	}
	if (!count) {
//		printk("Read timed out!\n");
	}
#endif
	return __a24xx_raw_oct_in(mem32, 0x0004);
}

void __opvx_a24xx_oct_out(unsigned long mem32, unsigned int addr, unsigned int value)
{
#ifdef PEDANTIC_OCTASIC_CHECKING
	int count = 1000;
#endif
	__a24xx_raw_oct_out(mem32, 0x0008, (addr >> 20));
	__a24xx_raw_oct_out(mem32, 0x000a, (addr >> 4) & ((1 << 16) - 1));
	__a24xx_raw_oct_out(mem32, 0x0004, value);
	__a24xx_raw_oct_out(mem32, 0x0000, (((addr >> 1) & 0x7) << 9) | (1 << 8) | (3 << 12) | 1);
#ifdef PEDANTIC_OCTASIC_CHECKING
	while((__a24xx_raw_oct_in(mem32, 0x0000) & (1 << 8)) && --count);
	if (count != 1000) {
//		printk("Yah, write can be slow\n");
	}
	if (!count) {
//		printk("Write timed out!\n");
	}
#endif
}

int __opvx_a24xx_check_vpm(unsigned long mem32)
{
	unsigned int check1, check2;
	
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_VPM, 0);    //disable vpm support at first.

	if (!vpmsupport) {
//		printk("OpenVox VPM: Support Disabled\n");
		return -1;
	}

	__a24xx_raw_oct_out(mem32, 0x000a, 0x5678);
	__a24xx_raw_oct_out(mem32, 0x0004, 0x1234);
	check1 = __a24xx_raw_oct_in(mem32, 0x0004);
	check2 = __a24xx_raw_oct_in(mem32, 0x000a);

//	if (ec_debug) {
//		printk("OCT Result: %04x/%04x\n", __a24xx_raw_oct_in(mem32, 0x0004), __a24xx_raw_oct_in(mem32, 0x000a));
//	}
	
	if (__a24xx_raw_oct_in(mem32, 0x0004) != 0x1234) {
//		printk("OpenVox VPM: Not Present\n");
		return -2;
	}

	return 0;
}

void __opvx_a24xx_vpm_setpresent(unsigned long mem32)
{
	__opvx_a24xx_setcreg(mem32, EC_BASE, OPVX_EC_VPM, BIT_EC_PRESENT);
}

void __opvx_a24xx_set_chunk(void *readchunk, void *writechunk,unsigned int frq,int buf_mult) 
{
	unsigned char *tmp;
	tmp =  *((unsigned char **)(writechunk)) + frq * ZT_MAX_CHUNKSIZE * (MAX_NUM_CARDS) * 2;	/* in bytes */
	*(char **)readchunk = tmp;
}

void __opvx_a24xx_transmit(unsigned long mem32, volatile unsigned char *writechunk, volatile unsigned char **txbuf,unsigned int irq_frq , unsigned int order)
{
	unsigned int int_cnt_lo = __opvx_a24xx_get_irqcnt_lo(mem32);
	
	if (int_cnt_lo & 0x01) {
		/* Write is at interrupt address.  Start writing from normal offset */
		*txbuf = writechunk + ZT_CHUNKSIZE * MAX_NUM_CARDS * order;
	} else {
		*txbuf = writechunk + ZT_CHUNKSIZE * MAX_NUM_CARDS * irq_frq + ZT_CHUNKSIZE * MAX_NUM_CARDS * order;
	}
}

void __opvx_a24xx_receive(unsigned long mem32, volatile unsigned char *readchunk, volatile unsigned char **rxbuf,unsigned int irq_frq , unsigned int order)
{	
	unsigned int int_cnt_lo = __opvx_a24xx_get_irqcnt_lo(mem32);
	
	if (int_cnt_lo & 0x01) {
		/* Read is at interrupt address.  Valid data is available at normal offset */
		*rxbuf = readchunk + ZT_CHUNKSIZE * MAX_NUM_CARDS * order;
	} else {
		*rxbuf = readchunk + ZT_CHUNKSIZE * MAX_NUM_CARDS * irq_frq + ZT_CHUNKSIZE * MAX_NUM_CARDS * order;
	}
}

void __opvx_a24xx_reset_modules_v2(unsigned long mem32, void (*func)(int), int data)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, V2_OPVX_PIO_DATA, 0x02020200);          /* gpio bit[1] set to 1*/
	(*func)(data/2);                                                                /* delay 1/2 sec */
	__opvx_a24xx_setcreg(mem32, REG_BASE, V2_OPVX_PIO_DATA, 0x02020000);          /* gpio bit[1] set to 0*/
	(*func)(data/2);                                                                /* delay 1/2 sec */
	__opvx_a24xx_setcreg(mem32, REG_BASE, V2_OPVX_PIO_DATA, 0x02020200);          /* gpio bit[1] set to 1*/
	(*func)(data/2);                                                                /* delay 1/2 sec */
}

unsigned int __opvx_a24xx_oct_in_v2(unsigned long mem32, unsigned int addr)
{
	int count = 1000;
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0008, (addr >> 20));
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x000a, (addr >> 4) & ((1 << 16) - 1)); 
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0000, (((addr >> 1) & 0x7) << 9) | (1 << 8) | (1));
	while((__opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x0000) & (1 << 8)) && --count);
	if (count != 1000) {
//		printk("Yah, read can be slow...\n");
	}
	if (!count) {
//		printk("Read timed out!\n");
	}
	return __opvx_a24xx_getcreg(mem32,V2_EC_BASE, 0x0004);
}


void __opvx_a24xx_oct_out_v2(unsigned long mem32, unsigned int addr, unsigned int value)
{
	int count = 1000;
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0008, (addr >> 20));
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x000a, (addr >> 4) & ((1 << 16) - 1));
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0004, value);
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0000, (((addr >> 1) & 0x7) << 9) | (1 << 8) | (3 << 12) | 1);
	while((__opvx_a24xx_getcreg(mem32,V2_EC_BASE, 0x0000) & (1 << 8)) && --count);
	if (count != 1000) {
//		printk("Yah, write can be slow\n");
	}
	if (!count) {
//		printk("Write timed out!\n");
	}
}

int __opvx_a24xx_check_vpm_v2(unsigned long mem32)
{
	unsigned int check1, check2;
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_VPM_PRESENT, 0);    //disable vpm support at first.

	if (!vpmsupport) {
//		printk("OpenVox VPM: Support Disabled\n");
		return -1;
	}
	
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x000a, 0x5678);
	__opvx_a24xx_setcreg(mem32, V2_EC_BASE, 0x0004, 0x1234);
	check1 = __opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x0004);
	check2 = __opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x000a);

//	if (ec_debug) {
//		printk("OCT Result: %04x/%04x\n", __opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x0004), __opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x000a));
//	}
	
	if (__opvx_a24xx_getcreg(mem32, V2_EC_BASE, 0x0004) != 0x1234) {
//		printk("OpenVox VPM: Not Present\n");
		return -2;
	}
	return 0;
}

void __opvx_a24xx_vpm_setpresent_v2(unsigned long mem32)
{
	__opvx_a24xx_setcreg(mem32, REG_BASE, OPVX_VPM_PRESENT, BIT_EC_PRESENT);

}
