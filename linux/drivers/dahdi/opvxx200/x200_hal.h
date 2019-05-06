
#ifndef _X200_HAL_H
#define _X200_HAL_H


/* hardware architecture descrption *******************************************************************


  +---------------------------------------+
  | blade                                 |
  |                                       |\
  |  |--------|        |--------|         | \
  |  | card   |        | card   |         |  \
  |  |        |        |        |         |   \
  |  |        |        |        |         |    \
  |  |--------|        |--------|         |  B  \
  |                                       |\  U  \
  |_______________________________________+ \  S  \
                                             \     \
                                              \     \
              +---------------------------------------+
              | blade                           \     |
              |                                  \    |
              |                                   \   |
              |                                    \  |
              |                                     \ |
              |                                       |
              |                                       |
              |                                       |
              |_______________________________________+

*******************************************************************************************************/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <dahdi/kernel.h>
#include <dahdi/version.h>


#define VERSION_CODE(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define DAHDI_VERSION_CODE 	VERSION_CODE(2,11,1)

//controller address
#define X200_SPI_BASE		            0x0500
#define V2_EC_BASE                  0x0800
#define X200_PCI_BASE		            0x4000
#define X200_PIO_BASE		            0x0440
#define X200_TDM_BASE	              0x2000
#define X200_TSI_EC_BASE	          (0x2000+0x800)
#define X200_TDM_MEM_BASE	          (0x2000+0x1000)
#define X200_REG_BASE		            0x0

#define BIT_BUS_RESET                   0x0001  // bit 0
#define BIT_TDMCLK_SEL                  0x0002  // bit 1
#define BIT_FPGA_LED                    0x0004  // bit 2
//#define BIT_EC_RESET                    0x0008  // bit 3
#define BIT_EC_PRESENT                  0x0010  // bit 4
#define BIT_TEST_1                      0x0020  // bit 5  test_1
#define BIT_TEST_2                      0x0040  // bit 6  test_2
#define BIT_CONFIG_N                    0x0080  // bit 7  config_n
#define BIT_FSC_MOD_0                   0x0100  // bit 8  tdm_fsc_mod_0
#define BIT_FSC_MOD_1                   0x0200  // bit 9  tdm_fsc_mod_1
#define BIT_TEST_3                      0x0400  // bit 10  0:use fsc generated from local 8M osc, 1 use back(from card) fsc.
#define BIT_TEST_4                      0x0800  // bit 11  0:use local 8M as tdm_clk, 1 use back(from card) tdm_clk.
#define BIT_TEST_5                      0x1000  // bit 12  test_5
#define BIT_TEST_6                      0x2000  // bit 13  test_6
#define BIT_TEST_7                      0x4000  // bit 14  test_7
#define BIT_TEST_8                      0x8000  // bit 15  test_8

#define PIO_REG0_IO  		            0x0
#define PIO_REG1_REG		            0x1
#define PIO_REG2_TIMER_RESET		    0x2
#define PIO_REG3_TIMER_INCREMENT        0x3
#define PIO_REG4_DEBUG_POINT            0x4
#define PIO_REG5_DEBUG_INFO             0x5
#define PIO_REG6_LEDS                   0x6

/* SPI register offset */
#define OPVX_SPI_IN                     0
#define OPVX_SPI_OUT                    1
#define OPVX_SPI_STATUS                 2
#define OPVX_SPI_CNTL                   3
#define OPVX_SPI_CS                     5
#define OPVX_SPI_MODE                   7

/* swap memory offset */
#define OPVX_REG_RUN                    0x00
#define OPVX_REG_FWREADY                0x01
#define OPVX_REG_FWVERSION              0x02
#define OPVX_REG_DMA_HOST_TO_NIOS       0x03
#define OPVX_REG_ERR_CNT                0x04
#define OPVX_REG_IRQ_CNT_LO             0x05
#define OPVX_REG_IRQ_CNT_HI             0x06
#define OPVX_REG_MS_PER_IRQ             0x07
#define OPVX_REG_MAX_BUF_NO             0x08
#define OPVX_REG_BUF_INDEX              0x09
#define OPVX_REG_DMA_NIOS_TO_HOST       0x0A
#define OPVX_TDM_MODE                   0x0B
#define OPVX_TDM_DAT_LEN                0x0C
#define OPVX_HW_ERR_FLAG                0x10

/* irq status register */
#define OPVX_REG_IRQ_STATUS             (0x40>>2)       /* irq status register */
#define OPVX_REG_IRQ_ENABLE             (0x50>>2)       /* irq status register */


#define MAX_BLADE_NUM			        8			/* max blades on the bus */
#define MAX_CARDS_PER_BLADE	            4			/* max modules per blade */
#define MAX_TDM_SLOTS                   128
#define LED_PER_BLADE                   12

/* cs for spi on the bus */
#define X200_CS_BROADCAST               0x80
#define X200_CS_BLADE                   0x40    /* bit6=1, blade reg selected; bit6=0, spi reg selected(cs set before); */
#define X200_CS_CARD                    0x20
#define X200_CS_ADDR                    0x1f

#define X200_CMD_READ_BLADE_REG         0x20        /* [7:5]=001 */
#define X200_CMD_WRITE_BLADE_REG        0x40        /* [7:5]=010 */

#define X200_BLADE_REG_VERSION          0
#define X200_BLADE_REG_FLAG             1
#define X200_BLADE_REG_RESET            2
#define X200_BLADE_REG_CLK_SEL          3
#define X200_BLADE_REG_FSC_MOD          7   //0 for D card , 1 for O/S/B card


#define X200_BLADE_TAG                  0xa1    /* content of flag register in x200 blade controller */


#define DEV_TYPE_NONE									 0x0
#define DEV_TYPE_D100M                 0x1
#define DEV_TYPE_B200M                 0x2
#define DEV_TYPE_FXO200M               0x3
#define DEV_TYPE_FXS200M               0x4

#define X200_DRIVER_FLAG_HAVE_IRQ       0x0001  /* this board will generate special irq except tdm transfer */
#define X200_DRIVER_FLAG_CLK_MST        0x0002  /* this board can export clock and be clock master */
#define X200_DRIVER_FLAG_SYNC_MST       0x0004  /* this board have to be FSYNC master when in clock master model */


#define X200_LED_TYPE_GREEN             0
#define X200_LED_TYPE_RED               1
#define LED_OFF    (0)
#define LED_RED    (1)
#define LED_GREEN  (2)
#define LED_YELLOW (3)


#define X200_MAX_CLOCK_SOURCE           8

/* Bit Position definition */
#define	BIT0				1
#define	BIT1				(1 << 1)
#define	BIT2				(1 << 2)
#define	BIT3				(1 << 3)
#define	BIT4				(1 << 4)
#define	BIT5				(1 << 5)
#define	BIT6				(1 << 6)
#define	BIT7				(1 << 7)
#define	BIT8				(1 << 8)
#define	BIT9				(1 << 9)
#define	BIT10				(1 << 10)
#define	BIT11				(1 << 11)
#define	BIT12				(1 << 12)
#define	BIT13				(1 << 13)
#define	BIT14				(1 << 14)
#define	BIT15				(1 << 15)
#define	BIT16				(1 << 16)
#define	BIT17				(1 << 17)
#define	BIT18				(1 << 18)
#define	BIT19				(1 << 19)
#define	BIT20				(1 << 20)
#define	BIT21				(1 << 21)
#define	BIT22				(1 << 22)
#define	BIT23				(1 << 23)
#define	BIT24				(1 << 24)
#define	BIT25				(1 << 25)
#define	BIT26				(1 << 26)
#define	BIT27				(1 << 27)
#define	BIT28				(1 << 28)
#define	BIT29				(1 << 29)
#define	BIT30				(1 << 30)
#define	BIT31				(1 << 31)

#define CMD_RD_DATA         0x80
#define CMD_WR_ADDR         0x40    //write register address to chip

#define WC_MAX_INTERFACES       8

struct x200;
struct x200_dev;

struct ints_ops {
		int			(*irq_handler)(int int_count, struct x200_dev *dev);	/* interrupt handler */
    int     (*get_clk_src)(struct x200_dev *dev, int *srcpos, int *port);     
    int     (*set_clk_src)(struct x200_dev *dev, int syncpos, int port);     
    int     (*unset_clk_src)(struct x200_dev *dev);     
};

/* data structure for each kind of device driver on x200 bus */
struct x200_blade {
    struct x200* parent;
    int blade_id;           /* if -1 means no blade on this position */
    int exist;              /* 0 means no such blade */
    unsigned int	cardflag;						/* bit-position 1 = init done, 0 = to init */
    unsigned char tdm_fsc_mod;		    /*  1 = fxs200m/fxo200m/b200m card, 0 = d100m card */
    int ver;
    unsigned char reg_reset;
    int max_cards;      /* how many cards at the max can be install in this blade */
};

/* data structure for each card(device) on x200 bus */
struct x200_dev {
    struct  x200_blade  *parent;
    struct  x200        *bus;
    int     bus_id;
    int     blade_id;
	  int     slot_id;
    int     dev_type;
    int     registered;
    struct  device   dev;
    void		*drv_data;
    struct  ints_ops *ops;
};


/* data structure for x200 voiceboard */
struct x200 {
	struct pci_dev *dev;
	
    int bus_id;
	spinlock_t lock;
	unsigned int intcount;
	char *variety;
	unsigned int flag;
	int clocktimeout;
	int sync;
    int checktiming;    
	int dead;
    int selected_blade;
    unsigned char reg_cs;
    int ledreg;
    unsigned int last_chunk_index;
    volatile unsigned char *current_writechunk;
    volatile unsigned char *current_readchunk;

	unsigned long mem_region;	/* 32 bit Region */
	unsigned long mem_len;		/* Length of 32 bit region */
	volatile unsigned long mem32;	/* Virtual representation of 32 bit memory area */

	unsigned int dma_buf_size;
	dma_addr_t 	readdma;
	dma_addr_t	writedma;
	volatile unsigned char *writechunk;					/* Double-word aligned write memory */
	volatile unsigned char *readchunk;					/* Double-word aligned read memory */
    int irq;

	struct vpm450m *vpm450m;
	int vpm_present;
	int card_type[MAX_BLADE_NUM][MAX_CARDS_PER_BLADE] ;				
	
	unsigned long dtmfactive;
	unsigned long dtmfmask;
	unsigned long dtmfmutemask;

    struct x200_blade blades[MAX_BLADE_NUM];
    struct x200_dev  devices[MAX_BLADE_NUM][MAX_CARDS_PER_BLADE];

    struct x200_dev* clock_master;     /* point to the clock provider slot, null means we are clock master, provide on board clock to all slots*/
};

static inline void __x200_setcreg(struct x200 *x2, unsigned int offset, unsigned int reg, unsigned int val)
{
	volatile unsigned int *p = (unsigned int*)(x2->mem32 + offset + (reg<<2));
	*p = val;
}

static inline unsigned int __x200_getcreg(struct x200 *x2, unsigned int offset, unsigned char reg)
{
	volatile unsigned int *p = (unsigned int*)(x2->mem32 + offset + (reg<<2));
	return (*p);
}

static inline void __x200_set_cs(struct x200 *x2, unsigned char data)
{
    if(x2->reg_cs != data)
    {
        x2->reg_cs = data;
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, data);
    }
}

static inline void __x200_set_led(struct x200 *x2, int span, int color)
{
	int oldreg = x2->ledreg;
	x2->ledreg &= ~(0x3 << (span << 1));
	x2->ledreg |= (color << (span << 1));
	if (oldreg != x2->ledreg) {
        __x200_setcreg(x2, X200_PIO_BASE, PIO_REG6_LEDS, x2->ledreg);  //low to light
    }
}

static inline void __x200_write_8bits(struct x200 *x2, unsigned char bits)
{
	volatile unsigned int t;
	while( (__x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_STATUS)&0x40) != 0x40);      /* wait tx buffer empty */
	__x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_OUT, bits);
	while( (__x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_STATUS)&0x80) != 0x80);      /* wait rx finish */
	t = __x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_IN);                             /* ????????? this line cannot be removed ???????? */
}

static inline unsigned char __x200_read_8bits(struct x200 *x2)
{
	unsigned int res=0;
    while( (__x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_STATUS)&0x40) != 0x40){;}      /* wait tx buffer empty */
	__x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_OUT, 0);                             /* we have to write something so the spi can work */
	while( (__x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_STATUS)&0x80) != 0x80){;}     /* wait rx finish */
	res = __x200_getcreg(x2, X200_SPI_BASE, OPVX_SPI_IN);
	return res&0xff;
}

static inline void __x200_select_blade(struct x200 *x2, int blade)
{
    blade  &= 0x7;
    if(x2->selected_blade != blade) {
        //printk( "selected_blade changed from %d to %d.\n", x2->selected_blade, blade);
        __x200_set_cs(x2, X200_CS_BROADCAST);
        __x200_write_8bits(x2, blade);
        x2->selected_blade = blade;
    }
}

static inline unsigned char __x200_read_blade_reg(struct x200 *x2, int blade, int reg)
{
    unsigned char ret;
    __x200_set_cs(x2, X200_CS_BLADE | (blade & 0x7));
    __x200_write_8bits(x2, X200_CMD_READ_BLADE_REG | (reg&0x1f));
    ret = __x200_read_8bits(x2);
    return ret;
}

static inline unsigned char x200_read_blade_reg(struct x200 *x2, int blade, int reg)
{
    unsigned char ret;
    unsigned long flags;
    spin_lock_irqsave(&x2->lock, flags);
    ret = __x200_read_blade_reg(x2, blade, reg);
    spin_unlock_irqrestore(&x2->lock, flags);
    return ret;
}

static inline void __x200_write_blade_reg(struct x200 *x2, int blade, int reg, unsigned char data)
{
    __x200_set_cs(x2, X200_CS_BLADE | (blade & 0x7));
    __x200_write_8bits(x2, X200_CMD_WRITE_BLADE_REG | (reg&0x1f));
    __x200_write_8bits(x2, data);
}

static inline void x200_write_blade_reg(struct x200 *x2, int blade, int reg, unsigned char data)
{
    unsigned long flags;
    spin_lock_irqsave(&x2->lock, flags);
    __x200_write_blade_reg(x2, blade, reg, data);
    spin_unlock_irqrestore(&x2->lock, flags);
}

static inline void __x200_select_card(struct x200 *x2, int blade, int card)
{
    __x200_select_blade(x2, blade);
    __x200_set_cs(x2, X200_CS_CARD | (card&0x1f));
}

/* export functions for other modules */
int x200_set_clock_master(struct x200 *x2, struct x200_dev* master);
inline struct x200_dev* to_x200_dev(struct device *dev);
struct bus_type *get_bus(int i);
void __wait_spi_wr_fifo(struct x200_dev *dev, unsigned char low_level );
void wait_spi_wr_fifo(struct x200_dev *dev, unsigned char low_level );
unsigned int __get_spi_fifo_data(struct x200_dev *dev, unsigned char size );
unsigned int get_spi_fifo_data(struct x200_dev *dev, unsigned char size );

//for d100m
unsigned char d100m_read_reg(struct x200_dev *dev, unsigned char addr);
void d100m_write_reg(struct x200_dev *dev, unsigned char addr,unsigned char value );
void __d100m_write_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
void d100m_write_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
void __d100m_read_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
void d100m_read_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
int d100m_burst_test(struct x200_dev *dev,  unsigned char *buf,  unsigned char buf_size );
int __t1_framer_burst_in(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
int t1_framer_burst_in(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value);
unsigned int __t1_framer_in(struct x200_dev *dev, const unsigned int reg);
unsigned int t1_framer_in(struct x200_dev *dev, const unsigned int addr);
void __t1_framer_out(struct x200_dev *dev, const unsigned int reg, const unsigned int val);
void t1_framer_out(struct x200_dev *dev, const unsigned int addr, const unsigned int value);
void serial_setup(struct x200_dev* dev);

//for b200m
unsigned char __b200m_read_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char read_immediately);
unsigned char b200m_read_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char read_immediately);
void __b200m_write_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char value);
void b200m_write_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char value);
void b200m_write_xhfc_ra(struct x200_dev *dev, unsigned char r, unsigned char rd, unsigned char addr, unsigned char value);

//for fxo200m
unsigned char __fxo200m_read_si3050reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx);
unsigned char fxo200m_read_reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx);
void __fxo200m_write_si3050reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx);
void fxo200m_write_reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx);

//for fxs200m
unsigned char __fxs200m_read_si3215reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx);
unsigned char fxs200m_read_reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx);
void __fxs200m_write_si3215reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx);
void fxs200m_write_reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx);

#endif /*_X200_HAL_H */
