
#ifndef _X2B200M_H
#define _X2B200M_H

#define HFC_NR_FIFOS	8
#define HFC_ZMIN	0x00			/* from datasheet */
#define HFC_ZMAX	0x7f
#define HFC_FMIN	0x00
#define HFC_FMAX	0x07

/*
 * yuck. Any reg which is not mandated read/write or read-only is write-only.
 * Also, there are dozens of registers with the same address.  Additionally,
 * there are array registers (A_) which have an index register These A_
 * registers require an index register to be written to indicate WHICH in the
 * array you want.
 */

#define R_CIRM			0x00	/* WO       */
#define R_CTRL			0x01	/* WO       */
#define R_CLK_CFG		0x02	/* WO       */
#define A_Z1			0x04	/*    RO    */
#define A_Z2			0x06	/*    RO    */
#define R_RAM_ADDR		0x08	/* WO       */
#define R_RAM_CTRL		0x09	/* WO       */
#define R_FIRST_FIFO		0x0b	/* WO       */
#define R_FIFO_THRES		0x0c	/* WO       */
#define A_F1			0x0c	/*    RO    */
#define R_FIFO_MD		0x0d	/* WO       */
#define A_F2			0x0d	/*    RO    */
#define A_INC_RES_FIFO		0x0e	/* WO       */
#define A_FIFO_STA		0x0e	/*    RO    */
#define R_FSM_IDX		0x0f	/* WO       */
#define R_FIFO			0x0f	/* WO       */

#define R_SLOT			0x10	/* WO       */
#define R_IRQ_OVIEW		0x10	/*    RO    */
#define R_MISC_IRQMSK		0x11	/* WO       */
#define R_MISC_IRQ		0x11	/*    RO    */
#define R_SU_IRQMSK		0x12	/* WO       */
#define R_SU_IRQ		0x12	/*    RO    */
#define R_IRQ_CTRL		0x13	/* WO       */
#define R_AF0_OVIEW		0x13	/*    RO    */
#define R_PCM_MD0		0x14	/* WO       */
#define A_USAGE			0x14	/*    RO    */
#define R_MSS0			0x15	/* WO       */
#define R_MSS1			0x15	/* WO       */
#define R_PCM_MD1		0x15	/* WO       */
#define R_PCM_MD2		0x15	/* WO       */
#define R_SH0H			0x15	/* WO       */
#define R_SH1H			0x15	/* WO       */
#define R_SH0L			0x15	/* WO       */
#define R_SH1L			0x15	/* WO       */
#define R_SL_SEL0		0x15	/* WO       */
#define R_SL_SEL1		0x15	/* WO       */
#define R_SL_SEL7		0x15	/* WO       */
#define R_RAM_USE		0x15	/*    RO    */
#define R_SU_SEL		0x16	/* WO       */
#define R_CHIP_ID		0x16	/*    RO    */
#define R_SU_SYNC		0x17	/* WO       */
#define R_BERT_STA		0x17	/*    RO    */
#define R_F0_CNTL		0x18	/*    RO    */
#define R_F0_CNTH		0x19	/*    RO    */
#define R_TI_WD			0x1a	/* WO       */
#define R_BERT_ECL		0x1a	/*    RO    */
#define R_BERT_WD_MD		0x1b	/* WO       */
#define R_BERT_ECH		0x1b	/*    RO    */
#define R_STATUS		0x1c	/*    RO    */
#define R_SL_MAX		0x1d	/*    RO    */
#define R_PWM_CFG		0x1e	/* WO       */
#define R_CHIP_RV		0x1f	/*    RO    */

#define R_FIFO_BL0_IRQ		0x20	/*    RO    */
#define R_FIFO_BL1_IRQ		0x21	/*    RO    */
#define R_FIFO_BL2_IRQ		0x22	/*    RO    */
#define R_FIFO_BL3_IRQ		0x23	/*    RO    */
#define R_FILL_BL0		0x24	/*    RO    */
#define R_FILL_BL1		0x25	/*    RO    */
#define R_FILL_BL2		0x26	/*    RO    */
#define R_FILL_BL3		0x27	/*    RO    */
#define R_CI_TX			0x28	/* WO       */
#define R_CI_RX			0x28	/*    RO    */
#define R_CGI_CFG0		0x29	/* WO       */
#define R_CGI_STA		0x29	/*    RO    */
#define R_CGI_CFG1		0x2a	/* WO       */
#define R_MON_RX		0x2a	/*    RO    */
#define R_MON_TX		0x2b	/* WO       */

#define A_SU_WR_STA		0x30	/* WO       */
#define A_SU_RD_STA		0x30	/*    RO    */
#define A_SU_CTRL0		0x31	/* WO       */
#define A_SU_DLYL		0x31	/*    RO    */
#define A_SU_CTRL1		0x32	/* WO       */
#define A_SU_DLYH		0x32	/*    RO    */
#define A_SU_CTRL2		0x33	/* WO       */
#define A_MS_TX			0x34	/* WO       */
#define A_MS_RX			0x34	/*    RO    */
#define A_ST_CTRL3		0x35	/* WO       */
#define A_UP_CTRL3		0x35	/* WO       */
#define A_SU_STA		0x35	/*    RO    */
#define A_MS_DF			0x36	/* WO       */
#define A_SU_CLK_DLY		0x37	/* WO       */
#define R_PWM0			0x38	/* WO       */
#define R_PWM1			0x39	/* WO       */
#define A_B1_TX			0x3c	/* WO       */
#define A_B1_RX			0x3c	/*    RO    */
#define A_B2_TX			0x3d	/* WO       */
#define A_B2_RX			0x3d	/*    RO    */
#define A_D_TX			0x3e	/* WO       */
#define A_D_RX			0x3e	/*    RO    */
#define A_BAC_S_TX		0x3f	/* WO       */
#define A_E_RX			0x3f	/*    RO    */

#define R_GPIO_OUT1		0x40	/* WO       */
#define R_GPIO_IN1		0x40	/*    RO    */
#define R_GPIO_OUT3		0x41	/* WO       */
#define R_GPIO_IN3		0x41	/*    RO    */
#define R_GPIO_EN1		0x42	/* WO       */
#define R_GPIO_EN3		0x43	/* WO       */
#define R_GPIO_SEL_BL		0x44	/* WO       */
#define R_GPIO_OUT2		0x45	/* WO       */
#define R_GPIO_IN2		0x45	/*    RO    */
#define R_PWM_MD		0x46	/* WO       */
#define R_GPIO_EN2		0x47	/* WO       */
#define R_GPIO_OUT0		0x48	/* WO       */
#define R_GPIO_IN0		0x48	/*    RO    */
#define R_GPIO_EN0		0x4a	/* WO       */
#define R_GPIO_SEL		0x4c	/* WO       */

#define R_PLL_CTRL		0x50	/* WO       */
#define R_PLL_STA		0x50	/*    RO    */
#define R_PLL_P			0x51	/*       RW */
#define R_PLL_N			0x52	/*       RW */
#define R_PLL_S			0x53	/*       RW */

#define A_FIFO_DATA		0x80	/*       RW */
#define A_FIFO_DATA_NOINC	0x84	/*       RW */

#define R_INT_DATA		0x88	/*    RO    */

#define R_RAM_DATA		0xc0	/*       RW */

#define A_SL_CFG		0xd0	/*       RW */
#define A_CH_MSK		0xf4	/*       RW */
#define A_CON_HDLC		0xfa	/*       RW */
#define A_SUBCH_CFG		0xfb	/*       RW */
#define A_CHANNEL		0xfc	/*       RW */
#define A_FIFO_SEQ		0xfd	/*       RW */
#define A_FIFO_CTRL		0xff	/*       RW */

/* R_CIRM bits */
#define V_CLK_OFF		(1 << 0)	/* 1=internal clocks disabled */
#define V_WAIT_PROC		(1 << 1)	/* 1=additional /WAIT after write access */
#define V_WAIT_REG		(1 << 2)	/* 1=additional /WAIT for internal BUSY phase */
#define V_SRES			(1 << 3)	/* soft reset (group 0) */
#define V_HFC_RES		(1 << 4)	/* HFC reset (group 1) */
#define V_PCM_RES		(1 << 5)	/* PCM reset (group 2) */
#define V_SU_RES		(1 << 6)	/* S/T reset (group 3) */
#define XHFC_FULL_RESET	(V_SRES | V_HFC_RES | V_PCM_RES | V_SU_RES)

/* R_STATUS bits */
#define V_BUSY			(1 << 0)	/* 1=HFC busy, limited register access */
#define V_PROC			(1 << 1)	/* 1=HFC in processing phase */
#define V_LOST_STA		(1 << 3)	/* 1=frames have been lost */
#define V_PCM_INIT		(1 << 4)	/* 1=PCM init in progress */
#define V_WAK_STA		(1 << 5)	/* state of WAKEUP pin wien V_WAK_EN=1 */
#define V_MISC_IRQSTA		(1 << 6)	/* 1=misc interrupt has occurred */
#define V_FR_IRQSTA		(1 << 7)	/* 1=fifo interrupt has occured */
#define XHFC_INTS	(V_MISC_IRQSTA | V_FR_IRQSTA)

/* R_FIFO_BLx_IRQ bits */
#define V_FIFOx_TX_IRQ		(1 << 0)	/* FIFO TX interrupt occurred */
#define V_FIFOx_RX_IRQ		(1 << 1)	/* FIFO RX interrupt occurred */
#define FIFOx_TXRX_IRQ		(V_FIFOx_TX_IRQ | V_FIFOx_RX_IRQ)

/* R_FILL_BLx bits */
#define V_FILL_FIFOx_TX		(1 << 0)	/* TX FIFO reached V_THRES_TX level */
#define V_FILL_FIFOx_RX		(1 << 1)	/* RX FIFO reached V_THRES_RX level */
#define FILL_FIFOx_TXRX		(V_FILL_FIFOx_TX | V_FILL_FIFOx_RX)

/* R_MISC_IRQ / R_MISC_IRQMSK bits */
#define V_SLP_IRQ		(1 << 0)	/* frame sync pulse flips */
#define V_TI_IRQ		(1 << 1)	/* timer elapsed */
#define V_PROC_IRQ		(1 << 2)	/* processing/non-processing transition */
#define V_CI_IRQ		(1 << 4)	/* indication bits changed */
#define V_WAK_IRQ		(1 << 5)	/* WAKEUP pin */
#define V_MON_TX_IRQ		(1 << 6)	/* monitor byte can be written */
#define V_MON_RX_IRQ		(1 << 7)	/* monitor byte received */

/* R_SU_IRQ/R_SU_IRQMSK bits */
#define V_SU0_IRQ		(1 << 0)	/* interrupt/mask port 1 */
#define V_SU1_IRQ		(1 << 1)	/* interrupt/mask port 2 */
#define V_SU2_IRQ		(1 << 2)	/* interrupt/mask port 3 */
#define V_SU3_IRQ		(1 << 3)	/* interrupt/mask port 4 */

/* R_IRQ_CTRL bits */
#define V_FIFO_IRQ_EN		(1 << 0)	/* enable any unmasked FIFO IRQs */
#define V_GLOB_IRQ_EN		(1 << 3)	/* enable any unmasked IRQs */
#define V_IRQ_POL		(1 << 4)	/* 1=IRQ active high */

/* R_BERT_WD_MD bits */
#define V_BERT_ERR		(1 << 3)	/* 1=generate an error bit in BERT stream */
#define V_AUTO_WD_RES		(1 << 5)	/* 1=automatically kick the watchdog */
#define V_WD_RES		(1 << 7)	/* 1=kick the watchdog (bit auto clears) */

/* R_TI_WD bits */
#define V_EV_TS_SHIFT		(0)
#define V_EV_TS_MASK		(0x0f)
#define V_WD_TS_SHIFT		(4)
#define V_WD_TS_MASK		(0xf0)

/* A_FIFO_CTRL bits */
#define V_FIFO_IRQMSK		(1 << 0)	/* 1=FIFO can generate interrupts */
#define V_BERT_EN		(1 << 1)	/* 1=BERT data replaces FIFO data */
#define V_MIX_IRQ		(1 << 2)	/* IRQ when 0=end of frame only, 1=also when Z1==Z2 */
#define V_FR_ABO		(1 << 3)	/* 1=generate frame abort/frame abort detected */
#define V_NO_CRC		(1 << 4)	/* 1=do not send CRC at end of frame */
#define V_NO_REP		(1 << 5)	/* 1=frame deleted after d-chan contention */

/* R_CLK_CFG bits */
#define V_CLK_PLL		(1 << 0)	/* Sysclk select 0=OSC_IN, 1=PLL output */
#define V_CLKO_HI		(1 << 1)	/* CLKOUT selection 0=PLL/8, 1=PLL */
#define V_CLKO_PLL		(1 << 2)	/* CLKOUT source 0=divider or PLL input, 1=PLL output */
#define V_PCM_CLK		(1 << 5)	/* 1=PCM clk = OSC, 0 = PCM clk is 2x OSC */
#define V_CLKO_OFF		(1 << 6)	/* CLKOUT enable 0=enabled */
#define V_CLK_F1		(1 << 7)	/* PLL input pin 0=OSC_IN, 1=F1_1 */

/* R_PCM_MD0 bits */
#define V_PCM_MD		(1 << 0)	/* 1=PCM master */
#define V_C4_POL		(1 << 1)	/* 1=F0IO sampled on rising edge of C4IO */
#define V_F0_NEG		(1 << 2)	/* 1=negative polarity of F0IO */
#define V_F0_LEN		(1 << 3)	/* 1=F0IO active for 2 C4IO clocks */
#define V_PCM_IDX_SEL0		(0x0 << 4)	/* reg15 = R_SL_SEL0 */
#define V_PCM_IDX_SEL1		(0x1 << 4)	/* reg15 = R_SL_SEL1 */
#define V_PCM_IDX_SEL7		(0x7 << 4)	/* reg15 = R_SL_SEL7 */
#define V_PCM_IDX_MSS0		(0x8 << 4)	/* reg15 = R_MSS0 */
#define V_PCM_IDX_MD1		(0x9 << 4)	/* reg15 = R_PCM_MD1 */
#define V_PCM_IDX_MD2		(0xa << 4)	/* reg15 = R_PCM_MD2 */
#define V_PCM_IDX_MSS1		(0xb << 4)	/* reg15 = R_MSS1 */
#define V_PCM_IDX_SH0L		(0xc << 4)	/* reg15 = R_SH0L */
#define V_PCM_IDX_SH0H		(0xd << 4)	/* reg15 = R_SH0H */
#define V_PCM_IDX_SH1L		(0xe << 4)	/* reg15 = R_SH1L */
#define V_PCM_IDX_SH1H		(0xf << 4)	/* reg15 = R_SH1H */
#define V_PCM_IDX_MASK		(0xf0)

/* R_PCM_MD1 bits */
#define V_PLL_ADJ_00		(0x0 << 2)	/* adj 4 times by 0.5 system clk cycles */
#define V_PLL_ADJ_01		(0x1 << 2)	/* adj 3 times by 0.5 system clk cycles */
#define V_PLL_ADJ_10		(0x2 << 2)	/* adj 2 times by 0.5 system clk cycles */
#define V_PLL_ADJ_11		(0x3 << 2)	/* adj 1 time by 0.5 system clk cycles */
#define V_PCM_DR_2048		(0x0 << 4)	/* 2.048Mbps, 32 timeslots */
#define V_PCM_DR_4096		(0x1 << 4)	/* 4.096Mbps, 64 timeslots */
#define V_PCM_DR_8192		(0x2 << 4)	/* 8.192Mbps, 128 timeslots */
#define V_PCM_DR_075		(0x3 << 4)	/* 0.75Mbps, 12 timeslots */
#define V_PCM_LOOP		(1 << 6)	/* 1=internal loopback */
#define V_PCM_SMPL		(1 << 7)	/* 0=sample at middle of bit cell, 1=sample at 3/4 point */
#define V_PLL_ADJ_MASK		(0x3 << 2)
#define V_PCM_DR_MASK		(0x3 << 4)

/* R_PCM_MD2 bits */
#define V_SYNC_OUT1		(1 << 1)	/* SYNC_O source 0=SYNC_I or FSX_RX, 1=512kHz from PLL or multiframe */
#define V_SYNC_SRC		(1 << 2)	/* 0=line interface, 1=SYNC_I */
#define V_SYNC_OUT2		(1 << 3)	/* SYNC_O source 0=rx sync or FSC_RX 1=SYNC_I or received superframe */
#define V_C2O_EN		(1 << 4)	/* C2IO output enable (when V_C2I_EN=0) */
#define V_C2I_EN		(1 << 5)	/* PCM controller clock source 0=C4IO, 1=C2IO */
#define V_PLL_ICR		(1 << 6)	/* 0=reduce PCM frame time, 1=increase */
#define V_PLL_MAN		(1 << 7)	/* 0=auto, 1=manual */

/* A_SL_CFG bits */
#define V_CH_SDIR		(1 << 0)	/* 1=HFC channel receives data from PCM TS */
#define V_ROUT_TX_DIS		(0x0 << 6)	/* disabled, output disabled */
#define V_ROUT_TX_LOOP		(0x1 << 6)	/* internally looped, output disabled */
#define V_ROUT_TX_STIO1		(0x2 << 6)	/* output data to STIO1 */
#define V_ROUT_TX_STIO2		(0x3 << 6)	/* output data to STIO2 */
#define V_ROUT_RX_DIS		(0x0 << 6)	/* disabled, input data ignored */
#define V_ROUT_RX_LOOP		(0x1 << 6)	/* internally looped, input data ignored */
#define V_ROUT_RX_STIO2		(0x2 << 6)	/* channel data comes from STIO1 */
#define V_ROUT_RX_STIO1		(0x3 << 6)	/* channel data comes from STIO2 */
#define V_CH_SNUM_SHIFT		(1)
#define V_CH_SNUM_MASK		(31 << 1)

/* A_CON_HDLC bits */
#define V_IFF			(1 << 0)	/* Inter-Frame Fill: 0=0x7e, 1=0xff */
#define V_HDLC_TRP		(1 << 1)	/* 0=HDLC mode, 1=transparent */
#define V_TRP_DISABLED		(0x0 << 2)	/* FIFO disabled, no interrupt */
#define V_TRP_IRQ_64		(0x1 << 2)	/* FIFO enabled, int @ 8 bytes */
#define V_TRP_IRQ_128		(0x2 << 2)	/* FIFO enabled, int @ 16 bytes */
#define V_TRP_IRQ_256		(0x3 << 2)	/* FIFO enabled, int @ 32 bytes */
#define V_TRP_IRQ_512		(0x4 << 2)	/* FIFO enabled, int @ 64 bytes */
#define V_TRP_IRQ_1024		(0x5 << 2)	/* FIFO enabled, int @ 128 bytes */
#define V_TRP_NO_IRQ		(0x7 << 2)	/* FIFO enabled, no interrupt */
#define V_HDLC_IRQ		(0x3 << 2)	/* HDLC: FIFO enabled, interrupt at end of frame or when FIFO > 16 byte boundary (Mixed IRQ) */
#define V_DATA_FLOW_000		(0x0 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_001		(0x1 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_010		(0x2 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_011		(0x3 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_100		(0x4 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_101		(0x5 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_110		(0x6 << 5)	/* see A_CON_HDLC reg description in datasheet */
#define V_DATA_FLOW_111		(0x7 << 5)	/* see A_CON_HDLC reg description in datasheet */

/* R_FIFO bits */
#define V_FIFO_DIR		(1 << 0)	/* 1=RX FIFO data */
#define V_REV			(1 << 7)	/* 1=MSB first */
#define V_FIFO_NUM_SHIFT	(1)
#define V_FIFO_NUM_MASK		(0x3e)

/* A_CHANNEL bits */
#define V_CH_FDIR		(1 << 0)	/* 1=HFC chan for RX data */
#define V_CH_FNUM_SHIFT		(1)
#define V_CH_FNUM_MASK		(0x3e)

/* R_SLOT bits */
#define V_SL_DIR		(1 << 0)	/* 1=timeslot will RX PCM data from bus */
#define V_SL_NUM_SHIFT		(1)
#define V_SL_NUM_MASK		(0xfe)

/* A_INC_RES_FIFO bits */
#define V_INC_F			(1 << 0)	/* 1=increment FIFO F-counter (bit auto-clears) */
#define V_RES_FIFO		(1 << 1)	/* 1=reset FIFO (bit auto-clears) */
#define V_RES_LOST		(1 << 2)	/* 1=reset LOST error (bit auto-clears) */
#define V_RES_FIFO_ERR		(1 << 3)	/* 1=reset FIFO error (bit auto-clears), check V_ABO_DONE before setting */

/* R_FIFO_MD bits */
#define V_FIFO_MD_00		(0x0 << 0)	/* 16 FIFOs, 64 bytes TX/RX, 128 TX or RX if V_UNIDIR_RX */
#define V_FIFO_MD_01		(0x1 << 0)	/* 8 FIFOs, 128 bytes TX/RX, 256 TX or RX if V_UNIDIR_RX */
#define V_FIFO_MD_10		(0x2 << 0)	/* 4 FIFOs, 256 bytes TX/RX, invalid mode with V_UNIDIR_RX */
#define V_DF_MD_SM		(0x0 << 2)	/* simple data flow mode */
#define V_DF_MD_CSM		(0x1 << 2)	/* channel select mode */
#define V_DF_MD_FSM		(0x3 << 2)	/* FIFO sequence mode */
#define V_UNIDIR_MD		(1 << 4)	/* 1=unidirectional FIFO mode */
#define V_UNIDIR_RX		(1 << 5)	/* 1=unidirection FIFO is RX */

/* A_SUBCH_CFG bits */
#define V_BIT_CNT_8BIT		(0)		/* process 8 bits */
#define V_BIT_CNT_1BIT		(1)		/* process 1 bit */
#define V_BIT_CNT_2BIT		(2)		/* process 2 bits */
#define V_BIT_CNT_3BIT		(3)		/* process 3 bits */
#define V_BIT_CNT_4BIT		(4)		/* process 4 bits */
#define V_BIT_CNT_5BIT		(5)		/* process 5 bits */
#define V_BIT_CNT_6BIT		(6)		/* process 6 bits */
#define V_BIT_CNT_7BIT		(7)		/* process 7 bits */
#define V_LOOP_FIFO		(1 << 6)	/* loop FIFO data */
#define V_INV_DATA		(1 << 7)	/* invert FIFO data */
#define V_START_BIT_SHIFT	(3)
#define V_START_BIT_MASK	(0x38)

/* R_SU_SYNC bits */
#define V_SYNC_SEL_PORT0	(0x0 << 0)	/* sync to TE port 0 */
#define V_SYNC_SEL_PORT1	(0x1 << 0)	/* sync to TE port 1 */
#define V_SYNC_SEL_PORT2	(0x2 << 0)	/* sync to TE port 2 */
#define V_SYNC_SEL_PORT3	(0x3 << 0)	/* sync to TE port 3 */
#define V_SYNC_SEL_SYNCI	(0x4 << 0)	/* sync to SYNC_I */
#define V_MAN_SYNC		(1 << 3)	/* 1=manual sync mode */
#define V_AUTO_SYNCI		(1 << 4)	/* 1=SYNC_I used if FSC_RX not found */
#define V_D_MERGE_TX		(1 << 5)	/* 1=all 4 dchan taken from one byte in TX */
#define V_E_MERGE_RX		(1 << 6)	/* 1=all 4 echan combined in RX direction */
#define V_D_MERGE_RX		(1 << 7)	/* 1=all 4 dchan combined in RX direction */
#define V_SYNC_SEL_MASK		(0x03)

/* A_SU_WR_STA bits */
#define V_SU_SET_STA_MASK	(0x0f)
#define V_SU_LD_STA		(1 << 4)	/* 1=force SU_SET_STA mode, must be manually cleared 6us later */
#define V_SU_ACT_NOP		(0x0 << 5)	/* NOP */
#define V_SU_ACT_DEACTIVATE	(0x2 << 5)	/* start deactivation. auto-clears */
#define V_SU_ACT_ACTIVATE	(0x3 << 5)	/* start activation. auto-clears. */
#define V_SET_G2_G3		(1 << 7)	/* 1=auto G2->G3 in NT mode. auto-clears after transition. */

/* A_SU_RD_STA */
#define V_SU_STA_MASK		(0x0f)
#define V_SU_FR_SYNC		(1 << 4)	/* 1=synchronized */
#define V_SU_T2_EXP		(1 << 5)	/* 1=T2 expired (NT only) */
#define V_SU_INFO0		(1 << 6)	/* 1=INFO0 */
#define V_G2_G3			(1 << 7)	/* 1=allows G2->G3 (NT only, auto-clears) */

/* A_SU_CLK_DLY bits */
#define V_SU_DLY_MASK		(0x0f)
#define V_SU_SMPL_MASK		(0xf0)
#define V_SU_SMPL_SHIFT		(4)

/* A_SU_CTRL0 bits */
#define V_B1_TX_EN		(1 << 0)	/* 1=B1-channel transmit */
#define V_B2_TX_EN		(1 << 1)	/* 1=B2-channel transmit */
#define V_SU_MD			(1 << 2)	/* 0=TE, 1=NT */
#define V_ST_D_LPRIO		(1 << 3)	/* D-Chan priority 0=high, 1=low */
#define V_ST_SQ_EN		(1 << 4)	/* S/Q bits transmit (1=enabled) */
#define V_SU_TST_SIG		(1 << 5)	/* 1=transmit test signal */
#define V_ST_PU_CTRL		(1 << 6)	/* 1=enable end of pulse control */
#define V_SU_STOP		(1 << 7)	/* 1=power down */

/* A_SU_CTRL1 bits */
#define V_G2_G3_EN		(1 << 0)	/* 1=G2->G3 allowed without V_SET_G2_G3 */
#define V_D_RES			(1 << 2)	/* 1=D-chan reset */
#define V_ST_E_IGNO		(1 << 3)	/* TE:1=ignore Echan, NT:should always be 1. */
#define V_ST_E_LO		(1 << 4)	/* NT only: 1=force Echan low */
#define V_BAC_D			(1 << 6)	/* 1=BAC bit controls Dchan TX */
#define V_B12_SWAP		(1 << 7)	/* 1=swap B1/B2 */

/* A_SU_CTRL2 bits */
#define V_B1_RX_EN		(1 << 0)	/* 1=enable B1 RX */
#define V_B2_RX_EN		(1 << 1)	/* 1=enable B2 RX */
#define V_MS_SSYNC2		(1 << 2)	/* 0 normally, see datasheet */
#define V_BAC_S_SEL		(1 << 3)	/* see datasheet */
#define V_SU_SYNC_NT		(1 << 4)	/* 0=sync pulses generated only in TE, 1=in TE and NT */
#define V_SU_2KHZ		(1 << 5)	/* 0=96kHz test tone, 1=2kHz */
#define V_SU_TRI		(1 << 6)	/* 1=tristate output buffer */
#define V_SU_EXCHG		(1 << 7)	/* 1=invert output drivers */

/* R_IRQ_OVIEW bits */
#define V_FIFO_BL0_IRQ		(1 << 0)	/* FIFO 0-3 IRQ */
#define V_FIFO_BL1_IRQ		(1 << 1)	/* FIFO 4-7 IRQ */
#define V_FIFO_BL2_IRQ		(1 << 2)	/* FIFO 8-11 IRQ */
#define V_FIFO_BL3_IRQ		(1 << 3)	/* FIFO 12-15 IRQ */
#define V_MISC_IRQ		(1 << 4)	/* R_MISC_IRQ changed */
#define V_STUP_IRQ		(1 << 5)	/* R_SU_IRQ changed */
#define V_FIFO_BLx_IRQ		(V_FIFO_BL0_IRQ | V_FIFO_BL1_IRQ | V_FIFO_BL2_IRQ | V_FIFO_BL3_IRQ)

/* R_FIRST_FIFO bits */
#define V_FIRST_FIFO_NUM_SHIFT	(1)

/* A_FIFO_SEQ bits */
#define V_NEXT_FIFO_NUM_SHIFT	(1)
#define V_SEQ_END		(1 << 6)


#endif /* _X2BX00M_H */
