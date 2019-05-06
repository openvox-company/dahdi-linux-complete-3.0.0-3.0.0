#ifndef _X2FXS200M_H
#define _X2FXS200M_H

#define BIT_RW      7                   //BIT 7=0/1代表读/写
#define BIT_CTRL    6                   //BIT 6代表是读写控制器内部寄存器还是外部寄存器
#define BIT_BURST   5       // burst read or write

#define FLAG_R          (0<<BIT_RW)
#define FLAG_W          (1<<BIT_RW)
#define FLAG_CTRL_REG   (1<<BIT_CTRL)
#define FLAG_BUS        (0<<BIT_CTRL)
#define FLAG_BURST      (1<<BIT_BURST)


/* Si3215 Register Definition*/
#define REG_SPIMODE        		00
#define REG_PNI          			06
#define	REG_AUDIO_LOOPBACK		8

#define REG_HYBRID_CTRL         11
#define REG_PWRDOWNCTRL1        14
#define REG_PWRDOWNCTRL2        15

#define REG_INT1_STS        18
#define REG_INT2_STS        19
#define REG_INT3_STS        20
#define REG_INT1_EN        	21
#define REG_INT2_EN        	22
#define REG_INT3_EN        	23


#define REG_IDA_LO		          28
#define REG_IDA_HI		          29
#define REG_IA		          	30
#define REG_IA_STS          	31

#define	REG_ROCTRL						34

#define	REG_RAT_LO						48
#define	REG_RAT_HI						49
#define	REG_RIT_LO						50
#define	REG_RIT_HI						51
						
#define	REG_LFCTRL						64
#define	REG_BATFEED_CTRL			66
#define	REG_AUTO_CTRL					67

#define	REG_LOOPRTD_STS							68

#define	REG_LOOP_I_LIMIT			71
#define	REG_ON_HOOK_V	72
#define	REG_COMMON_V	73
#define	REG_BAT_V_HI	74
#define	REG_BAT_V_LO	75

#define REG_VBATS1						82
#define REG_VBATS2						83

#define	REG_IQ5								88
#define	REG_IQ6								89
#define	REG_RESERVED90				90
#define	REG_RESERVED91				91

#define	REG_DCPWM							92
#define	REG_DCDELAY						93

#define REG_CALIBR1						96
#define REG_CALIBR2						97
#define	REG_RING_GAIN_CAL			98
#define	REG_TIP_GAIN_CAL			99
#define	REG_DC_PEAK_CAL	107
#define	REG_ENHANCE_EN			108

/* Si3215 indirect register definition */
#define	IREG_RCO						7
#define	IREG_RNGX						8
#define	IREG_RNGY						9
#define	IREG_RPTP						16

#define NEON_MWI_RNGY_PULSEWIDTH	0x3e8	/*=> period of 250 mS */

/* Proslic Linefeed options for register 64 - Linefeed Control */
#define SLIC_LF_OPEN				0x0
#define SLIC_LF_ACTIVE_FWD	0x1
#define SLIC_LF_OHTRAN_FWD	0x2 /* Forward On Hook Transfer */
#define SLIC_LF_TIP_OPEN		0x3
#define SLIC_LF_RINGING			0x4
#define SLIC_LF_ACTIVE_REV	0x5
#define SLIC_LF_OHTRAN_REV	0x6 /* Reverse On Hook Transfer */
#define SLIC_LF_RING_OPEN		0x7

#define SLIC_LF_SETMASK			0x7
#define SLIC_LF_OPPENDING 	0x10
/* Mask used to reverse the linefeed mode between forward and
 * reverse polarity. */
#define SLIC_LF_REVMASK 	0x4

#define X2FXS200M_SPI_DELAY   50        /* need delay 15ns between spi operate */

typedef struct {
	unsigned char address;
	unsigned char altaddr;
	char *name;
	unsigned short initial;
} alpha;

#endif
