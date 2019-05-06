
#ifndef _X2D100M_H
#define _X2D100M_H

#define BIT_RW      7
#define BIT_CTRL    6
#define BIT_BURST   5

#define FLAG_READ       (0<<BIT_RW)
#define FLAG_WRITE      (1<<BIT_RW)
#define FLAG_REGS       (1<<BIT_CTRL)
#define FLAG_FALC       (0<<BIT_CTRL)
#define FLAG_BURST      (1<<BIT_BURST)


#define CMD_RD_FALC     FLAG_READ
#define CMD_WR_FALC     FLAG_WRITE
#define CMD_RD_REG(x)   (FLAG_READ  | FLAG_REGS | x)
#define CMD_WR_REG(x)   (FLAG_WRITE | FLAG_REGS | x)


#define CMD_BRD_FALC(x) (FLAG_READ  | FLAG_BURST | x)
#define CMD_BWR_FALC(x) (FLAG_WRITE | FLAG_BURST | x)
#define CMD_BRD_REG(x)  (FLAG_READ  | FLAG_BURST | FLAG_REGS | x)
#define CMD_BWR_REG(x)  (FLAG_WRITE | FLAG_BURST | FLAG_REGS | x)



#define REG_TAG             0
#define REG_CARD_TYPE       1
#define REG_VER             2
#define REG_BULID_YM        3
#define REG_BULID_DATA      4
#define FIFO_WR_PTR         8
#define FIFO_RD_PTR         9

#define FLAG_STARTED 		(1 << 0)
#define FLAG_NMF 			(1 << 1)
#define FLAG_SENDINGYELLOW 	(1 << 2)

#endif /* _X2D100M_H */
