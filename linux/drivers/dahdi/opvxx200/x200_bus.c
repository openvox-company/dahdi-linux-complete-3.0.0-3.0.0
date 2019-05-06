/*
 * OpenVox x200 hybrid telephony card driver.
 *
 * Written by Miao Lin<lin.miao@openvox.cn>
 *
 * Copyright (C) 2011, OpenVox Communication Co Ltd..
 *
 * All rights reserved.
 */

/*
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>

//#include <linux/usb.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/string.h>

#include <dahdi/kernel.h>

#include "ec3000.h"
#include "x200_hal.h"
#include "wct4xxp/wct4xxp.h"    /* For certain definitions */
#include "x2d100m.h"
#include "x2b200m.h"
#include "x2fxo200m.h"
#include "x2fxs200m.h"

unsigned int interface_num = 0;

int vpmsupport = 1;
int reset_fpga = 0x1;
int reset_fpga_delay = 0x40;                //reset delays in 66m clocks.
unsigned int spi_fifo_mode= 0x01;            //bit0:spi in sigle byte mode, bit1:spi in fifo mode
unsigned int tdm_mode = 0x20;               //bit7:linear16bits1_xlaw8bits0, bit6:pak_data, bit5:active, bit4:loopbak
unsigned int x200_spi_mode = 0x130333;      //0x222=11MSPI, 0x333=8.25MSPI,0x444=6MSPI,0x555=5.5MSPI,0x666=4.17MSPI,0x777=4.125MSPI,0x888=3.66MSPI
int spi_daisy_chain = 0;        						// for x200,spi_daisy_chain=1;for x204,spi_daisy_chain=0.

#define MAX_DMA_BUFFER_NUM			2


EXPORT_SYMBOL(tdm_mode);
EXPORT_SYMBOL(spi_fifo_mode);
EXPORT_SYMBOL(x200_spi_mode);
EXPORT_SYMBOL(spi_daisy_chain);
EXPORT_SYMBOL(vpmsupport);
EXPORT_SYMBOL(interface_num);


struct x200_variant {
    char* name;
    unsigned int flag;
};

static struct x200_variant x200_devel = { "devel", 0 };
static struct x200_variant x204v1 = { "x204", 0 };


static spinlock_t interfaces_lock;
static struct x200 *interfaces[WC_MAX_INTERFACES];


struct bus_type x200_bus_type[WC_MAX_INTERFACES];
struct device x200_devroot[WC_MAX_INTERFACES];


struct bus_type *get_bus(int i)
{
		return &x200_bus_type[i];
}
EXPORT_SYMBOL(get_bus);

inline struct x200_dev* to_x200_dev(struct device *dev)
{
		return container_of(dev, struct x200_dev, dev);	
}
EXPORT_SYMBOL(to_x200_dev);

static int x200_match(struct device *dev, struct device_driver *driver) 
{
		struct x200_dev *x2dev = to_x200_dev(dev);
    return (((x2dev->dev_type == DEV_TYPE_D100M)&&(!strncmp("d100m_driver", driver->name, strlen(driver->name)))) ||
    				((x2dev->dev_type == DEV_TYPE_B200M)&&(!strncmp("b200m_driver", driver->name, strlen(driver->name)))) ||
    				((x2dev->dev_type == DEV_TYPE_FXO200M)&&(!strncmp("fxo200m_driver", driver->name, strlen(driver->name)))) ||
    				((x2dev->dev_type == DEV_TYPE_FXS200M)&&(!strncmp("fxs200m_driver", driver->name, strlen(driver->name))))
    				);
}

static void x200_devroot_release(struct device *dev) 
{
}

static void x200_bus_setup(int i)
{
		 x200_bus_type[i].name = kasprintf(GFP_KERNEL,"x200_bus_%d",i);
		 x200_bus_type[i].match = x200_match;
		 
		 dev_set_name(&x200_devroot[i],"x200_devroot_%d",i);
		 x200_devroot[i].release = x200_devroot_release;
}

static int x200_bus_register(int i)
{
		int ret;
		
		ret = bus_register(&x200_bus_type[i]);									
    if(ret) {   
        printk("X200 ERROR: x200 bus %d register failed!\n",i);
        return ret;
    }
    
    ret = device_register(&x200_devroot[i]);					
    if (ret){
    		bus_unregister(&x200_bus_type[i]);
        printk("X200 ERROR: x200 bus device register failed!\n");
        return ret;
    }
    
    return 0;
}

static void x200_bus_unregister(int i)
{
		device_unregister(&x200_devroot[i]);
		bus_unregister(&x200_bus_type[i]);				
}


static void x200_stop_dma(struct x200 *x2)
{
		unsigned long flags;
		
    printk("x200: Stop DMA\n");
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_RUN, 0);
    spin_unlock_irqrestore(&x2->lock, flags);
}

static void x200_start_dma(struct x200 *x2)
{
		unsigned long flags;
		
    printk(KERN_DEBUG "x200: Start DMA\n");
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_TDM_MODE, tdm_mode);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_TDM_DAT_LEN, 1024);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_DMA_NIOS_TO_HOST, x2->readdma);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_DMA_HOST_TO_NIOS, x2->writedma);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_MS_PER_IRQ, 1);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_MAX_BUF_NO, MAX_DMA_BUFFER_NUM);
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_BUF_INDEX, 0);
    x2->last_chunk_index = 0;
    __x200_setcreg(x2, X200_REG_BASE, OPVX_REG_RUN,1);
    spin_unlock_irqrestore(&x2->lock, flags);
}

static inline void x200_reset_modules(struct x200 *x2)
{
		unsigned long flags;
		
		spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_PIO_BASE, 0, (BIT_BUS_RESET<<16)|BIT_BUS_RESET);
    spin_unlock_irqrestore(&x2->lock, flags);
    
    mdelay(100);
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_PIO_BASE, 0, (BIT_BUS_RESET<<16)|0x0);
    spin_unlock_irqrestore(&x2->lock, flags);
    
    mdelay(50);
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_PIO_BASE, 0, (BIT_BUS_RESET<<16)|BIT_BUS_RESET);
    spin_unlock_irqrestore(&x2->lock, flags);
}

void __x200_reset_dev(struct x200_dev *dev, int ms)
{
    unsigned char mask = 1<<dev->slot_id;
    dev->parent->reg_reset = __x200_read_blade_reg(dev->parent->parent, dev->parent->blade_id, X200_BLADE_REG_RESET);
    dev->parent->reg_reset |= mask;
    __x200_write_blade_reg(dev->parent->parent, dev->parent->blade_id, X200_BLADE_REG_RESET, dev->parent->reg_reset);
    mdelay(ms);
    dev->parent->reg_reset &= ~mask;
    __x200_write_blade_reg(dev->parent->parent, dev->parent->blade_id, X200_BLADE_REG_RESET, dev->parent->reg_reset);
    mdelay(ms);
    dev->parent->reg_reset |= mask;
    __x200_write_blade_reg(dev->parent->parent, dev->parent->blade_id, X200_BLADE_REG_RESET, dev->parent->reg_reset);
}

static void __x200_enable_interrupt(struct x200 *x2)
{
    printk(KERN_DEBUG "x200: Enabled interrupts!\n");
}

static void __x200_disable_interrupt(struct x200 *x2)
{
    printk(KERN_DEBUG "x200: Disable interrupts!\n");
}

int x200_set_clock_master(struct x200 *x2, struct x200_dev* master)
{	
		unsigned long flags;
		spin_lock_irqsave(&x2->lock, flags);
    if(master) {   //master != NULL, using card back clock
        __x200_setcreg(x2, X200_PIO_BASE, 0, 0x08000800);
        __x200_write_blade_reg(x2, master->blade_id, X200_BLADE_REG_CLK_SEL,  master->slot_id+1);
    } else {     //master=NULL,  using local clock
        __x200_setcreg(x2, X200_PIO_BASE, 0, 0x08000000);
    }
    spin_unlock_irqrestore(&x2->lock, flags);
    x2->clock_master = master;
    return 0;
}
EXPORT_SYMBOL(x200_set_clock_master);

static int x200_set_timing_source_auto(struct x200 *x2)
{
		int x,y;
    int port_id, new_port_id = 0 ;
    struct x200_dev *dev, *new_dev=NULL ;
    int syncpos, new_syncpos = 0 ;
    
    unsigned long flags;

    if(x2->checktiming) {
        x2->checktiming = 0 ;
    } else {
        return 0 ;
    }
    
    for(x=0; x<MAX_BLADE_NUM; x++) {
    		if(x2->blades[x].exist) {
    				for(y=0; y<x2->blades[x].max_cards; y++){
    						dev = &x2->devices[x][y] ;
    						if((dev->dev_type != DEV_TYPE_D100M) && (dev->dev_type != DEV_TYPE_B200M))
    							 continue;
    						if((dev->ops == NULL)||(dev->drv_data == NULL))
    							 continue;    						
    						
    						syncpos=0;
        				port_id=0;
        				dev->ops->get_clk_src(dev, &syncpos, &port_id);
        				if ( (syncpos > 0) && ((syncpos < new_syncpos ) || (new_syncpos == 0 )) ) {
            				new_dev = dev ;
            				new_port_id = port_id ;
            				new_syncpos = syncpos ;
        				}
    				}
    		}
    }
        
    if ( (x2->sync != new_syncpos)||(new_dev != x2->clock_master) ) {
        //unset the prevous selection
        if ( x2->clock_master && (  x2->clock_master != new_dev ) ) {
            printk("x200: line %d, unset clock source from slot:%d syncpos=%d. \n",__LINE__, x2->clock_master->slot_id,x2->sync);
            x2->clock_master->ops->unset_clk_src(x2->clock_master);
        }
              
        if ( !new_dev )  {
            printk("x200: %s,line:%d no available clock, use onboard clock.\n",__func__,__LINE__);
            
            spin_lock_irqsave(&x2->lock, flags);
            __x200_setcreg(x2, X200_PIO_BASE, 0, 0x08000000);
            spin_unlock_irqrestore(&x2->lock, flags);
            
            x2->sync = 0 ;
            x2->clock_master = NULL ;
            
            return 0;
        } else {
             //set the new selection
            printk("x200: master clock changed from %d to %d(slot_id/port_id=%d/%d)\n",
            					 x2->sync, new_syncpos,new_dev->slot_id,new_port_id);
            
            new_dev->ops->set_clk_src(new_dev, new_syncpos, new_port_id);
            
            spin_lock_irqsave(&x2->lock, flags);
            __x200_write_blade_reg(x2, new_dev->blade_id, X200_BLADE_REG_CLK_SEL,  new_dev->slot_id+1);
            __x200_setcreg(x2, X200_PIO_BASE, 0, 0x08000800);
            spin_unlock_irqrestore(&x2->lock, flags);
            
            x2->sync = new_syncpos ;
            x2->clock_master = new_dev ;            
        }  
    }
    
    return  new_syncpos;
}

DAHDI_IRQ_HANDLER(x200_interrupt)
{
    struct x200 *       x2 = dev_id;
    unsigned int        ints;
    unsigned long       flags;
    int                 index;	
    int 								x,y;
		
		
		spin_lock_irqsave(&x2->lock, flags);
    ints = __x200_getcreg(x2, X200_PCI_BASE, OPVX_REG_IRQ_STATUS) & (1<<16);
    spin_unlock_irqrestore(&x2->lock, flags);
    
    if (!ints)
        return IRQ_NONE;

    //__x200_setcreg(x2, X200_PIO_BASE, 3, 0);
    x2->intcount++;
    
    spin_lock_irqsave(&x2->lock, flags);
    index = __x200_getcreg(x2, X200_REG_BASE, OPVX_REG_BUF_INDEX);
    spin_unlock_irqrestore(&x2->lock, flags);

    /* assume dual buffer now */
    x2->current_writechunk = x2->writechunk;
    x2->current_readchunk = x2->readchunk;
    if(index==0) {  /* Nios is using first half */
        x2->current_writechunk += 1024 ;
        x2->current_readchunk += 1024 ;
    }

    for(x=0; x<MAX_BLADE_NUM; x++) {
    		if(x2->blades[x].exist) {
    				for(y=0; y<x2->blades[x].max_cards; y++) {
    						if((x2->devices[x][y].drv_data != NULL)&&(x2->devices[x][y].ops != NULL)){
    								x2->devices[x][y].ops->irq_handler(x2->intcount, &x2->devices[x][y]);
    						}
    				}
    		}
    }
		
    x2->last_chunk_index = (x2->last_chunk_index+1)%MAX_DMA_BUFFER_NUM;
    
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_PCI_BASE, OPVX_REG_IRQ_STATUS, ints); // clear interrupt register
    spin_unlock_irqrestore(&x2->lock, flags);

    /* clock source checking  */
    x200_set_timing_source_auto(x2);
    
    return IRQ_RETVAL(1);
}

static int x200_d100m_probe(struct x200_dev * dev)
{
		struct x200* x2 = dev->bus;
		unsigned long flags;
		unsigned int i,detect_loopcnt=16 ;
	  unsigned char stmp[detect_loopcnt+1];
	  unsigned char buf0[] = "OpenVox D100M module for x200 telephony card driver.";
	  unsigned char *buf;
	  unsigned char buf_size ;
    unsigned char type;
		
		if(dev->parent->cardflag & (1<<dev->slot_id))	{
    		return	1;					//inused.
    }
    
    spin_lock_irqsave(&x2->lock, flags);
		__x200_reset_dev(dev, 60);
		spin_unlock_irqrestore(&x2->lock, flags);
    mdelay(300);
    
    // now we try to see if there have openvox hardware.
    memset(stmp, 0, sizeof(stmp));
    for(i=0; i<detect_loopcnt; i++) {
        stmp[i] = d100m_read_reg(dev, REG_TAG);
    }
    
    if(strstr(stmp, "OpenVox")) {
        type = d100m_read_reg(dev, REG_CARD_TYPE);
        if(DEV_TYPE_D100M == type) {    	    	
            d100m_write_reg(dev,FIFO_WR_PTR,0) ;
            d100m_write_reg(dev,FIFO_RD_PTR,0) ;

            buf = buf0 ;
            buf_size = sizeof(buf0) - 1;

            if( d100m_burst_test(dev, buf,buf_size ) ) {
            	return 4;
            }	
            dev->parent->cardflag |= (1 << dev->slot_id);
            dev->parent->tdm_fsc_mod &= ~(1 << dev->slot_id);
            spin_lock_irqsave(&x2->lock, flags);
						__x200_reset_dev(dev, 60);
						spin_unlock_irqrestore(&x2->lock, flags);
            mdelay(300);						
						printk("x200 : bus %d, blade %d, slot %d find d100m device.\n",x2->bus_id, dev->parent->blade_id, dev->slot_id);           
        } else {
            return 3;
        }
    } else {
        return 2;
    }
		
		return 0;
}

static int x200_b200m_probe(struct x200_dev *dev)
{
    unsigned char chip_id, reversion;
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    if(dev->parent->cardflag & (1<<dev->slot_id)) {
  	    return	1;
    }
    
    spin_lock_irqsave(&x2->lock, flags);
    __x200_reset_dev(dev, 15*4);
    spin_unlock_irqrestore(&x2->lock, flags);
    
    
    mdelay(100);
    b200m_write_xhfc(dev, 0, 0x8);
    mdelay(100);

    chip_id = b200m_read_xhfc(dev, 0x16,1);                   //b200m_probe
    reversion = b200m_read_xhfc(dev, 0x1f,1);                 //b200m_probe
		
    if( (chip_id==0x61) && (reversion==0) ) {
        dev->parent->cardflag |= (1 << dev->slot_id);
        dev->parent->tdm_fsc_mod |= (1 << dev->slot_id);        //fsync output = !bus_tdm_fync
        dev->parent->tdm_fsc_mod |= (1 << (dev->slot_id +4));   //spi_cs1_fsync_back use as fsync_back, input to fpga
        printk("x200 : bus %d, blade %d, slot %d find b200m device.\n",x2->bus_id, dev->parent->blade_id, dev->slot_id);
        return  0;
    } else {
        return 1;
    }
}

static int x200_fxo200m_probe(struct x200_dev *dev)
{
    int i;
    int detect_loopcnt=64;
    unsigned char stmp[detect_loopcnt+1];
    struct x200* x2 = dev->bus;
    unsigned long flags;

    if(dev->parent->cardflag & (1<<dev->slot_id)) {
        return  1;
    }

    spin_lock_irqsave(&x2->lock, flags);
    __x200_reset_dev(dev, 60);
    spin_unlock_irqrestore(&x2->lock, flags);
    mdelay(200);  //200ms delay is ok, 100us delay is failed. what is the proper delay??????

    memset(stmp, 0, sizeof(stmp));
    for(i=0; i<detect_loopcnt; i++) {
        stmp[i] = fxo200m_read_reg(dev, i, dev->slot_id);
    }

    //now check the si3050 revision id, reg11:Line-side ID and REVA + reg13: Line Side Device Revision
    if( ((stmp[11]==0x04) || (stmp[11]==0x05) ) && (stmp[13]==0x40) ) {
        dev->parent->cardflag |= (1 << dev->slot_id);
        dev->parent->tdm_fsc_mod |= (1 << dev->slot_id);
        dev->parent->tdm_fsc_mod &= ~(1 << (dev->slot_id+4));
        printk("x200 : bus %d, blade %d, slot %d find fxo200m device.\n",x2->bus_id, dev->parent->blade_id, dev->slot_id);
        return  0;
    } else {
        return 1;
    }
}

static int x200_fxs200m_probe(struct x200_dev *dev)
{
    struct x200 *x2 = dev->bus;
		unsigned char is_fxs200m, chanx ;
		unsigned char reg0,reg1,reg6 ;
		unsigned char reg_cs;
		unsigned long flags;

    if(dev->parent->cardflag & (1<<dev->slot_id))	{
  	    return	1;
    }
    
    spin_lock_irqsave(&x2->lock, flags);
    __x200_reset_dev(dev, 50);
    
    // set blade
		if(x2->selected_blade != dev->blade_id) {
				 if(spi_fifo_mode)
         		__wait_spi_wr_fifo(dev,0);
         		
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    //set cs
    reg_cs = X200_CS_CARD | dev->slot_id;
    if(x2->reg_cs != reg_cs) {
    	  if(spi_fifo_mode)
        		__wait_spi_wr_fifo(dev,0);
        		
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS,reg_cs); 
        x2->reg_cs = reg_cs;
    }
    
    if ( spi_daisy_chain ) {
        __x200_write_8bits(x2, REG_SPIMODE);
        __x200_write_8bits(x2, 0x80);   //set bit7 to enable SPI daisy chain mode.
    }
		spin_unlock_irqrestore(&x2->lock, flags);
		
    is_fxs200m = 1;
    for (chanx=0;chanx<CHANS_PER_MODULE;chanx++)  {
        reg0 = fxs200m_read_reg(dev, 0, chanx);
        reg1 = fxs200m_read_reg(dev, 1, chanx);
        reg6 = fxs200m_read_reg(dev, 6, chanx);
    		if( ((reg0&0x3f)!=0x03) || (reg1!=0x88) ||(reg6!=0) ) {
    				is_fxs200m = 0;
    				break ;
    		}
    }

    if  ( is_fxs200m == 1 )  {
        dev->parent->cardflag |= (1 << dev->slot_id);
        dev->parent->tdm_fsc_mod |= (1 << dev->slot_id);
        dev->parent->tdm_fsc_mod &= ~(1 << (dev->slot_id+4));
    		printk("x200 : bus %d, blade %d, slot %d find fxs200m device.\n",x2->bus_id, dev->parent->blade_id, dev->slot_id);
	    	return 0 ;
    } else {
	    	return 1 ;
    }
}

static void x200_dev_release(struct device *dev) 
{
}

static void x200_dev_probe(struct x200 * x2)
{		
		int i,j;
		struct x200_dev *xdev;
		struct device *dev;
		
		for(i=0; i<MAX_BLADE_NUM; i++)
		{
				if(x2->blades[i].exist)
				{
						for(j=0; j<x2->blades[i].max_cards; j++)
						{
								xdev = &x2->devices[i][j];
								dev = &(xdev->dev);
								
								dev->bus = &x200_bus_type[x2->bus_id];
								dev->parent = &x200_devroot[x2->bus_id];
								if(x200_d100m_probe(xdev) == 0){
										dev_set_name(dev,"x200_d100m_%d_%d_%d",x2->bus_id,i,j);
										dev->release = x200_dev_release;										
										
										if(device_register(dev)){
												printk("X200 ERROR: x200_d100m_%d_%d_%d register failed!\n",x2->bus_id,i,j);
												continue;
										}
										xdev->registered = 1;
										xdev->dev_type = DEV_TYPE_D100M;
										serial_setup(xdev);
								}
								else if( x200_b200m_probe(xdev)== 0 )
								{
										dev_set_name(dev,"x200_b200m_%d_%d_%d",x2->bus_id,i,j);
										dev->release = x200_dev_release;										
										
										if(device_register(dev)){
												printk("X200 ERROR: x200_b200m_%d_%d_%d register failed!\n",x2->bus_id,i,j);
												continue;
										}
										xdev->registered = 1;
										xdev->dev_type = DEV_TYPE_B200M;
								}
								else if( x200_fxo200m_probe(xdev)== 0 )
								{
										dev_set_name(dev,"x200_fxo200m_%d_%d_%d",x2->bus_id,i,j);
										dev->release = x200_dev_release;										
										
										if(device_register(dev)){
												printk("X200 ERROR: x200_fxo200m_%d_%d_%d register failed!\n",x2->bus_id,i,j);
												continue;
										}
										xdev->registered = 1;
										xdev->dev_type = DEV_TYPE_FXO200M;
								}
								else if( x200_fxs200m_probe(xdev) == 0 )
								{
										dev_set_name(dev,"x200_fxs200m_%d_%d_%d",x2->bus_id,i,j);
										dev->release = x200_dev_release;										
										
										if(device_register(dev)){
												printk("X200 ERROR: x200_fxs200m_%d_%d_%d register failed!\n",x2->bus_id,i,j);
												continue;
										}
										xdev->registered = 1;
										xdev->dev_type = DEV_TYPE_FXS200M;
								}
								else{
										printk("x200 : bus %d, blade %d, slot %d find no device!\n",x2->bus_id,i,j);
								}
						}
						x200_write_blade_reg(x2, x2->blades[i].blade_id, X200_BLADE_REG_FSC_MOD, x2->blades[i].tdm_fsc_mod);
				}
		}
}

static int x200_init_bus(struct x200* x2)
{
    int i, j;
    int retval = -ENOMEM;
    int blade_cnt = 0;
    unsigned long flags;

    /* init data structure */
    for(i=0; i<MAX_BLADE_NUM; i++) {
        x2->blades[i].blade_id = i;
        x2->blades[i].exist = 0;
        x2->blades[i].parent = x2;
        for(j=0; j<MAX_CARDS_PER_BLADE; j++) {
            x2->devices[i][j].parent = &x2->blades[i];
            x2->devices[i][j].bus = x2;
            x2->devices[i][j].slot_id = j;
            x2->devices[i][j].blade_id = x2->devices[i][j].parent->blade_id;
            x2->devices[i][j].bus_id = x2->bus_id;
            x2->devices[i][j].registered = 0;
            x2->devices[i][j].dev_type = DEV_TYPE_NONE;
            x2->devices[i][j].drv_data = NULL;
            x2->devices[i][j].ops = NULL;
        }
    }

    /* first init hardware, spi, gpio etc */
    x200_reset_modules(x2);                                           /* generate reset signals */
     x200_set_clock_master(x2, NULL);																/* use onboard osc as clock master at first */
    
    spin_lock_irqsave(&x2->lock, flags);                         
    __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CNTL, 0);                /* setup spi controller */
    __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_MODE, x200_spi_mode);    /* setup spi clock speed */
		spin_unlock_irqrestore(&x2->lock, flags);
		
    /* probe blades */
    for(i=0; i<MAX_BLADE_NUM; i++) {
        unsigned char ch, ver;
        //lock the SPI Bus
        spin_lock_irqsave(&x2->lock, flags);
        ch = __x200_read_blade_reg(x2, i, X200_BLADE_REG_FLAG);
        spin_unlock_irqrestore(&x2->lock, flags);
        
        if(X200_BLADE_TAG == ch) {  /* yes, it is the valid blade */
        		spin_lock_irqsave(&x2->lock, flags);
            ver = __x200_read_blade_reg(x2, i, X200_BLADE_REG_VERSION);
            spin_unlock_irqrestore(&x2->lock, flags);
            
            printk("x200: bus %d blade %d detected, tag 0x%x, version 0x%x\n", x2->bus_id, i, ch, ver);
            x2->blades[i].exist = 1;
            x2->blades[i].ver = ver;
            x2->blades[i].max_cards = 4;
            blade_cnt++;
        } else {
            printk("x200: bus %d blade %d not found, tag is 0x%x\n", x2->bus_id, i, ch);
        }
    }

    if(blade_cnt) {
        printk("x200: %d blades found\n", blade_cnt);
        retval = 0;
    }

    return retval;
}

static int x200_vpm_init(struct x200 * x2)
{
    int x;
    int laws[4] = { 1, }; // DAHDI_LAW_ALAW in default
    unsigned int check1, check2;
    unsigned long flags;

    unsigned int vpm_capacity;
    struct firmware embedded_firmware;
    const struct firmware *firmware = &embedded_firmware;

#if !defined(HOTPLUG_FIRMWARE)
    extern void _binary_dahdi_fw_oct6114_032_bin_size;
    extern void _binary_dahdi_fw_oct6114_064_bin_size;
    extern void _binary_dahdi_fw_oct6114_128_bin_size;
    extern u8 _binary_dahdi_fw_oct6114_032_bin_start[];
    extern u8 _binary_dahdi_fw_oct6114_064_bin_start[];
    extern u8 _binary_dahdi_fw_oct6114_128_bin_start[];
#else
    static const char oct032_firmware[] = "dahdi-fw-oct6114-032.bin";
    static const char oct064_firmware[] = "dahdi-fw-oct6114-064.bin";
    static const char oct128_firmware[] = "dahdi-fw-oct6114-128.bin";
#endif


    if (!vpmsupport) {
        printk("OpenVox VPM: Support Disabled\n");
        return 0;
    }

		spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, V2_EC_BASE, 0x0004, 0x1234);
    __x200_setcreg(x2, V2_EC_BASE, 0x000a, 0x5678);
    check1 = __x200_getcreg(x2, V2_EC_BASE, 0x0004);
    check2 = __x200_getcreg(x2, V2_EC_BASE, 0x000a);
    spin_unlock_irqrestore(&x2->lock, flags);

    if ((check1 != 0x1234) || (check2 != 0x5678)) {
        printk("OpenVox VPM: Not Present\n");
        return -1;
    }

    printk("OpenVox VPM: found.\n");

    /* Setup alaw vs ulaw rules */
    for (x = 0;x < 4; x++) {
        //if(x2->span.deflaw == DAHDI_LAW_ALAW)
            laws[x] = 1;
    }

    vpm_capacity = get_vpm450m_capacity(x2);
    printk("OpenVox VPM: echo cancellation supports %d channels\n", vpm_capacity);

    switch (vpm_capacity) {
    case 32:
#if defined(HOTPLUG_FIRMWARE)
        if ((request_firmware(&firmware, oct032_firmware, &x2->dev->dev) != 0) ||!firmware) {
            printk("OpenVox VPM: firmware %s not available from userspace\n", oct032_firmware);
            return -2;
        }
#else
        embedded_firmware.data = _binary_dahdi_fw_oct6114_032_bin_start;
        embedded_firmware.size = (size_t) &_binary_dahdi_fw_oct6114_032_bin_size;
#endif
        break;
    case 64:
#if defined(HOTPLUG_FIRMWARE)
        if ((request_firmware(&firmware, oct064_firmware, &x2->dev->dev) != 0) ||!firmware) {
            printk("OpenVox VPM: firmware %s not available from userspace\n", oct064_firmware);
            return -2;
        }
#else
        embedded_firmware.data = _binary_dahdi_fw_oct6114_064_bin_start;
        embedded_firmware.size = (size_t) &_binary_dahdi_fw_oct6114_064_bin_size;
#endif
        break;
    case 128:
#if defined(HOTPLUG_FIRMWARE)
        if ((request_firmware(&firmware, oct128_firmware, &x2->dev->dev) != 0) || !firmware) {
            printk("OpenVox VPM: firmware %s not available from userspace\n", oct128_firmware);
            return -2;
        }
#else
        embedded_firmware.data = _binary_dahdi_fw_oct6114_128_bin_start;
        embedded_firmware.size = (size_t) &_binary_dahdi_fw_oct6114_128_bin_size;
#endif
        break;
    default:
        printk("Unsupported channel capacity found on VPM module (%d).\n", vpm_capacity);
        return -2;
    }

    x2->vpm450m = init_vpm450m(x2, laws, vpm_capacity, firmware) ;
    if (!(x2->vpm450m)) {
        printk("OpenVox VPM: Failed to initialize\n");
        if (firmware != &embedded_firmware) {
            release_firmware(firmware);
        }
        return -3;
    }

    if (firmware != &embedded_firmware) {
        release_firmware(firmware);
    }

    x2->vpm_present = BIT_EC_PRESENT;
    spin_lock_irqsave(&x2->lock, flags);
    __x200_setcreg(x2, X200_PIO_BASE, 0, (BIT_EC_PRESENT<<16) | BIT_EC_PRESENT );
    spin_unlock_irqrestore(&x2->lock, flags);
    
    return 0;
}

static int __devinit x200_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct x200 *x2;
    unsigned int x, i;
    unsigned long flags;
    unsigned int fw_version, fw_io ;

    if (pci_enable_device(pdev)) {
        return -EIO;
    }

    pci_set_drvdata(pdev, NULL);

    if (!(x2 = kmalloc(sizeof(*x2), GFP_KERNEL))) {
        return -ENOMEM;
    }
    memset(x2, 0x0, sizeof(*x2));
    x2->selected_blade = -1;
    x2->reg_cs = -1;

    spin_lock_init(&x2->lock);
    x2->mem_region = pci_resource_start(pdev, 0);
    x2->mem_len = pci_resource_len(pdev, 0);
    x2->mem32 = (unsigned long)ioremap(x2->mem_region, x2->mem_len);
    x2->dev = pdev;

    /* We now know which version of card we have */

    x2->variety = ((struct x200_variant*)(ent->driver_data))->name;
    x2->flag = ((struct x200_variant*)(ent->driver_data))->flag;

    __x200_setcreg(x2, X200_PIO_BASE, 0, 0xffff0000);  // set all bit to 0.
    if(reset_fpga) {
        __x200_setcreg(x2,X200_PIO_BASE, PIO_REG4_DEBUG_POINT, 0x81);
        __x200_setcreg(x2, X200_PIO_BASE, PIO_REG2_TIMER_RESET, reset_fpga_delay);
        __x200_setcreg(x2,X200_PIO_BASE, PIO_REG4_DEBUG_POINT, 0x82);
        udelay(5);
        __x200_setcreg(x2,X200_PIO_BASE, PIO_REG4_DEBUG_POINT, 0x83);
    }

    fw_version = __x200_getcreg(x2, X200_REG_BASE, OPVX_REG_FWVERSION) ;
    fw_io =  __x200_getcreg(x2, X200_PIO_BASE, 0 ) ;
    printk(KERN_INFO "Found a x200, variant: %s, fw_version: 0x%08x, fw_io:0x%08x.\n", x2->variety,fw_version,fw_io);

    //  /* Set 128-timeslot to transparent mode */
    //  for (i=0;i<127;i++) {
    //     __x200_setcreg(x2, X200_TSI_EC_BASE, i, BIT16|BIT7);
    //  }

    x = x200_init_bus(x2);
    if(x) {
        iounmap((void*)x2->mem32);
        kfree(x2);
        return x;
    }
    printk(KERN_NOTICE "x200: init bus success\n");
    /* 128 channels, MAX_DMA_BUFFER_NUM, Read/Write */
    x2->writechunk =(unsigned char *)pci_alloc_consistent(pdev, 1024  * MAX_DMA_BUFFER_NUM * 2, &x2->writedma);
    if (!x2->writechunk) {
        printk(KERN_NOTICE "x200: Unable to allocate DMA-able memory\n");
        iounmap((void*)x2->mem32);
        kfree(x2);
        return -ENOMEM;
    }

    /* Read is after the whole write piece (in bytes) */
    x2->readchunk = x2->writechunk + 1024 * MAX_DMA_BUFFER_NUM;

    /* Same thing...  */
    x2->readdma = x2->writedma + 1024 * MAX_DMA_BUFFER_NUM;

    printk(KERN_NOTICE "x200: %d bytes DMA memory alloced, read at %p, write at %p, dma start 0x%x\n",
        1024 * MAX_DMA_BUFFER_NUM * 2, x2->writechunk, x2->readchunk, (unsigned int)x2->writedma);

    /* Initialize Write/Buffers to all blank data */
    memset((void *)x2->writechunk,0x00,1024 * MAX_DMA_BUFFER_NUM * 2);
    /* Enable bus mastering */
    pci_set_master(pdev);

    /* init ec module */
    if (!x2->vpm_present) {  
        if(x200_vpm_init(x2)){
        	printk(KERN_NOTICE "x200: x200_vpm_init error. vpm not work!\n");
        }
    }

    if (request_irq(pdev->irq, x200_interrupt, IRQF_SHARED, "x200", x2)) {
        printk(KERN_NOTICE "x200: Unable to request IRQ %d\n", pdev->irq);
        pci_free_consistent(pdev, 1024 * MAX_DMA_BUFFER_NUM * 2, (void *)x2->writechunk, x2->writedma);
        iounmap((void*)x2->mem32);
        kfree(x2);
        return -EIO;
    }

    x2->irq = pdev->irq;

    /* save the x2 to global interface array */
    spin_lock_irqsave(&interfaces_lock, flags);
    for(i=0; i<WC_MAX_INTERFACES; i++) {
        if( NULL==interfaces[i]) {
            interfaces[i] = x2;
            x2->bus_id = i;
            break;
        }
    }
    spin_unlock_irqrestore(&interfaces_lock, flags);

    if(i>=WC_MAX_INTERFACES) {
        printk("x200: too many interfaces installed\n");
        goto init_error;
    }
		
		x200_bus_setup(i);
		if(x200_bus_register(i))
		{
				goto init_error;
		}
		interface_num++;
		x200_dev_probe(x2);
		
    /* Keep track of which device we are */
    pci_set_drvdata(pdev, x2);
    __x200_enable_interrupt(x2);
    x200_start_dma(x2);
    return 0;
    
init_error:
		free_irq(pdev->irq, x2);
    pci_free_consistent(pdev, 1024 * MAX_DMA_BUFFER_NUM * 2, (void *)x2->writechunk, x2->writedma);
    iounmap((void*)x2->mem32);
    kfree(x2);
    return -EIO;
}

static void __devexit x200_remove_one(struct pci_dev *pdev)
{
    struct x200 *x2 = pci_get_drvdata(pdev);
    unsigned long flags;
    int i,j;
    
    if (x2) {
        /* Stop any DMA */
        x200_stop_dma(x2);
        __x200_disable_interrupt(x2);
        
		    /* wait before pending irq cleared */
		    msleep(200);   /* delay 200 ms*/

        /* Release echo canceller */
        if (x2->vpm450m) {
            release_vpm450m(x2->vpm450m);
        }
        x2->vpm450m = NULL;

				for(i=0; i<MAX_BLADE_NUM; i++)
				{
						if(x2->blades[i].exist)
						{
								for(j=0; j<x2->blades[i].max_cards; j++)
								{
										if(x2->devices[i][j].registered){
												device_unregister(&(x2->devices[i][j].dev));
												x2->devices[i][j].registered = 0;
										}
								}
						}
				}
				
        /* Immediately free resources */
        pci_free_consistent(pdev, 1024 * MAX_DMA_BUFFER_NUM * 2, (void *)x2->writechunk, x2->writedma);
        free_irq(pdev->irq, x2);

        iounmap((void*)x2->mem32);

        /* Release span, possibly delayed */
        pci_set_drvdata(x2->dev, NULL);
        
        spin_lock_irqsave(&interfaces_lock, flags);
        interfaces[x2->bus_id] = NULL;
        x200_bus_unregister(x2->bus_id);
        interface_num--;
        spin_unlock_irqrestore(&interfaces_lock, flags);
        
        kfree(x2);

        printk(KERN_INFO "Free a OpenVox x200 card\n");
    }
}

static struct pci_device_id x200_pci_tbl[] = {
    { 0x1172, 0x0004, 0x0000, 0x0000, 0, 0, (unsigned long) &x200_devel },
		{ 0x1b74, 0xc200, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long)&x204v1 },
    { 0 }
};

MODULE_DEVICE_TABLE(pci,x200_pci_tbl);

static struct pci_driver x200_driver = {
    .name = "x200",
    .probe = x200_init_one,
    .remove = __devexit_p(x200_remove_one),
    .suspend = NULL,
    .resume = NULL,
    .id_table = x200_pci_tbl,
};

static int __init x200_init(void)
{
    int res;

    memset(interfaces, 0, sizeof(interfaces));
    spin_lock_init(&interfaces_lock);

    res = dahdi_pci_module(&x200_driver);
    if (res)
        return -ENODEV;
    return 0;
}

static void __exit x200_cleanup(void)
{
    pci_unregister_driver(&x200_driver);
}


module_param(spi_fifo_mode, int, 0600);
module_param(x200_spi_mode, int, 0600);
module_param(spi_daisy_chain, int, 0600);
module_param(vpmsupport, int, 0600);
module_param(tdm_mode, int, 0600);

MODULE_DESCRIPTION("OpenVox x200 Driver");
MODULE_AUTHOR("Miao Lin <lin.miao@openvox.cn>");
MODULE_LICENSE("GPL v2");

module_init(x200_init);
module_exit(x200_cleanup);












