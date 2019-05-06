#include "x200_hal.h"
#include "x2d100m.h"
#include "x2b200m.h"
#include "x2fxo200m.h"
#include "x2fxs200m.h"

extern int spi_fifo_mode;
extern int spi_daisy_chain;

unsigned char spi_tx_pendings_local = 0x0;
unsigned char spi_tx_pendings_max = 0x80;
unsigned char spi_tx_pendings_min = 0x20;
static int while_loops_max = 0xc0;

void __wait_spi_wr_fifo(struct x200_dev *dev, unsigned char low_level ) {
    struct x200* x2 = dev->bus;
    volatile unsigned int reg_5h11;
    unsigned int while_loops ;
    int spi_tx_pendings_fpga ;

    while_loops = 0 ;
    while( 1 ) {
        while_loops++;
        reg_5h11 = __x200_getcreg(x2, X200_SPI_BASE, 0x11);
        spi_tx_pendings_fpga=(reg_5h11>>24)&0xff ;
        if ( while_loops > while_loops_max ) {
            printk("d100m:line %d,while_loops=0x%02x,reg_5h11=0x%08x\n", __LINE__, while_loops,reg_5h11);
            break ;
        }
        if ( spi_tx_pendings_fpga <= low_level ) {
            break ;
        }
    }
}
EXPORT_SYMBOL(__wait_spi_wr_fifo);

void wait_spi_wr_fifo(struct x200_dev *dev, unsigned char low_level ) {
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    __wait_spi_wr_fifo(dev, low_level);
    spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(wait_spi_wr_fifo);

 unsigned int __get_spi_fifo_data(struct x200_dev *dev, unsigned char size ) {
    struct x200* x2 = dev->bus;
    volatile unsigned int reg_5h11;
    volatile unsigned int read_data=0;
    unsigned int while_loops ;
    static unsigned char spi_rx_pendings_fpga = 0 ;  //use static value, avoid to read back every time.

    if ( size > 4 ) {  // valid input is 1,2,3,4
        printk(" error d100m:line %d,size=0x%02x error.\n",__LINE__, size);
    }

    while_loops = 0 ;
    while(spi_rx_pendings_fpga < size) {
        while_loops++;
        reg_5h11 = __x200_getcreg(x2, X200_SPI_BASE, 0x11);
        spi_rx_pendings_fpga = (reg_5h11>>16)&0xff ;
        if ( while_loops > while_loops_max ) {
            printk("d100m:line %d,while_loops=0x%02x,reg_5h11=0x%08x.\n",__LINE__, while_loops,reg_5h11);
            break ;
        }
    }
    spi_rx_pendings_fpga -= size ;
    read_data = __x200_getcreg(x2, X200_SPI_BASE, 0x07 + size ) ; //get data
    return read_data ;
}
EXPORT_SYMBOL(__get_spi_fifo_data);


unsigned int get_spi_fifo_data(struct x200_dev *dev, unsigned char size )
{
		struct x200 *x2 = dev->bus;
    unsigned long flags;
		volatile unsigned int read_data=0;
		
    spin_lock_irqsave(&x2->lock, flags);
    read_data = __get_spi_fifo_data(dev,size);
    spin_unlock_irqrestore(&x2->lock, flags);	
    return read_data;
}
EXPORT_SYMBOL(get_spi_fifo_data);


//////////For d100m ////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char d100m_read_reg(struct x200_dev *dev, unsigned char addr)
{
    unsigned char dat;
    struct x200 *x2 = dev->bus;
    unsigned long flags;
    unsigned char reg_cs;
    
    spin_lock_irqsave(&x2->lock, flags);
    // set blade
		if(x2->selected_blade != dev->blade_id) {
				if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
         
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
		
    // set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }
    if(spi_fifo_mode)
    	__wait_spi_wr_fifo(dev,0);  
      
    __x200_write_8bits(x2, CMD_RD_REG(addr));
    dat=__x200_read_8bits(x2);
    spin_unlock_irqrestore(&x2->lock, flags);
    return dat;
}
EXPORT_SYMBOL(d100m_read_reg);

void d100m_write_reg(struct x200_dev *dev, unsigned char addr,unsigned char value )
{
    struct x200 *x2 = dev->bus;
    unsigned long flags;
    unsigned char reg_cs;
    
    spin_lock_irqsave(&x2->lock, flags);
    // set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);

         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
		
    // set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }
    
    __x200_write_8bits(x2, CMD_WR_REG(addr));
    __x200_write_8bits(x2, value);
    spin_unlock_irqrestore(&x2->lock, flags);
    return ;
}
EXPORT_SYMBOL(d100m_write_reg);

void __d100m_write_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
    struct x200* x2 = dev->bus;
    int i,j;
    unsigned char reg_cs, reg_addr;
    unsigned int wdata;

    len &= 0x1f;
    if(0==len)
        return ;
		
		// set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
		
    // set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }

    //write out command   		
    __x200_write_8bits(x2, CMD_BWR_REG(len));
    __x200_write_8bits(x2, addr);

    //write out data, several 4bytes
    i=0 ;
    while ( (i+4) < len )  {
        wdata = (value[i+3]<<24) |(value[i+2]<<16) |(value[i+1]<<8) | value[i] ;
        reg_addr = 0xf ; //write data 4 bytes, with no back
        __x200_setcreg(x2,X200_SPI_BASE, reg_addr, wdata);   //write data 4 bytes, with no read back
        //printk("======line:%d,reg_addr=0x%02x,wdata=0x%08x.======\n",__LINE__,reg_addr,wdata);
        i+=4 ;
    }

    //write out data, lase 1~4 bytes
    wdata = 0 ;
    j=0 ;
    while ( (i+j) < len )  {
        wdata |=  (value[i+j])<<(8*j) ;
        j++ ;
    }
    reg_addr = 0xb+(len-i) ; //write data 1~4 bytes, with no back
    __x200_setcreg(x2,X200_SPI_BASE, reg_addr, wdata);   //write data 1~4 bytes, with no read back
    //printk("======line:%d,reg_addr=0x%02x,wdata=0x%08x.======\n",__LINE__,reg_addr,wdata);
    //__wait_spi_wr_fifo(dev,0);
    return ;
}
EXPORT_SYMBOL(__d100m_write_burst);

void d100m_write_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
		struct x200 *x2 = dev->bus;
    unsigned long flags;
    spin_lock_irqsave(&x2->lock, flags);
    __d100m_write_burst(dev,addr,len,value);
    spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(d100m_write_burst);

void __d100m_read_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
    struct x200* x2 = dev->bus;
    int i;
    unsigned char reg_cs, reg_addr;
    unsigned int wdata,rdata;

    len &= 0x1f;
    if(0==len)
        return ;
		
		// set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    
    // set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }

    //write out command
    if(spi_fifo_mode)
    		__wait_spi_wr_fifo(dev,0);  // make sure the previous command finished.
    		
    __x200_write_8bits(x2, CMD_BRD_REG(len));
    __x200_write_8bits(x2, addr);

    //write out data, several 4 bytes
    i=0 ;
    while ( (i+4) < len )  {
        reg_addr = 0xb ; //write data 4 bytes, with read back
        wdata=0 ;
        __x200_setcreg(x2,X200_SPI_BASE, reg_addr, wdata);
        i+=4 ;
    }
    //write out data, last 1~4 bytes
    reg_addr = 0x7+(len-i) ; //write data 1~4 bytes, with read back
    __x200_setcreg(x2,X200_SPI_BASE, reg_addr, wdata);   //write data 1~4 bytes, with no read back

    //get data to buf, several 4 bytes
    i=0 ;
    if(spi_fifo_mode)
    		__wait_spi_wr_fifo(dev,0);  // make sure the previous command finished.
    		
    while ( (i + 4) < len )  {
        reg_addr = 0xb ; //read 4 bytes,
        rdata = __x200_getcreg(x2, X200_SPI_BASE, reg_addr ) ;
        //printk("======line:%d,reg_addr=0x%02x,rdata=0x%08x.======\n",__LINE__,reg_addr, rdata);
        value[i+0] = (rdata>>0) & 0xff ;
        value[i+1] = (rdata>>8) & 0xff ;
        value[i+2] = (rdata>>16) & 0xff ;
        value[i+3] = (rdata>>24) & 0xff ;
        i+=4 ;
    }

    //get data to buf, last 1~4 bytes
    reg_addr = 0x7+(len-i) ; //read 1~4 bytes,
    rdata = __x200_getcreg(x2, X200_SPI_BASE,reg_addr ) ;
    //printk("======line:%d,reg_addr=0x%02x,rdata=0x%08x.======\n",__LINE__,reg_addr, rdata);
    while (i < len )  {
       value[i]= rdata & 0xff ;
       rdata >>= 8 ;
       i++ ;
    }
    return ;
}
EXPORT_SYMBOL(__d100m_read_burst);

void d100m_read_burst(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
		struct x200 *x2 = dev->bus;
    unsigned long flags;
    
    spin_lock_irqsave(&x2->lock, flags);
    __d100m_read_burst(dev,addr,len,value);
    spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(d100m_read_burst);

int d100m_burst_test(struct x200_dev *dev,  unsigned char *buf,  unsigned char buf_size )
{
    //struct x200* x2 = dev->bus;
    unsigned char i;
    unsigned char buf_read[256];
    unsigned char buf_offset;
    int test_ok;
    unsigned char bw_len = 0x1f;
    unsigned char br_len = 0x1f;

    buf_offset = 0x0 ;
    while( (buf_offset + bw_len) < buf_size ) {
        d100m_write_burst(dev,0x0,bw_len,&buf[buf_offset]);
        buf_offset += bw_len ;
    }
    d100m_write_burst(dev,0x0,buf_size - buf_offset,&buf[buf_offset]);
    //printk("D100M: burst write out is(len=0x%02x):%s\n",buf_size,buf);

    buf_offset = 0x0 ;
    while( (buf_offset + br_len) < buf_size ) {
        d100m_read_burst(dev,0x0,br_len,&buf_read[buf_offset]);
        buf_offset += br_len ;
    }
    d100m_read_burst(dev,0x0,buf_size - buf_offset,&buf_read[buf_offset]);
    //printk("D100M: burst read back is(len=0x%02x):",buf_size);

    test_ok=0 ;
    for(i=0; i<buf_size; i++) {
        if( buf_read[i] !=  buf[i]) {
            test_ok = 1 ;
        }
        //printk("%c",buf_read[i]);
    }
    //printk("\n");
    if(test_ok)
    		printk("d100m: spi burst write/read test is FAILURE.\n");
    			
    return test_ok;
}
EXPORT_SYMBOL(d100m_burst_test);

int __t1_framer_burst_in(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
    struct x200* x2 = dev->bus;
    int i;
    unsigned char reg_cs;
    unsigned int read_data;

    len &= 0x1f;
    if(0==len)
        return 0;

    if ( spi_fifo_mode ) {
    		// set blade
				if(x2->selected_blade != dev->blade_id) {
        	 __wait_spi_wr_fifo(dev,0);
         	 __x200_set_cs(x2, X200_CS_BROADCAST);
           __x200_write_8bits(x2, dev->blade_id);
           x2->selected_blade = dev->blade_id;
    		}
    		
        // set cs
        reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
        if(x2->reg_cs != reg_cs) {
            __wait_spi_wr_fifo(dev,0);
            __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
            x2->reg_cs = reg_cs;
        }

        __wait_spi_wr_fifo(dev,0);  // make sure the previous command finished.
        __x200_setcreg(x2,X200_SPI_BASE, 0x10, 0x0);   //reset the fifo
        
        __x200_write_8bits(x2, CMD_BRD_FALC(len));
        __x200_write_8bits(x2, addr);
                
        //write out data with readback,no wait
        i=0 ;
        while ( (i+4) < len )  {
            __x200_setcreg(x2,X200_SPI_BASE, 0xb, 0x0);
            i+=4 ;
        }
        __x200_setcreg(x2,X200_SPI_BASE, 0x7+(len-i), 0x0);

        //get data to buf
        i=0 ;
        __wait_spi_wr_fifo(dev,0);
        while ( (i + 4) < len )  {
            read_data = __x200_getcreg(x2, X200_SPI_BASE, 0xb ) ;
            value[i+0] = (read_data>>0) & 0xff ;
            value[i+1] = (read_data>>8) & 0xff ;
            value[i+2] = (read_data>>16) & 0xff ;
            value[i+3] = (read_data>>24) & 0xff ;
            i+=4 ;
        }
        read_data = __x200_getcreg(x2, X200_SPI_BASE, 0x7+(len-i) ) ;
        while (i < len )  {
           value[i]= read_data & 0xff ;
           read_data >>= 8 ;
           i++ ;
        }
    } else {
        __x200_set_cs(x2, X200_CS_CARD | (dev->slot_id&0x1f));
        __x200_write_8bits(x2, CMD_BRD_FALC(len));
        __x200_write_8bits(x2, addr);

        for(i=0; i<len; i++) {
            value[i] = __x200_read_8bits(x2);
        }
    }
    return len;
}
EXPORT_SYMBOL(__t1_framer_burst_in);

int t1_framer_burst_in(struct x200_dev *dev, unsigned char addr, unsigned int len, unsigned char *value)
{
		struct x200 *x2 = dev->bus;
    unsigned long flags;
    spin_lock_irqsave(&x2->lock, flags);
    __t1_framer_burst_in(dev, addr, len, value);
    spin_unlock_irqrestore(&x2->lock, flags);
    return len;
}
EXPORT_SYMBOL(t1_framer_burst_in);

unsigned int __t1_framer_in(struct x200_dev *dev, const unsigned int reg)
{
	unsigned char dat=0;
    struct x200* x2 = dev->bus;
   unsigned char reg_cs;
    
    // set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f);
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_set_cs(x2, reg_cs);
        x2->reg_cs = reg_cs;
    }
    
    if(spi_fifo_mode)
    		__wait_spi_wr_fifo(dev,0);    
    
    __x200_write_8bits(x2, CMD_RD_FALC);
    __x200_write_8bits(x2, reg);
    dat=__x200_read_8bits(x2);
	return dat;
}
EXPORT_SYMBOL(__t1_framer_in);

unsigned int t1_framer_in(struct x200_dev *dev, const unsigned int addr)
{
	unsigned long flags;
	unsigned int ret;
    struct x200* x2 = dev->bus;

	spin_lock_irqsave(&x2->lock, flags);
	ret = __t1_framer_in(dev, addr);
	spin_unlock_irqrestore(&x2->lock, flags);
	return ret;
}
EXPORT_SYMBOL(t1_framer_in);

void __t1_framer_out(struct x200_dev *dev, const unsigned int reg, const unsigned int val)
{
    struct x200 *x2 = dev->bus;
    unsigned char reg_cs;
    
    // set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f);
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
    				__wait_spi_wr_fifo(dev,0);
    				
        __x200_set_cs(x2, reg_cs);
        x2->reg_cs = reg_cs;
    }
    
    __x200_write_8bits(x2, CMD_WR_FALC);
    __x200_write_8bits(x2, reg);
    __x200_write_8bits(x2, val);

#if 0
    {   unsigned int tmp;
	    tmp = __t1_framer_in(dev, reg);
	    if (tmp != val) {
		    printk(KERN_DEBUG "Expected 0x%02x from  register:0x%02x but got 0x%02x instead\n", val, reg, tmp);
	    }
    }
#endif
}
EXPORT_SYMBOL(__t1_framer_out);

void t1_framer_out(struct x200_dev *dev, const unsigned int addr, const unsigned int value)
{
	unsigned long flags;
  struct x200* x2 = dev->bus;

	spin_lock_irqsave(&x2->lock, flags);
	__t1_framer_out(dev, addr, value);
	spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(t1_framer_out);

static void set_offset(struct x200_dev *dev)
{
    int rxstart = 0, txstart=0;
    int t = 0;

    if(t>=0 && t<=4) {
        rxstart = 4-t;
        txstart = 4-t;
    } else {
        rxstart = 2052 - t;
        //rxstart = 1023 - t;
        txstart = 1023 - t +4;
    }

    t1_framer_out(dev, 0x22, (txstart>>8)&0x7);	/* XC0: Normal operation of Sa-bits */
	  t1_framer_out(dev, 0x23, (txstart&0xff));	/* XC1: 0 offset */
	  t1_framer_out(dev, 0x24, (rxstart>>8)&0x7);	/* RC0: Just shy of 255 */
	  t1_framer_out(dev, 0x25, (rxstart&0xff));	/* RC1: The rest of RC0 */
}

void serial_setup(struct x200_dev* dev)
{
	t1_framer_out(dev, 0x85, 0xe0);	/* GPC1: Multiplex mode enabled, FSC is output, active low, RCLK from channel 0 */
	t1_framer_out(dev, 0x08, 0x05);	/* IPC: Interrupt push/pull active low */

    /* Global clocks (8.192 Mhz CLK)*/ 
	t1_framer_out(dev, 0x92, 0x00);
	t1_framer_out(dev, 0x93, 0x18);
	t1_framer_out(dev, 0x94, 0xfb);
	t1_framer_out(dev, 0x95, 0x0b);
	t1_framer_out(dev, 0x96, 0x00);
	t1_framer_out(dev, 0x97, 0x0b);
	t1_framer_out(dev, 0x98, 0xdb);
	t1_framer_out(dev, 0x99, 0xdf);
	
	/* Configure interrupts */
	t1_framer_out(dev, 0x46, 0x40);	/* GCR: Interrupt on Activation/Deactivation of AIX, LOS */

	/* Configure system interface */
	t1_framer_out(dev, 0x3e, 0xc2);	/* SIC1: 8.192 Mhz clock/bus, double buffer receive / transmit, byte interleaved */
	//t1_framer_out(dev, 0x3f, 0x00); /* SIC2: No FFS, no center receive eliastic buffer, phase 0 */
  t1_framer_out(dev, 0x3f, (dev->slot_id & 0x07)<<1 ); /* SIC2: No FFS, no center receive eliastic buffer, which phase */

#if 0	
	if (debug)
			printk("d100m: setting tdm slot start %d\n", dev->slot_id);
#endif

	t1_framer_out(dev, 0x40, 0x04 | 0x20 /* 0x20 is for highz rdo out*/);	/* SIC3: Edges for capture */
	t1_framer_out(dev, 0x44, 0x30);	/* CMR1: RCLK is at 8.192 Mhz dejittered */
	t1_framer_out(dev, 0x45, 0x00);	/* CMR2: We provide sync and clock for tx and rx. */

  set_offset(dev);

	/* Configure ports */
	t1_framer_out(dev, 0x80, 0x00);	/* PC1: SPYR/SPYX input on RPA/XPA */
	t1_framer_out(dev, 0x81, 0x22);	/* PC2: RMFB/XSIG output/input on RPB/XPB */
	t1_framer_out(dev, 0x82, 0x65);	/* PC3: Some unused stuff */
	t1_framer_out(dev, 0x83, 0x35);	/* PC4: Some more unused stuff */
	t1_framer_out(dev, 0x84, 0x31);	/* PC5: XMFS active low, SCLKR is input, RCLK is output */
	t1_framer_out(dev, 0x86, 0x03);	/* PC6: CLK1 is Tx Clock output, CLK2 is 8.192 Mhz from DCO-R */
	t1_framer_out(dev, 0x3b, 0x00);	/* Clear LCR1 */
}
EXPORT_SYMBOL(serial_setup);

/////////For b200m////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char __b200m_read_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char read_immediately)
{
    unsigned char dat=0;
    struct x200* x2 = dev->bus;
    unsigned int wdata;
    volatile unsigned int rdata=0;
    unsigned char reg_cs;			//,rx_cnt

    if ( spi_fifo_mode ) {
        if(x2->selected_blade != dev->blade_id) {
            __wait_spi_wr_fifo(dev,0);
            __x200_set_cs(x2, X200_CS_BROADCAST);
            __x200_write_8bits(x2, dev->blade_id);
            x2->selected_blade = dev->blade_id;
        }
        reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
        if(x2->reg_cs != reg_cs) {
            __wait_spi_wr_fifo(dev,0);
            __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
            x2->reg_cs = reg_cs;
        }
        
        if( read_immediately == 1 )     
        	  __wait_spi_wr_fifo(dev,0);
        	  
        //__x200_setcreg(x2,X200_SPI_BASE, 0xc, 0x1);      //reset fifo
        wdata =  (0x0<<24) + (CMD_RD_DATA<<16) + (addr<<8) + CMD_WR_ADDR ;
        __x200_setcreg(x2,X200_SPI_BASE, 0xb, wdata);   //write data

        spi_tx_pendings_local+=4 ;
        if( spi_tx_pendings_local > spi_tx_pendings_max ) {
            __wait_spi_wr_fifo(dev,spi_tx_pendings_min);
        }

        if( read_immediately == 1 ) {
            __wait_spi_wr_fifo(dev,0);
            rdata = __x200_getcreg(x2, X200_SPI_BASE, 0xb) ; //get data
            dat = (rdata>>24) & 0xff ;
        }
    } else {
        __x200_select_blade(x2, dev->blade_id);
        __x200_set_cs(x2, X200_CS_CARD | (dev->slot_id&0x1f));
        __x200_write_8bits(x2, CMD_WR_ADDR);    /* write command byte */
        __x200_write_8bits(x2, addr);           /* send register address */
        __x200_write_8bits(x2, CMD_RD_DATA);    /* write command byte */
        dat=__x200_read_8bits(x2);              /* read data */
    }

    return dat;
}
EXPORT_SYMBOL(__b200m_read_xhfc);

unsigned char b200m_read_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char read_immediately)
{
    unsigned char dat;
    struct x200 *x2 = dev->bus;
    unsigned long flags;
    spin_lock_irqsave(&x2->lock, flags);
    dat = __b200m_read_xhfc(dev, addr,read_immediately);
    spin_unlock_irqrestore(&x2->lock, flags);
    /* printk("b200m: b200m_read_xhfc reg 0x%02x get 0x%02x\n", addr&0xff, dat&0xff); */
    return dat;
}
EXPORT_SYMBOL(b200m_read_xhfc);

void __b200m_write_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char value)
{
    struct x200* x2 = dev->bus;
    unsigned int wdata;
    unsigned char reg_cs;
    
    if ( spi_fifo_mode  ) {
        if(x2->selected_blade != dev->blade_id) {
            __wait_spi_wr_fifo(dev,0);
            __x200_set_cs(x2, X200_CS_BROADCAST);
            __x200_write_8bits(x2, dev->blade_id);
            x2->selected_blade = dev->blade_id;
        }
        reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
        if(x2->reg_cs != reg_cs) {
            __wait_spi_wr_fifo(dev,0);
            __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
            x2->reg_cs = reg_cs;
        }
               
        //__x200_setcreg(x2,X200_SPI_BASE, 0xc, 0x1);       //reset fifo
        wdata =  (value<<24) + (0x0<<16) + (addr<<8) + CMD_WR_ADDR ;
        __x200_setcreg(x2,X200_SPI_BASE, 0xf, wdata);   //write data
        spi_tx_pendings_local+=4 ;
        if( spi_tx_pendings_local > spi_tx_pendings_max ) {
            __wait_spi_wr_fifo(dev,spi_tx_pendings_min);
        }
    } else {
        __x200_select_blade(x2, dev->blade_id);
        __x200_set_cs(x2, X200_CS_CARD | (dev->slot_id&0x1f));
        __x200_write_8bits(x2, CMD_WR_ADDR);    /* write command byte */
        __x200_write_8bits(x2, addr);           /* send register address */
        __x200_write_8bits(x2, 0);              /* send register address */
        __x200_write_8bits(x2, value);          /* write command byte */
    }
}
EXPORT_SYMBOL(__b200m_write_xhfc);

void b200m_write_xhfc(struct x200_dev *dev, unsigned char addr, unsigned char value)
{
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    __b200m_write_xhfc(dev, addr, value);
    spin_unlock_irqrestore(&x2->lock, flags);
    /* printk("b200m: b200m_write_xhfc reg 0x%02x value 0x%02x\n", addr&0xff, value&0xff); */
}
EXPORT_SYMBOL(b200m_write_xhfc);

void b200m_write_xhfc_ra(struct x200_dev *dev, unsigned char r, unsigned char rd, unsigned char addr, unsigned char value)
{
    //unsigned char dat;
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    __b200m_write_xhfc(dev, r, rd);
    __b200m_write_xhfc(dev, addr, value);
    spin_unlock_irqrestore(&x2->lock, flags);
    /* printk("b200m: b200m_read_xhfc reg 0x%02x get 0x%02x\n", addr&0xff, dat&0xff); */
}
EXPORT_SYMBOL(b200m_write_xhfc_ra);

//////////For fxo200m//////////////////////////////////////////////////////////////////////////////////////////////////////////

int __x2fxo200m_hit_fxo_daisy(int number_daisy)
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

unsigned char __fxo200m_read_si3050reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
    unsigned char retval;
    unsigned char reg_cs;

    // set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
         		__wait_spi_wr_fifo(dev,0);
         
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    //set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
        		__wait_spi_wr_fifo(dev,0);
        
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }
    
    if(spi_fifo_mode)
    		__wait_spi_wr_fifo(dev,0);
    
    __x200_write_8bits(x2, 0x60 | __x2fxo200m_hit_fxo_daisy(chanx%CHANS_PER_MODULE));   //1. send read command byte to port
    __x200_write_8bits(x2, addr & 0x7f);                                                //2. send the register address to port
    retval=__x200_read_8bits(x2);                                                       //3. read the value to port
    if(0)
        printk("module-debug: si3050 chip chanx=%d chip[%d] addr=0x%x read=0x%x \n", chanx, chanx%CHANS_PER_MODULE, addr, retval);
    return retval;
}
EXPORT_SYMBOL(__fxo200m_read_si3050reg);

unsigned char fxo200m_read_reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx)
{
    unsigned char dat;
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    dat = __fxo200m_read_si3050reg(dev, addr, chanx);
    spin_unlock_irqrestore(&x2->lock, flags);
    return dat;
}
EXPORT_SYMBOL(fxo200m_read_reg);

void __fxo200m_write_si3050reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
    unsigned char reg_cs;
    
    //set blade
    if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
         		__wait_spi_wr_fifo(dev,0);
         		
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    //set cs
    reg_cs = X200_CS_CARD | (dev->slot_id&0x1f) ;
    if(x2->reg_cs != reg_cs) {
        if(spi_fifo_mode)
        		__wait_spi_wr_fifo(dev,0);
        		
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }
       
    //1. send write command byte to port
    __x200_write_8bits(x2, 0x20 | __x2fxo200m_hit_fxo_daisy(chanx%CHANS_PER_MODULE));  // fxo daisy operate.
    //2. send the register address to port
    __x200_write_8bits(x2, addr);
    udelay(X2FXO200M_SPI_DELAY);        //fixed the issue: hooksig and isr both write si3050 cause sillicon abnormal
    //3. write the value to port
    __x200_write_8bits(x2, dat);
    udelay(X2FXO200M_SPI_DELAY);        //fixed the issue: hooksig and isr both write si3050 cause sillicon abnormal
    if(0) {     /*miaolin */
        unsigned char c = __fxo200m_read_si3050reg(dev, addr, chanx);
        if(c!=dat)
            printk("=======read is 0x%02x, write is 0x%02x, addr 0x%02x\n", c, dat, addr);
    }
}
EXPORT_SYMBOL(__fxo200m_write_si3050reg);

void fxo200m_write_reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    __fxo200m_write_si3050reg(dev, addr, dat, chanx);
    spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(fxo200m_write_reg);



////////For fxs200m////////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char __fxs200m_read_si3215reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
	  unsigned char retval;
	  unsigned char reg_cs;
	
	  if (chanx > CHANS_PER_MODULE )  
		  printk("error at line:%d \n", __LINE__); //chanx should little than CHANS_PER_MODULE

    // set blade
		if(x2->selected_blade != dev->blade_id) {
				 if(spi_fifo_mode)
         		__wait_spi_wr_fifo(dev,0);
         		
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    
    //set cs
    if ( spi_daisy_chain ) {
    	 reg_cs = X200_CS_CARD | dev->slot_id;
    }else{
    	 reg_cs = X200_CS_CARD |(chanx<<3)| dev->slot_id;  //card[4:3]:chan_id, card[2:0]:card_id
    }
    if(x2->reg_cs != reg_cs) {
    	  if(spi_fifo_mode)
        	__wait_spi_wr_fifo(dev,0);
        
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs);
        x2->reg_cs = reg_cs;
    }
    
    if(spi_fifo_mode)
    		__wait_spi_wr_fifo(dev,0);
    
    if ( spi_daisy_chain ) {
        __x200_write_8bits(x2,1<<chanx );
    } 
    
    __x200_write_8bits(x2, (addr|0x80));                    //send the register address to port
	  retval=__x200_read_8bits(x2);                           //read the value to port
    return retval;
}
EXPORT_SYMBOL(__fxs200m_read_si3215reg);

unsigned char fxs200m_read_reg(struct x200_dev *dev, unsigned char addr, unsigned int chanx)
{
    unsigned char dat;
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    dat = __fxs200m_read_si3215reg(dev, addr, chanx);
    spin_unlock_irqrestore(&x2->lock, flags);
    return dat;
}
EXPORT_SYMBOL(fxs200m_read_reg);

void __fxs200m_write_si3215reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
    unsigned char reg_cs;
    
	  if (chanx > CHANS_PER_MODULE )  
	  	printk("error at line:%d \n", __LINE__); //chanx should little than CHANS_PER_MODULE
    
    // set blade
		if(x2->selected_blade != dev->blade_id) {
         if(spi_fifo_mode)
         		__wait_spi_wr_fifo(dev,0);
         
         __x200_set_cs(x2, X200_CS_BROADCAST);
         __x200_write_8bits(x2, dev->blade_id);
         x2->selected_blade = dev->blade_id;
    }
    
    //set cs
    if ( spi_daisy_chain ) {
    		reg_cs = X200_CS_CARD | dev->slot_id;
    }else{
    		reg_cs = X200_CS_CARD |(chanx<<3)| dev->slot_id;
    }
    if(x2->reg_cs != reg_cs) {
    		if(spi_fifo_mode)
          __wait_spi_wr_fifo(dev,0);
        
        __x200_setcreg(x2, X200_SPI_BASE, OPVX_SPI_CS, reg_cs); //card[4:3]:chan_id, card[2:0]:card_id
        x2->reg_cs = reg_cs;
    }
    
    if ( spi_daisy_chain ) {
        __x200_write_8bits(x2,1<<chanx);
    } 
    
    __x200_write_8bits(x2, (addr&0x7f));    // send the register address to port
    udelay(X2FXS200M_SPI_DELAY);		    //fixed the issue: hooksig and isr both write si3050 cause sillicon abnormal
	  __x200_write_8bits(x2, dat);            //write the value to port
    udelay(X2FXS200M_SPI_DELAY);		    //fixed the issue: hooksig and isr both write si3050 cause sillicon abnormal
}
EXPORT_SYMBOL(__fxs200m_write_si3215reg);

void fxs200m_write_reg(struct x200_dev *dev, unsigned char addr, unsigned char dat, unsigned int chanx)
{
    struct x200 *x2 = dev->bus;
    unsigned long flags;

    spin_lock_irqsave(&x2->lock, flags);
    __fxs200m_write_si3215reg(dev, addr, dat, chanx);
    spin_unlock_irqrestore(&x2->lock, flags);
}
EXPORT_SYMBOL(fxs200m_write_reg);

