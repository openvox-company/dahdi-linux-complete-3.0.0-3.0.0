#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include "hfcopvx.h"


void OPVOS_MutexInit(OPVOS_mutex_t *mutex){
		if ( (*mutex =kmalloc(sizeof(struct semaphore), GFP_KERNEL)) != NULL )
      	sema_init((*mutex), 1);	
}

void OPVOS_MutexLock(OPVOS_mutex_t *mutex){
		down(*mutex);
}

void OPVOS_MutexLockInterruptible(OPVOS_mutex_t *mutex)
{
		down_interruptible(*mutex);	
}

void OPVOS_MutexUnlock(OPVOS_mutex_t *mutex) 
{ 
		up(*mutex);
}

void OPVOS_MutexDelete(OPVOS_mutex_t *mutex){
	  kfree(*mutex); 
	  *mutex = 0;
}

void OPVOS_Udelay(unsigned long usecs){
	udelay(usecs);
}
void OPVOS_Mdelay(unsigned long msecs){
	mdelay(msecs);	
}

void OPVOS_Ndelay(unsigned long nsecs){
	ndelay(nsecs);
}

