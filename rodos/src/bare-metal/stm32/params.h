

/*********************************************************** Copyright 
 **
 ** Copyright (c) 2008, German Aerospace Center (DLR)
 ** All rights reserved.
 ** 
 ** Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 ** 
 ** 1 Redistributions of source code must retain the above copyright
 **   notice, this list of conditions and the following disclaimer.
 ** 
 ** 2 Redistributions in binary form must reproduce the above copyright
 **   notice, this list of conditions and the following disclaimer in the
 **   documentation and/or other materials provided with the
 **   distribution.
 ** 
 ** 3 Neither the name of the German Aerospace Center nor the names of
 **   its contributors may be used to endorse or promote products derived
 **   from this software without specific prior written permission.
 ** 
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **  
 ****************************************************************************/


/**
* @file params.h
* @date 2008/04/24 16:13
* @author Lutz Dittrich, Sergio Montenegro
*
* Copyright 2008 DLR
*
* @brief configuration parameters ...
*
*/

#ifndef __PARAMS_H__
#define __PARAMS_H__

// #include "hw_datatypes.h"


#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

/** Version Number */
#define OSVERSION "STM32F103 Cortex M3 V1.0"

/*************** System Configuration *********/
/** the following define sets the UART used for debug outputs with xprintf **/
#define UART_DEBUG             UART_IDX2

/** Memory for allocation (xmalloc) eg for all stacks ***/
#ifdef STM32F10X_MD
	#define XMALLOC_SIZE		10*1024
#elif STM32F10X_HD
	#define XMALLOC_SIZE		40*1024
#elif STM32F10X_CL
	#define XMALLOC_SIZE		40*1024
#endif

/** default stack size (in bytes) for threads */
#ifdef STM32F10X_MD
	#define DEFAULT_STACKSIZE       1000
#elif STM32F10X_HD
	#define DEFAULT_STACKSIZE       2000
#elif STM32F10X_CL
	#define DEFAULT_STACKSIZE       2000
#endif

/** stack size (in bytes) for scheduler thread */
#define SCHEDULER_STACKSIZE DEFAULT_STACKSIZE

/** time interval between timer interrupts in microseconds - max. 233000us (if CPU_CLK==72MHz)!!! */
#define PARAM_TIMER_INTERVAL   10000

/*** time for time slice to swtich between threads with same priority ***/
#define  TIME_SLICE_FOR_SAME_PRIORITY (100*MILLISECONDS)

/** default priority for newly created threads */
#define DEFAULT_THREAD_PRIORITY                    100

/** user threads shall not have a priority higher than this */
#define MAX_THREAD_PRIORITY		       1000

/** high priority levels for priority ceiling  */
#define NETWORKREADER_PRIORITY	   	(MAX_THREAD_PRIORITY + 2)
/** high priority levels for priority ceiling  */
#define CEILING_PRIORITY			(NETWORKREADER_PRIORITY + 1)



/** using a network, the maximal number of nodes attached */
#define MAX_NUMBER_OF_NODES		            10
/** if using network it may have a limit for pakets, eg udp 1500 */
#define MAX_NETWORK_MESSAGE_LENGTH              1300


/*** If using a network: Maximal number of subscribers per node */
#define MAX_SUBSCRIBERS							60


#undef  stdout
#define stdout 
#define fflush nop


#ifndef NO_RODOS_NAMESPACE
}
#endif

#endif /* __PARAMS_H__ */
