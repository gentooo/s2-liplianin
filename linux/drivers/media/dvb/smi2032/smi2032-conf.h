/*
 * Driver for the Somagic SMI2032 PCIe bridge
 *
 * Copyright (c) 2009 Igor M. Liplianin <liplianin@me.by>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef SMI2032_CONF_H
#define SMI2032_CONF_H

/*#include "device.h" */

/* define for dual port D+D bda driver */
/*#define  PORT_B_IN_USE*/

/*
 Interrupt connect/disconnect configuration:

 this flag controls hardware interrupt and driver ISR routine, when defined to 1, driver will
 never go to ISR routine even hardware send interrupt message/packet to PC
 */
#define DISCONNECT_INTERRUPT (0)

#ifdef PORT_B_IN_USE  /* enable both port a and port b */
#define SMI_DMA_WORK_MANNER  PaC0C1_PbC0C1// enum declare in smiDrv.h; used in SmiDrvStartDevice()
#else
/* only enable port A */
#define SMI_DMA_WORK_MANNER  PaC0C1
#endif

/*
 Storage buffer size configuration:
 due to two application ports in hardware, so we need two storage buffer for data storing
 */
#define STORAGE_BUFFER_SIZE_PORT_A ( 1024 * 188 * 24 ) /*(1440*8192)*/
#define STORAGE_BUFFER_SIZE_PORT_B ( 1024 * 188 * 24 )


/*
 DMA mapped memory size for port0 and port1:
 due to we only create one DMA adapter during driver initialize and we only have one block
 of mapped memory space, so we define two memory size for both port0 and port1, then
 merged two block size into one mapped space
 */
#define DMA_MAPPED_MEMORY_SIZE_PORTA_CHAN0 ( 1024 * 188 * 1) /*(1440*1024)*/
#define DMA_MAPPED_MEMORY_SIZE_PORTA_CHAN1 ( 1024 * 188 * 1)
#define DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0 ( 1024 * 188 * 1)
#define DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN1 ( 1024 * 188 * 1)

#define DMA_MAPPED_MEMORY_SIZE_TOTAL  (DMA_MAPPED_MEMORY_SIZE_PORTA_CHAN0 \
                                       + DMA_MAPPED_MEMORY_SIZE_PORTA_CHAN1 \
                                       + DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0 \
                                       + DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN1)


/*
 DMA channel latency timer set
 latency timer indicates the timeout value when infront-end design has no data
 */
#define DMA_PORT0_CHAN0_LATENCY_TIMER (0)
#define DMA_PORT0_CHAN1_LATENCY_TIMER (0)
#define DMA_PORT1_CHAN0_LATENCY_TIMER (0)
#define DMA_PORT1_CHAN1_LATENCY_TIMER (0)


#endif
