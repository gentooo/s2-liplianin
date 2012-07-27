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
#ifndef SMI2032_INT_H
#define SMI2032_INT_H

/* Macro define for DMA engine selection */
#define USE_PORT0_CHAN0 (0x00)
#define USE_PORT0_CHAN1 (0x01)
#define USE_PORT1_CHAN0 (0x02)
#define USE_PORT1_CHAN1 (0x03)

/* Macro define for each TLP transfer length */
#define DMA_TRANS_UNIT_4   (0x00000000)
#define DMA_TRANS_UNIT_8   (0x00000001)
#define DMA_TRANS_UNIT_16  (0x00000002)
#define DMA_TRANS_UNIT_32  (0x00000003)
#define DMA_TRANS_UNIT_64  (0x00000004)
#define DMA_TRANS_UNIT_128 (0x00000005)
#define DMA_TRANS_UNIT_256 (0x00000006)
#define DMA_TRANS_UNIT_188 (0x00000007)

/* Macro define of 24 interrupt resource */
#define DMA_A_CHAN0_DONE_INT   (0x00000001)
#define DMA_A_CHAN1_DONE_INT   (0x00000002)
#define DMA_B_CHAN0_DONE_INT   (0x00000004)
#define DMA_B_CHAN1_DONE_INT   (0x00000008)
#define DMA_C_CHAN0_DONE_INT   (0x00000010)
#define DMA_C_CHAN1_DONE_INT   (0x00000020)
#define DMA_D_CHAN0_DONE_INT   (0x00000040)
#define DMA_D_CHAN1_DONE_INT   (0x00000080)
#define DATA_BUF_OVERFLOW_INT  (0x00000100)
#define UART_0_X_INT           (0x00000200)
#define UART_1_X_INT           (0x00000400)
#define IR_X_INT               (0x00000800)
#define GPIO_0_INT             (0x00001000)
#define GPIO_1_INT             (0x00002000)
#define GPIO_2_INT             (0x00004000)
#define GPIO_3_INT             (0x00008000)
#define ALL_INT                (0x0000FFFF)

#endif

