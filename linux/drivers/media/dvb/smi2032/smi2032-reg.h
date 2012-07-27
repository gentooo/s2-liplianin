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

#ifndef SMI2032_REG_H
#define SMI2032_REG_H

/* Register Base */
/* according to CSR_PCIe_MPW3_Oct25.xls */
#define    MSI_CONTROL_REG_BASE		0x0800
#define    SYSTEM_CONTROL_REG_BASE	0x0880
#define    PCIE_EP_DEBUG_REG_BASE	0x08C0
#define    IR_CONTROL_REG_BASE		0x0900
/* #define    SPI_CONTROL_REG_BASE	0x0940 */
#define    I2C_A_CONTROL_REG_BASE	0x0940
#define    I2C_B_CONTROL_REG_BASE	0x0980
#define    ATV_PORTA_CONTROL_REG_BASE	0x09C0
#define    DTV_PORTA_CONTROL_REG_BASE	0x0A00
#define    DMA_PORTA_CONTROL_REG_BASE	0x0AC0
#define    ATV_PORTB_CONTROL_REG_BASE	0x0B00
#define    DTV_PORTB_CONTROL_REG_BASE	0x0B40
#define    DMA_PORTB_CONTROL_REG_BASE	0x0C00
#define    UART_A_REGISTER_BASE		0x0C40
#define    UART_B_REGISTER_BASE		0x0C80
#define    GPS_CONTROL_REG_BASE		0x0CC0
#define    DMA_PORTC_CONTROL_REG_BASE	0x0D00
#define    DMA_PORTD_CONTROL_REG_BASE	0x0D00
#define    RANDOM_DATA_LIB_BASE		0x0E00
#define    IR_DATA_BUFFER_BASE		0x0F00
#define    PORTA_TS_BUFFER_BASE		0x1000
#define    PORTA_I2S_BUFFER_BASE	0x1400
#define    PORTB_TS_BUFFER_BASE		0x1800
#define    PORTB_I2S_BUFFER_BASE	0x1C00

/* MSI control and state register */
#define MSI_DELAY_TIMER		(MSI_CONTROL_REG_BASE + 0x00)

#define MSI_INT_STATUS		(MSI_CONTROL_REG_BASE + 0x08)
#define MSI_INT_STATUS_CLR	(MSI_CONTROL_REG_BASE + 0x0C)
#define MSI_INT_STATUS_SET	(MSI_CONTROL_REG_BASE + 0x10)

#define MSI_INT_ENA		(MSI_CONTROL_REG_BASE + 0x14)
#define MSI_INT_ENA_CLR		(MSI_CONTROL_REG_BASE + 0x18)
#define MSI_INT_ENA_SET		(MSI_CONTROL_REG_BASE + 0x1C)

#define MSI_SOFT_RESET		(MSI_CONTROL_REG_BASE + 0x20)
#define MSI_CFG_SRC0		(MSI_CONTROL_REG_BASE + 0x24)

/* Hybird Controller System Control register */
/* clear mask bits to 0 */
#define  CLR_MASK_BITS(data, rbMask)    (data) = ( (data)&(~(rbMask)) )

#define MUX_MODE_CTRL		(SYSTEM_CONTROL_REG_BASE + 0x00)
#define rbPaMSMask		0x07
#define rbPaMSDtvNoGpio 	0x00 //[2:0], DTV Simple (One Serial or Parallel) mode selected; (No GPIO)
#define rbPaMSDtv4bitGpio	0x01 //[2:0], DTV TS2 Serial mode selected;  (4bit GPIO)
#define rbPaMSDtv7bitGpio	0x02 //[2:0], DTV TS0 Serial mode selected;  (7bit GPIO)
#define rbPaMS8bitGpio		0x03 //[2:0], GPIO mode selected; (8bit GPIO)
#define rbPaMSAtv		0x04 //[2:0], 3'b1xx: ATV mode select

#define rbPbMSMask		0x38
#define rbPbMSDtvNoGpio 	0x00 //[5:3], DTV Simple (One Serial or Parallel) mode selected; (No GPIO)
#define rbPbMSDtv4bitGpio	0x08 //[5:3], DTV TS2 Serial mode selected;  (4bit GPIO)
#define rbPbMSDtv7bitGpio	0x10 //[5:3], DTV TS0 Serial mode selected;  (7bit GPIO)
#define rbPbMS8bitGpio		0x18 //[5:3], GPIO mode selected; (8bit GPIO)
#define rbPbMSAtv		0x20 //[5:3], 3'b1xx: ATV mode select

#define INTERNAL_RST		(SYSTEM_CONTROL_REG_BASE + 0x04)
#define PERIPHERAL_CTRL		(SYSTEM_CONTROL_REG_BASE + 0x08)
#define GPIO_0to7_CTRL		(SYSTEM_CONTROL_REG_BASE + 0x0C)
#define GPIO_8to15_CTRL		(SYSTEM_CONTROL_REG_BASE + 0x10)
#define GPIO_16to23_CTRL	(SYSTEM_CONTROL_REG_BASE + 0x14)
#define GPIO_INT_SRC_CFG	(SYSTEM_CONTROL_REG_BASE + 0x18)
#define SYS_BUF_STATUS		(SYSTEM_CONTROL_REG_BASE + 0x1C)
#define PCIE_IP_REG_ACS		(SYSTEM_CONTROL_REG_BASE + 0x20)
#define PCIE_IP_REG_ACS_ADDR	(SYSTEM_CONTROL_REG_BASE + 0x24)
#define PCIE_IP_REG_ACS_DATA	(SYSTEM_CONTROL_REG_BASE + 0x28)

/* PCIE EP Debug register */

/* IR Control register */
#define IR_Init_Reg		(IR_CONTROL_REG_BASE + 0x00)//0x0900+0x00
#define IR_Idle_Cnt_Low		(IR_CONTROL_REG_BASE + 0x04)//0x0900+0x04
#define IR_Idle_Cnt_High	(IR_CONTROL_REG_BASE + 0x05)//0x0900+0x05
#define IR_Unit_Cnt_Low		(IR_CONTROL_REG_BASE + 0x06)//0x0920+0x06
#define IR_Unit_Cnt_High	(IR_CONTROL_REG_BASE + 0x07)//0x0920+0x07
#define IR_Data_Cnt		(IR_CONTROL_REG_BASE + 0x08)// 0x0920+0x08

#define rbIRen		0x80
#define rbIRhighidle	0x10
#define rbIRlowidle	0x00
#define rbIRVld		0x04

/* SoftRest */
#define   rbSOFRST	 0x01

/* SPI control and state register */
/*
#define SPI_CONTROL	(SPI_CONTROL_REG_BASE + 0x00)
#define SPI_INT_ENABLE	(SPI_CONTROL_REG_BASE + 0x04)
#define SPI_DATA	(SPI_CONTROL_REG_BASE + 0x08)
*/
/* I2C A control and state register */
#define I2C_A_CTL_STATUS	(I2C_A_CONTROL_REG_BASE + 0x00)
#define I2C_A_ADDR		(I2C_A_CONTROL_REG_BASE + 0x04)
/* #define I2C_A_DATA_BUF	(I2C_A_CONTROL_REG_BASE + 0x08) */
#define I2C_A_SW_CTL		(I2C_A_CONTROL_REG_BASE + 0x08)
#define I2C_A_TIME_OUT_CNT	(I2C_A_CONTROL_REG_BASE + 0x0C)
#define I2C_A_FIFO_STATUS	(I2C_A_CONTROL_REG_BASE + 0x10)
#define I2C_A_FS_EN		(I2C_A_CONTROL_REG_BASE + 0x14)
#define I2C_A_FIFO_DATA		(I2C_A_CONTROL_REG_BASE + 0x20)

/* I2C B control and state register */
#define I2C_B_CTL_STATUS	(I2C_B_CONTROL_REG_BASE + 0x00)
#define I2C_B_ADDR		(I2C_B_CONTROL_REG_BASE + 0x04)
/* #define I2C_B_DATA_BUF	(I2C_B_CONTROL_REG_BASE + 0x08) */
#define I2C_B_SW_CTL		(I2C_B_CONTROL_REG_BASE + 0x08)
#define I2C_B_TIME_OUT_CNT	(I2C_B_CONTROL_REG_BASE + 0x0C)
#define I2C_B_FIFO_STATUS	(I2C_B_CONTROL_REG_BASE + 0x10)
#define I2C_B_FS_EN		(I2C_B_CONTROL_REG_BASE + 0x14)
#define I2C_B_FIFO_DATA		(I2C_B_CONTROL_REG_BASE + 0x20)

/* GPS control register */
#define GPS_CTRL		(GPS_CONTROL_REG_BASE + 0x00)

/* Analog TV control register, Port A */
#define I2S_CTRL_A	(ATV_PORTA_CONTROL_REG_BASE + 0x00)
#define rbI2SRXEN	0x01 //[0], I2S receiver enable bit(1-enable, 0-disable)
#define rbRSWAP 	0x02 //[1], I2S channel swap bit, 0-left first, 1-right first.
#define rbI2SEN  	0x04 //[2], I2S enable bit(1-enable, 0-disable)
#define rbI2S16BIT	0x00 //[4:3], 00: 16-bit receiver enable
#define rbI2S24BIT	0x08 //[4:3], 01: 24-bit receiver enable
#define rbI2S32BIT	0x10 //[4:3], 10: 32-bit receiver enable

#define VIDEO_CTRL_STATUS_A	(ATV_PORTA_CONTROL_REG_BASE + 0x04)
#define rbVREN		0x01 //[0], Video receiver enable bit(1-enable, 0-disable)
#define rbADEN   	0x02 //[1], Ancillary data receive enable bit (1-enable, 0-disable)
#define rbECEN  	0x04 //[2], Video error counter enable bit(1-enable, 0-disable)
#define rbBLEN  	0x08 //[3], Video blank line receive enable bit(1-enable, 0-disable)
#define rbTVMD  	0x10 //[4], Video mode bit(1-PAL, 0-NTSC)
#define SCHVMSK  	0xC0 //[7:6], Set check value field
#define SCHV_NONE	0x00 //No set
#define SCHV_VLL	0x40 //Set video line length check value
#define SCHV_OFLN	0x80 //Set video odd field line number check value
#define SCHV_EFLN	0x80 //Set video even field line number check value

#define VIDEO_DBG_REG_1_A		(ATV_PORTA_CONTROL_REG_BASE + 0x08)
#define VIDEO_DBG_REG_2_A		(ATV_PORTA_CONTROL_REG_BASE + 0x0C)
#define PCK_HEADER_STATUS_BYTE_A	(ATV_PORTA_CONTROL_REG_BASE + 0x10)
#define ATV_WORD_ACS_A			(ATV_PORTA_CONTROL_REG_BASE + 0x14)

/* Digital TV control register, Port A */
#define MPEG2_CTRL_A	(DTV_PORTA_CONTROL_REG_BASE + 0x00)
#define rbTSEN		0x80	// bit[7] TS enable
#define rbTSBSEN	0x40	// bit[6] TS bit serial enable
#define rbTSFTEN	0x20	// bit[5] TS Filter enable
#define rbTSHDEN	0x08	// bit[3] TS header enable
#define rbTSNPKT	0x07	// bit[2:0] Number of Packet(188B TS+ nB APT)  NumPacket = value + 1
#define TS4PKTS		0x03	// 4 TS packets
#define TS5PKTS		0x04	// 5 TS packets

#define SERIAL_IN_ADDR_A	(DTV_PORTA_CONTROL_REG_BASE + 0x4C)
#define VLD_CNT_ADDR_A		(DTV_PORTA_CONTROL_REG_BASE + 0x60)
#define ERR_CNT_ADDR_A		(DTV_PORTA_CONTROL_REG_BASE + 0x64)
#define BRD_CNT_ADDR_A		(DTV_PORTA_CONTROL_REG_BASE + 0x68)

/* DMA Control Register, Port A */
#define DMA_PORTA_CHAN0_ADDR_LOW	(DMA_PORTA_CONTROL_REG_BASE + 0x00)
#define DMA_PORTA_CHAN0_ADDR_HI		(DMA_PORTA_CONTROL_REG_BASE + 0x04)
#define DMA_PORTA_CHAN0_TRANS_STATE	(DMA_PORTA_CONTROL_REG_BASE + 0x08)
#define DMA_PORTA_CHAN0_CONTROL		(DMA_PORTA_CONTROL_REG_BASE + 0x0C)

#define DMA_PORTA_CHAN1_ADDR_LOW	(DMA_PORTA_CONTROL_REG_BASE + 0x10)
#define DMA_PORTA_CHAN1_ADDR_HI		(DMA_PORTA_CONTROL_REG_BASE + 0x14)
#define DMA_PORTA_CHAN1_TRANS_STATE	(DMA_PORTA_CONTROL_REG_BASE + 0x18)
#define DMA_PORTA_CHAN1_CONTROL		(DMA_PORTA_CONTROL_REG_BASE + 0x1C)

#define DMA_PORTA_MANAGEMENT		(DMA_PORTA_CONTROL_REG_BASE + 0x20)

/* Analog TV control register, Port B */
#define I2S_CTRL_B			(ATV_PORTB_CONTROL_REG_BASE + 0x00)
#define VIDEO_CTRL_STATUS_B		(ATV_PORTB_CONTROL_REG_BASE + 0x04)
#define VIDEO_DBG_REG_1_B		(ATV_PORTB_CONTROL_REG_BASE + 0x08)
#define VIDEO_DBG_REG_2_B		(ATV_PORTB_CONTROL_REG_BASE + 0x0C)
#define PCK_HEADER_STATUS_BYTE_B	(ATV_PORTB_CONTROL_REG_BASE + 0x10)
#define ATV_WORD_ACS_B			(ATV_PORTB_CONTROL_REG_BASE + 0x14)

/* Digital TV control register, Port B */
#define MPEG2_CTRL_B		(DTV_PORTB_CONTROL_REG_BASE + 0x00)
#define SERIAL_IN_ADDR_B	(DTV_PORTB_CONTROL_REG_BASE + 0x4C)
#define VLD_CNT_ADDR_B		(DTV_PORTB_CONTROL_REG_BASE + 0x60)
#define ERR_CNT_ADDR_B		(DTV_PORTB_CONTROL_REG_BASE + 0x64)
#define BRD_CNT_ADDR_B		(DTV_PORTB_CONTROL_REG_BASE + 0x68)

/* DMA Control Register, Port B */
#define DMA_PORTB_CHAN0_ADDR_LOW	(DMA_PORTB_CONTROL_REG_BASE + 0x00)
#define DMA_PORTB_CHAN0_ADDR_HI		(DMA_PORTB_CONTROL_REG_BASE + 0x04)
#define DMA_PORTB_CHAN0_TRANS_STATE	(DMA_PORTB_CONTROL_REG_BASE + 0x08)
#define DMA_PORTB_CHAN0_CONTROL		(DMA_PORTB_CONTROL_REG_BASE + 0x0C)

#define DMA_PORTB_CHAN1_ADDR_LOW	(DMA_PORTB_CONTROL_REG_BASE + 0x10)
#define DMA_PORTB_CHAN1_ADDR_HI		(DMA_PORTB_CONTROL_REG_BASE + 0x14)
#define DMA_PORTB_CHAN1_TRANS_STATE	(DMA_PORTB_CONTROL_REG_BASE + 0x18)
#define DMA_PORTB_CHAN1_CONTROL		(DMA_PORTB_CONTROL_REG_BASE + 0x1C)

#define DMA_PORTB_MANAGEMENT	(DMA_PORTB_CONTROL_REG_BASE + 0x20)

/* MPEG2 control register */
#define rbTSEN		0x80	// bit[7] TS enable
#define rbTSBSEN	0x40	// bit[6] TS bit serial enable
#define rbTSFTEN	0x20	// bit[5] TS Filter enable
#define rbTSHDEN	0x08	// bit[3] TS header enable
#define rbTSNPKT	0x07	// bit[2:0] Number of Packet(188B TS+ nB APT)  NumPacket = value + 1
#define TS4PKTS		0x03	// 4 TS packets
#define TS5PKTS		0x04	// 5 TS packets

/* GPS control register */
#define GPS_CTRL	(GPS_CONTROL_REG_BASE + 0x00)

#endif



