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
#ifndef SMI2032_H
#define SMI2032_H

#include <linux/pci.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/kdev_t.h>

#include <media/v4l2-device.h>
#include <media/tuner.h>
#include <media/tveeprom.h>
#include <media/videobuf-dma-sg.h>
#include <media/videobuf-dvb.h>

#include "smi2032-conf.h"
#include "smi2032-reg.h"
#include "smi2032-int.h"

#include <linux/version.h>
#include <linux/mutex.h>

#define UNSET (-1U)

#define SMI2032_MAX_BOARDS	0x08

#define DRIVER_NAME	"smi2032"

#define SMI2032_BOARD_NOAUTO		UNSET
#define SMI2032_BOARD_UNKNOWN		0
#define SMI2032_BOARD_DVBWORLD_2006	1

typedef enum {
	SMI2032_MPEG_UNDEFINED = 0,
	SMI2032_MPEG_DVB,
	SMI2032_ANALOG_VIDEO,
} port_t;

struct smi2032_board {
	char *name;
	port_t porta, portb;
};

struct smi2032_subid {
	u16 subvendor;
	u16 subdevice;
	u32 card;
};

struct smi2032_i2c {
	struct smi2032_dev *dev;
	int nr;

	/* i2c */
	struct i2c_adapter i2c_adap;
	struct i2c_algo_bit_data i2c_algo;
	struct i2c_client i2c_client;

	/* registers used for raw access */
	u32 reg_ctl_status;
	u32 reg_addr;
	u32 reg_sw_ctl;
	u32 reg_time_out_cnt;
	u32 reg_fifo_status;
	u32 reg_fs_en;
	u32 reg_fifo_data;
};

struct smi2032_dev {
	/* pci */
	struct pci_dev *pci;
	u32 __iomem *io_meml;

	/* dvb */
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
	struct dmxdev dmxdev;
	struct dvb_adapter dvb_adapter;
	struct dvb_demux demux;
	struct dvb_frontend *fe;
	struct dvb_net dvbnet;
	unsigned int full_ts_users;
	unsigned int boardnr;
	int nr;

	/* i2c */
	struct smi2032_i2c i2c_bus[2];

	/* irq */
	struct work_struct work;
	struct workqueue_struct *wq;
	char wqn[16];
	u32 int_status;

	/* dma */
	dma_addr_t dma_addr;
	unsigned char *ts_buf;
	u32 wrp;
	u32 nextwrp;
	u32 buffer_size;
	unsigned int	PacketErrorCount;
	unsigned int dmarst;
	spinlock_t lock;
};

#define smi_read(reg)		readl(dev->io_meml + ((reg) >> 2))
#define smi_write(reg, value)	writel((value), dev->io_meml + ((reg) >> 2))

#define smi_andor(reg, mask, value) \
	writel((readl(dev->io_meml + ((reg) >> 2)) & ~(mask)) |\
		((value) & (mask)), dev->io_meml + ((reg) >> 2))

#define smi_set(reg, bit)	smi_andor((reg), (bit), (bit))
#define smi_clear(reg, bit)	smi_andor((reg), (bit), 0)

extern int smi2032_i2c_register(struct smi2032_i2c *bus);
extern int smi2032_i2c_unregister(struct smi2032_i2c *bus);
/* smi2032-i2c */
extern void i2c_soft_reset(struct i2c_adapter *i2c_adap);
extern int smi2032_i2c_xfer(struct i2c_adapter *i2c_adap,
			struct i2c_msg *msgs, int num);
/* smi2032-cards.c */
extern struct smi2032_board smi2032_boards[];
extern const unsigned int smi2032_bcount;

extern struct smi2032_subid smi2032_subids[];
extern const unsigned int smi2032_idcount;

extern unsigned int smi2032_devcount;

extern void smi2032_card_list(struct smi2032_dev *dev);

#endif
