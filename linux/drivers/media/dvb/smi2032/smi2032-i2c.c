/*
 * Driver for Somagic SMI2032 PCIe bridge
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "smi2032.h"

#include <media/v4l2-common.h>

static unsigned int i2c_debug;
module_param(i2c_debug, int, 0644);
MODULE_PARM_DESC(i2c_debug, "enable debug messages [i2c]");

#define dprintk(fmt, arg...)\
	do { if (i2c_debug)\
		printk(KERN_DEBUG fmt, arg);\
	} while (0)

#define I2C_WAIT_DELAY 128
#define I2C_WAIT_RETRY 64

void i2c_soft_reset(struct i2c_adapter *i2c_adap)
{
	struct smi2032_i2c *bus = i2c_adap->algo_data;
	struct smi2032_dev *dev = bus->dev;

	dprintk("%s: \n", __func__);

	/* set bit 5, soft reset bit */
	smi_set(bus->reg_sw_ctl, 0x00000020);
	msleep(10);
	smi_write(bus->reg_sw_ctl, 0x00003200);
}

static int i2c_check_fifo(struct i2c_adapter *i2c_adap)
{
	struct smi2032_i2c *bus = i2c_adap->algo_data;
	struct smi2032_dev *dev = bus->dev;
	u32 fifo_status;
	u8 fifo_read_size;
	u8 fifo_write_size;

	fifo_status = smi_read(bus->reg_fifo_status);
	fifo_write_size = (u8)(fifo_status & 0x1f);
	fifo_read_size  = (u8)((fifo_status >> 8) & 0x1f);
	if ((fifo_write_size != 16) || (fifo_read_size != 0)) {
		dprintk("%s: FIFO not empty, FIFO status: (0x%08x)\n", __func__, fifo_status);
		i2c_soft_reset(i2c_adap);
		return -1;
	}

	return 0;
}

static int i2c_sendbytes(struct i2c_adapter *i2c_adap,
			 	const struct i2c_msg *msg)
{
	struct smi2032_i2c *bus = i2c_adap->algo_data;
	struct smi2032_dev *dev = bus->dev;
	u32 wdata, cnt = 0;
	u8 fifo_write_size;
	int i, data_index = 0;
	u8 i2c_state;

	dprintk("%s(msg->len=%d)\n", __func__, msg->len);

	if (0 != i2c_check_fifo(i2c_adap))
		return -1;

	smi_write(bus->reg_ctl_status, 0);/* clear i2c status */

	smi_write(bus->reg_addr, (msg->addr << 9));/* slave addr */
	dprintk("%s: reg_addr = %0x\n",__func__, (msg->addr << 9));
	for (i = 0; i < 16; i += 4) {
		wdata = *(msg->buf + data_index + 3);
		wdata = *(msg->buf + data_index + 2) + (wdata << 8);
		wdata = *(msg->buf + data_index + 1) + (wdata << 8);
		wdata = *(msg->buf + data_index + 0) + (wdata << 8);

		dprintk("%s: index: %d value: (0x%08x) !\n",
				__func__, data_index, wdata);

		smi_write(bus->reg_fifo_data, wdata);
		data_index += 4;

		if (data_index >= msg->len) {
			dprintk("%s: data <= 16 byte. Write to Fifo end!\n ",
					__func__);
			break ;
		}
	}

	smi_write(bus->reg_ctl_status, (msg->len << 8) | 0x80);/* trigger i2c write */
	while (cnt < I2C_WAIT_RETRY) {
		if (data_index < msg->len) {
			/* still have data need to send, check the fifo status first */
			fifo_write_size = (u8)(smi_read(bus->reg_fifo_status) & 0x1f);
			if (0 != (fifo_write_size % 4)) {
				dprintk("%s:  ERR: i2cFifo Size not DWORD in size!\n ",__func__);
				i2c_soft_reset(i2c_adap);
				return -1;
			}
	
			for (i = 0; i < fifo_write_size; i += 4) {
				wdata = *(msg->buf + data_index + 3);
				wdata = *(msg->buf + data_index + 2) + (wdata << 8);
				wdata = *(msg->buf + data_index + 1) + (wdata << 8);
				wdata = *(msg->buf + data_index + 0) + (wdata << 8);
				dprintk("%s: index: %d value: (0x%08x) !\n", __func__,
						data_index, wdata);
				smi_write(bus->reg_fifo_data, wdata);
				data_index += 4;
	
				if (data_index >= msg->len) {
					dprintk("%s:  write to Fifo end!\n ", __func__);
					break ;
				}
			}
		}

		udelay(I2C_WAIT_DELAY);
		i2c_state  = (smi_read(bus->reg_ctl_status) >> 16);
		dprintk("%s: i2cState = 8'h%02X\n", __func__, i2c_state);
		if (0 != (i2c_state & 0x40)) {
			/* if some error occur, return false */
			if ((0 != (i2c_state & 0x20)) || (0 != (i2c_state & 0x08))) {
				printk("%s: i2c error NAK or timeout occur\n", __func__);
	
				i2c_soft_reset(i2c_adap);
	
				return -1;
			} else {
				dprintk("%s: i2c write operation completed "
						"successful\n", __func__);
				break;
			}
		}
		cnt++;
	}

	return msg->len;
}

static int i2c_readbytes(struct i2c_adapter *i2c_adap,
				const struct i2c_msg *msg)
{
	struct smi2032_i2c *bus = i2c_adap->algo_data;
	struct smi2032_dev *dev = bus->dev;
	u32 cnt = 0;
	u8 fifo_read_size;
	u8 i2c_state;
	int i, data_index = 0;
	u32 read_data;

	dprintk("%s(msg->len=%d)\n", __func__, msg->len);

	if (0 != i2c_check_fifo(i2c_adap))
		return -1;

	smi_write(bus->reg_ctl_status, 0);/* clear i2c status */
	smi_write(bus->reg_addr, (msg->addr << 9) | 1);/* slave addr */
	dprintk("%s: reg_addr = %0x\n",__func__, (msg->addr << 9));
	/* trigger i2c read */
	smi_write(bus->reg_ctl_status, (msg->len << 8) | 0xa0);

	while(cnt < I2C_WAIT_RETRY) {
		udelay(I2C_WAIT_DELAY);
		fifo_read_size = (u8)((smi_read(bus->reg_fifo_status) >> 8) & 0x1f);
		dprintk("fifo_read_size = %d\n", fifo_read_size);
		for (i = 0; i < fifo_read_size; i += 4) {
			read_data = smi_read(bus->reg_fifo_data);

			*(msg->buf + (data_index++)) = (u8)read_data;
			if (data_index >= msg->len)
				break;

			*(msg->buf + (data_index++)) = (u8)(read_data >> 8);
			if (data_index >= msg->len)
				break;

			*(msg->buf + (data_index++)) = (u8)(read_data >> 16);
			if (data_index >= msg->len)
				break;

			*(msg->buf + (data_index++)) = (u8)(read_data >> 24);
			if (data_index >= msg->len)
				break;
		}

		i2c_state  = (u8)(smi_read(bus->reg_ctl_status) >> 16);
		dprintk("%s: i2cState = 8'h%02X\n", __func__, i2c_state);
		if (0 != (i2c_state & 0x80)) {
			/* if some error occur, return false */
			if ((0 != (i2c_state & 0x20)) || (0 != (i2c_state & 0x08))) {
				printk("%s: i2c error NAK or timeout occur\n",
						__func__);
				i2c_soft_reset(i2c_adap);
				return -1;
			} else {
				dprintk("%s: i2c read operation completed"
						" successful\n", __func__);
				break;
			}
		}
		cnt++;
	}

	return msg->len;
}

int smi2032_i2c_xfer(struct i2c_adapter *i2c_adap,
		    struct i2c_msg *msgs, int num)
{
	int i, retval = 0;

	dprintk("%s(num = %d)\n", __func__, num);

	for (i = 0 ; i < num; i++) {
		dprintk("%s(num = %d) addr = 0x%02x  len = 0x%x\n",
			__func__, num, msgs[i].addr, msgs[i].len);
		if (msgs[i].flags & I2C_M_RD)
			/* read */
			retval = i2c_readbytes(i2c_adap, &msgs[i]);
		else
			/* write */
			retval = i2c_sendbytes(i2c_adap, &msgs[i]);

		if (retval < 0)
			return retval;
	}

	return num;
}

static u32 functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm smi2032_i2c_algo_template = {
	.master_xfer   = smi2032_i2c_xfer,
	.functionality = functionality,
#ifdef NEED_ALGO_CONTROL
	.algo_control = dummy_algo_control,
#endif
};

static struct i2c_adapter smi2032_i2c_adap_template = {
	.name              = DRIVER_NAME,
	.owner             = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
//	.id                = I2C_HW_B_SMI2032,
#endif
	.algo              = &smi2032_i2c_algo_template,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	.class             = I2C_CLASS_TV_DIGITAL,
#endif
};

static struct i2c_client smi2032_i2c_client_template = {
	.name	= "smi2032 internal",
};

int smi2032_i2c_register(struct smi2032_i2c *bus)
{
	struct smi2032_dev *dev = bus->dev;
	int ret =0;

	dprintk("%s(bus = %d)\n", __func__, bus->nr);

	memcpy(&bus->i2c_adap, &smi2032_i2c_adap_template,
	       sizeof(bus->i2c_adap));
	memcpy(&bus->i2c_algo, &smi2032_i2c_algo_template,
	       sizeof(bus->i2c_algo));
	memcpy(&bus->i2c_client, &smi2032_i2c_client_template,
	       sizeof(bus->i2c_client));

	bus->i2c_adap.dev.parent = &dev->pci->dev;

	strcpy(bus->i2c_adap.name, DRIVER_NAME);

	bus->i2c_algo.data = bus;
	bus->i2c_adap.algo_data = bus;
	i2c_set_adapdata(&bus->i2c_adap, dev);
	ret = i2c_add_adapter(&bus->i2c_adap);

	bus->i2c_client.adapter = &bus->i2c_adap;

	dprintk("%s: i2c bus %d registered\n", DRIVER_NAME, bus->nr);

	return ret;
}

int smi2032_i2c_unregister(struct smi2032_i2c *bus)
{
	i2c_del_adapter(&bus->i2c_adap);
	return 0;
}
