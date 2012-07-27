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

#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/input.h>
#include <media/ir-common.h>

#include "demux.h"
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"
#include "dvb_net.h"
#include "dvbdev.h"


#include "smi2032.h"
#include "ds3000.h"

/* PCI ID's */
#ifndef PCI_VENDOR_ID_SOMAGIC
#define PCI_VENDOR_ID_SOMAGIC	0x1ade
#endif
#ifndef PCI_DEVICE_ID_SMI2032
#define PCI_DEVICE_ID_SMI2032	0x2032
#endif
#ifndef PCI_DEVICE_ID_DW2006
#define PCI_DEVICE_ID_DW2006	0x2006
#endif

/* EEPROM addr */
#define IIC_24C01_addr				0xa0

static unsigned int card[]  = {[0 ... 3] = UNSET };
module_param_array(card,  int, NULL, 0444);
MODULE_PARM_DESC(card, "card type");

#if 0
static int ir_debug;
module_param(ir_debug, int, 0644);
MODULE_PARM_DESC(ir_debug, "enable debugging information for IR decoding");
#endif

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static inline struct smi2032_dev *feed_to_smi2032dvb(struct dvb_demux_feed *feed)
{
	return container_of(feed->demux, struct smi2032_dev, demux);
}

static inline struct smi2032_dev *frontend_to_smi2032(struct dvb_frontend *fe)
{
	return container_of(fe->dvb, struct smi2032_dev, dvb_adapter);
}

static int smi2032_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct smi2032_dev *dev = frontend_to_smi2032(fe);

	printk("%s:\n", __func__);

	switch (dev->boardnr) {
	case SMI2032_BOARD_DVBWORLD_2006:
		/* need already configured port 0 as 8-bit gpio */
		smi_clear(GPIO_16to23_CTRL, 0x00008080);
		if (voltage == SEC_VOLTAGE_18)
			smi_set(GPIO_16to23_CTRL, 0x00000080);

		break;
	default:
		printk("%s: pehaps not satellite card?\n", __func__);
	}

	return 0;
}

static void smi2032_enable_irqs(struct smi2032_dev *dev, u32 irq);

static void smi2032_set_dma_addr(struct smi2032_dev *dev)
{
	u32 dmaMemPtrHi = 0;
	// pcie dma port B, channel 0 in used
//	u32 totalLength = DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0;
	u32 dmaMemPtrLow = cpu_to_le32(dev->dma_addr);
	u32 dmaCtlReg = (DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0) |
			(DMA_TRANS_UNIT_188 << 22) |
			(1 << 28);
	// enable the corresponding interrupt first
	smi2032_enable_irqs(dev, 0xffff);//1 << 2);

	// write DMA register, start DMA engine
	smi_write(DMA_PORTB_CHAN0_ADDR_LOW, dmaMemPtrLow);
	smi_write(DMA_PORTB_CHAN0_ADDR_HI, 0);
	smi_write(DMA_PORTB_CHAN0_CONTROL, dmaCtlReg);
	smi_set(DMA_PORTB_MANAGEMENT, 0x3);

	printk("smi_cfg_dma_B(): Config PCIE port B, DMA channel0\n");
	printk("smi_cfg_dma_B(): DMA memory low  address = 0x%08X\n", dmaMemPtrLow);
	printk("smi_cfg_dma_B(): DMA memory high address = 0x%08X\n", dmaMemPtrHi);
	printk("smi_cfg_dma_B(): DMA memory control register = 0x%08X\n", dmaCtlReg);
	printk("smi_cfg_dma_B(): DMA memory management = 0x%08X\n", 3);
}

static int __devinit smi2032_dma_map(struct smi2032_dev *dev)
{
	dev->ts_buf = pci_alloc_consistent(dev->pci, DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0, &dev->dma_addr);

	return !dev->ts_buf;
}

static void smi2032_dma_unmap(struct smi2032_dev *dev)
{
	pci_free_consistent(dev->pci, DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0, dev->ts_buf, dev->dma_addr);
}

static void smi2032_enable_irqs(struct smi2032_dev *dev, u32 irq)
{
	smi_write(MSI_INT_ENA_SET, irq);
}

static void smi2032_disable_irqs(struct smi2032_dev *dev, u32 irq)
{
	smi_write(MSI_INT_ENA_CLR, irq);
}

static int smi2032_start_feed(struct dvb_demux_feed *f)
{
	struct smi2032_dev *dev = feed_to_smi2032dvb(f);

	if (dev->full_ts_users++ == 0)
		smi2032_enable_irqs(dev, 0xffff);

	return 0;
}

static int smi2032_stop_feed(struct dvb_demux_feed *f)
{
	struct smi2032_dev *dev = feed_to_smi2032dvb(f);

	if (--dev->full_ts_users == 0)
		smi2032_disable_irqs(dev, 0xffff);

	return 0;
}

void smi_portB_dpc(struct smi2032_dev *dev)
{
	u8 dmaChan0State = 0;
	u8 dmaChan1State = 0;
	u32 dmaTotalLength = 0;
	u32 readData = 0;
	u8 dmaIntState = 0;  //Add for interrupt flag
	u8 i = 0;
	u32 dmaManagement = 0;

	printk("smi_portB_dpc(): Started !\n");

	do {
		printk("%s: <<<@ Record interrupt status = 0x%08x !\n",
		                      __func__, dev->int_status);
		if (DMA_B_CHAN0_DONE_INT == (dev->int_status & DMA_B_CHAN0_DONE_INT)) {
			dmaIntState = 0x00;  // Port B, channel 0
		} else if (DMA_B_CHAN1_DONE_INT == (dev->int_status & DMA_B_CHAN1_DONE_INT)) {
			dmaIntState = 0x01;  // port B, channel 1
		}

		printk("smi_portB_dpc(): <<<@ dmaIntState = 0x%x !\n", dmaIntState);
		//--End

		dmaManagement = smi_read(DMA_PORTB_MANAGEMENT);
		printk("smi_portB_dpc(): <<<@ Read DMA_PORTA_MANAGEMENT = 0x%08x !\n", dmaManagement);

		dmaChan0State = (unsigned char)((dmaManagement & 0x00000030) >> 4);
		dmaChan1State = (unsigned char)((dmaManagement & 0x00300000) >> 20);  //add

		printk("smi_portB_dpc(): <<<@ dmaChan0State = 0x%x !\n", dmaChan0State);
		printk("smi_portB_dpc(): <<<@ dmaChan1State = 0x%x !\n", dmaChan1State);

		if ((dmaIntState == 0x00)    // DMA port B channel 0 done
		    && (dmaChan0State == 0x01)  // DMA transaction complete successful
		    && (/*pFdoData->dmaPortBChanUsed&*/0x01)) { // DMA port B, channel 0 is in use
			printk("smi_portB_dpc(): DMA CH0 engine complete successful !\n");

			//
			// value of DMA_PORT0_CHAN0_TRANS_STATE register [21:0] indicate dma total transfer length
			// and zero of [21:0] indicate dma total transfer length equal to 0x400000 (4MB)
			//
			readData = smi_read(DMA_PORTB_CHAN0_TRANS_STATE);
			readData &= 0x003FFFFF;
			dmaTotalLength = (readData == 0) ? 0x00400000 : readData;

			if (dmaTotalLength != DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0) {
				printk( "smi_portB_dpc(): length mismatched, DMA CH0 engine complete %0d bytes, mapped buffer is %0d bytes",
				                        dmaTotalLength, DMA_MAPPED_MEMORY_SIZE_PORTB_CHAN0);
				break;
			} else {
				for (i = 0 ; i < (dmaTotalLength / 1024) ; i++)
					printk( "smi_portB_dpc(): copy data to storage memeory failed !\n");

#if 0  //def PORT_B_IN_USE

				// added for passing stream data to capture filter output pin
				for (i = 0; i < (dmaTotalLength / (TRANSPORT_PACKET_SIZE*TRANSPORT_PACKET_COUNT)); i++) {
					PutTsPacketToOutputPin_2(pFdoData);
				}
#endif
			}
		}
		else {
			if (dmaIntState == 0x00) {
				if (dmaChan0State == 0x10) {
					printk( "smi_portB_dpc(): DMA CH0 engine state reservered !\n");
				} else if (dmaChan0State == 0x10) {
					printk( "smi_portB_dpc(): DMA CH0 engine transaction canceled !\n");
				} else {
					printk( "smi_portB_dpc(): DMA CH0 engine transcation timeout !\n");
				}
			}
		}
	} while (0);
}


/* work handler */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void smi2032_dmx_buffer(void *_smi2032dvb)
#else
static void smi2032_dmx_buffer(struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	struct smi2032_dev *dev = _smi2032dvb;
#else
	struct smi2032_dev *dev =
				container_of(work, struct smi2032_dev, work);
#endif
	u32 oldwrp = dev->wrp;
	u32 nextwrp = dev->nextwrp;
	printk("%s: wrp=0x%08x, nextwrp=0x%08x\n", __func__, oldwrp, nextwrp);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
static irqreturn_t smi2032_irq(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t smi2032_irq(int irq, void *dev_id)
#endif
{
	struct smi2032_dev *dev = dev_id;

	/* Read-Write MSI_INT_STATUS Ack's Interrupt */
	u32 intsts = smi_read(MSI_INT_STATUS);

	if (0 != (intsts & DMA_B_CHAN0_DONE_INT)) {
		queue_work(dev->wq, &dev->work);
		printk("interrupt! \n");
	}
	
	smi_write(MSI_INT_STATUS_CLR, intsts);

	return IRQ_HANDLED;
}

static int __devinit smi2032_hw_init(struct smi2032_dev *dev)
{
	u8 value, value1;

	smi2032_disable_irqs(dev, 0xffff);

	/* map DMA and set address */
	smi2032_dma_map(dev);
	smi2032_set_dma_addr(dev);

	/* set port 0 to 8 bit GPIO mode, port 1 to DTV */
	smi_andor(MUX_MODE_CTRL, (rbPaMSMask | rbPbMSMask),
			(rbPaMS8bitGpio | rbPbMSDtvNoGpio));

	/*rbTSEN = 1<<7, ts enable, rbTSBSEN=1<<6 ts serial enable,
	so we choose port 1 ts enable, serial disable*/
	smi_andor(MPEG2_CTRL_B, rbTSBSEN, rbTSEN);

	/* read some register for debugging */
	printk("%s: i2c_sw_ctl %0x, \n", __func__, smi_read(I2C_A_SW_CTL));
	
	pci_read_config_byte(dev->pci, 0, &value);
	pci_read_config_byte(dev->pci, 1, &value1);
	printk("%s: pci config 0x%0x\n", __func__, value + (value1 << 8));

	return 0;
}

static void smi2032_hw_exit(struct smi2032_dev *dev)
{
	smi2032_disable_irqs(dev, 0xffff);//ALL_INT);
	smi2032_dma_unmap(dev);
}


static struct ds3000_config dvbworld_ds3000_config = {
	.demod_address = 0x68,
};

static int __devinit frontend_init(struct smi2032_dev *dev)
{
	int ret;

	switch (dev->boardnr) {
	case SMI2032_BOARD_DVBWORLD_2006:
		dev->fe = dvb_attach(
			ds3000_attach, &dvbworld_ds3000_config,
			&dev->i2c_bus[0].i2c_adap);
		if (dev->fe)
			dev->fe->ops.set_voltage = smi2032_set_voltage;

		break;
	}

	if (!dev->fe) {
		dev_err(&dev->pci->dev, "could not attach frontend\n");
		return -ENODEV;
	}

	ret = dvb_register_frontend(&dev->dvb_adapter, dev->fe);
	if (ret < 0) {
		if (dev->fe->ops.release)
			dev->fe->ops.release(dev->fe);
		dev->fe = NULL;
		return ret;
	}

	return 0;
}

#if 0
static void __devinit smi2032_read_mac(struct smi2032_dev *dev, u8 *mac)
{
	static u8 command[1] = { 0x28 };

	struct i2c_msg msg[] = {
		{
			.addr = IIC_24C01_addr >> 1,
			.flags = 0,
			.buf = command,
			.len = 1
		}, {
			.addr = IIC_24C01_addr >> 1,
			.flags = I2C_M_RD,
			.buf = mac,
			.len = 6
		},
	};

	smi2032_i2c_xfer(&dev->i2c_bus[1].i2c_adap, msg , 2);
	dev_info(&dev->pci->dev, "MAC %pM\n", mac);
}
#endif

static int __devinit smi2032_probe(struct pci_dev *pci,
				  const struct pci_device_id *ent)
{
	struct smi2032_dev *dev;
	struct dvb_adapter *dvb_adapter;
	struct dvb_demux *dvbdemux;
	struct dmx_demux *dmx;
	int ret = -ENOMEM;
	int i;

	dev = kzalloc(sizeof(struct smi2032_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* board config */
	dev->nr = smi2032_devcount;
	dev->boardnr = UNSET;
	if (card[dev->nr] < smi2032_bcount)
		dev->boardnr = card[dev->nr];
	for (i = 0; UNSET == dev->boardnr && i < smi2032_idcount; i++)
		if (pci->subsystem_vendor ==
			smi2032_subids[i].subvendor &&
				pci->subsystem_device ==
					smi2032_subids[i].subdevice)
			dev->boardnr = smi2032_subids[i].card;

	if (UNSET == dev->boardnr) {
		dev->boardnr = SMI2032_BOARD_UNKNOWN;
		smi2032_card_list(dev);
	}

	smi2032_devcount++;
	dev->pci = pci;
	dev->buffer_size = DMA_MAPPED_MEMORY_SIZE_TOTAL;
	dev->PacketErrorCount = 0;
	dev->dmarst = 0;

	ret = pci_enable_device(pci);
	if (ret < 0)
		goto err_kfree;

	ret = pci_set_dma_mask(pci, DMA_BIT_MASK(32));
	if (ret < 0)
		goto err_pci_disable_device;

	pci_set_master(pci);

	ret = pci_request_regions(pci, DRIVER_NAME);
	if (ret < 0)
		goto err_pci_disable_device;

	dev->io_meml = ioremap(pci_resource_start(pci, 0),
					pci_resource_len(pci, 0));
	if (!dev->io_meml) {
		ret = -EIO;
		goto err_pci_release_regions;
	}
	printk("%s: pci_resource_start = %0x\n", __func__, pci_resource_start(pci, 0));
	printk("%s: pci_resource_len = %0x\n", __func__, pci_resource_len(pci, 0));

	spin_lock_init(&dev->lock);
	pci_set_drvdata(pci, dev);

	ret = smi2032_hw_init(dev);
	if (ret < 0)
		goto err_pci_iounmap;

	/* i2c */
	dev->i2c_bus[0].nr = 0;
	dev->i2c_bus[0].dev = dev;
	dev->i2c_bus[0].reg_ctl_status = I2C_A_CTL_STATUS;
	dev->i2c_bus[0].reg_addr = I2C_A_ADDR;
	dev->i2c_bus[0].reg_sw_ctl = I2C_A_SW_CTL;
	dev->i2c_bus[0].reg_time_out_cnt = I2C_A_TIME_OUT_CNT;
	dev->i2c_bus[0].reg_fifo_status = I2C_A_FIFO_STATUS;
	dev->i2c_bus[0].reg_fs_en = I2C_A_FS_EN;
	dev->i2c_bus[0].reg_fifo_data = I2C_A_FIFO_DATA;

	dev->i2c_bus[1].nr = 1;
	dev->i2c_bus[1].dev = dev;
	dev->i2c_bus[1].reg_ctl_status = I2C_B_CTL_STATUS;
	dev->i2c_bus[1].reg_addr = I2C_B_ADDR;
	dev->i2c_bus[1].reg_sw_ctl = I2C_B_SW_CTL;
	dev->i2c_bus[1].reg_time_out_cnt = I2C_B_TIME_OUT_CNT;
	dev->i2c_bus[1].reg_fifo_status = I2C_B_FIFO_STATUS;
	dev->i2c_bus[1].reg_fs_en = I2C_B_FS_EN;
	dev->i2c_bus[1].reg_fifo_data = I2C_B_FIFO_DATA;

	ret = smi2032_i2c_register(&dev->i2c_bus[0]);
	ret |= smi2032_i2c_register(&dev->i2c_bus[1]);

	if (ret != 0)
		goto err_smi2032_hw_exit;

	/* dvb */
	ret = dvb_register_adapter(&dev->dvb_adapter, DRIVER_NAME,
					THIS_MODULE, &pci->dev, adapter_nr);
	if (ret < 0)
		goto err_i2c_del_adapter;

	dvb_adapter = &dev->dvb_adapter;

//	smi2032_read_mac(dev, dvb_adapter->proposed_mac);

	dvbdemux = &dev->demux;
	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = smi2032_start_feed;
	dvbdemux->stop_feed = smi2032_stop_feed;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
			DMX_SECTION_FILTERING | DMX_MEMORY_BASED_FILTERING);
	ret = dvb_dmx_init(dvbdemux);
	if (ret < 0)
		goto err_dvb_unregister_adapter;

	dmx = &dvbdemux->dmx;
	dev->dmxdev.filternum = 256;
	dev->dmxdev.demux = dmx;
	dev->dmxdev.capabilities = 0;

	ret = dvb_dmxdev_init(&dev->dmxdev, dvb_adapter);
	if (ret < 0)
		goto err_dvb_dmx_release;

	dev->hw_frontend.source = DMX_FRONTEND_0;

	ret = dmx->add_frontend(dmx, &dev->hw_frontend);
	if (ret < 0)
		goto err_dvb_dmxdev_release;

	dev->mem_frontend.source = DMX_MEMORY_FE;

	ret = dmx->add_frontend(dmx, &dev->mem_frontend);
	if (ret < 0)
		goto err_remove_hw_frontend;

	ret = dmx->connect_frontend(dmx, &dev->hw_frontend);
	if (ret < 0)
		goto err_remove_mem_frontend;

	i2c_soft_reset(&dev->i2c_bus[0].i2c_adap);
	i2c_soft_reset(&dev->i2c_bus[1].i2c_adap);

	ret = frontend_init(dev);
	if (ret < 0)
		goto err_disconnect_frontend;

	dvb_net_init(dvb_adapter, &dev->dvbnet, dmx);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	INIT_WORK(&dev->work, smi2032_dmx_buffer, dev);
#else
	INIT_WORK(&dev->work, smi2032_dmx_buffer);
#endif
	sprintf(dev->wqn, "%s/%d", dvb_adapter->name, dvb_adapter->num);
	dev->wq = create_singlethread_workqueue(dev->wqn);
	if (!dev->wq)
		goto err_dvb_net;

	ret = request_irq(pci->irq, smi2032_irq, IRQF_SHARED,
						DRIVER_NAME, dev);
	if (ret < 0)
		goto err_workqueue;

	return 0;

err_workqueue:
	destroy_workqueue(dev->wq);
err_dvb_net:
	dvb_net_release(&dev->dvbnet);
err_disconnect_frontend:
	dmx->disconnect_frontend(dmx);
err_remove_mem_frontend:
	dmx->remove_frontend(dmx, &dev->mem_frontend);
err_remove_hw_frontend:
	dmx->remove_frontend(dmx, &dev->hw_frontend);
err_dvb_dmxdev_release:
	dvb_dmxdev_release(&dev->dmxdev);
err_dvb_dmx_release:
	dvb_dmx_release(dvbdemux);
err_dvb_unregister_adapter:
	dvb_unregister_adapter(dvb_adapter);
err_i2c_del_adapter:
	smi2032_i2c_unregister(&dev->i2c_bus[0]);
	smi2032_i2c_unregister(&dev->i2c_bus[1]);
err_smi2032_hw_exit:
	smi2032_hw_exit(dev);
err_pci_iounmap:
	pci_iounmap(pci, dev->io_meml);
err_pci_release_regions:
	pci_release_regions(pci);
err_pci_disable_device:
	pci_disable_device(pci);
err_kfree:
	pci_set_drvdata(pci, NULL);
	kfree(dev);
	return ret;
}

static void __devexit smi2032_remove(struct pci_dev *pci)
{
	struct smi2032_dev *dev = pci_get_drvdata(pci);
	struct dvb_adapter *dvb_adapter = &dev->dvb_adapter;
	struct dvb_demux *dvbdemux = &dev->demux;
	struct dmx_demux *dmx = &dvbdemux->dmx;

	dmx->close(dmx);
	dvb_net_release(&dev->dvbnet);
	if (dev->fe)
		dvb_unregister_frontend(dev->fe);

	dmx->disconnect_frontend(dmx);
	dmx->remove_frontend(dmx, &dev->mem_frontend);
	dmx->remove_frontend(dmx, &dev->hw_frontend);
	dvb_dmxdev_release(&dev->dmxdev);
	dvb_dmx_release(dvbdemux);
	dvb_unregister_adapter(dvb_adapter);
	if (&dev->i2c_bus[0].i2c_adap)
		i2c_del_adapter(&dev->i2c_bus[0].i2c_adap);

	smi2032_hw_exit(dev);
	synchronize_irq(pci->irq);
	free_irq(pci->irq, dev);
	pci_iounmap(pci, dev->io_meml);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
	smi2032_devcount--;
	kfree(dev);
}

static struct pci_device_id smi2032_id_table[] __devinitdata = {
	{
		.vendor = PCI_VENDOR_ID_SOMAGIC,
		.device = PCI_DEVICE_ID_SMI2032,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
	}, {
		/* empty */
	},
};

MODULE_DEVICE_TABLE(pci, smi2032_id_table);

static struct pci_driver smi2032_driver = {
	.name = DRIVER_NAME,
	.id_table = smi2032_id_table,
	.probe = smi2032_probe,
	.remove = __devexit_p(smi2032_remove),
};

static int __init smi2032_init(void)
{
	return pci_register_driver(&smi2032_driver);
}

static void __exit smi2032_exit(void)
{
	pci_unregister_driver(&smi2032_driver);
}

module_init(smi2032_init);
module_exit(smi2032_exit);

MODULE_AUTHOR("Igor M. Liplianin <liplianin@me.by>");
MODULE_DESCRIPTION("Somagic smi2032 DVB driver");
MODULE_LICENSE("GPL");
