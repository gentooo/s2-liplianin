/* DVB USB compliant linux driver for LME2510 controller based DVB USB boxes.
 *
 * This for TDA8262 + TDA10086 as example
 *
 * I2C addresses:
 * 0x0e - TDA10086   - Demodulator
 * 0x50 - L24C02     - eeprom
 * 0x60 - TDA8262    - Tuner
 *
 * Copyright (C) 2008,2009 Igor M. Liplianin <liplianin@me.by>
 *
 *	This program is free software; you can redistribute it and/or modify it
 *	under the terms of the GNU General Public License as published by the Free
 *	Software Foundation, version 2.
 *
 * see Documentation/dvb/README.dvb-usb for more information
 */

/* QQBox has inside lme2510, the driver not ready, but experimental ready :)
 I don't know which demod & tuner in it yet.
 The firmware for lme2510 consist of two parts, let's say A and B. */

#define DVB_USB_LOG_PREFIX "lme2510"
#include "dvb-usb.h"

#include "tda826x.h"
#include "tda10086.h"
#include "lnbp21.h"

#define CMD_POWER 0x15

/* debug */
static int dvb_usb_lme2510_debug;
#define deb_info(args...) dprintk(dvb_usb_lme2510_debug, 0x01, args)
#define deb_xfer(args...) dprintk(dvb_usb_lme2510_debug, 0x02, args)
module_param_named(debug, dvb_usb_lme2510_debug, int, 0644);
MODULE_PARM_DESC(debug, "set debugging level (1=info (or-able))."
					DVB_USB_DEBUG_STATUS);

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

int lme2510_generic_rw(struct dvb_usb_device *d, u8 *wbuf,
				u16 wlen, u8 *rbuf, u16 rlen, int delay_ms)
{
	int actlen,ret = -ENOMEM;

	if (d->props.generic_bulk_ctrl_endpoint == 0) {
		err("endpoint for generic control not specified.");
		return -EINVAL;
	}

	if (wbuf == NULL || wlen == 0)
		return -EINVAL;

	if ((ret = mutex_lock_interruptible(&d->usb_mutex)))
		return ret;

	//deb_xfer(">>> ");
	//debug_dump(wbuf,wlen,deb_xfer);

	usb_clear_halt(d->udev, usb_sndbulkpipe(d->udev, 0x01));
	ret = usb_bulk_msg(d->udev,usb_sndbulkpipe(d->udev, 1),
				wbuf, wlen, &actlen, 2000);

	if (ret)
		err("bulk message failed: %d (%d/%d)",ret,wlen,actlen);
	else
		ret = actlen != wlen ? -1 : 0;

	/* an answer is expected, and no error before */
	if (!ret && rbuf && rlen) {
		usb_clear_halt(d->udev, usb_rcvbulkpipe(d->udev, 0x81));
		if (delay_ms)
			msleep(delay_ms);

		ret = usb_bulk_msg(d->udev,usb_rcvbulkpipe(d->udev, 0x81),
					rbuf, rlen, &actlen, 2000);

		if (ret)
			err("recv bulk message failed: %d",ret);
		else {
	//		deb_xfer("<<< ");
	//		debug_dump(rbuf,actlen,deb_xfer);
		}
	}

	mutex_unlock(&d->usb_mutex);
	return ret;
}

int lme2510_msg(struct dvb_usb_device *d, u8 cmd,
		u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	//struct lme2510_state *st = d->priv;
	u8 s[wlen+1], r[rlen];
	int ret = 0;

	memset(s, 0, wlen+1);
	memset(r, 0, rlen);

	s[0] = cmd;
	memcpy(&s[1], wbuf, wlen);

	//ret = dvb_usb_generic_rw(d, s, wlen+1, r, rlen, 0);
	ret = dvb_usb_generic_rw(d, s, wlen+1, r, rlen, 0);

	if (ret != 0 ) {
		warn("there might have been an error during "
			"control message transfer. (rlen = %d)",rlen);
		return -EIO;
	}

	if (rlen > 0)
		memcpy(rbuf, &r[0], rlen);

	return 0;
}

static int lme2510_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msg[],
								int num)
{
	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	static u8 obuf[0x40], ibuf[0x40];
	int actlen, ret;

	if (!d)
		return -ENODEV;
	if (mutex_lock_interruptible(&d->i2c_mutex) < 0)
		return -EAGAIN;

	switch (num) {
	case 1:
		/* always i2c write*/
		obuf[0] = 0x04;
		obuf[1] = msg[0].len + 1;
		obuf[2] = (msg[0].addr << 1);

		memcpy(&obuf[3],msg[0].buf,msg[0].len);

		printk("try write %x bytes \n", msg[0].len - 1);
		if (lme2510_generic_rw(d, obuf, 0x40, ibuf, 1, 0) < 0) {
			err("i2c transfer failed.");
			break;
		}

		break;
	case 2:
		/* always i2c read */
		obuf[0] = 0x84;
		obuf[1] = msg[0].len+2;
		obuf[2] = (msg[0].addr << 1);
		memcpy(&obuf[3], msg[0].buf, msg[0].len);
		obuf[3 + msg[0].len] = msg[1].len;

		printk("try read %x bytes \n", msg[1].len + 1);

		usb_clear_halt(d->udev, usb_sndbulkpipe(d->udev, 0x01));
		ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, 0x01),
					obuf, 5, &actlen, 2000);
		usb_clear_halt(d->udev, usb_rcvbulkpipe(d->udev, 0x81));
		ret |= usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, 0x81),
					ibuf, msg[1].len + 1, &actlen, 2000);
		if (ret) {
//		if (lme2510_generic_rw(d, obuf, 5, ibuf, msg[1].len + 1, 0) < 0) {
			err("i2c transfer failed.");
			break;
		}
		memcpy(msg[1].buf,&ibuf[1], msg[1].len);
		break;
	default:
		warn("more than 2 i2c messages at a time is not handled yet.");
		break;
	}
	mutex_unlock(&d->i2c_mutex);
	return num;
}

static u32 lme2510_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static struct i2c_algorithm lme2510_i2c_algo = {
	.master_xfer   = lme2510_i2c_xfer,
	.functionality = lme2510_i2c_func,
};

static int lme2510_read_mac_address(struct dvb_usb_device *d, u8 mac[6])
{
	int i;
	u8 obuf[] = { 0x84, 0x03, 0xa0, 0x00, 0x01 };
	u8 ibuf[] = { 0, 0 };
	u8 eeprom[16]; 

	for (i = 0; i < 256; i++) {
		obuf[3] = i;
		if (lme2510_generic_rw(d, obuf, 5, ibuf, 2, 0) < 0) {
			err("i2c transfer while read eeprom failed.");
			break;
		} else {
			eeprom[i%16] = ibuf[1];
		}
		if (((i % 16)==15) && dvb_usb_lme2510_debug) {
			printk("%02x: ", i-15);
			debug_dump( eeprom, 16, printk);
		}
	}

	return 0;
}

static int lme2510_power_ctrl(struct dvb_usb_device *d, int onoff)
{
	return -ENODEV;
#if 0
	u8 b = onoff;
	lme2510_msg(d, CMD_POWER, &b, 0, NULL, 0);
	return lme2510_msg(d, CMD_POWER, &b, 1, NULL, 0);
#endif
}

#if 0
static int lme2510_streaming_ctrl(struct dvb_usb_device *d, int onoff)
{
	return 0;
}
#endif

static struct tda10086_config tda10086_config = {
	.demod_address = 0x0e,
	.invert = 0,
	.diseqc_tone = 1,
	.xtal_freq = TDA10086_XTAL_16M,
};

static int lme2510_frontend_attach(struct dvb_usb_adapter *adap)
{
//	struct dvb_usb_device *d = i2c_get_adapdata(adap);
	u8 obuf[0x40] = { 0x06, 0x00, 0xcf, 0xba, 0xd0 };
	u8 ibuf[] = { 0 };
//	u8 obuf[0x40], ibuf[0x40];
//	int actlen, ret;

	if (lme2510_generic_rw(adap->dev, obuf, 5, ibuf, 1, 0) < 0) {
		err("command transfer failed.");
	}

	obuf[0] = 0x04;
	obuf[1] = 0x03;
	obuf[2] = 0x1c;
	obuf[3] = 0x00;
	obuf[4] = 0x11;

	if (lme2510_generic_rw(adap->dev, obuf, 0x40, ibuf, 1, 0) < 0)
			err("frontend transfer failed.");

	obuf[0] = 0x04;
	obuf[1] = 0x0c;
	obuf[2] = 0xc0;
	obuf[3] = 0x00;
	obuf[4] = 0x01;
	obuf[5] = 0x26;
	obuf[6] = 0x08;
	obuf[7] = 0x9a;
	obuf[8] = 0xf8;
	obuf[9] = 0x0e;
	obuf[10] = 0x83;
	obuf[11] = 0x80;
	obuf[12] = 0x1a;
	obuf[13] = 0xd4;
	obuf[14] = 0xff;

	if (lme2510_generic_rw(adap->dev, obuf, 0x40, ibuf, 1, 0) < 0)
			err("tuner transfer failed.");

	obuf[0] = 0x04;
	obuf[1] = 0x03;
	obuf[2] = 0x1c;
	obuf[3] = 0x00;
	obuf[4] = 0x01;

	if (lme2510_generic_rw(adap->dev, obuf, 0x40, ibuf, 1, 0) < 0)
			err("frontend transfer failed.");

	if ((adap->fe = dvb_attach(tda10086_attach, &tda10086_config,
				&adap->dev->i2c_adap)) == NULL) {
		deb_info("TDA10086 attach failed\n");
		return -ENODEV;
	}

	return 0;
}

static int lme2510_tuner_attach(struct dvb_usb_adapter *adap)
{
	if (dvb_attach(tda826x_attach, adap->fe, 0x60,
				&adap->dev->i2c_adap, 0) == NULL) {
		deb_info("TDA8263 attach failed\n");
		return -ENODEV;
	}

	if (dvb_attach(lnbp21_attach, adap->fe,
				&adap->dev->i2c_adap, 0, 0) == NULL) {
		deb_info("LNBP21 attach failed\n");
		return -ENODEV;
	}
	return 0;
}

static struct dvb_usb_device_properties lme2510_properties;

static int lme2510_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));

	if (usb_set_interface(udev,0,1) < 0)
		err("set interface to alts=1 failed");

	if (0 == dvb_usb_device_init(intf, &lme2510_properties,
				     THIS_MODULE, NULL, adapter_nr))
		return 0;
	return -ENODEV;
}

static struct usb_device_id lme2510_table [] = {
	/* former TeVii s630, now it is different product */
//	{ USB_DEVICE(0x3344, 0xd630 ) },
	{ USB_DEVICE(0x3344, 0x1120) },/* QQBox */
	{}
};
MODULE_DEVICE_TABLE (usb, lme2510_table);

/* simple sum */
u8 zum(u8 *buf, u8 len)
{
	u8 ret = 0;
	while (len--)
		ret += *buf++;

	return ret;
};

static int lme2510_load_firmware_by_num(struct usb_device *dev,
					const struct firmware *fw,
					u8 num)
{
	u8 *b, *p;
	u8 msg[] = { 0x06, 0x00, 0x54, 0x80, 0x08 };
	int pl = 50, ret = 0, i, actlen;

	info("start downloading lme2510 firmware");

	p = kmalloc(53, GFP_KERNEL);
	b = kmalloc(1, GFP_KERNEL);

	if ((p != NULL) && (b != NULL)) {
		for (i = 0; i < fw->size; i += 50) {
			if ((i + 50) < fw->size){
				pl = 50;
				p[0] = num;
			} else {
				pl = fw->size - i;
				p[0] = 0x80 | num;
			}

			p[1] = pl - 1;
			memcpy(p + 2, fw->data + i, pl);
			p[pl + 2] = zum(p + 2, pl);

			deb_xfer(">>> ");
			debug_dump(p, pl + 3, deb_xfer);

			ret = usb_bulk_msg(dev, usb_sndbulkpipe(dev, 0x01),
					p, pl + 3, &actlen, 2000);
			ret |= usb_bulk_msg(dev, usb_rcvbulkpipe(dev, 0x81),
					b, 1, &actlen, 2000);
			if ((ret != 0) || (b[0]!= 0x88)) {
				err("error while transferring firmware %d: %d, "
					"answer: 0x%02x\n", num, ret, b[0]);
				ret = -EINVAL;
				break;
			}
		}
	}

	memcpy(p, &msg[0], 5);
	usb_clear_halt(dev, usb_sndbulkpipe(dev, 0x01));
	ret = usb_bulk_msg(dev, usb_sndbulkpipe(dev, 0x01),
			p, 5, &actlen, 2000);
	usb_clear_halt(dev, usb_rcvbulkpipe(dev, 0x81));
	ret |= usb_bulk_msg(dev, usb_rcvbulkpipe(dev, 0x81),
			b, 1, &actlen, 2000);
	if ((ret != 0) || (b[0]!= 0x77)) {
		err("error after transferring firmware %d: %d, "
			"answer: 0x%02x\n",
				num, ret, b[0]);
		ret = -EINVAL;
	}

	kfree(p);
	kfree(b);

	return ret;
};

static int lme2510_load_firmware(struct usb_device *dev,
				const struct firmware *frmwr)
{
	u8 *p, *b;
	int ret, actlen;
	const struct firmware *fw;
	const char *filename = "dvb-usb-qqboxb.fw";

	p = kmalloc(1, GFP_KERNEL);
	b = kmalloc(1, GFP_KERNEL);

	ret = request_firmware(&fw, filename, &dev->dev);
	if (ret != 0) {
		err("did not find the firmware file. (%s) "
		"Please see linux/Documentation/dvb/ for more details "
		"on firmware-problems.", filename);
		goto exit;
	}

	ret = lme2510_load_firmware_by_num(dev, frmwr, 1);
	if (ret != 0)
		goto exit;

	ret = lme2510_load_firmware_by_num(dev, fw, 2);
	if (ret != 0)
		goto exit;

	p[0] = 0x8a;
	ret = usb_bulk_msg(dev, usb_sndbulkpipe(dev, 0x01), p, 1, &actlen, 2000);
	ret |= usb_bulk_msg(dev, usb_rcvbulkpipe(dev, 0x81), b, 1, &actlen, 2000);
	if ((ret != 0) || (b[0]!= 0x88))
		ret = -EINVAL;

exit:
	kfree(p);
	kfree(b);
	mdelay(2000);

	return ret;	
};

static int lme2510_identify_state(struct usb_device *dev,
				 struct dvb_usb_device_properties *props,
				 struct dvb_usb_device_description **desc,
				 int *cold)
{
	struct usb_host_interface *alt;
	struct usb_host_endpoint  *e;

	printk("%s\n",__func__);

	alt = usb_altnum_to_altsetting(usb_ifnum_to_if(dev, 0), 1);
	e = alt->endpoint + 4;

	if (e->desc.bmAttributes == USB_ENDPOINT_XFER_BULK)
		*cold = 0;
	else
		*cold = 1;

	mdelay(1000);

 return 0;
}

static struct dvb_usb_device_properties lme2510_properties = {
	.caps = DVB_USB_IS_AN_I2C_ADAPTER,

	.usb_ctrl = DEVICE_SPECIFIC,
	.firmware = "dvb-usb-qqboxa.fw",

	.no_reconnect = 0,
	.num_adapters = 1,
	.download_firmware = lme2510_load_firmware,
	.read_mac_address = lme2510_read_mac_address,
	.adapter = {
		{
			.streaming_ctrl   = NULL,

			.frontend_attach  = lme2510_frontend_attach,
			.tuner_attach     = lme2510_tuner_attach,

			.stream = {
				.type = USB_BULK,
				.count = 8,
				.endpoint = 0x88,
				.u = {
					.bulk = {
						.buffersize = 4096,
					}
				}
			}
		}
	},

	.power_ctrl	= lme2510_power_ctrl,
	.identify_state	= lme2510_identify_state,
	.i2c_algo	= &lme2510_i2c_algo,

	.generic_bulk_ctrl_endpoint = 0x01,

	.num_device_descs = 1,
	.devices = {
		{ "QQBox DVB-S USB2.0",
			{ &lme2510_table[0], NULL },
			{ NULL },
		},
	}
};

static struct usb_driver lme2510_driver = {
	.name		= "dvb_usb_lme2510",
	.probe		= lme2510_probe,
	.disconnect = dvb_usb_device_exit,
	.id_table	= lme2510_table,
};

static int __init lme2510_module_init(void)
{
	int result;
	if ((result = usb_register(&lme2510_driver))) {
		err("usb_register failed. Error number %d",result);
		return result;
	}

	return 0;
}

static void __exit lme2510_module_exit(void)
{
	usb_deregister(&lme2510_driver);
}

module_init (lme2510_module_init);
module_exit (lme2510_module_exit);

MODULE_AUTHOR("Igor M. Liplianin <liplianin@me.by>");
MODULE_DESCRIPTION("Driver for LME2510 USB2.0 controller based cards,"
							" like QQBox ");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
