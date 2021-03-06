/*
	Mantis VP-2033 driver

	Copyright (C) 2005, 2006 Manu Abraham (abraham.manu@gmail.com)

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include "mantis_common.h"
#include "mantis_vp2033.h"

#define MANTIS_MODEL_NAME	"VP-2033"
#define MANTIS_DEV_TYPE		"DVB-C"

struct mantis_hwconfig vp2033_mantis_config = {
	.model_name	= MANTIS_MODEL_NAME,
	.dev_type	= MANTIS_DEV_TYPE,
	.ts_size	= MANTIS_TS_204,
	.ir_codes	= RC_MAP_VP2033,
};

struct tda1002x_config philips_cu1216_config = {
	.demod_address = 0x18 >> 1,
	.invert = 1,
};

u8 read_pwm(struct mantis_pci *mantis)
{
	u8 b = 0xff;
	u8 pwm;
	struct i2c_msg msg[] = {
		{.addr = 0x50,.flags = 0,.buf = &b,.len = 1},
		{.addr = 0x50,.flags = I2C_M_RD,.buf = &pwm,.len = 1}
	};

	if ((i2c_transfer(&mantis->adapter, msg, 2) != 2)
	    || (pwm == 0xff))
		pwm = 0x48;

	return pwm;
}

int philips_cu1216_tuner_set(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct mantis_pci *mantis = fe->dvb->priv;

	u8 buf[6];
	struct i2c_msg msg = {.addr = 0x60,.flags = 0,.buf = buf,.len = sizeof(buf) };
	int i;

#define CU1216_IF 36125000
#define TUNER_MUL 62500

	u32 div = (params->frequency + CU1216_IF + TUNER_MUL / 2) / TUNER_MUL;

	buf[0] = (div >> 8) & 0x7f;
	buf[1] = div & 0xff;
	buf[2] = 0xce;
	buf[3] = (params->frequency < 150000000 ? 0x01 :
		  params->frequency < 445000000 ? 0x02 : 0x04);
	buf[4] = 0xde;
	buf[5] = 0x20;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	if (i2c_transfer(&mantis->adapter, &msg, 1) != 1)
		return -EIO;

	/* wait for the pll lock */
	msg.flags = I2C_M_RD;
	msg.len = 1;
	for (i = 0; i < 20; i++) {
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);

		if (i2c_transfer(&mantis->adapter, &msg, 1) == 1 && (buf[0] & 0x40))
			break;

		msleep(10);
	}

	/* switch the charge pump to the lower current */
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &buf[2];
	buf[2] &= ~0x40;
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);

	if (i2c_transfer(&mantis->adapter, &msg, 1) != 1)
		return -EIO;

	return 0;
}
