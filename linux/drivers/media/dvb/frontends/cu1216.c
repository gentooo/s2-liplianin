/*
	CU-1216 driver for the Mantis bridge based cards

	Copyright (C) 2005 Twinhan Technology Co. Ltd
		based on the TDA 10021 driver

	Copyright (C) 1999 Convergence Integrated Media GmbH <ralph@convergence.de>
	Copyright (C) 2004 Markus Schulz <msc@antzsystem.de>

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


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "dvb_frontend.h"
#include "mantis_core.h"
#include "cu1216.h"
#include "cu1216_regs.h"

unsigned int verbose = 1;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "print AFC offset after tuning for debugging the PWM setting");

struct cu1216_state {
	struct i2c_adapter *i2c;
	struct dvb_frontend_ops ops;

	/*	config settings		*/
	const struct cu1216_config *config;
	struct dvb_frontend frontend;

	u8 pwm;
	u8 reg0;

	struct dvb_frontend_parameters params;
};

typedef struct AC_TypeQAM_TAG {
	u8 bConf;
	u8 bAgcref;
	u8 bLockthr;
	u8 bMseth;
	u8 bAref;
} AC_TypeQAM_T;




static u32 AC_uSysClk;
static u8  li_Iq, li_oldIq = 0, uc_Gain, uc_oldGain = 0;

static void  cu1216_set_symbolRate(struct dvb_frontend *fe, u16 uFreqSymb);
static void  cu1216_set_QAM(struct dvb_frontend *fe, u8  bQAM);
static void  cu1216_set_IQ(struct dvb_frontend *fe, u8  bSI);
static void  cu1216_set_gain(struct dvb_frontend *fe, u8  bGain);
static void  cu1216_clear_register(struct dvb_frontend *fe);

#if 0
static void  cu1216_reset         (struct dvb_frontend *fe);
#endif

static int   cu1216_init          (struct dvb_frontend *fe);

static int cu1216_writereg (struct cu1216_state *state, u8 reg, u8 data)
{
	u8 buf[] = { reg, data };

	struct i2c_msg msg = {
		.addr = state->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};
	int ret;
#if 0
	printk("cu1216 write %d: %02x\n", reg, data);
#endif
	ret = i2c_transfer (state->i2c, &msg, 1);
	if (ret != 1)
	printk("DVB: TDA10021(%d): %s, writereg error "
	       "(reg == 0x%02x, val == 0x%02x, ret == %i)\n",
	       state->frontend.dvb->num, __FUNCTION__, reg, data, ret);

	msleep(10);
	return (ret != 1) ? -EREMOTEIO : 0;
}

static u8 cu1216_readreg (struct cu1216_state *state, u8 reg)
{
	u8 b0 [] = { reg };
	u8 b1 [] = { 0 };
	struct i2c_msg msg [] = {
		{
			.addr = state->config->demod_address,
			.flags = 0,
			.buf = b0,
			.len = 1
		},
		{
			.addr = state->config->demod_address,
			.flags = I2C_M_RD,
			.buf = b1,
			.len = 1
		}
	};
	int ret;

	ret = i2c_transfer(state->i2c, msg, 2);
	if (ret != 2)
		printk("DVB: TDA10021: %s: readreg error (ret == %i)\n",
			__FUNCTION__, ret);
	return b1[0];
}

static int cu1216_writereg_mask (struct cu1216_state *state, u8 reg, u8 mask, u8 data)
{
	u8  value;

	value = cu1216_readreg(state, reg);

	value = (value & ~mask) | (data & mask);
	//value = (value & ~mask) | data ;

	return cu1216_writereg(state, reg, value);
}


//get access to tuner
static int lock_tuner(struct cu1216_state *state)
{
	u8 buf[2] = { 0x0f, 0x40 | 0x80 };
	struct i2c_msg msg = {
		.addr = state->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	if (i2c_transfer(state->i2c, &msg, 1) != 1) {
		printk("cu1216: lock tuner fails\n");
		return -EREMOTEIO;
	}
	return 0;
}

//release access from tuner
static int unlock_tuner(struct cu1216_state *state)
{
	u8 buf[2] = { 0x0f, 0x40 & 0x7f };
	struct i2c_msg msg_post = {
		.addr = state->config->demod_address,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	if (i2c_transfer(state->i2c, &msg_post, 1) != 1) {
		printk("cu1216: unlock tuner failed\n");
		return -EREMOTEIO;
	}

	return 0;
}

static void cu1216_set_symbolRate(struct dvb_frontend *fe, u16 uFreqSymb)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8	pWrite[4], bNdec, bSFil;
	u32  uBDR, uBDRb, uFreqSymbInv, uFreqSymb480;

	/*	Return variables		*/
	u32 fSysClk;

	/*	calculate system frequency	*/
	fSysClk  = OM5734_XTALFREQ_DEF * (OM5734_PLLMFACTOR_DEF + 1);
	fSysClk /= (OM5734_PLLNFACTOR_DEF + 1) * (OM5734_PLLPFACTOR_DEF + 1);

	/*	add 480 ppm to the SR		*/
	uFreqSymb480  = uFreqSymb;
	uFreqSymb480  = uFreqSymb * 480;      // I Don't Know why it's * 480 not + 480
	uFreqSymb480 /= 1000000L;
	uFreqSymb480  =	uFreqSymb + uFreqSymb480;
	fSysClk = fSysClk/1000;
	bNdec = 0;
	bSFil = 1;

	if (((fSysClk / 123) < (uFreqSymb480 / 10)) &&
	    ((uFreqSymb480 / 10)  <= (fSysClk / 80)))
		bSFil = 0;

	if (((fSysClk / 246) < (uFreqSymb480 / 10)) &&
	    ((uFreqSymb480 / 10)  <= (fSysClk / 160)))
		bSFil = 0;

	if (((fSysClk / 492) < (uFreqSymb480 / 10)) &&
	    ((uFreqSymb480 / 10)  <= (fSysClk / 320)))
		bSFil = 0;

	if (((fSysClk / 984) < (uFreqSymb480 / 10)) &&
	    ((uFreqSymb480 / 10)  <= (fSysClk / 640)))
		bSFil = 0;

	// program SFIL
	//cu1216_writereg_mask(state, AC_GAIN_IND, AC_GAIN_SFIL_MSK, (u8)(bSFil<<4));
	cu1216_writereg(state, AC_GAIN_IND, 0x23);

	// program NDEC
	//cu1216_writereg_mask(state, AC_CLKCONF_IND, AC_CLKCONF_NDEC_MSK, (u8)(bNdec<<6));
	cu1216_writereg(state, AC_CLKCONF_IND, 0x0a);

	//---------------------------------------
	// program the symbol frequency registers
	//---------------------------------------
	// calculate the inversion of the symbol frequency
	uFreqSymbInv   = fSysClk * 16; // prefer to P21/58 Ice_Deng 2003/12/20
	uFreqSymbInv >>= bNdec;		    // divide by 2^decim
	uFreqSymbInv  += uFreqSymb / 2;	// rounding for division
	uFreqSymbInv  /= uFreqSymb;

	if (uFreqSymbInv > 255)
		uFreqSymbInv = 255;

	uBDRb = 1;
	uBDRb = uBDRb << (24 + bNdec);

	fSysClk = fSysClk / 10;
	uBDR = uBDRb / fSysClk;
	uBDR *= uFreqSymb;

	uBDRb %= fSysClk;
	uBDRb *= uFreqSymb;
	uBDRb /= fSysClk;
	uBDR  += uBDRb;
	uBDR  /= 10;

	// program the value in register of the symbol rate
	pWrite[0] = (unsigned char)(uBDR);
	pWrite[1] = (unsigned char)(uBDR >> 8);
	pWrite[2] = (unsigned char)(uBDR >> 16);
	pWrite[3] = (unsigned char)uFreqSymbInv;

	cu1216_writereg(state, AC_BDRLSB_IND, pWrite[0]);
	cu1216_writereg(state, AC_BDRMID_IND, pWrite[1]);
	cu1216_writereg(state, AC_BDRMSB_IND, pWrite[2]);
	cu1216_writereg(state, AC_BDRINV_IND, pWrite[3]);
}

/*	The default is 16-QAM  refer to P31/58		*/
static void cu1216_set_QAM(struct dvb_frontend *fe, u8  bQAM)
{
	struct cu1216_state *state = fe->demodulator_priv;

	AC_TypeQAM_T sTypeQAM[] = {
		{ 0x14, 120, 0x78, 114, 0x96 },		/*	4 QAM    <=> qam=0	*/
		{ 0x00, 140, 0x6e, 162, 0x91 },		/*	16 QAM   <=> qam=1	*/
		{ 0x04, 140, 0x4b, 116, 0x96 },		/*	32 QAM   <=> qam=2	*/
		{ 0x08, 106, 0x37, 67, 0x6a  },		/*	64 QAM   <=> qam=3	*/
		{ 0x0c, 120, 0x2d, 52, 0x7e  },		/*	128 QAM  <=> qam=4	*/
		{ 0x10, 92, 0x23, 35, 0x6b   },		/*	256 QAM  <=> qam=5	*/
	};

	// program the modulation in CONF register
	cu1216_writereg_mask(state, AC_CONF_IND, AC_CONF_QAM_MSK, sTypeQAM[bQAM].bConf);
	//cu1216_writereg(state, AC_CONF_IND, 0x6b);

	cu1216_writereg(state, AC_AGCREF_IND, sTypeQAM[bQAM].bAgcref);		/*	AGCREF		*/
	cu1216_writereg(state, AC_LOCKTHR_IND, sTypeQAM[bQAM].bLockthr);	/*	LOCKTHR		*/
	cu1216_writereg(state, AC_MSETH_IND, sTypeQAM[bQAM].bMseth);		/*	MSETH		*/
	cu1216_writereg(state, AC_AREF_IND, sTypeQAM[bQAM].bAref);		/*	AREF		*/
}

static void cu1216_set_IQ(struct dvb_frontend *fe, u8 bSI)
{
	struct cu1216_state *state = fe->demodulator_priv;

	bSI = (bSI+1) % 2;				/*	Set Spectral Inversion MODE		*/

	// write the ConfReg Register
	cu1216_writereg_mask(state, AC_CONF_IND, AC_CONF_INVIQ_BIT, (u8)(bSI << 5));
}

static void cu1216_set_gain(struct dvb_frontend *fe, u8  bGain)
{
	struct cu1216_state *state = fe->demodulator_priv;

	// write the gain
	cu1216_writereg_mask(state, AC_GAIN_IND, AC_GAIN_GNYQ_MSK, (u8)(bGain << 5));
}

static int cu1216_read_status(struct dvb_frontend *fe, fe_status_t *status)
{
	struct cu1216_state *state = fe->demodulator_priv;
#if 1
	int sync;

	*status = 0;
	/*	0x11[0] == EQALGO	-> Equalizer algorithms state		*/
	/*	0x11[1] == CARLOCK	-> Carrier locked			*/
	/*	0x11[2] == FSYNC	-> Frame synchronisation		*/
	/*	0x11[3] == FEL		-> Front End locked			*/
	/*	0x11[6] == NODVB	-> DVB Mode Information			*/
	sync = cu1216_readreg (state, 0x11);

	if (sync & 2)
		*status |= FE_HAS_SIGNAL | FE_HAS_CARRIER;

	if (sync & 4)
		*status |= FE_HAS_SYNC | FE_HAS_VITERBI;

	if (sync & 8)
		*status |= FE_HAS_LOCK;

	 return 0;
#else
	u8  uc_Index11 = 0x0;
	u8  uc_Read;
	u32 loopCount = 10;

	*status = 0 ;

	while (loopCount--) {
		uc_Index11 = cu1216_readreg(state, 0x11);
		//printk("lock status: %02x\n", uc_Index11);
		uc_Read = uc_Index11 & 0x0F;

		//*status = FE_HAS_LOCK;
		//*status |= FE_HAS_SIGNAL|FE_HAS_CARRIER;
		//*status |= FE_HAS_SYNC|FE_HAS_VITERBI;

		if ((uc_Read == 0x0f) && (uc_Index11 != 0xFF)) {
			*status = FE_HAS_LOCK ;
			break;
		}
		//mdelay(10);
	}

	return 0;
#endif
}


static int cu1216_read_errRate(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;

	int cCkOffset = 0;
	int iErrRyt;

	// read offset
	cCkOffset = cu1216_readreg(state, 0x1d);

	// convert the value to long
	if (cCkOffset < 0)
		iErrRyt = (int)(0xFFFFFF00 | cCkOffset);
	else
		iErrRyt = (int)cCkOffset;

	// read DYN bit
	cCkOffset = cu1216_readreg(state, 0x03);

	// calculate the error in ppm
	iErrRyt *= 1000000;
	iErrRyt /= 262144;

	if (!(cCkOffset & 0x08))
		iErrRyt /= 2;

	return iErrRyt;
}



static int cu1216_read_quality(struct dvb_frontend *fe, u16 *quality)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8 tempval;
	fe_status_t tunerStatus;

	cu1216_read_status(fe, &tunerStatus);

	if (tunerStatus == 0) {
		*quality = 0;
		return 0;
	}

	tempval = cu1216_readreg(state, 0x18);

	if (tempval <= 5) {
		*quality = 98;

		return 0;

	} else if(tempval <= 10) {
		*quality = 108 - 2*tempval;//88

		return 0;
	}

	switch (state->params.u.qam.modulation) {
	case QPSK:
	case QAM_16:
		if (tempval <= 110)
			*quality = 60 + (110 - tempval) * 3 / 10;//0.3
		else if (tempval <= 120)
			*quality = 30 + (120 - tempval) * 30 / 10;
		else
			*quality = (255 - tempval) * 30 / 135;
		break;
	case QAM_32:
		if (tempval <= 72)
			*quality = 60 + (72 - tempval) * 15 / 31;//0.5
		else if (tempval <= 82)
			*quality = 30 + (83 - tempval) * 30 / 11;
		else
			*quality = (255 - tempval) * 30 / 172;
		break;
	default:
	case QAM_64:
		if (tempval <= 42)
			*quality = 60 + (42- tempval) * 7 / 8;//0.875
		else if (tempval <= 52)
			*quality = 30 + (52 - tempval) * 30 / 10;
		else
			*quality = (255-tempval)*30/203;
		break;
	case QAM_128:
		if (tempval <= 28)
			*quality = 60 + (28 - tempval) * 14 / 9;//1.5
		else if(tempval <= 34)
			*quality = 30 + (34 - tempval) * 30 / 6;
		else
			*quality = (255 - tempval) * 30 / 221;
		break;
	case QAM_256:
		if (tempval <= 18)
			*quality = 60 + (18 - tempval) * 7 / 2;//3.5
		else if (tempval <= 22)
			*quality = 30 + (22 - tempval) * 30 / 4;
		else
			*quality = (255 - tempval) * 30 / 233;
		break;
	}

    return 0;
}


static int cu1216_read_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8	tempagc;

	tempagc = cu1216_readreg(state, AC_VAGC1_IND);

	if (tempagc > 0xF0)
		*strength = 2;
	else if (tempagc > 0xE5)
		*strength = 4;
	else if (tempagc < 0x6E)
		*strength = 98;
	else if (tempagc < 0x96)
		*strength = 70 + (0x96 - tempagc) / 2 * 7 / 5;
	else if (tempagc < 0xCD)
		*strength = 14 + (0xCD - tempagc);
	else
		*strength = 2 + (0xE5 - tempagc) / 2;

	return  0;
}



static int cu1216_read_ber(struct dvb_frontend *fe, u32 *BERvalue)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8 tempval;

	tempval = cu1216_readreg(state, 0x16);

	tempval &= 0x0f;
	*BERvalue = tempval;
	*BERvalue <<=8;

	tempval = cu1216_readreg(state, 0x15);

	*BERvalue += tempval;
	*BERvalue <<=8;

	tempval = cu1216_readreg(state, 0x14);

	*BERvalue += tempval;

	tempval = cu1216_readreg(state, 0x10);

	switch (tempval & 0xc0) {
	default:
	case 0x00:  /*	1,00E+05	*/
		*BERvalue *= 80;
		break;
	case 0x40:  /*	1,00E+06	*/
		*BERvalue *= 10;
		break;
	case 0x80:  /*	1,00E+07	*/
		*BERvalue /= 1;
		break;
	case 0xc0:  /*	1,00E+08	*/
		*BERvalue /= 10;
		break;
	}

	return 0;
}

static int cu1216_read_snr (struct dvb_frontend *fe, u16 *SNRvalue)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8 tempagc;

	tempagc = cu1216_readreg(state, 0x18);

	*SNRvalue = tempagc;
	switch (state->params.u.qam.modulation) {
	case QAM_16:
		*SNRvalue = 195000 / (32 * (*SNRvalue) + 138) + 100;
		break;
	case QAM_32:
		*SNRvalue = 215000 / (40 * (*SNRvalue) + 500) + 135;
		break;
	default:
	case QAM_64:
		*SNRvalue = 210000 / (40 * (*SNRvalue) + 500) + 125;
		break;
	case QAM_128:
		*SNRvalue = 185000 / (38 * (*SNRvalue) + 400) + 138;
		break;
	case QAM_256:
		*SNRvalue = 180000 / (100 * (*SNRvalue) + 40) + 203;
		break;
	}

	return 0;
}



static int cu1216_read_ubk(struct dvb_frontend *fe, u32 *ubk)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8	puBytes[4];
	u32	puUBK;

	/*	Implementation		*/
	puBytes[0] = cu1216_readreg(state, AC_CPTUNCOR_IND);
	puBytes[1] = cu1216_readreg(state, AC_BERLSB_IND);
	puBytes[2] = cu1216_readreg(state, AC_BERMID_IND);
	puBytes[3] = cu1216_readreg(state, AC_BERMSB_IND);


	puUBK = (puBytes[3] << 24) | (puBytes[2] << 16) | (puBytes[1] << 8) | puBytes[0];

	/*	mask the reset flag	*/
	puUBK &= AC_CPTUNCOR_CPTU_MSK;

	/*	reset counter, if uncorrectables	*/
	if (puUBK) {
		cu1216_writereg_mask(state, AC_RSCONF_IND, AC_RSCONF_CLBUNC_BIT, 0);
		cu1216_writereg_mask(state, AC_RSCONF_IND, AC_RSCONF_CLBUNC_BIT,AC_RSCONF_CLBUNC_BIT);
	}
	*ubk =  puUBK;

	return 0;
}

static void cu1216_clear_register(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;

	cu1216_writereg(state, AC_CONF_IND, 0x6a);
}

/*
static void cu1216_reset(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;

//	mantis_fe_reset((struct mantis_pci *) state->frontend.dvb->priv);
	mantis_fe_reset((struct mantis_pci *) state->frontend.dvb->priv);


	//    set_gpio_A12A13A14(mantis, 13, 0);
	//    mdelay(100);
	//    set_gpio_A12A13A14(mantis, 13, 1);


	switch (mantis->card_version) {
	case 0x07:
		mantisReg->gpio_0123 &= 0x80ffffff;
		mantisReg->gpio_0123 |= 0x80000000;
		mdelay(100);
		mantisReg->gpio_0123 &= 0x80ffffff;
		mantisReg->gpio_0123 |= 0x82000000;
		break;

	case 0x08:
		set_gpio_A12A13A14(mantis, 13, 0);
		mdelay(100);
		set_gpio_A12A13A14(mantis, 13, 1);
		break;
	}
	DBGMSG("<<cu1216_reset()\n");

}
*/

static int cu1216_init_none(struct dvb_frontend *fe)
{
	return 0;
}

static int cu1216_init(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8  bByte;
	s32 lDeltaF;

	// calculate the system frequency
	AC_uSysClk  = OM5734_XTALFREQ_DEF * (OM5734_PLLMFACTOR_DEF + 1);
	AC_uSysClk /= (OM5734_PLLNFACTOR_DEF + 1) * (OM5734_PLLPFACTOR_DEF + 1);

	// PLL factors
	// bPLL_M_Factor =0x07
	// bPLL_N_Factor =0x00
	// bPLL_P_Factor =0x03

	cu1216_writereg(state, AC_MDIV_IND, OM5734_PLLMFACTOR_DEF);
	cu1216_writereg(state, AC_NDIV_IND, OM5734_PLLNFACTOR_DEF);
	cu1216_writereg(state, AC_PLL_IND, OM5734_PLLPFACTOR_DEF);

	// add by ice_Deng 2004/01/06
	cu1216_writereg(state, AC_CONTROL_IND, 0x0d);

	// enable the PLL
	cu1216_writereg_mask(state, AC_PLL_IND, AC_PLL_BYPPLL_BIT, 0);

	// enable AGC2 and set PWMREF
	cu1216_writereg_mask(state, AC_AGCCONF2_IND, AC_AGCCONF2_ENAGC2_BIT, AC_AGCCONF2_ENAGC2_BIT);
	cu1216_writereg(state, AC_PWMREF_IND, AC_PWMREF_DEF);

	// use internal ADC
	//cu1216_writereg_mask(state, AC_ADC_IND, AC_ADC_PCLK_BIT, AC_ADC_PCLK_BIT);
	//cu1216_writereg_mask(state, AC_ADC_IND, AC_ADC_SW_MSK, AC_ADC_SW_DEF);
	cu1216_writereg(state, AC_ADC_IND, 0x31);
	cu1216_writereg(state, AC_ADC_IND, 0x31);
	// use only nyquist gain
	//cu1216_writereg_mask(state, AC_CLKCONF_IND, AC_CLKCONF_GAIN3_BIT, 0);
	cu1216_writereg(state, AC_CLKCONF_IND, 0x0a);
	// set the acquisition to +/-480ppm
	//cu1216_writereg_mask(state, AC_CLKCONF_IND, AC_CLKCONF_DYN_BIT, AC_CLKCONF_DYN_BIT);
	cu1216_writereg(state, AC_CLKCONF_IND, 0x0a);
	// POS_AGC - not in data sheet
#if 0
	cu1216_writereg_mask(state, AC_AGCCONF1_IND, AC_AGCCONF1_POSAGC_BIT, AC_AGCCONF1_POSAGC_BIT);
	// set the polarity of the PWM for the AGC
	if (AC_POLAPWM1_DEF)
		cu1216_writereg_mask(state, AC_AGCCONF1_IND, AC_AGCCONF1_PPWM1_BIT, AC_AGCCONF1_PPWM1_BIT);
	else
		cu1216_writereg_mask(state, AC_AGCCONF1_IND, AC_AGCCONF1_PPWM1_BIT, 0);
#else
	cu1216_writereg(state, AC_AGCCONF1_IND, 0x23);
	cu1216_writereg(state, AC_AGCCONF1_IND, 0x23);
#endif
	if (AC_POLAPWM2_DEF)
		cu1216_writereg_mask(state, AC_AGCCONF2_IND, AC_AGCCONF2_PPWM2_BIT, AC_AGCCONF2_PPWM2_BIT);
	else
		cu1216_writereg_mask(state, AC_AGCCONF2_IND, AC_AGCCONF2_PPWM2_BIT, 0);

	// set the threshold for the IF AGC
	cu1216_writereg(state, AC_IFMAX_IND, AC_IFMAX_DEF);
	cu1216_writereg(state, AC_IFMIN_IND, 0); //AC_IFMIN_DEF150 ;


	// set the threshold for the TUN AGC
	cu1216_writereg(state, AC_TUNMAX_IND, AC_TUNMAX_DEF);
	cu1216_writereg(state, AC_TUNMIN_IND, AC_TUNMIN_DEF);


	// set the counter of saturation to its maximun size
	//cu1216_writereg_mask(state, AC_GAIN_IND, AC_GAIN_SSAT_MSK, 0x03);
	cu1216_writereg(state, AC_GAIN_IND, 0x23);

	// set the MPEG output clock polarity
	//cu1216_writereg_mask(state, AC_POLA1_IND, AC_POLA1_POCLK1_BIT, 1);

	//cu1216_writereg(state, 0x12, 0xa0);

	// Added By IceDeng 12/15/2004 For Ts 188
	//cu1216_writereg_mask(state, AC_POLA1_IND, 0x40, 0x40);
	cu1216_writereg(state, 0x12, 0xe1);
	cu1216_writereg(state, 0x12, 0xe1);

	//cyq channge star
	cu1216_writereg_mask(state, AC_POLA2_IND, AC_POLA2_POCLK2_BIT, AC_POLA2_POCLK2_BIT);

	// set the position of the central coeffcient
	cu1216_writereg_mask(state, AC_EQCONF1_IND, AC_EQCONF1_POSI_MSK, 0x70);

	// set the equalizer type
	if (AC_EQUALTYPE_DEF)
		cu1216_writereg_mask(state, AC_EQCONF1_IND, AC_EQCONF1_DFE_BIT, AC_EQUALTYPE_DEF-1);

	cu1216_writereg_mask(state, AC_EQCONF2_IND, AC_EQCONF2_SGNALGO_BIT, AC_EQCONF2_SGNALGO_BIT);

	// set ALGOD and deltaF
	//57840000 = OM5734_XTALFREQ_DEF*(OM5734_PLLMFACTOR_DEF+1) / (OM5734_PLLNFACTOR_DEF+1)*(OM5734_PLLPFACTOR_DEF+1)

	lDeltaF  = (s32)(AC_uSysClk * 5 / 1000);
	lDeltaF /= -8;
	lDeltaF += (AC_TUNFI_DEF / 1000);
	lDeltaF *= 2048;
	lDeltaF /= (s32)(AC_uSysClk / 1000);

	cu1216_writereg(state, AC_DELTAF1_IND, (u8)lDeltaF);
	cu1216_writereg(state, AC_DELTAF2_IND, (u8)(((lDeltaF>>8) & 0x03) | AC_DELTAF2_ALGOD_BIT));

	// set the KAGC to its maximun value
	cu1216_writereg_mask(state, AC_AGCCONF1_IND, AC_AGCCONF1_KAGC_MSK, 0x03);

	// set carrier algorithm parameters and SELCAR
	bByte  = AC_SWEEP_DEF;
	bByte |= (u8)AC_CAROFFLENGTH_DEF;
	bByte |= (u8)(AC_CAROFFSTEP_DEF << 2);
	//bByte |= (u8)(AC_CARRANGE_DEF << 4);

	cu1216_writereg(state, AC_SWEEP_IND, bByte);

	// TS interface 1
	bByte = AC_INTP_DEF;
	if (AC_MSBFIRST1_DEF)
		bByte |= AC_INTP_MSBFIRST_BIT;
	if( AC_PARASER1_DEF)
		bByte |= AC_INTP_INTSEL_BIT;
	else
		bByte |= AC_INTP_MSBFIRST_BIT;	// set to 1 MSB if parallel

	if (AC_MODEAB1_DEF) {
		bByte |= AC_INTP_PARMOD_BIT;
		bByte |= (AC_PARADIV1_DEF << 4);
	}
	cu1216_writereg(state, AC_INTP_IND, bByte);
	cu1216_writereg_mask(state, AC_POLA2_IND, AC_POLA2_MSBFIRST2_BIT, AC_POLA2_MSBFIRST2_BIT);

	// set the BER depth
	cu1216_writereg_mask(state, AC_RSCONF_IND, AC_RSCONF_PVBER_MSK, (AC_BERDEPTH_DEF<< 6));

	return 0;
}

static void delay_ms_interruptible(u32 ms)
{
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(HZ * ms / 100);
}

static void delay_us_interruptible(u32 us)
{
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(HZ * us / 10000);
}


static int cu1216_set_parameters(struct dvb_frontend *fe, struct dvb_frontend_parameters *params)
{
	struct cu1216_state *state = fe->demodulator_priv;

	u8 i;
	u8 QamSize = 0;
	u32 ErrRate[3];
	fe_status_t value;
	int status = -EINVAL;

	printk("[%s]:frequency = %d , symbol = %d , qam = %d .\n",
		__func__,
		params->frequency , params->u.qam.symbol_rate,
		params->u.qam.modulation);

	switch (params->u.qam.modulation) {
	case QPSK   :
		QamSize = 0;
		break;
	case QAM_16 :
		QamSize = 1;
		break;
	case QAM_32 :
		QamSize = 2;
		break;
	case QAM_64 :
		QamSize = 3;
		break;
	case QAM_128:
		QamSize = 4;
		break;
	case QAM_256:
		QamSize = 5;
		break;
	default :
		printk("[cu1216_set_parameters]:QAM set error!\n");
		break;
	}

	if (li_oldIq >= 2)
		li_oldIq = 0;

//	cu1216_reset(fe);
//	FIXME ! need to do a Bridge RESET from here
//	state->config->fe_reset(fe);

	//To clear the Registers in TDA10021HT
	cu1216_clear_register(fe);

	//Write Frequency into tuner
	lock_tuner(state);
	state->config->pll_set(fe, params);
	unlock_tuner(state);

	//mdelay(20);
	delay_ms_interruptible(10);

	//Second step to init the cu1216ht's registers
	cu1216_init(fe);

	//Write Symborate
	cu1216_set_symbolRate(fe, params->u.qam.symbol_rate / 1000);

	//Write QAM
	cu1216_set_QAM(fe, QamSize);

	for (i = li_oldIq; i < li_oldIq + 2; i++) {
		li_Iq = i % 2;

		for (uc_Gain = 1; uc_Gain < 4; uc_Gain++) {
			cu1216_set_IQ(fe, li_Iq);

			cu1216_set_gain(fe, uc_Gain);

			//udelay(50);
			delay_us_interruptible(5);

			if (cu1216_read_status(fe, &value) == 0) {

				li_oldIq   = li_Iq;
				uc_oldGain = uc_Gain;
				ErrRate[0] = cu1216_read_errRate(fe);

				if (uc_Gain < 3) {
					cu1216_set_gain(fe, uc_Gain+1);
					//udelay(50);
					delay_us_interruptible(5);
					ErrRate[1] = cu1216_read_errRate(fe);

					if (ErrRate[0] > ErrRate[1]) {
						cu1216_set_gain(fe , uc_Gain);
						//udelay(50);
						delay_us_interruptible(5);

					} else {
						uc_oldGain = uc_Gain + 1;
						uc_Gain = uc_Gain + 1;

						if (uc_Gain < 3) {
							cu1216_set_gain(fe, uc_Gain + 1);

							//udelay(50);
							delay_us_interruptible(5);
							ErrRate[2] = cu1216_read_errRate(fe);

							if (ErrRate[1] > ErrRate[2]) {
								cu1216_set_gain(fe , uc_oldGain);

								//udelay(50);
								delay_us_interruptible(5);
							} else {
								uc_oldGain = uc_Gain + 1;
							}
						}
					}
				}
				goto ret;
			}
		}
	}

	status = -1;

ret:

	state->params = *params;

	return status ;
}

static int cu1216_get_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct cu1216_state *state = fe->demodulator_priv;
	int sync;
	s8 afc = 0;

	sync = cu1216_readreg(state, 0x11);
	afc = cu1216_readreg(state, 0x19);
	if (verbose) {
	/* AFC only valid when carrier has been recovered */
		printk(sync & 2 ? "DVB: TDA10021(%d): AFC (%d) %dHz\n" :
		       "DVB: TDA10021(%d): [AFC (%d) %dHz]\n",
		       state->frontend.dvb->num, afc,
		       - ((s32)p->u.qam.symbol_rate * afc) >> 10);
	}

	p->inversion = HAS_INVERSION(state->reg0) ? INVERSION_ON : INVERSION_OFF;
	p->u.qam.modulation = ((state->reg0 >> 2) & 7) + QAM_16;

	p->u.qam.fec_inner = FEC_NONE;
	p->frequency = ((p->frequency + 31250) / 62500) * 62500;

	if (sync & 2)
		p->frequency -= ((s32)p->u.qam.symbol_rate * afc) >> 10;

	return 0;
}

static int cu1216_get_tune_settings(struct dvb_frontend *fe,
				    struct dvb_frontend_tune_settings *p)
{
	p->min_delay_ms = 50;
	p->step_size = 0;
	p->max_drift = 0;

	return 0;
}

static int cu1216_sleep(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;

	cu1216_writereg (state, 0x1b, 0x02);  /* pdown ADC */
	cu1216_writereg (state, 0x00, 0x80);  /* standby */

	return 0;
}

static void cu1216_release(struct dvb_frontend *fe)
{
	struct cu1216_state *state = fe->demodulator_priv;
	kfree(state);
}

static struct dvb_frontend_ops cu1216_ops;

struct dvb_frontend *cu1216_attach(const struct cu1216_config *config,
				   struct i2c_adapter *i2c)
{
	struct cu1216_state *state = NULL;

	/* allocate memory for the internal state */
	state = kmalloc(sizeof (struct cu1216_state), GFP_KERNEL);
	if (state == NULL)
		goto error;

	/* setup the state */
	state->config = config;
	state->i2c = i2c;
	memcpy(&state->ops, &cu1216_ops, sizeof (struct dvb_frontend_ops));
	//state->pwm = pwm;
	//state->reg0 = cu1216_inittab[0];

	/* check if the demod is there */
	if ((cu1216_readreg(state, 0x1a) & 0xf0) != 0x70)
		goto error;

	/* create dvb_frontend */
	state->frontend.ops = state->ops;
	state->frontend.demodulator_priv = state;
	return &state->frontend;

error:
	kfree(state);
	return NULL;
}

static struct dvb_frontend_ops cu1216_ops = {

	.info = {
		    .name			= "Philips CU1216 DVB-C",
		    .type			= FE_QAM,
		    .frequency_stepsize		= 62500,
		    .frequency_min		= 51000000,
		    .frequency_max		= 858000000,
		    .symbol_rate_min		= (XIN / 2) / 64,	/*	SACLK/64 == (XIN/2)/64	*/
		    .symbol_rate_max		= (XIN / 2) / 4,	/*	SACLK/4			*/
#if 0
		    .frequency_tolerance	= ???,
		    .symbol_rate_tolerance	= ???,			/*	ppm  == 8% (spec p. 5)	*/
#endif
		    .caps			= 0x400		 |	/*	FE_CAN_QAM_4		*/
						  FE_CAN_QAM_16  |
						  FE_CAN_QAM_32  |
						  FE_CAN_QAM_64  |
						  FE_CAN_QAM_128 |
						  FE_CAN_QAM_256 |
						  FE_CAN_FEC_AUTO
	},

	.release				= cu1216_release,
	.init					= cu1216_init_none,
	.sleep					= cu1216_sleep,
	.set_frontend				= cu1216_set_parameters,
	.get_frontend				= cu1216_get_frontend,
	.read_status				= cu1216_read_status,
	.read_ber				= cu1216_read_ber,
	.read_signal_strength			= cu1216_read_strength,
	.read_snr				= cu1216_read_snr,
	.read_ucblocks				= cu1216_read_ubk,
	.get_tune_settings			= cu1216_get_tune_settings,
};


MODULE_DESCRIPTION("Philips CU1216 DVB-C demodulator driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(cu1216_attach);
