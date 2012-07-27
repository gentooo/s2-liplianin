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

#ifndef __CU1216_REGS_H
#define __CU1216_REGS_H

#define XIN 57840000UL

#define DISABLE_INVERSION(reg0)		do { reg0 |= 0x20; } while (0)
#define ENABLE_INVERSION(reg0)		do { reg0 &= ~0x20; } while (0)
#define HAS_INVERSION(reg0)		(!(reg0 & 0x20))

#define FIN				(XIN >> 4)

#define C_1216_CHIP_ADDRESS		0x18
#define C_1216_TUNER_ADDRESS		0xc0

#define OM5734_XTALFREQ_DEF		28920000
#define OM5734_PLLMFACTOR_DEF		0x07
#define OM5734_PLLNFACTOR_DEF		0x00
#define OM5734_PLLPFACTOR_DEF		0x03

#define AC_POLAPWM1_DEF			0
#define AC_POLAPWM2_DEF			0
#define AC_IFMAX_DEF			255
#define OM5735_IFMIN_DEF		64
#define OM5734_IFMIN_DEF		88
#define AC_TUNMAX_DEF			255
#define AC_TUNMIN_DEF			0
#define AC_IFMIN_DEF150			150

#define AC_EQUALTYPE_DEF		2
#define AC_BERDEPTH_DEF			2
#define AC_CAROFFSTEP_DEF		1
#define AC_CAROFFLENGTH_DEF		1
#define AC_CARRANGE_DEF			1
#define AC_DISAFC_DEF			0

#define AC_IQSWAPPED_DEF		1
#define AC_TUNID_DEF			0
#define AC_TUNREADENA_DEF		0
#define AC_TUNFI_DEF			36125000

#define AC_POCLK1_DEF			1
#define AC_PARASER1_DEF			0
#define AC_MSBFIRST1_DEF		0
#define AC_MODEAB1_DEF			0
#define AC_PARADIV1_DEF			0
#define AC_POCLK2_DEF			1
#define AC_PARASER2_DEF			0

#define OM5734_DEF			0
#define OM5735_DEF			1
#define CUSTOM_DEF			2

/*	Index register definition	*/
#define AC_CONF_IND			0x00
#define AC_AGCREF_IND			0x01
#define AC_AGCCONF1_IND			0x02
#define AC_CLKCONF_IND			0x03
#define AC_CARCONF_IND			0x04
#define AC_LOCKTHR_IND			0x05
#define AC_EQCONF1_IND			0x06
#define AC_EQSTEP_IND			0X07
#define AC_MSETH_IND			0x08
#define AC_AREF_IND			0x09
#define AC_BDRLSB_IND			0x0A
#define AC_BDRMID_IND			0x0B
#define AC_BDRMSB_IND			0x0C
#define AC_BDRINV_IND			0x0D
#define AC_GAIN_IND			0x0E
#define AC_TEST_IND			0x0F
#define AC_RSCONF_IND			0x10
#define AC_SYNC_IND			0x11
#define AC_POLA1_IND			0x12
#define AC_CPTUNCOR_IND			0x13
#define AC_BERLSB_IND			0x14
#define AC_BERMID_IND			0x15
#define AC_BERMSB_IND			0x16
#define AC_VAGC1_IND			0x17
#define AC_MSE_IND			0x18
#define AC_VAFC_IND			0x19
#define AC_IDENTITY_IND			0x1A
#define AC_ADC_IND			0x1B
#define AC_EQCONF2_IND			0x1C
#define AC_CKOFFSET_IND			0x1D

#define AC_INTP_IND			0x20
#define AC_SATNYQ_IND			0x21
#define AC_SATADC_IND			0x22
#define AC_HALFADC_IND			0x23
#define AC_SATDEC1_IND			0x24
#define AC_SATDEC2_IND			0x25
#define AC_SATDEC3_IND			0x26
#define AC_SATAAF_IND			0x27
#define AC_MDIV_IND			0x28
#define AC_NDIV_IND			0x29
#define AC_PLL_IND			0x2A
#define AC_POLA2_IND			0x2B
#define AC_CONTROL_IND			0x2C
#define AC_SWEEP_IND			0x2D
#define AC_AGCCONF2_IND			0x2E
#define AC_VAGC2_IND			0x2F
#define AC_SATTHR_IND			0x30
#define AC_HALFTHR_IND			0x31
#define AC_ITSEL_IND			0x32
#define AC_ITSTAT_IND			0x33
#define AC_PWMREF_IND			0x34
#define AC_TUNMAX_IND			0x35
#define AC_TUNMIN_IND			0x36
#define AC_DELTAF1_IND			0x37
#define AC_DELTAF2_IND			0x38
#define AC_CONSTI_IND			0x39
#define AC_CONSTQ_IND			0x3A
#define AC_IFMAX_IND			0x3B
#define AC_IFMIN_IND			0x3C

#define AC_REQCO_IND			0x40
#define AC_REQCO_CENTRALCOEF_IND	0x50

#define AC_IEQCO_IND			0x80
#define AC_IEQCO_CENTRALCOEF_IND	0x90

/*	DEFAULT VALUES			*/
#define AC_ADC_SW_DEF			0x30
#define AC_CARCONFHIGHSR_DEF		0x02
#define AC_CARCONFLOWSR_DEF		0x0A
#define AC_CARCONFVERYLOWSR_DEF		0x05
#define AC_CARCONFALGO_DEF		0x0C
#define AC_PWMREF_DEF			0x80
#define AC_INTP_DEF			0x00
#define AC_SWEEP_DEF			0x80

/*	DEFINE VALUES			*/
#define AC_NOSI_VAL			0
#define AC_YESSI_VAL			1
#define AC_AUTOSI_VAL			2

#define AC_PHILIPS_VAL			0
#define AC_PHILIPSLHI_VAL		1
#define AC_SONY_VAL			2
#define AC_NOTUNER_VAL			3

#define AC_16QAM_VAL			0
#define AC_32QAM_VAL			1
#define AC_64QAM_VAL			2
#define AC_128QAM_VAL			3
#define AC_256QAM_VAL			4
#define AC_4QAM_VAL			5

#define AC_AUTOQAM_VAL 2
#define AC_FREF_VAL62500		62500L
#define AC_FREF_VAL			78125

#define AC_PHILIPSLOW_VAL		0xA1
#define AC_PHILIPSMID_VAL		0x92
#define AC_PHILIPSHIGH_VAL		0x34

#define AC_PHILIPSLHILOW_VAL		0x06
#define AC_PHILIPSLHIMID_VAL		0x05
#define AC_PHILIPSLHIHIGH_VAL		0x03

#define AC_VHF1_SONY_VAL		0x01
#define AC_VHF3_SONY_VAL		0x02
#define AC_UHF_SONY_VAL			0x04

#define AC_BER_DEPTH5_VAL		0x00
#define AC_BER_DEPTH6_VAL		0x40
#define AC_BER_DEPTH7_VAL		0x80
#define AC_BER_DEPTH8_VAL		0xC0

#define AC_VERYFASTAGCCONV_VAL		0
#define AC_FASTAGCCONV_VAL		1
#define AC_MIDAGCCONV_VAL		2
#define AC_SLOWAGCCONV_VAL		3

#define AC_ALGOAGCTIMER_VAL		100000
#define AC_ALGOGAINTIMER_VAL		10000
#define AC_ALGOSITIMER_VAL		30000
#define AC_ALGOLOCKTIMER_VAL		200000
#define AC_ALGOLOCKCARRIER_VAL		265533	// 2*SWDYN/SWSTEP*SWLENGTH

#define AC_COEFTRESHOLD_VAL		490000 //562500

#define AC_ALGOGAINMAX_VAL		5
#define AC_ALGOGAINMIN_VAL		0
#define AC_ALGOGAINSCANMIN_VAL		0
#define AC_ALGOGAINSCANMID_VAL		2
#define AC_ALGOGAINSCANMAX_VAL		4

#define AC_DVB_ROLLOFF_VAL		115
#define AC_SCANSTEP_VAL			47
#define AC_FREQSTEP_VAL			62

/*	DEFINE MASKS			*/
#define AC_EQCONF1_POSI_MSK		0x70
#define AC_EQCONF1_ENEQUAL_MSK		0x02
#define AC_ADC_SW_MSK			0x30
#define AC_GAIN_SFIL_MSK		0x10
#define AC_CLKCONF_NDEC_MSK		0xC0
#define AC_CARCONF_MSK			0x3F
#define AC_CONF_QAM_MSK			0x1C
#define AC_RSCONF_PVBER_MSK		0xC0
#define AC_CPTUNCOR_CPTU_MSK		0x7F
#define AC_GAIN_GNYQ_MSK		0xE0
#define AC_AGCCONF1_KAGC_MSK		0x03
#define AC_GAIN_SSAT_MSK		0x03
#define AC_SYNC_BER_MSK			0x30
#define AC_DEMODSTAT_FEL_MSK		0x08
#define AC_DEMODSTAT_UNCOR_MSK		0x80


/*	DEFINE BITS			*/
#define AC_CONF_CLB_BIT			0x01
#define AC_AGCCONF1_POSAGC_BIT		0x20
#define AC_AGCCONF2_ENAGC2_BIT		0x08
#define AC_ADC_PCLK_BIT			0x01
#define AC_TEST_BYPIIC_BIT		0x80
#define AC_EQCONF_ENEQUAL_BIT		0x02
#define AC_CONF_INVIQ_BIT		0x20
#define AC_RSCONF_CLBUNC_BIT		0x20
#define AC_CLKCONF_DYN_BIT		0x08
#define AC_SYNC_NODVB_BIT		0x40
#define AC_CLKCONF_GAIN3_BIT		0x10
#define AC_AGCCONF1_PPWM1_BIT		0x04
#define AC_AGCCONF2_PPWM2_BIT		0x02
#define AC_PLL_BYPPLL_BIT		0x10
#define AC_POLA1_POCLK1_BIT		0x01
#define AC_POLA2_POCLK2_BIT		0x01
#define AC_POLA2_MSBFIRST2_BIT		0x40
#define AC_EQCONF1_DFE_BIT		0x01
#define AC_EQCONF1_ENEQUAL_BIT		0x02
#define AC_EQCONF2_SGNALGO_BIT		0x20
#define AC_EQCONF2_CTADAPT_BIT		0x08
#define AC_DELTAF2_ALGOD_BIT		0x04
#define AC_INTP_INTSEL_BIT		0x01
#define AC_INTP_MSBFIRST_BIT		0x02
#define AC_INTP_PARMOD_BIT		0x08
#define AC_EQCONF1_ENADAPT_BIT		0x04
#define AC_SYNC_CARLOCK_BIT		0x02
#define AC_SYNC_FSYNC_BIT		0x04
#define AC_SYNC_FEL_BIT			0x08

/*	DEFINE RETURN VALUES		*/
#define AC_SUCCESS_RET			0
#define AC_FAILED_RET			1
#define AC_NOT_FINISHED_RET		2
#define AC_NO_ERROR_RET			0

#endif //__CU1216_REGS_H
