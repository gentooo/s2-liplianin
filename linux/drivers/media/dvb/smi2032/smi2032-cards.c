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
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "smi2032.h"

unsigned int smi2032_devcount;

struct smi2032_board smi2032_boards[] = {
	[SMI2032_BOARD_UNKNOWN] = {
		.name		= "UNKNOWN/GENERIC",
	},
	[SMI2032_BOARD_DVBWORLD_2006] = {
		.name		= "DVBWorld PCI 2006",
		.portb		= SMI2032_MPEG_DVB,
	},
};
const unsigned int smi2032_bcount = ARRAY_SIZE(smi2032_boards);

struct smi2032_subid smi2032_subids[] = {
	{
		.subvendor = 0x0000,
		.subdevice = 0x2006,
		.card      = SMI2032_BOARD_DVBWORLD_2006,
	}, {
		.subvendor = 0x1ade,
		.subdevice = 0x1234,
		.card      = SMI2032_BOARD_DVBWORLD_2006,
	},
};
const unsigned int smi2032_idcount = ARRAY_SIZE(smi2032_subids);

void smi2032_card_list(struct smi2032_dev *dev)
{
	int i;

	if (0 == dev->pci->subsystem_vendor &&
			0 == dev->pci->subsystem_device) {
		printk(KERN_ERR
			"smi2032: Your board has no valid PCI Subsystem ID\n"
			"smi2032: and thus can't be autodetected\n"
			"smi2032: Please pass card=<n> insmod option to\n"
			"smi2032: workaround that.  Redirect complaints to\n"
			"smi2032: the vendor of the TV card.  Best regards,\n"
			"smi2032: -- tux\n");
	} else {
		printk(KERN_ERR
			"smi2032: Your board isn't known (yet) to the driver.\n"
			"smi2032: You can try to pick one of the existing\n"
			"smi2032: card configs via card=<n> insmod option.\n"
			"smi2032: Updating to the latest version might help\n"
			"smi2032: as well.\n");
	}
	printk(KERN_ERR "Here is a list of valid choices for the card=<n> "
		   "insmod option:\n");
	for (i = 0; i < ARRAY_SIZE(smi2032_boards); i++)
		printk(KERN_ERR "smi2032:    card=%d -> %s\n",
				i, smi2032_boards[i].name);
}
