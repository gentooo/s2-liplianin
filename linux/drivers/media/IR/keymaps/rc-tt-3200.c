/* rc-tt-3200.c - Keytable for TT s2-3200 Remote Controller
 *
 * Copyright (c) 2010 by Igor M. Liplianin <liplianin@me.by>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <media/rc-map.h>

static struct ir_scancode tt_3200[] = {
	{ 0x03, KEY_1 },
	{ 0x04, KEY_2 },
	{ 0x05, KEY_3 },
	{ 0x06, KEY_4 },
	{ 0x07, KEY_5 },
	{ 0x08, KEY_6 },
	{ 0x09, KEY_7 },
	{ 0x0a, KEY_8 },
	{ 0x0b, KEY_9 },
	{ 0x0c, KEY_0 },

	{ 0x19, KEY_TEXT },	/* keypad asterisk as well */
	{ 0x14, KEY_RED },	/* red button */
	{ 0x12, KEY_MENU },	/* The "i" key */
	{ 0x18, KEY_MUTE },
	{ 0x25, KEY_VOLUMEUP },
	{ 0x26, KEY_VOLUMEDOWN },
	{ 0x0d, KEY_UP },
	{ 0x11, KEY_DOWN },
	{ 0x0e, KEY_LEFT },
	{ 0x10, KEY_RIGHT },

	{ 0x22, KEY_EPG },	/* Guide */
	{ 0x1a, KEY_TV },
	{ 0x1e, KEY_NEXTSONG },		/* skip >| */
	{ 0x13, KEY_EXIT },		/* back/exit */
	{ 0x23, KEY_CHANNELUP },	/* channel / program + */
	{ 0x24, KEY_CHANNELDOWN },	/* channel / program - */
	{ 0x0f, KEY_ENTER },		/* OK */
	{ 0x17, KEY_BLUE },		/* blue button */
	{ 0x15, KEY_GREEN },		/* green button */
	{ 0x3e, KEY_PAUSE },		/* pause */
	{ 0x3d, KEY_REWIND },		/* backward << */
	{ 0x3f, KEY_FASTFORWARD },	/* forward >> */
	{ 0x3b, KEY_PLAY },
	{ 0x3c, KEY_STOP },
	{ 0x3a, KEY_RECORD },          /* recording */
	{ 0x16, KEY_YELLOW },          /* yellow key */
	{ 0x01, KEY_POWER },           /* system power */
};

static struct rc_keymap tt_3200_map = {
	.map = {
		.scan    = tt_3200,
		.size    = ARRAY_SIZE(tt_3200),
		.ir_type = IR_TYPE_UNKNOWN,	/* Legacy IR type */
		.name    = RC_MAP_TT_3200,
	}
};

static int __init init_rc_map_tt_3200(void)
{
	return ir_register_map(&tt_3200_map);
}

static void __exit exit_rc_map_tt_3200(void)
{
	ir_unregister_map(&tt_3200_map);
}

module_init(init_rc_map_tt_3200)
module_exit(exit_rc_map_tt_3200)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Igor M. Liplianin <liplianin@me.by>");
