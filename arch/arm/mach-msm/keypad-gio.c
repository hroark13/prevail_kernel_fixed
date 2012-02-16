/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

static unsigned int keypad_row_gpios_gio[] = {72, 73, 92, 40};  //keyscan
static unsigned int keypad_col_gpios_gio[] = {33, 74, 91, 36};  //keysense

static unsigned int keypad_row_gpios_gio_rev03[] = {72, 73, 74, 40};  //keyscan
static unsigned int keypad_col_gpios_gio_rev03[] = {33, 92, 91, 36};  //keysense

#define KEYMAP_INDEX_GIO(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_gio) + (col))

static unsigned short keypad_keymap_gio[ARRAY_SIZE(keypad_col_gpios_gio) *
						  ARRAY_SIZE(keypad_row_gpios_gio)] = {

		[KEYMAP_INDEX_GIO(0, 0)] = KEY_RESERVED,  
		[KEYMAP_INDEX_GIO(0, 1)] = KEY_HOME, 
		[KEYMAP_INDEX_GIO(0, 2)] = KEY_VOLUMEDOWN,  
		[KEYMAP_INDEX_GIO(0, 3)] = KEY_VOLUMEUP,

		[KEYMAP_INDEX_GIO(1, 0)] = KEY_RESERVED,  
		[KEYMAP_INDEX_GIO(1, 1)] = KEY_RESERVED, 
		[KEYMAP_INDEX_GIO(1, 2)] = KEY_RESERVED,  
		[KEYMAP_INDEX_GIO(1, 3)] = KEY_RESERVED, 

		[KEYMAP_INDEX_GIO(2, 0)] = KEY_RESERVED,
		[KEYMAP_INDEX_GIO(2, 1)] = KEY_RESERVED,
		[KEYMAP_INDEX_GIO(2, 2)] = KEY_RESERVED,  
		[KEYMAP_INDEX_GIO(2, 3)] = KEY_RESERVED, 

		[KEYMAP_INDEX_GIO(3, 0)] = KEY_POWER,
		[KEYMAP_INDEX_GIO(3, 1)] = KEY_RESERVED,
		[KEYMAP_INDEX_GIO(3, 2)] = KEY_RESERVED,  
		[KEYMAP_INDEX_GIO(3, 3)] = KEY_RESERVED, 
};

static struct gpio_event_matrix_info gio_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap 	= keypad_keymap_gio,
	.output_gpios	= keypad_row_gpios_gio,
	.input_gpios	= keypad_col_gpios_gio,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gio),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gio),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_matrix_info gio_keypad_matrix_info_rev03 = {
	.info.func	= gpio_event_matrix_func,
	.keymap 	= keypad_keymap_gio,
	.output_gpios	= keypad_row_gpios_gio_rev03,
	.input_gpios	= keypad_col_gpios_gio_rev03,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gio_rev03),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gio_rev03),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS		  
};

static struct gpio_event_info *gio_keypad_info[] = {
	&gio_keypad_matrix_info.info
};

static struct gpio_event_info *gio_keypad_info_rev03[] = {
	&gio_keypad_matrix_info_rev03.info
};

static struct gpio_event_platform_data gio_keypad_data = {
	.name		= "sec_key",
	.info		= gio_keypad_info,
	.info_count = ARRAY_SIZE(gio_keypad_info)
};

static struct gpio_event_platform_data gio_keypad_data_rev03 = {
	.name		= "sec_key",
	.info		= gio_keypad_info_rev03,
	.info_count = ARRAY_SIZE(gio_keypad_info_rev03)
};

struct platform_device keypad_device_gio = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id = -1,
	.dev	= {
		.platform_data	= &gio_keypad_data,
	},
};

struct platform_device keypad_device_gio_rev03 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id = -1,
	.dev	= {
		.platform_data	= &gio_keypad_data_rev03,
	},
};

