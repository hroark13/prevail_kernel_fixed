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

static unsigned int keypad_row_gpios_rookie[] = {72, 73, 92, 40};  //keyscan
static unsigned int keypad_col_gpios_rookie[] = {33, 74, 91, 36};  //keysense

#define KEYMAP_INDEX_ROOKIE(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_rookie) + (col))


static unsigned short keypad_keymap_rookie[ARRAY_SIZE(keypad_col_gpios_rookie) *
						  ARRAY_SIZE(keypad_row_gpios_rookie)] = {

		[KEYMAP_INDEX_ROOKIE(0, 0)] = KEY_SEARCH,  
		[KEYMAP_INDEX_ROOKIE(0, 1)] = KEY_BACK, 
		[KEYMAP_INDEX_ROOKIE(0, 2)] = KEY_VOLUMEUP,  
		[KEYMAP_INDEX_ROOKIE(0, 3)] = KEY_VOLUMEDOWN, 
		
		[KEYMAP_INDEX_ROOKIE(1, 0)] = KEY_HOME,  
		[KEYMAP_INDEX_ROOKIE(1, 1)] = KEY_MENU, 
		[KEYMAP_INDEX_ROOKIE(1, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(1, 3)] = KEY_TEXT_RANT, 
		
		[KEYMAP_INDEX_ROOKIE(2, 0)] = KEY_CAMERA_SNAPSHOT,
		[KEYMAP_INDEX_ROOKIE(2, 1)] = KEY_CAMERA_FOCUS,
		[KEYMAP_INDEX_ROOKIE(2, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(2, 3)] = KEY_TEXT_RANT, 
		
		[KEYMAP_INDEX_ROOKIE(3, 0)] = KEY_POWER,
		[KEYMAP_INDEX_ROOKIE(3, 1)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_ROOKIE(3, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(3, 3)] = KEY_TEXT_RANT, 
};

static unsigned short keypad_keymap_rookie_rev03[ARRAY_SIZE(keypad_col_gpios_rookie) *
						  ARRAY_SIZE(keypad_row_gpios_rookie)] = {

		[KEYMAP_INDEX_ROOKIE(0, 0)] = KEY_SEARCH,  
		[KEYMAP_INDEX_ROOKIE(0, 1)] = KEY_BACK, 
		[KEYMAP_INDEX_ROOKIE(0, 2)] = KEY_VOLUMEDOWN,  
		[KEYMAP_INDEX_ROOKIE(0, 3)] = KEY_VOLUMEUP,
		
		[KEYMAP_INDEX_ROOKIE(1, 0)] = KEY_HOME,  
		[KEYMAP_INDEX_ROOKIE(1, 1)] = KEY_MENU, 
		[KEYMAP_INDEX_ROOKIE(1, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(1, 3)] = KEY_TEXT_RANT, 
		
		[KEYMAP_INDEX_ROOKIE(2, 0)] = KEY_CAMERA_SNAPSHOT,
		[KEYMAP_INDEX_ROOKIE(2, 1)] = KEY_CAMERA_FOCUS,
		[KEYMAP_INDEX_ROOKIE(2, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(2, 3)] = KEY_TEXT_RANT, 
		
		[KEYMAP_INDEX_ROOKIE(3, 0)] = KEY_POWER,
		[KEYMAP_INDEX_ROOKIE(3, 1)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_ROOKIE(3, 2)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_ROOKIE(3, 3)] = KEY_TEXT_RANT, 
};


/* ROOKIE keypad platform device information */
static struct gpio_event_matrix_info rookie_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_rookie,
	.output_gpios	= keypad_row_gpios_rookie,
	.input_gpios	= keypad_col_gpios_rookie,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_rookie),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_rookie),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_matrix_info rookie_keypad_matrix_info_rev03 = {
	.info.func	= gpio_event_matrix_func,
	.keymap 	= keypad_keymap_rookie_rev03,
	.output_gpios	= keypad_row_gpios_rookie,
	.input_gpios	= keypad_col_gpios_rookie,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_rookie),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_rookie),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *rookie_keypad_info[] = {
	&rookie_keypad_matrix_info.info
};

static struct gpio_event_info *rookie_keypad_info_rev03[] = {
	&rookie_keypad_matrix_info_rev03.info
};


static struct gpio_event_platform_data rookie_keypad_data = {
	.name		= "sec_key",
	.info		= rookie_keypad_info,
	.info_count	= ARRAY_SIZE(rookie_keypad_info)
};
static struct gpio_event_platform_data rookie_keypad_data_rev03 = {
	.name		= "sec_key",
	.info		= rookie_keypad_info_rev03,
	.info_count = ARRAY_SIZE(rookie_keypad_info_rev03)
};

struct platform_device keypad_device_rookie = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &rookie_keypad_data,
	},
};

struct platform_device keypad_device_rookie_rev03 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id = -1,
	.dev	= {
		.platform_data	= &rookie_keypad_data_rev03,
	},
};

