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

static unsigned int keypad_row_gpios_escape[] = {72, 73, 92, 40, 21, 26, 90, 94, 96, 106, 108, 123};  //keyscan
static unsigned int keypad_col_gpios_escape[] = {33, 74, 3, 122, 36, 91};  //keysense

static unsigned int keypad_row_gpios_escape_rev03[] = {72, 73, 3, 33, 74, 26, 90, 122, 96, 106, 108, 123};  //keyscan
static unsigned int keypad_col_gpios_escape_rev03[] = {21, 36, 40, 92, 78, 91};  //keysense

static unsigned int keypad_row_gpios_escape_rev04[] = {72, 73, 3, 33, 74, 26, 90, 122, 96, 81, 108, 123};  //keyscan
static unsigned int keypad_col_gpios_escape_rev04[] = {21, 36, 40, 92, 78, 91};  //keysense

#define KEYMAP_INDEX_ESCAPE(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_escape) + (col))

static unsigned short keypad_keymap_escape[ARRAY_SIZE(keypad_col_gpios_escape) *
						  ARRAY_SIZE(keypad_row_gpios_escape)] = {
		[KEYMAP_INDEX_ESCAPE(0, 0)] = KEY_FUNCTION,  
		[KEYMAP_INDEX_ESCAPE(0, 1)] = KEY_SHIFT, 
		[KEYMAP_INDEX_ESCAPE(0, 2)] = KEY_DOT,  
		[KEYMAP_INDEX_ESCAPE(0, 3)] = KEY_Z, 
		[KEYMAP_INDEX_ESCAPE(0, 4)] = KEY_VOLUMEUP,  
		[KEYMAP_INDEX_ESCAPE(0, 5)] = KEY_VOLUMEDOWN, 

		[KEYMAP_INDEX_ESCAPE(1, 0)] = KEY_1,  
		[KEYMAP_INDEX_ESCAPE(1, 1)] = KEY_Q, 
		[KEYMAP_INDEX_ESCAPE(1, 2)] = KEY_A,
		[KEYMAP_INDEX_ESCAPE(1, 3)] = KEY_X,
		[KEYMAP_INDEX_ESCAPE(1, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(1, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(2, 0)] = KEY_2,
		[KEYMAP_INDEX_ESCAPE(2, 1)] = KEY_W,
		[KEYMAP_INDEX_ESCAPE(2, 2)] = KEY_S,
		[KEYMAP_INDEX_ESCAPE(2, 3)] = KEY_C, 
		[KEYMAP_INDEX_ESCAPE(2, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(2, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(3, 0)] = KEY_3,
		[KEYMAP_INDEX_ESCAPE(3, 1)] = KEY_E,
		[KEYMAP_INDEX_ESCAPE(3, 2)] = KEY_D,
		[KEYMAP_INDEX_ESCAPE(3, 3)] = KEY_V,
		[KEYMAP_INDEX_ESCAPE(3, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(3, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(4, 0)] = KEY_4,
		[KEYMAP_INDEX_ESCAPE(4, 1)] = KEY_R,
		[KEYMAP_INDEX_ESCAPE(4, 2)] = KEY_F,
		[KEYMAP_INDEX_ESCAPE(4, 3)] = KEY_B,
		[KEYMAP_INDEX_ESCAPE(4, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(4, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(5, 0)] = KEY_5,
		[KEYMAP_INDEX_ESCAPE(5, 1)] = KEY_T,
		[KEYMAP_INDEX_ESCAPE(5, 2)] = KEY_G, 
		[KEYMAP_INDEX_ESCAPE(5, 3)] = KEY_SPACE,
		[KEYMAP_INDEX_ESCAPE(5, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(5, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(6, 0)] = KEY_6,
		[KEYMAP_INDEX_ESCAPE(6, 1)] = KEY_Y,
		[KEYMAP_INDEX_ESCAPE(6, 2)] = KEY_H, 
		[KEYMAP_INDEX_ESCAPE(6, 3)] = KEY_POWER,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(6, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(6, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(7, 0)] = KEY_7,
		[KEYMAP_INDEX_ESCAPE(7, 1)] = KEY_U,
		[KEYMAP_INDEX_ESCAPE(7, 2)] = KEY_J, 
		[KEYMAP_INDEX_ESCAPE(7, 3)] = KEY_N, 
		[KEYMAP_INDEX_ESCAPE(7, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(7, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(8, 0)] = KEY_8,
		[KEYMAP_INDEX_ESCAPE(8, 1)] = KEY_I,
		[KEYMAP_INDEX_ESCAPE(8, 2)] = KEY_K, 
		[KEYMAP_INDEX_ESCAPE(8, 3)] = KEY_M, 
		[KEYMAP_INDEX_ESCAPE(8, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(8, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(9, 0)] = KEY_9,
		[KEYMAP_INDEX_ESCAPE(9, 1)] = KEY_O,
		[KEYMAP_INDEX_ESCAPE(9, 2)] = KEY_L, 
		[KEYMAP_INDEX_ESCAPE(9, 3)] = KEY_UP, //KEY_LEFT, 
		[KEYMAP_INDEX_ESCAPE(9, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(9, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(10, 0)] = KEY_0,
		[KEYMAP_INDEX_ESCAPE(10, 1)] = KEY_P,
		[KEYMAP_INDEX_ESCAPE(10, 2)] = KEY_RIGHT, //KEY_UP,	
		[KEYMAP_INDEX_ESCAPE(10, 3)] = KEY_LEFT, //KEY_DOWN,	
		[KEYMAP_INDEX_ESCAPE(10, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(10, 5)] = KEY_TEXT_RANT,  // NOTHING	

		[KEYMAP_INDEX_ESCAPE(11, 0)] = KEY_BACKSPACE,
		[KEYMAP_INDEX_ESCAPE(11, 1)] = KEY_ENTER,
		[KEYMAP_INDEX_ESCAPE(11, 2)] = KEY_OK,	
		[KEYMAP_INDEX_ESCAPE(11, 3)] = KEY_DOWN, //KEY_RIGHT,	
		[KEYMAP_INDEX_ESCAPE(11, 4)] = KEY_TEXT_RANT,  // NOTHING	
		[KEYMAP_INDEX_ESCAPE(11, 5)] = KEY_TEXT_RANT,  // NOTHING	
};

/* ESCAPE keypad platform device information */
static struct gpio_event_matrix_info escape_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_escape,
	.output_gpios	= keypad_row_gpios_escape,
	.input_gpios	= keypad_col_gpios_escape,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_escape),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_escape),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *escape_keypad_info[] = {
	&escape_keypad_matrix_info.info
};

static struct gpio_event_platform_data escape_keypad_data = {
	.name		= "sec_keypad",
	.info		= escape_keypad_info,
	.info_count	= ARRAY_SIZE(escape_keypad_info)
};

struct platform_device keypad_device_escape = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &escape_keypad_data,
	},
};

/* ESCAPE keypad platform device information */
static struct gpio_event_matrix_info escape_keypad_matrix_info_rev03 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_escape,
	.output_gpios	= keypad_row_gpios_escape_rev03,
	.input_gpios	= keypad_col_gpios_escape_rev03,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_escape_rev03),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_escape_rev03),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *escape_keypad_info_rev03[] = {
	&escape_keypad_matrix_info_rev03.info
};

static struct gpio_event_platform_data escape_keypad_data_rev03 = {
	.name		= "sec_keypad01",
	.info		= escape_keypad_info_rev03,
	.info_count	= ARRAY_SIZE(escape_keypad_info_rev03)
};

struct platform_device keypad_device_escape_rev03 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &escape_keypad_data_rev03,
	},
};

/* ESCAPE keypad platform device information */
static struct gpio_event_matrix_info escape_keypad_matrix_info_rev04 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_escape,
	.output_gpios	= keypad_row_gpios_escape_rev04,
	.input_gpios	= keypad_col_gpios_escape_rev04,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_escape_rev04),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_escape_rev04),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *escape_keypad_info_rev04[] = {
	&escape_keypad_matrix_info_rev04.info
};

static struct gpio_event_platform_data escape_keypad_data_rev04 = {
	.name		= "sec_keypad01",
	.info		= escape_keypad_info_rev04,
	.info_count	= ARRAY_SIZE(escape_keypad_info_rev04)
};

struct platform_device keypad_device_escape_rev04 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &escape_keypad_data_rev04,
	},
};

