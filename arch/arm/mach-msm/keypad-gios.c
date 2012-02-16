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

// REV09
static unsigned int keypad_row_gpios_gios_rev_09[] = {93, 122};  //SCAN 9(93),SCAN 10(122)
static unsigned int keypad_col_gpios_gios_rev_09[] = {42,41,40,39};      //SENSE 0 (42), SENSE 1(41), SENSE 2 (40),SENSE 3(39)
		
#define KEYMAP_INDEX_GIOS_REV_09(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_gios_rev_09) + (col))
		
static const unsigned short keypad_keymap_gios_rev_09[ARRAY_SIZE(keypad_col_gpios_gios_rev_09) * ARRAY_SIZE(keypad_row_gpios_gios_rev_09)] = {
  [KEYMAP_INDEX_GIOS_REV_09(0, 0)] = 0,
  [KEYMAP_INDEX_GIOS_REV_09(0, 1)] = 0,
  [KEYMAP_INDEX_GIOS_REV_09(0, 2)] = 0,
  [KEYMAP_INDEX_GIOS_REV_09(0, 3)] = KEY_CAMERA,

  [KEYMAP_INDEX_GIOS_REV_09(1, 0)] = KEY_POWER,
  [KEYMAP_INDEX_GIOS_REV_09(1, 1)] = KEY_VOLUMEUP,
  [KEYMAP_INDEX_GIOS_REV_09(1, 2)] = KEY_VOLUMEDOWN,
  [KEYMAP_INDEX_GIOS_REV_09(1, 3)] = 0
};

/* GIOS keypad platform device information */
static struct gpio_event_matrix_info gios_keypad_matrix_info_rev_09 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_gios_rev_09,
	.output_gpios	= keypad_row_gpios_gios_rev_09,
	.input_gpios	= keypad_col_gpios_gios_rev_09,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gios_rev_09),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gios_rev_09),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	//.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
//  .flags    =GPIOKPF_LEVEL_TRIGGERED_IRQ| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_UNMAPPED_KEYS
	.flags		= GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS

};

static struct gpio_event_info *gios_keypad_info_rev_09[] = {
	&gios_keypad_matrix_info_rev_09.info
};

static struct gpio_event_platform_data gios_keypad_data_rev_09 = {
	.name		= "gios-keypad",
	.info		= gios_keypad_info_rev_09,
	.info_count	= ARRAY_SIZE(gios_keypad_info_rev_09)
};

struct platform_device keypad_device_gios_rev_09 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &gios_keypad_data_rev_09,
	},
};

// REV03
static unsigned int keypad_row_gpios_gios_rev_03[] = {93, 122};  //SCAN 9(93),SCAN 10(122)
static unsigned int keypad_col_gpios_gios_rev_03[] = {42,41,40};      //SENSE 0 (42), SENSE 1(41), SENSE 2 (40)
		
#define KEYMAP_INDEX_GIOS_REV_03(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_gios_rev_03) + (col))
		
static const unsigned short keypad_keymap_gios_rev_03[ARRAY_SIZE(keypad_col_gpios_gios_rev_03) * ARRAY_SIZE(keypad_row_gpios_gios_rev_03)] = {
  [KEYMAP_INDEX_GIOS_REV_03(0, 0)] = KEY_CAMERA,
  [KEYMAP_INDEX_GIOS_REV_03(0, 1)] = 0,
  [KEYMAP_INDEX_GIOS_REV_03(0, 2)] = 0,

  [KEYMAP_INDEX_GIOS_REV_03(1, 0)] = KEY_POWER,
  [KEYMAP_INDEX_GIOS_REV_03(1, 1)] = KEY_VOLUMEUP,
  [KEYMAP_INDEX_GIOS_REV_03(1, 2)] = KEY_VOLUMEDOWN
};

/* GIOS keypad platform device information */
static struct gpio_event_matrix_info gios_keypad_matrix_info_rev_03 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_gios_rev_03,
	.output_gpios	= keypad_row_gpios_gios_rev_03,
	.input_gpios	= keypad_col_gpios_gios_rev_03,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gios_rev_03),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gios_rev_03),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	//.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
//  .flags    =GPIOKPF_LEVEL_TRIGGERED_IRQ| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_UNMAPPED_KEYS
	.flags		= GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS

};

static struct gpio_event_info *gios_keypad_info_rev_03[] = {
	&gios_keypad_matrix_info_rev_03.info
};

static struct gpio_event_platform_data gios_keypad_data_rev_03 = {
	.name		= "gios-keypad",
	.info		= gios_keypad_info_rev_03,
	.info_count	= ARRAY_SIZE(gios_keypad_info_rev_03)
};

struct platform_device keypad_device_gios_rev_03 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &gios_keypad_data_rev_03,
	},
};
// REV02 
// REV01 =================================================================================
static unsigned int keypad_row_gpios_gios[] = {36,34,31,93};  //SCAN 0(36),SCAN 2(34),SCAN 5(31),SCAN 9(93)
static unsigned int keypad_col_gpios_gios[] = {42} ;      //SENSE 0 (42)
		
		
#define KEYMAP_INDEX_GIOS(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_gios) + (col))
		
static const unsigned short keypad_keymap_gios[ARRAY_SIZE(keypad_col_gpios_gios) * ARRAY_SIZE(keypad_row_gpios_gios)] = {
  [KEYMAP_INDEX_GIOS(0, 0)] = KEY_VOLUMEUP,
  [KEYMAP_INDEX_GIOS(1, 0)] = KEY_VOLUMEDOWN,
  [KEYMAP_INDEX_GIOS(2, 0)] = KEY_POWER,
  [KEYMAP_INDEX_GIOS(3, 0)] = KEY_CAMERA
};


/* GIOS keypad platform device information */
static struct gpio_event_matrix_info gios_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_gios,
	.output_gpios	= keypad_row_gpios_gios,
	.input_gpios	= keypad_col_gpios_gios,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gios),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	//.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
  .flags    =GPIOKPF_LEVEL_TRIGGERED_IRQ| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *gios_keypad_info[] = {
	&gios_keypad_matrix_info.info
};

static struct gpio_event_platform_data gios_keypad_data = {
	.name		= "gios-keypad",
	.info		= gios_keypad_info,
	.info_count	= ARRAY_SIZE(gios_keypad_info)
};

struct platform_device keypad_device_gios = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &gios_keypad_data,
	},
};


// REV00 =================================================================================
static unsigned int keypad_row_gpios_gios_rev0[] = {41,38,37} ;      //SENSE 1 (41),SENSE 4(38),SENSE 5(37),
static unsigned int keypad_col_gpios_gios_rev0[] = {36,35,73};  //SCAN 0,1,7

#define KEYMAP_INDEX_REV0(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_gios_rev0) + (col))

static const unsigned short keypad_keymap_gios_rev0[ARRAY_SIZE(keypad_col_gpios_gios_rev0) * ARRAY_SIZE(keypad_row_gpios_gios_rev0)] = {
  [KEYMAP_INDEX_REV0(0, 0)] = 0,
  [KEYMAP_INDEX_REV0(0, 1)] = KEY_CAMERA,
  [KEYMAP_INDEX_REV0(0, 2)] = 0,

  [KEYMAP_INDEX_REV0(1, 0)] = KEY_VOLUMEUP,
  [KEYMAP_INDEX_REV0(1, 1)] = 0,
  [KEYMAP_INDEX_REV0(1, 2)] = 0,


  [KEYMAP_INDEX_REV0(2, 0)] = KEY_VOLUMEDOWN,
  [KEYMAP_INDEX_REV0(2, 1)] = 0,
  [KEYMAP_INDEX_REV0(2, 2)] = KEY_POWER
};

/* GIOS keypad platform device information */
static struct gpio_event_matrix_info gios_rev0_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_gios_rev0,
	.output_gpios	= keypad_row_gpios_gios_rev0,
	.input_gpios	= keypad_col_gpios_gios_rev0,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_gios_rev0),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_gios_rev0),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *gios_rev0_keypad_info[] = {
	&gios_rev0_keypad_matrix_info.info
};

static struct gpio_event_platform_data gios_rev0_keypad_data = {
	.name		= "gios-keypad",
	.info		= gios_rev0_keypad_info,
	.info_count	= ARRAY_SIZE(gios_rev0_keypad_info)
};

struct platform_device keypad_device_gios_rev0 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &gios_rev0_keypad_data,
	},
};
