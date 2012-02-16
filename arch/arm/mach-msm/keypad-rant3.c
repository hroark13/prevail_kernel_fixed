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

#ifdef CONFIG_MACH_RANT3
static unsigned int keypad_row_gpios_rant[] = {42,41,40,39,38}; //output //keysense
static unsigned int keypad_col_gpios_rant[] = { 36,35,34,33,32,31,72,73,88,93 }; //input //keyscan

#define KEYMAP_INDEX_RANT(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_rant) + (col))


static unsigned short keypad_keymap_rant[ARRAY_SIZE(keypad_col_gpios_rant) *
						  ARRAY_SIZE(keypad_row_gpios_rant)] = {

		[KEYMAP_INDEX_RANT(0, 0)] = KEY_VOLUMEUP,  
		[KEYMAP_INDEX_RANT(0, 1)] = KEY_MENU, 
		[KEYMAP_INDEX_RANT(0, 2)] = KEY_VOLUMEDOWN, 		 
		[KEYMAP_INDEX_RANT(0, 3)] = KEY_HOME,
		[KEYMAP_INDEX_RANT(0, 4)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT(0, 5)] = KEY_POWER,
		[KEYMAP_INDEX_RANT(0, 6)] = KEY_BACK,
		[KEYMAP_INDEX_RANT(0, 7)] = KEY_SPEAKER, 
		[KEYMAP_INDEX_RANT(0, 8)] = KEY_ZOOM_RANT,
		[KEYMAP_INDEX_RANT(0, 9)] = KEY_CAMERA,				

		[KEYMAP_INDEX_RANT(1, 0)] = KEY_Q,
		[KEYMAP_INDEX_RANT(1, 1)] = KEY_W,
		[KEYMAP_INDEX_RANT(1, 2)] = KEY_E,		
		[KEYMAP_INDEX_RANT(1, 3)] = KEY_R,			
		[KEYMAP_INDEX_RANT(1, 4)] = KEY_T,
		[KEYMAP_INDEX_RANT(1, 5)] = KEY_Y,		 
		[KEYMAP_INDEX_RANT(1, 6)] = KEY_U,		
		[KEYMAP_INDEX_RANT(1, 7)] = KEY_I,
		[KEYMAP_INDEX_RANT(1, 8)] = KEY_O,
		[KEYMAP_INDEX_RANT(1, 9)] = KEY_P,				
		
		[KEYMAP_INDEX_RANT(2, 0)] = KEY_A, 
		[KEYMAP_INDEX_RANT(2, 1)] = KEY_S,
		[KEYMAP_INDEX_RANT(2, 2)] = KEY_D,	
		[KEYMAP_INDEX_RANT(2, 3)] = KEY_F,
		[KEYMAP_INDEX_RANT(2, 4)] = KEY_G, 		 
		[KEYMAP_INDEX_RANT(2, 5)] = KEY_H, 		
		[KEYMAP_INDEX_RANT(2, 6)] = KEY_J,
		[KEYMAP_INDEX_RANT(2, 7)] = KEY_K,	  
		[KEYMAP_INDEX_RANT(2, 8)] = KEY_L,
		[KEYMAP_INDEX_RANT(2, 9)] = KEY_BACKSPACE,				
		
		[KEYMAP_INDEX_RANT(3, 0)] = KEY_FUNCTION,	
		[KEYMAP_INDEX_RANT(3, 1)] = KEY_Z,
		[KEYMAP_INDEX_RANT(3, 2)] = KEY_X, 		 
		[KEYMAP_INDEX_RANT(3, 3)] = KEY_C,	
		[KEYMAP_INDEX_RANT(3, 4)] = KEY_V,
		[KEYMAP_INDEX_RANT(3, 5)] = KEY_B,
		[KEYMAP_INDEX_RANT(3, 6)] = KEY_N,
		[KEYMAP_INDEX_RANT(3, 7)] = KEY_M,
		[KEYMAP_INDEX_RANT(3, 8)] = KEY_UP,
		[KEYMAP_INDEX_RANT(3, 9)] = KEY_ENTER,				
		
		[KEYMAP_INDEX_RANT(4, 0)] = KEY_SHIFT,
		[KEYMAP_INDEX_RANT(4, 1)] = KEY_SYMBOL,
		[KEYMAP_INDEX_RANT(4, 2)] = KEY_EMAIL,//@
		[KEYMAP_INDEX_RANT(4, 3)] = KEY_COMMA,
		[KEYMAP_INDEX_RANT(4, 4)] = KEY_SPACE,
		[KEYMAP_INDEX_RANT(4, 5)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT(4, 6)] = KEY_DOT,//Period
		[KEYMAP_INDEX_RANT(4, 7)] = KEY_LEFT,
		[KEYMAP_INDEX_RANT(4, 8)] = KEY_DOWN,
		[KEYMAP_INDEX_RANT(4, 9)] = KEY_RIGHT,		
};

static unsigned int keypad_row_gpios_rant_rev02[] = {42,41,40,39,38}; //output //keysense
static unsigned int keypad_col_gpios_rant_rev02[] = { 36,35,34,33,32,31,72,73,88,93,122 }; //input //keyscan

#define KEYMAP_INDEX_RANT_REV02(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_rant_rev02) + (col))


static unsigned short keypad_keymap_rant_rev02[ARRAY_SIZE(keypad_col_gpios_rant_rev02) *
						  ARRAY_SIZE(keypad_row_gpios_rant_rev02)] = {

		[KEYMAP_INDEX_RANT_REV02(0, 0)] = KEY_TEXT_RANT,  
		[KEYMAP_INDEX_RANT_REV02(0, 1)] = KEY_MENU, 
		[KEYMAP_INDEX_RANT_REV02(0, 2)] = KEY_TEXT_RANT, 		 
		[KEYMAP_INDEX_RANT_REV02(0, 3)] = KEY_HOME,
		[KEYMAP_INDEX_RANT_REV02(0, 4)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT_REV02(0, 5)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT_REV02(0, 6)] = KEY_BACK,
		[KEYMAP_INDEX_RANT_REV02(0, 7)] = KEY_SPEAKER, 
		[KEYMAP_INDEX_RANT_REV02(0, 8)] = KEY_ZOOM_RANT,
		[KEYMAP_INDEX_RANT_REV02(0, 9)] = KEY_CAMERA,				
		[KEYMAP_INDEX_RANT_REV02(0, 10)] = KEY_POWER,				
		
		[KEYMAP_INDEX_RANT_REV02(1, 0)] = KEY_Q,
		[KEYMAP_INDEX_RANT_REV02(1, 1)] = KEY_W,
		[KEYMAP_INDEX_RANT_REV02(1, 2)] = KEY_E,		
		[KEYMAP_INDEX_RANT_REV02(1, 3)] = KEY_R,			
		[KEYMAP_INDEX_RANT_REV02(1, 4)] = KEY_T,
		[KEYMAP_INDEX_RANT_REV02(1, 5)] = KEY_Y,		 
		[KEYMAP_INDEX_RANT_REV02(1, 6)] = KEY_U,		
		[KEYMAP_INDEX_RANT_REV02(1, 7)] = KEY_I,
		[KEYMAP_INDEX_RANT_REV02(1, 8)] = KEY_O,
		[KEYMAP_INDEX_RANT_REV02(1, 9)] = KEY_P,				
		[KEYMAP_INDEX_RANT_REV02(1, 10)] = KEY_VOLUMEUP,				
				
		[KEYMAP_INDEX_RANT_REV02(2, 0)] = KEY_A, 
		[KEYMAP_INDEX_RANT_REV02(2, 1)] = KEY_S,
		[KEYMAP_INDEX_RANT_REV02(2, 2)] = KEY_D,	
		[KEYMAP_INDEX_RANT_REV02(2, 3)] = KEY_F,
		[KEYMAP_INDEX_RANT_REV02(2, 4)] = KEY_G, 		 
		[KEYMAP_INDEX_RANT_REV02(2, 5)] = KEY_H, 		
		[KEYMAP_INDEX_RANT_REV02(2, 6)] = KEY_J,
		[KEYMAP_INDEX_RANT_REV02(2, 7)] = KEY_K,	  
		[KEYMAP_INDEX_RANT_REV02(2, 8)] = KEY_L,
		[KEYMAP_INDEX_RANT_REV02(2, 9)] = KEY_BACKSPACE,				
		[KEYMAP_INDEX_RANT_REV02(2, 10)] = KEY_VOLUMEDOWN,				
				
		[KEYMAP_INDEX_RANT_REV02(3, 0)] = KEY_FUNCTION,	
		[KEYMAP_INDEX_RANT_REV02(3, 1)] = KEY_Z,
		[KEYMAP_INDEX_RANT_REV02(3, 2)] = KEY_X, 		 
		[KEYMAP_INDEX_RANT_REV02(3, 3)] = KEY_C,	
		[KEYMAP_INDEX_RANT_REV02(3, 4)] = KEY_V,
		[KEYMAP_INDEX_RANT_REV02(3, 5)] = KEY_B,
		[KEYMAP_INDEX_RANT_REV02(3, 6)] = KEY_N,
		[KEYMAP_INDEX_RANT_REV02(3, 7)] = KEY_M,
		[KEYMAP_INDEX_RANT_REV02(3, 8)] = KEY_UP,
		[KEYMAP_INDEX_RANT_REV02(3, 9)] = KEY_ENTER,				
		[KEYMAP_INDEX_RANT_REV02(3, 10)] = KEY_TEXT_RANT,				
				
		[KEYMAP_INDEX_RANT_REV02(4, 0)] = KEY_SHIFT,
		[KEYMAP_INDEX_RANT_REV02(4, 1)] = KEY_SYMBOL,
		[KEYMAP_INDEX_RANT_REV02(4, 2)] = KEY_EMAIL,//@
		[KEYMAP_INDEX_RANT_REV02(4, 3)] = KEY_COMMA,
		[KEYMAP_INDEX_RANT_REV02(4, 4)] = KEY_SPACE,
		[KEYMAP_INDEX_RANT_REV02(4, 5)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT_REV02(4, 6)] = KEY_DOT,//Period
		[KEYMAP_INDEX_RANT_REV02(4, 7)] = KEY_LEFT,
		[KEYMAP_INDEX_RANT_REV02(4, 8)] = KEY_DOWN,
		[KEYMAP_INDEX_RANT_REV02(4, 9)] = KEY_RIGHT,		
		[KEYMAP_INDEX_RANT_REV02(4, 10)] = KEY_TEXT_RANT,								
};

static unsigned int keypad_row_gpios_rant_rev06[] = { 91,35,34,33,32,31,72,73,88,93 }; //keyscan - output
static unsigned int keypad_col_gpios_rant_rev06[] = {42,41,40,39,38,36}; //keysense - input

#define KEYMAP_INDEX_RANT_REV06(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_rant_rev06) + (col))


static unsigned short keypad_keymap_rant_rev06[ARRAY_SIZE(keypad_col_gpios_rant_rev06) *
						  ARRAY_SIZE(keypad_row_gpios_rant_rev06)] = {

		[KEYMAP_INDEX_RANT_REV06(0, 0)] = KEY_FUNCTION,  
		[KEYMAP_INDEX_RANT_REV06(0, 1)] = KEY_Q,		
		[KEYMAP_INDEX_RANT_REV06(0, 2)] = KEY_A, 		
		[KEYMAP_INDEX_RANT_REV06(0, 3)] = KEY_TEXT_RANT,			
		[KEYMAP_INDEX_RANT_REV06(0, 4)] = KEY_TEXT_RANT,		
		[KEYMAP_INDEX_RANT_REV06(0, 5)] = KEY_TEXT_RANT,  	
		
		[KEYMAP_INDEX_RANT_REV06(1, 0)] = KEY_MENU, 
		[KEYMAP_INDEX_RANT_REV06(1, 1)] = KEY_W,
		[KEYMAP_INDEX_RANT_REV06(1, 2)] = KEY_S,		
		[KEYMAP_INDEX_RANT_REV06(1, 3)] = KEY_Z,		
		[KEYMAP_INDEX_RANT_REV06(1, 4)] = KEY_SYMBOL,		
		[KEYMAP_INDEX_RANT_REV06(1, 5)] = KEY_POWER,  			
				
		[KEYMAP_INDEX_RANT_REV06(2, 0)] = KEY_SHIFT, 	
		[KEYMAP_INDEX_RANT_REV06(2, 1)] = KEY_E,			
		[KEYMAP_INDEX_RANT_REV06(2, 2)] = KEY_D,	
		[KEYMAP_INDEX_RANT_REV06(2, 3)] = KEY_X, 			
        	[KEYMAP_INDEX_RANT_REV06(2, 4)] = KEY_EMAIL,//@		
		[KEYMAP_INDEX_RANT_REV06(2, 5)] = KEY_VOLUMEUP,				        	
				
		[KEYMAP_INDEX_RANT_REV06(3, 0)] = KEY_HOME,
		[KEYMAP_INDEX_RANT_REV06(3, 1)] = KEY_R,					
		[KEYMAP_INDEX_RANT_REV06(3, 2)] = KEY_F,		
		[KEYMAP_INDEX_RANT_REV06(3, 3)] = KEY_C,	
		[KEYMAP_INDEX_RANT_REV06(3, 4)] = KEY_COMMA,		
		[KEYMAP_INDEX_RANT_REV06(3, 5)] = KEY_VOLUMEDOWN,						
				
		[KEYMAP_INDEX_RANT_REV06(4, 0)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT_REV06(4, 1)] = KEY_T,		
		[KEYMAP_INDEX_RANT_REV06(4, 2)] = KEY_G, 				
		[KEYMAP_INDEX_RANT_REV06(4, 3)] = KEY_V,		
		[KEYMAP_INDEX_RANT_REV06(4, 4)] = KEY_SPACE,
		[KEYMAP_INDEX_RANT_REV06(4, 5)] = KEY_TEXT_RANT,  			
		
		[KEYMAP_INDEX_RANT_REV06(5, 0)] = KEY_TEXT_RANT,
		[KEYMAP_INDEX_RANT_REV06(5, 1)] = KEY_Y,			
		[KEYMAP_INDEX_RANT_REV06(5, 2)] = KEY_H, 			
		[KEYMAP_INDEX_RANT_REV06(5, 3)] = KEY_B,		
		[KEYMAP_INDEX_RANT_REV06(5, 4)] = KEY_TEXT_RANT,		
		[KEYMAP_INDEX_RANT_REV06(5, 5)] = KEY_TEXT_RANT,  					
		
		[KEYMAP_INDEX_RANT_REV06(6, 0)] = KEY_BACK,
		[KEYMAP_INDEX_RANT_REV06(6, 1)] = KEY_U,			
		[KEYMAP_INDEX_RANT_REV06(6, 2)] = KEY_J,		
		[KEYMAP_INDEX_RANT_REV06(6, 3)] = KEY_N,		
		[KEYMAP_INDEX_RANT_REV06(6, 4)] = KEY_DOT,//Period		
		[KEYMAP_INDEX_RANT_REV06(6, 5)] = KEY_TEXT_RANT,  					
		
		[KEYMAP_INDEX_RANT_REV06(7, 0)] = KEY_SPEAKER, 
		[KEYMAP_INDEX_RANT_REV06(7, 1)] = KEY_I,		
		[KEYMAP_INDEX_RANT_REV06(7, 2)] = KEY_K,	  		
		[KEYMAP_INDEX_RANT_REV06(7, 3)] = KEY_M,		
		[KEYMAP_INDEX_RANT_REV06(7, 4)] = KEY_LEFT,	
		[KEYMAP_INDEX_RANT_REV06(7, 5)] = KEY_TEXT_RANT,  					
		
		[KEYMAP_INDEX_RANT_REV06(8, 0)] = KEY_ZOOM_RANT,
		[KEYMAP_INDEX_RANT_REV06(8, 1)] = KEY_O,		
		[KEYMAP_INDEX_RANT_REV06(8, 2)] = KEY_L,		
		[KEYMAP_INDEX_RANT_REV06(8, 3)] = KEY_UP,		
		[KEYMAP_INDEX_RANT_REV06(8, 4)] = KEY_DOWN,		
		[KEYMAP_INDEX_RANT_REV06(8, 5)] = KEY_TEXT_RANT,  					
		
		[KEYMAP_INDEX_RANT_REV06(9, 0)] = KEY_CAMERA,				
		[KEYMAP_INDEX_RANT_REV06(9, 1)] = KEY_P,				
		[KEYMAP_INDEX_RANT_REV06(9, 2)] = KEY_BACKSPACE,				
		[KEYMAP_INDEX_RANT_REV06(9, 3)] = KEY_ENTER,				
		[KEYMAP_INDEX_RANT_REV06(9, 4)] = KEY_RIGHT,				
		[KEYMAP_INDEX_RANT_REV06(9, 5)] = KEY_TEXT_RANT,  							
};
#elif CONFIG_MACH_REALITY2
static unsigned int keypad_row_gpios_rant[] = {42,41,40,39,38,37/*,122,123*/}; //output //keysense
static unsigned int keypad_col_gpios_rant[] = { 36,35,34,33,32,31,72,73}; //input //keyscan

#define KEYMAP_INDEX_RANT(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios_rant) + (col))


static unsigned short keypad_keymap_rant[ARRAY_SIZE(keypad_col_gpios_rant) *
						  ARRAY_SIZE(keypad_row_gpios_rant)] = {

		[KEYMAP_INDEX_RANT(0, 0)] = KEY_SEND,  
		[KEYMAP_INDEX_RANT(0, 1)] = KEY_CAMERA, 
		[KEYMAP_INDEX_RANT(0, 2)] = KEY_HOME, 		 
		[KEYMAP_INDEX_RANT(0, 3)] = KEY_POWER,//KEY_HOLD,
		[KEYMAP_INDEX_RANT(0, 4)] = KEY_BACK,
		[KEYMAP_INDEX_RANT(0, 5)] = KEY_END,
		[KEYMAP_INDEX_RANT(0, 6)] = KEY_VOLUMEDOWN,
		[KEYMAP_INDEX_RANT(0, 7)] = KEY_VOLUMEUP, 			

		[KEYMAP_INDEX_RANT(1, 0)] = KEY_RIGHT,
		[KEYMAP_INDEX_RANT(1, 1)] = KEY_CAMERA,
		[KEYMAP_INDEX_RANT(1, 2)] = KEY_DOT,		
		[KEYMAP_INDEX_RANT(1, 3)] = KEY_TEXT_RANT, // NOTHING			
		[KEYMAP_INDEX_RANT(1, 4)] = KEY_9,
		[KEYMAP_INDEX_RANT(1, 5)] = KEY_O,		 
		[KEYMAP_INDEX_RANT(1, 6)] = KEY_L,		
		[KEYMAP_INDEX_RANT(1, 7)] = KEY_N,			
		
		[KEYMAP_INDEX_RANT(2, 0)] = KEY_1, 
		[KEYMAP_INDEX_RANT(2, 1)] = KEY_2,
		[KEYMAP_INDEX_RANT(2, 2)] = KEY_3,	
		[KEYMAP_INDEX_RANT(2, 3)] = KEY_4,
		[KEYMAP_INDEX_RANT(2, 4)] = KEY_5, 		 
		[KEYMAP_INDEX_RANT(2, 5)] = KEY_6, 		
		[KEYMAP_INDEX_RANT(2, 6)] = KEY_7,
		[KEYMAP_INDEX_RANT(2, 7)] = KEY_8,	  		
		
		[KEYMAP_INDEX_RANT(3, 0)] = KEY_Q,	
		[KEYMAP_INDEX_RANT(3, 1)] = KEY_W,
		[KEYMAP_INDEX_RANT(3, 2)] = KEY_E, 		 
		[KEYMAP_INDEX_RANT(3, 3)] = KEY_R,	
		[KEYMAP_INDEX_RANT(3, 4)] = KEY_T,
		[KEYMAP_INDEX_RANT(3, 5)] = KEY_Y,
		[KEYMAP_INDEX_RANT(3, 6)] = KEY_U,
		[KEYMAP_INDEX_RANT(3, 7)] = KEY_I,		
		
		[KEYMAP_INDEX_RANT(4, 0)] = KEY_A,
		[KEYMAP_INDEX_RANT(4, 1)] = KEY_S,
		[KEYMAP_INDEX_RANT(4, 2)] = KEY_D,
		[KEYMAP_INDEX_RANT(4, 3)] = KEY_F,
		[KEYMAP_INDEX_RANT(4, 4)] = KEY_G,
		[KEYMAP_INDEX_RANT(4, 5)] = KEY_H,
		[KEYMAP_INDEX_RANT(4, 6)] = KEY_J,
		[KEYMAP_INDEX_RANT(4, 7)] = KEY_K,	

		[KEYMAP_INDEX_RANT(5, 0)] = KEY_Z,
		[KEYMAP_INDEX_RANT(5, 1)] = KEY_X,
		[KEYMAP_INDEX_RANT(5, 2)] = KEY_C,
		[KEYMAP_INDEX_RANT(5, 3)] = KEY_V,
		[KEYMAP_INDEX_RANT(5, 4)] = KEY_SPACE,
		[KEYMAP_INDEX_RANT(5, 5)] = KEY_CLEAR,
		[KEYMAP_INDEX_RANT(5, 6)] = KEY_ENTER,
		[KEYMAP_INDEX_RANT(5, 7)] = KEY_B,		
/*
		[KEYMAP_INDEX_RANT(6, 0)] = KEY_0,
		[KEYMAP_INDEX_RANT(6, 1)] = KEY_P,
		[KEYMAP_INDEX_RANT(6, 2)] = KEY_LEFT,
		[KEYMAP_INDEX_RANT(6, 3)] = KEY_M,
		[KEYMAP_INDEX_RANT(6, 4)] = KEY_SEARCH,
		[KEYMAP_INDEX_RANT(6, 5)] = KEY_UP,
		[KEYMAP_INDEX_RANT(6, 6)] = KEY_OK,
		[KEYMAP_INDEX_RANT(6, 7)] = KEY_DOWN,		

		[KEYMAP_INDEX_RANT(7, 0)] = KEY_MENU,
		[KEYMAP_INDEX_RANT(7, 1)] = KEY_MAIL,
		[KEYMAP_INDEX_RANT(7, 2)] = KEY_FUNCTION,
		[KEYMAP_INDEX_RANT(7, 3)] = KEY_SHIFT,
		[KEYMAP_INDEX_RANT(7, 4)] = KEY_TEXT_RANT, // NOTHING
		[KEYMAP_INDEX_RANT(7, 5)] = KEY_TEXT_RANT, // NOTHING,
		[KEYMAP_INDEX_RANT(7, 6)] = KEY_TEXT_RANT, // NOTHING,
		[KEYMAP_INDEX_RANT(7, 7)] = KEY_TEXT_RANT, // NOTHING,		
*/		
};
#endif 

/* RANT keypad platform device information */
static struct gpio_event_matrix_info rant_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_rant,
	.output_gpios	= keypad_row_gpios_rant,
	.input_gpios	= keypad_col_gpios_rant,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_rant),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_rant),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *rant_keypad_info[] = {
	&rant_keypad_matrix_info.info
};

static struct gpio_event_platform_data rant_keypad_data = {
	.name		= "sec_keypad01",
	.info		= rant_keypad_info,
	.info_count	= ARRAY_SIZE(rant_keypad_info)
};

struct platform_device keypad_device_rant = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &rant_keypad_data,
	},
};

#ifdef CONFIG_MACH_RANT3
// REV02
static struct gpio_event_matrix_info rant_keypad_matrix_info_rev02 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_rant_rev02,
	.output_gpios	= keypad_row_gpios_rant_rev02,
	.input_gpios	= keypad_col_gpios_rant_rev02,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_rant_rev02),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_rant_rev02),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		=GPIOKPF_ACTIVE_HIGH| GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
			  
};

static struct gpio_event_info *rant_keypad_info_rev02[] = {
	&rant_keypad_matrix_info_rev02.info
};

static struct gpio_event_platform_data rant_keypad_data_rev02 = {
	.name		= "sec_keypad01",
	.info		= rant_keypad_info_rev02,
	.info_count	= ARRAY_SIZE(rant_keypad_info_rev02)
};

struct platform_device keypad_device_rant_rev02 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &rant_keypad_data_rev02,
	},
};

// REV06
static struct gpio_event_matrix_info rant_keypad_matrix_info_rev06 = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_rant_rev06,
	.output_gpios	= keypad_row_gpios_rant_rev06,
	.input_gpios	= keypad_col_gpios_rant_rev06,
	.noutputs	= ARRAY_SIZE(keypad_row_gpios_rant_rev06),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios_rant_rev06),
	.settle_time.tv.nsec = 100 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 40 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_DRIVE_INACTIVE|GPIOKPF_PRINT_MAPPED_KEYS|GPIOKPF_REMOVE_PHANTOM_KEYS
};

static struct gpio_event_info *rant_keypad_info_rev06[] = {
	&rant_keypad_matrix_info_rev06.info
};

static struct gpio_event_platform_data rant_keypad_data_rev06 = {
	.name		= "sec_keypad",
	.info		= rant_keypad_info_rev06,
	.info_count	= ARRAY_SIZE(rant_keypad_info_rev06)
};

struct platform_device keypad_device_rant_rev06 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &rant_keypad_data_rev06,
	},
};
#endif


