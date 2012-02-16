/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <linux/spinlock.h>
#include <mach/gpio.h>
#include "msm_fb.h"

// quattro_jiny46kim
#include <mach/vreg.h>

#include <lcdc_s6d_backlight.h>

#define LCDC_DEBUG

extern unsigned char hw_version;

//#define FEATURE_LCD_ESD_DET 

#ifdef LCDC_DEBUG
#define DPRINT(x...)    printk("s6d05a1 " x)
#else
#define DPRINT(x...)    
#endif

#define __LCD_CONTROL_BY_FILE__    	//MBjclee 2011.1.24    LCD controled by FILE

#if defined(CONFIG_MACH_ESCAPE)
#define GPIO_BL_CTRL  27
#elif defined(CONFIG_MACH_GIO)
#define GPIO_BL_CTRL  26
#endif

#define BACKLIGHT_LEVEL_VALUE   0xff

#ifdef FEATURE_LCD_ESD_DET
#if defined(CONFIG_MACH_ESCAPE)
#define GPIO_ESD_DET    97
#else
#define GPIO_ESD_DET    94
#endif
#endif

static boolean bdisplay_init = FALSE;

#define RDID1			0xDA		// 2 parameters
#define RDID2			0xDB		// 2 parameters
#define RDID3			0xDC		// 2 parameters

/*
 * Serial Interface
 */
#define LCD_CSX_HIGH    gpio_set_value(spi_cs, 1);
#define LCD_CSX_LOW     gpio_set_value(spi_cs, 0);

#define LCD_SCL_HIGH    gpio_set_value(spi_sclk, 1);
#define LCD_SCL_LOW     gpio_set_value(spi_sclk, 0);

#define LCD_SDI_HIGH    gpio_set_value(spi_sdi, 1);
#define LCD_SDI_LOW     gpio_set_value(spi_sdi, 0);

#define LCD_SDI_DATA	gpio_get_value(spi_sdi)

#define LCD_SDO_DATA	gpio_get_value(spi_sdo)
#define LCD_SDO_SWITCH_INPUT		gpio_direction_input(spi_sdo);

#define LCD_SDI_SWITCH_INPUT		gpio_direction_input(spi_sdi);
#define LCD_SDI_SWITCH_OUTPUT_LOW	gpio_direction_output(spi_sdi, 0);
#define LCD_SDI_SWITCH_OUTPUT_HIGH	gpio_direction_output(spi_sdi, 1);

#define DEFAULT_USLEEP  1  
#define DEFAUTL_NSLEEP 110
#define PWRCTL          0xF4
#define SLPIN           0x10
#define SLPOUT          0x11
#define DISCTL          0xF2
#define VCMCTL          0xF5
#define SRCCTL          0xF6
#define COLMOD          0x3A
#define WRDISBV         0x51
#define PASSWD1         0xF0
#define PASSWD2         0xF1
#define IFCTL           0xF7
#define PANELCTL        0xF8
#define GAMMASEL        0xF9
#define PGAMMACTL       0xFA
#define NGAMMACTL       0xFB
#define DISPON          0x29
#define DISPOFF         0x28
#define BCMODE          0xC1
#define WRCTRLD         0x53
#define WRBLCTL         0xC3
#define MADCTL          0x36
#define CASET           0x2A
#define PASET           0x2B
#define TEON             0x35

struct setting_table {
    unsigned char command;  
    unsigned char parameters;
    unsigned char parameter[20];
    long wait;
};

static int lcd_type = 0;
enum {
	LcdTypeNONE = -1,
	LcdTypeAUO = 1,
	LcdTypeSHARP = 2,
};

static struct setting_table power_on_setting_table_sharp[] = {
   { PASSWD1,	 2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { PASSWD2,	 2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },	
   { DISCTL,    19,  { 0x3B, 0x34, 0x03, 0x0A, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x34, 0x0A, 0x08, 0x0A, 0x08, 0x00 },     0 },
   { PWRCTL,	14,  { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x7F, 0x04, 0x0D, 0x7F, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { VCMCTL,     9,  { 0x00, 0x68, 0x70, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },

   { COLMOD,     1,  { 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { MADCTL,     1,  { 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { CASET,      4,  { 0x00, 0x00, 0x01, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PASET,      4,  { 0x00, 0x00, 0x01, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },

   { SRCCTL,	 6,  { 0x03, 0x00, 0x08, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { IFCTL, 	 5,  { 0x48, 0x01, 0xF0, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { PANELCTL,	 2,  { 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },

   { GAMMASEL,   1,  { 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x00, 0x10, 0x0A, 0x25, 0x29, 0x33, 0x37, 0x0E, 0x1C, 0x22, 0x28, 0x1A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { NGAMMACTL, 16,  { 0x0E, 0x00, 0x02, 0x1A, 0x28, 0x22, 0x1C, 0x0E, 0x37, 0x33, 0x29, 0x25, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { GAMMASEL,   1,  { 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x11, 0x19, 0x09, 0x2A, 0x2F, 0x36, 0x39, 0x0C, 0x1A, 0x1E, 0x25, 0x1E, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { NGAMMACTL, 16,  { 0x17, 0x11, 0x02, 0x1E, 0x25, 0x1E, 0x1A, 0x0C, 0x39, 0x36, 0x2F, 0x2A, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },
   { GAMMASEL,   1,  { 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x36, 0x16, 0x05, 0x0E, 0x10, 0x10, 0x13, 0x32, 0x3C, 0x3B, 0x35, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { NGAMMACTL, 16,  { 0x14, 0x36, 0x02, 0x10, 0x35, 0x3B, 0x3C, 0x32, 0x13, 0x10, 0x10, 0x0E, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	 0 },

   { SLPOUT,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   120 },
   { DISPON,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    40 },

   { WRCTRLD,    1,  { 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { WRBLCTL,    3,  { 0x00, 0x2F, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },   
};


//2.8
static struct setting_table power_on_setting_table_auo[] = {
   { PASSWD1,    2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
   { PASSWD2,    2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },   
   { DISCTL,    19,  { 0x3B, 0x3A, 0x03, 0x04, 0x02, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x54, 0x08, 0x08, 0x08, 0x08, 0x00 },     0 },
   { PWRCTL,    11,  { 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x6B, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
   { VCMCTL,     9,  { 0x00, 0x47, 0x75, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },

   { COLMOD,     1,  { 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
   { MADCTL,     1,  { 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
   { CASET,     4,  { 0x00, 0x00, 0x01, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
   { PASET,     4,  { 0x00, 0x00, 0x01, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },

   { SRCCTL,     6,  { 0x03, 0x00, 0x0B, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { IFCTL,      5,  { 0x48, 0x01, 0xF0, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PANELCTL,   2,  { 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },

   { GAMMASEL,   1,  { 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x23, 0x00, 0x09, 0x1D, 0x1D, 0x25, 0x2B, 0x18, 0x21, 0x25, 0x26, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { GAMMASEL,   1,  { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x30, 0x10, 0x08, 0x1D, 0x20, 0x2A, 0x31, 0x10, 0x1B, 0x1A, 0x1F, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { GAMMASEL,   1,  { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { PGAMMACTL, 16,  { 0x30, 0x10, 0x09, 0x1E, 0x27, 0x32, 0x39, 0x0A, 0x16, 0x17, 0x19, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },

   { SLPOUT,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  120 },
   { DISPON,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },

   { WRCTRLD,    1,  { 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
   { WRBLCTL,    3,  { 0x00, 0x2F, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },   
};

#define POWER_ON_SETTINGS(a)	(int)(sizeof(a)/sizeof(struct setting_table))

static struct setting_table power_off_setting_table[] = {
   { DISPOFF,      0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  40 },
   { SLPIN,   0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

static struct setting_table power_on_setting_table_sleep_out[] = {
//	Sleep out
   {SLPOUT, 0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 10 },
};

#define POWER_OFF_SETTINGS  (int)(sizeof(power_off_setting_table)/sizeof(struct setting_table))

static struct setting_table backlight_setting_table =
   { WRDISBV,    1,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 };

static int lcdc_s6d05a1_panel_off(struct platform_device *pdev);
#ifdef FEATURE_LCD_ESD_DET
static irqreturn_t s6d05a1_esd_irq_handler(int irq, void *handle);
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdo;
static int spi_sdi;
//static int spi_dac;
static int lcd_en;
static int lcd_reset;

static char lcd_brightness = 0;

/* To synchronize between gp2a and platform to control backlight */
spinlock_t lcd_lock;
/* to store platform bl level */
int app_bl_level;

#if 0
static unsigned char bit_shift[8] = { (1 << 7), /* MSB */
    (1 << 6),
    (1 << 5),
    (1 << 4),
    (1 << 3),
    (1 << 2),
    (1 << 1),
    (1 << 0)                       /* LSB */
};
#endif

// quattro_jiny46kim
struct vreg *vreg_ldo14;
struct vreg *vreg_ldo15;


struct s6d05a1_state_type{
    boolean disp_initialized;
    boolean display_on;
    boolean disp_powered_up;
    boolean panel_initialized;
#ifdef FEATURE_LCD_ESD_DET
    boolean irq_disabled;
#endif
};


//static int backlight_level;

static struct s6d05a1_state_type s6d05a1_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6d05a1_pdata;

static void read_ldi_register(u8 addr, u8 *buf, int count)
{
	long i, j;

	LCD_CSX_HIGH
	udelay(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	udelay(DEFAULT_USLEEP);

	/* Write Command */
	LCD_CSX_LOW
	udelay(DEFAULT_USLEEP);
	LCD_SCL_LOW 
	udelay(DEFAULT_USLEEP);		
	LCD_SDI_LOW 
	udelay(DEFAULT_USLEEP);
	
	LCD_SCL_HIGH 
	udelay(DEFAULT_USLEEP); 

   	for (i = 7; i >= 0; i--) { 
		LCD_SCL_LOW
		udelay(DEFAULT_USLEEP);
		if ((addr >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_USLEEP);	
		LCD_SCL_HIGH
		udelay(DEFAULT_USLEEP);	
	}

	//swith input
	LCD_SDI_SWITCH_INPUT

	if(count>1)
		{
		//dummy clock cycle
		LCD_SCL_LOW
		udelay(DEFAULT_USLEEP); 	
		LCD_SCL_HIGH
		udelay(DEFAULT_USLEEP);	
		}

	/* Read Parameter */
	if (count > 0) {
		for (j = 0; j < count; j++) {

			for (i = 7; i >= 0; i--) { 
				LCD_SCL_LOW
				udelay(DEFAULT_USLEEP);	
				// read bit
				if(LCD_SDI_DATA)
					buf[j] |= (0x1<<i);
				else
					buf[j] &= ~(0x1<<i);
				LCD_SCL_HIGH
				udelay(DEFAULT_USLEEP);	
			}

		}
	}

	LCD_CSX_HIGH
	udelay(DEFAULT_USLEEP);	

	//switch output
	LCD_SDI_SWITCH_OUTPUT_LOW
}

static void read_ldi_register_sdo(u8 addr, u8 *buf, int count)
{
	long i, j;

	LCD_CSX_HIGH
	udelay(DEFAULT_USLEEP);
	LCD_SCL_HIGH 
	udelay(DEFAULT_USLEEP);

	/* Write Command */
	LCD_CSX_LOW
	udelay(DEFAULT_USLEEP);
	LCD_SCL_LOW 
	udelay(DEFAULT_USLEEP);		
	LCD_SDI_LOW 
	udelay(DEFAULT_USLEEP);
	
	LCD_SCL_HIGH 
	udelay(DEFAULT_USLEEP); 

   	for (i = 7; i >= 0; i--) { 
		LCD_SCL_LOW
		udelay(DEFAULT_USLEEP);
		if ((addr >> i) & 0x1)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(DEFAULT_USLEEP);	
		LCD_SCL_HIGH
		udelay(DEFAULT_USLEEP);	
	}

	//swith input
	LCD_SDO_SWITCH_INPUT

	if(count>1)
		{
		//dummy clock cycle
		LCD_SCL_LOW
		udelay(DEFAULT_USLEEP); 	
		LCD_SCL_HIGH
		udelay(DEFAULT_USLEEP);	
		}

	/* Read Parameter */
	if (count > 0) {
		for (j = 0; j < count; j++) {

			for (i = 7; i >= 0; i--) { 
				LCD_SCL_LOW
				udelay(DEFAULT_USLEEP);	
				// read bit
				if(LCD_SDO_DATA)
					buf[j] |= (0x1<<i);
				else
					buf[j] &= ~(0x1<<i);
				LCD_SCL_HIGH
				udelay(DEFAULT_USLEEP);	
			}

		}
	}

	LCD_CSX_HIGH
	udelay(DEFAULT_USLEEP);	

	//switch output
	LCD_SDI_SWITCH_OUTPUT_LOW
}

static void setting_table_write(struct setting_table *table)
{
    long i, j;
	
    spin_lock(&lcd_lock);
    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP);

    /* Write Command */
    LCD_CSX_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SCL_LOW 
    udelay(DEFAULT_USLEEP);     
    LCD_SDI_LOW 
    udelay(DEFAULT_USLEEP);
    
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP); 

    for (i = 7; i >= 0; i--) { 
        LCD_SCL_LOW
        udelay(DEFAULT_USLEEP);
        if ((table->command >> i) & 0x1)
            LCD_SDI_HIGH
        else
            LCD_SDI_LOW
        udelay(DEFAULT_USLEEP); 
        LCD_SCL_HIGH
        udelay(DEFAULT_USLEEP); 
    }

    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP); 

    /* Write Parameter */
    if ((table->parameters) > 0) {
    for (j = 0; j < table->parameters; j++) {
        LCD_CSX_LOW 
        udelay(DEFAULT_USLEEP);     
        
        LCD_SCL_LOW 
        udelay(DEFAULT_USLEEP);     
        LCD_SDI_HIGH 
        udelay(DEFAULT_USLEEP);
        LCD_SCL_HIGH 
        udelay(DEFAULT_USLEEP);     

        for (i = 7; i >= 0; i--) { 
            LCD_SCL_LOW
            udelay(DEFAULT_USLEEP); 
            if ((table->parameter[j] >> i) & 0x1)
                LCD_SDI_HIGH
            else
                LCD_SDI_LOW
            udelay(DEFAULT_USLEEP); 
            LCD_SCL_HIGH
            udelay(DEFAULT_USLEEP);                 
        }

            LCD_CSX_HIGH
            udelay(DEFAULT_USLEEP); 
    }
    }
	spin_unlock(&lcd_lock);

	if(!bdisplay_init) //cha_temp
	{
		mdelay(table->wait);
	}else{
		if(table->wait)
		msleep(table->wait);
	}
}

static unsigned char mtp_value_auo[3] = { 0x6B, 0x4C, 0x10 };
static unsigned char mtp_value_sharp[3] = { 0x6B, 0x1C, 0x10 };

static void determine_lcd_type(unsigned char* ids)
{
	unsigned char data[4] = {0, };

	DPRINT("start %s\n", __func__);	
	
	// read MTP from LDI
	read_ldi_register(RDID1, &data[0], 1);

	if( data[0] == 0x6B )
	{
	read_ldi_register(RDID2, &data[1], 1);
	read_ldi_register(RDID3, &data[2], 1);
		printk ( "@@@@@     %s : read by SDI == AUO or SHARP\n", __func__ );
	}
	else
	{
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		read_ldi_register_sdo(RDID1, &data[0], 1);
		read_ldi_register_sdo(RDID2, &data[1], 1);
		read_ldi_register_sdo(RDID3, &data[2], 1);
		printk ( "@@@@@     %s : read by SDO == SMD\n", __func__ );
	}

	printk("ldi mtpdata: %x %x %x\n", data[0], data[1], data[2]);

	if(data[0]==mtp_value_auo[0] && data[1]==mtp_value_auo[1] && data[2]==mtp_value_auo[2])
	{
		printk("lcd type is AUO\n");
		lcd_type = LcdTypeAUO;
	}
	else if(data[0]==mtp_value_sharp[0] && data[1]==mtp_value_sharp[1] && data[2]==mtp_value_sharp[2])
	{
		printk("lcd type is SHARP\n");
		lcd_type = LcdTypeSHARP;
	}
	else
	{
		printk("lcd default type is AUO\n");
		lcd_type = LcdTypeAUO;
	}
}

static void spi_init(void)
{
    /* Setting the Default GPIO's */
    spi_sclk = *(lcdc_s6d05a1_pdata->gpio_num);
    spi_cs   = *(lcdc_s6d05a1_pdata->gpio_num + 1);
    spi_sdi  = *(lcdc_s6d05a1_pdata->gpio_num + 2);
    spi_sdo   = *(lcdc_s6d05a1_pdata->gpio_num + 3); //sdo
    lcd_reset= *(lcdc_s6d05a1_pdata->gpio_num + 4);
//  spi_sdo  = *(lcdc_s6d05a1_pdata->gpio_num + 3);

    /* Set the output so that we dont disturb the slave device */
    gpio_set_value(spi_sclk, 0);
    gpio_set_value(spi_sdi, 0);
	gpio_set_value(spi_sdo, 0);

    /* Set the Chip Select De-asserted */
    gpio_set_value(spi_cs, 0);

}

static void s6d05a1_disp_powerup(void)
{

    DPRINT("start %s\n", __func__); 

    if (!s6d05a1_state.disp_powered_up && !s6d05a1_state.display_on) 
    {
        /* Reset the hardware first */
        //msleep(1);
        //gpio_set_value(lcd_reset, 1);
        if(!bdisplay_init)
        	mdelay(50);
		else
			msleep(50);
        
        //LCD_RESET_N_LO
        gpio_set_value(lcd_reset, 0);
        if(!bdisplay_init)		
        	mdelay(100);
		else
			msleep(100);
        //LCD_RESET_N_HI
        gpio_set_value(lcd_reset, 1);
		if(!bdisplay_init)
			mdelay(20);
		else 
			msleep(20);

        /* Include DAC power up implementation here */      
        s6d05a1_state.disp_powered_up = TRUE;
    }
}

static void s6d05a1_disp_powerdown(void)
{

    DPRINT("start %s\n", __func__); 

        /* Reset Assert */
    gpio_set_value(lcd_reset, 0);   
    //LCD_CSX_LOW
    //LCD_SCL_LOW
    //LCD_SDI_LOW
    s6d05a1_state.disp_powered_up = FALSE;
}

static void s6d05a1_disp_on(void)
{
    int i;

    DPRINT("start %s\n", __func__); 

    //spin_lock(&lcd_lock);

    if (s6d05a1_state.disp_powered_up && !s6d05a1_state.display_on) {
    
        /* s6d05a1 setting */
		if(lcd_type == LcdTypeAUO)
		{
			for (i = 0; i < POWER_ON_SETTINGS(power_on_setting_table_auo); i++)
				setting_table_write(&power_on_setting_table_auo[i]);
		}
		else if(lcd_type == LcdTypeSHARP)
		{
			for (i = 0; i < POWER_ON_SETTINGS(power_on_setting_table_sharp); i++)
				setting_table_write(&power_on_setting_table_sharp[i]);			
		}
		else
		{
			for (i = 0; i < POWER_ON_SETTINGS(power_on_setting_table_auo); i++)
				setting_table_write(&power_on_setting_table_auo[i]);			
		}

        mdelay(1);
        s6d05a1_state.display_on = TRUE;
    }
    
   // spin_unlock(&lcd_lock);
    
}

static int lcdc_s6d05a1_panel_on(struct platform_device *pdev)
{
    static int first_kernel_enter_flag = 0;
	unsigned char data[4] = {0, };
    
    DPRINT("start %s\n", __func__); 

    if(first_kernel_enter_flag == 0)
    {
       lcdc_s6d05a1_pdata->panel_config_gpio(1);
            
      /* Control LCD_D(23) for sleep current */
      //gpio_tlmm_config(GPIO_CFG(100, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);

      spi_init();   /* LCD needs SPI */ 
	  
	  if( lcd_type == 0 )
	  {
		  // Sleep Out before ID read
		  setting_table_write(&power_on_setting_table_sleep_out[0]);
	  
		  // read MTP from LDI
		  determine_lcd_type(data);
	  }

      s6d05a1_state.disp_initialized = TRUE;
      s6d05a1_state.display_on = TRUE;
      s6d05a1_state.disp_powered_up = TRUE;
      
      first_kernel_enter_flag = 1;
    }
    else
    {
        if (!s6d05a1_state.disp_initialized) 
        {
        lcdc_s6d05a1_pdata->panel_config_gpio(1);
        
        /* Control LCD_D(23) for sleep current */
        //gpio_tlmm_config(GPIO_CFG(100, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);

        s6d05a1_disp_powerup();
        spi_init(); /* LCD needs SPI */
		if( lcd_type == 0 )
		{
			// Sleep Out before ID read
			setting_table_write(&power_on_setting_table_sleep_out[0]);

			// read MTP from LDI
			determine_lcd_type(data);
		}		
        s6d05a1_disp_on();
        
#ifdef FEATURE_LCD_ESD_DET
        if (s6d05a1_state.irq_disabled) 
        {
            enable_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET));
            s6d05a1_state.irq_disabled = FALSE;
        }
#endif
        //mdelay(50);
        s6d05a1_state.disp_initialized = TRUE;
    }
   }
    bdisplay_init = TRUE ; //cha_temp
    DPRINT("end %s\n", __func__); 
    return 0;
}

static int lcdc_s6d05a1_panel_off(struct platform_device *pdev)
{
    int i;

    DPRINT("start %s\n", __func__); 

    // TEMP CODE for BLU
    //gpio_set_value(GPIO_BL_CTRL, 0);

    if (s6d05a1_state.disp_powered_up && s6d05a1_state.display_on) 
    {   
        lcd_brightness = 0;
    
#ifdef FEATURE_LCD_ESD_DET
        disable_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET));
        s6d05a1_state.irq_disabled = TRUE;
#endif

        for (i = 0; i < POWER_OFF_SETTINGS; i++)
            setting_table_write(&power_off_setting_table[i]);   
        //mdelay(120);    
        s6d05a1_disp_powerdown();
        
        lcdc_s6d05a1_pdata->panel_config_gpio(0);
        s6d05a1_state.display_on = FALSE;
        s6d05a1_state.disp_initialized = FALSE;
        
        /* Control LCD_D(23) for sleep current */
        //gpio_tlmm_config(GPIO_CFG(100, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);

    }
    return 0;
}

#if defined(CONFIG_MACH_ESCAPE)
#define MAX_BRIGHTNESS_IN_BLU   32 //64
#else
#define MAX_BRIGHTNESS_IN_BLU   64
#endif

#define PULSE_CAL(x) (((x)==0) ? MAX_BRIGHTNESS_IN_BLU:(MAX_BRIGHTNESS_IN_BLU -((x)*2) + 1))

/*controlling back light from both platform and gp2a (light sensor) */
static void control_backlight(int bl_level )
  {
      //int pulse,pulse2;

      /* lock necessary in order control lcd backlight from gp2a light sensor and platform code */
//      spin_lock(&lcd_lock);

      DPRINT("lcdc_s6d05a1_set_backlight!!!\n");
      DPRINT("bl_level = %d , lcd_brightness = %d\n", bl_level, lcd_brightness); 
      if(!bl_level || lcd_brightness <= 0)            // reduce log msg
        DPRINT("start %s:%d\n", __func__, bl_level);

      backlight_setting_table.parameter[0] = 0;
      setting_table_write(&backlight_setting_table);
      udelay(100); 
   	  bl_level = (bl_level * 175)/255;	//MBjclee
      backlight_setting_table.parameter[0] = (bl_level & BACKLIGHT_LEVEL_VALUE);
     // DPRINT("bl_level = %d , reg = 0x%x\n", bl_level, backlight_setting_table.parameter[0]); //MBjclee
      setting_table_write(&backlight_setting_table);

      lcd_brightness = bl_level;

      /* unlock the baklight control */
//      spin_unlock(&lcd_lock);
 }

static void lcdc_s6d05a1_set_backlight(struct msm_fb_data_type *mfd)
{
        int bl_level = mfd->bl_level;
        /* Value necessary for controlling backlight in case light sensor gp2a is off */
        app_bl_level = bl_level;

        /* To control back light from both platform and gp2a (light sensor) */
        control_backlight(bl_level);
}

/* Auto brightness to be controlled by gp2a driver in case Auto brightess in enabled */
void lcdc_set_backlight_autobrightness(int bl_level)
{
        /* Control backlight routine */             
        control_backlight(bl_level);
}

#ifdef __LCD_CONTROL_BY_FILE__
static int s3cfb_sysfs_show_lcd_power(struct device *dev, struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", s6d05a1_state.disp_initialized);
}

static int s3cfb_sysfs_store_lcd_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{

        struct platform_device* pdev;

    if (len < 1)
        return -EINVAL;

        pdev = container_of(dev, struct platform_device, dev);

    if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0)
        {
                lcdc_s6d05a1_panel_on(pdev);
        lcdc_set_backlight_autobrightness(15);
        }
    else if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0)
        {
        lcdc_set_backlight_autobrightness(0);
                lcdc_s6d05a1_panel_off(pdev);
        }
    else
            return -EINVAL;

    return len;
}

static DEVICE_ATTR(lcd_power, 0664,                                             //sm.kim: give write permission for application
                        s3cfb_sysfs_show_lcd_power,
                        s3cfb_sysfs_store_lcd_power);

#endif //MBjclee 2011.1.24     LCD controled by FILE

static int __init s6d05a1_probe(struct platform_device *pdev)
{
    int err;
	unsigned char data[4] = {0, };	
    DPRINT("start %s\n", __func__); 

#ifdef __LCD_CONTROL_BY_FILE__  //MBjclee 2011.1.24    LCD controled by FILE

    err = device_create_file(&(pdev->dev), &dev_attr_lcd_power);
    if ( err < 0)
            printk(KERN_WARNING "s6d05a1: failed to add entries\n");
#endif


#ifdef FEATURE_LCD_ESD_DET
    err = request_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET), s6d05a1_esd_irq_handler, IRQF_TRIGGER_RISING,
      "LCD_ESD_DET", (void*)pdev->dev.platform_data);

    if (err) {
        DPRINT("%s, request_irq failed %d(ESD_DET), ret= %d\n", __func__, GPIO_ESD_DET, err);    
    }  
#endif

    if (pdev->id == 0) {
        lcdc_s6d05a1_pdata = pdev->dev.platform_data;
        return 0;
    }

	// read MTP from LDI
	determine_lcd_type(data);

	if(data[0]==mtp_value_auo[0] && data[1]==mtp_value_auo[1] && data[2]==mtp_value_auo[2])
	{
		lcd_type = LcdTypeAUO;
	}
	else if(data[0]==mtp_value_sharp[0] && data[1]==mtp_value_sharp[1] && data[2]==mtp_value_sharp[2])
	{
		lcd_type = LcdTypeSHARP;
	}
	else
	{
		lcd_type = LcdTypeAUO;
	}
	
    msm_fb_add_device(pdev);
	lcdc_s6d05a1_panel_on(pdev);
    return 0;
}

static void s6d05a1_shutdown(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__); 

    lcdc_s6d05a1_panel_off(pdev);
}

static struct platform_driver this_driver = {
    .probe  = s6d05a1_probe,
    .shutdown   = s6d05a1_shutdown,
    .driver = {
        .name   = "lcdc_s6d05a1_hvga",
    },
};

static struct msm_fb_panel_data s6d05a1_panel_data = {
    .on = lcdc_s6d05a1_panel_on,
    .off = lcdc_s6d05a1_panel_off,
    .set_backlight = lcdc_s6d05a1_set_backlight,
};

static struct platform_device this_device = {
    .name   = "lcdc_panel",
    .id = 1,
    .dev    = {
        .platform_data = &s6d05a1_panel_data,
    }
};

#define LCDC_FB_XRES    320
#define LCDC_FB_YRES    480

#define LCDC_HBP        15//16//3//22
#define LCDC_HPW        5//3//14
#define LCDC_HFP        15//16//3//14
#define LCDC_VBP        8//8
#define LCDC_VPW        2//
#define LCDC_VFP        8//8//2

#define LCDC_PIXELCLK    98 \
                    * (LCDC_HFP+LCDC_HPW+LCDC_HBP+LCDC_FB_XRES) \
                    * (LCDC_VFP+LCDC_VPW+LCDC_VBP+LCDC_FB_YRES)

static int __init lcdc_s6d05a1_panel_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

    s6d05a1_state.panel_initialized = TRUE;
#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
    if (msm_fb_detect_client("lcdc_s6d05a1_hvga"))
    {
        printk(KERN_ERR "%s: msm_fb_detect_client failed!\n", __func__); 
        return 0;
    }
#endif
    DPRINT("start %s\n", __func__); 

    /* used for controlling control backilght code from gp2a light sensor and platform code */
    spin_lock_init(&lcd_lock);
    
    ret = platform_driver_register(&this_driver);
    if (ret)
    {
        printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n", __func__, ret); 
        return ret;
    }

    pinfo = &s6d05a1_panel_data.panel_info;
    pinfo->xres = LCDC_FB_XRES;
    pinfo->yres = LCDC_FB_YRES;
    pinfo->height = 70; //in mm for DPI calibration on platform
    pinfo->width = 42; //in mm for DPI calibration on platform
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 24;
    pinfo->fb_num = 2;
	pinfo->clk_rate = 17096348; //(14894 * 1000);//7500000;//8192000;

    pinfo->bl_max = 255; //Max brightness of LCD
    pinfo->bl_min = 1;

    pinfo->lcdc.h_back_porch = LCDC_HBP;
    pinfo->lcdc.h_front_porch = LCDC_HFP;
    pinfo->lcdc.h_pulse_width = LCDC_HPW;
    pinfo->lcdc.v_back_porch = LCDC_VBP;
    pinfo->lcdc.v_front_porch = LCDC_VFP;
    pinfo->lcdc.v_pulse_width = LCDC_VPW;
    pinfo->lcdc.border_clr = 0;     /* blk */
    pinfo->lcdc.underflow_clr = 0xff;       /* blue */
    pinfo->lcdc.hsync_skew = 0;

    printk("%s\n", __func__);

    ret = platform_device_register(&this_device);
    if (ret)
    {
        printk(KERN_ERR "%s: platform_device_register failed! ret=%d\n", __func__, ret); 
        platform_driver_unregister(&this_driver);
    }

    s6d05a1_state.panel_initialized = FALSE;
    return ret;
}

#ifdef FEATURE_LCD_ESD_DET
void s6d05a1_esd(void)
{
    if (gpio_get_value(GPIO_ESD_DET) == 0)
        return;
    
    DPRINT("start %s, panel_initialized: %d\n", __func__, s6d05a1_state.panel_initialized);
    if (!s6d05a1_state.panel_initialized) {
        lcdc_s6d05a1_panel_off(NULL);
        mdelay(20);
        lcdc_s6d05a1_panel_on(NULL);
    }
}

static DECLARE_WORK(lcd_esd_work, s6d05a1_esd);

static irqreturn_t s6d05a1_esd_irq_handler(int irq, void *handle)
{
    DPRINT("start %s\n", __func__);
    schedule_work(&lcd_esd_work);  

    return IRQ_HANDLED;
}
#endif

module_init(lcdc_s6d05a1_panel_init);
