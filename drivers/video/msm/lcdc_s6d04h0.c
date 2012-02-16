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

#define LCDC_DEBUG
//#define FEATURE_LCD_ESD_DET

#ifdef LCDC_DEBUG
#define DPRINT(x...)    printk("s6d04h0 " x)
#else
#define DPRINT(x...)    
#endif

#define GPIO_BL_CTRL  26

#define BACKLIGHT_LEVEL_VALUE   0xff

#ifdef FEATURE_LCD_ESD_DET
#define GPIO_ESD_DET    94
#endif

/*
 * Serial Interface
 */
#define LCD_CSX_HIGH    gpio_set_value(spi_cs, 1);
#define LCD_CSX_LOW     gpio_set_value(spi_cs, 0);

#define LCD_SCL_HIGH    gpio_set_value(spi_sclk, 1);
#define LCD_SCL_LOW     gpio_set_value(spi_sclk, 0);

#define LCD_SDI_HIGH    gpio_set_value(spi_sdi, 1);
#define LCD_SDI_LOW     gpio_set_value(spi_sdi, 0);

#define DEFAULT_USLEEP  5   
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
#define GAMMASEL 		0xF9
#define PGAMMACTL 		0xFA
#define NGAMMACTL 		0xFB
#define TEON 			0x35
#define DISPON          0x29
#define BCMODE          0xC1
#define WRCTRLD         0x53
#define MADCTL          0x36
#define CASET 			0x2A
#define PASET 			0x2B


struct setting_table {
    unsigned char command;  
    unsigned char parameters;
    unsigned char parameter[19];
    long wait;
};

static struct setting_table power_on_setting_table[] = {
    {	PASSWD1,	2, { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },
    {	PASSWD2,	2, { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },

    {	PWRCTL,    14, { 0x09, 0x00, 0x00, 0x00, 0x77, 0x77, 0x07, 0x02, 0x2E, 0x47, 0x05, 0x2A, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },		
    {	VCMCTL,    10, { 0x00, 0x23, 0x5D, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },		
    {	SRCCTL,     9, { 0x00, 0x01, 0x07, 0x00, 0x01, 0x0C, 0x01, 0x0C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },		

    {	TEON,  	1, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
    {	MADCTL,  	1, { 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
    {	COLMOD,  	1, { 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },	
    {	IFCTL,  	4, { 0x02, 0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				

    {	PANELCTL,   2, { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	DISCTL,    17, { 0x28, 0x67, 0x03, 0x02, 0x02, 0x00, 0x00, 0x15, 0x48, 0x00, 0x00, 0x01, 0x00, 0x00, 0x59, 0x08, 0x08, 0x00, 0x00 },	0 },		
    {	GAMMASEL,   1, { 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	PGAMMACTL, 16, { 0x00, 0x00, 0x10, 0x17, 0x25, 0x31, 0x21, 0x31, 0x2B, 0x28, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	NGAMMACTL, 16, { 0x00, 0x00, 0x30, 0x32, 0x32, 0x33, 0x23, 0x29, 0x22, 0x1B, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	GAMMASEL,   1, { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	PGAMMACTL, 16, { 0x00, 0x00, 0x10, 0x17, 0x25, 0x31, 0x21, 0x31, 0x2B, 0x28, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	NGAMMACTL, 16, { 0x00, 0x00, 0x30, 0x32, 0x32, 0x33, 0x23, 0x29, 0x22, 0x1B, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	GAMMASEL,   1, { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	PGAMMACTL, 16, { 0x00, 0x00, 0x10, 0x17, 0x25, 0x31, 0x21, 0x31, 0x2B, 0x28, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				
    {	NGAMMACTL, 16, { 0x00, 0x00, 0x30, 0x32, 0x32, 0x33, 0x23, 0x29, 0x22, 0x1B, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },				

    {	CASET,	4, { 0x00, 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },
    {	PASET,	   4, { 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },

    {	PASSWD1,	2, { 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },
    {	PASSWD2,	2, { 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },

    {	SLPOUT,	0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	160 },

    {	DISPON,	0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	0 },
};

#define POWER_ON_SETTINGS   (int)(sizeof(power_on_setting_table)/sizeof(struct setting_table))

static struct setting_table power_off_setting_table[] = {
    { SLPIN,   0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

#define POWER_OFF_SETTINGS  (int)(sizeof(power_off_setting_table)/sizeof(struct setting_table))

static int lcdc_s6d04h0_panel_off(struct platform_device *pdev);
#ifdef FEATURE_LCD_ESD_DET
static irqreturn_t s6d04h0_esd_irq_handler(int irq, void *handle);
#endif

static int spi_cs;
static int spi_sclk;
//static int spi_sdo;
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


struct s6d04h0_state_type{
    boolean disp_initialized;
    boolean display_on;
    boolean disp_powered_up;
    boolean panel_initialized;
#ifdef FEATURE_LCD_ESD_DET
    boolean irq_disabled;
#endif
};


static int backlight_level;

static struct s6d04h0_state_type s6d04h0_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6d04h0_pdata;

static void setting_table_write(struct setting_table *table)
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

    msleep(table->wait);
}


static void spi_init(void)
{
    /* Setting the Default GPIO's */
    spi_sclk = *(lcdc_s6d04h0_pdata->gpio_num);
    spi_cs   = *(lcdc_s6d04h0_pdata->gpio_num + 1);
    spi_sdi  = *(lcdc_s6d04h0_pdata->gpio_num + 2);
    lcd_en   = *(lcdc_s6d04h0_pdata->gpio_num + 3);
    lcd_reset= *(lcdc_s6d04h0_pdata->gpio_num + 4);
//  spi_sdo  = *(lcdc_s6d04h0_pdata->gpio_num + 3);

    /* Set the output so that we dont disturb the slave device */
    gpio_set_value(spi_sclk, 0);
    gpio_set_value(spi_sdi, 0);

    /* Set the Chip Select De-asserted */
    gpio_set_value(spi_cs, 0);

}

static void s6d04h0_disp_powerup(void)
{
    int i; // quattro_jiny46kim [[
    DPRINT("start %s\n", __func__); 

    if (!s6d04h0_state.disp_powered_up && !s6d04h0_state.display_on) 
    {
        /* Reset the hardware first */
        msleep(1);
        gpio_set_value(lcd_reset, 0);
        msleep(20);
        gpio_set_value(lcd_reset, 1);
        msleep(50);

        /* Include DAC power up implementation here */      
        s6d04h0_state.disp_powered_up = TRUE;
    }
}

static void s6d04h0_disp_powerdown(void)
{
    int i ; // quattro_jiny46kim [[

    DPRINT("start %s\n", __func__); 

        /* Reset Assert */
    gpio_set_value(lcd_reset, 0);   
    LCD_CSX_LOW
    LCD_SCL_LOW
    LCD_SDI_LOW
    s6d04h0_state.disp_powered_up = FALSE;
}

static void s6d04h0_disp_on(void)
{
    int i;

    DPRINT("start %s\n", __func__); 

    if (s6d04h0_state.disp_powered_up && !s6d04h0_state.display_on) {
    
        /* s6d04h0 setting */
        for (i = 0; i < POWER_ON_SETTINGS; i++)
            setting_table_write(&power_on_setting_table[i]);            // new lcd

        s6d04h0_state.display_on = TRUE;
    }
}

static int lcdc_s6d04h0_panel_on(struct platform_device *pdev)
{
    static int first_kernel_enter_flag = 0;
    int i;
    
    DPRINT("start %s\n", __func__); 

    if(first_kernel_enter_flag == 0)
    {
       lcdc_s6d04h0_pdata->panel_config_gpio(1);
            
      spi_init();   /* LCD needs SPI */ 

      s6d04h0_state.disp_initialized = TRUE;
      s6d04h0_state.display_on = TRUE;
      s6d04h0_state.disp_powered_up = TRUE;
      
      first_kernel_enter_flag = 1;
    }
    else
    {
        if (!s6d04h0_state.disp_initialized) 
        {
        lcdc_s6d04h0_pdata->panel_config_gpio(1);
        
        spi_init(); /* LCD needs SPI */
        s6d04h0_disp_powerup();
        s6d04h0_disp_on();
        
#ifdef FEATURE_LCD_ESD_DET
        if (s6d04h0_state.irq_disabled) 
        {      
            enable_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET));
            s6d04h0_state.irq_disabled = FALSE;
        }
#endif
        s6d04h0_state.disp_initialized = TRUE;
    }
   }
    
    return 0;
}

#define MAX_BRIGHTNESS_IN_BLU   64

#define PULSE_CAL(x) (((x)==0) ? MAX_BRIGHTNESS_IN_BLU:(MAX_BRIGHTNESS_IN_BLU -((x)*2) + 1))

/*controlling back light from both platform and gp2a (light sensor) */
static void control_backlight(int bl_level )
{
  
      int pulse,pulse2;

      /* lock necessary in order control lcd backlight from gp2a light sensor and platform code */
      spin_lock(&lcd_lock);

      DPRINT("lcdc_s6d04h0_set_backlight!!!\n");
      DPRINT("bl_level = %d , lcd_brightness = %d\n", bl_level, lcd_brightness); 
      if(!bl_level || lcd_brightness <= 0)            // reduce log msg
        DPRINT("start %s:%d\n", __func__, bl_level);

      if (bl_level <= 0)
      {
        /* keep back light OFF */
        
        gpio_set_value(GPIO_BL_CTRL, 0);
        mdelay(5);              //guaranteed shutdown
      }
      else
      {    
        if(lcd_brightness != bl_level)
        {
          if(bl_level > 32)
            bl_level = 32;
            
          pulse2 = pulse = MAX_BRIGHTNESS_IN_BLU -(bl_level*2) + 1; //MAX_BRIGHTNESS_IN_BLU - bl_level;

          gpio_set_value(GPIO_BL_CTRL, 0);
          mdelay(3); udelay(400);

          for(; pulse>0; pulse--)
          {
                gpio_set_value(GPIO_BL_CTRL, 0);
                ndelay(600);
                gpio_set_value(GPIO_BL_CTRL, 1);

                if(pulse2 == pulse)
                {  mdelay(1); udelay(100);}
                else
                  ndelay(600);
          }
       }
      }
      lcd_brightness = bl_level;

      /* unlock the baklight control */
      spin_unlock(&lcd_lock);
 }

static int lcdc_s6d04h0_panel_off(struct platform_device *pdev)
  
{
    int i;

    DPRINT("start %s\n", __func__); 

    control_backlight(0);
    lcd_brightness = 0;

    if (s6d04h0_state.disp_powered_up && s6d04h0_state.display_on) 
    {   
        for (i = 0; i < POWER_OFF_SETTINGS; i++)
            setting_table_write(&power_off_setting_table[i]);   
            
        s6d04h0_disp_powerdown();
        
        lcdc_s6d04h0_pdata->panel_config_gpio(0);
        s6d04h0_state.display_on = FALSE;
        s6d04h0_state.disp_initialized = FALSE;
        
        #ifdef FEATURE_LCD_ESD_DET
        disable_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET));
        s6d04h0_state.irq_disabled = TRUE;
    #endif
    }
    return 0;
}

static void lcdc_s6d04h0_set_backlight(struct msm_fb_data_type *mfd)
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
    //control_backlight(bl_level);
}

static int __init s6d04h0_probe(struct platform_device *pdev)
{
    int err;
    DPRINT("start %s\n", __func__); 

#ifdef FEATURE_LCD_ESD_DET
    err = request_irq(MSM_GPIO_TO_INT(GPIO_ESD_DET), s6d04h0_esd_irq_handler, IRQF_TRIGGER_RISING,
      "LCD_ESD_DET", (void*)pdev->dev.platform_data);

    if (err) {
        DPRINT("%s, request_irq failed %d(ESD_DET), ret= %d\n", __func__, GPIO_ESD_DET, err);    
    }  
#endif

    if (pdev->id == 0) {
        lcdc_s6d04h0_pdata = pdev->dev.platform_data;
        return 0;
    }
    msm_fb_add_device(pdev);
    return 0;
}

static void s6d04h0_shutdown(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__); 

    lcdc_s6d04h0_panel_off(pdev);
}

static struct platform_driver this_driver = {
    .probe  = s6d04h0_probe,
    .shutdown   = s6d04h0_shutdown,
    .driver = {
        .name   = "lcdc_s6d04h0_qvga",
    },
};

static struct msm_fb_panel_data s6d04h0_panel_data = {
    .on = lcdc_s6d04h0_panel_on,
    .off = lcdc_s6d04h0_panel_off,
    .set_backlight = lcdc_s6d04h0_set_backlight,
};

static struct platform_device this_device = {
    .name   = "lcdc_panel",
    .id = 1,
    .dev    = {
        .platform_data = &s6d04h0_panel_data,
    }
};

#define LCDC_FB_XRES    240
#define LCDC_FB_YRES    320

#define LCDC_HBP        16//2//32//16//8
#define LCDC_HPW       2//16//14//5//8
#define LCDC_HFP        16//2//16//8
#define LCDC_VBP        2//8
#define LCDC_VPW       2//8
#define LCDC_VFP        2//14//8

#define LCDC_PIXELCLK    50 \
                    * (LCDC_HFP+LCDC_HPW+LCDC_HBP+LCDC_FB_XRES) \
                    * (LCDC_VFP+LCDC_VPW+LCDC_VBP+LCDC_FB_YRES)

static int __init lcdc_s6d04h0_panel_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

    s6d04h0_state.panel_initialized = TRUE;
#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
    if (msm_fb_detect_client("lcdc_s6d04h0_qvga"))
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

    pinfo = &s6d04h0_panel_data.panel_info;
    pinfo->xres = LCDC_FB_XRES;
    pinfo->yres = LCDC_FB_YRES;
    pinfo->height = 69; //in mm for DPI calibration on platform
    pinfo->width = 42; //in mm for DPI calibration on platform
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 18;
    pinfo->fb_num = 2;
    
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_GIOS)
    pinfo->clk_rate = 24576000; //8192000; //LCDC_PIXELCLK ;//7500000;//8192000;
#elif defined(CONFIG_MACH_ROOKIE)
    pinfo->clk_rate = LCDC_PIXELCLK ;//7500000;//8192000;
#else
    pinfo->clk_rate = LCDC_PIXELCLK ;//7500000;//8192000;
#endif

    pinfo->bl_max = 29; //Max brightness of LCD
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

    s6d04h0_state.panel_initialized = FALSE;
    return ret;
}

#ifdef FEATURE_LCD_ESD_DET
void s6d04h0_esd(void)
{
    DPRINT("start %s, panel_initialized: %d\n", __func__, s6d04h0_state.panel_initialized);
    if (!s6d04h0_state.panel_initialized) {
        lcdc_s6d04h0_panel_off(NULL);
        mdelay(20);
        lcdc_s6d04h0_panel_on(NULL);
    }
}

static DECLARE_WORK(lcd_esd_work, s6d04h0_esd);

static irqreturn_t s6d04h0_esd_irq_handler(int irq, void *handle)
{
    DPRINT("start %s\n", __func__);
    schedule_work(&lcd_esd_work);  

    return IRQ_HANDLED;
}
#endif

module_init(lcdc_s6d04h0_panel_init);
