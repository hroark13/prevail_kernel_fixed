/*
 * =====================================================================================
 *
 *       Filename:  lcdc_s6d_backlight.c
 *
 *    Description:  LCD Backlight control Function
 *
 *        Version:  1.0
 *        Created:  2010??11??25??15??21ë¶?38ì´?
 *       Revision:  1.0
 *       Compiler:  arm-linux-gcc
 *
 *         Author:  File (System S/W Group Kernel Part), 
 *        Company:  Samsung Electronics
 *
 * =====================================================================================
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
#include <mach/gpio.h>
#include "lcdc_s6d_backlight.h"
#include "msm_fb.h"

int lcd_type = 1;
static int lcd_brightness = -1;

struct brt_value brt_table_ktd[] = {
#if defined(LCD_GIO_HW_REV)
   { 0,  5 }, // Off
   { 20, 1 }, // Dimming pulse 
   { MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse 1(33-32) by HW
   { 44,  2 }, 
   { 53,  3 },  
   { 62,  4 }, 
   { 71,  5 }, 
   { 80,  6 }, 
   { 89,  7 },
   { 98,  8 },
   { 107,  9 }, 
   { 116,  10 },
   { 125,  11 },
   { 133,  12 },
   { 145,  13 }, // default pulse 
   { 157,  13 }, 
   { 167,  14 },
   { 177,  15 },
   { 187,  16 },
   { 197,  17 }, 
   { 207,  18 },
   { 217,  19 },
   { 227,  20 },   
   { 237,  21 },      
   { 247,  22 },         
   { MAX_BRIGHTNESS_VALUE,  23},
#else
   { 0,  5 }, // Off
   { 20, 1 }, // Dimming pulse 
   { MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse 1(33-32) by HW
   { 44,  2 }, 
   { 52,  3 },  
   { 60,  4 }, 
   { 69,  5 }, 
   { 77,  6 }, 
   { 85,  7 }, 
   { 93,  8 },
   { 102,  9 },
   { 110,  10 }, 
   { 118,  11 },
   { 126,  12 },
   { 134,  13 },
   { 145,  14 }, // default pulse 19(33-14) by HW
   { 153,  15 }, 
   { 161,  16 }, 
   { 169,  17 }, 
   { 177,  18 }, 
   { 185,  19 }, 
   { 193,  20  },
   { 201,  21  },
   { 209,  22  },
   { 217,  23  },
   { 225,  24  },
   { 233,  25  },
   { 241,  26  },
   { MAX_BRIGHTNESS_VALUE,  27 }, // Max pulse 6(33-27) by HW
#endif	  
};
struct brt_value brt_table_shp[] = {
	  { 0,  5 }, // Off
	  { 20, 1 }, // Dimming pulse 
	  { MIN_BRIGHTNESS_VALUE,  1 }, // Min pulse 1(33-32) by HW
	  { 44,  2 }, 
	  { 52,  3 },  
	  { 60,  4 }, 
	  { 69,  5 }, 
	  { 77,  6 }, 
	  { 85,  7 }, 
	  { 93,  8 },
	  { 102,  9 },
	  { 110,  10 }, 
	  { 118,  11 },
	  { 126,  12 },
	  { 134,  13 },
	  { 145,  15 }, // default pulse 18(33-15) by HW
	  { 153,  15 }, 
	  { 161,  16 }, 
	  { 169,  17 }, 
	  { 177,  18 }, 
	  { 185,  19 }, 
	  { 193,  20  },
	  { 201,  21  },
	  { 209,  22  },
	  { 217,  23  },
	  { 225,  24  },
	  { 233,  25  },
	  { 241,  26  },
	  //{ MAX_BRIGHTNESS_VALUE,  27 }, // Max pulse 6(33-27) by HW
	  { MAX_BRIGHTNESS_VALUE,  31 }, // Max pulse 2(33-31) by HW
};

#define MAX_BRT_STAGE_KTD (int)(sizeof(brt_table_ktd)/sizeof(struct brt_value))
#define MAX_BRT_STAGE_SHP (int)(sizeof(brt_table_shp)/sizeof(struct brt_value))

static DEFINE_SPINLOCK(bl_ctrl_lock);

void lcdc_s6d_set_brightness_by_ktd259(int level)
{
   int pulse;
   int tune_level = 0;
   int i;
   unsigned long flag;

    //		spin_lock(&bl_ctrl_lock);
#if 1 //defined CONFIG_MACH_GIO
   if(level > 0)
   {
      if(level < MIN_BRIGHTNESS_VALUE)
   	{
   		if(lcd_type == 1)  // SMD
   		{
   			tune_level = brt_table_ktd[1].tune_level;			
   		}
   		else   // lcd_type = 2, Sharp
   		{
   			tune_level = brt_table_shp[1].tune_level;
   		}
   	} 
   	else if (level == MAX_BRIGHTNESS_VALUE)
   	{
   		if(lcd_type == 1)  // SMD
   		{
   			tune_level = brt_table_ktd[MAX_BRT_STAGE_KTD-1].tune_level;			
   		}
   		else   // lcd_type = 2, Sharp
   		{
   			tune_level = brt_table_shp[MAX_BRT_STAGE_SHP-1].tune_level;
   		}
   	}
   	else
   	{
   		if(lcd_type == 1)
   		{
   			for(i = 0; i < MAX_BRT_STAGE_KTD; i++)
   			{
   				if(level <= brt_table_ktd[i].level )
   				{
   					tune_level = brt_table_ktd[i].tune_level;
   					break;
   				}
   			}
   		}
   		else
   		{
   			for(i = 0; i < MAX_BRT_STAGE_SHP; i++)
   			{
   				if(level <= brt_table_shp[i].level )
   				{
   					tune_level = brt_table_shp[i].tune_level;
   					break;
   				}
   			}
   		}
     }
   }
#endif

    printk("Backlight Platform V:%d, Find V:%d\n",level, tune_level);

   if(tune_level <= 0) 
   {
      gpio_set_value(GPIO_BL_CTRL, 0);
      mdelay(3);
      lcd_brightness = tune_level;
   } 
   else 
   {
      if(unlikely(lcd_brightness < 0)) 
      {
    			// first time
         int val = gpio_get_value(GPIO_BL_CTRL);
         if(val)
         {
            lcd_brightness = 0;
            gpio_set_value(GPIO_BL_CTRL, 0);
            mdelay(3); // guaranteed for shutdown
         }
    	} 
    	if(!lcd_brightness)
    	{
         gpio_set_value(GPIO_BL_CTRL, 1);
         udelay(3);
         lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
    	}
    	pulse = (lcd_brightness - tune_level + MAX_BRIGHTNESS_IN_BLU) % MAX_BRIGHTNESS_IN_BLU;
      
//      printk("Baclight ADC Value:[%d], Tune level:[%d], Pluse:[%d], lcd_brightness:[%d] \n",level, tune_level,pulse,lcd_brightness);
      spin_lock_irqsave(&bl_ctrl_lock,flag); // minimize the sipn lock time
      for(;pulse>0;pulse--) 
      {
         gpio_set_value(GPIO_BL_CTRL, 0);
         udelay(1); // recommanded from the backlight company by 3 to 1
         gpio_set_value(GPIO_BL_CTRL, 1);
         udelay(1);
      }
      spin_unlock_irqrestore(&bl_ctrl_lock,flag);                
   }
   lcd_brightness = tune_level;
    //		spin_unlock(&bl_ctrl_lock);
}


