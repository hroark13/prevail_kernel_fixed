/* drivers/input/touchscreen/Melfas_mcs8000.c
 *
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>

#include <linux/slab.h>

#include "mcs8000_download.h"

#define INPUT_INFO_REG 0x10
#define IRQ_TOUCH_INT   MSM_GPIO_TO_INT(GPIO_TOUCH_INT)

#define NEW_FIRMWARE_VERSION 0x10
#define TSP_TEST_MODE 

#undef CONFIG_CPU_FREQ
#undef CONFIG_MOUSE_OPTJOY

#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

static int debug_level = 4; 
#define debugprintk(level,x...)  if(debug_level>=level) printk(x)

extern int mcsdl_download_binary_data(INT32 hw_ver);//eunsuk test  [int hw_ver -> void]
extern int mms100_ISC_download_binary_file(INT32 hw_ver);
#ifdef CONFIG_MOUSE_OPTJOY
extern int get_sending_oj_event();
#endif
#if defined(CONFIG_MACH_ESCAPE)
extern void keypad_backlight_control(void);
#endif

extern struct class *sec_class_1;


struct input_info {
	int max_x;
	int max_y;
	int state;
	int x;
	int y;
	int z;
	int x2; 
	int y2;
	int z2;
	int width;
	int finger_id;
	int keycode;
};

struct mcs7000_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	struct input_info info;
	int suspended;
	struct early_suspend	early_suspend;
};

struct mcs7000_ts_driver *melfas_mcs7000_ts = NULL;
struct i2c_driver mcs7000_ts_i2c;
struct workqueue_struct *melfas_mcs7000_ts_wq;

static struct vreg *vreg_touch;

extern unsigned char hw_version;

#if defined(CONFIG_MACH_ESCAPE)
#define TOUCH_LDO           ((hw_version>=2)?("ldo3"):("ldo19"))
#define TOUCH_LDO_LEVEL     (hw_version>=2 ? OUT2700mV : OUT3300mV)  // OUT3000mV -> OUT2700mV
#else
#define TOUCH_LDO           "ldo19"
#define TOUCH_LDO_LEVEL     OUT3300mV
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs7000_ts_early_suspend(struct early_suspend *h);
void melfas_mcs7000_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
int melfas_mcs7000_ts_gen_touch_up(void);

#define TOUCH_HOME	KEY_HOME
#define TOUCH_MENU	KEY_MENU
#define TOUCH_BACK	KEY_BACK
#define TOUCH_SEARCH  KEY_SEARCH

int melfas_ts_tk_keycode[] =
{ TOUCH_MENU, TOUCH_HOME, TOUCH_BACK, TOUCH_SEARCH, };

struct device *mcs7000_ts_dev;

#ifdef TSP_TEST_MODE
static uint16_t tsp_test_reference[10];
static uint16_t tsp_test_inspection[14][10];
uint8_t refer_y_channel_num = 1;
uint8_t inspec_y_channel_num = 1;
#endif

void mcsdl_vdd_on(void)
{ 
  vreg_set_level(vreg_touch, TOUCH_LDO_LEVEL);
  vreg_enable(vreg_touch);
  mdelay(25); //MUST wait for 25ms after vreg_enable() 
}

void mcsdl_vdd_off(void)
{
  vreg_disable(vreg_touch);
  mdelay(100); //MUST wait for 100ms before vreg_enable() 
}

static int melfas_mcs7000_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}
	
	return 0;
}

static int melfas_mcs7000_i2c_write(struct i2c_client* p_client, u8* data, int len)
{
	struct i2c_msg msg;

	msg.addr = p_client->addr;
	msg.flags = 0; /* I2C_M_WR */
	msg.len = len;
	msg.buf = data ;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void melfas_mcs7000_read_version(void)
{
	u8 buf[2] = {0,};
	
	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf, 2))
	{

		melfas_mcs7000_ts->hw_rev = buf[0];
		melfas_mcs7000_ts->fw_ver = buf[1];
		
		printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
	}
	else
	{
		melfas_mcs7000_ts->hw_rev = 0;
		melfas_mcs7000_ts->fw_ver = 0;
		
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}
}

static void melfas_mcs7000_read_resolution(void)
{
	
	uint16_t max_x=0, max_y=0;	

	u8 buf[3] = {0,};
	
	if(0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_RESOL_HIGH_REG , buf, 3)){

		printk("%s :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,buf[0],buf[1],buf[2]);

		if((buf[0] == 0)||(buf[0] == 0)||(buf[0] == 0)){
			melfas_mcs7000_ts->info.max_x = 320;
			melfas_mcs7000_ts->info.max_y = 480;
			
			printk("%s : Can't find Resolution!\n", __func__);
			}
		
		else{
			max_x = buf[1] | ((uint16_t)(buf[0] & 0x0f) << 8); 
			max_y = buf[2] | (((uint16_t)(buf[0] & 0xf0) >> 4) << 8); 
			melfas_mcs7000_ts->info.max_x = max_x;
		    melfas_mcs7000_ts->info.max_y = max_y;

			printk("%s :max_x: %d, max_y: %d\n", __func__, melfas_mcs7000_ts->info.max_x, melfas_mcs7000_ts->info.max_y);
			}
		}

	else
	{
		melfas_mcs7000_ts->info.max_x = 320;
		melfas_mcs7000_ts->info.max_y = 480;
		
		printk("%s : Can't find Resolution!\n", __func__);
	}
}

static ssize_t registers_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 buf1[2] = {0,};
	u8 buf2[2] = {0,};

	int status=0, mode_ctl=0, hw_rev=0, fw_ver=0;

	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_STATUS_REG, buf1, 2))
	{
		status = buf1[0];
		mode_ctl = buf1[1];	 
	}
	else
	{
		printk("%s : Can't find status, mode_ctl!\n", __func__); 
	}

	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf2, 2))
	{
		hw_rev = buf2[0];
		fw_ver = buf2[1];	 
	}
	else
	{
		printk("%s : Can't find HW Ver, FW ver!\n", __func__); 
	}
	
	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(melfas_mcs7000_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
    mcsdl_vdd_on();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
    mcsdl_vdd_off();
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
    mcsdl_vdd_off();
		mdelay(500);
    mcsdl_vdd_on();
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}


static ssize_t firmware_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver;


	if (0 == melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
	{
		hw_rev = buf1[0];
		fw_ver = buf1[1];	 
		sprintf(buf,"HW Ver : 0x%02x, FW Ver : 0x%02x\n", hw_rev, fw_ver);
	}
	else
	{	 
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}

return sprintf(buf, "%s", buf); 
}


static ssize_t firmware_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_mcs7000_ts->hw_rev);
		printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs7000_ts->fw_ver);
		if((melfas_mcs7000_ts->fw_ver != 0x24 && melfas_mcs7000_ts->hw_rev == 0x40)
		  ||(melfas_mcs7000_ts->fw_ver != 0x25 && melfas_mcs7000_ts->hw_rev == 0x50))
        { 
		disable_irq(melfas_mcs7000_ts->client->irq);

		printk("[F/W D/L] Entry gpio_tlmm_config\n");
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		
		printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
		ret = mms100_ISC_download_binary_file(melfas_mcs7000_ts->hw_rev);
		//ret = mcsdl_download_binary_data(melfas_mcs7000_ts->hw_rev); //eunsuk test [melfas_mcs7000_ts->hw_rev -> ()]
		
		enable_irq(melfas_mcs7000_ts->client->irq);
		
		melfas_mcs7000_read_version();
			
		if(ret > 0){
				if (melfas_mcs7000_ts->hw_rev < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");;
				}
				
				if (melfas_mcs7000_ts->fw_ver < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");
				}
				
				printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs7000_ts->hw_rev, melfas_mcs7000_ts->fw_ver);

		}
		else {
			printk("[TOUCH] Firmware update failed.. RESET!\n");
      mcsdl_vdd_off();
			mdelay(500);
      mcsdl_vdd_on();
			mdelay(200);
		}
		}
	}
#endif

	return size;
}



static ssize_t debug_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store_mcs7000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}

/* Test Mode ******************************************************************/
static int  melfas_mcs8000_test_mode(uint8_t test_mode)
{
    uint8_t     buff[2] ;

    buff[0] = 0xA1 ;        /* register address */
    buff[1] = test_mode ;
    if (melfas_mcs7000_i2c_write(melfas_mcs7000_ts->client, buff, 2) != 0)
    {
        printk(KERN_ERR "can not enter the test(or normal) mode 0x%02x\n", test_mode) ;
        return -1 ;
    }

    return 0 ;
}

#define MELFAS_ENTER_TEST_MODE(_mode_)          melfas_mcs8000_test_mode(_mode_)
#define MELFAS_EXIT_TEST_MODE()                 melfas_mcs8000_test_mode(0x00)

/* Touch key Sensitivity ******************************************************/
static ssize_t tkey_sensitivity_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t     tkey_sens[4] ;

    if (MELFAS_ENTER_TEST_MODE(0x03) < 0)
        goto error_return ;

    msleep(20) ;

    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_TKEY_RT_STRNTH_REG, tkey_sens, 4) != 0)
    {
        printk(KERN_ERR "tkey_sensitivity_read read fail!!!\n") ;
        goto error_return ;
    }
    printk(KERN_DEBUG "tkey_sens : %d %d %d %d\n", tkey_sens[0], tkey_sens[1], tkey_sens[2], tkey_sens[3]) ;

    MELFAS_EXIT_TEST_MODE() ;

    /* tkey_touched = (1, 2, 3, 4) */
    return sprintf(buf, "%d %d %d %d", tkey_sens[0], tkey_sens[1], tkey_sens[2], tkey_sens[3]) ;

error_return :
    return sprintf(buf, "-1") ;
}

/* Noise Threshold : Touch Key default Sensitivity ****************************/
static ssize_t tkey_noise_thd_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   noise_thd ;

    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, MCSTS_TKEY_THD_REG, &noise_thd, 1) == 0)
    {
        noise_thd &= 0x1F ;
        printk("TKEY_THRESHOLD : %02d\n", noise_thd) ;
        return sprintf(buf, "%d", noise_thd) ;
    }
    /* else */
    printk(KERN_ERR "TKEY_THRESHOLD : error\n") ;
    return sprintf(buf, "0") ;  // TODO:
}

/* Touch Reference ************************************************************/
static ssize_t reference_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   v, exciting_ch, sensing_ch, exct_ch ;
    unsigned char   buff[20],buf1[3] ;
    int             written_bytes = 0,  /* & error check */
                    ref_value ;

    printk(KERN_DEBUG "%s\n", __func__) ;
    /* sensing channle */
    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x3c, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(sensing_ch)\n") ;
        goto error_ret ;
    }
    sensing_ch = v & 0x1F ;

    /* exciting channel */
    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x3d, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(exciting_ch)\n") ;
        goto error_ret ;
    }
    exciting_ch = v & 0x1F ;
    printk(KERN_DEBUG "sensing_ch = %d, exciting_ch = %d\n", sensing_ch, exciting_ch) ;

    /* disable TSP_IRQ */
    disable_irq(melfas_mcs7000_ts->irq);

    /* enter test mode 0x02 */
    //if (MELFAS_ENTER_TEST_MODE(0x02) < 0)
    //    goto enable_irq_ret ;

    /* read reference data */
    for (exct_ch = 0 ; exct_ch < exciting_ch ; exct_ch++)
    {
        int     retry_n ;
        
        buf1[0] = 0xA1 ;		/* register address */			
        buf1[1] = 0x02 ;			
        buf1[2] = exct_ch;			
        if (melfas_mcs7000_i2c_write(melfas_mcs7000_ts->client, buf1, 3) != 0)			
        {				
        	printk(KERN_ERR "Failed to enter testmode\n") ; 
        }	
        
        /* wait IRQ low */
        for (retry_n = 20 ; retry_n ; retry_n--)
        {
            if (gpio_get_value(GPIO_TOUCH_INT) == 0)
            {
                printk(KERN_DEBUG "TSP_INT low...OK\n") ;
                break ; /* reference data were prepared */
            }
            msleep(1) ;
        }

        if (gpio_get_value(GPIO_TOUCH_INT))
        {
            printk(KERN_ERR "Error INT HIGH!!!\n") ;
            written_bytes = 0 ;
            goto normal_mode_ret ;
        }

        if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x50, buff, 20) != 0)
        {
            printk(KERN_ERR "Failed to read the reference data.\n") ;
            written_bytes = 0 ;
            goto normal_mode_ret ;
        }

        printk(KERN_DEBUG "EXCITING_CH=%d ", exct_ch) ;
        for (v = 0 ; v < sensing_ch ; v++)
        {
            ref_value = (buff[2*v] << 8) | buff[2*v+1] ;
            written_bytes += sprintf(buf+written_bytes, " %d", ref_value) ;
            //printk("%d, ", ref_value) ;
        }
//        written_bytes += sprintf(buf+written_bytes, "\n") ;
        //printk("\n") ;
    }

normal_mode_ret :
    /* enter normal mode */
    if (MELFAS_EXIT_TEST_MODE() < 0)
        goto error_ret ;

enable_irq_ret :
    /* enable TSP_IRQ */
    enable_irq(melfas_mcs7000_ts->irq);

    printk(KERN_DEBUG "written = %d\n", written_bytes) ;
    if (written_bytes > 0)
        return written_bytes ;
    /* else */
error_ret :
    return sprintf(buf, "-1") ; // TODO:RECHECK with Platform App
}

/* Touch Reference ************************************************************/
static ssize_t raw_store_mcs7000(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	if(strncasecmp(buf, "start", 5) == 0)
	{
        unsigned char   data ;
        /* disable TSP_IRQ */
        disable_irq(melfas_mcs7000_ts->irq);

        /* enter test mode 0x04 */
        if (MELFAS_ENTER_TEST_MODE(0x04) < 0)   /* Document erratum : not 0xB4 */
        {
            debugprintk(5, "TSP TEST MODE(0x04) Error!!!\n") ;
            return size ;
        }

        if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x00, &data, 1) != 0)
        {
            debugprintk(5, "TSP TEST MODE(0x04) Dummy Read Error!!!\n") ;
        }
	}

	else if(strncasecmp(buf, "stop", 4) == 0)
	{
        /* enter normal mode */
        if (MELFAS_EXIT_TEST_MODE() < 0)
        {
            debugprintk(5, "TSP TEST MODE(0x00) Error!!!\n") ;
            return size ;
        }

        mcsdl_vdd_off();
        mdelay(200);
        mcsdl_vdd_on();
        printk("[TOUCH] reset.\n");
        mdelay(700);

        /* enable TSP_IRQ */
        enable_irq(melfas_mcs7000_ts->irq);
	}
    else
    {
        debugprintk(5, "TSP Error Unknwon commad!!!\n") ;
    }

    return size ;
}
static ssize_t raw_show_mcs7000(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   data[3*14] ;    /* R40_V19 [RAW(2) + DIFF(1)] x 14 */
    int             written_bytes = 0 ;  /* & error check */
    int             n_ch_sens, n, n_wait ;
    unsigned short  raw ;
    signed char     diff ;

    printk(KERN_DEBUG "%s\n", __func__) ;

    for (n_ch_sens = 0 ; n_ch_sens < 10 ; n_ch_sens++)
    {
        /* wait IRQ low */
        for (n_wait = 50 ; n_wait ; n_wait--)  // TODO:YUNG_RECHECK < 5 msec
        {
            if (gpio_get_value(GPIO_TOUCH_INT) == 0)
            {
                // printk(KERN_DEBUG "TSP_INT low...OK\n") ;
            break ; /* reference data were prepared */
        }
        msleep(1) ;
    }

    if (gpio_get_value(GPIO_TOUCH_INT))
    {
        printk(KERN_ERR "Error INT HIGH!!!\n") ;
        written_bytes = 0 ;
        goto normal_ret ;
    }

    /* read raw(2)+reference(2)+Intensity(1) data x (5x9) */
    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x00, data, sizeof(data)) != 0)
    {
        printk(KERN_ERR "Failed to read the raw+ref+intensity data.\n") ;
        written_bytes = 0 ;
        goto normal_ret ;
    }

        /* 2 ~ 5msec */
        for (n_wait = 5 ; gpio_get_value(GPIO_TOUCH_INT) == 1 && n_wait >= 0 ; n_wait--)
        {
           // printk(KERN_DEBUG "TSP_INT is HIGH!...wait for LOW..\n") ;
            msleep(1) ;
        }

        for (n = 0 ; n < 14 ; n++)
        {
            /* assume big-endian */
            raw = (data[3*n] << 8) | data[3*n+1] ;
            diff = (signed char)data[3*n+2] ;
            written_bytes += sprintf(buf+written_bytes, "%d %d\n", raw, diff) ;
           // printk("[%2d][%2d] : %4d %+3d\n",n_ch_sens, n, raw, diff) ;
        }
    }

normal_ret :
    /* enable TSP_IRQ */
    // TODO:R20_V98 enable_irq(melfas_mcs7000_ts->irq);

    printk(KERN_DEBUG "written = %d\n", written_bytes) ;
    if (written_bytes > 0)
        return written_bytes ;
    /* else */
    return sprintf(buf, "-1") ; // TODO:RECHECK with Platform App
}

#ifdef TSP_TEST_MODE

//****************************************************************************
//
// Function Name:   ts_melfas_test_mode
//
// Description:     
//
// Notes: 
//
//****************************************************************************

static ssize_t tsp_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "MELFAS,MMS-128\n");
}

static ssize_t tsp_channel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   v,exciting_ch, sensing_ch ;

    printk(KERN_DEBUG "%s\n", __func__) ;
    /* sensing channle */
    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x3c, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(sensing_ch)\n") ;
    }
    sensing_ch = v & 0x1F ;

    /* exciting channel */
    if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x3d, &v, 1) != 0)
    {
        printk(KERN_ERR "Failed to read(exciting_ch)\n") ;
    }
    exciting_ch = v & 0x1F ;
    printk(KERN_DEBUG "sensing_ch = %d, exciting_ch = %d\n", sensing_ch, exciting_ch) ;
	return sprintf(buf, "%d,%d\n",sensing_ch,exciting_ch);

}

static ssize_t tsp_test_reference_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int j,i,ref_value;
	uint8_t buf1[2],buff[20];

	int k =refer_y_channel_num-1;

	printk(KERN_DEBUG "Reference START %s\n", __func__) ;
	
	/* disable TSP_IRQ */
	disable_irq(melfas_mcs7000_ts->irq);
	buf1[0] = 0xA1 ;		/* register address */			
	buf1[1] = 0x02 ;			
	buf1[2] = refer_y_channel_num-1;			
	if (melfas_mcs7000_i2c_write(melfas_mcs7000_ts->client, buf1, 3) != 0)			
	{				
		printk(KERN_ERR "Failed to enter testmode\n") ;	
	}		

	mdelay(350);


	if(melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0x50, buff, 20)!= 0)
	{
		printk(KERN_ERR "Failed to read(referece data)\n") ;
	}

	mdelay(4);
	
	for ( i = 0; i < 10; i++ )
	{	
		ref_value = (buff[2*i] << 8) | buff[2*i+1] ;
		tsp_test_reference[i] = ref_value;	    
	}
			
	mcsdl_vdd_off();
	mdelay(50);
	mcsdl_vdd_on();
	mdelay(250);
	printk("[TOUCH] reset.\n");	
	/* enable TSP_IRQ */
	enable_irq(melfas_mcs7000_ts->client->irq);
	
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d \n", 
	tsp_test_reference[0],tsp_test_reference[1],tsp_test_reference[2],tsp_test_reference[3],
	tsp_test_reference[4],tsp_test_reference[5],tsp_test_reference[6],tsp_test_reference[7],
	tsp_test_reference[8],tsp_test_reference[9]);
}

static ssize_t tsp_test_reference_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{	
	unsigned int position;
	int ret;
	
	sscanf(buf,"%d\n",&position);

	if (position < 0 || position >= 15) {
		printk(KERN_DEBUG "Invalid values\n");
		return -EINVAL; 	   
	}
	refer_y_channel_num = (uint8_t)position;

	return size;
}

static ssize_t tsp_test_inspection_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i, j,k,ret,retry_n;
	unsigned char buf1[6],buf2[2];
	int written_bytes=0; 
	k = inspec_y_channel_num-1;
	
	printk(KERN_DEBUG "Inspection START %s,%d\n", __func__,inspec_y_channel_num) ;
	/* disable TSP_IRQ */	
	disable_irq(melfas_mcs7000_ts->client->irq);

	buf1[0] = 0xA0;
	buf1[1] = 0x1A;
	buf1[2] = 0x0;
	buf1[3] = 0x0;
	buf1[4] = 0x0;
	buf1[5] = 0x01;	// start flag
	ret = melfas_mcs7000_i2c_write(melfas_mcs7000_ts->client, buf1, 6);

	
	for (retry_n = 1000 ; retry_n ; retry_n--)
	{
		if (gpio_get_value(GPIO_TOUCH_INT) == 0)
		{
			//printk(KERN_DEBUG "TSP_INT low...OK1  delay= %d ms \n",1000-retry_n) ;
			break ; /* reference data were prepared */
		}
		msleep(1) ;
	}


	if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0xA1, buf2, 2) != 0)
	{
		printk(KERN_ERR "Failed to read(dummy value)\n") ;
	}


	for ( j = 0; j < 14; j++ )
	{
		for ( i = 0; i < 10; i++ )
		{	
			buf1[0] = 0xA0;
			buf1[1] = 0x1A;
			buf1[2] = j;		// exciting ch
			buf1[3] = i;		// sensing ch
			buf1[4] = 0x0;		// reserved
			buf1[5] = 0x02;	// start flag, 2: output inspection data, 3: output low data
			ret = melfas_mcs7000_i2c_write(melfas_mcs7000_ts->client, buf1, 6);

			for (retry_n = 20 ; retry_n ; retry_n--)
			{
				if (gpio_get_value(GPIO_TOUCH_INT) == 0)
				{
					//printk(KERN_DEBUG "TSP_INT low...OK2 delay= %d ms\n",20-retry_n) ;
					break ; /* reference data were prepared */
				}
					msleep(1) ;
			}

			if (melfas_mcs7000_i2c_read(melfas_mcs7000_ts->client, 0xA8, buf2, 2) != 0)
			{
				printk(KERN_ERR "Failed to read(inspection value)\n") ;
			}
			tsp_test_inspection[j][i]= ((buf2[0] << 8) | buf2[1]);
			if(inspec_y_channel_num == 140)
			{
				written_bytes += sprintf(buf+written_bytes, "%d, ", tsp_test_inspection[j][i]) ;
				//printk("%d, ", tsp_test_inspection[j][i]) ;
			}
		}
			//printk(" \n") ;
	}
	printk(KERN_DEBUG "written = %d\n", written_bytes) ;

	mcsdl_vdd_off();
	msleep(50);
	mcsdl_vdd_on();
	msleep(250);
	printk("[TOUCH] reset.\n");

	/* enable TSP_IRQ */
	enable_irq(melfas_mcs7000_ts->client->irq);
    if(inspec_y_channel_num == 140)
    {
    	printk("[INSPECTION ALL]\n");
    	return written_bytes;
    }
    else
    {
    	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n", 
    		tsp_test_inspection[k][0],tsp_test_inspection[k][1],tsp_test_inspection[k][2],tsp_test_inspection[k][3],
    		tsp_test_inspection[k][4],tsp_test_inspection[k][5],tsp_test_inspection[k][6],tsp_test_inspection[k][7],
    		tsp_test_inspection[k][8],tsp_test_inspection[k][9]);
    }
}

static ssize_t tsp_test_inspection_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	unsigned int position;
	int ret;
	
	sscanf(buf,"%d\n",&position);
	if(position == 140)
	inspec_y_channel_num = (uint8_t)position;
	else {
	if (position < 0 || position >= 15) {
		printk(KERN_DEBUG "Invalid values\n");
		return -EINVAL; 	   
	}

	inspec_y_channel_num = (uint8_t)position;
	}
	return size;
}
#endif

static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show_mcs7000, gpio_store_mcs7000);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show_mcs7000, registers_store_mcs7000);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR | S_IWGRP, firmware_show_mcs7000, firmware_store_mcs7000);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show_mcs7000, debug_store_mcs7000);
static DEVICE_ATTR(tkey_sensitivity, S_IRUGO, tkey_sensitivity_show_mcs7000, NULL) ;
static DEVICE_ATTR(tkey_sens_pline, S_IRUGO, tkey_sensitivity_show_mcs7000, NULL) ;
static DEVICE_ATTR(tkey_noise_thd, S_IRUGO, tkey_noise_thd_show_mcs7000, NULL) ;
static DEVICE_ATTR(reference, S_IRUGO, reference_show_mcs7000, NULL) ;
static DEVICE_ATTR(raw, S_IRUGO | S_IWUSR | S_IWGRP, raw_show_mcs7000, raw_store_mcs7000) ;
#ifdef TSP_TEST_MODE
static DEVICE_ATTR(tsp_name, S_IRUGO , tsp_name_show, NULL) ;
static DEVICE_ATTR(tsp_channel, S_IRUGO , tsp_channel_show, NULL) ;
static DEVICE_ATTR(tsp_reference, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_reference_show, tsp_test_reference_store) ;
static DEVICE_ATTR(tsp_inspection, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_inspection_show, tsp_test_inspection_store) ;
#endif

void melfas_mcs7000_upgrade(INT32 hw_ver)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	
	printk("[TOUCH] Melfas	H/W version: 0x%02x.\n", melfas_mcs7000_ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs7000_ts->fw_ver);

	printk("[F/W D/L] Entry gpio_tlmm_config\n");
	
	printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
	ret = mcsdl_download_binary_data(melfas_mcs7000_ts->hw_rev); 
	
	melfas_mcs7000_read_version();
		
	if(ret > 0){
			if (melfas_mcs7000_ts->hw_rev < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");;
			}
			
			if (melfas_mcs7000_ts->fw_ver < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");
			}
			
			printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs7000_ts->hw_rev, melfas_mcs7000_ts->fw_ver);

	}
	else {
		printk("[TOUCH] Firmware update failed.. RESET!\n");
  		mcsdl_vdd_off();
		mdelay(500);
  		mcsdl_vdd_on();
		mdelay(200);
	}
#endif
}

void meflas_mcs7000_reset( void )
{ 
	mcsdl_vdd_off();
	gpio_set_value(GPIO_I2C0_SCL, 0);	// TOUCH SCL DIS
	gpio_set_value(GPIO_I2C0_SDA, 0);	// TOUCH SDA DIS
	msleep(200);
	
	gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
	gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN	  
	mcsdl_vdd_on();
	msleep(300);
}


static int melfas_mcs7000_process_key(int key, int state)
{
	melfas_mcs7000_ts->info.keycode = key;
	int	key_event = 0;
	if(key == 0x1)
	{
	  key_event = TOUCH_MENU;
	}
	else if(key == 0x2)
	{
	  key_event = TOUCH_HOME;
	}
	else if(key == 0x3)
	{
	  key_event = TOUCH_BACK;
	}
	else if(key == 0x4)
	{
	  key_event = TOUCH_SEARCH;
	}
	else
	{
	  enable_irq(melfas_mcs7000_ts->irq);
	  return 0;
	}
	
	debugprintk(5,"[TOUCH_KEY] key_event = [%d], touchaction = [%d] \n", key_event, state);		
	input_report_key(melfas_mcs7000_ts->input_dev, key_event, state);
	
#if defined(CONFIG_MACH_ESCAPE)
	keypad_backlight_control();
#endif
	return 1;
}

void melfas_mcs7000_ts_work_func(struct work_struct *work)
{
  int ret;
  int ret1; 
  int key_ret;
  struct i2c_msg msg[2];  
  uint8_t start_reg;
  uint8_t buf1[13];

  msg[0].addr = melfas_mcs7000_ts->client->addr;
  msg[0].flags = 0; 
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = MCSTS_INPUT_INFO_REG;
  msg[1].addr = melfas_mcs7000_ts->client->addr;
  msg[1].flags = I2C_M_RD; 
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;
  
  ret  = i2c_transfer(melfas_mcs7000_ts->client->adapter, &msg[0], 1);
  ret1 = i2c_transfer(melfas_mcs7000_ts->client->adapter, &msg[1], 1);

  if((ret < 0) ||  (ret1 < 0)) 
  	{
  		printk(KERN_ERR "==melfas_mcs7000_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
  		meflas_mcs7000_reset();
		enable_irq(melfas_mcs7000_ts->irq); 
	}
  else
  	{    
	    int x = buf1[2] | ((uint16_t)(buf1[1] & 0x0f) << 8); 
	    int y = buf1[3] | (((uint16_t)(buf1[1] & 0xf0) >> 4) << 8); 
	    int z = buf1[5];
		
		int x2 = buf1[8] | ((uint16_t)(buf1[7] & 0x0f) << 8); 
	    int y2 = buf1[9] | (((uint16_t)(buf1[7] & 0xf0) >> 4) << 8); 
	    int z2 = buf1[11];
		
	    int key_value =(int)( buf1[12] & 0x07); //key value
	    
	    int finger1_state = buf1[0] & 0x01;
		int finger2_state = buf1[6] & 0x01;
	    int key_state = (int)((buf1[12] >> 3) & 0x01); //key_status
	    		
		#ifdef CONFIG_CPU_FREQ
	    set_dvfs_perf_level();
		#endif

		int do_report1 = false; 
		int do_report2 = false;
		static int key_release = 0;

		if( buf1[0] == 0x7F ) //for ESD
		{
		
  			printk(KERN_ERR "--TSP: ESD !!!!!!!!!!!\n");
			melfas_mcs7000_ts_gen_touch_up();
			meflas_mcs7000_reset();
		}else
		{
			if(key_value!=0 && finger1_state==0 && finger2_state==0){
				if(key_state)
					key_release = 0;
				else
					key_release = 1;
				
				key_ret = melfas_mcs7000_process_key(key_value , key_state);
				if(key_ret==0)
				return;
			}
			else {
				if(key_value != 0){
					if(key_state==0 && key_release==0){
						key_release = 1;
						key_ret = melfas_mcs7000_process_key(key_value , key_state);
						if(key_ret==0)
						return;
					}
					else{
						enable_irq(melfas_mcs7000_ts->irq);
					 	return;		
					}
				}	
				
				if(finger1_state){
					melfas_mcs7000_ts->info.x = x;
					melfas_mcs7000_ts->info.y = y;
					melfas_mcs7000_ts->info.z = z;
					melfas_mcs7000_ts->info.finger_id = 1; 
					do_report1 = true;
				}
				if(finger2_state){
					melfas_mcs7000_ts->info.x2 = x2;
					melfas_mcs7000_ts->info.y2 = y2;
					melfas_mcs7000_ts->info.z2 = z2;
					melfas_mcs7000_ts->info.finger_id = 2; 	  
					do_report2 = true;
				}

				if(finger1_state == 0 && finger2_state ==0 ){
					melfas_mcs7000_ts->info.x = -1;
					melfas_mcs7000_ts->info.y = -1;
					melfas_mcs7000_ts->info.z = -1;
					melfas_mcs7000_ts->info.x2 = -1;
					melfas_mcs7000_ts->info.y2 = -1;
					melfas_mcs7000_ts->info.z2 = -1; 		  
					melfas_mcs7000_ts->info.finger_id = -1; 
					z2 = 0;
					do_report1 = false;
					do_report2 = false;
				}
					  	
				if(do_report1) {
					debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger1_state);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, y);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z);		
					//input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TRACKING_ID, 1);		
					input_mt_sync(melfas_mcs7000_ts->input_dev);			
				}

				if(do_report2) {
					debugprintk(5,"[TOUCH_MT2] x2: %4d, y2: %4d, z2: %4d, finger: %4d,\n", x2, y2, z2, finger2_state);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, x2);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, y2);
					input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z2);	  
					//input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TRACKING_ID, 2);	  
					input_mt_sync(melfas_mcs7000_ts->input_dev);
				}
				
				if(finger1_state ==0 & finger2_state==0)	  	
				input_mt_sync(melfas_mcs7000_ts->input_dev);				
			}			
	  	  input_sync(melfas_mcs7000_ts->input_dev);
		  
		  melfas_mcs7000_ts->info.state = (finger1_state | (finger2_state << 1) |(key_state << 2));
		  printk(KERN_DEBUG "[TOUCH] finger_id = [%d], finger_state = [%d] \n", melfas_mcs7000_ts->info.finger_id, melfas_mcs7000_ts->info.state);					  
		  
  		}
  		enable_irq(melfas_mcs7000_ts->irq);
  	}  
}

irqreturn_t melfas_mcs7000_ts_irq_handler(int irq, void *dev_id)
{
	disable_irq_nosync(melfas_mcs7000_ts->irq);
	//disable_irq(melfas_mcs7000_ts->irq);
	queue_work(melfas_mcs7000_ts_wq, &melfas_mcs7000_ts->work);
	return IRQ_HANDLED;
}

int melfas_mcs7000_ts_probe(void)
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	//int fw_ver = 0;
	//int hw_rev = 0;
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] PROBE       =========");
	printk("\n====================================================\n");


	if (!i2c_check_functionality(melfas_mcs7000_ts->client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		printk(KERN_ERR "melfas_mcs7000_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	INIT_WORK(&melfas_mcs7000_ts->work, melfas_mcs7000_ts_work_func);
	
	melfas_mcs7000_read_version(); 
	// firmware updgrade
	if((melfas_mcs7000_ts->fw_ver!=0x11 && melfas_mcs7000_ts->hw_rev==0x12)
		|| (melfas_mcs7000_ts->fw_ver!=0x15 && melfas_mcs7000_ts->hw_rev==0x20)
		|| (melfas_mcs7000_ts->fw_ver!=0x15 && melfas_mcs7000_ts->hw_rev==0x30)
        || (melfas_mcs7000_ts->fw_ver!=0x24 && melfas_mcs7000_ts->hw_rev==0x40)
        || (melfas_mcs7000_ts->fw_ver!=0x25 && melfas_mcs7000_ts->hw_rev==0x50))
		melfas_mcs7000_upgrade(melfas_mcs7000_ts->hw_rev);

	printk(KERN_INFO "[TOUCH] Melfas  H/W version: 0x%x.\n", melfas_mcs7000_ts->hw_rev);
	printk(KERN_INFO "[TOUCH] Current F/W version: 0x%x.\n", melfas_mcs7000_ts->fw_ver);
	
	//melfas_mcs7000_read_resolution(); 
	max_x = 320; //melfas_mcs7000_ts->info.max_x ;
	max_y = 480; //melfas_mcs7000_ts->info.max_y ;
	printk("melfas_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	melfas_mcs7000_ts->input_dev = input_allocate_device();
	if (melfas_mcs7000_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_mcs7000_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_mcs7000_ts->input_dev->name = "sec_touchscreen";

	set_bit(EV_SYN, melfas_mcs7000_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_mcs7000_ts->input_dev->evbit);
	set_bit(TOUCH_HOME, melfas_mcs7000_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_mcs7000_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_mcs7000_ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, melfas_mcs7000_ts->input_dev->keybit);

	melfas_mcs7000_ts->input_dev->keycode = melfas_ts_tk_keycode;	
	set_bit(BTN_TOUCH, melfas_mcs7000_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_mcs7000_ts->input_dev->evbit);

	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	printk("melfas_mcs7000_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	ret = input_register_device(melfas_mcs7000_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_mcs7000_ts_probe: Unable to register %s input device\n", melfas_mcs7000_ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	melfas_mcs7000_ts->irq = melfas_mcs7000_ts->client->irq; //add by KJB
	ret = request_irq(melfas_mcs7000_ts->client->irq, melfas_mcs7000_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	if(ret == 0) {
		printk(KERN_INFO "melfas_mcs7000_ts_probe: Start touchscreen %s \n", melfas_mcs7000_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_mcs7000_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_mcs7000_ts->early_suspend.suspend = melfas_mcs7000_ts_early_suspend;
	melfas_mcs7000_ts->early_suspend.resume = melfas_mcs7000_ts_late_resume;
	register_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	return 0;
err_misc_register_device_failed:
err_input_register_device_failed:
	input_free_device(melfas_mcs7000_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_mcs7000_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

int melfas_mcs7000_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs7000_ts->irq, 0);
	input_unregister_device(melfas_mcs7000_ts->input_dev);
	return 0;
}

int melfas_mcs7000_ts_gen_touch_up(void)
{
  // report up key if needed
  int finger_state = melfas_mcs7000_ts->info.state;
  
  if(finger_state != 0x0) //down state
  {
  		if(finger_state==0x1 || finger_state==0x3)
		{
			int x = melfas_mcs7000_ts->info.x;
			int y = melfas_mcs7000_ts->info.y;
			int z = 0;
			debugprintk(5,"[TOUCH_1] GENERATE UP KEY x: %4d, y: %4d, z: %4d\n", x, y, z);
			if (x)	input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, x);
			if (y)	input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z);	
			input_mt_sync(melfas_mcs7000_ts->input_dev);
		}
	  
		if(finger_state==0x2 || finger_state==0x3)
		{
			int x2 = melfas_mcs7000_ts->info.x2;
			int y2 = melfas_mcs7000_ts->info.y2;
			int z2 = 0;
			debugprintk(5,"[TOUCH_2] GENERATE UP KEY x2: %4d, y2: %4d, z2: %4d\n", x2, y2, z2);
			if (x2) input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_X, x2);
			if (y2) input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_POSITION_Y, y2);
			input_report_abs(melfas_mcs7000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z2); 
			input_mt_sync(melfas_mcs7000_ts->input_dev);		
		}

		if(finger_state == 0x4)
		{			
			debugprintk(5,"[TOUCH_KEY] GENERATE UP KEY keyevent=%d \n",melfas_mcs7000_ts->info.keycode);
			melfas_mcs7000_process_key(melfas_mcs7000_ts->info.keycode,0);
		}
	  }
		melfas_mcs7000_ts->info.state = 0x0;
		input_sync(melfas_mcs7000_ts->input_dev);   
}

int melfas_mcs7000_ts_suspend(pm_message_t mesg)
{
  melfas_mcs7000_ts->suspended = true;
 
  if(!work_pending(&melfas_mcs7000_ts->work)){
    disable_irq(melfas_mcs7000_ts->irq);
  }
  cancel_work_sync(&melfas_mcs7000_ts->work); 
  melfas_mcs7000_ts_gen_touch_up();  
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  
  mcsdl_vdd_off();
  gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
  gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
  
  return 0;
}

static void ts_resume_work_func(struct work_struct *ignored);
static DECLARE_DELAYED_WORK(ts_resume_work, ts_resume_work_func);

static void ts_resume_work_func(struct work_struct *ignored)
{
	melfas_mcs7000_ts->suspended = false;
	enable_irq(melfas_mcs7000_ts->irq);
}

int melfas_mcs7000_ts_resume()
{
	//gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	//gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	//gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
	//gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
	gpio_direction_output( GPIO_I2C0_SCL , 1 ); 
	gpio_direction_output( GPIO_I2C0_SDA , 1 ); 	
	mcsdl_vdd_on();
	//msleep(300);
	schedule_delayed_work(&ts_resume_work, 30 );//300 msec
	
	return 0;
}
#if 0 // blocked for now.. we will gen touch when suspend func is called
int tsp_preprocess_suspend(void)
{
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_mcs7000_ts->suspended == false) {  
    // fake as suspended
    melfas_mcs7000_ts->suspended = true;
    
    //generate and report touch event
    melfas_mcs7000_ts_gen_touch_up();
  }
  return 0;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs7000_ts_early_suspend(struct early_suspend *h)
{
	melfas_mcs7000_ts_suspend(PMSG_SUSPEND);
}

void melfas_mcs7000_ts_late_resume(struct early_suspend *h)
{
	melfas_mcs7000_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


int melfas_mcs7000_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_mcs7000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs7000_ts);
	return 0;
}

static int __devexit melfas_mcs7000_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs7000_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs7000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs7000_ts->input_dev);
   
	melfas_mcs7000_ts = i2c_get_clientdata(client);
	kfree(melfas_mcs7000_ts);
	return 0;
}

struct i2c_device_id melfas_mcs7000_id[] = {
	{ "mcs8000_i2c", 0 },
	{ }
};

struct i2c_driver mcs7000_ts_i2c = {
	.driver = {
		.name	= "mcs8000_i2c",
		.owner	= THIS_MODULE,
	},
	.probe 		= melfas_mcs7000_i2c_probe,
	.remove		= __devexit_p(melfas_mcs7000_i2c_remove),
	.id_table	= melfas_mcs7000_id,
};


void init_hw_setting_mcs7000(void)
{
	int ret;

	vreg_touch = vreg_get(NULL, TOUCH_LDO); /* VTOUCH_2.8V */
    vreg_set_level(vreg_touch, TOUCH_LDO_LEVEL);
	ret = vreg_enable(vreg_touch);
	if (ret) { 
		printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
		return;//-EIO;
	}
	else {
		printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
	}

	mdelay(100);
	msleep(250);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_EDGE_FALLING);

	mdelay(10);

}

struct platform_driver mcs7000_ts_driver =  {
	.probe	= melfas_mcs7000_ts_probe,
	.remove = melfas_mcs7000_ts_remove,
	.driver = {
		.name = "mcs8000-ts",
		.owner	= THIS_MODULE,
	},
};


int __init melfas_mcs7000_ts_init(void)
{
	int ret;

	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] INIT        =========");
	printk("\n====================================================\n");

	init_hw_setting_mcs7000();

	mcs7000_ts_dev = device_create(sec_class_1, NULL, 0, NULL, "ts");
	
	if (IS_ERR(mcs7000_ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(mcs7000_ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);

	if (device_create_file(mcs7000_ts_dev, &dev_attr_tkey_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sensitivity.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_tkey_sens_pline) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sens_pline.attr.name);
    if (device_create_file(mcs7000_ts_dev, &dev_attr_tkey_noise_thd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_noise_thd.attr.name);
    if (device_create_file(mcs7000_ts_dev, &dev_attr_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_reference.attr.name);
    if (device_create_file(mcs7000_ts_dev, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
#ifdef TSP_TEST_MODE
	if (device_create_file(mcs7000_ts_dev, &dev_attr_tsp_name) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_name.attr.name); 
	if (device_create_file(mcs7000_ts_dev, &dev_attr_tsp_channel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_name.attr.name); 	
	if (device_create_file(mcs7000_ts_dev, &dev_attr_tsp_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_reference.attr.name);
	if (device_create_file(mcs7000_ts_dev, &dev_attr_tsp_inspection) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_inspection.attr.name);
#endif	

	melfas_mcs7000_ts = kzalloc(sizeof(struct mcs7000_ts_driver), GFP_KERNEL);
	if(melfas_mcs7000_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&mcs7000_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	if(!melfas_mcs7000_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&mcs7000_ts_i2c);
		return 0;
	}
	melfas_mcs7000_ts_wq = create_singlethread_workqueue("melfas_mcs7000_ts_wq");
	if (!melfas_mcs7000_ts_wq)
		return -ENOMEM;

	return platform_driver_register(&mcs7000_ts_driver);

}

void __exit melfas_mcs7000_ts_exit(void)
{
	i2c_del_driver(&mcs7000_ts_i2c);

	if (melfas_mcs7000_ts_wq)
		destroy_workqueue(melfas_mcs7000_ts_wq);
}
late_initcall(melfas_mcs7000_ts_init);
// module_init(melfas_mcs7000_ts_init);
module_exit(melfas_mcs7000_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
