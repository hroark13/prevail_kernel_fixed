/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

//PCAM 1/5" s5k5cagx

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>


#include "s5k5cagx.h"
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>


#define SENSOR_DEBUG 0


//#define CONFIG_LOAD_FILE

#define CAM_RESET 0
#define CAM_STANDBY 1
#define CAM_EN 2
#define CAM_I2C_SCL 60
#define CAM_I2C_SDA 61

#define S5K5CAGX_BURST_WRITE_LIST(A)	s5k5cagx_sensor_burst_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#define S5K5CAGX_WRITE_LIST(A)			s5k5cagx_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);



static char first_start_camera = 1;//  1 is not init a sensor

static char mEffect = 0;
static char mBrightness = 0;
static char mContrast = 0;
static char mSaturation = 0;
static char mSharpness = 0;
static char mWhiteBalance = 0;
static char mISO = 0;
static char mAutoExposure = 0;
static char mScene = 0;
static char mAfMode = 0;
static char mDTP = 0;
static char mInit = 0;
static char mMode = 0;
static int flash_mode = 1; //kkd temp


struct s5k5cagx_work {
	struct work_struct work;
};

static struct  s5k5cagx_work *s5k5cagx_sensorw;
static struct  i2c_client *s5k5cagx_client;

struct s5k5cagx_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct s5k5cagx_ctrl *s5k5cagx_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(s5k5cagx_wait_queue);
DECLARE_MUTEX(s5k5cagx_sem);
static int16_t s5k5cagx_effect = CAMERA_EFFECT_OFF;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5k5cagx_reg s5k5cagx_regs;
extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
extern void pcam_msm_i2c_pwr_mgmt(struct i2c_adapter *adap, int on);
/*=============================================================*/

static int cam_hw_init(void);

void sensor_DTP_control(char value);

#ifdef CONFIG_LOAD_FILE
	static int s5k5cagx_regs_table_write(char *name);
#endif





static int s5k5cagx_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { s5k5cagx_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static int s5k5cagx_sensor_read_multi(unsigned short subaddr, unsigned long *data)
{
	unsigned char buf[4];
	struct i2c_msg msg = {s5k5cagx_client->addr, 0, 2, buf};

	int ret;

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	msg.len = 4;

	ret = i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	/*
	 * [Arun c]Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = ((buf[0] << 8) | (buf[1] ) | (buf[2] <<24) | buf[3] <<16);

error:
	return ret;
}


static int s5k5cagx_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { s5k5cagx_client->addr, 0, 4, buf };

//	printk("[PGH] on write func s5k5cagx_client->addr : %x\n", s5k5cagx_client->addr);
//	printk("[PGH] on write func  s5k5cagx_client->adapter->nr : %d\n", s5k5cagx_client->adapter->nr);


	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

	return i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}



static int s5k5cagx_sensor_write_list(struct s5k5cagx_reg *list,int size, char *name)	
{
	int ret = 0;

#ifdef CONFIG_LOAD_FILE	
	ret = s5k5cagx_regs_table_write(name);
#else
	int i = 0;

	printk("<=PCAM=> s5k5cagx_sensor_write_list : %s\n", name);


	for (i = 0; i < size; i++)
	{
	//	printk("[PGH] %x      %x\n", list[i].subaddr, list[i].value);
		if(list[i].addr == 0xffff)
		{
//			printk("<=PCAM=> now SLEEP!!!!\n  %d", list[i].value);
			msleep(list[i].val);
		}
		else
		{
		    if(s5k5cagx_sensor_write(list[i].addr, list[i].val) < 0)
		    {
			    printk("<=PCAM=> sensor_write_list fail...-_-\n");
			    return -1;
		    }
		}
	}
       printk("<=PCAM=> sensor_write_list success size = %d\n",size);
#endif
	return ret;
}


#define BURST_MODE_BUFFER_MAX_SIZE 2700
unsigned char s5k5cagx_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
static int s5k5cagx_sensor_burst_write_list(struct s5k5cagx_reg *list,int size, char *name)	
{
	int err = -EINVAL;
	int i = 0;
	int idx = 0;

	unsigned short subaddr = 0, next_subaddr = 0;
	unsigned short value = 0;

	struct i2c_msg msg = {  s5k5cagx_client->addr, 0, 0, s5k5cagx_buf_for_burstmode};
	


	for (i = 0; i < size; i++)
	{

		if(idx > (BURST_MODE_BUFFER_MAX_SIZE-10))
		{
			printk("<=PCAM=> BURST MODE buffer overflow!!!\n");
			 return err;
		}



		subaddr = list[i].addr;

		if(subaddr == 0x0F12)
			next_subaddr = list[i+1].addr;

		value = list[i].val;
		

		switch(subaddr)
		{

			case 0x0F12:
			{
				// make and fill buffer for burst mode write
				if(idx ==0) 
				{
					s5k5cagx_buf_for_burstmode[idx++] = 0x0F;
					s5k5cagx_buf_for_burstmode[idx++] = 0x12;
				}
				s5k5cagx_buf_for_burstmode[idx++] = value>> 8;
				s5k5cagx_buf_for_burstmode[idx++] = value & 0xFF;

			
			 	//write in burstmode	
				if(next_subaddr != 0x0F12)
				{
					msg.len = idx;
					err = i2c_transfer(s5k5cagx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					//printk("s5k4ecgx_sensor_burst_write, idx = %d\n",idx);
					idx=0;
				}
				
			}
			break;

			case 0xFFFF:
			{
				msleep(value);
			}
			break;
		
			default:
			{
			    idx = 0;
			    err = s5k5cagx_sensor_write(subaddr, value);
			}
			break;
			
		}

		
	}

	if (unlikely(err < 0))
	{
		printk("<=PCAM=>%s: register set failed\n",__func__);
		return err;
	}
	return 0;

}



static int s5k5cagx_set_preview_start()
{		

	// 1. preview size 
#if 0 //kkd temp
	err = s5k5cagx_set_preview_size(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: Could not set preview size\n", __func__);
		return -EIO;
	}
#endif
      //kkd temp S5K5CAGX_WRITE_LIST(S5K5CAGX_PREVIEW_SIZE_640); //kkd temp	
       
	// 2. DTP or preview start
	if(mDTP == 1) //output Test Pattern
	{	
		printk( "pattern on setting~~~~~~~~~~~~~~\n");
		//kkd temp S5K5CAGX_WRITE_LIST(S5K5CAGX_DTP_ON);
		printk( "pattern on setting done~~~~~~~~~~~~~~\n");	
	}
	
	else //output preview start
	{
#if 0
		// 3. FPS setting
		if(mMode == 0) //in case camera mode, fixed auto_30
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_AUTO30_FPS);
		}
		else
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_30_FPS);
		}
#endif
		// 4. PREVIEW setting
		if (mScene == PCAM_SCENE_NIGHTSHOT || mScene == PCAM_SCENE_FIREWORK) // NIGTSHOT or FIREWORKS PREVIEW
		{
			//kkd temp S5K5CAGX_WRITE_LIST(S5K5CAGX_PREVIEW_NIGHT);
		}
		else // NORMAL PREVIEW
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_PREVIEW);
		}
		
	}
	

	return 0;
}


static unsigned long s5k5cagx_get_illumination()
{
	int err;
	unsigned long read_value;

	s5k5cagx_sensor_write(0xFCFC, 0xD000);
	s5k5cagx_sensor_write(0x002C, 0x7000);
	s5k5cagx_sensor_write(0x002E, 0x2A3C);
	s5k5cagx_sensor_read_multi(0x0F12, &read_value);

	printk("s5k5cagx_get_illumination() : lux_value == 0x%x\n", read_value);

	return read_value;
	
}

static int s5k5cagx_set_flash(int lux_val)
{
#if 0 //kkd temp
	int i = 0;
	int err = 0;

	//printk("%s, flash set is %d\n", __func__, lux_val);

	err = gpio_request(GPIO_CAM_FLASH_SET, "CAM_FLASH_SET");
	if (err) 
	{
		printk(KERN_ERR "failed to request MP04 for camera control\n");
		return err;
	}
	err = gpio_request(GPIO_CAM_FLASH_EN, "CAM_FLASH_EN");
	if (err) 
	{
		printk(KERN_ERR "failed to request MP04 for camera control\n");
		return err;
	}

	if (lux_val == 100)
	{
		//movie mode
		lux_val = 3;
		gpio_direction_output(GPIO_CAM_FLASH_EN, 0);
		for (i = lux_val; i > 1; i--)
		{
//			printk("%s : highlow\n", __func__);
			//gpio on
			gpio_direction_output(GPIO_CAM_FLASH_SET, 1);
			udelay(1);
			//gpio off
			gpio_direction_output(GPIO_CAM_FLASH_SET, 0);
			udelay(1);
		}
		gpio_direction_output(GPIO_CAM_FLASH_SET, 1);
		msleep(2);
	}
	else if (lux_val == 0)
	{
		//flash off
		gpio_direction_output(GPIO_CAM_FLASH_SET, 0);
		gpio_direction_output(GPIO_CAM_FLASH_EN, 0);
	//	err = isx005_i2c_write(sd, isx005_Flash_off, sizeof(isx005_Flash_off) / sizeof(isx005_Flash_off[0]),				"isx005_Flash_off");
	}
	else
	{
		gpio_direction_output(GPIO_CAM_FLASH_EN, 1);
		udelay(20);
		for (i = lux_val; i > 1; i--)
		{
//			printk("%s : highlow\n", __func__);
			//gpio on
			gpio_direction_output(GPIO_CAM_FLASH_SET, 1);
			udelay(1);
			//gpio off
			gpio_direction_output(GPIO_CAM_FLASH_SET, 0);
			udelay(1);
		}
		gpio_direction_output(GPIO_CAM_FLASH_SET, 1);
		msleep(2);
	}
	gpio_free(GPIO_CAM_FLASH_SET);	
	gpio_free(GPIO_CAM_FLASH_EN);
	return err;
#endif
}

static int s5k5cagx_set_flash_mode(int flash_brightness_value, bool value)
{
	unsigned long lux_value;
	
	if(value){
		switch(flash_mode)
			{
				case FLASHMODE_AUTO:
					lux_value = s5k5cagx_get_illumination();
					if (lux_value < 0x0020) 
						s5k5cagx_set_flash(flash_brightness_value);
					break;
					
				case FLASHMODE_ON:
						s5k5cagx_set_flash(flash_brightness_value);
					break;
					
				case FLASHMODE_OFF:
						//err =s5k5cagx_set_flash(flash_brightness_value, sd);
					break;
					
				default:
					break;
			}
		}
	else
	{
		s5k5cagx_set_flash(0);
	}
	
	return 0;
}

static int s5k5cagx_set_capture_start()
{
	unsigned long lux_value;

	// 1. CAPTURE SIZE
#if 0 //kkd temp
	err =s5k5cagx_set_capture_size(sd);
	if(err < 0){
		dev_err(&client->dev, "%s: failed: i2c_write for capture_resolution\n", __func__);
		return -EIO; 
	}
#endif
       S5K5CAGX_WRITE_LIST(S5K5CAGX_CAPTURE_SIZE_2048); //kkd temp

	// 2. FLASH ON
	s5k5cagx_set_flash_mode(1,true);

	// 3. FLASH SET and DELAY
	//kkd temp S5K5CAGX_WRITE_LIST(S5K5CAGX_FLASH_SET);
	
	// 4. CAPTURE START
	if (mScene == PCAM_SCENE_NIGHTSHOT || mScene == PCAM_SCENE_FIREWORK) // NIGTSHOT or FIREWORKS PREVIEW
	{
		S5K5CAGX_WRITE_LIST(S5K5CAGX_NIGHT_SNAPSHOT);
	}
	else // NORMAL CAPTURE
	{
		lux_value = s5k5cagx_get_illumination();

		if(lux_value > 0xFFFE) 		// highlight snapshot
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_HIGH_SNAPSHOT);	
		}
		else if(lux_value > 0x0020) 	// Normalt snapshot
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_NORMAL_SNAPSHOT);		
		}
		else 						//lowlight snapshot
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_LOWLIGHT_SNAPSHOT);
		}
	}

	// 5. FLASH SETING OFF
	s5k5cagx_set_flash_mode(0,false);

	return 0;
}


void sensor_effect_control(char value)
{

	//int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!
	//i2c_clk_addr = 0xd500c004;

	switch(value)
	{
		case PCAM_EFFECT_NORMAL :{
		PCAM_DEBUG("PCAM_EFFECT_NORMAL");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_EFFECT_OFF);
		}
		break;		

		case PCAM_EFFECT_NEGATIVE :{
		PCAM_DEBUG("PCAM_EFFECT_NEGATIVE");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_EFFECT_NEGATIVE);
		}
		break;	
		
		case PCAM_EFFECT_MONO :{
		PCAM_DEBUG("PCAM_EFFECT_MONO");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_EFFECT_MONO);
		}
		break;	

		case PCAM_EFFECT_SEPIA :{
		PCAM_DEBUG("PCAM_EFFECT_SEPIA");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_EFFECT_SEPIA);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected Effect mode : %d\n",  value);
		}
		break;
				
	}

}


void sensor_whitebalance_control(char value)
{
	switch(value)
	{
		case PCAM_WB_AUTO :{
		PCAM_DEBUG("PCAM_WB_AUTO");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_WB_AUTO);
		}
		break;	

		case PCAM_WB_DAYLIGHT :{
		PCAM_DEBUG("PCAM_WB_DAYLIGHT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_WB_DAYLIGHT);
		}
		break;	

		case PCAM_WB_CLOUDY :{
		PCAM_DEBUG("PCAM_WB_CLOUDY");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_WB_CLOUDY);
		}
		break;	

		case PCAM_WB_FLUORESCENT :{
		PCAM_DEBUG("PCAM_WB_FLUORESCENT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_WB_FLUORESCENT);
		}
		break;	
		
		case PCAM_WB_INCANDESCENT :{
		PCAM_DEBUG("PCAM_WB_INCANDESCENT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_WB_INCANDESCENT);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected WB mode : %d\n",  value);
		}
		break;
		
	}// end of switch

}


void sensor_brightness_control(char value)
{
	switch(value)
	{
		case PCAM_BR_STEP_P_4 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_4");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_P_4);
		}
		break;
		
		case PCAM_BR_STEP_P_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_3");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_P_3);
		}
		break;

		case PCAM_BR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_P_2);
		}
		break;

		case PCAM_BR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_P_1);
		}
		break;

		case PCAM_BR_STEP_0 :{
		PCAM_DEBUG("PCAM_BR_STEP_0");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_0);
		}
		break;

		case PCAM_BR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_N_1);
		}
		break;

		case PCAM_BR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_N_2);
		}
		break;

		case PCAM_BR_STEP_M_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_3");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_N_3);
		}
		break;

		case PCAM_BR_STEP_M_4 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_4");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_BRIGHTNESS_N_4);
		}
		break;


		default :{
			printk("<=PCAM=> Unexpected BR mode : %d\n",  value);
		}
		break;

	}
	
}


void sensor_iso_control(char value)
{
	switch(value)
	{
		case PCAM_ISO_AUTO :{
		PCAM_DEBUG("PCAM_ISO_AUTO");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_ISO_AUTO);
		}
		break;

	//	case PCAM_ISO_50 :{
	//	PCAM_DEBUG("PCAM_ISO_50");
	//	S5K5CAGX_WRITE_LIST(S5K5CAGX_ISO_50);
	//	}
	//	break;

		case PCAM_ISO_100 :{
		PCAM_DEBUG("PCAM_ISO_100");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_ISO_100);
		}
		break;

		case PCAM_ISO_200 :{
		PCAM_DEBUG("PCAM_ISO_200");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_ISO_200);
		}
		break;

		case PCAM_ISO_400 :{
		PCAM_DEBUG("PCAM_ISO_400");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_ISO_400);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected ISO mode : %d\n",  value);
		}
		break;
		
	}

}


void sensor_metering_control(char value)
{
	switch(value)
	{
		case PCAM_METERING_NORMAL :{
		PCAM_DEBUG("PCAM_METERING_NORMAL");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_METERING_NORMAL);
		}
		break;
		
		case PCAM_METERING_SPOT :{
		PCAM_DEBUG("PCAM_METERING_SPOT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_METERING_SPOT);
		}
		break;

		case PCAM_METERING_CENTER :{
		PCAM_DEBUG("PCAM_METERING_CENTER");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_METERING_CENTER);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected METERING mode : %d\n",  value);
		}
		break;
	}

}


void sensor_scene_control(char value)
{
	switch(value)
	{
		case PCAM_SCENE_OFF :{
		PCAM_DEBUG("PCAM_SCENE_OFF");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_OFF);
		}
		break;

		case PCAM_SCENE_PORTRAIT :{
		PCAM_DEBUG("PCAM_SCENE_PORTRAIT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_PORTRAIT);
		}
		break;

		case PCAM_SCENE_LANDSCAPE :{
		PCAM_DEBUG("PCAM_SCENE_LANDSCAPE");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_LANDSCAPE);
		}
		break;

		case PCAM_SCENE_SPORTS :{
		PCAM_DEBUG("PCAM_SCENE_SPORTS");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_SPORTS);
		}
		break;

		case PCAM_SCENE_PARTY :{
		PCAM_DEBUG("PCAM_SCENE_PARTY");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_PARTY);			
		}
		break;

		case PCAM_SCENE_BEACH :{
		PCAM_DEBUG("PCAM_SCENE_BEACH");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_BEACH);
		}
		break;

		case PCAM_SCENE_SUNSET :{
		PCAM_DEBUG("PCAM_SCENE_SUNSET");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_SUNSET);
		}
		break;
		
		case PCAM_SCENE_DAWN :{
		PCAM_DEBUG("PCAM_SCENE_DAWN");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_DAWN);	
		}
		break;

//		case PCAM_SCENE_FALL :{
//		PCAM_DEBUG("PCAM_SCENE_FALL");
//		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_FALL);		
//		}
//		break;

	//	case PCAM_SCENE_NIGHTSHOT :{
	//	S5K5CAGX_WRITE_LIST(s5k5cagx_scene_off);
	//	PCAM_DEBUG("PCAM_SCENE_NIGHTSHOT");
	//	S5K5CAGX_WRITE_LIST(s5k5cagx_scene_nightshot);			
	//	}
	//	break;

		case PCAM_SCENE_BACKLIGHT :{
		PCAM_DEBUG("PCAM_SCENE_BACKLIGHT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_BACKLIGHT);
		}
		break;

		case PCAM_SCENE_FIREWORK :{
		PCAM_DEBUG("PCAM_SCENE_FIREWORK");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_FIRE);				
		}
		break;

		case PCAM_SCENE_TEXT :{
		PCAM_DEBUG("PCAM_SCENE_TEXT");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_TEXT);
		}
		break;

		case PCAM_SCENE_CANDLE :{
		PCAM_DEBUG("PCAM_SCENE_CANDLE");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SCENE_CANDLE);	
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected SCENE mode : %d\n",  value);
		}
		break;				
	}
	s5k5cagx_set_preview_start();
}


void sensor_contrast_control(char value)
{
	switch(value)
	{
		case PCAM_CR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_CONTRAST_M_2);
		}
		break;

		case PCAM_CR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_CONTRAST_M_1);	
		}
		break;

		case PCAM_CR_STEP_0 :{
		PCAM_DEBUG("PCAM_CR_STEP_0");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_CONTRAST_0);
		}
		break;

		case PCAM_CR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_CONTRAST_P_1);
		}
		break;

		case PCAM_CR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_CONTRAST_P_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_CR_CONTROL mode : %d\n",  value);
		}
		break;												
	}
}


void sensor_saturation_control(char value)
{
	switch(value)
	{
		case PCAM_SA_STEP_M_2 :{
		PCAM_DEBUG("PCAM_SA_STEP_M_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SATURATION_M_2);
		}
		break;

		case PCAM_SA_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_M_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SATURATION_M_1);		
		}
		break;

		case PCAM_SA_STEP_0 :{
		PCAM_DEBUG("PCAM_SA_STEP_0");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SATURATION_0);
		}
		break;

		case PCAM_SA_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SATURATION_P_1);
		}
		break;

		case PCAM_SA_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SATURATION_P_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_SA_CONTROL mode : %d\n",  value);
		}
		break;					
	}

}


void sensor_sharpness_control(char value)
{
	switch(value)
	{
		case PCAM_SP_STEP_M_2 :{
		PCAM_DEBUG("PCAM_SP_STEP_M_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SHAPNESS_M_2);
		}
		break;

		case PCAM_SP_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_M_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SHAPNESS_M_1);		
		}
		break;

		case PCAM_SP_STEP_0 :{
		PCAM_DEBUG("PCAM_SP_STEP_0");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SHAPNESS_0);	
		}
		break;

		case PCAM_SP_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_1");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SHAPNESS_P_1);			
		}
		break;

		case PCAM_SP_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_2");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_SHAPNESS_P_2);			
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_SP_CONTROL mode : %d\n",  value);
		}
		break;					
	}
}


int sensor_af_control(char value)
{
	switch(value)
	{

		case PCAM_AF_CHECK_STATUS :{
		}
		break;
		
		case PCAM_AF_OFF :{
		}
		break;

		case PCAM_AF_SET_NORMAL :{
		}
		break;

		case PCAM_AF_SET_MACRO :{
		}
		break;

		case PCAM_AF_DO :{
		}
		break;


		default :{
			printk("<=PCAM=> unexpected AF command : %d\n",value);
		}		
		break;
		
	}	
	return;

}



void sensor_DTP_control(char value)
{
#if 0 //kkd temp
	switch(value)
	{
		case PCAM_DTP_OFF:{
		PCAM_DEBUG("DTP OFF");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_DTP_OFF);
		}
		break;

		case PCAM_DTP_ON:{
		PCAM_DEBUG("DTP ON");
		S5K5CAGX_WRITE_LIST(S5K5CAGX_DTP_ON);
		}
		break;

		default:{
		printk("<=PCAM=> unexpected DTP control on PCAM\n");
		}
		break;
	}
#endif
}





static void sensor_rough_control(void __user *arg)
{

	ioctl_pcam_info_8bit		ctrl_info;


	PCAM_DEBUG("START");


	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}

/*
	printk("<=PCAM=> TEST %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address,\
	 ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);
*/

	switch(ctrl_info.mode)
	{
		case PCAM_AUTO_TUNNING:
		break;

		
		case PCAM_SDCARD_DETECT:
		break;

		case PCAM_GET_INFO:{
			unsigned short lsb, msb, rough_iso;					
			s5k5cagx_sensor_write(0xFCFC, 0xD000);					
			s5k5cagx_sensor_write(0x002C, 0x7000);
			s5k5cagx_sensor_write(0x002E, 0x2A14);
			s5k5cagx_sensor_read(0x0F12, &lsb); 		//0x2A14
			s5k5cagx_sensor_read(0x0F12, &msb);		//0x2A16
			s5k5cagx_sensor_read(0x0F12, &rough_iso); //0x2A8
													
			ctrl_info.value_1 = lsb;					
			ctrl_info.value_2 = msb;	
			//printk("<=PCAM=> exposure %x %x \n", lsb, msb);
			ctrl_info.value_3 = rough_iso;	
			//printk("<=PCAM=> rough_iso %x \n", rough_iso);			
		}
		break;

		case PCAM_FRAME_CONTROL:
		{
			switch(ctrl_info.value_1)
			{

				case PCAM_FRAME_AUTO :{
				PCAM_DEBUG("PCAM_FRAME_AUTO");		
				}					
				break;
				
				case PCAM_FRAME_FIX_15 :{
				PCAM_DEBUG("PCAM_FRAME_FIX_15");
				S5K5CAGX_WRITE_LIST(S5K5CAGX_15_FPS);		
				}
				break;

				case PCAM_FRAME_FIX_30 :{
				printk("PCAM_FRAME_FIX_30");
				PCAM_DEBUG("PCAM_FRAME_FIX_30");
				//kkd temp S5K5CAGX_WRITE_LIST(S5K5CAGX_30_FPS);	
				}
				break;


				default :{
					printk("<=PCAM=> Unexpected PCAM_FRAME_CONTROL mode : %d\n", ctrl_info.value_1);
				}
				break;				
			
			}
		}
		break;


		case PCAM_AF_CONTROL:
		{
			mAfMode= ctrl_info.value_1;
			ctrl_info.value_3 = sensor_af_control(mAfMode);
		}
		break;

		
		case PCAM_EFFECT_CONTROL:
		{
			mEffect = ctrl_info.value_1;
			sensor_effect_control(mEffect);
				
			
		}// end of PCAM_EFFECT_CONTROL
		break;


		case PCAM_WB_CONTROL:
		{
			mWhiteBalance = ctrl_info.value_1;
			sensor_whitebalance_control(mWhiteBalance);
			

		}//end of PCAM_WB_CONTROL
		break;


		case PCAM_BR_CONTROL:
		{
			mBrightness = ctrl_info.value_1;
			if(mInit)
				sensor_brightness_control(mBrightness);
			
		}//end of PCAM_BR_CONTROL
		break;

		case PCAM_ISO_CONTROL:
		{
			mISO = ctrl_info.value_1;
			sensor_iso_control(mISO);
	
		}
		break;


		case PCAM_METERING_CONTROL:
		{
			mAutoExposure = ctrl_info.value_1;
			sensor_metering_control(mAutoExposure);
			
		}//end of case
		break;


		case PCAM_SCENE_CONTROL:
		{
			mScene = ctrl_info.value_1;
			sensor_scene_control(mScene);

		}
		break;


		case PCAM_AWB_AE_CONTROL:
		{
			printk("<=PCAM=> PCAM_AWB_AE_CONTROL skip~~~\n");
			/*
			switch(ctrl_info.value_1)
			{
				case PCAM_AWB_AE_LOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_LOCK");
				s5k5cagx_sensor_write_list(s5k5cagx_awb_ae_lock , s5k5cagx_AWB_AE_LOCK_REGS, \
				"s5k5cagx_awb_ae_lock"); 					
				}
				break;

				case PCAM_AWB_AE_UNLOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_UNLOCK");
				s5k5cagx_sensor_write_list(s5k5cagx_awb_ae_unlock , s5k5cagx_AWB_AE_UNLOCK_REGS, \
				"s5k5cagx_awb_ae_unlock"); 
					
				}
				break;

				default :{
					printk("<=PCAM=> Unexpected AWB_AE mode : %d\n", ctrl_info.value_1);
				}
				break;						
				
			}
			*/
		}
		break;
			
		case PCAM_CR_CONTROL:
		{
			mContrast = ctrl_info.value_1;
			if(mInit)
				sensor_contrast_control(mContrast);

		}
		break;
			

		case PCAM_SA_CONTROL:
		{
			mSaturation = ctrl_info.value_1;
			if(mInit)
				sensor_saturation_control(mSaturation);
			
		}
		break;

		

		case PCAM_SP_CONTROL:
		{
			mSharpness = ctrl_info.value_1;
			if(mInit)
				sensor_sharpness_control(mSharpness);
			
		}
		break;

		case PCAM_CPU_CONTROL:
		{
#if 0 //kkd temp
			switch(ctrl_info.value_1)
			{
				case PCAM_CPU_CONSERVATIVE:{
				PCAM_DEBUG("now conservative");
				cpufreq_direct_set_policy(0, "conservative");
				}
				break;

				case PCAM_CPU_ONDEMAND:{
				PCAM_DEBUG("now ondemand");
				cpufreq_direct_set_policy(0, "ondemand");
				}
				break;	

				case PCAM_CPU_PERFORMANCE:{
				PCAM_DEBUG("now performance");
				cpufreq_direct_set_policy(0, "performance");
				}
				break;
				
				default:{
					printk("<=PCAM=> unexpected CPU control on PCAM\n");
				}
				break;
			}
#endif
		}
		break;

		case PCAM_DTP_CONTROL:
		{
			if(mInit == 0)
			{
				if(ctrl_info.value_1 == 0)
					ctrl_info.value_3 = 2;

				else if(ctrl_info.value_1 == 1)
					ctrl_info.value_3 = 3;

				mDTP = 1;
			}

			else
			{
				sensor_DTP_control(ctrl_info.value_1);

				if(ctrl_info.value_1 == 0)
					ctrl_info.value_3 = 2;

				else if(ctrl_info.value_1 == 1)
					ctrl_info.value_3 = 3;
			
				mDTP = 0;
			}

		}
		break;

		case PCAM_SET_PREVIEW_SIZE:
		{
			mMode = ctrl_info.value_1;
		}
		break;


		default :{
			printk("<=PCAM=> Unexpected mode on sensor_rough_control : %d\n", ctrl_info.mode);
		}
		break;
	}



	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail on copy_to_user!\n", __func__);
	}
	
}










static void cam_pw(int status)
{
    	struct vreg *vreg_cam_28A,*vreg_cam_28IO,*vreg_cam_18D;

	vreg_cam_28A = vreg_get(NULL, "ldo9");
	vreg_cam_28IO = vreg_get(NULL, "ldo10");
	vreg_cam_18D = vreg_get(NULL, "ldo13");

	//gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //CAM_EN
        gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE); //i2c_scl  only for S5k5CCGX
        gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);//i2c_sda  only for S5k5CCGX

	if(status == 1) //POWER ON
	{
		PCAM_DEBUG("POWER ON");

	       vreg_enable(vreg_cam_18D);
		vreg_set_level(vreg_cam_18D, OUT1800mV);

 		mdelay(1);
		
	       vreg_enable(vreg_cam_28A);
		vreg_set_level(vreg_cam_28A, OUT2800mV);

	       vreg_enable(vreg_cam_28IO);
		vreg_set_level(vreg_cam_28IO, OUT2800mV);

		gpio_set_value(CAM_I2C_SCL, 1); //i2c_scl  only for S5k5CCGX	
		gpio_set_value(CAM_I2C_SDA, 1); //i2c_sda  only for S5k5CCGX	
	
		mdelay(1);
		
		//gpio_set_value(2, 1); //CAM_EN->UP	

		udelay(1);
	}
	else //POWER OFF
	{
		//gpio_set_value(2, 0); //CAM_EN->UP	

		udelay(1);
		
	       vreg_disable(vreg_cam_28IO);
		udelay(10);
	       vreg_disable(vreg_cam_28A);
	       vreg_disable(vreg_cam_18D);
		
		udelay(1);
	}
	
}



static int cam_hw_init()
{

	int rc = 0;
	unsigned short	id = 0; //CAM FOR FW
//	unsigned int	before_time, after_time, i;//I2C SPEED TEST
	cam_pw(1);
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //RESET
	//escape not use gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //STANDBY
	PCAM_DEBUG("next cam_hw_init");


	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	msm_camio_camif_pad_reg_reset();
	mdelay(1);
	
	//escape not use gpio_set_value(CAM_STANDBY, 1); //STBY UP   
	udelay(30);
	gpio_set_value(0, 1); //RESET UP   


#if 0//PGH I2C SPEED TEST
        before_time = get_jiffies_64();
    for (i = 0; i < 3000; i++) 
        {
		s5k5cagx_sensor_write(0x002E, 0x0040);
        }       
 
        after_time = get_jiffies_64();
        printk("<=PCAM=> Total Time 3000: %d\n",  jiffies_to_msecs(after_time-before_time));
#endif//PGH I2C SPEED TEST


	mdelay(15);

	s5k5cagx_sensor_write(0x002C, 0x0000);
	s5k5cagx_sensor_write(0x002E, 0x0040);
	s5k5cagx_sensor_read(0x0F12, &id);


	if(id != 0x05CA)
	{
		printk("<=PCAM=> [TASS] WRONG SENSOR FW => id 0x%x \n", id);
		//rc = -1;
	}
	else
		printk("<=PCAM=> [TASS] CURRENT SENSOR FW => id 0x%x \n", id);

	return rc;
}







static long s5k5cagx_set_effect(int mode, int effect)
{
	long rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	    	PCAM_DEBUG("SENSOR_PREVIEW_MODE");
		break;

	case SENSOR_SNAPSHOT_MODE:
		PCAM_DEBUG("SENSOR_SNAPSHOT_MODE");
		break;

	default:
		printk("[PGH] %s default\n", __func__);
		break;
	}

	switch (effect) {
	case CAMERA_EFFECT_OFF: {
	PCAM_DEBUG("CAMERA_EFFECT_OFF");
	}
			break;

	case CAMERA_EFFECT_MONO: {
	PCAM_DEBUG("CAMERA_EFFECT_MONO");
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {
	PCAM_DEBUG("CAMERA_EFFECT_NEGATIVE");
	}
		break;

	case CAMERA_EFFECT_SOLARIZE: {
	PCAM_DEBUG("CAMERA_EFFECT_SOLARIZE");
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
	PCAM_DEBUG("CAMERA_EFFECT_SEPIA");
	}
		break;

	default: {
	printk("<=PCAM=> unexpeceted effect  %s/%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	
	}
	s5k5cagx_effect = effect;
	
	return rc;
}

static long s5k5cagx_set_sensor_mode(int mode)
{
	//long rc = 0;
	unsigned short	msb, lsb;
	int base_high, base_low, cur_lux;
	char vsync_value, cnt;

	unsigned int	before_time, after_time, i, time_gab;//I2C SPEED TEST

	int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!

	PCAM_DEBUG("START");
	i2c_clk_addr = 0xd500c004;

	if(first_start_camera)
	{
		printk("<=PCAM=> camera init for TASS ver0.1");

		//i2c_clk_addr = 0xd500c004;
		//printk("<=PCAM=> i2c clk :%x\n", readl(i2c_clk_addr));

		before_time = get_jiffies_64();


		#if defined(CONFIG_LOAD_FILE)
		S5K5CAGX_WRITE_LIST(S5K5CAGX_INIT_SET);
		#else
		//S5K5CAGX_BURST_WRITE_LIST(S5K5CAGX_INIT_SET);
		S5K5CAGX_WRITE_LIST(S5K5CAGX_INIT_SET);
		#endif


		after_time = get_jiffies_64();
		time_gab = jiffies_to_msecs(after_time-before_time);
#if 0 //kkd temp
		if(time_gab > 280)
		{
			printk("<=PCAM=> i2c speed is going down : %dmsec\n", time_gab);

			pcam_msm_i2c_pwr_mgmt(s5k5cagx_client->adapter, 1);
			//this funcion have to call after clk_enable by PCAM, do not use it without pcam's allowance
			pcam_msm_i2c_pwr_mgmt(s5k5cagx_client->adapter, 1);
			
			//this funcion have to call after clk_enable by PCAM, do not use it without pcam's allowance
			printk("<=PCAM=> i2c clk set forcely 0x316\n");
			writel(0x316, i2c_clk_addr);
			printk("<=PCAM=> re-check i2c clk :%x\n", readl(i2c_clk_addr));

		    }
		else
		{
			printk("<=PCAM=> init speed is fine : %dmsec\n", time_gab);
		}
#endif
		first_start_camera = 0;
		mInit = 1;
	} 



	switch (mode) {
	case SENSOR_PREVIEW_MODE:
#if 0
	{
		PCAM_DEBUG("PREVIEW~~~");

		/*
		if(mMode)
		{
			printk("<=PCAM=> now CAMCORDER MODE~~~~~~~~~\n");	
			S5K5CAGX_WRITE_LIST(s5k5cagx_camcorder_set);	
			S5K5CAGX_WRITE_LIST(s5k5cagx_fps_30fix);	
		}
		else
		{
			printk("<=PCAM=> now CAMERA MODE~~~~~~~~~\n");
		}
		*/


		if((mScene == PCAM_SCENE_NIGHTSHOT) ||(mScene == PCAM_SCENE_FIREWORK) )
		{
			for(cnt=0; cnt<1200; cnt++)
			{
				vsync_value = gpio_get_value(14);

				if(vsync_value)
					break;
				else
				{
					PCAM_DEBUG(" wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
					msleep(1);
				}
		
			}

	
		}

		if(mDTP == 1)
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_DTP_ON);		
		}
		else
		{
			S5K5CAGX_WRITE_LIST(S5K5CAGX_PREVIEW_SIZE_800); //kkd temp	
		}


		if(mScene == PCAM_SCENE_OFF)
			sensor_effect_control(mEffect);

		sensor_scene_control(mScene);

		//if(mScene != PCAM_SCENE_SPORTS && mScene != PCAM_SCENE_NIGHTSHOT)
		//	sensor_iso_control(mISO);			

		if(mScene == PCAM_SCENE_OFF)
		{
		    sensor_brightness_control(mBrightness);
		    sensor_metering_control(mAutoExposure);
		    sensor_contrast_control(mContrast);
		    sensor_saturation_control(mSaturation);
		    sensor_sharpness_control(mSharpness);
		    sensor_whitebalance_control(mWhiteBalance);
		    //sensor_af_control(mAfMode);
		}

		
	}
#endif
		s5k5cagx_set_preview_start();
		break;
		
	case SENSOR_SNAPSHOT_MODE:
#if 0
	{
		PCAM_DEBUG("SNAPSHOT~~~");

		msb = lsb = 0;
		cur_lux = 0;

		s5k5cagx_sensor_write(0xFCFC, 0xD000);
		s5k5cagx_sensor_write(0x002C, 0x7000);
		s5k5cagx_sensor_write(0x002E, 0x2A3C);
		s5k5cagx_sensor_read(0x0F12, &lsb);
		s5k5cagx_sensor_read(0x0F12, &msb);

		cur_lux = (msb<<16) | lsb;
		printk("cur_lux : 0x%08x \n", cur_lux);


		for(cnt=0; cnt<700; cnt++)
		{
			vsync_value = gpio_get_value(14);

			if(vsync_value)
				break;
			else
			{
				printk(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);			
				PCAM_DEBUG(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
				msleep(3);
			}
	
		}


		if(cur_lux > 0xFFFE)
		{
			PCAM_DEBUG("HighLight Snapshot!\n");
			printk("HighLight Snapshot!\n");			
			S5K5CAGX_WRITE_LIST(s5k5cagx_high_snapshot);				
		}
		else if(cur_lux < 0x0020)
		{
			if((mScene == PCAM_SCENE_NIGHTSHOT) ||(mScene == PCAM_SCENE_FIREWORK) )
			{
				PCAM_DEBUG("Night Snapshot!\n");
				printk("Night Snapshot!\n");			
				S5K5CAGX_WRITE_LIST(s5k5cagx_night_snapshot);								
			}
			else
			{
				PCAM_DEBUG("LowLight Snapshot delay ~~~~!\n");
				printk("LowLight Snapshot 500...sec!\n");
				S5K5CAGX_WRITE_LIST(s5k5cagx_lowlight_snapshot);
			}
		}
		else
		{
			PCAM_DEBUG("Normal Snapshot!\n");
			printk("Normal Snapshot!\n");		
			S5K5CAGX_WRITE_LIST(s5k5cagx_snapshot);
		}
		
		
	}
#endif

	s5k5cagx_set_capture_start();
	break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		PCAM_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int s5k5cagx_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	return rc;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k5cagx_regs_table = NULL;

static int s5k5cagx_regs_table_size;

void s5k5cagx_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
//	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/s5k5cagx.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/sdcard/s5k5cagx.h", O_RDONLY, 0);
#endif
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);
	
	set_fs(fs);

	s5k5cagx_regs_table = dp;
	
	s5k5cagx_regs_table_size = l;

	*((s5k5cagx_regs_table + s5k5cagx_regs_table_size) - 1) = '\0';

//	printk("s5k5cagx_regs_table 0x%x, %ld\n", dp, l);
}

void s5k5cagx_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5k5cagx_regs_table) {
		kfree(s5k5cagx_regs_table);
		s5k5cagx_regs_table = NULL;
	}	
}

static int s5k5cagx_regs_table_write(char *name)
{
	char *start, *end, *reg;//, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	addr = value = 0;

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(s5k5cagx_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);	
			memcpy(data_buf, (reg + 9), 6);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
//			printk("addr 0x%04x, value 0x%04x\n", addr, value);
			if (addr == 0xffff)
			{
				msleep(value);
				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
			{
				if( s5k5cagx_sensor_write(addr, value) < 0 )
				{
					printk("<=PCAM=> %s fail on sensor_write\n", __func__);
				}
			}
		}
		else
			printk("<=PCAM=> EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
	}

	return 0;
}
#endif



int s5k5cagx_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	s5k5cagx_ctrl = kzalloc(sizeof(struct s5k5cagx_ctrl), GFP_KERNEL);
	if (!s5k5cagx_ctrl) {
		CDBG("s5k5cagx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5k5cagx_ctrl->sensordata = data;

	rc = cam_hw_init();
	if(rc < 0)
	{
		printk("<=PCAM=> cam_fw_init failed!\n");
		goto init_fail;
	}

#ifdef CONFIG_LOAD_FILE
	s5k5cagx_regs_table_init();
#endif	

	rc = s5k5cagx_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("s5k5cagx_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(s5k5cagx_ctrl);
	return rc;
}

static int s5k5cagx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k5cagx_wait_queue);
	return 0;
}

int s5k5cagx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&s5k5cagx_sem); */

	CDBG("s5k5cagx_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = s5k5cagx_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = s5k5cagx_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&s5k5cagx_sem); */

	return rc;
}

int s5k5cagx_sensor_release(void)
{
	int rc = 0;
        //int *switch_i2c_addr; //TEMP Dirty Code, Do not use it!


	first_start_camera = 1;

	//If did not init below that, it can keep the previous status. it depend on concept by PCAM
	mEffect = 0;
	mBrightness = 0;
	mContrast = 0;
	mSaturation = 0;
	mSharpness = 0;
	mWhiteBalance = 0;
	mISO = 0;
	mAutoExposure = 0;
	mScene = 0;
	//mAfMode = 0;
	mDTP = 0;
	mInit = 0;


	//TEMP Dirty Code, Do not use it! PCAM...
        //switch_i2c_addr = 0xd500c010;
        //writel(1, switch_i2c_addr);

	printk("<=PCAM=> s5k5cagx_sensor_release\n");

	kfree(s5k5cagx_ctrl);
	/* up(&s5k5cagx_sem); */

#ifdef CONFIG_LOAD_FILE
	s5k5cagx_regs_table_exit();
#endif
	//msm_camio_clk_disable(CAMIO_VFE_CLK);
	//msm_camio_clk_disable(CAMIO_MDC_CLK);
	//msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
	gpio_set_value(0, 0);//RESET
	//escape not use gpio_set_value(1, 0);//STBY
	mdelay(2);
	cam_pw(0);
	return rc;
}

static int s5k5cagx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;


	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k5cagx_sensorw =
		kzalloc(sizeof(struct s5k5cagx_work), GFP_KERNEL);

	if (!s5k5cagx_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k5cagx_sensorw);
	s5k5cagx_init_client(client);
	s5k5cagx_client = client;


	CDBG("s5k5cagx_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(s5k5cagx_sensorw);
	s5k5cagx_sensorw = NULL;
	CDBG("s5k5cagx_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k5cagx_i2c_id[] = {
	{ "s5k5cagx", 0},
	{ },
};

static struct i2c_driver s5k5cagx_i2c_driver = {
	.id_table = s5k5cagx_i2c_id,
	.probe  = s5k5cagx_i2c_probe,
	.remove = __exit_p(s5k5cagx_i2c_remove),
	.driver = {
		.name = "s5k5cagx",
	},
};

static int s5k5cagx_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{

	int rc = i2c_add_driver(&s5k5cagx_i2c_driver);
	if (rc < 0 || s5k5cagx_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}


#if 0//PCAM
	rc = s5k5cagx_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //RESET
	//escape not use gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //STANDBY
	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //CAM_EN
	
	gpio_set_value(0, 0);//RESET
	gpio_set_value(1, 0);//STBY
	cam_pw(0);


	s->s_init = s5k5cagx_sensor_init;
	s->s_release = s5k5cagx_sensor_release;
	s->s_config  = s5k5cagx_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __s5k5cagx_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k5cagx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k5cagx_probe,
	.driver = {
		.name = "msm_camera_s5k5cagx",
		.owner = THIS_MODULE,
	},
};

static int __init s5k5cagx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5cagx_init);
