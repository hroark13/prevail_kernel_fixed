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

//PCAM 1/5" s5k5ccgx

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>


#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>

#include <linux/slab.h>

#if defined(CONFIG_MACH_ROOKIE) 
#include "s5k5ccgx_rookie.h"
#elif defined(CONFIG_MACH_ESCAPE)
#include "s5k5ccgx_escape.h"
#elif defined(CONFIG_MACH_GIO)
#include "s5k5ccgx_gio.h"
#endif

#define SENSOR_DEBUG 0


//#define CONFIG_LOAD_FILE //dha23 110226

#define CAM_RESET 0
#if defined(CONFIG_MACH_ROOKIE)
#define CAM_STANDBY 1
#define CAM_EN 2
#define CAM_EN_2 3
#elif defined(CONFIG_MACH_GIO)
#define CAM_EN_2 3
#elif defined(CONFIG_MACH_ESCAPE)
#define CAM_STANDBY 1
#define CAM_EN 2
#endif
#define CAM_I2C_SCL 60
#define CAM_I2C_SDA 61
#define S5K5CCGX_BURST_WRITE_LIST(A)	s5k5ccgx_sensor_burst_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#define S5K5CCGX_WRITE_LIST(A)			s5k5ccgx_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);

// TODO: QualcommCameraHardware.cpp 에 있는 Define 된 숫자와 맞출 것. 
#define PCAM_AUTO_TUNNING			0
#define PCAM_SDCARD_DETECT			1
#define PCAM_GET_INFO				2
#define PCAM_FRAME_CONTROL			3
#define PCAM_AF_CONTROL				4

#define PCAM_SCENE_CONTROL			10
#define PCAM_AWB_AE_CONTROL		11
#define PCAM_CR_CONTROL				12
#define PCAM_SA_CONTROL				13
#define PCAM_SP_CONTROL				14
#define PCAM_CPU_CONTROL			15
#define PCAM_DTP_CONTROL			16
#define PCAM_SET_PREVIEW_SIZE		17
#define PCAM_GET_MODULE_STATUS		18

#define PCAM_LIGHT_CHECK				26

#define PCAM_AF_POWER_CONTROL		28

#define PCAM_FRAME_AUTO			0
#define PCAM_FRAME_FIX_15		1
#define PCAM_FRAME_FIX_20		2
#define PCAM_FRAME_FIX_24		3
#define PCAM_FRAME_FIX_30		4


#define PCAM_EFFECT_NORMAL		0
#define PCAM_EFFECT_NEGATIVE		1
#define PCAM_EFFECT_MONO		2
#define PCAM_EFFECT_SEPIA		3	


#define PCAM_WB_AUTO                    0
#define PCAM_WB_DAYLIGHT                1
#define PCAM_WB_CLOUDY                  2
#define PCAM_WB_FLUORESCENT             3
#define PCAM_WB_INCANDESCENT            4


#define PCAM_BR_STEP_P_4                4
#define PCAM_BR_STEP_P_3                3
#define PCAM_BR_STEP_P_2                2
#define PCAM_BR_STEP_P_1                1
#define PCAM_BR_STEP_0                  0
#define PCAM_BR_STEP_M_1                255//-1
#define PCAM_BR_STEP_M_2                254//-2
#define PCAM_BR_STEP_M_3                253//-3
#define PCAM_BR_STEP_M_4                252//-4

#define PCAM_CR_STEP_P_2                4
#define PCAM_CR_STEP_P_1                3
#define PCAM_CR_STEP_0                  2
#define PCAM_CR_STEP_M_1                1
#define PCAM_CR_STEP_M_2                0

#define PCAM_SA_STEP_P_2                4
#define PCAM_SA_STEP_P_1                3
#define PCAM_SA_STEP_0                  2
#define PCAM_SA_STEP_M_1                1
#define PCAM_SA_STEP_M_2                0

#define PCAM_SP_STEP_P_2                4
#define PCAM_SP_STEP_P_1                3
#define PCAM_SP_STEP_0                  2
#define PCAM_SP_STEP_M_1                1
#define PCAM_SP_STEP_M_2                0




#define PCAM_ISO_AUTO			0
#define PCAM_ISO_50			1
#define PCAM_ISO_100			2
#define PCAM_ISO_200			3
#define PCAM_ISO_400			4


#define PCAM_METERING_NORMAL		0 //CENTER?
#define PCAM_METERING_SPOT		1
#define PCAM_METERING_CENTER		2


#define PCAM_SCENE_AUTO				0
#define PCAM_SCENE_PORTRAIT			1
#define PCAM_SCENE_LANDSCAPE		2
#define PCAM_SCENE_SPORTS			3
#define PCAM_SCENE_PARTY_INDOOR	4
#define PCAM_SCENE_BEACH_SNOW		5
#define PCAM_SCENE_SUNSET			6
#define PCAM_SCENE_DAWN_DUSK		7
#define PCAM_SCENE_FALL_COLOR		8
#define PCAM_SCENE_NIGHT 			9
#define PCAM_SCENE_BACKLIGHT		10
#define PCAM_SCENE_FIREWORKS		11
#define PCAM_SCENE_TEXT				12
#define PCAM_SCENE_CANDLELIGHT		13


#define PCAM_AF_SET_INFINITY		0
#define PCAM_AF_SET_MACRO		1
#define PCAM_AF_SET_AUTO		2
#define PCAM_AF_CHECK_STATUS		3
#define PCAM_AF_OFF			4
//#define PCAM_AF_SET_AUTO		2
#define PCAM_AF_DO			5
//#define PCAM_AF_SET_MANUAL		5
#define PCAM_AF_CHECK_2nd_STATUS	6
#define PCAM_AF_ABORT			7



#define PCAM_AF_PROGRESS                1
#define PCAM_AF_SUCCESS                 2
#define PCAM_AF_LOWCONF                 3 //Fail
#define PCAM_AF_CANCELED                4
#define PCAM_AF_TIMEOUT                 5


#define PCAM_AWB_AE_LOCK		0
#define PCAM_AWB_AE_UNLOCK		1
#define PCAM_AE_LOCK			2
#define PCAM_AE_UNLOCK			3


#define PCAM_CPU_CONSERVATIVE		0
#define PCAM_CPU_ONDEMAND		1
#define PCAM_CPU_PERFORMANCE		2		

#define PCAM_DTP_OFF			0
#define PCAM_DTP_ON			1

//for PCAM_SET_PREVIEW_SIZE
#define PCAM_CAMERA_MODE			0
#define PCAM_CAMCORDER_MODE		1

#define FLASHMODE_OFF	1
#define FLASHMODE_AUTO	2
#define FLASHMODE_ON	3

static char first_start_camera = 1;//  1 is not init a sensor

static int16_t s5k5ccgx_effect = CAMERA_EFFECT_OFF;
static int previous_scene_mode = -1;
static int previous_WB_mode = 0;
static int af_mode = -1;
#if defined(CONFIG_MACH_ROOKIE)
static int af_power_enable = 0; //dha23 110422
#endif

typedef enum{
    af_stop = 0,
    af_running,
    af_status_max
};

static int af_status = af_status_max;

typedef enum{
    af_position_auto = 0,
    af_position_infinity,
    af_position_macro,
    af_position_max
};
static unsigned short AFPosition[af_position_max] = {0xFF, 0xFF, 0x48, 0xFF };       // auto, infinity, macro, default
static unsigned short DummyAFPosition[af_position_max] = {0xFE, 0xFE, 0x50, 0xFE};     // auto, infinity, macro, default
static unsigned short set_AF_postion = 0xFF;
static unsigned short set_Dummy_AF_position = 0xFE;
static unsigned short lux_value = 0;
static int preview_start = 0;

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
static char mLowLight = 0;


struct s5k5ccgx_work {
	struct work_struct work;
};

static struct  s5k5ccgx_work *s5k5ccgx_sensorw;
static struct  i2c_client *s5k5ccgx_client;

struct s5k5ccgx_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct s5k5ccgx_ctrl *s5k5ccgx_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(s5k5ccgx_wait_queue);
DECLARE_MUTEX(s5k5ccgx_sem);

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5k5ccgx_reg s5k5ccgx_regs;
extern unsigned char hw_version;

extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
extern void pcam_msm_i2c_pwr_mgmt(struct i2c_adapter *adap, int on);
extern int* get_i2c_clock_addr(struct i2c_adapter *adap);
/*=============================================================*/

static int cam_hw_init(void);
static int s5k5ccgx_sensor_af_control(int type);
static int s5k5ccgx_DTP_control(char value);

#ifdef CONFIG_LOAD_FILE
	static int s5k5ccgx_regs_table_write(char *name);
#endif





static int s5k5ccgx_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { s5k5ccgx_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static int s5k5ccgx_sensor_read_multi(unsigned short subaddr, unsigned long *data)
{
	unsigned char buf[4];
	struct i2c_msg msg = {s5k5ccgx_client->addr, 0, 2, buf};

	int ret;

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	msg.len = 4;

	ret = i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
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


static int s5k5ccgx_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { s5k5ccgx_client->addr, 0, 4, buf };

//	printk("[PGH] on write func s5k5ccgx_client->addr : %x\n", s5k5ccgx_client->addr);
//	printk("[PGH] on write func  s5k5ccgx_client->adapter->nr : %d\n", s5k5ccgx_client->adapter->nr);


	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

	return i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}



static int s5k5ccgx_sensor_write_list(struct s5k5ccgx_reg *list,int size, char *name)	
{
	int ret = 0;

#ifdef CONFIG_LOAD_FILE	
	ret = s5k5ccgx_regs_table_write(name);
#else
	int i = 0;

	printk("s5k5ccgx_sensor_write_list : %s\n", name);

	for (i = 0; i < size; i++)
	{
		if(list[i].addr == 0xffff)
		{
			msleep(list[i].val);
		}
		else
		{
		    if(s5k5ccgx_sensor_write(list[i].addr, list[i].val) < 0)
		    {
			    printk("<=PCAM=> sensor_write_list fail...-_-\n");
			    return -1;
		    }
		}
	}
#endif
	return ret;
}


#define BURST_MODE_BUFFER_MAX_SIZE 2700
unsigned char s5k5ccgx_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
static int s5k5ccgx_sensor_burst_write_list(struct s5k5ccgx_reg *list,int size, char *name)	
{
	int err = -EINVAL;
	int i = 0;
	int idx = 0;

	unsigned short subaddr = 0, next_subaddr = 0;
	unsigned short value = 0;

	struct i2c_msg msg = {  s5k5ccgx_client->addr, 0, 0, s5k5ccgx_buf_for_burstmode};
	
	printk("s5k5ccgx_sensor_burst_write_list : %s\n", name);

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
					s5k5ccgx_buf_for_burstmode[idx++] = 0x0F;
					s5k5ccgx_buf_for_burstmode[idx++] = 0x12;
				}
				s5k5ccgx_buf_for_burstmode[idx++] = value>> 8;
				s5k5ccgx_buf_for_burstmode[idx++] = value & 0xFF;

			
			 	//write in burstmode	
				if(next_subaddr != 0x0F12)
				{
					msg.len = idx;
					err = i2c_transfer(s5k5ccgx_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
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
			    err = s5k5ccgx_sensor_write(subaddr, value);
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



static int s5k5ccgx_set_preview_start()
{		
		char vsync_value, cnt;
		
		for(cnt=0; cnt<200; cnt++)
		{
			vsync_value = gpio_get_value(14);
	
			if(vsync_value)
					{	   
				printk("end vsync check cnt:%d\n", cnt);				
				break;
					}
			else
			{
	//			PCAM_DEBUG(" wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
				msleep(1);
			}
	
		}
		   
		// 2. DTP or preview start
		if(mDTP == 1) //output Test Pattern
		{	
			//printk( "pattern on setting~~~~~~~~~~~~~~\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_DTP_ON);
			//printk( "pattern on setting done~~~~~~~~~~~~~~\n");	
		}
		
		else //output preview start
		{
			S5K5CCGX_WRITE_LIST(S5K5CCGX_PREVIEW);
		}
		
	
		return 0;
}



static unsigned long s5k5ccgx_get_illumination()
{
	int err;
	unsigned long read_value;

	s5k5ccgx_sensor_write(0xFCFC, 0xD000);
	s5k5ccgx_sensor_write(0x002C, 0x7000);
	s5k5ccgx_sensor_write(0x002E, 0x2A3C);
	s5k5ccgx_sensor_read_multi(0x0F12, &read_value);

	//printk("s5k5ccgx_get_illumination() : lux_value == 0x%x\n", read_value);

	return read_value;
	
}

static int s5k5ccgx_set_flash(int lux_val)
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

static int s5k5ccgx_set_flash_mode(int flash_brightness_value, bool value)
{
	unsigned long lux_value;
	
	if(value){
		switch(flash_mode)
			{
				case FLASHMODE_AUTO:
					lux_value = s5k5ccgx_get_illumination();
					if (lux_value < 0x0020) 
						s5k5ccgx_set_flash(flash_brightness_value);
					break;
					
				case FLASHMODE_ON:
						s5k5ccgx_set_flash(flash_brightness_value);
					break;
					
				case FLASHMODE_OFF:
						//err =s5k5ccgx_set_flash(flash_brightness_value, sd);
					break;
					
				default:
					break;
			}
		}
	else
	{
		s5k5ccgx_set_flash(0);
	}
	
	return 0;
}

static int s5k5ccgx_set_capture_start()
{
	unsigned long lux_value;
	char vsync_value, cnt;
	// 1. CAPTURE SIZE
       //S5K5CCGX_WRITE_LIST(S5K5CCGX_CAPTURE_SIZE_2048); //kkd temp

	// 2. FLASH ON
//	s5k5ccgx_set_flash_mode(1,true);

	// 3. FLASH SET and DELAY
//	S5K5CCGX_WRITE_LIST(S5K5CCGX_FLASH_SET);
	
	// 4. CAPTURE START
#if 0
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
#endif	
	if (mScene == PCAM_SCENE_NIGHT || mScene == PCAM_SCENE_FIREWORKS) // NIGTSHOT or FIREWORKS PREVIEW
	{
		S5K5CCGX_WRITE_LIST(S5K5CCGX_NIGHT_SNAPSHOT);
	}
	else // NORMAL CAPTURE
	{
		lux_value = s5k5ccgx_get_illumination();

		if(lux_value > 0xFFFE) 		// highlight snapshot
		{
			mLowLight = 0;
			S5K5CCGX_WRITE_LIST(S5K5CCGX_HIGH_SNAPSHOT);	
		}
		else if(lux_value > 0x0020) 	// Normalt snapshot
		{
			mLowLight = 0;
			S5K5CCGX_WRITE_LIST(S5K5CCGX_NORMAL_SNAPSHOT);		
		}
		else 						//lowlight snapshot
		{
			mLowLight = 1;
			S5K5CCGX_WRITE_LIST(S5K5CCGX_LOWLIGHT_SNAPSHOT);
		}
	}

	// 5. FLASH SETING OFF
//	s5k5ccgx_set_flash_mode(0,false);

	return 0;
}


void sensor_effect_control(char value)
{

	//int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!
	//i2c_clk_addr = 0xd500c004;

	switch(value)
	{
		case PCAM_EFFECT_NORMAL :{
		PCAM_DEBUG("PCAM_EFFECT_NORMAL\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_OFF);
		}
		break;		

		case PCAM_EFFECT_NEGATIVE :{
		PCAM_DEBUG("PCAM_EFFECT_NEGATIVE\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_NEGATIVE);
		}
		break;	
		
		case PCAM_EFFECT_MONO :{
		PCAM_DEBUG("PCAM_EFFECT_MONO\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_MONO);
		}
		break;	

		case PCAM_EFFECT_SEPIA :{
		PCAM_DEBUG("PCAM_EFFECT_SEPIA\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_SEPIA);
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
		PCAM_DEBUG("PCAM_WB_AUTO\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_AUTO);
		}
		break;	

		case PCAM_WB_DAYLIGHT :{
		PCAM_DEBUG("PCAM_WB_DAYLIGHT\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_DAYLIGHT);
		}
		break;	

		case PCAM_WB_CLOUDY :{
		PCAM_DEBUG("PCAM_WB_CLOUDY\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_CLOUDY);
		}
		break;	

		case PCAM_WB_FLUORESCENT :{
		PCAM_DEBUG("PCAM_WB_FLUORESCENT\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_FLUORESCENT);
		}
		break;	
		
		case PCAM_WB_INCANDESCENT :{
		PCAM_DEBUG("PCAM_WB_INCANDESCENT\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_INCANDESCENT);
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
		PCAM_DEBUG("PCAM_BR_STEP_P_4\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_4);
		}
		break;
		
		case PCAM_BR_STEP_P_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_3\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_3);
		}
		break;

		case PCAM_BR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_2);
		}
		break;

		case PCAM_BR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_P_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_1);
		}
		break;

		case PCAM_BR_STEP_0 :{
		PCAM_DEBUG("PCAM_BR_STEP_0\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_0);
		}
		break;

		case PCAM_BR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_1);
		}
		break;

		case PCAM_BR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_2);
		}
		break;

		case PCAM_BR_STEP_M_3 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_3\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_3);
		}
		break;

		case PCAM_BR_STEP_M_4 :{
		PCAM_DEBUG("PCAM_BR_STEP_M_4\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_4);
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
		PCAM_DEBUG("PCAM_ISO_AUTO\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_AUTO);
		}
		break;

		case PCAM_ISO_50 :{
		PCAM_DEBUG("PCAM_ISO_50\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_50);
		}
		break;

		case PCAM_ISO_100 :{
		PCAM_DEBUG("PCAM_ISO_100\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_100);
		}
		break;

		case PCAM_ISO_200 :{
		PCAM_DEBUG("PCAM_ISO_200\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_200);
		}
		break;

		case PCAM_ISO_400 :{
		PCAM_DEBUG("PCAM_ISO_400\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_400);
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
		PCAM_DEBUG("PCAM_METERING_NORMAL\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_NORMAL);
		}
		break;
		
		case PCAM_METERING_SPOT :{
		PCAM_DEBUG("PCAM_METERING_SPOT\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_SPOT);
		}
		break;

		case PCAM_METERING_CENTER :{
		PCAM_DEBUG("PCAM_METERING_CENTER\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_CENTER);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected METERING mode : %d\n",  value);
		}
		break;
	}

}

static int sensor_scene_control(char value)
{
	switch(value)
	{
		case PCAM_SCENE_AUTO :
			PCAM_DEBUG("PCAM_SCENE_AUTO\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			break;

		case PCAM_SCENE_PORTRAIT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_PORTRAIT\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_PORTRAIT);
			break;

		case PCAM_SCENE_LANDSCAPE :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_LANDSCAPE\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_LANDSCAPE);
			break;

		case PCAM_SCENE_SPORTS:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_SPORTS\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_SPORTS);
			break;

		case PCAM_SCENE_PARTY_INDOOR:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_PARTY_INDOOR\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_PARTY_INDOOR);
			break;

		case PCAM_SCENE_BEACH_SNOW :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_BEACH_SNOW\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_BEACH_SNOW);
			break;

		case PCAM_SCENE_SUNSET :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_SUNSET\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_SUNSET);
			break;
		
		case PCAM_SCENE_DAWN_DUSK :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_DAWN_DUSK\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_DAWN_DUSK);
			break;

		case PCAM_SCENE_FALL_COLOR :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_FALL_COLOR\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_FALL_COLOR);
			break;

		case PCAM_SCENE_NIGHT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_NIGHT\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_NIGHT);			
			break;

		case PCAM_SCENE_BACKLIGHT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_BACKLIGHT\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_BACKLIGHT);			
			break;

		case PCAM_SCENE_FIREWORKS :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_FIREWORKS\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_FIREWORKS);				
			break;

		case PCAM_SCENE_TEXT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_TEXT\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_TEXT);				
			break;

		case PCAM_SCENE_CANDLELIGHT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_AUTO);
			PCAM_DEBUG("PCAM_SCENE_CANDLELIGHT\n");
			S5K5CCGX_WRITE_LIST(S5K5CCGX_SCENE_CANDLELIGHT);				
			break;

		default :
			printk("<=PCAM=> Unexpected SCENE mode : %d\n",  value);
			break;				
	}
	 mScene = value;
        return 0;
}


void sensor_contrast_control(char value)
{
	switch(value)
	{
		case PCAM_CR_STEP_M_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_CONTRAST_M_2);
		}
		break;

		case PCAM_CR_STEP_M_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_M_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_CONTRAST_M_1);	
		}
		break;

		case PCAM_CR_STEP_0 :{
		PCAM_DEBUG("PCAM_CR_STEP_0\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_CONTRAST_0);
		}
		break;

		case PCAM_CR_STEP_P_1 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_CONTRAST_P_1);
		}
		break;

		case PCAM_CR_STEP_P_2 :{
		PCAM_DEBUG("PCAM_CR_STEP_P_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_CONTRAST_P_2);
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
		PCAM_DEBUG("PCAM_SA_STEP_M_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SATURATION_M_2);
		}
		break;

		case PCAM_SA_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_M_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SATURATION_M_1);		
		}
		break;

		case PCAM_SA_STEP_0 :{
		PCAM_DEBUG("PCAM_SA_STEP_0\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SATURATION_0);
		}
		break;

		case PCAM_SA_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SATURATION_P_1);
		}
		break;

		case PCAM_SA_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SA_STEP_P_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SATURATION_P_2);
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
		PCAM_DEBUG("PCAM_SP_STEP_M_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SHAPNESS_M_2);
		}
		break;

		case PCAM_SP_STEP_M_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_M_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SHAPNESS_M_1);		
		}
		break;

		case PCAM_SP_STEP_0 :{
		PCAM_DEBUG("PCAM_SP_STEP_0\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SHAPNESS_0);	
		}
		break;

		case PCAM_SP_STEP_P_1 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_1\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SHAPNESS_P_1);			
		}
		break;

		case PCAM_SP_STEP_P_2 :{
		PCAM_DEBUG("PCAM_SP_STEP_P_2\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_SHAPNESS_P_2);			
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected PCAM_SP_CONTROL mode : %d\n",  value);
		}
		break;					
	}
}

static int s5k5ccgx_DTP_control(char value)
{
	switch(value)
	{
		case PCAM_DTP_OFF:{
		PCAM_DEBUG("DTP OFF\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_DTP_OFF);
		}
		break;

		case PCAM_DTP_ON:{
		PCAM_DEBUG("DTP ON\n");
		S5K5CCGX_WRITE_LIST(S5K5CCGX_DTP_ON);
		}
		break;

		default:{
		printk("<=PCAM=> unexpected DTP control on PCAM\n");
		}
		break;
	}
	return 0;
}





void sensor_rough_control(void __user *arg)
{

	ioctl_pcam_info_8bit		ctrl_info;


	PCAM_DEBUG("START\n");
	if(SENSOR_DEBUG) printk("sensor_rough_control :: ctrl_info.mode = %d\n",ctrl_info.mode);
	
	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}

	switch(ctrl_info.mode)
	{
		case PCAM_AUTO_TUNNING:
		break;

		
		case PCAM_SDCARD_DETECT:
		break;

		case PCAM_GET_INFO:{
			unsigned short lsb, msb, rough_iso;					
			s5k5ccgx_sensor_write(0xFCFC, 0xD000);					
			s5k5ccgx_sensor_write(0x002C, 0x7000);
			s5k5ccgx_sensor_write(0x002E, 0x2A14);
			s5k5ccgx_sensor_read(0x0F12, &lsb); 		//0x2A14
			s5k5ccgx_sensor_read(0x0F12, &msb);		//0x2A16
			s5k5ccgx_sensor_read(0x0F12, &rough_iso); //0x2A8
													
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

				case PCAM_FRAME_AUTO :
					PCAM_DEBUG("PCAM_FRAME_AUTO\n");
					S5K5CCGX_WRITE_LIST(S5K5CCGX_AUTO15_FPS);		
					break;
				
				case PCAM_FRAME_FIX_15 :
					PCAM_DEBUG("PCAM_FRAME_FIX_15\n");
					S5K5CCGX_WRITE_LIST(S5K5CCGX_15_FPS);		
					break;

				case PCAM_FRAME_FIX_20 :
					PCAM_DEBUG("PCAM_FRAME_FIX_20\n");
					S5K5CCGX_WRITE_LIST(S5K5CCGX_20_FPS);	
					break;

#if defined(CONFIG_MACH_ESCAPE)
				case PCAM_FRAME_FIX_24 :
					PCAM_DEBUG("PCAM_FRAME_FIX_24\n");
					S5K5CCGX_WRITE_LIST(S5K5CCGX_24_FPS);	
					break;
#endif
				case PCAM_FRAME_FIX_30 :
					PCAM_DEBUG("PCAM_FRAME_FIX_30\n");
					S5K5CCGX_WRITE_LIST(S5K5CCGX_30_FPS);	
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
			ctrl_info.value_3 = s5k5ccgx_sensor_af_control(mAfMode);
		}
		break;

		case PCAM_AWB_AE_CONTROL:
		{
			//printk("<=PCAM=> PCAM_AWB_AE_CONTROL skip~~~\n");
			switch(ctrl_info.value_1)
			{
				case PCAM_AWB_AE_LOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_LOCK\n");
				S5K5CCGX_WRITE_LIST(S5K5CCGX_AE_AWB_LOCK); 					
				}
				break;

				case PCAM_AWB_AE_UNLOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_UNLOCK\n");
				S5K5CCGX_WRITE_LIST(S5K5CCGX_AE_AWB_UNLOCK); 										
				}
				break;
				case PCAM_AE_LOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_LOCK\n");
				S5K5CCGX_WRITE_LIST(S5K5CCGX_AE_LOCK); 					
				}
				break;

				case PCAM_AE_UNLOCK :{
				PCAM_DEBUG("PCAM_AWB_AE_UNLOCK\n");
				S5K5CCGX_WRITE_LIST(S5K5CCGX_AE_UNLOCK); 										
				}
				break;

				default :{
					printk("<=PCAM=> Unexpected AWB_AE mode : %d\n", ctrl_info.value_1);
				}
				break;						
				
			}
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
			switch(ctrl_info.value_1)
			{
				case PCAM_CPU_CONSERVATIVE:{
				PCAM_DEBUG("now conservative\n");
				cpufreq_direct_set_policy(0, "conservative");
				}
				break;

				case PCAM_CPU_ONDEMAND:{
				PCAM_DEBUG("now ondemand\n");
				cpufreq_direct_set_policy(0, "ondemand");
				}
				break;	

				case PCAM_CPU_PERFORMANCE:{
				PCAM_DEBUG("now performance\n");
				cpufreq_direct_set_policy(0, "performance");
				}
				break;
				
				default:{
					printk("<=PCAM=> unexpected CPU control on PCAM\n");
				}
				break;
			}
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
				//sensor_DTP_control(ctrl_info.value_1);

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

		case PCAM_LIGHT_CHECK:
		{
			ctrl_info.value_3 = mLowLight;
		}
		break;

#if defined(CONFIG_MACH_ROOKIE)
		//dha23 110422 set af power off on camcorder to lower consumption current <<<
		case PCAM_AF_POWER_CONTROL:
		{
			struct vreg *vreg_cam_af;

			vreg_cam_af = vreg_get(NULL, "ldo16");

			if(af_power_enable !=  ctrl_info.value_1){
				udelay(1);
				if(ctrl_info.value_1 == 1){
					printk("<=PCAM=> AF Power ON\n");
					vreg_enable(vreg_cam_af);
				}else{
					printk("<=PCAM=> AF Power OFF\n");
					vreg_disable(vreg_cam_af);
				}
				af_power_enable = ctrl_info.value_1;
				udelay(1);
			}
		}
		break;
#endif
		
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

#if defined(CONFIG_MACH_ROOKIE)
void cam_pw(int status)
{
    struct vreg *vreg_cam_out_vdda, *vreg_cam_af;

    vreg_cam_out_vdda = vreg_get(NULL, "ldo15");
    vreg_cam_af		= vreg_get(NULL, "ldo16");

	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //CAM_EN VDD_IO
    gpio_tlmm_config(GPIO_CFG(CAM_EN_2, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //CAM_EN_2 VDD
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //i2c_scl  only for S5k5CCGX
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);//i2c_sda  only for S5k5CCGX

	if(status == 1) //POWER ON
	{
		PCAM_DEBUG("POWER ON\n");
		vreg_set_level(vreg_cam_out_vdda,  OUT2800mV);		// VDDA 2.8V
		vreg_enable(vreg_cam_out_vdda);
		mdelay(2);
		gpio_set_value(CAM_EN_2, 1); //CAM_EN->UP	VDD
		mdelay(2);
		gpio_set_value(CAM_EN, 1); //CAM_EN_2->UP	VDD_IO

		mdelay(1);
		gpio_set_value(CAM_I2C_SCL, 1); //i2c_scl  only for S5k5CCGX	
		gpio_set_value(CAM_I2C_SDA, 1); //i2c_sda  only for S5k5CCGX	
		vreg_set_level(vreg_cam_af,  OUT3000mV);		// AF 3V
		vreg_enable(vreg_cam_af);
		af_power_enable = 1; //dha23 110422
	}
	else //POWER OFF
	{
		PCAM_DEBUG("POWER OFF");
		gpio_set_value(CAM_EN, 0); //CAM_EN	VDD_IO
		mdelay(2);
		gpio_set_value(CAM_EN_2, 0); //CAM_EN_2	VDD
		mdelay(1);
		vreg_disable(vreg_cam_out_vdda);
		
		udelay(1);
		vreg_disable(vreg_cam_af);
		af_power_enable = 0; //dha23 110422		
	}	
}

#elif defined(CONFIG_MACH_GIO)

void cam_pw(int status)
{
    struct vreg *vreg_cam_vddio;
    struct vreg *vreg_cam_vddd;
    struct vreg *vreg_cam_vdda;

    if(hw_version >= 3)
    {
        vreg_cam_vddio = vreg_get(NULL, "ldo16");
    }
    else
    {
        vreg_cam_vddd = vreg_get(NULL, "ldo16");
    }        
    vreg_cam_vdda = vreg_get(NULL, "ldo18");

    gpio_tlmm_config(GPIO_CFG(CAM_EN_2, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); // VDDIO_1.8V
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //i2c_scl  only for S5k5CCGX
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);//i2c_sda  only for S5k5CCGX

    if(status == 1) //POWER ON
    {
        PCAM_DEBUG("POWER ON\n");

        // VDDA 2.8V
        vreg_set_level(vreg_cam_vdda,  OUT2800mV);
        vreg_enable(vreg_cam_vdda);
        udelay(5);

        if(hw_version >= 3)
        {
            // VDDD_1.2V
            gpio_set_value(CAM_EN_2, 1);
            udelay(5);

            // VDDIO_2.8V
            vreg_set_level(vreg_cam_vddio,  OUT2800mV);
            vreg_enable(vreg_cam_vddio);
            mdelay(1);
        }
        else
        {
            // VDDD 1.8V
            vreg_set_level(vreg_cam_vddd,  OUT1800mV);
            vreg_enable(vreg_cam_vddd);
            udelay(5);

            // VDDIO_1.8V
            gpio_set_value(CAM_EN_2, 1);
            mdelay(1);
        }

        gpio_set_value(CAM_I2C_SCL, 1); //i2c_scl  only for S5k5CCGX	
        gpio_set_value(CAM_I2C_SDA, 1); //i2c_sda  only for S5k5CCGX	
    }
    else //POWER OFF
    {
        PCAM_DEBUG("POWER OFF");

        if(hw_version >= 3)
        {
            // VDDIO_2.8V
            vreg_disable(vreg_cam_vddio);            
            udelay(5);

            // VDDD_1.2V
            gpio_set_value(CAM_EN_2, 0);
            udelay(5);
        }
        else
        {
            // VDDIO 1.8V
            gpio_set_value(CAM_EN_2, 0);
            udelay(5);

            // VDDD 1.8V
            vreg_disable(vreg_cam_vddd);
            udelay(5);
        }

        // VDDA_2.8V
        vreg_disable(vreg_cam_vdda);
        udelay(5);		
    }	
}

#elif defined(CONFIG_MACH_ESCAPE)

void cam_pw(int status)
{
    struct vreg *vreg_cam_out_vdda;

    vreg_cam_out_vdda = vreg_get(NULL, "ldo15");

	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //CAM_EN VDD_IO
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE); //i2c_scl  only for S5k5CCGX
    gpio_tlmm_config(GPIO_CFG(CAM_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);//i2c_sda  only for S5k5CCGX

	if(status == 1) //POWER ON
	{
		PCAM_DEBUG("POWER ON\n");
		vreg_set_level(vreg_cam_out_vdda,  OUT2800mV);		// VDDA 2.8V
		vreg_enable(vreg_cam_out_vdda);
		mdelay(2);
		gpio_set_value(CAM_EN, 1); //CAM_EN_2->UP	VDD_IO

		mdelay(1);
		gpio_set_value(CAM_I2C_SCL, 1); //i2c_scl  only for S5k5CCGX	
		gpio_set_value(CAM_I2C_SDA, 1); //i2c_sda  only for S5k5CCGX	
	}
	else //POWER OFF
	{
		PCAM_DEBUG("POWER OFF");
		gpio_set_value(CAM_EN, 0); //CAM_EN	VDD_IO
		mdelay(2);
		vreg_disable(vreg_cam_out_vdda);		
	}	
}
#endif

static int cam_hw_init()
{
	int rc = 0;
	unsigned short id = 0, evt =0;

	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //RESET
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //STANDBY
#endif // defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)

	PCAM_DEBUG("cam_pw_2\n");

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	msm_camio_camif_pad_reg_reset();
	udelay(1);
	
	gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),GPIO_CFG_ENABLE); //mclk enable
	mdelay(1);
	
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)
	gpio_set_value(CAM_STANDBY, 1); //STBY UP   
	mdelay(2);
#endif // defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)

	gpio_set_value(CAM_RESET, 1); //RESET UP   

	PCAM_DEBUG("next cam_hw_init\n");

#if 0//PGH I2C SPEED TEST
        before_time = get_jiffies_64();
    for (i = 0; i < 3000; i++) 
        {
		s5k5ccgx_sensor_write(0x002E, 0x0040);
        }       
 
        after_time = get_jiffies_64();
        printk("<=PCAM=> Total Time 3000: %d\n",  jiffies_to_msecs(after_time-before_time));
#endif//PGH I2C SPEED TEST

	mdelay(15);

	s5k5ccgx_sensor_write(0xFCFC, 0xD000);
	s5k5ccgx_sensor_write(0x002C, 0x0000);
	s5k5ccgx_sensor_write(0x002E, 0x0040);
	s5k5ccgx_sensor_read(0x0F12, &id);
	
	s5k5ccgx_sensor_write(0xFCFC, 0xD000);
	s5k5ccgx_sensor_write(0x002C, 0x7000);
	s5k5ccgx_sensor_write(0x002E, 0x0152);
	s5k5ccgx_sensor_read(0x0F12, &evt);

	if(id != 0x05CC)
	{
		printk("<=PCAM=> [TASS] WRONG SENSOR FW => id 0x%x, evt 0x%x \n", id,evt);
		rc = -1;
	}
	else
	{
		printk("<=PCAM=> [TASS] CURRENT SENSOR FW => id 0x%x, evt 0x%x \n", id,evt);
    }
	return rc;
}

static long s5k5ccgx_set_effect(int mode, int effect)
{
	long rc = 0;

	switch (effect) {
		case CAMERA_EFFECT_OFF:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_OFF);
			break;

		case CAMERA_EFFECT_MONO:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_MONO);
			break;

		case CAMERA_EFFECT_NEGATIVE:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_NEGATIVE);
			break;

		case CAMERA_EFFECT_SEPIA:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_SEPIA);
			break;
			
		case CAMERA_EFFECT_AQUA:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_AQUA);
			break;
			
	 	//case CAMERA_EFFECT_WHITEBOARD: 
	 	case CAMERA_EFFECT_SKETCH: //dha23 110523
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_SKETCH);
			break; 

		default: 
			S5K5CCGX_WRITE_LIST(S5K5CCGX_EFFECT_OFF);
		//			return -EINVAL;
        return 0;
	}
	s5k5ccgx_effect = effect;

	return rc;
}

static long s5k5ccgx_set_brightness(int mode, int brightness)
{
	long rc = 0;

	switch (brightness) {
		case CAMERA_BRIGTHNESS_0:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_3);
			break;

		case CAMERA_BRIGTHNESS_1:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_2);
			break;

		case CAMERA_BRIGTHNESS_2:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_N_1);
			break;

		case CAMERA_BRIGTHNESS_3:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_0);
			break;

		case CAMERA_BRIGTHNESS_4:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_1);
			break;

		case CAMERA_BRIGTHNESS_5:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_2);
 			break;

		case CAMERA_BRIGTHNESS_6:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_BRIGHTNESS_P_3);
 			break;

		default:
			printk("<=PCAM=> unexpected brightness %s/%d\n", __func__, __LINE__);
			//			return -EINVAL;
		return 0;

 	}
	return rc;
}

static long s5k5ccgx_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	switch (wb) {
		case CAMERA_WB_AUTO:
			previous_WB_mode = wb;
			S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_AUTO);
			break;

		case CAMERA_WB_INCANDESCENT:
			previous_WB_mode = wb;            
			S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_INCANDESCENT);
			break;

		case CAMERA_WB_DAYLIGHT:
			previous_WB_mode = wb;            
			S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_DAYLIGHT);
			break;

		case CAMERA_WB_FLUORESCENT:
			previous_WB_mode = wb;            
			S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_FLUORESCENT);
			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
			previous_WB_mode = wb;            
			S5K5CCGX_WRITE_LIST(S5K5CCGX_WB_CLOUDY);
			break;

		default:
			printk("<=PCAM=> unexpected WB mode %s/%d\n", __func__, __LINE__);
			//			return -EINVAL;
		return 0;

 	}
	return rc;
}

static long s5k5ccgx_set_metering(int mode, int metering)
{
	long rc = 0;

	switch (metering) {

		case CAMERA_AEC_CENTER_WEIGHTED:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_CENTER);
			break;

		case CAMERA_AEC_SPOT_METERING:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_SPOT);
			break;

		case CAMERA_AEC_FRAME_AVERAGE:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_METERING_NORMAL);
			break;

		default:
			printk("<=PCAM=> unexpected metering %s/%d\n", __func__, __LINE__);
			//			return -EINVAL;
		return 0;

 	}
	return rc;
}

static long s5k5ccgx_set_ISO(int mode, int iso)
{
	long rc = 0;

	switch (iso) {
		case CAMERA_ISOValue_AUTO:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_AUTO);
			break;

		case CAMERA_ISOValue_100:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_100);
			break;

		case CAMERA_ISOValue_200:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_200);
			break;

		case CAMERA_ISOValue_400:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_ISO_400);
			break;

		default:
			printk("<=PCAM=> unexpected ISO value %s/%d\n", __func__, __LINE__);
			//			return -EINVAL;
		return 0;

 	}
	return rc;
}

static void s5k5ccgx_sensor_reset_af_position(void)
{
	// printk("RESET AF POSITION : af_mode = %d \n",af_mode);

	s5k5ccgx_sensor_write(0xFCFC, 0xD000);    
	s5k5ccgx_sensor_write(0x0028, 0x7000);
	s5k5ccgx_sensor_write(0x002A, 0x030E);    
	s5k5ccgx_sensor_write(0x0F12, 0x00FE); // dummy lens position

	s5k5ccgx_sensor_write(0x002A, 0x030C);    
	s5k5ccgx_sensor_write(0x0F12, 0x0000);
	msleep(130);

	s5k5ccgx_sensor_write(0x002A, 0x030E);
	if(af_mode == 4)
		s5k5ccgx_sensor_write(0x0F12, 0x0048); // move lens to 0x48 (macro mode initial lens position)
	else
		s5k5ccgx_sensor_write(0x0F12, 0x00FF); // move lens to 0xFF (normal/infinity/off mode initial lens position)

	msleep(50);    
}

static int s5k5ccgx_sensor_af_control(int type)
{
	unsigned short ret = 0;
	int size = 0;

	// printk("[CAM-SENSOR] s5k5ccgx_sensor_af_control : %d, preview_start : %d\n", type, preview_start); 

	switch (type)
	{
		case PCAM_AF_SET_MACRO:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_AF_MACRO_ON);
			break;

		case PCAM_AF_SET_AUTO:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_AF_NORMAL_ON);
			break;

		case PCAM_AF_SET_INFINITY:
		//case PCAM_AF_OFF:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_AF_OFF);
			break;

		case PCAM_AF_DO:
			S5K5CCGX_WRITE_LIST(S5K5CCGX_AF_DO);
			break;
		
		case PCAM_AF_ABORT :
			S5K5CCGX_WRITE_LIST(S5K5CCGX_AF_ABORT);
			break;
		
		case PCAM_AF_CHECK_STATUS:
		{   
		  	unsigned short AF_status = 0 , AF_result=0;
		  	int err = 0;

		        err  = s5k5ccgx_sensor_write(0xFCFC, 0xD000);
		        err += s5k5ccgx_sensor_write(0x002C, 0x7000);
		        err += s5k5ccgx_sensor_write(0x002E, 0x2D12);
		        err += s5k5ccgx_sensor_read(0x0F12, &AF_status);

		        if(err < 0)
		        {
		    		printk("AF is Failure~~~~~~~(I2C Failed) \n", __func__);
		    		return 0;
		        }
			AF_result = AF_status & 0x000f;
			switch(AF_result)
			{
				case 1:
		    	    	ret = PCAM_AF_PROGRESS;
				break;
		    	      	case 2: 
		        	{
		        		//printk("AF is success~~~~~~~(1st Search) \n", __func__);
					ret = PCAM_AF_SUCCESS;
					break;
		        	}
		        	case 0:
				case 3:
				case 4:
				case 6:
				case 8:
				default:
		        	{
		        		printk("AF is Failure~~~~~~~(1st Search) \n", __func__);
		        		ret = PCAM_AF_LOWCONF;
					break;
		        	}
		        }
		}
		break;
		
		case PCAM_AF_CHECK_2nd_STATUS:
		{
			 unsigned short AF_status = 0;
			 int err = 0;
			 
		        err  = s5k5ccgx_sensor_write(0xFCFC, 0xD000);
		        err += s5k5ccgx_sensor_write(0x002C, 0x7000);
		        err += s5k5ccgx_sensor_write(0x002E, 0x1F2F);
		        err += s5k5ccgx_sensor_read(0x0F12, &AF_status);

		        if(err < 0)
		        {
		    		printk("AF is Failure~~~~~~~(I2C Failed) \n", __func__);
		    		return 0;
		        }

		    	if((AF_status & 0xFF00) == 0x0000) 
		    	{
		    		//printk("AF is success~~~~~~~(2nd Search) \n", __func__);
		    		ret = PCAM_AF_SUCCESS;
		    	}
		    	else if((AF_status & 0xFF00) == 0x0100)
		    	{
		    	       //printk("AF is in progress~~~~~~~(2nd Search) \n", __func__);
		    		ret = PCAM_AF_PROGRESS;
		    	}
			else
			{
		        	printk("AF is Failure~~~~~~~(2st Search) \n", __func__);
		        	ret = PCAM_AF_LOWCONF;
			}
		}		    
		break;
		
		default:
			break;
	}

	return ret;	
}

static long s5k5ccgx_set_sensor_mode(int mode)
{
	//long rc = 0;
	unsigned short	msb, lsb;
	int base_high, base_low, cur_lux;
	char vsync_value, cnt;

	unsigned int	before_time, after_time, i, time_gab;//I2C SPEED TEST

	int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!

	PCAM_DEBUG("START\n");
	//i2c_clk_addr = 0xd580c004;	//for SCH-I559 TASS
	i2c_clk_addr = get_i2c_clock_addr(s5k5ccgx_client->adapter);	
	preview_start = 1;

	if(first_start_camera)
	{
		//printk("<=PCAM=> camera init for TASS ver0.1");

		//i2c_clk_addr = 0xd500c004;
		printk("<=PCAM=>i2c_clk_addr:%x,  i2c clk :%x\n", i2c_clk_addr,readl(i2c_clk_addr));


		before_time = get_jiffies_64();
		#if defined(CONFIG_LOAD_FILE)
		S5K5CCGX_WRITE_LIST(S5K5CCGX_INIT_SET);
		#else
		S5K5CCGX_BURST_WRITE_LIST(S5K5CCGX_INIT_SET);
		//S5K5CCGX_WRITE_LIST(S5K5CCGX_INIT_SET);
		#endif


		after_time = get_jiffies_64();
		time_gab = jiffies_to_msecs(after_time-before_time);

		if(time_gab > 530)
		{
			printk("<=PCAM=> i2c speed is going down : %dmsec\n", time_gab);

			pcam_msm_i2c_pwr_mgmt(s5k5ccgx_client->adapter, 1);
			//this funcion have to call after clk_enable by PCAM, do not use it without pcam's allowance
			printk("<=PCAM=> i2c clk set forcely 0x316\n");
			writel(0x316, i2c_clk_addr);
			printk("<=PCAM=> re-check i2c clk :%x\n", readl(i2c_clk_addr));

		    }
		else
		{
			printk("<=PCAM=> init speed is fine : %dmsec\n", time_gab);
		}

		first_start_camera = 0;
		mInit = 1;
	} 



	switch (mode) {
		case SENSOR_PREVIEW_MODE:
			s5k5ccgx_set_preview_start();
			break;
			
		case SENSOR_SNAPSHOT_MODE:
			s5k5ccgx_set_capture_start();
			break;

		case SENSOR_RAW_SNAPSHOT_MODE:
			PCAM_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!\n");
			break;

		default:
			//			return -EINVAL;
		return 0;

	}

	return 0;
}

static int s5k5ccgx_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	#ifndef CONFIG_LOAD_FILE
	S5K5CCGX_WRITE_LIST(S5K5CCGX_PRE_INIT_SET);
	#endif

	mdelay(10);
	return rc;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k5ccgx_regs_table = NULL;

static int s5k5ccgx_regs_table_size;

void s5k5ccgx_regs_table_init(void)
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
#if defined(CONFIG_MACH_ROOKIE) 
	filp = filp_open("/mnt/sdcard/s5k5ccgx_tuning_for_rookie.h", O_RDONLY, 0);
#elif defined (CONFIG_MACH_ESCAPE)
	filp = filp_open("/mnt/sdcard/s5k5ccgx_tuning_for_escape.h", O_RDONLY, 0);
#elif defined (CONFIG_MACH_GIO)
	filp = filp_open("/mnt/sdcard/s5k5ccgx_tuning_for_gio.h", O_RDONLY, 0);
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

	s5k5ccgx_regs_table = dp;
	
	s5k5ccgx_regs_table_size = l;

	*((s5k5ccgx_regs_table + s5k5ccgx_regs_table_size) - 1) = '\0';

//	printk("s5k5ccgx_regs_table 0x%x, %ld\n", dp, l);
}

void s5k5ccgx_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5k5ccgx_regs_table) {
		kfree(s5k5ccgx_regs_table);
		s5k5ccgx_regs_table = NULL;
	}	
}

static int s5k5ccgx_regs_table_write(char *name)
{
	char *start, *end, *reg;//, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	addr = value = 0;

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';
	start = strstr(s5k5ccgx_regs_table, name);
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
				//printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
			{
				if( s5k5ccgx_sensor_write(addr, value) < 0 )
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

int s5k5ccgx_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	s5k5ccgx_ctrl = kzalloc(sizeof(struct s5k5ccgx_ctrl), GFP_KERNEL);
	if (!s5k5ccgx_ctrl) {
		CDBG("s5k5ccgx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5k5ccgx_ctrl->sensordata = data;

	rc = cam_hw_init();
	if(rc < 0)
	{
		printk("<=PCAM=> cam_fw_init failed!\n");
		goto init_fail;
	}

#ifdef CONFIG_LOAD_FILE
	s5k5ccgx_regs_table_init();
#endif	

	rc = s5k5ccgx_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("s5k5ccgx_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(s5k5ccgx_ctrl);
	return rc;
}

static int s5k5ccgx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k5ccgx_wait_queue);
	return 0;
}

int s5k5ccgx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&s5k5ccgx_sem); */
	PCAM_DEBUG("s5k5ccgx_ioctl, cfgtype = %d, mode = %d\n", cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = s5k5ccgx_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = s5k5ccgx_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_SET_BRIGHTNESS:
			rc = s5k5ccgx_set_brightness(cfg_data.mode,
						cfg_data.cfg.brightness);
			break;

		case CFG_SET_WB:
			rc = s5k5ccgx_set_whitebalance(cfg_data.mode,
						cfg_data.cfg.whitebalance);
			break;

		case CFG_SET_ISO:
			rc = s5k5ccgx_set_ISO(cfg_data.mode,
						cfg_data.cfg.iso);
			break;

		case CFG_SET_EXPOSURE_MODE:
			rc = s5k5ccgx_set_metering(cfg_data.mode,
						cfg_data.cfg.metering);
			break;

        	case CFG_MOVE_FOCUS:
        		rc = s5k5ccgx_sensor_af_control(cfg_data.cfg.focus.steps);
        		break;

        	case CFG_SET_DEFAULT_FOCUS:
        		rc = s5k5ccgx_sensor_af_control(cfg_data.cfg.focus.steps);
        		break;

		case CFG_SET_DATALINE_CHECK:
			rc = s5k5ccgx_DTP_control(cfg_data.cfg.dataline);
			break;

			
		case CFG_SET_SCENE_MODE:
			rc = sensor_scene_control(cfg_data.cfg.scene);
			break;
			
		case CFG_GET_AF_MAX_STEPS:
		default:
			//			return -EINVAL;
		return 0;

			break;
		}

	/* up(&s5k5ccgx_sem); */

	return rc;
}

int s5k5ccgx_sensor_release(void)
{
	int rc = 0;
 
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
	mAfMode = 0;
	mDTP = 0;
	mInit = 0;

    preview_start = 0;

	printk("<=PCAM=> s5k5ccgx_sensor_release\n");

	kfree(s5k5ccgx_ctrl);
	/* up(&s5k5ccgx_sem); */

#ifdef CONFIG_LOAD_FILE
	s5k5ccgx_regs_table_exit();
#endif
    cam_pw(0);// added by insook
    
	return rc;
}

static int s5k5ccgx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k5ccgx_sensorw =
		kzalloc(sizeof(struct s5k5ccgx_work), GFP_KERNEL);

	if (!s5k5ccgx_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k5ccgx_sensorw);
	s5k5ccgx_init_client(client);
	s5k5ccgx_client = client;


	CDBG("s5k5ccgx_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(s5k5ccgx_sensorw);
	s5k5ccgx_sensorw = NULL;
	CDBG("s5k5ccgx_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k5ccgx_i2c_id[] = {
	{ "s5k5ccgx", 0},
	{ },
};

static struct i2c_driver s5k5ccgx_i2c_driver = {
	.id_table = s5k5ccgx_i2c_id,
	.probe  = s5k5ccgx_i2c_probe,
	.remove = __exit_p(s5k5ccgx_i2c_remove),
	.driver = {
		.name = "s5k5ccgx",
	},
};

static int s5k5ccgx_sensor_probe(const struct msm_camera_sensor_info *info,
				                    struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&s5k5ccgx_i2c_driver);
	
	if(rc < 0 || s5k5ccgx_client == NULL) 
	{
		rc = -ENOTSUPP;
		goto probe_done;
	}
	
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //RESET
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); //STANDBY
#endif // defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)

	cam_pw(1);
	mdelay(1);

	/* Input MCLK = 24MHz */
	gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),GPIO_CFG_ENABLE); //mclk enable
	mdelay(1);
	
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)
	gpio_set_value(CAM_STANDBY, 1); //STBY UP   
	udelay(30);
#endif // defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)

	gpio_set_value(CAM_RESET, 1); //RESET UP   
 	rc = s5k5ccgx_sensor_init_probe(info);
	if(rc < 0)
	{
		goto probe_done;
    }		

	gpio_set_value(CAM_RESET, 0);
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)
	gpio_set_value(CAM_STANDBY, 0);
#endif // defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)

	s->s_init = s5k5ccgx_sensor_init;
	s->s_release = s5k5ccgx_sensor_release;
	s->s_config  = s5k5ccgx_sensor_config;
	s->s_camera_type  = BACK_CAMERA_2D;
	s->s_mount_angle  = 180;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __s5k5ccgx_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k5ccgx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k5ccgx_probe,
	.driver = {
		.name = "msm_camera_s5k5ccgx",
		.owner = THIS_MODULE,
	},
};

static int __init s5k5ccgx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5ccgx_init);
