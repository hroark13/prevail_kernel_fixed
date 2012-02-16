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
//bestiq TEST S5k4CA
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "s5k4ca.h"

#include <mach/camera.h>//bestiq

#define SENSOR_DEBUG 0
#undef CONFIG_LOAD_FILE
//#define CONFIG_LOAD_FILE
#define I2C_BURST_MODE //dha23 100325 카메라 기동 시간 줄이기 위해 I2C Burst mode 사용.

//aswoogi add
#define CAM_RESET 0
#define CAM_STANDBY 1
#define CAM_EN 2
#define CAM_I2C_SCL 60
#define CAM_I2C_SDA 61


#define PCAM_AUTO_TUNNING		0
#define PCAM_SDCARD_DETECT		1
#define PCAM_EXPOSURE_TIME		2
#define PCAM_ISO_SPEED			3
#define PCAM_FIXED_FRAME		4
#define PCAM_AUTO_FRAME			5
#define PCAM_NIGHT_PREVIEW		6
#define PCAM_NIGHT_CAPTURE		7

struct s5k4ca_work {
	struct work_struct work;
};

static struct  s5k4ca_work *s5k4ca_sensorw;
static struct  i2c_client *s5k4ca_client;

struct s5k4ca_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct s5k4ca_ctrl *s5k4ca_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(s5k4ca_wait_queue);
DECLARE_MUTEX(s5k4ca_sem);


#ifdef CONFIG_LOAD_FILE
static int s5k4ca_regs_table_write(char *name);
#endif
static int cam_hw_init(void);


static int previous_scene_mode = -1;
static int previous_WB_mode = 0;

static int16_t s5k4ca_effect = CAMERA_EFFECT_OFF;
static int af_mode = -1;

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

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5k4ca_reg s5k4ca_regs;


/*=============================================================*/

//aswoogi add
#if 0
static void s5k4ca_sensor_gpio_init(void)
{
	int test_value =0; //PGH FOR TEST

	printk("[PGH] using other gpio funcs\n");

	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE); //RESET
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE); //STANDBY
	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE); //CAM_EN

	gpio_set_value(CAM_RESET, 0);//RESET
	gpio_set_value(CAM_STANDBY, 0);//STANDBY
	gpio_set_value(CAM_EN, 0); //CAM_EN

        test_value = gpio_get_value(CAM_STANDBY);
        printk("[PGH] 1 STANDBY  : %d\n",test_value);	

	test_value = gpio_get_value(CAM_EN);
	printk("[PGH] CAM_EN  : %d\n",test_value);	
}
#endif

#if 0//bestiq
static int s5k4ca_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;

	rc = gpio_request(dev->sensor_reset, "s5k4ca");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		mdelay(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}

	gpio_free(dev->sensor_reset);
	return rc;
}


static int32_t s5k4ca_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_w: 0x%04x 0x%04x\n",
			*(u16 *) txdata, *(u16 *) (txdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_w: 0x%04x\n", *(u16 *) txdata);
	else
		CDBG("msm_io_i2c_w: length = %d\n", length);
#endif
	if (i2c_transfer(s5k4ca_client->adapter, msg, 1) < 0) {
		CDBG("s5k4ca_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4ca_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum s5k4ca_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = s5k4ca_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = waddr;
		buf[1] = wdata;
		rc = s5k4ca_i2c_txdata(saddr, buf, 2);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t s5k4ca_i2c_write_table(
	struct s5k4ca_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = s5k4ca_i2c_write(s5k4ca_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata,
			reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static int s5k4ca_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

#if SENSOR_DEBUG
	if (length == 2)
		CDBG("msm_io_i2c_r: 0x%04x 0x%04x\n",
			*(u16 *) rxdata, *(u16 *) (rxdata + 2));
	else if (length == 4)
		CDBG("msm_io_i2c_r: 0x%04x\n", *(u16 *) rxdata);
	else
		CDBG("msm_io_i2c_r: length = %d\n", length);
#endif

	if (i2c_transfer(s5k4ca_client->adapter, msgs, 2) < 0) {
		CDBG("s5k4ca_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4ca_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum s5k4ca_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
	case WORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = s5k4ca_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		CDBG("s5k4ca_i2c_read failed!\n");

	return rc;
}
#endif

#if 1//PGH
static int s5k4ca_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { s5k4ca_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
        {   
                printk("[S5K4CA] s5k4ca_sensor_read fail : %d \n", ret);                 
		goto error;
        }

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
        {   
                printk("[S5K4CA] s5k4ca_sensor_read fail : %d \n", ret);                 
		goto error;
        }

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}
#endif//PGH

int s5k4ca_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { s5k4ca_client->addr, 0, 4, buf };

//	printk("[PGH] on write func s5k4ca_client->addr : %x\n", s5k4ca_client->addr);
//	printk("[PGH] on write func  s5k4ca_client->adapter->nr : %d\n", s5k4ca_client->adapter->nr);

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

        if(i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1)
        {
            return 0;
        }
        else
        {
            printk("[S5K4CA] s5k4ca_sensor_write fail \n");        
            return -EIO;
        }
//	return i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

int s5k4ca_sensor_write_list(struct samsung_short_t *list,char *name)
{
#ifndef CONFIG_LOAD_FILE 
	int i;
#endif
        int ret;
	ret = 0;

#ifdef CONFIG_LOAD_FILE 
	s5k4ca_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(s5k4ca_sensor_write(list[i].subaddr, list[i].value) < 0)
                {      
                        printk("[S5K4CA] s5k4ca_sensor_write_list fail : %d, %x, %x \n", i, list[i].value, list[i].subaddr);            
			return -1;
                 }
	}
#endif    
	return ret;
}

#ifdef I2C_BURST_MODE //dha23 100325
#define BURST_MODE_SET			1
#define BURST_MODE_END			2
#define NORMAL_MODE_SET			3
#define MAX_INDEX				1000
static int s5k4ca_sensor_burst_write_list(struct samsung_short_t *list,char *name)
{
	__u8 temp_buf[MAX_INDEX];
	int index_overflow = 1;
	int new_addr_start = 0;
	int burst_mode = NORMAL_MODE_SET;
	unsigned short pre_subaddr = 0;
	struct i2c_msg msg = { s5k4ca_client->addr, 0, 4, temp_buf };
	int i=0, ret=0;
	unsigned int index = 0;

        memset(temp_buf, 0x00 ,1000);
	
	printk("s5k4ca_sensor_burst_write_list( %s ) \n", name); 
	printk("s5k4ca_sensor_i2c_address(0x%02x) \n", s5k4ca_client->addr);     
#ifdef CONFIG_LOAD_FILE 
	s5k4ca_regs_table_write(name);	
#else
	for (i = 0; list[i].subaddr != 0xffff; i++)
	{
		if(list[i].subaddr == 0xdddd)
		{
		    msleep(list[i].value);
                    printk("delay 0x%04x, value 0x%04x\n", list[i].subaddr, list[i].value);
		}	
		else
		{					
			if( list[i].subaddr == list[i+1].subaddr )
			{
				burst_mode = BURST_MODE_SET;
				if((list[i].subaddr != pre_subaddr) || (index_overflow == 1))
				{
					new_addr_start = 1;
					index_overflow = 0;
				}
			}
			else
			{
				if(burst_mode == BURST_MODE_SET)
				{
					burst_mode = BURST_MODE_END;
					if(index_overflow == 1)
					{
						new_addr_start = 1;
						index_overflow = 0;
					}
				}
				else
				{
					burst_mode = NORMAL_MODE_SET;
				}
			}

			if((burst_mode == BURST_MODE_SET) || (burst_mode == BURST_MODE_END))
			{
				if(new_addr_start == 1)
				{
					index = 0;
					//memset(temp_buf, 0x00 ,1000);
					index_overflow = 0;

					temp_buf[index] = (list[i].subaddr >> 8);
					temp_buf[++index] = (list[i].subaddr & 0xFF);

					new_addr_start = 0;
				}
				
				temp_buf[++index] = (list[i].value >> 8);
				temp_buf[++index] = (list[i].value & 0xFF);
				
				if(burst_mode == BURST_MODE_END)
				{
					msg.len = ++index;

					ret = i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("i2c_transfer fail ! \n");
						return -1;
					}
				}
				else if( index >= MAX_INDEX-1 )
				{
					index_overflow = 1;
					msg.len = ++index;
					
					ret = i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					if( ret < 0)
					{
						printk("I2C_transfer Fail ! \n");
						return -1;
					}
				}
				
			}
			else
			{
				//memset(temp_buf, 0x00 ,4);
			
				temp_buf[0] = (list[i].subaddr >> 8);
				temp_buf[1] = (list[i].subaddr & 0xFF);
				temp_buf[2] = (list[i].value >> 8);
				temp_buf[3] = (list[i].value & 0xFF);

				msg.len = 4;
				ret = i2c_transfer(s5k4ca_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
				if( ret < 0)
				{
					printk("I2C_transfer Fail ! \n");
					return -1;
				}
			}
		}
		
		pre_subaddr = list[i].subaddr;
	}
#endif
	return ret;
}
#endif

#if 0//PGH
static int32_t s5k4ca_set_lens_roll_off(void)
{
	int32_t rc = 0;
	rc = s5k4ca_i2c_write_table(&s5k4ca_regs.rftbl[0],
								 s5k4ca_regs.rftbl_size);
	return rc;
}
#endif//PGH

#if 0//PGH
static long s5k4ca_reg_init(void)
{
	int32_t array_length;
	int32_t i;
	long rc;

	/* PLL Setup Start */
	rc = s5k4ca_i2c_write_table(&s5k4ca_regs.plltbl[0],
					s5k4ca_regs.plltbl_size);

	if (rc < 0)
		return rc;
	/* PLL Setup End   */

	array_length = s5k4ca_regs.prev_snap_reg_settings_size;

	/* Configure sensor for Preview mode and Snapshot mode */
	for (i = 0; i < array_length; i++) {
		rc = s5k4ca_i2c_write(s5k4ca_client->addr,
		  s5k4ca_regs.prev_snap_reg_settings[i].register_address,
		  s5k4ca_regs.prev_snap_reg_settings[i].register_value,
		  WORD_LEN);

		if (rc < 0)
			return rc;
	}

	/* Configure for Noise Reduction, Saturation and Aperture Correction */
	array_length = s5k4ca_regs.noise_reduction_reg_settings_size;

	for (i = 0; i < array_length; i++) {
		rc = s5k4ca_i2c_write(s5k4ca_client->addr,
			s5k4ca_regs.noise_reduction_reg_settings[i].register_address,
			s5k4ca_regs.noise_reduction_reg_settings[i].register_value,
			WORD_LEN);

		if (rc < 0)
			return rc;
	}

	/* Set Color Kill Saturation point to optimum value */
	rc =
	s5k4ca_i2c_write(s5k4ca_client->addr,
	0x35A4,
	0x0593,
	WORD_LEN);
	if (rc < 0)
		return rc;

	rc = s5k4ca_i2c_write_table(&s5k4ca_regs.stbl[0],
					s5k4ca_regs.stbl_size);
	if (rc < 0)
		return rc;

	rc = s5k4ca_set_lens_roll_off();
	if (rc < 0)
		return rc;

	return 0;
}
#endif//PGH

static long s5k4ca_set_effect(int mode, int effect)
{
//	uint16_t reg_addr;
	//uint16_t reg_val;
	long rc = 0;

	switch (effect) {
	case CAMERA_EFFECT_OFF: {
	printk("[PGH] CAMERA_EFFECT_OFF\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_off_04,"s5k4ca_effect_off_04");
	}
			break;

	case CAMERA_EFFECT_MONO: {
	printk("[PGH] CAMERA_EFFECT_MONO\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_gray_04,"s5k4ca_effect_gray_04");
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {
	printk("[PGH] CAMERA_EFFECT_NEGATIVE\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_negative_04,"s5k4ca_effect_negative_04");
	}
		break;

	case CAMERA_EFFECT_SOLARIZE: {

	printk("[PGH] CAMERA_EFFECT_SOLARIZE\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_aqua_04,"s5k4ca_effect_aqua_04");
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
	printk("[PGH] CAMERA_EFFECT_SEPIA\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_sepia_04,"s5k4ca_effect_sepia_04");
	}
		break;
	case CAMERA_EFFECT_AQUA: {
	printk("[PGH] CAMERA_EFFECT_AQUA\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_aqua_04,"s5k4ca_effect_aqua_04");
	}
		break;
 	case CAMERA_EFFECT_WHITEBOARD: {
	printk("[PGH] CAMERA_EFFECT_WHITEBOARD\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_sketch_04,"s5k4ca_effect_sketch_04");
	}
		break; 

	default: {

	printk("[PGH] default .dsfsdf\n");
	s5k4ca_sensor_write_list(s5k4ca_effect_off_04,"s5k4ca_effect_off_04");

		return -EINVAL;
	}
	}
	s5k4ca_effect = effect;

	return rc;
}

static long s5k4ca_set_brightness(int mode, int brightness)
{
	long rc = 0;

	printk("mode : %d,   brightness : %d", mode, brightness);

	switch (brightness) {
		case CAMERA_BRIGTHNESS_0:
			printk("CAMERA_BRIGTHNESS_0");
			s5k4ca_sensor_write_list(s5k4ca_br_minus3_04, "s5k4ca_br_minus3_04");
			break;

		case CAMERA_BRIGTHNESS_1:
			printk("CAMERA_BRIGTHNESS_1");
			s5k4ca_sensor_write_list(s5k4ca_br_minus2_04, "s5k4ca_br_minus2_04");
			break;

		case CAMERA_BRIGTHNESS_2:
			printk("CAMERA_BRIGTHNESS_2");
			s5k4ca_sensor_write_list(s5k4ca_br_minus1_04, "s5k4ca_br_minus1_04");
			break;

		case CAMERA_BRIGTHNESS_3:
			printk("CAMERA_BRIGTHNESS_3");
			s5k4ca_sensor_write_list(s5k4ca_br_zero_04, "s5k4ca_br_zero_04");
			break;

		case CAMERA_BRIGTHNESS_4:
			printk("CAMERA_BRIGTHNESS_4");
			s5k4ca_sensor_write_list(s5k4ca_br_plus1_04, "s5k4ca_br_plus1_04");
			break;

		case CAMERA_BRIGTHNESS_5:
			printk("CAMERA_BRIGTHNESS_5");
			s5k4ca_sensor_write_list(s5k4ca_br_plus2_04, "s5k4ca_br_plus2_04");
 			break;

		case CAMERA_BRIGTHNESS_6:
			printk("CAMERA_BRIGTHNESS_6");
			s5k4ca_sensor_write_list(s5k4ca_br_plus3_04, "s5k4ca_br_plus3_04");
 			break;

		default:
			printk("<=PCAM=> unexpected brightness %s/%d\n", __func__, __LINE__);
			return -EINVAL;
 	}
	return rc;
}

static long s5k4ca_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	printk("mode : %d,   whitebalance : %d", mode, wb);

	switch (wb) {
		case CAMERA_WB_AUTO:
			printk("CAMERA_WB_AUTO");
                        previous_WB_mode = wb;
			s5k4ca_sensor_write_list(s5k4ca_wb_auto_04, "s5k4ca_wb_auto_04");
			break;

		case CAMERA_WB_INCANDESCENT:
			printk("CAMERA_WB_INCANDESCENT");
                        previous_WB_mode = wb;            
			s5k4ca_sensor_write_list(s5k4ca_wb_tungsten_04, "s5k4ca_wb_tungsten_04");
			break;

		case CAMERA_WB_FLUORESCENT:
			printk("CAMERA_WB_FLUORESCENT");
                        previous_WB_mode = wb;            
			s5k4ca_sensor_write_list(s5k4ca_wb_fluorescent_04, "s5k4ca_wb_fluorescent_04");
			break;

		case CAMERA_WB_DAYLIGHT:
			printk("<=PCAM=> CAMERA_WB_DAYLIGHT");
                        previous_WB_mode = wb;            
			s5k4ca_sensor_write_list(s5k4ca_wb_sunny_04, "s5k4ca_wb_sunny_04");
			break;

		case CAMERA_WB_CLOUDY_DAYLIGHT:
			printk("<=PCAM=> CAMERA_WB_CLOUDY_DAYLIGHT");
                        previous_WB_mode = wb;            
			s5k4ca_sensor_write_list(s5k4ca_wb_cloudy_04, "s5k4ca_wb_cloudy_04");
			break;

		default:
			printk("<=PCAM=> unexpected WB mode %s/%d\n", __func__, __LINE__);
			return -EINVAL;
 	}
	return rc;
}

static long s5k4ca_set_metering(int mode, int metering)
{
	long rc = 0;

	printk("mode : %d,   metering : %d", mode, metering);

	switch (metering) {

		case CAMERA_AEC_CENTER_WEIGHTED:
			printk("CAMERA_AEC_CENTER_WEIGHTED");
#ifdef I2C_BURST_MODE //dha23 100325	
			s5k4ca_sensor_burst_write_list(s5k4ca_measure_brightness_center_04, "s5k4ca_measure_brightness_center_04");
#else            
			s5k4ca_sensor_write_list(s5k4ca_measure_brightness_center_04, "s5k4ca_measure_brightness_center_04");
#endif
			break;

		case CAMERA_AEC_SPOT_METERING:
			printk("CAMERA_AEC_SPOT_METERING");
#ifdef I2C_BURST_MODE //dha23 100325	
			s5k4ca_sensor_burst_write_list(s5k4ca_measure_brightness_spot_04, "s5k4ca_measure_brightness_spot_04");
#else            
			s5k4ca_sensor_write_list(s5k4ca_measure_brightness_spot_04, "s5k4ca_measure_brightness_spot_04");
#endif
			break;

		case CAMERA_AEC_FRAME_AVERAGE:
			printk("CAMERA_AEC_FRAME_AVERAGE");
#ifdef I2C_BURST_MODE //dha23 100325	
			s5k4ca_sensor_burst_write_list(s5k4ca_measure_brightness_default_04, "s5k4ca_measure_brightness_default_04");
#else            
			s5k4ca_sensor_write_list(s5k4ca_measure_brightness_default_04, "s5k4ca_measure_brightness_default_04");
#endif
			break;

		default:
			printk("<=PCAM=> unexpected metering %s/%d\n", __func__, __LINE__);
			return -EINVAL;
 	}
	return rc;
}

static long s5k4ca_set_ISO(int mode, int iso)
{
	long rc = 0;

	printk("mode : %d,   ISO : %d", mode, iso);

	switch (iso) {
		case CAMERA_ISOValue_AUTO:
			printk("CAMERA_ISO_AUTO");
			s5k4ca_sensor_write_list(s5k4ca_iso_auto_04, "s5k4ca_iso_auto_04");			
			break;

		case CAMERA_ISOValue_100:
			printk("CAMERA_ISO_100");
			s5k4ca_sensor_write_list(s5k4ca_iso100_04, "s5k4ca_iso100_04");			
			break;

		case CAMERA_ISOValue_200:
			printk("CAMERA_ISO_200");
			s5k4ca_sensor_write_list(s5k4ca_iso200_04, "s5k4ca_iso200_04");			
			break;

		case CAMERA_ISOValue_400:
			printk("CAMERA_ISO_400");
			s5k4ca_sensor_write_list(s5k4ca_iso400_04, "s5k4ca_iso400_04");			
			break;

		default:
			printk("<=PCAM=> unexpected ISO value %s/%d\n", __func__, __LINE__);
			return -EINVAL;
 	}
	return rc;
}

static void s5k4ca_sensor_reset_af_position(void)
{
    printk("RESET AF POSITION : af_mode = %d \n",af_mode);
    
    s5k4ca_sensor_write(0xFCFC, 0xD000);    
    s5k4ca_sensor_write(0x0028, 0x7000);
    s5k4ca_sensor_write(0x002A, 0x030E);    
    s5k4ca_sensor_write(0x0F12, 0x00FE); // dummy lens position

    s5k4ca_sensor_write(0x002A, 0x030C);    
    s5k4ca_sensor_write(0x0F12, 0x0000);
    msleep(130);

    s5k4ca_sensor_write(0x002A, 0x030E);
    if(af_mode == 4)
        s5k4ca_sensor_write(0x0F12, 0x0048); // move lens to 0x48 (macro mode initial lens position)
    else
        s5k4ca_sensor_write(0x0F12, 0x00FF); // move lens to 0xFF (normal/infinity/off mode initial lens position)
        
    msleep(50);    
}

static int s5k4ca_sensor_set_ae_awb_lock(void)
{
    printk("SET AW & AWB LOCK \n"); 

    //s5k4ca_sensor_write_list(5k4ca_ae_awb_lock,"s5k4ca_ae_awb_lock");
    s5k4ca_sensor_write(0xFCFC, 0xD000);
    s5k4ca_sensor_write(0x0028, 0x7000);
    s5k4ca_sensor_write(0x002A, 0x0578);
    s5k4ca_sensor_write(0x0F12, 0x0075);
}

static void s5k4ca_sensor_set_ae_awb_unlock(void)
{
    int ret_val = 0;
    printk("SET AW & AWB UNLOCK : previous_WB_mode = %d, previous_scene_mode = %d\n",previous_WB_mode,previous_scene_mode); 
    
    if(previous_WB_mode == 0 && previous_scene_mode != 4)
    {
        //s5k4ca_sensor_write_list(5k4ca_ae_awb_unlock,"s5k4ca_ae_awb_unlock");
        s5k4ca_sensor_write(0xFCFC, 0xD000);        
        s5k4ca_sensor_write(0x0028, 0x7000); 
        s5k4ca_sensor_write(0x002A, 0x0578); 
        s5k4ca_sensor_write(0x0F12, 0x007F);        
    }
    else 
    {
        //s5k4ca_sensor_write_list(5k4ca_ae_mwb_unlock,"s5k4ca_ae_mwb_unlock");
        s5k4ca_sensor_write(0xFCFC, 0xD000);
        s5k4ca_sensor_write(0x0028, 0x7000);
        s5k4ca_sensor_write(0x002A, 0x0578);
        s5k4ca_sensor_write(0x0F12, 0x0077);  
    }
}

static int s5k4ca_sensor_af_control(int type)
{
    int count = 50;
    int tmpVal = 0;
    int ret = 0;
    int size = 0;
    int i = 0;
    unsigned short light = 0;

    printk("[CAM-SENSOR] s5k4ca_sensor_af_control : %d, preview_start : %d\n", type, preview_start); 

    s5k4ca_sensor_write(0x002C, 0x7000);	
    s5k4ca_sensor_write(0x002E, 0x12FE);
    s5k4ca_sensor_read(0x0F12, &light);

    switch (type)
    {
        case 0: // release
            printk("[CAM-SENSOR] Focus Mode -> release\n"); 
            af_status = af_stop;
            
            // AW & AWB UNLOCK
            s5k4ca_sensor_set_ae_awb_unlock();

            // AF POSITION RESET
            s5k4ca_sensor_reset_af_position();

            break;

        case 1: // AF start
            printk("Focus Mode -> Single\n");
            af_status = af_running;
            
            s5k4ca_sensor_set_ae_awb_lock();      // lock AWB/AE
            
             // setting AF single
            s5k4ca_sensor_write(0xFCFC, 0xD000); 
            s5k4ca_sensor_write(0x0028, 0x7000); 
            s5k4ca_sensor_write(0x002A, 0x030C);
            s5k4ca_sensor_write(0x0F12, 0x0002); //AF Single 

            if (light <= 0x18)
               msleep(260);    // delay 2frames before af status check
            else
               msleep(130);    // delay 2frames before af status check 
            
            do
            {
                if( count == 0)
                    break;

                s5k4ca_sensor_write(0xFCFC, 0xD000);
                s5k4ca_sensor_write(0x002C, 0x7000);    
                s5k4ca_sensor_write(0x002E, 0x130E);
                msleep(100);

                s5k4ca_sensor_read(0x0F12, &tmpVal); 
                count--;

                printk("CAM 3M AF Status Value = %x \n", tmpVal); 
            }
            while( (tmpVal & 0x3) != 0x3 && (tmpVal & 0x3) != 0x2 );

            if(count == 0  )
            {
                s5k4ca_sensor_write(0xFCFC, 0xD000); 
                s5k4ca_sensor_write(0x0028, 0x7000); 

                s5k4ca_sensor_write(0x002A, 0x030E);  
                s5k4ca_sensor_write(0x0F12, set_Dummy_AF_position);  //030E = 00FF 입력위해 다른값 임시입력

                s5k4ca_sensor_write(0x002A, 0x030C); 
                s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual 

                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                   
                //여기까지 lens 움직임 없음        
                s5k4ca_sensor_write(0x002A, 0x030E);  
                s5k4ca_sensor_write(0x0F12, set_AF_postion);  // 030E = 00FF 입력. lens 움직임 

                msleep(50);  //lens가 목표지점까지 도달하기 위해 필요한 delay

                ret = 0;
                printk("CAM 3M AF_Single Mode Fail.==> TIMEOUT \n");
            }

            if((tmpVal & 0x3) == 0x02)
            {
                s5k4ca_sensor_write(0xFCFC, 0xD000); 
                s5k4ca_sensor_write(0x0028, 0x7000); 

                s5k4ca_sensor_write(0x002A, 0x030E);  
                s5k4ca_sensor_write(0x0F12, set_Dummy_AF_position);  //030E = 00FF 입력위해 다른값 임시입력

                s5k4ca_sensor_write(0x002A, 0x030C); 
                s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual 

                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                //여기까지 lens 움직임 없음
                s5k4ca_sensor_write(0x002A, 0x030E);  
                s5k4ca_sensor_write(0x0F12, set_AF_postion);  // 030E = 00FF 입력. lens 움직임 

                msleep(50);  //lens가 목표지점까지 도달하기 위해 필요한 delay

                ret = 0;

                printk("CAM 3M AF_Single Mode Fail.==> FAIL \n");
            }

            if(tmpVal & 0x3 == 0x3)
            {	  
                s5k4ca_sensor_write(0x002A, 0x030C); // set to Manual for next snapshot and Single AF
                s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual     

                printk("CAM 3M AF_Single Mode SUCCESS. \r\n");
        	       ret = 1;
            }

            printk("CAM:3M AF_SINGLE SET \r\n");	
            break;
        
        case 2: // auto
            if(af_status == af_running || preview_start == 0)
                break;
            
            printk("[CAM-SENSOR] =Focus Mode -> auto\n"); 

            set_AF_postion = AFPosition[af_position_auto];
            set_Dummy_AF_position = DummyAFPosition[af_position_auto];
            
            s5k4ca_sensor_write(0xFCFC, 0xD000);    
            s5k4ca_sensor_write(0x0028, 0x7000);
            
            s5k4ca_sensor_write(0x002A, 0x161C);    
            s5k4ca_sensor_write(0x0F12, 0x82A8); // Set Normal AF Mode
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, DummyAFPosition[af_position_auto]); // dummy lens position
            
            s5k4ca_sensor_write(0x002A, 0x030C);    
            s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual
            
                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, AFPosition[af_position_auto]); // move lens to initial position
            
            msleep(50); // Lens가 초기 위치로 돌아가는 이동시간.          
            af_mode = 2;
            break;
            
        case 3: // infinity
            if(af_status == af_running|| preview_start == 0)
                break;
        
            printk("[CAM-SENSOR] =Focus Mode -> infinity\n");	
            
            set_AF_postion = AFPosition[af_position_infinity];
            set_Dummy_AF_position = DummyAFPosition[af_position_infinity];            
            
            s5k4ca_sensor_write(0xFCFC, 0xD000);
            s5k4ca_sensor_write(0x0028, 0x7000);
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, DummyAFPosition[af_position_infinity]);
            
            s5k4ca_sensor_write(0x002A, 0x030C);
            s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual
            
                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, AFPosition[af_position_infinity]); // move lens to initial position
            
            msleep(50);

            af_mode = 3;
            break;
            
        case 4: // macro
            if(af_status == af_running|| preview_start == 0)
                break;
        
            printk("[CAM-SENSOR] =Focus Mode -> Macro\n");

            set_AF_postion = AFPosition[af_position_macro];
            set_Dummy_AF_position = DummyAFPosition[af_position_macro];        
            
            s5k4ca_sensor_write(0xFCFC, 0xD000);	
            s5k4ca_sensor_write(0x0028, 0x7000);
            
            s5k4ca_sensor_write(0x002A, 0x161C);
            s5k4ca_sensor_write(0x0F12, 0xA2A8);  // Set Macro AF mode
            
            s5k4ca_sensor_write(0x002A, 0x030E);	
            s5k4ca_sensor_write(0x0F12, DummyAFPosition[af_position_macro]); // set dummy position
            
            
            s5k4ca_sensor_write(0x002A, 0x030C);	
            s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual
            
                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
            
            s5k4ca_sensor_write(0x002A, 0x030E);	
            s5k4ca_sensor_write(0x0F12, AFPosition[af_position_macro]); // move lens to initial position
            
            msleep(50);

            af_mode = 4;
            break;
        
        case 5: // fixed, same as infinity
            if(af_status == af_running|| preview_start == 0)
                break;
        
            printk("[CAM-SENSOR] =Focus Mode -> fixed\n");	

            set_AF_postion = 0x00FF;
            set_Dummy_AF_position = 0x00FE;      
            
            s5k4ca_sensor_write(0xFCFC, 0xD000);
            s5k4ca_sensor_write(0x0028, 0x7000);
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, 0x00FE); // set dummy position
            
            s5k4ca_sensor_write(0x002A, 0x030C);
            s5k4ca_sensor_write(0x0F12, 0x0000); // AF Manual
            
                if (light <= 0x18)
                   msleep(130); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
                else
                   msleep(65); //AF Manual 명령 인식위한 1frame delay (저조도 7.5fps 133ms 고려)
            
            s5k4ca_sensor_write(0x002A, 0x030E);    
            s5k4ca_sensor_write(0x0F12, 0x00FF); // move lens to initial position
            
            msleep(50);

            af_mode = 5;
            break;
            
        default:
            break;
    }

    return ret;
}


static long s5k4ca_set_sensor_mode(int mode)
{
	unsigned short light;

	printk("[CAM-SENSOR] =Sensor Mode ");
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		printk("[PGH]-> Preview \n");
                af_status = af_stop;  
                preview_start = 1;
            // AW & AWB UNLOCK
                s5k4ca_sensor_set_ae_awb_unlock();
                if(af_mode != 2)
                {
                    s5k4ca_sensor_reset_af_position();            
                }
#ifdef I2C_BURST_MODE //dha23 100325
		s5k4ca_sensor_burst_write_list(s5k4ca_preview_04,"s5k4ca_preview_04"); // preview start
#else            
		s5k4ca_sensor_write_list(s5k4ca_preview_04,"s5k4ca_preview_04"); // preview start
#endif		
		mdelay(100);		
		
		break;

	case SENSOR_SNAPSHOT_MODE:
		printk("[PGH}-> Capture \n");		

		s5k4ca_sensor_set_ae_awb_unlock();

		s5k4ca_sensor_write(0x002C, 0x7000);	
		s5k4ca_sensor_write(0x002E, 0x12FE);
		s5k4ca_sensor_read(0x0F12, &light);
		lux_value = light;

             if (previous_scene_mode == 6) /* fireworks use own capture routine */
            {
                printk("Snapshot : firework capture\n");
#ifdef I2C_BURST_MODE //dha23 100325	
                s5k4ca_sensor_burst_write_list(s5k4ca_snapshot_fireworks_04,"s5k4ca_snapshot_fireworks_04");
#else            
                s5k4ca_sensor_write_list(s5k4ca_snapshot_fireworks_04,"s5k4ca_snapshot_fireworks_04");
#endif
            }
            else
            {
                if (light <= 0x18) /* Low light */
                {	
                    printk("Snapshot : Low Light\n");
                    
                    if(previous_scene_mode == 3) 
                    {
                        printk("Snapshot : Night Mode\n");
#ifdef I2C_BURST_MODE //dha23 100325	
                        s5k4ca_sensor_burst_write_list(s5k4ca_snapshot_nightmode_04,"s5k4ca_snapshot_nightmode_04");
#else            
                        s5k4ca_sensor_write_list(s5k4ca_snapshot_nightmode_04,"s5k4ca_snapshot_nightmode_04");
#endif
                    }
                    else
                    {
                        printk("Snapshot : Normal mode \n");
#ifdef I2C_BURST_MODE //dha23 100325
                        s5k4ca_sensor_burst_write_list(s5k4ca_snapshot_normal_low_04,"s5k4ca_snapshot_normal_low_04");
#else            
                        s5k4ca_sensor_write_list(s5k4ca_snapshot_normal_low_04,"s5k4ca_snapshot_normal_low_04");
#endif
                    }
                }
                else
                {
                    {	
                        printk("Snapshot : Normal Normal Light\n");
#ifdef I2C_BURST_MODE //dha23 100325	
                        s5k4ca_sensor_burst_write_list(s5k4ca_capture_normal_normal_04,"s5k4ca_capture_normal_normal_04");
#else            
                        s5k4ca_sensor_write_list(s5k4ca_capture_normal_normal_04,"s5k4ca_capture_normal_normal_04");
#endif
                    }
                }              
              }
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:

		printk("[PGH}-> Capture RAW \n");		
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

int s5k4ca_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	unsigned short	id = 0; //PGH FOR TEST

	printk("[PGH] TEST I2C COMMAND PGH~~\n");

	s5k4ca_sensor_write(0x002C, 0x7000);
	s5k4ca_sensor_write(0x002E, 0x01FA);
	s5k4ca_sensor_read(0x0F12, &id);
	printk("[PGH] SENSOR FW => id 0x%04x \n", id);



	printk("[PGH]  111111 %s/%d \n", __func__, __LINE__);	
#ifdef I2C_BURST_MODE //dha23 100325
	s5k4ca_sensor_burst_write_list(s5k4ca_init0_04,"s5k4ca_init0_04");
#else            
	s5k4ca_sensor_write_list(s5k4ca_init0_04,"s5k4ca_init0_04");
#endif
	msleep(10);	

	printk("[PGH]  22222 %s/%d \n", __func__, __LINE__);	
#ifdef I2C_BURST_MODE //dha23 100325	
	s5k4ca_sensor_burst_write_list(s5k4ca_init1_04,"s5k4ca_init1_04");
#else            
	s5k4ca_sensor_write_list(s5k4ca_init1_04,"s5k4ca_init1_04");
#endif

	return rc;

}

#if 1//PCAM REMOVE OR FIX ME ROUGH CODE
void s5k4ca_rough_control(void __user *arg)
{
	ioctl_pcam_info_8bit		ctrl_info;

	printk("<=PCAM=> %s start\n", __func__);

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}
//	printk("<=PCAM=> TEST %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address, ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);


	switch(ctrl_info.mode)
	{
		case PCAM_EXPOSURE_TIME:
                	s5k4ca_sensor_write(0x002C, 0x7000);
                	s5k4ca_sensor_write(0x002E, 0x1C3C);
                	s5k4ca_sensor_read(0x0F12, &ctrl_info.value_1);
                	printk("[PGH] PCAM_EXPOSURE_TIME : 0x%04x \n", ctrl_info.value_1);
			break;

		case PCAM_ISO_SPEED:
                	s5k4ca_sensor_write(0x002C, 0x7000);
                	s5k4ca_sensor_write(0x002E, 0x12FA);
                	s5k4ca_sensor_read(0x0F12, &ctrl_info.value_1);
                	printk("[PGH] PCAM_ISO_SPEED : 0x%04x \n", ctrl_info.value_1);
			break;

		default :
			printk("<=PCAM=> Unexpected mode on sensor_rough_control!!!\n");
			break;
	}


	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail on copy_to_user!\n", __func__);
	}
	
}
#endif//PCAM


static int cam_hw_init()
{

	int rc = 0;

	printk("<=PCAM=> ++++++++++++++++++++++++++test driver++++++++++++++++++++++++++++++++++++ \n");
//	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //RESET
//	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //STANDBY
//	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //CAM_EN
//	gpio_tlmm_config(GPIO_CFG(123, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //VGA_RESET
//	gpio_tlmm_config(GPIO_CFG(122, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //VGA_STBY

//	gpio_set_value(0, 0);//RESET	
//	gpio_set_value(1, 0);//STBY 
//	gpio_set_value(2, 0);//CAM_EN 
//        gpio_set_value(123, 0);//VGA_RESET
//        gpio_set_value(122, 0); //VGA_STBY
        
	mdelay(1);

//	gpio_set_value(2, 1); //CAM_EN->UP	

	mdelay(3);

//        gpio_set_value(122, 1); //VGA_STBY UP

	mdelay(5);
	gpio_tlmm_config(GPIO_CFG(15, 1,GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE); //VGA_STBY

	printk(KERN_ERR "[WOOGI] : %s: msm_camio_clk_rate_set\n",__func__);    
    
	msm_camio_clk_rate_set(24000000);

	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	mdelay(1);

//        gpio_set_value(123, 1); //VGA_RESET UP

	mdelay(5);

//        gpio_set_value(122, 0); //VGA_STBY DOWN

	mdelay(1);

//	gpio_set_value(0, 1); //RESET UP

	s5k4ca_sensor_write(0x0028, 0xD000);
	s5k4ca_sensor_write(0x002A, 0x0042);
	s5k4ca_sensor_write(0x0F12, 0x00CA);

	mdelay(1);
    
//	gpio_set_value(1, 1); //STBY -> UP

	mdelay(50);

	return rc;
}
#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k4ca_regs_table = NULL;

static int s5k4ca_regs_table_size;

void s5k4ca_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/s5k4ca.h", O_RDONLY, 0);
#else
	filp = filp_open("/sdcard/s5k4ca.h", O_RDONLY, 0);
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

	s5k4ca_regs_table = dp;
	
	s5k4ca_regs_table_size = l;

	*((s5k4ca_regs_table + s5k4ca_regs_table_size) - 1) = '\0';

//	printk("s5k4ca_regs_table 0x%04x, %ld\n", dp, l);
}

void s5k4ca_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5k4ca_regs_table) {
		kfree(s5k4ca_regs_table);
		s5k4ca_regs_table = NULL;
	}	
}

static int s5k4ca_regs_table_write(char *name)
{
	char *start, *end, *reg;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(s5k4ca_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{ 0x");		
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 2), 6);	
			memcpy(data_buf, (reg + 10), 6);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
//			printk("addr 0x%04x, value 0x%04x\n", addr, value);
/*
			if (addr == 0xdddd)
			{
			    if (value == 0x0010)
					mdelay(10);
				else if (value == 0x0020)
					mdelay(20);
				else if (value == 0x0030)
					mdelay(30);
				else if (value == 0x0040)
					mdelay(40);
				else if (value == 0x0050)
					mdelay(50);
				else if (value == 0x0100)
					mdelay(100);
	
				mdelay(value);

				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
*/			
				s5k4ca_sensor_write(addr, value);
		}
	}

	return 0;
}

#endif



int s5k4ca_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
//	int test_value =0; //PGH FOR TEST

	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	

#ifdef CONFIG_LOAD_FILE
	s5k4ca_regs_table_init();
#endif

//        s5k4ca_sensor_gpio_init();
        
	s5k4ca_ctrl = kzalloc(sizeof(struct s5k4ca_ctrl), GFP_KERNEL);
	if (!s5k4ca_ctrl) {
		CDBG("s5k4ca_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5k4ca_ctrl->sensordata = data;


	rc = cam_hw_init();
	if (rc < 0) 
	{
		printk("<=PCAM=> cam_hw_init failed!\n");
		goto init_fail;
	}
#if 0
	gpio_set_value(CAM_STANDBY, 1);
        test_value = gpio_get_value(CAM_STANDBY);
        printk("[PGH] 2 STANDBY  : %d\n",test_value);	

	msleep(1);


	gpio_set_value(CAM_EN, 1);
        test_value = gpio_get_value(CAM_EN);
        printk("[PGH] 2 CAM_EN  : %d\n",test_value);	


	msleep(1);

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
    
	msleep(5);

	msm_camio_camif_pad_reg_reset();

	msleep(1);
	
	gpio_set_value(CAM_RESET, 1);
        test_value = gpio_get_value(CAM_RESET);
        printk("[PGH] 2 RESET  : %d\n",test_value);	

	msleep(40);

//I2C HIGH
	printk("[PGH]  22222  I2C HIGH  %s/%d \n", __func__, __LINE__);	

	gpio_set_value(CAM_I2C_SCL, 1);
	gpio_set_value(CAM_I2C_SDA, 1);	
    
	msleep(40);	
#endif
	rc = s5k4ca_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("s5k4ca_sensor_init failed!\n");
		goto init_fail;
	}
	printk("[PGH]  3333333 %s/%d \n", __func__, __LINE__);	


init_done:
	return rc;

init_fail:
	kfree(s5k4ca_ctrl);
	return rc;
}

int s5k4ca_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k4ca_wait_queue);
	return 0;
}

int s5k4ca_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&s5k4ca_sem); */

	printk("s5k4ca_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = s5k4ca_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = s5k4ca_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_SET_BRIGHTNESS:
			rc = s5k4ca_set_brightness(cfg_data.mode,
						cfg_data.cfg.brightness);
			break;

		case CFG_SET_WB:
			rc = s5k4ca_set_whitebalance(cfg_data.mode,
						cfg_data.cfg.whitebalance);
			break;

		case CFG_SET_ISO:
			rc = s5k4ca_set_ISO(cfg_data.mode,
						cfg_data.cfg.iso);
			break;

		case CFG_SET_EXPOSURE_MODE:
			rc = s5k4ca_set_metering(cfg_data.mode,
						cfg_data.cfg.metering);
			break;

        	case CFG_MOVE_FOCUS:
        		rc = s5k4ca_sensor_af_control(cfg_data.cfg.focus.steps);
        		break;

        	case CFG_SET_DEFAULT_FOCUS:
        		rc = s5k4ca_sensor_af_control(cfg_data.cfg.focus.steps);
        		break;

        	case CFG_GET_AF_MAX_STEPS:
//        		cfg_data.max_steps = MT9T013_TOTAL_STEPS_NEAR_TO_FAR;
        		if (copy_to_user((void *)argp,
        				&cfg_data,
        				sizeof(struct sensor_cfg_data)))
        			rc = -EFAULT;
        		break;

		default:
//			rc = -EINVAL;
                        rc = 0;
			break;
		}

	/* up(&s5k4ca_sem); */

	return rc;
}

int s5k4ca_sensor_release(void)
{
	int rc = 0;
        preview_start = 0;
	/* down(&s5k4ca_sem); */

	kfree(s5k4ca_ctrl);
	/* up(&s5k4ca_sem); */

	return rc;
}

int s5k4ca_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k4ca_sensorw =
		kzalloc(sizeof(struct s5k4ca_work), GFP_KERNEL);

	if (!s5k4ca_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k4ca_sensorw);
	s5k4ca_init_client(client);
	s5k4ca_client = client;

	printk("[PGH] s5k4ca_probe succeeded!  %s/%d \n", __func__, __LINE__);	

	CDBG("s5k4ca_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(s5k4ca_sensorw);
	s5k4ca_sensorw = NULL;
	CDBG("s5k4ca_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k4ca_i2c_id[] = {
	{ "s5k4ca", 0},
	{ },
};

static struct i2c_driver s5k4ca_i2c_driver = {
	.id_table = s5k4ca_i2c_id,
	.probe  = s5k4ca_i2c_probe,
	.remove = __exit_p(s5k4ca_i2c_remove),
	.driver = {
		.name = "s5k4ca",
	},
};

int s5k4ca_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{

	int rc = i2c_add_driver(&s5k4ca_i2c_driver);
	if (rc < 0 || s5k4ca_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	
	printk("[PGH]  22222 %s/%d \n", __func__, __LINE__);	



	printk("[PGH] s5k4ca_client->addr : %x\n", s5k4ca_client->addr);
	printk("[PGH] s5k4ca_client->adapter->nr : %d\n", s5k4ca_client->adapter->nr);


	printk(KERN_ERR "[WOOGI] : %s: msm_camio_clk_rate_set\n",__func__);    

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

#if 0//bestiq
	rc = s5k4ca_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif

	s->s_init = s5k4ca_sensor_init;
	s->s_release = s5k4ca_sensor_release;
	s->s_config  = s5k4ca_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

int __s5k4ca_probe(struct platform_device *pdev)
{
	printk("[PGH]  %s/%d \n", __func__, __LINE__);	
	return msm_camera_drv_start(pdev, s5k4ca_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k4ca_probe,
	.driver = {
		.name = "msm_camera_s5k4ca",
		.owner = THIS_MODULE,
	},
};

int __init s5k4ca_init(void)
{
	printk("[PGH]  %s/%d \n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k4ca_init);
