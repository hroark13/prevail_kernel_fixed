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
//bestiq TEST s5ka3d
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "s5ka3d.h"

#include <mach/camera.h>//bestiq

#define SENSOR_DEBUG 0
//#define CONFIG_LOAD_FILE 0

//aswoogi add
#define CAM_RESET 0
#define CAM_STANDBY 1
#define CAM_EN 2
#define CAM_I2C_SCL 60
#define CAM_I2C_SDA 61

struct s5ka3d_work {
	struct work_struct work;
};

static struct  s5ka3d_work *s5ka3d_sensorw;
static struct  i2c_client *s5ka3d_client;

struct s5ka3d_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct s5ka3d_ctrl *s5ka3d_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(s5ka3d_wait_queue);
DECLARE_MUTEX(s5ka3d_sem);

#if 0//def CONFIG_LOAD_FILE
static int s5ka3d_regs_table_write(char *name);
#endif
static int cam_hw_init(void);
extern int s5k4ca_sensor_write(unsigned short subaddr, unsigned short val);


//static int16_t s5ka3d_effect = CAMERA_EFFECT_OFF;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5ka3d_reg s5ka3d_regs;


/*=============================================================*/

//aswoogi add
#if 0
static int s5ka3d_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = { s5ka3d_client->addr, 0, 1, buf };
	
	buf[0] = subaddr;
//	buf[1] = 0x0;

	ret = i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

//	*data = ((buf[0] << 8) | buf[1]);
	*data = buf[0];

error:
	return ret;
}
#endif

static int s5ka3d_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = {0};
	struct i2c_msg msg = { s5ka3d_client->addr, 0, 2, buf };

	buf[0] = subaddr;
	buf[1] = val;

//	return i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
        if(i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1)
        {
            return 0;
        }
        else
        {
            printk("[s5ka3d] s5ka3d_sensor_write fail \n");        
            return -EIO;
        }
}

static int s5ka3d_sensor_write2(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { 0x3C, 0, 4, buf };

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

        if(i2c_transfer(s5ka3d_client->adapter, &msg, 1) == 1)
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
static int s5ka3d_sensor_write_list(struct samsung_short_t *list,int size, char *name)
{

	int ret = 0;
	int i;


	for (i = 0; i < size; i++)
	{
		if(s5ka3d_sensor_write(list[i].subaddr, list[i].value) < 0)
		{
			printk("<=PCAM=> sensor_write_list fail...-_-\n");
			return -1;
		}
	}
	return ret;
}



static long s5ka3d_set_sensor_mode(int mode)
{
	//long rc = 0;

	printk("[CAM-SENSOR] =Sensor Mode ");
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		printk("[PGH]-> Preview \n");
		s5ka3d_sensor_write_list(reg_preview, sizeof(reg_preview)/\
		sizeof(reg_preview[0]),"reg_preview"); // preview start
		mdelay(100);		
		
		break;

	case SENSOR_SNAPSHOT_MODE:
		printk("[PGH}-> Capture \n");		
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		printk("[PGH}-> Capture RAW \n");		
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int s5ka3d_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk("%s/%d \n", __func__, __LINE__);	

	s5ka3d_sensor_write_list(reg_vt_init, sizeof(reg_vt_init)/\
		sizeof(reg_vt_init[0]),"reg_vt_init");

	msleep(10);	

	return rc;

}
static int cam_hw_init()
{

	int rc = 0;

	printk("<=PCAM=> ++++++++++++++++++++++++++test driver++++++++++++++++++++++++++++++++++++ \n");
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //RESET
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //STANDBY
	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //CAM_EN
//	gpio_tlmm_config(GPIO_CFG(123, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //VGA_RESET
//	gpio_tlmm_config(GPIO_CFG(122, 0,GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE); //VGA_STBY

	gpio_set_value(0, 0);//RESET	
	gpio_set_value(1, 0);//STBY 
	gpio_set_value(2, 0);//CAM_EN 
//        gpio_set_value(123, 0);//VGA_RESET
//        gpio_set_value(122, 0); //VGA_STBY
        
	mdelay(1);

	gpio_set_value(2, 1); //CAM_EN->UP	

	mdelay(1);

//        gpio_set_value(122, 1); //VGA_STBY UP

	mdelay(1);

        gpio_tlmm_config(GPIO_CFG(15, 1,GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE); //VGA_STBY

	msm_camio_clk_rate_set(24000000);

	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	mdelay(1);

        gpio_set_value(0, 1); //3M_RESET UP

	mdelay(1);

	s5ka3d_sensor_write2(0x0028, 0xD000);
	s5ka3d_sensor_write2(0x002A, 0x0042);
	s5ka3d_sensor_write2(0x0F12, 0x00CA);

	mdelay(10);

//        gpio_set_value(123, 1); //VGA_RESET UP

	mdelay(50);

	return rc;
}

int s5ka3d_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
//	int test_value =0; //PGH FOR TEST

	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	

#if 0 //def CONFIG_LOAD_FILE
	s5ka3d_regs_table_init();
#endif

//        s5ka3d_sensor_gpio_init();
        
	s5ka3d_ctrl = kzalloc(sizeof(struct s5ka3d_ctrl), GFP_KERNEL);
	if (!s5ka3d_ctrl) {
		CDBG("s5ka3d_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5ka3d_ctrl->sensordata = data;


	rc = cam_hw_init();
	if (rc < 0) 
	{
		printk("<=PCAM=> cam_hw_init failed!\n");
		goto init_fail;
	}

	rc = s5ka3d_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("s5ka3d_sensor_init failed!\n");
		goto init_fail;
	}
	printk("[PGH]  3333333 %s/%d \n", __func__, __LINE__);	


init_done:
	return rc;

init_fail:
	kfree(s5ka3d_ctrl);
	return rc;
}

static int s5ka3d_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5ka3d_wait_queue);
	return 0;
}

int s5ka3d_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&s5ka3d_sem); */

	CDBG("s5ka3d_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = s5ka3d_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
//			rc = -EINVAL;
                        rc = 0;
			break;
		}

	/* up(&s5ka3d_sem); */

	return rc;
}

int s5ka3d_sensor_release(void)
{
	int rc = 0;

	/* down(&s5ka3d_sem); */

	kfree(s5ka3d_ctrl);
	/* up(&s5ka3d_sem); */

	return rc;
}

static int s5ka3d_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5ka3d_sensorw =
		kzalloc(sizeof(struct s5ka3d_work), GFP_KERNEL);

	if (!s5ka3d_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5ka3d_sensorw);
	s5ka3d_init_client(client);
	s5ka3d_client = client;

	printk("[PGH] s5ka3d_probe succeeded!  %s/%d \n", __func__, __LINE__);	

	CDBG("s5ka3d_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(s5ka3d_sensorw);
	s5ka3d_sensorw = NULL;
	CDBG("s5ka3d_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5ka3d_i2c_id[] = {
	{ "s5ka3d", 0},
	{ },
};

static struct i2c_driver s5ka3d_i2c_driver = {
	.id_table = s5ka3d_i2c_id,
	.probe  = s5ka3d_i2c_probe,
	.remove = __exit_p(s5ka3d_i2c_remove),
	.driver = {
		.name = "s5ka3d",
	},
};

static int s5ka3d_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{

	int rc = i2c_add_driver(&s5ka3d_i2c_driver);
	if (rc < 0 || s5ka3d_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	printk("[PGH]  11111 %s/%d \n", __func__, __LINE__);	
	printk("[PGH]  22222 %s/%d \n", __func__, __LINE__);	



	printk("[PGH] s5ka3d_client->addr : %x\n", s5ka3d_client->addr);
	printk("[PGH] s5ka3d_client->adapter->nr : %d\n", s5ka3d_client->adapter->nr);



	/* Input MCLK = 24MHz */
//	msm_camio_clk_rate_set(24000000);
//	mdelay(5);

#if 0//bestiq
	rc = s5ka3d_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif

	s->s_init = s5ka3d_sensor_init;
	s->s_release = s5ka3d_sensor_release;
	s->s_config  = s5ka3d_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __s5ka3d_probe(struct platform_device *pdev)
{
	printk("[PGH]  %s/%d \n", __func__, __LINE__);	
	return msm_camera_drv_start(pdev, s5ka3d_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5ka3d_probe,
	.driver = {
		.name = "msm_camera_s5ka3d",
		.owner = THIS_MODULE,
	},
};

static int __init s5ka3d_init(void)
{
	printk("[PGH]  %s/%d \n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5ka3d_init);
