/*
* Copyright   2011 Samsung Electronics. All rights reserved.
* 
* Copyright   2010 STMicroelectronics.
* Author: matteo dameno <mailto:matteo.dameno@st.com>
* Author: carmine iascone <mailto:carmine.iascone@st.com>
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
* GNU General Public License for more details.
*
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/i2c-algo-bit.h>
#include <linux/wakelock.h>
#include <linux/input.h>

#include <linux/earlysuspend.h>

#define STM_DEBUG
//#define WORK_DEBUG

#define ACC_DEV_MAJOR 241
/* KR3DM I2C Slave Address */
//#define KR3DM_I2C_ADDR    0x10

static int cal_data[3]={0,0,0};
static struct file *cal_filp = NULL;

#define KR3DM_IOCTL_BASE 'a'
/** The following define the IOCTL command values via the ioctl macros */
#define KR3DM_IOCTL_SET_DELAY       _IOW(KR3DM_IOCTL_BASE, 0, int)
#define KR3DM_IOCTL_GET_DELAY       _IOR(KR3DM_IOCTL_BASE, 1, int)
#define KR3DM_IOCTL_SET_ENABLE      _IOW(KR3DM_IOCTL_BASE, 2, int)
#define KR3DM_IOCTL_GET_ENABLE      _IOR(KR3DM_IOCTL_BASE, 3, int)
#define KR3DM_IOCTL_SET_G_RANGE     _IOW(KR3DM_IOCTL_BASE, 4, int)
#define KR3DM_IOCTL_READ_RAW_XYZ  _IOW(KR3DM_IOCTL_BASE, 7, int)
#define KR3DM_IOCTL_READ_ACCEL_XYZ  _IOW(KR3DM_IOCTL_BASE, 8, int)
#define KR3DM_IOCTL_SET_CALDATA  _IOW(KR3DM_IOCTL_BASE, 9, int)

#define OFFSET_PATH	"/sys/class/accelerometer/accelerometer/cal_offset"

//kr3dm registers
#define WHO_AM_I    0x0F
/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1       0x20    /* power control reg */
#define CTRL_REG2       0x21    /* power control reg */
#define CTRL_REG3       0x22    /* power control reg */
#define CTRL_REG4       0x23    /* interrupt control reg */
#define CTRL_REG5       0x24    /* interrupt control reg */
#define AXISDATA_REG    0x28

#define OUT_X_L			0x29
#define OUT_X_H			0x2A
#define OUT_Y_L			0x2B
#define OUT_Y_H			0x2C
#define OUT_Z_L			0x2D
#define OUT_Z_H			0x2E

#define KR3DM_G_2G    0x00
#define KR3DM_G_4G    0x10
#define KR3DM_G_8G    0x30

#define PM_OFF            0x00
#define PM_NORMAL         0x20
#define ENABLE_ALL_AXES   0x07

#define ODRHALF           0x40  /* 0.5Hz output data rate */
#define ODR1              0x60  /* 1Hz output data rate */
#define ODR2              0x80  /* 2Hz output data rate */
#define ODR5              0xA0  /* 5Hz output data rate */
#define ODR10             0xC0  /* 10Hz output data rate */
#define ODR50             0x00  /* 50Hz output data rate */
#define ODR100            0x08  /* 100Hz output data rate */
#define ODR400            0x10  /* 400Hz output data rate */

extern unsigned char hw_version;
#if defined(CONFIG_MACH_ROOKIE)
#define USE_I2C_SMBUS ( hw_version >= 4 ? 0 : 1)
#elif defined(CONFIG_MACH_ESCAPE)
#define USE_I2C_SMBUS ( hw_version >= 1 ? 0 : 1)
#else
#define USE_I2C_SMBUS 1
#endif


/** KR3DM acceleration data 
  \brief Structure containing acceleration values for x,y and z-axis in signed short

*/

typedef struct  
{
  signed char	x, /**< holds x-axis acceleration data sign extended. Range -128 to +127. */
		y, /**< holds y-axis acceleration data sign extended. Range -128 to +127. */
		z; /**< holds z-axis acceleration data sign extended. Range -128 to +127. */

} kr3dm_acc_t;

typedef struct  
{
  int	x, /**< holds x-axis acceleration data sign extended. Range -128 to +127. */
		y, /**< holds y-axis acceleration data sign extended. Range -128 to +127. */
		z; /**< holds z-axis acceleration data sign extended. Range -128 to +127. */

} kr3dm_cal_t;

static struct i2c_client *kr3dm_client = NULL;

typedef struct
{
 	struct work_struct work_acc;
	struct hrtimer timer;
	ktime_t acc_poll_delay;
	u8 state;
	struct mutex power_lock;
	struct workqueue_struct *wq;
	struct input_dev *acc_input_dev;
} kr3dm_t;

struct kr3dm_data{
	struct work_struct work_acc;
	struct hrtimer timer;
	ktime_t acc_poll_delay;
	u8 state;
	struct mutex power_lock;
	struct workqueue_struct *wq;
	struct input_dev *acc_input_dev;
	kr3dm_t			kr3dm;
	struct early_suspend early_suspend;
	struct i2c_client client;
};

struct device *kr3dm_dev_t;
EXPORT_SYMBOL(kr3dm_dev_t);

struct class *kr3d_dev_class;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kr3dm_early_suspend(struct early_suspend *h);
static void kr3dm_late_resume(struct early_suspend *h);
#endif

static char kr3dm_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char kr3dm_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);


struct kr3dm_data * g_kr3dm;

#define ACC_ENABLED 1

// this proc file system's path is "/proc/driver/kr3dm"
// usage :	(at the path) type "cat kr3dm" , it will show short information for current accelation
// 			use it for simple working test only

//#define KR3DM_PROC_FS

#ifdef KR3DM_PROC_FS

#include <linux/proc_fs.h>

#define DRIVER_PROC_ENTRY		"driver/kr3dm"
static void kr3dm_acc_enable(void);

static int kr3dm_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *p = page;
	int len = 0;
	
	mutex_lock(&g_kr3dm->power_lock);
	printk("kr3dm_proc_read\n");

	g_kr3dm->state |= ACC_ENABLED;
	kr3dm_acc_enable();

	mutex_unlock(&g_kr3dm->power_lock);

	len = (p - page) - off;
	if (len < 0) {
		len = 0;
	}

	printk("kr3dm_proc_read: success full\n");

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;
	return len;
}
#endif	//KR3DM_PROC_FS


/*************************************************************************/
/*		KR3DM Functions	  				         */
/*************************************************************************/
// set kr3dm bandwidth
int kr3dm_set_bandwidth(char bw) 
{
	int res = 0;
	unsigned char data = 0;
	
	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
	if (res>=0){         
		data = res & 0x00e7;
	}
	data = data + bw;
	
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);
	return res;
}

// read selected bandwidth from kr3dm 
int kr3dm_get_bandwidth(unsigned char *bw) {
	int res = 1;
	//TO DO 
	return res;
}

int kr3dm_set_enable(char mode)
{
	int res = 0;
	unsigned char data = 0;

	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
	if (res>=0){         
		data = res & 0x001f;
	}
	data = mode + data;
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);
	return res;
}

/** X,Y and Z-axis acceleration data readout 
  \param *acc pointer to \ref kr3dm_acc_t structure for x,y,z data readout
  \note data will be read by multi-byte protocol into a 6 byte structure 
*/

// iamaj ec27 add
#define KR3DM_REF 5
#define KR3DM_X 0
#define KR3DM_Y 1
#define KR3DM_Z 2

int kr3dmData[3][KR3DM_REF+1]={0,};  // iamaj ec27
int dataPo=0;  // iamaj ec24

void kr3dm_open_calibration()
{
	mm_segment_t old_fs;
	char cal_buffer[256]={0,};
	int err = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	cal_filp = filp_open(OFFSET_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}
	err = cal_filp->f_op->read(cal_filp,cal_buffer, 256, &cal_filp->f_pos);

//	printk("[KR3DM_read_accel_xyz] cal_buffer string = %s \n", cal_buffer);
	
	if(cal_buffer[0]!='x') {
		cal_data[0] = 0;
		cal_data[1] = 0;
		cal_data[2] = 0;
	} else {
		sscanf(cal_buffer,"x:%d,y:%d,z:%d", &cal_data[0], &cal_data[1], &cal_data[2]);
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);
}

int kr3dm_read_raw_xyz(kr3dm_acc_t * acc)
{

#if 0
  int res;
  unsigned char acc_data[6];

printk(KERN_INFO"%s\n", __FUNCTION__);
  res = kr3dm_i2c_read(AXISDATA_REG,&acc_data[0], 6);
  
/*   
	acc->x = (short) (((acc_data[1]) << 8) | acc_data[0]);
	acc->y = (short) (((acc_data[3]) << 8) | acc_data[2]);
	acc->z = (short) (((acc_data[5]) << 8) | acc_data[4]);

	acc->x = acc->x/16;
	acc->y = acc->y/16;
	acc->z = acc->z/16;

*/

	acc->x= acc_data[1];
	acc->y= acc_data[3];
	acc->z= acc_data[5];	
	
  return res;

#else


	int comres;
	unsigned char data[3];
	int KisChangeX=1,KisChangeY=1,KisChangeZ=1, i;  // iamaj ec27

//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_X_L, &data[0], 1);
//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Y_L, &data[1], 1);
//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Z_L, &data[2], 1);

	comres =kr3dm_i2c_read(OUT_X_L,&data[0], 1);
	comres =kr3dm_i2c_read(OUT_Y_L,&data[1], 1);
	comres =kr3dm_i2c_read(OUT_Z_L,&data[2], 1);

	// 2' complement
	data[0] = (~data[0] + 1);
	data[1] = (~data[1] + 1);
	data[2] = (~data[2] + 1);
	
    #if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_GIO)  || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_RANT3) // -90 degree ( y -> x, x -> -y)
	if(data[1]& 0x80)
		acc->x = (0x100-data[1]) *(-1);
	else
		acc->x = ((data[1]) & 0xFF);
    #elif (defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS))
	if(data[1]& 0x80)
		acc->x = (0x100-data[1]);
	else
		acc->x = ((data[1]) & 0xFF) *(-1);
	#else
	if(data[0] & 0x80)
		acc->x = (0x100-data[0]);
	else
		acc->x = ((data[0]) & 0xFF)*(-1);
	#endif
	
	#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_GIO)
	if(data[0] & 0x80)
		acc->y = (0x100-data[0]);
	else
		acc->y = ((data[0]) & 0xFF)*(-1);
	#elif defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
	if(data[0] & 0x80)
		acc->y = (0x100-data[0])*(-1);
	else
		acc->y = ((data[0]) & 0xFF);
	#else
	if(data[1]& 0x80)
	acc->y = (0x100-data[1]);
	else
	acc->y = ((data[1]) & 0xFF)*(-1);
	#endif
	
	#if defined(CONFIG_MACH_ESCAPE)  || defined(CONFIG_MACH_RANT3)
	if(data[2]& 0x80)
	acc->z = (0x100-data[2])*(-1);
	else
	acc->z = ((data[2]) & 0xFF);
	#else
	if(data[2]& 0x80)
	acc->z = (0x100-data[2]);
	else
	acc->z = ((data[2]) & 0xFF)*(-1);
	#endif


// iamaj ec27 [[
    if(++dataPo == KR3DM_REF) dataPo =0;
    kr3dmData[KR3DM_X][dataPo] = acc->x;
    kr3dmData[KR3DM_Y][dataPo] = acc->y;
    kr3dmData[KR3DM_Z][dataPo] = acc->z;
		
	for(i=0; i<KR3DM_REF; i++){
		if(kr3dmData[KR3DM_X][i] == kr3dmData[KR3DM_X][KR3DM_REF]) KisChangeX = 0;
		if(kr3dmData[KR3DM_Y][i] == kr3dmData[KR3DM_Y][KR3DM_REF]) KisChangeY = 0;
		if(kr3dmData[KR3DM_Z][i] == kr3dmData[KR3DM_Z][KR3DM_REF]) KisChangeZ = 0;
	}

	if(	KisChangeX) kr3dmData[KR3DM_X][KR3DM_REF] = acc->x;
		else acc->x = kr3dmData[KR3DM_X][KR3DM_REF];
	if(	KisChangeY) kr3dmData[KR3DM_Y][KR3DM_REF] = acc->y;
		else acc->y = kr3dmData[KR3DM_Y][KR3DM_REF];
	if(	KisChangeZ) kr3dmData[KR3DM_Z][KR3DM_REF] = acc->z;
		else acc->z = kr3dmData[KR3DM_Z][KR3DM_REF];
// ]]

#if 0
	printk("[KR3DM] x = %d  /  y =  %d  /  z = %d converted data \n", acc->x, acc->y, acc->z ); //kimhyuns_temp
#endif	
	return comres;	

#endif
  
}



int kr3dm_read_accel_xyz(kr3dm_acc_t * acc)
{

#if 0
  int res;
  unsigned char acc_data[6];

printk(KERN_INFO"%s\n", __FUNCTION__);
  res = kr3dm_i2c_read(AXISDATA_REG,&acc_data[0], 6);
  
/*   
	acc->x = (short) (((acc_data[1]) << 8) | acc_data[0]);
	acc->y = (short) (((acc_data[3]) << 8) | acc_data[2]);
	acc->z = (short) (((acc_data[5]) << 8) | acc_data[4]);

	acc->x = acc->x/16;
	acc->y = acc->y/16;
	acc->z = acc->z/16;

*/

	acc->x= acc_data[1];
	acc->y= acc_data[3];
	acc->z= acc_data[5];	
	
  return res;

#else


	int comres;
	unsigned char data[3];
	int KisChangeX=1,KisChangeY=1,KisChangeZ=1, i;  // iamaj ec27
	int temp[3]={0,0,0};
	int err = 0;

//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_X_L, &data[0], 1);
//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Y_L, &data[1], 1);
//	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Z_L, &data[2], 1);

	comres =kr3dm_i2c_read(OUT_X_L,&data[0], 1);
	comres =kr3dm_i2c_read(OUT_Y_L,&data[1], 1);
	comres =kr3dm_i2c_read(OUT_Z_L,&data[2], 1);

	// 2' complement
	data[0] = (~data[0] + 1);
	data[1] = (~data[1] + 1);
	data[2] = (~data[2] + 1);
	
    #if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_GIO)  || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_RANT3) // -90 degree ( y -> x, x -> -y)
	if(data[1]& 0x80)
		acc->x = (0x100-data[1]) *(-1);
	else
		acc->x = ((data[1]) & 0xFF);
    #elif defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	if(data[1]& 0x80)
		acc->x = (0x100-data[1]);
	else
		acc->x = ((data[1]) & 0xFF) *(-1);
	#else
	if(data[0] & 0x80)
		acc->x = (0x100-data[0]);
	else
		acc->x = ((data[0]) & 0xFF)*(-1);
	#endif
	
	#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_GIO)
	if(data[0] & 0x80)
		acc->y = (0x100-data[0]);
	else
		acc->y = ((data[0]) & 0xFF)*(-1);
	#elif defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
	if(data[0] & 0x80)
		acc->y = (0x100-data[0])*(-1);
	else
		acc->y = ((data[0]) & 0xFF);
	#else
	if(data[1]& 0x80)
	acc->y = (0x100-data[1]);
	else
	acc->y = ((data[1]) & 0xFF)*(-1);
	#endif
	
	#if defined(CONFIG_MACH_ESCAPE)  || defined(CONFIG_MACH_RANT3)
	if(data[2]& 0x80)
	acc->z = (0x100-data[2])*(-1);
	else
	acc->z = ((data[2]) & 0xFF);
	#else
	if(data[2]& 0x80)
	acc->z = (0x100-data[2]);
	else
	acc->z = ((data[2]) & 0xFF)*(-1);
	#endif

#if defined(CONFIG_MACH_GIO)
	temp[0] = (int)acc->x;
	temp[1] = (int)acc->y;
	temp[2] = (int)acc->z;

	temp[0] -= cal_data[0];
	temp[1] -= cal_data[1];
	temp[2] -= cal_data[2];
	//printk("[kr3dm] accelerometer data(adjust cal_offset) x: %d, y:%d, z:%d\n", temp[0], temp[1], temp[2]);

	acc->x = (char)temp[0];
	acc->y = (char)temp[1];
	acc->z = (char)temp[2];
#endif
	
	
// iamaj ec27 [[
    if(++dataPo == KR3DM_REF) dataPo =0;
    kr3dmData[KR3DM_X][dataPo] = acc->x;
    kr3dmData[KR3DM_Y][dataPo] = acc->y;
    kr3dmData[KR3DM_Z][dataPo] = acc->z;
		
	for(i=0; i<KR3DM_REF; i++){
		if(kr3dmData[KR3DM_X][i] == kr3dmData[KR3DM_X][KR3DM_REF]) KisChangeX = 0;
		if(kr3dmData[KR3DM_Y][i] == kr3dmData[KR3DM_Y][KR3DM_REF]) KisChangeY = 0;
		if(kr3dmData[KR3DM_Z][i] == kr3dmData[KR3DM_Z][KR3DM_REF]) KisChangeZ = 0;
	}

	if(	KisChangeX) kr3dmData[KR3DM_X][KR3DM_REF] = acc->x;
		else acc->x = kr3dmData[KR3DM_X][KR3DM_REF];
	if(	KisChangeY) kr3dmData[KR3DM_Y][KR3DM_REF] = acc->y;
		else acc->y = kr3dmData[KR3DM_Y][KR3DM_REF];
	if(	KisChangeZ) kr3dmData[KR3DM_Z][KR3DM_REF] = acc->z;
		else acc->z = kr3dmData[KR3DM_Z][KR3DM_REF];
// ]]

#if 0
	printk("[KR3DM] x = %d  /  y =  %d  /  z = %d converted data \n", acc->x, acc->y, acc->z ); //kimhyuns_temp
#endif	
	return comres;	

#endif
  
}


/* Device Initialization  */
int device_init(void/*kr3dm_t *pLis*/) 
{
	int res;
	unsigned char buf[5];
	buf[0]=0x27;
	buf[1]=0x00;
	buf[2]=0x00;
	buf[3]=0x00;
	buf[4]=0x00;
	res = kr3dm_i2c_write(CTRL_REG1, &buf[0], 5);
	return res;
}

int kr3dm_set_range(char range) 
{     
	int err = 0;
	unsigned char buf[2];
	
	buf[0] = range;
	err = kr3dm_i2c_write(CTRL_REG4, buf, 1);
	return err;
}

/*************************************************************************/
/*		KR3DM I2C_API	  				         */
/*************************************************************************/

/*  i2c write routine for kr3dm */
static char kr3dm_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{

	if(USE_I2C_SMBUS)
	{
		int dummy;
		int i; 
		//printk(KERN_INFO"%s\n", __FUNCTION__);
		if( kr3dm_client == NULL )  /*  No global client pointer? */
			return -1;
		for(i=0;i<len;i++)
		{ 
			dummy = i2c_smbus_write_byte_data(kr3dm_client, reg_addr++, data[i]);
			if(dummy)
			{
				printk(KERN_INFO"i2c write error\n");
				return dummy; 
			}
		}
	} else{
		uint8_t i;
		unsigned char tmp[2];
		tmp[0] = reg_addr;
		tmp[1] = *data;
		
		struct i2c_msg msg[] = {
			{
				.addr	= kr3dm_client->addr,
				.flags	= 0,
				.len	= 2,
				.buf	= tmp,
			}
		};
		
		for (i = 0; i < 3; i++) {
			if (i2c_transfer(kr3dm_client->adapter, msg, 1) >= 0) {
				break;
			}
			mdelay(10);
		}

		if (i >= 3) {
			pr_err("%s: I2C Fail error\n", __FUNCTION__);
			return -EIO;
		}

	}
	return 0;
}

/*  i2c read routine for kr3dm  */
static char kr3dm_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	if(USE_I2C_SMBUS)
	{
		int dummy=0;
		int i=0;
		//printk(KERN_INFO"%s\n", __FUNCTION__);
		if( kr3dm_client == NULL )  /*  No global client pointer? */
			return -1;
		while(i<len)
		{        
			dummy = i2c_smbus_read_word_data(kr3dm_client, reg_addr++);
			if (dummy>=0)
			{         
				data[i] = dummy & 0x00ff;
				i++;
			} 
			else
			{
				printk(KERN_INFO" i2c read error\n "); 
				return dummy;
			}
			dummy = len;
		}
		return dummy;
	}else{
		uint8_t i;
		struct i2c_msg msg[1];
		unsigned char data_v[1];
		unsigned char val[2];
		int err;

		msg->addr	= kr3dm_client->addr;
		msg->flags	= 0;
		msg->len	= 1;
		msg->buf	= data_v;
		*data_v		= reg_addr;
		
		err = i2c_transfer(kr3dm_client->adapter, msg, 1); 

		if (err < 0) {
			pr_err("%s: I2C Fail error\n", __FUNCTION__);
			return -EIO;
		}

		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = data;

		err = i2c_transfer(kr3dm_client->adapter, msg, 1);

		if (err < 0) {
			pr_err("%s: I2C Fail error\n", __FUNCTION__);
			return -EIO;
		}	
	}
	
	return 0;	
}


/*************************************************************************/
/*		KR3DM Sysfs	  				         */
/*************************************************************************/
//TEST
static ssize_t kr3dm_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	kr3dm_acc_t acc;

	kr3dm_open_calibration();
	kr3dm_read_accel_xyz(&acc);
	
	printk("[kr3dm_fs_read-acc_file] x: %d,y: %d,z: %d\n", acc.x, acc.y, acc.z);
	count = sprintf(buf,"%d,%d,%d\n", acc.x, acc.y, acc.z );

	return count;
}

static ssize_t kr3dm_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
//	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(acc_file, S_IRUGO | S_IWUSR | S_IWGRP, kr3dm_fs_read, kr3dm_fs_write);


/*************************************************************************/
/*		KR3DM operation functions			         */
/*************************************************************************/
/*  read command for KR3DM device file  */
static ssize_t kr3dm_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	kr3dm_acc_t acc;  
	if( kr3dm_client == NULL )
		return -1;
	kr3dm_read_accel_xyz(&acc);

#ifdef STM_DEBUG
	printk(KERN_INFO "KR3DM Accel: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#ifdef STM_DEBUG
	printk(KERN_INFO "KR3DM Accel: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*  write command for KR3DM device file */
static ssize_t kr3dm_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( kr3dm_client == NULL )
		return -1;
#ifdef STM_DEBUG
	printk(KERN_INFO "KR3DM Accel should be accessed with ioctl command\n");
#endif
	return 0;
}


/*  open command for KR3DM device file  */
static int kr3dm_open(struct inode *inode, struct file *file)
{
#ifdef STM_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif
	if( kr3dm_client == NULL)
	{
#ifdef STM_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}
	device_init();
	
#ifdef STM_DEBUG
	printk(KERN_INFO "KR3DM Accel has been opened\n");
#endif

	kr3dm_open_calibration();

	return 0;
}


/*  release command for KR3DM device file */
static int kr3dm_close(struct inode *inode, struct file *file)
{
#ifdef STM_DEBUG
	printk(KERN_INFO "KR3DM Accel has been closed\n");
#endif
	return 0;
}


/*  ioctl command for KR3DM device file */
static int kr3dm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	mm_segment_t old_fs;
	
#ifdef STM_DEBUG
	//printk("kr3dm_ioctl\n");  
#endif
	
	// check kr3dm_client 
	if( kr3dm_client == NULL)
	{
#ifdef STM_DEBUG
		printk("I2C driver not install\n"); 
#endif
		return -EFAULT;
	}
	
	/* cmd mapping */
	
	switch(cmd)
	{
		
		/*case KR3DM_SELFTEST:
		//TO DO
		return err;*/
		
		case KR3DM_IOCTL_SET_G_RANGE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_from_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dm_set_range(*data);
			return err;
		
		case KR3DM_IOCTL_SET_ENABLE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0){
#ifdef STM_DEBUG
				printk("copy_to_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dm_set_enable(*data);
			return err;
		
		case KR3DM_IOCTL_SET_DELAY:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_from_user error\n");
#endif
				return -EFAULT;
			}
			err = kr3dm_set_bandwidth(*data);
			return err;
		
		case KR3DM_IOCTL_GET_DELAY:
			err = kr3dm_get_bandwidth(data);
			if(copy_to_user((unsigned char*)arg,data,1)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_to_user error\n");
#endif
				return -EFAULT;
			}
			return err;

	case KR3DM_IOCTL_READ_RAW_XYZ:
#ifdef STM_DEBUG
//			printk("[KR3DM] KR3DM_IOCTL_READ_RAW_XYZ !!!\r\n");
#endif
			err = kr3dm_read_raw_xyz((kr3dm_acc_t*)data);
			if(copy_to_user((kr3dm_acc_t*)arg,(kr3dm_acc_t*)data,3)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
			return err;

		
		case KR3DM_IOCTL_READ_ACCEL_XYZ:
#ifdef STM_DEBUG
//			printk("[KR3DM] KR3DM_IOCTL_READ_ACCEL_XYZ !!!\r\n");
#endif

			err = kr3dm_read_accel_xyz((kr3dm_acc_t*)data);
			if(copy_to_user((kr3dm_acc_t*)arg,(kr3dm_acc_t*)data,3)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
			return err;


		case KR3DM_IOCTL_SET_CALDATA:
			
			if(copy_from_user((kr3dm_cal_t*)data,(kr3dm_cal_t*)arg,3)!=0)
			{
#ifdef STM_DEBUG
				printk("copy_from_user error\n");
#endif
				return -EFAULT;
			}
			cal_data[0] = ((kr3dm_cal_t*)arg)->x;
			cal_data[1] = ((kr3dm_cal_t*)arg)->y;
			cal_data[2] = ((kr3dm_cal_t*)arg)->z;
		
			printk("\n KR3DM_IOCTL_SET_CALDATA %d %d %d \n",cal_data[0], cal_data[1], cal_data[2]);
			return;

		
		default:
			return 0;
	}
}

static const struct file_operations kr3dm_fops = {
	.owner = THIS_MODULE,
	.read = kr3dm_read,
	.write = kr3dm_write,
	.open = kr3dm_open,
	.release = kr3dm_close,
	.ioctl = kr3dm_ioctl,
};

static struct miscdevice kr3dm_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kr3dm_accel",
	.fops = &kr3dm_fops,
};

static int kr3dm_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef STM_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "kr3dm", I2C_NAME_SIZE);

	return 0;
}

static void kr3dm_acc_enable(void)
{
	printk("starting poll timer, delay %lldns\n", ktime_to_ns(g_kr3dm->acc_poll_delay));
	hrtimer_start(&g_kr3dm->timer, g_kr3dm->acc_poll_delay, HRTIMER_MODE_REL);
}

static void kr3dm_acc_disable(void)
{
	printk("cancelling poll timer\n");
	hrtimer_cancel(&g_kr3dm->timer);
	cancel_work_sync(&g_kr3dm->work_acc);
}

/////////////////////////////////////////////////////////////////////////////////////

static ssize_t poll_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lld\n", ktime_to_ns(g_kr3dm->acc_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	/* HACK: workaround to prevent apps such as Google Maps 6.x from polling sensors excessively.
	         Please let me know if there's a better way to do this! -psyke83 */
	if (new_delay < 10000000 ) {
		new_delay = 10000000;
	}

	printk("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(g_kr3dm->acc_poll_delay));
	mutex_lock(&g_kr3dm->power_lock);
	if (new_delay != ktime_to_ns(g_kr3dm->acc_poll_delay)) {
		kr3dm_acc_disable();
		g_kr3dm->acc_poll_delay = ns_to_ktime(new_delay);
		if (g_kr3dm->state & ACC_ENABLED) {
			kr3dm_acc_enable();
		}
	}
	mutex_unlock(&g_kr3dm->power_lock);

	return size;
}

static ssize_t acc_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (g_kr3dm->state & ACC_ENABLED) ? 1 : 0);
}


static ssize_t acc_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&g_kr3dm->power_lock);
	printk("new_value = %d, old state = %d\n", new_value, (g_kr3dm->state & ACC_ENABLED) ? 1 : 0);
	if (new_value && !(g_kr3dm->state & ACC_ENABLED)) {
		g_kr3dm->state |= ACC_ENABLED;
		kr3dm_acc_enable();
	} else if (!new_value && (g_kr3dm->state & ACC_ENABLED)) {
		kr3dm_acc_disable();
		g_kr3dm->state = 0;
	}
	mutex_unlock(&g_kr3dm->power_lock);
	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_acc_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       acc_enable_show, acc_enable_store);

static struct attribute *acc_sysfs_attrs[] = {
	&dev_attr_acc_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group acc_attribute_group = {
	.attrs = acc_sysfs_attrs,
};

#ifdef WORK_DEBUG
#include "../../../arch/arm/mach-msm/smd_private.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_iomap-7xxx.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>

static volatile unsigned int panel_cpu = UINT_MAX;

void check_system_time(void) {
	unsigned long long sec_sys;
	unsigned long nanosec_sys, result_sys;

	/* Follow the token with the time */
	sec_sys = cpu_clock(panel_cpu);
	nanosec_sys = do_div(sec_sys, 1000000000);
	result_sys = ((unsigned long) sec_sys) * 1000 + (nanosec_sys / 1000000); /* means XX.XXX sec */
	//printk(KERN_ERR "%s - system time : %lu\n", __func__, result_sys);

	if(result_sys > 30200) {
		msm_proc_comm_modem_normal_reset();
		//msm_proc_comm_reset_modem_now();
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////////

static void kr3dm_work_func_acc(struct work_struct *work)
{
	int err;
	kr3dm_acc_t acc;
	
	err = kr3dm_read_accel_xyz(&acc);

#ifdef WORK_DEBUG
	check_system_time();
	//printk("kr3dm_work_func_acc : %d,  %d,  %d\n", acc.x, acc.y, acc.z );
#endif

	input_report_rel(g_kr3dm->acc_input_dev, REL_X, acc.x);
	input_report_rel(g_kr3dm->acc_input_dev, REL_Y, acc.y);
	input_report_rel(g_kr3dm->acc_input_dev, REL_Z, acc.z);
	input_sync(g_kr3dm->acc_input_dev);
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart kr3dm_timer_func(struct hrtimer *timer)
{
	queue_work(g_kr3dm->wq, &g_kr3dm->work_acc);
	hrtimer_forward_now(&g_kr3dm->timer, g_kr3dm->acc_poll_delay);
	return HRTIMER_RESTART;
}

static void I2c_udelaySet(void)
{
	struct i2c_algo_bit_data *adap = kr3dm_client->adapter->algo_data;
	adap->udelay = 2;
}

static int kr3dm_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct input_dev *input_dev;
	struct kr3dm_data *data;
#ifdef STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "[KR3DM] i2c_check_functionality error\n");
		err = -ENODEV;
		goto exit;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct kr3dm_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct kr3dm_data));
	i2c_set_clientdata(client, data);
	kr3dm_client = client;

	if (i2c_smbus_read_byte(client) < 0)
	{
		printk(KERN_ERR "[KR3DM] i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	}
	else
	{
		printk("KR3DM Device detected!\n");
	}

	printk("[%s] slave addr = %x\n", __func__, client->addr);
	
	/* read chip id */
	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client,WHO_AM_I);
	//  if((tempvalue&0x00FF) == 0x0032) 
	if((tempvalue&0x00FF) == 0x0012)  // changed for KR3DM.
	{
		printk("KR3DM I2C driver registered!\n");
		kr3dm_client = client;
	}
	else
	{
		printk(KERN_ERR "KR3DM I2C driver not registered 0x%x!\n", tempvalue);
		kr3dm_client = NULL;
		goto exit_kfree;
	}

//	kr3dm_set_enable( PM_NORMAL );
//	i2c_set_clientdata(kr3dm_client, data);
  	err = misc_register(&kr3dm_device);
	if (err) {
		printk(KERN_ERR "KR3DM accel device register failed\n");
		goto exit_kfree;
	}
	printk(KERN_INFO "KR3DM accel device create ok\n");

	g_kr3dm = &data->kr3dm;
//////////////////////////////////////////////////////////////////////////////
	mutex_init(&g_kr3dm->power_lock);

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&g_kr3dm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_kr3dm->acc_poll_delay = ns_to_ktime(50 * NSEC_PER_MSEC);
	g_kr3dm->timer.function = kr3dm_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	g_kr3dm->wq = create_singlethread_workqueue("kr3dm_wq");
	if (!g_kr3dm->wq) {
		err = -ENOMEM;
		printk("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&g_kr3dm->work_acc, kr3dm_work_func_acc);

///////////////////////////////////////////////////////////////////////////////////
	/* allocate accelerometer_sensor input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, g_kr3dm);
	input_dev->name = "accelerometer_sensor";

	set_bit(EV_REL, input_dev->evbit);	
	/* 32768 == 1g, range -4g ~ +4g */
	/* acceleration x-axis */
	input_set_capability(input_dev, EV_REL, REL_X);
	input_set_abs_params(input_dev, REL_X, -256, 256, 0, 0);
	/* acceleration y-axis */
	input_set_capability(input_dev, EV_REL, REL_Y);
	input_set_abs_params(input_dev, REL_Y, -256, 256, 0, 0);
	/* acceleration z-axis */
	input_set_capability(input_dev, EV_REL, REL_Z);
	input_set_abs_params(input_dev, REL_Z, -256, 256, 0, 0);

	printk("registering accelerometer_sensor input device\n");
	err = input_register_device(input_dev);
	if (err < 0) {
		printk("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	g_kr3dm->acc_input_dev = input_dev;

	err = sysfs_create_group(&input_dev->dev.kobj,&acc_attribute_group);
	if (err) {
		printk("Creating acc attribute group failed");
		goto error_device;
	}

	if(!USE_I2C_SMBUS){
		I2c_udelaySet();
	}
//////////////////////////////////////////////////////////////////////////////

	device_init();

#ifdef KR3DM_PROC_FS
	create_proc_read_entry(DRIVER_PROC_ENTRY, 0, 0, kr3dm_proc_read, NULL);
#endif	//KR3DM_PROC_FS
#ifdef CONFIG_HAS_EARLYSUSPEND
	g_kr3dm->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_kr3dm->early_suspend.suspend = kr3dm_early_suspend;
	g_kr3dm->early_suspend.resume  = kr3dm_late_resume;
	register_early_suspend(&g_kr3dm->early_suspend);
#endif

	return 0;

error_device:
	sysfs_remove_group(&client->dev.kobj, &acc_attribute_group);
err_input_register_device_light:
	input_unregister_device(g_kr3dm->acc_input_dev);
err_input_allocate_device_light:	
	destroy_workqueue(g_kr3dm->wq);
exit_kfree:
	kfree(data);
err_create_workqueue:
	mutex_destroy(&data->power_lock);	
exit:
	return err;
}


static int kr3dm_remove(struct i2c_client *client)
{
	struct kr3dm_data *data = i2c_get_clientdata(client);

	if (g_kr3dm->state & ACC_ENABLED)
	{
		g_kr3dm->state = 0;
		kr3dm_acc_disable();
	}
	sysfs_remove_group(&g_kr3dm->acc_input_dev->dev.kobj, &acc_attribute_group);
	input_unregister_device(g_kr3dm->acc_input_dev);

	destroy_workqueue(g_kr3dm->wq);
	mutex_destroy(&g_kr3dm->power_lock);
	

#ifdef STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
	misc_deregister(&kr3dm_device);

	kfree(data);
	kr3dm_client = NULL;
	g_kr3dm = NULL;

	return 0;
}


#ifdef CONFIG_PM
static int kr3dm_suspend(struct i2c_client *client, pm_message_t mesg)
{

	int res = 0;
	unsigned char data;

	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
	 printk(KERN_INFO "[kr3dm_suspend] res = %x \n",res);    
		data = res & 0x001f;
	printk(KERN_INFO "[kr3dm_suspend] data = %x \n",data);
	
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);

	if (g_kr3dm->state & ACC_ENABLED) 
		kr3dm_acc_disable();
	
#ifdef STM_DEBUG
	printk(KERN_INFO "[%s] KR3DM !!suspend mode!!\n",__FUNCTION__);
#endif

	return 0;
}

static int kr3dm_resume(struct i2c_client *client)
{

	int res = 0;
	unsigned char data;

	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
      printk(KERN_INFO "[kr3dm_resume] res = %x \n",res);    
		data = (res & 0x003f)|(0x1<<5);
	printk(KERN_INFO "[kr3dm_resume] data = %x \n",data);
	
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);

	if (g_kr3dm->state & ACC_ENABLED)
		kr3dm_acc_enable();
	
#ifdef STM_DEBUG
	printk(KERN_INFO "[%s] KR3DM !!resume mode!!\n",__FUNCTION__);
#endif

	return 0;
}
#else
#define kr3dm_suspend NULL
#define kr3dm_resume NULL
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void kr3dm_early_suspend(struct early_suspend *h)
{
	int res = 0;
	unsigned char data;

	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
	 printk(KERN_INFO "[kr3dm_suspend] res = %x \n",res);    
		data = res & 0x001f;
	printk(KERN_INFO "[kr3dm_suspend] data = %x \n",data);
	
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);

	if (g_kr3dm->state & ACC_ENABLED) 
		kr3dm_acc_disable();
	
#ifdef STM_DEBUG
	printk(KERN_INFO "[%s] KR3DM !!suspend mode!!\n",__FUNCTION__);
#endif

}

static void kr3dm_late_resume(struct early_suspend *h)
{
	int res = 0;
	unsigned char data;

	res = i2c_smbus_read_word_data(kr3dm_client, CTRL_REG1);
      printk(KERN_INFO "[kr3dm_resume] res = %x \n",res);    
		data = (res & 0x003f)|(0x1<<5);
	printk(KERN_INFO "[kr3dm_resume] data = %x \n",data);
	
	res = kr3dm_i2c_write(CTRL_REG1, &data, 1);

	if (g_kr3dm->state & ACC_ENABLED)
		kr3dm_acc_enable();
	
#ifdef STM_DEBUG
	printk(KERN_INFO "[%s] KR3DM !!resume mode!!\n",__FUNCTION__);
#endif
}

#endif


static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(kr3dm_accel);

static const struct i2c_device_id kr3dm_id[] = {
	{ "kr3dm_accel", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, kr3dm_id);

static struct i2c_driver kr3dm_accel_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "kr3dm_accel",
	},
//	.class		= I2C_CLASS_HWMON,
	.id_table	= kr3dm_id,
//	.address_data	= &addr_data,//kerner version 3.2
	.probe		= kr3dm_probe,
	.remove		= kr3dm_remove,
	.detect		= kr3dm_detect,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= kr3dm_suspend,
	.resume		= kr3dm_resume,
#endif
};

static int __init kr3dm_init(void)
{
	//struct device *dev_t;

#ifdef STM_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	kr3d_dev_class = class_create(THIS_MODULE, "accelerometer");

	//sec_class = class_create(THIS_MODULE, "sec");
	
	if (IS_ERR(kr3d_dev_class)) 
		return PTR_ERR( kr3d_dev_class );

	kr3dm_dev_t = device_create( kr3d_dev_class, NULL, MKDEV(ACC_DEV_MAJOR, 0), "%s", "accelerometer");

	if (IS_ERR(kr3dm_dev_t)) 
	{
		return PTR_ERR(kr3dm_dev_t);
	}

	if (device_create_file(kr3dm_dev_t, &dev_attr_acc_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_file.attr.name);

	return i2c_add_driver(&kr3dm_accel_driver);
}

static void __exit kr3dm_exit(void)
{
	i2c_del_driver(&kr3dm_accel_driver);
	printk(KERN_ERR "KR3DM_ACCEL exit\n");

	class_destroy( kr3d_dev_class );
}



module_init(kr3dm_init);
module_exit(kr3dm_exit);


MODULE_DESCRIPTION("kr3dm accelerometer driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

                     
