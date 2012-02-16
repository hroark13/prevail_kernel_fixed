/*
 * KR3DH/KR3DM accelerometer driver
 *
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/i2c/kr3dh.h>

#define KR3DH_VERSION "1.2.0"

#define KR3DH_NAME "kr3dm_accel"

/* for debugging */
#define DEBUG_LOG 0
#define DEBUG 0
#define DEBUG_THRESHOLD 0
#define TRACE_FUNC() pr_debug(KR3DH_NAME ": <trace> %s()\n", __FUNCTION__)

/*
 * Default parameters
 */
#define KR3DH_DEFAULT_DELAY         200 //dha23 110309 <-- 100
#define KR3DH_MAX_DELAY             2000

/*
 * Registers
 */
#define KR3DH_WHO_AM_I              0x0f
#define KR3DH_CTRL_REG1             0x20
#define KR3DH_CTRL_REG2             0x21
#define KR3DH_CTRL_REG3             0x22
#define KR3DH_CTRL_REG4             0x23
#define KR3DH_CTRL_REG5             0x24
#define KR3DH_HP_FILTER_RESET       0x25
#define KR3DH_REFERENCE             0x26
#define KR3DH_STATUS_REG            0x27
#define KR3DH_OUT_X_L               0x28
#define KR3DH_OUT_X                 0x29
#define KR3DH_OUT_Y_L               0x2a
#define KR3DH_OUT_Y                 0x2b
#define KR3DH_OUT_Z_L               0x2c
#define KR3DH_OUT_Z                 0x2d

#define KR3DM_ID                    0x12
#define KR3DH_ID                    0x32

#define KR3DH_CR1_PM_NORMAL         0x20
#define KR3DH_CR1_DR_50             0x00
#define KR3DH_CR1_DR_100            0x08
#define KR3DH_CR1_DR_400            0x10
#define KR3DH_CR1_DR_1000           0x18
#define KR3DH_CR1_Z_ENABLE          0x04
#define KR3DH_CR1_Y_ENABLE          0x02
#define KR3DH_CR1_X_ENABLE          0x01
#define KR3DH_CR1_XYZ_ENABLE        0x07
#define KR3DH_CR1_DISABLE           0x00

#define KR3DH_CR4_BDU_ON            0x80
#define KR3DH_CR4_BDU_OFF           0x00
#define KR3DH_CR4_BLE_BE            0x40
#define KR3DH_CR4_BLE_LE            0x00
#define KR3DH_CR4_FS_2G             0x00
#define KR3DH_CR4_FS_4G             0x10
#define KR3DH_CR4_FS_8G             0x30

/*
 * Acceleration measurement
 */
#define KR3DM_RESOLUTION            64
#define KR3DH_RESOLUTION            16384

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)

struct acceleration {
	int x;
	int y;
	int z;
};

/*
 * Output data rate
 */
/* DR bits of CTRL_REG1 in low power mode */
#define DR_LP_LPF_37HZ                  (0<<3)
#define DR_LP_LPF_74HZ                  (1<<3)
#define DR_LP_LPF_292HZ                 (2<<3)
#define DR_LP_LPF_780HZ                 (3<<3)

struct kr3dh_odr {
	unsigned long delay;            /* min delay (msec) in the range of ODR */
	u8 odr;                         /* register value of ODR */
};

static const struct kr3dh_odr kr3dh_odr_table[] = {
	{1,    0x38},                   /* ODR = 1000 (Hz) */
	{3,    0x30},                   /*        400      */
	{10,   0x28},                   /*        100      */
	{20,   0x20},                   /*         50      */
	{100,  0xc0 | DR_LP_LPF_780HZ}, /*         10      */
	{200,  0xa0 | DR_LP_LPF_780HZ}, /*          5      */
	{500,  0x80 | DR_LP_LPF_780HZ}, /*          2      */
	{1000, 0x60 | DR_LP_LPF_780HZ}, /*          1      */
	{2000, 0x40 | DR_LP_LPF_780HZ}, /*          0.5    */
};

static const struct kr3dh_odr kr3dm_odr_table[] = {
	{3,    0x30},                   /* ODR =  400 (Hz) */
	{10,   0x28},                   /*        100      */
	{20,   0x20},                   /*         50      */
	{100,  0xc0 | DR_LP_LPF_74HZ},  /*         10      */
	{200,  0xa0 | DR_LP_LPF_74HZ},  /*          5      */
	{500,  0x80 | DR_LP_LPF_74HZ},  /*          2      */
	{1000, 0x60 | DR_LP_LPF_74HZ},  /*          1      */
	{2000, 0x40 | DR_LP_LPF_74HZ},  /*          0.5    */
};

/*
 * Transformation matrix for chip mounting position
 */
static const int kr3dh_position_map[][3][3] = {
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}}, /* top/upper-left */
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}}, /* top/upper-right */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}}, /* top/lower-right */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}}, /* top/lower-left */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}}, /* bottom/upper-left */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}}, /* bottom/upper-right */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}}, /* bottom/lower-right */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}}, /* bottom/lower-right */};

/*
 * driver private data
 */
struct kr3dh_data {
	int id;                         /* value of WHO_AM_I */
	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;              /* attribute value */
	atomic_t threshold;             /* attribute value */
	u8 odr;                         /* output data rate register value */
	struct acceleration last;       /* last measured data */
	struct mutex enable_mutex;
	struct mutex data_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct kr3d_platform_data *pdata;
#if DEBUG
	int suspend;
#endif
};

#define is_kr3dm(p) ((p)->id == KR3DM_ID)

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/* register access functions */
//#define kr3dh_read(c,a,d,l)  kr3dh_i2c_transfer((c),(a),(d),(l),I2C_M_RD)
//#define kr3dh_write(c,a,d,l) kr3dh_i2c_transfer((c),(a),(d),(l),0)
static int kr3dh_i2c_transfer(struct i2c_client *client,
				  u8 addr, u8 *data, int len, int flag);


static char kr3dh_write(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;
	int i; 
	if(DEBUG_LOG) printk(KERN_INFO"%s\n", __FUNCTION__);
	if( client == NULL )  /*  No global client pointer? */
		return -1;
	for(i=0;i<len;i++)
	{ 
		dummy = i2c_smbus_write_byte_data(client, reg_addr++, data[i]);
		if(dummy)
		{
			printk(KERN_INFO"i2c write error\n");
			return dummy; 
		}
	}
	return 0;
}

/*  i2c read routine for kr3dm  */
static int kr3dh_read(struct i2c_client *client, unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	int dummy=0;
	int i=0;
	//if(DEBUG_LOG) printk(KERN_INFO"%s\n", __FUNCTION__);
	if( client == NULL )  /*  No global client pointer? */
		return -1;
	while(i<len)
	{        
		dummy = i2c_smbus_read_word_data(client, reg_addr++);
		if (dummy>=0)
		{         
			data[i] = dummy & 0x00ff;
			i++;
		} 
		else
		{
			printk(KERN_INFO" i2c read error(%d) \n ",dummy); 
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}


/*
 * Device dependant operations
 */
static int kr3dh_power_up(struct kr3dh_data *kr3dh)
{
	struct i2c_client *client = kr3dh->client;
	u8 data;
	
	//data = kr3dh->odr | KR3DH_CR1_XYZ_ENABLE;
	data = KR3DH_CR1_XYZ_ENABLE | KR3DH_CR1_PM_NORMAL;
	kr3dh_write(client, KR3DH_CTRL_REG1, &data, 1);

	return 0;
}

static int kr3dh_power_down(struct kr3dh_data *kr3dh)
{
	struct i2c_client *client = kr3dh->client;
	u8 data;

	//data = KR3DH_CR1_XYZ_ENABLE;
	data = KR3DH_CR1_DISABLE;
	kr3dh_write(client, KR3DH_CTRL_REG1, &data, 1);
		
	return 0;
}

static int kr3dh_hw_init(struct kr3dh_data *kr3dh)
{
	struct i2c_client *client = kr3dh->client;

	unsigned char buf[5];
	
	buf[0]=0x27;
	buf[1]=0x00;
	buf[2]=0x00;
	buf[3]=0x00;
	buf[4]=0x00;

    kr3dh_write(client, KR3DH_CTRL_REG1, &buf[0], 5);	
/*	u8 data;

	data = KR3DH_CR1_XYZ_ENABLE;
	kr3dh_write(client, KR3DH_CTRL_REG1, &data, 1);

	data = 0x00;
	kr3dh_write(client, KR3DH_CTRL_REG2, &data, 1);

	data = 0x00;
	kr3dh_write(client, KR3DH_CTRL_REG3, &data, 1);

	data = KR3DH_CR4_FS_2G;
	if (kr3dh->id == KR3DH_ID)
		data |= KR3DH_CR4_BDU_ON | KR3DH_CR4_BLE_LE;
	kr3dh_write(client, KR3DH_CTRL_REG4, &data, 1);

	data = 0x00;
	kr3dh_write(client, KR3DH_CTRL_REG5, &data, 1);
*/
	return 0;
}

static int kr3dh_get_enable(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	return atomic_read(&kr3dh->enable);
}

static void kr3dh_set_enable(struct device *dev, unsigned long enable)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	int delay = atomic_read(&kr3dh->delay);

	mutex_lock(&kr3dh->enable_mutex);

	if (enable) {                   /* enable if state will be changed */
		if (!atomic_cmpxchg(&kr3dh->enable, 0, 1)) {
			kr3dh_power_up(kr3dh);
			schedule_delayed_work(&kr3dh->work,
					      delay_to_jiffies(delay) + 1);
		}
	} else {                        /* disable if state will be changed */
		if (atomic_cmpxchg(&kr3dh->enable, 1, 0)) {
			cancel_delayed_work_sync(&kr3dh->work);
			kr3dh_power_down(kr3dh);
		}
	}
	atomic_set(&kr3dh->enable, enable);

	mutex_unlock(&kr3dh->enable_mutex);
}

static int kr3dh_get_delay(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	return atomic_read(&kr3dh->delay);
}

static void kr3dh_set_delay(struct device *dev, unsigned long delay)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	struct i2c_client *client = kr3dh->client;
	const struct kr3dh_odr *odr_table = is_kr3dm(kr3dh) ?
		kr3dm_odr_table : kr3dh_odr_table;
	const int size = is_kr3dm(kr3dh) ?
		ARRAY_SIZE(kr3dm_odr_table) : ARRAY_SIZE(kr3dh_odr_table);
	int i;
	u8 data;

	/* determine optimum ODR */
	for (i = 1; (i < size) && (actual_delay(delay) >= odr_table[i].delay); i++)
		;
	kr3dh->odr = odr_table[i-1].odr;
	atomic_set(&kr3dh->delay, delay);

	mutex_lock(&kr3dh->enable_mutex);

	/* update CTRL register and reschedule work_queue if enable=1 */
	if (kr3dh_get_enable(dev)) {
		cancel_delayed_work_sync(&kr3dh->work);
		//data = kr3dh->odr | KR3DH_CR1_XYZ_ENABLE;
		data = KR3DH_CR1_XYZ_ENABLE | KR3DH_CR1_PM_NORMAL;
		kr3dh_write(client, KR3DH_CTRL_REG1, &data, 1);
		schedule_delayed_work(&kr3dh->work,
				      delay_to_jiffies(delay) + 1);
	}

	mutex_unlock(&kr3dh->enable_mutex);
}

static int kr3dh_get_position(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	return atomic_read(&kr3dh->position);
}

static void kr3dh_set_position(struct device *dev, unsigned long position)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	atomic_set(&kr3dh->position, position);
}

static int kr3dh_get_threshold(struct device *dev)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	return atomic_read(&kr3dh->threshold);
}

static void kr3dh_set_threshold(struct device *dev, int threshold)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	atomic_set(&kr3dh->threshold, threshold);
}

static int kr3dh_data_filter(struct device *dev, struct acceleration *accel, int data[])
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	int threshold = atomic_read(&kr3dh->threshold);
#if DEBUG_THRESHOLD
	int update;
#endif
#if DEBUG_THRESHOLD
	update = 0;
#endif
	mutex_lock(&kr3dh->data_mutex);
	if ((abs(kr3dh->last.x - data[0]) > threshold) ||
	    (abs(kr3dh->last.y - data[1]) > threshold) ||
            (abs(kr3dh->last.z - data[2]) > threshold)) {
		accel->x = data[0];
		accel->y = data[1];
		accel->z = data[2];
#if DEBUG_THRESHOLD
		update = 1;
#endif
	} else {
		*accel = kr3dh->last;
	}

#if DEBUG_THRESHOLD
	if (update == 1) {
		dev_info(dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d) ****\n", threshold,
                         kr3dh->last.x - data[0], kr3dh->last.y - data[1], kr3dh->last.z - data[2], accel->x, accel->y, accel->z);
	} else {
		dev_info(dev, "threshold=%d x(%d) y(%d) z(%d) accel(%d,%d,%d)\n", threshold,
                         kr3dh->last.x - data[0], kr3dh->last.y - data[1], kr3dh->last.z - data[2], accel->x, accel->y, accel->z);
	}
#endif
	mutex_unlock(&kr3dh->data_mutex);

	return 0;
}

static int kr3dh_measure(struct kr3dh_data *kr3dh,
			     struct acceleration *accel)
{
	struct i2c_client *client = kr3dh->client;
	u8 buf[6];
	int raw[3], data[3];
	int pos = atomic_read(&kr3dh->position);
	int i, j;

	/* read acceleration data */
	if (kr3dh_read(client, KR3DH_OUT_X_L, buf, 6) < 0) {
		dev_err(&client->dev,
			"I2C block read error: addr=0x%02x, len=%d\n",
			KR3DH_OUT_X_L, 6);
		for (i = 0; i < 3; i++) {
			raw[i] = 0;
		}
	} else {
		for (i = 0; i < 3; i++) {
			raw[i] = *(s16 *)&buf[i*2];
		}
	}

	/* for X, Y, Z axis */
	for (i = 0; i < 3; i++) {
		/* coordinate transformation */
		data[i] = 0;
		for (j = 0; j < 3; j++) {
			data[i] += raw[j] * kr3dh_position_map[pos][i][j];
		}
		/* normalization */
		if (is_kr3dm(kr3dh)) {
			data[i] = (data[i] >> 8) * GRAVITY_EARTH / KR3DM_RESOLUTION;
		} else {
			long long g;
			g = (long long)data[i] * GRAVITY_EARTH / KR3DH_RESOLUTION;
			data[i] = g;
		}
	}

	dev_dbg(&client->dev, "raw(%6d,%6d,%6d) => norm(%8d,%8d,%8d)\n",
		raw[0], raw[1], raw[2], data[0], data[1], data[2]);

	kr3dh_data_filter(&client->dev, accel, data);

	return 0;
}

static void kr3dh_work_func(struct work_struct *work)
{
	struct kr3dh_data *kr3dh = container_of((struct delayed_work *)work,
							struct kr3dh_data, work);
	struct acceleration accel;
	unsigned long delay = delay_to_jiffies(atomic_read(&kr3dh->delay));
	
	if(DEBUG_LOG) printk("kr3dh_work_func start!!!\n");

	kr3dh_measure(kr3dh, &accel);
	
	#if defined(CONFIG_MACH_ROOKIE) // x = -data[1], y = data[0]
	input_report_abs(kr3dh->input, ABS_X, -accel.y);
	input_report_abs(kr3dh->input, ABS_Y, accel.x);
	input_report_abs(kr3dh->input, ABS_Z, accel.z);
	#else
	input_report_abs(kr3dh->input, ABS_X, accel.x);
	input_report_abs(kr3dh->input, ABS_Y, accel.y);
	input_report_abs(kr3dh->input, ABS_Z, accel.z);
	#endif
	input_sync(kr3dh->input);

	mutex_lock(&kr3dh->data_mutex);
	
	kr3dh->last = accel;
	
	mutex_unlock(&kr3dh->data_mutex);
	
	schedule_delayed_work(&kr3dh->work, delay);
	if(DEBUG_LOG) printk("kr3dh_work_func end!!!\n");
}

/*
 * Input device interface
 */
static int kr3dh_input_init(struct kr3dh_data *kr3dh)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev) {
		return -ENOMEM;
	}
	dev->name = "accelerometer";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, kr3dh);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	kr3dh->input = dev;

	return 0;
}

static void kr3dh_input_fini(struct kr3dh_data *kr3dh)
{
	struct input_dev *dev = kr3dh->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

/*
 * sysfs device attributes
 */
static ssize_t kr3dh_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kr3dh_get_enable(dev));
}

static ssize_t kr3dh_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	if ((enable == 0) || (enable == 1)) {
		kr3dh_set_enable(dev, enable);
	}

	return count;
}

static ssize_t kr3dh_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kr3dh_get_delay(dev));
}

static ssize_t kr3dh_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned long delay = simple_strtoul(buf, NULL, 10);

	if (delay > KR3DH_MAX_DELAY) {
		delay = KR3DH_MAX_DELAY;
	}

	kr3dh_set_delay(dev, delay);

	return count;
}

static ssize_t kr3dh_position_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kr3dh_get_position(dev));
}

static ssize_t kr3dh_position_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long position;

	position = simple_strtoul(buf, NULL,10);
	if ((position >= 0) && (position <= 7)) {
		kr3dh_set_position(dev, position);
	}

	return count;
}

static ssize_t kr3dh_threshold_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kr3dh_get_threshold(dev));
}

static ssize_t kr3dh_threshold_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned long threshold;

	threshold = simple_strtoul(buf, NULL,10);
	if (threshold >= 0 && threshold <= ABSMAX_2G) {
		kr3dh_set_threshold(dev, threshold);
	}

	return count;
}

static ssize_t kr3dh_wake_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

	return count;
}

static ssize_t kr3dh_data_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	struct acceleration accel;

	mutex_lock(&kr3dh->data_mutex);
	accel = kr3dh->last;
	mutex_unlock(&kr3dh->data_mutex);

	return sprintf(buf, "%d,%d,%d\n", accel.x, accel.y, accel.z);
}

#if DEBUG
static ssize_t kr3dh_debug_reg_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	struct i2c_client *client = kr3dh->client;
	u8 reg[5];

	kr3dh_read(client, KR3DH_CTRL_REG1, reg, 5);
	return sprintf(buf, "%02x %02x %02x %02x %02x\n",
		       reg[0], reg[1], reg[2], reg[3], reg[4]);
}

static int kr3dh_suspend(struct i2c_client *client, pm_message_t mesg);
static int kr3dh_resume(struct i2c_client *client);

static ssize_t kr3dh_debug_suspend_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);

	return sprintf(buf, "%d\n", kr3dh->suspend);
}

static ssize_t kr3dh_debug_suspend_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct kr3dh_data *kr3dh = input_get_drvdata(input);
	struct i2c_client *client = kr3dh->client;
	unsigned long suspend = simple_strtoul(buf, NULL, 10);

	if (suspend) {
		pm_message_t msg;
		kr3dh_suspend(client, msg);
	} else {
		kr3dh_resume(client);
	}

	return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		   kr3dh_enable_show, kr3dh_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		   kr3dh_delay_show, kr3dh_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR,
		   kr3dh_position_show, kr3dh_position_store);
static DEVICE_ATTR(threshold, S_IRUGO|S_IWUSR,
		   kr3dh_threshold_show, kr3dh_threshold_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		   NULL, kr3dh_wake_store);
static DEVICE_ATTR(data, S_IRUGO,
		   kr3dh_data_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_reg, S_IRUGO,
		   kr3dh_debug_reg_show, NULL);
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
		   kr3dh_debug_suspend_show, kr3dh_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *kr3dh_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_threshold.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group kr3dh_attribute_group = {
	.attrs = kr3dh_attributes
};

/*
 * I2C client
 */
static int kr3dh_i2c_transfer(struct i2c_client *client,
				  u8 addr, u8 *data, int len, int flag)
{
	struct i2c_msg msg[2];
	u8 reg;
	int err;

	/* set msb of register address when multi-byte access */
	reg = addr | (len>1 ? 0x80 : 0);
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = client->addr;
	msg[1].flags = (client->flags & I2C_M_TEN) | flag;
	msg[1].len = len;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&client->dev,
			"I2C %s error: slave_addr=%d, addr=0x%02x, err=%d\n",
		       (flag & I2C_M_RD)?"read":"write", client->addr, reg, err);
		return err;
	}

	return 0;
}

static int kr3dh_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct kr3dh_data *kr3dh = i2c_get_clientdata(client);
	s32 id;

	id = i2c_smbus_read_byte_data(client, KR3DH_WHO_AM_I);
	if ((id&0x00FF) != 0x0012) {
		return -ENODEV;
	}
	kr3dh->id = id;

	return 0;
}

static int kr3dh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct kr3dh_data *kr3dh;
	int err;
	
 	if(DEBUG_LOG) printk("kr3dm_accel_driver probe start!\n");

	/* setup private data */
	kr3dh = kzalloc(sizeof(struct kr3dh_data), GFP_KERNEL);
	if (!kr3dh) {
		err = -ENOMEM;
		goto error_0;
	}
	mutex_init(&kr3dh->enable_mutex);
	mutex_init(&kr3dh->data_mutex);

	/* setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto error_1;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
        err = -ENODEV;
        goto error_1;
    }

	i2c_set_clientdata(client, kr3dh);
	kr3dh->client = client;
	kr3dh->pdata = client->dev.platform_data;

//	if(kr3dh->pdata && kr3dh->pdata->vreg_en)
//		kr3dh->pdata->vreg_en(1);
	
	//* detect and init hardware */
	if ((err = kr3dh_detect(client, NULL) < 0)) {
		goto error_1;
	}
	dev_info(&client->dev, "device id=%02x: %s found\n",
		 kr3dh->id, id->name);
	kr3dh_hw_init(kr3dh);
	kr3dh_set_delay(&client->dev, KR3DH_DEFAULT_DELAY);
	kr3dh_set_position(&client->dev, CONFIG_INPUT_KR3DH_POSITION);

	/* setup driver interfaces */
	INIT_DELAYED_WORK(&kr3dh->work, kr3dh_work_func);

	err = kr3dh_input_init(kr3dh);
	if (err < 0) {
		goto error_1;
	}

	err = sysfs_create_group(&kr3dh->input->dev.kobj,
				 &kr3dh_attribute_group);
	if (err < 0) {
		goto error_2;
	}

	if(DEBUG_LOG) printk("kr3dm_accel_driver probe complete!!\n");

	return 0;

error_2:
	kr3dh_input_fini(kr3dh);
error_1:
	kfree(kr3dh);
error_0:
	return err;
}

static int kr3dh_remove(struct i2c_client *client)
{
	struct kr3dh_data *kr3dh = i2c_get_clientdata(client);

	kr3dh_set_enable(&client->dev, 0);

	sysfs_remove_group(&kr3dh->input->dev.kobj, &kr3dh_attribute_group);
	kr3dh_input_fini(kr3dh);
	kfree(kr3dh);

	return 0;
}

static int kr3dh_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kr3dh_data *kr3dh = i2c_get_clientdata(client);
	if(DEBUG_LOG) printk("kr3dh_suspend\n");
	TRACE_FUNC();

	mutex_lock(&kr3dh->enable_mutex);

	if (kr3dh_get_enable(&client->dev)) {
		cancel_delayed_work_sync(&kr3dh->work);
		kr3dh_power_down(kr3dh);
	}

	//if(kr3dh->pdata && kr3dh->pdata->vreg_en)
	//	kr3dh->pdata->vreg_en(0);

#if DEBUG
	kr3dh->suspend = 1;
#endif

	mutex_unlock(&kr3dh->enable_mutex);

	return 0;
}

static int kr3dh_resume(struct i2c_client *client)
{
	struct kr3dh_data *kr3dh = i2c_get_clientdata(client);
	int delay = atomic_read(&kr3dh->delay);

	if(DEBUG_LOG) printk("kr3dh_resume\n");

	TRACE_FUNC();

	//if(kr3dh->pdata && kr3dh->pdata->vreg_en)
	//	kr3dh->pdata->vreg_en(1);

	kr3dh_hw_init(kr3dh);

	mutex_lock(&kr3dh->enable_mutex);

	if (kr3dh_get_enable(&client->dev)) {
		kr3dh_power_up(kr3dh);
		schedule_delayed_work(&kr3dh->work,
				      delay_to_jiffies(delay) + 1);
	}
#if DEBUG
	kr3dh->suspend = 0;
#endif

	mutex_unlock(&kr3dh->enable_mutex);

	return 0;
}

static const struct i2c_device_id kr3dh_id[] = {
	{KR3DH_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, kr3dh_id);

static struct i2c_driver kr3dh_driver ={
	.driver = {
		.name = "kr3dh",
		.owner = THIS_MODULE,
	},
	.probe = kr3dh_probe,
	.remove = kr3dh_remove,
	.suspend = kr3dh_suspend,
	.resume = kr3dh_resume,
	.id_table = kr3dh_id,
};

/*
 * Module init and exit
 */
static int __init kr3dh_init(void)
{
	return i2c_add_driver(&kr3dh_driver);
}
module_init(kr3dh_init);

static void __exit kr3dh_exit(void)
{
	i2c_del_driver(&kr3dh_driver);
}
module_exit(kr3dh_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_DESCRIPTION("KR3DH/KR3DM accelerometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(KR3DH_VERSION);
