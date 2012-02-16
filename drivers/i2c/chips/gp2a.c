/* 
 *  Title : Optical Sensor(light sensor + proximity sensor) driver for GP2AP002A00F   
 *  Date  : 2009.02.27
 *  Name  : ms17.kim
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <linux/wakelock.h>
#include <mach/vreg.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <linux/earlysuspend.h>
#include <linux/i2c/gp2a.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"

#if !defined(CONFIG_MACH_RANT3)
#define LIGHT_SENSOR_ENABLED 1
#endif
/*********** for debug **********************************************************/
#if 0 // DEBUG 
#define gprintk(fmt, x... ) printk( "[GP2A]%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* global var */
/* Used for controlling the Autobrightness in lcdc_s6d04d1 */
extern void lcdc_set_backlight_autobrightness(int bl_level);
extern int app_bl_level;

static struct i2c_client *opt_i2c_client = NULL;

struct class *proxsensor_class;

struct device *switch_cmd_dev;

#if !defined(CONFIG_MACH_RANT3)
static struct vreg *vreg_als;
static struct vreg *vreg_led; // MBjgnoh 11.01.17 For sleep current
#endif

#ifdef LIGHT_SENSOR_ENABLED
struct class *lightsensor_class;
static state_type cur_state = LIGHT_INIT;
static int adc_value_buf[ADC_BUFFER_NUM] = {0, 0, 0, 0, 0, 0};
static int cur_adc_value = 0;
static bool lightsensor_test = 0;
struct workqueue_struct *gp2a_wq;
//static ktime_t timeA,timeB,timeSub;
#endif

static bool light_enable = OFF;
static bool proximity_enable = OFF;

//static int state_change_count = 0;
static short proximity_value = 0;

static struct wake_lock prx_wake_lock;
#if !defined(CONFIG_MACH_RANT3)
static struct wake_lock prx_irq_wake_lock;
#endif

static unsigned int gp2a_time = 0;
static int is_suspend = 0;

static ktime_t timeA,timeB;
#if USE_INTERRUPT
static ktime_t timeSub;
#endif

int gp2a_on(struct gp2a_data *gp2a, int type);
int gp2a_off(struct gp2a_data *gp2a, int type);

#ifdef LIGHT_SENSOR_ENABLED

#if defined(CONFIG_MACH_ESCAPE)
#define ADC_LEVEL1      315		//400   //600         // EG01 150  // 10 lux
#define ADC_LEVEL1_BUF  380		//550   //740         // EG01 500  // 15 lux
#define ADC_LEVEL2      1115	//1060  //1270        // EG01 1000 // 70 lux
#define ADC_LEVEL2_BUF  1160	//1320  //1530        // EG01 1230 // 150 lux
#define ADC_LEVEL3      1830 	//1960  //2185        // EG01 1800 // 1000 lux   // TODO:
#define ADC_LEVEL3_BUF  1880	//2100  //2325        // EG01 1910 // 1500 lux   // TODO:
#else
// iamaj EE25
#define ADC_LEVEL1 150					// 10 lux
#define ADC_LEVEL1_BUF 500			// 15 lux
#define ADC_LEVEL2 1000					// 70 lux
#define ADC_LEVEL2_BUF 1230		// 150 lux
#define ADC_LEVEL3 1800					// 1000 lux
#define ADC_LEVEL3_BUF 1910		// 1500 lux
#endif

static int buffering = 2;

static int StateToLux(state_type state);
static int light_open(struct inode *, struct file *);
static int light_release(struct inode *, struct file *);
int backlight_level = LIGHT_LEVEL2; //tmp, should be fixed(display driver etern value)

// SMEM_PROC_COMM_GET_ADC data 1
enum {
	SMEM_PROC_COMM_GET_ADC_BATTERY = 0x0,
	SMEM_PROC_COMM_GET_ADC_TEMP,
	SMEM_PROC_COMM_GET_ADC_VF,
	SMEM_PROC_COMM_GET_ADC_ALL,
	SMEM_PROC_COMM_GET_ADC_EAR_ADC,
	SMEM_PROC_COMM_GET_ADC_SSENS,
	SMEM_PROC_COMM_GET_ADC_CURRENT,
	SMEM_PROC_COMM_GET_ADC_MAX
};



//static bool light_init_check = false;
//static int light_init_check_count = 0;

static int light_init_period = 4;


static ssize_t light_read(struct file *filp, char *lux, size_t count, loff_t *f_pos);

static struct file_operations light_fops = {
	.owner  = THIS_MODULE,
	.open   = light_open,
	.release = light_release,
	.read = light_read,
};
                 
static struct miscdevice light_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "light",
    .fops   = &light_fops,
};

#endif

/*************************************************************************/
/*		GP2A I2C_API	  				         */
/*************************************************************************/
/*  i2c read routine for gp2a  */
#if USE_INTERRUPT
int opt_i2c_read(u8 reg, u8 *val, unsigned int len )
{

	int err;
	u8 buf[3];
	
	struct i2c_msg msg[2];


	buf[0] = reg; 

	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;
	
	msg[0].len = 2;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

	*val = buf[0] << 8 | buf[1];
	
    if (err >= 0) return 0;

    gprintk("i2c transfer error\n");
    return err;
}
#endif

/*  i2c write routine for gp2a */
int opt_i2c_write( u8 reg, u8 *val )
{
    int err = 0;
    struct i2c_msg msg[1];
    unsigned char data[2];
    int retry = 10;

    if( (opt_i2c_client == NULL) || (!opt_i2c_client->adapter) ){
        return -ENODEV;
    }

    while(retry--)
    {
        data[0] = reg;
        data[1] = *val;

        msg->addr = opt_i2c_client->addr;
        msg->flags = I2C_M_WR;
        msg->len = 2;
        msg->buf = data;

        err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

        if (err >= 0) return 0;
    }
    printk("%s(%d) i2c transfer error : %d, 0x%x\n", __func__, __LINE__, err, *val);
    return err;
}


/*************************************************************************/
/*		GP2A sysfs	  				         */
/*************************************************************************/

short gp2a_get_proximity_value()
{
	return ((proximity_value==1)? 0:1);
}

EXPORT_SYMBOL(gp2a_get_proximity_value);

static ssize_t proxsensor_file_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	unsigned int detect = 0;

	gprintk("called %s \n",__func__);

//	detect = gpio_get_value(GPIO_PS_VOUT);	

	return sprintf(buf,"%u\n",proximity_value);
//	return detect;
}

static DEVICE_ATTR(proxsensor_file_state,0660, proxsensor_file_state_show, NULL);

#if defined(CONFIG_MACH_RANT3)
static ssize_t proxsensor_onoff_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("Proximity Sensor OnOff : %d\n",proximity_enable);

    return sprintf(buf,"%u\n",proximity_enable);
}

static ssize_t proxsensor_onoff_ctrl_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
    struct gp2a_data *gp2a = dev_get_drvdata(switch_cmd_dev);
    unsigned int onoff;
    int ret = 0;

    sscanf(buf,"%d\n",&onoff);
    printk("Set Proximity Sensor OnOff : %d\n",onoff);

    if(onoff == 1)
    {
        ret = gp2a_on(gp2a,PROXIMITY);
        if(ret == 0)
            proximity_enable =1;
    }
    else
    {
        ret = gp2a_off(gp2a,PROXIMITY);
        if(ret == 0)      
            proximity_enable =0;
    }

    return size;
}

static DEVICE_ATTR(proxsensor_onoff_ctrl,0660, proxsensor_onoff_ctrl_show, proxsensor_onoff_ctrl_store);
#endif

#ifdef LIGHT_SENSOR_ENABLED
short gp2a_get_proximity_enable(void)
{
	  return proximity_enable;

}
EXPORT_SYMBOL(gp2a_get_proximity_enable);


bool gp2a_get_lightsensor_status()
{
	  return light_enable;

}
EXPORT_SYMBOL(gp2a_get_lightsensor_status);
#endif



/*****************************************************************************************
 *  
 *  function    : gp2a_work_func_prox 
 *  description : This function is for proximity sensor (using B-1 Mode ). 
 *                when INT signal is occured , it gets value from VO register.   
 *
 *                 
 */
#if USE_INTERRUPT
static void gp2a_work_func_prox(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of(work, struct gp2a_data, work_prox);
	
	unsigned char value;
	unsigned char int_val=REGS_PROX;
	unsigned char vout=0;
#if defined(CONFIG_MACH_RANT3)
	int err;
	static int retry = 0;
#endif

	/* Read VO & INT Clear */
	
	printk("[PROXIMITY] %s \n",__func__);

	if(INT_CLEAR)
	{
//		int_val = REGS_PROX | (1 <<7);
	}
#if defined(CONFIG_MACH_RANT3)
	err = opt_i2c_read((u8)(int_val),&value,1);

	if(err < 0 && retry == 0)
	{
	    printk("[PROXIMITY] I2C read error, try again==============================!!! \n");
	    retry++;
	    schedule_delayed_work(&gp2a->work_prox, 10);
	    return;
	}
	else
	    retry = 0;
#else
	opt_i2c_read((u8)(int_val),&value,1);
#endif
	
	vout = value & 0x01;
	printk(KERN_INFO "[PROXIMITY] value = %d \n",vout);

	/* Report proximity information */
	proximity_value = vout;

	
	if(proximity_value ==0)
	{
		timeB = ktime_get();
		
		timeSub = ktime_sub(timeB,timeA);
		printk(KERN_INFO "[PROXIMITY] timeSub sec = %d, timeSub nsec = %d \n",timeSub.tv.sec,timeSub.tv.nsec);
		
		if (timeSub.tv.sec>=3 )
		{
		    wake_lock_timeout(&prx_wake_lock,HZ/2);
			printk(KERN_INFO "[PROXIMITY] wake_lock_timeout : HZ/2 \n");
		}
		else
			printk(KERN_INFO "[PROXIMITY] wake_lock is already set \n");

	}

	if(USE_INPUT_DEVICE)
	{
		input_report_abs(gp2a->input_dev,ABS_DISTANCE,(int)vout);
		input_sync(gp2a->input_dev);
		mdelay(1);
	}

	/* Write HYS Register */
	if(!vout)
	{
#if defined(CONFIG_MACH_RANT3)
		value = 0x2F;
#else
		value = 0x40;
#endif
	}
	else
	{
#if defined(CONFIG_MACH_RANT3)
		value = 0x0F;
#else
		value = 0x20;
#endif
	}
	
	opt_i2c_write((u8)(REGS_HYS),&value);

	/* Forcing vout terminal to go high */

	value = 0x18;
	opt_i2c_write((u8)(REGS_CON),&value);

	/* enable INT */

	enable_irq(gp2a->irq);

	/* enabling VOUT terminal in nomal operation */

	value = 0x00;

	opt_i2c_write((u8)(REGS_CON),&value);
	
	printk("[PROXIMITY] end %s \n",__func__);
}

static irqreturn_t gp2a_irq_handler(int irq, void *dev_id)
{
	struct gp2a_data *gp2a = dev_id;

	printk("[HSIL] %s\n", __func__);

	if(gp2a->irq !=-1)
	{
		disable_irq_nosync(gp2a->irq);
		gprintk("[PROXIMITY] disable_irq \n");

		if(is_suspend == 1)
		    gp2a_time = 1;
		else
		    gp2a_time = 0;
#if !defined(CONFIG_MACH_RANT3)
		wake_lock_timeout(&prx_irq_wake_lock,HZ/2);
#endif
		schedule_delayed_work(&gp2a->work_prox, gp2a_time);
	}
	
	printk("[PROXIMITY] IRQ_HANDLED : %d \n", gp2a_time);

	return IRQ_HANDLED;
}
#endif



/*****************************************************************************************
 *  
 *  function    : gp2a_work_func_light 
 *  description : This function is for light sensor. It has a few state.
 *                "STATE_0" means that circumstance in which you are now is clear as day.
 *                The more value is increased, the more circumstance is dark. 
 *                 
 */
extern int backlight_level;

//#define MDNIE_TUNINGMODE_FOR_BACKLIGHT
#ifdef LIGHT_SENSOR_ENABLED
#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
int pre_val = -1;
extern int old_gamma_value;
extern int level_outdoor;
extern u16 *pmDNIe_Gamma_set[];
extern void mDNIe_Mode_set_for_backlight(u16 *buf);

int value_buf[4] = {0};

int IsChangedADC(int val)
{
	int j = 0;
	int i = 0;
	int ret = 0;

	int adc_total = 0;

	int adc_index = 0;
	static int adc_index_count = 0;



	adc_index = (adc_index_count)%4;		
	adc_index_count++;

	if(pre_val == -1) //ADC buffer initialize 
	{
		for(j = 0; j<4; j++)
			value_buf[j] = val;

		pre_val = 0;
	}
    else
    {
    	value_buf[adc_index] = val;
	}

	ret = ((value_buf[0] == value_buf[1])&& \
			(value_buf[1] == value_buf[2])&& \
			(value_buf[2] == value_buf[3]))? 1 : 0;

	
	if(adc_index_count == 4)
		adc_index_count = 0;

	return ret;
}
#endif

// iamaj add [[
static int lightsensor_adc=0;	

short gp2a_get_light_sensor_value()
{
	return lightsensor_adc;
}

EXPORT_SYMBOL(gp2a_get_light_sensor_value);
// ]]

static void gp2a_work_func_light(struct work_struct *work)
{
	int adc=0;
	int lux = 0;	// iamaj
	state_type level_state = LIGHT_INIT;
	struct gp2a_data *gp2a = container_of(work, struct gp2a_data, work_light);
	ktime_t light_polling_time;
	
	static state_type prev_level_state = 0;
	static state_type changed_level_state = 0;
	static int check_count = 0;

#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
	int val = 0;
#endif

	/* read adc data from s5p110 */
	adc = lightsensor_get_adcvalue();
	gprintk("Optimized adc = %d \n",adc);
	gprintk("cur_state = %d\n",cur_state);
	gprintk("light_enable = %d\n",light_enable);
/* As MDNIE not used currently this is commented and used for Autobrightness and factory test */

//	lightsensor_adc = adc;	// iamaj 
#if 1	// iamaj EE25
	if(adc >= ADC_LEVEL3_BUF)
	{
			level_state = LIGHT_LEVEL4;
			buffering = 4;
	}
	else if(adc >= ADC_LEVEL3 && adc < ADC_LEVEL3_BUF)
	{
		if(buffering == 4)
		{	
			level_state = LIGHT_LEVEL4;
			buffering = 4;
		}
		else if((buffering == 1)||(buffering == 2)||(buffering == 3))
		{
			level_state = LIGHT_LEVEL3;
			buffering = 3;
		}
	}
	else if(adc >= ADC_LEVEL2_BUF && adc < ADC_LEVEL3)
	{
		level_state = LIGHT_LEVEL3;
		buffering = 3;
	}
	else if(adc >= ADC_LEVEL2 && adc < ADC_LEVEL2_BUF)
	{
		if((buffering == 3)||(buffering == 4))
		{	
			level_state = LIGHT_LEVEL3;
			buffering = 3;
		}
		else if((buffering == 1)||(buffering == 2))
		{
			level_state = LIGHT_LEVEL2;
			buffering = 2;
		}
	}
	else if(adc >= ADC_LEVEL1_BUF && adc < ADC_LEVEL2)
	{
		level_state = LIGHT_LEVEL2;
		buffering = 2;
	}
	else if(adc >= ADC_LEVEL1 && adc < ADC_LEVEL1_BUF)
	{
		if((buffering == 2)||(buffering == 3)||(buffering == 4))
		{	
			level_state = LIGHT_LEVEL2;
			buffering = 2;
		}
		else if(buffering == 1)
		{
			level_state = LIGHT_LEVEL1;
			buffering = 1;
		}
	}
	else if(adc < ADC_LEVEL1)
	{
		level_state = LIGHT_LEVEL1;
		buffering = 1;
	}

#else 

       if(adc >= ADC_LEVEL3 ){		// iamaj 1800 -> 1900 -> 1910
			level_state = LIGHT_LEVEL4;//255;//29;
		}
		else if(adc >=ADC_LEVEL2 && adc < ADC_LEVEL3){    // iamaj  1000->1200 ->1230,  1200->1900->1910
			level_state = LIGHT_LEVEL3;//145; //ed11  //MBjclee //14;
		}
		else if(adc >= ADC_LEVEL1 && adc < ADC_LEVEL2){     // iamaj 200->1->500 ,  1000->1200->1230
		level_state = LIGHT_LEVEL2;//80;//MBjclee //14
		}
		else if(adc < ADC_LEVEL1){		// iamaj 200 -> 1->500
			level_state = LIGHT_LEVEL1;//15;//5; 
		}
#endif

	cur_state = level_state;	

    lux = StateToLux(cur_state);

	lightsensor_adc = lux;	// iamaj 
	
#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
	if((old_gamma_value >= 25)&&(level_state == LIGHT_LEVEL8))
	{
		if(adc >= 2300 ){
			val = 2;
			level_outdoor = 2;
		}
		else if(adc >= 2100 && adc < 2300){
			val = 1;
			level_outdoor = 1;
		}
		else if(adc >= 1810 && adc < 2100){
			val = 0;
			level_outdoor = 0;
		}

		printk("mDNIe_Mode_set_for_backlight pre_val(%d) val(%d)\n",pre_val, val); //Temp

		if(IsChangedADC(val)){
			while(pre_val != val)
			{
				if (pre_val < val)
					pre_val++;
				else
					pre_val--;

				if((pre_val == 0) && ((level_outdoor == 1) || (level_outdoor == 2)))
					mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[3]);
				else
					mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[pre_val]);
				
				level_outdoor = pre_val;
				
				printk("mDNIe_Mode_set_for_backlight - pmDNIe_Gamma_set[%d]\n", pre_val);
			}
		}	
	}
#endif
		
	
	gprintk("cur_state = %d\n",cur_state); //Temp
}

int get_adc(void)
{
	int res = 0;
	int data1 = SMEM_PROC_COMM_GET_ADC_SSENS;
	int data2 = 0;

	res = msm_proc_comm(PCOM_OEM_SAMSUNG_GET_ADC, &data1, &data2);
	if(res < 0)
	{
		gprintk(" error data1[0x%x]  data2[0x%x]\n", data1, data2);
		return res;
	}

    gprintk(" data1[0x%x]  data2[0x%x]\n", data1, data2); //temp for test
	return data1;
}

int lightsensor_get_adcvalue(void)
{
	int i = 0;
	int j = 0;
	unsigned int adc_total = 0;
	static int adc_avr_value = 0;
	unsigned int adc_index = 0;
	static unsigned int adc_index_count = 0;
	unsigned int adc_max = 0;
	unsigned int adc_min = 0;
	int value =0;

	//get ADC
	//value = s3c_adc_get_adc_data(ADC_CHANNEL);

	/* Delay is added for bit late response , to make similar to vinsq model ,
	   if not necessary can be removed, sensor response will be fast */ 	
	/* If below delay is there , AT factory commands fail , so commented the delay */
//	mdelay(200);
	value = get_adc();
    //value = 1000; //temparary, adc value is controled in CP, 
    //check Modem/products/76xx/services/mproc/smem/smem_ext_pc.c, "get_batt_adc();" in europa_battery.c

	gprintk("adc = %d \n",value);
	cur_adc_value = value;
	
	adc_index = (adc_index_count++)%ADC_BUFFER_NUM;		

	if(cur_state == LIGHT_INIT) //ADC buffer initialize (light sensor off ---> light sensor on)
	{
		for(j = 0; j<ADC_BUFFER_NUM; j++)
			adc_value_buf[j] = value;
	}
    else
    {
    	adc_value_buf[adc_index] = value;
	}
	
	adc_max = adc_value_buf[0];
	adc_min = adc_value_buf[0];

	for(i = 0; i <ADC_BUFFER_NUM; i++)
	{
		adc_total += adc_value_buf[i];

		if(adc_max < adc_value_buf[i])
			adc_max = adc_value_buf[i];
					
		if(adc_min > adc_value_buf[i])
			adc_min = adc_value_buf[i];
	}
	adc_avr_value = (adc_total-(adc_max+adc_min))/(ADC_BUFFER_NUM-2);
	
	if(adc_index_count == ADC_BUFFER_NUM-1)
		adc_index_count = 0;

	return adc_avr_value;
}


/*****************************************************************************************
 *  
 *  function    : gp2a_timer_func 
 *  description : This function is for light sensor. 
 *                it operates every a few seconds. 
 *                 
 */

static enum hrtimer_restart gp2a_timer_func(struct hrtimer *timer)
{
	struct gp2a_data *gp2a = container_of(timer, struct gp2a_data, timer);

	/* If Auto brightness is only enabled, queue the work */
	if ( light_enable )	
	  	queue_work(gp2a_wq, &gp2a->work_light);
  
	//hrtimer_start(&gp2a->timer,ktime_set(light_init_period/2,0),HRTIMER_MODE_REL);
	hrtimer_start(&gp2a->timer,ktime_set(0,500000000),HRTIMER_MODE_REL);
  
	return HRTIMER_NORESTART;
}
#endif

#if defined(CONFIG_MACH_RANT3)
int gp2a_chip_init(void)
{
    gprintk("\n");
	gpio_tlmm_config(GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);


    return 0;
}
#else
int gp2a_chip_init(void)
{
	gprintk("\n");
	gpio_tlmm_config(GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
	gpio_tlmm_config(GPIO_CFG(GP2A_SENSOR_SDA , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GP2A_SENSOR_SCL , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
	vreg_als = vreg_get(0, "ldo9"); // MBjgnoh 11.01.17 For sleep current
	if(IS_ERR(vreg_als))
	    printk("error ldo 9 \n");

#if defined(CONFIG_MACH_ESCAPE)
    if (hw_version >= 2) /* R730_HW_REV02 ldo18 <--> ldo16 */
    {
    	vreg_led = vreg_get(0, "ldo16");
    	if(IS_ERR(vreg_led))
    	    printk("error ldo 16 \n");
    }
    else
#endif
    {
	vreg_led = vreg_get(0, "ldo18");
	if(IS_ERR(vreg_led))
	    printk("error ldo 18 \n");
    }
    return 0;
}
#endif

/*****************************************************************************************
 *  
 *  function    : gp2a_on 
 *  description : This function is power-on function for optical sensor.
 *
 *  int type    : Sensor type. Two values is available (PROXIMITY,LIGHT).
 *                it support power-on function separately.
 *                
 *                 
 */
int gp2a_on(struct gp2a_data *gp2a, int type)
{
	u8 value;
	int err = 0;
#ifdef LIGHT_SENSOR_ENABLED
	ktime_t light_polling_time;
#endif
	gprintk("gp2a_on(%d)\n",type);
	gprintk("gp2a power on voltage up \n");
	if(type == PROXIMITY)
	{
		gprintk("[PROXIMITY] go nomal mode : power on \n");
		// ASD : Select switch for analog sleep function ( 0:ineffective, 1:effective )
		// OCON[1:0] : Select switches for enabling/disabling VOUT terminal 
		//             ( 00:enable, 11:force to go High, 10:forcr to go Low )
		value = 0x18;	// 11:force to go High
		err = opt_i2c_write((u8)(REGS_CON),&value);
		if(err < 0) return err;

#if defined(CONFIG_MACH_RANT3)
	value = 0x2F;	// HYSD enable
#else
		value = 0x40;	// HYSD enable
#endif
		err = opt_i2c_write((u8)(REGS_HYS),&value);
		if(err < 0) return err;

		// VCON : VOUT terminal output method control ( 0:normal mode, 1:interrupt mode )
		// SSD : Software shutdown function ( 0:shutdown mode, 1:opteration mode )
		value = 0x03;	// VCON enable, SSD enable
		err = opt_i2c_write((u8)(REGS_OPMOD),&value);
		if(err < 0) return err;
#if USE_INTERRUPT
		gprintk("enable irq for proximity\n");
		enable_irq(gp2a ->irq);
#if !defined(CONFIG_MACH_RANT3)
		set_irq_wake(gp2a->irq, 1);
#endif
#endif
		// OCON[1:0] : Select switches for enabling/disabling VOUT terminal 
		//             ( 00:enable, 11:force to go High, 10:forcr to go Low )
		value = 0x00;	// 00:enable
		err = opt_i2c_write((u8)(REGS_CON),&value);
		
	}
#ifdef LIGHT_SENSOR_ENABLED
	if(type == LIGHT)
	{
		gprintk(KERN_INFO "[LIGHT_SENSOR] timer start for light sensor\n");
		//hrtimer_start(&gp2a->timer,ktime_set(light_init_period/2,0),HRTIMER_MODE_REL);
		hrtimer_start(&gp2a->timer,ktime_set(0,500000000),HRTIMER_MODE_REL);
		light_enable = ON;
	}
#endif	
       return err;
}

/*****************************************************************************************
 *  
 *  function    : gp2a_off 
 *  description : This function is power-off function for optical sensor.
 *
 *  int type    : Sensor type. Three values is available (PROXIMITY,LIGHT,ALL).
 *                it support power-on function separately.
 *                
 *                 
 */

int gp2a_off(struct gp2a_data *gp2a, int type)
{
	u8 value;
	int err = 0;

	gprintk("gp2a_off(%d)\n",type);
	if(type == PROXIMITY || type == ALL)
	{
#if USE_INTERRUPT	  
		gprintk("disable irq for proximity \n");
#if !defined(CONFIG_MACH_RANT3)
		set_irq_wake(gp2a->irq, 0);
#endif
		disable_irq_nosync(gp2a ->irq);
#endif
		// SSD : Software shutdown function ( 0:shutdown mode, 1:opteration mode )
		value = 0x02;	// VCON enable, SSD disable
		err = opt_i2c_write((u8)(REGS_OPMOD),&value);
	
	}
#ifdef LIGHT_SENSOR_ENABLED
	if(type ==LIGHT)
	{
		gprintk("[LIGHT_SENSOR] timer cancel for light sensor\n");
		hrtimer_cancel(&gp2a->timer);
		/* In case light sensor is off, set the default backlight level */
		//lcdc_set_backlight_autobrightness(app_bl_level);
		light_enable = OFF;
	}
#endif	
       return err;
}

/*************************************************************************/
/*		GP2A file operations  				         */
/*************************************************************************/
static int proximity_open(struct inode *ip, struct file *fp)
{
	return 0;

}

static int proximity_release(struct inode *ip, struct file *fp)
{
	return 0;

}

static long proximity_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	struct gp2a_data *gp2a = dev_get_drvdata(switch_cmd_dev);
	int ret=0;
	switch(cmd) {

		case SHARP_GP2AP_OPEN:
			{
				printk(KERN_INFO "[PROXIMITY] %s : case OPEN\n", __FUNCTION__);
				ret = gp2a_on(gp2a,PROXIMITY);
				if(ret == 0)
				proximity_enable =1;
				
			}
			break;

		case SHARP_GP2AP_CLOSE:
			{
				printk(KERN_INFO "[PROXIMITY] %s : case CLOSE\n", __FUNCTION__);
				ret = gp2a_off(gp2a,PROXIMITY);
				if(ret == 0)
				proximity_enable=0;
			}
			break;

		default:
			printk(KERN_INFO "[PROXIMITY] unknown ioctl %d\n", cmd);
			ret = -1;
			break;
	}
	return ret;
}

static struct file_operations proximity_fops = {
	.owner  = THIS_MODULE,
	.open   = proximity_open,
    	.release = proximity_release,
    	.unlocked_ioctl = proximity_ioctl,
};
                 
static struct miscdevice proximity_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "proximity",
    .fops   = &proximity_fops,
};



/*					 */
/*					 */
/*					 */
/* for  test mode 	start */
/*					 */
/*					 */
#if 0
static int AdcToLux_Testmode(int adc)
{
	unsigned int lux = 0;

    gprintk("[%s] adc:%d\n",__func__,adc);

	/*temporary code start*/
	
	if(adc >= 1800)
		lux = 100000;
	
	else if(adc >= 800 && adc < 1800){
		lux = 5000;
	}
	else if(adc < 800){
		lux = 10;
	}
	/*temporary code end*/
	
	return lux;
}
#endif

#ifdef LIGHT_SENSOR_ENABLED
static ssize_t lightsensor_file_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int adc = 0;
	int sum = 0;
	int i = 0;

    gprintk("called %s \n",__func__);

	if(!light_enable)
	{
		for(i = 0; i < 10; i++)
		{
			adc = lightsensor_get_adcvalue();
			msleep(20);
			sum += adc;
		}
		adc = sum/10;
		gprintk("called %s  - subdued alarm(adc : %d)\n",__func__,adc);
		return sprintf(buf,"%d\n",adc);
	}
	else
	{
		gprintk("called %s  - *#0589#\n",__func__);
		#if 1
	return sprintf(buf,"%d\n",cur_adc_value);
		#else 
		adc = s3c_adc_get_adc_data(ADC_CHANNEL);
		return sprintf(buf,"%d\n",adc);
		#endif
	}
}
static ssize_t lightsensor_file_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	gprintk("called %s \n",__func__);
  
    sscanf(buf, "%d", &value);

	return size;
}
#endif

/* for light sensor on/off control from platform */
static ssize_t lightsensor_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	gprintk("called %s \n",__func__);
	return sprintf(buf,"%u\n",light_enable);
}
static ssize_t lightsensor_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	int value;
    sscanf(buf, "%d", &value);
	int ret;

	printk(KERN_INFO "[LIGHT_SENSOR] in lightsensor_file_cmd_store, input value = %d, light_enable = %d \n",value, light_enable);
	switch(value){		// iamaj  add case 8, 9 for  enable light sensor without autobrightness
#ifdef LIGHT_SENSOR_ENABLED
		case 0 :
		{
			if(light_enable == ON)
			{
				gp2a_off(gp2a,LIGHT); //Light sensor is always on
			}
			lightsensor_test = 0;
			value = OFF;
			printk(KERN_INFO "[LIGHT_SENSOR] *#0589# test end, input value = %d \n",value);
			break;
		}
		case 1 : 
		{
			if(light_enable == OFF)
			{
				gp2a_on(gp2a,LIGHT);  //Light sensor is always on
			}
			lightsensor_test = 1;
			value = ON;
			printk(KERN_INFO "[LIGHT_SENSOR] *#0589# test start, input value = %d \n",value);
			break;
		}
#endif
	/* temporary test code for proximity sensor */
		case 2 :
		{
			if(proximity_enable == ON)
			{
				ret = gp2a_off(gp2a,PROXIMITY);
				if(ret==0) proximity_enable =0;
				printk("[PROXIMITY] Temporary : Power OFF\n");
			}
			break;
		}		
		case 3 :
		{
			if(proximity_enable == OFF){
				ret = gp2a_on(gp2a,PROXIMITY);
				if(ret==0) proximity_enable =1;
				printk("[PROXIMITY] Temporary : Power ON\n");
				}
			break;
		}
#ifdef LIGHT_SENSOR_ENABLED
		case 7 :
		{
			if(light_enable == OFF)
			{
				light_init_period = 2;
				gp2a_on(gp2a,LIGHT);
				value = 7;
			}
			break;
		}		
#endif
		default :
			break;
		}
				
	return size;
}

static DEVICE_ATTR(lightsensor_file_cmd,0660, lightsensor_file_cmd_show, lightsensor_file_cmd_store);
#ifdef LIGHT_SENSOR_ENABLED
static DEVICE_ATTR(lightsensor_file_state,0660, lightsensor_file_state_show, lightsensor_file_state_store);
#endif
/*					 */
/*					 */
/* for  test mode 	end  */
/*					 */
/*					 */


//------------------------------------------------------------------------------------
static int gp2a_opt_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = -1;
	int i;
	u8 value;	
#if USE_INTERRUPT
	int irq;
#endif
	int ret;

	struct gp2a_data *gp2a;

	printk("GP2A PROBE!\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return err;	

	/* OK. For now, we presume we have a valid client. We now create the
	client structure, even though we cannot fill it completely yet. */
	if (!(gp2a = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	i2c_set_clientdata(client, gp2a);
	opt_i2c_client = client;

   	if(opt_i2c_client == NULL)
	{
		pr_err("opt_probe failed : i2c_client is NULL\n"); 
		return -ENODEV;
	}
	else
		printk("opt_i2c_client : (%s)\n",opt_i2c_client->name);

	printk("[%s] slave addr = %x\n", __func__, client->addr);

#ifdef LIGHT_SENSOR_ENABLED
	/* hrtimer Settings */
	hrtimer_init(&gp2a->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gp2a->timer.function = gp2a_timer_func;

    gp2a_wq = create_singlethread_workqueue("gp2a_wq");
    if (!gp2a_wq)
    {
		printk("create_singlethread_workqueue error \r\n");
		return -ENOMEM;
    }
    INIT_WORK(&gp2a->work_light, gp2a_work_func_light);
    
	err = misc_register(&light_device);
	if(err) {
		pr_err(KERN_ERR "misc_register failed - light\n");
	}

	/* set sysfs for light sensor */
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
		pr_err("Failed to create class(lightsensor)!\n");

	switch_cmd_dev = device_create(lightsensor_class, NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
		pr_err("Failed to create device(switch_cmd_dev)!\n");

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);
  
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_state.attr.name);
	dev_set_drvdata(switch_cmd_dev,gp2a);

	gp2a_off(gp2a,LIGHT);
#endif

	/* Input device Settings */
	if(USE_INPUT_DEVICE)
	{
		gp2a->input_dev = input_allocate_device();
		if (gp2a->input_dev == NULL) 
		{
			pr_err("Failed to allocate input device\n");
			return -ENOMEM;
		}
		gp2a->input_dev->name = "proximity";
	
		set_bit(EV_SYN,gp2a->input_dev->evbit);
		set_bit(EV_ABS,gp2a->input_dev->evbit);
		
        input_set_abs_params(gp2a->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		
	
		err = input_register_device(gp2a->input_dev);
		if (err) 
		{
			pr_err("Unable to register %s input device\n", gp2a->input_dev->name);
			input_free_device(gp2a->input_dev);
			kfree(gp2a);
			return -1;
		}

	}

#if USE_INTERRUPT
	/* WORK QUEUE Settings */
	INIT_DELAYED_WORK(&gp2a->work_prox, gp2a_work_func_prox);
	gprintk("Workqueue Settings complete\n");
#endif

	/* misc device Settings */
	err = misc_register(&proximity_device);
	if(err) {
		pr_err(KERN_ERR "misc_register failed - prox \n");
	}

	/* wake lock init */
	wake_lock_init(&prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");
#if !defined(CONFIG_MACH_RANT3)
	wake_lock_init(&prx_irq_wake_lock, WAKE_LOCK_SUSPEND, "prx_irq_wake_lock");
#endif

	/* set sysfs for light sensor */
	proxsensor_class = class_create(THIS_MODULE, "proxsensor");
	if (IS_ERR(proxsensor_class))
		pr_err("Failed to create class(proxsensor)!\n");

	switch_cmd_dev = device_create(proxsensor_class, NULL, 0, NULL, "switch_cmd");	
	if (device_create_file(switch_cmd_dev, &dev_attr_proxsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proxsensor_file_state.attr.name);

#if defined(CONFIG_MACH_RANT3)
	if (device_create_file(switch_cmd_dev, &dev_attr_proxsensor_onoff_ctrl) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proxsensor_onoff_ctrl.attr.name);
#endif

	dev_set_drvdata(switch_cmd_dev,gp2a);
	
	/* ktime init */
	timeA = ktime_set(0,0);
	timeB = ktime_set(0,0);

	/* GP2A Regs INIT SETTINGS */
	value = 0x00;
	err = opt_i2c_write((u8)(REGS_OPMOD),&value);
	if(err == -EIO) // NC gp2a
	    goto exit_kfree;
	    
/*	
	for(i=1;i<5;i++)
	{
		opt_i2c_write((u8)(i),&gp2a_original_image[i]);
	}
*/
	mdelay(2);
#if USE_INTERRUPT
	/* INT Settings */	
	irq = gpio_to_irq(GPIO_PS_VOUT);
	gp2a->irq = -1;
	set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);

	err = request_irq(irq, gp2a_irq_handler, IRQF_DISABLED, "gp2a_int", gp2a);
	if (err)
	{
		printk("[GP2A] request_irq failed for gp2a\n");
		goto exit_kfree;
	}

	printk("[GP2A] register irq = %d\n",irq);
	err = set_irq_wake(irq, 1);
	printk("[GP2A] register wakeup source = %d\n",err);
	if (err) 
		printk("[GP2A] register wakeup source failed\n");
	
	gp2a->irq = irq;
	gprintk("INT Settings complete\n");
#endif

	// maintain power-down mode before using sensor
	ret = gp2a_off(gp2a,ALL);
#if !defined(CONFIG_MACH_RANT3)
	if(ret==0) proximity_enable = 0;
#endif

	printk("gp2a_opt_probe is OK!!\n");
	return 0;
	
exit_kfree:
	printk("gp2a_opt_probe is exit_kfree!!\n");
	kfree(gp2a);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void gp2a_opt_early_suspend(struct early_suspend *);
void gp2a_opt_late_resume(struct early_suspend *);


void gp2a_opt_early_suspend(struct early_suspend *h)
{
	int config = 0;
	int err=0;
	u8 value;
#ifdef LIGHT_SENSOR_ENABLED
        struct gp2a_data *gp2a = i2c_get_clientdata(opt_i2c_client);
#endif
	printk("[%s] proximity_enable : %d, proximity_value : %d\n", __func__, proximity_enable, proximity_value);
	if(!proximity_enable)
	{
		config = GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		err = gpio_tlmm_config(config, GPIO_CFG_ENABLE);
		if (err) 
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, GPIO_PS_VOUT, err);
	}
	else
	{
	      is_suspend = 1;
//        	value = 0x02;
//        	opt_i2c_write((u8)(REGS_OPMOD),&value);
	}

#ifdef LIGHT_SENSOR_ENABLED
        if(light_enable)
        {
                gprintk("[%s] : hrtimer_cancle \n",__func__);
                hrtimer_cancel(&gp2a->timer);
        }
#endif

#if !defined(CONFIG_MACH_RANT3) //&& !defined(CONFIG_MACH_ESCAPE)	koh82.kwon	11.06.09	For sleep current
	if(!proximity_enable)
	{
	err = vreg_disable(vreg_als); // MBjgnoh 11.01.17 For sleep current
	if(err)
	    printk(KERN_ERR,"can not enable ldo9\n");

			err = vreg_disable(vreg_led);
	if(err)
	    printk(KERN_ERR,"can not enable ldo18\n");
	}
#endif
}


void gp2a_opt_late_resume(struct early_suspend *h)
{
	int config = 0;
	int err = 0;
	u8 value;

#ifdef LIGHT_SENSOR_ENABLED
	struct gp2a_data *gp2a = i2c_get_clientdata(opt_i2c_client);
#endif

	printk("[%s] proximity_enable : %d, proximity_value : %d\n", __func__, proximity_enable, proximity_value);

#if !defined(CONFIG_MACH_RANT3) //&& !defined(CONFIG_MACH_ESCAPE)	koh82.kwon	11.06.09	For sleep current
	vreg_set_level(vreg_als, OUT3000mV); // MBjgnoh 11.01.17 For sleep current
	err = vreg_enable(vreg_als);

	if(err)
	    printk(KERN_ERR "can not enable ldo9\n");

	vreg_set_level(vreg_led, OUT3000mV);
	err = vreg_enable(vreg_led);

	if(err)
	    printk(KERN_ERR "can not enable ldo18\n");
#endif

	if(!proximity_enable)
	{
		config = GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
		err = gpio_tlmm_config(config, GPIO_CFG_ENABLE);
		if (err) 
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, GPIO_PS_VOUT, err);	
	}
	else
	{
            is_suspend = 0;	
	    
#if !defined(CONFIG_MACH_RANT3)
        	value = 0x18;	// 11:force to go High
        	opt_i2c_write((u8)(REGS_CON),&value);

        	value = 0x40;	// HYSD enable
        	opt_i2c_write((u8)(REGS_HYS),&value);

        	value = 0x03;	// VCON enable, SSD enable
        	opt_i2c_write((u8)(REGS_OPMOD),&value);

        	value = 0x00;	// 00:enable
        	opt_i2c_write((u8)(REGS_CON),&value);
#endif

			wake_lock_timeout(&prx_wake_lock,3 * HZ);
			timeA = ktime_get();
			printk("[%s] : wake_lock_timeout 3 Sec \n",__func__);

	}	

#ifdef LIGHT_SENSOR_ENABLED
        cur_state = LIGHT_INIT;

	if(light_enable)
        {
                gprintk("[%s] : hrtimer_cancle \n",__func__);
		/* start the timer in resume mode in case auto brightness is enabled  */
		hrtimer_start(&gp2a->timer,ktime_set(0,500000000),HRTIMER_MODE_REL);
        }
#endif	

}

#endif

#ifdef LIGHT_SENSOR_ENABLED

static int StateToLux(state_type state)
{
	int lux = 0;

	gprintk("[%s] cur_state:%d\n",__func__,state);
	
	switch (state) {
		case LIGHT_LEVEL1:
			lux = 10;
			break;
		case LIGHT_LEVEL2:
			lux = 70;
			break;
		case LIGHT_LEVEL3:
			lux = 1000;
			break;
		case LIGHT_LEVEL4:
			lux = 1500;
			break;
		default:
			lux = 1200;			
			break;
		}
	return lux;
}

static int light_open(struct inode *ip, struct file *fp)
{
	struct gp2a_data *gp2a = dev_get_drvdata(switch_cmd_dev);

	gprintk("[%s] \n",__func__);
	gp2a_on(gp2a,LIGHT);
	return 0;

}

static ssize_t light_read(struct file *filp, char *lux, size_t count, loff_t *f_pos)
{
	long lux_val;

	lux_val = StateToLux(cur_state);
	put_user(lux_val, lux);
	
	return 1;
}

static int light_release(struct inode *ip, struct file *fp)
{
	struct gp2a_data *gp2a = dev_get_drvdata(switch_cmd_dev);

	gp2a_off(gp2a,LIGHT);	
	gprintk("[%s] \n",__func__);
	return 0;
}
#endif

static int gp2a_opt_remove(struct i2c_client *client)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);

	gprintk("%s\n",__FUNCTION__);

	if(USE_INPUT_DEVICE)
		input_unregister_device(gp2a->input_dev);
	kfree(gp2a);

	misc_deregister(&proximity_device);

	return 0;
}

static int gp2a_opt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);

	int config = 0;
	int err = 0;
	u8 value;
	
	printk("[HSIL] %s\n", __func__);
	if(!proximity_enable)
	{
		config = GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
		err = gpio_tlmm_config(config, GPIO_CFG_ENABLE);
		if (err) 
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, GPIO_PS_VOUT, err);
	}
	else
	{
#if USE_INTERRUPT	  
		disable_irq_nosync(gp2a ->irq);
#endif
		value = 0x02;
		opt_i2c_write((u8)(REGS_OPMOD),&value);
		printk(KERN_INFO "[%s] GP2A !!suspend mode proximity_enable \n",__FUNCTION__);
	}
	return 0;
}

static int gp2a_opt_resume(struct i2c_client *client)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);

	int config = 0;
	int err = 0;
	u8 value;

	printk("[HSIL] %s\n", __func__);
	if(!proximity_enable)
	{
		config = GPIO_CFG(GPIO_PS_VOUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
		err = gpio_tlmm_config(config, GPIO_CFG_ENABLE);
		if (err) 
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, GPIO_PS_VOUT, err);	
	}
	else
	{
		value = 0x18;
		opt_i2c_write((u8)(REGS_CON),&value);

#if defined(CONFIG_MACH_RANT3)
		value = 0x2F;
#else
		value = 0x40;
#endif
		opt_i2c_write((u8)(REGS_HYS),&value);
		
		
		value = 0x03;
		opt_i2c_write((u8)(REGS_OPMOD),&value);
#if USE_INTERRUPT
		enable_irq(gp2a->irq);
#endif		
		value = 0x00;
		opt_i2c_write((u8)(REGS_CON),&value);

	       wake_lock_timeout(&prx_wake_lock,3 * HZ);
		timeA = ktime_get();
		printk("[%s] : wake_lock_timeout 3 Sec \n",__func__);
	}
	return 0;
}

static const struct i2c_device_id gp2a_id[] = {
	{ "gp2a", 0 },
	{ }
};

static struct i2c_driver gp2a_opt_driver = {
	.driver  = {
		.owner	= THIS_MODULE,	
		.name = "gp2a",
	},
	.id_table	= gp2a_id,
	.probe		= gp2a_opt_probe,
	.remove		= gp2a_opt_remove,
//	.suspend        = gp2a_opt_suspend,
//	.resume	        = gp2a_opt_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend optsens_earlysuspend = {
  	.suspend = gp2a_opt_early_suspend,
	.resume = gp2a_opt_late_resume,
};
#endif

static int __init gp2a_opt_init(void)
{
	printk("%s\n",__FUNCTION__);
	
	gp2a_chip_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&optsens_earlysuspend);
#endif

	return i2c_add_driver(&gp2a_opt_driver);
}

static void __exit gp2a_opt_exit(void)
{

#ifdef LIGHT_SENSOR_ENABLED
	struct gp2a_data *gp2a = dev_get_drvdata(switch_cmd_dev);
	
    if (gp2a_wq)
		destroy_workqueue(gp2a_wq);

	kfree(gp2a);
#endif	
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&optsens_earlysuspend);
#endif

	i2c_del_driver(&gp2a_opt_driver);

	gprintk("%s\n",__FUNCTION__);
}

module_init( gp2a_opt_init );
module_exit( gp2a_opt_exit );

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for gp2ap002a00f");
MODULE_LICENSE("GPL");
