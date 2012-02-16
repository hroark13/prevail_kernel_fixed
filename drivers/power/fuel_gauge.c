#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>

#include <linux/slab.h>

#if 0
/* Slave address */
#define MAX17040_SLAVE_ADDR	0x6D
#endif



/* Register address */
#define VCELL0_REG			0x02
#define VCELL1_REG			0x03
#define SOC0_REG			0x04
#define SOC1_REG			0x05
#define MODE0_REG			0x06
#define MODE1_REG			0x07
#define RCOMP0_REG			0x0C
#define RCOMP1_REG			0x0D
#define CMD0_REG			0xFE
#define CMD1_REG			0xFF


#define ALERT_THRESHOLD_VALUE_01 0x1E // 1% alert
#define ALERT_THRESHOLD_VALUE_05 0x1A // 5% alert
#define ALERT_THRESHOLD_VALUE_15 0x10 // 15% alert
#define I2C_2_SDA 110
#define I2C_2_SCL 109


//#define __ADVANCED_SOC_VALUE__
#define SOC_LB_FOR_POWER_OFF	27	// hanapark_Victory

//#define __FUEL_GAUGE_DEBUG__ //MBksjung
#if defined(CONFIG_MACH_ROOKIE)
#define __BATT_SOC_TEST_LOG__ //MBksjung
#endif
static struct i2c_driver fg_i2c_driver;
static struct i2c_client *fg_i2c_client = NULL;

static int is_reset_soc = 0;

extern unsigned char hw_version;

#ifdef __BATT_SOC_TEST_LOG__
int gRealsoc;
int gRealsocData0;
int gRealsocData1;
#endif //__BATT_SOC_TEST_LOG__
struct fuel_gauge_data
{
	struct work_struct work;
};

static struct determin_table_change_battery {
	u32 max_adc;
	u32 min_adc;
	u32 y_interception;
	u32 slope;
} max17043_table[10] = {
	 { 412000000, 405650000, 206791949, 2026527},
	 { 405650000, 391090000, 316480978,  908615},
	 { 391090000, 376600000, 347102193,  535708},
	 { 376600000, 371150000, 363280169,  241922},
	 { 371150000, 367680000, 363440479,  237041},
	 { 367680000, 362890000, 356812112,  607598},
	 { 362890000, 359520000, 356904982,  598762},
	 { 359520000, 354850000, 349428113, 2314224},
	 { 354850000, 347780000, 342568757, 5252476},
	 { 347780000, 339960000, 339961760, 7910562},
};

static int fg_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg; 

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) 
		return -EIO;

	*data = buf[0];
	
	return 0;
}

static int fg_i2c_write(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[3];

	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = *data;
	buf[2] = *(data + 1);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) 
		return -EIO;

	return 0;
}

unsigned int fg_read_vcell(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 vcell = 0;

	if (fg_i2c_read(client, VCELL0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read VCELL0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, VCELL1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read VCELL1\n", __func__);
		return -1;
	}
	vcell = ((((data[0] << 4) & 0xFF0) | ((data[1] >> 4) & 0xF)) * 125)/100;

	return vcell;
}

void Is_Interrupt(void)
{
	struct i2c_client *client = fg_i2c_client;	
	u8 data[1];
	u8 rst_cmd[2];
	s32 ret = 0;

	extern int fg_alert;

	if (fg_i2c_read(client, RCOMP1_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read rcomp1\n", __func__);
		return -1;
	}
	data[0] = ((data[0] >> 5) & 0x1);
	if(data[0] == 1)
	{
		printk("interrupt accured ============= > OK\n");
		rst_cmd[0] = 0xB7;
		rst_cmd[1] = 0x1C;

		
		pr_info("%s: Write RCOMP = 0x%x%x\n", __func__, rst_cmd[0], rst_cmd[1]);
	
		ret = fg_i2c_write(client, RCOMP0_REG, rst_cmd);
		if (ret)
		{
			pr_err("%s: Failed to write RCOMP...\n", __func__);
			return;
		}
		fg_alert = 0;
	}
}

unsigned int fg_read_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	
	int adj_soc = 100;
	extern int fg_gFull_chg;

	if (fg_i2c_read(client, SOC0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read SOC0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOC1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read SOC1\n", __func__);
		return -1;
	}

	if (is_reset_soc) 
	{
		pr_info("%s: Reseting SOC\n", __func__);
		return -1;
	} 
	else 
	{
		if (data[0] || data[1])
		{
#if 1 // MBjdyoo 2011.03.14 : Value of FUEL GAUGE is returned different SOC value when voltage is 4.2/3.7V.

			adj_soc = (((data[0]*10 + (data[1]>>7)*5 ) - 14) * 100) / (948 - 14); 
		
#ifdef __FUEL_GAUGE_DEBUG__			
			printk(KERN_CRIT "[[[[[[[[[[[[[[[[[[[[[FUEL GUAGE]]]]]]]]]]]]]]]]]]] \n");
            printk(KERN_CRIT "[Battery_Fuel_Gauge] %s : data[0]  : %d data[1]: %d\n", __func__, data[0], data[1]);
			printk(KERN_CRIT "[Battery_Fuel_Gauge] %s : Real SOC : %d \n", __func__, (data[0]*100 + (data[1]>>6)*25 )/10 );

			printk(KERN_CRIT "[Battery_Fuel_Gauge] %s : adj_SOC : %d \n", __func__, adj_soc );	
#endif //__FUEL_GAUGE_DEBUG__				
#ifdef __BATT_SOC_TEST_LOG__
gRealsocData0 = data[0];
gRealsocData1 = data[1];
gRealsoc = (data[0]*100 + (data[1]>>6)*25 )/10;
#endif //__BATT_SOC_TEST_LOG__
#else
			adj_soc = (((data[0]*10) - 13) * 100) / (959 - 13); 
#endif
		
			if (adj_soc >= 100)
			{
				if(fg_gFull_chg)
				adj_soc = 100;
				else
					adj_soc = 99;
			}
			else if (adj_soc < 0)
				adj_soc = 0;

#ifdef __FUEL_GAUGE_DEBUG__
            printk(KERN_CRIT "[Battery_Fuel_Gauge] %s :  Return adj_SOC : %d \n", __func__, adj_soc );
            printk(KERN_CRIT "[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]] \n");
#endif //__FUEL_GAUGE_DEBUG__
			return (unsigned int)adj_soc;
		}
		else 
		{

			if (data[1] > SOC_LB_FOR_POWER_OFF)
			{
#ifdef __FUEL_GAUGE_DEBUG__			
				printk(KERN_CRIT "[Battery_Fuel_Gauge] %s IF SOC_LB_FOR_POWER_OFF : data[0]  : %d data[1]: %d\n", __func__, data[0], data[1]);
#endif //__FUEL_GAUGE_DEBUG__
				return 1;
			}
			else
			{
#ifdef __FUEL_GAUGE_DEBUG__			
			    printk(KERN_CRIT "[Battery_Fuel_Gauge] %s ELSE SOC_LB_FOR_POWER_OFF : data[0]  : %d data[1]: %d\n", __func__, data[0], data[1]);
#endif //__FUEL_GAUGE_DEBUG__
				return 0;
		}
		}
	}
}

unsigned int fg_read_real_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];

	if (fg_i2c_read(client, SOC0_REG, &data[0]) < 0) {
		pr_err("%s: Failed to read SOC0\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOC1_REG, &data[1]) < 0) {
		pr_err("%s: Failed to read SOC1\n", __func__);
		return -1;
	}

	if (is_reset_soc) {
		pr_info("%s: Reseting SOC\n", __func__);
		return -1;
	} else {
		if (data[0])
			return data[0];
		else {
			if (data[1] > SOC_LB_FOR_POWER_OFF)
				return 1;
			else
				return 0;
		}
	}
}

unsigned int fg_reset_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 rst_cmd[2];
	s32 ret = 0;

	is_reset_soc = 1;

	/* Quick-start */
	rst_cmd[0] = 0x40;
	rst_cmd[1] = 0x00;

	ret = fg_i2c_write(client, MODE0_REG, rst_cmd);
	if (ret)
		pr_err("%s: Failed to reset SOC(%d)\n", __func__, ret);

	msleep(500);
	is_reset_soc = 0;

	return ret;
}

unsigned int check_quick_start_condition(void)
{
	unsigned int adc_raw, adc;
	int soc, soc_tmp;
	int i;

	/* get vcell. */
	adc = fg_read_vcell();
	adc = adc * 100000;

	/* get soc. */
	soc = fg_read_real_soc();

	if ((adc >= 412000000 && soc <= 90) ||
			(adc <= 340000000 && soc >= 4)) {
		fg_reset_soc();
		return 0;
	}

	for (i = 0; i < (int)ARRAY_SIZE(max17043_table); i++) {
		if (adc < max17043_table[i].max_adc &&
				adc >= max17043_table[i].min_adc) {
			int limit_min, limit_max;

			soc_tmp = (adc - max17043_table[i].y_interception);
			soc_tmp = soc_tmp / max17043_table[i].slope;

			limit_min = soc_tmp - 15;
			limit_max = soc_tmp + 15;

			if (limit_min < 0)
				limit_min = 0;
			if(limit_max > 100)
				limit_max = 100;

			if (soc > limit_max || soc < limit_min) 
			{
				fg_reset_soc();
			}
		}
	}
	return 0;

}

static void fuel_gauge_write_rcomp(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 rst_cmd[2];
#if 1
	s32 ret = 0;
#if defined(CONFIG_MACH_ROOKIE)// MBhlee 2011.03.08 : Change RCOMP value to A71C->C000
	if(hw_version >= 7){
	    rst_cmd[0] = 0xC0; //REV07 FF -> C0 // MBksjung 2011.05.25 : HW Request C0->FF
	    rst_cmd[1] = 0x1C;				///MBjdyoo 2011.03.17 :  Restore to the former condition (0x00->0x1C)
	}
	else{
	    rst_cmd[0] = 0xFF; // REV06
	    rst_cmd[1] = 0x1C;	
	}
#elif defined(CONFIG_MACH_ESCAPE)  
	rst_cmd[0] = 0xC0;
	rst_cmd[1] = 0x1C;	
#elif defined(CONFIG_MACH_GIO) 
	rst_cmd[0] = 0xD0;	// 0xA0
	rst_cmd[1] = 0x1E;		 
#elif defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	rst_cmd[0] = 0xB7;
	rst_cmd[1] = 0x1C;	
#elif defined(CONFIG_MACH_RANT3)
	rst_cmd[0] = 0xC0;
	rst_cmd[1] = 0x1C;	//Real_soc == 3% 	
#else
	rst_cmd[0] = 0x97;
	rst_cmd[1] = 0x1C;
#endif
  
	pr_info("%s: Write RCOMP = 0x%x%x\n", __func__, rst_cmd[0], rst_cmd[1]);

	ret = fg_i2c_write(client, RCOMP0_REG, rst_cmd);
  
	if (ret)
		pr_err("%s: Failed to write RCOMP...\n", __func__);
#else
/*	RCOMP 실측값 넣기 전이므로 일단 chip default 값인 0x971C를 사용하도록 한다. */
	fg_i2c_read(client, RCOMP0_REG, &rst_cmd[0]);
	fg_i2c_read(client, RCOMP1_REG, &rst_cmd[1]);
	pr_info("%s: Read RCOMP = 0x%x%x\n", __func__, rst_cmd[0], rst_cmd[1]); 	// It might be 0x971C !
#endif
}

static int fuel_gauge_init_client(struct i2c_client *client)
{
	/* Initialize the max17043 Chip */
//	init_waitqueue_head(&g_data_ready_wait_queue);
	return 0;
}

static void fuel_gauge_chip_init(void)
{
	fuel_gauge_write_rcomp();
}

static int fuel_gauge_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fuel_gauge_data *mt;
	int err = -1;

	gpio_tlmm_config(GPIO_CFG(I2C_2_SDA , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(I2C_2_SCL , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		goto exit_check_functionality_failed;
	}

	if (!(mt = kzalloc(sizeof(struct fuel_gauge_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, mt);
	fuel_gauge_init_client(client);
	fg_i2c_client = client;

	fuel_gauge_chip_init();

	return 0;

exit_alloc_data_failed:
exit_check_functionality_failed:
	pr_err("%s: Error! (%d)\n", __func__, err);
	return err;

}

static int fuel_gauge_remove(struct i2c_client *client)
{
	struct fuel_gauge_data *mt = i2c_get_clientdata(client);
	printk(KERN_INFO "fuel_gauge_remove: start\n");

//	free_irq(client->irq, mt);

	//khsuh_imsi i2c_detach_client(client);
	fg_i2c_client = NULL;

	kfree(mt);
	return 0;
}


static const struct i2c_device_id fuel_gauge_id[] = {
	{ "fuelgauge", 0 },
	{ }
};

static struct i2c_driver fg_i2c_driver = {
	.probe		= fuel_gauge_probe,
	.remove 	= fuel_gauge_remove,
	.id_table = fuel_gauge_id,
	.driver = {
		.name 	= "fuelgauge",
	},
};

int fuel_gauge_init(void)
{
	int ret;

	ret = i2c_add_driver(&fg_i2c_driver);
	if (ret)
		pr_err("%s: Failed to add fuel gauge i2c driver...\n", __func__);
 
	//check_quick_start_condition();

	fg_read_vcell();

	return ret;
}

void fuel_gauge_exit(void)
{
	i2c_del_driver(&fg_i2c_driver);
}

