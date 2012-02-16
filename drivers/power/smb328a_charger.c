/*
 *  SMB328A-charger.c
 *  SMB328A charger interface driver
 *
 *  Copyright (C) 2011 Samsung Electronics
 *
 *  <jongmyeong.ko@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>
#if 0    // TODO: @disys compile error, re-check
#include <linux/smb328a_charger.h>
#include <linux/i2c/fsa9480.h>
#endif

// TODO: @disys temp
struct smb328a_platform_data {
	void (*hw_init)(void);
};



/* Register define */
#define SMB328A_INPUT_AND_CHARGE_CURRENTS	0x00
#define SMB328A_CURRENT_TERMINATION			0x01
#define SMB328A_FLOAT_VOLTAGE				0x02
#define SMB328A_FUNCTION_CONTROL_A1			0x03
#define SMB328A_FUNCTION_CONTROL_A2			0x04
#define SMB328A_FUNCTION_CONTROL_B			0x05
#define SMB328A_OTG_PWR_AND_LDO_CONTROL		0x06
#define SMB328A_VARIOUS_CONTROL_FUNCTION_A	0x07
#define SMB328A_CELL_TEMPERATURE_MONITOR	0x08
#define SMB328A_INTERRUPT_SIGNAL_SELECTION	0x09
#define SMB328A_I2C_BUS_SLAVE_ADDRESS		0x0A

#define SMB328A_CLEAR_IRQ					0x30
#define SMB328A_COMMAND						0x31
#define SMB328A_INTERRUPT_STATUS_A			0x32
#define SMB328A_BATTERY_CHARGING_STATUS_A	0x33
#define SMB328A_INTERRUPT_STATUS_B			0x34
#define SMB328A_BATTERY_CHARGING_STATUS_B	0x35
#define SMB328A_BATTERY_CHARGING_STATUS_C	0x36
#define SMB328A_INTERRUPT_STATUS_C			0x37
#define SMB328A_BATTERY_CHARGING_STATUS_D	0x38
#define SMB328A_AUTOMATIC_INPUT_CURRENT_LIMMIT_STATUS	0x39

/* Set Value define */
#if defined(CONFIG_MACH_ESCAPE)
#define SMB328A_INPUT_AND_CHARGE_CURRENTS_SET_VAULE	0x9E	/* 900mA */
#elif defined(CONFIG_MACH_GIO)
#define SMB328A_INPUT_AND_CHARGE_CURRENTS_SET_VAULE	0x5E	/* 700mA */
#else
#define SMB328A_INPUT_AND_CHARGE_CURRENTS_SET_VAULE	0x9E	/* 900mA */
#endif

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

enum {
	CHG_MODE_NONE,
	CHG_MODE_AC,
	CHG_MODE_USB
};

struct smb328a_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct power_supply		psy_bat;
	struct smb328a_platform_data	*pdata;

	int chg_mode;
};
static struct smb328a_chip  *g_chip = NULL ;

static enum power_supply_property smb328a_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

extern unsigned char hw_version;

static int smb328a_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

//   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb328a_read_reg(struct i2c_client *client, int reg)
{
	int ret;

//   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void smb328a_print_reg(struct i2c_client *client, int reg)
{
	u8 data = 0;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	data = i2c_smbus_read_byte_data(client, reg);

	if (data < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, data);
	else
		printk("%s : reg (0x%x) = 0x%x\n", __func__, reg, data);
}

static void smb328a_print_all_regs(struct i2c_client *client)
{
   printk(KERN_ERR "[SMB328a][%s] ----- START -----\n", __func__);   // @disys
	smb328a_print_reg(client, 0x31);
	smb328a_print_reg(client, 0x32);
	smb328a_print_reg(client, 0x33);
	smb328a_print_reg(client, 0x34);
	smb328a_print_reg(client, 0x35);
	smb328a_print_reg(client, 0x36);
	smb328a_print_reg(client, 0x37);
	smb328a_print_reg(client, 0x38);
	smb328a_print_reg(client, 0x39);
	smb328a_print_reg(client, 0x00);
	smb328a_print_reg(client, 0x01);
	smb328a_print_reg(client, 0x02);
	smb328a_print_reg(client, 0x03);
	smb328a_print_reg(client, 0x04);
	smb328a_print_reg(client, 0x05);
	smb328a_print_reg(client, 0x06);
	smb328a_print_reg(client, 0x07);
	smb328a_print_reg(client, 0x08);
	smb328a_print_reg(client, 0x09);
	smb328a_print_reg(client, 0x0a);

   printk(KERN_ERR "[SMB328a][%s] ----- END -----\n", __func__);   // @disys
}

static void smb328a_allow_volatile_writes(struct i2c_client *client)
{
	int val;
	u8 data;

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if ((val >= 0) && !(val&0x80)) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data |= (0x1 << 7);
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			printk("%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8)data;
			printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
}

static void smb328a_charger_function_conrol(struct i2c_client *client)
{
	int val;
	u8 data;

	smb328a_allow_volatile_writes(client);

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
		if (data != SMB328A_INPUT_AND_CHARGE_CURRENTS_SET_VAULE) { /* this can be changed with top-off setting */	// GIO 700mA  0x5E
            /*  100x xxxx : Fast Charge Current = 900mA		// 010 = 700mA
                xxx1 1xxx : Pre-Charge Current = 120mA,
                xxxx x110 : Charge Completion Termination Current = 175mA */
			data = SMB328A_INPUT_AND_CHARGE_CURRENTS_SET_VAULE;
			if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_CURRENT_TERMINATION, data);
		if (data != 0x90) {
            /* 101x xxxx : AC Input Current Limit = 900mA
               xxx1 xxxx : Pre-Bias Current = Enabled
               xxxx x0xx : Automatic Input Current Limit = Enabled
               xxxx xx00 : Automatic Input Current Limit Threshold = 4.25V */
			data = 0x90;
			if (smb328a_write_reg(client, SMB328A_CURRENT_TERMINATION, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CURRENT_TERMINATION, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FLOAT_VOLTAGE, data);
		if (data != 0xca) {
			data = 0xca; /* 4.2V float voltage */
			if (smb328a_write_reg(client, SMB328A_FLOAT_VOLTAGE, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FLOAT_VOLTAGE, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A1, data);
		if (data != 0xd2) {
            /* 1xxx xxxx : Automatic Recharge = Enabled
               x1xx xxxx : Current Termination = Not Allowed to end a charge cycle
               xx01 0xxx : Pre-Charge to Fast-charge Voltage Threshold = 2.5V
               xxxx x0xx : LDO input Under-voltage Level = 3.50V
               xxxx xx10 : Not Used (Default)                                  */
			data = 0xd2;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A1, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A1, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A2, data);
		if (data != 0x45) {
            /* 0xxx xxxx : STATOutput = Indicates Charging State
               x1xx xxxx : Battery OV = Battery OV does cause charge cycle to end
               xx0x xxxx : Reload Functionality = Reload non-volatile values on valid input power presence
               xxx0 xxxx : Pre-to-fast charge Functionality = Enabled
               xxxx 0xxx : Charge Safety Timers = Enabled
               xxxx x1xx : Not Used
               xxxx xx0x : Watchdog Timer = disabled
               xxxx xxx1 : Interrupt(IRQ) Output = Enabled */
			data = 0x45;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_A2, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_A2, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
		if (data != 0x0) {
            /* 0xxx xxxx : Battery missing Detection Method = BMD pin
               xxxx 00xx : Enable(EN) Control = "0" in 0x31[4] turns on (enables) charger
               xxxx xx0x : Fast Charge Current Control Method = Config. Reg.
               xxxx xxx0 : Input Current Limit Control Method = Command Reg.    */
			data = 0x0;
			if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
#if defined (CONFIG_TARGET_LOCALE_USA)
		if (data != 0x45) {
			data = 0x45;
#else
        /*  0xxx xxxx : Battery Missing Detection = Disable
            x1xx xxxx : Automatic Recharge Threshold = 105mV
            xx1x xxxx : LDO Control = Disable
            xxx0 0xxx : OTG Current Limit = 950mA
            xxxx x101 : OTG Mode UVLO Threshold = 3.30V         */
		if (data != 0x65 /* 0xc5 */) {
			data = 0x65 ; /* 0xc5 SUMMIT_REQ */
#endif
			if (smb328a_write_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		if (data != 0xf6) { /* this can be changed with top-off setting */
            /* 111x xxxx : STAT Assertion Termination Current = 200mA
               xxx1 0xxx : Float Volatge Compensation Level = 180mV
               xxxx x11x : Thermistor Current = 0uA(Disabled)
               xxxx xxx0 : Float Voltage Compensation = Disabled        */
			data = 0xf6;
			if (smb328a_write_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_CELL_TEMPERATURE_MONITOR, data);
		if (data != 0x0) {
			data = 0x0;
			if (smb328a_write_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_CELL_TEMPERATURE_MONITOR, data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
	if (val >= 0) {
		data = (u8)val;
//		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
		if (data != 0x0) {
            /* Does not Trigger interrupt Signal(IRQ) */
			data = 0x0;
			if (smb328a_write_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION, data) < 0)
				printk("%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
			if (val >= 0) {
				data = (u8)val;
				printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
			}
		}
	}

	/* command register setting */
#if 0
	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data |= (0x1 << 3 | 0x1 << 5);
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			printk("%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8)data;
			printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
#endif
}

static int smb328a_check_charging_status(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	int ret = -1;

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		ret = (data&(0x3<<1))>>1;
		printk("%s : status = 0x%x\n", __func__, data);
	}

	return ret;
}

static bool smb328a_check_is_charging(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data&0x1)
			ret = true; /* charger enabled */
	}

	return ret;
}

static bool smb328a_check_bat_full(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data&(0x1<<6))
			ret = true; /* full */
	}

	return ret;
}

/* vf check */
static bool smb328a_check_bat_missing(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_B);
	if (val >= 0) {
		data = (u8)val;
		//printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_B, data);

		if (data&(0x1<<1)) {
			printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_B, data);
			ret = true; /* missing battery */
		}
	}

	return ret;
}

/* whether valid dcin or not */
static bool smb328a_check_vdcin(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_A);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_BATTERY_CHARGING_STATUS_A, data);

		if (data&(0x1<<1))
			ret = true;
	}

	return ret;
}

static int smb328a_chg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct smb328a_chip *chip = container_of(psy,
				struct smb328a_chip, psy_bat);

    printk(KERN_ERR "[SMB328a][%s] ### psp[%d]\n", __func__, psp);    // @disys
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (smb328a_check_vdcin(chip->client))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
#if 1 // @disys   from shlim
         val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
#else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (smb328a_check_bat_missing(chip->client))
			val->intval = BAT_NOT_DETECTED;
		else
			val->intval = BAT_DETECTED;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* battery is always online */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (smb328a_check_bat_full(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (smb328a_check_charging_status(chip->client)) {
			case 0:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
				break;
			case 1:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
				break;
			case 2:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
				break;
			case 3:
				val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
				break;
			default:
				printk("get charge type error!\n");
				return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (smb328a_check_is_charging(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb328a_set_top_off(struct i2c_client *client, int top_off)
{
	int val, set_val = 0;
	u8 data;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	smb328a_allow_volatile_writes(client);

	set_val = top_off/25;
	set_val -= 1;

	if (set_val < 0 || set_val > 7) {
		printk("%s: invalid topoff set value(%d)\n", __func__, set_val);
		return -EINVAL;
	}

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8)val & ~(0x07<<0) ;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
		data |= (set_val << 0);
		if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8)val & ~(0x07 << 5) ;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		data |= (set_val << 5);
		if (smb328a_write_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
	}

	return 0;
}

static int smb328a_set_charging_current(struct i2c_client *client, int chg_current)
{
	int val;
	u8 data, ac_usb, input, output;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

    printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	if (chg_current < 200 || chg_current > 950)
		return -EINVAL;

	smb328a_allow_volatile_writes(client);

	// disable, at auto-limit case
	val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
	if (val >= 0) {
		input = (u8)val;
		input &= ~(0x7 << 5);
	} else {
		printk("%s: can't read register(0x%x)\n",
			__func__, SMB328A_CURRENT_TERMINATION);
		return -1;
	}
	

	
	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		output = (u8)val;
		output &= ~(0x7 << 5);
	} else {
		printk("%s: can't read register(0x%x)\n",
			__func__, SMB328A_INPUT_AND_CHARGE_CURRENTS);
		return -1;
	}
	

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		ac_usb = (u8)val;
		ac_usb &= ~(0x1 << 2);
	} else {
		printk("%s: can't read register(0x%x)\n",
			__func__, SMB328A_COMMAND);
		return -1;
	}
#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
	if (chg_current == 450) {
		input |= (0x0 << 5); /* 450mA disable, at auto-limit case */
		output |= (0x0 << 5); /* 500mA */
		ac_usb |= (0x1 << 2); //AC Mode
		
		if (smb328a_write_reg(client, SMB328A_CURRENT_TERMINATION, input) < 0) {
			printk("%s : 450 error(1)!\n", __func__);
			return -1;
		}
		if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, output) < 0) {
			printk("%s : 450 error(2)!\n", __func__);
			return -1;
		}
		
		if (smb328a_write_reg(client, SMB328A_COMMAND, ac_usb) < 0) {
			printk("%s : 450 set ac_usb error error!\n", __func__);
			return -1;
		}
	} else if (chg_current == 900) {

#if defined(CONFIG_MACH_ESCAPE)
		if(hw_version >= 4){
			input |= (0x4 << 5); /* 900mA disable, at auto-limit case */
			output |= (0x4 << 5); /* 900mA */
		}else{
			input |= (0x4 << 5); /* 900mA disable, at auto-limit case */
			output |= (0x3 << 5); /* 800mA */
		}
#elif defined(CONFIG_MACH_GIO)
		input |= (0x2 << 5); /* 700mA */
		output |= (0x2 << 5); /* 700mA */
#endif
		ac_usb |= (0x1 << 2);

		//disable, at auto-limit case
		if (smb328a_write_reg(client, SMB328A_CURRENT_TERMINATION, input) < 0) {
			printk("%s : 600 error!(1)\n", __func__);
			return -1;
		}
		if (smb328a_write_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS, output) < 0) {
			printk("%s : 600 error!(2)\n", __func__);
			return -1;
		}
		
		if (smb328a_write_reg(client, SMB328A_COMMAND, ac_usb) < 0) {
			printk("%s : 600 set ac_usb error!\n", __func__);
			return -1;
		}
	} else {
		printk("%s : error! invalid setting current\n", __func__);
		return -1;
	}

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}	
	
#endif
	if (chg_current == 450) {
		chip->chg_mode = CHG_MODE_USB;
	} else if (chg_current == 900) {
		chip->chg_mode = CHG_MODE_AC;
	} else {
		printk("%s : error! invalid setting current\n", __func__);
		chip->chg_mode = CHG_MODE_NONE;
		return -1;
	}

	return 0;
}

static int smb328a_enable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);
	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		if (data != 0x9a)
		{
			printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
			data = 0x9a;
			if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
				printk("%s : error!\n", __func__);
				return -1;
			}
			msleep(100);

			data = smb328a_read_reg(client, SMB328A_COMMAND);
			printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		}
	}
	return 0;
}

static int smb328a_disable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);
	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);
		data = 0x0c;
		if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_FUNCTION_CONTROL_B, data);

	}

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
#if 0    // TODO: @disys compile error, re-check
		fsa9480_otg_detach();
#endif
	}
	return 0;
}
//

static int smb328a_enable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		//data &= ~(0x1 << 4); /* "0" turn off the charger */
		if (chip->chg_mode == CHG_MODE_AC)
			data = 0x8c;
		else if (chip->chg_mode == CHG_MODE_USB){
#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
			data = 0x8c; //set AC Mode for 450mA limit
#else
			data = 0x88;
#endif
		}else
			data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}

	return 0;
}

static int smb328a_disable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	//printk("%s : \n", __func__);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
		//data |= (0x1 << 4); /* "1" turn off the charger */
		data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk("%s : error!\n", __func__);
			return -1;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		printk("%s : => reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND, data);
	}

	return 0;
}

int smb328a_charger_control(int fast_charging_current, int termination_current, int on)
{
    int ret ;

    if (!g_chip)
        return -ENODEV ;

    if (on)
    {
        /* step1) Set charging current */
    	smb328a_charger_function_conrol(g_chip->client);
    	ret = smb328a_set_charging_current(g_chip->client, fast_charging_current);

        /* step2) Set top-off current */
    	if (termination_current < 25 || termination_current > 200) {
    		dev_err(&g_chip->client->dev, "%s: invalid topoff current(%d)\n",
    				__func__, termination_current);
    		return -EINVAL;
    	}
    	ret = smb328a_set_top_off(g_chip->client, termination_current);

    	/* step3) Enable/Disable charging */
    	ret = smb328a_enable_charging(g_chip->client);
    }
    else{
		smb328a_charger_function_conrol(g_chip->client);
		
    	ret = smb328a_disable_charging(g_chip->client);
    }
    return 0 ;
}
EXPORT_SYMBOL(smb328a_charger_control) ;

static int smb328a_chg_set_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    const union power_supply_propval *val)
{
	struct smb328a_chip *chip = container_of(psy,
				struct smb328a_chip, psy_bat);
	int ret;

    printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW: /* step1) Set charging current */
		smb328a_charger_function_conrol(chip->client);
		//smb328a_print_all_regs(chip->client);
		ret = smb328a_set_charging_current(chip->client, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: /* step2) Set top-off current */
		if (val->intval < 25 || val->intval > 200) {
			dev_err(&chip->client->dev, "%s: invalid topoff current(%d)\n",
					__func__, val->intval);
			return -EINVAL;
		}
		ret = smb328a_set_top_off(chip->client, val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:	/* step3) Enable/Disable charging */
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING)
			ret = smb328a_enable_charging(chip->client);
		else
			ret = smb328a_disable_charging(chip->client);
		//smb328a_print_all_regs(chip->client);
		break;
#if 0    // TODO: @disys compile error, re-check
	case POWER_SUPPLY_PROP_OTG:
		if (val->intval == POWER_SUPPLY_CAPACITY_OTG_ENABLE)
		{
			smb328a_charger_function_conrol(chip->client);
			ret = smb328a_enable_otg(chip->client);
		}
		else
			ret = smb328a_disable_otg(chip->client);
		break;
#endif
	default:
		return -EINVAL;
	}
	return ret;
}

static int __devinit smb328a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb328a_chip *chip;
	int ret;

   printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	printk("%s: SMB328A driver Loading! \n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	chip->pdata->hw_init(); /* important */

#if 0 /* ESCAPE */
	chip->psy_bat.name = "smb328a-charger",
	chip->psy_bat.type = POWER_SUPPLY_TYPE_BATTERY,
	chip->psy_bat.properties = smb328a_battery_props,
	chip->psy_bat.num_properties = ARRAY_SIZE(smb328a_battery_props),
	chip->psy_bat.get_property = smb328a_chg_get_property,
	chip->psy_bat.set_property = smb328a_chg_set_property,
	ret = power_supply_register(&client->dev, &chip->psy_bat);
	if (ret) {
		pr_err("Failed to register power supply psy_bat\n");
		goto err_kfree;
	}

	chip->chg_mode = CHG_MODE_NONE;
	//smb328a_charger_function_conrol(client);
	//smb328a_print_all_regs(client);
#else
    g_chip = chip ;
#endif

	return 0;

err_kfree:
	kfree(chip);
	return ret;
}

static int __devexit smb328a_remove(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy_bat);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int smb328a_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	return 0;
}

static int smb328a_resume(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	return 0;
}
#else
#define smb328a_suspend NULL
#define smb328a_resume NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id smb328a_id[] = {
	{ "smb328a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb328a_id);

static struct i2c_driver smb328a_i2c_driver = {
	.driver	= {
		.name	= "smb328a",
	},
	.probe		= smb328a_probe,
	.remove		= __devexit_p(smb328a_remove),
	.suspend	= smb328a_suspend,
	.resume		= smb328a_resume,
	.id_table	= smb328a_id,
};

static int __init smb328a_init(void)
{
   printk(KERN_ERR "[SMB328a][%s]\n", __func__);
#if defined(CONFIG_MACH_ESCAPE)
    if (hw_version < 2)
        return -ENODEV ;
#elif defined(CONFIG_MACH_GIO)
    if (hw_version < 3)
        return -ENODEV ;	
#endif
   return i2c_add_driver(&smb328a_i2c_driver);
}
module_init(smb328a_init);

static void __exit smb328a_exit(void)
{
    printk(KERN_ERR "[SMB328a][%s]\n", __func__);
#if defined(CONFIG_MACH_ESCAPE)
    if (hw_version < 2)
        return -ENODEV ;
#elif defined(CONFIG_MACH_GIO)
    if (hw_version < 3)
        return -ENODEV ;		
#endif
   	i2c_del_driver(&smb328a_i2c_driver);
}
module_exit(smb328a_exit);


MODULE_DESCRIPTION("SMB328A charger control driver");
MODULE_AUTHOR("<jongmyeong.ko@samsung.com>");
MODULE_LICENSE("GPL");
