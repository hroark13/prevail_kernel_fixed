/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>
#include <mach/vreg.h>
#include <linux/earlysuspend.h>

#include <linux/i2c/mmc328x.h>

#define DEBUG					0

#define MMC328X_DELAY_TM		10	/* ms */
#define MMC328X_DELAY_RM		10	/* ms */
#define MMC328X_DELAY_MD		1	/* ms */
#define READMD			1

#define MMC328X_RETRY_COUNT	3
#define MMC328X_RESET_INTV	20

#define MMC328X_DEV_NAME	"mmc328X"
#define LDO_LCD			"ldo14" //Quattro LDIO setting : ldo3 -> ldo14

static u32 read_idx = 0;
static u32 read_md_idx = 0;
struct class *mag_class;

static struct i2c_client *this_client;

static int mmc328X_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int mmc328X_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MMC328X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC328X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MMC328X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

/*************************************************************************/
/*						BMA023 Sysfs						   */
/*************************************************************************/
static ssize_t mmc328X_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned char data[16] = {0};
  int vec[3] = {0};
  int count;
  int res = 0;

  /* Always turn on  No Boost : the external capacitor will be charged from VDD */
  /* magnetize the MR */
  // Write 0x30 to 0x07
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_RM | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
    return 0;
  }
  /* wait external capacitor charging done for next SET/RESET */
  msleep(MMC328X_DELAY_RM);

  /* Charge pump reset before TM command */
  // Write 0x50 to 0x07
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_RST | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
  }

  /* Take measurement */
  // Write 0x11 to 0x07
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_TM | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
  }

  /* wait TM done for coming data read */
  msleep(MMC328X_DELAY_TM);
  data[0] = MMC328X_REG_DATA;
  res = mmc328X_i2c_rx_data(data, 6);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_rx_data(data[%02x])", __FUNCTION__, __LINE__, res, data[0]);
    return 0;
  }

  vec[0] = data[1] << 8 | data[0];
  vec[1] = data[3] << 8 | data[2];
  vec[2] = data[5] << 8 | data[4];

  printk("x: %d,y: %d,z: %d\n", vec[0], vec[1], vec[2]);
  count = sprintf(buf,"%d,%d,%d\n", vec[0], vec[1], vec[2]);

  return count;
}

static ssize_t mmc328X_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  printk("input data --> %s\n", buf);

  return size;
}

static ssize_t mmc328X_power_on(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned char data[16] = {0};
  int count;
  int res = 0;

  /* Charge pump reset before TM command */
  // Write 0x50 to 0x07
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_RST | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
  }

  /* Take measurement */
  // Write 0x11 to 0x07
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_TM | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
  }

  /* wait TM done for coming data read */
  msleep(MMC328X_DELAY_TM);

#if DEBUG
  printk("[%s] result of i2c writing: %d\n", __func__, !(res < 0));
#endif
  count = sprintf(buf,"%d\n", !(res < 0));

  return count;
}
static DEVICE_ATTR(read_mag, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, mmc328X_fs_read, mmc328X_fs_write);
static DEVICE_ATTR(power_on, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, mmc328X_power_on, NULL);

static int mmc328X_open(struct inode *inode, struct file *file)
{
  return nonseekable_open(inode, file);
}

static int mmc328X_release(struct inode *inode, struct file *file)
{
  return 0;
}

static int mmc328X_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
  void __user *pa = (void __user *)arg;
  unsigned char data[16] = {0};
  int vec[3] = {0};
  int i = 0;
  int res = 0;

	switch (cmd) {
  case MMC328X_IOC_TM:
    /* Charge pump reset before TM command */
    // Write 0x50 to 0x07
    data[0] = MMC328X_REG_CTRL;
    data[1] = MMC328X_CTRL_RST | MMC328X_CTRL_NB;
    res = mmc328X_i2c_tx_data(data, 2);
    if(res < 0) {
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
      return -EFAULT;
    }

    /* Take measurement */
    // Write 0x11 to 0x07
    data[0] = MMC328X_REG_CTRL;
    data[1] = MMC328X_CTRL_TM | MMC328X_CTRL_NB;
    res = mmc328X_i2c_tx_data(data, 2);
    if(res < 0) {
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
      return -EFAULT;
    }
    /* wait TM done for coming data read */
    msleep(MMC328X_DELAY_TM);
    break;

  case MMC328X_IOC_RM:
    /* magnetize the MR */
    // Write 0x30 to 0x07
    data[0] = MMC328X_REG_CTRL;
    data[1] = MMC328X_CTRL_RM | MMC328X_CTRL_NB;
    res = mmc328X_i2c_tx_data(data, 2);
    if(res < 0) {
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
      return -EFAULT;
    }
    /* wait external capacitor charging done for next SET/RESET */
    msleep(MMC328X_DELAY_RM);
    break;

	case MMC328X_IOC_READ:
    data[0] = MMC328X_REG_DATA;
    res = mmc328X_i2c_rx_data(data, 6);
    if(res < 0) {
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_rx_data(data[%02x])", __FUNCTION__, __LINE__, res, data[0]);
      return -EFAULT;
    }

    vec[0] = data[1] << 8 | data[0];
    vec[1] = data[3] << 8 | data[2];
    vec[2] = data[5] << 8 | data[4];

#if DEBUG
    printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
#endif
    if (copy_to_user(pa, vec, sizeof(vec))) {
      return -EFAULT;
    }
    break;

  case MMC328X_IOC_READXYZ:
    /* do RM every MMC328X_RESET_INTV times read */
    	if (!(read_md_idx % MMC328X_RESET_INTV)) {
      data[0] = MMC328X_REG_CTRL;
      data[1] = MMC328X_CTRL_RM | MMC328X_CTRL_NB;
      res = mmc328X_i2c_tx_data(data, 2);
      if(res < 0) {
        /* RM - not check return value here, assume it always OK */
        printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
      }
      /* wait external capacitor charging done for next RM */
      msleep(MMC328X_DELAY_RM);
      read_md_idx = 0;
    }
    read_md_idx++;

    /* Charge pump reset before TM command */
    data[0] = MMC328X_REG_CTRL;
    data[1] = MMC328X_CTRL_RST | MMC328X_CTRL_NB;
    res = mmc328X_i2c_tx_data(data, 2);
    if(res < 0) {
      /* Reset - not check return value here, assume it always OK */
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
    }

    /* send TM cmd before read */
    data[0] = MMC328X_REG_CTRL;
    data[1] = MMC328X_CTRL_TM;
    mmc328X_i2c_tx_data(data, 2);
    if(res < 0) {
      /* TM - not check return value here, assume it always OK */
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
    }
    /* wait TM done for coming data read */
    msleep(MMC328X_DELAY_TM);

#if READMD
    while(1) {
      /* Read Measure Done */
      data[0] = MMC328X_REG_STATUS;
      res = mmc328X_i2c_rx_data(data, 1);
      if(res < 0) {
        printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_rx_data(data[%02x])", __FUNCTION__, __LINE__, res, data[0]);
        return -EFAULT;
      }
      if (data[0] & 0x01) break;
      msleep(MMC328X_DELAY_MD);
      i++;
      if (i > MMC328X_RETRY_COUNT) {
        printk(KERN_ERR "TM not work!!");
        return -EFAULT;
      }
    }
#endif /* READMD */
    /* read xyz raw data */
    read_idx++;
    data[0] = MMC328X_REG_DATA;
    res = mmc328X_i2c_rx_data(data, 6);
    if(res < 0) {
      printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_rx_data(data[%02x])", __FUNCTION__, __LINE__, res, data[0]);
      return -EFAULT;
    }

    vec[0] = data[1] << 8 | data[0];
    vec[1] = data[3] << 8 | data[2];
    vec[2] = data[5] << 8 | data[4];
#if DEBUG
    printk("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
#endif
    if (copy_to_user(pa, vec, sizeof(vec))) {
      return -EFAULT;
    }
    break;
  default:
    printk(KERN_ERR "cmd=0x%X\n", cmd);
    break;
  }

  return 0;
}

static ssize_t mmc328X_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  ssize_t ret = 0;

  sprintf(buf, "MMC328X");
  ret = strlen(buf) + 1;

  return ret;
}

static DEVICE_ATTR(mmc328X, S_IRUGO, mmc328X_show, NULL);

static struct file_operations mmc328X_fops = {
  .owner	= THIS_MODULE,
  .open	= mmc328X_open,
  .release	= mmc328X_release,
  .ioctl		= mmc328X_ioctl,
};

static struct miscdevice mmc328X_device = {
  .minor 	= MISC_DYNAMIC_MINOR,
  .name 	= MMC328X_DEV_NAME,
  .fops 	= &mmc328X_fops,
};

int mmc328X_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct vreg *mmc328X_vreg = vreg_get(0, LDO_LCD);
  unsigned char data[16] = {0};
  int res = 0;

  printk(KERN_ERR "[mmc328X_probe] +\n");

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    pr_err("%s: functionality check failed\n", __FUNCTION__);
    res = -ENODEV;
    goto out;
  }
  this_client = client;

  res = misc_register(&mmc328X_device);
  if (res) {
    pr_err("%s: mmc328X_device register failed\n", __FUNCTION__);
    goto out;
  }
  res = device_create_file(&client->dev, &dev_attr_mmc328X);
  if (res) {
    pr_err("%s: device_create_file failed\n", __FUNCTION__);
    goto out_deregister;
  }
  // Power On
  if(vreg_enable(mmc328X_vreg))
    printk(KERN_ERR "vreg_enable: ldo15 vreg operation failed\n");

  /* send ST cmd to mag sensor first of all */
  data[0] = MMC328X_REG_CTRL;
  data[1] = MMC328X_CTRL_RM | MMC328X_CTRL_NB;
  res = mmc328X_i2c_tx_data(data, 2);
  if(res < 0) {
    /* assume SET always success */
    printk(KERN_ERR "%s(%d) %d=mmc328X_i2c_tx_data(data[%02x,%02x])", __FUNCTION__, __LINE__, res, data[0], data[1]);
  }
  /* wait external capacitor charging done for next SET/RESET */
  msleep(MMC328X_DELAY_RM);

  printk(KERN_ERR "[mmc328X_probe] -\n");

  return 0;

out_deregister:
  misc_deregister(&mmc328X_device);
out:
  return res;
}

static int mmc328X_remove(struct i2c_client *client)
{
  device_remove_file(&client->dev, &dev_attr_mmc328X);
  misc_deregister(&mmc328X_device);

  return 0;
}

#ifdef CONFIG_PM
static int mmc328X_suspend(struct i2c_client *client, pm_message_t mesg)
{
  struct vreg *mmc328X_vreg = vreg_get(0, LDO_LCD);
  if(vreg_disable(mmc328X_vreg))
    printk(KERN_ERR "vreg_disable: ldo15 vreg operation failed\n");

#if DEBUG
  printk(KERN_INFO "[%s]\n",__FUNCTION__);
#endif

  return 0;
}

static int mmc328X_resume(struct i2c_client *client)
{
  struct vreg *mmc328X_vreg = vreg_get(0, LDO_LCD);
  if(vreg_enable(mmc328X_vreg))
    printk(KERN_ERR "vreg_enable: ldo15 vreg operation failed\n");

#if DEBUG
  printk(KERN_INFO "[%s]\n",__FUNCTION__);
#endif

  return 0;
}
#else
#define mmc328X_suspend NULL
#define mmc328X_resume NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void mmc328x_early_suspend(struct early_suspend *h)
{
  struct vreg *mmc328x_vreg = vreg_get(0, LDO_LCD);
  if(vreg_disable(mmc328x_vreg))
    printk(KERN_ERR "vreg_disable: ldo14 vreg operation failed\n");
}

void mmc328x_late_resume(struct early_suspend *h)
{
  struct vreg *mmc328x_vreg = vreg_get(0, LDO_LCD);
  if(vreg_enable(mmc328x_vreg))
    printk(KERN_ERR "vreg_enable: ldo14 vreg operation failed\n");
}

static struct early_suspend mmc328x_earlysuspend = {
  .suspend = mmc328x_early_suspend,
  .resume = mmc328x_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id mmc328X_id[] = {
  { MMC328X_I2C_NAME, 0 },
  { }
};

static struct i2c_driver mmc328X_driver = {
  .probe 		= mmc328X_probe,
  .remove 	= mmc328X_remove,
  .suspend	= mmc328X_suspend,
  .resume		= mmc328X_resume,
  .id_table	= mmc328X_id,
  .driver 	= {
    .owner	= THIS_MODULE,
    .name	= MMC328X_I2C_NAME,
  },
};


static int __init mmc328X_init(void)
{
  struct device *dev_t;
  pr_info("mmc328X driver: init\n");
  mag_class = class_create(THIS_MODULE, "magnetic");

  if (IS_ERR(mag_class))
    return PTR_ERR( mag_class );

  dev_t = device_create( mag_class, NULL, 0, "%s", "magnetic");

  if (device_create_file(dev_t, &dev_attr_read_mag) < 0)
    printk("Failed to create device file(%s)!\n", dev_attr_read_mag.attr.name);

  if (device_create_file(dev_t, &dev_attr_power_on) < 0)
    printk("Failed to create device file(%s)!\n", dev_attr_power_on.attr.name);

  if (IS_ERR(dev_t))
  {
    return PTR_ERR(dev_t);
  }
  /* Registration for Andriod PM */
#ifdef CONFIG_HAS_EARLYSUSPEND
  register_early_suspend(&mmc328x_earlysuspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

  return i2c_add_driver(&mmc328X_driver);
}

static void __exit mmc328X_exit(void)
{
  pr_info("mmc328X driver: exit\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
  unregister_early_suspend(&mmc328x_earlysuspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

  i2c_del_driver(&mmc328X_driver);
}

module_init(mmc328X_init);
module_exit(mmc328X_exit);

MODULE_AUTHOR("Robbie Cao<hjcao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MMC328X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");

