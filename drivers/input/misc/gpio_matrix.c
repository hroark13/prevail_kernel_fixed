/* drivers/input/misc/gpio_matrix.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/input.h>

#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_ROOKIE)
unsigned int Volume_Up_irq = 0;
unsigned int Volume_Down_irq = 0;
#endif

struct gpio_kp {
	struct gpio_event_input_devs *input_devs;
	struct gpio_event_matrix_info *keypad_info;
	struct hrtimer timer;
	struct wake_lock wake_lock;
	int current_output;
	unsigned int use_irq:1;
	unsigned int key_state_changed:1;
	unsigned int last_key_state_changed:1;
	unsigned int some_keys_pressed:2;
	unsigned int disabled_irq:1;
	unsigned long keys_pressed[0];
};

static struct gpio_kp *pgpio_key;

extern unsigned char hw_version;

#if defined(CONFIG_MACH_ROOKIE)
#define GPIO_KEY_BL_CTRL  81
// MB ysahn 2011.04.17 - Main Key Index
#define BACK_KEY_INDEX 1
#define HOME_KEY_INDEX 4
// MB ysahn 2011.04.17 - Main Key Index
#elif defined(CONFIG_MACH_ESCAPE)
#define GPIO_KEY_BL_CTRL  (hw_version >= 4 ? 84 : 81)
#define HALL_GPIO         (hw_version >= 3 ? 94 : 84)
#elif defined(CONFIG_MACH_GIO)
#define GPIO_KEY_BL_CTRL  81
#endif

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)  || defined(CONFIG_MACH_GIOS)
#include <linux/earlysuspend.h>
spinlock_t keypad_led_lock;

#if defined(CONFIG_MACH_RANT3)
#define DEFAULT_KEYPADBACKLIGHT_TIMEOUT  6
#else
#define DEFAULT_KEYPADBACKLIGHT_TIMEOUT  3
#endif

/* Keypad Backlight time */
unsigned int backlight_time = DEFAULT_KEYPADBACKLIGHT_TIMEOUT ;
static /*short*/ int  backlight_flag  ;
static /*short*/ int is_suspend = 0;
static int touch_press = 0;
/* sysfs class registration */
struct class *bl_class;
/* sysfs device registration */
// /sys/devices/virtual/keypad-backlight/backlight/keypad_bl_time
struct device *kpd_dev;

struct early_suspend    early_suspend;
static int keypad_backlight_power(int mainkey);

static ssize_t keyshort_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	if(pgpio_key->some_keys_pressed) {
		count = sprintf(buf, "PRESS\n");
	}
	else {
		count = sprintf(buf, "RELEASE\n");
	}

	return count;
}

static DEVICE_ATTR(key_pressed, S_IRUGO | S_IWUSR | S_IWGRP, keyshort_test, NULL);

static ssize_t keypad_backlight_ctrl_onoff_store(struct device *dev, struct device_attribute *attr, char *buf)
{
    int onoff;

    sscanf(buf,"%d\n",&onoff);

    keypad_backlight_power(onoff);

    return sprintf(buf,"Backlight control set to : %s\n",onoff==1?"ON":"OFF");
}

static DEVICE_ATTR(backlight_ctrl, S_IRUGO | S_IWUSR | S_IWGRP, NULL, keypad_backlight_ctrl_onoff_store);

static ssize_t keypad_backlight_lpm_ctrl_onoff_store(struct device *dev, struct device_attribute *attr, char *buf)
{
    sscanf(buf,"%d\n",&backlight_flag);
    return sprintf(buf,"Backlight low power control set to : %d\n",backlight_flag);
}

ssize_t keypad_backlight_lpm_ctrl_onoff_show(struct device *dev,struct device_attribute *attr,char *buf)
{
    sprintf(buf,"Current backlight low power control : %d\n",backlight_flag);
    return sprintf(buf,"%s",buf);
}

static DEVICE_ATTR(backlight_lpm_ctrl, S_IRUGO |S_IWUSR | S_IWGRP, keypad_backlight_lpm_ctrl_onoff_show, keypad_backlight_lpm_ctrl_onoff_store);

ssize_t keypad_backlight_time_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{

 unsigned int bl_time;
 sscanf(buf,"%d\n",&bl_time);
 printk(" Backlight time set to : %d : seconds \n",bl_time);
 backlight_time = bl_time;
 return sprintf(buf,"Backlight time set to : %d : seconds \n",bl_time);

}

ssize_t keypad_backlight_time_show(struct device *dev,struct device_attribute *attr,char *buf)
{

 unsigned int bl_time = backlight_time;
 sprintf(buf,"Current backlight time : %d : seconds\n",bl_time);
 return sprintf(buf,"%s",buf);

}

static struct device_attribute bl_keypad_attributes[] = {
        __ATTR(keypad_bl_time, 0664, keypad_backlight_time_show, keypad_backlight_time_store),
};
#endif

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
#include <mach/vreg.h>
#include <linux/workqueue.h>

#define ON        1
#define OFF       0

struct delayed_work backlightoff;

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
static short int  backlight_onoff_state = OFF;
#endif
static int keypad_backlight_power(int mainkey)
{
#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
  struct vreg *vreg_backlight;  
  int ret;

  /* control VDD_MAINKEY_3.3V */
  if (hw_version >= 2) /* R730_HW_REV02  ldo16 <--> ldo18 */
     vreg_backlight = vreg_get(NULL, "ldo18");
  else
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
  vreg_backlight = vreg_get(NULL, "ldo18");
#else
  vreg_backlight = vreg_get(NULL, "ldo16");
#endif
  
  if(IS_ERR(vreg_backlight)) {
    printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__,PTR_ERR(vreg_backlight));
    return PTR_ERR(vreg_backlight);
  }  
  
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
  if( backlight_onoff_state == mainkey) {
    return 0;
  }
#endif
  if(mainkey == ON
	#if defined(CONFIG_MACH_ESCAPE)
	&& touch_press
	#endif
	) {
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3)|| defined(CONFIG_MACH_GIOS)
    ret = vreg_set_level(vreg_backlight, OUT3300mV);
#else
    if (hw_version >= 2) /* R730_HW_REV02  ldo16 <--> ldo18 */
        ret = vreg_set_level(vreg_backlight, OUT3300mV);
    else
        ret = vreg_set_level(vreg_backlight, OUT3000mV);
#endif
    if(ret) {
      printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__,ret);
      return -EIO;
    }
    
    ret = vreg_backlight_enable(vreg_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  }
  else 
  { 
#if defined(CONFIG_MACH_ESCAPE)
	touch_press = false;
#endif
    ret = vreg_backlight_disable(vreg_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  }
#endif
#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
  if(mainkey == ON
#if defined(CONFIG_MACH_ESCAPE)
	&& !touch_press
#endif
  ) {
  	#if defined(CONFIG_MACH_ESCAPE)
    if(gpio_get_value(HALL_GPIO) && !touch_press)
	#endif
      gpio_set_value(GPIO_KEY_BL_CTRL, 1);
  } else { 
    gpio_set_value(GPIO_KEY_BL_CTRL, 0);
  }
#endif

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
  backlight_onoff_state = mainkey;
#endif
  return 0;
}

/* Bottom half for switching off the keypad backlight in case intervention is not there
   for the delayed period */ 
static void keypad_backlightoff_work(struct work_struct *work)
{
//    printk("[%s]\n", __FUNCTION__);
    keypad_backlight_power(OFF);
}

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
int sleep_volumnkey_awake = OFF;
EXPORT_SYMBOL(sleep_volumnkey_awake);

static int sleep_volumnkey_awake_state = OFF;
static ssize_t keypad_volumnkey_store(struct device *dev, struct device_attribute *attr, char *buf)
{
  sscanf(buf,"%d\n",&sleep_volumnkey_awake);
  return sprintf(buf,"%d\n",sleep_volumnkey_awake);
}

static ssize_t keypad_volumnkey_show(struct device *dev,struct device_attribute *attr,char *buf)
{
  sprintf(buf," %d\n",sleep_volumnkey_awake);
  return sprintf(buf,"%s\n",buf);
}

static DEVICE_ATTR(volumnkey_ctrl, S_IRUGO | S_IWUSR | S_IWGRP, keypad_volumnkey_show, keypad_volumnkey_store);

#define VOLUMN_UP_KEYSENSE    41
#define VOLUMN_DN_KEYSENSE    40

static void keypad_volumnkey_wake_ctrl(void)
{
	int err;

  if( sleep_volumnkey_awake == OFF)
  {
    if( sleep_volumnkey_awake_state == ON )
    {
      err = set_irq_wake(gpio_to_irq(VOLUMN_UP_KEYSENSE), OFF);
      if (err) {
        pr_err("volumnkey_wake: set_irq_wake failed for input %d, "
            "err %d\n", VOLUMN_UP_KEYSENSE, err);
      }

      err = set_irq_wake(gpio_to_irq(VOLUMN_DN_KEYSENSE), OFF);
      if (err) {
        pr_err("volumnkey_wake: set_irq_wake failed for input %d, "
            "err %d\n", VOLUMN_UP_KEYSENSE, err);
      }

      sleep_volumnkey_awake_state = OFF;
    }
  }
  else
  {
    if( sleep_volumnkey_awake_state == OFF )
    {
      err = set_irq_wake(gpio_to_irq(VOLUMN_UP_KEYSENSE), ON);
      if (err) {
        pr_err("volumnkey_wake: set_irq_wake failed for input %d, "
            "err %d\n", VOLUMN_UP_KEYSENSE, err);
      }

      err = set_irq_wake(gpio_to_irq(VOLUMN_DN_KEYSENSE), ON);
      if (err) {
        pr_err("volumnkey_wake: set_irq_wake failed for input %d, "
            "err %d\n", VOLUMN_UP_KEYSENSE, err);
      }

      sleep_volumnkey_awake_state = ON;
    }
  }
}

void keypad_tsp_touch_event(void)
{
    unsigned long flags;
    
    if(backlight_flag && is_suspend == 0)
    {
      spin_lock_irqsave(&keypad_led_lock, flags);
  
      cancel_delayed_work_sync(&backlightoff);
      keypad_backlight_power(ON);
      schedule_delayed_work(&backlightoff, backlight_time * HZ);

      spin_unlock_irqrestore(&keypad_led_lock, flags);      
    }
}
EXPORT_SYMBOL(keypad_tsp_touch_event);

void keypad_early_suspend(struct early_suspend *h)
{
    printk("[%s]\n", __FUNCTION__);
    keypad_volumnkey_wake_ctrl();
    keypad_backlight_power(OFF) ;
    is_suspend = 1;
}

void keypad_late_resume(struct early_suspend *h)
{
    printk("[%s]\n", __FUNCTION__);
    is_suspend = 0;
    if(backlight_flag)
    {
        keypad_backlight_power(ON);
        schedule_delayed_work(&backlightoff, backlight_time * HZ);
    }
}
#elif defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2)
void keypad_early_suspend(struct early_suspend *h)
{
    printk("[%s]\n", __FUNCTION__);
    keypad_backlight_power(OFF) ;
    is_suspend = 1;
}

void keypad_late_resume(struct early_suspend *h)
{
    printk("[%s]\n", __FUNCTION__);
    is_suspend = 0;
    if(backlight_flag)
    {
        keypad_backlight_power(ON);
        schedule_delayed_work(&backlightoff, backlight_time * HZ);
    }
}
#else
void keypad_early_suspend(struct early_suspend *h)
{
//    printk("[%s]\n", __FUNCTION__);      
    keypad_backlight_power(OFF) ;
    backlight_flag = 0 ;
}

void keypad_late_resume(struct early_suspend *h)
{
//    printk("[%s]\n", __FUNCTION__);
    keypad_backlight_power(ON);
    backlight_flag = 1 ;
    schedule_delayed_work(&backlightoff, backlight_time * HZ);
}
#endif
#endif

static void clear_phantom_key(struct gpio_kp *kp, int out, int in)
{
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	int key_index = out * mi->ninputs + in;
	unsigned short keyentry = mi->keymap[key_index];
	unsigned short keycode = keyentry & MATRIX_KEY_MASK;
	unsigned short dev = keyentry >> MATRIX_CODE_BITS;

	if (!test_bit(keycode, kp->input_devs->dev[dev]->key)) {
		if (mi->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			pr_debug("gpiomatrix: phantom key %x, %d-%d (%d-%d) "
				"cleared\n", keycode, out, in,
				mi->output_gpios[out], mi->input_gpios[in]);
		__clear_bit(key_index, kp->keys_pressed);
	} else {
		if (mi->flags & GPIOKPF_PRINT_PHANTOM_KEYS)
			pr_debug("gpiomatrix: phantom key %x, %d-%d (%d-%d) "
				"not cleared\n", keycode, out, in,
				mi->output_gpios[out], mi->input_gpios[in]);
	}
}

static int restore_keys_for_input(struct gpio_kp *kp, int out, int in)
{
	int rv = 0;
	int key_index;

	key_index = out * kp->keypad_info->ninputs + in;
	while (out < kp->keypad_info->noutputs) {
		if (test_bit(key_index, kp->keys_pressed)) {
			rv = 1;
			clear_phantom_key(kp, out, in);
		}
		key_index += kp->keypad_info->ninputs;
		out++;
	}
	return rv;
}

static void remove_phantom_keys(struct gpio_kp *kp)
{
	int out, in, inp;
	int key_index;

	if (kp->some_keys_pressed < 3)
		return;

	for (out = 0; out < kp->keypad_info->noutputs; out++) {
		inp = -1;
		key_index = out * kp->keypad_info->ninputs;
		for (in = 0; in < kp->keypad_info->ninputs; in++, key_index++) {
			if (test_bit(key_index, kp->keys_pressed)) {
				if (inp == -1) {
					inp = in;
					continue;
				}
				if (inp >= 0) {
					if (!restore_keys_for_input(kp, out + 1,
									inp))
						break;
					clear_phantom_key(kp, out, inp);
					inp = -2;
				}
				restore_keys_for_input(kp, out, in);
			}
		}
	}
}

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
#include "../../../arch/arm/mach-msm/smd_private.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_iomap-7xxx.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
static int lockup_capture_check(int keycode, u8 keypress)
{ 
	#define NUM_CAPTURE_KEYS 3
#if defined (CONFIG_MACH_ESCAPE)	//sdk_ed12 'F' + 'Vol_Down' + 'Pwr'
	static int capture_keys[NUM_CAPTURE_KEYS]={33, 114, 116}; 
#elif defined(CONFIG_MACH_GIO)
	static int capture_keys[NUM_CAPTURE_KEYS]={102, 115, 116}; //cha Home + Volume down + Pwr Key
#elif defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	static int capture_keys[]={KEY_VOLUMEUP, KEY_CAMERA, KEY_POWER};
#elif defined(CONFIG_MACH_RANT3)
	static int capture_keys[]={KEY_VOLUMEDOWN, KEY_P, KEY_POWER};
#else
	static int capture_keys[NUM_CAPTURE_KEYS]={158, 115, 116}; //cha back + volme up + pwr key // MBjgnoh 11.02.14 Volume down + back + Pwr Key
#endif
	int i, capture_keys_size = 0;
	int capture_key_cnt = 0;

	capture_keys_size = ARRAY_SIZE(capture_keys);

	for(i=0;i<capture_keys_size;i++) {
		if(keycode == (capture_keys[i]&~0x80000000)) {
			if(keypress)
				capture_keys[i]|=0x80000000;
			else
				capture_keys[i]&=~0x80000000;

		}
		if(capture_keys[i]&0x80000000)
			capture_key_cnt++;
	}
#ifndef TARGET_BUILD_USER
	printk("[lockup_capture_check] capture_key_cnt:%d, capture_keys_size:%d \n", capture_key_cnt, capture_keys_size);
#endif

	if(capture_key_cnt == capture_keys_size) {
		// notify to ARM9 for ram dump
		writel(0xCCCC, MSM_SHARED_RAM_BASE + 0x30); 
		printk("[PANIC] LOCKUP CAPTURED!!! \n");
		msm_proc_comm_reset_modem_now();
	}
}

static int hw_reset_capture_check(u8 keycode, u8 keypress)
{
	static int capture_keys[]={KEY_VOLUMEDOWN, KEY_CAMERA, KEY_POWER};

	int i, capture_keys_size = 0;
	int capture_key_cnt = 0;

	capture_keys_size = ARRAY_SIZE(capture_keys);

	for(i=0;i<capture_keys_size;i++) {
		if(keycode == (capture_keys[i]&~0x80000000)) {
			if(keypress)
				capture_keys[i]|=0x80000000;
			else
				capture_keys[i]&=~0x80000000;

		}
		if(capture_keys[i]&0x80000000)
			capture_key_cnt++;
	}

	if(capture_key_cnt == capture_keys_size) {
		msm_proc_comm_modem_normal_reset();
	}

}
#else
#define lockup_capture_check(keycode, keypress)
#define hw_reset_capture_check(keycode, keypress)
#endif

static void report_key(struct gpio_kp *kp, int key_index, int out, int in)
{
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	int pressed = test_bit(key_index, kp->keys_pressed);
	unsigned short keyentry = mi->keymap[key_index];
	unsigned short keycode = keyentry & MATRIX_KEY_MASK;
	unsigned short dev = keyentry >> MATRIX_CODE_BITS;

	if (pressed != test_bit(keycode, kp->input_devs->dev[dev]->key)) {
		if (keycode == KEY_RESERVED) {
			//if (mi->flags & GPIOKPF_PRINT_UNMAPPED_KEYS)
			//	pr_info("gpiomatrix: unmapped key, %d-%d "
			//		"(%d-%d) changed to %d\n",
			//		out, in, mi->output_gpios[out],
			//		mi->input_gpios[in], pressed);
		} else {
			//if (mi->flags & GPIOKPF_PRINT_MAPPED_KEYS)
			//	pr_info("gpiomatrix: key %x, %d-%d (%d-%d) "
			//		"changed to %d\n", keycode,
			//		out, in, mi->output_gpios[out],
			//		mi->input_gpios[in], pressed);
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
			if( hw_version == 5 && !backlight_flag && keycode == KEY_CAMERA )
			{
				keycode = KEY_POWER;
			}
#endif
			input_report_key(kp->input_devs->dev[dev], keycode, pressed);
#ifndef TARGET_BUILD_USER
			printk("key event (keycode:%d, pressed:%d)\n", keycode, pressed);
#endif


#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIO)
			lockup_capture_check(keycode, pressed);
#elif defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_GIOS)/* requested by s/w pl */
#ifndef TARGET_BUILD_USER
			lockup_capture_check(keycode, pressed);
#endif
#endif

#if 0//defined(CONFIG_MACH_RANT3)
			hw_reset_capture_check(keycode, pressed);
#endif
		}
	}
}

static enum hrtimer_restart gpio_keypad_timer_func(struct hrtimer *timer)
{
	int out, in;
	int key_index;
	int gpio;
	struct gpio_kp *kp = container_of(timer, struct gpio_kp, timer);
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	unsigned gpio_keypad_flags = mi->flags;
	unsigned polarity = !!(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH);

	out = kp->current_output;
	if (out == mi->noutputs) {
		out = 0;
		kp->last_key_state_changed = kp->key_state_changed;
		kp->key_state_changed = 0;
		kp->some_keys_pressed = 0;
	} 
	else {
		key_index = out * mi->ninputs;
#if defined (CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
		for (in = 0; in < mi->ninputs-2; in++, key_index++) {
			gpio = mi->input_gpios[in];
			if (gpio_get_value(gpio) ^ !polarity) {
				if (kp->some_keys_pressed < 3)
					kp->some_keys_pressed++;
#if defined (CONFIG_MACH_ROOKIE) //MB ysahn 2011.04.17 - Main Key Overlap Handling
				if(key_index == BACK_KEY_INDEX && test_bit(HOME_KEY_INDEX,kp->keys_pressed))
				{
					kp->key_state_changed |= __test_and_set_bit(HOME_KEY_INDEX, kp->keys_pressed);
				}
				else if(key_index == HOME_KEY_INDEX && test_bit(BACK_KEY_INDEX,kp->keys_pressed))
				{
					kp->key_state_changed |= __test_and_set_bit(BACK_KEY_INDEX, kp->keys_pressed);
				}
				else
#endif //MB ysahn 2011.04.17 - Main Key Overlap Handling
				{
					kp->key_state_changed |= !__test_and_set_bit(key_index, kp->keys_pressed);
					//printk("kp->keys_pressed[0] : %d\n",in,kp->keys_pressed[0]);
	 				//printk("key event (key gpio:%d pressed, !polarity : %d)\n", gpio, !polarity);
				}
			} else
				kp->key_state_changed |= __test_and_clear_bit(key_index, kp->keys_pressed);
		}
		gpio = mi->output_gpios[out];

		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
		{
			gpio_set_value(gpio, !polarity);
		}
		else
			gpio_direction_input(gpio);

		if (out == 0)  // for volume up/down key
		{
            static unsigned int     volume_pressed = 0 ;
			for (in ; in < mi->ninputs; in++, key_index++) {
				gpio = mi->input_gpios[in];
				if (gpio_get_value(gpio) == 0) { /* pressed */
                    if (volume_pressed == (1 << in) || !volume_pressed)    /* only this key */
                    {
    					if (kp->some_keys_pressed < 3)
    						kp->some_keys_pressed++;
    					kp->key_state_changed |= !__test_and_set_bit(key_index, kp->keys_pressed);
                        volume_pressed |= (1 << in) ;
    					//printk("key event (key gpio:%d, pressed)\n", gpio);
                    }
				} else
				{
					kp->key_state_changed |= __test_and_clear_bit(key_index, kp->keys_pressed);
                    volume_pressed &= ~(1 << in) ;
				}
			}
		}
#else
		for (in = 0; in < mi->ninputs; in++, key_index++) {
			gpio = mi->input_gpios[in];
			if (gpio_get_value(gpio) ^ !polarity) {
				if (kp->some_keys_pressed < 3)
					kp->some_keys_pressed++;
				kp->key_state_changed |= !__test_and_set_bit(
						key_index, kp->keys_pressed);
 				//printk("key event (key gpio:%d pressed, polarity : %d)\n", gpio, polarity);
			} else
				kp->key_state_changed |= __test_and_clear_bit(
						key_index, kp->keys_pressed);
		}
		gpio = mi->output_gpios[out];
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(gpio, !polarity);
		else
			gpio_direction_input(gpio);
#endif
		out++;
	}
	kp->current_output = out;

	if (out < mi->noutputs) {
		gpio = mi->output_gpios[out];
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(gpio, polarity);
		else
			gpio_direction_output(gpio, polarity);
		hrtimer_start(timer, mi->settle_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
	}
	if (gpio_keypad_flags & GPIOKPF_DEBOUNCE) {
		if (kp->key_state_changed) {
			hrtimer_start(&kp->timer, mi->debounce_delay,
				      HRTIMER_MODE_REL);
			return HRTIMER_NORESTART;
		}
		kp->key_state_changed = kp->last_key_state_changed;
	}
	if (kp->key_state_changed) {
		if (gpio_keypad_flags & GPIOKPF_REMOVE_SOME_PHANTOM_KEYS)
			remove_phantom_keys(kp);
		key_index = 0;
		for (out = 0; out < mi->noutputs; out++)
			for (in = 0; in < mi->ninputs; in++, key_index++)
			{
				report_key(kp, key_index, out, in);
			}
	}
	if (!kp->use_irq || kp->some_keys_pressed) {
		hrtimer_start(timer, mi->poll_time, HRTIMER_MODE_REL);
		return HRTIMER_NORESTART;
	}

	/* No keys are pressed, reenable interrupt */
	for (out = 0; out < mi->noutputs; out++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(mi->output_gpios[out], polarity);
		else
			gpio_direction_output(mi->output_gpios[out], polarity);
	}
	for (in = 0; in < mi->ninputs; in++)
		enable_irq(gpio_to_irq(mi->input_gpios[in]));
	wake_unlock(&kp->wake_lock);
	return HRTIMER_NORESTART;
}

#if defined(CONFIG_MACH_ESCAPE)
static irqreturn_t slide_int_handler(int irq_in, void *dev_id)
{
    struct gpio_kp *kp = dev_id;
    int gpio_hall_ic = gpio_get_value(HALL_GPIO);

    if (!kp->use_irq) /* ignore interrupt while registering the handler */
        return IRQ_HANDLED;
		
    input_report_switch(kp->input_devs->dev[0], SW_LID, !gpio_hall_ic);
    input_sync(kp->input_devs->dev[0]);

    if(backlight_flag)
    {
		touch_press = false;
        cancel_delayed_work_sync(&backlightoff);
        if (gpio_hall_ic)
            keypad_backlight_power(ON);
    }
      
    printk("[%s] slide_int - !gpio_hall_ic %d\n",__FUNCTION__, gpio_hall_ic);

    if(backlight_flag)
        schedule_delayed_work(&backlightoff, backlight_time * HZ);

    return IRQ_HANDLED;
}

#endif

#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
void keypad_backlight_control(void)
{
    if(backlight_flag)
    {
        cancel_delayed_work_sync(&backlightoff);
	#if defined(CONFIG_MACH_ESCAPE)	
		touch_press = true;
	#endif
        keypad_backlight_power(ON);
        schedule_delayed_work(&backlightoff, backlight_time * HZ);
    }
}
#endif

static irqreturn_t gpio_keypad_irq_handler(int irq_in, void *dev_id)
{
	int i;
	struct gpio_kp *kp = dev_id;
	struct gpio_event_matrix_info *mi = kp->keypad_info;
	unsigned gpio_keypad_flags = mi->flags;
	unsigned long flags;

	if (!kp->use_irq) {
		/* ignore interrupt while registering the handler */
		kp->disabled_irq = 1;
		disable_irq_nosync(irq_in);
		return IRQ_HANDLED;
	}

	printk(" %s \r\n",__FUNCTION__);

	for (i = 0; i < mi->ninputs; i++)
		disable_irq_nosync(gpio_to_irq(mi->input_gpios[i]));
	for (i = 0; i < mi->noutputs; i++) {
		if (gpio_keypad_flags & GPIOKPF_DRIVE_INACTIVE)
			gpio_set_value(mi->output_gpios[i],
				!(gpio_keypad_flags & GPIOKPF_ACTIVE_HIGH));
		else
			gpio_direction_input(mi->output_gpios[i]);
	}
	wake_lock(&kp->wake_lock);
	hrtimer_start(&kp->timer, ktime_set(0, 10000), HRTIMER_MODE_REL);
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
      if(backlight_flag && is_suspend == 0)
#else
      if(backlight_flag)
#endif
      {
      #if defined(CONFIG_MACH_ESCAPE)
      	  touch_press = false;
	  #endif
          spin_lock_irqsave(&keypad_led_lock, flags);
	  
          cancel_delayed_work_sync(&backlightoff);
          keypad_backlight_power(ON);
          schedule_delayed_work(&backlightoff, backlight_time * HZ);

          spin_unlock_irqrestore(&keypad_led_lock, flags);
      }
#endif      
	return IRQ_HANDLED;
}

static int gpio_keypad_request_irqs(struct gpio_kp *kp)
{
	int i;
	int err;
	unsigned int irq;
	unsigned long request_flags;
	struct gpio_event_matrix_info *mi = kp->keypad_info;

	switch (mi->flags & (GPIOKPF_ACTIVE_HIGH|GPIOKPF_LEVEL_TRIGGERED_IRQ)) {
	default:
		request_flags = IRQF_TRIGGER_FALLING;
		break;
	case GPIOKPF_ACTIVE_HIGH:
		request_flags = IRQF_TRIGGER_RISING;
		break;
	case GPIOKPF_LEVEL_TRIGGERED_IRQ:
		request_flags = IRQF_TRIGGER_LOW;
		break;
	case GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_ACTIVE_HIGH:
		request_flags = IRQF_TRIGGER_HIGH;
		break;
	}

	for (i = 0; i < mi->ninputs; i++) {
		err = irq = gpio_to_irq(mi->input_gpios[i]);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
#if defined(CONFIG_MACH_ESCAPE)
	if(hw_version >= 3)
	{

		switch(mi->input_gpios[i])
		{
			case 78:
				Volume_Up_irq = irq;
				break;
				
			case 91:
				Volume_Down_irq = irq;
				break;
		}
	 }else{
		 switch(mi->input_gpios[i])
		 {
			 case 36:
				 Volume_Up_irq = irq;
				 break;
				 
			 case 91:
				 Volume_Down_irq = irq;
				 break;
		 }
	 }
#elif defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_ROOKIE)
		switch(mi->input_gpios[i])
		{
			case 36:
				Volume_Up_irq = irq;
				break;
				
			case 91:
				Volume_Down_irq = irq;
				break;
		}

#endif	

#if defined (CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
		if ( i >= mi->ninputs-2) // ( (i == 2) || (i == 3))
			err = request_irq(irq, gpio_keypad_irq_handler, IRQF_TRIGGER_FALLING, "gpio_kp", kp);
		else
#endif
		err = request_irq(irq, gpio_keypad_irq_handler, request_flags, "gpio_kp", kp);
		if (err) {
			pr_err("gpiomatrix: request_irq failed for input %d, "
				"irq %d\n", mi->input_gpios[i], irq);
			goto err_request_irq_failed;
		}
#if defined(CONFIG_MACH_RANT3)		
		if(hw_version >= 6)
		{
        		if(mi->input_gpios[i] == 36)
        		{
                		err = set_irq_wake(irq, 1);
                		if (err) {
                			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
                				"irq %d\n", mi->input_gpios[i], irq);
							goto err_set_irq_wake_failed;
                		}
        		}
		}
		else if(hw_version >= 2 && hw_version < 6)
		{
        		if(mi->input_gpios[i] == 122)
        		{
                		err = set_irq_wake(irq, 1);
                		if (err) {
                			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
                				"irq %d\n", mi->input_gpios[i], irq);
							goto err_set_irq_wake_failed;
                		}
        		}
		}
		else
		{
        		err = set_irq_wake(irq, 1);
        		if (err) {
        			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
        				"irq %d\n", mi->input_gpios[i], irq);
					goto err_set_irq_wake_failed;
        		}
		}
#elif defined(CONFIG_MACH_VINO)
		if(hw_version >= 3)
		{
        		if(mi->input_gpios[i] == 42)
        		{
                		err = set_irq_wake(irq, 1);
                		if (err) {
                			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
                				"irq %d\n", mi->input_gpios[i], irq);
							goto err_set_irq_wake_failed;
                		}
        		}
		}
		else
		{
        		err = set_irq_wake(irq, 1);
        		if (err) {
        			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
        				"irq %d\n", mi->input_gpios[i], irq);
					goto err_set_irq_wake_failed;
        		}
		}
#elif defined(CONFIG_MACH_GIOS)
		if(hw_version >= 3)
		{
        		if(mi->input_gpios[i] == 42)
        		{
                		err = set_irq_wake(irq, 1);
                		if (err) {
                			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
                				"irq %d\n", mi->input_gpios[i], irq);
			goto err_set_irq_wake_failed;
                		}
        		}
		}
		else
		{
        		err = set_irq_wake(irq, 1);
        		if (err) {
        			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
        				"irq %d\n", mi->input_gpios[i], irq);
			goto err_set_irq_wake_failed;
        		}
		}		
#else
		err = set_irq_wake(irq, 1);
		if (err) {
			pr_err("gpiomatrix: set_irq_wake failed for input %d, "
				"irq %d\n", mi->input_gpios[i], irq);
			goto err_set_irq_wake_failed;
		}
#endif		
		disable_irq(irq);
		if (kp->disabled_irq) {
			kp->disabled_irq = 0;
			enable_irq(irq);
		}
	}
#if defined(CONFIG_MACH_ESCAPE)
	gpio_tlmm_config(GPIO_CFG(HALL_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE) ;
	err = request_irq(gpio_to_irq(HALL_GPIO), slide_int_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_DISABLED,
        		    "Slide", (void*)kp);

	if (err) {
		pr_err("gpiomatrix: set_slide_irq failed\n");
		goto err_request_slide_irq_failed;
	}

//-- wake slide irg
    err = set_irq_wake(gpio_to_irq(HALL_GPIO), 1);
    if (err) {
        pr_err("gpiomatrix: set_irq_wake failed\n");
    }
#endif
	return 0;

#if defined(CONFIG_MACH_ESCAPE)
err_request_slide_irq_failed:
#endif
err_set_irq_wake_failed:
	for (i = mi->noutputs - 1; i >= 0; i--) {
	free_irq(gpio_to_irq(mi->input_gpios[i]), kp);
	}
err_request_irq_failed:
err_gpio_get_irq_num_failed:	
	return err;
}

int gpio_event_matrix_func(struct gpio_event_input_devs *input_devs,
	struct gpio_event_info *info, void **data, int func)
{
	int i;
	int err;
	int key_count;
	struct gpio_kp *kp;
	struct gpio_event_matrix_info *mi;

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
	/* Implementation for sysfs entries , creating Keypad-backlight class */
	bl_class = class_create(THIS_MODULE, "keypad-backlight");
	if ( bl_class) {
		printk(KERN_WARNING "Unable to create Keypad Backlight class \n");
	}

	/* Creating the device backlight under keypad-backlight class */
   	kpd_dev = device_create( bl_class, NULL, 0, NULL, "backlight");
   	if (!kpd_dev){
		class_destroy(bl_class);
     		printk("Failed to create device(keypad backlight) \n");
	}

       if (device_create_file(kpd_dev, &dev_attr_key_pressed) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_key_pressed.attr.name);

       if (device_create_file(kpd_dev, &dev_attr_backlight_ctrl) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_backlight_ctrl.attr.name);

       if (device_create_file(kpd_dev, &dev_attr_backlight_lpm_ctrl) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_backlight_lpm_ctrl.attr.name);

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
       if (device_create_file(kpd_dev, &dev_attr_volumnkey_ctrl) < 0)
  		printk("Failed to create device file(%s)!\n", dev_attr_volumnkey_ctrl.attr.name);

#endif
	/* Creating device attributes for setting keypad backlight time */
  	if (device_create_file(kpd_dev, &bl_keypad_attributes[0]) < 0){
   		printk("Failed to create device attributes for Keypad Backlight \n");
		device_destroy(bl_class,0);
		class_destroy(bl_class);
	}

      /* Initializing delayed work for Back light off */
      INIT_DELAYED_WORK(&backlightoff, keypad_backlightoff_work);	
#endif      
	mi = container_of(info, struct gpio_event_matrix_info, info);
	if (func == GPIO_EVENT_FUNC_SUSPEND || func == GPIO_EVENT_FUNC_RESUME) {
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
  if(func == GPIO_EVENT_FUNC_SUSPEND) 
    keypad_backlight_power(OFF);
  else
    keypad_backlight_power(ON);
#endif
		/* TODO: disable scanning */
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
		if (mi->keymap == NULL ||
		   mi->input_gpios == NULL ||
		   mi->output_gpios == NULL) {
			err = -ENODEV;
			pr_err("gpiomatrix: Incomplete pdata\n");
			goto err_invalid_platform_data;
		}
		key_count = mi->ninputs * mi->noutputs;

		pgpio_key=*data = kp = kzalloc(sizeof(*kp) + sizeof(kp->keys_pressed[0]) *
				     BITS_TO_LONGS(key_count), GFP_KERNEL);
		if (kp == NULL) {
			err = -ENOMEM;
			pr_err("gpiomatrix: Failed to allocate private data\n");
			goto err_kp_alloc_failed;
		}
		kp->input_devs = input_devs;
		kp->keypad_info = mi;
		for (i = 0; i < key_count; i++) {
			unsigned short keyentry = mi->keymap[i];
			unsigned short keycode = keyentry & MATRIX_KEY_MASK;
			unsigned short dev = keyentry >> MATRIX_CODE_BITS;
			if (dev >= input_devs->count) {
				pr_err("gpiomatrix: bad device index %d >= "
					"%d for key code %d\n",
					dev, input_devs->count, keycode);
				err = -EINVAL;
				goto err_bad_keymap;
			}
			if (keycode && keycode <= KEY_MAX)
				input_set_capability(input_devs->dev[dev],
							EV_KEY, keycode);
		}

#if defined(CONFIG_MACH_ESCAPE)
		input_set_capability(kp->input_devs->dev[0], EV_SW, SW_LID);

		if(gpio_get_value(HALL_GPIO))
			kp->input_devs->dev[0]->sw[SW_LID] = 0;
		else
			kp->input_devs->dev[0]->sw[SW_LID] = 1;
#endif

		for (i = 0; i < mi->noutputs; i++) {
			err = gpio_request(mi->output_gpios[i], "gpio_kp_out");
			if (err) {
				pr_err("gpiomatrix: gpio_request failed for "
					"output %d\n", mi->output_gpios[i]);
				goto err_request_output_gpio_failed;
			}
			if (gpio_cansleep(mi->output_gpios[i])) {
				pr_err("gpiomatrix: unsupported output gpio %d,"
					" can sleep\n", mi->output_gpios[i]);
				err = -EINVAL;
				goto err_output_gpio_configure_failed;
			}
			if (mi->flags & GPIOKPF_DRIVE_INACTIVE){
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_ESCAPE)
	#if defined(CONFIG_MACH_RANT3)
				if(hw_version < 6)
	#endif
    				{
        				err = gpio_tlmm_config(GPIO_CFG(mi->output_gpios[i] , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        				printk("Gpio config %d return %d \r\n",mi->output_gpios[i] ,err);
        				if (err) {
        				pr_err("gpiomatrix: gpio_tlmm_config failed for "
        					"output %d\n", mi->output_gpios[i]);
        				goto err_request_output_gpio_failed;
        				}
    				}
#endif

				err = gpio_direction_output(mi->output_gpios[i],
					!(mi->flags & GPIOKPF_ACTIVE_HIGH));
			}
			else
				err = gpio_direction_input(mi->output_gpios[i]);
			if (err) {
				pr_err("gpiomatrix: gpio_configure failed for "
					"output %d\n", mi->output_gpios[i]);
				goto err_output_gpio_configure_failed;
			}
		}
		for (i = 0; i < mi->ninputs; i++) {
			err = gpio_request(mi->input_gpios[i], "gpio_kp_in");
			if (err) {
				pr_err("gpiomatrix: gpio_request failed for "
					"input %d\n", mi->input_gpios[i]);
				goto err_request_input_gpio_failed;
			}
#if defined (CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE)// || defined(CONFIG_MACH_GIO)
			if ( i >= mi->ninputs-2) // ( (i == 2) || (i == 3))
				err = gpio_tlmm_config(GPIO_CFG(mi->input_gpios[i] , 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			else
#endif
#if defined(CONFIG_MACH_RANT3)
			if(hw_version < 6)
    			{
        			err = gpio_tlmm_config(GPIO_CFG(mi->input_gpios[i] , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        			pr_err("Gpio config %d return %d\r\n",mi->input_gpios[i] ,err);
        			if (err) {
        				printk("gpiomatrix: gpio_tlmm_config failed for "
        					"input %d\n", mi->input_gpios[i]);
        				goto err_request_input_gpio_failed;
        			}
    			}
#elif defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)

#else
			err = gpio_tlmm_config(GPIO_CFG(mi->input_gpios[i] , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        			pr_err("Gpio config %d return %d\r\n",mi->input_gpios[i] ,err);
			if (err) {
        				printk("gpiomatrix: gpio_tlmm_config failed for "
					"input %d\n", mi->input_gpios[i]);
				goto err_request_input_gpio_failed;
			}
#endif
			err = gpio_direction_input(mi->input_gpios[i]);
			if (err) {
				pr_err("gpiomatrix: gpio_direction_input failed"
					" for input %d\n", mi->input_gpios[i]);
				goto err_gpio_direction_input_failed;
			}
		}
		for (i = 0; i < mi->ninputs; i++) {
			printk("Reading Input GPIO %d at init is %d \r\n",mi->input_gpios[i],gpio_get_value(mi->input_gpios[i]));

		}
		
		kp->current_output = mi->noutputs;
		kp->key_state_changed = 1;

		hrtimer_init(&kp->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		kp->timer.function = gpio_keypad_timer_func;
		wake_lock_init(&kp->wake_lock, WAKE_LOCK_SUSPEND, "gpio_kp");
		err = gpio_keypad_request_irqs(kp);
		kp->use_irq = err == 0;

		pr_debug("GPIO Matrix Keypad Driver: Start keypad matrix for "
			"%s%s in %s mode\n", input_devs->dev[0]->name,
			(input_devs->count > 1) ? "..." : "",
			kp->use_irq ? "interrupt" : "polling");

		if (kp->use_irq)
			wake_lock(&kp->wake_lock);
    
		hrtimer_start(&kp->timer, ktime_set(0, 10000), HRTIMER_MODE_REL);
    
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_REALITY2) || defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
		early_suspend.suspend = keypad_early_suspend;
		early_suspend.resume = keypad_late_resume;
		register_early_suspend(&early_suspend);	
		backlight_flag = 1;
		keypad_backlight_power(OFF);
		spin_lock_init(&keypad_led_lock);
#endif

		return 0;
	}

	err = 0;
	kp = *data;

	if (kp->use_irq)
		for (i = mi->noutputs - 1; i >= 0; i--)
			free_irq(gpio_to_irq(mi->input_gpios[i]), kp);

	hrtimer_cancel(&kp->timer);
	wake_lock_destroy(&kp->wake_lock);
	for (i = mi->noutputs - 1; i >= 0; i--) {
err_gpio_direction_input_failed:
		gpio_free(mi->input_gpios[i]);
err_request_input_gpio_failed:
		;
	}
	for (i = mi->noutputs - 1; i >= 0; i--) {
err_output_gpio_configure_failed:
		gpio_free(mi->output_gpios[i]);
err_request_output_gpio_failed:
		;
	}
err_bad_keymap:
	kfree(kp);
err_kp_alloc_failed:
err_invalid_platform_data:
	return err;
}
