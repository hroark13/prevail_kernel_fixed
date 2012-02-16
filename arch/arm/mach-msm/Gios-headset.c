/*
 *  H2W device detection driver.
 *
 * Copyright (C) 2008 SAMSUNG Corporation.
 * Copyright (C) 2008 Google, Inc.
 *
 * Authors: 
 *  Laurence Chen <Laurence_Chen@htc.com>
 *  Nick Pelly <npelly@google.com>
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

/*  For detecting SAMSUNG 2 Wire devices, such as wired headset.

    Logically, the H2W driver is always present, and H2W state (hi->state)
    indicates what is currently plugged into the H2W interface.

    When the headset is plugged in, CABLE_IN1 is pulled low. When the headset
    button is pressed, CABLE_IN2 is pulled low. These two lines are shared with
    the TX and RX (respectively) of UART3 - used for serial debugging.

    This headset driver keeps the CPLD configured as UART3 for as long as
    possible, so that we can do serial FIQ debugging even when the kernel is
    locked and this driver no longer runs. So it only configures the CPLD to
    GPIO while the headset is plugged in, and for 10ms during detection work.

    Unfortunately we can't leave the CPLD as UART3 while a headset is plugged
    in, UART3 is pullup on TX but the headset is pull-down, causing a 55 mA
    drain on bigfoot.

    The headset detection work involves setting CPLD to GPIO, and then pulling
    CABLE_IN1 high with a stronger pullup than usual. A H2W headset will still
    pull this line low, whereas other attachments such as a serial console
    would get pulled up by this stronger pullup.

    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/h2w/state to be updated.

    Button presses are interpreted as input event (KEY_MEDIA). Button presses
    are ignored if the headset is plugged in, so the buttons on 11 pin -> 3.5mm
    jack adapters do not work until a headset is plugged into the adapter. This
    is to avoid serial RX traffic causing spurious button press events.

    We tend to check the status of CABLE_IN1 a few more times than strictly
    necessary during headset detection, to avoid spurious headset insertion
    events caused by serial debugger TX traffic.
*/

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <mach/pmic.h>

#include <linux/gpio.h>
#include <linux/slab.h>
#include <mach/hardware.h>

#define FEATURE_SENDEND_ENABLE 
#define JACK_ACTIVE_LOW
#define CONFIG_DEBUG_H2W


extern unsigned char hw_version;

#define GPIO_JACK_S_35 		((hw_version>2)?((hw_version>3)?28:36):(123))
#define GPIO_POPUP_SW_EN	108
#define GPIO_SEND_END 		((hw_version>2)?(37):(1))

#define USED_VEAR_LDO15  ((hw_version>=5)?(1):(0))
#define LDO_VEAR			"ldo15" 

#ifdef CONFIG_DEBUG_H2W
#define H2W_DBG(fmt, arg...) printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

static struct workqueue_struct *g_detection_work_queue;
static void detection_work(struct work_struct *work);
static DECLARE_WORK(g_detection_work, detection_work);

#if 1	// for DFMS test
static struct workqueue_struct *g_earbutton_work_queue;
static void earbutton_work(struct work_struct *work);
static DECLARE_WORK(g_earbutton_work, earbutton_work);
static unsigned int earbutton_pressed = 0;
#endif

#define BIT_HEADSET			(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)

enum {
	H2W_NO_DEVICE	= 0,
	H2W_SEC_HEADSET	= 1,
	H2W_NORMAL_HEADSET = 2, 
};

struct h2w_info {
	struct switch_dev sdev;
	struct switch_dev button_sdev;	// for DFMS test

	struct input_dev *input;
	struct mutex mutex_lock;

	atomic_t btn_state;
	int ignore_btn;

	unsigned int irq;
	unsigned int irq_btn;
	int btn_11pin_35mm_flag;

	struct hrtimer timer;
	ktime_t debounce_time;
	int headset_state;

	struct hrtimer btn_timer;
	ktime_t btn_debounce_time;
	int button_state;
	
	unsigned int use_irq : 1;
};
static struct h2w_info *hi;

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
static void h2w_early_suspend(struct early_suspend*);
static void h2w_late_resume(struct early_suspend*);
struct early_suspend    early_suspend_h2w;
static struct wake_lock h2w_wake_lock;
#endif


static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hi->sdev)) {
	case H2W_NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case H2W_SEC_HEADSET:
		return sprintf(buf, "H2W_SEC_HEADSET\n");
	case H2W_NORMAL_HEADSET:
		return sprintf(buf, "H2W_NORMAL_HEADSET\n");
	}
	return -EINVAL;
}
static ssize_t h2w_set_state(struct switch_dev *sdev, char *buf)
{
	int state = *buf - 48;
	switch(state){
	case H2W_NO_DEVICE:
		switch_set_state(&hi->sdev, H2W_NO_DEVICE);
		return 0;
	case H2W_SEC_HEADSET:
		switch_set_state(&hi->sdev, H2W_SEC_HEADSET);
		return 0;
	case H2W_NORMAL_HEADSET:
		switch_set_state(&hi->sdev, H2W_NORMAL_HEADSET);
		return 0;
	default:
		break;
	}
	return -EINVAL;
}


#if 1	// for DFMS test
static ssize_t button_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hi->button_sdev)) {
	case 0:
		return sprintf(buf, "No Input\n");
	case 1:
		return sprintf(buf, "Button Pressed\n");
	}
	return -EINVAL;
}
#endif

static void button_pressed(void)
{
	printk("button_pressed \n");
	atomic_set(&hi->btn_state, 1);
#if 0
	input_report_key(hi->input, KEY_MEDIA, 1);
#else
	input_report_key(hi->input, KEY_SEND, 1);
#endif	
	input_sync(hi->input);

	earbutton_pressed = 1;
	// for DFMS test
	queue_work(g_earbutton_work_queue, &g_earbutton_work);
}

static void button_released(void)
{
	printk("button_released \n");
	atomic_set(&hi->btn_state, 0);
#if 0
	input_report_key(hi->input, KEY_MEDIA, 0);
#else
	input_report_key(hi->input, KEY_SEND, 0);
#endif	
	input_sync(hi->input);

	earbutton_pressed = 0;
	// for DFMS test
	queue_work(g_earbutton_work_queue, &g_earbutton_work);
}

static void vear_control(int OnOff)
{
  if( USED_VEAR_LDO15 )
  {
    struct vreg *vear_vreg = vreg_get(0, LDO_VEAR);

    if( OnOff )
    {
      if(vreg_enable(vear_vreg))
      {
        printk(KERN_ERR "vreg_enable: ldo15 vreg operation failed\n");
      }
    }
    else
    {
      if(vreg_disable(vear_vreg))
      {
        printk(KERN_ERR "vreg_disable: ldo15 vreg operation failed\n");
      }
    }
  }
}


#ifdef FEATURE_SENDEND_ENABLE
static void remove_headset(void)
{
	unsigned long irq_flags;

	H2W_DBG("");

	//vear_control(0);
	gpio_set_value(GPIO_POPUP_SW_EN, 0);	//popup_sw_en off

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, switch_get_state(&hi->sdev) & ~(BIT_HEADSET | BIT_HEADSET_NO_MIC));
	mutex_unlock(&hi->mutex_lock);

	if (hi->btn_11pin_35mm_flag) 
	{
		set_irq_wake(hi->irq_btn, 0);
		local_irq_save(irq_flags);
		disable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);
		hi->btn_11pin_35mm_flag = 0;
		if (atomic_read(&hi->btn_state)) button_released();
	}
	hi->debounce_time = ktime_set(0, 200000000);  /* 100 ms */
}
#else
static void remove_headset(void)
{
	unsigned long irq_flags;

	H2W_DBG("");
#if 0
	/* Disable button */
	local_irq_save(irq_flags);
	disable_irq(hi->irq_btn);
	local_irq_restore(irq_flags);
#endif
	if (atomic_read(&hi->btn_state))
		button_released();

	hi->debounce_time = ktime_set(0, 200000000);  /* 100 ms */
}
#endif


#ifdef FEATURE_SENDEND_ENABLE
static void insert_headset(void)
{
	unsigned long irq_flags;
	int state;

	H2W_DBG("");

	state = BIT_HEADSET | BIT_HEADSET_NO_MIC;

	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_HEADSET_NO_MIC | BIT_HEADSET);

	//vear_control(1);
	gpio_set_value(GPIO_POPUP_SW_EN, 1);	//popup_sw_en on

	/* Wait pin be stable */
	msleep(200);

  // headset detect issue ===>
#ifdef JACK_ACTIVE_LOW
	if (gpio_get_value(GPIO_JACK_S_35) != 0) 
#else
	if (gpio_get_value(GPIO_JACK_S_35) != 1)
#endif        
  {
	gpio_set_value(GPIO_POPUP_SW_EN, 0);
	set_irq_type(hi->irq, IRQF_TRIGGER_FALLING);
    return;
  }
  // <==== headset detect issue

	state |= BIT_HEADSET;
	/* Enable button irq */
	if (!hi->btn_11pin_35mm_flag) 
	{
		set_irq_type(hi->irq_btn,IRQF_TRIGGER_LOW);		// detect high
		set_irq_wake(hi->irq_btn, 1);

		local_irq_save(irq_flags);
		enable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);

		hi->btn_11pin_35mm_flag = 1;
	}
	printk(KERN_INFO "11pin_3.5mm with microphone\n");

	hi->debounce_time = ktime_set(0, 20000000);  /* 500 -> 20 ms */

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
}
#else
static void insert_headset(void)
{
	unsigned long irq_flags;

	H2W_DBG("");

	switch_set_state(&hi->sdev, H2W_SEC_HEADSET);

	hi->ignore_btn = 1; // !gpio_get_value(BIGFOOT_GPIO_CABLE_IN2);
	#if 0
	/* Enable button irq */
	local_irq_save(irq_flags);
	enable_irq(hi->irq_btn);
	local_irq_restore(irq_flags);
	#endif
	hi->debounce_time = ktime_set(0, 20000000);  /* 20 ms */
}
#endif

#ifdef FEATURE_SENDEND_ENABLE
static int is_accessary_pluged_in(void)
{
	int type = 0;
	int jack_state = 0;
//	int sendend_state = 0;
	
	jack_state = gpio_get_value(GPIO_JACK_S_35);
//	sendend_state = gpio_get_value(GPIO_SEND_END);

	printk("[H2W] jack_state = %d\n",jack_state);
//	printk("[H2W]sendend_state = %d\n",sendend_state);

	if ((jack_state == 1))		type = H2W_SEC_HEADSET;
	else if ( jack_state == 0 )	type = H2W_NO_DEVICE;
	else 					type = H2W_NO_DEVICE;

	return type;
}
#endif

#if 1	// for DFMS test
static void earbutton_work(struct work_struct *work)
{
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->button_sdev, earbutton_pressed);
	mutex_unlock(&hi->mutex_lock);
}
#endif

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int jack_state;
	int type;

	H2W_DBG("");
#ifdef JACK_ACTIVE_LOW
	if (gpio_get_value(GPIO_JACK_S_35) != 0) 
#else
	if (gpio_get_value(GPIO_JACK_S_35) != 1)
#endif        
	{
		/* Headset not plugged in */
		if (switch_get_state(&hi->sdev) != H2W_NO_DEVICE)
			remove_headset();
		return;
	}

	/* Something plugged in, lets make sure its a headset */

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hi->irq);
	local_irq_restore(irq_flags);
#ifdef FEATURE_SENDEND_ENABLE
	/* Something plugged in, lets make sure its a headset */
	type = is_accessary_pluged_in();
#endif

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hi->irq);
	local_irq_restore(irq_flags);

	jack_state = gpio_get_value(GPIO_JACK_S_35);
//	msleep(1);
#ifdef JACK_ACTIVE_LOW
	if ( gpio_get_value(GPIO_JACK_S_35) == 0 )
#else
	if ( gpio_get_value(GPIO_JACK_S_35) == 1 )
#endif
	{
		if (switch_get_state(&hi->sdev) == H2W_NO_DEVICE)
		{
			insert_headset();
		}
		else
		{
			printk("GPIO_JACK_S_35 = 1, but not insert_headset\n\n");
		}
	} 
	else 
	{
		printk("JACK_S_35 was low, but not a headset " "(recent jack_state = %d)", jack_state);
	}
}

void headset_button_event(int is_press)
{
	if (!is_press) {
		if (hi->ignore_btn)
			hi->ignore_btn = 0;
		else if (atomic_read(&hi->btn_state))
			button_released();
	} else {
		if (!hi->ignore_btn && !atomic_read(&hi->btn_state))
			button_pressed();
	}
}

static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
    int count = 12;  //FMT_HanJS_DL29
	H2W_DBG("");
#ifdef FEATURE_SENDEND_ENABLE
	// check again
	while(count--)	//FMT_HanJS_DL10
    { 
      mdelay(10);
      if(hi->button_state != gpio_get_value(GPIO_SEND_END))
      {
      	printk("ERROR - button value : %d -> %d\n", hi->button_state, 
            gpio_get_value(GPIO_SEND_END));
      	return HRTIMER_NORESTART;
      }
     }
  // butten active high	
  if (gpio_get_value(GPIO_SEND_END)) 
	{
		headset_button_event(1);
		/* 10 ms */
		hi->btn_debounce_time = ktime_set(0, 10000000);
	} 
	else 
	{
		headset_button_event(0);
		/* 100 ms */
		hi->btn_debounce_time = ktime_set(0, 150000000);
	}
//	printk("%s - SEND_END=%d.\n", __func__, gpio_get_value(GPIO_SEND_END));
#else
	if (switch_get_state(&hi->sdev) == H2W_SEC_HEADSET) {
		if (0){ // gpio_get_value(GPIO_SEND_END )) {
			if (hi->ignore_btn)
				hi->ignore_btn = 0;
			else if (atomic_read(&hi->btn_state))
				button_released();
		} else {
			if (!hi->ignore_btn && !atomic_read(&hi->btn_state))
				button_pressed();
		}
	}
#endif
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	H2W_DBG("");

	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	set_irq_type(hi->irq_btn, IRQF_TRIGGER_HIGH);		// detect high

	H2W_DBG("");
	do {
		value1 = gpio_get_value(GPIO_JACK_S_35);
		set_irq_type(hi->irq, value1 ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(GPIO_JACK_S_35);
	} while (value1 != value2 && retry_limit-- > 0);

//	printk("headset value2 = %d (%d retries)\n", value2, (10-retry_limit));

#ifdef JACK_ACTIVE_LOW
	if ((switch_get_state(&hi->sdev) == H2W_NO_DEVICE) ^ value2) 
#else
	if ((switch_get_state(&hi->sdev) == H2W_NO_DEVICE) ^ !value2)
#endif	
	{ // 상태가 변할 때i i
	//	gpio_set_value(GPIO_POPUP_SW_EN, 1);	//popup_sw_en on
		hi->headset_state = value1;
		H2W_DBG("EARJACK detection.JACK_35_value=%d->%d.", value1, value2);
		if (switch_get_state(&hi->sdev) == H2W_SEC_HEADSET){
			hi->ignore_btn = 1;
		}
		/* Do the rest of the work in timer context */
		hrtimer_start(&hi->timer, hi->debounce_time, HRTIMER_MODE_REL);
	}
	return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	//H2W_DBG("");
	do {
#ifdef FEATURE_SENDEND_ENABLE
		value1 = gpio_get_value(GPIO_SEND_END );
		set_irq_type(hi->irq_btn, value1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(GPIO_SEND_END );
#else
		value1 = 0;
		set_irq_type(hi->irq_btn, value1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = 0;
#endif
	} while (value1 != value2 && retry_limit-- > 0);

	//printk("button value2 = %d (%d retries)\n", value2, (10-retry_limit));

	if(value1 == value2 && retry_limit > 0)
	{
		hi->button_state = value1;
		//printk("EARBUTTON detection.\n SEND_END_value=%d->%d.\n", value1, value2);
		hrtimer_start(&hi->btn_timer, hi->btn_debounce_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)
static int h2w_debug_set(void *data, u64 val)
{
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, (int)val);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static int h2w_debug_get(void *data, u64 *val)
{
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(h2w_debug_fops, h2w_debug_get, h2w_debug_set, "%llu\n");
static int __init h2w_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("h2w", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &h2w_debug_fops);

	return 0;
}

device_initcall(h2w_debug_init);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
extern int sleep_volumnkey_awake;
static int h2w_enable_wake_lock = 0;
void h2w_early_suspend(struct early_suspend *h)
{
  if( sleep_volumnkey_awake )
  {
     h2w_enable_wake_lock = 1;
	  wake_lock(&h2w_wake_lock);
   }
}

void h2w_late_resume(struct early_suspend *h)
{
   if(h2w_enable_wake_lock)
   {
	  wake_unlock(&h2w_wake_lock);
      h2w_enable_wake_lock = 0;
   }
}
#endif

static int h2w_probe(struct platform_device *pdev)
{
	int ret;

	printk(KERN_INFO "[H2W] Registering H2W (headset) driver\n");
	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	atomic_set(&hi->btn_state, 0);
	hi->ignore_btn = 0;
	hi->headset_state = 0;
	hi->button_state = 0;

	hi->debounce_time = ktime_set(0, 200000000);  /* 100 ms -> 200 ms */
	hi->btn_debounce_time = ktime_set(0, 50000000); /* 50 ms */
        
	hi->btn_11pin_35mm_flag = 0;

	mutex_init(&hi->mutex_lock);

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;
	hi->sdev.set_state = h2w_set_state;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

#if 1	// for DFMS test
	hi->button_sdev.name = "sec_earbutton";
	hi->button_sdev.print_name = button_print_name;

	ret = switch_dev_register(&hi->button_sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	switch_set_state(&hi->button_sdev, 0);
	
	g_earbutton_work_queue = create_workqueue("earbutton");
	if (g_earbutton_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}
#endif

	g_detection_work_queue = create_workqueue("detection");
	if (g_detection_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

//    printk("%s:%d GPIO_SEND_END=%d.\n",__func__, __LINE__, GPIO_SEND_END);

/*****************************************************************/
// JACK_S_35 GPIO setting     
/*****************************************************************/    
	ret = gpio_request(GPIO_JACK_S_35, "h2w_detect");
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_direction_input(GPIO_JACK_S_35);
	if (ret < 0)
		goto err_set_detect_gpio;

	hi->irq = gpio_to_irq(GPIO_JACK_S_35);
	if (hi->irq < 0) {
		ret = hi->irq;
		goto err_get_h2w_detect_irq_num_failed;
	}

	hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->timer.function = detect_event_timer_func;
/*****************************************************************/
// SEND_END GPIO setting     
/*****************************************************************/

	ret = gpio_request(GPIO_SEND_END , "h2w_button");
	if (ret < 0)
		goto err_request_button_gpio;

	ret = gpio_direction_input(GPIO_SEND_END );
	if (ret < 0)
		goto err_set_button_gpio;

	hi->irq_btn = gpio_to_irq(GPIO_SEND_END );
	if (hi->irq_btn < 0) {
		ret = hi->irq_btn;
		goto err_get_button_irq_num_failed;
	}

	hrtimer_init(&hi->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->btn_timer.function = button_event_timer_func;

	if( USED_VEAR_LDO15 )
	{
		struct vreg *vear_vreg = vreg_get(0, LDO_VEAR);
		vreg_set_level(vear_vreg, OUT1800mV);
		vear_control(1);
	}
/*****************************************************************/
//POPUP_SW_EN GPIO setting     
/*****************************************************************/
	ret = gpio_request(GPIO_POPUP_SW_EN, "h2w_popup_sw");
	if (ret < 0)
		goto err_request_popup_gpio;

	ret = gpio_direction_output(GPIO_POPUP_SW_EN, 0);
	if (ret < 0)
		goto err_set_popup_gpio;
  
/*****************************************************************/
// JACK_S_35 irq setting     
/*****************************************************************/       
#ifdef JACK_ACTIVE_LOW
	ret = request_irq(hi->irq, detect_irq_handler,
			  IRQF_TRIGGER_FALLING, "h2w_detect", NULL);
#else
	ret = request_irq(hi->irq, detect_irq_handler,
			  IRQF_TRIGGER_RISING, "h2w_detect", NULL);
#endif        
	if (ret < 0)
		goto err_request_detect_irq;

	hi->use_irq = ret == 0;

 	printk(KERN_INFO "Headset Driver: Start gpio inputs for %s in %s mode\n", hi->sdev.name, hi->use_irq ? "interrupt" : "polling");

	ret = set_irq_wake(hi->irq, 1);
	if (ret < 0)
		goto err_request_input_dev;
/*****************************************************************/
// SEND_END irq setting     
/*****************************************************************/
	/* Disable button until plugged in */
	set_irq_flags(hi->irq_btn, IRQF_VALID | IRQF_NOAUTOEN);
	ret = request_irq(hi->irq_btn, button_irq_handler, IRQF_TRIGGER_HIGH, "h2w_button", NULL);       // detect high
	if (ret < 0)
    	goto err_request_button_irq;

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "vino-headset";	// Behold - SGH-T939
	set_bit(EV_KEY, hi->input->evbit);
#if 0	
	set_bit(KEY_MEDIA, hi->input->keybit);
#else
	set_bit(KEY_SEND, hi->input->keybit);
#endif
	ret = input_register_device(hi->input);
	if (ret < 0)
		goto err_register_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend_h2w.suspend = h2w_early_suspend;
	early_suspend_h2w.resume = h2w_late_resume;
	register_early_suspend(&early_suspend_h2w);	

	wake_lock_init(&h2w_wake_lock, WAKE_LOCK_SUSPEND, "h2w_detect");
#endif


	return 0;


err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
#ifdef FEATURE_SENDEND_ENABLE
	free_irq(hi->irq, 0);
#endif
	free_irq(hi->irq_btn, 0);
err_get_button_irq_num_failed:
err_get_h2w_detect_irq_num_failed:
err_set_button_gpio:
err_set_popup_gpio:
err_set_detect_gpio:
err_request_detect_irq:    
       gpio_free(GPIO_JACK_S_35);
err_request_button_gpio:
err_request_popup_gpio:
err_request_button_irq: 
#if 1 // def FEATURE_SENDEND_ENABLE
	gpio_free(GPIO_SEND_END );
#endif
err_request_detect_gpio:
	destroy_workqueue(g_detection_work_queue);
	destroy_workqueue(g_earbutton_work_queue);
err_create_work_queue:
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
	printk(KERN_ERR "H2W: Failed to register driver\n");

	return ret;
}

static int h2w_remove(struct platform_device *pdev)
{
	H2W_DBG("");
	if (switch_get_state(&hi->sdev))
		remove_headset();
	input_unregister_device(hi->input);
#ifdef FEATURE_SENDEND_ENABLE
	gpio_free(GPIO_SEND_END );
#endif
	gpio_free(GPIO_JACK_S_35);
#ifdef FEATURE_SENDEND_ENABLE    
	free_irq(hi->irq_btn, 0);
#endif
	free_irq(hi->irq, 0);
	destroy_workqueue(g_detection_work_queue);
	destroy_workqueue(g_earbutton_work_queue);
	switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct platform_device h2w_device = {
	.name		= "vino-headset",
};

static struct platform_driver h2w_driver = {
	.probe		= h2w_probe,
	.remove		= h2w_remove,
	.driver		= {
		.name		= "vino-headset",
		.owner		= THIS_MODULE,
	},
};

 int __init h2w_init(void)
{
	int ret;
	H2W_DBG("JACK_S_35(%d), SEND_END(%d)", GPIO_JACK_S_35, GPIO_SEND_END);
	ret = platform_driver_register(&h2w_driver);
	if (ret)
		return ret;
	return platform_device_register(&h2w_device);
}

static void __exit h2w_exit(void)
{
	platform_device_unregister(&h2w_device);

	platform_driver_unregister(&h2w_driver);
}

module_init(h2w_init);
module_exit(h2w_exit);

MODULE_AUTHOR("anonymous <anonymous@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG Headset driver for vino");
MODULE_LICENSE("GPL");
