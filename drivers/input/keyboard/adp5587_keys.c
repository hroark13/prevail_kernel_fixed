/*
 * File: drivers/input/keyboard/adp5587_keys.c
 * Description:  keypad driver for ADP5588 and ADP5587
 *               I2C QWERTY Keypad and IO Expander *
 * 
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#include <linux/i2c/adp5587.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <mach/irqs.h>
#include <linux/earlysuspend.h>

#ifdef CONFIG_DEBUG_KEY_ACTIVE
#define LOCKUP_CAPTURE
#endif

 /* Configuration Register1 */
#define AUTO_INC        (1 << 7)
#define GPIEM_CFG       (1 << 6)
#define OVR_FLOW_M      (1 << 5)
#define INT_CFG         (1 << 4)
#define OVR_FLOW_IEN    (1 << 3)
#define K_LCK_IM        (1 << 2)
#define GPI_IEN         (1 << 1)
#define KE_IEN          (1 << 0)

/* Interrupt Status Register */
#define CMP2_INT        (1 << 5)
#define CMP1_INT        (1 << 4)
#define OVR_FLOW_INT    (1 << 3)
#define K_LCK_INT       (1 << 2)
#define GPI_INT         (1 << 1)
#define KE_INT          (1 << 0)

/* Key Lock and Event Counter Register */
#define K_LCK_EN        (1 << 6)
#define LCK21           0x30
#define KEC             0xF

/* Key Event Register xy */
#define KEY_EV_PRESSED          (1 << 7)
#define KEY_EV_MASK             (0x7F)

#define KP_SEL(x)               (0xFFFF >> (16 - x))    /* 2^x-1 */

#define KEYP_MAX_EVENT          10

#define ADP5587_DEVICE_ID_MASK  0xF /* 00001111 */
#define ADP5587_KEYMAPSIZE      80  /* KE[6:0] reflects the value 1 to 80 for key press events */

#if defined(CONFIG_MACH_CHIEF) /* need to synchronize on keypad-XXX.kl */
#define VOLUMEUP_GPIO           49
#define VOLUME_UP               61
#define VOLUME_DOWN             22
#else
#define HALL_GPIO               124
#if (CONFIG_BOARD_REVISION >= 4)
#define VOLUMEUP_GPIO           36
#define VOLUME_UP               61
#define VOLUME_DOWN             67
#endif
#endif

#define ON        1
#define OFF       0

#define DEFAULT_KEYPADBACKLIGHT_TIMEOUT  3
#define FEATURE_DISABLE_SUBKEY //Quattro use only Mainkey
#define POPUP_DELAY 1000
#define DELAY_JIFFIES	((HZ / 10) * (POPUP_DELAY / 100)) + (HZ / 100) * ((POPUP_DELAY % 100) / 10)

/*
 * Early pre 4.0 Silicon required to delay readout by at least 25ms,
 * since the Event Counter Register updated 25ms after the interrupt
 * asserted.
 */

#define WA_DELAYED_READOUT_REVID(rev)           ((rev) < 4)

struct early_suspend    early_suspend;
static short int  backlight_flag  ; 
void keypad_early_suspend(struct early_suspend *h);
void keypad_late_resume(struct early_suspend *h);

struct adp5587_kpad {
        struct i2c_client *client;
        struct input_dev *input;
        struct delayed_work work;
	struct delayed_work backlightoff;
        unsigned long delay;
        unsigned int keycode[ADP5587_KEYMAPSIZE];        
};

/* Keypad Backlight time */
unsigned int backlight_time = DEFAULT_KEYPADBACKLIGHT_TIMEOUT ;
/* sysfs class registration */
struct class *bl_class;
/* sysfs device registration */
struct device *kpd_dev;

/* sys fs : /sys/devices/virtual/key/key/key */
struct class *key_class;
struct device *key_dev;

static int key_pressed;
struct i2c_client *client_clone;

static int adp5587_read(struct i2c_client *client, u8 reg)
{
  int ret;  

  ret = i2c_smbus_read_byte_data(client, reg);

  if (ret < 0)
    dev_err(&client->dev, "Read Error\n");

  return ret;
}

static int adp5587_write(struct i2c_client *client, u8 reg, u8 val)
{
  return i2c_smbus_write_byte_data(client, reg, val);
}

void popup_switch_on(struct work_struct *ignored)
{
	adp5587_write(client_clone, ADP5587_REG_DAT_OUT3, 0x02); /* Status is W1C */
}

static DECLARE_DELAYED_WORK(popup_on_work, popup_switch_on);


static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", key_pressed );
}

static ssize_t key_store(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
     char value[120];
     sscanf(buf, "%s", value);	 
     if(value[0] =='1')
           schedule_delayed_work(&popup_on_work, DELAY_JIFFIES); 	
     else
	adp5587_write(client_clone, ADP5587_REG_DAT_OUT3, 0x00); /* Status is W1C */
}


static DEVICE_ATTR(key , S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, key_show, key_store);
/* sys fs */



#ifdef FEATURE_DISABLE_SUBKEY
static int keypad_backlight_power(int mainkey);
#else
static int keypad_backlight_power(int mainkey, int subkey);
#endif

#if (CONFIG_BOARD_REVISION >= 4)
static int convert_keycode(int key_code);
#endif


#ifdef LOCKUP_CAPTURE
#include "../../../arch/arm/mach-msm/smd_private.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_iomap-7xxx.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
static int lockup_capture_check(u8 keycode, u8 keypress)
{ 
	#define NUM_CAPTURE_KEYS 3
	static int capture_keys[NUM_CAPTURE_KEYS]={1, 51, 31}; //menu + search + power
	int i;
	int capture_key_cnt = 0;
	
	for(i=0;i<NUM_CAPTURE_KEYS;i++) {
		if(keycode == (capture_keys[i]&~0x80000000)) {
			if(keypress)
				capture_keys[i]|=0x80000000;
			else
				capture_keys[i]&=~0x80000000;
			
		}
		if(capture_keys[i]&0x80000000)
			capture_key_cnt++;
	}

	if(capture_key_cnt == NUM_CAPTURE_KEYS) {
		// notify to ARM9 for ram dump
		writel(0xCCCC, MSM_SHARED_RAM_BASE + 0x30); 
		printk("[PANIC] LOCKUP CAPTURED!!! \n");
		msm_proc_comm_reset_modem_now();
	}
}
#else
#define lockup_capture_check(keycode, keypress)
#endif

/* Bottom half for switching off the keypad backlight in case intervention is not there
   for the delayed period */ 
static void adp5587_backlightoff_work(struct work_struct *work)
{
#ifdef FEATURE_DISABLE_SUBKEY
  keypad_backlight_power(OFF);
#else
  keypad_backlight_power(OFF, OFF);
#endif

}

static void adp5587_work(struct work_struct *work)
{
  struct adp5587_kpad *kpad = container_of(work, struct adp5587_kpad, work.work);
  struct i2c_client *client = kpad->client;
  int i, key, status, ev_cnt, keypress;  
  unsigned int keycode;
   
  /* Cancel the scheduled work in case key press are continous */
  cancel_delayed_work_sync(&kpad->backlightoff);
  
  status = adp5587_read(client, ADP5587_REG_INT_STAT);
  
  if (status & OVR_FLOW_INT)      /* Unlikely and should never happen */
    dev_err(&client->dev, "Event Overflow Error\n");

  if (status & KE_INT)
  {
    ev_cnt = adp5587_read(client, ADP5587_REG_KEY_LCK_EC_STAT) & KEC;

    if (ev_cnt) 
    {
      for (i = 0; i < ev_cnt; i++) 
      {
        key = adp5587_read(client, ADP5587_REG_KEY_EVENTA + i);

        keycode = kpad->keycode[(key & KEY_EV_MASK) - 1];

#ifndef CONFIG_MACH_CHIEF
#if (CONFIG_BOARD_REVISION >= 4)        
         keycode = convert_keycode(keycode);
#endif
#endif

        keypress = key & KEY_EV_PRESSED;
        key_pressed = keypress;
        
        input_report_key(kpad->input, keycode, keypress);
        input_sync(kpad->input);
        
        printk("[input_report_key] key %d, keycode %d, keypress %d\n", key, keycode, keypress);

		    lockup_capture_check(keycode, keypress);
      }
    }
  }

  adp5587_write(client, ADP5587_REG_INT_STAT, status); /* Status is W1C */
  
  printk(" Scheduling delayed work for backlight off in adp5587_work \n");

   /* Switch on the keypad backlight */
  if(backlight_flag)
  {
    #ifdef FEATURE_DISABLE_SUBKEY
      keypad_backlight_power(ON);
    #else
     keypad_backlight_power(ON, ON);
    #endif
  }
   
  if(backlight_flag)	
    schedule_delayed_work(&kpad->backlightoff, backlight_time * HZ);
}

static irqreturn_t adp5587_irq(int irq, void *handle)
{
  struct adp5587_kpad *kpad = handle;
  
 /*
  * use keventd context to read the event fifo registers
  * Schedule readout at least 25ms after notification for
  * REVID < 4
  */  

  schedule_delayed_work(&kpad->work, kpad->delay);
  
  return IRQ_HANDLED;
}

#ifndef CONFIG_MACH_CHIEF
static irqreturn_t slide_int_handler(int irq, void *handle)
{
  struct adp5587_kpad *kpad = handle;
  int gpio_hall_ic = gpio_get_value(HALL_GPIO);
  
  #if 0
  //gpio124 == 1 -> open
  if(gpio_hall_ic) //open
  {
    input_report_switch(kpad->input, SW_LID, !gpio_hall_ic);
  }
  else
  {
    input_report_switch(kpad->input, SW_LID, !gpio_hall_ic);
  }
 
  input_sync(kpad->input);

  printk("[input_report_switch] slide_int - !gpio_hall_ic %d\n", !gpio_hall_ic);
  #ifdef FEATURE_DISABLE_SUBKEY
    keypad_backlight_power(ON);
  #else
    keypad_backlight_power(ON, ON);
  #endif

   /* Schedule work for keypad backlight off */
   schedule_delayed_work(&kpad->backlightoff, backlight_time * HZ);
  #else
    //gpio124 == 1 -> open
    if(gpio_hall_ic) //open
    {
      input_report_switch(kpad->input, SW_LID, !gpio_hall_ic);
      input_sync(kpad->input);
    }
    else
    {
      input_report_switch(kpad->input, SW_LID, !gpio_hall_ic);
      input_sync(kpad->input);
    }

    printk("[input_report_switch] slide_int - !gpio_hall_ic %d\n", !gpio_hall_ic);
    
    schedule_delayed_work(&kpad->work, kpad->delay);
  #endif
	return IRQ_HANDLED;
}
#endif

#if (CONFIG_BOARD_REVISION >= 4) /* Volume up key */
static irqreturn_t volumeup_int_handler(int irq, void *handle)
{
  struct adp5587_kpad *kpad = handle;    
  int gpio_volumeup = gpio_get_value(VOLUMEUP_GPIO);
  unsigned int keycode, keypress;

  /* VOLUME_UP(61), VOLUME_DOWN(67) */
  keycode = VOLUME_UP;
  if(!gpio_volumeup) //pressed
  {
    keypress = 128;
  }
  else //released
  {
    keypress = 0;
  }

  input_report_key(kpad->input, keycode, keypress);

  printk("[input_report_key] VOL_UP, keycode %d, keypress %d\n", keycode, keypress);

  input_sync(kpad->input);
  
	return IRQ_HANDLED;
}

static int convert_keycode(int key_code) //for keep up keymap in platform
{
  if(key_code == VOLUME_UP)
  {
    return VOLUME_DOWN;
  }
  else
  {
    return key_code;
  }
}
#endif


static int __devinit adp5587_setup(struct i2c_client *client)
{  
  int i;
  
#if 1
#if defined(CONFIG_MACH_CHIEF)

  /* Program Row0 thru Row7 as Keypad */
  adp5587_write(client, ADP5587_REG_KP_GPIO1, 0x07);

  /* Program Column0 thru Column 7 as Keypad */
  adp5587_write(client, ADP5587_REG_KP_GPIO2, 0x07);

  /* Program Column8 as Keypad and Column 9 as GPIO*/
  adp5587_write(client, ADP5587_REG_KP_GPIO3, 0x00);

  /* Disable pullup for Row0 thru Row7 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL1, 0xFF);

  /* Disable pullup for Col0 thru Col7 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL2, 0xFF);

  /* Disable pullup for Col8 and Col9 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL3, 0x03);

#else

  /* Program Row0 thru Row7 as Keypad */
  adp5587_write(client, ADP5587_REG_KP_GPIO1, 0xFF);
  
  /* Program Column0 thru Column 7 as Keypad */
//  adp5587_write(client, ADP5587_REG_KP_GPIO2, 0xFF);  // quattro_jiny46kim 0 1 1 1 1 1 1 1  0x7F
  adp5587_write(client, ADP5587_REG_KP_GPIO2, 0x7F); // col7번 gpio 타입으로 셋팅 
  
  /* Program Column8 as Keypad and Column 9 as GPIO*/
  adp5587_write(client, ADP5587_REG_KP_GPIO3, 0x00); // quattro_jiny46kim 0x01->0x00 8,9 use for GPIO
  
  /* Disable pullup for Row0 thru Row7 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL1, 0xFF);
  
  /* Disable pullup for Col0 thru Col7 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL2, 0xFF);  // 1111 1111 -> col7 pullup disabled.
  
  /* Disable pullup for Col8 and Col9 */
  adp5587_write(client, ADP5587_REG_GPIO_PULL3, 0x01); // quattro_jiny46kim 0x01 -> col8번 pullup disabled

  /* Program Column 9 as GPO*/
  adp5587_write(client, ADP5587_REG_GPIO_DIR3, 0x02);

  /* Program Column 9 GPO init*/
  adp5587_write(client, ADP5587_REG_DAT_OUT3, 0x00);

#endif
  
  for(i = 0; i < KEYP_MAX_EVENT; i++)
  {    
    adp5587_read(client, ADP5587_REG_KEY_EVENTA+i);
  }
  
  adp5587_write(client, ADP5587_REG_INT_STAT, (OVR_FLOW_INT | K_LCK_INT | GPI_INT | KE_INT));  
  adp5587_write(client, ADP5587_REG_CFG, 0x19);

#else
  struct adp5587_kpad_platform_data *pdata = client->dev.platform_data;  
  int i, ret;
  ret = adp5587_write(client, ADP5587_REG_KP_GPIO1, KP_SEL(pdata->rows));
  ret |= adp5587_write(client, ADP5587_REG_KP_GPIO2, KP_SEL(pdata->cols) & 0xFF);
  ret |= adp5587_write(client, ADP5587_REG_KP_GPIO3, KP_SEL(pdata->cols) >> 8);
  
  if (pdata->en_keylock) {
    ret |= adp5587_write(client, ADP5587_REG_UNLOCK1, pdata->unlock_key1);
    ret |= adp5587_write(client, ADP5587_REG_UNLOCK2, pdata->unlock_key2);
    ret |= adp5587_write(client, ADP5587_REG_KEY_LCK_EC_STAT, K_LCK_EN);
  }
  
  for (i = 0; i < KEYP_MAX_EVENT; i++) {
    ret |= adp5587_read(client, ADP5587_REG_KEY_EVENTA);
  }

  ret |= adp5587_write(client, ADP5587_REG_INT_STAT, OVR_FLOW_INT | K_LCK_INT | GPI_INT | KE_INT);
  //ret |= adp5587_write(client, ADP5587_REG_INT_STAT, CMP2_INT | CMP1_INT | OVR_FLOW_INT | K_LCK_INT | GPI_INT | KE_INT); /* Status is W1C */
  
  ret |= adp5587_write(client, ADP5587_REG_CFG, INT_CFG | OVR_FLOW_IEN | KE_IEN);
  
  if (ret < 0) {
    return ret;
  }
#endif

  return 0;
}

static int __devinit adp5587_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  struct adp5587_kpad *kpad;
  struct adp5587_kpad_platform_data *pdata = client->dev.platform_data;
  struct input_dev *input;
  unsigned int revid;
  int ret, i;
  int error;
  client_clone = client;
  printk("+-------------------------------------------+\n");
  printk("|         ADP5587 Keyboard Probe!!!         |\n");
  printk("+-------------------------------------------+\n");
  
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
    dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
    return -EIO;
  }
  
  if (!pdata) {
    dev_err(&client->dev, "no platform data?\n");
    return -EINVAL;
  }
  
  if (!pdata->rows || !pdata->cols || !pdata->keymap) {
    dev_err(&client->dev, "no rows, cols or keymap from pdata\n");
    return -EINVAL;
  }
  
  if (pdata->keymapsize != ADP5587_KEYMAPSIZE) {
    dev_err(&client->dev, "invalid keymapsize\n");
    return -EINVAL;
  }

  if (!client->irq) {
    dev_err(&client->dev, "no IRQ?\n");
    return -EINVAL;
  }
  
  kpad = kzalloc(sizeof(*kpad), GFP_KERNEL);
  input = input_allocate_device();
  
  if (!kpad || !input) {
    dev_err(&client->dev, "invalid kpad or input!\n");
    error = -ENOMEM;
    goto err_free_mem;
  }
  
  kpad->client = client;
  kpad->input = input;
  INIT_DELAYED_WORK(&kpad->work, adp5587_work);
  /* Initializing delayed work for Back light off */
  INIT_DELAYED_WORK(&kpad->backlightoff, adp5587_backlightoff_work);

  ret = adp5587_read(client, ADP5587_REG_DEV_ID);
  if (ret < 0) {
    error = ret;
    dev_err(&client->dev, "invalid return of adp5587_read()\n");
    goto err_free_mem;
  }

  revid = (u8) ret & ADP5587_DEVICE_ID_MASK;

  if (WA_DELAYED_READOUT_REVID(revid)) {
    kpad->delay = msecs_to_jiffies(30);
  }
  
  input->name = client->name;
  input->phys = "adp5587-keys/input0";
  input->dev.parent = &client->dev;
  
  input_set_drvdata(input, kpad);
  
  input->id.bustype = BUS_I2C;
  input->id.vendor = 0x0001;
  input->id.product = 0x0001;
  input->id.version = revid;
  
  input->keycodesize = sizeof(kpad->keycode[0]);
  input->keycodemax = pdata->keymapsize;
  input->keycode = kpad->keycode;
  
  memcpy(kpad->keycode, pdata->keymap, pdata->keymapsize * input->keycodesize);
  
  /* setup input device */
  __set_bit(EV_KEY, input->evbit);
  
  if (pdata->repeat)
    __set_bit(EV_REP, input->evbit);

  //dev_err(&client->dev, "keycodemax %d, keymapsize %d, keycodesize %d\n", input->keycodemax, pdata->keymapsize, input->keycodesize);

  for (i = 0; i < input->keycodemax; i++) {
    //dev_err(&client->dev, "kpad->keycode[%d]=%d\n", i, kpad->keycode[i]);
    __set_bit(kpad->keycode[i] & KEY_MAX, input->keybit);
  }

#ifndef CONFIG_MACH_CHIEF
  {
    //JBKIM
    input_set_capability(input, EV_SW, SW_LID);
    
    if(gpio_get_value(HALL_GPIO))
      input->sw[SW_LID] = 0;
    else
      input->sw[SW_LID] = 1;
  }    
#endif

  __clear_bit(KEY_RESERVED, input->keybit);
  
  error = input_register_device(input);
  if (error) {
    dev_err(&client->dev, "unable to register input device\n");
    goto err_free_mem;
  }
  
  error = request_irq(client->irq, adp5587_irq,
                      IRQF_TRIGGER_FALLING | IRQF_DISABLED,
                      client->dev.driver->name, kpad);
  if (error) {
    dev_err(&client->dev, "irq %d busy?\n", client->irq);
    goto err_unreg_dev;
  }

#ifndef CONFIG_MACH_CHIEF
  /*JBKIM*/
  {
    error = request_irq(MSM_GPIO_TO_INT(HALL_GPIO), slide_int_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_DISABLED,
		    "Quattro_Slide", (void*)kpad);

    if (error) {
      dev_err(&client->dev, "irq %d busy?\n", 188);
      goto err_unreg_hall_dev;
    }
  }
#endif

#if (CONFIG_BOARD_REVISION >= 4) /* Volume up key */
  #if CONFIG_MACH_CHIEF
      gpio_tlmm_config(GPIO_CFG(49 , 0, 0, GPIO_PULL_UP,GPIO_2MA), GPIO_ENABLE);
  #endif
  error = request_irq(MSM_GPIO_TO_INT(VOLUMEUP_GPIO), volumeup_int_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_DISABLED,
	    "Quattro_Volumeup", (void*)kpad);

  if (error) {
    dev_err(&client->dev, "IRQ for VOLUMEUP_GPIO\n");
    goto err_unreg_hall_dev;
  }
#endif

  error = adp5587_setup(client);
  if (error)
  {
    dev_err(&client->dev, "invalid return of adp5587_setup()\n");
    goto err_free_irq;
  }
  
  device_init_wakeup(&client->dev, 1);
  i2c_set_clientdata(client, kpad);

  early_suspend.suspend = keypad_early_suspend;
  early_suspend.resume = keypad_late_resume;
  register_early_suspend(&early_suspend);
  backlight_flag = 1 ;
#ifdef FEATURE_DISABLE_SUBKEY
  keypad_backlight_power(ON);
#else
  keypad_backlight_power(ON, ON);
#endif

  dev_info(&client->dev, "Rev.%d keypad, irq %d, keypad_backlight ON\n", revid, client->irq);
  
  return 0;
  
err_unreg_hall_dev:
#ifndef CONFIG_MACH_CHIEF  
    free_irq(MSM_GPIO_TO_INT(HALL_GPIO), kpad);
#endif
#if (CONFIG_BOARD_REVISION >= 4) /* Volume up key */
    free_irq(MSM_GPIO_TO_INT(VOLUMEUP_GPIO), kpad);
#endif
err_free_irq:
    free_irq(client->irq, kpad);
err_unreg_dev:
    input_unregister_device(input);
    input = NULL;
err_free_mem:
    input_free_device(input);
    kfree(kpad);
  
  return error;
}

static int __devexit adp5587_remove(struct i2c_client *client)
{
  struct adp5587_kpad *kpad = i2c_get_clientdata(client);
  
  adp5587_write(client, ADP5587_REG_CFG, 0);
  free_irq(client->irq, kpad);
  cancel_delayed_work_sync(&kpad->work);
  cancel_delayed_work_sync(&kpad->backlightoff);
  input_unregister_device(kpad->input);
  i2c_set_clientdata(client, NULL);
  kfree(kpad);
  
  return 0;
}

#ifdef CONFIG_PM
static int adp5587_suspend(struct i2c_client *client, pm_message_t mesg)
{
  struct adp5587_kpad *kpad = i2c_get_clientdata(client); 

  dev_err(&client->dev, "adp5587_suspend(), keypad_backlight OFF\n");

  disable_irq(client->irq);
#ifndef CONFIG_MACH_CHIEF
  disable_irq(MSM_GPIO_TO_INT(HALL_GPIO));
#endif
  cancel_delayed_work_sync(&kpad->work);
  
  if (device_may_wakeup(&client->dev))
  {
    enable_irq_wake(client->irq);
#ifndef CONFIG_MACH_CHIEF
    enable_irq_wake(MSM_GPIO_TO_INT(HALL_GPIO));
#endif
  }
  
  return 0;
}

static int adp5587_resume(struct i2c_client *client)
{
  dev_err(&client->dev, "adp5587_resume(), keypad_backlight ON\n");

  if (device_may_wakeup(&client->dev))
  {
    disable_irq_wake(client->irq);
#ifndef CONFIG_MACH_CHIEF
    disable_irq_wake(MSM_GPIO_TO_INT(HALL_GPIO));
#endif
  }
  
  enable_irq(client->irq);
#ifndef CONFIG_MACH_CHIEF
  enable_irq(MSM_GPIO_TO_INT(HALL_GPIO));
#endif
  
  return 0;
}
#if 0
static const struct dev_pm_ops adp5587_dev_pm_ops = {
        .suspend = adp5587_suspend,
        .resume  = adp5587_resume,
};
#endif
#endif

static const struct i2c_device_id adp5587_id[] = {        
        { "adp5587-keys", 0 },
        { }
};

MODULE_DEVICE_TABLE(i2c, adp5587_id);

static struct i2c_driver adp5587_driver = {
        .driver = {
                .name = "adp5587-keys",
#ifdef CONFIG_PM
                //.pm   = &adp5587_dev_pm_ops,
#endif
        },
        .probe    = adp5587_probe,
        .remove   = __devexit_p(adp5587_remove),
        .id_table = adp5587_id,
#ifdef CONFIG_PM
        .suspend = adp5587_suspend,
        .resume  = adp5587_resume,
#endif
};

#ifdef FEATURE_DISABLE_SUBKEY
static int keypad_backlight_power(int mainkey)
#else
static int keypad_backlight_power(int mainkey, int subkey)
#endif
{
  struct vreg *vreg_mainkey_backlight;  
  int ret;

  /* control VDD_MAINKEY_3.3V */
  vreg_mainkey_backlight = vreg_get(NULL, "ldo18");
  
  if(IS_ERR(vreg_mainkey_backlight)) {
    printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__,PTR_ERR(vreg_mainkey_backlight));
    return PTR_ERR(vreg_mainkey_backlight);
  }  
  
  if(mainkey == ON) {
    ret = vreg_set_level(vreg_mainkey_backlight, OUT3300mV);
    if(ret) {
      printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__,ret);
      return -EIO;
    }
    
    ret = vreg_backlight_enable(vreg_mainkey_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  } 
  else 
  { 
    ret = vreg_backlight_disable(vreg_mainkey_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  }

#ifndef FEATURE_DISABLE_SUBKEY
  struct vreg *vreg_subkey_backlight;

  /* control VDD_SUBKEY_3.3V */
  vreg_subkey_backlight = vreg_get(NULL, "ldo19");

  if(IS_ERR(vreg_subkey_backlight)) {
    printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__,PTR_ERR(vreg_subkey_backlight));
    return PTR_ERR(vreg_subkey_backlight);
  }

  if(subkey == ON) {
    ret = vreg_set_level(vreg_subkey_backlight, OUT3300mV);
    if(ret) {
      printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__,ret);
      return -EIO;
    }
    
    ret = vreg_backlight_enable(vreg_subkey_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  }
  else
  { 
    ret = vreg_backlight_disable(vreg_subkey_backlight);
    if(ret) {
      printk(KERN_ERR "%s: vreg disable failed (%d)\n", __func__,ret);
      return -EIO;
    }
  }
#endif

  return 0;
}


void keypad_early_suspend(struct early_suspend *h)
{


	#ifdef FEATURE_DISABLE_SUBKEY
  	keypad_backlight_power(OFF);
	#else
  	keypad_backlight_power(OFF, OFF);
	#endif
	backlight_flag = 0 ;

}

void keypad_late_resume(struct early_suspend *h)
{

	#ifdef FEATURE_DISABLE_SUBKEY
  	keypad_backlight_power(ON);
	#else
	keypad_backlight_power(ON, ON);
	#endif
	backlight_flag = 1 ;


}

#if defined(CONFIG_MACH_CHIEF)
int expandergpio_get_value(unsigned int gpio)
{
  int ret, reg_index, reg;
  
  reg_index = (gpio & 0x60) >> 5;

  if(reg_index == 1)
    reg = ADP5587_REG_DAT_STAT1;
  else if(reg_index == 2)
    reg = ADP5587_REG_DAT_STAT2;
  else if(reg_index == 3)
    reg = ADP5587_REG_DAT_STAT3;
  else
    return -ENOTSUPP;

  ret = adp5587_read(client_clone, reg);

  reg_index = (gpio & 0x0F);
  ret = ret >> reg_index;

  return ret;
}

void expandergpio_set_value(unsigned int gpio, int on)
{
  int ret, reg_index, reg1, reg2, value1, value2;
  
  reg_index = (gpio & 0x60) >> 5;

  if(reg_index == 1) {
    reg1 = ADP5587_REG_GPIO_DIR1;
    reg2 = ADP5587_REG_DAT_OUT1;
  }
  else if(reg_index == 2) {
    reg1 = ADP5587_REG_GPIO_DIR2;
    reg2 = ADP5587_REG_DAT_OUT2;
  }
  else if(reg_index == 3) {
    reg1 = ADP5587_REG_GPIO_DIR3;
    reg2 = ADP5587_REG_DAT_OUT3;
  }
  else
    return -ENOTSUPP;

  value1 = adp5587_read(client_clone, reg1);
  value2 = adp5587_read(client_clone, reg2);

  reg_index = (gpio & 0x0F);
  value1 |= (1 << reg_index); /* set GPIO data direction to output */

  if(on > 0) /* set GPIO data out */
    value2 |= (1 << reg_index);
  else
    value2 &= ~(1 << reg_index);

  adp5587_write(client_clone, reg1, value1);
  adp5587_write(client_clone, reg2, value2);

  return ret;
}

int expandergpio_configure(unsigned int gpio, unsigned long flags)
{
  int ret=0, reg_index, reg1, reg2, value1, value2, flags_index;
  
  reg_index = (gpio & 0x60) >> 5;

  if(reg_index == 1) {
    reg1 = ADP5587_REG_GPIO_DIR1;
    reg2 = ADP5587_REG_DAT_OUT1;
  }
  else if(reg_index == 2) {
    reg1 = ADP5587_REG_GPIO_DIR2;
    reg2 = ADP5587_REG_DAT_OUT2;
  }
  else if(reg_index == 3) {
    reg1 = ADP5587_REG_GPIO_DIR3;
    reg2 = ADP5587_REG_DAT_OUT3;
  }
  else
    return -ENOTSUPP;

  value1 = adp5587_read(client_clone, reg1);
  value2 = adp5587_read(client_clone, reg2);

  reg_index = (gpio & 0x0F);
  flags_index = flags >> 17;

  if(flags_index == 1) /* GPIOF_DRIVE_INPUT */
  {
    value1 &= ~(1 << reg_index); /* set GPIO data direction to input */
    adp5587_write(client_clone, reg1, value1);
  }
  else /* GPIOF_DRIVE_OUTPUT */
  {  
    value1 |= (1 << reg_index); /* set GPIO data direction to output */
    adp5587_write(client_clone, reg1, value1);

    if(flags_index == 0x0A) /* GPIOF_OUTPUT_HIGH */
      value2 |= (1 << reg_index);
    else if(flags_index == 0x06) /* GPIOF_OUTPUT_LOW */
      value2 &= ~(1 << reg_index);
    else /* no set */
      return ret;

    adp5587_write(client_clone, reg2, value2);
  }
  return ret;
}

unsigned int expandergpio_int_en(unsigned int gpio)
{
  int ret=0, reg_index, reg1, reg2, value1, value2, flags_index;

  reg_index = (gpio & 0x60) >> 5;

  if(reg_index == 1) {
    reg1 = ADP5587_REG_GPIO_DIR2;
    reg2 = ADP5587_REG_INT_EN1;
  }
  else if(reg_index == 2) {
    reg1 = ADP5587_REG_GPIO_DIR2;
    reg2 = ADP5587_REG_INT_EN2;
  }
  else if(reg_index == 3) {
    reg1 = ADP5587_REG_GPIO_DIR3;
    reg2 = ADP5587_REG_INT_EN3;
  }
  else
    return -ENOTSUPP;

  value1 = adp5587_read(client_clone, reg1);
  value2 = adp5587_read(client_clone, reg2);

  reg_index = (gpio & 0x0F);
  value1 &= ~(1 << reg_index); /* set GPIO data direction to input */
  value2 |= (1 << reg_index); /* set GPIO interrupt enable */  

  adp5587_write(client_clone, reg1, value1);
  adp5587_write(client_clone, reg2, value2);

  return ret;
}


void keyexpander_gpio_control()//(int val, int onoff)
{
//  int direction = adp5587_read(client_clone, ADP5587_REG_GPIO_DIR2);
//  int level = adp5587_read(client_clone, ADP5587_REG_DAT_OUT2);
  int level;

  if (1 > 0)//onoff)
  {
    level |= (1 << 3);//val);
  }
  else
  {
  	 level &= ~(1 << 3);//val);
  }
  adp5587_write(client_clone, ADP5587_REG_GPIO_DIR2, level);//direction); //GPIO data direction
  adp5587_write(client_clone, ADP5587_REG_DAT_OUT2, level); //GPIO data out
}
#endif


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
        __ATTR(keypad_bl_time, 0777, keypad_backlight_time_show, keypad_backlight_time_store),
};
 
static int __init adp5587_init(void)
{

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

	/* Creating device attributes for setting keypad backlight time */
  	if (device_create_file(kpd_dev, &bl_keypad_attributes[0]) < 0){
   		printk("Failed to create device attributes for Keypad Backlight \n");
		device_destroy(bl_class,0);
		class_destroy(bl_class);
	}

	/* sys fs */
	key_class = class_create(THIS_MODULE, "key");
	if (!key_class)
		printk("Failed to create class(key)!\n");

	key_dev = device_create(key_class, NULL, 0, NULL, "key");
	if (!key_dev)
		printk("Failed to create device(key)!\n");

	if (device_create_file(key_dev, &dev_attr_key) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_key.attr.name);
	/* sys fs */

		
  return i2c_add_driver(&adp5587_driver);
}
module_init(adp5587_init);

static void __exit adp5587_exit(void)
{
  /* removing device attributes entries */
  device_remove_file(&kpd_dev, &bl_keypad_attributes[0]);
  /* removing device backlight under keypad backlight class */
  device_destroy(bl_class,0);
  /* removing class keypad backlight from sysfs entries */
  class_destroy(bl_class);

  i2c_del_driver(&adp5587_driver);
}
module_exit(adp5587_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADP5588/87 Keypad driver");
MODULE_ALIAS("platform:adp5587-keys");
