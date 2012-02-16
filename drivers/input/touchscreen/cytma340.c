#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/timer.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>

#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/vreg.h>
#include <linux/slab.h>

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
#define CYTSP_WDOG_ENABLE
#else
//#define CYTSP_WDOG_ENABLE
#endif

#define CYTSP_FWUPG_ENABLE
#define CYTSP_BOOT_CALIBRATE

#undef CONFIG_CPU_FREQ

#ifdef CYTSP_FWUPG_ENABLE

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_GIOS)
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
#include "cytma340_fw_VINO.h"
#include "cytma340_fw_VINO_NP.h" // new patern
#elif defined(CONFIG_MACH_RANT3)
#include "cytma340_fw_RANT3.h"
#endif
#include <linux/sched.h>
#include <linux/reboot.h>
#include <linux/sys.h>
extern void msm_proc_comm_modem_normal_reset(void);
#else
#include "cytma340_fw.h"
#endif

#endif

#ifdef CONFIG_CPU_FREQ
//#include <plat/s5p6442-dvfs.h>
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define TSP_SDA 29
#define TSP_SCL 30
#define TSP_INT 19
#define TSP_RESET 21

#define CYTOUCH_REG_HST_MODE 		0x00
#define CYTOUCH_REG_TT_MODE 		0x01
#define CYTOUCH_REG_TT_STAT 		0x02
#define CYTOUCH_REG_TOUCH1_XH 		0x03
#define CYTOUCH_REG_TOUCH1_XL 		0x04
#define CYTOUCH_REG_TOUCH1_YH 		0x05
#define CYTOUCH_REG_TOUCH1_YL 		0x06
#define CYTOUCH_REG_TOUCH1_Z 		0x07
#define CYTOUCH_REG_TOUCH12_ID 		0x08
#define CYTOUCH_REG_TOUCH2_XH 		0x09
#define CYTOUCH_REG_TOUCH2_XL 		0x0A
#define CYTOUCH_REG_TOUCH2_YH 		0x0B
#define CYTOUCH_REG_TOUCH2_YL 		0x0C
#define CYTOUCH_REG_TOUCH2_Z 		0x0D
#define CYTOUCH_REG_GEST_CNT 		0x0E
#define CYTOUCH_REG_GEST_ID 		0x0F
#define CYTOUCH_REG_TOUCH3_XH 		0x10
#define CYTOUCH_REG_TOUCH3_XL 		0x11
#define CYTOUCH_REG_TOUCH3_YH 		0x12
#define CYTOUCH_REG_TOUCH3_YL 		0x13
#define CYTOUCH_REG_TOUCH3_Z 		0x14
#define CYTOUCH_REG_TOUCH34_ID 		0x15
#define CYTOUCH_REG_TOUCH4_XH 		0x16
#define CYTOUCH_REG_TOUCH4_XL 		0x17
#define CYTOUCH_REG_TOUCH4_YH 		0x18
#define CYTOUCH_REG_TOUCH4_YL 		0x19
#define CYTOUCH_REG_TOUCH4_Z 		0x1A
#define CYTOUCH_REG_VENDOR_ID 		0x1B
#define CYTOUCH_REG_MODULE_ID 		0x1C
#define CYTOUCH_REG_FW_VER 			0x1D
#define CYTOUCH_REG_GEST_SET 		0x1E
#define CYTOUCH_REG_WDOG 			0x1F
#define CYTOUCH_REG_CALIBRATE		0x00
#define CYTOUCH_REG_READ_START		CYTOUCH_REG_TT_STAT
#define CYTOUCH_REG_READ_SIZE		(CYTOUCH_REG_TOUCH2_Z-CYTOUCH_REG_TT_STAT+1)
#define CYTOUCH_REG_READ_POS(x)		(x-CYTOUCH_REG_READ_START)
#define CYTOUCH_MAX_ID			(15)

//moon #define IRQ_TOUCH_INT IRQ_EINT_GROUP(15,2)
#define IRQ_TOUCH_INT MSM_GPIO_TO_INT(TSP_INT) // rant3.boot

#define CYTOUCH_MULTI_TOUCH_NUM		2

#define TOUCH_MENU		KEY_MENU
#define TOUCH_BACK		KEY_BACK

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS) // temp key define
#define MENUKEY_BIT    (0x01)
#define HOMEKEY_BIT    (0x02)
#define BACKKEY_BIT    (0x04)
#define SEARCHKEY_BIT  (0x08)
#define TOUCH_HOME		KEY_HOME
#define TOUCH_SEARCH		KEY_SEARCH

static const int cytouch_keycodes[] = { TOUCH_MENU, TOUCH_HOME, TOUCH_BACK, TOUCH_SEARCH};
#else
static const int cytouch_keycodes[] = { TOUCH_MENU, TOUCH_BACK };
#endif
static struct vreg *vreg_touch;

static int cytouch_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len);
static int cytouch_i2c_write(struct i2c_client* p_client, u8 reg, u8 data);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cytouch_early_suspend(struct early_suspend*);
static void cytouch_late_resume(struct early_suspend*);
#endif
static void cytouch_hw_reset(void);

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static struct device* gp_ts_dev;

static int g_vendor_id;
static int g_module_id;
static int g_fw_ver;
int tma340_frimware_update(void);


#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
#define TSP_POWERON_DELAY_TIME    500
#else
#define TSP_POWERON_DELAY_TIME    400
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//TSP F/W update form sd card
#define TSP_SDCARD_UPDATE
#if defined(CONFIG_MACH_RANT3)
#define TSP_SDCARD_INDICATE
#endif
#ifdef TSP_SDCARD_UPDATE
	#include <linux/fs.h>
	#include <linux/vmalloc.h>
	uint8_t* pfirmware = NULL;
	uint8_t fromsdcard = false;

	//test indicate
	#ifdef TSP_SDCARD_INDICATE
		#define GPIO_LED_B 89
		#define GPIO_LED_R 90
	#endif
#endif
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
#define CYTOUCH_REG_BTN_STAT 		CYTOUCH_REG_TT_STAT
#define CYTOUCH_NEW_PATERN    ((g_module_id >= 0x04) ? 1 : 0 )
#endif


static struct workqueue_struct* cytouch_irq_queue;

typedef enum
{
	CYTOUCH_PWROFF = 0,
	CYTOUCH_PWRON = 1,
}CYTOUCH_PWRSTAT;

static int cytouch_hw_set_pwr(CYTOUCH_PWRSTAT onoff);

typedef struct
{
	int x;
	int y;
	int z;
	int stat;
	int id;
}CYTOUCH_POINT;

enum
{
	CYTOUCH_ID_STAT_RELEASED = 0,
	CYTOUCH_ID_STAT_PRESSED = 1,
	CYTOUCH_ID_STAT_MOVED = 1,
};

enum
{
	CYTOUCH_ID_STAT_DIRTY = 0,
	CYTOUCH_ID_STAT_NEW = 1,
};

typedef struct
{
	int status;
	int dirty;
	int x,y,z;
	int b_report;
}CYTOUCH_ID_STAT;


typedef struct
{
	u8 tt_stat;
	u8 touch1_xh;
	u8 touch1_xl;
	u8 touch1_yh;
	u8 touch1_yl;
	u8 touch1_z;
	u8 touch12_id;
	u8 touch2_xh;
	u8 touch2_xl;
	u8 touch2_yh;
	u8 touch2_yl;
	u8 touch2_z;
	u8 gest_cnt;
	u8 gest_id;
	u8 touch3_xh;
	u8 touch3_xl;
	u8 touch3_yh;
	u8 touch3_yl;
	u8 touch3_z;
	u8 touch34_id;
	u8 touch4_xh;
	u8 touch4_xl;
	u8 touch4_yh;
	u8 touch4_yl;
	u8 touch4_z;
}__attribute__((packed))CYTOUCH_RAW_DATA;

static struct timer_list g_cytouch_touch_timer;
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
static struct timer_list g_button_timer;
#endif

struct cytouch_ts_data
{
  uint16_t addr;
  struct i2c_client *client;
  struct input_dev *input_dev;
  int use_irq;
  struct work_struct  work;

  CYTOUCH_ID_STAT id_stat[CYTOUCH_MAX_ID+1];

  // temp,, =..
  int prev_menu;
  int prev_back;
  int prev_home;
  int prev_search;

#if defined(CONFIG_HAS_EARLYSUSPEND)
  struct early_suspend early_suspend;
#endif
};


static struct cytouch_ts_data* ts_global;
static int read_tsp_version;

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
static unsigned int touch_button_enable = 1;
extern void keypad_tsp_touch_event(void);
static int tsp_i2c_read_access = 0;
static int bTargetEraseState = 0;
#endif

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
static void deferred_restart(struct work_struct *dummy)
{
    msm_proc_comm_modem_normal_reset();
	//sys_sync();
	kernel_restart(NULL);
}
static DECLARE_WORK(restart_work, deferred_restart);
#endif


void cytouch_release_all(void)
{
	int i = 0;

  del_timer(&g_cytouch_touch_timer);

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
  del_timer(&g_button_timer);

  if( ts_global->prev_menu )
  {
    input_report_key(ts_global->input_dev, TOUCH_MENU, 0);
    ts_global->prev_menu = 0;
  }
  if( ts_global->prev_home )
  {
    input_report_key(ts_global->input_dev, TOUCH_HOME, 0);
    ts_global->prev_home = 0;
  }
  if( ts_global->prev_back )
  {
    input_report_key(ts_global->input_dev, TOUCH_BACK, 0);
    ts_global->prev_back = 0;
  }
  if( ts_global->prev_search )
  {
    input_report_key(ts_global->input_dev, TOUCH_SEARCH, 0);
    ts_global->prev_search = 0;
  }
#endif

	/* check previous touch input and return RELEASE */
	for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
	{
		if (ts_global->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
		{
			input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, ts_global->id_stat[i].x);
			input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, ts_global->id_stat[i].y);
			input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, ts_global->id_stat[i].z);
			input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			ts_global->id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
		}

		ts_global->id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;

		input_mt_sync(ts_global->input_dev);
	}
	input_sync(ts_global->input_dev);
}
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
void cytouch_button_timer_body(struct work_struct* p_work)
{
  u8 buf = 0x0;
  int ret = 0;

  ret = cytouch_i2c_read(ts_global->client, CYTOUCH_REG_BTN_STAT, &buf, 1);
  if (0 != ret)
  {
    pr_debug("%s : fail!\n", __func__);
    return;
  }

  //pr_debug("%s : Buffer _ shift!!! buf=[0x%x, 0x%x, %d] !\n", __func__, buf, buf >> 4, touch_button_enable);
  buf = buf >> 4;

  if( ts_global->prev_menu && (buf & MENUKEY_BIT)==0 )
  {
    pr_debug("%s : timeout MENU generated(UP)!\n", __func__);
    if( touch_button_enable )
      input_report_key(ts_global->input_dev, TOUCH_MENU, 0);
    ts_global->prev_menu = FALSE;
  }

  if( ts_global->prev_home && !(buf & HOMEKEY_BIT) )
  {
    pr_debug("%s : timeout HOME generated(UP)!\n", __func__);
    if( touch_button_enable )
      input_report_key(ts_global->input_dev, TOUCH_HOME, 0);
    ts_global->prev_home = FALSE;
  }
  if( ts_global->prev_back && !(buf & BACKKEY_BIT) )
  {
    pr_debug("%s : timeout BACK key generated(UP)!\n", __func__);
    if( touch_button_enable )
      input_report_key(ts_global->input_dev, TOUCH_BACK, 0);
    ts_global->prev_back = FALSE;
  }
  if( ts_global->prev_search && !(buf & SEARCHKEY_BIT) )
  {
    pr_debug("%s : timeout SEARCH key generated(UP)!\n", __func__);
    if( touch_button_enable )
      input_report_key(ts_global->input_dev, TOUCH_SEARCH, 0);
    ts_global->prev_search = FALSE;
  }
}

DECLARE_WORK(cytouch_button_timer_wq, cytouch_button_timer_body);

/* this is called when last touch key IRQ isn't handled */
/* since i2c read should be done outside of atomic scope, */
/* schedule_work() is used.. -.- */
static void cytouch_button_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_button_timer_wq);
}
#endif

void cytouch_touch_timer_body(struct work_struct* p_work)
{
  u8 buf = 0x0;
  int i = 0;
  int ret = 0;
  int tsp_mt_sync_event = 0;
  int tsp_sync_event = 0;

  ret = cytouch_i2c_read(ts_global->client, CYTOUCH_REG_TT_STAT, &buf, 1);

  if (0 != ret)
  {
    pr_debug("%s : fail!\n", __func__);
    return;
  }

  if (0 == (buf & 0xf))	/* lower 4bits are number of touch inputs */
  {
    /* check previous touch input and return RELEASE */
    for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
    {
      tsp_mt_sync_event = 0;
      if (ts_global->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
      {
        input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, ts_global->id_stat[i].x);
        input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, ts_global->id_stat[i].y);
        input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, ts_global->id_stat[i].z);
        input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        ts_global->id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
        pr_debug("%s : lost TOUCH UP generated!\n", __func__);
        tsp_sync_event = 1;
        tsp_mt_sync_event = 1;
      }

      ts_global->id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;
      if( tsp_mt_sync_event ) {
        input_mt_sync(ts_global->input_dev);
      }
    }

    if( tsp_sync_event ) {
      input_sync(ts_global->input_dev);
    }
  }
}

DECLARE_WORK(cytouch_touch_timer_wq, cytouch_touch_timer_body);

/* as same as cytouch_key_timer_handler -.- */
static void cytouch_touch_timer_handler(unsigned long data)
{
	schedule_work(&cytouch_touch_timer_wq);
}

static void cytouch_init_rel_timer(void)
{
	init_timer(&g_cytouch_touch_timer);

	g_cytouch_touch_timer.function = cytouch_touch_timer_handler;
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
  init_timer(&g_button_timer);
  g_button_timer.function = cytouch_button_timer_handler;
#endif
}

#ifdef CYTSP_WDOG_ENABLE
static void cytouch_wdog_wq_body(struct work_struct* p_work);
DECLARE_DELAYED_WORK(cytouch_wdog_wq, cytouch_wdog_wq_body);


#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
int tsp_esd_detect = false;
#endif

static void cytouch_wdog_wq_body(struct work_struct* p_work)
{
  u8 wdog_val = 0x0;
  static int prev_wdog_val = -1;
  int fail = 0;
  int ret = 0;
  static int repeat_count = 0;

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
    if( tsp_i2c_read_access ) {
      pr_debug(" body i2c access ]\n");
      repeat_count = 0;
      goto wdog_wq_restart;
    }

    if (1 != gpio_get_value(TSP_SDA) &&  1 != gpio_get_value(TSP_SCL)) {
      if( repeat_count < 1 )
        repeat_count += 1;
      else
        fail = 1;
    }

    if( fail == 0 )  {
      ret = cytouch_i2c_read(ts_global->client, CYTOUCH_REG_WDOG, &wdog_val, 1);
      if (0 != ret)  {
        pr_debug("read %d, repeat_count=%d : fail!\n", CYTOUCH_REG_WDOG, repeat_count);
        repeat_count += 1;
      }  else  {
        repeat_count = 0;
      }
    }
#else
  if (1 == gpio_get_value(TSP_SDA) &&  1 == gpio_get_value(TSP_SCL))
  {
    ret = cytouch_i2c_read(ts_global->client, CYTOUCH_REG_WDOG, &wdog_val, 1);

    if (0 != ret)
    {
      pr_debug("%s : fail!\n", __func__);
      fail = 1;
    }
  }
  else
  {
    pr_debug("%s : I2C line setup fail!\n", __func__);
    fail = 1;
  }
#endif

  /* wdog value isn't changed OR i2c fails */
  if ((wdog_val == (u8)prev_wdog_val && repeat_count ==1 ) || fail == 1
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
    || tsp_esd_detect == true
#endif
  )
  {
    pr_debug("%s : Touch WDOG fail! (%d->%d)\n", __func__, prev_wdog_val, wdog_val);
    disable_irq(IRQ_TOUCH_INT);

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
    cytouch_release_all();
#endif

    cytouch_hw_reset();

    gpio_set_value(TSP_SCL, 1);
    gpio_set_value(TSP_SDA, 1);

    enable_irq(IRQ_TOUCH_INT);
    prev_wdog_val = -1;
    repeat_count = 0;
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
    tsp_esd_detect = false;
#endif
  }
  else
  {
    //pr_debug("%s : Touch WDOG OK! (%d->%d)\n", __func__, prev_wdog_val, wdog_val);
    prev_wdog_val = wdog_val;
  }

wdog_wq_restart:
  schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(1200));
}

static void cytouch_init_wdog(void)
{
    pr_debug("%s : Touch WDOG initial!!!\n", __func__);

    if(g_fw_ver == -1)
      schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(3000));
    else
      schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(2000));
}

static void cytouch_pause_wdog(void)
{
    pr_debug("%s : Touch WDOG paused!\n", __func__);
    cancel_delayed_work_sync(&cytouch_wdog_wq);
}

static void cytouch_resume_wdog(void)
{
    pr_debug("%s : Touch WDOG resumed!\n", __func__);
    schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(2000));
}

void tsp_detect_esd_wrok(void)
{
  tsp_esd_detect = true;
  cytouch_pause_wdog();
  schedule_delayed_work(&cytouch_wdog_wq, msecs_to_jiffies(20));
}
EXPORT_SYMBOL(tsp_detect_esd_wrok);
#endif

static int cytouch_hw_set_pwr(CYTOUCH_PWRSTAT onoff)
{
  int ret;

  pr_debug("%s\n", __func__);

  if (onoff != CYTOUCH_PWRON && onoff != CYTOUCH_PWROFF)
  {
    pr_debug("%s : Error! Unknown parameter => %d\n", __func__, onoff);
    return FALSE;
  }

  vreg_touch = vreg_get(NULL, "ldo8"); /* VTOUCH_3.0V */

  if(onoff)
  {
    ret = vreg_enable(vreg_touch);
    if (ret)
    {
      printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
      return FALSE;
    }


    else
    {
      printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
    }
  }
  else
  {
    ret = vreg_disable(vreg_touch);
    if (ret)
    {
      printk(KERN_ERR "%s: vreg_touch disable failed (%d)\n", __func__, ret);
      return FALSE;
    }
    else
    {
      printk(KERN_INFO "%s: vreg_touch disable success!\n", __func__);
    }
  }
  return TRUE;
}


#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
void cytouch_irq_body(struct work_struct* p_work)
{
  struct cytouch_ts_data *ts = container_of(p_work, struct cytouch_ts_data, work);

  CYTOUCH_POINT point[3] = {{0,0,0,0,0}, {0,0,0,0,0}, {0,0,0,0,0}};
  CYTOUCH_RAW_DATA buf = {0,};
  u8 num_of_touch = 0;

  int cur_key = 0;
  int b_add_touch_timer = FALSE;

  int i = 0;
  int ret = 0;

  int btn_ret = 0;
  u8 btn_buf = 0;
  int b_add_button_timer = FALSE;
  int btn_count = 0;

  int ignore_touch_event = 0;
  int ignore_btn_event = 0;

  int touch_sync_event = 0;
  int button_release_event = 0;

#ifdef CONFIG_CPU_FREQ		// This makes cpu work fast when user touch display
  set_dvfs_perf_level();
#endif

  tsp_i2c_read_access = 1;

  /* delete key timer if this irq is enabled by touch button or touch */
  cancel_work_sync(&cytouch_touch_timer_wq);
  del_timer(&g_cytouch_touch_timer);
  cancel_work_sync(&cytouch_button_timer_wq);
  del_timer(&g_button_timer);

  ret = cytouch_i2c_read(ts->client, CYTOUCH_REG_READ_START, (u8*)&buf, CYTOUCH_REG_READ_SIZE);
  tsp_i2c_read_access = 0;

  if (0 != ret)
  {
    pr_debug("%s : read fail!\n", __func__);
    goto int_clear;
  }

  num_of_touch = buf.tt_stat & 0xf;	/* lower 4bits are used */
  cur_key = buf.tt_stat >> 4; 		/* upper 4bits are used (menu key, back key) */
  if( cur_key ) {
    // check button count
    for(i=0; i< 4; i++) {
      if( cur_key & ( 1<<i) ) {
        btn_count += 1;
      }
    }
  }

  // only button or toych event
  if( num_of_touch > 0 && btn_count  > 0 )
  {
    if( !(ts->prev_menu || ts->prev_home || ts->prev_back || ts->prev_search ) )  {
      ignore_btn_event = 1;
    } else {
       ignore_touch_event = 1;
    }
  }

  if (num_of_touch > 2 || btn_count > 2 )	/* for invalid touch input -.- */
  {
    ignore_btn_event = 0;
    ignore_touch_event = 0;
    // button timer check,
    if( ts->prev_menu || ts->prev_home ||  ts->prev_back || ts->prev_search ) {
      ignore_btn_event = 1;
    }

    /* check previous touch input and return RELEASE */
    for (i = 0; i < CYTOUCH_MAX_ID+1; i++) {
      if (ts->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED) {
        ignore_touch_event = 1;
        break;
      }
    }

    if( ignore_touch_event )  {
      del_timer(&g_cytouch_touch_timer);
      g_cytouch_touch_timer.expires = get_jiffies_64() + msecs_to_jiffies(60/*40*/);
      add_timer(&g_cytouch_touch_timer);
    }
    else if(ignore_btn_event ) {
      del_timer(&g_button_timer);
      g_button_timer.expires = get_jiffies_64() + msecs_to_jiffies(60/*40*/);
      add_timer(&g_button_timer);
    }

    if( ignore_touch_event || ignore_btn_event )
      pr_debug("%s : waiting next interrupt or time out event! <num_of_touch=%d, cur_key=%d\n", __func__, num_of_touch, btn_count);

    goto int_clear;
  }

  // button press state
  if( cur_key && ignore_btn_event == 0)
  {
    if( cur_key & MENUKEY_BIT ) {
      pr_debug("%s : MENUKEY PRESS <prev_state=%d>\n", __func__, ts->prev_menu );
      if( !ts->prev_menu ) {
        if( touch_button_enable )
          input_report_key(ts->input_dev, TOUCH_MENU, 1);
        ts->prev_menu = 1;
      }
      b_add_button_timer = TRUE;
    }
    else if(ts->prev_menu)
    {
      if( touch_button_enable )
        input_report_key(ts_global->input_dev, TOUCH_MENU, 0);
      ts->prev_menu = 0;
    }

    if( cur_key & HOMEKEY_BIT )  {
      pr_debug("%s : HOMEKEY PRESS <prev_state=%d>\n", __func__, ts->prev_home);
      if( !ts->prev_home ) {
        if( touch_button_enable )
          input_report_key(ts->input_dev, TOUCH_HOME, 1);
        ts->prev_home = 1;
      }
      b_add_button_timer = TRUE;
    }
    else if(ts->prev_home)
    {
      if( touch_button_enable )
        input_report_key(ts_global->input_dev, TOUCH_HOME, 0);
      ts->prev_home = 0;
    }

    if( cur_key & BACKKEY_BIT )  {
      pr_debug("%s : BACKKEY PRESS <prev_state=%d>\n", __func__, ts->prev_back);
      if( !ts->prev_back ) {
        if( touch_button_enable )
          input_report_key(ts->input_dev, TOUCH_BACK, 1);
        ts->prev_back = 1;
      }
      b_add_button_timer = TRUE;
    }
    else if(ts->prev_back)
    {
      if( touch_button_enable )
        input_report_key(ts_global->input_dev, TOUCH_BACK, 0);
      ts->prev_back = 0;
    }

    if( cur_key & SEARCHKEY_BIT ) {
      pr_debug("%s : SEARCHKEY PRESS <prev_state=%d>\n", __func__, ts->prev_search);
      if( !ts->prev_search )  {
        if( touch_button_enable )
          input_report_key(ts->input_dev, TOUCH_SEARCH, 1);
        ts->prev_search = 1;
      }
      b_add_button_timer = TRUE;
    }
    else if(ts->prev_search)
    {
      if( touch_button_enable )
        input_report_key(ts_global->input_dev, TOUCH_SEARCH, 0);
      ts->prev_search = 0;
    }

  	/* activate key timer to prevent unprocessed UP event */
    if (TRUE == b_add_button_timer)
    {
      g_button_timer.expires = get_jiffies_64() + msecs_to_jiffies(60/*40*/);
      add_timer(&g_button_timer);
      if( touch_button_enable )
        keypad_tsp_touch_event();
    }
    if( touch_button_enable )
      input_sync(ts->input_dev);
  }
  else if( ignore_touch_event == 0)
  {
    if( ts->prev_menu && (cur_key & MENUKEY_BIT)==0 ) {
      pr_debug("%s : release TOUCH_MENU!\n", __func__);
      if( touch_button_enable )
        input_report_key(ts->input_dev, TOUCH_MENU, 0);
      ts->prev_menu = FALSE;
      button_release_event = 1;
    }
    if( ts->prev_home && !(cur_key & HOMEKEY_BIT) ) {
      pr_debug("%s : release TOUCH_HOME!\n", __func__);
      if( touch_button_enable )
        input_report_key(ts->input_dev, TOUCH_HOME, 0);
      ts->prev_home = FALSE;
      button_release_event = 1;
    }
    if( ts->prev_back && !(cur_key & BACKKEY_BIT) ) {
      pr_debug("%s : release TOUCH_BACK!\n", __func__);
      if( touch_button_enable )
        input_report_key(ts->input_dev, TOUCH_BACK, 0);
      ts->prev_back = FALSE;
      button_release_event = 1;
    }
    if( ts->prev_search && !(cur_key & SEARCHKEY_BIT) ) {
      pr_debug("%s : release TOUCH_SEARCH!\n", __func__);
      if( touch_button_enable )
        input_report_key(ts->input_dev, TOUCH_SEARCH, 0);
      ts->prev_search = FALSE;
      button_release_event = 1;
    }

    if( button_release_event == 1 )
    {
      input_sync(ts->input_dev);
      goto int_clear;
    }

    point[0].x = (int)buf.touch1_xh << 8 | (int)buf.touch1_xl;
    point[0].y = (int)buf.touch1_yh << 8 | (int)buf.touch1_yl;
    point[0].z = (int)buf.touch1_z;
    point[0].id = (int)buf.touch12_id >> 4;

    point[1].x = (int)buf.touch2_xh << 8 | (int)buf.touch2_xl;
    point[1].y = (int)buf.touch2_yh << 8 | (int)buf.touch2_yl;
    point[1].z = (int)buf.touch2_z;
    point[1].id = (int)buf.touch12_id & 0xf;

  	/*
  	pr_debug("%d %d  ", num_of_touch, buf.touch12_id);
  	pr_debug("(%x,%x) ", buf.tt_stat&1<<7, buf.tt_stat&1<<6);
  	pr_debug("(%3d,%3d,%3d,%d) ", point[0].x, point[0].y, point[0].z, point[0].id);
  	pr_debug("(%3d,%3d,%3d,%d)\n", point[1].x, point[1].y, point[1].z, point[1].id);
  	*/

	/* update the status of current touch inputs */
	for (i = 0; i < num_of_touch; i++)
	{
#if !(defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS))
		if (abs(ts->id_stat[point[i].id].x - point[i].x) <= 1 &&
			abs(ts->id_stat[point[i].id].y - point[i].y) >= 1)
		{
			/* discard new point which is near(<+-5) from previous point. (but accept all input for multi(2) touch input) */
			if (num_of_touch < 2)
			{
				ts->id_stat[point[i].id].b_report = FALSE;	/* suppress long touch input */
			}
		}
		else
#endif
		{
			ts->id_stat[point[i].id].b_report = TRUE;
			/* update x,y,z only when new point is far enough from previous point */
			ts->id_stat[point[i].id].x = point[i].x;
			ts->id_stat[point[i].id].y = point[i].y;
			ts->id_stat[point[i].id].z = point[i].z;
		}

		ts->id_stat[point[i].id].status = CYTOUCH_ID_STAT_PRESSED;
		ts->id_stat[point[i].id].dirty = CYTOUCH_ID_STAT_NEW;
	}

	/* check previous touch input and return RELEASE if new input is not */
	/* generated for that previous input */
	for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
	{
		if (ts->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
		{
			if (ts->id_stat[i].dirty == CYTOUCH_ID_STAT_NEW)
			{
				if (ts->id_stat[i].b_report == TRUE)
				{
					pr_debug("DN(%d,%d,%d,%d)\n", ts->id_stat[i].x, ts->id_stat[i].y, ts->id_stat[i].z, i);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->id_stat[i].x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->id_stat[i].y);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->id_stat[i].z);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
					input_mt_sync(ts->input_dev);
				}
				b_add_touch_timer = TRUE;	/* to prevent unprocessed UP event */
			}
			else	/* previously pressed... so this is release... */
			{
				pr_debug("UP(%d,%d,%d,%d)\n", ts->id_stat[i].x, ts->id_stat[i].y, ts->id_stat[i].z, i);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->id_stat[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->id_stat[i].y);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->id_stat[i].z);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				ts->id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
				input_mt_sync(ts->input_dev);
			}

          touch_sync_event = 1;
		}
		ts->id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;
	}

    if( touch_sync_event )
    {
    	input_sync(ts->input_dev);
    }

	if (TRUE == b_add_touch_timer)
	{
		keypad_tsp_touch_event(); // eGroupware
		g_cytouch_touch_timer.expires = get_jiffies_64() + msecs_to_jiffies(60/*40*/);
		add_timer(&g_cytouch_touch_timer);
  	}
  }

int_clear:

//	__raw_writel(0x1<<2, S5P64XX_GROUP15_INT_PEND);		/* for level trigger... why??? */
//	__raw_writel(0x1<<2, S5P64XX_GROUP13_INT_PEND);		/* for level trigger... why??? */
	enable_irq(ts->client->irq);
}
#else
void cytouch_irq_body(struct work_struct* p_work)
{
	struct cytouch_ts_data *ts = container_of(p_work, struct cytouch_ts_data, work);

	CYTOUCH_POINT point[3] = {{0,0,0,0,0}, {0,0,0,0,0}, {0,0,0,0,0}};
	CYTOUCH_RAW_DATA buf = {0,};
	u8 num_of_touch = 0;

	int cur_key = 0;
	int b_add_touch_timer = FALSE;

	int i = 0;
	int ret = 0;

#ifdef CONFIG_CPU_FREQ		// This makes cpu work fast when user touch display
	set_dvfs_perf_level();
#endif

  del_timer(&g_cytouch_touch_timer);

  ret = cytouch_i2c_read(ts->client, CYTOUCH_REG_READ_START, (u8*)&buf, CYTOUCH_REG_READ_SIZE);

  if (0 != ret)
  {
    pr_debug("%s : read fail!\n", __func__);
    goto int_clear;
  }

  num_of_touch = buf.tt_stat & 0xf;	/* lower 4bits are used */
  cur_key = buf.tt_stat >> 6; 		/* upper 2bits are used (menu key, back key) */

	if (num_of_touch > 2)	/* for invalid touch input -.- */
	{
		pr_debug("%s : invalid input found! release all input, <num_of_touch=%d>\n", __func__, num_of_touch);
		/* check previous touch input and return RELEASE */
		for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
		{
			if (ts->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
			{
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->id_stat[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->id_stat[i].y);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->id_stat[i].z);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				ts->id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
				pr_debug("%s : lost TOUCH UP generated!\n", __func__);
			}

			ts->id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;

			input_mt_sync(ts->input_dev);
		}
		input_sync(ts->input_dev);

		goto int_clear;
	}

	point[0].x = (int)buf.touch1_xh << 8 | (int)buf.touch1_xl;
	point[0].y = (int)buf.touch1_yh << 8 | (int)buf.touch1_yl;
	point[0].z = (int)buf.touch1_z;
	point[0].id = (int)buf.touch12_id >> 4;

	point[1].x = (int)buf.touch2_xh << 8 | (int)buf.touch2_xl;
	point[1].y = (int)buf.touch2_yh << 8 | (int)buf.touch2_yl;
	point[1].z = (int)buf.touch2_z;
	point[1].id = (int)buf.touch12_id & 0xf;

  	//
  	pr_debug("%d  ", num_of_touch);
  	pr_debug("(%x,%x) ", buf.tt_stat&1<<7, buf.tt_stat&1<<6);
  	pr_debug("(%3d,%3d,%3d,%d) ", point[0].x, point[0].y, point[0].z, point[0].id);
  	pr_debug("(%3d,%3d,%3d,%d)\n", point[1].x, point[1].y, point[1].z, point[1].id);
  	//

	/* update the status of current touch inputs */
	for (i = 0; i < num_of_touch; i++)
	{
#if !defined(CONFIG_MACH_RANT3)
		if (abs(ts->id_stat[point[i].id].x - point[i].x) <= 1 &&
			abs(ts->id_stat[point[i].id].y - point[i].y) <= 1)
		{
			/* discard new point which is near(<+-5) from previous point. (but accept all input for multi(2) touch input) */
			if (num_of_touch < 2)
			{
				ts->id_stat[point[i].id].b_report = FALSE;	/* suppress long touch input */
			}
		}
		else
#endif
		{
			ts->id_stat[point[i].id].b_report = TRUE;
			/* update x,y,z only when new point is far enough from previous point */
			ts->id_stat[point[i].id].x = point[i].x;
			ts->id_stat[point[i].id].y = point[i].y;
			ts->id_stat[point[i].id].z = point[i].z;
		}

		ts->id_stat[point[i].id].status = CYTOUCH_ID_STAT_PRESSED;
		ts->id_stat[point[i].id].dirty = CYTOUCH_ID_STAT_NEW;
	}

	/* check previous touch input and return RELEASE if new input is not */
	/* generated for that previous input */
	for (i = 0; i < CYTOUCH_MAX_ID+1; i++)
	{
		if (ts->id_stat[i].status == CYTOUCH_ID_STAT_PRESSED)
		{
			if (ts->id_stat[i].dirty == CYTOUCH_ID_STAT_NEW)
			{
				if (ts->id_stat[i].b_report == TRUE)
				{
					pr_debug("DN(%d,%d,%d,%d)\n", ts->id_stat[i].x, ts->id_stat[i].y, ts->id_stat[i].z, i);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->id_stat[i].x);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->id_stat[i].y);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->id_stat[i].z);
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
					input_mt_sync(ts->input_dev);
				}
				b_add_touch_timer = TRUE;	/* to prevent unprocessed UP event */
			}
			else	/* previously pressed... so this is release... */
			{
				pr_debug("UP(%d,%d,%d,%d)\n", ts->id_stat[i].x, ts->id_stat[i].y, ts->id_stat[i].z, i);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, ts->id_stat[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->id_stat[i].y);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, ts->id_stat[i].z);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
				ts->id_stat[i].status = CYTOUCH_ID_STAT_RELEASED;
				input_mt_sync(ts->input_dev);
			}
		}

		ts->id_stat[i].dirty = CYTOUCH_ID_STAT_DIRTY;
	}

	input_sync(ts->input_dev);

	if (TRUE == b_add_touch_timer)
	{
		g_cytouch_touch_timer.expires = get_jiffies_64() + msecs_to_jiffies(40);
		add_timer(&g_cytouch_touch_timer);
  	}

int_clear:
//	__raw_writel(0x1<<2, S5P64XX_GROUP15_INT_PEND);		/* for level trigger... why??? */
//	__raw_writel(0x1<<2, S5P64XX_GROUP13_INT_PEND);		/* for level trigger... why??? */
	enable_irq(ts->client->irq);
}
#endif


irqreturn_t cytouch_irq_handler(int irq, void* p_dev_id)
{
	struct cytouch_ts_data *ts = p_dev_id;
	CYTOUCH_RAW_DATA buf = {0,};

	disable_irq_nosync(ts->client->irq);

	//moon
	pr_debug("%s\n", __func__);

	//schedule_work(&cytouch_irq_wq);
	queue_work(cytouch_irq_queue, &ts->work);

	return IRQ_HANDLED;
}

static void cytouch_print_ver(void)
{
	u8 buf[3] = {0,};
	int retry = 4;
	int err;

	read_tsp_version = 1;
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	while(retry--) {
	    err = cytouch_i2c_read(ts_global->client, CYTOUCH_REG_VENDOR_ID, buf, 3);

            if(err >= 0) {
#if defined (CONFIG_MACH_RANT3) || defined (CONFIG_MACH_VINO) || defined (CONFIG_MACH_GIOS)
                if(buf[0] != cytma340_new_vendor_id || buf[1] != cytma340_new_module_id) {
                    if(retry == 2) {
                        cytouch_hw_reset();
                        mdelay(400);
                    }
                    if(retry != 0 ) continue;
                }
#endif /* CONFIG_MACH_RANT3 */
                break;
            }
	}

	if(err >= 0)
#else
	if (0 == cytouch_i2c_read(ts_global->client, CYTOUCH_REG_VENDOR_ID, buf, 3))
#endif
	{
		g_vendor_id = buf[0];
		g_module_id = buf[1];
		g_fw_ver = buf[2];

		printk("%s :Vendor ID : 0x%x, Module ID : 0x%x, FW Ver : 0x%x\n", __func__, buf[0], buf[1], buf[2]);
	}
	else
	{
		g_vendor_id = g_module_id = g_fw_ver = -1;
		printk("%s : Can't find vendor id, module id, fw ver!\n", __func__);
	}

	read_tsp_version = 0;
}

static void cytouch_calibrate(void)
{
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	u8 buf = 0x00;
#else
	u8 buf = 0x84;
#endif

	if (0 == cytouch_i2c_write(ts_global->client, CYTOUCH_REG_CALIBRATE, buf))
	{
		mdelay(TSP_POWERON_DELAY_TIME);
		if (0 == cytouch_i2c_read(ts_global->client, CYTOUCH_REG_CALIBRATE, &buf, 1))
		{
			if (0 == buf);
			{
				printk("%s : SUCCEED!\n", __func__);
				return;
			}
		}
	}

	printk("%s : FAILED!\n", __func__);
}

static void cytouch_lpm_mode(void)
{
    u8 buf[3] = {0,};

    if (0 == cytouch_i2c_write(ts_global->client, 00, 0x94)) // set LPM mode
    {
        //mdelay(200);
        if (0 == cytouch_i2c_read(ts_global->client, 00, &buf, 1))
            pr_debug("%s : Device Mode! : 0x%x\n", __func__, buf[0]);

        if (0 == cytouch_i2c_write(ts_global->client, 0x1E, 0x32)) // set LPM timeout
        {
            //mdelay(200);
            if (0 == cytouch_i2c_read(ts_global->client, 0x1D, buf, 3))
                pr_debug("%s :ACT_INTRVL : 0x%x, TCH_TMOUT : 0x%x, LP_INTRVL : 0x%x\n", __func__, buf[0], buf[1], buf[2]);
        }
        else
        {
            pr_debug("%s : LPM timeout fail!\n", __func__);
        }
    }
    else
    {
        pr_debug("%s : LPM mode fail!\n", __func__);
    }

    cytouch_calibrate();
}

#ifdef CYTSP_FWUPG_ENABLE
static void cytouch_upgrade_fw(void)
{
  int retry_count = 0;
  int download_ret = 0;
#if (defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS))
#if 0 // new paterrn upgrade is delete
  int new_patern = 0;

  if( g_vendor_id == 0 || g_module_id == 0 || g_fw_ver == 0)
  {
    //new_patern = 1;
  }

  if( CYTOUCH_NEW_PATERN )
  {
    if (g_vendor_id == cytma340_new_vendor_id_np || new_patern == 1 )
    {
      if (g_module_id < cytma340_new_module_id_np  || (g_module_id == cytma340_new_module_id_np && g_fw_ver < cytma340_new_fw_ver_np   ))
      {
        pr_debug("%s : Firmware Upgrade (%d -> %d)\n", __func__, g_fw_ver, cytma340_new_fw_ver);
        for (retry_count = 0; retry_count < 3; retry_count++)
        {
          download_ret = tma340_frimware_update();
          if( download_ret == 0 ) break;
        }

        /* reset phone */
        schedule_work(&restart_work);
      }
    }
  }
  else
#endif
  {
    if (g_vendor_id == cytma340_new_vendor_id || g_vendor_id == 0) {
      if ( (g_module_id < cytma340_new_module_id  || g_module_id == 0)
        || (g_module_id == cytma340_new_module_id && g_fw_ver < cytma340_new_fw_ver  || g_fw_ver == 0 ))
      {
        printk("%s : Firmware Upgrade (%d -> %d)\n", __func__, g_fw_ver, cytma340_new_fw_ver);
        for (retry_count = 0; retry_count < 3; retry_count++) {
          download_ret = tma340_frimware_update();
          if( download_ret == 0 ) break;
        }

        if( bTargetEraseState == 1 && download_ret != 0 ) {
          bTargetEraseState = 0;
          for (retry_count = 0; retry_count < 3; retry_count++)
          {
            download_ret = tma340_frimware_update();
            if( download_ret == 0 ) break;
          }
        }

        /* reset phone */
        if( download_ret == 0 )
          schedule_work(&restart_work);
      }
    }
  }
#else
  int need_fw_update = 0;

  if (g_vendor_id == cytma340_new_vendor_id || g_vendor_id == 0)
    if (g_module_id == cytma340_new_module_id || g_module_id == 0)
      if (g_fw_ver < cytma340_new_fw_ver || g_fw_ver == 0)
        need_fw_update = 1;

  if (need_fw_update)
  {
    printk("%s : Firmware Upgrade (%d -> %d)\n", __func__, g_fw_ver, cytma340_new_fw_ver);

#ifdef TSP_SDCARD_INDICATE
    gpio_set_value(GPIO_LED_B, 1);
    gpio_set_value(GPIO_LED_R, 0);
#endif

    for (retry_count = 0; retry_count < 3; retry_count++)
    {
        download_ret = tma340_frimware_update();
        if( download_ret == 0 ) break;
    }

#ifdef TSP_SDCARD_INDICATE
		if( download_ret == 0 )
        		gpio_set_value(GPIO_LED_B, 0);

    		gpio_set_value(GPIO_LED_R, 1);
#endif

	 /* reset phone */
   if(download_ret == 0)
  	    schedule_work(&restart_work);
  }

#endif
}
#endif

#ifdef TSP_SDCARD_UPDATE
static int do_tsp_firmware_load(const char *fn, char **fp)
{
	struct file* filp;
	long l = 0;
	char *dp;
	loff_t pos;

	filp = filp_open(fn, 0, 0);
	if (IS_ERR(filp))
	{
		printk(KERN_INFO "Unable to load '%s'.\n", fn);
		return 0;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	if (l <= 0 || l > (512*1024))
	{
		printk(KERN_INFO "Invalid firmware '%s'\n", fn);
		filp_close(filp, current->files);
		return 0;
	}
	dp = vmalloc(l);
	if (dp == NULL)
	{
		printk(KERN_INFO "Out of memory loading '%s'.\n", fn);
		filp_close(filp, current->files);
		return 0;
	}
	pos = 0;
	if (vfs_read(filp, dp, l, &pos) != l)
	{
		printk(KERN_INFO "Failed to read '%s'.\n", fn);
		vfree(dp);
		filp_close(filp, current->files);
		return 0;
	}
	filp_close(filp, current->files);
	*fp = dp;
	return (int) l;
}

int tsp_firmware_load(const char *fn, char **fp)
{
	int r;
	mm_segment_t fs = get_fs();

	set_fs(get_ds());
	r = do_tsp_firmware_load(fn, fp);
	set_fs(fs);
	return r;
}

int tsp_firmware_unload(char *fp)
{
    if(fp)
        vfree(fp);

    return 0;
}

void firmware_update_from_sdcard()
{
	char *after;
	int retry_count = 0;
	int download_ret = 0;
	size_t firmware_size = 0;

#if defined(CYTSP_WDOG_ENABLE)
    cytouch_pause_wdog();
#endif

#ifdef TSP_SDCARD_INDICATE
	gpio_set_value(GPIO_LED_B, 1);
	gpio_set_value(GPIO_LED_R, 0);
#endif

#if defined(CONFIG_MACH_VINO)
  if((firmware_size = tsp_firmware_load("/sdcard/VINO_E_R3_JHTL.hex", &pfirmware)) > 1024)
#elif defined(CONFIG_MACH_GIOS)
  if((firmware_size = tsp_firmware_load("/sdcard/gios_tsp_firmware.hex", &pfirmware)) > 1024)  
#else
	//if((firmware_size = tsp_firmware_load("/sdcard/RANT3_hex_HW01_SW03_20101222_1121_ESD_Flag.hex", &pfirmware)) > 1024)
	//if((firmware_size = tsp_firmware_load("/sdcard/RANT3_TEST_HW01_SW04.hex", &pfirmware)) > 1024)
  if((firmware_size = tsp_firmware_load("/sdcard/rant3_tsp_firmware.hex", &pfirmware)) > 1024)
#endif
	{
		printk("[TOUCH] %02x %02x %02x %02x \n", pfirmware[0], pfirmware[1], pfirmware[2], pfirmware[3]);
		printk("[TOUCH] %02x %02x %02x %02x \n", pfirmware[4], pfirmware[5], pfirmware[6], pfirmware[7]);
		fromsdcard = true;

		disable_irq(IRQ_TOUCH_INT);
		for (retry_count = 0; retry_count < 3; retry_count++)
		{
			download_ret = tma340_frimware_update();

			if( download_ret == 0 ) break;
		}
		printk("[TSP] Firmware Update complete!!\n");

		cytouch_hw_set_pwr(CYTOUCH_PWROFF); mdelay(10);
		cytouch_hw_set_pwr(CYTOUCH_PWRON);	mdelay(TSP_POWERON_DELAY_TIME);

		enable_irq(IRQ_TOUCH_INT);
#ifdef TSP_SDCARD_INDICATE
		if( download_ret == 0 )
        		gpio_set_value(GPIO_LED_B, 0);

    		gpio_set_value(GPIO_LED_R, 1);
#endif

	}
	else
	{
		printk("[TSP] Firmware File Not find!!\n");
	}

#if defined(CYTSP_WDOG_ENABLE)
    cytouch_resume_wdog();
#endif

    if(download_ret == 0 )
        schedule_work(&restart_work);
}
#endif


static void cytouch_register_irq(void)
{
  int ret;

  if (ts_global->client->irq) {
    set_irq_type(ts_global->client->irq, IRQ_TYPE_LEVEL_LOW);

    ret = request_irq(ts_global->client->irq, cytouch_irq_handler, IRQF_DISABLED , "cytma340 irq", ts_global);
    if (ret == 0)
    {
      ts_global->use_irq = 1;
    }
    else
    {
      printk("%s request_irq failed\n", __func__);
    }
  }
}


static ssize_t firmware1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[TSP] Cypress Firmware Ver. %x:%x:%x\n", g_vendor_id, g_module_id, g_fw_ver);

	return sprintf(buf, "%x:%x:%x\n", g_vendor_id, g_module_id, g_fw_ver);
}

static ssize_t firmware1_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  char *after;
  int retry_count = 0;
  int download_ret = 0;

  if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0)
  {
#if 0//!defined(CONFIG_MACH_VINO)  
  	firmware_update_from_sdcard();
#else
#if defined(CYTSP_WDOG_ENABLE)
    cytouch_pause_wdog();
#endif
    //FMT_WSHAN 20101028 ...Firmware Download, irq disable
    disable_irq(IRQ_TOUCH_INT);
    for (retry_count = 0; retry_count < 3; retry_count++)
    {
      download_ret = tma340_frimware_update();

      if( download_ret == 0 )
        break;
    }
    printk("[TSP] Firmware Update\n");

    cytouch_hw_set_pwr(CYTOUCH_PWROFF);
    mdelay(10);
    cytouch_hw_set_pwr(CYTOUCH_PWRON);
    mdelay(TSP_POWERON_DELAY_TIME);

    enable_irq(IRQ_TOUCH_INT);// irq
#if defined(CYTSP_WDOG_ENABLE)
    cytouch_resume_wdog();
#endif

#if defined(CONFIG_MACH_RANT3)  
if(download_ret == 0 )
	schedule_work(&restart_work);
#endif

#endif

  }
  else
    printk("[TSP] operate nothing\n");

  return size;
}

static DEVICE_ATTR(firmware1, S_IRUGO | S_IWUSR | S_IWGRP, firmware1_show, firmware1_store);

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
static ssize_t touchbutton_ctrl_store(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned int touchbutton_state;
  sscanf(buf,"%d\n",&touchbutton_state);
  printk(" touchbutton_state set to : %d \n",touchbutton_state);
  touch_button_enable = (unsigned int)touchbutton_state;
  return sprintf(buf,"%d",touchbutton_state);
}

static ssize_t touchbutton_ctrl_show(struct device *dev,struct device_attribute *attr,char *buf)
{
  unsigned int touchbutton_state = touch_button_enable;
  //sprintf(buf,"%d",touchbutton_state);
  return sprintf(buf,"%d",touchbutton_state);
}

static DEVICE_ATTR(touchbutton_ctrl, S_IRUGO | S_IWUSR | S_IWGRP, touchbutton_ctrl_show, touchbutton_ctrl_store);

#endif


static void cytouch_hw_reset(void)
{
	pr_debug("%s : Start!\n", __func__);

	if (cytouch_hw_set_pwr(CYTOUCH_PWROFF) == FALSE)
	{
		goto CYTOUCH_HW_RESET_FAIL;
	}

	mdelay(10);

	if (cytouch_hw_set_pwr(CYTOUCH_PWRON) == FALSE)
	{
		goto CYTOUCH_HW_RESET_FAIL;
	}

	mdelay(TSP_POWERON_DELAY_TIME);

	return;

CYTOUCH_HW_RESET_FAIL:
	pr_debug("%s : End but FAIL!!!!\n", __func__);

}


static struct i2c_device_id cytouch_idtable[] =
{
	{"cytma340", 0},
};

MODULE_DEVICE_TABLE(i2c, cytouch_idtable);


static int cytouch_i2c_write(struct i2c_client* p_client, u8 reg, u8 data)
{
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = data;

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(0x%x) with data(0x%x)\n", __func__, reg, data);
		return -EIO;
	}

	printk("%s : reg(0x%x) with data(0x%x)\n", __func__, reg, data);

	return 0;
}

static int cytouch_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{
	struct i2c_msg msg;
	int err;
	int retry = 5;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	while(retry--)
	{
        	msg.addr = p_client->addr;
        	msg.flags = 0;
        	msg.len = 1;
        	msg.buf = &reg;

        	err = i2c_transfer(p_client->adapter, &msg, 1);

        	if(err >= 0 || read_tsp_version == 1)
        	    break;

        	if(retry == 0)
        	{
        		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
        		cytouch_hw_reset();
        		return -EIO;
        	}
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	//printk("%s read success : reg(0x%x) data(0x%x)\n", __func__, reg, buf[0]);

	return 0;
}

static int cytouch_probe(struct i2c_client* p_client, const struct i2c_device_id* p_id)
{
  struct cytouch_ts_data *ts;
  int ret;

  printk("Cypress Touch Sensor Probe!\n");

  if (i2c_adapter_id(p_client->adapter) != 2)
  {
    printk("Cypreess Touch not on this bus(%x)\n", p_client->adapter->nr);
  }
/*
  ret = cytouch_hw_set_pwr(CYTOUCH_PWROFF);
  if( !ret )
  {
    printk(KERN_ERR "%s: vreg disable failed (%d)\n",__func__, ret);
    return -EIO;
  }

  mdelay(10);
*/
  ret = cytouch_hw_set_pwr(CYTOUCH_PWRON);
  if ( !ret )
  {
    printk(KERN_ERR "%s: vreg enable failed (%d)\n",__func__, ret);
    return -EIO;
  }

  mdelay(TSP_POWERON_DELAY_TIME);

  ts = kzalloc(sizeof(*ts), GFP_KERNEL);
  if (ts == NULL) {
    ret = -ENOMEM;
    goto err_alloc_data_failed;
  }
  INIT_WORK(&ts->work, cytouch_irq_body);
  ts->client = p_client;
  i2c_set_clientdata(p_client, ts);

  ts_global = ts;

  ts->input_dev = input_allocate_device();

  if (NULL == ts->input_dev)
  {
    ret = -ENOMEM;
    printk(KERN_ERR "cytouch_probe: Failed to allocate input device\n");
    goto err_input_dev_alloc_failed;;
  }

  ts->input_dev->name = "sec_touchscreen";

  set_bit(EV_SYN, ts->input_dev->evbit);
  set_bit(EV_KEY, ts->input_dev->evbit);
  set_bit(BTN_TOUCH, ts->input_dev->keybit);
  set_bit(EV_ABS, ts->input_dev->evbit);

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS) //FMT_WSHAN 101021
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 320, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 480, 0, 0);
#else
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 240, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 320, 0, 0);
#endif

  input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

  ts->input_dev->keycode = cytouch_keycodes;

  /* set up two keys (menu & back) */
  input_set_capability(ts->input_dev, EV_KEY, ((int*)ts->input_dev->keycode)[0]);
  input_set_capability(ts->input_dev, EV_KEY, ((int*)ts->input_dev->keycode)[1]);
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
  input_set_capability(ts->input_dev, EV_KEY, ((int*)ts->input_dev->keycode)[2]);
  input_set_capability(ts->input_dev, EV_KEY, ((int*)ts->input_dev->keycode)[3]);
#endif

  ret = input_register_device(ts->input_dev);
  if (ret)
  {
    printk("%s : unable to register input device!\n", __func__);
    goto err_input_register_device_failed;
  }

  cytouch_print_ver();

  sec_class = class_create(THIS_MODULE, "sec");
  gp_ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
  if (IS_ERR(gp_ts_dev))
  {
    printk("Failed to create device(ts)!\n");
  }
  if (device_create_file(gp_ts_dev, &dev_attr_firmware1) < 0)
  {
    pr_err("Failed to create device file(%s)!\n", dev_attr_firmware1.attr.name);
  }

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
  if (device_create_file(gp_ts_dev, &dev_attr_touchbutton_ctrl) < 0)
  {
    printk("Failed to create device file(%s)!\n", dev_attr_touchbutton_ctrl.attr.name);
  }
#endif

//#ifdef CYTSP_FWUPG_ENABLE
//#if !defined(CONFIG_MACH_VINO)
//  cytouch_upgrade_fw();
//#endif
//#endif

#ifdef CYTSP_BOOT_CALIBRATE
#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
 // cytouch_calibrate();
#else
    if(g_fw_ver == -1)
        cytouch_calibrate();
#endif
#endif

  cytouch_register_irq();
  cytouch_init_rel_timer();

#ifdef CYTSP_WDOG_ENABLE
  cytouch_init_wdog();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
  ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
  ts->early_suspend.suspend = cytouch_early_suspend;
  ts->early_suspend.resume = cytouch_late_resume;
  register_early_suspend(&ts->early_suspend);
#endif

  return 0;

  err_input_register_device_failed:
    input_free_device(ts->input_dev);

  err_input_dev_alloc_failed:
    kfree(ts);
  err_alloc_data_failed:
  err_check_functionality_failed:
    return ret;
}

static int cytouch_remove(struct i2c_client *p_client)
{
	struct cytouch_ts_data *ts = i2c_get_clientdata(p_client);


#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

	if (ts->use_irq)
		free_irq(p_client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int cytouch_suspend(struct i2c_client* p_client, pm_message_t msg)
{
	struct cytouch_ts_data *ts = i2c_get_clientdata(p_client);
	int ret;

	// del if (ts->use_irq)
	{
		disable_irq(p_client->irq);
	}

#ifdef CYTSP_WDOG_ENABLE
	cytouch_pause_wdog();
#endif

  gpio_tlmm_config(GPIO_CFG(TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	free_irq(p_client->irq, ts); // struct pointer

	cytouch_release_all();	/* release previous pressed key and touch */

	cytouch_hw_set_pwr(CYTOUCH_PWROFF);

  gpio_set_value(TSP_SCL, 0);
  gpio_set_value(TSP_SDA, 0);

	printk("%s \n", __func__);

	return 0;
}

static int cytouch_resume(struct i2c_client *client)
{
//struct cytouch_ts_data *ts = i2c_get_clientdata(client);
  gpio_tlmm_config(GPIO_CFG(TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
/*
	cytouch_hw_set_pwr(CYTOUCH_PWRON);
	mdelay(10);
	cytouch_hw_set_pwr(CYTOUCH_PWROFF);
	mdelay(10);
*/
	cytouch_hw_set_pwr(CYTOUCH_PWRON);
	//Rossi bhmoon 2010.7.21
	// During music playing, when LCD is Off and On, then music is dropped
	//mdelay(400);
	msleep(TSP_POWERON_DELAY_TIME);

      gpio_set_value(TSP_SCL, 1);
      gpio_set_value(TSP_SDA, 1);

	cytouch_register_irq();

//	enable_irq(IRQ_TOUCH_INT);
#ifdef CYTSP_WDOG_ENABLE
	cytouch_resume_wdog();
#endif

	printk("%s \n",__func__);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cytouch_early_suspend(struct early_suspend* p_h)
{
	//struct cytouch_ts_data *ts;
	//ts = container_of(p_h, struct cytouch_ts_data, early_suspend);
	cytouch_suspend(ts_global->client, PMSG_SUSPEND);
}

static void cytouch_late_resume(struct early_suspend* p_h)
{
	//struct cytouch_ts_data *ts;
	//ts = container_of(p_h, struct cytouch_ts_data, early_suspend);
	cytouch_resume(ts_global->client);
}

static struct dev_pm_ops cytouch_ts_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
  .suspend	= cytouch_suspend,
  .resume		= cytouch_resume,
#endif
};
#endif


static struct i2c_driver cytouch_driver =
{
	.driver = {
		.name = "cytma340",
		.owner = THIS_MODULE,
#if 0//def CONFIG_PM
    .pm = &cytouch_ts_pm_ops,
#endif
	},
	.id_table = cytouch_idtable,
	.class = I2C_CLASS_HWMON,
	.probe = cytouch_probe,
	.remove = cytouch_remove,
};

static int __init cytouch_init(void)
{
	int ret;

	cytouch_irq_queue = create_singlethread_workqueue("cytma340_queue");
	if (!cytouch_irq_queue)
  {
    printk(KERN_ERR "%s: cytouch_irq_queue failed (%d)\n", __func__, ret);
		return -ENOMEM;
  }

	ret = i2c_add_driver(&cytouch_driver);
	if (ret) {
		printk(KERN_ERR "%s: i2c_add_driver enable failed (%d)\n", __func__, ret);
		return -EIO;
	}

	return ret;
}

static void __exit cytouch_exit(void)
{
	i2c_del_driver(&cytouch_driver);
	if (cytouch_irq_queue)
		destroy_workqueue(cytouch_irq_queue);
}

module_init(cytouch_init);
module_exit(cytouch_exit);

MODULE_AUTHOR("Jaemin Yoo");
MODULE_DESCRIPTION("Cypress Touch Sensor Driver");
MODULE_LICENSE("GPL");

#ifdef CYTSP_FWUPG_ENABLE

#define TX_ON

#ifndef UInt32
#define UInt32 u32
#endif

#ifndef UInt16
#define UInt16 u16
#endif

#ifndef UInt8
#define UInt8 u8
#endif

#define I2C_GPIO_SDA	TSP_SDA
#define I2C_GPIO_SCL	TSP_SCL
#define I2C_GPIO_IRQ	TSP_INT

#define GPIO_LEVEL_HIGH   1
#define GPIO_LEVEL_LOW    0

//#define I2C_GPIO_IRQ	S5P64XX_GPG2(2)		/* GPG2_2 */
//#define I2C_GPIO_IRQ	IRQ_EINT_GROUP(15,2)	/* GPG2_2 */

#if !defined(CONFIG_MACH_RANT3) //&& !defined(CONFIG_MACH_VINO)
#define I2C_SET_SCL_GPIO() 		gpio_direction_input(I2C_GPIO_SCL)		// gpio input
#define I2C_CLR_SCL_GPIO() 		gpio_direction_output(I2C_GPIO_SCL, 0)	// gpio output
#define I2C_SET_SCL_GPIO_LOW() 	gpio_direction_output(I2C_GPIO_SCL, 0)		// gpio output low
#define I2C_SET_SCL_GPIO_HIGH() gpio_direction_output(I2C_GPIO_SCL, 1)		// gpio output high

#define I2C_SET_SDA_GPIO() 		gpio_direction_input(I2C_GPIO_SDA)		// gpio input
#define I2C_CLR_SDA_GPIO() 		gpio_direction_output(I2C_GPIO_SDA, 1)
#define I2C_SET_SDA_GPIO_LOW() 	gpio_direction_output(I2C_GPIO_SDA, 0)		// gpio output low
#define I2C_SET_SDA_GPIO_HIGH() gpio_direction_output(I2C_GPIO_SDA, 1)	// gpio output high
#else
#define I2C_SET_SCL_GPIO() 		do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE); } while(0)		// gpio input
#define I2C_CLR_SCL_GPIO() 		do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE); } while(0)	// gpio output
#define I2C_SET_SCL_GPIO_LOW() 	do { gpio_set_value(I2C_GPIO_SCL, GPIO_LEVEL_LOW); } while(0)		// gpio output low
#define I2C_SET_SCL_GPIO_HIGH() do { gpio_set_value(I2C_GPIO_SCL, GPIO_LEVEL_HIGH); } while(0)		// gpio output high
#define I2C_SET_SCL_GPIO_INPUT()    do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE); } while(0)		// gpio input
#define I2C_SET_SCL_GPIO_OUTPUT()   do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE); } while(0)		// gpio input

#define I2C_SET_SDA_GPIO() 		do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE); } while(0)		// gpio input
#define I2C_CLR_SDA_GPIO() 		do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE); } while(0)	// gpio output
#define I2C_SET_SDA_GPIO_LOW() 	do { gpio_set_value(I2C_GPIO_SDA, GPIO_LEVEL_LOW); } while(0)		// gpio output low
#define I2C_SET_SDA_GPIO_HIGH() do { gpio_set_value(I2C_GPIO_SDA, GPIO_LEVEL_HIGH); } while(0)		// gpio output high
#define I2C_SET_SDA_GPIO_INPUT()    do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),GPIO_CFG_ENABLE); } while(0)		// gpio input
#define I2C_SET_SDA_GPIO_OUTPUT()   do { gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE); } while(0)		// gpio input
#endif

#define I2C_READ_SDA_GPIO() 	gpio_get_value(I2C_GPIO_SDA)
#define I2C_READ_SCL_GPIO() 	gpio_get_value(I2C_GPIO_SCL)


#define TARGET_DATABUFF_LEN		128

#define NUM_BANKS				1
#define BLOCKS_PER_BANK			256
#define SECURITY_BYTES_PER_BANK	64

// The following are defines for error messages from the ISSP program.
#define PASS           0
// PASS is used to indicate that a function completed successfully.
#define ERROR         -1
// ERROR is a generic failure used within lower level functions before the
// error is reported.  This should not be seen as an error that is reported
// from main.
#define INIT_ERROR     1
// INIT_ERROR means a step in chip initialization failed.
#define SiID_ERROR     2
// SiID_ERROR means that the Silicon ID check failed. This happens if the
// target part does not match the device type that the ISSP program is
// configured for.
#define ERASE_ERROR    3
// ERASE_ERROR means that the bulk erase step failed.
#define BLOCK_ERROR    4
// BLOCK_ERROR means that a step in programming a Flash block or the verify
// of the block failed.
#define VERIFY_ERROR   5
// VERIFY_ERROR means that the checksum verification failed.
#define SECURITY_ERROR 6
// SECURITY_ERROR means that the write of the security information failed.
#define STATUS_ERROR 7

#define CHECKSUM_ERROR 8


#define DELAY_M    1
#define DELAY_B    3
#if defined(CONFIG_MACH_RANT3)
#define TRANSITION_TIMEOUT     (65535*20)
#elif defined(CONFIG_MACH_VINO)
#define TRANSITION_TIMEOUT     (65535*40)
#elif defined(CONFIG_MACH_GIOS)
#define TRANSITION_TIMEOUT     (65535*40)
#else
#define TRANSITION_TIMEOUT     (65535*10)
#endif
#define XRES_CLK_DELAY    ((63 - DELAY_B) / DELAY_M)
#define POWER_CYCLE_DELAY ((150 - DELAY_B) / DELAY_M)
#define DELAY100us        ((100 - DELAY_B) / DELAY_M)

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)   //FMT_WSHAN 20101028
unsigned char target_id_v[] = {0x05, 0x9A};     //ID for CY8CTMA340_48LQXI-01
#else
unsigned char target_id_v[] = {0x05, 0x96};     //ID for CY8CTMA340_36LQXI
#endif

const unsigned int num_bits_checksum = 418;
const unsigned char checksum_v[] =
{
	0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF4, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x00, 0x7D, 0xE0,
    0x0F, 0xF7, 0xC0, 0x07, 0xDF, 0x28, 0x1F, 0x7D, 0x18, 0x7D,
    0xFE, 0x25, 0xC0
};

const unsigned char read_status[] =
{
	0xBF, 0x00, 0x80
};


const unsigned int num_bits_id_setup_1 = 616;
const unsigned char id_setup_1[] =
{
	0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0xEE, 0x21, 0xF7,
    0xF0, 0x27, 0xDC, 0x40,	0x9F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xE7, 0xC1,	0xD7, 0x9F, 0x20, 0x7E, 0x3F, 0x9D, 0x78, 0xF6,
	0x21, 0xF7, 0xB8, 0x87, 0xDF, 0xC0, 0x1F, 0x71,	0x00, 0x7D,
	0xC0, 0x07, 0xF7, 0xB8, 0x07, 0xDE,	0x80, 0x7F, 0x7A, 0x80,
	0x7D, 0xEC, 0x01, 0xF7,	0x80, 0x4F, 0xDF, 0x00, 0x1F, 0x7C,
	0xA0, 0x7D,	0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_id_setup_2 = 418;
const unsigned char id_setup_2[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF4, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x00, 0x7D, 0xE0,
    0x0D, 0xF7, 0xC0, 0x07, 0xDF, 0x28, 0x1F, 0x7D, 0x18, 0x7D,
    0xFE, 0x25, 0xC0
};

const unsigned int num_bits_tsync_enable = 110;
const unsigned char tsync_enable[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0xDE, 0xE0, 0x1C
};
const unsigned int num_bits_tsync_disable = 110;
const unsigned char tsync_disable[] =
{
    0xDE, 0xE2, 0x1F, 0x71, 0x00, 0x7D, 0xFC, 0x01, 0xF7, 0x00,
    0x1F, 0xDE, 0xE0, 0x1C
};


#if 0
const unsigned int num_bits_set_block_num = 33;
const unsigned char set_block_num[] =
{
    0xDE, 0xE0, 0x1E, 0x7D, 0x00, 0x70
};
#else
const unsigned int num_bits_set_block_num = 11;
const unsigned char set_block_num[] =
{
    0x9F, 0x40
};
#endif

const unsigned int num_bits_set_block_num_end = 3;		//PTJ: this selects the first three bits of set_block_num_end
const unsigned char set_block_num_end = 0xE0;

const unsigned int num_bits_read_write_setup = 66;		//PTJ:
const unsigned char read_write_setup[] =
{
    0xDE, 0xF0, 0x1F, 0x78, 0x00, 0x7D, 0xA0, 0x03, 0xC0
};

const unsigned int num_bits_my_verify_setup = 440;
const unsigned char verify_setup[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA8, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x0F, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_erase = 396;		//PTJ: erase with TSYNC Enable and Disable
const unsigned char erase[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x85, 0xFD, 0xFC, 0x01, 0xF7,
    0x10, 0x07, 0xDC, 0x00, 0x7F, 0x7B, 0x80, 0x7D, 0xE0, 0x0B,
    0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F, 0x7B, 0x04, 0x7D, 0xF0,
    0x01, 0xF7, 0xC9, 0x87, 0xDF, 0x48, 0x1F, 0x7F, 0x89, 0x70
};

const unsigned int num_bits_secure = 440;		//PTJ: secure with TSYNC Enable and Disable
const unsigned char secure[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA0, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x27, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned int num_bits_program_and_verify = 440;		//PTJ: length of program_block[], not including zero padding at end
const unsigned char program_and_verify[] =
{
    0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00,
    0x1F, 0x9F, 0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF7, 0x01, 0xF7,
    0xF0, 0x07, 0xDC, 0x40, 0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01,
    0xF6, 0xA0, 0x0F, 0xDE, 0x80, 0x7F, 0x7A, 0x80, 0x7D, 0xEC,
    0x01, 0xF7, 0x80, 0x57, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
    0xF4, 0x61, 0xF7, 0xF8, 0x97
};

const unsigned char read_id_v[] =
{
    0xBF, 0x00, 0xDF, 0x90, 0x00, 0xFE, 0x60, 0xFF, 0x00
};

const unsigned char    write_byte_start = 0x90;			//PTJ: this is set to SRAM 0x80
const unsigned char    write_byte_end = 0xE0;

const unsigned char    num_bits_wait_and_poll_end = 40;
const unsigned char    wait_and_poll_end[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char read_checksum_v[] =
{
    0xBF, 0x20, 0xDF, 0x80, 0x80
};


const unsigned char read_byte_v[] =
{
    0xB0, 0x80
};


#ifdef TX_ON

void UART_PutChar(char ch)
{
	printk("%c", ch);
}

void UART_PutString(char* str)
{
	printk("%s\n", str);
}

#define UART_PutCRLF(x)	do { ; } while(0)

void UART_PutHexHalf(char ch)
{
    if(ch >=10)
        UART_PutChar(ch + 'A'-10);
    else
        UART_PutChar(ch + '0');
}

void UART_PutHexByte(unsigned char ch)
{
    UART_PutHexHalf(ch >> 4);
    UART_PutHexHalf(ch & 0x0f);
}

void UART_PutHexWord(unsigned int ch)
{
    UART_PutHexByte(ch>>8);
    UART_PutHexByte(ch&0xff);
}

#endif


/***********************
*
* Touchpad Tuning APIs
*
************************/

void TchDrv_DownloadVddSetHigh(void)
{
	int ret;

	//pr_debug("%s\n", __func__);

	vreg_touch = vreg_get(NULL, "ldo8"); /* VTOUCH_3.0V */

	ret = vreg_enable(vreg_touch);
	if (ret) {
		//printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
		return -EIO;
	}
	else {
		//printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
	}

#if 0
    if (TRUE != Set_MAX8998_PM_REG(ELDO14, 1))
    {
        pr_debug("%s : Error! Can't turn on VTOUCH3.0V\n", __func__);
    }
    if (TRUE != Set_MAX8998_PM_REG(ELDO13, 1))
    {
        pr_debug("%s : Error! Can't turn on VTOUCH2.8V\n", __func__);
    }
#endif
}

void TchDrv_DownloadVddSetLow(void)
{
	int ret;

	//pr_debug("%s\n", __func__);

	vreg_touch = vreg_get(NULL, "ldo8"); /* VTOUCH_3.0V */

	ret = vreg_disable(vreg_touch);
	if (ret) {
		//printk(KERN_ERR "%s: vreg_touch enable failed (%d)\n", __func__, ret);
		return -EIO;
	}
	else {
		//printk(KERN_INFO "%s: vreg_touch enable success!\n", __func__);
	}

#if 0
    if (TRUE != Set_MAX8998_PM_REG(ELDO14, 0))
    {
        pr_debug("%s : Error! Can't turn off VTOUCH3.0V\n", __func__);
    }
    if (TRUE != Set_MAX8998_PM_REG(ELDO13, 0))
    {
        pr_debug("%s : Error! Can't turn off VTOUCH2.8V\n", __func__);
	}
#endif
}

void TchDrv_DownloadIntSetHigh(void)
{
	gpio_set_value(I2C_GPIO_IRQ, GPIO_LEVEL_HIGH);
}

void TchDrv_DownloadIntSetLow(void)
{
	gpio_set_value(I2C_GPIO_IRQ, GPIO_LEVEL_LOW);
}

void TchDrv_DownloadDisableIRQ(void)
{
	disable_irq(I2C_GPIO_IRQ);
}

void TchDrv_DownloadEnableIRQ(void)
{
	enable_irq(I2C_GPIO_IRQ);
}

void TchDrv_DownloadDisableWD(void)
{
	/* null */
}

void TchDrv_DownloadEnableWD(void)
{
	/* null */
}

// provides delays in uS
static void Delay1us(void)
{
	udelay(1);
}

static void Delay10us(UInt32 uSdelay)
{
	udelay(uSdelay);
}

void OSTASK_Sleep(int delay)
{
	mdelay(delay);
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_driver_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//--------------------------------------------------------------------------

#define SECURITY_DATA	0xFF

unsigned char    bTargetDataPtr;
unsigned char    abTargetDataOUT[TARGET_DATABUFF_LEN];
unsigned char    firmData[512][64];


// ****************************** PORT BIT MASKS ******************************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
#define SDATA_PIN   0x80        // P1.7
#define SCLK_PIN    0x40        // P1.6
#define XRES_PIN    0x40        // P2.6
#define TARGET_VDD  0x08        // P2.3









// ((((((((((((((((((((((( DEMO ISSP SUBROUTINE SECTION )))))))))))))))))))))))
// ((((( Demo Routines can be deleted in final ISSP project if not used   )))))
// ((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))))

// ============================================================================
// InitTargetTestData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Loads a 64-Byte array to use as test data to program target. Ultimately,
// this data should be fed to the Host by some other means, ie: I2C, RS232,
// etc. Data should be derived from hex file.
//  Global variables affected:
//    bTargetDataPtr
//    abTargetDataOUT
// ============================================================================
void InitTargetTestData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // create unique data for each block
    for (bTargetDataPtr = 0; bTargetDataPtr < TARGET_DATABUFF_LEN; bTargetDataPtr++)
    {
        abTargetDataOUT[bTargetDataPtr] = 0x55;
    }
}


// ============================================================================
// LoadArrayWithSecurityData()
// !!!!!!!!!!!!!!!!!!FOR TEST!!!!!!!!!!!!!!!!!!!!!!!!!!
// PROCESSOR_SPECIFIC
// Most likely this data will be fed to the Host by some other means, ie: I2C,
// RS232, etc., or will be fixed in the host. The security data should come
// from the hex file.
//   bStart  - the starting byte in the array for loading data
//   bLength - the number of byte to write into the array
//   bType   - the security data to write over the range defined by bStart and
//             bLength
// ============================================================================
void LoadArrayWithSecurityData(unsigned char bStart, unsigned char bLength, unsigned char bType)
{
    // Now, write the desired security-bytes for the range specified
    for (bTargetDataPtr = bStart; bTargetDataPtr < bLength; bTargetDataPtr++) {
        abTargetDataOUT[bTargetDataPtr] = bType;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// Delay()
// This delay uses a simple "nop" loop. With the CPU running at 24MHz, each
// pass of the loop is about 1 usec plus an overhead of about 3 usec.
//      total delay = (n + 3) * 1 usec
// To adjust delays and to adapt delays when porting this application, see the
// ISSP_Delays.h file.
// ****************************************************************************
void Delay(unsigned int n)
{
    while(n)
    {
        //asm("nop");
        //_nop_();

        n -= 1;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// LoadProgramData()
// The final application should load program data from HEX file generated by
// PSoC Designer into a 64 byte host ram buffer.
//    1. Read data from next line in hex file into ram buffer. One record
//      (line) is 64 bytes of data.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread data from file or abort
//    4. Exit this Function and Program block or verify the block.
// This demo program will, instead, load predetermined data into each block.
// The demo does it this way because there is no comm link to get data.
// ****************************************************************************
void LoadProgramData(unsigned char bBlockNum, unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function InitTargetTestData fills buffer for demo
    InitTargetTestData(bBlockNum, bBankNum);

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns VOID.
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fLoadSecurityData()
// Load security data from hex file into 64 byte host ram buffer. In a fully
// functional program (not a demo) this routine should do the following:
//    1. Read data from security record in hex file into ram buffer.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread security data from file or abort
//    4. Exit this Function and Program block
// In this demo routine, all of the security data is set to unprotected (0x00)
// and it returns.
// This function always returns PASS. The flag return is reserving
// functionality for non-demo versions.
// ****************************************************************************
signed char fLoadSecurityData(unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function LoadArrayWithSecurityData fills buffer for demo
//    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0x00);
    LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, SECURITY_DATA);		//PTJ: 0x1B (00 01 10 11) is more interesting security data than 0x00 for testing purposes

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns PASS.
    return(PASS);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fSDATACheck()
// Check SDATA pin for high or low logic level and return value to calling
// routine.
// Returns:
//     0 if the pin was low.
//     1 if the pin was high.
// ****************************************************************************
unsigned char fSDATACheck(void)
{
#if 0
    //if(PRT1DR & SDATA_PIN)
    if (SDATA_Read())
        return(1);
    else
        return(0);
#endif
	//I2C_SET_SDA_GPIO();	//gpio input

	if (I2C_READ_SDA_GPIO())
        return(1);
    else
        return(0);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKHigh()
// Set the SCLK pin High
// ****************************************************************************
void SCLKHigh(void)
{
#if 0
    //PRT1DR |= SCLK_PIN;
    SCLK_Write(1);
#endif


	//I2C_CLR_SCL_GPIO();		//gpio output //FMT_WSHAN 20101028 ...Firmware Download
	I2C_SET_SCL_GPIO_HIGH();//gpio output high
	Delay10us(1);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKLow()
// Make Clock pin Low
// ****************************************************************************
void SCLKLow(void)
{
#if 0
    //PRT1DR &= ~SCLK_PIN;
    SCLK_Write(0);
#endif
	//I2C_CLR_SCL_GPIO(); 	//gpio output //FMT_WSHAN 20101028 ...Firmware Download
	I2C_SET_SCL_GPIO_LOW();	//gpio output low
	Delay10us(1);
}

#ifndef RESET_MODE  // Only needed for power cycle mode
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKHiZ()
// Set SCLK pin to HighZ drive mode.
// ****************************************************************************
void SetSCLKHiZ(void)
{
#if 0
    //PRT1DM0 &= ~SCLK_PIN;
    //PRT1DM1 |=  SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    SCLK_SetDriveMode(SCLK_DM_DIG_HIZ);
#endif

	I2C_SET_SCL_GPIO();	//gpio input
}
#endif

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKStrong()
// Set SCLK to an output (Strong drive mode)
// ****************************************************************************
void SetSCLKStrong(void)
{
#if 0
    //PRT1DM0 |=  SCLK_PIN;
    //PRT1DM1 &= ~SCLK_PIN;
    //PRT1DM2 &= ~SCLK_PIN;
    SCLK_SetDriveMode(SCLK_DM_STRONG);
#endif

	I2C_CLR_SCL_GPIO(); 	//gpio output
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHigh()
// Make SDATA pin High
// ****************************************************************************
void SetSDATAHigh(void)
{
#if 0
    //PRT1DR |= SDATA_PIN;
    SDATA_Write(1);
#endif
	//I2C_CLR_SDA_GPIO(); 	//gpio output //FMT_WSHAN 20101028 ...Firmware Download
	I2C_SET_SDA_GPIO_HIGH();//gpio output high
	Delay10us(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATALow()
// Make SDATA pin Low
// ****************************************************************************
void SetSDATALow(void)
{
#if 0
    //PRT1DR &= ~SDATA_PIN;
    SDATA_Write(0);
#endif
	//I2C_CLR_SDA_GPIO(); 	//gpio output //FMT_WSHAN 20101028 ...Firmware Download
	I2C_SET_SDA_GPIO_LOW();	//gpio output low
	Delay10us(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHiZ()
// Set SDATA pin to an input (HighZ drive mode).
// ****************************************************************************
void SetSDATAHiZ(void)
{
#if 0
    //PRT1DM0 &= ~SDATA_PIN;
    //PRT1DM1 |=  SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    SDATA_SetDriveMode(SDATA_DM_DIG_HIZ);
#endif

	I2C_SET_SDA_GPIO(); //gpio input
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAStrong()
// Set SDATA for transmission (Strong drive mode) -- as opposed to being set to
// High Z for receiving data.
// ****************************************************************************
void SetSDATAStrong(void)
{
#if 0
    //PRT1DM0 |=  SDATA_PIN;
    //PRT1DM1 &= ~SDATA_PIN;
    //PRT1DM2 &= ~SDATA_PIN;
    SDATA_SetDriveMode(SDATA_DM_STRONG);
#endif

	I2C_CLR_SDA_GPIO(); 	//gpio output
}

#ifdef RESET_MODE
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetXRESStrong()
// Set external reset (XRES) to an output (Strong drive mode).
// ****************************************************************************
void SetXRESStrong(void)
{
    PRT2DM0 |=  XRES_PIN;
    PRT2DM1 &= ~XRES_PIN;
    PRT2DM2 &= ~XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// AssertXRES()
// Set XRES pin High
// ****************************************************************************
void AssertXRES(void)
{
    PRT2DR |= XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// DeassertXRES()
// Set XRES pin low.
// ****************************************************************************
void DeassertXRES(void)
{
    PRT2DR &= ~XRES_PIN;
}
#else

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetTargetVDDStrong()
// Set VDD pin (PWR) to an output (Strong drive mode).
// ****************************************************************************
void SetTargetVDDStrong(void)
{
#if 0
    //PRT2DM0 |=  TARGET_VDD;
    //PRT2DM1 &= ~TARGET_VDD;
    //PRT2DM2 &= ~TARGET_VDD;
#endif
	TchDrv_DownloadVddSetLow();
//	OsSleep(200);
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
	OSTASK_Sleep(500);
#else
	OSTASK_Sleep(200);
#endif
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// ApplyTargetVDD()
// Provide power to the target PSoC's Vdd pin through a GPIO.
// ****************************************************************************
void ApplyTargetVDD(void)
{
#if 0
    //PRT2DR |= TARGET_VDD;
    Vdd_Write(1);
#endif
	TchDrv_DownloadVddSetHigh();
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// RemoveTargetVDD()
// Remove power from the target PSoC's Vdd pin.
// ****************************************************************************
void RemoveTargetVDD(void)
{
#if 0
    //PRT2DR &= ~TARGET_VDD;
    Vdd_Write(0);
#endif
}
#endif
//end of file ISSP_Drive_Routines.c



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//--------------------------------------------------------------------------

//#include <m8c.h>        // part specific constants and macros
//#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
//#include "ISSP_Defs.h"
//#include "ISSP_Vectors.h"
//#include "ISSP_Extern.h"
//#include "ISSP_Errors.h"
//#include "ISSP_Directives.h"
//#include "ISSP_Delays.h"
//#include "Device.h"

#define PROGRAM_DATA	0x11

unsigned char  bTargetDataIN;
unsigned char  abTargetDataOUT_secure[TARGET_DATABUFF_LEN] ={0x00,};

unsigned char  bTargetAddress;
unsigned char  bTargetDataPtr = 0;
unsigned char  bTargetID[10];
unsigned char  bTargetStatus; // bTargetStatus[10];			//PTJ: created to support READ-STATUS in fReadStatus()

unsigned char  fIsError = 0;

/* ((((((((((((((((((((( LOW-LEVEL ISSP SUBROUTINE SECTION ))))))))))))))))))))
   (( The subroutines in this section use functions from the C file          ))
   (( ISSP_Drive_Routines.c. The functions in that file interface to the     ))
   (( processor specific hardware. So, these functions should work as is, if ))
   (( the routines in ISSP_Drive_Routines.c are correctly converted.         ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

// ============================================================================
// RunClock()
// Description:
// Run Clock without sending/receiving bits. Use this when transitioning from
// write to read and read to write "num_cycles" is number of SCLK cycles, not
// number of counter cycles.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// ============================================================================
void RunClock(unsigned int iNumCycles)
{
    int i;

    for(i=0; i < iNumCycles; i++)
    {
        SCLKLow();
        SCLKHigh();
    }
}

// ============================================================================
// bReceiveBit()
// Clocks the SCLK pin (high-low-high) and reads the status of the SDATA pin
// after the rising edge.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if SDATA was low
//     1 if SDATA was high
// ============================================================================
unsigned char bReceiveBit(void)
{
    SCLKLow();
    SCLKHigh();
    if (fSDATACheck()) {
        return(1);
    }
    else {
        return(0);
    }
}

// ============================================================================
// bReceiveByte()
// Calls ReceiveBit 8 times to receive one byte.
// Returns:
//     The 8-bit values recieved.
// ============================================================================
unsigned char bReceiveByte(void)
{
    unsigned char b;
    unsigned char bCurrByte = 0x00;

    for (b=0; b<8; b++) {
        bCurrByte = (bCurrByte<<1) + bReceiveBit();
    }
    return(bCurrByte);
}


// ============================================================================
// SendByte()
// This routine sends up to one byte of a vector, one bit at a time.
//    bCurrByte   the byte that contains the bits to be sent.
//    bSize       the number of bits to be sent. Valid values are 1 to 8.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// There is no returned value.
// ============================================================================
void SendByte(unsigned char bCurrByte, unsigned char bSize)
{
    unsigned char b = 0;

    for(b=0; b<bSize; b++)
    {
        if (bCurrByte & 0x80)
        {
            // Send a '1'
            SetSDATAHigh();
            SCLKHigh();
            SCLKLow();
        }
        else
        {
            // Send a '0'
            SetSDATALow();
            SCLKHigh();
            SCLKLow();
        }
        bCurrByte = bCurrByte << 1;
    }
}

// ============================================================================
// SendVector()
// This routine sends the vector specifed. All vectors constant strings found
// in ISSP_Vectors.h.  The data line is returned to HiZ after the vector is
// sent.
//    bVect      a pointer to the vector to be sent.
//    nNumBits   the number of bits to be sent.
//    bCurrByte  scratch var to keep the byte to be sent.
//
// There is no returned value.
// ============================================================================
void SendVector(const unsigned char* bVect, unsigned int iNumBits)
{
    SetSDATAStrong();
    while(iNumBits > 0)
    {
        if (iNumBits >= 8) {
            SendByte(*(bVect), 8);
            iNumBits -= 8;
            bVect++;
        }
        else {
            SendByte(*(bVect), iNumBits);
            iNumBits = 0;
        }
    }
#if 1 //FMT_WSHAN 20101028 ...Firmware Download
    gpio_tlmm_config(GPIO_CFG(I2C_GPIO_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

#else
    SetSDATAHiZ();
#endif
}


// ============================================================================
// fDetectHiLoTransition()
// Waits for transition from SDATA = 1 to SDATA = 0.  Has a 100 msec timeout.
// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for
// a high-to-low transition. This is used in the polling loop of
// fDetectHiLoTransition(). The timing of the while(1) loops can be calculated
// and the number of loops is counted, using iTimer, to determine when 100
// msec has passed.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if successful
//    -1 if timed out.
// ============================================================================
#if defined(CONFIG_MACH_RANT3) //|| defined(CONFIG_MACH_VINO)
signed char fDetectHiLoTransition(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = TRANSITION_TIMEOUT;
    while(1)
    {
        SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        Delay10us(1);
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    return (PASS);
}

signed char fDetectHiLoTransition_2(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = 700; //400;//TRANSITION_TIMEOUT;
    while(1)
    {
        SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
//            printk(KERN_INFO "%s: High check error\n", __func__);
            break;//return (PASS);
        }
    }

    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = 700; //400;//TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        Delay10us(1);
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
//            printk(KERN_INFO "%s: Low check error\n", __func__);
            break;//return (PASS);
        }
    }
//    printk(KERN_INFO "%s: Low check OK!\n", __func__);
    return (PASS);
}
#else
signed char fDetectHiLoTransition(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = TRANSITION_TIMEOUT;
    while(1)
    {
        SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    return (PASS);
}

signed char fDetectHiLoTransition_2(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    unsigned long int iTimer=0;

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
    iTimer = TRANSITION_TIMEOUT;
    while(1)
    {
        //SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
        break;
        //SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1)
    {
        //SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        //SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
            return (ERROR);
        }
    }
    return (PASS);
}
#endif

/* ((((((((((((((((((((( HIGH-LEVEL ISSP ROUTINE SECTION ))))))))))))))))))))))
   (( These functions are mostly made of calls to the low level routines     ))
   (( above.  This should isolate the processor-specific changes so that     ))
   (( these routines do not need to be modified.                             ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

#ifdef RESET_MODE
// ============================================================================
// fXRESInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fXRESInitializeTargetForISSP(void)
{
    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();
    SetXRESStrong();

    // Cycle reset and put the device in programming mode when it exits reset
    AssertXRES();
    Delay(XRES_CLK_DELAY);
    DeassertXRES();

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    //PTJ: Send id_setup_1 instead of init1_v
    //PTJ: both send CA Test Key and do a Calibrate1 SROM function
    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    // NOTE: DO NOT not wait for HiLo on SDATA after vector Init-3
    //       it does not occur (per spec).
    return(PASS);
}

#else  //else = the part is power cycle programmed

// ============================================================================
// fPowerCycleInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// The first time fDetectHiLoTransition is called the Clk pin is highZ because
// the clock is not needed during acquire.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fPowerCycleInitializeTargetForISSP(void)
{
    unsigned char n;

    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
#if !defined(CONFIG_MACH_RANT3)  //&& !defined(CONFIG_MACH_VINO)
    SetSCLKHiZ();
    SetSDATAHiZ();
#endif

    // Turn on power to the target device before other signals
    SetTargetVDDStrong();
    ApplyTargetVDD();
    // wait 1msec for the power to stabilize

//    for (n=0; n<10; n++) {
//        Delay(DELAY100us);
//    }

	//OsSleep(1);
#if !defined(CONFIG_MACH_RANT3) //&& !defined(CONFIG_MACH_VINO)
	OSTASK_Sleep(1);
#endif

    // Set SCLK to high Z so there is no clock and wait for a high to low
    // transition on SDAT. SCLK is not needed this time.
    SetSCLKHiZ();
#if defined(CONFIG_MACH_RANT3) //|| defined(CONFIG_MACH_VINO)
    SetSDATAHiZ();

	udelay(100);

	if (fSDATACheck())       // exit once SDATA goes HI
	{
		if (fIsError = fDetectHiLoTransition()) {
			return(INIT_ERROR);
		}
	}
	else
	{
		if (fIsError = fDetectHiLoTransition_2()) {
			return(INIT_ERROR);
		}
	}
#else
    if (fIsError = fDetectHiLoTransition()) {
        return(INIT_ERROR);
    }
#endif

    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();					//PTJ: DO NOT SET A BREAKPOINT HERE AND EXPECT SILICON ID TO PASS!

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    // NOTE: DO NOT not wait for HiLo on SDATA after vector Init-3
    //       it does not occur (per spec).
    return(PASS);
}
#endif


// ============================================================================
// fVerifySiliconID()
// Returns:
//     0 if successful
//     Si_ID_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySiliconID(void)
{
    SendVector(id_setup_2, num_bits_id_setup_2);
    if (fIsError = fDetectHiLoTransition())
    {
        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("fDetectHiLoTransition Error");
        #endif
        return(SiID_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    SendVector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetID[0] = bReceiveByte();
    RunClock(1);
    SendVector(read_id_v+2, 12);    // 1+11 bits starting from the 3rd byte

    RunClock(2);                    // Read-LSB Command
    bTargetID[1] = bReceiveByte();

    RunClock(1);
    SendVector(read_id_v+4, 1);     // 1 bit starting from the 5th byte

    //read Revision ID from Accumulator A and Accumulator X
#if defined(CONFIG_MACH_RANT3) //|| defined(CONFIG_MACH_VINO)
    SendVector(read_id_v+5, 11);	//11 bits starting from the 6th byte
    RunClock(2);
    bTargetID[2] = bReceiveByte();	//Read from Acc.X
    RunClock(1);
    SendVector(read_id_v+7, 12);    //1+11 bits starting from the 8th byte

    RunClock(2);
    bTargetID[3] = bReceiveByte();	//Read from Acc.A

    RunClock(1);
    SendVector(read_id_v+4, 1);     // 1 bit starting from the 5th byte,
#endif

    SendVector(tsync_disable, num_bits_tsync_disable);


    #ifdef TX_ON
        // Print READ-ID
        UART_PutCRLF(0);
        UART_PutString("Silicon-ID : ");
        UART_PutChar(' ');
        UART_PutHexByte(bTargetID[0]);
        UART_PutChar(' ');
        UART_PutHexByte(bTargetID[1]);
        UART_PutChar(' ');
    #endif

    #ifdef LCD_ON
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("ID : ");
        LCD_Char_PrintInt8(bTargetID[0]);
        LCD_Char_PutChar(' ');
        LCD_Char_PrintInt8(bTargetID[1]);
        LCD_Char_PutChar(' ');
    #endif

    if (bTargetID[0] != target_id_v[0] /*|| bTargetID[1] != target_id_v[1]*/)
    {
        return(SiID_ERROR);
    }
    else
    {
        return(PASS);
    }
}

// PTJ: =======================================================================
// fReadStatus()
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadStatus(void)
{
    SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    SendVector(read_status, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetStatus = bReceiveByte();
    RunClock(1);
    SendVector(read_status+2, 1);    // 12 bits starting from the 3rd character

    SendVector(tsync_disable, num_bits_tsync_disable);

    if (bTargetStatus == 0x00)  // if bTargetStatus is 0x00, result is pass.
    {
        return PASS;
    }
    else
    {
        return BLOCK_ERROR;
    }

}

// PTJ: =======================================================================
// fReadWriteSetup()
// PTJ: The READ-WRITE-SETUP vector will enable TSYNC and switches the device
//		to SRAM bank1 for PROGRAM-AND-VERIFY, SECURE and VERIFY-SETUP.
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadWriteSetup(void)
{
	SendVector(read_write_setup, num_bits_read_write_setup);
	return(PASS);					//PTJ: is there anything else that should be done?
}

// ============================================================================
// fEraseTarget()
// Perform a bulk erase of the target device.
// Returns:
//     0 if successful
//     ERASE_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fEraseTarget(void)
{
    SendVector(erase, num_bits_erase);
    if (fIsError = fDetectHiLoTransition()) {
        return(ERASE_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}


// ============================================================================
// LoadTarget()
// Transfers data from array in Host to RAM buffer in the target.
// Returns the checksum of the data.
// ============================================================================
unsigned int iLoadTarget(void)
{
unsigned char bTemp;
unsigned int  iChecksumData = 0;

    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();

    // Transfer the temporary RAM array into the target.
    // In this section, a 128-Byte array was specified by #define, so the entire
    // 128-Bytes are written in this loop.
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    while(bTargetDataPtr < TARGET_DATABUFF_LEN) {
        bTemp = abTargetDataOUT[bTargetDataPtr];
        iChecksumData += bTemp;

        SendByte(write_byte_start,4);    //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTargetAddress, 7);	 //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);

        // !!!NOTE:
        // SendByte() uses MSbits, so inc by '2' to put the 0..128 address into
        // the seven MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 7-Bits long. The SendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..128 as the address using
        // SendByte(), we have to left justify the address by 1-Bit.
        //   This can be done easily by incrementing the address each time by
        // '2' rather than by '1'.

        bTargetAddress += 2;			//PTJ: inc by 2 in order to support a 128 byte address space
        bTargetDataPtr++;
    }

    return(iChecksumData);
}


// ============================================================================
// fProgramTargetBlock()
// Program one block with data that has been loaded into a RAM buffer in the
// target device.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fProgramTargetBlock(unsigned char bBankNumber, unsigned char bBlockNumber)
{

    SendVector(tsync_enable, num_bits_tsync_enable);

    SendVector(set_block_num, num_bits_set_block_num);

	// Set the drive here because SendByte() does not.
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    SendByte(set_block_num_end, 3);

    SendVector(tsync_disable, num_bits_tsync_disable);	//PTJ:

    // Send the program-block vector.
    SendVector(program_and_verify, num_bits_program_and_verify);		//PTJ: PROGRAM-AND-VERIFY
    // wait for acknowledge from target.
    if (fIsError = fDetectHiLoTransition())
    {
        return(BLOCK_ERROR);
    }
    // Send the Wait-For-Poll-End vector
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);

    //PTJ: Don't do READ-STATUS here because that will
    //PTJ: require that we return multiple error values, if error occurs
}


// ============================================================================
// fAddTargetBankChecksum()
// Reads and adds the target bank checksum to the referenced accumulator.
// Returns:
//     0 if successful
//     VERIFY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fAccTargetBankChecksum(unsigned int* pAcc)
{
    unsigned int wCheckSumData=0;

    SendVector(checksum_v, num_bits_checksum);

    if (fIsError = fDetectHiLoTransition())
    {
        return(CHECKSUM_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    //SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read Checksum vector and get Target Checksum
    SendVector(read_checksum_v, 11);     // first 11-bits is ReadCKSum-MSB
    RunClock(2);                         // Two SCLKs between write & read
    bTargetDataIN = bReceiveByte();
    wCheckSumData = ((unsigned int)(bTargetDataIN))<<8;

    RunClock(1);                         // See Fig. 6
    SendVector(read_checksum_v + 2, 12); // 12 bits starting from 3rd character
    RunClock(2);                         // Read-LSB Command
    bTargetDataIN = bReceiveByte();
    wCheckSumData |= (unsigned int) bTargetDataIN;
    RunClock(1);
    SendVector(read_checksum_v + 4, 1);  // Send the final bit of the command

    //SendVector(tsync_disable, num_bits_tsync_disable);

    *pAcc = wCheckSumData;

    return(PASS);
}


// ============================================================================
// ReStartTarget()
// After programming, the target PSoC must be reset to take it out of
// programming mode. This routine performs a reset.
// ============================================================================
void ReStartTarget(void)
{
#ifdef RESET_MODE
    // Assert XRES, then release, then disable XRES-Enable
    AssertXRES();
    Delay(XRES_CLK_DELAY);
    DeassertXRES();
#else
    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();
    // Cycle power on the target to cause a reset
    RemoveTargetVDD();
    Delay(POWER_CYCLE_DELAY);
    ApplyTargetVDD();
#endif
}

// ============================================================================
// fVerifySetup()
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySetup(unsigned char bBankNumber, unsigned char bBlockNumber)
{
    SendVector(tsync_enable, num_bits_tsync_enable);

    SendVector(set_block_num, num_bits_set_block_num);

	//Set the drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    SendByte(set_block_num_end, 3);

    SendVector(tsync_disable, num_bits_tsync_disable);

    SendVector(verify_setup, num_bits_my_verify_setup);
    if (fIsError = fDetectHiLoTransition())
    {
        return(VERIFY_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    return(PASS);
}

// ============================================================================
// fReadByteLoop()
// Reads the data back from Target SRAM and compares it to expected data in
// Host SRAM
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================

signed char fReadByteLoop(void)
{
	bTargetAddress = 0;
    bTargetDataPtr = 0;

    while(bTargetDataPtr < TARGET_DATABUFF_LEN)
    {
        //Send Read Byte vector and then get a byte from Target
        SendVector(read_byte_v, 4);
        // Set the drive here because SendByte() does not
        SetSDATAStrong();
        SendByte(bTargetAddress,7);

        RunClock(2);       // Run two SCLK cycles between writing and reading
        SetSDATAHiZ();     // Set to HiZ so Target can drive SDATA
        bTargetDataIN = bReceiveByte();

        RunClock(1);
        SendVector(read_byte_v + 1, 1);     // Send the ReadByte Vector End

        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if (bTargetDataIN != abTargetDataOUT[bTargetDataPtr])
        {
            #ifdef TX_ON
                UART_PutCRLF(0);
                UART_PutString("bTargetDataIN : ");
                UART_PutHexByte(bTargetDataIN);
                UART_PutString(" abTargetDataOUT : ");
                UART_PutHexByte(abTargetDataOUT[bTargetDataPtr]);
            #endif
            return(BLOCK_ERROR);
        }

        bTargetDataPtr++;
        // Increment the address by 2 to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        bTargetAddress += 2;

    }

    return(PASS);
}

// ============================================================================
// fSecureTargetFlash()
// Before calling, load the array, abTargetDataOUT, with the desired security
// settings using LoadArrayWithSecurityData(StartAddress,Length,SecurityType).
// The can be called multiple times with different SecurityTypes as needed for
// particular Flash Blocks. Or set them all the same using the call below:
// LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0);
// Returns:
//     0 if successful
//     SECURITY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fSecureTargetFlash(void)
{
    unsigned char bTemp;

    // Transfer the temporary RAM array into the target
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    SetSDATAStrong();
    while(bTargetDataPtr < SECURITY_BYTES_PER_BANK)
    {
#if defined(CONFIG_MACH_RANT3) //|| defined(CONFIG_MACH_VINO)
        bTemp = 0xAA;
#else
        bTemp = abTargetDataOUT_secure[bTargetDataPtr];
#endif
        SendByte(write_byte_start,4);
        SendByte(bTargetAddress, 7);
        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);


        // SendBytes() uses MSBits, so increment the address by '2' to put
        // the 0..n address into the seven MSBit locations
        bTargetAddress += 2;				//PTJ: inc by 2 in order to support a 128 byte address space
        bTargetDataPtr++;
    }

    SendVector(secure, num_bits_secure);	//PTJ:
    if (fIsError = fDetectHiLoTransition())
    {
        return(SECURITY_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Main.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// Copyright 2006-2007, Cypress Semiconductor Corporation.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
//BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//LIABILITY, WHETHER IN CONRTACT, STRICT LIABILITY, OR TORT (INCLUDING
//NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//---------------------------------------------------------------------------*/

/* ############################################################################
   ###################  CRITICAL PROJECT CONSTRAINTS   ########################
   ############################################################################

   ISSP programming can only occur within a temperature range of 5C to 50C.
   - This project is written without temperature compensation and using
     programming pulse-widths that match those used by programmers such as the
     Mini-Prog and the ISSP Programmer.
     This means that the die temperature of the PSoC device cannot be outside
     of the above temperature range.
     If a wider temperature range is required, contact your Cypress Semi-
     conductor FAE or sales person for assistance.

   The project can be configured to program devices at 5V or at 3.3V.
   - Initialization of the device is different for different voltages. The
     initialization is hardcoded and can only be set for one voltage range.
     The supported voltages ranges are 3.3V (3.0V to 3.6V) and 5V (4.75V to
     5.25V). See the device datasheet for more details. If varying voltage
     ranges must be supported, contact your Cypress Semiconductor FAE or sales
     person for assistance.
   - ISSP programming for the 2.7V range (2.7V to 3.0V) is not supported.

   This program does not support programming all PSoC Devices
   - It does not support obsoleted PSoC devices. A list of devices that are
     not supported is shown here:
         CY8C22x13 - not supported
         CY8C24x23 - not supported (CY8C24x23A is supported)
         CY8C25x43 - not supported
         CY8C26x43 - not supported
   - It does not suport devices that have not been released for sale at the
     time this version was created. If you need to ISSP program a newly released
     device, please contact Cypress Semiconductor Applications, your FAE or
     sales person for assistance.
     The CY8C20x23 devices are not supported at the time of this release.

   ############################################################################
   ##########################################################################*/


/* (((((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))
 PSoC In-System Serial Programming (ISSP) Template
 This PSoC Project is designed to be used as a template for designs that
 require PSoC ISSP Functions.

 This project is based on the AN2026 series of Application Notes. That app
 note should be referenced before any modifications to this project are made.

 The subroutines and files were created in such a way as to allow easy cut &
 paste as needed. There are no customer-specific functions in this project.
 This demo of the code utilizes a PSoC as the Host.

 Some of the subroutines could be merged, or otherwise reduced, but they have
 been written as independently as possible so that the specific steps involved
 within each function can easily be seen. By merging things, some code-space
 savings could be realized.

 As is, and with all features enabled, the project consumes approximately 3500
 bytes of code space, and 19-Bytes of RAM (not including stack usage). The
 Block-Verify requires a 64-Byte buffer for read-back verification. This same
 buffer could be used to hold the (actual) incoming program data.

 Please refer to the compiler-directives file "directives.h" to see the various
 features.

 The pin used in this project are assigned as shown below. The HOST pins are
 arbitrary and any 3 pins could be used (the masks used to control the pins
 must be changed). The TARGET pins cannot be changed, these are fixed function
 pins on the PSoC.
 The PWR pin is used to provide power to the target device if power cycle
 programming mode is used. The compiler directive RESET_MODE in ISSP_directives.h
 is used to select the programming mode. This pin could control the enable on
 a voltage regulator, or could control the gate of a FET that is used to turn
 the power to the PSoC on.
 The TP pin is a Test Point pin that can be used signal from the host processor
 that the program has completed certain tasks. Predefined test points are
 included that can be used to observe the timing for bulk erasing, block
 programming and security programming.

      SIGNAL  HOST  TARGET
      ---------------------
      SDATA   P1.0   P1.0
      SCLK    P1.1   P1.1
      XRES    P2.0   XRES
      PWR     P2.1   Vdd
      TP      P0.7   n/a

 For test & demonstration, this project generates the program data internally.
 It does not take-in the data from an external source such as I2C, UART, SPI,
 etc. However, the program was written in such a way to be portable into such
 designs. The spirit of this project was to keep it stripped to the minimum
 functions required to do the ISSP functions only, thereby making a portable
 framework for integration with other projects.

 The high-level functions have been written in C in order to be portable to
 other processors. The low-level functions that are processor dependent, such
 as toggling pins and implementing specific delays, are all found in the file
 ISSP_Drive_Routines.c. These functions must be converted to equivalent
 functions for the HOST processor.  Care must be taken to meet the timing
 requirements when converting to a new processor. ISSP timing information can
 be found in Application Note AN2026.  All of the sections of this program
 that need to be modified for the host processor have "PROCESSOR_SPECIFIC" in
 the comments. By performing a "Find in files" using "PROCESSOR_SPECIFIC" these
 sections can easily be identified.

 The variables in this project use Hungarian notation. Hungarian prepends a
 lower case letter to each variable that identifies the variable type. The
 prefixes used in this program are defined below:
  b = byte length variable, signed char and unsigned char
  i = 2-byte length variable, signed int and unsigned int
  f = byte length variable used as a flag (TRUE = 0, FALSE != 0)
  ab = an array of byte length variables


 After this program has been ported to the desired host processor the timing
 of the signals must be confirmed.  The maximum SCLK frequency must be checked
 as well as the timing of the bulk erase, block write and security write
 pulses.

 The maximum SCLK frequency for the target device can be found in the device
 datasheet under AC Programming Specifications with a Symbol of "Fsclk".
 An oscilloscope should be used to make sure that no half-cycles (the high
 time or the low time) are shorter than the half-period of the maximum
 freqency. In other words, if the maximum SCLK frequency is 8MHz, there can be
 no high or low pulses shorter than 1/(2*8MHz), or 62.5 nsec.

 The test point (TP) functions, enabled by the define USE_TP, provide an output
 from the host processor that brackets the timing of the internal bulk erase,
 block write and security write programming pulses. An oscilloscope, along with
 break points in the PSoC ICE Debugger should be used to verify the timing of
 the programming.  The Application Note, "Host-Sourced Serial Programming"
 explains how to do these measurements and should be consulted for the expected
 timing of the erase and program pulses.

/* ############################################################################
   ############################################################################

(((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))) */



/*----------------------------------------------------------------------------
//                               C main line
//----------------------------------------------------------------------------
*/

//#include <m8c.h>        // part specific constants and macros
//#include "PSoCAPI.h"    // PSoC API definitions for all User Modules


// ------ Declarations Associated with ISSP Files & Routines -------
//     Add these to your project as needed.
//#include "ISSP_extern.h"
//#include "ISSP_directives.h"
//#include "ISSP_defs.h"
//#include "ISSP_errors.h"
//#include "Device.h"
/* ------------------------------------------------------------------------- */

unsigned char bBankCounter;
unsigned int  iBlockCounter;
unsigned int  iChecksumData;
unsigned int  iChecksumTarget;




/* ========================================================================= */
// ErrorTrap()
// Return is not valid from main for PSOC, so this ErrorTrap routine is used.
// For some systems returning an error code will work best. For those, the
// calls to ErrorTrap() should be replaced with a return(bErrorNumber). For
// other systems another method of reporting an error could be added to this
// function -- such as reporting over a communcations port.
/* ========================================================================= */
void ErrorTrap(unsigned char bErrorNumber)
{
    #ifndef RESET_MODE
        // Set all pins to highZ to avoid back powering the PSoC through the GPIO
        // protection diodes.
        SetSCLKHiZ();
        SetSDATAHiZ();
        // If Power Cycle programming, turn off the target
        RemoveTargetVDD();
    #endif


    #ifdef TX_ON
        UART_PutCRLF(0);
        UART_PutString("ErrorTrap");
        UART_PutHexByte(bErrorNumber);
    #endif

    #ifdef LCD_ON
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("                ");
        LCD_Char_Position(1, 0);
        LCD_Char_PrintString("ErrorTrap");
        LCD_Char_PrintInt8(bErrorNumber);
    #endif

	/* Enable watchdog and interrupt */
//	TchDrv_DownloadEnableWD();
//	TchDrv_DownloadEnableIRQ();

    //while (1);
    // return(bErrorNumbers);
}

/* ========================================================================= */
/* MAIN LOOP                                                                 */
/* Based on the diagram in the AN2026                                        */
/* ========================================================================= */
unsigned char make2ChTo1(unsigned char hi, unsigned char lo)
{
    unsigned char ch;

    if(hi == 'A' || hi == 'a')
        hi = 0xa;
    else if(hi == 'B' || hi == 'b')
        hi = 0xb;
    else if(hi == 'C' || hi == 'c')
        hi = 0xc;
    else if(hi == 'D' || hi == 'd')
        hi = 0xd;
    else if(hi == 'E' || hi == 'e')
        hi = 0xe;
    else if(hi == 'F' || hi == 'f')
        hi = 0xf;
    else
        hi = hi;

    if(lo == 'A' || lo == 'a')
        lo = 0xa;
    else if(lo == 'B' || lo == 'b')
        lo = 0xb;
    else if(lo == 'C' || lo == 'c')
        lo = 0xc;
    else if(lo == 'D' || lo == 'd')
        lo = 0xd;
    else if(lo == 'E' || lo == 'e')
        lo = 0xe;
    else if(lo == 'F' || lo == 'f')
        lo = 0xf;
    else
        lo = lo;

    ch = ((hi&0x0f) << 4) | (lo & 0x0f);

    return ch;
}

UInt16 load_tma340_frimware_data(void)
{
	UInt8 temp_onelinedata[128];
	UInt16 i, j, firmwareline, onelinelength;

	for(firmwareline=0; firmwareline<512; firmwareline++)
	{
		i = 0;
		if(fromsdcard)
		{
			for(j = 0; j < 128; j++)
			{
				temp_onelinedata[j] = pfirmware[9 + 141 * firmwareline + j];
			}
		}
		else
		{
#if defined(CONFIG_MCH_VINO)
			if( CYTOUCH_NEW_PATERN )
			{
				strncpy(temp_onelinedata, cytma340_fw_np + 141*firmwareline + 9, 128);
			}
			else
#endif
			{
				strncpy(temp_onelinedata, cytma340_fw + 141*firmwareline + 9, 128);
			}

		}

		for(onelinelength=0; onelinelength<64; onelinelength++)
		{
			firmData[firmwareline][onelinelength] = make2ChTo1(temp_onelinedata[i], temp_onelinedata[i+1]);
			i += 2;
		}
	}

	return PASS;
}


int tma340_frimware_update(void)
{
    // -- This example section of commands show the high-level calls to -------
    // -- perform Target Initialization, SilcionID Test, Bulk-Erase, Target ---
    // -- RAM Load, FLASH-Block Program, and Target Checksum Verification. ----
	UInt16 i;
	UInt16 aIndex;

    #ifdef TX_ON
        UART_PutString("Start HSSP - TMA3x0");
        UART_PutCRLF(0);

    #endif

	if (fIsError = load_tma340_frimware_data())
    {
        ErrorTrap(fIsError);
		return fIsError;
    }

    // >>>> ISSP Programming Starts Here <<<<

    // Acquire the device through reset or power cycle
    #ifdef RESET_MODE
        // Initialize the Host & Target for ISSP operations
        if (fIsError = fXRESInitializeTargetForISSP())
        {
            ErrorTrap(fIsError);
			return fIsError;
        }
    #else
        // Initialize the Host & Target for ISSP operations
        if (fIsError = fPowerCycleInitializeTargetForISSP())
        {
            ErrorTrap(fIsError);
			return fIsError;
        }
    #endif


    // Run the SiliconID Verification, and proceed according to result.
    if (fIsError = fVerifySiliconID())
    {
        ErrorTrap(fIsError);
		return fIsError;
    }
    #ifdef TX_ON
        UART_PutCRLF(0);
        UART_PutString("End VerifySiliconID");
    #endif

	/* Disable watchdog and interrupt */
	TchDrv_DownloadDisableIRQ();	// Disable Baseband touch interrupt ISR.
	TchDrv_DownloadDisableWD();		// Disable Baseband watchdog timer

    #if 1
        // Bulk-Erase the Device.
        if (fIsError = fEraseTarget())
        {
            ErrorTrap(fIsError);
			//return fIsError;
			goto MCSDL_DOWNLOAD_FINISH;
        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("End EraseTarget");
            UART_PutCRLF(0);
            UART_PutString("Program Flash Blocks Start");
            UART_PutCRLF(0);
        #endif

#if defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS)
        bTargetEraseState = 1;
#endif

    #endif

    #if 1   // program flash block
        //LCD_Char_Position(1, 0);
        //LCD_Char_PrintString("Program Flash Blocks Start");

        //==============================================================//
        // Program Flash blocks with predetermined data. In the final application
        // this data should come from the HEX output of PSoC Designer.

        iChecksumData = 0;     // Calculte the device checksum as you go
        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
            if (fIsError = fReadWriteSetup())
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

			aIndex = iBlockCounter*2;

			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}

            //LoadProgramData(bBankCounter, (unsigned char)iBlockCounter);
            iChecksumData += iLoadTarget();

            if (fIsError = fProgramTargetBlock(bBankCounter,(unsigned char)iBlockCounter))
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            if (fIsError = fReadStatus())
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            #ifdef TX_ON
                UART_PutChar('#');
            #endif

        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Program Flash Blocks End");
        #endif

    #endif


    #if !defined(CONFIG_MACH_RANT3) //&& !defined(CONFIG_MACH_VINO)  // verify
        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Verify Start");
            UART_PutCRLF(0);
        #endif

        //=======================================================//
        //PTJ: Doing Verify
        //PTJ: this code isnt needed in the program flow because we use PROGRAM-AND-VERIFY (ProgramAndVerify SROM Func)
        //PTJ: which has Verify built into it.
        // Verify included for completeness in case host desires to do a stand-alone verify at a later date.

        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {
        	//LoadProgramData(bBankCounter, (unsigned char) iBlockCounter);
			aIndex = iBlockCounter*2;

			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}

            if (fIsError = fReadWriteSetup())
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            if (fIsError = fVerifySetup(bBankCounter,(unsigned char)iBlockCounter))
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if (fIsError = fReadStatus()) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if (fIsError = fReadWriteSetup()) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }


            if (fIsError = fReadByteLoop()) {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }

            #ifdef TX_ON
                UART_PutChar('.');
            #endif

        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Verify End");
        #endif

    #endif // end verify


    #if 1

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("Security Start");
        #endif


        //=======================================================//
        // Program security data into target PSoC. In the final application this
        // data should come from the HEX output of PSoC Designer.
        for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
        {
            //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1

            if (fIsError = fReadWriteSetup())
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }
            // Secure one bank of the target flash
            if (fIsError = fSecureTargetFlash())
            {
                ErrorTrap(fIsError);
				//return fIsError;
				goto MCSDL_DOWNLOAD_FINISH;
            }
        }

        #ifdef TX_ON
            UART_PutCRLF(0);
            UART_PutString("End Security data");
        #endif

    #endif

printk("[firmware downlod complete\n");


MCSDL_DOWNLOAD_FINISH :

	Delay10us(50*1000);
	Delay10us(50*1000);

	/* Enable watchdog and interrupt */
	TchDrv_DownloadEnableWD();
//	TchDrv_DownloadEnableIRQ();

	Delay10us(50*1000);
	Delay10us(50*1000);
	Delay10us(50*1000);
	Delay10us(50*1000);

	return fIsError;

}


#endif
