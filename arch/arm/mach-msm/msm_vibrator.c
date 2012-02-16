/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>
#include <linux/earlysuspend.h>
//#include <mach/clk.h>
#include <mach/samsung_vibe.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/delay.h>

struct clk *android_vib_clk; /* gp_clk */

#define GP_CLK_M_DEFAULT			21
#define GP_CLK_N_DEFAULT			18000
#define GP_CLK_D_DEFAULT			9000	/* 50% duty cycle */ 
#define IMM_PWM_MULTIPLIER		    17778	/* Must be integer */

#if defined(CONFIG_MACH_GIO)
#define VIB_LDO "ldo19"
#define VIB_LDO_LEVEL  OUT3300mV
#elif defined(CONFIG_MACH_ESCAPE)
	/* R730_HWREV0.2 LDO3 --> LDO19 */
    extern unsigned char hw_version ;
	#define VIB_LDO		(hw_version >= 2 ? "ldo19" : "ldo3")
	#define VIB_LDO_LEVEL  (hw_version >= 2 ? OUT3300mV : OUT3000mV)
#else
#define VIB_LDO "ldo3"
#define VIB_LDO_LEVEL  OUT3000mV
#endif

/*
 * ** Global variables for LRA PWM M,N and D values.
 * */
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;

static struct hrtimer vibe_timer;
static int is_vibe_on = 0;


static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state);
static int msm_vibrator_resume(struct platform_device *pdev);
static int msm_vibrator_probe(struct platform_device *pdev);
static int msm_vibrator_exit(struct platform_device *pdev);
static int msm_vibrator_power(int power_mode);


/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;

/*
 * This function is used to set and re-set the GP_CLK M and N counters
 * to output the desired target frequency.
 * 
 */

/* for the suspend/resume VIBRATOR Module */
static struct platform_driver msm_vibrator_platdrv = 
{
	.probe   = msm_vibrator_probe,
	.suspend = msm_vibrator_suspend,
	.resume  = msm_vibrator_resume,
	.remove  = __devexit_p(msm_vibrator_exit),
	.driver = 
	{
			.name = MODULE_NAME,
			.owner = THIS_MODULE,
	},
};

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(is_vibe_on) {
		clk_disable(android_vib_clk);
		is_vibe_on = 0;
	}
	msm_vibrator_power(VIBRATION_OFF);
	printk("[VIB] susepend\n");
	return VIBE_S_SUCCESS;
}

static int msm_vibrator_resume(struct platform_device *pdev)
{
	msm_vibrator_power(VIBRATION_ON);
	printk("[VIB] resume\n");
	return VIBE_S_SUCCESS;
}

static int __devexit msm_vibrator_exit(struct platform_device *pdev)
{
		printk("[VIB] EXIT\n");
		return 0;
}

/* Implemetation for Android power Management */
#ifdef CONFIG_HAS_EARLYSUSPEND

void msm_vibrator_early_suspend(struct early_suspend *);
void msm_vibrator_late_resume(struct early_suspend *);


void msm_vibrator_early_suspend(struct early_suspend *h)
{
	if(is_vibe_on) {
		clk_disable(android_vib_clk);
		is_vibe_on = 0;
	}
	msm_vibrator_power(VIBRATION_OFF);
}

void msm_vibrator_late_resume(struct early_suspend *h)
{
	msm_vibrator_power(VIBRATION_ON);
}

#endif


#ifdef CONFIG_HAS_EARLYSUSPEND

static struct early_suspend msm_vibrator_earlysuspend = {

  	.suspend = msm_vibrator_early_suspend,
	.resume = msm_vibrator_late_resume,

};

#endif

static int msm_vibrator_power(int on)
{
#if !(defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO))
	struct vreg *vreg_msm_vibrator;
	int ret;

	vreg_msm_vibrator = vreg_get(NULL, VIB_LDO);

	if(IS_ERR(vreg_msm_vibrator)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
							__func__,PTR_ERR(vreg_msm_vibrator));
			return PTR_ERR(vreg_msm_vibrator);
		}

	if(on) {
		ret = vreg_set_level(vreg_msm_vibrator, VIB_LDO_LEVEL);
		if(ret) {
				printk(KERN_ERR "%s: vreg set level failed (%d)\n",
								__func__,ret);
				return -EIO;
		}

		ret = vreg_enable(vreg_msm_vibrator);
		if(ret) {
				printk(KERN_ERR "%s: vreg enable failed (%d)\n",
								__func__,ret);
				return -EIO;
		}
	} else {
	
		ret = vreg_disable(vreg_msm_vibrator);
		if(ret) {
				printk(KERN_ERR "%s: vreg disable failed (%d)\n",
								__func__,ret);
				return -EIO;
		}
	}
#endif

	return VIBE_S_SUCCESS;
}

static int vibe_set_pwm_freq(int nForce)
{
#if 1
		/* Put the MND counter in reset mode for programming */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT); /* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT); /* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); /* Dual-edge mode */
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
		g_nForce_32 = ((nForce * g_nLRA_GP_CLK_PWM_MUL) >> 8) + g_nLRA_GP_CLK_D;
		printk("%s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
		HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);
		HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);                    /* Enable M/N counter */
		printk("%x, %x, %x\n",( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT,~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT,1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);
#else
		clk_set_rate(android_vib_clk,32583);
#endif	
		return VIBE_S_SUCCESS;
}

static void set_pmic_vibrator(int on)
{
	printk("[VIB] %s, input : %s\n",__func__,on ? "ON":"OFF");
	
	if (on) {
		clk_enable(android_vib_clk);
/*
Power off시 Ramdump 진입하는 문제에 대해 아래와 같이 수정. by System s/w sunil07.hwang@samsung.com
*/
        //jjh_110412 		
		if (gpio_request(VIB_ON, "vib_on"))
							   pr_err("failed to request vib_on\n");
		gpio_direction_output(VIB_ON, VIBRATION_ON);
		//jjh_110412
		gpio_free(VIB_ON);

		is_vibe_on = 1;

	} else {
		if(is_vibe_on) {
            //jjh_110412 			
			if (gpio_request(VIB_ON, "vib_on"))
										   pr_err("failed to request vib_on\n");
			gpio_direction_output(VIB_ON, VIBRATION_OFF);
            //jjh_110412 			
			gpio_free(VIB_ON);
			clk_disable(android_vib_clk);
			is_vibe_on = 0;
		}
	}

}

#if 0
static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}
#else
static void pmic_vibrator_on(void)
{
#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
	struct vreg *vreg_msm_vibrator;
	int ret;

	vreg_msm_vibrator = vreg_get(NULL, VIB_LDO);

	if(IS_ERR(vreg_msm_vibrator)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
							__func__,PTR_ERR(vreg_msm_vibrator));
			return PTR_ERR(vreg_msm_vibrator);
	}

	ret = vreg_set_level(vreg_msm_vibrator, VIB_LDO_LEVEL);
	if(ret) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
								__func__,ret);
			return -EIO;
	}

	ret = vreg_enable(vreg_msm_vibrator);
	if(ret) {
			printk(KERN_ERR "%s: vreg enable failed (%d)\n",
								__func__,ret);
			return -EIO;
	}
		mdelay(15);
#else
	set_pmic_vibrator(VIBRATION_ON);
#endif
}

static void pmic_vibrator_off(void)
{
#if defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
	struct vreg *vreg_msm_vibrator;
	int ret;

	vreg_msm_vibrator = vreg_get(NULL, VIB_LDO);

	if(IS_ERR(vreg_msm_vibrator)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
							__func__,PTR_ERR(vreg_msm_vibrator));
			return PTR_ERR(vreg_msm_vibrator);
	}

	ret = vreg_disable(vreg_msm_vibrator);
	if(ret) {
			printk(KERN_ERR "%s: vreg disable failed (%d)\n",
								__func__,ret);
			return -EIO;
	}
#else
	set_pmic_vibrator(VIBRATION_OFF);
#endif
}


#endif

spinlock_t msm_vibrator_lock;
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
// Escape disabled haptics. HW req. 20110520
#if 0 // /* defined(CONFIG_MACH_ESCAPE) ||*/ defined(CONFIG_MACH_GIO)
    if(hrtimer_active(&vibe_timer))
    {
    	unsigned int remain;
		ktime_t r = hrtimer_get_remaining(&vibe_timer);

		remain=r.tv.sec * 1000000 + r.tv.nsec;
		remain = remain / 1000;
		if(r.tv.sec < 0) {
				remain = 0;
		}

		printk("[VIB] (vibrator_enable) hrtimer active, remain:%d\n",remain);
		if(!remain)
			pmic_vibrator_off();
	}
#endif

	unsigned long flags;

	spin_lock_irqsave(&msm_vibrator_lock, flags);
	
	hrtimer_cancel(&vibe_timer);

	if (value == 0) {
		printk("[VIB] OFF\n");
		pmic_vibrator_off();
	}
	else {

		if(value < 0)
			value = ~value;
		printk("[VIB] ON, %d ms\n",value);

		//value = (value > 15000 ? 15000 : value);
// Escape disabled haptics. HW req. 20110520
#if 0 // /* defined(CONFIG_MACH_ESCAPE) ||*/ defined(CONFIG_MACH_GIO)
		if (value <= 30) value += 15 ; // TODO:RECHECK!!!
#endif

		pmic_vibrator_on();

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&msm_vibrator_lock, flags);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
#if 0
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
#else
		unsigned int remain;

		printk("[VIB] %s\n",__func__);
		if(hrtimer_active(&vibe_timer)) {
				ktime_t r = hrtimer_get_remaining(&vibe_timer);
				remain=r.tv.sec * 1000000 + r.tv.nsec;
				remain = remain / 1000;
				if(r.tv.sec < 0) {
						remain = 0;
				}
				printk("[VIB] hrtimer active, remain:%d\n",remain);
				if(!remain) 
					pmic_vibrator_off();
		} else {
				printk("[VIB] hrtimer not active\n");
			pmic_vibrator_off();
		}
		return HRTIMER_NORESTART;
#endif
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int __devinit msm_vibrator_probe(struct platform_device *pdev)
{
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
	
	if (gpio_request(VIB_ON, "vibrator"))
			pr_err("failed to request gpio VIB_ON\n");

	msm_vibrator_power(VIBRATION_ON);

	/* Vibrator init sequence 
	 * 1. power on ( vreg get )
	 * 2. clock get & enable ( gp_clk )
	 * 3. VIB_EN on
	 */

	android_vib_clk = clk_get(NULL,"gp_clk");

	if(IS_ERR(android_vib_clk)) {
		printk("[VIB] android vib clk failed!!!\n");
	} else {
		printk("[VIB] android vib clk is successful\n");
	}
#if !(defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO))
	vibe_set_pwm_freq(216);
#endif

	return 0;
}

static int __init msm_init_pmic_vibrator(void)
{
	int nRet;

	nRet = platform_driver_register(&msm_vibrator_platdrv);

	printk("[VIB] platform driver register result : %d\n",nRet);
	if (nRet)
	{ 
		printk("[VIB] platform_driver_register failed\n");
	}

	spin_lock_init(&msm_vibrator_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&msm_vibrator_earlysuspend);
#endif

	return nRet;

}

static void __exit msm_exit_pmic_vibrator(void)
{
	platform_driver_unregister(&msm_vibrator_platdrv);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&msm_vibrator_earlysuspend);
#endif

}

module_init(msm_init_pmic_vibrator);
module_exit(msm_exit_pmic_vibrator);


MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

