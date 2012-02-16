/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
//#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/regulator/max8893.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>
#include <linux/gpio_event.h>

#include "devices.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include <linux/i2c/max8893_gpio.h>
#include "pm.h"
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#include <linux/i2c/bma023_dev.h>

#ifdef CONFIG_SENSORS_MMC31XX
#include <linux/i2c/mmc31xx.h>
#endif

#include "smd_private.h"

#include <linux/i2c/tsp_gpio.h>

#ifdef CONFIG_SENSORS_KR3D_ACCEL
#include <linux/i2c/kr3dm_i2c.h>
#endif

#include <linux/skbuff.h>     //use wlan static buffer
#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#endif

#if defined(CONFIG_BCM4329) //#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/wlan_plat.h>
#endif

#ifdef CONFIG_ARCH_MSM7X27
#define MSM_PMEM_MDP_SIZE	0x1B76000
#define MSM_PMEM_ADSP_SIZE	0xB71000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define MSM_FB_SIZE		0x177000
#define PMEM_KERNEL_EBI1_SIZE	0x1C000
#endif

#define FRONT_CAMERA 1 ////SecFeature.Quattro aswoogi

extern struct platform_device keypad_device_vino;
extern struct platform_device keypad_device_vino_rev0;
extern struct platform_device keypad_device_vino_rev_03;
extern struct platform_device keypad_device_vino_rev_09;


unsigned char hw_version = 0xff;
EXPORT_SYMBOL(hw_version);

#define	BT_PWR		88

#define GPIO_BT_WAKE 81
#define GPIO_BT_HOST_WAKE 20//shiks_DJ23
#define GPIO_BT_RESET 74
#define GPIO_WLAN_RESET 82
#define GPIO_WLAN_WAKES_MSM 17
#define GPIO_BT_WLAN_REG_ON 22
#define I2C_SCL 60
#define I2C_SDA 61
#define I2C_2_SDA 110 // for Fuel Gauge
#define I2C_2_SCL 109
#define GAUGE_ALRT 21
#define T_FLASH_DET ((hw_version>2)?(49):(96))
#define IF_SDA 75 // for FSA9280
#define IF_SCL 16
#define TSP_SDA 29
#define TSP_SCL 30
#define TSP_INT 19
#define TSP_RESET 21
#define GPIO_JACK_S_35 		((hw_version>2)?((hw_version>3)?28:36):(123))
#define GPIO_SEND_END 		((hw_version>2)?(37):(1))
#define GPIO_POPUP_SW_EN 108

#if 1 // MBdkhan fixme later
#define GPIO_OUTPUT_HIGH 1
#define GPIO_OUTPUT_LOW	 0
#endif

#if defined(CONFIG_BCM4329) //#ifdef CONFIG_WIFI_CONTROL_FUNC
#define GPIO_WLAN_LEVEL_LOW			0
#define GPIO_WLAN_LEVEL_HIGH			1
#define GPIO_WLAN_LEVEL_NONE			2

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wifi_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};
#endif

#ifdef CONFIG_SAMSUNG_JACK

static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 3,
		.delay_ms	= 10,
		.check_count	= 5,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 99,
		.delay_ms	= 10,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[2] = {
		.adc_high	= 9999,
		.delay_ms	= 10,
		.check_count	= 5,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};

static int get_msm7x27_det_jack_state(void)
{
	/* Active Low */
	return(gpio_get_value(GPIO_JACK_S_35)) ^ 1;
}

#if 0
static int get_msm7x27_send_key_state(void)
{
	/* Active High */
	return(gpio_get_value(GPIO_SEND_END));
}
#endif

static void set_msm7x27_micbias_state(bool state)
{
	gpio_set_value(GPIO_EARBIAS_EN, state);  

}

static int sec_jack_get_adc_value(void)
{
	int data1=120;
	
	return data1;
}

static void sec_jack_gpio_init(void)
{
	gpio_request(GPIO_JACK_S_35, "h2w_ear_det");
	gpio_direction_input(GPIO_JACK_S_35);
	gpio_request(GPIO_EARBIAS_EN, "h2w_earmic_bias");
	gpio_direction_output(GPIO_EARBIAS_EN, 0);

}

static struct sec_jack_platform_data sec_jack_data = {
	.get_det_jack_state	= get_msm7x27_det_jack_state,
	//.get_send_key_state	= get_msm7x27_send_key_state,
	.set_micbias_state	= set_msm7x27_micbias_state,
	.get_adc_value	= sec_jack_get_adc_value,
	.zones		= jack_zones,
	.num_zones	= ARRAY_SIZE(jack_zones),
	.det_int	= MSM_GPIO_TO_INT(GPIO_JACK_S_35),
	//.send_int	= MSM_GPIO_TO_INT(GPIO_SEND_END),
};

static struct platform_device sec_device_jack = {
	.name           = "sec_jack",
	.id             = -1,
	.dev            = {
		.platform_data  = &sec_jack_data,
	},
};
#endif
static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C0043ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(132),
		.end	= MSM_GPIO_TO_INT(132),
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
  .name           = "usb_mass_storage",
  .id             = -1,
  .dev            = {
	.platform_data          = &usb_mass_storage_pdata,
  },
};
#endif
#ifdef CONFIG_USB_ANDROID

static char *usb_functions_default[] = {
	"usb_mass_storage",
};

static char *usb_functions_adb[] = {
	"acm",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_diag[] = {
	"acm",
	"usb_mass_storage",
	"diag",
};

static char *usb_functions_adb_diag[] = {
	"diag",
	"acm",
	"usb_mass_storage",
	"adb",
};

static char *fusion_usb_functions_default[] = {
	"diag",
	"nmea",
	"usb_mass_storage",
};

static char *fusion_usb_functions_default_adb[] = {
	"diag",
	"nmea",
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_rndis_diag[] = {
	"rndis",
	"diag",
};

static char *usb_functions_rndis_adb_diag[] = {
	"rndis",
	"diag",
	"adb",
};
static char *usb_functions_mtp_only[] = {
	"usb_mtp_gadget",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"usb_mass_storage",
	"adb",	
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
#ifdef CONFIG_USB_F_SERIAL
	"modem",
#ifndef CONFIG_USB_SAMSUNG_DRIVER
	"nmea",
#endif
#endif
};


static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0xF000,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_MASS_STORAGE,
		.device_subclass = 0x06,
		.device_protocol = 0x50,
		#endif
	},
// for anti-fruad
	{
		.product_id	= 0x681C,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_COMM,
		.device_subclass = 0,
		.device_protocol = 0,
		#endif
	},
	{
		.product_id	= 0x689E,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
		.functions	= usb_functions_diag,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_COMM,
		.device_subclass = 0,
		.device_protocol = 0,
		#endif
	},
// end for anti-fruad
	{
		.product_id	= 0x689E,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_COMM,
		.device_subclass = 0,
		.device_protocol = 0,
		#endif
	},
	{
		.product_id	= 0x5A0F,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_only),
		.functions	= usb_functions_mtp_only,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_COMM,
		.device_subclass = 0,
		.device_protocol = 0,
		#endif
	},
	{
		.product_id	= 0x6881,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
		#ifdef CONFIG_USB_SAMSUNG_DRIVER
		.device_class = USB_CLASS_COMM,
		.device_subclass = 0,
		.device_protocol = 0,
		#endif
	},
};


static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,

	.vendor		= "SAMSUNG",
	.product        = "SPH-M820 Card",
	.release	= 0x0100,
	.can_stall	= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x04E8,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x04E8, // Samsung Vendor ID
	.product_id	= 0xF000,
	.version	= 0x0100,
	.product_name	= "Samsung Android USB Device",
	.manufacturer_name = "SAMSUNG Electronics Co., Ltd.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,

	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
	{"rmnet", 6},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},
#ifdef CONFIG_USB_FUNCTION_RMNET
	{
		.product_id         = 0x9021,
		/* DIAG + RMNET */
		.functions	    = 0x41,
	},
	{
		.product_id         = 0x9022,
		/* DIAG + ADB + RMNET */
		.functions	    = 0x43,
	},
#endif

};

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
};
#endif

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
};

static void __init msm7x2x_init_host(void)
{
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		return;

	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_enable(vreg_3p3);
		vreg_disable(vreg_3p3);
		vreg_put(vreg_3p3);
}

		return 0;
}

static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		ret = msm_pm_app_rpc_init(callback);
	} else {
		msm_pm_app_rpc_deinit(callback);
		ret = 0;
	}
	return ret;
}

static int msm_otg_rpc_phy_reset(void __iomem *regs)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.ldo_init		= msm_hsusb_ldo_init,
	.pclk_required_during_lpm = 1,
	.pclk_src_name		= "ebi1_usb_clk",
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(USB, 14),               //ykp_sound Voice Dial for Hand MIC
	SND(STEREO_USB, 15),        //ykp_sound Coice Dial for Head MIC
	SND(IN_S_SADC_OUT_HANDSET, 16),
    SND(IN_S_SADC_OUT_HEADSET, 17),
	SND(BT_CONFERENCE, 24),     //ykp_sound added for Voice Dial with BT
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
    SND(FORCE_SPEAKER, 26),
    SND(FM_SPEAKER, 27),
    SND(NO_MIC_HEADSET, 29),
    SND(MEDIA_SPEAKER, 30),
    SND(MEDIA_STEREO_HEADSET, 31),
    SND(VOIP_HANDSET, 32),
    SND(VOIP_HEADSET, 33),
    SND(VOIP_SPEAKER, 34),
    SND(CURRENT, 36),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))

#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)


static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */

};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(87, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(86, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(85, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(84, 4, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "tsif_clk",
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

#define LCDC_CONFIG_PROC          21
#define LCDC_UN_CONFIG_PROC       22
#define LCDC_API_PROG             0x30000066
#define LCDC_API_VERS             0x00010001

#define GPIO_OUT_132    132
#define GPIO_OUT_131    131
#define GPIO_OUT_103    103
#define GPIO_OUT_102    102
#define GPIO_OUT_101    101

static struct msm_rpc_endpoint *lcdc_ep;
static int lcdc_config_lock = 0;

static int msm_fb_lcdc_config(int on)
{
	int rc = 0;
	struct rpc_request_hdr hdr;

	if( !lcdc_config_lock ) {
		lcdc_config_lock = 1;
	if (on)
		pr_info("lcdc config\n");
	else
		pr_info("lcdc un-config\n");

	lcdc_ep = msm_rpc_connect_compatible(LCDC_API_PROG, LCDC_API_VERS, 0);
	if (IS_ERR(lcdc_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
			__func__, PTR_ERR(lcdc_ep));
		return -EINVAL;
	}

	rc = msm_rpc_call(lcdc_ep,
				(on) ? LCDC_CONFIG_PROC : LCDC_UN_CONFIG_PROC,
				&hdr, sizeof(hdr),
				5 * HZ);
	if (rc)
		printk(KERN_ERR
			"%s: msm_rpc_call failed! rc = %d\n", __func__, rc);

	msm_rpc_close(lcdc_ep);
		lcdc_config_lock = 0;
	}
	return rc;
}

static int gpio_array_num[] = {
				GPIO_OUT_132, /* spi_clk */
				GPIO_OUT_131, /* spi_cs  */
				GPIO_OUT_103, /* spi_sdi */
				GPIO_OUT_102, /* spi_sdoi */
				GPIO_OUT_101,  /* lcd_reset */
				};

static void lcdc_gordon_gpio_init(void)
{
	if (gpio_request(GPIO_OUT_132, "spi_clk"))
		pr_err("failed to request gpio spi_clk\n");
	if (gpio_request(GPIO_OUT_131, "spi_cs"))
		pr_err("failed to request gpio spi_cs\n");
	if (gpio_request(GPIO_OUT_103, "spi_sdi"))
		pr_err("failed to request gpio spi_sdi\n");
	if (gpio_request(GPIO_OUT_102, "spi_sdoi"))
		pr_err("failed to request gpio spi_sdoi\n");
	if (gpio_request(GPIO_OUT_101, "gpio_dac"))
		pr_err("failed to request gpio_dac\n");
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_CFG(GPIO_OUT_132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_102, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_101,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void lcdc_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

static char *msm_fb_lcdc_vreg[] = {
    "ldo14", "ldo15"
};

static int msm_fb_lcdc_power_save(int on)
{
	struct vreg *vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)];
	int i, rc = 0;

   if(hw_version >= 5) return;

	for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
		if (on) {
			vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);
			rc = vreg_enable(vreg[i]);
			if (rc) {
				printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed \n",
						msm_fb_lcdc_vreg[i]);
				goto bail;
			}
            if (i==0){   
              vreg_set_level(vreg[i], OUT3000mV);

			}
            else if (i==1)
            vreg_set_level(vreg[i], OUT1800mV);
		}
        else{
			int tmp;
			vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);
			tmp = vreg_disable(vreg[i]);
			if (tmp) {
				printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed \n",
						msm_fb_lcdc_vreg[i]);
				if (!rc)
					rc = tmp;
			}
		}
	}	
	return rc;

bail:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(vreg[i - 1]);
	}

	return rc;
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = msm_fb_lcdc_config,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static struct msm_panel_common_pdata lcdc_panel_data = {
    .panel_config_gpio = lcdc_config_gpios,
	.gpio_num          = gpio_array_num,
};


static struct platform_device lcdc_s6d04d1_panel_device = {
    .name   = "lcdc_s6d04d1_wqvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_panel_data,
    }
};


static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

        if (!strcmp(name, "lcdc_s6d04d1_wqvga"))
			ret = 0;
		else
			ret = -ENODEV;
//  }

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_power_on[] = {
    GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* RFR */
    GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* CTS */
    GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* Rx */
    GPIO_CFG(46, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* Tx */
    GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* PCM_DOUT */
    GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* PCM_DIN */
    GPIO_CFG(70, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* PCM_SYNC */
    GPIO_CFG(71, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),     /* PCM_CLK */
};
static unsigned bt_config_power_off[] = {
    GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* RFR */
    GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* CTS */
    GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* Rx */
    GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* Tx */
    GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_DOUT */
    GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_DIN */
    GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_SYNC */
    GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),    /* PCM_CLK */
};
#if !defined(CONFIG_BCM4329)
void wlan_setup_clock(int on);
#endif
extern int bluesleep_start(void);
extern void bluesleep_stop(void);

static int bluetooth_power(int on)
{
#if 0 // MBdkhan fixme later
  struct vreg *vreg_bt;
#endif
	int pin, rc;

	printk(KERN_DEBUG "%s\n", __func__);

#if 0 // for Quattro
	/* do not have vreg bt defined, gp6 is the same */
	/* vreg_get parameter 1 (struct device *) is ignored */
	vreg_bt = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_bt)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_bt));
		return PTR_ERR(vreg_bt);
	}
#endif

	if (on) {
#if !defined(CONFIG_BCM4329)
		/* If WiFi isn't working, 
		   we should turn on the power for the clock supplied to BT */
		if (gpio_get_value(GPIO_WLAN_RESET) == 0)
			wlan_setup_clock(1);
#endif
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}
#if 0 // for Quattro
		/* units of mV, steps of 50 mV */
		rc = vreg_set_level(vreg_bt, 2600);
		if (rc) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else
#if 1 // MBdkhan fixme later
	if (gpio_request(GPIO_BT_WAKE, "wlan_ar6000_pm") < 0) {
				printk(KERN_ERR "GPIO_BT_WAKE gpio_request fail. \n");
			}
	if (gpio_request(GPIO_BT_WLAN_REG_ON, "wlan_ar6000_pm") < 0) {
				printk(KERN_ERR "GPIO_BT_WLAN_REG_ON gpio_request fail. \n");
			}
	if (gpio_request(GPIO_BT_RESET, "wlan_ar6000_pm") < 0) {
				printk(KERN_ERR "GPIO_BT_RESET gpio_request fail. \n");
			}
	
    gpio_direction_output(GPIO_BT_WAKE, GPIO_OUTPUT_HIGH);  /* BT_WAKE */
    gpio_direction_output(GPIO_BT_WLAN_REG_ON, GPIO_OUTPUT_HIGH); /* BT_VREG_CTL */
    mdelay(150);
    gpio_direction_output(GPIO_BT_RESET, GPIO_OUTPUT_HIGH);  /* BT_RESET */

    mdelay(100); //Delay between turning bluetooth power and starting bluesleep
#else
    gpio_configure(GPIO_BT_WAKE, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);  /* BT_WAKE */
    gpio_configure(GPIO_BT_WLAN_REG_ON, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH); /* BT_VREG_CTL */
    mdelay(150);
    gpio_configure(GPIO_BT_RESET, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);  /* BT_RESET */

    mdelay(100); //Delay between turning bluetooth power and starting bluesleep
#endif	
    bluesleep_start();
#endif
	} else {
#if 0 // for Quattro
		rc = vreg_disable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg disable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else
        bluesleep_stop();
        gpio_set_value(GPIO_BT_RESET, 0);/* BT_VREG_CTL */

#if defined(CONFIG_BCM4329) //MBjkseo 20110321 : After BT off, sleep current is 3mA high. because do not excute function below.
        if( gpio_get_value(GPIO_WLAN_RESET) == 0 ) //SEC_BLUETOOTH : pjh_2010.06.30
#endif        
        {
            gpio_set_value(GPIO_BT_WLAN_REG_ON, 0);/* BT_RESET */
            mdelay(150);
        }
        gpio_set_value(GPIO_BT_WAKE, 0);/* BT_VREG_CTL */
#endif
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}
	}
	return 0;
}

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

#ifdef CONFIG_ARCH_MSM7X27
static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_platform_data kgsl_pdata;

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};
#endif

static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};

#if 0 // for Quattro
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 83,
		.end	= 83,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(83),
		.end	= MSM_GPIO_TO_INT(83),
		.flags	= IORESOURCE_IRQ,
	},
};
#else
static struct resource bluesleep_resources[] = {
{
    .name = "gpio_host_wake",
    .start = GPIO_BT_HOST_WAKE,
    .end = GPIO_BT_HOST_WAKE,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "gpio_ext_wake",
    .start = GPIO_BT_WAKE,//81,//35,
    .end = GPIO_BT_WAKE,//81, //35,
    .flags = IORESOURCE_IO,
    },
    {
    .name = "host_wake",
    .start = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .end = MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE),
    .flags = IORESOURCE_IRQ,
    },
};
#endif

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

static struct i2c_board_info i2c_devices[] = {
#ifdef CONFIG_SR200PC10 //PGH
    {
        I2C_BOARD_INFO("sr200pc10", 0x40>>1),
    },
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#ifdef CONFIG_SENSORS_MMC31XX
    {
        I2C_BOARD_INFO(MMC31XX_I2C_NAME,  MMC31XX_I2C_ADDR),
    },
#endif
#ifdef CONFIG_SENSORS_KR3D_ACCEL
{
	I2C_BOARD_INFO("kr3dm_accel", KR3DM_SLAVE_ADDRESS>>1),
},
#endif

};


static struct i2c_board_info touch_i2c_devices[] = {
  {
    I2C_BOARD_INFO("cytma340", 0x20),  
        .irq = MSM_GPIO_TO_INT( TSP_INT ),       
  },
};

#ifdef CONFIG_FSA9280
static struct i2c_board_info mus_i2c_devices[] = {
  {
    I2C_BOARD_INFO("fsa9280",0x4A>>1),
  },
};
#endif  

static struct i2c_board_info fg_i2c_devices[] = {
  { 
    I2C_BOARD_INFO( "fuelgauge", 0x6D>>1 ),
  },
 }; //hanapark_fuelgauge

#ifdef CONFIG_MSM_CAMERA//aswoogi quattro setting
static uint32_t camera_off_gpio_table[] = {
/* parallel CAMERA interfaces */
    GPIO_CFG(0,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* nCAM_3M_RST */
    GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* CAM_EN */
    GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
    GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
    GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
    GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
    GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
    GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
    GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
    GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
    GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
    GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
    GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)  /* MCLK */
};
static uint32_t camera_on_gpio_table[] = {
    /* parallel CAMERA interfaces */
    GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
    GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
    GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
    GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
    GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
    GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
    GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
    GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
    GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), /* PCLK */
    GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

        rc = vreg_set_level(vreg_gp2, 2800);//aswoogi rant3 imsi 1800); //SecFeature aswoogi for RANT3
		if (rc) {
			printk(KERN_ERR "%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "gp3");
		if (IS_ERR(vreg_gp3)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

        rc = vreg_set_level(vreg_gp3, 2800);//aswoogi rant3 imsi 2850); //SecFeature aswoogi for RANT3
		if (rc) {
			printk(KERN_ERR "%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 enable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 disable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}

static int config_camera_on_gpios(void)
{
	int vreg_en = 1;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
	int vreg_en = 0;
	struct vreg *vreg_cam_28A,*vreg_cam_af30;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

}

static void reconfig_gpio(void)
{
  gpio_tlmm_config(GPIO_CFG(GPIO_JACK_S_35,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);    /* EAR DETECT*/

  gpio_tlmm_config(GPIO_CFG(GPIO_SEND_END,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);   /* SEND/END DETECT*/
}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

int pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	int rc;
	rc = pmic_flash_led_set_current(mA);
	return rc;
}

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 30,
	._fsrc.pmic_src.high_current = 100,
	._fsrc.pmic_src.led_src_1 = 0,
	._fsrc.pmic_src.led_src_2 = 0,
	._fsrc.pmic_src.pmic_set_current = pmic_set_flash_led_current,
};

#ifdef CONFIG_SR200PC10 //PGH
static struct msm_camera_sensor_flash_data flash_sr200pc10 = {
    .flash_type = MSM_CAMERA_FLASH_LED,
    .flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_sr200pc10_data = {
        .sensor_name    = "sr200pc10",
        .sensor_reset   = 0, 
        .sensor_pwd     = 2, //SecFeature aswoogi for RANT3
        .vcm_pwd        = 0, 
        .pdata          = &msm_camera_device_data,
        .flash_data     = &flash_sr200pc10
};
 
static struct platform_device msm_camera_sensor_sr200pc10 = {
        .name      = "msm_camera_sr200pc10",
        .dev       = {  
                .platform_data = &msm_camera_sensor_sr200pc10_data,
        },   
};
#endif
#if 0
//#ifdef FRONT_CAMERA
static struct msm_camera_sensor_flash_data flash_s5ka3d = {
    .flash_type = MSM_CAMERA_FLASH_LED,
    .flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5ka3d_data = {
    .sensor_name    = "s5ka3d",
    .sensor_reset   = 0,
    .sensor_pwd     = 2, //107 //aswoogi quattro setting
    .vcm_pwd        = 0,
    .pdata          = &msm_camera_device_data,
    .flash_data     = &flash_s5ka3d
};

static struct platform_device msm_camera_sensor_s5ka3d = {
    .name      = "msm_camera_s5ka3d",
    .dev       = {
        .platform_data = &msm_camera_sensor_s5ka3d_data,
    },
};
//#else
#endif
#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9t013
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif
#endif

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

static struct platform_device msm_vibrator_device = {
    .name           = "msm_vibrator",
    .id         = -1,
};

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static struct i2c_gpio_platform_data touch_i2c_gpio_data = {
    .sda_pin    = 29,//TSP_SDA,
    .scl_pin    = 30,//TSP_SCL,
    .scl_is_output_only     = 1,
};

static struct platform_device touch_i2c_gpio_device = {  
    .name       = "i2c-gpio",
    .id     = 2,
    .dev        = {
        .platform_data  = &touch_i2c_gpio_data,
    },
};

static struct platform_device msm_wlan_pm_device = {
    .name       = "wlan_ar6000_pm",
    .id         = -1,
};

#if defined(CONFIG_FSA9280)
static struct i2c_gpio_platform_data fsa9280_i2c_gpio_data = {
    .sda_pin  = IF_SDA, 
    .scl_pin  = IF_SCL, 
};

static struct platform_device fsa9280_i2c_gpio_device = {  
    .name       = "i2c-gpio",
    .id     = 3,
    .dev        = {
        .platform_data  = &fsa9280_i2c_gpio_data,
    },
};

#endif

static struct i2c_gpio_platform_data fuelgauge_i2c_gpio_data = {
    .sda_pin  = I2C_2_SDA,
    .scl_pin  = I2C_2_SCL,
};  // hanapark_fuelgauge

static struct platform_device fuelgauge_i2c_gpio_device = {  
    .name       = "i2c-gpio",
    .id     = 6,
    .dev        = {
        .platform_data  = &fuelgauge_i2c_gpio_data,
    },
};	// hanapark_fuelgauge


#if defined(CONFIG_BCM4329)
static struct platform_device sec_device_wifi;
#endif

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif

#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
	&msm_device_i2c,
    &touch_i2c_gpio_device,
#ifdef CONFIG_FSA9280 
    &fsa9280_i2c_gpio_device,
#endif  

	&fuelgauge_i2c_gpio_device,	// hanapark_fuelgauge
	&smc91x_device,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_fb_device,
    &lcdc_s6d04d1_panel_device,
	&msm_device_uart_dm1,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&msm_device_pmic_leds,
	&msm_device_snd,
	&msm_device_adspdec,

#ifdef CONFIG_SR200PC10 //PGH
    &msm_camera_sensor_sr200pc10,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
	&msm_bluesleep_device,
#ifdef CONFIG_ARCH_MSM7X27
	&msm_device_kgsl,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
	&hs_device,
	&msm_batt_device,
	&msm_wlan_pm_device,
    &msm_vibrator_device,
#if defined(CONFIG_BCM4329) //#ifdef CONFIG_WIFI_CONTROL_FUNC
    &sec_device_wifi,
#endif
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", 0);
	msm_fb_register_device("lcdc", &lcdc_pdata);
}

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x2x_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 400000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static unsigned mpp_mmc = 2;

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(21, 4, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
#if !defined(CONFIG_BCM4329)
		.sleep_cfg_data = sdc2_sleep_cfg_data,
#else
		.sleep_cfg_data = NULL,//SecFeatue.WIFI sdc2_sleep_cfg_data,
#endif
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (pdev->id != 1)
		return 0;

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			if (machine_is_msm7x25_ffa() ||
					machine_is_msm7x27_ffa()) {
				rc = mpp_config_digital_out(mpp_mmc,
				     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				     MPP_DLOGIC_OUT_CTRL_LOW));
			} else {
				rc = vreg_disable(vreg_mmc);
			}
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
			rc = mpp_config_digital_out(mpp_mmc,
			     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			     MPP_DLOGIC_OUT_CTRL_HIGH));
		} else {
		    rc = vreg_set_level(vreg_mmc, OUT3000mV);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int sdcc_slot_status(struct device *dev)
{
	int rc;

	rc = gpio_get_value(T_FLASH_DET);

    rc = rc?0:1 ;
    return rc;
}
#endif

#ifndef ATH_POLLING
static void (*wlan_status_notify_cb)(int card_present, void *dev_id);
void *wlan_devid;

static int register_wlan_status_notify(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
    wlan_status_notify_cb = callback;
    wlan_devid = dev_id;
    return 0;
}
static unsigned int wlan_status(struct device *dev)
{
    int rc;
    rc = gpio_get_value(GPIO_WLAN_RESET);

    return rc;
}
#endif /* ATH_POLLING */

#if defined(CONFIG_BCM4329) //#ifdef CONFIG_WIFI_CONTROL_FUNC
static uint32_t wlan_gpio_table[] = {
	/* GPIO_WLAN_WAKES_MSM */
	GPIO_CFG(GPIO_WLAN_WAKES_MSM, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* BT_WLAN_REG_ON */
	GPIO_CFG(GPIO_BT_WLAN_REG_ON, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* GPIO_WLAN_RESET */
	GPIO_CFG(GPIO_WLAN_RESET, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
};

static void config_wlan_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int wlan_init (void)
{
	gpio_direction_output(GPIO_BT_WLAN_REG_ON, GPIO_WLAN_LEVEL_LOW);
	msleep(100);
	gpio_direction_output(GPIO_WLAN_RESET, GPIO_WLAN_LEVEL_LOW);
	msleep (100);
}

static int wlan_power_en(int onoff)
{
	if (onoff) {
		if (gpio_get_value (GPIO_BT_WLAN_REG_ON) == GPIO_WLAN_LEVEL_LOW)
			gpio_set_value (GPIO_BT_WLAN_REG_ON, GPIO_WLAN_LEVEL_HIGH);
		msleep(150);
		gpio_set_value (GPIO_WLAN_RESET, GPIO_WLAN_LEVEL_HIGH);
		msleep (150);
	} else {
		gpio_set_value (GPIO_WLAN_RESET, GPIO_WLAN_LEVEL_LOW);
		msleep (150);
		if (gpio_get_value(GPIO_BT_RESET) == GPIO_WLAN_LEVEL_LOW) {
			msleep(150);
			gpio_set_value (GPIO_BT_WLAN_REG_ON, GPIO_WLAN_LEVEL_LOW);
		}
	}

#ifndef ATH_POLLING
			msleep(250);
#endif /* ATH_POLLING */

	/* Detect card */
	if (wlan_status_notify_cb){

		printk(KERN_ERR "wlan_carddetect_en2 = %d ~~~\n", onoff);
		wlan_status_notify_cb(onoff, wlan_devid);
		}
	else
		printk(KERN_ERR "WLAN: No notify available\n");

	return 0;
}

static int wlan_reset_en(int onoff)
{
	gpio_direction_output(GPIO_WLAN_RESET,
			onoff ? GPIO_WLAN_LEVEL_HIGH : GPIO_WLAN_LEVEL_LOW);
	return 0;
}

static int wlan_carddetect_en(int onoff)
{
	printk(KERN_ERR "wlan_carddetect_en = %d ~~~\n", onoff);
    if(!onoff) msleep(500);
	return 0;
}

static struct resource wifi_resources[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= MSM_GPIO_TO_INT(GPIO_WLAN_WAKES_MSM),
		.end	= MSM_GPIO_TO_INT(GPIO_WLAN_WAKES_MSM),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(wlan_mem_prealloc);


#define DHD_SKB_HDRSIZE 		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

static struct wifi_platform_data wifi_pdata = {
	.set_power		    = wlan_power_en,
	.set_reset		    = wlan_reset_en,
	.set_carddetect		= wlan_carddetect_en,
	.mem_prealloc		= wlan_mem_prealloc,
};


static struct platform_device sec_device_wifi = {
	.name			= "bcm4329_wlan",
	.id			    = 1,
	.num_resources	= ARRAY_SIZE(wifi_resources),
	.resource		= wifi_resources,
	.dev			= {
		.platform_data = &wifi_pdata,
	},
};
#endif

#if !defined(CONFIG_BCM4329)
#define WLAN_HOST_WAKE 
#endif
#ifdef WLAN_HOST_WAKE
struct wlansleep_info {
    unsigned host_wake;
    unsigned host_wake_irq;
    struct wake_lock wake_lock;
};

static struct wlansleep_info *wsi;
static struct tasklet_struct hostwake_task;

static void wlan_hostwake_task(unsigned long data)
{
    printk(KERN_INFO "WLAN: wake lock timeout 0.5 sec...\n");

    wake_lock_timeout(&wsi->wake_lock, HZ / 2);
}

static irqreturn_t wlan_hostwake_isr(int irq, void *dev_id)
{   
//please fix    gpio_clear_detect_status(wsi->host_wake_irq);

    /* schedule a tasklet to handle the change in the host wake line */
    tasklet_schedule(&hostwake_task);
    return IRQ_HANDLED;
}

static int wlan_host_wake_init(void)
{
    int ret;
    
    wsi = kzalloc(sizeof(struct wlansleep_info), GFP_KERNEL);
    if (!wsi)
        return -ENOMEM;

    wake_lock_init(&wsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
    tasklet_init(&hostwake_task, wlan_hostwake_task, 0);

    wsi->host_wake = 17;
    wsi->host_wake_irq = MSM_GPIO_TO_INT(wsi->host_wake);
    
//please fix    gpio_configure(wsi->host_wake, GPIOF_INPUT);    
    ret = request_irq(wsi->host_wake_irq, wlan_hostwake_isr,
                IRQF_DISABLED | IRQF_TRIGGER_RISING,
                "wlan hostwake", NULL);
    if (ret < 0) {
        printk(KERN_ERR "WLAN: Couldn't acquire WLAN_HOST_WAKE IRQ");
        return -1;
    }

    ret = enable_irq_wake(wsi->host_wake_irq);
    if (ret < 0) {
        printk(KERN_ERR "WLAN: Couldn't enable WLAN_HOST_WAKE as wakeup interrupt");
        free_irq(wsi->host_wake_irq, NULL);
        return -1;
    }
    
    return 0;
}

static void wlan_host_wake_exit(void)
{
    if (disable_irq_wake(wsi->host_wake_irq))
        printk(KERN_ERR "WLAN: Couldn't disable hostwake IRQ wakeup mode \n");
    free_irq(wsi->host_wake_irq, NULL);

    wake_lock_destroy(&wsi->wake_lock);
    kfree(wsi); 
}
#endif /* WLAN_HOST_WAKE */

#if !defined(CONFIG_BCM4329)
void wlan_setup_clock(int on)
{
	struct vreg *vwlan_3_3v;	
	struct vreg *vwlan_1_8v;

	vwlan_3_3v = vreg_get(NULL, "ldo13");
	if (IS_ERR(vwlan_3_3v)) {
	printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		   __func__, PTR_ERR(vwlan_3_3v));
	return;
	}

	vwlan_1_8v = vreg_get(NULL, "ldo10");
	if (IS_ERR(vwlan_1_8v)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
               __func__, PTR_ERR(vwlan_1_8v));
        return;
    }

    printk("%s %s --enter\n", __func__, on ? "on" : "down");

	if (on) {
		vreg_set_level(vwlan_3_3v, OUT3300mV);
		vreg_enable(vwlan_3_3v);		
		vreg_set_level(vwlan_1_8v, OUT1800mV);
		vreg_enable(vwlan_1_8v);

    }
}
#endif

void wlan_setup_power(int on, int detect)
{
	static int is_gpio_init = 0;
	printk("%s %s --enter\n", __func__, on ? "on" : "down");

	if(is_gpio_init == 0)
	{
		if (gpio_request(GPIO_WLAN_RESET, "wlan_ar6000_pm") < 0) {
				printk(KERN_ERR "GPIO_WLAN_RESET gpio_request fail. \n");
		}
		is_gpio_init = 1;
	}

#if defined(CONFIG_BCM4329)
	
	if (detect != 1) {
			printk(/*KERN_DEBUG*/ "(on=%d, detect=%d)\n", on, detect);
	//For Starting/Stopping Tethering service
#if 1
			if (on)
				gpio_direction_output(GPIO_WLAN_RESET, 1);
			else
				gpio_direction_output(GPIO_WLAN_RESET, 0);
#endif
			return;
	}
#endif
    if (on) {
#if defined(CONFIG_BCM4329)
		gpio_direction_output(GPIO_BT_WLAN_REG_ON, 1);	/* BT_WLAN_REG_ON */
		msleep(150);	
#else
        if (gpio_get_value(BT_PWR) == 0) {          
            wlan_setup_clock(1);    
			mdelay(30);
		}		
#endif
		gpio_direction_output(GPIO_WLAN_RESET, 1);	/* WLAN_RESET */
      
#ifdef WLAN_HOST_WAKE
		wlan_host_wake_init();
#endif /* WLAN_HOST_WAKE */
	}
	else {
#ifdef WLAN_HOST_WAKE
		wlan_host_wake_exit();
#endif /* WLAN_HOST_WAKE */
		gpio_direction_output(GPIO_WLAN_RESET, 0);	/* WLAN_RESET */


		if(gpio_get_value(GPIO_BT_RESET) == 0)
		{
            msleep(150);
			
			if (gpio_request(GPIO_BT_WLAN_REG_ON, "wlan_ar6000_pm") < 0) {
				printk(KERN_ERR "GPIO_BT_WLAN_REG_ON gpio_request fail. \n");
			}
			
            gpio_direction_output(GPIO_BT_WLAN_REG_ON, 0);	/* BT_WLAN_REG_ON */
		}
	}	
#ifndef ATH_POLLING
#if defined(CONFIG_BCM4329)
		msleep(250);
#else
    mdelay(100);
    if (detect)
#endif
   {
        /* Detect card */
        if (wlan_status_notify_cb)
            wlan_status_notify_cb(on, wlan_devid);
        else
            printk(KERN_ERR "WLAN: No notify available\n");
    }
#endif /* ATH_POLLING */
}
EXPORT_SYMBOL(wlan_setup_power);


#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm7x2x_sdc1_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
    .status         = sdcc_slot_status,
    .irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x2x_sdc2_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifndef ATH_POLLING
    .status = wlan_status,
    .register_status_notify = register_wlan_status_notify,
#endif /* ATH_POLLING */
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
#if defined(CONFIG_BCM4329)
//SecFeature.WIFI	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#else
    .sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,//SecFeature.WIFI
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,//SecFeature.WIFI
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x2x_sdc3_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,

#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x2x_sdc4_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

static void __init msm7x2x_init_mmc(void)
{
#if defined(CONFIG_BCM4329)
    wlan_init();
#endif
	if (!machine_is_msm7x25_ffa() && !machine_is_msm7x27_ffa()) {

        vreg_mmc = vreg_get(NULL, "ldo16");

		if (IS_ERR(vreg_mmc)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			       __func__, PTR_ERR(vreg_mmc));
			return;
		}
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
  gpio_tlmm_config(GPIO_CFG(T_FLASH_DET, 0, GPIO_CFG_INPUT,GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
  msm7x2x_sdc1_data.status_irq = MSM_GPIO_TO_INT(T_FLASH_DET);
	msm_add_sdcc(1, &msm7x2x_sdc1_data);
#endif

	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf() ||
		machine_is_msm7x27_ffa()) {
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
		msm_sdcc_setup_gpio(2, 1);
		msm_add_sdcc(2, &msm7x2x_sdc2_data);
#endif
	}

	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf()) {
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
		msm_add_sdcc(3, &msm7x2x_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		msm_add_sdcc(4, &msm7x2x_sdc4_data);
#endif
	}
}
#else
#define msm7x2x_init_mmc() do {} while (0)
#endif


static struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
/* for SIM recognization */
#if 0
		gpio_scl = 95;
		gpio_sda = 96;
#endif
        return;
	} else {
		gpio_scl = I2C_SCL;
		gpio_sda = I2C_SDA;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
#if 1//PCAM : Fast I2C
	.clk_freq = 380000,
#else//ORG : Normal I2C
	.clk_freq = 100000,
#endif//PCAM	.rmutex  = 0,
	.pri_clk = I2C_SCL,
	.pri_dat = I2C_SDA,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(I2C_SCL, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(I2C_SDA, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
/* for SIM recognization */
#if 0
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");
#endif

	if (cpu_is_msm7x27())
		msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	else
		msm_i2c_pdata.pm_lat =
		msm7x25_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 7;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
			MPP_CFG(MPP_DLOGIC_LVL_VDD,
				MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
				"to enable 3.3V LDO failed\n", __func__);
	}
}

#if defined(CONFIG_MSM_ARM9_USES_UART3)
extern int arm9_uses_uart3; 
#endif

unsigned int customer_binary=0;
static int __init customer_download(char *rooting)
{
	if(!strcmp(rooting,"custom_bin")){
		customer_binary = 1;
	}
	return 1;
}
__setup("androidboot.custom_kernel=", customer_download);

samsung_vendor1_id *Quattro_vendor1_id;
EXPORT_SYMBOL(Quattro_vendor1_id);

extern void * klog_buf_addr(void);
extern void * p_main_addr(void);
extern void * p_radio_addr(void);
extern void * p_events_addr(void);
extern void * p_system_addr(void);

static void mark_getlog(void)
{
    Quattro_vendor1_id = (samsung_vendor1_id *)smem_find(SMEM_ID_VENDOR1, sizeof(samsung_vendor1_id));

    //kernel
    Quattro_vendor1_id->klog_mark.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0));
    Quattro_vendor1_id->klog_mark.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0));
    Quattro_vendor1_id->klog_mark.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0));
    Quattro_vendor1_id->klog_mark.special_mark_4 = (('k' << 24) | ('l' << 16) | ('o' << 8) | ('g' << 0));
    Quattro_vendor1_id->klog_mark.p__log_buf = klog_buf_addr();

    //platform
    Quattro_vendor1_id->plog_mark.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0));
    Quattro_vendor1_id->plog_mark.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0));
    Quattro_vendor1_id->plog_mark.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0));
    Quattro_vendor1_id->plog_mark.special_mark_4 = (('p' << 24) | ('l' << 16) | ('o' << 8) | ('g' << 0));
    Quattro_vendor1_id->plog_mark.p_main = p_main_addr();
    Quattro_vendor1_id->plog_mark.p_radio = p_radio_addr();
    Quattro_vendor1_id->plog_mark.p_events = p_events_addr();
	Quattro_vendor1_id->plog_mark.p_system = p_system_addr();	

    //version
    Quattro_vendor1_id->vlog_mark.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0));
    Quattro_vendor1_id->vlog_mark.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0));
    Quattro_vendor1_id->vlog_mark.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0));
    Quattro_vendor1_id->vlog_mark.special_mark_4 = (('v' << 24) | ('e' << 16) | ('r' << 8) | ('s' << 0));
    Quattro_vendor1_id->vlog_mark.log_mark_version = 1;
    Quattro_vendor1_id->vlog_mark.framebuffer_mark_version = 1;
    Quattro_vendor1_id->vlog_mark.this = (&Quattro_vendor1_id->vlog_mark + 0x03000000);

    if (Quattro_vendor1_id->hw_version > 0)
        Quattro_vendor1_id->vlog_mark.first_size = (256+128)*1024*1024;
    else
        Quattro_vendor1_id->vlog_mark.first_size = 256*1024*1024;

    Quattro_vendor1_id->vlog_mark.first_start_addr = 0x03000000;
    Quattro_vendor1_id->vlog_mark.second_size = 0;
    Quattro_vendor1_id->vlog_mark.second_start_addr = 0;

    //frame buffer
    Quattro_vendor1_id->flog_mark.special_mark_1 = (('*' << 24) | ('^' << 16) | ('^' << 8) | ('*' << 0));
    Quattro_vendor1_id->flog_mark.special_mark_2 = (('I' << 24) | ('n' << 16) | ('f' << 8) | ('o' << 0));
    Quattro_vendor1_id->flog_mark.special_mark_3 = (('H' << 24) | ('e' << 16) | ('r' << 8) | ('e' << 0));
	Quattro_vendor1_id->flog_mark.special_mark_4 = (('f' << 24) | ('b' << 16) | ('u' << 8) | ('f' << 0));
	Quattro_vendor1_id->flog_mark.p_fb   = 0;
	Quattro_vendor1_id->flog_mark.resX   = 320;
	Quattro_vendor1_id->flog_mark.resY   = 480;
	Quattro_vendor1_id->flog_mark.bpp    = 24;
	Quattro_vendor1_id->flog_mark.frames = 2;
}

#if defined(CONFIG_BCM4329)
int __init aries_init_wifi_mem(void)
{
	int i;
	int j;

	printk("aries_init_wifi_mem!!!!!!!!!!!!!\n");
	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}
	
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		wifi_mem_array[i].mem_ptr = 
			kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0; j < i; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
// use wlan static buffer ]
#endif

static void __init msm7x2x_init(void)
{
	unsigned size;

	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);



#if defined(CONFIG_SMC91X)
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		smc91x_resources[0].start = 0x98000300;
		smc91x_resources[0].end = 0x980003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(85);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(85);
		if (gpio_tlmm_config(GPIO_CFG(85, 0,
					      GPIO_CFG_INPUT,
					      GPIO_CFG_PULL_DOWN,
					      GPIO_CFG_2MA),
				     GPIO_CFG_ENABLE)) {
			printk(KERN_ERR
			       "%s: Err: Config GPIO-85 INT\n",
				__func__);
		}
	}
#endif

	if (cpu_is_msm7x27())
		msm7x2x_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&msm7x2x_clock_data);

#ifdef CONFIG_ARCH_MSM7X27
	/* This value has been set to 160000 for power savings. */
	/* OEMs may modify the value at their discretion for performance */
	/* The appropriate maximum replacement for 160000 is: */
	/* clk_get_max_axi_khz() */
	kgsl_pdata.high_axi_3d = 160000;

	/* 7x27 doesn't allow graphics clocks to be run asynchronously to */
	/* the AXI bus */
	kgsl_pdata.max_grp2d_freq = 0;
	kgsl_pdata.min_grp2d_freq = 0;
	kgsl_pdata.set_grp2d_async = NULL;
	kgsl_pdata.max_grp3d_freq = 0;
	kgsl_pdata.min_grp3d_freq = 0;
	kgsl_pdata.set_grp3d_async = NULL;
	kgsl_pdata.imem_clk_name = "imem_clk";
	kgsl_pdata.grp3d_clk_name = "grp_clk";
	kgsl_pdata.grp3d_pclk_name = "grp_pclk";
	kgsl_pdata.grp2d0_clk_name = NULL;
	kgsl_pdata.idle_timeout_3d = HZ/5;
	kgsl_pdata.idle_timeout_2d = 0;

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	kgsl_pdata.pt_va_size = SZ_32M;
        /* Maximum of 32 concurrent processes */
        kgsl_pdata.pt_max_count = 32;
#else
	kgsl_pdata.pt_va_size = SZ_128M;
	//kgsl_pdata.pt_va_size = SZ_64M;
	//kgsl_pdata.pt_va_size = (SZ_128M+SZ_64M);
        /* We only ever have one pagetable for everybody */
        kgsl_pdata.pt_max_count = 1;
#endif
#endif
	usb_mpp_init();

#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;

	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	if (machine_is_msm7x25_surf() || machine_is_msm7x25_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_75_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
		msm_otg_pdata.phy_reset = msm_otg_rpc_phy_reset;
	}
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_75_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_DISABLE;
		msm_otg_pdata.phy_reset_sig_inverted = 1;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_gadget_pdata.is_phy_status_timer_on = 1;
#endif
#endif

    size = sizeof(samsung_vendor1_id);
    Quattro_vendor1_id = (samsung_vendor1_id *)smem_get_entry(SMEM_ID_VENDOR1, &size);
    hw_version = Quattro_vendor1_id->hw_version;
    printk("hw_version = %d\n", hw_version);
    system_rev = hw_version;
    
    mark_getlog(); /* Mark for GetLog */

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif

	/* Add the Devices */

	{
	platform_add_devices(devices, ARRAY_SIZE(devices));
	}

#if defined(CONFIG_MSM_ARM9_USES_UART3)
    if ( !arm9_uses_uart3 )
        platform_device_register(&msm_device_uart3);
#endif

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif

#if defined(CONFIG_MACH_VINO)
    reconfig_gpio();
#endif



	msm_device_i2c_init();

	{
		i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	}


	{
		i2c_register_board_info(2, touch_i2c_devices, ARRAY_SIZE(touch_i2c_devices));
	}

#ifdef CONFIG_FSA9280
    i2c_register_board_info(3, mus_i2c_devices, ARRAY_SIZE(mus_i2c_devices));
#endif

	i2c_register_board_info(6, fg_i2c_devices, ARRAY_SIZE(fg_i2c_devices));	// hanapark_fuelgauge

	if(1)
	{
		;
	}
	else

	{
	}

  if( hw_version == 0 )
    platform_device_register(&keypad_device_vino_rev0);
  else if((hw_version == 1)||(hw_version == 2) )
    platform_device_register(&keypad_device_vino);
  else if(hw_version <=8 && hw_version >2)
     platform_device_register(&keypad_device_vino_rev_03); 
  else //(hw_version >=9)
     platform_device_register(&keypad_device_vino_rev_09); 
	lcdc_gordon_gpio_init();
	msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM
	msm7x2x_init_host();
#endif
	msm7x2x_init_mmc();
	bt_power_init();
#ifdef CONFIG_SAMSUNG_JACK
	sec_jack_gpio_init();
	platform_device_register(&sec_device_jack);
#endif

	if (cpu_is_msm7x27())
		msm_pm_set_platform_data(msm7x27_pm_data,
					ARRAY_SIZE(msm7x27_pm_data));
	else
		msm_pm_set_platform_data(msm7x25_pm_data,
					ARRAY_SIZE(msm7x25_pm_data));
#if 0 // MBdkhan fixme later
	msm7x27_wlan_init();
#endif	
#if defined(CONFIG_BCM4329)
	aries_init_wifi_mem();	 //use wlan static buffer
#endif
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes (at %lx physical) for audio "
			"pmem arena\n", size , __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();
	msm_msm7x2x_allocate_memory_regions();

	if (socinfo_init() < 0)
		BUG();

#ifdef CONFIG_CACHE_L2X0
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa()) {
		/* 7x27 has 256KB L2 cache:
			64Kb/Way and 4-Way Associativity;
			evmon/parity/share disabled. */
		if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) > 1)
			|| ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
			&& (SOCINFO_VERSION_MINOR(socinfo_get_version()) >= 3)))
			/* R/W latency: 4 cycles; */
			l2x0_init(MSM_L2CC_BASE, 0x0006801B, 0xfe000000);
		else
			/* R/W latency: 3 cycles; */
			l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
	}
#endif
}

MACHINE_START(MSM7X27_SURF, "SPH-M820")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
