/****************************************************************************

**

** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2010 ALL RIGHTS RESERVED

**

** AUTHOR       : Kim, Geun-Young <geunyoung.kim@samsung.com>			@LDK@

**                                                                      @LDK@

****************************************************************************/
//#define _DEBUG
// hsil
#define MSM7201A
#define _ENABLE_ERROR_DEVICE

/* HSDPA DUN & Internal FTP Throughput Support. @LDK@ */
#define _HSDPA_DPRAM

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mm.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/irq.h>

#ifdef _ENABLE_ERROR_DEVICE
#include <linux/poll.h>
#include <linux/cdev.h>
#endif	/* _ENABLE_ERROR_DEVICE */

#include <asm/irq.h>
// hsil 
#include <asm/io.h>

//#include <asm/hardware.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
//#include <asm/arch/mux.h>
#include <mach/gpio.h>
//hsil
#include <mach/msm_iomap.h>

#ifdef CONFIG_EVENT_LOGGING
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/klog.h>
#include <asm/unistd.h>
#endif	/* CONFIG_EVENT_LOGGING */

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif	/* CONFIG_PROC_FS */
#include <linux/wakelock.h>

#include "dpram.h"
// DGS //open
#include "./fsr/FSR.h"
#include "./fsr/FSR_BML.h"
#include "./fsr/FSR_LLD_4K_OneNAND.h"
// hsil
//#include "smd_private.h"
#include "../../arch/arm/mach-msm/smd_private.h"
#include "../../arch/arm/mach-msm/proc_comm.h"

//screen capture //open
//#include "../../../hardware/ril/secril_multi/ioctl_sec.h"

#define DPRAM_HIGH_LATENCY_PATH_FOR_MUTEX_FAIL

#if 0
#if (CONFIG_NOWPLUS_REV >= CONFIG_NOWPLUS_REV02)
//#define _ENABLE_ONEDRAM_DEVICE	//hsil
#endif
#endif

#if 0
#define PRINT_WRITE
#define PRINT_READ
#define PRINT_RECV_IRQ
#define PRINT_SEND_IRQ
#endif

#define DRIVER_ID			"$Id: dpram.c,v 3.00 2008/09/09 08:00:00 $"
#define DRIVER_NAME 		"DPRAM"
#define DRIVER_PROC_ENTRY	"driver/dpram"
#define DRIVER_MAJOR_NUM	255

#ifdef CONFIG_EVENT_LOGGING
#define DPRAM_ID			3
#define DPRAM_READ			3
#define DPRAM_WRITE			4
#endif	/* CONFIG_EVENT_LOGGING */

#undef _DEBUG
#ifdef _DEBUG
#define dprintk(s, args...) printk("[DPRAM] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif	/* _DEBUG */

#define WRITE_TO_DPRAM(dest, src, size) \
	_memcpy((void *)(SmemBase + dest), src, size)

#define READ_FROM_DPRAM(dest, src, size) \
	_memcpy(dest, (void *)(SmemBase + src), size)

#define NUM_PDP_CONTEXT			8

#ifdef _ENABLE_ERROR_DEVICE
#define DPRAM_ERR_MSG_LEN			65
#define DPRAM_ERR_DEVICE			"dpramerr"
#endif	/* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
#define ONEDRAM_MAJOR_NUM	244
#define ONEDRAM_DEVICE				"onedram"

#define od_writeb(b,addr) (*(volatile u8 *) (addr) = (b))
#define od_writew(b,addr) (*(volatile u16 *) (addr) = (b))
#define od_writel(b,addr) (*(volatile u32 *) (addr) = (b))
#define od_writeq(b,addr) (*(volatile u64 *) (addr) = (b))
#endif


#define DEV_TYPE_SERIAL			1 /* serial device for CSD */

/* Maximum number of PDP context */
#define MAX_PDP_CONTEXT			10

/* Maximum PDP data length */
#define MAX_PDP_DATA_LEN		1500

/* Maximum PDP packet length including header and start/stop bytes */
#define MAX_PDP_PACKET_LEN		(MAX_PDP_DATA_LEN + 4 + 2)

/* PDP information table */
static struct pdp_info *pdp_table[MAX_PDP_CONTEXT];

/* Multiple PDP */
typedef struct pdp_arg {
	unsigned char	id;
	char		ifname[16];
} __attribute__ ((packed)) pdp_arg_t;

/* PDP data packet header format */
struct pdp_hdr {
	u16	len;		/* Data length */
	u8	id;			/* Channel ID */
	u8	control;	/* Control field */
} __attribute__ ((packed));

/* PDP information type */
struct pdp_info {
	/* PDP context ID */
	u8		id;

	/* Device type */
	unsigned		type;

	/* Device flags */
	unsigned		flags;

	/* Tx packet buffer */
	u8		*tx_buf;

	/* App device interface */
	union {
		/* Virtual serial interface */
		struct {
			struct tty_driver	tty_driver[NUM_PDP_CONTEXT];	// CSD, CDMA, TRFB, CIQ
			int			refcount;
			struct tty_struct	*tty_table[1];
			struct ktermios		*termios[1];
			struct ktermios		*termios_locked[1];
			char			tty_name[16];
			struct tty_struct	*tty;
			struct semaphore	write_lock;
		} vs_u;
	} dev_u;
#define vn_dev		dev_u.vnet_u
#define vs_dev		dev_u.vs_u
};


/* Device flags */
#define DEV_FLAG_STICKY			0x1 /* Sticky */

/* Device major & minor number */
//#define CSD_MAJOR_NUM			245
#define CSD_MAJOR_NUM			235
#define CSD_MINOR_NUM			0

//hsil 
//#define writel(b,addr) (*(volatile u32 *) (addr) = (b))
#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

#ifdef TARGET_BUILD_USER
#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_ROOKIE) ||  defined(CONFIG_MACH_GIO) ||  defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIOS)
#define RAMDUMP_LEVEL 0
#else
#define RAMDUMP_LEVEL 2
#endif
#else
#define RAMDUMP_LEVEL 2
#endif

int dump_enable = RAMDUMP_LEVEL;

//hsil
static volatile unsigned char *SmemBase;
static int DpramInited = 0;

/* DGS Info Cache */
static unsigned char aDGSBuf[4096];

#define DGS_TEST 0
static struct tty_driver *dpram_tty_driver;
static dpram_tasklet_data_t dpram_tasklet_data[MAX_INDEX];
static dpram_device_t dpram_table[MAX_INDEX] = {
	{
		.in_head_addr = DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_FORMATTED_SIZE,

		.out_head_addr = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_FORMATTED_SIZE,

		.mask_req_ack = INT_MASK_REQ_ACK_F,
		.mask_res_ack = INT_MASK_RES_ACK_F,
		.mask_send = INT_MASK_SEND_F,
	},
	{
		.in_head_addr = DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_RAW_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_RAW_SIZE,

		.out_head_addr = DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_RAW_SIZE,

		.mask_req_ack = INT_MASK_REQ_ACK_R,
		.mask_res_ack = INT_MASK_RES_ACK_R,
		.mask_send = INT_MASK_SEND_R,
	},
};

static struct tty_struct *dpram_tty[MAX_INDEX];
static struct ktermios *dpram_termios[MAX_INDEX];
static struct ktermios *dpram_termios_locked[MAX_INDEX];

// screen capture
static void* smem_vendor1_lcd_buf = NULL;

// hsil 
extern void *smem_alloc(unsigned, unsigned);
//extern void enable_dpram_pins(void);
static void print_smem(void);
static void dpram_ramdump(void);
static void dpram_drop_data(dpram_device_t *device);
static inline struct pdp_info * pdp_get_dev(u8 id);
static int multipdp_init(void);

//Get charging status & charger Connect value!!!
extern void get_charger_type(void);
extern void msm_batt_check_event(void);
extern int get_charging_status(void);

//extern void enable_dpram_pins(void);
static void print_smem(void);
static void dpram_ramdump(void);
static int dpram_get_dgs(void);

static void res_ack_tasklet_handler(unsigned long data);
static void fmt_rcv_tasklet_handler(unsigned long data);
static void raw_rcv_tasklet_handler(unsigned long data);

// [BML functions for DGS //open
INT32   (*bml_open)(UINT32        nVol, UINT32        nFlag);
VOID    (*bml_acquireSM)(UINT32        nVol);
INT32   (*ond_4k_read)(UINT32 nDev, UINT32 nPbn, UINT32 nPgOffset, UINT8 *pMBuf, FSRSpareBuf *pSBuf, UINT32 nFlag);
VOID    (*bml_release)(UINT32        nVol);
EXPORT_SYMBOL(bml_open);
EXPORT_SYMBOL(bml_acquireSM);
EXPORT_SYMBOL(ond_4k_read);
EXPORT_SYMBOL(bml_release);
// ]

static DECLARE_TASKLET(fmt_send_tasklet, fmt_rcv_tasklet_handler, 0);
static DECLARE_TASKLET(raw_send_tasklet, raw_rcv_tasklet_handler, 0);

static DECLARE_TASKLET(fmt_res_ack_tasklet, res_ack_tasklet_handler,
		(unsigned long)&dpram_table[FORMATTED_INDEX]);
static DECLARE_TASKLET(raw_res_ack_tasklet, res_ack_tasklet_handler,
		(unsigned long)&dpram_table[RAW_INDEX]);

#ifdef _ENABLE_ERROR_DEVICE
static unsigned int dpram_err_len;
static char dpram_err_buf[DPRAM_ERR_MSG_LEN];

struct class *dpram_class;

static DECLARE_WAIT_QUEUE_HEAD(dpram_err_wait_q);
static struct fasync_struct *dpram_err_async_q;
#endif	/* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
struct class *onedram_class;
#endif /* _ENABLE_ONEDRAM_DEVICE */

// 2008.12.09.
static DECLARE_MUTEX(write_mutex);
static DEFINE_MUTEX(pdp_lock);

struct wake_lock imei_wake_lock;
struct wake_lock dpram_wake_lock;
struct wake_lock silent_wake_lock;

struct workqueue_struct *dpram_work_queue;
struct work_struct dpram_irq_work;
static u16 request_command;

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
extern struct class *sec_class;
#else
struct class *sec_class_1;
EXPORT_SYMBOL(sec_class_1);
#endif

#define POWER_DOWN_TIME ( 20 * HZ )
struct device *pm_dev;
struct timer_list power_down_timer;
extern samsung_vendor1_id *Quattro_vendor1_id;

#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
int (*set_ram_dump_level)(int ram_dump_level);
EXPORT_SYMBOL(set_ram_dump_level);
int (*get_ram_dump_level)(void);
EXPORT_SYMBOL(get_ram_dump_level);
#endif

void power_down_timeout(unsigned long arg)
{
        printk("%s\n",__func__);

        Quattro_vendor1_id->off_charging_off = 0x22; // For power off charging
        

         //msm_proc_comm(PCOM_RESET_CHIP, 0, 0);
        msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
}

void power_down_registertimer(struct timer_list* ptimer, unsigned long timeover )
{
        printk("%s\n",__func__);
        init_timer(ptimer);
        ptimer->expires = get_jiffies_64() + timeover;
        ptimer->data = (long) NULL;
        ptimer->function = power_down_timeout;
        add_timer(ptimer);
}

static ssize_t store_power_down(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int i;
	char *after;
        unsigned long value = simple_strtoul(buf, &after, 10);

	if (value == 1)
	{
		printk("[HSIL] %s(%d)\n", __func__, __LINE__);
        	power_down_registertimer(&power_down_timer, POWER_DOWN_TIME);
	}

	return count;
}

static DEVICE_ATTR(power_down, S_IRUGO|S_IWUSR, NULL, store_power_down);

void command_handler(u16 cmd);

static void dpram_irq_work_queue(struct work_struct *work)
{
      wake_lock_timeout(&dpram_wake_lock, HZ/2);
      command_handler(request_command);

//      printk(KERN_INFO "%s  %d\n", __func__,request_command);
      enable_irq(INT_A9_M2A_3);
}

#ifdef CONFIG_EVENT_LOGGING
static inline EVENT_HEADER *getPayloadHeader(int flag, int size)
{
	EVENT_HEADER *header;
	struct timeval time_val;

	header = (EVENT_HEADER *)kmalloc(sizeof (EVENT_HEADER), GFP_ATOMIC);
	do_gettimeofday(&time_val);

	header->timeVal = time_val;
	header->class = (flag == DPRAM_READ ? DPRAM_READ : DPRAM_WRITE);
	header->repeat_count = 0;
	header->payload_length = size;

	return header;
}

static inline void dpram_event_logging(int direction, void *src, int size)
{
	EVENT_HEADER *header;
	unsigned long flags;

	header = getPayloadHeader(direction, size);

	local_irq_save(flags);
	klog(header, sizeof (EVENT_HEADER), DPRAM_ID);

	if (direction == DPRAM_WRITE) {
		klog(src, size, DPRAM_ID);
	}

	else if (direction == DPRAM_READ) {
		klog((void *)(SmemBase + src), size, DPRAM_ID);
	}

	local_irq_restore(flags);
	kfree(header);
}
#endif	/* CONFIG_EVENT_LOGGING */

/* tty related functions. */
static inline void byte_align(unsigned long dest, unsigned long src)
{
	u16 *p_src;
	volatile u16 *p_dest;

	if (!(dest % 2) && !(src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0xFF00) | (*p_src & 0x00FF);
	}

	else if ((dest % 2) && (src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0x00FF) | (*p_src & 0xFF00);
	}

	else if (!(dest % 2) && (src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0xFF00) | ((*p_src >> 8) & 0x00FF);
	}

	else if ((dest % 2) && !(src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0x00FF) | ((*p_src << 8) & 0xFF00);
	}

	else {
		dprintk("oops.~\n");
	}
}

static inline void _memcpy(void *p_dest, const void *p_src, int size)
{
	unsigned long dest = (unsigned long)p_dest;
	unsigned long src = (unsigned long)p_src;

	if (size <= 0) {
		return;
	}

	if (dest & 1) {
		byte_align(dest, src);
		dest++, src++;
		size--;
	}

	if (size & 1) {
		byte_align(dest + size - 1, src + size - 1);
		size--;
	}

	if (src & 1) {
		unsigned char *s = (unsigned char *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) {
			*d++ = s[0] | (s[1] << 8);
			s += 2;
		}
	}

	else {
		u16 *s = (u16 *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) { *d++ = *s++; }
	}
}

static inline int _memcmp(u8 *dest, u8 *src, int size)
{
	int i = 0;

	while (i < size) {
		if (*(dest + i) != *(src + i)) {
			return 1;
		}
		i++;
	}

	return 0;
}

static inline int WRITE_TO_DPRAM_VERIFY(u32 dest, void *src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)(SmemBase + dest), (void *)src, size);

		if (!_memcmp((u8 *)(SmemBase + dest), (u8 *)src, size))
			return 0;
	}

	return -1;
}

static inline int READ_FROM_DPRAM_VERIFY(void *dest, u32 src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)dest, (void *)(SmemBase + src), size);

		if (!_memcmp((u8 *)dest, (u8 *)(SmemBase + src), size))
			return 0;
	}

	return -1;
}

static void send_interrupt_to_phone(u16 irq_mask)
{
#ifdef PRINT_SEND_IRQ
			printk(KERN_ERR "=====> send IRQ: %x \n", irq_mask);
#endif

	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, &irq_mask,
			DPRAM_INTERRUPT_PORT_SIZE);
	// hsil
	writel(1, MSM_A2M_INT(3));
//	dprintk("PDA -> Phone interrupt!\n");
}

static int dpram_write(dpram_device_t *device,
		const unsigned char *buf, int len)
{
	int retval = 0;
	int size = 0;

	u16 head, tail;
	u16 irq_mask = 0;
	

#ifdef PRINT_WRITE
	int i;
	printk(KERN_ERR "\nWRITE ");
	for (i = 0; i < len; i++)	
		printk(KERN_ERR "%02x ", *((unsigned char *)buf + i));
	printk(KERN_ERR "\n");
#endif
    
	
	down(&write_mutex);

	READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

#ifdef _HSDPA_DPRAM

	// +++++++++ head ---------- tail ++++++++++ //
	if (head < tail) {
		size = tail - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// tail +++++++++++++++ head --------------- //
	else if (tail == 0) {
		size = device->out_buff_size - head - 1;
		size = (len > size) ? size : len;

		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;
	}

	// ------ tail +++++++++++ head ------------ //
	else {
		size = device->out_buff_size - head;
		size = (len > size) ? size : len;
		
		WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
		retval = size;

		if (len > retval) {
			size = (len - retval > tail - 1) ? tail - 1 : len - retval;
			
			WRITE_TO_DPRAM(device->out_buff_addr, buf + retval, size);
			retval += size;
		}
	}

#endif	/* _HSDPA_DPRAM */

	/* @LDK@ calculate new head */
	head = (u16)((head + retval) % device->out_buff_size);
	WRITE_TO_DPRAM_VERIFY(device->out_head_addr, &head, sizeof(head));

#ifdef CONFIG_EVENT_LOGGING
	dpram_event_logging(DPRAM_WRITE, (void *)&head, size);
#endif	/* CONFIG_EVENT_LOGGING */

	/* @LDK@ send interrupt to the phone, if.. */
	irq_mask = INT_MASK_VALID;

	if (retval > 0)
		irq_mask |= device->mask_send;

	if (len > retval)
		irq_mask |= device->mask_req_ack;

//	send_interrupt_to_phone(irq_mask);
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, &irq_mask,
			DPRAM_INTERRUPT_PORT_SIZE);

	up(&write_mutex);

	writel(1, MSM_A2M_INT(3));
	
	return retval;
}
static inline int dpram_tty_insert_data(dpram_device_t *device, const u8 *psrc, u16 size)
{
#define CLUSTER_SEGMENT	1500

	u16 copied_size = 0;
	int retval = 0;
	
#ifdef PRINT_READ
	int i;
	printk(KERN_ERR "READ: %d\n", size);
	for(i=0; i<size; i++)	
		printk(KERN_ERR "%02x ", *(psrc+ + i));
	printk(KERN_ERR "\n");
#endif

	if (size > CLUSTER_SEGMENT && (device->serial.tty->index == 1)) {
		while (size) {
			copied_size = (size > CLUSTER_SEGMENT) ? CLUSTER_SEGMENT : size;
			tty_insert_flip_string(device->serial.tty, psrc + retval, copied_size);

			size -= copied_size;
			retval += copied_size;
		}

		return retval;
	}

	return tty_insert_flip_string(device->serial.tty, psrc, size);
}
static int dpram_read_fmt(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int retval_add = 0;
	int size = 0;
	u16 head, tail;


	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

//	printk(KERN_ERR "=====> %s,  head: %d, tail: %d\n", __func__, head, tail);

	if (head != tail) {
		u16 up_tail = 0;

		// ------- tail ++++++++++++ head -------- //
		if (head > tail) {
			size = head - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(SmemBase + (device->in_buff_addr + tail)), size);
			if(size!= retval)
				printk(KERN_ERR "[OneDRAM: size: %d, retval: %d\n", size, retval);
#ifdef PRINT_READ_SHORT
			else
				printk(KERN_ERR "READ -return: %d\n", retval);
#endif
		}

		// +++++++ head ------------ tail ++++++++ //
		else {
			int tmp_size = 0;

			// Total Size.
			size = device->in_buff_size - tail + head;

			// 1. tail -> buffer end.
			tmp_size = device->in_buff_size - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(SmemBase + (device->in_buff_addr + tail)), tmp_size);
			if(tmp_size!= retval)
				printk(KERN_ERR "[OneDRAM: size: %d, retval: %d\n", size, retval);
#ifdef PRINT_READ_SHORT
			else
				printk(KERN_ERR "READ -return: %d\n", retval);
#endif

			// 2. buffer start -> head.
			if (size > tmp_size) {
				retval_add = dpram_tty_insert_data(device, (u8 *)(SmemBase + device->in_buff_addr), size - tmp_size);
				retval += retval_add;
		
				if((size - tmp_size)!= retval_add)
					printk(KERN_ERR "[OneDRAM: size - tmp_size: %d, retval_add: %d\n", size - tmp_size, retval_add);
#ifdef PRINT_READ_SHORT
				else
					printk(KERN_ERR "READ -return_add: %d\n", retval_add);
#endif


			}
		}

		/* new tail */
		up_tail = (u16)((tail + retval) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}
		

	if (non_cmd & device->mask_req_ack) 	
		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));

	return retval;
	
}
static int dpram_read_raw(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int size = 0;
	u16 head, tail;
	u16 up_tail = 0;
	
	int ret;
	u32 len;
	struct pdp_info *dev = NULL;
	struct pdp_hdr hdr;
	u16 read_offset;
	u8 len_high, len_low, id, control;
	u16  pre_data_size;
	u8 ch;

#ifdef PRINT_READ
       printk(KERN_ERR "enter :dpram_read_raw\n");
#endif
	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

//	printk(KERN_ERR "=====> %s,  head: %d, tail: %d\n", __func__, head, tail);

	if(head != tail) {
	
		up_tail = 0;

		if (head > tail) {
			size = head - tail;									/* ----- (tail) 7f 00 00 7e (head) ----- */ 
#if 0		
		printk(KERN_ERR "READ\n");
		for(i=0; i<size; i++)	
			printk(KERN_ERR "%02x ", *((unsigned char *)(DPRAM_VBASE + (device->in_buff_addr + tail)) + i));
		printk(KERN_ERR "\n");
#endif
		}
		else
			size = device->in_buff_size - tail + head;			/* 00 7e (head) ----------- (tail) 7f 00 */ 

		read_offset = 0;
//		printk(KERN_ERR "=====> %s,  head: %d, tail: %d, size: %d\n", __func__, head, tail, size);

		while(size){			
			READ_FROM_DPRAM(&ch, device->in_buff_addr +((u16)(tail + read_offset) % device->in_buff_size), sizeof(ch));
#ifdef PRINT_READ
                    printk(KERN_ERR "%02x ",ch);
#endif

			if(ch == 0x7f) {
				read_offset ++;
			}
			else {
				printk(KERN_ERR "[OneDram] %s failed.. First byte: %d, drop byte: %d\n", __func__, ch, size);
				printk(KERN_ERR "buff addr: %lx\n", (device->in_buff_addr));
				printk(KERN_ERR "read addr: %lx\n", (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));

				dpram_drop_data(device);
				return -1;
			}

			len_high = len_low = id = control = 0;
			READ_FROM_DPRAM(&len_low, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(len_high));
			read_offset ++;
			READ_FROM_DPRAM(&len_high, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(len_low));
			read_offset ++;
			READ_FROM_DPRAM(&id, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(id));
			read_offset ++;
			READ_FROM_DPRAM(&control, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size) ,sizeof(control));
			read_offset ++;

			hdr.len = len_high <<8 | len_low;
			hdr.id = id;
			hdr.control = control;
	
			len = hdr.len - sizeof(struct pdp_hdr);	
			if(len <= 0) {
				printk(KERN_ERR "[OneDram] %s uups..\n", __func__);
				printk(KERN_ERR "%s, %d read_offset: %d, len: %d hdr.id: %d\n", __func__, __LINE__, read_offset, len, hdr.id);

				dpram_drop_data(device);
				return -1;


			}
			dev = pdp_get_dev(hdr.id);
//			printk(KERN_ERR "%s, %d read_offset: %d, len: %d hdr.id: %d\n", __func__, __LINE__, read_offset, len, hdr.id);

			if(!dev) {
				printk(KERN_ERR "[OneDram] %s failed.. NULL dev detected \n", __func__);
				dpram_drop_data(device);
				return -1;
			}

				
			if (dev->vs_dev.tty != NULL && dev->vs_dev.refcount) {
#ifdef DPRAM_HIGH_LATENCY_PATH_FOR_MUTEX_FAIL
				unsigned char prev_latency = dev->vs_dev.tty->low_latency;
				if(atomic_read(&dev->vs_dev.tty->termios_mutex.count) != 1) {
					/* other task has the mutex so let's take the high latency path */
					dev->vs_dev.tty->low_latency = 0;
					pr_emerg("[dpram] taking high latency path\n");
				}
#endif
				if((u16)(tail + read_offset) % device->in_buff_size + len < device->in_buff_size) {
					ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(SmemBase + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
					tty_flip_buffer_push(dev->vs_dev.tty);
				}else {
					pre_data_size = device->in_buff_size - (tail + read_offset); 
					ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(SmemBase + (device->in_buff_addr + tail + read_offset)), pre_data_size);
					ret += tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(SmemBase + (device->in_buff_addr)),len - pre_data_size);
					tty_flip_buffer_push(dev->vs_dev.tty);
//					printk(KERN_ERR "=====> pre_data_size: %d, len-pre_data_size: %d, ret: %d\n", pre_data_size, len- pre_data_size, ret);
				}
#ifdef DPRAM_HIGH_LATENCY_PATH_FOR_MUTEX_FAIL
				if(dev->vs_dev.tty->low_latency != prev_latency) {
					dev->vs_dev.tty->low_latency = prev_latency;
					pr_info("[dpram] latency restored (low_latency=%d)", prev_latency);
				}
#endif
			}
			else {
				printk(KERN_ERR "[%s]failed.. tty channel(id:%d) is not opened.\n", __func__, dev->id);
				ret = len;
			}
			
			if(!ret) {
				printk(KERN_ERR "[OneDram] %s failed.. (tty_insert_flip_string) drop byte: %d\n", __func__, size);
				printk(KERN_ERR "buff addr: %lx\n", (device->in_buff_addr));
				printk(KERN_ERR "read addr: %lx\n", (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));
				dpram_drop_data(device);
				return -1;
			}
			
			read_offset += ret;
//			printk(KERN_ERR "%s,%d read_offset: %d ret= %d\n", __func__, __LINE__, read_offset, ret);

			READ_FROM_DPRAM(&ch, (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)), sizeof(ch));
			if(ch == 0x7e) 				
				read_offset ++;
			else {
				printk(KERN_ERR "[OneDram] %s failed.. Last byte: %d, drop byte: %d\n", __func__, ch, size);
				printk(KERN_ERR "buff addr: %lx\n", (device->in_buff_addr));
				printk(KERN_ERR "read addr: %lx\n", (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));
				dpram_drop_data(device);
				return -1;
			}

			size -= (ret + sizeof(struct pdp_hdr) + 2);
			retval += (ret + sizeof(struct pdp_hdr) + 2);
//			printk(KERN_ERR "%s, %d retval= %d, read_offset: %d, size: %d\n", __func__, __LINE__, retval, read_offset, size);

			if(size < 0) {
				printk(KERN_ERR "something wrong....\n");
				break;
			}

		}
		up_tail = (u16)((tail + read_offset) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}
#if 0
	/* new tail */
	up_tail = (u16)((tail + retval) % device->in_buff_size);
	WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
#endif	
 #ifdef PRINT_READ
       printk(KERN_ERR "exit :dpram_read_raw\n");
#endif


	if (non_cmd & device->mask_req_ack)
		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));


	return retval;

}

static void dpram_clear(void)
{
	long i = 0;
	unsigned long flags;
	
	u16 value = 0;

	dprintk("SmemBase = %x\n", (unsigned int)SmemBase);
	/* @LDK@ clear DPRAM except interrupt area */
	local_irq_save(flags);

	for (i=DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS; i<DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2); i+=2)
	{
		*((u16 *)(SmemBase + i)) = 0;
	}

	// hsil 0125 for LPM mode booting
	*((u16 *)(SmemBase + DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS)) = 0x01;

	local_irq_restore(flags);

	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
}

static void dpram_init_and_report(void)
{
	const u16 magic_code = 0x00aa;
//	const u16 init_start = INT_COMMAND(INT_MASK_CMD_INIT_START);
	const u16 init_end = INT_COMMAND(INT_MASK_CMD_INIT_END);

	u16 ac_code = 0;

	dprintk("start\n");

// hsil
#if 0
	/* @LDK@ send init start code to phone */
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&init_start, DPRAM_INTERRUPT_PORT_SIZE);
	//hsil
	writel(1, MSM_A2M_INT(3));

	/* @LDK@ write DPRAM disable code */
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));
#endif

	/* @LDK@ dpram clear */
	dpram_clear();


	/* @LDK@ write magic code */
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));

	/* @LDK@ write DPRAM enable code */
	ac_code = 0x0001;
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));

	// hsil for temporary
//	READ_FROM_DPRAM(&value, DPRAM_MAGIC_CODE_ADDRESS, sizeof(value));
//	dprintk("DPRAM_MAiGIC_CODE_ADDRESS value = 0x%x\n", value);
//	READ_FROM_DPRAM(&value, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(value));
//	dprintk("DPRAM_ACCESS_ENABLE_ADDRESS value = %d\n", value);
	
	/* @LDK@ send init end code to phone */
	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&init_end, DPRAM_INTERRUPT_PORT_SIZE);
	//hsil
	writel(1, MSM_A2M_INT(3));
	dprintk("finish\n");
}

static inline int dpram_get_read_available(dpram_device_t *device)
{
	u16 head, tail;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	dprintk("%s : head = 0x%x\n", __func__, head);
	dprintk("%s : tail = 0x%x\n", __func__, tail);
	return head - tail;
}

static void dpram_drop_data(dpram_device_t *device)
{
	u16 head;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &head, sizeof(head));
}

static void dpram_phone_on(void)
{
// hsil
#ifdef MSM7201A
	dprintk("\n");
	dpram_init_and_report();
//	print_smem();
#else
	int pin_active = omap_get_gpio_datain(OMAP3430_GPIO_PHONE_ACTIVE18);

	switch (pin_active) {
		case GPIO_LEVEL_LOW:
			omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_HIGH);
	
			/* phone power */
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
			break;

		case GPIO_LEVEL_HIGH:
			omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_LOW);
	
			/* phone power */
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);

			msleep(1000);

			omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_HIGH);
	
			/* phone power */
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);
			msleep(500);
			omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
			break;
	}
#endif
}

static void dpram_phone_off(void)
{
#ifndef MSM7201A
	const u16 ac_code = 0;

	/* @LDK@ write dpram disable code. */
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS,
			&ac_code, sizeof(ac_code));

	omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_LOW);
	
	/* phone power */
	omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);
	msleep(500);
	omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_LOW);
	msleep(500);
	omap_set_gpio_dataout(OMAP3430_GPIO_FONE_ON, GPIO_LEVEL_HIGH);
#endif
}

static int dpram_phone_getstatus(void)
{
#ifndef MSM7201A 
	return omap_get_gpio_datain(OMAP3430_GPIO_PHONE_ACTIVE18);
#endif	
	return 0;
}

static void dpram_phone_reset(void)
{
#ifdef MSM7201A
// hsii 0211
//	const u16 phone_reset = INT_COMMAND(INT_MASK_CMD_PHONE_RESET);
// hsil 0514
//	const u16 reboot_magic_code = 0x00ab;
	const u16 reboot_magic_code = 0x3569;
	u16 magic_read;

	dprintk("[RAM DUMP] REBOOT_MAGIC_CODE\n");
	
	READ_FROM_DPRAM((void *)&magic_read, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic_read));
	dprintk("[RAM DUMP] Prev Magic Code : 0x%x\n", magic_read); 

	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &reboot_magic_code, sizeof(reboot_magic_code));
	dprintk("[RAM DUMP] SMSM WRITE\n");	
	
	READ_FROM_DPRAM((void *)&magic_read, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic_read));
	dprintk("[RAM DUMP] Cur Magic Code : 0x%x\n", magic_read); 
	
	msleep(100);		// hsil 0602
	smsm_reset_modem(SMSM_SYSTEM_DOWNLOAD);

#else
	omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_LOW);
	msleep(100);
	omap_set_gpio_dataout(OMAP3430_GPIO_nMSM_RST, GPIO_LEVEL_HIGH);
#endif	
}

static void dpram_mem_rw(struct _mem_param *param)
{
	/* @LDK@ write */
	if (param->dir) {
		WRITE_TO_DPRAM(param->addr, (void *)&param->data, sizeof(param->data));
	}

	/* @LDK@ read */
	else {
		READ_FROM_DPRAM((void *)&param->data, param->addr, sizeof(param->data));
	}
}

static int dpram_get_dgs(void)
{
	int                   nRet;		
#if DGS_TEST
	typedef struct{
		UINT8 header_code[4];  // magic 넘버   // {0x7F,0xAA,0xAF,0x7E}
		UINT8 model[20];       // 모델명
		UINT8 nature[40];      // 출항지
		UINT8 custt_code[8];   // 거래선
		UINT8 date[14];        // 버전 발행일
		UINT8 charger[24];     // 담당자
		UINT8 version[20];     // SW 버전명
		UINT8 checksum[10];    // binary ckecksum
		UINT8 crcsum[10];      // binary CRC
		UINT8 Unique_Number[20];  // UN number
		UINT8 mem_name[20];    // 메모리 명
		UINT8 sec_code[20];    // 
		UINT8 etc[40];         // 기타
    }NAND_HEAD_INFO;
	NAND_HEAD_INFO header_info = {{0x7F,0xAA,0xAF,0x7E}, {"GT-I5500"}, {"EUR-Open"}, {"TIM"},
								{"2010-03-25"}, {"LKH"}, {"I5500AIJC3"}, {"C721"}, {"41D7"}, 
								{"7011000003"}, {"KAC00200JM-D4YY000"}, {"1108-000061"}, {""}};
#endif

	printk("[DPRAM] Start getting DGS info.\n");

	if(bml_open == NULL || bml_acquireSM == NULL || ond_4k_read == NULL || bml_release == NULL)
	{
		printk(KERN_ERR "[DPRAM] bml functions are not initilized yet!!\n");
		return -1;
	}


	if (bml_open(0, FSR_BML_FLAG_NONE) != FSR_BML_SUCCESS)
	{
		printk("[DPRAM] BML_Open Error !!\n");
		return -1;
	}

	printk("[DPRAM] Try to read DGS Info.\n");

	/* Acquire SM */
	bml_acquireSM(0);

	/* 
	 * Read DGS Page 
	 * Read 5th Page of 2047 Block
	 */
	nRet = ond_4k_read(0, 2047, 5, aDGSBuf, NULL, FSR_LLD_FLAG_ECC_OFF);	

	/* Release SM*/
	bml_release(0);

#if DGS_TEST
	memcpy(aDGSBuf, &header_info,sizeof(NAND_HEAD_INFO));
#endif

	if(nRet != FSR_LLD_SUCCESS)
	{
		printk("[DPRAM] Reading DGS information failed !!\n");
		return -1;
	}

	printk("[DPRAM] DSG buffer =  %s \n", aDGSBuf);
	printk("[OneDRAM] %s complete\n", __func__);

	return 0;
}
static void dpram_ramdump(void)
{
	printk("[DPRAM] RAMDUMP MODE START!\n");
//	printk("[DPRAM] write 0xCCCC flag at MSM_SHARED_RAM_BASE + 0x30 address\n");
//	printk("[DPRAM] MSM_SHARED_RAM_BASE: 0x%08x\n", (unsigned int) MSM_SHARED_RAM_BASE);
	writel(0xCCCC, MSM_SHARED_RAM_BASE + 0x30); 
	printk("[DPRAM] call msm_proc_comm_reset_modem_now func\n");
	msm_proc_comm_reset_modem_now();
}
// hsil
static void print_smem(void)
{
	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;
	u8 raw_out_buf;
//	u16 dump_data = 0;
#if 1
	struct file *filp;
//  int i;
	int writelen;
	mm_segment_t old_fs;
	static char buf[1024*32];
//	static int buf[1024];
	int count, chr_count;
//	char *src;
//	fl_owner_t id;
#endif

	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));
	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fmt_out_tail));
	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(raw_out_tail));
	READ_FROM_DPRAM((void *)&raw_out_buf, DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS, sizeof(raw_out_buf));
	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, DPRAM_INTERRUPT_PORT_SIZE);
	READ_FROM_DPRAM((void *)&out_interrupt, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, DPRAM_INTERRUPT_PORT_SIZE);
	dprintk("\n");
	printk("#####################################\n");
	printk("#########  DPRAM DUMP DATA (0604)  #########\n");
	printk("#####################################\n");
	printk("-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"| MAGIC CODE\t\t| 0x%04x\n"
			"| ENABLE CODE\t\t| 0x%04x\n"
			"| PHONE->PDA FMT HEAD\t| %u\n"
			"| PHONE->PDA FMT TAIL\t| %u\n"
			"| PDA->PHONE FMT HEAD\t| %u\n"
			"| PDA->PHONE FMT TAIL\t| %u\n"
			"| PHONE->PDA RAW HEAD\t| %u\n"
			"| PHONE->PDA RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW HEAD\t| %u\n"
			"| PDA->PHONE RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW BUFF\t| %u\n"
			"| PHONE->PDA INT.\t| 0x%04x\n"
			"| PDA->PHONE INT.\t| 0x%04x\n"
			"-------------------------------------\n",
			magic, enable,
			fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
			raw_in_head, raw_in_tail, raw_out_head, raw_out_tail, raw_out_buf,
			in_interrupt, out_interrupt
		);
#if 1
//		id = current->files;

		count = 1024 * 8;
		chr_count = 0;
		old_fs = get_fs();
        set_fs(KERNEL_DS);

		filp = filp_open("/sdcard/dpram_dump",O_CREAT|O_WRONLY,0666);
		if(!filp)
    		printk("Can't creat /sdcard/dpram_dump file\n");
		else 
		{
			memcpy((void *)buf, (void *)SmemBase, 1024*32);
			writelen = filp->f_op->write(filp,(void *)buf,1024*32,&filp->f_pos);
		}
		set_fs(old_fs);
#if 0
		printk("\n");	
		printk("#####################################\n");
		printk("# PDA2PHONE FORMATTED BUFFER DUMP   #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
		
		for (i=DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS; i<DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("|\t0x%04x\t|\t0x%04x\t\t\n", i, dump_data);
		}    
																															   
		printk("\n");	
		printk("#####################################\n");
		printk("# PDA2PHONE RAW BUFFER DUMP         #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS; i<DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}    
																																											  
		printk("\n");	
		printk("#####################################\n");
		printk("# PHONE2PDA FORMATTED BUFFER DUMP   #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS; i<DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}    
		
		printk("\n");	
		printk("#####################################\n");
		printk("# PHONE2PDA RAW BUFFER DUMP         #\n");
		printk("#####################################\n");
		printk("-------------------------------------\n");
		printk("|\tADDRESS\t|\tVALUE\t|\n");
		printk("-------------------------------------\n");
			
		for (i=DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS; i<DPRAM_PDA2PHONE_INTERRUPT_ADDRESS; i=i+0x0002)
		{    
			READ_FROM_DPRAM((void *)&dump_data, i, sizeof(dump_data));
			printk("| 0x%04x\t\t| 0x%04x\t\n", i, dump_data);
		}
#endif		
#endif
}


#ifdef CONFIG_PROC_FS
static int dpram_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	char *p = page;
	int len;

	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;

#ifdef _ENABLE_ERROR_DEVICE
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;
#endif	/* _ENABLE_ERROR_DEVICE */
	
	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));

	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, 
			sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, 
		    sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, 
		    sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, 
		    sizeof(fmt_out_tail));

	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, 
		    sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, 
		    sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, 
		    sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, 
		    sizeof(raw_out_tail));

	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, 
		    DPRAM_INTERRUPT_PORT_SIZE);
	READ_FROM_DPRAM((void *)&out_interrupt, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, 
		    DPRAM_INTERRUPT_PORT_SIZE);


#ifdef _ENABLE_ERROR_DEVICE
	memset((void *)buf, '\0', DPRAM_ERR_MSG_LEN);
	local_irq_save(flags);
	memcpy(buf, dpram_err_buf, DPRAM_ERR_MSG_LEN - 1);
	local_irq_restore(flags);
#endif	/* _ENABLE_ERROR_DEVICE */

	p += sprintf(p,
			"-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"| MAGIC CODE\t\t| 0x%04x\n"
			"| ENABLE CODE\t\t| 0x%04x\n"
			"| PHONE->PDA FMT HEAD\t| %u\n"
			"| PHONE->PDA FMT TAIL\t| %u\n"
			"| PDA->PHONE FMT HEAD\t| %u\n"
			"| PDA->PHONE FMT TAIL\t| %u\n"
			"| PHONE->PDA RAW HEAD\t| %u\n"
			"| PHONE->PDA RAW TAIL\t| %u\n"
			"| PDA->PHONE RAW HEAD\t| %u\n"
			"| PDA->PHONE RAW TAIL\t| %u\n"
			"| PHONE->PDA INT.\t| 0x%04x\n"
			"| PDA->PHONE INT.\t| 0x%04x\n"
#ifdef _ENABLE_ERROR_DEVICE
			"| LAST PHONE ERR MSG\t| %s\n"
#endif	/* _ENABLE_ERROR_DEVICE */
// hsil
//			"| PHONE ACTIVE\t\t| %s\n"
//			"| DPRAM INT Level\t| %d\n"
			"-------------------------------------\n",
			magic, enable,
			fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
			raw_in_head, raw_in_tail, raw_out_head, raw_out_tail,
			in_interrupt, out_interrupt,

#ifdef _ENABLE_ERROR_DEVICE
			(buf[0] != '\0' ? buf : "NONE")
#endif	/* _ENABLE_ERROR_DEVICE */
// hsil
//			(dpram_phone_getstatus() ? "ACTIVE" : "INACTIVE"),
//			omap_get_gpio_datain(OMAP3430_GPIO_nDPRAM_INT)
		);

	len = (p - page) - off;
	if (len < 0) {
		len = 0;
	}

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif /* CONFIG_PROC_FS */

/* dpram tty file operations. */
static int dpram_tty_open(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = &dpram_table[tty->index];

	dprintk("tty->index = %d\n", tty->index);
	device->serial.tty = tty;
	device->serial.open_count++;

	if (device->serial.open_count > 1) {
		device->serial.open_count--;
		return -EBUSY;
	}

	#if 1	// hobac.

	if (tty->index == 1)	// dpram1
	{
		struct termios termios;
		mm_segment_t oldfs;

		oldfs = get_fs(); set_fs(get_ds());

		if (file->f_op->ioctl)
		{
			file->f_op->ioctl(file->f_dentry->d_inode, file,
					TCGETA, (unsigned long)&termios);
		}

		else if (file->f_op->unlocked_ioctl)
		{
			file->f_op->unlocked_ioctl(file, TCGETA, (unsigned long)&termios);
		}

		set_fs(oldfs);

		termios.c_cflag = CS8 | CREAD | HUPCL | CLOCAL | B115200;
		termios.c_iflag = IGNBRK | IGNPAR;
		termios.c_lflag = 0;
		termios.c_oflag = 0;
		termios.c_cc[VMIN] = 1;
		termios.c_cc[VTIME] = 1;

		oldfs = get_fs(); set_fs(get_ds());

		if (file->f_op->ioctl)
		{
			file->f_op->ioctl(file->f_dentry->d_inode, file,
					TCSETA, (unsigned long)&termios);
		}

		else if (file->f_op->unlocked_ioctl)
		{
			file->f_op->unlocked_ioctl(file, TCSETA, (unsigned long)&termios);
		}

		set_fs(oldfs);
	}

	#endif
	tty->driver_data = (void *)device;
	tty->low_latency = 1;

	return 0;
}

static void dpram_tty_close(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device && (device == &dpram_table[tty->index])) {
		down(&device->serial.sem);
		device->serial.open_count--;
		device->serial.tty = NULL;
		up(&device->serial.sem);
	}
}

static int dpram_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (!device) {
		return 0;
	}

	return dpram_write(device, buffer, count);
}

static int dpram_tty_write_room(struct tty_struct *tty)
{
	int avail;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
		READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

		avail = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;

		return avail;
	}

	return 0;
}

static int dpram_tty_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned int val;

	switch (cmd) {
		case HN_DPRAM_PHONE_ON:
			if (DpramInited) 
			{
				dprintk("Doubled Phone On Cmd : do nothing\n");
				return 0;
			}
			dprintk("[Version 3] HN_DPRAM_PHONE_ON\n");
			dpram_phone_on();
			DpramInited = 1;
			return 0;

		case HN_DPRAM_PHONE_OFF:
			dprintk("HN_DPRAM_PHONE_OFF\n");
			dpram_phone_off();
			return 0;

		case HN_DPRAM_PHONE_GETSTATUS:
			dprintk("HN_DPRAM_PHONE_GETSTATUS\n");
			val = dpram_phone_getstatus();
			return copy_to_user((unsigned int *)arg, &val, sizeof(val));

		case HN_DPRAM_PHONE_RESET:
			dprintk("[RAM DUMP]HN_DPRAM_PHONE_RESET\n");
			dpram_phone_reset();
			return 0;

		case HN_DPRAM_MEM_RW:
		{
			struct _mem_param param;

			dprintk("HN_DPRAM_MEM_RW\n");
			val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
			dpram_mem_rw(&param);

			if (!param.dir) {
				return copy_to_user((unsigned long *)arg, &param, sizeof(param));
			}

			return 0;
		}

		case HN_DPRAM_DUMP:
			print_smem();
			return 0;

		case HN_DPRAM_WAKELOCK:
			wake_lock(&imei_wake_lock);
			return 0;
/*//open
		case HN_DPRAM_WAKEUNLOCK:
			wake_unlock(&imei_wake_lock);
			return 0;
*/
		case DPRAM_GET_DGS_INFO:
			printk("[%s] DGS ioctl called\n", __func__);
			if( !dpram_get_dgs() )
				return copy_to_user((unsigned long *)arg, aDGSBuf, 256);
			else
				printk(KERN_ERR "[%s] Get DGS info failed\n", __func__);
			return 0;

		case HN_DPRAM_RAMDUMP:
			dpram_ramdump();
			return 0;
/*//open
		case HN_DPRAM_EXTRA_MEM_RW:  // for screen capture
			memcpy(smem_vendor1_lcd_buf, ((_param_em *)arg)->addr, ((_param_em *)arg)->size);
			return 0;
*/
		default:
			dprintk("default\n");
			break;
	}

	return -ENOIOCTLCMD;
}

static int dpram_tty_chars_in_buffer(struct tty_struct *tty)
{
	int data;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
		READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

		data = (head > tail) ? head - tail - 1 :
			device->out_buff_size - tail + head;

		return data;
	}

	return 0;
}

#ifdef _ENABLE_ERROR_DEVICE
static int dpram_err_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);

	unsigned long flags;
	ssize_t ret;
	size_t ncopy;

	add_wait_queue(&dpram_err_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	while (1) {
		local_irq_save(flags);

		if (dpram_err_len) {
			ncopy = min(count, dpram_err_len);

			if (copy_to_user(buf, dpram_err_buf, ncopy)) {
				ret = -EFAULT;
			}

			else {
				ret = ncopy;
			}

			dpram_err_len = 0;
			
			local_irq_restore(flags);
			break;
		}

		local_irq_restore(flags);

		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		schedule();
	}

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&dpram_err_wait_q, &wait);

	return ret;
}

static int dpram_err_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &dpram_err_async_q);
}

static unsigned int dpram_err_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	poll_wait(filp, &dpram_err_wait_q, wait);
	return ((dpram_err_len) ? (POLLIN | POLLRDNORM) : 0);
}
#endif	/* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
static int onedram_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{

	return 0;
}

static int onedram_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;

	u32 *buffer, *src;
	u32 __iomem *dst;
	int c, i, cnt = 0, err = 0;


	buffer = kmalloc((count > PAGE_SIZE) ? PAGE_SIZE : count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	dst = (u32 __iomem *) (ONEDRAM_SHARED_AREA_VIRT+ p);

	while (count) {
		c = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		src = buffer;

		if (copy_from_user(src, buf, c)) {
			err = -EFAULT;
			break;
		}

		for (i = c >> 2; i--; )
			od_writel(*src++, dst++);

		if (c & 3) {
			u8 *src8 = (u8 *) src;
			u8 __iomem *dst8 = (u8 __iomem *) dst;

			for (i = c & 3; i--; )
				od_writeb(*src8++, dst8++);

			dst = (u32 __iomem *) dst8;
		}

		*ppos += c;
		buf += c;
		cnt += c;
		count -= c;
	}

	kfree(buffer);

	return (cnt) ? cnt : err;
}

static int onedram_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case HN_DPRAM_PHONE_ON:
			dpram_phone_on();
			return 0;

		case HN_DPRAM_PHONE_OFF:
			dpram_phone_off();
			return 0;

		default:
			break;
	}

	return -ENOIOCTLCMD;
}

static int onedram_mmap(struct file *file, struct vm_area_struct * vma)
{
	unsigned long off;
	unsigned long start;
	u32 len;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	off = vma->vm_pgoff << PAGE_SHIFT;

	lock_kernel();

	start = ONEDRAM_SHARED_AREA_PHYS;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + ONEDRAM_SHARED_AREA_SIZE);
	if (off >= len) {
		/* memory mapped io */
		off -= len;
		start = ONEDRAM_SHARED_AREA_PHYS;
		len = PAGE_ALIGN((start & ~PAGE_MASK) + ONEDRAM_SHARED_AREA_SIZE);
	}
	
	unlock_kernel();
	
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			     vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}
#endif

/* handlers. */
static void res_ack_tasklet_handler(unsigned long data)
{
	dpram_device_t *device = (dpram_device_t *)data;

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
				tty->ldisc->ops->write_wakeup) {
			(tty->ldisc->ops->write_wakeup)(tty);
		}

		wake_up_interruptible(&tty->write_wait);
	}
}

static void fmt_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;

	dprintk("%s\n", __func__);
	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

#ifdef _HSDPA_DPRAM

		while (dpram_get_read_available(device)) {
			ret = dpram_read_fmt(device, non_cmd);

			if (ret < 0) {
				dprintk("%s : dpram_read fail\n", __func__);
				/* TODO: ... wrong.. */
			}
			tty_flip_buffer_push(tty);
		}

#endif	/* _HSDPA_DPRAM */
	}

	else {
		dprintk("%s : will drop data\n", __func__);
		dpram_drop_data(device);
	}

	/* @LDK@ modified by hobac on 2008.09.04. */
	/* @LDK@ dpram_read()에서 인터럽트 처리 */
//	if (non_cmd & device->mask_req_ack) {
//		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));
//	}
}
static void raw_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;
	
	while (dpram_get_read_available(device)) {
		ret = dpram_read_raw(device, non_cmd);
		if (ret < 0) {
			printk(KERN_ERR "%s, dpram_read failed\n", __func__);
			/* TODO: ... wrong.. */
		}
	}
}

static void cmd_req_active_handler(void)
{
	send_interrupt_to_phone(INT_COMMAND(INT_MASK_CMD_RES_ACTIVE));
}

static void cmd_error_display_handler(void)
{
	char buf[DPRAM_ERR_MSG_LEN];

#ifdef _ENABLE_ERROR_DEVICE
	unsigned long flags;
#endif	/* _ENABLE_ERROR_DEVICE */

	// hsil for silent reset
	printk("[DPRAM] %s : silent reset,\n", __func__);
	wake_lock(&silent_wake_lock);
	memset((void *)buf, 0, sizeof (buf));
	buf[0] = '1';
	buf[1] = ' ';
	READ_FROM_DPRAM((buf + 2), DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
			sizeof (buf) - 3);

	dprintk("[PHONE ERROR] ->> %s\n", buf);

#ifdef _ENABLE_ERROR_DEVICE
	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	dpram_err_len = 64;
	local_irq_restore(flags);

	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);
#endif	/* _ENABLE_ERROR_DEVICE */
}

static void cmd_phone_start_handler(void)
{
	dprintk("\n");
	dpram_init_and_report();
}

static void cmd_req_time_sync_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_phone_deep_sleep_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_nv_rebuilding_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_emer_down_handler(void)
{
	/* TODO: add your codes here.. */
}

static void cmd_chg_detect_noti(void)
{
	u16 value;
	u16 irq_clear = 0x0000;

	get_charger_type();
	get_charging_status();
//	msm_batt_check_event();

	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x40C0) {
		WRITE_TO_DPRAM(DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, &irq_clear, DPRAM_INTERRUPT_PORT_SIZE);
		printk("[DPRAM:%s] chg_detect irq: 0x%x cleared.\n", __func__, value);
	}
	else {
		READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
		printk("[DPRAM:%s] changed irq: 0x%x detected.\n", __func__, value);
	}
	msm_batt_check_event();

}

static void cmd_chg_state_changed(void)
{
	u16 value;
	u16 irq_clear = 0x0000;

	get_charging_status();
	msm_batt_check_event();
	
	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x50C0) {
		WRITE_TO_DPRAM(DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, &irq_clear, DPRAM_INTERRUPT_PORT_SIZE);
		printk("[DPRAM:%s] chg_state irq: 0x%x cleared.\n", __func__, value);
	}
	else {
		READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
		printk("[DPRAM:%s] changed irq: 0x%x detected.\n", __func__, value);
	}

}
void command_handler(u16 cmd)
{
	switch (cmd) {
		case INT_MASK_CMD_REQ_ACTIVE:
			cmd_req_active_handler();
			break;

		case INT_MASK_CMD_ERR_DISPLAY:
			cmd_error_display_handler();
			break;

		case INT_MASK_CMD_PHONE_START:
			cmd_phone_start_handler();
			break;

		case INT_MASK_CMD_REQ_TIME_SYNC:
			cmd_req_time_sync_handler();
			break;

		case INT_MASK_CMD_PHONE_DEEP_SLEEP:
			cmd_phone_deep_sleep_handler();
			break;

		case INT_MASK_CMD_NV_REBUILDING:
			cmd_nv_rebuilding_handler();
			break;

		case INT_MASK_CMD_EMER_DOWN:
			cmd_emer_down_handler();
			break;
		case INT_MASK_CMD_CHG_DETECT_NOTI:
			cmd_chg_detect_noti();
			break;

		case INT_MASK_CMD_CHG_STATE_CHANGED:
			cmd_chg_state_changed();
			break;

		default:
			dprintk("Unknown command..\n");
	}
}

static void non_command_handler(u16 non_cmd)
{
	u16 head, tail;

	/* @LDK@ formatted check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	if (head != tail)
		non_cmd |= INT_MASK_SEND_F;

	/* @LDK@ raw check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(tail));

	if (head != tail)
		non_cmd |= INT_MASK_SEND_R;

	/* @LDK@ +++ scheduling.. +++ */
	if (non_cmd & INT_MASK_SEND_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		dpram_tasklet_data[FORMATTED_INDEX].device = &dpram_table[FORMATTED_INDEX];
		dpram_tasklet_data[FORMATTED_INDEX].non_cmd = non_cmd;
		
		fmt_send_tasklet.data = (unsigned long)&dpram_tasklet_data[FORMATTED_INDEX];
		tasklet_schedule(&fmt_send_tasklet);
	}

	if (non_cmd & INT_MASK_SEND_R) {
		wake_lock_timeout(&dpram_wake_lock, 4*HZ);
		dpram_tasklet_data[RAW_INDEX].device = &dpram_table[RAW_INDEX];
		dpram_tasklet_data[RAW_INDEX].non_cmd = non_cmd;

		raw_send_tasklet.data = (unsigned long)&dpram_tasklet_data[RAW_INDEX];
		/* @LDK@ raw buffer op. -> soft irq level. */
		tasklet_hi_schedule(&raw_send_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		tasklet_schedule(&fmt_res_ack_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_R) {
		wake_lock_timeout(&dpram_wake_lock, 4*HZ);
		tasklet_hi_schedule(&raw_res_ack_tasklet);
	}
}

static inline
void check_int_pin_level(void)
{
#if 0
	u16 mask = 0, cnt = 0;

	while (cnt++ < 3) {
		READ_FROM_DPRAM(&mask, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(mask));

		if (!omap_get_gpio_datain(OMAP3430_GPIO_nDPRAM_INT))
			break;
	}
#endif	
}

/* @LDK@ interrupt handlers. */
static irqreturn_t dpram_interrupt(int irq, void *dev_id)
{
	u16 irq_mask = 0;
//	u16 reset_int = 0;

	dprintk("%s : interrupt handler\n", __func__);	

#ifdef PRINT_RECV_IRQ	
	      printk(KERN_ERR "<===== received dpram interrupt\n");
#endif

	
//	wake_lock_timeout(&dpram_wake_lock, HZ/2);

	READ_FROM_DPRAM(&irq_mask, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(irq_mask));
#if 0
	printk("=====>[%s,%d] irq_mask: %x\n", __func__, __LINE__, irq_mask);	
	READ_FROM_DPRAM_VERIFY(&fih, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fih));
	READ_FROM_DPRAM_VERIFY(&fit, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fit));
	READ_FROM_DPRAM_VERIFY(&foh, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(foh));
	READ_FROM_DPRAM_VERIFY(&fot, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fot));
	READ_FROM_DPRAM_VERIFY(&rih, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(rih));
	READ_FROM_DPRAM_VERIFY(&rit, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(rit));
	READ_FROM_DPRAM_VERIFY(&roh, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(roh));
	READ_FROM_DPRAM_VERIFY(&rot, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(rot));
	printk("\n fmt_in  H:%4d, T:%4d\n fmt_out H:%4d, T:%4d\n raw_in  H:%4d, T:%4d\n raw out H:%4d, T:%4d\n", fih, fit, foh, fot, rih, rit, roh, rot);
#endif

	/* 2008.09.18 geunyoung, streaming bug working-around.. @LDK@ */
//	check_int_pin_level();	// hsil

	/* valid bit verification. @LDK@ */
	if (!(irq_mask & INT_MASK_VALID)) {
		dprintk("Invalid interrupt mask: 0x%04x\n", irq_mask);
		return IRQ_NONE;
	}

	/* command or non-command? @LDK@ */
	if (irq_mask & INT_MASK_COMMAND) {
		irq_mask &= ~(INT_MASK_VALID | INT_MASK_COMMAND);
#ifdef PRINT_RECV_IRQ	
	      printk(KERN_ERR "<===== received command IRQ: %x\n", irq_mask);
#endif

              #if 0
                wake_lock_timeout(&dpram_wake_lock, HZ/2);
                command_handler(irq_mask);
              #else
                request_command = irq_mask;
                queue_work(dpram_work_queue, &dpram_irq_work);
                disable_irq_nosync(INT_A9_M2A_3);
              #endif
	}

	else {
		irq_mask &= ~INT_MASK_VALID;
#ifdef PRINT_RECV_IRQ	
	       printk(KERN_ERR "<===== received NON command IRQ: %x\n", irq_mask);
#endif
		non_command_handler(irq_mask);
		//wake_lock_timeout(&dpram_wake_lock, 6*HZ);
	}

// hsil 0609
//	WRITE_TO_DPRAM(DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, &reset_int, sizeof(reset_int));

	return IRQ_HANDLED;
}
#if 0
static irqreturn_t phone_active_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
#endif
/* basic functions. */
#ifdef _ENABLE_ERROR_DEVICE
static struct file_operations dpram_err_ops = {
	.owner = THIS_MODULE,
	.read = dpram_err_read,
	.fasync = dpram_err_fasync,
	.poll = dpram_err_poll,
	.llseek = no_llseek,

	/* TODO: add more operations */
};
#endif	/* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
static struct file_operations onedram_ops = {
	.owner = THIS_MODULE,
	.read = onedram_read,
	.write = onedram_write,
	.ioctl = onedram_ioctl,
	.mmap = onedram_mmap,
};
#endif	/* _ENABLE_ONEDRAM_DEVICE */

static struct tty_operations dpram_tty_ops = {
	.open = dpram_tty_open,
	.close = dpram_tty_close,
	.write = dpram_tty_write,
	.write_room = dpram_tty_write_room,
	.ioctl = dpram_tty_ioctl,
	.chars_in_buffer = dpram_tty_chars_in_buffer,

	/* TODO: add more operations */
};

#ifdef _ENABLE_ERROR_DEVICE

static void unregister_dpram_err_device(void)
{
	unregister_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE);
	class_destroy(dpram_class);
}

static int register_dpram_err_device(void)
{
	/* @LDK@ 1 = formatted, 2 = raw, so error device is '0' */
	struct device *dpram_err_dev_t;
	int ret = register_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE, &dpram_err_ops);

	if ( ret < 0 )
	{
		return ret;
	}

	dpram_class = class_create(THIS_MODULE, "err");

	if (IS_ERR(dpram_class))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	dpram_err_dev_t = device_create(dpram_class, NULL,
			MKDEV(DRIVER_MAJOR_NUM, 0), NULL, DPRAM_ERR_DEVICE);

	if (IS_ERR(dpram_err_dev_t))
	{
		unregister_dpram_err_device();
		return -EFAULT;
	}

	return 0;
}
#endif	/* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
static void unregister_onedram_device(void)
{
	unregister_chrdev(ONEDRAM_MAJOR_NUM, ONEDRAM_DEVICE);
	class_destroy(onedram_class);
}

static int register_onedram_device(void)
{
	struct device *onedram_dev_t;
	int ret = register_chrdev(ONEDRAM_MAJOR_NUM, ONEDRAM_DEVICE, &onedram_ops);

	if ( ret < 0 )
	{
		return ret;
	}

	onedram_class = class_create(THIS_MODULE, "one");

	if (IS_ERR(dpram_class))
	{
		unregister_onedram_device();
		return -EFAULT;
	}

	onedram_dev_t = device_create(onedram_class, NULL,
			MKDEV(ONEDRAM_MAJOR_NUM, 0), ONEDRAM_DEVICE);

	if (IS_ERR(onedram_dev_t))
	{
		unregister_onedram_device();
		return -EFAULT;
	}

	return 0;
}
#endif

static int register_dpram_driver(void)
{
	int retval = 0;

	/* @LDK@ allocate tty driver */
	dpram_tty_driver = alloc_tty_driver(MAX_INDEX);

	if (!dpram_tty_driver) {
		return -ENOMEM;
	}

	/* @LDK@ initialize tty driver */
	dpram_tty_driver->owner = THIS_MODULE;
	dpram_tty_driver->magic = TTY_DRIVER_MAGIC;
	dpram_tty_driver->driver_name = DRIVER_NAME;
	dpram_tty_driver->name = "dpram";
	dpram_tty_driver->major = DRIVER_MAJOR_NUM;
	dpram_tty_driver->minor_start = 1;
// hsil
	dpram_tty_driver->num = 2;	// original
//	dpram_tty_driver->num = 1;
	dpram_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	dpram_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	dpram_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	dpram_tty_driver->init_termios = tty_std_termios;
	dpram_tty_driver->init_termios.c_cflag =
		(B115200 | CS8 | CREAD | CLOCAL | HUPCL);

	tty_set_operations(dpram_tty_driver, &dpram_tty_ops);

	dpram_tty_driver->ttys = dpram_tty;
	dpram_tty_driver->termios = dpram_termios;
	dpram_tty_driver->termios_locked = dpram_termios_locked;

	/* @LDK@ register tty driver */
	retval = tty_register_driver(dpram_tty_driver);

	if (retval) {
		dprintk("tty_register_driver error\n");
		put_tty_driver(dpram_tty_driver);
		return retval;
	}

	return 0;
}

static void unregister_dpram_driver(void)
{
	tty_unregister_driver(dpram_tty_driver);
}

static void init_devices(void)
{
	int i;

	for (i = 0; i < MAX_INDEX; i++) {
		init_MUTEX(&dpram_table[i].serial.sem);

		dpram_table[i].serial.open_count = 0;
		dpram_table[i].serial.tty = NULL;
	}
}

static void kill_tasklets(void)
{
	tasklet_kill(&fmt_res_ack_tasklet);
	tasklet_kill(&raw_res_ack_tasklet);

	tasklet_kill(&fmt_send_tasklet);
	tasklet_kill(&raw_send_tasklet);
}

static int register_interrupt_handler(void)
{
//hsil
//	unsigned int dpram_irq, phone_active_irq;
	int retval = 0;

//	dpram_irq = OMAP_GPIO_IRQ(OMAP3430_GPIO_nDPRAM_INT);
//	phone_active_irq = OMAP_GPIO_IRQ(OMAP3430_GPIO_PHONE_ACTIVE18);

	/* @LDK@ interrupt area read - pin level will be driven high. */
	dprintk("Dpram clear start\n");
	dpram_clear();

	/* @LDK@ Phone active INT. */
//	set_irq_type(OMAP_GPIO_IRQ(OMAP3430_GPIO_PHONE_ACTIVE18), IRQT_BOTHEDGE);
	/* @LDK@ DPRAM INT. */
//	set_irq_type(OMAP_GPIO_IRQ(OMAP3430_GPIO_nDPRAM_INT), IRQT_FALLING);
	
	/* @LDK@ dpram interrupt */
//	retval = request_irq(dpram_irq, dpram_interrupt, IRQF_DISABLED, DRIVER_NAME, NULL);
	retval = request_irq(INT_A9_M2A_3, dpram_interrupt, IRQF_TRIGGER_RISING, DRIVER_NAME, NULL);

	if (retval) {
		dprintk("DPRAM interrupt handler failed.\n");
		unregister_dpram_driver();
		return -1;
	}
	dprintk("INT_A9_M2A_3 interrupt handler success\n");

	/* @LDK@ phone active interrupt */
//	retval = request_irq(phone_active_irq, phone_active_interrupt,
//			IRQF_DISABLED, "Phone Active", NULL);

//	if (retval) {
//		dprintk("Phone active interrupt handler failed.\n");
//		free_irq(dpram_irq, NULL);
//		unregister_dpram_driver();
//		return -1;
//	}

	return 0;
}

static void check_miss_interrupt(void)
{
	unsigned long flags;
	u16 head, tail;
	u16 value;

	printk("[%s]\n", __func__);
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	dprintk("%s : head = 0x%x\n", __func__, head);
	dprintk("%s : tail = 0x%x\n", __func__, tail);

	if (head != tail) 
	{
		dprintk("there is a missed interrupt. try to read it!\n");
		printk("[DPRAM:%s] there is a missed interrupt. try to read it!\n", __func__);

		local_irq_save(flags);
		dpram_interrupt(INT_A9_M2A_3, NULL);
		local_irq_restore(flags);
	}
	
	READ_FROM_DPRAM(&value, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(value));
	if(value == 0x40C0 || value == 0x50C0) {
		printk("[DPRAM:%s] there is a missed battery interrupt. try to read it!\n", __func__);

		local_irq_save(flags);
		dpram_interrupt(INT_A9_M2A_3, NULL);
		local_irq_restore(flags);
	}

#if 0 //hsil
#if 1
	unsigned long flags;

	if (omap_get_gpio_datain(OMAP3430_GPIO_PHONE_ACTIVE18) && !omap_get_gpio_datain(OMAP3430_GPIO_nDPRAM_INT)) {
		dprintk("there is a missed interrupt. try to read it!\n");

		local_irq_save(flags);
		dpram_interrupt(OMAP_GPIO_IRQ(OMAP3430_GPIO_nDPRAM_INT), NULL);
		local_irq_restore(flags);
	}
#else
	unsigned long flags;
	int irq, pin_level;

	pin_level = omap_get_gpio_datain(OMAP3430_GPIO_PHONE_ACTIVE18) && !omap_get_gpio_datain(OMAP3430_GPIO_nDPRAM_INT);

	printk("phone active:%d, dpram interrupt:%d, pin level:%d\n", omap_get_gpio_datain(OMAP3430_GPIO_PHONE_ACTIVE18), omap_get_gpio_datain(OMAP3430_GPIO_nDPRAM_INT), pin_level);
	irq = OMAP_GPIO_IRQ(OMAP3430_GPIO_nDPRAM_INT);

	if (pin_level) {
		DPRINTK("Missing interrupt!\n");
		local_irq_save(flags);
		idpram_interrupt(irq, NULL);
		local_irq_restore(flags);
	}
#endif
#endif
}

static int dpram_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int dpram_resume(struct platform_device *dev)
{
	check_miss_interrupt();
	return 0;
}

enum {
	RESET,
	POWEROFF,
};
#if 0
void request_phone_power_off_reset(int flag)
{
	unsigned char fmt_cmd_reset[12] = {0x7F, 0x0A, 0x00, 0x5C, 0x07, 0x00, 0x5C, 0x00, 0x01, 0x03, 0x05, 0x7E};
	unsigned char fmt_cmd_powoff[12] = {0x7F, 0x0A, 0x00, 0x5D, 0x07, 0x00, 0x5D, 0x00, 0x01, 0x02, 0x01, 0x7E};

	switch(flag) {
		case RESET :
			printk("Dpram Reset's called\n");
			dpram_write(&dpram_table[0], fmt_cmd_reset, sizeof(fmt_cmd_reset));
			break;
		case POWEROFF : 
			printk("Dpram Poweroff's called\n");
			dpram_write(&dpram_table[0], fmt_cmd_powoff, sizeof(fmt_cmd_powoff));
			break;
	}
}
#endif

static int __devinit dpram_probe(struct platform_device *dev)
{
	int retval;

	// hsil
	/* allocate smem dpram area */
	printk("[DPRAM] %s:%d - SMEM_DPRAM allocation\n", __func__, __LINE__);
//	SmemBase = (volatile unsigned char *)(smem_alloc(SMEM_DPRAM, 0x4000));
	SmemBase = (volatile unsigned char *)(smem_alloc(SMEM_ID_VENDOR0, 0x4000*2));
//	printk("=====>[%s,%d] SMEM vendor0 id: %d\n", __func__, __LINE__, SMEM_ID_VENDOR0);
	if (!SmemBase)
	{
		printk("[DPRAM] %s:%d - smem_alloc failed : SmemBase = 0x%x\n", __func__, __LINE__, (unsigned int)SmemBase);
		return -1;
	}
	dprintk("SmemBase = 0x%x\n", (unsigned int)SmemBase);

   smem_vendor1_lcd_buf = (void *)((samsung_vendor1_id *)smem_alloc(SMEM_ID_VENDOR1, sizeof(samsung_vendor1_id)))->capture_buffer;
   if (!smem_vendor1_lcd_buf)
   {
     printk("[DPRAM] %s:%d - smem_alloc failed : smem_vendor1_lcd_buf = 0x%x\n", __func__, __LINE__, (unsigned int)smem_vendor1_lcd_buf);
     return -1;
   }
 
   dpram_work_queue = create_singlethread_workqueue("dpram_work_queue");
   if (!dpram_work_queue)
    return -ENOMEM;

  //dpram_irq_work = kzalloc(sizeof(struct work_struct), GFP_KERNEL);
   INIT_WORK(&dpram_irq_work, dpram_irq_work_queue);
   
	/* @LDK@ register dpram (tty) driver */
	retval = register_dpram_driver();

	if (retval) {
		printk("[DPRAM] %s:%d - Failed to register dpram (tty) driver.\n", __func__, __LINE__);
		return -1;
	}

#ifdef _ENABLE_ERROR_DEVICE
	/* @LDK@ register dpram error device */
	retval = register_dpram_err_device();

	if (retval) {
		printk("[DPRAM] %s:%d - Failed to register dpram error device.\n", __func__, __LINE__);

		unregister_dpram_driver();
		return -1;
	}

	memset((void *)dpram_err_buf, '\0', sizeof dpram_err_buf);
#endif /* _ENABLE_ERROR_DEVICE */

#ifdef _ENABLE_ONEDRAM_DEVICE
	retval = register_onedram_device();

	if (retval) {
		printk("[DPRAM] %s:%d - Failed to register onedram device.\n", __func__, __LINE__);

		unregister_dpram_driver();
		return -1;
	}

#endif
       multipdp_init();
	/* @LDK@ H/W setting */
//	enable_dpram_pins();	// hsil

	/* @LDK@ register interrupt handler */
	printk("[DPRAM] %s:%d - Register interrupt handler\n", __func__, __LINE__);
	if ((retval = register_interrupt_handler()) < 0) {
		return -1;
	}

	/* @LDK@ initialize device table */
	init_devices();

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(DRIVER_PROC_ENTRY, 0, 0, dpram_read_proc, NULL);
#endif	/* CONFIG_PROC_FS */

	/* @LDK@ check out missing interrupt from the phone */
	//check_miss_interrupt();

	dprintk(DRIVER_ID "\n");

	// hsil for temporary
	// hsil 0211
//	dpram_init_and_report();
//	print_smem();
	return 0;
}

static int __devexit dpram_remove(struct platform_device *dev)
{
	/* @LDK@ unregister dpram (tty) driver */
	unregister_dpram_driver();

#ifdef _ENABLE_ERROR_DEVICE
	/* @LDK@ unregister dpram error device */
	unregister_dpram_err_device();
#endif

#ifdef _ENABLE_ONEDRAM_DEVICE
	unregister_onedram_device();
#endif

	/* @LDK@ unregister irq handler */
// hsil 	
//	free_irq(OMAP_GPIO_IRQ(OMAP3430_GPIO_nDPRAM_INT), NULL);
//	free_irq(OMAP_GPIO_IRQ(OMAP3430_GPIO_PHONE_ACTIVE18), NULL);

	kill_tasklets();

	return 0;
}

static struct platform_driver platform_dpram_driver = {
	.probe = dpram_probe,
	.remove = __devexit_p(dpram_remove),
	.suspend = dpram_suspend,
	.resume = dpram_resume,
	.driver = {
		.name = "dpram",
	},
};



int silent_value = 0;
static int silent_read_proc_debug(char *page, char **start, off_t offset,
		                    int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "%u\n", silent_value);
}

static int silent_write_proc_debug(struct file *file, const char *buffer,
		                    unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		silent_value = 0;
		printk("Set silent : %d\n", silent_value);
	} else if (buf[0] == '1') {
		silent_value = 1;
		printk("Set silent : %d\n", silent_value);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

#if !(defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS))
static int dump_enable_read_proc_debug(char *page, char **start, off_t offset,
							int count, int *eof, void *data)
{
	samsung_vendor1_id* smem_vendor1 = (samsung_vendor1_id *)smem_alloc(SMEM_ID_VENDOR1, sizeof(samsung_vendor1_id));

	*eof = 1;
	return sprintf(page, "%u\n", smem_vendor1->ram_dump_level);
}

static int dump_enable_write_proc_debug(struct file *file, const char *buffer,
							unsigned long count, void *data)
{
	char *buf;
	samsung_vendor1_id* smem_vendor1 = (samsung_vendor1_id *)smem_alloc(SMEM_ID_VENDOR1, sizeof(samsung_vendor1_id));
	int ram_dump_level;
	   
	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {			   //low
		dump_enable = 0;
		printk("Set silent : %d\n", dump_enable);
	} else if (buf[0] == '1') {   //medium
		dump_enable = 1;
		printk("Set silent : %d\n", dump_enable);
	}  else if (buf[0] == '2') {   //high
		dump_enable = 2;
		printk("Set silent : %d\n", dump_enable);
	}else {
		kfree(buf);
		return -EINVAL;
	}

#if defined(CONFIG_MACH_ROOKIE) || defined(CONFIG_MACH_ESCAPE) || defined(CONFIG_MACH_GIO)
	ram_dump_level = get_ram_dump_level();

	if(ram_dump_level !=  dump_enable){
		ram_dump_level = dump_enable;
		set_ram_dump_level(ram_dump_level);
	}
#endif

	smem_vendor1->ram_dump_level = dump_enable;

	kfree(buf);
	return count;
}
#endif

/* init & cleanup. */
static int __init dpram_init(void)
{
	int ret;
	struct proc_dir_entry *ent;
	struct silent_info {
		unsigned long int info;
	} *smem_silent_mode;

	samsung_vendor1_id* smem_vendor1 = (samsung_vendor1_id *)smem_alloc(SMEM_ID_VENDOR1, sizeof(samsung_vendor1_id));	

	ret = platform_driver_register(&platform_dpram_driver);
	if (ret) {
		goto error_return;
	}

	platform_device_register_simple("dpram", -1, NULL, 0);
	wake_lock_init(&imei_wake_lock, WAKE_LOCK_SUSPEND, "IEMI");
	wake_lock_init(&dpram_wake_lock, WAKE_LOCK_SUSPEND, "DPRAM");
	wake_lock_init(&silent_wake_lock, WAKE_LOCK_SUSPEND, "SILENT_RESET");

	// For silent booting

	ent = create_proc_entry("silent", S_IRWXUGO, NULL);

	ent->read_proc = silent_read_proc_debug;
	ent->write_proc = silent_write_proc_debug;

#if defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIO) || defined(CONFIG_MACH_GIOS)
	pm_dev = device_create(sec_class, NULL, 0, NULL, "pm");
#else
    sec_class_1 = class_create(THIS_MODULE, "sec");	  
	pm_dev = device_create(sec_class_1, NULL, 0, NULL, "pm");
#endif
	if(IS_ERR(pm_dev))
		pr_err("Failed to create device(pm)!\n");
	if(device_create_file(pm_dev, &dev_attr_power_down) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_power_down.attr.name);

	smem_silent_mode = (struct silent_info *) smem_alloc(SMEM_APPS_BOOT_MODE, sizeof(struct silent_info));
//	printk("smem_silent_mode : 0x%08x\n", smem_silent_mode->info);
	if(smem_silent_mode->info == 0x1FEDCBA9)
		silent_value = 1;

#if !(defined(CONFIG_MACH_RANT3) || defined(CONFIG_MACH_VINO) || defined(CONFIG_MACH_GIOS))
	ent = create_proc_entry("ram_dump_level", S_IRWXUGO, NULL);
	ent->read_proc = dump_enable_read_proc_debug;
	ent->write_proc = dump_enable_write_proc_debug;

	smem_vendor1->ram_dump_level = dump_enable;
#endif

	error_return:

	return ret;
}

static void __exit dpram_exit(void)
{
	wake_lock_destroy(&dpram_wake_lock);
	wake_lock_destroy(&imei_wake_lock);
	wake_lock_destroy(&silent_wake_lock);
	platform_driver_unregister(&platform_dpram_driver);
}

static inline struct pdp_info * pdp_get_serdev(const char *name)
{
	int slot;
	struct pdp_info *dev;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		dev = pdp_table[slot];
		if (dev && dev->type == DEV_TYPE_SERIAL &&
		    strcmp(name, dev->vs_dev.tty_name) == 0) {
			return dev;
		}
	}
	return NULL;
}


static inline struct pdp_info * pdp_remove_dev(u8 id)
{
	int slot;
	struct pdp_info *dev;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			dev = pdp_table[slot];
			pdp_table[slot] = NULL;
			return dev;
		}
	}
	return NULL;
}


static int vs_open(struct tty_struct *tty, struct file *filp)
{
	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name); // 2.6 kernel porting

	if (dev == NULL) {
		return -ENODEV;
	}

	tty->driver_data = (void *)dev;
	tty->low_latency = 1;
	dev->vs_dev.tty = tty;
	dev->vs_dev.refcount++;
	printk(KERN_ERR "[%s] %s, refcount: %d \n", __func__, tty->driver->name, dev->vs_dev.refcount);

	return 0;
}

static void vs_close(struct tty_struct *tty, struct file *filp)
{
	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name); 

	if (!dev )
		return;
	dev->vs_dev.refcount--;
	printk(KERN_ERR "[%s] %s, refcount: %d \n", __func__, tty->driver->name, dev->vs_dev.refcount);

	// TODO..

	return;
}

static int pdp_mux(struct pdp_info *dev, const void *data, size_t len   )
{
	int ret;
	size_t nbytes;
	u8 *tx_buf;
	struct pdp_hdr *hdr;
	const u8 *buf;

       
	tx_buf = dev->tx_buf;
	hdr = (struct pdp_hdr *)(tx_buf + 1);
	buf = data;

	hdr->id = dev->id;
	hdr->control = 0;

	while (len) {
		if (len > MAX_PDP_DATA_LEN) {
			nbytes = MAX_PDP_DATA_LEN;
		} else {
			nbytes = len;
		}
		hdr->len = nbytes + sizeof(struct pdp_hdr);

		tx_buf[0] = 0x7f;
		
		memcpy(tx_buf + 1 + sizeof(struct pdp_hdr), buf,  nbytes);
		
		tx_buf[1 + hdr->len] = 0x7e;

//		printk(KERN_ERR "hdr->id: %d, hdr->len: %d\n", hdr->id, hdr->len);
#ifdef PRINT_WRITE
              printk(KERN_ERR "pdp_mux :");
              for(int i=0;i<=tx_buf[1 + hdr->len];i++)
              {
                   printk(KERN_ERR "%02x  ",tx_buf[i]);
              }
              printk(KERN_ERR"\n");
#endif		
		ret = dpram_write(&dpram_table[RAW_INDEX], tx_buf, hdr->len + 2);

		if (ret < 0) {
			printk(KERN_ERR "write_to_dpram() failed: %d\n", ret);
			return ret;
		}
		buf += nbytes;
		len -= nbytes;
	}
	return 0;
}


static int vs_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	int ret;
	struct pdp_info *dev = (struct pdp_info *)tty->driver_data;

	ret = pdp_mux(dev, buf, count);

	if (ret == 0) {
		ret = count;
	}

	return ret;
}

static int vs_write_room(struct tty_struct *tty) 
{
//	return TTY_FLIPBUF_SIZE;
	return 8192*2;
}

static int vs_chars_in_buffer(struct tty_struct *tty) 
{
	return 0;
}

static int vs_ioctl(struct tty_struct *tty, struct file *file, 
		    unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}



static struct tty_operations multipdp_tty_ops = {
	.open 		= vs_open,
	.close 		= vs_close,
	.write 		= vs_write,
	.write_room = vs_write_room,
	.ioctl 		= vs_ioctl,
	.chars_in_buffer = vs_chars_in_buffer,

	/* TODO: add more operations */
};

static struct tty_driver* get_tty_driver_by_id(struct pdp_info *dev)
{
	int index = 0;

	switch (dev->id) {
		case 1:		index = 0;	break;
		case 7:		index = 1;	break;
		case 8:		index = 2;	break;
		case 5:	       index = 3;	break;
		case 6:	       index = 4;	break;
		case 9:	       index = 5;	break;
      		case 27:	       index = 6;	break;      		
      		case 29:	       index = 7;	break;      		      		
		default:	       index = 0;
	}

	return &dev->vs_dev.tty_driver[index];
}

static int get_minor_start_index(int id)
{
	int start = 0;

	switch (id) {
		case 1:		start = 0;	break;
		case 7:		start = 1;	break;
		case 8:		start = 2;	break;
		case 5:	       start = 3;	break;
      		case 6:	       start = 4;	break;
		case 9:	       start = 5;	break;
      		case 27:	       start = 6;	break;      		      		
      		case 29:	       start = 7;	break;      		      		      		
		default:	       start = 0;
	}

	return start;
}


static int vs_add_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver;

	tty_driver = get_tty_driver_by_id(dev);

	if (!tty_driver) {
		printk(KERN_ERR "tty driver is NULL!\n");
		return -1;
	}

	kref_init(&tty_driver->kref);

	tty_driver->magic	= TTY_DRIVER_MAGIC;
	tty_driver->driver_name	= "multipdp";
	tty_driver->name	= dev->vs_dev.tty_name;
	tty_driver->major	= CSD_MAJOR_NUM;
	tty_driver->minor_start = get_minor_start_index(dev->id);
	tty_driver->num		= 1;
	tty_driver->type	= TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype	= SERIAL_TYPE_NORMAL;
	tty_driver->flags	= TTY_DRIVER_REAL_RAW;
//	kref_set(&tty_driver->kref, dev->vs_dev.refcount);
	tty_driver->ttys	= dev->vs_dev.tty_table; // 2.6 kernel porting
	tty_driver->termios	= dev->vs_dev.termios;
	tty_driver->termios_locked	= dev->vs_dev.termios_locked;

	tty_set_operations(tty_driver, &multipdp_tty_ops);
	return tty_register_driver(tty_driver);
}

static void vs_del_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver = NULL;

	tty_driver = get_tty_driver_by_id(dev);
	tty_unregister_driver(tty_driver);
}

static inline struct pdp_info * pdp_get_dev(u8 id)
{
	int slot;


	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			return pdp_table[slot];
		}
	}
	return NULL;
}

static inline int pdp_add_dev(struct pdp_info *dev)
{
	int slot;

	if (pdp_get_dev(dev->id)) {
		return -EBUSY;
	}

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] == NULL) {
			pdp_table[slot] = dev;
			return slot;
		}
	}
	return -ENOSPC;
}



static int pdp_activate(pdp_arg_t *pdp_arg, unsigned type, unsigned flags)
{
	int ret;
	struct pdp_info *dev;

	printk(KERN_ERR "%s, id: %d\n", __func__, pdp_arg->id);

	dev = kmalloc(sizeof(struct pdp_info) + MAX_PDP_PACKET_LEN, GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "out of memory\n");
		return -ENOMEM;
	}
	memset(dev, 0, sizeof(struct pdp_info));

	dev->id = pdp_arg->id;

	dev->type = type;
	dev->flags = flags;
	dev->tx_buf = (u8 *)(dev + 1);

	if (type == DEV_TYPE_SERIAL) {
		init_MUTEX(&dev->vs_dev.write_lock);
		strcpy(dev->vs_dev.tty_name, pdp_arg->ifname);

		ret = vs_add_dev(dev);
		if (ret < 0) {
			kfree(dev);
			return ret;
		}

		mutex_lock(&pdp_lock);
		ret = pdp_add_dev(dev);
		if (ret < 0) {
			printk(KERN_ERR "pdp_add_dev() failed\n");
			mutex_unlock(&pdp_lock);
			vs_del_dev(dev);
			kfree(dev);
			return ret;
		}
		mutex_unlock(&pdp_lock);

		{
			struct tty_driver * tty_driver = get_tty_driver_by_id(dev);

			printk(KERN_ERR "%s(id: %u) serial device is created.\n",
					tty_driver->name, dev->id);
		}
	}
	return 0;
}
static int multipdp_init(void)
{
	int i;

	pdp_arg_t pdp_args[NUM_PDP_CONTEXT] = {
		{ .id = 1, .ifname = "ttyCSD" },
              { .id = 7, .ifname = "ttyCDMA" },
		{ .id = 8, .ifname = "ttyEFS" },
		{ .id = 5, .ifname = "ttyGPS" },
		{ .id = 6, .ifname = "ttyXTRA" },
		{ .id = 9, .ifname = "ttyCIQ1" },
		{ .id = 27, .ifname = "ttyCIQ0" },		
		{ .id = 29, .ifname = "ttyCPLOG" },				
	};

	/* create serial device for Circuit Switched Data */
	for (i = 0; i < NUM_PDP_CONTEXT; i++) {
		if (pdp_activate(&pdp_args[i], DEV_TYPE_SERIAL, DEV_FLAG_STICKY) < 0) {
			printk(KERN_ERR "failed to create a serial device for %s\n", pdp_args[i].ifname);
		}
	}

	return 0;
}
module_init(dpram_init);
module_exit(dpram_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");
MODULE_DESCRIPTION("DPRAM Device Driver for Linux MITs.");
MODULE_LICENSE("GPL");

