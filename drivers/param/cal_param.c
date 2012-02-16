#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <asm/unistd.h>
#include <asm/errno.h>
#include <asm/uaccess.h>
#include <linux/device.h>

#include "param.h"
#include <samsung_flash.h>

#define PARAM_nID				FSR_PARTID_BML9
#define NAND_PAGE_PER_UNIT		64
#define NAND_SECTOR_PER_PAGE	8

#define DATE_SIZE 13
#define ACCL_OFFSET_SIZE 25
//suhjwoo_param
#define SYSPROPS_INT32_SIZE 64
#define SYSPROPS_STR_SIZE 256

#if 1
struct class *param_class;
struct device *param_dev_t;

extern struct class *sec_class;
#else
#ifdef CONFIG_SENSORS_KR3DM_ACCEL
extern struct device *kr3dm_dev_t;
#elif CONFIG_SENSORS_BMA_ACCEL
extern struct device *bma_dev_t;
#endif
#endif


// must be same as bootable/bootloader/lk/app/aboot/common.h
typedef struct _cal_param {	
	char result;
	char date[DATE_SIZE];	
	char acc_offset[ACCL_OFFSET_SIZE];

    //suhjwoo_param
    char hfa_num_retries[SYSPROPS_INT32_SIZE];
    char hfa_retry_interval[SYSPROPS_INT32_SIZE];
    
    char browser[SYSPROPS_STR_SIZE];
    
    char mms[SYSPROPS_STR_SIZE];
    
    char mediastream_buffersize[SYSPROPS_INT32_SIZE];
    char mediastream_httppd_proxy_addr[SYSPROPS_STR_SIZE];
    char mediastream_httppd_proxy_port[SYSPROPS_INT32_SIZE];
    char mediastream_mcdurl[SYSPROPS_STR_SIZE];
    char mediastream_rtsp_proxy_addr[SYSPROPS_STR_SIZE];
    char mediastream_rtsp_proxy_port[SYSPROPS_INT32_SIZE];
    
    char spa_dom_data_guard[SYSPROPS_INT32_SIZE];
    char spa_dom_data_roam[SYSPROPS_INT32_SIZE];
    char spa_dom_voice_guard[SYSPROPS_INT32_SIZE];
    char spa_dom_voice_roam[SYSPROPS_INT32_SIZE];
    char spa_intl_data_guard[SYSPROPS_INT32_SIZE];
    char spa_intl_data_roam[SYSPROPS_INT32_SIZE];
    char spa_intl_voice_guard[SYSPROPS_INT32_SIZE];
    char spa_intl_voice_roam[SYSPROPS_INT32_SIZE];

    char dss_proxy_ip[SYSPROPS_STR_SIZE];
    char dss_proxy_port[SYSPROPS_STR_SIZE];
    char dss_server_url[SYSPROPS_STR_SIZE];

    char law_locked[SYSPROPS_INT32_SIZE];
    char law_code[SYSPROPS_INT32_SIZE];
} CAL_RESULT_PARAM;

FSRPartI pstPartI;
int cal_param_n1stVun;
char mBuf[NAND_PAGE_SIZE];
extern struct proc_dir_entry *fsr_proc_dir;

static int get_cal_param_start_unit(void)
{
	int cnt;

	if(cal_param_n1stVun == 0) {
		samsung_get_full_bmlparti(&pstPartI);

		for(cnt = 0; cnt < pstPartI.nNumOfPartEntry; cnt++) 
			if(pstPartI.stPEntry[cnt].nID == PARAM_nID) 
				break;

		cal_param_n1stVun = (pstPartI.stPEntry[cnt].n1stVun) + 2;	/* 3rd unit of param partition : NEED TO SYNC. init.c */
	}

	return cal_param_n1stVun;
}

static int cal_result_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;
	CAL_RESULT_PARAM cal_result;
	
	memset(mBuf, 0xff, NAND_PAGE_SIZE);

	// read first page from param block
	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		return err;
	}
	
	memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
	printk("ACC CAL PARAM result : %c\n", cal_result.result);
	printk("ACC CAL PARAM date : %s\n", cal_result.date);	

	return sprintf(buf, "%c%s\n", cal_result.result, cal_result.date);
}
 
static int cal_result_param_write(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size)
{	
	int err;	
	char *buf;
	unsigned int nByteRet = 0;
	CAL_RESULT_PARAM bak_cal_result;	
	FSRChangePA stChangePA;
	
	printk("[%s] size = %d\n", __func__, size);
	printk("[%s] buffer = %s\n", __func__, buffer);

	if (size < 1)
		return -EINVAL;

	if(size > (DATE_SIZE+1)){
		printk(KERN_ERR "[%s] size of written buffer is bigger than CAL_RESULT_PARAM structure\n", __func__);
		return -EFAULT;
	}
	
	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
	{
		printk(KERN_ERR "[%s] Memory Allocation Failed.\n", __func__);
		return -ENOMEM;
	}

#if 0
	if ((err = copy_from_user(buf, buffer, size))){
		printk(KERN_ERR "[%s] Failed from copy user data. err = %d\n", __func__, err);
		kfree(buf);
		return -EFAULT;
	}	
#endif
	memset(buf, 0xff, size);
	memcpy(buf, buffer, size);
	
	printk("[%s] new result.result = %c\n", __func__, ((CAL_RESULT_PARAM*)buf)->result);
	printk("[%s] new result.date = %s\n", __func__, ((CAL_RESULT_PARAM*)buf)->date);

	// initialize buffer
	memset(mBuf, 0xff, NAND_PAGE_SIZE);	

#if 0	// for format 2nd unit of param part	
	stChangePA.nPartID  = PARAM_nID;
	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}

	err = samsung_bml_erase(get_cal_param_start_unit(), 1);
	if(err) {
		printk("PARAMERTER BML ERASE FAIL!\n");		
		kfree(buf);
		return err;
	}
#endif
	
	// read first page of cal param block
	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		kfree(buf);
		return err;
	}

	memcpy(&bak_cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
	// copy user data to cal result		
	memcpy(&bak_cal_result.result, &((CAL_RESULT_PARAM*)buf)->result, sizeof(char));	
	memcpy(&bak_cal_result.date, ((CAL_RESULT_PARAM*)buf)->date, DATE_SIZE);
	memcpy(mBuf, &bak_cal_result, sizeof(CAL_RESULT_PARAM));

	stChangePA.nPartID  = PARAM_nID;
	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}

	err = samsung_bml_erase(get_cal_param_start_unit(), 1);
	if(err) {
		printk("PARAMERTER BML ERASE FAIL!\n");		
		kfree(buf);
		return err;
	}

	// write back to chagned cal result
	err = samsung_bml_write(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT, 1, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML WRITE FAIL!\n");		
		kfree(buf);
		return err;
	}

	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RO;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}
	
	kfree(buf);
	
	return size;
}

static int cal_offset_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;
	CAL_RESULT_PARAM cal_result;
	
	memset(mBuf, 0xff, NAND_PAGE_SIZE);

	// read first page from param block
	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		return err;
	}
	
	memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));	
	//printk("ACC CAL PARAM offset : %s\n", cal_result.acc_offset);	

	return sprintf(buf, "%s\n", cal_result.acc_offset);
}
 
static int cal_offset_param_write(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size)
{	
	int err;	
	char *buf;
	unsigned int nByteRet = 0;
	CAL_RESULT_PARAM bak_cal_result;	
	FSRChangePA stChangePA;
	
	printk("[%s] size = %d\n", __func__, size);
	printk("[%s] buffer = %s\n", __func__, buffer);

	if (size < 1)
		return -EINVAL;

	if(size > ACCL_OFFSET_SIZE){
		printk(KERN_ERR "[%s] size of written buffer is bigger than CAL_RESULT_PARAM structure\n", __func__);
		return -EFAULT;
	}
	
	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
	{
		printk(KERN_ERR "[%s] Memory Allocation Failed.\n", __func__);
		return -ENOMEM;
	}

#if 0
	if ((err = copy_from_user(buf, buffer, size))){
		printk(KERN_ERR "[%s] Failed from copy user data. err = %d\n", __func__, err);
		kfree(buf);
		return -EFAULT;
	}	
#endif
	memset(buf, 0xff, size);
	memcpy(buf, buffer, size);
		
	printk("[%s] new offset = %s\n", __func__, buf);

	// initialize buffer
	memset(mBuf, 0xff, NAND_PAGE_SIZE);	
	
	// read first page of cal param block
	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		kfree(buf);
		return err;
	}

	memcpy(&bak_cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
	// copy user data to cal result			
	memcpy(&bak_cal_result.acc_offset, buf, ACCL_OFFSET_SIZE);
	memcpy(mBuf, &bak_cal_result, sizeof(CAL_RESULT_PARAM));

	stChangePA.nPartID  = PARAM_nID;
	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}

	err = samsung_bml_erase(get_cal_param_start_unit(), 1);
	if(err) {
		printk("PARAMERTER BML ERASE FAIL!\n");		
		kfree(buf);
		return err;
	}

	// write back to chagned cal result
	err = samsung_bml_write(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT, 1, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML WRITE FAIL!\n");		
		kfree(buf);
		return err;
	}

	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RO;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}
	
	kfree(buf);
	
	return size;
}

//suhjwoo_param
static int hfa_num_retries_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.hfa_num_retries);
    return sprintf(buf, "%s", cal_result.hfa_num_retries);
}

static int hfa_retry_interval_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.hfa_retry_interval);
    return sprintf(buf, "%s", cal_result.hfa_retry_interval);
}

static int browser_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.browser);
    return sprintf(buf, "%s", cal_result.browser);
}

static int mms_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mms);
    return sprintf(buf, "%s", cal_result.mms);
}

static int mediastream_buffersize_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_buffersize);
    return sprintf(buf, "%s", cal_result.mediastream_buffersize);
}

static int mediastream_httppd_proxy_addr_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_httppd_proxy_addr);
    return sprintf(buf, "%s", cal_result.mediastream_httppd_proxy_addr);
}

static int mediastream_httppd_proxy_port_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_httppd_proxy_port);
    return sprintf(buf, "%s", cal_result.mediastream_httppd_proxy_port);
}

static int mediastream_mclurl_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_mcdurl);
    return sprintf(buf, "%s", cal_result.mediastream_mcdurl);
}

static int mediastream_rtsp_proxy_addr_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_rtsp_proxy_addr);
    return sprintf(buf, "%s", cal_result.mediastream_rtsp_proxy_addr);
}

static int mediastream_rtsp_proxy_port_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.mediastream_rtsp_proxy_port);
    return sprintf(buf, "%s", cal_result.mediastream_rtsp_proxy_port);
}

static int spa_dom_data_guard_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_dom_data_guard);
    return sprintf(buf, "%s", cal_result.spa_dom_data_guard);
}

static int spa_dom_data_roam_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_dom_data_roam);
    return sprintf(buf, "%s", cal_result.spa_dom_data_roam);
}

static int spa_dom_voice_guard_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_dom_voice_guard);
    return sprintf(buf, "%s", cal_result.spa_dom_voice_guard);
}

static int spa_dom_voice_roam_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_dom_voice_roam);
    return sprintf(buf, "%s", cal_result.spa_dom_voice_roam);
}

static int spa_intl_data_guard_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_intl_data_guard);
    return sprintf(buf, "%s", cal_result.spa_intl_data_guard);
}

static int spa_intl_data_roam_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_intl_data_roam);
    return sprintf(buf, "%s", cal_result.spa_intl_data_roam);
}

static int spa_intl_voice_guard_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_intl_voice_guard);
    return sprintf(buf, "%s", cal_result.spa_intl_voice_guard);
}

static int spa_intl_voice_roam_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.spa_intl_voice_roam);
    return sprintf(buf, "%s", cal_result.spa_intl_voice_roam);
}

static int dss_proxy_ip_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.dss_proxy_ip);
    return sprintf(buf, "%s", cal_result.dss_proxy_ip);
}

static int dss_proxy_port_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.dss_proxy_port);
    return sprintf(buf, "%s", cal_result.dss_proxy_port);
}

static int dss_server_url_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.dss_server_url);
    return sprintf(buf, "%s", cal_result.dss_server_url);
}

static int law_locked_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.law_locked);
    return sprintf(buf, "%s", cal_result.law_locked);
}

static int law_code_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int err;
    CAL_RESULT_PARAM cal_result;
    
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page from param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
        return err;
    }
    
    memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
    printk("sysprops param value : %s\n", cal_result.law_code);
    return sprintf(buf, "%s", cal_result.law_code);
}

static int sysprops_param_write(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size)
{
    int err;
    //char *buf;
    char buf[SYSPROPS_STR_SIZE];
    unsigned int nByteRet = 0;
    CAL_RESULT_PARAM bak_cal_result;
    FSRChangePA stChangePA;
    
    printk("[%s] size = %d\n", __func__, size);
    printk("[%s] buffer = %s\n", __func__, buffer);
    
    if (size < 1)
        return -EINVAL;
    
    if(size > SYSPROPS_STR_SIZE){
        printk(KERN_ERR "[%s] size of written buffer is bigger than CAL_RESULT_PARAM structure\n", __func__);
        return -EFAULT;
    }
/*
    buf = kmalloc(size, GFP_KERNEL);
    if (!buf)
    {
        printk(KERN_ERR "[%s] Memory Allocation Failed.\n", __func__);
        return -ENOMEM;
    }

    memset(buf, 0xff, size);
    memcpy(buf, buffer, size);
*/
    memset(buf, 0, SYSPROPS_STR_SIZE);
    strncpy(buf, buffer, size);
    
    // initialize buffer
    memset(mBuf, 0xff, NAND_PAGE_SIZE);
    
    // read first page of cal param block
    err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML READ FAIL!\n");
//        kfree(buf);
        return err;
    }

    memcpy(&bak_cal_result, mBuf, sizeof(CAL_RESULT_PARAM));

    printk("sysprops_param_write id=%s\n", buf);
    
    // copy user data to cal result
    if(strncmp(buf, "hfa_num_retries=", strlen("hfa_num_retries="))==0)
    {
        memset(bak_cal_result.hfa_num_retries, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.hfa_num_retries, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "hfa_retry_interval=", strlen("hfa_retry_interval="))==0)
    {
        memset(bak_cal_result.hfa_retry_interval, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.hfa_retry_interval, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "browser=", strlen("browser="))==0)
    {
        memset(bak_cal_result.browser, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.browser, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "mms=", strlen("mms="))==0)
    {
        memset(bak_cal_result.mms, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.mms, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_buffersize=", strlen("mediastream_buffersize="))==0)
    {
        memset(bak_cal_result.mediastream_buffersize, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.mediastream_buffersize, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_httppd_proxy_addr=", strlen("mediastream_httppd_proxy_addr="))==0)
    {
        memset(bak_cal_result.mediastream_httppd_proxy_addr, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.mediastream_httppd_proxy_addr, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_httppd_proxy_port=", strlen("mediastream_httppd_proxy_port="))==0)
    {
        memset(bak_cal_result.mediastream_httppd_proxy_port, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.mediastream_httppd_proxy_port, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_mcdurl=", strlen("mediastream_mcdurl="))==0)
    {
        memset(bak_cal_result.mediastream_mcdurl, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.mediastream_mcdurl, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_rtsp_proxy_addr=", strlen("mediastream_rtsp_proxy_addr="))==0)
    {
        memset(bak_cal_result.mediastream_rtsp_proxy_addr, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.mediastream_rtsp_proxy_addr, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "mediastream_rtsp_proxy_port=", strlen("mediastream_rtsp_proxy_port="))==0)
    {
        memset(bak_cal_result.mediastream_rtsp_proxy_port, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.mediastream_rtsp_proxy_port, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_dom_data_guard=", strlen("spa_dom_data_guard="))==0)
    {
        memset(bak_cal_result.spa_dom_data_guard, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_dom_data_guard, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_dom_data_roam=", strlen("spa_dom_data_roam="))==0)
    {
        memset(bak_cal_result.spa_dom_data_roam, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_dom_data_roam, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_dom_voice_guard=", strlen("spa_dom_voice_guard="))==0)
    {
        memset(bak_cal_result.spa_dom_voice_guard, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_dom_voice_guard, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_dom_voice_roam=", strlen("spa_dom_voice_roam="))==0)
    {
        memset(bak_cal_result.spa_dom_voice_roam, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_dom_voice_roam, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_intl_data_guard=", strlen("spa_intl_data_guard="))==0)
    {
        memset(bak_cal_result.spa_intl_data_guard, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_intl_data_guard, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_intl_data_roam=", strlen("spa_intl_data_roam="))==0)
    {
        memset(bak_cal_result.spa_intl_data_roam, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_intl_data_roam, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_intl_voice_guard=", strlen("spa_intl_voice_guard="))==0)
    {
        memset(bak_cal_result.spa_intl_voice_guard, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_intl_voice_guard, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "spa_intl_voice_roam=", strlen("spa_intl_voice_roam="))==0)
    {
        memset(bak_cal_result.spa_intl_voice_roam, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.spa_intl_voice_roam, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "dss_proxy_ip=", strlen("dss_proxy_ip="))==0)
    {
        memset(bak_cal_result.dss_proxy_ip, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.dss_proxy_ip, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "dss_proxy_port=", strlen("dss_proxy_port="))==0)
    {
        memset(bak_cal_result.dss_proxy_port, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.dss_proxy_port, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "dss_server_url=", strlen("dss_server_url="))==0)
    {
        memset(bak_cal_result.dss_server_url, 0, SYSPROPS_STR_SIZE);
        strncpy(bak_cal_result.dss_server_url, buf, SYSPROPS_STR_SIZE-1);
    }
    else if(strncmp(buf, "law_locked=", strlen("law_locked="))==0)
    {
        memset(bak_cal_result.law_locked, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.law_locked, buf, SYSPROPS_INT32_SIZE-1);
    }
    else if(strncmp(buf, "law_code=", strlen("law_code="))==0)
    {
        memset(bak_cal_result.law_code, 0, SYSPROPS_INT32_SIZE);
        strncpy(bak_cal_result.law_code, buf, SYSPROPS_INT32_SIZE-1);
    }
    else
    {
        printk("accessing invalid sysprops param value");
        return 0;
    }
    memcpy(mBuf, &bak_cal_result, sizeof(CAL_RESULT_PARAM));

    stChangePA.nPartID  = PARAM_nID;
    stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
    if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
//        kfree(buf);
        return FS_DEVICE_FAIL;
    }

    err = samsung_bml_erase(get_cal_param_start_unit(), 1);
    if(err) {
        printk("PARAMERTER BML ERASE FAIL!\n");		
//        kfree(buf);
        return err;
    }

    // write back to chagned cal result
    err = samsung_bml_write(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT, 1, mBuf, NULL);
    if(err) {
        printk("PARAMERTER BML WRITE FAIL!\n");		
//        kfree(buf);
        return err;
    }
    
    stChangePA.nNewAttr = FSR_BML_PI_ATTR_RO;
    if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
//        kfree(buf);
        return FS_DEVICE_FAIL;
    }
    
//    kfree(buf);
    
    return size;
}

static DEVICE_ATTR(cal_result, 0777, cal_result_param_read, cal_result_param_write);
static DEVICE_ATTR(cal_offset, 0777, cal_offset_param_read, cal_offset_param_write);

//suhjwoo_param
static DEVICE_ATTR(hfa_num_retries, 0777, hfa_num_retries_param_read, sysprops_param_write);
static DEVICE_ATTR(hfa_retry_interval, 0777, hfa_retry_interval_param_read, sysprops_param_write);

static DEVICE_ATTR(browser, 0777, browser_param_read, sysprops_param_write);
static DEVICE_ATTR(mms, 0777, mms_param_read, sysprops_param_write);

static DEVICE_ATTR(mediastream_buffersize, 0777, mediastream_buffersize_param_read, sysprops_param_write);
static DEVICE_ATTR(mediastream_httppd_proxy_addr, 0777, mediastream_httppd_proxy_addr_param_read, sysprops_param_write);
static DEVICE_ATTR(mediastream_httppd_proxy_port, 0777, mediastream_httppd_proxy_port_param_read, sysprops_param_write);
static DEVICE_ATTR(mediastream_mcdurl, 0777, mediastream_mclurl_param_read, sysprops_param_write);
static DEVICE_ATTR(mediastream_rtsp_proxy_addr, 0777, mediastream_rtsp_proxy_addr_param_read, sysprops_param_write);
static DEVICE_ATTR(mediastream_rtsp_proxy_port, 0777, mediastream_rtsp_proxy_port_param_read, sysprops_param_write);

static DEVICE_ATTR(spa_dom_data_guard, 0777, spa_dom_data_guard_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_dom_data_roam, 0777, spa_dom_data_roam_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_dom_voice_guard, 0777, spa_dom_voice_guard_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_dom_voice_roam, 0777, spa_dom_voice_roam_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_intl_data_guard, 0777, spa_intl_data_guard_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_intl_data_roam, 0777, spa_intl_data_roam_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_intl_voice_guard, 0777, spa_intl_voice_guard_param_read, sysprops_param_write);
static DEVICE_ATTR(spa_intl_voice_roam, 0777, spa_intl_voice_roam_param_read, sysprops_param_write);

static DEVICE_ATTR(dss_proxy_ip, 0777, dss_proxy_ip_param_read, sysprops_param_write);
static DEVICE_ATTR(dss_proxy_port, 0777, dss_proxy_port_param_read, sysprops_param_write);
static DEVICE_ATTR(dss_server_url, 0777, dss_server_url_param_read, sysprops_param_write);
static DEVICE_ATTR(law_locked, 0777, law_locked_param_read, sysprops_param_write);
static DEVICE_ATTR(law_code, 0777, law_code_param_read, sysprops_param_write);

static int __init cal_param_init(void)
{
#if 1
  printk("<0><<< cal_param_init >>>");

  param_class = class_create(THIS_MODULE, "param");

  param_dev_t = device_create(param_class, NULL, MKDEV(PARAM_MAJOR, 0), "%s", "param");  

  if (device_create_file(param_dev_t, &dev_attr_cal_result) < 0)
		printk("<0>Failed to create device file(%s)!\n", dev_attr_cal_result.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_cal_offset) < 0)
		printk("<0>Failed to create device file(%s)!\n", dev_attr_cal_offset.attr.name);

  //suhjwoo_param
  if (device_create_file(param_dev_t, &dev_attr_hfa_num_retries) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_hfa_num_retries.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_hfa_retry_interval) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_hfa_retry_interval.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_browser) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_browser.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_mms) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mms.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_mediastream_buffersize) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_buffersize.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_mediastream_httppd_proxy_addr) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_httppd_proxy_addr.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_mediastream_httppd_proxy_port) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_httppd_proxy_port.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_mediastream_mcdurl) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_mcdurl.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_mediastream_rtsp_proxy_addr) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_rtsp_proxy_addr.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_mediastream_rtsp_proxy_port) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_mediastream_rtsp_proxy_port.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_spa_dom_data_guard) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_dom_data_guard.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_dom_data_roam) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_dom_data_roam.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_dom_voice_guard) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_dom_voice_guard.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_dom_voice_roam) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_dom_voice_roam.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_intl_data_guard) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_intl_data_guard.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_intl_data_roam) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_intl_data_roam.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_intl_voice_guard) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_intl_voice_guard.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_spa_intl_voice_roam) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_spa_intl_voice_roam.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_dss_proxy_ip) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_dss_proxy_ip.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_dss_proxy_port) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_dss_proxy_port.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_dss_server_url) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_dss_server_url.attr.name);

  if (device_create_file(param_dev_t, &dev_attr_law_locked) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_law_locked.attr.name);
  if (device_create_file(param_dev_t, &dev_attr_law_code) < 0)
      printk("<0>Failed to create device file(%s)!\n", dev_attr_law_code.attr.name);


#else
#ifdef CONFIG_SENSORS_KR3DM_ACCEL
	if (device_create_file(kr3dm_dev_t, &dev_attr_cal_result) < 0)
		printk(KERN_INFO "Failed to create device file(%s)!\n", dev_attr_cal_result.attr.name);

  if (device_create_file(kr3dm_dev_t, &dev_attr_cal_offset) < 0)
		printk(KERN_INFO "Failed to create device file(%s)!\n", dev_attr_cal_offset.attr.name);	
#else //if CONFIG_SENSORS_BMA_ACCEL
	if (device_create_file(bma_dev_t, &dev_attr_cal_result) < 0)
		printk(KERN_INFO "Failed to create device file(%s)!\n", dev_attr_cal_result.attr.name);

	if (device_create_file(bma_dev_t, &dev_attr_cal_offset) < 0)
		printk(KERN_INFO "Failed to create device file(%s)!\n", dev_attr_cal_offset.attr.name);	
#endif
#endif
	return 0;
}

static void __exit cal_param_exit(void)
{
	remove_proc_entry("efs_info", fsr_proc_dir);
}

module_init(cal_param_init);
module_exit(cal_param_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Samsung Param Operation");
