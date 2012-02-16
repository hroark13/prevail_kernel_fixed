#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <asm/unistd.h>
#include <asm/errno.h>
#include <asm/uaccess.h>
#include <linux/device.h>

//#include "param.h"
//open #include <samsung_flash.h>

//open#define PARAM_nID				FSR_PARTID_BML9
#define NAND_PAGE_PER_UNIT		64
#define NAND_SECTOR_PER_PAGE	8

#define DATE_SIZE 13
#define ACCL_OFFSET_SIZE 25
#define NAND_PAGE_SIZE 4096

extern struct device *kr3dm_dev_t;

// must be same as bootable/bootloader/lk/app/aboot/common.h
typedef struct _cal_param {	
	char result;
	char date[DATE_SIZE];	
	char acc_offset[ACCL_OFFSET_SIZE];
} CAL_RESULT_PARAM;

//open FSRPartI pstPartI;
int cal_param_n1stVun;
char mBuf[NAND_PAGE_SIZE];
//open extern struct proc_dir_entry *fsr_proc_dir;

static int get_cal_param_start_unit(void)
{
	int cnt;

	if(cal_param_n1stVun == 0) {
/*open		samsung_get_full_bmlparti(&pstPartI);

		for(cnt = 0; cnt < pstPartI.nNumOfPartEntry; cnt++) 
			if(pstPartI.stPEntry[cnt].nID == PARAM_nID) 
				break;

		cal_param_n1stVun = (pstPartI.stPEntry[cnt].n1stVun) + 1;		// from second unit of param partition
*/		
	}

	return cal_param_n1stVun;
}

static int cal_result_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err;
	CAL_RESULT_PARAM cal_result;
	
	memset(mBuf, 0xff, NAND_PAGE_SIZE);

	// read first page from param block
//open	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
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
//open	FSRChangePA stChangePA;
	
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
//open	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
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

/*open	stChangePA.nPartID  = PARAM_nID;
	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}
*/
//open	err = samsung_bml_erase(get_cal_param_start_unit(), 1);
	if(err) {
		printk("PARAMERTER BML ERASE FAIL!\n");		
		kfree(buf);
		return err;
	}

	// write back to chagned cal result
//open	err = samsung_bml_write(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT, 1, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML WRITE FAIL!\n");		
		kfree(buf);
		return err;
	}

/*open	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RO;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}
*/	
	kfree(buf);
	
	return size;
}

static int cal_check(const char *buf, size_t size)
{
	int i=2;
	printk("[%s] buf = %s\n", __func__, buf);
	while(i < size)
	{
		//printk("%d line buf : %c, Zero : %c\n", i, buf[i], '0'); 
		if(buf[i] != '0')
		{
			return 1;
		}
		i++;
		if( (i == 6) || (i == 13) ) i=i+3;
	}
	return 0;
}

static int cal_offset_param_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err, result;
	CAL_RESULT_PARAM cal_result;
	
	memset(mBuf, 0xff, NAND_PAGE_SIZE);

	// read first page from param block
//open	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		return err;
	}
	
	memcpy(&cal_result, mBuf, sizeof(CAL_RESULT_PARAM));	
	//printk("ACC CAL PARAM offset : %s\n", cal_result.acc_offset);	
	result = cal_check(cal_result.acc_offset, 19);
	//printk("[%s] cal_result = %d\n", __func__, result);

	if(!result) return sprintf(buf, "%d\n", 1);
	return sprintf(buf, "%s\n", cal_result.acc_offset);
}
 
static int cal_offset_param_write(struct device *dev, struct device_attribute *attr, const char *buffer, size_t size)
{	
	int err, cal_result=0;	
	char *buf;
	unsigned int nByteRet = 0;
	CAL_RESULT_PARAM bak_cal_result;	
//open	FSRChangePA stChangePA;
	
	printk("[%s] size = %d\n", __func__, size);
	printk("[%s] buffer = %s\n", __func__, buffer);

	if (size < 1)
	{
		return -EINVAL;
	}
	else if(size > ACCL_OFFSET_SIZE)
	{
		printk(KERN_ERR "[%s] size of written buffer is bigger than CAL_RESULT_PARAM structure\n", __func__);
		return -EFAULT;
	}
	else
	{
		cal_result=cal_check(buffer, size);
		//printk("[%s] cal_result = %d\n", __func__, cal_result);
	}
	
	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
	{
		printk(KERN_ERR "[%s] Memory Allocation Failed.\n", __func__);
		if(cal_result)	sprintf(buffer,"%d\n", 1);
		return -ENOMEM;
	}

#if 0
	if ((err = copy_from_user(buf, buffer, size))){
		printk(KERN_ERR "[%s] Failed from copy user data. err = %d\n", __func__, err);
		kfree(buf);
		return -EFAULT;
	}	
#endif
	memcpy(buf, buffer, size);
		
	printk("[%s] new offset = %s\n", __func__, buf);

	// initialize buffer
	memset(mBuf, 0xff, NAND_PAGE_SIZE);	
	
	// read first page of cal param block
//open	err = samsung_bml_read(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT * NAND_SECTOR_PER_PAGE, NAND_SECTOR_PER_PAGE, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML READ FAIL!\n");
		kfree(buf);
		if(cal_result)	sprintf(buffer,"%d\n", 1);
		return err;
	}

	memcpy(&bak_cal_result, mBuf, sizeof(CAL_RESULT_PARAM));
	// copy user data to cal result			
	memcpy(&bak_cal_result.acc_offset, buf, ACCL_OFFSET_SIZE);
	memcpy(mBuf, &bak_cal_result, sizeof(CAL_RESULT_PARAM));

/*open	stChangePA.nPartID  = PARAM_nID;
	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RW;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		return FS_DEVICE_FAIL;
	}
*/
//open	err = samsung_bml_erase(get_cal_param_start_unit(), 1);
	if(err) {
		printk("PARAMERTER BML ERASE FAIL!\n");		
		kfree(buf);
		if(cal_result)	sprintf(buffer,"%d\n", 1);
		return err;
	}

	// write back to chagned cal result
//open	err = samsung_bml_write(get_cal_param_start_unit() * NAND_PAGE_PER_UNIT, 1, mBuf, NULL);
	if(err) {
		printk("PARAMERTER BML WRITE FAIL!\n");		
		kfree(buf);
		if(cal_result)	sprintf(buffer,"%d\n", 1);
		return err;
	}

/*open	stChangePA.nNewAttr = FSR_BML_PI_ATTR_RO;
	if (FSR_BML_IOCtl(0, FSR_BML_IOCTL_CHANGE_PART_ATTR , (UINT8 *) &stChangePA, sizeof(stChangePA), NULL, 0, &nByteRet) != FSR_BML_SUCCESS) {
		kfree(buf);
		if(cal_result)	sprintf(buffer,"%d\n", 1);
		return FS_DEVICE_FAIL;
	}
*/	
	kfree(buf);
	if(!cal_result) 
	{
		sprintf(buffer,"%d\n", 0);
	}
	else 
	{
		sprintf(buffer, "%s\n", bak_cal_result.acc_offset);
	}
	return size;
}

static DEVICE_ATTR(cal_result, 0777, cal_result_param_read, cal_result_param_write);
static DEVICE_ATTR(cal_offset, 0777, cal_offset_param_read, cal_offset_param_write);

static int __init cal_param_init(void)
{	
	if (device_create_file(kr3dm_dev_t, &dev_attr_cal_result) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_cal_result.attr.name);	
		
	if (device_create_file(kr3dm_dev_t, &dev_attr_cal_offset) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_cal_offset.attr.name);	
	
	return 0;
}

static void __exit cal_param_exit(void)
{
//open 	remove_proc_entry("efs_info", fsr_proc_dir);
}

module_init(cal_param_init);
module_exit(cal_param_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Samsung Param Operation");
