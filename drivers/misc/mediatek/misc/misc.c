#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include<linux/kthread.h>
#include <asm/uaccess.h>
#include <asm/uaccess.h>	/* copy_*_user */
#include <linux/semaphore.h>  
#include <linux/device.h>   /*class_create*/ 
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/fs.h>

/*----------------------------------------------------------------------------*/
#define DEV_NAME   "misc_yyd"
static struct cdev *misc_cdev;
static dev_t misc_dev;
static struct class *misc_class = NULL;
static struct device *misc_device = NULL;
extern  void aw2013_breath_all(int led0,int led1,int led2);
//extern  int reset_mic_cx20810(void);
int danceflag=false;
//static struct wake_lock m_lock;
bool current_mode_is_fatory=false;

//extern int cw2015_read_version(void);
//extern int cw2015_read_all_reg(char *buf);

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_EARLYSUSPEND
  
  static void m_suspend( struct early_suspend *h )
  {

	printk("m_suspend----------\n");
  }
  
  static void m_resume( struct early_suspend *h )
  {
  
	 printk("m_suspend++++++++++\n");
  }
  
  static struct early_suspend misc_early_suspend_handler = {
	  .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1,
	  .suspend = m_suspend,
	  .resume = m_resume,
  };
  
#endif
#endif

 static ssize_t yyd_misc_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
 {
#if 0
	char pbuf[11];
	if(buf == NULL)return 0;

	if(buf[0] == 'A')
	{
		cw2015_read_all_reg(pbuf);
	}
#endif	
	return 1;
}
 static ssize_t yyd_misc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
	int ret;	
	char data[2] = {0};
	//ret=cw2015_read_version();
	if(ret==0x6f)
	data[0]='1';
	else 
	data[0]='0';
	
	data[1]='\0';
	return sprintf(buf, "%s\n", data);	
#endif	
return 0;

}
 static DEVICE_ATTR(yyd_misc, S_IWUSR | S_IWGRP | S_IRUGO, yyd_misc_show, yyd_misc_store);

 static int misc_release (struct inode *node, struct file *file)
 {
	 printk("misc_release !\n");
	 return 0;

	
 }
 static int misc_open (struct inode *inode, struct file *file)
 {
	 return nonseekable_open(inode, file);

 }

static ssize_t misc_write(struct file *pfile, const char __user *buf, size_t len, loff_t * offset)
{
	 char pbuf[50] ;
	
	   if(copy_from_user(pbuf, buf,len))
	   {
		   return	 -EFAULT;  
	   }
	    printk("111111%c,%d\n",pbuf[0],pbuf[1]);
	  if(pbuf[0] == 'A')
	  {
		if(pbuf[1] == '0'){current_mode_is_fatory=false;}
		else if(pbuf[1] == '1'){current_mode_is_fatory=true;}			
	  }
	 	
	return len;
}

static ssize_t misc_read(struct file *pfile, char __user *to, size_t len, loff_t *offset)
{
	//char buf,ret;
	//printk("misc_read !\n");
	
	//buf= cw2015_read_version();
	//ret=copy_to_user(to, &buf,  1);
	return 0;
}


static long misc_unlocked_ioctl (struct file *pfile, unsigned int cmd, unsigned long param)
{
			
	 return 0;
}

static const struct file_operations misc_fops = {
	//.owner = THIS_MODULE,
	.open = misc_open,
	.write = misc_write,
	.read = misc_read,
	.release = misc_release,
	.unlocked_ioctl = misc_unlocked_ioctl,
};

static int __init misc_init(void)
{
	int ret = 0;
	printk("misc setting  init  finish--------\n");

	#ifdef CONFIG_HAS_EARLYSUSPEND
	#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&misc_early_suspend_handler);
	#endif
	#endif
	
	ret = alloc_chrdev_region(&misc_dev, 0, 1, DEV_NAME);
			 if (ret< 0) {
			 printk("misc alloc_chrdev_region failed, %d", ret);
			return ret;
		}
		 misc_cdev= cdev_alloc();
		 if (misc_cdev == NULL) {
				 printk("misc cdev_alloc failed");
				 ret = -ENOMEM;
				 goto EXIT;
			 }
		cdev_init(misc_cdev, &misc_fops);
		 misc_cdev->owner = THIS_MODULE;
		 ret = cdev_add(misc_cdev, misc_dev, 1);
		 if (ret < 0) {
			  printk("Attatch file misc operation failed, %d", ret);
			 goto EXIT;
		 }
				 
		 misc_class = class_create(THIS_MODULE, DEV_NAME);
				 if (IS_ERR(misc_class)) {
					 printk("Failed to create class(misc)!\n");
					 return PTR_ERR(misc_class);
				 }
				 
		 misc_device = device_create(misc_class, NULL, misc_dev, NULL,DEV_NAME);
		 if (IS_ERR(misc_device))
			 printk("Failed to create misc_dev device\n");

		 if (device_create_file(misc_device, &dev_attr_yyd_misc) < 0)
				printk("Failed to create device file(%s)!\n",
					  dev_attr_yyd_misc.attr.name);   
		printk("daviekuo misc	init  finish--------\n");
	
		return 0;
		
	EXIT:
		if(misc_cdev != NULL)
		{
			cdev_del(misc_cdev);
			misc_cdev = NULL;
		}
		unregister_chrdev_region(misc_dev, 1);
	
		printk("misc device  init  failed--------\n");			 
	
		return ret;
}
/*----------------------------------------------------------------------------*/

static void __exit misc_exit(void)
{

}

module_init(misc_init);
module_exit(misc_exit);
MODULE_AUTHOR("Yongyida");
MODULE_DESCRIPTION("misc driver");
MODULE_LICENSE("GPL");

