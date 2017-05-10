#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>   /*class_create*/ 
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>

#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#endif

#define DEV_NAME   "motor_updt_ctl"
static struct cdev *motor_cdev;
static dev_t m_dev;
static struct class *motor_class = NULL;
struct device *motor_device = NULL;
 static unsigned int motor_power,motor_rst,motor_boot;
 static int motor_release (struct inode *node, struct file *file)
 {
	 printk("motor_release !\n");
	 return 0;

	 return 0;
 }
 static int motor_open (struct inode *inode, struct file *file)
 {
	 //MOTOR_INIT();
	 //POWER_EN();
	 printk("motor_open !\n");
	 return 0;
 }
 void motor_gpio_ctl(unsigned int pin,int val) 
 {
	 gpio_direction_output(pin, val);
	 gpio_set_value(pin, val);	 
 }
 static ssize_t motor_read(struct file * pfile, char __user * puser, size_t len, loff_t * poff)
 {
	return 0;
 }

static ssize_t motor_write(struct file *pfile, const char __user *from, size_t len, loff_t * offset)
{
	int  ret = 0;
	char data[20] = {0};
	char dev,sta;
	ret=copy_from_user(data, from, len);
	
	dev = *(data+0);
	 sta = *(data+1);
	//printk("daviekuo %s %s", __func__, data);
	if (len != 0) {
		if(dev == 'A' || dev == 'a')
		{
			switch(sta)
			{
				case '1':
					
					break;
				case '0':
					
					break;
							
			}
		}
		else if(dev == 'B' || dev == 'b')
		{
			switch(sta)
			{
				case '1':
					/*
					mt_set_gpio_out(MOTOR_BOOT0_PIN, 1);
					mt_set_gpio_out(MOTOR_RST_PIN, 1);
					mdelay(10);					
					mt_set_gpio_out(MOTOR_POWEN_PIN, 0);
					mdelay(10);
					mt_set_gpio_out(MOTOR_POWEN_PIN, 1);
					mdelay(100);
					*/
					motor_gpio_ctl(motor_boot,1);
					motor_gpio_ctl(motor_rst,1);
					mdelay(10);
					motor_gpio_ctl(motor_power,0);
					mdelay(10);
					motor_gpio_ctl(motor_power,1);
					printk("daviekuo pull up STM32 to update !\n");
					break;
				case '0':
					/*
					mt_set_gpio_out(MOTOR_RST_PIN, 1);
					mt_set_gpio_out(MOTOR_BOOT0_PIN, 0);
					mdelay(10);
					mt_set_gpio_out(MOTOR_POWEN_PIN, 0);
					mdelay(10);
					mt_set_gpio_out(MOTOR_POWEN_PIN, 1);
					mdelay(10);
					*/
					motor_gpio_ctl(motor_boot,0);
					motor_gpio_ctl(motor_rst,1);
					mdelay(10);
					motor_gpio_ctl(motor_power,0);
					mdelay(10);
					motor_gpio_ctl(motor_power,1);
					mdelay(10);
					printk("daviekuo pull down STM32 to out update !\n");
					break;
			}
		}
		else if(dev == 'C' || dev == 'c')
		{
			switch(sta)
			{
				case '1':
					
					break;
				case '0':
					
					break;
				default:
					ret = -1;
					break;
							
			}
		}
		else
			ret = -1;
	}
	else
		ret = -2;

	return ret;
}
static ssize_t onoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(onoff, S_IWUSR | S_IWGRP | S_IRUGO, onoff_show, NULL);

static int motor_probe(struct platform_device *dev)
{
	struct device_node *node;
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,motorctl");

	motor_power =   of_get_named_gpio(node, "mcu_power_gpio44", 0);
	motor_rst =         of_get_named_gpio(node, "mcu_rst_gpio40", 0);
	motor_boot =      of_get_named_gpio(node, "mcu_boot_gpio39", 0);
	
	 gpio_request(motor_power, "motor_power");	
	gpio_request(motor_rst, "motor_rst");
	gpio_request(motor_boot, "motor_boot");

	// ---------init-------
	motor_gpio_ctl(motor_boot,0);
	motor_gpio_ctl(motor_rst,1);	
	motor_gpio_ctl(motor_power,1);

	printk("motor_boot---------\n");
	return 0;
}

static struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.open = motor_open,
	.write = motor_write,
	.read = motor_read,
	.release = motor_release,
};

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static const struct of_device_id motor_of_match[] = {
	{.compatible = "mediatek,motorctl"},
	{},
};
#endif

static struct platform_driver motor_platform_driver = {
	.probe = motor_probe,
	
	.driver = {
		   .name = "motorctl",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = motor_of_match,
#endif
	},
};

static int __init motor_init(void)
{
	int ret;
	printk("daviekuo motor  init  finish--------\n");
#if 0
	mt_set_gpio_mode(MOTOR_RST_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(MOTOR_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(MOTOR_RST_PIN, 1);

	mt_set_gpio_mode(MOTOR_BOOT0_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(MOTOR_BOOT0_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(MOTOR_BOOT0_PIN, 0);
	
	mt_set_gpio_mode(MOTOR_POWEN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(MOTOR_POWEN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(MOTOR_POWEN_PIN, 1);
#endif
	ret=platform_driver_register(&motor_platform_driver);
  	ret = alloc_chrdev_region(&m_dev, 0, 1, DEV_NAME);
         if (ret< 0) {
		 printk("motor  alloc_chrdev_region failed, %d", ret);
		return ret;
	}
	 motor_cdev= cdev_alloc();
	 if (motor_cdev == NULL) {
			 printk("motor cdev_alloc failed");
			 ret = -ENOMEM;
			 goto EXIT;
		 }
 	cdev_init(motor_cdev, &motor_fops);
	 motor_cdev->owner = THIS_MODULE;
	 ret = cdev_add(motor_cdev, m_dev, 1);
	 if (ret < 0) {
		  printk("Attatch file motor operation failed, %d", ret);
		 goto EXIT;
	 }
		 	 
	 motor_class = class_create(THIS_MODULE, DEV_NAME);
			 if (IS_ERR(motor_class)) {
				 printk("Failed to create class(motor)!\n");
				 return PTR_ERR(motor_class);
			 }
			 
	 motor_device = device_create(motor_class, NULL, m_dev, NULL,DEV_NAME);
	 if (IS_ERR(motor_device))
		 printk("Failed to create motor_dev device\n");
	
	if (device_create_file(motor_device, &dev_attr_onoff) < 0)
					printk("Failed to create device file(%s)!\n",
						  dev_attr_onoff.attr.name);  

	printk("daviekuo motor  init  finish--------\n");

	return 0;

EXIT:
	if(motor_cdev != NULL)
    {
        cdev_del(motor_cdev);
        motor_cdev = NULL;
    }
	unregister_chrdev_region(m_dev, 1);
		 
	return ret;

}
/*----------------------------------------------------------------------------*/

static void __exit motor_exit(void)
{

	if(motor_cdev != NULL)
	{
	    cdev_del(motor_cdev);
	    motor_cdev = NULL;
	}
    unregister_chrdev_region(m_dev, 1);

	return;

}

late_initcall(motor_init);
module_exit(motor_exit);
MODULE_AUTHOR("Yongyida");
MODULE_DESCRIPTION("motor driver");
MODULE_LICENSE("GPL");

