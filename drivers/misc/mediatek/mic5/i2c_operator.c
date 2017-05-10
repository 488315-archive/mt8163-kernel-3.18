#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/module.h>
//#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#define GP_CLASS_NAME  "i2c_operator"
#define GP_CHR_DEV_NAME "i2c_operator"
#define GP_I2C_DEV_NAME "i2c_operator_device"


static struct class * p_gp_class;
static struct device * p_class_i2c;
static int major_num;
static unsigned char * i2c_msg;


int send_to_device(char * data, int count);

int receive_from_device(char * data, int count);
int cx20810_set_mode(int mode, int index);
//int led_array_set_mode(unsigned char * param, int length);
int led_array_set_enable(int enable);

enum
{
	I2C_DEVICE_FPGA = 0,
	I2C_DEVICE_CX20810,
	I2C_DEVICE_CX20810_0,
	I2C_DEVICE_CX20810_1,
	I2C_DEVICE_CX20810_2,
	I2C_DEVICE_LED_ARRAY,
};

// add by Timothy
// time:2015-02-09
// start==========
// i2c设备write处理
static ssize_t gp_i2c_write_dispatch(int len)
{
//	printk("Timothy:i2c_operator.c->gp_i2c_write_dispatch(), len = %d\n", len);
	int ret = 0;
	int i;
	int device_index = (int)i2c_msg[0];
	int length = len-sizeof(char);
	printk("Timothy:device_index = %d\n", device_index);

	//all 3 cx20810
	if(I2C_DEVICE_CX20810 == device_index)
	{
//		printk("Timothy:operate device:cx20810\n");
		for(i = 0; i < 3; i++)
		{
			cx20810_set_mode((int)i2c_msg[1], i);
		}
		return 0;
	}

	else if(I2C_DEVICE_CX20810_0 == device_index)
	{
//		printk("Timothy:operate device:cx20810_0\n");
		return cx20810_set_mode((int)i2c_msg[1], 0);
	}
	else if(I2C_DEVICE_CX20810_1 == device_index)
	{
//		printk("Timothy:operate device:cx20810_1\n");
		return cx20810_set_mode((int)i2c_msg[1], 1);
	}
	else if(I2C_DEVICE_CX20810_2 == device_index)
	{
//		printk("Timothy:operate device:cx20810_2\n");
		return cx20810_set_mode((int)i2c_msg[1], 2);
	}

	// LED
//	else if(I2C_DEVICE_LED_ARRAY == device_index)
//	{
//		printk("Timothy:operate device:led_array\n");
//		return led_array_set_mode(&i2c_msg[1], length);
//	}

	// FPGA
	else if(I2C_DEVICE_FPGA == device_index)
	{
//		printk("Timothy:operate device:FPGA, data lendth is %d\n", length);
		ret = send_to_device((char *)&i2c_msg[1], length);
		return ret;
	}
	return -1;
}

static int gp_i2c_read_dispatch(int len)
{
//	printk("Timothy:i2c_operator.c->gp_i2c_read_dispatch()\n");
	int ret =  0;
	int device_index = (int)i2c_msg[0];
	int length = len-sizeof(char);
//	printk("Timothy:device_index = %d\n", device_index);
	if(I2C_DEVICE_FPGA == device_index) // FPGA
	{
//		printk("Timothy:operate device:FPGA, data lendth is %d\n", length);
		ret = receive_from_device((char *)&i2c_msg[1], length);
		return ret;
	}
	return -1;
}
// end==========
//extern void init_i2c_io(void);

static int gp_open(struct inode * pnode, struct file * pfile)
{
	
	//int major = MAJOR(pnode->i_rdev);
	int minor = MINOR(pnode->i_rdev);
	printk("Timothy:i2c_operator.c->gp_open()\n");

	//init_i2c_io();

	if(minor == 0)
	{
		pfile->private_data = (void*)p_class_i2c;
		printk("Timothy:gp_open_i2c\n");
	}
	else
	{
		pfile->private_data = (void*)NULL;
		printk("Timothy:gp_open:unknow device\n");
	}
	return 0;
}

static int gp_close(struct inode * pnode, struct file * pfile)
{
	pfile->private_data = (void*)NULL;
	printk("gp_close\n");
	return 0;
}

static ssize_t gp_read(struct file * pfile, char __user * puser, size_t len, loff_t * poff)
{
	int nread = 0;
	int ret;
//	printk("Timothy:i2c_operator.c->gp_read()\n");
	if(pfile->private_data == p_class_i2c)
	{
//		printk("Timothy:gp_read():i2c device\n");
		i2c_msg = (unsigned char*)kzalloc(len * sizeof(unsigned char), GFP_KERNEL);
		ret=copy_from_user((void*)i2c_msg, puser, len);
		nread = gp_i2c_read_dispatch(len);
//		printk("Timothy:the data will send to user is:%d\n", i2c_msg[1]);
		ret=copy_to_user(puser, (void*)i2c_msg, len);
		kfree(i2c_msg);
	}
//	printk("Timothy:return value is %d\n", nread);
	return nread;
}

static ssize_t gp_write(struct file * pfile, const char __user * puser, size_t len, loff_t * poff)
{
//	printk("Timothy:i2c_operator.c->gp_write()\n");
	int ret = 0;
	int i;
	if(pfile->private_data == p_class_i2c)
	{
//		printk("Timothy:gp_write():i2c device\n");
		i2c_msg = (unsigned char *)kzalloc(len*sizeof(unsigned char), GFP_KERNEL);
		ret=copy_from_user((void*)i2c_msg, puser, len);

		printk("Timothy:the data is ");
		for(i = 0; i < len; i++)
		{
			printk("%x ", i2c_msg[i]);
		}
		printk("\n");
		ret = gp_i2c_write_dispatch(len);
		kfree(i2c_msg);
	}
	else
	{
//		printk("Timothy:gp_write_null\n");
	}
//	printk("Timothy:return value is %d\n", ret);
	return ret;
}


static struct file_operations gp_fops =
{
	.owner = THIS_MODULE,
	.open = gp_open,
	.release = gp_close,
	.read = gp_read,
	.write = gp_write,
};

static int __init i2c_operator_init(void)
{
	major_num = register_chrdev(0, GP_CHR_DEV_NAME, &gp_fops);
	if (major_num < 0)
	{
		printk("Timothy:gen_prov_reg_chr_err\n");
		return 1;
	}
	p_gp_class = class_create(THIS_MODULE, GP_CLASS_NAME);
	if(p_gp_class == NULL)
	{
		printk("Timothy:gen_prov:class create err\n");
	}

	p_class_i2c = device_create(p_gp_class, NULL, MKDEV(major_num, 0), NULL, GP_I2C_DEV_NAME);
	if(p_class_i2c == NULL)
	{
		printk("Timothy:gen_prov:device_create err:i2c\n");
		return 1;
	}
	  printk("Timothy:i2c_operator_init\n");
	return 1;
}

static void __exit i2c_operator_exit(void)
{
}

late_initcall(i2c_operator_init);
module_exit(i2c_operator_exit);

MODULE_AUTHOR("Timothy");
MODULE_DESCRIPTION("i2c operator driver");
MODULE_LICENSE("GPL");
